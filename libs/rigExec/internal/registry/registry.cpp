#include "rigExec/api/registry.h"

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace rigExec {

namespace {

// Published once with release; read with acquire. Never destroyed.
std::atomic<const RigExecRegistry *> gRegistry{nullptr};
std::atomic<bool> gInitializing{false};

uint64_t
Mix(uint64_t h, uint64_t v)
{
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h;
}

uint64_t
MixText(uint64_t h, const char *text)
{
    uint64_t v = 1469598103934665603ull;
    for (const char *c = text ? text : ""; *c; ++c) {
        v = (v ^ uint64_t(uint8_t(*c))) * 1099511628211ull;
    }
    return Mix(h, v);
}

std::string
Text(const char *text)
{
    return text ? std::string(text) : std::string();
}

std::string
QualifiedName(const char *nameSpace, const char *name)
{
    std::string ns = Text(nameSpace);
    return ns.empty() ? Text(name) : ns + ":" + Text(name);
}

} // namespace

/// Runs the freeze over local tables; Publish writes handles and the
/// singleton.
class RigExecRegistryBuilder {
public:
    bool Build(const RigExecRegistryInit &init,
               std::vector<std::string> *errors);
    void Publish();

private:
    struct _TypeEntry {
        const RigExecTypeHandle *handle;
        const RigExecTypeDesc *desc;
        const RigExecTypeLibraryDesc *library;
    };
    struct _OpEntry {
        const RigExecOpHandle *handle;
        const RigExecOpDesc *desc;
        const RigExecOpLibraryDesc *library;
    };
    struct _TypeCollector : RigExecTypeRegistrar {
        const RigExecTypeLibraryDesc *library = nullptr;
        std::vector<_TypeEntry> *out = nullptr;
        void Add(const RigExecTypeHandle &handle,
                 const RigExecTypeDesc &desc) override
        {
            out->push_back({&handle, &desc, library});
        }
    };
    struct _OpCollector : RigExecOpRegistrar {
        const RigExecOpLibraryDesc *library = nullptr;
        std::vector<_OpEntry> *out = nullptr;
        void Add(const RigExecOpHandle &handle,
                 const RigExecOpDesc &desc) override
        {
            out->push_back({&handle, &desc, library});
        }
    };

    bool _FreezeTypes(const RigExecRegistryInit &init);
    bool _FreezeOps(const RigExecRegistryInit &init);
    void _Error(std::string message)
    {
        _errors.push_back(std::move(message));
    }

    std::vector<_TypeEntry> _typeEntries;
    std::vector<_OpEntry> _opEntries;
    std::unordered_map<const RigExecTypeHandle *, uint32_t> _typeIndex;
    std::unordered_map<const RigExecOpHandle *, uint32_t> _opIndex;
    std::vector<std::pair<const RigExecPortHandle *, uint32_t>> _portIndex;
    std::unique_ptr<RigExecRegistry> _registry;
    std::vector<std::string> _errors;
};

bool
RigExecRegistryBuilder::_FreezeTypes(const RigExecRegistryInit &init)
{
    std::set<const RigExecTypeLibraryDesc *> seen;
    for (const RigExecTypeLibraryDesc *library : init.typeLibraries) {
        if (!library) {
            _Error("null type library");
            continue;
        }
        const std::string name = Text(library->name);
        if (library->structSize < sizeof(RigExecTypeLibraryDesc) ||
            library->abiVersion != kRigExecRegistryAbiVersion) {
            _Error("type library " + name + ": ABI version " +
                   std::to_string(library->abiVersion) + ", expected " +
                   std::to_string(kRigExecRegistryAbiVersion));
            continue;
        }
        if (!seen.insert(library).second) {
            _Error("type library " + name + " listed twice");
            continue;
        }
        for (size_t i = 0; i < library->requiredCount; ++i) {
            if (!seen.count(library->required[i])) {
                _Error("type library " + name + " requires " +
                       Text(library->required[i] ? library->required[i]->name
                                                 : "<null>") +
                       ", which is not registered before it");
            }
        }
        if (!library->registerTypes) {
            continue;
        }
        _TypeCollector collector;
        collector.library = library;
        collector.out = &_typeEntries;
        library->registerTypes(collector);
    }

    std::map<std::tuple<std::string, std::string, uint32_t>, std::string> ids;
    for (const _TypeEntry &entry : _typeEntries) {
        const RigExecTypeDesc &d = *entry.desc;
        const std::string name = QualifiedName(d.nameSpace, d.name);
        if (d.structSize < sizeof(RigExecTypeDesc)) {
            _Error("type " + name + ": descriptor from an older ABI");
            continue;
        }
        if (!_typeIndex.emplace(entry.handle, uint32_t(_typeIndex.size()))
                 .second) {
            _Error("type " + name + ": handle registered twice");
            continue;
        }
        if (!ids.emplace(
                    std::make_tuple(Text(d.nameSpace), Text(d.name), d.major),
                    Text(entry.library->name))
                 .second) {
            _Error("type " + name + " v" + std::to_string(d.major) +
                   ": registered by two libraries");
        }
        if (d.size == 0 || d.align == 0 || (d.align & (d.align - 1)) != 0 ||
            d.size % d.align != 0) {
            _Error("type " + name + ": invalid size or alignment");
        }
        if (!d.triviallyCopyable && (!d.copy || !d.destroy)) {
            _Error("type " + name + ": needs copy and destroy");
        }
        if (d.hashClass != RigExecHashClass::Unhashable && !d.hash) {
            _Error("type " + name + ": hash class without a hash");
        }
    }
    // Composition handles must name registered types; a registered type
    // may compose only types registered before it, so there is no cycle.
    for (size_t i = 0; i < _typeEntries.size(); ++i) {
        const RigExecTypeDesc &d = *_typeEntries[i].desc;
        const std::string name = QualifiedName(d.nameSpace, d.name);
        const auto check = [&](const RigExecTypeHandle *part,
                               const char *what) {
            const auto found = _typeIndex.find(part);
            if (found == _typeIndex.end()) {
                _Error("type " + name + ": " + what +
                       " type is not registered");
            } else if (found->second >= i) {
                _Error("type " + name + ": " + what +
                       " type is registered after it");
            }
        };
        if (d.element) {
            check(d.element, "element");
        }
        for (size_t f = 0; f < d.fieldCount; ++f) {
            check(d.fields ? d.fields[f] : nullptr, "field");
        }
    }
    return _errors.empty();
}

bool
RigExecRegistryBuilder::_FreezeOps(const RigExecRegistryInit &init)
{
    std::set<const RigExecOpLibraryDesc *> seen;
    for (const RigExecOpLibraryDesc *library : init.opLibraries) {
        if (!library) {
            _Error("null op library");
            continue;
        }
        const std::string name = Text(library->name);
        if (library->structSize < sizeof(RigExecOpLibraryDesc) ||
            library->abiVersion != kRigExecRegistryAbiVersion) {
            _Error("op library " + name + ": ABI version " +
                   std::to_string(library->abiVersion) + ", expected " +
                   std::to_string(kRigExecRegistryAbiVersion));
            continue;
        }
        if (!seen.insert(library).second) {
            _Error("op library " + name + " listed twice");
            continue;
        }
        if (!library->registerOps) {
            continue;
        }
        _OpCollector collector;
        collector.library = library;
        collector.out = &_opEntries;
        library->registerOps(collector);
    }

    std::map<std::pair<std::string, std::string>, std::string> ids;
    for (const _OpEntry &entry : _opEntries) {
        const RigExecOpDesc &d = *entry.desc;
        const std::string name = QualifiedName(d.nameSpace, d.name);
        if (d.structSize < sizeof(RigExecOpDesc)) {
            _Error("op " + name + ": descriptor from an older ABI");
            continue;
        }
        if (!_opIndex.emplace(entry.handle, uint32_t(_opIndex.size())).second) {
            _Error("op " + name + ": handle registered twice");
            continue;
        }
        if (!ids.emplace(std::make_pair(Text(d.nameSpace), Text(d.name)),
                         Text(entry.library->name))
                 .second) {
            _Error("op " + name + ": registered by two libraries");
        }
        if (!d.run) {
            _Error("op " + name + ": no run function");
        }
        std::set<const RigExecPortHandle *> ports;
        for (size_t p = 0; p < d.portCount; ++p) {
            const RigExecPortDesc &port = d.ports[p];
            if (!port.port || !ports.insert(port.port).second) {
                _Error("op " + name + ": port " + Text(port.name) +
                       " has no handle or a repeated one");
            }
            if (!(port.flags & (kRigExecPortInput | kRigExecPortOutput))) {
                _Error("op " + name + ": port " + Text(port.name) +
                       " is neither input nor output");
            }
            if (!_typeIndex.count(port.type)) {
                _Error("op " + name + ": port " + Text(port.name) +
                       " uses a type that is not registered");
            }
        }
        if (d.payload.size && (!d.payload.copy || !d.payload.destroy)) {
            _Error("op " + name + ": payload needs copy and destroy");
        }
        if (d.state.size && (!d.state.create || !d.state.destroy)) {
            _Error("op " + name + ": state needs create and destroy");
        }
    }
    if (!_errors.empty()) {
        return false;
    }

    _registry =
        std::make_unique<RigExecRegistry>(RigExecRegistry::_BuilderKey(0));
    RigExecRegistry &r = *_registry;
    uint64_t fingerprint = 0;
    r._types.resize(_typeEntries.size());
    for (size_t i = 0; i < _typeEntries.size(); ++i) {
        const _TypeEntry &e = _typeEntries[i];
        r._types[i] = {e.desc, e.library, uint32_t(i)};
        fingerprint = MixText(fingerprint, e.desc->nameSpace);
        fingerprint = MixText(fingerprint, e.desc->name);
        fingerprint = Mix(fingerprint, e.desc->major);
        fingerprint = Mix(fingerprint, e.desc->minor);
        fingerprint = Mix(fingerprint, e.desc->layoutHash);
    }
    r._ops.resize(_opEntries.size());
    for (size_t i = 0; i < _opEntries.size(); ++i) {
        const _OpEntry &e = _opEntries[i];
        RigExecRegisteredOp &op = r._ops[i];
        op.desc = e.desc;
        op.library = e.library;
        op.index = uint32_t(i);
        for (size_t p = 0; p < e.desc->portCount; ++p) {
            op.portTypes.push_back(_typeIndex.at(e.desc->ports[p].type));
            _portIndex.emplace_back(e.desc->ports[p].port, uint32_t(p));
        }
        fingerprint = MixText(fingerprint, e.desc->nameSpace);
        fingerprint = MixText(fingerprint, e.desc->name);
        fingerprint = Mix(fingerprint, e.desc->version);
        for (uint32_t t : op.portTypes) {
            fingerprint = Mix(fingerprint, t);
        }
    }
    r._fingerprint = fingerprint;
    return true;
}

bool
RigExecRegistryBuilder::Build(const RigExecRegistryInit &init,
                              std::vector<std::string> *errors)
{
    const bool ok = _FreezeTypes(init) && _FreezeOps(init);
    if (errors) {
        errors->insert(errors->end(), _errors.begin(), _errors.end());
    }
    return ok;
}

void
RigExecRegistryBuilder::Publish()
{
    for (const auto &[handle, index] : _typeIndex) {
        handle->_index = index;
    }
    for (const auto &[handle, index] : _opIndex) {
        handle->_index = index;
    }
    for (const auto &[handle, index] : _portIndex) {
        handle->_index = index;
    }
    // Immortal: handles and descriptors outlive static destruction.
    gRegistry.store(_registry.release(), std::memory_order_release);
}

bool
RigExecRegistry::Validate(const RigExecRegistryInit &init,
                          std::vector<std::string> *errors)
{
    RigExecRegistryBuilder builder;
    return builder.Build(init, errors);
}

bool
RigExecRegistry::Initialize(const RigExecRegistryInit &init,
                            std::vector<std::string> *errors)
{
    bool expected = false;
    if (!gInitializing.compare_exchange_strong(expected, true)) {
        if (errors) {
            errors->push_back("RigExecRegistry::Initialize called twice");
        }
        return false;
    }
    RigExecRegistryBuilder builder;
    if (!builder.Build(init, errors)) {
        gInitializing.store(false);
        return false;
    }
    builder.Publish();
    return true;
}

bool
RigExecRegistry::IsInitialized()
{
    return gRegistry.load(std::memory_order_acquire) != nullptr;
}

const RigExecRegistry &
RigExecRegistry::Get()
{
    const RigExecRegistry *registry = gRegistry.load(std::memory_order_acquire);
    if (!registry) {
        std::abort();
    }
    return *registry;
}

} // namespace rigExec
