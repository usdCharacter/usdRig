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

/// Appends \p part's columns and runs to \p type's; \p part's own values
/// lie in \p type's run \p outerRun.
void
AppendColumns(RigExecRegisteredType &type, const RigExecRegisteredType &part,
              uint32_t outerRun)
{
    const uint32_t base = uint32_t(type.runParents.size()) - 1;
    for (const RigExecTypeColumn &c : part.columns)
        type.columns.push_back({c.kind, c.run ? base + c.run : outerRun});
    for (size_t r = 1; r < part.runParents.size(); ++r) {
        const uint32_t parent = part.runParents[r];
        type.runParents.push_back(parent ? base + parent : outerRun);
    }
}

/// Fills \p type's columns from its descriptor and its parts' columns.
template <class Index>
void
DeriveColumns(RigExecRegisteredType &type,
              const std::vector<RigExecRegisteredType> &types,
              const Index &index)
{
    const RigExecTypeDesc &d = *type.desc;
    type.runParents.assign(1, kRigExecInvalidIndex);
    bool columnar = true;
    if (d.element) {
        const RigExecRegisteredType &element = types[index.at(d.element)];
        type.runParents.push_back(0);
        columnar = !element.columns.empty();
        if (columnar)
            AppendColumns(type, element, 1);
    } else if (d.fieldCount) {
        for (size_t f = 0; columnar && f < d.fieldCount; ++f) {
            const RigExecRegisteredType &part = types[index.at(d.fields[f])];
            columnar = !part.columns.empty();
            if (columnar)
                AppendColumns(type, part, 0);
        }
    } else {
        columnar = d.columnKind != RigExecColumnKind::None;
        if (columnar)
            type.columns.assign(d.columnCount, {d.columnKind, 0});
    }
    if (!columnar) {
        type.columns.clear();
        type.runParents.assign(1, kRigExecInvalidIndex);
    }
}

std::string
Text(const char *text)
{
    return text ? std::string(text) : std::string();
}

bool
LittleEndian()
{
    const uint16_t one = 1;
    uint8_t first = 0;
    std::memcpy(&first, &one, 1);
    return first == 1;
}

bool
IntegerSize(size_t size)
{
    return size == 1 || size == 2 || size == 4 || size == 8;
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
    void _CheckBound(const std::string &op, const RigExecOpDesc &d,
                     const RigExecPortDesc &port);
    void _CheckRanges(const std::string &op, const RigExecOpDesc &d);
    void _CheckFields(const std::string &op, const RigExecOpDesc &d,
                      bool payload);
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
        if (!d.reserve != !d.elements) {
            _Error("type " + name + ": reserve and elements go together");
        }
        if (d.resize && (!d.count || !d.reserve)) {
            _Error("type " + name + ": resize needs count and reserve");
        }
        if (d.columnKind == RigExecColumnKind::None
                ? d.columnCount != 0
                : d.element || d.fieldCount ||
                      d.columnCount * RigExecColumnKindSize(d.columnKind) !=
                          d.size) {
            _Error("type " + name + ": columns do not make up its values");
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
        if (!d.run && !d.runBatch && !d.range.run) {
            _Error("op " + name + ": no run or batch function");
        }
        _CheckRanges(name, d);
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
            if ((port.flags & kRigExecPortArray) &&
                (port.flags & kRigExecPortOutput)) {
                _Error("op " + name + ": port " + Text(port.name) +
                       " is an array output; only inputs can be arrays");
            }
            if (!_typeIndex.count(port.type)) {
                _Error("op " + name + ": port " + Text(port.name) +
                       " uses a type that is not registered");
            } else {
                _CheckBound(name, d, port);
            }
        }
        if (d.payload.size && (!d.payload.copy || !d.payload.destroy)) {
            _Error("op " + name + ": payload needs copy and destroy");
        }
        if ((d.capabilities & kRigExecOpBakeable) && d.payload.size &&
            (!d.payload.encode || !d.payload.decode)) {
            _Error("op " + name +
                   ": a Bakeable op's payload needs encode and decode");
        }
        if (d.state.size && (!d.state.create || !d.state.destroy)) {
            _Error("op " + name + ": state needs create and destroy");
        }
        if ((d.threading == RigExecOpThreading::Windowed) !=
            (d.windowFrames != nullptr)) {
            _Error("op " + name +
                   ": Windowed threading and windowFrames go together");
        }
        _CheckFields(name, d, true);
        _CheckFields(name, d, false);
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
        r._types[i] = {e.desc, e.library, uint32_t(i), e.handle};
        DeriveColumns(r._types[i], r._types, _typeIndex);
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
        op.handle = e.handle;
        if (e.desc->range.run)
            op.runBatch = nullptr;
        else
            op.runBatch =
                e.desc->runBatch ? e.desc->runBatch : &RigExecRunEachInstance;
        for (size_t f = 0; f < e.desc->payloadFieldCount; ++f)
            op.payloadFieldTypes.push_back(
                _typeIndex.at(e.desc->payloadFields[f].type));
        for (size_t f = 0; f < e.desc->stateFieldCount; ++f)
            op.stateFieldTypes.push_back(
                _typeIndex.at(e.desc->stateFields[f].type));
        for (size_t p = 0; p < e.desc->portCount; ++p) {
            op.portTypes.push_back(_typeIndex.at(e.desc->ports[p].type));
            _portIndex.emplace_back(e.desc->ports[p].port, uint32_t(p));
            if (e.desc->ports[p].bound != RigExecOutputBound::None)
                op.boundedOutputs.push_back(uint32_t(p));
            if (!e.desc->range.run)
                continue;
            const uint32_t flags = e.desc->ports[p].flags;
            if (flags & kRigExecPortOutput)
                op.rangeOutputs.push_back(uint32_t(p));
            else if (op.rangeCountPort == kRigExecInvalidIndex &&
                     (flags &
                      (kRigExecPortPointwise | kRigExecPortNeighbourhood)))
                op.rangeCountPort = uint32_t(p);
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

void
RigExecRegistryBuilder::_CheckBound(const std::string &op,
                                    const RigExecOpDesc &d,
                                    const RigExecPortDesc &port)
{
    const std::string where = "op " + op + ": port " + Text(port.name);
    const bool output = (port.flags & kRigExecPortOutput) != 0;
    const bool arrays =
        _typeEntries[_typeIndex.at(port.type)].desc->reserve != nullptr;
    if (port.bound == RigExecOutputBound::None) {
        if (output && arrays)
            _Error(where + " holds arrays and declares no bound");
        return;
    }
    if (!output || !arrays) {
        _Error(where + " declares a bound but is not an output that holds "
                       "arrays");
        return;
    }
    const auto isInput = [&](const RigExecPortHandle *handle, bool single) {
        for (size_t p = 0; p < d.portCount; ++p)
            if (d.ports[p].port == handle &&
                (d.ports[p].flags & kRigExecPortInput) &&
                !(single && (d.ports[p].flags & kRigExecPortArray)))
                return true;
        return false;
    };
    const bool counted = port.bound == RigExecOutputBound::Input ||
                         port.bound == RigExecOutputBound::InputValue;
    if (counted &&
        (!isInput(port.boundPort,
                  port.bound == RigExecOutputBound::InputValue) ||
         (port.boundAltPort && !isInput(port.boundAltPort, false)))) {
        _Error(where + " takes its bound from a port that is not one of its "
                       "op's inputs");
    }
    const bool singleInput = isInput(port.boundPort, true);
    if (port.bound == RigExecOutputBound::InputValue) {
        const RigExecTypeDesc *type = nullptr;
        for (size_t p = 0; p < d.portCount; ++p)
            if (d.ports[p].port == port.boundPort &&
                _typeIndex.count(d.ports[p].type))
                type = _typeEntries[_typeIndex.at(d.ports[p].type)].desc;
        const bool integer = type && type->columnCount == 1 &&
                             (type->columnKind == RigExecColumnKind::UInt8 ||
                              type->columnKind == RigExecColumnKind::Int32 ||
                              type->columnKind == RigExecColumnKind::UInt32 ||
                              type->columnKind == RigExecColumnKind::UInt64);
        if (!singleInput || !integer)
            _Error(where + " takes its bound from a port that is not one of "
                           "its op's one-value integer inputs");
    }
    if (port.bound == RigExecOutputBound::Field &&
        (port.boundField >= d.payloadFieldCount ||
         d.payloadFields[port.boundField].extent !=
             RigExecFieldExtent::Scalar)) {
        _Error(where + " takes its bound from a field that is not a one-value "
                       "payload field");
    }
}

void
RigExecRegistryBuilder::_CheckRanges(const std::string &op,
                                     const RigExecOpDesc &d)
{
    const std::string where = "op " + op;
    const bool ranged = (d.capabilities & kRigExecOpRanged) != 0;
    constexpr uint32_t kRangeFlags =
        kRigExecPortPointwise | kRigExecPortNeighbourhood;
    if (ranged != (d.range.run != nullptr)) {
        _Error(where + ": the Ranged capability and range handling go "
                       "together");
        return;
    }
    if (!ranged) {
        for (size_t p = 0; p < d.portCount; ++p)
            if (d.ports[p].flags & kRangeFlags)
                _Error(where + ": port " + Text(d.ports[p].name) +
                       " is pointwise but the op does not run in ranges");
        return;
    }
    if (d.run || d.runBatch)
        _Error(where + ": a Ranged op runs through its range handling only");
    if (d.threading != RigExecOpThreading::Pure)
        _Error(where + ": a Ranged op must be Pure");
    if (d.range.scratch.size &&
        (!d.range.scratch.create || !d.range.scratch.destroy))
        _Error(where + ": range scratch needs create and destroy");
    bool counted = false;
    for (size_t p = 0; p < d.portCount; ++p) {
        const RigExecPortDesc &port = d.ports[p];
        const std::string at = where + ": port " + Text(port.name);
        const auto type = _typeIndex.find(port.type);
        const RigExecTypeDesc *t = type == _typeIndex.end()
                                       ? nullptr
                                       : _typeEntries[type->second].desc;
        if (port.flags & kRigExecPortOutput) {
            if (!(port.flags & kRigExecPortPointwise) ||
                (port.flags & kRigExecPortNeighbourhood) || !t || !t->resize)
                _Error(at + " is not a pointwise output of an array type");
            continue;
        }
        if (!(port.flags & kRangeFlags))
            continue;
        if ((port.flags & kRigExecPortArray) || !t || !t->count)
            _Error(at + " is pointwise but not one array value");
        if (!counted && (port.flags & kRigExecPortOptional))
            _Error(at + " gives the element count but is optional");
        counted = true;
    }
    if (!counted)
        _Error(where + ": a Ranged op needs a pointwise input");
}

void
RigExecRegistryBuilder::_CheckFields(const std::string &op,
                                     const RigExecOpDesc &d, bool payload)
{
    const RigExecFieldDesc *fields = payload ? d.payloadFields : d.stateFields;
    const size_t count = payload ? d.payloadFieldCount : d.stateFieldCount;
    if (!count)
        return;
    const std::string kind = payload ? "payload" : "state";
    if (!fields || (payload && d.payload.size) || (!payload && d.state.size))
        _Error("op " + op + ": declares " + kind +
               " fields and needs them in place of its " + kind + " handling");
    if (!d.runBatch && !d.range.run)
        _Error("op " + op + ": declares fields but has no batch function");
    if (!LittleEndian())
        _Error("op " + op + ": declared fields need a little-endian machine");
    if (!fields)
        return;
    const auto sizeOf = [&](const RigExecTypeHandle *type) -> size_t {
        const auto found = _typeIndex.find(type);
        return found == _typeIndex.end()
                   ? 0
                   : _typeEntries[found->second].desc->size;
    };
    std::set<std::string> names;
    for (size_t f = 0; f < count; ++f) {
        const RigExecFieldDesc &field = fields[f];
        const std::string name =
            "op " + op + ": " + kind + " field " + Text(field.name);
        if (!field.name || !*field.name || !names.insert(field.name).second)
            _Error(name + " has no name or a repeated one");
        const auto type = _typeIndex.find(field.type);
        if (type == _typeIndex.end()) {
            _Error(name + " uses a type that is not registered");
            continue;
        }
        const RigExecTypeDesc &t = *_typeEntries[type->second].desc;
        if (!t.triviallyCopyable || t.element || t.fieldCount || t.align > 64)
            _Error(name + " is not of a plain value type");
        if (field.limit != UINT64_MAX && (!payload || !IntegerSize(t.size)))
            _Error(name + " has a limit but is no payload integer");
        switch (field.extent) {
        case RigExecFieldExtent::Scalar:
            break;
        case RigExecFieldExtent::Fixed:
            if (!field.count)
                _Error(name + " has a fixed count of 0");
            break;
        case RigExecFieldExtent::FromField: {
            const size_t earlier = payload ? f : d.payloadFieldCount;
            if (field.source >= earlier || !d.payloadFields) {
                _Error(name + " takes its count from a field that is not an "
                              "earlier payload field");
                break;
            }
            const RigExecFieldDesc &source = d.payloadFields[field.source];
            if (source.extent != RigExecFieldExtent::Scalar ||
                !IntegerSize(sizeOf(source.type)))
                _Error(name + " takes its count from a field that is not "
                              "one unsigned integer");
            break;
        }
        case RigExecFieldExtent::FromInput: {
            size_t p = 0;
            while (p < d.portCount && d.ports[p].port != field.port)
                ++p;
            if (p == d.portCount || !(d.ports[p].flags & kRigExecPortInput) ||
                (d.ports[p].flags & kRigExecPortArray)) {
                _Error(name + " takes its count from something that is not "
                              "an input of the op");
                break;
            }
            const auto input = _typeIndex.find(d.ports[p].type);
            if (input != _typeIndex.end() &&
                !_typeEntries[input->second].desc->count)
                _Error(name + " takes its count from an input whose type "
                              "has no element count");
            break;
        }
        }
    }
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
RigExecRunEachInstance(const RigExecOpBatch &batch, size_t *failed,
                       std::string *error)
{
    const RigExecOpRunFn run = batch.op->run;
    for (size_t k = 0; k < batch.count; ++k) {
        if (!RigExecBatchActive(batch, k))
            continue;
        if (error)
            error->clear();
        if (!run(RigExecBatchPayload(batch, k), RigExecBatchState(batch, k),
                 RigExecBatchInstance(batch, k), error)) {
            *failed = k;
            return false;
        }
    }
    return true;
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
