#include "rigExecScene/opPlugins.h"

#include "rigExec/api/registry.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecScene/externalOps.h"

#include "pxr/base/arch/library.h"
#include "pxr/base/js/value.h"
#include "pxr/base/plug/plugin.h"
#include "pxr/base/plug/registry.h"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

using OpEntry = const RigExecOpLibraryDesc *(*)();
using TypeEntry = const RigExecTypeLibraryDesc *(*)();
using BinderEntry = const RigExecExternalOpBinderTable *(*)();

/// A plugin that could not join: why, and the schema types it binds.
struct Failure {
    std::string error;
    std::vector<std::string> schemaTypes;
};

/// What discovery found: the plugins it considered, their libraries, in
/// plugin order, the names of their op libraries with the schema types
/// each plugin binds, and the plugins that could not join.
struct Discovery {
    std::vector<std::string> considered;
    RigExecRegistryInit libraries;
    std::vector<std::string> opLibraries;
    std::vector<std::vector<std::string>> opLibraryTypes;
    std::vector<Failure> failures;
};

// Discovery runs once per process, at the first scene program's Prepare on
// its owner thread; every later call reads its result.
std::once_flag gDiscovered;
Discovery gDiscovery;

const char *kKey = "RigExecOpLibrary";

/// The plugInfo's schemaTypes and the schema types \p binders bind.
std::vector<std::string>
SchemaTypes(const JsObject &info,
            const std::vector<const RigExecExternalOpBinder *> &binders)
{
    std::vector<std::string> types;
    const auto declared = info.find("schemaTypes");
    if (declared != info.end() && declared->second.IsArray())
        for (const JsValue &name : declared->second.GetJsArray())
            if (name.IsString())
                types.push_back(name.GetString());
    for (const RigExecExternalOpBinder *binder : binders)
        if (binder && binder->schemaType)
            types.emplace_back(binder->schemaType);
    return types;
}

std::string
StringField(const JsObject &object, const char *name)
{
    const auto found = object.find(name);
    return found != object.end() && found->second.IsString()
               ? found->second.GetString()
               : std::string();
}

/// One plugin's libraries and binders, or false with the reason.
bool
LoadPlugin(const PlugPluginPtr &plugin, const JsObject &info,
           RigExecRegistryInit *libraries,
           std::vector<const RigExecExternalOpBinder *> *binders,
           std::string *why)
{
    const auto abi = info.find("abiVersion");
    if (abi == info.end() || !abi->second.IsInt() ||
        abi->second.GetInt() != int(kRigExecRegistryAbiVersion)) {
        *why = "was not built for registry ABI " +
               std::to_string(kRigExecRegistryAbiVersion);
        return false;
    }
    const std::string entry = StringField(info, "entry");
    if (entry.empty()) {
        *why = "names no op library entry";
        return false;
    }
    if (!plugin->Load()) {
        *why = "did not load";
        return false;
    }
    // Held for the life of the process, as the loaded plugin is.
    void *library = ArchLibraryOpen(plugin->GetPath(), ARCH_LIBRARY_NOW);
    if (!library) {
        *why = "could not be opened at " + plugin->GetPath();
        return false;
    }
    const auto symbol = [&](const std::string &name) {
        return ArchLibraryGetSymbolAddress(library, name.c_str());
    };
    const auto opEntry = reinterpret_cast<OpEntry>(symbol(entry));
    const RigExecOpLibraryDesc *ops = opEntry ? opEntry() : nullptr;
    if (!ops) {
        *why = "has no op library at " + entry;
        return false;
    }
    if (ops->abiVersion != kRigExecRegistryAbiVersion) {
        *why = "has an op library built for registry ABI " +
               std::to_string(ops->abiVersion);
        return false;
    }
    const auto types = info.find("typeEntries");
    if (types != info.end()) {
        if (!types->second.IsArray()) {
            *why = "lists its type entries as something other than an array";
            return false;
        }
        for (const JsValue &name : types->second.GetJsArray()) {
            const auto typeEntry = reinterpret_cast<TypeEntry>(
                name.IsString() ? symbol(name.GetString()) : nullptr);
            const RigExecTypeLibraryDesc *library =
                typeEntry ? typeEntry() : nullptr;
            if (!library || library->abiVersion != kRigExecRegistryAbiVersion) {
                *why = "has no usable type library at " +
                       (name.IsString() ? name.GetString() : std::string("?"));
                return false;
            }
            libraries->typeLibraries.push_back(library);
        }
    }
    libraries->opLibraries.push_back(ops);
    const std::string binderEntry = StringField(info, "binders");
    if (!binderEntry.empty()) {
        const auto entryFn = reinterpret_cast<BinderEntry>(symbol(binderEntry));
        const RigExecExternalOpBinderTable *table =
            entryFn ? entryFn() : nullptr;
        if (!table || table->abiVersion != kRigExecExternalOpBinderAbiVersion ||
            (table->count && !table->binders)) {
            *why = "has no binder table for binder ABI " +
                   std::to_string(kRigExecExternalOpBinderAbiVersion) + " at " +
                   binderEntry;
            return false;
        }
        for (size_t k = 0; k < table->count; ++k)
            binders->push_back(&table->binders[k]);
    }
    return true;
}

void
Append(const RigExecRegistryInit &from, RigExecRegistryInit *to)
{
    to->typeLibraries.insert(to->typeLibraries.end(),
                             from.typeLibraries.begin(),
                             from.typeLibraries.end());
    to->opLibraries.insert(to->opLibraries.end(), from.opLibraries.begin(),
                           from.opLibraries.end());
}

void
Discover(Discovery *out)
{
    std::vector<const RigExecExternalOpBinder *> binders;
    for (const PlugPluginPtr &plugin :
         PlugRegistry::GetInstance().GetAllPlugins()) {
        const JsObject metadata = plugin->GetMetadata();
        const auto info = metadata.find(kKey);
        if (info == metadata.end())
            continue;
        const std::string who = "op plugin " + plugin->GetName();
        out->considered.push_back(plugin->GetName() + " at " +
                                  plugin->GetPath());
        if (!info->second.IsObject()) {
            out->failures.push_back(
                {who + ": " + kKey + " is not an object", {}});
            continue;
        }
        const JsObject &fields = info->second.GetJsObject();
        if (RigExecRegistry::IsInitialized()) {
            out->failures.push_back({who + " joined no registry: it was "
                                           "initialized before op plugin "
                                           "discovery",
                                     SchemaTypes(fields, {})});
            continue;
        }
        RigExecRegistryInit libraries;
        std::vector<const RigExecExternalOpBinder *> found;
        std::string why;
        if (!LoadPlugin(plugin, fields, &libraries, &found, &why)) {
            out->failures.push_back(
                {who + " " + why, SchemaTypes(fields, found)});
            continue;
        }
        // The registry refuses a whole initialization for one bad library,
        // so each plugin is checked with the ones accepted before it.
        RigExecRegistryInit accepted = out->libraries;
        Append(libraries, &accepted);
        RigExecRegistryInit candidate = RigExecBuiltinRegistrations();
        Append(accepted, &candidate);
        std::vector<std::string> refused;
        if (!RigExecRegistry::Validate(candidate, &refused)) {
            out->failures.push_back(
                {who + ": " +
                     (refused.empty() ? std::string("refused")
                                      : refused.front()),
                 SchemaTypes(fields, found)});
            continue;
        }
        out->libraries = std::move(accepted);
        out->opLibraries.push_back(libraries.opLibraries.front()->name);
        out->opLibraryTypes.push_back(SchemaTypes(fields, found));
        binders.insert(binders.end(), found.begin(), found.end());
    }
    std::string why;
    if (!RigExecAddExternalOpBinders(binders, &why))
        out->failures.push_back(
            {"op plugin binders: " + why, SchemaTypes(JsObject(), binders)});
}

bool
Registered(const RigExecRegistry &registry, const std::string &library)
{
    for (size_t k = 0; k < registry.OpCount(); ++k) {
        const RigExecOpLibraryDesc *owner = registry.OpAt(uint32_t(k)).library;
        if (owner && owner->name && library == owner->name)
            return true;
    }
    return false;
}

/// The plugins discovery considered, for a diagnostic.
std::string
ConsideredText()
{
    std::string considered;
    for (const std::string &plugin : gDiscovery.considered)
        considered += (considered.empty() ? "" : ", ") + plugin;
    return "op plugins considered: " +
           (considered.empty() ? std::string("none") : considered);
}

/// Discovery's failures, and the op libraries it accepted that a registry
/// initialized before discovery does not hold.
std::vector<Failure>
Failures()
{
    std::vector<Failure> failures = gDiscovery.failures;
    if (!RigExecRegistry::IsInitialized())
        return failures;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (size_t k = 0; k < gDiscovery.opLibraries.size(); ++k)
        if (!Registered(registry, gDiscovery.opLibraries[k]))
            failures.push_back({"op library " + gDiscovery.opLibraries[k] +
                                    " is not in this process's registry: it "
                                    "was initialized before op plugin "
                                    "discovery",
                                gDiscovery.opLibraryTypes[k]});
    return failures;
}

} // namespace

bool
RigExecEnsureSceneOpRegistry(std::string *error)
{
    std::call_once(gDiscovered, [] { Discover(&gDiscovery); });
    std::string why;
    if (RigExecEnsureBuiltinRegistry(&why, &gDiscovery.libraries))
        return true;
    if (error)
        *error = why + "; " + ConsideredText();
    return false;
}

std::vector<std::string>
RigExecSceneOpPluginErrors()
{
    std::vector<std::string> errors;
    for (const Failure &failure : Failures())
        errors.push_back(failure.error);
    if (!errors.empty())
        errors.push_back(ConsideredText());
    return errors;
}

bool
RigExecSceneOpSchemaJoined(const std::string &schemaType, std::string *error)
{
    std::string why;
    for (const Failure &failure : Failures())
        if (std::find(failure.schemaTypes.begin(), failure.schemaTypes.end(),
                      schemaType) != failure.schemaTypes.end())
            why += (why.empty() ? "" : "; ") + failure.error;
    if (why.empty())
        return true;
    if (error)
        *error = why;
    return false;
}

std::vector<std::string>
RigExecSceneOpPluginsConsidered()
{
    return gDiscovery.considered;
}

} // namespace rigExec
