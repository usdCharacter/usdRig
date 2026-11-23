#include "rigExecScene/opPlugins.h"

#include "rigExec/api/registry.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecScene/externalOps.h"

#include "pxr/base/arch/library.h"
#include "pxr/base/js/value.h"
#include "pxr/base/plug/plugin.h"
#include "pxr/base/plug/registry.h"

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

/// What discovery found: the plugins' libraries, in plugin order, the names
/// of their op libraries, and why any plugin could not join.
struct Discovery {
    RigExecRegistryInit libraries;
    std::vector<std::string> opLibraries;
    std::vector<std::string> errors;
};

// Discovery runs once per process, at the first scene program's Prepare on
// its owner thread; every later call reads its result.
std::once_flag gDiscovered;
Discovery gDiscovery;

const char *kKey = "RigExecOpLibrary";

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
        if (!info->second.IsObject()) {
            out->errors.push_back(who + ": " + kKey + " is not an object");
            continue;
        }
        if (RigExecRegistry::IsInitialized()) {
            out->errors.push_back(who +
                                  " joined no registry: it was initialized "
                                  "before op plugin discovery");
            continue;
        }
        RigExecRegistryInit libraries;
        std::vector<const RigExecExternalOpBinder *> found;
        std::string why;
        if (!LoadPlugin(plugin, info->second.GetJsObject(), &libraries, &found,
                        &why)) {
            out->errors.push_back(who + " " + why);
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
            out->errors.push_back(
                who + ": " +
                (refused.empty() ? std::string("refused") : refused.front()));
            continue;
        }
        out->libraries = std::move(accepted);
        out->opLibraries.push_back(libraries.opLibraries.front()->name);
        binders.insert(binders.end(), found.begin(), found.end());
    }
    std::string why;
    if (!RigExecAddExternalOpBinders(binders, &why))
        out->errors.push_back("op plugin binders: " + why);
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

} // namespace

bool
RigExecEnsureSceneOpRegistry(std::string *error)
{
    std::call_once(gDiscovered, [] { Discover(&gDiscovery); });
    std::string why;
    if (!RigExecEnsureBuiltinRegistry(&why, &gDiscovery.libraries)) {
        if (error)
            *error = why;
        return false;
    }
    std::vector<std::string> errors = gDiscovery.errors;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (const std::string &library : gDiscovery.opLibraries)
        if (!Registered(registry, library))
            errors.push_back("op library " + library +
                             " is not in this process's registry: it was "
                             "initialized before op plugin discovery");
    if (errors.empty())
        return true;
    if (error) {
        error->clear();
        for (const std::string &e : errors)
            *error += (error->empty() ? "" : "; ") + e;
    }
    return false;
}

} // namespace rigExec
