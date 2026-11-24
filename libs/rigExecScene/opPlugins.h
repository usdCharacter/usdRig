#ifndef RIGEXEC_SCENE_OP_PLUGINS_H
#define RIGEXEC_SCENE_OP_PLUGINS_H

/// Op libraries shipped as OpenUSD plugins. A plugin's plugInfo carries
///
///     "Info": {"RigExecOpLibrary": {
///         "abiVersion": <kRigExecRegistryAbiVersion>,
///         "entry": "<lib>_RigExecOpLibrary",
///         "typeEntries": ["<lib>_RigExecTypeLibrary", ...],
///         "binders": "<lib>_RigExecExternalOpBinders",
///         "schemaTypes": ["<schema type>", ...]}}
///
/// where typeEntries, binders and schemaTypes are optional; schemaTypes
/// names the schema types the binders bind, so a plugin that cannot load
/// still says which movers it fails. The cmake function
/// rigexec_add_op_plugin writes it. Discovery loads each such plugin, takes
/// its libraries into the one registry initialization, and adds its
/// binders (rigExecScene/externalOps.h), so a plugin mover runs on its op
/// with no core code naming it. A host without OpenUSD passes
/// its libraries to RigExecRegistry::Initialize itself.

#include <string>
#include <vector>

namespace rigExec {

/// Initializes the op registry with the built-in libraries and every op
/// plugin that could join, discovering the plugins on the first call.
/// False only when the registry does not initialize, with \p error ending
/// with the plugins discovery considered.
bool RigExecEnsureSceneOpRegistry(std::string *error = nullptr);

/// One line per op plugin that could not join, naming it: a registry or
/// binder ABI it was not built for, a library that does not load, an entry
/// symbol it lacks, libraries the registry refuses, a binder for a schema
/// type another already binds, or a registry initialized before discovery.
/// A last line lists the plugins discovery considered. Empty when every
/// plugin joined.
std::vector<std::string> RigExecSceneOpPluginErrors();

/// False with \p error naming each plugin that binds \p schemaType, by its
/// schemaTypes or its binder table, and could not join.
bool RigExecSceneOpSchemaJoined(const std::string &schemaType,
                                std::string *error = nullptr);

} // namespace rigExec

#endif
