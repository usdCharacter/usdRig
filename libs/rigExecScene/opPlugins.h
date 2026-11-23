#ifndef RIGEXEC_SCENE_OP_PLUGINS_H
#define RIGEXEC_SCENE_OP_PLUGINS_H

/// Op libraries shipped as OpenUSD plugins. A plugin's plugInfo carries
///
///     "Info": {"RigExecOpLibrary": {
///         "abiVersion": <kRigExecRegistryAbiVersion>,
///         "entry": "<lib>_RigExecOpLibrary",
///         "typeEntries": ["<lib>_RigExecTypeLibrary", ...],
///         "binders": "<lib>_RigExecExternalOpBinders"}}
///
/// where typeEntries and binders are optional. The cmake function
/// rigexec_add_op_plugin writes it. Discovery loads each such plugin, takes
/// its libraries into the one registry initialization, and adds its
/// binders (rigExecScene/externalOps.h), so the scene program runs a plugin
/// mover's op with no core code naming it. A host without OpenUSD passes
/// its libraries to RigExecRegistry::Initialize itself.

#include <string>

namespace rigExec {

/// Initializes the op registry with the built-in libraries and every op
/// plugin's, discovering the plugins on the first call. False with
/// \p error naming each plugin that could not join: a registry or binder
/// ABI it was not built for, a library that does not load, an entry symbol
/// it lacks, libraries the registry refuses, a binder for a schema type
/// another already binds, or a registry initialized before discovery.
bool RigExecEnsureSceneOpRegistry(std::string *error = nullptr);

} // namespace rigExec

#endif
