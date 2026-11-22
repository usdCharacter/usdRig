#ifndef RIGEXEC_BUILTINS_BUILTINS_H
#define RIGEXEC_BUILTINS_BUILTINS_H

/// The built-in type and op libraries (rigExecStdTypes, rigExecRigTypes,
/// rigExecOps) linked once into one shared module, so every library and
/// executable that runs ops sees one set of handles. Transitional: from
/// step 10 each host initializes the registry with the libraries it links.

#include "rigExec/api/registry.h"

#include <string>

namespace rigExec {

/// The built-in libraries in freeze order.
RigExecRegistryInit RigExecBuiltinRegistrations();

/// Initializes the registry with the built-ins unless it already is. False
/// with \p error when that fails, or when the registry was initialized
/// without them.
bool RigExecEnsureBuiltinRegistry(std::string *error = nullptr);

} // namespace rigExec

#endif
