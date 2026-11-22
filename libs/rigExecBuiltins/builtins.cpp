#include "rigExecBuiltins/builtins.h"

#include "rigExecOps/deform/matrixDeformOp.h"

#include <mutex>

RIGEXEC_TYPE_LIBRARY_ENTRY(rigExecStdTypes);
RIGEXEC_TYPE_LIBRARY_ENTRY(rigExecRigTypes);
RIGEXEC_OP_LIBRARY_ENTRY(rigExecOps);

namespace rigExec {

RigExecRegistryInit
RigExecBuiltinRegistrations()
{
    RigExecRegistryInit init;
    init.typeLibraries = {rigExecStdTypes_RigExecTypeLibrary(),
                          rigExecRigTypes_RigExecTypeLibrary()};
    init.opLibraries = {rigExecOps_RigExecOpLibrary()};
    return init;
}

bool
RigExecEnsureBuiltinRegistry(std::string *error)
{
    static std::once_flag once;
    static std::string failure;
    std::call_once(once, [] {
        if (RigExecRegistry::IsInitialized())
            return;
        std::vector<std::string> errors;
        if (!RigExecRegistry::Initialize(RigExecBuiltinRegistrations(),
                                         &errors))
            for (const std::string &e : errors)
                failure += (failure.empty() ? "" : "; ") + e;
    });
    if (failure.empty() && !rigExecOps::MatrixDeform().IsValid())
        failure = "the op registry was initialized without the built-ins";
    if (failure.empty())
        return true;
    if (error)
        *error = failure;
    return false;
}

} // namespace rigExec
