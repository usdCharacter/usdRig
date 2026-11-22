#include "rigExecOps/library.h"

namespace rigExecOps {
namespace {

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    RegisterPropertyMathOps(registrar);
    RegisterPointOps(registrar);
    RegisterDeltaMushOps(registrar);
    RegisterLatticeOps(registrar);
    RegisterWrinkleOps(registrar);
}

const rigExec::RigExecOpLibraryDesc kLibrary = [] {
    rigExec::RigExecOpLibraryDesc d;
    d.name = "rigExecOps";
    d.registerOps = &Register;
    return d;
}();

} // namespace
} // namespace rigExecOps

RIGEXEC_OP_LIBRARY_ENTRY(rigExecOps)
{
    return &rigExecOps::kLibrary;
}
