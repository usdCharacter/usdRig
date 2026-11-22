#ifndef RIGEXEC_OPS_LIBRARY_H
#define RIGEXEC_OPS_LIBRARY_H

/// The rigExecOps op library: every op family below libs/rigExecOps/.
/// USD-free.

#include "rigExec/api/registry.h"

namespace rigExecOps {

/// Each family's registration, called by the library's entry.
void RegisterPropertyMathOps(rigExec::RigExecOpRegistrar &registrar);
void RegisterPointOps(rigExec::RigExecOpRegistrar &registrar);
void RegisterDeltaMushOps(rigExec::RigExecOpRegistrar &registrar);
void RegisterLatticeOps(rigExec::RigExecOpRegistrar &registrar);
void RegisterWrinkleOps(rigExec::RigExecOpRegistrar &registrar);

} // namespace rigExecOps

RIGEXEC_OP_LIBRARY_ENTRY(rigExecOps);

#endif
