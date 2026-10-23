#ifndef RIGEXEC_RUNTIME_MOVER_CATALOG_H
#define RIGEXEC_RUNTIME_MOVER_CATALOG_H

#include "rigExecBinary/moverDefinition.h"

namespace rigExec {
// Generated from rigexec_add_mover(BUILTIN ...) declarations. Direct factory
// references retain each implementation when the runtime is a static archive.
const RigExecMoverFactory *RrBuiltinMovers(size_t *count);
}
#endif
