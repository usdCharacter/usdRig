// Compiled into each consumer of librigExecOracle. The call keeps the
// shared library on the link line when the consumer only reaches the
// hooks through librigExec, and registers its immutable dispatch table.
#include "rigExecOracle/registerOracle.h"

namespace rigExec {
namespace {
struct Anchor {
    Anchor() { RigExecRegisterOracle(); }
};
const Anchor anchor;
}
}
