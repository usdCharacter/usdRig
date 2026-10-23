// Compiled into each consumer of librigExecScene. The call keeps the
// shared library on the link line when the consumer only reaches the
// translation hooks through librigExec, and explicitly installs those hooks.
#include "rigExecScene/registerScene.h"

namespace rigExec {
namespace {
struct Anchor {
    Anchor() { RigExecRegisterScene(); }
};
const Anchor anchor;
}
}
