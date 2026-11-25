// The space switch's matrix primitives on Rr types, shared by the runtime's
// pose steps and the switch and match ops.
#ifndef RIGEXEC_RUNTIME_SPACE_SWITCH_MATH_H
#define RIGEXEC_RUNTIME_SPACE_SWITCH_MATH_H

#include "rigExecMath/switchMathKernel.h"
#include "rigExecRuntime/runtimeMath.h"

namespace rigExec {

struct RrSlerpOp {
    RrQuatd operator()(double alpha, const RrQuatd &q0, const RrQuatd &q1) const
    {
        return RrSlerp(alpha, q0, q1);
    }
};

using RrSpaceSwitchMath =
    RigExecSpaceSwitchMathT<RrMat3d, RrVec3d, RrMat4d, RrSlerpOp>;

} // namespace rigExec

#endif // RIGEXEC_RUNTIME_SPACE_SWITCH_MATH_H
