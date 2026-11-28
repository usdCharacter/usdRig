// RigExec arbitrary-length single-chain IK. The kernel is
// singleChainIkKernel.h, shared with the runtime and the ops.
#include "singleChainIk.h"
#include "dualQuat.h"
#include "singleChainIkKernel.h"

namespace rigExec {

std::vector<RigExecPointFrame>
RigExecSolveSingleChainIk(const std::vector<RigExecPointFrame> &currentFrames,
                          const RigExecPointFrame &effectorFrame,
                          const RigExecSingleChainIkParams &params)
{
    RigExecSingleChainIkKernelParams<GfVec3d> p;
    // RotatePlane 0, SingleChain 1; any other value fails the solve.
    p.mode = int(params.mode);
    p.pole = params.pole;
    p.twistDegrees = params.twistDegrees;
    p.weight = params.weight;
    p.preserveJointOrientation = params.preserveJointOrientation;
    p.stretch = params.stretch;
    return RigExecSolveSingleChainIkKernel<GfVec3f, GfVec3d>(currentFrames,
                                                             effectorFrame, p);
}

} // namespace rigExec
