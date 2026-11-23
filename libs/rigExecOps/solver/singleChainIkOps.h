#ifndef RIGEXEC_OPS_SOLVER_SINGLE_CHAIN_IK_OPS_H
#define RIGEXEC_OPS_SOLVER_SINGLE_CHAIN_IK_OPS_H

/// Arbitrary-length single-chain IK on rigExecRig frames. The kernel is
/// rigExecMath/singleChainIkKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

struct SingleChainIkPayload {
    /// FBX SingleChain: the effector's orientation sets the plane and the
    /// end; pole and twist are ignored. Otherwise RotatePlane.
    bool singleChain = false;
    /// Keep each joint's orientation relative to its bone.
    bool preserveJointOrientation = false;

    bool operator==(const SingleChainIkPayload &o) const
    {
        return singleChain == o.singleChain &&
               preserveJointOrientation == o.preserveJointOrientation;
    }
};

const rigExec::RigExecOpHandle &SingleChainIk();

// Chain (PointFrameArray, root to end), Effector (PointFrame), optional
// Pole (Vec3d, an absolute point; 0, 1, 0), Twist (Double, degrees; 0),
// Weight (Double, clamped to [0, 1]; 1) and Stretch (Double, in [0, 1];
// 0). Out (PointFrameArray, the chain's rests). An unsolvable chain comes
// back flagged degenerate.
const rigExec::RigExecPortHandle &SingleChainIkChain();
const rigExec::RigExecPortHandle &SingleChainIkEffector();
const rigExec::RigExecPortHandle &SingleChainIkPole();
const rigExec::RigExecPortHandle &SingleChainIkTwist();
const rigExec::RigExecPortHandle &SingleChainIkWeight();
const rigExec::RigExecPortHandle &SingleChainIkStretch();
const rigExec::RigExecPortHandle &SingleChainIkOut();

} // namespace rigExecOps

#endif
