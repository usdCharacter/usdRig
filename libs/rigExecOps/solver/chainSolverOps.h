#ifndef RIGEXEC_OPS_SOLVER_CHAIN_SOLVER_OPS_H
#define RIGEXEC_OPS_SOLVER_CHAIN_SOLVER_OPS_H

/// The FK chain and the analytic two-bone IK on rigExecRig frames. The kernel
/// is rigExecMath/chainSolverKernel.h.

#include "rigExecOps/library.h"
#include "rigExecMath/jointGroupKernel.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

struct FkChainPayload {
    /// One per control: the parent's index, below its own, or -1.
    std::vector<int> parentIndices;
    /// segmentScale "toChild": each joint scales along its bone.
    bool scaleSegments = false;

    bool operator==(const FkChainPayload &o) const
    {
        return parentIndices == o.parentIndices &&
               scaleSegments == o.scaleSegments;
    }
};

struct TwoBoneIkPayload {
    /// stretchPolicy "softDistance" (the limb model) rather than
    /// "uniformSegments".
    bool softDistancePolicy = false;
    /// segmentScale "toChild".
    bool scaleSegments = false;
    /// Subtracted from both scales inside the soft stretch.
    double scaleCalibration = 0.0;

    bool operator==(const TwoBoneIkPayload &o) const
    {
        return softDistancePolicy == o.softDistancePolicy &&
               scaleSegments == o.scaleSegments &&
               scaleCalibration == o.scaleCalibration;
    }
};

/// An FK chain's joint groups (RigExecJointGroupAPI).
struct FkJointGroupsPayload {
    /// The first joint's element in In; earlier elements pass through.
    int jointBegin = 0;
    /// Each joint's parent among the joints, or -1.
    std::vector<int32_t> parents;
    std::vector<rigExec::RigExecJointGroup> groups;
    /// The groups' concatenated members, each a joint index.
    std::vector<int32_t> members;

    bool operator==(const FkJointGroupsPayload &o) const
    {
        return jointBegin == o.jointBegin && parents == o.parents &&
               groups == o.groups && members == o.members;
    }
};

const rigExec::RigExecOpHandle &FkChain();
const rigExec::RigExecOpHandle &TwoBoneIk();
const rigExec::RigExecOpHandle &FkJointGroups();

// FkChain: Controls (PointFrameArray, the posed controls and their rests),
// optional OutRests (PointLandmarksList, one per control: the basis each
// accumulated map is applied to; unfed, the control rests), Out
// (PointFrameArray, rests as applied).
const rigExec::RigExecPortHandle &FkChainControls();
const rigExec::RigExecPortHandle &FkChainOutRests();
const rigExec::RigExecPortHandle &FkChainOut();

// TwoBoneIk: Root, Effector and Pole (PointFrame), Rests (PointLandmarksList,
// the root, mid and end joint rests), optional Space (Mat4d, the space the
// chain is measured in; identity), Bend (Double, radians), UpperOffset and
// LowerOffset (Double), Stretch (Float, 1), Softness (Float), Pin (Float),
// UpperScale and LowerScale (Double, 1), SoftDistance (Float, in rest
// units) and Twist (Float, degrees). Out (PointFrameArray, three frames and
// the rests).
const rigExec::RigExecPortHandle &TwoBoneIkRoot();
const rigExec::RigExecPortHandle &TwoBoneIkEffector();
const rigExec::RigExecPortHandle &TwoBoneIkPole();
const rigExec::RigExecPortHandle &TwoBoneIkRests();
const rigExec::RigExecPortHandle &TwoBoneIkSpace();
const rigExec::RigExecPortHandle &TwoBoneIkBend();
const rigExec::RigExecPortHandle &TwoBoneIkUpperOffset();
const rigExec::RigExecPortHandle &TwoBoneIkLowerOffset();
const rigExec::RigExecPortHandle &TwoBoneIkStretch();
const rigExec::RigExecPortHandle &TwoBoneIkSoftness();
const rigExec::RigExecPortHandle &TwoBoneIkPin();
const rigExec::RigExecPortHandle &TwoBoneIkUpperScale();
const rigExec::RigExecPortHandle &TwoBoneIkLowerScale();
const rigExec::RigExecPortHandle &TwoBoneIkSoftDistance();
const rigExec::RigExecPortHandle &TwoBoneIkTwist();
const rigExec::RigExecPortHandle &TwoBoneIkOut();

// FkJointGroups: In (PointFrameArray, an FkChain's Out), Selectors
// (DoubleArray, each group's active member), optional Start (PointFrame) and
// StartRest (PointLandmarks), fed together. Out (PointFrameArray): In, with
// each joint under an inactive member set to its rest carried by the start's
// rest-to-pose map.
const rigExec::RigExecPortHandle &FkJointGroupsIn();
const rigExec::RigExecPortHandle &FkJointGroupsSelectors();
const rigExec::RigExecPortHandle &FkJointGroupsStart();
const rigExec::RigExecPortHandle &FkJointGroupsStartRest();
const rigExec::RigExecPortHandle &FkJointGroupsOut();

} // namespace rigExecOps

#endif
