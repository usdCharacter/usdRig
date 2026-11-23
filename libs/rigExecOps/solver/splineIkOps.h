#ifndef RIGEXEC_OPS_SOLVER_SPLINE_IK_OPS_H
#define RIGEXEC_OPS_SOLVER_SPLINE_IK_OPS_H

/// The spline-IK spine on rigExecRig frames. The kernel is
/// rigExecMath/splineIkKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

struct SplineIkPayload {
    /// restLength "chain" (the summed rest segments) rather than "curve".
    bool restLengthChain = false;
    /// Length floor as a fraction of the rest root-to-end chord; 0 is off.
    double minLengthRatio = 0.0;
    /// Turn the root tangent onto the root-to-end chord.
    bool aimRootTangent = false;

    bool operator==(const SplineIkPayload &o) const
    {
        return restLengthChain == o.restLengthChain &&
               minLengthRatio == o.minLengthRatio &&
               aimRootTangent == o.aimRootTangent;
    }
};

const rigExec::RigExecOpHandle &SplineIk();

// SplineIk: RestJoints (PointFrameList, the chain at rest), RestRoot,
// RestMid and RestEnd (PointFrame, the controls at rest), Root, Mid and
// End (PointFrame, the posed controls), optional VolumeWeights
// (DoubleArray, one per joint or empty), PreserveVolume (Double, 1),
// MidFollow (Double, 0.5), Roll and Twist (Double, radians). Out
// (PointFrameArray, the joints on their rests).
const rigExec::RigExecPortHandle &SplineIkRestJoints();
const rigExec::RigExecPortHandle &SplineIkRestRoot();
const rigExec::RigExecPortHandle &SplineIkRestMid();
const rigExec::RigExecPortHandle &SplineIkRestEnd();
const rigExec::RigExecPortHandle &SplineIkRoot();
const rigExec::RigExecPortHandle &SplineIkMid();
const rigExec::RigExecPortHandle &SplineIkEnd();
const rigExec::RigExecPortHandle &SplineIkVolumeWeights();
const rigExec::RigExecPortHandle &SplineIkPreserveVolume();
const rigExec::RigExecPortHandle &SplineIkMidFollow();
const rigExec::RigExecPortHandle &SplineIkRoll();
const rigExec::RigExecPortHandle &SplineIkTwist();
const rigExec::RigExecPortHandle &SplineIkOut();

} // namespace rigExecOps

#endif
