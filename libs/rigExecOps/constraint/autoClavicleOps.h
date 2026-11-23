#ifndef RIGEXEC_OPS_CONSTRAINT_AUTO_CLAVICLE_OPS_H
#define RIGEXEC_OPS_CONSTRAINT_AUTO_CLAVICLE_OPS_H

/// The auto clavicle's shift of its target. The kernel is
/// rigExecMath/autoClavicleKernel.h.

#include "rigExecMath/autoClavicleKernel.h"
#include "rigExecOps/library.h"

namespace rigExecOps {

struct AutoClavicleOpPayload {
    rigExec::RigExecAutoClavicleConstants constants;
    /// The limb's own soft-distance IK options are fed.
    bool hasLimb = false;

    bool operator==(const AutoClavicleOpPayload &o) const
    {
        return constants == o.constants && hasLimb == o.hasLimb;
    }
};

const rigExec::RigExecOpHandle &AutoClavicle();

// Frames are Mat4d, row vectors, rig-common. Target, Pivot, Anchor and
// AnchorDefault, FkPosed (the first FK control) and FkDefault (Mat4dArray,
// the three FK defaults), optional IkTarget and Pole, IkBlend and Amount
// (Double). With the limb: LimbRests (Mat4dArray, the IK joints' three rest
// frames), Stretch, Pin, UpperScale, LowerScale, SoftDistance,
// ScaleCalibration, Twist (degrees), UpperOffset and LowerOffset (Double).
// Delta (Vec3d, the shift of Target's origin) and Out (Mat4d, Target
// shifted by Delta).
const rigExec::RigExecPortHandle &AutoClavicleTarget();
const rigExec::RigExecPortHandle &AutoClaviclePivot();
const rigExec::RigExecPortHandle &AutoClavicleAnchor();
const rigExec::RigExecPortHandle &AutoClavicleAnchorDefault();
const rigExec::RigExecPortHandle &AutoClavicleFkPosed();
const rigExec::RigExecPortHandle &AutoClavicleFkDefault();
const rigExec::RigExecPortHandle &AutoClavicleIkTarget();
const rigExec::RigExecPortHandle &AutoClaviclePole();
const rigExec::RigExecPortHandle &AutoClavicleIkBlend();
const rigExec::RigExecPortHandle &AutoClavicleAmount();
const rigExec::RigExecPortHandle &AutoClavicleLimbRests();
const rigExec::RigExecPortHandle &AutoClavicleStretch();
const rigExec::RigExecPortHandle &AutoClaviclePin();
const rigExec::RigExecPortHandle &AutoClavicleUpperScale();
const rigExec::RigExecPortHandle &AutoClavicleLowerScale();
const rigExec::RigExecPortHandle &AutoClavicleSoftDistance();
const rigExec::RigExecPortHandle &AutoClavicleScaleCalibration();
const rigExec::RigExecPortHandle &AutoClavicleTwist();
const rigExec::RigExecPortHandle &AutoClavicleUpperOffset();
const rigExec::RigExecPortHandle &AutoClavicleLowerOffset();
const rigExec::RigExecPortHandle &AutoClavicleDelta();
const rigExec::RigExecPortHandle &AutoClavicleOut();

} // namespace rigExecOps

#endif
