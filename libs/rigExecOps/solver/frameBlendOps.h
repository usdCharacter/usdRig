#ifndef RIGEXEC_OPS_SOLVER_FRAME_BLEND_OPS_H
#define RIGEXEC_OPS_SOLVER_FRAME_BLEND_OPS_H

/// The SRT frame blend and the swing/twist distribution on rigExecRig frames.
/// The kernel is rigExecMath/frameBlendKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

struct BlendFramesPayload {
    /// scaleBlend "log" rather than "linear".
    bool logScale = false;

    bool operator==(const BlendFramesPayload &o) const
    {
        return logScale == o.logScale;
    }
};

const rigExec::RigExecOpHandle &BlendFrames();
const rigExec::RigExecOpHandle &DistributeTwist();

// BlendFrames (shortest-arc rotation): A and B (PointFrame), Rest (PointLandmarks, the rest both
// are posed from), Weight (Double, clamped to [0, 1]; NaN blends as the
// kernel does), optional OutRest
// (PointLandmarks, the basis the blend is applied to; unfed, Rest). Out
// (PointFrame).
const rigExec::RigExecPortHandle &BlendFramesA();
const rigExec::RigExecPortHandle &BlendFramesB();
const rigExec::RigExecPortHandle &BlendFramesRest();
const rigExec::RigExecPortHandle &BlendFramesWeight();
const rigExec::RigExecPortHandle &BlendFramesOutRest();
const rigExec::RigExecPortHandle &BlendFramesOut();

// DistributeTwist: Start and End (PointFrame), StartRest and EndRest
// (PointLandmarks), Weights (DoubleArray, one sample each), optional Turns
// (Double, whole extra turns of twist; 0), JointRests (PointLandmarksList)
// and JointRestLive (BoolArray): each sample k live in both is re-based
// onto JointRests[k]. Out (PointFrameArray, one frame per weight on
// StartRest or its live joint rest).
const rigExec::RigExecPortHandle &DistributeTwistStart();
const rigExec::RigExecPortHandle &DistributeTwistEnd();
const rigExec::RigExecPortHandle &DistributeTwistStartRest();
const rigExec::RigExecPortHandle &DistributeTwistEndRest();
const rigExec::RigExecPortHandle &DistributeTwistWeights();
const rigExec::RigExecPortHandle &DistributeTwistTurns();
const rigExec::RigExecPortHandle &DistributeTwistJointRests();
const rigExec::RigExecPortHandle &DistributeTwistJointRestLive();
const rigExec::RigExecPortHandle &DistributeTwistOut();

} // namespace rigExecOps

#endif
