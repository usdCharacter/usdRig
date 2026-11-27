#ifndef RIGEXEC_OPS_SOLVER_FRAME_BLEND_OPS_H
#define RIGEXEC_OPS_SOLVER_FRAME_BLEND_OPS_H

/// The SRT frame blend and the swing/twist distribution on rigExecRig frames.
/// The kernel is rigExecMath/frameBlendKernel.h.

#include "rigExecOps/library.h"

#include <cstdint>

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
const rigExec::RigExecOpHandle &BlendFrameArrays();

struct BlendFrameArraysPayload {
    /// scaleBlend "log" rather than "linear".
    bool logScale = false;
    /// The frames A and B lead with that are no element of the blend.
    uint32_t leadingA = 0, leadingB = 0;

    bool operator==(const BlendFrameArraysPayload &o) const
    {
        return logScale == o.logScale && leadingA == o.leadingA &&
               leadingB == o.leadingB;
    }
};

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

// BlendFrameArrays: optional A and B (PointFrameArray, each after its leading
// frames), Weight (Double, clamped to [0, 1]), optional JointRests
// (PointLandmarksList) and JointRestLive (BoolArray). Out (PointFrameArray):
// with one input fed, that input; with both, each element blended as
// BlendFrames does on A's rest, re-based onto JointRests[k] where
// JointRestLive marks it, or no frame when the inputs' counts differ.
const rigExec::RigExecPortHandle &BlendFrameArraysA();
const rigExec::RigExecPortHandle &BlendFrameArraysB();
const rigExec::RigExecPortHandle &BlendFrameArraysWeight();
const rigExec::RigExecPortHandle &BlendFrameArraysJointRests();
const rigExec::RigExecPortHandle &BlendFrameArraysJointRestLive();
const rigExec::RigExecPortHandle &BlendFrameArraysOut();

} // namespace rigExecOps

#endif
