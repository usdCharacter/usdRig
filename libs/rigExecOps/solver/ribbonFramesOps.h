#ifndef RIGEXEC_OPS_SOLVER_RIBBON_FRAMES_OPS_H
#define RIGEXEC_OPS_SOLVER_RIBBON_FRAMES_OPS_H

/// The ribbon solver: rotation-minimizing frames sampled along a posed and a
/// rest B-spline. The kernel is rigExecMath/ribbonFramesKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

struct SampleRibbonFramesPayload {
    /// Samples along each curve, at least two.
    int sampleCount = 2;

    bool operator==(const SampleRibbonFramesPayload &o) const
    {
        return sampleCount == o.sampleCount;
    }
};

const rigExec::RigExecOpHandle &SampleRibbonFrames();

// Posed and Rest (Vec3fArray, cubic uniform B-spline control points),
// optional JointRests (PointLandmarksList) and JointRestLive (BoolArray):
// each sample k live in both is re-based onto JointRests[k]. Out
// (PointFrameArray, one posed frame per sample on the rest sample's
// landmarks or its live joint rest). A curve that does not sample is an
// error.
const rigExec::RigExecPortHandle &SampleRibbonFramesPosed();
const rigExec::RigExecPortHandle &SampleRibbonFramesRest();
const rigExec::RigExecPortHandle &SampleRibbonFramesJointRests();
const rigExec::RigExecPortHandle &SampleRibbonFramesJointRestLive();
const rigExec::RigExecPortHandle &SampleRibbonFramesOut();

} // namespace rigExecOps

#endif
