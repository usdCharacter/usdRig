#ifndef RIGEXEC_OPS_SOLVER_RIBBON_FRAMES_OPS_H
#define RIGEXEC_OPS_SOLVER_RIBBON_FRAMES_OPS_H

/// The ribbon solver: rotation-minimizing frames sampled along a posed
/// and a rest B-spline. The kernel is rigExecMath/ribbonFramesKernel.h,
/// shared with the native and zero-USD runtimes. USD-free.

#include "rigExecOps/library.h"

namespace rigExecOps {

struct SampleRibbonFramesPayload {
    /// Samples along each curve, at least two.
    int sampleCount = 2;
};

const rigExec::RigExecOpHandle &SampleRibbonFrames();

// Posed and Rest (Vec3fArray, cubic uniform B-spline control points). Out
// (PointFrameArray, one posed frame per sample on the rest sample's
// landmarks). A curve that does not sample is an error.
const rigExec::RigExecPortHandle &SampleRibbonFramesPosed();
const rigExec::RigExecPortHandle &SampleRibbonFramesRest();
const rigExec::RigExecPortHandle &SampleRibbonFramesOut();

} // namespace rigExecOps

#endif
