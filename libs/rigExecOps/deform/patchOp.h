#ifndef RIGEXEC_OPS_DEFORM_PATCH_OP_H
#define RIGEXEC_OPS_DEFORM_PATCH_OP_H

/// Patch: points placed on a NURBS patch at UV points, which are first
/// mixed toward weighted UV shapes and kept from crossing at contact pairs.
/// The kernel is rigExecMath/patchKernel.h.

#include "rigExecOps/library.h"

#include <cstdint>

namespace rigExecOps {

/// The patch's structure, fixed at compile.
struct PatchPayload {
    int32_t uVertexCount = 0;
    int32_t vVertexCount = 0;
    int32_t uOrder = 0;
    int32_t vOrder = 0;
    /// Closed or periodic form in u and v; open otherwise.
    bool uPeriodic = false;
    bool vPeriodic = false;
    /// How many UV shapes UvShapes holds, one after another.
    int32_t shapes = 0;

    bool operator==(const PatchPayload &o) const
    {
        return uVertexCount == o.uVertexCount &&
               vVertexCount == o.vVertexCount && uOrder == o.uOrder &&
               vOrder == o.vOrder && uPeriodic == o.uPeriodic &&
               vPeriodic == o.vPeriodic && shapes == o.shapes;
    }
};

const rigExec::RigExecOpHandle &Patch();

// A whole ranged op. Points (Vec3fArray), the patch's ControlPoints
// (Vec3fArray, u fastest), PointWeights (DoubleArray, empty or one per
// control point), UKnots and VKnots (DoubleArray), UvPoints (Vec3fArray, one
// per point or one for all), UvShapes (optional Vec3fArray: the payload's
// shapes, each as many points as UvPoints, one after another), UvWeights
// (optional DoubleArray, one per shape) or else AuthoredUvWeights (optional
// FloatArray, one per shape; empty or unfed is zero for each), ContactLower
// and ContactUpper (IntArray, UV point pairs), Out at full strength. An
// unusable patch, shape, weight, contact or UV point is an error.
const rigExec::RigExecPortHandle &PatchPoints();
const rigExec::RigExecPortHandle &PatchControlPoints();
const rigExec::RigExecPortHandle &PatchPointWeights();
const rigExec::RigExecPortHandle &PatchUKnots();
const rigExec::RigExecPortHandle &PatchVKnots();
const rigExec::RigExecPortHandle &PatchUvPoints();
const rigExec::RigExecPortHandle &PatchUvShapes();
const rigExec::RigExecPortHandle &PatchUvWeights();
const rigExec::RigExecPortHandle &PatchAuthoredUvWeights();
const rigExec::RigExecPortHandle &PatchContactLower();
const rigExec::RigExecPortHandle &PatchContactUpper();
const rigExec::RigExecPortHandle &PatchOut();

} // namespace rigExecOps

#endif
