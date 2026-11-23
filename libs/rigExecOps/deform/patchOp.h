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

    bool operator==(const PatchPayload &o) const
    {
        return uVertexCount == o.uVertexCount &&
               vVertexCount == o.vVertexCount && uOrder == o.uOrder &&
               vOrder == o.vOrder && uPeriodic == o.uPeriodic &&
               vPeriodic == o.vPeriodic;
    }
};

const rigExec::RigExecOpHandle &Patch();

// Points (Vec3fArray), the patch's ControlPoints (Vec3fArray, u fastest),
// PointWeights (DoubleArray, empty or one per control point), UKnots and
// VKnots (DoubleArray), UvPoints (Vec3fArray, one per point or one for
// all), UvShapes (Vec3fArray each, as many UV points; any number), UvWeights
// (DoubleArray, one per shape), ContactLower and ContactUpper (IntArray,
// UV point pairs), Envelope (optional FloatArray, one per point), Out. An
// unusable patch, shape, weight, contact or UV point is an error.
const rigExec::RigExecPortHandle &PatchPoints();
const rigExec::RigExecPortHandle &PatchControlPoints();
const rigExec::RigExecPortHandle &PatchPointWeights();
const rigExec::RigExecPortHandle &PatchUKnots();
const rigExec::RigExecPortHandle &PatchVKnots();
const rigExec::RigExecPortHandle &PatchUvPoints();
const rigExec::RigExecPortHandle &PatchUvShapes();
const rigExec::RigExecPortHandle &PatchUvWeights();
const rigExec::RigExecPortHandle &PatchContactLower();
const rigExec::RigExecPortHandle &PatchContactUpper();
const rigExec::RigExecPortHandle &PatchEnvelope();
const rigExec::RigExecPortHandle &PatchOut();

} // namespace rigExecOps

#endif
