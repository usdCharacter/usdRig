#ifndef RIGEXEC_OPS_DEFORM_LAYERED_SKIN_OP_H
#define RIGEXEC_OPS_DEFORM_LAYERED_SKIN_OP_H

/// LayeredSkin: points skinned from their entering or base positions, the
/// entering points first carried by an input transform, then blended toward
/// the skinned result by a per-point mask. The kernel is
/// rigExecMath/layeredSkinKernel.h.

#include "rigExecOps/library.h"

#include <cstdint>

namespace rigExecOps {

/// How a layered skin is configured at compile.
struct LayeredSkinPayload {
    /// Influence slots per point.
    int32_t elementSize = 1;
    /// Dual quaternion skinning, else linear.
    bool dualQuaternion = false;

    bool operator==(const LayeredSkinPayload &o) const
    {
        return elementSize == o.elementSize &&
               dualQuaternion == o.dualQuaternion;
    }
};

const rigExec::RigExecOpHandle &LayeredSkin();

// A whole ranged op. Points (Vec3fArray), Transforms (Mat4dArray, one per
// influence), Indices (IntArray) and Weights (FloatArray, elementSize per
// point), Mask (FloatArray, empty or one weight in [0, 1] per point), Base
// (optional Vec3fArray, empty or unfed to skin the entering points),
// InputTransform (optional Mat4d, the identity unfed), Out at full strength.
// An unusable layout, mask, base or result is an error.
const rigExec::RigExecPortHandle &LayeredSkinPoints();
const rigExec::RigExecPortHandle &LayeredSkinTransforms();
const rigExec::RigExecPortHandle &LayeredSkinIndices();
const rigExec::RigExecPortHandle &LayeredSkinWeights();
const rigExec::RigExecPortHandle &LayeredSkinMask();
const rigExec::RigExecPortHandle &LayeredSkinBase();
const rigExec::RigExecPortHandle &LayeredSkinInputTransform();
const rigExec::RigExecPortHandle &LayeredSkinOut();

} // namespace rigExecOps

#endif
