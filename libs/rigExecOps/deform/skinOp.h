#ifndef RIGEXEC_OPS_DEFORM_SKIN_OP_H
#define RIGEXEC_OPS_DEFORM_SKIN_OP_H

/// Skin: points moved by weighted influence transforms, linear blend or
/// scale-aware dual quaternion. The incoming points are the bind pose. The
/// kernel is rigExecMath/skinKernel.h, shared with the baked and zero-USD
/// runtimes. USD-free.

#include "rigExecOps/library.h"

namespace rigExecOps {

enum class SkinMethod { ClassicLinear, DualQuaternion };

/// What a skin is configured with at compile.
struct SkinPayload {
    SkinMethod method = SkinMethod::ClassicLinear;
    /// Influence slots per point.
    int elementSize = 1;
};

const rigExec::RigExecOpHandle &Skin();

// Points (Vec3fArray), Transforms (Mat4dArray, finite and affine), Indices
// (IntArray) and Weights (FloatArray, finite, non-negative), each
// elementSize per point, Out. An unusable layout or a degenerate dual
// quaternion blend is an error.
const rigExec::RigExecPortHandle &SkinPoints();
const rigExec::RigExecPortHandle &SkinTransforms();
const rigExec::RigExecPortHandle &SkinIndices();
const rigExec::RigExecPortHandle &SkinWeights();
const rigExec::RigExecPortHandle &SkinOut();

} // namespace rigExecOps

#endif
