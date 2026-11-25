#ifndef RIGEXEC_OPS_DEFORM_SKIN_OP_H
#define RIGEXEC_OPS_DEFORM_SKIN_OP_H

/// Skin: points moved by weighted influence transforms, linear blend or
/// scale-aware dual quaternion. The incoming points are the bind pose. The
/// kernels are rigExecMath/skinKernel.h and dualQuatKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

enum class SkinMethod { ClassicLinear, DualQuaternion };

/// What a skin is configured with at compile.
struct SkinPayload {
    SkinMethod method = SkinMethod::ClassicLinear;
    /// Influence slots per point.
    int elementSize = 1;

    bool operator==(const SkinPayload &o) const
    {
        return method == o.method && elementSize == o.elementSize;
    }
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
/// Optional: blends the skinned points back toward Points.
const rigExec::RigExecPortHandle &SkinEnvelope();
const rigExec::RigExecPortHandle &SkinOut();
/// Optional, from SkinMatrices over the same Transforms: the narrowed rows
/// a SIMD linear blend reads, or all four palette ports of a dual
/// quaternion blend. Unfed, the skin folds them itself.
const rigExec::RigExecPortHandle &SkinRows();
const rigExec::RigExecPortHandle &SkinPaletteReal();
const rigExec::RigExecPortHandle &SkinPaletteDual();
const rigExec::RigExecPortHandle &SkinPaletteStretch();
const rigExec::RigExecPortHandle &SkinPaletteRigid();

/// One bind set's skinning matrices, folded once per frame for every skin
/// that reads them: the matrices checked, and the forms Skin's optional
/// ports take for the method.
struct SkinMatricesPayload {
    SkinMethod method = SkinMethod::ClassicLinear;

    bool operator==(const SkinMatricesPayload &o) const
    {
        return method == o.method;
    }
};

const rigExec::RigExecOpHandle &SkinMatrices();

// Influences (Mat4dArray, finite and affine, at least one) in; Matrices,
// Rows (FloatArray, filled on the SIMD path of a linear blend), and the
// dual quaternion palette one entry per matrix as Real, Dual (QuatdArray),
// Stretch (Mat3dArray) and Rigid (BoolArray) out.
const rigExec::RigExecPortHandle &SkinMatricesInfluences();
const rigExec::RigExecPortHandle &SkinMatricesMatrices();
const rigExec::RigExecPortHandle &SkinMatricesRows();
const rigExec::RigExecPortHandle &SkinMatricesReal();
const rigExec::RigExecPortHandle &SkinMatricesDual();
const rigExec::RigExecPortHandle &SkinMatricesStretch();
const rigExec::RigExecPortHandle &SkinMatricesRigid();

} // namespace rigExecOps

#endif
