#ifndef RIGEXEC_OPS_DEFORM_SURFACE_BINDING_OP_H
#define RIGEXEC_OPS_DEFORM_SURFACE_BINDING_OP_H

/// SurfaceBinding: bound points follow a surface through per-point stencils
/// of surface points, each carried by its tangent frame and an offset in
/// it. The kernel is rigExecMath/surfaceBindingKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

/// How a surface binding is configured at compile.
struct SurfaceBindingPayload {
    /// Limit-surface stencils, else polygon ones.
    bool limit = false;
    /// Smooth surface normals, else each polygon's own.
    bool smooth = false;
    /// Frames carries the surface's and the target's frames.
    bool framed = false;

    bool operator==(const SurfaceBindingPayload &o) const
    {
        return limit == o.limit && smooth == o.smooth && framed == o.framed;
    }
};

const rigExec::RigExecOpHandle &SurfaceBinding();

// Points (Vec3fArray); the binding's Vertices, VertexOffsets,
// PolygonOffsets, PointIndices, LimitOffsets and LimitIndices (IntArray),
// BarycentricWeights, BindingWeights, Mask, DeltaMultipliers, LimitWeights,
// LimitDuWeights and LimitDvWeights (FloatArray), Offsets (Vec3fArray),
// Strength and DeltaMultiplier (Float), SurfaceRest, TargetRest and
// SurfaceToBinding (Mat4d); Frames (Mat4dArray of two, fed when framed);
// the bound surface's Surface points (Vec3fArray), Counts and Indices
// (optional IntArray, its faces, read for smooth normals); Out at full
// strength. A whole ranged op. Unusable arrays, stencils, mask, frames or
// results are an error.
const rigExec::RigExecPortHandle &SurfaceBindingPoints();
const rigExec::RigExecPortHandle &SurfaceBindingVertices();
const rigExec::RigExecPortHandle &SurfaceBindingVertexOffsets();
const rigExec::RigExecPortHandle &SurfaceBindingPolygonOffsets();
const rigExec::RigExecPortHandle &SurfaceBindingPointIndices();
const rigExec::RigExecPortHandle &SurfaceBindingLimitOffsets();
const rigExec::RigExecPortHandle &SurfaceBindingLimitIndices();
const rigExec::RigExecPortHandle &SurfaceBindingBarycentricWeights();
const rigExec::RigExecPortHandle &SurfaceBindingBindingWeights();
const rigExec::RigExecPortHandle &SurfaceBindingMask();
const rigExec::RigExecPortHandle &SurfaceBindingDeltaMultipliers();
const rigExec::RigExecPortHandle &SurfaceBindingLimitWeights();
const rigExec::RigExecPortHandle &SurfaceBindingLimitDuWeights();
const rigExec::RigExecPortHandle &SurfaceBindingLimitDvWeights();
const rigExec::RigExecPortHandle &SurfaceBindingOffsets();
const rigExec::RigExecPortHandle &SurfaceBindingStrength();
const rigExec::RigExecPortHandle &SurfaceBindingDeltaMultiplier();
const rigExec::RigExecPortHandle &SurfaceBindingSurfaceRest();
const rigExec::RigExecPortHandle &SurfaceBindingTargetRest();
const rigExec::RigExecPortHandle &SurfaceBindingSurfaceToBinding();
const rigExec::RigExecPortHandle &SurfaceBindingFrames();
const rigExec::RigExecPortHandle &SurfaceBindingSurface();
const rigExec::RigExecPortHandle &SurfaceBindingCounts();
const rigExec::RigExecPortHandle &SurfaceBindingIndices();
const rigExec::RigExecPortHandle &SurfaceBindingOut();

} // namespace rigExecOps

#endif
