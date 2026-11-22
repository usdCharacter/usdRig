#ifndef RIGEXEC_OPS_DEFORM_DERIVED_OPS_H
#define RIGEXEC_OPS_DEFORM_DERIVED_OPS_H

/// Derived ops: normals and extent recomputed from final points, blended
/// toward an optional Authored value by an optional Envelope (Envelope
/// needs Authored). The kernels are rigExecMath/normalsLayoutKernel.h and
/// meshBoundsKernel.h, shared with the baked and zero-USD runtimes.
/// USD-free.

#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

namespace rigExecOps {

/// How the authored normals store what the kernel computes per point.
struct NormalsPayload {
    bool perCorner = false;  // one per face corner, via Indices
    bool leftHanded = false; // negated
};

const rigExec::RigExecOpHandle &RecomputeNormals();
/// Two values, min then max, padded by half of Widths (one per point or
/// one for all; any other length pads nothing).
const rigExec::RigExecOpHandle &RecomputeExtent();

// RecomputeNormals: Points (Vec3fArray), Counts and Indices (IntArray),
// optional Authored (Vec3fArray, the laid-out length) and Envelope, Out.
const rigExec::RigExecPortHandle &RecomputeNormalsPoints();
const rigExec::RigExecPortHandle &RecomputeNormalsCounts();
const rigExec::RigExecPortHandle &RecomputeNormalsIndices();
const rigExec::RigExecPortHandle &RecomputeNormalsAuthored();
const rigExec::RigExecPortHandle &RecomputeNormalsEnvelope();
const rigExec::RigExecPortHandle &RecomputeNormalsOut();

// RecomputeExtent: Points, optional Widths (FloatArray), Authored
// (Vec3fArray of 2) and Envelope (2 values), Out.
const rigExec::RigExecPortHandle &RecomputeExtentPoints();
const rigExec::RigExecPortHandle &RecomputeExtentWidths();
const rigExec::RigExecPortHandle &RecomputeExtentAuthored();
const rigExec::RigExecPortHandle &RecomputeExtentEnvelope();
const rigExec::RigExecPortHandle &RecomputeExtentOut();

} // namespace rigExecOps

#endif
