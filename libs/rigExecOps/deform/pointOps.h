#ifndef RIGEXEC_OPS_DEFORM_POINT_OPS_H
#define RIGEXEC_OPS_DEFORM_POINT_OPS_H

/// Point ops: a Vec3fArray in, the same count out, blended back toward the
/// incoming points by an optional per-point Envelope (unfed is 1 at every
/// point; its length must match). The kernels are rigExecMath's.

#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

namespace rigExecOps {

/// One Jacobi pass of Laplacian smoothing over the face-vertex topology's
/// edges. Invalid topology passes the points through.
const rigExec::RigExecOpHandle &Smooth();
/// Uniform scale about the centroid moving the bound volume toward
/// Reference. A degenerate bound passes the points through.
const rigExec::RigExecOpHandle &VolumeCorrect();

// Smooth: Points, Counts and Indices (IntArray), optional Strength (float,
// clamped to [0, 1], unfed 1) and Envelope, Out.
const rigExec::RigExecPortHandle &SmoothPoints();
const rigExec::RigExecPortHandle &SmoothCounts();
const rigExec::RigExecPortHandle &SmoothIndices();
const rigExec::RigExecPortHandle &SmoothStrength();
const rigExec::RigExecPortHandle &SmoothEnvelope();
const rigExec::RigExecPortHandle &SmoothOut();

// VolumeCorrect: Points, Reference (double), optional Strength (float,
// unfed 1) and Envelope, Out.
const rigExec::RigExecPortHandle &VolumeCorrectPoints();
const rigExec::RigExecPortHandle &VolumeCorrectReference();
const rigExec::RigExecPortHandle &VolumeCorrectStrength();
const rigExec::RigExecPortHandle &VolumeCorrectEnvelope();
const rigExec::RigExecPortHandle &VolumeCorrectOut();

} // namespace rigExecOps

#endif
