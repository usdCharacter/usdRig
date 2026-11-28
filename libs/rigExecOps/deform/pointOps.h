#ifndef RIGEXEC_OPS_DEFORM_POINT_OPS_H
#define RIGEXEC_OPS_DEFORM_POINT_OPS_H

/// Point ops: a Vec3fArray in, the same count out, blended back toward the
/// incoming points by an optional per-point Envelope (unfed is 1 at every
/// point; its length must match). The kernels are rigExecMath's.

#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

namespace rigExecOps {

/// What a smooth is configured with at compile.
struct SmoothPayload {
    /// Runs once over every point instead of group by group: its topology
    /// can move per frame.
    bool whole = false;

    bool operator==(const SmoothPayload &o) const
    {
        return whole == o.whole;
    }
};

/// One Jacobi pass of Laplacian smoothing over the face-vertex topology's
/// edges. Invalid topology passes the points through.
const rigExec::RigExecOpHandle &Smooth();
/// Uniform scale about the centroid moving the bound volume toward
/// Reference, or the bound volume of Base. A degenerate bound passes the
/// points through. Runs once over every point.
const rigExec::RigExecOpHandle &VolumeCorrect();

// Smooth: Points, Counts and Indices (IntArray), optional Strength (float,
// clamped to [0, 1], unfed 1) and Envelope, Out. A mover's own inputs come
// as Enabled (Bool, optional; false passes the points through) and
// DefaultWeight (Float, optional; every point's weight, finite and in
// [0, 1] or an error); a weight object's packet as Packet (PointWeights,
// optional; resolved over the points each frame, an error when it does not
// resolve). At most one of Envelope, DefaultWeight and Packet is fed.
const rigExec::RigExecPortHandle &SmoothPoints();
const rigExec::RigExecPortHandle &SmoothCounts();
const rigExec::RigExecPortHandle &SmoothIndices();
const rigExec::RigExecPortHandle &SmoothStrength();
const rigExec::RigExecPortHandle &SmoothEnvelope();
const rigExec::RigExecPortHandle &SmoothOut();
const rigExec::RigExecPortHandle &SmoothEnabled();
const rigExec::RigExecPortHandle &SmoothDefaultWeight();
const rigExec::RigExecPortHandle &SmoothPacket();

/// Blends a full-strength Candidate back toward the entering Points by a
/// mover's weights, as Smooth weighs: an External revision's op computes the
/// candidate, this applies the engine's enabled flag and envelope. A
/// candidate point that is not finite is an error.
const rigExec::RigExecOpHandle &MoverBlend();

// MoverBlend: Candidate and Points (Vec3fArray, pointwise, the candidate
// giving the count), optional Envelope, Enabled, DefaultWeight and Packet
// as Smooth's, Out.
const rigExec::RigExecPortHandle &MoverBlendCandidate();
const rigExec::RigExecPortHandle &MoverBlendPoints();
const rigExec::RigExecPortHandle &MoverBlendEnvelope();
const rigExec::RigExecPortHandle &MoverBlendEnabled();
const rigExec::RigExecPortHandle &MoverBlendDefaultWeight();
const rigExec::RigExecPortHandle &MoverBlendPacket();
const rigExec::RigExecPortHandle &MoverBlendOut();

// VolumeCorrect: Points, one of Reference (double) and Base (Vec3fArray),
// optional Strength (float, unfed 1) and Envelope, Out. Enabled,
// DefaultWeight and Packet as Smooth's; Written (IntArray, optional) the
// ascending [begin, end) pairs a packet may reach.
const rigExec::RigExecPortHandle &VolumeCorrectPoints();
const rigExec::RigExecPortHandle &VolumeCorrectReference();
const rigExec::RigExecPortHandle &VolumeCorrectStrength();
const rigExec::RigExecPortHandle &VolumeCorrectEnvelope();
const rigExec::RigExecPortHandle &VolumeCorrectOut();
const rigExec::RigExecPortHandle &VolumeCorrectBase();
const rigExec::RigExecPortHandle &VolumeCorrectEnabled();
const rigExec::RigExecPortHandle &VolumeCorrectDefaultWeight();
const rigExec::RigExecPortHandle &VolumeCorrectPacket();
const rigExec::RigExecPortHandle &VolumeCorrectWritten();

} // namespace rigExecOps

#endif
