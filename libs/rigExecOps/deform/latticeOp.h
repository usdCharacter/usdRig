#ifndef RIGEXEC_OPS_DEFORM_LATTICE_OP_H
#define RIGEXEC_OPS_DEFORM_LATTICE_OP_H

/// Lattice: points deformed by a cage. A regular grid displaces each point by
/// the posed nodes' offsets from their grid positions; otherwise the points
/// bind into the rest cage's bound and follow the posed cage. The kernels are
/// rigExecMath/latticeKernel.h.

#include "rigExecMath/latticeKernel.h"
#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

#include <array>

namespace rigExecOps {

/// What a lattice is configured with at compile. settings.regularGrid
/// picks the grid evaluation.
struct LatticePayload {
    std::array<int, 3> divisions = {2, 2, 2};
    rigExec::RigExecLatticeSettings settings;
    /// Runs once over every point instead of group by group.
    bool whole = false;

    bool operator==(const LatticePayload &o) const
    {
        return divisions == o.divisions && settings == o.settings &&
               whole == o.whole;
    }
};

const rigExec::RigExecOpHandle &Lattice();

// Points and PosedCage (Vec3fArray) always. A bound lattice also needs
// RestPoints (one per point) and RestCage; a regular grid reads the
// optional TargetToLattice, LatticeToTarget and CageToLattice (Mat4d,
// unfed identity). Envelope as the point ops. Out. Inputs a mode needs
// and lacks, or a grid the kernel refuses, are an error. A bound lattice's
// cages must hold one point per lattice node and be finite; a rest cage with
// no volume passes the points through. Enabled (Bool, unfed true) false
// passes them through; at most one of Envelope, DefaultWeight (Float) and
// Packet (PointWeights) weighs the result; Written (IntArray, [begin, end)
// pairs) bounds the groups a sparse packet may reach. Evaluation (Int: 0
// bound, 1 regular grid), Divisions and Interpolation (IntArray of three),
// Origin and Spacing (FloatArray of three), Strength (Float) and Mask
// (FloatArray) override the payload's settings when fed. A grid fed
// CageMatrix (Mat4d) maps its points from it, TargetMatrix (Mat4d, unfed
// identity) and PointSpace (Int: 0 local, 1 common), each frame carried by
// CageFrame and TargetFrame (Mat4d, both or neither), instead of the three
// map inputs.
const rigExec::RigExecPortHandle &LatticePoints();
const rigExec::RigExecPortHandle &LatticeRestPoints();
const rigExec::RigExecPortHandle &LatticeRestCage();
const rigExec::RigExecPortHandle &LatticePosedCage();
const rigExec::RigExecPortHandle &LatticeTargetToLattice();
const rigExec::RigExecPortHandle &LatticeLatticeToTarget();
const rigExec::RigExecPortHandle &LatticeCageToLattice();
const rigExec::RigExecPortHandle &LatticeEnvelope();
const rigExec::RigExecPortHandle &LatticeOut();
const rigExec::RigExecPortHandle &LatticeEnabled();
const rigExec::RigExecPortHandle &LatticeDefaultWeight();
const rigExec::RigExecPortHandle &LatticePacket();
const rigExec::RigExecPortHandle &LatticeWritten();
const rigExec::RigExecPortHandle &LatticeEvaluation();
const rigExec::RigExecPortHandle &LatticeDivisions();
const rigExec::RigExecPortHandle &LatticeInterpolation();
const rigExec::RigExecPortHandle &LatticeOrigin();
const rigExec::RigExecPortHandle &LatticeSpacing();
const rigExec::RigExecPortHandle &LatticeStrength();
const rigExec::RigExecPortHandle &LatticeMask();
const rigExec::RigExecPortHandle &LatticeCageMatrix();
const rigExec::RigExecPortHandle &LatticeTargetMatrix();
const rigExec::RigExecPortHandle &LatticePointSpace();
const rigExec::RigExecPortHandle &LatticeCageFrame();
const rigExec::RigExecPortHandle &LatticeTargetFrame();

} // namespace rigExecOps

#endif
