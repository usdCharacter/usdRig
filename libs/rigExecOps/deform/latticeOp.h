#ifndef RIGEXEC_OPS_DEFORM_LATTICE_OP_H
#define RIGEXEC_OPS_DEFORM_LATTICE_OP_H

/// Lattice: points deformed by a cage. A regular grid displaces each point
/// by the posed nodes' offsets from their grid positions; otherwise the
/// points bind into the rest cage's bound and follow the posed cage. The
/// kernels are rigExecMath/latticeKernel.h, shared with the baked and
/// zero-USD runtimes. USD-free.

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
};

const rigExec::RigExecOpHandle &Lattice();

// Points and PosedCage (Vec3fArray) always. A bound lattice also needs
// RestPoints (one per point) and RestCage; a regular grid reads the
// optional TargetToLattice, LatticeToTarget and CageToLattice (Mat4d,
// unfed identity). Envelope as the point ops. Out. Inputs a mode needs
// and lacks, or a grid the kernel refuses, are an error.
const rigExec::RigExecPortHandle &LatticePoints();
const rigExec::RigExecPortHandle &LatticeRestPoints();
const rigExec::RigExecPortHandle &LatticeRestCage();
const rigExec::RigExecPortHandle &LatticePosedCage();
const rigExec::RigExecPortHandle &LatticeTargetToLattice();
const rigExec::RigExecPortHandle &LatticeLatticeToTarget();
const rigExec::RigExecPortHandle &LatticeCageToLattice();
const rigExec::RigExecPortHandle &LatticeEnvelope();
const rigExec::RigExecPortHandle &LatticeOut();

} // namespace rigExecOps

#endif
