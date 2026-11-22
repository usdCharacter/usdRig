#ifndef RIGEXEC_OPS_DEFORM_WRINKLE_OP_H
#define RIGEXEC_OPS_DEFORM_WRINKLE_OP_H

/// Wrinkle: guided quasistatic edge constraints that buckle compressed
/// regions of a mesh into wrinkles. The kernel is
/// rigExecMath/wrinkleKernel.h, shared with the baked and zero-USD
/// runtimes. USD-free.

#include "rigExecMath/wrinkleSettings.h"
#include "rigExecOps/library.h"

namespace rigExecOps {

/// What a wrinkle is configured with at compile.
struct WrinklePayload {
    rigExec::RigExecWrinkleSettings settings;
};

const rigExec::RigExecOpHandle &Wrinkle();

// Points, RestPoints (Vec3fArray, one per point), Counts and Indices
// (IntArray, the rest mesh's faces), Envelope (optional FloatArray, one
// per point), Out. Unusable settings, topology or points are an error.
const rigExec::RigExecPortHandle &WrinklePoints();
const rigExec::RigExecPortHandle &WrinkleRest();
const rigExec::RigExecPortHandle &WrinkleCounts();
const rigExec::RigExecPortHandle &WrinkleIndices();
const rigExec::RigExecPortHandle &WrinkleEnvelope();
const rigExec::RigExecPortHandle &WrinkleOut();

} // namespace rigExecOps

#endif
