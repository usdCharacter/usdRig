// rigExec's parallel regions: the frozen serial flag, the caller-environment
// parallel loop and the range runner handed to kernels.
#ifndef RIGEXEC_PARALLEL_H
#define RIGEXEC_PARALLEL_H

#include "rigExec/api/parallel.h"
#include "rigExecMath/rangeRunner.h"

#include <cstddef>

namespace rigExec {

/// Whether this thread is running a frozen frame inside a frozen serial
/// scope. Frozen workers take the serial variant of every parallel region
/// (see the per-point kernels in moverGraph.cpp): a background job
/// must never dispatch TBB work past the host's own scheduling, and the
/// host's main thread means the wait differently anyway. Defined by the
/// frozen-context TU; declared here because the kernels gate on it.
bool RigExecFrozenSerialActive();

// RigExecGeometryGrainSize and RigExecGeometryParallelThreshold live in
// rigExecMath/rangeRunner.h, which the runtime shares.

/// Whether this thread's floating-point environment is the IEEE default:
/// round to nearest, no flush-to-zero, no denormals-are-zero. A shortcut that
/// writes down what an arithmetic sequence returns, instead of performing it,
/// is exact only there; a host (a DCC) may set either flag. False where the
/// environment cannot be read.
bool RigExecFloatEnvironmentIsDefault();

/// WorkParallelForN over [0, n) in slices of at least \p grain, every slice
/// run in the CALLING thread's floating-point environment (rounding mode,
/// flush-to-zero, denormals-are-zero), restored afterwards. A loop split this
/// way rounds exactly as the same loop on the calling thread, whatever the
/// host set and whatever the pool's threads carry. No gating: callers decide.
void RigExecParallelForInCallerEnvironment(size_t n, size_t grain,
                                           const RigExecRangeBody &body);

/// The runner rigExec hands to pure kernels (rigExecMath) for their
/// per-element loops: RigExecParallelForInCallerEnvironment when
/// RigExecParallelEvaluationEnabled holds, this thread is not in a frozen
/// serial scope and the loop has at least RigExecGeometryParallelThreshold
/// elements; the calling thread otherwise. Never null.
const RigExecRangeRunner *RigExecGeometryRanges();

} // namespace rigExec

#endif // RIGEXEC_PARALLEL_H
