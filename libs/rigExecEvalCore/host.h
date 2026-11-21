// The host an EvalCore step runs against, and the mapping each front end's
// adapter keeps. EvalCore executes one already-scheduled step: rigExecGraph
// still owns the edges, readiness, invalidation and invocation, and the
// baked program and RigExecRuntime still bind their own inputs and publish
// their own outputs. `Host` is a compile-time storage and math binding, not
// an evaluator vtable: an adapter never starts a traversal, and no
// engine-identity flag selects a path. Internal to RigExec and
// RigExecRuntime, never an exported API.
#ifndef RIGEXEC_EVAL_CORE_HOST_H
#define RIGEXEC_EVAL_CORE_HOST_H

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "rigExecEvalCore/kinds.h"
#include "rigExecEvalCore/step.h"
#include "rigExecMath/rangeRunner.h"

namespace rigExec {
namespace eval {

// The object or part a StepRef does not name yet: the -1 an
// `RigExecBakedStep` or a wire `RigExecWireStep` already carries, kept by an
// adapter until its lift validates the reference.
inline constexpr int32_t kNoStepObject = -1;
inline constexpr int32_t kNoStepPart = -1;

// `StepResult::detail` when the operation had no reason to report one.
inline constexpr int32_t kNoStepDetail = 0;

// A host binds storage and math by nesting Math, Inputs and State and
// spelling the three accessors with exactly these return types. Family
// dispatch headers instantiate their templates against a host and assert
// this, so a mismatch is caught at the call site rather than inside a body.
template <class Host, class = void> struct IsBoundHost {
    static constexpr bool value = false;
};

template <class Host>
struct IsBoundHost<
    Host, std::void_t<typename Host::Math, typename Host::Inputs,
                      typename Host::State,
                      decltype(std::declval<const Host &>().inputs()),
                      decltype(std::declval<Host &>().state()),
                      decltype(std::declval<const Host &>().ranges())>> {
    static constexpr bool value =
        std::is_same<decltype(std::declval<const Host &>().inputs()),
                     const typename Host::Inputs &>::value &&
        std::is_same<decltype(std::declval<Host &>().state()),
                     typename Host::State &>::value &&
        std::is_same<decltype(std::declval<const Host &>().ranges()),
                     const RigExecRangeRunner *>::value;
};

// The binding every family header reads, and what each part may and may not
// do:
//
//   Math     primitive Gf or runtime affine operations only, in the
//            <Point, Double> hook convention of dualQuatKernel.h and
//            pointBlocks.h (Point is the geometry's point type, Double its
//            affine double counterpart): RigExecGfAffineMath and
//            RigExecAffineFrameInputsT over Gf, the RrMat4d/RrVec3d mirror in
//            rigExecRuntime/affineMath.h over the runtime types. Matrices
//            stay row-vector, as in affineFrameKernel.h. Whole operation
//            bodies do not live here, and no host method hides one.
//
//   inputs() immutable borrowed views of facts the front end already
//            validated: index maps, per-kind parameters, declared input
//            handles, base groups, layout, partitions, and the gate facts
//            captured at Build (groupGates, chunkVertexTarget, chunkCap,
//            rangeChains, groupVertexTarget, groupCap). A view never
//            outlives the packet or program it borrows: decoded wire records
//            are pinned at Open and captured statics at Build, so nothing
//            re-decodes per vertex or per frame.
//
//   state()    the existing slot, revision and group result, scratch, memo,
//            counters and versions, addressed by the compiled index the
//            adapter already uses (RigExecGroupState, RigExecPointsRef,
//            RigExecGroupSource, RigExecSparseWalkMemo,
//            RigExecBakedStepCounters). Type-parametric record views and
//            accessors only: never a copy of every point, never a second
//            graph beside rigExecGraph's.
//
//   ranges()   the host's RigExecRangeRunner, or null. Kernels spread a
//            loop with RigExecRunRanges only over independent points, so
//            Range and Whole give the same bits however the slices fall;
//            order-dependent sums stay outside the runner.

// StepRef::kind is a StepKind ordinal from kinds.h, never a new opcode; the
// reserved SnapshotFinals ordinal (8) stays refused by the wire reader, and
// the slot domain Snapshots (25) stays as the schema declares it.
// StepRef::slot binds the graph output the step already writes.
//
// An outcome is the step's own decision, mapped by the adapter:
//   Applied   the body wrote the result into the bound slot;
//   Held      the bound result stands untouched (the skip path's answer);
//   Rejected  the operation ran and refused, with the status semantics it
//             already had - never a malformed file;
//   Invalid   the existing diagnostic path, and never read as success.
// `detail` is the reason the adapter already holds, zero when there is none.

// What the two adapters keep, so a lift changes the body and not the front
// end:
//
//   RigExecBakedRunGeometryStep(RigExecBakedProgramImpl *, RigExecBakedStep *,
//   UsdTimeCode) keeps its void API, samples time on the owning thread before
//   handing CPU work to RunGeometryStep, and keeps its prologue, its skip
//   counters, its GPU selection and the deferred Derived step at the
//   publication boundary (RigExecBakedResolveGpuChains).
//
//   RrRunGeometryStep(RrProgram *, size_t, std::string *) keeps its null,
//   index and table checks, its labeled errors (RrGeoStepLabel) and its bool
//   result. Playback binds the partition the file recorded; it never
//   recomputes one from its own settings.
//
// After a lift, neither wrapper holds a second CPU step switch: the kind,
// object, part and output translate once into a StepRef, and one body runs.
// RigExecBakedSkipGeometryStep and RrSkipGeometryStep stay the adapter's
// skip entry points and publish nothing fabricated.

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_HOST_H
