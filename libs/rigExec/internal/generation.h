// RigExec generation tokens: the edit fence in front of the per-frame cache.
// A generation is a rig's edit serial. The UI thread takes a cancellation
// token for the current generation at enqueue time, samples the job's input
// vector under it, and the worker polls the token twice: before it runs and
// again before it publishes. A canceled token drops the job's result, and
// the frame it would have warmed simply evaluates live when asked. The token
// source belongs to whoever schedules background evaluation (cancellation.h);
// what lives here is the checking discipline -- which call sites ask, and
// what each answer means -- plus the no-baked-program admission policy, so
// that every producer and consumer of a token answers those questions the
// same way.
// The bump contract: every rig-affecting edit cancels the rig's source, and
// the edit-commit trigger enqueues fresh jobs under the new generation.
// Entering a rig at generation 0 and bumping on every edit is what makes
// "stale" decidable from the token alone, with no frame-level invalidation
// bookkeeping.
#ifndef RIGEXEC_GENERATION_H
#define RIGEXEC_GENERATION_H

#include "rigExec/api/cancellation.h"

namespace rigExec {

/// Whether the job holding \p cancel may run: true while the token is not
/// canceled. Asked by the worker before it evaluates.
///
/// A stale token means an edit landed between enqueue and run. The sampled
/// input vector is still a faithful record of the older edit, but the compiled rig
/// it was sampled for no longer stands, so running would warm a frame nobody
/// will ever ask for under a digest nobody will ever compute. The worker
/// frees its arena and exits; the caller evaluates live instead.
inline bool
RigExecGenerationCheckAtStart(const RigExecCancellationToken &cancel)
{
    return !cancel.IsCanceled();
}

/// Whether the result computed under \p cancel may be published: true
/// when the token survived the run. Asked by the worker after it evaluates
/// and before the result reaches the cache.
///
/// The same predicate as CheckAtStart. Re-checked after the run: an edit
/// during the run drops it.
inline bool
RigExecGenerationCheckBeforePublish(const RigExecCancellationToken &cancel)
{
    return !cancel.IsCanceled();
}

/// Whether a background warming job may be created for a rig whose bake
/// state is \p bakeRefused.
///
/// False for a refusal rig: the dynamic path drives OpenExec against the
/// live stage and cannot run on a worker, so there is nothing a background
/// job could evaluate. The refusal is not an error -- the rig still incidentally
/// memoizes the UI thread's own live evaluations through the same publish
/// path -- but no job is ever enqueued for it, and the frozen worker declines
/// a context carrying the refusal flag even if one somehow arrives.
inline bool
RigExecShouldEnqueueBackgroundJob(bool bakeRefused)
{
    return !bakeRefused;
}

/// Whether the UI thread's own live result for a rig may be memoized into
/// the frame cache. Always true, including for bake refusals: a refusal rig memoizes
/// UI-thread evaluations on the same key with no background fill, so a
/// refusal rig still hits on a repeated frame -- it just never warms one it
/// has not visited.
inline bool
RigExecShouldMemoizeUiThreadResult(bool bakeRefused)
{
    (void)bakeRefused;
    return true;
}

} // namespace rigExec

#endif // RIGEXEC_GENERATION_H
