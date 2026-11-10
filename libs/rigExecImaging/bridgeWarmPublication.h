// Background-warm publication: completing a background warming job into
// the frame cache and the optional warm index, never into the viewport's
// snapshot store. Bridge-free on purpose: pool workers outlive sessions,
// so a completion holds the cache without holding the bridge.
#ifndef RIGEXEC_IMAGING_BRIDGE_WARM_PUBLICATION_H
#define RIGEXEC_IMAGING_BRIDGE_WARM_PUBLICATION_H

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"

#include <cstddef>
#include <functional>
#include <memory>

namespace rigExec {

class RigExecWarmFrameIndex;

/// Publishes one background warming completion into \p cache and the
/// optional warm index, never into the snapshot store the viewport reads.
///
/// The generation fence is checked first: a token that is no longer the
/// rig's current one means an edit landed while the job ran, and the result
/// is dropped. An invalid pose is likewise dropped. Either way the frame the
/// job would have warmed simply evaluates live when asked.
///
/// The fence-check, cache insert and optional index completion run under the
/// scheduler's fence mutex: a running job can never pass its
/// check before a generation bump and insert after the fenced clear. The
/// fenced clear (generation-bump + queue-purge + cache-clear + index-reset)
/// holds the same mutex, so a clear-while-warming retires the late
/// completion instead of letting it repopulate the cache. Lock order: fence
/// outermost, then the scheduler mutex, cache shards or warm index. Cache
/// eviction callbacks run outside the shard locks.
/// The UI-thread memoization path needs no fence (the UI thread serializes
/// with itself).
///
/// Free function, not a bridge method, so a worker closure can hold the
/// cache without holding the bridge: sessions (and their bridges) die on
/// Deactivate while pool workers outlive them, and a completion for a dead
/// session must land in a dead cache, not in freed memory.
bool RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig);

/// The fenced, retained-state completion: as above, plus the
/// per-time fence token and the retained handle with its provenance. The
/// fence requires generation AND token match (see
/// RigExecBackgroundScheduler::IsWarmRequestCurrent): an old job for a
/// scoped-purged time drops here and can never overwrite the requeued
/// result. The overload above publishes pose-only under token zero, which
/// matches only a never-purged time. When supplied, \p warmIndex records the
/// successful completion under the same fence so a scoped edit cannot
/// retire the entry between publication and the index update.
bool RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig,
    RigExecWarmFenceToken fenceToken, std::shared_ptr<const void> retained,
    size_t retainedBytes, const RigExecEntryProvenance *provenance,
    RigExecWarmFrameIndex *warmIndex = nullptr);

/// Test-only probe for the deterministic fence-race test: when set, the
/// fenced publish above runs it while holding the fence mutex, after a
/// passing fence-check and before the cache insert. Null in production.
/// Set only while no publish is in flight (tests set it around one driven
/// job); the probe itself must never publish or clear.
void RigExecSetPublishFenceProbeForTesting(std::function<void()> probe);

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_WARM_PUBLICATION_H
