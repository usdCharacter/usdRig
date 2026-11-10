// Background-warm publication implementation. The fence probe is a
// static of this TU: the fenced publish below runs it while holding the
// fence mutex, so probe and publication must share a translation unit.
#include "bridgeWarmPublication.h"

#include "warmIndex.h"

#include <mutex>
#include <utility>

namespace rigExec {

// The test-only fence probe (see RigExecSetPublishFenceProbeForTesting):
// null in production, set only while no publish is in flight.
static std::function<void()> _sPublishFenceProbe;

bool
RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig)
{
    return RigExecPublishBackgroundCompletion(
        cache, key, time, pose, generation, scheduler, rig,
        RigExecWarmFenceToken(0), nullptr, 0, nullptr);
}

bool
RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig,
    RigExecWarmFenceToken fenceToken, std::shared_ptr<const void> retained,
    size_t retainedBytes, const RigExecEntryProvenance *provenance,
    RigExecWarmFrameIndex *warmIndex)
{
    if (!cache || !pose.valid) {
        return false;
    }
    const auto publish = [&] {
        const bool stored = cache->Publish(key, time, pose, retainedBytes,
                                           retained, provenance);
        if (stored && warmIndex && time.IsNumeric()) {
            warmIndex->NoteCompleted(rig, time.GetValue(), key, generation);
        }
        return stored;
    };
    if (!scheduler) {
        return publish();
    }
    // Fence-check, cache insert and index completion share the fence (plan
    // 3.3): the fenced clear holds the same mutex across bump + purge +
    // clear + reset, so no clear can land between the check and the store.
    // Lock order: fence, then scheduler, cache shards or warm index.
    std::lock_guard<std::mutex> fenceLock(scheduler->FenceMutex());
    if (!scheduler->IsWarmRequestCurrent(rig, generation, time, fenceToken)) {
        return false;
    }
    if (_sPublishFenceProbe) {
        _sPublishFenceProbe();
    }
    return publish();
}

void
RigExecSetPublishFenceProbeForTesting(std::function<void()> probe)
{
    _sPublishFenceProbe = std::move(probe);
}

} // namespace rigExec
