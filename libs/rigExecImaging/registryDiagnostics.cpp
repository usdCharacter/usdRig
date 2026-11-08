// RigExec imaging diagnostics surface: the read-only queries over the
// scheduler, bridge and warm-index owners. A facade: it adds no counters,
// no state and no mutex; every number is read from the owner that already
// keeps it. The mutators stay with their owners (RefreshFrameCacheSettings,
// ClearFrameCache and SetWarmRange in registryCache.cpp; the warm setters
// with warming).
#include "registry.h"

#include "registryInternal.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"
#include "rigExec/profiler.h"

#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstddef>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

// Adds \p from's counters into \p into. Gauges (queue depth, running) are
// instantaneous, so they are added only when \p gauges is set -- a stopped
// scheduler has none.
void
_AddSchedulerStats(RigExecBackgroundSchedulerStats *into,
                   const RigExecBackgroundSchedulerStats &from, bool gauges)
{
    if (gauges) {
        into->queuedDepth += from.queuedDepth;
        into->running += from.running;
    }
    into->completed += from.completed;
    into->published += from.published;
    into->declinedInvalid += from.declinedInvalid;
    into->declinedGeneration += from.declinedGeneration;
    into->droppedStale += from.droppedStale;
    into->canceled += from.canceled;
    into->shed += from.shed;
    into->coalesced += from.coalesced;
    into->upgraded += from.upgraded;
    into->declined += from.declined;
    into->declinedFreezeRefused += from.declinedFreezeRefused;
    into->declinedUnsampleable += from.declinedUnsampleable;
    into->declinedD7Exempt += from.declinedD7Exempt;
    into->droppedAtShutdown += from.droppedAtShutdown;
    into->factoryInvocations += from.factoryInvocations;
}

size_t
RigExecImagingRegistry::GetSessionEvaluationCount(const SdfPath &rigPath)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetSessionEvaluationCount(rigPath);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rigPath)
            return session.evaluationCount;
    }
    return 0;
}

RigExecProfiler *
RigExecImagingRegistry::MutableSchedulerProfiler()
{
    if (const Ptr routed = _Routed()) {
        return routed->MutableSchedulerProfiler();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _scheduler ? _scheduler->MutableProfiler() : nullptr;
}

RigExecProfiler *
RigExecImagingRegistry::MutableBridgeProfiler(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->MutableBridgeProfiler(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge) {
            return session.bridge->MutableProfiler();
        }
    }
    return nullptr;
}

RigExecBackgroundSchedulerStats
RigExecImagingRegistry::GetBackgroundStats()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetBackgroundStats();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    RigExecBackgroundSchedulerStats stats = _retiredStats;
    if (_scheduler) {
        _AddSchedulerStats(&stats, _scheduler->Stats(), /* gauges = */ true);
    }
    return stats;
}

void
RigExecImagingRegistry::WaitUntilBackgroundIdle()
{
    if (const Ptr routed = _Routed()) {
        routed->WaitUntilBackgroundIdle();
        return;
    }
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    // Outside _mutex: workers never take it, but idleness can outlast a
    // caller's patience for holding it.
    if (scheduler) {
        scheduler->WaitUntilIdle();
    }
}

RigExecFrameCacheStats
RigExecImagingRegistry::GetFrameCacheStats(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetFrameCacheStats(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig && session.bridge) {
            return session.bridge->GetFrameCacheStats();
        }
    }
    return RigExecFrameCacheStats();
}

RigExecImagingBridge *
RigExecImagingRegistry::GetBridge(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetBridge(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge) {
            return session.bridge.get();
        }
    }
    return nullptr;
}

RigExecBakedPlayback *
RigExecImagingRegistry::GetPlayback(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetPlayback(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && session.playback) {
            return session.playback.get();
        }
    }
    return nullptr;
}

std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>>
RigExecImagingRegistry::GetCompletedKeys(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetCompletedKeys(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge &&
            _warmIndex) {
            return _warmIndex->CompletedKeys(rig);
        }
    }
    return {};
}

bool
RigExecImagingRegistry::WriteProfileSummary(const std::string &path)
{
    if (const Ptr routed = _Routed()) {
        return routed->WriteProfileSummary(path);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "rig\tname\tcategory\tcount\ttotal_ms\tmax_ms\n";
    for (const RigSession &session : _sessions) {
        // Playback records no phases, so it contributes no rows.
        if (session.playback) {
            continue;
        }
        const RigExecRigEvaluator &evaluator = session.bridge->GetEvaluator();
        for (const RigExecProfileSummaryRow &row :
             session.bridge->GetProfiler().Summarize()) {
            out << session.rigPath.GetString() << '\t' << row.name << '\t'
                << row.category << '\t' << row.count << '\t'
                << (row.totalUs / 1000.0) << '\t' << (row.maxUs / 1000.0)
                << '\n';
        }
        session.bridge->MutableProfiler()->Clear();
    }
    return true;
}

} // namespace rigExec
