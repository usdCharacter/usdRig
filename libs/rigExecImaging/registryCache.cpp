// RigExec imaging frame-cache control: the surface that changes what the
// background workers warm and what the frame cache holds -- the settings
// refresh, the fenced clear, a rig's warm range and the warm-index frame
// states. Every mutator acts on its owner's state (the bridge's cache, the
// warm index, the scheduler's generation); none computes a pose.
#include "registry.h"
#include "registryInternal.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"

#include "warmIndex.h"

#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace rigExec {

void
RigExecImagingRegistry::RefreshFrameCacheSettings()
{
    if (const Ptr routed = _Routed()) {
        routed->RefreshFrameCacheSettings();
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.bridge) {
            session.bridge->RefreshFrameCacheSettings();
        }
    }
}

void
RigExecImagingRegistry::ClearFrameCache(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        routed->ClearFrameCache(rig);
        return;
    }
    // The fence is outermost: read the scheduler under a short lock, then
    // hold the fence across the whole cancel-THEN-clear below. Lock order
    // fence, then registry, then scheduler, then cache shards, then the
    // warm index -- never inverted.
    // Held, not borrowed: a Deactivate while the fence is held stops the
    // pool, and the scheduler (and its fence mutex) must outlive the lock.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
    // Cancel in its own scope BEFORE locking for the clear: the registry
    // mutex is non-recursive, so cancel-under-clear-lock self-deadlocks.
    CancelFrameGeneration(rig);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig && session.bridge) {
                session.bridge->ClearFrameCache();
            }
        }
        _warmIndex->ResetRig(rig);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig && session.warmRangeActive &&
                session.bridge)
                _warmIndex->SetRequest(
                    rig, session.warmRange,
                    _scheduler ? _scheduler->CurrentGeneration(rig) : 0,
                    RigExecFrameCacheCompiledRigSignature(
                        session.bridge->GetEvaluator()));
        }
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig) {
                session.fallbackDeclined.clear();
            }
        }
    }
}

bool
RigExecImagingRegistry::SetWarmRange(const SdfPath &rig,
                                     const std::vector<double> &frames)
{
    if (const Ptr routed = _Routed()) {
        return routed->SetWarmRange(rig, frames);
    }
    bool found = false;
    bool repull = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        const auto ranges = [&]() {
            std::vector<std::pair<double, double>> out;
            for (const RigSession &session : _sessions) {
                if (session.warmRangeActive && !session.warmRange.empty()) {
                    out.emplace_back(session.warmRange.front(),
                                     session.warmRange.back());
                }
            }
            return out;
        };
        const UsdStageRefPtr stage = _PreviewStageLocked();
        const RigExecUpstreamPullWindow before =
            _PullWindow(ranges(), stage, _lastTime);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig && !session.playback && session.bridge) {
                session.warmRange.clear();
                for (const double time : frames) {
                    if (std::isfinite(time)) {
                        session.warmRange.push_back(time);
                    }
                }
                std::sort(session.warmRange.begin(), session.warmRange.end());
                session.warmRange.erase(std::unique(session.warmRange.begin(),
                                                    session.warmRange.end()),
                                        session.warmRange.end());
                session.warmRangeActive = true;
                _warmIndex->SetRequest(
                    rig, session.warmRange,
                    _scheduler ? _scheduler->CurrentGeneration(rig) : 0,
                    RigExecFrameCacheCompiledRigSignature(
                        session.bridge->GetEvaluator()));
                session.fallbackDeclined.clear();
                found = true;
                break;
            }
        }
        // Pulled tables reconstruct the window's frames: a moved window
        // makes them stale, and the re-pull below (its serial move cancels
        // the generation) replaces them before any job reads them.
        if (found && !_upstreamTables.empty()) {
            const RigExecUpstreamPullWindow after =
                _PullWindow(ranges(), stage, _lastTime);
            repull = after.lo != before.lo || after.hi != before.hi;
        }
    }
    if (repull) {
        // Unlocked: the pull calls into upstream data sources.
        for (const Chain &chain : _BoundChains()) {
            if (chain.results) {
                chain.results->PullUpstreamInputs();
            }
        }
    }
    return found;
}

std::vector<RigExecWarmFrameState>
RigExecImagingRegistry::GetFrameStates(const SdfPath &rig,
                                       const std::vector<double> &frames)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetFrameStates(rig, frames);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (frames.empty() || frames.size() > 100000) {
        return {};
    }
    const RigSession *session = nullptr;
    for (const RigSession &candidate : _sessions) {
        if (candidate.rigPath == rig && !candidate.playback &&
            candidate.bridge) {
            session = &candidate;
            break;
        }
    }
    if (!session || !_scheduler || !_warmIndex) {
        return {};
    }
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(rig);
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(session->bridge->GetEvaluator());
    return _warmIndex->States(rig, frames, generation, compiledRig);
}

} // namespace rigExec
