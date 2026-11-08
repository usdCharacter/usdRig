// RigExec imaging warming: the background-warming triggers and their
// budgets. OnEditCommitted and OnIdle resolve each session's standing burst
// route, build the sweep (retired frames prepended ahead of the cursor
// sweep) and enqueue through the scheduler with the sampling stop; OnIdle
// serves frames on demand first and falls back to one live-evaluator fill
// once the workers have drained. The rewarm queue, the per-frame fallback,
// the cursor sweep and the skip/budget accessors live here.
#include "registry.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"
#include "rigExec/parallel.h"
#include "rigExec/profiler.h"

#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

// The nearest-frame-first sweep around \p playhead: closest-first, skipping
// the neighbor band the commit trigger already covers (+-1..N) and the
// playhead itself. 64 frames: one full commit burst (16 neighbors + 64
// sweep) fits the default per-rig in-flight cap with room to spare.
std::vector<UsdTimeCode>
_SweepTimes(UsdTimeCode playhead)
{
    std::vector<UsdTimeCode> times;
    if (playhead.IsDefault() || !std::isfinite(playhead.GetValue())) {
        return times;
    }
    const double center = playhead.GetValue();
    for (int i = kRigExecFrameCacheDefaultNeighborRadius + 1;
         i <= kRigExecFrameCacheDefaultNeighborRadius + 32; ++i) {
        times.push_back(UsdTimeCode(center - double(i)));
        times.push_back(UsdTimeCode(center + double(i)));
    }
    return times;
}

} // namespace

bool
RigExecImagingRegistry::GetWarmBurstUsable(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmBurstUsable(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.standingBurst.usable;
        }
    }
    return false;
}

void
RigExecImagingRegistry::SetWarmBurstPrepSliceMs(double ms)
{
    if (const Ptr routed = _Routed()) {
        routed->SetWarmBurstPrepSliceMs(ms);
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _warmBurstPrepSliceMs = ms;
    // A new slice re-times every verdict: an overrun latched under the
    // old slice unlatches (the next trigger retries the build), while a
    // usable burst stands -- the slice guards rebuild cost, not standing
    // bursts -- and a shape decline, which no slice forgives, stays.
    for (RigSession &session : _sessions) {
        session.burstOverrun = false;
    }
}

double
RigExecImagingRegistry::GetWarmBurstPrepSliceMs()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmBurstPrepSliceMs();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmBurstPrepSliceMs;
}

RigExecWarmSkipReason
RigExecImagingRegistry::GetLastWarmSkipReason(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetLastWarmSkipReason(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.lastWarmSkip;
        }
    }
    return RigExecWarmSkipReason::None;
}

std::string
RigExecImagingRegistry::GetLastWarmSkipDetail(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetLastWarmSkipDetail(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.lastWarmSkipDetail;
        }
    }
    return std::string();
}

void
RigExecImagingRegistry::SetWarmSamplingBudget(size_t maxFactoryInvocations,
                                              double maxMs)
{
    if (const Ptr routed = _Routed()) {
        routed->SetWarmSamplingBudget(maxFactoryInvocations, maxMs);
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _warmSamplingMaxInvocations = maxFactoryInvocations;
    _warmSamplingMaxMs = maxMs;
}

size_t
RigExecImagingRegistry::GetWarmSamplingMaxInvocations()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmSamplingMaxInvocations();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmSamplingMaxInvocations;
}

double
RigExecImagingRegistry::GetWarmSamplingMaxMs()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmSamplingMaxMs();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmSamplingMaxMs;
}

std::vector<UsdTimeCode>
RigExecImagingRegistry::_CursorSweepTimes(const RigSession &session,
                                          UsdTimeCode playhead)
{
    if (!session.warmRangeActive || !session.bridge || !_scheduler ||
        !_warmIndex) {
        return _SweepTimes(playhead);
    }
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(session.rigPath);
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(session.bridge->GetEvaluator());
    const double center = playhead.GetValue();
    std::vector<std::pair<double, double>> scored;
    for (const double time : session.warmRange) {
        // The playhead is never queued, however it arises (the scheduler
        // enforces this too; skipping here keeps the vector honest).
        if (time == center) {
            continue;
        }
        if (!_warmIndex->IsCursorVisitable(session.rigPath, time, generation,
                                           compiledRig)) {
            continue;
        }
        scored.emplace_back(std::abs(time - center), time);
    }
    // Closest first; ties prefer the future (playback warms ahead).
    std::sort(scored.begin(), scored.end(),
              [](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) {
                  if (a.first != b.first) {
                      return a.first < b.first;
                  }
                  return a.second > b.second;
              });
    std::vector<UsdTimeCode> times;
    times.reserve(scored.size());
    for (const std::pair<double, double> &entry : scored) {
        times.emplace_back(entry.second);
    }
    return times;
}

size_t
RigExecImagingRegistry::OnEditCommitted(RigExecFrozenStepRunner runner)
{
    if (const Ptr routed = _Routed()) {
        return routed->OnEditCommitted(runner);
    }
    // Snapshot the rigs and the playhead under a short lock; the triggers
    // below sample outside it (their factories re-resolve each session, so
    // a Deactivate mid-trigger only skips frames, never dangles).
    std::vector<SdfPath> rigs;
    UsdTimeCode playhead;
    // Held across the unlocked trigger below: a Deactivate in between stops
    // the context's pool, and the last holder is what destroys it.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_scheduler) {
            return 0;
        }
        scheduler = _scheduler;
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                rigs.push_back(session.rigPath);
            }
        }
        playhead = _lastTime;
    }
    if (rigs.empty() || playhead.IsDefault() ||
        !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The runner passes through unresolved: BuildWarmWork reads
    // production-ness off the null before resolving it to the production
    // runner itself.
    size_t enqueued = 0;
    for (const SdfPath &rig : rigs) {
        const RigExecFrameGeneration generation =
            scheduler->CurrentGeneration(rig);
        // The session's standing burst: rebuilt when its pins moved,
        // served across ticks while current. Usable takes the cached
        // route; overrun takes the plain per-frame route (null burst),
        // since frames can still sample; shape-declined takes no
        // factory -- no frame could sample. The pointer names session
        // storage, captured for synchronous same-thread use inside the
        // trigger only (no session can activate or deactivate under it).
        RigExecBurstSampleCache *burst = nullptr;
        bool useCachedRoute = false;
        bool allowPlainRoute = false;
        std::vector<UsdTimeCode> sweepTimes;
        RigExecWarmSamplingBudget budget;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (RigSession &session : _sessions) {
                if (session.rigPath == rig && !session.playback &&
                    session.bridge) {
                    burst = _PrepareWarmBurst(&session);
                    useCachedRoute = burst && burst->usable;
                    allowPlainRoute =
                        burst && !burst->usable &&
                        (session.burstOverrun || session.burstUpstreamVaries ||
                         (session.burstDeclined && session.upstream));
                    sweepTimes = _CursorSweepTimes(session, playhead);
                    // Retired frames re-warm FIRST: the
                    // affected-time set replaces the fixed sweep for the
                    // edit, so a release re-warms what it retired before
                    // the cursor wanders on.
                    _PrependPendingRewarm(&session, playhead, &sweepTimes);
                    budget.maxFactoryInvocations = _warmSamplingMaxInvocations;
                    budget.maxMs = _warmSamplingMaxMs;
                    // Stamped after prep: the stop bounds factory sampling,
                    // not the burst prep above (frozen refresh + rebuild),
                    // which runs outside the factory budget under its own
                    // slice. Measured from the tick's start, a prep slower
                    // than the stop would trip it before the first sample on
                    // every tick, and the rig would never warm.
                    budget.startUs = RigExecProfiler::NowUs();
                    break;
                }
            }
        }
        RigExecWarmJobFactory factory;
        if (useCachedRoute || allowPlainRoute) {
            RigExecBurstSampleCache *route = useCachedRoute ? burst : nullptr;
            factory = [this, rig, generation, runner, route](UsdTimeCode time) {
                return BuildWarmWork(rig, time, generation, runner, route);
            };
        }
        enqueued += scheduler->OnEditCommitted(
            rig, playhead, generation, std::move(sweepTimes),
            std::move(factory), kRigExecFrameCacheDefaultNeighborRadius,
            budget);
    }
    return enqueued;
}

bool
RigExecImagingRegistry::_WarmOneFallbackFrame(RigSession *session,
                                              UsdTimeCode playhead)
{
    if (!session || !session->warmRangeActive || session->warmRange.empty() ||
        !session->bridge || session->playback || !_warmIndex || !_scheduler ||
        !session->bridge->GetInteractiveOverrides().empty()) {
        return false;
    }
    // The live evaluator holds this time's upstream values, not the
    // candidate's: a varying source warms through jobs only.
    if (session->upstream && session->upstream->AnyVaries()) {
        return false;
    }
    const uint64_t serial =
        session->bridge->GetEvaluator().GetStageEditSerial();
    if (session->fallbackSerial != serial) {
        session->fallbackSerial = serial;
        session->fallbackDeclined.clear();
    }
    const auto states = _warmIndex->States(
        session->rigPath, session->warmRange,
        _scheduler->CurrentGeneration(session->rigPath),
        RigExecFrameCacheCompiledRigSignature(session->bridge->GetEvaluator()));
    double nearest = std::numeric_limits<double>::infinity();
    double candidate = 0.0;
    bool found = false;
    for (size_t i = 0; i < session->warmRange.size(); ++i) {
        const double time = session->warmRange[i];
        if (_warmIndex->HasRequestPublished(
                session->rigPath, time,
                _scheduler->CurrentGeneration(session->rigPath),
                RigExecFrameCacheCompiledRigSignature(
                    session->bridge->GetEvaluator())))
            continue;
        if (states[i] == RigExecWarmFrameState::Cached ||
            states[i] == RigExecWarmFrameState::Warming ||
            session->fallbackDeclined.count(time)) {
            continue;
        }
        const double distance = std::abs(time - playhead.GetValue());
        if (!found || distance < nearest) {
            candidate = time;
            nearest = distance;
            found = true;
        }
    }
    if (!found) {
        return false;
    }
    ++session->fallbackAttempts;
    ++session->evaluationCount;
    const uint64_t token = _warmIndex->RequestToken(session->rigPath);
    if (!session->bridge->WarmFrameOnCallingThread(UsdTimeCode(candidate))) {
        session->fallbackDeclined.insert(candidate);
    } else {
        RigExecFrameCacheKey key;
        const auto generation = _scheduler->CurrentGeneration(session->rigPath);
        const uint64_t compiledRig = RigExecFrameCacheCompiledRigSignature(
            session->bridge->GetEvaluator());
        if (_warmIndex->FindCachedKey(session->rigPath, candidate, generation,
                                      compiledRig, &key))
            _warmIndex->NoteRequestPublished(session->rigPath, candidate, key,
                                             generation, token);
    }
    return true;
}

size_t
RigExecImagingRegistry::OnIdle(RigExecFrozenStepRunner runner)
{
    if (const Ptr routed = _Routed()) {
        return routed->OnIdle(runner);
    }
    // Frames on demand first: cached frames become entries, the rest join
    // the rewarm lane the sweep below enqueues before anything else.
    ServiceFrameDemand(8, /* allowFallback = */ true);
    std::vector<SdfPath> rigs;
    UsdTimeCode playhead;
    // Held across the unlocked trigger below: a Deactivate in between stops
    // the context's pool, and the last holder is what destroys it.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_scheduler) {
            return 0;
        }
        scheduler = _scheduler;
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                rigs.push_back(session.rigPath);
            }
        }
        playhead = _lastTime;
    }
    if (rigs.empty() || playhead.IsDefault() ||
        !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The runner passes through unresolved: BuildWarmWork reads
    // production-ness off the null before resolving it to the production
    // runner itself.
    size_t enqueued = 0;
    for (const SdfPath &rig : rigs) {
        const RigExecFrameGeneration generation =
            scheduler->CurrentGeneration(rig);
        // The standing burst, as in OnEditCommitted: cached route when
        // usable, plain per-frame route on overrun, no factory on a
        // shape decline.
        RigExecBurstSampleCache *burst = nullptr;
        bool useCachedRoute = false;
        bool allowPlainRoute = false;
        std::vector<UsdTimeCode> sweepTimes;
        RigExecWarmSamplingBudget budget;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (RigSession &session : _sessions) {
                if (session.rigPath == rig && !session.playback &&
                    session.bridge) {
                    burst = _PrepareWarmBurst(&session);
                    useCachedRoute = burst && burst->usable;
                    allowPlainRoute =
                        burst && !burst->usable &&
                        (session.burstOverrun || session.burstUpstreamVaries ||
                         (session.burstDeclined && session.upstream));
                    sweepTimes = _CursorSweepTimes(session, playhead);
                    // Retired frames first here too: without-Qt the idle
                    // tick is the only recurring driver.
                    _PrependPendingRewarm(&session, playhead, &sweepTimes);
                    budget.maxFactoryInvocations = _warmSamplingMaxInvocations;
                    budget.maxMs = _warmSamplingMaxMs;
                    // Stamped after prep: the stop bounds factory sampling,
                    // not the burst prep above (frozen refresh + rebuild),
                    // which runs outside the factory budget under its own
                    // slice. Measured from the tick's start, a prep slower
                    // than the stop would trip it before the first sample on
                    // every tick, and the rig would never warm.
                    budget.startUs = RigExecProfiler::NowUs();
                    break;
                }
            }
        }
        RigExecWarmJobFactory factory;
        if (useCachedRoute || allowPlainRoute) {
            RigExecBurstSampleCache *route = useCachedRoute ? burst : nullptr;
            factory = [this, rig, generation, runner, route](UsdTimeCode time) {
                return BuildWarmWork(rig, time, generation, runner, route);
            };
        }
        enqueued +=
            scheduler->OnIdle(rig, playhead, generation, std::move(sweepTimes),
                              std::move(factory), budget);
    }
    // Workers remain the preferred route. If they made no progress and
    // have drained, fill one outstanding frame through the live evaluator.
    // This covers dynamic rigs and operations whose inputs cannot freeze,
    // without maintaining another operation allowlist. Never run during a
    // preview, when warming is disabled, or for an injected test runner.
    const auto background = scheduler->Stats();
    if (!runner && enqueued == 0 && background.queuedDepth == 0 &&
        background.running == 0 && RigExecParallelEvaluationEnabled()) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_previewActive && _warmSamplingMaxInvocations > 0 &&
            _warmSamplingMaxMs > 0.0) {
            for (RigSession &session : _sessions) {
                // The bridge's stored switch, not the environment.
                if (!session.bridge ||
                    !RigExecBackgroundWarmingEnabled(
                        session.bridge->GetFrameCacheMode(), true)) {
                    continue;
                }
                if (_WarmOneFallbackFrame(&session, playhead)) {
                    break;
                }
            }
        }
    }
    return enqueued;
}

size_t
RigExecImagingRegistry::GetWarmingProgressCount()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmingProgressCount();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    size_t progress = _scheduler ? _scheduler->Stats().factoryInvocations : 0;
    for (const RigSession &session : _sessions) {
        progress += session.fallbackAttempts;
    }
    return progress;
}

void
RigExecImagingRegistry::_NotePendingRewarm(RigSession *session,
                                           const std::vector<double> &times)
{
    if (!session) {
        return;
    }
    // De-duplicated against the standing list; capped past which the
    // overflow is dropped (dirtied frames stay visitable, so the cursor
    // sweep covers them when a range is set, and every frame still
    // evaluates live on visit when none is).
    static constexpr size_t kPendingRewarmCap = 4096;
    for (const double time : times) {
        if (!std::isfinite(time)) {
            continue;
        }
        if (session->pendingRewarm.size() >= kPendingRewarmCap) {
            break;
        }
        if (std::find(session->pendingRewarm.begin(),
                      session->pendingRewarm.end(),
                      time) == session->pendingRewarm.end()) {
            session->pendingRewarm.push_back(time);
        }
    }
}

void
RigExecImagingRegistry::_PrependPendingRewarm(
    RigSession *session, UsdTimeCode playhead,
    std::vector<UsdTimeCode> *sweepTimes)
{
    if (!session || !sweepTimes || session->pendingRewarm.empty() ||
        !session->bridge || !_scheduler || !_warmIndex) {
        return;
    }
    const double center = playhead.IsNumeric() ? playhead.GetValue() : 0.0;
    // One batched read, no sampling on the query path.
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(session->rigPath);
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(session->bridge->GetEvaluator());
    const std::vector<RigExecWarmFrameState> states = _warmIndex->States(
        session->rigPath, session->pendingRewarm, generation, compiledRig);
    std::vector<std::pair<double, double>> scored;
    std::vector<double> keep;
    for (size_t i = 0; i < session->pendingRewarm.size(); ++i) {
        const double time = session->pendingRewarm[i];
        const RigExecWarmFrameState state =
            i < states.size() ? states[i] : RigExecWarmFrameState::Uncached;
        if (state == RigExecWarmFrameState::Cached) {
            continue;
        }
        keep.push_back(time);
        if (state == RigExecWarmFrameState::Warming) {
            continue;
        }
        scored.emplace_back(std::abs(time - center), time);
    }
    session->pendingRewarm.swap(keep);
    std::sort(scored.begin(), scored.end(),
              [](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) {
                  if (a.first != b.first) {
                      return a.first < b.first;
                  }
                  return a.second > b.second;
              });
    std::vector<UsdTimeCode> first;
    first.reserve(scored.size());
    for (const auto &entry : scored) {
        // The playhead is never queued, however it arises (the scheduler
        // enforces this too; skipping here keeps the vector honest).
        if (playhead.IsNumeric() && entry.second == center) {
            continue;
        }
        first.emplace_back(entry.second);
    }
    sweepTimes->insert(sweepTimes->begin(), first.begin(), first.end());
}

} // namespace rigExec
