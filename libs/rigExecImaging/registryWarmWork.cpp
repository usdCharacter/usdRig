// RigExec imaging warm work: the frozen-work job factory and the state it
// prepares. BuildWarmWork is the only factory; it samples on the owner
// thread, refuses loudly (D7Exempt, FreezeRefuse, Unsampleable), binds the
// session's frozen snapshot and chain pins, and hands workers only sampled
// values, retained state, cache/index ownership and per-lane workspace
// leases. The lazy frozen refresh, the standing burst sample cache (pins,
// prep slice, latched overrun/declined verdicts) and the workspace pool
// live here.
#include "registry.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCache.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/frozenContext.h"
#include "rigExec/profiler.h"

#include "pxr/usd/usd/timeCode.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

// A session owns independent worker lanes. A lease protects lazy construction,
// snapshot replacement and the mutable retained execution state.
struct RigExecWarmWorkspacePool {
    struct Lane {
        std::atomic<bool> busy{false};
        std::shared_ptr<const RigExecFrozenProgram> snapshot;
        uint64_t programDigest = 0;
        std::unique_ptr<RigExecFrozenWorkspace> workspace;
    };
    std::array<Lane, kRigExecBackgroundSchedulerMaxWorkers> lanes;

    Lane *Acquire()
    {
        for (Lane &lane : lanes) {
            bool expected = false;
            if (lane.busy.compare_exchange_strong(expected, true))
                return &lane;
        }
        return nullptr;
    }
};

namespace {

// The injected runner, or the production one when none was given. Null used
// to mean "skip every frame"; since the production runner factory exists it
// means "warm for real", which is what the C-API triggers run under.
RigExecFrozenStepRunner
_ProductionRunner(const RigExecFrozenStepRunner &runner)
{
    return runner ? runner : RigExecMakeProductionStepRunner();
}

} // namespace

bool
RigExecImagingRegistry::_AdoptBridgeChainBindings(RigSession *session)
{
    uint64_t generation = 0;
    const RigExecChainSampleBindings *live =
        session->bridge->AcquireChainBindings(&generation);
    if (!live) {
        session->chainBindingsValid = false;
        return false;
    }
    if (!session->chainBindingsValid ||
        session->chainBindingsGeneration != generation) {
        session->chainBindings = *live;
        session->chainBindingsGeneration = generation;
    }
    session->chainBindingsValid = true;
    return true;
}

void
RigExecImagingRegistry::_RefreshFrozenSnapshot(RigSession *session)
{
    if (!session || session->playback || !session->bridge) {
        return;
    }
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    const uint64_t compiledRig = evaluator.GetCompiledRigSignature();
    const uint64_t serial = evaluator.GetStageEditSerial();
    // The pins: the bridge's, which its notice tracking keeps current (a
    // constant edited mid-compiled rig moves no digest, but its notice drops them).
    // Re-verifying a private copy after every edit would re-probe every chain
    // mover on every commit.
    const auto refreshPins = [&]() { _AdoptBridgeChainBindings(session); };
    if (session->frozenValid && session->frozenProgram == program &&
        session->frozenCompiledRig == compiledRig) {
        // Still serial, same program and compiled rig, valid pins: no notice
        // moved the bindings or the avar region since the standing
        // snapshot verified, so there is nothing to re-walk or
        // re-digest. (The burst pins, including overrides, compare
        // after the refresh regardless.) Failed pins still retry
        // every trigger -- only verified pins skip. During
        // notice-free playback this skips the refresh on every tick.
        if (session->frozenSerial == serial && session->chainBindingsValid) {
            return;
        }
        refreshPins();
        if (!program) {
            session->frozen.reset();
            session->frozenPatch.reset();
            session->frozenAvarDigest = 0;
            session->frozenError.clear();
            session->frozenSerial = serial;
            return;
        }
        // Same program, same compiled rig: what may have moved is the avar region
        // (ApplyAvarValueEdits), the inputs a routed value edit marked
        // (ApplyValueEdits), and the program stamp. Capture those values;
        // each worker materializes its copy under its lane lease. The last
        // two move no digest,
        // and the snapshot's history predates them either way, so they are
        // asked of the snapshot itself. A refused patch (a shape change
        // wearing a patch's clothes) falls through to a re-freeze.
        const uint64_t avarDigest = RigExecFrozenAvarRegionDigest(*program);
        if (avarDigest == session->frozenAvarDigest &&
            !(session->frozen &&
              RigExecFrozenSnapshotOwesLiveEdits(*session->frozen, *program))) {
            session->frozenSerial = serial;
            return;
        }
        if (session->frozen) {
            std::shared_ptr<const RigExecFrozenAvarPatch> patched;
            std::string ignored;
            if (RigExecCaptureFrozenAvarPatch(*session->frozen, *program,
                                              &patched, &ignored)) {
                session->frozenPatch = std::move(patched);
                session->frozenAvarDigest = avarDigest;
                session->frozenSerial = serial;
                return;
            }
        }
    }
    // A new program, a new compiled rig, or a patch that refused: re-freeze from
    // the live state. A refusal parks a validly empty entry -- production
    // jobs decline and the frame evaluates live when asked.
    session->frozen.reset();
    session->frozenPatch.reset();
    session->frozenProgram = program;
    session->frozenCompiledRig = compiledRig;
    session->frozenAvarDigest = 0;
    session->frozenValid = true;
    session->frozenSerial = serial;
    refreshPins();
    if (!program) {
        session->frozenError.clear();
        return;
    }
    session->frozenAvarDigest = RigExecFrozenAvarRegionDigest(*program);
    std::shared_ptr<const RigExecFrozenProgram> fresh;
    std::string frozenError;
    if (RigExecFreezeProgram(evaluator, &fresh, &frozenError)) {
        session->frozen = std::move(fresh);
        session->frozenError.clear();
    } else {
        session->frozenError = std::move(frozenError);
    }
}

RigExecBurstSampleCache *
RigExecImagingRegistry::_PrepareWarmBurst(RigSession *session)
{
    if (!session || session->playback || !session->bridge) {
        return nullptr;
    }
    RigExecProfileScope prepScope(session->bridge->MutableProfiler(),
                                  "Imaging.PrepareWarmBurst", "imaging");
    _RefreshFrozenSnapshot(session);
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        session->standingBurst.Clear();
        session->burstProgram = nullptr;
        session->burstCompiledRigSignature = 0;
        session->burstSerial = ~uint64_t(0);
        session->burstOverrides.clear();
        session->burstOverrun = false;
        session->burstDeclined = false;
        // Session-level verdict: no factory means no per-frame verdicts,
        // so prep names the cause instead of spinning silently.
        session->lastWarmSkip = RigExecWarmSkipReason::D7Exempt;
        session->lastWarmSkipDetail = "no baked program (D7)";
        return &session->standingBurst;
    }
    // No burst while an upstream source varies: a burst samples every frame
    // under one upstream list, and each job needs its own time's values, so
    // those frames take the plain per-frame route (no skip is recorded).
    session->burstUpstreamVaries =
        session->upstream && session->upstream->AnyVaries();
    if (session->burstUpstreamVaries) {
        session->standingBurst.Clear();
        session->burstProgram = nullptr;
        session->burstSerial = ~uint64_t(0);
        session->burstUpstreamSerial = ~uint64_t(0);
        session->burstOverrun = false;
        session->burstDeclined = false;
        return &session->standingBurst;
    }
    // The uniform values every frame of the burst samples under.
    std::vector<RigExecUpstreamValue> upstream;
    if (session->upstream) {
        session->upstream->ValuesAt(session->upstream->t0, &upstream);
    }
    const uint64_t compiledRigSignature =
        RigExecFrameCacheCompiledRigSignature(evaluator);
    const uint64_t serial = evaluator.GetStageEditSerial();
    const std::vector<RigExecValueOverride> overrides =
        session->bridge->GetInteractiveOverrides();
    // Pins current and the standing burst usable: serve it, with no
    // rebuild. The burst embeds its own pins copy, built from the bindings
    // generation it records, so a rebind moves it. Include the stage serial:
    // routed value edits can leave the compiled rig unchanged while invalidating
    // the burst's memoized static input values and attribute queries.
    const bool pinsCurrent =
        session->burstProgram == program &&
        session->burstCompiledRigSignature == compiledRigSignature &&
        session->burstSerial == serial &&
        session->burstOverrides == overrides &&
        session->burstUpstreamSerial == session->upstreamSerial &&
        session->burstUpstream == upstream &&
        session->burstChainGeneration == session->chainBindingsGeneration;
    if (pinsCurrent && session->standingBurst.usable) {
        return &session->standingBurst;
    }
    // The refresh above verified the pins, rebinding or invalidating
    // them; valid here means verified-current, and no notice can move
    // them before the burst's last frame samples. A burst is never
    // rebuilt onto unverified pins; the standing pins stay, so the next
    // trigger retries. The Clear parks neither latch flag, so the retry
    // is a rebuild, not a latched serve.
    if (!session->chainBindingsValid) {
        session->standingBurst.Clear();
        session->burstOverrun = false;
        session->burstDeclined = false;
        // Session-level verdict, as above (same words as the factory's).
        session->lastWarmSkip = RigExecWarmSkipReason::Unsampleable;
        session->lastWarmSkipDetail = "chain-bind";
        return &session->standingBurst;
    }
    // A pins-determined unusable outcome under the current pins:
    // re-serve it without paying for the doomed rebuild that produced
    // it. The latch sits past the transient check above, so a failed
    // bindings refresh never latches.
    if (pinsCurrent && (session->burstOverrun || session->burstDeclined)) {
        return &session->standingBurst;
    }
    RigExecBurstSampleCache rebuilt;
    std::string buildError;
    const uint64_t buildStartUs = RigExecProfiler::NowUs();
    const bool built = RigExecBuildBurstSampleCache(
        *program, session->chainBindings, overrides, upstream,
        compiledRigSignature, &rebuilt, &buildError);
    const double buildMs =
        double(RigExecProfiler::NowUs() - buildStartUs) / 1000.0;
    session->burstProgram = program;
    session->burstCompiledRigSignature = compiledRigSignature;
    session->burstSerial = serial;
    session->burstOverrides = overrides;
    session->burstUpstreamSerial = session->upstreamSerial;
    session->burstUpstream = std::move(upstream);
    session->burstChainGeneration = session->chainBindingsGeneration;
    if (buildMs > _warmBurstPrepSliceMs) {
        // Past the slice: park unusable and mark the overrun, so this
        // tick and every tick until the pins move take the plain
        // per-frame route instead of paying for another doomed build.
        session->standingBurst.Clear();
        session->burstOverrun = true;
        session->burstDeclined = false;
    } else {
        session->standingBurst = std::move(rebuilt);
        session->standingBurst.usable = built && session->standingBurst.usable;
        session->burstOverrun = false;
        session->burstDeclined = !session->standingBurst.usable;
        if (session->burstDeclined) {
            // Session-level verdict, as above; the latched serve below
            // re-records nothing (this verdict stands while it latches).
            session->lastWarmSkip = RigExecWarmSkipReason::Unsampleable;
            session->lastWarmSkipDetail = "burst-declined: " + buildError;
        }
    }
    session->bridge->MutableProfiler()->RecordInstant(
        "warmBurstRebuild", "imaging", RigExecProfiler::NowUs(),
        {{"usable", session->standingBurst.usable ? "1" : "0"},
         {"overrun", session->burstOverrun ? "1" : "0"},
         {"ms", std::to_string(buildMs)}});
    return &session->standingBurst;
}

RigExecWarmFactoryResult
RigExecImagingRegistry::BuildWarmWork(const SdfPath &rig, UsdTimeCode time,
                                      RigExecFrameGeneration generation,
                                      RigExecFrozenStepRunner runner,
                                      RigExecBurstSampleCache *burst)
{
    std::unique_lock<std::mutex> lock(_mutex);
    // Production-ness is read BEFORE the runner resolves: a null runner
    // means "warm for real", and real execution needs the session's
    // snapshot, while an injected test runner brings its own answers and
    // runs without one.
    const bool production = !runner;
    runner = _ProductionRunner(runner);
    if (!runner || time.IsDefault() || !std::isfinite(time.GetValue())) {
        return RigExecWarmFactoryResult();
    }
    // Warm-route admission runs before any session or burst pointer is
    // captured: the preview proof reads the stage and releases _mutex for
    // those reads, and a context can activate or deactivate across that
    // window. Interactive overrides are already on the bridge (a preview
    // authors nothing), so the frame the preview stands on has no committed
    // consumer yet and is refused outright. Every other frame keeps its route;
    // work with no prepared burst behind it is built ahead of its destination
    // and must be provably stable against the declared edits.
    RigExecWarmRouteDescriptor route;
    route.rigPath = rig;
    route.time = time;
    route.previewSession = _previewIdentity.session;
    route.previewRevision = _previewIdentity.revision;
    if (_previewActive && time == _lastTime) {
        route.route = RigExecWarmRoute::PendingPreview;
    } else {
        route.route = burst ? RigExecWarmRoute::Committed
                            : RigExecWarmRoute::CommittedSpeculative;
    }
    if (!_AdmitWarmRouteLocked(route, nullptr, lock)) {
        return RigExecWarmFactoryResult();
    }
    // Non-const: a built job records its freshness proof on the bridge.
    RigSession *session = nullptr;
    for (RigSession &candidate : _sessions) {
        if (candidate.rigPath == rig) {
            session = &candidate;
            break;
        }
    }
    // Unknown rigs, playback rigs (a .rigexec never warms), and anything
    // without a bridge take no job: the frame evaluates live when asked.
    if (!session || session->playback || !session->bridge) {
        return RigExecWarmFactoryResult();
    }
    // The burst the trigger handed over names this session's storage; the
    // admission window may have retired it, so it is re-resolved here.
    if (burst) {
        burst = &session->standingBurst;
    }
    // A reasoned skip: records the session's last skip (the named cause a
    // test or the strip reads back) and returns it with the empty work.
    const auto skip = [&](RigExecWarmSkipReason reason,
                          const std::string &detail) {
        session->lastWarmSkip = reason;
        session->lastWarmSkipDetail = detail;
        // A reasoned skip is a non-publish signal toward un-warmable.
        _warmIndex->NoteSkipped(rig, time.GetValue());
        RigExecWarmFactoryResult result;
        result.skip = reason;
        result.detail = detail;
        return result;
    };
    if (!_scheduler || !_scheduler->IsGenerationCurrent(rig, generation)) {
        return RigExecWarmFactoryResult();
    }
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    // The cached route, when the burst prepared one for this rig's
    // program: prepared pins, visit sets, and compiled rig signature, with static
    // reads and digests served from the burst maps. Anything else takes
    // the plain per-frame route below, which re-derives everything
    // exactly as before.
    const std::vector<RigExecValueOverride> overrides =
        session->bridge->GetInteractiveOverrides();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (program == nullptr) {
        // A rig with no baked program evaluates dynamically against
        // the live stage, which a worker must never touch. No job, ever;
        // the rig still memoizes its own UI-thread results.
        return skip(RigExecWarmSkipReason::D7Exempt, "no baked program (D7)");
    }
    // The job time's upstream values, from the session's pulled table only
    // (no data source is called here; a varying array is reconstructed
    // from its samples, with the frame's fold hash, and nothing is written
    // back). A varying source with no value at this time -- outside the
    // window, or a pull still pending -- warms nothing.
    std::vector<RigExecUpstreamValue> upstream;
    if (session->upstream &&
        !session->upstream->ValuesAt(time.GetValue(), &upstream)) {
        return skip(RigExecWarmSkipReason::Unsampleable,
                    "upstream not pulled for this time");
    }
    const bool cached = burst && burst->usable && program == burst->program &&
                        upstream == burst->upstream;
    if (!cached) {
        // Refreshed here -- lazily, at warm-build time -- and not on the
        // per-frame path: only warming reads the snapshot. The live state
        // a refresh pins is the last evaluated frame's either way (no
        // evaluation runs between that frame and this build), so warming
        // sees the same history it did under the eager refresh, including
        // after cache hits, which evaluate nothing on either path.
        _RefreshFrozenSnapshot(session);
        // The session's compiled rig chain bindings, verified at use and
        // rebound when a mid-compiled rig edit moved them. A rig that cannot bind
        // takes no job: the sampler would mark its chains stale and decline
        // anyway, but declining here skips the doomed sample.
        if (!_AdoptBridgeChainBindings(session)) {
            return skip(RigExecWarmSkipReason::Unsampleable, "chain-bind");
        }
    }
    // No snapshot, no job -- checked BEFORE sampling on both routes (the
    // cached route's refresh ran in burst prep), so a refusing rig skips
    // every sweep time without paying for doomed samples. Production
    // execution runs from the session's snapshot; an injected test runner
    // brings its own answers and runs without one.
    if (production && !session->frozen) {
        return skip(RigExecWarmSkipReason::FreezeRefused,
                    session->frozenError.empty() ? "freeze refused"
                                                 : session->frozenError);
    }
    RigExecProfiler *warmProfiler = session->bridge->MutableProfiler();
    uint64_t warmPhaseStart =
        warmProfiler->IsEnabled() ? RigExecProfiler::NowUs() : 0;
    const auto warmPhase = [&](const char *name) {
        if (warmProfiler->IsEnabled()) {
            const uint64_t end = RigExecProfiler::NowUs();
            warmProfiler->Record(name, "imagingWarm", warmPhaseStart, end);
            warmPhaseStart = end;
        }
    };
    RigExecFrameInputs inputs;
    if (cached) {
        if (!RigExecSampleFrameInputsWithBurstCache(evaluator, time, overrides,
                                                    upstream, burst, &inputs)) {
            return skip(RigExecWarmSkipReason::Unsampleable, "burst-sample");
        }
    } else {
        // Sampled here, on the UI thread: the worker receives values, never
        // live state, stage handles, or queries. The chains evaluate through
        // the session's pins -- the same values a fresh bind would read, held
        // to account sample by sample by the equivalence test.
        if (!RigExecSampleFrameInputsWithChainBindings(
                evaluator, time, overrides, upstream, session->chainBindings,
                &inputs)) {
            return skip(RigExecWarmSkipReason::Unsampleable, "chain-sample");
        }
    }
    warmPhase("Warm.SampleInputs");
    // A vector carrying a sample marked stale (viaChain) builds no job; the
    // frame evaluates live when asked and still memoizes through the live
    // path. The samplers mark none: chain reads resolve on the worker.
    if (inputs.HasChainResolvedInputs()) {
        return skip(RigExecWarmSkipReason::Unsampleable, "chain-resolved");
    }
    if (!RigExecControlStateDigestible(inputs, overrides)) {
        return skip(RigExecWarmSkipReason::Unsampleable, "undigestible");
    }
    RigExecFrozenEvalContext context;
    context.compiledRigSignature = evaluator.GetCompiledRigSignature();
    context.generation = generation;
    context.slotCount = program->GetProviderCount();
    context.programDigest =
        context.compiledRigSignature ^
        (uint64_t(program->GetBoundInputCount()) * 1099511628211ull) ^
        (uint64_t(program->GetVaryingInputCount()) * 16777619ull);
    context.varyingInputCount = inputs.values.size();
    uint32_t flags = 0;
    if (evaluator.GetPublishWeightFields()) {
        flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator.GetSolverGuidesEnabled()) {
        flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.flags = flags;
    // The session's snapshot, bound per job; the existence check ran
    // before sampling above (the lock never dropped between, so the
    // session cannot have swapped it out from under this build).
    context.frozen = session->frozen.get();
    if (!session->warmWorkspaces)
        session->warmWorkspaces = std::make_shared<RigExecWarmWorkspacePool>();
    auto workspaces = session->warmWorkspaces;
    RigExecFrameCacheKey key;
    // The same compiled rig half the bridge's lookup computes (compiled rig structure +
    // build count), so a completion lands where the UI thread will look.
    // The context keeps the bare compiled rig: that is what the frozen
    // snapshot pins and cross-checks against. The cached route reads the
    // prepared halves; the digests agree by construction (the same level-1s
    // over the same served values). The constant digest folds beside the
    // sampled one on both routes: the session's frozen avar
    // digest IS the compiled rig constant digest, refreshed by
    // _RefreshFrozenSnapshot above, so a value patch opens a new key
    // namespace while the compiled rig half stands.
    const uint64_t constantDigest = session->frozenAvarDigest;
    uint64_t unfoldedDigest = 0;
    if (cached) {
        key.compiledRigSignature = burst->compiledRigSignature;
        unfoldedDigest =
            RigExecControlStateDigestWithBurstCache(inputs, overrides, burst);
        key.controlDigest =
            RigExecFoldConstantDigest(unfoldedDigest, constantDigest);
    } else {
        key.compiledRigSignature =
            RigExecFrameCacheCompiledRigSignature(evaluator);
        unfoldedDigest = RigExecControlStateDigest(inputs, overrides);
        key.controlDigest =
            RigExecFoldConstantDigest(unfoldedDigest, constantDigest);
    }
    warmPhase("Warm.DigestContext");
    // Recorded at enqueue, on this thread: a warmed entry with no proof is
    // unreachable (lookups serve only under one), and the worker cannot
    // record it -- the bridge is UI-thread state. Every sample is fresh at
    // this time (the checks above), so the digest names fresh values and
    // proves them.
    session->bridge->NoteWarmingEnqueued(time, key.controlDigest,
                                         unfoldedDigest, inputs);
    // Dynamic-override bootstrapping: an override identity the
    // compiled rig index never learned would answer every plan as foreign (all
    // clusters). Admission is exact -- the seeds mirror SetOverrides'
    // placement -- and a no-op for identities already known.
    session->bridge->AdmitOverrideControls(overrides);
    std::shared_ptr<RigExecFrameCache> cache = session->bridge->GetFrameCache();
    RigExecBackgroundScheduler *scheduler = _scheduler.get();
    std::shared_ptr<RigExecWarmFrameIndex> warmIndex = _warmIndex;
    const auto progressPublication = warmIndex->RequestPublicationFor(
        rig, time.GetValue(), generation, key.compiledRigSignature);
    // Null unless explicitly opted in; no worker registry access or lock.
    const WarmPublishObserver publishObserver =
        production ? _warmPublishObserver : WarmPublishObserver();
    // Only sampled values cross the thread boundary, plus the snapshot the
    // context points at: the job's own shared_ptr keeps it alive past a
    // deactivation or a refresh that swaps the session's entry out. The
    // cache is shared (a completion for a dead session lands in a dead
    // cache); the scheduler outlives every job it runs -- it owns the pool,
    // and its destructor (Deactivate stopping the context's pool) joins
    // the workers before it goes.
    std::shared_ptr<const RigExecFrozenProgram> frozen = session->frozen;
    const auto frozenPatch = session->frozenPatch;
    // The source snapshot and dependency record travel with the pose.
    RigExecRetainedFrameState retainedState = RigExecCaptureRetainedState(
        inputs, overrides, key.compiledRigSignature,
        program->GetStepGraph().clustering.clusters.size(), constantDigest);
    warmPhase("Warm.EnqueueProofRetained");
    auto retained =
        std::make_shared<RigExecRetainedFrameState>(std::move(retainedState));
    const size_t retainedBytes = retained->RetainedBytes();
    RigExecEntryProvenance provenance =
        RigExecFullEvalProvenance(program->GetStepGraph(), time);
    provenance.unfoldedControlDigest = unfoldedDigest;
    warmPhase("Warm.Provenance");
    // The retained state holds its own copy, so the job takes the sampled
    // vector itself, immutable from here; nothing below reads `inputs`.
    std::shared_ptr<const RigExecFrameInputs> jobInputs =
        std::make_shared<RigExecFrameInputs>(std::move(inputs));
    RigExecWarmWork work = [jobInputs, context, key, cache, scheduler, rig,
                            runner, frozen, frozenPatch, workspaces, warmIndex,
                            retained, retainedBytes, provenance,
                            progressPublication, publishObserver](
                               const RigExecWarmRequest &request) {
        // Phase marks go to the scheduler's per-run timings (null unless
        // its profiler records); _FinishJob records them as one event.
        RigExecWarmPhaseTimings *const timings = request.timings;
        const auto workerPhase = [timings](const char *name) {
            if (timings)
                timings->Mark(name);
        };
        workerPhase(nullptr);
        RigExecFrozenEvalContext laneContext = context;
        RigExecWarmWorkspacePool::Lane *lane = workspaces->Acquire();
        struct Lease {
            RigExecWarmWorkspacePool::Lane *lane;
            ~Lease()
            {
                if (lane)
                    lane->busy.store(false);
            }
        } lease{lane};
        if (lane) {
            // The leased worker retires its old arena. A value edit must not
            // destroy every lane's potentially large arena on the UI thread.
            if (!lane->workspace || lane->snapshot != frozen ||
                lane->programDigest != context.programDigest) {
                lane->workspace.reset();
                lane->snapshot = frozen;
                lane->programDigest = context.programDigest;
                lane->workspace = RigExecCreateFrozenWorkspace(frozen);
            }
            laneContext.workspace = lane->workspace.get();
        }
        laneContext.avarPatch = frozenPatch.get();
        workerPhase("Warm.WorkerLane");
        const RigExecRigPose pose = RigExecEvaluateFrozen(
            laneContext, *jobInputs, runner, scheduler, rig);
        workerPhase("Warm.WorkerEvaluate");
        // Stale at publish means an edit landed mid-run -- a generation
        // bump or a scoped purge that moved this time's fence token (the
        // frame requeues under the live generation); anything else that
        // fails to publish is a genuine decline (the frame evaluates live
        // when asked). The fence is re-read per failure -- never trusted
        // across the evaluate/publish calls -- so a purge between them
        // still lands DeclinedGeneration.
        const auto stale = [&]() {
            return scheduler && !scheduler->IsWarmRequestCurrent(
                                    rig, request.generation, request.time,
                                    request.fenceToken);
        };
        if (!pose.valid) {
            return stale() ? RigExecWarmOutcome::DeclinedGeneration
                           : RigExecWarmOutcome::DeclinedInvalid;
        }
        const bool pubOk = RigExecPublishBackgroundCompletion(
            cache, key, request.time, pose, request.generation, scheduler, rig,
            request.fenceToken, retained, retainedBytes, &provenance,
            warmIndex.get());
        workerPhase("Warm.WorkerPublish");
        if (!pubOk) {
            return stale() ? RigExecWarmOutcome::DeclinedGeneration
                           : RigExecWarmOutcome::DeclinedInvalid;
        }
        // No new worker mutex: publication is fenced above; this immutable
        // captured cell only marks its own request, never owner request maps.
        progressPublication.Publish(key, request.generation);
        if (publishObserver)
            publishObserver(rig, request.time, key, request.generation, pose,
                            cache);
        return RigExecWarmOutcome::Published;
    };
    warmPhase("Warm.WorkConstruction");
    return RigExecWarmFactoryResult{std::move(work)};
}

void
RigExecImagingRegistry::SetWarmPublishObserverForTesting(
    WarmPublishObserver observer)
{
    _warmPublishObserver = std::move(observer);
}

} // namespace rigExec
