// RigExec imaging session lifecycle: activation and teardown. A (re-)
// activation prepares a complete replacement rig set — compile, guides,
// upstream shares, the initial evaluation — without touching the published
// state, then commits it in one swap under _mutex, revoking the previous
// stage listener only at the commit, and broadcasts once after unlocking.
// An initial failure still publishes the authored guides; a failed
// reactivation keeps the previous active publication. EnsureActivated is
// the adapter's automatic path: it never revives a host-deactivated
// stage. Deactivate stops the warming pool: the scheduler is
// moved out under _mutex and destroyed (joining its workers) after every
// lock is released. Rig-root noting takes _notedMutex, never _mutex.
#include "registry.h"

#include "registryInternal.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"
#include "rigExecSchema/schemaTokens.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/work/utils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

const TfToken &_tRigRootType = SchemaType::RigExecRoot;

// The warming pool a context owns while it has an active rig. The hook holds
// the index shared: transitions record into shared state past whatever the
// sessions do next, and a shutdown purge during destruction still lands (the
// hook keeps the index alive).
std::shared_ptr<RigExecBackgroundScheduler>
_MakeScheduler(const std::shared_ptr<RigExecWarmFrameIndex> &warmIndex)
{
    auto scheduler = std::make_shared<RigExecBackgroundScheduler>();
    std::shared_ptr<RigExecWarmFrameIndex> index = warmIndex;
    scheduler->SetTransitionHook(
        [index](const RigExecWarmTransition &transition) {
            index->NoteTransition(transition);
        });
    return scheduler;
}

} // namespace

bool
RigExecImagingRegistry::Activate(const UsdStageRefPtr &stage,
                                 const SdfPath &rigPath,
                                 UsdTimeCode initialTime,
                                 std::vector<std::string> *errors)
{
    return _Activate(stage, rigPath, initialTime, errors,
                     /* automatic = */ false);
}

std::shared_ptr<RigExecImagingSnapshot>
RigExecImagingRegistry::_BuildAuthoredGuides(
    const UsdStageRefPtr &stage, const std::vector<SdfPath> &rigPaths,
    UsdTimeCode time)
{
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->stage = UsdStageWeakPtr(stage);
    snapshot->sampleTimeIsDefault = time.IsDefault();
    snapshot->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();
    // Live sessions own the display state: a session without a bridge (a
    // playback) draws through a scratch builder that is not retained, so
    // _authoredGuides stays the guide-only state alone.
    const bool guideOnly = _sessions.empty();
    std::set<SdfPath> retained;
    std::unique_ptr<RigExecImagingBridge> scratch;
    SdfPath commonAssetRoot;
    bool firstRoot = true;
    for (const SdfPath &path : rigPaths) {
        const UsdPrim root = stage->GetPrimAtPath(path);
        if (!root || root.GetTypeName() != _tRigRootType)
            continue;
        RigExecImagingBridge *builder = nullptr;
        for (RigSession &session : _sessions) {
            if (session.rigPath == path && session.bridge) {
                builder = session.bridge.get();
                break;
            }
        }
        if (!builder && guideOnly) {
            auto &bridge = _authoredGuides[path];
            if (!bridge) {
                bridge = std::make_unique<RigExecImagingBridge>(stage, path);
            }
            builder = bridge.get();
            retained.insert(path);
        } else if (!builder) {
            scratch = std::make_unique<RigExecImagingBridge>(stage, path);
            builder = scratch.get();
        }
        builder->AppendAuthoredGuides(time, snapshot.get());
        if (firstRoot) {
            commonAssetRoot = path.GetParentPath();
            firstRoot = false;
        } else if (commonAssetRoot != path.GetParentPath()) {
            commonAssetRoot = SdfPath();
        }
    }
    if (guideOnly) {
        for (auto it = _authoredGuides.begin(); it != _authoredGuides.end();) {
            it = retained.count(it->first) ? std::next(it)
                                           : _authoredGuides.erase(it);
        }
        _authoredGuidesDirty = false;
    }
    snapshot->assetRoot = commonAssetRoot;
    return snapshot;
}

bool
RigExecImagingRegistry::_Activate(const UsdStageRefPtr &stage,
                                  const SdfPath &rigPath,
                                  UsdTimeCode initialTime,
                                  std::vector<std::string> *errors,
                                  bool automatic)
{
    if (stage && UsdStageWeakPtr(stage) != _boundStage) {
        // A context only ever evaluates its own stage. Another stage is
        // forwarded to ITS context (created when it has none), which is
        // exactly what the old singleton's "activate replaces" becomes when
        // every stage has its own registry: nothing here is replaced.
        const Ptr target = ForStage(stage, /* create = */ true);
        if (!target || target.get() == this) {
            if (errors) {
                errors->push_back("no imaging context for the stage");
            }
            return false;
        }
        return target->_Activate(stage, rigPath, initialTime, errors,
                                 automatic);
    }
    // unique_lock, not lock_guard: the broadcast at the end runs WITHOUT the
    // lock (see _Broadcast), because scene indices call back into the
    // registry -- the results index's time trigger evaluates through
    // IsActiveRigRoot/SetTime -- and broadcasting while holding it is a
    // self-deadlock. Same rule SetWeightOverlay documents, applied here.
    std::unique_lock<std::mutex> lock(_mutex);
    if (!stage) {
        if (errors) {
            errors->push_back("no stage to activate");
        }
        return false;
    }
    if (automatic) {
        // EnsureActivated tested these unlocked. Two rigs populating at
        // once both get here; the first commits and the second is a no-op
        // instead of a recompile -- and it never downgrades an explicit
        // activation that landed in between to an automatic one. A host
        // deactivation in between still wins.
        if (!_sessions.empty()) {
            return true;
        }
        if (_deactivatedByHost) {
            return false;
        }
    }

    std::vector<SdfPath> rigPaths;
    if (!rigPath.IsEmpty()) {
        rigPaths.push_back(rigPath);
    } else {
        for (const UsdPrim &prim : stage->Traverse()) {
            if (prim.GetTypeName() == _tRigRootType) {
                rigPaths.push_back(prim.GetPath());
            }
        }
    }
    std::sort(rigPaths.begin(), rigPaths.end());
    rigPaths.erase(std::unique(rigPaths.begin(), rigPaths.end()),
                   rigPaths.end());
    // A (re-)activation never inherits another stage's warming: the records
    // name the old stage's keys. Reset BEFORE the candidates evaluate, so
    // the initial evaluation's own memos survive; a failed activation below
    // leaves uncached records, which re-warm on demand.
    for (const SdfPath &path : rigPaths) {
        _warmIndex->ResetRig(path);
    }
    if (rigPaths.empty()) {
        if (errors) {
            errors->push_back("no RigExecRoot prim found");
        }
        return false;
    }

    // Edit-driven re-evaluation: listen on the source stage so property
    // edits under any active character asset republish at the current time.
    // REGISTERED FIRST, BEFORE Compile(), AND THAT ORDER IS LOAD-BEARING.
    // Tf_NoticeRegistry::_Register PREPENDS its deliverer, so listeners are
    // delivered most-recently-registered FIRST. OpenExec's own
    // ExecUsdSystem::_NoticeListener -- the thing that invalidates every
    // cached computed value when an attribute is authored -- is created by
    // the ExecUsdSystem constructor, which happens inside Compile(). So
    // registering after Compile() would put us AHEAD of exec in delivery
    // order, and _OnObjectsChanged would re-evaluate while exec still held
    // the pre-edit value: every generation published from an edit would be
    // one edit stale, and a second Evaluate in the same dispatch equally so.
    // Registering first puts us at the BACK of the list, which is where a
    // re-evaluating listener belongs: every recompile builds a new
    // ExecUsdSystem that prepends ahead of us again, so this holds for the
    // life of the rig rather than only until the first structural edit.
    // Registering this early also arms _OnObjectsChanged across the whole
    // compile-and-evaluate window, so _assetRoots -- what arms it -- stays
    // empty until the commit below, and a failure retains the previously
    // active stage's listener. Nothing in that window may author to \p stage
    // regardless: _OnObjectsChanged takes the same non-recursive _mutex this
    // function holds. Neither Compile() nor evaluation authors anything by
    // construction (that is what testRigExecNoAuthoring asserts), which is
    // what makes this safe.
    const std::set<SdfPath> previousAssetRoots = _assetRoots;
    _assetRoots.clear();
    TfNotice::Key candidateChangeKey =
        TfNotice::Register(TfCreateWeakPtr(this),
                           &RigExecImagingRegistry::_OnObjectsChanged, stage);

    // Activation is transactional: a rig that fails to compile or evaluate
    // leaves the previously published generation, and the listener feeding
    // it, exactly as they were rather than tearing the viewport down.
    // Keep the previous key registered until commit: re-registering it on
    // rollback would put it ahead of the retained ExecUsdSystem listener,
    // causing the next edit to publish before exec invalidates its cache.
    RigSessions candidate;
    candidate.reserve(rigPaths.size());
    const auto abandon = [&](bool evaluationFailed) {
        const bool hasRoot = std::any_of(
            rigPaths.begin(), rigPaths.end(), [&stage](const SdfPath &path) {
                const UsdPrim prim = stage->GetPrimAtPath(path);
                return prim && prim.GetTypeName() == _tRigRootType;
            });
        if (!_sessions.empty() || !hasRoot) {
            TfNotice::Revoke(candidateChangeKey);
            _assetRoots = previousAssetRoots;
            return;
        }
        // An incomplete rig still has editable authored guides. Evaluation
        // remains failed; only display data enters the snapshot store.
        TfNotice::Revoke(_changeKey);
        _changeKey = candidateChangeKey;
        _stage = stage;
        _lastTime = initialTime;
        _autoActivated = automatic;
        _deactivatedByHost = false;
        _authoredGuidesRequest = rigPath;
        _authoredGuidesRetryOnValues = evaluationFailed;
        for (RigSession &session : candidate) {
            if (session.bridge) {
                _authoredGuides[session.rigPath] = std::move(session.bridge);
            }
        }
        auto snapshot = _BuildAuthoredGuides(stage, rigPaths, initialTime);
        // Same scope and noted-root state a commit leaves: the guide
        // builders own their generated scopes, and the roots this attempt
        // covered are no longer pending eager activation.
        {
            std::lock_guard<std::mutex> notedLock(_notedMutex);
            _notedRigRoots.clear();
        }
        _generatedScopes.clear();
        for (const auto &[path, bridge] : _authoredGuides) {
            _generatedScopes.insert(bridge->GetGeneratedScope());
        }
        for (const Chain &chain : _BoundChains()) {
            if (chain.pruning) {
                chain.pruning->SetOwnedScopes(_generatedScopes);
            }
        }
        const auto published = _Publish(std::move(snapshot), nullptr);
        lock.unlock();
        _Broadcast(published);
    };

    // Prepare a complete replacement without touching the active stage,
    // scene-index scopes, or published store.
    for (const SdfPath &path : rigPaths) {
        const UsdPrim rig = stage->GetPrimAtPath(path);
        if (!rig || rig.GetTypeName() != _tRigRootType) {
            if (errors) {
                errors->push_back("activation path is not a RigExecRoot: " +
                                  path.GetString());
            }
            abandon(false);
            return false;
        }
        RigSession session;
        session.rigPath = path;
        session.assetRoot = path.GetParentPath();
        session.store = std::make_shared<RigExecSnapshotStore>();
        // Baked playback (M2b): a rig naming rigExec:asset plays the
        // file instead of evaluating. An unreadable or unplayable
        // file falls back to live evaluation with a warning -- the
        // schema contract -- rather than failing the activation.
        std::string assetPath;
        if (RigExecPlaybackAssetFor(rig, &assetPath)) {
            auto playback = std::make_unique<RigExecBakedPlayback>(
                stage, path, session.store);
            std::string why;
            if (playback->Open(assetPath, &why)) {
                playback->SetWeightOverlay(_weightOverlay);
                session.playback = std::move(playback);
                session.readRoots = {session.assetRoot};
                session.readRootsDirty = false;
            } else {
                TF_WARN("rigExec: %s names rigExec:asset %s (%s); "
                        "evaluating live",
                        path.GetString().c_str(), assetPath.c_str(),
                        why.c_str());
            }
        }
        if (!session.playback) {
            session.bridge = std::make_unique<RigExecImagingBridge>(
                stage, path, session.store);
            // The combined generation is diffed in _Publish; the
            // session's own changed-leaf sets would go unread.
            session.bridge->SetPublishesDirtied(false);
            session.bridge->SetWarmFrameIndex(_warmIndex);
            if (std::shared_ptr<RigExecFrameCache> cache =
                    session.bridge->GetFrameCache()) {
                // Evictions retire the index's completions: the strip never
                // shows cached for a drained entry. The callback holds the
                // index shared and the rig by value, so a drop for a dead
                // session lands in a live index under a dead rig.
                std::shared_ptr<RigExecWarmFrameIndex> index = _warmIndex;
                const SdfPath rigPath = path;
                cache->SetEvictionCallback(
                    [index, rigPath](
                        const std::vector<RigExecFrameCacheEviction> &evicted) {
                        for (const RigExecFrameCacheEviction &entry : evicted) {
                            if (entry.time.IsNumeric()) {
                                index->NoteEvicted(rigPath, entry.key,
                                                   entry.time.GetValue());
                            }
                        }
                    });
            }
            // A selection made before this rig was activated applies to it: the
            // overlay is a viewer mode, not a property of one bridge. Set before
            // the first evaluation so the initial generation already carries it.
            session.bridge->SetWeightOverlay(_weightOverlay);
            if (!session.bridge->Compile(errors)) {
                if (_sessions.empty()) {
                    _authoredGuides[path] = std::move(session.bridge);
                }
                abandon(false);
                return false;
            }
        }
        candidate.push_back(std::move(session));
    }

    // Upstream values already pulled for this stage stand from the first
    // generation (read roots are not known yet: the asset root decides,
    // and the commit below widens the share).
    if (!_upstreamTables.empty()) {
        const auto found = _upstreamTables.find(_upstreamChain);
        for (RigSession &session : candidate) {
            if (!session.bridge && !session.playback) {
                continue;
            }
            session.upstream = _UpstreamShare(
                found != _upstreamTables.end() ? found->second : nullptr,
                {session.assetRoot});
            _HandOffUpstreamLocked(session, initialTime);
        }
    }
    std::shared_ptr<RigExecImagingSnapshot> initialSnapshot;
    RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr
        initialCompiledRig;
    if (!_EvaluateSessions(candidate, stage, initialTime, initialSnapshot,
                           initialCompiledRig, errors)) {
        abandon(true);
        return false;
    }

    // Commit the fully compiled and evaluated stage in one transaction.
    // Repopulating _assetRoots is what arms _OnObjectsChanged.
    TfNotice::Revoke(_changeKey);
    _changeKey = candidateChangeKey;
    _sessions = std::move(candidate);
    _authoredGuides.clear();
    _authoredGuidesRequest = SdfPath();
    _authoredGuidesRetryOnValues = false;
    _active = !_sessions.empty();
    _deactivatedByHost = false;
    // Who owns the activation's lifetime: the host (explicit), or the
    // library, which releases it with its last bound chain (ReleaseChain).
    _autoActivated = automatic;
    // The warming pool starts with the first activation (a context without
    // an active rig owns no threads) and stops with Deactivate. It serves
    // this context only, so its per-rig-path keys never meet another
    // stage's equal paths.
    if (!_scheduler) {
        _scheduler = _MakeScheduler(_warmIndex);
    }
    // Fresh sessions retire every queued or running job for their rigs:
    // a completion holds its sampled cache, so this drops work, never
    // results, and a re-activation never inherits another stage's warming.
    for (const RigSession &session : _sessions) {
        // Fresh sessions retire in-flight jobs for their rigs -- but only
        // when any are in flight: an idle bump retires nothing and would
        // orphan the initial evaluation's memos, which stamped the
        // pre-commit generation.
        if (_scheduler->InFlightCount(session.rigPath) > 0) {
            _CancelGenerationLocked(session.rigPath);
        }
    }
    _stage = stage;
    {
        // Nested _mutex -> _notedMutex: the documented lock order.
        std::lock_guard<std::mutex> notedLock(_notedMutex);
        _notedRigRoots.clear();
    }
    _generatedScopes.clear();
    _assetRoots.clear();
    for (const RigSession &session : _sessions) {
        _generatedScopes.insert(session.playback
                                    ? session.playback->GetGeneratedScope()
                                    : session.bridge->GetGeneratedScope());
        _assetRoots.insert(session.assetRoot);
    }
    _RefreshReadRoots();
    for (const Chain &chain : _BoundChains()) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes(_generatedScopes);
        }
    }
    _lastTime = initialTime;
    RigExecImagingBridge::PublishResult published =
        _Publish(std::move(initialSnapshot), initialCompiledRig);
    // Unlocked: the commit above is complete, and _Broadcast sends scene
    // index notices whose observers call back into this registry.
    lock.unlock();
    _Broadcast(published);
    return true;
}

bool
RigExecImagingRegistry::EnsureActivated(const UsdStageRefPtr &stage,
                                        UsdTimeCode time)
{
    if (!stage) {
        return false;
    }
    if (UsdStageWeakPtr(stage) != _boundStage) {
        // The stage's own context decides; there is no other stage to
        // refuse on account of.
        const Ptr target = ForStage(stage, /* create = */ true);
        return target && target.get() != this &&
               target->EnsureActivated(stage, time);
    }
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_sessions.empty()) {
            // Already active on this (its only) stage: a no-op.
            return true;
        }
        if (!_authoredGuides.empty())
            return false;
        if (_deactivatedByHost) {
            // The host turned this stage off; only its explicit Activate
            // turns it back on (see the header).
            return false;
        }
    }
    // Outside the lock: Activate takes the same non-recursive mutex (the
    // rule SetWeightOverlay documents). Two rigs populating at once can both
    // reach here; the activations serialize inside _Activate, which
    // re-checks both tests above under the lock, so the second is a no-op.
    // Automatic: the library releases this activation with the last chain
    // bound to this context (ReleaseChain).
    std::vector<std::string> errors;
    if (!_Activate(stage, SdfPath::EmptyPath(), time, &errors,
                   /* automatic = */ true)) {
        // One line, not silent: a broken rig in a batch host (usdrecord)
        // otherwise renders its rest pose with no explanation. During live
        // authoring this fires at most once per root definition -- defining
        // the outputs that complete the rig does not resync the root, so the
        // adapter is not asked again; the host's explicit activation (or the
        // next resync of the root itself) retries.
        TF_WARN("RigExec: automatic activation failed for %s: %s",
                stage ? stage->GetRootLayer()->GetIdentifier().c_str()
                      : "<no stage>",
                errors.empty() ? "unknown error" : errors.front().c_str());
        return false;
    }
    return true;
}

bool
RigExecImagingRegistry::IsActiveRigRoot(const SdfPath &path)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == path) {
            return true;
        }
    }
    return false;
}

bool
RigExecImagingRegistry::IsDisplayedRigRoot(const SdfPath &path)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_authoredGuides.count(path))
        return true;
    for (const RigSession &session : _sessions) {
        if (session.rigPath == path)
            return true;
    }
    return false;
}

void
RigExecImagingRegistry::NoteRigRoot(const SdfPath &rigPath)
{
    if (rigPath.IsEmpty()) {
        return;
    }
    // _notedMutex, never _mutex: the adapter calls this from inside scene
    // index traversals that run while _mutex is held.
    std::lock_guard<std::mutex> lock(_notedMutex);
    _notedRigRoots.insert(rigPath);
}

bool
RigExecImagingRegistry::IsNotedRigRoot(const SdfPath &path)
{
    std::lock_guard<std::mutex> lock(_notedMutex);
    return _notedRigRoots.count(path) != 0;
}

void
RigExecImagingRegistry::Deactivate()
{
    _Deactivate(/* byHost = */ true);
}

bool
RigExecImagingRegistry::IsAutoActivated() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _autoActivated && (!_sessions.empty() || !_authoredGuides.empty());
}

void
RigExecImagingRegistry::_Deactivate(bool byHost)
{
    // Stopped outside every lock, below: the scheduler's destructor joins
    // its workers, and a fenced clear may be holding its fence while it
    // waits for _mutex.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    // unique_lock: the broadcast below runs unlocked (see Activate), because
    // the notices it sends re-enter this registry through the results
    // index's time trigger.
    std::unique_lock<std::mutex> lock(_mutex);
    if (!byHost) {
        // The library releasing an automatic activation: only while it is
        // still automatic (an explicit Activate took it over otherwise) and
        // no chain is bound any more. Tested under _mutex -- the directory
        // lock nests inside it -- so a chain binding meanwhile either shows
        // up here or adopts the cleared state and re-activates through its
        // own population.
        if (!_autoActivated || (_sessions.empty() && _authoredGuides.empty()) ||
            !_BoundChains().empty()) {
            return;
        }
    }
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Key();
    _RetirePreviewIdentityLocked();
    _previewSlots.clear();
    _previewXformDeltas.clear();
    _previewActive = false;
    _previewStage = UsdStageWeakPtr();
    _assetRoots.clear();
    _readRoots.clear();
    _readRootsDirty = false;
    _generatedScopes.clear();
    for (const RigSession &session : _sessions) {
        _warmIndex->ResetRig(session.rigPath);
        if (_scheduler) {
            _scheduler->ResetRig(session.rigPath);
        }
    }
    _sessions.clear();
    _authoredGuides.clear();
    _authoredGuidesDirty = false;
    _authoredGuidesRequest = SdfPath();
    _authoredGuidesRetryOnValues = false;
    _active = false;
    // A library release is not a host decision: a later engine on the same
    // stage activates it again (EnsureActivated). A host deactivation of an
    // inactive context still counts -- it keeps the automatic path off.
    if (byHost) {
        _deactivatedByHost = true;
    }
    _autoActivated = false;
    _stage.Reset();
    // A context without an active rig owns no threads.
    scheduler = std::move(_scheduler);
    {
        // Nested _mutex -> _notedMutex: the documented lock order.
        std::lock_guard<std::mutex> notedLock(_notedMutex);
        _notedRigRoots.clear();
    }
    _publishedCompiledRigId = 0;
    _publishedCompiledRig.reset();
    for (const Chain &chain : _BoundChains()) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes({});
        }
    }
    RigExecImagingBridge::PublishResult cleared;
    cleared.ok = true;
    cleared.dirtied = _store->Publish(nullptr);
    lock.unlock();
    _Broadcast(cleared);
    if (scheduler) {
        // The stopped scheduler's totals stay part of this context's
        // warming counters (GetBackgroundStats): the cache strip's repaint
        // gate and its "completed" label must not fall back to zero on
        // every re-activation. Read as late as possible, unlocked (the
        // scheduler's own lock is a leaf); completions of jobs still
        // running past this point are not counted.
        const RigExecBackgroundSchedulerStats stats = scheduler->Stats();
        std::lock_guard<std::mutex> relock(_mutex);
        _AddSchedulerStats(&_retiredStats, stats, /* gauges = */ false);
    }
    // The last holder joins the workers; running jobs finish, queued ones
    // drop (RigExecBackgroundScheduler's destructor).
    scheduler.reset();
}

} // namespace rigExec
