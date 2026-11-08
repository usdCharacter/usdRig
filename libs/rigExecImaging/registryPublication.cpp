// RigExec imaging publication: the combined-generation path. Evaluate the
// sessions at a time, merge their snapshots into one generation carrying the
// frame table, publish it against the store diffing what became dirty, and
// broadcast to the bound chains with _mutex released. The FNV compiled-rig
// signature helpers live here; only this path reads them.
#include "registry.h"

#include "pxr/base/work/utils.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

uint64_t
_HashCompiledRigPart(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= uint64_t(bytes[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

uint64_t
_HashCompiledRigPart(uint64_t hash, const std::string &value)
{
    return _HashCompiledRigPart(hash, value.data(), value.size());
}

} // namespace

bool
RigExecImagingRegistry::_EvaluateSessions(
    RigSessions *sessions, const UsdStageRefPtr &stage, UsdTimeCode time,
    std::shared_ptr<RigExecImagingSnapshot> *snapshot,
    RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr *compiledRig,
    std::vector<std::string> *errors)
{
    auto combined = std::make_shared<RigExecImagingSnapshot>();
    combined->stage = UsdStageWeakPtr(stage);
    combined->sampleTimeIsDefault = time.IsDefault();
    combined->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();

    auto combinedCompiledRig = std::make_shared<
        RigExecBindingResolvingSceneIndex::CompiledRigBinding>();
    uint64_t compiledRigId = 1469598103934665603ULL;
    SdfPath commonAssetRoot;
    bool firstRoot = true;

    for (RigSession &session : *sessions) {
        RigExecImagingSnapshotConstPtr rigSnapshot = session.store->Get();
        if (session.dirty || !rigSnapshot ||
            !rigSnapshot->Describes(stage, time)) {
            const RigExecImagingBridge::PublishResult result =
                session.playback
                    ? session.playback->EvaluateAndPublishResult(time)
                    : session.bridge->EvaluateAndPublishResult(time);
            // A cache hit is not an evaluator pull: the count is what the
            // scrub tests hold to zero over a warmed range.
            if (!result.cacheHit) {
                ++session.evaluationCount;
            }
            if (!result.ok) {
                if (errors) {
                    errors->push_back(
                        "initial/evaluated generation failed for rig " +
                        session.rigPath.GetString());
                }
                return false;
            }
            if (result.compiledRig)
                session.compiledRig = result.compiledRig;
            session.dirty = false;
            // Usually the last reference to the replaced generation.
            WorkMoveDestroyAsync(rigSnapshot);
            rigSnapshot = session.store->Get();
        }
        // No frozen-snapshot refresh here: only warming reads the snapshot,
        // so BuildWarmWork refreshes it lazily at warm-build time. Refreshing
        // eagerly on this per-frame path made every SetTime -- evaluated or
        // cache-hit, drag tick or scrub -- pay a chain-currency walk plus an
        // avar-region digest for state nobody on this path consumes.

        if (!rigSnapshot || !rigSnapshot->Describes(stage, time)) {
            if (errors) {
                errors->push_back(
                    "rig did not publish the requested stage/time: " +
                    session.rigPath.GetString());
            }
            return false;
        }

        // A zero digest is legal.  The bridge normally supplies the compiled rig on
        // its first publication; construct the same payload defensively so a
        // legal zero cannot leave a stage without a compiled rig.
        if (!session.compiledRig) {
            auto rigCompiledRig = std::make_shared<
                RigExecBindingResolvingSceneIndex::CompiledRigBinding>();
            rigCompiledRig->id =
                session.playback ? session.playback->GetCompiledRigSignature()
                                 : session.bridge->GetCompiledRigSignature();
            for (const auto &[path, published] : rigSnapshot->prims) {
                rigCompiledRig->publishedPrims.insert(path);
            }
            session.compiledRig = std::move(rigCompiledRig);
        }

        compiledRigId =
            _HashCompiledRigPart(compiledRigId, session.rigPath.GetString());
        const uint64_t rigCompiledRigId = session.compiledRig->id;
        compiledRigId = _HashCompiledRigPart(compiledRigId, &rigCompiledRigId,
                                             sizeof(rigCompiledRigId));
        combinedCompiledRig->publishedPrims.insert(
            session.compiledRig->publishedPrims.begin(),
            session.compiledRig->publishedPrims.end());
        combined->xformResetPaths.insert(rigSnapshot->xformResetPaths.begin(),
                                         rigSnapshot->xformResetPaths.end());

        // Carried across for the same reason xformResetPaths is: the
        // combined snapshot is BUILT here, field by field, so anything
        // not named is silently dropped. That is how this one was lost
        // the first time -- the bridge recorded every pose weight, the
        // store published them, and the merge quietly left them behind,
        // so RigExecImaging_GetMovedFloats found 0 of 121 with no error
        // anywhere. Two rigs cannot publish the same property path (the
        // prim check below already rejects a shared prim), so a plain
        // insert is right.
        combined->movedFloats.insert(rigSnapshot->movedFloats.begin(),
                                     rigSnapshot->movedFloats.end());
        combined->movedArrays.insert(rigSnapshot->movedArrays.begin(),
                                     rigSnapshot->movedArrays.end());

        if (firstRoot) {
            commonAssetRoot = session.assetRoot;
            firstRoot = false;
        } else if (commonAssetRoot != session.assetRoot) {
            commonAssetRoot = SdfPath();
        }

        // In path order, so a lone rig's prims each land at the end.
        for (const auto &[path, published] : rigSnapshot->prims) {
            const size_t size = combined->prims.size();
            combined->prims.emplace_hint(combined->prims.end(), path,
                                         published);
            if (combined->prims.size() == size) {
                if (errors) {
                    errors->push_back("two rigs publish the same Hydra prim " +
                                      path.GetString() +
                                      "; cross-rig output overlap is "
                                      "not allowed");
                }
                return false;
            }
        }
    }

    // Reserve zero for the registry's no-published-compiled rig state.
    combinedCompiledRig->id = compiledRigId ? compiledRigId : 1;
    combined->assetRoot = commonAssetRoot;
    *snapshot = std::move(combined);
    *compiledRig = std::move(combinedCompiledRig);
    return true;
}

RigExecImagingBridge::PublishResult
RigExecImagingRegistry::_Publish(
    std::shared_ptr<RigExecImagingSnapshot> snapshot,
    const RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr
        &compiledRig)
{
    RigExecImagingBridge::PublishResult result;
    snapshot->generation = ++_generation;
    // Frames on demand: offset reads of this generation's outputs go here.
    snapshot->frameTable = _frameTable;
    // The generation this replaces is freed on a worker: a thousand
    // published prims take longer to destroy than to diff.
    RigExecImagingSnapshotConstPtr replaced = _store->Get();
    result.dirtied = _store->Publish(std::move(snapshot));
    WorkMoveDestroyAsync(replaced);
    if (compiledRig && compiledRig->id != _publishedCompiledRigId) {
        result.compiledRig = compiledRig;
        _publishedCompiledRigId = compiledRig->id;
        _publishedCompiledRig = compiledRig;
    }
    result.ok = true;
    return result;
}

bool
RigExecImagingRegistry::SetTime(UsdTimeCode time)
{
    if (const Ptr routed = _Routed()) {
        return routed->SetTime(time);
    }
    // unique_lock: both broadcasts below run unlocked (see Activate), because
    // the notices they send re-enter this registry through the results
    // index's time trigger.
    std::unique_lock<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_stage) {
        if (_stage && !_authoredGuides.empty()) {
            const auto current = _store->Get();
            if (!_authoredGuidesDirty && current &&
                current->Describes(_stage, time)) {
                return false;
            }
            std::vector<SdfPath> roots;
            for (const auto &entry : _authoredGuides)
                roots.push_back(entry.first);
            auto snapshot = _BuildAuthoredGuides(_stage, roots, time);
            _lastTime = time;
            const auto published = _Publish(std::move(snapshot), nullptr);
            if (_authoredGuides.empty()) {
                TfNotice::Revoke(_changeKey);
                _changeKey = TfNotice::Key();
                _stage.Reset();
                _autoActivated = false;
            }
            lock.unlock();
            _Broadcast(published);
            return false;
        }
        // A plain stage has no evaluator, but its xform previews still
        // compose against the host's displayed frame.
        _lastTime = time;
        return false;
    }
    // Live hand-off: each session's upstream values at this time. A moved
    // value dirties the session; a time move alone cancels no warming (the
    // warm rows fold each frame's own values).
    for (RigSession &session : _sessions) {
        _HandOffUpstreamLocked(&session, time);
    }
    // A publish still dirties every prim in the generation through
    // _Broadcast, even when it republishes content nothing changed -- so a
    // redundant SetTime cost a full Hydra redraw for nothing. In usdview the
    // generation was observed advancing by 2 per frame change while the
    // plugin's Python _OnFrameChanged fired once, which means a second
    // SetTime arrives from somewhere else (UpdatePreview/EndPreview,
    // SetWeightOverlay and _OnObjectsChanged all republish at _lastTime).
    // Rather than chase which, make the redundant one free: nothing dirty
    // plus the same stage/time already published means there is nothing to
    // say. Headless, generation advanced by exactly 1 per SetTime, so this
    // guard is what closes the gap the usdview session showed.
    if (!_readRootsDirty && _publishedCompiledRigId != 0) {
        bool anyDirty = false;
        for (const RigSession &session : _sessions) {
            anyDirty = anyDirty || session.dirty || session.readRootsDirty;
        }
        if (!anyDirty) {
            const RigExecImagingSnapshotConstPtr current = _store->Get();
            if (current && current->Describes(UsdStageWeakPtr(_stage), time)) {
                _lastTime = time;
                return true;
            }
        }
    }
    for (RigSession &session : _sessions) {
        session.settleOverrides.clear();
    }
    if (_readRootsDirty) {
        _RefreshReadRoots();
        // Wider read roots may widen a session's upstream share.
        for (RigSession &session : _sessions) {
            _HandOffUpstreamLocked(&session, time);
        }
    }
    std::shared_ptr<RigExecImagingSnapshot> snapshot;
    RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr compiledRig;
    if (!_EvaluateSessions(&_sessions, _stage, time, &snapshot, &compiledRig,
                           nullptr)) {
        // A rig that can no longer evaluate stops driving the scene.
        // The per-session bridge already cleared its own store on the way
        // out, but this COMBINED generation is the one the scene indices
        // read, and returning here left the last good one current: remove a
        // mover's rigExec:moves and the recompile fails, so the mesh under
        // the driven Xform stayed where the constraint had put it through
        // every later edit and frame change.
        // Replace driven results with authored guides and notify observers.
        _lastTime = time;
        RigExecImagingBridge::PublishResult cleared;
        cleared.ok = true;
        std::vector<SdfPath> roots;
        for (const RigSession &session : _sessions)
            roots.push_back(session.rigPath);
        auto guides = _BuildAuthoredGuides(_stage, roots, time);
        guides->generation = ++_generation;
        RigExecImagingSnapshotConstPtr replaced = _store->Get();
        cleared.dirtied = _store->Publish(std::move(guides));
        WorkMoveDestroyAsync(replaced);
        // The next good generation must re-announce its compiled rig; the one the
        // binding index holds names prims this clear just removed.
        _publishedCompiledRigId = 0;
        _publishedCompiledRig.reset();
        lock.unlock();
        _Broadcast(cleared);
        return false;
    }
    const UsdTimeCode previousTime = _lastTime;
    _lastTime = time;
    // During active playback the sweep jobs the playhead already passed are
    // dead weight: the live path serves those frames now. Neighbors stay --
    // the frames ahead still warm the imminent scrub path. Only a FORWARD
    // step sheds: a backward scrub is heading into exactly the frames at or
    // behind the new playhead, and purging those would strip the warming
    // the scrub is about to use.
    const bool advanced = previousTime.IsNumeric() && time.IsNumeric() &&
                          time.GetValue() > previousTime.GetValue();
    if (advanced && _scheduler) {
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                _scheduler->NotePlaybackAdvanced(session.rigPath, time);
            }
        }
    }
    // Recorded into the first rig's profiler, so one summary holds the whole
    // update: the rig (Imaging.EvaluateAndPublish, per session, inside
    // _EvaluateSessions above) and then the COMBINED generation's publish
    // and the dirty notices that drive Hydra's sync.
    RigExecProfiler *profiler = nullptr;
    if (!_sessions.empty() && !_sessions.front().playback) {
        profiler = _sessions.front().bridge->MutableProfiler();
    }
    RigExecImagingBridge::PublishResult published;
    {
        RigExecProfileScope scope(profiler, "Imaging.CombinedPublish",
                                  "imaging");
        published = _Publish(std::move(snapshot), compiledRig);
    }
    // Frames on demand: an edit, a recompile or moved upstream values make
    // every other frame stale; readers holding them are told below.
    const bool framesReset =
        _frameTable->ResetIfStale(_FrameTableStampLocked());
    lock.unlock();
    {
        RigExecProfileScope scope(profiler, "Imaging.Broadcast", "imaging");
        _Broadcast(published);
    }
    if (framesReset) {
        _BroadcastFramesChanged();
    }
    return true;
}

void
RigExecImagingRegistry::_Broadcast(
    const RigExecImagingBridge::PublishResult &result)
{
    if (!result.ok) {
        return;
    }
    // The chain list is snapshotted under a short lock; the sends go out
    // WITHOUT it. Scene indices call back into this registry -- the results
    // index's time trigger evaluates through IsActiveRigRoot/SetTime -- so
    // every caller unlocks first (see Activate), and this function never
    // assumes the lock is held. A chain whose graph dies between the snapshot
    // and its send simply fails its weak-pointer check and is pruned on the
    // next broadcast.
    // This context's chains only, snapshotted under the directory's leaf
    // lock (which also prunes chains whose graphs were destroyed).
    const std::vector<Chain> chains = _BoundChains();
    for (const Chain &chain : chains) {
        if (result.compiledRig && chain.binding) {
            chain.binding->SetCompiledRigBinding(result.compiledRig);
        }
        if (chain.results) {
            chain.results->NotifyGenerationPublished(result.dirtied);
        }
    }
}

} // namespace rigExec
