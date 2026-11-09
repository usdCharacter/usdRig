// RigExec imaging edit response: the stage-notice handlers and the
// retirement they drive. Guide-only edits re-activate or republish the
// authored guides; rig edits refresh the read roots, partition the
// frame-cache completions by entry provenance (keep / carry-rekey /
// retire+rewarm) and cancel only the affected times, falling back to the
// unpartitioned per-time cancel when no capture index answers. The rewarm
// queue itself belongs to the warming domain (registryWarming.cpp).
#include "registry.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedClusters.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/outputAffectedIndex.h"
#include "rigExec/profiler.h"
#include "rigExec/tapSet.h"

#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

// Read dependencies may live outside the assets whose outputs we publish.
// Follow authored relationships and attribute connections transitively, also
// inspecting ancestors because provider frames can inherit their inputs.
std::set<SdfPath>
_CollectReadRoots(const UsdStageRefPtr &stage,
                  const std::set<SdfPath> &assetRoots)
{
    std::set<SdfPath> roots = assetRoots;
    std::vector<SdfPath> pending(assetRoots.begin(), assetRoots.end());
    std::set<SdfPath> scannedPrims;
    std::set<SdfPath> scannedSubtrees;
    const auto enqueue = [&](const SdfPath &target) {
        const SdfPath path = target.GetPrimPath();
        if (!path.IsAbsolutePath() || !path.IsPrimPath())
            return;
        roots.insert(path);
        for (const SdfPath &scope : scannedSubtrees) {
            if (path.HasPrefix(scope))
                return;
        }
        pending.push_back(path);
    };
    const auto scan = [&](const UsdPrim &prim) {
        if (!prim || !scannedPrims.insert(prim.GetPath()).second)
            return;
        for (const UsdRelationship &rel : prim.GetRelationships()) {
            SdfPathVector targets;
            rel.GetTargets(&targets);
            for (const SdfPath &target : targets)
                enqueue(target);
        }
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            SdfPathVector connections;
            attr.GetConnections(&connections);
            for (const SdfPath &target : connections)
                enqueue(target);
        }
    };
    while (!pending.empty()) {
        const SdfPath path = pending.back();
        pending.pop_back();
        if (!scannedSubtrees.insert(path).second)
            continue;
        const UsdPrim root = stage->GetPrimAtPath(path);
        if (!root)
            continue;
        for (const UsdPrim &prim : UsdPrimRange(root))
            scan(prim);
        for (UsdPrim parent = root.GetParent();
             parent && !parent.IsPseudoRoot(); parent = parent.GetParent()) {
            scan(parent);
        }
    }
    // Keep only maximal scopes; scanning a notice then costs the number of
    // independent dependency regions, not the number of connected properties.
    std::set<SdfPath> compact;
    for (const SdfPath &path : roots) {
        bool covered = false;
        for (const SdfPath &root : compact) {
            if (path.HasPrefix(root)) {
                covered = true;
                break;
            }
        }
        if (!covered)
            compact.insert(path);
    }
    return compact;
}

} // namespace

void
RigExecImagingRegistry::_RefreshReadRoots()
{
    _readRoots.clear();
    for (RigSession &session : _sessions) {
        if (session.readRootsDirty) {
            session.readRoots = _CollectReadRoots(_stage, {session.assetRoot});
            session.readRootsDirty = false;
        }
        _readRoots.insert(session.readRoots.begin(), session.readRoots.end());
    }
    _readRootsDirty = false;
    if (!_upstreamTables.empty()) {
        for (RigSession &session : _sessions) {
            _UpdateSessionUpstreamLocked(session);
        }
    }
}

RigExecFrameGeneration
RigExecImagingRegistry::CurrentFrameGeneration(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->CurrentFrameGeneration(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_scheduler) {
        return 0;
    }
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return _scheduler->CurrentGeneration(rig);
        }
    }
    return 0;
}

RigExecWarmFenceToken
RigExecImagingRegistry::CurrentFenceToken(const SdfPath &rig, UsdTimeCode time)
{
    if (const Ptr routed = _Routed()) {
        return routed->CurrentFenceToken(rig, time);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_scheduler) {
        return 0;
    }
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return _scheduler->CurrentFenceToken(rig, time);
        }
    }
    return 0;
}

void
RigExecImagingRegistry::_CancelGenerationLocked(const SdfPath &rig)
{
    if (_scheduler) {
        _scheduler->CancelGeneration(rig);
        _warmIndex->NoteGeneration(rig, _scheduler->CurrentGeneration(rig));
    }
}

void
RigExecImagingRegistry::_CancelGenerationTimesLocked(const SdfPath &rig)
{
    if (!_scheduler) {
        return;
    }
    _warmIndex->InvalidateRequest(rig);
    // The unpartitioned affected set: completions the warm index holds
    // plus queued and running times the scheduler holds. No generation
    // bump or index generation push: unaffected times keep their tokens and
    // their running jobs publish normally, while affected times get
    // fresh fence tokens and dirty flags. The partitioned path
    // (_RetireAffectedTimesLocked) narrows this by entry provenance;
    // this stays as its no-oracle fallback.
    std::vector<UsdTimeCode> times = _scheduler->InFlightTimes(rig);
    const std::vector<UsdTimeCode> completed = _warmIndex->CompletedTimes(rig);
    times.insert(times.end(), completed.begin(), completed.end());
    _scheduler->CancelGenerationTimes(rig, times);
    for (const UsdTimeCode &time : times) {
        _warmIndex->NoteDirtied(rig, time.IsDefault() ? 0.0 : time.GetValue());
    }
}

// No-oracle retirement: every completed time is affected. In-flight
// times retire (their provenance publishes later); proofs retire by
// the notice's paths.
void
RigExecImagingRegistry::_RetireAllTimesLocked(
    RigSession &session, const UsdNotice::ObjectsChanged &notice,
    const std::vector<UsdTimeCode> &inFlight)
{
    const SdfPath &rig = session.rigPath;
    _CancelGenerationTimesLocked(rig);
    const std::vector<UsdTimeCode> completed = _warmIndex->CompletedTimes(rig);
    std::vector<double> rewarm;
    for (const UsdTimeCode &time : inFlight) {
        rewarm.push_back(time.GetValue());
    }
    for (const UsdTimeCode &time : completed) {
        rewarm.push_back(time.IsDefault() ? 0.0 : time.GetValue());
    }
    _NotePendingRewarm(session, rewarm);
    session.bridge->RetireProofsForControls(RigExecNoticeControlIds(notice));
}

// The edit's controls: the evaluator's exact patched set for a patch (a
// Patched notice carries only patchable values and ancestors), the full
// adapter for a stamp bump. Each named property stands for itself where
// the index knows it, and brings its prim along where it does not.
std::vector<RigExecControlId>
_NoticeEditControls(const UsdNotice::ObjectsChanged &notice,
                    RigExecNoticeDisposition disposition,
                    const std::vector<SdfPath> &patchedPaths,
                    const RigExecOutputAffectedIndex &index)
{
    std::vector<RigExecControlId> controls;
    if (disposition == RigExecNoticeDisposition::Edited) {
        std::set<RigExecControlId> ids;
        for (const SdfPath &path : patchedPaths) {
            if (path.IsEmpty()) {
                continue;
            }
            const RigExecControlId id = RigExecControlIdForPath(path);
            ids.insert(id);
            if (path.IsPropertyPath() && !index.IsKnownControl(id)) {
                ids.insert(RigExecControlIdForPath(path.GetPrimPath()));
            }
        }
        controls.assign(ids.begin(), ids.end());
    } else if (disposition == RigExecNoticeDisposition::Patched) {
        std::set<RigExecControlId> ids;
        for (const SdfPath &path : patchedPaths) {
            if (path.IsEmpty()) {
                continue;
            }
            ids.insert(path.GetString());
            if (path.IsPropertyPath()) {
                ids.insert(path.GetPrimPath().GetString());
            }
        }
        controls.assign(ids.begin(), ids.end());
    } else {
        controls = RigExecNoticeControlIds(notice);
    }
    return controls;
}

// Every path an edit could reach: resynced and changed paths, each
// property also standing for its prim, plus resolved asset paths.
std::vector<SdfPath>
_NoticeEditPaths(const UsdNotice::ObjectsChanged &notice)
{
    std::vector<SdfPath> noticePaths;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        noticePaths.push_back(path);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        noticePaths.push_back(path);
        if (path.IsPropertyPath()) {
            noticePaths.push_back(path.GetPrimPath());
        }
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        noticePaths.push_back(path);
    }
    return noticePaths;
}

// Whether a warmed entry's recorded provenance reaches the edit, by the
// executor's own dirtiness rule (bakedSchedule). Constant regions are
// recorded but deliberately NOT consulted: the patchable binding's
// compose-cluster cone is the exact oracle.
bool
_EntryAffectedByEdit(const RigExecEntryProvenance &provenance,
                     const RigExecBakedClusterSet &dirty,
                     const std::vector<SdfPath> &noticePaths)
{
    for (const int cluster : provenance.clusters) {
        // Bounded by hand: ClusterSet::Test guards negative
        // only, and provenance is publisher-controlled.
        if (cluster >= 0 && size_t(cluster) < dirty.words.size() * 64 &&
            dirty.Test(cluster)) {
            return true;
        }
    }
    for (size_t i = 0; i < provenance.weightReads.size(); ++i) {
        const SdfPath weight(provenance.weightReads[i]);
        if (weight.IsEmpty()) {
            continue;
        }
        for (const SdfPath &path : noticePaths) {
            if (path == weight || path.HasPrefix(weight)) {
                return true;
            }
        }
    }
    return false;
}

void
RigExecImagingRegistry::_RetireAffectedTimesLocked(
    RigSession &session, const UsdNotice::ObjectsChanged &notice,
    RigExecNoticeDisposition disposition,
    const std::vector<SdfPath> &patchedPaths)
{
    if (!session.bridge || session.playback || !_scheduler || !_warmIndex) {
        return;
    }
    const SdfPath &rig = session.rigPath;
    RigExecProfileScope retireScope(session.bridge->MutableProfiler(),
                                    "Notice.RetireAffectedTimes", "notice");
    _warmIndex->InvalidateRequest(rig);
    RigExecImagingBridge &bridge = *session.bridge;
    // Snapshot in-flight work before completions: a first-fill job that
    // finishes between the reads must be covered by at least one set.
    const std::vector<UsdTimeCode> inFlight = _scheduler->InFlightTimes(rig);
    const RigExecBakedProgram *program =
        bridge.GetEvaluator().GetBakedProgram();
    const RigExecOutputAffectedIndex *index = [&]() {
        RigExecProfileScope scope(bridge.MutableProfiler(),
                                  "Notice.AffectedIndex", "notice");
        return bridge.GetAffectedIndex();
    }();
    if (!program || !index || index->Empty()) {
        // No oracle: every completed time is affected, as before.
        _RetireAllTimesLocked(session, notice, inFlight);
        return;
    }
    const std::vector<RigExecControlId> controls =
        _NoticeEditControls(notice, disposition, patchedPaths, *index);
    const RigExecBakedClusterSet dirty = index->AffectedByControls(controls);
    // Weight-object edits: a notice path at or under a weight object an
    // entry read (WeightPacket steps resolve through step.object, outside
    // the avar-only index, so the provenance read set is the oracle).
    const std::vector<SdfPath> noticePaths = _NoticeEditPaths(notice);
    const uint64_t newConstants = [&]() {
        RigExecProfileScope scope(bridge.MutableProfiler(),
                                  "Notice.ConstantDigest", "notice");
        return RigExecCompiledRigConstantDigest(*program);
    }();
    const std::shared_ptr<RigExecFrameCache> cache = bridge.GetFrameCache();
    const std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>> done =
        _warmIndex->CompletedKeys(rig);
    std::vector<UsdTimeCode> purge = inFlight;
    std::vector<double> rewarm;
    for (const UsdTimeCode &time : inFlight) {
        rewarm.push_back(time.GetValue());
    }
    for (const auto &entry : done) {
        RigExecProfileScope entryScope(bridge.MutableProfiler(),
                                       "Notice.RetireEntry", "notice");
        const UsdTimeCode time = entry.first;
        const RigExecFrameCacheKey &oldKey = entry.second;
        const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
        RigExecEntryProvenance provenance;
        bool affected = true;
        if (cache && cache->LookupProvenance(oldKey, &provenance) &&
            provenance.unfoldedControlDigest != 0) {
            affected = _EntryAffectedByEdit(provenance, dirty, noticePaths);
        }
        if (!affected) {
            // Lane a/b: the entry's computation touches nothing dirty.
            // Across a namespace move it carries (re-keyed, re-pointed,
            // zero recompute); otherwise it stands untouched -- no purge,
            // no dirty flag, no requeue. A failed carry degrades to
            // lane c rather than stranding the frame.
            if (cache && bridge.CarryEntry(time, oldKey, newConstants)) {
                // The new key holds the row now: old-key jobs queued or
                // running for this time must drop, not resurrect it.
                purge.push_back(time);
                continue;
            }
            if (cache) {
                RigExecEntryProvenance carried;
                if (cache->LookupProvenance(oldKey, &carried) &&
                    carried.unfoldedControlDigest != 0 &&
                    RigExecFoldConstantDigest(carried.unfoldedControlDigest,
                                              newConstants) !=
                        oldKey.controlDigest) {
                    // A namespace move the carry declined (evicted base,
                    // pose-only entry, over-cap re-publish): retire and
                    // re-warm rather than serve stale.
                    purge.push_back(time);
                    rewarm.push_back(stamped);
                    _warmIndex->NoteDirtied(rig, stamped);
                    continue;
                }
            }
            continue;
        }
        // Lane c: retire the time. The stale entry strands under its old
        // key (unreachable by construction -- the digest backstop -- and
        // drained by LRU); the row goes dirty and the frame requeues for
        // a full re-warm. (Cone-granular re-execution waits on slot
        // capture: sources-only retained state plans but cannot run a
        // Partial, so the planner's verdict here would only restate this
        // partition at higher cost.)
        purge.push_back(time);
        rewarm.push_back(stamped);
        _warmIndex->NoteDirtied(rig, stamped);
    }
    // Every in-flight time is included: its provenance may not have
    // published yet, so it cannot safely be classified as unaffected.
    _scheduler->CancelGenerationTimes(rig, purge);
    _NotePendingRewarm(session, rewarm);
    RigExecProfileScope proofScope(bridge.MutableProfiler(),
                                   "Notice.RetireProofs", "notice");
    bridge.RetireProofsForControls(controls);
}

void
RigExecImagingRegistry::CancelFrameGeneration(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        routed->CancelFrameGeneration(rig);
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _CancelGenerationLocked(rig);
}

void
RigExecImagingRegistry::_OnGuideOnlyObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &sender,
    const std::set<SdfPath> &guideRoots, const SdfPath &request,
    UsdTimeCode guideTime, bool automatic, bool retryOnValues)
{
    // Only edits inside or above a guide asset can change what it draws or
    // complete its missing inputs.
    const auto touchesGuides = [&guideRoots](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        for (const SdfPath &root : guideRoots) {
            if (primPath.HasPrefix(root) || root.HasPrefix(primPath)) {
                return true;
            }
        }
        return false;
    };
    bool relevant = false;
    bool structural = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (touchesGuides(path)) {
            relevant = structural = true;
            break;
        }
    }
    if (!structural) {
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (!touchesGuides(path)) {
                continue;
            }
            relevant = true;
            // A value edit to a display attribute (visibility, transform,
            // rest placement, guide styling) cannot complete a rig that
            // failed to compile; any other edit (targets, connections,
            // metadata, other values) can.
            bool displayValue = path.IsPropertyPath() && !retryOnValues;
            if (displayValue) {
                const TfToken &name = path.GetNameToken();
                displayValue =
                    name == UsdGeomTokens->visibility ||
                    name == UsdGeomTokens->purpose ||
                    name == UsdGeomTokens->xformOpOrder ||
                    TfStringStartsWith(name.GetString(), "xformOp:") ||
                    TfStringStartsWith(name.GetString(), "rest:") ||
                    TfStringStartsWith(name.GetString(), "guide:");
            }
            if (displayValue) {
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field != SdfFieldKeys->Default &&
                        field != SdfFieldKeys->TimeSamples &&
                        field != SdfFieldKeys->Spline) {
                        displayValue = false;
                        break;
                    }
                }
            }
            if (!displayValue) {
                structural = true;
                break;
            }
        }
    }
    if (!relevant) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _authoredGuidesDirty = true;
    }
    // Completing missing inputs can restore the ordinary evaluator. A
    // failed retry, or a display-only edit, republishes authored guides
    // with the edited values.
    if (structural) {
        std::vector<std::string> errors;
        if (_Activate(UsdStageRefPtr(sender), request, guideTime, &errors,
                      automatic)) {
            return;
        }
    }
    SetTime(guideTime);
}

void
RigExecImagingRegistry::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &sender)
{
    bool authoredOnly = false;
    bool automatic = false;
    bool retryOnValues = false;
    UsdTimeCode guideTime;
    SdfPath request;
    std::set<SdfPath> guideRoots;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        authoredOnly = _sessions.empty() && !_authoredGuides.empty() &&
                       sender == UsdStageWeakPtr(_stage);
        if (authoredOnly) {
            automatic = _autoActivated;
            retryOnValues = _authoredGuidesRetryOnValues;
            guideTime = _lastTime;
            request = _authoredGuidesRequest;
            for (const auto &entry : _authoredGuides) {
                guideRoots.insert(entry.first.GetParentPath());
            }
        }
    }
    if (authoredOnly) {
        _OnGuideOnlyObjectsChanged(notice, sender, guideRoots, request,
                                   guideTime, automatic, retryOnValues);
        return;
    }
    RigExecTapSet::PrepareStageChange(UsdStageRefPtr(sender), notice);
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    // Retiring cache rows and fencing their running jobs is atomic with
    // worker cache/index publication. The fence precedes the registry
    // lock, and is released before publishing any scene-index notices.
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
    std::set<SdfPath> readRoots;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || _assetRoots.empty() ||
            sender != UsdStageWeakPtr(_stage)) {
            return;
        }
        readRoots = _readRoots;
        // Pose-only keys include every stage edit. Retire their display
        // rows even when this notice misses the rig's known input roots.
        // A playback re-reads its static inputs after every edit too: one
        // it binds may lie outside them.
        for (RigSession &session : _sessions) {
            if (!session.playback && session.bridge) {
                session.bridge->RetirePoseOnlyCacheEntries();
            }
            if (session.playback) {
                session.playback->NoteStageChanged();
            }
        }
    }
    // Any edit touching the asset or its transitive reads can factor into
    // the final frame (solvers, joints, movers, controls, weights, driver geometry,
    // guide styling): re-evaluate at the current time. The evaluator's
    // compiled rig signature turns structural edits into recompiles; value edits
    // flow through exec invalidation on the shared layers.
    auto touchesInput = [&readRoots](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        for (const SdfPath &root : readRoots) {
            if (primPath.HasPrefix(root) || root.HasPrefix(primPath)) {
                return true;
            }
        }
        return false;
    };
    bool relevant = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (touchesInput(path)) {
            relevant = true;
            break;
        }
    }
    if (!relevant) {
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (touchesInput(path)) {
                relevant = true;
                break;
            }
        }
    }
    if (!relevant) {
        // Resolved-asset resyncs: the evaluator honors them (they break
        // the avar-only fast path and hit the capture index), so the
        // registry scan reads them too.
        for (const SdfPath &path :
             notice.GetResolvedAssetPathsResyncedPaths()) {
            if (touchesInput(path)) {
                relevant = true;
                break;
            }
        }
    }
    if (relevant) {
        UsdTimeCode time;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // A value-only edit keeps the cached dependency regions. Resyncs
            // and connection/relationship edits may introduce a new external
            // input even when the evaluator's compiled rig stays unchanged.
            // A property resync that names only a value (a spec created by
            // its first authored value, stating `custom` and `variability`
            // with it, or removed by the undo of it) is a value edit here
            // too: a new connection or target would arrive
            // as its own field, and a removed one leaves the cached regions
            // a superset of what is read, which is safe. The one input a
            // removal can reveal is a weaker layer's connection the removed
            // spec was hiding, so a property still connected afterwards
            // counts as structure.
            for (const SdfPath &path : notice.GetResyncedPaths()) {
                if (_readRootsDirty) {
                    break;
                }
                if (!path.IsPropertyPath()) {
                    _readRootsDirty = true;
                    break;
                }
                const UsdAttribute attribute =
                    _stage ? _stage->GetAttributeAtPath(path) : UsdAttribute();
                const UsdRelationship relationship =
                    _stage ? _stage->GetRelationshipAtPath(path)
                           : UsdRelationship();
                if ((attribute && attribute.HasAuthoredConnections()) ||
                    (relationship && relationship.HasAuthoredTargets())) {
                    _readRootsDirty = true;
                    break;
                }
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field != "typeName" && field != "default" &&
                        field != "timeSamples" && field != "spline" &&
                        field != "custom" && field != "variability") {
                        _readRootsDirty = true;
                        break;
                    }
                }
            }
            for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field == "connectionPaths" || field == "targetPaths") {
                        _readRootsDirty = true;
                    }
                }
            }
            // A resolved asset arriving from a new file can reveal a new
            // external input the cached regions never named.
            if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
                _readRootsDirty = true;
            }
            for (RigSession &session : _sessions) {
                const auto affects = [&session](const SdfPath &path) {
                    const SdfPath prim = path.GetPrimPath();
                    for (const SdfPath &root : session.readRoots) {
                        if (prim.HasPrefix(root) || root.HasPrefix(prim))
                            return true;
                    }
                    return false;
                };
                bool affected = false;
                for (const SdfPath &path : notice.GetResyncedPaths())
                    affected = affected || affects(path);
                for (const SdfPath &path : notice.GetChangedInfoOnlyPaths())
                    affected = affected || affects(path);
                for (const SdfPath &path :
                     notice.GetResolvedAssetPathsResyncedPaths())
                    affected = affected || affects(path);
                // A playback session never dirties: the binary is static,
                // so no stage edit changes what it publishes. Picking up
                // a new asset (or a new file behind it) needs a
                // re-activation.
                if (affected && !session.playback) {
                    RigExecProfileScope noticeScope(
                        session.bridge->MutableProfiler(), "Notice.Imaging",
                        "notice");
                    // A commit of the pose on screen: the published
                    // generation is already the stage's answer.
                    const bool settled = !session.settleOverrides.empty() &&
                                         _NoticeSettlesPreview(session, notice);
                    if (!settled) {
                        session.settleOverrides.clear();
                    }
                    session.dirty = !settled;
                    session.readRootsDirty =
                        session.readRootsDirty || _readRootsDirty;
                    // The bridge caches authored guide styling (shape, scale,
                    // colour, purpose...) across generations; an edit that
                    // reaches the rig is the only thing that can move it,
                    // and only the entries it names are dropped.
                    session.bridge->NoteGuideEdits(notice);
                    // The live sampler's pinned chain bindings, on the
                    // same footing: an edit under a chain mover moves a
                    // chain without moving the compiled rig signature.
                    session.bridge->NoteChainEdits(notice);
                    // Outcome-driven retirement: the
                    // evaluator classifies this notice (patched plus
                    // paths, routed plus the paths it reads, stamp-bumped,
                    // or stale) through a read-only query, so the verdict
                    // holds whatever order the two notice handlers ran in.
                    // Patch/routed/stamp edits partition
                    // completions by entry provenance -- clean entries
                    // stand or carry, dirty ones retire -- and purge
                    // affected times only, without bumping the
                    // generation, so other times' queued and running jobs
                    // still publish; a stale program (or no program)
                    // keeps the global cancel plus the capture-index
                    // compiled rig retirement.
                    std::vector<SdfPath> patchedPaths;
                    const RigExecNoticeDisposition disposition =
                        session.bridge->GetEvaluator()
                            .ClassifyNoticeDisposition(notice, &patchedPaths);
                    const auto namesRig = [&session](const SdfPath &path) {
                        return path.HasPrefix(session.rigPath) ||
                               session.rigPath.HasPrefix(path);
                    };
                    const auto resynced = notice.GetResyncedPaths();
                    const auto changed = notice.GetChangedInfoOnlyPaths();
                    const bool rigStructureChanged =
                        session.readRootsDirty &&
                        (std::any_of(resynced.begin(), resynced.end(),
                                     namesRig) ||
                         std::any_of(changed.begin(), changed.end(), namesRig));
                    if (rigStructureChanged) {
                        // A removed binding can disappear from the capture
                        // index before this handler sees its notice.
                        _CancelGenerationLocked(session.rigPath);
                        session.bridge->ClearFrameCache();
                    } else if (disposition ==
                                   RigExecNoticeDisposition::Patched ||
                               disposition ==
                                   RigExecNoticeDisposition::Edited ||
                               disposition ==
                                   RigExecNoticeDisposition::StampBumped) {
                        _RetireAffectedTimesLocked(session, notice, disposition,
                                                   patchedPaths);
                    } else {
                        // A new edit bumps the warming generation and
                        // cancels in-flight jobs for the rig; the commit
                        // trigger re-enqueues fresh jobs under the new
                        // token.
                        _CancelGenerationLocked(session.rigPath);
                        // The baked capture index retires cached frames
                        // eagerly: a notice that hits it drops the compiled rig
                        // now, while its half still names it (the
                        // evaluator rebuilds lazily); a miss drops
                        // nothing and digest reachability decides.
                        session.bridge->NoteCaptureIndex(notice);
                    }
                }
            }
            time = _lastTime;
        }
        if (fenceLock.owns_lock()) {
            fenceLock.unlock();
        }
        SetTime(time);
    }
}

} // namespace rigExec
