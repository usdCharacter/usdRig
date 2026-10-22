// USD notice classification and compiled rig invalidation.

#include "rigEvaluatorInternal.h"
#include "projectorCaptureNotice.h"
#include "rigEvaluatorPropertyBindings.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorConstraints.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// RIGEXEC_VERIFY_DIGEST_GATE=1: every notice the digest gate calls neutral,
// on a compiled rig nothing has dirtied since its last commit or settle, has the
// digest recomputed there and then, and moving it is fatal.
bool
_DigestGateVerifyRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_VERIFY_DIGEST_GATE", false);
    return requested;
}

// Whether \p notice is provably nothing but new VALUES on the numeric avar
// channels (see _OnObjectsChanged): the gate for the in-place patch path.
// Shared by the notice handler and ClassifyNoticeDisposition, so the two
// can never disagree about which branch a notice takes.
bool
_NoticeIsAvarValuesOnly(const UsdNotice::ObjectsChanged &notice)
{
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    static const TfToken kTypeName("typeName");
    // The numeric channels only. avars:defaultSpace,
    // avars:rotationOrder and avars:rotationSign choose how a frame is
    // composed, which is structure, so they are not in this list -- the
    // baked program captures the sign once per slot and only a recompile
    // can move it.
    static const TfToken kChannels[] = {
        TfToken("avars:tx"), TfToken("avars:ty"), TfToken("avars:tz"),
        TfToken("avars:sx"), TfToken("avars:sy"), TfToken("avars:sz"),
        TfToken("avars:rx"), TfToken("avars:ry"), TfToken("avars:rz"),
        TfToken("avars:rspin"), TfToken("avars:unitScaleFactor")};
    const auto isChannel = [](const TfToken &name) {
        for (const TfToken &avar : kChannels) {
            if (name == avar) {
                return true;
            }
        }
        return false;
    };
    bool sawAvar = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath() || !isChannel(path.GetNameToken())) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kTypeName && field != kDefault &&
                field != kTimeSamples && field != kSpline) {
                return false;
            }
        }
        sawAvar = true;
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path.IsPrimPath()) {
            continue;  // ancestor info around the edit
        }
        if (!isChannel(path.GetNameToken())) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline) {
                return false;
            }
        }
        sawAvar = true;
    }
    return sawAvar;
}

// Built while the library loads, so a notice on any thread compares without
// a guarded static.
const TfToken _mlMoverType("RigExecMlMover");
const TfToken _animatedMapsType("RigExecAnimatedMaps");

// The types whose prims the digest writes the path of for the type alone,
// wherever under the rig they sit: the discovered joints, controls, volume
// weights and pose interpolators, and every aggregate solver.
bool
_IsDigestDiscoveredType(const TfToken &type)
{
    static const TfToken kJoint("RigExecJoint");
    static const TfToken kControl("RigExecControl");
    static const TfToken kInterpolator("RigExecPoseInterpolator");
    return type == kJoint || type == kControl || type == kInterpolator ||
           type == _mlMoverType || type == _animatedMapsType ||
           _IsVolumeWeightType(type) || _IsAggregateSolverType(type);
}

} // namespace

namespace evaluatorDetail {

_CertainStructuralCounts *
_CertainStructuralVerifyCounts()
{
    static _CertainStructuralCounts *const counts =
        []() -> _CertainStructuralCounts * {
        if (!TfGetenvBool("RIGEXEC_VERIFY_CERTAIN_STRUCTURAL", false)) {
            return nullptr;
        }
        // Never freed: the exit hook reads it after statics may be gone.
        static _CertainStructuralCounts *const created =
            new _CertainStructuralCounts();
        std::atexit([]() {
            std::fprintf(stderr,
                         "RIGEXEC_VERIFY_CERTAIN_STRUCTURAL: %zu certainly "
                         "structural edit(s), %zu false positive(s); %zu "
                         "structural edit(s) found by the digest\n",
                         created->tagged.load(),
                         created->falsePositives.load(),
                         created->digestFound.load());
            std::fflush(stderr);
        });
        return created;
    }();
    return counts;
}

} // namespace evaluatorDetail

RigExecNoticeDisposition
RigExecRigEvaluator::ClassifyNoticeDisposition(
    const UsdNotice::ObjectsChanged &notice,
    std::vector<SdfPath> *patchedPaths) const
{
    if (patchedPaths) {
        patchedPaths->clear();
    }
    if (!_bakedProgram) {
        return RigExecNoticeDisposition::None;
    }
    // Routed to nothing: a value no step reads.
    if (RigExecNoticeIsDisplayOnly(notice)) {
        return RigExecNoticeDisposition::Edited;
    }
    std::vector<SdfPath> paths;
    if (_NoticeIsAvarValuesOnly(notice) &&
        _bakedProgram->DryRunAvarValueEdits(notice, &paths)) {
        if (patchedPaths) {
            *patchedPaths = std::move(paths);
        }
        return RigExecNoticeDisposition::Patched;
    }
    if (_bakedProgram->DryRunMixedValueEdits(notice, &paths)) {
        if (patchedPaths) {
            *patchedPaths = std::move(paths);
        }
        return RigExecNoticeDisposition::Edited;
    }
    if (_bakedProgram->IsInvalidatedBy(notice)) {
        return RigExecNoticeDisposition::Stale;
    }
    if (_bakedProgram->DryRunValueEdits(notice, &paths)) {
        if (patchedPaths) {
            *patchedPaths = std::move(paths);
        }
        return RigExecNoticeDisposition::Edited;
    }
    return RigExecNoticeDisposition::StampBumped;
}

RigExecRigEvaluator::_DigestGate
RigExecRigEvaluator::_MakeDigestGate(
    _DigestFootprint (&parts)[_DigestSegmentCount])
{
    _DigestGate gate;
    gate.valid = true;
    // Every recorded prim and every ancestor of one, walked up only until a
    // path already present: that path's own ancestors went in with it.
    const auto cover = [&gate](const SdfPath &prim) {
        for (SdfPath path = prim; !path.IsEmpty();
             path = path.GetParentPath()) {
            if (!gate.covered.insert(path).second ||
                path.IsAbsoluteRootPath()) {
                break;
            }
        }
    };
    for (size_t i = 0; i < _DigestSegmentCount; ++i) {
        const _DigestFootprint &part = parts[i];
        for (const SdfPath &prim : part.read) {
            gate.read.insert(prim);
            cover(prim);
        }
        for (const SdfPath &prim : part.ancestors) {
            cover(prim);
        }
        // Kept per segment rather than merged: each is asked by lookup, and
        // three lookups cost less than building one table out of three.
        gate.certain[i] = std::move(parts[i].certain);
    }
    return gate;
}

bool
RigExecRigEvaluator::_NoticeIsDigestSuspect(
    const UsdNotice::ObjectsChanged &notice) const
{
    if (!_digestGate.valid ||
        !notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return true;
    }
    // Judged per prim: a property path answers for its prim, which is the
    // grain the footprint was recorded at.
    const auto suspect = [this](const SdfPath &path, bool resync) {
        const SdfPath prim = path.GetPrimPath();
        // Anything in the rig: the digest reads the rig whole.
        if (prim.HasPrefix(_rigPath)) {
            return true;
        }
        // A resync at or above the rig recomposes all of it.
        if (resync && _rigPath.HasPrefix(prim)) {
            return true;
        }
        // Changed info on a prim outside the rig, rather than on one of its
        // properties. The digest reads no prim metadata there, and USD
        // reports the fields that could move it -- active, kind, specifier,
        // typeName, apiSchemas -- as resyncs. It is also what every edit
        // under a prim says about the prim itself: defining a child raises
        // it on the parent, which is an ancestor every chain walks through.
        if (!resync && path.IsPrimPath()) {
            return false;
        }
        // The digest follows instance proxies, and an edit behind one is
        // reported on the prototype, not on the proxy path it read.
        if (UsdPrim::IsPathInPrototype(prim)) {
            return true;
        }
        // On or above a prim the digest read or walked through.
        if (_digestGate.covered.count(prim)) {
            return true;
        }
        // Under a prim it read. Not under an ancestor it only walked
        // through: nothing below an ancestor is read by way of it.
        for (SdfPath above = prim.GetParentPath();
             !above.IsEmpty() && !above.IsAbsoluteRootPath();
             above = above.GetParentPath()) {
            if (_digestGate.read.count(above)) {
                return true;
            }
        }
        return false;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (suspect(path, true)) {
            return true;
        }
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (suspect(path, false)) {
            return true;
        }
    }
    return false;
}

bool
RigExecRigEvaluator::_NoticeIsDigestValueNeutral(
    const UsdNotice::ObjectsChanged &notice) const
{
    if (!_digestGate.valid || !_stage ||
        !notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    static const TfToken kTypeName("typeName");
    static const TfToken kCustom("custom");
    static const TfToken kVariability("variability");
    const auto valueRead = [this](const SdfPath &path) {
        for (const _CertainFootprint &certain : _digestGate.certain) {
            if (certain.valueReads.count(path)) {
                return true;
            }
        }
        return false;
    };
    // Whether a walk wrote this attribute's sample count and type, and
    // whether either could have moved: a sample authored or removed, or the
    // attribute not having the type the walk wrote (including appearing
    // where the walk wrote "missing").
    const auto samplesMoved = [this](const SdfPath &path,
                                     const std::vector<TfToken> &fields,
                                     const UsdAttribute &attribute) {
        static const TfToken kSamples("timeSamples");
        for (const _CertainFootprint &certain : _digestGate.certain) {
            const auto found = certain.sampleCountReads.find(path);
            if (found == certain.sampleCountReads.end()) {
                continue;
            }
            if (std::find(fields.begin(), fields.end(), kSamples) !=
                fields.end()) {
                return true;
            }
            const TfToken now = attribute ? attribute.GetTypeName().GetAsToken()
                                          : TfToken();
            if (now != found->second) {
                return true;
            }
        }
        return false;
    };
    // What the digest wrote about this attribute's prim: whether it listed
    // the prim's attributes, and the type it wrote for this one if so.
    const auto listed = [this](const SdfPath &path, const TfToken **type) {
        *type = nullptr;
        bool prim = false;
        for (const _CertainFootprint &certain : _digestGate.certain) {
            prim = prim || certain.closurePrims.count(path.GetPrimPath()) > 0;
            const auto found = certain.closureAttributes.find(path);
            if (found != certain.closureAttributes.end()) {
                *type = &found->second;
            }
        }
        return prim;
    };
    bool sawValue = false;
    // A property resync is a spec appearing or going away under a value --
    // the first value a layer authors for a channel, or the undo removing
    // it. It may restate the spec's own fields, never a connection. Only on
    // the walk of an input read the long way every frame, or on an avar the
    // program patches from the attribute itself: anything else may hold a
    // query pinned to the spec it resolved, which the settle refreshes.
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath() || valueRead(path) || !_bakedProgram ||
            !(_bakedProgram->ReadsLive(path) ||
              _bakedProgram->IsPatchableAvarPath(path))) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline && field != kTypeName && field != kCustom &&
                field != kVariability) {
                return false;
            }
        }
        const UsdAttribute attribute = _stage->GetAttributeAtPath(path);
        if (!attribute ||
            samplesMoved(path, notice.GetChangedFields(path), attribute)) {
            return false;
        }
        const TfToken *type = nullptr;
        if (listed(path, &type) &&
            (!type || *type != attribute.GetTypeName().GetAsToken())) {
            return false;  // appeared on, or retyped on, a listed prim
        }
        sawValue = true;
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path.IsPrimPath()) {
            // The empty info every ancestor of an edited spec reports.
            if (!notice.GetChangedFields(path).empty()) {
                return false;
            }
            continue;
        }
        if (valueRead(path)) {
            return false;
        }
        const std::vector<TfToken> fields = notice.GetChangedFields(path);
        for (const TfToken &field : fields) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline) {
                return false;
            }
        }
        if (samplesMoved(path, fields, _stage->GetAttributeAtPath(path))) {
            return false;
        }
        sawValue = true;
    }
    return sawValue;
}

void
RigExecRigEvaluator::_VerifyNeutralNotice(
    const UsdNotice::ObjectsChanged &notice) const
{
    // Nothing since the last commit or settle was suspect, so the committed
    // digest is the digest of the stage this notice found; recomputed now,
    // on the stage it left behind, it must not have moved.
    const size_t digest = _SettleStructureDigest();
    if (digest == _structureDigest) {
        return;
    }
    std::string paths;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        paths += " resync:" + path.GetString();
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        paths += " info:" + path.GetString();
    }
    const std::string message = TfStringPrintf(
        "RIGEXEC_VERIFY_DIGEST_GATE: a notice the digest gate called neutral "
        "moved the structure digest of <%s>; it named%s",
        _rigPath.GetText(), paths.c_str());
    // Written out first as well: a fatal error's own text was measured not
    // to reach a Windows test's stderr before the abort, and a violation
    // nobody can read names no path to fix.
    std::fputs((message + "\n").c_str(), stderr);
    std::fflush(stderr);
    TF_FATAL_ERROR("%s", message.c_str());
}

void
RigExecRigEvaluator::_NoteCertainStructuralCandidates(
    const UsdNotice::ObjectsChanged &notice)
{
    // Judged against the committed footprint, so there is nothing to note
    // without one: the settle computes the digest either way.
    if (!_digestGate.valid || _certainCandidatesOverflowed) {
        return;
    }
    // A bound on what one settle judges. An edit past it -- a layer swap, a
    // reference retargeted -- is one whose digest has to be computed to be
    // sure of anything, and it gets exactly that.
    static constexpr size_t kMaxCandidates = 256;
    const auto note = [this](const SdfPath &path) {
        if (_certainCandidates.size() == kMaxCandidates) {
            _certainCandidatesOverflowed = true;
            _certainCandidates.clear();
            return false;
        }
        _certainCandidates.push_back(path);
        return true;
    };
    // Whether the committed footprint recorded anything the property could
    // change: its targets, a solver's jointElements, or an attribute of a
    // pose-input closure prim.
    const auto recorded = [this](const SdfPath &property) {
        const _CertainFootprint::PropertyKey key{property.GetPrimPath(),
                                                 property.GetNameToken()};
        for (const _CertainFootprint &certain : _digestGate.certain) {
            if (certain.targets.count(key) ||
                certain.jointElements.count(key.prim) ||
                certain.closurePrims.count(key.prim)) {
                return true;
            }
        }
        return false;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        // A prim resync strictly inside the rig: at or above the rig root,
        // everything recomposes, and that is the digest's to judge.
        const bool candidate = path.IsPrimPath()
            ? path != _rigPath && path.HasPrefix(_rigPath)
            : path.IsPropertyPath() && recorded(path);
        if (candidate && !note(path)) {
            return;
        }
    }
    static const TfToken kTargetPaths("targetPaths");
    static const TfToken kConnectionPaths("connectionPaths");
    static const TfToken kJointElements("rigExec:jointElements");
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (!path.IsPropertyPath()) {
            continue;
        }
        bool listEdit = path.GetNameToken() == kJointElements;
        for (const TfToken &field : notice.GetChangedFields(path)) {
            listEdit = listEdit || field == kTargetPaths ||
                       field == kConnectionPaths;
        }
        if (listEdit && recorded(path) && !note(path)) {
            return;
        }
    }
}

bool
RigExecRigEvaluator::_EditIsCertainlyStructural() const
{
    if (!_digestGate.valid || _certainCandidatesOverflowed ||
        _certainCandidates.empty() || !_stage) {
        return false;
    }
    // The digest walks the rig with the default predicate, so a rig that no
    // longer passes it is a rig whose every list has changed shape, which is
    // the digest's to say.
    const UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (!rig || rig.IsInstanceProxy() || !UsdPrimDefaultPredicate(rig)) {
        return false;
    }
    static const TfToken kJoint("RigExecJoint");
    static const TfToken kJointElements("rigExec:jointElements");
    // A prim's state as the digest's discovery walks see it now: its type,
    // when it is in the walk and of a discovered type; empty when it is not
    // written for its type at all; nothing when that cannot be told. A joint
    // outside the walk is the case that cannot: a solver's rigExec:joints
    // still unions it into the joint set, so leaving the walk need not move
    // the digest.
    const auto stateNow = [this](const SdfPath &path)
        -> std::optional<TfToken> {
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        if (!prim) {
            return TfToken();
        }
        const TfToken type = prim.GetTypeName();
        if (!_IsDigestDiscoveredType(type)) {
            return TfToken();
        }
        bool walked = !prim.IsInstanceProxy();
        for (UsdPrim at = prim; walked && at.GetPath() != _rigPath;
             at = at.GetParent()) {
            walked = UsdPrimDefaultPredicate(at);
        }
        if (walked) {
            return type;
        }
        if (type == kJoint) {
            return std::nullopt;
        }
        return TfToken();
    };
    const auto changed = [&stateNow](const SdfPath &path,
                                     const TfToken &recorded) {
        const std::optional<TfToken> now = stateNow(path);
        return now && *now != recorded;
    };
    for (const SdfPath &path : _certainCandidates) {
        if (path.IsPrimPath()) {
            // The resynced prim and every recorded prim below it: a prim
            // resync recomposes the whole subtree, so it can remove, retype
            // or deactivate anything the last digest listed there. A prim
            // it creates below itself is not looked for.
            bool recordedHere = false;
            for (const _CertainFootprint &certain : _digestGate.certain) {
                for (auto it = certain.prims.lower_bound(path);
                     it != certain.prims.end() && it->first.HasPrefix(path);
                     ++it) {
                    recordedHere = recordedHere || it->first == path;
                    if (changed(it->first, it->second)) {
                        return true;
                    }
                }
            }
            if (!recordedHere && changed(path, TfToken())) {
                return true;
            }
            continue;
        }
        // A property: its owner must still stand for the digest to reach it
        // where it did.
        const UsdPrim owner = _stage->GetPrimAtPath(path.GetPrimPath());
        if (!owner) {
            continue;
        }
        const _CertainFootprint::PropertyKey key{path.GetPrimPath(),
                                                 path.GetNameToken()};
        for (const _CertainFootprint &certain : _digestGate.certain) {
            const auto targets = certain.targets.find(key);
            if (targets != certain.targets.end()) {
                SdfPathVector now;
                if (const UsdRelationship rel =
                        owner.GetRelationship(key.name)) {
                    rel.GetTargets(&now);
                }
                SdfPathVector before = targets->second.targets;
                if (targets->second.sorted) {
                    std::sort(now.begin(), now.end());
                    std::sort(before.begin(), before.end());
                }
                if (now != before) {
                    return true;
                }
            }
            const auto elements = certain.jointElements.find(key.prim);
            if (elements != certain.jointElements.end() &&
                key.name == kJointElements) {
                VtIntArray values;
                size_t samples = 0;
                if (const UsdAttribute attr = owner.GetAttribute(key.name)) {
                    attr.Get(&values);
                    samples = attr.GetNumTimeSamples();
                }
                if (values != elements->second.values ||
                    samples != elements->second.samples) {
                    return true;
                }
            }
            if (certain.closurePrims.count(key.prim)) {
                // Every attribute of a closure prim is written with its
                // sources, so its sources now are compared with the ones
                // recorded -- none, where it was not recorded as connected.
                // One that is gone was certainly written only if it was
                // recorded: an unconnected one may never have existed.
                const auto connected = certain.closureConnections.find(path);
                const bool wasConnected =
                    connected != certain.closureConnections.end();
                const UsdAttribute attr = owner.GetAttribute(key.name);
                if (!attr) {
                    if (wasConnected) {
                        return true;
                    }
                } else if (_AuthoredConnections(attr) !=
                           (wasConnected ? connected->second
                                         : SdfPathVector())) {
                    return true;
                }
            }
        }
    }
    return false;
}

void
RigExecRigEvaluator::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &)
{
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Notice.Evaluator", "notice");
    // Showing or hiding prims: no step reads visibility, so nothing the rig
    // holds is touched -- above all not the controls' parent transform,
    // which the bake reads whole.
    if (RigExecNoticeIsDisplayOnly(notice)) {
        _lastNoticeDisposition = _bakedProgram
                                     ? RigExecNoticeDisposition::Edited
                                     : RigExecNoticeDisposition::None;
        _lastNoticePatchedPaths.clear();
        return;
    }
    ++_stageEditSerial;
    // Never evaluate in a notice callback: ExecUsd must finish invalidating
    // its own caches before the next pull. External inputs can live anywhere
    // on the stage, so retain conservative structural checks after edits.
    // EXCEPT for a notice that is provably nothing but new VALUES on the
    // numeric avar channels: an Avar Editor slider tick, a typed value, a
    // key moved, a released gizmo. The structure digest never reads an
    // avar's value (only rigExec:* attributes, and the time-sample COUNT of
    // a handful of those), and the rest frames the compiled rig refresh on an edit
    // are rest:*, not avars -- so re-deriving both answered "nothing
    // changed" at ~115 ms per slider tick on the biped. Anything else in the
    // notice -- a resync, a non-avar property, a field other than a value --
    // keeps the conservative path.
    // A RESYNC on one of those channels still qualifies when it is on the
    // property alone. The first value a layer holds for an avar creates its
    // property spec, which USD reports as a property resync carrying only
    // typeName, and undoing it removes the spec, a resync carrying nothing.
    // Every released gizmo drag and its undo is exactly that pair, and
    // treating it as structure rebaked the program for ~250 ms per release.
    // A prim resync is never a value edit and keeps the conservative path.
    // And a notice that is not avar values reaches the digest only when it
    // could move it: _NoticeIsDigestSuspect is the prim-granular gate over
    // what the committed digest read outside the rig. An edit on a material
    // beside the rig, or on another rig, leaves the digest where it was and
    // costs the next settle nothing. The digest itself still trusts no
    // incremental invalidation: a suspect notice recomputes it whole.
    // A suspect notice also names, for the settle to judge, the paths that
    // could make it CERTAINLY structural: a prim whose path the digest
    // writes because of its type, a relationship whose targets it writes, a
    // connection inside a pose-input closure. When one of them has changed,
    // the digest has moved, and the settle compiles without computing it.
    // Rebuild the immutable raw Default meshWorldInverse capture, not
    // unrelated sampled provider xforms. The new compile also rechecks the
    // static-transform admission when time samples are added or removed.
    bool projectorCaptureChanged=false;
    for(const auto &entry:_graphDerivedChains)for(const auto &revision:entry.second)
        if((revision.op==RigExecRevisionOp::SurfaceProjector || revision.op==RigExecRevisionOp::ShaderDials) &&
           RigExecProjectorMeshCaptureAffected(notice,revision.binding.base))projectorCaptureChanged=true;
    if(projectorCaptureChanged) { _compiled=false; _structureDirty=true; }
    const bool avarValuesOnly = _NoticeIsAvarValuesOnly(notice);
    // ... unless the avar the notice names is one a property chain READS.
    //
    // A chain's inputs:value may be connected to a control's avar -- that is
    // how every "distance from rest, from the control's own translate" chain
    // in this rig is built -- and a binding that was resolved before the
    // avar moved is resolved against a stale number. The bindings' own dirty
    // test watches interactive overrides and upstream chains, not authored
    // values, so a drag updated such a chain and AUTHORING the same avar did
    // not: the cheeks followed the mouth corner while it was moving and
    // stopped the moment it was let go.
    const bool chainReadsAnEditedAvar = avarValuesOnly &&
        !_propertyChainInputs.empty() && [this, &notice]() {
            for (const SdfPath &path : notice.GetResyncedPaths()) {
                if (_propertyChainInputs.count(path)) return true;
            }
            for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
                if (_propertyChainInputs.count(path)) return true;
            }
            return false;
        }();
    if (chainReadsAnEditedAvar) {
        // The one thing an avar-only notice CAN invalidate, dropped on its
        // own so the rest of the fast path is untouched: an Avar Editor tick
        // still skips the skin layouts, the blend samples and the rest.
        _propertyChainBindings.reset();
    }
    // A notice that moves only values the digest never writes -- a switch,
    // a dial, a space index on a control -- cannot move it, so it is not
    // suspect even inside the rig, where every other notice is: the settle
    // that recomputes the digest for a suspect notice was ~200 ms of a
    // 220 ms IK/FK switch on the biped.
    if (!avarValuesOnly) {
        if (!_NoticeIsDigestValueNeutral(notice) &&
            _NoticeIsDigestSuspect(notice)) {
            _structureDirty = true;
            _NoteCertainStructuralCandidates(notice);
        } else if (!_structureDirty && _compiled &&
                   _DigestGateVerifyRequested()) {
            _VerifyNeutralNotice(notice);
        }
    }
    // The compiled rig's rest frames answer to their own gate, not to the digest:
    // the digest reads no rest:* value, so a rest:rx edit is not a
    // structural edit and never recompiles, and the rests it moved are still
    // owed to the dynamic walk.
    _NoteRestEdits(notice);
    // The value caches that outlive a generation hold authored values and
    // where they come from, so an edit has to reach the very next read of
    // anything it moved -- and only that: dropping them whole on every
    // notice cost ~2.5 ms of re-binding and re-reading on the first frame
    // after any edit on the biped, for chains, layouts and shapes the edit
    // never touched.
    _ClearValueCaches(notice, avarValuesOnly);
    // The baked program captured values, and the compiled rig signature is deliberately
    // blind to values, so the digest cannot say whether one of them moved.
    // The program's own index of what the bake read can: a notice that hits
    // it asks for a rebuild, and one that misses it -- a value on an input
    // read per frame, anything on a prim the bake never looked at -- leaves
    // the program standing, which is what keeps an edit elsewhere in the
    // scene from degrading the rig to the dynamic path.
    if (_bakedProgram) {
        // Classified first, through the same query the registry's notice
        // adapter consumes: the branch below and the adapter's
        // retire/re-resolve decision read one verdict.
        _lastNoticeDisposition =
            ClassifyNoticeDisposition(notice, &_lastNoticePatchedPaths);
        if (_lastNoticeDisposition == RigExecNoticeDisposition::Patched) {
            // Patched in place: the program's avar table already holds the
            // new constants, and its by-value slot comparison re-runs only
            // their cone. No rebuild, and no stamp bump -- the bump would
            // mark the whole program dirty for one run to find what the
            // comparison already finds. The dry run inside the
            // classification already passed, so this applies; a refusal
            // (impossible on this thread, but fail-closed) falls back to
            // the stamp bump.
            if (!_bakedProgram->ApplyAvarValueEdits(notice)) {
                _bakedProgram->BumpProgramStamp();
            }
        } else if (_lastNoticeDisposition ==
                   RigExecNoticeDisposition::Edited) {
            // Routed like an override placed for one run and lifted: the
            // inputs the edit reached are marked, and the next run re-runs
            // the cone of the steps that read them instead of everything.
            // Whatever else the notice named is compared by value where it
            // is read, or read by nothing. A refusal (impossible after the
            // classification, but fail-closed) falls back to the stamp.
            // A mixed notice (patchable avars beside routed values) is
            // applied as the classification decided it.
            if (!_bakedProgram->ApplyMixedValueEdits(notice) &&
                !_bakedProgram->ApplyValueEdits(notice)) {
                _bakedProgram->BumpProgramStamp();
            }
        } else if (_lastNoticeDisposition ==
                   RigExecNoticeDisposition::Stale) {
            _bakedProgramStale = true;
            // Input sampling may precede the lazy rebuild. Retire held raw
            // reads too; a structural notice has no routed leaf edits.
            _bakedProgram->BumpProgramStamp();
            // The rebuild is allowed to refuse where the standing program did
            // not, and a refusal remembered from before this notice would
            // otherwise answer for a stage that has since changed.
            _bakeRefused = false;
            _bakeRefusalReasons.clear();

        } else {
            // The other half of the same index, and the reason the program
            // may skip work at all. A notice that misses the index and that
            // the program could not route is a value edit on something the
            // frame path re-reads through a reader no dirty set names -- or
            // a resync, or layer metadata -- so the program is still right
            // about its structure and may be wrong about any value it cached
            // from the last frame. Saying so here is what makes the next
            // generation run everything once; the program compares its own
            // sources by value from then on.
            _bakedProgram->BumpProgramStamp();
        }
    } else {
        _lastNoticeDisposition = RigExecNoticeDisposition::None;
        _lastNoticePatchedPaths.clear();
    }
    if (_lastNoticeDisposition != RigExecNoticeDisposition::Patched &&
        _lastNoticeDisposition != RigExecNoticeDisposition::Edited) {
        ++_stageStructureSerial;
    }
    _LogValueEdit(notice);
}

void
RigExecRigEvaluator::_LogValueEdit(const UsdNotice::ObjectsChanged &notice)
{
    constexpr size_t kMaxValueEdits = 64;
    const bool absorbed =
        _lastNoticeDisposition == RigExecNoticeDisposition::Patched ||
        _lastNoticeDisposition == RigExecNoticeDisposition::Edited;
    _ValueEdit edit;
    edit.serial = _stageEditSerial;
    // A property resync -- a spec appearing or going away under a value, as
    // a first key or its undo does -- names its path like a value edit. A
    // read reaching a property that did not exist lists that path among its
    // hops too (RigExecRevisionLeafHops), so the path is all a consumer needs.
    bool covered = absorbed;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!covered) {
            break;
        }
        covered = path.IsPropertyPath();
        edit.paths.push_back(path);
    }
    // Changed info on a PRIM -- an `over` authored above a first value, or
    // metadata -- moves no attribute value: every field that could change
    // what a property composes to is reported as a resync instead.
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (!covered) {
            break;
        }
        if (path.IsPrimPath()) {
            continue;
        }
        covered = path.IsPropertyPath();
        edit.paths.push_back(path);
    }
    if (!covered) {
        _valueEdits.clear();
        _valueEditsFloor = _stageEditSerial;
        return;
    }
    if (_valueEdits.size() == kMaxValueEdits) {
        _valueEditsFloor = _valueEdits.front().serial;
        _valueEdits.erase(_valueEdits.begin());
    }
    _valueEdits.push_back(std::move(edit));
}

bool
RigExecRigEvaluator::GetValueEditsSince(uint64_t serial,
                                        std::vector<SdfPath> *paths) const
{
    paths->clear();
    if (serial < _valueEditsFloor || serial > _stageEditSerial) {
        return false;
    }
    for (const _ValueEdit &edit : _valueEdits) {
        if (edit.serial > serial) {
            paths->insert(paths->end(), edit.paths.begin(), edit.paths.end());
        }
    }
    std::sort(paths->begin(), paths->end());
    paths->erase(std::unique(paths->begin(), paths->end()), paths->end());
    return true;
}

} // namespace rigExec
