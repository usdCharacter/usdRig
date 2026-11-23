// Stage edits a baked program absorbs without a rebuild: value edits, avar
// patches and overrides.
#include "rigExec/internal/weightField.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/inputReplay.h"

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedProgramShared.h"
#include "rigExec/internal/bakedBuildAttempt.h"
#include "rigExec/internal/projectorCaptureNotice.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/oracleDispatch.h"
#include "rigExec/internal/bakedSchedule.h"
#include "rigExec/internal/bakedOpGraph.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/frozenContextInternal.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/movers/pathOperation.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/api/types.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecSchema/schemaTokens.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

using namespace bakedProgramDetail;

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    (xformOpOrder)
    ((default_, "default"))
    (timeSamples)
    (spline)
    (typeName)
    (custom)
    (variability)
    (visibility)
);

// Invalidation.

template <class B, class Fn>
void
_ForEachFixedNumericSource(B &program, Fn &&fn)
{
    for (auto &ladder : program.ladders) {
        for (auto &input : ladder.restAvars)
            if (input.sourceBacked)
                fn(input);
        if (ladder.rotationSign.sourceBacked)
            fn(ladder.rotationSign);
    }
    for (auto &constraint : program.constraints)
        if (constraint.offset.sourceBacked)
            fn(constraint.offset);
}

// Whether \p path is a property that composes one of the intervening Xforms
// bakeability judged (xformPrims): their transform is xformOpOrder and the
// ops it lists, so no other property of theirs can move it.
bool
_ComposesJudgedXform(const RigExecBakedProgramImpl &B, const SdfPath &path)
{
    if (!path.IsPropertyPath() || !B.xformPrims.count(path.GetPrimPath()))
        return false;
    const TfToken &kOpOrder = _tokens->xformOpOrder;
    const TfToken &name = path.GetNameToken();
    return name == kOpOrder || TfStringStartsWith(name.GetString(), "xformOp:");
}

// A schema numeric argument can acquire or lose its first authored spec
// without changing its direct source binding. All structural variants refuse.
bool
_FixedNumericSourceResync(const RigExecBakedProgramImpl &B,
                          const UsdNotice::ObjectsChanged &notice,
                          const SdfPath &path)
{
    if (!B.stage || !path.IsPropertyPath() ||
        !B.sourceBackedPaths.count(path) || B.rebuild.count(path) ||
        _ComposesJudgedXform(B, path))
        return false;
    for (const auto &field : notice.GetChangedFields(path))
        if (field != SdfFieldKeys->Default && field != SdfFieldKeys->TypeName)
            return false;
    const auto attribute = B.stage->GetAttributeAtPath(path);
    if (!attribute || !attribute.GetPrim() || !attribute.GetPrim().IsActive() ||
        attribute.GetNumTimeSamples() != 0 || attribute.HasSpline() ||
        attribute.ValueMightBeTimeVarying())
        return false;
    SdfPathVector connections;
    attribute.GetConnections(&connections);
    if (!connections.empty())
        return false;
    bool found = false, valid = true;
    _ForEachFixedNumericSource(B, [&](const auto &input) {
        if (input.head.GetPath() != path)
            return;
        using T = std::decay_t<decltype(input.sourceFallback)>;
        static_assert(std::is_same_v<T, double> || std::is_same_v<T, GfVec3d>);
        found = true;
        valid = valid &&
                attribute.GetTypeName().GetType() ==
                    (std::is_same_v<T, double> ? SdfValueTypeNames->Double
                                               : SdfValueTypeNames->Double3)
                        .GetType();
    });
    return found && valid;
}
// A property resync that only brings, changes or takes away a value spec on
// an attribute that is still there: the first value a layer authors for a
// switch or a dial is a spec appearing, which USD reports as a resync.
// Nothing but the value and the fields a new spec states may have changed --
// a connection, a target or metadata that changes how it is read still
// rebuilds, and so does a path some folded input reads too.
bool
_ValueSpecResync(const RigExecBakedProgramImpl &B,
                 const UsdNotice::ObjectsChanged &notice, const SdfPath &path)
{
    if (!B.stage || !path.IsPropertyPath() || B.rebuild.count(path) ||
        _ComposesJudgedXform(B, path))
        return false;
    const TfToken &kDefault = _tokens->default_,
                  &kTimeSamples = _tokens->timeSamples,
                  &kSpline = _tokens->spline, &kTypeName = _tokens->typeName,
                  &kCustom = _tokens->custom,
                  &kVariability = _tokens->variability;
    for (const TfToken &field : notice.GetChangedFields(path))
        if (field != kDefault && field != kTimeSamples && field != kSpline &&
            field != kTypeName && field != kCustom && field != kVariability)
            return false;
    const UsdAttribute attribute = B.stage->GetAttributeAtPath(path);
    return attribute && attribute.GetPrim() && attribute.GetPrim().IsActive();
}

// The same on the walk of an input read the long way every frame, which is
// what lets IsInvalidatedBy leave such a resync to the value router.
bool
_LiveInputResync(const RigExecBakedProgramImpl &B,
                 const UsdNotice::ObjectsChanged &notice, const SdfPath &path)
{
    return B.liveInputPaths.count(path) && _ValueSpecResync(B, notice, path);
}

// A value spec appearing on the walk of an input the router can place: one
// read the long way, or a per-frame input whose pinned query ApplyValueEdits
// re-selects (_RequeryResynced) -- a query keeps the spec it resolved, and a
// new stronger one escapes it.
bool
_RoutableResync(const RigExecBakedProgramImpl &B,
                const UsdNotice::ObjectsChanged &notice, const SdfPath &path)
{
    return (B.liveInputPaths.count(path) || B.overridableInputs.count(path)) &&
           _ValueSpecResync(B, notice, path);
}

} // namespace

bool
RigExecNoticeIsDisplayOnly(const UsdNotice::ObjectsChanged &notice)
{
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    const TfToken &kVisibility = _tokens->visibility;
    bool saw = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath() || path.GetNameToken() != kVisibility) {
            return false;
        }
        saw = true;
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (path.IsPropertyPath()) {
            if (path.GetNameToken() != kVisibility) {
                return false;
            }
            saw = true;
        } else if (path.IsAbsoluteRootPath() ||
                   !notice.GetChangedFields(path).empty()) {
            return false;
        }
    }
    return saw;
}

bool
RigExecBakedProgram::ReadsLive(const SdfPath &property) const
{
    return _impl->liveInputPaths.count(property) > 0;
}

namespace {

// IsInvalidatedBy, with the paths in \p skip -- avars the patch absorbs --
// left to their own entry point.
bool
_IsInvalidatedBy(const RigExecBakedProgramImpl &B,
                 const UsdNotice::ObjectsChanged &notice,
                 const std::set<SdfPath> *skip)
{
    const auto skipped = [skip](const SdfPath &path) {
        return skip && skip->count(path) > 0;
    };
    for (const auto &chain : B.chains)
        for (const auto &derived : chain.derived) {
            const auto &revision = derived.revision;
            if ((revision.op == RigExecRevisionOp::SurfaceProjector ||
                 revision.op == RigExecRevisionOp::ShaderDials) &&
                RigExecProjectorMeshCaptureAffected(notice,
                                                    revision.binding.base))
                return true;
        }
    // A changed-info notice is a VALUE edit on a property that already
    // existed. It matters only where the bake read that value -- and, for
    // the Xforms bakeability accepted for composing to the identity, on any
    // property of theirs at all, since it is their composed transform and
    // not one named attribute that was judged. The prim-level info that
    // arrives for every ancestor of an edit (an `over` being created above
    // it) is not that, and is exactly what must not rebuild.
    const TfToken &kDefault = _tokens->default_;
    const TfToken &kTimeSamples = _tokens->timeSamples;
    const TfToken &kSpline = _tokens->spline;
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (skipped(path))
            continue;
        if (path.IsPrimPath() && B.prims.count(path)) {
            for (const TfToken &field : notice.GetChangedFields(path))
                if (field == SdfFieldKeys->PrimOrder ||
                    field == SdfChildrenKeys->PrimChildren ||
                    field == SdfFieldKeys->TypeName)
                    return true;
        }

        // Adding animation or changing resolution invalidates a fixed
        // source binding even though ordinary Default value edits do not.
        if (B.sourceBackedPaths.count(path)) {
            for (const TfToken &field : notice.GetChangedFields(path))
                if (field != kDefault)
                    return true;
        }
        if (B.rebuild.count(path) || _ComposesJudgedXform(B, path)) {
            return true;
        }
        // Anything but a value on a property the bake asked about by name:
        // a connection or a target retargeted, or metadata that changes how
        // the value is read. The walk the bake recorded for that property
        // -- and with it which upstream hops a later value edit is routed
        // through (ApplyValueEdits) -- may no longer be the walk the frame
        // takes, and only a rebuild records the new one.
        if (path.IsPropertyPath() && B.named.count(path)) {
            for (const TfToken &field : notice.GetChangedFields(path)) {
                if (field != kDefault && field != kTimeSamples &&
                    field != kSpline) {
                    return true;
                }
            }
        }
    }
    // A resync names a subtree whose composition changed: properties can have
    // appeared, disappeared or been retargeted anywhere inside it, so the
    // question is whether the subtree and the index overlap in EITHER
    // direction -- the resync above something the bake read, or at a property
    // of a prim it read from.
    const auto overlaps = [&B](const SdfPath &path) {
        for (const std::set<SdfPath> *index :
             {&B.named, &B.prims, &B.xformPrims}) {
            // Descendants of `path` are contiguous from lower_bound: SdfPath
            // sorts in namespace order, which _OnObjectsChanged already
            // relies on for the solver-batch subtree walk.
            const auto it = index->lower_bound(path);
            if (it != index->end() && it->HasPrefix(path)) {
                return true;
            }
        }
        // A resync that names a PROPERTY is that property appearing,
        // disappearing or being retargeted. It matters where the bake asked
        // about that property by name -- not merely somewhere on the prim,
        // or every rig would rebuild for a property it never reads.
        const SdfPath prim = path.GetPrimPath();
        if (path.IsPropertyPath()) {
            return _ComposesJudgedXform(B, path);
        }
        return B.prims.count(prim) > 0 || B.xformPrims.count(prim) > 0;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (skipped(path))
            continue;
        if (_FixedNumericSourceResync(B, notice, path))
            continue;
        if (_LiveInputResync(B, notice, path))
            continue;
        if (overlaps(path)) {
            return true;
        }
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        if (overlaps(path)) {
            return true;
        }
    }
    return false;
}

} // namespace

bool
RigExecBakedProgram::IsInvalidatedBy(
    const UsdNotice::ObjectsChanged &notice) const
{
    return _IsInvalidatedBy(*_impl, notice, nullptr);
}

namespace {

// The attributes a resolved-input reader can reach through connections; see
// `connectedSources`. Every authored attribute of every resolvedRoutedPrims
// prim is a walk head, and every connection target is followed on -- all of
// them, where the reader follows only a single one: a reader that stops at
// two connections reads the attribute's own value instead, which is on a
// routed prim already, so the extra targets only report more.
const std::set<SdfPath> &
_ConnectedSources(const RigExecBakedProgramImpl &B)
{
    if (B.connectedSourcesFilled && B.connectedSourcesStamp == B.programStamp) {
        return B.connectedSources;
    }
    B.connectedSources.clear();
    B.connectedSourcesStamp = B.programStamp;
    B.connectedSourcesFilled = true;
    if (!B.stage) {
        return B.connectedSources;
    }
    std::vector<UsdAttribute> pending;
    for (const SdfPath &path : B.resolvedRoutedPrims) {
        const UsdPrim prim = B.stage->GetPrimAtPath(path);
        if (!prim) {
            continue;
        }
        for (const UsdAttribute &attribute : prim.GetAuthoredAttributes()) {
            if (attribute.HasAuthoredConnections()) {
                pending.push_back(attribute);
            }
        }
    }
    SdfPathVector connections;
    while (!pending.empty()) {
        const UsdAttribute attribute = std::move(pending.back());
        pending.pop_back();
        connections.clear();
        attribute.GetConnections(&connections);
        for (const SdfPath &target : connections) {
            if (!B.connectedSources.insert(target).second ||
                !target.IsPropertyPath()) {
                continue;
            }
            const UsdAttribute next = B.stage->GetAttributeAtPath(target);
            if (next && next.HasAuthoredConnections()) {
                pending.push_back(next);
            }
        }
    }
    return B.connectedSources;
}

// A prim resync can be a declaration elsewhere in the stage. Only a
// namespace disjoint from every compiled reader and source is provably
// neutral; unknown composition changes keep the conservative stamp path.
bool
_UnrelatedPrimResync(const RigExecBakedProgramImpl &B, const SdfPath &path)
{
    if (!B.stage || !path.IsPrimPath() || path.IsAbsoluteRootPath() ||
        UsdPrim::IsPathInPrototype(path) || B.assetRootPath.IsEmpty())
        return false;
    const auto overlaps = [&path](const SdfPath &source) {
        return !source.IsEmpty() &&
               (source.HasPrefix(path) || path.HasPrefix(source));
    };
    if (overlaps(B.assetRootPath))
        return false;
    // The rebuild entry at / records stage metadata read by exact path,
    // not ownership of every prim subtree. Actual root notices are still
    // rejected above and by the value router.
    for (const auto &source : B.rebuild)
        if (!source.IsAbsoluteRootPath() && overlaps(source))
            return false;
    for (const std::set<SdfPath> *index :
         {&B.named, &B.prims, &B.xformPrims, &B.resolvedRoutedPrims})
        if (std::any_of(index->begin(), index->end(), overlaps))
            return false;
    for (const auto &entry : B.leafByPath)
        if (overlaps(entry.first))
            return false;
    for (const auto &entry : B.overridableInputs)
        if (overlaps(entry.first))
            return false;
    // Connection targets enter this set even if their attributes do not yet
    // exist. Defining such a prim must never be classified as unrelated.
    const auto &connected = _ConnectedSources(B);
    return std::none_of(connected.begin(), connected.end(), overlaps);
}

// Declaring an unrelated attribute changes no compiled source route.
// Whole-prim and connection readers retain the conservative stamp path.
bool
_UnrelatedAttributeResync(const RigExecBakedProgramImpl &B,
                          const UsdNotice::ObjectsChanged &notice,
                          const SdfPath &path)
{
    if (!B.stage || !path.IsPropertyPath() || B.named.count(path) ||
        B.rebuild.count(path) || B.leafByPath.count(path) ||
        B.overridableInputs.count(path) || B.sourceBackedPaths.count(path) ||
        _ComposesJudgedXform(B, path) ||
        B.resolvedRoutedPrims.count(path.GetPrimPath()) ||
        _ConnectedSources(B).count(path))
        return false;
    const UsdAttribute attribute = B.stage->GetAttributeAtPath(path);
    if (!attribute || !attribute.GetPrim().IsActive() ||
        attribute.HasAuthoredConnections())
        return false;
    for (const TfToken &field : notice.GetChangedFields(path))
        if (field != SdfFieldKeys->Default && field != SdfFieldKeys->TypeName &&
            field != SdfFieldKeys->Custom && field != SdfFieldKeys->Variability)
            return false;
    return true;
}

// Decides where a stage VALUE edit lands in the program without writing
// anything, so the dry run and ApplyValueEdits share every check by
// construction. The caller has already asked IsInvalidatedBy, so no path
// here is one whose value the bake captured.
// Fills \p indices with the override numbers of the per-frame inputs the
// notice reached, and \p readPaths with every property path it names that
// something in the program can read -- the edited inputs, the properties of
// a prim read through the generation's resolved inputs, the sources those
// reads reach through connections, and the properties of a prim a
// value-compared source reads whole. A property that is none of these is
// read by nothing, and is in neither list. Returns false when the
// notice holds anything that can only be answered by running the whole
// program once (a stamp bump).
bool
_RouteValueEdits(const RigExecBakedProgramImpl &B,
                 const UsdNotice::ObjectsChanged &notice,
                 std::vector<int> *indices, std::vector<SdfPath> *readPaths,
                 std::vector<SdfPath> *leafPaths = nullptr,
                 const std::set<SdfPath> *skip = nullptr)
{
    const auto skipped = [skip](const SdfPath &path) {
        return skip && skip->count(path) > 0;
    };
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty())
        return false;
    for (const auto &path : notice.GetResyncedPaths())
        if (!skipped(path) && !_FixedNumericSourceResync(B, notice, path) &&
            !_RoutableResync(B, notice, path) &&
            !_UnrelatedPrimResync(B, path) &&
            !_UnrelatedAttributeResync(B, notice, path))
            return false;
    const TfToken &kDefault = _tokens->default_;
    const TfToken &kTimeSamples = _tokens->timeSamples;
    const TfToken &kSpline = _tokens->spline;
    // A prim some source of the program reads as a whole rather than by
    // property name: the Xforms accepted for composing to the identity, a
    // prim whose reads all go through the resolved inputs (a property-chain
    // mover, a geometry mover, a weight object), and every prim the bake
    // read from or an ancestor of one -- a transform read relative to the
    // asset root sees every Xform above the prim.
    const auto readWhole = [&B](const SdfPath &prim) {
        if (B.xformPrims.count(prim) || B.resolvedRoutedPrims.count(prim)) {
            return true;
        }
        const auto below = B.prims.lower_bound(prim);
        return below != B.prims.end() && below->HasPrefix(prim);
    };
    std::vector<int> decided;
    std::vector<SdfPath> read;
    SdfPathVector edits(notice.GetResyncedPaths().begin(),
                        notice.GetResyncedPaths().end());
    edits.insert(edits.end(), notice.GetChangedInfoOnlyPaths().begin(),
                 notice.GetChangedInfoOnlyPaths().end());
    for (const SdfPath &path : edits) {
        if (skipped(path))
            continue;
        const std::vector<TfToken> fields = notice.GetChangedFields(path);
        const bool fixedResync = _FixedNumericSourceResync(B, notice, path);
        // IsInvalidatedBy has already ruled the notice's structure out, so a
        // value spec appearing on a live input's walk is placed exactly like
        // a changed value. Any other input may hold a query pinned to the
        // spec it resolved at bake, which a new stronger spec escapes, so
        // its resync is answered by running the whole program.
        const bool liveResync = _RoutableResync(B, notice, path);
        if (_UnrelatedAttributeResync(B, notice, path))
            continue;
        // Layer metadata: the time codes, the frame rate -- read by every
        // time conversion and indexed nowhere.
        if (path.IsAbsoluteRootPath()) {
            return false;
        }
        if (!path.IsPropertyPath()) {
            // The empty info every ancestor of an edited spec reports moves
            // nothing. Prim metadata on a prim nothing reads moves nothing
            // either; on one the program reads, it is not a value the index
            // can place.
            if (fields.empty() || !readWhole(path.GetPrimPath())) {
                continue;
            }
            return false;
        }
        for (const TfToken &field : fields) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline &&
                !(fixedResync && field == SdfFieldKeys->TypeName) &&
                !(liveResync && (field == SdfFieldKeys->TypeName ||
                                 field == SdfFieldKeys->Custom ||
                                 field == SdfFieldKeys->Variability))) {
                return false;
            }
        }
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end()) {
            // A per-frame input, on its head or on an upstream hop of its
            // walk: routed exactly where a drag on it would be, provided
            // every input the path feeds has a reader that re-reads it.
            for (const int index : found->second) {
                if (index < 0 || size_t(index) >= B.cones.editRoute.size() ||
                    !B.cones.editRoute[size_t(index)]) {
                    return false;
                }
                decided.push_back(index);
            }
            read.push_back(path);
            continue;
        }
        const SdfPath prim = path.GetPrimPath();
        // Read through the resolved inputs, which the prologue rebuilds
        // every run: the property chains compare their results, and the
        // geometry movers and weight objects are assembled by sources that
        // compare what they assembled.
        if (B.resolvedRoutedPrims.count(prim)) {
            read.push_back(path);
            continue;
        }
        // Asked about by name and not an input a step declares: a reader
        // the index cannot name, so the whole program answers it.
        if (B.named.count(path)) {
            return false;
        }
        // Not read by name at all. A source that reads its prim whole -- a
        // stage transform, a chain's base points -- re-reads it every run
        // and compares by value, and so does a resolved-input reader whose
        // connection reaches the property, so the program owes either
        // nothing; the frame cache still does, and the path is reported for
        // that. A constraint's raw arrays are not re-read every run: they
        // are read again because this notice advanced the stage edit serial
        // they are kept under (ConstraintArrays).
        if (readWhole(prim) || (!B.resolvedRoutedPrims.empty() &&
                                _ConnectedSources(B).count(path))) {
            read.push_back(path);
        } else if (leafPaths && B.leafByPath.count(path)) {
            // A hop only a leaf reads (a shader dial on a prim nothing else
            // reads): the leaf re-reads it, which is all the program owes.
            leafPaths->push_back(path);
        }
    }
    if (indices) {
        *indices = std::move(decided);
    }
    if (readPaths) {
        *readPaths = std::move(read);
    }
    return true;
}

} // namespace

bool
RigExecBakedProgram::DryRunValueEdits(const UsdNotice::ObjectsChanged &notice,
                                      std::vector<SdfPath> *readPaths) const
{
    return _RouteValueEdits(*_impl, notice, nullptr, readPaths);
}

namespace {

bool
_ApplyValueEdits(RigExecBakedProgramImpl &B,
                 const UsdNotice::ObjectsChanged &notice,
                 const std::set<SdfPath> *skip)
{
    std::vector<int> indices;
    std::vector<SdfPath> read, leafOnly;
    if (!_RouteValueEdits(B, notice, &indices, &read, &leafOnly, skip)) {
        return false;
    }
    // UsdAttributeQuery caches resolve information. Refresh only named direct
    // source bindings on this owning thread before sampling their exact leaves.
    _ForEachFixedNumericSource(B, [&](auto &input) {
        if (std::find(read.begin(), read.end(), input.head.GetPath()) !=
            read.end())
            input.query = UsdAttributeQuery(input.head);
    });
    // A spec that appeared on a per-frame input's walk: the input's query is
    // re-selected over the walk as it stands now, as the bake selected it.
    std::set<int> requery;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (skip && skip->count(path))
            continue;
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end())
            requery.insert(found->second.begin(), found->second.end());
    }
    if (!requery.empty()) {
        frozenDetail::_ForEachPatchableInput(B, [&](auto &input) {
            using T = std::decay_t<decltype(input.constant)>;
            if (!input.query.IsValid() || !input.head ||
                !requery.count(input.overrideIndex))
                return;
            bool viaChain = false, varying = false;
            UsdAttribute selected;
            RigExecBakedClassifyInput<T>(input.head, UsdTimeCode::Default(),
                                         B.chainTargets, &viaChain, &varying,
                                         &selected);
            if (selected)
                input.query = UsdAttributeQuery(selected);
        });
    }
    // The leaves the edit reached, re-read by the next sample: every leaf
    // filed under an edited path, and on a routed prim -- whose edits carry
    // no number -- every leaf with a path on that prim.
    const auto mark = [&B](const std::vector<uint32_t> &ids) {
        for (const uint32_t id : ids) {
            RigExecBakedMarkLeaf(&B, id);
        }
    };
    for (const SdfPath &path : read) {
        const auto found = B.leafByPath.find(path);
        if (found != B.leafByPath.end()) {
            mark(found->second);
            continue;
        }
        const SdfPath prim = path.GetPrimPath();
        if (!B.resolvedRoutedPrims.count(prim)) {
            continue;
        }
        for (auto it = B.leafByPath.lower_bound(prim);
             it != B.leafByPath.end() && it->first.HasPrefix(prim); ++it) {
            if (it->first.GetPrimPath() == prim) {
                mark(it->second);
            }
        }
    }
    for (const SdfPath &path : leafOnly) {
        mark(B.leafByPath.at(path));
    }
    if (indices.empty()) {
        return true;
    }
    B.edited.resize(B.overridden.size(), 0);
    B.editSerial.resize(B.overridden.size(), 0);
    ++B.valueEditSerial;
    for (const int index : indices) {
        B.edited[size_t(index)] = 1;
        B.editSerial[size_t(index)] = B.valueEditSerial;
    }
    B.anyEdited = true;
    return true;
}

} // namespace

bool
RigExecBakedProgram::ApplyValueEdits(const UsdNotice::ObjectsChanged &notice)
{
    return _ApplyValueEdits(*_impl, notice, nullptr);
}

namespace {

// One captured avar, moved to a new constant.
// Three copies of the value exist and all three move: the binding (what a
// drag's release writes back from), the constant table (what a rebuild would
// have captured), and the working table the frame reads. The working slot is
// left alone while an override stands on it -- the drag owns that slot, and
// the constant pass that runs when it is released writes this new constant
// back, which is exactly the value an authored edit under a drag should land
// on. Nothing is marked dirty: the frame path compares every avar slot with
// last run's by VALUE, which is the whole of what the cone needs.
// \p animated moves the binding the other way instead: the avar has become a
// function of time (Animation mode keys a released drag as a spline knot),
// so from now on the frame reads it the long way, exactly as a rebuild would
// have bound it. And an animated binding that is given a plain value again
// (the undo of that key) comes back to being a constant here.
void
RigExecProgramAvarPatch(RigExecBakedProgramImpl *B, size_t bindingIndex,
                        double value, bool animated)
{
    ++B->avarConstantSerial;
    RigExecBakedProgramImpl::AvarBinding &binding =
        B->avarConstantBindings[bindingIndex];
    // The binding's leaf is re-read by the next sample, whichever way the
    // patch moves the binding below.
    if (binding.input.leaf >= 0) {
        RigExecBakedMarkPoolLeaf(&B->leaves.Of<double>(),
                                 size_t(binding.input.leaf));
    }
    const auto promoted = std::find(B->promotedAvars.begin(),
                                    B->promotedAvars.end(), bindingIndex);
    if (animated) {
        if (promoted == B->promotedAvars.end()) {
            binding.input.varying = true;
            binding.input.query = UsdAttributeQuery();
            binding.input.resolvedAttr = binding.input.head;
            B->promotedAvars.push_back(bindingIndex);
            ++B->varyingInputs;
        }
        // The frame's read decides the working value; it compares by value
        // against the last run like any other avar.
        return;
    }
    if (promoted != B->promotedAvars.end()) {
        binding.input.varying = false;
        binding.input.resolvedAttr = UsdAttribute();
        B->promotedAvars.erase(promoted);
        --B->varyingInputs;
    }
    binding.input.constant = value;
    B->avarConstants[binding.slot] = value;
    const int overrideIndex = binding.input.overrideIndex;
    // A drag or an upstream value standing on the avar keeps its slot: the
    // patched constant is the authored value beneath it.
    const auto flagged = [overrideIndex](const std::vector<char> &flags) {
        return overrideIndex >= 0 && size_t(overrideIndex) < flags.size() &&
               flags[size_t(overrideIndex)];
    };
    const bool dragged = flagged(B->overridden) || flagged(B->upstreamOn);
    if (!dragged) {
        B->avars[binding.slot] = value;
    }
}

} // namespace

bool
RigExecBakedProgram::IsPatchableAvarPath(const SdfPath &path) const
{
    return _impl->patchableAvars.find(path) != _impl->patchableAvars.end();
}

namespace {

// One decided patch: the binding to write, the value to write, and the
// property path the notice named. Decided by _DryRunAvarPatches without
// writing anything, so the dry-run query and ApplyAvarValueEdits share
// every check by construction.
struct _AvarPatch {
    size_t binding = 0;
    double value = 0.0;
    bool animated = false;
    SdfPath path;
};

// Decide EVERYTHING before writing anything: a notice this cannot fully
// absorb must leave the table exactly as the rebuild path expects it.
//
// With \p taken, a property that is not a patchable avar and that the bake
// did not capture is left for the value router instead of declining the
// notice, and the patched paths are filed there.
bool
_DryRunAvarPatches(const RigExecBakedProgramImpl &B,
                   const UsdNotice::ObjectsChanged &notice,
                   std::vector<_AvarPatch> *patches,
                   std::set<SdfPath> *taken = nullptr)
{
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    const TfToken &kDefault = _tokens->default_;
    const TfToken &kTypeName = _tokens->typeName;
    const TfToken &kSpline = _tokens->spline;
    const TfToken &kTimeSamples = _tokens->timeSamples;

    std::vector<_AvarPatch> decided;
    // A property resync is a spec appearing or going away under a value:
    // the first value a layer authors for an avar, and the undo that
    // removes it. Only the property itself may resync (a prim resync is
    // structure), and it may name only its type besides the value. What the
    // attribute composes to afterwards is checked below exactly as for a
    // plain value edit, so a spec that brings a connection or a spline, or
    // takes the last value away, still declines.
    std::vector<std::pair<SdfPath, bool>> edited;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath()) {
            if (taken)
                continue; // the router's to judge
            return false;
        }
        edited.emplace_back(path, true);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        edited.emplace_back(path, false);
    }
    for (const auto &[path, resynced] : edited) {
        if (taken && path.IsPropertyPath() && !B.patchableAvars.count(path)) {
            continue; // the router's to judge
        }
        const bool captured =
            B.rebuild.count(path) || _ComposesJudgedXform(B, path);
        if (!captured) {
            // Prim-level info arrives for the ancestors of an edit (an
            // `over` being touched above it). It moves no value the frame
            // reads, and IsInvalidatedBy ignores it for the same reason.
            if (path.IsPrimPath()) {
                continue;
            }
            // A PROPERTY the bake did not fold: a value the frame path
            // re-reads, or something on a prim it never looked at. That
            // still owes the program a stamp bump, which this entry point
            // promises the caller it does not need -- so it is declined.
            return false;
        }
        const auto found = B.patchableAvars.find(path);
        if (found == B.patchableAvars.end()) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kDefault && field != kSpline &&
                field != kTimeSamples && !(resynced && field == kTypeName)) {
                return false; // a connection...
            }
        }
        const RigExecBakedProgramImpl::AvarBinding &binding =
            B.avarConstantBindings[found->second];
        const UsdAttribute attribute = binding.input.head;
        if (!attribute) {
            return false;
        }
        // A new spec can restate the type, and the slot holds a double.
        if (attribute.GetTypeName() != SdfValueTypeNames->Double) {
            return false;
        }
        // A connection makes the value come from somewhere else, which only
        // a rebuild can walk to.
        SdfPathVector connections;
        if (attribute.HasAuthoredConnections() &&
            attribute.GetConnections(&connections) && !connections.empty()) {
            return false;
        }
        // Animated by a spline or by time samples: promoted to a per-frame
        // read. Time samples on a channel folded for its shape still
        // rebuild: the bake reads their count for more than this slot (a
        // SingleChainIK's autoDetect mode is decided by whether its chain's
        // translates are keyed that way, and splines do not count there).
        const bool sampled = attribute.GetNumTimeSamples() > 0;
        if (sampled && B.shapeFolded.count(path)) {
            return false;
        }
        const bool animated = sampled || attribute.HasSpline();
        if (!animated && attribute.ValueMightBeTimeVarying()) {
            return false; // varies some way this does not know
        }
        // Cleared back to nothing reads as the channel's default, which is
        // the fallback the bake captures for an attribute with no value.
        double value = RigExecBakedAvarDefaults[binding.slot % 11];
        if (!animated) {
            attribute.Get(&value, UsdTimeCode::Default());
        }
        decided.push_back({found->second, value, animated, path});
        // Every other input whose walk reaches this channel through a
        // connection: a folded avar there holds the same value and is
        // patched with it; one read per frame re-reads it, unless a spec
        // just appeared, which its pinned query cannot see; anything else
        // folded it some other way, and only a rebuild re-folds it.
        const auto readers = B.overridableInputs.find(path);
        if (readers == B.overridableInputs.end())
            continue;
        for (const int index : readers->second) {
            if (index == binding.input.overrideIndex)
                continue;
            bool placed = false;
            for (size_t k = 0; k < B.avarConstantBindings.size() && !placed;
                 ++k) {
                if (B.avarConstantBindings[k].input.overrideIndex != index)
                    continue;
                if (B.avarConstantBindings[k].input.head.GetTypeName() !=
                    SdfValueTypeNames->Double)
                    return false;
                // Promoting a reader would read its own head, not the
                // channel its connection reaches.
                if (animated)
                    return false;
                decided.push_back({k, value, false, path});
                placed = true;
            }
            for (size_t k = 0; k < B.avarBindings.size() && !placed; ++k) {
                if (B.avarBindings[k].input.overrideIndex != index)
                    continue;
                if (resynced)
                    return false;
                placed = true;
            }
            if (!placed)
                return false;
        }
    }
    if (decided.empty()) {
        return false;
    }
    if (taken) {
        for (const _AvarPatch &patch : decided) {
            taken->insert(patch.path);
        }
    }
    if (patches) {
        *patches = std::move(decided);
    }
    return true;
}

// The two halves of a mixed notice, decided without writing anything.
bool
_DryRunMixedEdits(const RigExecBakedProgramImpl &B,
                  const UsdNotice::ObjectsChanged &notice,
                  std::vector<_AvarPatch> *patches, std::set<SdfPath> *taken,
                  std::vector<SdfPath> *read)
{
    bool any = false;
    for (const SdfPath &path : notice.GetResyncedPaths())
        any = any || B.patchableAvars.count(path) > 0;
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths())
        any = any || B.patchableAvars.count(path) > 0;
    return any && _DryRunAvarPatches(B, notice, patches, taken) &&
           !_IsInvalidatedBy(B, notice, taken) &&
           _RouteValueEdits(B, notice, nullptr, read, nullptr, taken);
}

} // namespace

bool
RigExecBakedProgram::DryRunAvarValueEdits(
    const UsdNotice::ObjectsChanged &notice,
    std::vector<SdfPath> *patchedPaths) const
{
    std::vector<_AvarPatch> patches;
    if (!_DryRunAvarPatches(*_impl, notice, &patches)) {
        return false;
    }
    if (patchedPaths) {
        patchedPaths->clear();
        for (const _AvarPatch &patch : patches) {
            patchedPaths->push_back(patch.path);
        }
    }
    return true;
}

bool
RigExecBakedProgram::ApplyAvarValueEdits(
    const UsdNotice::ObjectsChanged &notice)
{
    RigExecBakedProgramImpl &B = *_impl;
    std::vector<_AvarPatch> patches;
    if (!_DryRunAvarPatches(B, notice, &patches)) {
        return false;
    }
    for (const _AvarPatch &patch : patches) {
        RigExecProgramAvarPatch(&B, patch.binding, patch.value, patch.animated);
        // Every other leaf whose read reaches the avar (a shader dial, a
        // mover input connected to it) re-reads it too.
        const auto found = B.leafByPath.find(patch.path);
        if (found != B.leafByPath.end()) {
            for (const uint32_t id : found->second) {
                RigExecBakedMarkLeaf(&B, id);
            }
        }
    }
    return true;
}

bool
RigExecBakedProgram::DryRunMixedValueEdits(
    const UsdNotice::ObjectsChanged &notice, std::vector<SdfPath> *paths) const
{
    std::vector<_AvarPatch> patches;
    std::set<SdfPath> taken;
    std::vector<SdfPath> read;
    if (!_DryRunMixedEdits(*_impl, notice, &patches, &taken, &read)) {
        return false;
    }
    if (paths) {
        *paths = std::move(read);
        paths->insert(paths->end(), taken.begin(), taken.end());
    }
    return true;
}

bool
RigExecBakedProgram::ApplyMixedValueEdits(
    const UsdNotice::ObjectsChanged &notice)
{
    RigExecBakedProgramImpl &B = *_impl;
    std::vector<_AvarPatch> patches;
    std::set<SdfPath> taken;
    if (!_DryRunMixedEdits(B, notice, &patches, &taken, nullptr) ||
        !_ApplyValueEdits(B, notice, &taken)) {
        return false;
    }
    for (const _AvarPatch &patch : patches) {
        RigExecProgramAvarPatch(&B, patch.binding, patch.value, patch.animated);
        const auto found = B.leafByPath.find(patch.path);
        if (found != B.leafByPath.end()) {
            for (const uint32_t id : found->second) {
                RigExecBakedMarkLeaf(&B, id);
            }
        }
    }
    return true;
}

void
RigExecBakedProgram::SetPublishWeightFields(bool publish)
{
    _impl->publishWeightFields = publish;
}

bool
RigExecBakedProgram::SetOverrides(
    const std::vector<RigExecValueOverride> &overrides)
{
    RigExecBakedProgramImpl &B = *_impl;
    B.routedOverrides.clear();
    if (overrides.empty()) {
        if (B.anyOverridden) {
            std::fill(B.overridden.begin(), B.overridden.end(), 0);
            B.anyOverridden = false;
        }
        return true;
    }
    if (B.anyOverridden) {
        std::fill(B.overridden.begin(), B.overridden.end(), 0);
        B.anyOverridden = false;
    }
    bool placeable = true;
    for (const RigExecValueOverride &o : overrides) {
        // A computation override names an exec computation, and there is no
        // exec here to hold it against.
        if (o.attribute.IsEmpty()) {
            placeable = false;
            continue;
        }
        const SdfPath path = o.prim.AppendProperty(o.attribute);
        // Folded first: a property can be both a per-frame input and the
        // source of something resolved once, and the once wins.
        if (B.folded.count(path)) {
            placeable = false;
            continue;
        }
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end()) {
            for (int index : found->second) {
                B.overridden[size_t(index)] = 1;
            }
            B.anyOverridden = true;
            continue;
        }
        // Routed readers consult resolved inputs at every connection hop,
        // so an override on their source is placed by that same overlay.
        // It carries no number, so the leaf sampler compares its value.
        if (B.resolvedRoutedPrims.count(o.prim) ||
            RigExecBakedHasExternalInputRoute(B, path) ||
            _ConnectedSources(B).count(path)) {
            B.routedOverrides[path] = o.value;
            continue;
        }
        // Raw chain bases ignore interactive whole-array values.
        // Upstream placement remains the only replacement of the authored base.
        bool rawChainBase = false;
        if (o.value.IsHolding<VtVec3fArray>())
            for (const auto &chain : B.chains)
                if (chain.target == path) {
                    rawChainBase = true;
                    break;
                }
        if (rawChainBase)
            continue;
        placeable = false;
    }
    return placeable;
}

} // namespace rigExec
