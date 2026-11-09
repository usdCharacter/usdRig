// RigExec Hydra publication bridge implementation.
#include "bridge.h"

#include "bridgeGuideGeometry.h"
#include "bridgeGuides.h"
#include "warmIndex.h"

#include "rigExecMath/pointFrame.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/parallel.h"
#include "rigExecScene/authoredRestSpace.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <optional>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <utility>

namespace rigExec {

namespace {

// The state the guide component borrows on every call: the bridge owns the
// stage, the evaluator's serials and mover order, the standing display
// overrides and the weight-overlay selection.
RigExecGuideCall
_MakeGuideCall(const UsdStageRefPtr &stage,
               const RigExecRigEvaluator &evaluator,
               const std::vector<RigExecValueOverride> &displayOverrides,
               const SdfPath &weightOverlay)
{
    RigExecGuideCall call;
    call.stage = stage;
    call.stageEditSerial = evaluator.GetStageEditSerial();
    call.compiledRigSignature = evaluator.GetCompiledRigSignature();
    call.moverOrder = &evaluator.GetMoverOrder();
    call.displayOverrides = &displayOverrides;
    call.weightOverlay = weightOverlay;
    return call;
}

} // namespace

RigExecImagingBridge::RigExecImagingBridge(const UsdStageRefPtr &stage,
                                           const SdfPath &rigPath)
    : RigExecImagingBridge(stage, rigPath,
                           std::make_shared<RigExecSnapshotStore>())
{
}

RigExecImagingBridge::RigExecImagingBridge(
    const UsdStageRefPtr &stage, const SdfPath &rigPath,
    std::shared_ptr<RigExecSnapshotStore> store)
    : _stage(stage), _rigPath(rigPath),
      _evaluator(std::make_unique<RigExecRigEvaluator>(stage, rigPath)),
      _guides(std::make_unique<RigExecBridgeGuides>(rigPath)),
      _store(std::move(store)),
      _frameCache(std::make_shared<RigExecFrameCache>())
{
    // No overlay is selected yet, so nothing consumes the per-point influence
    // field. SetWeightOverlay turns it back on the moment one is.
    _evaluator->SetPublishWeightFields(false);
    // The results scene index draws a deform program in shader mode.
    _evaluator->SetAcceptsGpuDeformPublications(true);
    // Viewport profiling: the evaluator's own phases plus the imaging
    // layer's (see EvaluateAndPublishResult and the registry), dumped with
    // RigExecImaging_WriteProfileSummary. Off by default; recording changes
    // no evaluated value.
    if (TfGetenvBool("RIGEXEC_IMAGING_PROFILE", false)) {
        _evaluator->SetProfilingEnabled(true);
    }
    RefreshFrameCacheSettings();
}

// Out-of-line so the unique_ptr components are destroyed where their types
// are complete.
RigExecImagingBridge::~RigExecImagingBridge() = default;

void
RigExecImagingBridge::RefreshFrameCacheSettings()
{
    _frameCacheMode = RigExecFrameCacheModeFromEnvironment();
    _frameCacheVerify = RigExecFrameCacheVerifyRequested();
}

bool
RigExecImagingBridge::Compile(std::vector<std::string> *errors)
{
    InvalidateGuideCaches();
    return _evaluator->Compile(errors);
}

void
RigExecImagingBridge::AppendAuthoredGuides(UsdTimeCode time,
                                           RigExecImagingSnapshot *snapshot)
{
    const UsdPrim root = _stage->GetPrimAtPath(_rigPath);
    if (!root)
        return;
    InvalidateGuideCaches();
    // Scratch inputs for the shared drawing helpers, never a valid rig pose.
    RigExecRigPose authored;
    authored.time = time;
    const RigExecPointFrame identity;
    for (const UsdPrim &prim : UsdPrimRange(root)) {
        const TfToken type = prim.GetTypeName();
        const bool provider = type == _tControlType || type == _tJointType;
        const bool volume = type == _tSphereWeightType ||
                            type == _tPlaneWeightType ||
                            type == _tCurveWeightType;
        if (!provider && !volume)
            continue;
        bool rigid = false;
        const GfMatrix4d rest =
            RigExecAuthoredGuideRestSpace(prim, time, &rigid);
        if (!rigid)
            continue;
        if (volume) {
            authored.weightFrames.emplace(prim.GetPath(), rest);
        } else {
            RigExecPointFrame frame;
            for (size_t i = 0; i < frame.points.size(); ++i) {
                frame.points[i] = rest.Transform(identity.points[i]);
            }
            if (type == _tControlType) {
                authored.controlFrames.emplace(prim.GetPath(), frame);
            } else {
                authored.jointFramesFinal.emplace(prim.GetPath(), frame);
            }
        }
    }
    const RigExecGuideCall call = _MakeGuideCall(
        _stage, *_evaluator, _interactiveOverrides, _weightOverlay);
    _guides->_FillGuides(call, authored, snapshot);
    _guides->_FillControlGuides(call, authored, snapshot);
    _guides->_FillVolumeGuides(call, authored, snapshot);
    // Manipulators must not mistake authored placements for evaluated frames.
    for (auto &[path, prim] : snapshot->prims) {
        prim.hasControlFrame = false;
        prim.hasVolumeGuideFrame = false;
    }
}

SdfPath
RigExecImagingBridge::GetGeneratedScope() const
{
    return _rigPath.AppendChild(_tRigExecGenerated);
}

// Constraint-driven transforms, published onto the prim itself so parented
// geometry rides along. Base and revision travel together: the scene index
// sits downstream of Hydra's flatten and turns the pair into a world-space
// delta (see RigExecPublishedPrim::xformBase).
void
RigExecImagingBridge::_FillProviderXforms(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    for (const auto &[providerPath, matrix] : pose.providerXforms) {
        const auto baseIt = pose.providerBaseXforms.find(providerPath);
        if (baseIt == pose.providerBaseXforms.end()) {
            // The evaluator writes both or neither; a revision without its
            // base cannot be turned into a delta, so publishing it would move
            // the prim by an arbitrary amount.
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[providerPath];
        published.xform = matrix;
        published.xformBase = baseIt->second;
        published.hasXform = true;
        snapshot->hasDrivenXforms = true;
    }
    // Flattening stamps resetXformStack=true on every output, so the results
    // index cannot recover authored reset boundaries from Hydra. Capture
    // them while the bridge still owns the source stage, under driven roots.
    // The walk runs when the driven set or the stage changes, not every
    // frame: any authored edit advances the evaluator's stage edit serial.
    std::vector<SdfPath> providerRoots;
    providerRoots.reserve(pose.providerXforms.size());
    for (const auto &[providerPath, matrix] : pose.providerXforms) {
        providerRoots.push_back(providerPath);
    }
    const uint64_t serial = _evaluator->GetStageEditSerial();
    if (!_xformResetsValid || _xformResetSerial != serial ||
        providerRoots != _xformResetRoots) {
        _xformResetPaths.clear();
        std::vector<SdfPath> scanned;
        for (const SdfPath &providerPath : providerRoots) {
            bool covered = false;
            for (const auto &root : scanned)
                covered = covered || providerPath.HasPrefix(root);
            if (covered)
                continue;
            const auto root = _stage->GetPrimAtPath(providerPath);
            if (!root)
                continue;
            scanned.push_back(providerPath);
            for (const auto &prim : UsdPrimRange(root)) {
                const UsdGeomXformable xform(prim);
                if (xform && xform.GetResetXformStack())
                    _xformResetPaths.insert(prim.GetPath());
            }
        }
        _xformResetRoots = std::move(providerRoots);
        _xformResetSerial = serial;
        _xformResetsValid = true;
    }
    snapshot->xformResetPaths.insert(_xformResetPaths.begin(),
                                     _xformResetPaths.end());
}

void
RigExecImagingBridge::InvalidateGuideCaches()
{
    _guides->InvalidateGuideCaches();
}

void
RigExecImagingBridge::NoteGuideEdits(const UsdNotice::ObjectsChanged &notice)
{
    _guides->NoteGuideEdits(notice);
}

void
RigExecImagingBridge::SetInteractiveOverrides(
    std::vector<RigExecValueOverride> overrides)
{
    _interactiveOverrides = std::move(overrides);
    _evaluationOverrides.clear();
    // guide:* inputs are display-only: imaging reads them, the graph never does.
    for (const auto &value : _interactiveOverrides) {
        if (std::strncmp(value.attribute.GetText(), "guide:", 6) != 0)
            _evaluationOverrides.push_back(value);
    }
    _evaluator->SetInteractiveOverrides(_evaluationOverrides);
}

// Stamps WHICH STAGE and WHICH TIME a generation describes onto it.
// The store is process-global, so a consumer that finds a prim by path
// alone -- the compute-extent callback is the one USD calls with a stage
// and time of its own choosing -- would otherwise accept whatever rig
// published last, for whatever frame.
void
RigExecImagingBridge::_StampGeneration(UsdTimeCode time,
                                       RigExecImagingSnapshot *snapshot) const
{
    snapshot->stage = UsdStageWeakPtr(_stage);
    snapshot->sampleTimeIsDefault = time.IsDefault();
    snapshot->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();
}

void
RigExecImagingBridge::SetWarmFrameIndex(
    std::shared_ptr<RigExecWarmFrameIndex> index)
{
    _warmIndex = std::move(index);
}

void
RigExecImagingBridge::ClearFrameCache()
{
    if (_frameCache) {
        _frameCache->Clear();
    }
    _taskListMemo.Clear();
    _lastPublishedCompiledRig.store(0, std::memory_order_relaxed);
    _freshDigests.clear();
    _freshCompiledRigValid = false;

    _sampleSupportValid = false;
}

void
RigExecImagingBridge::RetirePoseOnlyCacheEntries()
{
    if (_warmIndex && _frameCache) {
        for (const auto &entry : _warmIndex->CompletedKeys(_rigPath)) {
            RigExecEntryProvenance provenance;
            if (_frameCache->LookupProvenance(entry.second, &provenance) &&
                provenance.unfoldedControlDigest == 0 &&
                entry.first.IsNumeric()) {
                _warmIndex->NoteDirtied(_rigPath, entry.first.GetValue());
            }
        }
    }
    for (auto it = _freshDigests.begin(); it != _freshDigests.end();) {
        if (!it->second.sampledInputs) {
            it = _freshDigests.erase(it);
        } else {
            ++it;
        }
    }
}

void
RigExecImagingBridge::NoteWarmingEnqueued(UsdTimeCode time,
                                          uint64_t controlDigest,
                                          uint64_t unfoldedDigest,
                                          const RigExecFrameInputs &inputs)
{
    if (!_frameCache || _frameCacheMode == RigExecFrameCacheMode::Off) {
        return;
    }
    // Scope check, as on the lookup and memoization paths: the evaluation
    // compiled rig may have moved since the last proof was recorded.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    if (!_freshCompiledRigValid || _freshCompiledRig != compiledRig) {
        _freshDigests.clear();
        _freshCompiledRig = compiledRig;
        _freshCompiledRigValid = true;
    }
    if (_freshDigests.size() >= _kFreshDigestCap) {
        _freshDigests.clear();
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        controlDigest, unfoldedDigest, &inputs, &_evaluationOverrides);
}

const RigExecOutputAffectedIndex *
RigExecImagingBridge::_SyncAffectedIndex()
{
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        _affectedIndex.reset();
        _affectedValid = false;
        return nullptr;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    // The program object test matters as much as the compiled rig: a rebuild can
    // answer a colliding compiled rig for wholly new clustering, and an index
    // built for the old shape would retire against the wrong cones.
    // (The compiled rig signature folds the build count, so a rebuild moves it in
    // practice; the pointer pins the shape regardless.)
    if (_affectedValid && _affectedIndex &&
        _affectedCompiledRig == compiledRig && _affectedProgram == program) {
        return _affectedIndex.get();
    }
    _affectedIndex.reset(new RigExecOutputAffectedIndex());
    _affectedIndex->Build(impl, compiledRig);
    _affectedCompiledRig = compiledRig;
    _affectedProgram = program;
    _affectedValid = true;
    // A rebuild drops every admission with the old index: the standing
    // overrides re-admit against the new shape, so a drag straddling a
    // rebuild keeps its exact seeds.
    AdmitOverrideControls(_evaluationOverrides);
    return _affectedIndex.get();
}

const RigExecOutputAffectedIndex *
RigExecImagingBridge::GetAffectedIndex()
{
    return _SyncAffectedIndex();
}

void
RigExecImagingBridge::AdmitOverrideControls(
    const std::vector<RigExecValueOverride> &overrides)
{
    if (overrides.empty()) {
        return;
    }
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        return;
    }
    const RigExecOutputAffectedIndex *synced = _SyncAffectedIndex();
    if (!synced || synced->Empty()) {
        // No oracle, no admission: the overrides stay foreign and every
        // plan over them stays conservative, as before.
        return;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    for (const RigExecValueOverride &o : overrides) {
        const RigExecControlId id = RigExecControlIdForOverride(o);
        if (_affectedIndex->IsKnownControl(id)) {
            continue;
        }
        const std::vector<int> seeds = RigExecOverrideSeeds(impl, o);
        if (seeds.empty()) {
            // Places nowhere (folded, exec-typed, routed, or unknown):
            // left foreign, never admitted seedless. A seedless
            // admission would claim the override reaches nothing,
            // which only the placement below can prove.
            continue;
        }
        _affectedIndex->MapControl(id, seeds);
    }
}

size_t
RigExecImagingBridge::RetireProofsForControls(
    const std::vector<RigExecControlId> &controls)
{
    if (_freshDigests.empty() || controls.empty()) {
        return 0;
    }
    // Each doomed id once, beside its path when the id spells one
    // canonically: then the id equals a sample path's text exactly when
    // the paths are equal, and an id that spells none equals no sample
    // path's text, so it can only name an override.
    std::vector<std::pair<std::string, SdfPath>> doomed;
    {
        const std::set<std::string> unique(controls.begin(), controls.end());
        doomed.reserve(unique.size());
        for (const std::string &id : unique) {
            SdfPath path =
                SdfPath::IsValidPathString(id) ? SdfPath(id) : SdfPath();
            if (!path.IsEmpty() && path.GetString() != id) {
                path = SdfPath();
            }
            doomed.emplace_back(id, std::move(path));
        }
    }
    size_t retired = 0;
    for (auto it = _freshDigests.begin(); it != _freshDigests.end();) {
        bool hit = false;
        for (const auto &[id, path] : doomed) {
            if (it->second.Covers(id, path)) {
                hit = true;
                break;
            }
        }
        if (hit) {
            it = _freshDigests.erase(it);
            ++retired;
        } else {
            ++it;
        }
    }
    return retired;
}

bool
RigExecImagingBridge::RepointProof(UsdTimeCode time, uint64_t newDigest)
{
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const auto found = _freshDigests.find({time.IsDefault(), stamped});
    if (found == _freshDigests.end()) {
        return false;
    }
    // The digest names the new namespace; the unfolded half and the paths
    // are untouched -- the sampled values did not move, only the constants
    // they fold beside did. A second carry re-folds the same unfolded half
    // against the newer constants.
    found->second.digest = newDigest;
    return true;
}

bool
RigExecImagingBridge::GetFreshProof(UsdTimeCode time,
                                    RigExecFreshProof *proof) const
{
    if (!proof) {
        return false;
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const auto found = _freshDigests.find({time.IsDefault(), stamped});
    if (found == _freshDigests.end()) {
        return false;
    }
    *proof = found->second;
    return true;
}

bool
RigExecImagingBridge::NoteCaptureIndex(const UsdNotice::ObjectsChanged &notice)
{
    // First and unconditionally: the pinned bindings serve every live sample,
    // with or without a frame cache.
    _NoteChainBindings(notice);
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program || !_frameCache) {
        return false;
    }
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    const bool hit = RigExecNoteCaptureIndex(*_frameCache, _taskListMemo,
                                             compiledRig, *program, notice);
    if (hit && _freshCompiledRigValid && _freshCompiledRig == compiledRig) {
        // The proofs name evicted entries: drop them so the next visit
        // misses without paying a sample for the proof check.
        _freshDigests.clear();
    }
    // Compiled rig drift since the last memoization (an in-place avar patch, a
    // rebuild, a structural edit): the old half's frames are unreachable by
    // construction, so evict them eagerly rather than waiting for LRU.
    const uint64_t last =
        _lastPublishedCompiledRig.load(std::memory_order_relaxed);
    if (last != compiledRig) {
        _frameCache->EvictCompiledRig(last);
        _taskListMemo.InvalidateCompiledRig(last);
        if (_freshCompiledRigValid && _freshCompiledRig == last) {
            _freshDigests.clear();
        }
    }
    return hit;
}

RigExecFrameCacheStats
RigExecImagingBridge::GetFrameCacheStats() const
{
    if (!_frameCache) {
        return RigExecFrameCacheStats();
    }
    return _frameCache->Stats();
}

void
RigExecImagingBridge::_NoteChainBindings(
    const UsdNotice::ObjectsChanged &notice)
{
    if (!_liveChainValid) {
        return;
    }
    // Whether an edit to `path` -- a value, a connection, or a property spec
    // created or removed -- can move a bound chain: any property of a chain
    // mover, a chain target itself, or anything under a weight object. A
    // mover's connected input reads its source live, so a value beside a
    // target on the same prim (a switch on the control a chain writes to)
    // moves no binding; prim-level info on a mover or a target's prim
    // still does.
    const auto touchesChain = [this](const SdfPath &path) {
        const SdfPath prim = path.GetPrimPath();
        const auto has = [](const std::vector<SdfPath> &list,
                            const SdfPath &key) {
            return std::binary_search(list.begin(), list.end(), key);
        };
        if (path.IsPropertyPath()
                ? has(_liveChainMovers, prim) || has(_liveChainTargets, path)
                : has(_liveChainPrims, prim)) {
            return true;
        }
        for (const SdfPath &weight : _liveChainWeights) {
            if (prim.HasPrefix(weight)) {
                return true;
            }
        }
        return false;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        // A PRIM resync can add, remove or retype a mover or a target before
        // the compiled rig signature has been recomputed: rebind on the next sample. A
        // PROPERTY resync is a spec created or removed -- the first time an
        // avar is authored in a session is one -- and is judged like a value
        // edit, by its prim.
        if (!path.IsPropertyPath() || touchesChain(path)) {
            _liveChainValid = false;
            return;
        }
    }
    // A value or connection edit on a chain-relevant prim can change a
    // folded constant, a connection, or a weight object's answer -- none of
    // which moves the compiled rig signature. Anything else (an avar on a control, a
    // pose edit) leaves every bound chain exactly as it was.
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (touchesChain(path)) {
            _liveChainValid = false;
            return;
        }
    }
}

bool
RigExecImagingBridge::_EnsureChainBindings() const
{
    const size_t compiledRig = _evaluator->GetCompiledRigSignature();
    if (_liveChainValid && _liveChainCompiledRig == compiledRig) {
        return true;
    }
    _liveChainValid = false;
    _liveChainPrims.clear();
    _liveChainMovers.clear();
    _liveChainTargets.clear();
    _liveChainWeights.clear();
    RigExecChainSampleBindings bound;
    if (!RigExecBindChainSampleInputs(*_evaluator, &bound)) {
        return false;
    }
    for (const RigExecChainSampleChain &chain : bound.chains) {
        _liveChainPrims.push_back(chain.targetPath.GetPrimPath());
        _liveChainTargets.push_back(chain.targetPath);
        for (const RigExecChainSampleRevision &revision : chain.revisions) {
            _liveChainPrims.push_back(revision.moverPath);
            _liveChainMovers.push_back(revision.moverPath);
            _liveChainWeights.insert(_liveChainWeights.end(),
                                     revision.weightObjects.begin(),
                                     revision.weightObjects.end());
        }
    }
    for (std::vector<SdfPath> *list :
         {&_liveChainPrims, &_liveChainMovers, &_liveChainTargets,
          &_liveChainWeights}) {
        std::sort(list->begin(), list->end());
        list->erase(std::unique(list->begin(), list->end()), list->end());
    }
    _liveChainBindings = std::move(bound);
    _liveChainCompiledRig = compiledRig;
    _liveChainValid = true;
    ++_liveChainGeneration;
    return true;
}

const RigExecChainSampleBindings *
RigExecImagingBridge::AcquireChainBindings(uint64_t *generation) const
{
    if (!_EnsureChainBindings()) {
        return nullptr;
    }
    if (generation) {
        *generation = _liveChainGeneration;
    }
    return &_liveChainBindings;
}

bool
RigExecImagingBridge::_SampleLive(UsdTimeCode time,
                                  RigExecFrameInputs *out) const
{
    // The upstream values ride the vector (RigExecFrameInputs::upstream),
    // admitted as live admits them, so every digest of it folds them.
    if (_EnsureChainBindings() &&
        RigExecSampleFrameInputsWithTrustedChainBindings(
            *_evaluator, time, _evaluationOverrides, _upstreamInputs,
            _liveChainBindings, out)) {
        return true;
    }
    // The pinned route declined (a bind that failed, an override it cannot
    // place, a hook that refused): the self-binding sampler answers exactly
    // as it always has, at its own cost.
    return RigExecSampleFrameInputs(*_evaluator, time, _evaluationOverrides,
                                    _upstreamInputs, out);
}

void
RigExecImagingBridge::SetUpstreamInputs(
    std::vector<RigExecUpstreamValue> values)
{
    std::vector<RigExecValueOverride> inputs;
    inputs.reserve(values.size());
    for (const RigExecUpstreamValue &value : values) {
        inputs.push_back(
            RigExecValueOverride{value.path.GetPrimPath(), TfToken(),
                                 value.path.GetNameToken(), value.value});
    }
    _upstreamInputs = std::move(values);
    _evaluator->SetUpstreamInputs(std::move(inputs));
}

bool
RigExecImagingBridge::_CanSampleCacheInputs()
{
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program || _evaluator->cpuReference) {
        return false;
    }
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    // Whether the program freezes is a question about connections and
    // prims, which no value edit moves.
    const uint64_t serial = _evaluator->GetStageStructureSerial();
    if (!_sampleSupportValid || _sampleSupportProgram != program ||
        _sampleSupportCompiledRig != compiledRig ||
        _sampleSupportSerial != serial) {
        _sampleSupport = RigExecCanFreezeProgram(*_evaluator);
        _sampleSupportProgram = program;
        _sampleSupportCompiledRig = compiledRig;
        _sampleSupportSerial = serial;
        _sampleSupportValid = true;
    }
    return _sampleSupport;
}

bool
RigExecImagingBridge::_ComputePoseOnlyCacheKey(UsdTimeCode time,
                                               RigExecFrameCacheKey *key) const
{
    if (!key || !_frameCache ||
        !RigExecRefusalControlDigestible(_evaluationOverrides,
                                         _upstreamInputs)) {
        return false;
    }
    // Complete live results remain cacheable when an operation has no
    // frozen input contract. Every stage edit moves this namespace; no
    // per-operation input list or partial retained-state proof is needed.
    key->compiledRigSignature =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    key->controlDigest =
        RigExecRefusalControlDigest(time, _evaluator->GetStageEditSerial(),
                                    _evaluationOverrides, _upstreamInputs);
    return true;
}

void
RigExecImagingBridge::_ServeCachedPose(UsdTimeCode time,
                                       const RigExecRigPose &pose,
                                       const RigExecFrameCacheKey &key,
                                       PublishResult *result)
{
    _frameCache->NoteProvenanceAlias(key, time);
    RigExecRigPose cached = pose;
    const bool verify = _frameCacheVerify;
    if (verify) {
        // Verify the cached pose against a live evaluation. This evaluator
        // pull counts as a live publication regardless of its verdict.
        RigExecRigPose live;
        {
            RigExecProfileScope scope(MutableProfiler(), "Imaging.Evaluate",
                                      "imaging");
            live = _evaluator->Evaluate(time);
        }
        const RigExecShadowVerdict verdict =
            RigExecVerifyHitWithLive(cached, [&live]() { return live; });
        if (!verdict.match && verdict.mismatches > 0) {
            TF_WARN("rigExec: frame-cache hit at %s mismatches live "
                    "evaluation; serving live and repairing the entry:\n%s",
                    time.IsDefault() ? "default"
                                     : std::to_string(time.GetValue()).c_str(),
                    verdict.report.c_str());
            _PublishPoseSnapshot(time, verdict.poseToServe, result);
            _MemoizeLiveResult(time, verdict.poseToServe);
            result->cacheHit = false;
            return;
        }
        // A match serves the cached pose (proven identical); an unverified
        // shadow (live failed) serves it too, advisory-only. Either way the
        // evaluator ran, so cacheHit stays false.
        cached.time = time;
        _PublishPoseSnapshot(time, cached, result);
        result->cacheHit = false;
        return;
    }
    // Guide styling reads pose.time; the stored pose was evaluated for an
    // earlier visit, so it is re-stamped for the requested frame before
    // anything reads it.
    cached.time = time;
    _PublishPoseSnapshot(time, cached, result);
    result->cacheHit = true;
}

bool
RigExecImagingBridge::CarryEntry(UsdTimeCode time,
                                 const RigExecFrameCacheKey &oldKey,
                                 uint64_t newConstants)
{
    if (!_frameCache) {
        return false;
    }
    RigExecEntryProvenance provenance;
    if (!_frameCache->LookupProvenance(oldKey, &provenance) ||
        provenance.unfoldedControlDigest == 0) {
        return false;
    }
    RigExecFrameCacheKey newKey;
    newKey.compiledRigSignature = oldKey.compiledRigSignature;
    newKey.controlDigest = RigExecFoldConstantDigest(
        provenance.unfoldedControlDigest, newConstants);
    if (newKey.controlDigest == oldKey.controlDigest) {
        // No namespace move for this entry -- the caller stands it.
        return false;
    }
    RigExecRigPose pose;
    std::shared_ptr<const void> retained;
    size_t bytes = 0;
    if (!_frameCache->Lookup(oldKey, &pose, &retained, &bytes) || !pose.valid ||
        !retained) {
        return false;
    }
    // The rewrite sequence: publish under the new key, evict the old,
    // re-record the completion -- which REPLACES the row, so no Drop (a
    // Drop erases by (compiled rig, time) and would eat the new row). The old key
    // is unreachable by construction past the namespace move (no fresh
    // sample folds the old constants), so the evict frees without robbing
    // aliases: a frame whose values match serves the carried entry.
    RigExecEntryProvenance carried = provenance;
    if (time.IsNumeric()) {
        carried.aliasTimes.clear();
        carried.aliasTimes.push_back(time.GetValue());
    }
    if (!_frameCache->Publish(newKey, time, pose, bytes, retained, &carried)) {
        return false;
    }
    _frameCache->Evict(oldKey);
    if (_warmIndex && time.IsNumeric()) {
        _warmIndex->NoteCompleted(_rigPath, time.GetValue(), newKey,
                                  _warmIndex->CurrentGeneration(_rigPath));
    }
    RepointProof(time, newKey.controlDigest);
    return true;
}

bool
RigExecImagingBridge::_TrySparseServe(UsdTimeCode time,
                                      const RigExecFrameInputs &inputs,
                                      const RigExecFrameCacheKey &key,
                                      uint64_t unfolded, PublishResult *result)
{
    if (!result || !_frameCache || !_warmIndex || !time.IsNumeric()) {
        return false;
    }
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        return false;
    }
    const double stamped = time.GetValue();
    RigExecFrameCacheKey baseKey{0, 0};
    if (!_warmIndex->FindKey(_rigPath, stamped, &baseKey)) {
        return false;
    }
    // Same key, retired proof: the entry stands under the fresh digest
    // (a no-op edit retired the proof without moving values). Serve it
    // and heal the proof -- no re-key, no pulls.
    if (baseKey == key) {
        RigExecRigPose cached;
        bool hit = false;
        {
            RigExecProfileScope scope(MutableProfiler(),
                                      "Imaging.FrameCacheLookup", "imaging");
            hit = _frameCache->Lookup(key, &cached);
        }
        MutableProfiler()->RecordCacheLookup(hit, stamped);
        if (!hit || !cached.valid) {
            return false;
        }
        _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
            key.controlDigest, unfolded, &inputs, &_evaluationOverrides);
        _warmIndex->NoteCompleted(_rigPath, stamped, key,
                                  _warmIndex->CurrentGeneration(_rigPath));
        _ServeCachedPose(time, cached, key, result);
        return true;
    }
    RigExecRigPose base;
    std::shared_ptr<const void> handle;
    bool baseHit = false;
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.FrameCacheLookup",
                                  "imaging");
        baseHit = _frameCache->Lookup(baseKey, &base, &handle);
    }
    if (!baseHit || !base.valid || !handle) {
        // No retained base (evicted, or pose-only): the frame evaluates
        // live. Records the miss: the store was consulted.
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecRetainedFrameState *retained =
        static_cast<const RigExecRetainedFrameState *>(handle.get());
    // The constant gate: sampled values exclude avar constants by design,
    // so a value comparison alone cannot see a constant patch. A moved
    // namespace plans nothing here -- the eager carry covers clean
    // entries, and anything else re-warms.
    if (retained->compiledRigSignature != key.compiledRigSignature ||
        retained->constantDigest !=
            RigExecCompiledRigConstantDigest(*program)) {
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecOutputAffectedIndex *index = _SyncAffectedIndex();
    if (!index || index->Empty()) {
        // An empty index answers empty sets, not all-clusters: planning
        // against it would Hit everything and serve stale poses. Live.
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    if (!RigExecCanReuseRetainedPose(*index, *retained,
                                     key.compiledRigSignature, inputs,
                                     _evaluationOverrides)) {
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    // Proven identical with zero cluster work: re-key under the fresh
    // digest and serve with zero pulls. The old entry is deliberately
    // NOT evicted: another frame's values may still digest to it (an
    // alias serves it directly), and LRU drains what nobody serves.
    RigExecEntryProvenance provenance;
    const bool haveProvenance =
        _frameCache->LookupProvenance(baseKey, &provenance);
    if (haveProvenance) {
        provenance.unfoldedControlDigest = unfolded;
    }
    RigExecRigPose served = base;
    if (_frameCache->Publish(key, time, served, retained->RetainedBytes(),
                             handle, haveProvenance ? &provenance : nullptr)) {
        _warmIndex->NoteCompleted(_rigPath, stamped, key,
                                  _warmIndex->CurrentGeneration(_rigPath));
    }
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        key.controlDigest, unfolded, &inputs, &_evaluationOverrides);
    MutableProfiler()->RecordCacheLookup(true, stamped);
    _ServeCachedPose(time, served, key, result);
    return true;
}

uint64_t
RigExecImagingBridge::_NoteCacheEditSerial()
{
    // A bare bridge has no registry to retire proofs on edits.
    const uint64_t serial = _evaluator->GetStageEditSerial();
    if (!_warmIndex && _cacheEditSerial != serial) {
        _freshDigests.clear();
    }
    _cacheEditSerial = serial;
    return serial;
}

bool
RigExecImagingBridge::_LookupCachedPose(UsdTimeCode time, bool publishing,
                                        RigExecRigPose *pose,
                                        RigExecFrameCacheKey *key,
                                        _CacheMiss *miss)
{
    // Profiler scopes and lane records belong to the publishing lookup
    // only; a query that publishes nothing leaves the profiler alone.
    *miss = _CacheMiss();
    if (!_frameCache || _evaluator->cpuReference) {
        return false;
    }
    // Scope check: a new compiled rig retires every
    // freshness proof. Cached entries stay for LRU (their digests decide
    // reachability); proofs name the old scope's inputs and must not be
    // consulted. No proof can exist yet, so this is a miss either way.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    if (!_freshCompiledRigValid || _freshCompiledRig != compiledRig) {
        _freshDigests.clear();
        _freshCompiledRig = compiledRig;
        _freshCompiledRigValid = true;
        return false;
    }
    const auto storeLookup = [&](const RigExecFrameCacheKey &lookupKey,
                                 std::shared_ptr<const void> *retained) {
        if (!publishing) {
            return _frameCache->Lookup(lookupKey, pose, retained);
        }
        RigExecProfileScope scope(MutableProfiler(), "Imaging.FrameCacheLookup",
                                  "imaging");
        return _frameCache->Lookup(lookupKey, pose, retained);
    };
    // Servable completions skip sampling entirely: the index row names the
    // exact key, and the retirement paths (generation tags, compiled rig tags,
    // path-scoped dirt, eviction) keep a clean row exact, so a store hit
    // under it IS the proof compare. Sampling first here would cost every
    // hit the full input vector, which can exceed the evaluation it saves.
    // A row miss, a store miss under the row (concurrent eviction), or a rig
    // with no baked program falls through to the proof/sparse paths below.
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    const auto proof = _freshDigests.find({time.IsDefault(), stamped});
    const bool haveProof = proof != _freshDigests.end();
    if (haveProof && !proof->second.sampledInputs) {
        if (!_ComputePoseOnlyCacheKey(time, key) ||
            proof->second.digest != key->controlDigest) {
            return false;
        }
        const bool hit = storeLookup(*key, nullptr);
        if (publishing) {
            MutableProfiler()->RecordCacheLookup(hit, stamped);
        }
        if (!hit || !pose->valid) {
            *pose = RigExecRigPose();
            return false;
        }
        return true;
    }
    if (_warmIndex && time.IsNumeric()) {
        RigExecFrameCacheKey cachedKey{0, 0};
        if (_warmIndex->FindCachedKey(_rigPath, stamped,
                                      _warmIndex->CurrentGeneration(_rigPath),
                                      compiledRig, &cachedKey)) {
            std::shared_ptr<const void> retained;
            RigExecFrameCacheKey poseOnlyKey;
            const bool currentPoseOnly =
                _ComputePoseOnlyCacheKey(time, &poseOnlyKey) &&
                cachedKey == poseOnlyKey;
            const bool hit = storeLookup(cachedKey, &retained);
            // A pose-only entry whose proof was retired cannot use an
            // index row as its edit-serial check. Sampled entries carry
            // retained inputs (or their enqueue-time proof).
            if (hit && pose->valid &&
                (currentPoseOnly || (program && (retained || haveProof)))) {
                if (publishing) {
                    MutableProfiler()->RecordCacheLookup(true, stamped);
                }
                *key = cachedKey;
                return true;
            }
            *pose = RigExecRigPose();
            // Store miss under a live row: unrecorded, so the lane keeps
            // one record per SetTime -- the fallthrough records its own.
        }
    }
    // The proof is consulted by time BEFORE anything is sampled: a time with
    // no proof cannot hit whatever its digest says, and a cold frame is the
    // common case on a first scrub. Sampling and digesting it here would
    // make every miss pay the full vector twice (once here, once in the
    // memoization after the live run), which is exactly the cold-frame
    // latency the cache-off path never pays. A time with no proof but WITH
    // a retained base still samples when publishing: the sparse plan may
    // serve it with zero pulls, which is worth one sample.
    RigExecFrameCacheKey baseKey{0, 0};
    // Only the sparse lane reads the base, and it runs only when publishing;
    // a non-publishing query with no proof cannot hit, so it skips the sample.
    miss->haveBase = publishing && _warmIndex && time.IsNumeric() &&
                     _warmIndex->FindKey(_rigPath, stamped, &baseKey);
    if (!haveProof && !miss->haveBase) {
        return false;
    }
    if (!program || !_CanSampleCacheInputs()) {
        return false;
    }
    // One sample serves the proof compare and the sparse plan.
    {
        std::optional<RigExecProfileScope> sampleScope;
        if (publishing) {
            sampleScope.emplace(MutableProfiler(), "Imaging.LookupSampleDigest",
                                "imaging");
        }
        if (!_SampleLive(time, &miss->inputs) ||
            !RigExecControlStateDigestible(miss->inputs,
                                           _evaluationOverrides)) {
            return false;
        }
        miss->unfolded =
            RigExecControlStateDigest(miss->inputs, _evaluationOverrides);
    }
    miss->sampled = true;
    miss->key.compiledRigSignature = compiledRig;
    miss->key.controlDigest = RigExecFoldConstantDigest(
        miss->unfolded, RigExecCompiledRigConstantDigest(*program));
    if (haveProof && proof->second.digest == miss->key.controlDigest) {
        // The frameCache lane records actual store consultations only: a
        // lookup that never reaches the store (no proof, unsampleable,
        // stale scope) records nothing.
        const bool hit = storeLookup(miss->key, nullptr);
        if (publishing) {
            MutableProfiler()->RecordCacheLookup(hit, stamped);
        }
        if (hit && pose->valid) {
            *key = miss->key;
            return true;
        }
        // Proven but evicted: the sparse base may still serve (a re-keyed
        // entry under a moved digest), so the caller plans on the miss.
        *pose = RigExecRigPose();
    }
    return false;
}

bool
RigExecImagingBridge::_TryPublishCachedResult(UsdTimeCode time,
                                              PublishResult *result)
{
    if (!result) {
        return false;
    }
    RigExecRigPose cached;
    RigExecFrameCacheKey key{0, 0};
    _CacheMiss miss;
    if (_LookupCachedPose(time, /* publishing = */ true, &cached, &key,
                          &miss)) {
        _ServeCachedPose(time, cached, key, result);
        return true;
    }
    // No proof the pre-evaluation sample is fresh -- never evaluated here,
    // retired by an edit, or the inputs moved since. Chain-resolved inputs
    // sampled for a time the evaluator has not run come from the standing
    // resolved state, so an unproven sample must never serve directly; the
    // sparse plan compares it by VALUE against this time's own retained
    // base instead, which a stale sample cannot forge into a Hit.
    if (!miss.sampled || !miss.haveBase) {
        return false;
    }
    return _TrySparseServe(time, miss.inputs, miss.key, miss.unfolded, result);
}

bool
RigExecImagingBridge::_MemoizeLiveResult(UsdTimeCode time,
                                         const RigExecRigPose &pose)
{
    if (!_frameCache || !pose.valid) {
        return false;
    }
    // Scope check, as on the lookup path: the evaluation above may have
    // settled a new compiled rig.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    if (!_freshCompiledRigValid || _freshCompiledRig != compiledRig) {
        _freshDigests.clear();
        _freshCompiledRig = compiledRig;
        _freshCompiledRigValid = true;
    }
    // Sampled AFTER the evaluation, so the resolved state is fresh at exactly
    // this time: the digest below is the TRUE digest, and recording it as
    // this time's proof is what later lookups prove freshness against.
    // Sampled once, here, for the key AND the retained state:
    RigExecFrameCacheKey key;
    RigExecFrameInputs memoInputs;
    uint64_t memoUnfolded = 0;
    bool haveInputs = false;
    {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Imaging.MemoizeSampleDigest", "imaging");
        if (_CanSampleCacheInputs() && _SampleLive(time, &memoInputs) &&
            !memoInputs.HasChainResolvedInputs() &&
            RigExecControlStateDigestible(memoInputs, _evaluationOverrides)) {
            key.compiledRigSignature =
                RigExecFrameCacheCompiledRigSignature(*_evaluator);
            const uint64_t unfolded =
                RigExecControlStateDigest(memoInputs, _evaluationOverrides);
            key.controlDigest = RigExecFoldConstantDigest(
                unfolded, RigExecCompiledRigConstantDigest(
                              *_evaluator->GetBakedProgram()));
            memoUnfolded = unfolded;
            haveInputs = true;
        } else if (!_ComputePoseOnlyCacheKey(time, &key)) {
            return false;
        }
    }
    bool stored = false;
    {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Imaging.FrameCachePublish", "imaging");
        if (haveInputs) {
            // Retained-state publish: the source snapshot and
            // the dependency record ride with the pose, so a later edit
            // retains the source snapshot used to validate cache reuse.
            const RigExecBakedProgramImpl &impl =
                _evaluator->GetBakedProgram()->GetStepGraph();
            RigExecRetainedFrameState retainedState =
                RigExecCaptureRetainedState(
                    memoInputs, _evaluationOverrides, key.compiledRigSignature,
                    impl.clustering.clusters.size(),
                    RigExecCompiledRigConstantDigest(
                        *_evaluator->GetBakedProgram()));
            auto retained = std::make_shared<RigExecRetainedFrameState>(
                std::move(retainedState));
            const size_t retainedBytes = retained->RetainedBytes();
            RigExecEntryProvenance provenance =
                RigExecFullEvalProvenance(impl, time);
            provenance.unfoldedControlDigest = memoUnfolded;
            stored = _frameCache->Publish(key, time, pose, retainedBytes,
                                          retained, &provenance);
        } else {
            stored = _frameCache->Publish(key, time, pose);
        }
    }
    // A memoized entry is a cached frame: record the completion the strip
    // reads, under the generation this evaluation ran in. Default times
    // are not strip frames and record nothing.
    if (stored && _warmIndex && time.IsNumeric()) {
        _warmIndex->NoteCompleted(_rigPath, time.GetValue(), key,
                                  _warmIndex->CurrentGeneration(_rigPath));
    }
    // The compiled rig half frames were last stored under, for the notice path's
    // drift check -- kept even when the publish declines, so a later notice
    // evicts the half it would have named.
    _lastPublishedCompiledRig.store(key.compiledRigSignature,
                                    std::memory_order_relaxed);
    if (_freshDigests.size() >= _kFreshDigestCap) {
        _freshDigests.clear();
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        key.controlDigest, memoUnfolded, haveInputs ? &memoInputs : nullptr,
        &_evaluationOverrides);
    // No override admission here: memoization runs only when no overrides
    // stand (the drag bypass), so the list is always empty. Admission
    // lives on the warming path, which samples under standing overrides.
    return stored;
}

void
RigExecImagingBridge::_PublishGpuDeform(
    UsdTimeCode time, const SdfPath &primPath,
    const RigExecGpuDeformPublication &publication,
    RigExecPublishedPrim *published)
{
    if (!publication.layout || !publication.frame) {
        return;
    }
    const RigExecGpuDeformLayout &layout = *publication.layout;
    // The tables once per layout; a layout no publication names any more is
    // dropped when the cache would otherwise grow past a handful.
    auto &statics = _gpuDeformStatics[layout.serial];
    if (!statics) {
        if (_gpuDeformStatics.size() > 32) {
            _gpuDeformStatics.clear();
        }
        auto built = std::make_shared<RigExecPublishedPrim::GpuDeformStatic>();
        built->serial = layout.serial;
        built->ops.assign(layout.ops.begin(), layout.ops.end());
        built->staticF.assign(layout.staticF.begin(), layout.staticF.end());
        built->staticI.assign(layout.staticI.begin(), layout.staticI.end());
        built->opCount = int(layout.OpCount());
        built->pointCount = layout.pointCount;
        statics = built;
        _gpuDeformStatics[layout.serial] = statics;
    }
    published->gpuDeform =
        std::make_shared<const RigExecGpuDeformPublication>(publication);
    published->gpuDeformStatic = statics;
    // Velocities difference this frame against the one a frame earlier,
    // in units per second; anything else (a drag at one time, a jump, a
    // new layout) has no adjacent frame and publishes zero.
    _GpuDeformLast &last = _gpuDeformLast[primPath];
    if (last.serial != layout.serial ||
        last.frameVersion != publication.frameVersion ||
        last.numeric != time.IsNumeric() ||
        (time.IsNumeric() && last.time != time.GetValue())) {
        VtFloatArray frame(publication.frame->begin(),
                           publication.frame->end());
        const bool adjacent =
            time.IsNumeric() && last.numeric && last.serial == layout.serial &&
            std::abs(time.GetValue() - last.time - 1.0) < 1e-6;
        last.previous = adjacent ? last.frame : frame;
        last.frame = frame;
        last.serial = layout.serial;
        last.frameVersion = publication.frameVersion;
        last.numeric = time.IsNumeric();
        last.time = time.IsNumeric() ? time.GetValue() : 0.0;
    }
    published->gpuDeformFrame = last.frame;
    published->gpuDeformPreviousFrame = last.previous;
    published->gpuDeformVelocityScale =
        _stage ? float(_stage->GetTimeCodesPerSecond()) : 24.0f;
}

std::shared_ptr<RigExecImagingSnapshot>
RigExecImagingBridge::_BuildPoseSnapshot(UsdTimeCode time,
                                         const RigExecRigPose &pose,
                                         bool publishing)
{
    // Build the complete immutable generation from the exact native
    // results: points and normals as flat primvars, extent as min/max.
    // Only standard data crosses this boundary. Without publishing, only
    // what a pose query reads is built.
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    if (publishing) {
        _StampGeneration(time, snapshot.get());
    }
    // Matrix primvars a rig computes for a shader on the prim (a surface
    // projector's frame and dials), published as constant primvars. The
    // rig writes them as moved `primvars:<name>` properties and nothing
    // authors them; indexed before the geometry loop so a prim carrying
    // only a matrix primvar still gets published.
    static const std::string primvarsPrefix("primvars:");
    const std::vector<SdfPath> projectorTargets =
        _evaluator->GetSurfaceProjectorTargets();
    // Animated maps' float primvars: only those the evaluator names, so a
    // property mover's float result never mints a published prim.
    const std::vector<SdfPath> mapPrimvars =
        _evaluator->GetAnimatedMapPrimvars();
    for (const SdfPath &propertyPath : mapPrimvars) {
        const auto found = pose.movedProperties.find(propertyPath);
        if (found == pose.movedProperties.end() ||
            !found->second.IsHolding<float>()) {
            continue;
        }
        const std::string &name = propertyPath.GetName();
        auto &published = snapshot->prims[propertyPath.GetPrimPath()];
        published.shaderFloats[TfToken(name.substr(primvarsPrefix.size()))] =
            found->second.UncheckedGet<float>();
    }
    // Property chains' scalar primvars, from the compiled list only; a
    // double publishes as the float the chain computed.
    const std::vector<SdfPath> &scalarTargets =
        _evaluator->GetScalarPrimvarTargets();
    const std::vector<TfToken> &scalarNames =
        _evaluator->GetScalarPrimvarNames();
    for (size_t scalar = 0; scalar < scalarTargets.size(); ++scalar) {
        const SdfPath &propertyPath = scalarTargets[scalar];
        const auto found = pose.movedProperties.find(propertyPath);
        if (found == pose.movedProperties.end()) {
            continue;
        }
        float value = 0.0f;
        if (found->second.IsHolding<float>()) {
            value = found->second.UncheckedGet<float>();
        } else if (found->second.IsHolding<double>()) {
            value = float(found->second.UncheckedGet<double>());
        } else {
            continue;
        }
        auto &published = snapshot->prims[propertyPath.GetPrimPath()];
        published.shaderFloats[scalarNames[scalar]] = value;
    }
    for (const auto &[propertyPath, value] : pose.movedProperties) {
        if (!value.IsHolding<GfMatrix4d>()) {
            continue;
        }
        const std::string &name = propertyPath.GetName();
        if (name.compare(0, primvarsPrefix.size(), primvarsPrefix) != 0) {
            continue;
        }
        auto &published = snapshot->prims[propertyPath.GetPrimPath()];
        const TfToken primvar(name.substr(primvarsPrefix.size()));
        published.shaderMatrices[primvar] = value.UncheckedGet<GfMatrix4d>();
        if (std::binary_search(projectorTargets.begin(), projectorTargets.end(),
                               propertyPath)) {
            published.targetLocalShaderMatrices.insert(primvar);
        }
    }
    for (const auto &[propertyPath, value] : pose.movedProperties) {
        const SdfPath primPath = propertyPath.GetPrimPath();
        const TfToken property = propertyPath.GetNameToken();
        // Only geometry crosses into imaging. movedProperties also carries
        // the property-domain math mover results (a float dial, a vector
        // offset, a matrix), which have no Hydra representation -- and
        // indexing snapshot->prims for one of those would MINT an empty
        // published prim for its owner, which then enters the compiled rig
        // below and makes the scene index resolve a prim RigExec publishes
        // nothing for. Look the entry up only once there is something to
        // put in it.
        if (property == "points" &&
            value.IsHolding<RigExecGpuDeformPublication>()) {
            const auto &publication =
                value.UncheckedGet<RigExecGpuDeformPublication>();
            if (publishing) {
                _PublishGpuDeform(time, primPath, publication,
                                  &snapshot->prims[primPath]);
            } else if (publication.layout && publication.frame) {
                snapshot->prims[primPath].gpuDeform =
                    std::make_shared<const RigExecGpuDeformPublication>(
                        publication);
            }
            continue;
        }
        const bool isGeometry = value.IsHolding<VtVec3fArray>() &&
                                (property == "points" ||
                                 property == "normals" || property == "extent");
        if (!isGeometry) {
            // Not Hydra data -- but a scalar here is a rig output a
            // tool may want (a blend weight, a pose-interpolator
            // result), and recovering it otherwise costs a whole
            // second evaluation of the rig. Recorded beside the
            // generation that produced it; see
            // RigExecImagingSnapshot::movedFloats.
            if (value.IsHolding<float>()) {
                snapshot->movedFloats[propertyPath] =
                    value.UncheckedGet<float>();
            } else if (value.IsHolding<double>()) {
                snapshot->movedFloats[propertyPath] =
                    static_cast<float>(value.UncheckedGet<double>());
            } else if (value.IsHolding<VtDoubleArray>()) {
                snapshot->movedArrays[propertyPath] =
                    value.UncheckedGet<VtDoubleArray>();
            }
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[primPath];
        if (property == "points") {
            published.hasPoints = true;
            published.points = value.UncheckedGet<VtVec3fArray>();
        } else if (property == "normals") {
            published.hasNormals = true;
            published.normals = value.UncheckedGet<VtVec3fArray>();
        } else {
            const auto extent = value.UncheckedGet<VtVec3fArray>();
            if (extent.size() == 2) {
                published.hasExtent = true;
                published.extentMin = GfVec3d(extent[0]);
                published.extentMax = GfVec3d(extent[1]);
            }
        }
    }
    if (!publishing) {
        _FillProviderXforms(pose, snapshot.get());
        return snapshot;
    }
    // Joints and aggregate solvers publish guide payloads (drawn by the
    // results scene index like OpenExec's IrJointScope guides).
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.Guides",
                                  "imaging");
        const RigExecGuideCall call = _MakeGuideCall(
            _stage, *_evaluator, _interactiveOverrides, _weightOverlay);
        _FillProviderXforms(pose, snapshot.get());
        _guides->_FillGuides(call, pose, snapshot.get());
        _guides->_FillControlGuides(call, pose, snapshot.get());
        _guides->_FillVolumeGuides(call, pose, snapshot.get());
        _guides->_FillClothConstraintGuides(call, pose, snapshot.get());
        _guides->_FillWeightOverlay(call, pose, snapshot.get());
    }
    return snapshot;
}

void
RigExecImagingBridge::_PublishPoseSnapshot(UsdTimeCode time,
                                           const RigExecRigPose &pose,
                                           PublishResult *result)
{
    std::shared_ptr<RigExecImagingSnapshot> snapshot =
        _BuildPoseSnapshot(time, pose, /* publishing = */ true);
    snapshot->generation = ++_generation;

    // A structural recompile publishes a replacement compiled rig
    // before value notices.
    const size_t compiledRigSignature = _evaluator->GetCompiledRigSignature();
    if (compiledRigSignature != _publishedCompiledRigSignature) {
        auto compiledRig = std::make_shared<
            RigExecBindingResolvingSceneIndex::CompiledRigBinding>();
        compiledRig->id = compiledRigSignature;
        for (const auto &[primPath, published] : snapshot->prims) {
            compiledRig->publishedPrims.insert(primPath);
        }
        result->compiledRig = std::move(compiledRig);
        _publishedCompiledRigSignature = compiledRigSignature;
    }

    // Atomic snapshot swap; the caller sends the coalesced precise
    // dirtied notices from the notice owner.
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.StorePublish",
                                  "imaging");
        if (_publishesDirtied) {
            result->dirtied = _store->Publish(std::move(snapshot));
        } else {
            _store->Replace(std::move(snapshot));
        }
    }
    result->ok = true;
}

bool
RigExecImagingBridge::WarmFrameOnCallingThread(UsdTimeCode time)
{
    if (!RigExecBackgroundWarmingEnabled(_frameCacheMode,
                                         RigExecParallelEvaluationEnabled()) ||
        !_evaluationOverrides.empty()) {
        return false;
    }
    _NoteCacheEditSerial();
    RigExecRigPose pose;
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.WarmEvaluate",
                                  "imaging");
        pose = _evaluator->Evaluate(time);
    }
    return pose.valid && _MemoizeLiveResult(time, pose);
}

bool
RigExecImagingBridge::SnapshotAtTimeWithoutPublishing(
    UsdTimeCode time, bool evaluate, RigExecImagingSnapshotConstPtr *snapshot,
    std::string *diagnostic, bool *evaluated)
{
    if (evaluated) {
        *evaluated = false;
    }
    if (!snapshot || !_evaluationOverrides.empty()) {
        return false;
    }
    snapshot->reset();
    _NoteCacheEditSerial();
    const bool useCache = _frameCache && !_evaluator->cpuReference &&
                          _frameCacheMode != RigExecFrameCacheMode::Off;
    RigExecRigPose pose;
    RigExecFrameCacheKey key{0, 0};
    _CacheMiss miss;
    if (useCache &&
        _LookupCachedPose(time, /* publishing = */ false, &pose, &key, &miss) &&
        pose.valid) {
        // Stored for an earlier visit: re-stamped as _ServeCachedPose does.
        pose.time = time;
        if (_frameCacheVerify) {
            // RIGEXEC_FRAME_CACHE_VERIFY=1: proven against a live
            // evaluation, and a mismatch answers (and repairs the entry
            // with) the live pose.
            RigExecRigPose live = _evaluator->Evaluate(time);
            if (evaluated) {
                *evaluated = true;
            }
            const RigExecShadowVerdict verdict =
                RigExecVerifyHitWithLive(pose, [&live]() { return live; });
            if (!verdict.match && verdict.mismatches > 0) {
                TF_WARN("rigExec: frame-cache entry at %s mismatches live "
                        "evaluation (points at frame); answering live and "
                        "repairing the entry:\n%s",
                        time.IsDefault()
                            ? "default"
                            : std::to_string(time.GetValue()).c_str(),
                        verdict.report.c_str());
                pose = verdict.poseToServe;
                _MemoizeLiveResult(time, pose);
            }
        }
    } else {
        if (!evaluate) {
            return false;
        }
        pose = _evaluator->Evaluate(time);
        if (evaluated) {
            *evaluated = true;
        }
        if (!pose.valid) {
            if (diagnostic && !pose.diagnostics.empty()) {
                *diagnostic = pose.diagnostics.front();
            }
            return false;
        }
        if (useCache) {
            _MemoizeLiveResult(time, pose);
        }
    }
    *snapshot = _BuildPoseSnapshot(time, pose, /* publishing = */ false);
    return true;
}

bool
RigExecImagingBridge::EvaluateAndPublish(UsdTimeCode time)
{
    const PublishResult result = EvaluateAndPublishResult(time);
    // Deliver the notices even when evaluation failed. A failed generation
    // still CLEARS the previous one, and that clearing is only worth
    // anything if it reaches the observers -- returning early here was the
    // second half of the stale-pose bug: the store had been corrected and
    // nobody was told.
    if (_binding && result.compiledRig) {
        _binding->SetCompiledRigBinding(result.compiledRig);
    }
    if (_results && !result.dirtied.empty()) {
        _results->NotifyGenerationPublished(result.dirtied);
    }
    return result.ok;
}

RigExecImagingBridge::PublishResult
RigExecImagingBridge::EvaluateAndPublishResult(UsdTimeCode time)
{
    PublishResult result;
    RigExecProfileScope wholeScope(MutableProfiler(),
                                   "Imaging.EvaluateAndPublish", "imaging");
    // 0. Frame-cache fast path: a hit publishes the stored pose through the
    // same atomic snapshot without evaluating. Off, empty, unsampleable, and
    // unproven all fall through to the live path, which is the historical
    // behavior plus one memoization. Off disables both halves: the cache
    // is neither read nor written -- as does an active drag (below).
    const RigExecFrameCacheMode cacheMode = _frameCacheMode;
    // Interactive bypass: every drag tick carries unique overrides, so a
    // lookup would always miss after paying a full sample+digest, and
    // memoization would store single-use entries no scrub can reach --
    // pure overhead on the hottest path (the playhead is always live).
    // While overrides stand the cache is neither read nor written, exactly
    // as when off; proofs and entries are left untouched, so releasing the
    // drag resumes hitting the authored frames.
    const uint64_t serial = _NoteCacheEditSerial();
    // The frame an edit was just made at is not looked up: the evaluator
    // re-runs only the edit's cone from the generation it just published,
    // which no lookup beats -- one that misses first re-samples every input
    // the edit's serial retired. The live result still memoizes.
    const bool editedHere = _publishedTimeValid && _publishedTime == time &&
                            _publishedSerial != serial;
    _publishedTime = time;
    _publishedSerial = serial;
    _publishedTimeValid = true;
    const bool useCache =
        cacheMode != RigExecFrameCacheMode::Off && _evaluationOverrides.empty();
    if (useCache && !editedHere && _TryPublishCachedResult(time, &result)) {
        return result;
    }
    // 1. Evaluation always completes before publication.
    const RigExecRigPose pose = [&] {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.Evaluate",
                                  "imaging");
        return _evaluator->Evaluate(time);
    }();
    if (!pose.valid) {
        // A rig that cannot evaluate STOPS DRIVING THE SCENE.
        // Returning here without publishing would leave the last good
        // generation current, and every consumer would keep serving it (a
        // driven mesh stays where a now-failed constraint put it). The results
        // scene index handles a driven transform disappearing, and the store
        // diffs a prim that left the generation as structural -- but only
        // when there is a publication for them to run on.
        // Clearing is the rule the evaluator already applies to a
        // non-converged generation: an unevaluated result is recoverable, a
        // plausible wrong one is not. The scene falls back to what the stage
        // authors, which reads as "the rig is not running" instead of being
        // invisibly stale.
        result.dirtied = _store->Publish(nullptr);
        // The next good generation has to re-announce its compiled rig: the binding
        // index is still holding one whose published prim set no longer
        // exists, and an unchanged digest would suppress the replacement.
        _publishedCompiledRigSignature = 0;
        return result;
    }

    // 2-4. Snapshot publication, shared with the hit path so the two cannot
    // publish differently; then memoization into the frame cache (reads
    // on and no drag -- see step 0).
    _PublishPoseSnapshot(time, pose, &result);
    if (useCache) {
        _MemoizeLiveResult(time, pose);
    }
    return result;
}

bool
RigExecImagingBridge::PreflightMotionProfile(
    const std::vector<float> &shutterOffsets, MotionBlurSupport support,
    std::string *whyNot)
{
    if (shutterOffsets.empty()) {
        if (whyNot) {
            *whyNot = "render preflight requires at least one explicit "
                      "shutter offset";
        }
        return false;
    }
    for (size_t i = 0; i < shutterOffsets.size(); ++i) {
        if (!std::isfinite(shutterOffsets[i]) ||
            (i && shutterOffsets[i] <= shutterOffsets[i - 1])) {
            if (whyNot)
                *whyNot =
                    "shutter offsets must be finite and strictly increasing";
            return false;
        }
    }
    // motionBlurSupport = false permits one sample; the
    // capability bit never supplies offsets, and an absent bit leaves the
    // application's explicit render profile authoritative.
    if (support == MotionBlurSupport::False && shutterOffsets.size() > 1) {
        if (whyNot) {
            *whyNot = "renderer advertises motionBlurSupport = false; a "
                      "multi-sample motion profile fails preflight";
        }
        return false;
    }
    return true;
}

RigExecImagingBridge::PublishResult
RigExecImagingBridge::EvaluateAndPublishSamples(
    UsdTimeCode baseTime, const std::vector<float> &shutterOffsets,
    MotionBlurSupport support)
{
    PublishResult result;
    // Preflight fails deterministically before any evaluation:
    // capability mismatch or an invalid profile never
    // computes and never publishes.
    if (!PreflightMotionProfile(shutterOffsets, support) ||
        baseTime.IsDefault() || !std::isfinite(baseTime.GetValue())) {
        return result;
    }
    // The offset nearest zero supplies the primary (non-sampled) outputs.
    size_t baseIndex = 0;
    for (size_t i = 1; i < shutterOffsets.size(); ++i) {
        if (std::abs(shutterOffsets[i]) < std::abs(shutterOffsets[baseIndex])) {
            baseIndex = i;
        }
    }

    // Evaluation of every required offset completes before anything
    // publishes (spec 8.2, 10.5): all samples are captured, then one
    // complete immutable generation swaps in.
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    for (size_t i = 0; i < shutterOffsets.size(); ++i) {
        const UsdTimeCode sampleTime(baseTime.GetValue() +
                                     double(shutterOffsets[i]));
        const RigExecRigPose pose = _evaluator->Evaluate(sampleTime);
        if (!pose.valid) {
            return result; // an incomplete sample set never publishes
        }
        for (const auto &[path, matrix] : pose.providerXforms) {
            const auto base = pose.providerBaseXforms.find(path);
            if (base == pose.providerBaseXforms.end())
                return result;
            RigExecPublishedPrim &published = snapshot->prims[path];
            published.sampleOffsets = shutterOffsets;
            published.xformSamples.push_back(matrix);
            published.xformBaseSamples.push_back(base->second);
        }
        if (i == baseIndex) {
            // Guides are single-sampled at the base offset.
            const RigExecGuideCall call = _MakeGuideCall(
                _stage, *_evaluator, _interactiveOverrides, _weightOverlay);
            _FillProviderXforms(pose, snapshot.get());
            _guides->_FillGuides(call, pose, snapshot.get());
            _guides->_FillControlGuides(call, pose, snapshot.get());
            _guides->_FillVolumeGuides(call, pose, snapshot.get());
            _guides->_FillClothConstraintGuides(call, pose, snapshot.get());
            _guides->_FillWeightOverlay(call, pose, snapshot.get());
        }
        for (const auto &[propertyPath, value] : pose.movedProperties) {
            const SdfPath primPath = propertyPath.GetPrimPath();
            const TfToken property = propertyPath.GetNameToken();
            // Geometry only, and checked before snapshot->prims is indexed:
            // see EvaluateAndPublishResult for why minting an entry for a
            // property-domain result is wrong.
            const bool isGeometry =
                value.IsHolding<VtVec3fArray>() &&
                (property == "points" || property == "normals" ||
                 property == "extent");
            if (!isGeometry) {
                // Not Hydra data -- but a scalar here is a rig output a
                // tool may want (a blend weight, a pose-interpolator
                // result), and recovering it otherwise costs a whole
                // second evaluation of the rig. Recorded beside the
                // generation that produced it; see
                // RigExecImagingSnapshot::movedFloats.
                if (value.IsHolding<float>()) {
                    snapshot->movedFloats[propertyPath] =
                        value.UncheckedGet<float>();
                } else if (value.IsHolding<double>()) {
                    snapshot->movedFloats[propertyPath] =
                        static_cast<float>(value.UncheckedGet<double>());
                } else if (value.IsHolding<VtDoubleArray>()) {
                    snapshot->movedArrays[propertyPath] =
                        value.UncheckedGet<VtDoubleArray>();
                }
                continue;
            }
            RigExecPublishedPrim &published = snapshot->prims[primPath];
            published.sampleOffsets = shutterOffsets;
            const auto &array = value.UncheckedGet<VtVec3fArray>();
            if (property == "points")
                published.pointsSamples.push_back(array);
            else if (property == "normals")
                published.normalsSamples.push_back(array);
            else if (array.size() == 2) {
                published.extentMinSamples.push_back(GfVec3d(array[0]));
                published.extentMaxSamples.push_back(GfVec3d(array[1]));
            } else
                return result;
            if (i == baseIndex) {
                if (property == "points") {
                    published.hasPoints = true;
                    published.points = value.UncheckedGet<VtVec3fArray>();
                } else if (property == "normals") {
                    published.hasNormals = true;
                    published.normals = value.UncheckedGet<VtVec3fArray>();
                } else {
                    const auto extent = value.UncheckedGet<VtVec3fArray>();
                    if (extent.size() == 2) {
                        published.hasExtent = true;
                        published.extentMin = GfVec3d(extent[0]);
                        published.extentMax = GfVec3d(extent[1]);
                    }
                }
            }
        }
    }
    for (const auto &[primPath, published] : snapshot->prims) {
        for (size_t count :
             {published.pointsSamples.size(), published.normalsSamples.size(),
              published.xformSamples.size(), published.xformBaseSamples.size(),
              published.extentMinSamples.size(),
              published.extentMaxSamples.size()}) {
            if (count && count != shutterOffsets.size())
                return result;
        }
    }
    snapshot->generation = ++_generation;
    // The sampled path is identified by its BASE time: that is the frame a
    // consumer asks about, and the offsets are relative to it.
    _StampGeneration(baseTime, snapshot.get());

    const size_t compiledRigSignature = _evaluator->GetCompiledRigSignature();
    if (compiledRigSignature != _publishedCompiledRigSignature) {
        auto compiledRig = std::make_shared<
            RigExecBindingResolvingSceneIndex::CompiledRigBinding>();
        compiledRig->id = compiledRigSignature;
        for (const auto &[primPath, published] : snapshot->prims) {
            compiledRig->publishedPrims.insert(primPath);
        }
        result.compiledRig = std::move(compiledRig);
        _publishedCompiledRigSignature = compiledRigSignature;
    }
    result.dirtied = _store->Publish(std::move(snapshot));
    result.ok = true;
    return result;
}

} // namespace rigExec
