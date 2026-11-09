// RigExec Hydra publication bridge implementation.
#include "bridge.h"

#include "bridgeGuideGeometry.h"
#include "bridgeGuides.h"
#include "bridgeSampling.h"
#include "bridgeSnapshot.h"

#include "rigExecMath/pointFrame.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/parallel.h"
#include "rigExecScene/authoredRestSpace.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"

#include <algorithm>
#include <cmath>
#include <cstring>
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

// The state the snapshot builder borrows on every call: the bridge owns the
// stage and the time, and the evaluator's stage-edit serial and compiled
// publication metadata (projector targets, animated-map and scalar primvar
// targets/names) are fetched here so the builder never sees the evaluator.
RigExecSnapshotCall
_MakeSnapshotCall(const UsdStageRefPtr &stage,
                  const RigExecRigEvaluator &evaluator, UsdTimeCode time)
{
    RigExecSnapshotCall call;
    call.stage = stage;
    call.time = time;
    call.stageEditSerial = evaluator.GetStageEditSerial();
    call.projectorTargets = evaluator.GetSurfaceProjectorTargets();
    call.animatedMapPrimvars = evaluator.GetAnimatedMapPrimvars();
    call.scalarPrimvarTargets = &evaluator.GetScalarPrimvarTargets();
    call.scalarPrimvarNames = &evaluator.GetScalarPrimvarNames();
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
      _snapshot(std::make_unique<RigExecBridgeSnapshotBuilder>()),
      _sampling(std::make_unique<RigExecBridgeSampling>(*_evaluator)),
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
RigExecImagingBridge::NoteChainEdits(const UsdNotice::ObjectsChanged &notice)
{
    _sampling->NoteChainEdits(notice);
}

const RigExecChainSampleBindings *
RigExecImagingBridge::AcquireChainBindings(uint64_t *generation) const
{
    return _sampling->AcquireChainBindings(generation);
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

void
RigExecImagingBridge::_PublishPoseSnapshot(UsdTimeCode time,
                                           const RigExecRigPose &pose,
                                           PublishResult *result)
{
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    _StampGeneration(time, snapshot.get());
    const RigExecSnapshotCall snapCall =
        _MakeSnapshotCall(_stage, *_evaluator, time);
    _snapshot->_Build(snapCall, pose, RigExecSnapshotBuildMode::Displayed,
                      snapshot.get());
    // Joints and aggregate solvers publish guide payloads (drawn by the
    // results scene index like OpenExec's IrJointScope guides).
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.Guides",
                                  "imaging");
        const RigExecGuideCall call = _MakeGuideCall(
            _stage, *_evaluator, _interactiveOverrides, _weightOverlay);
        _guides->_FillGuides(call, pose, snapshot.get());
        _guides->_FillControlGuides(call, pose, snapshot.get());
        _guides->_FillVolumeGuides(call, pose, snapshot.get());
        _guides->_FillClothConstraintGuides(call, pose, snapshot.get());
        _guides->_FillWeightOverlay(call, pose, snapshot.get());
    }
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
    auto querySnapshot = std::make_shared<RigExecImagingSnapshot>();
    const RigExecSnapshotCall snapCall =
        _MakeSnapshotCall(_stage, *_evaluator, time);
    _snapshot->_Build(snapCall, pose, RigExecSnapshotBuildMode::Query,
                      querySnapshot.get());
    *snapshot = std::move(querySnapshot);
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
            RigExecSnapshotCall xformCall;
            xformCall.stage = _stage;
            xformCall.time = sampleTime;
            xformCall.stageEditSerial = _evaluator->GetStageEditSerial();
            _snapshot->_FillProviderXforms(xformCall, pose, snapshot.get());
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
