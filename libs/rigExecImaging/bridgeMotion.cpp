// Render-motion publication for the imaging bridge: the facade's
// multi-sample workflow as a stateless TU. All state belongs to the
// RigExecImagingBridge facade; this path never consults the frame cache.
#include "bridge.h"

#include "bridgeGuides.h"
#include "bridgeSnapshot.h"

#include "pxr/base/gf/vec3f.h"

#include <cmath>
#include <utility>

namespace rigExec {

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
            RigExecGuideCall call(_stage, _evaluator->GetStageEditSerial(),
                                  _evaluator->GetCompiledRigSignature(),
                                  _evaluator->GetMoverOrder(),
                                  _interactiveOverrides, _weightOverlay);
            RigExecProviderXformCall xformCall;
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

    // A structural recompile announces the replacement compiled rig before
    // the store swap, shared with the single-frame path.
    _AnnounceCompiledRig(*snapshot, &result);
    result.dirtied = _store->Publish(std::move(snapshot));
    result.ok = true;
    return result;
}

} // namespace rigExec
