// Render-motion publication for the imaging bridge: the facade's
// multi-sample workflow as a stateless TU. All state belongs to the
// RigExecImagingBridge facade; this path never consults the frame cache.
#include "bridge.h"

#include "bridgeGuides.h"
#include "bridgeSnapshot.h"

#include "rigExec/api/poseFrameReader.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/staticTokens.h"

#include <cmath>
#include <utility>

namespace rigExec {

namespace {

TF_DEFINE_PRIVATE_TOKENS(_motionTokens, (points)(normals)(extent)(
                                            (primvarsNormals,
                                             "primvars:normals")));

} // namespace

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
        const RigExecPoseFrameReader reader(pose);
        if (!reader.IsValid()) {
            return result; // an incomplete sample set never publishes
        }
        const size_t providerCount =
            reader.GetCount(RigExecFrameDomain::ProviderTransform);
        for (size_t provider = 0; provider < providerCount; ++provider) {
            const SdfPath &path =
                reader.GetPath(RigExecFrameDomain::ProviderTransform, provider);
            GfMatrix4d base;
            if (!reader.FindMatrix(RigExecFrameDomain::ProviderBaseTransform,
                                   path, &base))
                return result;
            RigExecPublishedPrim &published = snapshot->prims[path];
            published.sampleOffsets = shutterOffsets;
            published.xformSamples.push_back(reader.GetMatrix(
                RigExecFrameDomain::ProviderTransform, provider));
            published.xformBaseSamples.push_back(base);
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
            _snapshot->_FillProviderXforms(xformCall, reader, snapshot.get());
            _guides->_FillGuides(call, reader, snapshot.get());
            _guides->_FillControlGuides(call, reader, snapshot.get());
            _guides->_FillVolumeGuides(call, reader, snapshot.get());
            _guides->_FillClothConstraintGuides(call, reader, snapshot.get());
            _guides->_FillWeightOverlay(call, reader, snapshot.get());
        }
        const size_t movedCount =
            reader.GetCount(RigExecFrameDomain::MovedProperty);
        for (size_t moved = 0; moved < movedCount; ++moved) {
            const SdfPath &propertyPath =
                reader.GetPath(RigExecFrameDomain::MovedProperty, moved);
            const RigExecMovedValueKind kind = reader.GetMovedKind(moved);
            const SdfPath primPath = propertyPath.GetPrimPath();
            const TfToken property = propertyPath.GetNameToken();
            // Geometry only, and checked before snapshot->prims is indexed:
            // see EvaluateAndPublishResult for why minting an entry for a
            // property-domain result is wrong.
            const bool isNormals = property == _motionTokens->normals ||
                                   property == _motionTokens->primvarsNormals;
            const bool isGeometry =
                kind == RigExecMovedValueKind::Vec3fArray &&
                (property == _motionTokens->points || isNormals ||
                 property == _motionTokens->extent);
            if (!isGeometry) {
                // Not Hydra data -- but a scalar here is a rig output a
                // tool may want (a blend weight, a pose-interpolator
                // result), and recovering it otherwise costs a whole
                // second evaluation of the rig. Recorded beside the
                // generation that produced it; see
                // RigExecImagingSnapshot::movedFloats.
                if (kind == RigExecMovedValueKind::Float) {
                    reader.GetMovedValue(moved,
                                         &snapshot->movedFloats[propertyPath]);
                } else if (kind == RigExecMovedValueKind::Double) {
                    double value = 0.0;
                    reader.GetMovedValue(moved, &value);
                    snapshot->movedFloats[propertyPath] =
                        static_cast<float>(value);
                } else if (kind == RigExecMovedValueKind::DoubleArray) {
                    reader.GetMovedValue(moved,
                                         &snapshot->movedArrays[propertyPath]);
                }
                continue;
            }
            RigExecPublishedPrim &published = snapshot->prims[primPath];
            published.sampleOffsets = shutterOffsets;
            VtVec3fArray array;
            reader.GetMovedValue(moved, &array);
            if (property == _motionTokens->points)
                published.pointsSamples.push_back(array);
            else if (isNormals)
                published.normalsSamples.push_back(array);
            else if (array.size() == 2) {
                published.extentMinSamples.push_back(GfVec3d(array[0]));
                published.extentMaxSamples.push_back(GfVec3d(array[1]));
            } else
                return result;
            if (i == baseIndex) {
                if (property == _motionTokens->points) {
                    published.hasPoints = true;
                    published.points = array;
                } else if (isNormals) {
                    published.hasNormals = true;
                    published.normals = array;
                } else {
                    if (array.size() == 2) {
                        published.hasExtent = true;
                        published.extentMin = GfVec3d(array[0]);
                        published.extentMax = GfVec3d(array[1]);
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
