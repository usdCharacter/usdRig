#include "generateSkinWeights.h"
#include "rigBuilder.h"

#include "rigExec/capsuleSegment.h"
// _BakeFalloffLut: the falloff table compile bakes, so a generated field
// remaps distances exactly as the evaluated capsule does.
#include "rigExec/rigEvaluatorInternal.h"
#include "rigExec/weightPackets.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecScene/authoredRestSpace.h"
#include "rigExecScene/setupTransform.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/refPtr.h"
#include "pxr/base/tf/weakPtr.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

namespace rigExec {

namespace {

const TfToken _tSkinMover("RigExecSkinMover");
const TfToken _tRoot("RigExecRoot");
const TfToken _tMoves("rigExec:moves");
const TfToken _tInfluences("rigExec:influences");
const TfToken _tElementSize("rigExec:elementSize");
const TfToken _tJointIndices("rigExec:jointIndices");
const TfToken _tJointWeights("rigExec:jointWeights");
const TfToken _tPoints("points");
const TfToken _tRepresentation("rigExec:representation");
const TfToken _tRangePolicy("rigExec:rangePolicy");
const TfToken _tFalloffMin("inputs:falloffMin");
const TfToken _tFalloffMax("inputs:falloffMax");
const TfToken _tInvert("inputs:invert");
const TfToken _tStrength("inputs:strength");
const TfToken _tScaleX("inputs:scaleX");
const TfToken _tScaleY("inputs:scaleY");
const TfToken _tScaleZ("inputs:scaleZ");
const TfToken _tDense("dense");
const TfToken _tClamp("clamp");

using _Row = std::vector<std::pair<int, float>>;

float
_Float(const UsdPrim &prim, const TfToken &name, float fallback)
{
    float value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(name)) {
        a.Get(&value, UsdTimeCode::Default());
    }
    return value;
}

TfToken
_Token(const UsdPrim &prim, const TfToken &name, const TfToken &fallback)
{
    TfToken value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(name)) {
        a.Get(&value, UsdTimeCode::Default());
    }
    return value.IsEmpty() ? fallback : value;
}

SdfPathVector
_Targets(const UsdPrim &prim, const TfToken &name)
{
    SdfPathVector targets;
    if (const UsdRelationship rel = prim.GetRelationship(name)) {
        rel.GetTargets(&targets);
    }
    return targets;
}

// The rest frame of \p prim in asset space; a rest that is not rigid has no
// placement the evaluator would measure a volume from.
GfMatrix4d
_RigidRest(const UsdPrim &prim, const char *what)
{
    bool rigid = false;
    const GfMatrix4d rest = RigExecAuthoredGuideRestSpace(
        prim, UsdTimeCode::Default(), &rigid);
    if (!rigid) {
        throw std::runtime_error(
            prim.GetPath().GetString() + ": the " + what +
            "'s rest placement is not a rotation and translation");
    }
    return rest;
}

// One capsule's field over \p points (asset space), as the evaluator's
// packet builder measures it: the capsule placed at its rest frame, its
// segment end at the end provider's rest origin, scales and falloff read
// at Default, the range policy applied.
std::vector<float>
_CapsuleField(const UsdStagePtr &stage, const UsdPrim &capsule,
              const std::vector<GfVec3f> &points)
{
    const std::string who = capsule.GetPath().GetString();
    RigExecVolumeWeightInputs inputs;
    inputs.representation = _Token(capsule, _tRepresentation, _tDense);
    inputs.rangePolicy = _Token(capsule, _tRangePolicy, _tClamp);
    const GfMatrix4d rest = _RigidRest(capsule, "capsule");
    const RigExecPointFrame identity;
    for (size_t i = 0; i < identity.points.size(); ++i) {
        inputs.placement.points[i] = rest.Transform(identity.points[i]);
    }
    inputs.hasPlacement = true;
    inputs.params.falloffMin = _Float(capsule, _tFalloffMin, 0.0f);
    inputs.params.falloffMax = _Float(capsule, _tFalloffMax, 1.0f);
    inputs.params.invert = _Float(capsule, _tInvert, 0.0f);
    inputs.params.strength = _Float(capsule, _tStrength, 1.0f);
    inputs.params.curve = evaluatorDetail::_BakeFalloffLut(capsule);
    inputs.scales = GfVec3f(_Float(capsule, _tScaleX, 1.0f),
                            _Float(capsule, _tScaleY, 1.0f),
                            _Float(capsule, _tScaleZ, 1.0f));
    std::string endError;
    const SdfPath end = RigExecCapsuleSegmentEnd(capsule, &endError);
    if (!endError.empty()) {
        throw std::invalid_argument(endError);
    }
    if (!end.IsEmpty()) {
        const UsdPrim endPrim = stage->GetPrimAtPath(end);
        if (!endPrim) {
            throw std::invalid_argument(
                who + ": rigExec:segmentEnd target " + end.GetString() +
                " does not exist");
        }
        inputs.capsuleEnd = _RigidRest(endPrim, "segment end").ExtractTranslation();
        inputs.hasCapsuleEnd = true;
    }
    // The skin's own points are the sampled domain, whatever the capsule's
    // rigExec:weightTarget or rigExec:sampleSource name for an envelope.
    inputs.usePointViews = true;
    inputs.targetView = RigExecWeightPointView{points.data(), points.size()};
    RigExecWeightPacket packet =
        RigExecBuildVolumeWeightPacket(RigExecCapsuleWeightType(), inputs);
    if (!packet.valid || packet.values.size() != points.size()) {
        throw std::runtime_error(
            who + ": the capsule produces no field (check its falloff band, "
            "scales, representation and range policy)");
    }
    return std::move(packet.values);
}

}  // namespace

RigExecGeneratedSkinWeights
RigExecGenerateSkinWeights(const UsdStagePtr &stage, const SdfPath &skinMover,
                           const std::vector<int> *pointIndices)
{
    if (!stage) {
        throw std::invalid_argument("generate skin weights: no stage");
    }
    const UsdPrim mover = stage->GetPrimAtPath(skinMover);
    const std::string who = skinMover.GetString();
    if (!mover || mover.GetTypeName() != _tSkinMover) {
        throw std::invalid_argument(who + " is not a RigExecSkinMover");
    }

    // The target's points at Default, in the rig's asset space.
    const SdfPathVector moves = _Targets(mover, _tMoves);
    if (moves.size() != 1) {
        throw std::invalid_argument(
            who + ": rigExec:moves must name exactly one points property");
    }
    SdfPath pointsPath = moves[0];
    if (pointsPath.IsPrimPath()) {
        pointsPath = pointsPath.AppendProperty(_tPoints);
    }
    VtVec3fArray setupPoints;
    const UsdAttribute pointsAttr = stage->GetAttributeAtPath(pointsPath);
    if (!pointsAttr ||
        !pointsAttr.Get(&setupPoints, UsdTimeCode::Default())) {
        throw std::invalid_argument(
            who + ": " + pointsPath.GetString() +
            " has no point3f[] value at Default");
    }
    UsdPrim root = mover.GetParent();
    while (root && !root.IsPseudoRoot() && root.GetTypeName() != _tRoot) {
        root = root.GetParent();
    }
    if (!root || root.IsPseudoRoot()) {
        throw std::invalid_argument(who + " is not inside a RigExecRoot");
    }
    GfMatrix4d toAsset(1.0), fromAsset(1.0);
    SdfPathVector structure;
    std::string why;
    if (!RigExecSetupTransform(RigExecStageXformReader{get_pointer(stage)},
                               pointsPath.GetPrimPath(),
                               root.GetPath().GetParentPath(), &toAsset,
                               &fromAsset, &structure, &why)) {
        throw std::runtime_error(who + ": " + why);
    }
    const size_t count = setupPoints.size();
    std::vector<GfVec3f> points(count);
    for (size_t p = 0; p < count; ++p) {
        points[p] = GfVec3f(toAsset.Transform(GfVec3d(setupPoints[p])));
    }

    // The rows to replace.
    std::vector<bool> selected(count, pointIndices == nullptr);
    RigExecGeneratedSkinWeights result;
    if (pointIndices) {
        for (const int index : *pointIndices) {
            if (index < 0 || size_t(index) >= count) {
                throw std::invalid_argument(
                    who + ": point index " + std::to_string(index) +
                    " is outside the target's " + std::to_string(count) +
                    " points");
            }
            if (!selected[size_t(index)]) {
                selected[size_t(index)] = true;
                ++result.rowsWritten;
            }
        }
    } else {
        result.rowsWritten = count;
    }

    // Each influence's summed capsule field, point-major.
    const SdfPathVector influences = _Targets(mover, _tInfluences);
    if (influences.empty()) {
        throw std::invalid_argument(
            who + ": rigExec:influences must name at least one provider");
    }
    const size_t width = influences.size();
    std::vector<double> fields(count * width, 0.0);
    const TfToken &capsuleType = RigExecCapsuleWeightType();
    for (size_t j = 0; j < width; ++j) {
        const UsdPrim influence = stage->GetPrimAtPath(influences[j]);
        if (!influence) {
            throw std::invalid_argument(
                who + ": rigExec:influences target " +
                influences[j].GetString() + " does not exist");
        }
        bool owns = false;
        for (const UsdPrim &child : influence.GetChildren()) {
            if (child.GetTypeName() != capsuleType) continue;
            owns = true;
            if (count == 0) continue;  // no point to measure
            const std::vector<float> field = _CapsuleField(stage, child, points);
            for (size_t p = 0; p < count; ++p) {
                fields[p * width + j] += double(field[p]);
            }
        }
        if (!owns) {
            result.influencesWithoutCapsules.push_back(influences[j]);
        }
    }

    // The rows a call keeps come from the authored layout.
    std::vector<_Row> rows(count);
    if (pointIndices) {
        VtIntArray indices;
        VtFloatArray weights;
        int elementSize = 1;
        if (const UsdAttribute a = mover.GetAttribute(_tJointIndices)) {
            a.Get(&indices, UsdTimeCode::Default());
        }
        if (const UsdAttribute a = mover.GetAttribute(_tJointWeights)) {
            a.Get(&weights, UsdTimeCode::Default());
        }
        if (const UsdAttribute a = mover.GetAttribute(_tElementSize)) {
            a.Get(&elementSize, UsdTimeCode::Default());
        }
        if (!indices.empty() || !weights.empty()) {
            if (elementSize < 1 || indices.size() != weights.size() ||
                indices.size() != count * size_t(elementSize)) {
                throw std::invalid_argument(
                    who + ": the authored rows do not describe the target's " +
                    std::to_string(count) +
                    " points; generate every row first");
            }
            for (size_t p = 0; p < count; ++p) {
                if (selected[p]) continue;
                for (size_t k = 0; k < size_t(elementSize); ++k) {
                    const size_t slot = p * size_t(elementSize) + k;
                    if (weights[slot] != 0.0f) {
                        rows[p].emplace_back(indices[slot], weights[slot]);
                    }
                }
            }
        }
    }

    // Normalise the selected rows over the influences.
    for (size_t p = 0; p < count; ++p) {
        if (!selected[p]) continue;
        double sum = 0.0;
        for (size_t j = 0; j < width; ++j) {
            double &w = fields[p * width + j];
            w = std::min(w, 1.0);
            sum += w;
        }
        if (!(sum > 0.0)) continue;  // a zero row holds the rest point
        for (size_t j = 0; j < width; ++j) {
            const float w = float(fields[p * width + j] / sum);
            if (w > 0.0f) {
                rows[p].emplace_back(int(j), w);
            }
        }
    }

    size_t elementSize = 1;
    for (const _Row &row : rows) {
        elementSize = std::max(elementSize, row.size());
    }
    std::vector<int> indices(count * elementSize, 0);
    std::vector<float> weights(count * elementSize, 0.0f);
    for (size_t p = 0; p < count; ++p) {
        for (size_t k = 0; k < rows[p].size(); ++k) {
            indices[p * elementSize + k] = rows[p][k].first;
            weights[p * elementSize + k] = rows[p][k].second;
        }
    }
    result.elementSize = int(elementSize);
    RigExecSkinMoverHandle(TfCreateRefPtrFromProtectedWeakPtr(stage), skinMover)
        .SetJointInfluences(indices, weights, result.elementSize);
    return result;
}

}  // namespace rigExec
