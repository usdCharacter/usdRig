#include "generateSkinWeights.h"
#include "rigBuilder.h"

// _BakeFalloffLut: compile's own falloff table, so fields match evaluation.
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
const TfToken _tFalloffSamples("inputs:falloffSamples");
const TfToken _tInvert("inputs:invert");
const TfToken _tStrength("inputs:strength");
const TfToken _tScaleX("inputs:scaleX");
const TfToken _tScaleY("inputs:scaleY");
const TfToken _tScaleZ("inputs:scaleZ");
const TfToken _tScaleXPos("inputs:scaleXPos");
const TfToken _tScaleYPos("inputs:scaleYPos");
const TfToken _tScaleZPos("inputs:scaleZPos");
const TfToken _tScaleXNeg("inputs:scaleXNeg");
const TfToken _tScaleYNeg("inputs:scaleYNeg");
const TfToken _tScaleZNeg("inputs:scaleZNeg");
const TfToken _tPlaneAxis("rigExec:planeAxis");
const TfToken _tPlaneBounds("rigExec:planeBounds");
const TfToken _tExtentU("inputs:extentU");
const TfToken _tExtentV("inputs:extentV");
const TfToken _tCurve("rigExec:curve");
const TfToken _tSphereType("RigExecSphereWeight");
const TfToken _tPlaneType("RigExecPlaneWeight");
const TfToken _tCurveType("RigExecCurveWeight");
const TfToken _tDense("dense");
const TfToken _tClamp("clamp");
const TfToken _tY("y");
const TfToken _tUnbounded("unbounded");
const TfToken _tBounded("bounded");

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

// The rigExec:curve points at Default, in asset space like the skin's points.
std::vector<GfVec3f>
_CurvePoints(const UsdStagePtr &stage, const UsdPrim &volume,
             const SdfPath &assetPath)
{
    const std::string who = volume.GetPath().GetString();
    const SdfPathVector curves = _Targets(volume, _tCurve);
    if (curves.size() != 1) {
        throw std::invalid_argument(
            who + ": rigExec:curve must name exactly one points source");
    }
    const SdfPath pointsPath = curves[0].IsPropertyPath()
                                   ? curves[0]
                                   : curves[0].AppendProperty(_tPoints);
    VtVec3fArray raw;
    const UsdAttribute attr = stage->GetAttributeAtPath(pointsPath);
    if (!attr || attr.GetTypeName() != SdfValueTypeNames->Point3fArray ||
        !attr.Get(&raw, UsdTimeCode::Default())) {
        throw std::invalid_argument(
            who + ": rigExec:curve target " + curves[0].GetString() +
            " has no point3f[] value at Default");
    }
    GfMatrix4d toAsset(1.0), fromAsset(1.0);
    SdfPathVector structure;
    std::string why;
    if (!RigExecSetupTransform(RigExecStageXformReader{get_pointer(stage)},
                               pointsPath.GetPrimPath(), assetPath, &toAsset,
                               &fromAsset, &structure, &why)) {
        throw std::runtime_error(who + ": rigExec:curve: " + why);
    }
    std::vector<GfVec3f> points(raw.size());
    for (size_t p = 0; p < raw.size(); ++p) {
        points[p] = GfVec3f(toAsset.Transform(GfVec3d(raw[p])));
    }
    return points;
}

// One volume weight's field over \p points (asset space), measured by the
// evaluator's packet builder at the volume's rest placement and Default inputs.
std::vector<float>
_VolumeField(const UsdStagePtr &stage, const UsdPrim &volume,
             const SdfPath &assetPath, const std::vector<GfVec3f> &points)
{
    const std::string who = volume.GetPath().GetString();
    const TfToken type = volume.GetTypeName();
    RigExecVolumeWeightInputs inputs;
    inputs.representation = _Token(volume, _tRepresentation, _tDense);
    inputs.rangePolicy = _Token(volume, _tRangePolicy, _tClamp);
    const GfMatrix4d rest = _RigidRest(volume, "volume weight");
    const RigExecPointFrame identity;
    for (size_t i = 0; i < identity.points.size(); ++i) {
        inputs.placement.points[i] = rest.Transform(identity.points[i]);
    }
    inputs.hasPlacement = true;
    inputs.params.falloffMin = _Float(volume, _tFalloffMin, 0.0f);
    inputs.params.falloffMax = _Float(volume, _tFalloffMax, 1.0f);
    inputs.params.invert = _Float(volume, _tInvert, 0.0f);
    inputs.params.strength = _Float(volume, _tStrength, 1.0f);
    inputs.params.curve = evaluatorDetail::_BakeFalloffLut(volume);
    // Authored samples replace the compiled table, as in every evaluator.
    if (const UsdAttribute a = volume.GetAttribute(_tFalloffSamples)) {
        VtFloatArray samples;
        if (a.Get(&samples, UsdTimeCode::Default()) && !samples.empty()) {
            inputs.params.curve.assign(samples.cbegin(), samples.cend());
        }
    }
    inputs.scales = GfVec3f(_Float(volume, _tScaleX, 1.0f),
                            _Float(volume, _tScaleY, 1.0f),
                            _Float(volume, _tScaleZ, 1.0f));
    std::vector<GfVec3f> curve;
    if (type == _tSphereType) {
        inputs.positiveScales = GfVec3f(_Float(volume, _tScaleXPos, 1.0f),
                                        _Float(volume, _tScaleYPos, 1.0f),
                                        _Float(volume, _tScaleZPos, 1.0f));
        inputs.negativeScales = GfVec3f(_Float(volume, _tScaleXNeg, 1.0f),
                                        _Float(volume, _tScaleYNeg, 1.0f),
                                        _Float(volume, _tScaleZNeg, 1.0f));
    } else if (type == _tPlaneType) {
        inputs.planeAxis = _Token(volume, _tPlaneAxis, _tY);
        inputs.planeBounds = _Token(volume, _tPlaneBounds, _tUnbounded);
        if (inputs.planeBounds == _tBounded) {
            inputs.extentU = _Float(volume, _tExtentU, 1.0f);
            inputs.extentV = _Float(volume, _tExtentV, 1.0f);
        }
    } else if (type == _tCurveType) {
        curve = _CurvePoints(stage, volume, assetPath);
    }
    // The skin's own points are the sampled domain, whatever the volume's
    // rigExec:weightTarget or rigExec:sampleSource name for an envelope.
    inputs.usePointViews = true;
    inputs.targetView = RigExecWeightPointView{points.data(), points.size()};
    inputs.curveView = RigExecWeightPointView{curve.data(), curve.size()};
    RigExecWeightPacket packet = RigExecBuildVolumeWeightPacket(type, inputs);
    if (!packet.valid || packet.values.size() != points.size()) {
        throw std::runtime_error(
            who + ": the volume weight produces no field (check its falloff "
            "band, scales, representation, range policy and shape "
            "settings)");
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

    const SdfPathVector influences = _Targets(mover, _tInfluences);
    if (influences.empty()) {
        throw std::invalid_argument(
            who + ": rigExec:influences must name at least one provider");
    }
    const size_t width = influences.size();
    std::vector<double> fields(count * width, 0.0);
    const SdfPath assetPath = root.GetPath().GetParentPath();
    for (size_t j = 0; j < width; ++j) {
        const UsdPrim influence = stage->GetPrimAtPath(influences[j]);
        if (!influence) {
            throw std::invalid_argument(
                who + ": rigExec:influences target " +
                influences[j].GetString() + " does not exist");
        }
        bool owns = false;
        for (const UsdPrim &child : influence.GetChildren()) {
            if (!evaluatorDetail::_IsVolumeWeightType(child.GetTypeName())) {
                continue;
            }
            owns = true;
            if (count == 0) continue;  // no point to measure
            const std::vector<float> field =
                _VolumeField(stage, child, assetPath, points);
            for (size_t p = 0; p < count; ++p) {
                fields[p * width + j] += double(field[p]);
            }
        }
        if (!owns) {
            result.influencesWithoutVolumes.push_back(influences[j]);
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
