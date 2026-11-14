// RigExec guide geometry queries: the UsdGeomBoundable extent callback,
// its purpose resolution, the authored default-pose fallback, and the
// asset-space to local conversion. Bounds accumulation lives in
// guideBounds.cpp.
#include "guideBoundsInternal.h"

#include "registry.h"
#include "rigExecMath/avarScale.h"
#include "rigExecSchema/schemaTokens.h"
#include "rigExecScene/authoredRestSpace.h"
#include "sceneIndices.h"

#include "pxr/base/gf/bbox3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/boundable.h"
#include "pxr/usd/usdGeom/boundableComputeExtent.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <cmath>
#include <cstddef>
#include <limits>

namespace rigExec {
namespace {

// A prim's resolved render purpose -- the same value the bridge publishes
// and the same one UsdGeomBBoxCache files its extent under.
PXR_NS::TfToken
_ResolvedPurpose(const PXR_NS::UsdPrim &prim)
{
    if (!prim) {
        return PXR_NS::UsdGeomTokens->default_;
    }
    if (const PXR_NS::UsdGeomImageable imageable =
            PXR_NS::UsdGeomImageable(prim)) {
        const PXR_NS::TfToken purpose = imageable.ComputePurpose();
        if (!purpose.IsEmpty()) {
            return purpose;
        }
    }
    return PXR_NS::UsdGeomTokens->default_;
}

// Authored-attribute reads for the default-pose fallback: the values a
// guide would draw from, at the queried time, with the schema's defaults
// where nothing is authored.
struct _RestAttrs {
    const PXR_NS::UsdPrim &prim;
    PXR_NS::UsdTimeCode time;

    double number(const PXR_NS::TfToken &name, double fallback) const
    {
        double value = fallback;
        if (const PXR_NS::UsdAttribute a = prim.GetAttribute(name)) {
            a.Get(&value, time);
        }
        return value;
    }
    double floatNumber(const PXR_NS::TfToken &name, double fallback) const
    {
        float value = static_cast<float>(fallback);
        if (const PXR_NS::UsdAttribute a = prim.GetAttribute(name)) {
            a.Get(&value, time);
        }
        return double(value);
    }
    PXR_NS::TfToken token(const PXR_NS::TfToken &name,
                          const PXR_NS::TfToken &fallback) const
    {
        PXR_NS::TfToken value(fallback);
        if (const PXR_NS::UsdAttribute a = prim.GetAttribute(name)) {
            a.Get(&value, time);
        }
        return value;
    }
};

// One default-pose element's box, checked end to end: corners finite,
// and finite again as the float extent the host reads. A rejected
// element leaves the caller's range exactly as it found it.
bool
_ElementBoxUsable(const PXR_NS::GfVec3d &min, const PXR_NS::GfVec3d &max)
{
    const double limit = std::numeric_limits<float>::max();
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(min[axis]) || !std::isfinite(max[axis]) ||
            std::abs(min[axis]) > limit || std::abs(max[axis]) > limit) {
            return false;
        }
    }
    return true;
}

// The avars scale-channel attribute for an axis, in axis order.
const PXR_NS::TfToken &
_AvarScaleName(int axis)
{
    return axis == 0 ? SchemaAttr::avarsSx
                     : (axis == 1 ? SchemaAttr::avarsSy : SchemaAttr::avarsSz);
}

// The bounds a control's guide would draw at the default pose, from
// authored attributes alone, placed by its rigid rest:space.
bool
_AccumulateControlDefaultPose(const PXR_NS::UsdPrim &prim,
                              const _RestAttrs &attrs,
                              const PXR_NS::GfMatrix4d &rest,
                              PXR_NS::GfRange3d &range)
{
    if (attrs.floatNumber(SchemaAttr::guideDisplayOpacity, 0.5) <= 0.0)
        return false;
    // Same predicate the scene index draws through: an unrecognized
    // shape/drawMode pair synthesizes nothing and must bound nothing.
    const PXR_NS::TfToken shape =
        attrs.token(SchemaAttr::guideShape, SchemaValue::shapeCircle);
    const PXR_NS::TfToken drawMode =
        attrs.token(SchemaAttr::guideDrawMode, SchemaValue::drawModeWire);
    if (!rigExec::RigExecControlGuideIsDrawn(shape, drawMode)) {
        return false;
    }
    const PXR_NS::GfVec3d authoredScale(
        attrs.number(SchemaAttr::guideScaleX, 1.0),
        attrs.number(SchemaAttr::guideScaleY, 1.0),
        attrs.number(SchemaAttr::guideScaleZ, 1.0));
    PXR_NS::GfVec3d scale(1.0);
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(authoredScale[i]) || authoredScale[i] <= 0.0) {
            return false; // draws nothing, so it bounds nothing
        }
        // No evaluated frame exists here: resolve the authored channel at
        // the queried time with the runtime's floor and non-finite policy,
        // sized by magnitude so a supported reflection stays visible.
        const double avarMagnitude =
            std::abs(rigExec::RigExecNormalizeAvarScale(
                attrs.number(_AvarScaleName(i), 1.0)));
        scale[i] = authoredScale[i] * avarMagnitude;
        if (!std::isfinite(scale[i]) || scale[i] <= 0.0) {
            return false;
        }
    }
    double halfWidth = 0.0;
    if (drawMode == SchemaValue::drawModeWire) {
        const double width = attrs.number(SchemaAttr::guideWireWidth, 0.05);
        if (std::isfinite(width) && width > 0.0) {
            halfWidth = width * 0.5;
        }
    }
    if (shape == SchemaValue::shapeCustom) {
        PXR_NS::VtVec3fArray points;
        PXR_NS::VtIntArray counts;
        prim.GetAttribute(SchemaAttr::guidePoints).Get(&points, attrs.time);
        prim.GetAttribute(SchemaAttr::guideCurveVertexCounts)
            .Get(&counts, attrs.time);
        PXR_NS::GfRange3d local = _CustomControlRange(points, counts);
        if (local.IsEmpty())
            return false;
        const PXR_NS::GfVec3d pad(halfWidth);
        local = PXR_NS::GfRange3d(local.GetMin() - pad, local.GetMax() + pad);
        PXR_NS::GfVec3d signedScale;
        for (int axis = 0; axis < 3; ++axis) {
            signedScale[axis] = rigExec::RigExecNormalizeAvarScale(
                attrs.number(_AvarScaleName(axis), 1.0));
        }
        PXR_NS::GfVec3d offset(0.0);
        prim.GetAttribute(SchemaAttr::guideOffset).Get(&offset, attrs.time);
        PXR_NS::GfMatrix4d scaleMatrix(1.0);
        scaleMatrix.SetScale(signedScale);
        PXR_NS::GfMatrix4d shapeScale(1.0);
        shapeScale.SetScale(authoredScale);
        const PXR_NS::GfMatrix4d placement =
            shapeScale * PXR_NS::GfMatrix4d(1.0).SetTranslate(offset) *
            scaleMatrix * rest;
        const PXR_NS::GfRange3d drawn =
            PXR_NS::GfBBox3d(local, placement).ComputeAlignedRange();
        if (!_ElementBoxUsable(drawn.GetMin(), drawn.GetMax())) {
            return false;
        }
        range.UnionWith(drawn);
        return true;
    }
    // The unit shape, for the same reason the snapshot path uses it:
    // every guide shape is documented as unit-sized with half-extent 1.
    const PXR_NS::GfVec3d half(scale[0] * (1.0 + halfWidth),
                               scale[1] * (1.0 + halfWidth),
                               scale[2] * (1.0 + halfWidth));
    const PXR_NS::GfRange3d drawn =
        PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-half, half), rest)
            .ComputeAlignedRange();
    if (!_ElementBoxUsable(drawn.GetMin(), drawn.GetMax())) {
        return false;
    }
    range.UnionWith(drawn);
    return true;
}

// The bounds a joint's guide sphere and its outgoing links would draw at
// the default pose, placed by their rigid rest:space attributes.
bool
_AccumulateJointDefaultPose(const PXR_NS::UsdPrim &prim,
                            const _RestAttrs &attrs,
                            const PXR_NS::GfMatrix4d &rest,
                            PXR_NS::GfRange3d &range)
{
    const double radius = attrs.number(SchemaAttr::guideRadius, 1.0);
    if (!std::isfinite(radius) || radius <= 0.0) {
        return false;
    }
    const PXR_NS::GfVec3d origin = rest.ExtractTranslation();
    if (!_AllFinite(origin)) {
        return false;
    }
    const PXR_NS::GfVec3d extent(radius, radius, radius);
    // The element is applied or rejected as a whole: legal but huge
    // radius and translation can overflow one corner while the other
    // collapses toward the origin, and a half-applied box would frame a
    // point nobody authored.
    const PXR_NS::GfVec3d lo = origin - extent;
    const PXR_NS::GfVec3d hi = origin + extent;
    if (_ElementBoxUsable(lo, hi)) {
        range.UnionWith(lo);
        range.UnionWith(hi);
    }

    // A joint's outgoing links terminate at every descendant whose
    // nearest RigExecJoint ancestor is this prim.
    for (const PXR_NS::UsdPrim &candidate : PXR_NS::UsdPrimRange(prim)) {
        if (candidate == prim ||
            candidate.GetTypeName() != SchemaType::RigExecJoint) {
            continue;
        }
        PXR_NS::UsdPrim ancestor = candidate.GetParent();
        while (ancestor && ancestor != prim &&
               ancestor.GetTypeName() != SchemaType::RigExecJoint) {
            ancestor = ancestor.GetParent();
        }
        if (ancestor != prim) {
            continue;
        }
        bool childRigid = false;
        const PXR_NS::GfMatrix4d childRest =
            rigExec::RigExecAuthoredGuideRestSpace(candidate, attrs.time,
                                                   &childRigid);
        if (!childRigid) {
            continue;
        }
        const PXR_NS::GfVec3d tip = childRest.ExtractTranslation();
        if (!_AllFinite(tip) || (tip - origin).GetLength() <= 1e-12) {
            continue;
        }
        const PXR_NS::GfVec3d linkLo = tip - extent;
        const PXR_NS::GfVec3d linkHi = tip + extent;
        if (_ElementBoxUsable(linkLo, linkHi)) {
            range.UnionWith(linkLo);
            range.UnionWith(linkHi);
        }
    }
    return true;
}

// The facts the three weight guide shapes share: whether the mode draws
// a wire, its pad, and the two falloff radii.
struct _WeightGuideFacts {
    bool wire;
    double halfWidth;
    double falloffMin;
    double falloffMax;
};

// False when the draw mode synthesizes nothing or a falloff is not finite.
bool
_ReadWeightGuideFacts(const _RestAttrs &attrs, _WeightGuideFacts &facts)
{
    const PXR_NS::TfToken drawMode =
        attrs.token(SchemaAttr::guideDrawMode, SchemaValue::drawModeFalloff);
    const bool clean = drawMode == SchemaValue::drawModeFalloff ||
                       drawMode == SchemaValue::drawModeCage ||
                       drawMode == SchemaValue::drawModeBrackets ||
                       drawMode == SchemaValue::drawModeShell;
    if (drawMode != SchemaValue::drawModeWire &&
        drawMode != SchemaValue::drawModeGeometry && !clean) {
        return false;
    }
    facts.wire = drawMode == SchemaValue::drawModeWire || clean;
    facts.halfWidth = 0.0;
    if (facts.wire) {
        const double width = attrs.number(SchemaAttr::guideWireWidth, 0.05);
        if (std::isfinite(width) && width > 0.0) {
            facts.halfWidth = width * (clean ? 0.15 : 0.5);
        }
    }
    facts.falloffMin = attrs.floatNumber(SchemaAttr::inputsFalloffMin, 0.0);
    facts.falloffMax = attrs.floatNumber(SchemaAttr::inputsFalloffMax, 1.0);
    return std::isfinite(facts.falloffMin) && std::isfinite(facts.falloffMax);
}

// A plane weight's guide rectangle (plus ticks when unbounded) at each
// falloff distance: the exact local range _AppendPlaneVolumeGuide builds.
bool
_AccumulatePlaneWeightDefaultPose(const PXR_NS::UsdPrim &prim,
                                  const _RestAttrs &attrs,
                                  const PXR_NS::GfMatrix4d &rest,
                                  const _WeightGuideFacts &facts,
                                  PXR_NS::GfRange3d &range)
{
    const PXR_NS::TfToken axisName =
        attrs.token(SchemaAttr::rigExecPlaneAxis, SchemaValue::y);
    const int axis = axisName == SchemaValue::x
                         ? 0
                         : (axisName == SchemaValue::y
                                ? 1
                                : (axisName == SchemaValue::z ? 2 : -1));
    const PXR_NS::TfToken bounds =
        attrs.token(SchemaAttr::rigExecPlaneBounds, SchemaValue::unbounded);
    const double extentU = attrs.floatNumber(SchemaAttr::inputsExtentU, 1.0);
    const double extentV = attrs.floatNumber(SchemaAttr::inputsExtentV, 1.0);
    if (axis < 0 ||
        (bounds != SchemaValue::bounded && bounds != SchemaValue::unbounded) ||
        !std::isfinite(extentU) || !std::isfinite(extentV) || extentU <= 0.0 ||
        extentV <= 0.0) {
        return false;
    }
    // Unbounded planes add four outward ticks ending at 1.25 times the
    // rectangle half-extent.
    const double tickScale = bounds == SchemaValue::unbounded ? 1.25 : 1.0;
    PXR_NS::GfRange3d local;
    for (const double distance : {facts.falloffMin, facts.falloffMax}) {
        PXR_NS::GfVec3d min(-facts.halfWidth), max(facts.halfWidth);
        min[axis] += distance;
        max[axis] += distance;
        min[(axis + 1) % 3] -= extentU * tickScale;
        max[(axis + 1) % 3] += extentU * tickScale;
        min[(axis + 2) % 3] -= extentV * tickScale;
        max[(axis + 2) % 3] += extentV * tickScale;
        local.UnionWith(PXR_NS::GfRange3d(min, max));
    }
    const PXR_NS::GfRange3d drawn =
        PXR_NS::GfBBox3d(local, rest).ComputeAlignedRange();
    if (!_ElementBoxUsable(drawn.GetMin(), drawn.GetMax())) {
        return false;
    }
    range.UnionWith(drawn);
    return true;
}

// A sphere weight's guide: both falloff iso-surfaces, each scaled by the
// per-axis and per-side input scales.
bool
_AccumulateSphereWeightDefaultPose(const PXR_NS::UsdPrim &prim,
                                   const _RestAttrs &attrs,
                                   const PXR_NS::GfMatrix4d &rest,
                                   const _WeightGuideFacts &facts,
                                   PXR_NS::GfRange3d &range)
{
    const PXR_NS::GfVec3d axisScale(
        attrs.floatNumber(SchemaAttr::inputsScaleX, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleY, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleZ, 1.0));
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(axisScale[axis]) || axisScale[axis] <= 0.0) {
            return false;
        }
    }
    const PXR_NS::GfVec3d positiveScales(
        attrs.floatNumber(SchemaAttr::inputsScaleXPos, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleYPos, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleZPos, 1.0));
    const PXR_NS::GfVec3d negativeScales(
        attrs.floatNumber(SchemaAttr::inputsScaleXNeg, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleYNeg, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleZNeg, 1.0));
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(positiveScales[axis]) || positiveScales[axis] <= 0 ||
            !std::isfinite(negativeScales[axis]) || negativeScales[axis] <= 0)
            return false;
    }
    // Wire mode draws dotted curves on both iso-surfaces, so each
    // carries guide:wireWidth. Geometry mode draws both solid.
    const double radii[2] = {facts.falloffMin, facts.falloffMax};
    bool found = false;
    for (int surface = 0; surface < 2; ++surface) {
        const double radius = radii[surface];
        if (!std::isfinite(radius) || radius <= 0.0) {
            continue;
        }
        const double pad = facts.wire ? facts.halfWidth : 0.0;
        PXR_NS::GfMatrix4d scale(1.0);
        scale.SetScale(PXR_NS::GfVec3d(radius * axisScale[0],
                                       radius * axisScale[1],
                                       radius * axisScale[2]));
        const PXR_NS::GfVec3d padding(pad);
        const PXR_NS::GfRange3d drawn =
            PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-negativeScales - padding,
                                               positiveScales + padding),
                             scale * rest)
                .ComputeAlignedRange();
        if (!_ElementBoxUsable(drawn.GetMin(), drawn.GetMax())) {
            continue;
        }
        range.UnionWith(drawn);
        found = true;
    }
    return found;
}

// A curve weight's guide: its driver curve expanded by each falloff
// radius. Accepts the same two relationship spellings as the evaluator
// and bridge -- an exact point3f[] property or a prim whose native
// points attribute supplies at least two vertices.
bool
_AccumulateCurveWeightDefaultPose(const PXR_NS::UsdPrim &prim,
                                  const _RestAttrs &attrs,
                                  const PXR_NS::GfMatrix4d &rest,
                                  const _WeightGuideFacts &facts,
                                  PXR_NS::GfRange3d &range)
{
    const PXR_NS::GfVec3d axisScale(
        attrs.floatNumber(SchemaAttr::inputsScaleX, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleY, 1.0),
        attrs.floatNumber(SchemaAttr::inputsScaleZ, 1.0));
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(axisScale[axis]) || axisScale[axis] <= 0.0) {
            return false;
        }
    }
    PXR_NS::SdfPathVector targets;
    if (const PXR_NS::UsdRelationship rel =
            prim.GetRelationship(SchemaAttr::rigExecCurve)) {
        rel.GetTargets(&targets);
    }
    if (targets.size() != 1) {
        return false;
    }
    const PXR_NS::SdfPath pointsPath =
        targets[0].IsPropertyPath()
            ? targets[0]
            : targets[0].AppendProperty(PXR_NS::UsdGeomTokens->points);
    PXR_NS::VtVec3fArray curvePoints;
    const PXR_NS::UsdAttribute points =
        prim.GetStage()->GetAttributeAtPath(pointsPath);
    if (!points || !points.Get(&curvePoints, attrs.time) ||
        curvePoints.size() < 2) {
        return false;
    }
    PXR_NS::GfMatrix4d divide(1.0);
    divide.SetScale(PXR_NS::GfVec3d(1.0 / axisScale[0], 1.0 / axisScale[1],
                                    1.0 / axisScale[2]));
    const PXR_NS::GfMatrix4d toLocal = rest.GetInverse() * divide;
    PXR_NS::GfRange3d localCenters;
    PXR_NS::GfVec3f previous(0.0f);
    size_t distinctPointCount = 0;
    for (const PXR_NS::GfVec3f &point : curvePoints) {
        const PXR_NS::GfVec3f localPoint(
            toLocal.Transform(PXR_NS::GfVec3d(point)));
        if (!_AllFinite(localPoint)) {
            return false;
        }
        // Match _AppendCurveVolumeGuide: coincident consecutive points
        // form no drawable segment and collapse before anything is built.
        if (distinctPointCount == 0 ||
            (localPoint - previous).GetLength() > 1e-6f) {
            localCenters.UnionWith(PXR_NS::GfVec3d(localPoint));
            previous = localPoint;
            ++distinctPointCount;
        }
    }
    if (distinctPointCount < 2) {
        return false;
    }
    PXR_NS::GfMatrix4d scale(1.0);
    scale.SetScale(axisScale);
    const PXR_NS::GfMatrix4d toAsset = scale * rest;
    bool found = false;
    for (const double radius : {facts.falloffMin, facts.falloffMax}) {
        if (!std::isfinite(radius) || radius <= 0.0) {
            continue;
        }
        // The bridge's rings live inside this radius-expanded box;
        // conservative at diagonal tangents beats clipping a valid
        // authored guide when framing cold.
        const PXR_NS::GfVec3d pad(radius + facts.halfWidth);
        const PXR_NS::GfRange3d local(localCenters.GetMin() - pad,
                                      localCenters.GetMax() + pad);
        const PXR_NS::GfRange3d drawn =
            PXR_NS::GfBBox3d(local, toAsset).ComputeAlignedRange();
        if (!_ElementBoxUsable(drawn.GetMin(), drawn.GetMax())) {
            continue;
        }
        range.UnionWith(drawn);
        found = true;
    }
    return found;
}

// The bounds a volume weight's guide would draw at the default pose,
// placed by its rigid rest:space (plane, sphere) or read through the
// rest transform (curve), dispatched by the prim's schema type.
bool
_AccumulateVolumeWeightDefaultPose(const PXR_NS::UsdPrim &prim,
                                   const _RestAttrs &attrs,
                                   const PXR_NS::GfMatrix4d &rest,
                                   PXR_NS::GfRange3d &range)
{
    _WeightGuideFacts facts;
    if (!_ReadWeightGuideFacts(attrs, facts)) {
        return false;
    }
    if (prim.GetTypeName() == SchemaType::RigExecPlaneWeight) {
        return _AccumulatePlaneWeightDefaultPose(prim, attrs, rest, facts,
                                                 range);
    }
    if (prim.GetTypeName() == SchemaType::RigExecSphereWeight) {
        return _AccumulateSphereWeightDefaultPose(prim, attrs, rest, facts,
                                                  range);
    }
    return _AccumulateCurveWeightDefaultPose(prim, attrs, rest, facts, range);
}

// The default-pose answer for a stage that has never been evaluated,
// dispatched by the prim's schema type: a box that collapsed whenever the
// rig was not running would make framing fail where users reach first.
bool
_AccumulateDefaultPoseGuideBounds(const PXR_NS::UsdPrim &prim,
                                  const PXR_NS::UsdTimeCode &time,
                                  PXR_NS::GfRange3d &range)
{
    if (!prim) {
        return false;
    }
    const _RestAttrs attrs{prim, time};
    bool rigid = false;
    const PXR_NS::GfMatrix4d rest =
        rigExec::RigExecAuthoredGuideRestSpace(prim, time, &rigid);
    if (!rigid) {
        return false;
    }
    const PXR_NS::TfToken &type = prim.GetTypeName();
    if (type == SchemaType::RigExecControl) {
        return _AccumulateControlDefaultPose(prim, attrs, rest, range);
    }
    if (type == SchemaType::RigExecJoint) {
        return _AccumulateJointDefaultPose(prim, attrs, rest, range);
    }
    if (type == SchemaType::RigExecSphereWeight ||
        type == SchemaType::RigExecPlaneWeight ||
        type == SchemaType::RigExecCurveWeight) {
        return _AccumulateVolumeWeightDefaultPose(prim, attrs, rest, range);
    }
    return false;
}

// The published bounds in the prim's OWN space: divide out what
// UsdGeomBBoxCache re-applies between the asset root and this prim.
// Conservative -- bounds may be too big, never too small. The rest
// fallback needs no division, so both branches agree as a generation
// publishes.
PXR_NS::GfRange3d
_AssetSpaceToLocal(const PXR_NS::UsdPrim &prim,
                   const PXR_NS::SdfPath &assetRootPath,
                   const PXR_NS::UsdTimeCode &time,
                   const PXR_NS::GfRange3d &range)
{
    if (range.IsEmpty() || assetRootPath.IsEmpty()) {
        return range;
    }
    const PXR_NS::UsdPrim assetRoot =
        prim.GetStage()->GetPrimAtPath(assetRootPath);
    if (!assetRoot || prim == assetRoot) {
        return range;
    }
    PXR_NS::UsdGeomXformCache cache(time);
    bool resetsBelowAsset = false;
    const PXR_NS::GfMatrix4d localToAsset =
        cache.ComputeRelativeTransform(prim, assetRoot, &resetsBelowAsset);
    // A reset detaches the prim from the asset root, so the relative
    // transform is world-space and there is nothing to divide out. The
    // evaluator instead recomputes an asset-relative matrix there.
    if (resetsBelowAsset || localToAsset == PXR_NS::GfMatrix4d(1.0)) {
        return range;
    }
    return PXR_NS::GfBBox3d(range, localToAsset.GetInverse())
        .ComputeAlignedRange();
}

bool
_ComputeRigExecGuideExtent(const PXR_NS::UsdGeomBoundable &boundable,
                           const PXR_NS::UsdTimeCode &time,
                           const PXR_NS::GfMatrix4d *transform,
                           PXR_NS::VtVec3fArray *extent)
{
    const PXR_NS::UsdPrim prim = boundable.GetPrim();
    if (!prim || !extent) {
        return false;
    }
    PXR_NS::GfRange3d range;
    bool found = false;

    // The live generation wins only if it describes THIS stage object at
    // this time; otherwise fall through to the default pose.
    const RigExecImagingRegistry::Ptr context =
        RigExecImagingRegistry::ForStage(prim.GetStage(),
                                         /* create = */ false);
    if (const rigExec::RigExecImagingSnapshotConstPtr snapshot =
            context ? context->GetStore()->Get() : nullptr) {
        if (snapshot->Describes(prim.GetStage(), time)) {
            found = _AccumulateSubtreeGuideBounds(
                *snapshot, prim.GetPath(), _ResolvedPurpose(prim), &range);
            if (found) {
                range =
                    _AssetSpaceToLocal(prim, snapshot->assetRoot, time, range);
            }
        }
    }
    // ...otherwise the default pose, from authored attributes alone, and
    // with the same subtree rule: an extent speaks for nested providers.
    if (!found) {
        const PXR_NS::TfToken purpose = _ResolvedPurpose(prim);
        for (const PXR_NS::UsdPrim &descendant : PXR_NS::UsdPrimRange(prim)) {
            // Purpose-scoped for the same reason the snapshot path is: one
            // extent carries one purpose.
            if (_ResolvedPurpose(descendant) != purpose) {
                continue;
            }
            found =
                _AccumulateDefaultPoseGuideBounds(descendant, time, range) ||
                found;
        }
    }
    if (!found || range.IsEmpty()) {
        return false;
    }
    if (transform) {
        range = PXR_NS::GfBBox3d(range, *transform).ComputeAlignedRange();
    }
    // The extent crosses into float: a box whose corners are finite
    // doubles but outside the float range would reach the host as
    // infinities, so report no extent instead of a poisoned one.
    const PXR_NS::GfVec3f lo(range.GetMin()), hi(range.GetMax());
    if (!_AllFinite(lo) || !_AllFinite(hi)) {
        return false;
    }
    *extent = PXR_NS::VtVec3fArray{lo, hi};
    return true;
}

} // namespace
} // namespace rigExec

PXR_NAMESPACE_OPEN_SCOPE

// Keyed by TfType because the RigExec schema is CODELESS; the lookup
// walks ancestor types, so the abstract base plus the non-inheriting
// aggregates answer for every provider. PXR_NAMESPACE_OPEN_SCOPE is
// required: at global scope this registers into a registry nobody
// subscribes to, failing silently.
TF_REGISTRY_FUNCTION(UsdGeomBoundable)
{
    // RigExecXformable covers joints and controls through its ancestors.
    // Text names, not the SchemaType tokens: Plug loads this dylib on
    // demand, so this body can run mid-load, before inline TfToken
    // globals in any translation unit are constructed. TfType indexes
    // schema types by name text.
    for (const char *name : {rigExec::SchemaTypeName::RigExecXformable,
                             rigExec::SchemaTypeName::RigExecVolumeWeight,
                             rigExec::SchemaTypeName::RigExecClothConstraintSet,
                             rigExec::SchemaTypeName::RigExecFkChain,
                             rigExec::SchemaTypeName::RigExecTwoBoneIk,
                             rigExec::SchemaTypeName::RigExecBlendPointFrames,
                             rigExec::SchemaTypeName::RigExecTwistDistribution,
                             rigExec::SchemaTypeName::RigExecRibbon,
                             rigExec::SchemaTypeName::RigExecSplineIk}) {
        const TfType type = TfType::FindByName(name);
        if (type.IsUnknown()) {
            // The schema plugin is not registered: nothing is Boundable
            // anyway.
            continue;
        }
        UsdGeomRegisterComputeExtentFunction(
            type, rigExec::_ComputeRigExecGuideExtent);
    }
}

PXR_NAMESPACE_CLOSE_SCOPE
