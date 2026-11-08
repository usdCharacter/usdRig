// RigExec guide geometry queries: the UsdGeomBoundable extent callback,
// its purpose resolution, the authored-rest fallback, and the asset-space
// to local conversion. Bounds accumulation lives in guideBounds.cpp.
#include "guideBoundsInternal.h"

#include "registry.h"
#include "rigExecMath/avarScale.h"
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

namespace rigExec {
namespace {

// A prim's resolved render purpose -- the same value the bridge publishes
// and the same one UsdGeomBBoxCache files its extent under.
PXR_NS::TfToken
_ResolvedPurpose(const PXR_NS::UsdPrim &prim)
{
    if (const PXR_NS::UsdGeomImageable imageable =
            PXR_NS::UsdGeomImageable(prim)) {
        const PXR_NS::TfToken purpose = imageable.ComputePurpose();
        if (!purpose.IsEmpty()) {
            return purpose;
        }
    }
    return PXR_NS::UsdGeomTokens->default_;
}

// The bounds a provider's guide would draw AT REST, from authored
// attributes alone.
// This is the answer for a stage that has never been evaluated -- opened in
// a host that has not activated RigExec, or queried before the first
// generation is published. It is deliberately the rest pose rather than
// nothing: a bounding box that collapses the moment the rig is not running
// makes framing fail exactly where a user reaches for it first.
bool
_AccumulateRestGuideBounds(const PXR_NS::UsdPrim &prim,
                           const PXR_NS::UsdTimeCode &time,
                           PXR_NS::GfRange3d *range)
{
    auto number = [&prim, &time](const char *name, double fallback) {
        double value = fallback;
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    auto floatNumber = [&prim, &time](const char *name, double fallback) {
        float value = static_cast<float>(fallback);
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return double(value);
    };
    auto token = [&prim, &time](const char *name, const char *fallback) {
        PXR_NS::TfToken value(fallback);
        if (const PXR_NS::UsdAttribute a =
                prim.GetAttribute(PXR_NS::TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    bool rigid = false;
    const PXR_NS::GfMatrix4d rest =
        rigExec::RigExecAuthoredGuideRestSpace(prim, time, &rigid);
    if (!rigid) {
        return false;
    }
    const PXR_NS::TfToken type = prim.GetTypeName();

    if (type == "RigExecControl") {
        if (floatNumber("guide:displayOpacity", 0.5) <= 0.0)
            return false;
        // Same predicate the scene index draws through: an unrecognized
        // shape/drawMode pair synthesizes nothing and must bound nothing.
        const PXR_NS::TfToken shape = token("guide:shape", "circle");
        const PXR_NS::TfToken drawMode = token("guide:drawMode", "wire");
        if (!rigExec::RigExecControlGuideIsDrawn(shape, drawMode)) {
            return false;
        }
        const PXR_NS::GfVec3d authoredScale(number("guide:scaleX", 1.0),
                                            number("guide:scaleY", 1.0),
                                            number("guide:scaleZ", 1.0));
        PXR_NS::GfVec3d scale(1.0);
        static const char *avarScaleNames[3] = {"avars:sx", "avars:sy",
                                                "avars:sz"};
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(authoredScale[i]) || authoredScale[i] <= 0.0) {
                return false; // draws nothing, so it bounds nothing
            }
            // No evaluated frame exists on this fallback path. Resolve the
            // authored channel at the queried time with the SAME floor and
            // non-finite policy as the runtime, then size the guide by its
            // magnitude so a supported reflection remains visible.
            const double avarMagnitude =
                std::abs(rigExec::RigExecNormalizeAvarScale(
                    number(avarScaleNames[i], 1.0)));
            scale[i] = authoredScale[i] * avarMagnitude;
            if (!std::isfinite(scale[i]) || scale[i] <= 0.0) {
                return false;
            }
        }
        double halfWidth = 0.0;
        if (drawMode == PXR_NS::TfToken("wire")) {
            const double width = number("guide:wireWidth", 0.05);
            if (std::isfinite(width) && width > 0.0) {
                halfWidth = width * 0.5;
            }
        }
        if (shape == PXR_NS::TfToken("custom")) {
            PXR_NS::VtVec3fArray points;
            PXR_NS::VtIntArray counts;
            prim.GetAttribute(PXR_NS::TfToken("guide:points"))
                .Get(&points, time);
            prim.GetAttribute(PXR_NS::TfToken("guide:curveVertexCounts"))
                .Get(&counts, time);
            PXR_NS::GfRange3d local = _CustomControlRange(points, counts);
            if (local.IsEmpty())
                return false;
            const PXR_NS::GfVec3d pad(halfWidth);
            local =
                PXR_NS::GfRange3d(local.GetMin() - pad, local.GetMax() + pad);
            PXR_NS::GfVec3d signedScale;
            for (int axis = 0; axis < 3; ++axis) {
                signedScale[axis] = rigExec::RigExecNormalizeAvarScale(
                    number(avarScaleNames[axis], 1.0));
            }
            PXR_NS::GfVec3d offset(0.0);
            prim.GetAttribute(PXR_NS::TfToken("guide:offset"))
                .Get(&offset, time);
            PXR_NS::GfMatrix4d scaleMatrix(1.0);
            scaleMatrix.SetScale(signedScale);
            PXR_NS::GfMatrix4d shapeScale(1.0);
            shapeScale.SetScale(authoredScale);
            const PXR_NS::GfMatrix4d placement =
                shapeScale * PXR_NS::GfMatrix4d(1.0).SetTranslate(offset) *
                scaleMatrix * rest;
            range->UnionWith(
                PXR_NS::GfBBox3d(local, placement).ComputeAlignedRange());
            return true;
        }
        // The unit shape, for the same reason the snapshot path uses it:
        // every guide shape is documented as unit-sized with half-extent 1.
        const PXR_NS::GfVec3d half(scale[0] * (1.0 + halfWidth),
                                   scale[1] * (1.0 + halfWidth),
                                   scale[2] * (1.0 + halfWidth));
        range->UnionWith(PXR_NS::GfBBox3d(PXR_NS::GfRange3d(-half, half), rest)
                             .ComputeAlignedRange());
        return true;
    }
    if (type == "RigExecJoint") {
        const double radius = number("guide:radius", 1.0);
        if (!std::isfinite(radius) || radius <= 0.0) {
            return false;
        }
        const PXR_NS::GfVec3d origin = rest.ExtractTranslation();
        const PXR_NS::GfVec3d extent(radius, radius, radius);
        range->UnionWith(origin - extent);
        range->UnionWith(origin + extent);

        // A joint's outgoing links terminate at every descendant whose
        // nearest RigExecJoint ancestor is this prim. Intervening grouping
        // scopes therefore preserve the same hierarchy used by evaluation.
        for (const PXR_NS::UsdPrim &candidate : PXR_NS::UsdPrimRange(prim)) {
            if (candidate == prim ||
                candidate.GetTypeName() != "RigExecJoint") {
                continue;
            }
            PXR_NS::UsdPrim ancestor = candidate.GetParent();
            while (ancestor && ancestor != prim &&
                   ancestor.GetTypeName() != "RigExecJoint") {
                ancestor = ancestor.GetParent();
            }
            if (ancestor != prim) {
                continue;
            }
            bool childRigid = false;
            const PXR_NS::GfMatrix4d childRest =
                rigExec::RigExecAuthoredGuideRestSpace(candidate, time,
                                                       &childRigid);
            if (!childRigid) {
                continue;
            }
            const PXR_NS::GfVec3d tip = childRest.ExtractTranslation();
            if (!std::isfinite(tip[0]) || !std::isfinite(tip[1]) ||
                !std::isfinite(tip[2]) || (tip - origin).GetLength() <= 1e-12) {
                continue;
            }
            range->UnionWith(tip - extent);
            range->UnionWith(tip + extent);
        }
        return true;
    }
    if (type == "RigExecSphereWeight" || type == "RigExecPlaneWeight" ||
        type == "RigExecCurveWeight") {
        const PXR_NS::TfToken drawMode = token("guide:drawMode", "falloff");
        const bool clean = drawMode == rigExec::_tWeightFalloff ||
                           drawMode == rigExec::_tWeightCage ||
                           drawMode == rigExec::_tWeightBrackets ||
                           drawMode == rigExec::_tWeightShell;
        if (drawMode != rigExec::_tWeightWire &&
            drawMode != rigExec::_tWeightGeometry && !clean) {
            return false;
        }
        const bool wire = drawMode == rigExec::_tWeightWire || clean;
        double halfWidth = 0.0;
        if (wire) {
            const double width = number("guide:wireWidth", 0.05);
            if (std::isfinite(width) && width > 0.0) {
                halfWidth = width * (clean ? 0.15 : 0.5);
            }
        }
        const double falloffMin = floatNumber("inputs:falloffMin", 0.0);
        const double falloffMax = floatNumber("inputs:falloffMax", 1.0);
        if (!std::isfinite(falloffMin) || !std::isfinite(falloffMax)) {
            return false;
        }

        if (type == "RigExecPlaneWeight") {
            const PXR_NS::TfToken axisName = token("rigExec:planeAxis", "y");
            const int axis =
                axisName == PXR_NS::TfToken("x")
                    ? 0
                    : (axisName == PXR_NS::TfToken("y")
                           ? 1
                           : (axisName == PXR_NS::TfToken("z") ? 2 : -1));
            const PXR_NS::TfToken bounds =
                token("rigExec:planeBounds", "unbounded");
            const double extentU = floatNumber("inputs:extentU", 1.0);
            const double extentV = floatNumber("inputs:extentV", 1.0);
            if (axis < 0 ||
                (bounds != PXR_NS::TfToken("bounded") &&
                 bounds != PXR_NS::TfToken("unbounded")) ||
                !std::isfinite(extentU) || !std::isfinite(extentV) ||
                extentU <= 0.0 || extentV <= 0.0) {
                return false;
            }
            // Unbounded planes add four outward ticks ending at 1.25 times
            // the rectangle half-extent. This is the exact local range of the
            // guide elements built by _AppendPlaneVolumeGuide.
            const double tickScale =
                bounds == PXR_NS::TfToken("unbounded") ? 1.25 : 1.0;
            PXR_NS::GfRange3d local;
            for (const double distance : {falloffMin, falloffMax}) {
                PXR_NS::GfVec3d min(-halfWidth), max(halfWidth);
                min[axis] += distance;
                max[axis] += distance;
                min[(axis + 1) % 3] -= extentU * tickScale;
                max[(axis + 1) % 3] += extentU * tickScale;
                min[(axis + 2) % 3] -= extentV * tickScale;
                max[(axis + 2) % 3] += extentV * tickScale;
                local.UnionWith(PXR_NS::GfRange3d(min, max));
            }
            range->UnionWith(
                PXR_NS::GfBBox3d(local, rest).ComputeAlignedRange());
            return true;
        }

        PXR_NS::GfVec3d axisScale(floatNumber("inputs:scaleX", 1.0),
                                  floatNumber("inputs:scaleY", 1.0),
                                  floatNumber("inputs:scaleZ", 1.0));
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(axisScale[axis]) || axisScale[axis] <= 0.0) {
                return false;
            }
        }
        if (type == "RigExecSphereWeight") {
            const PXR_NS::GfVec3d positiveScales(
                floatNumber("inputs:scaleXPos", 1.0),
                floatNumber("inputs:scaleYPos", 1.0),
                floatNumber("inputs:scaleZPos", 1.0));
            const PXR_NS::GfVec3d negativeScales(
                floatNumber("inputs:scaleXNeg", 1.0),
                floatNumber("inputs:scaleYNeg", 1.0),
                floatNumber("inputs:scaleZNeg", 1.0));
            for (int axis = 0; axis < 3; ++axis) {
                if (!std::isfinite(positiveScales[axis]) ||
                    positiveScales[axis] <= 0 ||
                    !std::isfinite(negativeScales[axis]) ||
                    negativeScales[axis] <= 0)
                    return false;
            }
            // Wire mode draws dotted curves on both iso-surfaces, so each
            // carries guide:wireWidth. Geometry mode draws both solid.
            const double radii[2] = {falloffMin, falloffMax};
            bool found = false;
            for (int surface = 0; surface < 2; ++surface) {
                const double radius = radii[surface];
                if (!std::isfinite(radius) || radius <= 0.0) {
                    continue;
                }
                const double pad = wire ? halfWidth : 0.0;
                PXR_NS::GfMatrix4d scale(1.0);
                scale.SetScale(PXR_NS::GfVec3d(radius * axisScale[0],
                                               radius * axisScale[1],
                                               radius * axisScale[2]));
                const PXR_NS::GfVec3d padding(pad);
                range->UnionWith(PXR_NS::GfBBox3d(PXR_NS::GfRange3d(
                                                      -negativeScales - padding,
                                                      positiveScales + padding),
                                                  scale * rest)
                                     .ComputeAlignedRange());
                found = true;
            }
            return found;
        }

        // CurveWeight: accept the same two relationship spellings as the
        // evaluator and bridge -- an exact point3f[] property or a prim whose
        // native `points` attribute supplies at least two vertices.
        PXR_NS::SdfPathVector targets;
        if (const PXR_NS::UsdRelationship rel =
                prim.GetRelationship(PXR_NS::TfToken("rigExec:curve"))) {
            rel.GetTargets(&targets);
        }
        if (targets.size() != 1) {
            return false;
        }
        const PXR_NS::SdfPath pointsPath =
            targets[0].IsPropertyPath()
                ? targets[0]
                : targets[0].AppendProperty(PXR_NS::TfToken("points"));
        PXR_NS::VtVec3fArray curvePoints;
        const PXR_NS::UsdAttribute points =
            prim.GetStage()->GetAttributeAtPath(pointsPath);
        if (!points || !points.Get(&curvePoints, time) ||
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
            if (!std::isfinite(localPoint[0]) ||
                !std::isfinite(localPoint[1]) ||
                !std::isfinite(localPoint[2])) {
                return false;
            }
            // Match _AppendCurveVolumeGuide: consecutive coincident points
            // do not form a drawable segment and are collapsed before the
            // wire or mesh is built.
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
        for (const double radius : {falloffMin, falloffMax}) {
            if (!std::isfinite(radius) || radius <= 0.0) {
                continue;
            }
            // The bridge's rings live inside this radius-expanded box. It is
            // conservative at diagonal tangents, which is preferable to a
            // cold framing bound that clips a valid authored guide.
            const PXR_NS::GfVec3d pad(radius + halfWidth);
            const PXR_NS::GfRange3d local(localCenters.GetMin() - pad,
                                          localCenters.GetMax() + pad);
            range->UnionWith(
                PXR_NS::GfBBox3d(local, toAsset).ComputeAlignedRange());
            found = true;
        }
        return found;
    }
    return false;
}

// UsdGeomBoundable::ComputeExtentFromPlugins entry point for every RigExec
// transform provider.
// This is what makes framing a control work in EVERY host rather than in
// the one whose Python we could reach. UsdGeomBBoxCache is what usdview
// and Solaris both consult, and it asks a Boundable for its extent; a type
// that is not Boundable reports nothing, so a rig would have no bounds
// anywhere and framing a control would not move the camera.
// The extent BAKES the posed frame, because the prim carries no stage
// transform of its own: rest:space plus avars are the only transform
// authority (the Ir alignment), and they are asset-relative.
// "Asset-relative" is not "local", and an extent is LOCAL --
// UsdGeomBBoxCache multiplies it by the prim's own local-to-world. The two
// coincide only while nothing between the asset root and the provider
// contributes a transform. Something may: an intervening
// Xformable is composed into the published frames
// (RigExecRigEvaluator::_ComposeInterveningXforms), so the snapshot branch
// below divides it back out. Leaving it in draws the guide in one place and
// its bounding box in another, offset by exactly that Xform.
// The rest fallback needs no such division: it reads authored rest
// attributes alone, which never carried the intervening transform in the
// first place. Both branches therefore return the same local quantity, and
// the box does not jump when a generation is published.
// The published bounds expressed in the prim's OWN space, by dividing out
// everything UsdGeomBBoxCache is about to re-apply between the asset root
// and this prim.
// That is the whole namespace chain, the prim's own xformOps included. Ops
// authored on a provider are not a transform authority for EVALUATION --
// rest:space and the avars are -- but BBoxCache applies them regardless, so
// an extent that ignored them would be wrong in the same way and for the
// same reason.
// Conservative rather than tight: an inverse-rotated box has to be
// re-aligned to axes to be expressed as a min/max pair, and USD re-aligns
// it again on the way out, so a rotated chain grows the box slightly.
// Bounds are allowed to be too big and are never allowed to be too small.
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
    // A reset detaches the prim from the asset root, so there is no
    // relative transform to divide out and guessing at one would move the
    // box somewhere nobody asked for. Matches what the evaluator does with
    // the same condition.
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

    // The live generation wins -- but ONLY if it describes this stage at
    // this time. USD calls this with a stage and a time of its own
    // choosing, so the generation consulted is the one of the queried
    // stage's OWN imaging context (never another stage's, even with equal
    // paths), and it must still describe that stage at that time: a query
    // at frame 12 is not answered from the frame-30 generation. Identity is
    // the stage OBJECT -- two stages commonly share a root layer and differ
    // only by session layer. Anything else falls through to the rest pose,
    // which makes this callback a pure function of (stage, time).
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
    // ...otherwise the rest pose, from authored attributes alone, so an
    // un-evaluated stage still frames. The subtree rule applies here too:
    // BBoxCache stops descending at a Boundable, so a provider's extent
    // has to speak for the providers nested under it.
    if (!found) {
        const PXR_NS::TfToken purpose = _ResolvedPurpose(prim);
        for (const PXR_NS::UsdPrim &descendant : PXR_NS::UsdPrimRange(prim)) {
            // Purpose-scoped for the same reason the snapshot path is: one
            // extent carries one purpose.
            if (_ResolvedPurpose(descendant) != purpose) {
                continue;
            }
            found =
                _AccumulateRestGuideBounds(descendant, time, &range) || found;
        }
    }
    if (!found || range.IsEmpty()) {
        return false;
    }
    if (transform) {
        range = PXR_NS::GfBBox3d(range, *transform).ComputeAlignedRange();
    }
    *extent = PXR_NS::VtVec3fArray{PXR_NS::GfVec3f(range.GetMin()),
                                   PXR_NS::GfVec3f(range.GetMax())};
    return true;
}

} // namespace
} // namespace rigExec

PXR_NAMESPACE_OPEN_SCOPE

// Keyed by TfType rather than by the templated overload: the RigExec schema
// is CODELESS, so there is no C++ class to name as a template argument. The
// type still exists -- Plug declares it from the schema plugInfo, and its
// ancestor chain reaches UsdGeomBoundable -- which is all
// ComputeExtentFromPlugins needs to find this.
// Registered on the abstract base, not on each concrete type: the lookup
// walks a prim's ancestor types, so one registration answers for every
// provider that inherits it.
// Inside PXR_NAMESPACE_OPEN_SCOPE, like every other TF_REGISTRY_FUNCTION in
// this tree -- the macro's tag type has to resolve the way usdGeom's own
// subscription resolves it, and at global scope with a PXR_NS:: qualifier
// it registers into a registry nobody subscribes to, which fails silently.
TF_REGISTRY_FUNCTION(UsdGeomBoundable)
{
    // RigExecXformable covers joints and controls through its ancestors. The
    // volume base gets its own registration so its shape-specific authored
    // fallback can be framed cold; aggregate solvers draw guides too but
    // inherit Boundable directly, so they are named individually.
    for (const char *name :
         {"RigExecXformable", "RigExecVolumeWeight",
          "RigExecClothConstraintSet", "RigExecFkChain", "RigExecTwoBoneIk",
          "RigExecBlendPointFrames", "RigExecTwistDistribution",
          "RigExecRibbon", "RigExecSplineIk"}) {
        const TfType type = TfType::FindByName(name);
        if (type.IsUnknown()) {
            // The schema plugin is not registered in this process, so
            // nothing can be Boundable anyway. Not worth shouting about.
            continue;
        }
        UsdGeomRegisterComputeExtentFunction(
            type, rigExec::_ComputeRigExecGuideExtent);
    }
}

PXR_NAMESPACE_CLOSE_SCOPE
