#include "pathOperation.h"

#include "rigExecMath/propertyMath.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/basisCurves.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <cmath>

PXR_NAMESPACE_USING_DIRECTIVE

// Interned once: binders run on parallel lanes, and building a token from
// text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_pathTokens,
    ((path, "rigExec:path"))
    ((pathComponent, "rigExec:pathComponent"))
    ((pathOffset, "rigExec:pathOffset"))
    ((pathHeadingOffset, "rigExec:pathHeadingOffset"))
    ((pathMeasure, "rigExec:pathMeasure"))
    (x)
    (length));

namespace rigExec {

namespace {

bool
_Fail(std::string *error, const std::string &why)
{
    if (error) {
        *error = why;
    }
    return false;
}

template <class T>
T
_Uniform(const UsdPrim &prim, const TfToken &name, const T &fallback,
         std::vector<SdfPath> *structural)
{
    T value = fallback;
    structural->push_back(prim.GetPath().AppendProperty(name));
    if (const UsdAttribute a = prim.GetAttribute(name)) {
        a.Get(&value, UsdTimeCode::Default());
    }
    return value;
}

}  // namespace

bool
RigExecBindPathOperation(const UsdPrim &mover, RigExecPathBinding *out,
                         std::string *error)
{
    const TfToken &pathRel = _pathTokens->path;
    const TfToken &componentName = _pathTokens->pathComponent;
    const TfToken &offsetName = _pathTokens->pathOffset;
    const TfToken &headingName = _pathTokens->pathHeadingOffset;
    const TfToken &measureName = _pathTokens->pathMeasure;
    *out = RigExecPathBinding();
    if (!mover) {
        return _Fail(error, "no mover prim");
    }
    const std::string who = mover.GetPath().GetString();
    out->structural.push_back(mover.GetPath().AppendProperty(pathRel));
    const TfToken component =
        _Uniform(mover, componentName, _pathTokens->x, &out->structural);
    const GfVec2f offset =
        _Uniform(mover, offsetName, GfVec2f(0.0f), &out->structural);
    const float heading =
        _Uniform(mover, headingName, 0.0f, &out->structural);
    const TfToken measure =
        _Uniform(mover, measureName, _pathTokens->length, &out->structural);
    RigExecPathSettings &settings = out->settings;
    settings.offsetX = offset[0];
    settings.offsetZ = offset[1];
    settings.headingOffset = heading;
    if (!RigExecParsePathComponent(component, &settings.component)) {
        return _Fail(error, who + ": unknown rigExec:pathComponent '" +
                                component.GetString() + "'");
    }
    if (measure != "length" && measure != "planar") {
        return _Fail(error, who + ": unknown rigExec:pathMeasure '" +
                                measure.GetString() + "'");
    }
    settings.planar = measure == "planar";
    if (!std::isfinite(offset[0]) || !std::isfinite(offset[1]) ||
        !std::isfinite(heading)) {
        return _Fail(error, who + ": rigExec:pathOffset and "
                                  "rigExec:pathHeadingOffset must be finite");
    }
    SdfPathVector targets;
    if (const UsdRelationship rel = mover.GetRelationship(pathRel)) {
        rel.GetTargets(&targets);
    }
    if (targets.size() != 1 || !targets[0].IsPrimPath()) {
        return _Fail(error, who + ": rigExec:path must name exactly one "
                                  "UsdGeomBasisCurves prim");
    }
    const UsdPrim curvePrim = mover.GetStage()->GetPrimAtPath(targets[0]);
    const UsdGeomBasisCurves curves(curvePrim);
    if (!curvePrim || !curvePrim.IsA<UsdGeomBasisCurves>()) {
        return _Fail(error, who + ": rigExec:path target " +
                                targets[0].GetString() +
                                " is not a UsdGeomBasisCurves prim");
    }
    out->curve = targets[0];
    out->points = curves.GetPointsAttr();
    const SdfPath curvePath = curvePrim.GetPath();
    for (const TfToken &name :
         {UsdGeomTokens->type, UsdGeomTokens->basis, UsdGeomTokens->wrap,
          UsdGeomTokens->curveVertexCounts}) {
        out->structural.push_back(curvePath.AppendProperty(name));
    }
    TfToken type, basis, wrap;
    curves.GetTypeAttr().Get(&type, UsdTimeCode::Default());
    curves.GetBasisAttr().Get(&basis, UsdTimeCode::Default());
    curves.GetWrapAttr().Get(&wrap, UsdTimeCode::Default());
    if (!RigExecParsePathBasis(type, basis, wrap, &settings.basis,
                               &settings.pinned)) {
        return _Fail(error, who + ": path curve " + curvePath.GetString() +
                                " is " + type.GetString() + "/" +
                                basis.GetString() + "/" + wrap.GetString() +
                                "; a path is linear or cubic bezier, bspline "
                                "or catmullRom, nonperiodic or pinned");
    }
    // One curve. Counts are checked against the points the compile sees;
    // a points input set later keeps its element count (the runtime refuses
    // another), so the shape stands.
    VtIntArray counts;
    curves.GetCurveVertexCountsAttr().Get(&counts, UsdTimeCode::Default());
    VtVec3fArray points;
    if (!out->points.Get(&points, UsdTimeCode::Default())) {
        out->points.Get(&points, UsdTimeCode::EarliestTime());
    }
    if (counts.size() > 1 ||
        (counts.size() == 1 && size_t(counts[0]) != points.size())) {
        return _Fail(error, who + ": path curve " + curvePath.GetString() +
                                " must hold exactly one curve whose vertex "
                                "count matches its points");
    }
    RigExecPathCurve curve;
    curve.points = points.empty() ? nullptr : points.cdata()->data();
    curve.pointCount = points.size();
    curve.basis = settings.basis;
    curve.pinned = settings.pinned;
    curve.planar = settings.planar;
    if (!RigExecPathCurveValid(curve)) {
        return _Fail(error, who + ": path curve " + curvePath.GetString() +
                                " has too few points for its basis, or a "
                                "point that is not finite");
    }
    settings.valid = true;
    return true;
}

}  // namespace rigExec
