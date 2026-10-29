#include "pxr/base/ts/spline.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "usdSceneAccess.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/resolveInfo.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/base/tf/staticTokens.h"
#include <algorithm>
#include <cmath>
#include <utility>
namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (held)
    (linear)
);
namespace {
VtDictionary Metadata(const UsdObject &object) {
    VtDictionary result;
    auto metadata = object.GetAllMetadata();
    for (auto &[key, value] : metadata) {
        const auto &name = key.GetString();
        if (name != "default" && name != "timeSamples" &&
            name != "spline" && name != "connectionPaths" &&
            name != "targetPaths") result[name] = std::move(value);
    }
    return result;
}
}
SdfPathVector RigExecUsdSceneAccess::Prims(const SdfPath &root) const {
    SdfPathVector result;
    if (!_stage) return result;
    const auto prim = root == SdfPath::AbsoluteRootPath() ? _stage->GetPseudoRoot() : _stage->GetPrimAtPath(root);
    if (prim) for (const auto &entry : UsdPrimRange(prim, UsdPrimAllPrimsPredicate)) result.push_back(entry.GetPath());
    std::sort(result.begin(), result.end());
    return result;
}
bool RigExecUsdSceneAccess::Prim(const SdfPath &path, RigExecScenePrim *result) const {
    if (!_stage || !result) return false;
    const auto prim = _stage->GetPrimAtPath(path);
    if (!prim) return false;
    *result = {path, prim.GetTypeName(), prim.GetAppliedSchemas(), Metadata(prim), prim.IsActive()};
    for (const auto &child : prim.GetAllChildren()) result->children.push_back(child.GetPath());
    return true;
}
SdfPathVector RigExecUsdSceneAccess::Attributes(const SdfPath &path) const {
    SdfPathVector result;
    if (_stage) if (const auto prim = _stage->GetPrimAtPath(path))
        for (const auto &attribute : prim.GetAttributes()) result.push_back(attribute.GetPath());
    std::sort(result.begin(), result.end());
    return result;
}
SdfPathVector RigExecUsdSceneAccess::Relationships(const SdfPath &path) const {
    SdfPathVector result;
    if (_stage) if (const auto prim = _stage->GetPrimAtPath(path))
        for (const auto &relationship : prim.GetRelationships()) result.push_back(relationship.GetPath());
    std::sort(result.begin(), result.end());
    return result;
}
bool RigExecUsdSceneAccess::Attribute(const SdfPath &path, RigExecSceneAttribute *result) const {
    if (!_stage || !result) return false;
    return Attribute(BindAttribute(path),result);
}
UsdAttribute RigExecUsdSceneAccess::BindAttribute(const SdfPath &path) const {
    return _stage ? _stage->GetAttributeAtPath(path) : UsdAttribute();
}
bool RigExecUsdSceneAccess::Attribute(const UsdAttribute &attribute, RigExecSceneAttribute *result) const {
    if (!_stage || !result || !attribute) return false;
    *result = {attribute.GetPath(), attribute.GetTypeName(), Metadata(attribute), {}};
    // Composed connections are empty when nothing authors any, so the
    // authored flag this fact already records gates the composition walk:
    // the overwhelmingly common unconnected attribute skips it. Exact, and
    // no new query: the flag read moves up from below.
    result->hasAuthoredConnections = attribute.HasAuthoredConnections();
    if (result->hasAuthoredConnections)
        attribute.GetConnections(&result->connections);
    // One resolve for the value-source facts: each direct query below
    // recomputes the same timeless resolve info (HasValue, the two authored
    // flags, ValueMightBeTimeVarying, HasSpline, GetTimeSamples), while the
    // query object resolves once and answers every one of them from it --
    // USD documents each answer identical to its direct query.
    const UsdAttributeQuery query(attribute);
    result->hasValue = query.HasValue();
    result->variability = attribute.GetVariability();
    if(query.HasSpline())result->spline=VtValue(query.GetSpline());
    result->hasAuthoredValue = query.HasAuthoredValueOpinion();
    result->hasAuthoredReadableValue = query.HasAuthoredValue();
    result->mightBeTimeVarying = query.ValueMightBeTimeVarying();
    query.GetTimeSamples(&result->sampleTimes);
    for (const auto &spec : attribute.GetPropertyStack(UsdTimeCode::Default())) {
        if (!spec->HasInfo(SdfFieldKeys->Default)) continue;
        result->hasAuthoredDefault = true;
        result->defaultBlocked = spec->GetInfo(SdfFieldKeys->Default).IsHolding<SdfValueBlock>();
        break;
    }
    return true;
}
bool RigExecUsdSceneAccess::Relationship(const SdfPath &path, RigExecSceneRelationship *result) const {
    if (!_stage || !result) return false;
    const auto relationship = _stage->GetRelationshipAtPath(path);
    if (!relationship) return false;
    *result = {path, Metadata(relationship), {}};
    relationship.GetTargets(&result->targets);
    return true;
}
bool RigExecUsdSceneAccess::Resolve(const SdfPath &path, UsdTimeCode time, VtValue *result) const {
    if (!result) return false;
    *result = VtValue();
    if (!HasIdentity(time)) return false;
    return Resolve(BindAttribute(path),time,result);
}
bool RigExecUsdSceneAccess::Resolve(const UsdAttribute &attribute, UsdTimeCode time, VtValue *result) const {
    if (!result) return false;
    *result = VtValue();
    if (!HasIdentity(time) || !attribute || !attribute.GetPrim().IsActive()) return false;
    attribute.Get(result, time);
    return true;
}
bool RigExecUsdSceneAccess::HasIdentity(UsdTimeCode time) const { return _stage && (time.IsDefault() || std::isfinite(time.GetValue())); }
bool RigExecUsdSceneAccess::ValueBlocked(const SdfPath &path, UsdTimeCode time) const {
    if (!HasIdentity(time)) return false;
    return ValueBlocked(BindAttribute(path),time);
}
bool RigExecUsdSceneAccess::ValueBlocked(const UsdAttribute &attribute, UsdTimeCode time) const {
    if (!HasIdentity(time)) return false;
    return attribute && attribute.GetResolveInfo(time).ValueIsBlocked();
}
double RigExecUsdSceneAccess::TimeCodesPerSecond() const { return _stage ? _stage->GetTimeCodesPerSecond() : 24; }
double RigExecUsdSceneAccess::FramesPerSecond() const { return _stage ? _stage->GetFramesPerSecond() : 24; }
TfToken RigExecUsdSceneAccess::Interpolation() const { return _stage && _stage->GetInterpolationType() == UsdInterpolationTypeHeld ? _tokens->held : _tokens->linear; }
TfToken RigExecUsdSceneAccess::UpAxis() const { return _stage?UsdGeomGetStageUpAxis(_stage):UsdGeomTokens->y; }

}
