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
    ((readPhase, "rigExecReadPhase"))
    ((inputElement, "rigExecInputElement"))
    (interpolation)
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
// Capture-subset metadata: the only keys scene translation reads from
// captured facts. Attribute facts: "rigExecReadPhase" (Phase),
// "rigExecInputElement" (sceneTypedReads), "interpolation" (cloth
// inputs). Relationship facts: "rigExecReadPhase" (Phase). Prim facts
// keep nothing: the capture's own clip-stack probe reads "clips", but
// GetAllMetadata filters clip-related keys, so the complete facts never
// carry it -- an empty subset matches them exactly. A new consumer of
// another key extends these lists; until then GetAllMetadata's compose
// of every other field is pure overhead -- each targeted read below
// composes one field through the same _GetMetadataImpl, so a kept key is
// present with the same composed value exactly when the complete facts
// carry it.
template <class Object>
void Keep(VtDictionary *result, const char *name, const TfToken &key,
    const Object &object) {
    VtValue value;
    if (object.GetMetadata(key, &value)) (*result)[name] = std::move(value);
}
VtDictionary CapturedPrimMetadata(const UsdPrim &) {
    return VtDictionary();
}
VtDictionary CapturedAttributeMetadata(const UsdAttribute &attribute) {
    VtDictionary result;
    Keep(&result, "rigExecReadPhase", _tokens->readPhase, attribute);
    Keep(&result, "rigExecInputElement", _tokens->inputElement, attribute);
    Keep(&result, "interpolation", _tokens->interpolation, attribute);
    return result;
}
VtDictionary CapturedRelationshipMetadata(const UsdRelationship &relationship) {
    VtDictionary result;
    Keep(&result, "rigExecReadPhase", _tokens->readPhase, relationship);
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
bool RigExecUsdSceneAccess::PrimProperties(const SdfPath &path, RigExecScenePrim *result,
    SdfPathVector *attributes, SdfPathVector *relationships,
    std::vector<UsdAttribute> *boundAttributes, bool subsetMetadata) const {
    if (!_stage || !result || !attributes || !relationships) return false;
    const auto prim = _stage->GetPrimAtPath(path);
    if (!prim) return false;
    *result = {path, prim.GetTypeName(), prim.GetAppliedSchemas(),
        subsetMetadata ? CapturedPrimMetadata(prim) : Metadata(prim), prim.IsActive()};
    for (const auto &child : prim.GetAllChildren()) result->children.push_back(child.GetPath());
    // One property enumeration serves both lists: GetAttributes and
    // GetRelationships each compose the prim's property names, while
    // GetProperties composes them once. A property is exactly one of the
    // two kinds, and both lists are sorted after, so the split answers
    // the same sets in the same order.
    std::vector<std::pair<SdfPath, UsdAttribute>> pairs;
    SdfPathVector relPaths;
    for (const auto &property : prim.GetProperties()) {
        if (const auto attribute = property.As<UsdAttribute>())
            pairs.emplace_back(attribute.GetPath(), attribute);
        else
            relPaths.push_back(property.GetPath());
    }
    std::sort(pairs.begin(), pairs.end(),
        [](const auto &a, const auto &b) { return a.first < b.first; });
    attributes->clear();
    attributes->reserve(pairs.size());
    if (boundAttributes) {
        // Bound objects stay parallel to the sorted paths: collect pairs,
        // sort once on the materialized paths, then split.
        boundAttributes->clear();
        boundAttributes->reserve(pairs.size());
        for (auto &entry : pairs) {
            attributes->push_back(std::move(entry.first));
            boundAttributes->push_back(std::move(entry.second));
        }
    } else {
        for (auto &entry : pairs) attributes->push_back(std::move(entry.first));
    }
    std::sort(relPaths.begin(), relPaths.end());
    *relationships = std::move(relPaths);
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
bool RigExecUsdSceneAccess::Attribute(const UsdAttribute &attribute, RigExecSceneAttribute *result, bool clipped, bool subsetMetadata) const {
    if (!_stage || !result || !attribute) return false;
    *result = {attribute.GetPath(), attribute.GetTypeName(),
        subsetMetadata ? CapturedAttributeMetadata(attribute) : Metadata(attribute), {}};
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
    // No variability query: no scene translation reads the captured fact
    // (only the standalone Db carries variability, populated separately),
    // so the field keeps its default. Re-add the query if one ever does.
    const bool spline = query.HasSpline();
    if(spline)result->spline=VtValue(query.GetSpline());
    result->hasAuthoredValue = query.HasAuthoredValueOpinion();
    result->hasAuthoredReadableValue = query.HasAuthoredValue();
    result->mightBeTimeVarying = query.ValueMightBeTimeVarying();
    // Time samples arrive only through authored opinions, splines, or value
    // clips, so without all three the list is empty and the stack walk is
    // skipped. A block is itself an opinion, so blocked attributes still
    // walk; mightBeTimeVarying stays as a backstop for any other source.
    if (result->hasAuthoredValue || spline || result->mightBeTimeVarying || clipped)
        query.GetTimeSamples(&result->sampleTimes);
    // No default-opinion walk: no scene translation reads the captured
    // hasAuthoredDefault/defaultBlocked facts (only the standalone Db
    // carries them, populated separately from the stage specs), so the
    // fields keep their defaults. Re-add the walk if one ever does.
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
bool RigExecUsdSceneAccess::RelationshipCaptured(const SdfPath &path, RigExecSceneRelationship *result) const {
    if (!_stage || !result) return false;
    const auto relationship = _stage->GetRelationshipAtPath(path);
    if (!relationship) return false;
    *result = {path, CapturedRelationshipMetadata(relationship), {}};
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
bool RigExecUsdSceneAccess::ResolveActive(const UsdAttribute &attribute, UsdTimeCode time, VtValue *result) const {
    if (!result) return false;
    *result = VtValue();
    if (!HasIdentity(time) || !attribute) return false;
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
