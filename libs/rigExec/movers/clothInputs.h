// Relationship translation for cloth data. All stage reads happen while
// declaring and sampling leaves, on the owning thread; assembly below
// consumes only captured typed values. ExpandUV, Samples, ReadMap and
// Assemble run inside operation bodies (the window solve builds the
// material model once per rig compilation, live and frozen): they take no lock and
// build no token from text, so they compare tokens with literal text,
// which does not intern.
#ifndef RIGEXEC_CLOTH_INPUTS_H
#define RIGEXEC_CLOTH_INPUTS_H

#include "moverRegistry.h"
#include "../moverGraph.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/usd/usd/stage.h"
#include <functional>

namespace cloth_inputs {
using namespace rigExec;

TF_DEFINE_PRIVATE_TOKENS(_clothInputsTokens,
    (points)
    (faceVertexCounts)
    (faceVertexIndices)
    (orientation)
    (rightHanded)
    (vertex)
    (interpolation)
    (stretch)
    (soft)
    ((primvarsSt, "primvars:st"))
    ((inputsValues, "inputs:values"))
    ((rigExecMode, "rigExec:mode"))
    ((rigExecIndices, "rigExec:indices"))
    ((rigExecCoordinates, "rigExec:coordinates"))
    ((rigExecUvSet, "rigExec:uvSet"))
    ((rigExecValues, "rigExec:values"))
    ((rigExecSourceMesh, "rigExec:sourceMesh"))
    ((rigExecBinding, "rigExec:binding"))
    ((rigExecRestShape, "rigExec:restShape"))
    ((rigExecConstraints, "rigExec:constraints"))
    ((rigExecKind, "rigExec:kind"))
    ((rigExecTwoSided, "rigExec:twoSided"))
    ((rigExecForceCurve, "rigExec:forceCurve"))
    ((rigExecTargetMesh, "rigExec:targetMesh"))
    ((rigExecSourceBinding, "rigExec:sourceBinding"))
    ((rigExecTargetBinding, "rigExec:targetBinding"))
    ((inputsMinStrength, "inputs:minStrength"))
    ((inputsMaxStrength, "inputs:maxStrength"))
    ((inputsCompliance, "inputs:compliance"))
    ((inputsWarp, "inputs:warp"))
    ((inputsWeft, "inputs:weft"))
    ((inputsNormal, "inputs:normal"))
    ((inputsThickness, "inputs:thickness"))
    ((inputsFriction, "inputs:friction"))
    ((inputsRestLength, "inputs:restLength"))
    ((rigExecMaskMap, "rigExec:maskMap"))
    ((rigExecMinStrengthMap, "rigExec:minStrengthMap"))
    ((rigExecMaxStrengthMap, "rigExec:maxStrengthMap"))
    ((rigExecWarpMap, "rigExec:warpMap"))
    ((rigExecWeftMap, "rigExec:weftMap"))
    ((rigExecNormalMap, "rigExec:normalMap"))
    ((rigExecThicknessMap, "rigExec:thicknessMap"))
    ((rigExecFrictionMap, "rigExec:frictionMap")));

using Relationships = std::function<SdfPathVector(const SdfPath &, const TfToken &)>;
using Interpolation = std::function<TfToken(const SdfPath &)>;
using Dictionary = VtDictionary;

inline SdfPath One(const Relationships &relationships, const SdfPath &prim, const TfToken &name)
{
    const auto paths = relationships(prim, name);
    return paths.size() == 1 ? paths[0] : SdfPath();
}

inline int Index(const Dictionary &d, const char *name)
{
    const auto it = d.find(name);
    return it != d.end() && it->second.IsHolding<int>() ? it->second.UncheckedGet<int>() : -1;
}

inline const Dictionary *Object(const Dictionary &d, const char *name)
{
    const auto it = d.find(name);
    return it != d.end() && it->second.IsHolding<Dictionary>()
        ? &it->second.UncheckedGet<Dictionary>() : nullptr;
}

template <class T> inline const T *Value(const std::vector<VtValue> &in,
                                         const Dictionary &d, const char *name)
{
    const int i = Index(d, name);
    return i >= 0 && size_t(i) < in.size() && in[size_t(i)].IsHolding<T>()
        ? &in[size_t(i)].UncheckedGet<T>() : nullptr;
}

struct Declaration {
    std::vector<RigExecRevisionLeafKey> *inputs;
    RigExecRevisionBinding *binding;
    Relationships relationships;
    Interpolation interpolation;
    std::function<TfToken(const SdfPath &)> type;
    bool valid = true;

    SdfPathVector Targets(const SdfPath &prim, const TfToken &name)
    {
        binding->externalStructure.push_back(prim.AppendProperty(name));
        return relationships(prim, name);
    }
    SdfPath Target(const SdfPath &prim, const TfToken &name)
    {
        const auto paths = Targets(prim, name);
        if (paths.size() > 1) valid = false;
        return paths.size() == 1 ? paths[0] : SdfPath();
    }
    SdfPath Target(const SdfPath &prim, const char *name) { return Target(prim, TfToken(name)); }
    void Field(Dictionary *d, const char *key, const SdfPath &path,
               RigExecRevisionLeafType type, VtValue fallback)
    {
        (*d)[key] = VtValue(int(inputs->size()));
        inputs->push_back({path, type, RigExecRevisionLeafTime::AtDefault,
                          RigExecRevisionLeafFlavour::Raw, std::move(fallback)});
    }
    void Attribute(Dictionary *d, const char *key, const SdfPath &prim, const TfToken &name,
                   RigExecRevisionLeafType type, VtValue fallback)
    { Field(d, key, prim.AppendProperty(name), type, std::move(fallback)); }
    void Attribute(Dictionary *d, const char *key, const SdfPath &prim, const char *name,
                   RigExecRevisionLeafType type, VtValue fallback)
    { Attribute(d, key, prim, TfToken(name), type, std::move(fallback)); }
    Dictionary UV(SdfPath path)
    {
        Dictionary d;
        if (path.IsEmpty()) return d;
        Field(&d, "values", path, RigExecRevisionLeafType::Vec2fArray, VtValue(VtVec2fArray()));
        Field(&d, "indices", path.GetPrimPath().AppendProperty(TfToken(path.GetName() + ":indices")),
              RigExecRevisionLeafType::IntArray, VtValue(VtIntArray()));
        const TfToken mode = interpolation(path);
        if (!mode.IsEmpty() && mode != "vertex" && mode != "varying" &&
            mode != "faceVarying") valid = false;
        d["faceVarying"] = VtValue(mode == "faceVarying");
        binding->externalStructure.push_back(path);
        return d;
    }
    Dictionary Geometry(const SdfPath &mesh, SdfPath uv)
    {
        Dictionary d;
        Attribute(&d, "points", mesh, _clothInputsTokens->points, RigExecRevisionLeafType::Vec3fArray, VtValue(VtVec3fArray()));
        Attribute(&d, "counts", mesh, _clothInputsTokens->faceVertexCounts, RigExecRevisionLeafType::IntArray, VtValue(VtIntArray()));
        Attribute(&d, "indices", mesh, _clothInputsTokens->faceVertexIndices, RigExecRevisionLeafType::IntArray, VtValue(VtIntArray()));
        Attribute(&d, "orientation", mesh, _clothInputsTokens->orientation, RigExecRevisionLeafType::Token, VtValue(_clothInputsTokens->rightHanded));
        if (uv.IsEmpty()) uv = mesh.AppendProperty(_clothInputsTokens->primvarsSt);
        d["uv"] = VtValue(UV(uv));
        return d;
    }
    Dictionary Binding(const SdfPath &prim, const SdfPath &mesh)
    {
        Dictionary d;
        if (prim.IsEmpty()) return d;
        if (type(prim) != "RigExecClothBinding") valid = false;
        Attribute(&d, "mode", prim, _clothInputsTokens->rigExecMode, RigExecRevisionLeafType::Token, VtValue(_clothInputsTokens->vertex));
        Attribute(&d, "indices", prim, _clothInputsTokens->rigExecIndices, RigExecRevisionLeafType::IntArray, VtValue(VtIntArray()));
        Attribute(&d, "coordinates", prim, _clothInputsTokens->rigExecCoordinates, RigExecRevisionLeafType::Vec3fArray, VtValue(VtVec3fArray()));
        d["geometry"] = VtValue(Geometry(mesh, Target(prim, _clothInputsTokens->rigExecUvSet)));
        return d;
    }
    Dictionary Map(const SdfPath &path, const SdfPath &cloth)
    {
        Dictionary d;
        if (path.IsEmpty()) return d;
        if (path.IsPropertyPath()) {
            Field(&d, "values", path, RigExecRevisionLeafType::FloatArray, VtValue(VtFloatArray()));
            return d;
        }
        if (type(path) != "RigExecClothMap") valid = false;
        SdfPath values = Target(path, _clothInputsTokens->rigExecValues);
        if (values.IsEmpty()) values = path.AppendProperty(_clothInputsTokens->inputsValues);
        Field(&d, "values", values, RigExecRevisionLeafType::FloatArray, VtValue(VtFloatArray()));
        SdfPath mesh = Target(path, _clothInputsTokens->rigExecSourceMesh);
        if (mesh.IsEmpty()) mesh = cloth;
        d["binding"] = VtValue(Binding(Target(path, _clothInputsTokens->rigExecBinding), mesh.GetPrimPath()));
        return d;
    }
};

/// A declaration over a live stage, made as the cloth mover makes it: a
/// relationship's own targets (RigExecRelationshipTargets; forwarded
/// targets are not followed, and the compiled structure does not track
/// them), a primvar's interpolation metadata, vertex when unauthored, and
/// a prim's type. Imaging guides use it to resolve what the mover resolves.
/// Reads the stage: owning thread only.
inline Declaration StageDeclaration(const UsdStagePtr &stage,
    std::vector<RigExecRevisionLeafKey> *inputs, RigExecRevisionBinding *binding)
{
    return Declaration{inputs, binding,
        [stage](const SdfPath &path, const TfToken &name) {
            const UsdPrim prim = stage->GetPrimAtPath(path);
            return prim ? RigExecRelationshipTargets(prim, name) : SdfPathVector();
        },
        [stage](const SdfPath &path) {
            TfToken result(_clothInputsTokens->vertex);
            if (const auto a = stage->GetAttributeAtPath(path)) a.GetMetadata(_clothInputsTokens->interpolation, &result);
            return result;
        },
        [stage](const SdfPath &path) {
            const auto prim = stage->GetPrimAtPath(path);
            return prim ? prim.GetTypeName() : TfToken();
        }};
}

/// The declared leaves' values as an evaluation samples them, without
/// interactive overrides (RigExecSampleRevisionLeaf). Owning thread only.
inline std::vector<VtValue> SampleLeaves(const UsdStagePtr &stage,
    const std::vector<RigExecRevisionLeafKey> &leaves, UsdTimeCode time)
{
    std::vector<VtValue> values;
    values.reserve(leaves.size());
    for (const auto &leaf : leaves)
        values.push_back(RigExecSampleRevisionLeaf(
            leaf, stage->GetAttributeAtPath(leaf.path), nullptr, time));
    return values;
}

inline Dictionary Declare(const SdfPath &mover, const SdfPath &cloth,
    const SdfPathVector &colliders, Declaration *declaration)
{
    auto &d = *declaration;
    Dictionary layout;
    layout["colliderCount"] = VtValue(int(colliders.size()));
    SdfPath rest = d.Target(mover, _clothInputsTokens->rigExecRestShape);
    if (!rest.IsEmpty()) {
        if (rest.IsPrimPath()) rest = rest.AppendProperty(_clothInputsTokens->points);
        (*d.inputs)[RigExecClothRest].path = rest;
    }
    layout["uv"] = VtValue(d.UV(d.Target(mover, _clothInputsTokens->rigExecUvSet)));
    std::vector<VtValue> profiles;
    for (const auto &prim : d.Targets(mover, _clothInputsTokens->rigExecConstraints)) {
        if (!prim.IsPrimPath() || d.type(prim) != "RigExecClothConstraintSet") d.valid = false;
        Dictionary p;
        d.Attribute(&p, "kind", prim, _clothInputsTokens->rigExecKind, RigExecRevisionLeafType::Token, VtValue(_clothInputsTokens->stretch));
        d.Attribute(&p, "mode", prim, _clothInputsTokens->rigExecMode, RigExecRevisionLeafType::Token, VtValue(_clothInputsTokens->soft));
        d.Attribute(&p, "twoSided", prim, _clothInputsTokens->rigExecTwoSided, RigExecRevisionLeafType::Bool, VtValue(true));
        const char *keys[] = {"minStrength", "maxStrength", "compliance", "warp", "weft",
            "normal", "thickness", "friction", "restLength"};
        const TfToken names[] = {_clothInputsTokens->inputsMinStrength, _clothInputsTokens->inputsMaxStrength,
            _clothInputsTokens->inputsCompliance, _clothInputsTokens->inputsWarp, _clothInputsTokens->inputsWeft,
            _clothInputsTokens->inputsNormal, _clothInputsTokens->inputsThickness,
            _clothInputsTokens->inputsFriction, _clothInputsTokens->inputsRestLength};
        const float defaults[] = {1, 1, 1e-4f, 1, 1, 1, .5f, .3f, -1};
        for (size_t k = 0; k < 9; ++k)
            d.Attribute(&p, keys[k], prim, names[k],
                        RigExecRevisionLeafType::Float, VtValue(defaults[k]));
        d.Attribute(&p, "curve", prim, _clothInputsTokens->rigExecForceCurve, RigExecRevisionLeafType::Vec2fArray,
                    VtValue(VtVec2fArray()));
        const SdfPath target = d.Target(prim, _clothInputsTokens->rigExecTargetMesh).GetPrimPath();
        int collider = -1;
        if (!target.IsEmpty() && target != cloth) {
            const auto found = std::find(colliders.begin(), colliders.end(), target);
            collider = found == colliders.end() ? -2 : int(found - colliders.begin());
        }
        p["targetCollider"] = VtValue(collider);
        p["source"] = VtValue(d.Binding(d.Target(prim, _clothInputsTokens->rigExecSourceBinding), cloth));
        p["target"] = VtValue(d.Binding(d.Target(prim, _clothInputsTokens->rigExecTargetBinding),
                                        target.IsEmpty() ? cloth : target));
        const TfToken mapNames[] = {_clothInputsTokens->rigExecMaskMap, _clothInputsTokens->rigExecMinStrengthMap,
            _clothInputsTokens->rigExecMaxStrengthMap, _clothInputsTokens->rigExecWarpMap,
            _clothInputsTokens->rigExecWeftMap, _clothInputsTokens->rigExecNormalMap,
            _clothInputsTokens->rigExecThicknessMap, _clothInputsTokens->rigExecFrictionMap};
        std::vector<VtValue> maps;
        for (size_t k = 0; k < ClothMapChannelCount; ++k)
            maps.emplace_back(d.Map(d.Target(prim, mapNames[k]), cloth));
        p["maps"] = VtValue(maps);
        profiles.emplace_back(p);
    }
    layout["profiles"] = VtValue(profiles);
    layout["valid"] = VtValue(d.valid);
    return layout;
}

inline bool ExpandUV(const std::vector<VtValue> &in, const Dictionary &layout,
    const std::vector<int32_t> &counts, const std::vector<int32_t> &indices,
    size_t points, bool left, std::vector<float> *out)
{
    const auto *values = Value<VtVec2fArray>(in, layout, "values");
    const auto *map = Value<VtIntArray>(in, layout, "indices");
    out->clear();
    if (!values || values->empty()) return layout.empty();
    const auto fv = layout.find("faceVarying");
    const bool corners = fv != layout.end() && fv->second.IsHolding<bool>() &&
        fv->second.UncheckedGet<bool>();
    const size_t expected = corners ? indices.size() : points;
    if (map && !map->empty()) { if (map->size() != expected) return false; }
    else if (values->size() != expected) return false;
    size_t offset = 0;
    for (int32_t count : counts) {
        if (count < 3 || offset + size_t(count) > indices.size()) return false;
        for (int32_t c = 1; c + 1 < count; ++c) {
            const int32_t order[] = {0, left ? c + 1 : c, left ? c : c + 1};
            for (int32_t j : order) {
                size_t at = corners ? offset + size_t(j) : size_t(indices[offset + size_t(j)]);
                if (at >= expected) return false;
                if (map && !map->empty()) {
                    if ((*map)[at] < 0) return false;
                    at = size_t((*map)[at]);
                }
                if (at >= values->size()) return false;
                out->push_back((*values)[at][0]); out->push_back((*values)[at][1]);
            }
        }
        offset += size_t(count);
    }
    return offset == indices.size();
}

inline bool Samples(const std::vector<VtValue> &in, const Dictionary &layout,
                     std::vector<RigExecClothSample> *samples, std::string *why)
{
    const auto fail = [&](const char *s) { if (why) *why = s; return false; };
    samples->clear();
    if (layout.empty()) return true;
    const auto *mode = Value<TfToken>(in, layout, "mode");
    const auto *indices = Value<VtIntArray>(in, layout, "indices");
    const auto *coordinates = Value<VtVec3fArray>(in, layout, "coordinates");
    const auto *geometry = Object(layout, "geometry");
    if (!mode || !indices || !coordinates || !geometry) return fail("missing cloth binding data");
    const auto *points = Value<VtVec3fArray>(in, *geometry, "points");
    const auto *counts = Value<VtIntArray>(in, *geometry, "counts");
    const auto *faces = Value<VtIntArray>(in, *geometry, "indices");
    const auto *orientation = Value<TfToken>(in, *geometry, "orientation");
    if (!points || !counts || !faces || !orientation) return fail("missing cloth binding mesh");
    if (*mode == "vertex") {
        if (indices->empty()) return fail("an authored cloth binding must not be empty");
        if (!coordinates->empty()) return fail("vertex bindings do not use coordinates");
        for (int32_t index : *indices) {
            if (index < 0 || size_t(index) >= points->size()) return fail("cloth vertex binding out of range");
            samples->push_back(RigExecClothVertex(size_t(index)));
        }
        return true;
    }
    std::vector<int32_t> triangles;
    const bool left = *orientation == "leftHanded";
    if (!cloth_detail::Triangulate(points->size(), counts->cdata(), counts->size(), faces->cdata(),
                                   faces->size(), left, &triangles)) return fail("invalid binding topology");
    if (*mode == "barycentric") {
        if (indices->empty()) return fail("an authored cloth binding must not be empty");
        if (indices->size() != coordinates->size()) return fail("cloth barycentric binding size mismatch");
        for (size_t k = 0; k < indices->size(); ++k) {
            const int32_t t = (*indices)[k];
            if (t < 0 || size_t(t) >= triangles.size() / 3) return fail("cloth triangle binding out of range");
            RigExecClothSample sample;
            for (size_t q = 0; q < 3; ++q) {
                sample.vertices[q] = triangles[3 * size_t(t) + q];
                sample.weights[q] = (*coordinates)[k][int(q)];
            }
            samples->push_back(sample);
        }
        return true;
    }
    if (*mode != "uv") return fail("unknown cloth binding mode");
    if (coordinates->empty()) return fail("an authored UV binding must not be empty");
    std::vector<float> uv;
    const auto *uvLayout = Object(*geometry, "uv");
    if (!uvLayout || !ExpandUV(in, *uvLayout, {counts->begin(), counts->end()},
        {faces->begin(), faces->end()}, points->size(), left, &uv) || uv.empty())
        return fail("UV binding requires usable mesh UVs");
    if (!indices->empty() && indices->size() != coordinates->size())
        return fail("UV binding triangle hints must match coordinates");
    for (size_t k = 0; k < coordinates->size(); ++k) {
        const GfVec3f &p = (*coordinates)[k];
        if (!std::isfinite(p[0]) || !std::isfinite(p[1])) return fail("non-finite UV binding");
        RigExecClothSample found;
        bool have = false;
        const int32_t hint = indices->empty() ? -1 : (*indices)[k];
        if (hint < -1 || (hint >= 0 && size_t(hint) >= triangles.size() / 3))
            return fail("UV binding triangle hint out of range");
        for (size_t t = 0; t < triangles.size() / 3; ++t) {
            if (hint >= 0 && t != size_t(hint)) continue;
            const float *v = uv.data() + 6 * t;
            const float x1 = v[2] - v[0], y1 = v[3] - v[1];
            const float x2 = v[4] - v[0], y2 = v[5] - v[1];
            const float det = x1 * y2 - y1 * x2;
            if (std::abs(det) <= 1e-12f) continue;
            const float x = p[0] - v[0], y = p[1] - v[1];
            float b = (x * y2 - y * x2) / det, c = (x1 * y - y1 * x) / det;
            float a = 1 - b - c;
            if (a < -1e-6f || b < -1e-6f || c < -1e-6f) continue;
            RigExecClothSample sample;
            sample.vertices = {{triangles[3*t], triangles[3*t+1], triangles[3*t+2]}};
            sample.weights = {{std::max(0.0f, a), std::max(0.0f, b), std::max(0.0f, c)}};
            float sum = sample.weights[0] + sample.weights[1] + sample.weights[2];
            for (float &w : sample.weights) w /= sum;
            if (have) {
                // Shared edges are one binding if both triangles resolve
                // to the same weighted vertex set. Overlapping islands need
                // an explicit triangle hint; never silently pick one.
                std::map<int32_t, float> first, second;
                for (size_t q = 0; q < 3; ++q) {
                    if (found.weights[q] > 1e-6f) first[found.vertices[q]] += found.weights[q];
                    if (sample.weights[q] > 1e-6f) second[sample.vertices[q]] += sample.weights[q];
                }
                if (first.size() != second.size()) return fail("ambiguous UV binding; supply a triangle hint");
                for (const auto &entry : first)
                    if (!second.count(entry.first) || std::abs(second[entry.first] - entry.second) > 1e-5f)
                        return fail("ambiguous UV binding; supply a triangle hint");
            } else { found = sample; have = true; }
        }
        if (!have) return fail("UV binding lies outside the mesh UVs");
        samples->push_back(found);
    }
    return true;
}

inline bool ReadMap(const std::vector<VtValue> &in, const Dictionary &layout,
                     size_t n, std::vector<float> *map, std::string *why)
{
    map->clear();
    if (layout.empty()) return true;
    const auto *values = Value<VtFloatArray>(in, layout, "values");
    if (!values || values->empty()) { if (why) *why = "cloth map has no values"; return false; }
    const auto *binding = Object(layout, "binding");
    if (!binding || binding->empty()) {
        if (values->size() == 1) map->assign(n, (*values)[0]);
        else map->assign(values->begin(), values->end());
        return map->size() == n;
    }
    std::vector<RigExecClothSample> samples;
    if (!Samples(in, *binding, &samples, why) || samples.size() != n) return false;
    for (const auto &sample : samples) {
        float value = 0, sum = 0;
        for (size_t q = 0; q < 3; ++q) {
            if (sample.vertices[q] < 0 || size_t(sample.vertices[q]) >= values->size() ||
                !std::isfinite(sample.weights[q]) || sample.weights[q] < 0) return false;
            value += sample.weights[q] * (*values)[size_t(sample.vertices[q])];
            sum += sample.weights[q];
        }
        if (std::abs(sum - 1) > 1e-5f) return false;
        map->push_back(value);
    }
    return true;
}

inline void Region(const std::vector<RigExecClothSample> &samples, size_t n,
                    std::vector<float> *mask)
{
    if (samples.empty()) return;
    mask->assign(n, 0);
    for (const auto &s : samples)
        for (size_t k = 0; k < 3; ++k)
            if (s.vertices[k] >= 0 && size_t(s.vertices[k]) < n)
                (*mask)[size_t(s.vertices[k])] = std::max((*mask)[size_t(s.vertices[k])], s.weights[k]);
}

inline bool Assemble(const std::vector<VtValue> &in, const Dictionary &layout,
    const RigExecClothCompiledData &compiledRig, RigExecClothModel *model, std::string *why)
{
    const auto valid = layout.find("valid");
    if (valid == layout.end() || !valid->second.IsHolding<bool>() || !valid->second.UncheckedGet<bool>()) {
        if (why) *why = "invalid cloth relationship cardinality or target type";
        return false;
    }
    const auto *uv = Object(layout, "uv");
    if (uv && !ExpandUV(in, *uv, compiledRig.counts, compiledRig.indices, compiledRig.pointCount,
                        compiledRig.leftHanded, &model->triangleUV)) return false;
    const auto found = layout.find("profiles");
    if (found == layout.end() || !found->second.IsHolding<std::vector<VtValue>>()) return false;
    for (const auto &entry : found->second.UncheckedGet<std::vector<VtValue>>()) {
        const auto &d = entry.UncheckedGet<Dictionary>();
        const auto *kind = Value<TfToken>(in, d, "kind");
        const auto *mode = Value<TfToken>(in, d, "mode");
        if (!kind || !mode) return false;
        const char *kinds[] = {"stretch", "bend", "damping", "pin", "attachment", "collision", "selfCollision", "tether"};
        size_t k = 0;
        while (k < 8 && kind->GetString() != kinds[k]) ++k;
        if (k == 8 || (*mode != "hard" && *mode != "soft")) return false;
        RigExecClothProfile p;
        p.kind = RigExecClothProfileKind(k); p.hard = *mode == "hard";
        const auto *twoSided = Value<bool>(in, d, "twoSided");
        if (!twoSided) return false;
        p.twoSided = *twoSided;
        p.targetCollider = Index(d, "targetCollider");
        const char *keys[] = {"minStrength", "maxStrength", "compliance", "warp", "weft",
            "normal", "thickness", "friction", "restLength"};
        float *values[] = {&p.minStrength, &p.maxStrength, &p.compliance, &p.warp, &p.weft,
            &p.normal, &p.thickness, &p.friction, &p.restLength};
        for (size_t j = 0; j < 9; ++j) {
            const auto *v = Value<float>(in, d, keys[j]);
            if (!v) return false;
            *values[j] = *v;
        }
        const auto *curve = Value<VtVec2fArray>(in, d, "curve");
        if (!curve) return false;
        for (const auto &v : *curve) { p.forceCurve.push_back(v[0]); p.forceCurve.push_back(v[1]); }
        const auto *source = Object(d, "source");
        const auto *target = Object(d, "target");
        if (!source || !target || !Samples(in, *source, &p.source, why) ||
            !Samples(in, *target, &p.target, why)) return false;
        Region(p.source, compiledRig.pointCount, &p.sourceMask);
        if (p.targetCollider < -1 || (p.targetCollider >= 0 && size_t(p.targetCollider) >= compiledRig.colliders.size())) return false;
        Region(p.target, p.targetCollider < 0 ? compiledRig.pointCount : compiledRig.colliders[size_t(p.targetCollider)].pointCount,
               &p.targetMask);
        const auto mapsIt = d.find("maps");
        if (mapsIt == d.end() || !mapsIt->second.IsHolding<std::vector<VtValue>>()) return false;
        const auto &maps = mapsIt->second.UncheckedGet<std::vector<VtValue>>();
        if (maps.size() != ClothMapChannelCount) return false;
        for (size_t j = 0; j < ClothMapChannelCount; ++j)
            if (!ReadMap(in, maps[j].UncheckedGet<Dictionary>(), compiledRig.pointCount, &p.maps[j], why)) return false;
        model->profiles.push_back(p);
    }
    std::vector<size_t> colliderCounts;
    for (const auto &c : compiledRig.colliders) colliderCounts.push_back(c.pointCount);
    size_t triangles = 0;
    for (int32_t count : compiledRig.counts) if (count >= 3) triangles += size_t(count - 2);
    return RigExecClothValidateModel(*model, compiledRig.pointCount, triangles, colliderCounts, why);
}

}  // namespace cloth_inputs
#endif
