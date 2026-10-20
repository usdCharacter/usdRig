// RigExecPatchMover: NURBS patch surface attachment from phased UV points,
// mixed toward weighted UV shapes, with contact pairs kept from crossing.
#include "moverRegistry.h"
#include "externalPlayback.h"
#include "../parallel.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecScene/sceneDescriptors.h"

#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/usdGeom/pointBased.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

// Declared inputs: the patch's ten, the neutral UV points, one points array
// per rigExec:uvShapes target, then the shape weights -- one per shape, read
// from the sources inputs:uvWeights connects to, in order, or, with no
// connection, the authored array itself -- and last the two contact index
// lists, rigExec:uvContactLower and rigExec:uvContactUpper.
enum Input : size_t {
    PatchPoints = 0,
    PatchPointWeights,
    PatchUVertexCount,
    PatchVVertexCount,
    PatchUOrder,
    PatchVOrder,
    PatchUKnots,
    PatchVKnots,
    PatchUForm,
    PatchVForm,
    UvPoints,
    UvShapes
};

// Interned once at load: the oracle asks per frame, and interning takes
// the token registry's lock.
const TfToken _uvWeights("inputs:uvWeights");
const TfToken _uvShapes("rigExec:uvShapes");
const TfToken _uvContactLower("rigExec:uvContactLower");
const TfToken _uvContactUpper("rigExec:uvContactUpper");

// How many shapes the declared inputs hold, and whether their weights are
// connected sources (one Dial each) or the authored array.
struct Layout {
    size_t shapes = 0;
    bool connected = false;
};

size_t
WeightsBegin(const Layout &layout)
{
    return UvShapes + layout.shapes;
}

size_t
ContactsBegin(const Layout &layout)
{
    return WeightsBegin(layout) + (layout.connected ? layout.shapes : 1);
}

size_t
InputCount(const Layout &layout)
{
    return ContactsBegin(layout) + 2;
}

Layout
LayoutOf(const std::vector<RigExecRevisionLeafKey> &keys)
{
    Layout layout;
    size_t k = UvShapes;
    while (k < keys.size() && keys[k].type == RigExecRevisionLeafType::Vec3fArray) {
        ++layout.shapes;
        ++k;
    }
    layout.connected = k < keys.size() &&
                       keys[k].type == RigExecRevisionLeafType::Dial;
    return layout;
}

std::vector<RigExecRevisionLeafKey>
DeclaredInputs(const SdfPath &mover, const SdfPath &patchTarget,
               const SdfPath &uvTarget, const SdfPathVector &shapeTargets,
               const SdfPathVector &weightSources)
{
    using Type = RigExecRevisionLeafType;
    const auto key = [](const SdfPath &path, Type type, VtValue fallback) {
        return RigExecRevisionLeafKey{
            path, type, RigExecRevisionLeafTime::AtTime,
            RigExecRevisionLeafFlavour::Resolved, std::move(fallback)};
    };
    std::vector<RigExecRevisionLeafKey> inputs;
    inputs.reserve(UvShapes + 2 * shapeTargets.size() + 3);
    if (patchTarget.IsEmpty()) {
        const SdfPath unbound = mover.AppendProperty(TfToken("rigExec:patch"));
        const Type patchTypes[] = {
            Type::Vec3fArray, Type::DoubleArray, Type::Int, Type::Int,
            Type::Int, Type::Int, Type::DoubleArray, Type::DoubleArray,
            Type::Token, Type::Token};
        for (Type type : patchTypes) {
            inputs.push_back({unbound, type, RigExecRevisionLeafTime::AtTime,
                              RigExecRevisionLeafFlavour::Raw, VtValue()});
        }
    } else {
        const SdfPath prim = patchTarget.GetPrimPath();
        inputs.push_back(key(prim.AppendProperty(TfToken("points")),
                             Type::Vec3fArray, VtValue(VtVec3fArray())));
        inputs.push_back(key(prim.AppendProperty(TfToken("pointWeights")),
                             Type::DoubleArray, VtValue(VtDoubleArray())));
        inputs.push_back(key(prim.AppendProperty(TfToken("uVertexCount")),
                             Type::Int, VtValue(0)));
        inputs.push_back(key(prim.AppendProperty(TfToken("vVertexCount")),
                             Type::Int, VtValue(0)));
        inputs.push_back(key(prim.AppendProperty(TfToken("uOrder")),
                             Type::Int, VtValue(0)));
        inputs.push_back(key(prim.AppendProperty(TfToken("vOrder")),
                             Type::Int, VtValue(0)));
        inputs.push_back(key(prim.AppendProperty(TfToken("uKnots")),
                             Type::DoubleArray, VtValue(VtDoubleArray())));
        inputs.push_back(key(prim.AppendProperty(TfToken("vKnots")),
                             Type::DoubleArray, VtValue(VtDoubleArray())));
        inputs.push_back(key(prim.AppendProperty(TfToken("uForm")),
                             Type::Token, VtValue(TfToken("open"))));
        inputs.push_back(key(prim.AppendProperty(TfToken("vForm")),
                             Type::Token, VtValue(TfToken("open"))));
    }
    if (uvTarget.IsEmpty()) {
        const SdfPath unbound =
            mover.AppendProperty(TfToken("rigExec:uvPoints"));
        inputs.push_back({unbound, Type::Vec3fArray,
                          RigExecRevisionLeafTime::AtTime,
                          RigExecRevisionLeafFlavour::Raw, VtValue()});
    } else {
        inputs.push_back(key(RigExecPointsOf(uvTarget), Type::Vec3fArray,
                             VtValue(VtVec3fArray())));
    }
    for (const SdfPath &shape : shapeTargets) {
        inputs.push_back(key(RigExecPointsOf(shape), Type::Vec3fArray,
                             VtValue(VtVec3fArray())));
    }
    // A Dial reads a float source as a float and widens it, any other as a
    // double: avars come in both precisions.
    if (!weightSources.empty()) {
        for (const SdfPath &source : weightSources) {
            inputs.push_back(key(source, Type::Dial, VtValue(0.0)));
        }
    } else {
        inputs.push_back(key(mover.AppendProperty(_uvWeights),
                             Type::FloatArray, VtValue(VtFloatArray())));
    }
    inputs.push_back(key(mover.AppendProperty(_uvContactLower),
                         Type::IntArray, VtValue(VtIntArray())));
    inputs.push_back(key(mover.AppendProperty(_uvContactUpper),
                         Type::IntArray, VtValue(VtIntArray())));
    return inputs;
}

SdfPathVector
WeightSources(const UsdPrim &mover)
{
    SdfPathVector sources;
    if (const UsdAttribute weights = mover.GetAttribute(_uvWeights)) {
        weights.GetConnections(&sources);
    }
    return sources;
}

template <class T>
bool As(const VtValue &value, T *out)
{
    if (!value.IsHolding<T>()) {
        return false;
    }
    *out = value.UncheckedGet<T>();
    return true;
}

// A weight as a double, from a float or a double value.
bool Scalar(const VtValue &value, double *out)
{
    if (value.IsHolding<double>()) {
        *out = value.UncheckedGet<double>();
        return true;
    }
    if (value.IsHolding<float>()) {
        *out = double(value.UncheckedGet<float>());
        return true;
    }
    return false;
}

template <class T>
const T *Get(const VtDictionary &d, const char *key)
{
    const auto found = d.find(key);
    return found != d.end() && found->second.IsHolding<T>()
               ? &found->second.UncheckedGet<T>()
               : nullptr;
}

bool Text(const VtValue &value, const char **out)
{
    if (value.IsHolding<TfToken>()) {
        *out = value.UncheckedGet<TfToken>().GetText();
        return true;
    }
    if (value.IsHolding<std::string>()) {
        *out = value.UncheckedGet<std::string>().c_str();
        return true;
    }
    return false;
}

bool Is(const char *token, const char *text)
{
    return std::strcmp(token, text) == 0;
}

bool ValidForm(const char *form, bool *periodic)
{
    if (!form) {
        return false;
    }
    if (Is(form, "open")) {
        *periodic = false;
        return true;
    }
    if (Is(form, "closed") || Is(form, "periodic")) {
        *periodic = true;
        return true;
    }
    return false;
}

// The neutral UV points mixed toward the shapes:
// uv = neutral + sum_k w_k (shape_k - neutral), in UV space, accumulated in
// double. Only the XY components move. False when a shape's point count
// differs from the neutral's or a weight is not finite.
bool MixShapes(const std::vector<const GfVec3f *> &shapes,
               const std::vector<double> &weights, VtVec3fArray *uvPoints)
{
    for (const double weight : weights) {
        if (!std::isfinite(weight)) {
            return false;
        }
    }
    const VtVec3fArray neutral = *uvPoints;
    GfVec3f *const mixed = uvPoints->data();
    // Each point mixes independently, so the loop may be split.
    RigExecRunRanges(RigExecGeometryRanges(), neutral.size(), 1024,
                     [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            double u = double(neutral[i][0]), v = double(neutral[i][1]);
            for (size_t k = 0; k < shapes.size(); ++k) {
                if (weights[k] == 0.0) {
                    continue;
                }
                u += weights[k] *
                     (double(shapes[k][i][0]) - double(neutral[i][0]));
                v += weights[k] *
                     (double(shapes[k][i][1]) - double(neutral[i][1]));
            }
            mixed[i] = GfVec3f(float(u), float(v), neutral[i][2]);
        }
    });
    return true;
}

// Why the contact lists are unusable for pointCount UV points, or empty
// when they are usable: as long as each other, every index in range, and
// no point named twice, so pairs do not interact.
std::string ContactError(const VtIntArray &lower, const VtIntArray &upper,
                         size_t pointCount)
{
    if (lower.size() != upper.size()) {
        return "rigExec:uvContactLower holds " + std::to_string(lower.size()) +
               " indices; rigExec:uvContactUpper holds " +
               std::to_string(upper.size());
    }
    std::vector<int> named;
    named.reserve(2 * lower.size());
    for (size_t k = 0; k < lower.size(); ++k) {
        for (const int index : {lower[k], upper[k]}) {
            if (index < 0 || size_t(index) >= pointCount) {
                return "UV contact " + std::to_string(k) + " names point " +
                       std::to_string(index) + "; rigExec:uvPoints has " +
                       std::to_string(pointCount);
            }
            named.push_back(index);
        }
    }
    std::sort(named.begin(), named.end());
    const auto twice = std::adjacent_find(named.begin(), named.end());
    if (twice != named.end()) {
        return "UV contacts name point " + std::to_string(*twice) +
               " more than once";
    }
    return std::string();
}

// Contacts, after the shapes mix: point lower[k] never stands above point
// upper[k] in v. A pair that would cross meets at the midpoint of its two
// v values, so shapes that close past contact press together instead of
// passing through. Only v moves; a pair meant to keep two curves apart
// shares its u (one column of each curve), and then the curves cannot
// cross between columns either. False when ContactError reports.
bool TouchContacts(const VtIntArray &lower, const VtIntArray &upper,
                   VtVec3fArray *uvPoints)
{
    if (lower.empty() && upper.empty()) {
        return true;
    }
    if (!ContactError(lower, upper, uvPoints->size()).empty()) {
        return false;
    }
    GfVec3f *uv = uvPoints->data();
    for (size_t k = 0; k < lower.size(); ++k) {
        GfVec3f &below = uv[lower[k]];
        GfVec3f &above = uv[upper[k]];
        if (below[1] > above[1]) {
            const float mid =
                float(0.5 * (double(below[1]) + double(above[1])));
            below[1] = mid;
            above[1] = mid;
        }
    }
    return true;
}

void Bind(const RigExecMoverBindContext &ctx)
{
    auto &b = *ctx.binding;
    const auto patches =
        RigExecRelationshipTargets(ctx.moverPrim, "rigExec:patch");
    if (patches.size() == 1) {
        const SdfPath patchPoints =
            patches[0].GetPrimPath().AppendProperty(TfToken("points"));
        const RigExecReadPhase phase =
            RigExecPhaseForInput(ctx.moverPrim, "rigExec:patch");
        if (!phase.IsBase()) {
            b.phases[patchPoints] = phase;
        }
    }
    const auto uvs =
        RigExecRelationshipTargets(ctx.moverPrim, "rigExec:uvPoints");
    if (uvs.size() == 1) {
        const SdfPath uvPoints = RigExecPointsOf(uvs[0]);
        const RigExecReadPhase phase =
            RigExecPhaseForInput(ctx.moverPrim, "rigExec:uvPoints");
        if (!phase.IsBase()) {
            b.phases[uvPoints] = phase;
        }
    }
    const RigExecReadPhase shapePhase =
        RigExecPhaseForInput(ctx.moverPrim, "rigExec:uvShapes");
    if (!shapePhase.IsBase()) {
        for (const SdfPath &shape :
             RigExecRelationshipTargets(ctx.moverPrim, "rigExec:uvShapes")) {
            b.phases[RigExecPointsOf(shape)] = shapePhase;
        }
    }
    // Which sources the weights read, and in what order, is wiring: a
    // connection edit rebuilds the program.
    b.externalStructure.push_back(
        ctx.moverPrim.GetPath().AppendProperty(_uvWeights));
}

void DeclareInputs(const RigExecMoverBindContext &ctx,
                   std::vector<RigExecRevisionLeafKey> *inputs)
{
    const auto patches =
        RigExecRelationshipTargets(ctx.moverPrim, "rigExec:patch");
    const auto uvs =
        RigExecRelationshipTargets(ctx.moverPrim, "rigExec:uvPoints");
    *inputs = DeclaredInputs(
        ctx.moverPrim.GetPath(), patches.size() == 1 ? patches[0] : SdfPath(),
        uvs.size() == 1 ? uvs[0] : SdfPath(),
        RigExecRelationshipTargets(ctx.moverPrim, "rigExec:uvShapes"),
        WeightSources(ctx.moverPrim));
}

bool CompileScene(const RigExecSceneDescriptors &scene, const SdfPath &mover,
                  const SdfPath &, RigExecRevisionBinding *binding,
                  std::string *error)
{
    if (!binding) {
        return false;
    }
    const auto relationship = [&](const char *name) {
        const auto found =
            scene.relationships.find(mover.AppendProperty(TfToken(name)));
        return found == scene.relationships.end() ? nullptr : &found->second;
    };
    const auto phaseOf = [&](const RigExecSceneRelationshipDescriptor &rel,
                             RigExecReadPhase *phase) {
        return RigExecParseReadPhase(rel.readPhase.GetString(), phase, error);
    };
    SdfPath patchTarget, uvTarget;
    SdfPathVector shapeTargets, weightSources;
    if (const auto *rel = relationship("rigExec:patch");
        rel && rel->fact.targets.size() == 1) {
        patchTarget = rel->fact.targets[0];
        RigExecReadPhase phase;
        if (!phaseOf(*rel, &phase)) {
            return false;
        }
        if (!phase.IsBase()) {
            binding->phases[patchTarget.GetPrimPath().AppendProperty(
                TfToken("points"))] = phase;
        }
    }
    if (const auto *rel = relationship("rigExec:uvPoints");
        rel && rel->fact.targets.size() == 1) {
        uvTarget = rel->fact.targets[0];
        RigExecReadPhase phase;
        if (!phaseOf(*rel, &phase)) {
            return false;
        }
        if (!phase.IsBase()) {
            binding->phases[RigExecPointsOf(uvTarget)] = phase;
        }
    }
    if (const auto *rel = relationship("rigExec:uvShapes")) {
        shapeTargets = rel->fact.targets;
        RigExecReadPhase phase;
        if (!phaseOf(*rel, &phase)) {
            return false;
        }
        if (!phase.IsBase()) {
            for (const SdfPath &shape : shapeTargets) {
                binding->phases[RigExecPointsOf(shape)] = phase;
            }
        }
    }
    const SdfPath weights = mover.AppendProperty(_uvWeights);
    const auto attribute = scene.attributes.find(weights);
    if (attribute != scene.attributes.end()) {
        weightSources = attribute->second.fact.connections;
    }
    binding->externalStructure.push_back(weights);
    binding->externalInputs = DeclaredInputs(mover, patchTarget, uvTarget,
                                             shapeTargets, weightSources);
    return true;
}

bool Payload(const std::vector<VtValue> &in, const Layout &layout,
             const RigExecExternalProviderValues &v, VtValue *out)
{
    if (in.size() != InputCount(layout)) {
        return false;
    }
    VtVec3fArray patchPoints, uvPoints;
    VtDoubleArray pointWeights, uKnots, vKnots;
    int uVertexCount = 0, vVertexCount = 0, uOrder = 0, vOrder = 0;
    const char *uForm = nullptr, *vForm = nullptr;
    if (!As(in[PatchPoints], &patchPoints) ||
        !As(in[PatchPointWeights], &pointWeights) ||
        !As(in[PatchUVertexCount], &uVertexCount) ||
        !As(in[PatchVVertexCount], &vVertexCount) ||
        !As(in[PatchUOrder], &uOrder) ||
        !As(in[PatchVOrder], &vOrder) ||
        !As(in[PatchUKnots], &uKnots) ||
        !As(in[PatchVKnots], &vKnots) ||
        !Text(in[PatchUForm], &uForm) ||
        !Text(in[PatchVForm], &vForm) ||
        !As(in[UvPoints], &uvPoints)) {
        return false;
    }
    bool uPeriodic = false, vPeriodic = false;
    if (!ValidForm(uForm, &uPeriodic) || !ValidForm(vForm, &vPeriodic)) {
        return false;
    }
    RigExecNurbsPatch patch;
    patch.points = patchPoints.cdata();
    patch.pointCount = patchPoints.size();
    patch.pointWeights = pointWeights.cdata();
    patch.pointWeightCount = pointWeights.size();
    patch.uVertexCount = uVertexCount;
    patch.vVertexCount = vVertexCount;
    patch.uOrder = uOrder;
    patch.vOrder = vOrder;
    patch.uKnots = uKnots.cdata();
    patch.uKnotCount = uKnots.size();
    patch.vKnots = vKnots.cdata();
    patch.vKnotCount = vKnots.size();
    patch.uPeriodic = uPeriodic;
    patch.vPeriodic = vPeriodic;
    if (!patch.IsValid()) {
        return false;
    }
    const size_t targetCount = v.basePoints.size();
    if (targetCount > 0 && uvPoints.size() != targetCount &&
        uvPoints.size() != 1) {
        return false;
    }
    // The weights: one per shape. An unconnected, empty array leaves every
    // shape at zero.
    std::vector<double> weights(layout.shapes, 0.0);
    const size_t weightsBegin = WeightsBegin(layout);
    if (layout.connected) {
        for (size_t k = 0; k < layout.shapes; ++k) {
            if (!Scalar(in[weightsBegin + k], &weights[k])) {
                return false;
            }
        }
    } else {
        VtFloatArray authored;
        if (!As(in[weightsBegin], &authored) ||
            (!authored.empty() && authored.size() != layout.shapes)) {
            return false;
        }
        for (size_t k = 0; k < authored.size(); ++k) {
            weights[k] = double(authored[k]);
        }
    }
    std::vector<VtVec3fArray> shapeArrays(layout.shapes);
    std::vector<const GfVec3f *> shapes(layout.shapes);
    for (size_t k = 0; k < layout.shapes; ++k) {
        if (!As(in[UvShapes + k], &shapeArrays[k]) ||
            shapeArrays[k].size() != uvPoints.size()) {
            return false;
        }
        shapes[k] = shapeArrays[k].cdata();
    }
    VtIntArray contactLower, contactUpper;
    const size_t contactsBegin = ContactsBegin(layout);
    if (!As(in[contactsBegin], &contactLower) ||
        !As(in[contactsBegin + 1], &contactUpper) ||
        !MixShapes(shapes, weights, &uvPoints) ||
        !TouchContacts(contactLower, contactUpper, &uvPoints)) {
        return false;
    }
    for (const GfVec3f &uv : uvPoints) {
        if (!std::isfinite(uv[0]) || !std::isfinite(uv[1])) {
            return false;
        }
    }
    VtDictionary d;
    d["patchPoints"] = VtValue(patchPoints);
    d["pointWeights"] = VtValue(pointWeights);
    d["uVertexCount"] = VtValue(uVertexCount);
    d["vVertexCount"] = VtValue(vVertexCount);
    d["uOrder"] = VtValue(uOrder);
    d["vOrder"] = VtValue(vOrder);
    d["uKnots"] = VtValue(uKnots);
    d["vKnots"] = VtValue(vKnots);
    d["uPeriodic"] = VtValue(uPeriodic);
    d["vPeriodic"] = VtValue(vPeriodic);
    d["uvPoints"] = VtValue(uvPoints);
    *out = VtValue(d);
    return true;
}

bool Assemble(const RigExecExternalInputContext &ctx, VtValue *out)
{
    return Payload(ctx.inputs, LayoutOf(ctx.binding.externalInputs),
                   ctx.values, out);
}

bool Apply(const VtValue &data, std::vector<GfVec3f> *output)
{
    if (!output || !data.IsHolding<VtDictionary>()) {
        return false;
    }
    const auto &d = data.UncheckedGet<VtDictionary>();
    const auto *patchPoints = Get<VtVec3fArray>(d, "patchPoints");
    const auto *pointWeights = Get<VtDoubleArray>(d, "pointWeights");
    const auto *uVertexCount = Get<int>(d, "uVertexCount");
    const auto *vVertexCount = Get<int>(d, "vVertexCount");
    const auto *uOrder = Get<int>(d, "uOrder");
    const auto *vOrder = Get<int>(d, "vOrder");
    const auto *uKnots = Get<VtDoubleArray>(d, "uKnots");
    const auto *vKnots = Get<VtDoubleArray>(d, "vKnots");
    const auto *uPeriodic = Get<bool>(d, "uPeriodic");
    const auto *vPeriodic = Get<bool>(d, "vPeriodic");
    const auto *uvPoints = Get<VtVec3fArray>(d, "uvPoints");
    if (!patchPoints || !pointWeights || !uVertexCount || !vVertexCount ||
        !uOrder || !vOrder || !uKnots || !vKnots || !uPeriodic ||
        !vPeriodic || !uvPoints) {
        return false;
    }
    RigExecNurbsPatch patch;
    patch.points = patchPoints->cdata();
    patch.pointCount = patchPoints->size();
    patch.pointWeights = pointWeights->cdata();
    patch.pointWeightCount = pointWeights->size();
    patch.uVertexCount = *uVertexCount;
    patch.vVertexCount = *vVertexCount;
    patch.uOrder = *uOrder;
    patch.vOrder = *vOrder;
    patch.uKnots = uKnots->cdata();
    patch.uKnotCount = uKnots->size();
    patch.vKnots = vKnots->cdata();
    patch.vKnotCount = vKnots->size();
    patch.uPeriodic = *uPeriodic;
    patch.vPeriodic = *vPeriodic;
    return RigExecApplyPatchMover(
        output, patch, uvPoints->cdata(), uvPoints->size(),
        RigExecGeometryRanges());
}

// A weight source's value through the resolved scene, as a double.
double OracleWeight(const RigExecMoverOracleContext &ctx, const SdfPath &path)
{
    const RigExecOracleAttribute source = ctx.stage.GetAttributeAtPath(path);
    if (!source) {
        return 0.0;
    }
    if (source.GetTypeName() == SdfValueTypeNames->Float) {
        float value = 0.0f;
        ctx.resolved.GetAttribute(source, ctx.time, &value);
        return double(value);
    }
    double value = 0.0;
    ctx.resolved.GetAttribute(source, ctx.time, &value);
    return value;
}

RigExecOracleResult Oracle(const RigExecMoverOracleContext &ctx)
{
    SdfPathVector patches, uvs, shapeTargets;
    if (RigExecOracleRelationship rel =
            ctx.prim.GetRelationship(TfToken("rigExec:patch"))) {
        rel.GetTargets(&patches);
    }
    if (RigExecOracleRelationship rel =
            ctx.prim.GetRelationship(TfToken("rigExec:uvPoints"))) {
        rel.GetTargets(&uvs);
    }
    if (RigExecOracleRelationship rel =
            ctx.prim.GetRelationship(_uvShapes)) {
        rel.GetTargets(&shapeTargets);
    }
    if (patches.size() != 1 || uvs.size() != 1) {
        return RigExecOracleResult::PassThrough;
    }
    const SdfPath patchPrimPath = patches[0].GetPrimPath();
    const SdfPath patchPointsPath =
        patchPrimPath.AppendProperty(TfToken("points"));
    const SdfPath uvPointsPath = RigExecPointsOf(uvs[0]);

    VtVec3fArray patchPoints, uvPoints;
    RigExecReadPhasedPoints(ctx, "rigExec:patch", patchPointsPath, &patchPoints);
    RigExecReadPhasedPoints(ctx, "rigExec:uvPoints", uvPointsPath, &uvPoints);

    // The shapes and their weights, read independently of the packet.
    bool shapesValid = true;
    std::vector<VtVec3fArray> shapeArrays(shapeTargets.size());
    std::vector<const GfVec3f *> shapes(shapeTargets.size());
    std::vector<double> weights(shapeTargets.size(), 0.0);
    for (size_t k = 0; k < shapeTargets.size(); ++k) {
        RigExecReadPhasedPoints(ctx, "rigExec:uvShapes",
                                RigExecPointsOf(shapeTargets[k]),
                                &shapeArrays[k]);
        shapesValid = shapesValid && shapeArrays[k].size() == uvPoints.size();
        shapes[k] = shapeArrays[k].cdata();
    }
    const RigExecOracleAttribute weightsAttr =
        ctx.prim.GetAttribute(_uvWeights);
    if (weightsAttr && !weightsAttr.connections.empty()) {
        shapesValid = shapesValid &&
                      weightsAttr.connections.size() == shapeTargets.size();
        for (size_t k = 0;
             k < weightsAttr.connections.size() && k < weights.size(); ++k) {
            weights[k] = OracleWeight(ctx, weightsAttr.connections[k]);
        }
    } else if (weightsAttr) {
        VtFloatArray authored;
        ctx.resolved.GetAttribute(weightsAttr, ctx.time, &authored);
        shapesValid = shapesValid && (authored.empty() ||
                                      authored.size() == shapeTargets.size());
        for (size_t k = 0; k < authored.size() && k < weights.size(); ++k) {
            weights[k] = double(authored[k]);
        }
    }
    const auto readContacts = [&](const TfToken &name) {
        VtIntArray indices;
        if (const RigExecOracleAttribute a = ctx.prim.GetAttribute(name)) {
            if (!ctx.resolved.GetAttribute(a, ctx.time, &indices)) {
                a.Get(&indices, ctx.time);
            }
        }
        return indices;
    };
    const VtIntArray contactLower = readContacts(_uvContactLower);
    const VtIntArray contactUpper = readContacts(_uvContactUpper);

    const RigExecOraclePrim patchPrim = ctx.stage.GetPrimAtPath(patchPrimPath);
    if (!patchPrim) {
        return RigExecOracleResult::PassThrough;
    }
    const auto read = [&](const char *name, auto fallback) {
        const auto a = patchPrim.GetAttribute(TfToken(name));
        if (a) {
            if (!ctx.resolved.GetAttribute(a, ctx.time, &fallback)) {
                a.Get(&fallback, ctx.time);
            }
        }
        return fallback;
    };
    const VtDoubleArray pointWeights = read("pointWeights", VtDoubleArray());
    const int uVertexCount = read("uVertexCount", 0);
    const int vVertexCount = read("vVertexCount", 0);
    const int uOrder = read("uOrder", 0);
    const int vOrder = read("vOrder", 0);
    const VtDoubleArray uKnots = read("uKnots", VtDoubleArray());
    const VtDoubleArray vKnots = read("vKnots", VtDoubleArray());
    const TfToken uForm = read("uForm", TfToken("open"));
    const TfToken vForm = read("vForm", TfToken("open"));

    bool uPeriodic = false, vPeriodic = false;
    if (!ValidForm(uForm.GetText(), &uPeriodic) ||
        !ValidForm(vForm.GetText(), &vPeriodic)) {
        return RigExecOracleResult::PassThrough;
    }

    RigExecNurbsPatch patch;
    patch.points = patchPoints.cdata();
    patch.pointCount = patchPoints.size();
    patch.pointWeights = pointWeights.cdata();
    patch.pointWeightCount = pointWeights.size();
    patch.uVertexCount = uVertexCount;
    patch.vVertexCount = vVertexCount;
    patch.uOrder = uOrder;
    patch.vOrder = vOrder;
    patch.uKnots = uKnots.cdata();
    patch.uKnotCount = uKnots.size();
    patch.vKnots = vKnots.cdata();
    patch.vKnotCount = vKnots.size();
    patch.uPeriodic = uPeriodic;
    patch.vPeriodic = vPeriodic;

    std::vector<GfVec3f> scratch(ctx.points->begin(), ctx.points->end());
    if (!shapesValid || !MixShapes(shapes, weights, &uvPoints) ||
        !TouchContacts(contactLower, contactUpper, &uvPoints) ||
        !RigExecApplyPatchMover(
            &scratch, patch, uvPoints.cdata(), uvPoints.size())) {
        if (ctx.diagnostics) {
            ctx.diagnostics->push_back(
                "MoverFailed " + ctx.moverPath.GetString() +
                ": invalid NURBS patch, UV points, UV shapes, UV weights or "
                "UV contacts");
        }
        return RigExecOracleResult::PassThrough;
    }
    std::copy(scratch.begin(), scratch.end(), ctx.points->begin());
    return RigExecOracleResult::Blend;
}

constexpr char kEpochTag[] = {'R', 'P', 'M', '3'};

// The phased paths each declared points input reads, so playback can hand a
// phased revision to its input, and the layout of the weights.
struct PreparedState {
    std::string patchPoints;
    std::string uvPoints;
    std::vector<std::string> shapes;
    Layout layout;
};

void AppendCount(std::vector<uint8_t> *out, uint32_t count)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&count);
    out->insert(out->end(), bytes, bytes + sizeof(count));
}

void AppendString(std::vector<uint8_t> *out, const std::string &s)
{
    AppendCount(out, uint32_t(s.size()));
    out->insert(out->end(), s.begin(), s.end());
}

bool ReadCount(const uint8_t *data, size_t size, size_t *offset,
               uint32_t *out)
{
    if (*offset + sizeof(uint32_t) > size) {
        return false;
    }
    std::memcpy(out, data + *offset, sizeof(*out));
    *offset += sizeof(*out);
    return true;
}

bool ReadString(const uint8_t *data, size_t size, size_t *offset,
                std::string *out)
{
    uint32_t len = 0;
    if (!ReadCount(data, size, offset, &len) || *offset + len > size) {
        return false;
    }
    out->assign(reinterpret_cast<const char *>(data + *offset), len);
    *offset += len;
    return true;
}

bool EncodeEpoch(const RigExecRevisionBinding &binding,
                 std::vector<uint8_t> *epoch)
{
    epoch->assign(kEpochTag, kEpochTag + sizeof(kEpochTag));
    const auto &inputs = binding.externalInputs;
    const Layout layout = LayoutOf(inputs);
    if (inputs.size() != InputCount(layout)) {
        return false;
    }
    AppendString(epoch, inputs[PatchPoints].path.GetString());
    AppendString(epoch, inputs[UvPoints].path.GetString());
    AppendCount(epoch, uint32_t(layout.shapes));
    for (size_t k = 0; k < layout.shapes; ++k) {
        AppendString(epoch, inputs[UvShapes + k].path.GetString());
    }
    AppendCount(epoch, layout.connected ? 1u : 0u);
    return true;
}

bool Encode(const VtValue &, const RigExecRevisionBinding &binding,
            std::vector<uint8_t> *epoch, std::vector<uint8_t> *frame)
{
    frame->clear();
    return EncodeEpoch(binding, epoch);
}

std::shared_ptr<const void> Prepare(const uint8_t *epoch, size_t size,
                                    std::string *error)
{
    if (size < sizeof(kEpochTag) ||
        std::memcmp(epoch, kEpochTag, sizeof(kEpochTag)) != 0) {
        if (error) {
            *error = "not a patch mover epoch";
        }
        return nullptr;
    }
    size_t offset = sizeof(kEpochTag);
    auto state = std::make_shared<PreparedState>();
    uint32_t shapes = 0, connected = 0;
    bool ok = ReadString(epoch, size, &offset, &state->patchPoints) &&
              ReadString(epoch, size, &offset, &state->uvPoints) &&
              ReadCount(epoch, size, &offset, &shapes);
    if (ok) {
        state->shapes.resize(shapes);
        for (std::string &path : state->shapes) {
            ok = ok && ReadString(epoch, size, &offset, &path);
        }
    }
    ok = ok && ReadCount(epoch, size, &offset, &connected) && connected <= 1;
    if (!ok || offset != size) {
        if (error) {
            *error = "corrupt patch mover epoch";
        }
        return nullptr;
    }
    state->layout.shapes = shapes;
    state->layout.connected = connected != 0;
    return state;
}

bool Play(const void *rawState, const uint8_t *, size_t,
          const RigExecExternalPhasedPoints *phased, size_t phasedCount,
          const RigExecExternalInputValue *inputs, size_t inputCount,
          const RigExecExternalProviders &providers, float *xyz, size_t count)
{
    if (!rawState) {
        return false;
    }
    auto values = RigExecExternalPlaybackInputs(inputs, inputCount);
    const auto &state = *static_cast<const PreparedState *>(rawState);
    if (values.size() != InputCount(state.layout)) {
        return false;
    }
    for (size_t k = 0; k < phasedCount; ++k) {
        if (!phased[k].path || !phased[k].xyz) {
            continue;
        }
        VtVec3fArray points(phased[k].count);
        for (size_t i = 0; i < phased[k].count; ++i) {
            points[i] = GfVec3f(phased[k].xyz[i * 3],
                                phased[k].xyz[i * 3 + 1],
                                phased[k].xyz[i * 3 + 2]);
        }
        if (state.patchPoints == phased[k].path) {
            values[PatchPoints] = VtValue(points);
        }
        if (state.uvPoints == phased[k].path) {
            values[UvPoints] = VtValue(points);
        }
        for (size_t s = 0; s < state.shapes.size(); ++s) {
            if (state.shapes[s] == phased[k].path) {
                values[UvShapes + s] = VtValue(points);
            }
        }
    }
    const RigExecExternalPlaybackProviders provided(providers);
    VtValue data;
    return Payload(values, state.layout,
                   RigExecExternalProviderValues(provided.values), &data) &&
           RigExecExternalPlaybackApply(data, &Apply, xyz, count);
}

// A points property, as rigExec:uvPoints and every rigExec:uvShapes target
// must be.
bool IsPointsProperty(const UsdStageRefPtr &stage, const SdfPath &path)
{
    const UsdAttribute attr = stage->GetAttributeAtPath(path);
    return attr &&
           attr.GetTypeName().GetType() == TfType::Find<VtVec3fArray>();
}

bool Validate(const RigExecMoverValidateContext &ctx, std::string *error)
{
    const std::string who = ctx.prim.GetPath().GetString();
    if (ctx.targets.size() != 1) {
        *error = who + ": patch mover requires a single target";
        return false;
    }
    const auto patches = RigExecRelationshipTargets(ctx.prim, "rigExec:patch");
    if (patches.size() != 1) {
        *error = who + ": rigExec:patch requires exactly one target";
        return false;
    }
    const auto uvs = RigExecRelationshipTargets(ctx.prim, "rigExec:uvPoints");
    if (uvs.size() != 1) {
        *error = who + ": rigExec:uvPoints requires exactly one target";
        return false;
    }
    const UsdPrim patchPrim =
        ctx.stage->GetPrimAtPath(patches[0].GetPrimPath());
    if (!patchPrim || !patchPrim.GetAttribute(TfToken("points"))) {
        *error = who + ": rigExec:patch target has no points attribute";
        return false;
    }
    const SdfPath patchPointsPath =
        patchPrim.GetPath().AppendProperty(TfToken("points"));
    const SdfPath uvPointsPath = RigExecPointsOf(uvs[0]);
    if (!IsPointsProperty(ctx.stage, uvPointsPath)) {
        *error = who + ": rigExec:uvPoints target is not a point3f[]/float3[] property";
        return false;
    }
    if (patchPointsPath == uvPointsPath &&
        RigExecPhaseForInput(ctx.prim, "rigExec:patch") !=
            RigExecPhaseForInput(ctx.prim, "rigExec:uvPoints")) {
        *error = who + ": rigExec:patch and rigExec:uvPoints name the same "
                       "points property with conflicting read phases";
        return false;
    }
    const auto shapes = RigExecRelationshipTargets(ctx.prim, "rigExec:uvShapes");
    VtVec3fArray neutral;
    const bool haveNeutral =
        ctx.stage->GetAttributeAtPath(uvPointsPath).Get(&neutral);
    for (const SdfPath &shape : shapes) {
        const SdfPath shapePath = RigExecPointsOf(shape);
        if (!IsPointsProperty(ctx.stage, shapePath)) {
            *error = who + ": rigExec:uvShapes target " + shapePath.GetString() +
                     " is not a point3f[]/float3[] property";
            return false;
        }
        VtVec3fArray points;
        if (haveNeutral &&
            ctx.stage->GetAttributeAtPath(shapePath).Get(&points) &&
            points.size() != neutral.size()) {
            *error = who + ": UV shape " + shapePath.GetString() + " has " +
                     std::to_string(points.size()) +
                     " points; rigExec:uvPoints has " +
                     std::to_string(neutral.size());
            return false;
        }
    }
    // The weights: as many connected sources as shapes, each a float or a
    // double; or, unconnected, an authored array of one weight per shape
    // (or none).
    const SdfPathVector sources = WeightSources(ctx.prim);
    if (!sources.empty() && sources.size() != shapes.size()) {
        *error = who + ": inputs:uvWeights connects " +
                 std::to_string(sources.size()) +
                 " sources; rigExec:uvShapes names " +
                 std::to_string(shapes.size());
        return false;
    }
    for (const SdfPath &source : sources) {
        const UsdAttribute attr = ctx.stage->GetAttributeAtPath(source);
        const TfType type = attr ? attr.GetTypeName().GetType() : TfType();
        if (type != TfType::Find<float>() && type != TfType::Find<double>()) {
            *error = who + ": inputs:uvWeights source " + source.GetString() +
                     " is not a float or double attribute";
            return false;
        }
    }
    if (sources.empty()) {
        VtFloatArray authored;
        const UsdAttribute weights = ctx.prim.GetAttribute(_uvWeights);
        if (weights && weights.GetNumTimeSamples() == 0 &&
            weights.Get(&authored) && !authored.empty() &&
            authored.size() != shapes.size()) {
            *error = who + ": inputs:uvWeights holds " +
                     std::to_string(authored.size()) +
                     " weights; rigExec:uvShapes names " +
                     std::to_string(shapes.size());
            return false;
        }
    }
    // The contact lists pair points of the neutral UV points.
    VtIntArray contactLower, contactUpper;
    if (const UsdAttribute a = ctx.prim.GetAttribute(_uvContactLower)) {
        a.Get(&contactLower);
    }
    if (const UsdAttribute a = ctx.prim.GetAttribute(_uvContactUpper)) {
        a.Get(&contactUpper);
    }
    const std::string contactError = ContactError(
        contactLower, contactUpper,
        haveNeutral ? neutral.size() : size_t(std::numeric_limits<int>::max()));
    if (!contactError.empty()) {
        *error = who + ": " + contactError;
        return false;
    }
    // Validate authored Default values when present; time-sampled or connected
    // arrays without Default values are checked per frame during assembly.
    VtVec3fArray targetPoints;
    const UsdAttribute targetAttr = ctx.stage->GetAttributeAtPath(ctx.targets[0]);
    const bool haveTargetDefault = targetAttr && targetAttr.Get(&targetPoints);
    std::vector<VtValue> inputs;
    const auto keys =
        DeclaredInputs(ctx.prim.GetPath(), patches[0], uvs[0], shapes, sources);
    bool anyAnimatedOrConnected = false;
    for (const auto &key : keys) {
        VtValue value = key.fallback;
        if (const UsdAttribute a = ctx.stage->GetAttributeAtPath(key.path)) {
            a.Get(&value);
            if (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections()) {
                anyAnimatedOrConnected = true;
            }
        }
        inputs.push_back(std::move(value));
    }
    RigExecProviderValues v;
    if (haveTargetDefault) {
        v.basePoints.assign(targetPoints.begin(), targetPoints.end());
    }
    VtValue data;
    if (Payload(inputs, LayoutOf(keys), RigExecExternalProviderValues(v),
                &data) ||
        anyAnimatedOrConnected) {
        return true;
    }
    *error = who + ": invalid NURBS patch topology, knots, weights, form, "
                   "UV point count or UV shapes";
    return false;
}

RigExecMoverHandler Handler()
{
    RigExecMoverHandler h(
        "RigExecPatchMover",
        &RigExecFixedMoverOp<RigExecRevisionOp::External>,
        RigExecMoverDomain::Points);
    h.singleTarget = true;
    h.bind = &Bind;
    h.validate = &Validate;
    h.declareExternalInputs = &DeclareInputs;
    h.compileScene = &CompileScene;
    h.assembleExternal = &Assemble;
    h.applyExternal = &Apply;
    h.oracle = &Oracle;
    h.encodeExternalEpoch = &EncodeEpoch;
    h.encodeExternal = &Encode;
    h.runtimeKernel.prepare = &Prepare;
    h.runtimeKernel.applyWithProviders = &Play;
    return h;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(Handler());
