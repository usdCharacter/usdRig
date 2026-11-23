// Persistent surface bindings; coordinate/algorithm references: docs/references.md.
#include "rigExecSchema/schemaNames.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/movers/externalPlayback.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExecMath/surfaceBindingKernel.h"
#include "rigExecScene/sceneDescriptors.h"
#include "rigExecScene/surfaceBindingInputs.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/base/work/loops.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

// Relationship names the binders and validators read, interned once:
// building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_relTokens,
    ((surface, rigExec::SchemaAttrText::rigExecSurface))
    ((frames, rigExec::SchemaAttrText::rigExecFrames)));

// Interned once: callbacks, binders and validators run on parallel
// lanes, and building a token from text takes the registry lock.
TF_DEFINE_PRIVATE_TOKENS(_bindingTokens,
    (polygon)
    (geometric)
    ((surface, rigExec::SchemaAttrText::rigExecSurface))
    (points)
    (faceVertexCounts)
    (faceVertexIndices)
    ((offsets, rigExec::SchemaAttrText::rigExecOffsets))
    ((surfaceMode, rigExec::SchemaAttrText::rigExecSurfaceMode))
    ((normalMode, rigExec::SchemaAttrText::rigExecNormalMode))
    ((strength, rigExec::SchemaAttrText::rigExecStrength))
    ((deltaMultiplier, rigExec::SchemaAttrText::rigExecDeltaMultiplier))
    ((surfaceRestMatrix, rigExec::SchemaAttrText::rigExecSurfaceRestMatrix))
    ((targetRestMatrix, rigExec::SchemaAttrText::rigExecTargetRestMatrix))
    ((surfaceToBinding, rigExec::SchemaAttrText::rigExecSurfaceToBinding))
    ((frames, rigExec::SchemaAttrText::rigExecFrames))
    ((rigExecVertices, rigExec::SchemaAttrText::rigExecVertices))
    ((rigExecVertexOffsets, rigExec::SchemaAttrText::rigExecVertexOffsets))
    ((rigExecPolygonOffsets, rigExec::SchemaAttrText::rigExecPolygonOffsets))
    ((rigExecPointIndices, rigExec::SchemaAttrText::rigExecPointIndices))
    ((rigExecLimitOffsets, rigExec::SchemaAttrText::rigExecLimitOffsets))
    ((rigExecLimitIndices, rigExec::SchemaAttrText::rigExecLimitIndices))
    ((rigExecBarycentricWeights, rigExec::SchemaAttrText::rigExecBarycentricWeights))
    ((rigExecBindingWeights, rigExec::SchemaAttrText::rigExecBindingWeights))
    ((rigExecMask, rigExec::SchemaAttrText::rigExecMask))
    ((rigExecDeltaMultipliers, rigExec::SchemaAttrText::rigExecDeltaMultipliers))
    ((rigExecLimitWeights, rigExec::SchemaAttrText::rigExecLimitWeights))
    ((rigExecLimitDuWeights, rigExec::SchemaAttrText::rigExecLimitDuWeights))
    ((rigExecLimitDvWeights, rigExec::SchemaAttrText::rigExecLimitDvWeights)));

namespace {
// The binding's own arrays and settings, then the bound surface's points and
// topology, in this order: assembly reads them by position.
const char *const _intArrays[] = {"vertices",       "vertexOffsets",
                                  "polygonOffsets", "pointIndices",
                                  "limitOffsets",   "limitIndices"};
const char *const _floatArrays[] = {
    "barycentricWeights", "bindingWeights", "mask",          "deltaMultipliers",
    "limitWeights",       "limitDuWeights", "limitDvWeights"};
constexpr size_t _intCount = sizeof(_intArrays) / sizeof(_intArrays[0]);
constexpr size_t _floatCount = sizeof(_floatArrays) / sizeof(_floatArrays[0]);
static_assert(_intCount == surfaceBindingInputs::kIntArrays &&
                  _floatCount == surfaceBindingInputs::kFloatArrays,
              "declared inputs out of step with surfaceBindingInputs.h");
// The arrays' attribute names, interned once at load in the same order.
const std::array<TfToken, _intCount> _intArrayNames = {
    _bindingTokens->rigExecVertices,       _bindingTokens->rigExecVertexOffsets,
    _bindingTokens->rigExecPolygonOffsets, _bindingTokens->rigExecPointIndices,
    _bindingTokens->rigExecLimitOffsets,   _bindingTokens->rigExecLimitIndices};
const std::array<TfToken, _floatCount> _floatArrayNames = {
    _bindingTokens->rigExecBarycentricWeights,
    _bindingTokens->rigExecBindingWeights,
    _bindingTokens->rigExecMask,
    _bindingTokens->rigExecDeltaMultipliers,
    _bindingTokens->rigExecLimitWeights,
    _bindingTokens->rigExecLimitDuWeights,
    _bindingTokens->rigExecLimitDvWeights};
using namespace surfaceBindingInputs;

// Every read is through the resolved inputs at the evaluated time, over the
// schema fallback; the surface's points carry the rigExec:surface phase.
std::vector<RigExecRevisionLeafKey>
DeclaredInputs(const SdfPath &mover, const SdfPath &surface)
{
    using Type = RigExecRevisionLeafType;
    const auto key = [](const SdfPath &path, Type type, VtValue fallback) {
        return RigExecRevisionLeafKey{
            path, type, RigExecRevisionLeafTime::AtTime,
            RigExecRevisionLeafFlavour::Resolved, std::move(fallback)};
    };
    const auto own = [&](const TfToken &name) {
        return mover.AppendProperty(name);
    };
    std::vector<RigExecRevisionLeafKey> inputs;
    for (const TfToken &name : _intArrayNames)
        inputs.push_back(key(own(name), Type::IntArray, VtValue(VtIntArray())));
    for (const TfToken &name : _floatArrayNames)
        inputs.push_back(
            key(own(name), Type::FloatArray, VtValue(VtFloatArray())));
    inputs.push_back(key(own(_bindingTokens->offsets), Type::Vec3fArray,
                         VtValue(VtVec3fArray())));
    inputs.push_back(key(own(_bindingTokens->surfaceMode), Type::Token,
                         VtValue(_bindingTokens->polygon)));
    inputs.push_back(key(own(_bindingTokens->normalMode), Type::Token,
                         VtValue(_bindingTokens->geometric)));
    inputs.push_back(
        key(own(_bindingTokens->strength), Type::Float, VtValue(1.0f)));
    inputs.push_back(
        key(own(_bindingTokens->deltaMultiplier), Type::Float, VtValue(1.0f)));
    inputs.push_back(key(own(_bindingTokens->surfaceRestMatrix), Type::Matrix4d,
                         VtValue(GfMatrix4d(1.0))));
    inputs.push_back(key(own(_bindingTokens->targetRestMatrix), Type::Matrix4d,
                         VtValue(GfMatrix4d(1.0))));
    inputs.push_back(key(own(_bindingTokens->surfaceToBinding), Type::Matrix4d,
                         VtValue(GfMatrix4d(1.0))));
    if (surface.IsEmpty()) {
        // No single surface: a raw read of the relationship, where no
        // attribute stands, answers empty and the assembly refuses it.
        const SdfPath unbound = mover.AppendProperty(_bindingTokens->surface);
        for (Type type : {Type::Vec3fArray, Type::IntArray, Type::IntArray}) {
            inputs.push_back({unbound, type, RigExecRevisionLeafTime::AtTime,
                              RigExecRevisionLeafFlavour::Raw, VtValue()});
        }
        return inputs;
    }
    const SdfPath prim = surface.GetPrimPath();
    inputs.push_back(key(prim.AppendProperty(_bindingTokens->points),
                         Type::Vec3fArray, VtValue(VtVec3fArray())));
    inputs.push_back(key(prim.AppendProperty(_bindingTokens->faceVertexCounts),
                         Type::IntArray, VtValue(VtIntArray())));
    inputs.push_back(key(prim.AppendProperty(_bindingTokens->faceVertexIndices),
                         Type::IntArray, VtValue(VtIntArray())));
    return inputs;
}

template <class T>
bool
As(const VtValue &value, T *out)
{
    if (!value.IsHolding<T>())
        return false;
    *out = value.UncheckedGet<T>();
    return true;
}
template <class T>
const T *
Get(const VtDictionary &d, const char *key)
{
    const auto found = d.find(key);
    return found != d.end() && found->second.IsHolding<T>()
               ? &found->second.UncheckedGet<T>()
               : nullptr;
}
// A token input's text: a TfToken from the stage-side declared inputs, a
// std::string from playback, which builds no token (TfToken(text) takes the
// registry).
bool
Text(const VtValue &value, const char **out)
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
bool
Is(const char *token, const char *text)
{
    return std::strcmp(token, text) == 0;
}
template <class T>
RigExecSurfaceBindingList<T>
List(const VtArray<T> &a)
{
    return {a.cdata(), a.size()};
}
// The surface binding the payload \p d holds, as the kernel reads it.
// Its arrays stay owned by \p d.
RigExecSurfaceBinding<GfVec3f>
View(const VtDictionary &d)
{
    RigExecSurfaceBinding<GfVec3f> b;
    RigExecSurfaceBindingList<int> *ints[] = {
        &b.vertices,     &b.vertexOffsets, &b.polygonOffsets,
        &b.pointIndices, &b.limitOffsets,  &b.limitIndices};
    RigExecSurfaceBindingList<float> *floats[] = {
        &b.barycentricWeights, &b.bindingWeights, &b.mask,
        &b.deltaMultipliers,   &b.limitWeights,   &b.limitDuWeights,
        &b.limitDvWeights};
    for (size_t i = 0; i < _intCount; ++i)
        if (const auto *a = Get<VtIntArray>(d, _intArrays[i]))
            *ints[i] = List(*a);
    for (size_t i = 0; i < _floatCount; ++i)
        if (const auto *a = Get<VtFloatArray>(d, _floatArrays[i]))
            *floats[i] = List(*a);
    if (const auto *a = Get<VtVec3fArray>(d, "offsets"))
        b.offsets = List(*a);
    if (const auto *x = Get<bool>(d, "limit"))
        b.limit = *x;
    if (const auto *x = Get<bool>(d, "smooth"))
        b.smooth = *x;
    if (const auto *x = Get<float>(d, "strength"))
        b.strength = *x;
    if (const auto *x = Get<float>(d, "deltaMultiplier"))
        b.deltaMultiplier = *x;
    return b;
}
// The bind loop's runner: split across threads in the caller's
// floating-point environment whenever parallel evaluation is on and this
// thread is not in a frozen serial scope. The kernel decides by the
// stencil work whether to split at all.
void
RunInCallerEnvironment(size_t count, size_t grain, const RigExecRangeBody &body)
{
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive())
        RigExecParallelForInCallerEnvironment(count, grain, body);
    else
        body(0, count);
}
// Built at load: a function-local static would run its initialization
// guard on every call.
const RigExecRangeRunner _bindRanges(&RunInCallerEnvironment);
void
Bind(const RigExecMoverBindContext &ctx)
{
    auto &b = *ctx.binding;
    const auto surfaces =
        RigExecRelationshipTargets(ctx.moverPrim, _relTokens->surface);
    if (surfaces.size() == 1) {
        b.surfacePoints =
            surfaces[0].GetPrimPath().AppendProperty(_bindingTokens->points);
        b.phases[b.surfacePoints] =
            RigExecPhaseForInput(ctx.moverPrim, _relTokens->surface);
    }
    b.influences =
        RigExecRelationshipTargets(ctx.moverPrim, _relTokens->frames);
    b.transformPhase = RigExecPhaseForInput(ctx.moverPrim, _relTokens->frames);
    if (b.transformPhase.kind == RigExecReadPhaseKind::Final)
        for (auto &p : b.influences) {
            auto it = ctx.frameChainHeads.find(p);
            if (it != ctx.frameChainHeads.end())
                p = it->second;
        }
}
void
DeclareInputs(const RigExecMoverBindContext &ctx,
              std::vector<RigExecRevisionLeafKey> *inputs)
{
    const auto surfaces =
        RigExecRelationshipTargets(ctx.moverPrim, _relTokens->surface);
    *inputs = DeclaredInputs(ctx.moverPrim.GetPath(),
                             surfaces.size() == 1 ? surfaces[0] : SdfPath());
}
bool
CompileScene(const RigExecSceneDescriptors &scene, const SdfPath &mover,
             const SdfPath &, RigExecRevisionBinding *binding,
             std::string *error)
{
    if (!binding)
        return false;
    const auto relationship = [&](const TfToken &name) {
        const auto found = scene.relationships.find(mover.AppendProperty(name));
        return found == scene.relationships.end() ? nullptr : &found->second;
    };
    SdfPath surface;
    if (const auto *rel = relationship(_bindingTokens->surface);
        rel && rel->fact.targets.size() == 1) {
        surface = rel->fact.targets[0];
        binding->surfacePoints =
            surface.GetPrimPath().AppendProperty(_bindingTokens->points);
        RigExecReadPhase phase;
        if (!RigExecParseReadPhase(rel->readPhase.GetString(), &phase, error))
            return false;
        binding->phases[binding->surfacePoints] = phase;
    }
    if (const auto *rel = relationship(_bindingTokens->frames)) {
        binding->influences = rel->fact.targets;
        if (!RigExecParseReadPhase(rel->readPhase.GetString(),
                                   &binding->transformPhase, error))
            return false;
    }
    binding->externalInputs = DeclaredInputs(mover, surface);
    return true;
}
// The payload from the declared inputs and the provider values: the
// binding's arrays and settings, the surface's points in binding space and
// the map to the target. Pure: no stage, registry or shared state.
bool
Payload(const std::vector<VtValue> &in, const RigExecExternalProviderValues &v,
        VtValue *out)
{
    if (in.size() != InputCount)
        return false;
    VtDictionary d;
    for (size_t i = 0; i < _intCount; ++i) {
        VtIntArray a;
        if (!As(in[FirstInt + i], &a))
            return false;
        d[_intArrays[i]] = VtValue(a);
    }
    for (size_t i = 0; i < _floatCount; ++i) {
        VtFloatArray a;
        if (!As(in[FirstFloat + i], &a))
            return false;
        d[_floatArrays[i]] = VtValue(a);
    }
    VtVec3fArray offsets;
    if (!As(in[OffsetVectors], &offsets))
        return false;
    d["offsets"] = VtValue(offsets);
    const char *mode = nullptr, *normal = nullptr;
    float strength = 0, deltaMultiplier = 0;
    if (!Text(in[SurfaceMode], &mode) || !Text(in[NormalMode], &normal) ||
        !As(in[Strength], &strength) ||
        !As(in[DeltaMultiplier], &deltaMultiplier) ||
        (!Is(mode, "polygon") && !Is(mode, "limit")) ||
        (!Is(normal, "geometric") && !Is(normal, "smooth")))
        return false;
    const bool limit = Is(mode, "limit"), smooth = Is(normal, "smooth");
    d["limit"] = VtValue(limit);
    d["smooth"] = VtValue(smooth);
    d["strength"] = VtValue(strength);
    d["deltaMultiplier"] = VtValue(deltaMultiplier);
    VtVec3fArray points;
    GfMatrix4d sourceRest, targetRest, bind;
    if (!As(in[SurfacePoints], &points) ||
        !As(in[SurfaceRestMatrix], &sourceRest) ||
        !As(in[TargetRestMatrix], &targetRest) ||
        !As(in[SurfaceToBinding], &bind))
        return false;
    if (v.influenceTransforms && v.influenceTransforms->size() != 2)
        return false;
    VtIntArray counts, indices;
    if (smooth &&
        (!As(in[SurfaceCounts], &counts) || !As(in[SurfaceIndices], &indices)))
        return false;
    GfMatrix4d toTarget;
    if (!RigExecPrepareSurfaceBinding<GfVec3f, GfVec3d, GfMatrix4d>(
            View(d), v.basePoints.size(), sourceRest, targetRest, bind,
            v.influenceTransforms ? v.influenceTransforms->data() : nullptr,
            List(counts), List(indices), points.data(), points.size(),
            &toTarget, RigExecGeometryRanges()))
        return false;
    d["points"] = VtValue(points);
    d["toTarget"] = VtValue(toTarget);
    d["counts"] = VtValue(counts);
    d["topology"] = VtValue(indices);
    *out = VtValue(d);
    return true;
}
bool
Assemble(const RigExecExternalInputContext &ctx, VtValue *out)
{
    return Payload(ctx.inputs, ctx.values, out);
}
bool
Apply(const VtValue &data, std::vector<GfVec3f> *output)
{
    if (!data.IsHolding<VtDictionary>())
        return false;
    const auto &d = data.UncheckedGet<VtDictionary>();
    const auto *points = Get<VtVec3fArray>(d, "points");
    const auto *toTarget = Get<GfMatrix4d>(d, "toTarget");
    const auto *counts = Get<VtIntArray>(d, "counts"),
               *topology = Get<VtIntArray>(d, "topology");
    if (!points || !toTarget || !counts || !topology)
        return false;
    for (const char *key :
         {"offsets", "limit", "smooth", "strength", "deltaMultiplier"})
        if (d.find(key) == d.end())
            return false;
    for (const char *key : _intArrays)
        if (!Get<VtIntArray>(d, key))
            return false;
    for (const char *key : _floatArrays)
        if (!Get<VtFloatArray>(d, key))
            return false;
    return RigExecApplySurfaceBinding<GfVec3f, GfVec3d, GfMatrix4d>(
        View(d), points->cdata(), points->size(), List(*counts),
        List(*topology), *toTarget, output->data(), output->size(),
        &_bindRanges);
}
// .rigexec playback: the compiled rig is a format tag and the bound surface's
// points path, and the frame bytes are empty. The kernel assembles and
// deforms through Payload and Apply from the declared inputs, the surface
// points playback evaluated at the binding's phase, and its provider
// values, so a posed playback follows the surface and the frames.
constexpr char kCompiledDataTag[] = {'R', 'S', 'B', '1'};
bool
EncodeCompiledData(const RigExecRevisionBinding &binding,
                   std::vector<uint8_t> *compiledRig)
{
    compiledRig->assign(kCompiledDataTag,
                        kCompiledDataTag + sizeof(kCompiledDataTag));
    std::string surface;
    if (!binding.surfacePoints.IsEmpty() &&
        !RigExecSpellPathText(binding.surfacePoints, &surface))
        surface = binding.surfacePoints.GetAsString();
    compiledRig->insert(compiledRig->end(), surface.begin(), surface.end());
    return true;
}
bool
Encode(const VtValue &, const RigExecRevisionBinding &binding,
       std::vector<uint8_t> *compiledRig, std::vector<uint8_t> *frame)
{
    frame->clear();
    return EncodeCompiledData(binding, compiledRig);
}
// The prepared state: the bound surface's points path.
std::shared_ptr<const void>
Prepare(const uint8_t *compiledRig, size_t size, std::string *error)
{
    if (size < sizeof(kCompiledDataTag) ||
        std::memcmp(compiledRig, kCompiledDataTag, sizeof(kCompiledDataTag)) !=
            0) {
        if (error)
            *error = "not a surface compiled rig";
        return nullptr;
    }
    return std::make_shared<const std::string>(
        reinterpret_cast<const char *>(compiledRig) + sizeof(kCompiledDataTag),
        size - sizeof(kCompiledDataTag));
}
bool
Play(const void *state, const uint8_t *, size_t,
     const RigExecExternalPhasedPoints *phased, size_t phasedCount,
     const RigExecExternalInputValue *inputs, size_t inputCount,
     const RigExecExternalProviders &providers, float *xyz, size_t count)
{
    auto values = RigExecExternalPlaybackInputs(inputs, inputCount);
    // The surface's points at the binding's phase, as playback evaluated
    // them, in place of the declared read -- the assembly's phase overlay.
    const auto &surface = *static_cast<const std::string *>(state);
    for (size_t k = 0; k < phasedCount && SurfacePoints < values.size(); ++k) {
        if (!phased[k].path || !phased[k].xyz || surface != phased[k].path)
            continue;
        VtVec3fArray points(phased[k].count);
        for (size_t i = 0; i < phased[k].count; ++i)
            points[i] = GfVec3f(phased[k].xyz[i * 3], phased[k].xyz[i * 3 + 1],
                                phased[k].xyz[i * 3 + 2]);
        values[SurfacePoints] = VtValue(points);
    }
    const RigExecExternalPlaybackProviders provided(providers);
    VtValue data;
    return Payload(values, RigExecExternalProviderValues(provided.values),
                   &data) &&
           RigExecExternalPlaybackApply(data, &Apply, xyz, count);
}
bool
Validate(const RigExecMoverValidateContext &ctx, std::string *error)
{
    VtVec3fArray p;
    if (ctx.targets.size() != 1 ||
        !ctx.stage->GetAttributeAtPath(ctx.targets[0]).Get(&p))
        return false;
    const auto frames =
        RigExecRelationshipTargets(ctx.prim, _relTokens->frames);
    if (!frames.empty() && frames.size() != 2) {
        *error = "surface binding requires zero or two frame providers";
        return false;
    }
    const auto surfaces =
        RigExecRelationshipTargets(ctx.prim, _relTokens->surface);
    // The authored arrays against identity frames, the authored points and
    // the surface's Default shape.
    std::vector<VtValue> inputs;
    for (const auto &key :
         DeclaredInputs(ctx.prim.GetPath(),
                        surfaces.size() == 1 ? surfaces[0] : SdfPath())) {
        VtValue value = key.fallback;
        if (const UsdAttribute a = ctx.stage->GetAttributeAtPath(key.path))
            a.Get(&value);
        inputs.push_back(value);
    }
    RigExecProviderValues v;
    v.basePoints.assign(p.begin(), p.end());
    VtValue data;
    if (Payload(inputs, RigExecExternalProviderValues(v), &data))
        return true;
    *error =
        ctx.prim.GetPath().GetString() +
        ": invalid surface binding arrays, stencil, mask or coordinate frame";
    return false;
}
RigExecMoverHandler
Handler()
{
    RigExecMoverHandler h(rigExec::SchemaTypeName::RigExecSurfaceBindingMover,
                          &RigExecFixedMoverOp<RigExecRevisionOp::External>,
                          RigExecMoverDomain::Points);
    h.singleTarget = true;
    h.bind = &Bind;
    h.validate = &Validate;
    h.declareExternalInputs = &DeclareInputs;
    h.compileScene = &CompileScene;
    h.assembleExternal = &Assemble;
    h.applyExternal = &Apply;
    h.encodeExternalCompiledData = &EncodeCompiledData;
    h.encodeExternal = &Encode;
    h.runtimeKernel.prepare = &Prepare;
    h.runtimeKernel.applyWithProviders = &Play;
    h.frameRelationships = {rigExec::SchemaAttrText::rigExecFrames};
    h.transformRelationship = rigExec::SchemaAttrText::rigExecFrames;
    h.hasScalarOracle = false;
    return h;
}
} // namespace
RIGEXEC_REGISTER_MOVER(Handler());
