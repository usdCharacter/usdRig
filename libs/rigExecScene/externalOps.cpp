#include "rigExecScene/externalOps.h"

#include "rigExec/api/moverRegistry.h"
#include "rigExec/api/registry.h"
#include "rigExecOps/deform/clothOp.h"
#include "rigExecOps/deform/layeredSkinOp.h"
#include "rigExecOps/deform/patchOp.h"
#include "rigExecOps/deform/surfaceBindingOp.h"
#include "rigExecSchema/schemaTokens.h"
#include "rigExecScene/clothDeclaredInputs.h"
#include "rigExecScene/layeredSkinInputs.h"
#include "rigExecScene/opPlugins.h"
#include "rigExecScene/patchInputs.h"
#include "rigExecScene/surfaceBindingInputs.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <iterator>

namespace rigExec {

namespace {

/// The cloth mover's op, written from one frame's values until its binder
/// builds from compile facts.
struct LegacyBinder {
    const char *schemaType = "";
    const RigExecOpHandle &(*op)() = nullptr;
    bool (*add)(const RigExecRevisionBinding &, const RigExecExternalOpValues &,
                RigExecRigDescBuilder &, RigExecExternalOpNode &,
                std::string *) = nullptr;
    bool (*write)(const RigExecRevisionBinding &,
                  const RigExecExternalOpValues &,
                  const RigExecExternalOpNode &, RigExecEvaluator &,
                  std::string *) = nullptr;
    bool (*stale)(const RigExecRevisionBinding &,
                  const RigExecExternalOpValues &,
                  const RigExecExternalOpNode &) = nullptr;
};

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
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

/// A token input's text: a TfToken from the stage, a std::string from
/// playback.
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
Is(const char *text, const char *literal)
{
    return std::strcmp(text, literal) == 0;
}

RigExecArray<Vec3f>
ToStd(const GfVec3f *points, size_t count)
{
    RigExecArray<Vec3f> out(count);
    for (size_t i = 0; i < count; ++i)
        out[i] = Vec3f(points[i][0], points[i][1], points[i][2]);
    return out;
}

RigExecArray<Vec3f>
ToStd(const VtVec3fArray &points)
{
    return ToStd(points.cdata(), points.size());
}

template <class T>
RigExecArray<T>
ToArray(const VtArray<T> &v)
{
    return RigExecArray<T>(v.cdata(), v.size());
}

bool
Valid(const RigExecInputId &id)
{
    return id.index != kRigExecInvalidIndex;
}

// --- Built-in binders ------------------------------------------------------

using Source = RigExecExternalOpSource;

/// An open, closed or periodic form; false for any other.
bool
Periodic(const VtValue &value, bool *periodic)
{
    const char *form = nullptr;
    if (!Text(value, &form))
        return false;
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

/// A weight as a double, from a float or a double value.
bool
Scalar(const VtValue &value, double *out)
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

Mat4d
ToStd(const GfMatrix4d &m)
{
    Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out._mtx[r][c] = m[r][c];
    return out;
}

/// Pins declared input \p k of \p context and reads it as \p T through
/// \p read, false with \p error where it cannot be pinned or does not read.
template <class T, class Read>
bool
PinAs(const RigExecExternalOpContext &context, size_t k,
      RigExecExternalOpPlan &plan, const char *what, Read read, T *out,
      std::string *error)
{
    const VtValue *value =
        RigExecExternalOpPin(context, uint32_t(k), plan, error);
    if (!value)
        return false;
    if (!read(*value, out))
        return Fail(error, context.binding.moverPath.GetString() + ": " + what +
                               " is not usable");
    return true;
}

bool
ReadInt(const VtValue &value, int32_t *out)
{
    if (!value.IsHolding<int>())
        return false;
    *out = int32_t(value.UncheckedGet<int>());
    return true;
}

bool
ReadBool(const VtValue &value, bool *out)
{
    return As(value, out);
}

bool
BuildPatch(const RigExecExternalOpContext &context,
           RigExecRigDescBuilder &builder, RigExecExternalOpPlan &plan,
           std::string *error)
{
    using namespace patchInputs;
    namespace op = rigExecOps;
    const RigExecRevisionBinding &binding = context.binding;
    const Layout layout = LayoutOf(binding.externalInputs);
    if (binding.externalInputs.size() != InputCount(layout))
        return Fail(error, binding.moverPath.GetString() +
                               ": the patch mover declares other inputs");
    op::PatchPayload p;
    if (!PinAs(context, PatchUVertexCount, plan, "the u vertex count", &ReadInt,
               &p.uVertexCount, error) ||
        !PinAs(context, PatchVVertexCount, plan, "the v vertex count", &ReadInt,
               &p.vVertexCount, error) ||
        !PinAs(context, PatchUOrder, plan, "the u order", &ReadInt, &p.uOrder,
               error) ||
        !PinAs(context, PatchVOrder, plan, "the v order", &ReadInt, &p.vOrder,
               error) ||
        !PinAs(context, PatchUForm, plan, "the u form", &Periodic, &p.uPeriodic,
               error) ||
        !PinAs(context, PatchVForm, plan, "the v form", &Periodic, &p.vPeriodic,
               error))
        return false;
    p.shapes = int32_t(layout.shapes);
    const uint32_t shapes = uint32_t(layout.shapes);
    const uint32_t weights = uint32_t(WeightsBegin(layout));
    const uint32_t contacts = uint32_t(ContactsBegin(layout));
    plan.node = builder.AddOp(op::Patch(), &p);
    plan.out = &op::PatchOut();
    plan.feeds = {{&op::PatchPoints(), Source::Points},
                  {&op::PatchControlPoints(), Source::Input, PatchPoints},
                  {&op::PatchPointWeights(), Source::Input, PatchPointWeights},
                  {&op::PatchUKnots(), Source::Input, PatchUKnots},
                  {&op::PatchVKnots(), Source::Input, PatchVKnots},
                  {&op::PatchUvPoints(), Source::Input, UvPoints},
                  {&op::PatchContactLower(), Source::Input, contacts},
                  {&op::PatchContactUpper(), Source::Input, contacts + 1}};
    if (shapes)
        plan.feeds.push_back(
            {&op::PatchUvShapes(), Source::Input, UvShapes, shapes});
    if (!layout.connected)
        plan.feeds.push_back(
            {&op::PatchAuthoredUvWeights(), Source::Input, weights});
    else if (shapes)
        plan.feeds.push_back(
            {&op::PatchUvWeights(), Source::Input, weights, shapes});
    return true;
}

/// A surface or normal mode: \p yes for \p on, false for \p off.
template <const char *const *Modes>
bool
ReadMode(const VtValue &value, bool *on)
{
    const char *mode = nullptr;
    if (!Text(value, &mode) || (!Is(mode, Modes[0]) && !Is(mode, Modes[1])))
        return false;
    *on = Is(mode, Modes[1]);
    return true;
}

constexpr const char *kSurfaceModes[] = {"polygon", "limit"};
constexpr const char *kNormalModes[] = {"geometric", "smooth"};

bool
BuildSurfaceBinding(const RigExecExternalOpContext &context,
                    RigExecRigDescBuilder &builder, RigExecExternalOpPlan &plan,
                    std::string *error)
{
    using namespace surfaceBindingInputs;
    namespace op = rigExecOps;
    const RigExecRevisionBinding &binding = context.binding;
    if (binding.externalInputs.size() != InputCount)
        return Fail(error, binding.moverPath.GetString() +
                               ": the surface binding declares other inputs");
    op::SurfaceBindingPayload p;
    if (!PinAs(context, SurfaceMode, plan, "the surface mode",
               &ReadMode<kSurfaceModes>, &p.limit, error) ||
        !PinAs(context, NormalMode, plan, "the normal mode",
               &ReadMode<kNormalModes>, &p.smooth, error))
        return false;
    p.framed = !binding.influences.empty();
    if (p.framed && binding.influences.size() != 2)
        return Fail(error, binding.moverPath.GetString() +
                               ": a surface binding is framed by the "
                               "surface's and the target's frames");
    plan.node = builder.AddOp(op::SurfaceBinding(), &p);
    plan.out = &op::SurfaceBindingOut();
    const RigExecPortHandle &(*const ints[kIntArrays])() = {
        &op::SurfaceBindingVertices,       &op::SurfaceBindingVertexOffsets,
        &op::SurfaceBindingPolygonOffsets, &op::SurfaceBindingPointIndices,
        &op::SurfaceBindingLimitOffsets,   &op::SurfaceBindingLimitIndices};
    const RigExecPortHandle &(*const floats[kFloatArrays])() = {
        &op::SurfaceBindingBarycentricWeights,
        &op::SurfaceBindingBindingWeights,
        &op::SurfaceBindingMask,
        &op::SurfaceBindingDeltaMultipliers,
        &op::SurfaceBindingLimitWeights,
        &op::SurfaceBindingLimitDuWeights,
        &op::SurfaceBindingLimitDvWeights};
    plan.feeds = {{&op::SurfaceBindingPoints(), Source::Points}};
    for (size_t i = 0; i < kIntArrays; ++i)
        plan.feeds.push_back(
            {&ints[i](), Source::Input, uint32_t(FirstInt + i)});
    for (size_t i = 0; i < kFloatArrays; ++i)
        plan.feeds.push_back(
            {&floats[i](), Source::Input, uint32_t(FirstFloat + i)});
    for (const auto &[port, input] :
         {std::make_pair(&op::SurfaceBindingOffsets, OffsetVectors),
          std::make_pair(&op::SurfaceBindingStrength, Strength),
          std::make_pair(&op::SurfaceBindingDeltaMultiplier, DeltaMultiplier),
          std::make_pair(&op::SurfaceBindingSurfaceRest, SurfaceRestMatrix),
          std::make_pair(&op::SurfaceBindingTargetRest, TargetRestMatrix),
          std::make_pair(&op::SurfaceBindingSurfaceToBinding, SurfaceToBinding),
          std::make_pair(&op::SurfaceBindingSurface, SurfacePoints)})
        plan.feeds.push_back({&port(), Source::Input, uint32_t(input)});
    // Faces are read for smooth normals only.
    if (p.smooth) {
        plan.feeds.push_back(
            {&op::SurfaceBindingCounts(), Source::Input, SurfaceCounts});
        plan.feeds.push_back(
            {&op::SurfaceBindingIndices(), Source::Input, SurfaceIndices});
    }
    if (p.framed)
        plan.feeds.push_back(
            {&op::SurfaceBindingFrames(), Source::Influence, 0, 2});
    return true;
}

constexpr const char *kSkinningMethods[] = {"classicLinear", "dualQuaternion"};

bool
BuildLayeredSkin(const RigExecExternalOpContext &context,
                 RigExecRigDescBuilder &builder, RigExecExternalOpPlan &plan,
                 std::string *error)
{
    using namespace layeredSkinInputs;
    namespace op = rigExecOps;
    const RigExecRevisionBinding &binding = context.binding;
    if (binding.externalInputs.size() != kInputCount)
        return Fail(error, binding.moverPath.GetString() +
                               ": the layered skin declares other inputs");
    if (binding.influences.empty())
        return Fail(error,
                    binding.moverPath.GetString() + ": names no influences");
    op::LayeredSkinPayload p;
    bool fromBase = false, transformInput = false;
    if (!PinAs(context, kElementSize, plan, "the element size", &ReadInt,
               &p.elementSize, error) ||
        !PinAs(context, kSkinningMethod, plan, "the skinning method",
               &ReadMode<kSkinningMethods>, &p.dualQuaternion, error) ||
        !PinAs(context, kUseBaseInput, plan, "inputs:useBaseInput", &ReadBool,
               &fromBase, error) ||
        !PinAs(context, kTransformInput, plan, "the transform input", &ReadBool,
               &transformInput, error))
        return false;
    plan.node = builder.AddOp(op::LayeredSkin(), &p);
    plan.out = &op::LayeredSkinOut();
    plan.feeds = {{&op::LayeredSkinPoints(), Source::Points},
                  {&op::LayeredSkinTransforms(), Source::Influence, 0,
                   uint32_t(binding.influences.size())},
                  {&op::LayeredSkinIndices(), Source::Input, kJointIndices},
                  {&op::LayeredSkinWeights(), Source::Input, kJointWeights},
                  {&op::LayeredSkinMask(), Source::Input, kMask}};
    if (fromBase)
        plan.feeds.push_back({&op::LayeredSkinBase(), Source::BasePoints});
    if (transformInput)
        plan.feeds.push_back(
            {&op::LayeredSkinInputTransform(), Source::Transform});
    return true;
}

const RigExecExternalOpBinder kBuiltinBinders[] = {
    {sizeof(RigExecExternalOpBinder), SchemaTypeName::RigExecPatchMover,
     &rigExecOps::Patch, &BuildPatch},
    {sizeof(RigExecExternalOpBinder),
     SchemaTypeName::RigExecSurfaceBindingMover, &rigExecOps::SurfaceBinding,
     &BuildSurfaceBinding},
    {sizeof(RigExecExternalOpBinder), SchemaTypeName::RigExecLayeredSkinMover,
     &rigExecOps::LayeredSkin, &BuildLayeredSkin},
};

// --- Cloth -----------------------------------------------------------------

/// Whether \p input is one of the per-tick inputs, which reach the op as
/// ports rather than its compiled rig: the parameters, and a collider's
/// points, whose count alone the compiled rig reads.
bool
PerTick(size_t input, size_t colliders, bool *countOnly)
{
    *countOnly = false;
    if (input >= RigExecClothStretchCompliance && input <= RigExecClothFriction)
        return true;
    if (input >= RigExecClothFirstCollider &&
        input < RigExecClothFirstCollider +
                    colliders * RigExecClothInputsPerCollider &&
        (input - RigExecClothFirstCollider) % RigExecClothInputsPerCollider ==
            0) {
        *countOnly = true;
        return true;
    }
    return false;
}

/// Whether \p in builds the compiled rig \p kept built: every input equal
/// but the per-tick ones, of which a collider's point count must hold.
bool
SameClothRig(const std::vector<VtValue> &in, const std::vector<VtValue> &kept,
             size_t colliders)
{
    if (in.size() != kept.size())
        return false;
    for (size_t k = 0; k < in.size(); ++k) {
        bool countOnly = false;
        if (!PerTick(k, colliders, &countOnly)) {
            if (in[k] != kept[k])
                return false;
        } else if (countOnly) {
            if (!in[k].IsHolding<VtVec3fArray>() ||
                !kept[k].IsHolding<VtVec3fArray>() ||
                in[k].UncheckedGet<VtVec3fArray>().size() !=
                    kept[k].UncheckedGet<VtVec3fArray>().size())
                return false;
        }
    }
    return true;
}

/// One tick's values: the parameters and each collider's points.
struct ClothTick {
    RigExecArray<float> parameters;
    std::vector<RigExecArray<Vec3f>> colliders;
};

bool
ReadClothTick(const RigExecRevisionBinding &binding,
              const std::vector<VtValue> &in, ClothTick *out)
{
    if (in.size() < RigExecClothFirstCollider)
        return false;
    const size_t colliders = RigExecClothColliderCount(
        RigExecClothLayout(binding.externalCompileData), in.size());
    if (colliders >
        (in.size() - RigExecClothFirstCollider) / RigExecClothInputsPerCollider)
        return false;
    const auto parameters = RigExecClothParameterValues(in);
    out->parameters = RigExecArray<float>(parameters.data(), parameters.size());
    out->colliders.resize(colliders);
    for (size_t k = 0; k < colliders; ++k) {
        VtVec3fArray points;
        if (!As(in[RigExecClothFirstCollider +
                   k * RigExecClothInputsPerCollider],
                &points))
            return false;
        out->colliders[k] = ToStd(points);
    }
    return true;
}

/// The cloth's compiled rig, as the mover's compile-time assembly builds and
/// saves it for playback. False where the declared inputs do not assemble;
/// an assembled rig that does not compile leaves the default, which records
/// ticks and solves none.
bool
ClothRig(const RigExecRevisionBinding &binding, const std::vector<VtValue> &in,
         rigExecOps::ClothPayload *payload)
{
    const RigExecMoverHandler *handler = binding.handler;
    if (!handler || !handler->assembleExternal || !handler->encodeExternal)
        return false;
    const RigExecProviderValues none;
    const RigExecExternalProviderValues providers(none);
    const RigExecExternalInputContext context{binding, providers, in};
    VtValue data;
    if (!handler->assembleExternal(context, &data))
        return false;
    std::vector<uint8_t> bytes, frame;
    RigExecClothCompiledData compiledRig;
    if (handler->encodeExternal(data, binding, &bytes, &frame) &&
        RigExecDecodeClothCompiledData(bytes.data(), bytes.size(), &compiledRig,
                                       nullptr))
        payload->compiledRig = std::move(compiledRig);
    return true;
}

bool
AddCloth(const RigExecRevisionBinding &binding,
         const RigExecExternalOpValues &values, RigExecRigDescBuilder &builder,
         RigExecExternalOpNode &node, std::string *error)
{
    const std::vector<VtValue> &in = *values.inputs;
    rigExecOps::ClothPayload payload;
    ClothTick tick;
    if (!ReadClothTick(binding, in, &tick) || !ClothRig(binding, in, &payload))
        return Fail(error, "cloth mover inputs do not translate");
    node.node = builder.AddOp(rigExecOps::Cloth(), &payload);
    node.inputs.push_back(
        builder.BindInput(node.node, rigExecOps::ClothParameters()));
    for (size_t k = 0; k < tick.colliders.size(); ++k)
        node.inputs.push_back(
            builder.BindInput(node.node, rigExecOps::ClothColliders()));
    node.points = builder.BindInput(node.node, rigExecOps::ClothPoints());
    node.envelope = builder.BindInput(node.node, rigExecOps::ClothEnvelope());
    node.out = builder.BindOutput(node.node, rigExecOps::ClothOut());
    node.windowed = true;
    node.source = in;
    return true;
}

bool
ClothMoved(const RigExecRevisionBinding &binding,
           const RigExecExternalOpValues &values,
           const RigExecExternalOpNode &node, ClothTick *tick)
{
    return !ReadClothTick(binding, *values.inputs, tick) ||
           node.inputs.size() != 1 + tick->colliders.size() ||
           !SameClothRig(*values.inputs, node.source, tick->colliders.size());
}

bool
WriteCloth(const RigExecRevisionBinding &binding,
           const RigExecExternalOpValues &values,
           const RigExecExternalOpNode &node, RigExecEvaluator &evaluator,
           std::string *error)
{
    ClothTick tick;
    if (ClothMoved(binding, values, node, &tick))
        return Fail(error, "cloth settings changed");
    if (!evaluator.SetInput(node.inputs[0], &tick.parameters, error))
        return false;
    for (size_t k = 0; k < tick.colliders.size(); ++k)
        if (!evaluator.SetInput(node.inputs[1 + k], &tick.colliders[k], error))
            return false;
    return true;
}

bool
ClothStale(const RigExecRevisionBinding &binding,
           const RigExecExternalOpValues &values,
           const RigExecExternalOpNode &node)
{
    ClothTick tick;
    return ReadClothTick(binding, *values.inputs, &tick) &&
           ClothMoved(binding, values, node, &tick);
}

// --- Table ----------------------------------------------------------------

const LegacyBinder kLegacyBinders[] = {
    {SchemaTypeName::RigExecClothMover, &rigExecOps::Cloth, &AddCloth,
     &WriteCloth, &ClothStale},
};

// Written once by RigExecAddExternalOpBinders before `added` is set; read
// only after it is.
std::vector<const RigExecExternalOpBinder *> gAddedBinders;
std::atomic<bool> gAdded{false};

const LegacyBinder *
FindLegacy(const char *schemaType)
{
    for (const LegacyBinder &binder : kLegacyBinders)
        if (std::strcmp(binder.schemaType, schemaType) == 0)
            return &binder;
    return nullptr;
}

const RigExecExternalOpBinder *
FindAdded(const char *schemaType)
{
    if (gAdded.load(std::memory_order_acquire))
        for (const RigExecExternalOpBinder *binder : gAddedBinders)
            if (std::strcmp(binder->schemaType, schemaType) == 0)
                return binder;
    return nullptr;
}

const LegacyBinder *
Legacy(const RigExecRevisionBinding &binding)
{
    return binding.externalSchema.IsEmpty()
               ? nullptr
               : FindLegacy(binding.externalSchema.GetText());
}

const RigExecExternalOpBinder *
FindBuiltin(const char *schemaType)
{
    for (const RigExecExternalOpBinder &binder : kBuiltinBinders)
        if (std::strcmp(binder.schemaType, schemaType) == 0)
            return &binder;
    return nullptr;
}

/// \p binding's built-in binder, or its plugin binder after op plugin
/// discovery.
const RigExecExternalOpBinder *
Planned(const RigExecRevisionBinding &binding)
{
    if (binding.externalSchema.IsEmpty())
        return nullptr;
    if (const RigExecExternalOpBinder *builtin =
            FindBuiltin(binding.externalSchema.GetText()))
        return builtin;
    if (!RigExecEnsureSceneOpRegistry(nullptr))
        return nullptr;
    return FindAdded(binding.externalSchema.GetText());
}

/// \p value as an input of \p type: written to \p id, or false.
bool
SetValue(RigExecEvaluator &evaluator, const RigExecInputId &id,
         const RigExecTypeHandle *type, const VtValue &value,
         std::string *error)
{
    const auto set = [&](const auto &v) {
        return evaluator.SetInput(id, &v, error);
    };
    const auto vec3fs = [&](const VtVec3fArray &a) { return set(ToStd(a)); };
    if (type == &rigExecStd::BoolType() && value.IsHolding<bool>())
        return set(uint8_t(value.UncheckedGet<bool>() ? 1 : 0));
    if (type == &rigExecStd::IntType() && value.IsHolding<int>())
        return set(int32_t(value.UncheckedGet<int>()));
    if (type == &rigExecStd::FloatType() && value.IsHolding<float>())
        return set(value.UncheckedGet<float>());
    double scalar = 0.0;
    if (type == &rigExecStd::DoubleType() && Scalar(value, &scalar))
        return set(scalar);
    if (type == &rigExecStd::Vec3fType() && value.IsHolding<GfVec3f>()) {
        const GfVec3f &v = value.UncheckedGet<GfVec3f>();
        return set(Vec3f(v[0], v[1], v[2]));
    }
    if (type == &rigExecStd::Mat4dType() && value.IsHolding<GfMatrix4d>())
        return set(ToStd(value.UncheckedGet<GfMatrix4d>()));
    if (type == &rigExecStd::IntArray() && value.IsHolding<VtIntArray>())
        return set(ToArray(value.UncheckedGet<VtIntArray>()));
    if (type == &rigExecStd::FloatArray() && value.IsHolding<VtFloatArray>())
        return set(ToArray(value.UncheckedGet<VtFloatArray>()));
    if (type == &rigExecStd::DoubleArray() && value.IsHolding<VtDoubleArray>())
        return set(ToArray(value.UncheckedGet<VtDoubleArray>()));
    if (type == &rigExecStd::Vec3fArray() && value.IsHolding<VtVec3fArray>())
        return vec3fs(value.UncheckedGet<VtVec3fArray>());
    return Fail(error, "a declared input does not hold its op input's type");
}

/// Declared inputs [index, index + count) as one array of \p type: scalars
/// listed, arrays joined end to end.
bool
SetValues(RigExecEvaluator &evaluator, const RigExecInputId &id,
          const RigExecTypeHandle *type, const std::vector<VtValue> &inputs,
          size_t index, size_t count, std::string *error)
{
    if (index + count > inputs.size())
        return Fail(error, "a declared input the mover lacks");
    if (type == &rigExecStd::DoubleArray()) {
        RigExecArray<double> out(count);
        for (size_t k = 0; k < count; ++k)
            if (!Scalar(inputs[index + k], &out[k]))
                return Fail(error, "a declared input is not a number");
        return evaluator.SetInput(id, &out, error);
    }
    if (type == &rigExecStd::Vec3fArray()) {
        std::vector<Vec3f> joined;
        for (size_t k = 0; k < count; ++k) {
            const VtValue &value = inputs[index + k];
            if (!value.IsHolding<VtVec3fArray>())
                return Fail(error, "a declared input is not a point array");
            for (const GfVec3f &p : value.UncheckedGet<VtVec3fArray>())
                joined.emplace_back(p[0], p[1], p[2]);
        }
        const RigExecArray<Vec3f> out(joined.data(), joined.size());
        return evaluator.SetInput(id, &out, error);
    }
    return Fail(error, "declared inputs do not join into their op input");
}

/// Writes one frame of \p node's feeds.
bool
WritePlanned(const RigExecExternalOpValues &values,
             const std::vector<GfVec3f> &points,
             const RigExecExternalOpNode &node, RigExecEvaluator &evaluator,
             std::string *error)
{
    const RigExecOpDesc &desc = *RigExecRegistry::Get().Op(*node.op).desc;
    for (size_t i = 0; i < node.plan.feeds.size(); ++i) {
        const RigExecExternalOpFeed &feed = node.plan.feeds[i];
        const RigExecInputId &id = node.inputs[i];
        bool ok = false;
        switch (feed.source) {
        case RigExecExternalOpSource::Points: {
            const RigExecArray<Vec3f> in = ToStd(points.data(), points.size());
            ok = evaluator.SetInput(id, &in, error);
            break;
        }
        case RigExecExternalOpSource::BasePoints: {
            const std::vector<GfVec3f> &base =
                values.basePoints ? *values.basePoints : points;
            const RigExecArray<Vec3f> in = ToStd(base.data(), base.size());
            ok = evaluator.SetInput(id, &in, error);
            break;
        }
        case RigExecExternalOpSource::Input:
            if (!values.inputs || feed.index >= values.inputs->size())
                return Fail(error, "a declared input the mover lacks");
            ok = feed.count
                     ? SetValues(evaluator, id,
                                 desc.ports[feed.port->Index()].type,
                                 *values.inputs, feed.index, feed.count, error)
                     : SetValue(evaluator, id,
                                desc.ports[feed.port->Index()].type,
                                (*values.inputs)[feed.index], error);
            break;
        case RigExecExternalOpSource::Transform:
            if (!values.transform)
                return Fail(error, "no transform");
            ok = SetValue(evaluator, id, &rigExecStd::Mat4dType(),
                          VtValue(*values.transform), error);
            break;
        case RigExecExternalOpSource::Influence: {
            const size_t count = feed.count ? feed.count : 1;
            if (!values.influenceTransforms ||
                feed.index + count > values.influenceTransforms->size())
                return Fail(error, "an influence the mover lacks");
            if (!feed.count) {
                ok = SetValue(
                    evaluator, id, &rigExecStd::Mat4dType(),
                    VtValue((*values.influenceTransforms)[feed.index]), error);
                break;
            }
            RigExecArray<Mat4d> list(count);
            for (size_t k = 0; k < count; ++k)
                list[k] = ToStd((*values.influenceTransforms)[feed.index + k]);
            ok = evaluator.SetInput(id, &list, error);
            break;
        }
        }
        if (!ok)
            return false;
    }
    return true;
}

} // namespace

bool
RigExecAddExternalOpBinders(
    const std::vector<const RigExecExternalOpBinder *> &binders,
    std::string *error)
{
    if (gAdded.load(std::memory_order_acquire))
        return Fail(error, "op binders were already added");
    for (size_t i = 0; i < binders.size(); ++i) {
        const RigExecExternalOpBinder *binder = binders[i];
        if (!binder || binder->structSize != sizeof(RigExecExternalOpBinder) ||
            !binder->schemaType || !binder->op || !binder->build)
            return Fail(error, "an incomplete op binder");
        bool taken = FindLegacy(binder->schemaType) != nullptr ||
                     FindBuiltin(binder->schemaType) != nullptr;
        for (size_t j = 0; j < i && !taken; ++j)
            taken =
                std::strcmp(binders[j]->schemaType, binder->schemaType) == 0;
        if (taken)
            return Fail(error, std::string("two op binders for ") +
                                   binder->schemaType);
    }
    gAddedBinders = binders;
    gAdded.store(true, std::memory_order_release);
    return true;
}

bool
RigExecBindsExternalOp(const RigExecRevisionBinding &binding)
{
    return Planned(binding) != nullptr;
}

bool
RigExecBuildExternalOp(const RigExecExternalOpContext &context,
                       RigExecRigDescBuilder &builder,
                       RigExecExternalOpPlan *plan, std::string *error)
{
    const RigExecRevisionBinding &binding = context.binding;
    const RigExecExternalOpBinder *binder = Planned(binding);
    if (!binder)
        return Fail(error, binding.moverPath.GetString() +
                               ": no op binder binds its mover");
    RigExecExternalOpPlan built;
    if (!binder->build(context, builder, built, error))
        return false;
    if (built.node == kRigExecInvalidIndex || !built.out)
        return Fail(error, binding.moverPath.GetString() +
                               ": its binder named no node or output");
    for (const RigExecExternalOpFeed &feed : built.feeds)
        if (!feed.port)
            return Fail(error, binding.moverPath.GetString() +
                                   ": its binder feeds no port");
    built.op = &binder->op();
    *plan = std::move(built);
    return true;
}

const RigExecOpHandle *
RigExecExternalOpHandle(const RigExecRevisionBinding &binding)
{
    if (const LegacyBinder *legacy = Legacy(binding))
        return &legacy->op();
    const RigExecExternalOpBinder *planned = Planned(binding);
    return planned ? &planned->op() : nullptr;
}

bool
RigExecAddExternalOp(const RigExecRevisionBinding &binding,
                     const RigExecExternalOpValues &values,
                     RigExecRigDescBuilder *builder,
                     RigExecExternalOpNode *node, std::string *error)
{
    if (!builder || !node || !values.inputs)
        return Fail(error, "null external op destination or inputs");
    RigExecExternalOpNode result;
    if (const LegacyBinder *op = Legacy(binding)) {
        result.op = &op->op();
        if (!op->add(binding, values, *builder, result, error))
            return false;
        *node = std::move(result);
        return true;
    }
    const RigExecExternalOpBinder *planned = Planned(binding);
    if (!planned)
        return Fail(error, "mover has no op");
    result.op = &planned->op();
    result.planned = true;
    result.source = *values.inputs;
    if (!RigExecBuildExternalOp({binding, result.source}, *builder,
                                &result.plan, error))
        return false;
    result.node = result.plan.node;
    for (const RigExecExternalOpFeed &feed : result.plan.feeds)
        result.inputs.push_back(builder->BindInput(result.node, *feed.port));
    result.out = builder->BindOutput(result.node, *result.plan.out);
    *node = std::move(result);
    return true;
}

bool
RigExecWriteExternalOpInputs(const RigExecRevisionBinding &binding,
                             const RigExecExternalOpValues &values,
                             const RigExecWeightPacket &weights,
                             const std::vector<GfVec3f> &points,
                             const RigExecExternalOpNode &node,
                             RigExecEvaluator *evaluator, std::string *error)
{
    if (!evaluator || !values.inputs)
        return Fail(error, "null evaluator or inputs");
    if (node.planned)
        return !RigExecExternalOpNodeStale(binding, values, node)
                   ? WritePlanned(values, points, node, *evaluator, error)
                   : Fail(error, "the mover's op settings changed");
    const LegacyBinder *op = Legacy(binding);
    if (!op || node.op != &op->op())
        return Fail(error, "node is not this mover's op");
    if (!op->write(binding, values, node, *evaluator, error))
        return false;
    std::vector<float> resolved;
    if (!weights.ResolveAll(points.size(), &resolved))
        return Fail(error, "envelope does not resolve over the points");
    const RigExecArray<float> envelope(resolved.data(), resolved.size());
    const RigExecArray<Vec3f> in = ToStd(points.data(), points.size());
    return (!Valid(node.envelope) ||
            evaluator->SetInput(node.envelope, &envelope, error)) &&
           evaluator->SetInput(node.points, &in, error);
}

bool
RigExecExternalOpNodeStale(const RigExecRevisionBinding &binding,
                           const RigExecExternalOpValues &values,
                           const RigExecExternalOpNode &node)
{
    if (node.planned) {
        if (node.op != RigExecExternalOpHandle(binding) || !values.inputs)
            return true;
        for (const uint32_t k : node.plan.pins)
            if (k >= values.inputs->size() || k >= node.source.size() ||
                (*values.inputs)[k] != node.source[k])
                return true;
        // The binding's structure (its influences, say) can move the feeds.
        RigExecRigDescBuilder scratch;
        RigExecExternalOpPlan plan;
        if (!RigExecBuildExternalOp({binding, *values.inputs}, scratch, &plan,
                                    nullptr))
            return true;
        const auto same = [](const RigExecExternalOpFeed &a,
                             const RigExecExternalOpFeed &b) {
            return a.port == b.port && a.source == b.source &&
                   a.index == b.index && a.count == b.count;
        };
        return !std::equal(plan.feeds.begin(), plan.feeds.end(),
                           node.plan.feeds.begin(), node.plan.feeds.end(),
                           same);
    }
    const LegacyBinder *op = Legacy(binding);
    if (!op || node.op != &op->op())
        return true;
    return values.inputs && op->stale(binding, values, node);
}

} // namespace rigExec
