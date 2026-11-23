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
#include "rigExecScene/patchInputs.h"
#include "rigExecScene/surfaceBindingInputs.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"

#include <cstring>
#include <iterator>

namespace rigExec {

namespace {

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

// --- Patch ---------------------------------------------------------------

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

struct PatchValues {
    rigExecOps::PatchPayload payload;
    RigExecArray<Vec3f> controls, uvPoints;
    RigExecArray<double> pointWeights, uKnots, vKnots, weights;
    std::vector<RigExecArray<Vec3f>> shapes;
    RigExecArray<int> lower, upper;
};

/// The patch mover's declared inputs as the op's payload and inputs.
bool
ReadPatch(const RigExecRevisionBinding &binding,
          const RigExecExternalOpValues &values, PatchValues *out)
{
    using namespace patchInputs;
    const std::vector<VtValue> &in = *values.inputs;
    const Layout layout = LayoutOf(binding.externalInputs);
    if (in.size() != InputCount(layout))
        return false;
    VtVec3fArray controls, uvPoints;
    VtDoubleArray pointWeights, uKnots, vKnots;
    rigExecOps::PatchPayload &p = out->payload;
    if (!As(in[PatchPoints], &controls) ||
        !As(in[PatchPointWeights], &pointWeights) ||
        !As(in[PatchUVertexCount], &p.uVertexCount) ||
        !As(in[PatchVVertexCount], &p.vVertexCount) ||
        !As(in[PatchUOrder], &p.uOrder) || !As(in[PatchVOrder], &p.vOrder) ||
        !As(in[PatchUKnots], &uKnots) || !As(in[PatchVKnots], &vKnots) ||
        !Periodic(in[PatchUForm], &p.uPeriodic) ||
        !Periodic(in[PatchVForm], &p.vPeriodic) || !As(in[UvPoints], &uvPoints))
        return false;
    out->controls = ToStd(controls);
    out->uvPoints = ToStd(uvPoints);
    out->pointWeights = ToArray(pointWeights);
    out->uKnots = ToArray(uKnots);
    out->vKnots = ToArray(vKnots);
    out->shapes.resize(layout.shapes);
    for (size_t k = 0; k < layout.shapes; ++k) {
        VtVec3fArray shape;
        if (!As(in[UvShapes + k], &shape))
            return false;
        out->shapes[k] = ToStd(shape);
    }
    // One weight per shape; an unconnected, empty array leaves every shape
    // at zero.
    const size_t weightsBegin = WeightsBegin(layout);
    if (layout.connected) {
        out->weights = RigExecArray<double>(layout.shapes);
        for (size_t k = 0; k < layout.shapes; ++k)
            if (!Scalar(in[weightsBegin + k], &out->weights[k]))
                return false;
    } else {
        VtFloatArray authored;
        if (!As(in[weightsBegin], &authored))
            return false;
        out->weights = RigExecArray<double>(authored.empty() ? layout.shapes
                                                             : authored.size());
        for (size_t k = 0; k < out->weights.Size(); ++k)
            out->weights[k] = authored.empty() ? 0.0 : double(authored[k]);
    }
    VtIntArray lower, upper;
    const size_t contactsBegin = ContactsBegin(layout);
    if (!As(in[contactsBegin], &lower) || !As(in[contactsBegin + 1], &upper))
        return false;
    out->lower = ToArray(lower);
    out->upper = ToArray(upper);
    return true;
}

/// The patch's other inputs, in the order the binding writes them; the
/// shapes, which take many values, write one each.
const RigExecPortHandle &(*const kPatchLeading[])() = {
    &rigExecOps::PatchControlPoints, &rigExecOps::PatchPointWeights,
    &rigExecOps::PatchUKnots, &rigExecOps::PatchVKnots,
    &rigExecOps::PatchUvPoints};
const RigExecPortHandle &(*const kPatchTrailing[])() = {
    &rigExecOps::PatchUvWeights, &rigExecOps::PatchContactLower,
    &rigExecOps::PatchContactUpper};

bool
AddPatch(const RigExecRevisionBinding &binding,
         const RigExecExternalOpValues &values, RigExecRigDescBuilder &builder,
         RigExecExternalOpNode &node, std::string *error)
{
    PatchValues v;
    if (!ReadPatch(binding, values, &v))
        return Fail(error, "patch mover inputs do not translate");
    node.node = builder.AddOp(rigExecOps::Patch(), &v.payload);
    for (const auto port : kPatchLeading)
        node.inputs.push_back(builder.BindInput(node.node, port()));
    for (size_t k = 0; k < v.shapes.size(); ++k)
        node.inputs.push_back(
            builder.BindInput(node.node, rigExecOps::PatchUvShapes()));
    for (const auto port : kPatchTrailing)
        node.inputs.push_back(builder.BindInput(node.node, port()));
    node.points = builder.BindInput(node.node, rigExecOps::PatchPoints());
    node.envelope = builder.BindInput(node.node, rigExecOps::PatchEnvelope());
    node.out = builder.BindOutput(node.node, rigExecOps::PatchOut());
    node.payload = std::make_shared<rigExecOps::PatchPayload>(v.payload);
    return true;
}

/// Translates \p values; false where they do not, and with \p moved set
/// where they translate to another payload or shape count than \p node's.
bool
ReadPatchFor(const RigExecRevisionBinding &binding,
             const RigExecExternalOpValues &values,
             const RigExecExternalOpNode &node, PatchValues *v, bool *moved)
{
    if (!ReadPatch(binding, values, v))
        return false;
    const auto *kept =
        static_cast<const rigExecOps::PatchPayload *>(node.payload.get());
    *moved = !kept || !(*kept == v->payload) ||
             node.inputs.size() != std::size(kPatchLeading) + v->shapes.size() +
                                       std::size(kPatchTrailing);
    return true;
}

bool
WritePatch(const RigExecRevisionBinding &binding,
           const RigExecExternalOpValues &values,
           const RigExecExternalOpNode &node, RigExecEvaluator &evaluator,
           std::string *error)
{
    PatchValues v;
    bool moved = false;
    if (!ReadPatchFor(binding, values, node, &v, &moved))
        return Fail(error, "patch mover inputs do not translate");
    if (moved)
        return Fail(error, "patch mover settings changed");
    const void *leading[] = {&v.controls, &v.pointWeights, &v.uKnots, &v.vKnots,
                             &v.uvPoints};
    const void *trailing[] = {&v.weights, &v.lower, &v.upper};
    size_t i = 0;
    for (const void *value : leading)
        if (!evaluator.SetInput(node.inputs[i++], value, error))
            return false;
    for (const RigExecArray<Vec3f> &shape : v.shapes)
        if (!evaluator.SetInput(node.inputs[i++], &shape, error))
            return false;
    for (const void *value : trailing)
        if (!evaluator.SetInput(node.inputs[i++], value, error))
            return false;
    return true;
}

bool
PatchStale(const RigExecRevisionBinding &binding,
           const RigExecExternalOpValues &values,
           const RigExecExternalOpNode &node)
{
    PatchValues v;
    bool moved = false;
    return ReadPatchFor(binding, values, node, &v, &moved) && moved;
}

// --- SurfaceBinding --------------------------------------------------------

Mat4d
ToStd(const GfMatrix4d &m)
{
    Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out._mtx[r][c] = m[r][c];
    return out;
}

struct SurfaceBindingValues {
    rigExecOps::SurfaceBindingPayload payload;
    RigExecArray<int> ints[surfaceBindingInputs::kIntArrays];
    RigExecArray<float> floats[surfaceBindingInputs::kFloatArrays];
    RigExecArray<Vec3f> offsets, surface;
    float strength = 0.0f, deltaMultiplier = 0.0f;
    Mat4d surfaceRest, targetRest, surfaceToBinding;
    RigExecArray<Mat4d> frames;
    RigExecArray<int> counts, indices;
};

/// The surface binding mover's declared inputs and frames as the op's
/// payload and inputs.
bool
ReadSurfaceBinding(const RigExecRevisionBinding &,
                   const RigExecExternalOpValues &values,
                   SurfaceBindingValues *out)
{
    using namespace surfaceBindingInputs;
    const std::vector<VtValue> &in = *values.inputs;
    if (in.size() != InputCount)
        return false;
    for (size_t i = 0; i < kIntArrays; ++i) {
        VtIntArray a;
        if (!As(in[FirstInt + i], &a))
            return false;
        out->ints[i] = ToArray(a);
    }
    for (size_t i = 0; i < kFloatArrays; ++i) {
        VtFloatArray a;
        if (!As(in[FirstFloat + i], &a))
            return false;
        out->floats[i] = ToArray(a);
    }
    VtVec3fArray offsets, surface;
    const char *mode = nullptr, *normal = nullptr;
    GfMatrix4d surfaceRest, targetRest, surfaceToBinding;
    if (!As(in[OffsetVectors], &offsets) || !Text(in[SurfaceMode], &mode) ||
        !Text(in[NormalMode], &normal) || !As(in[Strength], &out->strength) ||
        !As(in[DeltaMultiplier], &out->deltaMultiplier) ||
        (!Is(mode, "polygon") && !Is(mode, "limit")) ||
        (!Is(normal, "geometric") && !Is(normal, "smooth")) ||
        !As(in[SurfacePoints], &surface) ||
        !As(in[SurfaceRestMatrix], &surfaceRest) ||
        !As(in[TargetRestMatrix], &targetRest) ||
        !As(in[SurfaceToBinding], &surfaceToBinding))
        return false;
    rigExecOps::SurfaceBindingPayload &p = out->payload;
    p.limit = Is(mode, "limit");
    p.smooth = Is(normal, "smooth");
    p.framed = values.influenceTransforms != nullptr;
    out->offsets = ToStd(offsets);
    out->surface = ToStd(surface);
    out->surfaceRest = ToStd(surfaceRest);
    out->targetRest = ToStd(targetRest);
    out->surfaceToBinding = ToStd(surfaceToBinding);
    if (p.framed) {
        const std::vector<GfMatrix4d> &frames = *values.influenceTransforms;
        out->frames = RigExecArray<Mat4d>(frames.size());
        for (size_t k = 0; k < frames.size(); ++k)
            out->frames[k] = ToStd(frames[k]);
    }
    // Faces are read for smooth normals only.
    if (p.smooth) {
        VtIntArray counts, indices;
        if (!As(in[SurfaceCounts], &counts) ||
            !As(in[SurfaceIndices], &indices))
            return false;
        out->counts = ToArray(counts);
        out->indices = ToArray(indices);
    }
    return true;
}

/// The binding's inputs, in the order the binding writes them; Frames
/// last, written when framed.
const RigExecPortHandle &(*const kSurfaceBindingPorts[])() = {
    &rigExecOps::SurfaceBindingVertices,
    &rigExecOps::SurfaceBindingVertexOffsets,
    &rigExecOps::SurfaceBindingPolygonOffsets,
    &rigExecOps::SurfaceBindingPointIndices,
    &rigExecOps::SurfaceBindingLimitOffsets,
    &rigExecOps::SurfaceBindingLimitIndices,
    &rigExecOps::SurfaceBindingBarycentricWeights,
    &rigExecOps::SurfaceBindingBindingWeights,
    &rigExecOps::SurfaceBindingMask,
    &rigExecOps::SurfaceBindingDeltaMultipliers,
    &rigExecOps::SurfaceBindingLimitWeights,
    &rigExecOps::SurfaceBindingLimitDuWeights,
    &rigExecOps::SurfaceBindingLimitDvWeights,
    &rigExecOps::SurfaceBindingOffsets,
    &rigExecOps::SurfaceBindingStrength,
    &rigExecOps::SurfaceBindingDeltaMultiplier,
    &rigExecOps::SurfaceBindingSurfaceRest,
    &rigExecOps::SurfaceBindingTargetRest,
    &rigExecOps::SurfaceBindingSurfaceToBinding,
    &rigExecOps::SurfaceBindingSurface,
    &rigExecOps::SurfaceBindingCounts,
    &rigExecOps::SurfaceBindingIndices};

bool
AddSurfaceBinding(const RigExecRevisionBinding &binding,
                  const RigExecExternalOpValues &values,
                  RigExecRigDescBuilder &builder, RigExecExternalOpNode &node,
                  std::string *error)
{
    SurfaceBindingValues v;
    if (!ReadSurfaceBinding(binding, values, &v))
        return Fail(error, "surface binding inputs do not translate");
    node.node = builder.AddOp(rigExecOps::SurfaceBinding(), &v.payload);
    for (const auto port : kSurfaceBindingPorts)
        node.inputs.push_back(builder.BindInput(node.node, port()));
    if (v.payload.framed)
        node.inputs.push_back(
            builder.BindInput(node.node, rigExecOps::SurfaceBindingFrames()));
    node.points =
        builder.BindInput(node.node, rigExecOps::SurfaceBindingPoints());
    node.envelope =
        builder.BindInput(node.node, rigExecOps::SurfaceBindingEnvelope());
    node.out = builder.BindOutput(node.node, rigExecOps::SurfaceBindingOut());
    node.payload =
        std::make_shared<rigExecOps::SurfaceBindingPayload>(v.payload);
    return true;
}

bool
ReadSurfaceBindingFor(const RigExecRevisionBinding &binding,
                      const RigExecExternalOpValues &values,
                      const RigExecExternalOpNode &node,
                      SurfaceBindingValues *v, bool *moved)
{
    if (!ReadSurfaceBinding(binding, values, v))
        return false;
    const auto *kept = static_cast<const rigExecOps::SurfaceBindingPayload *>(
        node.payload.get());
    *moved = !kept || !(*kept == v->payload);
    return true;
}

bool
WriteSurfaceBinding(const RigExecRevisionBinding &binding,
                    const RigExecExternalOpValues &values,
                    const RigExecExternalOpNode &node,
                    RigExecEvaluator &evaluator, std::string *error)
{
    SurfaceBindingValues v;
    bool moved = false;
    if (!ReadSurfaceBindingFor(binding, values, node, &v, &moved))
        return Fail(error, "surface binding inputs do not translate");
    if (moved)
        return Fail(error, "surface binding settings changed");
    std::vector<const void *> ordered;
    for (const RigExecArray<int> &a : v.ints)
        ordered.push_back(&a);
    for (const RigExecArray<float> &a : v.floats)
        ordered.push_back(&a);
    for (const void *value : {static_cast<const void *>(&v.offsets),
                              static_cast<const void *>(&v.strength),
                              static_cast<const void *>(&v.deltaMultiplier),
                              static_cast<const void *>(&v.surfaceRest),
                              static_cast<const void *>(&v.targetRest),
                              static_cast<const void *>(&v.surfaceToBinding),
                              static_cast<const void *>(&v.surface),
                              static_cast<const void *>(&v.counts),
                              static_cast<const void *>(&v.indices)})
        ordered.push_back(value);
    if (v.payload.framed)
        ordered.push_back(&v.frames);
    if (ordered.size() != node.inputs.size())
        return Fail(error, "surface binding inputs changed");
    for (size_t i = 0; i < ordered.size(); ++i)
        if (!evaluator.SetInput(node.inputs[i], ordered[i], error))
            return false;
    return true;
}

bool
SurfaceBindingStale(const RigExecRevisionBinding &binding,
                    const RigExecExternalOpValues &values,
                    const RigExecExternalOpNode &node)
{
    SurfaceBindingValues v;
    bool moved = false;
    return ReadSurfaceBindingFor(binding, values, node, &v, &moved) && moved;
}

// --- LayeredSkin -----------------------------------------------------------

struct LayeredSkinValues {
    rigExecOps::LayeredSkinPayload payload;
    RigExecArray<Mat4d> transforms;
    RigExecArray<int> indices;
    RigExecArray<float> weights, mask;
    RigExecArray<Vec3f> base;
    Mat4d inputTransform;
};

/// The layered skin mover's declared inputs, influences and transform as
/// the op's payload and inputs.
bool
ReadLayeredSkin(const RigExecRevisionBinding &,
                const RigExecExternalOpValues &values, LayeredSkinValues *out)
{
    using namespace layeredSkinInputs;
    const std::vector<VtValue> &in = *values.inputs;
    if (in.size() != kInputCount || !values.influenceTransforms ||
        values.influenceTransforms->empty())
        return false;
    VtIntArray indices;
    VtFloatArray weights, mask;
    const char *method = nullptr;
    bool fromBase = false, transformInput = false;
    rigExecOps::LayeredSkinPayload &p = out->payload;
    if (!As(in[kJointIndices], &indices) || !As(in[kJointWeights], &weights) ||
        !As(in[kElementSize], &p.elementSize) ||
        !Text(in[kSkinningMethod], &method) || !As(in[kMask], &mask) ||
        !As(in[kUseBaseInput], &fromBase) ||
        !As(in[kTransformInput], &transformInput))
        return false;
    if ((transformInput && !values.transform) ||
        (!Is(method, "classicLinear") && !Is(method, "dualQuaternion")))
        return false;
    p.dualQuaternion = Is(method, "dualQuaternion");
    const std::vector<GfMatrix4d> &transforms = *values.influenceTransforms;
    out->transforms = RigExecArray<Mat4d>(transforms.size());
    for (size_t k = 0; k < transforms.size(); ++k)
        out->transforms[k] = ToStd(transforms[k]);
    out->indices = ToArray(indices);
    out->weights = ToArray(weights);
    out->mask = ToArray(mask);
    if (fromBase && values.basePoints)
        out->base = ToStd(values.basePoints->data(), values.basePoints->size());
    out->inputTransform =
        ToStd(transformInput ? *values.transform : GfMatrix4d(1));
    return true;
}

const RigExecPortHandle &(*const kLayeredSkinPorts[])() = {
    &rigExecOps::LayeredSkinTransforms, &rigExecOps::LayeredSkinIndices,
    &rigExecOps::LayeredSkinWeights,    &rigExecOps::LayeredSkinMask,
    &rigExecOps::LayeredSkinBase,       &rigExecOps::LayeredSkinInputTransform};

bool
AddLayeredSkin(const RigExecRevisionBinding &binding,
               const RigExecExternalOpValues &values,
               RigExecRigDescBuilder &builder, RigExecExternalOpNode &node,
               std::string *error)
{
    LayeredSkinValues v;
    if (!ReadLayeredSkin(binding, values, &v))
        return Fail(error, "layered skin inputs do not translate");
    node.node = builder.AddOp(rigExecOps::LayeredSkin(), &v.payload);
    for (const auto port : kLayeredSkinPorts)
        node.inputs.push_back(builder.BindInput(node.node, port()));
    node.points = builder.BindInput(node.node, rigExecOps::LayeredSkinPoints());
    node.envelope =
        builder.BindInput(node.node, rigExecOps::LayeredSkinEnvelope());
    node.out = builder.BindOutput(node.node, rigExecOps::LayeredSkinOut());
    node.payload = std::make_shared<rigExecOps::LayeredSkinPayload>(v.payload);
    return true;
}

bool
ReadLayeredSkinFor(const RigExecRevisionBinding &binding,
                   const RigExecExternalOpValues &values,
                   const RigExecExternalOpNode &node, LayeredSkinValues *v,
                   bool *moved)
{
    if (!ReadLayeredSkin(binding, values, v))
        return false;
    const auto *kept =
        static_cast<const rigExecOps::LayeredSkinPayload *>(node.payload.get());
    *moved = !kept || !(*kept == v->payload);
    return true;
}

bool
WriteLayeredSkin(const RigExecRevisionBinding &binding,
                 const RigExecExternalOpValues &values,
                 const RigExecExternalOpNode &node, RigExecEvaluator &evaluator,
                 std::string *error)
{
    LayeredSkinValues v;
    bool moved = false;
    if (!ReadLayeredSkinFor(binding, values, node, &v, &moved))
        return Fail(error, "layered skin inputs do not translate");
    if (moved)
        return Fail(error, "layered skin settings changed");
    const void *ordered[] = {&v.transforms, &v.indices, &v.weights,
                             &v.mask,       &v.base,    &v.inputTransform};
    for (size_t i = 0; i < std::size(ordered); ++i)
        if (!evaluator.SetInput(node.inputs[i], ordered[i], error))
            return false;
    return true;
}

bool
LayeredSkinStale(const RigExecRevisionBinding &binding,
                 const RigExecExternalOpValues &values,
                 const RigExecExternalOpNode &node)
{
    LayeredSkinValues v;
    bool moved = false;
    return ReadLayeredSkinFor(binding, values, node, &v, &moved) && moved;
}

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
    node.status = builder.BindOutput(node.node, rigExecOps::ClothStatusOut());
    node.solved = uint32_t(rigExecOps::ClothStatus::Solved);
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

/// One built-in External mover: its schema, its op, and how its declared
/// inputs and provider values become the op's payload and inputs.
struct ExternalOp {
    const TfToken *schema;
    const RigExecOpHandle &(*op)();
    bool (*add)(const RigExecRevisionBinding &, const RigExecExternalOpValues &,
                RigExecRigDescBuilder &, RigExecExternalOpNode &,
                std::string *);
    bool (*write)(const RigExecRevisionBinding &,
                  const RigExecExternalOpValues &,
                  const RigExecExternalOpNode &, RigExecEvaluator &,
                  std::string *);
    bool (*stale)(const RigExecRevisionBinding &,
                  const RigExecExternalOpValues &,
                  const RigExecExternalOpNode &);
};

const ExternalOp kExternalOps[] = {
    {&SchemaType::RigExecPatchMover, &rigExecOps::Patch, &AddPatch, &WritePatch,
     &PatchStale},
    {&SchemaType::RigExecSurfaceBindingMover, &rigExecOps::SurfaceBinding,
     &AddSurfaceBinding, &WriteSurfaceBinding, &SurfaceBindingStale},
    {&SchemaType::RigExecLayeredSkinMover, &rigExecOps::LayeredSkin,
     &AddLayeredSkin, &WriteLayeredSkin, &LayeredSkinStale},
    {&SchemaType::RigExecClothMover, &rigExecOps::Cloth, &AddCloth, &WriteCloth,
     &ClothStale},
};

const ExternalOp *
Find(const RigExecRevisionBinding &binding)
{
    for (const ExternalOp &op : kExternalOps)
        if (binding.externalSchema == *op.schema)
            return &op;
    return nullptr;
}

} // namespace

const RigExecOpHandle *
RigExecExternalOpHandle(const RigExecRevisionBinding &binding)
{
    const ExternalOp *op = Find(binding);
    return op ? &op->op() : nullptr;
}

bool
RigExecAddExternalOp(const RigExecRevisionBinding &binding,
                     const RigExecExternalOpValues &values,
                     RigExecRigDescBuilder *builder,
                     RigExecExternalOpNode *node, std::string *error)
{
    const ExternalOp *op = Find(binding);
    if (!op)
        return Fail(error, "mover has no op");
    if (!builder || !node || !values.inputs)
        return Fail(error, "null external op destination or inputs");
    RigExecExternalOpNode result;
    result.op = &op->op();
    if (!op->add(binding, values, *builder, result, error))
        return false;
    *node = std::move(result);
    return true;
}

bool
RigExecWriteExternalOpInputs(const RigExecRevisionBinding &binding,
                             const RigExecExternalOpValues &values,
                             const RigExecWeightPacket &weights,
                             const std::vector<GfVec3f> &points,
                             const RigExecExternalOpNode &node,
                             RigExecEvaluator *evaluator, std::string *error,
                             bool *envelopeResolved)
{
    const ExternalOp *op = Find(binding);
    if (!op || node.op != &op->op())
        return Fail(error, "node is not this mover's op");
    if (!evaluator || !values.inputs)
        return Fail(error, "null evaluator or inputs");
    if (!op->write(binding, values, node, *evaluator, error))
        return false;
    std::vector<float> resolved;
    const bool resolves = weights.ResolveAll(points.size(), &resolved);
    if (envelopeResolved)
        *envelopeResolved = resolves;
    if (!resolves) {
        if (!node.windowed)
            return Fail(error, "envelope does not resolve over the points");
        // The tick still records; the run's result is refused.
        resolved.assign(points.size(), 1.0f);
    }
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
    const ExternalOp *op = Find(binding);
    if (!op || node.op != &op->op())
        return true;
    return values.inputs && op->stale(binding, values, node);
}

} // namespace rigExec
