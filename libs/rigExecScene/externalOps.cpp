#include "rigExecScene/externalOps.h"

#include "rigExec/api/registry.h"
#include "rigExecOps/deform/patchOp.h"
#include "rigExecSchema/schemaTokens.h"
#include "rigExecScene/patchInputs.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"

#include <cstring>
#include <iterator>

namespace rigExec {

namespace {

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
                             RigExecEvaluator *evaluator, std::string *error)
{
    const ExternalOp *op = Find(binding);
    if (!op || node.op != &op->op())
        return Fail(error, "node is not this mover's op");
    if (!evaluator || !values.inputs)
        return Fail(error, "null evaluator or inputs");
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
    const ExternalOp *op = Find(binding);
    if (!op || node.op != &op->op())
        return true;
    return values.inputs && op->stale(binding, values, node);
}

} // namespace rigExec
