// The fixture mover as an op plugin: the Quadratic op deforms with the
// mover's own kernel (kernel.h), and its binder turns the mover's declared
// inputs into the op's, as the mover's Assemble reads them.
#include "kernel.h"
#include "rigExec/api/opPorts.h"
#include "rigExec/api/registry.h"
#include "rigExecMath/envelope.h"
#include "rigExecScene/externalOps.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"

#include <cmath>
#include <iterator>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;
using Envelope = RigExecArray<float>;

RIGEXEC_DEFINE_OP_HANDLE(Quadratic)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticPoints)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticGain)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticReferenceY)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(QuadraticOut)

bool
Fail(std::string *error, const char *why)
{
    if (error)
        *error = std::string("Quadratic: ") + why;
    return false;
}

bool
Run(const void *, void *, const rigExec::RigExecOpIo &io, std::string *error)
{
    using rigExec::opPorts::In;
    using rigExec::opPorts::Out;
    const Points &in = *In<Points>(io, QuadraticPoints());
    const float gain = *In<float>(io, QuadraticGain());
    // The mover's payload sentinel for a result of another point count.
    if (gain == -2.0f)
        return Fail(error, "the result changes the point count");
    std::vector<Vec3f> points(in.begin(), in.end());
    if (!rigExec::TestQuadraticDefinition::Deform(
            gain, *In<float>(io, QuadraticReferenceY()),
            points.empty() ? nullptr : &points[0][0], points.size()))
        return Fail(error, "a negative gain");
    for (const Vec3f &p : points)
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
            !std::isfinite(p[2]))
            return Fail(error, "a point is not finite");
    const Envelope *envelope = In<Envelope>(io, QuadraticEnvelope());
    if (envelope && envelope->Size() != in.Size())
        return Fail(error, "envelope length differs from the points");
    Points &out = Out<Points>(io, QuadraticOut());
    out = Points(points.data(), points.size());
    if (envelope)
        for (size_t i = 0; i < in.Size(); ++i)
            out[i] =
                rigExec::RigExecBlendEnvelope(in[i], out[i], (*envelope)[i]);
    return true;
}

const rigExec::RigExecPortDesc kPorts[] = {
    {&QuadraticPoints(), &rigExecStd::Vec3fArray(), rigExec::kRigExecPortInput,
     "points"},
    {&QuadraticGain(), &rigExecStd::FloatType(), rigExec::kRigExecPortInput,
     "gain"},
    {&QuadraticReferenceY(), &rigExecStd::FloatType(),
     rigExec::kRigExecPortInput, "referenceY"},
    {&QuadraticEnvelope(), &rigExecStd::FloatArray(),
     rigExec::kRigExecPortInput | rigExec::kRigExecPortOptional, "envelope"},
    {&QuadraticOut(), &rigExecStd::Vec3fArray(), rigExec::kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "test";
    d.name = "Quadratic";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    return d;
}();

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Quadratic(), kDesc);
}

const rigExec::RigExecOpLibraryDesc kLibrary = [] {
    rigExec::RigExecOpLibraryDesc d;
    d.name = "testQuadraticOps";
    d.registerOps = &Register;
    return d;
}();

/// The gain and the reference's first y, as the mover's Assemble reads its
/// declared inputs: a finite float gain, then an optional non-empty
/// reference points array; the mover needs base points.
bool
Read(const rigExec::RigExecExternalOpValues &values, float *gain,
     float *referenceY)
{
    const std::vector<VtValue> &in = *values.inputs;
    if (!values.basePoints || values.basePoints->empty() || in.empty() ||
        !in[0].IsHolding<float>())
        return false;
    *gain = in[0].UncheckedGet<float>();
    if (!std::isfinite(*gain))
        return false;
    *referenceY = 0.0f;
    if (in.size() == 2) {
        if (!in[1].IsHolding<VtVec3fArray>() ||
            in[1].UncheckedGet<VtVec3fArray>().empty())
            return false;
        *referenceY = in[1].UncheckedGet<VtVec3fArray>()[0][1];
    }
    return true;
}

bool
Add(const rigExec::RigExecRevisionBinding &,
    const rigExec::RigExecExternalOpValues &values,
    rigExec::RigExecRigDescBuilder &builder,
    rigExec::RigExecExternalOpNode &node, std::string *error)
{
    float gain = 0.0f, referenceY = 0.0f;
    if (!Read(values, &gain, &referenceY)) {
        if (error)
            *error = "quadratic mover inputs do not translate";
        return false;
    }
    node.node = builder.AddOp(Quadratic());
    node.inputs = {builder.BindInput(node.node, QuadraticGain()),
                   builder.BindInput(node.node, QuadraticReferenceY())};
    node.points = builder.BindInput(node.node, QuadraticPoints());
    node.envelope = builder.BindInput(node.node, QuadraticEnvelope());
    node.out = builder.BindOutput(node.node, QuadraticOut());
    return true;
}

bool
Write(const rigExec::RigExecRevisionBinding &,
      const rigExec::RigExecExternalOpValues &values,
      const rigExec::RigExecExternalOpNode &node,
      rigExec::RigExecEvaluator &evaluator, std::string *error)
{
    float gain = 0.0f, referenceY = 0.0f;
    if (!Read(values, &gain, &referenceY)) {
        if (error)
            *error = "quadratic mover inputs do not translate";
        return false;
    }
    return evaluator.SetInput(node.inputs[0], &gain, error) &&
           evaluator.SetInput(node.inputs[1], &referenceY, error);
}

bool
Stale(const rigExec::RigExecRevisionBinding &,
      const rigExec::RigExecExternalOpValues &,
      const rigExec::RigExecExternalOpNode &)
{
    return false;
}

const rigExec::RigExecExternalOpBinder kBinders[] = {
    {sizeof(rigExec::RigExecExternalOpBinder), "ExternalQuadraticMover",
     &Quadratic, &Add, &Write, &Stale},
};

const rigExec::RigExecExternalOpBinderTable kBinderTable = {
    rigExec::kRigExecExternalOpBinderAbiVersion, kBinders, std::size(kBinders)};

} // namespace

RIGEXEC_OP_LIBRARY_ENTRY(testQuadraticOps)
{
    return &kLibrary;
}

RIGEXEC_EXTERNAL_OP_BINDERS_ENTRY(testQuadraticOps)
{
    return &kBinderTable;
}
