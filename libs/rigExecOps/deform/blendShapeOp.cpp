#include "rigExecOps/deform/blendShapeOp.h"

#include "rigExecMath/deltaMushKernel.h"
#include "rigExecOps/opIo.h"

#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(BlendShape)

RIGEXEC_DEFINE_PORT_HANDLE(BlendShapePoints)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeWeights)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeBase)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeRest)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeCounts)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeIndices)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeOut)

namespace {

using namespace opIo;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Ints = rigExecStd::RigExecArray<int32_t>;

struct SampleTraits {
    static float Activation(const BlendShapeSample &s)
    {
        return s.activation;
    }
    static const BlendShapeLayout *Layout(const BlendShapeSample &s)
    {
        return s.layout.get();
    }
    static const Vec3f *DensePoints(const BlendShapeSample &s)
    {
        return s.points.data();
    }
    static size_t DenseCount(const BlendShapeSample &s)
    {
        return s.points.size();
    }
};

bool
Run(const void *payload, void *, const rigExec::RigExecOpIo &io,
    std::string *error)
{
    const BlendShapePayload &p =
        *static_cast<const BlendShapePayload *>(payload);
    const Points &in = *In<Points>(io, BlendShapePoints());
    const Envelope &weights = *In<Envelope>(io, BlendShapeWeights());
    const Points *base = In<Points>(io, BlendShapeBase());
    if (!base)
        base = &in;
    if (weights.Size() != p.channels.size())
        return Fail(error, "BlendShape", "needs one weight per channel");
    if (base->Size() != in.Size())
        return Fail(error, "BlendShape", "base length differs from the points");

    const BlendShapeChannel *first = p.channels.data();
    std::vector<Vec3f> deltas;
    if (!rigExec::RigExecSumBlendChannelsKernel<SampleTraits>(
            p.channels,
            [&](const BlendShapeChannel &c) {
                return weights[size_t(&c - first)];
            },
            base->Data(), base->Size(), &deltas,
            [](size_t n, const auto &fn) { fn(0, n); }))
        return Fail(error, "BlendShape",
                    "a channel has no samples or a sample doesn't fit");

    std::vector<Vec3f> points = ToVector(in);
    if (p.surfaceFrame) {
        const Points *rest = In<Points>(io, BlendShapeRest());
        const Ints *counts = In<Ints>(io, BlendShapeCounts());
        const Ints *indices = In<Ints>(io, BlendShapeIndices());
        std::vector<Vec3f> transported;
        if (!rest || !counts || !indices ||
            !rigExec::RigExecTransportSurfaceOffsetsKernel<Vec3f, Vec3d>(
                ToVector(*rest), points, ToVector(*counts), ToVector(*indices),
                deltas, &transported))
            return Fail(error, "BlendShape",
                        "surface frame needs a usable rest mesh");
        deltas.swap(transported);
    }
    for (size_t i = 0; i < points.size(); ++i)
        points[i] = points[i] + deltas[i];
    return WritePoints(io, BlendShapeOut(), in, points,
                       In<Envelope>(io, BlendShapeEnvelope()), "BlendShape",
                       error);
}

} // namespace

void
RegisterBlendShapeOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &ints = rigExecStd::IntArray();
    static const rigExec::RigExecPortDesc ports[] = {
        {&BlendShapePoints(), &points, kRigExecPortInput, "points"},
        {&BlendShapeWeights(), &rigExecStd::FloatArray(), kRigExecPortInput,
         "weights"},
        {&BlendShapeBase(), &points, kOptional, "base"},
        {&BlendShapeRest(), &points, kOptional, "rest"},
        {&BlendShapeCounts(), &ints, kOptional, "counts"},
        {&BlendShapeIndices(), &ints, kOptional, "indices"},
        {&BlendShapeEnvelope(), &rigExecStd::FloatArray(), kOptional,
         "envelope"},
        {&BlendShapeOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "BlendShape";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &Run;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<BlendShapePayload>();
        return d;
    }();
    registrar.Add(BlendShape(), desc);
}

} // namespace rigExecOps
