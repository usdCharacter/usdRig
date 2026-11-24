#include "rigExecOps/deform/blendShapeOp.h"

#include "rigExecMath/deltaMushKernel.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"

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

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&BlendShapePoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&BlendShapeWeights(), &rigExecStd::FloatArray(), kRigExecPortInput,
     "weights"},
    {&BlendShapeBase(), &rigExecStd::Vec3fArray(), kOptional, "base"},
    {&BlendShapeRest(), &rigExecStd::Vec3fArray(), kOptional, "rest"},
    {&BlendShapeCounts(), &rigExecStd::IntArray(), kOptional, "counts"},
    {&BlendShapeIndices(), &rigExecStd::IntArray(), kOptional, "indices"},
    {&BlendShapeEnvelope(), &rigExecStd::FloatArray(), kOptional, "envelope"},
    {&BlendShapeOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &BlendShapePoints()},
};

using payloadIo::Reader;
using payloadIo::Writer;

// A sample's fixed part: activation, the points count and the layout flag.
constexpr size_t kSampleBytes = 4 + 8 + 1;

void
PutSample(Writer &w, const BlendShapeSample &s)
{
    w.F32(s.activation);
    payloadIo::PutList(w, s.points);
    w.Bool(s.layout != nullptr);
    if (!s.layout)
        return;
    payloadIo::PutList(w, s.layout->offsets);
    payloadIo::PutList(w, s.layout->indices);
    w.U64(s.layout->pointCount);
    w.Bool(s.layout->valid);
}

void
GetSample(Reader &r, BlendShapeSample &s)
{
    s.activation = r.F32();
    payloadIo::GetList(r, s.points);
    s.layout.reset();
    if (!r.Bool() || !r.Ok())
        return;
    auto layout = std::make_shared<BlendShapeLayout>();
    payloadIo::GetList(r, layout->offsets);
    payloadIo::GetList(r, layout->indices);
    layout->pointCount = size_t(r.U64());
    layout->valid = r.Bool();
    s.layout = std::move(layout);
}

void
Save(Writer &w, const BlendShapePayload &p)
{
    w.Version(1);
    w.List(p.channels, [](Writer &out, const BlendShapeChannel &c) {
        out.List(c.samples, &PutSample);
    });
    w.Bool(p.surfaceFrame);
}

void
Load(Reader &r, BlendShapePayload &p)
{
    r.Version(1);
    r.List(p.channels, 8, [](Reader &in, BlendShapeChannel &c) {
        in.List(c.samples, kSampleBytes, &GetSample);
    });
    p.surfaceFrame = r.Bool();
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "BlendShape";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable | rigExec::kRigExecOpRanged |
                     rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<BlendShapePayload, &Save, &Load>();
    return d;
}();

} // namespace

void
RegisterBlendShapeOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(BlendShape(), kDesc);
}

} // namespace rigExecOps
