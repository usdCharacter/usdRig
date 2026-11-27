#include "rigExecOps/deform/blendShapeOp.h"

#include "rigExecMath/deltaMushKernel.h"
#include "rigExecOps/deform/weightPacket.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"

#include <algorithm>
#include <cmath>
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
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeDefaultWeight)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapePacket)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeWritten)

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

const BlendShapePayload &
PayloadOf(const rigExec::RigExecOpBatch &batch, size_t k)
{
    return *static_cast<const BlendShapePayload *>(
        rigExec::RigExecBatchPayload(batch, k));
}

/// Whether the node's deltas sum per point range: no surface frame, and
/// every sample an offset layout the range kernel reads as the whole
/// kernel does.
bool
SumsInRanges(const BlendShapePayload &p)
{
    if (p.surfaceFrame)
        return false;
    for (const BlendShapeChannel &channel : p.channels)
        for (const BlendShapeSample &sample : channel.samples)
            if (!sample.layout ||
                (sample.layout->indices.empty() &&
                 !sample.layout->offsets.empty() &&
                 sample.layout->offsets.size() != sample.layout->pointCount))
                return false;
    return true;
}

rigExec::RigExecRangePlan
Plan(const rigExec::RigExecOpBatch &batch, size_t k)
{
    return SumsInRanges(PayloadOf(batch, k)) ? rigExec::RigExecRangePlan::Ranges
                                             : rigExec::RigExecRangePlan::Whole;
}

/// The point count validate could not see, checked each frame; this
/// frame's pass through and the weight at every point when a default weight
/// or a packet gives them.
struct BlendState {
    bool checkEachFrame = true;
    bool pass = false;
    std::vector<float> dense;
};

bool
ChannelsFit(const BlendShapePayload &p, size_t count, std::string *error)
{
    if (!count || !rigExec::RigExecBlendChannelsFitKernel<SampleTraits>(
                      p.channels, count))
        return Fail(error, "BlendShape",
                    "a channel has no samples or a sample doesn't fit");
    return true;
}

bool
Validate(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const BlendShapePayload &p = PayloadOf(check.batch, check.k);
    if (int(Fed(check, BlendShapeEnvelope())) +
            int(Fed(check, BlendShapeDefaultWeight())) +
            int(Fed(check, BlendShapePacket())) >
        1)
        return Fail(error, "BlendShape",
                    "one of an envelope, a default weight and a weight "
                    "packet");
    const size_t weights = CheckElements(check, BlendShapeWeights());
    if (!SameCount(weights, p.channels.size()))
        return Fail(error, "BlendShape", "needs one weight per channel");
    const size_t points = CheckElements(check, BlendShapePoints());
    const size_t base = Fed(check, BlendShapeBase())
                            ? CheckElements(check, BlendShapeBase())
                            : points;
    if (!SameCount(base, points))
        return Fail(error, "BlendShape", "base length differs from the points");
    if (p.surfaceFrame &&
        (!Fed(check, BlendShapeRest()) || !Fed(check, BlendShapeCounts()) ||
         !Fed(check, BlendShapeIndices())))
        return Fail(error, "BlendShape",
                    "surface frame needs a usable rest mesh");
    if (!EnvelopeFits(check, BlendShapePoints(), BlendShapeEnvelope(),
                      "BlendShape", error))
        return false;
    const bool known = base < rigExec::kRigExecPortUnfed;
    if (BlendState *state = CheckState<BlendState>(check))
        state->checkEachFrame = !known;
    return !known || ChannelsFit(p, base, error);
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t count,
        std::string *error)
{
    auto &state =
        *static_cast<BlendState *>(rigExec::RigExecBatchState(batch, k));
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const uint8_t *enabled = In<uint8_t>(io, BlendShapeEnabled());
    state.pass = enabled && !*enabled;
    state.dense.clear();
    if (state.pass)
        return true;
    const Points *base = In<Points>(io, BlendShapeBase());
    if (!EnvelopeFits(*In<Points>(io, BlendShapePoints()),
                      In<Envelope>(io, BlendShapeEnvelope()), "BlendShape",
                      error))
        return false;
    if (state.checkEachFrame) {
        if (base && base->Size() != count)
            return Fail(error, "BlendShape",
                        "base length differs from the points");
        if (!ChannelsFit(PayloadOf(batch, k), count, error))
            return false;
    }
    const float *weight = In<float>(io, BlendShapeDefaultWeight());
    const weightPacket::PointWeights *packet =
        In<weightPacket::PointWeights>(io, BlendShapePacket());
    if (weight && *weight != 1.0f) {
        if (!std::isfinite(*weight) || *weight < 0.0f || *weight > 1.0f)
            return Fail(error, "BlendShape",
                        "a default weight is not finite or outside [0, 1]");
        state.dense.assign(count, *weight);
    }
    if (const auto *written =
            In<rigExecStd::RigExecArray<int32_t>>(io, BlendShapeWritten())) {
        const rigExec::eval::EnvelopePacketView view =
            packet ? weightPacket::PacketView(*packet)
                   : rigExec::eval::EnvelopePacketView();
        bool holds = rigExec::eval::EnvelopeIsSparseWalk(view);
        size_t at = 0;
        for (size_t e = 0; holds && e < view.indexCount; ++e) {
            const int32_t index = view.indices[e];
            while (at + 1 < written->Size() && (*written)[at + 1] <= index)
                at += 2;
            holds = at + 1 < written->Size() && (*written)[at] <= index;
        }
        if (!holds)
            return Fail(error, "BlendShape",
                        "the weights reach a vertex group the revision does "
                        "not write");
    }
    if (packet &&
        !rigExec::eval::EnvelopeIsFullStrength(
            weightPacket::PacketView(*packet)) &&
        !weightPacket::ResolvePacket(*packet, count, &state.dense))
        return Fail(error, "BlendShape",
                    "the weight packet does not resolve over the points");
    return true;
}

const BlendState &
BlendStateAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    return *static_cast<const BlendState *>(
        rigExec::RigExecBatchState(batch, k));
}

/// The whole node: the summed deltas, carried on the surface frames when
/// it has them, added to the points.
bool
Whole(const BlendShapePayload &p, const BlendState &state,
      const rigExec::RigExecOpIo &io, std::string *error)
{
    const Points &in = *In<Points>(io, BlendShapePoints());
    if (state.pass) {
        Out<Points>(io, BlendShapeOut()).Assign(in.Data(), in.Size());
        return true;
    }
    const Envelope &weights = *In<Envelope>(io, BlendShapeWeights());
    const Points *base = In<Points>(io, BlendShapeBase());
    if (!base)
        base = &in;
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
        std::vector<Vec3f> transported;
        if (!rigExec::RigExecTransportSurfaceOffsetsKernel<Vec3f, Vec3d>(
                ToVector(*In<Points>(io, BlendShapeRest())), points,
                ToVector(*In<Ints>(io, BlendShapeCounts())),
                ToVector(*In<Ints>(io, BlendShapeIndices())), deltas,
                &transported))
            return Fail(error, "BlendShape",
                        "surface frame needs a usable rest mesh");
        deltas.swap(transported);
    }
    for (size_t i = 0; i < points.size(); ++i)
        points[i] = points[i] + deltas[i];
    if (!state.dense.empty()) {
        BlendEnvelopeAt(in.Data(), state.dense.data(), points.data(),
                        points.size());
        return WritePoints(io, BlendShapeOut(), in, points, nullptr,
                           "BlendShape", error);
    }
    return WritePoints(io, BlendShapeOut(), in, points,
                       In<Envelope>(io, BlendShapeEnvelope()), "BlendShape",
                       error);
}

/// A range's summed deltas, and the channels with a sample reaching
/// [begin, end), found on its first run.
struct BlendScratch {
    std::vector<Vec3f> deltas;
    std::vector<uint32_t> channels;
    size_t begin = 0, end = 0;
    bool keyed = false;
};

bool
Reaches(const BlendShapeLayout &layout, size_t begin, size_t end)
{
    if (layout.indices.empty())
        return !layout.offsets.empty();
    for (const int i : layout.indices)
        if (size_t(i) >= begin && size_t(i) < end)
            return true;
    return false;
}

bool
BlendRange(const rigExec::RigExecOpBatch &batch, size_t k,
           const rigExec::RigExecOpRange &range, std::string *error)
{
    const BlendShapePayload &p = PayloadOf(batch, k);
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const BlendState &state = BlendStateAt(batch, k);
    if (!SumsInRanges(p))
        return Whole(p, state, io, error);
    const Vec3f *in = RangeIn<Vec3f>(io, range, BlendShapePoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, BlendShapeOut());
    const size_t count = range.end - range.begin;
    if (state.pass) {
        std::copy(in, in + count, out);
        return true;
    }
    const Envelope &weights = *In<Envelope>(io, BlendShapeWeights());
    BlendScratch &scratch = *static_cast<BlendScratch *>(range.scratch);
    if (!scratch.keyed || scratch.begin != range.begin ||
        scratch.end != range.end) {
        scratch.channels.clear();
        for (size_t c = 0; c < p.channels.size(); ++c)
            for (const BlendShapeSample &sample : p.channels[c].samples)
                if (Reaches(*sample.layout, range.begin, range.end)) {
                    scratch.channels.push_back(uint32_t(c));
                    break;
                }
        scratch.begin = range.begin;
        scratch.end = range.end;
        scratch.keyed = true;
    }
    std::vector<Vec3f> &deltas = scratch.deltas;
    deltas.assign(count, Vec3f(0.0f));
    for (const uint32_t c : scratch.channels)
        rigExec::RigExecAccumulateBlendRange(p.channels[c].samples, weights[c],
                                             range.begin, range.end,
                                             deltas.data());
    for (size_t i = 0; i < count; ++i)
        out[i] = in[i] + deltas[i];
    BlendEnvelopeAt(in,
                    state.dense.empty()
                        ? RangeIn<float>(io, range, BlendShapeEnvelope())
                        : state.dense.data() + range.begin,
                    out, count);
    return true;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&BlendShapePoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortPointwise, "points"},
    {&BlendShapeWeights(), &rigExecStd::FloatArray(), kRigExecPortInput,
     "weights"},
    {&BlendShapeBase(), &rigExecStd::Vec3fArray(), kOptional, "base"},
    {&BlendShapeRest(), &rigExecStd::Vec3fArray(), kOptional, "rest"},
    {&BlendShapeCounts(), &rigExecStd::IntArray(), kOptional, "counts"},
    {&BlendShapeIndices(), &rigExecStd::IntArray(), kOptional, "indices"},
    {&BlendShapeEnvelope(), &rigExecStd::FloatArray(),
     kOptional | rigExec::kRigExecPortPointwise, "envelope"},
    {&BlendShapeOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &BlendShapePoints()},
    {&BlendShapeEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    {&BlendShapeDefaultWeight(), &rigExecStd::FloatType(), kOptional,
     "defaultWeight"},
    {&BlendShapePacket(), &rigExecRig::PointWeightsType(), kOptional, "packet"},
    {&BlendShapeWritten(), &rigExecStd::IntArray(), kOptional, "written"},
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
    d.validate = &Validate;
    d.range.plan = &Plan;
    d.range.prepare = &Prepare;
    d.range.run = &BlendRange;
    d.state = StateOf<BlendState>();
    d.range.scratch = StateOf<BlendScratch>();
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
