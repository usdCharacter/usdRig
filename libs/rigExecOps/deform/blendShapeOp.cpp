#include "rigExecOps/deform/blendShapeOp.h"

#include "rigExecMath/blendLayoutKernel.h"
#include "rigExecMath/deltaMushKernel.h"
#include "rigExecOps/deform/weightPacket.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"

#include <algorithm>
#include <cmath>
#include <cstring>
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
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeActivations)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeTargets)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeOffsets)
RIGEXEC_DEFINE_PORT_HANDLE(BlendShapeSampleIndices)

namespace {

using namespace opIo;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Ints = rigExecStd::RigExecArray<int32_t>;
using PointsList = rigExecStd::RigExecArrayList<Vec3f>;
using IntsList = rigExecStd::RigExecArrayList<int32_t>;

/// A sample as this frame reads it: its activation, and its dense points or
/// its layout, held by the payload, an input or the node's state.
struct LiveSample {
    float activation = 1.0f;
    const Vec3f *points = nullptr;
    size_t count = 0;
    const BlendShapeLayout *layout = nullptr;
};

struct LiveChannel {
    std::vector<LiveSample> samples;
};

/// An input array read as the layout builder reads its arguments.
template <class T> struct ArraySpan {
    const rigExecStd::RigExecArray<T> &array;
    bool empty() const
    {
        return array.Empty();
    }
    size_t size() const
    {
        return array.Size();
    }
    const T *begin() const
    {
        return array.begin();
    }
    const T *end() const
    {
        return array.end();
    }
};

struct LiveTraits {
    static float Activation(const LiveSample &s)
    {
        return s.activation;
    }
    static const BlendShapeLayout *Layout(const LiveSample &s)
    {
        return s.layout;
    }
    static const Vec3f *DensePoints(const LiveSample &s)
    {
        return s.points;
    }
    static size_t DenseCount(const LiveSample &s)
    {
        return s.count;
    }
};

const BlendShapePayload &
PayloadOf(const rigExec::RigExecOpBatch &batch, size_t k)
{
    return *static_cast<const BlendShapePayload *>(
        rigExec::RigExecBatchPayload(batch, k));
}

/// Whether the node's deltas sum per point range: every point's delta reads
/// only its own samples' values, unless the surface frame carries them.
bool
SumsInRanges(const BlendShapePayload &p)
{
    return !p.surfaceFrame;
}

rigExec::RigExecRangePlan
Plan(const rigExec::RigExecOpBatch &batch, size_t k)
{
    return SumsInRanges(PayloadOf(batch, k)) ? rigExec::RigExecRangePlan::Ranges
                                             : rigExec::RigExecRangePlan::Whole;
}

/// The point count validate could not see, checked each frame; this
/// frame's pass through, the weight at every point when a default weight
/// or a packet gives them, its samples sorted by activation, and the
/// layouts of the samples whose layout moves.
struct BlendState {
    bool checkEachFrame = true;
    bool pass = false;
    std::vector<float> dense;
    std::vector<LiveChannel> channels;
    std::vector<BlendShapeLayout> built;
};

bool
ChannelsFit(const std::vector<LiveChannel> &channels, size_t count,
            std::string *error)
{
    if (!count ||
        !rigExec::RigExecBlendChannelsFitKernel<LiveTraits>(channels, count))
        return Fail(error, "BlendShape",
                    "a channel has no samples or a sample doesn't fit");
    return true;
}

/// Fills \p state's channels with this frame's samples: the payload's,
/// with the activations, points and layouts the inputs give in place of
/// those that can move, each channel stably sorted by activation.
bool
GatherSamples(const BlendShapePayload &p, const rigExec::RigExecOpIo &io,
              size_t count, BlendState *state, std::string *error)
{
    const Envelope *activations = In<Envelope>(io, BlendShapeActivations());
    const PointsList *targets = In<PointsList>(io, BlendShapeTargets());
    const PointsList *offsets = In<PointsList>(io, BlendShapeOffsets());
    const IntsList *indices = In<IntsList>(io, BlendShapeSampleIndices());
    size_t k = 0;
    state->channels.resize(p.channels.size());
    for (size_t c = 0; c < p.channels.size(); ++c) {
        const std::vector<BlendShapeSample> &samples = p.channels[c].samples;
        std::vector<LiveSample> &live = state->channels[c].samples;
        live.resize(samples.size());
        for (size_t s = 0; s < samples.size(); ++s, ++k) {
            const BlendShapeSample &sample = samples[s];
            LiveSample &l = live[s];
            l = LiveSample();
            if (activations && k >= activations->Size())
                return Fail(error, "BlendShape",
                            "needs one activation per sample");
            l.activation = activations ? (*activations)[k] : sample.activation;
            if (sample.target >= 0) {
                if (!targets || size_t(sample.target) >= targets->Size())
                    return Fail(error, "BlendShape",
                                "a sample's target points are missing");
                const auto &points = (*targets)[size_t(sample.target)];
                l.points = points.Data();
                l.count = points.Size();
            } else if (sample.sparse >= 0) {
                const size_t at = size_t(sample.sparse);
                if (!offsets || !indices || at >= offsets->Size() ||
                    at >= indices->Size())
                    return Fail(error, "BlendShape",
                                "a sample's offsets or indices are missing");
                if (state->built.size() <= at)
                    state->built.resize(at + 1);
                BlendShapeLayout &layout = state->built[at];
                const auto &o = (*offsets)[at];
                const auto &i = (*indices)[at];
                const std::vector<Vec3f> &held = layout.offsets;
                const std::vector<int> &heldIndices = layout.indices;
                const bool same =
                    layout.pointCount == count && held.size() == o.Size() &&
                    heldIndices.size() == i.Size() &&
                    std::equal(i.begin(), i.end(), heldIndices.begin()) &&
                    (!o.Size() || std::memcmp(held.data(), o.Data(),
                                              o.Size() * sizeof(Vec3f)) == 0);
                if (!same && !rigExec::RigExecBuildBlendLayout(
                                 ArraySpan<Vec3f>{o}, ArraySpan<int32_t>{i},
                                 count, &layout))
                    return Fail(error, "BlendShape",
                                "a sample's offsets do not fit the points");
                if (!layout.valid)
                    return Fail(error, "BlendShape",
                                "a sample's offsets do not fit the points");
                l.layout = &layout;
            } else if (sample.layout) {
                l.layout = sample.layout.get();
            } else {
                l.points = sample.points.data();
                l.count = sample.points.size();
            }
        }
        if (activations)
            for (size_t s = 1; s < live.size(); ++s)
                for (size_t j = s;
                     j > 0 && live[j].activation < live[j - 1].activation; --j)
                    std::swap(live[j], live[j - 1]);
    }
    if (activations && k != activations->Size())
        return Fail(error, "BlendShape", "needs one activation per sample");
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
    const bool live = Fed(check, BlendShapeActivations()) ||
                      Fed(check, BlendShapeTargets()) ||
                      Fed(check, BlendShapeOffsets()) ||
                      Fed(check, BlendShapeSampleIndices());
    const bool known = base < rigExec::kRigExecPortUnfed;
    if (BlendState *state = CheckState<BlendState>(check))
        state->checkEachFrame = live || !known;
    if (live || !known)
        return true;
    std::vector<LiveChannel> channels(p.channels.size());
    for (size_t c = 0; c < p.channels.size(); ++c)
        for (const BlendShapeSample &sample : p.channels[c].samples) {
            if (sample.target >= 0 || sample.sparse >= 0)
                return Fail(error, "BlendShape",
                            "a sample reads an input that is not fed");
            LiveSample l;
            l.activation = sample.activation;
            l.layout = sample.layout.get();
            l.points = sample.layout ? nullptr : sample.points.data();
            l.count = sample.layout ? 0 : sample.points.size();
            channels[c].samples.push_back(l);
        }
    return ChannelsFit(channels, base, error);
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
    if (state.checkEachFrame && base && base->Size() != count)
        return Fail(error, "BlendShape", "base length differs from the points");
    if (!GatherSamples(PayloadOf(batch, k), io, count, &state, error))
        return false;
    if (state.checkEachFrame && !ChannelsFit(state.channels, count, error))
        return false;
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
    const LiveChannel *first = state.channels.data();
    std::vector<Vec3f> deltas;
    if (!rigExec::RigExecSumBlendChannelsKernel<LiveTraits>(
            state.channels,
            [&](const LiveChannel &c) { return weights[size_t(&c - first)]; },
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

/// Whether \p sample can move a point in [begin, end): a dense sample, or
/// one whose values move, can move any.
bool
Reaches(const BlendShapeSample &sample, size_t begin, size_t end)
{
    if (!sample.layout || sample.target >= 0 || sample.sparse >= 0)
        return true;
    const BlendShapeLayout &layout = *sample.layout;
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
                if (Reaches(sample, range.begin, range.end)) {
                    scratch.channels.push_back(uint32_t(c));
                    break;
                }
        scratch.begin = range.begin;
        scratch.end = range.end;
        scratch.keyed = true;
    }
    const Points *base = In<Points>(io, BlendShapeBase());
    const Vec3f *from = base ? base->Data() + range.begin : in;
    std::vector<Vec3f> &deltas = scratch.deltas;
    deltas.assign(count, Vec3f(0.0f));
    for (const uint32_t c : scratch.channels)
        rigExec::RigExecSumBlendChannelRangeKernel<LiveTraits>(
            state.channels[c], weights[c], from, range.begin, range.end,
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
    {&BlendShapeActivations(), &rigExecStd::FloatArray(), kOptional,
     "activations"},
    {&BlendShapeTargets(), &rigExecStd::Vec3fArrayList(), kOptional, "targets"},
    {&BlendShapeOffsets(), &rigExecStd::Vec3fArrayList(), kOptional, "offsets"},
    {&BlendShapeSampleIndices(), &rigExecStd::IntArrayList(), kOptional,
     "sampleIndices"},
};

using payloadIo::Reader;
using payloadIo::Writer;

// A sample's fixed part: activation, the points count, the layout flag and
// its target and sparse elements.
constexpr size_t kSampleBytes = 4 + 8 + 1 + 4 + 4;

void
PutSample(Writer &w, const BlendShapeSample &s)
{
    w.F32(s.activation);
    payloadIo::PutList(w, s.points);
    w.I32(s.target);
    w.I32(s.sparse);
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
    s.target = r.I32();
    s.sparse = r.I32();
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
    w.Version(2);
    w.List(p.channels, [](Writer &out, const BlendShapeChannel &c) {
        out.List(c.samples, &PutSample);
    });
    w.Bool(p.surfaceFrame);
}

void
Load(Reader &r, BlendShapePayload &p)
{
    r.Version(2);
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
