#include "rigExecOps/deform/pointOps.h"

#include "rigExecOps/authoring.h"
#include "rigExecOps/deform/weightPacket.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecMath/meshConnectivity.h"
#include "rigExecMath/smoothKernel.h"
#include "rigExecMath/meshBoundsKernel.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <vector>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Smooth)
RIGEXEC_DEFINE_OP_HANDLE(VolumeCorrect)

RIGEXEC_DEFINE_PORT_HANDLE(SmoothPoints)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothCounts)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothIndices)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothStrength)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothOut)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothDefaultWeight)
RIGEXEC_DEFINE_PORT_HANDLE(SmoothPacket)

RIGEXEC_DEFINE_PORT_HANDLE(VolumeCorrectPoints)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeCorrectReference)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeCorrectStrength)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeCorrectEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeCorrectOut)

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;
using namespace opIo;
using namespace fieldIo;

enum SmoothField : uint32_t { kSmoothWhole };

rigExec::RigExecRangePlan
SmoothPlan(const rigExec::RigExecOpBatch &batch, size_t k)
{
    return FlagAt(batch, kSmoothWhole, k) ? rigExec::RigExecRangePlan::Whole
                                          : rigExec::RigExecRangePlan::Ranges;
}

void
SmoothValues(const void *payload, rigExec::RigExecFieldValues &out)
{
    SetFlag(out, kSmoothWhole,
            static_cast<const SmoothPayload *>(payload)->whole);
}

const rigExec::RigExecFieldDesc kSmoothFields[] = {Flag("whole")};

float
StrengthOr1(const float *strength)
{
    return strength ? *strength : 1.0f;
}

/// A node's edge adjacency, rebuilt only when its topology or point count
/// moves; this frame's pass through and the weight at every point when a
/// default weight or a packet gives them, written by prepare.
struct SmoothState {
    rigExec::RigExecMeshAdjacency adjacency;
    bool pass = false;
    std::vector<float> dense;
    std::vector<int> counts;
    std::vector<int> indices;
    size_t pointCount = SIZE_MAX;
    /// Per range index: the range and the points it reads.
    std::vector<std::array<size_t, 4>> reads;
};

bool
SmoothValidate(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const int envelopes = int(Fed(check, SmoothEnvelope())) +
                          int(Fed(check, SmoothDefaultWeight())) +
                          int(Fed(check, SmoothPacket()));
    if (envelopes > 1)
        return Fail(error, "Smooth",
                    "one of an envelope, a default weight and a weight "
                    "packet");
    return EnvelopeFits(check, SmoothPoints(), SmoothEnvelope(), "Smooth",
                        error);
}

bool
SmoothPrepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t count,
              std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    if (!EnvelopeFits(*In<Points>(io, SmoothPoints()),
                      In<RigExecArray<float>>(io, SmoothEnvelope()), "Smooth",
                      error))
        return false;
    SmoothState &state =
        *static_cast<SmoothState *>(rigExec::RigExecBatchState(batch, k));
    const uint8_t *enabled = In<uint8_t>(io, SmoothEnabled());
    const float *weight = In<float>(io, SmoothDefaultWeight());
    const weightPacket::PointWeights *packet =
        In<weightPacket::PointWeights>(io, SmoothPacket());
    state.pass = enabled && !*enabled;
    state.dense.clear();
    if (weight && !state.pass && *weight != 1.0f) {
        if (!std::isfinite(*weight) || *weight < 0.0f || *weight > 1.0f)
            return Fail(error, "Smooth",
                        "a default weight is not finite or outside [0, 1]");
        state.dense.assign(count, *weight);
    }
    if (packet && !state.pass &&
        !rigExec::eval::EnvelopeIsFullStrength(
            weightPacket::PacketView(*packet)) &&
        !weightPacket::ResolvePacket(*packet, count, &state.dense))
        return Fail(error, "Smooth",
                    "the weight packet does not resolve over the points");
    const auto &counts = *In<RigExecArray<int>>(io, SmoothCounts());
    const auto &indices = *In<RigExecArray<int>>(io, SmoothIndices());
    if (state.pointCount == count &&
        std::equal(counts.begin(), counts.end(), state.counts.begin(),
                   state.counts.end()) &&
        std::equal(indices.begin(), indices.end(), state.indices.begin(),
                   state.indices.end()))
        return true;
    state.counts.assign(counts.begin(), counts.end());
    state.indices.assign(indices.begin(), indices.end());
    state.pointCount = count;
    state.reads.clear();
    // An adjacency that fails to build covers nothing, so the group kernel
    // passes the points through, as the whole kernel does.
    state.adjacency = rigExec::RigExecMeshAdjacency();
    rigExec::RigExecBuildMeshAdjacency(count, counts.Data(), counts.Size(),
                                       indices.Data(), indices.Size(),
                                       &state.adjacency);
    return true;
}

void
SmoothReads(const rigExec::RigExecOpBatch &batch, size_t k,
            const rigExec::RigExecOpRange &range, size_t *begin, size_t *end)
{
    SmoothState &state =
        *static_cast<SmoothState *>(rigExec::RigExecBatchState(batch, k));
    if (state.reads.size() <= range.index)
        state.reads.resize(range.index + 1, {SIZE_MAX, 0, 0, 0});
    std::array<size_t, 4> &memo = state.reads[range.index];
    if (memo[0] != range.begin || memo[1] != range.end) {
        size_t lo = range.begin, hi = range.end;
        if (!state.adjacency.Covers(range.count)) {
            lo = 0;
            hi = range.count;
        } else {
            for (size_t i = range.begin; i < range.end; ++i)
                for (const int n : state.adjacency.neighbors[i]) {
                    lo = std::min(lo, size_t(n));
                    hi = std::max(hi, size_t(n) + 1);
                }
        }
        memo = {range.begin, range.end, lo, hi};
    }
    *begin = memo[2];
    *end = memo[3];
}

bool
SmoothRange(const rigExec::RigExecOpBatch &batch, size_t k,
            const rigExec::RigExecOpRange &range, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const SmoothState &state =
        *static_cast<const SmoothState *>(rigExec::RigExecBatchState(batch, k));
    const rigExec::RigExecGroupPoints<Vec3f> groups =
        RangeGroups<Vec3f>(range, SmoothPoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, SmoothOut());
    if (state.pass) {
        const Vec3f *in = groups.At(range.begin);
        std::copy(in, in + (range.end - range.begin), out);
        return true;
    }
    if (!rigExec::RigExecApplyLaplacianSmoothGroup(
            state.adjacency, StrengthOr1(In<float>(io, SmoothStrength())),
            range.count, groups, range.begin, range.end, out))
        return Fail(error, "Smooth", "topology addresses no point");
    BlendEnvelopeAt(groups.At(range.begin),
                    state.dense.empty()
                        ? RangeIn<float>(io, range, SmoothEnvelope())
                        : state.dense.data() + range.begin,
                    out, range.end - range.begin);
    return true;
}

bool
VolumeCorrectOne(const RigExecOpIo &io, std::string *error)
{
    const Points &in = *In<Points>(io, VolumeCorrectPoints());
    std::vector<Vec3f> points = ToVector(in);
    rigExec::RigExecApplyVolumeCorrectKernel(
        &points, *In<double>(io, VolumeCorrectReference()),
        StrengthOr1(In<float>(io, VolumeCorrectStrength())));
    return WritePoints(io, VolumeCorrectOut(), in, points,
                       In<RigExecArray<float>>(io, VolumeCorrectEnvelope()),
                       "VolumeCorrect", error);
}

bool
CheckVolumeCorrect(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return EnvelopeFits(check, VolumeCorrectPoints(), VolumeCorrectEnvelope(),
                        "VolumeCorrect", error);
}

bool
RunVolumeCorrect(const rigExec::RigExecOpBatch &batch, size_t *failed,
                 std::string *error)
{
    for (size_t k = 0; k < batch.count; ++k) {
        if (rigExec::RigExecBatchActive(batch, k) &&
            !VolumeCorrectOne(rigExec::RigExecBatchInstance(batch, k), error)) {
            *failed = k;
            return false;
        }
    }
    return true;
}

rigExec::RigExecOpDesc
PointDesc(const char *name, const rigExec::RigExecPortDesc *ports,
          size_t portCount, rigExec::RigExecOpBatchFn runBatch,
          uint32_t capabilities)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable |
                     rigExec::kRigExecOpFromLeaves | capabilities;
    return d;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecSchemaInfo kSmoothSchema{
    rigExec::SchemaTypeName::RigExecSmoothMover,
    R"(Moves exactly one exact native UsdGeomPointBased points property through
one iteration of uniform-weight Laplacian smoothing with fixed edge
adjacency taken from the destination's standard topology. The common
MoverAPI envelope blends the full Laplacian step over the incoming
points; an authored inputs:strength is a compile error.)",
    authoring::kTyped, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecSchemaInfo kVolumeSchema{
    rigExec::SchemaTypeName::RigExecVolumeCorrectMover,
    R"(Moves exactly one exact native UsdGeomPointBased points property by
uniform centroid scaling toward the axis-aligned bound volume of the
target's authored base points. The common MoverAPI envelope blends full
bound-volume restoration over the incoming points; an authored
inputs:strength is a compile error.)",
    authoring::kTyped, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecPortDesc kSmoothPorts[] = {
    {&SmoothPoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortNeighbourhood, "points"},
    {&SmoothCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&SmoothIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&SmoothStrength(), &rigExecStd::FloatType(), kOptional, "strength"},
    {&SmoothEnvelope(), &rigExecStd::FloatArray(),
     kOptional | rigExec::kRigExecPortPointwise, "envelope"},
    {&SmoothOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &SmoothPoints()},
    {&SmoothEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    {&SmoothDefaultWeight(), &rigExecStd::FloatType(), kOptional,
     "defaultWeight"},
    {&SmoothPacket(), &rigExecRig::PointWeightsType(), kOptional, "packet"},
};

const rigExec::RigExecPortDesc kVolumePorts[] = {
    {&VolumeCorrectPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&VolumeCorrectReference(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "referenceVolume"},
    {&VolumeCorrectStrength(), &rigExecStd::FloatType(), kOptional, "strength"},
    {&VolumeCorrectEnvelope(), &rigExecStd::FloatArray(), kOptional,
     "envelope"},
    {&VolumeCorrectOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &VolumeCorrectPoints()},
};

const rigExec::RigExecOpDesc kSmoothDesc = [] {
    rigExec::RigExecOpDesc d =
        PointDesc("Smooth", kSmoothPorts, std::size(kSmoothPorts), nullptr,
                  rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
                      rigExec::kRigExecOpGate);
    d.validate = &SmoothValidate;
    d.schema = &kSmoothSchema;
    d.range.prepare = &SmoothPrepare;
    d.range.run = &SmoothRange;
    d.range.reads = &SmoothReads;
    d.range.plan = &SmoothPlan;
    d.state = StateOf<SmoothState>();
    d.payloadFields = kSmoothFields;
    d.payloadFieldCount = std::size(kSmoothFields);
    d.payloadVersion = 1;
    d.payloadValues = &SmoothValues;
    return d;
}();

const rigExec::RigExecOpDesc kVolumeDesc = [] {
    rigExec::RigExecOpDesc d =
        PointDesc("VolumeCorrect", kVolumePorts, std::size(kVolumePorts),
                  &RunVolumeCorrect, rigExec::kRigExecOpSeparateBlend);
    d.validate = &CheckVolumeCorrect;
    d.schema = &kVolumeSchema;
    return d;
}();

} // namespace

void
RegisterPointOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Smooth(), kSmoothDesc);
    registrar.Add(VolumeCorrect(), kVolumeDesc);
}

} // namespace rigExecOps
