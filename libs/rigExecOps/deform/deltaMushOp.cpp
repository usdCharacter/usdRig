#include "rigExecOps/deform/deltaMushOp.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/deltaMushKernel.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(DeltaMush)

RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushPoints)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushRest)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushCounts)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushIndices)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushComputationToTarget)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(DeltaMushOut)

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;
using namespace opIo;
using namespace fieldIo;

enum DeltaMushField : uint32_t {
    kIterations,
    kStep,
    kPinBorders,
    kDistanceWeight,
    kDisplacement,
    kSmoothing,
    kFrameTransport,
    kSmoothWeightCount,
    kSmoothWeights,
    kEdgeCount,
    kEdges,
    kOnlySmooth,
};

/// Instance \p k's settings, its lists read in place.
struct MushInstance {
    int iterations = 0;
    double step = 0.0;
    bool pinBorders = false;
    double distanceWeight = 0.0;
    double displacement = 0.0;
    rigExec::RigExecDeltaMushSettingsView settings;
};

MushInstance
PayloadAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    MushInstance p;
    p.iterations = ValueAt<int32_t>(batch, kIterations, k);
    p.step = ValueAt<double>(batch, kStep, k);
    p.pinBorders = FlagAt(batch, kPinBorders, k);
    p.distanceWeight = ValueAt<double>(batch, kDistanceWeight, k);
    p.displacement = ValueAt<double>(batch, kDisplacement, k);
    p.settings.smoothing =
        EnumAt<rigExec::RigExecDeltaMushSmoothing>(batch, kSmoothing, k);
    p.settings.frameTransport = EnumAt<rigExec::RigExecDeltaMushFrameTransport>(
        batch, kFrameTransport, k);
    const auto weights = ListAt<float>(batch, kSmoothWeights, k);
    p.settings.smoothWeights = {weights.data, weights.Size()};
    const auto edges = ListAt<int32_t>(batch, kEdges, k);
    p.settings.edges = {edges.data, edges.Size()};
    p.settings.onlySmooth = FlagAt(batch, kOnlySmooth, k);
    return p;
}

using RestData = rigExec::RigExecDeltaMushRestData<Vec3f, rigExecStd::Vec3d>;
using GroupState =
    rigExec::RigExecDeltaMushGroupState<Vec3f, rigExecStd::Vec3d>;

constexpr const char *kUnusable = "unusable topology, rest or computation map";

/// A node's rest data, rebuilt only when its rest points or topology move.
struct MushState {
    std::shared_ptr<const RestData> rest;
};

Mat4d
MapOf(const RigExecOpIo &io)
{
    const Mat4d *map = In<Mat4d>(io, DeltaMushComputationToTarget());
    return map ? *map : Mat4d(1.0);
}

rigExec::RigExecRangePlan
Accept(const rigExec::RigExecOpBatch &batch, size_t k, size_t count,
       std::string *error)
{
    constexpr auto kRefuse = rigExec::RigExecRangePlan::Refuse;
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    if (!rigExec::deltaMushDetail::ComputationMapValid(MapOf(io)) ||
        In<Points>(io, DeltaMushRest())->Size() != count)
        return Fail(error, "DeltaMush", kUnusable), kRefuse;
    if (!EnvelopeFits(*In<Points>(io, DeltaMushPoints()),
                      In<Envelope>(io, DeltaMushEnvelope()), "DeltaMush",
                      error))
        return kRefuse;
    return rigExec::RigExecRangePlan::Ranges;
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
        std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    MushState &state =
        *static_cast<MushState *>(rigExec::RigExecBatchState(batch, k));
    const Points &rest = *In<Points>(io, DeltaMushRest());
    const auto &counts = *In<RigExecArray<int>>(io, DeltaMushCounts());
    const auto &indices = *In<RigExecArray<int>>(io, DeltaMushIndices());
    const auto same = [](const auto &a, const auto &b) {
        return a.Size() == b.size() &&
               (b.empty() ||
                std::memcmp(a.Data(), b.data(), b.size() * sizeof(b[0])) == 0);
    };
    if (state.rest && same(rest, state.rest->rest) &&
        same(counts, state.rest->counts) && same(indices, state.rest->indices))
        return true;
    state.rest.reset();
    const MushInstance p = PayloadAt(batch, k);
    rigExec::RigExecDeltaMushSettings settings;
    settings.smoothing = p.settings.smoothing;
    settings.frameTransport = p.settings.frameTransport;
    settings.smoothWeights.assign(p.settings.smoothWeights.begin(),
                                  p.settings.smoothWeights.end());
    settings.edges.assign(p.settings.edges.begin(), p.settings.edges.end());
    settings.onlySmooth = p.settings.onlySmooth;
    auto built = std::make_shared<RestData>();
    if (!rigExec::RigExecBuildDeltaMushRestData<Vec3f, rigExecStd::Vec3d>(
            ToVector(rest), ToVector(counts), ToVector(indices), p.iterations,
            p.step, p.pinBorders, p.distanceWeight, settings, built.get()))
        return Fail(error, "DeltaMush", kUnusable);
    state.rest = std::move(built);
    return true;
}

/// Points [begin, end) of the whole kernel's result through the group
/// kernel, its region read from the whole entering array; the range's
/// scratch keeps its plan.
bool
MushRange(const rigExec::RigExecOpBatch &batch, size_t k,
          const rigExec::RigExecOpRange &range, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const MushState &state =
        *static_cast<const MushState *>(rigExec::RigExecBatchState(batch, k));
    const Points &in = *In<Points>(io, DeltaMushPoints());
    Points &out = Out<Points>(io, DeltaMushOut());
    const int bounds[2] = {0, int(in.Size())};
    const Vec3f *const groupPoints[1] = {in.Data()};
    rigExec::RigExecGroupPoints<Vec3f> groups;
    groups.bounds = bounds;
    groups.groups = 1;
    groups.points = groupPoints;
    if (!rigExec::RigExecApplyDeltaMushGroup<Vec3f, rigExecStd::Vec3d, Mat4d>(
            state.rest, ValueAt<double>(batch, kDisplacement, k), MapOf(io),
            groups, range.begin, range.end, out.Data() + range.begin,
            static_cast<GroupState *>(range.scratch)))
        return Fail(error, "DeltaMush", kUnusable);
    BlendEnvelopeRange(in, In<Envelope>(io, DeltaMushEnvelope()), out,
                       range.begin, range.end);
    return true;
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const DeltaMushPayload &p = *static_cast<const DeltaMushPayload *>(payload);
    out.Set(kIterations, int32_t(p.iterations));
    out.Set(kStep, p.step);
    SetFlag(out, kPinBorders, p.pinBorders);
    out.Set(kDistanceWeight, p.distanceWeight);
    out.Set(kDisplacement, p.displacement);
    SetEnum(out, kSmoothing, p.settings.smoothing);
    SetEnum(out, kFrameTransport, p.settings.frameTransport);
    SetList(out, kSmoothWeightCount, p.settings.smoothWeights);
    SetList(out, kEdgeCount, p.settings.edges);
    SetFlag(out, kOnlySmooth, p.settings.onlySmooth);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Value("iterations", rigExecStd::IntType()),
    Value("step", rigExecStd::DoubleType()),
    Flag("pinBorders"),
    Value("distanceWeight", rigExecStd::DoubleType()),
    Value("displacement", rigExecStd::DoubleType()),
    Enum("smoothing", rigExec::RigExecDeltaMushSmoothing::LengthWeighted),
    Enum("frameTransport", rigExec::RigExecDeltaMushFrameTransport::Corner),
    Count("smoothWeightCount"),
    Items("smoothWeights", rigExecStd::FloatType(), kSmoothWeightCount),
    Count("edgeCount"),
    Items("edges", rigExecStd::IntType(), kEdgeCount),
    Flag("onlySmooth"),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&DeltaMushPoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortNeighbourhood, "points"},
    {&DeltaMushRest(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "restPoints"},
    {&DeltaMushCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&DeltaMushIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&DeltaMushComputationToTarget(), &rigExecStd::Mat4dType(), kOptional,
     "computationToTarget"},
    {&DeltaMushEnvelope(), &rigExecStd::FloatArray(),
     kOptional | rigExec::kRigExecPortPointwise, "envelope"},
    {&DeltaMushOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &DeltaMushPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "DeltaMush";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.range.accept = &Accept;
    d.range.prepare = &Prepare;
    d.range.run = &MushRange;
    d.range.scratch = StateOf<GroupState>();
    d.state = StateOf<MushState>();
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
        rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    return d;
}();

} // namespace

void
RegisterDeltaMushOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(DeltaMush(), kDesc);
}

} // namespace rigExecOps
