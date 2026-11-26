#include "rigExecOps/deform/deltaMushOp.h"

#include "rigExecOps/authoring.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/deltaMushKernel.h"
#include "rigExecSchema/schemaNames.h"

#include <algorithm>
#include <array>
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
    /// Per range index: the range and the points it reads, for the rest
    /// data in readsRest.
    std::shared_ptr<const RestData> readsRest;
    std::vector<std::array<size_t, 4>> reads;
    std::vector<int> readList;
};

Mat4d
MapOf(const RigExecOpIo &io)
{
    const Mat4d *map = In<Mat4d>(io, DeltaMushComputationToTarget());
    return map ? *map : Mat4d(1.0);
}

bool
Validate(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, DeltaMushRest()),
                   CheckElements(check, DeltaMushPoints())))
        return Fail(error, "DeltaMush", kUnusable);
    return EnvelopeFits(check, DeltaMushPoints(), DeltaMushEnvelope(),
                        "DeltaMush", error);
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t count,
        std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    if (!rigExec::deltaMushDetail::ComputationMapValid(MapOf(io)))
        return Fail(error, "DeltaMush", kUnusable);
    if (In<Points>(io, DeltaMushRest())->Size() != count)
        return Fail(error, "DeltaMush", kUnusable);
    if (!EnvelopeFits(*In<Points>(io, DeltaMushPoints()),
                      In<Envelope>(io, DeltaMushEnvelope()), "DeltaMush",
                      error))
        return false;
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

void
MushReads(const rigExec::RigExecOpBatch &batch, size_t k,
          const rigExec::RigExecOpRange &range, size_t *begin, size_t *end)
{
    MushState &state =
        *static_cast<MushState *>(rigExec::RigExecBatchState(batch, k));
    if (state.readsRest != state.rest) {
        state.readsRest = state.rest;
        state.reads.clear();
    }
    if (state.reads.size() <= range.index)
        state.reads.resize(range.index + 1, {SIZE_MAX, 0, 0, 0});
    std::array<size_t, 4> &memo = state.reads[range.index];
    if (memo[0] != range.begin || memo[1] != range.end) {
        size_t lo = 0, hi = range.count;
        if (state.rest &&
            rigExec::RigExecDeltaMushGroupReads(*state.rest, range.begin,
                                                range.end, &state.readList) &&
            !state.readList.empty()) {
            lo = std::min(range.begin, size_t(state.readList.front()));
            hi = std::max(range.end, size_t(state.readList.back()) + 1);
        }
        memo = {range.begin, range.end, lo, hi};
    }
    *begin = memo[2];
    *end = memo[3];
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
    const rigExec::RigExecGroupPoints<Vec3f> groups =
        RangeGroups<Vec3f>(range, DeltaMushPoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, DeltaMushOut());
    if (!rigExec::RigExecApplyDeltaMushGroup<Vec3f, rigExecStd::Vec3d, Mat4d>(
            state.rest, ValueAt<double>(batch, kDisplacement, k), MapOf(io),
            groups, range.begin, range.end, out,
            static_cast<GroupState *>(range.scratch)))
        return Fail(error, "DeltaMush", kUnusable);
    BlendEnvelopeAt(groups.At(range.begin),
                    RangeIn<float>(io, range, DeltaMushEnvelope()), out,
                    range.end - range.begin);
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

namespace names = rigExec::SchemaAttrText;
namespace values = rigExec::SchemaValueText;
namespace types = rigExec::SchemaTypeName;
using authoring::Authored;

const int32_t kDefaultIterations = 10;
const double kHalf = 0.5;
const double kZero = 0.0;
const double kOne = 1.0;
const bool kTrue = true;
const bool kFalse = false;
const uint32_t kRestSmoothing =
    uint32_t(rigExec::RigExecDeltaMushSmoothing::Rest);
const uint32_t kVertexTransport =
    uint32_t(rigExec::RigExecDeltaMushFrameTransport::Vertex);
const rigExecStd::RigExecArray<float> kNoFloats;
const rigExecStd::RigExecArray<int32_t> kNoInts;
const rigExecStd::Mat4d kIdentity(1.0);
const char *const kSmoothings[] = {values::rest, values::simple,
                                   values::lengthWeighted};
const char *const kTransports[] = {values::vertex, values::corner};

const rigExec::RigExecPropertyInfo kRestPoints =
    authoring::WithRole(authoring::Property(names::inputsRestPoints, nullptr),
                        rigExec::RigExecPropertyRole::Point);
const rigExec::RigExecPropertyInfo kIterationsProperty =
    authoring::Property(names::inputsIterations, &kDefaultIterations);
const rigExec::RigExecPropertyInfo kStepProperty = authoring::WithType(
    authoring::Property(names::inputsStep, &kHalf), "float");
const rigExec::RigExecPropertyInfo kPinBordersProperty =
    authoring::Property(names::inputsPinBorders, &kTrue);
const rigExec::RigExecPropertyInfo kDistanceWeightProperty =
    authoring::WithType(
        authoring::Property(names::inputsDistanceWeight, &kZero), "float");
const rigExec::RigExecPropertyInfo kDisplacementProperty = authoring::WithType(
    authoring::Property(names::inputsDisplacement, &kOne), "float");
const rigExec::RigExecPropertyInfo kSmoothingProperty = authoring::Tokens(
    names::inputsSmoothing, &kRestSmoothing, kSmoothings,
    R"(Rest uses fixed reference edge weights, each edge's reference length to
the power -distanceWeight (uniform at the default 0), and needs step and
displacement in [0,1]. Simple averages incident edge displacements.
LengthWeighted recomputes current edge lengths per iteration and divides
by length sum times edge valence, using twice the step. The latter modes
permit any finite step and detail scale.)");
const rigExec::RigExecPropertyInfo kFrameTransportProperty = authoring::Tokens(
    names::inputsFrameTransport, &kVertexTransport, kTransports,
    R"(Vertex uses an averaged surface normal and reference edge. Corner
transports detail in each directed face-corner frame and combines posed
corner-angle weights. Degenerate corners contribute zero weight.)");
const rigExec::RigExecPropertyInfo kSmoothWeightsProperty = authoring::Property(
    names::inputsSmoothWeights, &kNoFloats,
    R"(Empty means one at every vertex; otherwise one finite value in [0,1] per
vertex, multiplying the smoothing step in both reference and incoming
iterations. Distinct from the common final envelope.)");
const rigExec::RigExecPropertyInfo kEdgesProperty = authoring::Uniform(
    names::inputsEdges, &kNoInts,
    R"(Optional flattened unique undirected edge pairs, including all polygon
edges and any loose edges. Empty derives polygon edges. Border pinning
uses polygon incidence.)");
const rigExec::RigExecPropertyInfo kOnlySmoothProperty = authoring::Property(
    names::inputsOnlySmooth, &kFalse,
    R"(Return the smoothed incoming shape without restoring detail.)");
const rigExec::RigExecPropertyInfo kComputationToTarget = authoring::Property(
    names::inputsComputationToTarget, &kIdentity,
    R"(Nonsingular affine row-vector map from the reference/smoothing
coordinate space into the target points space. Reference points remain
in computation space; incoming points transform back before smoothing.
Explicit restPoints are required for a nonidentity map or frame
provider, except in onlySmooth mode.)");

const rigExec::RigExecSchemaInfo kMushSchema{
    types::RigExecDeltaMushMover, nullptr, authoring::kTyped,
    rigExec::RigExecSchemaMode::HandAuthored};

const rigExec::RigExecFieldDesc kFields[] = {
    Authored(Value("iterations", rigExecStd::IntType()), kIterationsProperty),
    Authored(Value("step", rigExecStd::DoubleType()), kStepProperty),
    Authored(Flag("pinBorders"), kPinBordersProperty),
    Authored(Value("distanceWeight", rigExecStd::DoubleType()),
             kDistanceWeightProperty),
    Authored(Value("displacement", rigExecStd::DoubleType()),
             kDisplacementProperty),
    Authored(
        Enum("smoothing", rigExec::RigExecDeltaMushSmoothing::LengthWeighted),
        kSmoothingProperty),
    Authored(
        Enum("frameTransport", rigExec::RigExecDeltaMushFrameTransport::Corner),
        kFrameTransportProperty),
    Count("smoothWeightCount"),
    Authored(
        Items("smoothWeights", rigExecStd::FloatType(), kSmoothWeightCount),
        kSmoothWeightsProperty),
    Count("edgeCount"),
    Authored(Items("edges", rigExecStd::IntType(), kEdgeCount), kEdgesProperty),
    Authored(Flag("onlySmooth"), kOnlySmoothProperty),
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
    Authored({&DeltaMushRest(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
              "restPoints"},
             kRestPoints),
    {&DeltaMushCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&DeltaMushIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    Authored({&DeltaMushComputationToTarget(), &rigExecStd::Mat4dType(),
              kOptional, "computationToTarget"},
             kComputationToTarget),
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
    d.schema = &kMushSchema;
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.validate = &Validate;
    d.range.prepare = &Prepare;
    d.range.run = &MushRange;
    d.range.reads = &MushReads;
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
