#include "rigExecOps/deform/wrinkleOp.h"

#include "rigExecMath/wrinkleKernel.h"
#include "rigExecOps/authoring.h"
#include "rigExecOps/deform/weightPacket.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecSchema/schemaNames.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Wrinkle)

RIGEXEC_DEFINE_PORT_HANDLE(WrinklePoints)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleRest)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleCounts)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleIndices)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleOut)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleBase)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleSettings)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleTopology)
RIGEXEC_DEFINE_PORT_HANDLE(WrinklePinPoints)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleDefaultWeight)
RIGEXEC_DEFINE_PORT_HANDLE(WrinklePacket)
RIGEXEC_DEFINE_PORT_HANDLE(WrinkleWritten)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

enum WrinkleField : uint32_t {
    kIterations,
    kTopology,
    kNeighborDistance,
    kRestLengthScale,
    kStretchStiffness,
    kCompressionStiffness,
    kBendStiffness,
    kMaxDisplacement,
    kPinBorders,
    kTangentPlaneCollisions,
    kTangentPlaneInset,
    kWrinkleScale,
    kSmoothingIterations,
    kPinPointCount,
    kPinPoints,
};

/// Instance \p k's settings.
rigExec::RigExecWrinkleSettingsView
SettingsAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    rigExec::RigExecWrinkleSettingsView s;
    s.iterations = ValueAt<int32_t>(batch, kIterations, k);
    s.topology = EnumAt<rigExec::RigExecWrinkleTopology>(batch, kTopology, k);
    s.neighborDistance = ValueAt<int32_t>(batch, kNeighborDistance, k);
    s.restLengthScale = ValueAt<float>(batch, kRestLengthScale, k);
    s.stretchStiffness = ValueAt<float>(batch, kStretchStiffness, k);
    s.compressionStiffness = ValueAt<float>(batch, kCompressionStiffness, k);
    s.bendStiffness = ValueAt<float>(batch, kBendStiffness, k);
    s.maxDisplacement = ValueAt<float>(batch, kMaxDisplacement, k);
    s.pinBorders = FlagAt(batch, kPinBorders, k);
    s.tangentPlaneCollisions = FlagAt(batch, kTangentPlaneCollisions, k);
    s.tangentPlaneInset = ValueAt<float>(batch, kTangentPlaneInset, k);
    s.wrinkleScale = ValueAt<float>(batch, kWrinkleScale, k);
    s.smoothingIterations = ValueAt<int32_t>(batch, kSmoothingIterations, k);
    const auto pins = ListAt<int32_t>(batch, kPinPoints, k);
    s.pinPoints = {pins.data, pins.Size()};
    return s;
}

/// A node's wrinkle topology, rebuilt only when the faces, point count or
/// payload rule move; this frame's pass through, settings, rest and weights,
/// and the points the kernel moves.
struct WrinkleState {
    std::shared_ptr<const rigExec::RigExecWrinkleMesh> mesh;
    std::vector<int> counts, indices;
    bool pass = false;
    rigExec::RigExecWrinkleSettingsView settings;
    std::vector<int> pins;
    std::vector<float> dense;
    std::vector<Vec3f> points, restPoints;
};

/// The apply half walks the mesh's own faces, never the caller's.
const rigExec::RigExecWrinkleMesh *
MeshOf(WrinkleState &state, size_t pointCount, const RigExecArray<int> &counts,
       const RigExecArray<int> &indices,
       const rigExec::RigExecWrinkleSettingsView &settings)
{
    const rigExec::RigExecWrinkleMesh *mesh = state.mesh.get();
    if (mesh && mesh->pointCount == pointCount &&
        std::equal(counts.begin(), counts.end(), state.counts.begin(),
                   state.counts.end()) &&
        std::equal(indices.begin(), indices.end(), state.indices.begin(),
                   state.indices.end()) &&
        mesh->mode == settings.topology &&
        mesh->neighborDistance == settings.neighborDistance)
        return mesh;
    state.mesh.reset();
    state.counts.assign(counts.begin(), counts.end());
    state.indices.assign(indices.begin(), indices.end());
    auto built = std::make_shared<rigExec::RigExecWrinkleMesh>();
    if (!rigExec::RigExecBuildWrinkleMesh(
            pointCount, state.counts, state.indices, settings.topology,
            settings.neighborDistance, built.get()))
        return nullptr;
    state.mesh = std::move(built);
    return state.mesh.get();
}

/// An integer setting the host wrote as a double; one outside int's range
/// reads -1, which every integer setting refuses.
int
IntOf(double v)
{
    return std::isfinite(v) && v >= -2147483648.0 && v <= 2147483647.0 ? int(v)
                                                                       : -1;
}

rigExec::RigExecRangePlan
Plan(const rigExec::RigExecOpBatch &, size_t)
{
    // The constraint solve couples every point.
    return rigExec::RigExecRangePlan::Whole;
}

bool
Check(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!Fed(check, WrinkleBase()) &&
        !SameCount(CheckElements(check, WrinkleRest()),
                   CheckElements(check, WrinklePoints())))
        return Fail(error, "Wrinkle", "needs one rest point per point");
    const int envelopes = int(Fed(check, WrinkleEnvelope())) +
                          int(Fed(check, WrinkleDefaultWeight())) +
                          int(Fed(check, WrinklePacket()));
    if (envelopes > 1)
        return Fail(error, "Wrinkle",
                    "one of an envelope, a default weight and a weight "
                    "packet");
    return EnvelopeFits(check, WrinklePoints(), WrinkleEnvelope(), "Wrinkle",
                        error);
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t count,
        std::string *error)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    WrinkleState &state =
        *static_cast<WrinkleState *>(rigExec::RigExecBatchState(batch, k));
    const uint8_t *enabled = In<uint8_t>(io, WrinkleEnabled());
    state.pass = enabled && !*enabled;
    state.dense.clear();
    if (state.pass)
        return true;
    if (!EnvelopeFits(*In<Points>(io, WrinklePoints()),
                      In<Envelope>(io, WrinkleEnvelope()), "Wrinkle", error))
        return false;
    rigExec::RigExecWrinkleSettingsView &s = state.settings;
    s = SettingsAt(batch, k);
    if (const auto *v = In<RigExecArray<double>>(io, WrinkleSettings())) {
        if (v->Size() != 12)
            return Fail(error, "Wrinkle", "the settings hold twelve values");
        const RigExecArray<double> &a = *v;
        s.iterations = IntOf(a[0]);
        s.neighborDistance = IntOf(a[1]);
        s.restLengthScale = float(a[2]);
        s.stretchStiffness = float(a[3]);
        s.compressionStiffness = float(a[4]);
        s.bendStiffness = float(a[5]);
        s.maxDisplacement = float(a[6]);
        s.pinBorders = a[7] != 0.0;
        s.tangentPlaneCollisions = a[8] != 0.0;
        s.tangentPlaneInset = float(a[9]);
        s.wrinkleScale = float(a[10]);
        s.smoothingIterations = IntOf(a[11]);
    }
    if (const int32_t *topology = In<int32_t>(io, WrinkleTopology())) {
        if (*topology != 0 && *topology != 1)
            return Fail(error, "Wrinkle",
                        "the topology names neither cloth nor surfaceStruts");
        s.topology = *topology == 0
                         ? rigExec::RigExecWrinkleTopology::Cloth
                         : rigExec::RigExecWrinkleTopology::SurfaceStruts;
    }
    if (const auto *pins = In<RigExecArray<int32_t>>(io, WrinklePinPoints())) {
        state.pins.assign(pins->begin(), pins->end());
        s.pinPoints = {state.pins.data(), state.pins.size()};
    }
    const Points *authored = In<Points>(io, WrinkleRest());
    const Points *rest =
        authored && authored->Size() ? authored : In<Points>(io, WrinkleBase());
    const auto &counts = *In<RigExecArray<int>>(io, WrinkleCounts());
    if (!rest || !rest->Size() || !counts.Size())
        return Fail(error, "Wrinkle", "no rest points or no faces");
    state.restPoints.assign(rest->begin(), rest->end());
    if (!MeshOf(state, rest->Size(), counts,
                *In<RigExecArray<int>>(io, WrinkleIndices()), s))
        return Fail(error, "Wrinkle",
                    "unusable settings, topology, rest or points");
    const char *why = "";
    if (!weightPacket::ResolveMemberWeights(
            In<float>(io, WrinkleDefaultWeight()),
            In<weightPacket::PointWeights>(io, WrinklePacket()),
            In<RigExecArray<int32_t>>(io, WrinkleWritten()), count,
            &state.dense, &why))
        return Fail(error, "Wrinkle", why);
    return true;
}

bool
WrinkleRange(const rigExec::RigExecOpBatch &batch, size_t k,
             const rigExec::RigExecOpRange &range, std::string *error)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    WrinkleState &state =
        *static_cast<WrinkleState *>(rigExec::RigExecBatchState(batch, k));
    const Vec3f *from = RangeIn<Vec3f>(io, range, WrinklePoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, WrinkleOut());
    const size_t n = range.end - range.begin;
    if (state.pass) {
        std::copy(from, from + n, out);
        return true;
    }
    state.points.assign(from, from + n);
    const rigExec::RigExecWrinkleMesh &mesh = *state.mesh;
    if (!rigExec::RigExecApplyWrinkleWithTopology<Vec3f, rigExecStd::Vec3d>(
            &state.points, state.restPoints, mesh.counts, mesh.indices,
            state.settings, mesh))
        return Fail(error, "Wrinkle",
                    "unusable settings, topology, rest or points");
    std::copy(state.points.begin(), state.points.end(), out);
    BlendEnvelopeAt(from,
                    state.dense.empty()
                        ? RangeIn<float>(io, range, WrinkleEnvelope())
                        : state.dense.data() + range.begin,
                    out, n);
    return true;
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const rigExec::RigExecWrinkleSettings &s =
        static_cast<const WrinklePayload *>(payload)->settings;
    out.Set(kIterations, int32_t(s.iterations));
    SetEnum(out, kTopology, s.topology);
    out.Set(kNeighborDistance, int32_t(s.neighborDistance));
    out.Set(kRestLengthScale, s.restLengthScale);
    out.Set(kStretchStiffness, s.stretchStiffness);
    out.Set(kCompressionStiffness, s.compressionStiffness);
    out.Set(kBendStiffness, s.bendStiffness);
    out.Set(kMaxDisplacement, s.maxDisplacement);
    SetFlag(out, kPinBorders, s.pinBorders);
    SetFlag(out, kTangentPlaneCollisions, s.tangentPlaneCollisions);
    out.Set(kTangentPlaneInset, s.tangentPlaneInset);
    out.Set(kWrinkleScale, s.wrinkleScale);
    out.Set(kSmoothingIterations, int32_t(s.smoothingIterations));
    SetList(out, kPinPointCount, s.pinPoints);
}

namespace names = rigExec::SchemaAttrText;
namespace values = rigExec::SchemaValueText;
namespace types = rigExec::SchemaTypeName;
using authoring::Authored;

const rigExecStd::RigExecArray<rigExecStd::Vec3f> kNoPoints;
const rigExecStd::RigExecArray<int32_t> kNoInts;
const int32_t kDefaultIterations = 80;
const uint32_t kClothTopology =
    uint32_t(rigExec::RigExecWrinkleTopology::Cloth);
const int32_t kDefaultNeighborDistance = 2;
const float kOne = 1.0f;
const float kZero = 0.0f;
const float kDefaultBendStiffness = 0.1f;
const float kDefaultMaxDisplacement = 0.2f;
const bool kTrue = true;
const int32_t kNoIterations = 0;
const char *const kTopologies[] = {values::cloth, values::surfaceStruts};

const rigExec::RigExecPropertyInfo kRestPoints = authoring::WithRole(
    authoring::Uniform(
        names::inputsRestPoints, &kNoPoints,
        R"(Static, unconnected reference points in target mesh local space, with
the same ordering and count as the target. Empty uses the target's
authored base points at the evaluation time, before its mover chain.)"),
    rigExec::RigExecPropertyRole::Point);
const rigExec::RigExecPropertyInfo kIterationsProperty = authoring::Property(
    names::inputsIterations, &kDefaultIterations,
    R"(Constraint solver iteration count in [0,1000]. Zero passes through.)");
const rigExec::RigExecPropertyInfo kTopologyProperty = authoring::Tokens(
    names::inputsTopology, &kClothTopology, kTopologies,
    R"(Static, unconnected constraint topology. Both modes use triangulated
surface edges. Cloth adds weak across-triangle bend links; surfaceStruts
replaces these with weak links at exactly neighborDistance hops through
triangulated edge adjacency.)");
const rigExec::RigExecPropertyInfo kNeighborDistanceProperty =
    authoring::Property(
        names::inputsNeighborDistance, &kDefaultNeighborDistance,
        R"(Exact triangulated-edge graph distance for surfaceStruts, in [1,8].)");
const rigExec::RigExecPropertyInfo kRestLengthScaleProperty =
    authoring::Property(
        names::inputsRestLengthScale, &kOne,
        R"(Finite positive scale on reference constraint lengths.)");
const rigExec::RigExecPropertyInfo kStretchStiffnessProperty =
    authoring::Property(
        names::inputsStretchStiffness, &kOne,
        R"(Correction strength for structural links stretched in the incoming pose,
finite and in [0,1].)");
const rigExec::RigExecPropertyInfo kCompressionStiffnessProperty =
    authoring::Property(
        names::inputsCompressionStiffness, &kOne,
        R"(Correction strength for structural links compressed in the incoming
pose, finite and in [0,1].)");
const rigExec::RigExecPropertyInfo kBendStiffnessProperty = authoring::Property(
    names::inputsBendStiffness, &kDefaultBendStiffness,
    R"(Bend-link and surface-strut correction strength, finite and in [0,1].)");
const rigExec::RigExecPropertyInfo kMaxDisplacementProperty = authoring::Property(
    names::inputsMaxDisplacement, &kDefaultMaxDisplacement,
    R"(Finite nonnegative attachment radius around each incoming point in mesh
local units. The final displacement remains inside this radius,
including after wrinkleScale and displacement smoothing.)");
const rigExec::RigExecPropertyInfo kPinBordersProperty = authoring::Property(
    names::inputsPinBorders, &kTrue,
    R"(Keep vertices incident to a boundary edge at their incoming position.)");
const rigExec::RigExecPropertyInfo kPinPointsProperty = authoring::Uniform(
    names::inputsPinPoints, &kNoInts,
    R"(Static, unconnected, unique nonnegative indices of additional pinned
vertices.)");
const rigExec::RigExecPropertyInfo kTangentPlaneCollisionsProperty =
    authoring::Property(
        names::inputsTangentPlaneCollisions, &kTrue,
        R"(Prevent inward movement through each incoming vertex's local tangent
plane, offset inward by tangentPlaneInset. This is a local surface
approximation; there is no external or self-collision solve.)");
const rigExec::RigExecPropertyInfo kTangentPlaneInsetProperty =
    authoring::Property(
        names::inputsTangentPlaneInset, &kZero,
        R"(Finite nonnegative inward tangent-plane offset in mesh local units.)");
const rigExec::RigExecPropertyInfo kWrinkleScaleProperty = authoring::Property(
    names::inputsWrinkleScale, &kOne,
    R"(Finite nonnegative scale on solved displacement, still bounded by
maxDisplacement.)");
const rigExec::RigExecPropertyInfo kSmoothingIterationsProperty =
    authoring::Property(
        names::inputsSmoothingIterations, &kNoIterations,
        R"(Displacement-field averaging passes in [0,100]; not rest-detail
transport.)");

const rigExec::RigExecSchemaInfo kWrinkleSchema{
    types::RigExecWrinkleMover,
    R"(Adds automatic quasistatic wrinkles to one native mesh points property.
Reference lengths and a deterministic fold guide define target edge
vectors. Iterative projection adjusts the incoming shape in three
dimensions, subject to attachment bounds, pins, and optional local
tangent-plane collisions. The guide follows smooth compression without
previous-frame state, so playback and seeking agree. The common mover
envelope blends the result once.)",
    authoring::kTyped, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecFieldDesc kFields[] = {
    Authored(Value("iterations", rigExecStd::IntType()), kIterationsProperty),
    Authored(Enum("topology", rigExec::RigExecWrinkleTopology::SurfaceStruts),
             kTopologyProperty),
    Authored(Value("neighborDistance", rigExecStd::IntType()),
             kNeighborDistanceProperty),
    Authored(Value("restLengthScale", rigExecStd::FloatType()),
             kRestLengthScaleProperty),
    Authored(Value("stretchStiffness", rigExecStd::FloatType()),
             kStretchStiffnessProperty),
    Authored(Value("compressionStiffness", rigExecStd::FloatType()),
             kCompressionStiffnessProperty),
    Authored(Value("bendStiffness", rigExecStd::FloatType()),
             kBendStiffnessProperty),
    Authored(Value("maxDisplacement", rigExecStd::FloatType()),
             kMaxDisplacementProperty),
    Authored(Flag("pinBorders"), kPinBordersProperty),
    Authored(Flag("tangentPlaneCollisions"), kTangentPlaneCollisionsProperty),
    Authored(Value("tangentPlaneInset", rigExecStd::FloatType()),
             kTangentPlaneInsetProperty),
    Authored(Value("wrinkleScale", rigExecStd::FloatType()),
             kWrinkleScaleProperty),
    Authored(Value("smoothingIterations", rigExecStd::IntType()),
             kSmoothingIterationsProperty),
    Count("pinPointCount"),
    Authored(Items("pinPoints", rigExecStd::IntType(), kPinPointCount),
             kPinPointsProperty),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&WrinklePoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortPointwise, "points"},
    Authored({&WrinkleRest(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
              "restPoints"},
             kRestPoints),
    {&WrinkleCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&WrinkleIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&WrinkleEnvelope(), &rigExecStd::FloatArray(),
     kOptional | rigExec::kRigExecPortPointwise, "envelope"},
    {&WrinkleOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &WrinklePoints()},
    {&WrinkleBase(), &rigExecStd::Vec3fArray(), kOptional, "base"},
    {&WrinkleSettings(), &rigExecStd::DoubleArray(), kOptional, "settings"},
    {&WrinkleTopology(), &rigExecStd::IntType(), kOptional, "topology"},
    {&WrinklePinPoints(), &rigExecStd::IntArray(), kOptional, "pinPoints"},
    {&WrinkleEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    {&WrinkleDefaultWeight(), &rigExecStd::FloatType(), kOptional,
     "defaultWeight"},
    {&WrinklePacket(), &rigExecRig::PointWeightsType(), kOptional, "packet"},
    {&WrinkleWritten(), &rigExecStd::IntArray(), kOptional, "written"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Wrinkle";
    d.schema = &kWrinkleSchema;
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.validate = &Check;
    d.range.plan = &Plan;
    d.range.prepare = &Prepare;
    d.range.run = &WrinkleRange;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable |
                     rigExec::kRigExecOpSeparateBlend |
                     rigExec::kRigExecOpRanged | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    d.state = StateOf<WrinkleState>();
    return d;
}();

} // namespace

void
RegisterWrinkleOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Wrinkle(), kDesc);
}

} // namespace rigExecOps
