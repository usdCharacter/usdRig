#include "rigExecOps/deform/wrinkleOp.h"

#include "rigExecMath/wrinkleKernel.h"
#include "rigExecOps/authoring.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecSchema/schemaNames.h"

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

/// A node's wrinkle topology, built on its first run.
struct WrinkleState {
    std::shared_ptr<const rigExec::RigExecWrinkleMesh> mesh;
};

/// Topology is fixed per compiled rig and the state lives as long as the
/// bound program, so the mesh is keyed by its sizes and payload rule only;
/// the apply half walks the mesh's own faces, never the caller's.
const rigExec::RigExecWrinkleMesh *
MeshOf(WrinkleState &state, size_t pointCount, const RigExecArray<int> &counts,
       const RigExecArray<int> &indices,
       const rigExec::RigExecWrinkleSettingsView &settings)
{
    const rigExec::RigExecWrinkleMesh *mesh = state.mesh.get();
    if (mesh && mesh->pointCount == pointCount &&
        mesh->counts.size() == counts.Size() &&
        mesh->indices.size() == indices.Size() &&
        mesh->mode == settings.topology &&
        mesh->neighborDistance == settings.neighborDistance)
        return mesh;
    state.mesh.reset();
    auto built = std::make_shared<rigExec::RigExecWrinkleMesh>();
    if (!rigExec::RigExecBuildWrinkleMesh(
            pointCount, ToVector(counts), ToVector(indices), settings.topology,
            settings.neighborDistance, built.get()))
        return nullptr;
    state.mesh = std::move(built);
    return state.mesh.get();
}

bool
WrinkleOne(const rigExec::RigExecOpIo &io,
           const rigExec::RigExecWrinkleSettingsView &settings,
           WrinkleState &state, std::string *error)
{
    const Points &in = *In<Points>(io, WrinklePoints());
    const Points &rest = *In<Points>(io, WrinkleRest());
    std::vector<Vec3f> points = ToVector(in);
    const rigExec::RigExecWrinkleMesh *mesh =
        MeshOf(state, rest.Size(), *In<RigExecArray<int>>(io, WrinkleCounts()),
               *In<RigExecArray<int>>(io, WrinkleIndices()), settings);
    if (!mesh ||
        !rigExec::RigExecApplyWrinkleWithTopology<Vec3f, rigExecStd::Vec3d>(
            &points, ToVector(rest), mesh->counts, mesh->indices, settings,
            *mesh))
        return Fail(error, "Wrinkle",
                    "unusable settings, topology, rest or points");
    return WritePoints(io, WrinkleOut(), in, points,
                       In<Envelope>(io, WrinkleEnvelope()), "Wrinkle", error);
}

bool
Check(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (!SameCount(CheckElements(check, WrinkleRest()),
                   CheckElements(check, WrinklePoints())))
        return Fail(error, "Wrinkle", "needs one rest point per point");
    return EnvelopeFits(check, WrinklePoints(), WrinkleEnvelope(), "Wrinkle",
                        error);
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return WrinkleOne(
            rigExec::RigExecBatchInstance(batch, k), SettingsAt(batch, k),
            *static_cast<WrinkleState *>(rigExec::RigExecBatchState(batch, k)),
            error);
    });
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

const rigExec::RigExecPortDesc kPorts[] = {
    {&WrinklePoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    Authored({&WrinkleRest(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
              "restPoints"},
             kRestPoints),
    {&WrinkleCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&WrinkleIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&WrinkleEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&WrinkleOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &WrinklePoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Wrinkle";
    d.schema = &kWrinkleSchema;
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &Run;
    d.validate = &Check;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
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
