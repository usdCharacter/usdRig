#include "rigExecOps/deform/wrinkleOp.h"

#include "rigExecMath/wrinkleKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"

#include <cstdint>
#include <iterator>
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

bool
WrinkleOne(const rigExec::RigExecOpIo &io,
           const rigExec::RigExecWrinkleSettingsView &settings,
           std::string *error)
{
    const Points &in = *In<Points>(io, WrinklePoints());
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecApplyWrinkleKernel<Vec3f, rigExecStd::Vec3d>(
            &points, ToVector(*In<Points>(io, WrinkleRest())),
            ToVector(*In<RigExecArray<int>>(io, WrinkleCounts())),
            ToVector(*In<RigExecArray<int>>(io, WrinkleIndices())), settings))
        return Fail(error, "Wrinkle",
                    "unusable settings, topology, rest or points");
    return WritePoints(io, WrinkleOut(), in, points,
                       In<Envelope>(io, WrinkleEnvelope()), "Wrinkle", error);
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return WrinkleOne(rigExec::RigExecBatchInstance(batch, k),
                          SettingsAt(batch, k), error);
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

const rigExec::RigExecFieldDesc kFields[] = {
    Value("iterations", rigExecStd::IntType()),
    Enum("topology", rigExec::RigExecWrinkleTopology::SurfaceStruts),
    Value("neighborDistance", rigExecStd::IntType()),
    Value("restLengthScale", rigExecStd::FloatType()),
    Value("stretchStiffness", rigExecStd::FloatType()),
    Value("compressionStiffness", rigExecStd::FloatType()),
    Value("bendStiffness", rigExecStd::FloatType()),
    Value("maxDisplacement", rigExecStd::FloatType()),
    Flag("pinBorders"),
    Flag("tangentPlaneCollisions"),
    Value("tangentPlaneInset", rigExecStd::FloatType()),
    Value("wrinkleScale", rigExecStd::FloatType()),
    Value("smoothingIterations", rigExecStd::IntType()),
    Count("pinPointCount"),
    Items("pinPoints", rigExecStd::IntType(), kPinPointCount),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&WrinklePoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&WrinkleRest(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "restPoints"},
    {&WrinkleCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&WrinkleIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&WrinkleEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&WrinkleOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Wrinkle";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    return d;
}();

} // namespace

void
RegisterWrinkleOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Wrinkle(), kDesc);
}

} // namespace rigExecOps
