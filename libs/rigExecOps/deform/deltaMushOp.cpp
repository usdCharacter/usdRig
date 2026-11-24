#include "rigExecOps/deform/deltaMushOp.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/deltaMushKernel.h"

#include <cstdint>
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

bool
MushOne(const RigExecOpIo &io, const MushInstance &p, std::string *error)
{
    const Points &in = *In<Points>(io, DeltaMushPoints());
    const Mat4d *map = In<Mat4d>(io, DeltaMushComputationToTarget());
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecApplyDeltaMushInSpaceKernel<Vec3f, rigExecStd::Vec3d,
                                                     Mat4d>(
            &points, ToVector(*In<Points>(io, DeltaMushRest())),
            ToVector(*In<RigExecArray<int>>(io, DeltaMushCounts())),
            ToVector(*In<RigExecArray<int>>(io, DeltaMushIndices())),
            p.iterations, p.step, p.pinBorders, p.distanceWeight,
            p.displacement, p.settings, map ? *map : Mat4d(1.0)))
        return Fail(error, "DeltaMush",
                    "unusable topology, rest or computation map");
    return WritePoints(io, DeltaMushOut(), in, points,
                       In<Envelope>(io, DeltaMushEnvelope()), "DeltaMush",
                       error);
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return MushOne(rigExec::RigExecBatchInstance(batch, k),
                       PayloadAt(batch, k), error);
    });
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
    {&DeltaMushPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&DeltaMushRest(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "restPoints"},
    {&DeltaMushCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&DeltaMushIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&DeltaMushComputationToTarget(), &rigExecStd::Mat4dType(), kOptional,
     "computationToTarget"},
    {&DeltaMushEnvelope(), &rigExecStd::FloatArray(), kOptional, "envelope"},
    {&DeltaMushOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "DeltaMush";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &Run;
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
