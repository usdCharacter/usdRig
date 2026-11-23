#include "rigExecOps/deform/pointOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/meshConnectivity.h"
#include "rigExecMath/smoothKernel.h"
#include "rigExecMath/meshBoundsKernel.h"

#include <iterator>
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

float
StrengthOr1(const float *strength)
{
    return strength ? *strength : 1.0f;
}

bool
SmoothOne(const RigExecOpIo &io, std::string *error)
{
    const Points &in = *In<Points>(io, SmoothPoints());
    const auto &counts = *In<RigExecArray<int>>(io, SmoothCounts());
    const auto &indices = *In<RigExecArray<int>>(io, SmoothIndices());
    Points &out = Out<Points>(io, SmoothOut());
    out = in;
    // An adjacency that fails to build covers nothing, so the group kernel
    // passes the points through, as the whole kernel does.
    rigExec::RigExecMeshAdjacency adjacency;
    rigExec::RigExecBuildMeshAdjacency(in.Size(), counts.Data(), counts.Size(),
                                       indices.Data(), indices.Size(),
                                       &adjacency);
    const int bounds[2] = {0, int(in.Size())};
    const Vec3f *const groupPoints[1] = {in.Data()};
    rigExec::RigExecGroupPoints<Vec3f> groups;
    groups.bounds = bounds;
    groups.groups = 1;
    groups.points = groupPoints;
    if (!in.Empty() &&
        !rigExec::RigExecApplyLaplacianSmoothGroup(
            adjacency, StrengthOr1(In<float>(io, SmoothStrength())), in.Size(),
            groups, 0, in.Size(), out.Data()))
        return Fail(error, "Smooth", "topology addresses no point");
    return BlendEnvelope(in, In<RigExecArray<float>>(io, SmoothEnvelope()),
                         &out, "Smooth", error);
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
RunSmooth(const rigExec::RigExecOpBatch &batch, size_t *failed,
          std::string *error)
{
    for (size_t k = 0; k < batch.count; ++k) {
        if (rigExec::RigExecBatchActive(batch, k) &&
            !SmoothOne(rigExec::RigExecBatchInstance(batch, k), error)) {
            *failed = k;
            return false;
        }
    }
    return true;
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

const rigExec::RigExecPortDesc kSmoothPorts[] = {
    {&SmoothPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&SmoothCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&SmoothIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&SmoothStrength(), &rigExecStd::FloatType(), kOptional, "strength"},
    {&SmoothEnvelope(), &rigExecStd::FloatArray(), kOptional, "envelope"},
    {&SmoothOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecPortDesc kVolumePorts[] = {
    {&VolumeCorrectPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&VolumeCorrectReference(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "referenceVolume"},
    {&VolumeCorrectStrength(), &rigExecStd::FloatType(), kOptional, "strength"},
    {&VolumeCorrectEnvelope(), &rigExecStd::FloatArray(), kOptional,
     "envelope"},
    {&VolumeCorrectOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kSmoothDesc =
    PointDesc("Smooth", kSmoothPorts, std::size(kSmoothPorts), &RunSmooth,
              rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
                  rigExec::kRigExecOpGate);

const rigExec::RigExecOpDesc kVolumeDesc =
    PointDesc("VolumeCorrect", kVolumePorts, std::size(kVolumePorts),
              &RunVolumeCorrect, rigExec::kRigExecOpSeparateBlend);

} // namespace

void
RegisterPointOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Smooth(), kSmoothDesc);
    registrar.Add(VolumeCorrect(), kVolumeDesc);
}

} // namespace rigExecOps
