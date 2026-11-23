#include "rigExecOps/deform/deltaMushOp.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/deltaMushKernel.h"

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

bool
Run(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const DeltaMushPayload &p = *static_cast<const DeltaMushPayload *>(payload);
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

} // namespace

void
RegisterDeltaMushOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &ints = rigExecStd::IntArray();
    static const rigExec::RigExecPortDesc ports[] = {
        {&DeltaMushPoints(), &points, kRigExecPortInput, "points"},
        {&DeltaMushRest(), &points, kRigExecPortInput, "restPoints"},
        {&DeltaMushCounts(), &ints, kRigExecPortInput, "faceVertexCounts"},
        {&DeltaMushIndices(), &ints, kRigExecPortInput, "faceVertexIndices"},
        {&DeltaMushComputationToTarget(), &rigExecStd::Mat4dType(), kOptional,
         "computationToTarget"},
        {&DeltaMushEnvelope(), &rigExecStd::FloatArray(), kOptional,
         "envelope"},
        {&DeltaMushOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "DeltaMush";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &Run;
        d.capabilities =
            rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
            rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
            rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
        d.payload = rigExec::RigExecPayloadHandlingFor<DeltaMushPayload>();
        return d;
    }();
    registrar.Add(DeltaMush(), desc);
}

} // namespace rigExecOps
