#include "rigExecOps/deform/deltaMushOp.h"

#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
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

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const DeltaMushPayload &p)
{
    w.Version(1);
    w.I32(p.iterations);
    w.F64(p.step);
    w.Bool(p.pinBorders);
    w.F64(p.distanceWeight);
    w.F64(p.displacement);
    w.Enum(p.settings.smoothing);
    w.Enum(p.settings.frameTransport);
    payloadIo::PutList(w, p.settings.smoothWeights);
    payloadIo::PutList(w, p.settings.edges);
    w.Bool(p.settings.onlySmooth);
}

void
Load(Reader &r, DeltaMushPayload &p)
{
    r.Version(1);
    p.iterations = r.I32();
    p.step = r.F64();
    p.pinBorders = r.Bool();
    p.distanceWeight = r.F64();
    p.displacement = r.F64();
    p.settings.smoothing =
        r.Enum(rigExec::RigExecDeltaMushSmoothing::LengthWeighted);
    p.settings.frameTransport =
        r.Enum(rigExec::RigExecDeltaMushFrameTransport::Corner);
    payloadIo::GetList(r, p.settings.smoothWeights);
    payloadIo::GetList(r, p.settings.edges);
    p.settings.onlySmooth = r.Bool();
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "DeltaMush";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
        rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<DeltaMushPayload, &Save, &Load>();
    return d;
}();

} // namespace

void
RegisterDeltaMushOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(DeltaMush(), kDesc);
}

} // namespace rigExecOps
