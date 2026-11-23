#include "rigExecOps/deform/wrinkleOp.h"

#include "rigExecMath/wrinkleKernel.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"

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
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

bool
Run(const void *payload, void *, const rigExec::RigExecOpIo &io,
    std::string *error)
{
    const WrinklePayload &p = *static_cast<const WrinklePayload *>(payload);
    const Points &in = *In<Points>(io, WrinklePoints());
    std::vector<Vec3f> points = ToVector(in);
    // The topology is rebuilt every run until step 9 gives ops state.
    if (!rigExec::RigExecApplyWrinkleKernel<Vec3f, rigExecStd::Vec3d>(
            &points, ToVector(*In<Points>(io, WrinkleRest())),
            ToVector(*In<RigExecArray<int>>(io, WrinkleCounts())),
            ToVector(*In<RigExecArray<int>>(io, WrinkleIndices())), p.settings))
        return Fail(error, "Wrinkle",
                    "unusable settings, topology, rest or points");
    return WritePoints(io, WrinkleOut(), in, points,
                       In<Envelope>(io, WrinkleEnvelope()), "Wrinkle", error);
}

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

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const WrinklePayload &p)
{
    const rigExec::RigExecWrinkleSettings &s = p.settings;
    w.Version(1);
    w.I32(s.iterations);
    w.Enum(s.topology);
    w.I32(s.neighborDistance);
    w.F32(s.restLengthScale);
    w.F32(s.stretchStiffness);
    w.F32(s.compressionStiffness);
    w.F32(s.bendStiffness);
    w.F32(s.maxDisplacement);
    w.Bool(s.pinBorders);
    w.Bool(s.tangentPlaneCollisions);
    w.F32(s.tangentPlaneInset);
    w.F32(s.wrinkleScale);
    w.I32(s.smoothingIterations);
    payloadIo::PutList(w, s.pinPoints);
}

void
Load(Reader &r, WrinklePayload &p)
{
    rigExec::RigExecWrinkleSettings &s = p.settings;
    r.Version(1);
    s.iterations = r.I32();
    s.topology = r.Enum(rigExec::RigExecWrinkleTopology::SurfaceStruts);
    s.neighborDistance = r.I32();
    s.restLengthScale = r.F32();
    s.stretchStiffness = r.F32();
    s.compressionStiffness = r.F32();
    s.bendStiffness = r.F32();
    s.maxDisplacement = r.F32();
    s.pinBorders = r.Bool();
    s.tangentPlaneCollisions = r.Bool();
    s.tangentPlaneInset = r.F32();
    s.wrinkleScale = r.F32();
    s.smoothingIterations = r.I32();
    payloadIo::GetList(r, s.pinPoints);
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Wrinkle";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<WrinklePayload, &Save, &Load>();
    return d;
}();

} // namespace

void
RegisterWrinkleOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Wrinkle(), kDesc);
}

} // namespace rigExecOps
