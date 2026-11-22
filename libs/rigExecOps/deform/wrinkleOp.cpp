#include "rigExecOps/deform/wrinkleOp.h"

#include "rigExecMath/wrinkleKernel.h"
#include "rigExecOps/opIo.h"

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

void
RegisterWrinkleOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &ints = rigExecStd::IntArray();
    static const rigExec::RigExecPortDesc ports[] = {
        {&WrinklePoints(), &points, kRigExecPortInput, "points"},
        {&WrinkleRest(), &points, kRigExecPortInput, "restPoints"},
        {&WrinkleCounts(), &ints, kRigExecPortInput, "faceVertexCounts"},
        {&WrinkleIndices(), &ints, kRigExecPortInput, "faceVertexIndices"},
        {&WrinkleEnvelope(), &rigExecStd::FloatArray(),
         kRigExecPortInput | kRigExecPortOptional, "envelope"},
        {&WrinkleOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "Wrinkle";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &Run;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<WrinklePayload>();
        return d;
    }();
    registrar.Add(Wrinkle(), desc);
}

} // namespace rigExecOps
