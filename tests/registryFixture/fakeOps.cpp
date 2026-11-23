#include "fakeOps.h"
#include "fakeTypes.h"

#include "rigExec/api/opPorts.h"

namespace fakeOps {

RIGEXEC_DEFINE_OP_HANDLE(Scale)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleIn)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleOut)
RIGEXEC_DEFINE_OP_HANDLE(Offset)
RIGEXEC_DEFINE_PORT_HANDLE(OffsetIn)
RIGEXEC_DEFINE_PORT_HANDLE(OffsetOut)

namespace {

using fakeTypes::Vec3;
using rigExec::opPorts::In;
using rigExec::opPorts::Out;

bool
RunScale(const void *payload, void *, const rigExec::RigExecOpIo &io,
         std::string *)
{
    const float s = static_cast<const ScalePayload *>(payload)->scale;
    const Vec3 &in = *In<Vec3>(io, ScaleIn());
    Out<Vec3>(io, ScaleOut()) = Vec3{in.x * s, in.y * s, in.z * s};
    return true;
}

bool
RunOffset(const void *, void *, const rigExec::RigExecOpIo &io, std::string *)
{
    const Vec3 &in = *In<Vec3>(io, OffsetIn());
    Out<Vec3>(io, OffsetOut()) = Vec3{in.x + 1.0f, in.y + 1.0f, in.z + 1.0f};
    return true;
}

const rigExec::RigExecPortDesc kScalePorts[] = {
    {&ScaleIn(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortInput, "in"},
    {&ScaleOut(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kScale = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "fake";
    d.name = "Scale";
    d.ports = kScalePorts;
    d.portCount = 2;
    d.run = &RunScale;
    d.payload = rigExec::RigExecPayloadHandlingFor<ScalePayload>();
    return d;
}();

const rigExec::RigExecPortDesc kOffsetPorts[] = {
    {&OffsetIn(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortInput, "in"},
    {&OffsetOut(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kOffset = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "fake";
    d.name = "Offset";
    d.ports = kOffsetPorts;
    d.portCount = 2;
    d.run = &RunOffset;
    return d;
}();

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Scale(), kScale);
    registrar.Add(Offset(), kOffset);
}

const rigExec::RigExecOpLibraryDesc kLibrary = [] {
    rigExec::RigExecOpLibraryDesc d;
    d.name = "fakeOps";
    d.registerOps = &Register;
    return d;
}();

} // namespace
} // namespace fakeOps

RIGEXEC_OP_LIBRARY_ENTRY(rigExecTestFakeOps)
{
    return &fakeOps::kLibrary;
}
