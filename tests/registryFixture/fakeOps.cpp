#include "fakeOps.h"
#include "fakeTypes.h"

namespace fakeOps {

RIGEXEC_DEFINE_OP_HANDLE(Scale)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleIn)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleOut)
RIGEXEC_DEFINE_OP_HANDLE(Offset)
RIGEXEC_DEFINE_PORT_HANDLE(OffsetIn)
RIGEXEC_DEFINE_PORT_HANDLE(OffsetOut)

namespace {

using fakeTypes::Vec3;

bool
RunScale(const void *payload, void *, const rigExec::RigExecOpIo &io,
         std::string *)
{
    const float s = static_cast<const ScalePayload *>(payload)->scale;
    const Vec3 &in = *static_cast<const Vec3 *>(io.inputs[ScaleIn().Index()]);
    Vec3 &out = *static_cast<Vec3 *>(io.outputs[ScaleOut().Index()]);
    out = Vec3{in.x * s, in.y * s, in.z * s};
    return true;
}

bool
RunOffset(const void *, void *, const rigExec::RigExecOpIo &io, std::string *)
{
    const Vec3 &in = *static_cast<const Vec3 *>(io.inputs[OffsetIn().Index()]);
    Vec3 &out = *static_cast<Vec3 *>(io.outputs[OffsetOut().Index()]);
    out = Vec3{in.x + 1.0f, in.y + 1.0f, in.z + 1.0f};
    return true;
}

void
CopyPayload(void *dst, const void *src)
{
    *static_cast<ScalePayload *>(dst) = *static_cast<const ScalePayload *>(src);
}

void
DestroyPayload(void *)
{
}

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    static const rigExec::RigExecPortDesc scalePorts[] = {
        {&ScaleIn(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortInput, "in"},
        {&ScaleOut(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortOutput,
         "out"},
    };
    static const rigExec::RigExecOpDesc scale = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "fake";
        d.name = "Scale";
        d.ports = scalePorts;
        d.portCount = 2;
        d.run = &RunScale;
        d.payload.size = sizeof(ScalePayload);
        d.payload.align = alignof(ScalePayload);
        d.payload.copy = &CopyPayload;
        d.payload.destroy = &DestroyPayload;
        return d;
    }();
    static const rigExec::RigExecPortDesc offsetPorts[] = {
        {&OffsetIn(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortInput, "in"},
        {&OffsetOut(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortOutput,
         "out"},
    };
    static const rigExec::RigExecOpDesc offset = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "fake";
        d.name = "Offset";
        d.ports = offsetPorts;
        d.portCount = 2;
        d.run = &RunOffset;
        return d;
    }();
    registrar.Add(Scale(), scale);
    registrar.Add(Offset(), offset);
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
