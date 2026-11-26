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

rigExec::RigExecPropertyInfo
ScaleInInfo(const char *doc)
{
    rigExec::RigExecPropertyInfo info;
    info.schemaName = "inputs:in";
    info.doc = doc;
    return info;
}

const rigExec::RigExecPropertyInfo kScaleIn = ScaleInInfo("The value scaled.");
const rigExec::RigExecPropertyInfo kScaleInRedoc =
    ScaleInInfo("The value to scale.");

const rigExec::RigExecSchemaInfo kScaleSchema{
    "FakeScale", "Scales a value.", "Typed",
    rigExec::RigExecSchemaMode::Generated};
const rigExec::RigExecSchemaInfo kScaleSchemaRedoc{
    "FakeScale", "Multiplies a value.", "Typed",
    rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecPortDesc kScalePorts[] = {
    {&ScaleIn(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortInput, "in",
     rigExec::RigExecOutputBound::None, nullptr, nullptr,
     rigExec::kRigExecInvalidIndex, 1, &kScaleIn},
    {&ScaleOut(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortOutput, "out"},
};

const rigExec::RigExecPortDesc kScalePortsRedoc[] = {
    {&ScaleIn(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortInput, "in",
     rigExec::RigExecOutputBound::None, nullptr, nullptr,
     rigExec::kRigExecInvalidIndex, 1, &kScaleInRedoc},
    {&ScaleOut(), &fakeTypes::Vec3Type(), rigExec::kRigExecPortOutput, "out"},
};

rigExec::RigExecOpDesc
ScaleDesc(const rigExec::RigExecPortDesc *ports,
          const rigExec::RigExecSchemaInfo &schema, uint32_t version)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "fake";
    d.name = "Scale";
    d.version = version;
    d.ports = ports;
    d.portCount = 2;
    d.run = &RunScale;
    d.payload = rigExec::RigExecPayloadHandlingFor<ScalePayload>();
    d.schema = &schema;
    return d;
}

const rigExec::RigExecOpDesc kScale = ScaleDesc(kScalePorts, kScaleSchema, 1);
const rigExec::RigExecOpDesc kScaleRedoc =
    ScaleDesc(kScalePortsRedoc, kScaleSchemaRedoc, 1);
const rigExec::RigExecOpDesc kScaleVersion2 =
    ScaleDesc(kScalePorts, kScaleSchema, 2);

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

void
RegisterRedoc(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Scale(), kScaleRedoc);
    registrar.Add(Offset(), kOffset);
}

void
RegisterVersion2(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Scale(), kScaleVersion2);
    registrar.Add(Offset(), kOffset);
}

rigExec::RigExecOpLibraryDesc
Library(void (*registerOps)(rigExec::RigExecOpRegistrar &))
{
    rigExec::RigExecOpLibraryDesc d;
    d.name = "fakeOps";
    d.registerOps = registerOps;
    return d;
}

const rigExec::RigExecOpLibraryDesc kLibrary = Library(&Register);
const rigExec::RigExecOpLibraryDesc kLibraryRedoc = Library(&RegisterRedoc);
const rigExec::RigExecOpLibraryDesc kLibraryVersion2 =
    Library(&RegisterVersion2);

} // namespace

const rigExec::RigExecOpLibraryDesc &
RedocumentedLibrary()
{
    return kLibraryRedoc;
}

const rigExec::RigExecOpLibraryDesc &
Version2Library()
{
    return kLibraryVersion2;
}
} // namespace fakeOps

RIGEXEC_OP_LIBRARY_ENTRY(rigExecTestFakeOps)
{
    return &fakeOps::kLibrary;
}
