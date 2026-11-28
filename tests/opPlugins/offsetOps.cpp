// An op plugin for the discovery tests: one op that moves points by an
// offset, its library entry, and a binder for the TestOffsetMover schema
// type its codeless schema adds.
#include "rigExec/api/opPorts.h"
#include "rigExec/api/registry.h"
#include "rigExecScene/externalOps.h"
#include "rigExecStdTypes/stdTypes.h"
#include "testOffsetOpsSchema.h"

#include <iterator>
#include <string>

namespace {

using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;

RIGEXEC_DEFINE_OP_HANDLE(Offset)
RIGEXEC_DEFINE_PORT_HANDLE(OffsetPoints)
RIGEXEC_DEFINE_PORT_HANDLE(OffsetBy)
RIGEXEC_DEFINE_PORT_HANDLE(OffsetOut)

bool
Run(const void *, void *, const rigExec::RigExecOpIo &io, std::string *)
{
    using rigExec::opPorts::In;
    using rigExec::opPorts::Out;
    const Points &in = *In<Points>(io, OffsetPoints());
    const Vec3f &by = *In<Vec3f>(io, OffsetBy());
    Points &out = Out<Points>(io, OffsetOut());
    out.Assign(in.Size());
    for (size_t i = 0; i < in.Size(); ++i)
        out[i] = Vec3f(in[i][0] + by[0], in[i][1] + by[1], in[i][2] + by[2]);
    return true;
}

const rigExec::RigExecPortDesc kPorts[] = {
    {&OffsetPoints(), &rigExecStd::Vec3fArray(), rigExec::kRigExecPortInput,
     "points"},
    {&OffsetBy(), &rigExecStd::Vec3fType(), rigExec::kRigExecPortInput, "by"},
    {&OffsetOut(), &rigExecStd::Vec3fArray(), rigExec::kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &OffsetPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "test";
    d.name = "Offset";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    return d;
}();

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Offset(), kDesc);
}

const rigExec::RigExecOpLibraryDesc kLibrary = [] {
    rigExec::RigExecOpLibraryDesc d;
    d.name = "testOffsetOps";
    d.registerOps = &Register;
    return d;
}();

bool
Build(const rigExec::RigExecRevisionBinding &, rigExec::RigExecRigDescBuilder &,
      rigExec::RigExecExternalOpPlan &, std::string *error)
{
    if (error)
        *error = "the offset binder binds no mover";
    return false;
}

const rigExec::RigExecExternalOpBinder kBinders[] = {
    {sizeof(rigExec::RigExecExternalOpBinder),
     rigExec::testOffsetOpsSchema::Type, &Offset, &Build},
};

const rigExec::RigExecExternalOpBinderTable kBinderTable = {
    rigExec::kRigExecExternalOpBinderAbiVersion, kBinders, std::size(kBinders)};

} // namespace

/// The op and port handles the discovery test evaluates through.
const rigExec::RigExecOpHandle &
TestOffsetOp()
{
    return Offset();
}

RIGEXEC_OP_LIBRARY_ENTRY(testOffsetOps)
{
    return &kLibrary;
}

RIGEXEC_EXTERNAL_OP_BINDERS_ENTRY(testOffsetOps)
{
    return &kBinderTable;
}
