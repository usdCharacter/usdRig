#include "rigExecOps/deform/derivedOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
#include "rigExecMath/meshBoundsKernel.h"
#include "rigExecMath/normalsLayoutKernel.h"
#include "rigExecMath/normalsLayoutKernel.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(RecomputeNormals)
RIGEXEC_DEFINE_OP_HANDLE(RecomputeExtent)

RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsPoints)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsCounts)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsIndices)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeNormalsOut)

RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentPoints)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentWidths)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentAuthored)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(RecomputeExtentOut)

namespace {

using rigExec::RigExecOpIo;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using namespace opIo;

// Writes \p values to \p port, blended from \p authored by \p envelope,
// as the derived revision kernel does.
bool
WriteDerived(const RigExecOpIo &io, const rigExec::RigExecPortHandle &port,
             std::vector<Vec3f> values, const Points *authored,
             const Envelope *envelope, const char *op, std::string *error)
{
    if (authored && authored->Size() != values.size())
        return Fail(error, op, "authored length differs from the result");
    if (envelope && !authored)
        return Fail(error, op, "an envelope needs the authored value");
    if (envelope) {
        if (envelope->Size() != values.size())
            return Fail(error, op, "envelope length differs from the result");
        for (size_t i = 0; i < values.size(); ++i)
            values[i] = rigExec::RigExecBlendEnvelope((*authored)[i], values[i],
                                                      (*envelope)[i]);
    }
    Out<Points>(io, port) = Points(values.data(), values.size());
    return true;
}

bool
RunNormals(const void *payload, void *, const RigExecOpIo &io,
           std::string *error)
{
    const NormalsPayload &p = *static_cast<const NormalsPayload *>(payload);
    const Points &points = *In<Points>(io, RecomputeNormalsPoints());
    const auto &counts = *In<RigExecArray<int>>(io, RecomputeNormalsCounts());
    const auto &indices = *In<RigExecArray<int>>(io, RecomputeNormalsIndices());
    if (points.Empty())
        return Fail(error, "RecomputeNormals", "no points");
    std::vector<Vec3f> normals = rigExec::RigExecComputeVertexNormalsKernel(
        points.Data(), points.Size(), counts.Data(), counts.Size(),
        indices.Data(), indices.Size());
    if (!rigExec::RigExecLayoutNormals(&normals, indices.Data(), indices.Size(),
                                       p.perCorner, p.leftHanded))
        return Fail(error, "RecomputeNormals", "a corner names no point");
    return WriteDerived(io, RecomputeNormalsOut(), std::move(normals),
                        In<Points>(io, RecomputeNormalsAuthored()),
                        In<Envelope>(io, RecomputeNormalsEnvelope()),
                        "RecomputeNormals", error);
}

bool
RunExtent(const void *, void *, const RigExecOpIo &io, std::string *error)
{
    const Points &points = *In<Points>(io, RecomputeExtentPoints());
    const auto *widths = In<RigExecArray<float>>(io, RecomputeExtentWidths());
    if (points.Empty())
        return Fail(error, "RecomputeExtent", "no points");
    return WriteDerived(
        io, RecomputeExtentOut(),
        rigExec::RigExecComputeExtentKernel(points.Data(), points.Size(),
                                            widths ? widths->Data() : nullptr,
                                            widths ? widths->Size() : 0),
        In<Points>(io, RecomputeExtentAuthored()),
        In<Envelope>(io, RecomputeExtentEnvelope()), "RecomputeExtent", error);
}

rigExec::RigExecOpDesc
DerivedDesc(const char *name, const rigExec::RigExecPortDesc *ports,
            size_t portCount, rigExec::RigExecOpRunFn run,
            uint32_t capabilities)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.run = run;
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

const rigExec::RigExecPortDesc kNormalsPorts[] = {
    {&RecomputeNormalsPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&RecomputeNormalsCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&RecomputeNormalsIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&RecomputeNormalsAuthored(), &rigExecStd::Vec3fArray(), kOptional,
     "authored"},
    {&RecomputeNormalsEnvelope(), &rigExecStd::FloatArray(), kOptional,
     "envelope"},
    {&RecomputeNormalsOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecPortDesc kExtentPorts[] = {
    {&RecomputeExtentPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&RecomputeExtentWidths(), &rigExecStd::FloatArray(), kOptional, "widths"},
    {&RecomputeExtentAuthored(), &rigExecStd::Vec3fArray(), kOptional,
     "authored"},
    {&RecomputeExtentEnvelope(), &rigExecStd::FloatArray(), kOptional,
     "envelope"},
    {&RecomputeExtentOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput,
     "out"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const NormalsPayload &p)
{
    w.Version(1);
    w.Bool(p.perCorner);
    w.Bool(p.leftHanded);
}

void
Load(Reader &r, NormalsPayload &p)
{
    r.Version(1);
    p.perCorner = r.Bool();
    p.leftHanded = r.Bool();
}

const rigExec::RigExecOpDesc kNormalsDesc = [] {
    rigExec::RigExecOpDesc d =
        DerivedDesc("RecomputeNormals", kNormalsPorts, std::size(kNormalsPorts),
                    &RunNormals, 0);
    d.payload =
        rigExec::RigExecPayloadHandlingFor<NormalsPayload, &Save, &Load>();
    return d;
}();

const rigExec::RigExecOpDesc kExtentDesc = DerivedDesc(
    "RecomputeExtent", kExtentPorts, std::size(kExtentPorts), &RunExtent, 0);

} // namespace

void
RegisterDerivedOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(RecomputeNormals(), kNormalsDesc);
    registrar.Add(RecomputeExtent(), kExtentDesc);
}

} // namespace rigExecOps
