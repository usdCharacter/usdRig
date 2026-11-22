#include "rigExecOps/deform/derivedOps.h"

#include "rigExecOps/opIo.h"
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
            size_t portCount, rigExec::RigExecOpRunFn run)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.run = run;
    d.capabilities = rigExec::kRigExecOpBakeable;
    return d;
}

} // namespace

void
RegisterDerivedOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &ints = rigExecStd::IntArray();
    const auto &floats = rigExecStd::FloatArray();

    static const rigExec::RigExecPortDesc normalsPorts[] = {
        {&RecomputeNormalsPoints(), &points, kRigExecPortInput, "points"},
        {&RecomputeNormalsCounts(), &ints, kRigExecPortInput,
         "faceVertexCounts"},
        {&RecomputeNormalsIndices(), &ints, kRigExecPortInput,
         "faceVertexIndices"},
        {&RecomputeNormalsAuthored(), &points, kOptional, "authored"},
        {&RecomputeNormalsEnvelope(), &floats, kOptional, "envelope"},
        {&RecomputeNormalsOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc extentPorts[] = {
        {&RecomputeExtentPoints(), &points, kRigExecPortInput, "points"},
        {&RecomputeExtentWidths(), &floats, kOptional, "widths"},
        {&RecomputeExtentAuthored(), &points, kOptional, "authored"},
        {&RecomputeExtentEnvelope(), &floats, kOptional, "envelope"},
        {&RecomputeExtentOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc normalsDesc = [] {
        rigExec::RigExecOpDesc d =
            DerivedDesc("RecomputeNormals", normalsPorts,
                        std::size(normalsPorts), &RunNormals);
        d.payload = rigExec::RigExecPayloadHandlingFor<NormalsPayload>();
        return d;
    }();
    static const rigExec::RigExecOpDesc extentDesc = DerivedDesc(
        "RecomputeExtent", extentPorts, std::size(extentPorts), &RunExtent);
    registrar.Add(RecomputeNormals(), normalsDesc);
    registrar.Add(RecomputeExtent(), extentDesc);
}

} // namespace rigExecOps
