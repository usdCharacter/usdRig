#include "rigExecOps/deform/surfaceProjectOp.h"

#include "rigExecMath/surfaceProjectKernel.h"
#include "rigExecOps/opIo.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SurfaceProject)

RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectPoints)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectSurface)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectCounts)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectIndices)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectStrength)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectTargetToSurface)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectSurfaceToTarget)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectSurfaceToMetric)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectOut)

namespace {

using namespace opIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Ints = RigExecArray<int>;

bool
Run(const void *payload, void *, const rigExec::RigExecOpIo &io,
    std::string *error)
{
    const rigExec::RigExecSurfaceSnapSettings &settings =
        static_cast<const SurfaceProjectPayload *>(payload)->settings;
    const Points &in = *In<Points>(io, SurfaceProjectPoints());
    const Points &surface = *In<Points>(io, SurfaceProjectSurface());
    const Ints &counts = *In<Ints>(io, SurfaceProjectCounts());
    const Ints &indices = *In<Ints>(io, SurfaceProjectIndices());
    const Mat4d identity(1.0);
    const Mat4d toSurface = InOr(io, SurfaceProjectTargetToSurface(), identity);
    const Mat4d toTarget = InOr(io, SurfaceProjectSurfaceToTarget(), identity);
    const Mat4d toMetric = InOr(io, SurfaceProjectSurfaceToMetric(), identity);
    std::vector<Vec3f> points = ToVector(in);
    if (rigExec::RigExecSurfaceSnapIsLegacy(settings, toSurface, toTarget,
                                            toMetric)) {
        // Brute force; a BVH waits for ops to carry state.
        rigExec::RigExecApplySurfaceProjectKernel(
            &points, surface.Data(), surface.Size(), counts.Data(),
            counts.Size(), indices.Data(), indices.Size(),
            InOr(io, SurfaceProjectStrength(), 1.0f));
    } else if (!rigExec::RigExecApplySurfaceSnapKernel<Vec3f,
                                                       rigExecStd::Vec3d>(
                   &points, ToVector(surface), ToVector(counts),
                   ToVector(indices), settings, toSurface, toTarget,
                   toMetric)) {
        return Fail(error, "SurfaceProject",
                    "unusable surface, settings or spaces");
    }
    return WritePoints(io, SurfaceProjectOut(), in, points,
                       In<Envelope>(io, SurfaceProjectEnvelope()),
                       "SurfaceProject", error);
}

} // namespace

void
RegisterSurfaceProjectOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &ints = rigExecStd::IntArray();
    const auto &m = rigExecStd::Mat4dType();
    static const rigExec::RigExecPortDesc ports[] = {
        {&SurfaceProjectPoints(), &points, kRigExecPortInput, "points"},
        {&SurfaceProjectSurface(), &points, kRigExecPortInput, "surface"},
        {&SurfaceProjectCounts(), &ints, kRigExecPortInput, "faceVertexCounts"},
        {&SurfaceProjectIndices(), &ints, kRigExecPortInput,
         "faceVertexIndices"},
        {&SurfaceProjectStrength(), &rigExecStd::FloatType(), kOptional,
         "strength"},
        {&SurfaceProjectTargetToSurface(), &m, kOptional, "targetToSurface"},
        {&SurfaceProjectSurfaceToTarget(), &m, kOptional, "surfaceToTarget"},
        {&SurfaceProjectSurfaceToMetric(), &m, kOptional, "surfaceToMetric"},
        {&SurfaceProjectEnvelope(), &rigExecStd::FloatArray(), kOptional,
         "envelope"},
        {&SurfaceProjectOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "SurfaceProject";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &Run;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<SurfaceProjectPayload>();
        return d;
    }();
    registrar.Add(SurfaceProject(), desc);
}

} // namespace rigExecOps
