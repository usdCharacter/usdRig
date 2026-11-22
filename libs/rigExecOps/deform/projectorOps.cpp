#include "rigExecOps/deform/projectorOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/normalsLayoutKernel.h"
#include "rigExecMath/surfaceProjectorKernel.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SurfaceProjector)
RIGEXEC_DEFINE_OP_HANDLE(ShaderDials)

RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorBasePoints)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorPoints)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorCounts)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorIndices)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorSourceBase)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorSourceFinal)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorSourceSpaceBase)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorSourceSpaceFinal)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorSpaceFinal)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorWorldToMesh)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceProjectorOut)

RIGEXEC_DEFINE_PORT_HANDLE(ShaderDialsDials)
RIGEXEC_DEFINE_PORT_HANDLE(ShaderDialsOut)

namespace {

using rigExec::RigExecOpIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;

// Both of a pair, or neither.
bool
Pair(const Mat4d *a, const Mat4d *b, Mat4d *first, Mat4d *second)
{
    if (!a || !b)
        return false;
    *first = *a;
    *second = *b;
    return true;
}

bool
RunSurfaceProjector(const void *payload, void *, const RigExecOpIo &io,
                    std::string *error)
{
    const auto &p = *static_cast<const SurfaceProjectorPayload *>(payload);
    const Mat4d *sourceBase = In<Mat4d>(io, SurfaceProjectorSourceBase());
    const Mat4d *sourceFinal = In<Mat4d>(io, SurfaceProjectorSourceFinal());
    const Mat4d *spaceBase = In<Mat4d>(io, SurfaceProjectorSourceSpaceBase());
    const Mat4d *spaceFinal = In<Mat4d>(io, SurfaceProjectorSourceSpaceFinal());
    if (!sourceBase != !sourceFinal || !spaceBase != !spaceFinal)
        return Fail(error, "SurfaceProjector",
                    "a source or source space needs its base and final");
    rigExec::RigExecSurfaceProjectorInputs<Mat4d, Vec3d> in;
    in.rayOrigin = p.rayOrigin;
    in.rayDirection = p.rayDirection;
    in.rayUp = p.rayUp;
    in.shaderOffset = p.shaderOffset;
    in.reproject = p.reproject;
    in.sourceBase.SetIdentity();
    in.sourceFinal.SetIdentity();
    in.sourceSpaceBase.SetIdentity();
    in.sourceSpaceFinal.SetIdentity();
    in.hasSource =
        Pair(sourceBase, sourceFinal, &in.sourceBase, &in.sourceFinal);
    in.sourceSpaceNamed = p.sourceSpaceNamed;
    in.hasSourceSpace =
        p.sourceSpaceNamed &&
        Pair(spaceBase, spaceFinal, &in.sourceSpaceBase, &in.sourceSpaceFinal);
    in.spaceNamed = p.spaceNamed;
    const Mat4d *space = In<Mat4d>(io, SurfaceProjectorSpaceFinal());
    in.hasSpace = p.spaceNamed && space;
    in.spaceFinal = space ? *space : Mat4d(1.0);
    in.worldToMesh = InOr(io, SurfaceProjectorWorldToMesh(), Mat4d(1.0));

    const auto normals = [](const std::vector<Vec3f> &points,
                            const std::vector<int> &counts,
                            const std::vector<int> &indices) {
        return rigExec::RigExecComputeMeshVertexNormalsKernel(points, counts,
                                                              indices);
    };
    std::vector<std::string> diagnostics;
    Mat4d &out = Out<Mat4d>(io, SurfaceProjectorOut());
    if (!rigExec::RigExecSolveSurfaceProjectorT(
            in, ToVector(*In<Points>(io, SurfaceProjectorBasePoints())),
            ToVector(*In<Points>(io, SurfaceProjectorPoints())),
            ToVector(*In<RigExecArray<int>>(io, SurfaceProjectorCounts())),
            ToVector(*In<RigExecArray<int>>(io, SurfaceProjectorIndices())),
            normals, "", &out, &diagnostics)) {
        // The kernel prefixes each line with its own name.
        const std::string prefix = "SurfaceProjector : ";
        std::string why;
        for (const std::string &d : diagnostics)
            why += (why.empty() ? "" : "; ") +
                   (d.compare(0, prefix.size(), prefix) == 0
                        ? d.substr(prefix.size())
                        : d);
        return Fail(error, "SurfaceProjector",
                    why.empty() ? std::string("no frame measured") : why);
    }
    return true;
}

bool
RunShaderDials(const void *, void *, const RigExecOpIo &io, std::string *)
{
    Out<Mat4d>(io, ShaderDialsOut()) = rigExec::RigExecPackShaderDialsT<Mat4d>(
        ToVector(*In<RigExecArray<double>>(io, ShaderDialsDials())));
    return true;
}

rigExec::RigExecOpDesc
ProjectorDesc(const char *name, const rigExec::RigExecPortDesc *ports,
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
RegisterProjectorOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &ints = rigExecStd::IntArray();
    const auto &m = rigExecStd::Mat4dType();

    static const rigExec::RigExecPortDesc projectorPorts[] = {
        {&SurfaceProjectorBasePoints(), &points, kRigExecPortInput,
         "basePoints"},
        {&SurfaceProjectorPoints(), &points, kRigExecPortInput, "points"},
        {&SurfaceProjectorCounts(), &ints, kRigExecPortInput,
         "faceVertexCounts"},
        {&SurfaceProjectorIndices(), &ints, kRigExecPortInput,
         "faceVertexIndices"},
        {&SurfaceProjectorSourceBase(), &m, kOptional, "sourceBase"},
        {&SurfaceProjectorSourceFinal(), &m, kOptional, "sourceFinal"},
        {&SurfaceProjectorSourceSpaceBase(), &m, kOptional, "sourceSpaceBase"},
        {&SurfaceProjectorSourceSpaceFinal(), &m, kOptional,
         "sourceSpaceFinal"},
        {&SurfaceProjectorSpaceFinal(), &m, kOptional, "spaceFinal"},
        {&SurfaceProjectorWorldToMesh(), &m, kOptional, "worldToMesh"},
        {&SurfaceProjectorOut(), &m, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc dialPorts[] = {
        {&ShaderDialsDials(), &rigExecStd::DoubleArray(), kRigExecPortInput,
         "dials"},
        {&ShaderDialsOut(), &m, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc projectorDesc = [] {
        rigExec::RigExecOpDesc d =
            ProjectorDesc("SurfaceProjector", projectorPorts,
                          std::size(projectorPorts), &RunSurfaceProjector);
        d.payload =
            rigExec::RigExecPayloadHandlingFor<SurfaceProjectorPayload>();
        return d;
    }();
    static const rigExec::RigExecOpDesc dialDesc = ProjectorDesc(
        "ShaderDials", dialPorts, std::size(dialPorts), &RunShaderDials);
    registrar.Add(SurfaceProjector(), projectorDesc);
    registrar.Add(ShaderDials(), dialDesc);
}

} // namespace rigExecOps
