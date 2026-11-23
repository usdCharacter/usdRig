#include "rigExecOps/deform/projectorOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
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

const rigExec::RigExecPortDesc kProjectorPorts[] = {
    {&SurfaceProjectorBasePoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput, "basePoints"},
    {&SurfaceProjectorPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&SurfaceProjectorCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&SurfaceProjectorIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&SurfaceProjectorSourceBase(), &rigExecStd::Mat4dType(), kOptional,
     "sourceBase"},
    {&SurfaceProjectorSourceFinal(), &rigExecStd::Mat4dType(), kOptional,
     "sourceFinal"},
    {&SurfaceProjectorSourceSpaceBase(), &rigExecStd::Mat4dType(), kOptional,
     "sourceSpaceBase"},
    {&SurfaceProjectorSourceSpaceFinal(), &rigExecStd::Mat4dType(), kOptional,
     "sourceSpaceFinal"},
    {&SurfaceProjectorSpaceFinal(), &rigExecStd::Mat4dType(), kOptional,
     "spaceFinal"},
    {&SurfaceProjectorWorldToMesh(), &rigExecStd::Mat4dType(), kOptional,
     "worldToMesh"},
    {&SurfaceProjectorOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecPortDesc kDialPorts[] = {
    {&ShaderDialsDials(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "dials"},
    {&ShaderDialsOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const SurfaceProjectorPayload &p)
{
    w.Version(1);
    payloadIo::Put(w, p.rayOrigin);
    payloadIo::Put(w, p.rayDirection);
    payloadIo::Put(w, p.rayUp);
    payloadIo::Put(w, p.shaderOffset);
    w.Bool(p.reproject);
    w.Bool(p.sourceSpaceNamed);
    w.Bool(p.spaceNamed);
}

void
Load(Reader &r, SurfaceProjectorPayload &p)
{
    r.Version(1);
    payloadIo::Get(r, p.rayOrigin);
    payloadIo::Get(r, p.rayDirection);
    payloadIo::Get(r, p.rayUp);
    payloadIo::Get(r, p.shaderOffset);
    p.reproject = r.Bool();
    p.sourceSpaceNamed = r.Bool();
    p.spaceNamed = r.Bool();
}

const rigExec::RigExecOpDesc kProjectorDesc = [] {
    rigExec::RigExecOpDesc d = ProjectorDesc(
        "SurfaceProjector", kProjectorPorts, std::size(kProjectorPorts),
        &RunSurfaceProjector, rigExec::kRigExecOpSeparateBlend);
    d.payload = rigExec::RigExecPayloadHandlingFor<SurfaceProjectorPayload,
                                                   &Save, &Load>();
    return d;
}();

const rigExec::RigExecOpDesc kDialDesc =
    ProjectorDesc("ShaderDials", kDialPorts, std::size(kDialPorts),
                  &RunShaderDials, rigExec::kRigExecOpSeparateBlend);

} // namespace

void
RegisterProjectorOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SurfaceProjector(), kProjectorDesc);
    registrar.Add(ShaderDials(), kDialDesc);
}

} // namespace rigExecOps
