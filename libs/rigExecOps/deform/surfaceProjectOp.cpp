#include "rigExecOps/deform/surfaceProjectOp.h"

#include "rigExecMath/surfaceProjectKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"

#include <cstdint>
#include <iterator>
#include <memory>
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
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Ints = RigExecArray<int>;

enum SurfaceProjectField : uint32_t {
    kMode,
    kOffset,
    kMaskCount,
    kMask,
    kTriangleCount,
    kTriangles,
};

/// Instance \p k's settings.
rigExec::RigExecSurfaceSnapSettingsView
SettingsAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    rigExec::RigExecSurfaceSnapSettingsView s;
    s.mode = EnumAt<rigExec::RigExecSurfaceSnapMode>(batch, kMode, k);
    s.offset = ValueAt<float>(batch, kOffset, k);
    const auto mask = ListAt<float>(batch, kMask, k);
    s.mask = {mask.data, mask.Size()};
    const auto triangles = ListAt<int32_t>(batch, kTriangles, k);
    s.triangles = {triangles.data, triangles.Size()};
    return s;
}

/// A surface's snap triangulation, from its faces or authored triangles.
struct SnapTriangles {
    size_t surfacePoints = 0;
    size_t counts = 0;
    size_t indices = 0;
    bool valid = false;
    std::vector<int> triangles;
};

/// A node's snap triangulation, built on its first run, and the buffers the
/// snap kernel refills each run.
struct SurfaceProjectState {
    std::shared_ptr<const SnapTriangles> snap;
    rigExec::RigExecSurfaceSnapScratch<Vec3f> scratch;
};

/// Topology is fixed per compiled rig and the authored triangles are the
/// node's payload, so the triangulation is keyed by sizes only; every
/// triangle was checked against the surface point count it is keyed by.
/// Null when the faces do not triangulate: the kernel then refuses them.
const std::vector<int> *
TrianglesOf(SurfaceProjectState &state, size_t surfacePoints,
            const Ints &counts, const Ints &indices,
            const rigExec::RigExecSurfaceSnapSettingsView &settings)
{
    const SnapTriangles *snap = state.snap.get();
    if (!snap || snap->surfacePoints != surfacePoints ||
        snap->counts != counts.Size() || snap->indices != indices.Size()) {
        auto built = std::make_shared<SnapTriangles>();
        built->surfacePoints = surfacePoints;
        built->counts = counts.Size();
        built->indices = indices.Size();
        built->valid = rigExec::RigExecBuildSurfaceSnapTriangles(
            surfacePoints, ToVector(counts), ToVector(indices),
            settings.triangles, &built->triangles);
        state.snap = std::move(built);
        snap = state.snap.get();
    }
    return snap->valid ? &snap->triangles : nullptr;
}

bool
ProjectOne(const rigExec::RigExecOpIo &io,
           const rigExec::RigExecSurfaceSnapSettingsView &settings,
           SurfaceProjectState &state, std::string *error)
{
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
        rigExec::RigExecApplySurfaceProjectKernel(
            &points, surface.Data(), surface.Size(), counts.Data(),
            counts.Size(), indices.Data(), indices.Size(),
            InOr(io, SurfaceProjectStrength(), 1.0f));
    } else if (const std::vector<int> *triangles = TrianglesOf(
                   state, surface.Size(), counts, indices, settings);
               triangles
                   ? !rigExec::RigExecApplySurfaceSnapKernel<Vec3f,
                                                             rigExecStd::Vec3d>(
                         &points, ToVector(surface), {}, {}, settings,
                         toSurface, toTarget, toMetric, triangles, nullptr,
                         &state.scratch)
                   : !rigExec::RigExecApplySurfaceSnapKernel<Vec3f,
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

bool
Check(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return EnvelopeFits(check, SurfaceProjectPoints(), SurfaceProjectEnvelope(),
                        "SurfaceProject", error);
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return ProjectOne(rigExec::RigExecBatchInstance(batch, k),
                          SettingsAt(batch, k),
                          *static_cast<SurfaceProjectState *>(
                              rigExec::RigExecBatchState(batch, k)),
                          error);
    });
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const rigExec::RigExecSurfaceSnapSettings &s =
        static_cast<const SurfaceProjectPayload *>(payload)->settings;
    SetEnum(out, kMode, s.mode);
    out.Set(kOffset, s.offset);
    SetList(out, kMaskCount, s.mask);
    SetList(out, kTriangleCount, s.triangles);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Enum("mode", rigExec::RigExecSurfaceSnapMode::OutsideSurface),
    Value("offset", rigExecStd::FloatType()),
    Count("maskCount"),
    Items("mask", rigExecStd::FloatType(), kMaskCount),
    Count("triangleCount"),
    Items("triangles", rigExecStd::IntType(), kTriangleCount),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&SurfaceProjectPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&SurfaceProjectSurface(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "surface"},
    {&SurfaceProjectCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&SurfaceProjectIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&SurfaceProjectStrength(), &rigExecStd::FloatType(), kOptional,
     "strength"},
    {&SurfaceProjectTargetToSurface(), &rigExecStd::Mat4dType(), kOptional,
     "targetToSurface"},
    {&SurfaceProjectSurfaceToTarget(), &rigExecStd::Mat4dType(), kOptional,
     "surfaceToTarget"},
    {&SurfaceProjectSurfaceToMetric(), &rigExecStd::Mat4dType(), kOptional,
     "surfaceToMetric"},
    {&SurfaceProjectEnvelope(), &rigExecStd::FloatArray(), kOptional,
     "envelope"},
    {&SurfaceProjectOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &SurfaceProjectPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SurfaceProject";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &Run;
    d.validate = &Check;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    d.state = StateOf<SurfaceProjectState>();
    return d;
}();

} // namespace

void
RegisterSurfaceProjectOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SurfaceProject(), kDesc);
}

} // namespace rigExecOps
