#include "rigExecOps/deform/surfaceBindingOp.h"

#include "rigExecMath/surfaceBindingKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"

#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SurfaceBinding)

RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingPoints)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingVertices)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingVertexOffsets)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingPolygonOffsets)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingPointIndices)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingLimitOffsets)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingLimitIndices)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingBarycentricWeights)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingBindingWeights)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingMask)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingDeltaMultipliers)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingLimitWeights)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingLimitDuWeights)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingLimitDvWeights)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingOffsets)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingStrength)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingDeltaMultiplier)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingSurfaceRest)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingTargetRest)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingSurfaceToBinding)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingFrames)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingSurface)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingCounts)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingIndices)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(SurfaceBindingOut)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

template <class T>
rigExec::RigExecSurfaceBindingList<T>
List(const rigExec::RigExecOpIo &io, const rigExec::RigExecPortHandle &port)
{
    const RigExecArray<T> &a = *In<RigExecArray<T>>(io, port);
    return {a.Data(), a.Size()};
}

enum SurfaceBindingField : uint32_t { kLimit, kSmooth, kFramed };

bool
BindOne(const rigExec::RigExecOpIo &io, const SurfaceBindingPayload &p,
        std::string *error)
{
    rigExec::RigExecSurfaceBinding<Vec3f> b;
    b.vertices = List<int>(io, SurfaceBindingVertices());
    b.vertexOffsets = List<int>(io, SurfaceBindingVertexOffsets());
    b.polygonOffsets = List<int>(io, SurfaceBindingPolygonOffsets());
    b.pointIndices = List<int>(io, SurfaceBindingPointIndices());
    b.limitOffsets = List<int>(io, SurfaceBindingLimitOffsets());
    b.limitIndices = List<int>(io, SurfaceBindingLimitIndices());
    b.barycentricWeights = List<float>(io, SurfaceBindingBarycentricWeights());
    b.bindingWeights = List<float>(io, SurfaceBindingBindingWeights());
    b.mask = List<float>(io, SurfaceBindingMask());
    b.deltaMultipliers = List<float>(io, SurfaceBindingDeltaMultipliers());
    b.limitWeights = List<float>(io, SurfaceBindingLimitWeights());
    b.limitDuWeights = List<float>(io, SurfaceBindingLimitDuWeights());
    b.limitDvWeights = List<float>(io, SurfaceBindingLimitDvWeights());
    b.offsets = List<Vec3f>(io, SurfaceBindingOffsets());
    b.limit = p.limit;
    b.smooth = p.smooth;
    b.strength = *In<float>(io, SurfaceBindingStrength());
    b.deltaMultiplier = *In<float>(io, SurfaceBindingDeltaMultiplier());
    const Mat4d *frames = nullptr;
    if (p.framed) {
        const auto *f = In<RigExecArray<Mat4d>>(io, SurfaceBindingFrames());
        if (!f || f->Size() != 2)
            return Fail(error, "SurfaceBinding",
                        "needs the surface's and the target's frames");
        frames = f->Data();
    }
    const auto counts = List<int>(io, SurfaceBindingCounts());
    const auto indices = List<int>(io, SurfaceBindingIndices());
    const Points &in = *In<Points>(io, SurfaceBindingPoints());
    std::vector<Vec3f> surface =
        ToVector(*In<Points>(io, SurfaceBindingSurface()));
    Mat4d toTarget = *In<Mat4d>(io, SurfaceBindingTargetRest());
    if (!rigExec::RigExecPrepareSurfaceBinding<Vec3f, Vec3d, Mat4d>(
            b, in.Size(), *In<Mat4d>(io, SurfaceBindingSurfaceRest()),
            *In<Mat4d>(io, SurfaceBindingTargetRest()),
            *In<Mat4d>(io, SurfaceBindingSurfaceToBinding()), frames, counts,
            indices, surface.data(), surface.size(), &toTarget))
        return Fail(error, "SurfaceBinding",
                    "unusable binding arrays, stencil, mask or coordinate "
                    "frame");
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecApplySurfaceBinding<Vec3f, Vec3d, Mat4d>(
            b, surface.data(), surface.size(), counts, indices, toTarget,
            points.data(), points.size()) ||
        !AllFinite(points))
        return Fail(error, "SurfaceBinding", "a bound point is not finite");
    return WritePoints(io, SurfaceBindingOut(), in, points,
                       In<Envelope>(io, SurfaceBindingEnvelope()),
                       "SurfaceBinding", error);
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    const uint8_t *limit =
        rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kLimit);
    const uint8_t *smooth =
        rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kSmooth);
    const uint8_t *framed =
        rigExec::RigExecPayloadColumnOf<uint8_t>(batch, kFramed);
    return EachInstance(batch, failed, [&](size_t k) {
        SurfaceBindingPayload p;
        p.limit = limit[k] != 0;
        p.smooth = smooth[k] != 0;
        p.framed = framed[k] != 0;
        return BindOne(rigExec::RigExecBatchInstance(batch, k), p, error);
    });
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const auto &p = *static_cast<const SurfaceBindingPayload *>(payload);
    SetFlag(out, kLimit, p.limit);
    SetFlag(out, kSmooth, p.smooth);
    SetFlag(out, kFramed, p.framed);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Flag("limit"),
    Flag("smooth"),
    Flag("framed"),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&SurfaceBindingPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&SurfaceBindingVertices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "vertices"},
    {&SurfaceBindingVertexOffsets(), &rigExecStd::IntArray(), kRigExecPortInput,
     "vertexOffsets"},
    {&SurfaceBindingPolygonOffsets(), &rigExecStd::IntArray(),
     kRigExecPortInput, "polygonOffsets"},
    {&SurfaceBindingPointIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "pointIndices"},
    {&SurfaceBindingLimitOffsets(), &rigExecStd::IntArray(), kRigExecPortInput,
     "limitOffsets"},
    {&SurfaceBindingLimitIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "limitIndices"},
    {&SurfaceBindingBarycentricWeights(), &rigExecStd::FloatArray(),
     kRigExecPortInput, "barycentricWeights"},
    {&SurfaceBindingBindingWeights(), &rigExecStd::FloatArray(),
     kRigExecPortInput, "bindingWeights"},
    {&SurfaceBindingMask(), &rigExecStd::FloatArray(), kRigExecPortInput,
     "mask"},
    {&SurfaceBindingDeltaMultipliers(), &rigExecStd::FloatArray(),
     kRigExecPortInput, "deltaMultipliers"},
    {&SurfaceBindingLimitWeights(), &rigExecStd::FloatArray(),
     kRigExecPortInput, "limitWeights"},
    {&SurfaceBindingLimitDuWeights(), &rigExecStd::FloatArray(),
     kRigExecPortInput, "limitDuWeights"},
    {&SurfaceBindingLimitDvWeights(), &rigExecStd::FloatArray(),
     kRigExecPortInput, "limitDvWeights"},
    {&SurfaceBindingOffsets(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "offsets"},
    {&SurfaceBindingStrength(), &rigExecStd::FloatType(), kRigExecPortInput,
     "strength"},
    {&SurfaceBindingDeltaMultiplier(), &rigExecStd::FloatType(),
     kRigExecPortInput, "deltaMultiplier"},
    {&SurfaceBindingSurfaceRest(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "surfaceRestMatrix"},
    {&SurfaceBindingTargetRest(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "targetRestMatrix"},
    {&SurfaceBindingSurfaceToBinding(), &rigExecStd::Mat4dType(),
     kRigExecPortInput, "surfaceToBinding"},
    {&SurfaceBindingFrames(), &rigExecStd::Mat4dArray(),
     kRigExecPortInput | kRigExecPortOptional, "frames"},
    {&SurfaceBindingSurface(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "surfacePoints"},
    {&SurfaceBindingCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&SurfaceBindingIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&SurfaceBindingEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&SurfaceBindingOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SurfaceBinding";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    return d;
}();

} // namespace

void
RegisterSurfaceBindingOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SurfaceBinding(), kDesc);
}

} // namespace rigExecOps
