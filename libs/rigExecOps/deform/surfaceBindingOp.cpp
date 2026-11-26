#include "rigExecOps/deform/surfaceBindingOp.h"

#include "rigExecMath/surfaceBindingKernel.h"
#include "rigExecOps/authoring.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecSchema/schemaNames.h"

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
Check(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const bool framed = rigExec::RigExecPayloadColumnOf<uint8_t>(
                            check.batch, kFramed)[check.k] != 0;
    if (framed && (!Fed(check, SurfaceBindingFrames()) ||
                   !SameCount(CheckElements(check, SurfaceBindingFrames()), 2)))
        return Fail(error, "SurfaceBinding",
                    "needs the surface's and the target's frames");
    return EnvelopeFits(check, SurfaceBindingPoints(), SurfaceBindingEnvelope(),
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

namespace names = rigExec::SchemaAttrText;
namespace values = rigExec::SchemaValueText;
namespace types = rigExec::SchemaTypeName;
using authoring::Authored;

const rigExecStd::RigExecArray<int32_t> kNoInts;
const rigExecStd::RigExecArray<int32_t> kFirstOffset(1);
const rigExecStd::RigExecArray<float> kNoFloats;
const rigExecStd::RigExecArray<rigExecStd::Vec3f> kNoOffsets;
const float kOne = 1.0f;
const rigExecStd::Mat4d kIdentity(1.0);
const bool kFalse = false;
const char *const kSurfaceModes[] = {values::polygon, values::limit};
const char *const kNormalModes[] = {values::geometric, values::smooth};

const rigExec::RigExecPropertyInfo kVertices =
    authoring::Property(names::rigExecVertices, &kNoInts);
const rigExec::RigExecPropertyInfo kVertexOffsets =
    authoring::Property(names::rigExecVertexOffsets, &kFirstOffset);
const rigExec::RigExecPropertyInfo kPolygonOffsets =
    authoring::Property(names::rigExecPolygonOffsets, &kFirstOffset);
const rigExec::RigExecPropertyInfo kPointIndices =
    authoring::Property(names::rigExecPointIndices, &kNoInts);
const rigExec::RigExecPropertyInfo kLimitOffsets =
    authoring::Property(names::rigExecLimitOffsets, &kFirstOffset);
const rigExec::RigExecPropertyInfo kLimitIndices =
    authoring::Property(names::rigExecLimitIndices, &kNoInts);
const rigExec::RigExecPropertyInfo kBarycentricWeights =
    authoring::Property(names::rigExecBarycentricWeights, &kNoFloats);
const rigExec::RigExecPropertyInfo kBindingWeights =
    authoring::Property(names::rigExecBindingWeights, &kNoFloats);
const rigExec::RigExecPropertyInfo kMask =
    authoring::Property(names::rigExecMask, &kNoFloats);
const rigExec::RigExecPropertyInfo kDeltaMultipliers = authoring::Property(
    names::rigExecDeltaMultipliers, &kNoFloats,
    R"(Optional finite signed offset multipliers indexed by target vertex;
empty supplies one at every vertex.)");
const rigExec::RigExecPropertyInfo kLimitWeights =
    authoring::Property(names::rigExecLimitWeights, &kNoFloats);
const rigExec::RigExecPropertyInfo kLimitDuWeights =
    authoring::Property(names::rigExecLimitDuWeights, &kNoFloats);
const rigExec::RigExecPropertyInfo kLimitDvWeights =
    authoring::Property(names::rigExecLimitDvWeights, &kNoFloats);
const rigExec::RigExecPropertyInfo kOffsets =
    authoring::WithRole(authoring::Property(names::rigExecOffsets, &kNoOffsets),
                        rigExec::RigExecPropertyRole::Vector);
const rigExec::RigExecPropertyInfo kStrength =
    authoring::Property(names::rigExecStrength, &kOne);
const rigExec::RigExecPropertyInfo kDeltaMultiplier = authoring::Property(
    names::rigExecDeltaMultiplier, &kOne,
    R"(Finite signed scale applied only to transported binding offsets; zero
attaches to the surface and one preserves the offsets.)");
const rigExec::RigExecPropertyInfo kSurfaceRestMatrix =
    authoring::Property(names::rigExecSurfaceRestMatrix, &kIdentity);
const rigExec::RigExecPropertyInfo kTargetRestMatrix =
    authoring::Property(names::rigExecTargetRestMatrix, &kIdentity);
const rigExec::RigExecPropertyInfo kSurfaceToBinding =
    authoring::Property(names::rigExecSurfaceToBinding, &kIdentity);
const rigExec::RigExecPropertyInfo kFrames = authoring::Relationship(
    names::rigExecFrames,
    R"(Optional surface and target rest-to-pose providers: none, or exactly
these two in that order, composed after rigExec:surfaceRestMatrix and
rigExec:targetRestMatrix.)");
const rigExec::RigExecPropertyInfo kSurfaceMode =
    authoring::Tokens(names::rigExecSurfaceMode, &kFalse, kSurfaceModes);
const rigExec::RigExecPropertyInfo kNormalMode =
    authoring::Tokens(names::rigExecNormalMode, &kFalse, kNormalModes);

const rigExec::RigExecSchemaInfo kBindingSchema{
    types::RigExecSurfaceBindingMover,
    R"(Persistent generalized barycentric attachments. Each affected vertex
blends one or more bindings. Offsets are vectors in the current
surface's tangent, bitangent, normal frame. Polygon bindings and
optional limit position/derivative stencils are immutable authored
attributes; evaluation rebinds nothing and reads the driver points at
the rigExec:surface read phase. Of the inherited RigExecSurfaceMover
settings it reads rigExec:surface, rigExec:mask and rigExec:frames;
rigExec:mode, rigExec:snapMode, rigExec:offset, rigExec:triangles,
rigExec:pointSpace, rigExec:surfaceMatrix and rigExec:targetMatrix are
not read.)",
    types::RigExecSurfaceMover, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecFieldDesc kFields[] = {
    Authored(Flag("limit"), kSurfaceMode),
    Authored(Flag("smooth"), kNormalMode),
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
    Authored({&SurfaceBindingVertices(), &rigExecStd::IntArray(),
              kRigExecPortInput, "vertices"},
             kVertices),
    Authored({&SurfaceBindingVertexOffsets(), &rigExecStd::IntArray(),
              kRigExecPortInput, "vertexOffsets"},
             kVertexOffsets),
    Authored({&SurfaceBindingPolygonOffsets(), &rigExecStd::IntArray(),
              kRigExecPortInput, "polygonOffsets"},
             kPolygonOffsets),
    Authored({&SurfaceBindingPointIndices(), &rigExecStd::IntArray(),
              kRigExecPortInput, "pointIndices"},
             kPointIndices),
    Authored({&SurfaceBindingLimitOffsets(), &rigExecStd::IntArray(),
              kRigExecPortInput, "limitOffsets"},
             kLimitOffsets),
    Authored({&SurfaceBindingLimitIndices(), &rigExecStd::IntArray(),
              kRigExecPortInput, "limitIndices"},
             kLimitIndices),
    Authored({&SurfaceBindingBarycentricWeights(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "barycentricWeights"},
             kBarycentricWeights),
    Authored({&SurfaceBindingBindingWeights(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "bindingWeights"},
             kBindingWeights),
    Authored({&SurfaceBindingMask(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "mask"},
             kMask),
    Authored({&SurfaceBindingDeltaMultipliers(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "deltaMultipliers"},
             kDeltaMultipliers),
    Authored({&SurfaceBindingLimitWeights(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "limitWeights"},
             kLimitWeights),
    Authored({&SurfaceBindingLimitDuWeights(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "limitDuWeights"},
             kLimitDuWeights),
    Authored({&SurfaceBindingLimitDvWeights(), &rigExecStd::FloatArray(),
              kRigExecPortInput, "limitDvWeights"},
             kLimitDvWeights),
    Authored({&SurfaceBindingOffsets(), &rigExecStd::Vec3fArray(),
              kRigExecPortInput, "offsets"},
             kOffsets),
    Authored({&SurfaceBindingStrength(), &rigExecStd::FloatType(),
              kRigExecPortInput, "strength"},
             kStrength),
    Authored({&SurfaceBindingDeltaMultiplier(), &rigExecStd::FloatType(),
              kRigExecPortInput, "deltaMultiplier"},
             kDeltaMultiplier),
    Authored({&SurfaceBindingSurfaceRest(), &rigExecStd::Mat4dType(),
              kRigExecPortInput, "surfaceRestMatrix"},
             kSurfaceRestMatrix),
    Authored({&SurfaceBindingTargetRest(), &rigExecStd::Mat4dType(),
              kRigExecPortInput, "targetRestMatrix"},
             kTargetRestMatrix),
    Authored({&SurfaceBindingSurfaceToBinding(), &rigExecStd::Mat4dType(),
              kRigExecPortInput, "surfaceToBinding"},
             kSurfaceToBinding),
    Authored({&SurfaceBindingFrames(), &rigExecStd::Mat4dArray(),
              kRigExecPortInput | kRigExecPortOptional, "frames"},
             kFrames),
    {&SurfaceBindingSurface(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "surfacePoints"},
    {&SurfaceBindingCounts(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexCounts"},
    {&SurfaceBindingIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "faceVertexIndices"},
    {&SurfaceBindingEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&SurfaceBindingOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &SurfaceBindingPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SurfaceBinding";
    d.schema = &kBindingSchema;
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
    return d;
}();

} // namespace

void
RegisterSurfaceBindingOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SurfaceBinding(), kDesc);
}

} // namespace rigExecOps
