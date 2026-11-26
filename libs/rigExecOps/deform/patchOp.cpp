#include "rigExecOps/deform/patchOp.h"

#include "rigExecMath/patchKernel.h"
#include "rigExecOps/authoring.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecSchema/schemaNames.h"

#include <cmath>
#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Patch)

RIGEXEC_DEFINE_PORT_HANDLE(PatchPoints)
RIGEXEC_DEFINE_PORT_HANDLE(PatchControlPoints)
RIGEXEC_DEFINE_PORT_HANDLE(PatchPointWeights)
RIGEXEC_DEFINE_PORT_HANDLE(PatchUKnots)
RIGEXEC_DEFINE_PORT_HANDLE(PatchVKnots)
RIGEXEC_DEFINE_PORT_HANDLE(PatchUvPoints)
RIGEXEC_DEFINE_PORT_HANDLE(PatchUvShapes)
RIGEXEC_DEFINE_PORT_HANDLE(PatchUvWeights)
RIGEXEC_DEFINE_PORT_HANDLE(PatchContactLower)
RIGEXEC_DEFINE_PORT_HANDLE(PatchContactUpper)
RIGEXEC_DEFINE_PORT_HANDLE(PatchEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(PatchOut)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Doubles = RigExecArray<double>;
using Ints = RigExecArray<int>;

enum PatchField : uint32_t {
    kUVertexCount,
    kVVertexCount,
    kUOrder,
    kVOrder,
    kUPeriodic,
    kVPeriodic,
};

/// Instance \p k's patch structure.
PatchPayload
PayloadAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    PatchPayload p;
    p.uVertexCount = ValueAt<int32_t>(batch, kUVertexCount, k);
    p.vVertexCount = ValueAt<int32_t>(batch, kVVertexCount, k);
    p.uOrder = ValueAt<int32_t>(batch, kUOrder, k);
    p.vOrder = ValueAt<int32_t>(batch, kVOrder, k);
    p.uPeriodic = FlagAt(batch, kUPeriodic, k);
    p.vPeriodic = FlagAt(batch, kVPeriodic, k);
    return p;
}

bool
PatchOne(const rigExec::RigExecOpIo &io, const PatchPayload &p,
         std::string *error)
{
    const Points &controls = *In<Points>(io, PatchControlPoints());
    const Doubles &pointWeights = *In<Doubles>(io, PatchPointWeights());
    const Doubles &uKnots = *In<Doubles>(io, PatchUKnots());
    const Doubles &vKnots = *In<Doubles>(io, PatchVKnots());
    rigExec::RigExecNurbsPatchKernel<Vec3f> patch;
    patch.points = controls.Data();
    patch.pointCount = controls.Size();
    patch.pointWeights = pointWeights.Data();
    patch.pointWeightCount = pointWeights.Size();
    patch.uVertexCount = p.uVertexCount;
    patch.vVertexCount = p.vVertexCount;
    patch.uOrder = p.uOrder;
    patch.vOrder = p.vOrder;
    patch.uKnots = uKnots.Data();
    patch.uKnotCount = uKnots.Size();
    patch.vKnots = vKnots.Data();
    patch.vKnotCount = vKnots.Size();
    patch.uPeriodic = p.uPeriodic;
    patch.vPeriodic = p.vPeriodic;
    if (!patch.IsValid())
        return Fail(error, "Patch", "unusable NURBS patch");

    std::vector<Vec3f> uv = ToVector(*In<Points>(io, PatchUvPoints()));
    const InList<Points> shapes(io, PatchUvShapes());
    const Doubles &weights = *In<Doubles>(io, PatchUvWeights());
    if (weights.Size() != shapes.Size())
        return Fail(error, "Patch", "one UV weight per UV shape");
    std::vector<const Vec3f *> shapePoints(shapes.Size());
    for (size_t k = 0; k < shapes.Size(); ++k) {
        if (shapes[k].Size() != uv.size())
            return Fail(error, "Patch",
                        "a UV shape's point count differs from the UV points");
        shapePoints[k] = shapes[k].Data();
    }
    if (!rigExec::RigExecMixPatchShapes(shapePoints.data(), weights.Data(),
                                        shapes.Size(), uv.data(), uv.size()))
        return Fail(error, "Patch", "UV weights must be finite");
    const Ints &lower = *In<Ints>(io, PatchContactLower());
    const Ints &upper = *In<Ints>(io, PatchContactUpper());
    if (!rigExec::RigExecTouchPatchContacts(lower.Data(), lower.Size(),
                                            upper.Data(), upper.Size(),
                                            uv.data(), uv.size()))
        return Fail(error, "Patch",
                    rigExec::RigExecPatchContactError(
                        lower.Data(), lower.Size(), upper.Data(), upper.Size(),
                        uv.size()));
    for (const Vec3f &x : uv)
        if (!std::isfinite(x[0]) || !std::isfinite(x[1]))
            return Fail(error, "Patch", "UV points must be finite");

    const Points &in = *In<Points>(io, PatchPoints());
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecApplyPatchKernel(points.data(), points.size(), patch,
                                          uv.data(), uv.size()))
        return Fail(error, "Patch",
                    "UV point count differs from the points, or a point on "
                    "the patch is not finite");
    return WritePoints(io, PatchOut(), in, points,
                       In<Envelope>(io, PatchEnvelope()), "Patch", error);
}

bool
Check(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const PatchPayload p = PayloadAt(check.batch, check.k);
    const auto *controls = CheckValue<Points>(check, PatchControlPoints());
    const auto *pointWeights = CheckValue<Doubles>(check, PatchPointWeights());
    const auto *uKnots = CheckValue<Doubles>(check, PatchUKnots());
    const auto *vKnots = CheckValue<Doubles>(check, PatchVKnots());
    if (controls && pointWeights && uKnots && vKnots) {
        rigExec::RigExecNurbsPatchKernel<Vec3f> patch;
        patch.points = controls->Data();
        patch.pointCount = controls->Size();
        patch.pointWeights = pointWeights->Data();
        patch.pointWeightCount = pointWeights->Size();
        patch.uVertexCount = p.uVertexCount;
        patch.vVertexCount = p.vVertexCount;
        patch.uOrder = p.uOrder;
        patch.vOrder = p.vOrder;
        patch.uKnots = uKnots->Data();
        patch.uKnotCount = uKnots->Size();
        patch.vKnots = vKnots->Data();
        patch.vKnotCount = vKnots->Size();
        patch.uPeriodic = p.uPeriodic;
        patch.vPeriodic = p.vPeriodic;
        if (!patch.IsValid())
            return Fail(error, "Patch", "unusable NURBS patch");
    }
    const size_t uv = CheckElements(check, PatchUvPoints());
    if (!SameCount(CheckElements(check, PatchUvWeights()),
                   Fed(check, PatchUvShapes())
                       ? CheckElements(check, PatchUvShapes())
                       : 0))
        return Fail(error, "Patch", "one UV weight per UV shape");
    if (uv != 1 && !SameCount(uv, CheckElements(check, PatchPoints())))
        return Fail(error, "Patch",
                    "UV point count differs from the points, or a point on "
                    "the patch is not finite");
    const auto *lower = CheckValue<Ints>(check, PatchContactLower());
    const auto *upper = CheckValue<Ints>(check, PatchContactUpper());
    if (lower && upper && uv < rigExec::kRigExecPortUnfed &&
        (lower->Size() || upper->Size())) {
        const std::string why = rigExec::RigExecPatchContactError(
            lower->Data(), lower->Size(), upper->Data(), upper->Size(), uv);
        if (!why.empty())
            return Fail(error, "Patch", why);
    }
    return EnvelopeFits(check, PatchPoints(), PatchEnvelope(), "Patch", error);
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return PatchOne(rigExec::RigExecBatchInstance(batch, k),
                        PayloadAt(batch, k), error);
    });
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const PatchPayload &p = *static_cast<const PatchPayload *>(payload);
    out.Set(kUVertexCount, p.uVertexCount);
    out.Set(kVVertexCount, p.vVertexCount);
    out.Set(kUOrder, p.uOrder);
    out.Set(kVOrder, p.vOrder);
    SetFlag(out, kUPeriodic, p.uPeriodic);
    SetFlag(out, kVPeriodic, p.vPeriodic);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Value("uVertexCount", rigExecStd::IntType()),
    Value("vVertexCount", rigExecStd::IntType()),
    Value("uOrder", rigExecStd::IntType()),
    Value("vOrder", rigExecStd::IntType()),
    Flag("uPeriodic"),
    Flag("vPeriodic"),
};

} // namespace

namespace {

using rigExec::kRigExecPortArray;
using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kChecked = kRigExecPortInput | rigExec::kRigExecPortChecked;

namespace names = rigExec::SchemaAttrText;
namespace values = rigExec::SchemaValueText;
namespace types = rigExec::SchemaTypeName;
using authoring::Authored;

const rigExecStd::RigExecArray<double> kNoWeights;
const rigExecStd::RigExecArray<int32_t> kNoPairs;

const rigExec::RigExecPropertyInfo kPatchProperty = authoring::Relationship(
    names::rigExecPatch,
    R"(Native UsdGeomNurbsPatch prim supplying the NURBS surface.)");
const rigExec::RigExecPropertyInfo kUvPoints = authoring::Relationship(
    names::rigExecUvPoints,
    R"(Native UsdGeomPointBased prim or points property whose XY coordinates
supply normalized (u, v) parameters on the patch: the neutral the UV
shapes are measured from. It holds one point per target point, or a
single point every target point shares. A parameter is clamped to [0, 1]
on an open patch axis and wrapped on a closed or periodic one.)");
const rigExec::RigExecPropertyInfo kUvShapes = authoring::Relationship(
    names::rigExecUvShapes,
    R"(Ordered UV shapes: native UsdGeomPointBased prims or points properties,
each with as many points as rigExec:uvPoints. Shapes add to the neutral,
uv = neutral + sum_k w_k (shape_k - neutral), with w_k the k-th entry of
inputs:uvWeights.)",
    rigExec::RigExecPropertyBinding::ConnectionList);
const rigExec::RigExecPropertyInfo kUvWeights = authoring::WithType(
    authoring::Property(
        names::inputsUvWeights, &kNoWeights,
        R"(One weight per rigExec:uvShapes target, by index. Connect it to one
float or double source per shape, in shape order -- a control's avars,
typically -- and connection k is the weight of shape k. With no
connection the authored array is the weights; an empty array leaves
every shape at zero.)"),
    "float[]");
const rigExec::RigExecPropertyInfo kContactLower = authoring::Uniform(
    names::rigExecUvContactLower, &kNoPairs,
    R"(UV point indices that never stand above their partners in
rigExec:uvContactUpper, by index, after the UV shapes mix. A pair that
would cross meets at the midpoint of its two v values; u is left alone.
Pair one column of a lower curve with the same column of an upper curve,
sharing u in the neutral and in every shape, and the two curves touch
but never cross. Each point belongs to at most one pair.)");
const rigExec::RigExecPropertyInfo kContactUpper = authoring::Uniform(
    names::rigExecUvContactUpper, &kNoPairs,
    R"(The partner of each rigExec:uvContactLower index: the UV point that
point never rises above. As long as rigExec:uvContactLower.)");

const rigExec::RigExecSchemaInfo kPatchSchema{
    types::RigExecPatchMover,
    R"(Evaluates a UsdGeomNurbsPatch at normalized UV parameters read from a
point-based UV input prim and writes an exact native UsdGeomPointBased
points property. rigExec:patch, rigExec:uvPoints and rigExec:uvShapes
honor rigExecReadPhase metadata, so controllers can deform the patch or
drive the UV input points through earlier movers. Weighted UV shapes mix
into the UV points before the patch is evaluated, so avars such as
smile, frown or mouthOpen can slide the attachment across the surface
together. Contact pairs then keep paired points from crossing in v, so
two curves on one patch -- upper and lower lips, upper and lower lids --
press together rather than pass through each other under any mix of
shapes.)",
    authoring::kTyped, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecPortDesc kPorts[] = {
    {&PatchPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    Authored({&PatchControlPoints(), &rigExecStd::Vec3fArray(),
              kRigExecPortInput, "controlPoints"},
             kPatchProperty),
    {&PatchPointWeights(), &rigExecStd::DoubleArray(), kChecked,
     "pointWeights"},
    {&PatchUKnots(), &rigExecStd::DoubleArray(), kChecked, "uKnots"},
    {&PatchVKnots(), &rigExecStd::DoubleArray(), kChecked, "vKnots"},
    Authored({&PatchUvPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
              "uvPoints"},
             kUvPoints),
    Authored({&PatchUvShapes(), &rigExecStd::Vec3fArray(),
              kRigExecPortInput | kRigExecPortArray | kRigExecPortOptional,
              "uvShapes"},
             kUvShapes),
    Authored({&PatchUvWeights(), &rigExecStd::DoubleArray(), kRigExecPortInput,
              "uvWeights"},
             kUvWeights),
    Authored({&PatchContactLower(), &rigExecStd::IntArray(), kChecked,
              "uvContactLower"},
             kContactLower),
    Authored({&PatchContactUpper(), &rigExecStd::IntArray(), kChecked,
              "uvContactUpper"},
             kContactUpper),
    {&PatchEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&PatchOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &PatchPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Patch";
    d.schema = &kPatchSchema;
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
RegisterPatchOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Patch(), kDesc);
}

} // namespace rigExecOps
