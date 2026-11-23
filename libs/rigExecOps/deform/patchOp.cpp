#include "rigExecOps/deform/patchOp.h"

#include "rigExecMath/patchKernel.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"

#include <cmath>
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
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Doubles = RigExecArray<double>;
using Ints = RigExecArray<int>;

bool
Run(const void *payload, void *, const rigExec::RigExecOpIo &io,
    std::string *error)
{
    const PatchPayload &p = *static_cast<const PatchPayload *>(payload);
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

} // namespace

namespace {

using rigExec::kRigExecPortArray;
using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&PatchPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&PatchControlPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "controlPoints"},
    {&PatchPointWeights(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "pointWeights"},
    {&PatchUKnots(), &rigExecStd::DoubleArray(), kRigExecPortInput, "uKnots"},
    {&PatchVKnots(), &rigExecStd::DoubleArray(), kRigExecPortInput, "vKnots"},
    {&PatchUvPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "uvPoints"},
    {&PatchUvShapes(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | kRigExecPortArray | kRigExecPortOptional, "uvShapes"},
    {&PatchUvWeights(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "uvWeights"},
    {&PatchContactLower(), &rigExecStd::IntArray(), kRigExecPortInput,
     "uvContactLower"},
    {&PatchContactUpper(), &rigExecStd::IntArray(), kRigExecPortInput,
     "uvContactUpper"},
    {&PatchEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&PatchOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const PatchPayload &p)
{
    w.Version(1);
    w.I32(p.uVertexCount);
    w.I32(p.vVertexCount);
    w.I32(p.uOrder);
    w.I32(p.vOrder);
    w.Bool(p.uPeriodic);
    w.Bool(p.vPeriodic);
}

void
Load(Reader &r, PatchPayload &p)
{
    r.Version(1);
    p.uVertexCount = r.I32();
    p.vVertexCount = r.I32();
    p.uOrder = r.I32();
    p.vOrder = r.I32();
    p.uPeriodic = r.Bool();
    p.vPeriodic = r.Bool();
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Patch";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<PatchPayload, &Save, &Load>();
    return d;
}();

} // namespace

void
RegisterPatchOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Patch(), kDesc);
}

} // namespace rigExecOps
