#include "rigExecOps/deform/layeredSkinOp.h"

#include "rigExecMath/layeredSkinKernel.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(LayeredSkin)

RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinPoints)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinTransforms)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinIndices)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinWeights)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinMask)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinBase)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinInputTransform)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(LayeredSkinOut)

namespace {

using namespace opIo;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

bool
Run(const void *payload, void *, const rigExec::RigExecOpIo &io,
    std::string *error)
{
    const LayeredSkinPayload &p =
        *static_cast<const LayeredSkinPayload *>(payload);
    const Points &in = *In<Points>(io, LayeredSkinPoints());
    const auto &transforms =
        *In<RigExecArray<Mat4d>>(io, LayeredSkinTransforms());
    const auto &indices = *In<RigExecArray<int>>(io, LayeredSkinIndices());
    const Envelope &weights = *In<Envelope>(io, LayeredSkinWeights());
    const Envelope &mask = *In<Envelope>(io, LayeredSkinMask());
    const Points &base = *In<Points>(io, LayeredSkinBase());
    if (!rigExec::RigExecLayeredSkinAccepts(
            transforms.Data(), transforms.Size(), indices.Data(),
            indices.Size(), weights.Data(), weights.Size(), p.elementSize,
            mask.Data(), mask.Size(), in.Size()))
        return Fail(error, "LayeredSkin", "unusable skin layout or mask");
    const rigExec::RigExecSkinLayoutOf<Mat4d> layout{
        transforms.Data(), transforms.Size(), indices.Data(),
        weights.Data(),    indices.Size(),    size_t(p.elementSize),
        in.Size()};
    std::vector<Vec3f> points = ToVector(in);
    if (!rigExec::RigExecApplyLayeredSkinKernel<Vec3f, Vec3d, Mat4d>(
            layout, p.dualQuaternion, base.Data(), base.Size(), mask.Data(),
            mask.Size(), *In<Mat4d>(io, LayeredSkinInputTransform()),
            points.data(), points.size()) ||
        !AllFinite(points))
        return Fail(error, "LayeredSkin",
                    "the base does not fit, the blend is degenerate or a "
                    "point is not finite");
    return WritePoints(io, LayeredSkinOut(), in, points,
                       In<Envelope>(io, LayeredSkinEnvelope()), "LayeredSkin",
                       error);
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&LayeredSkinPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&LayeredSkinTransforms(), &rigExecStd::Mat4dArray(), kRigExecPortInput,
     "transforms"},
    {&LayeredSkinIndices(), &rigExecStd::IntArray(), kRigExecPortInput,
     "jointIndices"},
    {&LayeredSkinWeights(), &rigExecStd::FloatArray(), kRigExecPortInput,
     "jointWeights"},
    {&LayeredSkinMask(), &rigExecStd::FloatArray(), kRigExecPortInput, "mask"},
    {&LayeredSkinBase(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "basePoints"},
    {&LayeredSkinInputTransform(), &rigExecStd::Mat4dType(), kRigExecPortInput,
     "inputTransform"},
    {&LayeredSkinEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&LayeredSkinOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const LayeredSkinPayload &p)
{
    w.Version(1);
    w.I32(p.elementSize);
    w.Bool(p.dualQuaternion);
}

void
Load(Reader &r, LayeredSkinPayload &p)
{
    r.Version(1);
    p.elementSize = r.I32();
    p.dualQuaternion = r.Bool();
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "LayeredSkin";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpFromLeaves;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<LayeredSkinPayload, &Save, &Load>();
    return d;
}();

} // namespace

void
RegisterLayeredSkinOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(LayeredSkin(), kDesc);
}

} // namespace rigExecOps
