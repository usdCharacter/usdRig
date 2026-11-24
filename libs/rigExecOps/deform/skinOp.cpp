#include "rigExecOps/deform/skinOp.h"

#include "rigExecMath/skinKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <cmath>
#include <cstdint>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Skin)

RIGEXEC_DEFINE_PORT_HANDLE(SkinPoints)
RIGEXEC_DEFINE_PORT_HANDLE(SkinTransforms)
RIGEXEC_DEFINE_PORT_HANDLE(SkinIndices)
RIGEXEC_DEFINE_PORT_HANDLE(SkinWeights)
RIGEXEC_DEFINE_PORT_HANDLE(SkinEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(SkinOut)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Transforms = rigExecStd::RigExecArray<Mat4d>;
using Indices = rigExecStd::RigExecArray<int32_t>;
using Layout = rigExec::RigExecSkinLayoutOf<Mat4d>;

bool
Usable(const Transforms &transforms)
{
    for (const Mat4d &m : transforms) {
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                if (!std::isfinite(m[r][c]))
                    return false;
        if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1)
            return false;
    }
    return true;
}

enum SkinField : uint32_t { kMethod, kElementSize };

bool
SkinOne(const rigExec::RigExecOpIo &io, SkinMethod method, int elementSize,
        std::string *error)
{
    const Points &in = *In<Points>(io, SkinPoints());
    const Transforms &transforms = *In<Transforms>(io, SkinTransforms());
    const Indices &indices = *In<Indices>(io, SkinIndices());
    const Envelope &weights = *In<Envelope>(io, SkinWeights());

    Layout layout;
    layout.transforms = transforms.Data();
    layout.transformCount = transforms.Size();
    layout.indices = indices.Data();
    layout.weights = weights.Data();
    layout.indexCount = indices.Size();
    layout.elementSize = elementSize < 1 ? 0 : size_t(elementSize);
    layout.pointCount = in.Size();
    if (!layout.elementSize || !layout.transformCount)
        return Fail(error, "Skin", "needs an influence and elementSize >= 1");
    if (weights.Size() != indices.Size() ||
        indices.Size() != in.Size() * layout.elementSize)
        return Fail(error, "Skin",
                    "indices and weights need elementSize per point");
    for (size_t i = 0; i < indices.Size(); ++i) {
        if (indices[i] < 0 || size_t(indices[i]) >= layout.transformCount)
            return Fail(error, "Skin", "an index is outside the influences");
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f)
            return Fail(error, "Skin",
                        "weights must be finite and non-negative");
    }
    if (!Usable(transforms))
        return Fail(error, "Skin", "transforms must be finite and affine");

    Points &out = Out<Points>(io, SkinOut());
    out.Assign(in.Size());
    if (method == SkinMethod::ClassicLinear) {
        if (io.useSimd)
            rigExec::RigExecApplyLinearBlendSkinSimdKernel<Vec3f, Vec3d>(
                in.Data(), out.Data(), layout, nullptr);
        else
            rigExec::RigExecApplyLinearBlendSkinKernel<Vec3f, Vec3d>(
                in.Data(), out.Data(), layout);
    } else {
        const auto palette =
            rigExec::RigExecSkinDualQuatPaletteKernel<Vec3f, Vec3d>(
                layout.transforms, layout.transformCount);
        if (!rigExec::RigExecApplyDualQuatSkinKernel<Vec3f, Vec3d>(
                in.Data(), out.Data(), layout, palette.data(), palette.size()))
            return Fail(error, "Skin", "dual quaternion blend is degenerate");
    }
    return BlendEnvelope(in, In<Envelope>(io, SkinEnvelope()), &out, "Skin",
                         error);
}

bool
Run(const rigExec::RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    const uint32_t *methods =
        rigExec::RigExecPayloadColumnOf<uint32_t>(batch, kMethod);
    const int32_t *elementSizes =
        rigExec::RigExecPayloadColumnOf<int32_t>(batch, kElementSize);
    return EachInstance(batch, failed, [&](size_t k) {
        return SkinOne(rigExec::RigExecBatchInstance(batch, k),
                       SkinMethod(methods[k]), elementSizes[k], error);
    });
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const SkinPayload &p = *static_cast<const SkinPayload *>(payload);
    SetEnum(out, kMethod, p.method);
    out.Set(kElementSize, int32_t(p.elementSize));
}

const rigExec::RigExecFieldDesc kFields[] = {
    Enum("method", SkinMethod::DualQuaternion),
    Value("elementSize", rigExecStd::IntType()),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&SkinPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&SkinTransforms(), &rigExecStd::Mat4dArray(), kRigExecPortInput,
     "transforms"},
    {&SkinIndices(), &rigExecStd::IntArray(), kRigExecPortInput, "indices"},
    {&SkinWeights(), &rigExecStd::FloatArray(), kRigExecPortInput, "weights"},
    {&SkinEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&SkinOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &SkinPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Skin";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &Run;
    d.capabilities =
        rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable |
        rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged |
        rigExec::kRigExecOpGate | rigExec::kRigExecOpFromLeaves;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &Values;
    return d;
}();

} // namespace

void
RegisterSkinOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Skin(), kDesc);
}

} // namespace rigExecOps
