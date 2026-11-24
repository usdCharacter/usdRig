#include "rigExecOps/deform/skinOp.h"

#include "rigExecMath/skinKernel.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <cmath>
#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

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

using Palette =
    decltype(rigExec::RigExecSkinDualQuatPaletteKernel<Vec3f, Vec3d>(
        static_cast<const Mat4d *>(nullptr), size_t(0)));

/// What every range of a node reads this frame: the narrowed rows of a
/// linear blend, or a dual-quaternion blend's palette.
struct SkinState {
    std::vector<float> rows;
    Palette palette;
};

Layout
LayoutOf(const rigExec::RigExecOpBatch &batch, size_t k)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const Transforms &transforms = *In<Transforms>(io, SkinTransforms());
    const Indices &indices = *In<Indices>(io, SkinIndices());
    const int elementSize =
        rigExec::RigExecPayloadColumnOf<int32_t>(batch, kElementSize)[k];
    Layout layout;
    layout.transforms = transforms.Data();
    layout.transformCount = transforms.Size();
    layout.indices = indices.Data();
    layout.weights = In<Envelope>(io, SkinWeights())->Data();
    layout.indexCount = indices.Size();
    layout.elementSize = elementSize < 1 ? 0 : size_t(elementSize);
    layout.pointCount = In<Points>(io, SkinPoints())->Size();
    return layout;
}

rigExec::RigExecRangePlan
Accept(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
       std::string *error)
{
    constexpr auto kRefuse = rigExec::RigExecRangePlan::Refuse;
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const Points &in = *In<Points>(io, SkinPoints());
    const Envelope &weights = *In<Envelope>(io, SkinWeights());
    const Indices &indices = *In<Indices>(io, SkinIndices());
    const Layout layout = LayoutOf(batch, k);
    if (!layout.elementSize || !layout.transformCount)
        return Fail(error, "Skin", "needs an influence and elementSize >= 1"),
               kRefuse;
    if (weights.Size() != indices.Size() ||
        indices.Size() != in.Size() * layout.elementSize)
        return Fail(error, "Skin",
                    "indices and weights need elementSize per point"),
               kRefuse;
    for (size_t i = 0; i < indices.Size(); ++i) {
        if (indices[i] < 0 || size_t(indices[i]) >= layout.transformCount)
            return Fail(error, "Skin", "an index is outside the influences"),
                   kRefuse;
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f)
            return Fail(error, "Skin",
                        "weights must be finite and non-negative"),
                   kRefuse;
    }
    if (!Usable(*In<Transforms>(io, SkinTransforms())))
        return Fail(error, "Skin", "transforms must be finite and affine"),
               kRefuse;
    if (!EnvelopeFits(in, In<Envelope>(io, SkinEnvelope()), "Skin", error))
        return kRefuse;
    return rigExec::RigExecRangePlan::Ranges;
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t, std::string *)
{
    SkinState &state =
        *static_cast<SkinState *>(rigExec::RigExecBatchState(batch, k));
    const Layout layout = LayoutOf(batch, k);
    const auto method = SkinMethod(
        rigExec::RigExecPayloadColumnOf<uint32_t>(batch, kMethod)[k]);
    if (method != SkinMethod::ClassicLinear) {
        state.palette = rigExec::RigExecSkinDualQuatPaletteKernel<Vec3f, Vec3d>(
            layout.transforms, layout.transformCount);
        return true;
    }
    if (!batch.useSimd)
        return true;
    state.rows.resize(layout.transformCount * rigExec::kRigExecSkinRowStride);
    for (size_t t = 0; t < layout.transformCount; ++t)
        rigExec::RigExecNarrowSkinRowsKernel(
            layout.transforms[t],
            &state.rows[t * rigExec::kRigExecSkinRowStride]);
    return true;
}

/// Points [begin, end) through the kernels over the layout's part for
/// them, then the envelope.
bool
SkinRange(const rigExec::RigExecOpBatch &batch, size_t k,
          const rigExec::RigExecOpRange &range, std::string *error)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const SkinState &state =
        *static_cast<const SkinState *>(rigExec::RigExecBatchState(batch, k));
    const Points &in = *In<Points>(io, SkinPoints());
    Points &out = Out<Points>(io, SkinOut());
    const size_t begin = range.begin, end = range.end;
    Layout part = LayoutOf(batch, k);
    part.indices += begin * part.elementSize;
    part.weights += begin * part.elementSize;
    part.indexCount = (end - begin) * part.elementSize;
    part.pointCount = end - begin;
    const auto method = SkinMethod(
        rigExec::RigExecPayloadColumnOf<uint32_t>(batch, kMethod)[k]);
    if (method == SkinMethod::ClassicLinear) {
        if (io.useSimd)
            rigExec::RigExecApplyLinearBlendSkinSimdKernel<Vec3f, Vec3d>(
                in.Data() + begin, out.Data() + begin, part, state.rows.data());
        else
            rigExec::RigExecApplyLinearBlendSkinKernel<Vec3f, Vec3d>(
                in.Data() + begin, out.Data() + begin, part);
    } else if (!rigExec::RigExecApplyDualQuatSkinKernel<Vec3f, Vec3d>(
                   in.Data() + begin, out.Data() + begin, part,
                   state.palette.data(), state.palette.size())) {
        return Fail(error, "Skin", "dual quaternion blend is degenerate");
    }
    BlendEnvelopeRange(in, In<Envelope>(io, SkinEnvelope()), out, begin, end);
    return true;
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
using rigExec::kRigExecPortPointwise;

const rigExec::RigExecPortDesc kPorts[] = {
    {&SkinPoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | kRigExecPortPointwise, "points"},
    {&SkinTransforms(), &rigExecStd::Mat4dArray(), kRigExecPortInput,
     "transforms"},
    {&SkinIndices(), &rigExecStd::IntArray(), kRigExecPortInput, "indices"},
    {&SkinWeights(), &rigExecStd::FloatArray(), kRigExecPortInput, "weights"},
    {&SkinEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional | kRigExecPortPointwise,
     "envelope"},
    {&SkinOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &SkinPoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Skin";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.range.accept = &Accept;
    d.range.prepare = &Prepare;
    d.range.run = &SkinRange;
    d.state = StateOf<SkinState>();
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
