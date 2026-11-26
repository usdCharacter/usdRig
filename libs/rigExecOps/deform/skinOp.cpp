#include "rigExecOps/deform/skinOp.h"

#include "rigExecMath/skinKernel.h"
#include "rigExecOps/authoring.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/internalOps.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecSchema/schemaNames.h"

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
RIGEXEC_DEFINE_PORT_HANDLE(SkinRows)
RIGEXEC_DEFINE_PORT_HANDLE(SkinPaletteReal)
RIGEXEC_DEFINE_PORT_HANDLE(SkinPaletteDual)
RIGEXEC_DEFINE_PORT_HANDLE(SkinPaletteStretch)
RIGEXEC_DEFINE_PORT_HANDLE(SkinPaletteRigid)

RIGEXEC_DEFINE_OP_HANDLE(SkinMatrices)

RIGEXEC_DEFINE_PORT_HANDLE(SkinMatricesInfluences)
RIGEXEC_DEFINE_PORT_HANDLE(SkinMatricesMatrices)
RIGEXEC_DEFINE_PORT_HANDLE(SkinMatricesRows)
RIGEXEC_DEFINE_PORT_HANDLE(SkinMatricesReal)
RIGEXEC_DEFINE_PORT_HANDLE(SkinMatricesDual)
RIGEXEC_DEFINE_PORT_HANDLE(SkinMatricesStretch)
RIGEXEC_DEFINE_PORT_HANDLE(SkinMatricesRigid)

namespace {

using namespace opIo;
using namespace fieldIo;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Transforms = rigExecStd::RigExecArray<Mat4d>;
using Indices = rigExecStd::RigExecArray<int32_t>;
using Quats = rigExecStd::RigExecArray<rigExecStd::Quatd>;
using Stretches = rigExecStd::RigExecArray<rigExecStd::Mat3d>;
using Flags = rigExecStd::RigExecArray<uint8_t>;
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
    const float *rowsData = nullptr;
    Palette palette;
    /// The indices and weights validate could not see, checked each frame.
    bool checkEachFrame = true;
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

/// Whether every index names one of \p transforms influences and every
/// weight is finite and non-negative.
bool
IndicesAndWeightsFit(const Indices &indices, const Envelope &weights,
                     size_t transforms, std::string *error)
{
    for (size_t i = 0; i < indices.Size(); ++i)
        if (indices[i] < 0 || size_t(indices[i]) >= transforms)
            return Fail(error, "Skin", "an index is outside the influences");
    for (size_t i = 0; i < weights.Size(); ++i)
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f)
            return Fail(error, "Skin",
                        "weights must be finite and non-negative");
    return true;
}

bool
Validate(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const int elementSize = rigExec::RigExecPayloadColumnOf<int32_t>(
        check.batch, kElementSize)[check.k];
    if (elementSize < 1)
        return Fail(error, "Skin", "elementSize must be at least 1");
    const size_t points = CheckElements(check, SkinPoints());
    const size_t transforms = CheckElements(check, SkinTransforms());
    const size_t indices = CheckElements(check, SkinIndices());
    const size_t weights = CheckElements(check, SkinWeights());
    if (transforms == 0)
        return Fail(error, "Skin", "needs an influence");
    if (!SameCount(indices, weights) ||
        (points < rigExec::kRigExecPortUnfed &&
         !SameCount(indices, points * size_t(elementSize))))
        return Fail(error, "Skin",
                    "indices and weights need elementSize per point");
    if (!EnvelopeFits(check, SkinPoints(), SkinEnvelope(), "Skin", error))
        return false;
    const int paletteFed = int(Fed(check, SkinPaletteReal())) +
                           int(Fed(check, SkinPaletteDual())) +
                           int(Fed(check, SkinPaletteStretch())) +
                           int(Fed(check, SkinPaletteRigid()));
    if (paletteFed != 0 && paletteFed != 4)
        return Fail(error, "Skin", "the palette needs all four of its ports");
    const Indices *indexValues = CheckValue<Indices>(check, SkinIndices());
    const Envelope *weightValues = CheckValue<Envelope>(check, SkinWeights());
    const bool known =
        indexValues && weightValues && transforms < rigExec::kRigExecPortUnfed;
    if (SkinState *state = CheckState<SkinState>(check))
        state->checkEachFrame = !known;
    return !known ||
           IndicesAndWeightsFit(*indexValues, *weightValues, transforms, error);
}

/// The palette SkinMatrices folded, one entry per influence, then the
/// identity entry a weight shortfall blends.
bool
PaletteFrom(const rigExec::RigExecOpIo &io, size_t count, Palette *palette,
            std::string *error)
{
    const Quats &real = *In<Quats>(io, SkinPaletteReal());
    const Quats &dual = *In<Quats>(io, SkinPaletteDual());
    const Stretches &stretch = *In<Stretches>(io, SkinPaletteStretch());
    const Flags &rigid = *In<Flags>(io, SkinPaletteRigid());
    if (real.Size() != count || dual.Size() != count ||
        stretch.Size() != count || rigid.Size() != count)
        return Fail(error, "Skin", "the palette does not match the transforms");
    palette->resize(count + 1);
    for (size_t t = 0; t < count; ++t) {
        auto &entry = (*palette)[t];
        entry.rigid.real = real[t];
        entry.rigid.dual = dual[t];
        entry.stretch = stretch[t];
        entry.isRigid = rigid[t] != 0;
    }
    (*palette)[count] = Palette::value_type();
    return true;
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
        std::string *error)
{
    SkinState &state =
        *static_cast<SkinState *>(rigExec::RigExecBatchState(batch, k));
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const Transforms &transforms = *In<Transforms>(io, SkinTransforms());
    if (!Usable(transforms))
        return Fail(error, "Skin", "transforms must be finite and affine");
    if (!EnvelopeFits(*In<Points>(io, SkinPoints()),
                      In<Envelope>(io, SkinEnvelope()), "Skin", error))
        return false;
    if (state.checkEachFrame &&
        !IndicesAndWeightsFit(*In<Indices>(io, SkinIndices()),
                              *In<Envelope>(io, SkinWeights()),
                              transforms.Size(), error))
        return false;
    const Layout layout = LayoutOf(batch, k);
    const auto method = SkinMethod(
        rigExec::RigExecPayloadColumnOf<uint32_t>(batch, kMethod)[k]);
    const bool dualQuat = method != SkinMethod::ClassicLinear;
    const Envelope *rows = In<Envelope>(io, SkinRows());
    if (dualQuat && In<Quats>(io, SkinPaletteReal()))
        return PaletteFrom(io, layout.transformCount, &state.palette, error);
    if (!dualQuat && batch.useSimd && rows) {
        if (rows->Size() !=
            layout.transformCount * rigExec::kRigExecSkinRowStride)
            return Fail(error, "Skin", "the rows do not match the transforms");
        state.rowsData = rows->Data();
        return true;
    }
    rigExec::RigExecFoldSkinMatricesKernel<Vec3f, Vec3d>(
        dualQuat, layout.transforms, layout.transformCount, batch.useSimd,
        &state.rows, &state.palette);
    state.rowsData = state.rows.data();
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
    const Vec3f *in = RangeIn<Vec3f>(io, range, SkinPoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, SkinOut());
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
                in, out, part, state.rowsData);
        else
            rigExec::RigExecApplyLinearBlendSkinKernel<Vec3f, Vec3d>(in, out,
                                                                     part);
    } else if (!rigExec::RigExecApplyDualQuatSkinKernel<Vec3f, Vec3d>(
                   in, out, part, state.palette.data(), state.palette.size())) {
        return Fail(error, "Skin", "dual quaternion blend is degenerate");
    }
    BlendEnvelopeAt(in, RangeIn<float>(io, range, SkinEnvelope()), out,
                    end - begin);
    return true;
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const SkinPayload &p = *static_cast<const SkinPayload *>(payload);
    SetEnum(out, kMethod, p.method);
    out.Set(kElementSize, int32_t(p.elementSize));
}

namespace names = rigExec::SchemaAttrText;
namespace values = rigExec::SchemaValueText;
namespace types = rigExec::SchemaTypeName;
using authoring::Authored;

const uint32_t kClassicLinear = uint32_t(SkinMethod::ClassicLinear);
const int32_t kOneInfluence = 1;
const rigExecStd::RigExecArray<int32_t> kNoInts;
const rigExecStd::RigExecArray<float> kNoFloats;
const char *const kMethods[] = {values::classicLinear, values::dualQuaternion};

const rigExec::RigExecPropertyInfo kInfluences = authoring::Relationship(
    names::rigExecInfluences,
    R"(Ordered GfMatrix4d providers (computeMatrix), one per influence.
rigExec:jointIndices index this list. Named influences, not joints:
rigExec:joints is a solver's output claim, and this mover only reads.)");
const rigExec::RigExecPropertyInfo kJointIndices = authoring::Property(
    names::rigExecJointIndices, &kNoInts,
    R"(Per-point influence indices into rigExec:influences, rigExec:elementSize
entries per point in point order (UsdSkel's primvars:skel:jointIndices
layout).)");
const rigExec::RigExecPropertyInfo kJointWeights = authoring::Property(
    names::rigExecJointWeights, &kNoFloats,
    R"(Per-point influence weights parallel to rigExec:jointIndices. Finite and
non-negative; not renormalized.)");
const rigExec::RigExecPropertyInfo kElementSizeProperty = authoring::Uniform(
    names::rigExecElementSize, &kOneInfluence, R"(Influences per point.)");
const rigExec::RigExecPropertyInfo kSkinningMethod =
    authoring::Tokens(names::rigExecSkinningMethod, &kClassicLinear, kMethods);

const rigExec::RigExecSchemaInfo kSkinSchema{
    types::RigExecSkinMover,
    R"(Multi-influence skinning of an exact native point3f[] points property in
ONE pass: every influence's matrix is gathered per point and blended by
that point's authored weights, then the result is mixed with the common
MoverAPI envelope.

RigExecMatrixMover applies one influence per mover, sequentially, each
from the PRECEDING revision. That is exact for a point with a single
influence and diverges from a skinCluster everywhere weights blend, so a
skinned character needs this mover: the same jointIndices / jointWeights
layout UsdSkel uses, with the influence list in rigExec:influences
standing in for UsdSkel's joint order.

classicLinear: p' = p + sum_i w_i (T_i p - p) = (1 - sum_i w_i) p +
sum_i w_i T_i p. With weights that sum to one this is exactly linear
blend skinning; a shortfall keeps the rest point in proportion, so a
partially weighted point stays put rather than collapsing toward the
origin. dualQuaternion blends the influences' rigid motions as dual
quaternions (shortest arc, one normalisation, so a twisted joint keeps
its volume instead of candy-wrapping) and their scale and shear linearly
in each joint's pre-rotation frame, so squash-and-stretch survives; the
same weight shortfall enters that blend as an identity influence, so a
partially weighted point is likewise held toward rest.)",
    authoring::kTyped, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecFieldDesc kFields[] = {
    Authored(Enum("method", SkinMethod::DualQuaternion), kSkinningMethod),
    Authored(Value("elementSize", rigExecStd::IntType()), kElementSizeProperty),
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
    Authored({&SkinTransforms(), &rigExecStd::Mat4dArray(), kRigExecPortInput,
              "transforms"},
             kInfluences),
    Authored({&SkinIndices(), &rigExecStd::IntArray(),
              kRigExecPortInput | rigExec::kRigExecPortChecked, "indices"},
             kJointIndices),
    Authored({&SkinWeights(), &rigExecStd::FloatArray(),
              kRigExecPortInput | rigExec::kRigExecPortChecked, "weights"},
             kJointWeights),
    {&SkinEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional | kRigExecPortPointwise,
     "envelope"},
    {&SkinOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &SkinPoints()},
    {&SkinRows(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "rows"},
    {&SkinPaletteReal(), &rigExecStd::QuatdArray(),
     kRigExecPortInput | kRigExecPortOptional, "paletteReal"},
    {&SkinPaletteDual(), &rigExecStd::QuatdArray(),
     kRigExecPortInput | kRigExecPortOptional, "paletteDual"},
    {&SkinPaletteStretch(), &rigExecStd::Mat3dArray(),
     kRigExecPortInput | kRigExecPortOptional, "paletteStretch"},
    {&SkinPaletteRigid(), &rigExecStd::BoolArray(),
     kRigExecPortInput | kRigExecPortOptional, "paletteRigid"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Skin";
    d.schema = &kSkinSchema;
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.validate = &Validate;
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

namespace {

enum SkinMatricesField : uint32_t { kMatricesMethod };

/// The folded forms, kept between frames so a fold reuses their storage.
struct SkinMatricesState {
    std::vector<float> rows;
    Palette palette;
};

bool
MatricesOne(const rigExec::RigExecOpBatch &batch, size_t k, std::string *error)
{
    const rigExec::RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    SkinMatricesState &state =
        *static_cast<SkinMatricesState *>(rigExec::RigExecBatchState(batch, k));
    const Transforms &in = *In<Transforms>(io, SkinMatricesInfluences());
    if (in.Size() == 0)
        return Fail(error, "SkinMatrices", "needs an influence");
    if (!Usable(in))
        return Fail(error, "SkinMatrices",
                    "transforms must be finite and affine");
    const bool dualQuat =
        SkinMethod(rigExec::RigExecPayloadColumnOf<uint32_t>(
            batch, kMatricesMethod)[k]) != SkinMethod::ClassicLinear;
    rigExec::RigExecFoldSkinMatricesKernel<Vec3f, Vec3d>(
        dualQuat, in.Data(), in.Size(), io.useSimd, &state.rows,
        &state.palette);
    Out<Transforms>(io, SkinMatricesMatrices()).Assign(in.Data(), in.Size());
    Out<Envelope>(io, SkinMatricesRows())
        .Assign(state.rows.data(), state.rows.size());
    const size_t count = state.palette.empty() ? 0 : in.Size();
    Quats &real = Out<Quats>(io, SkinMatricesReal());
    Quats &dual = Out<Quats>(io, SkinMatricesDual());
    Stretches &stretch = Out<Stretches>(io, SkinMatricesStretch());
    Flags &rigid = Out<Flags>(io, SkinMatricesRigid());
    real.Resize(count);
    dual.Resize(count);
    stretch.Resize(count);
    rigid.Resize(count);
    for (size_t t = 0; t < count; ++t) {
        const auto &entry = state.palette[t];
        real[t] = entry.rigid.real;
        dual[t] = entry.rigid.dual;
        stretch[t] = entry.stretch;
        rigid[t] = entry.isRigid ? 1 : 0;
    }
    return true;
}

bool
RunMatrices(const rigExec::RigExecOpBatch &batch, size_t *failed,
            std::string *error)
{
    return EachInstance(batch, failed,
                        [&](size_t k) { return MatricesOne(batch, k, error); });
}

bool
ValidateMatrices(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (CheckElements(check, SkinMatricesInfluences()) == 0)
        return Fail(error, "SkinMatrices", "needs an influence");
    return true;
}

void
MatricesValues(const void *payload, rigExec::RigExecFieldValues &out)
{
    const SkinMatricesPayload &p =
        *static_cast<const SkinMatricesPayload *>(payload);
    SetEnum(out, kMatricesMethod, p.method);
}

const rigExec::RigExecFieldDesc kMatricesFields[] = {
    Enum("method", SkinMethod::DualQuaternion),
};

constexpr auto kPerInfluence = rigExec::RigExecOutputBound::Input;

const rigExec::RigExecPortDesc kMatricesPorts[] = {
    {&SkinMatricesInfluences(), &rigExecStd::Mat4dArray(), kRigExecPortInput,
     "influences"},
    {&SkinMatricesMatrices(), &rigExecStd::Mat4dArray(), kRigExecPortOutput,
     "matrices", kPerInfluence, &SkinMatricesInfluences()},
    {&SkinMatricesRows(), &rigExecStd::FloatArray(), kRigExecPortOutput, "rows",
     kPerInfluence, &SkinMatricesInfluences(), nullptr,
     rigExec::kRigExecInvalidIndex, rigExec::kRigExecSkinRowStride},
    {&SkinMatricesReal(), &rigExecStd::QuatdArray(), kRigExecPortOutput, "real",
     kPerInfluence, &SkinMatricesInfluences()},
    {&SkinMatricesDual(), &rigExecStd::QuatdArray(), kRigExecPortOutput, "dual",
     kPerInfluence, &SkinMatricesInfluences()},
    {&SkinMatricesStretch(), &rigExecStd::Mat3dArray(), kRigExecPortOutput,
     "stretch", kPerInfluence, &SkinMatricesInfluences()},
    {&SkinMatricesRigid(), &rigExecStd::BoolArray(), kRigExecPortOutput,
     "rigid", kPerInfluence, &SkinMatricesInfluences()},
};

const rigExec::RigExecOpDesc kMatricesDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SkinMatrices";
    d.schema = &internalOps::kSkinMatrices;
    d.ports = kMatricesPorts;
    d.portCount = std::size(kMatricesPorts);
    d.validate = &ValidateMatrices;
    d.runBatch = &RunMatrices;
    d.state = StateOf<SkinMatricesState>();
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kMatricesFields;
    d.payloadFieldCount = std::size(kMatricesFields);
    d.payloadVersion = 1;
    d.payloadValues = &MatricesValues;
    return d;
}();

} // namespace

void
RegisterSkinOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Skin(), kDesc);
    registrar.Add(SkinMatrices(), kMatricesDesc);
}

} // namespace rigExecOps
