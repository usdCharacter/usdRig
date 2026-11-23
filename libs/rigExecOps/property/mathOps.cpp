#include "rigExecOps/property/mathOps.h"

#include "rigExec/api/opFields.h"
#include "rigExecOps/opIo.h"

#include <cstdint>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(FloatMath)
RIGEXEC_DEFINE_OP_HANDLE(Vec3fMath)
RIGEXEC_DEFINE_OP_HANDLE(MatrixMath)

RIGEXEC_DEFINE_PORT_HANDLE(FloatIn)
RIGEXEC_DEFINE_PORT_HANDLE(FloatValue)
RIGEXEC_DEFINE_PORT_HANDLE(FloatMin)
RIGEXEC_DEFINE_PORT_HANDLE(FloatMax)
RIGEXEC_DEFINE_PORT_HANDLE(FloatWeight)
RIGEXEC_DEFINE_PORT_HANDLE(FloatPathPoints)
RIGEXEC_DEFINE_PORT_HANDLE(FloatOut)

RIGEXEC_DEFINE_PORT_HANDLE(Vec3fIn)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fValue)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fMin)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fMax)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fWeight)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fOut)

RIGEXEC_DEFINE_PORT_HANDLE(MatrixIn)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixValue)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixWeight)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixOut)

namespace {

using rigExec::RigExecBatchActive;
using rigExec::RigExecBatchInstance;
using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExec::RigExecPayloadColumnOf;
using rigExec::RigExecPayloadField;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::Vec2f;
using rigExecStd::Vec3f;
using namespace opIo;

const char *
OpName(RigExecPropertyOp op)
{
    switch (op) {
    case RigExecPropertyOp::Add:
        return "add";
    case RigExecPropertyOp::Multiply:
        return "multiply";
    case RigExecPropertyOp::Clamp:
        return "clamp";
    case RigExecPropertyOp::Remap:
        return "remap";
    case RigExecPropertyOp::Blend:
        return "blend";
    case RigExecPropertyOp::Curve:
        return "curve";
    case RigExecPropertyOp::Quantize:
        return "quantize";
    case RigExecPropertyOp::Path:
        return "path";
    }
    return "unknown";
}

/// The math payload's declared fields, in saved order. A count field
/// holds the length of the list after it.
enum MathField : uint32_t {
    kOp,
    kKeyCount,
    kKeys,
    kTangentCount,
    kTangents,
    kPathValid,
    kPathBasis,
    kPathPinned,
    kPathPlanar,
    kPathComponent,
    kPathOffsetX,
    kPathOffsetZ,
    kPathHeadingOffset,
};

/// Instance \p k's path settings.
rigExec::RigExecPathSettings
PathAt(const RigExecOpBatch &batch, size_t k)
{
    rigExec::RigExecPathSettings path;
    path.valid = RigExecPayloadColumnOf<uint8_t>(batch, kPathValid)[k] != 0;
    path.basis = rigExec::RigExecPathBasis(
        RigExecPayloadColumnOf<uint32_t>(batch, kPathBasis)[k]);
    path.pinned = RigExecPayloadColumnOf<uint8_t>(batch, kPathPinned)[k] != 0;
    path.planar = RigExecPayloadColumnOf<uint8_t>(batch, kPathPlanar)[k] != 0;
    path.component = rigExec::RigExecPathComponent(
        RigExecPayloadColumnOf<uint32_t>(batch, kPathComponent)[k]);
    path.offsetX = RigExecPayloadColumnOf<float>(batch, kPathOffsetX)[k];
    path.offsetZ = RigExecPayloadColumnOf<float>(batch, kPathOffsetZ)[k];
    path.headingOffset =
        RigExecPayloadColumnOf<float>(batch, kPathHeadingOffset)[k];
    return path;
}

bool
FloatOne(const RigExecOpBatch &batch, size_t k, RigExecPropertyOp op,
         std::string *error)
{
    const RigExecOpIo io = RigExecBatchInstance(batch, k);
    rigExec::RigExecPropertyMathKernelParams<float, Vec2f> params;
    params.op = op;
    params.value = InOr(io, FloatValue(), 0.0f);
    params.min = InOr(io, FloatMin(), 0.0f);
    params.max = InOr(io, FloatMax(), 0.0f);
    params.weight = *In<float>(io, FloatWeight());
    if (op == RigExecPropertyOp::Curve) {
        const auto keys = RigExecPayloadField<Vec2f>(batch, kKeys, k);
        const auto tangents = RigExecPayloadField<Vec2f>(batch, kTangents, k);
        if (keys.Empty() ||
            !rigExec::RigExecValidateLinearKeysKernel(keys.data, keys.Size()) ||
            (!tangents.Empty() && tangents.Size() != keys.Size()))
            return Fail(error, "FloatMath",
                        "curve needs finite, strictly increasing keys and "
                        "no tangents or one per key");
        params.keys = keys.data;
        params.keyCount = keys.Size();
        params.tangents = tangents.Empty() ? nullptr : tangents.data;
        params.tangentCount = tangents.Size();
    }
    if (op == RigExecPropertyOp::Path) {
        const rigExec::RigExecPathSettings path = PathAt(batch, k);
        const auto *points =
            In<rigExecStd::RigExecArray<Vec3f>>(io, FloatPathPoints());
        params.path.points =
            points && !points->Empty() ? points->Data()[0]._data : nullptr;
        params.path.pointCount = points ? points->Size() : 0;
        params.path.basis = path.basis;
        params.path.pinned = path.pinned;
        params.path.planar = path.planar;
        params.pathSettings = path;
        if (!path.valid || !rigExec::RigExecPathCurveValid(params.path))
            return Fail(error, "FloatMath", "path has no valid curve");
    }
    Out<float>(io, FloatOut()) =
        rigExec::RigExecApplyFloatMathKernel(*In<float>(io, FloatIn()), params);
    return true;
}

bool
RunFloat(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    const uint32_t *ops = RigExecPayloadColumnOf<uint32_t>(batch, kOp);
    for (size_t k = 0; k < batch.count; ++k) {
        if (RigExecBatchActive(batch, k) &&
            !FloatOne(batch, k, RigExecPropertyOp(ops[k]), error)) {
            *failed = k;
            return false;
        }
    }
    return true;
}

bool
RunVec3f(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    const uint32_t *ops = RigExecPayloadColumnOf<uint32_t>(batch, kOp);
    const Vec3f zero(0.0f, 0.0f, 0.0f);
    for (size_t k = 0; k < batch.count; ++k) {
        if (!RigExecBatchActive(batch, k))
            continue;
        const RigExecPropertyOp op = RigExecPropertyOp(ops[k]);
        if (op == RigExecPropertyOp::Curve ||
            op == RigExecPropertyOp::Quantize ||
            op == RigExecPropertyOp::Path) {
            *failed = k;
            return Fail(error, "Vec3fMath",
                        std::string(OpName(op)) + " is float-only");
        }
        const RigExecOpIo io = RigExecBatchInstance(batch, k);
        rigExec::RigExecPropertyMathKernelParams<Vec3f, Vec2f> params;
        params.op = op;
        params.value = InOr(io, Vec3fValue(), zero);
        params.min = InOr(io, Vec3fMin(), zero);
        params.max = InOr(io, Vec3fMax(), zero);
        params.weight = *In<float>(io, Vec3fWeight());
        Out<Vec3f>(io, Vec3fOut()) = rigExec::RigExecApplyVec3fMathKernel(
            *In<Vec3f>(io, Vec3fIn()), params);
    }
    return true;
}

bool
RunMatrix(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    const uint32_t *ops = RigExecPayloadColumnOf<uint32_t>(batch, kOp);
    const Mat4d identity(1.0);
    for (size_t k = 0; k < batch.count; ++k) {
        if (!RigExecBatchActive(batch, k))
            continue;
        const RigExecPropertyOp op = RigExecPropertyOp(ops[k]);
        const RigExecOpIo io = RigExecBatchInstance(batch, k);
        const Mat4d *value = In<Mat4d>(io, MatrixValue());
        if (!rigExec::RigExecApplyMatrixMathKernel(
                *In<Mat4d>(io, MatrixIn()), op, value ? *value : identity,
                *In<float>(io, MatrixWeight()), &Out<Mat4d>(io, MatrixOut()))) {
            *failed = k;
            return Fail(error, "MatrixMath",
                        std::string(OpName(op)) + " has no matrix meaning");
        }
    }
    return true;
}

void
MathValues(const void *payload, rigExec::RigExecFieldValues &out)
{
    const MathPayload &p = *static_cast<const MathPayload *>(payload);
    out.Set(kOp, uint32_t(p.op));
    out.Set(kKeyCount, uint64_t(p.keys.size()));
    out.Set(kKeys, p.keys);
    out.Set(kTangentCount, uint64_t(p.tangents.size()));
    out.Set(kTangents, p.tangents);
    out.Set(kPathValid, uint8_t(p.path.valid));
    out.Set(kPathBasis, uint32_t(p.path.basis));
    out.Set(kPathPinned, uint8_t(p.path.pinned));
    out.Set(kPathPlanar, uint8_t(p.path.planar));
    out.Set(kPathComponent, uint32_t(p.path.component));
    out.Set(kPathOffsetX, p.path.offsetX);
    out.Set(kPathOffsetZ, p.path.offsetZ);
    out.Set(kPathHeadingOffset, p.path.headingOffset);
}

rigExec::RigExecFieldDesc
Field(const char *name, const rigExec::RigExecTypeHandle &type,
      uint64_t limit = UINT64_MAX)
{
    rigExec::RigExecFieldDesc f;
    f.name = name;
    f.type = &type;
    f.limit = limit;
    return f;
}

rigExec::RigExecFieldDesc
ListField(const char *name, uint32_t countField)
{
    rigExec::RigExecFieldDesc f = Field(name, rigExecStd::Vec2fType());
    f.extent = rigExec::RigExecFieldExtent::FromField;
    f.source = countField;
    return f;
}

const rigExec::RigExecFieldDesc kMathFields[] = {
    Field("op", rigExecStd::UIntType(), uint64_t(RigExecPropertyOp::Path)),
    Field("keyCount", rigExecStd::UInt64Type()),
    ListField("keys", kKeyCount),
    Field("tangentCount", rigExecStd::UInt64Type()),
    ListField("tangents", kTangentCount),
    Field("pathValid", rigExecStd::BoolType(), 1),
    Field("pathBasis", rigExecStd::UIntType(),
          uint64_t(rigExec::RigExecPathBasis::CatmullRom)),
    Field("pathPinned", rigExecStd::BoolType(), 1),
    Field("pathPlanar", rigExecStd::BoolType(), 1),
    Field("pathComponent", rigExecStd::UIntType(),
          uint64_t(rigExec::RigExecPathComponent::End)),
    Field("pathOffsetX", rigExecStd::FloatType()),
    Field("pathOffsetZ", rigExecStd::FloatType()),
    Field("pathHeadingOffset", rigExecStd::FloatType()),
};

rigExec::RigExecOpDesc
MathDesc(const char *name, const rigExec::RigExecPortDesc *ports,
         size_t portCount, rigExec::RigExecOpBatchFn runBatch)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.payloadFields = kMathFields;
    d.payloadFieldCount = std::size(kMathFields);
    d.payloadVersion = 1;
    d.payloadValues = &MathValues;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    return d;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kFloatPorts[] = {
    {&FloatIn(), &rigExecStd::FloatType(), kRigExecPortInput, "in"},
    {&FloatValue(), &rigExecStd::FloatType(), kOptional, "value"},
    {&FloatMin(), &rigExecStd::FloatType(), kOptional, "min"},
    {&FloatMax(), &rigExecStd::FloatType(), kOptional, "max"},
    {&FloatWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&FloatPathPoints(), &rigExecStd::Vec3fArray(), kOptional, "pathPoints"},
    {&FloatOut(), &rigExecStd::FloatType(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecPortDesc kVec3fPorts[] = {
    {&Vec3fIn(), &rigExecStd::Vec3fType(), kRigExecPortInput, "in"},
    {&Vec3fValue(), &rigExecStd::Vec3fType(), kOptional, "value"},
    {&Vec3fMin(), &rigExecStd::Vec3fType(), kOptional, "min"},
    {&Vec3fMax(), &rigExecStd::Vec3fType(), kOptional, "max"},
    {&Vec3fWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&Vec3fOut(), &rigExecStd::Vec3fType(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecPortDesc kMatrixPorts[] = {
    {&MatrixIn(), &rigExecStd::Mat4dType(), kRigExecPortInput, "in"},
    {&MatrixValue(), &rigExecStd::Mat4dType(), kOptional, "value"},
    {&MatrixWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&MatrixOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kFloatDesc =
    MathDesc("FloatMath", kFloatPorts, std::size(kFloatPorts), &RunFloat);

const rigExec::RigExecOpDesc kVec3fDesc =
    MathDesc("Vec3fMath", kVec3fPorts, std::size(kVec3fPorts), &RunVec3f);

const rigExec::RigExecOpDesc kMatrixDesc =
    MathDesc("MatrixMath", kMatrixPorts, std::size(kMatrixPorts), &RunMatrix);

} // namespace

void
RegisterPropertyMathOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(FloatMath(), kFloatDesc);
    registrar.Add(Vec3fMath(), kVec3fDesc);
    registrar.Add(MatrixMath(), kMatrixDesc);
}

} // namespace rigExecOps
