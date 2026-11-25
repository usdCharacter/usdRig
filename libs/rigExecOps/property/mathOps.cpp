#include "rigExecOps/property/mathOps.h"

#include "rigExec/api/opFields.h"
#include "rigExecOps/opIo.h"

#include <cmath>
#include <cstdint>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(FloatMath)
RIGEXEC_DEFINE_OP_HANDLE(Vec3fMath)
RIGEXEC_DEFINE_OP_HANDLE(MatrixMath)
RIGEXEC_DEFINE_OP_HANDLE(DoubleMath)

RIGEXEC_DEFINE_PORT_HANDLE(FloatIn)
RIGEXEC_DEFINE_PORT_HANDLE(FloatValue)
RIGEXEC_DEFINE_PORT_HANDLE(FloatMin)
RIGEXEC_DEFINE_PORT_HANDLE(FloatMax)
RIGEXEC_DEFINE_PORT_HANDLE(FloatWeight)
RIGEXEC_DEFINE_PORT_HANDLE(FloatPathPoints)
RIGEXEC_DEFINE_PORT_HANDLE(FloatOut)
RIGEXEC_DEFINE_PORT_HANDLE(FloatEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(FloatKeys)
RIGEXEC_DEFINE_PORT_HANDLE(FloatTangents)

RIGEXEC_DEFINE_PORT_HANDLE(DoubleIn)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleValue)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleMin)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleMax)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleWeight)
RIGEXEC_DEFINE_PORT_HANDLE(DoublePathPoints)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleOut)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleKeys)
RIGEXEC_DEFINE_PORT_HANDLE(DoubleTangents)

RIGEXEC_DEFINE_PORT_HANDLE(Vec3fIn)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fValue)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fMin)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fMax)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fWeight)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fOut)
RIGEXEC_DEFINE_PORT_HANDLE(Vec3fEnabled)

RIGEXEC_DEFINE_PORT_HANDLE(MatrixIn)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixValue)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixWeight)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixOut)
RIGEXEC_DEFINE_PORT_HANDLE(MatrixEnabled)

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

using Keys = rigExecStd::RigExecArray<Vec2f>;

bool
Finite(float v)
{
    return std::isfinite(v);
}

bool
Finite(const Vec3f &v)
{
    return std::isfinite(v._data[0]) && std::isfinite(v._data[1]) &&
           std::isfinite(v._data[2]);
}

bool
Finite(const Mat4d &m)
{
    for (size_t r = 0; r < 4; ++r)
        for (size_t c = 0; c < 4; ++c)
            if (!std::isfinite(m._mtx[r][c]))
                return false;
    return true;
}

/// The ports a scalar op (FloatMath, DoubleMath) reads and writes.
struct ScalarPorts {
    const char *name;
    const RigExecPortHandle &in, &value, &min, &max, &weight, &pathPoints, &out,
        &enabled, &keys, &tangents;
};

ScalarPorts
FloatPorts()
{
    return {"FloatMath",    FloatIn(),     FloatValue(),      FloatMin(),
            FloatMax(),     FloatWeight(), FloatPathPoints(), FloatOut(),
            FloatEnabled(), FloatKeys(),   FloatTangents()};
}

ScalarPorts
DoublePorts()
{
    return {"DoubleMath",    DoubleIn(),     DoubleValue(),      DoubleMin(),
            DoubleMax(),     DoubleWeight(), DoublePathPoints(), DoubleOut(),
            DoubleEnabled(), DoubleKeys(),   DoubleTangents()};
}

/// False with the error unless \p weight mixes: finite and in [0, 1].
bool
WeightUsable(float weight, const char *op, std::string *error)
{
    if (std::isfinite(weight) && weight >= 0.0f && weight <= 1.0f)
        return true;
    return Fail(error, op, "weight must be finite and in [0, 1]");
}

/// Instance \p k of a scalar op: in when disabled, else the operation
/// computed in float on float(in). T is float or double.
template <class T>
bool
ScalarOne(const RigExecOpBatch &batch, size_t k, RigExecPropertyOp op,
          const ScalarPorts &p, std::string *error)
{
    const RigExecOpIo io = RigExecBatchInstance(batch, k);
    const T in = *In<T>(io, p.in);
    if (!InOr<uint8_t>(io, p.enabled, 1)) {
        Out<T>(io, p.out) = in;
        return true;
    }
    rigExec::RigExecPropertyMathKernelParams<float, Vec2f> params;
    params.op = op;
    params.weight = *In<float>(io, p.weight);
    if (!WeightUsable(params.weight, p.name, error))
        return false;
    params.value = InOr(io, p.value, 0.0f);
    params.min = InOr(io, p.min, 0.0f);
    params.max = InOr(io, p.max, 0.0f);
    if (!Finite(params.value) || !Finite(params.min) || !Finite(params.max))
        return Fail(error, p.name, "value, min or max is not finite");
    if (op == RigExecPropertyOp::Curve) {
        const Keys *keys = In<Keys>(io, p.keys);
        const Keys *tangents = In<Keys>(io, p.tangents);
        const size_t keyCount = keys ? keys->Size() : 0;
        const size_t tangentCount = tangents ? tangents->Size() : 0;
        if (!keyCount ||
            !rigExec::RigExecValidateLinearKeysKernel(keys->Data(), keyCount) ||
            (tangentCount && tangentCount != keyCount))
            return Fail(error, p.name,
                        "curve needs finite, strictly increasing keys and "
                        "no tangents or one per key");
        params.keys = keys->Data();
        params.keyCount = keyCount;
        params.tangents = tangentCount ? tangents->Data() : nullptr;
        params.tangentCount = tangentCount;
    }
    if (op == RigExecPropertyOp::Path) {
        const rigExec::RigExecPathSettings path = PathAt(batch, k);
        const auto *points =
            In<rigExecStd::RigExecArray<Vec3f>>(io, p.pathPoints);
        params.path.points =
            points && !points->Empty() ? points->Data()[0]._data : nullptr;
        params.path.pointCount = points ? points->Size() : 0;
        params.path.basis = path.basis;
        params.path.pinned = path.pinned;
        params.path.planar = path.planar;
        params.pathSettings = path;
        if (!rigExec::RigExecPathCurveValid(params.path))
            return Fail(error, p.name, "path has no valid curve");
    }
    const float result =
        rigExec::RigExecApplyFloatMathKernel(float(in), params);
    if (!Finite(result))
        return Fail(error, p.name, "produced a non-finite value");
    Out<T>(io, p.out) = T(result);
    return true;
}

template <class T>
bool
RunScalar(const RigExecOpBatch &batch, const ScalarPorts &p, size_t *failed,
          std::string *error)
{
    const uint32_t *ops = RigExecPayloadColumnOf<uint32_t>(batch, kOp);
    for (size_t k = 0; k < batch.count; ++k) {
        if (RigExecBatchActive(batch, k) &&
            !ScalarOne<T>(batch, k, RigExecPropertyOp(ops[k]), p, error)) {
            *failed = k;
            return false;
        }
    }
    return true;
}

bool
RunFloat(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return RunScalar<float>(batch, FloatPorts(), failed, error);
}

bool
RunDouble(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return RunScalar<double>(batch, DoublePorts(), failed, error);
}

bool
RunVec3f(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    const uint32_t *ops = RigExecPayloadColumnOf<uint32_t>(batch, kOp);
    const Vec3f zero(0.0f, 0.0f, 0.0f);
    for (size_t k = 0; k < batch.count; ++k) {
        if (!RigExecBatchActive(batch, k))
            continue;
        const RigExecOpIo io = RigExecBatchInstance(batch, k);
        const Vec3f &in = *In<Vec3f>(io, Vec3fIn());
        if (!InOr<uint8_t>(io, Vec3fEnabled(), 1)) {
            Out<Vec3f>(io, Vec3fOut()) = in;
            continue;
        }
        rigExec::RigExecPropertyMathKernelParams<Vec3f, Vec2f> params;
        params.op = RigExecPropertyOp(ops[k]);
        params.weight = *In<float>(io, Vec3fWeight());
        params.value = InOr(io, Vec3fValue(), zero);
        params.min = InOr(io, Vec3fMin(), zero);
        params.max = InOr(io, Vec3fMax(), zero);
        bool ok = WeightUsable(params.weight, "Vec3fMath", error);
        if (ok && (!Finite(params.value) || !Finite(params.min) ||
                   !Finite(params.max)))
            ok = Fail(error, "Vec3fMath", "value, min or max is not finite");
        Vec3f result;
        if (ok) {
            result = rigExec::RigExecApplyVec3fMathKernel(in, params);
            if (!Finite(result))
                ok = Fail(error, "Vec3fMath", "produced a non-finite value");
        }
        if (!ok) {
            *failed = k;
            return false;
        }
        Out<Vec3f>(io, Vec3fOut()) = result;
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
        const RigExecOpIo io = RigExecBatchInstance(batch, k);
        const Mat4d &in = *In<Mat4d>(io, MatrixIn());
        if (!InOr<uint8_t>(io, MatrixEnabled(), 1)) {
            Out<Mat4d>(io, MatrixOut()) = in;
            continue;
        }
        const float weight = *In<float>(io, MatrixWeight());
        const Mat4d *value = In<Mat4d>(io, MatrixValue());
        bool ok = WeightUsable(weight, "MatrixMath", error);
        if (ok && value && !Finite(*value))
            ok = Fail(error, "MatrixMath", "value is not finite");
        Mat4d result;
        if (ok && !rigExec::RigExecApplyMatrixMathKernel(
                      in, RigExecPropertyOp(ops[k]), value ? *value : identity,
                      weight, &result))
            ok = Fail(error, "MatrixMath",
                      "the operation has no matrix meaning");
        if (ok && !Finite(result))
            ok = Fail(error, "MatrixMath", "produced a non-finite value");
        if (!ok) {
            *failed = k;
            return false;
        }
        Out<Mat4d>(io, MatrixOut()) = result;
    }
    return true;
}

/// What a node's settings and feeds show before it runs: an operation its
/// value type has, a curve fed keys, a path with a curve, tangents one per
/// key where both counts are known.
bool
CheckMath(const rigExec::RigExecOpCheck &check, const char *name, bool scalar,
          bool vec3f, const ScalarPorts *p, std::string *error)
{
    const RigExecPropertyOp op = RigExecPropertyOp(
        RigExecPayloadColumnOf<uint32_t>(check.batch, kOp)[check.k]);
    const bool floatOnly = op == RigExecPropertyOp::Curve ||
                           op == RigExecPropertyOp::Quantize ||
                           op == RigExecPropertyOp::Path;
    if (vec3f && floatOnly)
        return Fail(error, name, std::string(OpName(op)) + " is float-only");
    if (!scalar && !vec3f && op != RigExecPropertyOp::Multiply &&
        op != RigExecPropertyOp::Blend)
        return Fail(error, name,
                    std::string(OpName(op)) + " has no matrix meaning");
    if (!p)
        return true;
    if (op == RigExecPropertyOp::Curve) {
        if (!Fed(check, p->keys))
            return Fail(error, name, "curve needs keys");
        const size_t keys = CheckElements(check, p->keys);
        const size_t tangents = CheckElements(check, p->tangents);
        if (keys < rigExec::kRigExecPortUnfed &&
            tangents < rigExec::kRigExecPortUnfed && tangents &&
            tangents != keys)
            return Fail(error, name, "curve needs no tangents or one per key");
    }
    if (op == RigExecPropertyOp::Path &&
        (!RigExecPayloadColumnOf<uint8_t>(check.batch, kPathValid)[check.k] ||
         !Fed(check, p->pathPoints)))
        return Fail(error, name, "path has no supported curve");
    return true;
}

bool
ValidateFloat(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const ScalarPorts p = FloatPorts();
    return CheckMath(check, p.name, true, false, &p, error);
}

bool
ValidateDouble(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const ScalarPorts p = DoublePorts();
    return CheckMath(check, p.name, true, false, &p, error);
}

bool
ValidateVec3f(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return CheckMath(check, "Vec3fMath", false, true, nullptr, error);
}

bool
ValidateMatrix(const rigExec::RigExecOpCheck &check, std::string *error)
{
    return CheckMath(check, "MatrixMath", false, false, nullptr, error);
}

void
MathValues(const void *payload, rigExec::RigExecFieldValues &out)
{
    const MathPayload &p = *static_cast<const MathPayload *>(payload);
    out.Set(kOp, uint32_t(p.op));
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

const rigExec::RigExecFieldDesc kMathFields[] = {
    Field("op", rigExecStd::UIntType(), uint64_t(RigExecPropertyOp::Path)),
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
         size_t portCount, rigExec::RigExecOpBatchFn runBatch,
         bool (*validate)(const rigExec::RigExecOpCheck &, std::string *))
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.payloadFields = kMathFields;
    d.payloadFieldCount = std::size(kMathFields);
    d.payloadVersion = 2;
    d.payloadValues = &MathValues;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.validate = validate;
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
    {&FloatEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    {&FloatKeys(), &rigExecStd::Vec2fArray(), kOptional, "keys"},
    {&FloatTangents(), &rigExecStd::Vec2fArray(), kOptional, "tangents"},
};

const rigExec::RigExecPortDesc kDoublePorts[] = {
    {&DoubleIn(), &rigExecStd::DoubleType(), kRigExecPortInput, "in"},
    {&DoubleValue(), &rigExecStd::FloatType(), kOptional, "value"},
    {&DoubleMin(), &rigExecStd::FloatType(), kOptional, "min"},
    {&DoubleMax(), &rigExecStd::FloatType(), kOptional, "max"},
    {&DoubleWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&DoublePathPoints(), &rigExecStd::Vec3fArray(), kOptional, "pathPoints"},
    {&DoubleOut(), &rigExecStd::DoubleType(), kRigExecPortOutput, "out"},
    {&DoubleEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    {&DoubleKeys(), &rigExecStd::Vec2fArray(), kOptional, "keys"},
    {&DoubleTangents(), &rigExecStd::Vec2fArray(), kOptional, "tangents"},
};

const rigExec::RigExecPortDesc kVec3fPorts[] = {
    {&Vec3fIn(), &rigExecStd::Vec3fType(), kRigExecPortInput, "in"},
    {&Vec3fValue(), &rigExecStd::Vec3fType(), kOptional, "value"},
    {&Vec3fMin(), &rigExecStd::Vec3fType(), kOptional, "min"},
    {&Vec3fMax(), &rigExecStd::Vec3fType(), kOptional, "max"},
    {&Vec3fWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&Vec3fOut(), &rigExecStd::Vec3fType(), kRigExecPortOutput, "out"},
    {&Vec3fEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
};

const rigExec::RigExecPortDesc kMatrixPorts[] = {
    {&MatrixIn(), &rigExecStd::Mat4dType(), kRigExecPortInput, "in"},
    {&MatrixValue(), &rigExecStd::Mat4dType(), kOptional, "value"},
    {&MatrixWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&MatrixOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
    {&MatrixEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
};

const rigExec::RigExecOpDesc kFloatDesc =
    MathDesc("FloatMath", kFloatPorts, std::size(kFloatPorts), &RunFloat,
             &ValidateFloat);

const rigExec::RigExecOpDesc kDoubleDesc =
    MathDesc("DoubleMath", kDoublePorts, std::size(kDoublePorts), &RunDouble,
             &ValidateDouble);

const rigExec::RigExecOpDesc kVec3fDesc =
    MathDesc("Vec3fMath", kVec3fPorts, std::size(kVec3fPorts), &RunVec3f,
             &ValidateVec3f);

const rigExec::RigExecOpDesc kMatrixDesc =
    MathDesc("MatrixMath", kMatrixPorts, std::size(kMatrixPorts), &RunMatrix,
             &ValidateMatrix);

} // namespace

void
RegisterPropertyMathOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(FloatMath(), kFloatDesc);
    registrar.Add(Vec3fMath(), kVec3fDesc);
    registrar.Add(MatrixMath(), kMatrixDesc);
    registrar.Add(DoubleMath(), kDoubleDesc);
}

} // namespace rigExecOps
