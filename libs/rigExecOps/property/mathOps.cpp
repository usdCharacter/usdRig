#include "rigExecOps/property/mathOps.h"

#include "rigExec/api/opFields.h"
#include "rigExecOps/opIo.h"
#include "rigExecSchema/schemaNames.h"

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
      uint64_t limit = UINT64_MAX,
      const rigExec::RigExecPropertyInfo *authoring = nullptr)
{
    rigExec::RigExecFieldDesc f;
    f.name = name;
    f.type = &type;
    f.limit = limit;
    f.authoring = authoring;
    return f;
}

namespace names = rigExec::SchemaAttrText;
namespace values = rigExec::SchemaValueText;

const char *const kTyped = "Typed";

const char *const kFloatOperations[] = {
    values::add,   values::multiply, values::clamp,    values::remap,
    values::blend, values::curve,    values::quantize, values::path};
const char *const kVec3fOperations[] = {
    values::add, values::multiply, values::clamp, values::remap, values::blend};
const char *const kMatrixOperations[] = {nullptr, values::multiply, nullptr,
                                         nullptr, values::blend};
const char *const kPathComponents[] = {values::x, values::y, values::z,
                                       values::heading, values::end};
const char *const kPathMeasures[] = {values::length, values::planar};

const uint32_t kAddOp = uint32_t(RigExecPropertyOp::Add);
const uint32_t kMultiplyOp = uint32_t(RigExecPropertyOp::Multiply);
const uint32_t kClampOp = uint32_t(RigExecPropertyOp::Clamp);
const uint32_t kComponentX = uint32_t(rigExec::RigExecPathComponent::X);
const uint8_t kMeasureLength = 0;
const float kZero = 0.0f;
const float kOne = 1.0f;
const Vec3f kZero3(0.0f, 0.0f, 0.0f);
const Vec3f kOne3(1.0f, 1.0f, 1.0f);
const Mat4d kIdentity(1.0);
const Keys kNoKeys;

rigExec::RigExecPropertyInfo
Property(const char *schemaName, const void *fallback, const char *doc,
         bool uniform = false)
{
    rigExec::RigExecPropertyInfo info;
    info.schemaName = schemaName;
    info.fallback = fallback;
    info.doc = doc;
    info.uniform = uniform;
    return info;
}

template <size_t N>
rigExec::RigExecPropertyInfo
TokenProperty(const char *schemaName, const void *fallback,
              const char *const (&tokens)[N], const char *doc)
{
    rigExec::RigExecPropertyInfo info =
        Property(schemaName, fallback, doc, true);
    info.tokens = tokens;
    info.tokenCount = N;
    return info;
}

const rigExec::RigExecPropertyInfo kFloatOperation = TokenProperty(
    names::rigExecOperation, &kClampOp, kFloatOperations, nullptr);
const rigExec::RigExecPropertyInfo kVec3fOperation =
    TokenProperty(names::rigExecOperation, &kAddOp, kVec3fOperations, nullptr);
const rigExec::RigExecPropertyInfo kMatrixOperation = TokenProperty(
    names::rigExecOperation, &kMultiplyOp, kMatrixOperations, nullptr);

const rigExec::RigExecPropertyInfo kComponentProperty = TokenProperty(
    names::rigExecPathComponent, &kComponentX, kPathComponents,
    R"(What the path operation returns. x, y and z are the sampled
point plus rigExec:pathOffset carried by the heading. heading is the
planar tangent's yaw about +Y in degrees (the rotation taking local
+Z to the tangent) plus rigExec:pathHeadingOffset, wrapped to
(-180, 180]. end is inputs:value plus the curve's whole length.)");

const rigExec::RigExecPropertyInfo kMeasureProperty =
    TokenProperty(names::rigExecPathMeasure, &kMeasureLength, kPathMeasures,
                  R"(How the path operation measures distance along the curve:
its arc length, or (planar) the arc length of its projection on the
XZ plane -- ground distance, with the points' y carried as a height,
so a walk's stride does not change over terrain.)");

const rigExec::RigExecPropertyInfo kOffsetProperty = [] {
    rigExec::RigExecPropertyInfo info = Property(
        names::rigExecPathOffset, &kZero,
        R"(A local (x, z) offset from the sampled point, rotated by the
heading, for x and z: where a frame that travels along the path
carries a point (a foot's landing spot, say).)",
        true);
    info.authoredType = "float2";
    return info;
}();

const rigExec::RigExecPropertyInfo kHeadingProperty =
    Property(names::rigExecPathHeadingOffset, &kZero,
             R"(Degrees added to the tangent's heading, for heading and for
the rotation of rigExec:pathOffset.)",
             true);

const rigExec::RigExecPropertyInfo kPath = [] {
    rigExec::RigExecPropertyInfo info =
        Property(names::rigExecPath, nullptr,
                 R"(The path operation's curve: exactly one UsdGeomBasisCurves
prim holding one curve, linear or cubic (bezier, bspline or
catmullRom), nonperiodic or pinned. Its points are read as authored,
in the rig's space, and are an input a baked program can set.)");
    info.binding = rigExec::RigExecPropertyBinding::Connection;
    return info;
}();

const rigExec::RigExecPropertyInfo kFloatValue =
    Property(names::inputsValue, &kZero, nullptr);
const rigExec::RigExecPropertyInfo kFloatMin =
    Property(names::inputsMin, &kZero, nullptr);
const rigExec::RigExecPropertyInfo kFloatMax =
    Property(names::inputsMax, &kOne, nullptr);
const rigExec::RigExecPropertyInfo kKeys =
    Property(names::inputsKeys, &kNoKeys,
             R"((input, output) keys for the curve operation, strictly
increasing in input. One key is a constant output.)");
const rigExec::RigExecPropertyInfo kTangents =
    Property(names::inputsTangents, &kNoKeys,
             R"(Optional (in slope, out slope) per key for the curve
operation. Empty is piecewise linear, extrapolated along the end
segments; authored, the curve is a cubic Hermite through the keys,
extrapolated along the first in slope and the last out slope.)");

const rigExec::RigExecPropertyInfo kVec3fValue =
    Property(names::inputsValue, &kZero3, nullptr);
const rigExec::RigExecPropertyInfo kVec3fMin =
    Property(names::inputsMin, &kZero3, nullptr);
const rigExec::RigExecPropertyInfo kVec3fMax =
    Property(names::inputsMax, &kOne3, nullptr);
const rigExec::RigExecPropertyInfo kMatrixValue =
    Property(names::inputsValue, &kIdentity, nullptr);

const rigExec::RigExecSchemaInfo kFloatSchema{
    rigExec::SchemaTypeName::RigExecFloatMathMover,
    R"(Statically typed add, multiply, clamp, remap, blend, curve,
quantize, or path over one exact float property target, or a double one
such as a control's avar, which the mover computes in float.

The revision is r = op(incoming), then the common MoverAPI envelope mixes
it back over the incoming value. Zero is a pass-through and one applies
the operation outright -- the same rule every mover follows.

add and multiply apply inputs:value, clamp reads inputs:min and
inputs:max, and blend returns inputs:value, so the envelope mixes toward
it.

remap normalizes from [inputs:min, inputs:max] to [0, 1] and does NOT
clamp; compose a clamp mover after it to bound the result.

curve maps the incoming value through inputs:keys, piecewise linear
between keys (a cubic Hermite when inputs:tangents is authored) and
extrapolated linearly past the first and last key.

quantize rounds the incoming value down to inputs:min + k * inputs:value
for the largest integer k that does not exceed it: a stair of step
inputs:value through inputs:min. A step that is not positive passes the
value through.

path samples the curve rigExec:path names at arc length (incoming -
inputs:value), clamped to the curve, and returns rigExec:pathComponent
of it. inputs:value is the distance the curve's first point stands at,
so a moving window of points keeps one distance clock.)",
    kTyped, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecSchemaInfo kVec3fSchema{
    rigExec::SchemaTypeName::RigExecVec3fMathMover,
    R"(Statically typed add, multiply, clamp, remap, or blend over an
exact float3, vector3f, point3f, normal3f or color3f property target.

Every operation is component-wise, which is why the bounds are float3
rather than float. Remap and common MoverAPI-envelope semantics match
RigExecFloatMathMover.)",
    kTyped, rigExec::RigExecSchemaMode::Generated};

const rigExec::RigExecSchemaInfo kMatrixSchema{
    rigExec::SchemaTypeName::RigExecMatrixMathMover,
    R"(Statically typed multiply or blend over an exact matrix4d
property target. Distinct from the point-moving
RigExecMatrixMover.

multiply post-multiplies (inputs:value applied AFTER the incoming
matrix, in GfMatrix4d's row-vector convention); blend replaces. The common
MoverAPI envelope mixes component-wise and is exact at both endpoints.)",
    kTyped, rigExec::RigExecSchemaMode::Generated};

/// One math op's fields, in saved order (MathField). Every op has the path
/// fields; only the scalar ops author them.
struct MathFieldList {
    rigExec::RigExecFieldDesc fields[kPathHeadingOffset + 1];
};

MathFieldList
MathFields(const rigExec::RigExecPropertyInfo &operation, bool path)
{
    const auto authored = [&](const rigExec::RigExecPropertyInfo &info) {
        return path ? &info : nullptr;
    };
    MathFieldList list;
    rigExec::RigExecFieldDesc *out = list.fields;
    out[kOp] = Field("op", rigExecStd::UIntType(),
                     uint64_t(RigExecPropertyOp::Path), &operation);
    out[kPathValid] = Field("pathValid", rigExecStd::BoolType(), 1);
    out[kPathBasis] = Field("pathBasis", rigExecStd::UIntType(),
                            uint64_t(rigExec::RigExecPathBasis::CatmullRom));
    out[kPathPinned] = Field("pathPinned", rigExecStd::BoolType(), 1);
    out[kPathPlanar] = Field("pathPlanar", rigExecStd::BoolType(), 1,
                             authored(kMeasureProperty));
    out[kPathComponent] = Field("pathComponent", rigExecStd::UIntType(),
                                uint64_t(rigExec::RigExecPathComponent::End),
                                authored(kComponentProperty));
    out[kPathOffsetX] = Field("pathOffsetX", rigExecStd::FloatType(),
                              UINT64_MAX, authored(kOffsetProperty));
    out[kPathOffsetZ] = Field("pathOffsetZ", rigExecStd::FloatType(),
                              UINT64_MAX, authored(kOffsetProperty));
    out[kPathHeadingOffset] =
        Field("pathHeadingOffset", rigExecStd::FloatType(), UINT64_MAX,
              authored(kHeadingProperty));
    return list;
}

const MathFieldList kScalarFields = MathFields(kFloatOperation, true);
const MathFieldList kVec3fFields = MathFields(kVec3fOperation, false);
const MathFieldList kMatrixFields = MathFields(kMatrixOperation, false);

rigExec::RigExecOpDesc
MathDesc(const char *name, const rigExec::RigExecPortDesc *ports,
         size_t portCount, const rigExec::RigExecFieldDesc *fields,
         const rigExec::RigExecSchemaInfo &schema,
         rigExec::RigExecOpBatchFn runBatch,
         bool (*validate)(const rigExec::RigExecOpCheck &, std::string *))
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.payloadFields = fields;
    d.payloadFieldCount = std::size(kScalarFields.fields);
    d.schema = &schema;
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

rigExec::RigExecPortDesc
Authored(const RigExecPortHandle &port, const rigExec::RigExecTypeHandle &type,
         const char *name, const rigExec::RigExecPropertyInfo &authoring)
{
    rigExec::RigExecPortDesc d;
    d.port = &port;
    d.type = &type;
    d.flags = kOptional;
    d.name = name;
    d.authoring = &authoring;
    return d;
}

const rigExec::RigExecPortDesc kFloatPorts[] = {
    {&FloatIn(), &rigExecStd::FloatType(), kRigExecPortInput, "in"},
    Authored(FloatValue(), rigExecStd::FloatType(), "value", kFloatValue),
    Authored(FloatMin(), rigExecStd::FloatType(), "min", kFloatMin),
    Authored(FloatMax(), rigExecStd::FloatType(), "max", kFloatMax),
    {&FloatWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    Authored(FloatPathPoints(), rigExecStd::Vec3fArray(), "pathPoints", kPath),
    {&FloatOut(), &rigExecStd::FloatType(), kRigExecPortOutput, "out"},
    {&FloatEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    Authored(FloatKeys(), rigExecStd::Vec2fArray(), "keys", kKeys),
    Authored(FloatTangents(), rigExecStd::Vec2fArray(), "tangents", kTangents),
};

const rigExec::RigExecPortDesc kDoublePorts[] = {
    {&DoubleIn(), &rigExecStd::DoubleType(), kRigExecPortInput, "in"},
    Authored(DoubleValue(), rigExecStd::FloatType(), "value", kFloatValue),
    Authored(DoubleMin(), rigExecStd::FloatType(), "min", kFloatMin),
    Authored(DoubleMax(), rigExecStd::FloatType(), "max", kFloatMax),
    {&DoubleWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    Authored(DoublePathPoints(), rigExecStd::Vec3fArray(), "pathPoints", kPath),
    {&DoubleOut(), &rigExecStd::DoubleType(), kRigExecPortOutput, "out"},
    {&DoubleEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    Authored(DoubleKeys(), rigExecStd::Vec2fArray(), "keys", kKeys),
    Authored(DoubleTangents(), rigExecStd::Vec2fArray(), "tangents", kTangents),
};

const rigExec::RigExecPortDesc kVec3fPorts[] = {
    {&Vec3fIn(), &rigExecStd::Vec3fType(), kRigExecPortInput, "in"},
    Authored(Vec3fValue(), rigExecStd::Vec3fType(), "value", kVec3fValue),
    Authored(Vec3fMin(), rigExecStd::Vec3fType(), "min", kVec3fMin),
    Authored(Vec3fMax(), rigExecStd::Vec3fType(), "max", kVec3fMax),
    {&Vec3fWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&Vec3fOut(), &rigExecStd::Vec3fType(), kRigExecPortOutput, "out"},
    {&Vec3fEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
};

const rigExec::RigExecPortDesc kMatrixPorts[] = {
    {&MatrixIn(), &rigExecStd::Mat4dType(), kRigExecPortInput, "in"},
    Authored(MatrixValue(), rigExecStd::Mat4dType(), "value", kMatrixValue),
    {&MatrixWeight(), &rigExecStd::FloatType(), kRigExecPortInput, "weight"},
    {&MatrixOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
    {&MatrixEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
};

const rigExec::RigExecOpDesc kFloatDesc =
    MathDesc("FloatMath", kFloatPorts, std::size(kFloatPorts),
             kScalarFields.fields, kFloatSchema, &RunFloat, &ValidateFloat);

const rigExec::RigExecOpDesc kDoubleDesc =
    MathDesc("DoubleMath", kDoublePorts, std::size(kDoublePorts),
             kScalarFields.fields, kFloatSchema, &RunDouble, &ValidateDouble);

const rigExec::RigExecOpDesc kVec3fDesc =
    MathDesc("Vec3fMath", kVec3fPorts, std::size(kVec3fPorts),
             kVec3fFields.fields, kVec3fSchema, &RunVec3f, &ValidateVec3f);

const rigExec::RigExecOpDesc kMatrixDesc =
    MathDesc("MatrixMath", kMatrixPorts, std::size(kMatrixPorts),
             kMatrixFields.fields, kMatrixSchema, &RunMatrix, &ValidateMatrix);

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
