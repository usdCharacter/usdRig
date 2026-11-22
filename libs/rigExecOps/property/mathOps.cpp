#include "rigExecOps/property/mathOps.h"

#include <iterator>
#include <memory>
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

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::Vec2f;
using rigExecStd::Vec3f;

template <class T>
const T *
In(const RigExecOpIo &io, const RigExecPortHandle &port)
{
    return static_cast<const T *>(io.inputs[port.Index()]);
}

template <class T>
T
InOr(const RigExecOpIo &io, const RigExecPortHandle &port, const T &fallback)
{
    const T *value = In<T>(io, port);
    return value ? *value : fallback;
}

template <class T>
T &
Out(const RigExecOpIo &io, const RigExecPortHandle &port)
{
    return *static_cast<T *>(io.outputs[port.Index()]);
}

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

bool
Fail(std::string *error, const char *op, const std::string &why)
{
    if (error)
        *error = std::string(op) + ": " + why;
    return false;
}

bool
RunFloat(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const MathPayload &p = *static_cast<const MathPayload *>(payload);
    rigExec::RigExecPropertyMathKernelParams<float, Vec2f> params;
    params.op = p.op;
    params.value = InOr(io, FloatValue(), 0.0f);
    params.min = InOr(io, FloatMin(), 0.0f);
    params.max = InOr(io, FloatMax(), 0.0f);
    params.weight = *In<float>(io, FloatWeight());
    if (p.op == RigExecPropertyOp::Curve) {
        if (p.keys.empty() ||
            !rigExec::RigExecValidateLinearKeysKernel(p.keys.data(),
                                                      p.keys.size()) ||
            (!p.tangents.empty() && p.tangents.size() != p.keys.size()))
            return Fail(error, "FloatMath",
                        "curve needs finite, strictly increasing keys and "
                        "no tangents or one per key");
        params.keys = p.keys.data();
        params.keyCount = p.keys.size();
        params.tangents = p.tangents.empty() ? nullptr : p.tangents.data();
        params.tangentCount = p.tangents.size();
    }
    if (p.op == RigExecPropertyOp::Path) {
        const auto *points =
            In<rigExecStd::RigExecArray<Vec3f>>(io, FloatPathPoints());
        params.path.points =
            points && !points->Empty() ? points->Data()[0]._data : nullptr;
        params.path.pointCount = points ? points->Size() : 0;
        params.path.basis = p.path.basis;
        params.path.pinned = p.path.pinned;
        params.path.planar = p.path.planar;
        params.pathSettings = p.path;
        if (!p.path.valid || !rigExec::RigExecPathCurveValid(params.path))
            return Fail(error, "FloatMath", "path has no valid curve");
    }
    Out<float>(io, FloatOut()) =
        rigExec::RigExecApplyFloatMathKernel(*In<float>(io, FloatIn()), params);
    return true;
}

bool
RunVec3f(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const MathPayload &p = *static_cast<const MathPayload *>(payload);
    switch (p.op) {
    case RigExecPropertyOp::Curve:
    case RigExecPropertyOp::Quantize:
    case RigExecPropertyOp::Path:
        return Fail(error, "Vec3fMath",
                    std::string(OpName(p.op)) + " is float-only");
    default:
        break;
    }
    const Vec3f zero(0.0f, 0.0f, 0.0f);
    rigExec::RigExecPropertyMathKernelParams<Vec3f, Vec2f> params;
    params.op = p.op;
    params.value = InOr(io, Vec3fValue(), zero);
    params.min = InOr(io, Vec3fMin(), zero);
    params.max = InOr(io, Vec3fMax(), zero);
    params.weight = *In<float>(io, Vec3fWeight());
    Out<Vec3f>(io, Vec3fOut()) =
        rigExec::RigExecApplyVec3fMathKernel(*In<Vec3f>(io, Vec3fIn()), params);
    return true;
}

bool
RunMatrix(const void *payload, void *, const RigExecOpIo &io,
          std::string *error)
{
    const MathPayload &p = *static_cast<const MathPayload *>(payload);
    const Mat4d *value = In<Mat4d>(io, MatrixValue());
    const Mat4d identity(1.0);
    if (!rigExec::RigExecApplyMatrixMathKernel(
            *In<Mat4d>(io, MatrixIn()), p.op, value ? *value : identity,
            *In<float>(io, MatrixWeight()), &Out<Mat4d>(io, MatrixOut())))
        return Fail(error, "MatrixMath",
                    std::string(OpName(p.op)) + " has no matrix meaning");
    return true;
}

void
ConstructPayload(void *dst)
{
    std::uninitialized_value_construct_n(static_cast<MathPayload *>(dst), 1);
}

void
CopyPayload(void *dst, const void *src)
{
    std::uninitialized_copy_n(static_cast<const MathPayload *>(src), 1,
                              static_cast<MathPayload *>(dst));
}

void
DestroyPayload(void *dst)
{
    std::destroy_at(static_cast<MathPayload *>(dst));
}

rigExec::RigExecOpDesc
MathDesc(const char *name, const rigExec::RigExecPortDesc *ports,
         size_t portCount, rigExec::RigExecOpRunFn run)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.run = run;
    d.capabilities = rigExec::kRigExecOpBakeable;
    d.payload.size = sizeof(MathPayload);
    d.payload.align = alignof(MathPayload);
    d.payload.construct = &ConstructPayload;
    d.payload.copy = &CopyPayload;
    d.payload.destroy = &DestroyPayload;
    return d;
}

void
Register(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &f = rigExecStd::FloatType();
    const auto &v = rigExecStd::Vec3fType();
    const auto &m = rigExecStd::Mat4dType();

    static const rigExec::RigExecPortDesc floatPorts[] = {
        {&FloatIn(), &f, kRigExecPortInput, "in"},
        {&FloatValue(), &f, kOptional, "value"},
        {&FloatMin(), &f, kOptional, "min"},
        {&FloatMax(), &f, kOptional, "max"},
        {&FloatWeight(), &f, kRigExecPortInput, "weight"},
        {&FloatPathPoints(), &rigExecStd::Vec3fArray(), kOptional,
         "pathPoints"},
        {&FloatOut(), &f, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc vec3fPorts[] = {
        {&Vec3fIn(), &v, kRigExecPortInput, "in"},
        {&Vec3fValue(), &v, kOptional, "value"},
        {&Vec3fMin(), &v, kOptional, "min"},
        {&Vec3fMax(), &v, kOptional, "max"},
        {&Vec3fWeight(), &f, kRigExecPortInput, "weight"},
        {&Vec3fOut(), &v, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc matrixPorts[] = {
        {&MatrixIn(), &m, kRigExecPortInput, "in"},
        {&MatrixValue(), &m, kOptional, "value"},
        {&MatrixWeight(), &f, kRigExecPortInput, "weight"},
        {&MatrixOut(), &m, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc floatDesc =
        MathDesc("FloatMath", floatPorts, std::size(floatPorts), &RunFloat);
    static const rigExec::RigExecOpDesc vec3fDesc =
        MathDesc("Vec3fMath", vec3fPorts, std::size(vec3fPorts), &RunVec3f);
    static const rigExec::RigExecOpDesc matrixDesc =
        MathDesc("MatrixMath", matrixPorts, std::size(matrixPorts), &RunMatrix);
    registrar.Add(FloatMath(), floatDesc);
    registrar.Add(Vec3fMath(), vec3fDesc);
    registrar.Add(MatrixMath(), matrixDesc);
}

const rigExec::RigExecOpLibraryDesc kLibrary = [] {
    rigExec::RigExecOpLibraryDesc d;
    d.name = "rigExecOps";
    d.registerOps = &Register;
    return d;
}();

} // namespace
} // namespace rigExecOps

RIGEXEC_OP_LIBRARY_ENTRY(rigExecOps)
{
    return &rigExecOps::kLibrary;
}
