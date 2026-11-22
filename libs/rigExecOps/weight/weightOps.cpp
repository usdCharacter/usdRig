#include "rigExecOps/weight/weightOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/spatialAccel.h"
#include "rigExecMath/weightFieldKernel.h"

#include <cmath>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SphereWeight)
RIGEXEC_DEFINE_OP_HANDLE(PlaneWeight)
RIGEXEC_DEFINE_OP_HANDLE(CurveWeight)

#define RIGEXEC_DEFINE_WEIGHT_FIELD_PORTS(Op)                                  \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##Points)                                     \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##Placement)                                  \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##Strength)                                   \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##Invert)                                     \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##FalloffMin)                                 \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##FalloffMax)                                 \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##FalloffSamples)                             \
    RIGEXEC_DEFINE_PORT_HANDLE(Op##Out)

RIGEXEC_DEFINE_WEIGHT_FIELD_PORTS(SphereWeight)
RIGEXEC_DEFINE_PORT_HANDLE(SphereWeightScale)
RIGEXEC_DEFINE_PORT_HANDLE(SphereWeightPositiveScale)
RIGEXEC_DEFINE_PORT_HANDLE(SphereWeightNegativeScale)

RIGEXEC_DEFINE_WEIGHT_FIELD_PORTS(PlaneWeight)
RIGEXEC_DEFINE_PORT_HANDLE(PlaneWeightExtentU)
RIGEXEC_DEFINE_PORT_HANDLE(PlaneWeightExtentV)

RIGEXEC_DEFINE_WEIGHT_FIELD_PORTS(CurveWeight)
RIGEXEC_DEFINE_PORT_HANDLE(CurveWeightScale)
RIGEXEC_DEFINE_PORT_HANDLE(CurveWeightCurve)

#undef RIGEXEC_DEFINE_WEIGHT_FIELD_PORTS

namespace {

using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Weights = rigExecStd::RigExecArray<float>;
using namespace opIo;

/// One op's shared ports.
struct FieldPorts {
    const RigExecPortHandle &points, &placement, &strength, &invert,
        &falloffMin, &falloffMax, &falloffSamples, &out;
};

#define RIGEXEC_FIELD_PORTS(Op)                                                \
    FieldPorts                                                                 \
    {                                                                          \
        Op##Points(), Op##Placement(), Op##Strength(), Op##Invert(),           \
            Op##FalloffMin(), Op##FalloffMax(), Op##FalloffSamples(),          \
            Op##Out()                                                          \
    }

/// What every field reads before its kernel.
struct Field {
    const Points *points = nullptr;
    Mat4d worldToLocal;
    rigExec::RigExecFalloffKernelParams falloff;
};

bool
BeginField(const WeightFieldPayload &p, const RigExecOpIo &io,
           const FieldPorts &ports, const char *op, std::string *error,
           Field *field)
{
    field->points = In<Points>(io, ports.points);
    const Mat4d rigid = In<Mat4d>(io, ports.placement)->RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12)
        return Fail(error, op, "degenerate placement");
    field->worldToLocal = rigid.GetInverse();
    auto &f = field->falloff;
    f.falloffMin = InOr(io, ports.falloffMin, 0.0f);
    f.falloffMax = InOr(io, ports.falloffMax, 1.0f);
    f.invert = InOr(io, ports.invert, 0.0f);
    f.strength = InOr(io, ports.strength, 1.0f);
    const Weights *samples = In<Weights>(io, ports.falloffSamples);
    f.curve = samples ? samples->Data() : p.falloffCurve.data();
    f.curveCount = samples ? samples->Size() : p.falloffCurve.size();
    Out<Weights>(io, ports.out).Resize(field->points->Size());
    return true;
}

bool
Positive(const Vec3f &v)
{
    for (size_t a = 0; a < 3; ++a)
        if (!std::isfinite(v[a]) || v[a] <= 0.0f)
            return false;
    return true;
}

/// worldToLocal followed by 1 / \p scale per axis.
bool
ApplyScale(const RigExecOpIo &io, const RigExecPortHandle &port, const char *op,
           std::string *error, Mat4d *worldToLocal)
{
    const Vec3f scale = InOr(io, port, Vec3f(1.0f));
    if (!Positive(scale))
        return Fail(error, op, "scale must be finite and positive");
    Mat4d divide(1.0);
    divide.Set(1.0 / double(scale[0]), 0.0, 0.0, 0.0, 0.0,
               1.0 / double(scale[1]), 0.0, 0.0, 0.0, 0.0,
               1.0 / double(scale[2]), 0.0, 0.0, 0.0, 0.0, 1.0);
    *worldToLocal = *worldToLocal * divide;
    return true;
}

bool
FinishField(const WeightFieldPayload &p, const RigExecOpIo &io,
            const FieldPorts &ports, const char *op, std::string *error)
{
    for (float &w : Out<Weights>(io, ports.out))
        if (!rigExec::RigExecApplyWeightRangeKernel(p.clamp, &w))
            return Fail(error, op,
                        p.clamp ? "non-finite weight" : "weight out of range");
    return true;
}

bool
RunSphereWeight(const void *payload, void *, const RigExecOpIo &io,
                std::string *error)
{
    const auto &p = *static_cast<const WeightFieldPayload *>(payload);
    const FieldPorts ports = RIGEXEC_FIELD_PORTS(SphereWeight);
    Field f;
    if (!BeginField(p, io, ports, "SphereWeight", error, &f) ||
        !ApplyScale(io, SphereWeightScale(), "SphereWeight", error,
                    &f.worldToLocal))
        return false;
    const Vec3f positive = InOr(io, SphereWeightPositiveScale(), Vec3f(1.0f));
    const Vec3f negative = InOr(io, SphereWeightNegativeScale(), Vec3f(1.0f));
    if (!Positive(positive) || !Positive(negative))
        return Fail(error, "SphereWeight",
                    "signed scales must be finite and positive");
    rigExec::RigExecSphereWeightFieldKernel<Vec3f, Vec3d>(
        f.points->Data(), 0, f.points->Size(), f.worldToLocal, f.falloff,
        positive, negative, Out<Weights>(io, ports.out).Data());
    return FinishField(p, io, ports, "SphereWeight", error);
}

bool
RunPlaneWeight(const void *payload, void *, const RigExecOpIo &io,
               std::string *error)
{
    const auto &p = *static_cast<const WeightFieldPayload *>(payload);
    const FieldPorts ports = RIGEXEC_FIELD_PORTS(PlaneWeight);
    Field f;
    if (!BeginField(p, io, ports, "PlaneWeight", error, &f))
        return false;
    const float extentU = InOr(io, PlaneWeightExtentU(), 1.0f);
    const float extentV = InOr(io, PlaneWeightExtentV(), 1.0f);
    if (p.planeBounded)
        for (float extent : {extentU, extentV})
            if (!std::isfinite(extent) || extent <= 0.0f)
                return Fail(error, "PlaneWeight",
                            "extents must be finite and positive");
    rigExec::RigExecPlaneWeightFieldKernel<Vec3f, Vec3d>(
        f.points->Data(), 0, f.points->Size(), f.worldToLocal, int(p.planeAxis),
        f.falloff, p.planeBounded, extentU, extentV,
        Out<Weights>(io, ports.out).Data());
    return FinishField(p, io, ports, "PlaneWeight", error);
}

bool
RunCurveWeight(const void *payload, void *, const RigExecOpIo &io,
               std::string *error)
{
    const auto &p = *static_cast<const WeightFieldPayload *>(payload);
    const FieldPorts ports = RIGEXEC_FIELD_PORTS(CurveWeight);
    Field f;
    if (!BeginField(p, io, ports, "CurveWeight", error, &f) ||
        !ApplyScale(io, CurveWeightScale(), "CurveWeight", error,
                    &f.worldToLocal))
        return false;
    const Points &curve = *In<Points>(io, CurveWeightCurve());
    if (curve.Size() == 0)
        return Fail(error, "CurveWeight", "needs a curve");
    std::vector<Vec3f> local(curve.Size());
    for (size_t k = 0; k < curve.Size(); ++k)
        local[k] = rigExec::RigExecWeightToLocal<Vec3f, Vec3d>(f.worldToLocal,
                                                               curve[k]);
    float *const out = Out<Weights>(io, ports.out).Data();
    const size_t count = f.points->Size();
    // The same nearest segment as the scan, found faster on long curves.
    rigExec::RigExecSegmentBvh<Vec3f> bvh;
    if (local.size() - 1 >= rigExec::kRigExecBvhMinSegments &&
        count >= rigExec::kRigExecBvhMinQueries &&
        bvh.Build(local.data(), local.size())) {
        rigExec::RigExecCurveWeightFieldKernel<Vec3f, Vec3d>(
            f.points->Data(), 0, count, f.worldToLocal, f.falloff,
            [&](const Vec3f &q) {
                return bvh.QueryNearest(
                    q, local.data(),
                    [](const Vec3f &x, const Vec3f &a, const Vec3f &b) {
                        return rigExec::RigExecSegmentDistanceKernel(x, a, b);
                    });
            },
            out);
    } else {
        rigExec::RigExecCurveWeightFieldKernel<Vec3f, Vec3d>(
            f.points->Data(), 0, count, f.worldToLocal, f.falloff,
            [&](const Vec3f &q) {
                return rigExec::RigExecCurveDistanceKernel(q, local.data(),
                                                           local.size());
            },
            out);
    }
    return FinishField(p, io, ports, "CurveWeight", error);
}

} // namespace

void
RegisterWeightOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    using rigExec::RigExecPortDesc;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &floats = rigExecStd::FloatArray();
    const auto &scalar = rigExecStd::FloatType();
    const auto &vec = rigExecStd::Vec3fType();
    const auto &matrix = rigExecStd::Mat4dType();

#define RIGEXEC_FIELD_PORT_DESCS(Op)                                           \
    {&Op##Points(), &points, kRigExecPortInput, "points"},                     \
        {&Op##Placement(), &matrix, kRigExecPortInput, "placement"},           \
        {&Op##Strength(), &scalar, kOptional, "strength"},                     \
        {&Op##Invert(), &scalar, kOptional, "invert"},                         \
        {&Op##FalloffMin(), &scalar, kOptional, "falloffMin"},                 \
        {&Op##FalloffMax(), &scalar, kOptional, "falloffMax"},                 \
        {&Op##FalloffSamples(), &floats, kOptional, "falloffSamples"},         \
    {                                                                          \
        &Op##Out(), &floats, kRigExecPortOutput, "out"                         \
    }

    static const RigExecPortDesc spherePorts[] = {
        RIGEXEC_FIELD_PORT_DESCS(SphereWeight),
        {&SphereWeightScale(), &vec, kOptional, "scale"},
        {&SphereWeightPositiveScale(), &vec, kOptional, "positiveScale"},
        {&SphereWeightNegativeScale(), &vec, kOptional, "negativeScale"},
    };
    static const RigExecPortDesc planePorts[] = {
        RIGEXEC_FIELD_PORT_DESCS(PlaneWeight),
        {&PlaneWeightExtentU(), &scalar, kOptional, "extentU"},
        {&PlaneWeightExtentV(), &scalar, kOptional, "extentV"},
    };
    static const RigExecPortDesc curvePorts[] = {
        RIGEXEC_FIELD_PORT_DESCS(CurveWeight),
        {&CurveWeightScale(), &vec, kOptional, "scale"},
        {&CurveWeightCurve(), &points, kRigExecPortInput, "curve"},
    };
#undef RIGEXEC_FIELD_PORT_DESCS

    const auto desc = [](const char *name, const RigExecPortDesc *ports,
                         size_t count, rigExec::RigExecOpRunFn run) {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = name;
        d.ports = ports;
        d.portCount = count;
        d.run = run;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<WeightFieldPayload>();
        return d;
    };
    static const rigExec::RigExecOpDesc sphere = desc(
        "SphereWeight", spherePorts, std::size(spherePorts), &RunSphereWeight);
    static const rigExec::RigExecOpDesc plane =
        desc("PlaneWeight", planePorts, std::size(planePorts), &RunPlaneWeight);
    static const rigExec::RigExecOpDesc curve =
        desc("CurveWeight", curvePorts, std::size(curvePorts), &RunCurveWeight);
    registrar.Add(SphereWeight(), sphere);
    registrar.Add(PlaneWeight(), plane);
    registrar.Add(CurveWeight(), curve);
}

} // namespace rigExecOps
