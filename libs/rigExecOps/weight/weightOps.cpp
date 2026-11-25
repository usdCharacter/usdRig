#include "rigExecOps/weight/weightOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/weights.h"
#include "rigExecMath/spatialAccel.h"
#include "rigExecMath/weightFieldKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <cmath>
#include <cstring>
#include <iterator>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SphereWeight)
RIGEXEC_DEFINE_OP_HANDLE(PlaneWeight)
RIGEXEC_DEFINE_OP_HANDLE(CurveWeight)
RIGEXEC_DEFINE_OP_HANDLE(CombineWeights)
RIGEXEC_DEFINE_OP_HANDLE(VolumePlacement)
RIGEXEC_DEFINE_PORT_HANDLE(VolumePlacementFrame)
RIGEXEC_DEFINE_PORT_HANDLE(VolumePlacementOut)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsInputs)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsTargetCount)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsStrength)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsInvert)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsOut)
RIGEXEC_DEFINE_OP_HANDLE(StaticWeights)
RIGEXEC_DEFINE_PORT_HANDLE(StaticWeightsValues)
RIGEXEC_DEFINE_PORT_HANDLE(StaticWeightsIndices)
RIGEXEC_DEFINE_PORT_HANDLE(StaticWeightsDefaultWeight)
RIGEXEC_DEFINE_PORT_HANDLE(StaticWeightsOut)
RIGEXEC_DEFINE_OP_HANDLE(DynamicWeights)
RIGEXEC_DEFINE_PORT_HANDLE(DynamicWeightsBase)
RIGEXEC_DEFINE_PORT_HANDLE(DynamicWeightsDriver)
RIGEXEC_DEFINE_PORT_HANDLE(DynamicWeightsScale)
RIGEXEC_DEFINE_PORT_HANDLE(DynamicWeightsBias)
RIGEXEC_DEFINE_PORT_HANDLE(DynamicWeightsOut)
RIGEXEC_DEFINE_OP_HANDLE(VolumeWeights)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsFrame)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsPoints)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsSamples)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsCurve)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsStrength)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsInvert)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsFalloffMin)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsFalloffMax)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsFalloffSamples)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsScale)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsPositiveScale)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsNegativeScale)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsExtentU)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsExtentV)
RIGEXEC_DEFINE_PORT_HANDLE(VolumeWeightsOut)

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

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExec::RigExecPortHandle;
using rigExecStd::Mat4d;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using Weights = rigExecStd::RigExecArray<float>;
using namespace opIo;
using namespace fieldIo;

/// The weight field payload's declared fields, in saved order.
enum WeightField : uint32_t {
    kFalloffCount,
    kFalloffCurve,
    kClamp,
    kPlaneAxis,
    kPlaneBounded,
};
/// CombineWeights'.
enum CombineField : uint32_t { kMode, kInputs, kCombineClamp };

/// One instance's WeightFieldPayload, its falloff curve read in place.
struct WeightFieldSettings {
    rigExec::RigExecFieldView<const float> falloffCurve;
    bool clamp = false;
    WeightFieldAxis planeAxis = WeightFieldAxis::Z;
    bool planeBounded = false;
};

WeightFieldSettings
SettingsAt(const RigExecOpBatch &batch, size_t k)
{
    WeightFieldSettings p;
    p.falloffCurve = ListAt<float>(batch, kFalloffCurve, k);
    p.clamp = FlagAt(batch, kClamp, k);
    p.planeAxis = EnumAt<WeightFieldAxis>(batch, kPlaneAxis, k);
    p.planeBounded = FlagAt(batch, kPlaneBounded, k);
    return p;
}

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
BeginField(const WeightFieldSettings &p, const RigExecOpIo &io,
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
    f.curve = samples ? samples->Data() : p.falloffCurve.data;
    f.curveCount = samples ? samples->Size() : p.falloffCurve.Size();
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
FinishField(const WeightFieldSettings &p, const RigExecOpIo &io,
            const FieldPorts &ports, const char *op, std::string *error)
{
    for (float &w : Out<Weights>(io, ports.out))
        if (!rigExec::RigExecApplyWeightRangeKernel(p.clamp, &w))
            return Fail(error, op,
                        p.clamp ? "non-finite weight" : "weight out of range");
    return true;
}

bool
SphereWeightOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const WeightFieldSettings p = SettingsAt(batch, k);
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
RunSphereWeight(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return SphereWeightOne(batch, k, error);
    });
}

bool
PlaneWeightOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const WeightFieldSettings p = SettingsAt(batch, k);
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
RunPlaneWeight(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return PlaneWeightOne(batch, k, error);
    });
}

bool
CurveWeightOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const WeightFieldSettings p = SettingsAt(batch, k);
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

bool
RunCurveWeight(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return CurveWeightOne(batch, k, error);
    });
}

using rigExecRig::PointWeights;
using WeightList = InList<PointWeights>;

static_assert(int(WeightCombineMode::Overlay) ==
                      int(rigExec::RigExecWeightCombine::Overlay) &&
                  int(rigExecRig::WeightRepresentation::Sparse) ==
                      rigExec::kRigExecWeightSparse &&
                  int(rigExecRig::WeightRepresentation::Dense) ==
                      rigExec::kRigExecWeightDense,
              "weight codes follow rigExecMath");

/// \p w read in place.
rigExec::RigExecWeightPacketView
ViewOf(const PointWeights &w)
{
    rigExec::RigExecWeightPacketView view;
    view.valid = w.valid != 0;
    view.representation = int(w.representation);
    view.defaultWeight = w.defaultWeight;
    view.values = w.values.Data();
    view.valueCount = w.values.Size();
    view.indices = w.indices.Data();
    view.indexCount = w.indices.Size();
    return view;
}

/// Writes \p built to \p out as a valid packet.
void
WritePacket(const rigExec::RigExecWeightPacketBuild &built,
            rigExecRig::WeightRepresentation representation, bool clamp,
            PointWeights *out)
{
    out->values.Resize(built.values.size());
    if (!built.values.empty())
        std::memcpy(out->values.Data(), built.values.data(),
                    built.values.size() * sizeof(float));
    out->indices.Resize(built.indices.size());
    if (!built.indices.empty())
        std::memcpy(out->indices.Data(), built.indices.data(),
                    built.indices.size() * sizeof(int32_t));
    out->defaultWeight = built.defaultWeight;
    out->representation = uint32_t(representation);
    out->rangePolicy = uint32_t(clamp ? rigExecRig::WeightRange::Clamp
                                      : rigExecRig::WeightRange::Strict);
    out->valid = 1;
}

/// Op state: the combine fold's scratch and packet.
struct CombineState {
    std::vector<float> combineField;
    std::vector<rigExec::RigExecWeightPacketView> views;
    rigExec::RigExecWeightPacketBuild built;
};

void
WriteDense(const std::vector<float> &values, const CombineWeightsPayload &p,
           bool valid, PointWeights *out)
{
    out->values.Resize(values.size());
    if (!values.empty())
        std::memcpy(out->values.Data(), values.data(),
                    values.size() * sizeof(float));
    out->indices.Resize(0);
    out->defaultWeight = 0.0f;
    out->representation = uint32_t(rigExecRig::WeightRepresentation::Dense);
    out->rangePolicy = uint32_t(p.clamp ? rigExecRig::WeightRange::Clamp
                                        : rigExecRig::WeightRange::Strict);
    out->valid = valid ? 1 : 0;
}

bool
CombineWeightsOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    CombineWeightsPayload p;
    p.mode = EnumAt<WeightCombineMode>(batch, kMode, k);
    p.inputs = EnumAt<WeightCombineInputs>(batch, kInputs, k);
    p.clamp = FlagAt(batch, kCombineClamp, k);
    auto &s =
        *static_cast<CombineState *>(rigExec::RigExecBatchState(batch, k));
    const WeightList inputs(io, CombineWeightsInputs());
    const size_t targetCount =
        size_t(InOr(io, CombineWeightsTargetCount(), uint32_t(0)));
    const float strength = InOr(io, CombineWeightsStrength(), 1.0f);
    const float invert = InOr(io, CombineWeightsInvert(), 0.0f);
    PointWeights &out = Out<PointWeights>(io, CombineWeightsOut());

    if (p.inputs == WeightCombineInputs::Fields) {
        std::vector<float> &folded = s.combineField;
        if (!rigExec::RigExecCombineWeightFieldsStreamed(
                rigExec::RigExecWeightCombine(p.mode), inputs.Size(),
                targetCount,
                [&](size_t k, const float **data, size_t *size) {
                    const PointWeights &w = inputs[k];
                    if (!w.valid ||
                        w.representation !=
                            uint32_t(rigExecRig::WeightRepresentation::Dense))
                        return false;
                    *data = w.values.Data();
                    *size = w.values.Size();
                    return true;
                },
                &folded))
            return Fail(error, "CombineWeights",
                        "inputs disagree on element count");
        for (float &w : folded)
            w = (w + (1.0f - 2.0f * w) * invert) * strength;
        WriteDense(folded, p, true, &out);
        return true;
    }

    std::vector<rigExec::RigExecWeightPacketView> &views = s.views;
    views.resize(inputs.Size());
    for (size_t i = 0; i < inputs.Size(); ++i)
        views[i] = ViewOf(inputs[i]);
    if (const char *why = rigExec::RigExecCombineWeightPacketKernel(
            rigExec::RigExecWeightCombine(p.mode), p.clamp, strength, invert,
            views.size(),
            [&](size_t i) -> const rigExec::RigExecWeightPacketView & {
                return views[i];
            },
            targetCount, &s.combineField, &s.built))
        return Fail(error, "CombineWeights", why);
    WritePacket(s.built, rigExecRig::WeightRepresentation::Dense, p.clamp,
                &out);
    return true;
}

bool
CheckCurveWeight(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (CheckElements(check, CurveWeightCurve()) == 0)
        return Fail(error, "CurveWeight", "needs a curve");
    return true;
}

bool
CheckCombineWeights(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (EnumAt<WeightCombineInputs>(check.batch, kInputs, check.k) ==
            WeightCombineInputs::Packets &&
        CheckElements(check, CombineWeightsInputs()) == 0 &&
        !Fed(check, CombineWeightsTargetCount()))
        return Fail(error, "CombineWeights",
                    "needs an input or a target count for its element count");
    return true;
}

bool
RunCombineWeights(const RigExecOpBatch &batch, size_t *failed,
                  std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return CombineWeightsOne(batch, k, error);
    });
}

bool
RunVolumePlacement(const RigExecOpBatch &batch, size_t *failed,
                   std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
        const rigExecRig::PointFrame &frame =
            *In<rigExecRig::PointFrame>(io, VolumePlacementFrame());
        const auto placed = rigExec::RigExecVolumePlacement(
            frame.points, (frame.flags & rigExecRig::kPointFrameValid) != 0,
            (frame.flags & rigExecRig::kPointFrameDegenerate) != 0,
            [](const rigExecRig::PointLandmarks &rest,
               const rigExecRig::PointLandmarks &pose, Mat4d *matrix) {
                return rigExec::RrDualQuatPointsToMatrix(rest, pose, matrix);
            },
            &Out<Mat4d>(io, VolumePlacementOut()));
        if (placed == rigExec::RigExecVolumePlacementResult::NotFinite)
            return Fail(error, "VolumePlacement",
                        "a landmark of the volume's frame is not finite");
        return true;
    });
}

/// Op state: a painted packet's sort memo and packet.
struct StaticState {
    rigExec::RigExecStaticWeightScratch scratch;
    rigExec::RigExecWeightPacketBuild built;
};

/// A weight packet op's declared fields, in saved order.
enum PacketField : uint32_t { kRepresentation, kPacketClamp };

bool
StaticWeightsOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto representation =
        EnumAt<rigExecRig::WeightRepresentation>(batch, kRepresentation, k);
    const bool clamp = FlagAt(batch, kPacketClamp, k);
    auto &s = *static_cast<StaticState *>(rigExec::RigExecBatchState(batch, k));
    const Weights &values = *In<Weights>(io, StaticWeightsValues());
    const auto *indices =
        In<rigExecStd::RigExecArray<int32_t>>(io, StaticWeightsIndices());
    if (const char *why = rigExec::RigExecStaticWeightPacketKernel(
            int(representation), clamp,
            InOr(io, StaticWeightsDefaultWeight(), 0.0f), values.Data(),
            values.Size(), indices ? indices->Data() : nullptr,
            indices ? indices->Size() : 0, &s.scratch, &s.built))
        return Fail(error, "StaticWeights", why);
    WritePacket(s.built, representation, clamp,
                &Out<PointWeights>(io, StaticWeightsOut()));
    return true;
}

bool
RunStaticWeights(const RigExecOpBatch &batch, size_t *failed,
                 std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return StaticWeightsOne(batch, k, error);
    });
}

bool
DynamicWeightsOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto representation =
        EnumAt<rigExecRig::WeightRepresentation>(batch, kRepresentation, k);
    const bool clamp = FlagAt(batch, kPacketClamp, k);
    auto &built = *static_cast<rigExec::RigExecWeightPacketBuild *>(
        rigExec::RigExecBatchState(batch, k));
    const PointWeights *base = In<PointWeights>(io, DynamicWeightsBase());
    const rigExec::RigExecWeightPacketView view =
        base ? ViewOf(*base) : rigExec::RigExecWeightPacketView();
    if (const char *why = rigExec::RigExecDynamicWeightPacketKernel(
            int(representation), clamp, InOr(io, DynamicWeightsDriver(), 1.0f),
            InOr(io, DynamicWeightsScale(), 1.0f),
            InOr(io, DynamicWeightsBias(), 0.0f), base ? &view : nullptr,
            &built))
        return Fail(error, "DynamicWeights", why);
    WritePacket(built, representation, clamp,
                &Out<PointWeights>(io, DynamicWeightsOut()));
    return true;
}

bool
RunDynamicWeights(const RigExecOpBatch &batch, size_t *failed,
                  std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return DynamicWeightsOne(batch, k, error);
    });
}

/// The vector and matrix types rigExecEvalCore's field bodies run over.
struct VolumeMath {
    using Vec3f = rigExecStd::Vec3f;
    using Vec3d = rigExecStd::Vec3d;
    using Mat4 = rigExecStd::Mat4d;
    static Vec3d ToDouble(const Vec3f &p)
    {
        return {p[0], p[1], p[2]};
    }
    static Vec3f ToFloat(const Vec3d &p)
    {
        return {float(p[0]), float(p[1]), float(p[2])};
    }
    static float Dot(const Vec3f &a, const Vec3f &b)
    {
        return rigExec::RrDot(a, b);
    }
};

/// VolumeWeights' declared fields, in saved order.
enum VolumeField : uint32_t {
    kShape,
    kVolumeClamp,
    kVolumePlaneAxis,
    kVolumePlaneBounded
};

/// Op state: the field, and a curve carried into local space.
struct VolumeState {
    std::vector<float> weights;
    std::vector<Vec3f> localCurve;
};

bool
VolumeWeightsOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    namespace detail = rigExec::eval::weights_detail;
    using View = rigExec::eval::WeightPointViewT<VolumeMath>;
    const char *const op = "VolumeWeights";
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const auto shape = EnumAt<VolumeWeightShape>(batch, kShape, k);
    const bool clamp = FlagAt(batch, kVolumeClamp, k);
    auto &s = *static_cast<VolumeState *>(rigExec::RigExecBatchState(batch, k));
    const rigExecRig::PointFrame &frame =
        *In<rigExecRig::PointFrame>(io, VolumeWeightsFrame());
    Mat4d worldToLocal;
    if (!rigExec::RigExecRigidWorldToLocalKernel(
            frame.points, (frame.flags & rigExecRig::kPointFrameValid) != 0,
            (frame.flags & rigExecRig::kPointFrameDegenerate) != 0,
            [](const rigExecRig::PointLandmarks &rest,
               const rigExecRig::PointLandmarks &pose, Mat4d *matrix) {
                return rigExec::RrDualQuatPointsToMatrix(rest, pose, matrix);
            },
            [](const Mat4d &m) { return m.RemoveScaleShear(); }, &worldToLocal))
        return Fail(error, op,
                    "the volume's frame does not place it: it is not valid, "
                    "is degenerate or is not invertible");
    const Vec3f scale = InOr(io, VolumeWeightsScale(), Vec3f(1.0f));
    if (shape != VolumeWeightShape::Plane && !Positive(scale))
        return Fail(error, op, "scale must be finite and positive");
    const Points &target = *In<Points>(io, VolumeWeightsPoints());
    const Points *samples = In<Points>(io, VolumeWeightsSamples());
    const size_t sampled = samples ? samples->Size() : 0;
    if (!target.Size())
        return Fail(error, op, "the target has no points");
    if (sampled && sampled != target.Size())
        return Fail(error, op, "sampled point count does not match the target");
    const View points = sampled ? View{samples->Data(), sampled}
                                : View{target.Data(), target.Size()};
    rigExec::eval::WeightFalloffFacts params;
    params.falloffMin = InOr(io, VolumeWeightsFalloffMin(), 0.0f);
    params.falloffMax = InOr(io, VolumeWeightsFalloffMax(), 1.0f);
    params.invert = InOr(io, VolumeWeightsInvert(), 0.0f);
    params.strength = InOr(io, VolumeWeightsStrength(), 1.0f);
    if (const Weights *curve = In<Weights>(io, VolumeWeightsFalloffSamples())) {
        params.curve = curve->Data();
        params.curveCount = curve->Size();
    }
    if (shape == VolumeWeightShape::Plane) {
        const auto axis = EnumAt<WeightFieldAxis>(batch, kVolumePlaneAxis, k);
        const bool bounded = FlagAt(batch, kVolumePlaneBounded, k);
        const float extentU = InOr(io, VolumeWeightsExtentU(), 1.0f);
        const float extentV = InOr(io, VolumeWeightsExtentV(), 1.0f);
        if (bounded)
            for (float extent : {extentU, extentV})
                if (!std::isfinite(extent) || extent <= 0.0f)
                    return Fail(error, op,
                                "extents must be finite and positive");
        detail::PlaneWeightField<VolumeMath>(
            points, worldToLocal, int(axis), params, &s.weights,
            bounded ? extentU : 1.0f, bounded ? extentV : 1.0f, bounded,
            nullptr);
    } else if (shape == VolumeWeightShape::Sphere) {
        const Vec3f positive =
            InOr(io, VolumeWeightsPositiveScale(), Vec3f(1.0f));
        const Vec3f negative =
            InOr(io, VolumeWeightsNegativeScale(), Vec3f(1.0f));
        if (!Positive(positive) || !Positive(negative))
            return Fail(error, op, "signed scales must be finite and positive");
        detail::SphereWeightField<VolumeMath>(
            points, detail::ApplyAxisScales<VolumeMath>(worldToLocal, scale),
            params, &s.weights, positive, negative, nullptr);
    } else {
        const Points *curve = In<Points>(io, VolumeWeightsCurve());
        if (!curve || !curve->Size())
            return Fail(error, op, "needs a curve");
        detail::CurveWeightField<VolumeMath>(
            points, View{curve->Data(), curve->Size()},
            detail::ApplyAxisScales<VolumeMath>(worldToLocal, scale), params,
            &s.weights, &s.localCurve, nullptr);
    }
    for (float &weight : s.weights)
        if (!rigExec::RigExecApplyWeightRangeKernel(clamp, &weight))
            return Fail(error, op, rigExec::RigExecWeightRangeFailure(clamp));
    PointWeights &out = Out<PointWeights>(io, VolumeWeightsOut());
    out.values.Resize(s.weights.size());
    if (!s.weights.empty())
        std::memcpy(out.values.Data(), s.weights.data(),
                    s.weights.size() * sizeof(float));
    out.indices.Resize(0);
    out.defaultWeight = 0.0f;
    out.representation = uint32_t(rigExecRig::WeightRepresentation::Dense);
    out.rangePolicy = uint32_t(clamp ? rigExecRig::WeightRange::Clamp
                                     : rigExecRig::WeightRange::Strict);
    out.valid = 1;
    return true;
}

bool
RunVolumeWeights(const RigExecOpBatch &batch, size_t *failed,
                 std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return VolumeWeightsOne(batch, k, error);
    });
}

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
using rigExec::RigExecPortDesc;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

#define RIGEXEC_FIELD_PORT_DESCS(Op)                                           \
    {&Op##Points(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},   \
        {&Op##Placement(), &rigExecStd::Mat4dType(), kRigExecPortInput,        \
         "placement"},                                                         \
        {&Op##Strength(), &rigExecStd::FloatType(), kOptional, "strength"},    \
        {&Op##Invert(), &rigExecStd::FloatType(), kOptional, "invert"},        \
        {&Op##FalloffMin(), &rigExecStd::FloatType(), kOptional,               \
         "falloffMin"},                                                        \
        {&Op##FalloffMax(), &rigExecStd::FloatType(), kOptional,               \
         "falloffMax"},                                                        \
        {&Op##FalloffSamples(), &rigExecStd::FloatArray(), kOptional,          \
         "falloffSamples"},                                                    \
    {                                                                          \
        &Op##Out(), &rigExecStd::FloatArray(), kRigExecPortOutput, "out",      \
            rigExec::RigExecOutputBound::Input, &Op##Points()                  \
    }

const RigExecPortDesc kSpherePorts[] = {
    RIGEXEC_FIELD_PORT_DESCS(SphereWeight),
    {&SphereWeightScale(), &rigExecStd::Vec3fType(), kOptional, "scale"},
    {&SphereWeightPositiveScale(), &rigExecStd::Vec3fType(), kOptional,
     "positiveScale"},
    {&SphereWeightNegativeScale(), &rigExecStd::Vec3fType(), kOptional,
     "negativeScale"},
};
const RigExecPortDesc kPlanePorts[] = {
    RIGEXEC_FIELD_PORT_DESCS(PlaneWeight),
    {&PlaneWeightExtentU(), &rigExecStd::FloatType(), kOptional, "extentU"},
    {&PlaneWeightExtentV(), &rigExecStd::FloatType(), kOptional, "extentV"},
};
const RigExecPortDesc kCurvePorts[] = {
    RIGEXEC_FIELD_PORT_DESCS(CurveWeight),
    {&CurveWeightScale(), &rigExecStd::Vec3fType(), kOptional, "scale"},
    {&CurveWeightCurve(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "curve"},
};
#undef RIGEXEC_FIELD_PORT_DESCS

void
FieldValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const WeightFieldPayload *>(payload);
    SetList(out, kFalloffCount, p.falloffCurve);
    SetFlag(out, kClamp, p.clamp);
    SetEnum(out, kPlaneAxis, p.planeAxis);
    SetFlag(out, kPlaneBounded, p.planeBounded);
}

void
CombineValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const CombineWeightsPayload *>(payload);
    SetEnum(out, kMode, p.mode);
    SetEnum(out, kInputs, p.inputs);
    SetFlag(out, kCombineClamp, p.clamp);
}

const rigExec::RigExecFieldDesc kFieldFields[] = {
    Count("falloffCount"),
    Items("falloffCurve", rigExecStd::FloatType(), kFalloffCount),
    Flag("clamp"),
    Enum("planeAxis", WeightFieldAxis::Z),
    Flag("planeBounded"),
};

const rigExec::RigExecFieldDesc kCombineFields[] = {
    Enum("mode", WeightCombineMode::Overlay),
    Enum("inputs", WeightCombineInputs::Fields),
    Flag("clamp"),
};

rigExec::RigExecOpDesc
FieldDesc(const char *name, const RigExecPortDesc *ports, size_t count,
          rigExec::RigExecOpBatchFn runBatch,
          bool (*validate)(const rigExec::RigExecOpCheck &,
                           std::string *) = nullptr)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = count;
    d.runBatch = runBatch;
    d.validate = validate;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFieldFields;
    d.payloadFieldCount = std::size(kFieldFields);
    d.payloadVersion = 1;
    d.payloadValues = &FieldValues;
    return d;
}

const rigExec::RigExecOpDesc kSphere = FieldDesc(
    "SphereWeight", kSpherePorts, std::size(kSpherePorts), &RunSphereWeight);
const rigExec::RigExecOpDesc kPlane = FieldDesc(
    "PlaneWeight", kPlanePorts, std::size(kPlanePorts), &RunPlaneWeight);
const rigExec::RigExecOpDesc kCurve =
    FieldDesc("CurveWeight", kCurvePorts, std::size(kCurvePorts),
              &RunCurveWeight, &CheckCurveWeight);

const RigExecPortDesc kCombinePorts[] = {
    {&CombineWeightsInputs(), &rigExecRig::PointWeightsType(),
     kOptional | rigExec::kRigExecPortArray, "inputs"},
    {&CombineWeightsTargetCount(), &rigExecStd::UIntType(), kOptional,
     "targetCount"},
    {&CombineWeightsStrength(), &rigExecStd::FloatType(), kOptional,
     "strength"},
    {&CombineWeightsInvert(), &rigExecStd::FloatType(), kOptional, "invert"},
    {&CombineWeightsOut(), &rigExecRig::PointWeightsType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::InputValue,
     &CombineWeightsTargetCount(), &CombineWeightsInputs()},
};

const rigExec::RigExecOpDesc kCombine = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "CombineWeights";
    d.ports = kCombinePorts;
    d.portCount = std::size(kCombinePorts);
    d.runBatch = &RunCombineWeights;
    d.validate = &CheckCombineWeights;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kCombineFields;
    d.payloadFieldCount = std::size(kCombineFields);
    d.payloadVersion = 1;
    d.payloadValues = &CombineValues;
    d.state.size = sizeof(CombineState);
    d.state.align = alignof(CombineState);
    d.state.create = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<CombineState *>(dst),
                                             1);
    };
    d.state.destroy = [](void *dst) {
        std::destroy_at(static_cast<CombineState *>(dst));
    };
    return d;
}();

const RigExecPortDesc kVolumePlacementPorts[] = {
    {&VolumePlacementFrame(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "frame"},
    {&VolumePlacementOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecOpDesc kVolumePlacement = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "VolumePlacement";
    d.ports = kVolumePlacementPorts;
    d.portCount = std::size(kVolumePlacementPorts);
    d.runBatch = &RunVolumePlacement;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    return d;
}();

const RigExecPortDesc kStaticPorts[] = {
    {&StaticWeightsValues(), &rigExecStd::FloatArray(), kRigExecPortInput,
     "values"},
    {&StaticWeightsIndices(), &rigExecStd::IntArray(), kOptional, "indices"},
    {&StaticWeightsDefaultWeight(), &rigExecStd::FloatType(), kOptional,
     "defaultWeight"},
    {&StaticWeightsOut(), &rigExecRig::PointWeightsType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &StaticWeightsValues()},
};

const RigExecPortDesc kDynamicPorts[] = {
    {&DynamicWeightsBase(), &rigExecRig::PointWeightsType(), kOptional, "base"},
    {&DynamicWeightsDriver(), &rigExecStd::FloatType(), kOptional, "driver"},
    {&DynamicWeightsScale(), &rigExecStd::FloatType(), kOptional, "scale"},
    {&DynamicWeightsBias(), &rigExecStd::FloatType(), kOptional, "bias"},
    {&DynamicWeightsOut(), &rigExecRig::PointWeightsType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &DynamicWeightsBase()},
};

const RigExecPortDesc kVolumePorts[] = {
    {&VolumeWeightsFrame(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "frame"},
    {&VolumeWeightsPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "points"},
    {&VolumeWeightsSamples(), &rigExecStd::Vec3fArray(), kOptional, "samples"},
    {&VolumeWeightsCurve(), &rigExecStd::Vec3fArray(), kOptional, "curve"},
    {&VolumeWeightsStrength(), &rigExecStd::FloatType(), kOptional, "strength"},
    {&VolumeWeightsInvert(), &rigExecStd::FloatType(), kOptional, "invert"},
    {&VolumeWeightsFalloffMin(), &rigExecStd::FloatType(), kOptional,
     "falloffMin"},
    {&VolumeWeightsFalloffMax(), &rigExecStd::FloatType(), kOptional,
     "falloffMax"},
    {&VolumeWeightsFalloffSamples(), &rigExecStd::FloatArray(), kOptional,
     "falloffSamples"},
    {&VolumeWeightsScale(), &rigExecStd::Vec3fType(), kOptional, "scale"},
    {&VolumeWeightsPositiveScale(), &rigExecStd::Vec3fType(), kOptional,
     "positiveScale"},
    {&VolumeWeightsNegativeScale(), &rigExecStd::Vec3fType(), kOptional,
     "negativeScale"},
    {&VolumeWeightsExtentU(), &rigExecStd::FloatType(), kOptional, "extentU"},
    {&VolumeWeightsExtentV(), &rigExecStd::FloatType(), kOptional, "extentV"},
    {&VolumeWeightsOut(), &rigExecRig::PointWeightsType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &VolumeWeightsPoints()},
};

void
PacketValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const WeightPacketPayload *>(payload);
    SetEnum(out, kRepresentation, p.representation);
    SetFlag(out, kPacketClamp, p.clamp);
}

void
VolumeValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const VolumeWeightsPayload *>(payload);
    SetEnum(out, kShape, p.shape);
    SetFlag(out, kVolumeClamp, p.clamp);
    SetEnum(out, kVolumePlaneAxis, p.planeAxis);
    SetFlag(out, kVolumePlaneBounded, p.planeBounded);
}

const rigExec::RigExecFieldDesc kPacketFields[] = {
    Enum("representation", rigExecRig::WeightRepresentation::Sparse),
    Flag("clamp"),
};

const rigExec::RigExecFieldDesc kVolumeFields[] = {
    Enum("shape", VolumeWeightShape::Curve),
    Flag("clamp"),
    Enum("planeAxis", WeightFieldAxis::Z),
    Flag("planeBounded"),
};

template <class State>
rigExec::RigExecOpDesc
PacketDesc(const char *name, const RigExecPortDesc *ports, size_t count,
           rigExec::RigExecOpBatchFn runBatch,
           const rigExec::RigExecFieldDesc *fields, size_t fieldCount,
           void (*values)(const void *, RigExecFieldValues &))
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = count;
    d.runBatch = runBatch;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = fields;
    d.payloadFieldCount = fieldCount;
    d.payloadVersion = 1;
    d.payloadValues = values;
    d.state.size = sizeof(State);
    d.state.align = alignof(State);
    d.state.create = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<State *>(dst), 1);
    };
    d.state.destroy = [](void *dst) {
        std::destroy_at(static_cast<State *>(dst));
    };
    return d;
}

const rigExec::RigExecOpDesc kStatic = PacketDesc<StaticState>(
    "StaticWeights", kStaticPorts, std::size(kStaticPorts), &RunStaticWeights,
    kPacketFields, std::size(kPacketFields), &PacketValues);
const rigExec::RigExecOpDesc kDynamic =
    PacketDesc<rigExec::RigExecWeightPacketBuild>(
        "DynamicWeights", kDynamicPorts, std::size(kDynamicPorts),
        &RunDynamicWeights, kPacketFields, std::size(kPacketFields),
        &PacketValues);
const rigExec::RigExecOpDesc kVolume = PacketDesc<VolumeState>(
    "VolumeWeights", kVolumePorts, std::size(kVolumePorts), &RunVolumeWeights,
    kVolumeFields, std::size(kVolumeFields), &VolumeValues);

} // namespace

void
RegisterWeightOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(VolumePlacement(), kVolumePlacement);
    registrar.Add(SphereWeight(), kSphere);
    registrar.Add(PlaneWeight(), kPlane);
    registrar.Add(CurveWeight(), kCurve);
    registrar.Add(CombineWeights(), kCombine);
    registrar.Add(StaticWeights(), kStatic);
    registrar.Add(DynamicWeights(), kDynamic);
    registrar.Add(VolumeWeights(), kVolume);
}

} // namespace rigExecOps
