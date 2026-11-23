#include "rigExecOps/weight/weightOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
#include "rigExecEvalCore/weights.h"
#include "rigExecMath/spatialAccel.h"
#include "rigExecMath/weightFieldKernel.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cmath>
#include <cstring>
#include <iterator>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SphereWeight)
RIGEXEC_DEFINE_OP_HANDLE(PlaneWeight)
RIGEXEC_DEFINE_OP_HANDLE(CurveWeight)
RIGEXEC_DEFINE_OP_HANDLE(CombineWeights)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsInputs)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsTargetCount)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsStrength)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsInvert)
RIGEXEC_DEFINE_PORT_HANDLE(CombineWeightsOut)

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

using rigExecRig::PointWeights;
using WeightList = InList<PointWeights>;

static_assert(int(WeightCombineMode::Overlay) ==
                      rigExec::eval::kWeightCombineOverlay &&
                  int(rigExecRig::WeightRepresentation::Sparse) ==
                      rigExec::eval::kWeightSparse,
              "combine codes follow rigExecEvalCore");

/// Op state: the packet fold's scratch and its input indices.
struct CombineState {
    std::vector<float> combineField;
    std::vector<int32_t> indices;
    rigExec::eval::WeightPacketResult result;
};

/// The host eval::BuildCombinePacket reads its inputs through.
struct CombineHost {
    struct Math {
        using Vec3f = rigExecStd::Vec3f;
        using Vec3d = rigExecStd::Vec3d;
    };
    struct State {
        const WeightList &inputs;
        CombineState &state;
        CombineState *packetScratch(size_t)
        {
            return &state;
        }
        size_t packetCount() const
        {
            return inputs.Size();
        }
        rigExec::eval::WeightPacketView packetView(size_t k) const
        {
            const PointWeights &w = inputs[k];
            return {w.valid != 0,    int(w.representation), w.defaultWeight,
                    w.values.Data(), w.values.Size(),       w.indices.Data(),
                    w.indices.Size()};
        }
        void storePacket(size_t, rigExec::eval::WeightPacketResult &result)
        {
            std::swap(state.result, result);
        }
        void storePacket(size_t, rigExec::eval::WeightPacketResult &&result)
        {
            state.result = std::move(result);
        }
    };
    State s;
    State &state()
    {
        return s;
    }
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
RunCombineWeights(const void *payload, void *state, const RigExecOpIo &io,
                  std::string *error)
{
    namespace eval = rigExec::eval;
    const auto &p = *static_cast<const CombineWeightsPayload *>(payload);
    auto &s = *static_cast<CombineState *>(state);
    const WeightList inputs(io, CombineWeightsInputs());
    const size_t targetCount =
        size_t(InOr(io, CombineWeightsTargetCount(), uint32_t(0)));
    const float strength = InOr(io, CombineWeightsStrength(), 1.0f);
    const float invert = InOr(io, CombineWeightsInvert(), 0.0f);
    PointWeights &out = Out<PointWeights>(io, CombineWeightsOut());

    if (p.inputs == WeightCombineInputs::Fields) {
        std::vector<float> &folded = s.combineField;
        if (!eval::weights_detail::CombineStreamed(
                int(p.mode), inputs.Size(), targetCount,
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

    if (s.indices.size() < inputs.Size()) {
        s.indices.resize(inputs.Size());
        std::iota(s.indices.begin(), s.indices.end(), 0);
    }
    eval::WeightPacketFacts<CombineHost::Math> facts;
    facts.representation = eval::kWeightDense;
    facts.rangePolicy =
        p.clamp ? eval::kWeightPolicyClamp : eval::kWeightPolicyStrict;
    facts.combineMode = int(p.mode);
    facts.inputCount = inputs.Size();
    facts.inputIndices = s.indices.data();
    facts.combineTargetCount = targetCount;
    facts.scalars[4] = strength;
    facts.scalars[5] = invert;
    CombineHost host{{inputs, s}};
    eval::weights_detail::BuildCombinePacket(host, 0, facts);
    if (s.result.bare)
        return Fail(error, "CombineWeights",
                    p.clamp ? "non-finite weight" : "weight out of range");
    WriteDense(s.result.values, p, s.result.valid, &out);
    return true;
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
        &Op##Out(), &rigExecStd::FloatArray(), kRigExecPortOutput, "out"       \
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

using payloadIo::Reader;
using payloadIo::Writer;

void
Save(Writer &w, const WeightFieldPayload &p)
{
    w.Version(1);
    payloadIo::PutList(w, p.falloffCurve);
    w.Bool(p.clamp);
    w.Enum(p.planeAxis);
    w.Bool(p.planeBounded);
}

void
Load(Reader &r, WeightFieldPayload &p)
{
    r.Version(1);
    payloadIo::GetList(r, p.falloffCurve);
    p.clamp = r.Bool();
    p.planeAxis = r.Enum(WeightFieldAxis::Z);
    p.planeBounded = r.Bool();
}

void
Save(Writer &w, const CombineWeightsPayload &p)
{
    w.Version(1);
    w.Enum(p.mode);
    w.Enum(p.inputs);
    w.Bool(p.clamp);
}

void
Load(Reader &r, CombineWeightsPayload &p)
{
    r.Version(1);
    p.mode = r.Enum(WeightCombineMode::Overlay);
    p.inputs = r.Enum(WeightCombineInputs::Fields);
    p.clamp = r.Bool();
}

rigExec::RigExecOpDesc
FieldDesc(const char *name, const RigExecPortDesc *ports, size_t count,
          rigExec::RigExecOpRunFn run)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = count;
    d.run = run;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<WeightFieldPayload, &Save, &Load>();
    return d;
}

const rigExec::RigExecOpDesc kSphere = FieldDesc(
    "SphereWeight", kSpherePorts, std::size(kSpherePorts), &RunSphereWeight);
const rigExec::RigExecOpDesc kPlane = FieldDesc(
    "PlaneWeight", kPlanePorts, std::size(kPlanePorts), &RunPlaneWeight);
const rigExec::RigExecOpDesc kCurve = FieldDesc(
    "CurveWeight", kCurvePorts, std::size(kCurvePorts), &RunCurveWeight);

const RigExecPortDesc kCombinePorts[] = {
    {&CombineWeightsInputs(), &rigExecRig::PointWeightsType(),
     kOptional | rigExec::kRigExecPortArray, "inputs"},
    {&CombineWeightsTargetCount(), &rigExecStd::UIntType(), kOptional,
     "targetCount"},
    {&CombineWeightsStrength(), &rigExecStd::FloatType(), kOptional,
     "strength"},
    {&CombineWeightsInvert(), &rigExecStd::FloatType(), kOptional, "invert"},
    {&CombineWeightsOut(), &rigExecRig::PointWeightsType(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecOpDesc kCombine = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "CombineWeights";
    d.ports = kCombinePorts;
    d.portCount = std::size(kCombinePorts);
    d.run = &RunCombineWeights;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payload = rigExec::RigExecPayloadHandlingFor<CombineWeightsPayload, &Save,
                                                   &Load>();
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

} // namespace

void
RegisterWeightOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SphereWeight(), kSphere);
    registrar.Add(PlaneWeight(), kPlane);
    registrar.Add(CurveWeight(), kCurve);
    registrar.Add(CombineWeights(), kCombine);
}

} // namespace rigExecOps
