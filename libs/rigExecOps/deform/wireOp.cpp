#include "rigExecOps/deform/wireOp.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/borrowedSpan.h"
#include "rigExecMath/wireBasisKernel.h"

#include <cstdint>
#include <iterator>
#include <string>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Wire)

RIGEXEC_DEFINE_PORT_HANDLE(WirePoints)
RIGEXEC_DEFINE_PORT_HANDLE(WireRestCurve)
RIGEXEC_DEFINE_PORT_HANDLE(WireCurve)
RIGEXEC_DEFINE_PORT_HANDLE(WireBindCoords)
RIGEXEC_DEFINE_PORT_HANDLE(WireEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(WireWeightIndices)
RIGEXEC_DEFINE_PORT_HANDLE(WireWeights)
RIGEXEC_DEFINE_PORT_HANDLE(WireOut)

namespace {

using rigExec::RigExecOpIo;
using rigExecStd::RigExecArray;
using rigExecStd::Vec2f;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

/// The curve and basis records the wire kernel reads.
struct Curve {
    const std::vector<Vec3f> *points = nullptr;
    int order = 0;
    const rigExec::RigExecBorrowedSpan<double> *knots = nullptr;
};

struct Basis {
    std::vector<std::vector<std::pair<uint32_t, float>>> byControlPoint;
    bool ascendingIndices = false;
};

enum WireField : uint32_t { kOrder, kKnotCount, kKnots, kDropoffDistance };

/// Instance \p k's curve layout and falloff, its knots read in place.
struct WireInstance {
    int order = 0;
    rigExec::RigExecBorrowedSpan<double> knots;
    double dropoffDistance = 0.0;
};

WireInstance
PayloadAt(const rigExec::RigExecOpBatch &batch, size_t k)
{
    WireInstance p;
    p.order = ValueAt<int32_t>(batch, kOrder, k);
    const auto knots = ListAt<double>(batch, kKnots, k);
    p.knots = {knots.data, knots.Size()};
    p.dropoffDistance = ValueAt<double>(batch, kDropoffDistance, k);
    return p;
}

bool
WireOne(const RigExecOpIo &io, const WireInstance &p, std::string *error)
{
    const Points &in = *In<Points>(io, WirePoints());
    const std::vector<Vec3f> restPoints =
        ToVector(*In<Points>(io, WireRestCurve()));
    const std::vector<Vec3f> posedPoints =
        ToVector(*In<Points>(io, WireCurve()));
    const auto &bind = *In<RigExecArray<Vec2f>>(io, WireBindCoords());
    const Envelope *envelope = In<Envelope>(io, WireEnvelope());
    const auto *indices = In<RigExecArray<int>>(io, WireWeightIndices());
    const auto *weights = In<RigExecArray<float>>(io, WireWeights());
    if (!indices != !weights)
        return Fail(error, "Wire", "weight indices and weights go together");
    if (indices && envelope)
        return Fail(error, "Wire", "an envelope or sparse weights, not both");
    std::vector<Vec3f> points = ToVector(in);
    if (indices) {
        const std::vector<int> index = ToVector(*indices);
        const std::vector<float> weight = ToVector(*weights);
        Basis basis;
        if (!rigExec::RigExecNurbsCurveIsValidKernel<Vec3f, Vec3d, Vec2f>(
                &restPoints, &p.knots, p.order) ||
            restPoints.size() != posedPoints.size() ||
            !rigExec::RigExecBuildWireBasisKernel<Vec3f, Vec3d, Vec2f>(
                bind.Data(), bind.Size(), in.Size(), index, p.order, p.knots,
                restPoints.size(), p.dropoffDistance, &basis) ||
            !rigExec::RigExecApplyWireBasisGroupKernel<Vec3f, Vec3d, Vec2f>(
                points.data(), 0, points.size(), basis, index, weight,
                restPoints, posedPoints))
            return Fail(error, "Wire",
                        "curves, binds or sparse weights do not fit");
        return WritePoints(io, WireOut(), in, points, nullptr, "Wire", error);
    }
    const Curve rest{&restPoints, p.order, &p.knots};
    const Curve posed{&posedPoints, p.order, &p.knots};
    if (!rigExec::RigExecApplyWireGroupKernel<Vec3f, Vec3d, Vec2f>(
            points.data(), 0, points.size(), points.size(), rest, posed,
            bind.Data(), bind.Size(), p.dropoffDistance, nullptr, 0))
        return Fail(error, "Wire", "curves or binds do not fit");
    return WritePoints(io, WireOut(), in, points, envelope, "Wire", error);
}

bool
RunWire(const rigExec::RigExecOpBatch &batch, size_t *failed,
        std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return WireOne(rigExec::RigExecBatchInstance(batch, k),
                       PayloadAt(batch, k), error);
    });
}

void
Values(const void *payload, rigExec::RigExecFieldValues &out)
{
    const WirePayload &p = *static_cast<const WirePayload *>(payload);
    out.Set(kOrder, int32_t(p.order));
    SetList(out, kKnotCount, p.knots);
    out.Set(kDropoffDistance, p.dropoffDistance);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Value("order", rigExecStd::IntType()),
    Count("knotCount"),
    Items("knots", rigExecStd::DoubleType(), kKnotCount),
    Value("dropoffDistance", rigExecStd::DoubleType()),
};

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&WirePoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&WireRestCurve(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "restCurve"},
    {&WireCurve(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "curve"},
    {&WireBindCoords(), &rigExecStd::Vec2fArray(), kRigExecPortInput,
     "bindCoords"},
    {&WireEnvelope(), &rigExecStd::FloatArray(), kOptional, "envelope"},
    {&WireWeightIndices(), &rigExecStd::IntArray(), kOptional, "weightIndices"},
    {&WireWeights(), &rigExecStd::FloatArray(), kOptional, "weights"},
    {&WireOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Wire";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &RunWire;
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
RegisterWireOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Wire(), kDesc);
}

} // namespace rigExecOps
