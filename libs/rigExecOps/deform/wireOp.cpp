#include "rigExecOps/deform/wireOp.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/borrowedSpan.h"
#include "rigExecMath/wireBasisKernel.h"

#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

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

/// What every range of a node reads this frame: the curves, and for
/// sparse weights their lists and basis.
struct WireState {
    std::vector<Vec3f> rest;
    std::vector<Vec3f> posed;
    std::vector<int> indices;
    std::vector<float> weights;
    Basis basis;
};

rigExec::RigExecRangePlan
Accept(const rigExec::RigExecOpBatch &batch, size_t k, size_t,
       std::string *error)
{
    constexpr auto kRefuse = rigExec::RigExecRangePlan::Refuse;
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const Envelope *envelope = In<Envelope>(io, WireEnvelope());
    const auto *indices = In<RigExecArray<int>>(io, WireWeightIndices());
    const auto *weights = In<RigExecArray<float>>(io, WireWeights());
    if (!indices != !weights)
        return Fail(error, "Wire", "weight indices and weights go together"),
               kRefuse;
    if (indices && envelope)
        return Fail(error, "Wire", "an envelope or sparse weights, not both"),
               kRefuse;
    if (!EnvelopeFits(*In<Points>(io, WirePoints()), envelope, "Wire", error))
        return kRefuse;
    return rigExec::RigExecRangePlan::Ranges;
}

bool
Prepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t count,
        std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const WireInstance p = PayloadAt(batch, k);
    WireState &state =
        *static_cast<WireState *>(rigExec::RigExecBatchState(batch, k));
    const Points &rest = *In<Points>(io, WireRestCurve());
    const Points &posed = *In<Points>(io, WireCurve());
    state.rest.assign(rest.begin(), rest.end());
    state.posed.assign(posed.begin(), posed.end());
    const auto &bind = *In<RigExecArray<Vec2f>>(io, WireBindCoords());
    const auto *indices = In<RigExecArray<int>>(io, WireWeightIndices());
    if (!indices) {
        const Curve restCurve{&state.rest, p.order, &p.knots};
        const Curve posedCurve{&state.posed, p.order, &p.knots};
        if (!rigExec::RigExecWireInputsAreUsableKernel<Vec3f, Vec3d, Vec2f>(
                restCurve, posedCurve, bind.Data(), bind.Size(), count))
            return Fail(error, "Wire", "curves or binds do not fit");
        return true;
    }
    const auto &weights = *In<RigExecArray<float>>(io, WireWeights());
    state.indices.assign(indices->begin(), indices->end());
    state.weights.assign(weights.begin(), weights.end());
    if (!rigExec::RigExecNurbsCurveIsValidKernel<Vec3f, Vec3d, Vec2f>(
            &state.rest, &p.knots, p.order) ||
        state.rest.size() != state.posed.size() ||
        !rigExec::RigExecBuildWireBasisKernel<Vec3f, Vec3d, Vec2f>(
            bind.Data(), bind.Size(), count, state.indices, p.order, p.knots,
            state.rest.size(), p.dropoffDistance, &state.basis) ||
        state.indices.size() != state.weights.size())
        return Fail(error, "Wire",
                    "curves, binds or sparse weights do not fit");
    return true;
}

bool
WireRange(const rigExec::RigExecOpBatch &batch, size_t k,
          const rigExec::RigExecOpRange &range, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const WireInstance p = PayloadAt(batch, k);
    const WireState &state =
        *static_cast<const WireState *>(rigExec::RigExecBatchState(batch, k));
    const Points &in = *In<Points>(io, WirePoints());
    Points &out = Out<Points>(io, WireOut());
    CopyRange(in, out, range.begin, range.end);
    Vec3f *const at = out.Data() + range.begin;
    if (In<RigExecArray<int>>(io, WireWeightIndices())) {
        if (!rigExec::RigExecApplyWireBasisGroupKernel<Vec3f, Vec3d, Vec2f>(
                at, range.begin, range.end, state.basis, state.indices,
                state.weights, state.rest, state.posed))
            return Fail(error, "Wire",
                        "curves, binds or sparse weights do not fit");
        return true;
    }
    const auto &bind = *In<RigExecArray<Vec2f>>(io, WireBindCoords());
    const Curve rest{&state.rest, p.order, &p.knots};
    const Curve posed{&state.posed, p.order, &p.knots};
    if (!rigExec::RigExecApplyWireGroupKernel<Vec3f, Vec3d, Vec2f>(
            at, range.begin, range.end, range.count, rest, posed, bind.Data(),
            bind.Size(), p.dropoffDistance, nullptr, 0))
        return Fail(error, "Wire", "curves or binds do not fit");
    BlendEnvelopeRange(in, In<Envelope>(io, WireEnvelope()), out, range.begin,
                       range.end);
    return true;
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
    {&WirePoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortPointwise, "points"},
    {&WireRestCurve(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "restCurve"},
    {&WireCurve(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "curve"},
    {&WireBindCoords(), &rigExecStd::Vec2fArray(), kRigExecPortInput,
     "bindCoords"},
    {&WireEnvelope(), &rigExecStd::FloatArray(),
     kOptional | rigExec::kRigExecPortPointwise, "envelope"},
    {&WireWeightIndices(), &rigExecStd::IntArray(), kOptional, "weightIndices"},
    {&WireWeights(), &rigExecStd::FloatArray(), kOptional, "weights"},
    {&WireOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &WirePoints()},
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Wire";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.range.accept = &Accept;
    d.range.prepare = &Prepare;
    d.range.run = &WireRange;
    d.state = StateOf<WireState>();
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
