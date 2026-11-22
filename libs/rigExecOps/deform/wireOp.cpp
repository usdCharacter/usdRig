#include "rigExecOps/deform/wireOp.h"

#include "rigExecOps/opIo.h"
#include "rigExecMath/wireKernel.h"

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

rigExec::RigExecNurbsCurveSpan<Vec3f>
CurveOf(const Points &controlPoints, const WirePayload &p)
{
    return {controlPoints.Data(), controlPoints.Size(), p.order, p.knots.data(),
            p.knots.size()};
}

bool
RunWire(const void *payload, void *, const RigExecOpIo &io, std::string *error)
{
    const WirePayload &p = *static_cast<const WirePayload *>(payload);
    const Points &in = *In<Points>(io, WirePoints());
    const Points &restCurve = *In<Points>(io, WireRestCurve());
    const Points &curve = *In<Points>(io, WireCurve());
    const auto &bind = *In<RigExecArray<Vec2f>>(io, WireBindCoords());
    const Envelope *envelope = In<Envelope>(io, WireEnvelope());
    const auto *indices = In<RigExecArray<int>>(io, WireWeightIndices());
    const auto *weights = In<RigExecArray<float>>(io, WireWeights());
    if (!indices != !weights)
        return Fail(error, "Wire", "weight indices and weights go together");
    if (indices && envelope)
        return Fail(error, "Wire", "an envelope or sparse weights, not both");
    Points &out = Out<Points>(io, WireOut());
    out = in;
    const auto rest = CurveOf(restCurve, p);
    const auto posed = CurveOf(curve, p);
    if (indices) {
        rigExec::RigExecWireBasis basis;
        if (!rest.IsValid() || !rest.Matches(posed) ||
            !rigExec::RigExecBuildWireBasisKernel(
                bind.Data(), bind.Size(), in.Size(), indices->Data(),
                indices->Size(), p.order, p.knots.data(), p.knots.size(),
                restCurve.Size(), p.dropoffDistance, &basis) ||
            !rigExec::RigExecApplyWireBasisGroupKernel(
                out.Data(), 0, out.Size(), basis, indices->Data(),
                indices->Size(), weights->Data(), weights->Size(),
                restCurve.Data(), restCurve.Size(), curve.Data(), curve.Size()))
            return Fail(error, "Wire",
                        "curves, binds or sparse weights do not fit");
        return true;
    }
    if (!rigExec::RigExecApplyWireGroupKernel<Vec3f, Vec3d>(
            out.Data(), 0, out.Size(), out.Size(), rest, posed, bind.Data(),
            bind.Size(), p.dropoffDistance, nullptr, 0))
        return Fail(error, "Wire", "curves or binds do not fit");
    return BlendEnvelope(in, envelope, &out, "Wire", error);
}

} // namespace

void
RegisterWireOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();
    const auto &floats = rigExecStd::FloatArray();

    static const rigExec::RigExecPortDesc ports[] = {
        {&WirePoints(), &points, kRigExecPortInput, "points"},
        {&WireRestCurve(), &points, kRigExecPortInput, "restCurve"},
        {&WireCurve(), &points, kRigExecPortInput, "curve"},
        {&WireBindCoords(), &rigExecStd::Vec2fArray(), kRigExecPortInput,
         "bindCoords"},
        {&WireEnvelope(), &floats, kOptional, "envelope"},
        {&WireWeightIndices(), &rigExecStd::IntArray(), kOptional,
         "weightIndices"},
        {&WireWeights(), &floats, kOptional, "weights"},
        {&WireOut(), &points, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "Wire";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &RunWire;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<WirePayload>();
        return d;
    }();
    registrar.Add(Wire(), desc);
}

} // namespace rigExecOps
