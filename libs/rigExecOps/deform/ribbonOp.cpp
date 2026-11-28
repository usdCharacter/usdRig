#include "rigExecOps/deform/ribbonOp.h"

#include "rigExecOps/deform/weightPacket.h"
#include "rigExecOps/fieldIo.h"
#include "rigExecOps/internalOps.h"
#include "rigExecOps/opIo.h"
#include "rigExecMath/ribbonKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/dualQuatMath.h"

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Ribbon)
RIGEXEC_DEFINE_OP_HANDLE(GuidePoints)

RIGEXEC_DEFINE_PORT_HANDLE(RibbonPoints)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonBindCoords)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonFrames)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(RibbonOut)

RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsPoints)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsFrames)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsEnvelope)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsOut)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsEnabled)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsDefaultWeight)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsPacket)
RIGEXEC_DEFINE_PORT_HANDLE(GuidePointsWritten)

namespace {

using rigExec::RigExecOpIo;
using rigExecStd::RigExecArray;
using rigExecStd::Vec2f;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

bool
RibbonOne(const RigExecOpIo &io, std::string *error)
{
    const Points &in = *In<Points>(io, RibbonPoints());
    const auto &bind = *In<RigExecArray<Vec2f>>(io, RibbonBindCoords());
    const auto &frames = *In<rigExecRig::PointFrameArray>(io, RibbonFrames());
    if (frames.rests.Size() != frames.frames.Size())
        return Fail(error, "Ribbon", "rests are not parallel to frames");
    Points &out = Out<Points>(io, RibbonOut());
    out = in;
    if (!rigExec::RigExecApplyRibbonKernel<
            rigExec::RigExecDualQuatMath<Vec3f, Vec3d>>(
            out.Data(), out.Size(), bind.Data(), bind.Size(),
            frames.frames.Size(),
            [&](size_t k) -> const auto & { return frames.rests[k]; },
            [&](size_t k) -> const auto & { return frames.frames[k].points; }))
        return Fail(error, "Ribbon",
                    "needs one bind per point and two frame samples");
    return BlendEnvelope(in, In<Envelope>(io, RibbonEnvelope()), &out, "Ribbon",
                         error);
}

bool
CheckRibbon(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const size_t frames = CheckElements(check, RibbonFrames());
    if (!SameCount(CheckElements(check, RibbonBindCoords()),
                   CheckElements(check, RibbonPoints())) ||
        (frames < rigExec::kRigExecPortUnfed && frames < 2))
        return Fail(error, "Ribbon",
                    "needs one bind per point and two frame samples");
    if (const auto *value =
            CheckValue<rigExecRig::PointFrameArray>(check, RibbonFrames());
        value && value->rests.Size() != value->frames.Size())
        return Fail(error, "Ribbon", "rests are not parallel to frames");
    return EnvelopeFits(check, RibbonPoints(), RibbonEnvelope(), "Ribbon",
                        error);
}

bool
CheckGuidePoints(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (const auto *value =
            CheckValue<rigExecRig::PointFrameArray>(check, GuidePointsFrames());
        value && !SameCount(value->frames.Size(),
                            CheckElements(check, GuidePointsPoints())))
        return Fail(error, "GuidePoints", "needs one frame per point");
    const int envelopes = int(Fed(check, GuidePointsEnvelope())) +
                          int(Fed(check, GuidePointsDefaultWeight())) +
                          int(Fed(check, GuidePointsPacket()));
    if (envelopes > 1)
        return Fail(error, "GuidePoints",
                    "one of an envelope, a default weight and a weight "
                    "packet");
    return EnvelopeFits(check, GuidePointsPoints(), GuidePointsEnvelope(),
                        "GuidePoints", error);
}

/// This frame's pass through and weights.
struct GuideState {
    bool pass = false;
    std::vector<float> dense;
};

rigExec::RigExecRangePlan
GuidePlan(const rigExec::RigExecOpBatch &, size_t)
{
    return rigExec::RigExecRangePlan::Whole;
}

bool
GuidePrepare(const rigExec::RigExecOpBatch &batch, size_t k, size_t count,
             std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    GuideState &state =
        *static_cast<GuideState *>(rigExec::RigExecBatchState(batch, k));
    const uint8_t *enabled = In<uint8_t>(io, GuidePointsEnabled());
    state.pass = enabled && !*enabled;
    state.dense.clear();
    if (state.pass)
        return true;
    if (!EnvelopeFits(*In<Points>(io, GuidePointsPoints()),
                      In<Envelope>(io, GuidePointsEnvelope()), "GuidePoints",
                      error))
        return false;
    const auto &frames =
        *In<rigExecRig::PointFrameArray>(io, GuidePointsFrames());
    if (frames.frames.Size() != count)
        return Fail(error, "GuidePoints", "needs one frame per point");
    if (frames.rests.Size() != frames.frames.Size())
        return Fail(error, "GuidePoints", "rests are not parallel to frames");
    const char *why = "";
    if (!weightPacket::ResolveMemberWeights(
            In<float>(io, GuidePointsDefaultWeight()),
            In<weightPacket::PointWeights>(io, GuidePointsPacket()),
            In<RigExecArray<int32_t>>(io, GuidePointsWritten()), count,
            &state.dense, &why))
        return Fail(error, "GuidePoints", why);
    return true;
}

bool
GuideRange(const rigExec::RigExecOpBatch &batch, size_t k,
           const rigExec::RigExecOpRange &range, std::string *)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const GuideState &state =
        *static_cast<const GuideState *>(rigExec::RigExecBatchState(batch, k));
    const Vec3f *from = RangeIn<Vec3f>(io, range, GuidePointsPoints());
    Vec3f *out = RangeOut<Vec3f>(io, range, GuidePointsOut());
    const size_t n = range.end - range.begin;
    if (state.pass) {
        std::copy(from, from + n, out);
        return true;
    }
    const auto &frames =
        *In<rigExecRig::PointFrameArray>(io, GuidePointsFrames());
    for (size_t i = 0; i < n; ++i) {
        const Vec3d &o = frames.frames[range.begin + i].points[0];
        out[i] = Vec3f(float(o[0]), float(o[1]), float(o[2]));
    }
    BlendEnvelopeAt(from,
                    state.dense.empty()
                        ? RangeIn<float>(io, range, GuidePointsEnvelope())
                        : state.dense.data() + range.begin,
                    out, n);
    return true;
}

bool
RunRibbon(const rigExec::RigExecOpBatch &batch, size_t *failed,
          std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return RibbonOne(rigExec::RigExecBatchInstance(batch, k), error);
    });
}

rigExec::RigExecOpDesc
FrameDesc(const char *name, const rigExec::RigExecPortDesc *ports,
          size_t portCount, rigExec::RigExecOpBatchFn runBatch,
          bool (*validate)(const rigExec::RigExecOpCheck &, std::string *),
          uint32_t capabilities)
{
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = name;
    d.ports = ports;
    d.portCount = portCount;
    d.runBatch = runBatch;
    d.validate = validate;
    d.capabilities = rigExec::kRigExecOpBakeable |
                     rigExec::kRigExecOpFreezable |
                     rigExec::kRigExecOpFromLeaves | capabilities;
    return d;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;

const rigExec::RigExecPortDesc kPorts[] = {
    {&RibbonPoints(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "points"},
    {&RibbonBindCoords(), &rigExecStd::Vec2fArray(), kRigExecPortInput,
     "bindCoords"},
    {&RibbonFrames(), &rigExecRig::PointFrameArrayType(), kRigExecPortInput,
     "frames"},
    {&RibbonEnvelope(), &rigExecStd::FloatArray(),
     kRigExecPortInput | kRigExecPortOptional, "envelope"},
    {&RibbonOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
     rigExec::RigExecOutputBound::Input, &RibbonPoints()},
};

constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kGuidePorts[] = {
    {&GuidePointsPoints(), &rigExecStd::Vec3fArray(),
     kRigExecPortInput | rigExec::kRigExecPortPointwise, "points"},
    {&GuidePointsFrames(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortInput, "frames"},
    {&GuidePointsEnvelope(), &rigExecStd::FloatArray(),
     kOptional | rigExec::kRigExecPortPointwise, "envelope"},
    {&GuidePointsOut(), &rigExecStd::Vec3fArray(),
     kRigExecPortOutput | rigExec::kRigExecPortPointwise, "out",
     rigExec::RigExecOutputBound::Input, &GuidePointsPoints()},
    {&GuidePointsEnabled(), &rigExecStd::BoolType(), kOptional, "enabled"},
    {&GuidePointsDefaultWeight(), &rigExecStd::FloatType(), kOptional,
     "defaultWeight"},
    {&GuidePointsPacket(), &rigExecRig::PointWeightsType(), kOptional,
     "packet"},
    {&GuidePointsWritten(), &rigExecStd::IntArray(), kOptional, "written"},
};

const rigExec::RigExecOpDesc kRibbonDesc =
    FrameDesc("Ribbon", kPorts, std::size(kPorts), &RunRibbon, &CheckRibbon,
              rigExec::kRigExecOpSeparateBlend);

const rigExec::RigExecOpDesc kGuideDesc = [] {
    rigExec::RigExecOpDesc d = internalOps::WithSchema(
        FrameDesc("GuidePoints", kGuidePorts, std::size(kGuidePorts), nullptr,
                  &CheckGuidePoints,
                  rigExec::kRigExecOpSeparateBlend | rigExec::kRigExecOpRanged),
        internalOps::kGuidePoints);
    d.range.plan = &GuidePlan;
    d.range.prepare = &GuidePrepare;
    d.range.run = &GuideRange;
    d.state = StateOf<GuideState>();
    return d;
}();

} // namespace

void
RegisterRibbonOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Ribbon(), kRibbonDesc);
    registrar.Add(GuidePoints(), kGuideDesc);
}

} // namespace rigExecOps
