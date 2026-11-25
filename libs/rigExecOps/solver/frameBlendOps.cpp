#include "rigExecOps/solver/frameBlendOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/solver/liveRests.h"
#include "rigExecMath/frameBlendKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <cmath>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(BlendFrames)
RIGEXEC_DEFINE_OP_HANDLE(DistributeTwist)

RIGEXEC_DEFINE_PORT_HANDLE(BlendFramesA)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFramesB)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFramesRest)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFramesWeight)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFramesOutRest)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFramesOut)

RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistStart)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistEnd)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistStartRest)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistEndRest)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistWeights)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistTurns)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistJointRests)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistJointRestLive)
RIGEXEC_DEFINE_PORT_HANDLE(DistributeTwistOut)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;
using namespace fieldIo;

/// BlendFrames' declared field.
enum BlendField : uint32_t { kLogScale };

bool
BlendFramesOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    BlendFramesPayload p;
    p.logScale = FlagAt(batch, kLogScale, k);
    const double weight = *In<double>(io, BlendFramesWeight());
    Out<PointFrame>(io, BlendFramesOut()) =
        rigExec::RigExecBlendFramesKernel<Vec3f, Vec3d>(
            *In<PointFrame>(io, BlendFramesA()),
            *In<PointFrame>(io, BlendFramesB()),
            *In<PointLandmarks>(io, BlendFramesRest()), weight, p.logScale,
            In<PointLandmarks>(io, BlendFramesOutRest()));
    return true;
}

bool
RunBlendFrames(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return BlendFramesOne(batch, k, error);
    });
}

bool
CheckDistributeTwist(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (Fed(check, DistributeTwistJointRests()) !=
        Fed(check, DistributeTwistJointRestLive()))
        return Fail(error, "DistributeTwist",
                    "needs joint rests and their live flags together");
    return true;
}

bool
RunDistributeTwist(const void *, void *, const RigExecOpIo &io,
                   std::string *error)
{
    const RigExecArray<double> &weightArray =
        *In<RigExecArray<double>>(io, DistributeTwistWeights());
    const std::vector<double> weights(weightArray.begin(), weightArray.end());
    const double turns = InOr(io, DistributeTwistTurns(), 0.0);
    const PointLandmarks &startRest =
        *In<PointLandmarks>(io, DistributeTwistStartRest());
    std::vector<PointFrame> frames =
        rigExec::RigExecDistributeTwistKernel<Vec3f, Vec3d>(
            *In<PointFrame>(io, DistributeTwistStart()),
            *In<PointFrame>(io, DistributeTwistEnd()), startRest,
            *In<PointLandmarks>(io, DistributeTwistEndRest()), weights, turns);
    if (frames.size() != weights.size())
        return Fail(error, "DistributeTwist", "needs a finite twist");
    std::vector<PointLandmarks> rests(frames.size(), startRest);
    RebaseLive(io, DistributeTwistJointRests(), DistributeTwistJointRestLive(),
               &frames, &rests);
    PointFrameArray &out = Out<PointFrameArray>(io, DistributeTwistOut());
    out.frames.Assign(frames.data(), frames.size());
    out.rests.Assign(rests.data(), rests.size());
    return true;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kBlendPorts[] = {
    {&BlendFramesA(), &rigExecRig::PointFrameType(), kRigExecPortInput, "a"},
    {&BlendFramesB(), &rigExecRig::PointFrameType(), kRigExecPortInput, "b"},
    {&BlendFramesRest(), &rigExecRig::PointLandmarksType(), kRigExecPortInput,
     "rest"},
    {&BlendFramesWeight(), &rigExecStd::DoubleType(), kRigExecPortInput,
     "weight"},
    {&BlendFramesOutRest(), &rigExecRig::PointLandmarksType(), kOptional,
     "outRest"},
    {&BlendFramesOut(), &rigExecRig::PointFrameType(), kRigExecPortOutput,
     "out"},
};

const rigExec::RigExecPortDesc kTwistPorts[] = {
    {&DistributeTwistStart(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "start"},
    {&DistributeTwistEnd(), &rigExecRig::PointFrameType(), kRigExecPortInput,
     "end"},
    {&DistributeTwistStartRest(), &rigExecRig::PointLandmarksType(),
     kRigExecPortInput, "startRest"},
    {&DistributeTwistEndRest(), &rigExecRig::PointLandmarksType(),
     kRigExecPortInput, "endRest"},
    {&DistributeTwistWeights(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "weights"},
    {&DistributeTwistTurns(), &rigExecStd::DoubleType(), kOptional,
     "twistTurns"},
    {&DistributeTwistJointRests(), &rigExecRig::PointLandmarksList(), kOptional,
     "jointRests"},
    {&DistributeTwistJointRestLive(), &rigExecStd::BoolArray(), kOptional,
     "jointRestLive"},
    {&DistributeTwistOut(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortOutput, "out", rigExec::RigExecOutputBound::Input,
     &DistributeTwistWeights()},
};

void
BlendValues(const void *payload, RigExecFieldValues &out)
{
    SetFlag(out, kLogScale,
            static_cast<const BlendFramesPayload *>(payload)->logScale);
}

const rigExec::RigExecFieldDesc kBlendFields[] = {Flag("logScale")};

const rigExec::RigExecOpDesc kBlend = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "BlendFrames";
    d.ports = kBlendPorts;
    d.portCount = std::size(kBlendPorts);
    d.runBatch = &RunBlendFrames;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kBlendFields;
    d.payloadFieldCount = std::size(kBlendFields);
    d.payloadVersion = 1;
    d.payloadValues = &BlendValues;
    return d;
}();

const rigExec::RigExecOpDesc kTwist = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "DistributeTwist";
    d.ports = kTwistPorts;
    d.portCount = std::size(kTwistPorts);
    d.run = &RunDistributeTwist;
    d.validate = &CheckDistributeTwist;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    return d;
}();

} // namespace

void
RegisterFrameBlendOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(BlendFrames(), kBlend);
    registrar.Add(DistributeTwist(), kTwist);
}

} // namespace rigExecOps
