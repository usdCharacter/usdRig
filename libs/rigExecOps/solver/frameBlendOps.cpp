#include "rigExecOps/solver/frameBlendOps.h"

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

using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;
using namespace opIo;

bool
RunBlendFrames(const void *payload, void *, const RigExecOpIo &io,
               std::string *)
{
    const auto &p = *static_cast<const BlendFramesPayload *>(payload);
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
    out.frames = RigExecArray<PointFrame>(frames.data(), frames.size());
    out.rests = RigExecArray<PointLandmarks>(rests.data(), rests.size());
    return true;
}

} // namespace

void
RegisterFrameBlendOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &frame = rigExecRig::PointFrameType();
    const auto &landmarks = rigExecRig::PointLandmarksType();
    const auto &d = rigExecStd::DoubleType();

    static const rigExec::RigExecPortDesc blendPorts[] = {
        {&BlendFramesA(), &frame, kRigExecPortInput, "a"},
        {&BlendFramesB(), &frame, kRigExecPortInput, "b"},
        {&BlendFramesRest(), &landmarks, kRigExecPortInput, "rest"},
        {&BlendFramesWeight(), &d, kRigExecPortInput, "weight"},
        {&BlendFramesOutRest(), &landmarks, kOptional, "outRest"},
        {&BlendFramesOut(), &frame, kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecPortDesc twistPorts[] = {
        {&DistributeTwistStart(), &frame, kRigExecPortInput, "start"},
        {&DistributeTwistEnd(), &frame, kRigExecPortInput, "end"},
        {&DistributeTwistStartRest(), &landmarks, kRigExecPortInput,
         "startRest"},
        {&DistributeTwistEndRest(), &landmarks, kRigExecPortInput, "endRest"},
        {&DistributeTwistWeights(), &rigExecStd::DoubleArray(),
         kRigExecPortInput, "weights"},
        {&DistributeTwistTurns(), &d, kOptional, "twistTurns"},
        {&DistributeTwistJointRests(), &rigExecRig::PointLandmarksList(),
         kOptional, "jointRests"},
        {&DistributeTwistJointRestLive(), &rigExecStd::BoolArray(), kOptional,
         "jointRestLive"},
        {&DistributeTwistOut(), &rigExecRig::PointFrameArrayType(),
         kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc blend = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "BlendFrames";
        d.ports = blendPorts;
        d.portCount = std::size(blendPorts);
        d.run = &RunBlendFrames;
        d.capabilities = rigExec::kRigExecOpBakeable;
        d.payload = rigExec::RigExecPayloadHandlingFor<BlendFramesPayload>();
        return d;
    }();
    static const rigExec::RigExecOpDesc twist = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "DistributeTwist";
        d.ports = twistPorts;
        d.portCount = std::size(twistPorts);
        d.run = &RunDistributeTwist;
        d.capabilities = rigExec::kRigExecOpBakeable;
        return d;
    }();
    registrar.Add(BlendFrames(), blend);
    registrar.Add(DistributeTwist(), twist);
}

} // namespace rigExecOps
