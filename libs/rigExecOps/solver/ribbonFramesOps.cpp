#include "rigExecOps/solver/ribbonFramesOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecOps/solver/liveRests.h"
#include "rigExecMath/ribbonFramesKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/dualQuatMath.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(SampleRibbonFrames)

RIGEXEC_DEFINE_PORT_HANDLE(SampleRibbonFramesPosed)
RIGEXEC_DEFINE_PORT_HANDLE(SampleRibbonFramesRest)
RIGEXEC_DEFINE_PORT_HANDLE(SampleRibbonFramesJointRests)
RIGEXEC_DEFINE_PORT_HANDLE(SampleRibbonFramesJointRestLive)
RIGEXEC_DEFINE_PORT_HANDLE(SampleRibbonFramesOut)

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
RunSampleRibbonFrames(const void *payload, void *, const RigExecOpIo &io,
                      std::string *error)
{
    const auto &p = *static_cast<const SampleRibbonFramesPayload *>(payload);
    const Points &posed = *In<Points>(io, SampleRibbonFramesPosed());
    const Points &rest = *In<Points>(io, SampleRibbonFramesRest());
    std::vector<PointFrame> frames;
    std::vector<PointLandmarks> rests;
    rigExec::RigExecSampleRibbonFramesKernel<Vec3f, Vec3d>(
        posed.Data(), posed.Size(), rest.Data(), rest.Size(), p.sampleCount,
        &frames, &rests);
    if (p.sampleCount < 2 || frames.size() != size_t(p.sampleCount))
        return Fail(error, "SampleRibbonFrames",
                    "needs two samples and two distinct points per curve");
    RebaseLive(io, SampleRibbonFramesJointRests(),
               SampleRibbonFramesJointRestLive(), &frames, &rests);
    PointFrameArray &out = Out<PointFrameArray>(io, SampleRibbonFramesOut());
    out.frames = RigExecArray<PointFrame>(frames.data(), frames.size());
    out.rests = RigExecArray<PointLandmarks>(rests.data(), rests.size());
    return true;
}

} // namespace

void
RegisterRibbonFramesOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &points = rigExecStd::Vec3fArray();

    static const rigExec::RigExecPortDesc ports[] = {
        {&SampleRibbonFramesPosed(), &points, kRigExecPortInput, "posed"},
        {&SampleRibbonFramesRest(), &points, kRigExecPortInput, "rest"},
        {&SampleRibbonFramesJointRests(), &rigExecRig::PointLandmarksList(),
         kOptional, "jointRests"},
        {&SampleRibbonFramesJointRestLive(), &rigExecStd::BoolArray(),
         kOptional, "jointRestLive"},
        {&SampleRibbonFramesOut(), &rigExecRig::PointFrameArrayType(),
         kRigExecPortOutput, "out"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "SampleRibbonFrames";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &RunSampleRibbonFrames;
        d.capabilities =
            rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
        d.payload =
            rigExec::RigExecPayloadHandlingFor<SampleRibbonFramesPayload>();
        return d;
    }();
    registrar.Add(SampleRibbonFrames(), desc);
}

} // namespace rigExecOps
