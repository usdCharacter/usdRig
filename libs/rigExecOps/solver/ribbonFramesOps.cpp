#include "rigExecOps/solver/ribbonFramesOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/internalOps.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/solver/liveRests.h"
#include "rigExecMath/ribbonFramesKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/dualQuatMath.h"

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

/// SampleRibbonFrames' declared field.
enum RibbonField : uint32_t { kSampleCount };

/// SampleRibbonFrames' op state: both curves' sampling storage.
struct RibbonScratch {
    rigExec::RigExecCurveFrameScratchT<Vec3f> posed, rest;
};

bool
SampleRibbonFramesOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    SampleRibbonFramesPayload p;
    p.sampleCount = ValueAt<int32_t>(batch, kSampleCount, k);
    const Points &posed = *In<Points>(io, SampleRibbonFramesPosed());
    const Points &rest = *In<Points>(io, SampleRibbonFramesRest());
    if (p.sampleCount < 2)
        return Fail(error, "SampleRibbonFrames", "needs two samples");
    auto &scratch =
        *static_cast<RibbonScratch *>(rigExec::RigExecBatchState(batch, k));
    PointFrameArray &out = Out<PointFrameArray>(io, SampleRibbonFramesOut());
    const size_t n = size_t(p.sampleCount);
    out.frames.Resize(n);
    out.rests.Resize(n);
    if (!rigExec::RigExecSampleRibbonFramesKernel<Vec3f, Vec3d>(
            posed.Data(), posed.Size(), rest.Data(), rest.Size(), p.sampleCount,
            &scratch.posed, &scratch.rest, out.frames.Data(),
            out.rests.Data())) {
        out.frames.Resize(0);
        out.rests.Resize(0);
        return true;
    }
    RebaseLive(io, SampleRibbonFramesJointRests(),
               SampleRibbonFramesJointRestLive(), &out);
    return true;
}

bool
CheckSampleRibbonFrames(const rigExec::RigExecOpCheck &check,
                        std::string *error)
{
    if (ValueAt<int32_t>(check.batch, kSampleCount, check.k) < 2)
        return Fail(error, "SampleRibbonFrames", "needs two samples");
    if (Fed(check, SampleRibbonFramesJointRests()) !=
        Fed(check, SampleRibbonFramesJointRestLive()))
        return Fail(error, "SampleRibbonFrames",
                    "needs joint rests and their live flags together");
    return true;
}

bool
RunSampleRibbonFrames(const RigExecOpBatch &batch, size_t *failed,
                      std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return SampleRibbonFramesOne(batch, k, error);
    });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&SampleRibbonFramesPosed(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "posed"},
    {&SampleRibbonFramesRest(), &rigExecStd::Vec3fArray(), kRigExecPortInput,
     "rest"},
    {&SampleRibbonFramesJointRests(), &rigExecRig::PointLandmarksList(),
     kOptional, "jointRests"},
    {&SampleRibbonFramesJointRestLive(), &rigExecStd::BoolArray(), kOptional,
     "jointRestLive"},
    {&SampleRibbonFramesOut(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortOutput, "out", rigExec::RigExecOutputBound::Field, nullptr,
     nullptr, kSampleCount},
};

void
RibbonValues(const void *payload, RigExecFieldValues &out)
{
    out.Set(kSampleCount,
            int32_t(static_cast<const SampleRibbonFramesPayload *>(payload)
                        ->sampleCount));
}

const rigExec::RigExecFieldDesc kFields[] = {
    Value("sampleCount", rigExecStd::IntType())};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "SampleRibbonFrames";
    d.schema = &internalOps::kSampleRibbonFrames;
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &RunSampleRibbonFrames;
    d.validate = &CheckSampleRibbonFrames;
    d.state = StateOf<RibbonScratch>();
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 1;
    d.payloadValues = &RibbonValues;
    return d;
}();

} // namespace

void
RegisterRibbonFramesOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(SampleRibbonFrames(), kDesc);
}

} // namespace rigExecOps
