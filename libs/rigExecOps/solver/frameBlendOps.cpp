#include "rigExecOps/solver/frameBlendOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/internalOps.h"
#include "rigExecOps/opIo.h"
#include "rigExecOps/solver/liveRests.h"
#include "rigExecMath/frameBlendKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/dualQuatMath.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(BlendFrames)
RIGEXEC_DEFINE_OP_HANDLE(DistributeTwist)
RIGEXEC_DEFINE_OP_HANDLE(BlendFrameArrays)

RIGEXEC_DEFINE_PORT_HANDLE(BlendFrameArraysA)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFrameArraysB)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFrameArraysWeight)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFrameArraysJointRests)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFrameArraysJointRestLive)
RIGEXEC_DEFINE_PORT_HANDLE(BlendFrameArraysOut)

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

/// BlendFrameArrays' declared fields, in saved order.
enum ArrayBlendField : uint32_t { kArrayLogScale, kLeadingA, kLeadingB };

/// \p in's frames and rests after its first \p leading, into \p out.
void
CopyAfter(const PointFrameArray &in, size_t leading, PointFrameArray *out)
{
    const size_t frames =
        in.frames.Size() > leading ? in.frames.Size() - leading : 0;
    const size_t rests =
        in.rests.Size() > leading ? in.rests.Size() - leading : 0;
    out->frames.Assign(frames ? in.frames.Data() + leading : nullptr, frames);
    out->rests.Assign(rests ? in.rests.Data() + leading : nullptr, rests);
}

bool
BlendFrameArraysOne(const RigExecOpBatch &batch, size_t k)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const PointFrameArray *a = In<PointFrameArray>(io, BlendFrameArraysA());
    const PointFrameArray *b = In<PointFrameArray>(io, BlendFrameArraysB());
    const size_t leadA = ValueAt<uint32_t>(batch, kLeadingA, k);
    const size_t leadB = ValueAt<uint32_t>(batch, kLeadingB, k);
    PointFrameArray &out = Out<PointFrameArray>(io, BlendFrameArraysOut());
    if (!a || !b) {
        out.frames.Resize(0);
        out.rests.Resize(0);
        if (a || b)
            CopyAfter(a ? *a : *b, a ? leadA : leadB, &out);
        return true;
    }
    const size_t n = a->frames.Size() > leadA ? a->frames.Size() - leadA : 0;
    const size_t nb = b->frames.Size() > leadB ? b->frames.Size() - leadB : 0;
    const size_t restsA = a->rests.Size() > leadA ? a->rests.Size() - leadA : 0;
    if (n != nb || restsA != n) {
        out.frames.Resize(0);
        out.rests.Resize(0);
        return true;
    }
    const double weight =
        std::min(std::max(InOr(io, BlendFrameArraysWeight(), 0.0), 0.0), 1.0);
    const bool logScale = FlagAt(batch, kArrayLogScale, k);
    using Landmarks = RigExecArray<PointLandmarks>;
    const Landmarks *joints = In<Landmarks>(io, BlendFrameArraysJointRests());
    const auto *live =
        In<RigExecArray<uint8_t>>(io, BlendFrameArraysJointRestLive());
    out.frames.Resize(n);
    out.rests.Resize(n);
    for (size_t e = 0; e < n; ++e) {
        const PointLandmarks *outRest = joints && live && e < live->Size() &&
                                                (*live)[e] && e < joints->Size()
                                            ? &(*joints)[e]
                                            : nullptr;
        const PointLandmarks &rest = a->rests[e + leadA];
        out.frames[e] = rigExec::RigExecBlendFramesKernel<Vec3f, Vec3d>(
            a->frames[e + leadA], b->frames[e + leadB], rest, weight, logScale,
            outRest);
        out.rests[e] = outRest ? *outRest : rest;
    }
    return true;
}

bool
RunBlendFrameArrays(const RigExecOpBatch &batch, size_t *failed, std::string *)
{
    return EachInstance(
        batch, failed, [&](size_t k) { return BlendFrameArraysOne(batch, k); });
}

bool
CheckBlendFrameArrays(const rigExec::RigExecOpCheck &check, std::string *error)
{
    if (Fed(check, BlendFrameArraysJointRests()) !=
        Fed(check, BlendFrameArraysJointRestLive()))
        return Fail(error, "BlendFrameArrays",
                    "needs joint rests and their live flags together");
    return true;
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
    const RigExecArray<double> &weights =
        *In<RigExecArray<double>>(io, DistributeTwistWeights());
    const double turns = InOr(io, DistributeTwistTurns(), 0.0);
    const PointLandmarks &startRest =
        *In<PointLandmarks>(io, DistributeTwistStartRest());
    PointFrameArray &out = Out<PointFrameArray>(io, DistributeTwistOut());
    const size_t n = weights.Size();
    out.frames.Resize(n);
    if (!rigExec::RigExecDistributeTwistKernel<Vec3f, Vec3d>(
            *In<PointFrame>(io, DistributeTwistStart()),
            *In<PointFrame>(io, DistributeTwistEnd()), startRest,
            *In<PointLandmarks>(io, DistributeTwistEndRest()), weights.Data(),
            n, turns, out.frames.Data()))
        return Fail(error, "DistributeTwist", "needs a finite twist");
    out.rests.Resize(n);
    std::fill(out.rests.begin(), out.rests.end(), startRest);
    RebaseLive(io, DistributeTwistJointRests(), DistributeTwistJointRestLive(),
               &out);
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

const rigExec::RigExecPortDesc kArrayBlendPorts[] = {
    {&BlendFrameArraysA(), &rigExecRig::PointFrameArrayType(), kOptional, "a"},
    {&BlendFrameArraysB(), &rigExecRig::PointFrameArrayType(), kOptional, "b"},
    {&BlendFrameArraysWeight(), &rigExecStd::DoubleType(), kOptional, "weight"},
    {&BlendFrameArraysJointRests(), &rigExecRig::PointLandmarksList(),
     kOptional, "jointRests"},
    {&BlendFrameArraysJointRestLive(), &rigExecStd::BoolArray(), kOptional,
     "jointRestLive"},
    {&BlendFrameArraysOut(), &rigExecRig::PointFrameArrayType(),
     kRigExecPortOutput, "out", rigExec::RigExecOutputBound::Input,
     &BlendFrameArraysA(), &BlendFrameArraysB()},
};

void
ArrayBlendValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const BlendFrameArraysPayload *>(payload);
    SetFlag(out, kArrayLogScale, p.logScale);
    out.Set(kLeadingA, uint32_t(p.leadingA));
    out.Set(kLeadingB, uint32_t(p.leadingB));
}

const rigExec::RigExecFieldDesc kArrayBlendFields[] = {
    Flag("logScale"),
    Value("leadingA", rigExecStd::UIntType()),
    Value("leadingB", rigExecStd::UIntType()),
};

const rigExec::RigExecOpDesc kArrayBlend = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "BlendFrameArrays";
    d.schema = &internalOps::kBlendFrameArrays;
    d.ports = kArrayBlendPorts;
    d.portCount = std::size(kArrayBlendPorts);
    d.runBatch = &RunBlendFrameArrays;
    d.validate = &CheckBlendFrameArrays;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kArrayBlendFields;
    d.payloadFieldCount = std::size(kArrayBlendFields);
    d.payloadVersion = 1;
    d.payloadValues = &ArrayBlendValues;
    return d;
}();

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
    d.schema = &internalOps::kDistributeTwist;
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
    registrar.Add(BlendFrameArrays(), kArrayBlend);
}

} // namespace rigExecOps
