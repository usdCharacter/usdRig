#include "rigExecOps/solver/skeleton2dOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecRigTypes/rigTypes.h"

#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(Skeleton2d)

RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dRests)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dStartRest)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dStartPose)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dExternalRests)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dExternalPoses)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dChannels)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dSelectors)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dGroupSelectors)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dOut)
RIGEXEC_DEFINE_PORT_HANDLE(Skeleton2dResult)

namespace {

using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::RigExecArray;
using Landmarks = RigExecArray<PointLandmarks>;
using namespace opIo;

/// The solve's scratch and the vectors the kernel reads, kept per node.
struct Skeleton2dState {
    rigExec::RigExecSkeleton2dScratch scratch;
    std::vector<PointLandmarks> rests, externalRests, externalPoses;
    std::vector<PointFrame> frames;
};

const double *
Data(const RigExecArray<double> *a)
{
    return a ? a->Data() : nullptr;
}

size_t
Size(const RigExecArray<double> *a)
{
    return a ? a->Size() : 0;
}

bool
RunSkeleton2d(const void *payload, void *state, const RigExecOpIo &io,
              std::string *error)
{
    const auto &p = *static_cast<const Skeleton2dPayload *>(payload);
    Skeleton2dState &s = *static_cast<Skeleton2dState *>(state);
    const Landmarks &rests = *In<Landmarks>(io, Skeleton2dRests());
    const auto &channels = *In<RigExecArray<double>>(io, Skeleton2dChannels());
    const PointLandmarks *startRest =
        In<PointLandmarks>(io, Skeleton2dStartRest());
    const PointLandmarks *startPose =
        In<PointLandmarks>(io, Skeleton2dStartPose());
    const Landmarks *externalRests =
        In<Landmarks>(io, Skeleton2dExternalRests());
    const Landmarks *externalPoses =
        In<Landmarks>(io, Skeleton2dExternalPoses());
    const auto *selectors = In<RigExecArray<double>>(io, Skeleton2dSelectors());
    const auto *groupSelectors =
        In<RigExecArray<double>>(io, Skeleton2dGroupSelectors());
    const size_t externals = externalRests ? externalRests->Size() : 0;
    if (channels.Size() != rests.Size() * rigExec::kRigExecBone2dChannelCount)
        return Fail(error, "Skeleton2d", "needs every channel of every joint");
    if (!startRest != !startPose)
        return Fail(error, "Skeleton2d",
                    "needs the start rest and pose together");
    if ((externalPoses ? externalPoses->Size() : 0) != externals)
        return Fail(error, "Skeleton2d",
                    "needs one posed frame per outside parent space");
    s.rests.assign(rests.begin(), rests.end());
    s.externalRests.clear();
    s.externalPoses.clear();
    if (externals) {
        s.externalRests.assign(externalRests->begin(), externalRests->end());
        s.externalPoses.assign(externalPoses->begin(), externalPoses->end());
    }
    rigExec::RigExecSkeleton2dLandmarkInputs<PointLandmarks> frames;
    frames.startRest = startRest;
    frames.startPose = startPose;
    frames.externalRests = s.externalRests.data();
    frames.externalPoses = s.externalPoses.data();
    frames.externalCount = externals;
    frames.selectors = Data(selectors);
    frames.selectorCount = Size(selectors);
    frames.groupSelectors = Data(groupSelectors);
    frames.groupSelectorCount = Size(groupSelectors);
    const rigExec::RigExecSkeleton2dStatic extra =
        rigExec::RigExecSkeleton2dStaticOf(p.spaceSources, p.targetCandidates,
                                           p.groups, p.groupMembers);
    s.frames.clear();
    const rigExec::RigExecSkeleton2dResult result =
        rigExec::RigExecSolveSkeleton2dJoints(
            p.order, s.rests, p.setup, extra, frames,
            [&](size_t j, rigExec::RigExecBone2dChannels *c) {
                const double *v =
                    channels.Data() + j * rigExec::kRigExecBone2dChannelCount;
                c->tx = v[0];
                c->ty = v[1];
                c->rz = v[2];
                c->sx = v[3];
                c->sideFlip = v[4] != 0.0;
                c->endFlip = v[5] != 0.0;
                c->lock = v[6] != 0.0;
                c->lockX = v[7];
                c->lockY = v[8];
                c->lockAngle = v[9];
                c->offsetX = v[10];
                c->offsetY = v[11];
                c->targetIndex = v[12];
            },
            rigExecRig::kPointFrameValid, &s.scratch, &s.frames);
    PointFrameArray &out = Out<PointFrameArray>(io, Skeleton2dOut());
    if (result == rigExec::RigExecSkeleton2dResult::Malformed) {
        out.frames.Resize(0);
        out.rests.Resize(0);
    } else {
        out.frames = RigExecArray<PointFrame>(s.frames.data(), s.frames.size());
        out.rests = rests;
    }
    Out<uint32_t>(io, Skeleton2dResult()) = uint32_t(result);
    return true;
}

} // namespace

void
RegisterSkeleton2dOps(rigExec::RigExecOpRegistrar &registrar)
{
    using rigExec::kRigExecPortInput;
    using rigExec::kRigExecPortOptional;
    using rigExec::kRigExecPortOutput;
    constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
    const auto &landmarks = rigExecRig::PointLandmarksType();
    const auto &list = rigExecRig::PointLandmarksList();
    const auto &doubles = rigExecStd::DoubleArray();

    static const rigExec::RigExecPortDesc ports[] = {
        {&Skeleton2dRests(), &list, kRigExecPortInput, "rests"},
        {&Skeleton2dStartRest(), &landmarks, kOptional, "startRest"},
        {&Skeleton2dStartPose(), &landmarks, kOptional, "startPose"},
        {&Skeleton2dExternalRests(), &list, kOptional, "externalRests"},
        {&Skeleton2dExternalPoses(), &list, kOptional, "externalPoses"},
        {&Skeleton2dChannels(), &doubles, kRigExecPortInput, "channels"},
        {&Skeleton2dSelectors(), &doubles, kOptional, "selectors"},
        {&Skeleton2dGroupSelectors(), &doubles, kOptional, "groupSelectors"},
        {&Skeleton2dOut(), &rigExecRig::PointFrameArrayType(),
         kRigExecPortOutput, "out"},
        {&Skeleton2dResult(), &rigExecStd::UIntType(), kRigExecPortOutput,
         "result"},
    };
    static const rigExec::RigExecOpDesc desc = [] {
        rigExec::RigExecOpDesc d;
        d.nameSpace = "rig";
        d.name = "Skeleton2d";
        d.ports = ports;
        d.portCount = std::size(ports);
        d.run = &RunSkeleton2d;
        d.capabilities =
            rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
        d.payload = rigExec::RigExecPayloadHandlingFor<Skeleton2dPayload>();
        d.state.size = sizeof(Skeleton2dState);
        d.state.align = alignof(Skeleton2dState);
        d.state.create = [](void *dst) {
            std::uninitialized_value_construct_n(
                static_cast<Skeleton2dState *>(dst), 1);
        };
        d.state.destroy = [](void *dst) {
            std::destroy_at(static_cast<Skeleton2dState *>(dst));
        };
        return d;
    }();
    registrar.Add(Skeleton2d(), desc);
}

} // namespace rigExecOps
