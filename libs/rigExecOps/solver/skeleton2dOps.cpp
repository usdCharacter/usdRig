#include "rigExecOps/solver/skeleton2dOps.h"

#include "rigExecOps/opIo.h"
#include "rigExecOps/payloadIo.h"
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

namespace {

using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::RigExecArray;
using Landmarks = RigExecArray<PointLandmarks>;
using namespace opIo;

constexpr char kMalformed[] = "2D skeleton solve order is malformed";

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
    switch (result) {
    case rigExec::RigExecSkeleton2dResult::Solved:
        break;
    case rigExec::RigExecSkeleton2dResult::Malformed:
        return Fail(error, "Skeleton2d", kMalformed);
    case rigExec::RigExecSkeleton2dResult::NotPlanar:
        return Fail(error, "Skeleton2d",
                    "a 2D bone's rest (its rest:space, or the frame a pose "
                    "step below the skeleton wrote) or parent space is not a "
                    "rotation about +Z in the skeleton's plane");
    case rigExec::RigExecSkeleton2dResult::NotFinite:
        return Fail(error, "Skeleton2d",
                    "2D skeleton solve failed (a non-finite channel)");
    }
    PointFrameArray &out = Out<PointFrameArray>(io, Skeleton2dOut());
    out.frames.Assign(s.frames.data(), s.frames.size());
    out.rests = rests;
    return true;
}

bool
CheckSkeleton2d(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const auto &p = *static_cast<const Skeleton2dPayload *>(
        rigExec::RigExecBatchPayload(check.batch, check.k));
    const size_t n = p.order.size();
    if (p.setup.size() != n)
        return Fail(error, "Skeleton2d", kMalformed);
    for (const uint32_t joint : p.order)
        if (size_t(joint) >= n)
            return Fail(error, "Skeleton2d", kMalformed);
    if (Fed(check, Skeleton2dStartRest()) != Fed(check, Skeleton2dStartPose()))
        return Fail(error, "Skeleton2d",
                    "needs the start rest and pose together");
    if (Fed(check, Skeleton2dExternalRests()) !=
            Fed(check, Skeleton2dExternalPoses()) ||
        !SameCount(CheckElements(check, Skeleton2dExternalRests()),
                   CheckElements(check, Skeleton2dExternalPoses())))
        return Fail(error, "Skeleton2d",
                    "needs one posed frame per outside parent space");
    if (!SameCount(CheckElements(check, Skeleton2dRests()), n))
        return Fail(error, "Skeleton2d", kMalformed);
    if (!SameCount(CheckElements(check, Skeleton2dChannels()),
                   n * rigExec::kRigExecBone2dChannelCount))
        return Fail(error, "Skeleton2d", "needs every channel of every joint");
    return true;
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kPorts[] = {
    {&Skeleton2dRests(), &rigExecRig::PointLandmarksList(), kRigExecPortInput,
     "rests"},
    {&Skeleton2dStartRest(), &rigExecRig::PointLandmarksType(), kOptional,
     "startRest"},
    {&Skeleton2dStartPose(), &rigExecRig::PointLandmarksType(), kOptional,
     "startPose"},
    {&Skeleton2dExternalRests(), &rigExecRig::PointLandmarksList(), kOptional,
     "externalRests"},
    {&Skeleton2dExternalPoses(), &rigExecRig::PointLandmarksList(), kOptional,
     "externalPoses"},
    {&Skeleton2dChannels(), &rigExecStd::DoubleArray(), kRigExecPortInput,
     "channels"},
    {&Skeleton2dSelectors(), &rigExecStd::DoubleArray(), kOptional,
     "selectors"},
    {&Skeleton2dGroupSelectors(), &rigExecStd::DoubleArray(), kOptional,
     "groupSelectors"},
    {&Skeleton2dOut(), &rigExecRig::PointFrameArrayType(), kRigExecPortOutput,
     "out", rigExec::RigExecOutputBound::Input, &Skeleton2dRests()},
};

using payloadIo::Reader;
using payloadIo::Writer;

// The saved size of one RigExecBone2dSetup and one RigExecJointGroup.
constexpr size_t kBoneBytes = 125;
constexpr size_t kGroupBytes = 9;

void
PutBone(Writer &w, const rigExec::RigExecBone2dSetup &b)
{
    w.I32(b.parent);
    w.U8(b.flags);
    w.F64(b.minAngle);
    w.F64(b.maxAngle);
    w.Enum(b.scaling);
    w.F64(b.squashFactor);
    w.F64(b.maxStretch);
    w.I32(b.tipBone);
    w.F64(b.tipX);
    w.F64(b.tipY);
    w.U32(b.spaceBegin);
    w.U32(b.spaceCount);
    w.I32(b.spaceSelector);
    w.U32(b.targetBegin);
    w.U32(b.targetCount);
    w.I32(b.angleController);
    w.I32(b.posController);
    w.I32(b.scaleController);
    w.F64(b.angleFactor);
    w.F64(b.posFactorX);
    w.F64(b.posFactorY);
    w.F64(b.scaleFactor);
}

void
GetBone(Reader &r, rigExec::RigExecBone2dSetup &b)
{
    b.parent = r.I32();
    b.flags = r.U8();
    b.minAngle = r.F64();
    b.maxAngle = r.F64();
    b.scaling = r.Enum(rigExec::RigExecBone2dScaling::Uniform);
    b.squashFactor = r.F64();
    b.maxStretch = r.F64();
    b.tipBone = r.I32();
    b.tipX = r.F64();
    b.tipY = r.F64();
    b.spaceBegin = r.U32();
    b.spaceCount = r.U32();
    b.spaceSelector = r.I32();
    b.targetBegin = r.U32();
    b.targetCount = r.U32();
    b.angleController = r.I32();
    b.posController = r.I32();
    b.scaleController = r.I32();
    b.angleFactor = r.F64();
    b.posFactorX = r.F64();
    b.posFactorY = r.F64();
    b.scaleFactor = r.F64();
}

void
PutGroup(Writer &w, const rigExec::RigExecJointGroup &g)
{
    w.U32(g.memberBegin);
    w.U32(g.memberCount);
    w.Bool(g.enabled);
}

void
GetGroup(Reader &r, rigExec::RigExecJointGroup &g)
{
    g.memberBegin = r.U32();
    g.memberCount = r.U32();
    g.enabled = r.Bool();
}

void
Save(Writer &w, const Skeleton2dPayload &p)
{
    w.Version(1);
    payloadIo::PutList(w, p.order);
    w.List(p.setup, &PutBone);
    payloadIo::PutList(w, p.spaceSources);
    payloadIo::PutList(w, p.targetCandidates);
    w.List(p.groups, &PutGroup);
    payloadIo::PutList(w, p.groupMembers);
}

void
Load(Reader &r, Skeleton2dPayload &p)
{
    r.Version(1);
    payloadIo::GetList(r, p.order);
    r.List(p.setup, kBoneBytes, &GetBone);
    payloadIo::GetList(r, p.spaceSources);
    payloadIo::GetList(r, p.targetCandidates);
    r.List(p.groups, kGroupBytes, &GetGroup);
    payloadIo::GetList(r, p.groupMembers);
}

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "Skeleton2d";
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.run = &RunSkeleton2d;
    d.validate = &CheckSkeleton2d;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payload =
        rigExec::RigExecPayloadHandlingFor<Skeleton2dPayload, &Save, &Load>();
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

} // namespace

void
RegisterSkeleton2dOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(Skeleton2d(), kDesc);
}

} // namespace rigExecOps
