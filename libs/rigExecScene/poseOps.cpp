#include "rigExecScene/poseOps.h"

#include "rigExec/api/types.h"
#include "rigExecOps/constraint/constraintOps.h"
#include "rigExecOps/constraint/harmonicsOps.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecOps/solver/chainSolverOps.h"
#include "rigExecOps/solver/frameBlendOps.h"
#include "rigExecOps/solver/ribbonFramesOps.h"
#include "rigExecOps/solver/singleChainIkOps.h"
#include "rigExecOps/solver/skeleton2dOps.h"
#include "rigExecOps/constraint/autoClavicleOps.h"
#include "rigExecOps/solver/splineIkOps.h"
#include "rigExecRigTypes/rigTypes.h"

#include "pxr/base/gf/math.h"

#include <algorithm>
#include <cstring>

namespace rigExec {

struct RigExecPoseOpScratch::Values {
    rigExecRig::PointFrameArray frames;
    rigExecStd::RigExecArray<rigExecRig::PointLandmarks> landmarks, jointRests;
    rigExecStd::RigExecArray<rigExecRig::PointFrame> frameList;
    rigExecStd::RigExecArray<double> doubles;
    rigExecStd::RigExecArray<uint8_t> live;
    rigExecStd::RigExecArray<rigExecStd::Vec3f> posed, rest;
    rigExecStd::RigExecArray<rigExecStd::Mat4d> matrices;
};

RigExecPoseOpScratch::RigExecPoseOpScratch()
    : values(std::make_unique<Values>())
{
}
RigExecPoseOpScratch::~RigExecPoseOpScratch() = default;
RigExecPoseOpScratch::RigExecPoseOpScratch(RigExecPoseOpScratch &&) noexcept =
    default;
RigExecPoseOpScratch &
RigExecPoseOpScratch::operator=(RigExecPoseOpScratch &&) noexcept = default;

namespace {

using rigExecRig::PointFrame;
using rigExecRig::PointFrameArray;
using rigExecRig::PointLandmarks;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3f;

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

Vec3d
ToStd(const GfVec3d &v)
{
    return Vec3d(v[0], v[1], v[2]);
}

PointLandmarks
ToStd(const std::array<GfVec3d, 4> &points)
{
    PointLandmarks out;
    for (size_t k = 0; k < 4; ++k)
        out[k] = ToStd(points[k]);
    return out;
}

PointFrame
ToStd(const RigExecPointFrame &frame)
{
    PointFrame out;
    out.points = ToStd(frame.points);
    out.flags = frame.flags;
    return out;
}

Mat4d
ToStd(const GfMatrix4d &m)
{
    Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out._mtx[r][c] = m[r][c];
    return out;
}

std::array<GfVec3d, 4>
FromStd(const PointLandmarks &points)
{
    std::array<GfVec3d, 4> out;
    for (size_t k = 0; k < 4; ++k)
        out[k] = GfVec3d(points[k][0], points[k][1], points[k][2]);
    return out;
}

RigExecPointFrame
FromStd(const PointFrame &frame)
{
    RigExecPointFrame out;
    out.points = FromStd(frame.points);
    out.flags = frame.flags;
    return out;
}

/// \p output's frames and rests from an op's frame array, from \p first on.
void
FromStd(const PointFrameArray &a, size_t first, RigExecPointFrameArray *output)
{
    output->frames.clear();
    output->rests.clear();
    for (size_t k = first; k < a.frames.Size(); ++k)
        output->frames.push_back(FromStd(a.frames[k]));
    for (size_t k = first; k < a.rests.Size(); ++k)
        output->rests.push_back(FromStd(a.rests[k]));
}

/// Bitwise, so a NaN setting matches itself.
template <class T>
bool
Same(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

template <class T>
bool
SameVector(const std::vector<T> &a, const std::vector<T> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
}

bool
SameAxes(const rigExecOps::ConstraintAxes &a,
         const rigExecOps::ConstraintAxes &b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

rigExecOps::ConstraintAxes
Axes(const RigExecConstraintAxisMask &m)
{
    return {m.x, m.y, m.z};
}

rigExecOps::ConstraintEulerOrder
Order(RigExecEulerOrder order)
{
    return rigExecOps::ConstraintEulerOrder(uint8_t(order));
}

void
Landmarks(const std::vector<std::array<GfVec3d, 4>> &in,
          RigExecArray<PointLandmarks> *out)
{
    out->Resize(in.size());
    for (size_t k = 0; k < in.size(); ++k)
        (*out)[k] = ToStd(in[k]);
}

template <class T>
void
Assign(const std::vector<T> &in, RigExecArray<T> *out)
{
    out->Resize(in.size());
    if (!in.empty())
        std::memcpy(out->Data(), in.data(), in.size() * sizeof(T));
}

template <class T>
bool
Set(RigExecEvaluator *e, const RigExecInputId &id, const T &value,
    std::string *error)
{
    return e->SetInput(id, &value, error);
}

// Solvers.

rigExecOps::FkChainPayload
FkPayload(const RigExecSolverRecord &s)
{
    rigExecOps::FkChainPayload p;
    const int base = s.hasStart ? 1 : 0;
    if (base)
        p.parentIndices.push_back(-1);
    for (size_t k = 0; k < s.controlRests.size(); ++k)
        p.parentIndices.push_back(s.parentRelative ? base - 1
                                                   : int(k) + base - 1);
    p.scaleSegments = s.scaleSegments;
    return p;
}

rigExecOps::FkJointGroupsPayload
FkGroupsPayload(const RigExecSolverRecord &s)
{
    rigExecOps::FkJointGroupsPayload p;
    p.jointBegin = s.hasStart ? 1 : 0;
    p.parents = s.jointGroupParents;
    p.groups = s.jointGroups;
    p.members = s.jointGroupMembers;
    return p;
}

rigExecOps::TwoBoneIkPayload
IkPayload(const RigExecSolverRecord &s)
{
    rigExecOps::TwoBoneIkPayload p;
    p.softDistancePolicy = s.ikParams.softDistancePolicy;
    p.scaleSegments = s.ikParams.scaleSegments;
    p.scaleCalibration = s.ikParams.limb.scaleCalibration;
    return p;
}

rigExecOps::SplineIkPayload
SplinePayload(const RigExecSolverRecord &s)
{
    rigExecOps::SplineIkPayload p;
    p.restLengthChain = s.splineRestMode == RigExecSplineIkRestLength::Chain;
    p.aimRootTangent = s.splineParams.aimRootTangent;
    return p;
}

/// Per blend element, whether a live joint rest is the out rest.
std::vector<char>
BlendLive(const RigExecSolverRecord &s, size_t n)
{
    std::vector<char> live(n, 0);
    for (size_t k = 0; k < n; ++k)
        live[k] = k < s.restIsLive.size() && s.restIsLive[k] &&
                  k < s.jointRests.size();
    return live;
}

/// The live-rest re-basing every twist and ribbon frame goes through.
bool
WriteLive(const RigExecSolverRecord &s, const RigExecSolverOpNode &node,
          RigExecPoseOpScratch *scratch, RigExecEvaluator *e,
          std::string *error)
{
    RigExecPoseOpScratch::Values &v = *scratch->values;
    Landmarks(s.jointRests, &v.jointRests);
    v.live.Resize(s.restIsLive.size());
    for (size_t k = 0; k < s.restIsLive.size(); ++k)
        v.live[k] = s.restIsLive[k] ? 1 : 0;
    return Set(e, node.jointRests, v.jointRests, error) &&
           Set(e, node.jointRestLive, v.live, error);
}

bool
SameSetup(const RigExecBone2dSetup &a, const RigExecBone2dSetup &b)
{
    return a.parent == b.parent && a.flags == b.flags &&
           Same(a.minAngle, b.minAngle) && Same(a.maxAngle, b.maxAngle) &&
           a.scaling == b.scaling && Same(a.squashFactor, b.squashFactor) &&
           Same(a.maxStretch, b.maxStretch) && a.tipBone == b.tipBone &&
           Same(a.tipX, b.tipX) && Same(a.tipY, b.tipY) &&
           a.spaceBegin == b.spaceBegin && a.spaceCount == b.spaceCount &&
           a.spaceSelector == b.spaceSelector &&
           a.targetBegin == b.targetBegin && a.targetCount == b.targetCount &&
           a.angleController == b.angleController &&
           a.posController == b.posController &&
           a.scaleController == b.scaleController &&
           Same(a.angleFactor, b.angleFactor) &&
           Same(a.posFactorX, b.posFactorX) &&
           Same(a.posFactorY, b.posFactorY) &&
           Same(a.scaleFactor, b.scaleFactor);
}

/// Whether \p node was built from \p s's skeleton.
bool
SameSkeleton(const RigExecSolverRecord &s, const RigExecSolverOpNode &node)
{
    if (!SameVector(s.bone2dOrder, node.bone2dOrder) ||
        !SameVector(s.bone2dSpaceSources, node.spaceSources) ||
        !SameVector(s.bone2dTargetCandidates, node.targetCandidates) ||
        !SameVector(s.jointGroupMembers, node.groupMembers) ||
        s.bone2dSetup.size() != node.bone2dSetup.size() ||
        s.jointGroups.size() != node.groups.size())
        return false;
    for (size_t k = 0; k < s.bone2dSetup.size(); ++k)
        if (!SameSetup(s.bone2dSetup[k], node.bone2dSetup[k]))
            return false;
    for (size_t k = 0; k < s.jointGroups.size(); ++k) {
        const RigExecJointGroup &a = s.jointGroups[k], &b = node.groups[k];
        if (a.memberBegin != b.memberBegin || a.memberCount != b.memberCount ||
            a.enabled != b.enabled)
            return false;
    }
    return true;
}

// Constraints.

rigExecOps::ConstraintPayload
ConstraintPayloadOf(const RigExecConstraintRecord &r)
{
    rigExecOps::ConstraintPayload p;
    switch (r.kind) {
    case RigExecConstraintKind::Rotation:
        p.rotationOrder = Order(r.rotation.rotationOrder);
        break;
    case RigExecConstraintKind::Scale:
        p.blendShear = r.scale.blendShear;
        break;
    case RigExecConstraintKind::Parent:
        p.rotationOrder = Order(r.parent.rotationOrder);
        p.blendShear = r.parent.blendShear;
        break;
    case RigExecConstraintKind::Aim:
        p.rotationOrder = Order(r.aim.rotationOrder);
        p.preserveInputUp = r.aim.preserveInputUp;
        break;
    default:
        break;
    }
    return p;
}

RigExecArray<uint8_t>
Flags(const RigExecConstraintAxisMask &m)
{
    RigExecArray<uint8_t> out(3);
    out[0] = uint8_t(m.x);
    out[1] = uint8_t(m.y);
    out[2] = uint8_t(m.z);
    return out;
}

/// The Offset and Affect a Position, Rotation, Scale or Aim record feeds.
void
OffsetAndAffect(const RigExecConstraintRecord &r, Vec3d *offset,
                RigExecArray<uint8_t> *affect)
{
    switch (r.kind) {
    case RigExecConstraintKind::Position:
        *offset = ToStd(r.position.offset);
        *affect = Flags(r.position.affect);
        return;
    case RigExecConstraintKind::Rotation:
        *offset = ToStd(r.rotation.offsetDegrees);
        *affect = Flags(r.rotation.affect);
        return;
    case RigExecConstraintKind::Scale:
        *offset = ToStd(r.scale.offset);
        *affect = Flags(r.scale.affect);
        return;
    default:
        *offset = ToStd(r.aim.rotationOffsetDegrees);
        *affect = Flags(r.aim.affectRotation);
        return;
    }
}

/// Whether \p kind reads the carry.
bool
TakesSpace(RigExecConstraintKind kind)
{
    return kind == RigExecConstraintKind::Rotation ||
           kind == RigExecConstraintKind::Parent;
}

double
ConstraintWeight(const RigExecConstraintRecord &r)
{
    switch (r.kind) {
    case RigExecConstraintKind::Position:
        return r.position.weight;
    case RigExecConstraintKind::Rotation:
        return r.rotation.weight;
    case RigExecConstraintKind::Scale:
        return r.scale.weight;
    case RigExecConstraintKind::Parent:
        return r.parent.weight;
    case RigExecConstraintKind::Aim:
        return r.aim.weight;
    default:
        return r.singleChain.weight;
    }
}

} // namespace

bool
RigExecSolverOpSupported(const RigExecSolverRecord &s)
{
    if (s.degenerate)
        return false;
    switch (s.kind) {
    case RigExecSolverKind::FkChain:
    case RigExecSolverKind::TwoBoneIk:
    case RigExecSolverKind::TwistDistribution:
    case RigExecSolverKind::Ribbon:
        return true;
    case RigExecSolverKind::BlendPointFrames:
        return !s.blendRotationRejected;
    case RigExecSolverKind::SplineIk:
        return s.splineCount > 0;
    case RigExecSolverKind::Skeleton2d:
        return true;
    }
    return false;
}

RigExecSolverOpAnswer
RigExecAnswerSolverOp(const RigExecSolverRecord &s,
                      const RigExecSolverInputs &input,
                      RigExecPointFrameArray *output, std::string *error)
{
    output->frames.clear();
    output->rests.clear();
    if (s.kind == RigExecSolverKind::FkChain &&
        input.controls.size() != s.controlRests.size()) {
        Fail(error, "FK control/rest cardinality mismatch");
        return RigExecSolverOpAnswer::Failed;
    }
    if (s.kind == RigExecSolverKind::Skeleton2d) {
        const size_t n = s.bone2dOrder.size();
        if (s.jointRests.size() != n || s.bone2dSetup.size() != n ||
            input.bone2dChannels.size() != n ||
            input.controls.size() != s.controlRests.size()) {
            Fail(error, kRigExecSkeleton2dCountsDiffer);
            return RigExecSolverOpAnswer::Failed;
        }
    }
    if (s.kind != RigExecSolverKind::BlendPointFrames)
        return RigExecSolverOpAnswer::Run;
    const auto *a = input.blendA, *b = input.blendB;
    if (!a || !b) {
        if (a || b)
            *output = a ? *a : *b;
        return RigExecSolverOpAnswer::Done;
    }
    const size_t n = a->GetSize();
    if (n == 0)
        return RigExecSolverOpAnswer::Done;
    if (n != b->GetSize() || a->rests.size() != n) {
        Fail(error, "frame blend needs one frame and rest per joint on both "
                    "inputs");
        return RigExecSolverOpAnswer::Failed;
    }
    if (n != s.frameCount) {
        Fail(error, "frame blend inputs differ from its Build count");
        return RigExecSolverOpAnswer::Failed;
    }
    return RigExecSolverOpAnswer::Run;
}

bool
RigExecAddSolverOp(const RigExecSolverRecord &s,
                   const RigExecSolverInputs &input,
                   RigExecRigDescBuilder *builder, RigExecSolverOpNode *node,
                   std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null solver op destination");
    if (!RigExecSolverOpSupported(s))
        return Fail(error, "solver kind has no op");
    RigExecSolverOpNode r;
    r.kind = s.kind;
    RigExecRigDescBuilder &b = *builder;
    switch (s.kind) {
    case RigExecSolverKind::FkChain: {
        const rigExecOps::FkChainPayload payload = FkPayload(s);
        r.parents = payload.parentIndices;
        r.scaleSegments = payload.scaleSegments;
        const RigExecNodeId id = b.AddOp(rigExecOps::FkChain(), &payload);
        r.controls = b.BindInput(id, rigExecOps::FkChainControls());
        r.outRests = b.BindInput(id, rigExecOps::FkChainOutRests());
        if (s.jointGroups.empty()) {
            r.out = b.BindOutput(id, rigExecOps::FkChainOut());
            break;
        }
        const rigExecOps::FkJointGroupsPayload groups = FkGroupsPayload(s);
        r.groups = groups.groups;
        r.groupMembers = groups.members;
        r.groupParents = groups.parents;
        r.hasStart = s.hasStart;
        using namespace rigExecOps;
        const RigExecNodeId g = b.AddOp(FkJointGroups(), &groups);
        b.Connect(id, FkChainOut(), g, FkJointGroupsIn());
        r.groupSelectors = b.BindInput(g, FkJointGroupsSelectors());
        if (s.hasStart) {
            r.start = b.BindInput(g, FkJointGroupsStart());
            r.startRest = b.BindInput(g, FkJointGroupsStartRest());
        }
        r.out = b.BindOutput(g, FkJointGroupsOut());
        break;
    }
    case RigExecSolverKind::TwoBoneIk: {
        const rigExecOps::TwoBoneIkPayload payload = IkPayload(s);
        r.softDistancePolicy = payload.softDistancePolicy;
        r.scaleSegments = payload.scaleSegments;
        r.scaleCalibration = payload.scaleCalibration;
        const RigExecNodeId id = b.AddOp(rigExecOps::TwoBoneIk(), &payload);
        using namespace rigExecOps;
        r.root = b.BindInput(id, TwoBoneIkRoot());
        r.effector = b.BindInput(id, TwoBoneIkEffector());
        r.pole = b.BindInput(id, TwoBoneIkPole());
        r.rests = b.BindInput(id, TwoBoneIkRests());
        r.space = b.BindInput(id, TwoBoneIkSpace());
        r.bend = b.BindInput(id, TwoBoneIkBend());
        r.upperOffset = b.BindInput(id, TwoBoneIkUpperOffset());
        r.lowerOffset = b.BindInput(id, TwoBoneIkLowerOffset());
        r.stretch = b.BindInput(id, TwoBoneIkStretch());
        r.softness = b.BindInput(id, TwoBoneIkSoftness());
        r.pin = b.BindInput(id, TwoBoneIkPin());
        r.upperScale = b.BindInput(id, TwoBoneIkUpperScale());
        r.lowerScale = b.BindInput(id, TwoBoneIkLowerScale());
        r.softDistance = b.BindInput(id, TwoBoneIkSoftDistance());
        r.twist = b.BindInput(id, TwoBoneIkTwist());
        r.out = b.BindOutput(id, TwoBoneIkOut());
        break;
    }
    case RigExecSolverKind::BlendPointFrames: {
        if (!input.blendA)
            return Fail(error, "frame blend op needs its first input");
        r.logScale = s.scaleMode == RigExecScaleBlend::Log;
        r.blendLive = BlendLive(s, s.frameCount);
        rigExecOps::BlendFramesPayload payload;
        payload.logScale = r.logScale;
        for (const char live : r.blendLive) {
            using namespace rigExecOps;
            const RigExecNodeId id = b.AddOp(BlendFrames(), &payload);
            RigExecSolverOpNode::Blend e;
            e.a = b.BindInput(id, BlendFramesA());
            e.b = b.BindInput(id, BlendFramesB());
            e.rest = b.BindInput(id, BlendFramesRest());
            e.weight = b.BindInput(id, BlendFramesWeight());
            if (live)
                e.outRest = b.BindInput(id, BlendFramesOutRest());
            e.out = b.BindOutput(id, BlendFramesOut());
            r.blend.push_back(e);
        }
        break;
    }
    case RigExecSolverKind::TwistDistribution: {
        using namespace rigExecOps;
        const RigExecNodeId id = b.AddOp(DistributeTwist());
        r.start = b.BindInput(id, DistributeTwistStart());
        r.end = b.BindInput(id, DistributeTwistEnd());
        r.startRest = b.BindInput(id, DistributeTwistStartRest());
        r.endRest = b.BindInput(id, DistributeTwistEndRest());
        r.weights = b.BindInput(id, DistributeTwistWeights());
        r.turns = b.BindInput(id, DistributeTwistTurns());
        r.jointRests = b.BindInput(id, DistributeTwistJointRests());
        r.jointRestLive = b.BindInput(id, DistributeTwistJointRestLive());
        r.out = b.BindOutput(id, DistributeTwistOut());
        break;
    }
    case RigExecSolverKind::Ribbon: {
        using namespace rigExecOps;
        r.sampleCount = s.ribbonSamples;
        SampleRibbonFramesPayload payload;
        payload.sampleCount = r.sampleCount;
        const RigExecNodeId id = b.AddOp(SampleRibbonFrames(), &payload);
        r.posed = b.BindInput(id, SampleRibbonFramesPosed());
        r.restCurve = b.BindInput(id, SampleRibbonFramesRest());
        r.jointRests = b.BindInput(id, SampleRibbonFramesJointRests());
        r.jointRestLive = b.BindInput(id, SampleRibbonFramesJointRestLive());
        r.out = b.BindOutput(id, SampleRibbonFramesOut());
        break;
    }
    case RigExecSolverKind::SplineIk: {
        using namespace rigExecOps;
        const SplineIkPayload payload = SplinePayload(s);
        r.restLengthChain = payload.restLengthChain;
        r.aimRootTangent = payload.aimRootTangent;
        const RigExecNodeId id = b.AddOp(SplineIk(), &payload);
        r.restJoints = b.BindInput(id, SplineIkRestJoints());
        r.restRoot = b.BindInput(id, SplineIkRestRoot());
        r.restMid = b.BindInput(id, SplineIkRestMid());
        r.restEnd = b.BindInput(id, SplineIkRestEnd());
        r.root = b.BindInput(id, SplineIkRoot());
        r.mid = b.BindInput(id, SplineIkMid());
        r.end = b.BindInput(id, SplineIkEnd());
        r.volumeWeights = b.BindInput(id, SplineIkVolumeWeights());
        r.preserveVolume = b.BindInput(id, SplineIkPreserveVolume());
        r.midFollow = b.BindInput(id, SplineIkMidFollow());
        r.roll = b.BindInput(id, SplineIkRoll());
        r.twist = b.BindInput(id, SplineIkTwist());
        r.minLengthRatio = b.BindInput(id, SplineIkMinLengthRatio());
        r.out = b.BindOutput(id, SplineIkOut());
        break;
    }
    case RigExecSolverKind::Skeleton2d: {
        using namespace rigExecOps;
        Skeleton2dPayload payload;
        payload.order = s.bone2dOrder;
        payload.setup = s.bone2dSetup;
        payload.spaceSources = s.bone2dSpaceSources;
        payload.targetCandidates = s.bone2dTargetCandidates;
        payload.groups = s.jointGroups;
        payload.groupMembers = s.jointGroupMembers;
        r.hasStart = s.hasStart;
        r.bone2dOrder = payload.order;
        r.bone2dSetup = payload.setup;
        r.spaceSources = payload.spaceSources;
        r.targetCandidates = payload.targetCandidates;
        r.groups = payload.groups;
        r.groupMembers = payload.groupMembers;
        const RigExecNodeId id = b.AddOp(Skeleton2d(), &payload);
        r.rests = b.BindInput(id, Skeleton2dRests());
        if (r.hasStart) {
            r.startRest = b.BindInput(id, Skeleton2dStartRest());
            r.startPose = b.BindInput(id, Skeleton2dStartPose());
        }
        r.externalRests = b.BindInput(id, Skeleton2dExternalRests());
        r.externalPoses = b.BindInput(id, Skeleton2dExternalPoses());
        r.channels = b.BindInput(id, Skeleton2dChannels());
        r.selectors = b.BindInput(id, Skeleton2dSelectors());
        r.groupSelectors = b.BindInput(id, Skeleton2dGroupSelectors());
        r.out = b.BindOutput(id, Skeleton2dOut());
        break;
    }
    }
    *node = std::move(r);
    return true;
}

bool
RigExecWriteSolverOpInputs(const RigExecSolverRecord &s,
                           const RigExecSolverInputs &input,
                           const RigExecSolverOpNode &node,
                           RigExecPoseOpScratch *scratch, RigExecEvaluator *e,
                           std::string *error)
{
    if (!e || !scratch || !scratch->values || node.kind != s.kind)
        return Fail(error, "solver op node does not match the record");
    RigExecPoseOpScratch::Values &v = *scratch->values;
    switch (s.kind) {
    case RigExecSolverKind::FkChain: {
        const rigExecOps::FkChainPayload payload = FkPayload(s);
        if (!SameVector(payload.parentIndices, node.parents) ||
            payload.scaleSegments != node.scaleSegments)
            return Fail(error, "FK chain settings moved");
        // The start frame leads as a root control; its output is dropped.
        const size_t base = s.hasStart ? 1 : 0, n = input.controls.size();
        const bool jointBasis = s.jointRests.size() == n;
        PointFrameArray &controls = v.frames;
        controls.frames.Resize(n + base);
        controls.rests.Resize(n + base);
        v.landmarks.Resize(n + base);
        if (base) {
            controls.frames[0] = ToStd(input.start);
            controls.rests[0] = ToStd(s.startRest);
            v.landmarks[0] = controls.rests[0];
        }
        for (size_t k = 0; k < n; ++k) {
            controls.frames[k + base] = ToStd(input.controls[k]);
            controls.rests[k + base] = ToStd(s.controlRests[k]);
            const bool live =
                jointBasis && k < s.restIsLive.size() && s.restIsLive[k];
            v.landmarks[k + base] =
                live ? ToStd(s.jointRests[k]) : controls.rests[k + base];
        }
        if (!Set(e, node.controls, controls, error) ||
            !Set(e, node.outRests, v.landmarks, error))
            return false;
        if (s.jointGroups.empty() != node.groups.empty())
            return Fail(error, "FK chain settings moved");
        if (s.jointGroups.empty())
            return true;
        const rigExecOps::FkJointGroupsPayload groups = FkGroupsPayload(s);
        if (!(groups.groups == node.groups) ||
            !SameVector(groups.members, node.groupMembers) ||
            !SameVector(groups.parents, node.groupParents) ||
            s.hasStart != node.hasStart)
            return Fail(error, "FK joint groups moved");
        Assign(input.jointGroupSelectors, &v.doubles);
        if (!Set(e, node.groupSelectors, v.doubles, error))
            return false;
        if (!s.hasStart)
            return true;
        const PointFrame start = ToStd(input.start);
        const auto startRest = ToStd(s.startRest);
        return Set(e, node.start, start, error) &&
               Set(e, node.startRest, startRest, error);
    }
    case RigExecSolverKind::TwoBoneIk: {
        const rigExecOps::TwoBoneIkPayload payload = IkPayload(s);
        if (!input.refreshIkParams ||
            payload.softDistancePolicy != node.softDistancePolicy ||
            payload.scaleSegments != node.scaleSegments ||
            !Same(payload.scaleCalibration, node.scaleCalibration))
            return Fail(error, "two-bone IK settings moved");
        GfMatrix4d space = input.ikSpace;
        if (input.hasSpace) {
            GfMatrix4d delta(1.0);
            if (RigExecPointsToMatrix(input.spaceRest, input.space, &delta))
                space = delta * space;
        }
        v.landmarks.Resize(3);
        for (size_t k = 0; k < 3; ++k)
            v.landmarks[k] = ToStd(s.ikRests[k]);
        return Set(e, node.root, ToStd(input.root), error) &&
               Set(e, node.effector, ToStd(input.end), error) &&
               Set(e, node.pole, ToStd(input.pole), error) &&
               Set(e, node.rests, v.landmarks, error) &&
               Set(e, node.space, ToStd(space), error) &&
               Set(e, node.bend, input.bend, error) &&
               Set(e, node.upperOffset, input.upperOffset, error) &&
               Set(e, node.lowerOffset, input.lowerOffset, error) &&
               Set(e, node.stretch, input.stretch, error) &&
               Set(e, node.softness, input.softness, error) &&
               Set(e, node.pin, input.pin, error) &&
               Set(e, node.upperScale, input.upperScale, error) &&
               Set(e, node.lowerScale, input.lowerScale, error) &&
               Set(e, node.softDistance, input.softDistance, error) &&
               Set(e, node.twist, input.limbTwist, error);
    }
    case RigExecSolverKind::BlendPointFrames: {
        const auto *a = input.blendA, *b = input.blendB;
        if (!a || !b)
            return Fail(error, "frame blend op needs both inputs");
        const size_t n = a->GetSize();
        if (node.logScale != (s.scaleMode == RigExecScaleBlend::Log) ||
            node.blendLive != BlendLive(s, s.frameCount))
            return Fail(error, "frame blend settings moved");
        if (n != node.blend.size())
            return Fail(error,
                        "frame blend inputs differ from its Build count");
        const double weight = std::min(std::max(input.blendWeight, 0.0), 1.0);
        for (size_t k = 0; k < n; ++k) {
            const RigExecSolverOpNode::Blend &x = node.blend[k];
            if (!Set(e, x.a, ToStd(a->frames[k]), error) ||
                !Set(e, x.b, ToStd(b->frames[k]), error) ||
                !Set(e, x.rest, ToStd(a->rests[k]), error) ||
                !Set(e, x.weight, weight, error) ||
                (node.blendLive[k] &&
                 !Set(e, x.outRest, ToStd(s.jointRests[k]), error)))
                return false;
        }
        return true;
    }
    case RigExecSolverKind::TwistDistribution: {
        Assign(s.twistWeights, &v.doubles);
        return Set(e, node.start, ToStd(input.root), error) &&
               Set(e, node.end, ToStd(input.end), error) &&
               Set(e, node.startRest, ToStd(s.twistStartRest), error) &&
               Set(e, node.endRest, ToStd(s.twistEndRest), error) &&
               Set(e, node.weights, v.doubles, error) &&
               Set(e, node.turns, input.twistTurns, error) &&
               WriteLive(s, node, scratch, e, error);
    }
    case RigExecSolverKind::Ribbon: {
        if (s.ribbonSamples != node.sampleCount)
            return Fail(error, "ribbon sample count moved");
        const auto points = [](const std::vector<GfVec3f> &v,
                               RigExecArray<Vec3f> *out) {
            out->Resize(v.size());
            for (size_t i = 0; i < v.size(); ++i)
                (*out)[i] = Vec3f(v[i][0], v[i][1], v[i][2]);
        };
        points(input.ribbonPoints, &v.posed);
        points(s.ribbonRestPoints, &v.rest);
        return Set(e, node.posed, v.posed, error) &&
               Set(e, node.restCurve, v.rest, error) &&
               WriteLive(s, node, scratch, e, error);
    }
    case RigExecSolverKind::SplineIk: {
        const rigExecOps::SplineIkPayload payload = SplinePayload(s);
        const RigExecSplineIkParams params =
            RigExecSolverSplineParams(s, input);
        if (payload.restLengthChain != node.restLengthChain ||
            payload.aimRootTangent != node.aimRootTangent)
            return Fail(error, "spline IK settings moved");
        // The rests carried by the space's rest-to-pose map.
        GfMatrix4d delta(1.0);
        const bool spaced =
            input.hasSpace &&
            RigExecPointsToMatrix(input.spaceRest, input.space, &delta) &&
            delta != GfMatrix4d(1.0);
        const auto rest = [&](const RigExecPointFrame &frame) {
            return ToStd(spaced ? RigExecTransformFrame(frame, delta) : frame);
        };
        v.frameList.Resize(s.splineRestFrames.size());
        for (size_t k = 0; k < s.splineRestFrames.size(); ++k)
            v.frameList[k] = rest(s.splineRestFrames[k]);
        Assign(s.splineRestWeights, &v.doubles);
        return Set(e, node.restJoints, v.frameList, error) &&
               Set(e, node.restRoot, rest(s.splineRootRest), error) &&
               Set(e, node.restMid, rest(s.splineMidRest), error) &&
               Set(e, node.restEnd, rest(s.splineEndRest), error) &&
               Set(e, node.root, ToStd(input.root), error) &&
               Set(e, node.mid, ToStd(input.mid), error) &&
               Set(e, node.end, ToStd(input.end), error) &&
               Set(e, node.volumeWeights, v.doubles, error) &&
               Set(e, node.preserveVolume, params.preserveVolume, error) &&
               Set(e, node.midFollow, params.midFollowWeight, error) &&
               Set(e, node.roll, params.roll, error) &&
               Set(e, node.twist, params.twist, error) &&
               Set(e, node.minLengthRatio, params.minLengthRatio, error);
    }
    case RigExecSolverKind::Skeleton2d: {
        if (node.hasStart != s.hasStart || !SameSkeleton(s, node))
            return Fail(error, "2D skeleton settings moved");
        Landmarks(s.jointRests, &v.landmarks);
        if (!Set(e, node.rests, v.landmarks, error))
            return false;
        if (s.hasStart &&
            (!Set(e, node.startRest, ToStd(s.startRest), error) ||
             !Set(e, node.startPose, ToStd(input.start.points), error)))
            return false;
        Landmarks(s.controlRests, &v.landmarks);
        v.jointRests.Resize(input.controls.size());
        for (size_t k = 0; k < input.controls.size(); ++k)
            v.jointRests[k] = ToStd(input.controls[k].points);
        if (!Set(e, node.externalRests, v.landmarks, error) ||
            !Set(e, node.externalPoses, v.jointRests, error))
            return false;
        const size_t channels = rigExec::kRigExecBone2dChannelCount;
        v.doubles.Resize(input.bone2dChannels.size() * channels);
        for (size_t j = 0; j < input.bone2dChannels.size(); ++j) {
            const RigExecBone2dChannels &c = input.bone2dChannels[j];
            double *x = v.doubles.Data() + j * channels;
            const double values[] = {c.tx,
                                     c.ty,
                                     c.rz,
                                     c.sx,
                                     double(c.sideFlip),
                                     double(c.endFlip),
                                     double(c.lock),
                                     c.lockX,
                                     c.lockY,
                                     c.lockAngle,
                                     c.offsetX,
                                     c.offsetY,
                                     c.targetIndex};
            std::memcpy(x, values, sizeof(values));
        }
        if (!Set(e, node.channels, v.doubles, error))
            return false;
        Assign(input.bone2dSelectors, &v.doubles);
        if (!Set(e, node.selectors, v.doubles, error))
            return false;
        Assign(input.jointGroupSelectors, &v.doubles);
        return Set(e, node.groupSelectors, v.doubles, error);
    }
    }
    return Fail(error, "solver kind has no op");
}

bool
RigExecReadSolverOpOutput(const RigExecSolverRecord &s,
                          const RigExecSolverInputs &input,
                          const RigExecSolverOpNode &node,
                          const RigExecEvaluator &e,
                          RigExecPointFrameArray *output)
{
    output->frames.clear();
    output->rests.clear();
    if (s.kind == RigExecSolverKind::BlendPointFrames) {
        const RigExecPointFrameArray &a = *input.blendA;
        for (size_t k = 0; k < node.blend.size(); ++k) {
            const auto *frame =
                static_cast<const PointFrame *>(e.GetOutput(node.blend[k].out));
            if (!frame)
                return false;
            output->frames.push_back(FromStd(*frame));
            output->rests.push_back(node.blendLive[k] ? s.jointRests[k]
                                                      : a.rests[k]);
        }
        return true;
    }
    const auto *out =
        static_cast<const PointFrameArray *>(e.GetOutput(node.out));
    if (!out)
        return false;
    FromStd(*out, s.kind == RigExecSolverKind::FkChain && s.hasStart ? 1 : 0,
            output);
    // Native publishes the unspaced joint rests.
    if (s.kind == RigExecSolverKind::SplineIk)
        output->rests = s.splineJointRests;
    return true;
}

bool
RigExecConstraintOpSupported(RigExecConstraintKind kind)
{
    switch (kind) {
    case RigExecConstraintKind::Position:
    case RigExecConstraintKind::Rotation:
    case RigExecConstraintKind::Scale:
    case RigExecConstraintKind::Parent:
    case RigExecConstraintKind::Aim:
    case RigExecConstraintKind::SingleChainIk:
        return true;
    case RigExecConstraintKind::IkFkMatch:
    case RigExecConstraintKind::Harmonics:
        return false;
    }
    return false;
}

bool
RigExecAddConstraintOp(const RigExecConstraintRecord &r,
                       const RigExecConstraintInputs &input,
                       RigExecRigDescBuilder *builder,
                       RigExecConstraintOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null constraint op destination");
    if (!RigExecConstraintOpSupported(r.kind))
        return Fail(error, "constraint kind has no op");
    RigExecRigDescBuilder &b = *builder;
    RigExecConstraintOpNode n;
    n.kind = r.kind;
    if (r.kind == RigExecConstraintKind::SingleChainIk) {
        using namespace rigExecOps;
        SingleChainIkPayload payload;
        payload.singleChain =
            r.singleChain.mode == RigExecSingleChainIkMode::SingleChain;
        payload.preserveJointOrientation =
            r.singleChain.preserveJointOrientation;
        n.settings = r;
        const RigExecNodeId id = b.AddOp(SingleChainIk(), &payload);
        n.chain = b.BindInput(id, SingleChainIkChain());
        n.effector = b.BindInput(id, SingleChainIkEffector());
        n.pole = b.BindInput(id, SingleChainIkPole());
        n.twist = b.BindInput(id, SingleChainIkTwist());
        n.weight = b.BindInput(id, SingleChainIkWeight());
        n.stretch = b.BindInput(id, SingleChainIkStretch());
        n.out = b.BindOutput(id, SingleChainIkOut());
        *node = std::move(n);
        return true;
    }
    n.settings = r;
    const rigExecOps::ConstraintPayload payload = ConstraintPayloadOf(r);
    n.hasSpace = TakesSpace(r.kind) && input.carry.has_value();
    n.hasWorldUp = r.kind == RigExecConstraintKind::Aim &&
                   r.aim.worldUpDirection.has_value();
    using namespace rigExecOps;
    const RigExecPortHandle *in = nullptr, *sources = nullptr,
                            *weights = nullptr, *weight = nullptr,
                            *space = nullptr, *offset = nullptr,
                            *affect = nullptr, *out = nullptr;
    const RigExecOpHandle *op = nullptr;
    switch (r.kind) {
    case RigExecConstraintKind::Position:
        op = &PositionConstraint();
        in = &PositionConstraintIn();
        sources = &PositionConstraintSources();
        weights = &PositionConstraintSourceWeights();
        weight = &PositionConstraintWeight();
        offset = &PositionConstraintOffset();
        affect = &PositionConstraintAffect();
        out = &PositionConstraintOut();
        break;
    case RigExecConstraintKind::Rotation:
        op = &RotationConstraint();
        in = &RotationConstraintIn();
        sources = &RotationConstraintSources();
        weights = &RotationConstraintSourceWeights();
        weight = &RotationConstraintWeight();
        space = &RotationConstraintSpace();
        offset = &RotationConstraintOffset();
        affect = &RotationConstraintAffect();
        out = &RotationConstraintOut();
        break;
    case RigExecConstraintKind::Scale:
        op = &ScaleConstraint();
        in = &ScaleConstraintIn();
        sources = &ScaleConstraintSources();
        weights = &ScaleConstraintSourceWeights();
        weight = &ScaleConstraintWeight();
        offset = &ScaleConstraintOffset();
        affect = &ScaleConstraintAffect();
        out = &ScaleConstraintOut();
        break;
    case RigExecConstraintKind::Parent:
        op = &ParentConstraint();
        in = &ParentConstraintIn();
        sources = &ParentConstraintSources();
        weights = &ParentConstraintSourceWeights();
        weight = &ParentConstraintWeight();
        space = &ParentConstraintSpace();
        out = &ParentConstraintOut();
        break;
    case RigExecConstraintKind::Aim:
        op = &AimConstraint();
        in = &AimConstraintIn();
        weight = &AimConstraintWeight();
        offset = &AimConstraintOffset();
        affect = &AimConstraintAffect();
        out = &AimConstraintOut();
        break;
    default:
        return Fail(error, "constraint kind has no op");
    }
    const RigExecNodeId id = b.AddOp(*op, &payload);
    n.in = b.BindInput(id, *in);
    n.weight = b.BindInput(id, *weight);
    if (sources) {
        n.sources = b.BindInput(id, *sources);
        n.sourceWeights = b.BindInput(id, *weights);
    }
    if (n.hasSpace)
        n.space = b.BindInput(id, *space);
    if (offset) {
        n.offset = b.BindInput(id, *offset);
        n.affect = b.BindInput(id, *affect);
    }
    if (r.kind == RigExecConstraintKind::Parent) {
        n.translationAxes = b.BindInput(id, ParentConstraintTranslationAxes());
        n.rotationAxes = b.BindInput(id, ParentConstraintRotationAxes());
        n.scaleAxes = b.BindInput(id, ParentConstraintScaleAxes());
        n.translationOffsets =
            b.BindInput(id, ParentConstraintTranslationOffsets());
        n.rotationOffsets = b.BindInput(id, ParentConstraintRotationOffsets());
    }
    if (r.kind == RigExecConstraintKind::Aim) {
        n.aimVector = b.BindInput(id, AimConstraintAimVector());
        n.upVector = b.BindInput(id, AimConstraintUpVector());
        n.target = b.BindInput(id, AimConstraintTarget());
        if (n.hasWorldUp)
            n.worldUp = b.BindInput(id, AimConstraintWorldUp());
    }
    n.out = b.BindOutput(id, *out);
    *node = std::move(n);
    return true;
}

bool
RigExecWriteConstraintOpInputs(const RigExecConstraintRecord &r,
                               const RigExecConstraintInputs &input,
                               const RigExecConstraintOpNode &node,
                               RigExecPoseOpScratch *scratch,
                               RigExecEvaluator *e, std::string *error)
{
    if (!e || !scratch || !scratch->values || node.kind != r.kind)
        return Fail(error, "constraint op node does not match the record");
    RigExecPoseOpScratch::Values &v = *scratch->values;
    if (r.kind == RigExecConstraintKind::SingleChainIk) {
        const RigExecSingleChainIkParams &p = r.singleChain;
        if (node.settings.singleChain.mode != p.mode ||
            node.settings.singleChain.preserveJointOrientation !=
                p.preserveJointOrientation)
            return Fail(error, "single-chain IK settings moved");
        PointFrameArray &chain = v.frames;
        chain.frames.Resize(input.chain.size());
        chain.rests.Resize(0);
        for (size_t k = 0; k < input.chain.size(); ++k)
            chain.frames[k] = ToStd(input.chain[k]);
        return Set(e, node.chain, chain, error) &&
               Set(e, node.effector, ToStd(input.effector), error) &&
               Set(e, node.pole, ToStd(p.pole), error) &&
               Set(e, node.twist, p.twistDegrees, error) &&
               Set(e, node.weight, p.weight, error) &&
               Set(e, node.stretch, p.stretch, error);
    }
    if (!(ConstraintPayloadOf(node.settings) == ConstraintPayloadOf(r)) ||
        node.hasSpace != (TakesSpace(r.kind) && input.carry.has_value()) ||
        node.hasWorldUp != (r.kind == RigExecConstraintKind::Aim &&
                            r.aim.worldUpDirection.has_value()))
        return Fail(error, "constraint settings moved");
    if (!Set(e, node.in, ToStd(input.incoming), error) ||
        !Set(e, node.weight, ConstraintWeight(r), error))
        return false;
    if (node.hasSpace && !Set(e, node.space, ToStd(*input.carry), error))
        return false;
    if (r.kind != RigExecConstraintKind::Parent) {
        Vec3d offset;
        RigExecArray<uint8_t> affect;
        OffsetAndAffect(r, &offset, &affect);
        if (!Set(e, node.offset, offset, error) ||
            !Set(e, node.affect, affect, error))
            return false;
    }
    if (r.kind == RigExecConstraintKind::Aim)
        return Set(e, node.aimVector, ToStd(r.aim.localAimVector), error) &&
               Set(e, node.upVector, ToStd(r.aim.localUpVector), error) &&
               Set(e, node.target, ToStd(input.aimTarget), error) &&
               (!node.hasWorldUp ||
                Set(e, node.worldUp, ToStd(*r.aim.worldUpDirection), error));
    PointFrameArray &sources = v.frames;
    sources.frames.Resize(input.sources.size());
    sources.rests.Resize(0);
    v.doubles.Resize(input.sources.size());
    for (size_t k = 0; k < input.sources.size(); ++k) {
        sources.frames[k] = ToStd(input.sources[k].frame);
        v.doubles[k] = input.sources[k].normalizedWeight;
    }
    if (!Set(e, node.sources, sources, error) ||
        !Set(e, node.sourceWeights, v.doubles, error))
        return false;
    if (r.kind != RigExecConstraintKind::Parent)
        return true;
    RigExecArray<Vec3d> translations(input.sources.size()),
        rotations(input.sources.size());
    for (size_t k = 0; k < input.sources.size(); ++k) {
        translations[k] = ToStd(input.sources[k].translationOffset);
        rotations[k] = ToStd(input.sources[k].rotationOffsetDegrees);
    }
    return Set(e, node.translationAxes, Flags(r.parent.translationAxes),
               error) &&
           Set(e, node.rotationAxes, Flags(r.parent.rotationAxes), error) &&
           Set(e, node.scaleAxes, Flags(r.parent.scaleAxes), error) &&
           Set(e, node.translationOffsets, translations, error) &&
           Set(e, node.rotationOffsets, rotations, error);
}

bool
RigExecReadConstraintOpOutput(const RigExecConstraintRecord &r,
                              const RigExecConstraintInputs &input,
                              const RigExecConstraintOpNode &node,
                              const RigExecEvaluator &e,
                              RigExecConstraintResult *result)
{
    result->chain.clear();
    result->frame = input.incoming;
    if (r.kind == RigExecConstraintKind::SingleChainIk) {
        const auto *out =
            static_cast<const PointFrameArray *>(e.GetOutput(node.out));
        if (!out)
            return false;
        for (const PointFrame &frame : out->frames)
            result->chain.push_back(FromStd(frame));
        return true;
    }
    const auto *out = static_cast<const PointFrame *>(e.GetOutput(node.out));
    if (!out)
        return false;
    result->frame = FromStd(*out);
    return true;
}

bool
RigExecAddAutoClavicleOp(const RigExecAutoClavicleConstants &constants,
                         const RigExecAutoClavicleOpInputs &input,
                         RigExecRigDescBuilder *builder,
                         RigExecAutoClavicleOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null auto clavicle op destination");
    using namespace rigExecOps;
    RigExecRigDescBuilder &b = *builder;
    RigExecAutoClavicleOpNode n;
    n.constants = constants;
    n.hasLimb = input.hasLimb;
    n.hasIkTarget = input.frames[8] != nullptr;
    n.hasPole = input.frames[9] != nullptr;
    AutoClavicleOpPayload payload;
    payload.constants = constants;
    payload.hasLimb = n.hasLimb;
    const RigExecNodeId id = b.AddOp(AutoClavicle(), &payload);
    n.target = b.BindInput(id, AutoClavicleTarget());
    n.pivot = b.BindInput(id, AutoClaviclePivot());
    n.anchor = b.BindInput(id, AutoClavicleAnchor());
    n.anchorDefault = b.BindInput(id, AutoClavicleAnchorDefault());
    n.fkPosed = b.BindInput(id, AutoClavicleFkPosed());
    n.fkDefault = b.BindInput(id, AutoClavicleFkDefault());
    if (n.hasIkTarget)
        n.ikTarget = b.BindInput(id, AutoClavicleIkTarget());
    if (n.hasPole)
        n.pole = b.BindInput(id, AutoClaviclePole());
    n.ikBlend = b.BindInput(id, AutoClavicleIkBlend());
    n.amount = b.BindInput(id, AutoClavicleAmount());
    if (n.hasLimb) {
        n.limbRests = b.BindInput(id, AutoClavicleLimbRests());
        n.stretch = b.BindInput(id, AutoClavicleStretch());
        n.pin = b.BindInput(id, AutoClaviclePin());
        n.upperScale = b.BindInput(id, AutoClavicleUpperScale());
        n.lowerScale = b.BindInput(id, AutoClavicleLowerScale());
        n.softDistance = b.BindInput(id, AutoClavicleSoftDistance());
        n.scaleCalibration = b.BindInput(id, AutoClavicleScaleCalibration());
        n.twist = b.BindInput(id, AutoClavicleTwist());
        n.upperOffset = b.BindInput(id, AutoClavicleUpperOffset());
        n.lowerOffset = b.BindInput(id, AutoClavicleLowerOffset());
    }
    n.delta = b.BindOutput(id, AutoClavicleDelta());
    n.out = b.BindOutput(id, AutoClavicleOut());
    *node = std::move(n);
    return true;
}

bool
RigExecWriteAutoClavicleOpInputs(const RigExecAutoClavicleConstants &c,
                                 const RigExecAutoClavicleOpInputs &input,
                                 const RigExecAutoClavicleOpNode &node,
                                 RigExecPoseOpScratch *scratch,
                                 RigExecEvaluator *e, std::string *error)
{
    if (!e || !scratch || !scratch->values)
        return Fail(error, "auto clavicle op node has no evaluator");
    const RigExecAutoClavicleConstants &n = node.constants;
    if (node.hasLimb != input.hasLimb ||
        node.hasIkTarget != (input.frames[8] != nullptr) ||
        node.hasPole != (input.frames[9] != nullptr) ||
        std::memcmp(c.basis, n.basis, sizeof(c.basis)) != 0 ||
        !Same(c.ikValue, n.ikValue) || !Same(c.gain, n.gain) ||
        c.kernel != n.kernel || c.normalize != n.normalize ||
        !SameVector(c.swings, n.swings) || !SameVector(c.widths, n.widths) ||
        !SameVector(c.gains, n.gains) || !SameVector(c.weights, n.weights))
        return Fail(error, "auto clavicle settings moved");
    for (size_t i = 0; i < 8; ++i)
        if (!input.frames[i])
            return Fail(error, "auto clavicle frame missing");
    RigExecPoseOpScratch::Values &v = *scratch->values;
    const auto frames = [&](size_t first) -> const RigExecArray<Mat4d> & {
        v.matrices.Resize(3);
        for (size_t k = 0; k < 3; ++k)
            v.matrices[k] = ToStd(*input.frames[first + k]);
        return v.matrices;
    };
    if (!Set(e, node.target, ToStd(*input.frames[0]), error) ||
        !Set(e, node.pivot, ToStd(*input.frames[1]), error) ||
        !Set(e, node.anchor, ToStd(*input.frames[2]), error) ||
        !Set(e, node.anchorDefault, ToStd(*input.frames[3]), error) ||
        !Set(e, node.fkPosed, ToStd(*input.frames[4]), error) ||
        !Set(e, node.fkDefault, frames(5), error) ||
        (node.hasIkTarget &&
         !Set(e, node.ikTarget, ToStd(*input.frames[8]), error)) ||
        (node.hasPole && !Set(e, node.pole, ToStd(*input.frames[9]), error)) ||
        !Set(e, node.ikBlend, input.ikBlend, error) ||
        !Set(e, node.amount, input.amount, error))
        return false;
    if (!node.hasLimb)
        return true;
    for (size_t i = 10; i < 13; ++i)
        if (!input.frames[i])
            return Fail(error, "auto clavicle limb rest missing");
    return Set(e, node.limbRests, frames(10), error) &&
           Set(e, node.stretch, input.stretch, error) &&
           Set(e, node.pin, input.pin, error) &&
           Set(e, node.upperScale, input.upperScale, error) &&
           Set(e, node.lowerScale, input.lowerScale, error) &&
           Set(e, node.softDistance, input.softDistance, error) &&
           Set(e, node.scaleCalibration, input.scaleCalibration, error) &&
           Set(e, node.twist, input.twistDegrees, error) &&
           Set(e, node.upperOffset, input.upperOffset, error) &&
           Set(e, node.lowerOffset, input.lowerOffset, error);
}

bool
RigExecReadAutoClavicleOpOutput(const RigExecAutoClavicleOpNode &node,
                                const RigExecEvaluator &e, double delta[3],
                                GfMatrix4d *shifted)
{
    const auto *d = static_cast<const Vec3d *>(e.GetOutput(node.delta));
    const auto *m = static_cast<const Mat4d *>(e.GetOutput(node.out));
    if (!d || !m)
        return false;
    for (int k = 0; k < 3; ++k)
        delta[k] = (*d)[k];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            (*shifted)[r][c] = m->_mtx[r][c];
    return true;
}

namespace {

bool
SameHarmonics(const RigExecHarmonicsOpConstants &a,
              const RigExecHarmonicsOpConstants &b)
{
    return Same(a.settings.waves, b.settings.waves) &&
           Same(a.settings.waveLength, b.settings.waveLength) &&
           Same(a.settings.frequency, b.settings.frequency) &&
           a.settings.ignoreFirstFrame == b.settings.ignoreFirstFrame &&
           a.matchVelocity == b.matchVelocity && a.frames == b.frames;
}

} // namespace

bool
RigExecAddHarmonicsOp(const RigExecHarmonicsOpConstants &constants,
                      const RigExecHarmonicsOpInputs &input,
                      RigExecRigDescBuilder *builder,
                      RigExecHarmonicsOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null harmonics op destination");
    using namespace rigExecOps;
    HarmonicsPayload payload;
    payload.settings = constants.settings;
    payload.matchVelocity = constants.matchVelocity;
    payload.hasSpace = input.space.has_value();
    payload.frames = constants.frames;
    RigExecRigDescBuilder &b = *builder;
    RigExecHarmonicsOpNode n;
    n.constants = constants;
    n.hasSpace = payload.hasSpace;
    const RigExecNodeId id = b.AddOp(Harmonics(), &payload);
    n.in = b.BindInput(id, HarmonicsIn());
    if (n.hasSpace)
        n.space = b.BindInput(id, HarmonicsSpace());
    n.enabled = b.BindInput(id, HarmonicsEnabled());
    n.weight = b.BindInput(id, HarmonicsWeight());
    n.amplitude = b.BindInput(id, HarmonicsAmplitude());
    n.decay = b.BindInput(id, HarmonicsDecay());
    n.termination = b.BindInput(id, HarmonicsTermination());
    n.axisAmplitude = b.BindInput(id, HarmonicsAxisAmplitude());
    n.out = b.BindOutput(id, HarmonicsOut());
    *node = std::move(n);
    return true;
}

bool
RigExecWriteHarmonicsOpInputs(const RigExecHarmonicsOpConstants &constants,
                              const RigExecHarmonicsOpInputs &input,
                              const RigExecHarmonicsOpNode &node,
                              RigExecEvaluator *e, std::string *error)
{
    if (!e)
        return Fail(error, "null harmonics op evaluator");
    if (!SameHarmonics(node.constants, constants) ||
        node.hasSpace != input.space.has_value())
        return Fail(error, "harmonics settings moved");
    const uint8_t enabled = input.enabled ? 1 : 0;
    return Set(e, node.in, ToStd(input.incoming), error) &&
           (!node.hasSpace || Set(e, node.space, ToStd(*input.space), error)) &&
           Set(e, node.enabled, enabled, error) &&
           Set(e, node.weight, input.weight, error) &&
           Set(e, node.amplitude, input.amplitude, error) &&
           Set(e, node.decay, input.decay, error) &&
           Set(e, node.termination, input.termination, error) &&
           Set(e, node.axisAmplitude, ToStd(input.axisAmplitude), error);
}

bool
RigExecReadHarmonicsOpOutput(const RigExecHarmonicsOpNode &node,
                             const RigExecEvaluator &e,
                             RigExecPointFrame *frame)
{
    const auto *out = static_cast<const PointFrame *>(e.GetOutput(node.out));
    if (!out)
        return false;
    *frame = FromStd(*out);
    return true;
}

namespace {

bool
SameIkFkMatch(const RigExecIkFkMatchOpConstants &a,
              const RigExecIkFkMatchOpConstants &b)
{
    const RigExecSpaceSwitchRecord &x = a.poleRecord, &y = b.poleRecord;
    return x.filters == y.filters && Same(x.twistAxis, y.twistAxis) &&
           x.affectTranslation == y.affectTranslation &&
           x.affectRotation == y.affectRotation &&
           x.affectScale == y.affectScale && a.poleHasCarry == b.poleHasCarry &&
           a.poleWorld == b.poleWorld &&
           a.poleSourceRides == b.poleSourceRides &&
           Same(a.ikValue, b.ikValue) &&
           Same(a.scaleCalibration, b.scaleCalibration) &&
           Same(a.upperLengthOffset, b.upperLengthOffset) &&
           Same(a.lowerLengthOffset, b.lowerLengthOffset) &&
           a.stretchLimb == b.stretchLimb;
}

void
PackChannels(const RigExecIkFkMatchChannels &ch, double *out)
{
    for (int k = 0; k < 3; ++k) {
        out[k] = ch.t[k];
        out[3 + k] = ch.s[k];
        out[6 + k] = ch.r[k];
        out[10 + k] = ch.sign[k];
        out[15 + k] = double(ch.order[k]);
    }
    out[9] = ch.spin;
    out[13] = ch.units;
    out[14] = 0.0;
}

GfMatrix4d
FromStd(const Mat4d &m)
{
    GfMatrix4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r][c] = m._mtx[r][c];
    return out;
}

} // namespace

bool
RigExecAddIkFkMatchOp(const RigExecIkFkMatchOpConstants &constants,
                      RigExecRigDescBuilder *builder,
                      RigExecIkFkMatchOpNode *node, std::string *error)
{
    if (!builder || !node)
        return Fail(error, "null IK/FK match op destination");
    using namespace rigExecOps;
    IkFkMatchPayload payload;
    const RigExecSpaceSwitchRecord &record = constants.poleRecord;
    payload.poleSwitch.filters = record.filters;
    payload.poleSwitch.twistAxis = ToStd(record.twistAxis);
    payload.poleSwitch.affectTranslation = record.affectTranslation;
    payload.poleSwitch.affectRotation = record.affectRotation;
    payload.poleSwitch.affectScale = record.affectScale;
    payload.poleSwitch.hasCarry = constants.poleHasCarry;
    payload.poleSwitch.world.assign(constants.poleWorld.begin(),
                                    constants.poleWorld.end());
    payload.poleSourceRides.assign(constants.poleSourceRides.begin(),
                                   constants.poleSourceRides.end());
    payload.ikValue = constants.ikValue;
    payload.scaleCalibration = constants.scaleCalibration;
    payload.upperLengthOffset = constants.upperLengthOffset;
    payload.lowerLengthOffset = constants.lowerLengthOffset;
    payload.stretchLimb = constants.stretchLimb;
    RigExecRigDescBuilder &b = *builder;
    RigExecIkFkMatchOpNode n;
    n.constants = constants;
    const RigExecNodeId id = b.AddOp(IkFkMatch(), &payload);
    n.enabled = b.BindInput(id, IkFkMatchEnabled());
    n.weight = b.BindInput(id, IkFkMatchWeight());
    const RigExecPortHandle *scalars[7] = {
        &IkFkMatchSwitch(),       &IkFkMatchStretch(),
        &IkFkMatchSoftDistance(), &IkFkMatchUpperScale(),
        &IkFkMatchLowerScale(),   &IkFkMatchPin(),
        &IkFkMatchTwist()};
    for (size_t k = 0; k < 7; ++k)
        n.scalars[k] = b.BindInput(id, *scalars[k]);
    n.sources = b.BindInput(id, IkFkMatchSources());
    n.rests = b.BindInput(id, IkFkMatchRests());
    n.channels = b.BindInput(id, IkFkMatchChannels());
    n.hasSpace = b.BindInput(id, IkFkMatchHasSpace());
    n.spaceRest = b.BindInput(id, IkFkMatchSpaceRest());
    n.spacePosed = b.BindInput(id, IkFkMatchSpacePosed());
    n.poleMode = b.BindInput(id, IkFkMatchPoleMode());
    n.posedDefault = b.BindInput(id, IkFkMatchPosedDefault());
    n.parentDefaultInverse = b.BindInput(id, IkFkMatchParentDefaultInverse());
    n.parentPosed = b.BindInput(id, IkFkMatchParentPosed());
    n.parentDefault = b.BindInput(id, IkFkMatchParentDefault());
    if (constants.poleHasCarry) {
        n.spaceDefault = b.BindInput(id, IkFkMatchSwitchSpaceDefault());
        n.switchSpacePosed = b.BindInput(id, IkFkMatchSwitchSpacePosed());
    }
    n.sourceDefaults = b.BindInput(id, IkFkMatchSourceDefaults());
    n.sourcePosed = b.BindInput(id, IkFkMatchSourcePosed());
    n.active = b.BindInput(id, IkFkMatchActive());
    n.ikValues = b.BindOutput(id, IkFkMatchIkValues());
    n.fkValues = b.BindOutput(id, IkFkMatchFkValues());
    n.shown = b.BindOutput(id, IkFkMatchShown());
    n.shownMask = b.BindOutput(id, IkFkMatchShownMask());
    *node = std::move(n);
    return true;
}

bool
RigExecWriteIkFkMatchOpInputs(const RigExecIkFkMatchOpConstants &constants,
                              const RigExecIkFkMatchOpInputs &input,
                              const RigExecIkFkMatchOpNode &node,
                              RigExecPoseOpScratch *scratch,
                              RigExecEvaluator *e, std::string *error)
{
    if (!e || !scratch || !scratch->values)
        return Fail(error, "null IK/FK match op evaluator");
    if (!SameIkFkMatch(node.constants, constants))
        return Fail(error, "IK/FK match settings moved");
    RigExecPoseOpScratch::Values &v = *scratch->values;
    const uint8_t enabled = 1, hasSpace = input.hasSpace ? 1 : 0;
    if (!Set(e, node.enabled, enabled, error) ||
        !Set(e, node.weight, input.weight, error))
        return false;
    for (size_t k = 0; k < 7; ++k)
        if (!Set(e, node.scalars[k], input.scalars[k], error))
            return false;
    const auto matrices = [&](const GfMatrix4d *m, size_t count,
                              const RigExecInputId &id) {
        v.matrices.Resize(count);
        for (size_t k = 0; k < count; ++k)
            v.matrices[k] = ToStd(m[k]);
        return Set(e, id, v.matrices, error);
    };
    if (!matrices(input.sources.data(), 10, node.sources) ||
        !matrices(input.rests.data(), 8, node.rests))
        return false;
    v.doubles.Resize(5 * rigExecOps::kMatchChannelStride);
    for (size_t i = 0; i < 5; ++i)
        PackChannels(input.channels[i],
                     v.doubles.Data() + i * rigExecOps::kMatchChannelStride);
    if (!Set(e, node.channels, v.doubles, error) ||
        !Set(e, node.hasSpace, hasSpace, error) ||
        !Set(e, node.spaceRest, ToStd(input.spaceRest), error) ||
        !Set(e, node.spacePosed, ToStd(input.spacePosed), error) ||
        !Set(e, node.poleMode, input.poleMode, error))
        return false;
    const RigExecSpaceSwitchInputs &s = input.poleSwitch;
    if (!Set(e, node.posedDefault, ToStd(s.posedDefault), error) ||
        !Set(e, node.parentDefaultInverse, ToStd(s.parentDefaultInverse),
             error) ||
        !Set(e, node.parentPosed, ToStd(s.parentPosed), error) ||
        !Set(e, node.parentDefault, ToStd(s.parentDefault), error) ||
        !Set(e, node.active, s.active, error))
        return false;
    if (constants.poleHasCarry &&
        (!Set(e, node.spaceDefault, ToStd(s.spaceDefault), error) ||
         !Set(e, node.switchSpacePosed, ToStd(s.spacePosed), error)))
        return false;
    std::vector<GfMatrix4d> defaults, posed;
    for (const auto &source : s.sources) {
        defaults.push_back(source.defaultSpace);
        posed.push_back(source.posedSpace);
    }
    return matrices(defaults.data(), defaults.size(), node.sourceDefaults) &&
           matrices(posed.data(), posed.size(), node.sourcePosed);
}

bool
RigExecReadIkFkMatchOpOutput(const RigExecIkFkMatchOpNode &node,
                             const RigExecEvaluator &e,
                             RigExecIkFkMatchOpOutputs *output)
{
    const auto *ik =
        static_cast<const RigExecArray<double> *>(e.GetOutput(node.ikValues));
    const auto *fk =
        static_cast<const RigExecArray<double> *>(e.GetOutput(node.fkValues));
    const auto *shown =
        static_cast<const RigExecArray<Mat4d> *>(e.GetOutput(node.shown));
    const auto *mask =
        static_cast<const uint32_t *>(e.GetOutput(node.shownMask));
    if (!ik || !fk || !shown || !mask || ik->Size() != 11 || fk->Size() != 20 ||
        shown->Size() != 5)
        return false;
    std::copy(ik->begin(), ik->end(), output->ikValues.begin());
    std::copy(fk->begin(), fk->end(), output->fkValues.begin());
    for (size_t k = 0; k < 5; ++k)
        output->shown[k] = FromStd((*shown)[k]);
    output->shownMask = *mask;
    return true;
}

} // namespace rigExec
