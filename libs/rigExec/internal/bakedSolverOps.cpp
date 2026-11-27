#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecOps/solver/chainSolverOps.h"
#include "rigExecOps/solver/frameBlendOps.h"
#include "rigExecOps/solver/ribbonFramesOps.h"
#include "rigExecOps/solver/skeleton2dOps.h"
#include "rigExecOps/solver/splineIkOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"
#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/math.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace rigExec {

namespace {

using Solver = RigExecBakedProgramImpl::Solver;
using Landmarks = rigExecStd::RigExecArray<rigExecRig::PointLandmarks>;
using Read = eval::OpSolverRead;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

bool
Is(const RigExecTypeDesc &type, const RigExecTypeHandle &handle)
{
    return RigExecRegistry::Get().TypeAt(handle.Index()).desc == &type;
}

rigExecRig::PointLandmarks
ToOp(const std::array<GfVec3d, 4> &points)
{
    rigExecRig::PointLandmarks out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = rigExecStd::Vec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

rigExecRig::PointFrame
ToOp(const RigExecPointFrame &f)
{
    rigExecRig::PointFrame out;
    out.points = ToOp(f.points);
    out.flags = f.flags;
    return out;
}

std::array<GfVec3d, 4>
FromOp(const rigExecRig::PointLandmarks &points)
{
    std::array<GfVec3d, 4> out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = GfVec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

RigExecPointFrame
FromOp(const rigExecRig::PointFrame &f)
{
    RigExecPointFrame out;
    out.points = FromOp(f.points);
    out.flags = f.flags;
    return out;
}

/// \p v into \p value, a scalar of \p type; false for another type.
template <class T>
bool
Scalar(const RigExecTypeDesc &type, T v, void *value)
{
    if (type.element || type.size != sizeof(T))
        return false;
    std::memcpy(value, &v, sizeof(T));
    return true;
}

/// \p s's rigExec:space map from its rest frame, when the space names a
/// slot and moved it; false otherwise.
bool
SplineSpace(const RigExecBakedProgramImpl &B, const Solver &s,
            GfMatrix4d *delta)
{
    *delta = GfMatrix4d(1.0);
    return s.spaceSlot >= 0 && size_t(s.spaceSlot) < B.restPts.size() &&
           size_t(s.spaceRead) < B.fin.size() &&
           RigExecPointsToMatrix(B.restPts[size_t(s.spaceSlot)],
                                 B.fin[size_t(s.spaceRead)], delta) &&
           *delta != GfMatrix4d(1.0);
}

/// Whether \p s reads its spline settings from their leaves this run
/// rather than the ones Build folded.
bool
SplineParamsLive(const RigExecBakedProgramImpl &B, const Solver &s)
{
    const auto flagged = [](const auto &input, const std::vector<char> &on) {
        return input.overrideIndex >= 0 &&
               size_t(input.overrideIndex) < on.size() &&
               on[size_t(input.overrideIndex)];
    };
    const auto live = [&](const auto &input) {
        return input.varying || flagged(input, B.overridden) ||
               flagged(input, B.upstreamOn);
    };
    return s.splineParamsVary || live(s.preserveVolume) ||
           live(s.midFollowWeight) || live(s.roll) || live(s.twist) ||
           live(s.minLengthRatio);
}

/// The frames \p s's op output leads with that are no joint of its
/// aggregate: an FK chain's start frame.
size_t
LeadingFrames(const Solver &s)
{
    return s.type == SchemaTypeName::RigExecFkChain && s.start >= 0 ? 1 : 0;
}

} // namespace

int
RigExecBakedSolverOf(const RigExecBakedProgramImpl &B,
                     const RigExecBakedStep &step)
{
    if (step.kind == RigExecBakedStepKind::Solve)
        return step.object;
    const RigExecBakedOpOrigin *origin = RigExecBakedOriginOf(B, step);
    return origin && origin->family == RigExecBakedOpFamily::Solver
               ? origin->object
               : -1;
}

size_t
RigExecBakedSolverNodeCount(const RigExecBakedProgramImpl::Solver &s)
{
    if (s.type == SchemaTypeName::RigExecTwoBoneIk ||
        s.type == SchemaTypeName::RigExecTwistDistribution ||
        s.type == SchemaTypeName::RigExecRibbon ||
        s.type == SchemaTypeName::RigExecSplineIk ||
        s.type == SchemaTypeName::RigExecSkeleton2d)
        return 1;
    if (s.type != SchemaTypeName::RigExecFkChain)
        return 0;
    return s.jointGroups.empty() ? 1 : 2;
}

bool
RigExecBakedAddSolverOp(const RigExecBakedProgramImpl &B,
                        RigExecRigDescBuilder *builder,
                        eval::OpStepLayout *layout, size_t o,
                        std::string *error)
{
    using namespace rigExecOps;
    const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
    const uint32_t si = uint32_t(origin.object);
    const Solver &s = B.solvers[si];
    const auto look = [&](RigExecNodeId node, const RigExecPortHandle &port,
                          const eval::OpLookup &lookup) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(lookup);
    };
    const auto read = [&](RigExecNodeId node, const RigExecPortHandle &port,
                          Read part) {
        look(
            node, port,
            {uint8_t(eval::OpLookupSource::SolverInput), si, int32_t(part), 0});
    };
    const auto publish = [&](RigExecNodeId node,
                             const RigExecPortHandle &port) {
        builder->BindOutput(node, port);
        layout->hostOutputs.push_back(
            {node, port.Index(), uint8_t(eval::OpOutputTarget::SolverFrames),
             si});
    };
    const auto placed = [&](RigExecNodeId node) {
        return node == o ||
               Fail(error, s.pathText + ": its op node " + std::to_string(o) +
                               " was added as node " + std::to_string(node));
    };
    const auto version = [&](RigExecNodeId node, const RigExecPortHandle &port,
                             uint32_t read) {
        look(node, port,
             {uint8_t(eval::OpLookupSource::PoseVersion), read, 0, 0});
    };
    const auto constant = [&](RigExecNodeId node, const RigExecPortHandle &port,
                              const void *data, size_t bytes) {
        builder->BindInput(node, port);
        eval::OpConstant value;
        value.node = node;
        value.port = port.Index();
        value.bytes.resize(bytes);
        if (bytes)
            std::memcpy(value.bytes.data(), data, bytes);
        layout->constants.push_back(std::move(value));
    };
    if (s.type == SchemaTypeName::RigExecTwistDistribution) {
        const RigExecNodeId node = builder->AddOp(DistributeTwist());
        if (!placed(node))
            return false;
        version(node, DistributeTwistStart(), s.rootRead);
        version(node, DistributeTwistEnd(), s.endRead);
        read(node, DistributeTwistStartRest(), Read::TwistStartRest);
        read(node, DistributeTwistEndRest(), Read::TwistEndRest);
        constant(node, DistributeTwistWeights(), s.twistWeights.data(),
                 s.twistWeights.size() * sizeof(double));
        read(node, DistributeTwistTurns(), Read::TwistTurns);
        read(node, DistributeTwistJointRests(), Read::JointRests);
        read(node, DistributeTwistJointRestLive(), Read::JointRestLive);
        publish(node, DistributeTwistOut());
        return true;
    }
    if (s.type == SchemaTypeName::RigExecRibbon) {
        SampleRibbonFramesPayload p;
        p.sampleCount = s.ribbonSamples;
        const RigExecNodeId node = builder->AddOp(SampleRibbonFrames(), &p);
        if (!placed(node))
            return false;
        read(node, SampleRibbonFramesPosed(), Read::DriverPoints);
        constant(node, SampleRibbonFramesRest(), s.ribbonRestPoints.data(),
                 s.ribbonRestPoints.size() * sizeof(GfVec3f));
        read(node, SampleRibbonFramesJointRests(), Read::JointRests);
        read(node, SampleRibbonFramesJointRestLive(), Read::JointRestLive);
        publish(node, SampleRibbonFramesOut());
        return true;
    }
    if (s.type == SchemaTypeName::RigExecSplineIk) {
        SplineIkPayload p;
        p.restLengthChain =
            s.splineRestMode == RigExecSplineIkRestLength::Chain;
        p.aimRootTangent = s.splineParams.aimRootTangent;
        const RigExecNodeId node = builder->AddOp(SplineIk(), &p);
        if (!placed(node))
            return false;
        read(node, SplineIkRestJoints(), Read::SplineRestJoints);
        read(node, SplineIkRestRoot(), Read::SplineRestRoot);
        read(node, SplineIkRestMid(), Read::SplineRestMid);
        read(node, SplineIkRestEnd(), Read::SplineRestEnd);
        version(node, SplineIkRoot(), s.rootRead);
        version(node, SplineIkMid(), s.midRead);
        version(node, SplineIkEnd(), s.endRead);
        if (!s.splineRestWeights.empty())
            constant(node, SplineIkVolumeWeights(), s.splineRestWeights.data(),
                     s.splineRestWeights.size() * sizeof(double));
        read(node, SplineIkPreserveVolume(), Read::PreserveVolume);
        read(node, SplineIkMidFollow(), Read::MidFollow);
        read(node, SplineIkRoll(), Read::Roll);
        read(node, SplineIkTwist(), Read::SplineTwist);
        read(node, SplineIkMinLengthRatio(), Read::MinLengthRatio);
        read(node, SplineIkOutRests(), Read::SplineJointRests);
        publish(node, SplineIkOut());
        return true;
    }
    if (s.type == SchemaTypeName::RigExecSkeleton2d) {
        Skeleton2dPayload p;
        p.order = s.bone2dOrder;
        p.setup = s.bone2dSetup;
        p.spaceSources = s.bone2dSpaceSources;
        p.targetCandidates = s.bone2dTargetCandidates;
        p.groups = s.jointGroups;
        p.groupMembers = s.jointGroupMembers;
        const RigExecNodeId node = builder->AddOp(Skeleton2d(), &p);
        if (!placed(node))
            return false;
        read(node, Skeleton2dRests(), Read::JointRests);
        if (s.start >= 0) {
            read(node, Skeleton2dStartRest(), Read::StartRest);
            read(node, Skeleton2dStartPose(), Read::StartPose);
        }
        read(node, Skeleton2dExternalRests(), Read::ControlRests);
        read(node, Skeleton2dExternalPoses(), Read::ControlPoses);
        read(node, Skeleton2dChannels(), Read::Bone2dChannels);
        read(node, Skeleton2dSelectors(), Read::Bone2dSelectors);
        read(node, Skeleton2dGroupSelectors(), Read::GroupSelectors);
        publish(node, Skeleton2dOut());
        return true;
    }
    if (s.type == SchemaTypeName::RigExecTwoBoneIk) {
        TwoBoneIkPayload p;
        p.softDistancePolicy = s.ikParams.softDistancePolicy;
        p.scaleSegments = s.ikParams.scaleSegments;
        p.scaleCalibration = s.ikParams.limb.scaleCalibration;
        const RigExecNodeId node = builder->AddOp(TwoBoneIk(), &p);
        if (!placed(node))
            return false;
        version(node, TwoBoneIkRoot(), s.rootRead);
        version(node, TwoBoneIkEffector(), s.endRead);
        version(node, TwoBoneIkPole(), s.poleRead);
        read(node, TwoBoneIkRests(), Read::IkRests);
        read(node, TwoBoneIkSpace(), Read::IkSpace);
        read(node, TwoBoneIkBend(), Read::Bend);
        read(node, TwoBoneIkUpperOffset(), Read::UpperOffset);
        read(node, TwoBoneIkLowerOffset(), Read::LowerOffset);
        read(node, TwoBoneIkStretch(), Read::Stretch);
        read(node, TwoBoneIkSoftness(), Read::Softness);
        read(node, TwoBoneIkPin(), Read::Pin);
        read(node, TwoBoneIkUpperScale(), Read::UpperScale);
        read(node, TwoBoneIkLowerScale(), Read::LowerScale);
        read(node, TwoBoneIkSoftDistance(), Read::SoftDistance);
        read(node, TwoBoneIkTwist(), Read::Twist);
        publish(node, TwoBoneIkOut());
        return true;
    }
    const bool start = s.start >= 0;
    const size_t nodes = RigExecBakedSolverNodeCount(s);
    if ((origin.part >= 0 ? size_t(origin.part) : nodes - 1) == 0) {
        FkChainPayload p;
        const int base = start ? 1 : 0;
        if (base)
            p.parentIndices.push_back(-1);
        for (size_t k = 0; k < s.controls.size(); ++k)
            p.parentIndices.push_back(s.parentRelative ? base - 1
                                                       : int(k) + base - 1);
        p.scaleSegments = s.scaleSegments;
        const RigExecNodeId node = builder->AddOp(FkChain(), &p);
        if (!placed(node))
            return false;
        read(node, FkChainControls(), Read::FkControls);
        read(node, FkChainOutRests(), Read::FkOutRests);
        if (s.jointGroups.empty())
            publish(node, FkChainOut());
        return true;
    }
    FkJointGroupsPayload p;
    p.jointBegin = start ? 1 : 0;
    p.parents = s.jointGroupParents;
    p.groups = s.jointGroups;
    p.members = s.jointGroupMembers;
    const RigExecNodeId node = builder->AddOp(FkJointGroups(), &p);
    if (!placed(node))
        return false;
    builder->Connect(node - 1, FkChainOut(), node, FkJointGroupsIn());
    read(node, FkJointGroupsSelectors(), Read::GroupSelectors);
    if (start) {
        version(node, FkJointGroupsStart(), s.startRead);
        read(node, FkJointGroupsStartRest(), Read::StartRest);
    }
    publish(node, FkJointGroupsOut());
    return true;
}

bool
RigExecBakedLookUpSolver(const RigExecBakedProgramImpl &B,
                         const eval::OpLookup &lookup,
                         const RigExecTypeDesc &type, void *value,
                         std::string *error)
{
    if (lookup.object >= B.solvers.size())
        return Fail(error, "solver " + std::to_string(lookup.object) +
                               " is no solver of the program");
    const Solver &s = B.solvers[lookup.object];
    const size_t base = s.start >= 0 ? 1 : 0, n = s.controls.size();
    const auto rest = [&](int slot, rigExecRig::PointLandmarks *out) {
        if (slot < 0 || size_t(slot) >= B.restPts.size())
            return Fail(error, s.pathText + " names no rest slot " +
                                   std::to_string(slot));
        *out = ToOp(B.restPts[size_t(slot)]);
        return true;
    };
    const auto frame = [&](uint32_t version, rigExecRig::PointFrame *out) {
        if (version >= B.fin.size())
            return Fail(error, "pose version " + std::to_string(version) +
                                   " is no pose frame of the program");
        *out = ToOp(B.fin[version]);
        return true;
    };
    switch (Read(lookup.part)) {
    case Read::FkControls: {
        if (!Is(type, rigExecRig::PointFrameArrayType()) ||
            s.controlReads.size() != n)
            break;
        auto &out = *static_cast<rigExecRig::PointFrameArray *>(value);
        out.frames.Resize(n + base);
        out.rests.Resize(n + base);
        if (base && (!frame(s.startRead, &out.frames[0]) ||
                     !rest(s.start, &out.rests[0])))
            return false;
        for (size_t k = 0; k < n; ++k)
            if (!frame(s.controlReads[k], &out.frames[k + base]) ||
                !rest(s.controls[k], &out.rests[k + base]))
                return false;
        return true;
    }
    case Read::FkOutRests: {
        if (!Is(type, rigExecRig::PointLandmarksList()))
            break;
        auto &out = *static_cast<Landmarks *>(value);
        out.Resize(n + base);
        if (base && !rest(s.start, &out[0]))
            return false;
        const bool jointBasis = s.jointRests.size() == n;
        for (size_t k = 0; k < n; ++k) {
            if (!jointBasis || k >= s.restIsLive.size() || !s.restIsLive[k]) {
                if (!rest(s.controls[k], &out[k + base]))
                    return false;
                continue;
            }
            if (k >= s.restReads.size() || s.restReads[k] >= B.fin.size())
                return Fail(error, s.pathText + " names no live rest " +
                                       std::to_string(k));
            out[k + base] = ToOp(B.fin[s.restReads[k]].points);
        }
        return true;
    }
    case Read::StartRest:
        if (!Is(type, rigExecRig::PointLandmarksType()))
            break;
        return rest(s.start, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::IkRests: {
        if (!Is(type, rigExecRig::PointLandmarksList()))
            break;
        auto &out = *static_cast<Landmarks *>(value);
        out.Resize(3);
        for (size_t e = 0; e < 3; ++e)
            out[e] = ToOp(s.ikRests[e]);
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const auto [slot, element] = s.restRefs[k];
            if (element < 0 || element >= 3)
                continue;
            if (k < s.restIsLive.size() && s.restIsLive[k]) {
                if (k >= s.restReads.size() || s.restReads[k] >= B.fin.size())
                    return Fail(error, s.pathText + " names no live rest " +
                                           std::to_string(k));
                out[size_t(element)] = ToOp(B.fin[s.restReads[k]].points);
            } else if (!rest(slot, &out[size_t(element)])) {
                return false;
            }
        }
        return true;
    }
    case Read::IkSpace: {
        if (!Is(type, rigExecStd::Mat4dType()))
            break;
        GfMatrix4d space = RigExecBakedLeafRead(B, s.ikSpace);
        if (s.spaceSlot >= 0) {
            if (size_t(s.spaceSlot) >= B.restPts.size() ||
                size_t(s.spaceRead) >= B.fin.size())
                return Fail(error, s.pathText + " names no space frame");
            GfMatrix4d delta(1.0);
            if (RigExecPointsToMatrix(B.restPts[size_t(s.spaceSlot)],
                                      B.fin[size_t(s.spaceRead)], &delta))
                space = delta * space;
        }
        std::memcpy(value, space.data(), sizeof(double) * 16);
        return true;
    }
    case Read::Bend:
        return Scalar(type, RigExecBakedLeafRead(B, s.bend), value) ||
               Fail(error, s.pathText + " has no bend of that type");
    case Read::UpperOffset:
        return Scalar(type, RigExecBakedLeafRead(B, s.upperOffset), value) ||
               Fail(error, s.pathText + " has no upper offset of that type");
    case Read::LowerOffset:
        return Scalar(type, RigExecBakedLeafRead(B, s.lowerOffset), value) ||
               Fail(error, s.pathText + " has no lower offset of that type");
    case Read::Stretch:
        return Scalar(type, RigExecBakedLeafRead(B, s.stretch), value) ||
               Fail(error, s.pathText + " has no stretch of that type");
    case Read::Softness:
        return Scalar(type, RigExecBakedLeafRead(B, s.softness), value) ||
               Fail(error, s.pathText + " has no softness of that type");
    case Read::Pin:
        return Scalar(type, RigExecBakedLeafRead(B, s.pin), value) ||
               Fail(error, s.pathText + " has no pin of that type");
    case Read::UpperScale:
        return Scalar(type, RigExecBakedLeafRead(B, s.upperScale), value) ||
               Fail(error, s.pathText + " has no upper scale of that type");
    case Read::LowerScale:
        return Scalar(type, RigExecBakedLeafRead(B, s.lowerScale), value) ||
               Fail(error, s.pathText + " has no lower scale of that type");
    case Read::SoftDistance:
        return Scalar(type, RigExecBakedLeafRead(B, s.softDistance), value) ||
               Fail(error, s.pathText + " has no soft distance of that type");
    case Read::Twist:
        return Scalar(type, RigExecBakedLeafRead(B, s.limbTwist), value) ||
               Fail(error, s.pathText + " has no twist of that type");
    case Read::JointRests: {
        if (!Is(type, rigExecRig::PointLandmarksList()))
            break;
        auto &out = *static_cast<Landmarks *>(value);
        out.Resize(s.jointRests.size());
        for (size_t k = 0; k < s.jointRests.size(); ++k) {
            if (k < s.restIsLive.size() && s.restIsLive[k] &&
                k < s.restReads.size() && s.restReads[k] < B.fin.size())
                out[k] = ToOp(B.fin[s.restReads[k]].points);
            else if (k >= s.restRefs.size())
                out[k] = ToOp(s.jointRests[k]);
            else if (!rest(s.restRefs[k].first, &out[k]))
                return false;
        }
        return true;
    }
    case Read::JointRestLive: {
        if (!Is(type, rigExecStd::BoolArray()))
            break;
        auto &out = *static_cast<rigExecStd::RigExecArray<uint8_t> *>(value);
        out.Resize(s.jointRests.size());
        for (size_t k = 0; k < s.jointRests.size(); ++k)
            out[k] = k < s.restIsLive.size() && s.restIsLive[k] ? 1 : 0;
        return true;
    }
    case Read::TwistStartRest:
        if (!Is(type, rigExecRig::PointLandmarksType()))
            break;
        return rest(s.root, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::TwistEndRest:
        if (!Is(type, rigExecRig::PointLandmarksType()))
            break;
        return rest(s.end, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::TwistTurns:
        return Scalar(type, RigExecBakedLeafRead(B, s.twistTurns), value) ||
               Fail(error, s.pathText + " has no twist turns of that type");
    case Read::DriverPoints: {
        if (!Is(type, rigExecStd::Vec3fArray()))
            break;
        const GfVec3f *points = nullptr;
        size_t count = 0;
        if (!RigExecBakedResolvePoints(B, s.ribbonPointsBinding, &points,
                                       &count)) {
            const std::vector<GfVec3f> &sampled =
                s.ribbonPointsVarying ? s.ribbonPoints : s.ribbonConstantPoints;
            points = sampled.data();
            count = sampled.size();
        }
        auto &out =
            *static_cast<rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(value);
        out.Resize(count);
        for (size_t i = 0; i < count; ++i)
            out[i] =
                rigExecStd::Vec3f(points[i][0], points[i][1], points[i][2]);
        return true;
    }
    case Read::SplineRestJoints: {
        if (!Is(type, rigExecRig::PointFrameList()))
            break;
        GfMatrix4d delta;
        const bool spaced = SplineSpace(B, s, &delta);
        auto &out =
            *static_cast<rigExecStd::RigExecArray<rigExecRig::PointFrame> *>(
                value);
        out.Resize(s.splineCount);
        for (size_t e = 0; e < s.splineCount; ++e)
            out[e] =
                ToOp(spaced ? RigExecTransformFrame(RigExecPointFrame(), delta)
                            : RigExecPointFrame());
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const auto [slot, element] = s.restRefs[k];
            if (element < 0 || size_t(element) >= s.splineCount)
                continue;
            const bool live = k < s.restIsLive.size() && s.restIsLive[k] &&
                              k < s.restReads.size() &&
                              s.restReads[k] < B.fin.size();
            if (!live && (slot < 0 || size_t(slot) >= B.restFrames.size()))
                return Fail(error, s.pathText + " names no rest slot " +
                                       std::to_string(slot));
            const RigExecPointFrame &frame =
                live ? B.fin[s.restReads[k]] : B.restFrames[size_t(slot)];
            out[size_t(element)] =
                ToOp(spaced ? RigExecTransformFrame(frame, delta) : frame);
        }
        return true;
    }
    case Read::SplineRestRoot:
    case Read::SplineRestMid:
    case Read::SplineRestEnd: {
        if (!Is(type, rigExecRig::PointFrameType()))
            break;
        const int slot = Read(lookup.part) == Read::SplineRestRoot  ? s.root
                         : Read(lookup.part) == Read::SplineRestMid ? s.mid
                                                                    : s.end;
        if (slot >= 0 && size_t(slot) >= B.restFrames.size())
            return Fail(error, s.pathText + " names no rest slot " +
                                   std::to_string(slot));
        const RigExecPointFrame frame =
            slot >= 0 ? B.restFrames[size_t(slot)] : RigExecPointFrame();
        GfMatrix4d delta;
        *static_cast<rigExecRig::PointFrame *>(value) =
            ToOp(SplineSpace(B, s, &delta) ? RigExecTransformFrame(frame, delta)
                                           : frame);
        return true;
    }
    case Read::SplineJointRests: {
        if (!Is(type, rigExecRig::PointLandmarksList()))
            break;
        auto &out = *static_cast<Landmarks *>(value);
        out.Resize(s.splineJointRests.size());
        for (size_t e = 0; e < s.splineJointRests.size(); ++e)
            out[e] = ToOp(s.splineJointRests[e]);
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const auto [slot, element] = s.restRefs[k];
            if (element < 0 || size_t(element) >= out.Size())
                continue;
            if (k < s.restIsLive.size() && s.restIsLive[k] &&
                k < s.restReads.size() && s.restReads[k] < B.fin.size())
                out[size_t(element)] = ToOp(B.fin[s.restReads[k]].points);
            else if (!rest(slot, &out[size_t(element)]))
                return false;
        }
        return true;
    }
    case Read::PreserveVolume:
        return Scalar(type,
                      SplineParamsLive(B, s)
                          ? RigExecBakedLeafRead(B, s.preserveVolume)
                          : s.splineParams.preserveVolume,
                      value) ||
               Fail(error, s.pathText + " has no preserveVolume of that type");
    case Read::MidFollow:
        return Scalar(type,
                      SplineParamsLive(B, s)
                          ? RigExecBakedLeafRead(B, s.midFollowWeight)
                          : s.splineParams.midFollowWeight,
                      value) ||
               Fail(error, s.pathText + " has no midFollowWeight of that type");
    case Read::Roll:
        return Scalar(type,
                      SplineParamsLive(B, s)
                          ? GfDegreesToRadians(RigExecBakedLeafRead(B, s.roll))
                          : s.splineParams.roll,
                      value) ||
               Fail(error, s.pathText + " has no roll of that type");
    case Read::SplineTwist:
        return Scalar(type,
                      SplineParamsLive(B, s)
                          ? GfDegreesToRadians(RigExecBakedLeafRead(B, s.twist))
                          : s.splineParams.twist,
                      value) ||
               Fail(error, s.pathText + " has no twist of that type");
    case Read::MinLengthRatio:
        return Scalar(type,
                      SplineParamsLive(B, s)
                          ? RigExecBakedLeafRead(B, s.minLengthRatio)
                          : s.splineParams.minLengthRatio,
                      value) ||
               Fail(error, s.pathText + " has no minLengthRatio of that type");
    case Read::StartPose:
        if (!Is(type, rigExecRig::PointLandmarksType()) ||
            s.startRead >= B.fin.size())
            break;
        *static_cast<rigExecRig::PointLandmarks *>(value) =
            ToOp(B.fin[s.startRead].points);
        return true;
    case Read::ControlRests: {
        if (!Is(type, rigExecRig::PointLandmarksList()))
            break;
        auto &out = *static_cast<Landmarks *>(value);
        out.Resize(s.controls.size());
        for (size_t k = 0; k < s.controls.size(); ++k)
            if (!rest(s.controls[k], &out[k]))
                return false;
        return true;
    }
    case Read::ControlPoses: {
        if (!Is(type, rigExecRig::PointLandmarksList()) ||
            s.controlReads.size() != s.controls.size())
            break;
        auto &out = *static_cast<Landmarks *>(value);
        out.Resize(s.controls.size());
        for (size_t k = 0; k < s.controls.size(); ++k) {
            if (s.controlReads[k] >= B.fin.size())
                return Fail(error, "pose version " +
                                       std::to_string(s.controlReads[k]) +
                                       " is no pose frame of the program");
            out[k] = ToOp(B.fin[s.controlReads[k]].points);
        }
        return true;
    }
    case Read::Bone2dChannels: {
        if (!Is(type, rigExecStd::DoubleArray()))
            break;
        const size_t bones =
            std::min(s.bone2dDoubles.size() / kRigExecBone2dDoubleChannels,
                     s.bone2dBools.size() / kRigExecBone2dBoolChannels);
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(bones * kRigExecBone2dChannelCount);
        for (size_t k = 0; k < bones; ++k) {
            const auto *d = &s.bone2dDoubles[k * kRigExecBone2dDoubleChannels];
            const auto *f = &s.bone2dBools[k * kRigExecBone2dBoolChannels];
            RigExecBone2dChannels ch;
            for (size_t c = 0; c < kRigExecBone2dChannelCount; ++c) {
                const size_t at = RigExecBone2dChannelStorage(c);
                RigExecBone2dSetChannel(
                    &ch, c,
                    RigExecBone2dChannelIsBool(c)
                        ? (RigExecBakedLeafRead(B, f[at]) ? 1.0 : 0.0)
                        : RigExecBakedLeafRead(B, d[at]));
            }
            double *x = out.Data() + k * kRigExecBone2dChannelCount;
            const double values[] = {ch.tx,
                                     ch.ty,
                                     ch.rz,
                                     ch.sx,
                                     double(ch.sideFlip),
                                     double(ch.endFlip),
                                     double(ch.lock),
                                     ch.lockX,
                                     ch.lockY,
                                     ch.lockAngle,
                                     ch.offsetX,
                                     ch.offsetY,
                                     ch.targetIndex};
            std::memcpy(x, values, sizeof(values));
        }
        return true;
    }
    case Read::Bone2dSelectors: {
        if (!Is(type, rigExecStd::DoubleArray()))
            break;
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(s.bone2dSelectors.size());
        for (size_t g = 0; g < s.bone2dSelectors.size(); ++g)
            out[g] = RigExecBakedLeafRead(B, s.bone2dSelectors[g]);
        return true;
    }
    case Read::GroupSelectors: {
        if (!Is(type, rigExecStd::DoubleArray()))
            break;
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(s.jointGroupSelectors.size());
        for (size_t g = 0; g < s.jointGroupSelectors.size(); ++g)
            out[g] = RigExecBakedLeafRead(B, s.jointGroupSelectors[g]);
        return true;
    }
    }
    return Fail(error, s.pathText + " has no input " +
                           std::to_string(lookup.part) + " of that type");
}

bool
RigExecBakedStoreSolver(RigExecBakedProgramImpl &B,
                        const eval::OpHostOutput &output, const void *value,
                        std::string *error)
{
    if (output.object >= B.solvers.size() ||
        output.object >= B.aggregates.size())
        return Fail(error, "solver " + std::to_string(output.object) +
                               " is no solver of the program");
    Solver &s = B.solvers[output.object];
    const auto &out = *static_cast<const rigExecRig::PointFrameArray *>(value);
    const size_t first = LeadingFrames(s);
    RigExecPointFrameArray &aggregate = B.aggregates[output.object];
    const size_t frames =
        out.frames.Size() > first ? out.frames.Size() - first : 0;
    const size_t rests =
        out.rests.Size() > first ? out.rests.Size() - first : 0;
    aggregate.frames.resize(frames);
    aggregate.rests.resize(rests);
    for (size_t k = 0; k < frames; ++k)
        aggregate.frames[k] = FromOp(out.frames[k + first]);
    for (size_t k = 0; k < rests; ++k)
        aggregate.rests[k] = FromOp(out.rests[k + first]);
    s.fallbackSlots.clear();
    for (size_t k = 0; k < s.outputs.size(); ++k) {
        const auto &[slot, element] = s.outputs[k];
        if (element < 0 || size_t(element) >= aggregate.GetSize()) {
            s.outPresent[k] = 0;
            s.fallbackSlots.push_back(slot);
            continue;
        }
        s.outFrames[k] =
            RigExecExtractElementFrame(&aggregate, size_t(element));
        s.outPresent[k] = 1;
    }
    return true;
}

std::string
RigExecBakedSolverFailure(const RigExecBakedProgramImpl &B, int solver,
                          const std::string &error)
{
    if (solver < 0 || size_t(solver) >= B.solvers.size())
        return error;
    return B.solvers[size_t(solver)].pathText + ": " + error;
}

} // namespace rigExec
