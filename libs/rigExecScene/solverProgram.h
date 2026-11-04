#ifndef RIGEXEC_GRAPH_SOLVER_PROGRAM_H
#define RIGEXEC_GRAPH_SOLVER_PROGRAM_H
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"
#include "rigExecMath/skeleton2dKernel.h"
#include "rigExecMath/pointFrame.h"
namespace rigExec {
// Defined in rigExec/types.h; held and passed by pointer here.
struct RigExecPointFrameArray;
enum class RigExecSolverKind { FkChain, TwoBoneIk, BlendPointFrames, TwistDistribution, Ribbon, SplineIk,
    Skeleton2d };
/// Schema relationships that declare solver input prerequisites. A target
/// solver remains a structural dependency even when its numerical frame/curve
/// type is unavailable; scheduling comes only from the common op compiler.
inline std::vector<std::string> RigExecSolverRelationshipPorts(RigExecSolverKind kind)
{
    switch (kind) {
    case RigExecSolverKind::FkChain: return {"rigExec:controls", "rigExec:startFrame"};
    case RigExecSolverKind::TwoBoneIk:
        return {"rigExec:rootControl", "rigExec:effectorControl", "rigExec:poleControl", "rigExec:space"};
    case RigExecSolverKind::BlendPointFrames: return {"rigExec:inputA", "rigExec:inputB"};
    case RigExecSolverKind::TwistDistribution: return {"rigExec:start", "rigExec:end"};
    case RigExecSolverKind::Ribbon:
        return {"rigExec:driverCurve", "rigExec:startFrame", "rigExec:endFrame", "rigExec:twistFrames"};
    case RigExecSolverKind::SplineIk:
        return {"rigExec:rootControl", "rigExec:midControl", "rigExec:endControl", "rigExec:space"};
    case RigExecSolverKind::Skeleton2d:
        // Its avars are per-bone typed reads; the start frame is its only
        // solver-input relationship.
        return {"rigExec:startFrame"};
    }
    return {};
}
using RigExecSolverRest = std::array<GfVec3d,4>;
/// Detached structural/refresh data. Source bindings belong to the compiler,
/// never this record. All rest data is in rig-common row-vector space.
struct RigExecSolverRecord {
    RigExecSolverKind kind = RigExecSolverKind::FkChain;
    bool degenerate=false, parentRelative=false, hasStart=false, scaleSegments=false;
    std::vector<RigExecSolverRest> controlRests, jointRests;
    std::vector<bool> restIsLive;
    RigExecSolverRest startRest{};
    std::array<RigExecSolverRest,3> ikRests{};
    RigExecTwoBoneIkParams ikParams;
    RigExecScaleBlend scaleMode=RigExecScaleBlend::Log;
    bool blendRotationRejected=false;
    RigExecSolverRest twistStartRest{},twistEndRest{};
    std::vector<double> twistWeights;
    std::vector<GfVec3f> ribbonRestPoints;
    RigExecSplineIkRest splineRest;
    RigExecSplineIkParams splineParams;
    std::vector<RigExecPointFrame> splineRestFrames;
    RigExecPointFrame splineRootRest,splineMidRest,splineEndRest;
    std::vector<double> splineRestWeights;
    RigExecSplineIkRestLength splineRestMode=RigExecSplineIkRestLength::Curve;
    std::vector<RigExecSolverRest> splineJointRests;
    size_t splineCount=0;
    /// Skeleton2d: rigExec:joints index per solve entry (parents first) and
    /// the structural setup in that order. Rests are jointRests (rigExec:joints
    /// order), refreshed by the solver rest route; the start frame rides on
    /// hasStart/startRest and the outside parent spaces on controlRests, as
    /// for an FK chain. The space, candidate and member arrays below are
    /// concatenated in the setup's ranges.
    std::vector<uint32_t> bone2dOrder;
    std::vector<RigExecBone2dSetup> bone2dSetup;
    std::vector<int32_t> bone2dSpaceSources, bone2dTargetCandidates;
    std::vector<RigExecBone2dGroup> bone2dGroups;
    std::vector<int32_t> bone2dGroupMembers;
};
/// Current graph values, delivered after typed read-phase resolution. Null
/// aggregate pointers preserve BlendPointFrames unwired-input pass-through.
struct RigExecSolverInputs {
    std::vector<RigExecPointFrame> controls;
    RigExecPointFrame start,root,mid,end,pole;
    const RigExecPointFrameArray *blendA=nullptr,*blendB=nullptr;
    double blendWeight=0,twistTurns=0;
    std::vector<GfVec3f> ribbonPoints;
    int ribbonSampleCount=0;
    bool refreshIkParams=false;
    double bend=0,upperOffset=0,lowerOffset=0;
    float stretch=1,softness=0,pin=0,softDistance=0,limbTwist=0;
    double upperScale=1,lowerScale=1;
    GfMatrix4d ikSpace=GfMatrix4d(1.0);
    bool hasSpace=false;
    RigExecSolverRest spaceRest{};
    RigExecPointFrame space;
    bool refreshSplineParams=false;
    double preserveVolume=0,midFollowWeight=0,rollDegrees=0,twistDegrees=0,minLengthRatio=0;
    /// Skeleton2d: each bone's channels, rigExec:joints order, then the
    /// selectors: the active member index per group (rigExec:boneGroups
    /// order), then each space switch's active index. Its start frame is
    /// `start` and its outside parent spaces are `controls`.
    std::vector<RigExecBone2dChannels> bone2dChannels;
    std::vector<double> bone2dSelectors;
};
struct RigExecSolverWorkspace {
    std::vector<RigExecFkChainElement> fkElements;
    std::vector<RigExecPointFrame> spacedFrames;
    RigExecSplineIkRest spacedRest;
    RigExecSplineIkResult splineResult;
    RigExecSkeleton2dScratch bone2d;
    /// Skeleton2d: the outside parent spaces' posed landmarks this solve.
    std::vector<RigExecSolverRest> bone2dExternalPoses;
};
/// One production numerical body for native and direct SceneDb graphs.
/// Clears outputs even for malformed/degenerate bindings.
bool RigExecRunSolver(const RigExecSolverRecord &,const RigExecSolverInputs &,
    RigExecSolverWorkspace *,RigExecPointFrameArray *,std::string *error=nullptr);
}
#endif
