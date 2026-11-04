#include "solverProgram.h"
#include "rigExec/solverKernels.h"
#include "rigExec/types.h"
#include "pxr/base/gf/math.h"
#include <algorithm>
namespace rigExec {
bool RigExecRunSolver(const RigExecSolverRecord &s,const RigExecSolverInputs &input,
    RigExecSolverWorkspace *scratch,RigExecPointFrameArray *output,std::string *error) {
    if (!scratch || !output) {
        if(error)*error="null solver workspace/output";
        return false;
    }
    auto &aggregate=*output;
    aggregate.frames.clear(); aggregate.rests.clear();
    if(s.degenerate)return true;
    switch(s.kind) {
    case RigExecSolverKind::FkChain: {
        if(input.controls.size()!=s.controlRests.size()) {
            if(error)*error="FK control/rest cardinality mismatch";
            return false;
        }
        const bool jointBasis=s.jointRests.size()==input.controls.size();
        aggregate.rests=s.controlRests;
        const int base=s.hasStart?1:0;
        auto &elements=scratch->fkElements;
        elements.resize(input.controls.size()+size_t(base));
        if(base) {
            elements[0].restPoints=s.startRest;
            elements[0].posePoints=input.start.points;
            elements[0].hasOutRest=false; elements[0].parentIndex=-1;
        }
        for(size_t k=0;k<input.controls.size();++k) {
            auto &element=elements[k+size_t(base)];
            element.restPoints=s.controlRests[k]; element.posePoints=input.controls[k].points;
            const bool live=jointBasis && k<s.restIsLive.size() && s.restIsLive[k];
            element.hasOutRest=live;
            if(live) { element.outRestPoints=s.jointRests[k];aggregate.rests[k]=s.jointRests[k]; }
            element.parentIndex=s.parentRelative?base-1:int(k)+base-1;
        }
        aggregate.frames=RigExecSolveFkChain(elements);
        if(s.scaleSegments)RigExecScaleFkSegments(elements,&aggregate.frames);
        if(base && !aggregate.frames.empty())aggregate.frames.erase(aggregate.frames.begin());
        return true;
    }
    case RigExecSolverKind::TwoBoneIk: {
        auto params=s.ikParams;
        auto space=input.ikSpace;
        bool spaceMoved=false;
        if(input.hasSpace) {
            GfMatrix4d delta(1.0);
            if(RigExecPointsToMatrix(input.spaceRest,input.space,&delta)) {
                space=delta*space;spaceMoved=true;
            }
        }
        if(input.refreshIkParams || spaceMoved) {
            params.preferredBendRadians=input.bend;params.stretch=input.stretch;
            params.softness=input.softness;params.space=space;
            RigExecTwoBoneIkLengths(s.ikRests,space,input.upperOffset,input.lowerOffset,
                &params.upperLength,&params.lowerLength);
        }
        if(input.refreshIkParams || spaceMoved)
            RigExecSetTwoBoneLimbParams(s.ikRests,space,input.stretch,input.pin,input.upperScale,
                input.lowerScale,input.softDistance,input.limbTwist,&params);
        const auto frames=RigExecSolveTwoBoneIk(input.root,input.end,input.pole,s.ikRests,params);
        aggregate.frames.assign(frames.begin(),frames.end());
        aggregate.rests.assign(s.ikRests.begin(),s.ikRests.end());
        return true;
    }
    case RigExecSolverKind::BlendPointFrames: {
        const auto *a=input.blendA,*b=input.blendB;
        if(!a) { if(b)aggregate=*b;return true; }
        if(!b) { aggregate=*a;return true; }
        if(s.blendRotationRejected)return true;
        const size_t n=a->GetSize();
        const double weight=std::min(std::max(input.blendWeight,0.0),1.0);
        if(n==b->GetSize() && a->rests.size()==n) {
            aggregate.frames.reserve(n);aggregate.rests.reserve(n);
            for(size_t k=0;k<n;++k) {
                const bool live=k<s.restIsLive.size() && s.restIsLive[k] && k<s.jointRests.size();
                aggregate.frames.push_back(RigExecBlendFrames(a->frames[k],b->frames[k],a->rests[k],
                    weight,RigExecRotationBlend::ShortestArc,s.scaleMode,live?&s.jointRests[k]:nullptr));
                aggregate.rests.push_back(live?s.jointRests[k]:a->rests[k]);
            }
        }
        return true;
    }
    case RigExecSolverKind::TwistDistribution:
        aggregate=RigExecSolveTwistDistribution(input.root,input.end,s.twistStartRest,s.twistEndRest,
            s.twistWeights,input.twistTurns,s.jointRests,s.restIsLive);
        return true;
    case RigExecSolverKind::Ribbon:
        aggregate=RigExecSampleRibbonFrames(input.ribbonPoints,s.ribbonRestPoints,input.ribbonSampleCount,
            s.jointRests,s.restIsLive);
        return true;
    case RigExecSolverKind::SplineIk: {
        auto params=s.splineParams;
        if(input.refreshSplineParams) {
            params.preserveVolume=input.preserveVolume;params.midFollowWeight=input.midFollowWeight;
            params.roll=GfDegreesToRadians(input.rollDegrees);params.twist=GfDegreesToRadians(input.twistDegrees);
            params.minLengthRatio=input.minLengthRatio;
        }
        const auto *rest=&s.splineRest;
        if(input.hasSpace) {
            GfMatrix4d delta(1.0);
            if(RigExecPointsToMatrix(input.spaceRest,input.space,&delta) && delta!=GfMatrix4d(1.0)) {
                auto &spaced=scratch->spacedFrames;spaced.clear();spaced.reserve(s.splineRestFrames.size());
                for(const auto &frame:s.splineRestFrames)spaced.push_back(RigExecTransformFrame(frame,delta));
                scratch->spacedRest=RigExecSplineIkMakeRest(spaced,
                    RigExecTransformFrame(s.splineRootRest,delta),RigExecTransformFrame(s.splineMidRest,delta),
                    RigExecTransformFrame(s.splineEndRest,delta),s.splineRestWeights,s.splineRestMode);
                rest=&scratch->spacedRest;
            }
        }
        RigExecSplineIkControls controls;
        controls.root=input.root;controls.mid=input.mid;controls.end=input.end;
        RigExecSolveSplineIk(*rest,controls,params,&scratch->splineResult);
        if(scratch->splineResult.joints.size()==s.splineCount) {
            aggregate.frames.reserve(s.splineCount);
            for(const auto &joint:scratch->splineResult.joints)aggregate.frames.push_back(joint.frame);
            aggregate.rests=s.splineJointRests;
        }
        return true;
    }
    case RigExecSolverKind::Skeleton2d: {
        // Rests and outputs in rigExec:joints order; the kernel runs in solve
        // order, in the start frame's plane. A failed solve publishes every
        // bone's setup frame (no deformation) and says why.
        const size_t n=s.bone2dOrder.size();
        if(s.jointRests.size()!=n || s.bone2dSetup.size()!=n || input.bone2dChannels.size()!=n ||
           input.controls.size()!=s.controlRests.size()) {
            if(error)*error="2D skeleton bone, rest, channel and parent-space counts differ";
            return false;
        }
        const RigExecSkeleton2dStatic extra=RigExecSkeleton2dStaticOf(s.bone2dSpaceSources,
            s.bone2dTargetCandidates,s.bone2dGroups,s.bone2dGroupMembers);
        auto &posed=scratch->bone2dExternalPoses;
        posed.resize(input.controls.size());
        for(size_t e=0;e<input.controls.size();++e)posed[e]=input.controls[e].points;
        RigExecSkeleton2dLandmarkInputs<RigExecSolverRest> frames;
        if(s.hasStart) {frames.startRest=&s.startRest;frames.startPose=&input.start.points;}
        frames.externalRests=s.controlRests.data();frames.externalPoses=posed.data();
        frames.externalCount=posed.size();
        frames.selectors=input.bone2dSelectors.data();frames.selectorCount=input.bone2dSelectors.size();
        const RigExecSkeleton2dResult result=RigExecSolveSkeleton2dJoints(s.bone2dOrder,s.jointRests,
            s.bone2dSetup,extra,frames,
            [&](size_t j,RigExecBone2dChannels *channels){*channels=input.bone2dChannels[j];},
            RigExecPointFrameValid,&scratch->bone2d,&aggregate.frames);
        if(result==RigExecSkeleton2dResult::Malformed) {
            if(error)*error="2D skeleton solve order is malformed";
            return false;
        }
        aggregate.rests=s.jointRests;
        if(result==RigExecSkeleton2dResult::NotPlanar) {
            if(error)*error="a 2D bone's rest (its rest:space, or the frame a pose step below the skeleton wrote) or parent space is not a rotation about +Z in the skeleton's plane; publishing the rests";
            return false;
        }
        if(result==RigExecSkeleton2dResult::NotFinite) {
            if(error)*error="2D skeleton solve failed (a non-finite channel); publishing setup frames";
            return false;
        }
        return true;
    }
    }
    if(error)*error="unknown solver kind";
    return false;
}
}
