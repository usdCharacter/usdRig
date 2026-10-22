#include "solverSceneTranslation.h"
#include "sceneInputTypes.h"
#include "rigExec/solverKernels.h"
#include "rigExec/types.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/gf/math.h"
#include "pxr/base/tf/staticTokens.h"
#include <type_traits>
namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    ((rigExecJointElements,"rigExec:jointElements")) ((rigExecScaleCalibration,"rigExec:scaleCalibration"))
    ((rigExecWeights,"rigExec:weights")) ((rigExecCount,"rigExec:count")) (points)
    ((rigExecDriverCurve,"rigExec:driverCurve")) (base) ((rigExecVolumeWeights,"rigExec:volumeWeights"))
    ((rigExecControlSpace,"rigExec:controlSpace")) ((rigExecSegmentScale,"rigExec:segmentScale")) (none)
    ((rigExecStretchPolicy,"rigExec:stretchPolicy")) (uniformSegments)
    ((rigExecScaleBlend,"rigExec:scaleBlend")) ((rigExecRotationBlend,"rigExec:rotationBlend")) (shortestArc)
    ((rigExecRestLength,"rigExec:restLength")) ((rigExecRootTangent,"rigExec:rootTangent"))
    ((rigExecJoints,"rigExec:joints")) ((rigExecControls,"rigExec:controls"))
    ((rigExecStartFrame,"rigExec:startFrame")) ((rigExecRootControl,"rigExec:rootControl"))
    ((rigExecEffectorControl,"rigExec:effectorControl")) ((rigExecPoleControl,"rigExec:poleControl"))
    ((rigExecSpace,"rigExec:space")) ((rigExecInputA,"rigExec:inputA")) ((rigExecInputB,"rigExec:inputB"))
    ((rigExecStart,"rigExec:start")) ((rigExecEnd,"rigExec:end")) ((rigExecMidControl,"rigExec:midControl"))
    ((rigExecEndControl,"rigExec:endControl")) ((rigExecSpaceMatrix,"rigExec:spaceMatrix"))
    ((rigExecPreferredBendRadians,"rigExec:preferredBendRadians"))
    ((rigExecUpperLengthOffset,"rigExec:upperLengthOffset"))
    ((rigExecLowerLengthOffset,"rigExec:lowerLengthOffset")) ((inputsStretch,"inputs:stretch"))
    ((inputsSoftness,"inputs:softness")) ((inputsPin,"inputs:pin")) ((inputsUpperScale,"inputs:upperScale"))
    ((inputsLowerScale,"inputs:lowerScale")) ((inputsSoftDistance,"inputs:softDistance"))
    ((inputsTwist,"inputs:twist")) ((inputsWeight,"inputs:weight")) ((inputsTwistTurns,"inputs:twistTurns"))
    ((rigExecSampleCount,"rigExec:sampleCount")) ((inputsPreserveVolume,"inputs:preserveVolume"))
    ((inputsMidFollowWeight,"inputs:midFollowWeight")) ((inputsRoll,"inputs:roll"))
    ((inputsMinLengthRatio,"inputs:minLengthRatio"))
);
namespace {
bool Fail(std::string *error,const std::string &text) { if(error)*error=text;return false; }
}
bool RigExecTranslateSceneSolver(const RigExecSceneDescriptors &scene,const SdfPath &path,
    const std::map<SdfPath,RigExecPointFrame> &rests,RigExecSceneSolverDescriptor *output,
    std::string *error) {
    if(!output)return Fail(error,"null solver descriptor destination");
    const RigExecSceneCompileInputs source(scene);
    const auto *node=source.Node(path);
    if(!node || !node->fact.active || node->domain!=RigExecSceneDomain::Solver)
        return Fail(error,"missing active solver: "+path.GetString());
    RigExecSceneSolverDescriptor result;result.path=path;
    auto &s=result.record;
    const auto type=node->fact.type;
    if(type=="RigExecFkChain")s.kind=RigExecSolverKind::FkChain;
    else if(type=="RigExecTwoBoneIk")s.kind=RigExecSolverKind::TwoBoneIk;
    else if(type=="RigExecBlendPointFrames")s.kind=RigExecSolverKind::BlendPointFrames;
    else if(type=="RigExecTwistDistribution")s.kind=RigExecSolverKind::TwistDistribution;
    else if(type=="RigExecRibbon")s.kind=RigExecSolverKind::Ribbon;
    else if(type=="RigExecSplineIk")s.kind=RigExecSolverKind::SplineIk;
    else return Fail(error,"unsupported solver: "+type.GetString());
    auto targets=[&](const TfToken &name) { return source.Targets(path.AppendProperty(name)); };
    auto providerExists=[&](const SdfPath &provider) {
        const auto *fact=source.Node(provider);
        return fact && fact->domain==RigExecSceneDomain::Provider;
    };
    // Numerical seed availability cannot change structural solver cardinality.
    auto rest=[&](const SdfPath &provider) {
        const auto found=rests.find(provider);
        if(found!=rests.end())return found->second;
        RigExecPointFrame unavailable;unavailable.flags=0;return unavailable;
    };
    auto first=[&](const TfToken &name,bool provider) {
        const auto list=targets(name);
        return list.empty() || (provider && !providerExists(list.front()))?SdfPath{}:list.front();
    };
    auto read=[&](const SdfPath &property,UsdTimeCode time,auto *value,bool fixed) {
        const auto *attribute=source.Attribute(property);
        if(!attribute)return true;
        if(fixed && (attribute->fact.mightBeTimeVarying || !attribute->fact.sampleTimes.empty()))
            return Fail(error,"time-varying solver structure: "+property.GetString());
        RigExecSceneBoundInput binding;
        if(!source.Bind(property,RigExecSceneReadRoute::Raw,&binding,error))return false;
        VtValue raw;
        if(!source.Read(binding,time,&raw,nullptr,error))return false;
        using T=std::decay_t<decltype(*value)>;
        if(raw.IsEmpty())return true;
        if(!raw.IsHolding<T>())return true;
        *value=raw.UncheckedGet<T>();return true;
    };
    auto token=[&](const TfToken &name,const TfToken &fallback,TfToken *value) {
        *value=fallback;
        return read(path.AppendProperty(name),UsdTimeCode::Default(),value,true);
    };
    auto bind=[&](const TfToken &name,auto fallback) {
        const auto property=path.AppendProperty(name);
        if(!source.Attribute(property))return true;
        RigExecSceneBoundInput binding;
        if(!source.Bind(property,RigExecSceneReadRoute::ConnectionResolved,&binding,error))return false;
        // Empty schema values retain the typed computation fallback.
        for(auto &value:binding.values)if(value.IsEmpty())value=VtValue(fallback);
        RigExecSceneTypedRead typed;
        if(!RigExecBindSceneTypedRead(scene,property,RigExecSceneInputType<decltype(fallback)>(),&typed,error))return false;
        result.typedReads.emplace(name.GetString(),std::move(typed));
        result.inputs.emplace(name.GetString(),std::move(binding));return true;
    };
    auto initial=[&](const char *name,auto fallback) {
        const auto found=result.typedReads.find(name);
        if(found==result.typedReads.end() || scene.identities.empty())return fallback;
        VtValue value;
        if(!RigExecResolveSceneTypedRead(scene,found->second,scene.identities.front(),{},&value,nullptr))return fallback;
        using T=decltype(fallback);
        return value.IsHolding<T>()?value.UncheckedGet<T>():fallback;
    };
    VtIntArray elements;
    const bool consumesElements=s.kind==RigExecSolverKind::TwoBoneIk || s.kind==RigExecSolverKind::SplineIk;
    if(consumesElements && !read(path.AppendProperty(_tokens->rigExecJointElements),UsdTimeCode::Default(),&elements,true))return false;
    for(const auto &joint:targets(_tokens->rigExecJoints))if(providerExists(joint))result.joints.push_back(joint);
    const bool remap=consumesElements && !elements.empty();
    if(remap && elements.size()!=result.joints.size())s.degenerate=true;
    for(size_t k=0;k<result.joints.size();++k) {
        result.jointElements.push_back(remap && !s.degenerate?elements[k]:int(k));
        s.jointRests.push_back(rest(result.joints[k]).points);
    }
    s.restIsLive.assign(result.joints.size(),false);
    if(s.kind==RigExecSolverKind::FkChain) {
        TfToken mode;if(!token(_tokens->rigExecControlSpace,TfToken(),&mode))return false;
        s.parentRelative=mode=="parentRelative";
        if(!token(_tokens->rigExecSegmentScale,_tokens->none,&mode))return false;
        s.scaleSegments=mode=="toChild";
        for(const auto &control:targets(_tokens->rigExecControls))if(providerExists(control)) {
            result.controls.push_back(control);s.controlRests.push_back(rest(control).points);
        }
        result.start=first(_tokens->rigExecStartFrame,true);s.hasStart=!result.start.IsEmpty();
        if(s.hasStart)s.startRest=rest(result.start).points;
    } else if(s.kind==RigExecSolverKind::TwoBoneIk) {
        result.root=first(_tokens->rigExecRootControl,true);result.end=first(_tokens->rigExecEffectorControl,true);
        result.pole=first(_tokens->rigExecPoleControl,true);
        s.degenerate=s.degenerate || result.root.IsEmpty() || result.end.IsEmpty() || result.pole.IsEmpty();
        std::array<bool,3> seen{};
        for(size_t k=0;k<result.joints.size();++k) {
            const int element=result.jointElements[k];
            if(element<0 || element>=3)s.degenerate=true;
            else { s.ikRests[size_t(element)]=s.jointRests[k];seen[size_t(element)]=true; }
        }
        s.degenerate=s.degenerate || !(seen[0] && seen[1] && seen[2]);
        result.space=first(_tokens->rigExecSpace,true);
        if(!bind(_tokens->rigExecSpaceMatrix,GfMatrix4d(1.0)) || !bind(_tokens->rigExecPreferredBendRadians,0.0) ||
           !bind(_tokens->rigExecUpperLengthOffset,0.0) || !bind(_tokens->rigExecLowerLengthOffset,0.0) ||
           !bind(_tokens->inputsStretch,1.0f) || !bind(_tokens->inputsSoftness,0.0f) ||
           !bind(_tokens->inputsPin,0.0f) || !bind(_tokens->inputsUpperScale,1.0) || !bind(_tokens->inputsLowerScale,1.0) ||
           !bind(_tokens->inputsSoftDistance,0.0f) || !bind(_tokens->inputsTwist,0.0f))return false;
        TfToken policy,scaling;
        if(!token(_tokens->rigExecStretchPolicy,_tokens->uniformSegments,&policy) ||
           !token(_tokens->rigExecSegmentScale,_tokens->none,&scaling) ||
           !read(path.AppendProperty(_tokens->rigExecScaleCalibration),UsdTimeCode::Default(),&s.ikParams.limb.scaleCalibration,true))return false;
        s.ikParams.softDistancePolicy=policy=="softDistance";s.ikParams.scaleSegments=scaling=="toChild";
        s.ikParams.preferredBendRadians=initial("rigExec:preferredBendRadians",0.0);
        s.ikParams.stretch=initial("inputs:stretch",1.0f);
        s.ikParams.softness=initial("inputs:softness",0.0f);
        if(!s.degenerate)RigExecTwoBoneIkLengths(s.ikRests,initial("rigExec:spaceMatrix",GfMatrix4d(1.0)),
            initial("rigExec:upperLengthOffset",0.0),initial("rigExec:lowerLengthOffset",0.0),
            &s.ikParams.upperLength,&s.ikParams.lowerLength);

    } else if(s.kind==RigExecSolverKind::BlendPointFrames) {
        result.blendA=first(_tokens->rigExecInputA,false);result.blendB=first(_tokens->rigExecInputB,false);
        TfToken mode;if(!token(_tokens->rigExecScaleBlend,TfToken(),&mode))return false;
        s.scaleMode=mode=="linear"?RigExecScaleBlend::Linear:RigExecScaleBlend::Log;
        if(!token(_tokens->rigExecRotationBlend,_tokens->shortestArc,&mode))return false;
        s.blendRotationRejected=mode!="shortestArc";
        if(!bind(_tokens->inputsWeight,0.0f))return false;
    } else if(s.kind==RigExecSolverKind::TwistDistribution) {
        result.root=first(_tokens->rigExecStart,true);result.end=first(_tokens->rigExecEnd,true);
        s.degenerate=s.degenerate || result.root.IsEmpty() || result.end.IsEmpty();
        if(!result.root.IsEmpty())s.twistStartRest=rest(result.root).points;
        if(!result.end.IsEmpty())s.twistEndRest=rest(result.end).points;
        VtFloatArray weights;int count=1;
        if(!read(path.AppendProperty(_tokens->rigExecWeights),UsdTimeCode::Default(),&weights,true) ||
           !read(path.AppendProperty(_tokens->rigExecCount),UsdTimeCode::Default(),&count,true))return false;
        for(float weight:weights)s.twistWeights.push_back(weight);
        RigExecResolveTwistWeights(count,&s.twistWeights);
        if(!bind(_tokens->inputsTwistTurns,0.0))return false;
    } else if(s.kind==RigExecSolverKind::Ribbon) {
        result.ribbonPoints=first(_tokens->rigExecDriverCurve,false);
        if(result.ribbonPoints.IsPrimPath())result.ribbonPoints=result.ribbonPoints.AppendProperty(_tokens->points);
        const auto *relation=source.Relationship(path.AppendProperty(_tokens->rigExecDriverCurve));
        result.ribbonReadPhase=relation?relation->readPhase:_tokens->base;
        VtVec3fArray points;
        if(!result.ribbonPoints.IsEmpty() && !read(result.ribbonPoints,UsdTimeCode::Default(),&points,false))return false;
        s.ribbonRestPoints.assign(points.begin(),points.end());
        if(!result.ribbonPoints.IsEmpty() && source.Attribute(result.ribbonPoints)) {
            RigExecSceneBoundInput binding;
            if(!source.Bind(result.ribbonPoints,RigExecSceneReadRoute::Raw,&binding,error))return false;
            result.inputs.emplace("driverPoints",std::move(binding));
        }
        if(!bind(_tokens->rigExecSampleCount,5))return false;
    } else if(s.kind==RigExecSolverKind::SplineIk) {
        result.root=first(_tokens->rigExecRootControl,true);result.mid=first(_tokens->rigExecMidControl,true);
        result.end=first(_tokens->rigExecEndControl,true);result.space=first(_tokens->rigExecSpace,true);
        s.degenerate=s.degenerate || result.root.IsEmpty() || result.mid.IsEmpty() || result.end.IsEmpty();
        const size_t count=result.joints.size();s.splineCount=count;s.degenerate=s.degenerate || count==0;
        s.splineRestFrames.resize(count);s.splineJointRests.resize(count);std::vector<bool> filled(count,false);
        for(size_t k=0;k<count;++k) {
            const int element=result.jointElements[k];
            if(element<0 || size_t(element)>=count || filled[size_t(element)]) { s.degenerate=true;continue; }
            filled[size_t(element)]=true;s.splineRestFrames[size_t(element)]=rest(result.joints[k]);
            s.splineJointRests[size_t(element)]=s.jointRests[k];
        }
        VtFloatArray weights;TfToken mode;
        if(!read(path.AppendProperty(_tokens->rigExecVolumeWeights),UsdTimeCode::Default(),&weights,true) ||
           !token(_tokens->rigExecRestLength,TfToken(),&mode))return false;
        for(float weight:weights)s.splineRestWeights.push_back(weight);
        if(!weights.empty() && weights.size()!=count)s.degenerate=true;
        if(!mode.IsEmpty() && mode!="curve" && mode!="chain")s.degenerate=true;
        s.splineRestMode=mode=="chain"?RigExecSplineIkRestLength::Chain:RigExecSplineIkRestLength::Curve;
        if(!result.root.IsEmpty())s.splineRootRest=rest(result.root);
        if(!result.mid.IsEmpty())s.splineMidRest=rest(result.mid);
        if(!result.end.IsEmpty())s.splineEndRest=rest(result.end);
        s.splineRest=RigExecSplineIkMakeRest(s.splineRestFrames,s.splineRootRest,s.splineMidRest,s.splineEndRest,
            s.splineRestWeights,s.splineRestMode);
        if(!token(_tokens->rigExecRootTangent,TfToken(),&mode))return false;
        s.splineParams.aimRootTangent=mode=="aim";
        if(!mode.IsEmpty() && mode!="aim" && mode!="rigid")s.degenerate=true;
        if(!bind(_tokens->inputsPreserveVolume,1.0) || !bind(_tokens->inputsMidFollowWeight,0.5) || !bind(_tokens->inputsRoll,0.0) ||
           !bind(_tokens->inputsTwist,0.0) || !bind(_tokens->inputsMinLengthRatio,0.0))return false;
        s.splineParams.preserveVolume=initial("inputs:preserveVolume",1.0);
        s.splineParams.midFollowWeight=initial("inputs:midFollowWeight",0.5);
        s.splineParams.roll=GfDegreesToRadians(initial("inputs:roll",0.0));
        s.splineParams.twist=GfDegreesToRadians(initial("inputs:twist",0.0));
        s.splineParams.minLengthRatio=initial("inputs:minLengthRatio",0.0);

    }
    for(const TfToken *name:{&_tokens->rigExecControls,&_tokens->rigExecStartFrame,&_tokens->rigExecRootControl,
        &_tokens->rigExecMidControl,&_tokens->rigExecEffectorControl,&_tokens->rigExecEndControl,
        &_tokens->rigExecPoleControl,&_tokens->rigExecStart,&_tokens->rigExecEnd,&_tokens->rigExecSpace,
        &_tokens->rigExecInputA,&_tokens->rigExecInputB}) {
        const auto relationPath=path.AppendProperty(*name);
        RigExecSceneSolverFrameBinding binding;
        const auto *relation=source.Relationship(relationPath);
        binding.phase=relation?relation->readPhase:_tokens->base;
        if(relation)for(const auto &target:relation->forwardedTargets)
            binding.keys.emplace(target,relationPath.AppendTarget(target));
        result.frameBindings.emplace(name->GetString(),std::move(binding));
    }
    if(!result.space.IsEmpty())result.spaceRest=rest(result.space).points;
    *output=std::move(result);return true;
}
bool RigExecResolveSceneSolverInputs(const RigExecSceneDescriptors &scene,
    const RigExecSceneSolverDescriptor &descriptor,UsdTimeCode identity,
    const std::map<SdfPath,RigExecPointFrame> &frames,
    const std::map<SdfPath,RigExecPointFrameArray> &aggregates,
    const std::map<SdfPath,VtValue> &deliveredInputs,RigExecSolverInputs *output,std::string *error) {
    if(!output)return Fail(error,"null solver inputs destination");
    const RigExecSceneCompileInputs source(scene);
    auto &input=*output;
    auto frame=[&](const SdfPath &path,const char *relationName,RigExecPointFrame *value) {
        if(path.IsEmpty())return true;
        const auto relation=descriptor.frameBindings.find(relationName);
        if(relation!=descriptor.frameBindings.end()) {
            const auto key=relation->second.keys.find(path);
            if(key!=relation->second.keys.end()) {
                const auto bound=frames.find(key->second);
                if(bound!=frames.end()) { *value=bound->second;return true; }
            }
            if(relation->second.phase!="base")
                return Fail(error,"phased solver frame producer unavailable: "+path.GetString());
        }
        const auto found=frames.find(path);
        if(found==frames.end())return Fail(error,"solver frame producer unavailable: "+path.GetString());
        *value=found->second;return true;
    };
    const auto readValue=[&](const char *name,VtValue *result) {
        const auto found=descriptor.inputs.find(name);
        *result=VtValue();
        if(found==descriptor.inputs.end())return true;
        const auto &binding=found->second;
        VtValue boxed;
        const auto typed=descriptor.typedReads.find(name);
        if(typed!=descriptor.typedReads.end()) {
            if(binding.readPhase!="base" && binding.source!=binding.consumer && !deliveredInputs.count(binding.consumer))
                return Fail(error,"phased solver input producer unavailable: "+binding.consumer.GetString());
            if(!RigExecResolveSceneTypedRead(scene,typed->second,identity,deliveredInputs,&boxed,nullptr))return true;
        } else {
            const auto delivered=deliveredInputs.find(binding.consumer);
            if(binding.route!=RigExecSceneReadRoute::Raw && delivered!=deliveredInputs.end())boxed=delivered->second;
            else if(!source.Read(binding,identity,&boxed,nullptr,error))return false;
        }
        *result=std::move(boxed);return true;
    };
    if(descriptor.record.degenerate)return true;
    input.controls.resize(descriptor.controls.size());
    for(size_t k=0;k<descriptor.controls.size();++k)
        if(!frame(descriptor.controls[k],"rigExec:controls",&input.controls[k]))return false;
    const bool twist=descriptor.record.kind==RigExecSolverKind::TwistDistribution;
    const bool twoBone=descriptor.record.kind==RigExecSolverKind::TwoBoneIk;
    if(!frame(descriptor.start,"rigExec:startFrame",&input.start) ||
       !frame(descriptor.root,twist?"rigExec:start":"rigExec:rootControl",&input.root) ||
       !frame(descriptor.mid,"rigExec:midControl",&input.mid) ||
       !frame(descriptor.end,twist?"rigExec:end":twoBone?"rigExec:effectorControl":"rigExec:endControl",&input.end) ||
       !frame(descriptor.pole,"rigExec:poleControl",&input.pole) ||
       !frame(descriptor.space,"rigExec:space",&input.space))return false;
    input.hasSpace=!descriptor.space.IsEmpty();
    input.spaceRest=descriptor.spaceRest;
    input.blendA=nullptr;input.blendB=nullptr;
    auto aggregate=[&](const SdfPath &path,const char *name,const RigExecPointFrameArray **value) {
        if(path.IsEmpty())return true;
        const auto relation=descriptor.frameBindings.find(name);
        if(relation!=descriptor.frameBindings.end()) {
            const auto key=relation->second.keys.find(path);
            if(key!=relation->second.keys.end()) {
                const auto phased=aggregates.find(key->second);
                if(phased!=aggregates.end()) { *value=&phased->second;return true; }
            }
            if(relation->second.phase!="base")
                return Fail(error,"phased solver aggregate producer unavailable: "+path.GetString());
        }
        const auto found=aggregates.find(path);
        if(found!=aggregates.end())*value=&found->second;
        return true;
    };
    if(!aggregate(descriptor.blendA,"rigExec:inputA",&input.blendA) ||
       !aggregate(descriptor.blendB,"rigExec:inputB",&input.blendB))return false;
    return RigExecRefreshSolverParameters(descriptor.record.kind,readValue,&input,error);
}

bool RigExecRefreshSolverParameters(RigExecSolverKind kind,
    const std::function<bool(const char *,VtValue *)> &read,
    RigExecSolverInputs *output,std::string *error) {
    if(!output)return Fail(error,"null solver parameters destination");
    auto &input=*output;
    const auto value=[&](const char *name,auto fallback,auto *result) {
        *result=fallback;VtValue boxed;
        if(!read(name,&boxed))return false;
        using T=std::decay_t<decltype(*result)>;
        if(boxed.IsHolding<T>())*result=boxed.UncheckedGet<T>();
        return true;
    };
    switch(kind) {
    case RigExecSolverKind::TwoBoneIk:
        input.refreshIkParams=true;
        return value("rigExec:spaceMatrix",GfMatrix4d(1.0),&input.ikSpace) &&
            value("rigExec:preferredBendRadians",0.0,&input.bend) &&
            value("rigExec:upperLengthOffset",0.0,&input.upperOffset) &&
            value("rigExec:lowerLengthOffset",0.0,&input.lowerOffset) &&
            value("inputs:stretch",1.0f,&input.stretch) && value("inputs:softness",0.0f,&input.softness) &&
            value("inputs:pin",0.0f,&input.pin) && value("inputs:upperScale",1.0,&input.upperScale) &&
            value("inputs:lowerScale",1.0,&input.lowerScale) && value("inputs:softDistance",0.0f,&input.softDistance) &&
            value("inputs:twist",0.0f,&input.limbTwist);
    case RigExecSolverKind::BlendPointFrames: {
        float weight=0;
        if(!value("inputs:weight",0.0f,&weight))return false;
        input.blendWeight=double(weight);return true;
    }
    case RigExecSolverKind::TwistDistribution:
        return value("inputs:twistTurns",0.0,&input.twistTurns);
    case RigExecSolverKind::Ribbon: {
        VtVec3fArray points;
        if(!value("driverPoints",VtVec3fArray{},&points) ||
            !value("rigExec:sampleCount",5,&input.ribbonSampleCount))return false;
        input.ribbonPoints.assign(points.begin(),points.end());return true;
    }
    case RigExecSolverKind::SplineIk:
        input.refreshSplineParams=true;
        return value("inputs:preserveVolume",1.0,&input.preserveVolume) &&
            value("inputs:midFollowWeight",0.5,&input.midFollowWeight) &&
            value("inputs:roll",0.0,&input.rollDegrees) && value("inputs:twist",0.0,&input.twistDegrees) &&
            value("inputs:minLengthRatio",0.0,&input.minLengthRatio);
    case RigExecSolverKind::FkChain:return true;
    }
    return Fail(error,"unknown normalized solver input kind");
}

}
