#include "constraintSceneTranslation.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/gf/rotation.h"
#include <type_traits>
#include <algorithm>
#include <cmath>
#include <set>
#include "rigExec/frameExtraction.h"
#include "pxr/base/tf/staticTokens.h"
namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (XYZ)
    (aimX)
    ((avarsSx, "avars:sx"))
    ((avarsSy, "avars:sy"))
    ((avarsSz, "avars:sz"))
    ((avarsTx, "avars:tx"))
    ((avarsTy, "avars:ty"))
    ((avarsTz, "avars:tz"))
    ((inputsAffectRotationX, "inputs:affectRotationX"))
    ((inputsAffectRotationY, "inputs:affectRotationY"))
    ((inputsAffectRotationZ, "inputs:affectRotationZ"))
    ((inputsAffectScaleX, "inputs:affectScaleX"))
    ((inputsAffectScaleY, "inputs:affectScaleY"))
    ((inputsAffectScaleZ, "inputs:affectScaleZ"))
    ((inputsAffectTranslationX, "inputs:affectTranslationX"))
    ((inputsAffectTranslationY, "inputs:affectTranslationY"))
    ((inputsAffectTranslationZ, "inputs:affectTranslationZ"))
    ((inputsAimVector, "inputs:aimVector"))
    ((inputsDefaultWeight, "inputs:defaultWeight"))
    ((inputsEnabled, "inputs:enabled"))
    ((inputsLowerScale, "inputs:lowerScale"))
    ((inputsPin, "inputs:pin"))
    ((inputsPoleVector, "inputs:poleVector"))
    ((inputsPoleVectorWeights, "inputs:poleVectorWeights"))
    ((inputsRotationOffset, "inputs:rotationOffset"))
    ((inputsRotationOffsets, "inputs:rotationOffsets"))
    ((inputsScaleOffset, "inputs:scaleOffset"))
    ((inputsSoftDistance, "inputs:softDistance"))
    ((inputsSourceWeights, "inputs:sourceWeights"))
    ((inputsStretch, "inputs:stretch"))
    ((inputsSwitch, "inputs:switch"))
    ((inputsTranslationOffset, "inputs:translationOffset"))
    ((inputsTranslationOffsets, "inputs:translationOffsets"))
    ((inputsTwist, "inputs:twist"))
    ((inputsTwistDegrees, "inputs:twistDegrees"))
    ((inputsUpVector, "inputs:upVector"))
    ((inputsUpperScale, "inputs:upperScale"))
    ((inputsWorldUpVector, "inputs:worldUpVector"))
    (neverTS)
    (none)
    ((posedSpace, "posed:space"))
    ((rigExecAimAxis, "rigExec:aimAxis"))
    ((rigExecAimTarget, "rigExec:aimTarget"))
    ((rigExecBlendShear, "rigExec:blendShear"))
    ((rigExecEffector, "rigExec:effector"))
    ((rigExecEndJoint, "rigExec:endJoint"))
    ((rigExecEvaluationMode, "rigExec:evaluationMode"))
    ((rigExecFirstJoint, "rigExec:firstJoint"))
    ((rigExecOrientationMode, "rigExec:orientationMode"))
    ((rigExecPoleVectorMode, "rigExec:poleVectorMode"))
    ((rigExecPoleVectorObjects, "rigExec:poleVectorObjects"))
    ((rigExecRotationOrder, "rigExec:rotationOrder"))
    ((rigExecSolverMode, "rigExec:solverMode"))
    ((rigExecSources, "rigExec:sources"))
    ((rigExecSpace, "rigExec:space"))
    ((rigExecWorldUpObject, "rigExec:worldUpObject"))
    ((rigExecWorldUpRotationOnly, "rigExec:worldUpRotationOnly"))
    ((rigExecWorldUpType, "rigExec:worldUpType"))
    (rotatePlane)
    ((rigExecWaves, "rigExec:waves"))
    ((rigExecWaveLength, "rigExec:waveLength"))
    ((rigExecFrequency, "rigExec:frequency"))
    ((rigExecIgnoreFirstFrame, "rigExec:ignoreFirstFrame"))
    ((rigExecMatchVelocity, "rigExec:matchVelocity"))
    ((inputsAmplitude, "inputs:amplitude"))
    ((inputsDecay, "inputs:decay"))
    ((inputsTermination, "inputs:termination"))
    ((inputsAxisAmplitude, "inputs:axisAmplitude"))
    ((rigExecIkControl, "rigExec:ikControl"))
    ((rigExecPoleControl, "rigExec:poleControl"))
    ((rigExecRootControl, "rigExec:rootControl"))
    ((rigExecFkControls, "rigExec:fkControls"))
    ((rigExecLimbJoints, "rigExec:limbJoints"))
    ((rigExecMoves, "rigExec:moves"))
    ((rigExecIkValue, "rigExec:ikValue"))
    ((rigExecScaleCalibration, "rigExec:scaleCalibration"))
    ((rigExecUpperLengthOffset, "rigExec:upperLengthOffset"))
    ((rigExecLowerLengthOffset, "rigExec:lowerLengthOffset"))
    ((rigExecStretchPolicy, "rigExec:stretchPolicy"))
    (softDistance)
    (vector)
    (x)
);
namespace {
bool Fail(std::string *error,const std::string &message) { if(error)*error=message;return false; }
RigExecEulerOrder Order(const TfToken &token) {
    if(token=="XZY")return RigExecEulerOrder::XZY;
    if(token=="YXZ")return RigExecEulerOrder::YXZ;
    if(token=="YZX")return RigExecEulerOrder::YZX;
    if(token=="ZXY")return RigExecEulerOrder::ZXY;
    if(token=="ZYX")return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}
}
bool RigExecTranslateSceneConstraint(const RigExecSceneDescriptors &scene,const SdfPath &path,
    const SdfPathVector &targets,RigExecSceneConstraintDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null constraint descriptor destination");
    const RigExecSceneCompileInputs source(scene);const auto *node=source.Node(path);
    if(!node || !node->fact.active || node->domain!=RigExecSceneDomain::Constraint)
        return Fail(error,"missing active constraint: "+path.GetString());
    RigExecSceneConstraintDescriptor result;result.path=path;result.targets=targets;
    const auto type=node->fact.type;
    if(type=="RigExecPositionConstraint")result.record.kind=RigExecConstraintKind::Position;
    else if(type=="RigExecRotationConstraint")result.record.kind=RigExecConstraintKind::Rotation;
    else if(type=="RigExecScaleConstraint")result.record.kind=RigExecConstraintKind::Scale;
    else if(type=="RigExecParentConstraint")result.record.kind=RigExecConstraintKind::Parent;
    else if(type=="RigExecAimConstraint")result.record.kind=RigExecConstraintKind::Aim;
    else if(type=="RigExecSingleChainIkConstraint")result.record.kind=RigExecConstraintKind::SingleChainIk;
    else if(type=="RigExecIkFkMatchConstraint")result.record.kind=RigExecConstraintKind::IkFkMatch;
    else if(type=="RigExecHarmonicsConstraint")result.record.kind=RigExecConstraintKind::Harmonics;
    else return Fail(error,"unsupported constraint: "+type.GetString());
    auto relation=[&](const TfToken &name) { return source.Targets(path.AppendProperty(name)); };
    auto first=[&](const TfToken &name) { const auto rows=relation(name);return rows.empty()?SdfPath{}:rows.front(); };
    auto raw=[&](const TfToken &name,auto fallback,auto *value) {
        *value=fallback;const auto property=path.AppendProperty(name);
        if(!source.Attribute(property))return true;
        RigExecSceneBoundInput bound;VtValue boxed;
        if(!source.Bind(property,RigExecSceneReadRoute::Raw,&bound,error) ||
           !source.Read(bound,UsdTimeCode::Default(),&boxed,nullptr,error))return false;
        using T=std::decay_t<decltype(*value)>;
        if(boxed.IsHolding<T>())*value=boxed.UncheckedGet<T>();return true;
    };
    auto bind=[&](const TfToken &name,RigExecSceneReadRoute route=RigExecSceneReadRoute::ConnectionResolved,
                  const SdfValueTypeName *requestedType=nullptr,const SdfPath *boundAt=nullptr) {
        const auto head=path.AppendProperty(name);if(!source.Attribute(head))return true;
        const auto &property=boundAt?*boundAt:head;
        RigExecSceneBoundInput input;if(!source.Bind(property,route,&input,error))return false;
        const std::string &field=name.GetString();
        if(route!=RigExecSceneReadRoute::Raw) {
            const bool match=result.record.kind==RigExecConstraintKind::IkFkMatch;
            const auto requested=requestedType?*requestedType:field=="inputs:enabled" || field.rfind("inputs:affect",0)==0?SdfValueTypeNames->Bool:
                field=="inputs:defaultWeight" || (field=="inputs:stretch" && !match)?SdfValueTypeNames->Float:
                field=="inputs:twistDegrees" || match?SdfValueTypeNames->Double:SdfValueTypeNames->Double3;
            RigExecSceneTypedRead typed;
            if(!RigExecBindSceneTypedRead(scene,property,requested,&typed,error))return false;
            result.typedReads.emplace(field,std::move(typed));
        }
        result.inputs.emplace(field,std::move(input));return true;
    };
    // A scalar bound AT the attribute its connection walk ends on, in that
    // attribute's precision, as the baked program binds it: a double input
    // connected to a float channel reads the float channel.
    auto bindAtEndPrecision=[&](const TfToken &name) {
        SdfValueTypeName end;SdfPath endPath;std::set<SdfPath> visited;
        for(SdfPath at=path.AppendProperty(name);visited.insert(at).second;) {
            const auto *attribute=source.Attribute(at);if(!attribute)break;
            end=attribute->fact.type;endPath=at;
            if(attribute->fact.connections.size()!=1)break;
            at=attribute->fact.connections.front();
        }
        const auto requested=end==SdfValueTypeNames->Float?SdfValueTypeNames->Float:SdfValueTypeNames->Double;
        return bind(name,RigExecSceneReadRoute::ConnectionResolved,&requested,endPath.IsEmpty()?nullptr:&endPath);
    };
    if(!bind(_tokens->inputsEnabled) || !bind(_tokens->inputsDefaultWeight))return false;
    result.space=first(_tokens->rigExecSpace);
    // Harmonics has no sources, weights, masks or offsets. Its uniform
    // settings fix the history window; its scalars bind in the precision of
    // the attribute their connection walk ends on.
    if(result.record.kind==RigExecConstraintKind::Harmonics) {
        if(relation(_tokens->rigExecSpace).size()>1)
            return Fail(error,"RigExecHarmonicsConstraint "+path.GetString()+" has more than one rigExec:space");
        // A space that cannot supply a frame is dropped.
        if(const auto *space=result.space.IsEmpty()?nullptr:source.Node(result.space);
           !space || (space->fact.type!="RigExecControl" && space->fact.type!="RigExecJoint"))result.space=SdfPath();
        if(!raw(_tokens->rigExecWaves,5.0,&result.harmonicsWaves) ||
           !raw(_tokens->rigExecWaveLength,5.0,&result.harmonicsWaveLength) ||
           !raw(_tokens->rigExecFrequency,1.0,&result.harmonicsFrequency) ||
           !raw(_tokens->rigExecIgnoreFirstFrame,true,&result.harmonicsIgnoreFirstFrame) ||
           !raw(_tokens->rigExecMatchVelocity,true,&result.harmonicsMatchVelocity))return false;
        for(const TfToken *name:{&_tokens->inputsAmplitude,&_tokens->inputsDecay,&_tokens->inputsTermination})
            if(!bindAtEndPrecision(*name))return false;
        if(!bind(_tokens->inputsAxisAmplitude))return false;
        *output=std::move(result);return true;
    }
    if(!bind(_tokens->inputsSourceWeights,RigExecSceneReadRoute::Raw))return false;
    result.sources=relation(_tokens->rigExecSources);
    TfToken token;if(!raw(_tokens->rigExecRotationOrder,_tokens->XYZ,&token))return false;
    result.order=Order(token);
    if(!raw(_tokens->rigExecBlendShear,false,&result.blendShear) ||
       !raw(_tokens->rigExecWorldUpRotationOnly,false,&result.worldUpRotationOnly))return false;
    if(result.record.kind==RigExecConstraintKind::IkFkMatch) {
        result.effector=first(_tokens->rigExecEffector);
        auto &match=result.match;
        match.ikControl=relation(_tokens->rigExecIkControl);match.poleControl=relation(_tokens->rigExecPoleControl);
        match.rootControl=relation(_tokens->rigExecRootControl);match.effector=relation(_tokens->rigExecEffector);
        match.fkControls=relation(_tokens->rigExecFkControls);match.limbJoints=relation(_tokens->rigExecLimbJoints);
        match.moves=relation(_tokens->rigExecMoves);match.space=relation(_tokens->rigExecSpace);
        // A space that cannot supply a frame is dropped, as the compile drops it.
        if(const auto *space=result.space.IsEmpty()?nullptr:source.Node(result.space);
           !space || (space->fact.type!="RigExecControl" && space->fact.type!="RigExecJoint"))result.space=SdfPath();
        TfToken policy;
        if(!raw(_tokens->rigExecIkValue,1.0,&match.ikValue) ||
           !raw(_tokens->rigExecScaleCalibration,0.0,&match.scaleCalibration) ||
           !raw(_tokens->rigExecUpperLengthOffset,0.0,&match.upperLengthOffset) ||
           !raw(_tokens->rigExecLowerLengthOffset,0.0,&match.lowerLengthOffset) ||
           !raw(_tokens->rigExecStretchPolicy,_tokens->softDistance,&policy))return false;
        if(!std::isfinite(match.ikValue))match.ikValue=1.0;
        match.stretchLimb=policy==_tokens->softDistance;
        for(const TfToken *name:{&_tokens->inputsSwitch,&_tokens->inputsStretch,&_tokens->inputsSoftDistance,
                                 &_tokens->inputsUpperScale,&_tokens->inputsLowerScale,&_tokens->inputsPin,&_tokens->inputsTwist})
            if(!bindAtEndPrecision(*name))return false;
    } else if(result.record.kind==RigExecConstraintKind::SingleChainIk) {
        result.effector=first(_tokens->rigExecEffector);result.poleObjects=relation(_tokens->rigExecPoleVectorObjects);
        if(!raw(_tokens->rigExecSolverMode,_tokens->rotatePlane,&token))return false;
        result.record.singleChain.mode=token=="singleChain"?RigExecSingleChainIkMode::SingleChain:RigExecSingleChainIkMode::RotatePlane;
        if(!raw(_tokens->rigExecOrientationMode,_tokens->aimX,&token))return false;
        result.record.singleChain.preserveJointOrientation=token=="preserve";
        if(!raw(_tokens->rigExecPoleVectorMode,_tokens->vector,&token))return false;
        result.poleModeObject=token=="object";
        const auto firstJoint=first(_tokens->rigExecFirstJoint),lastJoint=first(_tokens->rigExecEndJoint);
        for(SdfPath current=lastJoint;!current.IsEmpty() && current!=SdfPath::AbsoluteRootPath();current=current.GetParentPath()) {
            const auto *joint=source.Node(current);
            if(joint && joint->fact.type=="RigExecJoint")result.ikChain.push_back(current);
            if(current==firstJoint)break;
        }
        if(result.ikChain.empty() || result.ikChain.back()!=firstJoint)
            return Fail(error,"single-chain IK endpoints do not form an inclusive joint chain: "+path.GetString());
        std::reverse(result.ikChain.begin(),result.ikChain.end());
        if(result.targets.empty())result.targets=result.ikChain;
        if(!raw(_tokens->rigExecEvaluationMode,_tokens->neverTS,&token))return false;
        bool animated=false;
        if(token=="autoDetect")for(const auto &joint:result.ikChain) {
            for(const auto &binding:scene.jointBindings)if(binding.joint==joint)animated=true;
            for(const TfToken *name:{&_tokens->posedSpace,&_tokens->avarsTx,&_tokens->avarsTy,&_tokens->avarsTz,
                                     &_tokens->avarsSx,&_tokens->avarsSy,&_tokens->avarsSz})
                if(const auto *attribute=source.Attribute(joint.AppendProperty(*name)))
                    animated=animated || !attribute->fact.sampleTimes.empty() ||
                        (attribute->fact.hasAuthoredConnections && !attribute->fact.connections.empty());
        }
        result.useAnimatedTs=token=="alwaysTS" || (token=="autoDetect" && animated);
        if(!bind(_tokens->inputsStretch))return false;
        if(result.record.singleChain.mode==RigExecSingleChainIkMode::RotatePlane) {
            if(!bind(_tokens->inputsPoleVector) || !bind(_tokens->inputsTwistDegrees))return false;
            if(result.poleModeObject && !result.poleObjects.empty() &&
               !bind(_tokens->inputsPoleVectorWeights,RigExecSceneReadRoute::Raw))return false;
        }
    } else {
        const bool parent=result.record.kind==RigExecConstraintKind::Parent;
        const int channel=result.record.kind==RigExecConstraintKind::Position?0:
            result.record.kind==RigExecConstraintKind::Scale?2:1;
        const TfToken *affect[3][3]={
            {&_tokens->inputsAffectTranslationX,&_tokens->inputsAffectTranslationY,&_tokens->inputsAffectTranslationZ},
            {&_tokens->inputsAffectRotationX,&_tokens->inputsAffectRotationY,&_tokens->inputsAffectRotationZ},
            {&_tokens->inputsAffectScaleX,&_tokens->inputsAffectScaleY,&_tokens->inputsAffectScaleZ}};
        for(int n=parent?0:channel;n<=(parent?2:channel);++n)
            for(int a=0;a<3;++a)if(!bind(*affect[n][a]))return false;
        if(parent) {
            if(!bind(_tokens->inputsTranslationOffsets,RigExecSceneReadRoute::Raw) ||
               !bind(_tokens->inputsRotationOffsets,RigExecSceneReadRoute::Raw))return false;
        } else {
            const TfToken &offset=result.record.kind==RigExecConstraintKind::Position?_tokens->inputsTranslationOffset:
                result.record.kind==RigExecConstraintKind::Scale?_tokens->inputsScaleOffset:_tokens->inputsRotationOffset;
            if(!bind(offset))return false;
        }
        if(result.record.kind==RigExecConstraintKind::Aim) {
            if(!bind(_tokens->inputsAimVector) || !bind(_tokens->inputsUpVector) || !bind(_tokens->inputsWorldUpVector))return false;
            const auto *aim=source.Attribute(path.AppendProperty(_tokens->inputsAimVector));
            result.aimVectorAuthored=aim && aim->fact.hasAuthoredValue;
            if(!raw(_tokens->rigExecAimAxis,_tokens->x,&token))return false;
            result.aimAxisFallback=token=="y"?GfVec3d(0,1,0):token=="z"?GfVec3d(0,0,1):GfVec3d(1,0,0);
            if(!raw(_tokens->rigExecWorldUpType,_tokens->none,&result.worldUpType))return false;
            result.worldUpObject=first(_tokens->rigExecWorldUpObject);result.preserveInputUp=result.sources.empty();
            if(result.sources.empty()) { const auto legacy=first(_tokens->rigExecAimTarget);if(!legacy.IsEmpty())result.sources.push_back(legacy); }
            result.sceneUp=scene.upAxis=="Z"?GfVec3d(0,0,1):GfVec3d(0,1,0);

        }
    }
    *output=std::move(result);return true;
}

bool RigExecResolveSceneConstraint(const RigExecSceneDescriptors &scene,
    RigExecSceneConstraintDescriptor *descriptor,UsdTimeCode identity,
    const std::map<SdfPath,VtValue> &delivered,RigExecConstraintInputs *input,
    bool *enabled,float *defaultWeight,std::string *error) {
    if(!descriptor || !input || !enabled || !defaultWeight)return Fail(error,"null constraint input destination");
    const RigExecSceneCompileInputs source(scene);
    const auto readValue=[&](const char *name,VtValue *value) {
        *value=VtValue();const auto found=descriptor->inputs.find(name);
        if(found==descriptor->inputs.end())return true;
        const auto &binding=found->second;VtValue boxed;
        const auto typed=descriptor->typedReads.find(name);
        if(typed!=descriptor->typedReads.end()) {
            if(binding.readPhase!="base" && binding.source!=binding.consumer && !delivered.count(binding.consumer))
                return Fail(error,"phased constraint input producer unavailable: "+binding.consumer.GetString());
            if(!RigExecResolveSceneTypedRead(scene,typed->second,identity,delivered,&boxed,nullptr))return true;
        } else if(!source.Read(binding,identity,&boxed,nullptr,error))return false;
        *value=std::move(boxed);return true;
    };
    return RigExecRefreshConstraintParameters(descriptor,readValue,input,enabled,defaultWeight,error);
}
bool RigExecRefreshConstraintParameters(RigExecSceneConstraintDescriptor *descriptor,
    const std::function<bool(const char *,VtValue *)> &readValue,
    RigExecConstraintInputs *input,bool *enabled,float *defaultWeight,std::string *error) {
    if(!descriptor || !input || !enabled || !defaultWeight)return Fail(error,"null constraint parameters destination");
    const auto read=[&](const char *name,auto fallback,auto *value) {
        *value=fallback;VtValue boxed;
        if(!readValue(name,&boxed))return false;
        using T=std::decay_t<decltype(*value)>;
        if(boxed.IsHolding<T>())*value=boxed.UncheckedGet<T>();
        return true;
    };
    if(!read("inputs:enabled",true,enabled) || !read("inputs:defaultWeight",1.0f,defaultWeight))return false;
    if(!*enabled)return true;
    auto &record=descriptor->record;
    if(record.kind==RigExecConstraintKind::SingleChainIk) {
        float stretch=0;if(!read("inputs:stretch",0.0f,&stretch))return false;
        record.singleChain.stretch=stretch;
        if(record.singleChain.mode==RigExecSingleChainIkMode::RotatePlane)
            if(!read("inputs:poleVector",GfVec3d(0,1,0),&record.singleChain.pole) ||
               !read("inputs:twistDegrees",0.0,&record.singleChain.twistDegrees))return false;
        if(record.singleChain.mode==RigExecSingleChainIkMode::RotatePlane && descriptor->poleModeObject &&
            !descriptor->poleObjects.empty()) {
            VtFloatArray weights;
            if(!read("inputs:poleVectorWeights",VtFloatArray{},&weights))return false;
            if(input->poleObjects.size()!=descriptor->poleObjects.size() ||
                (!weights.empty() && weights.size()!=input->poleObjects.size()))
                return Fail(error,"single-chain IK pole cardinality mismatch");
            GfVec3d pole(0);double total=0;
            for(size_t k=0;k<input->poleObjects.size();++k) {
                const double weight=weights.empty()?1.0:double(weights[k]);
                if(!RigExecConstraintFrameUsable(input->poleObjects[k]) || !std::isfinite(weight) || weight<0)
                    return Fail(error,"invalid single-chain IK pole object/weight");
                pole+=input->poleObjects[k].Origin()*weight;total+=weight;
            }
            if(!(total>0))return Fail(error,"single-chain IK pole weights have no positive total");
            record.singleChain.pole=pole/total;
        }
        record.singleChain.weight=double(*defaultWeight);
        return true;
    }
    if(record.kind==RigExecConstraintKind::IkFkMatch ||
       record.kind==RigExecConstraintKind::Harmonics)return true;
    VtFloatArray weights;
    if(!read("inputs:sourceWeights",VtFloatArray{},&weights))return false;
    if(!weights.empty() && weights.size()!=input->sources.size())
        return Fail(error,"constraint source weight cardinality mismatch: "+descriptor->path.GetString());
    for(size_t k=0;k<input->sources.size();++k)input->sources[k].normalizedWeight=weights.empty()?1.0:double(weights[k]);
    auto mask=[&](const char *channel,bool fallback,RigExecConstraintAxisMask *value) {
        const std::string prefix=std::string("inputs:affect")+channel;
        return read((prefix+"X").c_str(),fallback,&value->x) &&
            read((prefix+"Y").c_str(),fallback,&value->y) && read((prefix+"Z").c_str(),fallback,&value->z);
    };
    const double weight=double(*defaultWeight);
    switch(record.kind) {
    case RigExecConstraintKind::Position:
        record.position.weight=weight;
        return mask("Translation",true,&record.position.affect) &&
            read("inputs:translationOffset",GfVec3d(0),&record.position.offset);
    case RigExecConstraintKind::Rotation:
        record.rotation.weight=weight;record.rotation.rotationOrder=descriptor->order;
        return mask("Rotation",true,&record.rotation.affect) &&
            read("inputs:rotationOffset",GfVec3d(0),&record.rotation.offsetDegrees);
    case RigExecConstraintKind::Scale:
        record.scale.weight=weight;record.scale.blendShear=descriptor->blendShear;
        return mask("Scale",true,&record.scale.affect) && read("inputs:scaleOffset",GfVec3d(0),&record.scale.offset);
    case RigExecConstraintKind::Parent: {
        record.parent.weight=weight;record.parent.rotationOrder=descriptor->order;
        record.parent.blendShear=descriptor->blendShear;
        if(!mask("Translation",true,&record.parent.translationAxes) ||
           !mask("Rotation",true,&record.parent.rotationAxes) || !mask("Scale",false,&record.parent.scaleAxes))return false;
        VtVec3dArray translations,rotations;
        if(!read("inputs:translationOffsets",VtVec3dArray{},&translations) ||
           !read("inputs:rotationOffsets",VtVec3dArray{},&rotations))return false;
        if((!translations.empty() && translations.size()!=input->sources.size()) ||
           (!rotations.empty() && rotations.size()!=input->sources.size()))
            return Fail(error,"parent constraint offset cardinality mismatch: "+descriptor->path.GetString());
        for(size_t k=0;k<input->sources.size();++k) {
            input->sources[k].translationOffset=translations.empty()?GfVec3d(0):translations[k];
            input->sources[k].rotationOffsetDegrees=rotations.empty()?GfVec3d(0):rotations[k];
        }
        return true;
    }
    case RigExecConstraintKind::Aim: {
        auto &params=record.aim;params.weight=weight;params.rotationOrder=descriptor->order;
        params.preserveInputUp=descriptor->preserveInputUp;params.worldUpDirection.reset();
        if(!mask("Rotation",true,&params.affectRotation) ||
           !read("inputs:upVector",GfVec3d(0,1,0),&params.localUpVector) ||
           !read("inputs:rotationOffset",GfVec3d(0),&params.rotationOffsetDegrees))return false;
        if(descriptor->aimVectorAuthored) {
            if(!read("inputs:aimVector",GfVec3d(1,0,0),&params.localAimVector))return false;
        } else params.localAimVector=descriptor->aimAxisFallback;
        GfVec3d up(0,1,0);
        if(!read("inputs:worldUpVector",up,&up))return false;
        GfVec3d target(0);double total=0;
        for(const auto &source:input->sources) {
            if(!RigExecConstraintFrameUsable(source.frame) || !std::isfinite(source.normalizedWeight) ||
               source.normalizedWeight<0)return Fail(error,"invalid aim source frame/weight");
            target+=source.frame.Origin()*source.normalizedWeight;total+=source.normalizedWeight;
        }
        if(!(total>0))return Fail(error,"aim sources have no positive total weight");
        input->aimTarget=target/total;
        if(descriptor->worldUpType=="sceneUp")params.worldUpDirection=descriptor->sceneUp;
        else if(descriptor->worldUpType=="vector")params.worldUpDirection=up;
        else if(descriptor->worldUpType=="objectUp") {
            if(descriptor->worldUpObject.IsEmpty())params.worldUpDirection=GfVec3d(-input->incoming.Origin());
            else {
                if(!input->worldUp)return Fail(error,"aim world-up producer unavailable");
                params.worldUpDirection=input->worldUp->Origin()-input->incoming.Origin();
            }
        } else if(descriptor->worldUpType=="objectRotationUp") {
            if(descriptor->worldUpObject.IsEmpty())params.worldUpDirection=up;
            else {
                GfMatrix4d matrix(1.0);
                if(!input->worldUp || !RigExecPointsToMatrix(RigExecIdentityLandmarks(),input->worldUp->points,&matrix))
                    return Fail(error,"aim world-up producer is unavailable or degenerate");
                params.worldUpDirection=(descriptor->worldUpRotationOnly?matrix.GetOrthonormalized(false):matrix)
                    .ExtractRotation().TransformDir(up);
            }
        }
        return true;
    }
    case RigExecConstraintKind::SingleChainIk:return true;
    case RigExecConstraintKind::IkFkMatch:return true;
    case RigExecConstraintKind::Harmonics:return true;
    }
    return Fail(error,"unknown constraint input kind");
}

}
