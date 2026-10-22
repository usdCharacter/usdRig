#include "constraintGraphBinding.h"
#include "poseProgram.h"
#include <algorithm>
#include <cmath>
#include "pxr/base/vt/types.h"
#include "pxr/base/tf/staticTokens.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExec/frameExtraction.h"
#include "pxr/base/gf/matrix4d.h"
namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    ((rigExecAimTarget, "rigExec:aimTarget"))
    ((rigExecSources, "rigExec:sources"))
    ((rigExecPoleVectorObjects, "rigExec:poleVectorObjects"))
    ((rigExecEffector, "rigExec:effector"))
    ((rigExecWorldUpObject, "rigExec:worldUpObject"))
);
namespace {
bool Fail(std::string *error,const std::string &message) {if(error)*error=message;return false;}
void Add(std::vector<RigExecValueId> *reads,RigExecValueId id) {if(id!=UINT64_MAX)reads->push_back(id);}
bool Frame(const RigExecTypedValueStore &store,RigExecValueId id,RigExecPointFrame *out) {
    VtValue value;
    if(!RigExecReadSceneGraphValue(store,id,&value) || !value.IsHolding<RigExecPointFrame>())return false;
    *out=value.UncheckedGet<RigExecPointFrame>();return RigExecConstraintFrameUsable(*out);
}
}
bool RigExecBindConstraintGraph(const RigExecSceneDescriptors &scene,
    const RigExecSceneConstraintDescriptor &source,const RigExecSceneGraphBindingContext &context,
    const RigExecConstraintGraphTarget &target,RigExecConstraintGraphBinding *destination,std::string *error) {
    if(!destination || !context.resolve || target.incoming==UINT64_MAX || target.output==UINT64_MAX)
        return Fail(error,"invalid constraint graph target");
    RigExecConstraintGraphBinding bound;bound.structural=source;
    bound.structural.inputs.clear();bound.structural.typedReads.clear();bound.target=target;
    if(source.useAnimatedTs) {bound.target.chainRests.clear();bound.target.restLive.clear();}
    if(!RigExecBindGraphInputs(source.inputs,source.typedReads,source.path,context,&bound.inputs,&bound.reads,error))return false;
    Add(&bound.reads,target.incoming);Add(&bound.reads,target.carry);Add(&bound.reads,target.weight);
    for(auto id:bound.target.chainRests)Add(&bound.reads,id);
    const RigExecSceneCompileInputs facts(scene);
    const auto bind=[&](const SdfPath &path,const TfToken &name,RigExecValueId *id) {
        if(path.IsEmpty())return true;
        RigExecSceneGraphReadRequest request;
        request.consumer=source.path.AppendProperty(name);request.source=path;request.reader=source.path;
        request.domain=RigExecSceneValueDomain::Pose;
        const auto *relation=facts.Relationship(request.consumer);
        request.phase=relation?relation->readPhase:RigExecSceneBasePhase;
        if(!context.resolve(request,id,error))return false;
        if(*id==UINT64_MAX)return Fail(error,"constraint resolver returned no value ID");
        Add(&bound.reads,*id);return true;
    };
    const auto many=[&](const SdfPathVector &paths,const TfToken &name,std::vector<RigExecValueId> *ids) {
        for(const auto &path:paths) {ids->push_back(UINT64_MAX);if(!bind(path,name,&ids->back()))return false;}return true;
    };
    if(!many(source.sources,source.record.kind==RigExecConstraintKind::Aim && source.preserveInputUp?_tokens->rigExecAimTarget:_tokens->rigExecSources,&bound.sources) ||
       !many(source.poleObjects,_tokens->rigExecPoleVectorObjects,&bound.poles) ||
       !bind(source.effector,_tokens->rigExecEffector,&bound.effector) ||
       !bind(source.worldUpObject,_tokens->rigExecWorldUpObject,&bound.worldUp))return false;
    for(const auto &joint:source.ikChain) {
        RigExecSceneGraphReadRequest request;
        request.consumer=source.path;request.source=joint;request.reader=source.path;
        request.domain=RigExecSceneValueDomain::Pose;request.phase=RigExecScenePrecedingPhase;
        bound.chain.push_back(UINT64_MAX);
        if(!context.resolve(request,&bound.chain.back(),error))return false;
        if(bound.chain.back()==UINT64_MAX)return Fail(error,"constraint chain resolver returned no value ID");
        Add(&bound.reads,bound.chain.back());
    }
    if(source.record.kind==RigExecConstraintKind::SingleChainIk &&
        ((!source.useAnimatedTs && target.chainRests.size()!=bound.chain.size()) ||
         target.chainOutputs.size()!=bound.chain.size() ||
         (!target.restLive.empty() && target.restLive.size()!=bound.chain.size())))
        return Fail(error,"single-chain IK graph version cardinality mismatch");
    for(auto id:bound.target.chainOutputs)if(id==UINT64_MAX)
        return Fail(error,"constraint chain output has no value ID");
    if(!source.useAnimatedTs)for(auto id:bound.target.chainRests)if(id==UINT64_MAX)
        return Fail(error,"constraint chain rest has no value ID");
    std::sort(bound.reads.begin(),bound.reads.end());bound.reads.erase(std::unique(bound.reads.begin(),bound.reads.end()),bound.reads.end());
    *destination=std::move(bound);return true;
}
namespace {
// RigExecHarmonicsConstraint: the bakedHarmonics.cpp body over typed values.
// The capture is the incoming origin in rigExec:space and is taken before any
// gate; an unusable input captures nothing, which ends the history before it.
bool RunHarmonics(const RigExecConstraintGraphBinding &bound,RigExecTypedValueStore *store,
    RigExecWindowRun *window,std::string *error) {
    const auto &descriptor=bound.structural;
    const auto publish=[&](const RigExecPointFrame &frame,bool blocked,const std::string &reason) {
        store->Publish(bound.target.output,frame,blocked,false,1,reason);
    };
    const auto capture=[&](RigExecWindowPacketPtr packet) {
        if(window && window->capture && !RigExecWindowPacketsEqual(*window->capture,packet))*window->capture=std::move(packet);
    };
    RigExecPointFrame input;
    if(!Frame(*store,bound.target.incoming,&input)) {
        capture(nullptr);
        RigExecPointFrame invalid;invalid.flags=0;publish(invalid,true,"constraint incoming producer unavailable");
        return Fail(error,"constraint incoming producer unavailable");
    }
    const auto passThrough=[&](const std::string &reason) {publish(input,false,reason);return reason.empty() || Fail(error,reason);};
    GfMatrix4d space(1.0),toSpace(1.0);bool spaceUsable=true;
    const bool hasSpace=bound.target.carry!=UINT64_MAX;
    if(hasSpace) {
        VtValue matrix;
        spaceUsable=RigExecReadSceneGraphValue(*store,bound.target.carry,&matrix,true) && matrix.IsHolding<GfMatrix4d>();
        if(spaceUsable) {
            space=matrix.UncheckedGet<GfMatrix4d>();const double det=space.GetDeterminant();
            spaceUsable=std::isfinite(det) && det!=0.0;
            if(spaceUsable)toSpace=space.GetInverse();
        }
    }
    RigExecWindowPacketPtr captured;
    if(spaceUsable && RigExecConstraintFrameUsable(input)) {
        const GfVec3d p=hasSpace?toSpace.Transform(input.Origin()):input.Origin();
        if(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
            auto packet=std::make_shared<RigExecWindowPacket>();packet->scalars={p[0],p[1],p[2]};captured=std::move(packet);
        }
    }
    capture(captured);
    // A history tick that no windowed op reads the result of only captures.
    if(window && window->captureOnly)return passThrough({});
    const auto read=[&](const char *name,VtValue *value) {return RigExecReadBoundGraphInput(bound.inputs,name,*store,value,error);};
    VtValue enabled;
    if(!read("inputs:enabled",&enabled))return passThrough(error?*error:"constraint enabled input unavailable");
    if(enabled.IsHolding<bool>() && !enabled.UncheckedGet<bool>())return passThrough({});
    double weight=1.0;
    if(bound.target.weight!=UINT64_MAX) {
        VtValue field;
        if(!RigExecReadSceneGraphValue(*store,bound.target.weight,&field) ||
           !field.IsHolding<VtFloatArray>() || field.UncheckedGet<VtFloatArray>().size()!=1)
            return passThrough("constraint weight field producer unavailable");
        weight=field.UncheckedGet<VtFloatArray>()[0];
    } else {
        VtValue value;
        if(!read("inputs:defaultWeight",&value))return passThrough(error?*error:"constraint weight unavailable");
        if(value.IsHolding<float>())weight=value.UncheckedGet<float>();
        if(!std::isfinite(weight) || weight<0.0 || weight>1.0)return passThrough("constraint weight outside finite [0,1]");
    }
    if(weight<=0.0)return passThrough({});
    if(!captured)return passThrough(spaceUsable?"harmonics constraint could not measure its target":
        "harmonics constraint could not measure its target in a degenerate rigExec:space");
    // The usable tail of the ring, then this tick, as one position array.
    const RigExecWindowRing none;
    const RigExecWindowRing &ring=window && window->ring?*window->ring:none;
    size_t begin=ring.size();
    while(begin>0 && ring[begin-1] && ring[begin-1]->scalars.size()==3)--begin;
    std::vector<double> positions;positions.reserve(3*(ring.size()-begin+1));
    for(size_t k=begin;k<ring.size();++k)positions.insert(positions.end(),ring[k]->scalars.begin(),ring[k]->scalars.end());
    positions.insert(positions.end(),captured->scalars.begin(),captured->scalars.end());
    const auto scalar=[&](const char *name,double fallback) {
        VtValue value;if(!read(name,&value))return fallback;
        if(value.IsHolding<float>())return double(value.UncheckedGet<float>());
        return value.IsHolding<double>()?value.UncheckedGet<double>():fallback;
    };
    RigExecHarmonicsSettings settings;
    settings.waves=descriptor.harmonicsWaves;settings.waveLength=descriptor.harmonicsWaveLength;
    settings.frequency=descriptor.harmonicsFrequency;settings.ignoreFirstFrame=descriptor.harmonicsIgnoreFirstFrame;
    RigExecHarmonicsParameters params;
    params.amplitude=scalar("inputs:amplitude",1.0);params.decay=scalar("inputs:decay",3.0);
    params.termination=scalar("inputs:termination",0.0);params.matchVelocity=descriptor.harmonicsMatchVelocity;
    GfVec3d axis(1.0);
    {VtValue value;if(read("inputs:axisAmplitude",&value) && value.IsHolding<GfVec3d>())axis=value.UncheckedGet<GfVec3d>();}
    for(size_t a=0;a<3;++a)params.axisAmplitude[a]=axis[a];
    if(!std::isfinite(params.amplitude) || !std::isfinite(params.decay) || !std::isfinite(params.termination) ||
       !std::isfinite(axis[0]) || !std::isfinite(axis[1]) || !std::isfinite(axis[2]))
        return passThrough("harmonics constraint has a non-finite input");
    double ring3[3];
    // Fewer than two samples: no history yet, nothing rings.
    if(!RigExecHarmonicsOffset(settings,params,reinterpret_cast<const double(*)[3]>(positions.data()),positions.size()/3,ring3))
        return passThrough({});
    GfVec3d offset(ring3[0],ring3[1],ring3[2]);
    if(hasSpace)offset=space.TransformDir(offset);
    offset*=weight;
    RigExecPointFrame candidate=input;
    for(auto &q:candidate.points)q+=offset;
    if(!RigExecConstraintFrameUsable(candidate))return passThrough("harmonics constraint produced an invalid or degenerate frame");
    publish(candidate,false,{});return true;
}
bool MatrixOf(const RigExecTypedValueStore &store,RigExecValueId id,GfMatrix4d *out) {
    const auto *frame=store.Read<RigExecPointFrame>(id);
    return frame && RigExecConstraintFrameUsable(*frame) &&
        RigExecPointsToMatrix(RigExecIdentityLandmarks(),frame->points,out);
}
// RigExecBakedMatchChannelsOf over the avar values the control composes from.
RigExecIkFkMatchChannels ChannelsOf(const RigExecTypedValueStore &store,
    const std::array<RigExecValueId,13> &ids,bool scaleAvars) {
    const auto scalar=[&](size_t k,double fallback) {const auto *v=store.Read<double>(ids[k]);return v?*v:fallback;};
    RigExecIkFkMatchChannels channels;
    for(int k=0;k<3;++k) {
        channels.t[k]=scalar(size_t(1+k),0);
        channels.s[k]=scaleAvars?scalar(size_t(4+k),1):1.0;
        channels.r[k]=scalar(size_t(7+k),0);
    }
    channels.spin=scalar(10,0);channels.units=scalar(0,1);
    if(const auto *sign=store.Read<GfVec3d>(ids[12]))
        for(int k=0;k<3;++k)channels.sign[k]=RigExecNormalizeRotationSign((*sign)[k]);
    if(const auto *order=store.Read<TfToken>(ids[11]);order && order->size()==3)
        std::copy(order->GetText(),order->GetText()+3,channels.order);
    return channels;
}
// The pole switch's inputs as RigExecRunBoundSpaceSwitch gathers them.
bool PoleSwitchInputs(const RigExecBoundSpaceSwitch &packet,const RigExecTypedValueStore &values,
    RigExecSpaceSwitchInputs *input) {
    const auto matrix=[&](RigExecValueId id,GfMatrix4d *out) {
        if(id==UINT64_MAX){*out=GfMatrix4d(1.0);return true;}
        VtValue value;if(!RigExecReadSceneGraphValue(values,id,&value,true) || !value.IsHolding<GfMatrix4d>())return false;
        *out=value.UncheckedGet<GfMatrix4d>();return true;
    };
    *input=packet.inputs;VtValue active;
    input->active=packet.tokenIndex?0.0:packet.activeFallback;
    if(RigExecReadGraphTypedRead(packet.active,values,&active,nullptr)) {
        if(packet.tokenIndex && active.IsHolding<TfToken>()) {
            const auto found=std::find(packet.labels.begin(),packet.labels.end(),active.UncheckedGet<TfToken>());
            if(found!=packet.labels.end())input->active=double(found-packet.labels.begin());
        } else if(active.IsHolding<double>())input->active=active.UncheckedGet<double>();
    }
    GfMatrix4d parentExpression(1.0);
    bool usable=matrix(packet.avars,&input->avars) && matrix(packet.posedDefault,&input->posedDefault) &&
        matrix(packet.parentExpression,&parentExpression) && matrix(packet.parentPosed,&input->parentPosed) &&
        matrix(packet.parentDefault,&input->parentDefault);
    input->parentDefaultInverse=parentExpression.GetInverse();
    input->parentDefault=RigExecPoseRoundTrip(input->parentDefault);
    if(input->hasCarry) {
        usable=usable && matrix(packet.spaceDefault,&input->spaceDefault) && matrix(packet.spacePosed,&input->spacePosed);
        input->spaceDefault=RigExecPoseRoundTrip(input->spaceDefault);
    }
    for(size_t k=0;k<input->sources.size();++k)if(!input->sources[k].world) {
        usable=usable && matrix(packet.sourceDefault[k],&input->sources[k].defaultSpace) &&
            matrix(packet.sourcePosed[k],&input->sources[k].posedSpace);
        input->sources[k].defaultSpace=RigExecPoseRoundTrip(input->sources[k].defaultSpace);
    }
    return usable;
}
// RigExecIkFkMatchConstraint: RigExecBakedRunIkFkMatch's arithmetic, then
// RigExecBakedPresentIkFkMatches' presentation of the five controls and their
// riders. Every output is published on every run.
bool RunIkFkMatch(const RigExecConstraintGraphBinding &bound,RigExecTypedValueStore *store,std::string *error) {
    const auto &m=*bound.match;
    std::array<double,11> ikValues{};std::array<double,20> fkValues{};
    std::array<RigExecPointFrame,5> shown;
    bool valuesValid=false;unsigned shownMask=0;
    const std::string *diagnostic=nullptr;const std::string none;
    // Values the match could not solve are not published, as natively.
    const auto finish=[&]() {
        if(valuesValid) {
            store->Publish(m.ikValues,VtValue(VtDoubleArray(ikValues.begin(),ikValues.end())));
            store->Publish(m.fkValues,VtValue(VtDoubleArray(fkValues.begin(),fkValues.end())));
        } else {
            const std::string &reason=diagnostic?*diagnostic:none;
            store->Publish(m.ikValues,VtValue(),true,false,0,reason);
            store->Publish(m.fkValues,VtValue(),true,false,0,reason);
        }
        GfMatrix4d delta[5];bool deltaOk[5]={false,false,false,false,false};
        for(size_t k=0;k<5;++k) {
            if(!(shownMask&(1u<<k))) {store->Copy(m.presented[k],m.published[k],false);continue;}
            GfMatrix4d before,after;
            const auto *published=store->Read<RigExecPointFrame>(m.published[k]);
            deltaOk[k]=published &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),published->points,&before) &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),shown[k].points,&after);
            if(deltaOk[k])delta[k]=before.GetInverse()*after;
            store->Publish(m.presented[k],shown[k]);
        }
        for(const auto &rider:m.riders) {
            const auto *published=store->Read<RigExecPointFrame>(rider.published);GfMatrix4d frame;
            if(deltaOk[size_t(rider.rides)] && published &&
               RigExecPointsToMatrix(RigExecIdentityLandmarks(),published->points,&frame))
                store->Publish(rider.presented,RigExecFrameFromMatrix(frame*delta[size_t(rider.rides)]));
            else store->Copy(rider.presented,rider.published,false);
        }
        return !diagnostic || Fail(error,*diagnostic);
    };
    // An unreadable input answers its fallback, as the baked binding does.
    std::string unavailable;
    const auto read=[&](const char *name,VtValue *value) {return RigExecReadBoundGraphInput(bound.inputs,name,*store,value,&unavailable);};
    VtValue value;
    if(read("inputs:enabled",&value) && value.IsHolding<bool>() && !value.UncheckedGet<bool>())return finish();
    double weight=1.0;
    if(read("inputs:defaultWeight",&value) && value.IsHolding<float>())weight=double(value.UncheckedGet<float>());
    if(!std::isfinite(weight))weight=0.0;
    const char *const scalarNames[7]={"inputs:switch","inputs:stretch","inputs:softDistance",
        "inputs:upperScale","inputs:lowerScale","inputs:pin","inputs:twist"};
    const double fallbacks[7]={0,1,0,1,1,0,0};
    double scalars[7];
    for(size_t k=0;k<7;++k) {
        scalars[k]=fallbacks[k];
        if(!read(scalarNames[k],&value))continue;
        if(value.IsHolding<float>())scalars[k]=double(value.UncheckedGet<float>());
        else if(value.IsHolding<double>())scalars[k]=value.UncheckedGet<double>();
    }
    // Sources: joints 0-2, IK root 3, effector 4, IK control 5, pole 6, FK
    // controls 7-9. Rests: joints 0-2, then the controls' at 3-7.
    GfMatrix4d input[10],rest[8];
    bool usable=true;
    for(size_t i=0;usable && i<10;++i)usable=MatrixOf(*store,m.sources[i],&input[i]);
    for(size_t i=0;usable && i<8;++i)usable=MatrixOf(*store,m.rests[i],&rest[i]);
    if(!usable){diagnostic=&m.unreadable;return finish();}
    const GfMatrix4d *control=input+5;
    RigExecIkFkMatchChannels channels[5];
    for(size_t i=0;i<5;++i)channels[i]=ChannelsOf(*store,m.channels[i],m.scaleAvars[i]!=0);
    RigExecLimbStretch limb;
    limb.stretch=scalars[1];limb.softDistance=scalars[2];limb.upperScale=scalars[3];
    limb.lowerScale=scalars[4];limb.pin=scalars[5];limb.scaleCalibration=m.scaleCalibration;
    // The IK solve's space carries its rest lengths and soft distance.
    double space[16];const double *spaceMatrix=nullptr;
    if(bound.target.carry!=UINT64_MAX) {
        GfMatrix4d spaceRest(1.0);VtValue posed;
        if(MatrixOf(*store,m.spaceRest,&spaceRest) &&
           RigExecReadSceneGraphValue(*store,bound.target.carry,&posed,true) && posed.IsHolding<GfMatrix4d>() &&
           std::isfinite(posed.UncheckedGet<GfMatrix4d>()[3][0])) {
            const GfMatrix4d carry=spaceRest.GetInverse()*posed.UncheckedGet<GfMatrix4d>();
            std::copy(carry.data(),carry.data()+16,space);spaceMatrix=space;
        }
    }
    double restOrigins[3][3],restUpper=0,restLower=0,spaceFactor=1;
    for(size_t i=0;i<3;++i)for(int k=0;k<3;++k)restOrigins[i][k]=rest[i][3][k];
    RigExecIkFkMatchRestLengths(restOrigins,spaceMatrix,m.upperLengthOffset,m.lowerLengthOffset,
        &restUpper,&restLower,&spaceFactor);
    limb.softDistance*=spaceFactor;
    RigExecIkFkMatchIkInputs ikIn;
    for(size_t i=0;i<3;++i)ikIn.joints[i]=input[i].data();
    ikIn.root=input[3].data();ikIn.effectorPosed=input[4].data();ikIn.ikPosed=control[0].data();
    ikIn.ik=channels[0];
    ikIn.twistRadians=scalars[6]*std::acos(-1.0)/180.0;
    ikIn.poleDistance=(rest[4].ExtractTranslation()-rest[1].ExtractTranslation()).GetLength()*spaceFactor;
    ikIn.polePosed=control[1].data();
    ikIn.hasStretch=m.stretchLimb;ikIn.limb=limb;ikIn.restUpper=restUpper;ikIn.restLower=restLower;
    RigExecIkFkMatchIkResult ik;
    RigExecIkFkMatchIk(ikIn,&ik);
    // The pole's parent once the IK control has moved: its own space switch
    // re-run with every source that rides the IK control carried by the
    // move, or its namespace parent carried the same way.
    RigExecIkFkMatchChannels pole=channels[1];
    double poleFrame[16];bool poleOk=false;
    if(ik.ok) {
        GfMatrix4d ikNew;std::copy(ik.ikFrame,ik.ikFrame+16,ikNew.data());
        const GfMatrix4d moved=control[0].GetInverse()*ikNew;
        const auto composed=ikFkMatchKernel::Compose(channels[1]);
        GfMatrix4d avars;std::copy(composed.begin(),composed.end(),avars.data());
        GfMatrix4d parent=avars.GetInverse()*control[1];
        // An authored posed:space is the pole's pose: no switch composes it.
        const auto *posedSpace=store->Read<GfMatrix4d>(m.polePosedSpace);
        const bool posedAuthored=posedSpace && (store->values[size_t(m.polePosedSpace)].authoritative ||
            *posedSpace!=GfMatrix4d(1.0));
        if(m.poleSwitch && !posedAuthored) {
            RigExecSpaceSwitchInputs inputs;
            if(PoleSwitchInputs(m.poleSwitchBinding,*store,&inputs)) {
                inputs.avars=GfMatrix4d(1.0);
                for(size_t k=0;k<inputs.sources.size() && k<m.poleSourceRides.size();++k)
                    if(m.poleSourceRides[k] && !inputs.sources[k].world)
                        inputs.sources[k].posedSpace=inputs.sources[k].posedSpace*moved;
                RigExecPointFrame frame;GfMatrix4d switched(1.0);
                if(RigExecRunSpaceSwitch(m.poleSwitchBinding.record,inputs,&frame) &&
                   RigExecConstraintFrameUsable(frame) &&
                   RigExecPointsToMatrix(RigExecIdentityLandmarks(),frame.points,&switched))parent=switched;
            }
        } else if(m.poleParentRides)parent=parent*moved;
        poleOk=RigExecIkFkMatchPole(ik.poleTarget,parent.data(),&pole,poleFrame);
    }
    RigExecIkFkMatchFkInputs fkIn;GfMatrix4d offsets[3];
    for(size_t i=0;i<3;++i) {
        fkIn.joints[i]=input[i].data();fkIn.fkPosed[i]=control[2+i].data();fkIn.fk[i]=channels[2+i];
        offsets[i]=rest[5+i]*rest[i].GetInverse();fkIn.offsets[i]=offsets[i].data();
    }
    fkIn.hasStretch=m.stretchLimb;fkIn.upperScale=limb.upperScale;fkIn.lowerScale=limb.lowerScale;
    fkIn.restUpper=restUpper;fkIn.restLower=restLower;
    RigExecIkFkMatchFkResult fk;
    RigExecIkFkMatchFk(fkIn,&fk);
    if(ik.ok && poleOk && fk.ok) {
        RigExecIkFkMatchPack(ik,pole,fk,ikValues.data(),fkValues.data());valuesValid=true;
    } else diagnostic=&m.unmatched;
    const auto show=[&](size_t k,const double *frame) {
        GfMatrix4d matrix;std::copy(frame,frame+16,matrix.data());
        shown[k]=RigExecFrameFromMatrix(matrix);shownMask|=1u<<k;
    };
    switch(RigExecIkFkMatchShownSide(scalars[0],m.ikValue,weight)) {
    case RigExecIkFkMatchShown::IkSide:
        if(ik.ok && poleOk){show(0,ik.ikFrame);show(1,poleFrame);}
        break;
    case RigExecIkFkMatchShown::FkSide:
        if(fk.ok)for(size_t i=0;i<3;++i)show(2+i,fk.frames[i]);
        break;
    case RigExecIkFkMatchShown::None:break;
    }
    return finish();
}
}
bool RigExecRunConstraintGraph(const RigExecConstraintGraphBinding &bound,RigExecTypedValueStore *store,std::string *error,RigExecWindowRun *window) {
    if(!store || bound.target.output>=store->values.size())return Fail(error,"invalid constraint graph storage");
    if(bound.structural.record.kind==RigExecConstraintKind::Harmonics)return RunHarmonics(bound,store,window,error);
    if(bound.match)return RunIkFkMatch(bound,store,error);
    auto descriptor=bound.structural;RigExecConstraintInputs input;
    std::vector<char> chainAvailable;
    const auto publish=[&](const RigExecPointFrame &frame,bool blocked,const std::string &reason) {
        for(size_t i=0;i<input.chain.size();++i)
            if(bound.target.chainOutputs[i]!=bound.target.output)
                store->Publish(bound.target.chainOutputs[i],input.chain[i],!chainAvailable[i],false,1,reason);
        store->Publish(bound.target.output,frame,blocked,false,1,reason);
    };
    if(!Frame(*store,bound.target.incoming,&input.incoming)) {
        RigExecPointFrame invalid;invalid.flags=0;publish(invalid,true,"constraint incoming producer unavailable");
        for(auto output:bound.target.chainOutputs)if(output<store->values.size())
            if(output!=bound.target.output)store->Publish(output,invalid,true,false,1,"constraint incoming producer unavailable");
        return Fail(error,"constraint incoming producer unavailable");
    }
    // Stage current entering values and publish each output once, even when
    // disabled, zero-weight, or an unavailable source passes the body through.
    input.chain.resize(bound.chain.size());
    for(size_t i=0;i<bound.chain.size();++i) {
        RigExecPointFrame current;current.flags=0;
        const bool available=Frame(*store,bound.chain[i],&current);
        input.chain[i]=current;
        if(i>=bound.target.chainOutputs.size() || bound.target.chainOutputs[i]>=store->values.size())
            return Fail(error,"invalid constraint chain output storage");
        chainAvailable.push_back(available);
    }
    const auto failure=[&](const std::string &reason) {publish(input.incoming,false,reason);return Fail(error,reason);};
    const auto read=[&](const char *name,VtValue *value) {
        if(std::string(name)=="inputs:defaultWeight" && bound.target.weight!=UINT64_MAX) {
            VtValue field;
            if(!RigExecReadSceneGraphValue(*store,bound.target.weight,&field) ||
                !field.IsHolding<VtFloatArray>() || field.UncheckedGet<VtFloatArray>().size()!=1)
                return Fail(error,"constraint weight field producer unavailable");
            *value=VtValue(field.UncheckedGet<VtFloatArray>()[0]);return true;
        }
        return RigExecReadBoundGraphInput(bound.inputs,name,*store,value,error);
    };
    // Enabled and zero weight gate precede source reads, preserving dormant
    // pass-through even when a disconnected source is malformed.
    VtValue enabledValue,weightValue;
    if(!read("inputs:enabled",&enabledValue))return failure(error?*error:"constraint enabled input unavailable");
    const bool enabled=!enabledValue.IsHolding<bool>() || enabledValue.UncheckedGet<bool>();
    if(!enabled) {publish(input.incoming,false,{});return true;}
    if(!read("inputs:defaultWeight",&weightValue))return failure(error?*error:"constraint weight unavailable");
    const float weight=weightValue.IsHolding<float>()?weightValue.UncheckedGet<float>():1.0f;
    if(!std::isfinite(weight) || weight<0 || weight>1)return failure("constraint weight outside finite [0,1]");
    if(weight<=0) {publish(input.incoming,false,{});return true;}
    if(bound.target.carry!=UINT64_MAX) {
        VtValue matrix;
        if(!RigExecReadSceneGraphValue(*store,bound.target.carry,&matrix,true) || !matrix.IsHolding<GfMatrix4d>())
            return failure("constraint carry producer unavailable");
        input.carry=matrix.UncheckedGet<GfMatrix4d>();
    }
    input.sources.resize(bound.sources.size());
    for(size_t i=0;i<bound.sources.size();++i)if(!Frame(*store,bound.sources[i],&input.sources[i].frame))return failure("constraint source producer unavailable");
    input.poleObjects.resize(bound.poles.size());
    for(size_t i=0;i<bound.poles.size();++i)if(!Frame(*store,bound.poles[i],&input.poleObjects[i]))return failure("constraint pole producer unavailable");
    if(bound.worldUp!=UINT64_MAX) {RigExecPointFrame up;if(!Frame(*store,bound.worldUp,&up))return failure("constraint world-up producer unavailable");input.worldUp=up;}
    if(descriptor.record.kind==RigExecConstraintKind::SingleChainIk) {
        if(!Frame(*store,bound.effector,&input.effector))return failure("single-chain IK effector producer unavailable");
        for(const auto &frame:input.chain)if(!RigExecConstraintFrameUsable(frame))return failure("single-chain IK chain producer unavailable");
        if(!descriptor.useAnimatedTs) {
            std::vector<RigExecPointFrame> rests(bound.chain.size()),prepared;
            for(size_t i=0;i<rests.size();++i) {
                if(i<bound.target.restLive.size() && bound.target.restLive[i])rests[i]=input.chain[i];
                else if(!Frame(*store,bound.target.chainRests[i],&rests[i]))return failure("single-chain IK rest producer unavailable");
            }
            if(!RigExecPrepareRestDerivedIkChain(input.chain,rests,&prepared))return failure("single-chain IK rest-derived preparation failed");
            input.chain=std::move(prepared);
        }
    }
    bool refreshedEnabled=true;float refreshedWeight=1;
    if(!RigExecRefreshConstraintParameters(&descriptor,read,&input,&refreshedEnabled,&refreshedWeight,error))
        return failure(error?*error:"constraint parameter refresh failed");
    RigExecConstraintResult result;
    if(!RigExecRunConstraint(descriptor.record,input,&result,error))return failure(error?*error:"constraint kernel failed");
    if(descriptor.record.kind==RigExecConstraintKind::SingleChainIk) {
        if(result.chain.size()!=bound.target.chainOutputs.size())return failure("single-chain IK result cardinality mismatch");
        for(size_t i=0;i<result.chain.size();++i) {
            input.chain[i]=result.chain[i];chainAvailable[i]=true;
            if(bound.target.chainOutputs[i]==bound.target.output)result.frame=result.chain[i];
        }
    }
    publish(result.frame,false,{});return true;
}
}
