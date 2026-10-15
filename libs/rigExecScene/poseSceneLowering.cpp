#include "poseSceneLowering.h"
#include "pxr/base/gf/quatf.h"
#include "pxr/base/vt/types.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <type_traits>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) {if(error)*error=message;return false;}
bool Provider(const RigExecSceneCompileInputs &source,const SdfPath &path) {
    const auto *node=source.Node(path);if(!node || !node->fact.active)return false;
    const auto type=node->fact.type;
    return type=="RigExecJoint" || type=="RigExecControl" || type=="RigExecSphereWeight" ||
        type=="RigExecPlaneWeight" || type=="RigExecCurveWeight";
}
bool Driver(const RigExecSceneCompileInputs &source,const SdfPath &path) {
    const auto *node=source.Node(path);return node && node->fact.active &&
        (node->fact.type=="RigExecJoint" || node->fact.type=="RigExecControl");
}
template<class T> bool Raw(const RigExecSceneCompileInputs &source,const SdfPath &path,
    const T &fallback,T *value,std::string *error) {
    *value=fallback;if(!source.Attribute(path))return true;
    RigExecSceneBoundInput binding;VtValue boxed;
    if(!source.Bind(path,RigExecSceneReadRoute::Raw,&binding,error) ||
       !source.Read(binding,UsdTimeCode::Default(),&boxed,nullptr,error))return false;
    if(boxed.IsHolding<T>())*value=boxed.UncheckedGet<T>();return true;
}
}
bool RigExecLowerSceneSpaceSwitch(const RigExecSceneDescriptors &scene,const SdfPath &path,
    RigExecSceneSpaceSwitchDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null space switch descriptor");
    const RigExecSceneCompileInputs source(scene);const auto *node=source.Node(path);
    if(!node || !node->fact.active || node->fact.type!="RigExecSpaceSwitch")
        return Fail(error,"missing active space switch: "+path.GetString());
    RigExecSceneSpaceSwitchDescriptor result;result.path=path;
    const auto property=[&](const char *name){return path.AppendProperty(TfToken(name));};
    const auto targets=source.Targets(property("rigExec:target"));
    if(targets.size()!=1 || !Provider(source,targets[0]))
        return Fail(error,"space switch requires exactly one frame provider target: "+path.GetString());
    result.target=targets[0];
    VtFloatArray weights;if(!Raw(source,property("inputs:sourceWeights"),VtFloatArray{},&weights,error))return false;
    if(!weights.empty())return Fail(error,"space switch does not read inputs:sourceWeights: "+path.GetString());
    const auto sources=source.Targets(property("rigExec:sources"));
    if(sources.empty())return Fail(error,"space switch needs at least one source: "+path.GetString());
    VtTokenArray labels,filters;
    if(!Raw(source,property("rigExec:spaceLabels"),VtTokenArray{},&labels,error) ||
       !Raw(source,property("rigExec:rotationFilters"),VtTokenArray{},&filters,error) ||
       !Raw(source,property("rigExec:twistAxis"),GfVec3d(1,0,0),&result.record.twistAxis,error))return false;
    if((!labels.empty() && labels.size()!=sources.size()) ||
       (!filters.empty() && filters.size()!=sources.size()))
        return Fail(error,"space switch labels/filters must parallel its sources: "+path.GetString());
    for(size_t k=0;k<sources.size();++k) {
        result.sources.push_back(Provider(source,sources[k])?sources[k]:SdfPath{});
        result.labels.push_back(labels.empty()?TfToken(sources[k].GetName()):labels[k]);
        const TfToken filter=filters.empty()?TfToken("all"):filters[k];
        if(!filter.IsEmpty() && filter!="all" && filter!="twist" && filter!="swing" && filter!="orient")
            return Fail(error,"unknown space switch rotation filter: "+filter.GetString());
        result.record.filters.push_back(filter=="twist"?1:filter=="swing"?2:filter=="orient"?3:0);
    }
    const auto spaces=source.Targets(property("rigExec:space"));
    if(spaces.size()>1)return Fail(error,"space switch has more than one rigExec:space: "+path.GetString());
    if(!spaces.empty() && Provider(source,spaces[0]))result.space=spaces[0];
    const auto active=source.Targets(property("rigExec:activeSpaceAttribute"));
    if(active.size()>1)return Fail(error,"space switch has more than one active attribute: "+path.GetString());
    result.activeAttribute=active.empty()?property("inputs:activeSpace"):active[0];
    if(!result.activeAttribute.IsPropertyPath())return Fail(error,"space switch active target is not a property");
    if(!Raw(source,property("inputs:activeSpace"),0.0,&result.activeFallback,error))return false;
    const auto *attribute=source.Attribute(result.activeAttribute);
    result.tokenIndex=attribute && attribute->fact.type==SdfValueTypeNames->Token;
    if(!RigExecBindSceneTypedRead(scene,result.activeAttribute,
        result.tokenIndex?SdfValueTypeNames->Token:SdfValueTypeNames->Double,&result.active,error))return false;
    const char *channels[]={"Translation","Rotation","Scale"};
    for(size_t channel=0;channel<3;++channel)for(size_t axis=0;axis<3;++axis) {
        const std::string name=std::string("inputs:affect")+channels[channel]+"XYZ"[axis];
        auto &mask=channel==0?result.record.affectTranslation:
            channel==1?result.record.affectRotation:result.record.affectScale;
        if(!Raw(source,path.AppendProperty(TfToken(name)),true,&mask[axis],error))return false;
    }
    *output=std::move(result);return true;
}
bool RigExecResolveSceneSpaceSwitch(const RigExecSceneDescriptors &scene,
    const RigExecSceneSpaceSwitchDescriptor &record,UsdTimeCode time,
    const std::map<SdfPath,VtValue> &delivered,double *active,std::string *error) {
    if(!active)return Fail(error,"null space switch selector");
    *active=record.tokenIndex?0.0:record.activeFallback;VtValue value;
    if(!RigExecResolveSceneTypedRead(scene,record.active,time,delivered,&value,error))return true;
    if(record.tokenIndex && value.IsHolding<TfToken>()) {
        const auto found=std::find(record.labels.begin(),record.labels.end(),value.UncheckedGet<TfToken>());
        if(found!=record.labels.end())*active=double(found-record.labels.begin());
    } else if(value.IsHolding<double>())*active=value.UncheckedGet<double>();
    return true;
}
// The frame drivers of an interpolator or ML mover: each one's measuring
// ancestor is the nearest joint or control above it under the rig.
static bool ResolveDrivers(const RigExecSceneDescriptors &scene,const RigExecSceneCompileInputs &source,
    const SdfPathVector &drivers,const char *what,RigExecScenePoseInterpolatorDescriptor *result,
    std::string *error) {
    for(size_t k=0;k<drivers.size();++k) {
        const SdfPath driver=drivers[k].GetPrimPath();SdfPath driverParent;
        if(!Driver(source,driver))return Fail(error,std::string(what)+" driver is not a joint or control");
        for(auto parent=driver.GetParentPath();!parent.IsEmpty() && !parent.IsAbsoluteRootPath() &&
            parent!=scene.rigRoot.GetParentPath();parent=parent.GetParentPath())
            if(Driver(source,parent)) {driverParent=parent;break;}
        if(!driverParent.IsEmpty() && driverParent!=driver.GetParentPath())
            result->notes.push_back("warning: "+std::string(what)+" "+result->path.GetString()+
                " measures its driver against "+driverParent.GetString()+
                ", which is not the driver's immediate namespace parent");
        if(k==0) {result->driver=driver;result->parent=driverParent;}
        else {result->extraDrivers.push_back(driver);result->extraParents.push_back(driverParent);}
    }
    return true;
}
// A RigExecMlMover's network: frame drivers' rotations then numeric
// drivers in, one weight per RigExecMlOutput child out. Its blend inputs
// are the blend-shape revision's business, not the network's.
static bool RigExecLowerSceneMlMover(const RigExecSceneDescriptors &scene,const RigExecSceneCompileInputs &source,
    const RigExecSceneNodeDescriptor &node,RigExecScenePoseInterpolatorDescriptor *result,
    RigExecScenePoseInterpolatorDescriptor *output,std::string *error) {
    const SdfPath &path=result->path;
    const auto property=[&](const char *name){return path.AppendProperty(TfToken(name));};
    const auto drivers=source.Targets(property("rigExec:driver"));
    if(drivers.empty() && result->driverAttributes.empty())
        return Fail(error,"ML mover has no drivers: "+path.GetString());
    if(!ResolveDrivers(scene,source,drivers,"ML mover",result,error) ||
       !RigExecBindSceneTypedRead(scene,property("inputs:enabled"),SdfValueTypeNames->Bool,&result->enabled,error) ||
       !RigExecBindSceneTypedRead(scene,property("inputs:mask"),SdfValueTypeNames->Float,&result->mask,error))return false;
    VtIntArray sizes,activations;VtFloatArray alphas,weights,biases,mean,scale,defaults;
    if(!Raw(source,property("rigExec:layerSizes"),VtIntArray{},&sizes,error) ||
       !Raw(source,property("rigExec:layerActivations"),VtIntArray{},&activations,error) ||
       !Raw(source,property("rigExec:layerAlphas"),VtFloatArray{},&alphas,error) ||
       !Raw(source,property("rigExec:weights"),VtFloatArray{},&weights,error) ||
       !Raw(source,property("rigExec:biases"),VtFloatArray{},&biases,error) ||
       !Raw(source,property("rigExec:inputMean"),VtFloatArray{},&mean,error) ||
       !Raw(source,property("rigExec:inputScale"),VtFloatArray{},&scale,error) ||
       !Raw(source,property("rigExec:outputDefaults"),VtFloatArray{},&defaults,error))return false;
    auto network=std::make_shared<RigExecMlNetwork>();std::string why;
    if(!RigExecMlBuildNetwork(4*drivers.size()+result->driverAttributes.size(),
        {sizes.begin(),sizes.end()},{activations.begin(),activations.end()},{alphas.begin(),alphas.end()},
        {weights.begin(),weights.end()},{biases.begin(),biases.end()},network.get(),&why))
        return Fail(error,"ML mover "+path.GetString()+": "+why);
    network->inputMean.assign(mean.begin(),mean.end());
    network->inputScale.assign(scale.begin(),scale.end());
    network->defaults.assign(defaults.begin(),defaults.end());
    if(!RigExecMlNetworkValid(*network,&why))return Fail(error,"ML mover "+path.GetString()+": "+why);
    for(const auto &childPath:node.fact.children) {
        const auto *child=source.Node(childPath);
        if(!child || !child->fact.active || child->fact.type!="RigExecMlOutput")continue;
        const auto weight=childPath.AppendProperty(TfToken("outputs:weight"));
        if(const auto *attribute=source.Attribute(weight))if(attribute->fact.hasAuthoredConnections)
            return Fail(error,"ML output carries authored connections: "+weight.GetString());
        result->poseWeights.push_back(weight);
    }
    if(result->poseWeights.size()!=network->OutputCount())
        return Fail(error,"ML mover "+path.GetString()+" has "+std::to_string(result->poseWeights.size())+
            " RigExecMlOutput children for "+std::to_string(network->OutputCount())+" network outputs");
    result->record.poseCount=result->poseWeights.size();
    result->record.mlSolve=true;result->record.network=std::move(network);
    *output=std::move(*result);return true;
}
// A RigExecAnimatedMaps node: RigLogic's conditional table per
// RigExecAnimatedMap child over the numeric inputs, each map also
// republished as a constant float primvar on every bound mesh.
static bool RigExecLowerSceneAnimatedMaps(const RigExecSceneDescriptors &scene,
    const RigExecSceneCompileInputs &source,const RigExecSceneNodeDescriptor &node,
    RigExecScenePoseInterpolatorDescriptor *result,RigExecScenePoseInterpolatorDescriptor *output,
    std::string *error) {
    const SdfPath &path=result->path;
    const auto property=[&](const char *name){return path.AppendProperty(TfToken(name));};
    if(!RigExecBindSceneTypedRead(scene,property("inputs:enabled"),SdfValueTypeNames->Bool,&result->enabled,error))
        return false;
    auto table=std::make_shared<RigExecAnimatedMapsTable>();
    table->inputCount=result->driverAttributes.size();
    SdfPathVector meshes;
    for(const auto &mesh:source.Targets(property("rigExec:meshes")))meshes.push_back(mesh.GetPrimPath());
    std::set<SdfPath> primvars;
    for(const auto &childPath:node.fact.children) {
        const auto *child=source.Node(childPath);
        if(!child || !child->fact.active || child->fact.type!="RigExecAnimatedMap")continue;
        const auto at=[&](const char *name){return childPath.AppendProperty(TfToken(name));};
        VtIntArray inputs;VtFloatArray from,to,slope,cut;TfToken name;
        if(!Raw(source,at("rigExec:rowInputs"),VtIntArray{},&inputs,error) ||
           !Raw(source,at("rigExec:rowFrom"),VtFloatArray{},&from,error) ||
           !Raw(source,at("rigExec:rowTo"),VtFloatArray{},&to,error) ||
           !Raw(source,at("rigExec:rowSlope"),VtFloatArray{},&slope,error) ||
           !Raw(source,at("rigExec:rowCut"),VtFloatArray{},&cut,error) ||
           !Raw(source,at("rigExec:primvarName"),TfToken(),&name,error))return false;
        if(from.size()!=inputs.size() || to.size()!=inputs.size() || slope.size()!=inputs.size() ||
           cut.size()!=inputs.size())
            return Fail(error,"animated map rows must be parallel: "+childPath.GetString());
        std::vector<RigExecConditionalRow> rows;
        for(size_t r=0;r<inputs.size();++r) {
            if(!std::isfinite(from[r]) || !std::isfinite(to[r]) || !std::isfinite(slope[r]) || !std::isfinite(cut[r]))
                return Fail(error,"animated map row is not finite: "+childPath.GetString());
            rows.push_back({inputs[r],from[r],to[r],slope[r],cut[r]});
        }
        if(!table->AddMap(std::move(rows)))
            return Fail(error,"animated map row names no input: "+childPath.GetString());
        const auto value=childPath.AppendProperty(TfToken("outputs:value"));
        if(const auto *attribute=source.Attribute(value))if(attribute->fact.hasAuthoredConnections)
            return Fail(error,"animated map output carries authored connections: "+value.GetString());
        const size_t map=result->poseWeights.size();
        result->poseWeights.push_back(value);
        const std::string primvar="primvars:"+(name.IsEmpty()?childPath.GetName():name.GetString());
        if(!SdfPath::IsValidNamespacedIdentifier(primvar))
            return Fail(error,"animated map primvar name is not an identifier: "+childPath.GetString());
        for(const auto &mesh:meshes) {
            const SdfPath target=mesh.AppendProperty(TfToken(primvar));
            if(!primvars.insert(target).second)
                return Fail(error,"two animated maps publish "+target.GetString());
            result->primvarPaths.push_back(target);result->primvarSources.push_back(map);
        }
    }
    if(result->poseWeights.empty())return Fail(error,"animated maps have no RigExecAnimatedMap children: "+path.GetString());
    result->record.poseCount=result->poseWeights.size();
    result->record.mapsSolve=true;result->record.maps=std::move(table);
    *output=std::move(*result);return true;
}
bool RigExecLowerScenePoseInterpolator(const RigExecSceneDescriptors &scene,const SdfPath &path,
    RigExecScenePoseInterpolatorDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null pose interpolator descriptor");
    const RigExecSceneCompileInputs source(scene);const auto *node=source.Node(path);
    if(!node || !node->fact.active ||
       (node->fact.type!="RigExecPoseInterpolator" && node->fact.type!="RigExecMlMover" &&
        node->fact.type!="RigExecAnimatedMaps"))
        return Fail(error,"missing active pose interpolator: "+path.GetString());
    RigExecScenePoseInterpolatorDescriptor result;result.path=path;
    result.ml=node->fact.type=="RigExecMlMover";
    result.maps=node->fact.type=="RigExecAnimatedMaps";
    const bool typedInputs=result.ml || result.maps;
    const auto property=[&](const char *name){return path.AppendProperty(TfToken(name));};
    result.driverAttributes=source.Targets(property("rigExec:driverAttributes"));
    if(!typedInputs && result.driverAttributes.size()>3)
        return Fail(error,path.GetString()+": rigExec:driverAttributes has more than three numeric drivers");
    for(const auto &driver:result.driverAttributes) {
        const auto *attribute=driver.IsPropertyPath()?source.Attribute(driver):nullptr;
        if(!attribute)return Fail(error,"pose numeric driver is not an attribute");
        const bool isFloat=typedInputs && attribute->fact.type.GetType()==SdfValueTypeNames->Float.GetType();
        RigExecSceneTypedRead read;
        if(!RigExecBindSceneTypedRead(scene,driver,isFloat?SdfValueTypeNames->Float:SdfValueTypeNames->Double,
            &read,error))return false;
        result.numericReads.push_back(std::move(read));
        result.numericIsFloat.push_back(isFloat?1:0);
    }
    if(result.maps)return RigExecLowerSceneAnimatedMaps(scene,source,*node,&result,output,error);
    if(result.ml)return RigExecLowerSceneMlMover(scene,source,*node,&result,output,error);
    TfToken solverToken;
    if(!Raw(source,property("rigExec:solver"),TfToken("pose"),&solverToken,error))return false;
    if(solverToken!="pose" && solverToken!="additive" && solverToken!="interpolative")
        return Fail(error,"unknown pose interpolator solver "+solverToken.GetString()+": "+path.GetString());
    const bool rigLogic=solverToken!="pose";
    const auto drivers=source.Targets(property("rigExec:driver"));
    if(!rigLogic && drivers.size()!=1 && (result.driverAttributes.empty() || !drivers.empty()))
        return Fail(error,"pose interpolator requires exactly one frame driver");
    if(rigLogic && drivers.empty()==result.driverAttributes.empty())
        return Fail(error,"pose interpolator requires frame drivers or numeric drivers, not both: "+path.GetString());
    if(!ResolveDrivers(scene,source,drivers,"pose interpolator",&result,error))return false;
    if(!RigExecBindSceneTypedRead(scene,property("inputs:enabled"),SdfValueTypeNames->Bool,&result.enabled,error))return false;
    RigExecRbfSolverDesc desc;TfToken token;
    if(!Raw(source,property("rigExec:allowNegativeWeights"),true,&result.record.allowNegativeWeights,error) ||
       !Raw(source,property("rigExec:kernel"),TfToken("gaussian"),&token,error))return false;
    desc.kernel=token=="linear"?RigExecRbfKernel::Linear:token=="exponential"?RigExecRbfKernel::Exponential:
        token=="cubic"?RigExecRbfKernel::Cubic:token=="quintic"?RigExecRbfKernel::Quintic:RigExecRbfKernel::Gaussian;
    RigExecRbfMethodDesc method;
    method.solverType=solverToken=="interpolative"?RigExecRbfSolverType::Interpolative:RigExecRbfSolverType::Additive;
    method.weightFunction=desc.kernel==RigExecRbfKernel::Linear?RigExecRbfWeightFunction::Linear:
        desc.kernel==RigExecRbfKernel::Exponential?RigExecRbfWeightFunction::Exponential:
        desc.kernel==RigExecRbfKernel::Cubic?RigExecRbfWeightFunction::Cubic:
        desc.kernel==RigExecRbfKernel::Quintic?RigExecRbfWeightFunction::Quintic:RigExecRbfWeightFunction::Gaussian;
    if(rigLogic) {
        TfToken distance,normalizeMethod;float radius=0,threshold=0;
        if(!Raw(source,property("rigExec:distanceMethod"),TfToken("quaternion"),&distance,error) ||
           !Raw(source,property("rigExec:normalizeMethod"),TfToken("onlyAboveOne"),&normalizeMethod,error) ||
           !Raw(source,property("rigExec:automaticRadius"),false,&method.automaticRadius,error) ||
           !Raw(source,property("rigExec:radius"),0.0f,&radius,error) ||
           !Raw(source,property("rigExec:weightThreshold"),0.0f,&threshold,error))return false;
        if(distance!="euclidean" && distance!="quaternion" && distance!="swingAngle" && distance!="twistAngle")
            return Fail(error,"unknown pose interpolator distance method "+distance.GetString()+": "+path.GetString());
        if(normalizeMethod!="onlyAboveOne" && normalizeMethod!="always")
            return Fail(error,"unknown pose interpolator normalize method "+normalizeMethod.GetString()+": "+path.GetString());
        if(!std::isfinite(radius) || radius<0 || !std::isfinite(threshold))
            return Fail(error,"pose interpolator radius is negative or not finite: "+path.GetString());
        method.distanceMethod=distance=="euclidean"?RigExecRbfDistanceMethod::Euclidean:
            distance=="swingAngle"?RigExecRbfDistanceMethod::SwingAngle:
            distance=="twistAngle"?RigExecRbfDistanceMethod::TwistAngle:RigExecRbfDistanceMethod::Quaternion;
        method.normalizeMethod=normalizeMethod=="always"?RigExecRbfNormalizeMethod::Always:
            RigExecRbfNormalizeMethod::OnlyAboveOne;
        method.radius=radius;method.weightThreshold=threshold;
        if(!result.driverAttributes.empty() && method.distanceMethod!=RigExecRbfDistanceMethod::Euclidean)
            return Fail(error,"numeric pose drivers are measured by the euclidean distance method only: "+path.GetString());
        method.rawCount=result.driverAttributes.empty()?4*drivers.size():result.driverAttributes.size();
    }
    float regularization=0;
    if(!Raw(source,property("rigExec:regularization"),0.0f,&regularization,error) ||
       !Raw(source,property("rigExec:normalize"),true,&desc.normalize,error) ||
       !Raw(source,property("rigExec:enableRotation"),true,&desc.enableRotation,error) ||
       !Raw(source,property("rigExec:enableTranslation"),false,&desc.enableTranslation,error) ||
       !Raw(source,property("rigExec:twistAxis"),TfToken("X"),&token,error))return false;
    desc.regularization=regularization;
    desc.twistAxis=token=="Y"?GfVec3d(0,1,0):token=="Z"?GfVec3d(0,0,1):GfVec3d(1,0,0);
    method.twistAxis=token=="Y"?1:token=="Z"?2:0;
    if(!result.driverAttributes.empty()) {desc.enableRotation=false;desc.enableTranslation=true;}
    result.record.enableTranslation=desc.enableTranslation;
    std::vector<double> radii,translationRadii;size_t children=0;
    for(const auto &childPath:node->fact.children) {
        const auto *child=source.Node(childPath);if(!child || !child->fact.active)continue;
        if(child->fact.type!="RigExecPose")return Fail(error,"pose interpolator child is not a RigExecPose: "+childPath.GetString());
        ++children;const auto weight=childPath.AppendProperty(TfToken("outputs:weight"));
        if(const auto *attribute=source.Attribute(weight))if(attribute->fact.hasAuthoredConnections)
            return Fail(error,"pose weight output carries authored connections: "+weight.GetString());
        bool enabled=true;if(!Raw(source,childPath.AppendProperty(TfToken("inputs:enabled")),true,&enabled,error))return false;
        if(!enabled) {result.disabledPoseWeights.push_back(weight);continue;}
        GfQuatf rotation(1.0f);GfVec3f translation(0);TfToken kind;
        float radius=0,translationRadius=0;
        if(!Raw(source,childPath.AppendProperty(TfToken("rigExec:rotation")),GfQuatf(1.0f),&rotation,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:translation")),GfVec3f(0),&translation,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:poseType")),TfToken("swing"),&kind,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:rotationRadius")),0.0f,&radius,error) ||
           !Raw(source,childPath.AppendProperty(TfToken("rigExec:translationRadius")),0.0f,&translationRadius,error))return false;
        if(!std::isfinite(radius) || radius<0 || !std::isfinite(translationRadius) || translationRadius<0)
            return Fail(error,"pose radius is negative or not finite: "+childPath.GetString());
        if(rigLogic) {
            // The target row in the input's own layout: each driver's rotation
            // as (x, y, z, w) on the w >= 0 hemisphere, or the numeric values.
            VtQuatfArray rotations;float scale=1;
            if(!Raw(source,childPath.AppendProperty(TfToken("rigExec:rotations")),VtQuatfArray{},&rotations,error) ||
               !Raw(source,childPath.AppendProperty(TfToken("rigExec:scale")),1.0f,&scale,error))return false;
            if(!std::isfinite(scale))return Fail(error,"pose scale is not finite: "+childPath.GetString());
            if(!result.driverAttributes.empty()) {
                for(size_t k=0;k<result.driverAttributes.size();++k)method.targets.push_back(translation[int(k)]);
            } else {
                if(rotations.empty() && drivers.size()==1)rotations.push_back(rotation);
                if(rotations.size()!=drivers.size())
                    return Fail(error,"pose rigExec:rotations must hold one rotation per driver: "+childPath.GetString());
                for(const GfQuatf &q:rotations) {
                    const double s=q.GetReal()<0?-1.0:1.0;const GfVec3f &v=q.GetImaginary();
                    method.targets.insert(method.targets.end(),{s*v[0],s*v[1],s*v[2],s*q.GetReal()});
                }
            }
            method.targetScales.push_back(scale);
        }
        desc.poses.push_back(RigExecRbfEulerFromQuaternion(GfQuatd(rotation.GetReal(),GfVec3d(rotation.GetImaginary()))));
        desc.translations.push_back(GfVec3d(translation[0]/100.0,translation[1]/100.0,translation[2]/100.0));
        desc.poseTypes.push_back(kind=="twist"?RigExecRbfPoseType::Twist:kind=="whole"?RigExecRbfPoseType::Whole:RigExecRbfPoseType::Swing);
        radii.push_back(double(radius));translationRadii.push_back(double(translationRadius)/100.0);
        result.poseWeights.push_back(weight);
    }
    if(!children)return Fail(error,"pose interpolator has no pose children: "+path.GetString());
    result.record.poseCount=result.poseWeights.size();
    if(rigLogic) {
        result.record.rigLogicSolve=true;
        if(!result.poseWeights.empty() && !RigExecRbfMethodBuild(method,&result.record.method))
            return Fail(error,"pose interpolator could not be solved: "+path.GetString());
        const auto &table=result.record.method;bool coincident=false;
        for(size_t i=0;i<table.targetCount && !coincident;++i)for(size_t j=i+1;j<table.targetCount;++j)
            if(RigExecRbfMethodCoincident(table,i,j)) {coincident=true;break;}
        if(coincident)result.notes.push_back("warning: pose interpolator "+path.GetString()+
            " has poses that are coincident under its distance method; an interpolative solve is singular there");
    } else if(!result.poseWeights.empty()) {
        if(!desc.enableTranslation) {desc.translations.clear();translationRadii.clear();}
        RigExecRbfSolver solver(desc);solver.SetSolvedTable(radii,translationRadii,{});
        if(!solver.Solve())return Fail(error,"pose interpolator could not be solved: "+path.GetString());
        if(solver.Degenerate())result.notes.push_back("warning: pose interpolator "+path.GetString()+
            " has poses that are coincident under the channels it has enabled; its weights will sit at 1/n");
        result.record.solver=std::move(solver);
    }
    *output=std::move(result);return true;
}
bool RigExecResolveScenePoseInterpolator(const RigExecSceneDescriptors &scene,
    const RigExecScenePoseInterpolatorDescriptor &record,UsdTimeCode time,
    const std::map<SdfPath,VtValue> &delivered,RigExecPoseInterpolatorInputs *input,
    std::string *error) {
    if(!input)return Fail(error,"null pose interpolator inputs");
    input->enabled=true;VtValue value;
    if(RigExecResolveSceneTypedRead(scene,record.enabled,time,delivered,&value,error) && value.IsHolding<bool>())
        input->enabled=value.UncheckedGet<bool>();
    input->mask=1.0;
    if(record.ml && RigExecResolveSceneTypedRead(scene,record.mask,time,delivered,&value,error) &&
       value.IsHolding<float>())input->mask=double(value.UncheckedGet<float>());
    input->numeric.assign(record.numericReads.size(),0.0);
    for(size_t k=0;k<record.numericReads.size();++k)
        if(RigExecResolveSceneTypedRead(scene,record.numericReads[k],time,delivered,&value,error)) {
            if(value.IsHolding<double>())input->numeric[k]=value.UncheckedGet<double>();
            else if(value.IsHolding<float>())input->numeric[k]=double(value.UncheckedGet<float>());
        }
    return true;
}
}
