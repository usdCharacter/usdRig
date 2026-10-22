#include "propertySceneTranslation.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/tf/staticTokens.h"
#include <type_traits>
namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    ((rigExecOperation,"rigExec:operation")) ((rigExecWeightObject,"rigExec:weightObject"))
    ((inputsEnabled,"inputs:enabled")) ((inputsDefaultWeight,"inputs:defaultWeight"))
    ((inputsValue,"inputs:value")) ((inputsMin,"inputs:min")) ((inputsMax,"inputs:max"))
    ((inputsKeys,"inputs:keys")) ((inputsTangents,"inputs:tangents"))
);
namespace {
bool Fail(std::string *error,const std::string &message) { if(error)*error=message;return false; }
}
bool RigExecTranslateSceneProperty(const RigExecSceneDescriptors &scene,const SdfPath &mover,
    const SdfPath &target,RigExecScenePropertyDescriptor *output,std::string *error) {
    if(!output)return Fail(error,"null property descriptor destination");
    const RigExecSceneCompileInputs source(scene);
    const auto *node=source.Node(mover);const auto *attribute=source.Attribute(target);
    if(!node || !node->fact.active || node->domain!=RigExecSceneDomain::PropertyMover || !attribute)
        return Fail(error,"missing property mover/target: "+mover.GetString()+" -> "+target.GetString());
    RigExecScenePropertyDescriptor result;
    result.mover=mover;result.target=target;result.targetType=attribute->fact.type;
    const auto type=node->fact.type;
    const bool scalar=result.targetType==SdfValueTypeNames->Float || result.targetType==SdfValueTypeNames->Double;
    if(type=="RigExecFloatMathMover" && scalar)result.record=RigExecFloatPropertyRecord{};
    else if(type=="RigExecVec3fMathMover" && result.targetType.GetType()==TfType::Find<GfVec3f>())
        result.record=RigExecVec3PropertyRecord{};
    else if(type=="RigExecMatrixMathMover" && result.targetType==SdfValueTypeNames->Matrix4d)
        result.record=RigExecMatrixPropertyRecord{};
    else return Fail(error,"property mover/target type mismatch: "+target.GetString());
    TfToken operation;
    RigExecSceneBoundInput operationRead;
    const auto operationPath=mover.AppendProperty(_tokens->rigExecOperation);
    if(source.Attribute(operationPath)) {
        VtValue raw;
        if(!source.Bind(operationPath,RigExecSceneReadRoute::Raw,&operationRead,error) ||
           !source.Read(operationRead,UsdTimeCode::Default(),&raw,nullptr,error))return false;
        if(raw.IsHolding<TfToken>())operation=raw.UncheckedGet<TfToken>();
    }
    RigExecPropertyOp op=RigExecPropertyOp::Add;
    const bool valid=RigExecParsePropertyOp(operation,&op);
    std::visit([&](auto &record) { record.op=op;record.opValid=valid; },result.record);
    // Existing property movers read the first direct authored weight target,
    // rather than forwarding a relationship-property target.
    if(const auto *weight=source.Relationship(mover.AppendProperty(_tokens->rigExecWeightObject)))
        if(!weight->fact.targets.empty())result.weightObject=weight->fact.targets.front();
    auto bind=[&](const TfToken &name) {
        const auto path=mover.AppendProperty(name);
        if(!source.Attribute(path))return true;
        RigExecSceneBoundInput input;
        if(!source.Bind(path,RigExecSceneReadRoute::ConnectionResolved,&input,error))return false;
        SdfValueTypeName requested;
        if(name==_tokens->inputsEnabled)requested=SdfValueTypeNames->Bool;
        else if(name==_tokens->inputsDefaultWeight)requested=SdfValueTypeNames->Float;
        else if(name==_tokens->inputsKeys || name==_tokens->inputsTangents)requested=SdfValueTypeNames->Float2Array;
        else requested=scalar?SdfValueTypeNames->Float:
            std::holds_alternative<RigExecMatrixPropertyRecord>(result.record)?SdfValueTypeNames->Matrix4d:SdfValueTypeNames->Float3;
        RigExecSceneTypedRead typed;
        if(!RigExecBindSceneTypedRead(scene,path,requested,&typed,error))return false;
        result.typedReads.emplace(name.GetString(),std::move(typed));
        result.inputs.emplace(name.GetString(),std::move(input));return true;
    };
    if(!bind(_tokens->inputsEnabled) || !bind(_tokens->inputsDefaultWeight) || !bind(_tokens->inputsValue))return false;
    if(!std::holds_alternative<RigExecMatrixPropertyRecord>(result.record))
        if(!bind(_tokens->inputsMin) || !bind(_tokens->inputsMax))return false;
    if(scalar && valid && op==RigExecPropertyOp::Curve) {
        if(!bind(_tokens->inputsKeys) || !bind(_tokens->inputsTangents))return false;
        std::get<RigExecFloatPropertyRecord>(result.record).hasTangents=
            source.Attribute(mover.AppendProperty(_tokens->inputsTangents))!=nullptr;
    }
    *output=std::move(result);return true;
}
bool RigExecResolveSceneProperty(const RigExecSceneDescriptors &scene,
    RigExecScenePropertyDescriptor *descriptor,UsdTimeCode identity,
    const std::map<SdfPath,VtValue> &delivered,bool *enabled,float *weight,std::string *error) {
    if(!descriptor || !enabled || !weight)return Fail(error,"null property input destination");
    const RigExecSceneCompileInputs source(scene);
    auto read=[&](const char *name,auto fallback,auto *value) {
        *value=fallback;const auto found=descriptor->inputs.find(name);
        if(found==descriptor->inputs.end())return true;
        const auto &binding=found->second;
        VtValue boxed;
        const auto typed=descriptor->typedReads.find(name);
        if(typed!=descriptor->typedReads.end()) {
            if(binding.readPhase!="base" && binding.source!=binding.consumer && !delivered.count(binding.consumer))
                return Fail(error,"phased property input producer unavailable: "+binding.consumer.GetString());
            if(!RigExecResolveSceneTypedRead(scene,typed->second,identity,delivered,&boxed,nullptr))return true;
        } else if(!source.Read(binding,identity,&boxed,nullptr,error))return false;
        if(boxed.IsEmpty())return true;
        using T=std::decay_t<decltype(*value)>;
        if(!boxed.IsHolding<T>())return Fail(error,"property input type mismatch: "+binding.consumer.GetString());
        *value=boxed.UncheckedGet<T>();return true;
    };
    if(!read("inputs:enabled",true,enabled) || !read("inputs:defaultWeight",1.0f,weight))return false;
    if(auto *record=std::get_if<RigExecFloatPropertyRecord>(&descriptor->record)) {
        if(!read("inputs:value",0.0f,&record->value) || !read("inputs:min",0.0f,&record->minimum) ||
           !read("inputs:max",0.0f,&record->maximum))return false;
        if(record->opValid && record->op==RigExecPropertyOp::Curve) {
            VtVec2fArray keys,tangents;
            if(!read("inputs:keys",VtVec2fArray{},&keys) || !read("inputs:tangents",VtVec2fArray{},&tangents))return false;
            record->keys.assign(keys.begin(),keys.end());record->tangents.assign(tangents.begin(),tangents.end());
        }
        return true;
    }
    if(auto *record=std::get_if<RigExecVec3PropertyRecord>(&descriptor->record))
        return read("inputs:value",GfVec3f(0),&record->value) && read("inputs:min",GfVec3f(0),&record->minimum) &&
            read("inputs:max",GfVec3f(0),&record->maximum);
    auto &record=std::get<RigExecMatrixPropertyRecord>(descriptor->record);
    return read("inputs:value",GfMatrix4d(1.0),&record.value);
}
}
