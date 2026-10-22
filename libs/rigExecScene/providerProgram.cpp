#include "providerProgram.h"
#include "rigExecGraph/providerArithmetic.h"
#include "rigExecMath/affineFrameKernels.h"
#include "rigExecMath/avarScale.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usdGeom/xformOp.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <variant>

namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (xformOpOrder) ((restSpace,"rest:space")) ((posedSpace,"posed:space"))
    ((defaultSpace,"default:space")) ((avarsDefaultSpace,"avars:defaultSpace"))
    ((posedDefaultSpace,"posed:defaultSpace")) ((parentSpace,"parent:space"))
    ((parentDefaultSpace,"parent:defaultSpace")) ((restTx,"rest:tx")) ((restTy,"rest:ty"))
    ((restTz,"rest:tz")) ((restRx,"rest:rx")) ((restRy,"rest:ry")) ((restRz,"rest:rz"))
    ((defaultTx,"default:tx")) ((defaultTy,"default:ty")) ((defaultTz,"default:tz"))
    ((defaultRx,"default:rx")) ((defaultRy,"default:ry")) ((defaultRz,"default:rz"))
    ((avarsUnitScaleFactor,"avars:unitScaleFactor")) ((avarsTx,"avars:tx")) ((avarsTy,"avars:ty"))
    ((avarsTz,"avars:tz")) ((avarsSx,"avars:sx")) ((avarsSy,"avars:sy")) ((avarsSz,"avars:sz"))
    ((avarsRx,"avars:rx")) ((avarsRy,"avars:ry")) ((avarsRz,"avars:rz"))
    ((avarsRspin,"avars:rspin")) ((avarsRotationOrder,"avars:rotationOrder"))
    ((avarsRotationSign,"avars:rotationSign"))
);
namespace {
bool Fail(std::string *error,const std::string &message) { if(error)*error=message; return false; }
bool Provider(const TfToken &type) {
    return type=="RigExecControl" || type=="RigExecJoint" || type=="RigExecSphereWeight" ||
        type=="RigExecPlaneWeight" || type=="RigExecCurveWeight";
}
const char *const computations[]={"computeRestFrame","computedDefaultSpace","computeDefaultFrame",
    "computedParentDefaultSpace","computedParentSpace","computePointFrame","computeMatrix","interveningSpace","computeAvarMatrix"};
const char *const expressions[]={"default:space","avars:defaultSpace","posed:defaultSpace","parent:space","parent:defaultSpace"};
bool Expression(const std::string &name) {
    for(const auto *expression:expressions)if(name==expression)return true;
    return false;
}
SdfPath ParentProvider(const RigExecSceneDescriptors &scene,const SdfPath &path) {
    for(auto parent=path.GetParentPath();!parent.IsEmpty();parent=parent.GetParentPath()) {
        const auto found=scene.nodes.find(parent);
        if(found!=scene.nodes.end() && found->second.fact.active && Provider(found->second.fact.type))return parent;
    }
    return {};
}
// Affine frame expressions: each publishes its outputs:matrix from its own
// input attributes and the point frames of the providers its relationships
// name, as an ordinary op whose inputs are those values. The layout
// (RigExecAffineFrameTypes) states every input in op order; the kernels are
// affineFrameKernel.h's, which the runtime runs too.
using AffineInputs=RigExecAffineFrameInputs;
using AffineKernel=RigExecGfAffineFrameKernel;
constexpr uint32_t affineTypeCount=RigExecAffineFrameTypeCount;
// The affine type of \p type, or affineTypeCount.
uint32_t AffineTypeOf(const TfToken &type) {
    for(uint32_t k=0;k<affineTypeCount;++k)if(type==RigExecAffineFrameTypes[k].type)return k;
    return affineTypeCount;
}
bool AffineOutput(const RigExecSceneDescriptors &scene,const SdfPath &attribute,uint32_t *kind) {
    if(attribute.GetName()!="outputs:matrix")return false;
    const auto node=scene.nodes.find(attribute.GetPrimPath());
    if(node==scene.nodes.end() || !node->second.fact.active)return false;
    *kind=AffineTypeOf(node->second.fact.type);
    return *kind<affineTypeCount;
}
// RigExecAffineFrameTypes' USD-free names as tokens, built once when the
// library loads, so translation never builds one from text.
struct AffineTokens {
    std::vector<TfToken> attributes,relations;
    TfToken targets,targetObjects;
};
const std::vector<AffineTokens> _affineTokens=[] {
    std::vector<AffineTokens> table;
    for(const auto &type:RigExecAffineFrameTypes) {
        AffineTokens tokens;
        for(uint32_t k=0;k<type.attributeCount;++k)tokens.attributes.emplace_back(type.attributes[k].name);
        for(uint32_t k=0;k<type.relationCount;++k)tokens.relations.emplace_back(type.relations[k].name);
        if(type.targets)tokens.targets=TfToken(type.targets);
        if(type.targetObjects)tokens.targetObjects=TfToken(type.targetObjects);
        table.push_back(std::move(tokens));
    }
    return table;
}();
// The xform operation type a name's type part spells, from the type tokens.
UsdGeomXformOp::Type XformOpType(const std::string &name,size_t begin,size_t count) {
    for(int t=UsdGeomXformOp::TypeInvalid+1;t<=UsdGeomXformOp::TypeTransform;++t)
        if(!name.compare(begin,count,UsdGeomXformOp::GetOpTypeToken(UsdGeomXformOp::Type(t)).GetString()))return UsdGeomXformOp::Type(t);
    return UsdGeomXformOp::TypeInvalid;
}
SdfPathVector AffineTargets(const RigExecSceneDescriptors &scene,const SdfPath &prim,const TfToken &name) {
    if(name.IsEmpty())return {};
    const auto found=scene.relationships.find(prim.AppendProperty(name));
    return found==scene.relationships.end()?SdfPathVector():found->second.fact.targets;
}
// A provider's point frame from its four landmarks, as a matrix: the axes
// relative to the origin as rows, then the origin. No frame is identity.
GfMatrix4d AffineFrameMatrix(const RigExecPointFrame *frame) {
    if(!frame)return GfMatrix4d(1);
    return AffineKernel::FrameMatrix(frame->Origin(),frame->X(),frame->Y(),frame->Z());
}
// One input's value into its member: the store's typed value where it holds
// one, the boxed source value otherwise; an unavailable input keeps the
// member's default, as an absent exec input does.
void AffineRead(const RigExecTypedValueStore &store,RigExecValueId id,RigExecAffineField field,AffineInputs *in) {
    const auto boxed=[&](auto *out) {
        using T=std::decay_t<decltype(*out)>;
        const auto *value=store.Read<VtValue>(id);
        if(value && value->IsHolding<T>())*out=value->UncheckedGet<T>();
    };
    RigExecVisitAffineField(*in,field,[&](auto &out) {
        using T=std::decay_t<decltype(out)>;
        if constexpr(std::is_same_v<T,GfMatrix4d> || std::is_same_v<T,GfVec3d>) {
            if(const auto *value=store.Read<T>(id))out=*value;
        } else if constexpr(std::is_same_v<T,double>) {
            if(const auto *value=store.Read<double>(id))out=*value;
            else if(const auto *narrow=store.Read<float>(id))out=*narrow;
        } else if constexpr(std::is_same_v<T,std::string>) {
            // The token's own text: no registry lookup.
            if(const auto *value=store.Read<TfToken>(id))out=value->GetText();
        } else if constexpr(std::is_same_v<T,std::vector<int>>) {
            VtIntArray array;boxed(&array);if(!array.empty())out.assign(array.begin(),array.end());
        } else if constexpr(std::is_same_v<T,std::vector<GfMatrix4d>>) {
            VtMatrix4dArray array;boxed(&array);if(!array.empty())out.assign(array.begin(),array.end());
        } else if constexpr(std::is_same_v<T,std::vector<double>>) {
            VtDoubleArray array;boxed(&array);if(!array.empty())out.assign(array.begin(),array.end());
        } else {
            boxed(&out);
        }
    });
}
struct GfProviderMath {
    using Matrix=GfMatrix4d;using Vector=GfVec3d;using Rotation=GfRotation;using Frame=RigExecPointFrame;
    static bool PointsToMatrix(const std::array<Vector,4> &rest,const std::array<Vector,4> &pose,Matrix *out) {
        return RigExecPointsToMatrix(rest,pose,out);
    }
};
struct GfProviderStore {
    RigExecTypedValueStore &store;
    const GfMatrix4d *ReadMatrix(uint64_t id) const {return store.Read<GfMatrix4d>(id);}
    const RigExecPointFrame *ReadFrame(uint64_t id) const {return store.Read<RigExecPointFrame>(id);}
    const double *ReadScalar(uint64_t id) const {return store.Read<double>(id);}
    const GfVec3d *ReadVector(uint64_t id) const {return store.Read<GfVec3d>(id);}
    std::string_view ReadToken(uint64_t id) const {
        // A field read: GetString() on an empty token reads a function-local static.
        const auto *token=store.Read<TfToken>(id);return token?std::string_view(token->GetText(),token->size()):std::string_view();
    }
    bool ResetXformStack(uint64_t id) const {
        if(id>=store.values.size() || !store.values[size_t(id)].initialized ||
           store.values[size_t(id)].blocked)return false;
        const auto &value=store.values[size_t(id)].raw;
        if(!value.IsHolding<VtTokenArray>())return false;
        for(const auto &token:value.UncheckedGet<VtTokenArray>())if(token=="!resetXformStack!")return true;return false;
    }
    bool Present(uint64_t id) const {return id<store.values.size() && store.values[size_t(id)].initialized;}
    bool Authoritative(uint64_t id) const {return id<store.values.size() && store.values[size_t(id)].authoritative;}
    void Copy(uint64_t to,uint64_t from,bool authoritative){store.Copy(to,from,authoritative);}
    void Invalidate(uint64_t to,uint64_t from) {
        const bool blocked=from<store.values.size() && store.values[size_t(from)].blocked;
        store.values.at(size_t(to)).raw=VtValue();
        store.Publish(to,std::monostate(),blocked,false,0,"provider input unavailable");
    }

    void PublishMatrix(uint64_t id,const GfMatrix4d &value,bool authoritative){store.Publish(id,value,false,authoritative);}
    void PublishFrame(uint64_t id,const RigExecPointFrame &value){store.Publish(id,value);}
};
}
std::string RigExecProviderValueKey(const SdfPath &owner,const std::string &computation) {
    return "computation:"+owner.GetString()+":"+computation;
}
std::string RigExecProviderRawKey(const SdfPath &attribute) { return "raw:"+attribute.GetString(); }
std::string RigExecProviderAttributeKey(const SdfPath &attribute) { return "value:"+attribute.GetString(); }
std::string RigExecProviderRoutedKey(const SdfPath &consumer,const TfToken &phase) {
    return "connection:"+consumer.GetString()+":"+phase.GetString();
}
RigExecValueId RigExecProviderProgram::FindValue(const std::string &key) const {
    const auto found=valueIds.find(key);return found==valueIds.end()?RigExecNoProviderValue:found->second;
}
bool RigExecBuildProviderProgram(const RigExecSceneDescriptors &scene,bool composeFrames,
    RigExecProviderProgram *output,std::string *error,const std::set<SdfPath> *selectedProviders) {
    if(!output)return Fail(error,"provider compiler needs an output");
    RigExecProviderProgram program;
    std::set<SdfPath> providerPaths,usedAttributes,xformPaths,nativeFrames;
    std::function<void(const SdfPath &)> includeAttribute,includeProvider;
    std::function<void(const SdfPath &,uint32_t)> includeAffine;
    includeAttribute=[&](const SdfPath &path) {
        const auto attribute=scene.attributes.find(path);
        const auto owner=scene.nodes.find(path.GetPrimPath());
        if(attribute==scene.attributes.end() || owner==scene.nodes.end() || !owner->second.fact.active ||
            !usedAttributes.insert(path).second)return;
        if(Provider(owner->second.fact.type) && Expression(path.GetName()))includeProvider(path.GetPrimPath());
        uint32_t affine=0;
        if(AffineOutput(scene,path,&affine))includeAffine(path.GetPrimPath(),affine);
        for(const auto &connection:attribute->second.fact.connections)includeAttribute(connection);
    };
    // An affine frame expression reads its input attributes and the point
    // frames of the providers its relationships name.
    includeAffine=[&](const SdfPath &prim,uint32_t kind) {
        const AffineTokens &tokens=_affineTokens[kind];
        for(const auto &name:tokens.attributes)includeAttribute(prim.AppendProperty(name));
        const auto frames=[&](const TfToken &name) {
            if(name.IsEmpty())return;
            for(const auto &target:AffineTargets(scene,prim,name)) {
                const auto node=scene.nodes.find(target);
                if(target.IsPrimPath() && node!=scene.nodes.end() && node->second.fact.active &&
                   Provider(node->second.fact.type))includeProvider(target);
            }
        };
        for(const auto &name:tokens.relations)frames(name);
        frames(tokens.targets);frames(tokens.targetObjects);
    };
    includeProvider=[&](const SdfPath &path) {
        if(path.IsEmpty() || !providerPaths.insert(path).second)return;
        includeProvider(ParentProvider(scene,path));
        const TfToken *const inputs[]={&_tokens->restSpace,&_tokens->posedSpace,&_tokens->defaultSpace,&_tokens->avarsDefaultSpace,&_tokens->posedDefaultSpace,
            &_tokens->parentSpace,&_tokens->parentDefaultSpace,&_tokens->restTx,&_tokens->restTy,&_tokens->restTz,
            &_tokens->restRx,&_tokens->restRy,&_tokens->restRz,&_tokens->defaultTx,&_tokens->defaultTy,&_tokens->defaultTz,
            &_tokens->defaultRx,&_tokens->defaultRy,&_tokens->defaultRz,&_tokens->avarsUnitScaleFactor,&_tokens->avarsTx,&_tokens->avarsTy,
            &_tokens->avarsTz,&_tokens->avarsSx,&_tokens->avarsSy,&_tokens->avarsSz,&_tokens->avarsRx,&_tokens->avarsRy,
            &_tokens->avarsRz,&_tokens->avarsRspin,&_tokens->avarsRotationOrder,&_tokens->avarsRotationSign};
        for(const auto *input:inputs)includeAttribute(path.AppendProperty(*input));
    };
    for(const auto &[path,node]:scene.nodes)
        if(node.fact.active && Provider(node.fact.type) && (!selectedProviders || selectedProviders->count(path)))includeProvider(path);
    if(composeFrames)for(const auto &path:providerPaths) {
        const auto parent=ParentProvider(scene,path);
        const auto anchor=parent.IsEmpty()?scene.rigRoot.GetParentPath():parent;
        for(auto ancestor=path.GetParentPath();!ancestor.IsEmpty() && ancestor!=anchor;ancestor=ancestor.GetParentPath()) {
            const auto node=scene.nodes.find(ancestor);
            if(node==scene.nodes.end() || !node->second.fact.active || !node->second.transformProvider || Provider(node->second.fact.type))continue;
            xformPaths.insert(ancestor);
            for(const auto &attribute:node->second.attributes)
                if(attribute.GetName()=="xformOpOrder" || attribute.GetName().rfind("xformOp:",0)==0)includeAttribute(attribute);
        }
    }
    if(composeFrames)for(const auto &[path,node]:scene.nodes)if(node.fact.active && node.transformProvider && !Provider(node.fact.type)) {
        bool referenced=false;for(const auto &[unused,relation]:scene.relationships)for(const auto &target:relation.forwardedTargets)referenced|=target.GetPrimPath()==path;
        if(!referenced)continue;nativeFrames.insert(path);
        for(auto ancestor=path;!ancestor.IsEmpty() && ancestor!=scene.rigRoot.GetParentPath();ancestor=ancestor.GetParentPath()) {
            const auto captured=scene.nodes.find(ancestor);if(captured==scene.nodes.end() || !captured->second.fact.active || !captured->second.transformProvider)continue;
            xformPaths.insert(ancestor);for(const auto &attribute:captured->second.attributes)if(attribute.GetName()=="xformOpOrder" || attribute.GetName().rfind("xformOp:",0)==0)includeAttribute(attribute);
        }
    }
    std::set<std::string> keys;
    std::map<SdfPath,RigExecValueId> routed;
    for(const auto &[path,attr]:scene.attributes) {
        if(!usedAttributes.count(path))continue;
        const auto node=scene.nodes.find(path.GetPrimPath());
        if(node==scene.nodes.end() || !node->second.fact.active)continue;
        keys.insert(RigExecProviderRawKey(path));keys.insert(RigExecProviderAttributeKey(path));
        if(attr.readPhase!="base" && attr.fact.connections.size()==1) {
            const auto input=scene.attributes.find(attr.fact.connections.front());
            const auto inputNode=scene.nodes.find(attr.fact.connections.front().GetPrimPath());
            if(input!=scene.attributes.end() && inputNode!=scene.nodes.end() && inputNode->second.fact.active &&
                input->second.fact.type.GetType()==attr.fact.type.GetType())keys.insert(RigExecProviderRoutedKey(path,attr.readPhase));
        }
    }
    for(const auto &[path,node]:scene.nodes)if(node.fact.active) {
        if(providerPaths.count(path) || nativeFrames.count(path))for(const auto *computation:computations)
            keys.insert(RigExecProviderValueKey(path,computation));
        if(xformPaths.count(path))
            keys.insert(RigExecProviderValueKey(path,"localXform"));
    }
    for(const auto &key:keys) {
        const auto id=RigExecValueId(program.valueKeys.size());
        program.valueIds.emplace(key,id);program.valueKeys.push_back(key);
    }
    for(const auto &path:usedAttributes) {
        const auto &attr=scene.attributes.at(path);
        const auto value=program.FindValue(RigExecProviderRoutedKey(path,attr.readPhase));
        if(value!=RigExecNoProviderValue) {
            routed.emplace(path,value);program.leaves.push_back(value);
            program.routedInputs.push_back({value,path,attr.fact.connections.front(),attr.readPhase});
        }
    }
    const auto attrId=[&](const SdfPath &path,const TfToken &name) {
        return program.FindValue(RigExecProviderAttributeKey(path.AppendProperty(name)));
    };
    const auto compId=[&](const SdfPath &path,const char *name) {
        return path.IsEmpty()?RigExecNoProviderValue:program.FindValue(RigExecProviderValueKey(path,name));
    };
    const auto connect=[&](const RigExecSceneAttribute &fact) {
        const auto route=routed.find(fact.path);
        if(route!=routed.end())return route->second;
        if(fact.connections.size()!=1)return RigExecNoProviderValue;
        const auto found=scene.attributes.find(fact.connections.front());
        const auto node=scene.nodes.find(fact.connections.front().GetPrimPath());
        if(found==scene.attributes.end() || node==scene.nodes.end() || !node->second.fact.active ||
            found->second.fact.type.GetType()!=fact.type.GetType())return RigExecNoProviderValue;
        return program.FindValue(RigExecProviderAttributeKey(fact.connections.front()));
    };
    const auto add=[&](RigExecProviderOp op,const std::string &key) {
        RigExecOpDescriptor descriptor;
        descriptor.key=key;descriptor.kind=uint32_t(op.kind);descriptor.writes={op.output};
        for(const auto id:op.inputs)if(id!=RigExecNoProviderValue)descriptor.reads.push_back(id);
        for(const auto &xform:op.xforms)if(xform.raw!=RigExecNoProviderValue)descriptor.reads.push_back(xform.raw);
        std::sort(descriptor.reads.begin(),descriptor.reads.end());
        descriptor.reads.erase(std::unique(descriptor.reads.begin(),descriptor.reads.end()),descriptor.reads.end());
        program.ops.push_back(std::move(op));program.descriptors.push_back(std::move(descriptor));
    };
    for(const auto &[path,attr]:scene.attributes) {
        if(!usedAttributes.count(path))continue;
        const auto node=scene.nodes.find(path.GetPrimPath());
        if(node==scene.nodes.end() || !node->second.fact.active)continue;
        const auto raw=program.FindValue(RigExecProviderRawKey(path));
        const auto value=program.FindValue(RigExecProviderAttributeKey(path));
        program.rawInputs.emplace(path,raw);program.attributeValues.emplace(path,value);
        program.sampled.push_back({raw,path});program.leaves.push_back(raw);
        if(Provider(node->second.fact.type) && Expression(path.GetName()))continue;
        uint32_t affine=0;
        if(AffineOutput(scene,path,&affine)) {
            // Inputs in the table's order: the attributes, one frame per
            // single relation, then a constraint frame's target frames and
            // target-object frames.
            const RigExecAffineFrameType &type=RigExecAffineFrameTypes[affine];
            const SdfPath prim=path.GetPrimPath();
            RigExecProviderOp op{RigExecProviderOpKind::AffineFrame,prim,value,{}};
            op.affineKind=affine;
            const auto frame=[&](const SdfPath &target) {
                return target.IsPrimPath()?compId(target,"computePointFrame"):RigExecNoProviderValue;
            };
            const AffineTokens &tokens=_affineTokens[affine];
            for(uint32_t k=0;k<type.attributeCount;++k)op.inputs.push_back(attrId(prim,tokens.attributes[k]));
            for(uint32_t k=0;k<type.relationCount;++k) {
                const auto targets=AffineTargets(scene,prim,tokens.relations[k]);
                op.inputs.push_back(targets.empty()?RigExecNoProviderValue:frame(targets.front()));
            }
            if(type.targets) {
                const auto targets=AffineTargets(scene,prim,tokens.targets);
                for(const auto &target:targets)op.inputs.push_back(frame(target));
                op.affineTargets=uint32_t(targets.size());
                for(const auto &target:AffineTargets(scene,prim,tokens.targetObjects))op.inputs.push_back(frame(target));
            }
            add(std::move(op),"affineFrame:"+path.GetString());
            continue;
        }
        if(attr.fact.connections.size()>1 && attr.fact.type.IsArray() &&
            node->second.fact.type.GetString().rfind("RigExec",0)==0)
            return Fail(error,"provider translation needs typed array connection concatenation: "+path.GetString());
        add({RigExecProviderOpKind::Attribute,path,value,{raw,connect(attr.fact)}},"attribute:"+path.GetString());
    }
    if(composeFrames)for(const auto &[path,node]:scene.nodes) {
        if(!xformPaths.count(path))continue;
        const auto order=program.FindValue(RigExecProviderRawKey(path.AppendProperty(_tokens->xformOpOrder)));
        RigExecProviderOp op{RigExecProviderOpKind::LocalXform,path,compId(path,"localXform"),{order}};
        for(const auto &attribute:node.attributes) {
            const std::string name=attribute.GetName();
            if(name.rfind("xformOp:",0)!=0)continue;
            const auto end=name.find(':',8);
            const auto type=XformOpType(name,8,end==std::string::npos?end:end-8);
            if(type==UsdGeomXformOp::TypeInvalid)return Fail(error,"unknown pure xform operation: "+attribute.GetString());
            op.xforms.push_back({attribute.GetNameToken(),program.FindValue(RigExecProviderRawKey(attribute)),int(type)});
        }
        add(std::move(op),"localXform:"+path.GetString());
    }
    for(const auto &path:nativeFrames) {
        RigExecProviderOp relative{RigExecProviderOpKind::RelativeXform,path,compId(path,"computedDefaultSpace"),{}};
        for(auto ancestor=path;!ancestor.IsEmpty() && ancestor!=scene.rigRoot.GetParentPath();ancestor=ancestor.GetParentPath()) {
            const auto local=compId(ancestor,"localXform");if(local!=UINT64_MAX){relative.inputs.push_back(local);relative.inputs.push_back(program.FindValue(RigExecProviderRawKey(ancestor.AppendProperty(_tokens->xformOpOrder))));}
        }
        add(std::move(relative),"stageRelativeXform:"+path.GetString());
        add({RigExecProviderOpKind::MatrixToFrame,path,compId(path,"computePointFrame"),{compId(path,"computedDefaultSpace")}},"stageFrame:"+path.GetString());
        add({RigExecProviderOpKind::MatrixToFrame,path,compId(path,"computeRestFrame"),{compId(path,"computedDefaultSpace")}},"stageRest:"+path.GetString());
        add({RigExecProviderOpKind::JointMatrix,path,compId(path,"computeMatrix"),{compId(path,"computeRestFrame"),compId(path,"computePointFrame")}},"stageDelta:"+path.GetString());
    }
    for(const auto &[path,node]:scene.nodes) {
        if(!providerPaths.count(path))continue;
        const SdfPath parent=ParentProvider(scene,path);
        const auto selfRest=compId(path,"computeRestFrame"),selfPose=compId(path,"computePointFrame");
        const auto parentRest=compId(parent,"computeRestFrame");
        const auto intervening=compId(path,"interveningSpace");
        if(!composeFrames)for(const auto *computation:{"computeRestFrame","computePointFrame","interveningSpace"}) {
            const auto id=compId(path,computation);program.leaves.push_back(id);
            program.externalInputs.push_back({id,path,computation});
        }
        else {
            RigExecProviderOp ix{RigExecProviderOpKind::InterveningXform,path,intervening,{}};
            const SdfPath anchor=parent.IsEmpty()?scene.rigRoot.GetParentPath():parent;
            for(auto ancestor=path.GetParentPath();!ancestor.IsEmpty() && ancestor!=anchor;ancestor=ancestor.GetParentPath()) {
                const auto local=compId(ancestor,"localXform");
                if(local!=RigExecNoProviderValue) {
                    ix.inputs.push_back(local);
                    ix.inputs.push_back(program.FindValue(RigExecProviderRawKey(ancestor.AppendProperty(_tokens->xformOpOrder))));
                }
            }
            add(std::move(ix),"interveningXform:"+path.GetString());
            add({RigExecProviderOpKind::RestFrame,path,selfRest,{attrId(path,_tokens->restSpace),
                attrId(path,_tokens->restTx),attrId(path,_tokens->restTy),attrId(path,_tokens->restTz),
                attrId(path,_tokens->restRx),attrId(path,_tokens->restRy),attrId(path,_tokens->restRz),parentRest,intervening}},
                "restFrame:"+path.GetString());
        }
        add({RigExecProviderOpKind::FrameToSpace,path,compId(path,"computedParentDefaultSpace"),
            {compId(parent,"computeDefaultFrame")}},"parentDefault:"+path.GetString());
        add({RigExecProviderOpKind::FrameToSpace,path,compId(path,"computedParentSpace"),
            {compId(parent,"computePointFrame")}},"parentPosed:"+path.GetString());
        add({RigExecProviderOpKind::DefaultSpace,path,compId(path,"computedDefaultSpace"),
            {selfRest,parentRest,attrId(path,_tokens->parentDefaultSpace),
             attrId(path,_tokens->defaultTx),attrId(path,_tokens->defaultTy),attrId(path,_tokens->defaultTz),
             attrId(path,_tokens->defaultRx),attrId(path,_tokens->defaultRy),attrId(path,_tokens->defaultRz)}},"computedDefault:"+path.GetString());
        for(const auto *expression:{&_tokens->defaultSpace,&_tokens->avarsDefaultSpace,&_tokens->posedDefaultSpace,&_tokens->parentSpace,&_tokens->parentDefaultSpace}) {
            const SdfPath property=path.AppendProperty(*expression);
            const auto fact=scene.attributes.find(property);
            if(fact==scene.attributes.end())return Fail(error,"provider expression attribute missing: "+property.GetString());
            RigExecValueId fallback;
            if(*expression==_tokens->defaultSpace)fallback=compId(path,"computedDefaultSpace");
            else if(*expression==_tokens->avarsDefaultSpace)fallback=attrId(path,_tokens->defaultSpace);
            else if(*expression==_tokens->posedDefaultSpace)fallback=attrId(path,_tokens->avarsDefaultSpace);
            else if(*expression==_tokens->parentSpace)fallback=compId(path,"computedParentSpace");
            else fallback=compId(path,"computedParentDefaultSpace");
            add({RigExecProviderOpKind::SpaceExpression,property,
                program.FindValue(RigExecProviderAttributeKey(property)),
                {program.FindValue(RigExecProviderRawKey(property)),connect(fact->second.fact),fallback}},
                "spaceExpression:"+property.GetString());
        }
        add({RigExecProviderOpKind::MatrixToFrame,path,compId(path,"computeDefaultFrame"),{attrId(path,_tokens->defaultSpace)}},
            "defaultFrame:"+path.GetString());
        if(composeFrames) {
            RigExecProviderOp posed{RigExecProviderOpKind::PosedFrame,path,selfPose,
                {attrId(path,_tokens->posedSpace),attrId(path,_tokens->posedDefaultSpace),attrId(path,_tokens->parentDefaultSpace),attrId(path,_tokens->parentSpace),
                 attrId(path,_tokens->avarsUnitScaleFactor),attrId(path,_tokens->avarsTx),attrId(path,_tokens->avarsTy),attrId(path,_tokens->avarsTz),
                 attrId(path,_tokens->avarsSx),attrId(path,_tokens->avarsSy),attrId(path,_tokens->avarsSz),
                 attrId(path,_tokens->avarsRx),attrId(path,_tokens->avarsRy),attrId(path,_tokens->avarsRz),attrId(path,_tokens->avarsRspin),
                 attrId(path,_tokens->avarsRotationOrder),attrId(path,_tokens->avarsRotationSign)}};
            RigExecProviderOp avars{RigExecProviderOpKind::AvarMatrix,path,compId(path,"computeAvarMatrix"),
                std::vector<RigExecValueId>(posed.inputs.begin()+4,posed.inputs.end())};
            avars.scaleAvars=node.fact.type=="RigExecControl" || node.fact.type=="RigExecJoint";
            add(std::move(avars),"avarMatrix:"+path.GetString());
            posed.scaleAvars=node.fact.type=="RigExecControl" || node.fact.type=="RigExecJoint";
            add(std::move(posed),"posedFrame:"+path.GetString());
        }
        add({RigExecProviderOpKind::JointMatrix,path,compId(path,"computeMatrix"),{selfRest,selfPose}},"jointMatrix:"+path.GetString());
    }
    std::sort(program.leaves.begin(),program.leaves.end());
    program.leaves.erase(std::unique(program.leaves.begin(),program.leaves.end()),program.leaves.end());
    RigExecSpellProviderOwners(&program);
    *output=std::move(program);return true;
}
void RigExecSpellProviderOwners(RigExecProviderProgram *program) {
    if(!program)return;
    std::unordered_map<SdfPath,uint32_t,SdfPath::Hash> spelled;
    for(const auto &op:program->ops)
        if(op.ownerText<program->ownerTexts.size())spelled.emplace(op.owner,op.ownerText);
    for(auto &op:program->ops) {
        if(op.ownerText<program->ownerTexts.size())continue;
        const auto entry=spelled.emplace(op.owner,uint32_t(program->ownerTexts.size()));
        if(entry.second)program->ownerTexts.push_back(op.owner.GetString());
        op.ownerText=entry.first->second;
    }
}

void RigExecProviderContextIndex::Extend(const RigExecProviderProgram &program) {
    for(;indexedOps<program.ops.size();++indexedOps)
        producers.emplace(program.ops[indexedOps].output,indexedOps);
    for(;indexedExternals<program.externalInputs.size();++indexedExternals)
        externals.emplace(program.externalInputs[indexedExternals].value,indexedExternals);
}

bool RigExecCloneProviderContext(RigExecProviderProgram *program,RigExecValueId result,
    const std::string &prefix,const std::map<std::string,std::string> &externalNames,
    const std::map<RigExecValueId,RigExecValueId> &replacements,
    RigExecValueId *output,std::string *error) {
    return RigExecCloneProviderContext(program,result,prefix,externalNames,
        replacements,output,error,nullptr);
}
bool RigExecCloneProviderContext(RigExecProviderProgram *program,RigExecValueId result,
    const std::string &prefix,const std::map<std::string,std::string> &externalNames,
    const std::map<RigExecValueId,RigExecValueId> &replacements,
    RigExecValueId *output,std::string *error,RigExecProviderContextIndex *index) {
    if(!program || !output || result>=program->valueKeys.size() ||
        program->ops.size()!=program->descriptors.size())
        return Fail(error,"invalid provider context clone binding");
    RigExecProviderContextIndex local;
    if(!index)index=&local;
    index->Extend(*program);
    const auto &producer=index->producers;
    const auto &external=index->externals;
    std::map<RigExecValueId,RigExecValueId> remap;
    std::function<RigExecValueId(RigExecValueId)> clone=[&](RigExecValueId value)->RigExecValueId {
        if(value==RigExecNoProviderValue)return value;
        const auto replacement=replacements.find(value);
        if(replacement!=replacements.end())return replacement->second;
        const auto existing=remap.find(value);
        if(existing!=remap.end())return existing->second;
        const auto frame=external.find(value);const auto operation=producer.find(value);
        if(frame!=external.end() && !externalNames.count(program->externalInputs[frame->second].computation))return value;
        if(frame==external.end() && operation==producer.end())return value;
        const RigExecValueId next=RigExecValueId(program->valueKeys.size());
        const std::string key=prefix+program->valueKeys[size_t(value)];
        const auto already=program->valueIds.find(key);
        if(already!=program->valueIds.end()) { remap.emplace(value,already->second);return already->second; }
        remap.emplace(value,next);program->valueKeys.push_back(key);program->valueIds.emplace(key,next);
        if(frame!=external.end()) {
            auto input=program->externalInputs[frame->second];input.value=next;
            input.computation=externalNames.at(input.computation);
            program->externalInputs.push_back(std::move(input));return next;
        }
        auto op=program->ops[operation->second];auto descriptor=program->descriptors[operation->second];
        op.output=next;for(auto &input:op.inputs)input=clone(input);
        for(auto &xform:op.xforms)xform.raw=clone(xform.raw);
        descriptor.key=prefix+descriptor.key;descriptor.reads.clear();
        for(const auto input:op.inputs)if(input!=RigExecNoProviderValue)descriptor.reads.push_back(input);
        std::sort(descriptor.reads.begin(),descriptor.reads.end());
        descriptor.reads.erase(std::unique(descriptor.reads.begin(),descriptor.reads.end()),descriptor.reads.end());
        descriptor.writes={next};program->ops.push_back(std::move(op));program->descriptors.push_back(std::move(descriptor));
        return next;
    };
    *output=clone(result);return true;
}
namespace {
// The kinds RigExecRunProviderOp hands to RigExecRunProviderArithmetic.
bool ArithmeticKind(RigExecProviderOpKind kind) {
    return kind<=RigExecProviderOpKind::JointMatrix || kind==RigExecProviderOpKind::AvarMatrix ||
        kind==RigExecProviderOpKind::RelativeXform;
}
// The cases of RigExecRunProviderArithmetic's switch: the only kinds for
// which it does not return false.
bool ArithmeticHandles(RigExecProviderOpKind kind) {
    switch(kind) {
    case RigExecProviderOpKind::Attribute: case RigExecProviderOpKind::SpaceExpression:
    case RigExecProviderOpKind::RestFrame: case RigExecProviderOpKind::DefaultSpace:
    case RigExecProviderOpKind::FrameToSpace: case RigExecProviderOpKind::MatrixToFrame:
    case RigExecProviderOpKind::PosedFrame: case RigExecProviderOpKind::AvarMatrix:
    case RigExecProviderOpKind::RelativeXform: case RigExecProviderOpKind::JointMatrix:
        return true;
    default: return false;
    }
}
}
bool RigExecProviderOpStructurallyValid(const RigExecProviderProgram &program,
    uint32_t index,std::string *error) {
    if(index>=program.ops.size())return Fail(error,"invalid provider kernel/store binding");
    const auto &op=program.ops[index];
    if(op.output>=program.valueKeys.size())return Fail(error,"provider output is outside its typed layout");
    if(ArithmeticKind(op.kind))
        return ArithmeticHandles(op.kind) || Fail(error,"unsupported provider arithmetic operation kind");
    if(op.kind==RigExecProviderOpKind::AffineFrame) {
        if(op.affineKind>=affineTypeCount)return Fail(error,"unknown affine frame expression");
        if(!RigExecAffineFrameInputsMatch(op.affineKind,op.inputs.size(),op.affineTargets))
            return Fail(error,"affine frame inputs do not match their expression");
        return true;
    }
    if(op.kind!=RigExecProviderOpKind::LocalXform && op.kind!=RigExecProviderOpKind::InterveningXform)
        return Fail(error,"unknown provider operation kind");
    if(op.ownerText>=program.ownerTexts.size())
        return Fail(error,"provider operation owner was not spelled at build");
    return true;
}
bool RigExecRunProviderOp(const RigExecProviderProgram &program,uint32_t index,
    RigExecTypedValueStore *store,std::string *error) {
    if(!store || index>=program.ops.size() || store->values.size()<program.valueKeys.size())
        return Fail(error,"invalid provider kernel/store binding");
    if(!RigExecProviderOpStructurallyValid(program,index,error))return false;
    const auto &op=program.ops[index];
    const auto id=[&](size_t input){return input<op.inputs.size()?op.inputs[input]:RigExecNoProviderValue;};
    const auto raw=[&](RigExecValueId value)->const VtValue* {
        return value<store->values.size() && store->values[size_t(value)].initialized &&
            !store->values[size_t(value)].blocked?&store->values[size_t(value)].raw:nullptr;
    };
    const auto unavailable=[&](RigExecValueId source,const std::string &message) {
        const bool blocked=source<store->values.size() && store->values[size_t(source)].blocked;
        store->values[size_t(op.output)].raw=VtValue();
        store->Publish(op.output,std::monostate(),blocked,false,0,message);
        if(error)*error=message;
        return true;
    };
    if(ArithmeticKind(op.kind)) {
        GfProviderStore adapter{*store};
        if(!RigExecRunProviderArithmetic<GfProviderMath>(op,adapter))
            return Fail(error,"unsupported provider arithmetic operation kind");
        return true;
    }
    // The owner as the build spelled it, for the failure diagnostics below;
    // the structural check refused a LocalXform or InterveningXform without one.
    const std::string *spelled=op.ownerText<program.ownerTexts.size()?&program.ownerTexts[op.ownerText]:nullptr;
    switch(op.kind) {
    case RigExecProviderOpKind::LocalXform: {
        const std::string &owner=*spelled;
        GfMatrix4d result(1.0);
        const auto *order=raw(id(0));
        if((!order || !order->IsHolding<VtTokenArray>()) && !op.xforms.empty())
            return unavailable(id(0),"xform order unavailable: "+owner);
        if(order && order->IsHolding<VtTokenArray>()) {
            const auto &tokens=order->UncheckedGet<VtTokenArray>();
            for(size_t reverse=tokens.size();reverse>0;--reverse) {
                const auto &token=tokens[reverse-1];
                std::string_view name=token.GetString();
                if(name=="!resetXformStack!")break;
                const bool inverse=name.rfind("!invert!",0)==0;
                if(inverse)name.remove_prefix(8);
                if(reverse>1) {
                    std::string_view next=tokens[reverse-2].GetString();
                    const bool nextInverse=next.rfind("!invert!",0)==0;
                    if(nextInverse)next.remove_prefix(8);
                    if(name==next && inverse!=nextInverse){--reverse;continue;}
                }
                const auto input=std::find_if(op.xforms.begin(),op.xforms.end(),[&](const auto &candidate){return candidate.name.GetString()==name;});
                if(input==op.xforms.end())return unavailable(id(0),"xform order names an uncaptured operation: "+owner+":"+token.GetString());
                const auto *value=raw(input->raw);
                if(!value || value->IsEmpty())
                    return unavailable(input->raw,"xform operation unavailable: "+owner+":"+token.GetString());
                const auto transform=UsdGeomXformOp::GetOpTransform(UsdGeomXformOp::Type(input->type),*value,inverse);
                if(transform!=GfMatrix4d(1.0))result*=transform;
            }
        }
        store->Publish(op.output,result);break;
    }
    case RigExecProviderOpKind::InterveningXform: {
        const std::string &owner=*spelled;
        GfMatrix4d result(1.0);
        bool resets=false;
        for(size_t i=0;i<op.inputs.size();i+=2) {
            const auto *order=raw(id(i+1));
            if(order && order->IsHolding<VtTokenArray>())for(const auto &token:order->UncheckedGet<VtTokenArray>())
                if(token=="!resetXformStack!")resets=true;
            const auto *local=store->Read<GfMatrix4d>(id(i));
            if(!local)return unavailable(id(i),"intervening xform input unavailable: "+owner);
            result=result*(*local);
        }
        if(resets)store->Publish(op.output,GfMatrix4d(1.0),false,false,1,"resetXformStack in intervening provider ancestry");
        else store->Publish(op.output,result);
        break;
    }
    case RigExecProviderOpKind::AffineFrame: {
        // Validated by RigExecProviderOpStructurallyValid above.
        const RigExecAffineFrameType &type=RigExecAffineFrameTypes[op.affineKind];
        AffineInputs in;
        size_t at=0;
        for(uint32_t k=0;k<type.attributeCount;++k)AffineRead(*store,id(at++),type.attributes[k].field,&in);
        for(uint32_t k=0;k<type.relationCount;++k) {
            const GfMatrix4d frame=AffineFrameMatrix(store->Read<RigExecPointFrame>(id(at++)));
            RigExecVisitAffineField(in,type.relations[k].field,[&](auto &out) {
                if constexpr(std::is_same_v<std::decay_t<decltype(out)>,GfMatrix4d>)out=frame;
            });
        }
        if(type.targets) {
            for(uint32_t k=0;k<op.affineTargets;++k)
                in.targets.push_back(AffineFrameMatrix(store->Read<RigExecPointFrame>(id(at++))));
            while(at<op.inputs.size())
                in.targetObjects.push_back(AffineFrameMatrix(store->Read<RigExecPointFrame>(id(at++))));
        }
        GfMatrix4d result(1.0);
        const char *failure=nullptr;
        if(!AffineKernel::Compute(op.affineKind,in,&result,&failure))
            return unavailable(RigExecNoProviderValue,std::string("affine frame expression failed: ")+failure);
        store->Publish(op.output,result);
        break;
    }
    default:return Fail(error,"unknown provider operation kind");
    }
    return true;
}

bool RigExecSampleProviderProgram(const RigExecProviderProgram &program,
    const RigExecSceneDescriptors &scene,size_t identity,const std::map<SdfPath,VtValue> &overlays,
    RigExecTypedValueStore *store,std::vector<RigExecValueId> *changed,std::string *error) {
    if(!store || !changed || identity>=scene.identities.size() || store->values.size()<program.valueKeys.size())
        return Fail(error,"invalid provider sample identity/store binding");
    for(const auto &[path,value]:overlays) {
        const auto attribute=scene.attributes.find(path);
        if(attribute==scene.attributes.end() || (!value.IsEmpty() && value.GetType()!=attribute->second.fact.type.GetType()))
            return Fail(error,"provider overlay native type mismatch: "+path.GetString());
    }
    for(const auto &leaf:program.sampled) {
        const auto attribute=scene.attributes.find(leaf.attribute);
        if(attribute==scene.attributes.end() || identity>=attribute->second.inputs.size())return Fail(error,"provider source state missing");
    }
    changed->clear();
    for(const auto &leaf:program.sampled) {
        const auto attr=scene.attributes.find(leaf.attribute);
        if(attr==scene.attributes.end() || identity>=attr->second.inputs.size())return Fail(error,"provider source state missing");
        const auto overlay=overlays.find(leaf.attribute);
        const auto &source=attr->second.inputs[identity];
        const auto &value=overlay==overlays.end()?source.raw:overlay->second;
        if(!value.IsEmpty() && value.GetType()!=attr->second.fact.type.GetType())
            return Fail(error,"provider overlay native type mismatch: "+leaf.attribute.GetString());
        if(store->PublishSource(leaf.value,value,overlay==overlays.end()?source.rawBlocked:false,overlay!=overlays.end()))
            changed->push_back(leaf.value);
    }
    return true;
}
}
