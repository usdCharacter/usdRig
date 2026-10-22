#include "sceneProgramTranslation.h"
#include "pxr/base/tf/staticTokens.h"
#include "providerContextBinding.h"
#include "rigExecGraph/poseArithmetic.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExecMath/harmonicsKernel.h"
#include <set>
#include <algorithm>
#include <iterator>
#include <limits>
#include <memory>
namespace rigExec {
TF_DEFINE_PRIVATE_TOKENS(_tokens,
    (points) ((parentSpace,"parent:space")) (preceding) (base) ((inputsDefaultWeight,"inputs:defaultWeight"))
    ((rigExecWeightTarget,"rigExec:weightTarget")) ((rigExecWeightObject,"rigExec:weightObject")) (final) (dense) (strict)
    ((rigExecLimbJoints,"rigExec:limbJoints")) ((rigExecRootControl,"rigExec:rootControl")) ((rigExecEffector,"rigExec:effector"))
    ((rigExecIkControl,"rigExec:ikControl")) ((rigExecPoleControl,"rigExec:poleControl")) ((rigExecFkControls,"rigExec:fkControls"))
    ((outputsIkValues,"outputs:ikValues")) ((outputsFkValues,"outputs:fkValues"))
    ((rigExecJoints,"rigExec:joints"))
);
namespace {
bool Fail(std::string *error,const std::string &message){if(error)*error=message;return false;}
bool Hierarchical(const TfToken &type){return type=="RigExecControl" || type=="RigExecJoint" || type=="RigExecSphereWeight" || type=="RigExecPlaneWeight" || type=="RigExecCurveWeight";}
RigExecSceneValueDomain Domain(RigExecSceneDomain domain) {
    if(domain==RigExecSceneDomain::GeometryMover)return RigExecSceneValueDomain::Points;
    if(domain==RigExecSceneDomain::PropertyMover)return RigExecSceneValueDomain::Property;
    return RigExecSceneValueDomain::Pose;
}
}
static bool TranslateSceneProgramOnce(const RigExecSceneDescriptors &captured,
    const std::set<SdfPath> &excludedPoseWriters,const std::set<SdfPath> &excludedSolverBodies,
    const RigExecOpExclusionProof &proof,
    RigExecSceneProgram *destination,std::string *error) {
    RigExecSceneDescriptors scene=captured;
    // rigEvaluatorCompile's consumed-solver relaxation: a solver another
    // solver reads (an IK feeding a blend) names the joints an unread solver
    // poses for their rests only. Every other joint binding writes, and
    // several writers of one joint stack in stack order (below).
    {
        std::set<SdfPath> solvers,consumed,posedByUnconsumed;
        std::set<std::pair<SdfPath,SdfPath>> consumers;
        for(const auto &[path,node]:scene.nodes)if(node.domain==RigExecSceneDomain::Solver)solvers.insert(path);
        for(const auto &solver:solvers)for(const auto &property:scene.nodes.at(solver).relationships) {
            if(property.GetNameToken()==_tokens->rigExecJoints)continue;
            const auto relation=scene.relationships.find(property);if(relation==scene.relationships.end())continue;
            for(const auto &target:relation->second.fact.targets) {
                const auto prim=target.GetPrimPath();
                if(prim!=solver && solvers.count(prim)){consumed.insert(prim);consumers.insert({prim,solver});}
            }
        }
        // A consumed solver has no place of its own in the pose stack: data
        // flow runs it just before its first consumer, so its preceding reads
        // (a live joint rest) stand where that consumer's do and never see
        // the consumer's own writes.
        for(size_t pass=0;pass<=solvers.size();++pass) {
            bool moved=false;
            for(const auto &[producer,consumer]:consumers) {
                auto &own=scene.nodes.at(producer).stackOrdinal;const int reader=scene.nodes.at(consumer).stackOrdinal;
                if(reader>=0 && (own<0 || reader<own)){own=reader;moved=true;}
            }
            if(!moved)break;
        }
        for(const auto &binding:scene.jointBindings)if(!consumed.count(binding.solver))posedByUnconsumed.insert(binding.joint);
        scene.jointBindings.erase(std::remove_if(scene.jointBindings.begin(),scene.jointBindings.end(),
            [&](const auto &binding){return consumed.count(binding.solver) && posedByUnconsumed.count(binding.joint);}),
            scene.jointBindings.end());
    }
    scene.jointBindings.erase(std::remove_if(scene.jointBindings.begin(),scene.jointBindings.end(),
        [&](const auto &binding){return excludedPoseWriters.count(binding.solver)!=0;}),scene.jointBindings.end());
    scene.applications.erase(std::remove_if(scene.applications.begin(),scene.applications.end(),
        [&](const auto &application){return excludedPoseWriters.count(application.owner)!=0 &&
            (application.domain==RigExecSceneDomain::Constraint || application.domain==RigExecSceneDomain::AutoClavicle);}),scene.applications.end());
    // Authored stack scope and external input closure are separate: dependency
    // capture never imports the external source's mover application stack.
    std::set<SdfPath> needed;std::vector<SdfPath> pending;
    auto include=[&](const SdfPath &path) {const auto prim=path.GetPrimPath();if(scene.nodes.count(prim) && needed.insert(prim).second)pending.push_back(prim);};
    std::set<SdfPath> authored;
    const bool requestedPseudo=scene.rigRoot==SdfPath::AbsoluteRootPath() && !scene.compilerTargets.empty();
    SdfPathVector authoredRoots;
    if(requestedPseudo) {
        for(const auto &[path,node]:scene.nodes)if(node.fact.type=="RigExecRoot" && node.fact.active)authoredRoots.push_back(path);
    } else authoredRoots.push_back(scene.rigRoot);
    for(const auto &[path,node]:scene.nodes)if(node.fact.active)
        for(const auto &root:authoredRoots)if(path.HasPrefix(root)){authored.insert(path);include(path);break;}
    for(const auto &target:scene.compilerTargets)if(target.IsPrimPath())authored.insert(target);

    for(const auto &target:scene.compilerTargets)include(target);
    for(size_t i=0;i<pending.size();++i) {
        const auto path=pending[i];const auto &node=scene.nodes.at(path);
        for(const auto &property:node.attributes) {
            const auto attribute=scene.attributes.find(property);if(attribute==scene.attributes.end())continue;
            for(const auto &source:attribute->second.fact.connections)include(source);
        }
        for(const auto &property:node.relationships) {
            const auto relation=scene.relationships.find(property);if(relation==scene.relationships.end())continue;
            for(const auto &target:relation->second.forwardedTargets)include(target);
        }
        for(auto parent=path.GetParentPath();!parent.IsEmpty();parent=parent.GetParentPath()) {
            const auto ancestor=scene.nodes.find(parent);if(ancestor==scene.nodes.end())continue;
            // Provider ancestors contribute default/rest computations. Ordinary
            // Xform ancestors contribute their raw operation dependency chain.
            if(ancestor->second.transformProvider)include(parent);else needed.insert(parent);
        }
    }
    for(auto it=scene.nodes.begin();it!=scene.nodes.end();) {
        if(!needed.count(it->first))it=scene.nodes.erase(it);
        else {if(it->second.domain==RigExecSceneDomain::UnknownRigExec && !authored.count(it->first))it->second.domain=RigExecSceneDomain::Data;++it;}
    }
    for(auto it=scene.attributes.begin();it!=scene.attributes.end();)if(!needed.count(it->first.GetPrimPath()))it=scene.attributes.erase(it);else ++it;
    for(auto it=scene.relationships.begin();it!=scene.relationships.end();)if(!needed.count(it->first.GetPrimPath()))it=scene.relationships.erase(it);else ++it;
    scene.applications.erase(std::remove_if(scene.applications.begin(),scene.applications.end(),
        [&](const auto &application){return !authored.count(application.owner);}),scene.applications.end());
    scene.jointBindings.erase(std::remove_if(scene.jointBindings.begin(),scene.jointBindings.end(),
        [&](const auto &binding){return !authored.count(binding.solver);}),scene.jointBindings.end());
    if(!destination || scene.identities.empty())return Fail(error,"missing scene compiler input identities");
    for(const auto &[path,node]:scene.nodes)if(node.fact.active && node.domain==RigExecSceneDomain::UnknownRigExec)
        return Fail(error,"unsupported RigExec scene domain "+node.fact.type.GetString()+": "+path.GetString());
    // A RigExec API schema with no lowering refuses the program; any other
    // applied schema (physics, pipeline, material) is not the rig's and is
    // ignored, as the native evaluator ignores it.
    const std::set<std::string> loweredRigExecApis{"RigExecMoverAPI","RigExecControlAPI"};
    for(const auto &[path,node]:scene.nodes)for(const auto &applied:node.fact.appliedSchemas) {
        const auto &text=applied.GetString();
        const auto name=text.substr(0,text.find(':'));
        if(name.compare(0,7,"RigExec")==0 && !loweredRigExecApis.count(name))
            return Fail(error,"unsupported applied RigExec schema "+text+": "+path.GetString());
    }
    std::map<SdfPath,SdfPathVector> moverTargets;
    for(const auto &application:scene.applications)if(application.domain==RigExecSceneDomain::GeometryMover || application.domain==RigExecSceneDomain::PropertyMover)
        moverTargets[application.owner].push_back(application.canonicalTarget);
    for(const auto &[owner,targets]:moverTargets) {
        const auto node=scene.nodes.find(owner);if(node==scene.nodes.end())return Fail(error,"missing mover node");
        const auto *handler=RigExecFindMoverHandler(node->second.fact.type);
        if(handler && handler->singleTarget && targets.size()!=1)
            return Fail(error,node->second.fact.type.GetString()+" "+owner.GetString()+" must have exactly one canonical points target in v0.1 (multi-target fan-out would alias mover-level parameters)");
        if(handler && handler->legacyEnvelopeAttribute) {
            // Matched by name among the captured attributes: no token from text.
            auto old=scene.attributes.end();
            for(const auto &path:node->second.attributes)
                if(path.GetName()==handler->legacyEnvelopeAttribute){old=scene.attributes.find(path);break;}
            if(old!=scene.attributes.end() && old->second.fact.hasAuthoredReadableValue)
                return Fail(error,node->second.fact.type.GetString()+" "+owner.GetString()+" authors "+handler->legacyEnvelopeAttribute+", which was replaced by inputs:defaultWeight");
        }
        if(node->second.domain==RigExecSceneDomain::GeometryMover)for(const auto &target:targets) {
            const auto prim=scene.nodes.find(target.GetPrimPath());const auto attr=scene.attributes.find(target);
            if(!target.IsPropertyPath() || target.GetNameToken()!=_tokens->points || prim==scene.nodes.end() || !prim->second.pointBased || attr==scene.attributes.end() || attr->second.fact.type!=SdfValueTypeNames->Point3fArray)
                return Fail(error,node->second.fact.type.GetString()+" "+owner.GetString()+" target "+target.GetString()+" is not a native UsdGeomPointBased point3f[] points attribute");
        }
    }
    RigExecSceneProgram program;if(!program.layout.Prepare(scene,error))return false;
    for(const auto id:program.layout.unavailableFrames)
        if(!program.Append("inactiveProvider:"+program.layout.valueKeys[size_t(id)],
            RigExecSceneUnavailableFrameOp{id},error))return false;
    // Rest measurements use precisely the same shared provider arithmetic as
    // execution, with captured source leaves and no temporary source stage.
    if(!program.Compile(RigExecCyclePolicy::SetAside,error))return false;
    RigExecSceneProgramRuntime seed;
    if(!seed.Prepare(program,error) || !seed.Evaluate(program,0,{},error))return false;
    std::map<SdfPath,RigExecPointFrame> rests;
    for(const auto &[path,node]:scene.nodes) {
        const auto id=program.layout.providers.FindValue(RigExecProviderValueKey(path,"computeRestFrame"));
        if(const auto *frame=seed.values.Read<RigExecPointFrame>(id))rests.emplace(path,*frame);
    }
    // Solver joint extraction is a producer in its own right. Register the
    // solver base before any relationship selects a pose read version.
    // Several solvers writing one joint stack in stack order, as natively:
    // each later writer's commit starts from the version the earlier one
    // published, and the joint's solver base is the last of them.
    std::map<SdfPath,RigExecValueId> extracted;
    std::map<SdfPath,RigExecPoseCommitBinding> solverCommits;
    std::map<SdfPath,SdfPathVector> solverJointPaths,solverDescendantPaths;
    std::vector<SdfPath> solverOrder;
    const auto solverOrdinal=[&](const SdfPath &solver) {
        const auto node=scene.nodes.find(solver);
        return node==scene.nodes.end() || node->second.stackOrdinal<0?std::numeric_limits<int>::max():node->second.stackOrdinal;
    };
    std::vector<const RigExecSceneJointBindingDescriptor *> stackedBindings;
    for(const auto &binding:scene.jointBindings)stackedBindings.push_back(&binding);
    std::stable_sort(stackedBindings.begin(),stackedBindings.end(),[&](const auto *a,const auto *b){return solverOrdinal(a->solver)<solverOrdinal(b->solver);});
    for(const auto *stacked:stackedBindings) {
        const auto &binding=*stacked;
        const auto chain=program.layout.routes.chains.find({binding.joint,RigExecSceneValueDomain::Pose});
        const auto aggregate=program.layout.solverAggregates.find(binding.solver);
        if(chain==program.layout.routes.chains.end() || aggregate==program.layout.solverAggregates.end())continue;
        const auto earlier=extracted.find(binding.joint);
        const auto name=binding.solver.GetString()+":"+binding.joint.GetString();
        const auto id=program.layout.Allocate("solverJointCandidate:"+name);
        const auto published=program.layout.Allocate("solverJoint:"+name);
        // An unavailable solve passes through what the joint holds before it.
        const auto fallback=earlier==extracted.end()?chain->second.base:earlier->second;
        extracted[binding.joint]=published;
        if(!solverCommits.count(binding.solver))solverOrder.push_back(binding.solver);
        auto &commit=solverCommits[binding.solver];commit.candidates.push_back({chain->second.base,id,published});
        commit.reads.push_back(chain->second.base);commit.reads.push_back(id);commit.writes.push_back(published);solverJointPaths[binding.solver].push_back(binding.joint);
        RigExecSceneExtractOp op;op.aggregate=aggregate->second;op.fallback=fallback;op.output=id;op.element=binding.element;
        chain->second.poseBase=published;
        const auto solverNode=scene.nodes.find(binding.solver);
        if(solverNode!=scene.nodes.end() && solverNode->second.stackOrdinal>=0 && !program.layout.routes.Append({binding.joint,RigExecSceneValueDomain::Pose},{published,binding.solver,solverNode->second.stackOrdinal},error))return false;
        if(!program.Append("solverJoint:"+name,op,error))return false;
    }
    for(const auto &solver:solverOrder) {
        auto &commit=solverCommits.at(solver);
        const auto &joints=solverJointPaths.at(solver);
        for(const auto &[path,node]:scene.nodes)if(Hierarchical(node.fact.type) && node.fact.active && !extracted.count(path)) {
            size_t selected=SIZE_MAX;SdfPath closest;
            for(size_t i=0;i<joints.size();++i)if(path.HasPrefix(joints[i]) &&
                (closest.IsEmpty() || joints[i].GetPathElementCount()>closest.GetPathElementCount())){closest=joints[i];selected=i;}
            if(selected==SIZE_MAX)continue;
            bool solverBarrier=false,parentBlocked=false;
            for(auto parent=path.GetParentPath();!parent.IsEmpty() && parent!=closest;parent=parent.GetParentPath())solverBarrier|=extracted.count(parent)!=0;
            for(auto parent=path;!parent.IsEmpty() && parent!=closest;parent=parent.GetParentPath()) {
                const auto attribute=scene.attributes.find(parent.AppendProperty(_tokens->parentSpace));
                if(attribute!=scene.attributes.end())parentBlocked|=attribute->second.fact.hasAuthoredValue ||
                    (attribute->second.fact.hasAuthoredConnections && !attribute->second.fact.connections.empty());
            }
            if(solverBarrier)continue;
            auto chain=program.layout.routes.chains.find({path,RigExecSceneValueDomain::Pose});if(chain==program.layout.routes.chains.end())continue;
            const auto output=program.layout.Allocate("solverPropagate:"+solver.GetString()+":"+path.GetString());
            const auto current=chain->second.base;chain->second.poseBase=output;
            const auto solverNode=scene.nodes.find(solver);
            if(solverNode!=scene.nodes.end() && solverNode->second.stackOrdinal>=0 && !program.layout.routes.Append({path,RigExecSceneValueDomain::Pose},{output,solver,solverNode->second.stackOrdinal},error))return false;
            commit.descendants.push_back({current,output,selected,parentBlocked,true});commit.reads.push_back(current);commit.writes.push_back(output);
            solverDescendantPaths[solver].push_back(path);
        }
    }
    std::map<std::pair<SdfPath,SdfPath>,RigExecSceneGeometryDescriptor> geometryDescriptors;
    std::vector<RigExecSceneApplicationDescriptor> authoredApplications;std::set<SdfPath> movedPoints;
    // Optional driver-frames admission is revision-local. Resolve the real
    // typed descriptor first, so malformed read/binding declarations still
    // fail instead of being hidden by an unavailable aggregate.
    const auto admitDriverFrames=[&](const RigExecSceneGeometryDescriptor &descriptor) {
        const auto &driver=descriptor.record.binding.driverFrames;
        if(driver.IsEmpty() || (authored.count(driver) &&
           program.layout.solverAggregates.count(driver) && !excludedSolverBodies.count(driver)))return true;
        program.skippedOperations[descriptor.mover]="driver frames "+driver.GetString()+
            " is not a solver of this rig; mover set aside";
        return false;
    };
    int terminalOrdinal=0;for(const auto &[path,node]:scene.nodes)terminalOrdinal=std::max(terminalOrdinal,node.stackOrdinal+1);
    for(const auto &application:scene.applications)if(application.domain==RigExecSceneDomain::GeometryMover) {
        const auto node=scene.nodes.find(application.owner);if(node!=scene.nodes.end() && node->second.fact.type!="RigExecSurfaceProjector")movedPoints.insert(application.canonicalTarget);
    }
    for(const auto &application:scene.applications) {
        if(application.domain!=RigExecSceneDomain::GeometryMover){authoredApplications.push_back(application);continue;}
        const auto node=scene.nodes.find(application.owner);
        if(node!=scene.nodes.end() && node->second.fact.type=="RigExecSurfaceProjector" && !movedPoints.count(application.canonicalTarget))continue;
        std::vector<RigExecSceneGeometryDescriptor> variants;
        if(!RigExecTranslateSceneGeometryVariants(scene,application.owner,application.canonicalTarget,&variants,error))return false;
        for(size_t i=0;i<variants.size();++i) {
            if(!admitDriverFrames(variants[i]))continue;
            auto expanded=application;expanded.target=variants[i].target;expanded.canonicalTarget=variants[i].target;expanded.targetIndex=application.targetIndex*2+i;
            geometryDescriptors.emplace(std::make_pair(expanded.owner,expanded.canonicalTarget),std::move(variants[i]));authoredApplications.push_back(std::move(expanded));
        }
    }
    for(const auto &points:movedPoints) {
        std::vector<RigExecSceneGeometryDescriptor> variants;if(!RigExecTranslateSceneDerivedGeometry(scene,points,&variants,error))return false;
        for(size_t i=0;i<variants.size();++i) {
            auto &descriptor=variants[i];if(!admitDriverFrames(descriptor))continue;
            RigExecSceneApplicationDescriptor application{descriptor.mover,descriptor.target,RigExecSceneDomain::GeometryMover,terminalOrdinal,descriptor.target,i};
            geometryDescriptors.emplace(std::make_pair(application.owner,application.target),std::move(descriptor));authoredApplications.push_back(std::move(application));
        }
    }
    const auto applicationDomain=[&](const RigExecSceneApplicationDescriptor &application) {
        if(application.domain==RigExecSceneDomain::GeometryMover)return application.canonicalTarget.GetNameToken()=="points"?RigExecSceneValueDomain::Points:RigExecSceneValueDomain::Property;
        return Domain(application.domain);
    };
    std::map<SdfPath,RigExecSceneConstraintDescriptor> chainConstraints;
    for(const auto &[path,node]:scene.nodes)if(!excludedPoseWriters.count(path) && node.stackOrdinal>=0 && node.fact.active && node.fact.type=="RigExecSingleChainIkConstraint") {
        RigExecSceneConstraintDescriptor source;if(!RigExecTranslateSceneConstraint(scene,path,{},&source,error))return false;
        authoredApplications.erase(std::remove_if(authoredApplications.begin(),authoredApplications.end(),[&](const auto &a){return a.owner==path;}),authoredApplications.end());
        for(size_t i=0;i<source.ikChain.size();++i)authoredApplications.push_back({path,source.ikChain[i],RigExecSceneDomain::Constraint,node.stackOrdinal,source.ikChain[i],i});
        chainConstraints.emplace(path,std::move(source));
    }
    // An IK/FK match revises no frame a reader sees: its rigExec:moves only
    // restates the controls it presents, so it takes no pose application.
    // It is bound once every pose chain and space switch is (below).
    std::vector<SdfPath> ikFkMatches;
    for(const auto &[path,node]:scene.nodes)if(node.fact.active && authored.count(path) &&
        node.domain==RigExecSceneDomain::Constraint && node.fact.type=="RigExecIkFkMatchConstraint") {
        authoredApplications.erase(std::remove_if(authoredApplications.begin(),authoredApplications.end(),[&](const auto &a){return a.owner==path;}),authoredApplications.end());
        ikFkMatches.push_back(path);
    }
    std::stable_sort(ikFkMatches.begin(),ikFkMatches.end(),[&](const SdfPath &a,const SdfPath &b){return scene.nodes.at(a).stackOrdinal<scene.nodes.at(b).stackOrdinal;});
    std::stable_sort(authoredApplications.begin(),authoredApplications.end(),[](const auto &a,const auto &b){return a.stackOrdinal<b.stackOrdinal || (a.stackOrdinal==b.stackOrdinal && a.targetIndex<b.targetIndex);});
    struct Application { const RigExecSceneApplicationDescriptor *source;RigExecValueId incoming,output,candidate; };
    std::vector<Application> applications;std::map<SdfPath,RigExecPoseCommitBinding> commits;
    for(size_t begin=0;begin<authoredApplications.size();) {
        size_t end=begin+1;while(end<authoredApplications.size() && authoredApplications[end].owner==authoredApplications[begin].owner)++end;
        const auto applicationBegin=applications.size();
        for(size_t i=begin;i<end;++i) {
            const auto &application=authoredApplications[i];
            const RigExecSceneVersionKey key{application.canonicalTarget,applicationDomain(application)};
            const auto chain=program.layout.routes.chains.find(key);
            if(chain==program.layout.routes.chains.end()) {
                const auto base=program.layout.Allocate("missingBase:"+application.canonicalTarget.GetString());program.layout.constants.emplace(base,VtValue());program.layout.leaves.push_back(base);
                if(!program.layout.routes.RegisterBase(key,base,error))return false;
            }
            const auto targetChain=program.layout.routes.chains.find(key);
            auto incoming=targetChain->second.base;
            for(const auto &version:targetChain->second.versions)if(version.ordinal<application.stackOrdinal)incoming=version.value;else break;
            const auto name="revision:"+application.owner.GetString()+":"+application.canonicalTarget.GetString()+":"+std::to_string(application.targetIndex);
            const auto output=program.layout.Allocate(name);
            const auto candidate=key.domain==RigExecSceneValueDomain::Pose?program.layout.Allocate("candidate:"+name):output;
            if(!program.layout.routes.Append(key,{output,application.owner,application.stackOrdinal},error))return false;
            applications.push_back({&application,incoming,output,candidate});
        }
        if(applicationDomain(authoredApplications[begin])==RigExecSceneValueDomain::Pose) {
            RigExecPoseCommitBinding commit;
            for(size_t i=applicationBegin;i<applications.size();++i) {
                const auto &a=applications[i];commit.candidates.push_back({a.incoming,a.candidate,a.output});
                commit.reads.push_back(a.incoming);commit.reads.push_back(a.candidate);commit.writes.push_back(a.output);
            }
            for(const auto &[path,node]:scene.nodes)if(Hierarchical(node.fact.type) && node.fact.active) {
                size_t selected=SIZE_MAX;SdfPath closest;
                bool direct=false;
                for(size_t i=applicationBegin;i<applications.size();++i) {
                    const auto target=applications[i].source->canonicalTarget;
                    if(path==target){direct=true;break;}
                    if(path.HasPrefix(target) && (closest.IsEmpty() || target.GetPathElementCount()>closest.GetPathElementCount())){closest=target;selected=i-applicationBegin;}
                }
                if(direct || selected==SIZE_MAX)continue;
                bool solverBarrier=false,parentBlocked=false;
                for(auto parent=path;!parent.IsEmpty() && parent!=closest;parent=parent.GetParentPath()) {
                    solverBarrier|=extracted.count(parent)!=0;
                    const auto attribute=scene.attributes.find(parent.AppendProperty(_tokens->parentSpace));
                    if(attribute!=scene.attributes.end())parentBlocked|=attribute->second.fact.hasAuthoredValue ||
                        (attribute->second.fact.hasAuthoredConnections && !attribute->second.fact.connections.empty());
                }
                if(solverBarrier)continue;
                auto chain=program.layout.routes.chains.find({path,RigExecSceneValueDomain::Pose});if(chain==program.layout.routes.chains.end())continue;
                auto current=chain->second.base;
                for(const auto &version:chain->second.versions)if(version.ordinal<authoredApplications[begin].stackOrdinal)current=version.value;else break;
                const auto output=program.layout.Allocate("propagate:"+authoredApplications[begin].owner.GetString()+":"+path.GetString());
                if(!program.layout.routes.Append({path,RigExecSceneValueDomain::Pose},{output,authoredApplications[begin].owner,authoredApplications[begin].stackOrdinal},error))return false;
                commit.descendants.push_back({current,output,selected,parentBlocked,false});commit.reads.push_back(current);commit.writes.push_back(output);
            }
            commits.emplace(authoredApplications[begin].owner,std::move(commit));
        }
        begin=end;
    }
    for(auto &[solver,commit]:solverCommits) {
        commit.reads.clear();const auto &joints=solverJointPaths.at(solver);
        for(size_t i=0;i<commit.candidates.size();++i) {
            auto &candidate=commit.candidates[i];
            if(!program.layout.routes.Resolve(scene,{joints[i],RigExecSceneValueDomain::Pose},solver,_tokens->preceding,&candidate.before,error))return false;
            commit.reads.push_back(candidate.before);commit.reads.push_back(candidate.candidate);
        }
        const auto descendantPaths=solverDescendantPaths.find(solver);
        if(descendantPaths==solverDescendantPaths.end()?!commit.descendants.empty():descendantPaths->second.size()!=commit.descendants.size())
            return Fail(error,"solver descendant paths do not match its commit: "+solver.GetString());
        for(size_t i=0;i<commit.descendants.size();++i) {
            auto &descendant=commit.descendants[i];
            if(!program.layout.routes.Resolve(scene,{descendantPaths->second[i],RigExecSceneValueDomain::Pose},solver,_tokens->preceding,&descendant.current,error))return false;
            commit.reads.push_back(descendant.current);
        }
        if(!program.Append("solverCommit:"+solver.GetString(),std::move(commit),error))return false;
    }
    std::map<SdfPath,std::pair<std::vector<RigExecValueId>,std::vector<RigExecValueId>>> interpolatorOutputs;
    std::map<SdfPath,std::vector<RigExecValueId>> interpolatorPrimvars;
    // An ML mover is a geometry mover whose network publishes like an
    // interpolator as well.
    const auto publishesWeights=[](const RigExecSceneNodeDescriptor &node) {
        return node.domain==RigExecSceneDomain::PoseInterpolator || node.fact.type=="RigExecMlMover";
    };
    for(const auto &[path,node]:scene.nodes)if(publishesWeights(node)) {
        RigExecScenePoseInterpolatorDescriptor descriptor;
        if(!RigExecTranslateScenePoseInterpolator(scene,path,&descriptor,error))return false;
        auto &outputs=interpolatorOutputs[path];
        auto reserve=[&](const SdfPathVector &paths,std::vector<RigExecValueId> *ids) {
            for(const auto &attribute:paths) {
                const auto id=program.layout.Allocate("poseWeight:"+attribute.GetString());ids->push_back(id);
                const RigExecSceneVersionKey key{attribute,RigExecSceneValueDomain::Property};
                if(!program.layout.routes.chains.count(key)) {
                    if(!program.layout.routes.RegisterBase(key,id,error))return false;
                } else if(!program.layout.routes.Append(key,{id,path,node.stackOrdinal},error))return false;
            }return true;
        };
        if(!reserve(descriptor.poseWeights,&outputs.first) || !reserve(descriptor.disabledPoseWeights,&outputs.second) ||
           !reserve(descriptor.primvarPaths,&interpolatorPrimvars[path]))return false;
    }
    auto context=program.layout.BindingContext(scene);
    const auto directResolve=context.resolve;std::map<std::string,RigExecValueId> contexts;
    std::map<RigExecValueId,RigExecValueId> defaultEffectiveInputs;
    context.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *output,std::string *why) {
        RigExecValueId source;if(!directResolve(request,&source,why))return false;
        if(request.raw || (request.liveRest && *request.liveRest)){*output=source;return true;}
        const auto consumerFact=scene.attributes.find(request.consumer);
        const bool connected=consumerFact!=scene.attributes.end() && consumerFact->second.fact.connections.size()==1;
        const bool expression=request.atDefault || connected;
        if(!expression || (!request.computation.empty() && request.computation!="computeRestFrame") || source>=program.layout.providers.valueKeys.size()){*output=source;return true;}
        const auto phase=request.phase.IsEmpty()?_tokens->base:request.phase;
        const auto key=request.consumer.GetString()+"|"+request.reader.GetString()+"|"+phase.GetString()+"|"+std::to_string(source)+(request.atDefault?"|default":"|current");
        const auto cached=contexts.find(key);if(cached!=contexts.end()){*output=cached->second;return true;}
        std::map<RigExecValueId,RigExecValueId> selected;
        for(const auto &[route,chain]:program.layout.routes.chains) {
            if(route.domain==RigExecSceneValueDomain::Pose && connected) {
                const auto original=program.layout.providers.FindValue(RigExecProviderValueKey(route.path,"computePointFrame"));
                RigExecValueId value;if(original!=UINT64_MAX && program.layout.routes.Resolve(scene,route,request.reader,phase,&value,nullptr) && original!=value)selected.emplace(original,value);
            } else if(route.domain==RigExecSceneValueDomain::Property && !chain.versions.empty()) {
                const auto original=program.layout.providers.attributeValues.find(route.path);RigExecValueId value;
                if(original!=program.layout.providers.attributeValues.end() && program.layout.routes.Resolve(scene,route,request.reader,phase,&value,nullptr) && value!=chain.base)selected.emplace(original->second,value);
            }
        }
        if(request.atDefault && request.computation.empty()) {
            for(const auto &sample:program.layout.samples)if(sample.providerRaw!=UINT64_MAX)selected[sample.providerRaw]=sample.selectedDefault;
            selected.insert(defaultEffectiveInputs.begin(),defaultEffectiveInputs.end());
        }
        size_t begin;if(!RigExecBindProviderContext(&program.layout.providers,program.layout.valueKeys,source,request.consumer,phase.GetString()+(request.atDefault?":default":""),selected,output,&begin,why))return false;
        program.layout.valueKeys=program.layout.providers.valueKeys;
        for(size_t i=begin;i<program.layout.providers.ops.size();++i) {
            RigExecSceneProviderOp op;op.originalIndex=uint32_t(i);op.output=program.layout.providers.ops[i].output;op.reads=program.layout.providers.descriptors[i].reads;
            if(!program.Append(program.layout.providers.descriptors[i].key,std::move(op),why))return false;
        }
        contexts.emplace(key,*output);return true;
    };
    context.effectiveRead=[&](const RigExecGraphTypedRead &read,RigExecValueId *output,std::string *why) {
        RigExecSceneEffectiveReadOp op;op.read=read;
        op.output=program.layout.Allocate("effectiveRead:"+std::to_string(program.operations.size()));
        for(const auto &hops:{read.hops,read.doubleHops})for(const auto &hop:hops)
            for(auto id:{hop.raw,hop.overlay})if(id!=UINT64_MAX)op.reads.push_back(id);
        *output=op.output;
        return program.Append("effectiveRead:"+std::to_string(op.output),std::move(op),why);
    };
    if(!RigExecRebindProviderInputs(&program.layout.providers,program.layout.initialProviderOps,
        [&](const SdfPath &attribute,RigExecValueId original,RigExecValueId *bound,std::string *why) {
            const auto found=scene.attributes.find(attribute);
            if(found==scene.attributes.end() || found->second.fact.type==SdfValueTypeNames->Matrix4d){*bound=original;return true;}
            RigExecSceneTypedRead captured;RigExecGraphTypedRead read;
            if(!RigExecBindSceneTypedRead(scene,attribute,found->second.fact.type,&captured,why) ||
               !RigExecBindGraphTypedRead(captured,context,&read,why))return false;
            *bound=read.effective;
            auto defaultContext=context;const auto resolveDefault=context.resolve;
            defaultContext.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *error) {
                auto atDefault=request;atDefault.atDefault=true;return resolveDefault(atDefault,id,error);
            };
            RigExecGraphTypedRead defaultRead;
            if(!RigExecBindGraphTypedRead(captured,defaultContext,&defaultRead,why))return false;
            defaultEffectiveInputs[read.effective]=defaultRead.effective;
            program.layout.providers.valueKeys=program.layout.valueKeys;return true;
        },error))return false;
    // Switches read the final switched composition, before solver/constraint
    // commits. Parent checkpoints select the original semantic context rather
    // than introducing an edge to a later nested switch.
    std::vector<SdfPath> switchProviders;
    std::map<SdfPath,int> switchProviderIndex;
    for(const auto &[path,node]:scene.nodes)if(node.fact.active && node.transformProvider) {
        switchProviderIndex[path]=int(switchProviders.size());switchProviders.push_back(path);
    }
    std::vector<int> switchParents(switchProviders.size(),-1),switchSlots,switchCarries;
    std::vector<uint8_t> switchComposed(switchProviders.size(),0);
    std::vector<std::vector<int>> switchSources;
    std::vector<const Application *> switchApplications;
    std::map<SdfPath,RigExecValueId> switchedComposition;
    for(size_t i=0;i<switchProviders.size();++i) {
        const auto path=switchProviders[i];switchComposed[i]=Hierarchical(scene.nodes.at(path).fact.type);
        for(auto parent=path.GetParentPath();!parent.IsEmpty();parent=parent.GetParentPath()) {
            const auto found=switchProviderIndex.find(parent);
            if(found!=switchProviderIndex.end()){switchParents[i]=found->second;break;}
        }
        const auto original=program.layout.providers.FindValue(RigExecProviderValueKey(path,"computePointFrame"));
        if(original!=UINT64_MAX)switchedComposition[path]=original;
    }
    for(const auto &application:applications)if(application.source->domain==RigExecSceneDomain::SpaceSwitch) {
        const auto &a=*application.source;RigExecSceneSpaceSwitchDescriptor source;
        if(!RigExecTranslateSceneSpaceSwitch(scene,a.owner,&source,error))return false;
        const auto slot=switchProviderIndex.find(a.canonicalTarget);
        if(slot==switchProviderIndex.end())return Fail(error,"space switch target has no composed provider: "+a.canonicalTarget.GetString());
        switchApplications.push_back(&application);switchSlots.push_back(slot->second);switchSources.emplace_back();
        const auto carry=switchProviderIndex.find(source.space);switchCarries.push_back(carry==switchProviderIndex.end()?-1:carry->second);
        for(const auto &path:source.sources) {
            const auto found=switchProviderIndex.find(path);
            if(found!=switchProviderIndex.end())switchSources.back().push_back(found->second);
        }

        switchedComposition[a.canonicalTarget]=application.candidate;
    }
    // A switched subtree is composed from its nearest switch candidate. Generic
    // pose commit carry versions are not composition sources: they can include
    // the reader switch itself and would invent a dependency cycle.
    std::map<RigExecValueId,RigExecValueId> switchedRoots;
    std::set<SdfPath> switchedTargets;
    for(const auto *application:switchApplications) {
        const auto path=application->source->canonicalTarget;
        const auto original=program.layout.providers.FindValue(RigExecProviderValueKey(path,"computePointFrame"));
        if(original==UINT64_MAX)return Fail(error,"missing switched composition root: "+path.GetString());
        switchedRoots[original]=application->candidate;switchedTargets.insert(path);
    }
    for(const auto &path:switchProviders) {
        if(switchedTargets.count(path))continue;
        bool below=false;for(const auto &target:switchedTargets)below|=path.HasPrefix(target);
        if(!below)continue;
        const auto original=program.layout.providers.FindValue(RigExecProviderValueKey(path,"computePointFrame"));
        if(original==UINT64_MAX)continue;
        RigExecValueId output;size_t begin;
        if(!RigExecBindProviderContext(&program.layout.providers,program.layout.valueKeys,original,
            path,"switchedComposition",switchedRoots,&output,&begin,error))return false;
        program.layout.valueKeys=program.layout.providers.valueKeys;switchedComposition[path]=output;
        for(size_t opIndex=begin;opIndex<program.layout.providers.ops.size();++opIndex) {
            RigExecSceneProviderOp op;op.originalIndex=uint32_t(opIndex);op.output=program.layout.providers.ops[opIndex].output;
            op.reads=program.layout.providers.descriptors[opIndex].reads;
            if(!program.Append(program.layout.providers.descriptors[opIndex].key,std::move(op),error))return false;
        }
    }
    std::vector<RigExecSpaceParentContext> switchContexts,switchCarryContexts;
    if(!RigExecDeriveSpaceSwitchParentContexts(switchParents,switchComposed,switchSlots,switchSources,&switchContexts,&switchCarries,&switchCarryContexts))
        return Fail(error,"malformed space switch parent context");
    std::map<SdfPath,RigExecValueId> switchParentCheckpoints,switchCarryCheckpoints;
    for(int contextKind=0;contextKind<2;++contextKind) {
    for(size_t i=0;i<switchApplications.size();++i) {
        const auto &checkpoint=contextKind?switchCarryContexts[i]:switchContexts[i];if(checkpoint.anchor<0 && checkpoint.recompose.empty())continue;
        RigExecValueId value=UINT64_MAX;std::map<RigExecValueId,RigExecValueId> replacements;
        if(checkpoint.anchor>=0) {
            const auto anchor=switchProviders[size_t(checkpoint.anchor)];
            const auto original=program.layout.providers.FindValue(RigExecProviderValueKey(anchor,"computePointFrame"));
            const auto selected=switchedComposition.find(anchor);
            if(original==UINT64_MAX || selected==switchedComposition.end())return Fail(error,"missing switch context anchor: "+anchor.GetString());
            value=selected->second;replacements[original]=value;
        }
        if(!checkpoint.recompose.empty()) {
            const auto parent=switchProviders[size_t(checkpoint.recompose.back())];
            const auto original=program.layout.providers.FindValue(RigExecProviderValueKey(parent,"computePointFrame"));
            size_t begin;
            if(!RigExecBindProviderContext(&program.layout.providers,program.layout.valueKeys,original,
                switchApplications[i]->source->owner,contextKind?"switchCarry":"switchParent",replacements,&value,&begin,error))return false;
            program.layout.valueKeys=program.layout.providers.valueKeys;
            for(size_t opIndex=begin;opIndex<program.layout.providers.ops.size();++opIndex) {
                RigExecSceneProviderOp op;op.originalIndex=uint32_t(opIndex);op.output=program.layout.providers.ops[opIndex].output;
                op.reads=program.layout.providers.descriptors[opIndex].reads;
                if(!program.Append(program.layout.providers.descriptors[opIndex].key,std::move(op),error))return false;
            }
        }
        (contextKind?switchCarryCheckpoints:switchParentCheckpoints)[switchApplications[i]->source->owner]=value;
    }
    }
    std::map<RigExecValueId,RigExecValueId> compositionSeeds;
    for(const auto &[path,value]:switchedComposition) {
        auto chain=program.layout.routes.chains.find({path,RigExecSceneValueDomain::Pose});
        if(chain==program.layout.routes.chains.end())continue;
        const auto old=chain->second.base;compositionSeeds[old]=value;
        chain->second.base=value;if(chain->second.poseBase==old)chain->second.poseBase=value;
        auto &versions=chain->second.versions;
        versions.erase(std::remove_if(versions.begin(),versions.end(),[&](const auto &version){
            const auto writer=scene.nodes.find(version.writer);const bool switched=writer!=scene.nodes.end() && writer->second.domain==RigExecSceneDomain::SpaceSwitch;
            if(switched)compositionSeeds[version.value]=value;return switched;
        }),versions.end());
    }
    for(auto &application:applications)if(application.source->domain!=RigExecSceneDomain::SpaceSwitch) {
        const auto chain=program.layout.routes.chains.find({application.source->canonicalTarget,applicationDomain(*application.source)});
        if(chain==program.layout.routes.chains.end())return Fail(error,"missing application composed base");
        application.incoming=chain->second.base;
        for(const auto &version:chain->second.versions)if(version.ordinal<application.source->stackOrdinal)application.incoming=version.value;else break;
    }
    const auto bindCompositionSeed=[&](RigExecValueId *id){const auto found=compositionSeeds.find(*id);if(found!=compositionSeeds.end())*id=found->second;};
    for(size_t i=0;i<program.operations.size();++i) {

        if(auto *commit=std::get_if<RigExecPoseCommitBinding>(&program.operations[i])) {
            for(auto &candidate:commit->candidates)bindCompositionSeed(&candidate.before);
            for(auto &descendant:commit->descendants)bindCompositionSeed(&descendant.current);
            for(auto &read:commit->reads)bindCompositionSeed(&read);
            program.layout.descriptors[program.layout.initialProviderOps+program.layout.baseSelections.size()+i].reads=commit->reads;
        } else if(auto *extract=std::get_if<RigExecSceneExtractOp>(&program.operations[i])) {
            bindCompositionSeed(&extract->fallback);auto &descriptor=program.layout.descriptors[program.layout.initialProviderOps+program.layout.baseSelections.size()+i];descriptor.reads={extract->aggregate,extract->fallback};
        }
    }
    for(auto &[owner,commit]:commits) {
        const auto writer=scene.nodes.find(owner);if(writer!=scene.nodes.end() && writer->second.domain==RigExecSceneDomain::SpaceSwitch)continue;
        for(size_t i=0;i<commit.candidates.size();++i) {
            const auto application=std::find_if(applications.begin(),applications.end(),[&](const auto &entry){return entry.source->owner==owner && entry.candidate==commit.candidates[i].candidate;});
            if(application!=applications.end())commit.candidates[i].before=application->incoming;
        }
        for(auto &descendant:commit.descendants)bindCompositionSeed(&descendant.current);
        commit.reads.clear();for(const auto &candidate:commit.candidates){commit.reads.push_back(candidate.before);commit.reads.push_back(candidate.candidate);}
        for(const auto &descendant:commit.descendants)commit.reads.push_back(descendant.current);
    }
    for(const auto &[path,node]:scene.nodes) {
        if(node.domain==RigExecSceneDomain::Solver) {
            RigExecSceneSolverDescriptor source;RigExecSolverGraphBinding bound;
            if(!RigExecTranslateSceneSolver(scene,path,rests,&source,error) ||
               !RigExecBindSolverGraph(source,context,program.layout.solverAggregates.at(path),&bound,error) ||
               !program.Append("solver:"+path.GetString(),std::move(bound),error))return false;
        } else if(node.domain==RigExecSceneDomain::Weight) {
            RigExecSceneWeightProgram source;RigExecWeightGraphBinding bound;
            if(!RigExecTranslateSceneWeight(scene,path,&source,error) ||
               !RigExecBindWeightGraph(source,context,path,_tokens->base,&bound,error))return false;
            RigExecSceneWeightOp op;op.object=bound.root;op.output=program.layout.weightPackets.at(path);op.binding=std::move(bound);
            if(!program.Append("weightPacket:"+path.GetString(),std::move(op),error))return false;
        } else if(publishesWeights(node)) {
            RigExecScenePoseInterpolatorDescriptor source;RigExecBoundPoseInterpolator bound;
            if(!RigExecTranslateScenePoseInterpolator(scene,path,&source,error))return false;
            const auto &weights=interpolatorOutputs.at(path).first;
            const auto &disabled=interpolatorOutputs.at(path).second;
            if(!RigExecBindScenePoseInterpolator(source,context,weights,disabled,&bound,error,
                   interpolatorPrimvars.at(path)) ||
               !program.Append("poseInterpolator:"+path.GetString(),std::move(bound),error))return false;
        } else if(node.domain==RigExecSceneDomain::UnknownRigExec)return Fail(error,"unsupported RigExec scene domain "+node.fact.type.GetString()+": "+path.GetString());
    }
    auto bindField=[&](const SdfPath &consumer,const SdfPath &weightObject,const SdfPath &moverTarget,RigExecValueId points,const std::string &key,RigExecValueId *id) {
        RigExecSceneFieldOp op;op.points=points;op.output=program.layout.Allocate("field:"+key);
        RigExecSceneTypedRead weight;
        if(!RigExecBindSceneTypedRead(scene,consumer.AppendProperty(_tokens->inputsDefaultWeight),SdfValueTypeNames->Float,&weight,error) || !RigExecBindGraphTypedRead(weight,context,&op.defaultWeight,error))return false;
        if(!weightObject.IsEmpty()) {
            RigExecSceneWeightProgram source;
            if(!RigExecTranslateSceneWeight(scene,weightObject,&source,error))return false;
            for(const auto &object:source.objects) {
                const auto relation=scene.relationships.find(object.path.AppendProperty(_tokens->rigExecWeightTarget));
                if(relation==scene.relationships.end() || relation->second.fact.targets.size()!=1)
                    return Fail(error,object.path.GetString()+": rigExec:weightTarget must have exactly one target");
                auto declared=relation->second.fact.targets.front();
                if(points!=UINT64_MAX && declared.IsPrimPath()) {
                    const auto node=scene.nodes.find(declared);
                    if(node!=scene.nodes.end() && node->second.pointBased)declared=declared.AppendProperty(_tokens->points);
                }
                if(declared!=moverTarget)return Fail(error,object.path.GetString()+": rigExec:weightTarget does not match mover target "+moverTarget.GetString());
            }
            TfToken placement=_tokens->base;RigExecSceneCompileInputs facts(scene);
            if(const auto *relation=facts.Relationship(consumer.AppendProperty(_tokens->rigExecWeightObject)))placement=relation->readPhase;
            if(placement!="base" && placement!="final")op.staticError="weight placement must read base or final";
            if(op.staticError.empty() && !RigExecBindWeightGraph(source,context,consumer,placement,&op.binding,error))return false;
        }
        *id=op.output;return program.Append("field:"+key,std::move(op),error);
    };
    // Each switched provider's bound switch and its source paths, for the
    // IK/FK match's re-run of its pole's switch.
    std::map<SdfPath,std::pair<RigExecBoundSpaceSwitch,SdfPathVector>> boundSwitches;
    for(const auto &application:applications) {
        const auto &a=*application.source;const auto key="revision:"+a.owner.GetString()+":"+a.canonicalTarget.GetString();
        if(a.domain==RigExecSceneDomain::PropertyMover) {
            RigExecScenePropertyDescriptor source;RigExecPropertyGraphBinding bound;
            if(!RigExecTranslateSceneProperty(scene,a.owner,a.canonicalTarget,&source,error))return false;
            RigExecValueId field;if(!bindField(a.owner,source.weightObject,a.canonicalTarget,UINT64_MAX,key,&field))return false;
            if(!RigExecBindPropertyGraph(source,context,application.incoming,application.output,field,&bound,error) ||
               !program.Append(key,std::move(bound),error))return false;
        } else if(a.domain==RigExecSceneDomain::GeometryMover) {
            const auto source=geometryDescriptors.at({a.owner,a.canonicalTarget});RigExecSceneGeometryOp op;
            if(!RigExecBindGeometryGraph(source,context,&op.binding,error))return false;
            op.incoming=application.incoming;op.output=application.output;
            const auto points=source.pointsTarget.IsEmpty()?a.canonicalTarget:source.pointsTarget;
            const auto pointsChain=program.layout.routes.chains.find({points,RigExecSceneValueDomain::Points});
            if(pointsChain==program.layout.routes.chains.end())return Fail(error,"geometry source points have no typed chain: "+points.GetString());
            op.pointsBase=pointsChain->second.base;
            if(source.derived && !program.layout.routes.Resolve(scene,{points,RigExecSceneValueDomain::Points},a.owner,_tokens->final,&op.pointsFinal,error))return false;
            if(!source.derived && !bindField(a.owner,source.record.binding.weightObject,a.canonicalTarget,op.incoming,key,&op.field))return false;
            op.fieldPacket.representation=_tokens->dense;op.fieldPacket.rangePolicy=_tokens->strict;
            // A windowed mover reads its own inputs over earlier frames.
            if(const auto *handler=source.record.binding.handler;handler && handler->windowFramesScene)
                op.windowFrames=handler->windowFramesScene(scene,a.owner);
            if(!program.Append(key,std::move(op),error))return false;
        } else if(a.domain==RigExecSceneDomain::SpaceSwitch) {
            RigExecSceneSpaceSwitchDescriptor source;RigExecSceneSwitchOp op;
            if(!RigExecTranslateSceneSpaceSwitch(scene,a.owner,&source,error) ||
               !([&]() {
                   auto switchContext=context;
                   switchContext.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *why) {
                       if(request.computation=="computePointFrame") {
                           const auto selected=switchedComposition.find(request.source);
                           if(selected!=switchedComposition.end()){*id=selected->second;return true;}
                       }
                       return context.resolve(request,id,why);
                   };
                   const auto parent=switchParentCheckpoints.find(a.owner);
                   const auto carry=switchCarryCheckpoints.find(a.owner);
                   return RigExecBindSceneSpaceSwitch(scene,source,switchContext,&op.binding,error,
                       parent==switchParentCheckpoints.end()?UINT64_MAX:parent->second,
                       carry==switchCarryCheckpoints.end()?UINT64_MAX:carry->second);
               })())return false;
            boundSwitches[a.canonicalTarget]={op.binding,source.sources};
            op.output=application.candidate;if(!program.Append(key,std::move(op),error))return false;
        } else if(a.domain==RigExecSceneDomain::AutoClavicle) {
            RigExecBoundAutoClavicle op;
            if(!RigExecBindAutoClavicle(scene,a.owner,context,application.incoming,
                application.candidate,&op,error) || !program.Append(key,std::move(op),error))return false;
        } else if(a.domain==RigExecSceneDomain::Constraint) {
            RigExecSceneConstraintDescriptor source;RigExecConstraintGraphBinding bound;RigExecConstraintGraphTarget target;
            if(!RigExecTranslateSceneConstraint(scene,a.owner,{a.target},&source,error))return false;
            uint32_t windowFrames=0;
            if(source.record.kind==RigExecConstraintKind::Harmonics) {
                const auto targets=std::count_if(scene.applications.begin(),scene.applications.end(),
                    [&](const auto &entry){return entry.owner==a.owner && entry.domain==RigExecSceneDomain::Constraint;});
                if(targets!=1)return Fail(error,"harmonics constraint needs one pose-provider target: "+a.owner.GetString());
                RigExecHarmonicsSettings settings;
                settings.waves=source.harmonicsWaves;settings.waveLength=source.harmonicsWaveLength;
                settings.frequency=source.harmonicsFrequency;settings.ignoreFirstFrame=source.harmonicsIgnoreFirstFrame;
                if(!RigExecHarmonicsSettingsValid(settings))return Fail(error,"harmonics constraint window is invalid: "+a.owner.GetString());
                windowFrames=uint32_t(RigExecHarmonicsWindowFrames(settings));
            }
            target.incoming=application.incoming;target.output=application.candidate;
            RigExecSceneCompileInputs facts(scene);const auto weightTargets=facts.Targets(a.owner.AppendProperty(_tokens->rigExecWeightObject));
            if(!weightTargets.empty() && !bindField(a.owner,weightTargets.front(),a.target,UINT64_MAX,key,&target.weight))return false;
            if(!source.space.IsEmpty()) {
                RigExecSceneGraphReadRequest request;request.source=source.space;request.reader=a.owner;
                request.domain=RigExecSceneValueDomain::Pose;request.phase=_tokens->preceding;
                if(!context.resolve(request,&target.carry,error))return false;
            }
            if(source.record.kind==RigExecConstraintKind::SingleChainIk) {
                if(a.targetIndex!=0)continue;
                source=chainConstraints.at(a.owner);
                target.output=program.layout.Allocate("constraintStatus:"+a.owner.GetString());
                for(const auto &joint:source.ikChain) {
                    RigExecSceneGraphReadRequest request;request.source=joint;request.reader=a.owner;request.domain=RigExecSceneValueDomain::Pose;request.computation="computeRestFrame";
                    RigExecValueId rest;if(!context.resolve(request,&rest,error))return false;target.chainRests.push_back(rest);target.restLive.push_back(extracted.count(joint)!=0);
                    const auto match=std::find_if(applications.begin(),applications.end(),[&](const auto &entry){return entry.source->owner==a.owner && entry.source->target==joint;});
                    if(match==applications.end())return Fail(error,"missing IK chain commit version");target.chainOutputs.push_back(match->candidate);
                }
            }
            if(!RigExecBindConstraintGraph(scene,source,context,target,&bound,error))return false;
            bound.windowFrames=windowFrames;
            if(!program.Append(key,std::move(bound),error))return false;
        } else return Fail(error,"unsupported scene application: "+a.owner.GetString());
    }
    // RigExecIkFkMatchConstraint: the compile's checks (rigEvaluatorCompile),
    // then every read RigExecBakedRunIkFkMatch makes, bound to explicit
    // versions. Its presentation reads each control's published frame, which
    // is an earlier match's presentation when one revised it.
    for(const auto &path:ikFkMatches) {
        RigExecSceneConstraintDescriptor source;
        if(!RigExecTranslateSceneConstraint(scene,path,{},&source,error))return false;
        const auto &facts=source.match;const auto name="RigExecIkFkMatchConstraint "+path.GetString();
        const auto one=[&](const SdfPathVector &rows,const char *relationship) {
            return (rows.size()==1 && rows[0].IsPrimPath()) || Fail(error,name+" requires exactly one "+relationship);
        };
        if(!one(facts.ikControl,"rigExec:ikControl") || !one(facts.poleControl,"rigExec:poleControl") ||
           !one(facts.rootControl,"rigExec:rootControl") || !one(facts.effector,"rigExec:effector"))return false;
        if(facts.fkControls.size()!=3 || facts.limbJoints.size()!=3)
            return Fail(error,name+" requires three FK controls and three joints");
        const SdfPath controls[5]={facts.ikControl[0],facts.poleControl[0],facts.fkControls[0],facts.fkControls[1],facts.fkControls[2]};
        for(const auto &control:controls) {
            const auto node=scene.nodes.find(control);
            if(node==scene.nodes.end() || node->second.fact.type!="RigExecControl")
                return Fail(error,name+" revises "+control.GetString()+", which is not a RigExecControl");
        }
        if(std::set<SdfPath>(facts.moves.begin(),facts.moves.end())!=std::set<SdfPath>(std::begin(controls),std::end(controls)))
            return Fail(error,name+" rigExec:moves must name exactly its IK control, pole and FK controls");
        // Joints 0-2, IK root, effector, then the five controls; each read at
        // the phase its relationship declares.
        const SdfPath sourcePaths[10]={facts.limbJoints[0],facts.limbJoints[1],facts.limbJoints[2],facts.rootControl[0],
            facts.effector[0],controls[0],controls[1],controls[2],controls[3],controls[4]};
        const TfToken *relationships[10]={&_tokens->rigExecLimbJoints,&_tokens->rigExecLimbJoints,&_tokens->rigExecLimbJoints,
            &_tokens->rigExecRootControl,&_tokens->rigExecEffector,&_tokens->rigExecIkControl,&_tokens->rigExecPoleControl,
            &_tokens->rigExecFkControls,&_tokens->rigExecFkControls,&_tokens->rigExecFkControls};
        for(const auto &read:sourcePaths) {
            const auto node=scene.nodes.find(read);
            if(node==scene.nodes.end() || !node->second.fact.active || node->second.domain!=RigExecSceneDomain::Provider)
                return Fail(error,name+" reads "+read.GetString()+", which is not a RigExec transform provider");
        }
        if(facts.space.size()>1)return Fail(error,name+" has more than one rigExec:space");
        auto match=std::make_shared<RigExecIkFkMatchGraphBinding>();
        RigExecConstraintGraphBinding bound;bound.structural=source;
        bound.structural.inputs.clear();bound.structural.typedReads.clear();
        if(!RigExecBindGraphInputs(source.inputs,source.typedReads,path,context,&bound.inputs,&bound.reads,error))return false;
        const RigExecSceneCompileInputs compileInputs(scene);
        for(size_t i=0;i<10;++i) {
            RigExecSceneGraphReadRequest request;request.consumer=path.AppendProperty(*relationships[i]);
            request.source=sourcePaths[i];request.reader=path;request.domain=RigExecSceneValueDomain::Pose;
            const auto *relation=compileInputs.Relationship(request.consumer);
            request.phase=relation?relation->readPhase:RigExecSceneBasePhase;
            if(!context.resolve(request,&match->sources[i],error))return false;
            bound.reads.push_back(match->sources[i]);
        }
        const auto restOf=[&](const SdfPath &provider,RigExecValueId *id) {
            RigExecSceneGraphReadRequest request;request.source=provider;request.reader=path;request.consumer=path;
            request.domain=RigExecSceneValueDomain::Pose;request.computation="computeRestFrame";
            if(!context.resolve(request,id,error))return false;
            bound.reads.push_back(*id);return true;
        };
        for(size_t i=0;i<3;++i)if(!restOf(sourcePaths[i],&match->rests[i]))return false;
        for(size_t k=0;k<5;++k)if(!restOf(controls[k],&match->rests[3+k]))return false;
        // The IK solve's space: its posed frame is the carry a constraint's
        // rigExec:space reads, and its rest frame measures the rest lengths.
        if(!source.space.IsEmpty()) {
            RigExecSceneGraphReadRequest request;request.source=source.space;request.reader=path;
            request.domain=RigExecSceneValueDomain::Pose;request.phase=_tokens->preceding;
            if(!context.resolve(request,&bound.target.carry,error) || !restOf(source.space,&match->spaceRest))return false;
            bound.reads.push_back(bound.target.carry);
        }
        // The controls' channels and the pole's posed:space are the inputs
        // the providers compose them from.
        const auto &providers=program.layout.providers;
        const auto original=[&](const SdfPath &owner,RigExecProviderOpKind kind)->const RigExecProviderOp * {
            for(size_t i=0;i<program.layout.initialProviderOps && i<providers.ops.size();++i)
                if(providers.ops[i].kind==kind && providers.ops[i].owner==owner)return &providers.ops[i];
            return nullptr;
        };
        for(size_t k=0;k<5;++k) {
            const auto *op=original(controls[k],RigExecProviderOpKind::AvarMatrix);
            if(!op || op->inputs.size()<13)return Fail(error,name+" control "+controls[k].GetString()+" has no composed channels");
            std::copy(op->inputs.begin(),op->inputs.begin()+13,match->channels[k].begin());
            match->scaleAvars[k]=op->scaleAvars?1:0;
            for(const auto id:match->channels[k])if(id!=UINT64_MAX)bound.reads.push_back(id);
        }
        if(const auto *posed=original(controls[1],RigExecProviderOpKind::PosedFrame);posed && !posed->inputs.empty()) {
            match->polePosedSpace=posed->inputs[0];
            if(match->polePosedSpace!=UINT64_MAX)bound.reads.push_back(match->polePosedSpace);
        }
        // The pole's space switch, and which of its sources ride the IK
        // control; without one, whether its namespace parent does.
        if(const auto switched=boundSwitches.find(controls[1]);switched!=boundSwitches.end()) {
            match->poleSwitch=true;match->poleSwitchBinding=switched->second.first;
            for(const auto &switchSource:switched->second.second)
                match->poleSourceRides.push_back(!switchSource.IsEmpty() && switchSource.HasPrefix(controls[0])?1:0);
            bound.reads.insert(bound.reads.end(),match->poleSwitchBinding.reads.begin(),match->poleSwitchBinding.reads.end());
        }
        for(auto parent=controls[1].GetParentPath();!parent.IsEmpty() && !parent.IsAbsoluteRootPath();parent=parent.GetParentPath()) {
            const auto node=scene.nodes.find(parent);
            if(node==scene.nodes.end() || !node->second.fact.active || node->second.domain!=RigExecSceneDomain::Provider)continue;
            match->poleParentRides=parent.HasPrefix(controls[0]);break;
        }
        // The five controls and every other control under one, riding the
        // nearest of them above it.
        auto &presented=program.presentedPoses;
        const auto publishedOf=[&](const SdfPath &control,RigExecValueId *id) {
            const auto earlier=presented.find(control);
            if(earlier!=presented.end()){*id=earlier->second;return true;}
            return program.layout.routes.Resolve(scene,{control,RigExecSceneValueDomain::Pose},path,_tokens->final,id,error);
        };
        std::vector<SdfPath> riderPaths;
        for(size_t k=0;k<5;++k)if(!publishedOf(controls[k],&match->published[k]))return false;
        for(const auto &[controlPath,node]:scene.nodes) {
            if(!node.fact.active || node.fact.type!="RigExecControl" ||
               !program.layout.routes.chains.count({controlPath,RigExecSceneValueDomain::Pose}))continue;
            int rides=-1;size_t depth=0;
            for(size_t k=0;k<5;++k) {
                if(controlPath==controls[k]){rides=-1;break;}
                if(controlPath.HasPrefix(controls[k]) && controls[k].GetPathElementCount()>depth)
                    {rides=int(k);depth=controls[k].GetPathElementCount();}
            }
            if(rides<0)continue;
            RigExecIkFkMatchGraphBinding::Rider rider;rider.rides=rides;
            if(!publishedOf(controlPath,&rider.published))return false;
            match->riders.push_back(rider);riderPaths.push_back(controlPath);
        }
        for(const auto id:match->published)bound.reads.push_back(id);
        for(const auto &rider:match->riders)bound.reads.push_back(rider.published);
        match->ikValue=facts.ikValue;match->scaleCalibration=facts.scaleCalibration;
        match->upperLengthOffset=facts.upperLengthOffset;match->lowerLengthOffset=facts.lowerLengthOffset;
        match->stretchLimb=facts.stretchLimb;
        match->unreadable=path.GetString()+" could not read its limb; constraint passed through";
        match->unmatched=path.GetString()+" could not match its limb's other half";
        // Outputs, allocated once every read is resolved.
        const auto key="ikFkMatch:"+path.GetString();
        match->ikValues=program.layout.Allocate(key+":ikValues");match->fkValues=program.layout.Allocate(key+":fkValues");
        program.publicValues[path.AppendProperty(_tokens->outputsIkValues)]=match->ikValues;
        program.publicValues[path.AppendProperty(_tokens->outputsFkValues)]=match->fkValues;
        bound.target.output=match->ikValues;bound.writes.push_back(match->fkValues);
        for(size_t k=0;k<5;++k) {
            match->presented[k]=program.layout.Allocate("presented:"+path.GetString()+":"+controls[k].GetString());
            bound.writes.push_back(match->presented[k]);presented[controls[k]]=match->presented[k];
        }
        for(size_t r=0;r<match->riders.size();++r) {
            auto &rider=match->riders[r];
            rider.presented=program.layout.Allocate("presented:"+path.GetString()+":"+riderPaths[r].GetString());
            bound.writes.push_back(rider.presented);presented[riderPaths[r]]=rider.presented;
        }
        std::sort(bound.reads.begin(),bound.reads.end());
        bound.reads.erase(std::unique(bound.reads.begin(),bound.reads.end()),bound.reads.end());
        bound.reads.erase(std::remove(bound.reads.begin(),bound.reads.end(),UINT64_MAX),bound.reads.end());
        bound.match=std::move(match);
        if(!program.Append(key,std::move(bound),error))return false;
    }
    for(auto &[owner,commit]:commits) {
        const auto writer=scene.nodes.find(owner);
        if(writer!=scene.nodes.end() && writer->second.domain==RigExecSceneDomain::SpaceSwitch)continue;
        if(!program.Append("commit:"+owner.GetString(),std::move(commit),error))return false;
    }
    if(!RigExecRebindProviderInputs(&program.layout.providers,program.layout.initialProviderOps,
        [&](const SdfPath &attribute,RigExecValueId original,RigExecValueId *bound,std::string *why) {
            const auto found=scene.attributes.find(attribute);
            if(found==scene.attributes.end()){*bound=original;return true;}
            const auto &fact=found->second;
            if(fact.fact.type==SdfValueTypeNames->Matrix4d) {
                if(fact.fact.connections.size()!=1){*bound=original;return true;}
                RigExecSceneGraphReadRequest request;request.consumer=attribute;request.source=attribute;
                request.reader=attribute;request.phase=fact.readPhase;request.domain=RigExecSceneValueDomain::ProviderSpace;
                if(!context.resolve(request,bound,why))return false;
            } else *bound=original;
            program.layout.providers.valueKeys=program.layout.valueKeys;
            return true;
        },error))return false;
    for(size_t i=0;i<program.layout.initialProviderOps;++i)
        program.layout.descriptors[i]=program.layout.providers.descriptors[i];
    const auto routedInputs=program.layout.providers.routedInputs;
    for(const auto &routed:routedInputs) {
        RigExecSceneGraphReadRequest request;request.consumer=routed.consumer;request.source=routed.source;
        request.reader=routed.consumer;request.phase=routed.readPhase;
        request.domain=routed.source.GetNameToken()=="points"?RigExecSceneValueDomain::Points:RigExecSceneValueDomain::Property;
        RigExecSceneCopyOp op;op.output=routed.value;
        if(!context.resolve(request,&op.input,error))return false;
        auto &leaves=program.layout.leaves;leaves.erase(std::remove(leaves.begin(),leaves.end(),op.output),leaves.end());
        if(!program.Append("route:"+routed.consumer.GetString(),op,error))return false;
    }
    for(const auto &[path,node]:scene.nodes)if(node.fact.type=="RigExecBlendInput") {
        RigExecSceneGeometryDescriptor descriptor;RigExecSceneBlendChannelOp op;
        if(!RigExecTranslateSceneBlendChannel(scene,path,&descriptor,error) ||
           !RigExecBindGeometryGraph(descriptor,context,&op.binding,error))return false;
        const auto key=RigExecProviderValueKey(path,"computeBlendChannel");
        op.output=program.layout.Allocate(key);program.layout.providers.valueIds[key]=op.output;
        if(!program.Append(key,std::move(op),error))return false;
    }
    for(const auto &path:scene.compilerTargets)if(path.IsPropertyPath() && !program.publicValues.count(path)) {
        const auto attribute=scene.attributes.find(path);if(attribute==scene.attributes.end())return Fail(error,"missing public attribute: "+path.GetString());
        RigExecSceneTypedRead source;RigExecGraphTypedRead read;
        if(!RigExecBindSceneTypedRead(scene,path,attribute->second.fact.type,&source,error) ||
           !RigExecBindGraphTypedRead(source,context,&read,error))return false;
        program.publicValues[path]=read.effective;
    }
    if(!program.Compile(RigExecCyclePolicy::SetAside,&proof,error))return false;
    *destination=std::move(program);return true;
}

bool RigExecTranslateSceneProgram(const RigExecSceneDescriptors &captured,
    RigExecSceneProgram *destination,std::string *error) {
    if(!destination)return Fail(error,"null scene compiler output");
    std::set<SdfPath> excludedPoseWriters,excludedSolverBodies;
    RigExecOpExclusionProof proof;
    std::map<std::string,SdfPath> removableOwners;
    std::map<std::string,SdfPath> removableGeometryDrivers;
    // Each retry adds a previously unseen solver-body or pose-writer role.
    // Both sets are bounded by authored nodes; candidate tables stay local.
    if(captured.nodes.size()>(std::numeric_limits<size_t>::max()-1)/2)
        return Fail(error,"too many scene normalization roles");
    const size_t roleBudget=2*captured.nodes.size();
    for(size_t attempt=0;attempt<=roleBudget;++attempt) {
        RigExecSceneProgram candidate;
        if(!TranslateSceneProgramOnce(captured,excludedPoseWriters,excludedSolverBodies,proof,&candidate,error))return false;
        bool changed=false;
        const size_t first=candidate.layout.initialProviderOps+candidate.layout.baseSelections.size();
        for(size_t i=0;i<candidate.operations.size();++i) {
            const size_t descriptor=first+i;
            if(descriptor>=candidate.graph.canonicalIndex.size())return Fail(error,"scene exclusion descriptor mismatch");
            if(candidate.graph.canonicalIndex[descriptor]>=0)continue;
            const auto &operation=candidate.operations[i];
            const auto &key=candidate.layout.descriptors[descriptor].key;
            const char *prefix=nullptr;
            if(std::holds_alternative<RigExecSolverGraphBinding>(operation)) {
                prefix="solver:";
                if(key.compare(0,7,prefix)!=0)return Fail(error,"excluded solver has no stable owner key");
                const SdfPath owner(key.substr(7));
                const auto node=captured.nodes.find(owner);
                if(!owner.IsPrimPath() || node==captured.nodes.end() || node->second.domain!=RigExecSceneDomain::Solver)
                    return Fail(error,"excluded solver owner is missing or ambiguous");
                changed|=excludedSolverBodies.insert(owner).second;
            } else if(std::holds_alternative<RigExecPoseCommitBinding>(operation)) {
                const bool solver=key.compare(0,13,"solverCommit:")==0;
                prefix=solver?"solverCommit:":"commit:";
                const size_t length=solver?13:7;
                if(key.compare(0,length,prefix)!=0)return Fail(error,"excluded commit has no stable owner key");
                const SdfPath owner(key.substr(length));
                const auto node=captured.nodes.find(owner);
                if(!owner.IsPrimPath() || node==captured.nodes.end() ||
                   (solver?node->second.domain!=RigExecSceneDomain::Solver:
                    (node->second.domain!=RigExecSceneDomain::Constraint && node->second.domain!=RigExecSceneDomain::AutoClavicle)))
                    return Fail(error,"excluded commit owner is missing or ambiguous");
                const bool bound=solver?
                    std::any_of(captured.jointBindings.begin(),captured.jointBindings.end(),[&](const auto &binding){return binding.solver==owner;}):
                    std::any_of(captured.applications.begin(),captured.applications.end(),[&](const auto &application){return application.owner==owner;});
                if(!bound && (solver || node->second.fact.type!="RigExecSingleChainIkConstraint"))
                    return Fail(error,"excluded commit has no authored writer bindings");
                changed|=excludedPoseWriters.insert(owner).second;
            }
        }
        // Capture ownership from the actual emitted role, never a substring
        // match or a numeric slot that could belong to a replacement compiled rig.
        for(size_t i=0;i<candidate.operations.size();++i) {
            const auto &operation=candidate.operations[i];
            const auto &key=candidate.layout.descriptors[first+i].key;
            if(std::holds_alternative<RigExecPoseCommitBinding>(operation)) {
                const size_t length=key.compare(0,13,"solverCommit:")==0?13:7;
                const SdfPath owner(key.substr(length));
                if(!owner.IsPrimPath())return Fail(error,"commit removal owner is malformed");
                removableOwners.emplace(key,owner);
            } else if(std::holds_alternative<RigExecSceneExtractOp>(operation)) {
                const auto *extract=std::get_if<RigExecSceneExtractOp>(&operation);
                for(const auto &binding:captured.jointBindings)
                    if(key=="solverJoint:"+binding.solver.GetString()+":"+binding.joint.GetString() && binding.element==extract->element) {
                        const auto inserted=removableOwners.emplace(key,binding.solver);
                        if(!inserted.second && inserted.first->second!=binding.solver)
                            return Fail(error,"solver joint removal owner is ambiguous");
                    }
            } else if(std::holds_alternative<RigExecConstraintGraphBinding>(operation) ||
                      std::holds_alternative<RigExecSceneSwitchOp>(operation) ||
                      std::holds_alternative<RigExecBoundAutoClavicle>(operation) ||
                      std::holds_alternative<RigExecSceneFieldOp>(operation)) {
                for(const auto &application:captured.applications)if(application.domain==RigExecSceneDomain::Constraint || application.domain==RigExecSceneDomain::AutoClavicle) {
                    const auto revision="revision:"+application.owner.GetString()+":"+application.canonicalTarget.GetString();
                    if(key==revision || key=="field:"+revision)removableOwners.emplace(key,application.owner);
                }
                // Single-chain IK applications are generated from retained
                // authored chain facts rather than a moves application row.
                for(const auto &[owner,node]:captured.nodes)if(node.fact.type=="RigExecSingleChainIkConstraint") {
                    const auto prefix=(std::holds_alternative<RigExecSceneFieldOp>(operation)?
                        "field:revision:":"revision:")+owner.GetString()+":";
                    if(key.compare(0,prefix.size(),prefix)==0 && SdfPath(key.substr(prefix.size())).IsPrimPath())
                        removableOwners.emplace(key,owner);
                }
            }
        }
        // Only actual typed geometry bodies and their emitted field helpers
        // may be withdrawn with a disabled required aggregate. Remember exact
        // keys from the original candidate, never owner-substring guesses.
        for(size_t i=0;i<candidate.operations.size();++i) {
            const auto *geometry=std::get_if<RigExecSceneGeometryOp>(&candidate.operations[i]);
            if(!geometry)continue;
            const auto &source=geometry->binding.descriptor;
            const auto &driver=source.record.binding.driverFrames;
            if(driver.IsEmpty())continue;
            const auto &key=candidate.layout.descriptors[first+i].key;
            if(key!="revision:"+source.mover.GetString()+":"+source.target.GetString())
                return Fail(error,"geometry withdrawal owner key is inconsistent");
            removableGeometryDrivers.emplace(key,driver);
            const auto fieldKey="field:"+key;
            for(size_t field=0;field<candidate.operations.size();++field)
                if(candidate.layout.descriptors[first+field].key==fieldKey &&
                   std::holds_alternative<RigExecSceneFieldOp>(candidate.operations[field]))
                    removableGeometryDrivers.emplace(fieldKey,driver);
        }
        proof.cycles=candidate.graph.cycles;proof.cycleMembers=candidate.graph.cycleMembers;
        std::set<std::string> members;
        for(const auto &component:proof.cycleMembers)members.insert(component.begin(),component.end());
        proof.keys.clear();proof.removedKeys.clear();
        for(const auto &key:members) {
            const auto owner=removableOwners.find(key);
            const auto geometryDriver=removableGeometryDrivers.find(key);
            if((owner!=removableOwners.end() && excludedPoseWriters.count(owner->second)) ||
               (geometryDriver!=removableGeometryDrivers.end() && excludedSolverBodies.count(geometryDriver->second)))
                proof.removedKeys.push_back(key);
            else proof.keys.push_back(key);
        }
        if(!changed){candidate.retainedExclusions=proof;*destination=std::move(candidate);return true;}
    }
    return Fail(error,"scene pose writer normalization did not converge");
}
}
