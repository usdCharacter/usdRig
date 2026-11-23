// Compile steps: space switches, solver batches and the commit.

#include "rigExecSchema/schemaNames.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/internal/rigEvaluatorCompileAttempt.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/inputReplay.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExecScene/poseSceneTranslation.h"
#include "rigExecScene/startFrameDerivation.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/rigEvaluatorDependencies.h"
#include "rigExec/internal/rigEvaluatorPropertyBindings.h"
#include "rigExec/internal/rigEvaluatorConstraints.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/pyLock.h"
#include "pxr/base/tf/scoped.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <unordered_map>

// The 2D skeleton and space-switch names the compile reads, interned once.
TF_DEFINE_PRIVATE_TOKENS(_skeleton2dCompileTokens,
    (RigExecSkeleton2d) (RigExecSpaceSwitch)
    ((joints, rigExec::SchemaAttrText::rigExecJoints))
    ((target, rigExec::SchemaAttrText::rigExecTarget))
    ((sources, rigExec::SchemaAttrText::rigExecSources))
);

namespace rigExec {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((rigExecJoints, SchemaAttrText::rigExecJoints))
    ((rigExecBlendPointFrames, SchemaTypeName::RigExecBlendPointFrames))
    ((rigExecInputA, SchemaAttrText::rigExecInputA))
    ((rigExecInputB, SchemaAttrText::rigExecInputB))
    ((inputsValue, SchemaAttrText::inputsValue))
    ((inputsMin, SchemaAttrText::inputsMin))
    ((inputsMax, SchemaAttrText::inputsMax))
);

using namespace evaluatorDetail;

bool
RigExecRigEvaluator::_CompileSpaceSwitches(_CompileAttempt &attempt)
{
    auto &compileScene = attempt.compileScene;
    auto &compileSceneStage = attempt.compileSceneStage;
    auto &compileSceneSerial = attempt.compileSceneSerial;
    auto &compileSceneReusable = attempt.compileSceneReusable;
    auto &skeleton2dBones = attempt.skeleton2dBones;
    // RigExecSpaceSwitch: a labelled parent-space list for one xformable.
    //
    // Detached switch records provide structure; every current frame read is
    // bound by the common graph after provider closure and SSA assignment.
    auto &newSpaceSwitches = attempt.newSpaceSwitches;
    if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
        std::set<SdfPath> targets;
        for (const UsdPrim &prim : UsdPrimRange(rig)) {
            if (prim.GetTypeName() != SchemaTypeName::RigExecSpaceSwitch)
                continue;
            const SdfPath path = prim.GetPath();
            // A switch on a 2D skeleton's bone is the skeleton's to read.
            {
                SdfPathVector switched;
                prim.GetRelationship(_skeleton2dCompileTokens->target)
                    .GetTargets(&switched);
                if (switched.size() == 1 && skeleton2dBones.count(switched[0]))
                    continue;
            }
            const bool defaultCapture =
                compileScene && compileScene->rigRoot == _rigPath &&
                compileScene->identities.size() == 1 &&
                compileScene->identities[0] == UsdTimeCode::Default();
            if (!defaultCapture || !compileSceneReusable ||
                compileSceneStage != UsdStageWeakPtr(_stage) ||
                compileSceneSerial != GetStageEditSerial()) {
                compileScene = std::make_unique<RigExecSceneDescriptors>();
                const uint64_t before = GetStageEditSerial();
                std::string error;
                if (!RigExecDispatchCaptureSceneDescriptors(
                        _stage, _rigPath, {UsdTimeCode::Default()},
                        compileScene.get(), &error))
                    return attempt.Fail(error);
                // A notice during capture makes the snapshot ineligible for
                // later reuse; the switch translation still runs here.
                compileSceneSerial = before;
                compileSceneReusable = before == GetStageEditSerial();
            }
            auto captured =
                std::make_shared<RigExecSceneSpaceSwitchDescriptor>();
            std::string error;
            if (!RigExecDispatchTranslateSceneSpaceSwitch(
                    *compileScene, path, captured.get(), &error))
                return attempt.Fail(error);
            if (!targets.insert(captured->target).second)
                return attempt.Fail(
                    captured->target.GetString() +
                    " is the target of more than one space switch");
            _SpaceSwitch record;
            record.descriptor = captured;
            record.switchPath = path;
            record.target = captured->target;
            record.spacePath = captured->space;
            _SeedProvider(attempt, record.target);
            if (!record.spacePath.IsEmpty())
                _SeedProvider(attempt, record.spacePath);
            for (const auto &path : captured->sources) {
                _SpaceSwitch::Source source;
                source.path = path;
                if (!path.IsEmpty())
                    _SeedProvider(attempt, path);
                record.sources.push_back(std::move(source));
            }
            newSpaceSwitches.push_back(std::move(record));
        }
    }
    return true;
}

bool
RigExecRigEvaluator::_CompileProviderClosure(_CompileAttempt &attempt)
{
    auto &newNativeProviderPaths = attempt.newNativeProviderPaths;
    auto &newRestCompiledRigProviders = attempt.newRestCompiledRigProviders;
    auto &restLatePoseInputs = attempt.restLatePoseInputs;
    // Warming the closure over the seeded set is what grows newPoseInputInfo,
    // which the pass after this one iterates -- so the two are timed apart:
    // this one is stage walking, that one is exec prepares.
    std::vector<SdfPath> seedPaths;
    for (const SdfPath &provider : newNativeProviderPaths)
        seedPaths.push_back(provider);
    for (const SdfPath &provider : seedPaths)
        for (const SdfPath &member : _PoseProviderClosure(attempt, provider))
            _SeedProvider(attempt, member, false);
    // Close exactly the first-frame/rest provider roots, separately
    // from native geometry-only source roots. This adds no operation or tap.
    const std::vector<SdfPath> restSeedPaths(
        newRestCompiledRigProviders.begin(), newRestCompiledRigProviders.end());
    for (const SdfPath &provider : restSeedPaths) {
        const auto &closure = _PoseProviderClosure(attempt, provider);
        restLatePoseInputs.insert(closure.begin(), closure.end());
    }
    // Seed reached connected inputs and ancestors after explicit roots;
    // close once.
    for (const SdfPath &input : restLatePoseInputs)
        _SeedRestCompiledRigProvider(attempt, input);
    // The first-frame/rest request admits only valid USD providers.
    // Geometry-only inputs are not roots
    // of this request and retain their independent availability policy.
    for (const SdfPath &path : newRestCompiledRigProviders) {
        const UsdPrim provider = _stage->GetPrimAtPath(path);
        if (!provider || !UsdPrimDefaultPredicate(provider)) {
            return attempt.Fail("failed to prepare pose provider inputs");
        }
    }
    return true;
}

bool
RigExecRigEvaluator::_CompileSolverBatches(_CompileAttempt &attempt)
{
    std::vector<std::string> *errors = attempt.errors;
    auto &newJointBinding = attempt.newJointBinding;
    auto &newSolverDependencies = attempt.newSolverDependencies;
    auto &newSolverAggregateReads = attempt.newSolverAggregateReads;
    auto &newFrameConstraints = attempt.newFrameConstraints;
    auto &newFrameChains = attempt.newFrameChains;
    auto &poseStackOrdinal = attempt.poseStackOrdinal;
    auto &newGraphChains = attempt.newGraphChains;
    auto &newGraphDerivedChains = attempt.newGraphDerivedChains;
    auto &solverFrameInputs = attempt.solverFrameInputs;
    auto &newSolverJoints = attempt.newSolverJoints;
    auto &requiredSolvers = attempt.requiredSolvers;
    auto &poseDependencies = attempt.poseDependencies;
    auto &constraintIndices = attempt.constraintIndices;
    auto &solverRestPredecessor = attempt.solverRestPredecessor;
    using NativePoseOwner = _CompileAttempt::NativePoseOwner;
    auto &nativePoseOwners = attempt.nativePoseOwners;
    auto &newSolverBatches = attempt.newSolverBatches;
    // Capture native domain descriptors in authored stack order. Scheduling
    // belongs to the single graph built after every domain is translated.
    std::vector<SdfPath> authoredPose;
    for (const auto &entry : poseDependencies)
        authoredPose.push_back(entry.first);
    std::sort(authoredPose.begin(), authoredPose.end(),
              [&](const SdfPath &a, const SdfPath &b) {
                  const int oa = attempt.StackOrdinalOf(a),
                            ob = attempt.StackOrdinalOf(b);
                  return oa != ob ? oa < ob : a < b;
              });
    for (const auto &path : authoredPose) {
        const auto constraint = constraintIndices.find(path);
        if (constraint != constraintIndices.end()) {
            nativePoseOwners.push_back({false, constraint->second});
            continue;
        }
        if (!requiredSolvers.count(path))
            continue;
        _SolverBatch batch;
        const auto &dependencies = newSolverDependencies[path];
        batch.dependencies.insert(dependencies.begin(), dependencies.end());
        const auto &frames = solverFrameInputs[path];
        batch.frameInputs.insert(frames.begin(), frames.end());
        SdfPathVector named;
        if (const auto prim = _stage->GetPrimAtPath(path))
            if (const auto relationship =
                    prim.GetRelationship(_tokens->rigExecJoints))
                relationship.GetTargets(&named);
        const auto predecessors = solverRestPredecessor.find(path);
        if (predecessors != solverRestPredecessor.end() &&
            !predecessors->second.empty())
            for (const auto &joint : named) {
                const auto previous =
                    predecessors->second.find(joint.GetPrimPath());
                batch.restInputs[joint.GetPrimPath()] =
                    previous == predecessors->second.end() ? SdfPath()
                                                           : previous->second;
            }
        const size_t index = newSolverBatches.size();
        batch.solvers.insert(path);
        // This concerns the blend's numerical aggregate inputs. A guide-only
        // relationship naming a solver is not an aggregate read, and an
        // unpositioned producer (-1) cannot have run in a later stack batch.
        const auto consumerPrim = _stage->GetPrimAtPath(path);
        const bool readsAggregates =
            consumerPrim &&
            consumerPrim.GetTypeName() == _tokens->rigExecBlendPointFrames;
        std::set<SdfPath> numericalAggregates;
        if (readsAggregates)
            for (const TfToken &name :
                 {_tokens->rigExecInputA, _tokens->rigExecInputB}) {
                SdfPathVector inputs;
                const auto relationship = consumerPrim.GetRelationship(name);
                if (relationship)
                    relationship.GetTargets(&inputs);
                if (!inputs.empty())
                    numericalAggregates.insert(inputs.front().GetPrimPath());
            }
        const auto aggregates = newSolverAggregateReads.find(path);
        if (readsAggregates && poseStackOrdinal.count(path) &&
            aggregates != newSolverAggregateReads.end())
            for (const auto &source : aggregates->second)
                if (numericalAggregates.count(source) &&
                    poseStackOrdinal.count(source) &&
                    attempt.StackOrdinalOf(source) >=
                        attempt.StackOrdinalOf(path)) {
                    const std::string warning =
                        path.GetString() +
                        " formerly read the loop-carried aggregate of " +
                        source.GetString() +
                        "; the native graph waits for its same-frame producer";
                    if (errors)
                        errors->push_back("warning: " + warning);
                    TF_WARN("%s", warning.c_str());
                }
        nativePoseOwners.push_back({true, index});
        newSolverBatches.push_back(std::move(batch));
    }

    // A driver-frames revision requires an actually admitted aggregate
    // batch, not merely a prim of solver type. Refuse only its authored
    // owner.
    std::set<SdfPath> admittedAggregates;
    for (const _SolverBatch &batch : newSolverBatches)
        admittedAggregates.insert(batch.solvers.begin(), batch.solvers.end());
    const auto admitDriverFrames = [&](const auto &chains) {
        for (const auto &[target, revisions] : chains)
            for (const _GraphRevision &revision : revisions) {
                const SdfPath &driver = revision.binding.driverFrames;
                if (!driver.IsEmpty() && !admittedAggregates.count(driver))
                    return attempt.Fail(
                        "driver frames " + driver.GetString() +
                            " is not a solver of this rig; mover set aside",
                        {revision.moverPath});
            }
        return true;
    };
    if (!admitDriverFrames(newGraphChains) ||
        !admitDriverFrames(newGraphDerivedChains))
        return false;

    // Authored stack order defines the provider value checkpoints. The common
    // graph later schedules the operations that produce these identities.
    if (!newJointBinding.empty()) {
        std::map<SdfPath, std::vector<SdfPath>> walkChains;
        std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> walkWriters;
        for (const NativePoseOwner &step : nativePoseOwners) {
            if (step.solverBatch) {
                for (const SdfPath &solver :
                     newSolverBatches[step.index].solvers) {
                    const auto joints = newSolverJoints.find(solver);
                    if (joints == newSolverJoints.end()) {
                        continue;
                    }
                    for (const auto &[joint, element] : joints->second) {
                        walkChains[joint].push_back(solver);
                        walkWriters[joint].emplace_back(solver, element);
                    }
                }
                continue;
            }
            const _FrameConstraint &constraint =
                newFrameConstraints[step.index];
            if (!constraint.pointsTarget.IsEmpty()) {
                continue; // geometry domain: not a frame writer
            }
            for (const SdfPath &target : constraint.targets) {
                // Only joints a solver also writes get a rebuilt chain; a
                // provider only constraints revise already carries exactly
                // this chain from the constraint pass, so leaving it alone
                // keeps every rig without a solver bit-identical.
                if (!newJointBinding.count(target))
                    continue;
                walkChains[target].push_back(constraint.moverPath);
            }
        }
        for (auto &[joint, writers] : walkWriters) {
            newJointBinding[joint] = std::move(writers);
        }
        for (auto &[joint, chain] : walkChains) {
            newFrameChains[joint] = chain;
        }
    }
    return true;
}

void
RigExecRigEvaluator::_CompileRestConstancy(_CompileAttempt &attempt)
{
    auto &newPropertyChains = attempt.newPropertyChains;
    auto &newPoseInputInfo = attempt.newPoseInputInfo;
    auto &newRestCompiledRigProviders = attempt.newRestCompiledRigProviders;
    // Close the provider universe from connected-expression facts. Values
    // are composed by native domain operations after all producers exist.
    for (const auto &[provider, info] : newPoseInputInfo)
        _SeedProvider(attempt, provider, false);
    auto &newCompiledRigRestsConstant = attempt.newCompiledRigRestsConstant;
    std::set<SdfPath> restChainTargets;
    for (const auto &entry : newPropertyChains)
        restChainTargets.insert(entry.first);
    for (const SdfPath &provider : newRestCompiledRigProviders) {
        if (_ProviderRestMightVary(_stage, _restInputNames, provider,
                                   restChainTargets)) {
            newCompiledRigRestsConstant = false;
            break;
        }
    }
}

bool
RigExecRigEvaluator::_CompileValidatePhases(_CompileAttempt &attempt)
{
    auto &newMovers = attempt.newMovers;
    auto &volumeWeightError = attempt.volumeWeightError;
    auto &newFrameChains = attempt.newFrameChains;
    auto &newGraphChains = attempt.newGraphChains;
    auto &newPhaseCheckpoints = attempt.newPhaseCheckpoints;
    _CompileFailure chainFailure;
    if (!_ValidateNativePhases(newGraphChains, newFrameChains, newMovers,
                               &newPhaseCheckpoints, &chainFailure)) {
        return attempt.Fail(chainFailure.message,
                            std::move(chainFailure.operations));
    }
    if (!volumeWeightError.empty()) {
        return attempt.Fail(volumeWeightError);
    }
    return true;
}

void
RigExecRigEvaluator::_CompilePublish(_CompileAttempt &attempt)
{
    auto &newJointPaths = attempt.newJointPaths;
    auto &newControlPaths = attempt.newControlPaths;
    auto &solverArrayPaths = attempt.solverArrayPaths;
    auto &newPoseInterpolators = attempt.newPoseInterpolators;
    auto &newRibbonDriverPoints = attempt.newRibbonDriverPoints;
    auto &newMovers = attempt.newMovers;
    auto &newJointBinding = attempt.newJointBinding;
    auto &newSolverDependencies = attempt.newSolverDependencies;
    auto &newFalloffLutOverrides = attempt.newFalloffLutOverrides;
    auto &newCurrentPhaseWeights = attempt.newCurrentPhaseWeights;
    auto &newVolumeWeightProviders = attempt.newVolumeWeightProviders;
    auto &newFrameConstraints = attempt.newFrameConstraints;
    auto &newFrameChains = attempt.newFrameChains;
    auto &poseStackOrdinal = attempt.poseStackOrdinal;
    auto &newXformDerivedProviders = attempt.newXformDerivedProviders;
    auto &newPropertyChains = attempt.newPropertyChains;
    auto &newPropertyChainOrder = attempt.newPropertyChainOrder;
    auto &newPhasedConnections = attempt.newPhasedConnections;
    auto &newGraphChains = attempt.newGraphChains;
    auto &newGraphDerivedChains = attempt.newGraphDerivedChains;
    auto &newSolverJoints = attempt.newSolverJoints;
    auto &poseDependencies = attempt.poseDependencies;
    auto &newSolverBatches = attempt.newSolverBatches;
    auto &newNativeProviderPaths = attempt.newNativeProviderPaths;
    auto &newRestCompiledRigProviders = attempt.newRestCompiledRigProviders;
    auto &newSpaceSwitches = attempt.newSpaceSwitches;
    auto &newCompiledRigRestsConstant = attempt.newCompiledRigRestsConstant;
    auto &newPhaseCheckpoints = attempt.newPhaseCheckpoints;
    _movers = std::move(newMovers);
    _jointPaths = std::move(newJointPaths);
    _controlPaths = std::move(newControlPaths);
    _poseInterpolators = std::move(newPoseInterpolators);

    _poseWeightProperties.clear();
    for (const _PoseInterpolator &interpolator : _poseInterpolators) {
        _poseWeightProperties.insert(_poseWeightProperties.end(),
                                     interpolator.disabledPoseWeights.begin(),
                                     interpolator.disabledPoseWeights.end());
        _poseWeightProperties.insert(_poseWeightProperties.end(),
                                     interpolator.poseWeights.begin(),
                                     interpolator.poseWeights.end());
    }
    _frameConstraints = std::move(newFrameConstraints);
    _spaceSwitches = std::move(newSpaceSwitches);
    _frameChains = std::move(newFrameChains);
    _xformDerivedProviders = std::move(newXformDerivedProviders);
    _ribbonDriverPoints = std::move(newRibbonDriverPoints);

    _falloffLutOverrides = std::move(newFalloffLutOverrides);
    _currentPhaseWeights = std::move(newCurrentPhaseWeights);
    _nativeVolumeProviders = std::move(newVolumeWeightProviders);
    _nativeGuideSolvers.clear();
    _nativeGuideSolvers.insert(solverArrayPaths.begin(),
                               solverArrayPaths.end());

    _nativePoseDependencies = std::move(poseDependencies);
    _nativePoseOrdinals = std::move(poseStackOrdinal);

    _nativeProviderPaths = std::move(newNativeProviderPaths);
    _restCompiledRigProviderPaths = std::move(newRestCompiledRigProviders);
    _compiledRigRestsConstant = newCompiledRigRestsConstant;
    _hierarchicalProviderSet.clear();
    _hierarchicalProviderSet.reserve(_nativeProviderPaths.size());
    for (const SdfPath &provider : _nativeProviderPaths)
        _hierarchicalProviderSet.insert(provider);
    // Anchors and intervening-Xform candidates for the new compiled rig. Both are
    // pure namespace topology; recomputing them per frame cost an xform-cache
    // query per provider only to be told "identity" on every rig that has no
    // such Xform, which is nearly all of them.
    _poseProviderAnchors.clear();
    _interveningXformProviders.clear();
    {
        const SdfPath assetRootPath = _rigPath.GetParentPath();
        for (const SdfPath &provider : _nativeProviderPaths) {
            SdfPath anchorPath;
            for (SdfPath walk = provider.GetParentPath();
                 !walk.IsEmpty() && !walk.IsAbsoluteRootPath() &&
                 walk != assetRootPath;
                 walk = walk.GetParentPath()) {
                if (_nativeProviderPaths.count(walk)) {
                    anchorPath = walk;
                    break;
                }
            }
            _poseProviderAnchors[provider] = anchorPath;
            if (provider.GetParentPath() !=
                (anchorPath.IsEmpty() ? assetRootPath : anchorPath)) {
                _interveningXformProviders.push_back(provider);
            }
        }
    }
    _solverBatches = std::move(newSolverBatches);
    _solverJoints = std::move(newSolverJoints);
    _jointSolverBinding = std::move(newJointBinding);
    _solverDependencies = std::move(newSolverDependencies);
    _nativePhaseCheckpoints = std::move(newPhaseCheckpoints);
    _graphChains = std::move(newGraphChains);
    _graphDerivedChains = std::move(newGraphDerivedChains);
    // Both caches below are keyed by nothing but a path, so the compiled rig they
    // belong to has to be stated by emptying them when it ends: the
    // influence table a layout was range-checked against and the base points
    // a source holds are both things a recompile can have changed.
    _skinTopologies.Clear();
    _rigState.skinLayoutInputsValid = false;
    _blendSampleShapes.Clear();
    _rigState.blendSampleSourcesValid = false;
    // Derived results are keyed by target like the live graphs, and the
    // walk may read them from several tasks at once: every entry exists
    // before a generation starts, so no task ever inserts into the map.
    _derivedCache.clear();
    for (const auto &[chainTarget, revisions] : _graphDerivedChains) {
        for (const _GraphRevision &derived : revisions) {
            _derivedCache[derived.target];
        }
    }
    _propertyChains = std::move(newPropertyChains);
    // The scalar primvar subset, once per compile on the owner thread.
    // _propertyChains is ordered by target, so the list is sorted.
    constexpr size_t kPrimvarsLength = sizeof("primvars:") - 1;
    _scalarPrimvarTargets.clear();
    _scalarPrimvarNames.clear();
    for (const auto &[target, revisions] : _propertyChains) {
        if (revisions.empty())
            continue;
        const UsdAttribute attribute =
            _stage ? _stage->GetAttributeAtPath(target) : UsdAttribute();
        if (!attribute ||
            !RigExecIsScalarPrimvarTarget(target, attribute.GetTypeName())) {
            continue;
        }
        _scalarPrimvarTargets.push_back(target);
        _scalarPrimvarNames.emplace_back(
            target.GetName().substr(kPrimvarsLength));
    }
    _propertyChainOrder = std::move(newPropertyChainOrder);
    _phasedConnections = std::move(newPhasedConnections);
    // Every attribute a chain READS through a connection.
    //
    // The avar-only notice path below skips dropping the chain bindings, on
    // the reasoning that a float-typed chain input cannot reach a double
    // avar. It can: a connection from a mover's float inputs:value to a
    // control's double avars:ty resolves and is what every "distance from
    // its rest, from the control's own translate" chain in this rig is built
    // out of -- cheeks.py's mouth corner, squetch.py's head wire. Without
    // this set such a chain answered once and then held that answer for the
    // life of the stage, so the cheeks stopped following the mouth corner
    // the moment an avar was AUTHORED rather than dragged.
    _propertyChainInputs.clear();
    for (const auto &[target, revisions] : _propertyChains) {
        for (const _PropertyRevision &revision : revisions) {
            const UsdPrim mover = _stage->GetPrimAtPath(revision.moverPath);
            if (!mover)
                continue;
            for (const TfToken &name :
                 {_tokens->inputsValue, _tokens->inputsMin,
                  _tokens->inputsMax}) {
                const UsdAttribute a = mover.GetAttribute(name);
                if (!a)
                    continue;
                for (const SdfPath &source : _AuthoredConnections(a)) {
                    _propertyChainInputs.insert(source);
                }
            }
        }
    }
    // The bindings describe the chains entry for entry, so a recompile that
    // replaced them has replaced what the bindings are about.
    _propertyChainBindings.reset();
}

bool
RigExecRigEvaluator::_CompileCommit(_CompileAttempt &attempt)
{
    auto &compileSerial = attempt.compileSerial;
    auto &retiringBakedProgram = attempt.retiringBakedProgram;
    auto &digestParts = attempt.digestParts;
    auto &digestFootprints = attempt.digestFootprints;
    auto &digestDispatcher = attempt.digestDispatcher;
    auto &digestJoined = attempt.digestJoined;
    auto &compileScene = attempt.compileScene;
    auto &compileSceneStage = attempt.compileSceneStage;
    auto &compileSceneSerial = attempt.compileSceneSerial;
    auto &compileSceneReusable = attempt.compileSceneReusable;
    _rigState.Transition(_CompiledRigState::Edge::Committed);

    // Build the dense provider universe once per compile compiled rig. _providerPaths
    // is SdfPath-sorted, so ascending live-index iteration is SdfPath key
    // order, which the frame-domain publish loops rely on.
    {
        std::set<SdfPath> universe;
        auto addU = [&](const SdfPath &p) {
            if (!p.IsEmpty())
                universe.insert(p);
        };
        for (const auto &path : _nativeProviderPaths)
            addU(path);
        for (const auto &p : _xformDerivedProviders)
            addU(p);
        for (const auto &p : _interveningXformProviders)
            addU(p);
        for (const _FrameConstraint &fc : _frameConstraints) {
            addU(fc.moverPath);
            for (const auto &t : fc.targets)
                addU(t);
            for (const auto &s : fc.sources)
                addU(s.sourcePath);
            addU(fc.worldUpObject.sourcePath);
            addU(fc.effector.sourcePath);
            for (const auto &po : fc.poleObjects)
                addU(po.sourcePath);
            for (const auto &ik : fc.ikChain)
                addU(ik);
            addU(fc.pointsTarget);
            addU(fc.weightObject);
        }
        for (const auto &kv : _solverJoints) {
            addU(kv.first);
            for (const auto &p : kv.second)
                addU(p.first);
        }
        for (const _SolverBatch &b : _solverBatches) {
            for (const auto &d : b.dependencies)
                addU(d);
            for (const auto &d : b.frameInputs)
                addU(d);
            for (const auto &kv : b.restInputs) {
                addU(kv.first);
                addU(kv.second);
            }
            for (const auto &path : b.solvers)
                addU(path);
        }
        for (const auto &kv : _nativePhaseCheckpoints) {
            addU(kv.first);
            for (const auto &v : kv.second)
                addU(v);
        }
        _providerPaths.assign(universe.begin(), universe.end());
        _providerIndex.clear();
        _providerIndex.reserve(_providerPaths.size());
        for (std::size_t i = 0; i < _providerPaths.size(); ++i)
            _providerIndex.emplace(_providerPaths[i], static_cast<int>(i));
        _hierDescendants.assign(_providerPaths.size(), {});
        for (std::size_t i = 0; i < _providerPaths.size(); ++i) {
            if (_nativeProviderPaths.count(_providerPaths[i]) == 0)
                continue;
            for (std::size_t j = i + 1;
                 j < _providerPaths.size() &&
                 _providerPaths[j].HasPrefix(_providerPaths[i]);
                 ++j)
                if (_nativeProviderPaths.count(_providerPaths[j]))
                    _hierDescendants[i].push_back(static_cast<int>(j));
        }
    }
    // A notice that landed during this compile still stands.
    _rigState.Transition(_CompiledRigState::Edge::CommitJoined,
                         _stageEditSerial != compileSerial);
    // Compile the complete native domain graph for this compiled rig.
    _RebuildBakedProgram(std::move(retiringBakedProgram),
                         compileSceneReusable ? compileScene.get() : nullptr,
                         compileSceneStage, compileSceneSerial);
    _restEditedProviders.clear();
    {
        // Join the digest task: from here on its result is read. Last, and
        // after the bake: _ComputeStructureDigest reads only the stage and
        // the profiler, nothing this compile commits, so nothing in between
        // needs its answer -- and joined here the bake does not queue
        // behind it on the rigs where the digest is the longer lane.
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.DigestJoin", "compile");
        digestDispatcher.Wait();
        _structureDigest = _stage ? _JoinStructureDigest(digestParts) : 0;
        digestJoined = true;
    }
    if (_stage) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.DigestGate", "compile");
        _rigState.digestGate = _MakeDigestGate(digestFootprints);
    }
    return true;
}

} // namespace rigExec
