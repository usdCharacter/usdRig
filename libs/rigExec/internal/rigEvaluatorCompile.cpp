// Compiled rigs: the compile attempt driver, discovery and validation, and
// settling.

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

namespace rigExec {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((inputElement, RigExecInputElementMetadataName))
    ((rigExecDriverCurve, SchemaAttrText::rigExecDriverCurve))
    (points)
    ((rigExecSpace, SchemaAttrText::rigExecSpace))
);

using namespace evaluatorDetail;

bool
RigExecRigEvaluator::Compile(std::vector<std::string> *errors)
{
    _AssertOwnerThread();
    RigExecInputReplayCompileCall compileCall;
    if (_inputReplayObserver) {
        _inputReplayObserver->Options(_solverGuidesEnabled,
                                      _publishWeightFields);
        compileCall = _inputReplayObserver->Compile(errors);
    }
    RigExecInputReplayImplementationScope replayImplementation{
        bool(_inputReplayObserver)};
    // A caller asking for a compile is asking the stage again, whatever the
    // settle path last concluded about it: nothing the memo remembers may
    // answer for this call, and whatever this call concludes is the caller's
    // to read, not the settle path's to replay.
    _rigState.failedCompile = _FailedCompileMemo();
    // Element selection is an authored input contract, not a recoverable
    // operation cycle.
    const UsdPrim inputRoot =
        _stage ? _stage->GetPrimAtPath(_rigPath) : UsdPrim();
    if (inputRoot)
        for (const auto &prim : UsdPrimRange(inputRoot)) {
            for (const auto &attribute : prim.GetAttributes()) {
                if (!attribute.HasAuthoredMetadata(_tokens->inputElement))
                    continue;
                VtValue element;
                attribute.GetMetadata(_tokens->inputElement, &element);
                if (!element.IsHolding<int>() ||
                    element.UncheckedGet<int>() < 0 ||
                    attribute.GetTypeName().GetType() !=
                        SdfValueTypeNames->Float3.GetType()) {
                    if (errors)
                        errors->push_back(attribute.GetPath().GetString() +
                                          ": rigExecInputElement requires a "
                                          "nonnegative int on a Vec3f input");
                    if (_inputReplayObserver)
                        _inputReplayObserver->CompileResult(compileCall, false,
                                                            errors);
                    return false;
                }
            }
        }
    const bool compiled = _CompileRig(errors);
    // The shadow is asked every question this evaluator is, in the same
    // order, so that the one thing that differs between them is how their
    // notices clear the value caches.
    _EnsureScopedClearShadow();
    if (_scopedClearShadow) {
        _scopedClearShadow->Compile();
    }
    if (_inputReplayObserver)
        _inputReplayObserver->CompileResult(compileCall, compiled, errors);
    return compiled;
}

// A failure with no operation owner rejects the rig as a whole.
bool
RigExecRigEvaluator::_CompileRig(std::vector<std::string> *errors)
{
    std::string sceneError;
    if (!RigExecSceneAvailable(&sceneError)) {
        if (errors)
            errors->push_back(std::move(sceneError));
        return false;
    }

    static constexpr size_t kMaxSkippedOperations = 64;

    const std::map<SdfPath, std::string> committedSkips = _skippedOperations;
    _skippedOperations.clear();
    std::vector<std::string> attempt;
    for (;;) {
        attempt.clear();
        _CompileFailure failure;
        if (_CompileRigAttempt(&attempt, &failure)) {
            break;
        }
        // The operation the error is about -- or, for a dependency cycle,
        // every operation in it.
        const auto &culprits = failure.operations;
        const bool repeat = std::any_of(
            culprits.begin(), culprits.end(),
            [this](const SdfPath &path) { return _IsSkippedOperation(path); });
        if (culprits.empty() || repeat ||
            _skippedOperations.size() >= kMaxSkippedOperations) {
            // The rig fails as a whole, and the
            // program it had -- with the operations IT had set aside --
            // keeps running. What was set aside on the way is said first:
            // a rig whose only operation is broken fails with "publishes
            // no outputs" once that operation is gone, and that line alone
            // would hide the reason.
            if (errors) {
                // Once per reason: a cycle sets aside every member
                // for the one error.
                std::set<std::string> said;
                for (const auto &[path, reason] : _skippedOperations) {
                    if (said.insert(reason).second) {
                        errors->push_back(reason);
                    }
                }
                errors->insert(errors->end(), attempt.begin(), attempt.end());
            }
            _skippedOperations = committedSkips;
            return false;
        }
        for (const SdfPath &culprit : culprits) {
            _skippedOperations.emplace(culprit, failure.message);
        }
    }
    for (const auto &[path, reason] : _skippedOperations) {
        const std::string line =
            "Skipped operation " + path.GetString() + " -- " + reason;
        TF_WARN("RigExec %s: %s", _rigPath.GetText(), line.c_str());
        if (errors) {
            errors->push_back("warning: " + line);
        }
    }
    if (errors) {
        errors->insert(errors->end(), attempt.begin(), attempt.end());
    }
    return true;
}

bool
RigExecRigEvaluator::_CompileRigAttempt(std::vector<std::string> *errors,
                                        _CompileFailure *failure)
{
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile", "compile");
    const uint64_t compileSerial = _stageEditSerial;
    // Exec preparation may load Python plugins on a worker. Release the GIL
    // through dispatcher destruction too, since early returns still join tasks.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    _CompileAttempt attempt(*this, errors, failure, compileSerial);
    _CompileRetireProgram(attempt);
    attempt.StampCompileRegion("Compile.Prologue");
    _CompileDeriveStartFrames(attempt);
    attempt.StampCompileRegion("Compile.DerivedStartFrames");
    _CompileDispatchDigest(attempt);
    // Zero-width when the digest is dispatched, which is the point: it marks
    // WHERE the digest was launched so the trace can be read against the
    // worker row. With RIGEXEC_ENABLE_PARALLEL_EVAL=0 the call above ran
    // inline and this stamp measures the whole of it.
    attempt.StampCompileRegion("Compile.StructureDigest");

    RigExecProfilePhases &blocks = attempt.compileBlocks;
    blocks.Next("DiscoverValidate.Validate");
    if (!_CompileDiscoverOutputs(attempt)) {
        return false;
    }
    blocks.Next("DiscoverValidate.WarmupDispatch");
    blocks.Next("DiscoverValidate.Providers");
    if (!_CompileDiscoverProviders(attempt)) {
        return false;
    }
    blocks.Next("DiscoverValidate.AggregateProviders");
    _CompileAggregateProviders(attempt);
    blocks.Next("DiscoverValidate.MoverDiscovery");
    if (!_CompileDiscoverMovers(attempt)) {
        return false;
    }
    blocks.Next("DiscoverValidate.OutputCheck");
    if (!_CompileCheckOutputs(attempt)) {
        return false;
    }
    blocks.Next("DiscoverValidate.SolverJointBinding");
    if (!_CompileBindSolverJoints(attempt)) {
        return false;
    }
    blocks.Next("DiscoverValidate.SolverDag");
    _CompileSolverDag(attempt);
    blocks.Close();
    attempt.StampCompileRegion("Compile.DiscoverValidate");
    blocks.Next("SolverSchedule.ReplacementRequests");
    _CompileArmInvalidation(attempt);
    blocks.Next("SolverSchedule.VolumeWeights");
    _CompileVolumeWeights(attempt);
    blocks.Next("SolverSchedule.PoseConstraints");
    if (!_CompilePoseConstraints(attempt)) {
        return false;
    }
    blocks.Next("SolverSchedule.PropertyChains");
    if (!_CompilePropertyChains(attempt)) {
        return false;
    }
    blocks.Next("SolverSchedule.PointGraph");
    if (!_CompilePointGraph(attempt)) {
        return false;
    }
    blocks.Next("SolverSchedule.SampleReads");
    _CompileSampleReads(attempt);
    blocks.Next("SolverSchedule.DerivedMaintenance");
    if (!_CompileDerivedMaintenance(attempt)) {
        return false;
    }
    blocks.Next("SolverSchedule.PoseDag");
    if (!_CompilePoseDag(attempt)) {
        return false;
    }
    blocks.Next("SolverSchedule.RequiredSolvers");
    _CompileRequiredSolvers(attempt);
    blocks.Next("SolverSchedule.ConstraintDeps");
    _CompileConstraintDependencies(attempt);
    blocks.Next("SolverSchedule.FrameInheritance");
    if (!_CompileFrameInheritance(attempt)) {
        return false;
    }
    blocks.Next("SolverSchedule.PoseReady");
    _CompilePoseReady(attempt);
    blocks.Next("PrepareRequests.SpaceSwitches");
    if (!_CompileSpaceSwitches(attempt)) {
        return false;
    }
    blocks.Next("PrepareRequests.ProviderClosure");
    if (!_CompileProviderClosure(attempt)) {
        return false;
    }
    blocks.Next("SolverSchedule.SolverBatches");
    if (!_CompileSolverBatches(attempt)) {
        return false;
    }
    blocks.Close();
    attempt.StampCompileRegion("Compile.SolverSchedule");
    _CompileRestConstancy(attempt);
    blocks.Close();
    attempt.StampCompileRegion("Compile.PrepareRequests");
    if (!_CompileValidatePhases(attempt)) {
        return false;
    }
    attempt.StampCompileRegion("Compile.ChainOrderValidate");
    _CompilePublish(attempt);
    attempt.StampCompileRegion("Compile.Commit");
    return _CompileCommit(attempt);
}

RigExecRigEvaluator::_CompileAttempt::_CompileAttempt(
    RigExecRigEvaluator &evaluator, std::vector<std::string> *errors,
    _CompileFailure *failure, uint64_t compileSerial)
    : evaluator(evaluator), errors(errors), failure(failure),
      compileSerial(compileSerial),
      profileCompile(evaluator._profiler.IsEnabled()),
      compileRegionStart(profileCompile ? RigExecProfiler::NowUs() : 0),
      verifyDigestOnEarlyReturn([this]() {
          if (!digestJoined && this->evaluator._stage &&
              _DigestSplitVerifyRequested()) {
              digestDispatcher.Wait();
              this->evaluator._JoinStructureDigest(digestParts);
          }
      }),
      compileBlocks(&evaluator._profiler, "compile")
{
}

RigExecRigEvaluator::_CompileAttempt::~_CompileAttempt() = default;

bool
RigExecRigEvaluator::_CompileAttempt::Fail(const std::string &message,
                                           SdfPathVector operations)
{
    *failure = {message, std::move(operations)};
    if (errors) {
        errors->push_back(message);
    }
    if (invalidateCompiledRig)
        invalidateCompiledRig();
    return false;
}

void
RigExecRigEvaluator::_CompileAttempt::StampCompileRegion(const char *name)
{
    if (!profileCompile) {
        return;
    }
    const uint64_t now = RigExecProfiler::NowUs();
    evaluator._profiler.Record(name, "compile", compileRegionStart, now);
    compileRegionStart = now;
}

// A chain's rigExec:startFrame with no composed targets reads the
// provider its policy derived, exactly where an authored target would.
void
RigExecRigEvaluator::_CompileAttempt::AppendDerivedStart(
    const UsdRelationship &rel, SdfPathVector *targets) const
{
    if (!targets->empty() || rel.GetName() != RigExecStartFrameRelName) {
        return;
    }
    const auto derived = derivedStartFrames.find(rel.GetPrimPath());
    if (derived != derivedStartFrames.end()) {
        targets->push_back(derived->second);
    }
}

// Strictly before, in the stack restricted to solvers. solverStackOrdinal
// is the reverse composed pre-order of the whole rig, so its restriction
// to the solvers IS the unified pose stack restricted to them; the
// constraint half only interleaves between them and cannot reorder a
// solver pair.
// Only a READER that is itself a stack step consults it. A PRODUCER has no
// position for the comparison to be about, so it keeps the unconditional
// "wait for every writer of what I read" edge -- which is
// its data-flow meaning, and which leaves a genuine loop among producers
// (two of them reading each other's joints) a cycle, reported as one.
bool
RigExecRigEvaluator::_CompileAttempt::StackBefore(const SdfPath &a,
                                                  const SdfPath &b) const
{
    const auto ia = solverStackOrdinal.find(a);
    const auto ib = solverStackOrdinal.find(b);
    if (ia == solverStackOrdinal.end() || ib == solverStackOrdinal.end()) {
        return false;
    }
    return ia->second < ib->second;
}

// Ordinal or "no position": a producer sorts before every stack step and
// is deterministic about it, which is all the emission order needs.
int
RigExecRigEvaluator::_CompileAttempt::StackOrdinalOf(const SdfPath &path) const
{
    const auto it = poseStackOrdinal.find(path);
    return it == poseStackOrdinal.end() ? -1 : it->second;
}

void
RigExecRigEvaluator::_CompileRetireProgram(_CompileAttempt &attempt)
{
    // The program describes the compiled rig this call is about to replace, so
    // it stops being the rig's program here: a Compile that fails and restores
    // the previous compiled rig returns before the rebuild at the tail, this
    // local goes out of scope, and the rig is left dynamic -- slower and never
    // wrong.
    attempt.retiringBakedProgram = std::move(_bakedProgram);
    // A new compiled rig is a new question: whatever refused the last one said
    // nothing about this one, and whatever made its program bail neither.
    _programState.Transition(_ProgramState::Edge::CompileBegan);

    // The committed footprint answers for the committed digest, and this
    // call is about to replace both: until its own digest is joined, every
    // notice is suspect.
    // The candidates noted against it go with it: this compile reads the
    // stage they describe.
    _rigState.digestGate = _DigestGate();
    _rigState.certainCandidates.clear();
    _rigState.certainCandidatesOverflowed = false;
}

void
RigExecRigEvaluator::_CompileDeriveStartFrames(_CompileAttempt &attempt)
{
    std::vector<std::string> *errors = attempt.errors;
    auto &derivedStartFrames = attempt.derivedStartFrames;
    // rigExec:startFramePolicy, resolved once per compile into chain ->
    // provider. Every solver-input walk below reads a chain's start frame
    // through this map when it has no authored target; the baked program
    // and scene translation apply the same rule to the captured scene. The
    // stage is never written. The digest covers the rule's inputs (policy,
    // joints, their namespace ancestry). Warnings go to both channels, like
    // the transform-authority pass: TF_WARN is what a host surfaces to the
    // author, the errors vector is what a test can read.
    for (const std::string &message : _DeriveStartFrames(&derivedStartFrames)) {
        if (errors) {
            errors->push_back("warning: " + message);
        }
        TF_WARN("%s", message.c_str());
    }
}

void
RigExecRigEvaluator::_CompileDispatchDigest(_CompileAttempt &attempt)
{
    auto &digestParts = attempt.digestParts;
    auto &digestFootprints = attempt.digestFootprints;
    auto &digestDispatcher = attempt.digestDispatcher;
    const auto computeDigestPart = [this, &digestParts,
                                    &digestFootprints](size_t i) {
        if (_stage) {
            digestParts[i] = _ComputeStructureDigest(
                (1u << i) | _DigestBesideCompile, &digestFootprints[i]);
        }
    };
    for (const size_t i : {size_t(1), size_t(2), size_t(0)}) {
        if (RigExecParallelEvaluationEnabled()) {
            digestDispatcher.Run(
                [computeDigestPart, i]() { computeDigestPart(i); });
        } else {
            computeDigestPart(i);
        }
    }
}

bool
RigExecRigEvaluator::_CompileDiscoverOutputs(_CompileAttempt &attempt)
{
    if (!_stage) {
        return attempt.Fail("no stage; nothing to compile");
    }

    auto &rig = attempt.rig;
    rig = _stage->GetPrimAtPath(_rigPath);
    if (!rig) {
        return attempt.Fail("Rig prim not found: " + _rigPath.GetString());
    }
    // Prototype-hosted rigs fail validation (a rig in a
    // prototype or otherwise unable to deinstance is rejected).
    if (rig.IsInstanceProxy() || rig.IsInPrototype()) {
        return attempt.Fail("Rig is instance-proxy/prototype hosted: " +
                            _rigPath.GetString());
    }
    // Phase A: validation into locals. Nothing below mutates evaluator
    // state until every check passes, so a failed structural edit keeps
    // the previous compiled rig publishable.
    auto &newJointPaths = attempt.newJointPaths;
    newJointPaths = _DiscoverJointOutputs(_stage, _rigPath, _skippedOperations);
    // A rig with no joints is legal: a mover writes an exact target, and a
    // target is a points array, a plain UsdGeomXformable's transform, or a
    // scalar property just as readily as it is a joint frame -- three output
    // domains that all reach a consumer through RigExecRigPose. The simplest
    // rig there is, a constraint aiming one Xform at another, has none.
    // What the rig DOES need is at least one output, and that cannot be known
    // until the mover walk below has run, so the check is there.
    // Controls and placed volumes are discovered alongside the joints. Zero
    // of either kind is ordinary; the combined output gate below decides
    // whether the whole rig is genuinely empty.
    auto &newControlPaths = attempt.newControlPaths;
    newControlPaths = _DiscoverControls(_stage, _rigPath);
    auto &newVolumeWeightPaths = attempt.newVolumeWeightPaths;
    newVolumeWeightPaths = _DiscoverVolumeWeights(_stage, _rigPath);
    // Every aggregate frame provider, wherever the author placed it: their
    // computePointFrameArray results are published (and drawn as guides by
    // the imaging chain, like OpenExec's IrJointScope guides). Discovered
    // here, with the other providers, because it is the last thing the exec
    // lane below needs before it can start.
    auto &aggregateSolvers = attempt.aggregateSolvers;
    aggregateSolvers = _DiscoverAggregateSolvers(_stage, _rigPath);
    aggregateSolvers.erase(
        std::remove_if(aggregateSolvers.begin(), aggregateSolvers.end(),
                       [this](const UsdPrim &solver) {
                           return _IsSkippedOperation(solver.GetPath());
                       }),
        aggregateSolvers.end());
    auto &solverArrayPaths = attempt.solverArrayPaths;
    solverArrayPaths.reserve(aggregateSolvers.size());
    for (const UsdPrim &solver : aggregateSolvers) {
        solverArrayPaths.push_back(solver.GetPath());
    }
    return true;
}

bool
RigExecRigEvaluator::_CompileDiscoverProviders(_CompileAttempt &attempt)
{
    std::vector<std::string> *errors = attempt.errors;
    auto &newJointPaths = attempt.newJointPaths;
    auto &newControlPaths = attempt.newControlPaths;
    auto &newVolumeWeightPaths = attempt.newVolumeWeightPaths;
    auto &solverArrayPaths = attempt.solverArrayPaths;
    // Pose interpolators, discovered and SOLVED here. Solving is the whole
    // reason this is compile-time work: inverting the matrix of every pose's
    // kernel value at every other pose is a constant of the authored data,
    // and doing it per frame would be inverting the same matrix 326 times a
    // second to be handed the same answer.
    auto &compileScene = attempt.compileScene;
    auto &compileSceneStage = attempt.compileSceneStage;
    compileSceneStage = _stage;
    auto &compileSceneSerial = attempt.compileSceneSerial;
    auto &compileSceneReusable = attempt.compileSceneReusable;
    auto &newPoseInterpolators = attempt.newPoseInterpolators;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler, "Compile.Providers.PoseInterpolators", "compile");
        compileScene = std::make_unique<RigExecSceneDescriptors>();
        const uint64_t before = GetStageEditSerial();
        std::string interpolatorError;
        SdfPath interpolatorOperation;
        if (!_CompilePoseInterpolators(
                newJointPaths, newControlPaths, &newPoseInterpolators, errors,
                &interpolatorError, &interpolatorOperation, compileScene.get(),
                &_profiler)) {
            return attempt.Fail(interpolatorError, {interpolatorOperation});
        }
        // No interpolators means no capture. Translation reads use the fresh
        // Default snapshot.
        if (compileScene->rigRoot.IsEmpty()) {
            compileScene.reset();
        } else {
            compileSceneSerial = before;
            compileSceneReusable = before == GetStageEditSerial();
        }
    }

    // Transform-authority validation (host-durability redesign).
    // Neither condition can FAIL a compile, and both are reported rather
    // than fixed: the rig still evaluates exactly right, because the
    // evaluator reads rest:space and the avars and nothing else. What
    // breaks is the BOUNDS -- a provider's computed extent bakes its posed
    // frame into asset-relative space, which is only the whole story while
    // nothing else contributes a transform between the asset root and the
    // provider. Refusing to compile over a framing inaccuracy would be
    // wildly out of proportion; saying nothing would leave an author
    // wondering why one control frames to the wrong place.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler, "Compile.Providers.BoundsValidation", "compile");
        const SdfPath assetRoot = _rigPath.GetParentPath();
        auto warn = [errors](const std::string &message) {
            // TF_WARN for hosts, errors for tests; Compile returns true.
            if (errors) {
                errors->push_back("warning: " + message);
            }
            TF_WARN("%s", message.c_str());
        };
        // Resolving a prim's purpose walks up the namespace to the first
        // authored opinion, and this pass asks for it once per provider and
        // then again for every descendant of every provider -- so a joint
        // deep in a chain is resolved once per ancestor provider. Purpose is
        // a pure function of the composed stage, which does not change while
        // a compile runs, so resolve each prim once.
        std::unordered_map<SdfPath, TfToken, SdfPath::Hash> purposeCache;
        const auto resolvedPurpose = [&purposeCache](const UsdPrim &prim) {
            auto it = purposeCache.find(prim.GetPath());
            if (it == purposeCache.end()) {
                it = purposeCache
                         .emplace(prim.GetPath(),
                                  UsdGeomImageable(prim).ComputePurpose())
                         .first;
            }
            return it->second;
        };
        // Every Boundable provider, aggregate solvers included: they
        // inherit Boundable/Xformable too, so an authored op on one is
        // applied by BBoxCache to an already-baked extent while the guide
        // it draws ignores it entirely.
        std::vector<SdfPath> providers = newJointPaths;
        providers.insert(providers.end(), newControlPaths.begin(),
                         newControlPaths.end());
        providers.insert(providers.end(), newVolumeWeightPaths.begin(),
                         newVolumeWeightPaths.end());
        providers.insert(providers.end(), solverArrayPaths.begin(),
                         solverArrayPaths.end());
        for (const SdfPath &providerPath : providers) {
            const UsdPrim prim = _stage->GetPrimAtPath(providerPath);
            if (!prim) {
                continue;
            }
            // xformOps arrive on every provider because RigExecXformable
            // inherits UsdGeomBoundable, but they are NOT a transform
            // authority: rest:space plus the avars are the only one (the
            // Ir alignment). An authored op is a second one that nothing
            // reads, so the prim moves in a stock UsdGeom traversal while
            // the rig ignores it entirely.
            if (const UsdGeomXformable xformable = UsdGeomXformable(prim)) {
                bool resetsStack = false;
                if (!xformable.GetOrderedXformOps(&resetsStack).empty()) {
                    warn(prim.GetTypeName().GetString() + " " +
                         providerPath.GetString() +
                         " authors xformOps, which are not a transform "
                         "authority for a RigExec provider (rest:space and "
                         "the avars are); the ops are ignored by evaluation "
                         "and are not in the computed extent");
                }
            }
            // An Xformable BETWEEN the asset root and the provider is not
            // reported: it is composed at evaluation, by
            // _ComposeInterveningXforms, so placing a rig -- or one leg of an
            // assembly -- under an Xform inside the asset is a supported
            // shape, and warning ten times per compile about a configuration
            // that works is noise nobody can act on.
            // The check above is a different claim: an op authored on the
            // PROVIDER is not a transform authority.

            // Hoisted out of the walk: the provider's own purpose is the
            // same for every one of its descendants.
            const TfToken providerPurpose = resolvedPurpose(prim);

            // A provider's extent covers the guides beneath it, and only
            // those. Authored geometry parented under one is invisible to
            // it -- and to every ancestor, because UsdGeomBBoxCache stops
            // descending at a Boundable -- so the gprim silently drops out
            // of every bounding box in the scene.
            for (const UsdPrim &descendant : UsdPrimRange(prim)) {
                if (descendant == prim) {
                    continue;
                }
                // A provider nested under a provider with a DIFFERENT
                // resolved purpose is dropped from the ancestor's extent:
                // one extent, one purpose.
                if (descendant.IsA<UsdGeomImageable>()) {
                    const TfToken descendantPurpose =
                        resolvedPurpose(descendant);
                    if (!descendantPurpose.IsEmpty() &&
                        !providerPurpose.IsEmpty() &&
                        descendantPurpose != providerPurpose &&
                        TfStringStartsWith(descendant.GetTypeName().GetString(),
                                           SchemaNamePrefix::type)) {
                        warn(descendant.GetTypeName().GetString() + " " +
                             descendant.GetPath().GetString() +
                             " has purpose '" + descendantPurpose.GetString() +
                             "' but is nested under " +
                             providerPath.GetString() + " whose purpose is '" +
                             providerPurpose.GetString() +
                             "'; one extent carries one purpose, so this "
                             "provider is excluded from its ancestor's "
                             "bounds");
                    }
                }
                if (descendant.IsA<UsdGeomGprim>()) {
                    warn("gprim " + descendant.GetPath().GetString() +
                         " is parented under RigExec provider " +
                         providerPath.GetString() +
                         "; a provider's computed extent covers only the "
                         "guides beneath it, and bounds stop descending at "
                         "a Boundable, so this geometry is absent from "
                         "every bounding box that should contain it");
                }
            }
        }
    }

    auto &newRibbonDriverPoints = attempt.newRibbonDriverPoints;
    return true;
}

void
RigExecRigEvaluator::_CompileAggregateProviders(_CompileAttempt &attempt)
{
    auto &aggregateSolvers = attempt.aggregateSolvers;
    auto &newRibbonDriverPoints = attempt.newRibbonDriverPoints;
    for (const UsdPrim &child : aggregateSolvers) {
        // A ribbon's driver-curve points, resolved to the exact native
        // attribute. This replaces the compiler's last authoring pass:
        // the resolution is compiled state (rewiring the relationship is
        // structural, and the compiled rig signature already treats it that way),
        // and the values ride in as exec overrides at evaluation time.
        if (child.GetTypeName() == SchemaTypeName::RigExecRibbon) {
            SdfPathVector curves;
            if (const UsdRelationship rel =
                    child.GetRelationship(_tokens->rigExecDriverCurve)) {
                rel.GetTargets(&curves);
            }
            if (!curves.empty()) {
                newRibbonDriverPoints[child.GetPath()] =
                    curves[0].IsPrimPath()
                        ? curves[0].AppendProperty(_tokens->points)
                        : curves[0];
            }
        }
    }
}

bool
RigExecRigEvaluator::_CompileDiscoverMovers(_CompileAttempt &attempt)
{
    std::vector<std::string> *errors = attempt.errors;
    _CompileFailure *failure = attempt.failure;
    auto &newMovers = attempt.newMovers;
    auto &inertMovers = attempt.inertMovers;
    auto &newSurfaceProjectors = attempt.newSurfaceProjectors;
    if (!_DiscoverMovers(newMovers, newSurfaceProjectors, inertMovers, errors,
                         failure)) {
        return attempt.Fail(failure->message, failure->operations);
    }
    return true;
}

bool
RigExecRigEvaluator::_CompileCheckOutputs(_CompileAttempt &attempt)
{
    auto &newJointPaths = attempt.newJointPaths;
    auto &newControlPaths = attempt.newControlPaths;
    auto &newVolumeWeightPaths = attempt.newVolumeWeightPaths;
    auto &newMovers = attempt.newMovers;
    auto &inertMovers = attempt.inertMovers;
    // Error only when no control, joint, volume or mover exists. An inert mover
    // counts.
    if (newControlPaths.empty() && newJointPaths.empty() &&
        newVolumeWeightPaths.empty() && newMovers.empty() &&
        inertMovers.empty()) {
        return attempt.Fail(
            "Rig publishes no outputs: " + _rigPath.GetString() +
            " has no RigExecControl, RigExecJoint, or placed volume "
            "weight prims and no movers");
    }

    // Multiple writers of one target are an ordinary stack, not an error.
    // Their order is the reverse-sibling post-order walk of the FINAL COMPOSED
    // hierarchy above. UsdPrim::GetChildrenNames() returns the displayed
    // top-to-bottom order with any parent child-order instruction (reorder
    // nameChildren) already folded in; the stack consumes that order in
    // reverse so the bottom branch runs first. A reorder is a convenience for
    // redirecting that order, never a precondition for having one.
    // Reads only the final composed order; provenance is ignored.

    // final transform reads are legal only when every writer of that
    // provider precedes the reader in logical order: a
    // frame mover with a later ordinal than a consuming matrix mover is
    // an unsatisfied final read.
    {
        std::map<SdfPath, int> lastFrameWriterOrdinal;
        for (const RigExecMoverRecord &m : newMovers) {
            if (!_IsFrameConstraintType(m.schemaType)) {
                continue;
            }
            for (const SdfPath &t : m.targets) {
                if (t.IsPrimPath()) {
                    lastFrameWriterOrdinal[t] =
                        std::max(lastFrameWriterOrdinal.count(t)
                                     ? lastFrameWriterOrdinal[t]
                                     : -1,
                                 m.ordinal);
                }
            }
        }
        for (const RigExecMoverRecord &m : newMovers) {
            const RigExecMoverRegistration *transformHandler =
                RigExecFindMoverRegistration(m.schemaType);
            if (!transformHandler ||
                transformHandler->transformRelationship.IsEmpty()) {
                continue;
            }
            const TfToken &transformRel =
                transformHandler->transformRelationship;
            const UsdPrim prim = _stage->GetPrimAtPath(m.moverPath);
            if (!prim) {
                continue;
            }
            if (RigExecPhaseForInput(prim, transformRel).kind !=
                RigExecReadPhaseKind::Final) {
                continue;
            }
            SdfPathVector transforms;
            if (UsdRelationship rel = prim.GetRelationship(transformRel)) {
                rel.GetTargets(&transforms);
            }
            if (!transformHandler->spaceRelationship.IsEmpty()) {
                // Both spaces: the measuring one the handler names, and
                // rigExec:space, the rig's carry. They are read at the SAME
                // phase as the transform, so a final read of either carries
                // the same ordering obligation.
                for (const TfToken &name : {transformHandler->spaceRelationship,
                                            _tokens->rigExecSpace}) {
                    SdfPathVector spaces;
                    if (UsdRelationship rel = prim.GetRelationship(name)) {
                        rel.GetTargets(&spaces);
                    }
                    transforms.insert(transforms.end(), spaces.begin(),
                                      spaces.end());
                }
            }
            for (const SdfPath &provider : transforms) {
                const auto it = lastFrameWriterOrdinal.find(provider);
                if (it != lastFrameWriterOrdinal.end() &&
                    it->second > m.ordinal) {
                    return attempt.Fail(
                        "Unsatisfied final read: " + m.moverPath.GetString() +
                            " reads final of " + provider.GetString() +
                            " but a writer with a later ordinal exists "
                            "(spec §4.2)",
                        {m.moverPath});
                }
            }
        }
    }
    return true;
}

void
RigExecRigEvaluator::_CompileArmInvalidation(_CompileAttempt &attempt)
{
    auto &invalidateCompiledRig = attempt.invalidateCompiledRig;
    // Prepare replacement requests while retaining the previous requests and
    // their shared compiler context. The stock network handles changed USD
    // bindings incrementally; no whole execution-system replacement is needed.

    invalidateCompiledRig = [&]() {
        _rigState.Transition(_CompiledRigState::Edge::Invalidated);
    };
}

bool
RigExecRigEvaluator::_FailedCompileMemoMatches() const
{
    return _rigState.failedCompile.valid &&
           _rigState.failedCompile.stageEditSerial == _stageEditSerial;
}

bool
RigExecRigEvaluator::_CompileUnlessKnownBroken(
    std::vector<std::string> *diagnostics)
{
    // Library availability is process state, not a stage failure to memoize.
    std::string sceneError;
    if (!RigExecSceneAvailable(&sceneError)) {
        if (diagnostics)
            diagnostics->push_back(std::move(sceneError));
        return false;
    }

    // A compile that failed fails again for as long as nothing it read has
    // moved, and what it reads is the stage and the mode. So a failure is
    // answered from the memo while the key still matches: the same errors,
    // in the same order, where the compile would have put them, at the cost
    // of a key compare instead of a full compile (plus, at the structural
    // site, the digest ahead of it) on every frame of a scrub over a stage
    // somebody left broken.
    // The documented difference is what the compile says OUTSIDE the vector:
    // its TF_WARNs -- a derived start-frame note, the transform-authority
    // pass -- go to the host once, on the compile that failed, rather than
    // once per frame; and the program's build and attempt counters stand
    // still, because nothing is built or attempted.
    if (_FailedCompileMemoMatches()) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Settle.KnownBroken", "evaluate");
        if (diagnostics) {
            diagnostics->insert(diagnostics->end(),
                                _rigState.failedCompile.errors.begin(),
                                _rigState.failedCompile.errors.end());
        }
        return false;
    }
    // Compile's own lines, kept apart so that the memo holds exactly what a
    // compile says and never what the caller says around it ("structural
    // recompilation failed" is the settle's, and it adds that itself on a
    // replay as it does here).
    std::vector<std::string> errors;
    const bool ok = _CompileRig(&errors);
    if (diagnostics) {
        diagnostics->insert(diagnostics->end(), errors.begin(), errors.end());
    }
    if (ok) {
        _rigState.failedCompile = _FailedCompileMemo();
        return true;
    }
    // Keyed AFTER the compile, on what the next frame will compare against:
    // the compile authors its derived start frames under a notice block, so
    // the serial it leaves is the stage it read, and the mode is read the
    // same way the next settle reads it.
    _rigState.failedCompile.valid = true;
    _rigState.failedCompile.stageEditSerial = _stageEditSerial;

    _rigState.failedCompile.errors = std::move(errors);
    return false;
}

bool
RigExecRigEvaluator::_SettleCompiledRig(std::vector<std::string> *diagnostics)
{
    const uint64_t settleSerial = _stageEditSerial;
    if (!_rigState.Compiled() && !_CompileUnlessKnownBroken(diagnostics)) {
        return false;
    }
    // Recompile iff the topology digest changed; a failure memo with the same
    // serial answers first. An equal digest commits its recorded footprint. A
    // certainly-structural edit skips the digest here; the compile computes it.
    // RIGEXEC_VERIFY_CERTAIN_STRUCTURAL counts the false positives.
    bool recompiled = false;
    bool digestMoved = false;
    bool certain = false;
    bool replayed = false;
    if (_rigState.StructureDirty()) {
        replayed = _FailedCompileMemoMatches();
        digestMoved = replayed;
        if (!digestMoved) {
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Settle.CertainStructural",
                                          "evaluate");
                certain = _EditIsCertainlyStructural();
            }
            if (certain) {
                digestMoved = true;
            } else {
                _DigestGate gate;
                digestMoved = _SettleStructureDigest(&gate) != _structureDigest;
                if (!digestMoved) {
                    _rigState.digestGate = std::move(gate);
                }
            }
        }
    }
    if (digestMoved) {
        const size_t digestBefore = _structureDigest;
        _CertainStructuralCounts *const counts =
            _CertainStructuralVerifyCounts();
        if (counts && !replayed) {
            ++(certain ? counts->tagged : counts->digestFound);
        }
        if (!_CompileUnlessKnownBroken(diagnostics)) {
            return false;
        }
        recompiled = true;
        if (!certain || _structureDigest != digestBefore) {
            diagnostics->push_back("structural edit: compiled rig rebuilt");
        } else if (counts) {
            ++counts->falsePositives;
            std::fprintf(stderr,
                         "RIGEXEC_VERIFY_CERTAIN_STRUCTURAL: false positive "
                         "on <%s>: the edit taken for certainly structural "
                         "left the structure digest where it was\n",
                         _rigPath.GetText());
            std::fflush(stderr);
        }
    }
    if (_stageEditSerial == settleSerial) {
        _rigState.Transition(_CompiledRigState::Edge::Settled);
    }
    _restEditedProviders.clear();
    return true;
}

void
RigExecRigEvaluator::_CompiledRigState::Transition(Edge edge, bool dirty)
{
    switch (edge) {
    case Edge::Invalidated:
        _compiled = false;
        break;
    case Edge::CaptureMoved:
        _compiled = false;
        _structureDirty = true;
        break;
    case Edge::StructureSuspect:
        _structureDirty = true;
        break;
    case Edge::Committed:
        _compiled = true;
        break;
    case Edge::CommitJoined:
        assert(_compiled);
        _structureDirty = dirty;
        break;
    case Edge::Settled:
        assert(_compiled);
        _structureDirty = false;
        certainCandidates.clear();
        certainCandidatesOverflowed = false;
        break;
    }
}

} // namespace rigExec
