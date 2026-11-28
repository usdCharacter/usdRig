// Compile steps: the pose dependency graph and its provider closure.

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
    ((rigExecDriverCurveReadPhase, SchemaAttrText::rigExecDriverCurveReadPhase))
    ((rigExecSurfaceReadPhase, SchemaAttrText::rigExecSurfaceReadPhase))
    (points)
);

using namespace evaluatorDetail;

namespace {

// Attribute names compile rejects; interned once at load (registry lock).
const TfToken _removedRibbonPhases[] = {_tokens->rigExecDriverCurveReadPhase,
                                        _tokens->rigExecSurfaceReadPhase};

} // namespace

bool
RigExecRigEvaluator::_IsFrameProviderPath(const SdfPath &path) const
{
    const UsdPrim prim = _stage->GetPrimAtPath(path);
    return _IsFrameProvider(prim);
}

std::set<SdfPath>
RigExecRigEvaluator::_ComputePoseProviderClosure(_CompileAttempt &attempt,
                                                 const SdfPath &input)
{
    auto &newJointBinding = attempt.newJointBinding;
    auto &newPoseInputInfo = attempt.newPoseInputInfo;
    auto &poseInfoPrefetch = attempt.poseInfoPrefetch;
    auto &poseInputGraph = attempt.poseInputGraph;
    auto &isBoundJoint = attempt.isBoundJoint;
    std::set<SdfPath> closure;
    std::vector<SdfPath> pending{input};
    while (!pending.empty()) {
        const SdfPath path = pending.back();
        pending.pop_back();
        if (path.IsEmpty() || !closure.insert(path).second)
            continue;
        if (newJointBinding.count(path)) {
            newPoseInputInfo[path];
            continue;
        }
        auto it = newPoseInputInfo.find(path);
        if (it == newPoseInputInfo.end()) {
            // Same value either way: the prefetch ran the same function
            // on the same prim of the same stage. Only newPoseInputInfo
            // records that this path was REACHED.
            // MOVED out, not copied: a path lands in newPoseInputInfo
            // at most once (this branch is the find() miss), so nothing
            // reads the prefetch entry again, and leaving the sets
            // behind would hold every closure's inputs on the heap
            // TWICE for the rest of compile, and the batch pass that runs
            // after it would pay for that in allocator and locality cost.
            // A prim the prefetch did not ask about is asked about now,
            // through the same graph, so every closure of this compile
            // materializes its attributes the same way.
            auto warm = poseInfoPrefetch.find(path);
            if (warm == poseInfoPrefetch.end()) {
                poseInputGraph->Extend(_stage, {path}, isBoundJoint,
                                       RigExecParallelEvaluationEnabled(),
                                       _profiler, &poseInfoPrefetch);
                warm = poseInfoPrefetch.find(path);
            }
            it = newPoseInputInfo
                     .emplace(path, warm != poseInfoPrefetch.end()
                                        ? std::move(warm->second)
                                        : _CollectPoseInputInfo(
                                              _stage->GetPrimAtPath(path)))
                     .first;
        }
        pending.insert(pending.end(), it->second.providers.begin(),
                       it->second.providers.end());
        if (!_IsFrameProviderPath(path)) {
            const UsdPrim parent =
                _NamespaceFrameProvider(_stage->GetPrimAtPath(path));
            if (parent)
                pending.push_back(parent.GetPath());
        }
    }
    return closure;
}

const std::set<SdfPath> &
RigExecRigEvaluator::_PoseProviderClosure(_CompileAttempt &attempt,
                                          const SdfPath &input)
{
    auto &poseClosureCache = attempt.poseClosureCache;
    auto it = poseClosureCache.find(input);
    if (it == poseClosureCache.end()) {
        it = poseClosureCache
                 .emplace(input, _ComputePoseProviderClosure(attempt, input))
                 .first;
    }
    return it->second;
}

bool
RigExecRigEvaluator::_CompilePoseDag(_CompileAttempt &attempt)
{
    auto &newJointPaths = attempt.newJointPaths;
    auto &newControlPaths = attempt.newControlPaths;
    auto &newJointBinding = attempt.newJointBinding;
    auto &skeleton2dSwitchSources = attempt.skeleton2dSwitchSources;
    auto &jointWritingSolvers = attempt.jointWritingSolvers;
    auto &poseReverseEdges = attempt.poseReverseEdges;
    auto &newSolverDependencies = attempt.newSolverDependencies;
    auto &newFrameConstraints = attempt.newFrameConstraints;
    // Compile one pose DAG. Constraints retain authored preceding order;
    // solvers read the final constrained state of each declared frame input.
    // Bound joints replace their namespace callback, cutting ancestor edges.
    auto &newPoseInputInfo = attempt.newPoseInputInfo;
    // A stage closure that the schedule and request passes below ask for
    // over and over about the same prims: the pose-provider closure of one
    // input path. The same joint is an input to many solvers, many
    // constraints and many batches, and each ask re-walks the whole chain to
    // the rig root.
    // It is a pure function of the composed stage and of the joint binding
    // this compile has already decided above; neither changes while a compile
    // runs, so a memoized answer is the answer a recomputation would give.
    // Compile-local; must not share the digest's caches.
    auto &poseClosureCache = attempt.poseClosureCache;
    // The per-prim half of the pose closure, computed ahead of the walks
    // that need it by the pose-input graph, for every prim those walks will
    // ask about (see the prefetch below). A prim's closure is a pure read of
    // the composed stage, so its answer does not depend on when it is
    // computed, which is what lets the walk below take one out of here
    // instead of paying for it in line.
    // Separate from newPoseInputInfo, which must hold only reached prims.
    // This one may overshoot.
    auto &poseInfoPrefetch = attempt.poseInfoPrefetch;
    // Shared, not local: a deferred compiled rig hands it to
    // _RealizeDeferredExecPrep, which materializes the attribute sets the
    // solver-input index reads from it.
    auto &poseInputGraph = attempt.poseInputGraph;
    poseInputGraph = std::make_shared<_PoseInputGraph>();
    auto &isBoundJoint = attempt.isBoundJoint;
    isBoundJoint = [&](const SdfPath &path) {
        return newJointBinding.count(path) != 0;
    };
    auto &solverFrameInputs = attempt.solverFrameInputs;
    // The read phase a solver declares on an input relationship, through the
    // same optional rigExecReadPhase metadata a mover input uses. Without
    // one the hierarchy decides: a constraint ABOVE the solver
    // revises what the solver already read. "final", or a prim path at or
    // after that constraint, asks for the frame as the constraint left it,
    // and the solver then waits on it (the frame-inheritance walk below).
    // Base and preceding are the hierarchy rule and are not stored.
    std::map<SdfPath, std::map<SdfPath, std::vector<RigExecReadPhase>>>
        solverInputPhases;
    for (const auto &[solver, dependencies] : newSolverDependencies) {
        const UsdPrim prim = _stage->GetPrimAtPath(solver);
        // Read-phase attributes are refused; read phase is metadata.
        for (const TfToken &removed : _removedRibbonPhases) {
            const UsdAttribute old = prim.GetAttribute(removed);
            if (old && old.HasAuthoredValue()) {
                return attempt.Fail(
                    solver.GetString() + " authors " + removed.GetString() +
                        ", which was replaced by rigExecReadPhase "
                        "metadata on the input relationship",
                    {solver});
            }
        }
        for (const UsdRelationship &rel : prim.GetRelationships()) {
            if (rel.GetName() == SchemaAttrText::rigExecJoints) {
                continue;
            }
            RigExecReadPhase phase;
            std::string phaseError;
            if (!RigExecResolveReadPhase(rel, &phase, &phaseError)) {
                return attempt.Fail(solver.GetString() + " " +
                                        rel.GetName().GetString() + ": " +
                                        phaseError,
                                    {solver});
            }
            const bool phased = phase.kind == RigExecReadPhaseKind::Final ||
                                phase.kind == RigExecReadPhaseKind::AtPrim;
            SdfPathVector targets;
            rel.GetTargets(&targets);
            attempt.AppendDerivedStart(rel, &targets);
            for (const SdfPath &target : targets) {
                // Ribbon driverCurve is a geometry value, not a frame
                // relationship. Its exact Base/Preceding/AtPrim/Final points
                // producer is bound by FinalizeCrossDomainReads.
                if (prim.GetTypeName() == SchemaTypeName::RigExecRibbon &&
                    rel.GetName() == SchemaAttrText::rigExecDriverCurve) {
                    const SdfPath points =
                        target.IsPrimPath()
                            ? target.AppendProperty(_tokens->points)
                            : target;
                    const UsdAttribute input =
                        _stage->GetAttributeAtPath(points);
                    if (phased &&
                        (!input ||
                         input.GetTypeName().GetType() !=
                             SdfValueTypeNames->Float3Array.GetType())) {
                        return attempt.Fail(
                            solver.GetString() +
                                " rigExec:driverCurve declares "
                                "rigExecReadPhase '" +
                                phase.GetAsString() + "' on " +
                                target.GetString() +
                                ", which is not a Vec3f array points source",
                            {solver});
                    }
                    continue;
                }
                if (_IsFrameProviderPath(target.GetPrimPath())) {
                    solverFrameInputs[solver].insert(target.GetPrimPath());
                    if (phased) {
                        solverInputPhases[solver][target.GetPrimPath()]
                            .push_back(phase);
                    }
                } else if (phased) {
                    // A solver reads a phase only through a provider's
                    // frame chain; anything else would read base silently.
                    return attempt.Fail(
                        solver.GetString() + " " + rel.GetName().GetString() +
                            " declares rigExecReadPhase '" +
                            phase.GetAsString() + "' on " + target.GetString() +
                            ", which is not a frame provider; a "
                            "solver reads a phase only from a "
                            "control or joint",
                        {solver});
                }
            }
        }
        // A 2D skeleton's outside switch sources: frame inputs read at base.
        const auto switchSources = skeleton2dSwitchSources.find(solver);
        if (switchSources != skeleton2dSwitchSources.end()) {
            for (const SdfPath &source : switchSources->second) {
                if (_IsFrameProviderPath(source.GetPrimPath())) {
                    solverFrameInputs[solver].insert(source.GetPrimPath());
                }
            }
        }
    }
    // Pose closures precomputed level-parallel (_PoseInputGraph); walks below
    // do set ops only. Seeded with every path a closure will be asked for plus
    // its frame-provider ancestors; a missed prim is computed in line.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PoseInfoPrefetch", "compile");
        std::vector<SdfPath> seeds;
        const auto seed = [&](const SdfPath &path) {
            if (path.IsEmpty())
                return;
            seeds.push_back(path);
            // Only the FRAME-PROVIDER ancestors, because those are the ones
            // seedProvider puts in newNativeProviderPaths and ProviderClosure
            // then asks about. Pushing the whole namespace chain instead
            // reads a large part of the stage that no walk ever asks for,
            // and the allocation that costs lands on every pass after it.
            for (SdfPath a = path.GetParentPath();
                 !a.IsEmpty() && a != SdfPath::AbsoluteRootPath();
                 a = a.GetParentPath()) {
                if (_IsFrameProviderPath(a))
                    seeds.push_back(a);
            }
        };
        for (const auto &[solver, inputs] : solverFrameInputs) {
            for (const SdfPath &input : inputs)
                seed(input);
        }
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            for (const SdfPath &target : constraint.targets)
                seed(target);
            for (const auto &source : constraint.sources) {
                seed(source.sourcePath);
            }
            seed(constraint.worldUpObject.sourcePath);
            seed(constraint.spacePath);
            seed(constraint.effector.sourcePath);
            seed(constraint.weightObject);
            for (const auto &pole : constraint.poleObjects) {
                seed(pole.sourcePath);
            }
        }
        for (const SdfPath &path : newJointPaths)
            seed(path);
        for (const SdfPath &path : newControlPaths)
            seed(path);
        // A joint-bound path stops the real walk without being read, so the
        // graph does not ask about it either.
        poseInputGraph->Extend(_stage, seeds, isBoundJoint,
                               RigExecParallelEvaluationEnabled(), _profiler,
                               &poseInfoPrefetch);
    }
    auto &solverPoseReads = attempt.solverPoseReads;
    // solverInputPhases spread over each input's pose closure, so the walk
    // that visits closure members finds the phase of the input that reached
    // them.
    auto &solverClosurePhases = attempt.solverClosurePhases;
    for (const auto &[solver, inputs] : solverFrameInputs) {
        const auto declared = solverInputPhases.find(solver);
        for (const SdfPath &input : inputs) {
            const std::set<SdfPath> &closure =
                _PoseProviderClosure(attempt, input);
            solverPoseReads[solver].insert(closure.begin(), closure.end());
            if (declared != solverInputPhases.end()) {
                const auto phases = declared->second.find(input);
                if (phases != declared->second.end()) {
                    for (const SdfPath &member : closure) {
                        std::vector<RigExecReadPhase> &into =
                            solverClosurePhases[solver][member];
                        into.insert(into.end(), phases->second.begin(),
                                    phases->second.end());
                    }
                }
            }
            for (const SdfPath &provider : closure) {
                const auto owner = newJointBinding.find(provider);
                if (owner != newJointBinding.end()) {
                    // Every writer BELOW this solver in the stack, never this
                    // solver itself -- the same rule the direct relationship
                    // walk applies above, and the edge class that catches an
                    // indirect read (an IK control parented under a joint, a
                    // startFrameObject reached through a namespace parent).
                    // A writer ABOVE it gets the mirror edge instead: the
                    // solver read the version standing before that write, so
                    // the write has to wait.
                    for (const auto &[writer, writerElement] : owner->second) {
                        if (writer == solver) {
                            continue;
                        }
                        if (jointWritingSolvers.count(solver) &&
                            attempt.StackBefore(solver, writer)) {
                            poseReverseEdges.emplace_back(writer, solver);
                            continue;
                        }
                        newSolverDependencies[solver].insert(writer);
                    }
                }
            }
        }
    }
    // The solver DAG is complete here, but the POSE graph is not: the
    // constraint pass and the frame-inheritance walk below still add
    // solver -> constraint -> solver edges, and a pair those order is a pair
    // the stack must not order differently. So the stack order is settled
    // ONCE, against the finished graph, just before the schedule is built --
    // see "the stack order" block below SolverSchedule.FrameInheritance.
    auto &newSolverJoints = attempt.newSolverJoints;
    auto &requiredSolvers = attempt.requiredSolvers;
    for (const auto &[joint, writers] : newJointBinding) {
        for (const auto &[solver, element] : writers) {
            newSolverJoints[solver].emplace_back(joint, element);
            requiredSolvers.insert(solver);
        }
    }
    // The solver schedule's aggregate-consumer work, profiled as five blocks
    // because the passes are quite different work -- set closure, constraint
    // reads through the pose closure, an ancestor walk, a counting pass, and
    // the batch BFS that talks to exec.
    return true;
}

void
RigExecRigEvaluator::_CompileRequiredSolvers(_CompileAttempt &attempt)
{
    auto &newSolverDependencies = attempt.newSolverDependencies;
    auto &newGraphChains = attempt.newGraphChains;
    auto &newGraphDerivedChains = attempt.newGraphDerivedChains;
    auto &requiredSolvers = attempt.requiredSolvers;
    // Geometry aggregate consumers are authoritative even without joints.
    for (const auto &[target, revisions] : newGraphChains) {
        for (const _GraphRevision &revision : revisions) {
            if (newSolverDependencies.count(revision.binding.driverFrames)) {
                requiredSolvers.insert(revision.binding.driverFrames);
            }
        }
    }
    for (const auto &[target, revisions] : newGraphDerivedChains) {
        for (const _GraphRevision &revision : revisions) {
            if (newSolverDependencies.count(revision.binding.driverFrames))
                requiredSolvers.insert(revision.binding.driverFrames);
        }
    }
    std::vector<SdfPath> pendingSolvers(requiredSolvers.begin(),
                                        requiredSolvers.end());
    while (!pendingSolvers.empty()) {
        const SdfPath solver = pendingSolvers.back();
        pendingSolvers.pop_back();
        for (const SdfPath &dependency : newSolverDependencies[solver]) {
            if (requiredSolvers.insert(dependency).second) {
                pendingSolvers.push_back(dependency);
            }
        }
    }
    auto &poseDependencies = attempt.poseDependencies;
    auto &constraintIndices = attempt.constraintIndices;
    for (const SdfPath &solver : requiredSolvers) {
        poseDependencies[solver] = newSolverDependencies[solver];
    }
}

void
RigExecRigEvaluator::_CompileConstraintDependencies(_CompileAttempt &attempt)
{
    auto &newJointBinding = attempt.newJointBinding;
    auto &poseReverseEdges = attempt.poseReverseEdges;
    auto &newFrameConstraints = attempt.newFrameConstraints;
    auto &poseStackOrdinal = attempt.poseStackOrdinal;
    auto &poseDependencies = attempt.poseDependencies;
    auto &constraintIndices = attempt.constraintIndices;
    // Collected in the constraint loop, consumed by the upward walk just
    // below it. See the comment there for why the cross product this
    // replaces had to be turned inside out.
    auto &constraintsByTarget = attempt.constraintsByTarget;
    // Constraint -> constraint frame edges. A frame constraint writes its
    // target's final frame and, through commitConstraintFrames, every
    // namespace descendant up to an independently solved joint; a native
    // Xform source rides its closest revised provider ancestor with no such
    // stop. Pose-domain writers by target, as mover indices.
    std::map<SdfPath, std::vector<size_t>> constraintFrameWriters;
    for (size_t i = 0; i < newFrameConstraints.size(); ++i) {
        const _FrameConstraint &constraint = newFrameConstraints[i];
        if (!constraint.pointsTarget.IsEmpty())
            continue;
        for (const SdfPath &target : constraint.targets) {
            constraintFrameWriters[target].push_back(i);
        }
    }
    // The constraints whose commit can reach \p path: writers of the path
    // and of each namespace ancestor, the walk ending AFTER the first
    // solver-bound joint (its own writers count; nothing above it propagates
    // through it). \p throughJoints drops that stop, for a native source.
    // Authored parent:space also stops propagation, but per frame, so the
    // compile keeps the edge. Memoized; a pure function of the binding.
    std::map<std::pair<SdfPath, bool>, std::vector<size_t>>
        constraintWritersReaching;
    const auto writersReaching =
        [&](const SdfPath &path,
            bool throughJoints) -> const std::vector<size_t> & {
        const auto key = std::make_pair(path, throughJoints);
        const auto found = constraintWritersReaching.find(key);
        if (found != constraintWritersReaching.end()) {
            return found->second;
        }
        std::vector<size_t> writers;
        for (SdfPath p = path; !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
             p = p.GetParentPath()) {
            const auto it = constraintFrameWriters.find(p);
            if (it != constraintFrameWriters.end()) {
                writers.insert(writers.end(), it->second.begin(),
                               it->second.end());
            }
            if (!throughJoints && newJointBinding.count(p))
                break;
        }
        return constraintWritersReaching.emplace(key, std::move(writers))
            .first->second;
    };
    for (size_t i = 0; i < newFrameConstraints.size(); ++i) {
        const _FrameConstraint &constraint = newFrameConstraints[i];
        const SdfPath &path = constraint.moverPath;
        constraintIndices[path] = i;
        auto &dependencies = poseDependencies[path];
        // A constraint is ordered only by the frames it reads and writes:
        // the edges below, plus the per-joint writer chain built after the
        // frame-inheritance walk. Mover order adds no edge of its own, so
        // constraints that share no frame share a Kahn level and may run
        // concurrently.
        // Every writer of the frame, not just the last one, DIRECTIONALLY.
        // A frame read is POSITIONAL: the
        // constraint reads the version standing at its own place in the
        // unified pose stack. A writer BELOW it must therefore run first, and
        // a writer ABOVE it must wait, or which version was read is undefined
        // and the schedule decides it by accident. A missed edge here does not
        // fail loudly; it silently reads the wrong version.
        const bool positional = poseStackOrdinal.count(path) > 0;
        const int here = attempt.StackOrdinalOf(path);
        // Constraint writers of what this one reads, each ordered once, by
        // the same positional rule as a solver writer. A geometry-domain
        // constraint has no ordinal and keeps the place its mover index gives
        // it: after every earlier frame writer of what it reads, before every
        // later one.
        std::set<size_t> orderedWriters;
        const auto orderAgainst = [&](const std::vector<size_t> &writers) {
            for (const size_t w : writers) {
                if (w == i || !orderedWriters.insert(w).second)
                    continue;
                const SdfPath &writer = newFrameConstraints[w].moverPath;
                const bool above =
                    positional ? attempt.StackOrdinalOf(writer) > here : w > i;
                if (above) {
                    poseReverseEdges.emplace_back(writer, path);
                } else {
                    dependencies.insert(writer);
                }
            }
        };
        auto dependOnFrame = [&](const SdfPath &input,
                                 bool nativeSource = false) {
            for (const SdfPath &provider :
                 _PoseProviderClosure(attempt, input)) {
                const auto owner = newJointBinding.find(provider);
                if (owner != newJointBinding.end()) {
                    for (const auto &[writer, element] : owner->second) {
                        if (positional &&
                            attempt.StackOrdinalOf(writer) > here) {
                            poseReverseEdges.emplace_back(writer, path);
                            continue;
                        }
                        dependencies.insert(writer);
                    }
                }
                orderAgainst(writersReaching(provider, false));
            }
            if (nativeSource) {
                orderAgainst(writersReaching(input, true));
            }
        };
        const auto dependOnSource = [&](const _FrameSourceBinding &source) {
            dependOnFrame(source.sourcePath, !source.xformPath.IsEmpty());
        };
        for (const SdfPath &target : constraint.targets)
            dependOnFrame(target);
        for (const auto &source : constraint.sources)
            dependOnSource(source);
        dependOnSource(constraint.worldUpObject);
        // rigExec:space is read from the seed (compose) table, never from a
        // version the walk writes, so it orders nothing.
        dependOnSource(constraint.effector);
        dependOnFrame(constraint.weightObject);
        for (const auto &pole : constraint.poleObjects)
            dependOnSource(pole);
        // Geometry-domain constraints write only their own per-mover delta,
        // which the revision chains consume after the walk in mover order,
        // so two on one points target need no edge between them.
        // A geometry-domain constraint revises points, not the frame a solver
        // reads, so no solver can inherit one: this `continue` is the same
        // exclusion the ancestor walk below relies on, and it has to stay
        // exactly that strict.
        if (!constraint.pointsTarget.IsEmpty()) {
            continue;
        }
        for (const SdfPath &target : constraint.targets) {
            constraintsByTarget[target].push_back(path);
        }
    }
}

bool
RigExecRigEvaluator::_CompileFrameInheritance(_CompileAttempt &attempt)
{
    auto &newJointBinding = attempt.newJointBinding;
    auto &jointWritingSolvers = attempt.jointWritingSolvers;
    auto &poseReverseEdges = attempt.poseReverseEdges;
    auto &newFrameConstraints = attempt.newFrameConstraints;
    auto &poseStackOrdinal = attempt.poseStackOrdinal;
    auto &solverPoseReads = attempt.solverPoseReads;
    auto &solverClosurePhases = attempt.solverClosurePhases;
    auto &requiredSolvers = attempt.requiredSolvers;
    auto &poseDependencies = attempt.poseDependencies;
    auto &constraintsByTarget = attempt.constraintsByTarget;
    // Which solvers wait on which frame constraints: for each pose a solver
    // reads, walk up to the rig root collecting the constraints on each
    // ancestor. input inherits target's frame iff input.HasPrefix(target) and
    // no joint binding in [input, target); hence the binding check follows
    // recording p's own constraints.
    // The last stack ordinal at or beneath a prim: the moment an AtPrim read
    // phase names, since the stack runs a scope's contents before the scope.
    // -1 when nothing in the stack is there, which no constraint precedes.
    std::map<SdfPath, int> phasePrimOrdinals;
    const auto phaseOrdinal = [&](const SdfPath &prim) {
        const auto found = phasePrimOrdinals.find(prim);
        if (found != phasePrimOrdinals.end()) {
            return found->second;
        }
        int last = -1;
        for (const auto &[step, ordinal] : poseStackOrdinal) {
            if (step.HasPrefix(prim)) {
                last = std::max(last, ordinal);
            }
        }
        phasePrimOrdinals.emplace(prim, last);
        return last;
    };
    // Whether \p solver declared that it reads \p input as of \p constraint
    // or later: a "final" phase, or an AtPrim phase at or after it.
    const auto readsAfter = [&](const SdfPath &solver, const SdfPath &input,
                                const SdfPath &constraint) {
        const auto bySolver = solverClosurePhases.find(solver);
        if (bySolver == solverClosurePhases.end()) {
            return false;
        }
        const auto phases = bySolver->second.find(input);
        if (phases == bySolver->second.end()) {
            return false;
        }
        const int at = attempt.StackOrdinalOf(constraint);
        for (const RigExecReadPhase &phase : phases->second) {
            if (phase.kind == RigExecReadPhaseKind::Final ||
                (phase.kind == RigExecReadPhaseKind::AtPrim &&
                 phaseOrdinal(phase.prim) >= at)) {
                return true;
            }
        }
        return false;
    };
    if (!constraintsByTarget.empty()) {
        for (const SdfPath &solver : requiredSolvers) {
            for (const SdfPath &input : solverPoseReads[solver]) {
                for (SdfPath p = input;
                     !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
                     p = p.GetParentPath()) {
                    const auto it = constraintsByTarget.find(p);
                    if (it != constraintsByTarget.end()) {
                        // Directional for the same reason dependOnFrame is:
                        // a constraint ABOVE this solver revises what the
                        // solver already read, so the solver must NOT wait
                        // on it -- the constraint waits on the solver
                        // instead. (Producers have no position and keep the
                        // unconditional edge.) A solver input that declares
                        // a later read phase is the exception, and the only
                        // one: it reads the frame as the constraint left it.
                        const bool positional =
                            poseStackOrdinal.count(solver) > 0;
                        const int here = attempt.StackOrdinalOf(solver);
                        for (const SdfPath &constraint : it->second) {
                            const bool above =
                                positional &&
                                attempt.StackOrdinalOf(constraint) > here;
                            const bool declared =
                                above && readsAfter(solver, input, constraint);
                            if (declared) {
                                // The solver also writes the joint this
                                // constraint moves, and the stack orders the
                                // two writers by position: a declared read of
                                // the later one would be a cycle.
                                const auto writers = newJointBinding.find(p);
                                if (writers != newJointBinding.end()) {
                                    for (const auto &[writer, element] :
                                         writers->second) {
                                        if (writer != solver)
                                            continue;
                                        return attempt.Fail(
                                            solver.GetString() +
                                                " declares a read phase that "
                                                "reads " +
                                                p.GetString() + " after " +
                                                constraint.GetString() +
                                                ", but it also writes that "
                                                "joint and executes before "
                                                "the constraint; move the "
                                                "constraint below the solver "
                                                "instead (spec §4.2)",
                                            {solver});
                                    }
                                }
                            }
                            if (above && !declared) {
                                poseReverseEdges.emplace_back(constraint,
                                                              solver);
                                continue;
                            }
                            poseDependencies[solver].insert(constraint);
                        }
                    }
                    if (newJointBinding.count(p)) {
                        break;
                    }
                }
            }
        }
    }
    // poseDependencies is complete here: the constraint pass and the
    // frame-inheritance walk added the solver <-> constraint edges. Order =
    // poseStackOrdinal (namespace pre-order); data flow does not reorder. Per
    // joint: sort writers by poseStackOrdinal, link adjacent pairs. No
    // cross-joint edges, so unrelated steps share a Kahn level. Tested by
    // testRigExecSolverStacking.
    std::map<SdfPath, std::vector<SdfPath>> jointWriterChain;
    /// solver -> (joint -> the step that wrote the joint just before it), the
    /// compile-side half of "the incoming frame is the solver's rest".
    auto &solverRestPredecessor = attempt.solverRestPredecessor;
    {
        for (auto &[joint, writers] : newJointBinding) {
            if (writers.size() < 2)
                continue;
            std::stable_sort(writers.begin(), writers.end(),
                             [&attempt](const std::pair<SdfPath, int> &a,
                                        const std::pair<SdfPath, int> &b) {
                                 return attempt.StackOrdinalOf(a.first) <
                                        attempt.StackOrdinalOf(b.first);
                             });
        }
        // Which pose-domain constraints move each provider. Geometry-domain
        // constraints are excluded: they revise points, carry no stack
        // position, and are not writers of a frame.
        std::map<SdfPath, std::vector<SdfPath>> frameWritingConstraints;
        for (const _FrameConstraint &constraint : newFrameConstraints) {
            if (!constraint.pointsTarget.IsEmpty())
                continue;
            if (!poseStackOrdinal.count(constraint.moverPath))
                continue;
            for (const SdfPath &target : constraint.targets) {
                frameWritingConstraints[target].push_back(constraint.moverPath);
            }
        }
        std::set<SdfPath> written;
        for (const auto &[joint, writers] : newJointBinding) {
            written.insert(joint);
        }
        for (const auto &[target, constraints] : frameWritingConstraints) {
            written.insert(target);
        }
        for (const SdfPath &joint : written) {
            std::vector<std::pair<int, SdfPath>> chain;
            const auto solvers = newJointBinding.find(joint);
            if (solvers != newJointBinding.end()) {
                for (const auto &[writer, element] : solvers->second) {
                    chain.emplace_back(attempt.StackOrdinalOf(writer), writer);
                }
            }
            const auto constraints = frameWritingConstraints.find(joint);
            if (constraints != frameWritingConstraints.end()) {
                for (const SdfPath &constraint : constraints->second) {
                    chain.emplace_back(attempt.StackOrdinalOf(constraint),
                                       constraint);
                }
            }
            std::sort(chain.begin(), chain.end());
            std::vector<SdfPath> &ordered = jointWriterChain[joint];
            for (const auto &[ordinal, step] : chain) {
                ordered.push_back(step);
            }
            for (size_t i = 1; i < ordered.size(); ++i) {
                if (ordered[i] == ordered[i - 1])
                    continue;
                poseDependencies[ordered[i]].insert(ordered[i - 1]);
            }
            // "The incoming frame replaces the authored rest." For each
            // solver in the chain, the step immediately before it that wrote
            // this joint -- which is the frame that solver measures from.
            for (size_t i = 1; i < ordered.size(); ++i) {
                if (!jointWritingSolvers.count(ordered[i]))
                    continue;
                const auto owner = newJointBinding.find(joint);
                bool writesIt = false;
                if (owner != newJointBinding.end()) {
                    for (const auto &[writer, element] : owner->second) {
                        writesIt = writesIt || writer == ordered[i];
                    }
                }
                if (!writesIt)
                    continue;
                solverRestPredecessor[ordered[i]][joint] = ordered[i - 1];
            }
        }
    }
    // The single-chain IK constraint is the one CONSTRAINT that measures
    // from joint rests, so the same substitution reaches it: a joint a step
    // below it wrote hands it that step's frame as the rest reference.
    for (_FrameConstraint &constraint : newFrameConstraints) {
        if (constraint.ikChain.empty())
            continue;
        constraint.ikRestLive.assign(constraint.ikChain.size(), 0);
        for (size_t i = 0; i < constraint.ikChain.size(); ++i) {
            const auto it = jointWriterChain.find(constraint.ikChain[i]);
            if (it == jointWriterChain.end())
                continue;
            for (size_t k = 0; k < it->second.size(); ++k) {
                if (it->second[k] == constraint.moverPath) {
                    constraint.ikRestLive[i] = k > 0 ? 1 : 0;
                    break;
                }
            }
        }
    }
    // The reverse edges -- "a writer waits on a reader standing below it" --
    // collected wherever a read was resolved. Applied here, once, because
    // poseDependencies is only complete now and because an edge inserted into
    // it earlier would have been read back as a dependency by the passes
    // above.
    for (const auto &[waiter, waitedOn] : poseReverseEdges) {
        if (waiter == waitedOn)
            continue;
        if (!poseDependencies.count(waiter))
            continue;
        if (!poseDependencies.count(waitedOn))
            continue;
        poseDependencies[waiter].insert(waitedOn);
    }
    return true;
}

void
RigExecRigEvaluator::_SeedRestCompiledRigProvider(_CompileAttempt &attempt,
                                                  SdfPath path)
{
    auto &newRestCompiledRigProviders = attempt.newRestCompiledRigProviders;
    for (; !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
         path = path.GetParentPath()) {
        if (newRestCompiledRigProviders.count(path))
            break;
        if (!_IsFrameProviderPath(path))
            continue;
        newRestCompiledRigProviders.insert(path);
    }
}

void
RigExecRigEvaluator::_SeedProvider(_CompileAttempt &attempt, SdfPath path,
                                   bool restCompiledRig)
{
    auto &newNativeProviderPaths = attempt.newNativeProviderPaths;
    if (restCompiledRig)
        _SeedRestCompiledRigProvider(attempt, path);
    for (; !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
         path = path.GetParentPath()) {
        if (newNativeProviderPaths.count(path))
            break;
        if (!_IsFrameProviderPath(path))
            continue;
        newNativeProviderPaths.insert(path);
    }
}

void
RigExecRigEvaluator::_CompilePoseReady(_CompileAttempt &attempt)
{
    auto &newJointPaths = attempt.newJointPaths;
    auto &newControlPaths = attempt.newControlPaths;
    auto &newVolumeWeightProviders = attempt.newVolumeWeightProviders;
    auto &newFrameConstraints = attempt.newFrameConstraints;
    auto &newGraphChains = attempt.newGraphChains;
    auto &newGraphDerivedChains = attempt.newGraphDerivedChains;
    auto &newPoseInputInfo = attempt.newPoseInputInfo;
    auto &solverFrameInputs = attempt.solverFrameInputs;
    using NativePoseOwner = _CompileAttempt::NativePoseOwner;
    auto &nativePoseOwners = attempt.nativePoseOwners;
    auto &newSolverBatches = attempt.newSolverBatches;
    // Provider identities include every required namespace ancestor.
    auto &newNativeProviderPaths = attempt.newNativeProviderPaths;
    auto &newRestCompiledRigProviders = attempt.newRestCompiledRigProviders;
    // Preserve the already-reached connected-read keys, without seeding their
    // namespace ancestors until PrepareRequests.ProviderClosure.
    auto &restLatePoseInputs = attempt.restLatePoseInputs;
    for (const auto &entry : newPoseInputInfo)
        restLatePoseInputs.insert(entry.first);
    for (const SdfPath &path : newJointPaths)
        _SeedProvider(attempt, path);
    for (const SdfPath &path : newControlPaths)
        _SeedProvider(attempt, path);
    for (const SdfPath &path : newVolumeWeightProviders)
        _SeedProvider(attempt, path);
    const auto seedGeometryProviders = [&](const auto &chains) {
        for (const auto &[target, revisions] : chains)
            for (const _GraphRevision &revision : revisions) {
                _SeedProvider(attempt, revision.binding.transform, false);
                _SeedProvider(attempt, revision.binding.transformSpace, false);
                _SeedProvider(attempt, revision.binding.carrySpace, false);
                for (const SdfPath &provider : revision.binding.influences)
                    _SeedProvider(attempt, provider, false);
            }
    };
    seedGeometryProviders(newGraphChains);
    seedGeometryProviders(newGraphDerivedChains);
    for (const auto &[solver, frames] : solverFrameInputs) {
        for (const SdfPath &path : frames)
            _SeedProvider(attempt, path);
    }
    for (const _FrameConstraint &constraint : newFrameConstraints) {
        for (const SdfPath &target : constraint.targets)
            _SeedProvider(attempt, target);
        for (const auto &source : constraint.sources)
            _SeedProvider(attempt, source.sourcePath);
        _SeedProvider(attempt, constraint.worldUpObject.sourcePath);
        _SeedProvider(attempt, constraint.spacePath);
        _SeedProvider(attempt, constraint.effector.sourcePath);
        for (const auto &pole : constraint.poleObjects)
            _SeedProvider(attempt, pole.sourcePath);
    }
}

} // namespace rigExec
