// Compile steps: solver joint binding and the solver dependency graph.

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
    ((rigExecUpperLength, SchemaAttrText::rigExecUpperLength))
    ((rigExecLowerLength, SchemaAttrText::rigExecLowerLength))
    ((rigExecCount, SchemaAttrText::rigExecCount))
    ((rigExecWeights, SchemaAttrText::rigExecWeights))
    ((rigExecSampleCount, SchemaAttrText::rigExecSampleCount))
    ((rigExecVolumeWeights, SchemaAttrText::rigExecVolumeWeights))
    ((rigExecJoints, SchemaAttrText::rigExecJoints))
    ((rigExecJointElements, SchemaAttrText::rigExecJointElements))
    ((rigExecControls, SchemaAttrText::rigExecControls))
    ((posedSpace, SchemaAttrText::posedSpace))
);

using namespace evaluatorDetail;

bool
RigExecRigEvaluator::_CompileBindSolverJoints(_CompileAttempt &attempt)
{
    auto &rig = attempt.rig;
    auto &aggregateSolvers = attempt.aggregateSolvers;
    // Validates solver->joint bindings before teardown, so a bad binding keeps
    // the previous compiled rig. Authored-conflict checks read _stage.
    // Value: joint -> ordered writer stack (solver, element); index 0 writes
    // first, the last supplies the base frame. Ordered once the pose graph is
    // complete. orderedSolvers is walked once and shared by four passes.
    const std::vector<UsdPrim> &orderedSolvers = aggregateSolvers;
    // Solver -> its position in the SOLVER STACK ORDINAL: the reverse of the
    // composed pre-order of the whole rig, which is _GetMoverExecutionOrder's
    // rule (see the comment at its definition) applied to the solver set
    // instead of the movers. Bottom composed sibling first, a parent
    // after its descendants -- so "the bottom one executes first" reads the
    // same whichever kind of node a rigger is looking at, and a reorder in
    // usdview reorders the stack.
    // It is a pre-order of the WHOLE rig, not a sibling order: two solvers in
    // different scopes are ordered by where their scopes sit, and a nested
    // solver comes before its ancestor. Must match the digest's solver walk
    // (the aggregate-solver segment of the digest).
    auto &solverStackOrdinal = attempt.solverStackOrdinal;
    {
        int ordinal = 0;
        for (auto it = orderedSolvers.rbegin(); it != orderedSolvers.rend();
             ++it) {
            solverStackOrdinal[it->GetPath()] = ordinal++;
        }
    }
    auto &newJointBinding = attempt.newJointBinding;
    {
        {
            const std::vector<UsdPrim> &solvers = orderedSolvers;
            // Cardinality attributes must be static (compile-time
            // structural) for EVERY aggregate solver under the rig, not
            // only joint-bearing ones: a non-joint Twist/Ribbon feeding a
            // joint-bearing Blend still determines that Blend's element
            // count, so a time-sampled cardinality would silently shift a
            // blend-bound joint's frame. `uniform` is only
            // a hint; reject samples explicitly.
            for (const UsdPrim &solver : solvers) {
                const TfToken t = solver.GetTypeName();
                // rigExec:upperLength/lowerLength are not schema attrs;
                // refuse (lengths come from joint rests).
                if (t == SchemaTypeName::RigExecTwoBoneIk) {
                    const TfToken lengthAttrs[2] = {
                        _tokens->rigExecUpperLength,
                        _tokens->rigExecLowerLength};
                    for (const TfToken &name : lengthAttrs) {
                        const UsdAttribute a = solver.GetAttribute(name);
                        if (a && a.HasAuthoredValueOpinion()) {
                            return attempt.Fail(
                                solver.GetPath().GetString() + ": " +
                                    name.GetString() +
                                    " was removed from the schema; bone "
                                    "lengths "
                                    "are measured from the bound joints' rest "
                                    "positions. Remove this opinion and "
                                    "author " +
                                    name.GetString() +
                                    "Offset to adjust the measured bone",
                                {solver.GetPath()});
                        }
                    }
                }
                std::vector<TfToken> cardinalityAttrs;
                if (t == SchemaTypeName::RigExecTwistDistribution) {
                    cardinalityAttrs = {_tokens->rigExecCount,
                                        _tokens->rigExecWeights};
                } else if (t == SchemaTypeName::RigExecRibbon) {
                    cardinalityAttrs = {_tokens->rigExecSampleCount};
                } else if (t == SchemaTypeName::RigExecSplineIk) {
                    // Not cardinality (the joints list is), but a static
                    // parallel array: a sampled one would let the
                    // per-joint weight silently detach from the chain.
                    cardinalityAttrs = {_tokens->rigExecVolumeWeights};
                }
                for (const TfToken &name : cardinalityAttrs) {
                    const UsdAttribute a = solver.GetAttribute(name);
                    if (a && a.GetNumTimeSamples() > 0) {
                        return attempt.Fail(
                            solver.GetTypeName().GetString() + " " +
                                solver.GetPath().GetString() + ": " +
                                name.GetString() +
                                " must not be time-sampled (it defines frame "
                                "cardinality)",
                            {solver.GetPath()});
                    }
                }
            }
            // Solvers whose aggregate is READ by another solver. Such a
            // solver does not pose anything itself -- the consumer that
            // reads it is what writes to the joints -- so its
            // rigExec:joints is a rest reference, not an output claim.
            // That is what lets an IK feeding an IK/FK blend name the
            // chain it solves for: the blend still claims those joints
            // exclusively, while the IK gets the joint REST frames it
            // needs to measure its bone lengths from. Without this the
            // IK could not name them at all ("posed by two solvers") and
            // had no bones to measure.
            std::set<SdfPath> consumedSolvers;
            std::set<SdfPath> posedByUnconsumed;
            {
                const std::vector<UsdPrim> &allSolvers = orderedSolvers;
                std::set<SdfPath> solverPaths;
                for (const UsdPrim &solver : allSolvers) {
                    solverPaths.insert(solver.GetPath());
                }
                for (const UsdPrim &solver : allSolvers) {
                    for (const UsdRelationship &rel :
                         solver.GetRelationships()) {
                        if (rel.GetName() == SchemaAttrText::rigExecJoints) {
                            continue;
                        }
                        SdfPathVector targets;
                        rel.GetTargets(&targets);
                        for (const SdfPath &target : targets) {
                            // GetPrimPath(), not the raw target: the DAG pass
                            // below resolves a property spelling
                            // (</Rig/Solvers/IK.rigExec:aggregate>) to its
                            // prim, and a read that creates a dependency edge
                            // but does not mark the target consumed would
                            // leave the IK writing as well as feeding the
                            // blend -- a stack whose meaning then depended on
                            // namespace order.
                            const SdfPath targetPrim = target.GetPrimPath();
                            if (_IsSkippedOperation(targetPrim)) {
                                return attempt.Fail(
                                    solver.GetPath().GetString() +
                                        " reads skipped operation " +
                                        targetPrim.GetString(),
                                    {solver.GetPath()});
                            }
                            if (targetPrim != solver.GetPath() &&
                                solverPaths.count(targetPrim)) {
                                consumedSolvers.insert(targetPrim);
                            }
                        }
                    }
                }
                // Joints that a solver nobody reads writes to. Only these
                // are already spoken for; a consumed solver still POSES
                // any joint no such solver names, so feeding a blend does
                // not silently stop it driving its own extra outputs.
                for (const UsdPrim &solver : allSolvers) {
                    if (consumedSolvers.count(solver.GetPath())) {
                        continue;
                    }
                    if (const UsdRelationship rel =
                            solver.GetRelationship(_tokens->rigExecJoints)) {
                        SdfPathVector targets;
                        rel.GetTargets(&targets);
                        posedByUnconsumed.insert(targets.begin(),
                                                 targets.end());
                    }
                }
            }

            // A claim binds wherever the solver sits; a non-solver prim
            // carrying rigExec:joints is rejected wherever it sits.
            if (const UsdPrim rig = _stage->GetPrimAtPath(_rigPath)) {
                for (const UsdPrim &solver : UsdPrimRange(rig)) {
                    if (_IsSkippedOperation(solver.GetPath())) {
                        continue;
                    }
                    const UsdRelationship jointsRel =
                        solver.GetRelationship(_tokens->rigExecJoints);
                    if (!jointsRel) {
                        continue;
                    }
                    SdfPathVector jointTargets;
                    jointsRel.GetTargets(&jointTargets);
                    if (jointTargets.empty()) {
                        continue;
                    }
                    const auto who = [&solver] {
                        return solver.GetTypeName().GetString() + " " +
                               solver.GetPath().GetString();
                    };

                    // The claimant must be a real aggregate solver (it must
                    // publish computePointFrameArray for the joint to extract).
                    // A Scope or arbitrary prim carrying rigExec:joints would
                    // otherwise pass, and the binding would name a prim with no
                    // aggregate computation to extract an element from.
                    if (!_IsAggregateSolverType(solver.GetTypeName())) {
                        return attempt.Fail(
                            who() + " authors rigExec:joints but is not a "
                                    "recognized aggregate solver type",
                            {solver.GetPath()});
                    }

                    // jointElements parallel-array shape: empty, or exactly one
                    // entry per joint (no partial remap / silent truncation).
                    VtIntArray jointElements;
                    if (const UsdAttribute a = solver.GetAttribute(
                            _tokens->rigExecJointElements)) {
                        std::vector<double> times;
                        if (a.GetTimeSamples(&times) && !times.empty()) {
                            return attempt.Fail(
                                who() + ": rigExec:jointElements must not "
                                        "carry time samples",
                                {solver.GetPath()});
                        }
                        a.Get(&jointElements);
                    }
                    if (!jointElements.empty() &&
                        jointElements.size() != jointTargets.size()) {
                        return attempt.Fail(
                            who() + ": rigExec:jointElements length " +
                                std::to_string(jointElements.size()) +
                                " must equal rigExec:joints length " +
                                std::to_string(jointTargets.size()),
                            {solver.GetPath()});
                    }

                    // Knowable aggregate element count per solver type
                    // (-1 = not cheaply knowable, e.g. a blend of another
                    // aggregate: element bounds are enforced only at runtime).
                    int knownCount = -1;
                    const TfToken type = solver.GetTypeName();
                    if (type == SchemaTypeName::RigExecTwoBoneIk) {
                        knownCount = 3;
                    } else if (type == SchemaTypeName::RigExecFkChain) {
                        SdfPathVector controls;
                        if (const UsdRelationship c = solver.GetRelationship(
                                _tokens->rigExecControls)) {
                            c.GetTargets(&controls);
                        }
                        knownCount = static_cast<int>(controls.size());
                    } else if (type ==
                               SchemaTypeName::RigExecTwistDistribution) {
                        // count/weights time samples already rejected above.
                        VtFloatArray weights;
                        if (const UsdAttribute a =
                                solver.GetAttribute(_tokens->rigExecWeights)) {
                            a.Get(&weights);
                        }
                        if (!weights.empty()) {
                            knownCount = static_cast<int>(weights.size());
                        } else if (const UsdAttribute a = solver.GetAttribute(
                                       _tokens->rigExecCount)) {
                            int c = 1;
                            a.Get(&c);
                            knownCount = std::max(c, 1);
                        }
                    } else if (type == SchemaTypeName::RigExecRibbon) {
                        if (const UsdAttribute a = solver.GetAttribute(
                                _tokens->rigExecSampleCount)) {
                            int c = 5;
                            a.Get(&c);
                            // The transported-frame ribbon needs >= 2 samples;
                            // below that it produces no frames, so no element
                            // is bindable (matches the runtime cardinality).
                            knownCount = c >= 2 ? c : 0;
                        }
                    } else if (type == SchemaTypeName::RigExecSplineIk) {
                        // The chain IS the cardinality: one frame per
                        // joints entry. A remap must then be a permutation
                        // of the chain slots, and the per-joint volume
                        // weights must be parallel to it.
                        knownCount = static_cast<int>(jointTargets.size());
                        if (!jointElements.empty()) {
                            std::vector<bool> filled(jointTargets.size(),
                                                     false);
                            for (int e : jointElements) {
                                if (e >= 0 && e < knownCount) {
                                    if (filled[e]) {
                                        return attempt.Fail(
                                            who() +
                                                ": rigExec:jointElements "
                                                "fills chain slot " +
                                                std::to_string(e) + " twice",
                                            {solver.GetPath()});
                                    }
                                    filled[e] = true;
                                }
                            }
                        }
                        if (const UsdAttribute a = solver.GetAttribute(
                                _tokens->rigExecVolumeWeights)) {
                            VtFloatArray weights;
                            a.Get(&weights);
                            if (!weights.empty() &&
                                weights.size() != jointTargets.size()) {
                                return attempt.Fail(
                                    who() + ": rigExec:volumeWeights length " +
                                        std::to_string(weights.size()) +
                                        " must equal rigExec:joints length " +
                                        std::to_string(jointTargets.size()) +
                                        " (or be empty)",
                                    {solver.GetPath()});
                            }
                        }
                    } else if (type == SchemaTypeName::RigExecSkeleton2d) {
                        // One frame per bone, in list order: a bone's list
                        // position is its element, never a remap.
                        knownCount = static_cast<int>(jointTargets.size());
                        if (!jointElements.empty()) {
                            return attempt.Fail(
                                who() + ": rigExec:jointElements is not "
                                        "read by a 2D skeleton; a bone's "
                                        "list position is its element",
                                {solver.GetPath()});
                        }
                    }

                    for (size_t i = 0; i < jointTargets.size(); ++i) {
                        const SdfPath &jointPath = jointTargets[i];
                        const int element = i < jointElements.size()
                                                ? jointElements[i]
                                                : static_cast<int>(i);
                        if (element < 0) {
                            return attempt.Fail(
                                who() + ": negative element index " +
                                    std::to_string(element) + " for " +
                                    jointPath.GetString(),
                                {solver.GetPath()});
                        }
                        if (knownCount >= 0 && element >= knownCount) {
                            return attempt.Fail(
                                who() + ": element " + std::to_string(element) +
                                    " for " + jointPath.GetString() +
                                    " is out of range (solver produces " +
                                    std::to_string(knownCount) + " frames)",
                                {solver.GetPath()});
                        }
                        const UsdPrim jointPrim =
                            _stage->GetPrimAtPath(jointPath);
                        if (!jointPrim) {
                            return attempt.Fail(
                                who() +
                                    " rigExec:joints targets missing "
                                    "prim " +
                                    jointPath.GetString(),
                                {solver.GetPath()});
                        }
                        if (jointPrim.GetTypeName() !=
                            SchemaTypeName::RigExecJoint) {
                            return attempt.Fail(
                                who() + " rigExec:joints target " +
                                    jointPath.GetString() + " is a " +
                                    jointPrim.GetTypeName().GetString() +
                                    ", not a RigExecJoint",
                                {solver.GetPath()});
                        }
                        // Every joint is tapped individually and a bound one also
                        // carries a per-prim value override, both of which key on
                        // a real prim; only the rig's nearest instanceable
                        // ancestor is deinstanced. Reject an instance-proxy /
                        // prototype-hosted joint up front.
                        if (jointPrim.IsInstanceProxy() ||
                            jointPrim.IsInPrototype()) {
                            return attempt.Fail(
                                who() + " rigExec:joints target " +
                                    jointPath.GetString() +
                                    " is instance-proxy/prototype "
                                    "hosted and cannot "
                                    "receive a solver binding",
                                {solver.GetPath()});
                        }
                        // Exclusive ownership: a solver-posed joint must not
                        // also author its own posed:space connection (the solver
                        // pose is supplied as an override and would silently win
                        // over the connection the raw stage shows).
                        if (const UsdPrim srcJoint =
                                _stage->GetPrimAtPath(jointPath)) {
                            if (const UsdAttribute posed =
                                    srcJoint.GetAttribute(
                                        _tokens->posedSpace)) {
                                const SdfPathVector conns =
                                    _AuthoredConnections(posed);
                                if (!conns.empty()) {
                                    return attempt.Fail(
                                        who() + ": joint " +
                                            jointPath.GetString() +
                                            " also connects posed:space",
                                        {solver.GetPath()});
                                }
                            }
                        }
                        // A consumed solver naming a joint that a solver
                        // nobody reads already writes to is referencing it
                        // for its REST, not claiming it: the consumer is
                        // what poses it. Any other joint it names it still
                        // poses itself.
                        if (consumedSolvers.count(solver.GetPath()) &&
                            posedByUnconsumed.count(jointPath)) {
                            continue;
                        }
                        // Two SOLVERS writing one joint is legal and stacks.
                        // One solver naming one joint TWICE is not: the two
                        // entries would collapse at runtime (candidates[] is
                        // a map, and the baked commit's slots are uniqued),
                        // and a stack edge between them would be a self-edge
                        // that Kahn reports as an unexplained pose cycle. Say
                        // what actually happened instead.
                        std::vector<std::pair<SdfPath, int>> &writers =
                            newJointBinding[jointPath];
                        for (const auto &[writer, writerElement] : writers) {
                            if (writer == solver.GetPath()) {
                                return attempt.Fail(
                                    who() + ": rigExec:joints names " +
                                        jointPath.GetString() +
                                        " more than once",
                                    {solver.GetPath()});
                            }
                        }
                        writers.emplace_back(solver.GetPath(), element);
                    }
                }
            }
        }
    }

    // The binding loop appends in UsdPrimRange pre-order, which is the exact
    // REVERSE of the stack, so every writer list is put into stack-ordinal
    // order here. This is the AUTHORED order, and it is not decoration: the
    // "reads the version standing before its own commit" edges built while
    // the solver DAG is assembled are read off these lists. The pass that
    // settles the stack against the finished pose graph replaces the order
    // with the one data flow and the schedule actually produce, and the two
    // agree whenever nothing orders a pair.
    for (auto &[joint, writers] : newJointBinding) {
        if (writers.size() < 2) {
            continue; // a one-element stack has no order to get wrong
        }
        // find(), never operator[]: mutating a captured map from inside a
        // sort comparator would insert a silent 0 for any writer the ordinal
        // sweep did not see and make the comparator inconsistent. An unknown
        // writer sorts last instead.
        const auto ordinalOf = [&solverStackOrdinal](const SdfPath &path) {
            const auto it = solverStackOrdinal.find(path);
            return it == solverStackOrdinal.end()
                       ? std::numeric_limits<int>::max()
                       : it->second;
        };
        std::stable_sort(writers.begin(), writers.end(),
                         [&ordinalOf](const std::pair<SdfPath, int> &a,
                                      const std::pair<SdfPath, int> &b) {
                             return ordinalOf(a.first) < ordinalOf(b.first);
                         });
    }
    return true;
}

void
RigExecRigEvaluator::_CompileSolverDag(_CompileAttempt &attempt)
{
    const std::vector<UsdPrim> &orderedSolvers = attempt.aggregateSolvers;
    auto &newJointBinding = attempt.newJointBinding;
    // A switch on a 2D skeleton's bone is read by that skeleton, not stepped;
    // its sources outside the skeleton become the solve's frame inputs.
    auto &skeleton2dBones = attempt.skeleton2dBones;
    auto &skeleton2dSwitchSources = attempt.skeleton2dSwitchSources;
    {
        std::map<SdfPath, SdfPath> skeletonOf;
        std::map<SdfPath, std::set<SdfPath>> bonesOf;
        for (const UsdPrim &solver : orderedSolvers) {
            if (solver.GetTypeName() !=
                _skeleton2dCompileTokens->RigExecSkeleton2d)
                continue;
            SdfPathVector bones;
            solver.GetRelationship(_skeleton2dCompileTokens->joints)
                .GetTargets(&bones);
            for (const SdfPath &bone : bones) {
                skeletonOf.emplace(bone, solver.GetPath());
                bonesOf[solver.GetPath()].insert(bone);
                skeleton2dBones.insert(bone);
            }
        }
        if (!skeletonOf.empty()) {
            for (const UsdPrim &prim :
                 UsdPrimRange(_stage->GetPrimAtPath(_rigPath))) {
                if (prim.GetTypeName() !=
                    _skeleton2dCompileTokens->RigExecSpaceSwitch)
                    continue;
                SdfPathVector switched, sources;
                prim.GetRelationship(_skeleton2dCompileTokens->target)
                    .GetTargets(&switched);
                if (switched.size() != 1)
                    continue;
                const auto owner = skeletonOf.find(switched[0]);
                if (owner == skeletonOf.end())
                    continue;
                prim.GetRelationship(_skeleton2dCompileTokens->sources)
                    .GetTargets(&sources);
                for (const SdfPath &source : sources) {
                    if (!bonesOf[owner->second].count(source)) {
                        skeleton2dSwitchSources[owner->second].push_back(
                            source);
                    }
                }
            }
        }
    }
    // Compile the solver DAG, including frame inputs carried by a posed
    // namespace ancestor. Rest inputs are independent of posed outputs and
    // therefore never introduce a feedback edge (IK -> blend is legal).
    // Producers (no pose-stack position) are ordered by data flow only: a
    // solver whose aggregate another solver reads, or one that writes no joint.
    // Consumers of the ordinal map test membership, never operator[].
    std::set<SdfPath> aggregateProducers;
    {
        std::set<SdfPath> solverPaths;
        for (const UsdPrim &solver : orderedSolvers) {
            solverPaths.insert(solver.GetPath());
        }
        for (const UsdPrim &solver : orderedSolvers) {
            for (const UsdRelationship &rel : solver.GetRelationships()) {
                if (rel.GetName() == SchemaAttrText::rigExecJoints)
                    continue;
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    const SdfPath prim = target.GetPrimPath();
                    if (prim != solver.GetPath() && solverPaths.count(prim)) {
                        aggregateProducers.insert(prim);
                    }
                }
            }
        }
    }
    auto &jointWritingSolvers = attempt.jointWritingSolvers;
    for (const auto &[joint, writers] : newJointBinding) {
        for (const auto &[writer, element] : writers) {
            if (aggregateProducers.count(writer))
                continue;
            jointWritingSolvers.insert(writer);
        }
    }
    // "A writer waits on an EARLIER READER." The new edge class the unified
    // pose stack needs: a step that reads a provider from BELOW
    // its writer reads the version standing before that write, and without an
    // edge saying so Kahn may schedule the two either way round within a level
    // and the version the reader sees becomes undefined -- a nondeterministic
    // failure, which is the worst kind here. Collected wherever a read is
    // resolved and applied once poseDependencies exists, because the two
    // halves are found on opposite sides of the constraint pass.
    // (waiter, waited-on): poseDependencies[first].insert(second).
    auto &poseReverseEdges = attempt.poseReverseEdges;
    auto &newSolverDependencies = attempt.newSolverDependencies;
    // Aggregate reads bind a producer's same-frame output. Pose frame reads
    // select authored checkpoints; the common graph schedules both classes.
    auto &newSolverAggregateReads = attempt.newSolverAggregateReads;
    {
        const std::vector<UsdPrim> &solvers = orderedSolvers;
        std::set<SdfPath> solverPaths;
        for (const UsdPrim &solver : solvers) {
            solverPaths.insert(solver.GetPath());
            newSolverDependencies[solver.GetPath()];
        }
        for (const UsdPrim &solver : solvers) {
            // The relationship targets, then a 2D skeleton's outside
            // switch sources.
            std::vector<SdfPathVector> reads;
            for (const UsdRelationship &rel : solver.GetRelationships()) {
                if (rel.GetName() == SchemaAttrText::rigExecJoints) {
                    continue;
                }
                SdfPathVector targets;
                rel.GetTargets(&targets);
                attempt.AppendDerivedStart(rel, &targets);
                reads.push_back(std::move(targets));
            }
            const auto switchSources =
                skeleton2dSwitchSources.find(solver.GetPath());
            if (switchSources != skeleton2dSwitchSources.end()) {
                reads.push_back(switchSources->second);
            }
            for (const SdfPathVector &targets : reads) {
                for (const SdfPath &target : targets) {
                    const SdfPath targetPrim = target.GetPrimPath();
                    if (solverPaths.count(targetPrim)) {
                        newSolverDependencies[solver.GetPath()].insert(
                            targetPrim);
                        newSolverAggregateReads[solver.GetPath()].insert(
                            targetPrim);
                        continue;
                    }
                    const UsdPrim provider = _stage->GetPrimAtPath(targetPrim);
                    const bool frameProvider = _IsFrameProvider(provider);
                    for (SdfPath path = targetPrim;
                         !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
                         path = frameProvider ? path.GetParentPath()
                                              : SdfPath()) {
                        const auto binding = newJointBinding.find(path);
                        if (binding != newJointBinding.end()) {
                            // Every writer of this joint that precedes this
                            // solver in the stack, and NEVER this solver
                            // itself: a solver that reads a joint it also
                            // writes reads the version standing before its
                            // own commit, so it depends on the writers below
                            // it and on nothing above.
                            for (const auto &[writer, writerElement] :
                                 binding->second) {
                                if (writer == solver.GetPath()) {
                                    continue;
                                }
                                if (jointWritingSolvers.count(
                                        solver.GetPath()) &&
                                    attempt.StackBefore(solver.GetPath(),
                                                        writer)) {
                                    // The writer stands ABOVE this solver, so
                                    // the solver reads the version before that
                                    // write -- and the writer has to wait, or
                                    // which version it read is undefined.
                                    poseReverseEdges.emplace_back(
                                        writer, solver.GetPath());
                                    continue;
                                }
                                newSolverDependencies[solver.GetPath()].insert(
                                    writer);
                            }
                            // A solver override replaces the whole joint
                            // callback, so that joint never reads its own
                            // namespace-parent posed frame.
                            break;
                        }
                    }
                }
            }
        }
    }
}

} // namespace rigExec
