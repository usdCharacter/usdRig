#include "poseOpRunner.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/types.h"
#include "rigExec/internal/solverKernels.h"

namespace rigExec {

namespace {

bool
Fail(std::string *error, const std::string &message)
{
    if (error)
        *error = message;
    return false;
}

/// Writes into the built nodes; builds first when there are none, and
/// rebuilds once when the write refused settings that moved.
template <class Add, class Write>
bool
Ready(RigExecPoseOpRunner &runner, bool built, const Add &add,
      const Write &write, std::string *error)
{
    std::string moved;
    if (built && write(&moved))
        return true;
    runner.evaluator.reset();
    RigExecRigDescBuilder builder;
    if (!add(builder))
        return false;
    const auto desc = builder.Build(error);
    const auto program =
        desc ? RigExecCompile(*desc, runner.compile, error) : nullptr;
    runner.evaluator =
        program ? RigExecEvaluator::Create(program, runner.evaluate, error)
                : nullptr;
    return runner.evaluator && write(error);
}

} // namespace

bool
RigExecReferenceSolve(const RigExecSolverRecord &s,
                      const RigExecSolverInputs &input,
                      RigExecSolverWorkspace *workspace,
                      RigExecPointFrameArray *output, std::string *error)
{
    const bool ik = s.kind == RigExecSolverKind::TwoBoneIk;
    const bool twist = s.kind == RigExecSolverKind::TwistDistribution;
    const bool ribbon = s.kind == RigExecSolverKind::Ribbon;
    const bool spline = s.kind == RigExecSolverKind::SplineIk;
    if ((s.kind != RigExecSolverKind::FkChain && !ik && !twist && !ribbon &&
         !spline && s.kind != RigExecSolverKind::Skeleton2d &&
         s.kind != RigExecSolverKind::BlendPointFrames) ||
        s.degenerate)
        return RigExecRunSolver(s, input, workspace, output, error);
    output->frames.clear();
    output->rests.clear();
    if (twist) {
        *output = RigExecSolveTwistDistribution(
            input.root, input.end, s.twistStartRest, s.twistEndRest,
            s.twistWeights, input.twistTurns, s.jointRests, s.restIsLive);
        return true;
    }
    if (ribbon) {
        *output = RigExecSampleRibbonFrames(input.ribbonPoints,
                                            s.ribbonRestPoints, s.ribbonSamples,
                                            s.jointRests, s.restIsLive);
        return true;
    }
    if (s.kind == RigExecSolverKind::Skeleton2d) {
        // Rests and outputs in rigExec:joints order; the kernel runs in solve
        // order, in the start frame's plane. A failed solve publishes every
        // bone's setup frame (no deformation) and says why.
        const size_t n = s.bone2dOrder.size();
        if (s.jointRests.size() != n || s.bone2dSetup.size() != n ||
            input.bone2dChannels.size() != n ||
            input.controls.size() != s.controlRests.size()) {
            if (error)
                *error = kRigExecSkeleton2dCountsDiffer;
            return false;
        }
        const RigExecSkeleton2dStatic extra = RigExecSkeleton2dStaticOf(
            s.bone2dSpaceSources, s.bone2dTargetCandidates, s.jointGroups,
            s.jointGroupMembers);
        auto &posed = workspace->bone2dExternalPoses;
        posed.resize(input.controls.size());
        for (size_t e = 0; e < input.controls.size(); ++e)
            posed[e] = input.controls[e].points;
        RigExecSkeleton2dLandmarkInputs<RigExecSolverRest> frames;
        if (s.hasStart) {
            frames.startRest = &s.startRest;
            frames.startPose = &input.start.points;
        }
        frames.externalRests = s.controlRests.data();
        frames.externalPoses = posed.data();
        frames.externalCount = posed.size();
        frames.selectors = input.bone2dSelectors.data();
        frames.selectorCount = input.bone2dSelectors.size();
        frames.groupSelectors = input.jointGroupSelectors.data();
        frames.groupSelectorCount = input.jointGroupSelectors.size();
        const RigExecSkeleton2dResult result = RigExecSolveSkeleton2dJoints(
            s.bone2dOrder, s.jointRests, s.bone2dSetup, extra, frames,
            [&](size_t j, RigExecBone2dChannels *channels) {
                *channels = input.bone2dChannels[j];
            },
            RigExecPointFrameValid, &workspace->bone2d, &output->frames);
        if (result == RigExecSkeleton2dResult::Malformed) {
            if (error)
                *error = kRigExecSkeleton2dMalformed;
            return false;
        }
        output->rests = s.jointRests;
        if (result == RigExecSkeleton2dResult::NotFinite) {
            if (error)
                *error = kRigExecSkeleton2dNotFinite;
            return false;
        }
        return true;
    }
    if (s.kind == RigExecSolverKind::BlendPointFrames) {
        const auto *a = input.blendA, *b = input.blendB;
        if (!a) {
            if (b)
                *output = *b;
            return true;
        }
        if (!b) {
            *output = *a;
            return true;
        }
        if (s.blendRotationRejected)
            return true;
        const size_t n = a->GetSize();
        const double weight = std::min(std::max(input.blendWeight, 0.0), 1.0);
        if (n == b->GetSize() && a->rests.size() == n) {
            output->frames.reserve(n);
            output->rests.reserve(n);
            for (size_t k = 0; k < n; ++k) {
                const bool live = k < s.restIsLive.size() && s.restIsLive[k] &&
                                  k < s.jointRests.size();
                output->frames.push_back(RigExecBlendFrames(
                    a->frames[k], b->frames[k], a->rests[k], weight,
                    RigExecRotationBlend::ShortestArc, s.scaleMode,
                    live ? &s.jointRests[k] : nullptr));
                output->rests.push_back(live ? s.jointRests[k] : a->rests[k]);
            }
        }
        return true;
    }
    if (spline) {
        auto params = RigExecSolverSplineParams(s, input);
        const auto *rest = &s.splineRest;
        if (input.hasSpace) {
            GfMatrix4d delta(1.0);
            if (RigExecPointsToMatrix(input.spaceRest, input.space, &delta) &&
                delta != GfMatrix4d(1.0)) {
                auto &spaced = workspace->spacedFrames;
                spaced.clear();
                spaced.reserve(s.splineRestFrames.size());
                for (const auto &frame : s.splineRestFrames)
                    spaced.push_back(RigExecTransformFrame(frame, delta));
                workspace->spacedRest = RigExecSplineIkMakeRest(
                    spaced, RigExecTransformFrame(s.splineRootRest, delta),
                    RigExecTransformFrame(s.splineMidRest, delta),
                    RigExecTransformFrame(s.splineEndRest, delta),
                    s.splineRestWeights, s.splineRestMode);
                rest = &workspace->spacedRest;
            }
        }
        RigExecSplineIkControls controls;
        controls.root = input.root;
        controls.mid = input.mid;
        controls.end = input.end;
        RigExecSolveSplineIk(*rest, controls, params, &workspace->splineResult);
        if (workspace->splineResult.joints.size() == s.splineCount) {
            output->frames.reserve(s.splineCount);
            for (const auto &joint : workspace->splineResult.joints)
                output->frames.push_back(joint.frame);
            output->rests = s.splineJointRests;
        }
        return true;
    }
    if (ik) {
        auto params = s.ikParams;
        auto space = input.ikSpace;
        bool spaceMoved = false;
        if (input.hasSpace) {
            GfMatrix4d delta(1.0);
            if (RigExecPointsToMatrix(input.spaceRest, input.space, &delta)) {
                space = delta * space;
                spaceMoved = true;
            }
        }
        if (input.refreshIkParams || spaceMoved) {
            params.preferredBendRadians = input.bend;
            params.stretch = input.stretch;
            params.softness = input.softness;
            params.space = space;
            RigExecTwoBoneIkLengths(s.ikRests, space, input.upperOffset,
                                    input.lowerOffset, &params.upperLength,
                                    &params.lowerLength);
        }
        if (input.refreshIkParams || spaceMoved)
            RigExecSetTwoBoneLimbParams(
                s.ikRests, space, input.stretch, input.pin, input.upperScale,
                input.lowerScale, input.softDistance, input.limbTwist, &params);
        const auto frames = RigExecSolveTwoBoneIk(
            input.root, input.end, input.pole, s.ikRests, params);
        output->frames.assign(frames.begin(), frames.end());
        output->rests.assign(s.ikRests.begin(), s.ikRests.end());
        return true;
    }
    if (input.controls.size() != s.controlRests.size()) {
        if (error)
            *error = "FK control/rest cardinality mismatch";
        return false;
    }
    const bool jointBasis = s.jointRests.size() == input.controls.size();
    output->rests = s.controlRests;
    const int base = s.hasStart ? 1 : 0;
    auto &elements = workspace->fkElements;
    elements.resize(input.controls.size() + size_t(base));
    if (base) {
        elements[0].restPoints = s.startRest;
        elements[0].posePoints = input.start.points;
        elements[0].hasOutRest = false;
        elements[0].parentIndex = -1;
    }
    for (size_t k = 0; k < input.controls.size(); ++k) {
        auto &element = elements[k + size_t(base)];
        element.restPoints = s.controlRests[k];
        element.posePoints = input.controls[k].points;
        const bool live =
            jointBasis && k < s.restIsLive.size() && s.restIsLive[k];
        element.hasOutRest = live;
        if (live) {
            element.outRestPoints = s.jointRests[k];
            output->rests[k] = s.jointRests[k];
        }
        element.parentIndex = s.parentRelative ? base - 1 : int(k) + base - 1;
    }
    output->frames = RigExecSolveFkChain(elements);
    if (s.scaleSegments)
        RigExecScaleFkSegments(elements, &output->frames);
    if (base && !output->frames.empty())
        output->frames.erase(output->frames.begin());
    if (!s.jointGroups.empty()) {
        if (input.jointGroupSelectors.size() != s.jointGroups.size()) {
            if (error)
                *error = "FK joint group/selector cardinality mismatch";
            return false;
        }
        RigExecApplyFkJointGroups(
            elements, size_t(base), s.jointGroupParents, s.jointGroups,
            s.jointGroupMembers, input.jointGroupSelectors,
            base ? &s.startRest : nullptr, base ? &input.start.points : nullptr,
            &workspace->jointGroupSlots, &workspace->jointGroupActive,
            &output->frames);
    }
    return true;
}

bool
RigExecRunSolverOp(const RigExecSolverRecord &record,
                   const RigExecSolverInputs &input,
                   RigExecPoseOpRunner &runner, RigExecPointFrameArray *output,
                   std::string *error)
{
    if (!output)
        return Fail(error, "null solver workspace/output");
    switch (RigExecAnswerSolverOp(record, input, output, error)) {
    case RigExecSolverOpAnswer::Done:
        return true;
    case RigExecSolverOpAnswer::Failed:
        return false;
    case RigExecSolverOpAnswer::Run:
        break;
    }
    const bool built = runner.evaluator &&
                       runner.built == RigExecPoseOpRunner::Built::Solver &&
                       runner.solver.kind == record.kind;
    const bool ready = Ready(
        runner, built,
        [&](RigExecRigDescBuilder &builder) {
            runner.built = RigExecPoseOpRunner::Built::Solver;
            return RigExecAddSolverOp(record, input, &builder, &runner.solver,
                                      error);
        },
        [&](std::string *why) {
            return RigExecWriteSolverOpInputs(record, input, runner.solver,
                                              &runner.scratch,
                                              runner.evaluator.get(), why);
        },
        error);
    if (!ready)
        return false;
    if (!runner.evaluator->Evaluate(error))
        return false;
    if (!RigExecReadSolverOpOutput(record, input, runner.solver,
                                   *runner.evaluator, output))
        return Fail(error, "solver op has no output");
    return true;
}

bool
RigExecRunConstraintOp(const RigExecConstraintRecord &record,
                       const RigExecConstraintInputs &input,
                       RigExecPoseOpRunner &runner,
                       RigExecConstraintResult *result, std::string *error)
{
    if (!result)
        return Fail(error, "null constraint output");
    result->chain.clear();
    result->frame = input.incoming;
    const bool built = runner.evaluator &&
                       runner.built == RigExecPoseOpRunner::Built::Constraint &&
                       runner.constraint.kind == record.kind;
    const bool ready = Ready(
        runner, built,
        [&](RigExecRigDescBuilder &builder) {
            runner.built = RigExecPoseOpRunner::Built::Constraint;
            return RigExecAddConstraintOp(record, input, &builder,
                                          &runner.constraint, error);
        },
        [&](std::string *why) {
            return RigExecWriteConstraintOpInputs(
                record, input, runner.constraint, &runner.scratch,
                runner.evaluator.get(), why);
        },
        error);
    if (!ready || !runner.evaluator->Evaluate(error))
        return false;
    if (!RigExecReadConstraintOpOutput(record, input, runner.constraint,
                                       *runner.evaluator, result))
        return Fail(error, "constraint op has no output");
    return true;
}

bool
RigExecRunAutoClavicleOp(const RigExecAutoClavicleConstants &constants,
                         const RigExecAutoClavicleOpInputs &input,
                         RigExecPoseOpRunner &runner, double delta[3],
                         GfMatrix4d *shifted, std::string *error)
{
    const bool built = runner.evaluator &&
                       runner.built == RigExecPoseOpRunner::Built::AutoClavicle;
    const bool ready = Ready(
        runner, built,
        [&](RigExecRigDescBuilder &builder) {
            runner.built = RigExecPoseOpRunner::Built::AutoClavicle;
            return RigExecAddAutoClavicleOp(constants, input, &builder,
                                            &runner.clavicle, error);
        },
        [&](std::string *why) {
            return RigExecWriteAutoClavicleOpInputs(
                constants, input, runner.clavicle, &runner.scratch,
                runner.evaluator.get(), why);
        },
        error);
    if (!ready || !runner.evaluator->Evaluate(error))
        return false;
    if (!RigExecReadAutoClavicleOpOutput(runner.clavicle, *runner.evaluator,
                                         delta, shifted))
        return Fail(error, "auto clavicle op has no output");
    return true;
}

bool
RigExecRunHarmonicsOp(const RigExecHarmonicsOpConstants &constants,
                      const RigExecHarmonicsOpInputs &input,
                      const RigExecWindowRing *ring, bool captureOnly,
                      RigExecPoseOpRunner &runner,
                      RigExecWindowPacketPtr *capture, RigExecPointFrame *frame,
                      std::string *error)
{
    if (!capture || !frame)
        return Fail(error, "null harmonics output");
    capture->reset();
    *frame = input.incoming;
    const bool built = runner.evaluator &&
                       runner.built == RigExecPoseOpRunner::Built::Harmonics;
    const bool ready = Ready(
        runner, built,
        [&](RigExecRigDescBuilder &builder) {
            runner.built = RigExecPoseOpRunner::Built::Harmonics;
            return RigExecAddHarmonicsOp(constants, input, &builder,
                                         &runner.harmonics, error);
        },
        [&](std::string *why) {
            return RigExecWriteHarmonicsOpInputs(constants, input,
                                                 runner.harmonics,
                                                 runner.evaluator.get(), why);
        },
        error);
    if (!ready)
        return false;
    runner.rings.resize(1);
    if (ring)
        runner.rings[0] = *ring;
    else
        runner.rings[0].clear();
    if (!runner.evaluator->SetWindowRings(runner.rings, error))
        return false;
    if (captureOnly) {
        if (!runner.evaluator->EvaluateHistory(error))
            return false;
        const RigExecWindowRing &pushed = runner.evaluator->GetWindowRings()[0];
        if (!pushed.empty())
            *capture = pushed.back();
        return true;
    }
    if (!runner.evaluator->Evaluate(error))
        return false;
    *capture = runner.evaluator->GetWindowCaptures()[0];
    if (!RigExecReadHarmonicsOpOutput(runner.harmonics, *runner.evaluator,
                                      frame))
        return Fail(error, "harmonics op has no output");
    return true;
}

bool
RigExecRunIkFkMatchOp(const RigExecIkFkMatchOpConstants &constants,
                      const RigExecIkFkMatchOpInputs &input,
                      RigExecPoseOpRunner &runner,
                      RigExecIkFkMatchOpOutputs *output, std::string *error)
{
    if (!output)
        return Fail(error, "null IK/FK match output");
    const bool built = runner.evaluator &&
                       runner.built == RigExecPoseOpRunner::Built::IkFkMatch;
    const bool ready = Ready(
        runner, built,
        [&](RigExecRigDescBuilder &builder) {
            runner.built = RigExecPoseOpRunner::Built::IkFkMatch;
            return RigExecAddIkFkMatchOp(constants, &builder, &runner.match,
                                         error);
        },
        [&](std::string *why) {
            return RigExecWriteIkFkMatchOpInputs(constants, input, runner.match,
                                                 &runner.scratch,
                                                 runner.evaluator.get(), why);
        },
        error);
    if (!ready || !runner.evaluator->Evaluate(error))
        return false;
    if (!RigExecReadIkFkMatchOpOutput(runner.match, *runner.evaluator, output))
        return Fail(error, "IK/FK match op has no output");
    return true;
}

} // namespace rigExec
