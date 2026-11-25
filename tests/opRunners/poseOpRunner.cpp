#include "poseOpRunner.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/types.h"

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
