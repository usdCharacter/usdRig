#include "rigExecScene/poseOpRunner.h"

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
    const auto program = desc ? RigExecCompile(*desc, {}, error) : nullptr;
    runner.evaluator =
        program ? RigExecEvaluator::Create(program, {}, error) : nullptr;
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
    const bool built = runner.evaluator && runner.builtSolver &&
                       runner.solver.kind == record.kind;
    const bool ready = Ready(
        runner, built,
        [&](RigExecRigDescBuilder &builder) {
            runner.builtSolver = true;
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
    std::string why;
    if (!runner.evaluator->Evaluate(&why)) {
        // A curve or twist that does not solve: native's empty aggregate.
        if (record.kind == RigExecSolverKind::Ribbon ||
            record.kind == RigExecSolverKind::TwistDistribution) {
            if (error)
                error->clear();
            return true;
        }
        return Fail(error, why);
    }
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
    const bool built = runner.evaluator && !runner.builtSolver &&
                       runner.constraint.kind == record.kind;
    const bool ready = Ready(
        runner, built,
        [&](RigExecRigDescBuilder &builder) {
            runner.builtSolver = false;
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

} // namespace rigExec
