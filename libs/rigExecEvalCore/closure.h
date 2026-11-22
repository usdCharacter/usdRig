// The shared step-execution policy both front-end dispatchers run around
// their routing switch: the per-step run framing and the
// RequiredStageFramesAdmission refusal before the body, and the labeled
// error composition for the step that failed. The routing switches, each
// front end's spelling of the admission slot in its own enum, the label
// text, and the adapter's own guards (the baked diagnostics-size check)
// stay adapter-local; this header owns the order and framing they run in.
// EvalCore still runs one step rigExecGraph already scheduled: no
// traversal, no scheduler, no engine-identity flag. Bodies stay no-mutex
// zones: this policy takes no lock, reads no environment, and touches no
// stage.
#ifndef RIGEXEC_EVAL_CORE_CLOSURE_H
#define RIGEXEC_EVAL_CORE_CLOSURE_H

#include <utility>

namespace rigExec {
namespace eval {

/// What the policy decided about one scheduled step. AdmissionRefused: the
/// step declared the RequiredStageFramesAdmission input and the program
/// refused it; the body never started, the shared memo/scheduler's
/// ordinary pure outcome stands, and no arithmetic or owned post-refusal
/// state changed. Ran: the routing switch ran a body that accepted.
/// Failed: it ran a body that reported failure and the epilogue composed
/// the step's error.
enum class StepRun { AdmissionRefused, Ran, Failed };

/// Runs one already-scheduled step. Prologue: `runOutput.BeginRun()`
/// frames this run's output, so the clearing is the executor's promise and
/// not something every body has to remember; then a step that asks for
/// admission (per `requiresAdmission`, which asks the front end's own slot
/// question) is refused before its body starts when the program's
/// `admitted` is false. Body: `route` runs the adapter's routing switch
/// and reports whether its body accepted. Epilogue: on failure
/// `composeError` writes the labeled error for the step that failed (a
/// no-op for a front end with no error channel).
template <class Output, class RequiresAdmission, class Route,
          class ComposeError>
StepRun
RunScheduledStep(Output &runOutput, bool admitted,
                 RequiresAdmission &&requiresAdmission, Route &&route,
                 ComposeError &&composeError)
{
    runOutput.BeginRun();
    if (!admitted && requiresAdmission())
        return StepRun::AdmissionRefused;
    if (!route()) {
        composeError();
        return StepRun::Failed;
    }
    return StepRun::Ran;
}

/// Skips one scheduled step: the MarkSkipped framing keeps the step's
/// diagnostics and program-size counters and drops only this run's
/// observations, then `skipRoute` runs the front end's skip routing.
template <class Output, class SkipRoute>
void
SkipScheduledStep(Output &runOutput, SkipRoute &&skipRoute)
{
    runOutput.MarkSkipped();
    skipRoute();
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_CLOSURE_H
