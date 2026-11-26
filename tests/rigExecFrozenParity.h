// Live versus frame-cache worker parity for an operator's own tests: one
// warming job through the production step runner must reproduce live
// evaluation of the same time bit for bit.
#ifndef RIGEXEC_TEST_FROZEN_PARITY_H
#define RIGEXEC_TEST_FROZEN_PARITY_H

#include "rigExec/api/cancellation.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/api/frozenContext.h"
#include "rigExec/api/rigEvaluator.h"

#include <memory>
#include <string>

/// What the production runner answered inside RigExecEvaluateFrozen, which
/// hands back an empty invalid pose whenever the runner declines.
struct RigExecFrozenRunnerVerdict {
    bool called = false;
    bool ran = false;
    rigExec::RigExecRigPose pose;
};

/// The production step runner, recording its answer in \p verdict.
inline rigExec::RigExecFrozenStepRunner
RigExecRecordingStepRunner(RigExecFrozenRunnerVerdict *verdict)
{
    using namespace rigExec;
    return [verdict, inner = RigExecMakeProductionStepRunner()](
               const RigExecFrozenEvalContext &context,
               const RigExecFrameInputs &inputs, RigExecFrozenArena &arena,
               RigExecRigPose *pose) {
        verdict->called = true;
        verdict->ran = inner(context, inputs, arena, pose);
        if (pose)
            verdict->pose = *pose;
        return verdict->ran;
    };
}

/// Whether a frozen job at a frame live fails stopped as live did: admitted,
/// the runner ran the frame, its graph failed, and nothing was published.
/// The worker's pose carries no line of the error (review finding R35: the
/// frozen job does not report live's error).
inline bool
RigExecFrozenFailedInTheWorker(const rigExec::RigExecRigPose &frozen,
                               const RigExecFrozenRunnerVerdict &verdict)
{
    return !frozen.valid && verdict.called && !verdict.ran &&
           !verdict.pose.valid && verdict.pose.diagnostics.empty();
}

/// Freezes \p evaluator's program, samples \p time's inputs at its
/// interactive overrides, runs one
/// warming job and compares its pose with a live evaluation at \p time.
/// False with the reason: a refused freeze or sample, a declined job, or
/// the first parity mismatches.
inline bool
RigExecFrozenMatchesLive(rigExec::RigExecRigEvaluator *evaluator,
                         const PXR_NS::SdfPath &rig, PXR_NS::UsdTimeCode time,
                         std::string *why)
{
    using namespace rigExec;
    std::string error;
    if (!evaluator->Evaluate(time).valid) {
        *why = "the live evaluation is invalid";
        return false;
    }
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    if (!RigExecFreezeProgram(*evaluator, &frozen, &error) || !frozen) {
        *why = "freeze refused: " + error;
        return false;
    }
    RigExecFrameInputs inputs;
    if (!RigExecSampleFrameInputs(*evaluator, time,
                                  evaluator->GetInteractiveOverrides(), &inputs,
                                  &error)) {
        *why = "sampling refused: " + error;
        return false;
    }
    RigExecCancellationSource cancellation;
    RigExecFrozenEvalContext context;
    context.compiledRigSignature = evaluator->GetCompiledRigSignature();
    context.generation = cancellation.Serial();
    context.slotCount = evaluator->GetBakedProgram()->GetProviderCount();
    context.varyingInputCount = inputs.values.size();
    if (evaluator->GetPublishWeightFields()) {
        context.flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator->GetSolverGuidesEnabled()) {
        context.flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.frozen = frozen.get();
    const RigExecRigPose warmed = RigExecEvaluateFrozen(
        context, inputs, RigExecMakeProductionStepRunner(),
        cancellation.Token(context.generation));
    if (!warmed.valid) {
        *why = "the warming job was declined";
        return false;
    }
    const RigExecRigPose live = evaluator->Evaluate(time);
    RigExecRigPose diff;
    RigExecComparePoses(live, warmed, &diff);
    if (diff.comparisonMismatches != 0 || !diff.diagnostics.empty()) {
        *why = std::to_string(diff.comparisonMismatches) + " mismatch(es)";
        for (const std::string &line : diff.diagnostics) {
            *why += "; " + line;
        }
        return false;
    }
    return true;
}

#endif
