// The verification checks that watch an evaluator through
// RigExecEvaluationObserver, and the factory that installs the ones this
// process asks for.

#include "rigExec/api/bakedProgram.h"
#include "rigExec/api/evaluationObserver.h"
#include "rigExec/internal/inputReplay.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/stringUtils.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

namespace {

// RIGEXEC_VERIFY_SCOPED_CLEARS=1: every evaluator builds a shadow of itself
// whose notices drop the value caches whole, evaluates it beside every
// generation, and reports each way the two poses differ as a baked parity
// mismatch. The cone verifier cannot stand in for this: it captures the
// program after the prologue, which is where a stale cache would already
// have been read. Read once per process.
const bool kVerifyScopedClears =
    TfGetenvBool("RIGEXEC_VERIFY_SCOPED_CLEARS", false);

/// The shadow: an evaluator of the same rig on the same stage, told
/// everything a caller tells the observed one, in the same order, so that
/// the one thing that differs between them is how their notices clear the
/// value caches. Evaluated at every time the observed one is and compared
/// with it pose for pose.
class ScopedClearShadow : public RigExecEvaluationObserver {
public:
    void CompileFinished(RigExecRigEvaluator &evaluator, bool,
                         std::vector<std::string> *) override
    {
        _Ensure(evaluator);
        if (_shadow)
            _shadow->Compile();
    }
    void FrameStarting(RigExecRigEvaluator &evaluator, UsdTimeCode) override
    {
        _Ensure(evaluator);
    }
    void FrameEvaluated(RigExecRigEvaluator &evaluator, UsdTimeCode time,
                        RigExecRigPose &pose) override
    {
        if (_shadow)
            _Verify(evaluator, time, &pose);
    }
    void OptionsChanging(RigExecRigEvaluator &, bool solverGuides,
                         bool publishWeightFields) override
    {
        if (!_shadow)
            return;
        _shadow->SetPublishWeightFields(publishWeightFields);
        _shadow->SetSolverGuidesEnabled(solverGuides);
    }
    void BakedRoleModeChanging(RigExecRigEvaluator &, RigExecBakedRoleMode mode,
                               const std::set<SdfPath> &exportKeep) override
    {
        if (_shadow)
            _shadow->SetBakedRoleMode(mode, exportKeep);
    }
    void InteractiveOverridesChanging(
        RigExecRigEvaluator &,
        const std::vector<RigExecValueOverride> *overrides) override
    {
        if (!_shadow)
            return;
        if (overrides)
            _shadow->SetInteractiveOverrides(*overrides);
        else
            _shadow->ClearInteractiveOverrides();
    }
    void UpstreamInputsChanging(
        RigExecRigEvaluator &,
        const std::vector<RigExecValueOverride> &inputs) override
    {
        if (_shadow)
            _shadow->SetUpstreamInputs(inputs);
    }

private:
    /// Builds the shadow the first time the observed evaluator compiles or
    /// evaluates, handing over what a caller set before then in the order
    /// it would have been set on a fresh evaluator. A shadow has none.
    void _Ensure(RigExecRigEvaluator &evaluator)
    {
        if (_shadow || evaluator.GetWholesaleValueClears())
            return;
        RigExecInputReplaySuppression suppressInputReplay;
        auto shadow = std::make_unique<RigExecRigEvaluator>(
            evaluator.GetEvaluationStage(), evaluator.GetRigPath());
        shadow->SetWholesaleValueClears(true);
        shadow->SetPublishWeightFields(evaluator.GetPublishWeightFields());
        shadow->SetSolverGuidesEnabled(evaluator.GetSolverGuidesEnabled());
        if (!evaluator.GetInteractiveOverrides().empty())
            shadow->SetInteractiveOverrides(
                evaluator.GetInteractiveOverrides());
        if (!evaluator.GetUpstreamInputs().empty())
            shadow->SetUpstreamInputs(evaluator.GetUpstreamInputs());
        shadow->SetBakedRoleMode(evaluator.GetBakedRoleMode(),
                                 evaluator.GetBakedExportKeep());
        _shadow = std::move(shadow);
    }

    /// Evaluates the shadow at \p time and appends to \p pose every way the
    /// two disagree, counted on comparisonMismatches.
    void _Verify(RigExecRigEvaluator &evaluator, UsdTimeCode time,
                 RigExecRigPose *pose)
    {
        // A public field rather than a setter, so it is handed over per
        // call.
        _shadow->cpuReference = evaluator.cpuReference;
        const RigExecRigPose shadow = _shadow->Evaluate(time);
        // Every domain --pose-out writes. RigExecComparePoses covers the
        // maps, the ordered diagnostics and the work counters. Also compares
        // the scalars RigExecComparePoses skips.
        RigExecRigPose differences;
        RigExecComparePoses(shadow, *pose, &differences);
        const auto scalar = [&differences](size_t wholesale, size_t scoped,
                                           const char *what) {
            if (wholesale != scoped) {
                differences.diagnostics.push_back(
                    std::string("baked parity: ") + what + " differs (" +
                    std::to_string(wholesale) + " vs " +
                    std::to_string(scoped) + ")");
                ++differences.comparisonMismatches;
            }
        };
        scalar(shadow.valid, pose->valid, "valid");
        scalar(shadow.comparisonMismatches, pose->comparisonMismatches,
               "baked parity mismatches");
        scalar(shadow.referenceMismatches, pose->referenceMismatches,
               "mover graph parity mismatches");
        scalar(shadow.referenceAgreements, pose->referenceAgreements,
               "mover graph parity agreements");
        if (shadow.movedPropertiesCpu != pose->movedPropertiesCpu) {
            differences.diagnostics.push_back(
                "baked parity: CPU moved properties differ");
            ++differences.comparisonMismatches;
        }
        if (differences.comparisonMismatches == 0)
            return;
        for (const std::string &line : differences.diagnostics)
            pose->diagnostics.push_back("scoped cache clears: " + line);
        pose->comparisonMismatches += differences.comparisonMismatches;
        // The wording a parity disagreement uses, so one expression catches
        // it.
        TF_WARN("rigExec: baked parity mismatch: %zu difference(s) between "
                "the scoped cache clears and a shadow evaluator that clears "
                "them whole, at time %s: %s",
                differences.comparisonMismatches, TfStringify(time).c_str(),
                TfStringJoin(differences.diagnostics, "; ").c_str());
    }

    std::unique_ptr<RigExecRigEvaluator> _shadow;
};

void
InstallVerification(RigExecRigEvaluator &evaluator)
{
    if (kVerifyScopedClears)
        evaluator.AddEvaluationObserver(std::make_unique<ScopedClearShadow>());
}

const bool kInstalled =
    (RigExecSetEvaluationObserverFactory(&InstallVerification), true);

} // namespace

} // namespace rigExec
