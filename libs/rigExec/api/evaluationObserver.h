#ifndef RIGEXEC_API_EVALUATION_OBSERVER_H
#define RIGEXEC_API_EVALUATION_OBSERVER_H

/// A hook for checks that watch an evaluator from outside it: recording its
/// frames, replaying them later, or comparing it with a second evaluator.
/// The evaluator calls each installed observer at fixed points, on the
/// thread that owns the evaluator: in installation order, except that
/// CompileFinished, CompileRefused and FrameEnded go in reverse, so an
/// observer that opens something when a call starts closes it after those
/// installed later have finished. With none installed, each point only
/// finds the list empty.

#include "rigExec/api/rigEvaluator.h"

#include <set>
#include <string>
#include <vector>

namespace rigExec {

/// Watches one RigExecRigEvaluator. An observer reads the evaluator through
/// its public interface and may add diagnostics and mismatch counts to a
/// pose it is shown; it changes nothing else the evaluator holds. Every
/// call has an empty default.
class RigExecEvaluationObserver {
public:
    virtual ~RigExecEvaluationObserver() = default;

    /// Compile is about to run; \p errors is the caller's list, possibly
    /// null.
    virtual void CompileStarting(RigExecRigEvaluator &,
                                 std::vector<std::string> *)
    {
    }
    /// Compile ran and returns \p compiled.
    virtual void CompileFinished(RigExecRigEvaluator &, bool compiled,
                                 std::vector<std::string> *)
    {
        (void)compiled;
    }
    /// Compile refused the rig before compiling it, for example over a
    /// malformed input declaration, and returns false.
    virtual void CompileRefused(RigExecRigEvaluator &,
                                std::vector<std::string> *)
    {
    }
    /// Evaluate(\p time) is about to run.
    virtual void FrameStarting(RigExecRigEvaluator &, UsdTimeCode)
    {
    }
    /// The frame at \p time is complete in \p pose, which Evaluate returns
    /// once every observer has seen it.
    virtual void FrameEvaluated(RigExecRigEvaluator &, UsdTimeCode,
                                RigExecRigPose &pose)
    {
        (void)pose;
    }
    /// Evaluate is returning, after FrameEvaluated, or early.
    virtual void FrameEnded(RigExecRigEvaluator &)
    {
    }
    /// The solver guide and weight field settings are about to become
    /// \p solverGuides and \p publishWeightFields.
    virtual void OptionsChanging(RigExecRigEvaluator &, bool solverGuides,
                                 bool publishWeightFields)
    {
        (void)solverGuides, (void)publishWeightFields;
    }
    /// SetBakedRoleMode was called with these arguments.
    virtual void BakedRoleModeChanging(RigExecRigEvaluator &,
                                       RigExecBakedRoleMode,
                                       const std::set<SdfPath> &)
    {
    }
    /// The interactive overrides are about to become \p overrides; null
    /// for ClearInteractiveOverrides.
    virtual void
    InteractiveOverridesChanging(RigExecRigEvaluator &,
                                 const std::vector<RigExecValueOverride> *)
    {
    }
    /// The requested upstream inputs are about to become \p inputs.
    virtual void
    UpstreamInputsChanging(RigExecRigEvaluator &,
                           const std::vector<RigExecValueOverride> &)
    {
    }
    /// The evaluator replaced its compiled program with a rebuilt one.
    virtual void ProgramReplaced(RigExecRigEvaluator &)
    {
    }
};

/// Called by every evaluator's constructor to install the observers this
/// process asks for. Set once, before the first evaluator is made; null
/// installs none.
using RigExecEvaluationObserverFactory = void (*)(RigExecRigEvaluator &);
void
RigExecSetEvaluationObserverFactory(RigExecEvaluationObserverFactory factory);

} // namespace rigExec

#endif
