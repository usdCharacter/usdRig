#ifndef RIGEXEC_TESTS_POSE_OP_RUNNER_H
#define RIGEXEC_TESTS_POSE_OP_RUNNER_H

/// One solver, constraint, auto clavicle, harmonics or IK/FK match run through
/// the op core, for tests that compare an op with its native kernel. Built on
/// first use and rebuilt when a setting baked into it moved, so stale
/// settings never run.

#include "rigExec/api/evaluator.h"
#include "rigExecScene/poseOps.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecPoseOpRunner {
    enum class Built {
        None,
        Solver,
        Constraint,
        AutoClavicle,
        Harmonics,
        IkFkMatch
    };
    std::unique_ptr<RigExecEvaluator> evaluator;
    Built built = Built::None;
    RigExecSolverOpNode solver;
    RigExecConstraintOpNode constraint;
    RigExecAutoClavicleOpNode clavicle;
    RigExecHarmonicsOpNode harmonics;
    RigExecIkFkMatchOpNode match;
    RigExecPoseOpScratch scratch;
    RigExecCompileOptions compile;
    RigExecEvaluatorOptions evaluate;
    std::vector<RigExecWindowRing> rings;
};

/// RigExecRunSolver for a record RigExecSolverOpSupported accepts: the same
/// output where the op runs. False with the compile or evaluation error
/// where it does not.
bool RigExecRunSolverOp(const RigExecSolverRecord &record,
                        const RigExecSolverInputs &input,
                        RigExecPoseOpRunner &runner,
                        RigExecPointFrameArray *output,
                        std::string *error = nullptr);

/// A solver's frames from its reference kernels: an FK chain from the
/// chain kernels the FkChain and FkJointGroups ops run, any other kind
/// through RigExecRunSolver.
bool RigExecReferenceSolve(const RigExecSolverRecord &record,
                           const RigExecSolverInputs &input,
                           RigExecSolverWorkspace *workspace,
                           RigExecPointFrameArray *output,
                           std::string *error = nullptr);

/// RigExecRunConstraint for a kind RigExecConstraintOpSupported accepts.
bool RigExecRunConstraintOp(const RigExecConstraintRecord &record,
                            const RigExecConstraintInputs &input,
                            RigExecPoseOpRunner &runner,
                            RigExecConstraintResult *result,
                            std::string *error = nullptr);

/// RigExecAutoClavicleShift on the op core: the shift in \p delta and the
/// target frame moved by it in \p shifted.
bool RigExecRunAutoClavicleOp(const RigExecAutoClavicleConstants &constants,
                              const RigExecAutoClavicleOpInputs &input,
                              RigExecPoseOpRunner &runner, double delta[3],
                              GfMatrix4d *shifted,
                              std::string *error = nullptr);

/// A harmonics constraint's frame on the op core. The ring and capture stay
/// the caller's: \p ring (null without a window) is set as the op's window,
/// and this tick's capture comes back in \p capture. A \p captureOnly tick
/// runs as the op evaluator's history tick and answers the incoming frame.
/// False with \p error where the op's compile or evaluation fails.
bool RigExecRunHarmonicsOp(const RigExecHarmonicsOpConstants &constants,
                           const RigExecHarmonicsOpInputs &input,
                           const RigExecWindowRing *ring, bool captureOnly,
                           RigExecPoseOpRunner &runner,
                           RigExecWindowPacketPtr *capture,
                           RigExecPointFrame *frame,
                           std::string *error = nullptr);

/// An IK/FK match's values and presented frames on the op core.
bool RigExecRunIkFkMatchOp(const RigExecIkFkMatchOpConstants &constants,
                           const RigExecIkFkMatchOpInputs &input,
                           RigExecPoseOpRunner &runner,
                           RigExecIkFkMatchOpOutputs *output,
                           std::string *error = nullptr);

} // namespace rigExec

#endif
