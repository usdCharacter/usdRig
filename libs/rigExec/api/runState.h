#ifndef RIGEXEC_API_RUN_STATE_H
#define RIGEXEC_API_RUN_STATE_H

/// A run state is one independent set of values for a compiled program:
/// the inputs a host sets, the values its ops compute and keep from frame
/// to frame, and the outputs of its last evaluation. What compile fixed,
/// such as op settings, stays in the program and is shared by every run
/// state of it. Use one to evaluate a program beside an evaluator without
/// touching it: a what-if pose, a copy that continues from where an
/// evaluator is (RigExecEvaluator::CreateRunState), or a worker that
/// evaluates the same program on another thread.

#include "rigExec/api/evaluator.h"

#include <memory>
#include <string>
#include <thread>

namespace rigExec {

struct RigExecRunStateData;

/// Use a run state from one thread at a time: the thread that created it,
/// or, after ReleaseOwner, the next thread that uses it. Run states of one
/// program may evaluate on different threads at once.
class RigExecRunState {
    struct _CreateKey {
        explicit _CreateKey(int)
        {
        }
    };

public:
    RigExecRunState(_CreateKey, std::unique_ptr<RigExecRunStateData> data);
    ~RigExecRunState();
    RigExecRunState(const RigExecRunState &) = delete;
    RigExecRunState &operator=(const RigExecRunState &) = delete;

    /// A run state of \p program with every input at its default value.
    /// Null with \p error as for RigExecEvaluator::Create.
    static std::unique_ptr<RigExecRunState>
    Create(std::shared_ptr<const RigExecProgram> program,
           const RigExecEvaluatorOptions &options = {},
           std::string *error = nullptr);

    /// Opens an edit of this run state's inputs (see inputWriter.h).
    RigExecInputWriter BeginInputEdit();
    /// As RigExecEvaluator::Evaluate, for this run state's values. Ops
    /// that keep history read the history this run state was given when
    /// it was made; evaluating adds none.
    bool Evaluate(std::string *error = nullptr);
    /// As RigExecEvaluator::GetOutput and ViewOutput.
    const void *GetOutput(RigExecOutputId output) const;
    RigExecValueView ViewOutput(RigExecOutputId output) const;
    /// A new run state that continues from this one: the same committed
    /// inputs, op state, outputs and history. Call it while no edit of
    /// this run state's inputs is open.
    std::unique_ptr<RigExecRunState> Clone() const;
    /// Lets another thread take the run state over: the next call, from
    /// any thread, makes that thread its owner.
    void ReleaseOwner();
    const RigExecProgram &GetProgram() const;

private:
    friend class RigExecEvaluator;
    /// Debug builds check calls come from the owner thread.
    void _AssertOwner() const;

    std::unique_ptr<RigExecRunStateData> _data;
    mutable std::thread::id _owner = std::this_thread::get_id();
};

} // namespace rigExec

#endif
