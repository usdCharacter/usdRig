#ifndef RIGEXEC_API_EVALUATOR_H
#define RIGEXEC_API_EVALUATOR_H

/// Evaluates a compiled program. Owner-thread only. USD-free.

#include "rigExec/api/compile.h"
#include "rigExec/api/taps.h"
#include "rigExec/api/taskSystem.h"
#include "rigExecGraph/windowHistory.h"

#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rigExec {

struct RigExecEvaluatorState;

struct RigExecEvaluatorOptions {
    /// Borrowed; must outlive the evaluator. Null runs on the caller.
    RigExecTaskSystem *tasks = nullptr;
};

class RigExecEvaluator {
public:
    struct _CreateKey {
        explicit _CreateKey(int)
        {
        }
    };
    RigExecEvaluator(_CreateKey, std::shared_ptr<const RigExecProgram> program,
                     const RigExecEvaluatorOptions &options);
    ~RigExecEvaluator();
    RigExecEvaluator(const RigExecEvaluator &) = delete;
    RigExecEvaluator &operator=(const RigExecEvaluator &) = delete;

    static std::unique_ptr<RigExecEvaluator>
    Create(std::shared_ptr<const RigExecProgram> program,
           const RigExecEvaluatorOptions &options = {},
           std::string *error = nullptr);

    /// Copies \p value, of the bound input's type, into the input.
    bool SetInput(RigExecInputId input, const void *value,
                  std::string *error = nullptr);
    /// Runs the program. On failure no tap fires and outputs are those of
    /// the failed run.
    bool Evaluate(std::string *error = nullptr);
    /// The bound output's value; valid until the next Evaluate or
    /// SwapProgram.
    const void *GetOutput(RigExecOutputId output) const;
    /// Replaces the program between evaluations. Inputs, op state and window
    /// history reset.
    bool SwapProgram(std::shared_ptr<const RigExecProgram> program,
                     std::string *error = nullptr);
    /// Fails, keeping the previous taps, when a tap names a node or output
    /// the program lacks.
    bool SetTaps(const RigExecTapList &taps, RigExecTapCallback callback,
                 void *context, std::string *error = nullptr);

    /// History for Windowed ops, one window per windowed node in node order.
    /// A host playing in order calls PushHistory after each Evaluate that
    /// advanced a frame; a host that jumps calls ClearHistory, then, oldest
    /// first, sets each of GetHistoryDepth earlier ticks' inputs and calls
    /// EvaluateHistory.
    void PushHistory();
    void ClearHistory();
    /// Runs the windowed nodes and the nodes they depend on with the current
    /// inputs, on a run state of its own, so outputs stay as the last
    /// Evaluate left them; each window's capture becomes its newest ring
    /// entry. A windowed node that no windowed node depends on only
    /// captures. A failed tick pushes null captures and returns false.
    bool EvaluateHistory(std::string *error = nullptr);
    /// How many earlier ticks a complete history reaches back.
    size_t GetHistoryDepth() const;
    size_t GetWindowCount() const;
    /// Replaces every window's ring, oldest first, each trimmed to its
    /// newest frames; fails, changing nothing, on a count mismatch.
    bool SetWindowRings(const std::vector<RigExecWindowRing> &rings,
                        std::string *error = nullptr);
    const std::vector<RigExecWindowRing> &GetWindowRings() const;
    /// Each window's capture by the last Evaluate; null where it captured
    /// nothing or the run failed.
    std::vector<RigExecWindowPacketPtr> GetWindowCaptures() const;

private:
    bool _Bind(std::shared_ptr<const RigExecProgram> program,
               std::string *error);

    /// Debug builds check that the entry points run on the thread that
    /// created the evaluator.
    void _AssertOwnerThread() const;

    RigExecEvaluatorOptions _options;
    std::unique_ptr<RigExecEvaluatorState> _state;
    std::thread::id _ownerThread = std::this_thread::get_id();
};

} // namespace rigExec

#endif
