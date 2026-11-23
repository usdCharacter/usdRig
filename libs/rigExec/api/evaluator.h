#ifndef RIGEXEC_API_EVALUATOR_H
#define RIGEXEC_API_EVALUATOR_H

/// Runs a compiled program. A host creates an evaluator for a program, writes
/// its inputs, calls Evaluate once per frame and reads its outputs.

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
    /// Worker threads that ops with no dependency on each other can run on.
    /// The evaluator borrows them, so they must outlive it. Leave null to
    /// run everything on the calling thread.
    RigExecTaskSystem *tasks = nullptr;
};

/// Holds everything one run of a program needs besides the program itself:
/// input and output values, each op's private state and the history of
/// earlier frames. Several evaluators can run the same program at once,
/// each with its own values. Use an evaluator only from the thread that
/// created it.
class RigExecEvaluator {
    struct _CreateKey {
        explicit _CreateKey(int)
        {
        }
    };

public:
    RigExecEvaluator(_CreateKey, std::shared_ptr<const RigExecProgram> program,
                     const RigExecEvaluatorOptions &options);
    ~RigExecEvaluator();
    RigExecEvaluator(const RigExecEvaluator &) = delete;
    RigExecEvaluator &operator=(const RigExecEvaluator &) = delete;

    /// Null with \p error when \p program is null or was compiled against
    /// a different set of registered types and ops than this process has.
    static std::unique_ptr<RigExecEvaluator>
    Create(std::shared_ptr<const RigExecProgram> program,
           const RigExecEvaluatorOptions &options = {},
           std::string *error = nullptr);

    /// Sets one of the values the host feeds the program, such as a control
    /// value for this frame. \p value must point to a value of the type the
    /// input was declared with; it is copied, so it can be released after
    /// the call. The value stays until it is set again.
    bool SetInput(RigExecInputId input, const void *value,
                  std::string *error = nullptr);
    /// Runs every op of the program once, using the current inputs. If an
    /// op fails, Evaluate returns false with that op's message, no tap
    /// callback runs, and outputs may hold partial results.
    bool Evaluate(std::string *error = nullptr);
    /// One of the values the host reads back, as computed by the last
    /// Evaluate. The pointer stays valid until the next Evaluate or
    /// SwapProgram; null for an unknown output.
    const void *GetOutput(RigExecOutputId output) const;
    /// Switches to another compiled program, for example after the rig
    /// was edited. Call it between evaluations. Inputs, op state and the
    /// history of earlier frames start over; taps are kept and fail the
    /// swap if the new program lacks a tapped node or output.
    bool SwapProgram(std::shared_ptr<const RigExecProgram> program,
                     std::string *error = nullptr);
    /// Asks to see intermediate values, such as one op's output inside the
    /// rig, after each successful Evaluate (see taps.h). Replaces any
    /// earlier taps. Fails, keeping the earlier taps, when a tap names a
    /// node or output the program does not have.
    bool SetTaps(const RigExecTapList &taps, RigExecTapCallback callback,
                 void *context, std::string *error = nullptr);

    /// Some ops, such as secondary motion and cloth, compute the current
    /// frame from what their inputs were on earlier frames. The evaluator
    /// keeps that history: one window of recent frames per such op, in
    /// node order.
    ///
    /// When playing forward one frame at a time, call PushHistory after
    /// each Evaluate. After a jump to another frame, call ClearHistory,
    /// then rebuild the history from the oldest frame forward: for each of
    /// the GetHistoryDepth frames before the new one, set that frame's
    /// inputs and call EvaluateHistory.
    void PushHistory();
    void ClearHistory();
    /// Adds one earlier frame to the history. It runs only the ops that
    /// keep history, and the ops they depend on, with the current inputs,
    /// and leaves the outputs of the last Evaluate untouched. Returns false
    /// if one of those ops fails; that frame is then recorded as missing.
    bool EvaluateHistory(std::string *error = nullptr);
    /// How many earlier frames must be replayed with EvaluateHistory for
    /// the history to be complete; 0 when no op keeps history.
    size_t GetHistoryDepth() const;
    /// How many ops keep history.
    size_t GetWindowCount() const;
    /// Replaces the history with one saved earlier, for example from
    /// GetWindowRings on another evaluator: one ring per window, oldest
    /// frame first. Longer rings keep their newest frames. Fails, changing
    /// nothing, when the number of rings differs from GetWindowCount.
    bool SetWindowRings(const std::vector<RigExecWindowRing> &rings,
                        std::string *error = nullptr);
    /// The current history, one ring per window, oldest frame first.
    const std::vector<RigExecWindowRing> &GetWindowRings() const;
    /// What each window recorded during the last Evaluate; null for a
    /// window that recorded nothing or when that Evaluate failed.
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
