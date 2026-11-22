#ifndef RIGEXEC_API_EVALUATOR_H
#define RIGEXEC_API_EVALUATOR_H

/// Evaluates a compiled program. Owner-thread only. USD-free.

#include "rigExec/api/compile.h"
#include "rigExec/api/taps.h"
#include "rigExec/api/taskSystem.h"

#include <memory>
#include <string>

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
    /// Replaces the program between evaluations. Inputs and op state reset.
    bool SwapProgram(std::shared_ptr<const RigExecProgram> program,
                     std::string *error = nullptr);
    /// Fails, keeping the previous taps, when a tap names a node or output
    /// the program lacks.
    bool SetTaps(const RigExecTapSet &taps, RigExecTapCallback callback,
                 void *context, std::string *error = nullptr);

private:
    bool _Bind(std::shared_ptr<const RigExecProgram> program,
               std::string *error);

    RigExecEvaluatorOptions _options;
    std::unique_ptr<RigExecEvaluatorState> _state;
};

} // namespace rigExec

#endif
