#ifndef RIGEXEC_INTERNAL_EXEC_RUN_STATE_DATA_H
#define RIGEXEC_INTERNAL_EXEC_RUN_STATE_DATA_H

#include "rigExec/api/runState.h"
#include "rigExec/internal/exec/inputChannel.h"
#include "rigExec/internal/exec/run.h"

#include <memory>
#include <vector>

namespace rigExec {

/// A run state's own storage: its input channel, history and run over the
/// program's shared constant arena.
struct RigExecRunStateData {
    std::shared_ptr<const RigExecProgram> program;
    RigExecEvaluatorOptions options;
    RigExecInputChannel channel;
    std::vector<RigExecWindowRing> rings;
    RigExecRun run;

    RigExecRunStateData(std::shared_ptr<const RigExecProgram> program,
                        const RigExecEvaluatorOptions &options);

    /// A run state's storage continuing from \p run, which reads
    /// \p channel and \p rings.
    static std::unique_ptr<RigExecRunStateData>
    CloneOf(std::shared_ptr<const RigExecProgram> program,
            const RigExecEvaluatorOptions &options,
            const RigExecInputChannel &channel,
            const std::vector<RigExecWindowRing> &rings, const RigExecRun &run);
};

} // namespace rigExec

#endif
