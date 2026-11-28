#ifndef RIGEXEC_INTERNAL_EXEC_VIEW_H
#define RIGEXEC_INTERNAL_EXEC_VIEW_H

#include "rigExec/api/frameView.h"
#include "rigExec/api/view.h"
#include "rigExec/internal/exec/run.h"

namespace rigExec {

/// Makes views of a run's published values.
struct RigExecViewAccess {
    /// A pinned view of bound output \p output's current value in \p run;
    /// empty for an output the program does not have.
    static RigExecValueView Output(const RigExecRun &run,
                                   RigExecOutputId output,
                                   std::string *error = nullptr);
    /// Every bound output of \p run, pinned; invalid after a failed frame.
    static RigExecFrameView Frame(const RigExecRun &run, std::string *error);
    /// A view of \p data, which \p pins counts and \p owner keeps alive.
    static RigExecValueView Lend(std::shared_ptr<const void> owner,
                                 std::atomic<uint32_t> *pins, const void *data,
                                 const RigExecTypeHandle *type,
                                 uint64_t generation);
};

} // namespace rigExec

#endif
