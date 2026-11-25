#ifndef RIGEXEC_INTERNAL_EXEC_VIEW_H
#define RIGEXEC_INTERNAL_EXEC_VIEW_H

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
};

} // namespace rigExec

#endif
