#ifndef RIGEXEC_API_PROGRAM_VIEW_H
#define RIGEXEC_API_PROGRAM_VIEW_H

/// Read-only view of an evaluator's standing program, for bake, export and
/// imaging. Callers pass it to the program queries that take it instead of
/// reading the program's step graph.

#include "pxr/pxr.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstddef>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecBakedProgram;

class RigExecProgramView {
public:
    RigExecProgramView() = default;
    explicit RigExecProgramView(const RigExecBakedProgram *program)
        : _program(program)
    {
    }

    explicit operator bool() const
    {
        return _program != nullptr;
    }
    /// The viewed program; valid while the evaluator keeps it standing.
    const RigExecBakedProgram *Program() const
    {
        return _program;
    }
    /// Compose clusters the program runs.
    size_t ClusterCount() const;
    /// Time of the program's last evaluation; Default before the first.
    UsdTimeCode LastTime() const;

private:
    const RigExecBakedProgram *_program = nullptr;
};

} // namespace rigExec

#endif
