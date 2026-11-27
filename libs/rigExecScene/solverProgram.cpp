#include "solverProgram.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExec/api/types.h"
#include "pxr/base/gf/math.h"
#include <algorithm>
namespace rigExec {
bool
RigExecRunSolver(const RigExecSolverRecord &s, const RigExecSolverInputs &input,
                 RigExecSolverWorkspace *scratch,
                 RigExecPointFrameArray *output, std::string *error)
{
    if (!scratch || !output) {
        if (error)
            *error = "null solver workspace/output";
        return false;
    }
    auto &aggregate = *output;
    aggregate.frames.clear();
    aggregate.rests.clear();
    if (s.degenerate)
        return true;
    switch (s.kind) {
    case RigExecSolverKind::FkChain:
        if (error)
            *error = "an FK chain runs as op nodes";
        return false;
    case RigExecSolverKind::TwoBoneIk:
        if (error)
            *error = "a two-bone IK runs as an op node";
        return false;
    case RigExecSolverKind::BlendPointFrames:
        if (error)
            *error = "a frame blend runs as an op node";
        return false;
    case RigExecSolverKind::TwistDistribution:
        if (error)
            *error = "a twist distribution runs as an op node";
        return false;
    case RigExecSolverKind::Ribbon:
        if (error)
            *error = "a ribbon runs as an op node";
        return false;
    case RigExecSolverKind::SplineIk:
        if (error)
            *error = "a spline IK runs as an op node";
        return false;
    case RigExecSolverKind::Skeleton2d:
        if (error)
            *error = "a 2D skeleton runs as an op node";
        return false;
    }
    if (error)
        *error = "unknown solver kind";
    return false;
}
} // namespace rigExec
