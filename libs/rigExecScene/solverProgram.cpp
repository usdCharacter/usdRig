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
    case RigExecSolverKind::BlendPointFrames: {
        const auto *a = input.blendA, *b = input.blendB;
        if (!a) {
            if (b)
                aggregate = *b;
            return true;
        }
        if (!b) {
            aggregate = *a;
            return true;
        }
        if (s.blendRotationRejected)
            return true;
        const size_t n = a->GetSize();
        const double weight = std::min(std::max(input.blendWeight, 0.0), 1.0);
        if (n == b->GetSize() && a->rests.size() == n) {
            aggregate.frames.reserve(n);
            aggregate.rests.reserve(n);
            for (size_t k = 0; k < n; ++k) {
                const bool live = k < s.restIsLive.size() && s.restIsLive[k] &&
                                  k < s.jointRests.size();
                aggregate.frames.push_back(RigExecBlendFrames(
                    a->frames[k], b->frames[k], a->rests[k], weight,
                    RigExecRotationBlend::ShortestArc, s.scaleMode,
                    live ? &s.jointRests[k] : nullptr));
                aggregate.rests.push_back(live ? s.jointRests[k] : a->rests[k]);
            }
        }
        return true;
    }
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
