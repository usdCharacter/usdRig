// The pose commit fold shared by the baked CommitDelta, PropagateChunk and
// CommitApply steps and the PoseCommit op: a revised joint's hierarchy delta
// carried onto the descendants it moves. Header-only; no locks, statics or
// diagnostics. Math supplies PointsToMatrix, Usable and Carry
// (RigExecPropagatePoseArithmetic).
#ifndef RIGEXEC_MATH_POSE_COMMIT_KERNEL_H
#define RIGEXEC_MATH_POSE_COMMIT_KERNEL_H

#include "poseArithmeticKernel.h"

#include <cstddef>
#include <cstdint>

namespace rigExec {

/// A present candidate's hierarchy delta, \p before to \p after: 1 when it
/// solves, 2 when it is singular.
template <class Math, class Points, class Matrix>
uint8_t
RigExecMeasureCommitDelta(const Points &before, const Points &after,
                          Matrix *delta)
{
    return Math::PointsToMatrix(before, after, delta) ? 1 : 2;
}

/// Whether a descendant's walk from \p first up to, not including,
/// \p second crosses a joint whose parent space is authored.
/// \p parentOf(slot, &authored, &next) is false for an unknown slot, which
/// clears \p *known.
template <class ParentOf>
bool
RigExecCommitParentBlocked(int first, int second, ParentOf &&parentOf,
                           bool *known)
{
    *known = true;
    for (int slot = first; slot >= 0 && slot != second;) {
        bool authored = false;
        int next = -1;
        if (!parentOf(slot, &authored, &next)) {
            *known = false;
            return false;
        }
        if (authored)
            return true;
        slot = next;
    }
    return false;
}

/// Whether a descendant pair is stepped over: blocked, or its closest
/// candidate \p pos is absent.
inline bool
RigExecCommitPairSkipped(bool blocked, int pos, const char *present,
                         size_t presentCount)
{
    return blocked || pos < 0 || size_t(pos) >= presentCount ||
           !present[size_t(pos)];
}

/// One pair that is not stepped over: \p current carried by its closest
/// candidate's delta.
template <class Math, class Frame, class Matrix>
RigExecPosePropagateOutcome
RigExecStageCommitPair(const Frame &current, const Frame &before,
                       const Frame &after, const Matrix &delta, bool deltaOk,
                       bool solverOutput, Frame *out)
{
    return RigExecPropagatePoseArithmetic<Math, Frame, Matrix>(
        current, before, after, delta, deltaOk, solverOutput, false, true, out);
}

/// The first pair that neither staged nor was stepped over, which decides
/// the commit; \p count when every pair did.
inline size_t
RigExecFirstCommitFailure(const uint8_t *outcome, size_t count)
{
    using O = RigExecPosePropagateOutcome;
    for (size_t k = 0; k < count; ++k)
        if (O(outcome[k]) != O::Staged && O(outcome[k]) != O::Skipped)
            return k;
    return count;
}

} // namespace rigExec

#endif // RIGEXEC_MATH_POSE_COMMIT_KERNEL_H
