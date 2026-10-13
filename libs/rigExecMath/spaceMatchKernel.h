// Shared, USD-free arithmetic of space matching: the translate and rotate
// channels that keep a space-switched control's posed frame where it is once
// its switch selects another space. The native program and the binary
// runtime call it from their space-switch step with their own switch
// arithmetic, so they agree by construction.
//
// Conventions as ikFkMatchKernel.h: 16 doubles, row-major, row vectors; a
// control's posed frame is A * P with A its avar matrix. The switch's axis
// masks and rotation filters make P depend on A, so the solve repeats
// P = A^-1 * posed until the frame lands.
#ifndef RIGEXEC_MATH_SPACE_MATCH_KERNEL_H
#define RIGEXEC_MATH_SPACE_MATCH_KERNEL_H

#include "ikFkMatchKernel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rigExec {

/// Values per space in RigExecSpaceMatchSolveAll's output: tx ty tz rx ry rz.
constexpr size_t RigExecSpaceMatchStride = 6;

/// \p ch, starting from the control's current channels, solved so that
/// run(avars, active, posed) -- the backend's space switch with its current
/// sources -- puts the control on \p target with \p active selected. Scale
/// and spin are kept. False when the switch cannot run or a solve is
/// singular; true otherwise, with the closest channels a few passes reach.
template <class Run>
bool
RigExecSpaceMatchSolve(const double target[16], double active,
                       RigExecIkFkMatchChannels *ch, Run &&run)
{
    namespace k = ikFkMatchKernel;
    const k::M4 goal = k::Load(target);
    // Without masks or filters P does not depend on A, and the second pass
    // only confirms the first.
    constexpr int kPasses = 4;
    constexpr double kSettled = 1e-9;
    for (int pass = 0; pass < kPasses; ++pass) {
        const k::M4 avars = k::Compose(*ch);
        double posed[16];
        if (!run(avars.data(), active, posed)) return false;
        double error = 0.0;
        for (int i = 0; i < 16; ++i) {
            error = std::max(error, std::abs(posed[i] - target[i]));
        }
        if (error < kSettled) return true;
        k::M4 inverse;
        if (!k::Inverse(avars, &inverse)) return false;
        if (!k::Solve(goal, k::Mul(inverse, k::Load(posed)), ch)) return false;
    }
    return true;
}

/// One row per space index 0..spaces-1 into \p out (spaces * stride): the
/// channels that keep the control on \p target with that space selected.
/// The current space's row is the current channels, give or take a solve.
template <class Run>
bool
RigExecSpaceMatchSolveAll(const double target[16],
                          const RigExecIkFkMatchChannels &current,
                          size_t spaces, Run &&run, double *out)
{
    for (size_t space = 0; space < spaces; ++space) {
        RigExecIkFkMatchChannels ch = current;
        if (!RigExecSpaceMatchSolve(target, double(space), &ch, run)) {
            return false;
        }
        double *row = out + space * RigExecSpaceMatchStride;
        for (int k = 0; k < 3; ++k) {
            row[k] = ch.t[k];
            row[3 + k] = ch.r[k];
        }
    }
    return true;
}

}  // namespace rigExec

#endif
