// How a pure kernel lets its host spread a per-element loop across threads.
#ifndef RIGEXEC_MATH_RANGE_RUNNER_H
#define RIGEXEC_MATH_RANGE_RUNNER_H

#include <cstddef>
#include <functional>

namespace rigExec {

/// One slice [begin, end) of a per-element loop.
using RigExecRangeBody = std::function<void(size_t begin, size_t end)>;

/// Runs \p body over [0, count), possibly as concurrent disjoint slices of at
/// least \p grain elements. rigExecMath links no thread library, so a host
/// that may use threads passes one of these; a kernel given none runs the
/// loop here.
///
/// A kernel hands a runner only loops whose elements are independent: each
/// element's arithmetic reads shared inputs and writes its own outputs, so
/// the result is bitwise the same however the range is sliced. Any
/// order-dependent sum stays outside the runner.
using RigExecRangeRunner =
    std::function<void(size_t count, size_t grain, const RigExecRangeBody &body)>;

/// \p body over [0, count) through \p runner, or here when there is none.
inline void
RigExecRunRanges(const RigExecRangeRunner *runner, size_t count, size_t grain,
                 const RigExecRangeBody &body)
{
    if (count == 0) {
        return;
    }
    if (runner && *runner) {
        (*runner)(count, grain, body);
    } else {
        body(0, count);
    }
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_RANGE_RUNNER_H
