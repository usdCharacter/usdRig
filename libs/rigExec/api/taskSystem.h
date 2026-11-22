#ifndef RIGEXEC_API_TASK_SYSTEM_H
#define RIGEXEC_API_TASK_SYSTEM_H

/// The threads an evaluator runs batches on; implemented by the host or
/// taken from RigExecCreateThreadTaskSystem. USD-free.

#include <cstddef>
#include <memory>

namespace rigExec {

class RigExecTaskSystem {
public:
    virtual ~RigExecTaskSystem() = default;
    /// Calls \p fn(context, i) once for each i in [0, count) and returns
    /// when all calls have. Calls may run concurrently in any order.
    virtual void ParallelFor(size_t count, void (*fn)(void *, size_t),
                             void *context) = 0;
};

/// A fixed pool of \p threads std::threads, the caller included; 1 runs
/// everything on the caller.
std::unique_ptr<RigExecTaskSystem>
RigExecCreateThreadTaskSystem(size_t threads);

} // namespace rigExec

#endif
