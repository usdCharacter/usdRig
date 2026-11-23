#ifndef RIGEXEC_API_TASK_SYSTEM_H
#define RIGEXEC_API_TASK_SYSTEM_H

/// The worker threads an evaluator spreads independent ops across. A host
/// with its own thread pool implements RigExecTaskSystem on it; otherwise
/// RigExecCreateThreadTaskSystem makes one.

#include <cstddef>
#include <memory>

namespace rigExec {

class RigExecTaskSystem {
public:
    virtual ~RigExecTaskSystem() = default;
    /// Calls \p fn(context, i) once for each i in [0, count) and returns
    /// when all calls have returned. Calls may run at the same time, on any
    /// thread, in any order.
    virtual void ParallelFor(size_t count, void (*fn)(void *, size_t),
                             void *context) = 0;
};

/// A pool of \p threads threads, counting the calling thread; 1 runs
/// everything on the caller.
std::unique_ptr<RigExecTaskSystem>
RigExecCreateThreadTaskSystem(size_t threads);

} // namespace rigExec

#endif
