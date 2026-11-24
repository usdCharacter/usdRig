#ifndef RIGEXEC_API_TASK_SYSTEM_H
#define RIGEXEC_API_TASK_SYSTEM_H

/// The worker threads an evaluator spreads independent ops across. A host
/// with its own thread pool implements RigExecTaskSystem on it; otherwise
/// RigExecCreateThreadTaskSystem makes one.

#include <cstddef>
#include <cstdint>
#include <memory>

namespace rigExec {

/// One evaluation's work, handed over whole. Tasks are numbered by the
/// caller and grouped into levels: every task of a level finishes before
/// any task of the next level starts. Within a level, shared tasks may run
/// on any thread, and a shared task may wait inside run for shared tasks
/// listed before it in its level, so they start in the order listed: a
/// task starts only once every task listed before it has started. Caller
/// tasks run on the thread that called Run, in the order listed.
struct RigExecTaskSchedule {
    size_t levelCount = 0;
    /// Level l's shared tasks are shared[sharedStart[l]] up to, not
    /// including, shared[sharedStart[l + 1]]; levelCount + 1 entries.
    const uint32_t *sharedStart = nullptr;
    const uint32_t *shared = nullptr;
    /// Level l's caller tasks, laid out the same way.
    const uint32_t *callerStart = nullptr;
    const uint32_t *caller = nullptr;
    /// Runs one task. When a task returns false, the rest of its level
    /// still runs and later levels do not.
    bool (*run)(void *context, uint32_t task) = nullptr;
    void *context = nullptr;
};

class RigExecTaskSystem {
public:
    virtual ~RigExecTaskSystem() = default;
    /// Runs \p schedule and returns when it is finished: false when a task
    /// returned false. Several threads may call Run at once, each with its
    /// own schedule.
    virtual bool Run(const RigExecTaskSchedule &schedule) = 0;
};

/// Runs \p schedule on the calling thread alone, level by level, each
/// level's caller tasks before its shared tasks.
bool RigExecRunTaskScheduleInline(const RigExecTaskSchedule &schedule);

/// A pool of \p threads threads, counting the calling thread; 1 runs
/// everything on the caller. Idle workers sleep until the next Run.
std::unique_ptr<RigExecTaskSystem>
RigExecCreateThreadTaskSystem(size_t threads);

} // namespace rigExec

#endif
