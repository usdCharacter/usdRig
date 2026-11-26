#ifndef RIGEXEC_SCENE_WORK_TASK_SYSTEM_H
#define RIGEXEC_SCENE_WORK_TASK_SYSTEM_H

/// A RigExecTaskSystem on OpenUSD's work library, so op evaluation shares
/// the TBB workers the host's OpenUSD calls run on.

#include "rigExec/api/taskSystem.h"

#include <cstddef>
#include <memory>

namespace rigExec {

/// Runs schedules on WorkDispatcher tasks; at most \p threads threads,
/// counting the calling thread, work on one Run. 0 takes OpenUSD's
/// concurrency limit, read once here; 1 runs everything on the caller.
std::unique_ptr<RigExecTaskSystem>
RigExecCreateWorkTaskSystem(size_t threads = 0);

} // namespace rigExec

#endif
