// The switch every parallel region in rigExec is behind.
#ifndef RIGEXEC_API_PARALLEL_H
#define RIGEXEC_API_PARALLEL_H

namespace rigExec {

/// Whether rigExec may spread its own work across threads.
///
/// Backed by the TfEnvSetting RIGEXEC_ENABLE_PARALLEL_EVAL, default true.
/// Every parallel region this library opens -- the compile-time structure
/// digest and attribute prefetch, the Build-time resolution and bakeability
/// tasks, the parallel op-graph schedule, the per-point geometry kernels,
/// program retirement and background warming -- asks here first, so a
/// threading regression can be bisected to rigExec by one setting.
/// PXR_WORK_THREAD_LIMIT cannot answer that question: it also
/// serialises exec's own compilation and scheduling, so a run with it set
/// differs from the one being diagnosed in more ways than the one under test.
bool RigExecParallelEvaluationEnabled();

} // namespace rigExec

#endif // RIGEXEC_API_PARALLEL_H
