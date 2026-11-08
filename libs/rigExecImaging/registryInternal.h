// RigExec imaging registry internals: the free helpers shared across the
// registry domain translation units. Internal to the rigExecImaging target;
// not installed.
#ifndef RIGEXEC_IMAGING_REGISTRY_INTERNAL_H
#define RIGEXEC_IMAGING_REGISTRY_INTERNAL_H

#include "rigExec/backgroundScheduler.h"

namespace rigExec {

// Adds \p from's counters into \p into. Gauges (queue depth, running) are
// instantaneous, so they are added only when \p gauges is set -- a stopped
// scheduler has none. Defined in registryDiagnostics.cpp.
void _AddSchedulerStats(RigExecBackgroundSchedulerStats *into,
                        const RigExecBackgroundSchedulerStats &from,
                        bool gauges);

} // namespace rigExec

#endif
