// RigExec imaging registry internals: the free helpers shared across the
// registry domain translation units. Internal to the rigExecImaging target;
// not installed.
#ifndef RIGEXEC_IMAGING_REGISTRY_INTERNAL_H
#define RIGEXEC_IMAGING_REGISTRY_INTERNAL_H

#include "rigExec/backgroundScheduler.h"

#include "registry.h"
#include "upstreamTable.h"

#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {

// Adds \p from's counters into \p into. Gauges (queue depth, running) are
// instantaneous, so they are added only when \p gauges is set -- a stopped
// scheduler has none. Defined in registryDiagnostics.cpp.
void _AddSchedulerStats(RigExecBackgroundSchedulerStats *into,
                        const RigExecBackgroundSchedulerStats &from,
                        bool gauges);

// The entries of \p table at or under \p roots (prim paths): a session's
// share of a chain's upstream table. The table itself when every entry is
// the session's, null when none is. Defined in registryUpstream.cpp.
std::shared_ptr<const RigExecUpstreamTable>
_UpstreamShare(const std::shared_ptr<const RigExecUpstreamTable> &table,
               const std::set<SdfPath> &roots);

// The pull window under the context's lock: the union of the sessions'
// warm ranges, else the stage's time codes. Defined in registryUpstream.cpp.
RigExecUpstreamPullWindow
_PullWindow(const std::vector<std::pair<double, double>> &ranges,
            const UsdStageRefPtr &stage, UsdTimeCode lastTime);

} // namespace rigExec

#endif
