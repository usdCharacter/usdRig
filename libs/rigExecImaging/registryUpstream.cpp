// RigExec imaging upstream inputs: the chain-pulled upstream tables, each
// session's slice of one, the serial that retires warmed rows when they
// move, and the live hand-off of their values at a time. Tables arrive as
// immutable snapshots pulled outside the context lock (sceneIndices.cpp);
// nothing here reads a stage.
#include "registry.h"
#include "registryInternal.h"

#include "rigExec/frameCacheSparsity.h"

#include "upstreamTable.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {

// The entries of \p table at or under \p roots (prim paths): a session's
// share of a chain's upstream table. The table itself when every entry is
// the session's, null when none is.
std::shared_ptr<const RigExecUpstreamTable>
_UpstreamShare(const std::shared_ptr<const RigExecUpstreamTable> &table,
               const std::set<SdfPath> &roots)
{
    if (!table || table->entries.empty()) {
        return nullptr;
    }
    const auto mine = [&](const SdfPath &path) {
        const SdfPath prim = path.GetPrimPath();
        for (const SdfPath &root : roots) {
            if (prim.HasPrefix(root)) {
                return true;
            }
        }
        return false;
    };
    size_t kept = 0;
    for (const auto &[path, entry] : table->entries) {
        kept += mine(path) ? 1 : 0;
    }
    if (kept == table->entries.size()) {
        return table;
    }
    if (kept == 0) {
        return nullptr;
    }
    auto share = std::make_shared<RigExecUpstreamTable>(*table);
    for (auto it = share->entries.begin(); it != share->entries.end();) {
        it = mine(it->first) ? std::next(it) : share->entries.erase(it);
    }
    return share;
}

// The pull window under the context's lock: the union of the sessions'
// warm ranges, else the stage's time codes.
RigExecUpstreamPullWindow
_PullWindow(const std::vector<std::pair<double, double>> &ranges,
            const UsdStageRefPtr &stage, UsdTimeCode lastTime)
{
    RigExecUpstreamPullWindow window;
    bool ranged = false;
    for (const auto &[lo, hi] : ranges) {
        window.lo = ranged ? std::min(window.lo, lo) : lo;
        window.hi = ranged ? std::max(window.hi, hi) : hi;
        ranged = true;
    }
    if (stage) {
        if (!ranged) {
            window.lo = stage->GetStartTimeCode();
            window.hi = std::max(window.lo, stage->GetEndTimeCode());
        }
        window.held = stage->GetInterpolationType() == UsdInterpolationTypeHeld;
    }
    if (lastTime.IsNumeric() && std::isfinite(lastTime.GetValue())) {
        window.haveTime = true;
        window.time = lastTime.GetValue();
    }
    return window;
}

RigExecUpstreamPullWindow
RigExecImagingRegistry::GetUpstreamPullWindow()
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<std::pair<double, double>> ranges;
    for (const RigSession &session : _sessions) {
        if (session.warmRangeActive && !session.warmRange.empty()) {
            ranges.emplace_back(session.warmRange.front(),
                                session.warmRange.back());
        }
    }
    return _PullWindow(ranges, _PreviewStageLocked(), _lastTime);
}

void
RigExecImagingRegistry::SetUpstreamTable(
    const RigExecResultsSceneIndex *chain,
    std::shared_ptr<const RigExecUpstreamTable> table)
{
    if (!chain) {
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (table && !table->entries.empty()) {
        // The chain that delivered the table supplies the values; a key
        // another chain publishes differently is reported.
        for (const auto &[other, otherTable] : _upstreamTables) {
            if (other == chain || !otherTable) {
                continue;
            }
            for (const auto &[path, entry] : table->entries) {
                const auto found = otherTable->entries.find(path);
                // Tables can have different anchors. Compare the values at
                // this trigger, using stored samples only; an older table
                // outside its window cannot establish a conflict.
                RigExecUpstreamValue incoming, previous;
                if (found != otherTable->entries.end() &&
                    table->ValueAt(path, entry, table->t0, &incoming) &&
                    otherTable->ValueAt(path, found->second, table->t0,
                                        &previous) &&
                    !RigExecSameSourceValue(incoming.value, true,
                                            previous.value, true)) {
                    TF_WARN("upstream input %s: two scene-index chains "
                            "publish different values; the triggering "
                            "chain's are used",
                            path.GetText());
                }
            }
        }
        _upstreamTables[chain] = std::move(table);
    } else {
        _upstreamTables.erase(chain);
    }
    _upstreamChain = chain;
    for (RigSession &session : _sessions) {
        _UpdateSessionUpstreamLocked(session);
    }
}

void
RigExecImagingRegistry::DropUpstreamTable(const RigExecResultsSceneIndex *chain)
{
    std::lock_guard<std::mutex> lock(_mutex);
    const bool held = _upstreamTables.erase(chain) != 0;
    if (_upstreamChain == chain) {
        _upstreamChain = nullptr;
    }
    if (held) {
        for (RigSession &session : _sessions) {
            _UpdateSessionUpstreamLocked(session);
        }
    }
}

bool
RigExecImagingRegistry::RefreshUpstreamInputs()
{
    UsdTimeCode time;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_stage) {
            return false;
        }
        time = _lastTime;
    }
    // SetTime hands every session its values at that time and evaluates
    // only the sessions they moved.
    return SetTime(time);
}

std::shared_ptr<const RigExecUpstreamTable>
RigExecImagingRegistry::GetUpstreamTable(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.upstream;
        }
    }
    return nullptr;
}

uint64_t
RigExecImagingRegistry::GetUpstreamSerial(const SdfPath &rig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.upstreamSerial;
        }
    }
    return 0;
}

void
RigExecImagingRegistry::_UpdateSessionUpstreamLocked(RigSession &session)
{
    if (!session.bridge && !session.playback) {
        return;
    }
    const auto found = _upstreamTables.find(_upstreamChain);
    std::set<SdfPath> roots{session.assetRoot};
    if (!session.readRootsDirty) {
        roots.insert(session.readRoots.begin(), session.readRoots.end());
    }
    std::shared_ptr<const RigExecUpstreamTable> share = _UpstreamShare(
        found != _upstreamTables.end() ? found->second : nullptr, roots);
    const bool same = share == session.upstream ||
                      (share && session.upstream &&
                       RigExecSameUpstreamSources(*share, *session.upstream));
    session.upstream = std::move(share);
    if (!same) {
        // The sources, the window or a frame value moved: rows warmed under
        // the old ones must not serve (FindCachedKey compares no digest). A
        // playback rig never warms.
        ++session.upstreamSerial;
        if (session.bridge) {
            _CancelGenerationLocked(session.rigPath);
        }
    }
}

void
RigExecImagingRegistry::_HandOffUpstreamLocked(RigSession &session,
                                               UsdTimeCode time)
{
    if (!session.bridge && !session.playback) {
        return;
    }
    std::vector<RigExecUpstreamValue> values;
    if (session.upstream) {
        if (time.IsNumeric()) {
            session.upstream->ValuesAt(time.GetValue(), &values);
        } else {
            // At Default only uniform sources have a value.
            for (const auto &[path, entry] : session.upstream->entries) {
                if (!entry.varies) {
                    values.push_back(
                        RigExecUpstreamValue{path, entry.atT0, entry.hashAtT0});
                }
            }
        }
    }
    // A playback session applies them to the binary's inputs at its next
    // generation (RigExecBakedPlayback::SetUpstreamInputs).
    if (session.playback) {
        if (values != session.playback->GetUpstreamInputs()) {
            session.playback->SetUpstreamInputs(std::move(values));
            session.dirty = true;
        }
    } else if (values != session.bridge->GetUpstreamInputs()) {
        session.bridge->SetUpstreamInputs(std::move(values));
        session.dirty = true;
    }
}

} // namespace rigExec
