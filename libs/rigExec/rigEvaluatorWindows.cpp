// History for windowed operations (rigExecGraph/windowHistory.h).
//
// A windowed operation's result at time t reads its own inputs at the
// earlier ticks t-1 .. t-N. Those ticks are generations of the same compiled
// graph at those times, run on a frozen clone of the program the evaluator
// owns (the history lane), masked to the ops the windowed ops depend on.
// Inputs are sampled from the stage at each tick without the interactive
// overrides, so a drag moves the current frame and the earlier frames stay
// the authored animation. The lane's retained state is its own, so the live
// program's change tracking is untouched and a drag stays incremental.
//
// The memo holds each tick's captures while the stage, the program and the
// upstream inputs stand; any of them moving clears it. A tick whose inner
// windowed ops had their full history is marked complete, and only complete
// captures feed a ring the target generation reads, so a result never
// depends on where an earlier pass happened to start.

#include "frozenContext.h"
#include "rigEvaluator.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace rigExec {

struct RigExecRigEvaluator::_WindowHistory {
    std::shared_ptr<const RigExecFrozenProgram> snapshot;
    std::unique_ptr<RigExecFrozenWorkspace> workspace;
    /// The live program the snapshot was frozen from, and what it held.
    const RigExecBakedProgram *program = nullptr;
    size_t programBuild = 0;
    uint64_t avarRegion = 0;
    /// What the memo's ticks were evaluated under.
    uint64_t stageSerial = 0;
    std::vector<RigExecValueOverride> upstream;
    uint32_t flags = 0;
    std::vector<RigExecBakedProgram::WindowedOpInfo> ops;
    /// The chain bindings every tick samples through, bound once while the
    /// memo stands (a stage notice clears both).
    RigExecChainSampleBindings bindings;
    bool bound = false;

    struct Tick {
        std::vector<RigExecWindowPacketPtr> captures;
        /// Per windowed op: its capture read its inner ops' full history.
        std::vector<char> complete;
    };
    std::map<double, Tick> ticks;
    size_t tickRuns = 0;
    /// The time GetWindowRings last answered, and per windowed op whether
    /// the inner rings a generation there reads were complete.
    double placedTime = 0.0;
    bool placed = false;
    std::vector<char> placedInnerComplete;
};

namespace {

uint32_t
_FrozenFlags(bool weightFields, bool guides)
{
    return (weightFields ? kRigExecFrozenPublishWeightFields : 0u) |
           (guides ? kRigExecFrozenSolverGuidesEnabled : 0u);
}

}  // namespace

size_t
RigExecRigEvaluator::GetHistoryTickCountForTesting() const
{
    return _windowHistory ? _windowHistory->tickRuns : 0;
}

bool
RigExecRigEvaluator::GetWindowRings(UsdTimeCode time,
                                    std::vector<RigExecWindowRing> *rings,
                                    std::string *error) const
{
    const auto fail = [error](const std::string &why) {
        if (error) *error = why;
        return false;
    };
    if (!rings) return fail("no ring destination");
    rings->clear();
    const RigExecBakedProgram *live = _bakedProgram.get();
    if (!live || !live->GetWindowedOpCount()) return true;
    const size_t windowCount = live->GetWindowedOpCount();
    rings->assign(windowCount, RigExecWindowRing());
    // The default time has no earlier frames: every window is empty.
    if (time.IsDefault()) return true;

    if (!_windowHistory) _windowHistory = std::make_shared<_WindowHistory>();
    _WindowHistory &h = *_windowHistory;
    const uint32_t flags = _FrozenFlags(_publishWeightFields, _solverGuidesEnabled);
    const bool programMoved =
        !h.snapshot || h.program != live || h.programBuild != _bakedProgramBuilds ||
        RigExecFrozenSnapshotOwesLiveEdits(*h.snapshot, *live) ||
        RigExecFrozenAvarRegionDigest(*live) != h.avarRegion;
    if (programMoved || h.stageSerial != _stageEditSerial ||
        h.upstream != _upstreamAdmitted || h.flags != flags) {
        h.ticks.clear();
        h.bindings = RigExecChainSampleBindings();
        h.bound = false;
    }
    if (programMoved) {
        std::shared_ptr<const RigExecFrozenProgram> frozen;
        std::string why;
        h.snapshot.reset();
        h.workspace.reset();
        if (!RigExecFreezeProgram(*this, &frozen, &why) || !frozen) {
            return fail("windowed history unavailable: " + why);
        }
        h.snapshot = std::move(frozen);
        h.workspace = RigExecCreateFrozenWorkspace(h.snapshot);
        h.program = live;
        h.programBuild = _bakedProgramBuilds;
        h.avarRegion = RigExecFrozenAvarRegionDigest(*live);
        h.ops = live->GetWindowedOps();
    }
    h.stageSerial = _stageEditSerial;
    h.upstream = _upstreamAdmitted;
    h.flags = flags;
    const std::vector<RigExecBakedProgram::WindowedOpInfo> &ops = h.ops;
    if (ops.size() != windowCount) return fail("windowed history is out of date");

    // Which earlier ticks each op needs complete captures at, as offsets
    // from `time`: [1, frames] for every op; an inner op Y of X needs its
    // own window at each tick X needs, [lo + 1, hi + frames(Y)].
    std::vector<uint32_t> lo(windowCount, 1), hi(windowCount, 0);
    for (size_t w = 0; w < windowCount; ++w) hi[w] = ops[w].frames;
    for (bool moved = true; moved;) {
        moved = false;
        for (size_t x = 0; x < windowCount; ++x) {
            if (hi[x] < lo[x]) continue;
            for (const uint32_t y : ops[x].inner) {
                if (y >= windowCount) continue;
                const uint32_t needLo = lo[x] + 1;
                const uint32_t needHi = hi[x] + ops[y].frames;
                if (hi[y] < lo[y]) {
                    lo[y] = needLo; hi[y] = needHi; moved = true;
                    continue;
                }
                if (needLo < lo[y]) { lo[y] = needLo; moved = true; }
                if (needHi > hi[y]) { hi[y] = needHi; moved = true; }
            }
        }
    }
    uint32_t deepest = 0;
    for (size_t w = 0; w < windowCount; ++w) deepest = std::max(deepest, hi[w]);
    const double t = time.GetValue();
    const auto key = [t](uint32_t offset) { return t - double(offset); };
    const auto completeAt = [&h](double tau, size_t w) {
        const auto found = h.ticks.find(tau);
        return found != h.ticks.end() && w < found->second.complete.size() &&
               found->second.complete[w];
    };
    // The inner op Y's ring at tick tau: its captures at the frames before
    // it, and whether every one of them is complete.
    const auto ringAt = [&](double tau, size_t y, RigExecWindowRing *ring) {
        ring->clear();
        bool complete = true;
        for (uint32_t k = ops[y].frames; k >= 1; --k) {
            const auto found = h.ticks.find(tau - double(k));
            RigExecWindowPacketPtr packet;
            if (found != h.ticks.end() && y < found->second.captures.size()) {
                packet = found->second.captures[y];
                complete = complete && y < found->second.complete.size() &&
                           found->second.complete[y];
            } else {
                complete = false;
            }
            ring->push_back(std::move(packet));
        }
        return complete;
    };

    RigExecFrozenEvalContext context;
    context.compiledRigSignature = GetCompiledRigSignature();
    context.slotCount = live->GetProviderCount();
    context.programDigest =
        context.compiledRigSignature ^
        (uint64_t(live->GetBoundInputCount()) * 1099511628211ull) ^
        (uint64_t(live->GetVaryingInputCount()) * 16777619ull);
    context.flags = flags;
    context.frozen = h.snapshot.get();
    context.workspace = h.workspace.get();
    const std::vector<RigExecUpstreamValue> upstream =
        RigExecUpstreamValuesOf(_upstreamAdmitted);
    const auto bind = [&](std::string *why) {
        if (h.bound) return true;
        if (!RigExecBindChainSampleInputs(*this, &h.bindings, why)) return false;
        h.bound = true;
        return true;
    };

    std::string failure;
    // Oldest first, so an inner op's earlier captures exist before a tick
    // that reads them.
    for (uint32_t offset = deepest; offset >= 1; --offset) {
        const double tau = key(offset);
        bool needed = false;
        for (size_t w = 0; w < windowCount && !needed; ++w)
            needed = offset >= lo[w] && offset <= hi[w] && !completeAt(tau, w);
        if (!needed) continue;
        std::vector<RigExecWindowRing> innerRings(windowCount);
        std::vector<char> innerComplete(windowCount, 1);
        for (size_t y = 0; y < windowCount; ++y)
            if (ops[y].feedsWindow) innerComplete[y] = ringAt(tau, y, &innerRings[y]) ? 1 : 0;
        RigExecFrameInputs inputs;
        std::string why;
        std::vector<RigExecWindowPacketPtr> captures;
        bool ran = bind(&why) &&
                   RigExecSampleHistoryTickInputs(*this, UsdTimeCode(tau), upstream,
                                                  h.bindings, &inputs, &why);
        if (ran) {
            context.varyingInputCount = inputs.values.size();
            ran = RigExecRunFrozenHistoryTick(context, inputs, innerRings,
                                              &captures, &why);
        }
        ++h.tickRuns;
        if (!ran && failure.empty()) failure = why;
        _WindowHistory::Tick &tick = h.ticks[tau];
        tick.captures.resize(windowCount);
        tick.complete.resize(windowCount, 0);
        for (size_t w = 0; w < windowCount; ++w) {
            if (tick.complete[w]) continue;  // never downgrade a complete one
            bool complete = true;
            for (const uint32_t y : ops[w].inner)
                complete = complete && y < windowCount && innerComplete[y];
            // A declined tick holds nothing, and holds it completely: the
            // stage answers the same way every time it is asked.
            tick.captures[w] = ran && w < captures.size() ? captures[w]
                                                          : RigExecWindowPacketPtr();
            tick.complete[w] = complete ? 1 : 0;
        }
    }
    for (size_t w = 0; w < windowCount; ++w) {
        RigExecWindowRing &ring = (*rings)[w];
        ring.clear();
        for (uint32_t k = ops[w].frames; k >= 1; --k) {
            const auto found = h.ticks.find(key(k));
            ring.push_back(found != h.ticks.end() && w < found->second.captures.size()
                               ? found->second.captures[w]
                               : RigExecWindowPacketPtr());
        }
    }
    // What a generation at this time reads for its inner ops, so its own
    // captures can join the memo (_NoteWindowCaptures).
    h.placed = true;
    h.placedTime = t;
    h.placedInnerComplete.assign(windowCount, 1);
    for (size_t y = 0; y < windowCount; ++y) {
        if (!ops[y].feedsWindow) continue;
        RigExecWindowRing scratch;
        h.placedInnerComplete[y] = ringAt(t, y, &scratch) ? 1 : 0;
    }
    // Keep the memo near the time asked: a scrub back and forth reuses it,
    // a jump across the timeline does not grow it without bound.
    const double reach = double(4 * deepest + 64);
    for (auto it = h.ticks.begin(); it != h.ticks.end();) {
        if (std::fabs(it->first - t) > reach) it = h.ticks.erase(it);
        else ++it;
    }
    if (!failure.empty()) return fail("a windowed history tick declined: " + failure);
    return true;
}

void
RigExecRigEvaluator::_NoteWindowCaptures(UsdTimeCode time)
{
    // Only a generation of the authored stage is history: a drag's frame is
    // not what the stage holds at that time.
    if (!_windowHistory || time.IsDefault() || !_interactiveOverrides.empty() ||
        !_bakedProgram || !_bakedProgram->GetWindowedOpCount()) {
        return;
    }
    _WindowHistory &h = *_windowHistory;
    if (h.program != _bakedProgram.get() || h.programBuild != _bakedProgramBuilds ||
        h.stageSerial != _stageEditSerial || h.upstream != _upstreamAdmitted ||
        h.ops.size() != _bakedProgram->GetWindowedOpCount()) {
        return;
    }
    const double t = time.GetValue();
    if (!h.placed || h.placedTime != t ||
        h.placedInnerComplete.size() != h.ops.size()) {
        return;
    }
    const std::vector<RigExecWindowPacketPtr> captures =
        _bakedProgram->GetWindowCaptures();
    if (captures.size() != h.ops.size()) return;
    // A capture is complete when the inner rings the generation read were.
    _WindowHistory::Tick &tick = h.ticks[t];
    tick.captures.resize(captures.size());
    tick.complete.resize(captures.size(), 0);
    for (size_t w = 0; w < captures.size(); ++w) {
        if (tick.complete[w]) continue;
        bool complete = true;
        for (const uint32_t y : h.ops[w].inner)
            complete = complete && y < captures.size() && h.placedInnerComplete[y];
        if (!complete) continue;
        tick.captures[w] = captures[w];
        tick.complete[w] = 1;
    }
}

}  // namespace rigExec
