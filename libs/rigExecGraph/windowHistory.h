// History windows for windowed operations: secondary motion and cloth, whose
// result at the current frame reads their own inputs over a fixed number of
// earlier frames (docs/concepts/secondary-motion.md).
//
// A windowed operation's body captures a packet at every tick it runs: the
// inputs its solve reads, nothing computed from earlier ticks. The packets of
// the earlier ticks travel to the body as a ring, oldest first, placed before
// the generation like any sampled leaf. Earlier ticks are generations of the
// same compiled graph at those times, masked to the ops the windowed ops
// depend on (RigExecOpCallbacks::active), so history is produced by the one
// loop, never beside it. USD-free: the runtime keeps rings of its own ticks.
#ifndef RIGEXEC_GRAPH_WINDOW_HISTORY_H
#define RIGEXEC_GRAPH_WINDOW_HISTORY_H

#include "opGraph.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

namespace rigExec {

/// What one windowed operation captured at one tick. Immutable once
/// published; copies share it.
struct RigExecWindowPacket {
    /// Small values: a harmonics driver position, a cloth's parameters.
    std::vector<double> scalars;
    /// Large values: a cloth's entering and collider points.
    std::vector<float> points;

    bool operator==(const RigExecWindowPacket &o) const
    {
        // Bitwise, so a NaN payload compares equal to itself and -0 differs
        // from +0: a ring is unchanged only when its bits are.
        return scalars.size() == o.scalars.size() &&
               points.size() == o.points.size() &&
               (scalars.empty() ||
                std::memcmp(scalars.data(), o.scalars.data(),
                            scalars.size() * sizeof(double)) == 0) &&
               (points.empty() ||
                std::memcmp(points.data(), o.points.data(),
                            points.size() * sizeof(float)) == 0);
    }
    bool operator!=(const RigExecWindowPacket &o) const { return !(*this == o); }
};

/// Null when the operation captured nothing at that tick (an unusable
/// input): the history before it is not read.
using RigExecWindowPacketPtr = std::shared_ptr<const RigExecWindowPacket>;

/// The earlier ticks' packets, oldest first; the last entry is the tick
/// just before the current one.
using RigExecWindowRing = std::vector<RigExecWindowPacketPtr>;

/// Whether two packets hold the same bits (null equals null).
inline bool
RigExecWindowPacketsEqual(const RigExecWindowPacketPtr &a,
                          const RigExecWindowPacketPtr &b)
{
    if (a == b) return true;
    if (!a || !b) return false;
    return *a == *b;
}

/// Whether two rings hold the same packets, tick by tick.
inline bool
RigExecWindowRingsEqual(const RigExecWindowRing &a, const RigExecWindowRing &b)
{
    if (a.size() != b.size()) return false;
    for (size_t k = 0; k < a.size(); ++k)
        if (!RigExecWindowPacketsEqual(a[k], b[k])) return false;
    return true;
}

/// What a windowed operation's body is handed for one run: the earlier
/// ticks' packets, where its capture goes (written only by that body), and
/// whether the run only captures (a history tick that needs no result from
/// it). \p cache is a slot the body owns across runs for an epoch-fixed
/// build its solve reuses; anything held there must be answer-preserving.
struct RigExecWindowRun {
    const RigExecWindowRing *ring = nullptr;
    RigExecWindowPacketPtr *capture = nullptr;
    bool captureOnly = false;
    std::shared_ptr<const void> *cache = nullptr;
    /// Set when the solve runs at the publication boundary instead (a
    /// device solve): the body captures, fills it with the ticks it would
    /// solve over, oldest first, and passes its points through.
    std::vector<RigExecWindowPacketPtr> *deferred = nullptr;
    /// The boundary's run of a deferred solve: these ticks, no capture, on
    /// the device first when `device` is set.
    const std::vector<RigExecWindowPacketPtr> *solveTicks = nullptr;
    bool device = false;
};

/// What the compiled graph says about a program's windowed operations, the
/// same for every evaluator: which windowed ops each one depends on, which
/// ones another depends on, and the canonical-op mask of a history tick.
struct RigExecWindowLayout {
    /// Per windowed op, the windowed ops among its ancestors, ascending.
    std::vector<std::vector<uint32_t>> inner;
    /// Per windowed op, whether some windowed op depends on its result (so
    /// it solves at history ticks too).
    std::vector<char> feedsWindow;
    /// Per canonical op: 1 for the windowed ops and every op one depends on.
    std::vector<char> mask;
};

/// Settles \p out from \p graph's predecessors and each windowed op's
/// canonical op in \p ops (UINT32_MAX for an op the graph set aside, which
/// joins no mask). Linear in the ops each windowed op depends on.
inline void
RigExecSettleWindowLayout(const RigExecCompiledGraph &graph,
                          const std::vector<uint32_t> &ops,
                          RigExecWindowLayout *out)
{
    const size_t opCount = graph.ops.size();
    out->inner.assign(ops.size(), {});
    out->feedsWindow.assign(ops.size(), 0);
    out->mask.assign(opCount, 0);
    std::vector<uint32_t> windowOfOp(opCount, UINT32_MAX);
    for (size_t w = 0; w < ops.size(); ++w)
        if (ops[w] < opCount) windowOfOp[ops[w]] = uint32_t(w);
    std::vector<char> seen;
    std::vector<uint32_t> stack;
    for (size_t w = 0; w < ops.size(); ++w) {
        if (ops[w] >= opCount) continue;
        seen.assign(opCount, 0);
        stack.assign(1, ops[w]);
        seen[ops[w]] = 1;
        while (!stack.empty()) {
            const uint32_t c = stack.back();
            stack.pop_back();
            out->mask[c] = 1;
            for (const uint32_t p : graph.ops[c].predecessors) {
                if (p >= opCount || seen[p]) continue;
                seen[p] = 1;
                stack.push_back(p);
                if (windowOfOp[p] != UINT32_MAX) out->inner[w].push_back(windowOfOp[p]);
            }
        }
        std::sort(out->inner[w].begin(), out->inner[w].end());
    }
    for (const auto &inner : out->inner)
        for (const uint32_t y : inner) out->feedsWindow[y] = 1;
}

/// How many earlier ticks a full history reaches back: every windowed op
/// reads its own `frames`, and an inner op's captures at each of those
/// ticks read its own window in turn. \p frames and \p inner are per
/// windowed op (RigExecWindowLayout::inner).
inline uint32_t
RigExecWindowHistoryDepth(const std::vector<uint32_t> &frames,
                          const std::vector<std::vector<uint32_t>> &inner)
{
    const size_t n = frames.size();
    std::vector<uint32_t> lo(n, 1), hi(frames);
    for (bool moved = true; moved;) {
        moved = false;
        for (size_t x = 0; x < n && x < inner.size(); ++x) {
            if (hi[x] < lo[x]) continue;
            for (const uint32_t y : inner[x]) {
                if (y >= n) continue;
                const uint32_t needLo = lo[x] + 1, needHi = hi[x] + frames[y];
                if (needLo < lo[y]) { lo[y] = needLo; moved = true; }
                if (needHi > hi[y]) { hi[y] = needHi; moved = true; }
            }
        }
    }
    uint32_t deepest = 0;
    for (const uint32_t h : hi) deepest = std::max(deepest, h);
    return deepest;
}

/// The packets a solve reads: the usable tail of \p ring (after its last
/// null entry) followed by \p current. Pointers borrow the inputs.
inline void
RigExecWindowUsableTail(const RigExecWindowRing &ring,
                        const RigExecWindowPacket *current,
                        std::vector<const RigExecWindowPacket *> *out)
{
    out->clear();
    size_t begin = ring.size();
    while (begin > 0 && ring[begin - 1]) --begin;
    for (size_t k = begin; k < ring.size(); ++k) out->push_back(ring[k].get());
    if (current) out->push_back(current);
}

}  // namespace rigExec

#endif  // RIGEXEC_GRAPH_WINDOW_HISTORY_H
