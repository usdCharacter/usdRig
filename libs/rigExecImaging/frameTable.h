// Frames on demand (docs/specs/frames-on-demand.md): the rig's evaluated
// output at frames other than the displayed one, for any Hydra reader that
// samples a rig output at a time offset (onion skins, export, pickers).
//
// One table per imaging context. Each entry is an UNPUBLISHED combined
// snapshot of one absolute time, built on the owner (UI) thread from the
// frame cache by RigExecImagingRegistry::ServiceFrameDemand; nothing here
// evaluates. Readers on any thread (Storm's sync threads included) look a
// time up; a miss is recorded as demand and answered "not yet". The owner
// turns demand into warm requests for the background workers and, once a
// frame is cached, into an entry plus PrimsDirtied for the rig's outputs.
//
// Thread safety: Find, Demand and HasDemand are safe on any thread; every
// other member is owner-thread only. Entries are immutable once inserted.
#ifndef RIGEXEC_IMAGING_FRAME_TABLE_H
#define RIGEXEC_IMAGING_FRAME_TABLE_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>

namespace rigExec {

struct RigExecImagingSnapshot;

class RigExecFrameTable {
public:
    /// Most entries kept; inserting past it drops the entry farthest from
    /// the playhead the owner passes.
    static constexpr size_t kMaxEntries = 128;
    /// Most outstanding demanded frames; older demand is dropped first.
    static constexpr size_t kMaxDemand = 256;

    /// The snapshot of \p time, or null. Any thread.
    std::shared_ptr<const RigExecImagingSnapshot> Find(double time) const;

    /// Records that a reader asked for \p time and found nothing. Any
    /// thread; cheap (one leaf mutex).
    void Demand(double time);

    /// True while demand waits for the owner. Any thread (one atomic load).
    bool HasDemand() const { return _demandCount.load() != 0; }

    /// The demanded times, oldest first, and clears them. Owner thread.
    std::vector<double> TakeDemand();

    /// Adds or replaces \p time's entry. Owner thread.
    void Insert(double time,
                std::shared_ptr<const RigExecImagingSnapshot> snapshot,
                double playhead);

    /// True when \p time has an entry. Owner thread (or any thread).
    bool Contains(double time) const { return bool(Find(time)); }

    /// Drops every entry when \p stamp differs from the current one (the
    /// rig, its inputs or the stage changed). Returns true when it did.
    /// Demand is kept. Owner thread.
    bool ResetIfStale(uint64_t stamp);

    /// Drops every entry and every demand. Owner thread.
    void Clear();

    size_t EntryCountForTesting() const;
    uint64_t Stamp() const { return _stamp; }

    /// The table key of \p time: exact to 1e-6 frames, so a reader's
    /// base time plus a float offset finds the owner's entry.
    static int64_t Key(double time);

private:
    mutable std::shared_mutex _entriesMutex;
    std::map<int64_t, std::shared_ptr<const RigExecImagingSnapshot>> _entries;

    std::mutex _demandMutex;
    std::vector<double> _demand;
    std::atomic<size_t> _demandCount{0};

    uint64_t _stamp = 0;
    bool _haveStamp = false;
};

using RigExecFrameTablePtr = std::shared_ptr<RigExecFrameTable>;

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_FRAME_TABLE_H
