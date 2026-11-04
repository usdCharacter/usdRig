// Frames on demand: the frame table (frameTable.h).
#include "frameTable.h"

#include "snapshotStore.h"

#include <cmath>
#include <utility>

namespace rigExec {

int64_t
RigExecFrameTable::Key(double time)
{
    return static_cast<int64_t>(std::llround(time * 1.0e6));
}

std::shared_ptr<const RigExecImagingSnapshot>
RigExecFrameTable::Find(double time) const
{
    if (!std::isfinite(time)) {
        return nullptr;
    }
    std::shared_lock<std::shared_mutex> lock(_entriesMutex);
    const auto it = _entries.find(Key(time));
    return it == _entries.end() ? nullptr : it->second;
}

void
RigExecFrameTable::Demand(double time)
{
    if (!std::isfinite(time)) {
        return;
    }
    std::lock_guard<std::mutex> lock(_demandMutex);
    const int64_t key = Key(time);
    for (const double existing : _demand) {
        if (Key(existing) == key) {
            return;
        }
    }
    if (_demand.size() >= kMaxDemand) {
        _demand.erase(_demand.begin());
    }
    _demand.push_back(time);
    _demandCount.store(_demand.size());
}

std::vector<double>
RigExecFrameTable::TakeDemand()
{
    std::lock_guard<std::mutex> lock(_demandMutex);
    std::vector<double> out;
    out.swap(_demand);
    _demandCount.store(0);
    return out;
}

void
RigExecFrameTable::Insert(
    double time, std::shared_ptr<const RigExecImagingSnapshot> snapshot,
    double playhead)
{
    if (!std::isfinite(time) || !snapshot) {
        return;
    }
    // The replaced entries are released after the lock: a large snapshot
    // takes longer to destroy than to look up.
    std::vector<std::shared_ptr<const RigExecImagingSnapshot>> released;
    {
        std::unique_lock<std::shared_mutex> lock(_entriesMutex);
        auto &slot = _entries[Key(time)];
        released.push_back(std::move(slot));
        slot = std::move(snapshot);
        while (_entries.size() > kMaxEntries) {
            // The entry farthest from the playhead goes first.
            auto farthest = _entries.begin();
            double distance = -1.0;
            for (auto it = _entries.begin(); it != _entries.end(); ++it) {
                const double d =
                    std::abs(double(it->first) * 1.0e-6 - playhead);
                if (d > distance) {
                    distance = d;
                    farthest = it;
                }
            }
            released.push_back(std::move(farthest->second));
            _entries.erase(farthest);
        }
    }
}

bool
RigExecFrameTable::ResetIfStale(uint64_t stamp)
{
    if (_haveStamp && stamp == _stamp) {
        return false;
    }
    const bool hadStamp = _haveStamp;
    _stamp = stamp;
    _haveStamp = true;
    std::map<int64_t, std::shared_ptr<const RigExecImagingSnapshot>> released;
    {
        std::unique_lock<std::shared_mutex> lock(_entriesMutex);
        released.swap(_entries);
    }
    return hadStamp && !released.empty();
}

void
RigExecFrameTable::Clear()
{
    std::map<int64_t, std::shared_ptr<const RigExecImagingSnapshot>> released;
    {
        std::unique_lock<std::shared_mutex> lock(_entriesMutex);
        released.swap(_entries);
    }
    TakeDemand();
    _haveStamp = false;
}

size_t
RigExecFrameTable::EntryCountForTesting() const
{
    std::shared_lock<std::shared_mutex> lock(_entriesMutex);
    return _entries.size();
}

}  // namespace rigExec
