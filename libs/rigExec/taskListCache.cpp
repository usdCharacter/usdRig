// RigExec task-list cache. See taskListCache.h.
#include "taskListCache.h"

namespace rigExec {

bool
RigExecTaskListCache::Lookup(uint64_t compiledRig,
                             const RigExecControlId &control,
                             RigExecBakedClusterSet *out)
{
    if (!out) {
        std::lock_guard<std::mutex> lock(_mutex);
        ++_misses;
        return false;
    }
    // The set crosses the lock by copy; it is one or two words on any rig
    // that bakes today, so the copy is not the critical section's cost.
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _map.find(RigExecTaskListKey(compiledRig, control));
    if (found == _map.end()) {
        ++_misses;
        return false;
    }
    *out = found->second;
    ++_hits;
    return true;
}

void
RigExecTaskListCache::Store(uint64_t compiledRig,
                           const RigExecControlId &control,
                           const RigExecBakedClusterSet &set)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _map[RigExecTaskListKey(compiledRig, control)] = set;
    ++_stores;
}

size_t
RigExecTaskListCache::InvalidateCompiledRig(uint64_t compiledRig)
{
    std::lock_guard<std::mutex> lock(_mutex);
    size_t dropped = 0;
    for (auto it = _map.begin(); it != _map.end();) {
        if (it->first.first == compiledRig) {
            it = _map.erase(it);
            ++dropped;
        } else {
            ++it;
        }
    }
    return dropped;
}

void
RigExecTaskListCache::Clear()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _map.clear();
    _hits = 0;
    _misses = 0;
    _stores = 0;
}

RigExecTaskListStats
RigExecTaskListCache::Stats() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    RigExecTaskListStats stats;
    stats.hits = _hits;
    stats.misses = _misses;
    stats.stores = _stores;
    stats.entries = _map.size();
    return stats;
}

size_t
RigExecTaskListCache::Size() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _map.size();
}

}  // namespace rigExec
