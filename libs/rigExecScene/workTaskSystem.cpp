#include "rigExecScene/workTaskSystem.h"

#include "rigExec/internal/exec/addressWait.h"

#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/threadLimits.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

namespace rigExec {

namespace {

constexpr int kSpinRounds = 128;
/// Runs in flight at once; a Run beyond this many runs on its caller.
constexpr size_t kSlotCount = 16;

uint32_t
LevelTasks(const RigExecTaskSchedule &s, size_t level)
{
    return (s.sharedStart[level + 1] - s.sharedStart[level]) +
           (s.callerStart[level + 1] - s.callerStart[level]);
}

uint32_t
NextBusyLevel(const RigExecTaskSchedule &s, size_t level)
{
    while (level < s.levelCount && LevelTasks(s, level) == 0)
        ++level;
    return uint32_t(level);
}

uint64_t
Cursor(uint32_t level, uint32_t next)
{
    return (uint64_t(level) << 32) | next;
}

bool
WorthSharing(const RigExecTaskSchedule &s)
{
    for (size_t level = 0; level < s.levelCount; ++level) {
        const uint32_t shared = s.sharedStart[level + 1] - s.sharedStart[level];
        if (shared > 1 || (shared == 1 && LevelTasks(s, level) > 1))
            return true;
    }
    return false;
}

/// One Run in progress. The task system owns it, so a dispatcher task that
/// starts after its Run returned still finds it. The cursor holds the
/// current level in its high half and the next unclaimed shared task in
/// its low half; it only grows. A dispatcher task reads the schedule only
/// while counted in active, and Run returns only once active is zero after
/// the cursor passed the last level.
struct alignas(64) Slot {
    std::atomic<bool> taken{false};
    /// Dispatcher tasks posted for this slot that have not returned; the
    /// slot is reused only at zero.
    std::atomic<uint32_t> posted{0};
    std::atomic<uint32_t> active{0};
    const RigExecTaskSchedule *schedule = nullptr;
    uint32_t levelCount = 0;
    std::atomic<uint64_t> cursor{0};
    std::atomic<uint32_t> pending{0};
    std::atomic<uint32_t> epoch{0};
    std::atomic<uint32_t> callerParked{0};
    std::atomic<bool> failed{false};

    bool Done() const
    {
        return (cursor.load() >> 32) >= levelCount;
    }
};

class WorkTaskSystem final : public RigExecTaskSystem {
public:
    explicit WorkTaskSystem(size_t helpers)
        : _helpers(helpers), _slots(std::make_unique<Slot[]>(kSlotCount))
    {
    }

    bool Run(const RigExecTaskSchedule &schedule) override
    {
        if (_helpers == 0 || !WorthSharing(schedule))
            return RigExecRunTaskScheduleInline(schedule);
        Slot *slot = _TakeSlot();
        if (!slot)
            return RigExecRunTaskScheduleInline(schedule);
        slot->schedule = &schedule;
        slot->levelCount = uint32_t(schedule.levelCount);
        slot->failed.store(false, std::memory_order_relaxed);
        const uint32_t level = NextBusyLevel(schedule, 0);
        if (level < schedule.levelCount) {
            slot->pending.store(LevelTasks(schedule, level),
                                std::memory_order_relaxed);
            const uint32_t helpers = _HelpersFor(schedule, level);
            slot->cursor.store(Cursor(level, schedule.sharedStart[level]));
            _Post(slot, helpers);
            _Help(slot, true);
            for (int spins = 0; slot->active.load() != 0; ++spins) {
                if (spins < kSpinRounds)
                    RigExecCpuRelax();
                else
                    std::this_thread::yield();
            }
        }
        const bool ok = !slot->failed.load(std::memory_order_relaxed);
        slot->schedule = nullptr;
        slot->taken.store(false, std::memory_order_release);
        return ok;
    }

private:
    Slot *_TakeSlot()
    {
        for (size_t i = 0; i < kSlotCount; ++i) {
            Slot &slot = _slots[i];
            bool expected = false;
            if (slot.posted.load() == 0 &&
                !slot.taken.load(std::memory_order_relaxed) &&
                slot.taken.compare_exchange_strong(expected, true))
                return &slot;
        }
        return nullptr;
    }

    /// One helper per shared task the caller will not take itself, up to
    /// the thread budget.
    uint32_t _HelpersFor(const RigExecTaskSchedule &s, uint32_t level) const
    {
        const uint32_t shared = s.sharedStart[level + 1] - s.sharedStart[level];
        const uint32_t callers =
            s.callerStart[level + 1] - s.callerStart[level];
        const uint32_t wanted = callers || shared == 0 ? shared : shared - 1;
        return uint32_t(std::min<size_t>(wanted, _helpers));
    }

    void _Post(Slot *slot, uint32_t helpers)
    {
        for (uint32_t i = 0; i < helpers; ++i) {
            slot->posted.fetch_add(1);
            _dispatcher.Run([this, slot] {
                slot->active.fetch_add(1);
                if (!slot->Done())
                    _Help(slot, false);
                slot->active.fetch_sub(1);
                slot->posted.fetch_sub(1);
            });
        }
    }

    static bool _TryClaim(Slot *slot, uint32_t *task, uint32_t *level)
    {
        const RigExecTaskSchedule &s = *slot->schedule;
        uint64_t c = slot->cursor.load(std::memory_order_acquire);
        for (;;) {
            const uint32_t l = uint32_t(c >> 32);
            const uint32_t next = uint32_t(c);
            if (l >= s.levelCount || next >= s.sharedStart[l + 1])
                return false;
            if (slot->cursor.compare_exchange_weak(c, c + 1,
                                                   std::memory_order_acq_rel,
                                                   std::memory_order_acquire)) {
                *task = s.shared[next];
                *level = l;
                return true;
            }
        }
    }

    void _Finish(Slot *slot, uint32_t level, bool ok)
    {
        if (!ok)
            slot->failed.store(true, std::memory_order_relaxed);
        if (slot->pending.fetch_sub(1, std::memory_order_acq_rel) == 1)
            _Advance(slot, level);
    }

    /// Called by the thread that finished the level's last task.
    void _Advance(Slot *slot, uint32_t level)
    {
        const RigExecTaskSchedule &s = *slot->schedule;
        const uint32_t next = slot->failed.load(std::memory_order_relaxed)
                                  ? uint32_t(s.levelCount)
                                  : NextBusyLevel(s, size_t(level) + 1);
        uint32_t helpers = 0;
        if (next < s.levelCount) {
            slot->pending.store(LevelTasks(s, next), std::memory_order_relaxed);
            helpers = _HelpersFor(s, next);
            slot->cursor.store(Cursor(next, s.sharedStart[next]));
        } else {
            slot->cursor.store(Cursor(next, 0));
        }
        slot->epoch.fetch_add(1);
        if (slot->callerParked.load() != 0)
            RigExecAddressWakeAll(slot->epoch);
        _Post(slot, helpers);
    }

    /// The caller runs each level's caller tasks as the level opens and
    /// stays until the schedule is done, parking between levels. A helper
    /// leaves once its level has nothing left to claim, and each new level
    /// posts its own helpers, so no work thread blocks waiting for a level.
    void _Help(Slot *slot, bool isCaller)
    {
        const RigExecTaskSchedule &s = *slot->schedule;
        uint32_t callerLevel = UINT32_MAX;
        int spins = 0;
        for (;;) {
            const uint32_t level =
                uint32_t(slot->cursor.load(std::memory_order_acquire) >> 32);
            if (level >= s.levelCount)
                return;
            if (isCaller && level != callerLevel) {
                callerLevel = level;
                for (uint32_t k = s.callerStart[level];
                     k < s.callerStart[level + 1]; ++k)
                    _Finish(slot, level, s.run(s.context, s.caller[k]));
                spins = 0;
                continue;
            }
            uint32_t task = 0, taskLevel = 0;
            if (_TryClaim(slot, &task, &taskLevel)) {
                _Finish(slot, taskLevel, s.run(s.context, task));
                spins = 0;
                continue;
            }
            if (!isCaller)
                return;
            if (++spins < kSpinRounds) {
                RigExecCpuRelax();
                continue;
            }
            const uint32_t seen = slot->epoch.load();
            slot->callerParked.fetch_add(1);
            if (uint32_t(slot->cursor.load() >> 32) == level)
                RigExecAddressWait(slot->epoch, seen);
            slot->callerParked.fetch_sub(1);
            spins = 0;
        }
    }

    const size_t _helpers;
    std::unique_ptr<Slot[]> _slots;
    /// Declared after the slots: its destructor waits for posted helpers,
    /// which read their slot.
    PXR_NS::WorkDispatcher _dispatcher;
};

} // namespace

std::unique_ptr<RigExecTaskSystem>
RigExecCreateWorkTaskSystem(size_t threads)
{
    if (threads == 0)
        threads = std::max<size_t>(1, PXR_NS::WorkGetConcurrencyLimit());
    return std::make_unique<WorkTaskSystem>(threads - 1);
}

} // namespace rigExec
