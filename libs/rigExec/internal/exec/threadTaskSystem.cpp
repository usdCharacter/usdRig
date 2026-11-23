#include "rigExec/api/taskSystem.h"
#include "rigExec/internal/exec/addressWait.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

namespace rigExec {

namespace {

/// Pause rounds a thread spins before it parks: between levels of one Run,
/// and between Runs.
constexpr int kSpinRounds = 128;
/// Runs in flight at once; a Run beyond this many runs on its caller.
constexpr size_t kSlotCount = 16;

uint32_t
LevelTasks(const RigExecTaskSchedule &s, size_t level)
{
    return (s.sharedStart[level + 1] - s.sharedStart[level]) +
           (s.callerStart[level + 1] - s.callerStart[level]);
}

/// The next level at or after \p level that has a task; levelCount when
/// none does.
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

/// One Run in progress; it lives on the calling thread's stack. The cursor
/// holds the current level in its high half and the next unclaimed shared
/// task in its low half; it only grows.
struct Job {
    const RigExecTaskSchedule *schedule = nullptr;
    std::atomic<uint64_t> cursor{0};
    /// Tasks of the current level not yet finished.
    std::atomic<uint32_t> pending{0};
    /// Bumped on every level change; threads waiting for a level park on it.
    std::atomic<uint32_t> epoch{0};
    std::atomic<uint32_t> sleepers{0};
    std::atomic<bool> failed{false};

    bool Done() const
    {
        return (cursor.load(std::memory_order_acquire) >> 32) >=
               schedule->levelCount;
    }

    /// Opens the first level with a task; false when there is none.
    bool Start()
    {
        const RigExecTaskSchedule &s = *schedule;
        const uint32_t level = NextBusyLevel(s, 0);
        if (level >= s.levelCount)
            return false;
        pending.store(LevelTasks(s, level), std::memory_order_relaxed);
        cursor.store(Cursor(level, s.sharedStart[level]),
                     std::memory_order_relaxed);
        return true;
    }

    bool TryClaim(uint32_t *task, uint32_t *level)
    {
        const RigExecTaskSchedule &s = *schedule;
        uint64_t c = cursor.load(std::memory_order_acquire);
        for (;;) {
            const uint32_t l = uint32_t(c >> 32);
            const uint32_t next = uint32_t(c);
            if (l >= s.levelCount || next >= s.sharedStart[l + 1])
                return false;
            if (cursor.compare_exchange_weak(c, c + 1,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
                *task = s.shared[next];
                *level = l;
                return true;
            }
        }
    }

    void Finish(uint32_t level, bool ok)
    {
        if (!ok)
            failed.store(true, std::memory_order_relaxed);
        if (pending.fetch_sub(1, std::memory_order_acq_rel) == 1)
            Advance(level);
    }

    /// Called by the thread that finished the level's last task, so no
    /// other thread touches pending or claims until the cursor moves.
    void Advance(uint32_t level)
    {
        const RigExecTaskSchedule &s = *schedule;
        const uint32_t next = failed.load(std::memory_order_relaxed)
                                  ? uint32_t(s.levelCount)
                                  : NextBusyLevel(s, size_t(level) + 1);
        if (next < s.levelCount) {
            pending.store(LevelTasks(s, next), std::memory_order_relaxed);
            cursor.store(Cursor(next, s.sharedStart[next]));
        } else {
            cursor.store(Cursor(next, 0));
        }
        epoch.fetch_add(1);
        if (sleepers.load() != 0)
            RigExecAddressWakeAll(epoch);
    }

    /// Runs tasks until the schedule is done. The caller also runs each
    /// level's caller tasks, in order, as the level opens.
    void Help(bool isCaller)
    {
        const RigExecTaskSchedule &s = *schedule;
        uint32_t callerLevel = UINT32_MAX;
        int spins = 0;
        for (;;) {
            const uint32_t level =
                uint32_t(cursor.load(std::memory_order_acquire) >> 32);
            if (level >= s.levelCount)
                return;
            if (isCaller && level != callerLevel) {
                callerLevel = level;
                for (uint32_t k = s.callerStart[level];
                     k < s.callerStart[level + 1]; ++k)
                    Finish(level, s.run(s.context, s.caller[k]));
                spins = 0;
                continue;
            }
            uint32_t task = 0, taskLevel = 0;
            if (TryClaim(&task, &taskLevel)) {
                Finish(taskLevel, s.run(s.context, task));
                spins = 0;
                continue;
            }
            if (++spins < kSpinRounds) {
                RigExecCpuRelax();
                continue;
            }
            const uint32_t seen = epoch.load();
            sleepers.fetch_add(1);
            if (uint32_t(cursor.load() >> 32) == level)
                RigExecAddressWait(epoch, seen);
            sleepers.fetch_sub(1);
            spins = 0;
        }
    }
};

/// Whether handing \p s to workers can overlap any two tasks.
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

/// Where a Run posts its job for the workers. users counts the workers
/// inside the job, so the caller keeps it alive until they leave.
struct alignas(64) Slot {
    std::atomic<bool> taken{false};
    std::atomic<Job *> job{nullptr};
    std::atomic<uint32_t> users{0};
};

/// Workers spin briefly after a Run, then sleep until the next one; each
/// Run wakes them once. Runs from several threads share the workers.
class ThreadTaskSystem final : public RigExecTaskSystem {
public:
    explicit ThreadTaskSystem(size_t threads)
        : _slots(std::make_unique<Slot[]>(kSlotCount))
    {
        for (size_t i = 1; i < threads; ++i)
            _workers.emplace_back([this] { _WorkerLoop(); });
    }

    ~ThreadTaskSystem() override
    {
        _stop.store(true);
        _posted.fetch_add(1);
        RigExecAddressWakeAll(_posted);
        for (std::thread &worker : _workers)
            worker.join();
    }

    bool Run(const RigExecTaskSchedule &schedule) override
    {
        if (_workers.empty() || !WorthSharing(schedule))
            return RigExecRunTaskScheduleInline(schedule);
        Slot *slot = _TakeSlot();
        if (!slot)
            return RigExecRunTaskScheduleInline(schedule);
        Job job;
        job.schedule = &schedule;
        if (job.Start()) {
            slot->job.store(&job);
            _posted.fetch_add(1);
            if (_parked.load() != 0)
                RigExecAddressWakeAll(_posted);
            job.Help(true);
            slot->job.store(nullptr);
            for (int spins = 0; slot->users.load() != 0; ++spins) {
                if (spins < kSpinRounds)
                    RigExecCpuRelax();
                else
                    std::this_thread::yield();
            }
        }
        slot->taken.store(false, std::memory_order_release);
        return !job.failed.load(std::memory_order_relaxed);
    }

private:
    Slot *_TakeSlot()
    {
        for (size_t i = 0; i < kSlotCount; ++i) {
            bool expected = false;
            if (!_slots[i].taken.load(std::memory_order_relaxed) &&
                _slots[i].taken.compare_exchange_strong(
                    expected, true, std::memory_order_acquire))
                return &_slots[i];
        }
        return nullptr;
    }

    /// Whether a posted job still has work; a finished job stays posted
    /// until its caller collects it.
    bool _AnyOpen()
    {
        bool open = false;
        for (size_t i = 0; i < kSlotCount && !open; ++i) {
            Slot &slot = _slots[i];
            if (!slot.job.load())
                continue;
            slot.users.fetch_add(1);
            if (Job *job = slot.job.load())
                open = !job->Done();
            slot.users.fetch_sub(1);
        }
        return open;
    }

    /// Helps every posted job that is not done; true when it found one.
    bool _HelpPosted()
    {
        bool helped = false;
        for (size_t i = 0; i < kSlotCount; ++i) {
            Slot &slot = _slots[i];
            if (!slot.job.load(std::memory_order_relaxed))
                continue;
            slot.users.fetch_add(1);
            if (Job *job = slot.job.load()) {
                if (!job->Done()) {
                    job->Help(false);
                    helped = true;
                }
            }
            slot.users.fetch_sub(1);
        }
        return helped;
    }

    void _WorkerLoop()
    {
        int spins = 0;
        while (!_stop.load(std::memory_order_acquire)) {
            if (_HelpPosted()) {
                spins = 0;
                continue;
            }
            if (++spins < kSpinRounds) {
                RigExecCpuRelax();
                continue;
            }
            const uint32_t seen = _posted.load();
            _parked.fetch_add(1);
            if (!_stop.load() && !_AnyOpen())
                RigExecAddressWait(_posted, seen);
            _parked.fetch_sub(1);
            spins = 0;
        }
    }

    std::unique_ptr<Slot[]> _slots;
    /// Bumped by every Run and by shutdown; parked workers wait on it.
    std::atomic<uint32_t> _posted{0};
    std::atomic<uint32_t> _parked{0};
    std::atomic<bool> _stop{false};
    std::vector<std::thread> _workers;
};

} // namespace

bool
RigExecRunTaskScheduleInline(const RigExecTaskSchedule &s)
{
    for (size_t level = 0; level < s.levelCount; ++level) {
        bool ok = true;
        for (uint32_t k = s.callerStart[level]; k < s.callerStart[level + 1];
             ++k)
            ok = s.run(s.context, s.caller[k]) && ok;
        for (uint32_t k = s.sharedStart[level]; k < s.sharedStart[level + 1];
             ++k)
            ok = s.run(s.context, s.shared[k]) && ok;
        if (!ok)
            return false;
    }
    return true;
}

std::unique_ptr<RigExecTaskSystem>
RigExecCreateThreadTaskSystem(size_t threads)
{
    return std::make_unique<ThreadTaskSystem>(threads ? threads : 1);
}

} // namespace rigExec
