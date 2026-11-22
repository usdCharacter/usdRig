#include "rigExec/api/taskSystem.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace rigExec {

namespace {

/// Workers sleep between calls; the caller works too and waits for the
/// rest. One ParallelFor at a time. The mutex is a coordination lock taken
/// only to hand out and finish a call; ops never run under it.
class ThreadTaskSystem final : public RigExecTaskSystem {
public:
    explicit ThreadTaskSystem(size_t threads)
    {
        for (size_t i = 1; i < threads; ++i)
            _workers.emplace_back([this] { _WorkerLoop(); });
    }

    ~ThreadTaskSystem() override
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _stop = true;
        }
        _wake.notify_all();
        for (std::thread &worker : _workers)
            worker.join();
    }

    void ParallelFor(size_t count, void (*fn)(void *, size_t),
                     void *context) override
    {
        if (_workers.empty() || count < 2) {
            for (size_t i = 0; i < count; ++i)
                fn(context, i);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _fn = fn;
            _context = context;
            _count = count;
            _next.store(0, std::memory_order_relaxed);
            _active = _workers.size();
            ++_generation;
        }
        _wake.notify_all();
        _Drain();
        std::unique_lock<std::mutex> lock(_mutex);
        _done.wait(lock, [this] { return _active == 0; });
    }

private:
    void _Drain()
    {
        for (size_t i = _next.fetch_add(1); i < _count; i = _next.fetch_add(1))
            _fn(_context, i);
    }

    void _WorkerLoop()
    {
        uint64_t seen = 0;
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _wake.wait(lock, [&] { return _stop || _generation != seen; });
                if (_stop)
                    return;
                seen = _generation;
            }
            _Drain();
            {
                std::lock_guard<std::mutex> lock(_mutex);
                --_active;
            }
            _done.notify_one();
        }
    }

    std::vector<std::thread> _workers;
    std::mutex _mutex;
    std::condition_variable _wake;
    std::condition_variable _done;
    void (*_fn)(void *, size_t) = nullptr;
    void *_context = nullptr;
    size_t _count = 0;
    std::atomic<size_t> _next{0};
    size_t _active = 0;
    uint64_t _generation = 0;
    bool _stop = false;
};

} // namespace

std::unique_ptr<RigExecTaskSystem>
RigExecCreateThreadTaskSystem(size_t threads)
{
    return std::make_unique<ThreadTaskSystem>(threads ? threads : 1);
}

} // namespace rigExec
