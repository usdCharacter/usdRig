// Cancellation of background evaluation. Whoever schedules background
// evaluation owns a RigExecCancellationSource and cancels it; the engine
// polls a RigExecCancellationToken taken from it.
#ifndef RIGEXEC_API_CANCELLATION_H
#define RIGEXEC_API_CANCELLATION_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

namespace rigExec {

class RigExecCancellationSource;

/// What a background evaluation polls to learn that its result is no longer
/// wanted. A token names one serial of its source and reads canceled once the
/// source has moved past it. A default token is never canceled. Polling is
/// one atomic load, safe from any thread.
class RigExecCancellationToken {
public:
    RigExecCancellationToken() = default;

    bool IsCanceled() const
    {
        return _serial && _serial->load(std::memory_order_acquire) != _expected;
    }

private:
    friend class RigExecCancellationSource;
    RigExecCancellationToken(
        std::shared_ptr<const std::atomic<uint64_t>> serial, uint64_t expected)
        : _serial(std::move(serial)), _expected(expected)
    {
    }

    std::shared_ptr<const std::atomic<uint64_t>> _serial;
    uint64_t _expected = 0;
};

/// The owner's side: a serial that starts at zero and only moves forward.
/// Copies share one serial. Cancel cancels every token taken so far.
class RigExecCancellationSource {
public:
    RigExecCancellationSource()
        : _serial(std::make_shared<std::atomic<uint64_t>>(0))
    {
    }

    uint64_t Serial() const
    {
        return _serial->load(std::memory_order_acquire);
    }

    void Cancel()
    {
        _serial->fetch_add(1, std::memory_order_acq_rel);
    }

    /// A token that stays current while the serial equals \p serial.
    RigExecCancellationToken Token(uint64_t serial) const
    {
        return RigExecCancellationToken(_serial, serial);
    }

private:
    std::shared_ptr<std::atomic<uint64_t>> _serial;
};

} // namespace rigExec

#endif // RIGEXEC_API_CANCELLATION_H
