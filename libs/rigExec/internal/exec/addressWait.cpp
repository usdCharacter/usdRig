#include "rigExec/internal/exec/addressWait.h"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__linux__)
#include <climits>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>
#elif defined(__APPLE__)
extern "C" int __ulock_wait(uint32_t operation, void *addr, uint64_t value,
                            uint32_t timeoutMicroseconds);
extern "C" int __ulock_wake(uint32_t operation, void *addr, uint64_t wakeValue);
#else
#error "no wait-on-address primitive for this platform"
#endif

namespace rigExec {

namespace {

#if defined(__APPLE__)
// From xnu's sys/ulock.h, which the SDK does not ship.
constexpr uint32_t kUlCompareAndWait = 1;
constexpr uint32_t kUlfWakeAll = 0x00000100;
constexpr uint32_t kUlfNoErrno = 0x01000000;
#endif

uint32_t *
Word(const std::atomic<uint32_t> &word)
{
    return reinterpret_cast<uint32_t *>(
        const_cast<std::atomic<uint32_t> *>(&word));
}

} // namespace

void
RigExecAddressWait(const std::atomic<uint32_t> &word, uint32_t expected)
{
#if defined(_WIN32)
    WaitOnAddress(Word(word), &expected, sizeof(expected), INFINITE);
#elif defined(__linux__)
    syscall(SYS_futex, Word(word), FUTEX_WAIT_PRIVATE, expected, nullptr,
            nullptr, 0);
#elif defined(__APPLE__)
    __ulock_wait(kUlCompareAndWait | kUlfNoErrno, Word(word), expected, 0);
#endif
}

void
RigExecAddressWakeAll(std::atomic<uint32_t> &word)
{
#if defined(_WIN32)
    WakeByAddressAll(Word(word));
#elif defined(__linux__)
    syscall(SYS_futex, Word(word), FUTEX_WAKE_PRIVATE, INT_MAX, nullptr,
            nullptr, 0);
#elif defined(__APPLE__)
    __ulock_wake(kUlCompareAndWait | kUlfWakeAll | kUlfNoErrno, Word(word), 0);
#endif
}

} // namespace rigExec
