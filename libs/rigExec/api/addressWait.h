#ifndef RIGEXEC_API_ADDRESS_WAIT_H
#define RIGEXEC_API_ADDRESS_WAIT_H

/// Parks a thread on a 32-bit word until another thread changes it: Windows
/// WaitOnAddress, Linux futex, macOS __ulock. No lock is taken. The
/// primitive RigExecTaskSystem implementations park and wake workers with.

#include <atomic>
#include <cstdint>

#if defined(_MSC_VER) &&                                                       \
    (defined(_M_X64) || defined(_M_IX86) || defined(_M_ARM64))
#include <intrin.h>
#elif defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif

namespace rigExec {

static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t) &&
                  std::atomic<uint32_t>::is_always_lock_free,
              "the wait primitives take the atomic's address as a plain word");

/// Returns once \p word no longer holds \p expected, or spuriously; callers
/// recheck their condition.
void RigExecAddressWait(const std::atomic<uint32_t> &word, uint32_t expected);

void RigExecAddressWakeAll(std::atomic<uint32_t> &word);

inline void
RigExecCpuRelax()
{
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
    _mm_pause();
#elif defined(_MSC_VER) && defined(_M_ARM64)
    __yield();
#elif defined(__x86_64__) || defined(__i386__)
    _mm_pause();
#elif defined(__aarch64__) || defined(__arm__)
    __asm__ __volatile__("yield");
#endif
}

} // namespace rigExec

#endif
