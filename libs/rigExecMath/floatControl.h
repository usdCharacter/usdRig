// The calling thread's floating-point control state, as a comparable word.
#ifndef RIGEXEC_MATH_FLOAT_CONTROL_H
#define RIGEXEC_MATH_FLOAT_CONTROL_H

#include <cstdint>

#if defined(_M_X64) || defined(_M_IX86) || defined(__SSE2__)
#include <xmmintrin.h>
#endif

namespace rigExec {

/// What a remembered floating-point result is only valid under: the rounding
/// mode, flush-to-zero and denormals-are-zero (and the exception masks) of
/// the calling thread. x86: MXCSR without its sticky status flags. aarch64:
/// FPCR. Elsewhere it cannot be read and the answer is
/// kRigExecUnknownFloatControl, which a memo must never treat as a match.
inline constexpr uint64_t kRigExecUnknownFloatControl = ~uint64_t(0);

inline uint64_t
RigExecFloatControlWord()
{
#if defined(_M_X64) || defined(_M_IX86) || defined(__SSE2__)
    return uint64_t(_mm_getcsr() & ~0x3Fu);
#elif defined(__aarch64__) && (defined(__GNUC__) || defined(__clang__))
    uint64_t fpcr;
    __asm__ volatile("mrs %0, fpcr" : "=r"(fpcr));
    return fpcr;
#else
    return kRigExecUnknownFloatControl;
#endif
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_FLOAT_CONTROL_H
