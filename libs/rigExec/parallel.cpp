// RigExec parallelism switch. See parallel.h.
#include "parallel.h"

#include "pxr/base/tf/envSetting.h"
#include "pxr/base/work/loops.h"

#include <cfenv>
#if defined(_M_X64) || defined(_M_IX86) || defined(__SSE2__)
#include <xmmintrin.h>
#define RIGEXEC_HAVE_MXCSR 1
#endif

PXR_NAMESPACE_USING_DIRECTIVE

PXR_NAMESPACE_OPEN_SCOPE

TF_DEFINE_ENV_SETTING(
    RIGEXEC_ENABLE_PARALLEL_EVAL, true,
    "Whether rigExec may run its own work in parallel. Set to false to make "
    "every rigExec parallel region -- the compile-time digest and prefetch "
    "tasks, the op-graph clusters and the per-point geometry kernels -- run "
    "on the calling thread, and background warming stay off, without "
    "disturbing exec's own threading the way "
    "PXR_WORK_THREAD_LIMIT would.");

PXR_NAMESPACE_CLOSE_SCOPE

namespace rigExec {

bool
RigExecParallelEvaluationEnabled()
{
    return TfGetEnvSetting(RIGEXEC_ENABLE_PARALLEL_EVAL);
}

namespace {

// The floating-point control state a slice has to share with its caller.
// x86: MXCSR holds the SSE rounding mode, flush-to-zero and
// denormals-are-zero, which is all SSE arithmetic reads. Elsewhere the
// standard environment, which carries the rounding mode.
struct _FloatControl {
#if RIGEXEC_HAVE_MXCSR
    // The control bits only: rounding, flush-to-zero, denormals-are-zero and
    // the exception masks. The low six bits are sticky status flags, which
    // record what happened and change no result; each thread keeps its own.
    static constexpr unsigned kStatus = 0x3Fu;
    unsigned csr = _mm_getcsr() & ~kStatus;
    bool operator!=(const _FloatControl &o) const { return csr != o.csr; }
    void Apply() const { _mm_setcsr(csr | (_mm_getcsr() & kStatus)); }
#else
    std::fenv_t env;
    _FloatControl() { std::fegetenv(&env); }
    bool operator!=(const _FloatControl &) const { return true; }
    void Apply() const { std::fesetenv(&env); }
#endif
};

}  // namespace

bool
RigExecFloatEnvironmentIsDefault()
{
#if RIGEXEC_HAVE_MXCSR
    // DAZ is bit 6, the rounding control bits 13-14, FTZ bit 15.
    return (_mm_getcsr() & 0xE040u) == 0;
#elif defined(__aarch64__) && (defined(__GNUC__) || defined(__clang__))
    unsigned long long fpcr;
    __asm__ volatile("mrs %0, fpcr" : "=r"(fpcr));
    // FZ is bit 24, RMode bits 22-23.
    return (fpcr & ((1ull << 24) | (3ull << 22))) == 0;
#else
    return false;
#endif
}

void
RigExecParallelForInCallerEnvironment(size_t n, size_t grain,
                                      const RigExecRangeBody &body)
{
    const _FloatControl caller;
    WorkParallelForN(n, [&caller, &body](size_t begin, size_t end) {
        const _FloatControl own;
        const bool differs = own != caller;
        if (differs) {
            caller.Apply();
        }
        struct Restore {
            const _FloatControl &own;
            bool differs;
            ~Restore() { if (differs) own.Apply(); }
        } restore{own, differs};
        body(begin, end);
    }, grain);
}

const RigExecRangeRunner *
RigExecGeometryRanges()
{
    static const RigExecRangeRunner runner =
        [](size_t count, size_t grain, const RigExecRangeBody &body) {
            if (RigExecParallelEvaluationEnabled() &&
                !RigExecFrozenSerialActive() &&
                count >= RigExecGeometryParallelThreshold && count > grain) {
                RigExecParallelForInCallerEnvironment(count, grain, body);
            } else {
                body(0, count);
            }
        };
    return &runner;
}

}  // namespace rigExec
