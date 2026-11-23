// Build-tree executable completion hook. Never compile this into a DLL.
// Installed/custom observation hosts opt in by compiling this maintained file.
#include "rigExec/internal/observerHost.h"
#include "rigExec/internal/oracleDispatch.h"

#include "pxr/base/work/detachedTask.h"

#include <cstdio>
#include <cstdlib>
#include <exception>
namespace {
// OpenUSD destroys layer data on detached tasks. On Windows, ExitProcess kills
// a worker mid-destroy while it holds heap or C runtime locks that DLL teardown
// and its stdio flush then wait on: the process hangs, or ends with its
// buffered output lost.
void
WaitForDetachedWork()
{
    PXR_WORK_IMPL_NAMESPACE_USING_DIRECTIVE;
    pxr::WorkTBB_GetDetachedDispatcher().Wait();
}
// Registered first, so it runs after the observers complete.
const bool detachedWorkJoined = std::atexit(WaitForDetachedWork) == 0;
void
CompleteObservers()
{
    rigExec::RigExecFinalizeInputReplay();
    try {
        rigExec::RigExecFinalizeGoldenSuite();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "observer completion: %s\n", error.what());
        std::fflush(stderr);
        // A missing oracle library is reported and recoverable; a failure
        // of an installed suite changes the exit status.
        if (rigExec::RigExecOracleAvailable())
            std::_Exit(EXIT_FAILURE);
    }
}
const bool registered = [] {
    const char *golden = std::getenv("RIGEXEC_GOLDEN_SUITE");
    const char *inputs = std::getenv("RIGEXEC_INPUT_REPLAY");
    if ((golden && *golden) || (inputs && *inputs)) {
        // EXE-owned onexit runs before ExitProcess. DLL-owned onexit is too
        // late on Windows to guarantee a failure changes the process status.
        if (std::atexit(CompleteObservers) != 0) {
            std::fputs("observer host cannot register strict completion\n",
                       stderr);
            std::fflush(stderr);
            std::_Exit(EXIT_FAILURE);
        }
    }
    return true;
}();
} // namespace
