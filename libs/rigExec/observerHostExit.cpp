// Build-tree executable completion hook. Never compile this into a DLL.
// Installed/custom observation hosts opt in by compiling this maintained file.
#include "observerHost.h"
#include <cstdio>
#include <cstdlib>
#include <exception>
namespace {
void CompleteObservers() {
    rigExec::RigExecFinalizeInputReplay();
    try {
        rigExec::RigExecFinalizeGoldenSuite();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "observer completion: %s\n", error.what());
    }
}
const bool registered = [] {
    const char *golden = std::getenv("RIGEXEC_GOLDEN_SUITE");
    const char *inputs = std::getenv("RIGEXEC_INPUT_REPLAY");
    if ((golden && *golden) || (inputs && *inputs)) {
        // EXE-owned onexit runs before ExitProcess. DLL-owned onexit is too
        // late on Windows to guarantee a failure changes the process status.
        if (std::atexit(CompleteObservers) != 0) {
            std::fputs("observer host cannot register strict completion\n", stderr);
            std::fflush(stderr);std::_Exit(EXIT_FAILURE);
        }
    }
    return true;
}();
}
