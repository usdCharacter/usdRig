#include "rigExec/gpuCloth.h"

#include <atomic>
#include <cstdlib>
#include <cstring>

namespace rigExec {

namespace {

// Constant-initialized: no guard stands between a reader and it.
std::atomic<RigExecGpuClothRunner> _runner{nullptr};

}  // namespace

void
RigExecSetGpuClothRunner(RigExecGpuClothRunner runner)
{
    _runner.store(runner, std::memory_order_release);
}

RigExecGpuClothRunner
RigExecGetGpuClothRunner()
{
    return _runner.load(std::memory_order_acquire);
}

bool
RigExecGpuClothEnabled()
{
    if (!RigExecGetGpuClothRunner()) return false;
    const char *value = std::getenv("RIGEXEC_GPU_CLOTH");
    if (!value || !*value) return true;
    return std::strcmp(value, "0") != 0 && std::strcmp(value, "off") != 0 &&
           std::strcmp(value, "false") != 0 && std::strcmp(value, "OFF") != 0;
}

size_t
RigExecGpuClothMinPoints()
{
    const char *value = std::getenv("RIGEXEC_GPU_CLOTH_MIN_POINTS");
    if (!value || !*value) return 2048;
    char *end = nullptr;
    const unsigned long long parsed = std::strtoull(value, &end, 10);
    return end && *end == 0 ? size_t(parsed) : size_t(2048);
}

}  // namespace rigExec
