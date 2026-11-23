#ifndef RIGEXEC_API_TAPS_H
#define RIGEXEC_API_TAPS_H

/// Taps let a host look at values inside a rig, such as one op's output,
/// after each evaluation, without binding them as program outputs. Use
/// them for debugging, inspection and test checks.

#include "rigExec/api/rigDesc.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

/// One value to look at: output \p output of node \p node.
struct RigExecTap {
    RigExecNodeId node = 0;
    const RigExecPortHandle *output = nullptr;
};

struct RigExecTapList {
    std::vector<RigExecTap> taps;
};

/// Receives the tapped values after each successful Evaluate, on the thread
/// that called it: one call per tap, in list order. \p tap is the tap's
/// position in the list and \p type the value's registry type index.
/// \p value is read-only and valid only during the call; copy what you
/// need to keep.
using RigExecTapCallback = void (*)(void *context, size_t tap,
                                    const void *value, uint32_t type);

} // namespace rigExec

#endif
