#ifndef RIGEXEC_API_TAPS_H
#define RIGEXEC_API_TAPS_H

/// Reads of node outputs on the compiled program, delivered after each
/// evaluation. USD-free.

#include "rigExec/api/rigDesc.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

struct RigExecTap {
    RigExecNodeId node = 0;
    const RigExecPortHandle *output = nullptr;
};

struct RigExecTapSet {
    std::vector<RigExecTap> taps;
};

/// Called on the evaluating thread once per tap, in tap order, after the
/// evaluation completes. \p value is valid only during the call and must
/// not be written. \p type is the registry type index.
using RigExecTapCallback = void (*)(void *context, size_t tap,
                                    const void *value, uint32_t type);

} // namespace rigExec

#endif
