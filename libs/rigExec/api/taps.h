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

/// A tapped value, lent to the callback: `data` points at the value inside
/// the evaluator, and `type` names its registered type. The pointer is
/// valid only during the callback; copy what you need to keep.
struct RigExecTapValue {
    const void *data = nullptr;
    const RigExecTypeHandle *type = nullptr;

    /// The value as a \p T when it has type \p expected; null when it has
    /// another type.
    template <class T> const T *As(const RigExecTypeHandle &expected) const
    {
        return type == &expected ? static_cast<const T *>(data) : nullptr;
    }
};

/// Receives the tapped values after each successful Evaluate, once all of
/// the frame's ops have run. It is called on the thread that called
/// Evaluate, once per tap, in list order; \p tap is the tap's position in
/// the list.
using RigExecTapCallback = void (*)(void *context, size_t tap,
                                    const RigExecTapValue &value);

} // namespace rigExec

#endif
