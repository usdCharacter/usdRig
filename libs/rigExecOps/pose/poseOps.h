#ifndef RIGEXEC_OPS_POSE_POSE_OPS_H
#define RIGEXEC_OPS_POSE_POSE_OPS_H

/// The pose inputs a rig composes from: bound avar values. The kernel is
/// rigExecMath/avarScale.h.

#include "rigExecOps/library.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

struct AvarInputsPayload {
    /// The avars written, one per element of Out.
    uint64_t avarCount = 0;
    /// Per element of Values, the element of Out it writes.
    std::vector<uint32_t> targets;

    bool operator==(const AvarInputsPayload &o) const
    {
        return avarCount == o.avarCount && targets == o.targets;
    }
};

const rigExec::RigExecOpHandle &AvarInputs();

// Values (DoubleArray, one per binding, in binding order). Out (DoubleArray,
// avarCount): each avar's value, a later binding for the same avar winning.
const rigExec::RigExecPortHandle &AvarInputsValues();
const rigExec::RigExecPortHandle &AvarInputsOut();

} // namespace rigExecOps

#endif
