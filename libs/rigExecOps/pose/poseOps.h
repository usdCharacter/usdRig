#ifndef RIGEXEC_OPS_POSE_POSE_OPS_H
#define RIGEXEC_OPS_POSE_POSE_OPS_H

/// The pose inputs a rig composes from: bound avar values, and a space
/// switch's checkpoint frame. The kernels are rigExecMath/avarScale.h and
/// rigExecEvalCore/pose.h's SpaceCheckpointMatrix.

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

struct SpaceCheckpointPayload {
    /// Per recomposed slot, outermost first: its rotation order's three
    /// axis letters in the low three bytes, 0 for the default order.
    std::vector<uint32_t> rotOrders;
    /// Per slot, the avars:rotationSign mask (RigExecRotationSignMask).
    std::vector<uint32_t> rotationSigns;
    /// Per slot, whether its scale avars are ignored.
    std::vector<uint8_t> noScale;

    bool operator==(const SpaceCheckpointPayload &o) const
    {
        return rotOrders == o.rotOrders && rotationSigns == o.rotationSigns &&
               noScale == o.noScale;
    }
};

/// The payload code of a rotation order's text: 0 unless it has three
/// letters.
uint32_t SpaceCheckpointRotOrderCode(const char *text, size_t size);

const rigExec::RigExecOpHandle &SpaceCheckpoint();

// Per recomposed slot, outermost first: Avars (DoubleArray, 11 each),
// PosedDefault, ParentDefaultInverse, ParentExpression and
// PosedAuthoredMatrix (Mat4dArray), ParentExpressionAuthored and
// PosedAuthored (BoolArray). Optional Anchor (Mat4d, the posed matrix the
// outermost slot composes under; identity when unfed). Out (Mat4d): the
// switch frame, NaN in [3][0] where a slot's frame is unusable.
const rigExec::RigExecPortHandle &SpaceCheckpointAvars();
const rigExec::RigExecPortHandle &SpaceCheckpointPosedDefault();
const rigExec::RigExecPortHandle &SpaceCheckpointParentDefaultInverse();
const rigExec::RigExecPortHandle &SpaceCheckpointParentExpression();
const rigExec::RigExecPortHandle &SpaceCheckpointParentExpressionAuthored();
const rigExec::RigExecPortHandle &SpaceCheckpointPosedAuthored();
const rigExec::RigExecPortHandle &SpaceCheckpointPosedAuthoredMatrix();
const rigExec::RigExecPortHandle &SpaceCheckpointAnchor();
const rigExec::RigExecPortHandle &SpaceCheckpointOut();

} // namespace rigExecOps

#endif
