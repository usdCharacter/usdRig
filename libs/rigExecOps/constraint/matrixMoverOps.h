#ifndef RIGEXEC_OPS_CONSTRAINT_MATRIX_MOVER_OPS_H
#define RIGEXEC_OPS_CONSTRAINT_MATRIX_MOVER_OPS_H

/// The MatrixMover's revision of one transform target, and a geometry-domain
/// constraint's delta. The arithmetic is rigExecEvalCore/pose.h's
/// MatrixMoverCandidate and MeasureConstraintDelta.

#include "rigExecOps/library.h"

namespace rigExecOps {

struct MatrixMoverPayload {
    /// Blend by a partial transform (true) or move each landmark linearly
    /// toward its transformed place (false).
    bool radialBlend = false;

    bool operator==(const MatrixMoverPayload &o) const
    {
        return radialBlend == o.radialBlend;
    }
};

const rigExec::RigExecOpHandle &MatrixMover();
const rigExec::RigExecOpHandle &ConstraintDelta();

// MatrixMover: In (PointFrame, the target entering), Transform (PointFrame,
// the rigExec:transform source), optional TransformRest (PointFrame: its
// rest; unfed or not valid, the identity landmarks), optional Space and
// SpaceRest (PointFrame, the rigExec:transformSpace source and its rest,
// fed together), Weight (Double) and Out (PointFrame, flagged as In). A
// source frame that is not valid or maps to no matrix gives In flagged
// degenerate; a non-finite input is an error.
const rigExec::RigExecPortHandle &MatrixMoverIn();
const rigExec::RigExecPortHandle &MatrixMoverTransform();
const rigExec::RigExecPortHandle &MatrixMoverTransformRest();
const rigExec::RigExecPortHandle &MatrixMoverSpace();
const rigExec::RigExecPortHandle &MatrixMoverSpaceRest();
const rigExec::RigExecPortHandle &MatrixMoverWeight();
const rigExec::RigExecPortHandle &MatrixMoverOut();

// ConstraintDelta: Solved (PointFrame, the constraint's frame at full
// weight), Base (Mat4d, the target's authored transform), BaseOk (Bool,
// whether that transform was read), Weight (Double) and Delta (Mat4d, Base
// to Solved) and Measured (Bool). A weight of zero or less is the identity.
// A degenerate frame, an unread or singular base measures nothing: the
// identity, not Measured. A non-finite frame or base is an error.
const rigExec::RigExecPortHandle &ConstraintDeltaSolved();
const rigExec::RigExecPortHandle &ConstraintDeltaBase();
const rigExec::RigExecPortHandle &ConstraintDeltaBaseOk();
const rigExec::RigExecPortHandle &ConstraintDeltaWeight();
const rigExec::RigExecPortHandle &ConstraintDeltaDelta();
const rigExec::RigExecPortHandle &ConstraintDeltaMeasured();

} // namespace rigExecOps

#endif
