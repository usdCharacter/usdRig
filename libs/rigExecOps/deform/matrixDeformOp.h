#ifndef RIGEXEC_OPS_DEFORM_MATRIX_DEFORM_OP_H
#define RIGEXEC_OPS_DEFORM_MATRIX_DEFORM_OP_H

/// MatrixDeform: points moved by one transform, each by its envelope weight,
/// which is folded into the movement (p' = q + w (T q - q), or a fraction of
/// the rotation when radial) rather than blended after. The kernel is
/// rigExecMath/matrixKernel.h.

#include "rigExecOps/library.h"

namespace rigExecOps {

/// What a matrix deformer is configured with at compile.
struct MatrixDeformPayload {
    /// Fractional weights take a fraction of the rotation about the
    /// transform's screw axis, not of the position.
    bool radial = false;

    bool operator==(const MatrixDeformPayload &o) const
    {
        return radial == o.radial;
    }
};

const rigExec::RigExecOpHandle &MatrixDeform();

// Points (Vec3fArray), Transform (Mat4d), Envelope (optional FloatArray,
// one finite weight in [0, 1] per point; full strength when unfed), Out.
// An unusable envelope is an error.
const rigExec::RigExecPortHandle &MatrixDeformPoints();
const rigExec::RigExecPortHandle &MatrixDeformTransform();
const rigExec::RigExecPortHandle &MatrixDeformEnvelope();
const rigExec::RigExecPortHandle &MatrixDeformOut();

} // namespace rigExecOps

#endif
