#ifndef RIGEXEC_OPS_DEFORM_MATRIX_DEFORM_OP_H
#define RIGEXEC_OPS_DEFORM_MATRIX_DEFORM_OP_H

/// MatrixDeform: points moved by one transform, each by its envelope weight,
/// which is folded into the movement (p' = q + w (T q - q), or a fraction of
/// the rotation when radial) rather than blended after. The movement is
/// rigExecEvalCore/envelope.h's matrix kernel, the one every evaluator runs.

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
// A sparse field with a zero default comes instead as WeightIndices
// (IntArray, ascending point indices) and Weights (FloatArray, one weight
// per index) and is walked entry by entry. An unusable envelope is an
// error. A mover's own inputs come as Enabled (Bool, optional; false
// passes the points through) and DefaultWeight (Float, optional; every
// point's weight, full strength at 1; finite and in [0, 1] or an error, and
// not beside an envelope or sparse weights). A weight object's packet comes
// as Packet (PointWeights, optional; walked sparsely, at full strength or
// resolved densely each frame exactly as the matrix kernel reads a packet;
// one that does not resolve is an error, and not beside another envelope).
// Radial (Bool, optional) stands in for the payload's radial each frame.
// Written (IntArray, optional) lists the [begin, end) point ranges of the
// vertex groups the revision writes when it does not write them all: an
// enabled frame whose weights are not a sparse walk inside them is an
// error.
const rigExec::RigExecPortHandle &MatrixDeformPoints();
const rigExec::RigExecPortHandle &MatrixDeformTransform();
const rigExec::RigExecPortHandle &MatrixDeformEnvelope();
const rigExec::RigExecPortHandle &MatrixDeformWeightIndices();
const rigExec::RigExecPortHandle &MatrixDeformWeights();
const rigExec::RigExecPortHandle &MatrixDeformOut();
const rigExec::RigExecPortHandle &MatrixDeformEnabled();
const rigExec::RigExecPortHandle &MatrixDeformDefaultWeight();
const rigExec::RigExecPortHandle &MatrixDeformPacket();
const rigExec::RigExecPortHandle &MatrixDeformRadial();
const rigExec::RigExecPortHandle &MatrixDeformWritten();

/// What a mover transform is configured with at compile.
struct MoverTransformPayload {
    /// rigExec:pointFrame is "posed": the points were already carried, so a
    /// measured transform is conjugated into their frame.
    bool posedPoints = false;

    bool operator==(const MoverTransformPayload &o) const
    {
        return posedPoints == o.posedPoints;
    }
};

/// A mover's transform in its geometry's point frame
/// (rigExecMath/moverTransformKernel.h).
const rigExec::RigExecOpHandle &MoverTransform();

// Transform (Mat4d). Optional Mat4d inputs, each only when the mover names
// it: Space (rigExec:transformSpace), Reference (the transform's neutral),
// ReferenceSpace (the space's neutral; needs Space and Reference) and Carry
// (rigExec:space). Out (Mat4d).
const rigExec::RigExecPortHandle &MoverTransformTransform();
const rigExec::RigExecPortHandle &MoverTransformSpace();
const rigExec::RigExecPortHandle &MoverTransformReference();
const rigExec::RigExecPortHandle &MoverTransformReferenceSpace();
const rigExec::RigExecPortHandle &MoverTransformCarry();
const rigExec::RigExecPortHandle &MoverTransformOut();

} // namespace rigExecOps

#endif
