#ifndef RIGEXEC_OPS_SPACE_PROVIDER_OPS_H
#define RIGEXEC_OPS_SPACE_PROVIDER_OPS_H

/// A provider program's operations: the arithmetic kinds
/// (rigExecGraph/providerArithmetic.h) and the affine frame expressions
/// (rigExecMath/affineFrameKernel.h, sequenced by rigExecEvalCore/spaces.h).

#include "rigExecGraph/providerRecords.h"
#include "rigExecOps/library.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExecOps {

/// Which typed input list a provider operation's input reads.
enum class ProviderInputClass : uint8_t {
    None,
    Matrix,
    Frame,
    Scalar,
    Vector,
    Order,
    Reset,
};

/// The class of input \p position of an arithmetic operation of \p kind;
/// None where the kind reads no such input.
ProviderInputClass
ProviderArithmeticInputClass(rigExec::RigExecProviderOpKind kind,
                             size_t position);

/// A rotation order as the Orders input holds it: three characters packed
/// low byte first; any other spelling packs to 0 and reads as no order.
int32_t ProviderPackOrder(const std::string &order);

struct ProviderArithmeticPayload {
    rigExec::RigExecProviderOpKind kind =
        rigExec::RigExecProviderOpKind::FrameToSpace;
    bool scaleAvars = true;
    /// Per operation input, its element in the list of its class, or -1
    /// where the input is absent and the kind reads its default.
    std::vector<int32_t> slots;

    bool operator==(const ProviderArithmeticPayload &o) const
    {
        return kind == o.kind && scaleAvars == o.scaleAvars && slots == o.slots;
    }
};

const rigExec::RigExecOpHandle &ProviderArithmetic();

// Inputs, each a typed list the payload's slots index: Matrices (Mat4dArray)
// with Authoritative (BoolArray, empty or per matrix: an authored value that
// overrides its connection), Frames (PointFrameList), Scalars (DoubleArray),
// Vectors (Vec3dArray), Orders (IntArray, ProviderPackOrder) and Resets
// (BoolArray, per ancestor: its xform order resets the stack). Outputs:
// Matrix (Mat4d) for a kind that yields a matrix, Frame (PointFrame) for
// one that yields a frame, and Authoritative (Bool) of the result.
const rigExec::RigExecPortHandle &ProviderArithmeticMatrices();
const rigExec::RigExecPortHandle &ProviderArithmeticAuthoritative();
const rigExec::RigExecPortHandle &ProviderArithmeticFrames();
const rigExec::RigExecPortHandle &ProviderArithmeticScalars();
const rigExec::RigExecPortHandle &ProviderArithmeticVectors();
const rigExec::RigExecPortHandle &ProviderArithmeticOrders();
const rigExec::RigExecPortHandle &ProviderArithmeticResets();
const rigExec::RigExecPortHandle &ProviderArithmeticMatrix();
const rigExec::RigExecPortHandle &ProviderArithmeticFrame();
const rigExec::RigExecPortHandle &ProviderArithmeticAuthoritativeOut();

/// Which typed input list an affine expression's input reads.
enum class AffineInputClass : uint8_t {
    Matrix,
    Double,
    Bool,
    Int,
    Vector,
    IntVector,
    Text,
    IntItems,
    DoubleItems,
    MatrixItems,
    Frame,
};

/// The class of input \p position of affine expression \p kind: its
/// attributes' classes, then frames (relations, targets, target objects).
AffineInputClass AffineFrameInputClass(uint32_t kind, size_t position);

struct AffineFramePayload {
    /// The expression, in RigExecAffineFrameTypes order.
    uint32_t kind = 0;
    /// How many frames after the relations are a constraint's targets; the
    /// rest are its target objects.
    uint32_t targets = 0;
    /// Per expression input, its element in the list of its class (for an
    /// array, its pair in ItemSpans), or -1 where it reads its default.
    std::vector<int32_t> slots;
    /// The token values the Text slots index (uniform attributes).
    std::vector<std::string> texts;

    bool operator==(const AffineFramePayload &o) const
    {
        return kind == o.kind && targets == o.targets && slots == o.slots &&
               texts == o.texts;
    }
};

const rigExec::RigExecOpHandle &AffineFrame();

// Inputs, typed lists the payload's slots index: Matrices (Mat4dArray),
// Doubles (DoubleArray), Bools (BoolArray), Ints (IntArray), Vectors
// (Vec3dArray), IntVectors (Vec3iArray), the array attributes' items
// IntItems (IntArray), DoubleItems (DoubleArray) and MatrixItems
// (Mat4dArray) with ItemSpans (IntArray, a begin and a count per array
// slot), and Frames (PointFrameList). Out (Mat4d). An expression the
// kernel cannot evaluate is an error of the evaluation.
const rigExec::RigExecPortHandle &AffineFrameMatrices();
const rigExec::RigExecPortHandle &AffineFrameDoubles();
const rigExec::RigExecPortHandle &AffineFrameBools();
const rigExec::RigExecPortHandle &AffineFrameInts();
const rigExec::RigExecPortHandle &AffineFrameVectors();
const rigExec::RigExecPortHandle &AffineFrameIntVectors();
const rigExec::RigExecPortHandle &AffineFrameIntItems();
const rigExec::RigExecPortHandle &AffineFrameDoubleItems();
const rigExec::RigExecPortHandle &AffineFrameMatrixItems();
const rigExec::RigExecPortHandle &AffineFrameItemSpans();
const rigExec::RigExecPortHandle &AffineFrameFrames();
const rigExec::RigExecPortHandle &AffineFrameOut();

void RegisterProviderOps(rigExec::RigExecOpRegistrar &registrar);

} // namespace rigExecOps

#endif
