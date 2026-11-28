// How a provider operation's inputs fall into the typed input lists of the
// ProviderArithmetic and AffineFrame ops, which the hosts fill in the same
// order. Header-only and free of OpenUSD.
#ifndef RIGEXEC_GRAPH_PROVIDER_INPUTS_H
#define RIGEXEC_GRAPH_PROVIDER_INPUTS_H

#include "rigExecGraph/providerRecords.h"
#include "rigExecMath/affineFrameKernel.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace rigExec {
namespace eval {
namespace spaces {

/// Which typed list a provider arithmetic operation's input reads.
enum class ProviderInputClass : uint8_t {
    None,
    Matrix,
    Frame,
    Scalar,
    Vector,
    Order,
    Reset,
};

/// The class of input \p p of an arithmetic operation of \p kind; None
/// where the kind reads no such input. Avars are 11 scalars, an order and a
/// sign vector.
inline ProviderInputClass
ProviderArithmeticInputClass(RigExecProviderOpKind kind, size_t p)
{
    using C = ProviderInputClass;
    using K = RigExecProviderOpKind;
    const auto avar = [](size_t offset) {
        return offset <= 10   ? C::Scalar
               : offset == 11 ? C::Order
               : offset == 12 ? C::Vector
                              : C::None;
    };
    switch (kind) {
    case K::SpaceExpression:
        return p < 3 ? C::Matrix : C::None;
    case K::RestFrame:
        return p == 0 || p == 8   ? C::Matrix
               : p >= 1 && p <= 6 ? C::Scalar
               : p == 7           ? C::Frame
                                  : C::None;
    case K::DefaultSpace:
        return p <= 1   ? C::Frame
               : p == 2 ? C::Matrix
               : p <= 8 ? C::Scalar
                        : C::None;
    case K::FrameToSpace:
        return p == 0 ? C::Frame : C::None;
    case K::MatrixToFrame:
        return p == 0 ? C::Matrix : C::None;
    case K::PosedFrame:
        return p <= 3 ? C::Matrix : avar(p - 4);
    case K::AvarMatrix:
        return avar(p);
    case K::RelativeXform:
        return p % 2 == 0 ? C::Matrix : C::Reset;
    case K::JointMatrix:
        return p <= 1 ? C::Frame : C::None;
    default:
        return C::None;
    }
}

/// A rotation order as an Orders list holds it: three characters packed
/// low byte first; any other spelling packs to 0 and reads as no order.
inline int32_t
ProviderPackOrder(std::string_view order)
{
    if (order.size() != 3)
        return 0;
    return int32_t(uint32_t(uint8_t(order[0])) |
                   uint32_t(uint8_t(order[1])) << 8 |
                   uint32_t(uint8_t(order[2])) << 16);
}

/// Whether an arithmetic operation of \p kind yields a frame rather than a
/// matrix.
constexpr bool
ProviderArithmeticYieldsFrame(RigExecProviderOpKind kind)
{
    return kind == RigExecProviderOpKind::RestFrame ||
           kind == RigExecProviderOpKind::MatrixToFrame ||
           kind == RigExecProviderOpKind::PosedFrame;
}

/// Which typed list an affine expression's input reads.
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
/// attributes' classes by member type, then frames (relations, targets,
/// target objects).
template <class AffineMath>
AffineInputClass
AffineFrameInputClass(uint32_t kind, size_t position)
{
    using C = AffineInputClass;
    using Mat4 = typename AffineMath::Mat4;
    if (kind >= RigExecAffineFrameTypeCount ||
        position >= RigExecAffineFrameTypes[kind].attributeCount)
        return C::Frame;
    RigExecAffineFrameInputsT<AffineMath> scratch;
    C out = C::Frame;
    RigExecVisitAffineField(
        scratch, RigExecAffineFrameTypes[kind].attributes[position].field,
        [&](auto &value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, Mat4>)
                out = C::Matrix;
            else if constexpr (std::is_same_v<T, typename AffineMath::Vec3d>)
                out = C::Vector;
            else if constexpr (std::is_same_v<T, typename AffineMath::Vec3i>)
                out = C::IntVector;
            else if constexpr (std::is_same_v<T, double>)
                out = C::Double;
            else if constexpr (std::is_same_v<T, bool>)
                out = C::Bool;
            else if constexpr (std::is_same_v<T, int>)
                out = C::Int;
            else if constexpr (std::is_same_v<T, std::string>)
                out = C::Text;
            else if constexpr (std::is_same_v<T, std::vector<int>>)
                out = C::IntItems;
            else if constexpr (std::is_same_v<T, std::vector<double>>)
                out = C::DoubleItems;
            else if constexpr (std::is_same_v<T, std::vector<Mat4>>)
                out = C::MatrixItems;
        });
    return out;
}

/// OpLookup::read of a ProviderInputs lookup: which typed input list of
/// provider operation `object` the host fills from the provider values,
/// in input order, one element per input of that class the operation
/// names. Present is one flag per input, whether its value is readable as
/// its class this frame. The numbers are part of the file format.
enum class ProviderRead : uint8_t {
    Matrices = 0,
    Authoritative = 1,
    Frames = 2,
    Scalars = 3,
    Vectors = 4,
    Orders = 5,
    Resets = 6,
    Present = 7,
    AffineMatrices = 8,
    AffineDoubles = 9,
    AffineBools = 10,
    AffineInts = 11,
    AffineVectors = 12,
    AffineIntVectors = 13,
    AffineIntItems = 14,
    AffineDoubleItems = 15,
    AffineMatrixItems = 16,
    AffineItemSpans = 17,
    AffineFrames = 18,
    AffinePresent = 19,
};

/// What produces a provider value that is not an arithmetic or affine
/// operation's result: an Attribute operation, or a bridge from outside
/// the provider program. The numbers are the parts of the SpaceExpression
/// steps these producers once ran as, and part of the file format.
enum class ProviderProducer : uint8_t {
    Operation = 0,
    External = 1,
    Sampled = 2,
    Routed = 3,
    PoseFrame = 4,
};

/// The producers of a provider program's values in the order a
/// ProviderCandidates lookup and the ProviderAnswer outputs number them:
/// its operations, sampled attributes, external inputs, routed inputs, then
/// pose-frame inputs.
struct ProviderProducers {
    size_t operations = 0, sampled = 0, external = 0, routed = 0, frames = 0;

    size_t Count() const
    {
        return operations + sampled + external + routed + frames;
    }
    /// Producer \p index's part and its row in that part's table; false
    /// past the last.
    bool Of(uint64_t index, ProviderProducer *part, size_t *row) const
    {
        const std::pair<ProviderProducer, size_t> tables[] = {
            {ProviderProducer::Operation, operations},
            {ProviderProducer::Sampled, sampled},
            {ProviderProducer::External, external},
            {ProviderProducer::Routed, routed},
            {ProviderProducer::PoseFrame, frames}};
        for (const auto &[table, rows] : tables) {
            if (index < rows) {
                *part = table;
                *row = size_t(index);
                return true;
            }
            index -= rows;
        }
        return false;
    }
    /// The index of row \p row of \p part's table.
    uint64_t Index(ProviderProducer part, size_t row) const
    {
        switch (part) {
        case ProviderProducer::Operation:
            return row;
        case ProviderProducer::Sampled:
            return operations + row;
        case ProviderProducer::External:
            return operations + sampled + row;
        case ProviderProducer::Routed:
            return operations + sampled + external + row;
        case ProviderProducer::PoseFrame:
            break;
        }
        return operations + sampled + external + routed + row;
    }
};

/// OpLookup::read of a ProviderCandidates lookup: one flag per candidate
/// that can answer the producer's value this frame, in the order the
/// ProviderSelect op chooses among them. The numbers are part of the file
/// format.
enum class ProviderCandidateRead : uint8_t {
    /// It holds a value, blocked or not.
    Present = 0,
    Authoritative = 1,
    Blocked = 2,
};

} // namespace spaces
} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_GRAPH_PROVIDER_INPUTS_H
