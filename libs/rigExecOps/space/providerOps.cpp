#include "rigExecOps/space/providerOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/spaces.h"
#include "rigExecGraph/providerArithmetic.h"
#include "rigExecMath/affineFrameKernel.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/affineMath.h"

#include <array>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(ProviderArithmetic)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticMatrices)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticAuthoritative)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticFrames)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticScalars)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticVectors)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticOrders)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticResets)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticMatrix)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticFrame)
RIGEXEC_DEFINE_PORT_HANDLE(ProviderArithmeticAuthoritativeOut)

RIGEXEC_DEFINE_OP_HANDLE(AffineFrame)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameMatrices)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameDoubles)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameBools)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameInts)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameVectors)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameIntVectors)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameIntItems)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameDoubleItems)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameMatrixItems)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameItemSpans)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameFrames)
RIGEXEC_DEFINE_PORT_HANDLE(AffineFrameOut)

using rigExec::RigExecProviderOpKind;

namespace {

ProviderInputClass
AvarClass(size_t offset)
{
    using C = ProviderInputClass;
    return offset <= 10   ? C::Scalar
           : offset == 11 ? C::Order
           : offset == 12 ? C::Vector
                          : C::None;
}

} // namespace

ProviderInputClass
ProviderArithmeticInputClass(RigExecProviderOpKind kind, size_t p)
{
    using C = ProviderInputClass;
    using K = RigExecProviderOpKind;
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
        return p <= 3 ? C::Matrix : AvarClass(p - 4);
    case K::AvarMatrix:
        return AvarClass(p);
    case K::RelativeXform:
        return p % 2 == 0 ? C::Matrix : C::Reset;
    case K::JointMatrix:
        return p <= 1 ? C::Frame : C::None;
    default:
        return C::None;
    }
}

int32_t
ProviderPackOrder(const std::string &order)
{
    if (order.size() != 3)
        return 0;
    return int32_t(uint32_t(uint8_t(order[0])) |
                   uint32_t(uint8_t(order[1])) << 8 |
                   uint32_t(uint8_t(order[2])) << 16);
}

namespace {

AffineInputClass
FieldClass(rigExec::RigExecAffineField field)
{
    using C = AffineInputClass;
    using rigExecStd::Mat4d;
    rigExec::RrAffineFrameInputs scratch;
    C out = C::Frame;
    rigExec::RigExecVisitAffineField(scratch, field, [&](auto &value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, Mat4d>)
            out = C::Matrix;
        else if constexpr (std::is_same_v<T, rigExecStd::Vec3d>)
            out = C::Vector;
        else if constexpr (std::is_same_v<T, rigExecStd::Vec3i>)
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
        else if constexpr (std::is_same_v<T, std::vector<Mat4d>>)
            out = C::MatrixItems;
    });
    return out;
}

} // namespace

AffineInputClass
AffineFrameInputClass(uint32_t kind, size_t position)
{
    if (kind < rigExec::RigExecAffineFrameTypeCount &&
        position < rigExec::RigExecAffineFrameTypes[kind].attributeCount)
        return FieldClass(
            rigExec::RigExecAffineFrameTypes[kind].attributes[position].field);
    return AffineInputClass::Frame;
}

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecStd::Mat3d;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3d;
using rigExecStd::Vec3i;
using namespace opIo;
using namespace fieldIo;

/// A provider operation's inputs as the shared runners address them: input
/// i is i where its slot is set, absent otherwise.
struct InputIds {
    rigExec::RigExecFieldView<const int32_t> slots;
    size_t size() const
    {
        return slots.Size();
    }
    uint64_t operator[](size_t i) const
    {
        return slots.data[i] >= 0 ? uint64_t(i) : UINT64_MAX;
    }
};

template <class T>
const T *
Element(const RigExecArray<T> *list, int32_t slot)
{
    return list && slot >= 0 && size_t(slot) < list->Size()
               ? &(*list)[size_t(slot)]
               : nullptr;
}

/// Whether \p slot can name an element of \p port's list: fed, and inside
/// it where validate knows its count.
bool
SlotFits(const rigExec::RigExecOpCheck &check,
         const rigExec::RigExecPortHandle &port, size_t slot)
{
    const size_t n = CheckElements(check, port);
    return n != rigExec::kRigExecPortUnfed &&
           (n > rigExec::kRigExecPortUnfed || slot < n);
}

// ---- ProviderArithmetic ----

enum ArithmeticField : uint32_t { kKind, kScaleAvars, kSlotCount, kSlots };

bool
ArithmeticKind(RigExecProviderOpKind kind)
{
    using K = RigExecProviderOpKind;
    return kind == K::SpaceExpression || kind == K::RestFrame ||
           kind == K::DefaultSpace || kind == K::FrameToSpace ||
           kind == K::MatrixToFrame || kind == K::PosedFrame ||
           kind == K::JointMatrix || kind == K::AvarMatrix ||
           kind == K::RelativeXform;
}

struct ArithmeticFrame {
    rigExecRig::PointLandmarks points;
    uint32_t flags = rigExecRig::kPointFrameValid;
    bool IsValid() const
    {
        return (flags & rigExecRig::kPointFrameValid) != 0;
    }
    bool IsDegenerate() const
    {
        return (flags & rigExecRig::kPointFrameDegenerate) != 0;
    }
};

/// What RigExecRunProviderArithmetic needs of a math binding.
struct ArithmeticMath {
    using Matrix = Mat4d;
    using Vector = Vec3d;
    using Rotation = rigExec::RrRotation;
    using Frame = ArithmeticFrame;
    static bool PointsToMatrix(const std::array<Vector, 4> &rest,
                               const std::array<Vector, 4> &pose, Matrix *out)
    {
        return rigExec::RigExecPointsToMatrixT<Mat3d>(rest, pose, out);
    }
};

struct ArithmeticOp {
    RigExecProviderOpKind kind;
    uint64_t output = 0;
    InputIds inputs;
    bool scaleAvars = true;
};

/// RigExecRunProviderArithmetic's store over the node's typed lists; a read
/// the lists cannot answer stops the evaluation.
struct ArithmeticStore {
    RigExecProviderOpKind kind;
    rigExec::RigExecFieldView<const int32_t> slots;
    const RigExecArray<Mat4d> *matrices = nullptr;
    const RigExecArray<uint8_t> *authoritative = nullptr;
    std::vector<ArithmeticFrame> frames;
    const RigExecArray<double> *scalars = nullptr;
    const RigExecArray<Vec3d> *vectors = nullptr;
    const RigExecArray<int32_t> *orders = nullptr;
    const RigExecArray<uint8_t> *resets = nullptr;
    Mat4d *matrix = nullptr;
    PointFrame *frame = nullptr;
    uint8_t *authority = nullptr;
    char order[3] = {0, 0, 0};
    bool failed = false;

    int32_t Slot(uint64_t id, ProviderInputClass c) const
    {
        return id < slots.Size() && ProviderArithmeticInputClass(kind, id) == c
                   ? slots.data[id]
                   : -1;
    }
    const Mat4d *ReadMatrix(uint64_t id) const
    {
        return Element(matrices, Slot(id, ProviderInputClass::Matrix));
    }
    const ArithmeticFrame *ReadFrame(uint64_t id) const
    {
        const int32_t slot = Slot(id, ProviderInputClass::Frame);
        return slot >= 0 && size_t(slot) < frames.size() ? &frames[size_t(slot)]
                                                         : nullptr;
    }
    std::optional<double> ReadScalar(uint64_t id) const
    {
        const double *value =
            Element(scalars, Slot(id, ProviderInputClass::Scalar));
        return value ? std::optional<double>(*value) : std::nullopt;
    }
    const Vec3d *ReadVector(uint64_t id) const
    {
        return Element(vectors, Slot(id, ProviderInputClass::Vector));
    }
    std::string_view ReadToken(uint64_t id)
    {
        const int32_t *packed =
            Element(orders, Slot(id, ProviderInputClass::Order));
        if (!packed || *packed == 0)
            return std::string_view();
        for (int i = 0; i < 3; ++i)
            order[i] = char((uint32_t(*packed) >> (8 * i)) & 0xffu);
        return std::string_view(order, 3);
    }
    bool ResetXformStack(uint64_t id) const
    {
        const uint8_t *reset =
            Element(resets, Slot(id, ProviderInputClass::Reset));
        return reset && *reset != 0;
    }
    bool Present(uint64_t /*id*/) const
    {
        return false;
    }
    bool Authoritative(uint64_t id) const
    {
        const uint8_t *flag =
            Element(authoritative, Slot(id, ProviderInputClass::Matrix));
        return flag && *flag != 0;
    }
    void Copy(uint64_t, uint64_t, bool)
    {
        failed = true;
    }
    void Invalidate(uint64_t, uint64_t)
    {
        failed = true;
    }
    void PublishMatrix(uint64_t, const Mat4d &value, bool isAuthoritative)
    {
        *matrix = value;
        *authority = isAuthoritative ? 1 : 0;
    }
    void PublishFrame(uint64_t, const ArithmeticFrame &value)
    {
        frame->points = value.points;
        frame->flags = value.flags;
    }
};

const rigExec::RigExecPortHandle &
ArithmeticPort(ProviderInputClass c)
{
    using C = ProviderInputClass;
    switch (c) {
    case C::Frame:
        return ProviderArithmeticFrames();
    case C::Scalar:
        return ProviderArithmeticScalars();
    case C::Vector:
        return ProviderArithmeticVectors();
    case C::Order:
        return ProviderArithmeticOrders();
    case C::Reset:
        return ProviderArithmeticResets();
    case C::Matrix:
    case C::None:
        break;
    }
    return ProviderArithmeticMatrices();
}

bool
CheckProviderArithmetic(const rigExec::RigExecOpCheck &check,
                        std::string *error)
{
    const auto kind =
        EnumAt<RigExecProviderOpKind>(check.batch, kKind, check.k);
    if (!ArithmeticKind(kind))
        return Fail(error, "ProviderArithmetic",
                    "names a kind that is not provider arithmetic");
    const auto slots = ListAt<int32_t>(check.batch, kSlots, check.k);
    for (size_t p = 0; p < slots.Size(); ++p) {
        const ProviderInputClass c = ProviderArithmeticInputClass(kind, p);
        const int32_t slot = slots.data[p];
        if (slot < 0) {
            if (kind == RigExecProviderOpKind::RelativeXform &&
                c == ProviderInputClass::Matrix)
                return Fail(error, "ProviderArithmetic",
                            "a relative transform needs every local");
            continue;
        }
        if (c == ProviderInputClass::None)
            return Fail(error, "ProviderArithmetic",
                        "input " + std::to_string(p) +
                            " is not one the kind reads");
        if (!SlotFits(check, ArithmeticPort(c), size_t(slot)))
            return Fail(error, "ProviderArithmetic",
                        "input " + std::to_string(p) +
                            " names an element its list does not hold");
    }
    const size_t flags =
        CheckElements(check, ProviderArithmeticAuthoritative());
    if (flags != 0 && flags != rigExec::kRigExecPortUnfed &&
        !SameCount(flags, CheckElements(check, ProviderArithmeticMatrices())))
        return Fail(error, "ProviderArithmetic",
                    "needs no authority flags or one per matrix");
    return true;
}

bool
ProviderArithmeticOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    ArithmeticStore store;
    store.kind = EnumAt<RigExecProviderOpKind>(batch, kKind, k);
    store.slots = ListAt<int32_t>(batch, kSlots, k);
    store.matrices = In<RigExecArray<Mat4d>>(io, ProviderArithmeticMatrices());
    store.authoritative =
        In<RigExecArray<uint8_t>>(io, ProviderArithmeticAuthoritative());
    if (const auto *frames =
            In<RigExecArray<PointFrame>>(io, ProviderArithmeticFrames()))
        for (const PointFrame &frame : *frames)
            store.frames.push_back(ArithmeticFrame{frame.points, frame.flags});
    store.scalars = In<RigExecArray<double>>(io, ProviderArithmeticScalars());
    store.vectors = In<RigExecArray<Vec3d>>(io, ProviderArithmeticVectors());
    store.orders = In<RigExecArray<int32_t>>(io, ProviderArithmeticOrders());
    store.resets = In<RigExecArray<uint8_t>>(io, ProviderArithmeticResets());
    store.matrix = &Out<Mat4d>(io, ProviderArithmeticMatrix());
    store.frame = &Out<PointFrame>(io, ProviderArithmeticFrame());
    store.authority = &Out<uint8_t>(io, ProviderArithmeticAuthoritativeOut());
    store.matrix->SetIdentity();
    *store.frame = PointFrame{};
    *store.authority = 0;
    const ArithmeticOp op{store.kind, 0, InputIds{store.slots},
                          FlagAt(batch, kScaleAvars, k)};
    if (!rigExec::RigExecRunProviderArithmetic<ArithmeticMath>(op, store) ||
        store.failed)
        return Fail(error, "ProviderArithmetic", "provider input unavailable");
    return true;
}

bool
RunProviderArithmetic(const RigExecOpBatch &batch, size_t *failed,
                      std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return ProviderArithmeticOne(batch, k, error);
    });
}

// ---- AffineFrame ----

enum AffineField : uint32_t {
    kAffineKind,
    kTargets,
    kAffineSlotCount,
    kAffineSlots,
    kTextCount,
    kTextLengths,
    kCharCount,
    kChars,
};

/// The affine sequencing's store (rigExecEvalCore/spaces.h) over the
/// node's typed lists: an absent or empty input keeps its default, as the
/// expression's layout defines.
struct AffineState {
    rigExec::RigExecFieldView<const int32_t> slots;
    rigExec::RigExecFieldView<const int32_t> textLengths;
    rigExec::RigExecFieldView<const uint32_t> chars;
    const RigExecArray<Mat4d> *matrices = nullptr;
    const RigExecArray<double> *doubles = nullptr;
    const RigExecArray<uint8_t> *bools = nullptr;
    const RigExecArray<int32_t> *ints = nullptr;
    const RigExecArray<Vec3d> *vectors = nullptr;
    const RigExecArray<Vec3i> *intVectors = nullptr;
    const RigExecArray<int32_t> *intItems = nullptr;
    const RigExecArray<double> *doubleItems = nullptr;
    const RigExecArray<Mat4d> *matrixItems = nullptr;
    const RigExecArray<int32_t> *spans = nullptr;
    const RigExecArray<PointFrame> *frames = nullptr;
    Mat4d *out = nullptr;
    std::string failure;

    int32_t Slot(uint64_t id) const
    {
        return id < slots.Size() ? slots.data[id] : -1;
    }
    const Mat4d *ReadMatrix(uint64_t id) const
    {
        return Element(matrices, Slot(id));
    }
    const Vec3d *ReadVector(uint64_t id) const
    {
        return Element(vectors, Slot(id));
    }
    const PointFrame *ReadFrame(uint64_t id) const
    {
        return Element(frames, Slot(id));
    }
    bool ReadAffineVec3i(uint64_t id, Vec3i *value) const
    {
        const Vec3i *in = Element(intVectors, Slot(id));
        if (in)
            *value = *in;
        return in != nullptr;
    }
    bool ReadAffineDouble(uint64_t id, double *value) const
    {
        const double *in = Element(doubles, Slot(id));
        if (in)
            *value = *in;
        return in != nullptr;
    }
    bool ReadAffineBool(uint64_t id, bool *value) const
    {
        const uint8_t *in = Element(bools, Slot(id));
        if (in)
            *value = *in != 0;
        return in != nullptr;
    }
    bool ReadAffineInt(uint64_t id, int *value) const
    {
        const int32_t *in = Element(ints, Slot(id));
        if (in)
            *value = *in;
        return in != nullptr;
    }
    bool ReadAffineString(uint64_t id, std::string *value) const
    {
        const int32_t slot = Slot(id);
        if (slot < 0 || size_t(slot) >= textLengths.Size())
            return false;
        size_t begin = 0;
        for (int32_t i = 0; i < slot; ++i)
            begin += size_t(textLengths.data[i]);
        const size_t length = size_t(textLengths.data[slot]);
        if (begin + length > chars.Size())
            return false;
        value->resize(length);
        for (size_t i = 0; i < length; ++i)
            (*value)[i] = char(chars.data[begin + i]);
        return true;
    }
    template <class T, class V>
    bool ReadItems(const RigExecArray<T> *items, uint64_t id,
                   std::vector<V> *value) const
    {
        const int32_t slot = Slot(id);
        const int32_t *begin = Element(spans, slot < 0 ? -1 : 2 * slot);
        const int32_t *count = Element(spans, slot < 0 ? -1 : 2 * slot + 1);
        if (!items || !begin || !count || *begin < 0 || *count <= 0 ||
            size_t(*begin) + size_t(*count) > items->Size())
            return false;
        value->assign(items->Data() + *begin, items->Data() + *begin + *count);
        return true;
    }
    bool ReadAffineIntArray(uint64_t id, std::vector<int> *value) const
    {
        return ReadItems(intItems, id, value);
    }
    bool ReadAffineDoubleArray(uint64_t id, std::vector<double> *value) const
    {
        return ReadItems(doubleItems, id, value);
    }
    bool ReadAffineMatrixArray(uint64_t id, std::vector<Mat4d> *value) const
    {
        return ReadItems(matrixItems, id, value);
    }
    void PublishAffineFailure(uint64_t, const std::string &why)
    {
        failure = why;
    }
    void PublishMatrix(uint64_t, const Mat4d &value, bool)
    {
        *out = value;
    }
};

struct AffineHost {
    struct Math {
        using AffineMath = rigExec::RrAffineMath;
    };
    AffineState &affine;
    AffineState &state()
    {
        return affine;
    }
};

struct AffineOp {
    uint32_t affineKind = 0;
    uint32_t affineTargets = 0;
    InputIds inputs;
    uint64_t output = 0;
};

const rigExec::RigExecPortHandle &
AffinePort(AffineInputClass c)
{
    using C = AffineInputClass;
    switch (c) {
    case C::Double:
        return AffineFrameDoubles();
    case C::Bool:
        return AffineFrameBools();
    case C::Int:
        return AffineFrameInts();
    case C::Vector:
        return AffineFrameVectors();
    case C::IntVector:
        return AffineFrameIntVectors();
    case C::IntItems:
        return AffineFrameIntItems();
    case C::DoubleItems:
        return AffineFrameDoubleItems();
    case C::MatrixItems:
        return AffineFrameMatrixItems();
    case C::Frame:
        return AffineFrameFrames();
    case C::Matrix:
    case C::Text:
        break;
    }
    return AffineFrameMatrices();
}

bool
CheckAffineFrame(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const uint32_t kind = ValueAt<uint32_t>(check.batch, kAffineKind, check.k);
    const uint32_t targets = ValueAt<uint32_t>(check.batch, kTargets, check.k);
    const auto slots = ListAt<int32_t>(check.batch, kAffineSlots, check.k);
    const auto lengths = ListAt<int32_t>(check.batch, kTextLengths, check.k);
    const auto chars = ListAt<uint32_t>(check.batch, kChars, check.k);
    if (!rigExec::RigExecAffineFrameInputsMatch(kind, slots.Size(), targets))
        return Fail(error, "AffineFrame",
                    "its inputs do not match the expression's layout");
    size_t total = 0;
    for (size_t i = 0; i < lengths.Size(); ++i) {
        if (lengths.data[i] < 0)
            return Fail(error, "AffineFrame", "a text has a negative length");
        total += size_t(lengths.data[i]);
    }
    if (total != chars.Size())
        return Fail(error, "AffineFrame",
                    "its texts do not match their characters");
    for (size_t p = 0; p < slots.Size(); ++p) {
        const int32_t slot = slots.data[p];
        if (slot < 0)
            continue;
        const AffineInputClass c = AffineFrameInputClass(kind, p);
        bool fits = false;
        if (c == AffineInputClass::Text)
            fits = size_t(slot) < lengths.Size();
        else if (c == AffineInputClass::IntItems ||
                 c == AffineInputClass::DoubleItems ||
                 c == AffineInputClass::MatrixItems)
            fits =
                Fed(check, AffinePort(c)) &&
                SlotFits(check, AffineFrameItemSpans(), 2 * size_t(slot) + 1);
        else
            fits = SlotFits(check, AffinePort(c), size_t(slot));
        if (!fits)
            return Fail(error, "AffineFrame",
                        "input " + std::to_string(p) +
                            " names an element its list does not hold");
    }
    return true;
}

bool
AffineFrameOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    AffineState state;
    state.slots = ListAt<int32_t>(batch, kAffineSlots, k);
    state.textLengths = ListAt<int32_t>(batch, kTextLengths, k);
    state.chars = ListAt<uint32_t>(batch, kChars, k);
    state.matrices = In<RigExecArray<Mat4d>>(io, AffineFrameMatrices());
    state.doubles = In<RigExecArray<double>>(io, AffineFrameDoubles());
    state.bools = In<RigExecArray<uint8_t>>(io, AffineFrameBools());
    state.ints = In<RigExecArray<int32_t>>(io, AffineFrameInts());
    state.vectors = In<RigExecArray<Vec3d>>(io, AffineFrameVectors());
    state.intVectors = In<RigExecArray<Vec3i>>(io, AffineFrameIntVectors());
    state.intItems = In<RigExecArray<int32_t>>(io, AffineFrameIntItems());
    state.doubleItems = In<RigExecArray<double>>(io, AffineFrameDoubleItems());
    state.matrixItems = In<RigExecArray<Mat4d>>(io, AffineFrameMatrixItems());
    state.spans = In<RigExecArray<int32_t>>(io, AffineFrameItemSpans());
    state.frames = In<RigExecArray<PointFrame>>(io, AffineFrameFrames());
    state.out = &Out<Mat4d>(io, AffineFrameOut());
    state.out->SetIdentity();
    const AffineOp op{ValueAt<uint32_t>(batch, kAffineKind, k),
                      ValueAt<uint32_t>(batch, kTargets, k),
                      InputIds{state.slots}, 0};
    AffineHost host{state};
    const auto result = rigExec::eval::spaces::RunSpaceAffineOp(host, op);
    if (result.outcome != rigExec::eval::Outcome::Applied)
        return Fail(error, "AffineFrame",
                    "its inputs do not match the expression's layout");
    if (!state.failure.empty())
        return Fail(error, "AffineFrame", state.failure);
    return true;
}

bool
RunAffineFrame(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return AffineFrameOne(batch, k, error);
    });
}

} // namespace

namespace {

using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;

const rigExec::RigExecPortDesc kProviderArithmeticPorts[] = {
    {&ProviderArithmeticMatrices(), &rigExecStd::Mat4dArray(), kOptional,
     "matrices"},
    {&ProviderArithmeticAuthoritative(), &rigExecStd::BoolArray(), kOptional,
     "authoritative"},
    {&ProviderArithmeticFrames(), &rigExecRig::PointFrameList(), kOptional,
     "frames"},
    {&ProviderArithmeticScalars(), &rigExecStd::DoubleArray(), kOptional,
     "scalars"},
    {&ProviderArithmeticVectors(), &rigExecStd::Vec3dArray(), kOptional,
     "vectors"},
    {&ProviderArithmeticOrders(), &rigExecStd::IntArray(), kOptional, "orders"},
    {&ProviderArithmeticResets(), &rigExecStd::BoolArray(), kOptional,
     "resets"},
    {&ProviderArithmeticMatrix(), &rigExecStd::Mat4dType(), kRigExecPortOutput,
     "matrix"},
    {&ProviderArithmeticFrame(), &rigExecRig::PointFrameType(),
     kRigExecPortOutput, "frame"},
    {&ProviderArithmeticAuthoritativeOut(), &rigExecStd::BoolType(),
     kRigExecPortOutput, "authoritativeOut"},
};

void
ProviderArithmeticValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const ProviderArithmeticPayload *>(payload);
    SetEnum(out, kKind, p.kind);
    SetFlag(out, kScaleAvars, p.scaleAvars);
    SetList(out, kSlotCount, p.slots);
}

const rigExec::RigExecFieldDesc kProviderArithmeticFields[] = {
    Enum("kind", RigExecProviderOpKind::AffineFrame),
    Flag("scaleAvars"),
    Count("slotCount"),
    Items("slots", rigExecStd::IntType(), kSlotCount),
};

const rigExec::RigExecOpDesc kProviderArithmeticDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "ProviderArithmetic";
    d.ports = kProviderArithmeticPorts;
    d.portCount = std::size(kProviderArithmeticPorts);
    d.runBatch = &RunProviderArithmetic;
    d.validate = &CheckProviderArithmetic;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kProviderArithmeticFields;
    d.payloadFieldCount = std::size(kProviderArithmeticFields);
    d.payloadVersion = 1;
    d.payloadValues = &ProviderArithmeticValues;
    return d;
}();

const rigExec::RigExecPortDesc kAffineFramePorts[] = {
    {&AffineFrameMatrices(), &rigExecStd::Mat4dArray(), kOptional, "matrices"},
    {&AffineFrameDoubles(), &rigExecStd::DoubleArray(), kOptional, "doubles"},
    {&AffineFrameBools(), &rigExecStd::BoolArray(), kOptional, "bools"},
    {&AffineFrameInts(), &rigExecStd::IntArray(), kOptional, "ints"},
    {&AffineFrameVectors(), &rigExecStd::Vec3dArray(), kOptional, "vectors"},
    {&AffineFrameIntVectors(), &rigExecStd::Vec3iArray(), kOptional,
     "intVectors"},
    {&AffineFrameIntItems(), &rigExecStd::IntArray(), kOptional, "intItems"},
    {&AffineFrameDoubleItems(), &rigExecStd::DoubleArray(), kOptional,
     "doubleItems"},
    {&AffineFrameMatrixItems(), &rigExecStd::Mat4dArray(), kOptional,
     "matrixItems"},
    {&AffineFrameItemSpans(), &rigExecStd::IntArray(), kOptional, "itemSpans"},
    {&AffineFrameFrames(), &rigExecRig::PointFrameList(), kOptional, "frames"},
    {&AffineFrameOut(), &rigExecStd::Mat4dType(), kRigExecPortOutput, "out"},
};

void
AffineFrameValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const AffineFramePayload *>(payload);
    out.Set(kAffineKind, p.kind);
    out.Set(kTargets, p.targets);
    SetList(out, kAffineSlotCount, p.slots);
    std::vector<int32_t> lengths;
    std::vector<uint32_t> chars;
    for (const std::string &text : p.texts) {
        lengths.push_back(int32_t(text.size()));
        for (const char c : text)
            chars.push_back(uint32_t(uint8_t(c)));
    }
    SetList(out, kTextCount, lengths);
    SetList(out, kCharCount, chars);
}

const rigExec::RigExecFieldDesc kAffineFrameFields[] = {
    Value("kind", rigExecStd::UIntType()),
    Value("targets", rigExecStd::UIntType()),
    Count("slotCount"),
    Items("slots", rigExecStd::IntType(), kAffineSlotCount),
    Count("textCount"),
    Items("textLengths", rigExecStd::IntType(), kTextCount),
    Count("charCount"),
    Items("chars", rigExecStd::UIntType(), kCharCount),
};

const rigExec::RigExecOpDesc kAffineFrameDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "AffineFrame";
    d.ports = kAffineFramePorts;
    d.portCount = std::size(kAffineFramePorts);
    d.runBatch = &RunAffineFrame;
    d.validate = &CheckAffineFrame;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kAffineFrameFields;
    d.payloadFieldCount = std::size(kAffineFrameFields);
    d.payloadVersion = 1;
    d.payloadValues = &AffineFrameValues;
    return d;
}();

} // namespace

void
RegisterProviderOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(ProviderArithmetic(), kProviderArithmeticDesc);
    registrar.Add(AffineFrame(), kAffineFrameDesc);
}

} // namespace rigExecOps
