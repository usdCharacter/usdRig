#ifndef RIGEXEC_API_REGISTRY_H
#define RIGEXEC_API_REGISTRY_H

/// The list of data types and operations ("ops") a process can use. Type
/// and op libraries describe theirs in tables; RigExecRegistry::Initialize
/// collects them once at startup. From then on code refers to each type, op
/// and op port through a handle object rather than by name; names appear
/// only in messages and saved files.

#include "rigExec/api/payloadCodec.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace rigExec {

class RigExecRegistry;
class RigExecRegistryBuilder;
/// Defined in rigExecGraph/windowHistory.h.
struct RigExecWindowRun;

inline constexpr uint32_t kRigExecInvalidIndex = 0xffffffffu;

/// Version of the descriptor structs below. Each library records the version
/// it was built with, and Initialize rejects one built with another. Raise
/// it whenever a descriptor's layout changes.
inline constexpr uint32_t kRigExecRegistryAbiVersion = 14;

/// Refers to one registered data type. A type library defines one per type
/// with RIGEXEC_DEFINE_TYPE_HANDLE, and Initialize gives it its index; read
/// it only after Initialize.
class RigExecTypeHandle {
public:
    constexpr RigExecTypeHandle() = default;
    RigExecTypeHandle(const RigExecTypeHandle &) = delete;
    RigExecTypeHandle &operator=(const RigExecTypeHandle &) = delete;
    uint32_t Index() const
    {
        return _index;
    }
    bool IsValid() const
    {
        return _index != kRigExecInvalidIndex;
    }

private:
    friend class RigExecRegistryBuilder;
    mutable uint32_t _index = kRigExecInvalidIndex;
};

/// Refers to one registered op. Defined with RIGEXEC_DEFINE_OP_HANDLE and
/// valid after Initialize.
class RigExecOpHandle {
public:
    constexpr RigExecOpHandle() = default;
    RigExecOpHandle(const RigExecOpHandle &) = delete;
    RigExecOpHandle &operator=(const RigExecOpHandle &) = delete;
    uint32_t Index() const
    {
        return _index;
    }
    bool IsValid() const
    {
        return _index != kRigExecInvalidIndex;
    }

private:
    friend class RigExecRegistryBuilder;
    mutable uint32_t _index = kRigExecInvalidIndex;
};

/// Refers to one input or output of an op, by its position in the op's port
/// list. Defined with RIGEXEC_DEFINE_PORT_HANDLE and valid after Initialize.
class RigExecPortHandle {
public:
    constexpr RigExecPortHandle() = default;
    RigExecPortHandle(const RigExecPortHandle &) = delete;
    RigExecPortHandle &operator=(const RigExecPortHandle &) = delete;
    uint32_t Index() const
    {
        return _index;
    }
    bool IsValid() const
    {
        return _index != kRigExecInvalidIndex;
    }

private:
    friend class RigExecRegistryBuilder;
    mutable uint32_t _index = kRigExecInvalidIndex;
};

/// How a type's equal function treats floating-point members: Bitwise
/// compares bytes, NaNEqual compares numbers but counts two NaNs as equal,
/// and Ieee compares numbers, so a NaN equals nothing.
enum class RigExecEqualMode : uint8_t { Bitwise, NaNEqual, Ieee };
/// Whether values of a type can be hashed; any class but Unhashable needs a
/// hash function.
enum class RigExecHashClass : uint8_t { Exact, Generic, Unhashable };

/// The scalar one column holds. A value is stored as columns: one per
/// scalar it is made of, so a Vec3f is three Float32 columns.
enum class RigExecColumnKind : uint8_t {
    None,
    UInt8,
    Int32,
    UInt32,
    UInt64,
    Float32,
    Float64,
};

/// Bytes per scalar of \p kind; 0 for None.
constexpr size_t
RigExecColumnKindSize(RigExecColumnKind kind)
{
    switch (kind) {
    case RigExecColumnKind::UInt8:
        return 1;
    case RigExecColumnKind::Int32:
    case RigExecColumnKind::UInt32:
    case RigExecColumnKind::Float32:
        return 4;
    case RigExecColumnKind::UInt64:
    case RigExecColumnKind::Float64:
        return 8;
    case RigExecColumnKind::None:
        break;
    }
    return 0;
}

/// Describes one data type: its size and alignment, how to construct, copy,
/// compare and hash a value, and, for an array or a record, the types it is
/// made of. A type that is not trivially copyable needs copy and destroy.
struct RigExecTypeDesc {
    uint32_t structSize = sizeof(RigExecTypeDesc);
    /// With major, the type's name in saved files and messages.
    const char *nameSpace = "";
    const char *name = "";
    uint32_t major = 1;
    uint32_t minor = 0;
    uint64_t layoutHash = 0;
    size_t size = 0;
    size_t align = 1;
    bool triviallyCopyable = false;
    /// The element type, when this type is an array of them.
    const RigExecTypeHandle *element = nullptr;
    /// The member types, in order, when this type is a record.
    const RigExecTypeHandle *const *fields = nullptr;
    size_t fieldCount = 0;
    void (*construct)(void *dst) = nullptr;
    void (*copy)(void *dst, const void *src) = nullptr;
    void (*move)(void *dst, void *src) = nullptr;
    void (*destroy)(void *dst) = nullptr;
    bool (*equal)(const void *a, const void *b, RigExecEqualMode) = nullptr;
    RigExecHashClass hashClass = RigExecHashClass::Unhashable;
    uint64_t (*hash)(const void *value) = nullptr;
    /// For an array type: how many elements \p value holds.
    size_t (*count)(const void *value) = nullptr;
    /// For a type without element or member types: the scalar its values
    /// are made of and how many each holds, one after another in memory.
    /// None leaves the type, and every type built from it, without columns.
    /// The registry derives the columns of arrays and records from these.
    RigExecColumnKind columnKind = RigExecColumnKind::None;
    uint32_t columnCount = 0;
    /// For an array, or a record or fixed list holding arrays: makes room
    /// for \p count elements in each of its arrays, keeping what they
    /// hold, so filling them later allocates nothing. An output of such a
    /// type declares its bound (RigExecOutputBound).
    void (*reserve)(void *value, size_t count) = nullptr;
    /// With reserve: how many elements the longest of its arrays holds.
    size_t (*elements)(const void *value) = nullptr;
    /// For an array: makes \p value hold \p count elements, keeping the
    /// first ones it holds, in storage reserve made room for.
    void (*resize)(void *value, size_t count) = nullptr;
    /// For an array: where its first element is.
    void *(*data)(void *value) = nullptr;
    /// For an array: makes \p value read the \p count elements at \p data,
    /// which stay the caller's and unchanged while it is read; resizing or
    /// assigning it copies them into storage of its own first. Null \p data
    /// gives a value only whose count is read: its elements are elsewhere
    /// (RigExecOpRange::at).
    void (*borrow)(void *value, const void *data, size_t count) = nullptr;
};

/// One column of a registered type. `run` says how many values the column
/// holds for one value of the type: run 0 holds one, and run r > 0 holds
/// one per element of the r-th array met in member order, depth first.
struct RigExecTypeColumn {
    RigExecColumnKind kind = RigExecColumnKind::None;
    uint32_t run = 0;
};

/// Whether a port is an input or an output, and how it may be fed.
enum RigExecPortFlags : uint32_t {
    kRigExecPortInput = 1u << 0,
    kRigExecPortOutput = 1u << 1,
    kRigExecPortOptional = 1u << 2,
    /// An input that takes any number of values, in the order they were
    /// connected or bound. The port's type is the type of one value; the op
    /// reads them as a RigExecPortList.
    kRigExecPortArray = 1u << 3,
    /// On an op that runs in ranges (RigExecOpRangeHandling): an input
    /// whose element i a range reads for element i of the outputs, or an
    /// output, each of whose elements one range writes. The node's element
    /// count is what its first pointwise input holds.
    kRigExecPortPointwise = 1u << 4,
    /// On an op that runs in ranges: an input a range reads around its
    /// elements, through topology the node is given; each range keeps its
    /// halo plan in its scratch.
    kRigExecPortNeighbourhood = 1u << 5,
    /// An input whose value the op's validate reads, not only its size:
    /// a change of the host's value validates the node again before it
    /// runs (RigExecOpDesc::validate).
    kRigExecPortChecked = 1u << 6,
};

/// The most elements an output whose type holds arrays may hold. The
/// evaluator makes room for that many before a frame runs, so the op fills
/// storage that is already there, and a node that writes more fails.
enum class RigExecOutputBound : uint8_t {
    /// For an output whose type holds no arrays.
    None,
    /// boundCount elements.
    Fixed,
    /// boundCount elements for each element the value fed to input
    /// boundPort holds, or that input boundAltPort holds when it is set
    /// and holds more. For an input that takes many values, its longest
    /// value counts.
    Input,
    /// boundCount elements for each unit of the integer fed to input
    /// boundPort, a one-value integer input, or for each element input
    /// boundAltPort holds when it is set and holds more.
    InputValue,
    /// boundCount elements for each unit of the node's payload field
    /// boundField, a one-value integer field.
    Field,
    /// As many as the rig description gives for this output of the node
    /// (RigExecRigDescBuilder::SetOutputBound).
    Host,
};

/// How an authored property feeds a port or payload field.
enum class RigExecPropertyBinding : uint8_t {
    /// An attribute: its value, or the value its connection brings.
    Value,
    /// A relationship to one target, whose data the scene translation
    /// reads.
    Connection,
    /// A relationship whose targets, in order, feed a list input one value
    /// each.
    ConnectionList,
    /// The members of a collection of the prim.
    Collection,
    /// Child prims of the prim.
    Children,
};

/// The USD role of an authored float3 or double3 value.
enum class RigExecPropertyRole : uint8_t { None, Point, Normal, Vector, Color };

/// The schema property a port or payload field is authored with, for the
/// schema generator. It never changes how a program runs, and the registry
/// fingerprint does not include it.
struct RigExecPropertyInfo {
    /// With its namespace: rigExec:, inputs:, avars:, rest: or guide:.
    /// Payload fields that name one property are its components, in field
    /// order.
    const char *schemaName = "";
    const char *doc = nullptr;
    /// A value of the port's or field's registered type, or for a list
    /// payload field of the registered array type of its element; with
    /// tokens, the value whose token is the fallback. Null authors none.
    const void *fallback = nullptr;
    /// For an integer or bool authored as a token: entry i is the token of
    /// value i, null for a value the property does not allow.
    const char *const *tokens = nullptr;
    size_t tokenCount = 0;
    RigExecPropertyBinding binding = RigExecPropertyBinding::Value;
    RigExecPropertyRole role = RigExecPropertyRole::None;
    bool uniform = false;
    /// The USD type name the property is authored with when it is not the
    /// registered type's, such as "float" for a double port. Null uses the
    /// registered type's.
    const char *authoredType = nullptr;
};

/// One input or output of an op.
struct RigExecPortDesc {
    const RigExecPortHandle *port = nullptr;
    const RigExecTypeHandle *type = nullptr;
    uint32_t flags = 0;
    /// The port's name in generated schema and in messages.
    const char *name = "";
    /// Outputs only: see RigExecOutputBound.
    RigExecOutputBound bound = RigExecOutputBound::None;
    const RigExecPortHandle *boundPort = nullptr;
    const RigExecPortHandle *boundAltPort = nullptr;
    uint32_t boundField = kRigExecInvalidIndex;
    size_t boundCount = 1;
    /// The property the port is authored with; null for a port the compiler
    /// feeds.
    const RigExecPropertyInfo *authoring = nullptr;
};

/// Where an op may run. Pure ops depend only on their inputs and may run on
/// any worker thread. Every other op runs on the thread that calls
/// Evaluate: OwnerThread ops need that thread, Device ops drive a GPU or
/// other device, and Windowed ops also read what their inputs were on
/// earlier frames (see the history calls on RigExecEvaluator).
enum class RigExecOpThreading : uint8_t { Pure, OwnerThread, Device, Windowed };

/// What the evaluators may do with an op. A point op's bits mirror the
/// lists its RigExecRevisionOp is still classified by; a test keeps them
/// equal.
enum RigExecOpCapabilities : uint32_t {
    kRigExecOpBakeable = 1u << 0,
    kRigExecOpFreezable = 1u << 1,
    kRigExecOpGpu = 1u << 2,
    /// When the op's per-point weights list only some points, the points
    /// left out pass through unchanged.
    kRigExecOpGate = 1u << 3,
    /// The op computes its full effect; the per-point weights then blend
    /// that result with the incoming points, instead of the op applying
    /// the weights itself.
    kRigExecOpSeparateBlend = 1u << 4,
    /// The op runs in ranges: it has RigExecOpRangeHandling.
    kRigExecOpRanged = 1u << 5,
    /// The scene translation builds the op's inputs from values it reads
    /// directly from the stage.
    kRigExecOpFromLeaves = 1u << 6,
};

/// How to copy, compare, destroy and save an op's settings ("payload"):
/// values fixed when the rig is compiled, such as a solver's iteration
/// count. The payload is private to the op and is not a registered data
/// type. A Bakeable op with a payload must provide encode and decode, so a
/// saved rig can carry its settings.
struct RigExecOpPayloadHandling {
    size_t size = 0;
    size_t align = 1;
    void (*construct)(void *dst) = nullptr;
    void (*copy)(void *dst, const void *src) = nullptr;
    void (*destroy)(void *dst) = nullptr;
    uint64_t (*hash)(const void *payload) = nullptr;
    bool (*equal)(const void *a, const void *b) = nullptr;
    /// Replaces \p bytes with the payload's saved form.
    void (*encode)(const void *payload, std::vector<uint8_t> &bytes) = nullptr;
    /// Reads \p size bytes into \p payload, which construct has made.
    /// Returns false, with the reason in \p error, for bytes encode cannot
    /// have written; it never reads outside them.
    bool (*decode)(const uint8_t *bytes, size_t size, void *payload,
                   std::string *error) = nullptr;
};

template <class T, class = void>
struct RigExecPayloadComparable : std::false_type {};
template <class T>
struct RigExecPayloadComparable<
    T, std::void_t<decltype(std::declval<const T &>() ==
                            std::declval<const T &>())>> : std::true_type {};

/// Payload handling for a copyable settings struct \p T. Payloads compare
/// equal with \p T's operator== when it has one; they are not hashed.
template <class T>
RigExecOpPayloadHandling
RigExecPayloadHandlingFor()
{
    RigExecOpPayloadHandling h;
    h.size = sizeof(T);
    h.align = alignof(T);
    h.construct = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<T *>(dst), 1);
    };
    h.copy = [](void *dst, const void *src) {
        std::uninitialized_copy_n(static_cast<const T *>(src), 1,
                                  static_cast<T *>(dst));
    };
    h.destroy = [](void *dst) { std::destroy_at(static_cast<T *>(dst)); };
    if constexpr (RigExecPayloadComparable<T>::value)
        h.equal = [](const void *a, const void *b) {
            return *static_cast<const T *>(a) == *static_cast<const T *>(b);
        };
    return h;
}

/// Payload handling for \p T that also saves and reads it with
/// \p Encode(writer, payload) and \p Decode(reader, payload).
template <class T, void (*Encode)(RigExecPayloadWriter &, const T &),
          void (*Decode)(RigExecPayloadReader &, T &)>
RigExecOpPayloadHandling
RigExecPayloadHandlingFor()
{
    RigExecOpPayloadHandling h = RigExecPayloadHandlingFor<T>();
    h.encode = [](const void *payload, std::vector<uint8_t> &bytes) {
        bytes.clear();
        RigExecPayloadWriter writer(bytes);
        Encode(writer, *static_cast<const T *>(payload));
    };
    h.decode = [](const uint8_t *bytes, size_t size, void *payload,
                  std::string *error) {
        RigExecPayloadReader reader(bytes, size);
        Decode(reader, *static_cast<T *>(payload));
        return reader.Finish(error);
    };
    return h;
}

/// How to create, copy, reset and destroy an op's private working state,
/// such as caches it keeps from one evaluation to the next. Each node that
/// runs the op in each evaluator has its own state.
struct RigExecOpStateHandling {
    size_t size = 0;
    size_t align = 1;
    void (*create)(void *dst) = nullptr;
    void (*copy)(void *dst, const void *src) = nullptr;
    void (*reset)(void *dst) = nullptr;
    void (*destroy)(void *dst) = nullptr;
};

/// The values fed to an input that takes many values, in the order they
/// were connected or bound.
struct RigExecPortList {
    const void *const *items = nullptr;
    size_t count = 0;
};

/// What an op's run function receives: a pointer to each input value and
/// to each output to write, indexed by port handle. An unfed optional input
/// is null. An input that takes many values points to a RigExecPortList,
/// empty when nothing feeds it.
struct RigExecOpIo {
    const void *const *inputs = nullptr;
    void *const *outputs = nullptr;
    size_t portCount = 0;
    /// For a Windowed op: what it recorded on earlier frames, and where to
    /// record this frame's. Null for other ops.
    const RigExecWindowRun *window = nullptr;
    /// The program's RigExecCompileOptions::useSimd: whether an op with an
    /// SSE2 form and a scalar form uses the SSE2 form.
    bool useSimd = true;
};

/// Computes an op's outputs from its inputs. Returns false, optionally with
/// a message in \p error, when it cannot.
using RigExecOpRunFn = bool (*)(const void *payload, void *state,
                                const RigExecOpIo &io, std::string *error);

struct RigExecOpDesc;
class RigExecFieldValues;

/// How many values a declared field holds for one node.
enum class RigExecFieldExtent : uint8_t {
    /// One value.
    Scalar,
    /// RigExecFieldDesc::count values.
    Fixed,
    /// As many values as the one-value payload field `source` says; that
    /// field holds an unsigned integer.
    FromField,
    /// As many values as input `port`, an array, holds elements when the
    /// node runs.
    FromInput,
};

/// One declared value, or run of values, of an op's settings (payload) or
/// working state. Kernels refer to a field by its position in the op's
/// list.
struct RigExecFieldDesc {
    /// For messages and saved-file listings only.
    const char *name = "";
    /// A registered, trivially copyable value type without elements or
    /// members, aligned to at most 64 bytes. Saved files hold its bytes as
    /// they are in memory.
    const RigExecTypeHandle *type = nullptr;
    RigExecFieldExtent extent = RigExecFieldExtent::Scalar;
    /// Fixed only.
    uint32_t count = 0;
    /// FromField only: the payload field's position.
    uint32_t source = kRigExecInvalidIndex;
    /// FromInput only.
    const RigExecPortHandle *port = nullptr;
    /// For a payload field of an unsigned integer type: the largest value
    /// it may hold, such as an enum's last value or 1 for a flag.
    uint64_t limit = UINT64_MAX;
    /// Payload fields only: the property the field is authored with; null
    /// for a field the compiler derives.
    const RigExecPropertyInfo *authoring = nullptr;
};

/// A field whose count varies: one node's values and their count.
template <class Byte> struct RigExecFieldRunOf {
    Byte *data = nullptr;
    size_t count = 0;
};

/// One declared field across a batch. A Scalar or Fixed field is a column:
/// instance k's `count` values start at base + k * stride. Any other
/// field has one run per instance in `runs`.
template <class Byte> struct RigExecFieldColumnOf {
    Byte *base = nullptr;
    size_t stride = 0;
    size_t count = 0;
    const RigExecFieldRunOf<Byte> *runs = nullptr;
};

using RigExecPayloadColumn = RigExecFieldColumnOf<const std::byte>;
using RigExecStateColumn = RigExecFieldColumnOf<std::byte>;

/// What an op's batch function receives: several nodes of the op that run
/// together, called instances, numbered 0 to count - 1 in the order the
/// program runs them. Instance k's payload is at payloads + k *
/// payloadStride and its state at states + k * stateStride (null when the
/// op has none). Its port values are the portCount entries of inputs and
/// outputs starting at k * portCount, laid out as in RigExecOpIo.
struct RigExecOpBatch {
    const RigExecOpDesc *op = nullptr;
    size_t count = 0;
    /// Per instance: zero for one this run skips, as a history tick skips
    /// the nodes no windowed op depends on. Null runs every instance.
    const char *active = nullptr;
    const std::byte *payloads = nullptr;
    size_t payloadStride = 0;
    std::byte *states = nullptr;
    size_t stateStride = 0;
    size_t portCount = 0;
    const void *const *inputs = nullptr;
    void *const *outputs = nullptr;
    /// For a Windowed op, each instance's history record; null otherwise.
    const RigExecWindowRun *const *windows = nullptr;
    /// One per declared payload field and per declared state field, in
    /// the op's order; null when it declares none (see opFields.h).
    const RigExecPayloadColumn *payloadFields = nullptr;
    const RigExecStateColumn *stateFields = nullptr;
    /// The program's RigExecCompileOptions::useSimd, as in RigExecOpIo.
    bool useSimd = true;
};

/// Runs every active instance of \p batch, in instance order. On a failure
/// it stops, sets \p failed to that instance and may leave a message in
/// \p error, and returns false; the instances before it keep their results.
using RigExecOpBatchFn = bool (*)(const RigExecOpBatch &batch, size_t *failed,
                                  std::string *error);

inline bool
RigExecBatchActive(const RigExecOpBatch &batch, size_t k)
{
    return !batch.active || batch.active[k];
}

inline const void *
RigExecBatchPayload(const RigExecOpBatch &batch, size_t k)
{
    return batch.payloads ? batch.payloads + k * batch.payloadStride : nullptr;
}

inline void *
RigExecBatchState(const RigExecOpBatch &batch, size_t k)
{
    return batch.states ? batch.states + k * batch.stateStride : nullptr;
}

/// Instance \p k's ports, as its run function would receive them.
inline RigExecOpIo
RigExecBatchInstance(const RigExecOpBatch &batch, size_t k)
{
    RigExecOpIo io;
    io.inputs = batch.inputs + k * batch.portCount;
    io.outputs = batch.outputs + k * batch.portCount;
    io.portCount = batch.portCount;
    io.window = batch.windows ? batch.windows[k] : nullptr;
    io.useSimd = batch.useSimd;
    return io;
}

/// The batch function of an op that provides only run: calls run once per
/// active instance, in order.
bool RigExecRunEachInstance(const RigExecOpBatch &batch, size_t *failed,
                            std::string *error);

/// The elements of a neighbourhood input, cut into groups: group h holds
/// elements [bounds[h], bounds[h + 1]) at points[h], which is null for a
/// group the range does not read. bounds holds groups + 1 entries.
struct RigExecOpGroups {
    const int *bounds = nullptr;
    size_t groups = 0;
    const void *const *points = nullptr;
};

/// Elements [begin, end) of a ranged node's pointwise ports: range
/// \p index of the \p ranges its \p count elements are cut into.
struct RigExecOpRange {
    size_t begin = 0;
    size_t end = 0;
    size_t count = 0;
    uint32_t index = 0;
    uint32_t ranges = 1;
    /// This range's own scratch (RigExecOpRangeHandling::scratch), kept
    /// from frame to frame; null when the op has none.
    void *scratch = nullptr;
    /// By port index: for a pointwise port, where element begin is, with
    /// elements (begin, end) after it; for a neighbourhood input, its
    /// RigExecOpGroups. A range reads and writes these ports only here:
    /// their values may hold no elements of their own.
    void *const *at = nullptr;
};

/// How a ranged node runs for the life of its program, from its settings.
enum class RigExecRangePlan : uint8_t {
    /// The node runs as its partition's ranges.
    Ranges,
    /// The node runs as one range over every element, whatever the
    /// partition.
    Whole,
};

/// How a Pure op runs one node over ranges of its elements, the ranges in
/// parallel. The evaluator cuts a node's elements into ranges of the
/// program's RigExecCompileOptions::rangeGrain; the partition depends only
/// on the element count and the options. For each active instance k it
/// sizes the pointwise outputs to the element count, then calls prepare,
/// then run once per range. Results must be bitwise the same
/// for every partition. A range reads the inputs, the payload and the
/// node's state, and writes only its elements of the pointwise outputs and
/// its scratch; ranges of one node run concurrently. Ranged nodes that
/// feed one another's pointwise or neighbourhood inputs form a range
/// chain over one partition: prepare runs for every node of the chain
/// first, then each range runs through the chain's nodes in order as one
/// task, a neighbourhood input waiting only for the ranges it reads. So
/// prepare reads the sizes of pointwise and neighbourhood inputs, never
/// their elements.
struct RigExecOpRangeHandling {
    /// Whether instance \p k runs in ranges or whole, from its settings
    /// alone (the batch's payloads and payload fields; no inputs), decided
    /// once at compile; null is Ranges.
    RigExecRangePlan (*plan)(const RigExecOpBatch &batch, size_t k) = nullptr;
    /// Builds what every range shares this frame into the node's state.
    /// False fails the node before any range runs. Optional.
    bool (*prepare)(const RigExecOpBatch &batch, size_t k, size_t count,
                    std::string *error) = nullptr;
    /// Writes elements [range.begin, range.end) of the pointwise outputs.
    /// False fails the node.
    bool (*run)(const RigExecOpBatch &batch, size_t k,
                const RigExecOpRange &range, std::string *error) = nullptr;
    /// Elements [*begin, *end) of the neighbourhood input that \p range
    /// reads, so a range waits only for those of the node feeding it;
    /// called after prepare, before any range runs, never concurrently
    /// for one node. Null reads every element.
    void (*reads)(const RigExecOpBatch &batch, size_t k,
                  const RigExecOpRange &range, size_t *begin,
                  size_t *end) = nullptr;
    /// Each range's scratch: created when the range is first cut and kept
    /// until the evaluator unbinds. Results never depend on what it holds.
    RigExecOpStateHandling scratch;
};

/// The elements RigExecOpCheck gives an optional input nothing feeds.
inline constexpr size_t kRigExecPortUnfed = SIZE_MAX - 1;

/// What validate sees of node k of a batch, and per port what is known
/// before it runs. For an input the host writes, the batch's input is its
/// value and elements[port] what it holds; for an input another node
/// feeds, the input is null and elements[port] what it holds when its
/// bound says so exactly, else SIZE_MAX; for an unfed optional input,
/// kRigExecPortUnfed. At compile no value is known and every fed input's
/// elements are SIZE_MAX. A check that needs a value validate does not
/// see is made while the node runs, as an error of the evaluation; state,
/// the node's working state (null at compile), is where validate notes
/// that.
struct RigExecOpCheck {
    RigExecOpBatch batch;
    size_t k = 0;
    const size_t *elements = nullptr;
    void *state = nullptr;
    bool compiling = false;
};

/// Where the schema class of an op's authored properties comes from.
enum class RigExecSchemaMode : uint8_t {
    /// Generated from the op's authoring metadata.
    Generated,
    /// Written by hand; the properties the op declares are checked against
    /// it.
    HandAuthored,
    /// None: the op has no class of its own; the compiler builds its nodes
    /// from other nodes and compile facts.
    Internal,
};

/// The schema class an op is authored with.
struct RigExecSchemaInfo {
    /// Empty for an Internal op.
    const char *className = "";
    const char *doc = nullptr;
    /// The class it inherits, such as "Typed"; null for none.
    const char *base = nullptr;
    RigExecSchemaMode mode = RigExecSchemaMode::HandAuthored;
};

/// Describes one op: its name and version, its ports, how it runs, where
/// it may run, what it costs, and how to handle its settings and working
/// state. An op provides run, runBatch or both; the evaluator calls
/// runBatch once per batch when there is one.
struct RigExecOpDesc {
    uint32_t structSize = sizeof(RigExecOpDesc);
    const char *nameSpace = "";
    const char *name = "";
    uint32_t version = 1;
    const RigExecPortDesc *ports = nullptr;
    size_t portCount = 0;
    RigExecOpRunFn run = nullptr;
    RigExecOpThreading threading = RigExecOpThreading::Pure;
    uint32_t capabilities = 0;
    double costFixed = 0.0;
    double costPerElement = 0.0;
    RigExecOpPayloadHandling payload;
    RigExecOpStateHandling state;
    /// Windowed ops only, and required there: how many earlier frames a
    /// node reads, given its payload (null when it has none), or for an op
    /// that declares payload fields its RigExecFieldValues.
    uint32_t (*windowFrames)(const void *payload) = nullptr;
    RigExecOpBatchFn runBatch = nullptr;
    /// Settings declared as fields, in place of payload handling: the
    /// library lays them out, copies, compares, saves and reads them. An
    /// op that declares fields provides runBatch.
    const RigExecFieldDesc *payloadFields = nullptr;
    size_t payloadFieldCount = 0;
    /// Saved ahead of the declared settings and checked when they are
    /// read; 0 saves none.
    uint32_t payloadVersion = 0;
    /// Fills \p out with the field values of \p payload, the settings
    /// struct a host passes to RigExecRigDescBuilder::AddOp. Optional.
    void (*payloadValues)(const void *payload,
                          RigExecFieldValues &out) = nullptr;
    /// Working state declared as fields, in place of state handling: each
    /// value is constructed when an evaluator binds the program and kept
    /// from one evaluation to the next.
    const RigExecFieldDesc *stateFields = nullptr;
    size_t stateFieldCount = 0;
    /// For a Ranged op, in place of run and runBatch.
    RigExecOpRangeHandling range;
    /// Checks a node before it runs, from what RigExecOpCheck says is
    /// known: at compile, and before the first evaluation and every one
    /// after an input changed size or a kRigExecPortChecked input changed
    /// value. False with \p error fails that compile or evaluation; the
    /// node never runs on inputs it refused. What only the inputs' values
    /// computed in the frame can show is an error of run itself. Null
    /// accepts every node.
    bool (*validate)(const RigExecOpCheck &check, std::string *error) = nullptr;
    /// The schema class the op is authored with; null when not declared.
    const RigExecSchemaInfo *schema = nullptr;
};

/// Receives a type library's types during Initialize.
class RigExecTypeRegistrar {
public:
    virtual ~RigExecTypeRegistrar() = default;
    virtual void Add(const RigExecTypeHandle &handle,
                     const RigExecTypeDesc &desc) = 0;
};

/// Receives an op library's ops during Initialize. Op libraries do not
/// define types; their ports name types from type libraries.
class RigExecOpRegistrar {
public:
    virtual ~RigExecOpRegistrar() = default;
    virtual void Add(const RigExecOpHandle &handle,
                     const RigExecOpDesc &desc) = 0;
};

/// One type library: the function that adds its types and the libraries
/// whose types it builds on.
struct RigExecTypeLibraryDesc {
    uint32_t structSize = sizeof(RigExecTypeLibraryDesc);
    uint32_t abiVersion = kRigExecRegistryAbiVersion;
    const char *name = "";
    uint64_t fingerprint = 0;
    /// Type libraries this one builds types from; they must come before it
    /// in RigExecRegistryInit.
    const RigExecTypeLibraryDesc *const *required = nullptr;
    size_t requiredCount = 0;
    void (*registerTypes)(RigExecTypeRegistrar &) = nullptr;
};

/// One op library: the function that adds its ops.
struct RigExecOpLibraryDesc {
    uint32_t structSize = sizeof(RigExecOpLibraryDesc);
    uint32_t abiVersion = kRigExecRegistryAbiVersion;
    const char *name = "";
    uint64_t fingerprint = 0;
    void (*registerOps)(RigExecOpRegistrar &) = nullptr;
};

/// The libraries to register, in the order Initialize processes them.
struct RigExecRegistryInit {
    std::vector<const RigExecTypeLibraryDesc *> typeLibraries;
    std::vector<const RigExecOpLibraryDesc *> opLibraries;
};

/// A registered type: its description, the library that added it, its
/// index and its handle.
struct RigExecRegisteredType {
    const RigExecTypeDesc *desc = nullptr;
    const RigExecTypeLibraryDesc *library = nullptr;
    uint32_t index = kRigExecInvalidIndex;
    const RigExecTypeHandle *handle = nullptr;
    /// The type's columns: its own scalars, then each member's, an array's
    /// element columns in a run of their own. Empty when some part of the
    /// type declares no columns.
    std::vector<RigExecTypeColumn> columns;
    /// Per run: the run whose values each hold one array of it, so an
    /// array nested in an array's elements names the outer array's run.
    /// Entry 0, the value itself, is kRigExecInvalidIndex.
    std::vector<uint32_t> runParents;
};

/// A registered op: its description, the library that added it, its index
/// and its ports' types.
struct RigExecRegisteredOp {
    const RigExecOpDesc *desc = nullptr;
    const RigExecOpLibraryDesc *library = nullptr;
    uint32_t index = kRigExecInvalidIndex;
    /// Type index per port, in port order.
    std::vector<uint32_t> portTypes;
    /// The op's handle, for a reader that finds the op by its name in a
    /// saved file.
    const RigExecOpHandle *handle = nullptr;
    /// The op's runBatch, or RigExecRunEachInstance when it has none; null
    /// for a Ranged op.
    RigExecOpBatchFn runBatch = nullptr;
    /// A Ranged op's first pointwise or neighbourhood input, which gives
    /// the element count, and its outputs, by port index.
    uint32_t rangeCountPort = kRigExecInvalidIndex;
    std::vector<uint32_t> rangeOutputs;
    /// Type index per declared payload and state field, in field order.
    std::vector<uint32_t> payloadFieldTypes;
    std::vector<uint32_t> stateFieldTypes;
    /// The outputs that declare a bound, by port index, ascending.
    std::vector<uint32_t> boundedOutputs;
};

/// The one registry of the process. Initialize builds it once at startup;
/// after that it never changes and lives until the process ends, so any
/// thread may read it without locking.
class RigExecRegistry {
public:
    /// Registers the given libraries' types, then their ops, and makes the
    /// result available through Get. On any error, returns false with the
    /// errors and registers nothing. Call it once.
    static bool Initialize(const RigExecRegistryInit &init,
                           std::vector<std::string> *errors = nullptr);

    /// Reports the errors Initialize would, without registering anything,
    /// and the fingerprint the registry would have.
    static bool Validate(const RigExecRegistryInit &init,
                         std::vector<std::string> *errors = nullptr,
                         uint64_t *fingerprint = nullptr);

    static bool IsInitialized();

    /// The registry. Call only after Initialize has succeeded.
    static const RigExecRegistry &Get();

    const RigExecRegisteredType &Type(const RigExecTypeHandle &h) const
    {
        return _types[h.Index()];
    }
    const RigExecRegisteredOp &Op(const RigExecOpHandle &h) const
    {
        return _ops[h.Index()];
    }
    const RigExecRegisteredType &TypeAt(uint32_t index) const
    {
        return _types[index];
    }
    const RigExecRegisteredOp &OpAt(uint32_t index) const
    {
        return _ops[index];
    }
    size_t TypeCount() const
    {
        return _types.size();
    }
    size_t OpCount() const
    {
        return _ops.size();
    }

    /// Identifies this exact set of types and ops, with their order and
    /// versions. A compiled program records it, and an evaluator refuses a
    /// program compiled against a different registry. Authoring metadata
    /// (RigExecPropertyInfo, RigExecSchemaInfo) does not enter it.
    uint64_t Fingerprint() const
    {
        return _fingerprint;
    }

private:
    friend class RigExecRegistryBuilder;
    struct _BuilderKey {
        explicit _BuilderKey(int)
        {
        }
    };

public:
    explicit RigExecRegistry(_BuilderKey)
    {
    }

private:
    std::vector<RigExecRegisteredType> _types;
    std::vector<RigExecRegisteredOp> _ops;
    uint64_t _fingerprint = 0;
};

/// The libraries this binary was linked with by the CMake function
/// rigexec_link_registrations, in registration order.
RigExecRegistryInit RigExecLinkedRegistrations();

} // namespace rigExec

/// Defines the handle of a type, op or port with an accessor
/// `const RigExecTypeHandle &fn()` (or the op or port handle type). Use at
/// namespace scope in the library's source file and declare fn in its
/// header.
#define RIGEXEC_DEFINE_TYPE_HANDLE(fn)                                         \
    namespace {                                                                \
    ::rigExec::RigExecTypeHandle fn##_rigExecHandle;                           \
    }                                                                          \
    const ::rigExec::RigExecTypeHandle &fn()                                   \
    {                                                                          \
        return fn##_rigExecHandle;                                             \
    }

#define RIGEXEC_DEFINE_OP_HANDLE(fn)                                           \
    namespace {                                                                \
    ::rigExec::RigExecOpHandle fn##_rigExecHandle;                             \
    }                                                                          \
    const ::rigExec::RigExecOpHandle &fn()                                     \
    {                                                                          \
        return fn##_rigExecHandle;                                             \
    }

#define RIGEXEC_DEFINE_PORT_HANDLE(fn)                                         \
    namespace {                                                                \
    ::rigExec::RigExecPortHandle fn##_rigExecHandle;                           \
    }                                                                          \
    const ::rigExec::RigExecPortHandle &fn()                                   \
    {                                                                          \
        return fn##_rigExecHandle;                                             \
    }

/// Defines the function through which a library hands its description to
/// the registry. rigexec_link_registrations calls it by name.
#define RIGEXEC_TYPE_LIBRARY_ENTRY(lib)                                        \
    extern "C" const ::rigExec::RigExecTypeLibraryDesc                         \
        *lib##_RigExecTypeLibrary()

#define RIGEXEC_OP_LIBRARY_ENTRY(lib)                                          \
    extern "C" const ::rigExec::RigExecOpLibraryDesc *lib##_RigExecOpLibrary()

#endif
