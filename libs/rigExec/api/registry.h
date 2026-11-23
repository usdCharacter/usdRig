#ifndef RIGEXEC_API_REGISTRY_H
#define RIGEXEC_API_REGISTRY_H

/// Type and op registry. Types are global; ops reference registered types on
/// their ports. Everything is reached through handles; names are display
/// data and boundary keys only. USD-free.

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
/// rigExecGraph/windowHistory.h.
struct RigExecWindowRun;

inline constexpr uint32_t kRigExecInvalidIndex = 0xffffffffu;

/// Raises a library's ABI version on any layout change of the descriptors
/// below.
inline constexpr uint32_t kRigExecRegistryAbiVersion = 3;

/// Dense index assigned at freeze. Constant-initialized at namespace scope
/// by the RIGEXEC_DEFINE_*_HANDLE macros; read only after Initialize.
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

/// Port index within its op.
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

enum class RigExecEqualMode : uint8_t { Bitwise, NaNEqual, Ieee };
enum class RigExecHashClass : uint8_t { Exact, Generic, Unhashable };

/// One global data type.
struct RigExecTypeDesc {
    uint32_t structSize = sizeof(RigExecTypeDesc);
    /// Stable id: (nameSpace, name, major). Manifest and messages only.
    const char *nameSpace = "";
    const char *name = "";
    uint32_t major = 1;
    uint32_t minor = 0;
    uint64_t layoutHash = 0;
    size_t size = 0;
    size_t align = 1;
    bool triviallyCopyable = false;
    /// Array element type, when this type is an array.
    const RigExecTypeHandle *element = nullptr;
    /// Record field types, when this type is a record.
    const RigExecTypeHandle *const *fields = nullptr;
    size_t fieldCount = 0;
    void (*construct)(void *dst) = nullptr;
    void (*copy)(void *dst, const void *src) = nullptr;
    void (*move)(void *dst, void *src) = nullptr;
    void (*destroy)(void *dst) = nullptr;
    bool (*equal)(const void *a, const void *b, RigExecEqualMode) = nullptr;
    RigExecHashClass hashClass = RigExecHashClass::Unhashable;
    uint64_t (*hash)(const void *value) = nullptr;
};

enum RigExecPortFlags : uint32_t {
    kRigExecPortInput = 1u << 0,
    kRigExecPortOutput = 1u << 1,
    kRigExecPortOptional = 1u << 2,
    /// Input only: fed by any number of connections and bindings, kept in
    /// call order. The port's type is the element type; the op reads a
    /// RigExecPortList.
    kRigExecPortArray = 1u << 3,
};

struct RigExecPortDesc {
    const RigExecPortHandle *port = nullptr;
    const RigExecTypeHandle *type = nullptr;
    uint32_t flags = 0;
    /// Stable name used by schema generation and messages.
    const char *name = "";
};

enum class RigExecOpThreading : uint8_t { Pure, OwnerThread, Device, Windowed };

/// What the evaluators may do with an op. A point op's bits mirror the
/// lists its RigExecRevisionOp is still classified by; a test keeps them
/// equal.
enum RigExecOpCapabilities : uint32_t {
    kRigExecOpBakeable = 1u << 0,
    kRigExecOpFreezable = 1u << 1,
    kRigExecOpGpu = 1u << 2,
    /// A gated sparse zero-default envelope leaves unlisted points at their
    /// entering bytes.
    kRigExecOpGate = 1u << 3,
    /// The envelope is blended back over the entering points after the run,
    /// rather than folded into the op's arithmetic.
    kRigExecOpSeparateBlend = 1u << 4,
    /// Runs one vertex group at a time.
    kRigExecOpRanged = 1u << 5,
    /// The scene assembles its inputs from declared leaf reads.
    kRigExecOpFromLeaves = 1u << 6,
};

/// Opaque compile-time configuration an op owns. Not a data type.
struct RigExecOpPayloadHandling {
    size_t size = 0;
    size_t align = 1;
    void (*construct)(void *dst) = nullptr;
    void (*copy)(void *dst, const void *src) = nullptr;
    void (*destroy)(void *dst) = nullptr;
    uint64_t (*hash)(const void *payload) = nullptr;
    bool (*equal)(const void *a, const void *b) = nullptr;
};

template <class T, class = void>
struct RigExecPayloadComparable : std::false_type {};
template <class T>
struct RigExecPayloadComparable<
    T, std::void_t<decltype(std::declval<const T &>() ==
                            std::declval<const T &>())>> : std::true_type {};

/// Payload handling for a copyable \p T: equality where \p T has
/// operator==, no hash.
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

/// Per-instance caches and history an op owns.
struct RigExecOpStateHandling {
    size_t size = 0;
    size_t align = 1;
    void (*create)(void *dst) = nullptr;
    void (*copy)(void *dst, const void *src) = nullptr;
    void (*reset)(void *dst) = nullptr;
    void (*destroy)(void *dst) = nullptr;
};

/// An array input's element values, in feed order.
struct RigExecPortList {
    const void *const *items = nullptr;
    size_t count = 0;
};

/// Port values for one run, indexed by port handle. An array input's entry
/// points to its RigExecPortList, empty when unfed.
struct RigExecOpIo {
    const void *const *inputs = nullptr;
    void *const *outputs = nullptr;
    size_t portCount = 0;
    /// A Windowed op's earlier ticks and capture slot; null for other ops.
    /// The op writes only its own capture.
    const RigExecWindowRun *window = nullptr;
};

using RigExecOpRunFn = bool (*)(const void *payload, void *state,
                                const RigExecOpIo &io, std::string *error);

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
    /// Required for Windowed threading, and only there: the earlier ticks a
    /// node reads, from its payload (null without one). Fixed per program.
    uint32_t (*windowFrames)(const void *payload) = nullptr;
};

/// Collects a type library's types during freeze.
class RigExecTypeRegistrar {
public:
    virtual ~RigExecTypeRegistrar() = default;
    virtual void Add(const RigExecTypeHandle &handle,
                     const RigExecTypeDesc &desc) = 0;
};

/// Collects an op library's ops during freeze. No type API: ports name
/// type handles from type library headers.
class RigExecOpRegistrar {
public:
    virtual ~RigExecOpRegistrar() = default;
    virtual void Add(const RigExecOpHandle &handle,
                     const RigExecOpDesc &desc) = 0;
};

struct RigExecTypeLibraryDesc {
    uint32_t structSize = sizeof(RigExecTypeLibraryDesc);
    uint32_t abiVersion = kRigExecRegistryAbiVersion;
    const char *name = "";
    uint64_t fingerprint = 0;
    /// Type libraries this one composes types from; they freeze first.
    const RigExecTypeLibraryDesc *const *required = nullptr;
    size_t requiredCount = 0;
    void (*registerTypes)(RigExecTypeRegistrar &) = nullptr;
};

struct RigExecOpLibraryDesc {
    uint32_t structSize = sizeof(RigExecOpLibraryDesc);
    uint32_t abiVersion = kRigExecRegistryAbiVersion;
    const char *name = "";
    uint64_t fingerprint = 0;
    void (*registerOps)(RigExecOpRegistrar &) = nullptr;
};

struct RigExecRegistryInit {
    std::vector<const RigExecTypeLibraryDesc *> typeLibraries;
    std::vector<const RigExecOpLibraryDesc *> opLibraries;
};

/// A registered type, as Get().Type(handle) returns it.
struct RigExecRegisteredType {
    const RigExecTypeDesc *desc = nullptr;
    const RigExecTypeLibraryDesc *library = nullptr;
    uint32_t index = kRigExecInvalidIndex;
};

struct RigExecRegisteredOp {
    const RigExecOpDesc *desc = nullptr;
    const RigExecOpLibraryDesc *library = nullptr;
    uint32_t index = kRigExecInvalidIndex;
    /// Type index per port, in port order.
    std::vector<uint32_t> portTypes;
};

/// The process-wide registry: built once by Initialize on the owner thread,
/// immutable and immortal after it. Reads take no lock.
class RigExecRegistry {
public:
    /// Freezes types then ops and publishes. False with errors and nothing
    /// published on any error; a second call is an error.
    static bool Initialize(const RigExecRegistryInit &init,
                           std::vector<std::string> *errors = nullptr);

    /// Runs every freeze check without publishing or writing handles.
    static bool Validate(const RigExecRegistryInit &init,
                         std::vector<std::string> *errors = nullptr);

    static bool IsInitialized();

    /// Requires IsInitialized().
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

    /// Hash of the ordered types and ops with versions and layout hashes.
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

/// The libraries rigexec_link_registrations generated into this binary, in
/// freeze order.
RigExecRegistryInit RigExecLinkedRegistrations();

} // namespace rigExec

/// Defines a handle accessor `const RigExecTypeHandle &fn()`. Use at
/// namespace scope in the library's source; declare fn in its header.
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

/// Entry points a library exports; rigexec_link_registrations references
/// them by name from the generated tables.
#define RIGEXEC_TYPE_LIBRARY_ENTRY(lib)                                        \
    extern "C" const ::rigExec::RigExecTypeLibraryDesc                         \
        *lib##_RigExecTypeLibrary()

#define RIGEXEC_OP_LIBRARY_ENTRY(lib)                                          \
    extern "C" const ::rigExec::RigExecOpLibraryDesc *lib##_RigExecOpLibrary()

#endif
