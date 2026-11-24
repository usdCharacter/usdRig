#ifndef RIGEXEC_STD_TYPES_VALUE_OPS_H
#define RIGEXEC_STD_TYPES_VALUE_OPS_H

/// Equality, hashing, layout keys and type descriptors for the standard values,
/// their arrays, and records built from them. Type libraries built on the
/// standard one describe their types with these, so every type follows one
/// equality and hash rule set.
///
/// A record lists its members for these templates:
///
///     struct Foo {
///         rigExecStd::RigExecArray<float> values;
///         uint32_t count = 0;
///         static constexpr auto Members()
///         {
///             return std::make_tuple(&Foo::values, &Foo::count);
///         }
///     };

#include "rigExec/api/registry.h"
#include "rigExecRuntime/runtimeMath.h"
#include "rigExecStdTypes/array.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

namespace rigExecStd {

/// Scalar component type and count of a standard value.
template <class T> struct Components {
    using Scalar = T;
    static constexpr size_t count = 1;
};
#define RIGEXEC_STD_COMPONENTS(T, S, N)                                        \
    template <> struct Components<rigExec::T> {                                \
        using Scalar = S;                                                      \
        static constexpr size_t count = N;                                     \
    };
RIGEXEC_STD_COMPONENTS(RrVec2f, float, 2)
RIGEXEC_STD_COMPONENTS(RrVec2d, double, 2)
RIGEXEC_STD_COMPONENTS(RrVec3f, float, 3)
RIGEXEC_STD_COMPONENTS(RrVec3d, double, 3)
RIGEXEC_STD_COMPONENTS(RrVec4d, double, 4)
RIGEXEC_STD_COMPONENTS(RrVec3i, int32_t, 3)
RIGEXEC_STD_COMPONENTS(RrMat3d, double, 9)
RIGEXEC_STD_COMPONENTS(RrMat4d, double, 16)
RIGEXEC_STD_COMPONENTS(RrQuatd, double, 4)
#undef RIGEXEC_STD_COMPONENTS

/// The column kind of scalar \p S; None for any other type.
template <class S>
constexpr rigExec::RigExecColumnKind
ColumnKindOf()
{
    using K = rigExec::RigExecColumnKind;
    if constexpr (std::is_same_v<S, uint8_t>)
        return K::UInt8;
    else if constexpr (std::is_same_v<S, int32_t>)
        return K::Int32;
    else if constexpr (std::is_same_v<S, uint32_t>)
        return K::UInt32;
    else if constexpr (std::is_same_v<S, uint64_t>)
        return K::UInt64;
    else if constexpr (std::is_same_v<S, float>)
        return K::Float32;
    else if constexpr (std::is_same_v<S, double>)
        return K::Float64;
    else
        return K::None;
}

/// FNV-1a.
inline uint64_t
HashBytes(const void *data, size_t size, uint64_t seed)
{
    uint64_t h = seed ^ 1469598103934665603ull;
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        h ^= bytes[i];
        h *= 1099511628211ull;
    }
    return h;
}

template <class S>
bool
ScalarsEqual(const S *a, const S *b, size_t n, rigExec::RigExecEqualMode mode)
{
    if (mode == rigExec::RigExecEqualMode::Bitwise ||
        !std::is_floating_point_v<S>)
        return n == 0 || std::memcmp(a, b, n * sizeof(S)) == 0;
    for (size_t i = 0; i < n; ++i) {
        if (a[i] == b[i])
            continue;
        if (mode == rigExec::RigExecEqualMode::NaNEqual &&
            std::isnan(double(a[i])) && std::isnan(double(b[i])))
            continue;
        return false;
    }
    return true;
}

template <class T, class = void> struct IsRecord : std::false_type {};
template <class T>
struct IsRecord<T, std::void_t<decltype(T::Members())>> : std::true_type {};

template <class T> struct IsArray : std::false_type {};
template <class T> struct IsArray<RigExecArray<T>> : std::true_type {};

template <class T> struct IsFixed : std::false_type {};
template <class T, size_t N>
struct IsFixed<std::array<T, N>> : std::true_type {};

/// Whether T is a run of scalars with no padding, compared and hashed as
/// one span.
template <class T>
constexpr bool kScalarRun =
    !IsRecord<T>::value && !IsFixed<T>::value && !IsArray<T>::value;

template <class T>
bool ValueEqual(const T &a, const T &b, rigExec::RigExecEqualMode mode);
template <class T> uint64_t ValueHash(const T &value, uint64_t seed);

template <class T>
bool
ValueEqual(const T &a, const T &b, rigExec::RigExecEqualMode mode)
{
    if constexpr (IsRecord<T>::value) {
        return std::apply(
            [&](auto... member) {
                return (ValueEqual(a.*member, b.*member, mode) && ...);
            },
            T::Members());
    } else if constexpr (IsFixed<T>::value) {
        for (size_t i = 0; i < a.size(); ++i) {
            if (!ValueEqual(a[i], b[i], mode))
                return false;
        }
        return true;
    } else if constexpr (IsArray<T>::value) {
        using E = std::remove_cv_t<std::remove_reference_t<decltype(a[0])>>;
        using C = Components<E>;
        if (a.Size() != b.Size())
            return false;
        if constexpr (!kScalarRun<E>) {
            for (size_t i = 0; i < a.Size(); ++i) {
                if (!ValueEqual(a[i], b[i], mode))
                    return false;
            }
            return true;
        } else {
            return ScalarsEqual(
                reinterpret_cast<const typename C::Scalar *>(a.Data()),
                reinterpret_cast<const typename C::Scalar *>(b.Data()),
                a.Size() * C::count, mode);
        }
    } else {
        using C = Components<T>;
        static_assert(sizeof(T) == sizeof(typename C::Scalar) * C::count,
                      "a standard value is packed scalar components");
        return ScalarsEqual(reinterpret_cast<const typename C::Scalar *>(&a),
                            reinterpret_cast<const typename C::Scalar *>(&b),
                            C::count, mode);
    }
}

/// Exact hash: equal under Bitwise means equal hash.
template <class T>
uint64_t
ValueHash(const T &value, uint64_t seed)
{
    if constexpr (IsRecord<T>::value) {
        return std::apply(
            [&](auto... member) {
                uint64_t h = seed;
                ((h = ValueHash(value.*member, h)), ...);
                return h;
            },
            T::Members());
    } else if constexpr (IsFixed<T>::value) {
        uint64_t h = seed;
        for (const auto &element : value)
            h = ValueHash(element, h);
        return h;
    } else if constexpr (IsArray<T>::value) {
        using E = std::remove_pointer_t<decltype(value.Data())>;
        const uint64_t size = value.Size();
        const uint64_t h = HashBytes(&size, sizeof(size), seed);
        if constexpr (!kScalarRun<std::remove_cv_t<E>>) {
            uint64_t e = h;
            for (const auto &element : value)
                e = ValueHash(element, e);
            return e;
        } else {
            return HashBytes(value.Data(), value.Size() * sizeof(E), h);
        }
    } else {
        return HashBytes(&value, sizeof(T), seed);
    }
}

/// Whether T is an array or holds one among its members.
template <class T>
constexpr bool
HoldsArrays()
{
    if constexpr (IsArray<T>::value) {
        return true;
    } else if constexpr (IsRecord<T>::value) {
        return std::apply(
            [](auto... member) {
                return (HoldsArrays<std::remove_cv_t<std::remove_reference_t<
                            decltype(std::declval<T &>().*member)>>>() ||
                        ...);
            },
            T::Members());
    } else if constexpr (IsFixed<T>::value) {
        return HoldsArrays<typename T::value_type>();
    } else {
        return false;
    }
}

/// Makes room for \p count elements in each array \p value holds.
template <class T>
void
ReserveArrays(T &value, size_t count)
{
    if constexpr (IsArray<T>::value) {
        value.Reserve(count);
    } else if constexpr (IsRecord<T>::value) {
        std::apply(
            [&](auto... member) { (ReserveArrays(value.*member, count), ...); },
            T::Members());
    } else if constexpr (IsFixed<T>::value) {
        for (auto &element : value)
            ReserveArrays(element, count);
    }
}

/// The most elements any array \p value holds has.
template <class T>
size_t
LongestArray(const T &value)
{
    if constexpr (IsArray<T>::value) {
        return value.Size();
    } else if constexpr (IsRecord<T>::value) {
        return std::apply(
            [&](auto... member) {
                size_t longest = 0;
                ((longest = std::max(longest, LongestArray(value.*member))),
                 ...);
                return longest;
            },
            T::Members());
    } else if constexpr (IsFixed<T>::value) {
        size_t longest = 0;
        for (const auto &element : value)
            longest = std::max(longest, LongestArray(element));
        return longest;
    } else {
        return 0;
    }
}

/// Changes when the layout of T changes: scalar kinds and widths, counts,
/// sizes, alignments and member offsets.
template <class T>
uint64_t
LayoutKey()
{
    if constexpr (IsRecord<T>::value) {
        return std::apply(
            [](auto... member) {
                const T probe{};
                const auto base = reinterpret_cast<const char *>(&probe);
                uint64_t h = HashBytes("record", 6, 0);
                const uint64_t shape[] = {sizeof(T), alignof(T)};
                h = HashBytes(shape, sizeof(shape), h);
                const auto add = [&](auto m) {
                    using M = std::remove_cv_t<
                        std::remove_reference_t<decltype(probe.*m)>>;
                    const uint64_t key[] = {
                        uint64_t(reinterpret_cast<const char *>(&(probe.*m)) -
                                 base),
                        LayoutKey<M>()};
                    h = HashBytes(key, sizeof(key), h);
                };
                (add(member), ...);
                return h;
            },
            T::Members());
    } else if constexpr (IsFixed<T>::value) {
        using E = typename T::value_type;
        const uint64_t key[] = {std::tuple_size<T>::value, LayoutKey<E>()};
        return HashBytes(key, sizeof(key), HashBytes("fixed", 5, 0));
    } else if constexpr (IsArray<T>::value) {
        using E = std::remove_pointer_t<decltype(std::declval<T>().Data())>;
        const uint64_t key[] = {sizeof(T), alignof(T), LayoutKey<E>()};
        return HashBytes(key, sizeof(key), HashBytes("array", 5, 0));
    } else {
        using C = Components<T>;
        using S = typename C::Scalar;
        const uint64_t key[] = {
            sizeof(S),           std::is_floating_point_v<S>,
            std::is_signed_v<S>, C::count,
            sizeof(T),           alignof(T)};
        return HashBytes(key, sizeof(key), HashBytes("value", 5, 0));
    }
}

/// Type-erased entry points for a registered T.
template <class T> struct TypeOps {
    static void Construct(void *dst)
    {
        std::uninitialized_value_construct_n(static_cast<T *>(dst), 1);
    }
    static void Copy(void *dst, const void *src)
    {
        *static_cast<T *>(dst) = *static_cast<const T *>(src);
    }
    static void Move(void *dst, void *src)
    {
        *static_cast<T *>(dst) = std::move(*static_cast<T *>(src));
    }
    static void Destroy(void *dst)
    {
        std::destroy_at(static_cast<T *>(dst));
    }
    static bool Equal(const void *a, const void *b,
                      rigExec::RigExecEqualMode mode)
    {
        return ValueEqual(*static_cast<const T *>(a),
                          *static_cast<const T *>(b), mode);
    }
    static uint64_t Hash(const void *value)
    {
        return ValueHash(*static_cast<const T *>(value), 0);
    }
    static void Reserve(void *value, size_t count)
    {
        ReserveArrays(*static_cast<T *>(value), count);
    }
    static size_t Elements(const void *value)
    {
        return LongestArray(*static_cast<const T *>(value));
    }
};

/// The descriptor of T, registered as (nameSpace, name). \p element names
/// an array's element type; \p fields a record's member types in member
/// order. Both must outlive the registry.
template <class T>
rigExec::RigExecTypeDesc
TypeDescFor(const char *nameSpace, const char *name,
            const rigExec::RigExecTypeHandle *element = nullptr,
            const rigExec::RigExecTypeHandle *const *fields = nullptr,
            size_t fieldCount = 0)
{
    rigExec::RigExecTypeDesc d;
    d.nameSpace = nameSpace;
    d.name = name;
    d.layoutHash = LayoutKey<T>();
    d.size = sizeof(T);
    d.align = alignof(T);
    d.triviallyCopyable = std::is_trivially_copyable_v<T>;
    d.element = element;
    d.fields = fields;
    d.fieldCount = fieldCount;
    if (!std::is_trivially_default_constructible_v<T>)
        d.construct = &TypeOps<T>::Construct;
    if (!d.triviallyCopyable) {
        d.copy = &TypeOps<T>::Copy;
        d.move = &TypeOps<T>::Move;
        d.destroy = &TypeOps<T>::Destroy;
    }
    d.equal = &TypeOps<T>::Equal;
    d.hashClass = rigExec::RigExecHashClass::Exact;
    d.hash = &TypeOps<T>::Hash;
    if constexpr (IsArray<T>::value) {
        d.count = [](const void *value) {
            return static_cast<const T *>(value)->Size();
        };
        d.resize = [](void *value, size_t count) {
            static_cast<T *>(value)->Resize(count);
        };
    }
    if constexpr (HoldsArrays<T>()) {
        d.reserve = &TypeOps<T>::Reserve;
        d.elements = &TypeOps<T>::Elements;
    }
    if constexpr (!IsArray<T>::value && !IsRecord<T>::value &&
                  !IsFixed<T>::value) {
        if (!element && !fieldCount) {
            d.columnKind = ColumnKindOf<typename Components<T>::Scalar>();
            if (d.columnKind != rigExec::RigExecColumnKind::None)
                d.columnCount = uint32_t(Components<T>::count);
        }
    }
    return d;
}

} // namespace rigExecStd

#endif
