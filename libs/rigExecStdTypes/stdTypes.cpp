#include "rigExecStdTypes/stdTypes.h"

#include <cmath>
#include <cstring>
#include <memory>
#include <type_traits>

namespace rigExecStd {

RIGEXEC_DEFINE_TYPE_HANDLE(BoolType)
RIGEXEC_DEFINE_TYPE_HANDLE(IntType)
RIGEXEC_DEFINE_TYPE_HANDLE(UIntType)
RIGEXEC_DEFINE_TYPE_HANDLE(FloatType)
RIGEXEC_DEFINE_TYPE_HANDLE(DoubleType)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec2fType)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec2dType)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3fType)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3dType)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec4dType)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3iType)
RIGEXEC_DEFINE_TYPE_HANDLE(Mat3dType)
RIGEXEC_DEFINE_TYPE_HANDLE(Mat4dType)
RIGEXEC_DEFINE_TYPE_HANDLE(QuatdType)
RIGEXEC_DEFINE_TYPE_HANDLE(BoolArray)
RIGEXEC_DEFINE_TYPE_HANDLE(IntArray)
RIGEXEC_DEFINE_TYPE_HANDLE(UIntArray)
RIGEXEC_DEFINE_TYPE_HANDLE(FloatArray)
RIGEXEC_DEFINE_TYPE_HANDLE(DoubleArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec2fArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec2dArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3fArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3dArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec4dArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3iArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Mat3dArray)
RIGEXEC_DEFINE_TYPE_HANDLE(Mat4dArray)
RIGEXEC_DEFINE_TYPE_HANDLE(QuatdArray)

namespace {

using rigExec::RigExecEqualMode;
using rigExec::RigExecHashClass;
using rigExec::RigExecTypeDesc;
using rigExec::RigExecTypeHandle;
using rigExec::RigExecTypeRegistrar;

/// Scalar component type and count of a value type.
template <class T> struct Components {
    using Scalar = T;
    static constexpr size_t count = 1;
};
#define RIGEXEC_STD_COMPONENTS(T, S, N)                                        \
    template <> struct Components<T> {                                         \
        using Scalar = S;                                                      \
        static constexpr size_t count = N;                                     \
    };
RIGEXEC_STD_COMPONENTS(Vec2f, float, 2)
RIGEXEC_STD_COMPONENTS(Vec2d, double, 2)
RIGEXEC_STD_COMPONENTS(Vec3f, float, 3)
RIGEXEC_STD_COMPONENTS(Vec3d, double, 3)
RIGEXEC_STD_COMPONENTS(Vec4d, double, 4)
RIGEXEC_STD_COMPONENTS(Vec3i, int32_t, 3)
RIGEXEC_STD_COMPONENTS(Mat3d, double, 9)
RIGEXEC_STD_COMPONENTS(Mat4d, double, 16)
RIGEXEC_STD_COMPONENTS(Quatd, double, 4)
#undef RIGEXEC_STD_COMPONENTS

template <class T>
constexpr bool kPacked =
    sizeof(T) == sizeof(typename Components<T>::Scalar) * Components<T>::count;
static_assert(kPacked<Vec3f> && kPacked<Vec3d> && kPacked<Mat4d> &&
                  kPacked<Quatd> && kPacked<Vec3i>,
              "standard values are packed scalar components");

template <class S>
bool
ScalarsEqual(const S *a, const S *b, size_t n, RigExecEqualMode mode)
{
    if (mode == RigExecEqualMode::Bitwise || !std::is_floating_point_v<S>)
        return std::memcmp(a, b, n * sizeof(S)) == 0;
    for (size_t i = 0; i < n; ++i) {
        if (a[i] == b[i])
            continue;
        if (mode == RigExecEqualMode::NaNEqual && std::isnan(double(a[i])) &&
            std::isnan(double(b[i])))
            continue;
        return false;
    }
    return true;
}

uint64_t
HashBytes(const void *data, size_t size, uint64_t seed)
{
    // FNV-1a.
    uint64_t h = seed ^ 1469598103934665603ull;
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        h ^= bytes[i];
        h *= 1099511628211ull;
    }
    return h;
}

template <class T> struct ValueOps {
    using C = Components<T>;
    static bool Equal(const void *a, const void *b, RigExecEqualMode mode)
    {
        return ScalarsEqual(reinterpret_cast<const typename C::Scalar *>(a),
                            reinterpret_cast<const typename C::Scalar *>(b),
                            C::count, mode);
    }
    static uint64_t Hash(const void *value)
    {
        return HashBytes(value, sizeof(T), 0);
    }
};

template <class T> struct ArrayOps {
    using A = RigExecArray<T>;
    using C = Components<T>;
    static void Construct(void *dst)
    {
        std::uninitialized_value_construct_n(static_cast<A *>(dst), 1);
    }
    static void Copy(void *dst, const void *src)
    {
        *static_cast<A *>(dst) = *static_cast<const A *>(src);
    }
    static void Move(void *dst, void *src)
    {
        *static_cast<A *>(dst) = std::move(*static_cast<A *>(src));
    }
    static void Destroy(void *dst)
    {
        std::destroy_at(static_cast<A *>(dst));
    }
    static bool Equal(const void *a, const void *b, RigExecEqualMode mode)
    {
        const A &x = *static_cast<const A *>(a);
        const A &y = *static_cast<const A *>(b);
        return x.Size() == y.Size() &&
               ScalarsEqual(
                   reinterpret_cast<const typename C::Scalar *>(x.Data()),
                   reinterpret_cast<const typename C::Scalar *>(y.Data()),
                   x.Size() * C::count, mode);
    }
    static uint64_t Hash(const void *value)
    {
        const A &x = *static_cast<const A *>(value);
        const uint64_t size = x.Size();
        return HashBytes(x.Data(), x.Size() * sizeof(T),
                         HashBytes(&size, sizeof(size), 0));
    }
};

/// Layout hash: the scalar width and kind, component count and size, so a
/// change to any of them is a new layout.
template <class T>
uint64_t
LayoutHash(uint64_t arrayFlag)
{
    using C = Components<T>;
    const uint64_t fields[] = {sizeof(typename C::Scalar),
                               std::is_floating_point_v<typename C::Scalar>,
                               std::is_signed_v<typename C::Scalar>,
                               C::count,
                               sizeof(T),
                               alignof(T),
                               arrayFlag};
    return HashBytes(fields, sizeof(fields), 0);
}

template <class T>
RigExecTypeDesc
ValueDesc(const char *name)
{
    RigExecTypeDesc d;
    d.nameSpace = "std";
    d.name = name;
    d.layoutHash = LayoutHash<T>(0);
    d.size = sizeof(T);
    d.align = alignof(T);
    d.triviallyCopyable = true;
    d.equal = &ValueOps<T>::Equal;
    d.hashClass = RigExecHashClass::Exact;
    d.hash = &ValueOps<T>::Hash;
    return d;
}

template <class T>
RigExecTypeDesc
ArrayDesc(const char *name, const RigExecTypeHandle &element)
{
    using A = RigExecArray<T>;
    RigExecTypeDesc d;
    d.nameSpace = "std";
    d.name = name;
    d.layoutHash = LayoutHash<T>(1);
    d.size = sizeof(A);
    d.align = alignof(A);
    d.element = &element;
    d.construct = &ArrayOps<T>::Construct;
    d.copy = &ArrayOps<T>::Copy;
    d.move = &ArrayOps<T>::Move;
    d.destroy = &ArrayOps<T>::Destroy;
    d.equal = &ArrayOps<T>::Equal;
    d.hashClass = RigExecHashClass::Exact;
    d.hash = &ArrayOps<T>::Hash;
    return d;
}

void
Register(RigExecTypeRegistrar &registrar)
{
    struct Entry {
        const RigExecTypeHandle &handle;
        RigExecTypeDesc desc;
    };
    // Function-local so descriptors outlive registration; built once, on the
    // owner thread, during freeze.
    static const Entry entries[] = {
        {BoolType(), ValueDesc<uint8_t>("Bool")},
        {IntType(), ValueDesc<int32_t>("Int")},
        {UIntType(), ValueDesc<uint32_t>("UInt")},
        {FloatType(), ValueDesc<float>("Float")},
        {DoubleType(), ValueDesc<double>("Double")},
        {Vec2fType(), ValueDesc<Vec2f>("Vec2f")},
        {Vec2dType(), ValueDesc<Vec2d>("Vec2d")},
        {Vec3fType(), ValueDesc<Vec3f>("Vec3f")},
        {Vec3dType(), ValueDesc<Vec3d>("Vec3d")},
        {Vec4dType(), ValueDesc<Vec4d>("Vec4d")},
        {Vec3iType(), ValueDesc<Vec3i>("Vec3i")},
        {Mat3dType(), ValueDesc<Mat3d>("Mat3d")},
        {Mat4dType(), ValueDesc<Mat4d>("Mat4d")},
        {QuatdType(), ValueDesc<Quatd>("Quatd")},
        {BoolArray(), ArrayDesc<uint8_t>("BoolArray", BoolType())},
        {IntArray(), ArrayDesc<int32_t>("IntArray", IntType())},
        {UIntArray(), ArrayDesc<uint32_t>("UIntArray", UIntType())},
        {FloatArray(), ArrayDesc<float>("FloatArray", FloatType())},
        {DoubleArray(), ArrayDesc<double>("DoubleArray", DoubleType())},
        {Vec2fArray(), ArrayDesc<Vec2f>("Vec2fArray", Vec2fType())},
        {Vec2dArray(), ArrayDesc<Vec2d>("Vec2dArray", Vec2dType())},
        {Vec3fArray(), ArrayDesc<Vec3f>("Vec3fArray", Vec3fType())},
        {Vec3dArray(), ArrayDesc<Vec3d>("Vec3dArray", Vec3dType())},
        {Vec4dArray(), ArrayDesc<Vec4d>("Vec4dArray", Vec4dType())},
        {Vec3iArray(), ArrayDesc<Vec3i>("Vec3iArray", Vec3iType())},
        {Mat3dArray(), ArrayDesc<Mat3d>("Mat3dArray", Mat3dType())},
        {Mat4dArray(), ArrayDesc<Mat4d>("Mat4dArray", Mat4dType())},
        {QuatdArray(), ArrayDesc<Quatd>("QuatdArray", QuatdType())},
    };
    for (const Entry &entry : entries)
        registrar.Add(entry.handle, entry.desc);
}

const rigExec::RigExecTypeLibraryDesc kLibrary = [] {
    rigExec::RigExecTypeLibraryDesc d;
    d.name = "rigExecStdTypes";
    d.registerTypes = &Register;
    return d;
}();

} // namespace
} // namespace rigExecStd

RIGEXEC_TYPE_LIBRARY_ENTRY(rigExecStdTypes)
{
    return &rigExecStd::kLibrary;
}
