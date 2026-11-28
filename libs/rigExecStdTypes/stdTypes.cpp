#include "rigExecStdTypes/stdTypes.h"
#include "rigExecStdTypes/valueOps.h"

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
RIGEXEC_DEFINE_TYPE_HANDLE(UInt64Type)
RIGEXEC_DEFINE_TYPE_HANDLE(IntArrayList)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3fArrayList)

namespace {

using rigExec::RigExecTypeDesc;
using rigExec::RigExecTypeHandle;
using rigExec::RigExecTypeRegistrar;

template <class T>
RigExecTypeDesc
ValueDesc(const char *name)
{
    return TypeDescFor<T>("std", name);
}

template <class T>
RigExecTypeDesc
ArrayDesc(const char *name, const RigExecTypeHandle &element)
{
    return TypeDescFor<RigExecArray<T>>("std", name, &element);
}

template <class T>
RigExecTypeDesc
ListDesc(const char *name, const RigExecTypeHandle &element)
{
    using A = RigExecArray<T>;
    using L = RigExecArrayList<T>;
    RigExecTypeDesc d;
    d.nameSpace = "std";
    d.name = name;
    const uint64_t key[] = {sizeof(L), alignof(L), LayoutKey<A>()};
    d.layoutHash = HashBytes(key, sizeof(key), HashBytes("list", 4, 0));
    d.size = sizeof(L);
    d.align = alignof(L);
    d.element = &element;
    d.construct = &TypeOps<L>::Construct;
    d.copy = &TypeOps<L>::Copy;
    d.move = &TypeOps<L>::Move;
    d.destroy = &TypeOps<L>::Destroy;
    d.equal = [](const void *a, const void *b, rigExec::RigExecEqualMode mode) {
        const L &x = *static_cast<const L *>(a);
        const L &y = *static_cast<const L *>(b);
        if (x.Size() != y.Size())
            return false;
        for (size_t i = 0; i < x.Size(); ++i)
            if (!ValueEqual(x[i], y[i], mode))
                return false;
        return true;
    };
    d.hashClass = rigExec::RigExecHashClass::Exact;
    d.hash = [](const void *value) {
        const L &list = *static_cast<const L *>(value);
        const size_t size = list.Size();
        uint64_t h = HashBytes(&size, sizeof(size), 0);
        for (size_t i = 0; i < size; ++i)
            h = ValueHash(list[i], h);
        return h;
    };
    d.count = [](const void *value) {
        return static_cast<const L *>(value)->Size();
    };
    d.reserve = [](void *value, size_t count) {
        static_cast<L *>(value)->Reserve(count);
    };
    d.elements = [](const void *value) {
        return static_cast<const L *>(value)->Size();
    };
    d.resize = [](void *value, size_t count) {
        static_cast<L *>(value)->Resize(count);
    };
    d.data = [](void *value) -> void * {
        return static_cast<L *>(value)->Data();
    };
    return d;
}

struct Entry {
    const RigExecTypeHandle &handle;
    RigExecTypeDesc desc;
};

const Entry kEntries[] = {
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
    {UInt64Type(), ValueDesc<uint64_t>("UInt64")},
    {IntArrayList(), ListDesc<int32_t>("IntArrayList", IntArray())},
    {Vec3fArrayList(), ListDesc<Vec3f>("Vec3fArrayList", Vec3fArray())},
};

void
Register(RigExecTypeRegistrar &registrar)
{
    for (const Entry &entry : kEntries)
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
