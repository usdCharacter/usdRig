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
