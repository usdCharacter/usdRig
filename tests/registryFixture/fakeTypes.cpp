#include "fakeTypes.h"

#include <cstring>

namespace fakeTypes {

RIGEXEC_DEFINE_TYPE_HANDLE(Float)
RIGEXEC_DEFINE_TYPE_HANDLE(Vec3Type)
RIGEXEC_DEFINE_TYPE_HANDLE(Unregistered)

namespace {

template <class T>
bool
Equal(const void *a, const void *b, rigExec::RigExecEqualMode)
{
    return std::memcmp(a, b, sizeof(T)) == 0;
}

const rigExec::RigExecTypeDesc kFloatDesc = [] {
    rigExec::RigExecTypeDesc d;
    d.nameSpace = "fake";
    d.name = "Float";
    d.size = sizeof(float);
    d.align = alignof(float);
    d.triviallyCopyable = true;
    d.equal = &Equal<float>;
    return d;
}();

const rigExec::RigExecTypeHandle *const kVec3Fields[] = {&Float(), &Float(),
                                                         &Float()};

const rigExec::RigExecTypeDesc kVec3Desc = [] {
    rigExec::RigExecTypeDesc d;
    d.nameSpace = "fake";
    d.name = "Vec3";
    d.size = sizeof(Vec3);
    d.align = alignof(Vec3);
    d.triviallyCopyable = true;
    d.fields = kVec3Fields;
    d.fieldCount = 3;
    d.equal = &Equal<Vec3>;
    return d;
}();

void
Register(rigExec::RigExecTypeRegistrar &registrar)
{
    registrar.Add(Float(), kFloatDesc);
    registrar.Add(Vec3Type(), kVec3Desc);
}

const rigExec::RigExecTypeLibraryDesc kLibrary = [] {
    rigExec::RigExecTypeLibraryDesc d;
    d.name = "fakeTypes";
    d.registerTypes = &Register;
    return d;
}();

} // namespace
} // namespace fakeTypes

RIGEXEC_TYPE_LIBRARY_ENTRY(rigExecTestFakeTypes)
{
    return &fakeTypes::kLibrary;
}
