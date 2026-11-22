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

void
Register(rigExec::RigExecTypeRegistrar &registrar)
{
    static const rigExec::RigExecTypeDesc floatDesc = [] {
        rigExec::RigExecTypeDesc d;
        d.nameSpace = "fake";
        d.name = "Float";
        d.size = sizeof(float);
        d.align = alignof(float);
        d.triviallyCopyable = true;
        d.equal = &Equal<float>;
        return d;
    }();
    static const rigExec::RigExecTypeHandle *const vec3Fields[] = {
        &Float(), &Float(), &Float()};
    static const rigExec::RigExecTypeDesc vec3Desc = [] {
        rigExec::RigExecTypeDesc d;
        d.nameSpace = "fake";
        d.name = "Vec3";
        d.size = sizeof(Vec3);
        d.align = alignof(Vec3);
        d.triviallyCopyable = true;
        d.fields = vec3Fields;
        d.fieldCount = 3;
        d.equal = &Equal<Vec3>;
        return d;
    }();
    registrar.Add(Float(), floatDesc);
    registrar.Add(Vec3Type(), vec3Desc);
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
