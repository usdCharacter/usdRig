#ifndef RIGEXEC_TESTS_REGISTRY_FIXTURE_FAKE_TYPES_H
#define RIGEXEC_TESTS_REGISTRY_FIXTURE_FAKE_TYPES_H

#include "rigExec/api/registry.h"

namespace fakeTypes {

struct Vec3 {
    float x, y, z;
};

const rigExec::RigExecTypeHandle &Float();
const rigExec::RigExecTypeHandle &Vec3Type();
/// Registered by no library: a port naming it must fail freeze.
const rigExec::RigExecTypeHandle &Unregistered();

} // namespace fakeTypes

#endif
