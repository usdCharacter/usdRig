#ifndef RIGEXEC_STD_TYPES_STD_TYPES_H
#define RIGEXEC_STD_TYPES_STD_TYPES_H

/// The standard type library: scalars, rigExec's vector, matrix and quaternion
/// values (laid out like Gf), and arrays of each. Gf and Vt conversion belongs
/// to the USD adapter.

#include "rigExec/api/registry.h"
#include "rigExecRuntime/runtimeMath.h"
#include "rigExecStdTypes/array.h"

#include <cstdint>

namespace rigExecStd {

using Vec2f = rigExec::RrVec2f;
using Vec2d = rigExec::RrVec2d;
using Vec3f = rigExec::RrVec3f;
using Vec3d = rigExec::RrVec3d;
using Vec4d = rigExec::RrVec4d;
using Vec3i = rigExec::RrVec3i;
using Mat3d = rigExec::RrMat3d;
using Mat4d = rigExec::RrMat4d;
using Quatd = rigExec::RrQuatd;

// Each accessor names one registered type; the C++ type it holds follows.
const rigExec::RigExecTypeHandle &BoolType();   // uint8_t, 0 or 1
const rigExec::RigExecTypeHandle &IntType();    // int32_t
const rigExec::RigExecTypeHandle &UIntType();   // uint32_t
const rigExec::RigExecTypeHandle &FloatType();  // float
const rigExec::RigExecTypeHandle &DoubleType(); // double
const rigExec::RigExecTypeHandle &Vec2fType();
const rigExec::RigExecTypeHandle &Vec2dType();
const rigExec::RigExecTypeHandle &Vec3fType();
const rigExec::RigExecTypeHandle &Vec3dType();
const rigExec::RigExecTypeHandle &Vec4dType();
const rigExec::RigExecTypeHandle &Vec3iType();
const rigExec::RigExecTypeHandle &Mat3dType();
const rigExec::RigExecTypeHandle &Mat4dType();
const rigExec::RigExecTypeHandle &QuatdType();

// RigExecArray<T> of the element above.
const rigExec::RigExecTypeHandle &BoolArray();
const rigExec::RigExecTypeHandle &IntArray();
const rigExec::RigExecTypeHandle &UIntArray();
const rigExec::RigExecTypeHandle &FloatArray();
const rigExec::RigExecTypeHandle &DoubleArray();
const rigExec::RigExecTypeHandle &Vec2fArray();
const rigExec::RigExecTypeHandle &Vec2dArray();
const rigExec::RigExecTypeHandle &Vec3fArray();
const rigExec::RigExecTypeHandle &Vec3dArray();
const rigExec::RigExecTypeHandle &Vec4dArray();
const rigExec::RigExecTypeHandle &Vec3iArray();
const rigExec::RigExecTypeHandle &Mat3dArray();
const rigExec::RigExecTypeHandle &Mat4dArray();
const rigExec::RigExecTypeHandle &QuatdArray();

const rigExec::RigExecTypeHandle &UInt64Type(); // uint64_t

} // namespace rigExecStd

/// The library, for type libraries that compose standard types.
RIGEXEC_TYPE_LIBRARY_ENTRY(rigExecStdTypes);

#endif
