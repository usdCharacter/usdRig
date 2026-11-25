// The runtime's math policies for the shared kernels in rigExecMath: the Gf
// operations the affine frame expressions (affineFrameKernel.h) and the
// dual-quaternion skinning kernels (dualQuatKernel.h) use, spelled through
// runtimeMath.h's bit-identical mirror.
#ifndef RIGEXEC_MATH_AFFINE_MATH_H
#define RIGEXEC_MATH_AFFINE_MATH_H

#include "rigExecMath/affineFrameKernel.h"
#include "rigExecMath/dualQuatKernel.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecMath/runtimeMath.h"

#include <array>
#include <cmath>
#include <vector>

namespace rigExec {

/// The runtime instantiation of the shared dual-quaternion kernel: the
/// dual quaternion the affine frame expressions blend and the skin path
/// keeps in its palette.
using RrAffineDualQuat = RigExecDualQuatKernelT<RrVec3f, RrVec3d>;

struct RrAffineMath {
    using Mat4 = RrMat4d;
    using Mat3 = RrMat3d;
    using Vec3d = RrVec3d;
    using Vec3f = RrVec3f;
    using Vec3i = RrVec3i;
    using Quat = RrQuatd;
    using DualQuat = RrAffineDualQuat;
    static double Dot(const RrVec3d &a, const RrVec3d &b)
    {
        return RrDot(a, b);
    }
    static float Dot(const RrVec3f &a, const RrVec3f &b)
    {
        return RrDot(a, b);
    }
    /// GfDot(GfQuatd, GfQuatd).
    static double Dot(const RrQuatd &a, const RrQuatd &b)
    {
        return RrDot(a.GetImaginary(), b.GetImaginary()) +
               a.GetReal() * b.GetReal();
    }
    static RrVec3d Cross(const RrVec3d &a, const RrVec3d &b)
    {
        return RrCross(a, b);
    }
    /// GfCompMult(GfVec3d, GfVec3d).
    static RrVec3d CompMult(const RrVec3d &a, const RrVec3d &b)
    {
        return RrVec3d(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
    }
    /// GfVec3f(const GfVec3d &) and GfVec3d(const GfVec3f &).
    static RrVec3f ToFloat(const RrVec3d &v)
    {
        return RrVec3f(float(v[0]), float(v[1]), float(v[2]));
    }
    static RrVec3d ToDouble(const RrVec3f &v)
    {
        return RrVec3d(double(v[0]), double(v[1]), double(v[2]));
    }
    /// GfMatrix4d(GfRotation(axis, degrees), GfVec3d(0)).
    static RrMat4d Rotation(const RrVec3d &axis, double degrees)
    {
        return RrMat4d(RrRotation(axis, degrees), RrVec3d(0.0));
    }
    /// GfMatrix3d::ExtractRotation().GetQuat(): the 3x3 matrix's rotation
    /// quaternion is GfMatrix4d's with a unit homogeneous corner.
    static RrQuatd RotationQuat(const RrMat3d &m)
    {
        const RrMat4d embedded(m[0][0], m[0][1], m[0][2], 0.0, m[1][0], m[1][1],
                               m[1][2], 0.0, m[2][0], m[2][1], m[2][2], 0.0,
                               0.0, 0.0, 0.0, 1.0);
        return embedded.ExtractRotation().GetQuat();
    }
    /// GfMatrix3d(const GfQuatd &): the same rotation-from-quaternion rows.
    static RrMat3d FromQuat(const RrQuatd &q)
    {
        RrMat4d rotation(1.0);
        rotation._SetRotateFromQuat(q.GetReal(), q.GetImaginary());
        return rotation.ExtractRotationMatrix();
    }
    static RrAffineDualQuat DualQuatZero()
    {
        return RrAffineDualQuat(RrQuatd(0.0), RrQuatd(0.0));
    }
    static RrAffineDualQuat DualQuatFromMatrix(const RrMat4d &m)
    {
        return RigExecDualQuatFromMatrixKernel<RrVec3f, RrVec3d>(m);
    }
    static bool DualQuatNormalize(RrAffineDualQuat *dq)
    {
        return RigExecDualQuatNormalizeKernel(dq);
    }
    static RrMat4d DualQuatToMatrix(const RrAffineDualQuat &dq)
    {
        return RigExecDualQuatToMatrixKernel(dq);
    }
};

using RrAffineFrameInputs = RigExecAffineFrameInputsT<RrAffineMath>;
using RrAffineFrameKernel = RigExecAffineFrameKernel<RrAffineMath>;

} // namespace rigExec

#endif
