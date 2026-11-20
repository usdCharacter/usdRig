// The runtime's math policies for the shared kernels in rigExecMath: the Gf
// operations the affine frame expressions (affineFrameKernel.h) and the
// dual-quaternion skinning kernels (dualQuatKernel.h) use, spelled through
// runtimeMath.h's bit-identical mirror. USD-free.
#ifndef RIGEXEC_RUNTIME_AFFINE_MATH_H
#define RIGEXEC_RUNTIME_AFFINE_MATH_H

#include "rigExecMath/affineFrameKernel.h"
#include "rigExecMath/dualQuatKernel.h"
#include "rigExecRuntime/runtimeMath.h"
#include "rigExecRuntime/store.h"

#include <array>
#include <cmath>
#include <vector>

namespace rigExec {

/// The runtime instantiation of the shared dual-quaternion kernel: the
/// dual quaternion the affine frame expressions blend and the skin path
/// keeps in its palette.
using RrAffineDualQuat = RigExecDualQuatKernelT<RrVec3f, RrVec3d>;

/// GfMatrix3d::SetRotate(const GfQuatd&): the same _SetRotateFromQuat the
/// 4x4 uses (verified equal on the USD build over random quaternions).
inline void
RrMat3SetRotate(RrMat3d *m, const RrQuatd &q)
{
    const double r = q.GetReal();
    const RrVec3d i = q.GetImaginary();
    (*m)[0][0] = 1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2]);
    (*m)[0][1] = 2.0 * (i[0] * i[1] + i[2] * r);
    (*m)[0][2] = 2.0 * (i[2] * i[0] - i[1] * r);
    (*m)[1][0] = 2.0 * (i[0] * i[1] - i[2] * r);
    (*m)[1][1] = 1.0 - 2.0 * (i[2] * i[2] + i[0] * i[0]);
    (*m)[1][2] = 2.0 * (i[1] * i[2] + i[0] * r);
    (*m)[2][0] = 2.0 * (i[2] * i[0] + i[1] * r);
    (*m)[2][1] = 2.0 * (i[1] * i[2] - i[0] * r);
    (*m)[2][2] = 1.0 - 2.0 * (i[1] * i[1] + i[0] * i[0]);
}

/// GfMatrix3d::Orthonormalize: orthogonalize and normalize the row vectors
/// (matrix3d.cpp), without the convergence warning, which the frame path
/// never reads.
inline void
RrMat3Orthonormalize(RrMat3d *m)
{
    RrVec3d r0(m->GetRow(0)), r1(m->GetRow(1)), r2(m->GetRow(2));
    RrOrthogonalizeBasis(&r0, &r1, &r2, true);
    m->SetRow(0, r0);
    m->SetRow(1, r1);
    m->SetRow(2, r2);
}

/// GfMatrix3d::ExtractRotationQuaternion (matrix3d.cpp, transcribed verbatim
/// including the int-typed 4 in the else arm), followed by the GfRotation
/// round trip ExtractRotation().GetQuat() performs: SetQuat's length gate
/// and axis/angle extraction, then GetQuat's half-angle sine and cosine with
/// one normalization (rotation.cpp).
inline RrQuatd
RrMat3ExtractRotationQuaternion(const RrMat3d &m)
{
    int i;
    if (m[0][0] > m[1][1]) {
        i = (m[0][0] > m[2][2] ? 0 : 2);
    } else {
        i = (m[1][1] > m[2][2] ? 1 : 2);
    }
    RrVec3d im(0.0, 0.0, 0.0);
    double r = 0.0;
    if (m[0][0] + m[1][1] + m[2][2] > m[i][i]) {
        r = 0.5 * std::sqrt(m[0][0] + m[1][1] + m[2][2] + 1);
        im[0] = (m[1][2] - m[2][1]) / (4.0 * r);
        im[1] = (m[2][0] - m[0][2]) / (4.0 * r);
        im[2] = (m[0][1] - m[1][0]) / (4.0 * r);
    } else {
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const double q = 0.5 * std::sqrt(m[i][i] - m[j][j] - m[k][k] + 1);
        im[i] = q;
        im[j] = (m[i][j] + m[j][i]) / (4 * q);
        im[k] = (m[k][i] + m[i][k]) / (4 * q);
        r = (m[j][k] - m[k][j]) / (4 * q);
    }
    r = RrClamp(r, -1.0, 1.0);
    RrRotation rotation;
    rotation.SetQuat(RrQuatd(r, im));
    return rotation.GetQuat();
}

/// GfDot(const GfQuatd&, const GfQuatd&), verbatim from quatd.h: the
/// imaginary dot first, then the real product.
inline double
RrQuatDot(const RrQuatd &a, const RrQuatd &b)
{
    return RrDot(a.GetImaginary(), b.GetImaginary()) +
           a.GetReal() * b.GetReal();
}

template <> struct RigExecDualQuatMath<RrVec3f, RrVec3d> {
    using Mat4 = RrMat4d;
    using Mat3 = RrMat3d;
    using Quat = RrQuatd;

    static bool PointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                               const std::array<RrVec3d, 4> &posePoints,
                               RrMat4d *matrix)
    {
        return RrPointsToMatrix(restPoints, posePoints, matrix);
    }

    static std::array<RrVec3d, 4>
    MatrixToPoints(const std::array<RrVec3d, 4> &restPoints,
                   const RrMat4d &matrix)
    {
        return RrMatrixToPoints(restPoints, matrix).points;
    }

    static double Dot(const RrVec3d &a, const RrVec3d &b)
    {
        return RrDot(a, b);
    }

    static RrVec3d Cross(const RrVec3d &a, const RrVec3d &b)
    {
        return RrCross(a, b);
    }

    static double QuatDot(const RrQuatd &a, const RrQuatd &b)
    {
        return RrQuatDot(a, b);
    }

    static void SetRotate(RrMat3d *m, const RrQuatd &q)
    {
        RrMat3SetRotate(m, q);
    }

    static void Orthonormalize(RrMat3d *m)
    {
        RrMat3Orthonormalize(m);
    }

    static RrQuatd ExtractRotationQuaternion(const RrMat3d &m)
    {
        return RrMat3ExtractRotationQuaternion(m);
    }
};

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
