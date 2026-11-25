// The runtime's dual-quaternion math policy (dualQuatKernel.h) over
// runtimeMath.h's Gf mirror. Header-only and store-free, so the ops can skin
// with it.
#ifndef RIGEXEC_MATH_DUAL_QUAT_MATH_H
#define RIGEXEC_MATH_DUAL_QUAT_MATH_H

#include "rigExecMath/dualQuatKernel.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecMath/runtimeMath.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace rigExec {

/// RigExecFrameEpsilon: 1e-10 of the rest frame's longest edge, at least 1.
inline double
RrDualQuatFrameEpsilon(const std::array<RrVec3d, 4> &restPoints)
{
    return RigExecFrameEpsilonT(restPoints);
}

inline RrMat3d
RrDualQuatRowBasis(const std::array<RrVec3d, 4> &pts)
{
    return RigExecRowBasisT<RrMat3d>(pts);
}

/// RigExecPointsToMatrix: the affine map taking rest to pose; identity and
/// false when the rest frame is degenerate.
inline bool
RrDualQuatPointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                         const std::array<RrVec3d, 4> &posePoints,
                         RrMat4d *matrix)
{
    return RigExecPointsToMatrixT<RrMat3d>(restPoints, posePoints, matrix);
}

inline std::array<RrVec3d, 4>
RrDualQuatMatrixToPoints(const std::array<RrVec3d, 4> &restPoints,
                         const RrMat4d &matrix)
{
    std::array<RrVec3d, 4> points;
    for (size_t i = 0; i < 4; ++i)
        points[i] = matrix.TransformAffine(restPoints[i]);
    return points;
}

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

/// GfMatrix3d::ExtractRotationQuaternion (matrix3d.cpp, transcribed including
/// the int-typed 4 in the else arm), followed by the GfRotation round trip
/// ExtractRotation().GetQuat() performs: SetQuat's length gate and axis/angle
/// extraction, then GetQuat's half-angle sine and cosine with one
/// normalization (rotation.cpp).
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

/// GfDot(const GfQuatd&, const GfQuatd&) from quatd.h: the imaginary dot
/// first, then the real product.
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
        return RrDualQuatPointsToMatrix(restPoints, posePoints, matrix);
    }

    static std::array<RrVec3d, 4>
    MatrixToPoints(const std::array<RrVec3d, 4> &restPoints,
                   const RrMat4d &matrix)
    {
        return RrDualQuatMatrixToPoints(restPoints, matrix);
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

    static RrQuatd Slerp(double alpha, const RrQuatd &a, const RrQuatd &b)
    {
        return RrSlerp(alpha, a, b);
    }
};

/// GfRotation::RotateOntoProjected: the twist about \p axisParam taking
/// v1's projection onto v2's.
inline RrRotation
RrRotateOntoProjected(const RrVec3d &v1, const RrVec3d &v2,
                      const RrVec3d &axisParam)
{
    RrVec3d axis = axisParam.GetNormalized();

    RrVec3d v1Proj = v1 - RrDot(v1, axis) * axis;
    RrVec3d v2Proj = v2 - RrDot(v2, axis) * axis;
    v1Proj.Normalize();
    v2Proj.Normalize();
    RrVec3d crossAxis = RrCross(v1Proj, v2Proj);
    double sinTheta = RrDot(crossAxis, axis);
    double cosTheta = RrDot(v1Proj, v2Proj);
    double theta = 0;
    if (!(std::fabs(sinTheta) < 1e-6 && std::fabs(cosTheta) < 1e-6)) {
        theta = std::atan2(sinTheta, cosTheta);
    }

    const double toDeg = (180.0) / std::acos(-1.0);
    return RrRotation(axis, theta * toDeg);
}

template <class Point, class Double> struct RigExecConstraintMath;

/// The mirror's instantiation of the shared constraint kernel
/// (constraintKernel.h).
template <> struct RigExecConstraintMath<RrVec3f, RrVec3d> {
    using Rotation = RrRotation;

    static RrRotation RotateOntoProjected(const RrVec3d &v1, const RrVec3d &v2,
                                          const RrVec3d &axis)
    {
        return RrRotateOntoProjected(v1, v2, axis);
    }

    static void SetRotate(RrMat4d *m, const RrQuatd &q)
    {
        m->SetRotateOnly(q);
    }
};

} // namespace rigExec

#endif
