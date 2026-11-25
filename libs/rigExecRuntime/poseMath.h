// The pose.h Math over Rr types, for any frame type holding `points` (four
// RrVec3d) and `flags`: the runtime's RrPointFrame and the ops'
// rigExecRig::PointFrame run the same arithmetic.
#ifndef RIGEXEC_RUNTIME_POSE_MATH_H
#define RIGEXEC_RUNTIME_POSE_MATH_H

#include "rigExecRuntime/runtimeMath.h"

#include <array>

namespace rigExec {

template <class FrameT> struct RrPoseMathT {
    using Mat4 = RrMat4d;
    using Mat3 = RrMat3d;
    using Vec3 = RrVec3d;
    using Quat = RrQuatd;
    using Frame = FrameT;
    static Mat4 Identity4()
    {
        Mat4 m;
        m.SetIdentity();
        return m;
    }
    static Mat3 Identity3()
    {
        Mat3 m;
        m.SetIdentity();
        return m;
    }
    static Mat4 Mat4FromElements(double m00, double m01, double m02, double m03,
                                 double m10, double m11, double m12, double m13,
                                 double m20, double m21, double m22, double m23,
                                 double m30, double m31, double m32, double m33)
    {
        return Mat4(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23,
                    m30, m31, m32, m33);
    }
    static Mat3 Mat3FromElements(double m00, double m01, double m02, double m10,
                                 double m11, double m12, double m20, double m21,
                                 double m22)
    {
        return Mat3(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }
    static Vec3 Vec3Of(double x, double y, double z)
    {
        return Vec3(x, y, z);
    }
    static void SinCos(double v, double *s, double *c)
    {
        RrSinCos(v, s, c);
    }
    static Quat QuatFromComponents(double w, double x, double y, double z)
    {
        return Quat(w, Vec3(x, y, z));
    }
    static void QuatToComponents(const Quat &q, double *w, Vec3 *im)
    {
        *w = q.GetReal();
        *im = q.GetImaginary();
    }
    static Frame MakeFrame(const std::array<Vec3, 4> &points, unsigned flags)
    {
        Frame frame;
        frame.points = points;
        frame.flags = flags;
        return frame;
    }
};

} // namespace rigExec

#endif // RIGEXEC_RUNTIME_POSE_MATH_H
