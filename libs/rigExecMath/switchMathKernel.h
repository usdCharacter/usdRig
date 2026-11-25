#ifndef RIGEXEC_MATH_SWITCH_MATH_KERNEL_H
#define RIGEXEC_MATH_SWITCH_MATH_KERNEL_H

/// The space switch's matrix primitives over Gf or its mirror types: the
/// point landmarks of a matrix and the affine map back from them, the
/// GfTransform decomposition, transform blending, the twist and swing
/// rotation filters and the axis masks. The native, runtime and op
/// evaluators instantiate these one definitions.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace rigExec {

/// The landmarks the identity frame maps from.
template <class Vec3>
std::array<Vec3, 4>
RigExecIdentityLandmarksT()
{
    return {Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0),
            Vec3(0.0, 0.0, 1.0)};
}

/// 1e-10 of the rest frame's longest edge, and at least 1e-10.
template <class Vec3>
double
RigExecFrameEpsilonT(const std::array<Vec3, 4> &restPoints)
{
    const double lx = (restPoints[1] - restPoints[0]).GetLength();
    const double ly = (restPoints[2] - restPoints[0]).GetLength();
    const double lz = (restPoints[3] - restPoints[0]).GetLength();
    return 1e-10 * std::max({1.0, lx, ly, lz});
}

/// Row-basis matrix [q1-q0; q2-q0; q3-q0]: with the row-vector convention
/// this is the transpose of the column-basis matrix.
template <class Mat3, class Vec3>
Mat3
RigExecRowBasisT(const std::array<Vec3, 4> &pts)
{
    return Mat3(
        pts[1][0] - pts[0][0], pts[1][1] - pts[0][1], pts[1][2] - pts[0][2],
        pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2],
        pts[3][0] - pts[0][0], pts[3][1] - pts[0][1], pts[3][2] - pts[0][2]);
}

/// The affine map taking \p restPoints to \p posePoints. Identity and false
/// when the rest frame is degenerate. Column convention L = B_P B_Q^-1,
/// t = p0 - L q0; in row-vector storage M3 = MQ^-1 * MP.
template <class Mat3, class Vec3, class Mat4>
bool
RigExecPointsToMatrixT(const std::array<Vec3, 4> &restPoints,
                       const std::array<Vec3, 4> &posePoints, Mat4 *matrix)
{
    const Mat3 mq = RigExecRowBasisT<Mat3>(restPoints);
    const Mat3 mp = RigExecRowBasisT<Mat3>(posePoints);
    const double det = mq.GetDeterminant();
    const double eps = RigExecFrameEpsilonT(restPoints);
    if (std::abs(det) < eps * eps * eps) {
        if (matrix)
            matrix->SetIdentity();
        return false;
    }
    const Mat3 m3 = mq.GetInverse() * mp;
    const Vec3 t = posePoints[0] - restPoints[0] * m3;
    if (matrix) {
        *matrix = Mat4(m3[0][0], m3[0][1], m3[0][2], 0.0, m3[1][0], m3[1][1],
                       m3[1][2], 0.0, m3[2][0], m3[2][1], m3[2][2], 0.0, t[0],
                       t[1], t[2], 1.0);
    }
    return true;
}

/// \p matrix applied to \p restPoints, and whether the posed basis is
/// reflected and whether it is singular at the rest frame's epsilon.
template <class Mat3, class Vec3, class Mat4>
std::array<Vec3, 4>
RigExecMatrixToPointsT(const std::array<Vec3, 4> &restPoints,
                       const Mat4 &matrix, bool *reflected, bool *degenerate)
{
    std::array<Vec3, 4> points;
    for (size_t i = 0; i < 4; ++i)
        points[i] = matrix.TransformAffine(restPoints[i]);
    const double det = RigExecRowBasisT<Mat3>(points).GetDeterminant();
    const double eps = RigExecFrameEpsilonT(restPoints);
    *reflected = det < 0;
    *degenerate = std::abs(det) <= eps * eps * eps;
    return points;
}

/// \p m's identity landmarks. False at the first non-finite one, which is
/// kept; the later ones stay at their identity values.
template <class Mat4, class Vec3>
bool
RigExecMatrixLandmarksT(const Mat4 &m, std::array<Vec3, 4> *points)
{
    const std::array<Vec3, 4> identity = RigExecIdentityLandmarksT<Vec3>();
    *points = identity;
    for (size_t i = 0; i < 4; ++i) {
        (*points)[i] = m.TransformAffine(identity[i]);
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite((*points)[i][a]))
                return false;
    }
    return true;
}

/// Whether a point frame may be read as a transform: flagged valid, not
/// flagged degenerate, and every landmark finite. This is every evaluator's
/// frame check.
template <class Vec3>
bool
RigExecFrameUsableT(const std::array<Vec3, 4> &points, bool valid,
                    bool degenerate)
{
    if (!valid || degenerate)
        return false;
    for (const Vec3 &point : points)
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite(point[a]))
                return false;
    return true;
}

/// How RigExecReadLandmarkMatrixT ended.
enum class RigExecLandmarkRead { Read, NotUsable, Singular };

/// \p m read the way the pose walk reads a frame: its landmarks, refused
/// unless usable (RigExecFrameUsableT), then mapped back from the identity
/// landmarks into \p out.
template <class Mat3, class Vec3, class Mat4>
RigExecLandmarkRead
RigExecReadLandmarkMatrixT(const Mat4 &m, Mat4 *out)
{
    std::array<Vec3, 4> points;
    const bool finite = RigExecMatrixLandmarksT(m, &points);
    if (!RigExecFrameUsableT(points, finite, !finite))
        return RigExecLandmarkRead::NotUsable;
    return RigExecPointsToMatrixT<Mat3>(RigExecIdentityLandmarksT<Vec3>(),
                                        points, out)
               ? RigExecLandmarkRead::Read
               : RigExecLandmarkRead::Singular;
}

/// \p m through its point landmarks and back.
template <class Mat3, class Vec3, class Mat4>
Mat4
RigExecRoundTripT(const Mat4 &m)
{
    Mat4 out;
    out.SetIdentity();
    std::array<Vec3, 4> points;
    RigExecMatrixLandmarksT(m, &points);
    RigExecPointsToMatrixT<Mat3>(RigExecIdentityLandmarksT<Vec3>(), points,
                                 &out);
    return out;
}

namespace switchMathDetail {

template <class Mat4> struct Decomposed {
    using Vec3 = decltype(Mat4().ExtractTranslation());
    using Rotation = decltype(Mat4().ExtractRotation());
    using Quat = decltype(Rotation().GetQuat());
    Vec3 translation = Vec3(0.0, 0.0, 0.0);
    Quat rotation = Quat::GetIdentity();
    Vec3 scale = Vec3(1.0, 1.0, 1.0);
};

/// GfTransform::SetMatrix with a zero pivot: its translation, rotation and
/// scale. The scale orientation is dropped, as Recompose drops it.
template <class Mat4>
Decomposed<Mat4>
Decompose(const Mat4 &m)
{
    using Vec3 = typename Decomposed<Mat4>::Vec3;
    const Vec3 pivot(0.0, 0.0, 0.0);
    Mat4 pivotPosition, pivotInverse;
    pivotPosition.SetTranslate(pivot);
    pivotInverse.SetTranslate(-pivot);
    const Mat4 noPivot = pivotPosition * m * pivotInverse;
    Mat4 shearRotation, rotation, projection;
    Decomposed<Mat4> out;
    noPivot.Factor(&shearRotation, &out.scale, &rotation, &out.translation,
                   &projection);
    out.rotation = rotation.ExtractRotation().GetQuat();
    return out;
}

/// GfTransform::GetMatrix with no pivot and no scale orientation.
template <class Mat4>
Mat4
Recompose(const Decomposed<Mat4> &d)
{
    using Vec3 = typename Decomposed<Mat4>::Vec3;
    using Rotation = typename Decomposed<Mat4>::Rotation;
    const Rotation rotation(d.rotation);
    const bool doScale = d.scale != Vec3(1.0, 1.0, 1.0);
    const bool doRotation = rotation.GetAngle() != 0.0;
    const bool doTranslation = d.translation != Vec3(0.0, 0.0, 0.0);
    bool anySet = false;
    Mat4 mtx;
    if (doScale) {
        mtx.SetScale(d.scale);
        anySet = true;
    }
    if (doRotation) {
        if (anySet) {
            Mat4 tmp;
            tmp.SetRotate(rotation);
            mtx *= tmp;
        } else {
            mtx.SetRotate(rotation);
            anySet = true;
        }
    }
    if (doTranslation) {
        if (anySet) {
            Mat4 tmp;
            tmp.SetTranslate(d.translation);
            mtx *= tmp;
        } else {
            mtx.SetTranslate(d.translation);
            anySet = true;
        }
    }
    if (!anySet)
        mtx.SetIdentity();
    return mtx;
}

} // namespace switchMathDetail

/// \p a blended toward \p b by \p weight: translation and scale lerped,
/// rotation slerped the short way. The endpoints return their operand
/// untouched, so a switch on a whole number is that space exactly.
/// \p Slerp is GfSlerp(alpha, q0, q1) over the quaternion type.
template <class Mat4, class Slerp>
Mat4
RigExecBlendTransformsT(const Mat4 &a, const Mat4 &b, double weight,
                        const Slerp &slerp)
{
    if (!(weight > 0.0))
        return a;
    if (weight >= 1.0)
        return b;
    using D = switchMathDetail::Decomposed<Mat4>;
    using Quat = typename D::Quat;
    const D da = switchMathDetail::Decompose(a),
            db = switchMathDetail::Decompose(b);
    D out;
    out.translation = (1 - weight) * da.translation + weight * db.translation;
    out.scale = (1 - weight) * da.scale + weight * db.scale;
    Quat target = db.rotation;
    const double cosArc = da.rotation.GetImaginary() * target.GetImaginary() +
                          da.rotation.GetReal() * target.GetReal();
    if (cosArc < 0.0)
        target = Quat(-target.GetReal(), -target.GetImaginary());
    out.rotation = slerp(weight, da.rotation, target).GetNormalized();
    return switchMathDetail::Recompose(out);
}

/// RigExecFilterSpaceRotationT's filters; any other value keeps \p m.
enum : int { kRigExecSwitchTwist = 1, kRigExecSwitchSwing = 2 };

/// Keeps only the twist of \p m's rotation about \p axis, or only the
/// swing, leaving translation and scale. The exact swing-twist split
/// (Dobrowolski 2015, docs/references.md); a half turn square to the axis
/// has no twist.
template <class Mat4, class Vec3>
Mat4
RigExecFilterSpaceRotationT(const Mat4 &m, const Vec3 &axis, int filter)
{
    if (filter != kRigExecSwitchTwist && filter != kRigExecSwitchSwing)
        return m;
    const double length = axis.GetLength();
    if (length < 1e-12)
        return m;
    using D = switchMathDetail::Decomposed<Mat4>;
    using Quat = typename D::Quat;
    const Vec3 a = axis / length;
    D d = switchMathDetail::Decompose(m);
    const Quat q = d.rotation.GetNormalized();
    const double along = q.GetImaginary() * a;
    Quat twist(q.GetReal(), a * along);
    const double norm = std::sqrt(twist.GetReal() * twist.GetReal() +
                                  twist.GetImaginary().GetLengthSq());
    if (norm < 1e-12)
        twist = Quat::GetIdentity();
    else
        twist = Quat(twist.GetReal() / norm, twist.GetImaginary() / norm);
    d.rotation = filter == kRigExecSwitchTwist
                     ? twist
                     : (q * twist.GetInverse()).GetNormalized();
    return switchMathDetail::Recompose(d);
}

/// \p m with the unaffected axes of each channel reset. Rotation axes are
/// masked as an XYZ Euler statement, taken apart in the order the avar
/// compose puts one together (row-vector R = Rx * Ry * Rz); at gimbal lock
/// all of it goes to X.
template <class Mat4>
Mat4
RigExecMaskTransformT(const Mat4 &m, const bool translation[3],
                      const bool rotation[3], const bool scale[3])
{
    const bool all = translation[0] && translation[1] && translation[2] &&
                     rotation[0] && rotation[1] && rotation[2] && scale[0] &&
                     scale[1] && scale[2];
    if (all)
        return m;
    using D = switchMathDetail::Decomposed<Mat4>;
    using Vec3 = typename D::Vec3;
    using Rotation = typename D::Rotation;
    D d = switchMathDetail::Decompose(m);
    for (int axis = 0; axis < 3; ++axis) {
        if (!translation[axis])
            d.translation[axis] = 0.0;
        if (!scale[axis])
            d.scale[axis] = 1.0;
    }
    if (!rotation[0] || !rotation[1] || !rotation[2]) {
        const Mat4 r(Rotation(d.rotation), Vec3(0.0, 0.0, 0.0));
        double x = 0.0, y = 0.0, z = 0.0;
        const double r02 = r[0][2];
        const double sinY = r02 < -1.0 ? -1.0 : (r02 > 1.0 ? 1.0 : r02);
        y = std::asin(sinY);
        if (std::abs(sinY) < 1.0 - 1e-9) {
            x = std::atan2(r[1][2], r[2][2]);
            z = std::atan2(r[0][1], r[0][0]);
        } else {
            x = std::atan2(-r[2][1], r[1][1]);
            z = 0.0;
        }
        const Vec3 axes[3] = {Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0),
                              Vec3(0.0, 0.0, 1.0)};
        constexpr double kDegrees = 180.0 / 3.14159265358979323846;
        const double angles[3] = {x * kDegrees, y * kDegrees, z * kDegrees};
        Mat4 composed(1.0);
        for (int axis = 0; axis < 3; ++axis) {
            if (!rotation[axis] || angles[axis] == 0.0)
                continue;
            composed = composed * Mat4(Rotation(axes[axis], angles[axis]),
                                       Vec3(0.0, 0.0, 0.0));
        }
        d.rotation = composed.ExtractRotationQuat();
    }
    return switchMathDetail::Recompose(d);
}

/// RigExecRunSpaceSwitchArithmetic's matrix primitives over one set of
/// types. \p Slerp is a functor called as GfSlerp(alpha, q0, q1).
template <class Mat3, class Vec3, class Mat4, class Slerp>
struct RigExecSpaceSwitchMathT {
    using Matrix = Mat4;
    static Matrix RoundTrip(const Matrix &m)
    {
        return RigExecRoundTripT<Mat3, Vec3>(m);
    }
    static Matrix Filter(const Matrix &m, const Vec3 &axis, int filter)
    {
        return RigExecFilterSpaceRotationT(m, axis, filter);
    }
    static Matrix Blend(const Matrix &a, const Matrix &b, double weight)
    {
        return RigExecBlendTransformsT(a, b, weight, Slerp());
    }
    static Matrix Mask(const Matrix &m, const bool *translation,
                       const bool *rotation, const bool *scale)
    {
        return RigExecMaskTransformT(m, translation, rotation, scale);
    }
};

} // namespace rigExec

#endif // RIGEXEC_MATH_SWITCH_MATH_KERNEL_H
