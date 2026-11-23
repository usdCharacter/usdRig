// The spline-IK spine: the four-CV quadratic B-spline with its arc-length
// table, the rest description, the CV posing and the solve, shared by the
// native (Gf) path, the runtime (Rr) and the solver ops. Header-only.
//
// Host types come from RigExecDualQuatMath<Point, Double>. Frames, rest,
// controls, parameters and result are duck-typed after the
// RigExecSplineIk* structs in splineIk.h.
#ifndef RIGEXEC_MATH_SPLINE_IK_KERNEL_H
#define RIGEXEC_MATH_SPLINE_IK_KERNEL_H

#include "chainSolverKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rigExec {

/// Below this a length, a handle or a curve is collapsed.
constexpr double kRigExecSplineIkEpsilon = 1e-10;

namespace splineIkDetail {

constexpr uint32_t kFrameAffine = 1u << 3;

// Sub-intervals per Bezier span. The speed of a quadratic Bezier is the
// square root of a quadratic in u, smooth unless the curve has a cusp, so
// 8-point Gauss-Legendre on 1/32 of a span integrates it to near machine
// precision.
constexpr int kSubdivisionsPerSpan = 32;
constexpr int kSpans = 2;

constexpr double kGaussNodes[8] = {
    -0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
    -0.1834346424956498, 0.1834346424956498,  0.5255324099163290,
    0.7966664774136267,  0.9602898564975363,
};
constexpr double kGaussWeights[8] = {
    0.1012285362903763, 0.2223810344533745, 0.3137066458778873,
    0.3626837833783620, 0.3626837833783620, 0.3137066458778873,
    0.2223810344533745, 0.1012285362903763,
};

template <class Double>
bool
IsFinite(const Double &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

template <class Double, class F>
bool
IsFiniteFrame(const F &f)
{
    for (const Double &p : f.points)
        if (!IsFinite(p))
            return false;
    return true;
}

/// v by the minimal rotation taking unit \p from to unit \p to; v unchanged
/// for antiparallel input (only ever used perpendicular to from).
template <class Point, class Double>
Double
RotateToward(const Double &from, const Double &to, const Double &v)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    const Double axis = Math::Cross(from, to);
    const double s = axis.GetLength();
    const double c = Math::Dot(from, to);
    if (s <= kRigExecSplineIkEpsilon)
        return v;
    const Double k = axis / s;
    return v * c + Math::Cross(k, v) * s + k * (Math::Dot(k, v) * (1.0 - c));
}

/// The world axis least parallel to \p x, made perpendicular and unit.
template <class Point, class Double>
Double
FallbackUp(const Double &x)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    int least = 0;
    double best = std::abs(x[0]);
    for (int i = 1; i < 3; ++i) {
        if (std::abs(x[i]) < best) {
            best = std::abs(x[i]);
            least = i;
        }
    }
    Double up(0.0);
    up[least] = 1.0;
    up -= x * Math::Dot(x, up);
    return up.GetNormalized();
}

template <class F>
F
DegenerateCopy(const F &frame)
{
    F copy = frame;
    copy.flags |= chainSolverDetail::kFrameDegenerate;
    return copy;
}

/// A frame's orthonormal basis with its handle lengths and handedness; ok
/// is false for a collapsed or parallel X and Y.
template <class Double> struct Basis {
    Double x{1.0, 0.0, 0.0};
    Double y{0.0, 1.0, 0.0};
    double lx = 1.0, ly = 1.0, lz = 1.0;
    double handedness = 1.0;
    bool ok = false;
};

template <class Point, class Double, class F>
Basis<Double>
MakeBasis(const F &frame)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    Basis<Double> b;
    const Double origin = frame.points[0];
    const Double ax = frame.points[1] - origin;
    const Double ay = frame.points[2] - origin;
    const Double az = frame.points[3] - origin;
    b.lx = ax.GetLength();
    b.ly = ay.GetLength();
    b.lz = az.GetLength();
    if (!IsFiniteFrame<Double>(frame) || b.lx <= kRigExecSplineIkEpsilon ||
        b.ly <= kRigExecSplineIkEpsilon)
        return b;
    b.x = ax / b.lx;
    const Double up = ay - b.x * Math::Dot(b.x, ay);
    const double upLength = up.GetLength();
    if (upLength <= kRigExecSplineIkEpsilon * std::max(1.0, b.ly))
        return b;
    b.y = up / upLength;
    b.handedness = (Math::Dot(Math::Cross(b.x, b.y), az) < 0.0) ? -1.0 : 1.0;
    // A collapsed Z handle is rebuilt at unit length.
    if (b.lz <= kRigExecSplineIkEpsilon)
        b.lz = 1.0;
    b.ok = true;
    return b;
}

} // namespace splineIkDetail

/// The open degree-2, four-CV B-spline (two quadratic Bezier spans through
/// the midpoint of cv1 and cv2) over u in [0, 2], with an arc-length table.
template <class Double> class RigExecSplineIkCurveKernel {
public:
    RigExecSplineIkCurveKernel()
        : RigExecSplineIkCurveKernel(std::array<Double, 4>{
              Double(0.0), Double(0.0), Double(0.0), Double(0.0)})
    {
    }

    explicit RigExecSplineIkCurveKernel(const std::array<Double, 4> &cvs)
        : _cvs(cvs)
    {
        using namespace splineIkDetail;
        const int cells = kSpans * kSubdivisionsPerSpan;
        _u.resize(size_t(cells) + 1);
        _cumulative.resize(size_t(cells) + 1);
        _u[0] = 0.0;
        _cumulative[0] = 0.0;
        for (int i = 0; i < cells; ++i) {
            const double a = double(i) / kSubdivisionsPerSpan;
            const double b = double(i + 1) / kSubdivisionsPerSpan;
            _u[size_t(i) + 1] = b;
            _cumulative[size_t(i) + 1] =
                _cumulative[size_t(i)] + _Quadrature(a, b);
        }
        _length = _cumulative[size_t(cells)];
        if (!std::isfinite(_length))
            _length = 0.0;
    }

    const std::array<Double, 4> &Cvs() const
    {
        return _cvs;
    }

    double ArcLength() const
    {
        return _length;
    }

    /// Shorter than kRigExecSplineIkEpsilon: no usable tangent anywhere.
    bool IsDegenerate() const
    {
        return _length <= kRigExecSplineIkEpsilon;
    }

    /// Position and dC/du at u, clamped to [0, 2]. Either may be null.
    void Evaluate(double u, Double *position, Double *derivative) const
    {
        u = std::clamp(u, 0.0, double(splineIkDetail::kSpans));
        const int span = (u < 1.0) ? 0 : 1;
        const double t = u - span;
        const Double mid = (_cvs[1] + _cvs[2]) * 0.5;
        const Double q0 = span == 0 ? _cvs[0] : mid;
        const Double q1 = span == 0 ? _cvs[1] : _cvs[2];
        const Double q2 = span == 0 ? mid : _cvs[3];
        const double s = 1.0 - t;
        if (position)
            *position = q0 * (s * s) + q1 * (2.0 * s * t) + q2 * (t * t);
        // dB/dt = 2[(1-t)(q1-q0) + t(q2-q1)], and du = dt within a span.
        if (derivative)
            *derivative = ((q1 - q0) * s + (q2 - q1) * t) * 2.0;
    }

    /// u at arc distance \p distance, clamped to [0, ArcLength()];
    /// accurate to about 1e-13 of the arc length.
    double ParamAtArcLength(double distance) const
    {
        if (IsDegenerate())
            return 0.0;
        distance = std::clamp(distance, 0.0, _length);
        const auto it =
            std::lower_bound(_cumulative.begin(), _cumulative.end(), distance);
        size_t hi = size_t(it - _cumulative.begin());
        if (hi == 0)
            return _u[0];
        if (hi >= _cumulative.size())
            hi = _cumulative.size() - 1;
        const size_t lo = hi - 1;
        const double a = _u[lo], b = _u[hi];
        const double cellLength = _cumulative[hi] - _cumulative[lo];
        // A collapsed cell: any parameter in it is the same point.
        if (cellLength <= kRigExecSplineIkEpsilon * std::max(1.0, _length))
            return a;
        const double target = distance - _cumulative[lo];

        // Safeguarded Newton; [bl, bh] always brackets the root.
        double bl = a, bh = b;
        double u = a + (b - a) * (target / cellLength);
        const double tolerance = 1e-15 * std::max(1.0, _length);
        for (int iteration = 0; iteration < 32; ++iteration) {
            const double g = _Quadrature(a, u) - target;
            if (std::abs(g) <= tolerance)
                break;
            if (g > 0.0)
                bh = u;
            else
                bl = u;
            Double d(0.0);
            Evaluate(u, nullptr, &d);
            const double speed = d.GetLength();
            double next = (speed > kRigExecSplineIkEpsilon) ? u - g / speed
                                                            : 0.5 * (bl + bh);
            if (!(next > bl && next < bh))
                next = 0.5 * (bl + bh);
            if (std::abs(next - u) <= 1e-16 * std::max(1.0, u)) {
                u = next;
                break;
            }
            u = next;
        }
        return u;
    }

    /// Position and unit tangent at arc distance \p distance, extrapolated
    /// straight past either end. False (start CV, zero tangent) for a
    /// degenerate curve; a vanishing derivative falls back to the chord to
    /// the nearest distinct point.
    bool PointAtArcLength(double distance, Double *position,
                          Double *tangent) const
    {
        using namespace splineIkDetail;
        if (IsDegenerate()) {
            if (position)
                *position = _cvs[0];
            if (tangent)
                *tangent = Double(0.0);
            return false;
        }
        const double clamped = std::clamp(distance, 0.0, _length);
        const double u = ParamAtArcLength(clamped);
        Double p(0.0), dir(0.0);
        Evaluate(u, &p, &dir);
        double speed = dir.GetLength();
        if (speed <= kRigExecSplineIkEpsilon) {
            const double step = 1.0 / kSubdivisionsPerSpan;
            for (int i = 1; i <= kSpans * kSubdivisionsPerSpan &&
                            speed <= kRigExecSplineIkEpsilon;
                 ++i) {
                Double ahead(0.0), behind(0.0);
                Evaluate(u + i * step, &ahead, nullptr);
                Evaluate(u - i * step, &behind, nullptr);
                dir = ahead - p;
                speed = dir.GetLength();
                if (speed <= kRigExecSplineIkEpsilon) {
                    dir = p - behind;
                    speed = dir.GetLength();
                }
            }
        }
        if (speed <= kRigExecSplineIkEpsilon) {
            if (position)
                *position = p;
            if (tangent)
                *tangent = Double(0.0);
            return false;
        }
        dir /= speed;
        if (distance != clamped)
            p += dir * (distance - clamped);
        if (position)
            *position = p;
        if (tangent)
            *tangent = dir;
        return true;
    }

private:
    // Arc length over [a, b] by 8-point Gauss-Legendre.
    double _Quadrature(double a, double b) const
    {
        using namespace splineIkDetail;
        const double half = 0.5 * (b - a);
        const double mid = 0.5 * (a + b);
        double sum = 0.0;
        for (int g = 0; g < 8; ++g) {
            Double d(0.0);
            Evaluate(mid + half * kGaussNodes[g], nullptr, &d);
            sum += kGaussWeights[g] * d.GetLength();
        }
        return sum * half;
    }

    std::array<Double, 4> _cvs;
    std::vector<double> _u;          ///< table parameters (cell edges)
    std::vector<double> _cumulative; ///< arc length at each table parameter
    double _length = 0.0;
};

/// The rest description into \p rest: cvs from joint origins [0], [1],
/// [N-2], [N-1], segment lengths, and the reference length (the chain's
/// when \p chainLength, the curve's otherwise). Fewer than two joints use
/// the available origins for every CV.
template <class Double, class F, class Rest>
void
RigExecSplineIkMakeRestKernel(const std::vector<F> &joints,
                              const F &rootControl, const F &midControl,
                              const F &endControl,
                              const std::vector<double> &volumeWeights,
                              bool chainLength, Rest *rest)
{
    rest->rootControl = rootControl;
    rest->midControl = midControl;
    rest->endControl = endControl;
    rest->joints = joints;
    rest->volumeWeights = volumeWeights;
    const size_t n = joints.size();
    if (n == 0)
        return;
    const auto origin = [&](size_t i) {
        return joints[std::min(i, n - 1)].points[0];
    };
    rest->cvs = {origin(0), origin(1), origin(n >= 2 ? n - 2 : 0),
                 origin(n - 1)};
    double chain = 0.0;
    rest->segmentLengths.resize(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        rest->segmentLengths[i] = (origin(i + 1) - origin(i)).GetLength();
        chain += rest->segmentLengths[i];
    }
    rest->restArcLength =
        chainLength ? chain
                    : RigExecSplineIkCurveKernel<Double>(rest->cvs).ArcLength();
}

/// The rest CVs carried by the controls: cv0 and cv1 ride the root, cv2
/// and cv3 the end; then the length floor, the root tangent aim and the
/// mid control's offset from its follow point. False (rest CVs) for a
/// singular rest control or a non-finite result.
template <class Point, class Double, class Rest, class Controls, class Params>
bool
RigExecSplineIkPoseCvsKernel(const Rest &rest, const Controls &controls,
                             const Params &params, std::array<Double, 4> *cvs)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    using namespace splineIkDetail;
    *cvs = rest.cvs;
    typename Math::Mat4 rootMap(1.0), endMap(1.0);
    if (!Math::PointsToMatrix(rest.rootControl.points, controls.root.points,
                              &rootMap) ||
        !Math::PointsToMatrix(rest.endControl.points, controls.end.points,
                              &endMap))
        return false;
    (*cvs)[0] = rootMap.TransformAffine(rest.cvs[0]);
    (*cvs)[1] = rootMap.TransformAffine(rest.cvs[1]);
    (*cvs)[2] = endMap.TransformAffine(rest.cvs[2]);
    (*cvs)[3] = endMap.TransformAffine(rest.cvs[3]);

    // The mid control's follow point, from the unclamped maps so a follow
    // helper reading the same controls stays exact.
    const Double midRest = rest.midControl.points[0];
    const double w = params.midFollowWeight;
    const Double follow = rootMap.TransformAffine(midRest) * (1.0 - w) +
                          endMap.TransformAffine(midRest) * w;
    const Double offset = controls.mid.points[0] - follow;

    // The root's posed chain axis: the rest chord carried by the root.
    const Double restChord = rest.cvs[3] - rest.cvs[0];
    const double restChordLength = restChord.GetLength();
    Double rootAxis = rootMap.TransformDir(restChord);
    const bool haveAxis = restChordLength > kRigExecSplineIkEpsilon &&
                          rootAxis.GetLength() > kRigExecSplineIkEpsilon;
    if (haveAxis)
        rootAxis.Normalize();

    // Length floor along the root's axis; cv2 rides with cv3.
    if (params.minLengthRatio > 0.0 && haveAxis) {
        const double minAlong = params.minLengthRatio * restChordLength;
        const double along = Math::Dot((*cvs)[3] - (*cvs)[0], rootAxis);
        if (along < minAlong) {
            const Double lift = rootAxis * (minAlong - along);
            (*cvs)[2] += lift;
            (*cvs)[3] += lift;
        }
    }

    // Root tangent aim onto the floored chord.
    if (params.aimRootTangent && haveAxis) {
        const Double to = (*cvs)[3] - (*cvs)[0];
        if (to.GetLength() > kRigExecSplineIkEpsilon)
            (*cvs)[1] = (*cvs)[0] + RotateToward<Point, Double>(
                                        rootAxis, to.GetNormalized(),
                                        (*cvs)[1] - (*cvs)[0]);
    }

    (*cvs)[1] += offset;
    (*cvs)[2] += offset;
    for (const Double &cv : *cvs) {
        if (!IsFinite(cv)) {
            *cvs = rest.cvs;
            return false;
        }
    }
    return true;
}

/// Twist in [-pi, pi] of the rest->pose rotation about the unit \p axis,
/// scale and reflection removed; 0 for a singular or non-finite input.
template <class Point, class Double, class F>
double
RigExecSplineIkTwistAboutAxisKernel(const F &restFrame, const F &posedFrame,
                                    const Double &axis)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    using namespace splineIkDetail;
    const double axisLength = axis.GetLength();
    if (!IsFinite(axis) || axisLength <= kRigExecSplineIkEpsilon ||
        !IsFiniteFrame<Double>(restFrame) || !IsFiniteFrame<Double>(posedFrame))
        return 0.0;
    typename Math::Mat4 rotation(1.0);
    if (!Math::PointsToMatrix(restFrame.points, posedFrame.points, &rotation))
        return 0.0;
    rotation.SetTranslateOnly(Double(0.0));
    rotation.Orthonormalize();
    // Row-vector layout: drop a reflection through the Z row.
    if (rotation.GetDeterminant3() < 0.0)
        rotation.SetRow(2, -rotation.GetRow(2));
    auto q = rotation.ExtractRotationQuat();
    // Double cover: keep the twist in [-pi, pi].
    if (q.GetReal() < 0.0)
        q = -q;
    const double along = Math::Dot(q.GetImaginary(), axis / axisLength);
    return 2.0 * std::atan2(along, q.GetReal());
}

/// The chain laid out along the posed curve by arc length, aimed at the
/// next joint, its rest up transported by minimal rotation, twisted
/// linearly from root to end about the rest chain axis and scaled for
/// volume. True when every joint was determined; see RigExecSolveSplineIk
/// for the degenerate results.
template <class Point, class Double, class F, class Rest, class Controls,
          class Params, class Result>
bool
RigExecSolveSplineIkKernel(const Rest &rest, const Controls &controls,
                           const Params &params, Result *result)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    using namespace splineIkDetail;
    *result = Result();
    result->cvs = rest.cvs;

    const size_t n = rest.joints.size();
    if (n == 0 || rest.segmentLengths.size() + 1 != n ||
        (!rest.volumeWeights.empty() && rest.volumeWeights.size() != n))
        return false;

    const auto failWithRest = [&]() {
        result->joints.resize(n);
        for (size_t i = 0; i < n; ++i)
            result->joints[i].frame = DegenerateCopy(rest.joints[i]);
        return false;
    };

    bool finiteInputs =
        std::isfinite(params.preserveVolume) &&
        std::isfinite(params.midFollowWeight) && std::isfinite(params.roll) &&
        std::isfinite(params.twist) && std::isfinite(rest.restArcLength);
    for (const double s : rest.segmentLengths)
        finiteInputs = finiteInputs && std::isfinite(s);
    for (const double w : rest.volumeWeights)
        finiteInputs = finiteInputs && std::isfinite(w);
    if (!finiteInputs || rest.restArcLength <= kRigExecSplineIkEpsilon)
        return failWithRest();

    // 1. The posed curve.
    if (!RigExecSplineIkPoseCvsKernel<Point, Double>(rest, controls, params,
                                                     &result->cvs))
        return failWithRest();
    const RigExecSplineIkCurveKernel<Double> curve(result->cvs);
    result->arcLength = curve.ArcLength();
    result->ratio = result->arcLength / rest.restArcLength;
    const bool curveOk = !curve.IsDegenerate();

    // 2. Roll and twist about the rest chain axis.
    Double chainAxis = rest.cvs[3] - rest.cvs[0];
    if (chainAxis.GetLength() <= kRigExecSplineIkEpsilon)
        chainAxis = rest.rootControl.points[1] - rest.rootControl.points[0];
    double rootTwist = 0.0, endTwist = 0.0;
    if (chainAxis.GetLength() > kRigExecSplineIkEpsilon) {
        chainAxis.Normalize();
        rootTwist = RigExecSplineIkTwistAboutAxisKernel<Point, Double>(
            rest.rootControl, controls.root, chainAxis);
        endTwist = RigExecSplineIkTwistAboutAxisKernel<Point, Double>(
            rest.endControl, controls.end, chainAxis);
    }
    result->roll = rootTwist + params.roll;
    result->twist = (endTwist - rootTwist) + params.twist;

    // 3. Joints along the curve by arc length.
    result->joints.resize(n);
    std::vector<Double> positions(n, Double(0.0)), tangents(n, Double(0.0));
    double cumulative = 0.0;
    for (size_t i = 0; i < n; ++i) {
        if (i > 0)
            cumulative += rest.segmentLengths[i - 1];
        const double d = result->ratio * cumulative;
        result->joints[i].arcDistance = d;
        result->joints[i].arcParam = curveOk ? d / result->arcLength : 0.0;
        result->joints[i].twist =
            result->roll + result->twist * result->joints[i].arcParam;
        if (!curve.PointAtArcLength(d, &positions[i], &tangents[i])) {
            positions[i] = result->cvs[0];
            tangents[i] = Double(0.0);
        }
    }

    // 4. Orient, twist and scale each joint.
    bool allOk = curveOk;
    for (size_t i = 0; i < n; ++i) {
        auto &joint = result->joints[i];
        const Basis<Double> restBasis =
            MakeBasis<Point, Double>(rest.joints[i]);
        if (!restBasis.ok) {
            joint.frame = DegenerateCopy(rest.joints[i]);
            allOk = false;
            continue;
        }

        // Aim at the next joint, the last along the tangent; then the
        // tangent, then the rest aim.
        Double aim =
            (i + 1 < n) ? positions[i + 1] - positions[i] : tangents[i];
        if (aim.GetLength() <= kRigExecSplineIkEpsilon)
            aim = tangents[i];
        bool oriented = true;
        if (aim.GetLength() <= kRigExecSplineIkEpsilon) {
            aim = restBasis.x;
            oriented = false;
        }
        const Double x = aim.GetNormalized();

        // Rotation-minimising transport of the rest up vector.
        Double y = RotateToward<Point, Double>(restBasis.x, x, restBasis.y);
        y -= x * Math::Dot(x, y);
        if (y.GetLength() <= kRigExecSplineIkEpsilon)
            y = FallbackUp<Point, Double>(x);
        else
            y.Normalize();

        // y is perpendicular to x, so the twist is a rotation in the
        // (y, x cross y) plane.
        const double theta = joint.twist;
        const Double yTwisted =
            y * std::cos(theta) + Math::Cross(x, y) * std::sin(theta);
        const Double z = Math::Cross(x, yTwisted) * restBasis.handedness;

        // Volume: s_y = s_z = 1 - w_i * preserveVolume * (ratio - 1).
        const double w =
            rest.volumeWeights.empty() ? 0.0 : rest.volumeWeights[i];
        const double s =
            1.0 - w * params.preserveVolume * (result->ratio - 1.0);
        joint.scale = Double(1.0, s, s);

        auto &frame = joint.frame;
        frame.points[0] = positions[i];
        frame.points[1] = positions[i] + x * restBasis.lx;
        frame.points[2] = positions[i] + yTwisted * (restBasis.ly * s);
        frame.points[3] = positions[i] + z * (restBasis.lz * s);
        frame.flags = chainSolverDetail::kFrameValid;
        if (s != 1.0)
            frame.flags |= kFrameAffine;
        if (s < 0.0)
            frame.flags |= chainSolverDetail::kFrameReflected;
        if (!oriented || !curveOk) {
            frame.flags |= chainSolverDetail::kFrameDegenerate;
            allOk = false;
        }
    }
    return allOk;
}

} // namespace rigExec

#endif
