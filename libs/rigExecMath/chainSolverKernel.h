// The FK chain and the analytic two-bone IK over four-point frames, shared
// by the native (Gf) path, the zero-USD runtime (Rr) and the solver ops.
// Header-only and USD-free.
//
// Host types come from RigExecDualQuatMath<Point, Double>. Frames, chain
// elements and IK parameters are duck-typed: a frame has `points` (four
// Double) and `flags`; an element has restPoints, posePoints,
// outRestPoints, hasOutRest and parentIndex; the IK parameters carry the
// RigExecTwoBoneIkParams fields.
#ifndef RIGEXEC_MATH_CHAIN_SOLVER_KERNEL_H
#define RIGEXEC_MATH_CHAIN_SOLVER_KERNEL_H

#include "limbStretchKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rigExec {

template <class Point, class Double> struct RigExecDualQuatMath;

namespace chainSolverDetail {

constexpr uint32_t kFrameValid = 1u << 0;
constexpr uint32_t kFrameDegenerate = 1u << 1;
constexpr uint32_t kFrameReflected = 1u << 2;

/// RigExecFrameEpsilon: 1e-10 times the longest rest handle, at least 1.
template <class Double>
double
FrameEpsilon(const std::array<Double, 4> &rest)
{
    const double lx = (rest[1] - rest[0]).GetLength();
    const double ly = (rest[2] - rest[0]).GetLength();
    const double lz = (rest[3] - rest[0]).GetLength();
    return 1e-10 * std::max({1.0, lx, ly, lz});
}

/// RigExecMatrixToPoints: \p rest carried by \p matrix, classified
/// reflected and degenerate.
template <class Point, class Double, class F, class M>
F
MatrixToFrame(const std::array<Double, 4> &rest, const M &matrix)
{
    using Mat3 = typename RigExecDualQuatMath<Point, Double>::Mat3;
    F frame;
    for (size_t i = 0; i < 4; ++i)
        frame.points[i] = matrix.TransformAffine(rest[i]);
    frame.flags = kFrameValid;
    const auto &p = frame.points;
    const Mat3 posed(p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2],
                     p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2],
                     p[3][0] - p[0][0], p[3][1] - p[0][1], p[3][2] - p[0][2]);
    const double det = posed.GetDeterminant();
    if (det < 0)
        frame.flags |= kFrameReflected;
    const double eps = FrameEpsilon(rest);
    if (std::abs(det) <= eps * eps * eps)
        frame.flags |= kFrameDegenerate;
    return frame;
}

/// Rodrigues: \p v about the unit \p axis by \p angle.
template <class Point, class Double>
Double
Rotate(const Double &v, const Double &axis, double angle)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return v * c + Math::Cross(axis, v) * s +
           axis * Math::Dot(axis, v) * (1 - c);
}

/// A rigid frame at \p origin along \p ex, \p eyCandidate orthogonalized
/// against it (a world axis when parallel), keeping the rest handle lengths.
template <class Point, class Double, class F>
F
FrameFromAxes(const std::array<Double, 4> &rest, const Double &origin,
              const Double &ex, const Double &eyCandidate)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    Double ey = eyCandidate - ex * Math::Dot(ex, eyCandidate);
    if (ey.GetLength() < 1e-12) {
        const Double axes[3] = {Double(0, 1, 0), Double(0, 0, 1),
                                Double(1, 0, 0)};
        for (const Double &axis : axes) {
            ey = axis - ex * Math::Dot(ex, axis);
            if (ey.GetLength() >= 1e-12)
                break;
        }
    }
    ey.Normalize();
    Double ez = Math::Cross(ex, ey);
    ez.Normalize();
    const double lx = (rest[1] - rest[0]).GetLength();
    const double ly = (rest[2] - rest[0]).GetLength();
    const double lz = (rest[3] - rest[0]).GetLength();
    F f;
    f.points[0] = origin;
    f.points[1] = origin + ex * lx;
    f.points[2] = origin + ey * ly;
    f.points[3] = origin + ez * lz;
    f.flags = kFrameValid;
    return f;
}

} // namespace chainSolverDetail

/// Each control's rest-to-pose map composed with its parent's accumulated
/// one (row convention: own * parent), applied to the element's output
/// rest. Elements are topologically ordered.
template <class Point, class Double, class F, class E>
std::vector<F>
RigExecSolveFkChainKernel(const std::vector<E> &elements)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    using Mat4 = typename Math::Mat4;
    std::vector<F> result;
    result.reserve(elements.size());
    std::vector<Mat4> accumulated;
    accumulated.reserve(elements.size());
    for (const E &e : elements) {
        Mat4 own(1.0);
        Math::PointsToMatrix(e.restPoints, e.posePoints, &own);
        Mat4 w = own;
        if (e.parentIndex >= 0 && size_t(e.parentIndex) < accumulated.size())
            w = own * accumulated[size_t(e.parentIndex)];
        accumulated.push_back(w);
        result.push_back(chainSolverDetail::MatrixToFrame<Point, Double, F>(
            e.hasOutRest ? e.outRestPoints : e.restPoints, w));
    }
    return result;
}

/// segmentScale "toChild": each frame scales along the bone to the next by
/// posed over rest length, the rest measured under its own frame.
template <class Double, class F, class E>
void
RigExecScaleFkSegmentsKernel(const std::vector<E> &elements,
                             std::vector<F> *frames)
{
    for (size_t k = 0; k + 1 < elements.size() && k + 1 < frames->size(); ++k) {
        const E &a = elements[k];
        const E &b = elements[k + 1];
        const auto &ra = a.hasOutRest ? a.outRestPoints : a.restPoints;
        const auto &rb = b.hasOutRest ? b.outRestPoints : b.restPoints;
        const double rest = RigExecBoneLengthUnderFrame(
            ra.data(), (*frames)[k].points.data(), Double(rb[0] - ra[0]));
        const Double bone = (*frames)[k + 1].points[0] - (*frames)[k].points[0];
        const double posed = bone.GetLength();
        if (rest > 1e-12 && posed > 1e-12)
            RigExecScaleFrameAlong((*frames)[k].points.data(), bone,
                                   posed / rest);
    }
}

/// The chain's bone lengths measured in \p space, each offset riding its
/// own bone's factor.
template <class Double, class M>
void
RigExecTwoBoneIkLengthsKernel(
    const std::array<std::array<Double, 4>, 3> &restPoints, const M &space,
    double upperOffset, double lowerOffset, double *upperLength,
    double *lowerLength)
{
    const double rawUpper = (restPoints[1][0] - restPoints[0][0]).GetLength();
    const double rawLower = (restPoints[2][0] - restPoints[1][0]).GetLength();
    const Double s0 = space.Transform(restPoints[0][0]);
    const Double s1 = space.Transform(restPoints[1][0]);
    const Double s2 = space.Transform(restPoints[2][0]);
    const double spacedUpper = (s1 - s0).GetLength();
    const double spacedLower = (s2 - s1).GetLength();
    if (upperLength) {
        const double f = rawUpper > 1e-12 ? spacedUpper / rawUpper : 1.0;
        *upperLength = spacedUpper + upperOffset * f;
    }
    if (lowerLength) {
        const double f = rawLower > 1e-12 ? spacedLower / rawLower : 1.0;
        *lowerLength = spacedLower + lowerOffset * f;
    }
}

/// How much \p space lengthens the rest chain; 1 for identity.
template <class Double, class M>
double
RigExecTwoBoneIkSpaceFactorKernel(
    const std::array<std::array<Double, 4>, 3> &restPoints, const M &space)
{
    double rawUpper = 0.0, rawLower = 0.0, upper = 0.0, lower = 0.0;
    RigExecTwoBoneIkLengthsKernel(restPoints, M(1.0), 0.0, 0.0, &rawUpper,
                                  &rawLower);
    RigExecTwoBoneIkLengthsKernel(restPoints, space, 0.0, 0.0, &upper, &lower);
    const double raw = rawUpper + rawLower;
    return raw > 1e-12 ? (upper + lower) / raw : 1.0;
}

/// Analytic two-bone IK with a pole: root, mid and end frames. The end
/// copies the effector at the solved end position. A coincident goal with
/// a degenerate root aim returns the inputs flagged degenerate.
template <class Point, class Double, class F, class P>
std::array<F, 3>
RigExecSolveTwoBoneIkKernel(const F &rootFrame, const F &effectorFrame,
                            const F &poleFrame,
                            const std::array<std::array<Double, 4>, 3> &rest,
                            const P &params)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    using chainSolverDetail::Rotate;
    const double pi = std::acos(-1.0);
    const Double root = rootFrame.points[0];
    const Double goal = effectorFrame.points[0];
    const Double pole = poleFrame.points[0];
    const double l1 = std::max(params.upperLength, 1e-9);
    const double l2 = std::max(params.lowerLength, 1e-9);
    const double chain = l1 + l2;

    const Double toGoal = goal - root;
    const double dist = toGoal.GetLength();
    Double aim;
    if (dist > 1e-12) {
        aim = toGoal / dist;
    } else {
        const Double rootAim = rootFrame.points[1] - root;
        if (rootAim.GetLength() < 1e-12) {
            std::array<F, 3> failed = {rootFrame, rootFrame, effectorFrame};
            for (F &f : failed)
                f.flags |= chainSolverDetail::kFrameDegenerate;
            return failed;
        }
        aim = rootAim.GetNormalized();
    }

    double reach = dist;
    double s1 = l1, s2 = l2;
    if (params.softDistancePolicy) {
        // The limb model sets the lengths; the solve reaches what they allow.
        RigExecLimbSegmentLengths(l1, l2, dist, (pole - root).GetLength(),
                                  (goal - pole).GetLength(), params.limb, &s1,
                                  &s2);
        s1 = std::max(s1, 1e-9);
        s2 = std::max(s2, 1e-9);
        reach = std::min(dist, s1 + s2);
    } else {
        // clampWithSoftness: past chain - soft, ease toward the chain.
        const double soft = std::max(params.softness, 0.0) * chain;
        if (soft > 1e-12 && dist > chain - soft)
            reach = chain - soft * std::exp(-(dist - (chain - soft)) / soft);
        else if (dist > chain)
            reach = chain;
        // uniformSegments: blend toward the stretch that reaches the goal.
        if (dist > reach && params.stretch > 0.0) {
            const double factor =
                1.0 + (dist / chain - 1.0) *
                          std::min(std::max(params.stretch, 0.0), 1.0);
            if (factor > 1.0) {
                s1 = l1 * factor;
                s2 = l2 * factor;
                reach = std::min(dist, s1 + s2);
            }
        }
    }
    reach = std::min(reach, s1 + s2);
    reach = std::max(reach, std::abs(s1 - s2) + 1e-12);

    // Bend plane: the pole, else the rest up, else the world axis least
    // parallel to the aim, turned by the preferred bend.
    Double poleDir = pole - root;
    poleDir -= aim * Math::Dot(aim, poleDir);
    Double bendUp;
    if (poleDir.GetLength() > 1e-12) {
        bendUp = poleDir.GetNormalized();
    } else {
        Double candidate = rest[0][2] - rest[0][0];
        candidate -= aim * Math::Dot(aim, candidate);
        if (candidate.GetLength() < 1e-12) {
            const Double axes[3] = {Double(1, 0, 0), Double(0, 1, 0),
                                    Double(0, 0, 1)};
            double best = 2.0;
            for (const Double &axis : axes) {
                const double align = std::abs(Math::Dot(axis, aim));
                if (align < best) {
                    best = align;
                    candidate = axis - aim * Math::Dot(aim, axis);
                }
            }
        }
        bendUp = Rotate<Point, Double>(candidate.GetNormalized(), aim,
                                       params.preferredBendRadians);
    }
    if (params.twistRadians != 0.0 && std::isfinite(params.twistRadians))
        bendUp = Rotate<Point, Double>(bendUp, aim, params.twistRadians);
    Double bendAxis = Math::Cross(aim, bendUp);
    if (bendAxis.GetLength() < 1e-12)
        bendAxis =
            Math::Cross(aim, Rotate<Point, Double>(bendUp, aim, 0.5 * pi));
    bendAxis.Normalize();

    // Law of cosines for the root's interior angle.
    const double cosAlpha =
        (s1 * s1 + reach * reach - s2 * s2) / (2 * s1 * reach);
    const double alpha = std::acos(std::min(std::max(cosAlpha, -1.0), 1.0));
    const Double upperDir = Rotate<Point, Double>(aim, bendAxis, alpha);
    const Double mid = root + upperDir * s1;
    const Double endPos = root + aim * reach;
    const Double lowerDir = (endPos - mid).GetNormalized();

    std::array<F, 3> out;
    out[0] = chainSolverDetail::FrameFromAxes<Point, Double, F>(
        rest[0], root, upperDir, bendUp);
    out[1] = chainSolverDetail::FrameFromAxes<Point, Double, F>(
        rest[1], mid, lowerDir, bendUp);
    if (params.scaleSegments) {
        RigExecScaleFrameAlong(out[0].points.data(), upperDir, s1 / l1);
        RigExecScaleFrameAlong(out[1].points.data(), lowerDir,
                               (endPos - mid).GetLength() / l2);
    }

    // The space scales the published handles, so children read its scale.
    if (params.space != decltype(params.space)(1.0)) {
        for (size_t f = 0; f < 2; ++f) {
            const Double origin = out[f].points[0];
            for (size_t a = 1; a < 4; ++a) {
                const Double handle = out[f].points[a] - origin;
                const double length = handle.GetLength();
                if (length < 1e-12)
                    continue;
                const Double direction = handle / length;
                const double factor =
                    params.space.TransformDir(direction).GetLength();
                out[f].points[a] = origin + direction * (length * factor);
            }
        }
    }

    F end = effectorFrame;
    const Double offset = endPos - effectorFrame.points[0];
    for (auto &p : end.points)
        p += offset;
    out[2] = end;
    return out;
}

} // namespace rigExec

#endif
