// The SRT blend of two four-point frames and the swing/twist distribution
// between two frames, shared by the native (Gf) path, the runtime (Rr) and the
// solver ops. Header-only.
//
// Host types come from RigExecDualQuatMath<Point, Double>. A frame is
// duck-typed: `points` (four Double) and `flags`.
#ifndef RIGEXEC_MATH_FRAME_BLEND_KERNEL_H
#define RIGEXEC_MATH_FRAME_BLEND_KERNEL_H

#include "chainSolverKernel.h"
#include "frameParamsKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rigExec {

namespace frameBlendDetail {

/// True when \p a and \p b lie in opposite hemispheres.
template <class Point, class Double, class Q>
bool
OppositeHemisphere(const Q &a, const Q &b)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    return Math::Dot(a.GetImaginary(), b.GetImaginary()) +
               a.GetReal() * b.GetReal() <
           0;
}

} // namespace frameBlendDetail

/// Swing/twist split of \p q about the unit \p axis: q = swing * twist,
/// twist a rotation about the axis (Dobrowolski 2015).
template <class Point, class Double, class Q>
void
RigExecSwingTwistKernel(const Q &q, const Double &axis, Q *swing, Q *twist)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    const double proj = Math::Dot(q.GetImaginary(), axis);
    const Q t(q.GetReal(), axis * proj);
    const double len = std::sqrt(t.GetReal() * t.GetReal() +
                                 Math::Dot(t.GetImaginary(), t.GetImaginary()));
    if (len < 1e-15)
        *twist = Q::GetIdentity();
    else
        *twist = t * (1.0 / len);
    *swing = q * twist->GetInverse();
}

/// \p a and \p b blended by \p weight (clamped) through translation,
/// shortest-arc slerped rotation, linear or \p logScale scale, and linear
/// shear. The ends return the inputs bit for bit
/// unless \p outRest re-bases them. A singular input holds the stronger
/// input, flagged degenerate.
template <class Point, class Double, class F>
F
RigExecBlendFramesKernel(const F &a, const F &b,
                         const std::array<Double, 4> &rest, double weight,
                         bool logScale, const std::array<Double, 4> *outRest)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    using Params = RigExecTransformParamsKernelT<Point, Double>;
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (!outRest) {
        if (w <= 0.0)
            return a;
        if (w >= 1.0)
            return b;
    } else if (w <= 0.0 || w >= 1.0) {
        const F &pick = (w <= 0.0) ? a : b;
        typename Math::Mat4 end(1.0);
        if (!Math::PointsToMatrix(rest, pick.points, &end)) {
            F held = pick;
            held.flags |= chainSolverDetail::kFrameDegenerate;
            return held;
        }
        return chainSolverDetail::MatrixToFrame<Point, Double, F>(*outRest,
                                                                  end);
    }

    Params pa, pb;
    if (!RigExecPointsToParamsKernel<Point, Double>(rest, a.points, 2, &pa) ||
        !RigExecPointsToParamsKernel<Point, Double>(rest, b.points, 2, &pb)) {
        F held = (w < 0.5) ? a : b;
        held.flags |= chainSolverDetail::kFrameDegenerate;
        return held;
    }

    Params pr;
    pr.translation = pa.translation * (1 - w) + pb.translation * w;
    typename Math::Quat qa = pa.rotation, qb = pb.rotation;
    if (frameBlendDetail::OppositeHemisphere<Point, Double>(qa, qb))
        qb = -qb;
    pr.rotation = Math::Slerp(w, qa, qb).GetNormalized();
    for (size_t i = 0; i < 3; ++i) {
        if (logScale && pa.scale[i] > 0 && pb.scale[i] > 0)
            pr.scale[i] = std::exp(std::log(pa.scale[i]) * (1 - w) +
                                   std::log(pb.scale[i]) * w);
        else
            pr.scale[i] = pa.scale[i] * (1 - w) + pb.scale[i] * w;
        pr.shear[i] = pa.shear[i] * (1 - w) + pb.shear[i] * w;
    }
    return chainSolverDetail::MatrixToFrame<Point, Double, F>(
        outRest ? *outRest : rest,
        RigExecParamsToMatrixKernel<Point, Double>(pr));
}

/// One frame per weight (clamped) between \p start and \p end: swing
/// slerped, the aim-axis twist unwrapped by \p twistTurns and scaled
/// linearly, scale and shear linear, origins interpolated. Empty for a
/// non-finite twist; a singular end holds the nearer end, flagged
/// degenerate.
template <class Point, class Double, class F>
std::vector<F>
RigExecDistributeTwistKernel(const F &start, const F &end,
                             const std::array<Double, 4> &startRest,
                             const std::array<Double, 4> &endRest,
                             const std::vector<double> &weights,
                             double twistTurns)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    using Quat = typename Math::Quat;
    using Params = RigExecTransformParamsKernelT<Point, Double>;
    std::vector<F> result;
    if (!std::isfinite(twistTurns))
        return result;
    result.reserve(weights.size());

    Params ps, pe;
    const bool okS = RigExecPointsToParamsKernel<Point, Double>(
        startRest, start.points, 2, &ps);
    const bool okE =
        RigExecPointsToParamsKernel<Point, Double>(endRest, end.points, 2, &pe);
    if (!okS || !okE) {
        for (double w : weights) {
            F f = (w < 0.5) ? start : end;
            f.flags |= chainSolverDetail::kFrameDegenerate;
            result.push_back(f);
        }
        return result;
    }

    const Double aim = (start.points[1] - start.points[0]).GetNormalized();
    Quat qs = ps.rotation, qe = pe.rotation;
    if (frameBlendDetail::OppositeHemisphere<Point, Double>(qs, qe))
        qe = -qe;
    const Quat rel = qs.GetInverse() * qe;
    const Quat qsInv = qs.GetInverse();
    const Double localAim = qsInv.Transform(aim);
    Quat swing, twist;
    RigExecSwingTwistKernel<Point, Double>(rel, localAim, &swing, &twist);

    double twistAngle =
        2.0 *
        std::atan2(Math::Dot(twist.GetImaginary(), localAim), twist.GetReal());
    twistAngle += 2.0 * std::acos(-1.0) * twistTurns;
    if (!std::isfinite(twistAngle))
        return result;

    for (size_t k = 0; k < weights.size(); ++k) {
        const double w = std::min(std::max(weights[k], 0.0), 1.0);
        const Quat swingK =
            Math::Slerp(w, Quat::GetIdentity(), swing).GetNormalized();
        const double angK = twistAngle * w;
        const Quat twistK(std::cos(angK / 2), localAim * std::sin(angK / 2));

        Params pk;
        pk.translation = ps.translation * (1 - w) + pe.translation * w;
        pk.rotation = (qs * swingK * twistK).GetNormalized();
        pk.scale = ps.scale * (1 - w) + pe.scale * w;
        pk.shear = ps.shear * (1 - w) + pe.shear * w;

        // Samples carry start's rest landmarks; the origin interpolates
        // between the frame origins.
        F f = chainSolverDetail::MatrixToFrame<Point, Double, F>(
            startRest, RigExecParamsToMatrixKernel<Point, Double>(pk));
        const Double origin = start.points[0] * (1 - w) + end.points[0] * w;
        const Double shift = origin - f.points[0];
        for (auto &p : f.points)
            p += shift;
        result.push_back(f);
    }
    return result;
}

} // namespace rigExec

#endif
