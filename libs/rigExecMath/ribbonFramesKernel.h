// Ribbon frame sampling: rotation-minimizing frames along a cubic uniform
// B-spline, and the re-base of aggregate elements onto joint rests. Shared by
// the native (Gf) path, the runtime (Rr) and the solver ops.
//
// V is a float vector whose `*` is the dot product and `^` the cross
// product. Frames are duck-typed: `points` (four Double) and `flags`.
#ifndef RIGEXEC_MATH_RIBBON_FRAMES_KERNEL_H
#define RIGEXEC_MATH_RIBBON_FRAMES_KERNEL_H

#include "chainSolverKernel.h"
#include "matrixKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

/// Sampled curve frames: positions plus a rotation-minimizing frame per
/// sample, parameterized by normalized arc length.
template <class V> struct RigExecCurveFrameSamplesT {
    std::vector<V> positions;
    std::vector<V> tangents;
    std::vector<V> normals; ///< rotation-minimizing
    std::vector<V> binormals;
    std::vector<float> parameters; ///< normalized arc length in [0, 1]

    size_t GetSize() const
    {
        return positions.size();
    }
};

namespace ribbonFramesDetail {

/// Uniform cubic B-spline position on one nonperiodic segment span.
template <class V>
V
BsplinePoint(const V &p0, const V &p1, const V &p2, const V &p3, float t)
{
    const float t2 = t * t, t3 = t2 * t;
    const float b0 = (1 - 3 * t + 3 * t2 - t3) / 6.0f;
    const float b1 = (4 - 6 * t2 + 3 * t3) / 6.0f;
    const float b2 = (1 + 3 * t + 3 * t2 - 3 * t3) / 6.0f;
    const float b3 = t3 / 6.0f;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

} // namespace ribbonFramesDetail

/// \p sampleCount equal-arc-length samples of the nonperiodic cubic uniform
/// B-spline over \p cvs, rotation-minimizing by double reflection (Wang et
/// al. 2008). Under four points the control polygon interpolates linearly.
/// Empty for fewer than two points or samples, or a zero-length curve.
template <class V>
RigExecCurveFrameSamplesT<V>
RigExecSampleCurveRMFKernel(const V *cvs, size_t count, int sampleCount)
{
    RigExecCurveFrameSamplesT<V> samples;
    if (sampleCount < 2 || count < 2 || !cvs)
        return samples;

    // Dense presampling for the arc-length parameterization.
    const int dense = std::max(sampleCount * 16, 64);
    std::vector<V> densePoints;
    densePoints.reserve(size_t(dense) + 1);
    const int spans = int(count) - 3;
    for (int i = 0; i <= dense; ++i) {
        const float u = float(i) / float(dense);
        if (spans >= 1) {
            const float s = u * spans;
            const int span = std::min(spans - 1, int(s));
            const float t = s - span;
            densePoints.push_back(ribbonFramesDetail::BsplinePoint(
                cvs[span], cvs[span + 1], cvs[span + 2], cvs[span + 3], t));
        } else {
            const float s = u * (count - 1);
            const int seg = std::min(int(count) - 2, int(s));
            densePoints.push_back(cvs[seg] +
                                  (cvs[seg + 1] - cvs[seg]) * (s - seg));
        }
    }
    std::vector<float> arcLength(densePoints.size(), 0.0f);
    for (size_t i = 1; i < densePoints.size(); ++i)
        arcLength[i] = arcLength[i - 1] +
                       (densePoints[i] - densePoints[i - 1]).GetLength();
    const float total = arcLength.back();
    if (total <= 1e-12f)
        return samples;

    const size_t n = size_t(sampleCount);
    samples.positions.reserve(n);
    samples.parameters.reserve(n);
    size_t cursor = 0;
    for (int k = 0; k < sampleCount; ++k) {
        const float target = total * float(k) / float(sampleCount - 1);
        while (cursor + 1 < arcLength.size() && arcLength[cursor + 1] < target)
            ++cursor;
        const float span = arcLength[cursor + 1] - arcLength[cursor];
        const float t =
            span > 1e-12f ? (target - arcLength[cursor]) / span : 0.0f;
        samples.positions.push_back(
            densePoints[cursor] +
            (densePoints[cursor + 1] - densePoints[cursor]) * t);
        samples.parameters.push_back(float(k) / float(sampleCount - 1));
    }

    auto &pos = samples.positions;
    auto &tan = samples.tangents;
    auto &nrm = samples.normals;
    auto &bin = samples.binormals;
    tan.resize(n);
    for (size_t k = 0; k < n; ++k) {
        const V tangent = pos[std::min(n - 1, k + 1)] - pos[k ? k - 1 : 0];
        const float len = tangent.GetLength();
        tan[k] = len > 1e-12f ? tangent / len : V(1, 0, 0);
    }
    nrm.resize(n);
    bin.resize(n);
    {
        // The world axis least parallel to the first tangent, projected.
        const V t0 = tan[0];
        V candidate(1, 0, 0);
        float best = 2.0f;
        for (const V axis : {V(1, 0, 0), V(0, 1, 0), V(0, 0, 1)}) {
            const float align = std::abs(axis * t0);
            if (align < best) {
                best = align;
                candidate = axis;
            }
        }
        V n0 = candidate - t0 * (t0 * candidate);
        n0.Normalize();
        nrm[0] = n0;
        bin[0] = t0 ^ n0;
    }
    for (size_t k = 0; k + 1 < n; ++k) {
        const V v1 = pos[k + 1] - pos[k];
        const float c1 = v1 * v1;
        if (c1 <= 1e-20f) {
            nrm[k + 1] = nrm[k];
            bin[k + 1] = bin[k];
            continue;
        }
        const V nL = nrm[k] - v1 * (2.0f / c1) * (v1 * nrm[k]);
        const V tL = tan[k] - v1 * (2.0f / c1) * (v1 * tan[k]);
        const V v2 = tan[k + 1] - tL;
        const float c2 = v2 * v2;
        V m = c2 > 1e-20f ? nL - v2 * (2.0f / c2) * (v2 * nL) : nL;
        m -= tan[k + 1] * (tan[k + 1] * m);
        const float len = m.GetLength();
        nrm[k + 1] = len > 1e-12f ? m / len : nrm[k];
        bin[k + 1] = tan[k + 1] ^ nrm[k + 1];
    }
    return samples;
}

/// Re-bases \p frame onto \p jointRest: its map from \p ownRest is applied
/// to \p jointRest instead, flags kept. False, unchanged, when the map is
/// singular.
template <class Point, class Double, class F>
bool
RigExecRebaseFrameKernel(const std::array<Double, 4> &ownRest,
                         const std::array<Double, 4> &jointRest, F *frame)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    typename Math::Mat4 map(1.0);
    if (!Math::PointsToMatrix(ownRest, frame->points, &map))
        return false;
    frame->points = Math::MatrixToPoints(jointRest, map);
    return true;
}

/// Each element k whose \p jointRest(k) is non-null is re-based onto it,
/// and its rest becomes that joint rest; a singular element is left alone.
template <class Point, class Double, class F, class JointRest>
void
RigExecRebaseFramesKernel(JointRest &&jointRest, std::vector<F> *frames,
                          std::vector<std::array<Double, 4>> *rests)
{
    for (size_t k = 0; k < frames->size(); ++k) {
        const std::array<Double, 4> *joint = jointRest(k);
        if (joint && RigExecRebaseFrameKernel<Point, Double>(
                         (*rests)[k], *joint, &(*frames)[k]))
            (*rests)[k] = *joint;
    }
}

/// The ribbon solver's aggregate: one frame per sample of the \p posed
/// curve, each paired with the \p rest curve's sample frame. Both empty
/// unless both curves sample to \p sampleCount.
template <class Point, class Double, class F>
void
RigExecSampleRibbonFramesKernel(const Point *posed, size_t posedCount,
                                const Point *rest, size_t restCount,
                                int sampleCount, std::vector<F> *frames,
                                std::vector<std::array<Double, 4>> *rests)
{
    frames->clear();
    rests->clear();
    if (posedCount == 0 || restCount == 0 || sampleCount < 2)
        return;
    const auto p = RigExecSampleCurveRMFKernel(posed, posedCount, sampleCount);
    const auto r = RigExecSampleCurveRMFKernel(rest, restCount, sampleCount);
    const size_t n = size_t(sampleCount);
    if (p.GetSize() != n || r.GetSize() != n)
        return;
    const auto landmarks = [](const RigExecCurveFrameSamplesT<Point> &s,
                              size_t k) {
        const Point &o = s.positions[k];
        return std::array<Double, 4>{
            matrixDetail::Wide<Double>(o),
            matrixDetail::Wide<Double>(o + s.tangents[k]),
            matrixDetail::Wide<Double>(o + s.normals[k]),
            matrixDetail::Wide<Double>(o + s.binormals[k])};
    };
    frames->reserve(n);
    rests->reserve(n);
    for (size_t k = 0; k < n; ++k) {
        F frame;
        frame.points = landmarks(p, k);
        frame.flags = chainSolverDetail::kFrameValid;
        frames->push_back(frame);
        rests->push_back(landmarks(r, k));
    }
}

} // namespace rigExec

#endif
