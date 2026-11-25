// Volumetric weight fields and their composition over borrowed ranges, shared
// by the native (Gf) path, the runtime (Rr) and the weight ops. Header-only. V
// is a float vector whose `*` is the dot product, D its double peer and M a
// row-vector double matrix.
#ifndef RIGEXEC_MATH_WEIGHT_FIELD_KERNEL_H
#define RIGEXEC_MATH_WEIGHT_FIELD_KERNEL_H

#include "matrixKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace rigExec {

/// The landmarks a volume's placement maps from: the origin and the unit
/// axes. D is a double vector.
template <class D>
std::array<D, 4>
RigExecVolumeLandmarks()
{
    return {D(0.0), D(1.0, 0.0, 0.0), D(0.0, 1.0, 0.0), D(0.0, 0.0, 1.0)};
}

enum class RigExecVolumePlacementResult { Placed, Identity, NotFinite };

/// Where a volume ended up over its frame \p points: the map
/// \p pointsToMatrix(RigExecVolumeLandmarks, points, placement) solves.
/// Identity for a frame that is not \p valid or is \p degenerate, and
/// NotFinite, with the identity, when a landmark is not finite.
template <class D, class M, class PointsToMatrix>
RigExecVolumePlacementResult
RigExecVolumePlacement(const std::array<D, 4> &points, bool valid,
                       bool degenerate, const PointsToMatrix &pointsToMatrix,
                       M *placement)
{
    *placement = M(1.0);
    if (!valid || degenerate)
        return RigExecVolumePlacementResult::Identity;
    for (const D &point : points)
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2]))
            return RigExecVolumePlacementResult::NotFinite;
    pointsToMatrix(RigExecVolumeLandmarks<D>(), points, placement);
    return RigExecVolumePlacementResult::Placed;
}

/// How one weight field folds into the accumulated result.
///
/// Multiply, Add, Max, Min and Average are order independent. Subtract and
/// Overlay consume the authored input order.
enum class RigExecWeightCombine {
    Multiply, ///< identity 1
    Add,      ///< identity 0
    Subtract, ///< first input minus each subsequent one
    Max,      ///< identity 0
    Min,      ///< identity 1
    Average,  ///< arithmetic mean over the inputs
    Overlay,  ///< a < 0.5 ? 2ab : 1 - 2(1-a)(1-b), per element
};

/// The distance-to-weight remap's settings. curve holds curveCount uniform
/// samples over r in [0, 1]; fewer than two is linear.
struct RigExecFalloffKernelParams {
    float falloffMin = 0.0f;
    float falloffMax = 1.0f;
    float invert = 0.0f;
    float strength = 1.0f;
    const float *curve = nullptr;
    size_t curveCount = 0;
};

inline float
RigExecWeightClamp01(float x)
{
    return std::min(1.0f, std::max(0.0f, x));
}

/// \p curve at \p r, linearly interpolated, r clamped; fewer than two
/// samples is the identity.
inline float
RigExecSampleFalloffLutKernel(const float *curve, size_t count, float r)
{
    if (count < 2)
        return r;
    const float x = RigExecWeightClamp01(r) * float(count - 1);
    const size_t i = std::min(count - 2, size_t(std::floor(x)));
    const float t = x - float(i);
    return curve[i] + (curve[i + 1] - curve[i]) * t;
}

/// u = clamp01((d - min) / (max - min)), inverted by invert, then
/// strength * curve(1 - u); a degenerate band is a hard step. Unclamped:
/// the range policy decides out-of-range weights.
inline float
RigExecEvaluateFalloffKernel(float distance,
                             const RigExecFalloffKernelParams &p)
{
    const float span = p.falloffMax - p.falloffMin;
    float u;
    if (std::abs(span) <= std::numeric_limits<float>::min())
        u = distance < p.falloffMin ? 0.0f : 1.0f;
    else
        u = RigExecWeightClamp01((distance - p.falloffMin) / span);
    u = u + (1.0f - 2.0f * u) * p.invert;
    return RigExecSampleFalloffLutKernel(p.curve, p.curveCount, 1.0f - u) *
           p.strength;
}

/// Distance from \p p to the segment [a, b]; a degenerate segment is its
/// endpoint.
template <class V>
float
RigExecSegmentDistanceKernel(const V &p, const V &a, const V &b)
{
    const V ab = b - a;
    const float lengthSq = ab.GetLengthSq();
    if (lengthSq <= 1e-20f)
        return (p - a).GetLength();
    const float t = RigExecWeightClamp01(((p - a) * ab) / lengthSq);
    return (p - (a + ab * t)).GetLength();
}

/// Distance to the polyline through \p curve: infinity for none, the
/// point distance for one.
template <class V>
float
RigExecCurveDistanceKernel(const V &p, const V *curve, size_t count)
{
    if (count == 0 || !curve)
        return std::numeric_limits<float>::infinity();
    if (count == 1)
        return (p - curve[0]).GetLength();
    float best = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i + 1 < count; ++i)
        best = std::min(
            best, RigExecSegmentDistanceKernel(p, curve[i], curve[i + 1]));
    return best;
}

/// |u| <= extentU and |v| <= extentV about \p axis, in-plane axes
/// (axis + 1) % 3 and (axis + 2) % 3. Hard containment.
template <class V>
bool
RigExecPlaneWithinBoundsKernel(const V &p, int axis, float extentU,
                               float extentV)
{
    if (axis < 0 || axis > 2)
        return false;
    const float u = p[size_t((axis + 1) % 3)];
    const float v = p[size_t((axis + 2) % 3)];
    return std::abs(u) <= extentU && std::abs(v) <= extentV;
}

/// A weight under the range policy: false for a non-finite one or, unless
/// \p clamp, one outside [0, 1]; clamped otherwise.
inline bool
RigExecApplyWeightRangeKernel(bool clamp, float *w)
{
    if (!std::isfinite(*w))
        return false;
    if (*w < 0.0f || *w > 1.0f) {
        if (!clamp)
            return false;
        *w = std::min(std::max(*w, 0.0f), 1.0f);
    }
    return true;
}

/// \p p carried into the volume's local space in double.
template <class V, class D, class M>
V
RigExecWeightToLocal(const M &worldToLocal, const V &p)
{
    return matrixDetail::Narrow<V>(
        worldToLocal.TransformAffine(matrixDetail::Wide<D>(p)));
}

/// Sphere field for points [begin, end) into \p out[begin, end). Each local
/// coordinate is divided by the scale its sign selects.
template <class V, class D, class M>
void
RigExecSphereWeightFieldKernel(const V *points, size_t begin, size_t end,
                               const M &worldToLocal,
                               const RigExecFalloffKernelParams &params,
                               const V &positiveScales, const V &negativeScales,
                               float *out)
{
    for (size_t i = begin; i < end; ++i) {
        V local = RigExecWeightToLocal<V, D>(worldToLocal, points[i]);
        for (size_t axis = 0; axis < 3; ++axis)
            local[axis] /= local[axis] < 0.0f ? negativeScales[axis]
                                              : positiveScales[axis];
        out[i] = RigExecEvaluateFalloffKernel(local.GetLength(), params);
    }
}

/// Plane field: the signed distance along \p axis; with \p bounded, zero
/// outside the extentU by extentV rectangle.
template <class V, class D, class M>
void
RigExecPlaneWeightFieldKernel(const V *points, size_t begin, size_t end,
                              const M &worldToLocal, int axis,
                              const RigExecFalloffKernelParams &params,
                              bool bounded, float extentU, float extentV,
                              float *out)
{
    for (size_t i = begin; i < end; ++i) {
        const V local = RigExecWeightToLocal<V, D>(worldToLocal, points[i]);
        if (bounded &&
            !RigExecPlaneWithinBoundsKernel(local, axis, extentU, extentV)) {
            out[i] = 0.0f;
            continue;
        }
        const float d = axis >= 0 && axis <= 2 ? local[size_t(axis)] : 0.0f;
        out[i] = RigExecEvaluateFalloffKernel(d, params);
    }
}

/// Curve field over a curve already in local space (\p localCurve,
/// non-empty). \p distance(local point) may replace the polyline scan, as
/// an acceleration structure that returns the same minimum.
template <class V, class D, class M, class Distance>
void
RigExecCurveWeightFieldKernel(const V *points, size_t begin, size_t end,
                              const M &worldToLocal,
                              const RigExecFalloffKernelParams &params,
                              Distance &&distance, float *out)
{
    for (size_t i = begin; i < end; ++i)
        out[i] = RigExecEvaluateFalloffKernel(
            distance(RigExecWeightToLocal<V, D>(worldToLocal, points[i])),
            params);
}

/// The seed value a fold of zero inputs produces under \p mode.
inline float
RigExecWeightCombineIdentity(RigExecWeightCombine mode)
{
    switch (mode) {
    case RigExecWeightCombine::Multiply:
    case RigExecWeightCombine::Min:
        return 1.0f;
    case RigExecWeightCombine::Add:
    case RigExecWeightCombine::Subtract:
    case RigExecWeightCombine::Max:
    case RigExecWeightCombine::Average:
    case RigExecWeightCombine::Overlay:
        return 0.0f;
    }
    return 0.0f;
}

/// One value folded into an accumulator. Average sums here; the folds
/// below divide by the count.
inline float
RigExecFoldWeight(RigExecWeightCombine mode, float acc, float value)
{
    switch (mode) {
    case RigExecWeightCombine::Multiply:
        return acc * value;
    case RigExecWeightCombine::Add:
        return acc + value;
    case RigExecWeightCombine::Subtract:
        return acc - value;
    case RigExecWeightCombine::Max:
        return std::max(acc, value);
    case RigExecWeightCombine::Min:
        return std::min(acc, value);
    case RigExecWeightCombine::Average:
        return acc + value;
    case RigExecWeightCombine::Overlay:
        return acc < 0.5f ? 2.0f * acc * value
                          : 1.0f - 2.0f * (1.0f - acc) * (1.0f - value);
    }
    return acc;
}

/// Ordered fold of \p inputCount fields, each published on demand by
/// \p resolve(k, &data, &size) and alive until the next call. Subtract and
/// Overlay seed from the first input; zero inputs is the identity at
/// \p elementCount. A failed resolve or a field of another length clears
/// \p out and returns false.
template <typename Resolve>
bool
RigExecCombineWeightFieldsStreamed(RigExecWeightCombine mode, size_t inputCount,
                                   size_t elementCount, Resolve &&resolve,
                                   std::vector<float> *out)
{
    const bool seedFromFirst = mode == RigExecWeightCombine::Subtract ||
                               mode == RigExecWeightCombine::Overlay;
    if (inputCount == 0) {
        out->assign(elementCount, RigExecWeightCombineIdentity(mode));
        return true;
    }
    const float *data = nullptr;
    size_t size = 0;
    size_t first = 0;
    if (seedFromFirst) {
        if (!resolve(0, &data, &size) || size != elementCount) {
            out->clear();
            return false;
        }
        // data() on an empty input is not a range.
        if (elementCount == 0)
            out->clear();
        else
            out->assign(data, data + elementCount);
        first = 1;
    } else {
        out->assign(elementCount, RigExecWeightCombineIdentity(mode));
    }
    for (size_t k = first; k < inputCount; ++k) {
        if (!resolve(k, &data, &size) || size != elementCount) {
            out->clear();
            return false;
        }
        for (size_t i = 0; i < elementCount; ++i)
            (*out)[i] = RigExecFoldWeight(mode, (*out)[i], data[i]);
    }
    if (mode == RigExecWeightCombine::Average) {
        const float inv = 1.0f / float(inputCount);
        for (float &w : *out)
            w *= inv;
    }
    return true;
}

} // namespace rigExec

#endif
