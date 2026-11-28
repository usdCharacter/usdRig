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
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>
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

/// The inverse of the rigid part of a volume's placement over its frame
/// \p points (\p pointsToMatrix as above, scale and shear removed by
/// \p removeScaleShear); false for a frame that is not \p valid, is
/// \p degenerate, or places nothing finite and invertible.
template <class D, class M, class PointsToMatrix, class RemoveScaleShear>
bool
RigExecRigidWorldToLocalKernel(const std::array<D, 4> &points, bool valid,
                               bool degenerate,
                               const PointsToMatrix &pointsToMatrix,
                               const RemoveScaleShear &removeScaleShear,
                               M *worldToLocal)
{
    if (!valid || degenerate)
        return false;
    M placement(1.0);
    if (!pointsToMatrix(RigExecVolumeLandmarks<D>(), points, &placement))
        return false;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (!std::isfinite(placement[i][j]))
                return false;
    const M rigid = removeScaleShear(placement);
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12)
        return false;
    *worldToLocal = rigid.GetInverse();
    return true;
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

/// A weight packet's representation, numbered as rigExecEvalCore's
/// WeightRepresentation and rigExecRig's WeightRepresentation.
enum RigExecWeightPacketRepresentation : int {
    kRigExecWeightConstant = 0,
    kRigExecWeightDense = 1,
    kRigExecWeightSparse = 2,
};

/// A packet read in place.
struct RigExecWeightPacketView {
    bool valid = false;
    int representation = kRigExecWeightConstant;
    float defaultWeight = 0.0f;
    const float *values = nullptr;
    size_t valueCount = 0;
    const int32_t *indices = nullptr;
    size_t indexCount = 0;
};

/// What a packet kernel builds.
struct RigExecWeightPacketBuild {
    float defaultWeight = 0.0f;
    std::vector<float> values;
    std::vector<int32_t> indices;
};

/// A static packet's last sorted support, reused while its indices repeat,
/// and the pairs it sorts.
struct RigExecStaticWeightScratch {
    std::vector<std::pair<int32_t, float>> pairs;
    std::vector<int32_t> support;
    std::vector<size_t> supportOrder;
    bool uniqueSupport = false;
};

/// The weight of element \p i of a \p count-element target, or -1 for a
/// cardinality mismatch. An unknown representation reads as constant.
template <class View>
inline float
RigExecResolveWeightPacket(const View &packet, size_t i, size_t count)
{
    switch (packet.representation) {
    case kRigExecWeightDense:
        if (packet.valueCount != count || i >= count)
            return -1.0f;
        return packet.values[i];
    case kRigExecWeightSparse: {
        if (packet.indexCount != packet.valueCount)
            return -1.0f;
        if (!packet.indexCount)
            return packet.defaultWeight;
        const auto it = std::lower_bound(
            packet.indices, packet.indices + packet.indexCount, int32_t(i));
        if (it != packet.indices + packet.indexCount && *it == int32_t(i))
            return packet.values[size_t(it - packet.indices)];
        return packet.defaultWeight;
    }
    default:
        break;
    }
    return packet.defaultWeight;
}

/// A combine input as a dense field of \p count weights in \p data: a
/// dense input in place, any other written to \p scratch. False for a
/// length mismatch or a negative weight.
template <class View>
bool
RigExecResolveCombineWeightInput(const View &input, size_t count,
                                 std::vector<float> *scratch,
                                 const float **data, size_t *size)
{
    if (input.representation == kRigExecWeightDense) {
        if (input.valueCount != count)
            return false;
        for (size_t i = 0; i < input.valueCount; ++i)
            if (input.values[i] < 0.0f)
                return false;
        *data = input.values;
        *size = input.valueCount;
        return true;
    }
    if (input.representation != kRigExecWeightSparse) {
        if (input.defaultWeight < 0.0f)
            return false;
        scratch->assign(count, input.defaultWeight);
    } else {
        if (input.indexCount != input.valueCount)
            return false;
        scratch->assign(count, input.defaultWeight);
        const bool sorted =
            !input.indexCount ||
            std::is_sorted(input.indices, input.indices + input.indexCount);
        if (!sorted) {
            for (size_t i = 0; i < count; ++i)
                (*scratch)[i] = RigExecResolveWeightPacket(input, i, count);
        } else {
            size_t p = 0;
            for (size_t i = 0; i < count; ++i) {
                const int32_t index = int32_t(i);
                if (i && index < int32_t(i - 1))
                    p = 0;
                while (p < input.indexCount && input.indices[p] < index)
                    ++p;
                if (p < input.indexCount && input.indices[p] == index)
                    (*scratch)[i] = input.values[p++];
            }
        }
        for (float w : *scratch)
            if (w < 0.0f)
                return false;
    }
    *data = scratch->data();
    *size = scratch->size();
    return true;
}

/// The range policy's failure for a weight it refuses.
inline const char *
RigExecWeightRangeFailure(bool clamp)
{
    return clamp ? "a weight is not finite"
                 : "a weight is not finite or outside [0, 1]";
}

/// A painted packet of \p representation: \p values (and, sparse, their
/// \p indices, sorted into ascending order) and \p defaultWeight under the
/// range policy. Null on success, else why it fails.
inline const char *
RigExecStaticWeightPacketKernel(int representation, bool clamp,
                                float defaultWeight, const float *values,
                                size_t valueCount, const int32_t *indices,
                                size_t indexCount,
                                RigExecStaticWeightScratch *work,
                                RigExecWeightPacketBuild *out)
{
    out->defaultWeight = defaultWeight;
    out->values.clear();
    out->indices.clear();
    if (representation == kRigExecWeightSparse) {
        const size_t paired = std::min(valueCount, indexCount);
        auto &pairs = work->pairs;
        pairs.clear();
        for (size_t i = 0; i < paired; ++i)
            pairs.emplace_back(indices[i], values[i]);
        if (valueCount != indexCount)
            return "its values and indices differ in length";
        const bool same =
            work->support.size() == indexCount &&
            (indexCount == 0 ||
             std::equal(work->support.begin(), work->support.end(), indices));
        if (!same) {
            if (indexCount)
                work->support.assign(indices, indices + indexCount);
            else
                work->support.clear();
            work->supportOrder.resize(indexCount);
            std::iota(work->supportOrder.begin(), work->supportOrder.end(),
                      size_t(0));
            std::sort(
                work->supportOrder.begin(), work->supportOrder.end(),
                [&](size_t a, size_t b) { return indices[a] < indices[b]; });
            work->uniqueSupport = true;
            for (size_t i = 1; i < indexCount; ++i) {
                if (indices[work->supportOrder[i - 1]] ==
                    indices[work->supportOrder[i]]) {
                    work->uniqueSupport = false;
                    break;
                }
            }
        }
        if (work->uniqueSupport) {
            for (size_t i = 0; i < indexCount; ++i) {
                const size_t source = work->supportOrder[i];
                pairs[i] = {indices[source], values[source]};
            }
        } else {
            std::sort(pairs.begin(), pairs.end());
        }
        out->indices.reserve(pairs.size());
        out->values.reserve(pairs.size());
        for (size_t i = 0; i < pairs.size(); ++i) {
            if (i > 0 && pairs[i].first == pairs[i - 1].first)
                return "an index appears twice";
            out->indices.push_back(pairs[i].first);
            out->values.push_back(pairs[i].second);
        }
    } else if (valueCount) {
        out->values.assign(values, values + valueCount);
    }
    if (representation == kRigExecWeightDense && defaultWeight != 0.0f)
        return "a dense packet's default weight is not 0";
    for (float &w : out->values)
        if (!RigExecApplyWeightRangeKernel(clamp, &w))
            return RigExecWeightRangeFailure(clamp);
    if (!RigExecApplyWeightRangeKernel(clamp, &out->defaultWeight))
        return RigExecWeightRangeFailure(clamp);
    return nullptr;
}

/// A packet remapped from \p base as (w * driver) * scale + bias under the
/// range policy, or, with no base, a constant packet of driver * scale +
/// bias. Null on success, else why it fails.
template <class View>
const char *
RigExecDynamicWeightPacketKernel(int representation, bool clamp, float driver,
                                 float scale, float bias, const View *base,
                                 RigExecWeightPacketBuild *out)
{
    out->defaultWeight = 0.0f;
    out->values.clear();
    out->indices.clear();
    if (!base) {
        if (representation != kRigExecWeightConstant)
            return "a packet that is not constant needs a base packet";
        out->defaultWeight = (1.0f * driver) * scale + bias;
        if (!RigExecApplyWeightRangeKernel(clamp, &out->defaultWeight))
            return RigExecWeightRangeFailure(clamp);
        return nullptr;
    }
    if (!base->valid || base->representation != representation)
        return "its base packet is invalid or of another representation";
    if (base->indexCount)
        out->indices.assign(base->indices, base->indices + base->indexCount);
    out->values.reserve(base->valueCount);
    for (size_t i = 0; i < base->valueCount; ++i) {
        float r = (base->values[i] * driver) * scale + bias;
        if (!RigExecApplyWeightRangeKernel(clamp, &r))
            return RigExecWeightRangeFailure(clamp);
        out->values.push_back(r);
    }
    if (representation != kRigExecWeightDense) {
        out->defaultWeight = (base->defaultWeight * driver) * scale + bias;
        if (!RigExecApplyWeightRangeKernel(clamp, &out->defaultWeight))
            return RigExecWeightRangeFailure(clamp);
    }
    return nullptr;
}

/// The dense packet folding \p inputCount packets, \p inputAt(k) each, in
/// order under \p mode, then inverted and scaled, under the range policy.
/// The element count is a dense input's length, else \p targetCount. Null
/// on success, else why it fails.
template <class InputAt>
const char *
RigExecCombineWeightPacketKernel(RigExecWeightCombine mode, bool clamp,
                                 float strength, float invert,
                                 size_t inputCount, InputAt &&inputAt,
                                 size_t targetCount,
                                 std::vector<float> *scratch,
                                 RigExecWeightPacketBuild *out)
{
    out->defaultWeight = 0.0f;
    out->indices.clear();
    size_t elementCount = 0;
    for (size_t k = 0; k < inputCount; ++k) {
        const auto &in = inputAt(k);
        if (!in.valid)
            return "an input packet is invalid";
        if (in.representation == kRigExecWeightDense) {
            if (elementCount && in.valueCount != elementCount)
                return "its dense inputs differ in length";
            elementCount = in.valueCount;
        }
    }
    if (!elementCount)
        elementCount = targetCount;
    if (!elementCount)
        return "no element count is known";
    if (!RigExecCombineWeightFieldsStreamed(
            mode, inputCount, elementCount,
            [&](size_t k, const float **data, size_t *size) {
                return RigExecResolveCombineWeightInput(
                    inputAt(k), elementCount, scratch, data, size);
            },
            &out->values))
        return "an input resolves a negative weight or another length";
    for (float &w : out->values) {
        w = (w + (1.0f - 2.0f * w) * invert) * strength;
        if (!RigExecApplyWeightRangeKernel(clamp, &w))
            return RigExecWeightRangeFailure(clamp);
    }
    return nullptr;
}

} // namespace rigExec

#endif
