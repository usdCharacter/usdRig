// Shared weight field bodies. Hosts bind the existing producer slots, borrowed
// input views, and exclusive workspaces; weight packets are op nodes.
#ifndef RIGEXEC_EVAL_CORE_WEIGHTS_H
#define RIGEXEC_EVAL_CORE_WEIGHTS_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <limits>
#include <numeric>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "rigExecEvalCore/host.h"
#include "rigExecEvalCore/kinds.h"
#include "rigExecEvalCore/step.h"
#include "rigExecMath/rangeRunner.h"
#include "rigExecMath/spatialAccel.h"
#include "rigExecMath/volumeFieldCheck.h"
#include "rigExecMath/weightFieldKernel.h"
#include "rigExecSchema/schemaNames.h"

namespace rigExec {
namespace eval {

// The weight-object kind, representation, range policy, combine mode,
// plane axis and plane bounds as the integer codes both adapters already
// classify to at Build/Open (the baked weightProgram record's kind and
// representation codes, the runtime's RrWeightFieldKind): the bodies
// never compare token text.
enum WeightKindCode : int {
    kWeightKindUnknown = -1,
    kWeightKindStatic = 0,
    kWeightKindDynamic = 1,
    kWeightKindCombine = 2,
    kWeightKindSphere = 3,
    kWeightKindPlane = 4,
    kWeightKindCurve = 5
};
enum WeightRepresentation : int {
    kWeightRepresentationUnknown = -1,
    kWeightConstant = 0,
    kWeightDense = 1,
    kWeightSparse = 2
};
enum WeightRangePolicy : int {
    kWeightPolicyUnknown = -1,
    kWeightPolicyStrict = 0,
    kWeightPolicyClamp = 1
};
enum WeightCombineCode : int {
    kWeightCombineUnknown = -1,
    kWeightCombineMultiply = 0,
    kWeightCombineAdd = 1,
    kWeightCombineSubtract = 2,
    kWeightCombineMax = 3,
    kWeightCombineMin = 4,
    kWeightCombineAverage = 5,
    kWeightCombineOverlay = 6
};
enum WeightPlaneAxisCode : int {
    kWeightPlaneAxisUnknown = -1,
    kWeightPlaneAxisX = 0,
    kWeightPlaneAxisY = 1,
    kWeightPlaneAxisZ = 2
};
enum WeightPlaneBoundsCode : int {
    kWeightBoundsUnknown = -1,
    kWeightBoundsBounded = 0,
    kWeightBoundsUnbounded = 1
};

struct WeightObjectFacts {
    int kind = kWeightKindUnknown;
    int representation = kWeightRepresentationUnknown;
    int rangePolicy = kWeightPolicyUnknown;
    int combineMode = kWeightCombineUnknown;
    int planeAxis = kWeightPlaneAxisUnknown;
    int planeBounds = kWeightBoundsUnknown;
    int base = -1;
    bool clamp = false;
    bool strict = false;
    bool samplesInFlight = false;
    bool blocked = false;
    bool blockedBeforeStaticError = false;
    bool checkDynamicSupport = false;
    size_t inputCount = 0;
    const int32_t *inputIndices = nullptr;
    const float *falloffCurve = nullptr;
    size_t falloffCurveCount = 0;
};

inline int
WeightAxis(std::string_view axis)
{
    return axis == SchemaValueText::x   ? kWeightPlaneAxisX
           : axis == SchemaValueText::y ? kWeightPlaneAxisY
           : axis == SchemaValueText::z ? kWeightPlaneAxisZ
                                        : kWeightPlaneAxisUnknown;
}
inline int
WeightBounds(std::string_view bounds)
{
    return bounds == SchemaValueText::bounded     ? kWeightBoundsBounded
           : bounds == SchemaValueText::unbounded ? kWeightBoundsUnbounded
                                                  : kWeightBoundsUnknown;
}

// The band and LUT a volumetric field remaps through (the FalloffParams
// of the adapters' shared math, in plain pointers).
struct WeightFalloffFacts {
    float falloffMin = 0.0f;
    float falloffMax = 1.0f;
    float invert = 0.0f;
    float strength = 1.0f;
    const float *curve = nullptr;
    size_t curveCount = 0;
};

// A borrowed point run in the host's point type.
template <class Math> struct WeightPointViewT {
    const typename Math::Vec3f *data = nullptr;
    size_t count = 0;
};

template <class Math> struct WeightFieldPoint {
    bool declared, available;
    const typename Math::Vec3f *data;
    size_t count;
};

namespace weights_detail {

constexpr size_t kFieldGrain = 1024;

inline float
Clamp01(float x)
{
    return std::min(1.0f, std::max(0.0f, x));
}

inline float
SampleFalloffLut(const float *curve, size_t count, float r)
{
    if (count < 2) {
        return r;
    }
    const float x = Clamp01(r) * float(count - 1);
    const size_t i = std::min(count - 2, size_t(std::floor(x)));
    const float t = x - float(i);
    return curve[i] + (curve[i + 1] - curve[i]) * t;
}

inline float
EvaluateFalloff(float distance, const WeightFalloffFacts &p)
{
    const float span = p.falloffMax - p.falloffMin;
    float u;
    if (std::abs(span) <= std::numeric_limits<float>::min()) {
        u = distance < p.falloffMin ? 0.0f : 1.0f;
    } else {
        u = Clamp01((distance - p.falloffMin) / span);
    }
    u = u + (1.0f - 2.0f * u) * p.invert;
    const float w = SampleFalloffLut(p.curve, p.curveCount, 1.0f - u);
    return w * p.strength;
}

template <class Math>
typename Math::Vec3f
ToLocal(const typename Math::Mat4 &worldToLocal, const typename Math::Vec3f &p)
{
    return Math::ToFloat(worldToLocal.TransformAffine(Math::ToDouble(p)));
}

template <class Math>
inline float
SegmentDistance(const typename Math::Vec3f &p, const typename Math::Vec3f &a,
                const typename Math::Vec3f &b)
{
    const typename Math::Vec3f ab = b - a;
    const float lengthSq = ab.GetLengthSq();
    if (lengthSq <= 1e-20f) {
        return (p - a).GetLength();
    }
    const float t = Clamp01(Math::Dot(p - a, ab) / lengthSq);
    return (p - (a + ab * t)).GetLength();
}

template <class Math>
inline float
CurveDistance(const typename Math::Vec3f &p,
              const typename Math::Vec3f *curvePoints, size_t count)
{
    if (count == 0 || !curvePoints) {
        return std::numeric_limits<float>::infinity();
    }
    if (count == 1) {
        return (p - curvePoints[0]).GetLength();
    }
    float best = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i + 1 < count; ++i) {
        best = std::min(
            best, SegmentDistance<Math>(p, curvePoints[i], curvePoints[i + 1]));
    }
    return best;
}

// The three volumetric field kernels. Same arithmetic as
// rigExecMath/weightFields.cpp (which stays the Gf-shaped entry for the
// OpenExec registrations): identity landmarks are irrelevant here, the
// matrix is already world-to-volume.
template <class Math>
void
SphereWeightField(WeightPointViewT<Math> points,
                  const typename Math::Mat4 &worldToLocal,
                  const WeightFalloffFacts &params, std::vector<float> *weights,
                  const typename Math::Vec3f &positiveScales,
                  const typename Math::Vec3f &negativeScales,
                  const RigExecRangeRunner *ranges)
{
    using Vec = typename Math::Vec3f;
    weights->resize(points.count);
    float *const out = weights->data();
    RigExecRunRanges(
        ranges, points.count, kFieldGrain, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                Vec local = ToLocal<Math>(worldToLocal, points.data[i]);
                for (int axis = 0; axis < 3; ++axis) {
                    local[axis] /= local[axis] < 0.0f ? negativeScales[axis]
                                                      : positiveScales[axis];
                }
                const float d = local.GetLength();
                out[i] = EvaluateFalloff(d, params);
            }
        });
}

template <class Math>
void
PlaneWeightField(WeightPointViewT<Math> points,
                 const typename Math::Mat4 &worldToLocal, int axis,
                 const WeightFalloffFacts &params, std::vector<float> *weights,
                 float extentU, float extentV, bool bounded,
                 const RigExecRangeRunner *ranges)
{
    using Vec = typename Math::Vec3f;
    weights->resize(points.count);
    float *const out = weights->data();
    RigExecRunRanges(
        ranges, points.count, kFieldGrain, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                const Vec local = ToLocal<Math>(worldToLocal, points.data[i]);
                if (bounded) {
                    const float u = local[size_t((axis + 1) % 3)];
                    const float v = local[size_t((axis + 2) % 3)];
                    if (!(std::abs(u) <= extentU && std::abs(v) <= extentV)) {
                        out[i] = 0.0f;
                        continue;
                    }
                }
                const float d =
                    (axis >= 0 && axis <= 2) ? local[size_t(axis)] : 0.0f;
                out[i] = EvaluateFalloff(d, params);
            }
        });
}

template <class Math>
void
CurveWeightField(WeightPointViewT<Math> points,
                 WeightPointViewT<Math> curvePoints,
                 const typename Math::Mat4 &worldToLocal,
                 const WeightFalloffFacts &params, std::vector<float> *weights,
                 std::vector<typename Math::Vec3f> *localCurveScratch,
                 const RigExecRangeRunner *ranges)
{
    using Vec = typename Math::Vec3f;
    weights->resize(points.count);
    if (curvePoints.count == 0) {
        std::fill(weights->begin(), weights->end(), 0.0f);
        return;
    }
    std::vector<Vec> scratchOwned;
    std::vector<Vec> &localCurve =
        localCurveScratch ? *localCurveScratch : scratchOwned;
    localCurve.resize(curvePoints.count);
    for (size_t k = 0; k < curvePoints.count; ++k) {
        localCurve[k] = ToLocal<Math>(worldToLocal, curvePoints.data[k]);
    }
    if (localCurve.size() - 1 >= kRigExecBvhMinSegments &&
        points.count >= kRigExecBvhMinQueries) {
        RigExecSegmentBvh<Vec> bvh;
        if (bvh.Build(localCurve.data(), localCurve.size())) {
            float *const out = weights->data();
            RigExecRunRanges(
                ranges, points.count, kFieldGrain,
                [&](size_t begin, size_t end) {
                    for (size_t i = begin; i < end; ++i) {
                        const float d = bvh.QueryNearest(
                            ToLocal<Math>(worldToLocal, points.data[i]),
                            localCurve.data(),
                            [](const Vec &p, const Vec &a, const Vec &b) {
                                return SegmentDistance<Math>(p, a, b);
                            });
                        out[i] = EvaluateFalloff(d, params);
                    }
                });
            return;
        }
    }
    float *const out = weights->data();
    RigExecRunRanges(ranges, points.count, kFieldGrain,
                     [&](size_t begin, size_t end) {
                         for (size_t i = begin; i < end; ++i) {
                             const float d = CurveDistance<Math>(
                                 ToLocal<Math>(worldToLocal, points.data[i]),
                                 localCurve.data(), localCurve.size());
                             out[i] = EvaluateFalloff(d, params);
                         }
                     });
}

template <class Math>
typename Math::Mat4
ApplyAxisScales(const typename Math::Mat4 &worldToLocal,
                const typename Math::Vec3f &scales)
{
    typename Math::Mat4 divide(1.0);
    divide[0][0] = 1.0 / double(scales[0]);
    divide[1][1] = 1.0 / double(scales[1]);
    divide[2][2] = 1.0 / double(scales[2]);
    return worldToLocal * divide;
}

} // namespace weights_detail

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_WEIGHTS_H
