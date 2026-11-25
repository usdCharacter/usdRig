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

// ---------------------------------------------------------------------
// The weight-field resolution. One body: the structural
// failures first, then the combine fold / volume measurement, then the
// dynamic remap or the static table, with the range policy last.
// ---------------------------------------------------------------------

template <class Host>
bool ResolveField(Host &host, size_t id, size_t count,
                  const WeightPointViewT<typename Host::Math> *current,
                  std::vector<float> *out, std::string *error, size_t depth);

template <class Host>
bool
ResolveFieldVolume(Host &host, size_t id, size_t count,
                   const WeightPointViewT<typename Host::Math> *current,
                   std::vector<float> *weights, std::string *error,
                   size_t depth)
{
    using Math = typename Host::Math;
    const auto &f = host.inputs().weightFacts(id);
    const std::string &who = host.inputs().weightName(id);
    const auto read = [&](int member, float fallback) {
        return host.inputs().fieldScalar(id, member, fallback);
    };
    if (f.blocked) {
        *error = "operation cycle";
        return false;
    }
    if (f.kind == kWeightKindCombine) {
        if (f.combineMode == kWeightCombineUnknown) {
            *error = who + ": unknown rigExec:combineMode " +
                     host.inputs().weightCombineText(id);
            return false;
        }
        auto &child = host.state().fieldWorkspace().children[depth];
        if (!RigExecCombineWeightFieldsStreamed(
                RigExecWeightCombine(f.combineMode), size_t(f.inputCount),
                count,
                [&](size_t k, const float **data, size_t *size) {
                    if (!ResolveField(host, size_t(f.inputIndices[k]), count,
                                      current, &child, error, depth + 1)) {
                        return false;
                    }
                    *data = child.data();
                    *size = child.size();
                    return true;
                },
                weights)) {
            if (error->empty()) {
                *error = who + ": combine inputs disagree on element count";
            }
            return false;
        }
        const float strength = read(4, 1.0f);
        const float invert = read(5, 0.0f);
        for (float &w : *weights) {
            w = (w + (1.0f - 2.0f * w) * invert) * strength;
        }
        return true;
    }

    const auto *placement = host.inputs().fieldPlacement(id);
    if (!placement) {
        *error = who + ": no resolved placement for this volume weight";
        return false;
    }
    const typename Math::Mat4 rigid = Math::RemoveScaleShear(*placement);
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        *error = who + ": degenerate volume placement";
        return false;
    }
    typename Math::Mat4 worldToLocal = rigid.GetInverse();

    if (!host.inputs().weightPhaseError(id).empty()) {
        *error = host.inputs().weightPhaseError(id);
        return false;
    }
    const auto target = host.inputs().fieldPoints(id, 1, true);
    WeightPointViewT<Math> samples;
    if (target.declared) {
        if (!target.available) {
            *error = who + ": could not read the points to sample";
            return false;
        }
        samples.data = target.data;
        samples.count = target.count;
    } else if (f.samplesInFlight) {
        if (!current) {
            *error = who + ": rigExec:weightTarget reads `preceding` but no "
                           "in-flight points were supplied";
            return false;
        }
        samples = *current;
    } else {
        auto primary = host.inputs().fieldPoints(id, 0, true);
        if (!primary.declared)
            primary = host.inputs().fieldPoints(id, 0, false);
        if (!primary.available)
            primary = host.inputs().fieldPoints(id, 1, false);
        if (!primary.available) {
            *error = who + ": could not read the points to sample";
            return false;
        }
        samples = {primary.data, primary.count};
    }
    if (samples.count != count) {
        *error = who + ": sampled point count does not match the target";
        return false;
    }

    WeightFalloffFacts params;
    params.falloffMin = read(6, 0.0f);
    params.falloffMax = read(7, 1.0f);
    params.invert = read(5, 0.0f);
    params.strength = read(4, 1.0f);
    const auto falloff = host.inputs().fieldFalloff(id);
    params.curve = falloff.second ? falloff.first : f.falloffCurve;
    params.curveCount = falloff.second ? falloff.second : f.falloffCurveCount;
    if (f.kind == kWeightKindPlane) {
        const auto &axisName = host.inputs().fieldPlaneToken(id, false);
        const auto &boundsName = host.inputs().fieldPlaneToken(id, true);
        const int axis = WeightAxis(axisName);
        if (axis == kWeightPlaneAxisUnknown) {
            *error = who + ": unknown rigExec:planeAxis " + axisName;
            return false;
        }
        float extentU = 1.0f, extentV = 1.0f;
        bool bounded = false;
        if (boundsName == SchemaValueText::bounded) {
            extentU = read(17, 1.0f);
            extentV = read(18, 1.0f);
            if (!RigExecVolumeExtentsOk(who, extentU, extentV, error)) {
                return false;
            }
            bounded = true;
        } else if (boundsName != SchemaValueText::unbounded) {
            *error = RigExecUnknownPlaneBoundsMessage(who, boundsName);
            return false;
        }
        PlaneWeightField<Math>(samples, worldToLocal, axis, params, weights,
                               extentU, extentV, bounded, host.ranges());
        return true;
    }
    const float sx = read(8, 1.0f);
    const float sy = read(9, 1.0f);
    const float sz = read(10, 1.0f);
    if (!RigExecVolumeAxisScalesOk(who, sx, sy, sz, error)) {
        return false;
    }
    worldToLocal =
        ApplyAxisScales<Math>(worldToLocal, typename Math::Vec3f(sx, sy, sz));
    if (f.kind == kWeightKindSphere) {
        const typename Math::Vec3f positive(read(11, 1.0f), read(12, 1.0f),
                                            read(13, 1.0f));
        const typename Math::Vec3f negative(read(14, 1.0f), read(15, 1.0f),
                                            read(16, 1.0f));
        if (!RigExecSignedAxisScalesOk(who, positive, negative, error)) {
            return false;
        }
        SphereWeightField<Math>(samples, worldToLocal, params, weights,
                                positive, negative, host.ranges());
        return true;
    }
    // Curve.
    auto chosenCurve = host.inputs().fieldPoints(id, 2, true);
    if (!chosenCurve.declared)
        chosenCurve = host.inputs().fieldPoints(id, 2, false);
    if (!chosenCurve.available || !chosenCurve.count) {
        *error = who + ": rigExec:curve must name exactly one points source";
        return false;
    }
    CurveWeightField<Math>(
        samples, WeightPointViewT<Math>{chosenCurve.data, chosenCurve.count},
        worldToLocal, params, weights, host.state().fieldCurveScratch(),
        host.ranges());
    return true;
}

template <class Host>
bool
ResolveField(Host &host, size_t id, size_t count,
             const WeightPointViewT<typename Host::Math> *current,
             std::vector<float> *out, std::string *error, size_t depth)
{
    if (id >= host.state().weightObjectCount()) {
        *error = "weight input unavailable";
        return false;
    }
    const auto &f = host.inputs().weightFacts(id);
    const std::string &who = host.inputs().weightName(id);
    // The graph's cycle gate closes this object before anything else:
    // the packet producer is set aside, so its field has no answer.
    if (f.blocked && f.blockedBeforeStaticError) {
        *error = "operation cycle";
        return false;
    }
    // The structural failures (a missing prim, an unknown type, combine
    // mode or range policy, the dynamic descriptor checks, a static field
    // with samples or connections) precede every read and every
    // recursion of this object.
    if (!host.inputs().weightStaticError(id).empty()) {
        *error = host.inputs().weightStaticError(id);
        return false;
    }
    if (f.kind == kWeightKindUnknown) {
        *error = std::string("unknown weight object type ") +
                 host.inputs().weightTypeText(id) + " on " + who;
        return false;
    }

    if (f.kind == kWeightKindCombine || f.kind == kWeightKindSphere ||
        f.kind == kWeightKindPlane || f.kind == kWeightKindCurve) {
        if (!ResolveFieldVolume(host, id, count, current, out, error, depth)) {
            return false;
        }
        // Anything but clamp is strict here.
        for (float &w : *out) {
            if (!std::isfinite(w)) {
                *error = "non-finite weight on " + who;
                return false;
            }
            if (w < 0.0f || w > 1.0f) {
                if (!f.clamp) {
                    *error = "strict range violation on " + who;
                    return false;
                }
                w = std::min(std::max(w, 0.0f), 1.0f);
            }
        }
        return true;
    }

    const float defaultWeight = host.inputs().fieldScalar(id, 0, 0.0f);
    if (!f.strict && !f.clamp) {
        *error = "unknown rangePolicy on " + who;
        return false;
    }

    if (f.kind == kWeightKindDynamic) {
        // A sparse dynamic weight that paints its own support must paint
        // the base's. A mismatch of the authored arrays is the Build-time
        // static error; arrays bound as inputs are checked as they stand
        // this run.
        if (f.checkDynamicSupport && f.base >= 0 &&
            f.representation == kWeightSparse) {
            if (!host.inputs().dynamicSupportMatches(id, size_t(f.base))) {
                *error = "dynamic/base sparse support mismatch on " + who;
                return false;
            }
        }
        // Base field first (forwarding the in-flight points), then
        // r_i = (b_i * d) * s + a.
        if (f.base >= 0) {
            if (!ResolveField(host, size_t(f.base), count, current, out, error,
                              depth)) {
                return false;
            }
        } else {
            out->assign(count, 1.0f);
        }
        const float driver = host.inputs().fieldScalar(id, 1, 1.0f);
        const float scale = host.inputs().fieldScalar(id, 2, 1.0f);
        const float bias = host.inputs().fieldScalar(id, 3, 0.0f);
        for (size_t i = 0; i < count; ++i) {
            float r = ((*out)[i] * driver) * scale + bias;
            if (!std::isfinite(r)) {
                *error = "non-finite dynamic weight on " + who;
                return false;
            }
            if (r < 0.0f || r > 1.0f) {
                if (f.clamp) {
                    r = std::min(std::max(r, 0.0f), 1.0f);
                } else {
                    *error = "strict range violation on " + who;
                    return false;
                }
            }
            (*out)[i] = r;
        }
        return true;
    }

    // Static: the painted table, the adapter's views this run.
    const auto [values, valueCount] = host.inputs().fieldValues(id);
    if (f.representation == kWeightConstant) {
        if (valueCount != 0) {
            *error = "constant weight must not author values on " + who;
            return false;
        }
        out->assign(count, defaultWeight);
    } else if (f.representation == kWeightDense) {
        if (valueCount != count) {
            *error = "dense weight cardinality mismatch on " + who;
            return false;
        }
        if (defaultWeight != 0.0f) {
            *error =
                "dense weight requires canonical defaultWeight 0 on " + who;
            return false;
        }
        if (count) {
            out->assign(values, values + count);
        } else {
            out->clear();
        }
    } else if (f.representation == kWeightSparse) {
        const auto [indices, indexCount] = host.inputs().fieldIndices(id);
        if (indexCount != valueCount) {
            *error = "sparse index/value size mismatch on " + who;
            return false;
        }
        out->assign(count, defaultWeight);
        auto &seen = host.state().fieldWorkspace().seen;
        seen.resize(count, 0);
        ++host.state().fieldWorkspace().seenGeneration;
        if (host.state().fieldWorkspace().seenGeneration == 0) {
            std::fill(seen.begin(), seen.end(), 0);
            host.state().fieldWorkspace().seenGeneration = 1;
        }
        const uint32_t generation =
            host.state().fieldWorkspace().seenGeneration;
        for (size_t i = 0; i < indexCount; ++i) {
            if (indices[i] < 0 || size_t(indices[i]) >= count) {
                *error = "sparse index out of range on " + who;
                return false;
            }
            if (seen[size_t(indices[i])] == generation) {
                *error = "duplicate sparse index on " + who;
                return false;
            }
            seen[size_t(indices[i])] = generation;
            (*out)[size_t(indices[i])] = values[i];
        }
    } else {
        *error = "unknown weight representation on " + who;
        return false;
    }
    // The baked side sets strict for every non-clamp policy, so this is the
    // baked `!clamp` check there and the runtime `strict` check on playback:
    // an unknown policy fails a static table on the baked side and passes
    // it through (with the unknown-policy error already set) on playback.
    for (float &w : *out) {
        if (!std::isfinite(w) || (f.strict && (w < 0.0f || w > 1.0f))) {
            *error = "weight range violation on " + who;
            return false;
        }
        if (f.clamp) {
            w = std::min(std::max(w, 0.0f), 1.0f);
        }
    }
    return true;
}

} // namespace weights_detail

// Resolve one weight field: the shared entry both adapters run for the
// WeightField kind. `root` names the field's weight object; the returned
// `ok` mirrors each adapter's field-result semantics (a failure leaves
// `result` empty and `error` non-empty).
template <class Host>
bool
RunWeightField(Host &host, size_t root, size_t count,
               const WeightPointViewT<typename Host::Math> *current,
               std::vector<float> *result, std::string *error)
{
    static_assert(IsBoundHost<Host>::value,
                  "weights family: Host must bind host.h's contract");
    error->clear();
    const size_t objectCount = host.state().weightObjectCount();
    if (root >= objectCount) {
        result->clear();
        *error = "weight input unavailable";
        return false;
    }
    auto &workspace = host.state().fieldWorkspace();
    workspace.children.resize(objectCount + 1);
    const bool ok = weights_detail::ResolveField(host, root, count, current,
                                                 result, error, 0);
    if (!ok) {
        result->clear();
        if (error->empty()) {
            *error = "weight input unavailable";
        }
    }
    return ok;
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_WEIGHTS_H
