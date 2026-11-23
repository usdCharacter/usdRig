// RigExec volumetric weight-field kernels implementation.
#include "weightFields.h"
#include "spatialAccel.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rigExec {

std::vector<float>
RigExecBuildFalloffLut(RigExecFalloffProfile profile, size_t count)
{
    if (count < 2) {
        return {};
    }
    std::vector<float> lut(count);
    for (size_t i = 0; i < count; ++i) {
        const float r = float(i) / float(count - 1);
        switch (profile) {
        case RigExecFalloffProfile::Linear:
            lut[i] = r;
            break;
        case RigExecFalloffProfile::Smooth:
            lut[i] = r * r * (3.0f - 2.0f * r);
            break;
        case RigExecFalloffProfile::EaseIn:
            lut[i] = r * r;
            break;
        case RigExecFalloffProfile::EaseOut:
            lut[i] = 1.0f - (1.0f - r) * (1.0f - r);
            break;
        case RigExecFalloffProfile::Constant:
            lut[i] = r > 0.0f ? 1.0f : 0.0f;
            break;
        }
    }
    return lut;
}

float
RigExecSampleFalloffLut(const std::vector<float> &curve, float r)
{
    return RigExecSampleFalloffLut(curve.data(), curve.size(), r);
}

float
RigExecSampleFalloffLut(const float *curve, size_t count, float r)
{
    return RigExecSampleFalloffLutKernel(curve, count, r);
}

namespace {

RigExecFalloffKernelParams
_KernelParams(const RigExecFalloffParams &p)
{
    RigExecFalloffKernelParams k;
    k.falloffMin = p.falloffMin;
    k.falloffMax = p.falloffMax;
    k.invert = p.invert;
    k.strength = p.strength;
    k.curve = p.curveData ? p.curveData : p.curve.data();
    k.curveCount = p.curveData ? p.curveCount : p.curve.size();
    return k;
}

} // namespace

float
RigExecEvaluateFalloff(float distance, const RigExecFalloffParams &p)
{
    return RigExecEvaluateFalloffKernel(distance, _KernelParams(p));
}

float
RigExecSphereDistance(const GfVec3f &p)
{
    return p.GetLength();
}

float
RigExecPlaneDistance(const GfVec3f &p, int axis)
{
    if (axis < 0 || axis > 2) {
        return 0.0f;
    }
    return p[axis];
}

bool
RigExecPlaneWithinBounds(const GfVec3f &p, int axis,
                         const RigExecPlaneBounds &bounds)
{
    return RigExecPlaneWithinBoundsKernel(p, axis, bounds.extentU,
                                          bounds.extentV);
}

float
RigExecSegmentDistance(const GfVec3f &p, const GfVec3f &a, const GfVec3f &b)
{
    return RigExecSegmentDistanceKernel(p, a, b);
}

float
RigExecCurveDistance(const GfVec3f &p, const GfVec3f *curvePoints, size_t count)
{
    return RigExecCurveDistanceKernel(p, curvePoints, count);
}

namespace {

inline GfVec3f
_ToLocal(const GfMatrix4d &worldToLocal, const GfVec3f &p)
{
    return RigExecWeightToLocal<GfVec3f, GfVec3d>(worldToLocal, p);
}

// Points per slice when a host splits a field's point loop.
constexpr size_t kRigExecFieldGrain = 1024;

} // namespace

void
RigExecSphereWeightField(const std::vector<GfVec3f> &points,
                         const GfMatrix4d &matrix,
                         const RigExecFalloffParams &params,
                         std::vector<float> *weights, const GfVec3f &positive,
                         const GfVec3f &negative)
{
    RigExecSphereWeightField({points.data(), points.size()}, matrix, params,
                             weights, positive, negative);
}
void
RigExecPlaneWeightField(const std::vector<GfVec3f> &points,
                        const GfMatrix4d &matrix, int axis,
                        const RigExecFalloffParams &params,
                        std::vector<float> *weights,
                        const RigExecPlaneBounds *bounds)
{
    RigExecPlaneWeightField({points.data(), points.size()}, matrix, axis,
                            params, weights, bounds);
}
void
RigExecCurveWeightField(const std::vector<GfVec3f> &points,
                        const std::vector<GfVec3f> &curve,
                        const GfMatrix4d &matrix,
                        const RigExecFalloffParams &params,
                        std::vector<float> *weights)
{
    std::vector<GfVec3f> localCurve;
    RigExecCurveWeightField({points.data(), points.size()},
                            {curve.data(), curve.size()}, matrix, params,
                            weights, &localCurve);
}

void
RigExecSphereWeightField(RigExecWeightPointView points,
                         const GfMatrix4d &worldToLocal,
                         const RigExecFalloffParams &params,
                         std::vector<float> *weights,
                         const GfVec3f &positiveScales,
                         const GfVec3f &negativeScales,
                         const RigExecRangeRunner *ranges)
{
    weights->resize(points.count);
    float *const out = weights->data();
    const RigExecFalloffKernelParams k = _KernelParams(params);
    RigExecRunRanges(ranges, points.count, kRigExecFieldGrain,
                     [&](size_t begin, size_t end) {
                         RigExecSphereWeightFieldKernel<GfVec3f, GfVec3d>(
                             points.data, begin, end, worldToLocal, k,
                             positiveScales, negativeScales, out);
                     });
}

void
RigExecPlaneWeightField(RigExecWeightPointView points,
                        const GfMatrix4d &worldToLocal, int axis,
                        const RigExecFalloffParams &params,
                        std::vector<float> *weights,
                        const RigExecPlaneBounds *bounds,
                        const RigExecRangeRunner *ranges)
{
    weights->resize(points.count);
    float *const out = weights->data();
    const RigExecFalloffKernelParams k = _KernelParams(params);
    RigExecRunRanges(ranges, points.count, kRigExecFieldGrain,
                     [&](size_t begin, size_t end) {
                         RigExecPlaneWeightFieldKernel<GfVec3f, GfVec3d>(
                             points.data, begin, end, worldToLocal, axis, k,
                             bounds != nullptr, bounds ? bounds->extentU : 0.0f,
                             bounds ? bounds->extentV : 0.0f, out);
                     });
}

void
RigExecCurveWeightField(RigExecWeightPointView points,
                        RigExecWeightPointView curvePoints,
                        const GfMatrix4d &worldToLocal,
                        const RigExecFalloffParams &params,
                        std::vector<float> *weights,
                        std::vector<GfVec3f> *curveScratch,
                        const RigExecRangeRunner *ranges)
{
    weights->resize(points.count);
    if (curvePoints.count == 0) {
        // No curve is an empty field, not an identity one: an author who
        // loses the curve target should see the influence vanish rather
        // than silently get full weight everywhere.
        std::fill(weights->begin(), weights->end(), 0.0f);
        return;
    }
    // The curve is transformed once, not per point.
    auto &localCurve = *curveScratch;
    localCurve.resize(curvePoints.count);
    for (size_t k = 0; k < curvePoints.count; ++k) {
        localCurve[k] = _ToLocal(worldToLocal, curvePoints.data[k]);
    }
    if (localCurve.size() - 1 >= kRigExecBvhMinSegments &&
        points.count >= kRigExecBvhMinQueries) {
        RigExecSegmentBvh<GfVec3f> bvh;
        if (bvh.Build(localCurve.data(), localCurve.size())) {
            // QueryNearest is const and keeps its stack local, so the
            // queries may run concurrently.
            float *const out = weights->data();
            const RigExecFalloffKernelParams k = _KernelParams(params);
            RigExecRunRanges(
                ranges, points.count, kRigExecFieldGrain,
                [&](size_t begin, size_t end) {
                    RigExecCurveWeightFieldKernel<GfVec3f, GfVec3d>(
                        points.data, begin, end, worldToLocal, k,
                        [&](const GfVec3f &local) {
                            return bvh.QueryNearest(local, localCurve.data(),
                                                    RigExecSegmentDistance);
                        },
                        out);
                });
            return;
        }
    }
    float *const out = weights->data();
    const RigExecFalloffKernelParams k = _KernelParams(params);
    RigExecRunRanges(ranges, points.count, kRigExecFieldGrain,
                     [&](size_t begin, size_t end) {
                         RigExecCurveWeightFieldKernel<GfVec3f, GfVec3d>(
                             points.data, begin, end, worldToLocal, k,
                             [&](const GfVec3f &local) {
                                 return RigExecCurveDistanceKernel(
                                     local, localCurve.data(),
                                     localCurve.size());
                             },
                             out);
                     });
}

bool
RigExecCombineWeightFields(RigExecWeightCombine mode,
                           const std::vector<std::vector<float>> &inputs,
                           size_t elementCount, std::vector<float> *out)
{
    // Single definition: the streaming core owns the seeding, order, and
    // average rules, so this entry point cannot drift from it. The
    // inputs are already dense, so the core folds straight from them
    // with no per-input copy.
    return RigExecCombineWeightFieldsStreamed(
        mode, inputs.size(), elementCount,
        [&](size_t k, const float **data, size_t *size) {
            *data = inputs[k].data();
            *size = inputs[k].size();
            return true;
        },
        out);
}

} // namespace rigExec
