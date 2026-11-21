// RigExec geometry mover kernels implementation.
#include "geometryKernels.h"
#include "deltaMushKernel.h"
#include "latticeKernel.h"
#include "meshBoundsKernel.h"
#include "smoothKernel.h"
#include "surfaceProjectKernel.h"
#include "spatialAccel.h"
#include "wireBasisKernel.h"
#include "wrinkleKernel.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/rotation.h"

#include <atomic>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <utility>

namespace rigExec {

double
RigExecBoundVolume(const GfVec3f *points, size_t count)
{
    return RigExecBoundVolumeKernel(points, count);
}

void
RigExecApplyVolumeCorrect(std::vector<GfVec3f> *points, double referenceVolume,
                          double strength)
{
    RigExecApplyVolumeCorrectKernel(points, referenceVolume, strength);
}

bool
RigExecTransportSurfaceOffsets(const std::vector<GfVec3f> &rest,
                               const std::vector<GfVec3f> &posed,
                               const std::vector<int> &faceCounts,
                               const std::vector<int> &faceIndices,
                               const std::vector<GfVec3f> &deltas,
                               std::vector<GfVec3f> *out)
{
    return RigExecTransportSurfaceOffsetsKernel<GfVec3f, GfVec3d>(
        rest, posed, faceCounts, faceIndices, deltas, out);
}

bool
RigExecRaycastSurface(const std::vector<GfVec3f> &points,
                      const std::vector<int> &faceVertexCounts,
                      const std::vector<int> &faceVertexIndices,
                      const GfVec3d &origin, const GfVec3d &direction,
                      RigExecSurfaceHit *hit)
{
    return RigExecRaycastSurfaceT(points, faceVertexCounts, faceVertexIndices,
                                  origin, direction, hit);
}

bool
RigExecSurfaceFrameAtHit(const std::vector<GfVec3f> &points,
                         const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices,
                         const RigExecSurfaceHit &hit, const GfVec3d &upHint,
                         GfMatrix4d *frame)
{
    return RigExecSurfaceFrameAtHitT(
        points, faceVertexCounts, faceVertexIndices, hit, upHint,
        static_cast<RigExecVertexNormalsFn>(&RigExecComputeVertexNormals),
        frame);
}

bool
RigExecRaycastSurfaceFrame(const std::vector<GfVec3f> &points,
                           const std::vector<int> &faceVertexCounts,
                           const std::vector<int> &faceVertexIndices,
                           const GfVec3d &origin, const GfVec3d &direction,
                           const GfVec3d &upHint, GfMatrix4d *frame)
{
    if (!frame)
        return false;
    RigExecSurfaceHit hit;
    if (!RigExecRaycastSurface(points, faceVertexCounts, faceVertexIndices,
                               origin, direction, &hit)) {
        return false;
    }
    return RigExecSurfaceFrameAtHit(points, faceVertexCounts, faceVertexIndices,
                                    hit, upHint, frame);
}

RigExecPartialDecomposition
RigExecDecomposePartialTransform(const GfMatrix4d &transform)
{
    RigExecPartialDecomposition d;
    // Row lengths are the scale; dividing them out leaves the rotation.
    GfMatrix4d basis = transform;
    basis.SetTranslateOnly(GfVec3d(0));
    for (int axis = 0; axis < 3; ++axis) {
        const GfVec3d row(basis.GetRow3(axis));
        const double length = row.GetLength();
        if (length > 1e-12) {
            d.scale[axis] = length;
            basis.SetRow3(axis, row / length);
        }
    }

    // The same axis, a fraction of the angle, ABOUT THE SAME PIVOT. The
    // last part is what makes this radial rather than a rotation with a
    // separately-faded slide bolted on.
    //
    // A cluster's transform rotates about a pivot that is nowhere near
    // the origin -- a lid turning about an eyeball sits 165 units up --
    // and a rotation about a distant pivot carries an enormous
    // translation, P - R P. Scaling THAT by w while turning by w*theta
    // does not put the point on the arc at all: it lands inside, so a
    // half-weighted lid pulls back into the eye instead of sweeping
    // round it.
    //
    // So recover the pivot the transform actually turns about -- its
    // screw axis -- and turn a fraction about that.
    const GfRotation rotation = basis.ExtractRotation();
    d.axis = rotation.GetAxis().GetNormalized();
    d.angle = rotation.GetAngle();
    // Matrix extraction may choose either quaternion sign. Fractional
    // weights must follow the same shortest arc for both representations.
    if (d.angle > 180.0)
        d.angle -= 360.0;
    d.translation = transform.ExtractTranslation();

    // Below about a tenth of a degree there is no meaningful axis to
    // turn about and the chord and the arc agree to within float noise,
    // so the straight blend is both correct and better conditioned.
    const double radians = GfDegreesToRadians(d.angle);
    d.smallAngle = std::abs(std::sin(0.5 * radians)) < 1e-4;
    if (d.smallAngle) {
        return d;
    }

    // Split the translation into the part along the axis (a screw's own
    // travel, which stays linear in w) and the part across it, which is
    // (I - R) applied to the pivot and so names where the pivot is.
    d.along = GfDot(d.translation, d.axis);
    const GfVec3d across = d.translation - d.axis * d.along;
    const double half = 0.5 / std::tan(0.5 * radians);
    d.pivot = across * 0.5 + GfCross(d.axis, across) * half;
    return d;
}

GfMatrix4d
RigExecApplyPartialDecomposition(const RigExecPartialDecomposition &d, double w)
{
    GfMatrix4d scaled(1.0);
    scaled.SetScale(GfVec3d(1.0 + (d.scale[0] - 1.0) * w,
                            1.0 + (d.scale[1] - 1.0) * w,
                            1.0 + (d.scale[2] - 1.0) * w));

    GfMatrix4d out(1.0);
    out.SetRotate(GfRotation(d.axis, d.angle * w));
    out = scaled * out;

    if (d.smallAngle) {
        out.SetTranslateOnly(d.translation * w);
        return out;
    }

    // p' = R_w (p - pivot) + pivot + w * along * axis.
    const GfMatrix4d partial = out;
    out.SetTranslateOnly(d.pivot - partial.TransformDir(d.pivot) +
                         d.axis * (d.along * w));
    return out;
}

GfMatrix4d
RigExecPartialTransform(const GfMatrix4d &transform, double weight)
{
    const double w = GfClamp(weight, 0.0, 1.0);
    if (w <= 0.0) {
        return GfMatrix4d(1.0);
    }
    if (w >= 1.0) {
        return transform;
    }
    return RigExecApplyPartialDecomposition(
        RigExecDecomposePartialTransform(transform), w);
}

void
RigExecApplyLaplacianSmoothWithAdjacency(std::vector<GfVec3f> *points,
                                         const RigExecMeshAdjacency &adjacency,
                                         double strength, const GfVec3f *source,
                                         size_t sourceCount)
{
    RigExecApplyLaplacianSmoothWithAdjacencyKernel(points, adjacency, strength,
                                                   source, sourceCount);
}

void
RigExecApplyLaplacianSmooth(std::vector<GfVec3f> *points,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize, double strength,
                            const GfVec3f *source, size_t sourceCount)
{
    RigExecApplyLaplacianSmoothMeshKernel(
        points, faceVertexCounts, faceVertexCountsSize, faceVertexIndices,
        faceVertexIndicesSize, strength, source, sourceCount);
}

void
RigExecApplyLaplacianSmooth(std::vector<GfVec3f> *points,
                            const std::vector<int> &faceVertexCounts,
                            const std::vector<int> &faceVertexIndices,
                            double strength, const GfVec3f *source,
                            size_t sourceCount)
{
    RigExecApplyLaplacianSmooth(
        points, faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), strength, source,
        sourceCount);
}

bool
RigExecApplyDeltaMush(
    std::vector<GfVec3f> *points, const std::vector<GfVec3f> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders, double distanceWeight,
    double displacement, const RigExecDeltaMushSettings &settings,
    const GfMatrix4d &computationToTarget, const RigExecDeltaMushRest *restData)
{
    if (restData) {
        return RigExecApplyDeltaMushInSpaceWithRestData<GfVec3f, GfVec3d,
                                                        GfMatrix4d>(
            points, *restData, displacement, computationToTarget);
    }
    return RigExecApplyDeltaMushInSpaceKernel<GfVec3f, GfVec3d, GfMatrix4d>(
        points, rest, counts, indices, iterations, step, pinBorders,
        distanceWeight, displacement, settings, computationToTarget);
}

bool
RigExecApplyWrinkle(std::vector<GfVec3f> *points,
                    const std::vector<GfVec3f> &rest,
                    const std::vector<int> &counts,
                    const std::vector<int> &indices,
                    const RigExecWrinkleSettings &settings,
                    const RigExecWrinkleMesh *topology)
{
    if (topology) {
        return RigExecApplyWrinkleWithTopology<GfVec3f, GfVec3d>(
            points, rest, counts, indices, settings, *topology);
    }
    return RigExecApplyWrinkleKernel<GfVec3f, GfVec3d>(points, rest, counts,
                                                       indices, settings);
}

RigExecVertexNormalsTopology
RigExecBuildVertexNormalsTopology(size_t pointsSize,
                                  const int *faceVertexCounts,
                                  size_t faceVertexCountsSize,
                                  const int *faceVertexIndices,
                                  size_t faceVertexIndicesSize)
{
    return RigExecBuildVertexNormalsTopologyKernel(
        pointsSize, faceVertexCounts, faceVertexCountsSize, faceVertexIndices,
        faceVertexIndicesSize);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(const GfVec3f *points, size_t pointsSize,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize,
                            const RigExecVertexNormalsTopology &topology,
                            const RigExecRangeRunner *ranges)
{
    return RigExecComputeVertexNormalsKernel(
        points, pointsSize, faceVertexCounts, faceVertexCountsSize,
        faceVertexIndices, faceVertexIndicesSize, &topology, ranges);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(const GfVec3f *points, size_t pointsSize,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize,
                            const RigExecRangeRunner *ranges)
{
    return RigExecComputeVertexNormalsKernel(
        points, pointsSize, faceVertexCounts, faceVertexCountsSize,
        faceVertexIndices, faceVertexIndicesSize, nullptr, ranges);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(const std::vector<GfVec3f> &points,
                            const std::vector<int> &faceVertexCounts,
                            const std::vector<int> &faceVertexIndices)
{
    return RigExecComputeVertexNormals(
        points.data(), points.size(), faceVertexCounts.data(),
        faceVertexCounts.size(), faceVertexIndices.data(),
        faceVertexIndices.size());
}

std::vector<GfVec3f>
RigExecComputeExtent(const GfVec3f *points, size_t pointsSize,
                     const float *widths, size_t widthsSize)
{
    return RigExecComputeExtentKernel(points, pointsSize, widths, widthsSize);
}

std::vector<GfVec3f>
RigExecComputeExtent(const std::vector<GfVec3f> &points,
                     const std::vector<float> &widths)
{
    return RigExecComputeExtent(points.data(), points.size(), widths.data(),
                                widths.size());
}

void
RigExecApplyLattice(std::vector<GfVec3f> *points, const GfVec3f *restPoints,
                    size_t restPointsSize, const GfVec3f *restCage,
                    size_t restCageSize, const GfVec3f *posedCage,
                    size_t posedCageSize, const GfVec3i &divisions)
{
    RigExecApplyLatticeKernel(
        points, restPoints, restPointsSize, restCage, restCageSize, posedCage,
        posedCageSize, divisions[0], divisions[1], divisions[2],
        static_cast<RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *>(nullptr));
}

void
RigExecApplyLattice(std::vector<GfVec3f> *points,
                    const std::vector<GfVec3f> &restPoints,
                    const std::vector<GfVec3f> &restCage,
                    const std::vector<GfVec3f> &posedCage,
                    const GfVec3i &divisions)
{
    RigExecApplyLattice(points, restPoints.data(), restPoints.size(),
                        restCage.data(), restCage.size(), posedCage.data(),
                        posedCage.size(), divisions);
}

void
RigExecApplyLattice(std::vector<GfVec3f> *points,
                    const std::vector<GfVec3f> &restPoints,
                    const std::vector<GfVec3f> &restCage,
                    const std::vector<GfVec3f> &posedCage,
                    const GfVec3i &divisions,
                    RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache)
{
    RigExecApplyLatticeKernel(points, restPoints.data(), restPoints.size(),
                              restCage.data(), restCage.size(),
                              posedCage.data(), posedCage.size(), divisions[0],
                              divisions[1], divisions[2], cache);
}

void
RigExecBuildSurfaceAccel(const int *faceVertexCounts,
                         size_t faceVertexCountsSize,
                         const int *faceVertexIndices,
                         size_t faceVertexIndicesSize,
                         const GfVec3f *surfacePoints, size_t surfacePointsSize,
                         RigExecSurfaceAccel *accel)
{
    RigExecBuildSurfaceAccelKernel(faceVertexCounts, faceVertexCountsSize,
                                   faceVertexIndices, faceVertexIndicesSize,
                                   surfacePoints, surfacePointsSize, accel);
}

void
RigExecBuildSurfaceAccel(const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices,
                         const std::vector<GfVec3f> &surfacePoints,
                         RigExecSurfaceAccel *accel)
{
    RigExecBuildSurfaceAccel(faceVertexCounts.data(), faceVertexCounts.size(),
                             faceVertexIndices.data(), faceVertexIndices.size(),
                             surfacePoints.data(), surfacePoints.size(), accel);
}

void
RigExecApplySurfaceProject(std::vector<GfVec3f> *points,
                           const GfVec3f *surfacePoints,
                           size_t surfacePointsSize,
                           const int *faceVertexCounts,
                           size_t faceVertexCountsSize,
                           const int *faceVertexIndices,
                           size_t faceVertexIndicesSize, double weight,
                           const RigExecSurfaceAccel *accel)
{
    RigExecApplySurfaceProjectKernel(points, surfacePoints, surfacePointsSize,
                                     faceVertexCounts, faceVertexCountsSize,
                                     faceVertexIndices, faceVertexIndicesSize,
                                     weight, accel);
}

void
RigExecApplySurfaceProject(std::vector<GfVec3f> *points,
                           const std::vector<GfVec3f> &surfacePoints,
                           const std::vector<int> &faceVertexCounts,
                           const std::vector<int> &faceVertexIndices,
                           double weight, const RigExecSurfaceAccel *accel)
{
    RigExecApplySurfaceProject(
        points, surfacePoints.data(), surfacePoints.size(),
        faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), weight, accel);
}

namespace {

// Uniform cubic B-spline position on one nonperiodic segment span.
GfVec3f
_BsplinePoint(const GfVec3f &p0, const GfVec3f &p1, const GfVec3f &p2,
              const GfVec3f &p3, float t)
{
    const float t2 = t * t, t3 = t2 * t;
    const float b0 = (1 - 3 * t + 3 * t2 - t3) / 6.0f;
    const float b1 = (4 - 6 * t2 + 3 * t3) / 6.0f;
    const float b2 = (1 + 3 * t + 3 * t2 - 3 * t3) / 6.0f;
    const float b3 = t3 / 6.0f;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

} // namespace

RigExecCurveFrameSamples
RigExecSampleCurveRMF(const GfVec3f *controlPoints, size_t controlPointsSize,
                      int sampleCount)
{
    RigExecCurveFrameSamples samples;
    // The short-curve branch below interpolates segments [i, i + 1].  A
    // single point has no segment; accepting it makes `size() - 2` equal -1
    // and indexes before the range.  Treat it like every other degenerate
    // driver and publish no frames.
    if (sampleCount < 2 || controlPointsSize < 2 || !controlPoints) {
        return samples;
    }

    // Dense presampling for arc-length parameterization. Under four
    // control points the stock cubic segment count is zero: the control
    // polygon interpolates linearly (deterministic short-curve rule).
    const int dense = std::max(sampleCount * 16, 64);
    std::vector<GfVec3f> densePoints;
    densePoints.reserve(dense + 1);
    const int spans = static_cast<int>(controlPointsSize) - 3;
    for (int i = 0; i <= dense; ++i) {
        const float u = float(i) / float(dense);
        if (spans >= 1) {
            const float s = u * spans;
            const int span = std::min(spans - 1, int(s));
            const float t = s - span;
            densePoints.push_back(_BsplinePoint(
                controlPoints[span], controlPoints[span + 1],
                controlPoints[span + 2], controlPoints[span + 3], t));
        } else {
            const float s = u * (controlPointsSize - 1);
            const int seg = std::min(int(controlPointsSize) - 2, int(s));
            densePoints.push_back(
                controlPoints[seg] +
                (controlPoints[seg + 1] - controlPoints[seg]) * (s - seg));
        }
    }
    std::vector<float> arcLength(densePoints.size(), 0.0f);
    for (size_t i = 1; i < densePoints.size(); ++i) {
        arcLength[i] = arcLength[i - 1] +
                       (densePoints[i] - densePoints[i - 1]).GetLength();
    }
    const float total = arcLength.back();
    if (total <= 1e-12f) {
        return samples;
    }

    // Sample at equal arc length.
    samples.positions.reserve(sampleCount);
    samples.parameters.reserve(sampleCount);
    size_t cursor = 0;
    for (int k = 0; k < sampleCount; ++k) {
        const float target = total * float(k) / float(sampleCount - 1);
        while (cursor + 1 < arcLength.size() &&
               arcLength[cursor + 1] < target) {
            ++cursor;
        }
        const float span = arcLength[cursor + 1] - arcLength[cursor];
        const float t =
            span > 1e-12f ? (target - arcLength[cursor]) / span : 0.0f;
        samples.positions.push_back(
            densePoints[cursor] +
            (densePoints[cursor + 1] - densePoints[cursor]) * t);
        samples.parameters.push_back(float(k) / float(sampleCount - 1));
    }

    // Tangents by central differences; rotation-minimizing normals by the
    // double-reflection method (Wang et al. 2008).
    samples.tangents.resize(sampleCount);
    for (int k = 0; k < sampleCount; ++k) {
        const GfVec3f &prev = samples.positions[std::max(0, k - 1)];
        const GfVec3f &next =
            samples.positions[std::min(sampleCount - 1, k + 1)];
        GfVec3f tangent = next - prev;
        const float len = tangent.GetLength();
        samples.tangents[k] = len > 1e-12f ? tangent / len : GfVec3f(1, 0, 0);
    }
    samples.normals.resize(sampleCount);
    samples.binormals.resize(sampleCount);
    // Deterministic initial normal: world axis least parallel to the
    // first tangent, projected.
    {
        const GfVec3f t0 = samples.tangents[0];
        GfVec3f candidate(1, 0, 0);
        float best = 2.0f;
        for (const GfVec3f axis :
             {GfVec3f(1, 0, 0), GfVec3f(0, 1, 0), GfVec3f(0, 0, 1)}) {
            const float align = std::abs(GfDot(axis, t0));
            if (align < best) {
                best = align;
                candidate = axis;
            }
        }
        GfVec3f n0 = candidate - t0 * GfDot(t0, candidate);
        n0.Normalize();
        samples.normals[0] = n0;
        samples.binormals[0] = GfCross(t0, n0);
    }
    for (int k = 0; k + 1 < sampleCount; ++k) {
        const GfVec3f v1 = samples.positions[k + 1] - samples.positions[k];
        const float c1 = GfDot(v1, v1);
        if (c1 <= 1e-20f) {
            samples.normals[k + 1] = samples.normals[k];
            samples.binormals[k + 1] = samples.binormals[k];
            continue;
        }
        const GfVec3f nL = samples.normals[k] -
                           v1 * (2.0f / c1) * GfDot(v1, samples.normals[k]);
        const GfVec3f tL = samples.tangents[k] -
                           v1 * (2.0f / c1) * GfDot(v1, samples.tangents[k]);
        const GfVec3f v2 = samples.tangents[k + 1] - tL;
        const float c2 = GfDot(v2, v2);
        GfVec3f n = c2 > 1e-20f ? nL - v2 * (2.0f / c2) * GfDot(v2, nL) : nL;
        n -= samples.tangents[k + 1] * GfDot(samples.tangents[k + 1], n);
        const float len = n.GetLength();
        samples.normals[k + 1] = len > 1e-12f ? n / len : samples.normals[k];
        samples.binormals[k + 1] =
            GfCross(samples.tangents[k + 1], samples.normals[k + 1]);
    }
    return samples;
}

RigExecCurveFrameSamples
RigExecSampleCurveRMF(const std::vector<GfVec3f> &controlPoints,
                      int sampleCount)
{
    return RigExecSampleCurveRMF(controlPoints.data(), controlPoints.size(),
                                 sampleCount);
}

void
RigExecApplyRibbonTransport(std::vector<GfVec3f> *points,
                            const GfVec2f *bindCoords, size_t bindCoordsSize,
                            const RigExecCurveFrameSamples &restSamples,
                            const RigExecCurveFrameSamples &posedSamples)
{
    const size_t sampleCount = restSamples.GetSize();
    if (points->empty() || bindCoordsSize != points->size() ||
        sampleCount < 2 || posedSamples.GetSize() != sampleCount) {
        return; // invalid binding: pass through
    }
    if (!bindCoords) {
        return; // a null range with a nonzero size: pass through
    }

    // Per-sample rigid maps from the rest frame to the posed frame.
    std::vector<GfMatrix4d> maps(sampleCount);
    for (size_t k = 0; k < sampleCount; ++k) {
        const std::array<GfVec3d, 4> rest = {
            GfVec3d(restSamples.positions[k]),
            GfVec3d(restSamples.positions[k] + restSamples.tangents[k]),
            GfVec3d(restSamples.positions[k] + restSamples.normals[k]),
            GfVec3d(restSamples.positions[k] + restSamples.binormals[k])};
        const std::array<GfVec3d, 4> posed = {
            GfVec3d(posedSamples.positions[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.tangents[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.normals[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.binormals[k])};
        if (!RigExecPointsToMatrix(rest, posed, &maps[k])) {
            maps[k].SetIdentity();
        }
    }

    for (size_t i = 0; i < points->size(); ++i) {
        const float u = std::min(1.0f, std::max(0.0f, bindCoords[i][0]));
        const float s = u * float(sampleCount - 1);
        const size_t k = std::min(sampleCount - 2, size_t(s));
        const float t = s - float(k);
        const GfVec3d a = maps[k].TransformAffine(GfVec3d((*points)[i]));
        const GfVec3d b = maps[k + 1].TransformAffine(GfVec3d((*points)[i]));
        (*points)[i] = GfVec3f(a + (b - a) * double(t));
    }
}

void
RigExecApplyRibbonTransport(std::vector<GfVec3f> *points,
                            const std::vector<GfVec2f> &bindCoords,
                            const RigExecCurveFrameSamples &restSamples,
                            const RigExecCurveFrameSamples &posedSamples)
{
    RigExecApplyRibbonTransport(points, bindCoords.data(), bindCoords.size(),
                                restSamples, posedSamples);
}

} // namespace rigExec

namespace rigExec {

bool
RigExecNurbsCurve::IsValid() const
{
    return RigExecNurbsCurveIsValidKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, knots, order);
}

double
RigExecNurbsCurve::DomainStart() const
{
    return RigExecNurbsCurveDomainStartKernel(knots, order);
}

double
RigExecNurbsCurve::DomainEnd() const
{
    return RigExecNurbsCurveDomainEndKernel(points, knots);
}

GfVec3f
RigExecNurbsCurve::Evaluate(double u) const
{
    return RigExecEvaluateNurbsCurveKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, knots, order, u);
}

bool
RigExecBuildWireBasis(const GfVec2f *bindCoords, size_t bindCount,
                      size_t meshPointCount, const std::vector<int> &indices,
                      int order, const std::vector<double> &knots,
                      size_t controlPointCount, double dropoffDistance,
                      RigExecWireBasis *basis)
{
    return RigExecBuildWireBasisKernel<GfVec3f, GfVec3d, GfVec2f>(
        bindCoords, bindCount, meshPointCount, indices, order, knots,
        controlPointCount, dropoffDistance, basis);
}

bool
RigExecWireBasisInputsAreUsable(const GfVec2f *bindCoords, size_t bindCount,
                                size_t meshPointCount, size_t indexCount,
                                int order, const std::vector<double> &knots,
                                size_t controlPointCount)
{
    return RigExecWireBasisInputsAreUsableKernel<GfVec3f, GfVec3d, GfVec2f>(
        bindCoords, bindCount, meshPointCount, indexCount, order, knots,
        controlPointCount);
}

bool
RigExecApplyWireBasis(std::vector<GfVec3f> *points,
                      const RigExecWireBasis &basis,
                      const std::vector<int> &indices,
                      const std::vector<float> &weights,
                      const std::vector<GfVec3f> &restControlPoints,
                      const std::vector<GfVec3f> &posedControlPoints)
{
    return RigExecApplyWireBasisKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, basis, indices, weights, restControlPoints, posedControlPoints);
}

bool
RigExecApplyWireBasisRange(std::vector<GfVec3f> *points,
                           const RigExecWireBasis &basis,
                           const std::vector<int> &indices,
                           const std::vector<float> &weights,
                           const std::vector<GfVec3f> &restControlPoints,
                           const std::vector<GfVec3f> &posedControlPoints,
                           size_t begin, size_t end)
{
    return RigExecApplyWireBasisRangeKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, basis, indices, weights, restControlPoints, posedControlPoints,
        begin, end);
}

bool
RigExecApplyWireBasisGroup(GfVec3f *out, size_t begin, size_t end,
                           const RigExecWireBasis &basis,
                           const std::vector<int> &indices,
                           const std::vector<float> &weights,
                           const std::vector<GfVec3f> &restControlPoints,
                           const std::vector<GfVec3f> &posedControlPoints)
{
    return RigExecApplyWireBasisGroupKernel<GfVec3f, GfVec3d, GfVec2f>(
        out, begin, end, basis, indices, weights, restControlPoints,
        posedControlPoints);
}

std::vector<GfVec2f>
RigExecBindWire(const std::vector<GfVec3f> &points,
                const RigExecNurbsCurve &restCurve)
{
    std::vector<GfVec2f> out(points.size(), GfVec2f(0.0f, 0.0f));
    if (!restCurve.IsValid()) {
        return out;
    }
    const double u0 = restCurve.DomainStart();
    const double u1 = restCurve.DomainEnd();
    const size_t spans = std::max<size_t>(1, restCurve.points->size() -
                                                 size_t(restCurve.order) + 1);
    const size_t samples = spans * 32;
    std::vector<GfVec3f> table(samples + 1);
    for (size_t i = 0; i <= samples; ++i) {
        table[i] = restCurve.Evaluate(u0 + (u1 - u0) * double(i) / samples);
    }
    const double step = (u1 - u0) / double(samples);
    for (size_t pi = 0; pi < points.size(); ++pi) {
        const GfVec3f &p = points[pi];
        size_t best = 0;
        float bestSq = std::numeric_limits<float>::max();
        for (size_t i = 0; i <= samples; ++i) {
            const float dSq = (table[i] - p).GetLengthSq();
            if (dSq < bestSq) {
                bestSq = dSq;
                best = i;
            }
        }
        // Ternary refinement inside the neighbouring samples.
        double lo = std::max(u0, u0 + step * (double(best) - 1.0));
        double hi = std::min(u1, u0 + step * (double(best) + 1.0));
        for (int it = 0; it < 40; ++it) {
            const double m1 = lo + (hi - lo) / 3.0;
            const double m2 = hi - (hi - lo) / 3.0;
            if ((restCurve.Evaluate(m1) - p).GetLengthSq() <
                (restCurve.Evaluate(m2) - p).GetLengthSq()) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        const double u = 0.5 * (lo + hi);
        out[pi] =
            GfVec2f(float(u), float((restCurve.Evaluate(u) - p).GetLength()));
    }
    return out;
}

bool
RigExecApplyWire(std::vector<GfVec3f> *points,
                 const RigExecNurbsCurve &restCurve,
                 const RigExecNurbsCurve &posedCurve, const GfVec2f *bindCoords,
                 size_t bindCount, double dropoffDistance, size_t begin,
                 size_t end, const GfVec3f *restEvals, size_t restEvalCount)
{
    return RigExecApplyWireKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, restCurve, posedCurve, bindCoords, bindCount, dropoffDistance,
        begin, end, restEvals, restEvalCount);
}

bool
RigExecApplyWireGroup(GfVec3f *out, size_t begin, size_t end, size_t count,
                      const RigExecNurbsCurve &restCurve,
                      const RigExecNurbsCurve &posedCurve,
                      const GfVec2f *bindCoords, size_t bindCount,
                      double dropoffDistance, const GfVec3f *restEvals,
                      size_t restEvalCount)
{
    return RigExecApplyWireGroupKernel<GfVec3f, GfVec3d, GfVec2f>(
        out, begin, end, count, restCurve, posedCurve, bindCoords, bindCount,
        dropoffDistance, restEvals, restEvalCount);
}

bool
RigExecWireInputsAreUsable(const RigExecNurbsCurve &restCurve,
                           const RigExecNurbsCurve &posedCurve,
                           const GfVec2f *bindCoords, size_t bindCount,
                           size_t pointCount)
{
    return RigExecWireInputsAreUsableKernel<GfVec3f, GfVec3d, GfVec2f>(
        restCurve, posedCurve, bindCoords, bindCount, pointCount);
}

bool
RigExecApplyWireSparse(std::vector<GfVec3f> *points,
                       const RigExecNurbsCurve &restCurve,
                       const RigExecNurbsCurve &posedCurve,
                       const GfVec2f *bindCoords, size_t bindCount,
                       double dropoffDistance, const std::vector<int> &indices,
                       const std::vector<float> &weights)
{
    return RigExecApplyWireSparseKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, restCurve, posedCurve, bindCoords, bindCount, dropoffDistance,
        indices, weights);
}

bool
RigExecNurbsPatch::IsValid() const
{
    if (!points || !uKnots || !vKnots || uOrder < 2 || uOrder > 16 ||
        vOrder < 2 || vOrder > 16 || uVertexCount < uOrder ||
        vVertexCount < vOrder) {
        return false;
    }
    const size_t expectedPoints = size_t(uVertexCount) * size_t(vVertexCount);
    if (pointCount != expectedPoints ||
        uKnotCount != size_t(uVertexCount) + size_t(uOrder) ||
        vKnotCount != size_t(vVertexCount) + size_t(vOrder)) {
        return false;
    }
    for (size_t i = 0; i < uKnotCount; ++i) {
        if (!std::isfinite(uKnots[i]) || (i > 0 && uKnots[i] < uKnots[i - 1])) {
            return false;
        }
    }
    for (size_t i = 0; i < vKnotCount; ++i) {
        if (!std::isfinite(vKnots[i]) || (i > 0 && vKnots[i] < vKnots[i - 1])) {
            return false;
        }
    }
    if (!(uKnots[size_t(uOrder) - 1] < uKnots[size_t(uVertexCount)]) ||
        !(vKnots[size_t(vOrder) - 1] < vKnots[size_t(vVertexCount)])) {
        return false;
    }
    for (size_t i = 0; i < pointCount; ++i) {
        const GfVec3f &p = points[i];
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
            !std::isfinite(p[2])) {
            return false;
        }
    }
    if (pointWeightCount != 0) {
        if (!pointWeights || pointWeightCount != pointCount) {
            return false;
        }
        for (size_t i = 0; i < pointWeightCount; ++i) {
            if (!std::isfinite(pointWeights[i]) || pointWeights[i] <= 0.0) {
                return false;
            }
        }
    }
    return true;
}

double
RigExecNurbsPatch::UDomainStart() const
{
    return uKnots[size_t(uOrder) - 1];
}

double
RigExecNurbsPatch::UDomainEnd() const
{
    return uKnots[size_t(uVertexCount)];
}

double
RigExecNurbsPatch::VDomainStart() const
{
    return vKnots[size_t(vOrder) - 1];
}

double
RigExecNurbsPatch::VDomainEnd() const
{
    return vKnots[size_t(vVertexCount)];
}

GfVec3f
RigExecNurbsPatch::EvaluateNormalized(double uNorm, double vNorm) const
{
    auto mapParam = [](double norm, bool periodic, double d0, double d1) {
        double t = norm;
        if (periodic) {
            t = t - std::floor(t);
            if (t < 0.0 || t >= 1.0) {
                t = 0.0;
            }
        } else {
            t = std::min(std::max(t, 0.0), 1.0);
        }
        return std::min(std::max(d0 + t * (d1 - d0), d0), d1);
    };
    const double u = mapParam(uNorm, uPeriodic, UDomainStart(), UDomainEnd());
    const double v = mapParam(vNorm, vPeriodic, VDomainStart(), VDomainEnd());

    auto evalBasis = [](double param, int order, size_t vertexCount,
                        const double *knots, size_t *spanOut, double *N) {
        const int degree = order - 1;
        size_t span = size_t(degree);
        while (span + 1 < vertexCount && knots[span + 1] <= param) {
            ++span;
        }
        *spanOut = span;
        double left[16], right[16];
        N[0] = 1.0;
        for (int j = 1; j <= degree; ++j) {
            left[j] = param - knots[span + 1 - size_t(j)];
            right[j] = knots[span + size_t(j)] - param;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
                const double denom = right[r + 1] + left[j - r];
                const double temp = denom != 0.0 ? N[r] / denom : 0.0;
                N[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            N[j] = saved;
        }
    };

    size_t su = 0, sv = 0;
    double Nu[16], Nv[16];
    evalBasis(u, uOrder, size_t(uVertexCount), uKnots, &su, Nu);
    evalBasis(v, vOrder, size_t(vVertexCount), vKnots, &sv, Nv);

    const int pu = uOrder - 1;
    const int pv = vOrder - 1;
    const size_t nu = size_t(uVertexCount);
    GfVec3d sum(0.0, 0.0, 0.0);
    if (pointWeightCount == 0) {
        for (int b = 0; b <= pv; ++b) {
            const size_t iv = sv - size_t(pv) + size_t(b);
            GfVec3d row(0.0, 0.0, 0.0);
            for (int a = 0; a <= pu; ++a) {
                const size_t iu = su - size_t(pu) + size_t(a);
                row += GfVec3d(points[iv * nu + iu]) * Nu[a];
            }
            sum += row * Nv[b];
        }
        return GfVec3f(sum);
    }
    double wSum = 0.0;
    for (int b = 0; b <= pv; ++b) {
        const size_t iv = sv - size_t(pv) + size_t(b);
        GfVec3d row(0.0, 0.0, 0.0);
        double wRow = 0.0;
        for (int a = 0; a <= pu; ++a) {
            const size_t iu = su - size_t(pu) + size_t(a);
            const size_t idx = iv * nu + iu;
            const double w = Nu[a] * pointWeights[idx];
            row += GfVec3d(points[idx]) * w;
            wRow += w;
        }
        sum += row * Nv[b];
        wSum += wRow * Nv[b];
    }
    return wSum > 0.0 ? GfVec3f(sum / wSum) : GfVec3f(0.0f);
}

bool
RigExecApplyPatchMover(std::vector<GfVec3f> *points,
                       const RigExecNurbsPatch &patch, const GfVec3f *uvPoints,
                       size_t uvPointCount, const RigExecRangeRunner *ranges)
{
    if (!points || !patch.IsValid()) {
        return false;
    }
    const size_t n = points->size();
    if (n == 0) {
        return uvPointCount == 0 ||
               (uvPoints && uvPointCount == 1 &&
                std::isfinite(uvPoints[0][0]) && std::isfinite(uvPoints[0][1]));
    }
    if (!uvPoints || (uvPointCount != n && uvPointCount != 1)) {
        return false;
    }
    for (size_t i = 0; i < uvPointCount; ++i) {
        if (!std::isfinite(uvPoints[i][0]) || !std::isfinite(uvPoints[i][1])) {
            return false;
        }
    }
    const bool broadcast = uvPointCount == 1;
    GfVec3f *const out = points->data();
    // UVs read from the storage being written depend on the order of the
    // writes: only the single loop reproduces them.
    const std::uintptr_t outBegin = reinterpret_cast<std::uintptr_t>(out);
    const std::uintptr_t outEnd = reinterpret_cast<std::uintptr_t>(out + n);
    const std::uintptr_t uvBegin = reinterpret_cast<std::uintptr_t>(uvPoints);
    const std::uintptr_t uvEnd =
        reinterpret_cast<std::uintptr_t>(uvPoints + uvPointCount);
    const bool overlaps = uvBegin < outEnd && outBegin < uvEnd;
    std::atomic<bool> finite(true);
    RigExecRunRanges(
        overlaps ? nullptr : ranges, n, 512, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                if (!finite.load(std::memory_order_relaxed)) {
                    return;
                }
                const GfVec3f &uv = uvPoints[broadcast ? 0 : i];
                const GfVec3f evaluated =
                    patch.EvaluateNormalized(double(uv[0]), double(uv[1]));
                if (!std::isfinite(evaluated[0]) ||
                    !std::isfinite(evaluated[1]) ||
                    !std::isfinite(evaluated[2])) {
                    finite.store(false, std::memory_order_relaxed);
                    return;
                }
                out[i] = evaluated;
            }
        });
    return finite.load();
}

} // namespace rigExec
