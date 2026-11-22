// RigExec geometry mover kernels implementation.
#include "geometryKernels.h"
#include "deltaMushKernel.h"
#include "dualQuat.h"
#include "latticeKernel.h"
#include "ribbonKernel.h"
#include "spatialAccel.h"
#include "surfaceProjectKernel.h"
#include "volumeCorrectKernel.h"
#include "wrinkleKernel.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/rotation.h"

#include <atomic>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <map>
#include <set>
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
    RigExecApplyVolumeCorrectKernel(points->data(), points->size(),
                                    referenceVolume, strength);
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
    return RigExecDecomposePartialTransformKernel<GfVec3d>(transform);
}

GfMatrix4d
RigExecApplyPartialDecomposition(const RigExecPartialDecomposition &d, double w)
{
    return RigExecApplyPartialDecompositionKernel<GfMatrix4d>(d, w);
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
    const double s = std::min(std::max(strength, 0.0), 1.0);
    if (points->empty() || s <= 0.0) {
        return;
    }
    if (!adjacency.Covers(points->size())) {
        return; // invalid topology: pass through
    }
    std::vector<GfVec3f> owned;
    const GfVec3f *src = source;
    if (!src || sourceCount != points->size()) {
        if (src) {
            return; // a short range reads OOB: pass through instead
        }
        owned = *points;
        src = owned.data();
    }
    for (size_t i = 0; i < points->size(); ++i) {
        if (adjacency.neighbors[i].empty()) {
            continue;
        }
        GfVec3f average(0);
        for (int n : adjacency.neighbors[i]) {
            average += src[n];
        }
        average /= float(adjacency.neighbors[i].size());
        (*points)[i] = src[i] + (average - src[i]) * float(s);
    }
}

void
RigExecApplyLaplacianSmooth(std::vector<GfVec3f> *points,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize, double strength,
                            const GfVec3f *source, size_t sourceCount)
{
    const double s = std::min(std::max(strength, 0.0), 1.0);
    if (points->empty() || s <= 0.0) {
        return;
    }
    RigExecMeshAdjacency adjacency;
    RigExecBuildMeshAdjacency(points->size(), faceVertexCounts,
                              faceVertexCountsSize, faceVertexIndices,
                              faceVertexIndicesSize, &adjacency);
    RigExecApplyLaplacianSmoothWithAdjacency(points, adjacency, strength,
                                             source, sourceCount);
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

std::vector<GfVec3f>
RigExecComputeVertexNormals(const GfVec3f *points, size_t pointsSize,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize,
                            const RigExecVertexNormalsTopology &topology,
                            const RigExecRangeRunner *ranges)
{
    // A null range with a nonzero size answers like a mesh with no
    // usable faces: zero normals.
    if ((pointsSize > 0 && !points) ||
        (faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices) ||
        !topology.Fits(pointsSize, faceVertexCountsSize,
                       faceVertexIndicesSize)) {
        return std::vector<GfVec3f>(pointsSize, GfVec3f(0));
    }
    return RigExecVertexNormalsKernel(points, pointsSize, faceVertexCounts,
                                      faceVertexIndices, topology, ranges);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(const GfVec3f *points, size_t pointsSize,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize,
                            const RigExecRangeRunner *ranges)
{
    if ((pointsSize > 0 && !points) ||
        (faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        return std::vector<GfVec3f>(pointsSize, GfVec3f(0));
    }
    return RigExecVertexNormalsKernel(
        points, pointsSize, faceVertexCounts, faceVertexIndices,
        RigExecBuildVertexNormalsTopology(
            pointsSize, faceVertexCounts, faceVertexCountsSize,
            faceVertexIndices, faceVertexIndicesSize),
        ranges);
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
    return RigExecExtentKernel(points, pointsSize, widths, widthsSize);
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

namespace {

GfVec3f
_ClosestPointOnTriangle(const GfVec3f &p, const GfVec3f &a, const GfVec3f &b,
                        const GfVec3f &c)
{
    return RigExecClosestPointOnTriangleKernel(p, a, b, c);
}

// Whether an owned key still describes a borrowed range: same size and
// same values. A null range only ever matches the empty key, and the
// projector returns before validation on a null range with a nonzero
// size, so every validated call below compares against real storage.
template <typename T>
bool
_RangeMatches(const std::vector<T> &key, const T *data, size_t size)
{
    return key.size() == size &&
           (size == 0 || std::equal(key.begin(), key.end(), data));
}

} // namespace

void
RigExecBuildSurfaceAccel(const int *faceVertexCounts,
                         size_t faceVertexCountsSize,
                         const int *faceVertexIndices,
                         size_t faceVertexIndicesSize,
                         const GfVec3f *surfacePoints, size_t surfacePointsSize,
                         RigExecSurfaceAccel *accel)
{
    if (!accel) {
        return;
    }
    RigExecSurfaceAccel built;
    if (faceVertexCounts) {
        built.counts.assign(faceVertexCounts,
                            faceVertexCounts + faceVertexCountsSize);
    }
    if (faceVertexIndices) {
        built.indices.assign(faceVertexIndices,
                             faceVertexIndices + faceVertexIndicesSize);
    }
    if (surfacePoints) {
        built.points.assign(surfacePoints, surfacePoints + surfacePointsSize);
    }
    built.fan = std::make_shared<const RigExecFanTrisH7>(RigExecBuildFanTrisH7(
        faceVertexCounts, faceVertexCountsSize, faceVertexIndices,
        faceVertexIndicesSize, surfacePointsSize));
    if (!built.fan->truncated && built.fan->tris.size() >= kRigExecBvhMinTris &&
        (surfacePointsSize == 0 || surfacePoints)) {
        auto bvh = std::make_shared<RigExecTriangleBvh<GfVec3f>>();
        if (bvh->Build(surfacePoints, surfacePointsSize, built.fan->tris.data(),
                       built.fan->tris.size())) {
            built.bvh = std::move(bvh);
        }
        // Build failed (non-finite surface points): the null bvh runs the
        // verbatim loop, whose NaN behavior is the defined one.
    }
    *accel = std::move(built);
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
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (points->empty() || surfacePointsSize == 0 || w <= 0.0) {
        return;
    }
    if (!surfacePoints || (faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        return; // a null range with a nonzero size: pass through
    }
    // One fan triangulation for the whole call instead of one per point.
    // A truncated walk returns with no point modified: the nested walk
    // below always meets the overrun during its first point.
    RigExecSurfaceAccel unretained;
    // A stale entry -- same cardinality but new topology or, on the
    // BVH path, new points -- rebuilds rather than answering wrong.
    // The fan only depends on the topology and the point count, so
    // below the gate the values are not compared: the verbatim loop
    // walks the current inputs either way.
    if (!accel || !accel->fan ||
        !_RangeMatches(accel->counts, faceVertexCounts, faceVertexCountsSize) ||
        !_RangeMatches(accel->indices, faceVertexIndices,
                       faceVertexIndicesSize) ||
        accel->points.size() != surfacePointsSize ||
        (accel->bvh &&
         !_RangeMatches(accel->points, surfacePoints, surfacePointsSize))) {
        RigExecBuildSurfaceAccel(faceVertexCounts, faceVertexCountsSize,
                                 faceVertexIndices, faceVertexIndicesSize,
                                 surfacePoints, surfacePointsSize, &unretained);
        accel = &unretained;
    }
    const RigExecFanTrisH7 &fan = *accel->fan;
    if (fan.truncated) {
        return;
    }
    if (fan.tris.size() >= kRigExecBvhMinTris &&
        points->size() >= kRigExecBvhMinQueries && accel->bvh) {
        for (GfVec3f &p : *points) {
            const GfVec3f best = accel->bvh->QueryNearest(
                p, surfacePoints, fan.tris.data(), _ClosestPointOnTriangle);
            p = p + (best - p) * float(w);
        }
        return;
    }
    RigExecApplySurfaceProjectKernel(
        points->data(), points->size(), surfacePoints, surfacePointsSize,
        faceVertexCounts, faceVertexCountsSize, faceVertexIndices,
        faceVertexIndicesSize, w);
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

RigExecCurveFrameSamples
RigExecSampleCurveRMF(const GfVec3f *controlPoints, size_t controlPointsSize,
                      int sampleCount)
{
    return RigExecSampleCurveRMFKernel(controlPoints, controlPointsSize,
                                       sampleCount);
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
    if (points->empty() || posedSamples.GetSize() != sampleCount) {
        return; // invalid binding: pass through
    }
    const auto framePoints = [](const RigExecCurveFrameSamples &samples) {
        std::vector<std::array<GfVec3d, 4>> frames(samples.GetSize());
        for (size_t k = 0; k < frames.size(); ++k) {
            const GfVec3f &o = samples.positions[k];
            frames[k] = {GfVec3d(o), GfVec3d(o + samples.tangents[k]),
                         GfVec3d(o + samples.normals[k]),
                         GfVec3d(o + samples.binormals[k])};
        }
        return frames;
    };
    const std::vector<std::array<GfVec3d, 4>> rest = framePoints(restSamples);
    const std::vector<std::array<GfVec3d, 4>> posed = framePoints(posedSamples);
    // An invalid binding passes the points through.
    RigExecApplyRibbonKernel<RigExecDualQuatMath<GfVec3f, GfVec3d>>(
        points->data(), points->size(), bindCoords, bindCoordsSize, sampleCount,
        [&](size_t k) -> const auto & { return rest[k]; },
        [&](size_t k) -> const auto & { return posed[k]; });
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

RigExecNurbsCurveSpan<GfVec3f>
RigExecNurbsCurve::Span() const
{
    return {points ? points->data() : nullptr, points ? points->size() : 0,
            order, knots ? knots->data() : nullptr, knots ? knots->size() : 0};
}

bool
RigExecNurbsCurve::IsValid() const
{
    return Span().IsValid();
}

double
RigExecNurbsCurve::DomainStart() const
{
    return Span().DomainStart();
}

double
RigExecNurbsCurve::DomainEnd() const
{
    return Span().DomainEnd();
}

GfVec3f
RigExecNurbsCurve::Evaluate(double u) const
{
    return Span().Evaluate<GfVec3d>(u);
}

bool
RigExecBuildWireBasis(const GfVec2f *bindCoords, size_t bindCount,
                      size_t meshPointCount, const std::vector<int> &indices,
                      int order, const std::vector<double> &knots,
                      size_t controlPointCount, double dropoffDistance,
                      RigExecWireBasis *basis)
{
    return RigExecBuildWireBasisKernel(
        bindCoords, bindCount, meshPointCount, indices.data(), indices.size(),
        order, knots.data(), knots.size(), controlPointCount, dropoffDistance,
        basis);
}

bool
RigExecWireBasisInputsAreUsable(const GfVec2f *bindCoords, size_t bindCount,
                                size_t meshPointCount, size_t indexCount,
                                int order, const std::vector<double> &knots,
                                size_t controlPointCount)
{
    return RigExecWireBasisInputsAreUsableKernel(
        bindCoords, bindCount, meshPointCount, indexCount, order, knots.data(),
        knots.size(), controlPointCount);
}

bool
RigExecApplyWireBasis(std::vector<GfVec3f> *points,
                      const RigExecWireBasis &basis,
                      const std::vector<int> &indices,
                      const std::vector<float> &weights,
                      const std::vector<GfVec3f> &restControlPoints,
                      const std::vector<GfVec3f> &posedControlPoints)
{
    return RigExecApplyWireBasisRange(points, basis, indices, weights,
                                      restControlPoints, posedControlPoints, 0,
                                      points ? points->size() : 0);
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
    if (!points) {
        return false;
    }
    end = std::min(end, points->size());
    begin = std::min(begin, end);
    return RigExecApplyWireBasisGroup(points->data() + begin, begin, end, basis,
                                      indices, weights, restControlPoints,
                                      posedControlPoints);
}

bool
RigExecApplyWireBasisGroup(GfVec3f *out, size_t begin, size_t end,
                           const RigExecWireBasis &basis,
                           const std::vector<int> &indices,
                           const std::vector<float> &weights,
                           const std::vector<GfVec3f> &restControlPoints,
                           const std::vector<GfVec3f> &posedControlPoints)
{
    return RigExecApplyWireBasisGroupKernel(
        out, begin, end, basis, indices.data(), indices.size(), weights.data(),
        weights.size(), restControlPoints.data(), restControlPoints.size(),
        posedControlPoints.data(), posedControlPoints.size());
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
    if (!points) {
        return false;
    }
    end = std::min(end, points->size());
    begin = std::min(begin, end);
    return RigExecApplyWireGroup(points->data() + begin, begin, end,
                                 points->size(), restCurve, posedCurve,
                                 bindCoords, bindCount, dropoffDistance,
                                 restEvals, restEvalCount);
}

bool
RigExecApplyWireGroup(GfVec3f *out, size_t begin, size_t end, size_t count,
                      const RigExecNurbsCurve &restCurve,
                      const RigExecNurbsCurve &posedCurve,
                      const GfVec2f *bindCoords, size_t bindCount,
                      double dropoffDistance, const GfVec3f *restEvals,
                      size_t restEvalCount)
{
    return RigExecApplyWireGroupKernel<GfVec3f, GfVec3d>(
        out, begin, end, count, restCurve.Span(), posedCurve.Span(), bindCoords,
        bindCount, dropoffDistance, restEvals, restEvalCount);
}

bool
RigExecWireInputsAreUsable(const RigExecNurbsCurve &restCurve,
                           const RigExecNurbsCurve &posedCurve,
                           const GfVec2f *bindCoords, size_t bindCount,
                           size_t pointCount)
{
    return RigExecWireInputsAreUsableKernel(restCurve.Span(), posedCurve.Span(),
                                            bindCoords, bindCount, pointCount);
}

bool
RigExecApplyWireSparse(std::vector<GfVec3f> *points,
                       const RigExecNurbsCurve &restCurve,
                       const RigExecNurbsCurve &posedCurve,
                       const GfVec2f *bindCoords, size_t bindCount,
                       double dropoffDistance, const std::vector<int> &indices,
                       const std::vector<float> &weights)
{
    const RigExecNurbsCurveSpan<GfVec3f> rest = restCurve.Span();
    const RigExecNurbsCurveSpan<GfVec3f> posed = posedCurve.Span();
    if (!points || !bindCoords || !rest.IsValid() || !posed.IsValid() ||
        !rest.Matches(posed) ||
        (bindCount != points->size() && bindCount != indices.size()) ||
        indices.size() != weights.size()) {
        return false;
    }
    const bool parallel = bindCount == indices.size();
    for (size_t k = 0; k < indices.size(); ++k) {
        const size_t i = size_t(indices[k]);
        const GfVec2f &bind = bindCoords[parallel ? k : i];
        const double u = bind[0];
        const double f =
            weights[k] * RigExecWireFalloff(bind[1], dropoffDistance);
        if (f <= 0.0) {
            continue;
        }
        const GfVec3f delta =
            posed.Evaluate<GfVec3d>(u) - rest.Evaluate<GfVec3d>(u);
        (*points)[i] += delta * float(f);
    }
    return true;
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
