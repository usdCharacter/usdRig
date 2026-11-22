// Nearest-surface projection with optional retained acceleration.
#ifndef RIGEXEC_MATH_SURFACE_PROJECT_KERNEL_H
#define RIGEXEC_MATH_SURFACE_PROJECT_KERNEL_H

#include "spatialAccel.h"
#include "surfaceSnapKernel.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace rigExec {

namespace surfaceProjectDetail {

// Whether an owned key still describes a borrowed range: same size and
// same values. A null range only ever matches the empty key, and the
// projector returns before validation on a null range with a nonzero
// size, so every validated call below compares against real storage.
template <typename T>
bool
RangeMatches(const std::vector<T> &key, const T *data, size_t size)
{
    return key.size() == size &&
           (size == 0 || std::equal(key.begin(), key.end(), data));
}

} // namespace surfaceProjectDetail

template <class Point, class Accel>
void
RigExecBuildSurfaceAccelKernel(const int *faceVertexCounts,
                               size_t faceVertexCountsSize,
                               const int *faceVertexIndices,
                               size_t faceVertexIndicesSize,
                               const Point *surfacePoints,
                               size_t surfacePointsSize, Accel *accel)
{
    if (!accel) {
        return;
    }
    Accel built;
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
        auto bvh = std::make_shared<RigExecTriangleBvh<Point>>();
        if (bvh->Build(surfacePoints, surfacePointsSize, built.fan->tris.data(),
                       built.fan->tris.size())) {
            built.bvh = std::move(bvh);
        }
        // Build failed (non-finite surface points): the null bvh runs the
        // verbatim loop, whose NaN behavior is the defined one.
    }
    *accel = std::move(built);
}

template <class Point, class Accel = std::nullptr_t>
void
RigExecApplySurfaceProjectKernel(
    std::vector<Point> *points, const Point *surfacePoints,
    size_t surfacePointsSize, const int *faceVertexCounts,
    size_t faceVertexCountsSize, const int *faceVertexIndices,
    size_t faceVertexIndicesSize, double weight, const Accel *accel = nullptr)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (points->empty() || surfacePointsSize == 0 || w <= 0.0) {
        return;
    }
    if (!surfacePoints || (faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        return; // a null range with a nonzero size: pass through
    }
    // Only callers opting into acceleration build or validate a retained fan.
    if constexpr (!std::is_same_v<Accel, std::nullptr_t>) {
        // One fan triangulation for the whole call instead of one per point.
        // A truncated walk returns with no point modified: the nested walk
        // below always meets the overrun during its first point.
        Accel unretained;
        // A stale entry -- same cardinality but new topology or, on the
        // BVH path, new points -- rebuilds rather than answering wrong.
        // The fan only depends on the topology and the point count, so
        // below the gate the values are not compared: the verbatim loop
        // walks the current inputs either way.
        if (!accel || !accel->fan ||
            !surfaceProjectDetail::RangeMatches(accel->counts, faceVertexCounts,
                                                faceVertexCountsSize) ||
            !surfaceProjectDetail::RangeMatches(
                accel->indices, faceVertexIndices, faceVertexIndicesSize) ||
            accel->points.size() != surfacePointsSize ||
            (accel->bvh &&
             !surfaceProjectDetail::RangeMatches(accel->points, surfacePoints,
                                                 surfacePointsSize))) {
            RigExecBuildSurfaceAccelKernel(
                faceVertexCounts, faceVertexCountsSize, faceVertexIndices,
                faceVertexIndicesSize, surfacePoints, surfacePointsSize,
                &unretained);
            accel = &unretained;
        }
        const RigExecFanTrisH7 &fan = *accel->fan;
        if (fan.truncated) {
            return;
        }
        if (fan.tris.size() >= kRigExecBvhMinTris &&
            points->size() >= kRigExecBvhMinQueries && accel->bvh) {
            for (Point &p : *points) {
                const Point best =
                    accel->bvh->QueryNearest(p, surfacePoints, fan.tris.data(),
                                             surfaceSnapDetail::Closest<Point>);
                p = p + (best - p) * float(w);
            }
            return;
        }
    }
    for (Point &p : *points) {
        float bestDistSq = std::numeric_limits<float>::max();
        Point best = p;
        size_t offset = 0;
        for (size_t f = 0; f < faceVertexCountsSize; ++f) {
            const int faceCount = faceVertexCounts[f];
            for (int c = 1; c + 1 < faceCount; ++c) {
                const size_t i2 = offset + c + 1;
                if (i2 >= faceVertexIndicesSize) {
                    return;
                }
                const int ia = faceVertexIndices[offset];
                const int ib = faceVertexIndices[offset + c];
                const int ic = faceVertexIndices[i2];
                if (ia < 0 || ib < 0 || ic < 0 ||
                    size_t(ia) >= surfacePointsSize ||
                    size_t(ib) >= surfacePointsSize ||
                    size_t(ic) >= surfacePointsSize) {
                    continue;
                }
                const Point q = surfaceSnapDetail::Closest<Point>(
                    p, surfacePoints[ia], surfacePoints[ib], surfacePoints[ic]);
                const float distSq = (q - p).GetLengthSq();
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best = q;
                }
            }
            offset += faceCount;
        }
        p = p + (best - p) * float(w);
    }
}

} // namespace rigExec

#endif // RIGEXEC_MATH_SURFACE_PROJECT_KERNEL_H
