// Mesh bounds and whole-mesh volume correction over the entering points.
#ifndef RIGEXEC_MATH_MESH_BOUNDS_KERNEL_H
#define RIGEXEC_MATH_MESH_BOUNDS_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

template <class Point>
double
RigExecBoundVolumeKernel(const Point *points, size_t count)
{
    if (count < 2) {
        return 0.0;
    }
    Point lo = points[0], hi = points[0];
    for (size_t i = 1; i < count; ++i) {
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a]);
            hi[a] = std::max(hi[a], points[i][a]);
        }
    }
    const Point d = hi - lo;
    return double(d[0]) * double(d[1]) * double(d[2]);
}

template <class Point>
void
RigExecApplyVolumeCorrectKernel(std::vector<Point> *points,
                                double referenceVolume, double strength)
{
    if (points->empty() || referenceVolume <= 0.0 || strength <= 0.0) {
        return;
    }
    const double current =
        RigExecBoundVolumeKernel(points->data(), points->size());
    if (current <= 0.0) {
        return; // degenerate bound: no deterministic correction exists
    }
    // Uniform scale about the centroid moving the bound volume toward the
    // reference; strength blends the correction.
    const double full = std::cbrt(referenceVolume / current);
    const double scale =
        1.0 + std::min(std::max(strength, 0.0), 1.0) * (full - 1.0);
    Point centroid(0.0f);
    for (const Point &p : *points) {
        centroid += p;
    }
    centroid /= float(points->size());
    for (Point &p : *points) {
        p = centroid + (p - centroid) * float(scale);
    }
}

template <class Point>
std::vector<Point>
RigExecComputeExtentKernel(const Point *points, size_t pointsSize,
                           const float *widths, size_t widthsSize)
{
    if (pointsSize == 0 || !points) {
        return {};
    }
    // A null widths range reads as no widths, whatever its size claims.
    const size_t widthsCount = widths ? widthsSize : 0;
    Point lo = points[0], hi = points[0];
    for (size_t i = 0; i < pointsSize; ++i) {
        float pad = 0.0f;
        if (widthsCount == pointsSize) {
            pad = widths[i] * 0.5f;
        } else if (widthsCount == 1) {
            pad = widths[0] * 0.5f;
        }
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a] - pad);
            hi[a] = std::max(hi[a], points[i][a] + pad);
        }
    }
    return {lo, hi};
}

} // namespace rigExec

#endif // RIGEXEC_MATH_MESH_BOUNDS_KERNEL_H
