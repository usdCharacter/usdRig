// Bound-volume correction: one definition for native evaluation and the
// runtime. V is GfVec3f or RrVec3f, whose scalar operators share Gf's
// semantics. USD-free.
#ifndef RIGEXEC_MATH_VOLUME_CORRECT_KERNEL_H
#define RIGEXEC_MATH_VOLUME_CORRECT_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rigExec {

/// The axis-aligned bound's volume; 0 below two points.
template <class V>
double
RigExecBoundVolumeKernel(const V *points, size_t count)
{
    if (count < 2) {
        return 0.0;
    }
    V lo = points[0], hi = points[0];
    for (size_t i = 1; i < count; ++i) {
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a]);
            hi[a] = std::max(hi[a], points[i][a]);
        }
    }
    const V d = hi - lo;
    return double(d[0]) * double(d[1]) * double(d[2]);
}

/// Uniform scale about the centroid moving the bound volume toward
/// \p referenceVolume; \p strength, clamped to [0, 1], blends the
/// correction. A degenerate bound or non-positive reference or strength
/// passes the points through.
template <class V>
void
RigExecApplyVolumeCorrectKernel(V *points, size_t count, double referenceVolume,
                                double strength)
{
    if (count == 0 || referenceVolume <= 0.0 || strength <= 0.0) {
        return;
    }
    const double current = RigExecBoundVolumeKernel(points, count);
    if (current <= 0.0) {
        return;
    }
    const double full = std::cbrt(referenceVolume / current);
    const double scale =
        1.0 + std::min(std::max(strength, 0.0), 1.0) * (full - 1.0);
    V centroid(0.0f);
    for (size_t i = 0; i < count; ++i) {
        centroid += points[i];
    }
    centroid /= float(count);
    for (size_t i = 0; i < count; ++i) {
        points[i] = centroid + (points[i] - centroid) * float(scale);
    }
}

} // namespace rigExec

#endif
