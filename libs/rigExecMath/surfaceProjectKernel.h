// The legacy surface projection: each point moves toward its nearest point
// on the surface's fan triangulation. One definition for native evaluation,
// the runtime and the ops. V is GfVec3f or RrVec3f. USD-free.
#ifndef RIGEXEC_MATH_SURFACE_PROJECT_KERNEL_H
#define RIGEXEC_MATH_SURFACE_PROJECT_KERNEL_H

#include <algorithm>
#include <cstddef>
#include <limits>

namespace rigExec {

/// Nearest point to \p p on triangle (a, b, c). Ericson, Real-Time
/// Collision Detection, 5.1.5.
template <class V>
V
RigExecClosestPointOnTriangleKernel(const V &p, const V &a, const V &b,
                                    const V &c)
{
    const auto dot = [](const V &x, const V &y) {
        return float(x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
    };
    const V ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0)
        return a;
    const V bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3)
        return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }
    const V cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6)
        return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    return a + ab * v + ac * w;
}

/// Moves each of \p count points by \p weight (clamped to [0, 1]) toward
/// its nearest point on the fans of (counts, indices) over \p surface. Out
/// of range corners skip their triangle; an index walk that overruns
/// leaves every point as it was. The brute-force search; a native caller
/// may answer the same query from a BVH.
template <class V>
void
RigExecApplySurfaceProjectKernel(V *points, size_t count, const V *surface,
                                 size_t surfaceSize, const int *counts,
                                 size_t countsSize, const int *indices,
                                 size_t indicesSize, double weight)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (count == 0 || surfaceSize == 0 || w <= 0.0) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        V &p = points[i];
        float bestDistSq = std::numeric_limits<float>::max();
        V best = p;
        size_t offset = 0;
        for (size_t f = 0; f < countsSize; ++f) {
            const int faceCount = counts[f];
            for (int c = 1; c + 1 < faceCount; ++c) {
                const size_t i2 = offset + c + 1;
                if (i2 >= indicesSize) {
                    // Met during the first point, before any moved.
                    return;
                }
                const int ia = indices[offset];
                const int ib = indices[offset + c];
                const int ic = indices[i2];
                if (ia < 0 || ib < 0 || ic < 0 || size_t(ia) >= surfaceSize ||
                    size_t(ib) >= surfaceSize || size_t(ic) >= surfaceSize) {
                    continue;
                }
                const V q = RigExecClosestPointOnTriangleKernel(
                    p, surface[ia], surface[ib], surface[ic]);
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
