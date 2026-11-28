#ifndef RIGEXEC_MATH_SURFACE_BINDING_KERNEL_H
#define RIGEXEC_MATH_SURFACE_BINDING_KERNEL_H

/// The surface binding mover's arithmetic: each bound target vertex is a
/// weighted sum of stencils on a surface, each stencil a point on the
/// surface carried by its tangent frame and an offset in it (polygon or
/// limit-surface stencils; see docs/references.md). One definition for the
/// native mover and the SurfaceBinding op. V is a float point, D its double
/// twin and M a double 4x4, each with Gf's operators and members (GfVec3f,
/// GfVec3d and GfMatrix4d, or the runtime's RrVec3f, RrVec3d and RrMat4d).

#include "rigExecMath/rangeRunner.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

/// A read-only run of \p T; the storage outlives the call.
template <class T> struct RigExecSurfaceBindingList {
    const T *data = nullptr;
    size_t size = 0;

    const T &operator[](size_t i) const
    {
        return data[i];
    }
    bool empty() const
    {
        return size == 0;
    }
};

/// One surface binding: the bound target vertices, their stencils and
/// settings. vertexOffsets rows the offsets (stencils) by vertex;
/// polygonOffsets rows pointIndices and barycentricWeights by stencil, and
/// limitOffsets rows limitIndices and the limit weights by stencil.
template <class V> struct RigExecSurfaceBinding {
    RigExecSurfaceBindingList<int> vertices, vertexOffsets, polygonOffsets,
        pointIndices, limitOffsets, limitIndices;
    RigExecSurfaceBindingList<float> barycentricWeights, bindingWeights, mask,
        deltaMultipliers, limitWeights, limitDuWeights, limitDvWeights;
    RigExecSurfaceBindingList<V> offsets;
    /// Limit-surface stencils, else polygon ones.
    bool limit = false;
    /// Smooth surface normals, else each polygon's own.
    bool smooth = false;
    float strength = 1.0f;
    float deltaMultiplier = 1.0f;
};

namespace surfaceBindingDetail {

template <class V>
bool
Finite(const V &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

template <class V>
V
Cross(const V &a, const V &b)
{
    return V(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]);
}

template <class D, class V>
D
Widen(const V &v)
{
    return D(double(v[0]), double(v[1]), double(v[2]));
}

template <class V, class D>
V
Narrow(const D &d)
{
    return V(float(d[0]), float(d[1]), float(d[2]));
}

/// Row offsets over \p elements elements for \p rows rows, each row at
/// least \p minimum long.
inline bool
Offsets(const RigExecSurfaceBindingList<int> &a, size_t rows, size_t elements,
        int minimum)
{
    if (a.size != rows + 1 || a[0] != 0 || a[a.size - 1] < 0 ||
        size_t(a[a.size - 1]) != elements)
        return false;
    for (size_t i = 1; i < a.size; ++i)
        if (a[i] < a[i - 1] || a[i] - a[i - 1] < minimum)
            return false;
    return true;
}

/// Whether pred(i) holds for every i in [0, n). The checks are independent,
/// so \p ranges may split them; the answer is the same.
template <class Pred>
bool
AllOf(size_t n, const Pred &pred, const RigExecRangeRunner *ranges)
{
    std::atomic<bool> ok(true);
    RigExecRunRanges(ranges, n, 4096, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (!pred(i)) {
                ok.store(false, std::memory_order_relaxed);
                return;
            }
        }
    });
    return ok.load();
}

inline bool
AllFinite(const RigExecSurfaceBindingList<float> &a,
          const RigExecRangeRunner *ranges)
{
    const float *x = a.data;
    return AllOf(a.size, [x](size_t k) { return std::isfinite(x[k]); }, ranges);
}

inline bool
InRange(const RigExecSurfaceBindingList<int> &a, size_t n,
        const RigExecRangeRunner *ranges)
{
    const int *x = a.data;
    return AllOf(
        a.size, [x, n](size_t k) { return x[k] >= 0 && size_t(x[k]) < n; },
        ranges);
}

template <class V>
V
PolygonNormal(const V *p, const int *indices, size_t n)
{
    V normal(0.0f, 0.0f, 0.0f);
    for (size_t i = 0; i < n; ++i) {
        const V &a = p[indices[i]], &b = p[indices[(i + 1) % n]];
        normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
        normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
        normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    return normal;
}

} // namespace surfaceBindingDetail

/// Checks \p b against \p targetCount target points and the \p surfaceCount
/// points of the bound surface, whose faces \p counts and \p indices are
/// read for smooth normals, and moves \p surface into binding space: by the
/// inverse of \p surfaceRest, carried by frames[0] when \p frames is given,
/// then by \p surfaceToBinding. \p targetRest, carried by frames[1], becomes
/// \p toTarget. False, for inputs a binding cannot use, with \p surface
/// and \p toTarget unspecified.
template <class V, class D, class M>
bool
RigExecPrepareSurfaceBinding(const RigExecSurfaceBinding<V> &b,
                             size_t targetCount, M surfaceRest, M targetRest,
                             const M &surfaceToBinding, const M *frames,
                             const RigExecSurfaceBindingList<int> &counts,
                             const RigExecSurfaceBindingList<int> &indices,
                             V *surface, size_t surfaceCount, M *toTarget,
                             const RigExecRangeRunner *ranges = nullptr)
{
    using namespace surfaceBindingDetail;
    for (const RigExecSurfaceBindingList<float> *a :
         {&b.barycentricWeights, &b.bindingWeights, &b.mask,
          &b.deltaMultipliers, &b.limitWeights, &b.limitDuWeights,
          &b.limitDvWeights})
        if (!AllFinite(*a, ranges))
            return false;
    {
        const V *x = b.offsets.data;
        if (!AllOf(
                b.offsets.size, [x](size_t k) { return Finite(x[k]); }, ranges))
            return false;
    }
    if (!std::isfinite(b.strength) || !std::isfinite(b.deltaMultiplier))
        return false;
    if (frames) {
        surfaceRest *= frames[0];
        targetRest *= frames[1];
    }
    if (std::abs(surfaceRest.GetDeterminant()) < 1e-14)
        return false;
    const M toBinding = surfaceRest.GetInverse() * surfaceToBinding;
    {
        // Each point independently; a non-finite one fails whichever
        // thread meets it.
        std::atomic<bool> finite(true);
        RigExecRunRanges(
            ranges, surfaceCount, 2048, [&](size_t begin, size_t end) {
                for (size_t k = begin; k < end; ++k) {
                    surface[k] =
                        Narrow<V>(toBinding.Transform(Widen<D>(surface[k])));
                    if (!Finite(surface[k])) {
                        finite.store(false, std::memory_order_relaxed);
                        return;
                    }
                }
            });
        if (!finite.load())
            return false;
    }
    *toTarget = targetRest;
    if (b.smooth) {
        size_t count = 0;
        for (size_t f = 0; f < counts.size; ++f) {
            if (counts[f] < 3)
                return false;
            count += size_t(counts[f]);
        }
        if (count != indices.size || !InRange(indices, surfaceCount, ranges))
            return false;
    }
    // Every bound vertex in range and named once: the apply writes each
    // from its own thread.
    {
        std::vector<char> seen(targetCount, 0);
        for (size_t v = 0; v < b.vertices.size; ++v) {
            const int i = b.vertices[v];
            if (i < 0 || size_t(i) >= targetCount || seen[size_t(i)]++)
                return false;
        }
    }
    if (!InRange(b.pointIndices, surfaceCount, ranges))
        return false;
    if (!Offsets(b.vertexOffsets, b.vertices.size, b.offsets.size, 1) ||
        !Offsets(b.polygonOffsets, b.offsets.size, b.pointIndices.size, 3) ||
        b.barycentricWeights.size != b.pointIndices.size ||
        b.bindingWeights.size != b.offsets.size ||
        (!b.mask.empty() && b.mask.size != targetCount) ||
        (!b.deltaMultipliers.empty() && b.deltaMultipliers.size != targetCount))
        return false;
    {
        const float *x = b.mask.data;
        if (!AllOf(
                b.mask.size, [x](size_t k) { return !(x[k] < 0 || x[k] > 1); },
                ranges))
            return false;
    }
    if (b.limit) {
        if (!InRange(b.limitIndices, surfaceCount, ranges))
            return false;
        if (!Offsets(b.limitOffsets, b.offsets.size, b.limitIndices.size, 1) ||
            b.limitWeights.size != b.limitIndices.size ||
            b.limitDuWeights.size != b.limitIndices.size ||
            b.limitDvWeights.size != b.limitIndices.size)
            return false;
    }
    return true;
}

/// Moves the bound vertices of the \p count \p points toward \p b's
/// stencils on \p surface, in binding space, mapped by \p toTarget, each by
/// strength times its mask. \p b and \p surface have passed
/// RigExecPrepareSurfaceBinding. \p ranges, when given, splits the
/// vertices once the stencil work reaches RigExecGeometryParallelThreshold.
/// False, \p points untouched, where a vertex is out of range or a result
/// is not finite.
template <class V, class D, class M>
bool
RigExecApplySurfaceBinding(const RigExecSurfaceBinding<V> &b, const V *surface,
                           size_t surfaceCount,
                           const RigExecSurfaceBindingList<int> &counts,
                           const RigExecSurfaceBindingList<int> &indices,
                           const M &toTarget, V *points, size_t count,
                           const RigExecRangeRunner *ranges = nullptr)
{
    using namespace surfaceBindingDetail;
    const V *const p = surface;
    const auto &vertices = b.vertices;
    const auto &vo = b.vertexOffsets, &po = b.polygonOffsets,
               &pi = b.pointIndices;
    const auto &lo = b.limitOffsets, &li = b.limitIndices;
    const auto &weights = b.barycentricWeights, &bw = b.bindingWeights;
    const auto &lw = b.limitWeights, &du = b.limitDuWeights,
               &dv = b.limitDvWeights;
    const auto &mask = b.mask, &multipliers = b.deltaMultipliers;
    const auto &offsets = b.offsets;
    std::vector<V> normals(surfaceCount, V(0.0f, 0.0f, 0.0f));
    if (b.smooth) {
        size_t start = 0;
        for (size_t f = 0; f < counts.size; ++f) {
            const int n = counts[f];
            const V normal = PolygonNormal(p, indices.data + start, size_t(n));
            for (int j = 0; j < n; ++j)
                normals[size_t(indices[start + size_t(j)])] += normal;
            start += size_t(n);
        }
        for (V &n : normals)
            n.Normalize();
    }
    std::vector<V> candidate(points, points + count);
    // Every bound vertex is distinct, so each one is read and written by
    // exactly one iteration and the loop may be split. The work is the
    // stencil, not the vertex count: a harmonic-coordinate binding carries
    // hundreds of surface points per vertex.
    const size_t work = b.limit ? li.size : pi.size;
    const size_t grain = std::max<size_t>(
        1, std::min<size_t>(512,
                            1536 * vertices.size / std::max<size_t>(work, 1)));
    std::atomic<bool> good(true);
    const auto bindRange = [&](size_t begin, size_t end) {
        for (size_t v = begin; v < end; ++v) {
            if (!good.load(std::memory_order_relaxed))
                return;
            const int vertex = vertices[v];
            if (vertex < 0 || size_t(vertex) >= candidate.size()) {
                good = false;
                return;
            }
            const float deltaMultiplier =
                b.deltaMultiplier *
                (multipliers.empty() ? 1 : multipliers[size_t(vertex)]);
            V position(0.0f, 0.0f, 0.0f);
            for (int s = vo[v]; s < vo[v + 1]; ++s) {
                V base(0.0f, 0.0f, 0.0f), normal(0.0f, 0.0f, 0.0f),
                    tangent(0.0f, 0.0f, 0.0f);
                if (b.limit) {
                    V bitangent(0.0f, 0.0f, 0.0f);
                    for (int k = lo[size_t(s)]; k < lo[size_t(s) + 1]; ++k) {
                        base += p[li[size_t(k)]] * lw[size_t(k)];
                        tangent += p[li[size_t(k)]] * du[size_t(k)];
                        bitangent += p[li[size_t(k)]] * dv[size_t(k)];
                    }
                    normal = Cross(tangent, bitangent);
                } else {
                    for (int k = po[size_t(s)]; k < po[size_t(s) + 1]; ++k)
                        base += p[pi[size_t(k)]] * weights[size_t(k)];
                    normal = PolygonNormal(
                        p, pi.data + po[size_t(s)],
                        size_t(po[size_t(s) + 1] - po[size_t(s)]));
                    for (int k = po[size_t(s)] + 1; k < po[size_t(s) + 1];
                         ++k) {
                        tangent =
                            p[pi[size_t(k)]] - p[pi[size_t(po[size_t(s)])]];
                        if (tangent.GetLengthSq() > 1e-20f)
                            break;
                    }
                }
                if (b.smooth) {
                    normal = V(0.0f, 0.0f, 0.0f);
                    for (int k = po[size_t(s)]; k < po[size_t(s) + 1]; ++k)
                        normal +=
                            normals[size_t(pi[size_t(k)])] * weights[size_t(k)];
                }
                normal.Normalize();
                tangent -= normal * (normal * tangent);
                tangent.Normalize();
                const V bitangent = Cross(normal, tangent);
                const V transported = tangent * offsets[size_t(s)][0] +
                                      bitangent * offsets[size_t(s)][1] +
                                      normal * offsets[size_t(s)][2];
                position +=
                    (base + deltaMultiplier * transported) * bw[size_t(s)];
            }
            const float weight =
                b.strength * (mask.empty() ? 1 : mask[size_t(vertex)]);
            const V bound = Narrow<V>(toTarget.Transform(Widen<D>(position)));
            candidate[size_t(vertex)] +=
                weight * (bound - candidate[size_t(vertex)]);
            if (!Finite(candidate[size_t(vertex)])) {
                good = false;
                return;
            }
        }
    };
    RigExecRunRanges(work >= RigExecGeometryParallelThreshold &&
                             vertices.size > grain
                         ? ranges
                         : nullptr,
                     vertices.size, grain, bindRange);
    if (!good.load())
        return false;
    std::copy(candidate.begin(), candidate.end(), points);
    return true;
}

} // namespace rigExec

#endif
