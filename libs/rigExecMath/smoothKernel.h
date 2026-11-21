// The Laplacian smooth's vertex-group form: one Jacobi pass over the edge
// adjacency for the points of one group, reading neighbours that may sit in
// other groups. Header-only and USD-free, so the USD evaluators and the
// runtime run one definition. V is GfVec3f or the runtime's RrVec3f, whose
// scalar operators share Gf's semantics (division multiplies by 1.0 / s in
// double); that is what keeps a group bit for bit the whole kernel's.
#ifndef RIGEXEC_MATH_SMOOTH_KERNEL_H
#define RIGEXEC_MATH_SMOOTH_KERNEL_H

#include "meshConnectivity.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace rigExec {

/// The entering points of the vertex groups one group step reads. Group h
/// holds points [bounds[h], bounds[h + 1]) at points[h]; points[h] is null
/// for a group the step does not read. bounds holds groups + 1 entries.
template <class V> struct RigExecGroupPoints {
    const int *bounds = nullptr;
    size_t groups = 0;
    const V *const *points = nullptr;

    /// Point \p i, or null when its group is not read.
    const V *At(size_t i) const
    {
        if (!bounds || !points || groups == 0) {
            return nullptr;
        }
        const int *last = bounds + groups + 1;
        const int *it = std::upper_bound(bounds, last, int(i));
        if (it == bounds || it == last) {
            return nullptr;
        }
        const size_t h = size_t(it - bounds) - 1;
        const V *base = points[h];
        return base ? base + (i - size_t(bounds[h])) : nullptr;
    }
};

// Whole and group smoothing share the Jacobi walk. Only their source
// lookup and the need to copy untouched vertices to the output differ.
template <class Point, class Read>
bool
RigExecApplyLaplacianSmoothKernel(const RigExecMeshAdjacency &adjacency,
                                  double strength, size_t count, Read read,
                                  size_t begin, size_t end, Point *out,
                                  bool copyUntouched)
{
    if (begin > end || end > count) {
        return false;
    }
    const double s = std::min(std::max(strength, 0.0), 1.0);
    const bool passThrough = s <= 0.0 || !adjacency.Covers(count);
    for (size_t i = begin; i < end; ++i) {
        const Point *self = read(i);
        if (!self) {
            return false;
        }
        if (passThrough || adjacency.neighbors[i].empty()) {
            if (copyUntouched) {
                out[i - begin] = *self;
            }
            continue;
        }
        const auto &ring = adjacency.neighbors[i];
        Point average(0.0f);
        for (int n : ring) {
            const Point *p = read(size_t(n));
            if (!p) {
                return false;
            }
            average += *p;
        }
        average /= float(ring.size());
        out[i - begin] = *self + (average - *self) * float(s);
    }
    return true;
}

/// False when a point or neighbour lies outside the groups the step reads.
template <class Point>
bool
RigExecApplyLaplacianSmoothGroup(const RigExecMeshAdjacency &adjacency,
                                 double strength, size_t count,
                                 const RigExecGroupPoints<Point> &groups,
                                 size_t begin, size_t end, Point *out)
{
    return RigExecApplyLaplacianSmoothKernel(
        adjacency, strength, count, [&](size_t i) { return groups.At(i); },
        begin, end, out, true);
}

/// Borrow an entering range, copy the points, or swap caller-owned scratch
/// into the destination. Isolated vertices only need copying after a swap.
template <class Point>
void
RigExecApplyLaplacianSmoothWithAdjacencyKernel(
    std::vector<Point> *points, const RigExecMeshAdjacency &adjacency,
    double strength, const Point *source, size_t sourceCount,
    std::vector<Point> *scratch = nullptr)
{
    const size_t count = points->size();
    if (count == 0 || strength <= 0.0 || !adjacency.Covers(count) ||
        (source && sourceCount != count)) {
        return;
    }
    std::vector<Point> owned;
    const bool swapSource = !source && scratch;
    if (swapSource) {
        scratch->swap(*points);
        points->resize(count);
        source = scratch->data();
    } else if (!source) {
        owned = *points;
        source = owned.data();
    }
    RigExecApplyLaplacianSmoothKernel(
        adjacency, strength, count, [source](size_t i) { return source + i; },
        0, count, points->data(), swapSource);
}

template <class Point>
void
RigExecApplyLaplacianSmoothMeshKernel(std::vector<Point> *points,
                                      const int *counts, size_t countsSize,
                                      const int *indices, size_t indicesSize,
                                      double strength,
                                      const Point *source = nullptr,
                                      size_t sourceCount = 0)
{
    if (points->empty() || strength <= 0.0) {
        return;
    }
    RigExecMeshAdjacency adjacency;
    if (RigExecBuildMeshAdjacency(points->size(), counts, countsSize, indices,
                                  indicesSize, &adjacency)) {
        RigExecApplyLaplacianSmoothWithAdjacencyKernel(
            points, adjacency, strength, source, sourceCount);
    }
}

/// The groups a group's points reach through one ring of \p adjacency:
/// out[g] lists, ascending, every group other than g holding a neighbour of
/// a point of group g. Empty lists when the adjacency does not cover the
/// bounds' points.
inline void
RigExecSmoothHaloGroups(const RigExecMeshAdjacency &adjacency,
                        const std::vector<int> &bounds,
                        std::vector<std::vector<int>> *out)
{
    const size_t groups = bounds.empty() ? 0 : bounds.size() - 1;
    out->assign(groups, {});
    if (groups == 0 || !adjacency.Covers(size_t(bounds.back()))) {
        return;
    }
    std::vector<char> seen(groups, 0);
    for (size_t g = 0; g < groups; ++g) {
        std::fill(seen.begin(), seen.end(), char(0));
        seen[g] = 1;
        for (int i = bounds[g]; i < bounds[g + 1]; ++i) {
            for (const int n : adjacency.neighbors[size_t(i)]) {
                const size_t h =
                    size_t(std::upper_bound(bounds.begin(), bounds.end(), n) -
                           bounds.begin()) -
                    1;
                if (h < groups && !seen[h]) {
                    seen[h] = 1;
                    (*out)[g].push_back(int(h));
                }
            }
        }
        std::sort((*out)[g].begin(), (*out)[g].end());
    }
}

} // namespace rigExec

#endif // RIGEXEC_MATH_SMOOTH_KERNEL_H
