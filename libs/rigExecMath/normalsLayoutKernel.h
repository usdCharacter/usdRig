// Vertex normals and their authored interpolation/orientation layout.
#ifndef RIGEXEC_MATH_NORMALS_LAYOUT_KERNEL_H
#define RIGEXEC_MATH_NORMALS_LAYOUT_KERNEL_H

#include "rangeRunner.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

/// The vertex-normal walk of one topology, built once per compiled rig:
/// the usable faces before the first malformed one, and per point its
/// (face, corner) incidences in face order then corner order, which is the
/// order the serial sum adds them in.
struct RigExecVertexNormalsTopology {
    size_t pointCount = 0;
    size_t faceVertexCountsSize = 0;
    size_t faceVertexIndicesSize = 0;
    bool malformed = false;
    std::vector<size_t> faceOffsets;
    /// pointCount + 1 offsets into incidenceFaces / incidenceCorners.
    std::vector<size_t> pointIncidences;
    std::vector<size_t> incidenceFaces;
    std::vector<size_t> incidenceCorners;

    /// Whether this walk was built for arrays of these sizes. The contents
    /// are trusted: a topology that can change is never given a walk.
    bool Fits(size_t points, size_t counts, size_t indices) const
    {
        return pointCount == points && faceVertexCountsSize == counts &&
               faceVertexIndicesSize == indices;
    }
};

inline RigExecVertexNormalsTopology
RigExecBuildVertexNormalsTopologyKernel(size_t pointsSize,
                                        const int *faceVertexCounts,
                                        size_t faceVertexCountsSize,
                                        const int *faceVertexIndices,
                                        size_t faceVertexIndicesSize)
{
    RigExecVertexNormalsTopology t;
    t.pointCount = pointsSize;
    t.pointIncidences.assign(pointsSize + 1, 0);
    if ((faceVertexCountsSize > 0 && !faceVertexCounts) ||
        (faceVertexIndicesSize > 0 && !faceVertexIndices)) {
        t.pointCount = size_t(-1);
        return t;
    }
    t.faceVertexCountsSize = faceVertexCountsSize;
    t.faceVertexIndicesSize = faceVertexIndicesSize;
    // The faces before the first malformed one (fewer than three corners,
    // or running past the index array).
    t.faceOffsets.reserve(faceVertexCountsSize);
    size_t offset = 0;
    for (size_t f = 0; f < faceVertexCountsSize; ++f) {
        const int faceCount = faceVertexCounts[f];
        if (faceCount < 3 ||
            offset + static_cast<size_t>(faceCount) > faceVertexIndicesSize) {
            t.malformed = true;
            break;
        }
        t.faceOffsets.push_back(offset);
        offset += faceCount;
    }
    const auto isPoint = [pointsSize](int v) {
        return v >= 0 && static_cast<size_t>(v) < pointsSize;
    };
    for (size_t c = 0; c < offset; ++c) {
        const int v = faceVertexIndices[c];
        if (isPoint(v))
            ++t.pointIncidences[size_t(v) + 1];
    }
    for (size_t i = 0; i < pointsSize; ++i)
        t.pointIncidences[i + 1] += t.pointIncidences[i];
    t.incidenceFaces.resize(t.pointIncidences[pointsSize]);
    t.incidenceCorners.resize(t.pointIncidences[pointsSize]);
    std::vector<size_t> cursor(t.pointIncidences.begin(),
                               t.pointIncidences.end() - 1);
    for (size_t f = 0; f < t.faceOffsets.size(); ++f) {
        const size_t first = t.faceOffsets[f];
        const int faceCount = faceVertexCounts[f];
        for (int k = 0; k < faceCount; ++k) {
            const int v = faceVertexIndices[first + k];
            if (!isPoint(v))
                continue;
            const size_t slot = cursor[size_t(v)]++;
            t.incidenceFaces[slot] = f;
            t.incidenceCorners[slot] = first + k;
        }
    }
    return t;
}

namespace normalsDetail {

template <class Point>
std::vector<Point>
ComputeVertexNormals(const Point *points, size_t pointsSize,
                     const int *faceVertexCounts, const int *faceVertexIndices,
                     const RigExecVertexNormalsTopology &topology,
                     const RigExecRangeRunner *ranges)
{
    std::vector<Point> normals(pointsSize, Point(0.0f));
    const std::vector<size_t> &faceOffsets = topology.faceOffsets;
    const size_t faces = faceOffsets.size();
    const size_t corners =
        faces ? faceOffsets.back() + size_t(faceVertexCounts[faces - 1]) : 0;
    const auto isPoint = [pointsSize](int v) {
        return v >= 0 && static_cast<size_t>(v) < pointsSize;
    };

    // Per face, independently: its unit Newell normal and each usable
    // corner's interior angle. A face that contributes nothing (fewer than
    // three usable corners, or fully degenerate) is marked and skipped.
    std::vector<Point> faceNormals(faces, Point(0.0f));
    std::vector<char> contributes(faces, 0);
    std::vector<float> cornerWeights(corners, 0.0f);
    RigExecRunRanges(ranges, faces, 512, [&](size_t begin, size_t end) {
        std::vector<int> ring;
        std::vector<size_t> slots;
        for (size_t f = begin; f < end; ++f) {
            const size_t first = faceOffsets[f];
            const int faceCount = faceVertexCounts[f];
            ring.clear();
            slots.clear();
            for (int k = 0; k < faceCount; ++k) {
                const int v = faceVertexIndices[first + k];
                if (isPoint(v)) {
                    ring.push_back(v);
                    slots.push_back(first + k);
                }
            }
            const size_t n = ring.size();
            if (n < 3) {
                continue;
            }

            // Newell's method: correct for a non-planar n-gon and
            // independent of triangulation. A fan triangulation gives a
            // vertex beside the anchor only a sliver triangle, and mirrored
            // slivers along a symmetry seam cancel to a zero normal.
            Point faceNormal(0.0f);
            for (size_t k = 0; k < n; ++k) {
                const Point &p = points[ring[k]];
                const Point &q = points[ring[(k + 1) % n]];
                faceNormal += Point((p[1] - q[1]) * (p[2] + q[2]),
                                    (p[2] - q[2]) * (p[0] + q[0]),
                                    (p[0] - q[0]) * (p[1] + q[1]));
            }
            const float faceLength = faceNormal.GetLength();
            if (faceLength < 1e-20f) {
                continue;
            }
            faceNormal /= faceLength;
            faceNormals[f] = faceNormal;
            contributes[f] = 1;

            // Weighted by the interior angle at each corner; zero-length
            // edges fall back to an unweighted share.
            for (size_t k = 0; k < n; ++k) {
                const Point &p = points[ring[k]];
                const Point a = points[ring[(k + 1) % n]] - p;
                const Point b = points[ring[(k + n - 1) % n]] - p;
                const float la = a.GetLength(), lb = b.GetLength();
                float weight = 1.0f;
                if (la > 1e-20f && lb > 1e-20f) {
                    const Point ua = a / la, ub = b / lb;
                    weight = std::atan2((ua ^ ub).GetLength(), ua * ub);
                }
                cornerWeights[slots[k]] = weight;
            }
        }
    });

    // Each point sums its own incidences in face order then corner order,
    // the sequence of a single serial walk, so its bits do not depend on
    // how the points are split. A malformed topology leaves the sums
    // unnormalized.
    const bool normalize = !topology.malformed;
    RigExecRunRanges(ranges, pointsSize, 2048, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            Point sum(0.0f);
            for (size_t s = topology.pointIncidences[i];
                 s < topology.pointIncidences[i + 1]; ++s) {
                const size_t f = topology.incidenceFaces[s];
                if (contributes[f]) {
                    sum += faceNormals[f] *
                           cornerWeights[topology.incidenceCorners[s]];
                }
            }
            if (normalize) {
                const float len = sum.GetLength();
                if (len > 1e-12f) {
                    sum /= len;
                }
            }
            normals[i] = sum;
        }
    });
    return normals;
}

} // namespace normalsDetail

template <class Point>
std::vector<Point>
RigExecComputeVertexNormalsKernel(
    const Point *points, size_t pointsSize, const int *counts,
    size_t countsSize, const int *indices, size_t indicesSize,
    const RigExecVertexNormalsTopology *topology = nullptr,
    const RigExecRangeRunner *ranges = nullptr)
{
    if ((pointsSize && !points) || (countsSize && !counts) ||
        (indicesSize && !indices) ||
        (topology && !topology->Fits(pointsSize, countsSize, indicesSize))) {
        return std::vector<Point>(pointsSize, Point(0.0f));
    }
    RigExecVertexNormalsTopology built;
    if (!topology) {
        built = RigExecBuildVertexNormalsTopologyKernel(
            pointsSize, counts, countsSize, indices, indicesSize);
        topology = &built;
    }
    return normalsDetail::ComputeVertexNormals(points, pointsSize, counts,
                                               indices, *topology, ranges);
}

template <class Point>
std::vector<Point>
RigExecComputeMeshVertexNormalsKernel(const std::vector<Point> &points,
                                      const std::vector<int> &counts,
                                      const std::vector<int> &indices)
{
    return RigExecComputeVertexNormalsKernel(points.data(), points.size(),
                                             counts.data(), counts.size(),
                                             indices.data(), indices.size());
}

/// Lays \p normals (one per point) out as the authored normals store them:
/// one per face corner through \p cornerPoints (faceVertexIndices) when
/// \p perCorner, and negated on a leftHanded mesh. False, leaving
/// \p normals unchanged, when a corner names no point.
template <class Vec3>
bool
RigExecLayoutNormals(std::vector<Vec3> *normals, const int *cornerPoints,
                     size_t cornerCount, bool perCorner, bool leftHanded)
{
    if (perCorner) {
        if (cornerCount > 0 && !cornerPoints)
            return false;
        std::vector<Vec3> corners(cornerCount);
        for (size_t c = 0; c < cornerCount; ++c) {
            const int v = cornerPoints[c];
            if (v < 0 || static_cast<size_t>(v) >= normals->size())
                return false;
            corners[c] = (*normals)[static_cast<size_t>(v)];
        }
        normals->swap(corners);
    }
    if (leftHanded)
        for (Vec3 &n : *normals)
            n = -n;
    return true;
}

} // namespace rigExec

#endif
