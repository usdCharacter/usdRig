// Angle-weighted vertex normals and the extent of a point set. One
// definition for native evaluation, the runtime and the ops. V is a float
// point with Gf's operators. USD-free.
#ifndef RIGEXEC_MATH_NORMALS_KERNEL_H
#define RIGEXEC_MATH_NORMALS_KERNEL_H

#include "rangeRunner.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

/// The vertex-normal walk of one topology: the usable faces before the first
/// malformed one, and per point its (face, corner) incidences in face order
/// then corner order, which is the order the serial sum adds them in.
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
RigExecBuildVertexNormalsTopology(size_t pointsSize,
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
    // Faces before the first with fewer than three corners or running past
    // the index array.
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

/// Each face's unit Newell normal (correct for a non-planar n-gon) added to
/// its corners, weighted by the interior angle there; then each point's sum
/// normalized unless the topology is malformed. \p topology must Fit the
/// arrays. Per-point sums keep face then corner order, so \p ranges does
/// not change the bits.
template <class V>
std::vector<V>
RigExecVertexNormalsKernel(const V *points, size_t pointsSize,
                           const int *faceVertexCounts,
                           const int *faceVertexIndices,
                           const RigExecVertexNormalsTopology &topology,
                           const RigExecRangeRunner *ranges)
{
    std::vector<V> normals(pointsSize, V(0.0f));
    const std::vector<size_t> &faceOffsets = topology.faceOffsets;
    const size_t faces = faceOffsets.size();
    const size_t corners =
        faces ? faceOffsets.back() + size_t(faceVertexCounts[faces - 1]) : 0;
    const auto isPoint = [pointsSize](int v) {
        return v >= 0 && static_cast<size_t>(v) < pointsSize;
    };

    std::vector<V> faceNormals(faces, V(0.0f));
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
            if (n < 3)
                continue;
            V faceNormal(0.0f);
            for (size_t k = 0; k < n; ++k) {
                const V &p = points[ring[k]];
                const V &q = points[ring[(k + 1) % n]];
                faceNormal += V((p[1] - q[1]) * (p[2] + q[2]),
                                (p[2] - q[2]) * (p[0] + q[0]),
                                (p[0] - q[0]) * (p[1] + q[1]));
            }
            const float faceLength = faceNormal.GetLength();
            if (faceLength < 1e-20f)
                continue;
            faceNormal /= faceLength;
            faceNormals[f] = faceNormal;
            contributes[f] = 1;
            // Zero-length edges fall back to an unweighted share.
            for (size_t k = 0; k < n; ++k) {
                const V &p = points[ring[k]];
                const V a = points[ring[(k + 1) % n]] - p;
                const V b = points[ring[(k + n - 1) % n]] - p;
                const float la = a.GetLength(), lb = b.GetLength();
                float weight = 1.0f;
                if (la > 1e-20f && lb > 1e-20f) {
                    const V ua = a / la, ub = b / lb;
                    const V cross(ua[1] * ub[2] - ua[2] * ub[1],
                                  ua[2] * ub[0] - ua[0] * ub[2],
                                  ua[0] * ub[1] - ua[1] * ub[0]);
                    weight = std::atan2(cross.GetLength(), ua[0] * ub[0] +
                                                               ua[1] * ub[1] +
                                                               ua[2] * ub[2]);
                }
                cornerWeights[slots[k]] = weight;
            }
        }
    });

    const bool normalize = !topology.malformed;
    RigExecRunRanges(ranges, pointsSize, 2048, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            V sum(0.0f);
            for (size_t s = topology.pointIncidences[i];
                 s < topology.pointIncidences[i + 1]; ++s) {
                const size_t f = topology.incidenceFaces[s];
                if (contributes[f])
                    sum += faceNormals[f] *
                           cornerWeights[topology.incidenceCorners[s]];
            }
            if (normalize) {
                const float len = sum.GetLength();
                if (len > 1e-12f)
                    sum /= len;
            }
            normals[i] = sum;
        }
    });
    return normals;
}

/// (min, max) of \p points, widened by half the width: one per point, or
/// one for all. Empty for no points.
template <class V>
std::vector<V>
RigExecExtentKernel(const V *points, size_t pointsSize, const float *widths,
                    size_t widthsSize)
{
    if (pointsSize == 0 || !points)
        return {};
    const size_t widthsCount = widths ? widthsSize : 0;
    V lo = points[0], hi = points[0];
    for (size_t i = 0; i < pointsSize; ++i) {
        float pad = 0.0f;
        if (widthsCount == pointsSize)
            pad = widths[i] * 0.5f;
        else if (widthsCount == 1)
            pad = widths[0] * 0.5f;
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a] - pad);
            hi[a] = std::max(hi[a], points[i][a] + pad);
        }
    }
    return {lo, hi};
}

} // namespace rigExec

#endif
