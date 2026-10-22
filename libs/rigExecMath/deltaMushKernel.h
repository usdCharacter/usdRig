// Method reference: Mancewicz et al. (2014), https://doi.org/10.1145/2614106.2614144
// Shared, USD-free geometry math. Both USD and binary adapters instantiate
// these kernels; Point/Wide only supply float/double vector storage and arithmetic.
#ifndef RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#define RIGEXEC_MATH_DELTA_MUSH_KERNEL_H
#include "deltaMushSettings.h"
#include "meshConnectivity.h"
#include "floatControl.h"
#include "rangeRunner.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {
namespace deltaMushDetail {
template<class To, class From> To Convert(const From &p) {
    return To(p[0], p[1], p[2]);
}
template<class V> double Dot(const V &a, const V &b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
template<class V> V Cross(const V &a, const V &b) {
    return V(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);
}
template<class Wide> bool Normalize(Wide *value) {
    const double length = value->GetLength();
    if (!(length > 0) || !std::isfinite(length)) return false;
    *value /= length;
    return true;
}
// One definition, shared with the smooth kernel: the same sorted neighbor
// lists the set-based build produced, without a node per edge endpoint.
inline std::vector<std::vector<int>>
BuildAdjacency(
    size_t pointCount,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices)
{
    RigExecMeshAdjacency built;
    if (!RigExecBuildMeshAdjacency(
            pointCount, faceVertexCounts, faceVertexIndices, &built)) {
        return {};
    }
    return built.neighbors;
}

} // namespace deltaMushDetail

/// The rest half of a surface-offset transport: everything the kernel
/// derives from (rest, faceCounts, faceIndices) rather than from the posed
/// surface or the offsets. Built once for rest state shared across frames
/// or nodes; the posed half reads it, it never re-derives it.
template<class Point, class Wide>
struct RigExecTransportRestData {
    size_t pointCount = 0;
    /// Unnormalized Newell area sums over the rest surface.
    std::vector<Wide> restNormals;
    std::vector<std::vector<int>> adjacency;
    /// Per-vertex longest projected rest edge (see the kernel contract)
    /// and the rest frame it spans. Meaningful only where restOk holds.
    std::vector<int> neighbor;
    std::vector<Wide> nr;
    std::vector<Wide> tr;
    std::vector<Wide> br;
    /// Per-vertex rest-frame validity. A zero offset never consults it --
    /// "a zero offset needs no valid frame" -- so recording a failure here
    /// instead of returning one preserves the kernel's skip-then-fail
    /// behavior exactly.
    std::vector<bool> restOk;
    /// Whether every member covers \p n vertices, so the apply half fails
    /// closed on a hand-built entry instead of indexing past an end.
    bool Covers(size_t n) const {
        return pointCount == n && restNormals.size() == n &&
               adjacency.size() == n && neighbor.size() == n &&
               nr.size() == n && tr.size() == n && br.size() == n &&
               restOk.size() == n;
    }
};

/// Build the rest half of a transport: validate, accumulate rest normals,
/// build adjacency, and choose every vertex's rest frame. False leaves
/// \p data untouched. Per-vertex frame failures are RECORDED, not returned
/// (see RigExecTransportRestData::restOk).
template<class Point, class Wide>
bool
RigExecBuildTransportRestData(
    const std::vector<Point> &rest,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    RigExecTransportRestData<Point, Wide> *data)
{
    if (!data) return false;
    RigExecTransportRestData<Point, Wide> built;
    built.pointCount = rest.size();
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(rest[i][axis])) return false;
        }
    }
    size_t offset = 0;
    built.restNormals.assign(rest.size(), Wide(0));
    for (int count : faceCounts) {
        if (count < 3 || static_cast<size_t>(count) > faceIndices.size() - offset) return false;
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= rest.size()) return false;
        }
        // Newell's area vector, evaluated relative to the first corner to
        // avoid subtracting products of large translated coordinates.
        const int origin = faceIndices[offset];
        Wide restArea(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int a = faceIndices[offset + corner];
            const int b = faceIndices[offset + corner + 1];
            restArea += deltaMushDetail::Cross(deltaMushDetail::Convert<Wide>(rest[a]) - deltaMushDetail::Convert<Wide>(rest[origin]),
                                deltaMushDetail::Convert<Wide>(rest[b]) - deltaMushDetail::Convert<Wide>(rest[origin]));
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            built.restNormals[index] += restArea;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceIndices.size()) return false;
    built.adjacency = deltaMushDetail::BuildAdjacency(rest.size(), faceCounts, faceIndices);
    built.neighbor.assign(rest.size(), -1);
    built.nr.assign(rest.size(), Wide(0));
    built.tr.assign(rest.size(), Wide(0));
    built.br.assign(rest.size(), Wide(0));
    built.restOk.assign(rest.size(), false);
    for (size_t i = 0; i < rest.size(); ++i) {
        Wide nr = built.restNormals[i];
        if (!deltaMushDetail::Normalize(&nr)) continue;
        int neighbor = -1;
        double longest = 0;
        Wide tr(0);
        // deltaMushDetail::BuildAdjacency orders neighbors by index, making equal-length
        // choices independent of face order and corner traversal direction.
        for (int candidate : built.adjacency[i]) {
            const Wide edge = deltaMushDetail::Convert<Wide>(rest[candidate]) - deltaMushDetail::Convert<Wide>(rest[i]);
            const Wide projected = edge - deltaMushDetail::Dot(edge, nr) * nr;
            const double length2 = projected.GetLengthSq();
            if (length2 > longest) {
                neighbor = candidate;
                longest = length2;
                tr = projected;
            }
        }
        if (neighbor < 0 || !deltaMushDetail::Normalize(&tr)) continue;
        built.neighbor[i] = neighbor;
        built.nr[i] = nr;
        built.tr[i] = tr;
        built.br[i] = deltaMushDetail::Cross(nr, tr);
        built.restOk[i] = true;
    }
    *data = std::move(built);
    return true;
}

/// Apply a transport to \p posed with prebuilt rest state: accumulate
/// posed normals, then rotate each nonzero offset through its rest/posed
/// frame pair. The same arithmetic the single-call kernel performs, with
/// the same failure conditions; invalid input fails atomically, leaving
/// \p out unchanged.
template<class Point, class Wide>
bool
RigExecApplyTransportWithRestData(
    const std::vector<Point> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<Point> &deltas,
    const RigExecTransportRestData<Point, Wide> &restData,
    std::vector<Point> *out)
{
    if (!out || !restData.Covers(posed.size()) ||
        posed.size() != deltas.size()) return false;
    for (size_t i = 0; i < posed.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(posed[i][axis]) ||
                !std::isfinite(deltas[i][axis])) return false;
        }
    }
    size_t offset = 0;
    std::vector<Wide> posedNormals(posed.size(), Wide(0));
    for (int count : faceCounts) {
        if (count < 3 || static_cast<size_t>(count) > faceIndices.size() - offset) return false;
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= posed.size()) return false;
        }
        const int origin = faceIndices[offset];
        Wide posedArea(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int a = faceIndices[offset + corner];
            const int b = faceIndices[offset + corner + 1];
            posedArea += deltaMushDetail::Cross(deltaMushDetail::Convert<Wide>(posed[a]) - deltaMushDetail::Convert<Wide>(posed[origin]),
                                 deltaMushDetail::Convert<Wide>(posed[b]) - deltaMushDetail::Convert<Wide>(posed[origin]));
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            posedNormals[index] += posedArea;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceIndices.size()) return false;
    std::vector<Point> result = deltas;
    for (size_t i = 0; i < posed.size(); ++i) {
        if (deltas[i] == Point(0)) continue;
        if (!restData.restOk[i]) return false;
        Wide np = posedNormals[i];
        if (!deltaMushDetail::Normalize(&np)) return false;
        const int neighbor = restData.neighbor[i];
        if (neighbor < 0 || static_cast<size_t>(neighbor) >= posed.size()) return false;
        const Wide edge = deltaMushDetail::Convert<Wide>(posed[neighbor]) - deltaMushDetail::Convert<Wide>(posed[i]);
        Wide tp = edge - deltaMushDetail::Dot(edge, np) * np;
        if (tp.GetLengthSq() <= edge.GetLengthSq() * 1e-24 || !deltaMushDetail::Normalize(&tp)) return false;
        const Wide bp = deltaMushDetail::Cross(np, tp);
        const Wide delta = deltaMushDetail::Convert<Wide>(deltas[i]);
        const Wide rotated = deltaMushDetail::Dot(delta, restData.tr[i]) * tp + deltaMushDetail::Dot(delta, restData.br[i]) * bp + deltaMushDetail::Dot(delta, restData.nr[i]) * np;
        result[i] = deltaMushDetail::Convert<Point>(rotated);
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(result[i][axis])) return false;
        }
    }
    *out = std::move(result);
    return true;
}

template<class Point, class Wide>
bool
RigExecTransportSurfaceOffsetsKernel(
    const std::vector<Point> &rest,
    const std::vector<Point> &posed,
    const std::vector<int> &faceCounts,
    const std::vector<int> &faceIndices,
    const std::vector<Point> &deltas,
    std::vector<Point> *out)
{
    RigExecTransportRestData<Point, Wide> restData;
    if (!RigExecBuildTransportRestData(rest, faceCounts, faceIndices, &restData)) {
        return false;
    }
    return RigExecApplyTransportWithRestData(
        posed, faceCounts, faceIndices, deltas, restData, out);
}

// Corner tangent transport: each face corner's frame has x along the
// bisector of its two edges and z along their cross product, weighted by the
// corner angle; offsets move from the rest frames to the posed frames. See
// docs/references.md (Delta Mush).
template<class Point, class Wide>
bool
RigExecTransportCornerOffsetsKernel(
    const std::vector<Point> &rest, const std::vector<Point> &posed,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const std::vector<Point> &deltas, std::vector<Point> *out)
{
    if (!out || rest.size() != posed.size() || rest.size() != deltas.size()) return false;
    struct Basis { Wide x{0}, y{0}, z{0}; double weight = 0; };
    const auto basis = [](const std::vector<Point> &points, int previous, int vertex, int next) {
        const auto normalized = [](Wide v) {
            const double length = v.GetLength();
            return length > 0 ? v / length : Wide(0);
        };
        const Wide before = normalized(deltaMushDetail::Convert<Wide>(points[previous]) - deltaMushDetail::Convert<Wide>(points[vertex]));
        const Wide after = normalized(deltaMushDetail::Convert<Wide>(points[vertex]) - deltaMushDetail::Convert<Wide>(points[next]));
        Basis result;
        bool equal = true;
        for (int a = 0; a < 3; ++a)
            equal = equal && std::abs(before[a] - after[a]) <= std::numeric_limits<float>::epsilon() * 10;
        if (equal) { result.x = Wide(1,0,0); result.y = Wide(0,1,0); result.z = Wide(0,0,1); return result; }
        result.z = normalized(deltaMushDetail::Cross(before, after));
        result.x = normalized(before + after);
        result.y = deltaMushDetail::Cross(result.z, result.x);
        result.weight = std::acos(std::max(-1.0, std::min(1.0, deltaMushDetail::Dot(before, after))));
        return result;
    };
    std::vector<Basis> restBasis, posedBasis;
    std::vector<double> sums(rest.size(), 0);
    size_t offset = 0;
    for (int count : counts) {
        if (count < 3 || size_t(count) > indices.size() - offset) return false;
        for (int c = 0; c < count; ++c) {
            const int previous = indices[offset + (c+count-1)%count];
            const int vertex = indices[offset+c], next = indices[offset+(c+1)%count];
            if (previous < 0 || vertex < 0 || next < 0 || size_t(previous) >= rest.size() ||
                size_t(vertex) >= rest.size() || size_t(next) >= rest.size()) return false;
            restBasis.push_back(basis(rest, previous, vertex, next));
            posedBasis.push_back(basis(posed, previous, vertex, next));
            sums[vertex] += posedBasis.back().weight;
        }
        offset += count;
    }
    if (offset != indices.size()) return false;
    std::vector<Wide> result(rest.size(), Wide(0));
    for (size_t corner = 0; corner < indices.size(); ++corner) {
        const int vertex = indices[corner];
        const auto &r = restBasis[corner], &p = posedBasis[corner];
        if (!(sums[vertex] > 0) || !(p.weight > 0)) continue;
        const Wide delta = deltaMushDetail::Convert<Wide>(deltas[vertex]);
        result[vertex] += (deltaMushDetail::Dot(delta,r.x)*p.x + deltaMushDetail::Dot(delta,r.y)*p.y +
            deltaMushDetail::Dot(delta,r.z)*p.z) * (p.weight / sums[vertex]);
    }
    std::vector<Point> converted;
    converted.reserve(result.size());
    for (const auto &p : result) {
        for (int a = 0; a < 3; ++a) if (!std::isfinite(p[a])) return false;
        converted.push_back(deltaMushDetail::Convert<Point>(p));
    }
    *out = std::move(converted); return true;
}

/// The rest half of a delta-mush deformation: the smoothing table, the
/// border pins, the smoothed rest surface, and the vertex transport rest
/// state over it. The inputs the derived state was built from are echoed so
/// a cache can compare candidates by value (see RigExecSkinTopology): the
/// derived state is a pure function of exactly the echo. The smoothing
/// settings are part of it: the posed half smooths with the same mode and
/// per-vertex influence the rest half used.
template<class Point, class Wide>
struct RigExecDeltaMushRestData {
    std::vector<Point> rest;
    std::vector<int> counts;
    std::vector<int> indices;
    int iterations = 0;
    double step = 0.0;
    bool pinBorders = false;
    double distanceWeight = 0.0;
    RigExecDeltaMushSettings settings;
    /// Per-vertex (neighbor, edge-distance weight) table.
    std::vector<std::vector<std::pair<int, double>>> neighbors;
    std::vector<bool> pinned;
    /// Empty when the settings only smooth.
    std::vector<Point> smoothRest;
    /// Built for the vertex frame transport only.
    RigExecTransportRestData<Point, Wide> transport;
    /// The rest-side validation passed; only the posed points and the
    /// displacement are still frame business.
    bool validated = false;
    /// The rest half of RigExecDeltaMushMaskPreservesVerdict, proved by the
    /// builder: every smoothing denominator the posed half divides by is
    /// positive and finite, and every rest and smoothed-rest coordinate is
    /// bounded, so their float difference cannot overflow. False unless the
    /// builder established it, which keeps any other producer on the full path.
    bool maskRestSafe = false;
    /// The largest |rest| + |smoothRest| coordinate, valid when maskRestSafe.
    double maskDetailBound = 0;

    bool operator==(const RigExecDeltaMushRestData &o) const {
        return validated == o.validated && iterations == o.iterations &&
               step == o.step && pinBorders == o.pinBorders &&
               distanceWeight == o.distanceWeight && settings == o.settings &&
               rest == o.rest && counts == o.counts && indices == o.indices;
    }
    bool operator!=(const RigExecDeltaMushRestData &o) const {
        return !(*this == o);
    }
};

namespace deltaMushDetail {
// Laplacian relaxation, shared by the rest build and the posed apply:
// smoothing the same input twice must smooth it the same way, which is only
// guaranteed in one definition. A unit influence multiplies the step
// exactly, so the legacy rest-weighted relaxation keeps its bits.
template<class Point, class Wide>
std::vector<Point>
SmoothWithNeighbors(
    const std::vector<Point> &input,
    const std::vector<std::vector<std::pair<int, double>>> &neighbors,
    const std::vector<bool> &pinned,
    int iterations, double step,
    const RigExecDeltaMushSettings &settings)
{
    std::vector<Wide> current;
    current.reserve(input.size());
    for (const auto &p : input) current.push_back(Convert<Wide>(p));
    auto next = current;
    const bool weighted = !settings.smoothWeights.empty();
    for (int iteration = 0; iteration < iterations; ++iteration) {
        for (size_t i = 0; i < current.size(); ++i) {
            if (pinned[i] || neighbors[i].empty()) continue;
            const double influence = weighted ? double(settings.smoothWeights[i]) : 1.0;
            Wide sum(0); double total = 0;
            if (settings.smoothing == RigExecDeltaMushSmoothing::LengthWeighted) {
                for (const auto &n : neighbors[i]) {
                    const Wide edge = current[n.first] - current[i];
                    const double length = edge.GetLength();
                    sum += edge * length; total += length;
                }
                const double divisor = total * neighbors[i].size();
                next[i] = current[i];
                if (divisor > std::numeric_limits<float>::epsilon() * 10)
                    next[i] += sum * (2 * step * influence / divisor);
            } else {
                for (const auto &n : neighbors[i]) {
                    const double w = settings.smoothing == RigExecDeltaMushSmoothing::Simple ? 1 : n.second;
                    sum += current[n.first] * w; total += w;
                }
                next[i] = current[i] + step * influence * (sum / total - current[i]);
            }
        }
        current.swap(next);
    }
    std::vector<Point> result;
    result.reserve(current.size());
    for (const auto &p : current) result.push_back(Convert<Point>(p));
    return result;
}

// The magnitude both halves of the mask verdict proof stay under: far below
// float max (3.4e38), so no bounded intermediate overflows.
constexpr double kMaskSafeMagnitude = 1e30;

// Establishes RigExecDeltaMushRestData::maskRestSafe on a finished build. A
// rest-weighted vertex divides by its weight sum, which underflows to zero for
// tiny inverse-distance weights (0/0 in the full kernel, anywhere on the mesh);
// simple sums count neighbors and length-weighted guards its divisor.
template<class Point, class Wide>
void
ProveMaskRestSafe(RigExecDeltaMushRestData<Point, Wide> *data)
{
    data->maskRestSafe = false;
    data->maskDetailBound = 0;
    if (data->settings.smoothing == RigExecDeltaMushSmoothing::Rest) {
        for (size_t i = 0; i < data->neighbors.size(); ++i) {
            if (data->pinned[i] || data->neighbors[i].empty()) continue;
            double total = 0;
            for (const auto &n : data->neighbors[i]) total += n.second;
            if (!(total > 0) || !std::isfinite(total)) return;
        }
    }
    double bound = 0;
    for (size_t i = 0; i < data->rest.size(); ++i) {
        for (int a = 0; a < 3; ++a) {
            const double r = std::abs(double(data->rest[i][a]));
            const double s = data->smoothRest.empty()
                ? 0.0 : std::abs(double(data->smoothRest[i][a]));
            if (!(r <= kMaskSafeMagnitude) || !(s <= kMaskSafeMagnitude)) return;
            bound = std::max(bound, r + s);
        }
    }
    data->maskDetailBound = bound;
    data->maskRestSafe = true;
}

// The settings' own validation, independent of any mesh.
inline bool
SettingsValid(const RigExecDeltaMushSettings &settings, size_t pointCount)
{
    if ((settings.smoothing != RigExecDeltaMushSmoothing::Rest &&
         settings.smoothing != RigExecDeltaMushSmoothing::Simple &&
         settings.smoothing != RigExecDeltaMushSmoothing::LengthWeighted) ||
        (settings.frameTransport != RigExecDeltaMushFrameTransport::Vertex &&
         settings.frameTransport != RigExecDeltaMushFrameTransport::Corner) ||
        (!settings.smoothWeights.empty() && settings.smoothWeights.size() != pointCount) ||
        settings.edges.size() % 2) {
        return false;
    }
    for (float w : settings.smoothWeights) {
        if (!std::isfinite(w) || w < 0 || w > 1) return false;
    }
    return true;
}
} // namespace deltaMushDetail

/// Build the rest half of a delta-mush deformation: validate the
/// rest-side inputs, derive the smoothing table (from the polygon edges,
/// or from the explicit edge pairs, which must include every polygon edge),
/// smooth the rest surface, and build the vertex transport rest state over
/// it. The input echo is filled whatever the outcome, so a cache can keep a
/// rejected build as its candidate; the derived state is meaningful only on
/// true. A step outside [0, 1] is refused only for rest-weighted smoothing.
template<class Point, class Wide>
bool
RigExecBuildDeltaMushRestData(
    const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight, const RigExecDeltaMushSettings &settings,
    RigExecDeltaMushRestData<Point, Wide> *data)
{
    if (!data) return false;
    RigExecDeltaMushRestData<Point, Wide> built;
    built.rest = rest;
    built.counts = counts;
    built.indices = indices;
    built.iterations = iterations;
    built.step = step;
    built.pinBorders = pinBorders;
    built.distanceWeight = distanceWeight;
    built.settings = settings;
    const bool restWeighted =
        settings.smoothing == RigExecDeltaMushSmoothing::Rest;
    if (iterations < 0 || iterations > 1000 || !std::isfinite(step) ||
        (restWeighted && (step < 0 || step > 1)) ||
        !std::isfinite(distanceWeight) || distanceWeight < 0 ||
        distanceWeight > 10 ||
        !deltaMushDetail::SettingsValid(settings, rest.size())) {
        *data = std::move(built);
        return false;
    }
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(rest[i][a])) {
                *data = std::move(built);
                return false;
            }
        }
    }
    // Sorted polygon edges with run counts instead of a map: the same
    // lexicographic order the neighbor pushes below run in, without a node
    // per edge. A run of one is a border edge. Connectivity only -- the
    // distance weights stay rest-derived.
    std::vector<std::pair<int,int>> edges;
    size_t offset = 0;
    for (int count : counts) {
        if (count < 3 || size_t(count) > indices.size() - offset) {
            *data = std::move(built);
            return false;
        }
        for (int c = 0; c < count; ++c) {
            int a = indices[offset+c], b = indices[offset+(c+1)%count];
            if (a < 0 || b < 0 || size_t(a) >= rest.size() || size_t(b) >= rest.size() || a == b) {
                *data = std::move(built);
                return false;
            }
            edges.emplace_back(std::minmax(a,b));
        }
        offset += count;
    }
    std::sort(edges.begin(), edges.end());
    if (offset != indices.size() || (counts.empty() && !rest.empty())) {
        *data = std::move(built);
        return false;
    }
    // Explicit edge pairs replace the polygon edges as the smoothing
    // connectivity; polygon incidence still decides the border pins.
    std::vector<std::pair<int,int>> explicitEdges;
    if (!settings.edges.empty()) {
        explicitEdges.reserve(settings.edges.size() / 2);
        for (size_t i = 0; i < settings.edges.size(); i += 2) {
            const int a = settings.edges[i], b = settings.edges[i+1];
            if (a < 0 || b < 0 || size_t(a) >= rest.size() || size_t(b) >= rest.size() || a == b) {
                *data = std::move(built);
                return false;
            }
            explicitEdges.emplace_back(std::minmax(a,b));
        }
        std::sort(explicitEdges.begin(), explicitEdges.end());
        if (std::adjacent_find(explicitEdges.begin(), explicitEdges.end()) != explicitEdges.end()) {
            *data = std::move(built);
            return false;
        }
        for (const auto &edge : edges) {
            if (!std::binary_search(explicitEdges.begin(), explicitEdges.end(), edge)) {
                *data = std::move(built);
                return false;
            }
        }
    }
    if (!iterations || step == 0 || rest.empty()) {
        built.validated = true;
        deltaMushDetail::ProveMaskRestSafe(&built);
        *data = std::move(built);
        return true;
    }
    built.neighbors.assign(rest.size(), {});
    built.pinned.assign(rest.size(), false);
    const auto connect = [&](int a, int b, size_t polygonUses) {
        if (polygonUses == 1 && pinBorders) built.pinned[a] = built.pinned[b] = true;
        double length = (deltaMushDetail::Convert<Wide>(rest[a])-deltaMushDetail::Convert<Wide>(rest[b])).GetLength();
        double w = distanceWeight == 0 ? 1 : std::pow(std::max(length, 1e-12), -distanceWeight);
        if (!std::isfinite(w)) return false;
        built.neighbors[a].emplace_back(b,w);built.neighbors[b].emplace_back(a,w);
        return true;
    };
    if (explicitEdges.empty()) {
        for (size_t i = 0; i < edges.size();) {
            size_t j = i + 1;
            while (j < edges.size() && edges[j] == edges[i]) ++j;
            if (!connect(edges[i].first, edges[i].second, j - i)) {
                *data = std::move(built);
                return false;
            }
            i = j;
        }
    } else {
        for (const auto &edge : explicitEdges) {
            const auto uses = std::equal_range(edges.begin(), edges.end(), edge);
            if (!connect(edge.first, edge.second, size_t(uses.second - uses.first))) {
                *data = std::move(built);
                return false;
            }
        }
    }
    if (settings.onlySmooth) {
        built.validated = true;
        deltaMushDetail::ProveMaskRestSafe(&built);
        *data = std::move(built);
        return true;
    }
    built.smoothRest = deltaMushDetail::SmoothWithNeighbors<Point, Wide>(
        rest, built.neighbors, built.pinned, iterations, step, settings);
    if (settings.frameTransport == RigExecDeltaMushFrameTransport::Vertex &&
        !RigExecBuildTransportRestData(
            built.smoothRest, counts, indices, &built.transport)) {
        *data = std::move(built);
        return false;
    }
    built.validated = true;
    deltaMushDetail::ProveMaskRestSafe(&built);
    *data = std::move(built);
    return true;
}

/// The legacy-settings build.
template<class Point, class Wide>
bool
RigExecBuildDeltaMushRestData(
    const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight,
    RigExecDeltaMushRestData<Point, Wide> *data)
{
    return RigExecBuildDeltaMushRestData<Point, Wide>(
        rest, counts, indices, iterations, step, pinBorders, distanceWeight,
        RigExecDeltaMushSettings(), data);
}

/// Apply a delta-mush deformation with prebuilt rest state: smooth the
/// posed points, transport the rest detail onto them, and add it. The
/// same arithmetic the single-call kernel performs, with the same failure
/// conditions; invalid input fails atomically, leaving \p points unchanged.
/// The displacement is limited to [0, 1] only for rest-weighted smoothing.
template<class Point, class Wide>
bool
RigExecApplyDeltaMushWithRestData(
    std::vector<Point> *points,
    const RigExecDeltaMushRestData<Point, Wide> &restData,
    double displacement)
{
    if (!points || !restData.validated) return false;
    if (points->size() != restData.rest.size()) return false;
    const RigExecDeltaMushSettings &settings = restData.settings;
    if (!std::isfinite(displacement) ||
        (settings.smoothing == RigExecDeltaMushSmoothing::Rest &&
         (displacement < 0 || displacement > 1))) return false;
    for (size_t i = 0; i < points->size(); ++i)
        for (int a = 0; a < 3; ++a)
            if (!std::isfinite((*points)[i][a])) return false;
    if (!restData.iterations || restData.step == 0 || restData.rest.empty()) return true;
    if (restData.neighbors.size() != restData.rest.size() ||
        restData.pinned.size() != restData.rest.size()) return false;
    std::vector<Point> smoothPosed = deltaMushDetail::SmoothWithNeighbors<Point, Wide>(
        *points, restData.neighbors, restData.pinned,
        restData.iterations, restData.step, settings);
    if (settings.onlySmooth) {
        for (const auto &p : smoothPosed)
            for (int a = 0; a < 3; ++a)
                if (!std::isfinite(p[a])) return false;
        *points = std::move(smoothPosed);
        return true;
    }
    if (restData.smoothRest.size() != restData.rest.size()) return false;
    std::vector<Point> detail(restData.rest.size()), rotated;
    for (size_t i=0; i<restData.rest.size(); ++i) detail[i]=(restData.rest[i]-restData.smoothRest[i])*float(displacement);
    if (settings.frameTransport == RigExecDeltaMushFrameTransport::Corner) {
        if (!RigExecTransportCornerOffsetsKernel<Point, Wide>(
                restData.smoothRest, smoothPosed, restData.counts,
                restData.indices, detail, &rotated)) return false;
    } else {
        if (!restData.transport.Covers(restData.rest.size())) return false;
        if (!RigExecApplyTransportWithRestData(smoothPosed, restData.counts, restData.indices, detail, restData.transport, &rotated)) return false;
    }
    for (size_t i=0; i<restData.rest.size(); ++i) {
        smoothPosed[i] += rotated[i];
        for (int a=0;a<3;++a) if (!std::isfinite(smoothPosed[i][a])) return false;
    }
    *points = std::move(smoothPosed);
    return true;
}

/// What RigExecApplyDeltaMushWithRestDataMasked can keep between calls for one
/// mover: the region plan of an active mask, and the result of the last call
/// that succeeded on it. Owned by the caller next to the rest data it was
/// built for (RigExecSurfaceKernelCache resets it with that rest data); a
/// call checks the plan's `rest` and `active` before trusting any of it.
///
/// Copy on write, like the rest of that cache: a call publishes a new Plan or
/// Result by replacing a pointer and never edits one it already published.
/// A program clone's memberwise copy therefore shares both with the original
/// for free, and either side's next call replaces only its own pointers.
template<class Point>
struct RigExecDeltaMushMaskMemo {
    /// The region plan: valid for `rest` and exactly this `active` mask,
    /// published once a call with them passed the topology checks.
    struct Plan {
        const void *rest = nullptr;
        std::vector<char> active;
        /// F dilated by one smoothing ring per iteration, ascending.
        std::vector<int> members;
        /// The active vertices' corners (previous, vertex, next) in face
        /// order; built for the corner frame transport only.
        std::vector<std::array<int, 3>> corners;
        /// The active vertices, ascending.
        std::vector<int> activeList;
        /// Every point an active result reads: the members and their
        /// smoothing neighbors, ascending.
        std::vector<int> reads;
    };
    /// The last successful call on `plan`: its displacement, the calling
    /// thread's floating-point control word (the same inputs round and flush
    /// differently under another one), the entering points at the plan's
    /// `reads`, and its results at the plan's `activeList`.
    struct Result {
        double displacement = 0;
        uint64_t floatControl = kRigExecUnknownFloatControl;
        std::vector<Point> readPoints;
        std::vector<Point> activeResult;
    };
    std::shared_ptr<const Plan> plan;
    /// Describes `plan`; dropped whenever `plan` is replaced.
    std::shared_ptr<const Result> result;
};

/// RigExecApplyDeltaMushWithRestData restricted to the vertices flagged in
/// \p active: bit for bit the same result at every active vertex, while every
/// other point is left exactly as it entered. Callers pass the support of the
/// mover envelope, which returns the entering point wherever the weight is
/// zero, so the skipped vertices could never reach the output anyway.
///
/// Why the restriction is exact. An active vertex's result reads its incident
/// faces (corner frames, posed normals, transport neighbor) and the smoothed
/// points of those faces' vertices. Smoothing is a Jacobi relaxation: after k
/// iterations a vertex depends only on vertices within k rings. So smoothing
/// runs on F = every vertex of a face touching an active vertex, dilated by
/// `iterations` rings of the smoothing graph; points outside that region stay
/// frozen, and the error a frozen point introduces travels one ring per
/// iteration, never reaching F. Per-vertex arithmetic is unchanged, and the
/// face and corner accumulations visit the same faces in the same order, so
/// active results match the full kernel exactly. The bitwise equivalence is
/// pinned by testRigExecMath.
///
/// \p ranges may split the per-vertex and per-corner loops across threads;
/// every sum into a vertex still runs in face and corner order, so the result
/// is bitwise the same. \p memo, when given, keeps the region plan of the last
/// mask and the last result: the active results depend only on the rest data,
/// the mask, the displacement, the calling thread's floating-point control
/// state and the entering points the members and their neighbors read, so a
/// call that matches all of them bit for bit returns the remembered results.
/// An eyelid finish reruns for nothing on a mouth edit.
///
/// Failure differs only in reach: non-finite or degenerate data is detected
/// on the active region, where the full kernel would detect it anywhere.
template<class Point, class Wide>
bool
RigExecApplyDeltaMushWithRestDataMasked(
    std::vector<Point> *points,
    const RigExecDeltaMushRestData<Point, Wide> &restData,
    double displacement, const std::vector<char> &active,
    const RigExecRangeRunner *ranges = nullptr,
    RigExecDeltaMushMaskMemo<Point> *memo = nullptr)
{
    if (!points || !restData.validated) return false;
    const size_t n = restData.rest.size();
    if (points->size() != n || active.size() != n) return false;
    const RigExecDeltaMushSettings &settings = restData.settings;
    if (!std::isfinite(displacement) ||
        (settings.smoothing == RigExecDeltaMushSmoothing::Rest &&
         (displacement < 0 || displacement > 1))) return false;
    {
        std::atomic<bool> finite(true);
        RigExecRunRanges(ranges, n, 4096, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i)
                for (int a = 0; a < 3; ++a)
                    if (!std::isfinite((*points)[i][a])) { finite = false; return; }
        });
        if (!finite) return false;
    }
    if (!restData.iterations || restData.step == 0 || restData.rest.empty()) return true;
    if (restData.neighbors.size() != n || restData.pinned.size() != n) return false;
    const std::vector<int> &counts = restData.counts;
    const std::vector<int> &indices = restData.indices;
    const bool cornerFrames = !settings.onlySmooth &&
        settings.frameTransport == RigExecDeltaMushFrameTransport::Corner;

    using Memo = RigExecDeltaMushMaskMemo<Point>;
    std::shared_ptr<const typename Memo::Plan> planHandle;
    if (memo && memo->plan && memo->plan->rest == &restData &&
        memo->plan->active == active) {
        planHandle = memo->plan;
    } else {
        if (memo) {
            memo->plan.reset();
            memo->result.reset();
        }
        auto built = std::make_shared<typename Memo::Plan>();
        typename Memo::Plan &plan = *built;
        // F: every vertex of a face that touches an active vertex. The rest
        // data's topology is fixed, so a plan that passed these checks once
        // passes them again.
        std::vector<char> region(n, 0);
        std::vector<int> frontier;
        {
            size_t offset = 0;
            for (int count : counts) {
                if (count < 3 || size_t(count) > indices.size() - offset) return false;
                bool touches = false;
                for (int c = 0; c < count; ++c) {
                    const int v = indices[offset + c];
                    if (v < 0 || size_t(v) >= n) return false;
                    touches = touches || active[size_t(v)];
                }
                if (touches) {
                    for (int c = 0; c < count; ++c) {
                        const int v = indices[offset + c];
                        if (!region[size_t(v)]) { region[size_t(v)] = 1; frontier.push_back(v); }
                    }
                }
                offset += size_t(count);
            }
            if (offset != indices.size()) return false;
        }
        for (size_t i = 0; i < n; ++i)
            if (active[i] && !region[i]) { region[i] = 1; frontier.push_back(int(i)); }
        // Dilate F by one smoothing ring per iteration.
        plan.members = frontier;
        for (int ring = 0; ring < restData.iterations && !frontier.empty(); ++ring) {
            std::vector<int> next;
            for (int v : frontier)
                for (const auto &nb : restData.neighbors[size_t(v)])
                    if (!region[size_t(nb.first)]) {
                        region[size_t(nb.first)] = 1;
                        next.push_back(nb.first);
                        plan.members.push_back(nb.first);
                    }
            frontier.swap(next);
        }
        std::sort(plan.members.begin(), plan.members.end());
        if (cornerFrames) {
            size_t offset = 0;
            for (int count : counts) {
                for (int c = 0; c < count; ++c) {
                    const int vertex = indices[offset + c];
                    if (!active[size_t(vertex)]) continue;
                    plan.corners.push_back({indices[offset + (c + count - 1) % count], vertex,
                                            indices[offset + (c + 1) % count]});
                }
                offset += size_t(count);
            }
        }
        for (size_t i = 0; i < n; ++i)
            if (active[i]) plan.activeList.push_back(int(i));
        plan.rest = &restData;
        if (memo) {
            // What the members' smoothing reads: themselves and their
            // neighbors.
            std::vector<char> read(region);
            for (int v : plan.members)
                for (const auto &nb : restData.neighbors[size_t(v)]) read[size_t(nb.first)] = 1;
            for (size_t i = 0; i < n; ++i)
                if (read[i]) plan.reads.push_back(int(i));
            plan.active = active;
            memo->plan = built;
        }
        planHandle = std::move(built);
    }
    const typename Memo::Plan &plan = *planHandle;
    const std::vector<int> &members = plan.members;
    const uint64_t floatControl = RigExecFloatControlWord();
    if (memo && memo->result &&
        floatControl != kRigExecUnknownFloatControl &&
        memo->result->floatControl == floatControl &&
        !std::memcmp(&memo->result->displacement, &displacement, sizeof(double))) {
        const typename Memo::Result &last = *memo->result;
        bool same = last.readPoints.size() == plan.reads.size();
        for (size_t k = 0; same && k < plan.reads.size(); ++k)
            same = !std::memcmp(&(*points)[size_t(plan.reads[k])], &last.readPoints[k], sizeof(Point));
        if (same) {
            for (size_t k = 0; k < plan.activeList.size(); ++k)
                (*points)[size_t(plan.activeList[k])] = last.activeResult[k];
            return true;
        }
    }
    if (memo) memo->result.reset();
    // Every successful exit below publishes its inputs and results first, as
    // a new Result.
    const auto remember = [&](const std::vector<Point> &result) {
        if (!memo) return;
        auto kept = std::make_shared<typename Memo::Result>();
        kept->readPoints.resize(plan.reads.size());
        for (size_t k = 0; k < plan.reads.size(); ++k)
            kept->readPoints[k] = (*points)[size_t(plan.reads[k])];
        kept->activeResult.resize(plan.activeList.size());
        for (size_t k = 0; k < plan.activeList.size(); ++k)
            kept->activeResult[k] = result[size_t(plan.activeList[k])];
        kept->displacement = displacement;
        kept->floatControl = floatControl;
        memo->result = std::move(kept);
    };

    // Jacobi relaxation over the region only: the same per-vertex arithmetic
    // as deltaMushDetail::SmoothWithNeighbors, frozen elsewhere. A member's
    // next value reads only the previous iteration, so each iteration may be
    // split across the members.
    std::vector<Wide> current(n), nextValues;
    for (size_t i = 0; i < n; ++i) current[i] = deltaMushDetail::Convert<Wide>((*points)[i]);
    nextValues = current;
    const bool weighted = !settings.smoothWeights.empty();
    for (int iteration = 0; iteration < restData.iterations; ++iteration) {
        RigExecRunRanges(ranges, members.size(), 256, [&](size_t begin, size_t end) {
        for (size_t m = begin; m < end; ++m) {
            const size_t i = size_t(members[m]);
            if (restData.pinned[i] || restData.neighbors[i].empty()) continue;
            const double influence = weighted ? double(settings.smoothWeights[i]) : 1.0;
            Wide sum(0); double total = 0;
            if (settings.smoothing == RigExecDeltaMushSmoothing::LengthWeighted) {
                for (const auto &nb : restData.neighbors[i]) {
                    const Wide edge = current[nb.first] - current[i];
                    const double length = edge.GetLength();
                    sum += edge * length; total += length;
                }
                const double divisor = total * restData.neighbors[i].size();
                nextValues[i] = current[i];
                if (divisor > std::numeric_limits<float>::epsilon() * 10)
                    nextValues[i] += sum * (2 * restData.step * influence / divisor);
            } else {
                for (const auto &nb : restData.neighbors[i]) {
                    const double w = settings.smoothing == RigExecDeltaMushSmoothing::Simple ? 1 : nb.second;
                    sum += current[nb.first] * w; total += w;
                }
                nextValues[i] = current[i] + restData.step * influence * (sum / total - current[i]);
            }
        }
        });
        current.swap(nextValues);
    }
    std::vector<Point> smoothPosed(*points);
    for (int vi : members) smoothPosed[size_t(vi)] = deltaMushDetail::Convert<Point>(current[size_t(vi)]);

    if (settings.onlySmooth) {
        for (size_t i = 0; i < n; ++i) {
            if (!active[i]) continue;
            for (int a = 0; a < 3; ++a) if (!std::isfinite(smoothPosed[i][a])) return false;
        }
        remember(smoothPosed);
        for (size_t i = 0; i < n; ++i) if (active[i]) (*points)[i] = smoothPosed[i];
        return true;
    }
    if (restData.smoothRest.size() != n) return false;
    const auto detailAt = [&](size_t i) {
        return (restData.rest[i] - restData.smoothRest[i]) * float(displacement);
    };
    std::vector<Point> rotated(n, Point(0));
    if (settings.frameTransport == RigExecDeltaMushFrameTransport::Corner) {
        // RigExecTransportCornerOffsetsKernel over the active vertices'
        // corners, in its corner order.
        struct Basis { Wide x{0}, y{0}, z{0}; double weight = 0; };
        const auto basis = [](const std::vector<Point> &pts, int previous, int vertex, int nextIndex) {
            const auto normalized = [](Wide v) {
                const double length = v.GetLength();
                return length > 0 ? v / length : Wide(0);
            };
            const Wide before = normalized(deltaMushDetail::Convert<Wide>(pts[previous]) - deltaMushDetail::Convert<Wide>(pts[vertex]));
            const Wide after = normalized(deltaMushDetail::Convert<Wide>(pts[vertex]) - deltaMushDetail::Convert<Wide>(pts[nextIndex]));
            Basis result;
            bool equal = true;
            for (int a = 0; a < 3; ++a)
                equal = equal && std::abs(before[a] - after[a]) <= std::numeric_limits<float>::epsilon() * 10;
            if (equal) { result.x = Wide(1,0,0); result.y = Wide(0,1,0); result.z = Wide(0,0,1); return result; }
            result.z = normalized(deltaMushDetail::Cross(before, after));
            result.x = normalized(before + after);
            result.y = deltaMushDetail::Cross(result.z, result.x);
            result.weight = std::acos(std::max(-1.0, std::min(1.0, deltaMushDetail::Dot(before, after))));
            return result;
        };
        // Each corner's two bases depend on that corner alone, so they are
        // built in parallel; every sum into a vertex then runs in the plan's
        // corner order (face order), which keeps its bits.
        const std::vector<std::array<int, 3>> &corners = plan.corners;
        std::vector<Basis> restBasis(corners.size()), posedBasis(corners.size());
        RigExecRunRanges(ranges, corners.size(), 256, [&](size_t begin, size_t end) {
            for (size_t corner = begin; corner < end; ++corner) {
                const auto &k = corners[corner];
                restBasis[corner] = basis(restData.smoothRest, k[0], k[1], k[2]);
                posedBasis[corner] = basis(smoothPosed, k[0], k[1], k[2]);
            }
        });
        std::vector<double> sums(n, 0);
        for (size_t corner = 0; corner < corners.size(); ++corner)
            sums[size_t(corners[corner][1])] += posedBasis[corner].weight;
        // Each corner's rotated detail, weighted by its share of the vertex.
        std::vector<Wide> terms(corners.size(), Wide(0));
        std::vector<char> contributes(corners.size(), 0);
        RigExecRunRanges(ranges, corners.size(), 256, [&](size_t begin, size_t end) {
            for (size_t corner = begin; corner < end; ++corner) {
                const size_t vertex = size_t(corners[corner][1]);
                const Basis &r = restBasis[corner], &p = posedBasis[corner];
                if (!(sums[vertex] > 0) || !(p.weight > 0)) continue;
                const Wide delta = deltaMushDetail::Convert<Wide>(detailAt(vertex));
                terms[corner] = (deltaMushDetail::Dot(delta,r.x)*p.x + deltaMushDetail::Dot(delta,r.y)*p.y +
                    deltaMushDetail::Dot(delta,r.z)*p.z) * (p.weight / sums[vertex]);
                contributes[corner] = 1;
            }
        });
        std::vector<Wide> result(n, Wide(0));
        for (size_t corner = 0; corner < corners.size(); ++corner)
            if (contributes[corner]) result[size_t(corners[corner][1])] += terms[corner];
        for (size_t i = 0; i < n; ++i) {
            if (!active[i]) continue;
            for (int a = 0; a < 3; ++a) if (!std::isfinite(result[i][a])) return false;
            rotated[i] = deltaMushDetail::Convert<Point>(result[i]);
        }
    } else {
        // RigExecApplyTransportWithRestData over the active vertices: posed
        // normals from the faces that touch them, in face order.
        const RigExecTransportRestData<Point, Wide> &transport = restData.transport;
        if (!transport.Covers(n)) return false;
        std::vector<Wide> posedNormals(n, Wide(0));
        size_t offset = 0;
        for (int count : counts) {
            bool touches = false;
            for (int c = 0; c < count; ++c) touches = touches || active[size_t(indices[offset + c])];
            if (touches) {
                const int origin = indices[offset];
                Wide posedArea(0);
                for (int c = 1; c + 1 < count; ++c) {
                    const int a = indices[offset + c], b = indices[offset + c + 1];
                    posedArea += deltaMushDetail::Cross(deltaMushDetail::Convert<Wide>(smoothPosed[a]) - deltaMushDetail::Convert<Wide>(smoothPosed[origin]),
                                         deltaMushDetail::Convert<Wide>(smoothPosed[b]) - deltaMushDetail::Convert<Wide>(smoothPosed[origin]));
                }
                for (int c = 0; c < count; ++c) posedNormals[size_t(indices[offset + c])] += posedArea;
            }
            offset += size_t(count);
        }
        for (size_t i = 0; i < n; ++i) {
            if (!active[i]) continue;
            const Point delta = detailAt(i);
            for (int a = 0; a < 3; ++a)
                if (!std::isfinite(smoothPosed[i][a]) || !std::isfinite(delta[a])) return false;
            rotated[i] = delta;
            if (delta == Point(0)) continue;
            if (!transport.restOk[i]) return false;
            Wide np = posedNormals[i];
            if (!deltaMushDetail::Normalize(&np)) return false;
            const int neighbor = transport.neighbor[i];
            if (neighbor < 0 || size_t(neighbor) >= n) return false;
            const Wide edge = deltaMushDetail::Convert<Wide>(smoothPosed[neighbor]) - deltaMushDetail::Convert<Wide>(smoothPosed[i]);
            Wide tp = edge - deltaMushDetail::Dot(edge, np) * np;
            if (tp.GetLengthSq() <= edge.GetLengthSq() * 1e-24 || !deltaMushDetail::Normalize(&tp)) return false;
            const Wide bp = deltaMushDetail::Cross(np, tp);
            const Wide d = deltaMushDetail::Convert<Wide>(delta);
            rotated[i] = deltaMushDetail::Convert<Point>(deltaMushDetail::Dot(d, transport.tr[i]) * tp +
                deltaMushDetail::Dot(d, transport.br[i]) * bp + deltaMushDetail::Dot(d, transport.nr[i]) * np);
            for (int a = 0; a < 3; ++a) if (!std::isfinite(rotated[i][a])) return false;
        }
    }
    // Validate every active result before touching the caller's points, so a
    // failure is atomic, as it is in the full kernel.
    for (size_t i = 0; i < n; ++i) {
        if (!active[i]) continue;
        smoothPosed[i] += rotated[i];
        for (int a = 0; a < 3; ++a) if (!std::isfinite(smoothPosed[i][a])) return false;
    }
    remember(smoothPosed);
    for (size_t i = 0; i < n; ++i) if (active[i]) (*points)[i] = smoothPosed[i];
    return true;
}

/// Whether RigExecApplyDeltaMushWithRestDataMasked provably reaches the same
/// verdict as the full kernel on these inputs, so a host may substitute it.
/// The masked kernel validates only the active region; that is equivalent
/// exactly when nothing outside it can fail. Topology and entering points are
/// validated whole by both. The builder proves the rest side (maskRestSafe:
/// no zero smoothing denominator, bounded rest and smoothed rest). Given
/// that, smoothing-only and corner transport fail only through float
/// overflow, which the magnitude bound below rules out; vertex transport also
/// fails on a degenerate posed frame anywhere on the mesh, which only the
/// full kernel can see, so it is never substituted.
///
/// The bound: one Jacobi step moves a coordinate by at most 4|step| times the
/// largest coordinate (LengthWeighted's worst case; Rest and Simple move at
/// most 2|step|), and the transported detail is at most 6 times the largest
/// rest-minus-smoothed-rest coordinate times |displacement|.
template<class Point, class Wide>
bool
RigExecDeltaMushMaskPreservesVerdict(
    const std::vector<Point> &points,
    const RigExecDeltaMushRestData<Point, Wide> &restData,
    double displacement, const RigExecRangeRunner *ranges = nullptr)
{
    constexpr double kSafe = deltaMushDetail::kMaskSafeMagnitude;
    const RigExecDeltaMushSettings &settings = restData.settings;
    if (!restData.validated || !restData.maskRestSafe ||
        (!settings.onlySmooth &&
         settings.frameTransport != RigExecDeltaMushFrameTransport::Corner) ||
        !(std::abs(displacement) <= kSafe)) {   // also rejects NaN
        return false;
    }
    // The largest coordinate magnitude. A maximum does not depend on the
    // order it is taken in, so the scan is split into fixed blocks, each
    // writing only its own slot, and the slots fold here after the join.
    constexpr size_t kBlock = 32;
    const size_t blocks = (points.size() + kBlock - 1) / kBlock;
    std::vector<double> blockLargest(blocks, 0.0);
    std::vector<char> blockBounded(blocks, 1);
    RigExecRunRanges(ranges, blocks, 64, [&](size_t begin, size_t end) {
        for (size_t b = begin; b < end; ++b) {
            double slice = 0;
            const size_t last = std::min(points.size(), (b + 1) * kBlock);
            for (size_t i = b * kBlock; i < last && blockBounded[b]; ++i)
                for (int a = 0; a < 3; ++a) {
                    const double v = std::abs(double(points[i][a]));
                    if (!(v <= kSafe)) { blockBounded[b] = 0; break; }   // also rejects NaN and inf
                    slice = std::max(slice, v);
                }
            blockLargest[b] = slice;
        }
    });
    double largest = 0;
    for (size_t b = 0; b < blocks; ++b) {
        if (!blockBounded[b]) return false;
        largest = std::max(largest, blockLargest[b]);
    }
    const double growth = 1 + 4 * std::abs(restData.step);
    for (int k = 0; k < restData.iterations; ++k) {
        largest *= growth;
        if (!(largest <= kSafe)) return false;
    }
    if (settings.onlySmooth) return true;
    if (restData.smoothRest.size() != restData.rest.size()) return false;
    return largest + 6 * restData.maskDetailBound * std::abs(displacement) <= kSafe;
}

template<class Point, class Wide>
bool
RigExecApplyDeltaMushKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders,
    double distanceWeight, double displacement,
    const RigExecDeltaMushSettings &settings = {})
{
    if (!points || points->size() != rest.size()) return false;
    RigExecDeltaMushRestData<Point, Wide> restData;
    if (!RigExecBuildDeltaMushRestData(rest, counts, indices, iterations,
                                       step, pinBorders, distanceWeight,
                                       settings, &restData)) {
        return false;
    }
    return RigExecApplyDeltaMushWithRestData(points, restData, displacement);
}

namespace deltaMushDetail {
// The computation-to-target map's own validation: finite, affine up to
// computed roundoff, and invertible.
template<class Matrix>
bool
ComputationMapValid(const Matrix &computationToTarget)
{
    for (int row = 0; row < 4; ++row) for (int col = 0; col < 4; ++col)
        if (!std::isfinite(computationToTarget[row][col])) return false;
    return !(std::abs(computationToTarget[0][3]) > 1e-12 || std::abs(computationToTarget[1][3]) > 1e-12 ||
             std::abs(computationToTarget[2][3]) > 1e-12 || std::abs(computationToTarget[3][3]-1) > 1e-12 ||
             std::abs(computationToTarget.GetDeterminant()) < 1e-14);
}

// Runs \p apply on \p points carried into computation space by the inverse
// of the canonicalized row-vector affine map, then carries the result back.
template<class Point, class Wide, class Matrix, class Apply>
bool
ApplyInComputationSpace(std::vector<Point> *points,
                        const Matrix &computationToTarget, Apply &&apply)
{
    Matrix affine = computationToTarget;
    affine[0][3]=affine[1][3]=affine[2][3]=0; affine[3][3]=1;
    Matrix targetToComputation = affine.GetInverse();
    targetToComputation[0][3]=targetToComputation[1][3]=targetToComputation[2][3]=0; targetToComputation[3][3]=1;
    auto local = *points;
    for (auto &p : local)
        p = Convert<Point>(targetToComputation.TransformAffine(Convert<Wide>(p)));
    if (!apply(&local)) return false;
    for (auto &p : local) {
        p = Convert<Point>(affine.TransformAffine(Convert<Wide>(p)));
        for (int axis = 0; axis < 3; ++axis) if (!std::isfinite(p[axis])) return false;
    }
    *points = std::move(local); return true;
}
} // namespace deltaMushDetail

/// Row-vector affine adapter. Reference points are already in computation
/// space; the identity map runs the plain kernel, bit for bit.
template<class Point, class Wide, class Matrix>
bool RigExecApplyDeltaMushInSpaceKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders, double distanceWeight,
    double displacement, const RigExecDeltaMushSettings &settings,
    const Matrix &computationToTarget)
{
    if (!points || !deltaMushDetail::ComputationMapValid(computationToTarget)) return false;
    if (computationToTarget == Matrix(1))
        return RigExecApplyDeltaMushKernel<Point, Wide>(points, rest, counts, indices,
            iterations, step, pinBorders, distanceWeight, displacement, settings);
    return deltaMushDetail::ApplyInComputationSpace<Point, Wide>(
        points, computationToTarget, [&](std::vector<Point> *local) {
            return RigExecApplyDeltaMushKernel<Point, Wide>(local, rest, counts, indices,
                iterations, step, pinBorders, distanceWeight, displacement, settings);
        });
}

/// RigExecApplyDeltaMushInSpaceKernel over prebuilt rest state: the same
/// arithmetic and failure conditions.
template<class Point, class Wide, class Matrix>
bool RigExecApplyDeltaMushInSpaceWithRestData(
    std::vector<Point> *points,
    const RigExecDeltaMushRestData<Point, Wide> &restData,
    double displacement, const Matrix &computationToTarget)
{
    if (!points || !deltaMushDetail::ComputationMapValid(computationToTarget)) return false;
    if (computationToTarget == Matrix(1))
        return RigExecApplyDeltaMushWithRestData(points, restData, displacement);
    return deltaMushDetail::ApplyInComputationSpace<Point, Wide>(
        points, computationToTarget, [&](std::vector<Point> *local) {
            return RigExecApplyDeltaMushWithRestData(local, restData, displacement);
        });
}

} // namespace rigExec
#endif
