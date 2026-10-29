// Guided quasistatic constraints based on Muller and Chentanez, Wrinkle Meshes
// (2010), https://matthias-research.github.io/pages/publications/wrinkleMeshes.pdf
// See docs/concepts/wrinkle-deformation.md for the stateless adaptation.
#ifndef RIGEXEC_MATH_WRINKLE_KERNEL_H
#define RIGEXEC_MATH_WRINKLE_KERNEL_H

#include "wrinkleSettings.h"
#include "meshConnectivity.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {
namespace wrinkleDetail {

template<class To, class From>
To Convert(const From &p) { return To(p[0], p[1], p[2]); }

template<class V>
bool IsFinite(const V &p)
{
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

template<class V>
double Length(const V &p)
{
    return std::hypot(double(p[0]), double(p[1]), double(p[2]));
}

template<class V>
double Dot(const V &a, const V &b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template<class V>
V Cross(const V &a, const V &b)
{
    return V(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]);
}

inline bool ValidSettings(const RigExecWrinkleSettings &s)
{
    const auto unit = [](float v) { return std::isfinite(v) && v >= 0 && v <= 1; };
    const auto positive = [](float v) { return std::isfinite(v) && v >= 0; };
    return s.iterations >= 0 && s.iterations <= 1000 &&
        (s.topology == RigExecWrinkleTopology::Cloth ||
         s.topology == RigExecWrinkleTopology::SurfaceStruts) &&
        s.neighborDistance >= 1 && s.neighborDistance <= 8 &&
        positive(s.restLengthScale) && s.restLengthScale > 0 &&
        unit(s.stretchStiffness) && unit(s.compressionStiffness) &&
        unit(s.bendStiffness) && positive(s.maxDisplacement) &&
        positive(s.tangentPlaneInset) && positive(s.wrinkleScale) &&
        s.smoothingIterations >= 0 && s.smoothingIterations <= 100;
}

inline double Seed(size_t index)
{
    uint32_t value = static_cast<uint32_t>(index) + 0x9e3779b9u;
    value = (value ^ (value >> 16)) * 0x85ebca6bu;
    value = (value ^ (value >> 13)) * 0xc2b2ae35u;
    value ^= value >> 16;
    return double(value >> 8) * (2.0 / 16777215.0) - 1.0;
}

struct Constraint {
    int a, b;
    double restLength;
    bool bending;
};

} // namespace wrinkleDetail

/// The wrinkle kernel's retained rest side: unscaled rest lengths for one
/// (rest, topology) pair, built once and shared across frames. The
/// per-frame restLengthScale stays in the apply half, so an animated
/// scale never invalidates the entry. Only the retainer's key (the rest
/// bits it was built from) ties the tables to a rest pose; the apply
/// half trusts a validated entry the way it trusts its topology entry.
struct RigExecWrinkleRestData {
    struct Entry {
        int a = -1;
        int b = -1;
        bool bending = false;
        double restLength = 0.0;
    };
    /// Constraint set in apply order (triangle edges, then bending
    /// pairs), zero-length pairs excluded exactly as the per-frame build
    /// excludes them. restLength is UNSCALED (times restLengthScale).
    std::vector<Entry> constraints;
    /// Flat unscaled rest lengths in (vertex, ring) order, parallel to
    /// the apply half's phase-weight rows.
    std::vector<double> neighborRest;
    /// The builder established every entry; only the posed points, the
    /// scale, and the stiffnesses are still frame business.
    bool validated = false;
};

/// Derive the rest tables for \p rest over \p topology: the constraint
/// set's unscaled lengths and every neighbor ring's unscaled lengths, in
/// the order the apply half reads them. Fails closed (false,
/// \p data unvalidated) on the same rest verdicts the per-frame build
/// returns: a size mismatch, a non-finite rest coordinate, or a
/// non-finite constraint length. Neighbor lengths are stored raw --
/// the apply half never verdict-ed them -- so the cached path reads
/// back exactly what the per-frame hypot computed.
template<class Point, class Wide>
bool
RigExecBuildWrinkleRestData(
    const std::vector<Point> &rest,
    const RigExecWrinkleMesh &topology,
    RigExecWrinkleRestData *data)
{
    using namespace wrinkleDetail;
    if (!data) return false;
    RigExecWrinkleRestData built;
    if (!topology.valid || rest.size() != topology.pointCount ||
        topology.neighbors.size() != topology.pointCount) {
        *data = std::move(built);
        return false;
    }
    std::vector<Wide> restWide;
    restWide.reserve(rest.size());
    for (const Point &point : rest) {
        if (!IsFinite(point)) {
            *data = std::move(built);
            return false;
        }
        restWide.push_back(Convert<Wide>(point));
    }
    const auto inRange = [&](int index) {
        return index >= 0 && size_t(index) < rest.size();
    };
    const auto addLength = [&](int a, int b, bool bending) {
        if (!inRange(a) || !inRange(b)) return false;
        const double length = Length(restWide[a] - restWide[b]);
        if (!std::isfinite(length)) return false;
        if (length > 0) built.constraints.push_back({a, b, bending, length});
        return true;
    };
    for (const RigExecWrinkleEdge &edge : topology.edges) {
        if (!addLength(edge.a, edge.b, false)) {
            *data = std::move(built);
            return false;
        }
    }
    for (const auto &pair : topology.bendingPairs) {
        if (!addLength(pair.first, pair.second, true)) {
            *data = std::move(built);
            return false;
        }
    }
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int neighbor : topology.neighbors[i]) {
            if (!inRange(neighbor)) {
                *data = std::move(built);
                return false;
            }
            built.neighborRest.push_back(
                Length(restWide[neighbor] - restWide[i]));
        }
    }
    built.validated = true;
    *data = std::move(built);
    return true;
}

/// Full 3D affine edge projections, bounded by incoming attachment balls and
/// optional outward tangent halfspaces. Point/Wide provide storage
/// and arithmetic only; this shared implementation has no USD dependency.
///
/// The apply half of the split: \p topology carries the cached
/// triangulation, sorted edges, neighbor rings, border vertices, and
/// bending pairs RigExecBuildWrinkleMesh derived from (counts, indices),
/// and the same arithmetic the single-call kernel performs runs over it.
/// A topology built from anything else fails closed, exactly as its own
/// build would have verdict-ed. \p restData optionally supplies the
/// retained rest lengths for this rest pose; null derives them
/// per frame, bit-identically.
template<class Point, class Wide>
bool
RigExecApplyWrinkleWithTopology(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const RigExecWrinkleSettings &settings,
    const RigExecWrinkleMesh &topology,
    const RigExecWrinkleRestData *restData = nullptr)
{
    using namespace wrinkleDetail;
    if (!points || points->size() != rest.size() || !ValidSettings(settings)) return false;
    if (restData && !restData->validated) return false;
    const size_t pointCount = rest.size();
    std::vector<Wide> incoming, restWide;
    incoming.reserve(pointCount);
    if (!restData) restWide.reserve(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        if (!IsFinite(rest[i]) || !IsFinite((*points)[i])) return false;
        incoming.push_back(Convert<Wide>((*points)[i]));
        if (!restData) restWide.push_back(Convert<Wide>(rest[i]));
    }
    // Bytes, not bits: the packed probes of vector<bool> sit in the
    // per-vertex projection hot path below.
    std::vector<char> pinned(pointCount, 0);
    for (int index : settings.pinPoints) {
        if (index < 0 || size_t(index) >= pointCount || pinned[index]) return false;
        pinned[index] = 1;
    }
    if (!topology.Covers(pointCount, counts, indices,
                         settings.topology, settings.neighborDistance)) {
        return false;
    }

    // Posed face normals: the one face walk the apply half still makes, in
    // the single-call kernel's order. The shape verdicts (short faces,
    // overruns, range, duplicates) are the cached build's, covered above;
    // the area finiteness verdict is posed, so it stays here.
    std::vector<Wide> normals(pointCount, Wide(0));
    size_t offset = 0;
    for (int count : counts) {
        const int origin = indices[offset];
        Wide area(0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            area += Cross(incoming[indices[offset + corner]] - incoming[origin],
                          incoming[indices[offset + corner + 1]] - incoming[origin]);
        }
        if (!IsFinite(area)) return false;
        for (int corner = 0; corner < count; ++corner) normals[indices[offset + corner]] += area;
        offset += size_t(count);
    }
    if (settings.pinBorders) {
        for (int index : topology.borderVertices) pinned[index] = 1;
    }
    for (size_t i = 0; i < pointCount; ++i) {
        const double length = Length(normals[i]);
        if (!std::isfinite(length)) return false;
        if (length == 0) pinned[i] = 1;
        else normals[i] /= length;
    }
    if (pointCount == 0 || settings.iterations == 0 || settings.maxDisplacement == 0 ||
        settings.wrinkleScale == 0) return true;

    // The cached triangle edges supply stable solve order and the cached
    // bending pairs the distance-based bending model; rest lengths come
    // from the retained tables when supplied, else per-frame, exactly as
    // the single-call kernel derived them.
    const std::vector<std::vector<int>> &neighbors = topology.neighbors;
    std::vector<Constraint> constraints;
    const auto addConstraint = [&](int a, int b, bool bending) {
        const double length = Length(restWide[a] - restWide[b]);
        if (!std::isfinite(length)) return false;
        if (length > 0) {
            const double scaledLength = length * double(settings.restLengthScale);
            if (!std::isfinite(scaledLength)) return false;
            constraints.push_back({a, b, scaledLength, bending});
        }
        return true;
    };
    if (restData) {
        constraints.reserve(restData->constraints.size());
        for (const auto &cached : restData->constraints) {
            const double scaledLength =
                cached.restLength * double(settings.restLengthScale);
            if (!std::isfinite(scaledLength)) return false;
            constraints.push_back(
                {cached.a, cached.b, scaledLength, cached.bending});
        }
    } else {
        for (const RigExecWrinkleEdge &edge : topology.edges) {
            if (!addConstraint(edge.a, edge.b, false)) return false;
        }
        for (const auto &pair : topology.bendingPairs) {
            if (!addConstraint(pair.first, pair.second, true)) return false;
        }
    }

    // Equal stretch/compression stiffnesses make (selected - shared) an
    // exact +0, so the blend term below vanishes: the slow path computes
    // shared + 0.0 for every non-bending constraint. Return that directly.
    // The +0.0 keeps the slow path's -0.0-to-+0.0 mapping bit-identical;
    // unequal stiffnesses take the slow path as before.
    const bool uniformStiffness =
        settings.stretchStiffness == settings.compressionStiffness;
    const double uniformShared = double(std::min(settings.compressionStiffness,
                                                 settings.stretchStiffness));
    const auto stiffness = [&](const Constraint &constraint, double length) {
        if (constraint.bending) return double(settings.bendStiffness);
        if (uniformStiffness) return uniformShared + 0.0;
        const double selected = length < constraint.restLength
            ? settings.compressionStiffness : settings.stretchStiffness;
        const double shared = std::min(settings.compressionStiffness, settings.stretchStiffness);
        const double strain = std::abs(length / constraint.restLength - 1.0);
        const double blend = std::min(1.0, strain / .02);
        // Unequal stiffnesses must not switch the entire linear system at
        // zero strain. This blend also preserves an exactly disabled branch.
        return shared + (selected - shared) * blend * blend * (3.0 - 2.0 * blend);
    };
    const double strainTolerance = 1e-6;
    bool active = false;
    std::vector<double> compression(pointCount, 0);
    for (const auto &constraint : constraints) {
        if (pinned[constraint.a] && pinned[constraint.b]) continue;
        const double length = Length(incoming[constraint.b] - incoming[constraint.a]);
        if (!std::isfinite(length)) return false;
        if (stiffness(constraint, length) == 0) continue;
        const double strain = 1.0 - length / constraint.restLength;
        if (std::abs(strain) <= strainTolerance) continue;
        active = true;
        if (strain > strainTolerance) {
            compression[constraint.a] = std::max(compression[constraint.a], strain);
            compression[constraint.b] = std::max(compression[constraint.b], strain);
        }
    }
    if (!active) return true;

    // Squared lengths at or below shortSquared need no hypot: they sit a
    // 2^-20 band below maxDisplacement squared, far outside any rounding
    // between the dot and the hypot, so the scale verdict is exact. Maxima
    // below 1e-150 (whose square nears the subnormals) and
    // squared-overflowing maxima never skip; NaN, infinite, band, and
    // longer steps take the hypot path as before.
    const double maxStep = double(settings.maxDisplacement);
    const double stepSquared = maxStep * maxStep;
    const double shortSquared = (maxStep < 1e-150 || !std::isfinite(stepSquared))
        ? -1.0 : stepSquared * (1.0 - 0.00000095367431640625);
    const auto project = [&](size_t i, Wide value) {
        if (pinned[i]) return incoming[i];
        Wide delta = value - incoming[i];
        if (settings.tangentPlaneCollisions) {
            const double height = Dot(delta, normals[i]);
            if (height < -double(settings.tangentPlaneInset))
                delta += (-double(settings.tangentPlaneInset) - height) * normals[i];
        }
        if (!(Dot(delta, delta) <= shortSquared)) {
            const double length = Length(delta);
            if (length > maxStep)
                delta *= maxStep / length;
        }
        return incoming[i] + delta;
    };

    // Transport a material-index phase field along relatively uncompressed
    // edges. Smooth weights keep the field continuous at compression onset.
    // One flat row per vertex: the same weights in the same order, without
    // a heap block per vertex.
    std::vector<double> phase(pointCount);
    std::vector<size_t> phaseOffsets(pointCount + 1, 0);
    for (size_t i = 0; i < pointCount; ++i)
        phaseOffsets[i + 1] = phaseOffsets[i] + neighbors[i].size();
    if (restData && restData->neighborRest.size() != phaseOffsets[pointCount])
        return false;
    std::vector<double> phaseWeights(phaseOffsets[pointCount]);
    for (size_t i = 0; i < pointCount; ++i) {
        phase[i] = Seed(i);
        double *row = phaseWeights.data() + phaseOffsets[i];
        const double *cachedRow = restData
            ? restData->neighborRest.data() + phaseOffsets[i] : nullptr;
        size_t j = 0;
        for (int neighbor : neighbors[i]) {
            const double unscaled = restData ? cachedRow[j]
                : Length(restWide[neighbor] - restWide[i]);
            const double reference = unscaled * double(settings.restLengthScale);
            const double strain = reference > 0 ? std::max(0.0,
                1.0 - Length(incoming[neighbor] - incoming[i]) / reference) : 0;
            const double localCompression = std::max(compression[i], compression[neighbor]);
            // An exactly 0.0 or 1.0 base needs no libm call: pow(0,8) is
            // +0.0 and pow(1,8) is 1.0, so storing the base is
            // bit-identical. Unstrained edges land on 1.0 (a third of the
            // weights on the probe grid); NaN cannot reach here (the max
            // maps it to 0.0, whose pow is likewise +0.0).
            const double base = std::max(0.0,
                1.0 - strain / (localCompression + .002));
            row[j++] = (base == 1.0 || base == 0.0) ? base : std::pow(base, 8);
        }
    }
    // One alternate buffer for all passes, swapped between them: each
    // pass writes every element (the else keeps the per-pass copy's
    // behavior for zero-weight vertices), so no pass reads stale data.
    std::vector<double> nextPhase = phase;
    for (int pass = 0; pass < 10; ++pass) {
        for (size_t i = 0; i < pointCount; ++i) {
            double sum = 0, weight = 0;
            const double *row = phaseWeights.data() + phaseOffsets[i];
            for (size_t j = 0; j < neighbors[i].size(); ++j) {
                sum += phase[neighbors[i][j]] * row[j];
                weight += row[j];
            }
            nextPhase[i] = weight > 0 ? .5 * (phase[i] + sum / weight)
                                      : phase[i];
        }
        phase.swap(nextPhase);
    }
    const auto phaseRange = std::minmax_element(phase.begin(), phase.end());
    const double phaseWidth = .1 * (*phaseRange.second - *phaseRange.first) + 1e-6;

    // Fix each edge's lift branch before relaxation. A soft phase sign and
    // smooth missing-length onset avoid introducing threshold jumps. Unlike
    // nonlinear distance projection, the solve cannot change buckling basins.
    std::vector<Wide> targets;
    std::vector<double> strengths;
    targets.reserve(constraints.size());
    strengths.reserve(constraints.size());
    for (const auto &constraint : constraints) {
        const int a = constraint.a, b = constraint.b;
        const Wide baseEdge = incoming[b] - incoming[a];
        const double length = Length(baseEdge);
        Wide target = baseEdge;
        if (length > constraint.restLength) {
            target *= constraint.restLength / length;
        } else {
            Wide normal = normals[a] + normals[b];
            const double normalLength = Length(normal);
            if (normalLength > 0) {
                normal /= normalLength;
                const double ratio = length / constraint.restLength;
                const double deficit = std::max(0.0, 1.0 - ratio * ratio);
                const double difference = phase[b] - phase[a];
                const double height = difference / std::hypot(difference, phaseWidth) *
                    constraint.restLength * deficit / std::sqrt(deficit + .02);
                target += height * normal;
            }
        }
        if (!IsFinite(target)) return false;
        targets.push_back(target);
        strengths.push_back(stiffness(constraint, length));
    }

    // Per-constraint relaxation rate and move mask, fixed for the solve:
    // the strengths and pins never change between iterations. The rate is
    // the same quotient the loop divided per visit, so the correction is
    // bit-identical; a fully pinned constraint keeps its skip.
    std::vector<double> rates(constraints.size());
    std::vector<char> moveMask(constraints.size());
    for (size_t index = 0; index < constraints.size(); ++index) {
        const auto &constraint = constraints[index];
        const double weightA = pinned[constraint.a] ? 0.0 : 1.0;
        const double weightB = pinned[constraint.b] ? 0.0 : 1.0;
        moveMask[index] = char((weightA ? 1 : 0) | (weightB ? 2 : 0));
        rates[index] = moveMask[index]
            ? strengths[index] / (weightA + weightB) : 0.0;
    }
    std::vector<Wide> current = incoming;
    for (int iteration = 0; iteration < settings.iterations; ++iteration) {
        for (size_t cursor = 0; cursor < constraints.size(); ++cursor) {
            const size_t index = iteration % 2 ? constraints.size() - 1 - cursor : cursor;
            const char mask = moveMask[index];
            if (!mask) continue;
            const auto &constraint = constraints[index];
            const int a = constraint.a, b = constraint.b;
            const Wide edge = current[b] - current[a];
            const Wide correction = (edge - targets[index]) * rates[index];
            if (mask & 1) current[a] += correction;
            if (mask & 2) current[b] -= correction;
        }
        for (size_t i = 0; i < pointCount; ++i) {
            if (!IsFinite(current[i])) return false;
            current[i] = project(i, current[i]);
        }
    }

    std::vector<Wide> deltas(pointCount);
    for (size_t i = 0; i < pointCount; ++i) deltas[i] = current[i] - incoming[i];
    // As above: pinned and isolated vertices keep their values
    // through the else, exactly as the per-iteration copy did.
    std::vector<Wide> nextDeltas = deltas;
    for (int iteration = 0; iteration < settings.smoothingIterations; ++iteration) {
        for (size_t i = 0; i < pointCount; ++i) {
            if (pinned[i] || neighbors[i].empty()) {
                nextDeltas[i] = deltas[i];
                continue;
            }
            Wide average(0);
            for (int neighbor : neighbors[i]) average += deltas[neighbor];
            average /= double(neighbors[i].size());
            nextDeltas[i] = .5 * (deltas[i] + average);
        }
        deltas.swap(nextDeltas);
    }
    std::vector<Point> result = *points;
    for (size_t i = 0; i < pointCount; ++i) {
        if (pinned[i]) continue;
        const Wide value = project(i, incoming[i] + double(settings.wrinkleScale) * deltas[i]);
        if (!IsFinite(value)) return false;
        result[i] = Convert<Point>(value);
        if (!IsFinite(result[i])) return false;
    }
    *points = std::move(result);
    return true;
}

/// Single-call form: builds the topology half, then applies it. Behavior
/// is bit-exact against the split by construction; revisions that deform
/// the same mesh every frame retain the topology and call the apply half.
template<class Point, class Wide>
bool
RigExecApplyWrinkleKernel(
    std::vector<Point> *points, const std::vector<Point> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    const RigExecWrinkleSettings &settings)
{
    RigExecWrinkleMesh topology;
    if (!RigExecBuildWrinkleMesh(rest.size(), counts, indices,
                                 settings.topology,
                                 settings.neighborDistance, &topology)) {
        return false;
    }
    return RigExecApplyWrinkleWithTopology<Point, Wide>(
        points, rest, counts, indices, settings, topology);
}

} // namespace rigExec
#endif // RIGEXEC_MATH_WRINKLE_KERNEL_H
