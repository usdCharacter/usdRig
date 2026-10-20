// Method references (docs/references.md):
// - Macklin, Mueller and Chentanez, XPBD: Position-Based Simulation of
//   Compliant Constrained Dynamics (MIG 2016): compliant distance
//   constraints whose stiffness does not depend on the step count.
// - Macklin et al., Small Steps in Physics Simulation (SCA 2019): many
//   substeps with one iteration each, the best accuracy per unit of work.
// - Fratarcangeli, Tibaldo and Pellacini, Vivace: a Practical Gauss-Seidel
//   Method for Stable Soft Body Dynamics (SIGGRAPH Asia 2016): constraints
//   colored so no two of a color share a particle, each color solved in
//   parallel and the colors in sequence.
// - Kim, Chentanez and Mueller, Long Range Attachments (SCA 2012): tethers
//   from every free particle to its nearest pinned one.
// - Teschner et al., Optimized Spatial Hashing for Collision Detection of
//   Deformable Objects (VMV 2003): the collider triangle grid.
// - Bridson, Fedkiw and Anderson, Robust Treatment of Collisions, Contact
//   and Friction for Cloth Animation (SIGGRAPH 2002): collision thickness
//   and the positional friction rule.
//
// RigExecClothMover's solver. Header-only and USD-free, so the evaluators,
// the zero-USD runtime and the GPU shader (libs/rigExecGpu/clothShader.cpp)
// run one definition of every pass.
//
// The solve is a pure function of its window: the cloth starts at the oldest
// tick on its entering points, moving with them, and is simulated tick by
// tick to the newest, which is the current frame. Nothing survives between
// solves.
//
// Determinism: a constraint color is a set of constraints that share no
// particle, so its members may run in any order or side by side and write
// the same bits. Every other pass reads and writes one particle. The CPU loop
// and the GPU shader therefore agree for the same arithmetic.
#ifndef RIGEXEC_MATH_CLOTH_KERNEL_H
#define RIGEXEC_MATH_CLOTH_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

/// The longest window a cloth mover may simulate, in frames.
constexpr int RigExecClothMaxWindowFrames = 1024;
constexpr int RigExecClothMaxSubsteps = 256;
constexpr int RigExecClothMaxIterations = 64;

/// Constraint kinds in RigExecClothMesh::constraintKind.
enum class RigExecClothConstraint : uint8_t { Stretch = 0, Bend = 1 };

/// Epoch settings: they fix the window and the constraint set, so an edit
/// recompiles.
struct RigExecClothSettings {
    /// Frames simulated before the current one.
    int windowFrames = 16;
    int substeps = 10;
    /// Constraint and collision passes per substep.
    int iterations = 1;
    /// Tether every free particle to its nearest fully pinned particle.
    bool tethers = true;
    /// How far past its rest distance a tether lets a particle go, as a
    /// fraction of that distance.
    float tetherSlack = 0.05f;
    /// Pin weights at or above this make a particle kinematic.
    float pinThreshold = 0.999f;

    bool operator==(const RigExecClothSettings &o) const
    {
        return windowFrames == o.windowFrames && substeps == o.substeps &&
               iterations == o.iterations && tethers == o.tethers &&
               tetherSlack == o.tetherSlack && pinThreshold == o.pinThreshold;
    }
    bool operator!=(const RigExecClothSettings &o) const { return !(*this == o); }
};

/// Per-tick values. Read at every tick of the window, so an animated value
/// is simulated as animated.
struct RigExecClothParameters {
    /// XPBD compliance (inverse stiffness) of the edges and of the bend
    /// struts across shared edges; 0 is rigid.
    float stretchCompliance = 0.0f;
    float bendCompliance = 1.0e-3f;
    /// Scene units per second squared.
    float gravity[3] = {0.0f, -981.0f, 0.0f};
    /// The fraction of velocity lost per second.
    float damping = 0.1f;
    /// The fraction of the gap to the entering points a free particle
    /// closes per frame: 0 is free, 1 follows the input.
    float attract = 0.0f;
    /// Collision distance kept from collider surfaces.
    float thickness = 0.5f;
    /// How deep behind a collider surface a particle is still pushed out.
    float collisionDepth = 2.0f;
    /// Coulomb-style friction against colliders.
    float friction = 0.3f;

    bool Finite() const
    {
        const float v[] = {stretchCompliance, bendCompliance, gravity[0],
                           gravity[1], gravity[2], damping, attract,
                           thickness, collisionDepth, friction};
        for (float x : v)
            if (!std::isfinite(x)) return false;
        return true;
    }
};

/// What a cloth mesh fixes per epoch: triangles, colored constraints and
/// their rest lengths, masses, pins and tethers. Built from the topology,
/// the rest points and the pin weights.
struct RigExecClothMesh {
    size_t pointCount = 0;
    /// Three corners per triangle, fan-triangulated, in face order.
    std::vector<int32_t> triangles;
    /// Two particles per constraint, lower index first. Constraints are
    /// grouped by color; color k is [colorStart[k], colorStart[k+1]).
    std::vector<int32_t> constraintEnds;
    std::vector<float> constraintRest;
    std::vector<uint8_t> constraintKind;
    std::vector<uint32_t> colorStart;
    /// Zero for a kinematic (fully pinned) particle.
    std::vector<float> inverseMass;
    /// Clamped to [0, 1]; 0 everywhere without pins.
    std::vector<float> pin;
    /// The particle each free particle is tethered to, or -1, and the rest
    /// distance.
    std::vector<int32_t> tetherAnchor;
    std::vector<float> tetherRest;
    bool valid = false;

    size_t ColorCount() const { return colorStart.empty() ? 0 : colorStart.size() - 1; }
};

/// Collider triangles over the concatenated collider points.
struct RigExecClothColliders {
    size_t pointCount = 0;
    std::vector<int32_t> triangles;
};

/// One tick of the window: the entering points of the cloth, the collider
/// points, and the tick's parameters. Pointers borrow the caller's storage
/// for the solve.
struct RigExecClothTick {
    const float *entering = nullptr;
    const float *colliders = nullptr;
    RigExecClothParameters parameters;
};

/// One tick's collider grid: a Teschner-hashed uniform grid of the triangles
/// swept between the previous tick and this one. CSR over hash buckets,
/// triangles ascending within a bucket.
struct RigExecClothGrid {
    float cellSize = 1.0f;
    uint32_t bucketMask = 0;
    std::vector<uint32_t> bucketStart;
    std::vector<int32_t> bucketTriangles;
};

/// Scratch a solve reuses. Owned by the one body that solves; never shared.
struct RigExecClothWorkspace {
    std::vector<float> x, previous, velocity, lambda;
    RigExecClothGrid grid;
    std::vector<uint64_t> cellScratch;
};

namespace cloth_detail {

inline void Sub(const float *a, const float *b, float *o)
{
    o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2];
}
inline float Dot(const float *a, const float *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline void Cross(const float *a, const float *b, float *o)
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
inline float Length(const float *a) { return std::sqrt(Dot(a, a)); }
inline void Lerp(const float *a, const float *b, float t, float *o)
{
    o[0] = a[0] + (b[0] - a[0]) * t;
    o[1] = a[1] + (b[1] - a[1]) * t;
    o[2] = a[2] + (b[2] - a[2]) * t;
}

inline int64_t Cell(float v, float cell) { return int64_t(std::floor(v / cell)); }

inline uint32_t Hash(int64_t x, int64_t y, int64_t z, uint32_t mask)
{
    const uint32_t h = (uint32_t(x) * 73856093u) ^ (uint32_t(y) * 19349663u) ^
                       (uint32_t(z) * 83492791u);
    return h & mask;
}

/// Fan triangles of polygon faces, flipped for a left-handed mesh so every
/// normal points out of the surface.
inline bool Triangulate(size_t pointCount, const int *counts, size_t countSize,
                        const int *indices, size_t indexSize, bool leftHanded,
                        std::vector<int32_t> *out)
{
    out->clear();
    size_t offset = 0;
    for (size_t f = 0; f < countSize; ++f) {
        const int n = counts[f];
        if (n < 3 || size_t(n) > indexSize - offset) return false;
        for (int c = 0; c < n; ++c) {
            const int v = indices[offset + c];
            if (v < 0 || size_t(v) >= pointCount) return false;
        }
        for (int c = 1; c + 1 < n; ++c) {
            const int a = indices[offset], b = indices[offset + c],
                      d = indices[offset + c + 1];
            out->push_back(a);
            out->push_back(leftHanded ? d : b);
            out->push_back(leftHanded ? b : d);
        }
        offset += size_t(n);
    }
    return offset == indexSize;
}

/// The closest point of triangle (a, b, c) to p (Ericson, Real-Time
/// Collision Detection, 5.1.5), as barycentric weights.
inline void ClosestBarycentric(const float *p, const float *a, const float *b,
                               const float *c, float *w)
{
    float ab[3], ac[3], ap[3];
    Sub(b, a, ab); Sub(c, a, ac); Sub(p, a, ap);
    const float d1 = Dot(ab, ap), d2 = Dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { w[0] = 1; w[1] = 0; w[2] = 0; return; }
    float bp[3]; Sub(p, b, bp);
    const float d3 = Dot(ab, bp), d4 = Dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { w[0] = 0; w[1] = 1; w[2] = 0; return; }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        w[0] = 1 - v; w[1] = v; w[2] = 0; return;
    }
    float cp[3]; Sub(p, c, cp);
    const float d5 = Dot(ab, cp), d6 = Dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { w[0] = 0; w[1] = 0; w[2] = 1; return; }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float v = d2 / (d2 - d6);
        w[0] = 1 - v; w[1] = 0; w[2] = v; return;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        w[0] = 0; w[1] = 1 - v; w[2] = v; return;
    }
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom, u = vc * denom;
    w[0] = 1.0f - v - u; w[1] = v; w[2] = u;
}

}  // namespace cloth_detail

/// Builds the cloth's epoch half. \p restPoints holds pointCount float3s;
/// \p pinWeights is null or pointCount floats. False with \p why on bad
/// topology, a non-finite rest point or pin, or a degenerate edge.
inline bool
RigExecBuildClothMesh(size_t pointCount, const int *counts, size_t countSize,
                      const int *indices, size_t indexSize, bool leftHanded,
                      const float *restPoints, const float *pinWeights,
                      const RigExecClothSettings &settings,
                      RigExecClothMesh *out, std::string *why = nullptr)
{
    using namespace cloth_detail;
    const auto fail = [why](const char *text) {
        if (why) *why = text;
        return false;
    };
    if (!out || !restPoints) return fail("missing rest points");
    if (pointCount == 0) return fail("the cloth mesh has no points");
    for (size_t i = 0; i < pointCount * 3; ++i)
        if (!std::isfinite(restPoints[i])) return fail("a rest point is not finite");
    RigExecClothMesh built;
    built.pointCount = pointCount;
    if (!Triangulate(pointCount, counts, countSize, indices, indexSize,
                     leftHanded, &built.triangles)) {
        return fail("the cloth mesh topology is invalid");
    }
    // Edges with the opposite corner of each triangle using them.
    struct Edge { int32_t a, b, opposite; };
    std::vector<Edge> edges;
    edges.reserve(built.triangles.size());
    for (size_t t = 0; t + 2 < built.triangles.size(); t += 3) {
        for (int e = 0; e < 3; ++e) {
            const int32_t p = built.triangles[t + e];
            const int32_t q = built.triangles[t + (e + 1) % 3];
            const int32_t r = built.triangles[t + (e + 2) % 3];
            edges.push_back({std::min(p, q), std::max(p, q), r});
        }
    }
    std::sort(edges.begin(), edges.end(), [](const Edge &l, const Edge &r) {
        if (l.a != r.a) return l.a < r.a;
        if (l.b != r.b) return l.b < r.b;
        return l.opposite < r.opposite;
    });
    const auto rest = [restPoints](int32_t a, int32_t b) {
        float d[3];
        Sub(restPoints + 3 * size_t(a), restPoints + 3 * size_t(b), d);
        return Length(d);
    };
    struct Constraint { int32_t a, b; float rest; uint8_t kind; };
    std::vector<Constraint> constraints;
    for (size_t i = 0; i < edges.size();) {
        size_t j = i;
        while (j < edges.size() && edges[j].a == edges[i].a && edges[j].b == edges[i].b) ++j;
        if (edges[i].a == edges[i].b) return fail("a face repeats a corner");
        const float length = rest(edges[i].a, edges[i].b);
        if (!(length > 0.0f)) return fail("a cloth edge has zero rest length");
        constraints.push_back({edges[i].a, edges[i].b, length,
                               uint8_t(RigExecClothConstraint::Stretch)});
        // A strut between the opposite corners of each pair of triangles
        // sharing the edge resists bending.
        for (size_t p = i; p < j; ++p) {
            for (size_t q = p + 1; q < j; ++q) {
                const int32_t u = std::min(edges[p].opposite, edges[q].opposite);
                const int32_t v = std::max(edges[p].opposite, edges[q].opposite);
                if (u == v) continue;
                const float strut = rest(u, v);
                if (!(strut > 0.0f)) continue;
                constraints.push_back({u, v, strut, uint8_t(RigExecClothConstraint::Bend)});
            }
        }
        i = j;
    }
    std::sort(constraints.begin(), constraints.end(),
              [](const Constraint &l, const Constraint &r) {
                  if (l.kind != r.kind) return l.kind < r.kind;
                  if (l.a != r.a) return l.a < r.a;
                  return l.b < r.b;
              });
    constraints.erase(std::unique(constraints.begin(), constraints.end(),
                                  [](const Constraint &l, const Constraint &r) {
                                      return l.kind == r.kind && l.a == r.a && l.b == r.b;
                                  }),
                      constraints.end());
    // Greedy coloring in constraint order: each takes the lowest color
    // neither of its particles holds yet.
    std::vector<std::vector<uint32_t>> held(pointCount);
    std::vector<uint32_t> color(constraints.size());
    uint32_t colors = 0;
    for (size_t c = 0; c < constraints.size(); ++c) {
        const auto &ha = held[size_t(constraints[c].a)];
        const auto &hb = held[size_t(constraints[c].b)];
        uint32_t k = 0;
        while (std::find(ha.begin(), ha.end(), k) != ha.end() ||
               std::find(hb.begin(), hb.end(), k) != hb.end()) {
            ++k;
        }
        color[c] = k;
        held[size_t(constraints[c].a)].push_back(k);
        held[size_t(constraints[c].b)].push_back(k);
        colors = std::max(colors, k + 1);
    }
    built.colorStart.assign(size_t(colors) + 1, 0);
    for (uint32_t k : color) ++built.colorStart[size_t(k) + 1];
    for (size_t k = 0; k < colors; ++k) built.colorStart[k + 1] += built.colorStart[k];
    built.constraintEnds.resize(2 * constraints.size());
    built.constraintRest.resize(constraints.size());
    built.constraintKind.resize(constraints.size());
    {
        std::vector<uint32_t> fill(built.colorStart.begin(), built.colorStart.end() - 1);
        for (size_t c = 0; c < constraints.size(); ++c) {
            const uint32_t at = fill[color[c]]++;
            built.constraintEnds[2 * at] = constraints[c].a;
            built.constraintEnds[2 * at + 1] = constraints[c].b;
            built.constraintRest[at] = constraints[c].rest;
            built.constraintKind[at] = constraints[c].kind;
        }
    }
    // Lumped masses: a third of each adjacent triangle's rest area,
    // normalized to a mean of one so compliance does not scale with units.
    std::vector<double> mass(pointCount, 0.0);
    for (size_t t = 0; t + 2 < built.triangles.size(); t += 3) {
        const float *a = restPoints + 3 * size_t(built.triangles[t]);
        const float *b = restPoints + 3 * size_t(built.triangles[t + 1]);
        const float *c = restPoints + 3 * size_t(built.triangles[t + 2]);
        float ab[3], ac[3], n[3];
        Sub(b, a, ab); Sub(c, a, ac); Cross(ab, ac, n);
        const double area = 0.5 * double(Length(n));
        for (int k = 0; k < 3; ++k) mass[size_t(built.triangles[t + k])] += area / 3.0;
    }
    double total = 0.0;
    size_t massive = 0;
    for (double m : mass) if (m > 0.0) { total += m; ++massive; }
    const double mean = massive ? total / double(massive) : 1.0;
    built.pin.assign(pointCount, 0.0f);
    built.inverseMass.assign(pointCount, 0.0f);
    for (size_t i = 0; i < pointCount; ++i) {
        float p = pinWeights ? pinWeights[i] : 0.0f;
        if (!std::isfinite(p)) return fail("a pin weight is not finite");
        p = std::min(1.0f, std::max(0.0f, p));
        built.pin[i] = p;
        const double m = mass[i] > 0.0 ? mass[i] / mean : 1.0;
        built.inverseMass[i] = p >= settings.pinThreshold ? 0.0f : float(1.0 / m);
    }
    // Tethers: each free particle to its nearest kinematic particle in the
    // rest shape, lowest index on a tie.
    built.tetherAnchor.assign(pointCount, -1);
    built.tetherRest.assign(pointCount, 0.0f);
    if (settings.tethers) {
        std::vector<int32_t> anchors;
        for (size_t i = 0; i < pointCount; ++i)
            if (built.inverseMass[i] == 0.0f) anchors.push_back(int32_t(i));
        if (!anchors.empty()) {
            for (size_t i = 0; i < pointCount; ++i) {
                if (built.inverseMass[i] == 0.0f) continue;
                float best = std::numeric_limits<float>::infinity();
                int32_t bestAnchor = -1;
                for (int32_t a : anchors) {
                    const float d = rest(int32_t(i), a);
                    if (d < best) { best = d; bestAnchor = a; }
                }
                built.tetherAnchor[i] = bestAnchor;
                built.tetherRest[i] = best;
            }
        }
    }
    built.valid = true;
    *out = std::move(built);
    return true;
}

/// Appends one collider mesh's triangles, offset by the points already
/// collected. False on bad topology.
inline bool
RigExecAppendClothCollider(size_t pointCount, const int *counts,
                           size_t countSize, const int *indices,
                           size_t indexSize, bool leftHanded,
                           RigExecClothColliders *colliders)
{
    std::vector<int32_t> triangles;
    if (!cloth_detail::Triangulate(pointCount, counts, countSize, indices,
                                   indexSize, leftHanded, &triangles)) {
        return false;
    }
    for (int32_t &v : triangles) v += int32_t(colliders->pointCount);
    colliders->triangles.insert(colliders->triangles.end(), triangles.begin(),
                                triangles.end());
    colliders->pointCount += pointCount;
    return true;
}

/// Builds the grid of \p colliders swept from \p before to \p after (point
/// arrays of colliders.pointCount float3s), padded by \p pad. Triangles are
/// listed in ascending order in every bucket. \p scratch is reused storage.
inline void
RigExecBuildClothGrid(const RigExecClothColliders &colliders,
                      const float *before, const float *after, float pad,
                      RigExecClothGrid *grid, std::vector<uint64_t> *scratch)
{
    using namespace cloth_detail;
    const size_t triangleCount = colliders.triangles.size() / 3;
    grid->bucketStart.clear();
    grid->bucketTriangles.clear();
    if (!triangleCount) {
        grid->bucketMask = 0;
        grid->bucketStart.assign(2, 0);
        return;
    }
    const auto bounds = [&](size_t t, float *lo, float *hi) {
        bool finite = true;
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::numeric_limits<float>::infinity();
            hi[a] = -std::numeric_limits<float>::infinity();
        }
        for (int k = 0; k < 3; ++k) {
            const size_t v = size_t(colliders.triangles[3 * t + size_t(k)]);
            for (int a = 0; a < 3; ++a) {
                const float p = before[3 * v + size_t(a)], q = after[3 * v + size_t(a)];
                finite = finite && std::isfinite(p) && std::isfinite(q);
                lo[a] = std::min(lo[a], std::min(p, q));
                hi[a] = std::max(hi[a], std::max(p, q));
            }
        }
        return finite;
    };
    // Cell size: the mean swept triangle extent, at least twice the pad.
    double extent = 0.0;
    size_t measured = 0;
    for (size_t t = 0; t < triangleCount; ++t) {
        float lo[3], hi[3];
        if (!bounds(t, lo, hi)) continue;
        extent += double(std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2])));
        ++measured;
    }
    float cell = measured ? float(extent / double(measured)) : 1.0f;
    cell = std::max(cell, 2.0f * pad);
    if (!(cell > 0.0f) || !std::isfinite(cell)) cell = 1.0f;
    grid->cellSize = cell;
    uint32_t buckets = 1;
    while (buckets < 2 * triangleCount && buckets < (1u << 24)) buckets <<= 1;
    grid->bucketMask = buckets - 1;
    // Each triangle's buckets, once each, in triangle order; then a counting
    // sort by bucket, which keeps the triangles of a bucket ascending.
    scratch->clear();
    for (size_t t = 0; t < triangleCount; ++t) {
        float lo[3], hi[3];
        if (!bounds(t, lo, hi)) continue;
        int64_t c0[3], c1[3];
        for (int a = 0; a < 3; ++a) {
            c0[a] = Cell(lo[a] - pad, cell);
            c1[a] = Cell(hi[a] + pad, cell);
        }
        // A triangle swept across more cells than the table holds is
        // listed in every bucket once.
        const uint64_t span = uint64_t(c1[0] - c0[0] + 1) *
                              uint64_t(c1[1] - c0[1] + 1) *
                              uint64_t(c1[2] - c0[2] + 1);
        if (span >= buckets) {
            for (uint32_t b = 0; b < buckets; ++b)
                scratch->push_back((uint64_t(t) << 32) | uint64_t(b));
            continue;
        }
        const size_t first = scratch->size();
        for (int64_t x = c0[0]; x <= c1[0]; ++x)
            for (int64_t y = c0[1]; y <= c1[1]; ++y)
                for (int64_t z = c0[2]; z <= c1[2]; ++z)
                    scratch->push_back((uint64_t(t) << 32) |
                                       uint64_t(Hash(x, y, z, grid->bucketMask)));
        std::sort(scratch->begin() + std::ptrdiff_t(first), scratch->end());
        scratch->erase(std::unique(scratch->begin() + std::ptrdiff_t(first), scratch->end()),
                       scratch->end());
    }
    grid->bucketStart.assign(size_t(buckets) + 1, 0);
    for (uint64_t e : *scratch) ++grid->bucketStart[size_t(e & 0xffffffffull) + 1];
    for (size_t b = 0; b < buckets; ++b) grid->bucketStart[b + 1] += grid->bucketStart[b];
    grid->bucketTriangles.resize(scratch->size());
    std::vector<uint32_t> cursor(grid->bucketStart.begin(), grid->bucketStart.end() - 1);
    for (uint64_t e : *scratch)
        grid->bucketTriangles[cursor[size_t(e & 0xffffffffull)]++] = int32_t(e >> 32);
}

/// The collision answer for one particle at one substep: the closest
/// violated triangle, its normal and the closest point's barycentric
/// weights. Mirrored by the GPU shader.
struct RigExecClothContact {
    int32_t triangle = -1;
    float violation = 0.0f;
    float normal[3] = {0, 0, 0};
    float weights[3] = {0, 0, 0};
};

/// Finds the closest violated collider triangle for particle position \p p
/// at substep fraction \p t between collider points \p before and \p after.
/// A triangle is violated when its closest point lies within the reach,
/// max(\p thickness, \p depth), of \p p, and \p p is within \p thickness in
/// front of it or at most \p depth behind it. The closest one wins, so a
/// particle near one side of a thin part is never pushed through it by the
/// far side; a tie breaks to the lower triangle.
///
/// The grid only finds candidates: a triangle within the reach of \p p at
/// any substep of the tick lies in the swept box the grid padded by at
/// least the reach, so it is listed in the bucket of \p p's own cell, the
/// one bucket read. A bucket lists a triangle once; other triangles a hash
/// collision lists there are out of reach, so the answer does not depend on
/// the grid.
inline RigExecClothContact
RigExecClothFindContact(const RigExecClothColliders &colliders,
                        const RigExecClothGrid &grid, const float *before,
                        const float *after, float t, const float *p,
                        float thickness, float depth)
{
    using namespace cloth_detail;
    RigExecClothContact best;
    if (grid.bucketStart.size() < 2 || colliders.triangles.empty()) return best;
    const float reach = std::max(thickness, depth);
    float bestDistance = std::numeric_limits<float>::infinity();
    const uint32_t bucket = Hash(Cell(p[0], grid.cellSize), Cell(p[1], grid.cellSize),
                                 Cell(p[2], grid.cellSize), grid.bucketMask);
    {
        for (uint32_t k = grid.bucketStart[bucket]; k < grid.bucketStart[bucket + 1]; ++k) {
            const int32_t tri = grid.bucketTriangles[k];
            float v[3][3];
            for (int c = 0; c < 3; ++c) {
                const size_t i = size_t(colliders.triangles[3 * size_t(tri) + size_t(c)]);
                Lerp(before + 3 * i, after + 3 * i, t, v[c]);
            }
            float e1[3], e2[3], n[3];
            Sub(v[1], v[0], e1); Sub(v[2], v[0], e2); Cross(e1, e2, n);
            const float len = Length(n);
            if (!(len > 0.0f)) continue;
            n[0] /= len; n[1] /= len; n[2] /= len;
            float w[3];
            ClosestBarycentric(p, v[0], v[1], v[2], w);
            float q[3];
            for (int a = 0; a < 3; ++a)
                q[a] = w[0] * v[0][a] + w[1] * v[1][a] + w[2] * v[2][a];
            float d[3];
            Sub(p, q, d);
            const float signedDistance = Dot(d, n);
            const float distance = Length(d);
            if (distance > reach) continue;
            if (signedDistance >= thickness || signedDistance < -depth) continue;
            if (signedDistance >= 0.0f && distance >= thickness) continue;
            if (distance < bestDistance ||
                (distance == bestDistance && tri < best.triangle)) {
                bestDistance = distance;
                best.triangle = tri;
                best.violation = thickness - signedDistance;
                best.normal[0] = n[0]; best.normal[1] = n[1]; best.normal[2] = n[2];
                best.weights[0] = w[0]; best.weights[1] = w[1]; best.weights[2] = w[2];
            }
        }
    }
    return best;
}

/// One substep's tick-level constants, shared by the passes below and the
/// GPU shader's uniforms.
struct RigExecClothStep {
    float h = 0.0f;
    float t = 0.0f;
    float t0 = 0.0f;
    float alphaStretch = 0.0f;
    float alphaBend = 0.0f;
    float goalExponent = 1.0f;
    float damp = 1.0f;
    float attract = 0.0f;
    float thickness = 0.0f;
    float depth = 0.0f;
    float friction = 0.0f;
    float gravity[3] = {0, 0, 0};
};

/// Predict pass for particle \p i: gravity and inertia for a free particle;
/// a kinematic one is placed on its entering points at the substep.
inline void
RigExecClothPredict(const RigExecClothMesh &mesh, const RigExecClothStep &s,
                    const float *from, const float *to, size_t i, float *x,
                    float *previous, float *velocity)
{
    float *pi = x + 3 * i;
    float *qi = previous + 3 * i;
    qi[0] = pi[0]; qi[1] = pi[1]; qi[2] = pi[2];
    if (mesh.inverseMass[i] == 0.0f) {
        cloth_detail::Lerp(from + 3 * i, to + 3 * i, s.t, pi);
        return;
    }
    float *vi = velocity + 3 * i;
    for (int a = 0; a < 3; ++a) {
        vi[a] += s.gravity[a] * s.h;
        pi[a] += vi[a] * s.h;
    }
}

/// Projects constraint \p c (XPBD, in place). Within a color no two
/// constraints share a particle.
inline void
RigExecClothProjectConstraint(const RigExecClothMesh &mesh,
                              const RigExecClothStep &s, size_t c, float *x,
                              float *lambda)
{
    const size_t a = size_t(mesh.constraintEnds[2 * c]);
    const size_t b = size_t(mesh.constraintEnds[2 * c + 1]);
    const float wa = mesh.inverseMass[a], wb = mesh.inverseMass[b];
    const float alpha = mesh.constraintKind[c] == uint8_t(RigExecClothConstraint::Stretch)
                            ? s.alphaStretch : s.alphaBend;
    float d[3];
    cloth_detail::Sub(x + 3 * a, x + 3 * b, d);
    const float len = cloth_detail::Length(d);
    const float denom = wa + wb + alpha;
    if (!(len > 0.0f) || !(denom > 0.0f)) return;
    const float C = len - mesh.constraintRest[c];
    const float dl = (-C - alpha * lambda[c]) / denom;
    lambda[c] += dl;
    const float ka = wa * dl / len, kb = wb * dl / len;
    for (int q = 0; q < 3; ++q) {
        x[3 * a + size_t(q)] += d[q] * ka;
        x[3 * b + size_t(q)] -= d[q] * kb;
    }
}

/// The per-particle pass after the constraints of one iteration: tether,
/// goal (first iteration only), then collision with friction.
inline void
RigExecClothParticlePass(const RigExecClothMesh &mesh,
                         const RigExecClothColliders &colliders,
                         const RigExecClothGrid &grid,
                         const RigExecClothSettings &settings,
                         const RigExecClothStep &s, const float *from,
                         const float *to, const float *colliderFrom,
                         const float *colliderTo, bool firstIteration,
                         size_t i, float *x, const float *previous)
{
    using namespace cloth_detail;
    if (mesh.inverseMass[i] == 0.0f) return;
    float *pi = x + 3 * i;
    const int32_t anchor = mesh.tetherAnchor[i];
    if (anchor >= 0) {
        float d[3];
        Sub(pi, x + 3 * size_t(anchor), d);
        const float len = Length(d);
        const float limit = mesh.tetherRest[i] * (1.0f + settings.tetherSlack);
        if (len > limit && len > 0.0f) {
            const float k = (len - limit) / len;
            for (int a = 0; a < 3; ++a) pi[a] -= d[a] * k;
        }
    }
    if (firstIteration) {
        const float goal = 1.0f - (1.0f - s.attract) * (1.0f - mesh.pin[i]);
        if (goal > 0.0f) {
            const float g = 1.0f - std::pow(1.0f - goal, s.goalExponent);
            float target[3];
            Lerp(from + 3 * i, to + 3 * i, s.t, target);
            for (int a = 0; a < 3; ++a) pi[a] += (target[a] - pi[a]) * g;
        }
    }
    if (!colliderFrom || colliders.triangles.empty()) return;
    const RigExecClothContact contact = RigExecClothFindContact(
        colliders, grid, colliderFrom, colliderTo, s.t, pi, s.thickness,
        s.depth);
    if (contact.triangle < 0) return;
    for (int a = 0; a < 3; ++a) pi[a] += contact.normal[a] * contact.violation;
    if (!(s.friction > 0.0f)) return;
    // Friction against the contact point's own motion over the substep.
    float now[3] = {0, 0, 0}, then[3] = {0, 0, 0};
    for (int c = 0; c < 3; ++c) {
        const size_t v = size_t(colliders.triangles[3 * size_t(contact.triangle) + size_t(c)]);
        float a1[3], a0[3];
        Lerp(colliderFrom + 3 * v, colliderTo + 3 * v, s.t, a1);
        Lerp(colliderFrom + 3 * v, colliderTo + 3 * v, s.t0, a0);
        for (int a = 0; a < 3; ++a) {
            now[a] += contact.weights[c] * a1[a];
            then[a] += contact.weights[c] * a0[a];
        }
    }
    float rel[3];
    for (int a = 0; a < 3; ++a) rel[a] = (pi[a] - previous[3 * i + size_t(a)]) - (now[a] - then[a]);
    const float along = Dot(rel, contact.normal);
    float tangent[3];
    for (int a = 0; a < 3; ++a) tangent[a] = rel[a] - contact.normal[a] * along;
    const float slide = Length(tangent);
    if (slide > 0.0f) {
        const float k = std::min(1.0f, s.friction * contact.violation / slide);
        for (int a = 0; a < 3; ++a) pi[a] -= tangent[a] * k;
    }
}

/// Velocity pass for particle \p i: the substep's displacement, damped.
inline void
RigExecClothVelocity(const RigExecClothMesh &mesh, const RigExecClothStep &s,
                     size_t i, const float *x, const float *previous,
                     float *velocity)
{
    if (mesh.inverseMass[i] == 0.0f) return;
    for (int a = 0; a < 3; ++a) {
        const size_t k = 3 * i + size_t(a);
        velocity[k] = (x[k] - previous[k]) / s.h * s.damp;
    }
}

/// The substep constants of tick \p to, substep \p substep (1-based).
inline RigExecClothStep
RigExecClothStepFor(const RigExecClothParameters &P, int substeps, int substep,
                    float frameSeconds)
{
    RigExecClothStep s;
    s.h = frameSeconds / float(substeps);
    s.t = float(substep) / float(substeps);
    s.t0 = float(substep - 1) / float(substeps);
    s.alphaStretch = std::max(0.0f, P.stretchCompliance) / (s.h * s.h);
    s.alphaBend = std::max(0.0f, P.bendCompliance) / (s.h * s.h);
    s.goalExponent = 1.0f / float(substeps);
    s.damp = std::max(0.0f, 1.0f - std::max(0.0f, P.damping) * s.h);
    s.attract = std::min(1.0f, std::max(0.0f, P.attract));
    s.thickness = std::max(0.0f, P.thickness);
    s.depth = std::max(0.0f, P.collisionDepth);
    s.friction = std::max(0.0f, P.friction);
    for (int a = 0; a < 3; ++a) s.gravity[a] = P.gravity[a];
    return s;
}

/// Checks a window before it is solved: every tick has entering points
/// (and collider points when there are colliders) and finite parameters,
/// and the current tick's entering points are finite.
inline bool
RigExecClothWindowUsable(const RigExecClothMesh &mesh,
                         const RigExecClothColliders &colliders,
                         const RigExecClothTick *ticks, size_t tickCount,
                         std::string *why)
{
    const auto fail = [why](const char *text) {
        if (why && text) *why = text;
        return false;
    };
    if (!mesh.valid || !tickCount) return fail("invalid cloth");
    for (size_t k = 0; k < tickCount; ++k) {
        if (!ticks[k].entering) return fail("a tick has no entering points");
        if (!ticks[k].parameters.Finite()) return fail("a cloth input is not finite");
        if (colliders.pointCount && !ticks[k].colliders)
            return fail("a tick has no collider points");
    }
    for (size_t i = 0; i < 3 * mesh.pointCount; ++i)
        if (!std::isfinite(ticks[tickCount - 1].entering[i]))
            return fail("an entering point is not finite");
    return true;
}

/// Solves the window \p ticks (oldest first, the last the current frame)
/// into \p out (pointCount float3s). \p frameSeconds is the time between
/// ticks. A single tick returns its entering points. False, with \p out
/// holding the current entering points, on an invalid mesh, a non-finite
/// input, or a result that is not finite.
inline bool
RigExecSolveCloth(const RigExecClothMesh &mesh,
                  const RigExecClothColliders &colliders,
                  const RigExecClothSettings &settings,
                  const RigExecClothTick *ticks, size_t tickCount,
                  double frameSeconds, RigExecClothWorkspace *ws, float *out,
                  std::string *why = nullptr)
{
    const size_t n = mesh.pointCount;
    const auto passThrough = [&](const char *text) {
        if (tickCount && ticks[tickCount - 1].entering)
            std::copy(ticks[tickCount - 1].entering,
                      ticks[tickCount - 1].entering + 3 * n, out);
        if (why && text) *why = text;
        return false;
    };
    if (!ws || !out) return passThrough("invalid cloth");
    std::string reason;
    if (!RigExecClothWindowUsable(mesh, colliders, ticks, tickCount, &reason))
        return passThrough(reason.c_str());
    if (tickCount == 1) {
        std::copy(ticks[0].entering, ticks[0].entering + 3 * n, out);
        return true;
    }
    const int substeps = std::max(1, std::min(settings.substeps, RigExecClothMaxSubsteps));
    const int iterations = std::max(1, std::min(settings.iterations, RigExecClothMaxIterations));
    const float frame = float(frameSeconds);
    ws->x.assign(ticks[0].entering, ticks[0].entering + 3 * n);
    ws->previous.resize(3 * n);
    ws->velocity.resize(3 * n);
    // The cloth starts moving with its entering points.
    for (size_t i = 0; i < 3 * n; ++i)
        ws->velocity[i] = (ticks[1].entering[i] - ticks[0].entering[i]) / frame;
    ws->lambda.assign(mesh.constraintRest.size(), 0.0f);
    float *x = ws->x.data();
    float *xp = ws->previous.data();
    float *v = ws->velocity.data();
    const size_t colors = mesh.ColorCount();
    for (size_t k = 1; k < tickCount; ++k) {
        const RigExecClothTick &from = ticks[k - 1];
        const RigExecClothTick &to = ticks[k];
        const bool collide = colliders.pointCount && !colliders.triangles.empty();
        if (collide) {
            RigExecBuildClothGrid(colliders, from.colliders, to.colliders,
                                  std::max(std::max(0.0f, to.parameters.thickness),
                                           std::max(0.0f, to.parameters.collisionDepth)),
                                  &ws->grid, &ws->cellScratch);
        }
        for (int sub = 1; sub <= substeps; ++sub) {
            const RigExecClothStep s =
                RigExecClothStepFor(to.parameters, substeps, sub, frame);
            for (size_t i = 0; i < n; ++i)
                RigExecClothPredict(mesh, s, from.entering, to.entering, i, x, xp, v);
            std::fill(ws->lambda.begin(), ws->lambda.end(), 0.0f);
            for (int it = 0; it < iterations; ++it) {
                for (size_t color = 0; color < colors; ++color) {
                    for (uint32_t c = mesh.colorStart[color]; c < mesh.colorStart[color + 1]; ++c)
                        RigExecClothProjectConstraint(mesh, s, c, x, ws->lambda.data());
                }
                for (size_t i = 0; i < n; ++i) {
                    RigExecClothParticlePass(mesh, colliders, ws->grid, settings, s,
                                             from.entering, to.entering,
                                             collide ? from.colliders : nullptr,
                                             collide ? to.colliders : nullptr,
                                             it == 0, i, x, xp);
                }
            }
            for (size_t i = 0; i < n; ++i) RigExecClothVelocity(mesh, s, i, x, xp, v);
        }
    }
    for (size_t i = 0; i < 3 * n; ++i) {
        if (!std::isfinite(x[i])) return passThrough("the cloth solve diverged");
    }
    std::copy(x, x + 3 * n, out);
    return true;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_CLOTH_KERNEL_H
