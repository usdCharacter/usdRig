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
//   and Friction for Cloth Animation (SIGGRAPH 2002): collision thickness,
//   the positional friction rule, and vertex/triangle and edge/edge
//   self-contact.
// - Baraff and Witkin, Large Steps in Cloth Simulation (SIGGRAPH 1998):
//   warp and weft axes from a triangle's UV coordinates.
// - Ericson, Real-Time Collision Detection (2005), 5.1.5, 5.1.9 and 5.3.6:
//   closest triangle points, closest segments and segment/triangle crossing.
//
// RigExecClothMover's solver. Header-only and USD-free, so the evaluators
// and the zero-USD runtime share the solve. The GPU shader implements the
// legacy material passes; relationship-based profiles use this CPU kernel.
//
// The solve is a pure function of its window: the cloth starts at the oldest
// tick on its entering points, moving with them, and is simulated tick by
// tick to the newest, which is the current frame. Nothing survives between
// solves. The window starts windowFrames back, or earlier where a collider
// cuts through the entering cloth there (RigExecClothWindowStart).
//
// Determinism: a constraint color is a set of constraints that share no
// particle, so its members may run in any order or side by side and write
// the same bits. Every other pass reads and writes one particle. The CPU loop
// and the GPU shader therefore agree for the same arithmetic.
#ifndef RIGEXEC_MATH_CLOTH_KERNEL_H
#define RIGEXEC_MATH_CLOTH_KERNEL_H

#include "clothModel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
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

/// Compiled rig settings: they fix the window and the constraint set, so an edit
/// recompiles.
struct RigExecClothSettings {
    /// Frames simulated before the current one.
    int windowFrames = 16;
    /// The most frames a window may reach back when its first frame has the
    /// cloth cutting through a collider (RigExecClothWindowStart). At most
    /// windowFrames turns reaching back off.
    int maxWindowFrames = 64;
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
        return windowFrames == o.windowFrames &&
               maxWindowFrames == o.maxWindowFrames && substeps == o.substeps &&
               iterations == o.iterations && tethers == o.tethers &&
               tetherSlack == o.tetherSlack && pinThreshold == o.pinThreshold;
    }
    bool operator!=(const RigExecClothSettings &o) const { return !(*this == o); }
};

/// The frames of history a cloth's solve may read: windowFrames, or
/// maxWindowFrames when that is longer, within [1, RigExecClothMaxWindowFrames].
inline int
RigExecClothHistoryFrames(const RigExecClothSettings &s)
{
    return std::max(1, std::min(std::max(s.windowFrames, s.maxWindowFrames),
                                RigExecClothMaxWindowFrames));
}

/// Per-tick values. Read at every tick of the window, so an animated value
/// is simulated as animated.
struct RigExecClothParameters {
    /// XPBD compliance (inverse stiffness) of the edges and of the bend
    /// struts across shared edges; 0 is rigid.
    float stretchCompliance = 0.0f;
    float bendCompliance = 1.0e-3f;
    /// Scene units per second squared.
    float gravity[3] = {0.0f, -981.0f, 0.0f};
    /// How fast velocity decays: by a factor of e^-damping per second. A
    /// window forgets motion older than itself, so swings should settle
    /// within it.
    float damping = 2.0f;
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

/// What a cloth mesh fixes per compiled rig: triangles, colored constraints and
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
    RigExecClothModel model;
    std::vector<float> restPoints, materialFrames;
    std::vector<std::vector<int32_t>> incidentTriangles, neighbors;
    std::vector<size_t> colliderOffsets;
    std::vector<std::vector<float>> attachmentRest;
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
    std::vector<float> attachmentLambda;
    std::vector<float> contactLambda;
    std::vector<int32_t> contactTriangles;
    std::map<std::array<int32_t, 5>, float> selfLambda;
    RigExecClothGrid selfGrid;
    std::vector<uint64_t> selfScratch;
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
inline void ClosestSegments(const float *p, const float *q, const float *r,
                             const float *s, float *u, float *v)
{
    float a[3], b[3], d[3];
    Sub(q, p, a); Sub(s, r, b); Sub(p, r, d);
    const float aa = Dot(a, a), bb = Dot(b, b), ab = Dot(a, b);
    const float ad = Dot(a, d), bd = Dot(b, d);
    const auto clamp = [](float x) { return std::max(0.0f, std::min(1.0f, x)); };
    if (!(aa > 1e-12f)) { *u = 0; *v = bb > 1e-12f ? clamp(bd / bb) : 0; return; }
    if (!(bb > 1e-12f)) { *v = 0; *u = clamp(-ad / aa); return; }
    const float denom = aa * bb - ab * ab;
    *u = denom > 1e-12f ? clamp((ab * bd - ad * bb) / denom) : 0;
    *v = (ab * *u + bd) / bb;
    if (*v < 0) { *v = 0; *u = clamp(-ad / aa); }
    else if (*v > 1) { *v = 1; *u = clamp((ab - ad) / aa); }
}
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

/// Builds the cloth's compiled rig half. \p restPoints holds pointCount float3s;
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

inline RigExecClothSample RigExecClothVertex(size_t i)
{
    RigExecClothSample s;
    s.vertices.fill(int32_t(i));
    return s;
}

inline void RigExecClothSamplePoint(const RigExecClothSample &sample,
                                    const float *points, float *out)
{
    for (size_t a = 0; a < 3; ++a) {
        out[a] = 0;
        for (size_t k = 0; k < 3; ++k)
            out[a] += sample.weights[k] * points[3 * size_t(sample.vertices[k]) + a];
    }
}

inline float RigExecClothRegion(const std::vector<float> &mask,
                                const RigExecClothSample &sample)
{
    if (mask.empty()) return 1;
    float weight = 0;
    for (size_t k = 0; k < 3; ++k)
        weight += sample.weights[k] * mask[size_t(sample.vertices[k])];
    return weight;
}

inline const RigExecClothProfile *RigExecClothProfileFor(
    const RigExecClothMesh &mesh, RigExecClothProfileKind kind,
    const RigExecClothSample &sample)
{
    for (size_t k = mesh.model.profiles.size(); k-- > 0;) {
        const auto &p = mesh.model.profiles[k];
        if (p.kind == kind && RigExecClothRegion(p.sourceMask, sample) > 0) return &p;
    }
    return nullptr;
}

/// Material axes follow the current triangle tangents. Face-varying UVs
/// remain per corner; a mirrored island does not change squared strengths.
inline void RigExecClothMaterialFrame(const RigExecClothMesh &mesh,
                                      const float *points, size_t vertex,
                                      float *frame)
{
    using namespace cloth_detail;
    std::fill(frame, frame + 9, 0.0f);
    if (mesh.model.triangleUV.empty()) {
        frame[0] = frame[4] = frame[8] = 1;
        return;
    }
    for (int32_t tri : mesh.incidentTriangles[vertex]) {
        const size_t t = size_t(tri);
        const float *a = points + 3 * size_t(mesh.triangles[3 * t]);
        float e1[3], e2[3], n[3];
        Sub(points + 3 * size_t(mesh.triangles[3 * t + 1]), a, e1);
        Sub(points + 3 * size_t(mesh.triangles[3 * t + 2]), a, e2);
        Cross(e1, e2, n);
        const float *uv = mesh.model.triangleUV.data() + 6 * t;
        const float du1 = uv[2] - uv[0], dv1 = uv[3] - uv[1];
        const float du2 = uv[4] - uv[0], dv2 = uv[5] - uv[1];
        const float det = du1 * dv2 - dv1 * du2;
        if (std::abs(det) <= 1e-12f) continue;
        for (size_t q = 0; q < 3; ++q) {
            frame[q] += (e1[q] * dv2 - e2[q] * dv1) / det;
            frame[6 + q] += n[q];
        }
    }
    const float nl = Length(frame + 6);
    if (!(nl > 1e-12f)) { frame[0] = frame[4] = frame[8] = 1; return; }
    for (size_t q = 0; q < 3; ++q) frame[6 + q] /= nl;
    const float normalPart = Dot(frame, frame + 6);
    for (size_t q = 0; q < 3; ++q) frame[q] -= normalPart * frame[6 + q];
    float wl = Length(frame);
    if (!(wl > 1e-12f)) {
        float axis[3] = {1, 0, 0};
        if (std::abs(frame[6]) > 0.9f) { axis[0] = 0; axis[1] = 1; }
        Cross(axis, frame + 6, frame);
        wl = Length(frame);
    }
    for (size_t q = 0; q < 3; ++q) frame[q] /= wl;
    Cross(frame + 6, frame, frame + 3);
}

inline bool RigExecPrepareClothModel(RigExecClothMesh *mesh,
                                     const RigExecClothModel &model,
                                     const float *rest,
                                     const std::vector<size_t> &colliderCounts,
                                     std::string *why)
{
    if (!RigExecClothValidateModel(model, mesh->pointCount,
                                   mesh->triangles.size() / 3, colliderCounts, why)) return false;
    mesh->model = model;
    if (model.Empty()) return true;
    mesh->restPoints.assign(rest, rest + 3 * mesh->pointCount);
    mesh->incidentTriangles.assign(mesh->pointCount, {});
    mesh->neighbors.assign(mesh->pointCount, {});
    for (size_t t = 0; t < mesh->triangles.size() / 3; ++t) {
        for (size_t k = 0; k < 3; ++k) {
            const size_t i = size_t(mesh->triangles[3 * t + k]);
            mesh->incidentTriangles[i].push_back(int32_t(t));
            for (size_t j = 0; j < 3; ++j)
                mesh->neighbors[i].push_back(mesh->triangles[3 * t + j]);
        }
    }
    for (auto &neighbors : mesh->neighbors) {
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    }
    mesh->materialFrames.resize(9 * mesh->pointCount);
    for (size_t i = 0; i < mesh->pointCount; ++i)
        RigExecClothMaterialFrame(*mesh, rest, i, mesh->materialFrames.data() + 9 * i);
    mesh->colliderOffsets.assign(1, 0);
    for (size_t n : colliderCounts) mesh->colliderOffsets.push_back(mesh->colliderOffsets.back() + n);
    mesh->attachmentRest.assign(model.profiles.size(), {});
    for (size_t k = 0; k < model.profiles.size(); ++k) {
        const auto &p = model.profiles[k];
        if (p.kind != RigExecClothProfileKind::Attachment &&
            !(p.kind == RigExecClothProfileKind::Tether && !p.target.empty())) continue;
        if (p.targetCollider >= 0 && p.restLength < 0) {
            if (why) *why = "external cloth attachments need an explicit rest length";
            return false;
        }
        for (size_t j = 0; j < p.source.size(); ++j) {
            float length = p.restLength;
            if (length < 0) {
                float a[3], b[3], d[3];
                RigExecClothSamplePoint(p.source[j], rest, a);
                RigExecClothSamplePoint(p.target[j], rest, b);
                cloth_detail::Sub(a, b, d);
                length = cloth_detail::Length(d);
            }
            mesh->attachmentRest[k].push_back(length);
        }
    }
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
                        float thickness, float depth, const float *previous = nullptr,
                        float t0 = 0, size_t twoSidedBegin = 0,
                        size_t twoSidedEnd = std::numeric_limits<size_t>::max(),
                        bool filterCollider = false,
                        const std::vector<float> *region = nullptr)
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
            const size_t firstVertex = size_t(colliders.triangles[3 * size_t(tri)]);
            if (filterCollider && (firstVertex < twoSidedBegin || firstVertex >= twoSidedEnd)) continue;
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
            if (region && !region->empty()) {
                float selected = 0;
                for (size_t c = 0; c < 3; ++c)
                    selected += w[c] * (*region)[size_t(colliders.triangles[3 * size_t(tri) + c]) - twoSidedBegin];
                if (!(selected > 0)) continue;
            }
            float q[3];
            for (int a = 0; a < 3; ++a)
                q[a] = w[0] * v[0][a] + w[1] * v[1][a] + w[2] * v[2][a];
            if (previous && firstVertex >= twoSidedBegin && firstVertex < twoSidedEnd) {
                float oldQ[3] = {0, 0, 0};
                for (size_t c = 0; c < 3; ++c) {
                    const size_t vertex = size_t(colliders.triangles[3 * size_t(tri) + c]);
                    float old[3];
                    Lerp(before + 3 * vertex, after + 3 * vertex, t0, old);
                    for (size_t a = 0; a < 3; ++a) oldQ[a] += w[c] * old[a];
                }
                float side[3]; Sub(previous, oldQ, side);
                if (Dot(side, n) < 0) for (float &value : n) value = -value;
            }
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
    float alpha = mesh.constraintKind[c] == uint8_t(RigExecClothConstraint::Stretch)
                            ? s.alphaStretch : s.alphaBend;
    float d[3];
    cloth_detail::Sub(x + 3 * a, x + 3 * b, d);
    const float len = cloth_detail::Length(d);
    if (!(len > 0.0f)) return;
    const float C = len - mesh.constraintRest[c];
    if (!mesh.model.Empty()) {
        RigExecClothSample sample;
        sample.vertices = {{int32_t(a), int32_t(b), int32_t(b)}};
        sample.weights = {{0.5f, 0.5f, 0}};
        const auto kind = mesh.constraintKind[c] == uint8_t(RigExecClothConstraint::Stretch)
            ? RigExecClothProfileKind::Stretch : RigExecClothProfileKind::Bend;
        if (const auto *profile = RigExecClothProfileFor(mesh, kind, sample)) {
            float restDirection[3];
            cloth_detail::Sub(mesh.restPoints.data() + 3 * a,
                              mesh.restPoints.data() + 3 * b, restDirection);
            const float rl = cloth_detail::Length(restDirection);
            for (float &q : restDirection) q /= rl;
            const float force = wa + wb > 0 ? std::abs(lambda[c] - C / (wa + wb)) / (s.h * s.h) : 0;
            const float strength = RigExecClothStrength(*profile, sample, force,
                restDirection, mesh.materialFrames.data() + 9 * a);
            if (!(strength > 0)) { lambda[c] = 0; return; }
            alpha = profile->hard ? 0 : profile->compliance / (strength * s.h * s.h);
        }
    }
    const float denom = wa + wb + alpha;
    if (!(denom > 0.0f)) return;
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
                         size_t i, float *x, const float *previous,
                         float *contactLambda = nullptr, int32_t *contactTriangles = nullptr)
{
    using namespace cloth_detail;
    if (mesh.inverseMass[i] == 0.0f) return;
    float *pi = x + 3 * i;
    const int32_t anchor = mesh.tetherAnchor[i];
    if (anchor >= 0 && !RigExecClothProfileFor(mesh, RigExecClothProfileKind::Tether,
                                              RigExecClothVertex(i))) {
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
    const auto sample = RigExecClothVertex(i);
    const auto solveContact = [&](int collider) {
    const RigExecClothProfile *collisionProfile = nullptr;
    for (size_t k = mesh.model.profiles.size(); k-- > 0;) {
        const auto &profile = mesh.model.profiles[k];
        if (profile.kind == RigExecClothProfileKind::Collision &&
            (profile.targetCollider < 0 || profile.targetCollider == collider) &&
            RigExecClothRegion(profile.sourceMask, sample) > 0) {
            collisionProfile = &profile;
            break;
        }
    }
    const float thickness = collisionProfile
        ? collisionProfile->thickness * RigExecClothMap(*collisionProfile, ClothThickness, i)
        : s.thickness;
    const int targetCollider = collider;
    const size_t twoSidedBegin = targetCollider >= 0 ? mesh.colliderOffsets[size_t(targetCollider)] : 0;
    const size_t twoSidedEnd = targetCollider >= 0 ? mesh.colliderOffsets[size_t(targetCollider) + 1] : colliders.pointCount;
    const RigExecClothContact contact = RigExecClothFindContact(
        colliders, grid, colliderFrom, colliderTo, s.t, pi, thickness,
        s.depth, collisionProfile && collisionProfile->twoSided ? previous + 3 * i : nullptr,
        s.t0, twoSidedBegin, twoSidedEnd, collider >= 0,
        collisionProfile && collisionProfile->targetCollider >= 0 ? &collisionProfile->targetMask : nullptr);
    if (contact.triangle < 0) return;
    float correction = contact.violation;
    float friction = s.friction;
    if (const auto *profile = collisionProfile) {
        const size_t hit = size_t(colliders.triangles[3 * size_t(contact.triangle)]);
        const bool applies = profile->targetCollider < 0 ||
            (hit >= mesh.colliderOffsets[size_t(profile->targetCollider)] &&
             hit < mesh.colliderOffsets[size_t(profile->targetCollider) + 1]);
        if (applies) {
            const float wi = mesh.inverseMass[i];
            float frame[9];
            RigExecClothMaterialFrame(mesh, x, i, frame);
            float lambda = contactLambda && contactTriangles && contactTriangles[i] == contact.triangle
                ? contactLambda[i] : 0;
            const float force = (lambda + correction / wi) / (s.h * s.h);
            const float strength = RigExecClothStrength(*profile, sample, force, contact.normal, frame);
            if (!(strength > 0)) return;
            const float alpha = profile->hard ? 0 : profile->compliance / (strength * s.h * s.h);
            const float dl = std::max(-lambda, (correction - alpha * lambda) / (wi + alpha));
            correction = wi * dl;
            if (contactLambda && contactTriangles) {
                contactLambda[i] = lambda + dl;
                contactTriangles[i] = contact.triangle;
            }
            friction = profile->friction * RigExecClothMap(*profile, ClothFriction, i);
        }
    }
    for (int a = 0; a < 3; ++a) pi[a] += contact.normal[a] * correction;
    if (!(friction > 0.0f)) return;
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
        const float k = std::min(1.0f, friction * std::max(0.0f, correction) / slide);
        for (int a = 0; a < 3; ++a) pi[a] -= tangent[a] * k;
    }
    };
    if (mesh.model.Empty()) solveContact(-1);
    else for (size_t c = 0; c + 1 < mesh.colliderOffsets.size(); ++c) solveContact(int(c));
}

/// Velocity pass for particle \p i: the substep's displacement, damped.
inline void
RigExecClothVelocity(const RigExecClothMesh &mesh, const RigExecClothStep &s,
                     size_t i, const float *x, const float *previous,
                     float *velocity)
{
    if (mesh.inverseMass[i] == 0.0f) return;
    if (const auto *profile = RigExecClothProfileFor(mesh, RigExecClothProfileKind::Damping,
                                                   RigExecClothVertex(i))) {
        float frame[9], v[3], delta[3];
        RigExecClothMaterialFrame(mesh, x, i, frame);
        for (size_t a = 0; a < 3; ++a) {
            v[a] = (x[3 * i + a] - previous[3 * i + a]) / s.h;
            delta[a] = (v[a] - velocity[3 * i + a]) / (s.h * mesh.inverseMass[i]);
        }
        const float force = cloth_detail::Length(delta);
        std::fill(velocity + 3 * i, velocity + 3 * i + 3, 0.0f);
        for (size_t a = 0; a < 3; ++a) {
            const float rate = RigExecClothStrength(*profile, RigExecClothVertex(i), force,
                                                   frame + 3 * a, frame);
            const float damp = profile->hard && rate > 0 ? 0 : std::exp(-rate * s.h);
            const float along = cloth_detail::Dot(v, frame + 3 * a) * damp;
            for (size_t q = 0; q < 3; ++q) velocity[3 * i + q] += along * frame[3 * a + q];
        }
        return;
    }
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

/// Generalized XPBD projection. Repeated vertices are merged before the
/// effective mass is formed, so overlapping barycentric bindings conserve
/// momentum and never count the same degree of freedom twice.
inline float RigExecClothProjectSamples(const RigExecClothMesh &mesh,
    const RigExecClothStep &step, const RigExecClothProfile &profile,
    const RigExecClothSample &a, const RigExecClothSample *b,
    float C, const float *normal, bool contact, float *lambda, float *x,
    bool selectRegion = true)
{
    if (selectRegion && !(RigExecClothRegion(profile.sourceMask, a) > 0)) return 0;
    std::array<int32_t, 6> indices{};
    std::array<float, 6> gradients{};
    size_t count = 0;
    const auto add = [&](int32_t index, float weight) {
        for (size_t k = 0; k < count; ++k)
            if (indices[k] == index) { gradients[k] += weight; return; }
        indices[count] = index;
        gradients[count++] = weight;
    };
    for (size_t k = 0; k < 3; ++k) {
        add(a.vertices[k], a.weights[k]);
        if (b) add(b->vertices[k], -b->weights[k]);
    }
    float mass = 0;
    for (size_t k = 0; k < count; ++k)
        mass += mesh.inverseMass[size_t(indices[k])] * gradients[k] * gradients[k];
    if (!(mass > 0)) return 0;
    float frame[9];
    RigExecClothMaterialFrame(mesh, x, size_t(a.vertices[0]), frame);
    const float force = std::abs(*lambda - C / mass) / (step.h * step.h);
    const float strength = RigExecClothStrength(profile, a, force, normal, frame);
    if (!(strength > 0)) { *lambda = 0; return 0; }
    const float alpha = profile.hard ? 0 : profile.compliance / (strength * step.h * step.h);
    float dl = (-C - alpha * *lambda) / (mass + alpha);
    if (contact) dl = std::max(-*lambda, dl);
    *lambda += dl;
    for (size_t k = 0; k < count; ++k) {
        const size_t i = size_t(indices[k]);
        const float scale = mesh.inverseMass[i] * gradients[k] * dl;
        for (size_t q = 0; q < 3; ++q) x[3 * i + q] += normal[q] * scale;
    }
    return mass * dl;
}

inline void RigExecClothBindings(const RigExecClothMesh &mesh,
    const RigExecClothStep &step, const RigExecClothSettings &settings,
    const RigExecClothTick &from, const RigExecClothTick &to,
    float *x, float *lambda)
{
    using namespace cloth_detail;
    size_t offset = 0;
    for (size_t k = 0; k < mesh.model.profiles.size(); ++k) {
        const auto &p = mesh.model.profiles[k];
        const size_t count = p.source.empty() ? mesh.pointCount : p.source.size();
        for (size_t j = 0; j < count; ++j, ++offset) {
            if (p.kind != RigExecClothProfileKind::Pin &&
                p.kind != RigExecClothProfileKind::Attachment &&
                p.kind != RigExecClothProfileKind::Tether) continue;
            const auto a = p.source.empty() ? RigExecClothVertex(j) : p.source[j];
            RigExecClothSample b;
            bool internal = false, tether = p.kind == RigExecClothProfileKind::Tether;
            float target[3], point[3], d[3], length = 0;
            if (p.kind == RigExecClothProfileKind::Pin) {
                float before[3], after[3];
                RigExecClothSamplePoint(a, from.entering, before);
                RigExecClothSamplePoint(a, to.entering, after);
                Lerp(before, after, step.t, target);
            } else if (tether && p.target.empty()) {
                // Tethers bind vertices; explicit barycentric tethers use
                // an attachment profile with an authored rest length.
                const size_t vertex = size_t(a.vertices[0]);
                const int32_t anchor = mesh.tetherAnchor[vertex];
                if (anchor < 0) continue;
                b = RigExecClothVertex(size_t(anchor));
                RigExecClothSamplePoint(b, x, target);
                internal = true;
                length = mesh.tetherRest[vertex] * (1 + settings.tetherSlack);
            } else {
                b = p.target[j];
                internal = p.targetCollider < 0;
                if (internal) RigExecClothSamplePoint(b, x, target);
                else {
                    const size_t first = mesh.colliderOffsets[size_t(p.targetCollider)];
                    float before[3], after[3];
                    RigExecClothSamplePoint(b, from.colliders + 3 * first, before);
                    RigExecClothSamplePoint(b, to.colliders + 3 * first, after);
                    Lerp(before, after, step.t, target);
                }
                length = mesh.attachmentRest[k][j];
            }
            RigExecClothSamplePoint(a, x, point);
            Sub(point, target, d);
            const float distance = Length(d);
            const float C = distance - length;
            if (!(distance > 1e-12f) || (tether && C <= 0)) continue;
            for (float &v : d) v /= distance;
            RigExecClothProjectSamples(mesh, step, p, a, internal ? &b : nullptr,
                                       C, d, false, lambda + offset, x);
        }
    }
}

/// Two-sided vertex/triangle contact between selected cloth regions. The
/// broad phase is rebuilt inside the existing constraint iteration; all
/// corrections act on the same private particle arena as stretch and bend.
inline void RigExecClothSelfContacts(const RigExecClothMesh &mesh,
    const RigExecClothStep &step, const float *previous,
    RigExecClothWorkspace *ws, float *x)
{
    using namespace cloth_detail;
    RigExecClothColliders topology;
    topology.pointCount = mesh.pointCount;
    topology.triangles = mesh.triangles;
    float pad = 0;
    for (const auto &p : mesh.model.profiles)
        if (p.kind == RigExecClothProfileKind::SelfCollision)
            for (size_t i = 0; i < mesh.pointCount; ++i)
                pad = std::max(pad, p.thickness * RigExecClothMap(p, ClothThickness, i));
    if (!(pad > 0)) return;
    // A padded triangle bucket at an edge midpoint covers every possible
    // edge/edge encounter along that edge, not just its endpoints.
    float edgeReach = 0;
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != uint8_t(RigExecClothConstraint::Stretch)) continue;
        float d[3];
        Sub(x + 3 * size_t(mesh.constraintEnds[2*c]), x + 3 * size_t(mesh.constraintEnds[2*c+1]), d);
        edgeReach = std::max(edgeReach, 0.5f * Length(d));
    }
    RigExecBuildClothGrid(topology, previous, x, std::max(pad, step.depth),
                          &ws->selfGrid, &ws->selfScratch);
    const auto &grid = ws->selfGrid;
    for (size_t profileIndex = 0; profileIndex < mesh.model.profiles.size(); ++profileIndex) {
        const auto &p = mesh.model.profiles[profileIndex];
        if (p.kind != RigExecClothProfileKind::SelfCollision) continue;
        for (size_t i = 0; i < mesh.pointCount; ++i) {
            const auto a = RigExecClothVertex(i);
            // Either region may supply the vertex; contact is symmetric.
            const float source = RigExecClothRegion(p.sourceMask, a);
            const float target = RigExecClothRegion(p.targetMask, a);
            if (!(source > 0 || target > 0)) continue;
            const float thickness = p.thickness * RigExecClothMap(p, ClothThickness, i);
            const float *point = x + 3 * i;
            const uint32_t bucket = Hash(Cell(point[0], grid.cellSize), Cell(point[1], grid.cellSize),
                                          Cell(point[2], grid.cellSize), grid.bucketMask);
            for (uint32_t entry = grid.bucketStart[bucket]; entry < grid.bucketStart[bucket + 1]; ++entry) {
                const int32_t tri = grid.bucketTriangles[entry];
                RigExecClothSample b;
                bool adjacent = false;
                for (size_t q = 0; q < 3; ++q) {
                    b.vertices[q] = mesh.triangles[3 * size_t(tri) + q];
                    const auto &neighbors = mesh.neighbors[i];
                    adjacent = adjacent || std::binary_search(neighbors.begin(), neighbors.end(), b.vertices[q]);
                }
                if (adjacent) continue;
                const float *v0 = x + 3 * size_t(b.vertices[0]);
                const float *v1 = x + 3 * size_t(b.vertices[1]);
                const float *v2 = x + 3 * size_t(b.vertices[2]);
                ClosestBarycentric(point, v0, v1, v2, b.weights.data());
                const float bs = RigExecClothRegion(p.sourceMask, b);
                const float bt = RigExecClothRegion(p.targetMask, b);
                if (!(source * bt > 0 || target * bs > 0)) continue;
                float q[3], d[3], e1[3], e2[3], n[3], oldQ[3], oldD[3];
                RigExecClothSamplePoint(b, x, q);
                Sub(point, q, d);
                const float distance = Length(d);
                if (distance > std::max(thickness, step.depth)) continue;
                Sub(v1, v0, e1); Sub(v2, v0, e2); Cross(e1, e2, n);
                const float nl = Length(n);
                if (!(nl > 1e-12f)) continue;
                for (float &v : n) v /= nl;
                RigExecClothSamplePoint(b, previous, oldQ);
                Sub(previous + 3 * i, oldQ, oldD);
                if (Dot(oldD, n) < 0) for (float &v : n) v = -v;
                const float signedDistance = Dot(d, n);
                if (signedDistance >= thickness) continue;
                if (signedDistance >= 0 && distance >= thickness) continue;
                float &lambda = ws->selfLambda[{{int32_t(profileIndex), int32_t(i), tri, -1, -1}}];
                // Region reversal uses the same strength maps, without
                // excluding vertices solely because they are in target.
                const float correction = RigExecClothProjectSamples(mesh, step, p,
                    a, &b, signedDistance - thickness, n, true, &lambda, x, false);
                const float mu = p.friction * RigExecClothMap(p, ClothFriction, i);
                if (!(correction > 0 && mu > 0)) continue;
                RigExecClothSamplePoint(b, x, q);
                for (size_t axis = 0; axis < 3; ++axis)
                    d[axis] = (x[3 * i + axis] - previous[3 * i + axis]) - (q[axis] - oldQ[axis]);
                const float along = Dot(d, n);
                for (size_t axis = 0; axis < 3; ++axis) d[axis] -= along * n[axis];
                const float slide = Length(d);
                if (!(slide > 1e-12f)) continue;
                for (float &v : d) v /= slide;
                // A hard positional friction constraint, limited by the
                // normal correction, distributes its impulse to both sides.
                float mass = mesh.inverseMass[i];
                for (size_t c = 0; c < 3; ++c)
                    mass += mesh.inverseMass[size_t(b.vertices[c])] * b.weights[c] * b.weights[c];
                if (!(mass > 0)) continue;
                const float impulse = std::min(slide, mu * correction) / mass;
                for (size_t axis = 0; axis < 3; ++axis) {
                    x[3 * i + axis] -= mesh.inverseMass[i] * impulse * d[axis];
                    for (size_t c = 0; c < 3; ++c) {
                        const size_t j = size_t(b.vertices[c]);
                        x[3 * j + axis] += mesh.inverseMass[j] * b.weights[c] * impulse * d[axis];
                    }
                }
            }
        }
        RigExecBuildClothGrid(topology, previous, x, std::max(std::max(pad, step.depth), edgeReach),
                              &ws->selfGrid, &ws->selfScratch);
        std::set<std::array<int32_t, 4>> pairs;
        for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
            if (mesh.constraintKind[c] != uint8_t(RigExecClothConstraint::Stretch)) continue;
            const int32_t i = mesh.constraintEnds[2*c], j = mesh.constraintEnds[2*c+1];
            float mid[3]; Lerp(x + 3*size_t(i), x + 3*size_t(j), .5f, mid);
            const uint32_t bucket = Hash(Cell(mid[0], grid.cellSize), Cell(mid[1], grid.cellSize),
                                          Cell(mid[2], grid.cellSize), grid.bucketMask);
            for (uint32_t entry = grid.bucketStart[bucket]; entry < grid.bucketStart[bucket+1]; ++entry) {
                const size_t tri = size_t(grid.bucketTriangles[entry]);
                for (size_t edge = 0; edge < 3; ++edge) {
                    const int32_t k = std::min(mesh.triangles[3*tri+edge], mesh.triangles[3*tri+(edge+1)%3]);
                    const int32_t l = std::max(mesh.triangles[3*tri+edge], mesh.triangles[3*tri+(edge+1)%3]);
                    if (i == k || i == l || j == k || j == l) continue;
                    bool adjacent = false;
                    for (int32_t a : {i,j}) for (int32_t b : {k,l}) {
                        const auto &neighbors = mesh.neighbors[size_t(a)];
                        adjacent = adjacent || std::binary_search(neighbors.begin(), neighbors.end(), b);
                    }
                    if (adjacent) continue;
                    std::array<int32_t,4> key{{i,j,k,l}};
                    if (std::make_pair(k,l) < std::make_pair(i,j)) key = {{k,l,i,j}};
                    if (!pairs.insert(key).second) continue;
                    float u, v;
                    ClosestSegments(x+3*size_t(i), x+3*size_t(j), x+3*size_t(k), x+3*size_t(l), &u, &v);
                    RigExecClothSample a, b;
                    a.vertices = {{i,j,j}}; a.weights = {{1-u,u,0}};
                    b.vertices = {{k,l,l}}; b.weights = {{1-v,v,0}};
                    if (!(RigExecClothRegion(p.sourceMask,a) * RigExecClothRegion(p.targetMask,b) > 0 ||
                          RigExecClothRegion(p.targetMask,a) * RigExecClothRegion(p.sourceMask,b) > 0)) continue;
                    float pa[3], pb[3], d[3], oldA[3], oldB[3], oldD[3];
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb); Sub(pa,pb,d);
                    const float distance = Length(d);
                    const float thickness = p.thickness * RigExecClothSampleMap(p,ClothThickness,a);
                    if (distance >= thickness) continue;
                    if (distance > 1e-10f) for (float &q : d) q /= distance;
                    else {
                        float e1[3], e2[3];
                        Sub(x+3*size_t(j),x+3*size_t(i),e1); Sub(x+3*size_t(l),x+3*size_t(k),e2);
                        Cross(e1,e2,d); const float len = Length(d);
                        if (!(len > 1e-10f)) continue;
                        for (float &q : d) q /= len;
                    }
                    RigExecClothSamplePoint(a,previous,oldA); RigExecClothSamplePoint(b,previous,oldB);
                    Sub(oldA,oldB,oldD);
                    if (Dot(oldD,d) < 0) for (float &q : d) q = -q;
                    Sub(pa,pb,mid);
                    float &lambda = ws->selfLambda[{{int32_t(profileIndex),key[0],key[1],key[2],key[3]}}];
                    const float correction = RigExecClothProjectSamples(mesh,step,p,a,&b,
                        Dot(mid,d)-thickness,d,true,&lambda,x,false);
                    const float mu = p.friction * RigExecClothSampleMap(p,ClothFriction,a);
                    if (!(correction > 0 && mu > 0)) continue;
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb);
                    float tangent[3];
                    for (size_t q = 0; q < 3; ++q) tangent[q] = (pa[q]-oldA[q])-(pb[q]-oldB[q]);
                    const float along = Dot(tangent,d);
                    for (size_t q = 0; q < 3; ++q) tangent[q] -= along*d[q];
                    const float slide = Length(tangent);
                    if (!(slide > 1e-12f)) continue;
                    float mass = 0;
                    for (size_t q = 0; q < 3; ++q)
                        mass += mesh.inverseMass[size_t(a.vertices[q])]*a.weights[q]*a.weights[q] +
                                mesh.inverseMass[size_t(b.vertices[q])]*b.weights[q]*b.weights[q];
                    if (!(mass > 0)) continue;
                    const float impulse = std::min(slide,mu*correction)/(mass*slide);
                    for (size_t q = 0; q < 3; ++q) for (size_t axis = 0; axis < 3; ++axis) {
                        x[3*size_t(a.vertices[q])+axis] -= mesh.inverseMass[size_t(a.vertices[q])]*
                            a.weights[q]*impulse*tangent[axis];
                        x[3*size_t(b.vertices[q])+axis] += mesh.inverseMass[size_t(b.vertices[q])]*
                            b.weights[q]*impulse*tangent[axis];
                    }
                }
            }
        }
    }
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

namespace cloth_detail {

/// True when segment (p, q) crosses triangle (a, b, c): its ends lie strictly
/// on opposite sides of the triangle's plane and the crossing point lies in
/// the triangle (Ericson, Real-Time Collision Detection, 5.3.6).
inline bool SegmentCrossesTriangle(const float *p, const float *q, const float *a,
                                   const float *b, const float *c)
{
    float ab[3], ac[3], n[3], ap[3], aq[3];
    Sub(b, a, ab); Sub(c, a, ac); Cross(ab, ac, n);
    Sub(p, a, ap); Sub(q, a, aq);
    const float dp = Dot(ap, n), dq = Dot(aq, n);
    if (!((dp > 0.0f && dq < 0.0f) || (dp < 0.0f && dq > 0.0f))) return false;
    const float t = dp / (dp - dq);
    float x[3];
    Lerp(p, q, t, x);
    // Inside when x is on the inner side of all three edges.
    const float *v[3] = {a, b, c};
    for (int e = 0; e < 3; ++e) {
        float edge[3], toX[3], side[3];
        Sub(v[(e + 1) % 3], v[e], edge);
        Sub(x, v[e], toX);
        Cross(edge, toX, side);
        if (Dot(side, n) < 0.0f) return false;
    }
    return true;
}

}  // namespace cloth_detail

/// True when no edge of the cloth at points \p cloth crosses a collider
/// triangle at points \p colliderPoints: the two surfaces do not cut through
/// each other. \p grid and \p scratch are reused storage.
inline bool
RigExecClothClearOfColliders(const RigExecClothMesh &mesh,
                             const RigExecClothColliders &colliders,
                             const float *cloth, const float *colliderPoints,
                             RigExecClothGrid *grid, std::vector<uint64_t> *scratch)
{
    using namespace cloth_detail;
    if (!cloth || !colliderPoints || colliders.triangles.empty()) return true;
    const auto stretch = uint8_t(RigExecClothConstraint::Stretch);
    float longest = 0.0f;
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != stretch) continue;
        float d[3];
        Sub(cloth + 3 * size_t(mesh.constraintEnds[2 * c]),
            cloth + 3 * size_t(mesh.constraintEnds[2 * c + 1]), d);
        longest = std::max(longest, Length(d));
    }
    if (!(longest > 0.0f) || !std::isfinite(longest)) return true;
    // Padded by half the longest edge, a triangle an edge crosses is listed
    // in the bucket of the edge's midpoint, the one bucket read.
    RigExecBuildClothGrid(colliders, colliderPoints, colliderPoints, 0.5f * longest,
                          grid, scratch);
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != stretch) continue;
        const float *p = cloth + 3 * size_t(mesh.constraintEnds[2 * c]);
        const float *q = cloth + 3 * size_t(mesh.constraintEnds[2 * c + 1]);
        float mid[3];
        Lerp(p, q, 0.5f, mid);
        const uint32_t bucket = Hash(Cell(mid[0], grid->cellSize), Cell(mid[1], grid->cellSize),
                                     Cell(mid[2], grid->cellSize), grid->bucketMask);
        for (uint32_t k = grid->bucketStart[bucket]; k < grid->bucketStart[bucket + 1]; ++k) {
            const size_t tri = size_t(grid->bucketTriangles[k]);
            const float *a = colliderPoints + 3 * size_t(colliders.triangles[3 * tri]);
            const float *b = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 1]);
            const float *d = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 2]);
            if (SegmentCrossesTriangle(p, q, a, b, d)) return false;
        }
    }
    return true;
}

/// The tick a solve of \p ticks (oldest first, the last the current frame)
/// starts at: windowFrames before the current one, unless the cloth cuts
/// through a collider within windowFrames ticks before or after it. A cloth
/// that starts cut by a collider starts with points inside it, which the
/// contact test, reaching only collisionDepth behind a surface, cannot put
/// back on the side they came from. The start then moves back to the latest
/// tick with windowFrames clear ticks on either side, so contact has a
/// consistent settling interval before it begins, and its motion settles
/// for a window afterwards. The forward check ends at the current tick. At
/// most maxWindowFrames back, and the oldest tick allowed when no tick
/// qualifies. Ticks before the first one count as clear.
inline size_t
RigExecClothWindowStart(const RigExecClothMesh &mesh,
                        const RigExecClothColliders &colliders,
                        const RigExecClothSettings &settings,
                        const RigExecClothTick *ticks, size_t tickCount)
{
    if (tickCount < 2) return 0;
    const size_t last = tickCount - 1;
    const size_t nominal = size_t(std::max(1, settings.windowFrames));
    const size_t reach = size_t(RigExecClothHistoryFrames(settings));
    const size_t preferred = last > nominal ? last - nominal : 0;
    const size_t oldest = last > reach ? last - reach : 0;
    if (colliders.triangles.empty() || preferred <= oldest) return preferred;
    RigExecClothGrid grid;
    std::vector<uint64_t> scratch;
    std::vector<int8_t> clear(tickCount, -1);
    const auto isClear = [&](size_t k) {
        if (clear[k] < 0) {
            clear[k] = RigExecClothClearOfColliders(mesh, colliders, ticks[k].entering,
                                                    ticks[k].colliders, &grid, &scratch)
                           ? 1 : 0;
        }
        return clear[k] == 1;
    };
    for (size_t s = preferred + 1; s-- > oldest;) {
        bool calm = true;
        const size_t from = s > nominal ? s - nominal : 0;
        const size_t to = std::min(last, s + nominal);
        for (size_t k = from; calm && k <= to; ++k) calm = isClear(k);
        if (calm) return s;
    }
    return oldest;
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
    if (!(frame > 0) || !std::isfinite(frame)) return passThrough("invalid cloth frame interval");
    ws->x.assign(ticks[0].entering, ticks[0].entering + 3 * n);
    ws->previous.resize(3 * n);
    ws->velocity.resize(3 * n);
    // The cloth starts moving with its entering points.
    for (size_t i = 0; i < 3 * n; ++i)
        ws->velocity[i] = (ticks[1].entering[i] - ticks[0].entering[i]) / frame;
    ws->lambda.assign(mesh.constraintRest.size(), 0.0f);
    size_t bindingCount = 0;
    for (const auto &p : mesh.model.profiles)
        bindingCount += p.source.empty() ? n : p.source.size();
    ws->attachmentLambda.resize(bindingCount);
    ws->contactLambda.resize(n);
    ws->contactTriangles.resize(n);
    float *x = ws->x.data();
    float *xp = ws->previous.data();
    float *v = ws->velocity.data();
    const size_t colors = mesh.ColorCount();
    for (size_t k = 1; k < tickCount; ++k) {
        const RigExecClothTick &from = ticks[k - 1];
        const RigExecClothTick &to = ticks[k];
        const bool collide = colliders.pointCount && !colliders.triangles.empty();
        if (collide) {
            float pad = std::max(std::max(0.0f, to.parameters.thickness),
                                 std::max(0.0f, to.parameters.collisionDepth));
            for (const auto &p : mesh.model.profiles)
                if (p.kind == RigExecClothProfileKind::Collision)
                    for (size_t i = 0; i < n; ++i)
                        pad = std::max(pad, p.thickness * RigExecClothMap(p, ClothThickness, i));
            RigExecBuildClothGrid(colliders, from.colliders, to.colliders,
                                  pad,
                                  &ws->grid, &ws->cellScratch);
        }
        for (int sub = 1; sub <= substeps; ++sub) {
            const RigExecClothStep s =
                RigExecClothStepFor(to.parameters, substeps, sub, frame);
            for (size_t i = 0; i < n; ++i)
                RigExecClothPredict(mesh, s, from.entering, to.entering, i, x, xp, v);
            std::fill(ws->lambda.begin(), ws->lambda.end(), 0.0f);
            std::fill(ws->attachmentLambda.begin(), ws->attachmentLambda.end(), 0.0f);
            std::fill(ws->contactLambda.begin(), ws->contactLambda.end(), 0.0f);
            std::fill(ws->contactTriangles.begin(), ws->contactTriangles.end(), -1);
            ws->selfLambda.clear();
            for (int it = 0; it < iterations; ++it) {
                for (size_t color = 0; color < colors; ++color) {
                    for (uint32_t c = mesh.colorStart[color]; c < mesh.colorStart[color + 1]; ++c)
                        RigExecClothProjectConstraint(mesh, s, c, x, ws->lambda.data());
                }
                if (!mesh.model.Empty())
                    RigExecClothBindings(mesh, s, settings, from, to, x, ws->attachmentLambda.data());
                for (size_t i = 0; i < n; ++i) {
                    RigExecClothParticlePass(mesh, colliders, ws->grid, settings, s,
                                             from.entering, to.entering,
                                             collide ? from.colliders : nullptr,
                                             collide ? to.colliders : nullptr,
                                             it == 0, i, x, xp,
                                             ws->contactLambda.data(), ws->contactTriangles.data());
                }
                if (!mesh.model.Empty()) RigExecClothSelfContacts(mesh, s, xp, ws, x);
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
