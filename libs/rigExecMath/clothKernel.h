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
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

/// The longest window a cloth mover may simulate, in frames.
constexpr int RigExecClothMaxWindowFrames = 1024;
constexpr int RigExecClothMaxSubsteps = 256;
constexpr int RigExecClothMaxIterations = 64;
/// Meshes past this size keep the linear neighbor scans: their bitsets
/// would outgrow the fast caches they exist to serve.
constexpr size_t RigExecClothMaxBitsetPoints = 2048;
/// Meshes past this many constraints keep the twin skip and hash-set
/// edge/edge dedup: their pair bitset would outgrow the fast caches it
/// exists to serve. The bitset is one bit per constraint pair with its
/// diagonal, so 2048 constraints take 256 KiB; verdicts are exact either
/// way.
constexpr size_t RigExecClothMaxPairBitConstraints = 2048;

/// True when the edge/edge discovery dedups through the pair bitset:
/// both edges are stretch constraints, so the unordered pair is one
/// triangular index, cheaper than the twin skip and hash probe it
/// replaces. Past the cap the twin skip and hash sets serve instead.
inline bool RigExecClothUsePairBits(size_t constraintCount)
{
    return constraintCount <= RigExecClothMaxPairBitConstraints;
}

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
    /// Per color, the leading stretch constraints whose both endpoints
    /// are kinematic, partitioned first within the color. At zero
    /// stretch alpha on an empty model those project to no-ops (a zero
    /// denominator returns before any write), so the solve starts past
    /// them.
    std::vector<uint32_t> colorKkSkip;
    /// Zero for a kinematic (fully pinned) particle.
    std::vector<float> inverseMass;
    /// Clamped to [0, 1]; 0 everywhere without pins.
    std::vector<float> pin;
    /// The particle each free particle is tethered to, or -1, and the rest
    /// distance.
    std::vector<int32_t> tetherAnchor;
    std::vector<float> tetherRest;
    /// The tether limit and its short-circuit bound, derived once at build:
    /// limit[i] is tetherRest[i] * (1 + tetherSlack) and shortSquared[i]
    /// the 2^-12 band below its square, the same values the particle pass
    /// formed per substep.
    std::vector<float> tetherLimit;
    std::vector<float> tetherShortSquared;
    RigExecClothModel model;
    std::vector<float> restPoints, materialFrames;
    std::vector<std::vector<int32_t>> incidentTriangles, neighbors;
    /// One bit per vertex per vertex row, row-major, the same relation as
    /// neighbors. Built for self-contact meshes within
    /// RigExecClothMaxBitsetPoints; empty otherwise, selecting the linear
    /// scans.
    std::vector<uint64_t> neighborBits;
    size_t neighborBitStride = 0;
    /// One union row per constraint over the same stride: the vertices
    /// neighboring either endpoint. Built with the bitsets; the edge
    /// adjacency test reads one row instead of two.
    std::vector<uint64_t> edgeNeighborBits;
    /// One union row per triangle over the same stride: the vertices
    /// neighboring any corner. Built with the bitsets; the
    /// vertex/triangle adjacency test reads one row instead of three.
    std::vector<uint64_t> triNeighborBits;
    /// One row base per triangle into triNeighborBits: triangle times the
    /// stride, derived once at build. The adjacency test loads it instead
    /// of multiplying per visit; the same index either way.
    std::vector<size_t> triRowBase;
    /// The three triangle edges as sorted endpoint pairs, two ints per
    /// edge in grid order. Built for self-contact meshes, like the
    /// bitsets; the discovery loads them instead of sorting per visit.
    std::vector<int32_t> triEdges;
    /// Per triangle edge, the smallest other triangle sharing the edge,
    /// or -1 for a boundary edge. Built for self-contact meshes with
    /// the edges; the discovery skips a proven duplicate pair when its
    /// twin was already visited from the same constraint.
    std::vector<int32_t> triEdgeTwin;
    /// Per triangle edge, the stretch constraint over its endpoints.
    /// Every triangle edge is one (the build pushes one per unique mesh
    /// edge), so the discovery maps its triangle edge to a dense pair
    /// id through it. Built for self-contact meshes with the edges.
    std::vector<int32_t> triEdgeConstraint;
    /// Per-constraint material inputs, built with a nonempty model; empty
    /// without one. constraintProfile[c] is the Stretch/Bend profile
    /// projecting the constraint's midpoint sample, or -1, and
    /// constraintStrengthBase holds four floats per constraint (lo, hi,
    /// mask, directional), the position-independent part of its strength.
    /// The solve re-forms only the force curve over them, the same values.
    /// A model mutated after prepare needs a fresh prepare, like any
    /// other compiled input.
    std::vector<int32_t> constraintProfile;
    std::vector<float> constraintStrengthBase;
    /// Per-vertex Damping verdicts and inputs, built with the model: the
    /// damping profile index or -1, and six floats per vertex (lo, hi,
    /// mask, warp, weft, normal). The velocity pass re-forms only the
    /// force curve and the current-frame dots over them.
    std::vector<int32_t> vertexDamping;
    std::vector<float> vertexDampingBase;
    /// Per-vertex Tether verdicts, built with the model: the tether
    /// profile index or -1. The particle pass reads these instead of
    /// scanning the profiles per particle per iteration.
    std::vector<int32_t> vertexTether;
    /// Per-collider per-vertex Collision verdicts, built with the model:
    /// collider c holds [c * pointCount, (c + 1) * pointCount). The
    /// particle pass reads these instead of scanning per particle.
    std::vector<int32_t> vertexCollision;
    /// The self-contact pad over all self profiles and vertices, the same
    /// maximum the self pass formed per call.
    float selfPad = 0;
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
    /// Swept collider bounds over the (before, after) points the grid was
    /// built from: every listed triangle at any substep fraction lies
    /// inside, so the contact search skips particles outside them plus the
    /// reach. A grid that never went through the build covers everything,
    /// so the broadphase never skips on it.
    float sweptLo[3] = {-std::numeric_limits<float>::infinity(),
                        -std::numeric_limits<float>::infinity(),
                        -std::numeric_limits<float>::infinity()};
    float sweptHi[3] = {std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::infinity()};
    /// Per-triangle swept bounds from the measure walk, six floats each,
    /// and their finite verdicts: the span walk rereads these instead of
    /// re-deriving the same bounds. Transient within a build.
    std::vector<float> triBounds;
    std::vector<char> triFinite;
    /// Per-triangle swept-bound shape sum from the measure walk, one float
    /// each: the six absolute bound corners summed in bound order, the
    /// reach-independent part of the contact prefilter's margin. The
    /// prefilter adds the reach over it instead of re-summing per
    /// candidate; the regrouped sum differs by rounding only, far inside
    /// the margin, and the exact tail re-decides every survivor.
    std::vector<float> triShapes;
    /// Per-triangle prefilter boxes over one tick's reach, six floats
    /// each: the swept bounds widened by the tick's contact margin, in
    /// the same arithmetic the prefilter runs per candidate. Filled per
    /// tick where the reach is tick-fixed (the empty-model collider
    /// path); other paths leave it alone and the prefilter derives its
    /// boxes as before. Read only through an explicit pointer, never by
    /// emptiness, so a stale table from another tick is never mistaken
    /// for this tick's.
    std::vector<float> triFilter;
};

/// One substep's interpolated collider triangles: the vertices at the
/// substep fraction and their unit normals, derived once per substep
/// instead of once per particle per candidate. Bit-identical to the
/// per-particle derivation: the same lerps in the same order, then the
/// same normal arithmetic with the same degenerate verdict.
struct RigExecClothContactTris {
    size_t triangleCount = 0;
    /// Nine floats per collider triangle, three corners of three.
    std::vector<float> verts;
    /// Three floats per collider triangle, the unit normal.
    std::vector<float> normals;
    /// Whether the triangle's area vector failed the len > 0 verdict.
    std::vector<char> degenerate;
};

/// A flat set of normalized edge pairs for self-contact dedup: the same
/// insert-or-duplicate verdicts as the ordered set it replaces (the
/// discovery loops keep their order, so every impulse is unchanged), but
/// open-addressed over caller-retained storage with no per-pair work
/// beyond the probe. Slots are a power of two; occupancy is a bitmap, so
/// clearing touches words, not keys. Past 70% load the table doubles and
/// rehashes, so probing always terminates. Occupancy only grows between
/// clears and the hash is deterministic, so a recorded key is always
/// found: verdicts are exact set membership either way.
struct RigExecClothPairSet {
    std::vector<std::array<int32_t, 4>> slots;
    std::vector<uint64_t> occupied;
    uint32_t mask = 0;
    size_t count = 0;

    /// An integer mix, deterministic across runs and platforms. Hash
    /// quality affects probe length only; verdicts are exact regardless.
    /// Odd multipliers permute small vertex indices; keys that differ
    /// only above the mask hash together but stay correct.
    static uint64_t Hash(const std::array<int32_t, 4> &key);
    size_t SlotCount() const;
    void Clear();
    /// True when the key was absent (now recorded), false for a duplicate.
    bool Insert(const std::array<int32_t, 4> &key);
    /// Insert without the load check, after ReserveForBatch covered the
    /// batch: the same probe and the same verdicts.
    bool InsertNoGrow(const std::array<int32_t, 4> &key);
    /// Grows the table until a batch of \p batchKeys inserts fits under
    /// the 70% load cap, so the batch needs no per-insert check. Zero
    /// keys is a no-op, never a spurious grow.
    void ReserveForBatch(size_t batchKeys);
    /// Doubles the slots and rehashes, preserving the recorded set.
    void Grow();
};

/// The same edge-pair dedup set over packed 64-bit keys: four 16-bit
/// endpoint fields, one slot word instead of four, so probes compare one
/// word and the table fits twice the pairs per cache line. Packing is
/// injective exactly while every endpoint fits 16 bits, which the caller
/// guarantees with Packable; larger meshes keep the array set above, with
/// the same verdicts either way.
struct RigExecClothPairSet64 {
    std::vector<uint64_t> slots;
    std::vector<uint64_t> occupied;
    uint32_t mask = 0;
    size_t count = 0;

    /// True when every mesh endpoint packs: endpoints run below the point
    /// count, so a point count of 65536 or less fits every field.
    static bool Packable(size_t pointCount) { return pointCount <= 65536; }
    /// Packs four endpoints, each below 65536, into one key.
    static uint64_t Pack(int32_t a, int32_t b, int32_t c, int32_t d)
    {
        return uint64_t(uint32_t(a)) | (uint64_t(uint32_t(b)) << 16) |
               (uint64_t(uint32_t(c)) << 32) | (uint64_t(uint32_t(d)) << 48);
    }
    /// An integer mix, deterministic across runs and platforms. Hash
    /// quality affects probe length only; verdicts are exact regardless.
    static uint64_t Hash(uint64_t key);
    size_t SlotCount() const;
    void Clear();
    /// True when the key was absent (now recorded), false for a duplicate.
    bool Insert(uint64_t key);
    /// Insert without the load check, after ReserveForBatch covered the
    /// batch: the same probe and the same verdicts.
    bool InsertNoGrow(uint64_t key);
    /// Insert from four endpoints, hashing them directly: the packed
    /// fields all sit below 2^16, so the mix matches Hash(Pack(a, b,
    /// c, d)) exactly and the probe takes the identical slots.
    bool InsertNoGrowFields(int32_t a, int32_t b, int32_t c, int32_t d);
    /// Grows the table until a batch of \p batchKeys inserts fits under
    /// the 25% load cap, so the batch needs no per-insert check. The
    /// lower cap shortens probes; verdicts stay exact either way. Zero
    /// keys is a no-op, never a spurious grow.
    void ReserveForBatch(size_t batchKeys);
    /// Doubles the slots and rehashes, preserving the recorded set.
    void Grow();
};
// The method bodies live at the end of the header, after the solve, so
// this model-path-only code lands after the hot loop in .text instead of
// shifting it.

/// Lazily fresh edge spheres for the self edge/edge band, over caller-
/// retained storage. Slots [0, constraints) mirror the constraints and the
/// rest mirror the triangle edges in grid order. A sphere is fresh when its
/// compute tick covers both endpoint touch ticks; a correction retires
/// every sphere over a vertex it moves by advancing that vertex past them.
/// Ticks only grow within a solve, so the version test is exact.
struct RigExecClothEdgeSpheres {
    std::vector<float> centers;
    std::vector<float> radii;
    std::vector<uint64_t> edgeTick;
    std::vector<uint64_t> vertTick;
    uint64_t clock = 0;
};

/// Lazily fresh triangle spheres for the self vertex/triangle band, over
/// caller-retained storage. Freshness reuses the edge spheres' vertex
/// ticks and clock: a sphere is fresh when its compute tick covers all
/// three corner touch ticks, so a correction retires every sphere over a
/// vertex it moves with no new clock.
struct RigExecClothTriSpheres {
    std::vector<float> centers;
    std::vector<float> radii;
    std::vector<uint64_t> triTick;
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
    /// The cloth as its own collider set for the self-contact grids,
    /// assigned per call, its triangle buffer retained.
    RigExecClothColliders selfTopology;
    /// The edge-pair dedup set, cleared per self profile, its slots retained.
    RigExecClothPairSet selfPairs;
    /// The packed-key dedup set for meshes whose endpoints fit 16 bits,
    /// cleared per self profile like the array set, its slots retained.
    RigExecClothPairSet64 selfPairs64;
    /// Per-triangle bucket marks for the self edge/edge twin skip,
    /// cleared per call, the storage retained.
    std::vector<uint32_t> selfTwinStamp;
    /// The edge-pair bitset, one bit per constraint pair with its
    /// diagonal, cleared per self profile, its storage retained. Small
    /// meshes dedup through it instead of the twin skip and hash sets,
    /// with the same verdicts.
    std::vector<uint64_t> selfPairBits;
    /// The self edge/edge spheres, sized per call, their storage retained.
    RigExecClothEdgeSpheres selfSpheres;
    /// The self vertex/triangle spheres, sized per call, retained alike.
    RigExecClothTriSpheres selfTriSpheres;
    RigExecClothGrid grid;
    std::vector<uint64_t> cellScratch;
    RigExecClothContactTris contacts;
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
// AdjacentTo is declared here and defined at the end of the header, after
// the solve, with the other model-path-only code.
inline bool AdjacentTo(const std::vector<int32_t> &sorted, int32_t v);
// The same verdict from the neighbor bitsets, for meshes that built them.
inline bool AdjacentToBits(const RigExecClothMesh &mesh, int32_t a, int32_t b);
// True when either endpoint of constraint c neighbors v, from the
// constraint's union row.
inline bool AdjacentToEdgeBits(const RigExecClothMesh &mesh, size_t c, int32_t v);
// The same verdict from a held union row, for callers that hoisted it.
inline bool AdjacentToRowBits(const uint64_t *row, int32_t v);
// True when any corner of triangle t neighbors v, from the triangle's
// union row.
inline bool AdjacentToTriBits(const RigExecClothMesh &mesh, size_t t, int32_t v);

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
    built.tetherLimit.assign(pointCount, 0.0f);
    built.tetherShortSquared.assign(pointCount, -1.0f);
    for (size_t i = 0; i < pointCount; ++i) {
        const float limit = built.tetherRest[i] * (1.0f + settings.tetherSlack);
        const float limitSquared = limit * limit;
        built.tetherLimit[i] = limit;
        built.tetherShortSquared[i] = (limit < 1e-18f || !std::isfinite(limitSquared))
            ? -1.0f : limitSquared * (1.0f - 0.000244140625f);
    }
    // Stable-partition each color's stretch kinematic pairs first: a
    // reorder within the color, so the disjointness the colors promise
    // still holds and every evaluator reads the same sets.
    built.colorKkSkip.assign(built.ColorCount(), 0);
    const auto skippable = [&](uint32_t c) {
        const size_t a = size_t(built.constraintEnds[2 * c]);
        const size_t b = size_t(built.constraintEnds[2 * c + 1]);
        return built.constraintKind[c] == uint8_t(RigExecClothConstraint::Stretch) &&
               built.inverseMass[a] == 0.0f && built.inverseMass[b] == 0.0f;
    };
    for (size_t k = 0; k < built.ColorCount(); ++k) {
        const uint32_t lo = built.colorStart[k], hi = built.colorStart[k + 1];
        std::vector<uint32_t> order;
        for (uint32_t c = lo; c < hi; ++c)
            if (skippable(c)) order.push_back(c);
        built.colorKkSkip[k] = uint32_t(order.size());
        for (uint32_t c = lo; c < hi; ++c)
            if (!skippable(c)) order.push_back(c);
        std::vector<int32_t> ends(2 * order.size());
        std::vector<float> rest(order.size());
        std::vector<uint8_t> kind(order.size());
        for (size_t j = 0; j < order.size(); ++j) {
            ends[2 * j] = built.constraintEnds[2 * order[j]];
            ends[2 * j + 1] = built.constraintEnds[2 * order[j] + 1];
            rest[j] = built.constraintRest[order[j]];
            kind[j] = built.constraintKind[order[j]];
        }
        for (size_t j = 0; j < order.size(); ++j) {
            built.constraintEnds[2 * (lo + j)] = ends[2 * j];
            built.constraintEnds[2 * (lo + j) + 1] = ends[2 * j + 1];
            built.constraintRest[lo + j] = rest[j];
            built.constraintKind[lo + j] = kind[j];
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
    mesh->constraintProfile.clear();
    mesh->constraintStrengthBase.clear();
    mesh->vertexDamping.clear();
    mesh->vertexDampingBase.clear();
    mesh->vertexTether.clear();
    mesh->vertexCollision.clear();
    mesh->selfPad = 0;
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
    mesh->neighborBits.clear();
    mesh->neighborBitStride = 0;
    mesh->edgeNeighborBits.clear();
    mesh->triNeighborBits.clear();
    mesh->triRowBase.clear();
    bool selfContact = false;
    for (const auto &p : model.profiles)
        selfContact = selfContact || p.kind == RigExecClothProfileKind::SelfCollision;
    if (selfContact && mesh->pointCount <= RigExecClothMaxBitsetPoints) {
        mesh->neighborBitStride = (mesh->pointCount + 63) / 64;
        mesh->neighborBits.assign(mesh->pointCount * mesh->neighborBitStride, 0);
        for (size_t a = 0; a < mesh->pointCount; ++a)
            for (int32_t b : mesh->neighbors[a])
                mesh->neighborBits[a * mesh->neighborBitStride + (size_t(b) >> 6)] |=
                    1ull << (b & 63);
        mesh->edgeNeighborBits.assign(mesh->constraintRest.size() * mesh->neighborBitStride, 0);
        for (size_t c = 0; c < mesh->constraintRest.size(); ++c) {
            const size_t a = size_t(mesh->constraintEnds[2 * c]);
            const size_t b = size_t(mesh->constraintEnds[2 * c + 1]);
            for (size_t w = 0; w < mesh->neighborBitStride; ++w)
                mesh->edgeNeighborBits[c * mesh->neighborBitStride + w] =
                    mesh->neighborBits[a * mesh->neighborBitStride + w] |
                    mesh->neighborBits[b * mesh->neighborBitStride + w];
        }
        const size_t triCount = mesh->triangles.size() / 3;
        mesh->triNeighborBits.assign(triCount * mesh->neighborBitStride, 0);
        mesh->triRowBase.resize(triCount);
        for (size_t t = 0; t < triCount; ++t)
            mesh->triRowBase[t] = t * mesh->neighborBitStride;
        for (size_t t = 0; t < triCount; ++t) {
            const size_t a = size_t(mesh->triangles[3 * t]);
            const size_t b = size_t(mesh->triangles[3 * t + 1]);
            const size_t d = size_t(mesh->triangles[3 * t + 2]);
            for (size_t w = 0; w < mesh->neighborBitStride; ++w)
                mesh->triNeighborBits[t * mesh->neighborBitStride + w] =
                    mesh->neighborBits[a * mesh->neighborBitStride + w] |
                    mesh->neighborBits[b * mesh->neighborBitStride + w] |
                    mesh->neighborBits[d * mesh->neighborBitStride + w];
        }
    }
    mesh->triEdges.clear();
    mesh->triEdgeTwin.clear();
    if (selfContact) {
        mesh->triEdges.resize(2 * mesh->triangles.size());
        for (size_t t = 0; t < mesh->triangles.size() / 3; ++t)
            for (size_t e = 0; e < 3; ++e) {
                const int32_t p = mesh->triangles[3 * t + e];
                const int32_t q = mesh->triangles[3 * t + (e + 1) % 3];
                mesh->triEdges[6 * t + 2 * e] = std::min(p, q);
                mesh->triEdges[6 * t + 2 * e + 1] = std::max(p, q);
            }
        // The twin of each triangle edge: group the edges by endpoints
        // and record the smallest other triangle of each group.
        struct TwinEdge { uint64_t key; int32_t tri; int32_t edge; };
        std::vector<TwinEdge> order;
        order.reserve(mesh->triangles.size());
        for (size_t t = 0; t < mesh->triangles.size() / 3; ++t)
            for (size_t e = 0; e < 3; ++e) {
                const uint64_t k = uint64_t(uint32_t(mesh->triEdges[6 * t + 2 * e]));
                const uint64_t l = uint64_t(uint32_t(mesh->triEdges[6 * t + 2 * e + 1]));
                order.push_back({(k << 32) | l, int32_t(t), int32_t(e)});
            }
        std::sort(order.begin(), order.end(), [](const TwinEdge &a, const TwinEdge &b) {
            return a.key < b.key;
        });
        mesh->triEdgeTwin.assign(mesh->triangles.size(), -1);
        for (size_t r = 0; r < order.size();) {
            size_t s = r + 1;
            while (s < order.size() && order[s].key == order[r].key) ++s;
            int32_t lo = order[r].tri, next = -1;
            for (size_t m = r; m < s; ++m) {
                const int32_t t = order[m].tri;
                if (t < lo) { next = lo; lo = t; }
                else if (t != lo && (next < 0 || t < next)) next = t;
            }
            for (size_t m = r; m < s; ++m)
                mesh->triEdgeTwin[3 * size_t(order[m].tri) + size_t(order[m].edge)] =
                    (order[m].tri == lo) ? next : lo;
            r = s;
        }
        // The constraint index of each triangle edge: every triangle edge
        // is a stretch constraint (the build pushed one per unique mesh
        // edge), so a sorted join over the sorted endpoint pairs maps
        // each edge to its constraint. A miss is impossible by
        // construction and would stay -1.
        mesh->triEdgeConstraint.assign(mesh->triangles.size(), -1);
        struct EdgeConstraint { uint64_t key; int32_t c; };
        std::vector<EdgeConstraint> edgeOrder;
        edgeOrder.reserve(mesh->constraintRest.size());
        for (size_t c = 0; c < mesh->constraintRest.size(); ++c) {
            if (mesh->constraintKind[c] != uint8_t(RigExecClothConstraint::Stretch)) continue;
            const uint64_t a = uint64_t(uint32_t(mesh->constraintEnds[2 * c]));
            const uint64_t b = uint64_t(uint32_t(mesh->constraintEnds[2 * c + 1]));
            edgeOrder.push_back({(a << 32) | b, int32_t(c)});
        }
        std::sort(edgeOrder.begin(), edgeOrder.end(),
                  [](const EdgeConstraint &x, const EdgeConstraint &y) { return x.key < y.key; });
        for (size_t t = 0; t < mesh->triangles.size() / 3; ++t)
            for (size_t e = 0; e < 3; ++e) {
                const uint64_t a = uint64_t(uint32_t(mesh->triEdges[6 * t + 2 * e]));
                const uint64_t b = uint64_t(uint32_t(mesh->triEdges[6 * t + 2 * e + 1]));
                const uint64_t key = (a << 32) | b;
                const auto found =
                    std::lower_bound(edgeOrder.begin(), edgeOrder.end(), key,
                                     [](const EdgeConstraint &x, uint64_t k) { return x.key < k; });
                if (found != edgeOrder.end() && found->key == key)
                    mesh->triEdgeConstraint[3 * t + e] = found->c;
            }
    }
    mesh->materialFrames.resize(9 * mesh->pointCount);
    for (size_t i = 0; i < mesh->pointCount; ++i)
        RigExecClothMaterialFrame(*mesh, rest, i, mesh->materialFrames.data() + 9 * i);
    mesh->colliderOffsets.assign(1, 0);
    for (size_t n : colliderCounts) mesh->colliderOffsets.push_back(mesh->colliderOffsets.back() + n);
    // Position-independent strength inputs, derived once: profile verdicts,
    // sample maps, and rest-direction blends never see the solve's points,
    // so the per-substep kernels re-form only the force curve and the
    // current-frame dots over them. The same values, bit for bit.
    mesh->constraintProfile.assign(mesh->constraintRest.size(), -1);
    mesh->constraintStrengthBase.assign(4 * mesh->constraintRest.size(), 0);
    for (size_t c = 0; c < mesh->constraintRest.size(); ++c) {
        const size_t a = size_t(mesh->constraintEnds[2 * c]);
        const size_t b = size_t(mesh->constraintEnds[2 * c + 1]);
        RigExecClothSample sample;
        sample.vertices = {{int32_t(a), int32_t(b), int32_t(b)}};
        sample.weights = {{0.5f, 0.5f, 0}};
        const auto kind = mesh->constraintKind[c] == uint8_t(RigExecClothConstraint::Stretch)
            ? RigExecClothProfileKind::Stretch : RigExecClothProfileKind::Bend;
        const RigExecClothProfile *profile = RigExecClothProfileFor(*mesh, kind, sample);
        if (!profile) continue;
        mesh->constraintProfile[c] = int32_t(profile - mesh->model.profiles.data());
        float restDirection[3];
        cloth_detail::Sub(mesh->restPoints.data() + 3 * a,
                          mesh->restPoints.data() + 3 * b, restDirection);
        const float rl = cloth_detail::Length(restDirection);
        for (float &q : restDirection) q /= rl;
        float *base = mesh->constraintStrengthBase.data() + 4 * c;
        base[0] = profile->minStrength * RigExecClothSampleMap(*profile, ClothMinStrength, sample);
        base[1] = profile->maxStrength * RigExecClothSampleMap(*profile, ClothMaxStrength, sample);
        base[2] = RigExecClothSampleMap(*profile, ClothMask, sample);
        const float warpMaps[3] = {
            RigExecClothSampleMap(*profile, ClothWarp, sample),
            RigExecClothSampleMap(*profile, ClothWeft, sample),
            RigExecClothSampleMap(*profile, ClothNormal, sample)};
        base[3] = RigExecClothDirectional(*profile, restDirection,
                                           mesh->materialFrames.data() + 9 * a, warpMaps);
    }
    mesh->vertexDamping.assign(mesh->pointCount, -1);
    mesh->vertexDampingBase.assign(6 * mesh->pointCount, 0);
    mesh->vertexTether.assign(mesh->pointCount, -1);
    for (size_t i = 0; i < mesh->pointCount; ++i) {
        const RigExecClothSample vertex = RigExecClothVertex(i);
        if (const RigExecClothProfile *profile =
                RigExecClothProfileFor(*mesh, RigExecClothProfileKind::Damping, vertex)) {
            mesh->vertexDamping[i] = int32_t(profile - mesh->model.profiles.data());
            float *base = mesh->vertexDampingBase.data() + 6 * i;
            base[0] = profile->minStrength *
                RigExecClothSampleMap(*profile, ClothMinStrength, vertex);
            base[1] = profile->maxStrength *
                RigExecClothSampleMap(*profile, ClothMaxStrength, vertex);
            base[2] = RigExecClothSampleMap(*profile, ClothMask, vertex);
            base[3] = RigExecClothSampleMap(*profile, ClothWarp, vertex);
            base[4] = RigExecClothSampleMap(*profile, ClothWeft, vertex);
            base[5] = RigExecClothSampleMap(*profile, ClothNormal, vertex);
        }
        if (const RigExecClothProfile *profile =
                RigExecClothProfileFor(*mesh, RigExecClothProfileKind::Tether, vertex))
            mesh->vertexTether[i] = int32_t(profile - mesh->model.profiles.data());
    }
    const size_t colliderCount = mesh->colliderOffsets.size() - 1;
    mesh->vertexCollision.assign(colliderCount * mesh->pointCount, -1);
    for (size_t c = 0; c < colliderCount; ++c) {
        for (size_t i = 0; i < mesh->pointCount; ++i) {
            const RigExecClothSample vertex = RigExecClothVertex(i);
            for (size_t k = mesh->model.profiles.size(); k-- > 0;) {
                const auto &profile = mesh->model.profiles[k];
                if (profile.kind == RigExecClothProfileKind::Collision &&
                    (profile.targetCollider < 0 || profile.targetCollider == int32_t(c)) &&
                    RigExecClothRegion(profile.sourceMask, vertex) > 0) {
                    mesh->vertexCollision[c * mesh->pointCount + i] = int32_t(k);
                    break;
                }
            }
        }
    }
    for (const auto &p : mesh->model.profiles)
        if (p.kind == RigExecClothProfileKind::SelfCollision)
            for (size_t i = 0; i < mesh->pointCount; ++i)
                mesh->selfPad = std::max(mesh->selfPad,
                    p.thickness * RigExecClothMap(p, ClothThickness, i));
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
        for (int a = 0; a < 3; ++a) {
            grid->sweptLo[a] = -std::numeric_limits<float>::infinity();
            grid->sweptHi[a] = std::numeric_limits<float>::infinity();
        }
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
    // The same walk gathers the broadphase bounds over the same finite
    // triangles the grid lists, so every candidate lies inside them, and
    // keeps every triangle's bounds for the span walk below, which
    // rereads them instead of re-deriving the same values.
    double extent = 0.0;
    size_t measured = 0;
    for (int a = 0; a < 3; ++a) {
        grid->sweptLo[a] = std::numeric_limits<float>::infinity();
        grid->sweptHi[a] = -std::numeric_limits<float>::infinity();
    }
    grid->triBounds.resize(6 * triangleCount);
    grid->triFinite.resize(triangleCount);
    grid->triShapes.resize(triangleCount);
    for (size_t t = 0; t < triangleCount; ++t) {
        float lo[3], hi[3];
        const bool finite = bounds(t, lo, hi);
        grid->triFinite[t] = finite ? 1 : 0;
        float shapes = 0;
        for (int a = 0; a < 3; ++a) {
            grid->triBounds[6 * t + size_t(a)] = lo[a];
            grid->triBounds[6 * t + size_t(a) + 3] = hi[a];
            shapes += std::abs(lo[a]);
        }
        for (int a = 0; a < 3; ++a) shapes += std::abs(hi[a]);
        grid->triShapes[t] = shapes;
        if (!finite) continue;
        for (int a = 0; a < 3; ++a) {
            grid->sweptLo[a] = std::min(grid->sweptLo[a], lo[a]);
            grid->sweptHi[a] = std::max(grid->sweptHi[a], hi[a]);
        }
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
    // sort by bucket, which keeps the triangles of a bucket ascending. The
    // counting sort is stable over triangle order, so each bucket lists the
    // same triangles ascending whether one triangle's pairs arrive sorted
    // or in span order; the per-triangle dedup runs on stamps instead of
    // a sort, over the starts the counting sort clears next.
    grid->bucketStart.assign(size_t(buckets) + 1, 0);
    scratch->clear();
    const float *kept = grid->triBounds.data();
    const char *keptFinite = grid->triFinite.data();
    for (size_t t = 0; t < triangleCount; ++t) {
        if (!keptFinite[t]) continue;
        int64_t c0[3], c1[3];
        for (int a = 0; a < 3; ++a) {
            c0[a] = Cell(kept[6 * t + size_t(a)] - pad, cell);
            c1[a] = Cell(kept[6 * t + size_t(a) + 3] + pad, cell);
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
        const uint32_t mark = uint32_t(t) + 1;
        for (int64_t x = c0[0]; x <= c1[0]; ++x)
            for (int64_t y = c0[1]; y <= c1[1]; ++y)
                for (int64_t z = c0[2]; z <= c1[2]; ++z) {
                    const uint32_t b = Hash(x, y, z, grid->bucketMask);
                    if (grid->bucketStart[b] != mark) {
                        grid->bucketStart[b] = mark;
                        scratch->push_back((uint64_t(t) << 32) | uint64_t(b));
                    }
                }
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

/// The swept-collider-bounds-plus-reach box in doubles: the same
/// per-axis box the broadphase verdict compares against, derived once
/// per substep instead of once per particle.
struct RigExecClothReachBounds {
    double lo[3];
    double hi[3];
};

/// Derives the broadphase box of \p grid plus \p reach.
inline void
RigExecClothSweptReachBounds(const RigExecClothGrid &grid, float reach,
                             RigExecClothReachBounds *out)
{
    for (int a = 0; a < 3; ++a) {
        out->lo[a] = double(grid.sweptLo[a]) - double(reach) -
            (std::abs(double(grid.sweptLo[a])) + double(reach)) * 9.5367431640625e-07;
        out->hi[a] = double(grid.sweptHi[a]) + double(reach) +
            (std::abs(double(grid.sweptHi[a])) + double(reach)) * 9.5367431640625e-07;
    }
}

/// The broadphase verdict against derived bounds.
inline bool
RigExecClothOutsideSweptReach(const RigExecClothReachBounds *bounds,
                              const float *p)
{
    for (int a = 0; a < 3; ++a) {
        const double point = double(p[a]);
        if (point < bounds->lo[a] || point > bounds->hi[a]) return true;
    }
    return false;
}

/// True when particle position \p p lies outside the grid's swept collider
/// bounds plus \p reach: farther than the reach from every triangle, so
/// the contact search would fail each candidate on the reach verdict and
/// return empty. The 2^-20 relative margin dwarfs the bound arithmetic's
/// double rounding (2^-53) by 2^33, so a flagged particle is truly
/// outside; boundary, NaN, and infinite inputs fail the comparisons and
/// take the full path. The contact search's broadphase, shared so the
/// particle pass skips the search itself on the verdict.
inline bool
RigExecClothOutsideSweptBounds(const RigExecClothGrid &grid, const float *p,
                               float reach)
{
    RigExecClothReachBounds bounds;
    RigExecClothSweptReachBounds(grid, reach, &bounds);
    return RigExecClothOutsideSweptReach(&bounds, p);
}

/// Interpolates every collider triangle of \p colliders at substep
/// fraction \p t between \p before and \p after into \p out: the same
/// corner lerps in the same order RigExecClothFindContact derives them,
/// then the same area-vector arithmetic with the same verdict. A null
/// point range leaves \p out empty, which covers no triangle.
inline void
RigExecClothContactTrisAt(const RigExecClothColliders &colliders,
                           const float *before, const float *after, float t,
                           RigExecClothContactTris *out)
{
    using namespace cloth_detail;
    out->triangleCount = 0;
    out->verts.clear();
    out->normals.clear();
    out->degenerate.clear();
    if (!before || !after) return;
    const size_t count = colliders.triangles.size() / 3;
    out->verts.resize(9 * count);
    out->normals.resize(3 * count);
    out->degenerate.assign(count, 0);
    out->triangleCount = count;
    // The output and collider bases stay fixed across the triangle loop:
    // the vectors are fully sized above and the loop only assigns
    // elements, so no reallocation moves the bases and the loop rereads
    // the same bases without reloading them per triangle.
    float *ctVertsBase = out->verts.data();
    float *ctNormalsBase = out->normals.data();
    const int32_t *ctColliderTris = colliders.triangles.data();
    for (size_t tri = 0; tri < count; ++tri) {
        float *v0 = ctVertsBase + 9 * tri;
        float *v1 = v0 + 3;
        float *v2 = v0 + 6;
        for (int c = 0; c < 3; ++c) {
            const size_t i = size_t(ctColliderTris[3 * tri + size_t(c)]);
            Lerp(before + 3 * i, after + 3 * i, t, ctVertsBase + 9 * tri + 3 * c);
        }
        float e1[3], e2[3];
        float *n = ctNormalsBase + 3 * tri;
        Sub(v1, v0, e1); Sub(v2, v0, e2); Cross(e1, e2, n);
        const float len = Length(n);
        if (!(len > 0.0f)) {
            out->degenerate[tri] = 1;
            continue;
        }
        n[0] /= len; n[1] /= len; n[2] /= len;
    }
}

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
///
/// \p precomputed, when it covers a candidate, supplies the interpolated
/// triangle and its normal instead of deriving them; a triangle it does
/// not cover derives them as before. Either way the answer is the same.
/// \p filterBoxes, when given, holds the tick's prefilter boxes (six
/// floats per triangle, the swept bounds widened by this search's
/// reach in the prefilter's own arithmetic); the prefilter reads them
/// instead of deriving them. A null pointer derives them as before.
inline RigExecClothContact
RigExecClothFindContact(const RigExecClothColliders &colliders,
                        const RigExecClothGrid &grid, const float *before,
                        const float *after, float t, const float *p,
                        float thickness, float depth, const float *previous = nullptr,
                        float t0 = 0, size_t twoSidedBegin = 0,
                        size_t twoSidedEnd = std::numeric_limits<size_t>::max(),
                        bool filterCollider = false,
                        const std::vector<float> *region = nullptr,
                        const RigExecClothContactTris *precomputed = nullptr,
                        bool broadphaseKnownInside = false,
                        const float *filterBoxes = nullptr)
{
    using namespace cloth_detail;
    RigExecClothContact best;
    if (grid.bucketStart.size() < 2 || colliders.triangles.empty()) return best;
    const float reach = std::max(thickness, depth);
    float bestDistance = std::numeric_limits<float>::infinity();
    // Broadphase: outside the swept collider bounds plus the reach, a
    // particle is farther than the reach from every triangle, so the
    // bucket search below would fail each candidate on the reach verdict
    // and return empty. A caller that already tested the same box over
    // the same reach passes broadphaseKnownInside and skips the
    // re-derivation; the verdict would repeat bit for bit.
    if (!broadphaseKnownInside &&
        RigExecClothOutsideSweptBounds(grid, p, reach))
        return best;
    const uint32_t bucket = Hash(Cell(p[0], grid.cellSize), Cell(p[1], grid.cellSize),
                                 Cell(p[2], grid.cellSize), grid.bucketMask);
    // The normal flip below needs the previous point: without one no
    // candidate flips, so the vertex-slab pre-test runs one-sided.
    const bool mayFlip = previous != nullptr;
    {
        for (uint32_t k = grid.bucketStart[bucket]; k < grid.bucketStart[bucket + 1]; ++k) {
            const int32_t tri = grid.bucketTriangles[k];
            const size_t firstVertex = size_t(colliders.triangles[3 * size_t(tri)]);
            if (filterCollider && (firstVertex < twoSidedBegin || firstVertex >= twoSidedEnd)) continue;
            // Swept-box pre-filter: a candidate whose tick-swept bounds sit
            // farther than the reach from p fails the reach verdict below,
            // so it skips the fetch, the barycentric solve, and the distance.
            // The interpolated triangle stays within the bound of its swept
            // box: the corner lerps round by ~3ulp of the corner magnitudes,
            // the bound forms with ~3ulp more, and the distance rounds by
            // ~2.5ulp of itself (below twice the reach once the farther
            // half is excluded), all far inside the 2^-19 relative band
            // over the corners plus the reach. The 1e-30 floor covers
            // subnormal scales, where the relative band underflows below the
            // absolute rounding. NaN inputs compare false and take the full
            // path as before.
            {
                // The tick's prefilter boxes, where the caller derived
                // them: the same widened bounds the block below forms,
                // read instead of recomputed per candidate.
                if (filterBoxes) {
                    const float *fb = filterBoxes + 6 * size_t(tri);
                    bool outside = false;
                    for (int a = 0; a < 3; ++a) {
                        if (p[a] < fb[a] || p[a] > fb[a + 3]) {
                            outside = true;
                            break;
                        }
                    }
                    if (outside) continue;
                } else {
                    const float *tb = grid.triBounds.data() + 6 * size_t(tri);
                    const float shapes = reach + grid.triShapes[size_t(tri)];
                    const float bound = reach + shapes * 1.9073486328125e-06f + 1e-30f;
                    bool outside = false;
                    for (int a = 0; a < 3; ++a) {
                        if (p[a] < tb[a] - bound || p[a] > tb[a + 3] + bound) {
                            outside = true;
                            break;
                        }
                    }
                    if (outside) continue;
                }
            }
            float localV[3][3];
            float n[3];
            const float *v[3] = {localV[0], localV[1], localV[2]};
            if (precomputed && size_t(tri) < precomputed->triangleCount) {
                if (precomputed->degenerate[size_t(tri)]) continue;
                const float *row = precomputed->verts.data() + 9 * size_t(tri);
                v[0] = row; v[1] = row + 3; v[2] = row + 6;
                const float *held = precomputed->normals.data() + 3 * size_t(tri);
                n[0] = held[0]; n[1] = held[1]; n[2] = held[2];
            } else {
                for (int c = 0; c < 3; ++c) {
                    const size_t i = size_t(colliders.triangles[3 * size_t(tri) + size_t(c)]);
                    Lerp(before + 3 * i, after + 3 * i, t, localV[c]);
                }
                float e1[3], e2[3];
                Sub(localV[1], localV[0], e1); Sub(localV[2], localV[0], e2);
                Cross(e1, e2, n);
                const float len = Length(n);
                if (!(len > 0.0f)) continue;
                n[0] /= len; n[1] /= len; n[2] /= len;
            }
            // The vertex-slab pre-test: the closest point is a convex
            // combination of the corners, so its slab distance sits
            // between the corners' slab distances, and three dots over
            // the fetched triangle and normal decide the slab verdict
            // below without the barycentric solve, the point, or the
            // distance. The 2^-15 band over the prefilter's shape sum
            // plus the reach dwarfs the dots' and the slab's rounding
            // (tens of ulps of the coordinates, which the swept box
            // the survivor sits inside bounds), and the 1e-30 floor
            // covers subnormal scales, so a skipped candidate fails
            // the slab test bit for bit. Where the normal may flip,
            // both outcomes kill past the reach on either side. NaN
            // inputs compare false and take the full path as before.
            {
                const float np = Dot(n, p);
                const float s0 = np - Dot(n, v[0]);
                const float s1 = np - Dot(n, v[1]);
                const float s2 = np - Dot(n, v[2]);
                const float margin =
                    (grid.triShapes[size_t(tri)] + reach + reach) * 3.0517578125e-05f + 1e-30f;
                if (mayFlip && firstVertex >= twoSidedBegin && firstVertex < twoSidedEnd) {
                    const float front = reach + margin, back = -reach - margin;
                    if ((s0 > front && s1 > front && s2 > front) ||
                        (s0 < back && s1 < back && s2 < back))
                        continue;
                } else {
                    const float front = thickness + margin, back = -depth - margin;
                    if ((s0 > front && s1 > front && s2 > front) ||
                        (s0 < back && s1 < back && s2 < back))
                        continue;
                }
            }
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
            // The slab test before the root: a candidate outside
            // [-depth, thickness] along the normal is rejected with or
            // without the distance, so two compares skip the root. The
            // three verdicts are a conjunction of continues, so testing
            // the slab first rejects the same candidates bit for bit,
            // and NaN signed distances compare false either way.
            if (signedDistance >= thickness || signedDistance < -depth) continue;
            const float distance = Length(d);
            if (distance > reach) continue;
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
    float reach = 0.0f;
    float gravity[3] = {0, 0, 0};
};

/// Predict pass for particle \p i: gravity and inertia for a free particle;
/// a kinematic one is placed on its entering points at the substep.
/// \p gravityStep is the substep's gravity times its length, derived once
/// by the caller: the values are the same products the per-particle loop
/// formed, but the compiler cannot hoist them itself because the stores
/// below could alias the step it reads them from.
inline void
RigExecClothPredict(const RigExecClothMesh &mesh, const RigExecClothStep &s,
                    const float *from, const float *to, size_t i, float *x,
                    float *previous, float *velocity, const float *gravityStep)
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
        vi[a] += gravityStep[a];
        pi[a] += vi[a] * s.h;
    }
}

/// Projects constraint \p c with no lambda traffic: the iterations==1
/// form, where every lambda read sees the substep fill (+0.0f), so the
/// alpha term and the write are dead. Bit-identical: -C - (+0.0f) is -C
/// for every C including signed zeros, and the model path substitutes
/// the same +0.0f the fill wrote.
inline void
RigExecClothProjectConstraintSingle(const RigExecClothMesh &mesh,
                                    const RigExecClothStep &s, size_t c,
                                    float *x)
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
        const int32_t pi = mesh.constraintProfile[c];
        if (pi >= 0) {
            const RigExecClothProfile &profile = mesh.model.profiles[size_t(pi)];
            const float *base = mesh.constraintStrengthBase.data() + 4 * c;
            // A flat strength range needs no force estimate: the curve
            // interpolates between equal endpoints, so the strength is
            // the endpoint times the mask and the directional scalar,
            // and the force quotient is dead. The +0.0f keeps the
            // signed-zero behavior of the full form bit for bit;
            // infinite endpoints and NaN take the full path.
            float strength;
            if (base[0] == base[1] && std::isfinite(base[0])) {
                strength = (base[0] + 0.0f) * base[2] * base[3];
            } else {
                const float force =
                    wa + wb > 0 ? std::abs(0.0f - C / (wa + wb)) / (s.h * s.h) : 0;
                strength = RigExecClothCachedBase(profile, base, force) * base[3];
            }
            if (!(strength > 0)) return;
            alpha = profile.hard ? 0 : profile.compliance / (strength * s.h * s.h);
        }
    }
    const float denom = wa + wb + alpha;
    if (!(denom > 0.0f)) return;
    const float dl = -C / denom;
    const float ka = wa * dl / len, kb = wb * dl / len;
    for (int q = 0; q < 3; ++q) {
        x[3 * a + size_t(q)] += d[q] * ka;
        x[3 * b + size_t(q)] -= d[q] * kb;
    }
}

/// Projects one color's constraints [begin, end) with the
/// iterations==1 form, in two passes per chunk: the first derives each
/// live constraint's direction and deltas with no stores to \p x, the
/// second applies them. A color's members share no particle, so the
/// chunk's first-pass reads see the same values the serial order saw
/// and the second-pass writes land on disjoint particles; each
/// constraint runs the same operations in the same order, so the pass
/// writes the same bits while the root-and-divide chain overlaps
/// across the chunk instead of serializing. The stashed endpoints keep
/// their int32_t encoding so the pass-2 indexing matches the serial
/// conversion exactly. \p Model selects the material-profile path at
/// compile time; the caller instantiates false for an empty model so
/// the hot loop carries no model branch or dead code. The true path
/// reads the prepared strength cache behind one model check, so it
/// still agrees with the false path on an empty model.
template <bool Model>
inline void
RigExecClothProjectColorSingle(const RigExecClothMesh &mesh,
                               const RigExecClothStep &s, uint32_t begin,
                               uint32_t end, float *x)
{
    using namespace cloth_detail;
    constexpr uint32_t kChunk = 64;
    for (uint32_t base = begin; base < end;) {
        const uint32_t n = end - base < kChunk ? end - base : kChunk;
        float d[kChunk][3];
        float ka[kChunk], kb[kChunk];
        int32_t ends[kChunk][2];
        char live[kChunk];
        for (uint32_t k = 0; k < n; ++k) {
            const size_t c = size_t(base + k);
            ends[k][0] = mesh.constraintEnds[2 * c];
            ends[k][1] = mesh.constraintEnds[2 * c + 1];
            const size_t a = size_t(ends[k][0]);
            const size_t b = size_t(ends[k][1]);
            const float wa = mesh.inverseMass[a], wb = mesh.inverseMass[b];
            float alpha = mesh.constraintKind[c] == uint8_t(RigExecClothConstraint::Stretch)
                                    ? s.alphaStretch : s.alphaBend;
            Sub(x + 3 * a, x + 3 * b, d[k]);
            const float len = Length(d[k]);
            if (!(len > 0.0f)) { live[k] = 0; continue; }
            const float C = len - mesh.constraintRest[c];
            if constexpr (Model) {
                // The strength cache exists exactly when the model is
                // nonempty; without one every constraint keeps the default
                // alpha, the same verdicts a profile scan over no profiles
                // returned.
                if (!mesh.model.Empty()) {
                    const int32_t pi = mesh.constraintProfile[c];
                    if (pi >= 0) {
                        const RigExecClothProfile &profile = mesh.model.profiles[size_t(pi)];
                        const float *base = mesh.constraintStrengthBase.data() + 4 * c;
                        // A flat strength range needs no force estimate:
                        // the curve interpolates between equal endpoints,
                        // so the strength is the endpoint times the mask
                        // and the directional scalar, and the force
                        // quotient is dead. The +0.0f keeps the signed-zero
                        // behavior of the full form bit for bit; infinite
                        // endpoints and NaN take the full path.
                        float strength;
                        if (base[0] == base[1] && std::isfinite(base[0])) {
                            strength = (base[0] + 0.0f) * base[2] * base[3];
                        } else {
                            const float force = wa + wb > 0
                                ? std::abs(0.0f - C / (wa + wb)) / (s.h * s.h) : 0;
                            strength =
                                RigExecClothCachedBase(profile, base, force) * base[3];
                        }
                        if (!(strength > 0)) { live[k] = 0; continue; }
                        alpha = profile.hard ? 0 : profile.compliance / (strength * s.h * s.h);
                    }
                }
            }
            const float denom = wa + wb + alpha;
            if (!(denom > 0.0f)) { live[k] = 0; continue; }
            const float dl = -C / denom;
            ka[k] = wa * dl / len;
            kb[k] = wb * dl / len;
            live[k] = 1;
        }
        for (uint32_t k = 0; k < n; ++k) {
            if (!live[k]) continue;
            const size_t a = size_t(ends[k][0]);
            const size_t b = size_t(ends[k][1]);
            for (int q = 0; q < 3; ++q) {
                x[3 * a + size_t(q)] += d[k][q] * ka[k];
                x[3 * b + size_t(q)] -= d[k][q] * kb[k];
            }
        }
        base += n;
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
        const int32_t pi = mesh.constraintProfile[c];
        if (pi >= 0) {
            const RigExecClothProfile &profile = mesh.model.profiles[size_t(pi)];
            const float *base = mesh.constraintStrengthBase.data() + 4 * c;
            // A flat strength range needs no force estimate: the curve
            // interpolates between equal endpoints, so the strength is
            // the endpoint times the mask and the directional scalar,
            // and the force quotient is dead. The +0.0f keeps the
            // signed-zero behavior of the full form bit for bit;
            // infinite endpoints and NaN take the full path.
            float strength;
            if (base[0] == base[1] && std::isfinite(base[0])) {
                strength = (base[0] + 0.0f) * base[2] * base[3];
            } else {
                const float force = wa + wb > 0
                    ? std::abs(lambda[c] - C / (wa + wb)) / (s.h * s.h) : 0;
                strength = RigExecClothCachedBase(profile, base, force) * base[3];
            }
            if (!(strength > 0)) { lambda[c] = 0; return; }
            alpha = profile.hard ? 0 : profile.compliance / (strength * s.h * s.h);
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
/// goal (first iteration only), then collision with friction. \p broad,
/// when given, supplies the broadphase box for \p s's reach over \p grid,
/// derived once by the caller; a null pointer derives it per particle as
/// before. \p profileBroad, when given, supplies the same box over the
/// tick's pad reach for the profile path's per-collider search.
inline void
RigExecClothParticlePass(const RigExecClothMesh &mesh,
                         const RigExecClothColliders &colliders,
                         const RigExecClothGrid &grid,
                         const RigExecClothSettings &settings,
                         const RigExecClothStep &s, const float *from,
                         const float *to, const float *colliderFrom,
                         const float *colliderTo, bool firstIteration,
                         size_t i, float *x, const float *previous,
                         float *contactLambda = nullptr, int32_t *contactTriangles = nullptr,
                         const RigExecClothContactTris *contacts = nullptr,
                         const RigExecClothReachBounds *broad = nullptr,
                         const RigExecClothReachBounds *profileBroad = nullptr)
{
    using namespace cloth_detail;
    if (mesh.inverseMass[i] == 0.0f) return;
    float *pi = x + 3 * i;
    // Hoisted: every profile verdict below is uniformly false on an empty
    // model, so the per-particle lookups skip themselves.
    const bool modelEmpty = mesh.model.Empty();
    const int32_t anchor = mesh.tetherAnchor[i];
    if (anchor >= 0 && (modelEmpty || mesh.vertexTether[i] < 0)) {
        float d[3];
        Sub(pi, x + 3 * size_t(anchor), d);
        // The limit and its short-circuit bound arrive derived at build.
        // A tether at or below shortSquared needs no root: a 2^-12 band
        // under the squared limit, far outside any rounding between the
        // dot and the root, so the slack verdict is exact. Limits below
        // 1e-18 (whose square nears the subnormals), negative, and
        // squared-overflowing limits never skip; NaN and infinite
        // dots take the root path as before.
        const float limit = mesh.tetherLimit[i];
        const float shortSquared = mesh.tetherShortSquared[i];
        if (!(Dot(d, d) <= shortSquared)) {
            const float len = Length(d);
            if (len > limit && len > 0.0f) {
                const float k = (len - limit) / len;
                for (int a = 0; a < 3; ++a) pi[a] -= d[a] * k;
            }
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
    // The sample arrives as a pointer: the empty-model path passes null
    // and never reads it (no profile matches), so the per-particle sample
    // construction happens only where profiles exist.
    const auto solveContact = [&](int collider, const RigExecClothSample *sample,
                                    bool broadphaseKnownInside) {
    const RigExecClothProfile *collisionProfile = nullptr;
    if (!modelEmpty) {
        const int32_t pi = mesh.vertexCollision[size_t(collider) * mesh.pointCount + i];
        if (pi >= 0) collisionProfile = &mesh.model.profiles[size_t(pi)];
    }
    const float thickness = collisionProfile
        ? collisionProfile->thickness * RigExecClothMap(*collisionProfile, ClothThickness, i)
        : s.thickness;
    const int targetCollider = collider;
    const size_t twoSidedBegin = targetCollider >= 0 ? mesh.colliderOffsets[size_t(targetCollider)] : 0;
    const size_t twoSidedEnd = targetCollider >= 0 ? mesh.colliderOffsets[size_t(targetCollider) + 1] : colliders.pointCount;
    // The tick's prefilter boxes on the empty-model path: every search
    // there runs this step's thickness and depth, the reach the tick
    // block widened the boxes with. The profile path's reach varies per
    // particle, so it derives its boxes as before.
    const RigExecClothContact contact = RigExecClothFindContact(
        colliders, grid, colliderFrom, colliderTo, s.t, pi, thickness,
        s.depth, collisionProfile && collisionProfile->twoSided ? previous + 3 * i : nullptr,
        s.t0, twoSidedBegin, twoSidedEnd, collider >= 0,
        collisionProfile && collisionProfile->targetCollider >= 0 ? &collisionProfile->targetMask : nullptr,
        contacts, broadphaseKnownInside,
        modelEmpty ? grid.triFilter.data() : nullptr);
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
            const float strength = RigExecClothStrength(*profile, *sample, force, contact.normal, frame);
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
    if (modelEmpty) {
        // Hot path: without profiles the search below runs with this
        // step's thickness and depth, so its broadphase verdict is known
        // here. Nearly every particle is outside the swept bounds; those
        // skip the search call itself, bit-identically (the search would
        // return empty on the same verdict). The reach arrives derived
        // once per substep in the step, and the box over it arrives
        // derived once per substep in the bounds, the same doubles the
        // per-particle verdict formed.
        if (broad ? RigExecClothOutsideSweptReach(broad, pi)
                  : RigExecClothOutsideSweptBounds(grid, pi, s.reach))
            return;
        // The pre-check above tested this search's own box over the
        // search's own reach, so the search skips its re-derivation.
        solveContact(-1, nullptr, true);
        return;
    }
    // The profile path's search pre-check: the pad reach covers every
    // particle's collision reach (the tick's pad is the maximum over the
    // profiles and particles), so a particle outside the pad box is
    // outside every per-collider search's own broadphase verdict and each
    // search would return empty before any write. Particles inside take
    // the searches as before, with their own tighter boxes.
    if (profileBroad && RigExecClothOutsideSweptReach(profileBroad, pi)) return;
    const auto sample = RigExecClothVertex(i);
    for (size_t c = 0; c + 1 < mesh.colliderOffsets.size(); ++c)
        solveContact(int(c), &sample, false);
}

/// Velocity pass for particle \p i: the substep's displacement, damped.
inline void
RigExecClothVelocity(const RigExecClothMesh &mesh, const RigExecClothStep &s,
                     size_t i, const float *x, const float *previous,
                     float *velocity)
{
    if (mesh.inverseMass[i] == 0.0f) return;
    // Hoisted: an empty model matches no damping profile, so the
    // per-particle sample and profile lookup skip themselves here, as in
    // the particle pass. Profile rigs take the lookup as before.
    if (!mesh.model.Empty()) {
        const int32_t pi = mesh.vertexDamping[i];
        if (pi >= 0) {
            const RigExecClothProfile &profile = mesh.model.profiles[size_t(pi)];
            float frame[9], v[3], delta[3];
            RigExecClothMaterialFrame(mesh, x, i, frame);
            for (size_t a = 0; a < 3; ++a)
                v[a] = (x[3 * i + a] - previous[3 * i + a]) / s.h;
            const float *base = mesh.vertexDampingBase.data() + 6 * i;
            // A flat strength range needs no force estimate: the curve
            // interpolates between equal endpoints, so the isotropic part
            // is the endpoint times the mask, and the delta loop and its
            // root are dead. The +0.0f keeps the signed-zero behavior of
            // the full form bit for bit; infinite endpoints (an overflowed
            // product, where the full form subtracts to NaN) and NaN take
            // the full path. The same value the full path formed.
            float isotropic;
            if (base[0] == base[1] && std::isfinite(base[0])) {
                isotropic = (base[0] + 0.0f) * base[2];
            } else {
                for (size_t a = 0; a < 3; ++a)
                    delta[a] = (v[a] - velocity[3 * i + a]) / (s.h * mesh.inverseMass[i]);
                isotropic = RigExecClothCachedBase(
                    profile, base, cloth_detail::Length(delta));
            }
            // One sample, one force: the prepared isotropic part and warp
            // maps are the same for all three axes, so each axis re-forms
            // only its directional dots; the same rates the three full
            // strength evaluations formed.
            std::fill(velocity + 3 * i, velocity + 3 * i + 3, 0.0f);
            for (size_t a = 0; a < 3; ++a) {
                const float rate = isotropic * RigExecClothDirectional(profile, frame + 3 * a,
                                                                       frame, base + 3);
                const float damp = profile.hard && rate > 0 ? 0 : std::exp(-rate * s.h);
                const float along = cloth_detail::Dot(v, frame + 3 * a) * damp;
                for (size_t q = 0; q < 3; ++q) velocity[3 * i + q] += along * frame[3 * a + q];
            }
            return;
        }
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
    s.reach = std::max(s.thickness, s.depth);
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

namespace cloth_detail {

// The midpoint sphere of an edge, cached until a correction moves an
// endpoint. The caller passes the endpoints it already holds; the slot is
// the constraint index, or past the constraints for a triangle edge. A
// stamp from the current clock computed after every touch so far, so it
// covers this edge's touches too; otherwise the endpoint ticks decide.
// The clock arrives as the caller's held tick: it only advances in
// TouchVerts beside a correction, so the caller refreshes one copy at
// each touch instead of every fetch reloading it, with the same verdicts.
inline void EdgeSphere(RigExecClothEdgeSpheres *spheres, size_t slot, int32_t u,
                       int32_t v, const float *x, float *center, float *radius,
                       uint64_t now)
{
    const uint64_t stamp = spheres->edgeTick[slot];
    if (stamp == now || (stamp >= spheres->vertTick[size_t(u)] &&
                         stamp >= spheres->vertTick[size_t(v)])) {
        center[0] = spheres->centers[3 * slot];
        center[1] = spheres->centers[3 * slot + 1];
        center[2] = spheres->centers[3 * slot + 2];
        *radius = spheres->radii[slot];
        return;
    }
    const float *a = x + 3 * size_t(u), *b = x + 3 * size_t(v);
    center[0] = (a[0] + b[0]) * 0.5f;
    center[1] = (a[1] + b[1]) * 0.5f;
    center[2] = (a[2] + b[2]) * 0.5f;
    const float e0 = b[0] - a[0], e1 = b[1] - a[1], e2 = b[2] - a[2];
    *radius = 0.5f * std::sqrt(e0 * e0 + e1 * e1 + e2 * e2);
    spheres->centers[3 * slot] = center[0];
    spheres->centers[3 * slot + 1] = center[1];
    spheres->centers[3 * slot + 2] = center[2];
    spheres->radii[slot] = *radius;
    spheres->edgeTick[slot] = now;
}

// The centroid sphere of a triangle, cached until a correction moves a
// corner. Freshness reuses the edge spheres' ticks: the caller passes
// the corners it already holds. A stamp from the current clock covers
// every corner's ticks; otherwise the three tick reads decide. The clock
// arrives as the caller's held tick, refreshed at each touch.
inline void TriSphere(RigExecClothTriSpheres *tris, RigExecClothEdgeSpheres *spheres,
                      size_t tri, int32_t u, int32_t v, int32_t w,
                      const float *x, float *center, float *radius,
                      uint64_t now)
{
    const uint64_t stamp = tris->triTick[tri];
    if (stamp == now || (stamp >= spheres->vertTick[size_t(u)] &&
                         stamp >= spheres->vertTick[size_t(v)] &&
                         stamp >= spheres->vertTick[size_t(w)])) {
        center[0] = tris->centers[3 * tri];
        center[1] = tris->centers[3 * tri + 1];
        center[2] = tris->centers[3 * tri + 2];
        *radius = tris->radii[tri];
        return;
    }
    const float *a = x + 3 * size_t(u), *b = x + 3 * size_t(v), *c = x + 3 * size_t(w);
    center[0] = (a[0] + b[0] + c[0]) / 3.0f;
    center[1] = (a[1] + b[1] + c[1]) / 3.0f;
    center[2] = (a[2] + b[2] + c[2]) / 3.0f;
    const float d0 = a[0] - center[0], d1 = a[1] - center[1], d2 = a[2] - center[2];
    const float e0 = b[0] - center[0], e1 = b[1] - center[1], e2 = b[2] - center[2];
    const float f0 = c[0] - center[0], f1 = c[1] - center[1], f2 = c[2] - center[2];
    *radius = std::sqrt(std::max(d0 * d0 + d1 * d1 + d2 * d2,
                                 std::max(e0 * e0 + e1 * e1 + e2 * e2,
                                          f0 * f0 + f1 * f1 + f2 * f2)));
    tris->centers[3 * tri] = center[0];
    tris->centers[3 * tri + 1] = center[1];
    tris->centers[3 * tri + 2] = center[2];
    tris->radii[tri] = *radius;
    tris->triTick[tri] = now;
}

// Retires every cached sphere over four corrected vertices, after the
// writes. No sphere is computed between a pair's projection and its
// friction writes to the same vertices, so one touch covers both.
// Returns the advanced tick so the caller refreshes its held copy.
inline uint64_t TouchVerts(RigExecClothEdgeSpheres *spheres, int32_t a, int32_t b,
                           int32_t c, int32_t d)
{
    const uint64_t tick = ++spheres->clock;
    spheres->vertTick[size_t(a)] = tick;
    spheres->vertTick[size_t(b)] = tick;
    spheres->vertTick[size_t(c)] = tick;
    spheres->vertTick[size_t(d)] = tick;
    return tick;
}

} // namespace cloth_detail

/// True when the vertex/triangle self grid is still valid for the edge/edge
/// pass: no projection has moved x since it was built (the sphere clock only
/// advances in TouchVerts, beside each projection) and the edge reach fits
/// inside the vertex/triangle pad, so the edge/edge grid would take identical
/// inputs and build identical buckets. A NaN reach reuses: std::max keeps the
/// pad then, so the inputs still match.
inline bool RigExecClothSelfGridReusable(uint64_t clock, uint64_t gridClock,
                                         float edgeReach, float pad)
{
    return clock == gridClock && !(edgeReach > pad);
}

/// Two-sided vertex/triangle contact between selected cloth regions. The
/// broad phase is rebuilt inside the existing constraint iteration; all
/// corrections act on the same private particle arena as stretch and bend.
inline void RigExecClothSelfContacts(const RigExecClothMesh &mesh,
    const RigExecClothStep &step, const float *previous,
    RigExecClothWorkspace *ws, float *x)
{
    using namespace cloth_detail;
    const float pad = mesh.selfPad;
    if (!(pad > 0)) return;
    // Size the sphere cache once per mesh; a fresh clock per call retires
    // every sphere, since positions moved since the last call.
    RigExecClothEdgeSpheres &spheres = ws->selfSpheres;
    const size_t sphereEdges = mesh.constraintRest.size() + mesh.triangles.size();
    if (spheres.centers.size() != 3 * sphereEdges ||
        spheres.vertTick.size() != mesh.pointCount) {
        spheres.centers.assign(3 * sphereEdges, 0.0f);
        spheres.radii.assign(sphereEdges, 0.0f);
        spheres.edgeTick.assign(sphereEdges, 0);
        spheres.vertTick.assign(mesh.pointCount, 0);
    }
    std::fill(spheres.vertTick.begin(), spheres.vertTick.end(), ++spheres.clock);
    // The held sphere tick: the clock only advances in TouchVerts beside
    // a correction, so every freshness check below reads this copy and
    // each touch refreshes it; the same values fresh loads would read.
    uint64_t tickNow = spheres.clock;
    RigExecClothTriSpheres &triSpheres = ws->selfTriSpheres;
    const size_t sphereTris = mesh.triangles.size() / 3;
    if (triSpheres.centers.size() != 3 * sphereTris) {
        triSpheres.centers.assign(3 * sphereTris, 0.0f);
        triSpheres.radii.assign(sphereTris, 0.0f);
        triSpheres.triTick.assign(sphereTris, 0);
    }
    // The triangle buffer persists in the workspace: assignment reuses it
    // instead of allocating per call, and a model without a running self
    // profile never copies it. Same values.
    RigExecClothColliders &topology = ws->selfTopology;
    topology.pointCount = mesh.pointCount;
    topology.triangles = mesh.triangles;
    // A padded triangle bucket at an edge midpoint covers every possible
    // edge/edge encounter along that edge, not just its endpoints.
    const auto measureReach = [&]() {
        // One square root for the max half-length: sqrt is monotonic, so
        // the max of the roots is the root of the max, the same float.
        float maxD2 = 0;
        for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
            if (mesh.constraintKind[c] != uint8_t(RigExecClothConstraint::Stretch)) continue;
            float d[3];
            Sub(x + 3 * size_t(mesh.constraintEnds[2*c]), x + 3 * size_t(mesh.constraintEnds[2*c+1]), d);
            maxD2 = std::max(maxD2, Dot(d, d));
        }
        return 0.5f * std::sqrt(maxD2);
    };
    const float edgeReach = measureReach();
    // Loop-invariant mesh extents, held once per call: the same values
    // the discovery loops re-derive per iteration.
    const size_t constraintCount = mesh.constraintRest.size();
    // The pair bitset serves while the constraint count fits under its
    // cap; past it the twin skip and hash sets serve instead. The word
    // count multiplies only under the cap, so no size overflows.
    const bool usePairBits = RigExecClothUsePairBits(constraintCount);
    const size_t pairCount =
        usePairBits ? constraintCount * (constraintCount + 1) / 2 : 0;
    const size_t pairWords = (pairCount + 63) / 64;
    // The triangle band's reach bound: every triangle edge is a stretch
    // constraint, so the stretch half-length maximum covers each triangle
    // edge's sphere radius. Re-derived when vertex/triangle corrections
    // moved x since it was measured; the sphere clock only advances in
    // TouchVerts, beside each projection.
    float triReach = edgeReach;
    uint64_t triReachClock = spheres.clock;
    RigExecBuildClothGrid(topology, previous, x, std::max(pad, step.depth),
                          &ws->selfGrid, &ws->selfScratch);
    const auto &grid = ws->selfGrid;
    // While the vertex/triangle grid stays valid for the edge/edge pass,
    // the edge/edge discovery rereads it as is instead of rebuilding it.
    const uint64_t selfGridClock = spheres.clock;
    // Bitset adjacency where the mesh built the sets, the linear scans
    // past the size cap: the same verdicts either way.
    const bool useBits = !mesh.neighborBits.empty();
    const bool useEdgeBits = !mesh.edgeNeighborBits.empty();
    // The twin-skip bucket marks, one per triangle, cleared per call;
    // the mark runs across profiles so every constraint's mark is new.
    std::vector<uint32_t> &twinStamp = ws->selfTwinStamp;
    if (!usePairBits) twinStamp.assign(mesh.triangles.size() / 3, 0);
    uint32_t twinMark = 0;
    for (size_t profileIndex = 0; profileIndex < mesh.model.profiles.size(); ++profileIndex) {
        const auto &p = mesh.model.profiles[profileIndex];
        if (p.kind != RigExecClothProfileKind::SelfCollision) continue;
        // The bucket bases stay fixed across the vertex loop: the only
        // rebuild sits between this profile's vertex loop and its edge
        // loop, so the entry loop rereads the same bases without
        // reloading them per entry.
        const uint32_t *vtBucketBounds = grid.bucketStart.data();
        const int32_t *vtBucketTris = grid.bucketTriangles.data();
        // The mesh topology bases stay fixed across the discovery loops:
        // the mesh is compiled input, so the edge/edge entries reread the
        // same bases without reloading them per entry.
        // The triangle union-row base and stride stay fixed too: every
        // row base is its triangle times the stride, so the
        // vertex/triangle adjacency reads the row directly instead of
        // loading the base per entry, the same word either way.
        const uint64_t *vtTriBase = mesh.triNeighborBits.data();
        const size_t vtStride = mesh.neighborBitStride;
        const int32_t *meshTriangles = mesh.triangles.data();
        const int32_t *meshTriEdges = mesh.triEdges.data();
        const int32_t *meshTriTwin = mesh.triEdgeTwin.data();
        const int32_t *meshTriEdgeConstraint = mesh.triEdgeConstraint.data();
        const int32_t *meshConstraintEnds = mesh.constraintEnds.data();
        const uint8_t *meshConstraintKind = mesh.constraintKind.data();
        // The triangle-sphere bases stay fixed across both discovery
        // loops: the cache is sized per call and only assigned within
        // it, so the bands below reread the same bases without
        // reloading them per entry.
        const float *triCentersBase = triSpheres.centers.data();
        const float *triRadiiBase = triSpheres.radii.data();
        // Bulk-refresh every triangle sphere for the bands below: the
        // refresh computes exactly the stale spheres through the same
        // freshness logic the per-entry calls ran, stamping the held
        // tick, so both bands read them directly with no per-entry
        // tick check. The same values and stamps. A vertex/triangle
        // correction retires spheres, so the first one falls back to
        // checked fetches and re-validates ahead of the edge/edge pass.
        for (size_t bulkTri = 0; bulkTri < sphereTris; ++bulkTri) {
            float bulkC[3], bulkR;
            TriSphere(&triSpheres, &spheres, bulkTri, meshTriangles[3 * bulkTri],
                      meshTriangles[3 * bulkTri + 1], meshTriangles[3 * bulkTri + 2],
                      x, bulkC, &bulkR, tickNow);
        }
        bool vtBandDirect = true;
        for (size_t i = 0; i < mesh.pointCount; ++i) {
            const auto a = RigExecClothVertex(i);
            // Either region may supply the vertex; contact is symmetric.
            const float source = RigExecClothRegion(p.sourceMask, a);
            const float target = RigExecClothRegion(p.targetMask, a);
            if (!(source > 0 || target > 0)) continue;
            const float thickness = p.thickness * RigExecClothMap(p, ClothThickness, i);
            // The band and distance limits are fixed for the vertex: the
            // same values the entry loop recomputes, derived once.
            const float depthLimit = std::max(thickness, step.depth);
            const float bandLimit = std::max(depthLimit, 0.0f);
            const float *point = x + 3 * i;
            // The vertex's union-row word and bit stay fixed across its
            // bucket entries: the same shifts the per-entry test forms.
            const size_t vtWord = i >> 6;
            const uint64_t vtMask = 1ull << (i & 63);
            const uint32_t bucket = Hash(Cell(point[0], grid.cellSize), Cell(point[1], grid.cellSize),
                                          Cell(point[2], grid.cellSize), grid.bucketMask);
            for (uint32_t entry = vtBucketBounds[bucket]; entry < vtBucketBounds[bucket + 1]; ++entry) {
                const int32_t tri = vtBucketTris[entry];
                RigExecClothSample b;
                for (size_t q = 0; q < 3; ++q)
                    b.vertices[q] = mesh.triangles[3 * size_t(tri) + q];
                // The triangle union row holds the vertices neighboring any
                // corner, so one bit test answers the three corner tests;
                // adjacency is symmetric, the same verdicts. Past the size
                // cap the linear scans keep their order.
                bool adjacent;
                if (useBits)
                    adjacent =
                        (vtTriBase[size_t(tri) * vtStride + vtWord] & vtMask) != 0;
                else {
                    adjacent = false;
                    for (size_t q = 0; q < 3; ++q)
                        adjacent = adjacent || AdjacentTo(mesh.neighbors[i], b.vertices[q]);
                }
                if (adjacent) continue;
                // The triangle-sphere band: the centroid holds every
                // triangle point within the radius, so a point further
                // than radius plus reach from the centroid dies at the
                // distance test below. The 2^-10 margin is far outside
                // float rounding, a NaN input compares false and skips
                // nothing, and clamping a negative reach at zero stays
                // conservative. Skips the barycentric weights.
                float tc[3], tr;
                if (vtBandDirect) {
                    tc[0] = triCentersBase[3 * size_t(tri)];
                    tc[1] = triCentersBase[3 * size_t(tri) + 1];
                    tc[2] = triCentersBase[3 * size_t(tri) + 2];
                    tr = triRadiiBase[size_t(tri)];
                } else {
                    TriSphere(&triSpheres, &spheres, size_t(tri), b.vertices[0],
                              b.vertices[1], b.vertices[2], x, tc, &tr, tickNow);
                }
                const float reach = tr + bandLimit;
                const float dd0 = point[0] - tc[0], dd1 = point[1] - tc[1],
                            dd2 = point[2] - tc[2];
                if (dd0 * dd0 + dd1 * dd1 + dd2 * dd2 > reach * reach * 1.0009765625f)
                    continue;
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
                if (distance > depthLimit) continue;
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
                tickNow = TouchVerts(&spheres, int32_t(i), b.vertices[0], b.vertices[1], b.vertices[2]);
                vtBandDirect = false;
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
        if (!RigExecClothSelfGridReusable(spheres.clock, selfGridClock, edgeReach,
                                          std::max(pad, step.depth)))
            RigExecBuildClothGrid(topology, previous, x, std::max(std::max(pad, step.depth), edgeReach),
                                  &ws->selfGrid, &ws->selfScratch);
        if (spheres.clock != triReachClock) {
            triReach = measureReach();
            triReachClock = spheres.clock;
        }
        // The triangle band stays on until an edge/edge correction moves
        // x, which could grow a triangle edge past the reach bound.
        bool triBandOn = true;
        // Flat dedup over retained storage: the same verdicts as an
        // ordered set (the discovery loops keep their order), with no
        // per-pair allocation.
        RigExecClothPairSet &pairs = ws->selfPairs;
        RigExecClothPairSet64 &pairs64 = ws->selfPairs64;
        std::vector<uint64_t> &pairBits = ws->selfPairBits;
        // Packed keys where every endpoint fits 16 bits, the array set
        // past that: packing is injective on packed meshes, so both sets
        // reach the same verdicts on the same keys.
        const bool packPairs = RigExecClothPairSet64::Packable(mesh.pointCount);
        if (usePairBits) pairBits.assign(pairWords, 0);
        else if (packPairs) pairs64.Clear();
        else pairs.Clear();
        // The pair-bitset base stays fixed across the edge/edge pass:
        // the profile cleared it above and the discovery only flips
        // bits, so the edge tests reread the same base without
        // reloading it per entry.
        uint64_t *eePairBase = usePairBits ? pairBits.data() : nullptr;
        // The vertex-loop bulk above stamped every sphere with the held
        // tick; only a vertex/triangle correction retires any, so only
        // then does the edge/edge pass re-validate them. The same values
        // and stamps either way: without a correction the refresh would
        // only restamp the same tick.
        if (!vtBandDirect) {
            for (size_t bulkTri = 0; bulkTri < sphereTris; ++bulkTri) {
                float bulkC[3], bulkR;
                TriSphere(&triSpheres, &spheres, bulkTri, meshTriangles[3 * bulkTri],
                          meshTriangles[3 * bulkTri + 1], meshTriangles[3 * bulkTri + 2],
                          x, bulkC, &bulkR, tickNow);
            }
        }
        for (size_t c = 0; c < constraintCount; ++c) {
            if (meshConstraintKind[c] != uint8_t(RigExecClothConstraint::Stretch)) continue;
            const int32_t i = meshConstraintEnds[2*c], j = meshConstraintEnds[2*c+1];
            float mid[3]; Lerp(x + 3*size_t(i), x + 3*size_t(j), .5f, mid);
            const uint32_t bucket = Hash(Cell(mid[0], grid.cellSize), Cell(mid[1], grid.cellSize),
                                          Cell(mid[2], grid.cellSize), grid.bucketMask);
            if (!usePairBits) ++twinMark;
            // The bucket, twin, and adjacency bases stay fixed across
            // the discovery batch: the grid rebuilds between passes
            // only, the twin marks store without reallocating, and the
            // constraint's union row is fixed here, so the entry loop
            // rereads the same bases without reloading them per entry.
            const uint32_t *eeBucketBounds = grid.bucketStart.data();
            const int32_t *eeBucketTris = grid.bucketTriangles.data();
            uint32_t *eeTwinBase = twinStamp.data();
            const uint64_t *eeAdjRow = useEdgeBits
                ? mesh.edgeNeighborBits.data() + c * mesh.neighborBitStride
                : nullptr;
            // One load check per constraint covers its whole discovery
            // batch: at most three inserts per bucket entry, so the
            // table stays under the cap through the batch and the
            // inserts below skip their checks with the same verdicts.
            // The bitset path records into pre-cleared words instead.
            if (!usePairBits) {
                const size_t batchKeys =
                    3 * (eeBucketBounds[bucket + 1] - eeBucketBounds[bucket]);
                if (packPairs) pairs64.ReserveForBatch(batchKeys);
                else pairs.ReserveForBatch(batchKeys);
            }
            // The constraint edge's band inputs are fixed for the bucket
            // loop: its sphere and thickness factor derive once, lazily
            // at the first band test, and a correction retires the
            // sphere beside its touch. The same values the per-pair
            // derivation reads, without re-reading them per pair. The
            // reach bases fold the fixed addends once: reassociating
            // the band sums moves each by a few ulps, far inside the
            // 2^-10 margin, so every skip stays conservative and the
            // exact tails decide the same corrections.
            float heldC1[3] = {0, 0, 0}, heldR1 = 0;
            const float heldThick = std::fabs(p.thickness) * std::max(
                std::fabs(RigExecClothMap(p, ClothThickness, size_t(i))),
                std::fabs(RigExecClothMap(p, ClothThickness, size_t(j))));
            bool haveHeld = false;
            float triBandBase = 0, pairBandBase = 0;
            for (uint32_t entry = eeBucketBounds[bucket]; entry < eeBucketBounds[bucket+1]; ++entry) {
                const size_t tri = size_t(eeBucketTris[entry]);
                // The twin-skip mark lands with the visit: buckets list
                // triangles ascending, so a twin below this triangle is
                // already marked when the edge loop reads it, and the
                // same marks are set without a second bucket pass. The
                // bitset path records visits per pair instead, with no
                // per-entry mark.
                if (!usePairBits) eeTwinBase[tri] = twinMark;
                // The triangle band: the centroid sphere holds every edge
                // midpoint of the triangle within its radius, and the reach
                // bound covers every triangle edge's sphere radius, so a
                // separated triangle separates all three edge pairs and the
                // per-pair band below would skip each of them. Same skips,
                // decided once per triangle; a NaN input compares false and
                // skips nothing.
                if (triBandOn) {
                    if (!haveHeld) {
                        EdgeSphere(&spheres, c, i, j, x, heldC1, &heldR1, tickNow);
                        triBandBase = heldR1 + triReach + heldThick;
                        pairBandBase = heldR1 + heldThick;
                        haveHeld = true;
                    }
                    // Direct read over the bulk-refreshed spheres: the
                    // refresh ahead of the pass stamped every sphere with
                    // the held tick, and the band switches off beside the
                    // first correction that could retire them. The same
                    // values the checked fetch returned.
                    float ttc[3], ttr;
                    ttc[0] = triCentersBase[3*tri];
                    ttc[1] = triCentersBase[3*tri+1];
                    ttc[2] = triCentersBase[3*tri+2];
                    ttr = triRadiiBase[tri];
                    const float td0 = heldC1[0] - ttc[0], td1 = heldC1[1] - ttc[1],
                                td2 = heldC1[2] - ttc[2];
                    const float trr = ttr + triBandBase;
                    if (td0 * td0 + td1 * td1 + td2 * td2 > trr * trr * 1.0009765625f)
                        continue;
                }
                // Unrolled over the triangle's three edges: the body is too
                // large for the compiler's unroller, so the rolled loop
                // serializes the three edge bodies' independent chains
                // (adjacency tests, twin loads, dedup hashes, sphere
                // fetches). The three blocks are identical except for the
                // edge numeral and run in the same order as the rolled
                // loop, and do/while plus break preserves its continue
                // semantics, so discovery, dedup, and mutation order are
                // unchanged.
                do { // Triangle edge 0.
                    const int32_t k = meshTriEdges[6*tri];
                    const int32_t l = meshTriEdges[6*tri+1];
                    // Adjacency first where the mesh built the union rows: every
                    // neighbor list holds its own vertex, so the constraint
                    // row holds both endpoints and every endpoint-sharing
                    // pair tests adjacent. The share test would skip nothing
                    // more, and the twin skip and the dedup order see the
                    // same survivors in the same order, with the same
                    // verdicts. Past the size cap the share test stays ahead
                    // of the insert, and the linear scans stay behind it so
                    // a rediscovered pair still skips them.
                    if (useEdgeBits) {
                        if (AdjacentToRowBits(eeAdjRow, k) ||
                            AdjacentToRowBits(eeAdjRow, l)) break;
                    }
                    else if (i == k || i == l || j == k || j == l) break;
                    if (usePairBits) {
                        // One bit per constraint pair, triangular with its
                        // diagonal: both edges are stretch constraints, so
                        // the unordered pair is one index, and the verdict
                        // is exact set membership like the twin skip and
                        // hash sets below, in discovery order.
                        const size_t o2 = size_t(meshTriEdgeConstraint[3*tri]);
                        const size_t hi = c > o2 ? c : o2;
                        const size_t lo = c > o2 ? o2 : c;
                        const size_t bit = hi * (hi + 1) / 2 + lo;
                        const uint64_t mask = 1ull << (bit & 63);
                        uint64_t &slot = eePairBase[bit >> 6];
                        if (slot & mask) break;
                        slot |= mask;
                    } else {
                        // Twin skip: the edge's smallest twin triangle sits
                        // earlier in this ascending bucket, so the same pair
                        // was already visited from this constraint and the
                        // dedup set holds its key.
                        const int32_t twin = meshTriTwin[3*tri];
                        if (twin >= 0 && twin < int32_t(tri) &&
                            eeTwinBase[size_t(twin)] == twinMark) break;
                        // Both edges arrive sorted with four distinct
                        // endpoints, so the pair order is one compare, and
                        // the insert hashes the ordered endpoints directly
                        // instead of packing and unpacking them first.
                        if (packPairs) {
                            if (k < i) {
                                if (!pairs64.InsertNoGrowFields(k, l, i, j)) break;
                            } else {
                                if (!pairs64.InsertNoGrowFields(i, j, k, l)) break;
                            }
                        } else {
                            std::array<int32_t,4> key{{i,j,k,l}};
                            if (k < i) key = {{k,l,i,j}};
                            if (!pairs.InsertNoGrow(key)) break;
                        }
                    }
                    if (!useEdgeBits) {
                        bool adjacent = false;
                        for (int32_t a : {i,j}) for (int32_t b : {k,l}) {
                            adjacent = adjacent || (useBits ? AdjacentToBits(mesh, a, b)
                                                            : AdjacentTo(mesh.neighbors[size_t(a)], b));
                        }
                        if (adjacent) break;
                    }
                    // The edge-sphere band: a pair whose midpoint spheres
                    // plus contact reach cannot meet skips the
                    // closest-segment tail. Each edge holds its points
                    // within half its length of its midpoint, so separated
                    // spheres imply a separated pair; the 2^-10 margin is far
                    // outside float rounding, and a NaN input compares
                    // false and skips nothing. ClosestSegments clamps its
                    // parameters into [0,1], so the sample map is a convex
                    // combination of the endpoint maps and the absolute
                    // bound covers every thickness sign.
                    if (!haveHeld) {
                        EdgeSphere(&spheres, c, i, j, x, heldC1, &heldR1, tickNow);
                        triBandBase = heldR1 + triReach + heldThick;
                        pairBandBase = heldR1 + heldThick;
                        haveHeld = true;
                    }
                    float c2[3], r2;
                    EdgeSphere(&spheres, constraintCount + 3 * tri,
                               k, l, x, c2, &r2, tickNow);
                    const float dd0 = heldC1[0] - c2[0], dd1 = heldC1[1] - c2[1],
                                dd2 = heldC1[2] - c2[2];
                    const float reach = pairBandBase + r2;
                    if (dd0 * dd0 + dd1 * dd1 + dd2 * dd2 >
                        reach * reach * 1.0009765625f)
                        break;
                    float u, v;
                    ClosestSegments(x+3*size_t(i), x+3*size_t(j), x+3*size_t(k), x+3*size_t(l), &u, &v);
                    RigExecClothSample a, b;
                    a.vertices = {{i,j,j}}; a.weights = {{1-u,u,0}};
                    b.vertices = {{k,l,l}}; b.weights = {{1-v,v,0}};
                    if (!(RigExecClothRegion(p.sourceMask,a) * RigExecClothRegion(p.targetMask,b) > 0 ||
                          RigExecClothRegion(p.targetMask,a) * RigExecClothRegion(p.sourceMask,b) > 0)) break;
                    float pa[3], pb[3], d[3], oldA[3], oldB[3], oldD[3];
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb); Sub(pa,pb,d);
                    const float distance = Length(d);
                    const float thickness = p.thickness * RigExecClothSampleMap(p,ClothThickness,a);
                    if (distance >= thickness) break;
                    if (distance > 1e-10f) for (float &q : d) q /= distance;
                    else {
                        float e1[3], e2[3];
                        Sub(x+3*size_t(j),x+3*size_t(i),e1); Sub(x+3*size_t(l),x+3*size_t(k),e2);
                        Cross(e1,e2,d); const float len = Length(d);
                        if (!(len > 1e-10f)) break;
                        for (float &q : d) q /= len;
                    }
                    RigExecClothSamplePoint(a,previous,oldA); RigExecClothSamplePoint(b,previous,oldB);
                    Sub(oldA,oldB,oldD);
                    if (Dot(oldD,d) < 0) for (float &q : d) q = -q;
                    Sub(pa,pb,mid);
                    // The dedup key, rebuilt for the impulse map: the same
                    // normalized endpoints the discovery keyed on, whether
                    // packed or array.
                    std::array<int32_t,4> lkey{{i,j,k,l}};
                    if (k < i) lkey = {{k,l,i,j}};
                    float &lambda = ws->selfLambda[{{int32_t(profileIndex),lkey[0],lkey[1],lkey[2],lkey[3]}}];
                    const float correction = RigExecClothProjectSamples(mesh,step,p,a,&b,
                        Dot(mid,d)-thickness,d,true,&lambda,x,false);
                    tickNow = TouchVerts(&spheres, i, j, k, l);
                    haveHeld = false;
                    triBandOn = false;
                    const float mu = p.friction * RigExecClothSampleMap(p,ClothFriction,a);
                    if (!(correction > 0 && mu > 0)) break;
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb);
                    float tangent[3];
                    for (size_t q = 0; q < 3; ++q) tangent[q] = (pa[q]-oldA[q])-(pb[q]-oldB[q]);
                    const float along = Dot(tangent,d);
                    for (size_t q = 0; q < 3; ++q) tangent[q] -= along*d[q];
                    const float slide = Length(tangent);
                    if (!(slide > 1e-12f)) break;
                    float mass = 0;
                    for (size_t q = 0; q < 3; ++q)
                        mass += mesh.inverseMass[size_t(a.vertices[q])]*a.weights[q]*a.weights[q] +
                                mesh.inverseMass[size_t(b.vertices[q])]*b.weights[q]*b.weights[q];
                    if (!(mass > 0)) break;
                    const float impulse = std::min(slide,mu*correction)/(mass*slide);
                    for (size_t q = 0; q < 3; ++q) for (size_t axis = 0; axis < 3; ++axis) {
                        x[3*size_t(a.vertices[q])+axis] -= mesh.inverseMass[size_t(a.vertices[q])]*
                            a.weights[q]*impulse*tangent[axis];
                        x[3*size_t(b.vertices[q])+axis] += mesh.inverseMass[size_t(b.vertices[q])]*
                            b.weights[q]*impulse*tangent[axis];
                    }
                } while (0);
                do { // Triangle edge 1.
                    const int32_t k = meshTriEdges[6*tri+2];
                    const int32_t l = meshTriEdges[6*tri+3];
                    // Adjacency first where the mesh built the union rows: every
                    // neighbor list holds its own vertex, so the constraint
                    // row holds both endpoints and every endpoint-sharing
                    // pair tests adjacent. The share test would skip nothing
                    // more, and the twin skip and the dedup order see the
                    // same survivors in the same order, with the same
                    // verdicts. Past the size cap the share test stays ahead
                    // of the insert, and the linear scans stay behind it so
                    // a rediscovered pair still skips them.
                    if (useEdgeBits) {
                        if (AdjacentToRowBits(eeAdjRow, k) ||
                            AdjacentToRowBits(eeAdjRow, l)) break;
                    }
                    else if (i == k || i == l || j == k || j == l) break;
                    if (usePairBits) {
                        // One bit per constraint pair, triangular with its
                        // diagonal: both edges are stretch constraints, so
                        // the unordered pair is one index, and the verdict
                        // is exact set membership like the twin skip and
                        // hash sets below, in discovery order.
                        const size_t o2 = size_t(meshTriEdgeConstraint[3*tri+1]);
                        const size_t hi = c > o2 ? c : o2;
                        const size_t lo = c > o2 ? o2 : c;
                        const size_t bit = hi * (hi + 1) / 2 + lo;
                        const uint64_t mask = 1ull << (bit & 63);
                        uint64_t &slot = eePairBase[bit >> 6];
                        if (slot & mask) break;
                        slot |= mask;
                    } else {
                        // Twin skip: the edge's smallest twin triangle sits
                        // earlier in this ascending bucket, so the same pair
                        // was already visited from this constraint and the
                        // dedup set holds its key.
                        const int32_t twin = meshTriTwin[3*tri+1];
                        if (twin >= 0 && twin < int32_t(tri) &&
                            eeTwinBase[size_t(twin)] == twinMark) break;
                        // Both edges arrive sorted with four distinct
                        // endpoints, so the pair order is one compare, and
                        // the insert hashes the ordered endpoints directly
                        // instead of packing and unpacking them first.
                        if (packPairs) {
                            if (k < i) {
                                if (!pairs64.InsertNoGrowFields(k, l, i, j)) break;
                            } else {
                                if (!pairs64.InsertNoGrowFields(i, j, k, l)) break;
                            }
                        } else {
                            std::array<int32_t,4> key{{i,j,k,l}};
                            if (k < i) key = {{k,l,i,j}};
                            if (!pairs.InsertNoGrow(key)) break;
                        }
                    }
                    if (!useEdgeBits) {
                        bool adjacent = false;
                        for (int32_t a : {i,j}) for (int32_t b : {k,l}) {
                            adjacent = adjacent || (useBits ? AdjacentToBits(mesh, a, b)
                                                            : AdjacentTo(mesh.neighbors[size_t(a)], b));
                        }
                        if (adjacent) break;
                    }
                    // The edge-sphere band: a pair whose midpoint spheres
                    // plus contact reach cannot meet skips the
                    // closest-segment tail. Each edge holds its points
                    // within half its length of its midpoint, so separated
                    // spheres imply a separated pair; the 2^-10 margin is far
                    // outside float rounding, and a NaN input compares
                    // false and skips nothing. ClosestSegments clamps its
                    // parameters into [0,1], so the sample map is a convex
                    // combination of the endpoint maps and the absolute
                    // bound covers every thickness sign.
                    if (!haveHeld) {
                        EdgeSphere(&spheres, c, i, j, x, heldC1, &heldR1, tickNow);
                        triBandBase = heldR1 + triReach + heldThick;
                        pairBandBase = heldR1 + heldThick;
                        haveHeld = true;
                    }
                    float c2[3], r2;
                    EdgeSphere(&spheres, constraintCount + 3 * tri + 1,
                               k, l, x, c2, &r2, tickNow);
                    const float dd0 = heldC1[0] - c2[0], dd1 = heldC1[1] - c2[1],
                                dd2 = heldC1[2] - c2[2];
                    const float reach = pairBandBase + r2;
                    if (dd0 * dd0 + dd1 * dd1 + dd2 * dd2 >
                        reach * reach * 1.0009765625f)
                        break;
                    float u, v;
                    ClosestSegments(x+3*size_t(i), x+3*size_t(j), x+3*size_t(k), x+3*size_t(l), &u, &v);
                    RigExecClothSample a, b;
                    a.vertices = {{i,j,j}}; a.weights = {{1-u,u,0}};
                    b.vertices = {{k,l,l}}; b.weights = {{1-v,v,0}};
                    if (!(RigExecClothRegion(p.sourceMask,a) * RigExecClothRegion(p.targetMask,b) > 0 ||
                          RigExecClothRegion(p.targetMask,a) * RigExecClothRegion(p.sourceMask,b) > 0)) break;
                    float pa[3], pb[3], d[3], oldA[3], oldB[3], oldD[3];
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb); Sub(pa,pb,d);
                    const float distance = Length(d);
                    const float thickness = p.thickness * RigExecClothSampleMap(p,ClothThickness,a);
                    if (distance >= thickness) break;
                    if (distance > 1e-10f) for (float &q : d) q /= distance;
                    else {
                        float e1[3], e2[3];
                        Sub(x+3*size_t(j),x+3*size_t(i),e1); Sub(x+3*size_t(l),x+3*size_t(k),e2);
                        Cross(e1,e2,d); const float len = Length(d);
                        if (!(len > 1e-10f)) break;
                        for (float &q : d) q /= len;
                    }
                    RigExecClothSamplePoint(a,previous,oldA); RigExecClothSamplePoint(b,previous,oldB);
                    Sub(oldA,oldB,oldD);
                    if (Dot(oldD,d) < 0) for (float &q : d) q = -q;
                    Sub(pa,pb,mid);
                    // The dedup key, rebuilt for the impulse map: the same
                    // normalized endpoints the discovery keyed on, whether
                    // packed or array.
                    std::array<int32_t,4> lkey{{i,j,k,l}};
                    if (k < i) lkey = {{k,l,i,j}};
                    float &lambda = ws->selfLambda[{{int32_t(profileIndex),lkey[0],lkey[1],lkey[2],lkey[3]}}];
                    const float correction = RigExecClothProjectSamples(mesh,step,p,a,&b,
                        Dot(mid,d)-thickness,d,true,&lambda,x,false);
                    tickNow = TouchVerts(&spheres, i, j, k, l);
                    haveHeld = false;
                    triBandOn = false;
                    const float mu = p.friction * RigExecClothSampleMap(p,ClothFriction,a);
                    if (!(correction > 0 && mu > 0)) break;
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb);
                    float tangent[3];
                    for (size_t q = 0; q < 3; ++q) tangent[q] = (pa[q]-oldA[q])-(pb[q]-oldB[q]);
                    const float along = Dot(tangent,d);
                    for (size_t q = 0; q < 3; ++q) tangent[q] -= along*d[q];
                    const float slide = Length(tangent);
                    if (!(slide > 1e-12f)) break;
                    float mass = 0;
                    for (size_t q = 0; q < 3; ++q)
                        mass += mesh.inverseMass[size_t(a.vertices[q])]*a.weights[q]*a.weights[q] +
                                mesh.inverseMass[size_t(b.vertices[q])]*b.weights[q]*b.weights[q];
                    if (!(mass > 0)) break;
                    const float impulse = std::min(slide,mu*correction)/(mass*slide);
                    for (size_t q = 0; q < 3; ++q) for (size_t axis = 0; axis < 3; ++axis) {
                        x[3*size_t(a.vertices[q])+axis] -= mesh.inverseMass[size_t(a.vertices[q])]*
                            a.weights[q]*impulse*tangent[axis];
                        x[3*size_t(b.vertices[q])+axis] += mesh.inverseMass[size_t(b.vertices[q])]*
                            b.weights[q]*impulse*tangent[axis];
                    }
                } while (0);
                do { // Triangle edge 2.
                    const int32_t k = meshTriEdges[6*tri+4];
                    const int32_t l = meshTriEdges[6*tri+5];
                    // Adjacency first where the mesh built the union rows: every
                    // neighbor list holds its own vertex, so the constraint
                    // row holds both endpoints and every endpoint-sharing
                    // pair tests adjacent. The share test would skip nothing
                    // more, and the twin skip and the dedup order see the
                    // same survivors in the same order, with the same
                    // verdicts. Past the size cap the share test stays ahead
                    // of the insert, and the linear scans stay behind it so
                    // a rediscovered pair still skips them.
                    if (useEdgeBits) {
                        if (AdjacentToRowBits(eeAdjRow, k) ||
                            AdjacentToRowBits(eeAdjRow, l)) break;
                    }
                    else if (i == k || i == l || j == k || j == l) break;
                    if (usePairBits) {
                        // One bit per constraint pair, triangular with its
                        // diagonal: both edges are stretch constraints, so
                        // the unordered pair is one index, and the verdict
                        // is exact set membership like the twin skip and
                        // hash sets below, in discovery order.
                        const size_t o2 = size_t(meshTriEdgeConstraint[3*tri+2]);
                        const size_t hi = c > o2 ? c : o2;
                        const size_t lo = c > o2 ? o2 : c;
                        const size_t bit = hi * (hi + 1) / 2 + lo;
                        const uint64_t mask = 1ull << (bit & 63);
                        uint64_t &slot = eePairBase[bit >> 6];
                        if (slot & mask) break;
                        slot |= mask;
                    } else {
                        // Twin skip: the edge's smallest twin triangle sits
                        // earlier in this ascending bucket, so the same pair
                        // was already visited from this constraint and the
                        // dedup set holds its key.
                        const int32_t twin = meshTriTwin[3*tri+2];
                        if (twin >= 0 && twin < int32_t(tri) &&
                            eeTwinBase[size_t(twin)] == twinMark) break;
                        // Both edges arrive sorted with four distinct
                        // endpoints, so the pair order is one compare, and
                        // the insert hashes the ordered endpoints directly
                        // instead of packing and unpacking them first.
                        if (packPairs) {
                            if (k < i) {
                                if (!pairs64.InsertNoGrowFields(k, l, i, j)) break;
                            } else {
                                if (!pairs64.InsertNoGrowFields(i, j, k, l)) break;
                            }
                        } else {
                            std::array<int32_t,4> key{{i,j,k,l}};
                            if (k < i) key = {{k,l,i,j}};
                            if (!pairs.InsertNoGrow(key)) break;
                        }
                    }
                    if (!useEdgeBits) {
                        bool adjacent = false;
                        for (int32_t a : {i,j}) for (int32_t b : {k,l}) {
                            adjacent = adjacent || (useBits ? AdjacentToBits(mesh, a, b)
                                                            : AdjacentTo(mesh.neighbors[size_t(a)], b));
                        }
                        if (adjacent) break;
                    }
                    // The edge-sphere band: a pair whose midpoint spheres
                    // plus contact reach cannot meet skips the
                    // closest-segment tail. Each edge holds its points
                    // within half its length of its midpoint, so separated
                    // spheres imply a separated pair; the 2^-10 margin is far
                    // outside float rounding, and a NaN input compares
                    // false and skips nothing. ClosestSegments clamps its
                    // parameters into [0,1], so the sample map is a convex
                    // combination of the endpoint maps and the absolute
                    // bound covers every thickness sign.
                    if (!haveHeld) {
                        EdgeSphere(&spheres, c, i, j, x, heldC1, &heldR1, tickNow);
                        triBandBase = heldR1 + triReach + heldThick;
                        pairBandBase = heldR1 + heldThick;
                        haveHeld = true;
                    }
                    float c2[3], r2;
                    EdgeSphere(&spheres, constraintCount + 3 * tri + 2,
                               k, l, x, c2, &r2, tickNow);
                    const float dd0 = heldC1[0] - c2[0], dd1 = heldC1[1] - c2[1],
                                dd2 = heldC1[2] - c2[2];
                    const float reach = pairBandBase + r2;
                    if (dd0 * dd0 + dd1 * dd1 + dd2 * dd2 >
                        reach * reach * 1.0009765625f)
                        break;
                    float u, v;
                    ClosestSegments(x+3*size_t(i), x+3*size_t(j), x+3*size_t(k), x+3*size_t(l), &u, &v);
                    RigExecClothSample a, b;
                    a.vertices = {{i,j,j}}; a.weights = {{1-u,u,0}};
                    b.vertices = {{k,l,l}}; b.weights = {{1-v,v,0}};
                    if (!(RigExecClothRegion(p.sourceMask,a) * RigExecClothRegion(p.targetMask,b) > 0 ||
                          RigExecClothRegion(p.targetMask,a) * RigExecClothRegion(p.sourceMask,b) > 0)) break;
                    float pa[3], pb[3], d[3], oldA[3], oldB[3], oldD[3];
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb); Sub(pa,pb,d);
                    const float distance = Length(d);
                    const float thickness = p.thickness * RigExecClothSampleMap(p,ClothThickness,a);
                    if (distance >= thickness) break;
                    if (distance > 1e-10f) for (float &q : d) q /= distance;
                    else {
                        float e1[3], e2[3];
                        Sub(x+3*size_t(j),x+3*size_t(i),e1); Sub(x+3*size_t(l),x+3*size_t(k),e2);
                        Cross(e1,e2,d); const float len = Length(d);
                        if (!(len > 1e-10f)) break;
                        for (float &q : d) q /= len;
                    }
                    RigExecClothSamplePoint(a,previous,oldA); RigExecClothSamplePoint(b,previous,oldB);
                    Sub(oldA,oldB,oldD);
                    if (Dot(oldD,d) < 0) for (float &q : d) q = -q;
                    Sub(pa,pb,mid);
                    // The dedup key, rebuilt for the impulse map: the same
                    // normalized endpoints the discovery keyed on, whether
                    // packed or array.
                    std::array<int32_t,4> lkey{{i,j,k,l}};
                    if (k < i) lkey = {{k,l,i,j}};
                    float &lambda = ws->selfLambda[{{int32_t(profileIndex),lkey[0],lkey[1],lkey[2],lkey[3]}}];
                    const float correction = RigExecClothProjectSamples(mesh,step,p,a,&b,
                        Dot(mid,d)-thickness,d,true,&lambda,x,false);
                    tickNow = TouchVerts(&spheres, i, j, k, l);
                    haveHeld = false;
                    triBandOn = false;
                    const float mu = p.friction * RigExecClothSampleMap(p,ClothFriction,a);
                    if (!(correction > 0 && mu > 0)) break;
                    RigExecClothSamplePoint(a,x,pa); RigExecClothSamplePoint(b,x,pb);
                    float tangent[3];
                    for (size_t q = 0; q < 3; ++q) tangent[q] = (pa[q]-oldA[q])-(pb[q]-oldB[q]);
                    const float along = Dot(tangent,d);
                    for (size_t q = 0; q < 3; ++q) tangent[q] -= along*d[q];
                    const float slide = Length(tangent);
                    if (!(slide > 1e-12f)) break;
                    float mass = 0;
                    for (size_t q = 0; q < 3; ++q)
                        mass += mesh.inverseMass[size_t(a.vertices[q])]*a.weights[q]*a.weights[q] +
                                mesh.inverseMass[size_t(b.vertices[q])]*b.weights[q]*b.weights[q];
                    if (!(mass > 0)) break;
                    const float impulse = std::min(slide,mu*correction)/(mass*slide);
                    for (size_t q = 0; q < 3; ++q) for (size_t axis = 0; axis < 3; ++axis) {
                        x[3*size_t(a.vertices[q])+axis] -= mesh.inverseMass[size_t(a.vertices[q])]*
                            a.weights[q]*impulse*tangent[axis];
                        x[3*size_t(b.vertices[q])+axis] += mesh.inverseMass[size_t(b.vertices[q])]*
                            b.weights[q]*impulse*tangent[axis];
                    }
                } while (0);

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
/// the triangle (Ericson, Real-Time Collision Detection, 5.3.6). The area
/// vector arrives precomputed so a caller testing many segments against one
/// triangle derives it once; the value is the same.
inline bool SegmentCrossesTriangleArea(const float *p, const float *q,
                                       const float *a, const float *b,
                                       const float *c, const float *n)
{
    float ap[3], aq[3];
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
/// each other. \p grid, \p scratch, and \p areaScratch are reused storage;
/// a null \p areaScratch uses a call-local table. The table holds one area
/// vector per collider triangle, derived once per call instead of once per
/// edge pair; the values are the same.
inline bool
RigExecClothClearOfColliders(const RigExecClothMesh &mesh,
                             const RigExecClothColliders &colliders,
                             const float *cloth, const float *colliderPoints,
                             RigExecClothGrid *grid, std::vector<uint64_t> *scratch,
                             std::vector<float> *areaScratch = nullptr)
{
    using namespace cloth_detail;
    if (!cloth || !colliderPoints || colliders.triangles.empty()) return true;
    // Whole-surface box pretest: a crossing shares a point with both
    // surfaces, so boxes that clear each other hold no crossing and skip
    // the longest walk, the grid, the areas, and the edge walk below. The
    // margin is far outside the few ulps of slack in the plane-side,
    // lerp, and in-triangle tests. Non-finite bounds take the exact path,
    // and NaN points never win a strict bound so they cannot forge
    // disjointness (a NaN segment test is false either way).
    {
        const float inf = std::numeric_limits<float>::infinity();
        float cloLo[3] = {inf, inf, inf}, cloHi[3] = {-inf, -inf, -inf};
        float colLo[3] = {inf, inf, inf}, colHi[3] = {-inf, -inf, -inf};
        for (size_t i = 0; i < mesh.pointCount; ++i) {
            const float *p = cloth + 3 * i;
            for (int a = 0; a < 3; ++a) {
                if (p[a] < cloLo[a]) cloLo[a] = p[a];
                if (p[a] > cloHi[a]) cloHi[a] = p[a];
            }
        }
        for (size_t i = 0; i < colliders.pointCount; ++i) {
            const float *p = colliderPoints + 3 * i;
            for (int a = 0; a < 3; ++a) {
                if (p[a] < colLo[a]) colLo[a] = p[a];
                if (p[a] > colHi[a]) colHi[a] = p[a];
            }
        }
        float maxAbs = 0.0f;
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(cloLo[a]) || !std::isfinite(cloHi[a]) ||
                !std::isfinite(colLo[a]) || !std::isfinite(colHi[a])) {
                maxAbs = -1.0f;
                break;
            }
            maxAbs = std::max(maxAbs, std::fabs(cloLo[a]));
            maxAbs = std::max(maxAbs, std::fabs(cloHi[a]));
            maxAbs = std::max(maxAbs, std::fabs(colLo[a]));
            maxAbs = std::max(maxAbs, std::fabs(colHi[a]));
        }
        if (maxAbs >= 0.0f) {
            const float margin = (maxAbs + 1e-30f) * 0.0009765625f;
            for (int a = 0; a < 3; ++a)
                if (cloHi[a] + margin < colLo[a] || colHi[a] + margin < cloLo[a])
                    return true;
        }
    }
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
    const size_t triangleCount = colliders.triangles.size() / 3;
    std::vector<float> ownedAreas;
    if (!areaScratch) areaScratch = &ownedAreas;
    areaScratch->resize(3 * triangleCount);
    const float *areas = areaScratch->data();
    for (size_t tri = 0; tri < triangleCount; ++tri) {
        const float *a = colliderPoints + 3 * size_t(colliders.triangles[3 * tri]);
        const float *b = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 1]);
        const float *c = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 2]);
        float ab[3], ac[3];
        Sub(b, a, ab); Sub(c, a, ac);
        Cross(ab, ac, areaScratch->data() + 3 * tri);
    }
    // The swept collider bounds plus half the longest edge: an edge that
    // crosses a listed triangle holds its midpoint within half its own
    // length, hence within half the longest, of the crossing point on the
    // triangle, so an edge outside this box cannot cross and skips the
    // bucket test. The 2^-10 outward margin is far outside the few ulps of
    // rounding between the midpoint, the longest edge, and the segment
    // test. Non-finite bounds (no finite triangle) form NaN pads, which
    // compare false and skip nothing; overflowing pads only widen the box.
    const float half = 0.5f * longest;
    float padLo[3], padHi[3];
    for (int a = 0; a < 3; ++a) {
        const float margin =
            (std::max(std::fabs(grid->sweptLo[a]), std::fabs(grid->sweptHi[a])) + half) *
            0.0009765625f;
        padLo[a] = grid->sweptLo[a] - half - margin;
        padHi[a] = grid->sweptHi[a] + half + margin;
    }
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != stretch) continue;
        const float *p = cloth + 3 * size_t(mesh.constraintEnds[2 * c]);
        const float *q = cloth + 3 * size_t(mesh.constraintEnds[2 * c + 1]);
        float mid[3];
        Lerp(p, q, 0.5f, mid);
        if (mid[0] < padLo[0] || mid[0] > padHi[0] || mid[1] < padLo[1] ||
            mid[1] > padHi[1] || mid[2] < padLo[2] || mid[2] > padHi[2])
            continue;
        const uint32_t bucket = Hash(Cell(mid[0], grid->cellSize), Cell(mid[1], grid->cellSize),
                                     Cell(mid[2], grid->cellSize), grid->bucketMask);
        for (uint32_t k = grid->bucketStart[bucket]; k < grid->bucketStart[bucket + 1]; ++k) {
            const size_t tri = size_t(grid->bucketTriangles[k]);
            const float *a = colliderPoints + 3 * size_t(colliders.triangles[3 * tri]);
            const float *b = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 1]);
            const float *d = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 2]);
            if (SegmentCrossesTriangleArea(p, q, a, b, d, areas + 3 * tri))
                return false;
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
    std::vector<float> areas;
    std::vector<int8_t> clear(tickCount, -1);
    const auto isClear = [&](size_t k) {
        if (clear[k] < 0) {
            clear[k] = RigExecClothClearOfColliders(mesh, colliders, ticks[k].entering,
                                                    ticks[k].colliders, &grid, &scratch,
                                                    &areas)
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
        // The profile search's pre-check box over this tick's pad reach,
        // derived once per tick with the grid below.
        RigExecClothReachBounds profileBounds;
        const RigExecClothReachBounds *profileBroad = nullptr;
        // Skips the tick's per-substep triangle precompute while no
        // simulated particle starts the tick within the swept collider
        // reach. Every search then broadphase-rejects before any fetch,
        // and a particle crossing into reach mid-tick derives its
        // candidates with the same lerps, area vector, and verdict the
        // precompute ran, so the answers agree bit for bit either way.
        bool skipTris = false;
        bool tickStatic = false;
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
            // The tick's prefilter boxes on the empty-model path: the
            // reach is tick-fixed there (every search runs this step's
            // thickness and depth), so each triangle's widened bounds
            // derive once per tick instead of once per candidate. The
            // same arithmetic in the same order, so the boxes match the
            // per-candidate derivation bit for bit.
            if (mesh.model.Empty()) {
                const float thickness = std::max(0.0f, to.parameters.thickness);
                const float depth = std::max(0.0f, to.parameters.collisionDepth);
                const float reach = std::max(thickness, depth);
                const size_t triCount = colliders.triangles.size() / 3;
                ws->grid.triFilter.resize(6 * triCount);
                const float *tb = ws->grid.triBounds.data();
                const float *shapeSums = ws->grid.triShapes.data();
                float *fb = ws->grid.triFilter.data();
                for (size_t t = 0; t < triCount; ++t) {
                    const float shapeSum = reach + shapeSums[t];
                    const float bound = reach + shapeSum * 1.9073486328125e-06f + 1e-30f;
                    for (int a = 0; a < 3; ++a) {
                        fb[6 * t + size_t(a)] = tb[6 * t + size_t(a)] - bound;
                        fb[6 * t + size_t(a) + 3] = tb[6 * t + size_t(a) + 3] + bound;
                    }
                }
            }
            RigExecClothSweptReachBounds(ws->grid, pad, &profileBounds);
            profileBroad = &profileBounds;
            // The verdict scans every particle, so it runs only when the
            // tick's precompute is worth skipping: a substep per collider
            // triangle against two box tests per particle.
            const size_t triWork = size_t(substeps) * (colliders.triangles.size() / 3);
            if (triWork > 2 * n) {
                skipTris = true;
                for (size_t i = 0; i < n; ++i) {
                    if (!RigExecClothOutsideSweptBounds(ws->grid, x + 3 * i, pad)) {
                        skipTris = false;
                        break;
                    }
                }
            }
            // An empty precompute covers no triangle; the search derives
            // what it touches. The sized vectors stay for the next near tick.
            if (skipTris) ws->contacts.triangleCount = 0;
            // A tick whose collider points do not move interpolates every
            // substep to the same triangles: Lerp(a, a, t) is t-independent
            // for every a including signed zeros and NaNs, so the first
            // substep's precompute serves the rest bit for bit. Only without
            // material profiles: their self-contact pass can evict the reused
            // tables from the fast caches, which then costs more than the
            // rewrite it skips.
            tickStatic = mesh.model.Empty() && !skipTris && from.colliders && to.colliders &&
                         (from.colliders == to.colliders ||
                          std::memcmp(from.colliders, to.colliders,
                                       3 * colliders.pointCount * sizeof(float)) == 0);
        }
        for (int sub = 1; sub <= substeps; ++sub) {
            const RigExecClothStep s =
                RigExecClothStepFor(to.parameters, substeps, sub, frame);
            // The tick's collider triangles at this substep's fraction,
            // once for every particle and iteration below; far ticks skip
            // the precompute above and derive on touch instead, and static
            // ticks reuse the first substep's interpolation.
            if (collide && !skipTris && (sub == 1 || !tickStatic))
                RigExecClothContactTrisAt(colliders, from.colliders,
                                           to.colliders, s.t, &ws->contacts);
            const RigExecClothContactTris *contacts =
                collide ? &ws->contacts : nullptr;
            // The broadphase box over this substep's reach, once for
            // every particle below.
            RigExecClothReachBounds broadBounds;
            if (collide)
                RigExecClothSweptReachBounds(ws->grid, s.reach, &broadBounds);
            const RigExecClothReachBounds *broad =
                collide ? &broadBounds : nullptr;
            const float gravityStep[3] =
                {s.gravity[0] * s.h, s.gravity[1] * s.h, s.gravity[2] * s.h};
            for (size_t i = 0; i < n; ++i)
                RigExecClothPredict(mesh, s, from.entering, to.entering, i, x, xp, v,
                                    gravityStep);
            const bool single = iterations == 1;
            // Stretch kinematic pairs project to no-ops at zero alpha on
            // an empty model (the zero denominator returns before any
            // write, to positions or to lambda), so each pass starts
            // past the partitioned leading run.
            const bool skipKk = mesh.model.Empty() && s.alphaStretch == 0.0f;
            if (!single) std::fill(ws->lambda.begin(), ws->lambda.end(), 0.0f);
            std::fill(ws->attachmentLambda.begin(), ws->attachmentLambda.end(), 0.0f);
            std::fill(ws->contactLambda.begin(), ws->contactLambda.end(), 0.0f);
            std::fill(ws->contactTriangles.begin(), ws->contactTriangles.end(), -1);
            ws->selfLambda.clear();
            for (int it = 0; it < iterations; ++it) {
                for (size_t color = 0; color < colors; ++color) {
                    if (single) {
                        const uint32_t lo = skipKk ? mesh.colorStart[color] +
                                                         mesh.colorKkSkip[color]
                                                   : mesh.colorStart[color];
                        // The empty-model instantiation carries no model
                        // branch or dead profile code in the hot loop.
                        if (mesh.model.Empty())
                            RigExecClothProjectColorSingle<false>(mesh, s, lo,
                                                                 mesh.colorStart[color + 1],
                                                                 x);
                        else
                            RigExecClothProjectColorSingle<true>(mesh, s, lo,
                                                                mesh.colorStart[color + 1],
                                                                x);
                    } else {
                        const uint32_t lo = skipKk ? mesh.colorStart[color] +
                                                         mesh.colorKkSkip[color]
                                                   : mesh.colorStart[color];
                        for (uint32_t c = lo; c < mesh.colorStart[color + 1]; ++c)
                            RigExecClothProjectConstraint(mesh, s, c, x, ws->lambda.data());
                    }
                }
                if (!mesh.model.Empty())
                    RigExecClothBindings(mesh, s, settings, from, to, x, ws->attachmentLambda.data());
                for (size_t i = 0; i < n; ++i) {
                    RigExecClothParticlePass(mesh, colliders, ws->grid, settings, s,
                                             from.entering, to.entering,
                                             collide ? from.colliders : nullptr,
                                             collide ? to.colliders : nullptr,
                                             it == 0, i, x, xp,
                                             ws->contactLambda.data(), ws->contactTriangles.data(),
                                             contacts, broad, profileBroad);
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

inline uint64_t RigExecClothPairSet::Hash(const std::array<int32_t, 4> &key)
{
    return uint64_t(uint32_t(key[0])) + uint64_t(uint32_t(key[1])) * 0x9E3779B97F4A7C15ull +
           uint64_t(uint32_t(key[2])) * 0xC2B2AE3D27D4EB4Full +
           uint64_t(uint32_t(key[3])) * 0x165667B19E3779F9ull;
}

inline size_t RigExecClothPairSet::SlotCount() const
{
    return slots.size();
}

inline void RigExecClothPairSet::Clear()
{
    std::fill(occupied.begin(), occupied.end(), 0);
    count = 0;
}

inline bool RigExecClothPairSet::Insert(const std::array<int32_t, 4> &key)
{
    if ((count + 1) * 10 > slots.size() * 7) Grow();
    return InsertNoGrow(key);
}

inline void RigExecClothPairSet::ReserveForBatch(size_t batchKeys)
{
    if (!batchKeys) return;
    while ((count + batchKeys + 1) * 10 > slots.size() * 7) Grow();
}

inline bool RigExecClothPairSet::InsertNoGrow(const std::array<int32_t, 4> &key)
{
    uint32_t i = uint32_t(Hash(key)) & mask;
    for (;;) {
        const uint32_t word = i >> 6;
        const uint64_t bit = 1ull << (i & 63);
        if (!(occupied[word] & bit)) {
            occupied[word] |= bit;
            slots[i] = key;
            ++count;
            return true;
        }
        if (slots[i] == key) return false;
        i = (i + 1) & mask;
    }
}

inline bool cloth_detail::AdjacentTo(const std::vector<int32_t> &sorted, int32_t v)
{
    // Membership in a short sorted neighbor list: a linear scan with a
    // sorted early-out, fewer branches than a binary search here.
    for (int32_t n : sorted) {
        if (n == v) return true;
        if (n > v) return false;
    }
    return false;
}

inline bool cloth_detail::AdjacentToBits(const RigExecClothMesh &mesh, int32_t a, int32_t b)
{
    const size_t word = size_t(a) * mesh.neighborBitStride + (size_t(b) >> 6);
    return ((mesh.neighborBits[word] >> (b & 63)) & 1ull) != 0;
}

inline bool cloth_detail::AdjacentToEdgeBits(const RigExecClothMesh &mesh, size_t c, int32_t v)
{
    const size_t word = c * mesh.neighborBitStride + (size_t(v) >> 6);
    return ((mesh.edgeNeighborBits[word] >> (v & 63)) & 1ull) != 0;
}

inline bool cloth_detail::AdjacentToRowBits(const uint64_t *row, int32_t v)
{
    return ((row[size_t(v) >> 6] >> (v & 63)) & 1ull) != 0;
}

inline bool cloth_detail::AdjacentToTriBits(const RigExecClothMesh &mesh, size_t t, int32_t v)
{
    const size_t word = mesh.triRowBase[t] + (size_t(v) >> 6);
    return ((mesh.triNeighborBits[word] >> (v & 63)) & 1ull) != 0;
}

inline void RigExecClothPairSet::Grow()
{
    const size_t size = slots.empty() ? 4096 : slots.size() * 2;
    std::vector<std::array<int32_t, 4>> fresh(size);
    std::vector<uint64_t> bits(size / 64, 0);
    const uint32_t next = uint32_t(size - 1);
    for (size_t s = 0; s < slots.size(); ++s) {
        if (!(occupied[s >> 6] & (1ull << (s & 63)))) continue;
        uint32_t i = uint32_t(Hash(slots[s])) & next;
        while (bits[i >> 6] & (1ull << (i & 63))) i = (i + 1) & next;
        bits[i >> 6] |= 1ull << (i & 63);
        fresh[i] = slots[s];
    }
    slots.swap(fresh);
    occupied.swap(bits);
    mask = next;
}

inline uint64_t RigExecClothPairSet64::Hash(uint64_t key)
{
    // Unpacks the four fields and mixes them with the array set's
    // multipliers, so probe lengths match it; verdicts stay exact.
    const uint64_t k0 = key & 0xffffull;
    const uint64_t k1 = (key >> 16) & 0xffffull;
    const uint64_t k2 = (key >> 32) & 0xffffull;
    const uint64_t k3 = key >> 48;
    return k0 + k1 * 0x9E3779B97F4A7C15ull +
           k2 * 0xC2B2AE3D27D4EB4Full + k3 * 0x165667B19E3779F9ull;
}

inline size_t RigExecClothPairSet64::SlotCount() const
{
    return slots.size();
}

inline void RigExecClothPairSet64::Clear()
{
    std::fill(occupied.begin(), occupied.end(), 0);
    count = 0;
}

inline bool RigExecClothPairSet64::Insert(uint64_t key)
{
    if ((count + 1) * 4 > slots.size()) Grow();
    return InsertNoGrow(key);
}

inline void RigExecClothPairSet64::ReserveForBatch(size_t batchKeys)
{
    if (!batchKeys) return;
    while ((count + batchKeys + 1) * 4 > slots.size()) Grow();
}

inline bool RigExecClothPairSet64::InsertNoGrow(uint64_t key)
{
    uint32_t i = uint32_t(Hash(key)) & mask;
    for (;;) {
        const uint32_t word = i >> 6;
        const uint64_t bit = 1ull << (i & 63);
        if (!(occupied[word] & bit)) {
            occupied[word] |= bit;
            slots[i] = key;
            ++count;
            return true;
        }
        if (slots[i] == key) return false;
        i = (i + 1) & mask;
    }
}

inline bool RigExecClothPairSet64::InsertNoGrowFields(int32_t a, int32_t b,
                                                     int32_t c, int32_t d)
{
    // The same mix Hash applies to the unpacked packed key: every
    // endpoint sits below 2^16, so packing then unpacking recovers
    // these same four values and the hash matches bit for bit.
    const uint64_t h = uint64_t(uint32_t(a)) + uint64_t(uint32_t(b)) * 0x9E3779B97F4A7C15ull +
                       uint64_t(uint32_t(c)) * 0xC2B2AE3D27D4EB4Full +
                       uint64_t(uint32_t(d)) * 0x165667B19E3779F9ull;
    const uint64_t key = Pack(a, b, c, d);
    uint32_t i = uint32_t(h) & mask;
    for (;;) {
        const uint32_t word = i >> 6;
        const uint64_t bit = 1ull << (i & 63);
        if (!(occupied[word] & bit)) {
            occupied[word] |= bit;
            slots[i] = key;
            ++count;
            return true;
        }
        if (slots[i] == key) return false;
        i = (i + 1) & mask;
    }
}

inline void RigExecClothPairSet64::Grow()
{
    const size_t size = slots.empty() ? 4096 : slots.size() * 2;
    std::vector<uint64_t> fresh(size);
    std::vector<uint64_t> bits(size / 64, 0);
    const uint32_t next = uint32_t(size - 1);
    for (size_t s = 0; s < slots.size(); ++s) {
        if (!(occupied[s >> 6] & (1ull << (s & 63)))) continue;
        uint32_t i = uint32_t(Hash(slots[s])) & next;
        while (bits[i >> 6] & (1ull << (i & 63))) i = (i + 1) & next;
        bits[i >> 6] |= 1ull << (i & 63);
        fresh[i] = slots[s];
    }
    slots.swap(fresh);
    occupied.swap(bits);
    mask = next;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_CLOTH_KERNEL_H
