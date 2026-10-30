// clothKernel.h on small scenes whose answers are known without the solver:
// a pinned curtain hangs and keeps its edge lengths, a free sheet lands on a
// plane and stays above it, a sphere swept through a curtain leaves no
// particle inside, full pins and full attraction follow the input exactly,
// rigid input motion is carried rigidly, repeated solves are bit-identical,
// and a window whose first frame has a collider cutting through the cloth
// starts before the contact, so a ball pushed through a curtain keeps it in
// front frame after frame. The swept-bounds broadphase agrees with the full
// contact search. A format-25 (RCL1) compiled rig plays as it was exported.
// Header-only and USD-free.
#include "rigExecMath/clothKernel.h"
#include "rigExecBinary/clothPlayback.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <set>
#include <vector>

using namespace rigExec;

namespace {

int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

struct Mesh {
    std::vector<float> points;
    std::vector<int> counts, indices;
    size_t Size() const { return points.size() / 3; }
};

// A curtain in the xy plane, (n x n) points, `size` across, top at y = top.
Mesh Curtain(int n, float size, float top, float z = 0.0f)
{
    Mesh m;
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c) {
            m.points.push_back(-size / 2 + size * float(c) / float(n - 1));
            m.points.push_back(top - size * float(r) / float(n - 1));
            m.points.push_back(z);
        }
    for (int r = 0; r + 1 < n; ++r)
        for (int c = 0; c + 1 < n; ++c) {
            const int a = r * n + c;
            m.counts.push_back(4);
            m.indices.insert(m.indices.end(), {a, a + 1, a + n + 1, a + n});
        }
    return m;
}

// A horizontal sheet in the xz plane at height y, normals up.
Mesh Sheet(int n, float size, float y)
{
    Mesh m;
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c) {
            m.points.push_back(-size / 2 + size * float(c) / float(n - 1));
            m.points.push_back(y);
            m.points.push_back(-size / 2 + size * float(r) / float(n - 1));
        }
    for (int r = 0; r + 1 < n; ++r)
        for (int c = 0; c + 1 < n; ++c) {
            const int a = r * n + c;
            m.counts.push_back(4);
            // Counter-clockwise seen from +y.
            m.indices.insert(m.indices.end(), {a, a + n, a + n + 1, a + 1});
        }
    return m;
}

// A UV sphere, outward normals.
Mesh Sphere(float radius, int rings, int segments)
{
    Mesh m;
    m.points.insert(m.points.end(), {0.0f, radius, 0.0f});
    for (int r = 1; r < rings; ++r) {
        const float phi = 3.14159265f * float(r) / float(rings);
        for (int s = 0; s < segments; ++s) {
            const float theta = 6.2831853f * float(s) / float(segments);
            m.points.push_back(radius * std::sin(phi) * std::cos(theta));
            m.points.push_back(radius * std::cos(phi));
            m.points.push_back(radius * std::sin(phi) * std::sin(theta));
        }
    }
    m.points.insert(m.points.end(), {0.0f, -radius, 0.0f});
    const int south = int(m.Size()) - 1;
    const auto ring = [segments](int r, int s) { return 1 + (r - 1) * segments + (s % segments); };
    for (int s = 0; s < segments; ++s) {
        m.counts.push_back(3);
        m.indices.insert(m.indices.end(), {0, ring(1, s + 1), ring(1, s)});
    }
    for (int r = 1; r + 1 < rings; ++r)
        for (int s = 0; s < segments; ++s) {
            m.counts.push_back(4);
            m.indices.insert(m.indices.end(),
                             {ring(r, s), ring(r, s + 1), ring(r + 1, s + 1), ring(r + 1, s)});
        }
    for (int s = 0; s < segments; ++s) {
        m.counts.push_back(3);
        m.indices.insert(m.indices.end(), {south, ring(rings - 1, s), ring(rings - 1, s + 1)});
    }
    return m;
}

bool Build(const Mesh &m, const std::vector<float> *pins,
           const RigExecClothSettings &settings, RigExecClothMesh *out)
{
    std::string why;
    const bool ok = RigExecBuildClothMesh(
        m.Size(), m.counts.data(), m.counts.size(), m.indices.data(),
        m.indices.size(), false, m.points.data(), pins ? pins->data() : nullptr,
        settings, out, &why);
    if (!ok) std::printf("build failed: %s\n", why.c_str());
    return ok;
}

// Solves a window whose every tick holds \p entering and the collider
// points \p colliderAt(tick).
template <class ColliderAt>
std::vector<float> Solve(const RigExecClothMesh &mesh,
                         const RigExecClothColliders &colliders,
                         const RigExecClothSettings &settings,
                         const RigExecClothParameters &parameters,
                         const std::vector<std::vector<float>> &entering,
                         ColliderAt colliderAt, bool *ok = nullptr)
{
    std::vector<std::vector<float>> colliderPoints;
    for (size_t k = 0; k < entering.size(); ++k) colliderPoints.push_back(colliderAt(k));
    std::vector<RigExecClothTick> ticks(entering.size());
    for (size_t k = 0; k < entering.size(); ++k) {
        ticks[k].entering = entering[k].data();
        ticks[k].colliders = colliderPoints[k].empty() ? nullptr : colliderPoints[k].data();
        ticks[k].parameters = parameters;
    }
    RigExecClothWorkspace ws;
    std::vector<float> out(3 * mesh.pointCount);
    std::string why;
    const bool solved = RigExecSolveCloth(mesh, colliders, settings, ticks.data(),
                                          ticks.size(), 1.0 / 24.0, &ws, out.data(), &why);
    if (!solved) std::printf("solve: %s\n", why.c_str());
    if (ok) *ok = solved;
    return out;
}

std::vector<float> NoColliders(size_t) { return {}; }

float EdgeStretch(const RigExecClothMesh &mesh, const std::vector<float> &x)
{
    float worst = 0.0f;
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != uint8_t(RigExecClothConstraint::Stretch)) continue;
        const float *a = &x[3 * size_t(mesh.constraintEnds[2 * c])];
        const float *b = &x[3 * size_t(mesh.constraintEnds[2 * c + 1])];
        const float d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
        const float len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        worst = std::max(worst, len / mesh.constraintRest[c] - 1.0f);
    }
    return worst;
}

float MeanStretch(const RigExecClothMesh &mesh, const std::vector<float> &x)
{
    double sum = 0.0;
    size_t count = 0;
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != uint8_t(RigExecClothConstraint::Stretch)) continue;
        const float *a = &x[3 * size_t(mesh.constraintEnds[2 * c])];
        const float *b = &x[3 * size_t(mesh.constraintEnds[2 * c + 1])];
        const float d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
        sum += std::fabs(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) /
                         mesh.constraintRest[c] - 1.0f);
        ++count;
    }
    return count ? float(sum / double(count)) : 0.0f;
}

void TestTopology()
{
    const Mesh m = Curtain(3, 2.0f, 1.0f);
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, RigExecClothSettings(), &mesh));
    // 4 quads -> 8 triangles, 16 unique edges; the diagonals bend struts.
    CHECK(mesh.triangles.size() == 24);
    size_t stretch = 0, bend = 0;
    for (uint8_t k : mesh.constraintKind)
        (k == uint8_t(RigExecClothConstraint::Stretch) ? stretch : bend) += 1;
    CHECK(stretch == 16);
    CHECK(bend > 0);
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c)
        CHECK(mesh.constraintEnds[2 * c] < mesh.constraintEnds[2 * c + 1]);
    // No two constraints of a color share a particle, and the colors cover
    // every constraint once.
    CHECK(mesh.colorStart.front() == 0);
    CHECK(mesh.colorStart.back() == mesh.constraintRest.size());
    for (size_t k = 0; k < mesh.ColorCount(); ++k) {
        std::vector<int> seen(mesh.pointCount, 0);
        for (uint32_t c = mesh.colorStart[k]; c < mesh.colorStart[k + 1]; ++c) {
            CHECK(++seen[size_t(mesh.constraintEnds[2 * c])] == 1);
            CHECK(++seen[size_t(mesh.constraintEnds[2 * c + 1])] == 1);
        }
    }
    // Bad topology fails closed.
    Mesh bad = m;
    bad.indices[0] = 99;
    RigExecClothMesh refused;
    std::string why;
    CHECK(!RigExecBuildClothMesh(bad.Size(), bad.counts.data(), bad.counts.size(),
                                 bad.indices.data(), bad.indices.size(), false,
                                 bad.points.data(), nullptr, RigExecClothSettings(),
                                 &refused, &why));
    CHECK(!why.empty());
}

void TestPinnedCurtainHangs()
{
    const int n = 11;
    const Mesh m = Curtain(n, 20.0f, 10.0f);
    std::vector<float> pins(m.Size(), 0.0f);
    for (int c = 0; c < n; ++c) pins[size_t(c)] = 1.0f;  // the top row
    RigExecClothSettings settings;
    settings.windowFrames = 24;
    // Swing the curtain: start it tilted so gravity pulls it down.
    Mesh tilted = m;
    for (size_t i = 0; i < tilted.Size(); ++i) {
        const float drop = 10.0f - tilted.points[3 * i + 1];
        tilted.points[3 * i + 1] = 10.0f - 0.2f * drop;
        tilted.points[3 * i + 2] = drop * 0.98f;
    }
    RigExecClothMesh mesh;
    CHECK(Build(m, &pins, settings, &mesh));
    std::vector<std::vector<float>> entering(25, tilted.points);
    RigExecClothParameters p;
    const auto out = Solve(mesh, RigExecClothColliders(), settings, p, entering, NoColliders);
    // The top row stays where the input holds it.
    for (int c = 0; c < n; ++c)
        for (int a = 0; a < 3; ++a)
            CHECK(out[3 * size_t(c) + size_t(a)] == tilted.points[3 * size_t(c) + size_t(a)]);
    // The bottom row has fallen below where it started.
    const size_t bottom = size_t((n - 1) * n + n / 2);
    CHECK(out[3 * bottom + 1] < tilted.points[3 * bottom + 1] - 1.0f);
    // Tethers and rigid edges keep the stretch small. The worst edge is a
    // transient on the free border at the bottom of the swing; substeps buy
    // it back.
    const float worst = EdgeStretch(mesh, out);
    CHECK(worst < 0.12f);
    CHECK(MeanStretch(mesh, out) < 0.03f);
    for (float v : out) CHECK(std::isfinite(v));
    // Bit-identical when solved again.
    const auto again = Solve(mesh, RigExecClothColliders(), settings, p, entering, NoColliders);
    CHECK(std::memcmp(out.data(), again.data(), out.size() * sizeof(float)) == 0);
    RigExecClothSettings finer = settings;
    finer.substeps = 32;
    RigExecClothMesh fine;
    CHECK(Build(m, &pins, finer, &fine));
    const auto smooth = Solve(fine, RigExecClothColliders(), finer, p, entering, NoColliders);
    CHECK(EdgeStretch(fine, smooth) < worst);
    CHECK(EdgeStretch(fine, smooth) < 0.06f);
}

void TestSheetLandsOnPlane()
{
    const Mesh cloth = Sheet(9, 8.0f, 3.0f);
    const Mesh floor = Sheet(2, 40.0f, 0.0f);
    RigExecClothSettings settings;
    settings.windowFrames = 30;
    RigExecClothMesh mesh;
    CHECK(Build(cloth, nullptr, settings, &mesh));
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(floor.Size(), floor.counts.data(), floor.counts.size(),
                                     floor.indices.data(), floor.indices.size(), false,
                                     &colliders));
    RigExecClothParameters p;
    p.thickness = 0.25f;
    std::vector<std::vector<float>> entering(31, cloth.points);
    const auto out = Solve(mesh, colliders, settings, p, entering,
                           [&](size_t) { return floor.points; });
    float lowest = 1e9f;
    for (size_t i = 0; i < mesh.pointCount; ++i) lowest = std::min(lowest, out[3 * i + 1]);
    // It fell the whole way and rests on the floor at the thickness.
    CHECK(lowest > 0.25f - 0.02f);
    CHECK(lowest < 0.6f);
}

void TestSphereSweepsThroughCurtain()
{
    const int n = 17;
    const Mesh curtain = Curtain(n, 16.0f, 8.0f);
    std::vector<float> pins(curtain.Size(), 0.0f);
    for (int c = 0; c < n; ++c) pins[size_t(c)] = 1.0f;
    const float radius = 3.0f;
    const Mesh ball = Sphere(radius, 12, 20);
    RigExecClothSettings settings;
    settings.windowFrames = 24;
    settings.substeps = 12;
    RigExecClothMesh mesh;
    CHECK(Build(curtain, &pins, settings, &mesh));
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(ball.Size(), ball.counts.data(), ball.counts.size(),
                                     ball.indices.data(), ball.indices.size(), false,
                                     &colliders));
    RigExecClothParameters p;
    p.thickness = 0.2f;
    p.collisionDepth = radius;
    p.gravity[1] = 0.0f;
    // The ball travels from z = -6 to z = +4 across the window, at y = 2.
    const size_t ticks = 25;
    const auto ballAt = [&](size_t k) {
        std::vector<float> moved = ball.points;
        const float z = -6.0f + 10.0f * float(k) / float(ticks - 1);
        for (size_t i = 0; i < ball.Size(); ++i) {
            moved[3 * i + 1] += 2.0f;
            moved[3 * i + 2] += z;
        }
        return moved;
    };
    std::vector<std::vector<float>> entering(ticks, curtain.points);
    const auto out = Solve(mesh, colliders, settings, p, entering, ballAt);
    // No particle is left inside the (faceted) ball: allow the facet sag.
    const float sag = radius * (1.0f - std::cos(3.14159265f / 20.0f)) + 0.02f;
    size_t inside = 0;
    for (size_t i = 0; i < mesh.pointCount; ++i) {
        const float dx = out[3 * i], dy = out[3 * i + 1] - 2.0f, dz = out[3 * i + 2] - 4.0f;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < radius - sag) ++inside;
    }
    CHECK(inside == 0);
    // And the ball pushed the curtain forward.
    float front = -1e9f;
    for (size_t i = 0; i < mesh.pointCount; ++i) front = std::max(front, out[3 * i + 2]);
    CHECK(front > 4.0f);
}

void TestFollowsInput()
{
    const Mesh m = Curtain(6, 5.0f, 2.0f);
    RigExecClothSettings settings;
    RigExecClothParameters p;
    std::vector<std::vector<float>> entering;
    for (int k = 0; k < 10; ++k) {
        std::vector<float> moved = m.points;
        for (size_t i = 0; i < m.Size(); ++i) moved[3 * i] += 0.3f * float(k * k);
        entering.push_back(moved);
    }
    // Every particle pinned: exactly the input.
    std::vector<float> all(m.Size(), 1.0f);
    RigExecClothMesh pinned;
    CHECK(Build(m, &all, settings, &pinned));
    auto out = Solve(pinned, RigExecClothColliders(), settings, p, entering, NoColliders);
    CHECK(std::memcmp(out.data(), entering.back().data(), out.size() * sizeof(float)) == 0);
    // Full attraction: the input, to rounding.
    RigExecClothMesh free;
    CHECK(Build(m, nullptr, settings, &free));
    p.attract = 1.0f;
    out = Solve(free, RigExecClothColliders(), settings, p, entering, NoColliders);
    for (size_t i = 0; i < out.size(); ++i) CHECK(std::fabs(out[i] - entering.back()[i]) < 1e-4f);
    // One tick is the input.
    p.attract = 0.0f;
    out = Solve(free, RigExecClothColliders(), settings, p,
                std::vector<std::vector<float>>{entering.back()}, NoColliders);
    CHECK(std::memcmp(out.data(), entering.back().data(), out.size() * sizeof(float)) == 0);
}

void TestCarriedRigidly()
{
    // Constant input velocity, no gravity, no damping: the cloth starts
    // moving with its input and stays with it.
    const Mesh m = Curtain(7, 6.0f, 3.0f);
    RigExecClothSettings settings;
    RigExecClothParameters p;
    p.gravity[1] = 0.0f;
    p.damping = 0.0f;
    std::vector<std::vector<float>> entering;
    for (int k = 0; k < 20; ++k) {
        std::vector<float> moved = m.points;
        for (size_t i = 0; i < m.Size(); ++i) {
            moved[3 * i] += 0.5f * float(k);
            moved[3 * i + 2] -= 0.25f * float(k);
        }
        entering.push_back(moved);
    }
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, settings, &mesh));
    const auto out = Solve(mesh, RigExecClothColliders(), settings, p, entering, NoColliders);
    for (size_t i = 0; i < out.size(); ++i) CHECK(std::fabs(out[i] - entering.back()[i]) < 1e-3f);
}

// A 17 x 17 curtain at z = 0 pinned along its top, and a radius-6 ball that
// rolls into it from behind, holds 3 past the curtain's plane, and leaves.
struct PushThrough {
    static constexpr int kFrames = 70;
    Mesh curtain = Curtain(17, 24.0f, 30.0f);
    Mesh ball = Sphere(6.0f, 12, 24);
    std::vector<float> pins;
    std::vector<std::vector<float>> balls;
    PushThrough()
    {
        pins.assign(curtain.Size(), 0.0f);
        for (int c = 0; c < 17; ++c) pins[size_t(c)] = 1.0f;
        for (int k = 0; k < kFrames; ++k) {
            balls.push_back(ball.points);
            for (size_t i = 0; i < ball.Size(); ++i) {
                balls.back()[3 * i + 1] += 18.0f;
                balls.back()[3 * i + 2] += BallZ(k);
            }
        }
    }
    static float BallZ(int k)
    {
        const auto ease = [](float u) { u = std::min(1.0f, std::max(0.0f, u)); return u * u * (3 - 2 * u); };
        if (k < 30) return -20.0f + 23.0f * ease(float(k - 10) / 20.0f);
        return 3.0f - 23.0f * ease(float(k - 45) / 20.0f);
    }
};

void TestClearOfColliders()
{
    PushThrough scene;
    RigExecClothMesh mesh;
    CHECK(Build(scene.curtain, &scene.pins, RigExecClothSettings(), &mesh));
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(scene.ball.Size(), scene.ball.counts.data(),
                                     scene.ball.counts.size(), scene.ball.indices.data(),
                                     scene.ball.indices.size(), false, &colliders));
    RigExecClothGrid grid;
    std::vector<uint64_t> scratch;
    std::vector<float> areas;
    // Behind the curtain, through it, and on the far side of it.
    for (int k = 0; k < PushThrough::kFrames; ++k) {
        const float z = PushThrough::BallZ(k);
        const bool cuts = std::fabs(z) < 6.0f;
        const bool clear = RigExecClothClearOfColliders(mesh, colliders, scene.curtain.points.data(),
                                                        scene.balls[size_t(k)].data(), &grid,
                                                        &scratch);
        // Within a facet's sag of the radius either answer is right.
        if (std::fabs(std::fabs(z) - 6.0f) > 0.3f) CHECK(clear == !cuts);
        // Caller-held area storage agrees with the call-local table.
        const bool shared = RigExecClothClearOfColliders(mesh, colliders, scene.curtain.points.data(),
                                                         scene.balls[size_t(k)].data(), &grid,
                                                         &scratch, &areas);
        CHECK(shared == clear);
    }
    // No colliders: always clear.
    CHECK(RigExecClothClearOfColliders(mesh, RigExecClothColliders(), scene.curtain.points.data(),
                                       nullptr, &grid, &scratch));
}

// Brute force: every stretch edge against every collider triangle, the same
// segment test the grid path runs once it lists a pair.
bool BruteForceClear(const RigExecClothMesh &mesh, const RigExecClothColliders &colliders,
                     const float *cloth, const float *colliderPoints)
{
    const auto stretch = uint8_t(RigExecClothConstraint::Stretch);
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != stretch) continue;
        const float *p = cloth + 3 * size_t(mesh.constraintEnds[2 * c]);
        const float *q = cloth + 3 * size_t(mesh.constraintEnds[2 * c + 1]);
        for (size_t tri = 0; tri < colliders.triangles.size() / 3; ++tri) {
            const float *a = colliderPoints + 3 * size_t(colliders.triangles[3 * tri]);
            const float *b = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 1]);
            const float *d = colliderPoints + 3 * size_t(colliders.triangles[3 * tri + 2]);
            float ab[3], ac[3], n[3];
            cloth_detail::Sub(b, a, ab);
            cloth_detail::Sub(d, a, ac);
            cloth_detail::Cross(ab, ac, n);
            if (cloth_detail::SegmentCrossesTriangleArea(p, q, a, b, d, n)) return false;
        }
    }
    return true;
}

void TestClearOfCollidersBroadphase()
{
    const Mesh curtain = Curtain(9, 16.0f, 20.0f);
    std::vector<float> pins(curtain.Size(), 0.0f);
    for (int c = 0; c < 9; ++c) pins[size_t(c)] = 1.0f;
    RigExecClothMesh mesh;
    CHECK(Build(curtain, &pins, RigExecClothSettings(), &mesh));
    if (!mesh.valid) return;
    const Mesh ball = Sphere(4.0f, 8, 12);
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(ball.Size(), ball.counts.data(), ball.counts.size(),
                                     ball.indices.data(), ball.indices.size(), false,
                                     &colliders));
    RigExecClothGrid grid;
    std::vector<uint64_t> scratch;
    // Seeded placements from far away to straight through; the grid and
    // scratch are reused across calls as the window start reuses them.
    uint64_t state = 0x9e3779b97f4a7c15ULL;
    const auto rnd = [&]() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return double(state >> 33) / double(1ULL << 31) - 0.5;
    };
    int blocked = 0, clear = 0;
    for (int seed = 0; seed < 600; ++seed) {
        const float scale = seed % 3 == 0 ? 0.5f : (seed % 3 == 1 ? 1.0f : 2.0f);
        std::vector<float> at = ball.points;
        const float ox = float(40.0 * rnd()), oy = float(12.0 + 30.0 * rnd()),
                    oz = float(30.0 * rnd());
        for (size_t i = 0; i < ball.Size(); ++i) {
            at[3 * i] = at[3 * i] * scale + ox;
            at[3 * i + 1] = at[3 * i + 1] * scale + oy;
            at[3 * i + 2] = at[3 * i + 2] * scale + oz;
        }
        const bool want = BruteForceClear(mesh, colliders, curtain.points.data(), at.data());
        const bool got = RigExecClothClearOfColliders(mesh, colliders, curtain.points.data(),
                                                      at.data(), &grid, &scratch);
        if (got != want) {
            std::printf("seed %d: broadphase %d, brute force %d\n", seed, int(got),
                        int(want));
            CHECK(false);
        }
        blocked += !want;
        clear += want;
    }
    CHECK(blocked > 0 && clear > 0);
    // Edges snapped to the padded box: from two margins inside to half an
    // edge past it, on both x sides. Near-boundary skips must still agree.
    float longest = 0.0f;
    const auto stretch = uint8_t(RigExecClothConstraint::Stretch);
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
        if (mesh.constraintKind[c] != stretch) continue;
        const float *p = curtain.points.data() + 3 * size_t(mesh.constraintEnds[2 * c]);
        const float *q = curtain.points.data() + 3 * size_t(mesh.constraintEnds[2 * c + 1]);
        float d[3];
        cloth_detail::Sub(p, q, d);
        longest = std::max(longest, cloth_detail::Length(d));
    }
    const float half = 0.5f * longest;
    float sweptLo = std::numeric_limits<float>::infinity(),
          sweptHi = -std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < ball.Size(); ++i) {
        sweptLo = std::min(sweptLo, ball.points[3 * i]);
        sweptHi = std::max(sweptHi, ball.points[3 * i]);
    }
    const float margin = (std::max(std::fabs(sweptLo), std::fabs(sweptHi)) + half) *
                             0.0009765625f;
    size_t firstStretch = 0;
    while (firstStretch < mesh.constraintRest.size() &&
           mesh.constraintKind[firstStretch] != stretch)
        ++firstStretch;
    CHECK(firstStretch < mesh.constraintRest.size());
    const float *e0 =
        curtain.points.data() + 3 * size_t(mesh.constraintEnds[2 * firstStretch]);
    const float *e1 =
        curtain.points.data() + 3 * size_t(mesh.constraintEnds[2 * firstStretch + 1]);
    const float midX = 0.5f * (e0[0] + e1[0]);
    for (float delta : {-2.0f * margin, -0.5f * margin, 0.0f, 0.5f * margin, 2.0f * margin,
                        half}) {
        for (int side = 0; side < 2; ++side) {
            std::vector<float> at = ball.points;
            const float edge = side == 0 ? sweptHi + half + margin : sweptLo - half - margin;
            const float tx = side == 0 ? midX - edge + delta : midX - edge - delta;
            for (size_t i = 0; i < ball.Size(); ++i) at[3 * i] += tx;
            const bool want =
                BruteForceClear(mesh, colliders, curtain.points.data(), at.data());
            const bool got = RigExecClothClearOfColliders(mesh, colliders,
                                                          curtain.points.data(), at.data(),
                                                          &grid, &scratch);
            if (got != want) {
                std::printf("snap delta %g side %d: broadphase %d, brute force %d\n",
                            double(delta), side, int(got), int(want));
                CHECK(false);
            }
        }
    }
}

// A crossing whose midpoint sits outside the collider's swept box but
// within half the longest edge of it: the broadphase must test it, not
// skip it. A pads-hug-the-box mutation reports clear here.
void TestClearOfCollidersBandCrossing()
{
    const Mesh m = Curtain(2, 2.0f, 1.0f);
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, RigExecClothSettings(), &mesh));
    if (!mesh.valid) return;
    RigExecClothColliders colliders;
    const int counts[] = {3}, indices[] = {0, 1, 2};
    CHECK(RigExecAppendClothCollider(3, counts, 1, indices, 3, false, &colliders));
    // Edge (0, 1) pierces the triangle's interior at (9.9, 0.5, 0.5) with
    // its midpoint at x = 10.1, past the box face at x = 10 but within
    // half the longest edge of it; the other points sit clear of the box.
    std::vector<float> cloth = {9.6f, 0.47f, 0.53f, 10.6f, 0.57f, 0.43f,
                                20.0f, 0.0f, 0.0f, 20.0f, 1.0f, 0.0f};
    const std::vector<float> tri = {9.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                                    9.9f, 1.0f, 1.0f};
    RigExecClothGrid grid;
    std::vector<uint64_t> scratch;
    CHECK(!BruteForceClear(mesh, colliders, cloth.data(), tri.data()));
    CHECK(!RigExecClothClearOfColliders(mesh, colliders, cloth.data(), tri.data(), &grid,
                                        &scratch));
    // The same edge shifted half an edge past the padded box: clear by both.
    for (int a = 0; a < 3; ++a) {
        cloth[a] += 6.0f;
        cloth[3 + a] += 6.0f;
    }
    CHECK(BruteForceClear(mesh, colliders, cloth.data(), tri.data()));
    CHECK(RigExecClothClearOfColliders(mesh, colliders, cloth.data(), tri.data(), &grid,
                                       &scratch));
}

// Fraction of the curtain points within reach of the ball that lie on its
// far (+z) side: the side the ball pushed them to.
float AheadOfBall(const std::vector<float> &x, float z)
{
    size_t near = 0, ahead = 0;
    for (size_t i = 0; i < x.size() / 3; ++i) {
        const float dx = x[3 * i], dy = x[3 * i + 1] - 18.0f, dz = x[3 * i + 2] - z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 8.0f) continue;
        ++near;
        if (dz > 0.0f) ++ahead;
    }
    return near ? float(ahead) / float(near) : 1.0f;
}

void TestWindowStartsBeforeContact()
{
    PushThrough scene;
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(scene.ball.Size(), scene.ball.counts.data(),
                                     scene.ball.counts.size(), scene.ball.indices.data(),
                                     scene.ball.indices.size(), false, &colliders));
    RigExecClothParameters p;
    p.damping = 2.0f;
    std::vector<RigExecClothTick> all(PushThrough::kFrames);
    for (int k = 0; k < PushThrough::kFrames; ++k) {
        all[size_t(k)].entering = scene.curtain.points.data();
        all[size_t(k)].colliders = scene.balls[size_t(k)].data();
        all[size_t(k)].parameters = p;
    }
    // Solves frame t over the history a window may read, from its start.
    const auto solve = [&](const RigExecClothMesh &mesh, const RigExecClothSettings &s, int t,
                           size_t *startOut) {
        const int history = RigExecClothHistoryFrames(s);
        const size_t first = size_t(std::max(0, t - history));
        const RigExecClothTick *ticks = all.data() + first;
        const size_t count = size_t(t) + 1 - first;
        const size_t start = RigExecClothWindowStart(mesh, colliders, s, ticks, count);
        if (startOut) *startOut = first + start;
        RigExecClothWorkspace ws;
        std::vector<float> out(3 * mesh.pointCount);
        CHECK(RigExecSolveCloth(mesh, colliders, s, ticks + start, count - start, 1.0 / 24.0,
                                &ws, out.data()));
        return out;
    };
    RigExecClothSettings reach;
    reach.windowFrames = 12;
    reach.maxWindowFrames = 64;
    RigExecClothSettings fixed = reach;
    fixed.maxWindowFrames = reach.windowFrames;
    RigExecClothMesh mesh;
    CHECK(Build(scene.curtain, &scene.pins, reach, &mesh));
    // Before any contact the window is its own length.
    size_t start = 0;
    solve(mesh, reach, 20, &start);
    CHECK(start == 8);
    // With the ball held past the curtain's plane at the window's first
    // frame, the window starts before the ball reached the curtain.
    solve(mesh, reach, 44, &start);
    CHECK(start < 32);
    CHECK(std::fabs(PushThrough::BallZ(int(start))) >= 6.0f);
    // Turned off, it does not reach back.
    solve(mesh, fixed, 44, &start);
    CHECK(start == 32);
    // The first contact fixes the start before it by a full settling
    // interval, even while the nominal start is still outside the ball.
    // Otherwise each frame replays a different startup transient into the
    // same contact and the curtain jitters while the collider holds still.
    size_t contactStart = 0;
    solve(mesh, reach, 30, &contactStart);
    size_t firstContact = 0;
    RigExecClothGrid grid;
    std::vector<uint64_t> scratch;
    while (firstContact < all.size() &&
           RigExecClothClearOfColliders(mesh, colliders, all[firstContact].entering,
                                         all[firstContact].colliders, &grid, &scratch))
        ++firstContact;
    CHECK(firstContact < all.size());
    CHECK(contactStart + size_t(reach.windowFrames) < firstContact);
    for (int t = 31; t <= 46; ++t) {
        const auto got = solve(mesh, reach, t, &start);
        CHECK(start == contactStart);
        RigExecClothWorkspace ws;
        std::vector<float> continuous(got.size());
        CHECK(RigExecSolveCloth(mesh, colliders, reach, all.data() + contactStart,
                                size_t(t) + 1 - contactStart, 1.0 / 24.0,
                                &ws, continuous.data()));
        CHECK(std::memcmp(got.data(), continuous.data(), got.size() * sizeof(float)) == 0);
    }
    // Frame after frame the curtain stays on the side the ball pushed it to;
    // a window that starts with the ball inside the curtain drops it behind.
    float worstReach = 1.0f, worstFixed = 1.0f;
    for (int t = 30; t <= 46; ++t) {
        const float z = PushThrough::BallZ(t);
        worstReach = std::min(worstReach, AheadOfBall(solve(mesh, reach, t, nullptr), z));
        worstFixed = std::min(worstFixed, AheadOfBall(solve(mesh, fixed, t, nullptr), z));
    }
    std::printf("curtain ahead of the ball, worst frame: reaching back %.2f, fixed window %.2f\n",
                worstReach, worstFixed);
    CHECK(worstReach > 0.5f);
    CHECK(worstFixed < 0.5f);
}

bool SameContact(const RigExecClothContact &a, const RigExecClothContact &b)
{
    return a.triangle == b.triangle &&
        std::memcmp(&a.violation, &b.violation, sizeof(float)) == 0 &&
        std::memcmp(a.normal, b.normal, sizeof(a.normal)) == 0 &&
        std::memcmp(a.weights, b.weights, sizeof(a.weights)) == 0;
}

void TestSweptBoundsAndBroadphase()
{
    const float box[24] = {-8, 0, -8, 8, 0, -8, 8, 0, 8, -8, 0, 8,
                           -8, 15.5f, -8, 8, 15.5f, -8,
                           8, 15.5f, 8, -8, 15.5f, 8};
    const int counts[6] = {4, 4, 4, 4, 4, 4};
    const int indices[24] = {0, 1, 2, 3, 4, 7, 6, 5, 3, 2, 6, 7,
                             0, 4, 5, 1, 0, 3, 7, 4, 1, 5, 6, 2};
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(8, counts, 6, indices, 24, false, &colliders));
    // A grid that never went through the build covers everything, so the
    // broadphase never skips on it; so does an empty build.
    RigExecClothGrid fresh;
    for (int a = 0; a < 3; ++a) {
        CHECK(fresh.sweptLo[a] == -std::numeric_limits<float>::infinity());
        CHECK(fresh.sweptHi[a] == std::numeric_limits<float>::infinity());
    }
    RigExecClothGrid empty;
    std::vector<uint64_t> scratch;
    RigExecBuildClothGrid(RigExecClothColliders(), nullptr, nullptr, 1.0f, &empty,
                          &scratch);
    for (int a = 0; a < 3; ++a) {
        CHECK(empty.sweptLo[a] == -std::numeric_limits<float>::infinity());
        CHECK(empty.sweptHi[a] == std::numeric_limits<float>::infinity());
    }
    const float thicknessDepth[][2] = {{0.5f, 2.0f}, {0.0f, 0.0f}, {0.0f, 5.0f},
                                       {2.0f, 0.5f}, {1e-30f, 1e-30f}};
    const float fractions[] = {0.0f, 0.37f, 1.0f};
    size_t compared = 0;
    for (float shift : {0.0f, 1e15f}) {
        // The box swept from `before` to `after`; the second pass moves the
        // scene to 1e15 units, where the bound rounding dwarfs the reach and
        // only the margin keeps the broadphase sound.
        float before[24], after[24];
        for (int i = 0; i < 24; ++i) {
            before[i] = box[i] + shift;
            after[i] = box[i] + shift + (i % 3 == 0 ? 4.0f : (i % 3 == 1 ? -2.0f : 1.0f));
        }
        RigExecClothGrid grid;
        RigExecBuildClothGrid(colliders, before, after, 8.0f, &grid, &scratch);
        for (int a = 0; a < 3; ++a) {
            float lo = std::numeric_limits<float>::infinity();
            float hi = -std::numeric_limits<float>::infinity();
            for (int v = 0; v < 8; ++v) {
                lo = std::min(lo, std::min(before[3 * v + a], after[3 * v + a]));
                hi = std::max(hi, std::max(before[3 * v + a], after[3 * v + a]));
            }
            CHECK(grid.sweptLo[a] == lo);
            CHECK(grid.sweptHi[a] == hi);
        }
        // The same grid with the broadphase scrubbed off agrees bitwise.
        RigExecClothGrid scrubbed = grid;
        for (int a = 0; a < 3; ++a) {
            scrubbed.sweptLo[a] = -std::numeric_limits<float>::infinity();
            scrubbed.sweptHi[a] = std::numeric_limits<float>::infinity();
        }
        // Query lattice over the swept box plus the reach, boundary-adjacent
        // points, far points, and non-finite ones.
        std::vector<std::array<float, 3>> queries;
        for (const auto &td : thicknessDepth) {
            const float reach = std::max(td[0], td[1]);
            for (int a = 0; a < 3; ++a) {
                const float lo = grid.sweptLo[a] - reach;
                const float hi = grid.sweptHi[a] + reach;
                for (int k = 0; k <= 6; ++k) {
                    std::array<float, 3> q = {{shift, shift + 8.0f, shift}};
                    q[size_t(a)] = lo + (hi - lo) * float(k) / 6.0f;
                    queries.push_back(q);
                }
                for (float edge : {lo, hi}) {
                    for (float qv : {edge, std::nextafterf(edge, -1e30f),
                                     std::nextafterf(edge, 1e30f)}) {
                        std::array<float, 3> q = {{shift, shift + 8.0f, shift}};
                        q[size_t(a)] = qv;
                        queries.push_back(q);
                    }
                }
            }
        }
        for (float far : {-1e6f, 1e6f}) queries.push_back({{far + shift, shift, shift}});
        const float inf = std::numeric_limits<float>::infinity();
        const float nan = std::numeric_limits<float>::quiet_NaN();
        queries.push_back({{nan, shift, shift}});
        queries.push_back({{shift, inf, shift}});
        queries.push_back({{-inf, shift, shift}});
        for (const auto &td : thicknessDepth) {
            for (float t : fractions) {
                RigExecClothContactTris tris;
                RigExecClothContactTrisAt(colliders, before, after, t, &tris);
                for (bool table : {false, true}) {
                    for (const auto &q : queries) {
                        const RigExecClothContact a = RigExecClothFindContact(
                            colliders, grid, before, after, t, q.data(), td[0], td[1],
                            nullptr, 0, 0, std::numeric_limits<size_t>::max(), false,
                            nullptr, table ? &tris : nullptr);
                        const RigExecClothContact b = RigExecClothFindContact(
                            colliders, scrubbed, before, after, t, q.data(), td[0], td[1],
                            nullptr, 0, 0, std::numeric_limits<size_t>::max(), false,
                            nullptr, table ? &tris : nullptr);
                        CHECK(SameContact(a, b));
                        ++compared;
                    }
                }
            }
        }
    }
    CHECK(compared > 10000);
}

void TestMultiIterationSolves()
{
    // The solve dispatches the general projection when iterations > 1;
    // it must still succeed deterministically with finite results.
    const Mesh m = Curtain(4, 3.0f, 1.0f);
    RigExecClothSettings settings;
    settings.iterations = 3;
    settings.substeps = 2;
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, settings, &mesh));
    RigExecClothParameters p;
    std::vector<std::vector<float>> entering(3, m.points);
    bool ok = false, ok2 = false;
    const auto a = Solve(mesh, RigExecClothColliders(), settings, p, entering,
                         NoColliders, &ok);
    const auto b = Solve(mesh, RigExecClothColliders(), settings, p, entering,
                         NoColliders, &ok2);
    CHECK(ok);
    CHECK(ok2);
    CHECK(a.size() == 3 * mesh.pointCount);
    for (float v : a) CHECK(std::isfinite(v));
    CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0);
}

void TestRefusals()
{
    const Mesh m = Curtain(4, 3.0f, 1.0f);
    RigExecClothSettings settings;
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, settings, &mesh));
    RigExecClothParameters p;
    p.damping = std::nanf("");
    std::vector<std::vector<float>> entering(3, m.Size() ? m.points : std::vector<float>());
    bool ok = true;
    const auto out = Solve(mesh, RigExecClothColliders(), settings, p, entering, NoColliders, &ok);
    CHECK(!ok);
    CHECK(std::memcmp(out.data(), entering.back().data(), out.size() * sizeof(float)) == 0);
}

RigExecClothModel MaterialModel(const RigExecClothMesh &mesh, const Mesh &m,
                               const RigExecClothProfile &profile)
{
    RigExecClothModel model;
    model.profiles.push_back(profile);
    for (int32_t vertex : mesh.triangles) {
        model.triangleUV.push_back(m.points[3 * size_t(vertex)]);
        model.triangleUV.push_back(m.points[3 * size_t(vertex) + 1]);
    }
    return model;
}

void TestForceResponseAndDirectionalMaterial()
{
    RigExecClothProfile p;
    p.minStrength = 2; p.maxStrength = 10;
    p.forceCurve = {0, 0, 100, 1};
    const auto sample = RigExecClothVertex(0);
    CHECK(RigExecClothStrength(p, sample, 50) == 6);
    CHECK(RigExecClothStrength(p, sample, 200) == 10);
    p.forceCurve = {0, 1, 100, 0};
    CHECK(RigExecClothStrength(p, sample, 200) == 2);
    p.maps[ClothMask] = {0};
    CHECK(RigExecClothStrength(p, sample, 50) == 0);

    const Mesh m = Curtain(2, 1, 1);
    RigExecClothSettings settings;
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, settings, &mesh));
    p = RigExecClothProfile();
    p.kind = RigExecClothProfileKind::Stretch;
    p.warp = 100; p.weft = .01f; p.compliance = .001f;
    auto model = MaterialModel(mesh, m, p);
    std::string why;
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
    RigExecClothStep step;
    step.h = 1.0f / 24;
    float correction[2] = {0, 0};
    for (int axis = 0; axis < 2; ++axis) {
        std::vector<float> x = m.points;
        for (size_t i = 0; i < m.Size(); ++i) x[3*i+size_t(axis)] *= 2;
        std::vector<float> lambda(mesh.constraintRest.size());
        for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
            const size_t a = size_t(mesh.constraintEnds[2*c]), b = size_t(mesh.constraintEnds[2*c+1]);
            if (mesh.constraintKind[c] != 0 ||
                std::abs(m.points[3*a+size_t(axis)] - m.points[3*b+size_t(axis)]) != 1 ||
                m.points[3*a+size_t(1-axis)] != m.points[3*b+size_t(1-axis)]) continue;
            const float before = std::abs(x[3*a+size_t(axis)] - x[3*b+size_t(axis)]);
            RigExecClothProjectConstraint(mesh, step, c, x.data(), lambda.data());
            correction[axis] = before - std::abs(x[3*a+size_t(axis)] - x[3*b+size_t(axis)]);
            break;
        }
    }
    CHECK(correction[0] > 10 * correction[1]);

    p.kind = RigExecClothProfileKind::Damping;
    p.minStrength = p.maxStrength = 10;
    p.warp = 1; p.weft = p.normal = 0;
    model = MaterialModel(mesh, m, p);
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
    std::vector<float> x = m.points, previous = m.points, velocity(m.points.size());
    for (size_t i = 0; i < m.Size(); ++i) {
        x[3*i] = -m.points[3*i+1]; x[3*i+1] = m.points[3*i];
        previous[3*i] = x[3*i] - .1f; previous[3*i+1] = x[3*i+1] - .1f;
    }
    step.h = .1f;
    RigExecClothVelocity(mesh, step, 0, x.data(), previous.data(), velocity.data());
    CHECK(std::abs(velocity[0] - 1) < 1e-5f);
    CHECK(std::abs(velocity[1] - std::exp(-1.0f)) < 1e-5f);
}

void TestSingleIterationMatchesMulti()
{
    // The iterations==1 projection drops the lambda traffic: every lambda
    // read sees the substep fill (+0.0f). It must match the general form
    // with a zeroed lambda bitwise, including C == +0 (rest-length edges),
    // zero-length edges, and zero-denominator (kinematic-rigid) ones, on
    // the empty and material paths alike.
    const Mesh m = Curtain(3, 2.0f, 4.0f);
    std::vector<float> pins(m.Size(), 0.0f);
    pins[0] = pins[1] = pins[2] = 1.0f;
    RigExecClothSettings settings;
    RigExecClothMesh bare;
    CHECK(Build(m, &pins, settings, &bare));
    const auto same = [](const std::vector<float> &a, const std::vector<float> &b) {
        return a.size() == b.size() &&
               std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
    };
    std::vector<std::vector<float>> poses;
    poses.push_back(m.points);
    for (float scale : {0.5f, 1.5f}) {
        auto x = m.points;
        for (float &v : x) v *= scale;
        poses.push_back(x);
    }
    for (float scale : {1e-20f, 1e20f}) {
        auto x = m.points;
        for (size_t i = 0; i < m.Size(); ++i) x[3 * i] = m.points[3 * i] * scale;
        poses.push_back(x);
    }
    {
        // One coincident pair exercises the zero-length early return.
        auto x = m.points;
        const size_t a = size_t(bare.constraintEnds[0]);
        const size_t b = size_t(bare.constraintEnds[1]);
        x[3 * b] = x[3 * a];
        x[3 * b + 1] = x[3 * a + 1];
        x[3 * b + 2] = x[3 * a + 2];
        poses.push_back(x);
    }
    size_t compared = 0;
    const auto run = [&](RigExecClothMesh &mesh) {
        for (const auto &pose : poses) {
            for (float alpha : {0.0f, 1e-6f, 1e6f}) {
                for (float h : {1.0f / 24.0f, 1e-3f, 10.0f}) {
                    RigExecClothStep step;
                    step.h = h;
                    step.alphaStretch = alpha;
                    step.alphaBend = 0.5f * alpha;
                    for (size_t c = 0; c < mesh.constraintRest.size(); ++c) {
                        auto x = pose, y = pose;
                        std::vector<float> lambda(mesh.constraintRest.size(), 0.0f);
                        RigExecClothProjectConstraint(mesh, step, c, x.data(),
                                                     lambda.data());
                        RigExecClothProjectConstraintSingle(mesh, step, c, y.data());
                        CHECK(same(x, y));
                        ++compared;
                    }
                }
            }
        }
    };
    run(bare);
    for (int hard = 0; hard < 2; ++hard) {
        for (auto kind : {RigExecClothProfileKind::Stretch,
                          RigExecClothProfileKind::Bend}) {
            RigExecClothMesh mesh;
            CHECK(Build(m, &pins, settings, &mesh));
            RigExecClothProfile p;
            p.kind = kind;
            p.warp = 100;
            p.weft = .01f;
            p.compliance = .001f;
            p.hard = hard != 0;
            auto model = MaterialModel(mesh, m, p);
            std::string why;
            CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {},
                                           &why));
            run(mesh);
        }
    }
    CHECK(compared > 5000);
}

void TestColorSingleMatchesSerial()
{
    // The batched color projection runs each constraint's operations in
    // the same order as the serial per-constraint loop over disjoint
    // particles, so it must match bitwise: rest-length, stretched,
    // extreme-scale, zero-length, and signed-zero poses, on the empty
    // and material paths alike, across chunk-sized and multi-chunk
    // colors, and across split ranges.
    const auto same = [](const std::vector<float> &a, const std::vector<float> &b) {
        return a.size() == b.size() &&
               std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
    };
    const auto posesFor = [](const Mesh &m, const RigExecClothMesh &mesh) {
        std::vector<std::vector<float>> poses;
        poses.push_back(m.points);
        for (float scale : {0.5f, 1.5f}) {
            auto x = m.points;
            for (float &v : x) v *= scale;
            poses.push_back(x);
        }
        for (float scale : {1e-20f, 1e20f}) {
            auto x = m.points;
            for (size_t i = 0; i < m.Size(); ++i) x[3 * i] = m.points[3 * i] * scale;
            poses.push_back(x);
        }
        {
            // One coincident pair exercises the zero-length early return.
            auto x = m.points;
            const size_t a = size_t(mesh.constraintEnds[0]);
            const size_t b = size_t(mesh.constraintEnds[1]);
            x[3 * b] = x[3 * a];
            x[3 * b + 1] = x[3 * a + 1];
            x[3 * b + 2] = x[3 * a + 2];
            poses.push_back(x);
        }
        {
            // Signed zeros: a skipped constraint must not touch them, and
            // a zero-delta one must add the same signed zero both ways.
            auto x = m.points;
            const size_t b = size_t(mesh.constraintEnds[1]);
            x[3 * b] = -0.0f;
            x[3 * b + 1] = -0.0f;
            x[3 * b + 2] = -0.0f;
            poses.push_back(x);
        }
        return poses;
    };
    size_t compared = 0;
    // The solve's instantiation dispatch: the empty-model loop for an
    // empty model, the profile path otherwise.
    const auto projectColor = [](const RigExecClothMesh &mesh,
                                 const RigExecClothStep &step, uint32_t lo,
                                 uint32_t hi, float *x) {
        if (mesh.model.Empty())
            RigExecClothProjectColorSingle<false>(mesh, step, lo, hi, x);
        else
            RigExecClothProjectColorSingle<true>(mesh, step, lo, hi, x);
    };
    const auto run = [&](RigExecClothMesh &mesh,
                         const std::vector<std::vector<float>> &poses) {
        for (const auto &pose : poses) {
            for (float alpha : {0.0f, 1e-6f, 1e6f}) {
                for (float h : {1.0f / 24.0f, 1e-3f, 10.0f}) {
                    RigExecClothStep step;
                    step.h = h;
                    step.alphaStretch = alpha;
                    step.alphaBend = 0.5f * alpha;
                    for (size_t color = 0; color < mesh.ColorCount(); ++color) {
                        const uint32_t lo = mesh.colorStart[color];
                        const uint32_t hi = mesh.colorStart[color + 1];
                        auto x = pose, y = pose, z = pose;
                        for (uint32_t c = lo; c < hi; ++c)
                            RigExecClothProjectConstraintSingle(mesh, step, c, x.data());
                        projectColor(mesh, step, lo, hi, y.data());
                        CHECK(same(x, y));
                        // Split ranges re-chunk the color; the writes agree.
                        const uint32_t mid = lo + (hi - lo) / 2;
                        projectColor(mesh, step, lo, mid, z.data());
                        projectColor(mesh, step, mid, hi, z.data());
                        CHECK(same(x, z));
                        // An empty range writes nothing.
                        projectColor(mesh, step, lo, lo, z.data());
                        CHECK(same(x, z));
                        compared += 3;
                        // On an empty model both instantiations agree.
                        if (mesh.model.Empty()) {
                            auto w = pose;
                            RigExecClothProjectColorSingle<true>(mesh, step, lo, hi,
                                                                w.data());
                            CHECK(same(y, w));
                            compared += 1;
                        }
                    }
                }
            }
        }
    };
    {
        const Mesh m = Curtain(3, 2.0f, 4.0f);
        std::vector<float> pins(m.Size(), 0.0f);
        pins[0] = pins[1] = pins[2] = 1.0f;
        RigExecClothSettings settings;
        RigExecClothMesh bare;
        CHECK(Build(m, &pins, settings, &bare));
        run(bare, posesFor(m, bare));
        for (int hard = 0; hard < 2; ++hard) {
            for (auto kind : {RigExecClothProfileKind::Stretch,
                              RigExecClothProfileKind::Bend}) {
                RigExecClothMesh mesh;
                CHECK(Build(m, &pins, settings, &mesh));
                RigExecClothProfile p;
                p.kind = kind;
                p.warp = 100;
                p.weft = .01f;
                p.compliance = .001f;
                p.hard = hard != 0;
                auto model = MaterialModel(mesh, m, p);
                std::string why;
                CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {},
                                               &why));
                run(mesh, posesFor(m, mesh));
            }
        }
    }
    {
        // A mesh wide enough that colors span several chunks.
        const Mesh m = Curtain(17, 16.0f, 16.0f);
        std::vector<float> pins(m.Size(), 0.0f);
        for (int c = 0; c < 17; ++c) pins[size_t(c)] = 1.0f;
        RigExecClothSettings settings;
        RigExecClothMesh mesh;
        CHECK(Build(m, &pins, settings, &mesh));
        size_t widest = 0;
        for (size_t color = 0; color < mesh.ColorCount(); ++color)
            widest = std::max(widest, size_t(mesh.colorStart[color + 1] -
                                             mesh.colorStart[color]));
        CHECK(widest > 64);
        run(mesh, posesFor(m, mesh));
    }
    CHECK(compared > 5000);
}

void TestKinematicSkipPartition()
{
    // Each color's leading colorKkSkip entries are exactly its stretch
    // kinematic pairs, and projecting that leading run at zero stretch
    // alpha on an empty model leaves any pose bitwise unchanged (the
    // zero denominator returns before any write, NaN lengths included),
    // so the solve may start past it.
    const Mesh m = Curtain(9, 8.0f, 8.0f);
    std::vector<float> pins(m.Size(), 0.0f);
    for (int c = 0; c < 9; ++c) pins[size_t(c)] = 1.0f;
    RigExecClothSettings settings;
    RigExecClothMesh mesh;
    CHECK(Build(m, &pins, settings, &mesh));
    CHECK(mesh.model.Empty());
    CHECK(mesh.colorKkSkip.size() == mesh.ColorCount());
    const auto isKk = [&](uint32_t c) {
        const size_t a = size_t(mesh.constraintEnds[2 * c]);
        const size_t b = size_t(mesh.constraintEnds[2 * c + 1]);
        return mesh.constraintKind[c] == uint8_t(RigExecClothConstraint::Stretch) &&
               mesh.inverseMass[a] == 0.0f && mesh.inverseMass[b] == 0.0f;
    };
    size_t skipped = 0;
    for (size_t k = 0; k < mesh.ColorCount(); ++k) {
        const uint32_t lo = mesh.colorStart[k], hi = mesh.colorStart[k + 1];
        CHECK(mesh.colorKkSkip[k] <= hi - lo);
        for (uint32_t c = lo; c < lo + mesh.colorKkSkip[k]; ++c) CHECK(isKk(c));
        for (uint32_t c = lo + mesh.colorKkSkip[k]; c < hi; ++c) CHECK(!isKk(c));
        skipped += mesh.colorKkSkip[k];
    }
    CHECK(skipped == 8);
    const auto same = [](const std::vector<float> &a, const std::vector<float> &b) {
        return a.size() == b.size() &&
               std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
    };
    std::vector<std::vector<float>> poses;
    poses.push_back(m.points);
    {
        auto x = m.points;
        for (float &v : x) v *= 0.5f;
        poses.push_back(x);
    }
    {
        auto x = m.points;
        const size_t a = size_t(mesh.constraintEnds[0]);
        const size_t b = size_t(mesh.constraintEnds[1]);
        x[3 * b] = x[3 * a];
        x[3 * b + 1] = x[3 * a + 1];
        x[3 * b + 2] = x[3 * a + 2];
        poses.push_back(x);
    }
    {
        auto x = m.points;
        x[0] = -0.0f;
        x[1] = -0.0f;
        x[2] = -0.0f;
        poses.push_back(x);
    }
    {
        auto x = m.points;
        x[0] = std::numeric_limits<float>::quiet_NaN();
        poses.push_back(x);
    }
    RigExecClothStep step;
    step.h = 1.0f / 24.0f;
    step.alphaStretch = 0.0f;
    step.alphaBend = 0.5f;
    size_t compared = 0;
    for (const auto &pose : poses) {
        for (size_t color = 0; color < mesh.ColorCount(); ++color) {
            const uint32_t lo = mesh.colorStart[color];
            const uint32_t hi = mesh.colorStart[color + 1];
            const uint32_t mid = lo + mesh.colorKkSkip[color];
            auto x = pose, y = pose;
            for (uint32_t c = lo; c < mid; ++c)
                RigExecClothProjectConstraintSingle(mesh, step, c, x.data());
            CHECK(same(x, pose));
            for (uint32_t c = lo; c < hi; ++c)
                RigExecClothProjectConstraintSingle(mesh, step, c, x.data());
            // The mesh is empty (checked above), so the solve takes the
            // branch-free instantiation here.
            RigExecClothProjectColorSingle<false>(mesh, step, mid, hi, y.data());
            CHECK(same(x, y));
            // The general arm skips the same run: it returns before any
            // write, so positions and lambda come back untouched.
            auto z = pose;
            std::vector<float> lambda(mesh.constraintRest.size(), 0.0f);
            for (uint32_t c = lo; c < mid; ++c)
                RigExecClothProjectConstraint(mesh, step, c, z.data(), lambda.data());
            CHECK(same(z, pose));
            for (float l : lambda) CHECK(l == 0.0f);
            compared += 3;
        }
    }
    CHECK(compared > 100);
}

void TestHardSoftBarycentricAttachments()
{
    const Mesh m = Curtain(2, 2, 2);
    RigExecClothSettings settings;
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, settings, &mesh));
    mesh.inverseMass.assign(m.Size(), 1);
    RigExecClothProfile p;
    p.kind = RigExecClothProfileKind::Attachment;
    RigExecClothSample a, b = RigExecClothVertex(3);
    a.vertices = {{0,1,2}}; a.weights = {{.2f,.3f,.5f}};
    p.source = {a}; p.target = {b}; p.restLength = 0; p.compliance = 1;
    RigExecClothModel model; model.profiles = {p};
    std::string why;
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
    RigExecClothStep step; step.h = 1.0f / 24; step.t = 1;
    RigExecClothTick tick; tick.entering = m.points.data();
    float separation[2];
    for (int hard = 0; hard < 2; ++hard) {
        mesh.model.profiles[0].hard = hard != 0;
        std::vector<float> x = m.points;
        float lambda = 0;
        RigExecClothBindings(mesh, step, settings, tick, tick, x.data(), &lambda);
        float pa[3], pb[3], d[3];
        RigExecClothSamplePoint(a,x.data(),pa); RigExecClothSamplePoint(b,x.data(),pb);
        cloth_detail::Sub(pa,pb,d); separation[hard] = cloth_detail::Length(d);
        for (size_t axis = 0; axis < 3; ++axis) {
            float change = 0;
            for (size_t i = 0; i < m.Size(); ++i) change += x[3*i+axis] - m.points[3*i+axis];
            CHECK(std::abs(change) < 1e-5f);
        }
    }
    CHECK(separation[0] > .1f);
    CHECK(separation[1] < 1e-5f);
}

void TestSelfContactKeepsAttachments()
{
    Mesh m;
    m.points = {-2,-2,0, 2,-2,0, 0,2,0, -.5f,-.5f,.1f, .5f,-.5f,.1f, 0,.5f,.1f};
    m.counts = {3,3}; m.indices = {0,1,2,3,4,5};
    RigExecClothMesh mesh;
    RigExecClothSettings settings;
    CHECK(Build(m, nullptr, settings, &mesh));
    RigExecClothProfile attachment;
    attachment.kind = RigExecClothProfileKind::Attachment;
    attachment.hard = true; attachment.restLength = .1f;
    attachment.source = {RigExecClothVertex(3)};
    RigExecClothSample target;
    target.vertices = {{0,1,2}}; target.weights = {{.4375f,.1875f,.375f}};
    attachment.target = {target};
    RigExecClothProfile contact;
    contact.kind = RigExecClothProfileKind::SelfCollision;
    contact.hard = true; contact.thickness = .3f; contact.friction = 0;
    contact.sourceMask = {1,1,1,0,0,0}; contact.targetMask = {0,0,0,1,1,1};
    RigExecClothModel model; model.profiles = {attachment,contact};
    std::string why;
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
    std::vector<float> x = m.points;
    RigExecClothStep step; step.h = 1.0f / 24; step.t = 1; step.depth = 2;
    RigExecClothWorkspace ws;
    // An attachment exists and remains in the solve, yet its two regions
    // still receive contact impulses.
    RigExecClothSelfContacts(mesh, step, m.points.data(), &ws, x.data());
    CHECK(mesh.model.profiles[0].kind == RigExecClothProfileKind::Attachment);
    CHECK(x[3*3+2] > m.points[3*3+2]);
    CHECK(!ws.selfLambda.empty());
    for (size_t axis = 0; axis < 3; ++axis) {
        float momentum = 0;
        for (size_t i = 0; i < m.Size(); ++i)
            momentum += (x[3*i+axis] - m.points[3*i+axis]) / mesh.inverseMass[i];
        CHECK(std::abs(momentum) < 1e-4f);
    }
}

using PairKey = std::array<int32_t, 4>;

void TestPairSetMatchesOrderedSet()
{
    // The self-contact dedup set must give the same insert-or-duplicate
    // verdicts as an ordered set on every stream: sequential (crossing
    // growth), clustered duplicates, bench-scale, extreme values, and
    // brute-forced hash collisions that chain the probe and wrap the
    // mask. Replaying each stream after Clear must repeat the verdicts
    // exactly (stale slots stay unread).
    auto check = [](const std::vector<PairKey> &stream) {
        RigExecClothPairSet set;
        std::set<PairKey> ref;
        for (const PairKey &key : stream) {
            const bool want = ref.insert(key).second;
            CHECK(set.Insert(key) == want);
        }
        for (const PairKey &key : stream) CHECK(!set.Insert(key));
        set.Clear();
        ref.clear();
        for (const PairKey &key : stream) {
            const bool want = ref.insert(key).second;
            CHECK(set.Insert(key) == want);
        }
    };
    check({{{1, 2, 3, 4}}, {{1, 2, 3, 4}}, {{4, 3, 2, 1}}});
    std::vector<PairKey> sequential;
    for (int32_t i = 0; i < 40000; ++i)
        sequential.push_back({{i, i + 1, i + 2, i + 3}});
    check(sequential);
    // Clustered duplicates in a deterministic shuffle: a set that never
    // reports duplicates fails here.
    uint64_t state = 0x12345678u;
    const auto next = [&state]() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return state >> 33;
    };
    std::vector<PairKey> clustered;
    for (int round = 0; round < 50; ++round)
        for (int32_t i = 0; i < 100; ++i) clustered.push_back({{i, -i, 7, round % 5}});
    for (size_t i = clustered.size() - 1; i > 0; --i) {
        const size_t j = size_t(next() % (i + 1));
        std::swap(clustered[i], clustered[j]);
    }
    size_t dups = 0;
    {
        std::set<PairKey> ref;
        for (const PairKey &key : clustered) dups += !ref.insert(key).second;
    }
    CHECK(dups > 4000);
    check(clustered);
    check({{{0, 0, 0, 0}},
           {{std::numeric_limits<int32_t>::min(), -1, 0, 1}},
           {{std::numeric_limits<int32_t>::max(), 1, 0, -1}},
           {{std::numeric_limits<int32_t>::min(), -1, 0, 1}}});
    // Brute-forced collisions on one bucket, then on the last bucket so
    // the probe wraps: a broken probe or mask fails these.
    for (const uint32_t bucket : {uint32_t(0), uint32_t(4095)}) {
        std::vector<PairKey> colliding;
        for (int32_t i = 0; colliding.size() < 32; ++i) {
            const PairKey key = {{i, i * 31 + 7, -i, bucket}};
            if ((uint32_t(RigExecClothPairSet::Hash(key)) & 4095u) == bucket)
                colliding.push_back(key);
        }
        RigExecClothPairSet set;
        std::set<PairKey> ref;
        for (const PairKey &key : colliding) {
            const bool want = ref.insert(key).second;
            CHECK(set.Insert(key) == want);
        }
        CHECK(ref.size() == colliding.size());
        for (const PairKey &key : colliding) CHECK(!set.Insert(key));
    }
}

void TestEdgeSpheresRetireOnTouch()
{
    // The self edge/edge band caches midpoint spheres until a correction
    // moves an endpoint: a first fetch computes from the positions, a
    // touch elsewhere keeps the cached tick and values, and touching
    // either endpoint retires the sphere so the next fetch recomputes.
    Mesh m;
    m.points = {-1,-1,0, 1,-1,0, 0,1,0, -1,-1,.5f, 1,-1,.5f, 0,1,.5f};
    m.counts = {3,3}; m.indices = {0,1,2,3,4,5};
    RigExecClothMesh mesh;
    RigExecClothSettings settings;
    CHECK(Build(m, nullptr, settings, &mesh));
    const size_t edges = mesh.constraintRest.size() + mesh.triangles.size();
    RigExecClothEdgeSpheres spheres;
    spheres.centers.assign(3 * edges, 0.0f);
    spheres.radii.assign(edges, 0.0f);
    spheres.edgeTick.assign(edges, 0);
    spheres.vertTick.assign(m.Size(), 0);
    std::fill(spheres.vertTick.begin(), spheres.vertTick.end(), ++spheres.clock);
    std::vector<float> x = m.points;
    float c[3], r;
    cloth_detail::EdgeSphere(&spheres, 0, 0, 1, x.data(), c, &r);
    CHECK(c[0] == (x[0] + x[3]) * 0.5f && c[1] == (x[1] + x[4]) * 0.5f &&
          c[2] == (x[2] + x[5]) * 0.5f);
    const float e0 = x[3] - x[0], e1 = x[4] - x[1], e2 = x[5] - x[2];
    CHECK(r == 0.5f * std::sqrt(e0 * e0 + e1 * e1 + e2 * e2));
    const uint64_t first = spheres.edgeTick[0];
    CHECK(first == spheres.clock);
    // A touch that avoids both endpoints keeps the cached sphere, even
    // when other positions move.
    x[6] += 0.25f;
    cloth_detail::TouchVerts(&spheres, 2, 3, 4, 5);
    float kept[3], keptR;
    cloth_detail::EdgeSphere(&spheres, 0, 0, 1, x.data(), kept, &keptR);
    CHECK(spheres.edgeTick[0] == first);
    CHECK(kept[0] == c[0] && kept[1] == c[1] && kept[2] == c[2] && keptR == r);
    // Moving either endpoint and touching it retires the sphere.
    for (int32_t v : {0, 1}) {
        x[3 * size_t(v)] += 0.5f;
        cloth_detail::TouchVerts(&spheres, v, 3, 4, 5);
        float fresh[3], freshR;
        cloth_detail::EdgeSphere(&spheres, 0, 0, 1, x.data(), fresh, &freshR);
        CHECK(spheres.edgeTick[0] != first);
        CHECK(fresh[0] == (x[0] + x[3]) * 0.5f &&
              fresh[1] == (x[1] + x[4]) * 0.5f &&
              fresh[2] == (x[2] + x[5]) * 0.5f);
    }
}

void TestSelfGridReuseVerdict()
{
    // The edge/edge pass reuses the vertex/triangle grid exactly when no
    // projection moved x since it was built and the edge reach fits inside
    // the vertex/triangle pad. Equal clocks with a reach at the pad reuse
    // (both pads are that pad); any clock advance or overhang rebuilds.
    // A NaN reach reuses: std::max keeps the pad, so the inputs match.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CHECK(RigExecClothSelfGridReusable(7, 7, 0.5f, 1.0f));
    CHECK(RigExecClothSelfGridReusable(7, 7, 1.0f, 1.0f));
    CHECK(RigExecClothSelfGridReusable(7, 7, 0.0f, 0.0f));
    CHECK(RigExecClothSelfGridReusable(7, 7, nan, 1.0f));
    CHECK(RigExecClothSelfGridReusable(7, 7, 0.5f, inf));
    CHECK(!RigExecClothSelfGridReusable(7, 7, 1.5f, 1.0f));
    CHECK(!RigExecClothSelfGridReusable(7, 7, inf, 1.0f));
    CHECK(!RigExecClothSelfGridReusable(8, 7, 0.5f, 1.0f));
    CHECK(!RigExecClothSelfGridReusable(7, 8, 0.5f, 1.0f));
    CHECK(!RigExecClothSelfGridReusable(8, 7, 1.5f, 1.0f));
}

void TestClothGridDeterminism()
{
    // The grid-reuse verdict rests on the build being deterministic:
    // identical inputs rebuild identical buckets, cells, and bounds,
    // whatever the retained scratch held. A wider pad widens the cells.
    const float box[24] = {-8, 0, -8, 8, 0, -8, 8, 0, 8, -8, 0, 8,
                           -8, 15.5f, -8, 8, 15.5f, -8,
                           8, 15.5f, 8, -8, 15.5f, 8};
    const int counts[6] = {4, 4, 4, 4, 4, 4};
    const int indices[24] = {0, 1, 2, 3, 4, 7, 6, 5, 3, 2, 6, 7,
                             0, 4, 5, 1, 0, 3, 7, 4, 1, 5, 6, 2};
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(8, counts, 6, indices, 24, false, &colliders));
    float after[24];
    for (int i = 0; i < 24; ++i) after[i] = box[i] + 0.25f * float(i % 3 - 1);
    const auto same = [](const RigExecClothGrid &a, const RigExecClothGrid &b) {
        return a.cellSize == b.cellSize && a.bucketMask == b.bucketMask &&
               a.bucketStart == b.bucketStart &&
               a.bucketTriangles == b.bucketTriangles &&
               std::memcmp(a.sweptLo, b.sweptLo, sizeof(a.sweptLo)) == 0 &&
               std::memcmp(a.sweptHi, b.sweptHi, sizeof(a.sweptHi)) == 0;
    };
    RigExecClothGrid first, second;
    std::vector<uint64_t> scratch(64, 0xdeadbeefull);
    RigExecBuildClothGrid(colliders, box, after, 0.5f, &first, &scratch);
    scratch.assign(64, 0x12345678ull);
    RigExecBuildClothGrid(colliders, box, after, 0.5f, &second, &scratch);
    CHECK(same(first, second));
    RigExecClothGrid wide;
    RigExecBuildClothGrid(colliders, box, after, 20.0f, &wide, &scratch);
    CHECK(wide.cellSize > first.cellSize);
}

void TestSelfPairDiscoveryOrder()
{
    // A folded curtain layer over itself: vertex/triangle and edge/edge
    // corrections both fire. Dedup runs before the adjacency scans, so
    // the pair set also records adjacent pairs; the corrections that
    // reach projection, and the solved points, are pinned bit for bit.
    const int n = 4;
    Mesh m;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const bool top = j < n / 2;
            m.points.push_back(float(i) + (top ? 0.0f : 0.5f));
            m.points.push_back(top ? float(j) : float(n - 1 - j));
            m.points.push_back(top ? 0.0f : 0.3f);
        }
    for (int j = 0; j + 1 < n; ++j)
        for (int i = 0; i + 1 < n; ++i) {
            const int a = j * n + i;
            m.counts.push_back(3);
            m.indices.insert(m.indices.end(), {a, a + 1, a + n});
            m.counts.push_back(3);
            m.indices.insert(m.indices.end(), {a + 1, a + 1 + n, a + n});
        }
    RigExecClothMesh mesh;
    RigExecClothSettings settings;
    CHECK(Build(m, nullptr, settings, &mesh));
    RigExecClothProfile contact;
    contact.kind = RigExecClothProfileKind::SelfCollision;
    contact.hard = true;
    contact.thickness = 0.5f;
    contact.friction = 0.0f;
    RigExecClothModel model;
    model.profiles = {contact};
    std::string why;
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
    RigExecClothStep step;
    step.h = 1.0f / 24;
    step.depth = 0.15f;
    RigExecClothWorkspace ws;
    std::vector<float> x = m.points;
    RigExecClothSelfContacts(mesh, step, m.points.data(), &ws, x.data());
    size_t ve = 0, ee = 0;
    for (const auto &entry : ws.selfLambda) (entry.first[3] < 0 ? ve : ee)++;
    CHECK(ve == 7);
    CHECK(ee == 6);
    CHECK(ws.selfLambda.size() == 13);
    CHECK(ws.selfPairs.count == 351);
    uint64_t h = 1469598103934665603ull;
    const auto *p = reinterpret_cast<const uint8_t *>(x.data());
    for (size_t i = 0; i < x.size() * sizeof(float); ++i) {
        h ^= p[i];
        h *= 1099511628211ull;
    }
    CHECK(h == 0xc2fddc4fec45e38aull);
}

void TestNeighborBitsMatchLists()
{
    // The neighbor bitsets report the same relation as the sorted lists
    // for every vertex pair; past the size cap, or without a self-contact
    // profile, Prepare leaves them empty and the scans stay linear.
    const Mesh curtains[] = {Curtain(4, 3.0f, 2.0f), Curtain(2, 2.0f, 2.0f)};
    for (const Mesh &m : curtains) {
        RigExecClothMesh mesh;
        RigExecClothSettings settings;
        CHECK(Build(m, nullptr, settings, &mesh));
        RigExecClothProfile contact;
        contact.kind = RigExecClothProfileKind::SelfCollision;
        RigExecClothModel model;
        model.profiles = {contact};
        std::string why;
        CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
        CHECK(!mesh.neighborBits.empty());
        if (mesh.neighborBits.empty()) continue;
        CHECK(mesh.neighborBitStride == (m.Size() + 63) / 64);
        size_t compared = 0;
        for (size_t a = 0; a < m.Size(); ++a)
            for (size_t b = 0; b < m.Size(); ++b) {
                CHECK(cloth_detail::AdjacentToBits(mesh, int32_t(a), int32_t(b)) ==
                      cloth_detail::AdjacentTo(mesh.neighbors[a], int32_t(b)));
                ++compared;
            }
        CHECK(compared == m.Size() * m.Size());
    }
    // A model without self contact builds no sets either.
    {
        const Mesh m = Curtain(4, 3.0f, 2.0f);
        RigExecClothMesh mesh;
        RigExecClothSettings settings;
        CHECK(Build(m, nullptr, settings, &mesh));
        RigExecClothModel model;
        std::string why;
        CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
        CHECK(mesh.neighborBits.empty());
    }
    // Either side of the size cap: 2025 points build the sets, 2116 do
    // not (no solve: Prepare alone decides).
    for (const int n : {45, 46}) {
        const Mesh m = Curtain(n, 3.0f, 2.0f);
        CHECK(m.Size() == size_t(n * n));
        RigExecClothMesh mesh;
        RigExecClothSettings settings;
        CHECK(Build(m, nullptr, settings, &mesh));
        RigExecClothProfile contact;
        contact.kind = RigExecClothProfileKind::SelfCollision;
        RigExecClothModel model;
        model.profiles = {contact};
        std::string why;
        CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
        CHECK(mesh.neighborBits.empty() == (n == 46));
    }
}

void TestSelfEdgeCanonicalization()
{
    // Prepare stores each triangle edge sorted, and the discovery keys
    // them by one compare: with both edges sorted and four distinct
    // endpoints, (k,l) < (i,j) is exactly k < i. Constraints arrive
    // sorted from the build.
    const Mesh m = Curtain(4, 3.0f, 2.0f);
    RigExecClothMesh mesh;
    RigExecClothSettings settings;
    CHECK(Build(m, nullptr, settings, &mesh));
    for (size_t c = 0; c < mesh.constraintRest.size(); ++c)
        CHECK(mesh.constraintEnds[2 * c] < mesh.constraintEnds[2 * c + 1]);
    RigExecClothProfile contact;
    contact.kind = RigExecClothProfileKind::SelfCollision;
    RigExecClothModel model;
    model.profiles = {contact};
    std::string why;
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
    CHECK(mesh.triEdges.size() == 2 * mesh.triangles.size());
    for (size_t t = 0; t < mesh.triangles.size() / 3; ++t)
        for (size_t e = 0; e < 3; ++e) {
            const int32_t p = mesh.triangles[3 * t + e];
            const int32_t q = mesh.triangles[3 * t + (e + 1) % 3];
            CHECK(mesh.triEdges[6 * t + 2 * e] == std::min(p, q));
            CHECK(mesh.triEdges[6 * t + 2 * e + 1] == std::max(p, q));
        }
    size_t compared = 0;
    for (int32_t i = 0; i < 6; ++i)
        for (int32_t j = 0; j < 6; ++j)
            for (int32_t k = 0; k < 6; ++k)
                for (int32_t l = 0; l < 6; ++l) {
                    if (!(i < j && k < l)) continue;
                    if (i == k || i == l || j == k || j == l) continue;
                    CHECK((k < i) == (std::make_pair(k, l) < std::make_pair(i, j)));
                    ++compared;
                }
    CHECK(compared == 90);
}

void TestTriSpheresRetireOnTouch()
{
    // The vertex/triangle band caches centroid spheres until a correction
    // moves a corner: a first fetch computes from the positions, a touch
    // elsewhere keeps the cached tick and values, and touching any of the
    // three corners retires the sphere so the next fetch recomputes.
    // Freshness reuses the edge spheres' ticks and clock.
    Mesh m;
    m.points = {-1,-1,0, 1,-1,0, 0,1,0, -1,-1,.5f, 1,-1,.5f, 0,1,.5f};
    m.counts = {3,3}; m.indices = {0,1,2,3,4,5};
    RigExecClothMesh mesh;
    RigExecClothSettings settings;
    CHECK(Build(m, nullptr, settings, &mesh));
    RigExecClothEdgeSpheres spheres;
    spheres.vertTick.assign(m.Size(), 0);
    std::fill(spheres.vertTick.begin(), spheres.vertTick.end(), ++spheres.clock);
    RigExecClothTriSpheres tris;
    tris.centers.assign(6, 0.0f);
    tris.radii.assign(2, 0.0f);
    tris.triTick.assign(2, 0);
    std::vector<float> x = m.points;
    float c[3], r;
    cloth_detail::TriSphere(&tris, &spheres, 0, 0, 1, 2, x.data(), c, &r);
    CHECK(c[0] == (x[0] + x[3] + x[6]) / 3.0f &&
          c[1] == (x[1] + x[4] + x[7]) / 3.0f &&
          c[2] == (x[2] + x[5] + x[8]) / 3.0f);
    const uint64_t first = tris.triTick[0];
    CHECK(first == spheres.clock);
    x[9] += 0.25f;
    cloth_detail::TouchVerts(&spheres, 3, 4, 5, 3);
    float kept[3], keptR;
    cloth_detail::TriSphere(&tris, &spheres, 0, 0, 1, 2, x.data(), kept, &keptR);
    CHECK(tris.triTick[0] == first);
    CHECK(kept[0] == c[0] && kept[1] == c[1] && kept[2] == c[2] && keptR == r);
    for (int32_t v : {0, 1, 2}) {
        x[3 * size_t(v)] += 0.5f;
        cloth_detail::TouchVerts(&spheres, v, 3, 4, 5);
        float fresh[3], freshR;
        cloth_detail::TriSphere(&tris, &spheres, 0, 0, 1, 2, x.data(), fresh, &freshR);
        CHECK(tris.triTick[0] != first);
        CHECK(fresh[0] == (x[0] + x[3] + x[6]) / 3.0f &&
              fresh[1] == (x[1] + x[4] + x[7]) / 3.0f &&
              fresh[2] == (x[2] + x[5] + x[8]) / 3.0f);
    }
}

void TestMaterialCompiledRigRoundTrip()
{
    RigExecClothCompiledData compiledRig;
    const Mesh m = Curtain(2,2,2);
    compiledRig.pointCount = uint32_t(m.Size()); compiledRig.rest = m.points;
    compiledRig.counts = m.counts; compiledRig.indices = m.indices;
    RigExecClothProfile profile;
    profile.kind = RigExecClothProfileKind::Bend;
    profile.forceCurve = {0,1,100,0}; profile.minStrength = .2f;
    profile.maps[ClothMask] = {1,.5f,.25f,0};
    compiledRig.model.profiles = {profile};
    std::vector<uint8_t> bytes; RigExecEncodeClothCompiledData(compiledRig,&bytes);
    RigExecClothCompiledData decoded; std::string why;
    CHECK(RigExecDecodeClothCompiledData(bytes.data(),bytes.size(),&decoded,&why));
    CHECK(decoded == compiledRig);
    RigExecClothPrepared prepared;
    CHECK(RigExecPrepareCloth(decoded,&prepared,&why));
    bytes.pop_back();
    CHECK(!RigExecDecodeClothCompiledData(bytes.data(),bytes.size(),&decoded,&why));
    // RCL2 was never written by a released format: refused.
    compiledRig.model = RigExecClothModel();
    RigExecEncodeClothCompiledData(compiledRig,&bytes);
    CHECK(bytes[3] == '3');
    bytes.resize(bytes.size()-8); bytes[3] = '2';
    CHECK(!RigExecDecodeClothCompiledData(bytes.data(),bytes.size(),&decoded,&why) &&
          why == "not a cloth mover compiled rig");
}

// The bytes a format-25 writer wrote (RCL1): no maxWindowFrames and no
// material model. Written here rather than by RigExecEncodeClothCompiledData, so
// the layout stays pinned as the writer moves on.
std::vector<uint8_t> Format25CompiledData(const RigExecClothCompiledData &e)
{
    using namespace cloth_playback_detail;
    std::vector<uint8_t> out = {'R', 'C', 'L', '1'};
    PutU32(&out, e.pointCount);
    PutVector(&out, e.counts);
    PutVector(&out, e.indices);
    PutU32(&out, e.leftHanded ? 1u : 0u);
    PutVector(&out, e.rest);
    PutVector(&out, e.pins);
    PutU32(&out, uint32_t(e.settings.windowFrames));
    PutU32(&out, uint32_t(e.settings.substeps));
    PutU32(&out, uint32_t(e.settings.iterations));
    PutU32(&out, e.settings.tethers ? 1u : 0u);
    PutBytes(&out, &e.settings.tetherSlack, sizeof(float));
    PutBytes(&out, &e.settings.pinThreshold, sizeof(float));
    PutBytes(&out, &e.frameSeconds, sizeof(double));
    PutU32(&out, uint32_t(e.colliders.size()));
    for (const RigExecClothColliderTopology &c : e.colliders) {
        PutVector(&out, std::vector<char>(c.path.begin(), c.path.end()));
        PutU32(&out, c.pointCount);
        PutVector(&out, c.counts);
        PutVector(&out, c.indices);
        PutU32(&out, c.leftHanded ? 1u : 0u);
    }
    return out;
}

// A format-25 cloth plays as exported: its declared inputs at their
// format-25 slots (no rigExec:maxWindowFrames), no reach-back, and the same
// packets and solve as a current compiled rig whose reach-back is off.
void TestFormat25CompiledDataPlays()
{
    PushThrough scene;
    RigExecClothCompiledData current;
    current.pointCount = uint32_t(scene.curtain.Size());
    current.counts = scene.curtain.counts;
    current.indices = scene.curtain.indices;
    current.rest = scene.curtain.points;
    current.pins = scene.pins;
    current.settings.windowFrames = 8;
    current.settings.maxWindowFrames = 8;
    RigExecClothColliderTopology ball;
    ball.path = "/Ball";
    ball.pointCount = uint32_t(scene.ball.Size());
    ball.counts = scene.ball.counts;
    ball.indices = scene.ball.indices;
    current.colliders = {ball};

    // An RCL1 file cannot hold a longer reach-back: it reads as the window.
    RigExecClothCompiledData exported = current;
    exported.settings.maxWindowFrames = 64;
    const std::vector<uint8_t> old = Format25CompiledData(exported);
    RigExecClothCompiledData decoded;
    std::string why;
    bool legacy = false;
    CHECK(RigExecDecodeClothCompiledData(old.data(), old.size(), &decoded, &why, &legacy));
    CHECK(legacy && decoded == current);
    std::vector<uint8_t> now;
    RigExecEncodeClothCompiledData(current, &now);
    CHECK(RigExecDecodeClothCompiledData(now.data(), now.size(), &decoded, &why, &legacy));
    CHECK(!legacy && decoded == current);

    const RigExecExternalKernel kernel = RigExecClothPlaybackKernel();
    const auto oldState = kernel.prepare(old.data(), old.size(), &why);
    const auto newState = kernel.prepare(now.data(), now.size(), &why);
    CHECK(oldState && newState);
    if (!oldState || !newState) return;

    // The declared inputs in each format's order, with a distinct value per
    // parameter, so a misplaced slot changes the packet.
    const float values[] = {2e-6f, 3e-3f, 0.7f, 0.05f, 0.4f, 1.5f, 0.2f};
    const size_t params[] = {RigExecClothStretchCompliance, RigExecClothBendCompliance,
        RigExecClothDamping, RigExecClothAttract, RigExecClothThickness,
        RigExecClothCollisionDepth, RigExecClothFriction};
    const float gravity[3] = {0.0f, -500.0f, 0.0f};
    const auto inputs = [&](bool format25, const std::vector<float> &balls, bool damping) {
        const auto at = [&](size_t k) { return RigExecClothInputSlot(k, format25); };
        std::vector<RigExecExternalInputValue> in(at(RigExecClothFirstCollider) +
                                                  RigExecClothInputsPerCollider);
        for (size_t j = 0; j < 7; ++j) {
            if (params[j] == RigExecClothDamping && !damping) continue;
            in[at(params[j])] = {uint8_t(RigExecExternalInputType::Float), true, 1, &values[j]};
        }
        in[at(RigExecClothGravity)] = {uint8_t(RigExecExternalInputType::Vec3f), true, 1, gravity};
        in[at(RigExecClothFirstCollider)] = {uint8_t(RigExecExternalInputType::Vec3fArray), true,
                                             balls.size() / 3, balls.data()};
        return in;
    };
    const int t = 40;
    std::vector<RigExecWindowPacket> oldPackets, newPackets;
    for (int k = t - 8; k <= t; ++k) {
        const std::vector<float> &balls = scene.balls[size_t(k)];
        const auto a = inputs(true, balls, true), b = inputs(false, balls, true);
        RigExecWindowPacket pa, pb;
        CHECK(kernel.captureWindow(oldState.get(), nullptr, 0, a.data(), a.size(),
                                   scene.curtain.points.data(), scene.curtain.Size(), &pa));
        CHECK(kernel.captureWindow(newState.get(), nullptr, 0, b.data(), b.size(),
                                   scene.curtain.points.data(), scene.curtain.Size(), &pb));
        CHECK(pa == pb && pa.scalars.size() == RigExecClothParameterCount &&
              float(pa.scalars[5]) == 0.7f && float(pa.scalars[9]) == 0.2f);
        oldPackets.push_back(pa);
        newPackets.push_back(pb);
    }
    // An unread damping falls back to format 25's 0.1.
    {
        const auto a = inputs(true, scene.balls[size_t(t)], false);
        RigExecWindowPacket pa;
        CHECK(kernel.captureWindow(oldState.get(), nullptr, 0, a.data(), a.size(),
                                   scene.curtain.points.data(), scene.curtain.Size(), &pa));
        CHECK(pa.scalars.size() == RigExecClothParameterCount && float(pa.scalars[5]) == 0.1f);
    }
    std::vector<const RigExecWindowPacket *> oldTicks, newTicks;
    for (size_t k = 0; k < oldPackets.size(); ++k) {
        oldTicks.push_back(&oldPackets[k]);
        newTicks.push_back(&newPackets[k]);
    }
    std::vector<float> a = scene.curtain.points, b = scene.curtain.points;
    CHECK(kernel.applyWindow(oldState.get(), oldTicks.data(), oldTicks.size(), a.data(),
                             scene.curtain.Size()));
    CHECK(kernel.applyWindow(newState.get(), newTicks.data(), newTicks.size(), b.data(),
                             scene.curtain.Size()));
    CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0);
    CHECK(std::memcmp(a.data(), scene.curtain.points.data(), a.size() * sizeof(float)) != 0);
}

void TestEdgeContactAndModelValidation()
{
    Mesh m;
    // Overlapping triangle interiors, with all six vertices outside the
    // other triangle: only edge/edge contact can separate the layers.
    m.points = {-2,-1,0, 2,-1,0, 0,2,0, -2,1,.1f, 0,-2,.1f, 2,1,.1f};
    m.counts = {3,3}; m.indices = {0,1,2,3,4,5};
    RigExecClothMesh mesh; RigExecClothSettings settings;
    CHECK(Build(m, nullptr, settings, &mesh));
    RigExecClothProfile contact;
    contact.kind = RigExecClothProfileKind::SelfCollision;
    contact.hard = true; contact.thickness = .2f; contact.friction = 0;
    RigExecClothModel model; model.profiles = {contact};
    std::string why;
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(), {}, &why));
    RigExecClothStep step; step.h = 1.0f/24; step.depth = .4f;
    RigExecClothWorkspace ws; auto x = m.points;
    RigExecClothSelfContacts(mesh,step,m.points.data(),&ws,x.data());
    CHECK(!ws.selfLambda.empty());
    bool edge = false;
    for (const auto &entry : ws.selfLambda) edge = edge || (entry.first[3] >= 0 && entry.second > 0);
    CHECK(edge);
    float separation = 0;
    for (size_t i = 0; i < 3; ++i) separation += x[3*(i+3)+2] - x[3*i+2];
    CHECK(separation > .3f);
    model.profiles[0].warp = 2;
    CHECK(!RigExecPrepareClothModel(&mesh,model,m.points.data(),{},&why));
    model.triangleUV.assign(12, 0);
    CHECK(!RigExecPrepareClothModel(&mesh,model,m.points.data(),{},&why));
    model.triangleUV.clear(); model.profiles[0].warp = 1;
    model.profiles[0].restLength = -.5f;
    CHECK(!RigExecPrepareClothModel(&mesh,model,m.points.data(),{},&why));
}

}  // namespace

void TestPredictUsesGravityStep()
{
    // The predict pass takes the substep's gravity times its length from
    // the caller: a free particle gains exactly that step in velocity and
    // then moves by the new velocity times the length, a kinematic one is
    // placed on its entering points at the substep fraction, and both
    // record their previous position first.
    const Mesh m = Curtain(3, 2.0f, 4.0f);
    std::vector<float> pins(m.Size(), 0.0f);
    pins[0] = pins[1] = pins[2] = 1.0f;
    RigExecClothSettings settings;
    RigExecClothMesh mesh;
    CHECK(Build(m, &pins, settings, &mesh));
    const auto same = [](const std::vector<float> &a, const std::vector<float> &b) {
        return a.size() == b.size() &&
               std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
    };
    const float gravities[][3] = {{0.0f, 0.0f, 0.0f},
                                  {0.0f, -9.81f, 0.0f},
                                  {1.5f, -2.5f, 3.75f}};
    size_t compared = 0;
    for (const auto &g : gravities) {
        for (float h : {1.0f / 240.0f, 1e-6f, 2.0f}) {
            for (float t : {0.0f, 0.3f, 1.0f}) {
                RigExecClothStep step;
                step.h = h;
                step.t = t;
                step.gravity[0] = g[0];
                step.gravity[1] = g[1];
                step.gravity[2] = g[2];
                const float gravityStep[3] = {g[0] * h, g[1] * h, g[2] * h};
                auto from = m.points, to = m.points;
                for (size_t i = 0; i < m.Size(); ++i) to[3 * i] += 1.0f;
                for (size_t i = 0; i < m.Size(); ++i) {
                    auto x = m.points, wantX = m.points;
                    std::vector<float> xp(3 * m.Size(), 0.0f), wantXp(3 * m.Size(), 0.0f);
                    std::vector<float> v(3 * m.Size(), 0.0f), wantV(3 * m.Size(), 0.0f);
                    for (size_t k = 0; k < v.size(); ++k) {
                        v[k] = 0.25f * float(int(k) - 3);
                        wantV[k] = v[k];
                    }
                    for (int a = 0; a < 3; ++a)
                        wantXp[3 * i + size_t(a)] = m.points[3 * i + size_t(a)];
                    if (mesh.inverseMass[i] == 0.0f) {
                        for (int a = 0; a < 3; ++a)
                            wantX[3 * i + size_t(a)] = from[3 * i + size_t(a)] +
                                (to[3 * i + size_t(a)] - from[3 * i + size_t(a)]) * t;
                    } else {
                        for (int a = 0; a < 3; ++a) {
                            wantV[3 * i + size_t(a)] += gravityStep[a];
                            wantX[3 * i + size_t(a)] += wantV[3 * i + size_t(a)] * h;
                        }
                    }
                    RigExecClothPredict(mesh, step, from.data(), to.data(), i,
                                        x.data(), xp.data(), v.data(), gravityStep);
                    CHECK(same(x, wantX));
                    CHECK(same(xp, wantXp));
                    CHECK(same(v, wantV));
                    ++compared;
                }
            }
        }
    }
    CHECK(compared > 200);
}

void TestOutsideBoundsSkipsSearch()
{
    // The particle pass skips the contact search itself when the shared
    // broadphase flags the particle outside the swept bounds. That verdict
    // must imply an empty search (the direction the skip relies on): on and
    // off the boundary, far away, and for non-finite points. A grid that
    // never went through the build covers everything, so the precheck
    // never fires on it and the search runs as before.
    const float box[24] = {-8, 0, -8, 8, 0, -8, 8, 0, 8, -8, 0, 8,
                           -8, 15.5f, -8, 8, 15.5f, -8,
                           8, 15.5f, 8, -8, 15.5f, 8};
    const int counts[6] = {4, 4, 4, 4, 4, 4};
    const int indices[24] = {0, 1, 2, 3, 4, 7, 6, 5, 3, 2, 6, 7,
                             0, 4, 5, 1, 0, 3, 7, 4, 1, 5, 6, 2};
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(8, counts, 6, indices, 24, false, &colliders));
    float before[24], after[24];
    for (int i = 0; i < 24; ++i) {
        before[i] = box[i];
        after[i] = box[i] + (i % 3 == 0 ? 4.0f : (i % 3 == 1 ? -2.0f : 1.0f));
    }
    RigExecClothGrid grid, fresh;
    std::vector<uint64_t> scratch;
    RigExecBuildClothGrid(colliders, before, after, 8.0f, &grid, &scratch);
    const float thicknessDepth[][2] = {{0.5f, 2.0f}, {0.0f, 0.0f}, {2.0f, 0.5f}};
    std::vector<std::array<float, 3>> queries;
    queries.push_back({{0.0f, 8.0f, 0.0f}});
    for (const auto &td : thicknessDepth) {
        const float reach = std::max(td[0], td[1]);
        for (int a = 0; a < 3; ++a) {
            const float lo = grid.sweptLo[a] - reach;
            const float hi = grid.sweptHi[a] + reach;
            for (int k = 0; k <= 6; ++k) {
                std::array<float, 3> q = {{0.0f, 8.0f, 0.0f}};
                q[size_t(a)] = lo + (hi - lo) * float(k) / 6.0f;
                queries.push_back(q);
            }
            for (float edge : {lo, hi}) {
                for (float qv : {edge, std::nextafterf(edge, -1e30f),
                                 std::nextafterf(edge, 1e30f)}) {
                    std::array<float, 3> q = {{0.0f, 8.0f, 0.0f}};
                    q[size_t(a)] = qv;
                    queries.push_back(q);
                }
            }
            // Just outside the true box: a box derived with a larger
            // reach still covers these, so they discriminate the reach
            // the box was derived with.
            for (int side = 0; side < 2; ++side) {
                const double bound =
                    double(side == 0 ? grid.sweptLo[a] : grid.sweptHi[a]);
                const double margin =
                    (std::abs(bound) + double(reach)) * 9.5367431640625e-07;
                const double outside =
                    side == 0 ? bound - double(reach) - 2.0 * margin
                              : bound + double(reach) + 2.0 * margin;
                std::array<float, 3> q = {{0.0f, 8.0f, 0.0f}};
                q[size_t(a)] = float(outside);
                queries.push_back(q);
            }
        }
    }
    for (float far : {-1e6f, 1e6f}) queries.push_back({{far, 0.0f, 0.0f}});
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    queries.push_back({{nan, 0.0f, 0.0f}});
    queries.push_back({{0.0f, inf, 0.0f}});
    queries.push_back({{-inf, 0.0f, 0.0f}});
    size_t compared = 0;
    bool sawOutside = false, sawInside = false;
    for (const auto &td : thicknessDepth) {
        const float reach = std::max(td[0], td[1]);
        // The once-per-substep box agrees with the per-particle verdict
        // on every query, including the unbuilt grid that covers all.
        RigExecClothReachBounds built, unbuilt;
        RigExecClothSweptReachBounds(grid, reach, &built);
        RigExecClothSweptReachBounds(fresh, reach, &unbuilt);
        for (float t : {0.0f, 0.37f, 1.0f}) {
            for (const auto &q : queries) {
                CHECK(!RigExecClothOutsideSweptBounds(fresh, q.data(), reach));
                CHECK(!RigExecClothOutsideSweptReach(&unbuilt, q.data()));
                const bool outside =
                    RigExecClothOutsideSweptBounds(grid, q.data(), reach);
                CHECK(RigExecClothOutsideSweptReach(&built, q.data()) == outside);
                sawOutside = sawOutside || outside;
                sawInside = sawInside || !outside;
                const RigExecClothContact c = RigExecClothFindContact(
                    colliders, grid, before, after, t, q.data(), td[0], td[1]);
                if (outside) CHECK(c.triangle < 0);
                ++compared;
            }
        }
    }
    CHECK(compared > 1000);
    CHECK(sawOutside && sawInside);
}

void TestTetherLimitsPrecomputed()
{
    // The mesh carries the tether limit and its short-circuit bound
    // derived at build, and the particle pass reads them instead of
    // forming them per substep. Both must equal the documented formula
    // bitwise, on tethered and untethered builds alike.
    const Mesh m = Curtain(5, 4.0f, 8.0f);
    std::vector<float> pins(m.Size(), 0.0f);
    for (int c = 0; c < 5; ++c) pins[size_t(c)] = 1.0f;
    for (float slack : {0.0f, 0.05f, 1.0f}) {
        for (bool tethers : {true, false}) {
            RigExecClothSettings settings;
            settings.tetherSlack = slack;
            settings.tethers = tethers;
            RigExecClothMesh mesh;
            CHECK(Build(m, &pins, settings, &mesh));
            CHECK(mesh.tetherLimit.size() == m.Size());
            CHECK(mesh.tetherShortSquared.size() == m.Size());
            for (size_t i = 0; i < m.Size(); ++i) {
                const float limit = mesh.tetherRest[i] * (1.0f + settings.tetherSlack);
                const float limitSquared = limit * limit;
                const float shortSquared = (limit < 1e-18f || !std::isfinite(limitSquared))
                    ? -1.0f : limitSquared * (1.0f - 0.000244140625f);
                CHECK(std::memcmp(&limit, &mesh.tetherLimit[i], sizeof(float)) == 0);
                CHECK(std::memcmp(&shortSquared, &mesh.tetherShortSquared[i],
                                  sizeof(float)) == 0);
            }
        }
    }
}

void TestCollisionProfileContact()
{
    // A Collision profile drives the model contact path: the per-particle
    // sample selects the profile (Region reads it) and scales the response
    // (Strength reads it). A particle resting behind a collider plane is
    // pushed out; one beyond the reach is untouched.
    const Mesh m = Curtain(4, 3.0f, 1.0f);
    RigExecClothSettings settings;
    RigExecClothMesh mesh;
    CHECK(Build(m, nullptr, settings, &mesh));
    RigExecClothProfile p;
    p.kind = RigExecClothProfileKind::Collision;
    p.sourceMask.assign(m.Size(), 1.0f);
    auto model = MaterialModel(mesh, m, p);
    const Mesh plane = Sheet(2, 10.0f, -0.05f);
    std::string why;
    CHECK(RigExecPrepareClothModel(&mesh, model, m.points.data(),
                                   {plane.Size()}, &why));
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(plane.Size(), plane.counts.data(),
                                     plane.counts.size(), plane.indices.data(),
                                     plane.indices.size(), false, &colliders));
    RigExecClothParameters params;
    RigExecClothGrid grid;
    std::vector<uint64_t> scratch;
    RigExecBuildClothGrid(colliders, plane.points.data(), plane.points.data(),
                          std::max(params.thickness, params.collisionDepth),
                          &grid, &scratch);
    const RigExecClothStep step =
        RigExecClothStepFor(params, settings.substeps, 1, 1.0f / 24.0f);
    std::vector<float> x = m.points, previous = m.points;
    std::vector<float> contactLambda(m.Size(), 0.0f);
    std::vector<int32_t> contactTriangles(m.Size(), -1);
    // Row 1 sits at y = 0, just in front of the plane: contact pushes it up.
    const size_t near = 5;
    RigExecClothParticlePass(mesh, colliders, grid, settings, step,
                             m.points.data(), m.points.data(),
                             plane.points.data(), plane.points.data(), true,
                             near, x.data(), previous.data(),
                             contactLambda.data(), contactTriangles.data(),
                             nullptr);
    CHECK(x[3 * near + 1] > 0.0f);
    // Row 3 sits at y = -2, deep behind the plane: outside the violated
    // slab, so the pass leaves it alone.
    const size_t far = 13;
    RigExecClothParticlePass(mesh, colliders, grid, settings, step,
                             m.points.data(), m.points.data(),
                             plane.points.data(), plane.points.data(), true,
                             far, x.data(), previous.data(),
                             contactLambda.data(), contactTriangles.data(),
                             nullptr);
    CHECK(x[3 * far] == m.points[3 * far]);
    CHECK(x[3 * far + 1] == m.points[3 * far + 1]);
    CHECK(x[3 * far + 2] == m.points[3 * far + 2]);
}

int main()
{
    TestTopology();
    TestPinnedCurtainHangs();
    TestSheetLandsOnPlane();
    TestSphereSweepsThroughCurtain();
    TestFollowsInput();
    TestCarriedRigidly();
    TestClearOfColliders();
    TestClearOfCollidersBroadphase();
    TestClearOfCollidersBandCrossing();
    TestSweptBoundsAndBroadphase();
    TestSingleIterationMatchesMulti();
    TestColorSingleMatchesSerial();
    TestKinematicSkipPartition();
    TestPredictUsesGravityStep();
    TestOutsideBoundsSkipsSearch();
    TestTetherLimitsPrecomputed();
    TestCollisionProfileContact();
    TestMultiIterationSolves();
    TestWindowStartsBeforeContact();
    TestRefusals();
    TestForceResponseAndDirectionalMaterial();
    TestHardSoftBarycentricAttachments();
    TestSelfContactKeepsAttachments();
    TestPairSetMatchesOrderedSet();
    TestEdgeSpheresRetireOnTouch();
    TestSelfGridReuseVerdict();
    TestClothGridDeterminism();
    TestSelfPairDiscoveryOrder();
    TestNeighborBitsMatchLists();
    TestSelfEdgeCanonicalization();
    TestTriSpheresRetireOnTouch();
    TestMaterialCompiledRigRoundTrip();
    TestFormat25CompiledDataPlays();
    TestEdgeContactAndModelValidation();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecClothKernel: ok\n");
    return 0;
}
