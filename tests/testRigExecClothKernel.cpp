// clothKernel.h on small scenes whose answers are known without the solver:
// a pinned curtain hangs and keeps its edge lengths, a free sheet lands on a
// plane and stays above it, a sphere swept through a curtain leaves no
// particle inside, full pins and full attraction follow the input exactly,
// rigid input motion is carried rigidly, repeated solves are bit-identical,
// and a window whose first frame has a collider cutting through the cloth
// starts before the contact, so a ball pushed through a curtain keeps it in
// front frame after frame. A format-25 (RCL1) compiled rig plays as it was
// exported. Header-only and USD-free.
#include "rigExecMath/clothKernel.h"
#include "rigExecBinary/clothPlayback.h"

#include <cmath>
#include <cstdio>
#include <cstring>
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
    // Behind the curtain, through it, and on the far side of it.
    for (int k = 0; k < PushThrough::kFrames; ++k) {
        const float z = PushThrough::BallZ(k);
        const bool cuts = std::fabs(z) < 6.0f;
        const bool clear = RigExecClothClearOfColliders(mesh, colliders, scene.curtain.points.data(),
                                                        scene.balls[size_t(k)].data(), &grid,
                                                        &scratch);
        // Within a facet's sag of the radius either answer is right.
        if (std::fabs(std::fabs(z) - 6.0f) > 0.3f) CHECK(clear == !cuts);
    }
    // No colliders: always clear.
    CHECK(RigExecClothClearOfColliders(mesh, RigExecClothColliders(), scene.curtain.points.data(),
                                       nullptr, &grid, &scratch));
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

int main()
{
    TestTopology();
    TestPinnedCurtainHangs();
    TestSheetLandsOnPlane();
    TestSphereSweepsThroughCurtain();
    TestFollowsInput();
    TestCarriedRigidly();
    TestClearOfColliders();
    TestWindowStartsBeforeContact();
    TestRefusals();
    TestForceResponseAndDirectionalMaterial();
    TestHardSoftBarycentricAttachments();
    TestSelfContactKeepsAttachments();
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
