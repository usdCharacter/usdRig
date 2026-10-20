// clothKernel.h on small scenes whose answers are known without the solver:
// a pinned curtain hangs and keeps its edge lengths, a free sheet lands on a
// plane and stays above it, a sphere swept through a curtain leaves no
// particle inside, full pins and full attraction follow the input exactly,
// rigid input motion is carried rigidly, and repeated solves are
// bit-identical. Header-only and USD-free.
#include "rigExecMath/clothKernel.h"

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

}  // namespace

int main()
{
    TestTopology();
    TestPinnedCurtainHangs();
    TestSheetLandsOnPlane();
    TestSphereSweepsThroughCurtain();
    TestFollowsInput();
    TestCarriedRigidly();
    TestRefusals();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecClothKernel: ok\n");
    return 0;
}
