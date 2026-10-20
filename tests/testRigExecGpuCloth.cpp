// RigExecClothMover's device solve (rigExec/gpuCloth.h, rigExecGpu): the
// GPU kernel against the CPU kernel on the same windows -- a curtain a
// sphere sweeps through, and a body-sized one for timing -- and a live rig
// whose cloth the publication boundary solves on the GPU, against the same
// rig solving on the CPU. The device agrees with the CPU to float rounding,
// not bit for bit, so it is held to a distance and to the collisions. With
// no OpenGL 4.5 device the suite says so and checks the CPU fallback alone.
#include "rigExecClothStage.h"
#include "rigExec/gpuCloth.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecGpu/gpuDeformExecutor.h"
#include "rigExecMath/clothKernel.h"

#include "pxr/base/plug/registry.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace rigExec;
using namespace rigExecClothStage;

namespace {

int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

void
SetEnv(const char *name, const char *value)
{
#ifdef _WIN32
    _putenv_s(name, value ? value : "");
#else
    if (value) setenv(name, value, 1); else unsetenv(name);
#endif
}

struct Mesh {
    std::vector<float> points;
    std::vector<int> counts, indices;
    size_t Size() const { return points.size() / 3; }
};

Mesh
Curtain(int n, float size, float top)
{
    Mesh m;
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c)
            m.points.insert(m.points.end(), {-size / 2 + size * float(c) / float(n - 1),
                                             top - size * float(r) / float(n - 1), 0.0f});
    for (int r = 0; r + 1 < n; ++r)
        for (int c = 0; c + 1 < n; ++c) {
            const int a = r * n + c;
            m.counts.push_back(4);
            m.indices.insert(m.indices.end(), {a, a + 1, a + n + 1, a + n});
        }
    return m;
}

Mesh
Sphere(float radius, int rings, int segments)
{
    Mesh m;
    m.points.insert(m.points.end(), {0.0f, radius, 0.0f});
    for (int r = 1; r < rings; ++r) {
        const float phi = 3.14159265f * float(r) / float(rings);
        for (int s = 0; s < segments; ++s) {
            const float theta = 6.2831853f * float(s) / float(segments);
            m.points.insert(m.points.end(), {radius * std::sin(phi) * std::cos(theta),
                                             radius * std::cos(phi),
                                             radius * std::sin(phi) * std::sin(theta)});
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

double
Ms(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

// A curtain of n x n points, its top row pinned, hanging still while a
// sphere sweeps through it from behind to in front over the window: the
// CPU kernel and the device solve the same window.
void
CompareSweep(RigExecGpuDeformExecutor *executor, int n, float radius, int rings,
             bool timing)
{
    const float size = 60.0f;
    const Mesh curtain = Curtain(n, size, 40.0f);
    std::vector<float> pins(curtain.Size(), 0.0f);
    for (int c = 0; c < n; ++c) pins[size_t(c)] = 1.0f;
    RigExecClothSettings settings;
    RigExecClothMesh mesh;
    std::string why;
    CHECK(RigExecBuildClothMesh(curtain.Size(), curtain.counts.data(), curtain.counts.size(),
                                curtain.indices.data(), curtain.indices.size(), false,
                                curtain.points.data(), pins.data(), settings, &mesh, &why));
    const Mesh sphere = Sphere(radius, rings, 2 * rings);
    RigExecClothColliders colliders;
    CHECK(RigExecAppendClothCollider(sphere.Size(), sphere.counts.data(), sphere.counts.size(),
                                     sphere.indices.data(), sphere.indices.size(), false,
                                     &colliders));
    const size_t ticks = size_t(settings.windowFrames) + 1;
    std::vector<std::vector<float>> spheres(ticks);
    std::vector<RigExecClothTick> window(ticks);
    const float centerY = 20.0f;
    for (size_t k = 0; k < ticks; ++k) {
        const float z = -30.0f + 45.0f * float(k) / float(ticks - 1);
        spheres[k] = sphere.points;
        for (size_t i = 0; i < sphere.Size(); ++i) {
            spheres[k][3 * i + 1] += centerY;
            spheres[k][3 * i + 2] += z;
        }
        window[k].entering = curtain.points.data();
        window[k].colliders = spheres[k].data();
    }
    std::vector<float> cpu(3 * mesh.pointCount), gpu(3 * mesh.pointCount);
    RigExecClothWorkspace ws;
    const auto t0 = std::chrono::steady_clock::now();
    CHECK(RigExecSolveCloth(mesh, colliders, settings, window.data(), window.size(),
                            1.0 / 24.0, &ws, cpu.data(), &why));
    const auto t1 = std::chrono::steady_clock::now();
    // The same window with one entering coordinate a millionth off: how far
    // float rounding alone can carry this scene.
    std::vector<float> nudged = curtain.points;
    nudged[3 * (mesh.pointCount / 2)] += 1e-6f;
    std::vector<RigExecClothTick> nudgedWindow = window;
    for (auto &tick : nudgedWindow) tick.entering = nudged.data();
    std::vector<float> control(3 * mesh.pointCount);
    CHECK(RigExecSolveCloth(mesh, colliders, settings, nudgedWindow.data(),
                            nudgedWindow.size(), 1.0 / 24.0, &ws, control.data(), &why));
    float spread = 0.0f;
    for (size_t i = 0; i < mesh.pointCount; ++i) {
        float d2 = 0.0f;
        for (int a = 0; a < 3; ++a) {
            const float d = control[3 * i + size_t(a)] - cpu[3 * i + size_t(a)];
            d2 += d * d;
        }
        spread = std::max(spread, std::sqrt(d2));
    }
    std::printf("cloth %d x %d: CPU vs CPU nudged 1e-6: %.3g\n", n, n, spread);
    if (!executor) return;
    std::vector<RigExecGpuClothJob> jobs(1);
    RigExecGpuClothJob &job = jobs[0];
    job.key = uint64_t(n);
    job.epochSerial = 1;
    job.mesh = &mesh;
    job.colliders = &colliders;
    job.settings = settings;
    job.frameSeconds = 1.0 / 24.0;
    job.ticks = window.data();
    job.tickCount = window.size();
    job.out = gpu.data();
    // The first run compiles the kernel and uploads the mesh.
    CHECK(executor->RunCloth(&jobs));
    CHECK(job.ok);
    const auto t2 = std::chrono::steady_clock::now();
    CHECK(executor->RunCloth(&jobs));
    CHECK(job.ok);
    const auto t3 = std::chrono::steady_clock::now();
    float worst = 0.0f, inside = 0.0f;
    double mean = 0.0;
    const float cz = -30.0f + 45.0f;
    for (size_t i = 0; i < mesh.pointCount; ++i) {
        float d2 = 0.0f;
        for (int a = 0; a < 3; ++a) {
            const float d = gpu[3 * i + size_t(a)] - cpu[3 * i + size_t(a)];
            d2 += d * d;
        }
        worst = std::max(worst, std::sqrt(d2));
        mean += std::sqrt(double(d2)) / double(mesh.pointCount);
        const float dx = gpu[3 * i], dy = gpu[3 * i + 1] - centerY, dz = gpu[3 * i + 2] - cz;
        inside = std::max(inside, radius - std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    std::printf("cloth %d x %d (%zu points, sphere %zu triangles): device vs CPU %.3g "
                "(mean %.3g), deepest inside the sphere %.3g\n",
                n, n, mesh.pointCount, colliders.triangles.size() / 3, worst, mean, inside);
    if (timing)
        std::printf("  CPU %.2f ms, GPU %.2f ms (first run %.2f ms)\n", Ms(t0, t1), Ms(t2, t3),
                    Ms(t1, t2));
    // Float rounding only: on average a small fraction of the spacing, and
    // nowhere more than half of it -- a point resting on a facet edge may
    // settle on the other facet -- and the device keeps the cloth out of
    // the sphere as the CPU does (the facets sit inside its radius).
    const float spacing = size / float(n - 1);
    CHECK(mean < 0.02 * spacing);
    CHECK(worst < 0.5f * spacing);
    CHECK(inside < 0.2f * radius);
}

// A small curtain with no collider, over a few ticks and substeps: the
// device and the CPU must agree closely before collisions enter.
void
CompareFree(RigExecGpuDeformExecutor *executor, int ticks, int substeps, int n = 6)
{
    if (!executor) return;
    const Mesh curtain = Curtain(n, 20.0f, 40.0f);
    std::vector<float> pins(curtain.Size(), 0.0f);
    for (int c = 0; c < n; ++c) pins[size_t(c)] = 1.0f;
    RigExecClothSettings settings;
    settings.substeps = substeps;
    RigExecClothMesh mesh;
    std::string why;
    CHECK(RigExecBuildClothMesh(curtain.Size(), curtain.counts.data(), curtain.counts.size(),
                                curtain.indices.data(), curtain.indices.size(), false,
                                curtain.points.data(), pins.data(), settings, &mesh, &why));
    RigExecClothColliders colliders;
    std::vector<std::vector<float>> entering(size_t(ticks), curtain.points);
    std::vector<RigExecClothTick> window(static_cast<size_t>(ticks));
    for (int k = 0; k < ticks; ++k) {
        for (size_t i = 0; i < curtain.Size(); ++i) entering[size_t(k)][3 * i] += 2.0f * float(k);
        window[size_t(k)].entering = entering[size_t(k)].data();
    }
    std::vector<float> cpu(3 * mesh.pointCount), gpu(3 * mesh.pointCount);
    RigExecClothWorkspace ws;
    CHECK(RigExecSolveCloth(mesh, colliders, settings, window.data(), window.size(),
                            1.0 / 24.0, &ws, cpu.data(), &why));
    std::vector<RigExecGpuClothJob> jobs(1);
    jobs[0].key = 1000 + uint64_t(ticks * 100 + substeps) + uint64_t(n) * 100000;
    jobs[0].epochSerial = 1;
    jobs[0].mesh = &mesh;
    jobs[0].colliders = &colliders;
    jobs[0].settings = settings;
    jobs[0].ticks = window.data();
    jobs[0].tickCount = window.size();
    jobs[0].out = gpu.data();
    CHECK(executor->RunCloth(&jobs) && jobs[0].ok);
    float worst = 0.0f;
    for (size_t i = 0; i < cpu.size(); ++i) worst = std::max(worst, std::fabs(cpu[i] - gpu[i]));
    std::printf("free curtain, %d ticks x %d substeps: device vs CPU %.3g\n", ticks, substeps,
                worst);
    CHECK(worst < 1e-3f);
}

// The live rig: its cloth solves on the device at the publication boundary
// unless RIGEXEC_GPU_CLOTH=0, and the two agree to float rounding.
void
CompareRig(RigExecGpuDeformExecutor *executor)
{
    RigExecGpuRegisterClothRunner();
    UsdStageRefPtr stage = MakeStage();
    SetEnv("RIGEXEC_GPU_CLOTH", "0");
    RigExecRigEvaluator cpu(stage, kRig);
    CHECK(cpu.Compile());
    SetEnv("RIGEXEC_GPU_CLOTH", nullptr);
    // By default a cloth this small stays on the CPU: no device solve.
    {
        RigExecRigEvaluator small(stage, kRig);
        CHECK(small.Compile());
        const size_t before = executor ? executor->ClothSolveCount() : 0;
        const Points a = PointsOf(cpu.Evaluate(UsdTimeCode(13)), kCurtain);
        const Points b = PointsOf(small.Evaluate(UsdTimeCode(13)), kCurtain);
        CHECK(SameBits(a, b));
        if (executor) CHECK(executor->ClothSolveCount() == before);
    }
    SetEnv("RIGEXEC_GPU_CLOTH_MIN_POINTS", "0");
    RigExecRigEvaluator device(stage, kRig);
    CHECK(device.Compile());
    SetEnv("RIGEXEC_GPU_CLOTH_MIN_POINTS", nullptr);
    const size_t before = executor ? executor->ClothSolveCount() : 0;
    float worst = 0.0f;
    for (int t : {6, 13, 20, 27, 34}) {
        const Points a = PointsOf(cpu.Evaluate(UsdTimeCode(t)), kCurtain);
        const Points b = PointsOf(device.Evaluate(UsdTimeCode(t)), kCurtain);
        CHECK(!a.empty() && a.size() == b.size());
        worst = std::max(worst, Furthest(a, b));
    }
    // A drag solves again at the boundary.
    RigExecValueOverride drag;
    drag.prim = kRoot;
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(-6.0);
    cpu.SetInteractiveOverrides({drag});
    device.SetInteractiveOverrides({drag});
    worst = std::max(worst, Furthest(PointsOf(cpu.Evaluate(UsdTimeCode(14)), kCurtain),
                                     PointsOf(device.Evaluate(UsdTimeCode(14)), kCurtain)));
    std::printf("rig: device vs CPU %.3g\n", worst);
    CHECK(worst < 0.05f * kSpacing);
    if (executor) {
        // Every evaluation of the device rig went to the GPU.
        CHECK(executor->ClothSolveCount() >= before + 6);
    }
}

}  // namespace

int
main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    RigExecGpuDeformExecutor *executor = RigExecGpuDeformExecutor::Get();
    if (executor) {
        std::printf("GPU device: %s\n", executor->Describe().c_str());
    } else {
        std::printf("GPU device unavailable: the CPU fallback alone\n");
    }
    CompareFree(executor, 2, 1);
    CompareFree(executor, 3, 4);
    CompareFree(executor, 17, 10);
    CompareFree(executor, 17, 10, 96);
    CompareSweep(executor, 24, 12.0f, 12, true);
    CompareSweep(executor, 48, 12.0f, 24, true);
    CompareSweep(executor, 96, 14.0f, 48, true);
    CompareRig(executor);
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecGpuCloth: all tests passed\n");
    return 0;
}
