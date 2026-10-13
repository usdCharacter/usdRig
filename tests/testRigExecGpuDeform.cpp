// The per-vertex GPU program (rigExecMath/gpuDeformProgram.h) against the
// CPU revision kernels it replaces, op by op and as one chain: matrix,
// radial matrix, linear and dual-quaternion skin, sparse blend shapes with
// in-betweens, dense and sparse wires. The program runs in float, the
// kernels partly in double, so agreement is to a tolerance relative to the
// mesh's size. Also checks that a frame whose static keys hold reuses the
// layout, and that a refused packet passes its points through. Where a GPU
// device is available the same frames run through the compute executor,
// which must agree with the CPU judge to float rounding.
#include "rigExec/gpuDeformLowering.h"
#include "rigExec/moverGraph.h"
#include "rigExec/types.h"
#include "rigExecGpu/gpuDeformExecutor.h"
#include "rigExecMath/gpuDeformProgram.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3f.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,    \
                         __LINE__, #cond);                                 \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)

namespace {

constexpr size_t kPoints = 2000;
// Points span about +-50 units; float rounding of a 4x3 affine there is a
// few 1e-6, and a chain compounds it.
constexpr float kTolerance = 2e-4f;

std::mt19937 g_random(1234);
uint64_t g_baseVersion = 0;

float
Uniform(float lo, float hi)
{
    return std::uniform_real_distribution<float>(lo, hi)(g_random);
}

std::vector<GfVec3f>
Mesh()
{
    std::vector<GfVec3f> points(kPoints);
    for (size_t i = 0; i < kPoints; ++i) {
        points[i] = GfVec3f(Uniform(-50, 50), Uniform(-50, 50), Uniform(-50, 50));
    }
    return points;
}

GfMatrix4d
RandomTransform(bool scaled)
{
    GfMatrix4d m(1.0);
    m.SetRotate(GfRotation(GfVec3d(Uniform(-1, 1), Uniform(-1, 1),
                                   Uniform(-1, 1)).GetNormalized(),
                           Uniform(-120, 120)));
    if (scaled) {
        GfMatrix4d s(1.0);
        s.SetScale(GfVec3d(Uniform(0.7f, 1.4f), Uniform(0.7f, 1.4f),
                           Uniform(0.7f, 1.4f)));
        m = s * m;
    }
    m.SetTranslateOnly(GfVec3d(Uniform(-20, 20), Uniform(-20, 20),
                               Uniform(-20, 20)));
    return m;
}

RigExecWeightPacket
FullStrength()
{
    return RigExecWeightPacket::Constant(1.0f);
}

RigExecWeightPacket
DenseWeights()
{
    RigExecWeightPacket w;
    w.representation = TfToken("dense");
    w.rangePolicy = TfToken("strict");
    w.values.resize(kPoints);
    for (size_t i = 0; i < kPoints; ++i) {
        // Mostly endpoints, as authored falloffs are.
        const float r = Uniform(0, 1);
        w.values[i] = r < 0.3f ? 0.0f : r < 0.6f ? 1.0f : Uniform(0, 1);
    }
    w.valid = true;
    return w;
}

RigExecWeightPacket
SparseWeights()
{
    RigExecWeightPacket w;
    w.representation = TfToken("sparse");
    w.rangePolicy = TfToken("strict");
    for (size_t i = 0; i < kPoints; i += 3) {
        w.indices.push_back(int(i));
        w.values.push_back(Uniform(0, 1));
    }
    w.defaultWeight = 0.0f;
    w.valid = true;
    return w;
}

float
MaxError(const std::vector<GfVec3f> &a, const std::vector<float> &b)
{
    float worst = 0.0f;
    for (size_t i = 0; i < a.size(); ++i)
        for (int k = 0; k < 3; ++k)
            worst = std::max(worst, std::fabs(a[i][k] - b[3 * i + k]));
    return worst;
}

std::vector<float>
Flat(const std::vector<GfVec3f> &points)
{
    std::vector<float> out(points.size() * 3);
    for (size_t i = 0; i < points.size(); ++i)
        for (int k = 0; k < 3; ++k) out[3 * i + k] = points[i][k];
    return out;
}

struct Revision {
    RigExecRevisionOp op;
    RigExecMoverParameters packet;
    std::vector<RigExecBlendChannel> channels;
};

RigExecMoverParameters
MatrixPacket(const RigExecWeightPacket &weights, bool radial)
{
    RigExecMoverParameters p;
    p.kind = TfToken("matrix");
    p.valid = true;
    p.transform = RandomTransform(false);
    p.radialWeight = radial;
    p.weights = weights;
    return p;
}

RigExecMoverParameters
SkinPacket(const char *method, bool scaled, size_t influences, int k)
{
    RigExecMoverParameters p;
    p.kind = TfToken("skin");
    p.valid = true;
    p.skinningMethod = TfToken(method);
    p.skinElementSize = k;
    p.weights = FullStrength();
    for (size_t t = 0; t < influences; ++t)
        p.skinTransforms.push_back(RandomTransform(scaled && t % 3 == 0));
    for (size_t i = 0; i < kPoints; ++i) {
        float total = 0.0f;
        for (int s = 0; s < k; ++s) {
            p.skinIndices.push_back(int((i * 7 + size_t(s) * 13) % influences));
            // Some points short of one, so the complement enters.
            float w = s == k - 1 ? std::max(0.0f, (i % 5 ? 1.0f : 0.8f) - total)
                                 : Uniform(0, 1.0f - total) * 0.6f;
            if (s == 1 && i % 4 == 0) w = 0.0f;
            p.skinWeights.push_back(w);
            total += w;
        }
    }
    return p;
}

std::vector<RigExecBlendChannel>
BlendChannels()
{
    std::vector<RigExecBlendChannel> channels;
    for (int c = 0; c < 4; ++c) {
        RigExecBlendChannel channel;
        channel.weight = c == 3 ? 0.0f : Uniform(0.1f, 1.3f);
        const int samples = 1 + c % 3;
        for (int s = 0; s < samples; ++s) {
            auto layout = std::make_shared<RigExecBlendSampleLayout>();
            layout->pointCount = kPoints;
            for (size_t i = size_t(c + s); i < kPoints; i += size_t(5 + c)) {
                layout->indices.push_back(int(i));
                layout->offsets.emplace_back(Uniform(-2, 2), Uniform(-2, 2),
                                             Uniform(-2, 2));
            }
            layout->valid = true;
            RigExecBlendSampleData sample;
            sample.activation = float(s + 1) / float(samples);
            sample.layout = layout;
            channel.samples.push_back(sample);
        }
        channels.push_back(channel);
    }
    return channels;
}

RigExecMoverParameters
BlendPacket(const std::vector<RigExecBlendChannel> &channels,
            const RigExecWeightPacket &weights)
{
    RigExecMoverParameters p;
    p.kind = TfToken("blendShape");
    p.valid = true;
    p.weights = weights;
    const std::vector<GfVec3f> base(kPoints, GfVec3f(0));
    CHECK(RigExecSumBlendChannels(channels, base, &p.blendDeltas));
    return p;
}

RigExecMoverParameters
WirePacket(const RigExecWeightPacket &weights, bool sparseBinds)
{
    RigExecMoverParameters p;
    p.kind = TfToken("wire");
    p.valid = true;
    p.curveOrder = 4;
    p.curveKnots = {0, 0, 0, 0, 0.3, 0.6, 1, 1, 1, 1};
    for (int j = 0; j < 6; ++j) {
        p.restPoints.emplace_back(float(j) * 15.0f - 40.0f, 0.0f, 0.0f);
        p.auxPoints.push_back(p.restPoints.back() +
                              GfVec3f(Uniform(-3, 3), Uniform(-3, 3), Uniform(-3, 3)));
    }
    const size_t binds = sparseBinds ? weights.indices.size() : kPoints;
    for (size_t i = 0; i < binds; ++i)
        p.wireBindCoords.push_back(GfVec2f(Uniform(0, 1), Uniform(0, 30)));
    p.dropoffDistance = 25.0;
    p.weights = weights;
    return p;
}

/// Runs \p revisions through the CPU kernels and through the lowered
/// program, and returns the worst coordinate difference (or a large number
/// where either path refused).
float
Compare(const std::vector<Revision> &revisions, RigExecGpuDeformLowering *lowering,
        bool expectApplies = true)
{
    const std::vector<GfVec3f> mesh = Mesh();
    std::vector<GfVec3f> cpu = mesh;
    std::vector<RigExecGpuRevisionInput> inputs;
    for (const Revision &r : revisions) {
        std::vector<GfVec3f> before = cpu;
        const bool applied =
            RigExecRunRevisionKernel(r.op, r.packet, &cpu, false, nullptr);
        if (!applied) cpu = before;
        CHECK(applied == expectApplies);
        RigExecGpuRevisionInput in;
        in.op = r.op;
        in.parameters = &r.packet;
        in.channels = r.op == RigExecRevisionOp::BlendShape ? &r.channels : nullptr;
        in.applies = applied;
        inputs.push_back(in);
    }
    RigExecGpuDeformFrame frame;
    std::string why;
    if (!lowering->Lower(inputs, kPoints, &frame, &why)) {
        std::fprintf(stderr, "lowering refused: %s\n", why.c_str());
        return 1e9f;
    }
    std::vector<float> judge = Flat(mesh);
    if (!RigExecRunGpuDeformProgram(frame, judge.data(), 0, kPoints)) return 1e9f;
    float error = MaxError(cpu, judge);
    if (RigExecGpuDeformExecutor *executor = RigExecGpuDeformExecutor::Get()) {
        const std::vector<float> base = Flat(mesh);
        std::vector<float> device(base.size(), 0.0f);
        std::vector<RigExecGpuDeformJob> jobs(1);
        jobs[0].chain = 1;
        jobs[0].frame = &frame;
        jobs[0].base = base.data();
        jobs[0].baseVersion = ++g_baseVersion;
        jobs[0].out = device.data();
        CHECK(executor->Run(&jobs));
        CHECK(jobs[0].ok);
        float worst = 0.0f;
        for (size_t k = 0; k < device.size(); ++k)
            worst = std::max(worst, std::fabs(device[k] - judge[k]));
        // The judge and the device evaluate the same float expressions;
        // contraction into fused multiply-adds and the device's sin/cos
        // (a radial cluster turns points 70 units out) differ in the last
        // bits, so the device is held to the kernels' tolerance.
        if (!(worst <= kTolerance)) {
            std::fprintf(stderr, "device vs judge %.3g\n", worst);
            ++g_failures;
        }
        error = std::max(error, MaxError(cpu, device));
    }
    return error;
}

float
CompareOne(RigExecRevisionOp op, const RigExecMoverParameters &packet,
           const std::vector<RigExecBlendChannel> &channels = {})
{
    RigExecGpuDeformLowering lowering;
    return Compare({{op, packet, channels}}, &lowering);
}

void
Report(const char *name, float error)
{
    std::printf("%-28s max error %.3g\n", name, error);
    if (!(error <= kTolerance)) {
        std::fprintf(stderr, "%s: error %.3g over %.3g\n", name, error,
                     kTolerance);
        ++g_failures;
    }
}

}  // namespace

/// A body-sized chain on the device: 26,276 points through correctives,
/// fifty clusters and a dual-quaternion skin, timed per run.
void
TimeBodySizedChain(RigExecGpuDeformExecutor *executor)
{
    const size_t n = 26276;
    std::vector<RigExecGpuRevisionInput> inputs;
    std::vector<std::unique_ptr<RigExecMoverParameters>> packets;
    std::vector<RigExecBlendChannel> channels;
    for (int c = 0; c < 160; ++c) {
        RigExecBlendChannel channel;
        channel.weight = Uniform(0, 1);
        auto layout = std::make_shared<RigExecBlendSampleLayout>();
        layout->pointCount = n;
        for (size_t i = size_t(c); i < n; i += 20) {
            layout->indices.push_back(int(i));
            layout->offsets.emplace_back(0.1f, 0.2f, 0.3f);
        }
        layout->valid = true;
        RigExecBlendSampleData sample;
        sample.layout = layout;
        channel.samples.push_back(sample);
        channels.push_back(channel);
    }
    const auto add = [&](RigExecRevisionOp op, RigExecMoverParameters p,
                         const std::vector<RigExecBlendChannel> *ch) {
        packets.push_back(std::make_unique<RigExecMoverParameters>(std::move(p)));
        RigExecGpuRevisionInput in;
        in.op = op;
        in.parameters = packets.back().get();
        in.channels = ch;
        inputs.push_back(in);
    };
    RigExecMoverParameters blend;
    blend.kind = TfToken("blendShape");
    blend.valid = true;
    blend.weights = FullStrength();
    add(RigExecRevisionOp::BlendShape, blend, &channels);
    for (int k = 0; k < 50; ++k) {
        RigExecMoverParameters m;
        m.kind = TfToken("matrix");
        m.valid = true;
        m.transform = RandomTransform(false);
        m.radialWeight = k % 2 == 0;
        m.weights.representation = TfToken("sparse");
        m.weights.rangePolicy = TfToken("strict");
        for (size_t i = size_t(k) * 300; i < std::min(n, size_t(k) * 300 + 900); ++i) {
            m.weights.indices.push_back(int(i));
            m.weights.values.push_back(0.5f);
        }
        m.weights.valid = true;
        add(RigExecRevisionOp::Matrix, m, nullptr);
    }
    RigExecMoverParameters skin;
    skin.kind = TfToken("skin");
    skin.valid = true;
    skin.skinningMethod = TfToken("dualQuaternion");
    skin.skinElementSize = 10;
    skin.weights = FullStrength();
    for (int t = 0; t < 137; ++t) skin.skinTransforms.push_back(RandomTransform(false));
    for (size_t i = 0; i < n; ++i)
        for (int s = 0; s < 10; ++s) {
            skin.skinIndices.push_back(int((i + size_t(s) * 11) % 137));
            skin.skinWeights.push_back(0.1f);
        }
    add(RigExecRevisionOp::Skin, skin, nullptr);

    RigExecGpuDeformLowering lowering;
    RigExecGpuDeformFrame frame;
    CHECK(lowering.Lower(inputs, n, &frame));
    std::vector<float> base(n * 3), out(n * 3);
    for (size_t i = 0; i < base.size(); ++i) base[i] = Uniform(-50, 50);
    std::vector<RigExecGpuDeformJob> jobs(1);
    jobs[0].chain = 2;
    jobs[0].frame = &frame;
    jobs[0].base = base.data();
    jobs[0].baseVersion = 1;
    jobs[0].out = out.data();
    double best = 1e9, total = 0, lowerMs = 0;
    for (int run = 0; run < 20; ++run) {
        packets[1]->transform = RandomTransform(false);
        const auto t0 = std::chrono::steady_clock::now();
        CHECK(lowering.Lower(inputs, n, &frame));
        lowerMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        CHECK(executor->Run(&jobs) && jobs[0].ok);
        const double ms = executor->LastRunMs();
        if (run > 0) { best = std::min(best, ms); total += ms; }
    }
    std::printf("body-sized chain: lower %.3f ms, device run best %.3f ms, "
                "mean %.3f ms; %zu ops, %zu static floats\n",
                lowerMs, best, total / 19.0, frame.layout->OpCount(),
                frame.layout->staticF.size());
    std::vector<float> judge = base;
    const auto t0 = std::chrono::steady_clock::now();
    RigExecRunGpuDeformProgram(frame, judge.data(), 0, n);
    std::printf("body-sized chain on one CPU thread (judge): %.3f ms\n",
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count());
    float worst = 0.0f;
    for (size_t k = 0; k < out.size(); ++k)
        worst = std::max(worst, std::fabs(out[k] - judge[k]));
    std::printf("body-sized chain device vs judge %.3g\n", worst);
    CHECK(worst <= 1e-4f);
}

int
main()
{
    if (RigExecGpuDeformExecutor *executor = RigExecGpuDeformExecutor::Get()) {
        std::printf("GPU device: %s\n", executor->Describe().c_str());
        TimeBodySizedChain(executor);
    } else {
        std::printf("GPU device unavailable: CPU judge only\n");
    }
    Report("matrix full strength",
           CompareOne(RigExecRevisionOp::Matrix, MatrixPacket(FullStrength(), false)));
    Report("matrix dense",
           CompareOne(RigExecRevisionOp::Matrix, MatrixPacket(DenseWeights(), false)));
    Report("matrix sparse",
           CompareOne(RigExecRevisionOp::Matrix, MatrixPacket(SparseWeights(), false)));
    Report("matrix radial dense",
           CompareOne(RigExecRevisionOp::Matrix, MatrixPacket(DenseWeights(), true)));
    Report("skin linear",
           CompareOne(RigExecRevisionOp::Skin, SkinPacket("classicLinear", true, 9, 4)));
    {
        RigExecMoverParameters p = SkinPacket("classicLinear", false, 9, 4);
        p.weights = DenseWeights();
        Report("skin linear enveloped", CompareOne(RigExecRevisionOp::Skin, p));
    }
    Report("skin dual quaternion rigid",
           CompareOne(RigExecRevisionOp::Skin, SkinPacket("dualQuaternion", false, 11, 6)));
    Report("skin dual quaternion scaled",
           CompareOne(RigExecRevisionOp::Skin, SkinPacket("dualQuaternion", true, 11, 6)));
    {
        const auto channels = BlendChannels();
        Report("blend full strength",
               CompareOne(RigExecRevisionOp::BlendShape,
                          BlendPacket(channels, FullStrength()), channels));
        Report("blend dense envelope",
               CompareOne(RigExecRevisionOp::BlendShape,
                          BlendPacket(channels, DenseWeights()), channels));
    }
    Report("wire dense",
           CompareOne(RigExecRevisionOp::Wire, WirePacket(DenseWeights(), false)));
    Report("wire full strength",
           CompareOne(RigExecRevisionOp::Wire, WirePacket(FullStrength(), false)));
    Report("wire sparse",
           CompareOne(RigExecRevisionOp::Wire, WirePacket(SparseWeights(), false)));
    Report("wire sparse binds",
           CompareOne(RigExecRevisionOp::Wire, WirePacket(SparseWeights(), true)));

    // One chain in a body's order: correctives, clusters, a wire, the skin.
    {
        const auto channels = BlendChannels();
        std::vector<Revision> chain = {
            {RigExecRevisionOp::BlendShape, BlendPacket(channels, FullStrength()), channels},
            {RigExecRevisionOp::Matrix, MatrixPacket(SparseWeights(), true), {}},
            {RigExecRevisionOp::Matrix, MatrixPacket(DenseWeights(), false), {}},
            {RigExecRevisionOp::Wire, WirePacket(SparseWeights(), false), {}},
            {RigExecRevisionOp::Skin, SkinPacket("dualQuaternion", true, 13, 8), {}},
        };
        RigExecGpuDeformLowering lowering;
        Report("chain", Compare(chain, &lowering));
        // Next frame: only the moving values change.
        chain[1].packet.transform = RandomTransform(false);
        chain[2].packet.transform = RandomTransform(false);
        chain[4].packet.skinTransforms[2] = RandomTransform(true);
        chain[0].channels[0].weight = 0.37f;
        chain[0].packet = BlendPacket(chain[0].channels, FullStrength());
        Report("chain next frame", Compare(chain, &lowering));
        CHECK(lowering.LayoutBuilds() == 1);
        // A new envelope is a static change.
        chain[2].packet.weights = DenseWeights();
        Report("chain new envelope", Compare(chain, &lowering));
        CHECK(lowering.LayoutBuilds() == 2);
    }

    // A refused packet passes its points through on both paths.
    {
        RigExecMoverParameters p = MatrixPacket(DenseWeights(), false);
        p.weights.values.pop_back();  // wrong cardinality
        RigExecGpuDeformLowering lowering;
        Report("refused matrix",
               Compare({{RigExecRevisionOp::Matrix, p, {}}}, &lowering, false));
    }

    if (g_failures) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
