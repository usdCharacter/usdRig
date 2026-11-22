// The point ops evaluated through the core give the bits the baked
// evaluator's Gf kernels give, envelope blend included.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecOps/deform/pointOps.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

using namespace rigExec;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;
using Points = RigExecArray<Vec3f>;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

/// A jittered n x n grid of quads, and its topology.
struct Grid {
    std::vector<GfVec3f> points;
    std::vector<int> counts, indices;

    Grid(int n, unsigned seed, bool flat = false)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> jitter(-0.3f, 0.3f);
        for (int y = 0; y < n; ++y)
            for (int x = 0; x < n; ++x)
                points.emplace_back(x + jitter(rng), y + jitter(rng),
                                    flat ? 0.0f : jitter(rng) * 4.0f);
        for (int y = 0; y + 1 < n; ++y)
            for (int x = 0; x + 1 < n; ++x) {
                counts.push_back(4);
                const int i = y * n + x;
                indices.insert(indices.end(), {i, i + 1, i + n + 1, i + n});
            }
    }
};

Points
ToStd(const std::vector<GfVec3f> &v)
{
    Points out(v.size());
    for (size_t i = 0; i < v.size(); ++i)
        out[i] = Vec3f(v[i][0], v[i][1], v[i][2]);
    return out;
}

template <class T>
RigExecArray<T>
ToArray(const std::vector<T> &v)
{
    return RigExecArray<T>(v.data(), v.size());
}

bool
SameBits(const std::vector<GfVec3f> &expected, const Points &actual)
{
    return expected.size() == actual.Size() &&
           (expected.empty() ||
            std::memcmp(expected.data(), actual.Data(),
                        expected.size() * sizeof(GfVec3f)) == 0);
}

void
Blend(const std::vector<GfVec3f> &in, const std::vector<float> &envelope,
      std::vector<GfVec3f> *out)
{
    for (size_t i = 0; i < in.size(); ++i)
        (*out)[i] = RigExecBlendEnvelope(in[i], (*out)[i], envelope[i]);
}

/// One op node with the listed ports bound, the first output read.
struct Node {
    std::unique_ptr<RigExecEvaluator> evaluator;
    std::vector<RigExecInputId> inputs;
    RigExecOutputId out;

    Node(const RigExecOpHandle &op,
         const std::vector<const RigExecPortHandle *> &ins,
         const RigExecPortHandle &output)
    {
        RigExecRigDescBuilder b;
        const RigExecNodeId node = b.AddOp(op);
        for (const RigExecPortHandle *port : ins)
            inputs.push_back(b.BindInput(node, *port));
        out = b.BindOutput(node, output);
        std::string error;
        auto desc = b.Build(&error);
        auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
        evaluator =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
        if (!evaluator)
            std::printf("node: %s\n", error.c_str());
        CHECK(evaluator);
    }

    bool Run(const std::vector<const void *> &values, Points *result,
             std::string *error = nullptr)
    {
        if (!evaluator)
            return false;
        for (size_t i = 0; i < values.size(); ++i)
            if (!evaluator->SetInput(inputs[i], values[i], error))
                return false;
        if (!evaluator->Evaluate(error))
            return false;
        *result = *static_cast<const Points *>(evaluator->GetOutput(out));
        return true;
    }
};

void
TestSmooth()
{
    using namespace rigExecOps;
    Node node(Smooth(),
              {&SmoothPoints(), &SmoothCounts(), &SmoothIndices(),
               &SmoothStrength(), &SmoothEnvelope()},
              SmoothOut());
    Grid grid(9, 3);
    std::vector<float> envelope(grid.points.size());
    for (size_t i = 0; i < envelope.size(); ++i)
        envelope[i] = i % 5 == 0 ? 0.0f : i % 7 == 0 ? 1.0f : 0.13f * (i % 8);
    const Points in = ToStd(grid.points);
    const auto envelopeArray = ToArray(envelope);
    for (const float strength : {1.0f, 0.5f, 0.0f, 1.5f, -0.25f})
        for (const bool broken : {false, true}) {
            std::vector<int> indices = grid.indices;
            if (broken)
                indices[5] = int(grid.points.size()); // addresses no point
            std::vector<GfVec3f> expected = grid.points;
            RigExecApplyLaplacianSmooth(&expected, grid.counts, indices,
                                        strength);
            Blend(grid.points, envelope, &expected);
            const auto counts = ToArray(grid.counts);
            const auto indexArray = ToArray(indices);
            Points actual;
            CHECK(
                node.Run({&in, &counts, &indexArray, &strength, &envelopeArray},
                         &actual));
            CHECK(SameBits(expected, actual));
        }
    // Unfed strength and envelope apply the smoothing outright.
    Node bare(Smooth(), {&SmoothPoints(), &SmoothCounts(), &SmoothIndices()},
              SmoothOut());
    std::vector<GfVec3f> expected = grid.points;
    RigExecApplyLaplacianSmooth(&expected, grid.counts, grid.indices, 1.0);
    const auto counts = ToArray(grid.counts);
    const auto indices = ToArray(grid.indices);
    Points actual;
    CHECK(bare.Run({&in, &counts, &indices}, &actual));
    CHECK(SameBits(expected, actual));
    // An envelope of another length is refused.
    const float one = 1.0f;
    const RigExecArray<float> shortEnvelope(3);
    std::string error;
    CHECK(!node.Run({&in, &counts, &indices, &one, &shortEnvelope}, &actual,
                    &error));
    CHECK(error.find("envelope") != std::string::npos);
}

void
TestVolumeCorrect()
{
    using namespace rigExecOps;
    Node node(VolumeCorrect(),
              {&VolumeCorrectPoints(), &VolumeCorrectReference(),
               &VolumeCorrectStrength(), &VolumeCorrectEnvelope()},
              VolumeCorrectOut());
    for (const bool flat : {false, true}) {
        Grid grid(7, 11, flat);
        const Points in = ToStd(grid.points);
        std::vector<float> envelope(grid.points.size());
        for (size_t i = 0; i < envelope.size(); ++i)
            envelope[i] = 0.1f * (i % 12);
        const auto envelopeArray = ToArray(envelope);
        const double bound =
            RigExecBoundVolume(grid.points.data(), grid.points.size());
        for (const double reference : {bound * 1.7, bound * 0.4, 0.0})
            for (const float strength : {1.0f, 0.35f, 0.0f}) {
                std::vector<GfVec3f> expected = grid.points;
                RigExecApplyVolumeCorrect(&expected, reference, strength);
                Blend(grid.points, envelope, &expected);
                Points actual;
                CHECK(node.Run({&in, &reference, &strength, &envelopeArray},
                               &actual));
                CHECK(SameBits(expected, actual));
            }
    }
}

} // namespace

int
main()
{
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(RigExecLinkedRegistrations(), &errors)) {
        for (const std::string &e : errors)
            std::printf("%s\n", e.c_str());
        return 1;
    }
    TestSmooth();
    TestVolumeCorrect();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
