// The point ops evaluated through the core give the bits the baked
// evaluator's Gf kernels give, envelope blend included.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExec/api/runState.h"
#include "rigExec/api/taskSystem.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/moverTransformKernel.h"
#include "rigExecMath/normalsLayoutKernel.h"
#include "rigExecMath/ribbonFramesKernel.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/skinKernel.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"
#include "rigExecMath/surfaceProjectorKernel.h"
#include "rigExecMath/weightFields.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecOps/constraint/autoClavicleOps.h"
#include "rigExecOps/constraint/constraintOps.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecOps/solver/chainSolverOps.h"
#include "rigExecOps/solver/frameBlendOps.h"
#include "rigExecOps/solver/poseCommitOps.h"
#include "rigExecOps/solver/ribbonFramesOps.h"
#include "rigExecOps/solver/singleChainIkOps.h"
#include "rigExecOps/solver/splineIkOps.h"
#include "rigExecOps/deform/blendShapeOp.h"
#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/derivedOps.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/deform/projectorOps.h"
#include "rigExecOps/deform/ribbonOp.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecOps/deform/surfaceProjectOp.h"
#include "rigExecOps/deform/wireOp.h"
#include "rigExecOps/deform/wrinkleOp.h"
#include "rigExecOps/weight/weightOps.h"

#include "pxr/base/gf/rotation.h"

#include "opAllocationCount.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <type_traits>
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

bool
SamePoints(const Points &a, const Points &b)
{
    return a.Size() == b.Size() &&
           (a.Empty() ||
            std::memcmp(a.Data(), b.Data(), a.Size() * sizeof(Vec3f)) == 0);
}

/// Nodes of \p op, one per payload in \p payloads, each fed \p values on
/// \p ins in one rig; the outputs, in payload order.
template <class P>
bool
RunEach(const RigExecOpHandle &op,
        const std::vector<const RigExecPortHandle *> &ins,
        const RigExecPortHandle &output, const std::vector<P> &payloads,
        const std::vector<const void *> &values, std::vector<Points> *results)
{
    RigExecRigDescBuilder b;
    std::vector<std::vector<RigExecInputId>> inputs;
    std::vector<RigExecOutputId> outs;
    for (const P &payload : payloads) {
        const RigExecNodeId node = b.AddOp(op, &payload);
        inputs.emplace_back();
        for (const RigExecPortHandle *port : ins)
            inputs.back().push_back(b.BindInput(node, *port));
        outs.push_back(b.BindOutput(node, output));
    }
    std::string error;
    auto desc = b.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    if (!evaluator) {
        std::printf("nodes: %s\n", error.c_str());
        return false;
    }
    for (const auto &nodeInputs : inputs)
        for (size_t i = 0; i < values.size(); ++i)
            if (!evaluator->SetInput(nodeInputs[i], values[i], &error))
                return false;
    if (!evaluator->Evaluate(&error)) {
        std::printf("nodes: %s\n", error.c_str());
        return false;
    }
    results->clear();
    for (const RigExecOutputId &out : outs)
        results->push_back(
            *static_cast<const Points *>(evaluator->GetOutput(out)));
    return true;
}

bool
Contains(const std::string &text, const char *part)
{
    return text.find(part) != std::string::npos;
}

std::unique_ptr<RigExecTaskSystem> gTasks;

/// How a Node also runs its op: cut into ranges of `grain` points, on
/// worker threads or inline, with SIMD or without. The first setting of
/// each SIMD mode, whole and inline, is the one the others must match.
struct RangeSetting {
    bool simd;
    size_t grain;
    bool threads;
};

const RangeSetting kRangeSettings[] = {
    {true, 0, false},  {true, 1, true},   {true, 7, false}, {true, 64, true},
    {false, 0, false}, {false, 1, false}, {false, 5, true},
};

/// Bitwise equality of a value without padding or of an array; true for
/// other values, which no ranged op writes.
template <class T>
bool
SameResult(const T &a, const T &b)
{
    if constexpr (std::has_unique_object_representations_v<T>)
        return std::memcmp(&a, &b, sizeof(T)) == 0;
    return true;
}

template <class T>
bool
SameResult(const RigExecArray<T> &a, const RigExecArray<T> &b)
{
    return a.Size() == b.Size() &&
           (a.Empty() ||
            std::memcmp(a.Data(), b.Data(), a.Size() * sizeof(T)) == 0);
}

/// Frames compared on their points and flags, not their padding.
bool
SameResult(const RigExecArray<rigExecRig::PointFrame> &a,
           const RigExecArray<rigExecRig::PointFrame> &b)
{
    if (a.Size() != b.Size())
        return false;
    for (size_t k = 0; k < a.Size(); ++k)
        if (a[k].flags != b[k].flags ||
            std::memcmp(a[k].points.data(), b[k].points.data(),
                        sizeof(a[k].points)) != 0)
            return false;
    return true;
}

/// One op node with the listed ports bound, the first output read. It runs
/// under every range setting; each must give the bits, or the error, of the
/// whole run in its SIMD mode, and Run answers the first.
struct Node {
    std::vector<std::unique_ptr<RigExecEvaluator>> evaluators;
    std::vector<RigExecInputId> inputs;
    RigExecOutputId out;
    /// Why the node did not compile; Run answers it.
    std::string compileError;

    Node(const RigExecOpHandle &op,
         const std::vector<const RigExecPortHandle *> &ins,
         const RigExecPortHandle &output, const void *payload = nullptr)
    {
        RigExecRigDescBuilder b;
        const RigExecNodeId node = b.AddOp(op, payload);
        for (const RigExecPortHandle *port : ins)
            inputs.push_back(b.BindInput(node, *port));
        out = b.BindOutput(node, output);
        std::string error;
        auto desc = b.Build(&error);
        for (const RangeSetting &setting : kRangeSettings) {
            RigExecCompileOptions options;
            options.useSimd = setting.simd;
            options.rangeGrain = setting.grain;
            RigExecEvaluatorOptions evaluate;
            evaluate.tasks = setting.threads ? gTasks.get() : nullptr;
            auto program =
                desc ? RigExecCompile(*desc, options, &error) : nullptr;
            auto evaluator =
                program ? RigExecEvaluator::Create(program, evaluate, &error)
                        : nullptr;
            if (!evaluator) {
                compileError = error;
                evaluators.clear();
                return;
            }
            evaluators.push_back(std::move(evaluator));
        }
    }

    template <class T>
    bool Run(const std::vector<const void *> &values, T *result,
             std::string *error = nullptr)
    {
        if (evaluators.empty()) {
            if (error)
                *error = compileError;
            return false;
        }
        bool answer = false;
        bool wholeRan[2] = {false, false};
        T whole[2];
        std::string wholeError[2];
        for (size_t s = 0; s < evaluators.size(); ++s) {
            RigExecEvaluator *evaluator = evaluators[s].get();
            if (!evaluator)
                return false;
            std::string why;
            bool ran = true;
            for (size_t i = 0; ran && i < values.size(); ++i)
                ran = evaluator->SetInput(inputs[i], values[i], &why);
            ran = ran && evaluator->Evaluate(&why);
            T value{};
            if (ran)
                value = *static_cast<const T *>(evaluator->GetOutput(out));
            const int mode = kRangeSettings[s].simd ? 0 : 1;
            if (kRangeSettings[s].grain == 0 && !kRangeSettings[s].threads) {
                wholeRan[mode] = ran;
                whole[mode] = value;
                wholeError[mode] = why;
            } else if (ran != wholeRan[mode] ||
                       (ran ? !SameResult(value, whole[mode])
                            : why != wholeError[mode])) {
                std::printf("range setting %zu differs from the whole run: "
                            "%s\n",
                            s, why.c_str());
                CHECK(false);
            }
            if (s == 0) {
                answer = ran;
                *result = value;
                if (error)
                    *error = why;
            }
        }
        return answer;
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

rigExecStd::Mat4d
ToStd(const GfMatrix4d &m)
{
    rigExecStd::Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out._mtx[r][c] = m[r][c];
    return out;
}

void
TestDeltaMush()
{
    using namespace rigExecOps;
    const Grid rest(8, 5);
    // The posed grid: bent along x and lifted.
    std::vector<GfVec3f> posed = rest.points;
    for (GfVec3f &p : posed)
        p = GfVec3f(p[0], p[1] + 0.05f * p[0] * p[0], p[2] + 0.5f);
    std::vector<float> envelope(posed.size());
    for (size_t i = 0; i < envelope.size(); ++i)
        envelope[i] = 0.07f * (i % 15);
    const Points in = ToStd(posed), restArray = ToStd(rest.points);
    const auto counts = ToArray(rest.counts);
    const auto indices = ToArray(rest.indices);
    const auto envelopeArray = ToArray(envelope);

    std::vector<RigExecDeltaMushSettings> settings(6);
    settings[1].smoothing = RigExecDeltaMushSmoothing::Simple;
    settings[2].smoothing = RigExecDeltaMushSmoothing::LengthWeighted;
    settings[2].frameTransport = RigExecDeltaMushFrameTransport::Corner;
    settings[3].onlySmooth = true;
    settings[4].smoothWeights.assign(posed.size(), 0.0f);
    for (size_t i = 0; i < posed.size(); ++i)
        settings[4].smoothWeights[i] = 0.1f * (i % 11);
    // Explicit edges cover every polygon edge; add each quad's diagonal.
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x) {
            const int i = y * 8 + x;
            if (x + 1 < 8)
                settings[5].edges.insert(settings[5].edges.end(), {i, i + 1});
            if (y + 1 < 8)
                settings[5].edges.insert(settings[5].edges.end(), {i, i + 8});
            if (x + 1 < 8 && y + 1 < 8)
                settings[5].edges.insert(settings[5].edges.end(), {i, i + 9});
        }
    GfMatrix4d rotated(1.0);
    rotated.SetRotate(GfRotation(GfVec3d(0.2, 1, 0.1), 35.0));
    rotated.SetTranslateOnly(GfVec3d(0.5, -1, 2));
    GfMatrix4d projective(1.0);
    projective[0][3] = 0.5;

    for (size_t s = 0; s < settings.size(); ++s)
        for (const GfMatrix4d &map : {GfMatrix4d(1.0), rotated, projective})
            for (const bool pin : {true, false}) {
                DeltaMushPayload payload;
                payload.iterations = 4;
                payload.step = 0.4;
                payload.pinBorders = pin;
                payload.distanceWeight = s == 1 ? 0.5 : 0.0;
                payload.displacement = 0.8;
                payload.settings = settings[s];
                RigExecRigDescBuilder b;
                const RigExecNodeId node = b.AddOp(DeltaMush(), &payload);
                const RigExecInputId ins[] = {
                    b.BindInput(node, DeltaMushPoints()),
                    b.BindInput(node, DeltaMushRest()),
                    b.BindInput(node, DeltaMushCounts()),
                    b.BindInput(node, DeltaMushIndices()),
                    b.BindInput(node, DeltaMushComputationToTarget()),
                    b.BindInput(node, DeltaMushEnvelope())};
                const RigExecOutputId out = b.BindOutput(node, DeltaMushOut());
                std::string error;
                auto desc = b.Build(&error);
                auto program =
                    desc ? RigExecCompile(*desc, {}, &error) : nullptr;
                auto e = program ? RigExecEvaluator::Create(program, {}, &error)
                                 : nullptr;
                CHECK(e);
                if (!e)
                    continue;
                const auto mapArray = ToStd(map);
                const void *values[] = {&in,      &restArray, &counts,
                                        &indices, &mapArray,  &envelopeArray};
                for (size_t i = 0; i < std::size(ins); ++i)
                    CHECK(e->SetInput(ins[i], values[i], &error));
                std::vector<GfVec3f> expected = posed;
                const bool ok = RigExecApplyDeltaMush(
                    &expected, rest.points, rest.counts, rest.indices,
                    payload.iterations, payload.step, payload.pinBorders,
                    payload.distanceWeight, payload.displacement,
                    payload.settings, map);
                CHECK(map[0][3] == 0.0 || !ok);
                CHECK(map[0][3] != 0.0 || ok);
                CHECK(e->Evaluate(&error) == ok);
                if (!ok)
                    continue;
                Blend(posed, envelope, &expected);
                if (!SameBits(expected, *static_cast<const Points *>(
                                            e->GetOutput(out)))) {
                    std::printf("delta mush settings %zu pin %d differs\n", s,
                                int(pin));
                    CHECK(false);
                }
            }
    // Nodes with different settings and lists in one rig each run with
    // their own.
    std::vector<DeltaMushPayload> payloads(settings.size());
    for (size_t s = 0; s < settings.size(); ++s) {
        payloads[s].iterations = 2 + int(s);
        payloads[s].pinBorders = s % 2 == 0;
        payloads[s].settings = settings[s];
    }
    const std::vector<const RigExecPortHandle *> ports = {
        &DeltaMushPoints(), &DeltaMushRest(), &DeltaMushCounts(),
        &DeltaMushIndices(), &DeltaMushEnvelope()};
    const std::vector<const void *> values = {&in, &restArray, &counts,
                                              &indices, &envelopeArray};
    std::vector<Points> results;
    CHECK(RunEach(DeltaMush(), ports, DeltaMushOut(), payloads, values,
                  &results));
    for (size_t i = 0; i < results.size(); ++i) {
        Node single(DeltaMush(), ports, DeltaMushOut(), &payloads[i]);
        Points actual;
        CHECK(single.Run(values, &actual) && SamePoints(actual, results[i]));
    }
}

/// A cage of d.x * d.y * d.z points over [lo, hi], x fastest, jittered.
std::vector<GfVec3f>
Cage(const GfVec3i &d, const GfVec3f &lo, const GfVec3f &hi, float jitter,
     unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> j(-jitter, jitter);
    std::vector<GfVec3f> cage;
    for (int z = 0; z < d[2]; ++z)
        for (int y = 0; y < d[1]; ++y)
            for (int x = 0; x < d[0]; ++x) {
                const GfVec3f t(d[0] > 1 ? float(x) / (d[0] - 1) : 0.5f,
                                d[1] > 1 ? float(y) / (d[1] - 1) : 0.5f,
                                d[2] > 1 ? float(z) / (d[2] - 1) : 0.5f);
                cage.emplace_back(lo[0] + t[0] * (hi[0] - lo[0]) + j(rng),
                                  lo[1] + t[1] * (hi[1] - lo[1]) + j(rng),
                                  lo[2] + t[2] * (hi[2] - lo[2]) + j(rng));
            }
    return cage;
}

void
CheckLattice(Node &node, const std::vector<const void *> &values,
             const std::vector<GfVec3f> &expected, bool ok, const char *what)
{
    Points actual;
    std::string error;
    const bool ran = node.Run(values, &actual, &error);
    if (ran != ok || (ok && !SameBits(expected, actual))) {
        std::printf("lattice %s: ran %d, expected %d %s\n", what, int(ran),
                    int(ok), error.c_str());
        CHECK(false);
    }
}

void
TestLattice()
{
    using namespace rigExecOps;
    const Grid grid(9, 3);
    const Points in = ToStd(grid.points);
    std::vector<float> envelope(grid.points.size());
    for (size_t i = 0; i < envelope.size(); ++i)
        envelope[i] = 0.09f * (i % 12);
    const auto envelopeArray = ToArray(envelope);
    const Points rest = ToStd(grid.points);

    // Bound: the bind box from the rest cage, the posed cage moved.
    for (const GfVec3i &d :
         {GfVec3i(2, 2, 2), GfVec3i(3, 4, 2), GfVec3i(4, 3, 5)}) {
        const auto restCage =
            Cage(d, GfVec3f(-0.5f, -0.5f, -2), GfVec3f(8.5f, 8.5f, 2), 0, 1);
        const auto posedCage =
            Cage(d, GfVec3f(-1, -0.2f, -2.5f), GfVec3f(9, 9.5f, 1.5f), 0.4f,
                 d[0] * 7u + d[2]);
        const Points restCageArray = ToStd(restCage);
        const Points posedCageArray = ToStd(posedCage);
        LatticePayload payload;
        payload.divisions = {d[0], d[1], d[2]};
        std::vector<GfVec3f> expected = grid.points;
        RigExecApplyLattice(&expected, grid.points, restCage, posedCage, d);
        CHECK(expected != grid.points);

        Node plain(Lattice(),
                   {&LatticePoints(), &LatticeRestPoints(), &LatticeRestCage(),
                    &LatticePosedCage()},
                   LatticeOut(), &payload);
        CheckLattice(plain, {&in, &rest, &restCageArray, &posedCageArray},
                     expected, true, "bound");

        Node enveloped(Lattice(),
                       {&LatticePoints(), &LatticeRestPoints(),
                        &LatticeRestCage(), &LatticePosedCage(),
                        &LatticeEnvelope()},
                       LatticeOut(), &payload);
        Blend(grid.points, envelope, &expected);
        CheckLattice(
            enveloped,
            {&in, &rest, &restCageArray, &posedCageArray, &envelopeArray},
            expected, true, "bound enveloped");

        // A bound lattice without its rest inputs is an error.
        Node unbound(Lattice(), {&LatticePoints(), &LatticePosedCage()},
                     LatticeOut(), &payload);
        CheckLattice(unbound, {&in, &posedCageArray}, {}, false, "unbound");
    }

    // Regular grid: posed node offsets from the grid positions.
    GfMatrix4d toLattice(1.0);
    toLattice.SetRotate(GfRotation(GfVec3d(0.3, 0.2, 1), 20.0));
    toLattice.SetTranslateOnly(GfVec3d(-4, -4, 0.5));
    const GfMatrix4d fromLattice = RigExecSurfaceSnapAffineInverse(toLattice);
    GfMatrix4d cageToLattice(1.0);
    cageToLattice.SetScale(GfVec3d(1.0, 1.1, 0.9));
    const auto toLatticeStd = ToStd(toLattice);
    const auto fromLatticeStd = ToStd(fromLattice);
    const auto cageToLatticeStd = ToStd(cageToLattice);
    using Interp = RigExecLatticeInterpolation;
    for (const GfVec3i &d : {GfVec3i(4, 4, 3), GfVec3i(5, 1, 6)})
        for (const Interp mode : {Interp::BSpline, Interp::Linear,
                                  Interp::Cardinal, Interp::CatmullRom})
            for (const bool masked : {false, true}) {
                LatticePayload payload;
                payload.divisions = {d[0], d[1], d[2]};
                RigExecLatticeSettings &s = payload.settings;
                s.regularGrid = true;
                s.interpolation = {mode, Interp::BSpline, mode};
                s.origin = {-5, -5, -2};
                s.spacing = {10.0f / (d[0] > 1 ? d[0] - 1 : 1),
                             10.0f / (d[1] > 1 ? d[1] - 1 : 1),
                             4.0f / (d[2] > 1 ? d[2] - 1 : 1)};
                s.strength = masked ? 0.7f : 1.0f;
                if (masked)
                    s.mask = envelope;
                const auto posedCage = Cage(d, GfVec3f(-5, -5, -2),
                                            GfVec3f(5, 5, 2), 0.6f, 3u + d[0]);
                const Points posedCageArray = ToStd(posedCage);

                std::vector<GfVec3f> expected = grid.points;
                CHECK((RigExecApplyLatticeGridKernel<GfVec3f, GfVec3d>(
                    &expected, posedCage, d, s, toLattice, fromLattice,
                    cageToLattice)));
                CHECK(expected != grid.points);
                Node spaced(Lattice(),
                            {&LatticePoints(), &LatticePosedCage(),
                             &LatticeTargetToLattice(),
                             &LatticeLatticeToTarget(), &LatticeCageToLattice(),
                             &LatticeEnvelope()},
                            LatticeOut(), &payload);
                std::vector<GfVec3f> blended = expected;
                Blend(grid.points, envelope, &blended);
                CheckLattice(spaced,
                             {&in, &posedCageArray, &toLatticeStd,
                              &fromLatticeStd, &cageToLatticeStd,
                              &envelopeArray},
                             blended, true, "grid");

                // Unfed spaces are identity.
                expected = grid.points;
                CHECK((RigExecApplyLatticeGridKernel<GfVec3f, GfVec3d>(
                    &expected, posedCage, d, s, GfMatrix4d(1.0),
                    GfMatrix4d(1.0), GfMatrix4d(1.0))));
                Node plain(Lattice(), {&LatticePoints(), &LatticePosedCage()},
                           LatticeOut(), &payload);
                CheckLattice(plain, {&in, &posedCageArray}, expected, true,
                             "grid identity");

                // A cage of the wrong count is refused by both.
                const Points shortCage(posedCageArray.Data(),
                                       posedCageArray.Size() - 1);
                std::vector<GfVec3f> refused = grid.points;
                CHECK((!RigExecApplyLatticeGridKernel<GfVec3f, GfVec3d>(
                    &refused,
                    std::vector<GfVec3f>(posedCage.begin(),
                                         posedCage.end() - 1),
                    d, s, GfMatrix4d(1.0), GfMatrix4d(1.0), GfMatrix4d(1.0))));
                CheckLattice(plain, {&in, &shortCage}, {}, false, "short cage");
            }
}

void
TestWrinkle()
{
    using namespace rigExecOps;
    const Grid rest(8, 9, true);
    // Compressed along x, so the constraints buckle the sheet.
    std::vector<GfVec3f> posed = rest.points;
    for (GfVec3f &p : posed)
        p = GfVec3f(p[0] * 0.7f, p[1], p[2] + 0.01f * p[1]);
    std::vector<float> envelope(posed.size());
    for (size_t i = 0; i < envelope.size(); ++i)
        envelope[i] = 0.08f * (i % 13);
    const Points in = ToStd(posed), restArray = ToStd(rest.points);
    const auto counts = ToArray(rest.counts);
    const auto indices = ToArray(rest.indices);
    const auto envelopeArray = ToArray(envelope);

    std::vector<RigExecWrinkleSettings> settings(5);
    settings[0].iterations = 12;
    settings[1].iterations = 6;
    settings[1].topology = RigExecWrinkleTopology::SurfaceStruts;
    settings[1].neighborDistance = 3;
    settings[2].iterations = 8;
    settings[2].pinBorders = false;
    settings[2].tangentPlaneCollisions = false;
    settings[2].smoothingIterations = 2;
    settings[2].pinPoints = {0, 5, 17};
    settings[3].iterations = 10;
    settings[3].restLengthScale = 1.2f;
    settings[3].wrinkleScale = 0.5f;
    settings[3].tangentPlaneInset = 0.05f;
    settings[4].neighborDistance = 0; // Refused by both.

    for (size_t i = 0; i < settings.size(); ++i) {
        WrinklePayload payload;
        payload.settings = settings[i];
        std::vector<GfVec3f> expected = posed;
        const bool ok = RigExecApplyWrinkle(&expected, rest.points, rest.counts,
                                            rest.indices, settings[i]);
        CHECK(ok == (i != 4));
        CHECK(!ok || expected != posed);
        Node plain(Wrinkle(),
                   {&WrinklePoints(), &WrinkleRest(), &WrinkleCounts(),
                    &WrinkleIndices()},
                   WrinkleOut(), &payload);
        Points actual;
        std::string error;
        CHECK(plain.Run({&in, &restArray, &counts, &indices}, &actual,
                        &error) == ok);
        CHECK(!ok || SameBits(expected, actual));
        if (!ok)
            continue;
        Node enveloped(Wrinkle(),
                       {&WrinklePoints(), &WrinkleRest(), &WrinkleCounts(),
                        &WrinkleIndices(), &WrinkleEnvelope()},
                       WrinkleOut(), &payload);
        Blend(posed, envelope, &expected);
        CHECK(
            enveloped.Run({&in, &restArray, &counts, &indices, &envelopeArray},
                          &actual, &error));
        if (!SameBits(expected, actual)) {
            std::printf("wrinkle settings %zu differs\n", i);
            CHECK(false);
        }
    }
}

void
TestSurfaceProject()
{
    using namespace rigExecOps;
    // A small surface (brute force) and one past the native BVH's bar.
    for (const int n : {5, 11}) {
        const Grid surface(n, 21 + n);
        Grid target(n + 2, 4 + n);
        for (GfVec3f &p : target.points)
            p = GfVec3f(p[0] * 0.9f - 0.5f, p[1] * 0.9f - 0.5f,
                        p[2] * 0.5f + 0.7f);
        const Points in = ToStd(target.points);
        const Points surfaceArray = ToStd(surface.points);
        const auto counts = ToArray(surface.counts);
        const auto indices = ToArray(surface.indices);
        std::vector<float> envelope(target.points.size());
        for (size_t i = 0; i < envelope.size(); ++i)
            envelope[i] = 0.06f * (i % 17);
        const auto envelopeArray = ToArray(envelope);

        // Legacy: default settings and unfed spaces.
        SurfaceProjectPayload legacy;
        Node plain(SurfaceProject(),
                   {&SurfaceProjectPoints(), &SurfaceProjectSurface(),
                    &SurfaceProjectCounts(), &SurfaceProjectIndices(),
                    &SurfaceProjectStrength()},
                   SurfaceProjectOut(), &legacy);
        Node enveloped(SurfaceProject(),
                       {&SurfaceProjectPoints(), &SurfaceProjectSurface(),
                        &SurfaceProjectCounts(), &SurfaceProjectIndices(),
                        &SurfaceProjectStrength(), &SurfaceProjectEnvelope()},
                       SurfaceProjectOut(), &legacy);
        for (const float strength : {1.0f, 0.4f, 0.0f}) {
            std::vector<GfVec3f> expected = target.points;
            RigExecApplySurfaceProject(&expected, surface.points,
                                       surface.counts, surface.indices,
                                       strength);
            CHECK(strength == 0.0f || expected != target.points);
            Points actual;
            CHECK(plain.Run({&in, &surfaceArray, &counts, &indices, &strength},
                            &actual));
            const bool same = SameBits(expected, actual);
            Blend(target.points, envelope, &expected);
            CHECK(enveloped.Run({&in, &surfaceArray, &counts, &indices,
                                 &strength, &envelopeArray},
                                &actual));
            if (!same || !SameBits(expected, actual)) {
                std::printf("surface project %d strength %g differs\n", n,
                            strength);
                CHECK(false);
            }
        }

        // Snap: modes, offsets, a mask, explicit triangles and spaces.
        GfMatrix4d toSurface(1.0);
        toSurface.SetRotate(GfRotation(GfVec3d(1, 0.4, 0.2), 15.0));
        toSurface.SetTranslateOnly(GfVec3d(0.2, -0.1, 0.3));
        const GfMatrix4d toTarget = RigExecSurfaceSnapAffineInverse(toSurface);
        GfMatrix4d toMetric(1.0);
        toMetric.SetScale(GfVec3d(1.0, 1.2, 0.8));
        const auto toSurfaceStd = ToStd(toSurface),
                   toTargetStd = ToStd(toTarget), toMetricStd = ToStd(toMetric);
        using Mode = RigExecSurfaceSnapMode;
        for (const Mode mode : {Mode::OnSurface, Mode::Inside, Mode::Outside,
                                Mode::OutsideSurface})
            for (int variant = 0; variant < 3; ++variant) {
                SurfaceProjectPayload payload;
                RigExecSurfaceSnapSettings &s = payload.settings;
                s.mode = mode;
                s.offset = variant == 0 ? 0.0f : 0.15f;
                if (variant == 1)
                    s.mask = envelope;
                if (variant == 2)
                    for (size_t q = 0; q < surface.counts.size(); ++q) {
                        const int *f = &surface.indices[q * 4];
                        s.triangles.insert(
                            s.triangles.end(),
                            {f[0], f[1], f[3], f[1], f[2], f[3]});
                    }
                std::vector<GfVec3f> expected = target.points;
                const bool ok = RigExecApplySurfaceSnapKernel<GfVec3f, GfVec3d>(
                    &expected, surface.points, surface.counts, surface.indices,
                    s, toSurface, toTarget, toMetric);
                CHECK(ok);
                Blend(target.points, envelope, &expected);
                Node snap(SurfaceProject(),
                          {&SurfaceProjectPoints(), &SurfaceProjectSurface(),
                           &SurfaceProjectCounts(), &SurfaceProjectIndices(),
                           &SurfaceProjectTargetToSurface(),
                           &SurfaceProjectSurfaceToTarget(),
                           &SurfaceProjectSurfaceToMetric(),
                           &SurfaceProjectEnvelope()},
                          SurfaceProjectOut(), &payload);
                Points actual;
                CHECK(snap.Run({&in, &surfaceArray, &counts, &indices,
                                &toSurfaceStd, &toTargetStd, &toMetricStd,
                                &envelopeArray},
                               &actual));
                if (!SameBits(expected, actual)) {
                    std::printf("surface snap %d mode %d variant %d differs\n",
                                n, int(mode), variant);
                    CHECK(false);
                }
            }

        // A mask of the wrong length is refused by both.
        SurfaceProjectPayload bad;
        bad.settings.mask.assign(3, 1.0f);
        std::vector<GfVec3f> refused = target.points;
        CHECK((!RigExecApplySurfaceSnapKernel<GfVec3f, GfVec3d>(
            &refused, surface.points, surface.counts, surface.indices,
            bad.settings, GfMatrix4d(1.0), GfMatrix4d(1.0), GfMatrix4d(1.0))));
        Node refusing(SurfaceProject(),
                      {&SurfaceProjectPoints(), &SurfaceProjectSurface(),
                       &SurfaceProjectCounts(), &SurfaceProjectIndices()},
                      SurfaceProjectOut(), &bad);
        Points actual;
        CHECK(!refusing.Run({&in, &surfaceArray, &counts, &indices}, &actual));
    }
}

void
TestMatrixDeform()
{
    using namespace rigExecOps;
    const Grid grid(9, 31);
    std::vector<GfVec3f> points = grid.points;
    points[3] = GfVec3f(-0.0f, 0.0f, -0.0f); // signed zeros at weight 0
    const Points in = ToStd(points);
    std::vector<float> envelope(points.size());
    for (size_t i = 0; i < envelope.size(); ++i)
        envelope[i] = i % 5 == 0 ? 0.0f : i % 7 == 0 ? 1.0f : 0.11f * (i % 9);
    envelope[3] = 0.0f;
    const auto envelopeArray = ToArray(envelope);

    GfMatrix4d turn(1.0);
    turn.SetRotate(GfRotation(GfVec3d(0.1, 0.3, 1), 70.0));
    turn.SetTranslateOnly(GfVec3d(3, -2, 5));
    GfMatrix4d scaled = GfMatrix4d(1.0).SetScale(GfVec3d(1.2, 0.8, 1.0)) * turn;
    GfMatrix4d slide(1.0);
    slide.SetTranslate(GfVec3d(0.5, 0.25, -1));
    for (const GfMatrix4d &t : {turn, scaled, slide, GfMatrix4d(1.0)})
        for (const bool radial : {false, true}) {
            MatrixDeformPayload payload;
            payload.radial = radial;
            const auto transform = ToStd(t);
            Node full(MatrixDeform(),
                      {&MatrixDeformPoints(), &MatrixDeformTransform()},
                      MatrixDeformOut(), &payload);
            Node weighted(MatrixDeform(),
                          {&MatrixDeformPoints(), &MatrixDeformTransform(),
                           &MatrixDeformEnvelope()},
                          MatrixDeformOut(), &payload);

            // Full strength.
            std::vector<GfVec3f> expected = points;
            if (radial)
                for (GfVec3f &p : expected)
                    p = GfVec3f(t.TransformAffine(GfVec3d(p)));
            else
                RigExecApplyWeightedMatrixSimd(points.data(), expected.data(),
                                               1.0f, points.size(), t);
            Points actual;
            CHECK(full.Run({&in, &transform}, &actual));
            if (!SameBits(expected, actual)) {
                std::printf("matrix full radial %d differs\n", int(radial));
                CHECK(false);
            }

            // Weighted.
            if (radial)
                for (size_t i = 0; i < points.size(); ++i)
                    expected[i] =
                        GfVec3f(RigExecPartialTransform(t, envelope[i])
                                    .TransformAffine(GfVec3d(points[i])));
            else
                RigExecApplyWeightedMatrixSimd(points.data(), expected.data(),
                                               envelope.data(), points.size(),
                                               t);
            CHECK(weighted.Run({&in, &transform, &envelopeArray}, &actual));
            if (!SameBits(expected, actual)) {
                std::printf("matrix weighted radial %d differs\n", int(radial));
                CHECK(false);
            }
        }

    // A weight out of range and a short envelope are refused.
    MatrixDeformPayload payload;
    Node weighted(MatrixDeform(),
                  {&MatrixDeformPoints(), &MatrixDeformTransform(),
                   &MatrixDeformEnvelope()},
                  MatrixDeformOut(), &payload);
    const auto transform = ToStd(turn);
    std::vector<float> bad = envelope;
    bad[4] = 1.5f;
    const auto badArray = ToArray(bad);
    const RigExecArray<float> shortArray(envelope.data(), envelope.size() - 1);
    Points actual;
    CHECK(!weighted.Run({&in, &transform, &badArray}, &actual));
    CHECK(!weighted.Run({&in, &transform, &shortArray}, &actual));

    // Nodes with different settings in one rig each run with their own.
    MatrixDeformPayload radial;
    radial.radial = true;
    const std::vector<MatrixDeformPayload> payloads = {payload, radial,
                                                       payload};
    const std::vector<const RigExecPortHandle *> ports = {
        &MatrixDeformPoints(), &MatrixDeformTransform(),
        &MatrixDeformEnvelope()};
    const std::vector<const void *> values = {&in, &transform, &envelopeArray};
    std::vector<Points> results;
    CHECK(RunEach(MatrixDeform(), ports, MatrixDeformOut(), payloads, values,
                  &results));
    for (size_t i = 0; i < results.size(); ++i) {
        Node single(MatrixDeform(), ports, MatrixDeformOut(), &payloads[i]);
        CHECK(single.Run(values, &actual) && SamePoints(actual, results[i]));
    }
}

void
TestSkin()
{
    using namespace rigExecOps;
    const Grid grid(9, 31);
    const std::vector<GfVec3f> &points = grid.points;
    const Points in = ToStd(points);

    GfMatrix4d turn(1.0);
    turn.SetRotate(GfRotation(GfVec3d(0.1, 0.3, 1), 70.0));
    turn.SetTranslateOnly(GfVec3d(3, -2, 5));
    GfMatrix4d bend(1.0);
    bend.SetRotate(GfRotation(GfVec3d(1, 0, 0.2), -140.0));
    const GfMatrix4d squash =
        GfMatrix4d(1.0).SetScale(GfVec3d(1.3, 0.7, 1.0)) * bend;
    GfMatrix4d shear(1.0);
    shear[1][0] = 0.4;
    shear.SetTranslateOnly(GfVec3d(0.5, 0.25, -1));
    const std::vector<GfMatrix4d> gf = {turn, squash, shear, GfMatrix4d(1.0)};
    std::vector<rigExecStd::Mat4d> mats;
    for (const GfMatrix4d &m : gf)
        mats.push_back(ToStd(m));
    const auto transforms = ToArray(mats);

    const int elementSize = 3;
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    std::vector<int> indices(points.size() * elementSize);
    std::vector<float> weights(indices.size());
    for (size_t i = 0; i < points.size(); ++i) {
        float total = 0.0f;
        for (int k = 0; k < elementSize; ++k) {
            const size_t s = i * elementSize + k;
            indices[s] = int((i + 3 * k) % gf.size());
            weights[s] = i % 11 == size_t(k) ? 0.0f : unit(rng);
            total += weights[s];
        }
        // Normalized, under one and over one.
        const float scale = i % 3 == 0   ? 1.0f / total
                            : i % 3 == 1 ? 0.6f
                                         : 1.0f;
        for (int k = 0; k < elementSize; ++k)
            weights[i * elementSize + k] *= scale;
    }
    const auto indexArray = ToArray(indices);
    const auto weightArray = ToArray(weights);

    RigExecSkinLayout layout;
    layout.transforms = gf.data();
    layout.transformCount = gf.size();
    layout.indices = indices.data();
    layout.weights = weights.data();
    layout.indexCount = indices.size();
    layout.elementSize = elementSize;
    layout.pointCount = points.size();
    CHECK(layout.Validate());

    for (const SkinMethod method :
         {SkinMethod::ClassicLinear, SkinMethod::DualQuaternion}) {
        SkinPayload payload;
        payload.method = method;
        payload.elementSize = elementSize;
        Node node(
            Skin(),
            {&SkinPoints(), &SkinTransforms(), &SkinIndices(), &SkinWeights()},
            SkinOut(), &payload);
        std::vector<GfVec3f> expected(points.size());
        if (method == SkinMethod::ClassicLinear)
            RigExecApplyLinearBlendSkinSimd(points.data(), expected.data(),
                                            layout);
        else
            CHECK(RigExecApplyDualQuatSkin(points.data(), expected.data(),
                                           layout));
        Points actual;
        CHECK(node.Run({&in, &transforms, &indexArray, &weightArray}, &actual));
        if (!SameBits(expected, actual)) {
            std::printf("skin method %d differs\n", int(method));
            CHECK(false);
        }
    }

    // Unusable layouts are refused.
    SkinPayload payload;
    payload.elementSize = elementSize;
    Node node(
        Skin(),
        {&SkinPoints(), &SkinTransforms(), &SkinIndices(), &SkinWeights()},
        SkinOut(), &payload);
    Points actual;
    std::vector<int> outside = indices;
    outside[5] = int(gf.size());
    const auto outsideArray = ToArray(outside);
    CHECK(!node.Run({&in, &transforms, &outsideArray, &weightArray}, &actual));
    std::vector<float> negative = weights;
    negative[5] = -0.1f;
    const auto negativeArray = ToArray(negative);
    CHECK(!node.Run({&in, &transforms, &indexArray, &negativeArray}, &actual));
    const RigExecArray<float> shortArray(weights.data(), weights.size() - 1);
    CHECK(!node.Run({&in, &transforms, &indexArray, &shortArray}, &actual));
    std::vector<rigExecStd::Mat4d> projective = mats;
    projective[1][0][3] = 0.5;
    const auto projectiveArray = ToArray(projective);
    CHECK(
        !node.Run({&in, &projectiveArray, &indexArray, &weightArray}, &actual));
    payload.elementSize = 0;
    Node empty(
        Skin(),
        {&SkinPoints(), &SkinTransforms(), &SkinIndices(), &SkinWeights()},
        SkinOut(), &payload);
    CHECK(!empty.Run({&in, &transforms, &indexArray, &weightArray}, &actual));

    // Nodes with different settings in one rig each run with their own.
    SkinPayload linear;
    linear.elementSize = elementSize;
    SkinPayload dual = linear;
    dual.method = SkinMethod::DualQuaternion;
    const std::vector<SkinPayload> payloads = {dual, linear, dual};
    const std::vector<const RigExecPortHandle *> ports = {
        &SkinPoints(), &SkinTransforms(), &SkinIndices(), &SkinWeights()};
    const std::vector<const void *> values = {&in, &transforms, &indexArray,
                                              &weightArray};
    std::vector<Points> results;
    CHECK(RunEach(Skin(), ports, SkinOut(), payloads, values, &results));
    for (size_t i = 0; i < results.size(); ++i) {
        Node single(Skin(), ports, SkinOut(), &payloads[i]);
        CHECK(single.Run(values, &actual) && SamePoints(actual, results[i]));
    }
    CHECK(results.size() == 3 && !SamePoints(results[0], results[1]));

    // SkinMatrices folds the forms once; a skin fed them moves the points
    // bit for bit as one that folds its own.
    for (const bool simd : {false, true}) {
        for (const SkinMethod method :
             {SkinMethod::ClassicLinear, SkinMethod::DualQuaternion}) {
            const bool dualQuat = method == SkinMethod::DualQuaternion;
            SkinMatricesPayload folded;
            folded.method = method;
            SkinPayload skin;
            skin.method = method;
            skin.elementSize = elementSize;
            const auto run = [&](bool fed, const void *influenceValue,
                                 Points *out, RigExecArray<float> *rows,
                                 size_t *paletteCount) {
                RigExecRigDescBuilder b;
                const RigExecNodeId fold = b.AddOp(SkinMatrices(), &folded);
                const RigExecNodeId node = b.AddOp(Skin(), &skin);
                const RigExecInputId influences =
                    b.BindInput(fold, SkinMatricesInfluences());
                const RigExecOutputId rowsOut =
                    b.BindOutput(fold, SkinMatricesRows());
                const RigExecOutputId realOut =
                    b.BindOutput(fold, SkinMatricesReal());
                RigExecInputId transformsIn;
                if (fed) {
                    b.Connect(fold, SkinMatricesMatrices(), node,
                              SkinTransforms());
                    if (dualQuat) {
                        b.Connect(fold, SkinMatricesReal(), node,
                                  SkinPaletteReal());
                        b.Connect(fold, SkinMatricesDual(), node,
                                  SkinPaletteDual());
                        b.Connect(fold, SkinMatricesStretch(), node,
                                  SkinPaletteStretch());
                        b.Connect(fold, SkinMatricesRigid(), node,
                                  SkinPaletteRigid());
                    } else {
                        b.Connect(fold, SkinMatricesRows(), node, SkinRows());
                    }
                } else {
                    transformsIn = b.BindInput(node, SkinTransforms());
                }
                const RigExecInputId pointsIn = b.BindInput(node, SkinPoints()),
                                     indicesIn =
                                         b.BindInput(node, SkinIndices()),
                                     weightsIn =
                                         b.BindInput(node, SkinWeights());
                const RigExecOutputId outId = b.BindOutput(node, SkinOut());
                std::string error;
                const auto desc = b.Build(&error);
                RigExecCompileOptions options;
                options.useSimd = simd;
                const auto program =
                    desc ? RigExecCompile(*desc, options, &error) : nullptr;
                auto evaluator =
                    program ? RigExecEvaluator::Create(
                                  program, RigExecEvaluatorOptions(), &error)
                            : nullptr;
                const bool ran =
                    evaluator &&
                    evaluator->SetInput(influences, influenceValue, &error) &&
                    (fed || evaluator->SetInput(transformsIn, influenceValue,
                                                &error)) &&
                    evaluator->SetInput(pointsIn, &in, &error) &&
                    evaluator->SetInput(indicesIn, &indexArray, &error) &&
                    evaluator->SetInput(weightsIn, &weightArray, &error) &&
                    evaluator->Evaluate(&error);
                if (!ran)
                    return false;
                *out =
                    *static_cast<const Points *>(evaluator->GetOutput(outId));
                *rows = *static_cast<const RigExecArray<float> *>(
                    evaluator->GetOutput(rowsOut));
                *paletteCount =
                    static_cast<const RigExecArray<rigExecStd::Quatd> *>(
                        evaluator->GetOutput(realOut))
                        ->Size();
                return true;
            };
            Points own, shared;
            RigExecArray<float> rows;
            size_t paletteCount = 0;
            CHECK(run(false, &transforms, &own, &rows, &paletteCount));
            CHECK(run(true, &transforms, &shared, &rows, &paletteCount));
            CHECK(SamePoints(own, shared));
            CHECK(paletteCount == (dualQuat ? mats.size() : 0));
            const size_t rowCount =
                !dualQuat && simd ? mats.size() * kRigExecSkinRowStride : 0;
            CHECK(rows.Size() == rowCount);
            std::vector<float> narrowed(kRigExecSkinRowStride);
            for (size_t t = 0; t < rows.Size() / kRigExecSkinRowStride; ++t) {
                RigExecNarrowSkinRowsKernel(mats[t], narrowed.data());
                for (size_t c = 0; c < kRigExecSkinRowStride; ++c)
                    CHECK(rows[t * kRigExecSkinRowStride + c] == narrowed[c]);
            }
            CHECK(!run(true, &projectiveArray, &shared, &rows, &paletteCount));
        }
    }
}

// The baked channel sum, spelled out in Gf: dense pairs lerp their deltas,
// anything sparse accumulates each endpoint over its own indices.
struct RefSample {
    float activation;
    std::vector<GfVec3f> points;
    std::vector<GfVec3f> offsets;
    std::vector<int> indices;
    bool sparse;
};

std::vector<GfVec3f>
RefBlendDeltas(const std::vector<std::vector<RefSample>> &channels,
               const std::vector<float> &weights,
               const std::vector<GfVec3f> &base)
{
    std::vector<GfVec3f> deltas(base.size(), GfVec3f(0));
    const auto add = [&](const RefSample &s, float scale) {
        if (!s.sparse) {
            for (size_t i = 0; i < base.size(); ++i)
                deltas[i] += (s.points[i] - base[i]) * scale;
        } else if (s.indices.empty()) {
            for (size_t i = 0; i < s.offsets.size(); ++i)
                deltas[i] += s.offsets[i] * scale;
        } else {
            for (size_t k = 0; k < s.indices.size(); ++k)
                deltas[size_t(s.indices[k])] += s.offsets[k] * scale;
        }
    };
    for (size_t c = 0; c < channels.size(); ++c) {
        const auto &samples = channels[c];
        const float raw = std::isfinite(weights[c]) ? weights[c] : 0.0f;
        const float w =
            std::min(std::max(raw, 0.0f), samples.back().activation);
        if (w == 0.0f)
            continue;
        size_t hi = 0;
        while (hi + 1 < samples.size() && samples[hi].activation < w)
            ++hi;
        const float aHi = samples[hi].activation;
        const float aLo = hi ? samples[hi - 1].activation : 0.0f;
        const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
        const RefSample *lo = hi ? &samples[hi - 1] : nullptr;
        if (!samples[hi].sparse && (!lo || !lo->sparse)) {
            for (size_t i = 0; i < base.size(); ++i) {
                const GfVec3f dHi = samples[hi].points[i] - base[i];
                const GfVec3f dLo = lo ? lo->points[i] - base[i] : GfVec3f(0);
                deltas[i] += dLo + (dHi - dLo) * t;
            }
            continue;
        }
        if (lo)
            add(*lo, 1.0f - t);
        add(samples[hi], t);
    }
    return deltas;
}

void
TestBlendShape()
{
    using namespace rigExecOps;
    const Grid grid(9, 31);
    const std::vector<GfVec3f> &base = grid.points;
    std::vector<GfVec3f> posed = base;
    for (GfVec3f &p : posed)
        p = GfVec3f(p[0], p[1] + 0.05f * p[0] * p[0], p[2] + 0.5f);
    const size_t n = base.size();
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> unit(-0.3f, 0.3f);
    const auto moved = [&] {
        std::vector<GfVec3f> pts = base;
        for (GfVec3f &p : pts)
            p += GfVec3f(unit(rng), unit(rng), unit(rng));
        return pts;
    };
    const auto offsets = [&](size_t count) {
        std::vector<GfVec3f> o(count);
        for (GfVec3f &v : o)
            v = GfVec3f(unit(rng), unit(rng), unit(rng));
        return o;
    };
    std::vector<int> some;
    for (size_t i = 0; i < n; i += 7)
        some.push_back(int(i));
    some.push_back(3); // a repeated index accumulates twice

    // Dense pair, sparse pair, mixed pair, full-length layout, single.
    std::vector<std::vector<RefSample>> ref = {
        {{0.5f, moved(), {}, {}, false}, {1.0f, moved(), {}, {}, false}},
        {{0.4f, {}, offsets(some.size()), some, true},
         {1.0f, {}, offsets(some.size()), some, true}},
        {{0.3f, moved(), {}, {}, false},
         {0.9f, {}, offsets(some.size()), some, true}},
        {{1.0f, {}, offsets(n), {}, true}},
        {{1.0f, moved(), {}, {}, false}},
    };
    BlendShapePayload payload;
    for (const auto &samples : ref) {
        BlendShapeChannel channel;
        for (const RefSample &r : samples) {
            BlendShapeSample s;
            s.activation = r.activation;
            if (r.sparse) {
                auto layout = std::make_shared<BlendShapeLayout>();
                for (const GfVec3f &o : r.offsets)
                    layout->offsets.push_back(Vec3f(o[0], o[1], o[2]));
                layout->indices = r.indices;
                layout->pointCount = n;
                layout->valid = true;
                s.layout = layout;
            } else {
                for (const GfVec3f &q : r.points)
                    s.points.push_back(Vec3f(q[0], q[1], q[2]));
            }
            channel.samples.push_back(std::move(s));
        }
        payload.channels.push_back(std::move(channel));
    }
    std::vector<float> envelope(n);
    for (size_t i = 0; i < n; ++i)
        envelope[i] = 0.09f * (i % 13);
    const Points in = ToStd(posed), baseArray = ToStd(base);
    const auto envelopeArray = ToArray(envelope);
    const auto counts = ToArray(grid.counts);
    const auto indices = ToArray(grid.indices);

    const std::vector<std::vector<float>> weightSets = {
        {0.7f, 0.2f, 0.6f, 1.0f, 0.5f},
        {0.0f, 1.3f, 0.3f, -1.0f, NAN},
        {0.5f, 0.4f, 0.95f, 0.25f, 1.0f},
    };
    for (const bool surfaceFrame : {false, true})
        for (const auto &w : weightSets) {
            payload.surfaceFrame = surfaceFrame;
            const auto weightArray = ToArray(w);
            std::vector<GfVec3f> deltas = RefBlendDeltas(ref, w, base);
            if (surfaceFrame) {
                std::vector<GfVec3f> transported;
                CHECK(RigExecTransportSurfaceOffsets(base, posed, grid.counts,
                                                     grid.indices, deltas,
                                                     &transported));
                deltas = transported;
            }
            std::vector<GfVec3f> expected = posed;
            for (size_t i = 0; i < n; ++i)
                expected[i] = posed[i] + deltas[i];
            Node full(BlendShape(),
                      {&BlendShapePoints(), &BlendShapeWeights(),
                       &BlendShapeBase(), &BlendShapeRest(),
                       &BlendShapeCounts(), &BlendShapeIndices()},
                      BlendShapeOut(), &payload);
            Points actual;
            CHECK(full.Run(
                {&in, &weightArray, &baseArray, &baseArray, &counts, &indices},
                &actual));
            if (!SameBits(expected, actual)) {
                std::printf("blend shape frame %d differs\n",
                            int(surfaceFrame));
                CHECK(false);
            }
            Blend(posed, envelope, &expected);
            Node weighted(BlendShape(),
                          {&BlendShapePoints(), &BlendShapeWeights(),
                           &BlendShapeBase(), &BlendShapeRest(),
                           &BlendShapeCounts(), &BlendShapeIndices(),
                           &BlendShapeEnvelope()},
                          BlendShapeOut(), &payload);
            CHECK(weighted.Run({&in, &weightArray, &baseArray, &baseArray,
                                &counts, &indices, &envelopeArray},
                               &actual));
            if (!SameBits(expected, actual)) {
                std::printf("blend shape envelope frame %d differs\n",
                            int(surfaceFrame));
                CHECK(false);
            }
        }

    // Base defaults to the incoming points.
    payload.surfaceFrame = false;
    {
        const auto weightArray = ToArray(weightSets[0]);
        const std::vector<GfVec3f> deltas =
            RefBlendDeltas(ref, weightSets[0], posed);
        std::vector<GfVec3f> expected = posed;
        for (size_t i = 0; i < n; ++i)
            expected[i] = posed[i] + deltas[i];
        Node node(BlendShape(), {&BlendShapePoints(), &BlendShapeWeights()},
                  BlendShapeOut(), &payload);
        Points actual;
        CHECK(node.Run({&in, &weightArray}, &actual));
        CHECK(SameBits(expected, actual));
    }

    // Refusals: weight count, a short dense target, repeated activations,
    // and a surface frame with no rest mesh.
    Node node(BlendShape(), {&BlendShapePoints(), &BlendShapeWeights()},
              BlendShapeOut(), &payload);
    Points actual;
    const RigExecArray<float> shortWeights(weightSets[0].data(), 4);
    CHECK(!node.Run({&in, &shortWeights}, &actual));
    const auto weightArray = ToArray(weightSets[0]);
    BlendShapePayload bad = payload;
    bad.channels[4].samples[0].points.pop_back();
    Node shortTarget(BlendShape(), {&BlendShapePoints(), &BlendShapeWeights()},
                     BlendShapeOut(), &bad);
    CHECK(!shortTarget.Run({&in, &weightArray}, &actual));
    bad = payload;
    bad.channels[0].samples[0].activation = 1.0f;
    Node repeated(BlendShape(), {&BlendShapePoints(), &BlendShapeWeights()},
                  BlendShapeOut(), &bad);
    CHECK(!repeated.Run({&in, &weightArray}, &actual));
    bad = payload;
    bad.surfaceFrame = true;
    Node noRest(BlendShape(), {&BlendShapePoints(), &BlendShapeWeights()},
                BlendShapeOut(), &bad);
    CHECK(!noRest.Run({&in, &weightArray}, &actual));
}

} // namespace

void
TestRecomputeNormals()
{
    using namespace rigExecOps;
    Grid grid(8, 21);
    // A triangle, a sliver and a face naming a missing point join the quads.
    const int n = int(grid.points.size());
    grid.counts.insert(grid.counts.end(), {3, 3, 4});
    grid.indices.insert(grid.indices.end(), {0, 9, 1, 2, 2, 3, 4, 5, n + 3, 6});
    const Points in = ToStd(grid.points);
    const auto counts = ToArray(grid.counts);
    const auto indices = ToArray(grid.indices);
    for (const bool perCorner : {false, true})
        for (const bool leftHanded : {false, true}) {
            NormalsPayload payload;
            payload.perCorner = perCorner;
            payload.leftHanded = leftHanded;
            Node bare(RecomputeNormals(),
                      {&RecomputeNormalsPoints(), &RecomputeNormalsCounts(),
                       &RecomputeNormalsIndices()},
                      RecomputeNormalsOut(), &payload);
            Node node(RecomputeNormals(),
                      {&RecomputeNormalsPoints(), &RecomputeNormalsCounts(),
                       &RecomputeNormalsIndices(), &RecomputeNormalsAuthored(),
                       &RecomputeNormalsEnvelope()},
                      RecomputeNormalsOut(), &payload);
            std::vector<GfVec3f> expected = RigExecComputeVertexNormals(
                grid.points, grid.counts, grid.indices);
            // A corner naming a missing point has no per-corner layout.
            const bool laidOut = RigExecLayoutNormals(
                &expected, grid.indices.data(), grid.indices.size(), perCorner,
                leftHanded);
            CHECK(laidOut != perCorner);
            Points actual;
            std::string error;
            if (!laidOut) {
                CHECK(!bare.Run({&in, &counts, &indices}, &actual, &error));
                CHECK(error.find("corner") != std::string::npos);
                continue;
            }
            CHECK(bare.Run({&in, &counts, &indices}, &actual));
            CHECK(SameBits(expected, actual));
            std::vector<GfVec3f> authored(expected.size());
            std::vector<float> envelope(expected.size());
            for (size_t i = 0; i < authored.size(); ++i) {
                authored[i] = GfVec3f(0.1f * (i % 7), 1.0f, -0.3f);
                envelope[i] = i % 4 == 0 ? 0.0f : 0.11f * (i % 11);
            }
            std::vector<GfVec3f> blended = expected;
            Blend(authored, envelope, &blended);
            const Points authoredArray = ToStd(authored);
            const auto envelopeArray = ToArray(envelope);
            CHECK(node.Run(
                {&in, &counts, &indices, &authoredArray, &envelopeArray},
                &actual));
            CHECK(SameBits(blended, actual));
            const Points shortAuthored(3);
            CHECK(!node.Run(
                {&in, &counts, &indices, &shortAuthored, &envelopeArray},
                &actual, &error));
            CHECK(error.find("authored") != std::string::npos);
        }
    // An envelope needs the authored value.
    const NormalsPayload defaults;
    Node noAuthored(RecomputeNormals(),
                    {&RecomputeNormalsPoints(), &RecomputeNormalsCounts(),
                     &RecomputeNormalsIndices(), &RecomputeNormalsEnvelope()},
                    RecomputeNormalsOut(), &defaults);
    const RigExecArray<float> envelope(in.Size());
    Points actual;
    std::string error;
    CHECK(
        !noAuthored.Run({&in, &counts, &indices, &envelope}, &actual, &error));
    CHECK(error.find("authored") != std::string::npos);
}

void
TestRecomputeExtent()
{
    using namespace rigExecOps;
    Grid grid(6, 5);
    const Points in = ToStd(grid.points);
    Node bare(RecomputeExtent(),
              {&RecomputeExtentPoints(), &RecomputeExtentWidths()},
              RecomputeExtentOut());
    Node node(RecomputeExtent(),
              {&RecomputeExtentPoints(), &RecomputeExtentWidths(),
               &RecomputeExtentAuthored(), &RecomputeExtentEnvelope()},
              RecomputeExtentOut());
    std::vector<float> perPoint(grid.points.size());
    for (size_t i = 0; i < perPoint.size(); ++i)
        perPoint[i] = 0.05f * (i % 9);
    for (const std::vector<float> &widths :
         {std::vector<float>(), std::vector<float>{0.7f}, perPoint,
          std::vector<float>{0.2f, 0.4f}}) {
        const std::vector<GfVec3f> expected =
            RigExecComputeExtent(grid.points, widths);
        CHECK(expected.size() == 2);
        const auto widthArray = ToArray(widths);
        Points actual;
        CHECK(bare.Run({&in, &widthArray}, &actual));
        CHECK(SameBits(expected, actual));
        const std::vector<GfVec3f> authored = {GfVec3f(-9.0f), GfVec3f(9.0f)};
        const std::vector<float> envelope = {0.25f, 1.0f};
        std::vector<GfVec3f> blended = expected;
        Blend(authored, envelope, &blended);
        const Points authoredArray = ToStd(authored);
        const auto envelopeArray = ToArray(envelope);
        CHECK(node.Run({&in, &widthArray, &authoredArray, &envelopeArray},
                       &actual));
        CHECK(SameBits(blended, actual));
    }
    const Points none;
    const RigExecArray<float> noWidths;
    Points actual;
    std::string error;
    CHECK(!bare.Run({&none, &noWidths}, &actual, &error));
    CHECK(error.find("no points") != std::string::npos);
}

void
TestWire()
{
    using namespace rigExecOps;
    Grid grid(9, 17);
    // A clamped cubic along the grid's diagonal, and a posed copy.
    std::vector<GfVec3f> rest, posed;
    for (int i = 0; i < 6; ++i) {
        rest.emplace_back(1.6f * i, 1.5f * i, 0.4f * (i % 2));
        posed.push_back(rest.back() +
                        GfVec3f(0.3f * i, -0.2f * (i % 3), 0.5f + 0.1f * i));
    }
    posed[2] = rest[2]; // a control point at rest moves nothing
    WirePayload payload;
    payload.order = 4;
    payload.knots = {0, 0, 0, 0, 1, 2, 3, 3, 3, 3};
    const RigExecNurbsCurve restCurve{&rest, payload.order, &payload.knots};
    const RigExecNurbsCurve posedCurve{&posed, payload.order, &payload.knots};
    const std::vector<GfVec2f> bind = RigExecBindWire(grid.points, restCurve);

    const Points in = ToStd(grid.points);
    const Points restArray = ToStd(rest);
    const Points posedArray = ToStd(posed);
    RigExecArray<rigExecStd::Vec2f> bindArray(bind.size());
    for (size_t i = 0; i < bind.size(); ++i)
        bindArray[i] = rigExecStd::Vec2f(bind[i][0], bind[i][1]);
    std::vector<float> envelope(grid.points.size());
    for (size_t i = 0; i < envelope.size(); ++i)
        envelope[i] = i % 6 == 0 ? 0.0f : 0.09f * (i % 13);
    const auto envelopeArray = ToArray(envelope);

    for (const double dropoff : {0.0, 1.5, 4.0}) {
        payload.dropoffDistance = dropoff;
        Node dense(Wire(),
                   {&WirePoints(), &WireRestCurve(), &WireCurve(),
                    &WireBindCoords(), &WireEnvelope()},
                   WireOut(), &payload);
        std::vector<GfVec3f> expected = grid.points;
        CHECK(RigExecApplyWire(&expected, restCurve, posedCurve, bind.data(),
                               bind.size(), dropoff, 0, expected.size()));
        Blend(grid.points, envelope, &expected);
        Points actual;
        CHECK(dense.Run(
            {&in, &restArray, &posedArray, &bindArray, &envelopeArray},
            &actual));
        CHECK(SameBits(expected, actual));

        // A sparse field moves only the named points, by their weights,
        // with binds one per point or one per index.
        Node sparse(Wire(),
                    {&WirePoints(), &WireRestCurve(), &WireCurve(),
                     &WireBindCoords(), &WireWeightIndices(), &WireWeights()},
                    WireOut(), &payload);
        std::vector<int> indices;
        std::vector<float> weights;
        std::vector<GfVec2f> sparseBind;
        for (int i = 1; i < int(grid.points.size()); i += 3) {
            indices.push_back(i);
            weights.push_back(0.2f + 0.07f * (i % 9));
            sparseBind.push_back(bind[size_t(i)]);
        }
        const auto indexArray = ToArray(indices);
        const auto weightArray = ToArray(weights);
        for (const bool perIndex : {false, true}) {
            const std::vector<GfVec2f> &b = perIndex ? sparseBind : bind;
            RigExecWireBasis basis;
            CHECK(RigExecBuildWireBasis(b.data(), b.size(), grid.points.size(),
                                        indices, payload.order, payload.knots,
                                        rest.size(), dropoff, &basis));
            expected = grid.points;
            CHECK(RigExecApplyWireBasis(&expected, basis, indices, weights,
                                        rest, posed));
            RigExecArray<rigExecStd::Vec2f> bArray(b.size());
            for (size_t i = 0; i < b.size(); ++i)
                bArray[i] = rigExecStd::Vec2f(b[i][0], b[i][1]);
            CHECK(sparse.Run({&in, &restArray, &posedArray, &bArray,
                              &indexArray, &weightArray},
                             &actual));
            CHECK(SameBits(expected, actual));
        }
    }

    // Curves that disagree, binds of another length and both weightings at
    // once are refused.
    Node all(Wire(),
             {&WirePoints(), &WireRestCurve(), &WireCurve(), &WireBindCoords(),
              &WireEnvelope(), &WireWeightIndices(), &WireWeights()},
             WireOut(), &payload);
    Node dense(
        Wire(),
        {&WirePoints(), &WireRestCurve(), &WireCurve(), &WireBindCoords()},
        WireOut(), &payload);
    const Points shortCurve = ToStd({rest[0], rest[1], rest[2], rest[3]});
    const RigExecArray<rigExecStd::Vec2f> shortBind(3);
    const RigExecArray<int> oneIndex(1);
    const RigExecArray<float> oneWeight(1);
    Points actual;
    std::string error;
    CHECK(!dense.Run({&in, &restArray, &shortCurve, &bindArray}, &actual,
                     &error));
    CHECK(error.find("fit") != std::string::npos);
    CHECK(!dense.Run({&in, &restArray, &posedArray, &shortBind}, &actual,
                     &error));
    CHECK(error.find("fit") != std::string::npos);
    CHECK(!all.Run({&in, &restArray, &posedArray, &bindArray, &envelopeArray,
                    &oneIndex, &oneWeight},
                   &actual, &error));
    CHECK(error.find("not both") != std::string::npos);
    // Nodes with different falloffs in one rig each run with their own.
    std::vector<WirePayload> payloads(3, payload);
    payloads[0].dropoffDistance = 0.0;
    payloads[1].dropoffDistance = 1.5;
    payloads[2].dropoffDistance = 4.0;
    const std::vector<const RigExecPortHandle *> ports = {
        &WirePoints(), &WireRestCurve(), &WireCurve(), &WireBindCoords(),
        &WireEnvelope()};
    const std::vector<const void *> values = {&in, &restArray, &posedArray,
                                              &bindArray, &envelopeArray};
    std::vector<Points> results;
    CHECK(RunEach(Wire(), ports, WireOut(), payloads, values, &results));
    for (size_t i = 0; i < results.size(); ++i) {
        Node single(Wire(), ports, WireOut(), &payloads[i]);
        CHECK(single.Run(values, &actual) && SamePoints(actual, results[i]));
    }
}

void
TestRibbon()
{
    using namespace rigExecOps;
    Grid grid(8, 29);
    std::vector<GfVec3f> restCurve, posedCurve;
    for (int i = 0; i < 6; ++i) {
        restCurve.emplace_back(1.4f * i, 0.5f * i, 0.3f * (i % 2));
        posedCurve.emplace_back(1.3f * i, 0.5f * i + 0.2f * i * i,
                                0.4f * i - 0.3f * (i % 2));
    }
    const RigExecCurveFrameSamples rest = RigExecSampleCurveRMF(restCurve, 9);
    const RigExecCurveFrameSamples posed = RigExecSampleCurveRMF(posedCurve, 9);
    // The frame points as the samples give them: offsets summed in float.
    const auto landmarks = [](const RigExecCurveFrameSamples &f, size_t k) {
        const GfVec3f o = f.positions[k];
        const GfVec3f p[4] = {o, o + f.tangents[k], o + f.normals[k],
                              o + f.binormals[k]};
        rigExecRig::PointLandmarks l;
        for (int j = 0; j < 4; ++j)
            l[j] = rigExecStd::Vec3d(p[j][0], p[j][1], p[j][2]);
        return l;
    };
    rigExecRig::PointFrameArray frames;
    frames.frames = RigExecArray<rigExecRig::PointFrame>(rest.GetSize());
    frames.rests = RigExecArray<rigExecRig::PointLandmarks>(rest.GetSize());
    for (size_t k = 0; k < rest.GetSize(); ++k) {
        frames.rests[k] = landmarks(rest, k);
        frames.frames[k].points = landmarks(posed, k);
    }
    std::vector<GfVec2f> bind(grid.points.size());
    RigExecArray<rigExecStd::Vec2f> bindArray(bind.size());
    for (size_t i = 0; i < bind.size(); ++i) {
        // Past both ends too: u clamps to [0, 1].
        bind[i] = GfVec2f(-0.1f + 1.2f * float(i) / float(bind.size()), 0.0f);
        bindArray[i] = rigExecStd::Vec2f(bind[i][0], bind[i][1]);
    }
    std::vector<float> envelope(grid.points.size());
    for (size_t i = 0; i < envelope.size(); ++i)
        envelope[i] = i % 5 == 0 ? 1.0f : 0.07f * (i % 15);
    const auto envelopeArray = ToArray(envelope);
    const Points in = ToStd(grid.points);

    Node node(Ribbon(),
              {&RibbonPoints(), &RibbonBindCoords(), &RibbonFrames(),
               &RibbonEnvelope()},
              RibbonOut());
    std::vector<GfVec3f> expected = grid.points;
    RigExecApplyRibbonTransport(&expected, bind, rest, posed);
    Blend(grid.points, envelope, &expected);
    Points actual;
    CHECK(node.Run({&in, &bindArray, &frames, &envelopeArray}, &actual));
    CHECK(SameBits(expected, actual));

    // One sample, or binds of another length, are refused.
    rigExecRig::PointFrameArray one;
    one.frames = RigExecArray<rigExecRig::PointFrame>(1);
    one.rests = RigExecArray<rigExecRig::PointLandmarks>(1);
    const RigExecArray<rigExecStd::Vec2f> shortBind(2);
    std::string error;
    CHECK(!node.Run({&in, &bindArray, &one, &envelopeArray}, &actual, &error));
    CHECK(error.find("two frame samples") != std::string::npos);
    CHECK(
        !node.Run({&in, &shortBind, &frames, &envelopeArray}, &actual, &error));
    CHECK(error.find("bind") != std::string::npos);

    // Guide points are the posed frame origins, narrowed as the native path
    // narrows them.
    Node guides(
        GuidePoints(),
        {&GuidePointsPoints(), &GuidePointsFrames(), &GuidePointsEnvelope()},
        GuidePointsOut());
    const Points guideIn =
        ToStd(std::vector<GfVec3f>(rest.GetSize(), GfVec3f(0.5f, -1.0f, 2.0f)));
    std::vector<float> guideEnvelope(rest.GetSize());
    for (size_t k = 0; k < guideEnvelope.size(); ++k)
        guideEnvelope[k] = 0.125f * float(k);
    std::vector<GfVec3f> guideExpected(rest.GetSize());
    std::vector<GfVec3f> guideStart(rest.GetSize(), GfVec3f(0.5f, -1.0f, 2.0f));
    for (size_t k = 0; k < guideExpected.size(); ++k) {
        const rigExecStd::Vec3d &o = frames.frames[k].points[0];
        guideExpected[k] = GfVec3f(GfVec3d(o[0], o[1], o[2]));
    }
    Blend(guideStart, guideEnvelope, &guideExpected);
    const auto guideEnvelopeArray = ToArray(guideEnvelope);
    CHECK(guides.Run({&guideIn, &frames, &guideEnvelopeArray}, &actual));
    CHECK(SameBits(guideExpected, actual));
    CHECK(!guides.Run({&in, &frames, &envelopeArray}, &actual, &error));
    CHECK(error.find("one frame per point") != std::string::npos);
}

bool
SameBits(const GfMatrix4d &expected, const rigExecStd::Mat4d &actual)
{
    return std::memcmp(expected.GetArray(), &actual._mtx[0][0],
                       sizeof(double) * 16) == 0;
}

void
TestSurfaceProjector()
{
    using namespace rigExecOps;
    using rigExecStd::Mat4d;
    using rigExecStd::Vec3d;
    const Grid grid(9, 41, true);
    std::vector<GfVec3f> posed = grid.points;
    const GfMatrix4d move =
        GfMatrix4d().SetRotate(GfRotation(GfVec3d(0.3, 1.0, 0.2), 25.0)) *
        GfMatrix4d().SetTranslate(GfVec3d(0.5, -1.0, 2.0));
    for (size_t i = 0; i < posed.size(); ++i)
        posed[i] = GfVec3f(move.Transform(GfVec3d(posed[i])) +
                           GfVec3d(0.0, 0.0, 0.05 * double(i % 3)));
    const Points base = ToStd(grid.points), posedArray = ToStd(posed);
    const auto counts = ToArray(grid.counts);
    const auto indices = ToArray(grid.indices);
    const GfMatrix4d offset =
        GfMatrix4d().SetScale(GfVec3d(1.5, 0.75, 1.0)) *
        GfMatrix4d().SetTranslate(GfVec3d(0.25, 0.0, -0.5));
    // The source looks down -Z at the grid.
    const GfMatrix4d down =
        GfMatrix4d().SetRotate(GfRotation(GfVec3d(1, 0, 0), 180.0));
    const GfMatrix4d sourceBase =
        down * GfMatrix4d().SetTranslate(GfVec3d(4.0, 4.0, 5.0));
    const GfMatrix4d sourceFinal =
        down * GfMatrix4d().SetRotate(GfRotation(GfVec3d(0, 0, 1), 15.0)) *
        GfMatrix4d().SetTranslate(GfVec3d(4.5, 4.0, 5.5));
    const GfMatrix4d spaceBase = GfMatrix4d(1.0);
    const GfMatrix4d spaceFinal =
        GfMatrix4d().SetTranslate(GfVec3d(0.0, 0.5, 0.25));
    const GfMatrix4d rigSpace = GfMatrix4d().SetScale(1.25);
    const GfMatrix4d worldToMesh =
        GfMatrix4d().SetTranslate(GfVec3d(-0.5, 0.0, 0.0));
    const Mat4d sourceBaseArray = ToStd(sourceBase),
                sourceFinalArray = ToStd(sourceFinal),
                spaceBaseArray = ToStd(spaceBase),
                spaceFinalArray = ToStd(spaceFinal),
                rigSpaceArray = ToStd(rigSpace),
                worldToMeshArray = ToStd(worldToMesh);
    for (const bool reproject : {false, true})
        for (const bool named : {false, true}) {
            SurfaceProjectorPayload payload;
            payload.rayOrigin = Vec3d(4.0, 4.0, 5.0);
            payload.rayDirection = Vec3d(0.0, 0.0, -1.0);
            payload.rayUp = Vec3d(0.0, 1.0, 0.0);
            payload.shaderOffset = ToStd(offset);
            payload.reproject = reproject;
            payload.sourceSpaceNamed = named;
            payload.spaceNamed = named;
            RigExecSurfaceProjectorInputs<GfMatrix4d, GfVec3d> in;
            in.rayOrigin = GfVec3d(4.0, 4.0, 5.0);
            in.rayDirection = GfVec3d(0.0, 0.0, -1.0);
            in.rayUp = GfVec3d(0.0, 1.0, 0.0);
            in.shaderOffset = offset;
            in.reproject = reproject;
            in.hasSource = true;
            in.sourceBase = sourceBase;
            in.sourceFinal = sourceFinal;
            in.sourceSpaceNamed = named;
            in.hasSourceSpace = named;
            in.sourceSpaceBase = named ? spaceBase : GfMatrix4d(1.0);
            in.sourceSpaceFinal = named ? spaceFinal : GfMatrix4d(1.0);
            in.spaceNamed = named;
            in.hasSpace = named;
            in.spaceFinal = named ? rigSpace : GfMatrix4d(1.0);
            in.worldToMesh = worldToMesh;
            GfMatrix4d expected;
            std::vector<std::string> diagnostics;
            CHECK(RigExecSolveSurfaceProjectorT(
                in, grid.points, posed, grid.counts, grid.indices,
                static_cast<RigExecVertexNormalsFn>(
                    &RigExecComputeVertexNormals),
                "", &expected, &diagnostics));
            Node node(
                SurfaceProjector(),
                {&SurfaceProjectorBasePoints(), &SurfaceProjectorPoints(),
                 &SurfaceProjectorCounts(), &SurfaceProjectorIndices(),
                 &SurfaceProjectorSourceBase(), &SurfaceProjectorSourceFinal(),
                 &SurfaceProjectorSourceSpaceBase(),
                 &SurfaceProjectorSourceSpaceFinal(),
                 &SurfaceProjectorSpaceFinal(), &SurfaceProjectorWorldToMesh()},
                SurfaceProjectorOut(), &payload);
            Mat4d actual(0.0);
            CHECK(
                node.Run({&base, &posedArray, &counts, &indices,
                          &sourceBaseArray, &sourceFinalArray, &spaceBaseArray,
                          &spaceFinalArray, &rigSpaceArray, &worldToMeshArray},
                         &actual));
            CHECK(SameBits(expected, actual));
        }
    {
        SurfaceProjectorPayload steady;
        steady.rayOrigin = Vec3d(4.0, 4.0, 5.0);
        steady.rayDirection = Vec3d(0.0, 0.0, -1.0);
        steady.rayUp = Vec3d(0.0, 1.0, 0.0);
        steady.shaderOffset = ToStd(offset);
        steady.reproject = true;
        Mat4d sourceMoving = sourceFinalArray;
        std::string steadyError;
        CHECK(
            opAllocationCount::SteadyFrameAllocations(
                SurfaceProjector(), &steady,
                {&SurfaceProjectorBasePoints(), &SurfaceProjectorPoints(),
                 &SurfaceProjectorCounts(), &SurfaceProjectorIndices(),
                 &SurfaceProjectorSourceBase(), &SurfaceProjectorSourceFinal(),
                 &SurfaceProjectorWorldToMesh()},
                {&base, &posedArray, &counts, &indices, &sourceBaseArray,
                 &sourceFinalArray, &worldToMeshArray},
                SurfaceProjectorOut(), 5,
                [&](int frame) {
                    sourceMoving =
                        ToStd(sourceFinal * GfMatrix4d().SetTranslate(GfVec3d(
                                                0.05 * frame, 0.0, 0.0)));
                    return &sourceMoving;
                },
                &steadyError) == 0);
    }
    // Unfed source, spaces and worldToMesh are identity and unresolved.
    SurfaceProjectorPayload payload;
    payload.rayOrigin = Vec3d(3.0, 5.0, 4.0);
    payload.rayDirection = Vec3d(0.0, 0.0, -1.0);
    RigExecSurfaceProjectorInputs<GfMatrix4d, GfVec3d> in;
    in.rayOrigin = GfVec3d(3.0, 5.0, 4.0);
    in.rayDirection = GfVec3d(0.0, 0.0, -1.0);
    in.rayUp = GfVec3d(0.0, 1.0, 0.0);
    in.shaderOffset = GfMatrix4d(1.0);
    in.sourceBase = in.sourceFinal = in.sourceSpaceBase = in.sourceSpaceFinal =
        in.spaceFinal = in.worldToMesh = GfMatrix4d(1.0);
    GfMatrix4d expected;
    std::vector<std::string> diagnostics;
    CHECK(RigExecSolveSurfaceProjectorT(
        in, grid.points, posed, grid.counts, grid.indices,
        static_cast<RigExecVertexNormalsFn>(&RigExecComputeVertexNormals), "",
        &expected, &diagnostics));
    Node bare(SurfaceProjector(),
              {&SurfaceProjectorBasePoints(), &SurfaceProjectorPoints(),
               &SurfaceProjectorCounts(), &SurfaceProjectorIndices()},
              SurfaceProjectorOut(), &payload);
    Mat4d actual(0.0);
    CHECK(bare.Run({&base, &posedArray, &counts, &indices}, &actual));
    CHECK(SameBits(expected, actual));
    // A ray that misses the surface measures no frame: the output is the
    // shaderOffset, in material and reproject mode alike.
    payload.rayOrigin = Vec3d(40.0, 40.0, 5.0);
    payload.shaderOffset = ToStd(offset);
    const std::vector<const RigExecPortHandle *> missInputs = {
        &SurfaceProjectorBasePoints(), &SurfaceProjectorPoints(),
        &SurfaceProjectorCounts(), &SurfaceProjectorIndices()};
    const std::vector<const void *> missValues = {&base, &posedArray, &counts,
                                                  &indices};
    for (const bool reproject : {false, true}) {
        payload.reproject = reproject;
        Node miss(SurfaceProjector(), missInputs, SurfaceProjectorOut(),
                  &payload);
        actual = Mat4d(0.0);
        CHECK(miss.Run(missValues, &actual));
        CHECK(SameBits(offset, actual));
    }
    payload.reproject = false;
    std::string error;
    // A ray with no direction cannot be cast: the evaluation stops.
    payload.rayDirection = Vec3d(0.0, 0.0, 0.0);
    Node noDirection(SurfaceProjector(), missInputs, SurfaceProjectorOut(),
                     &payload);
    CHECK(!noDirection.Run(missValues, &actual, &error));
    CHECK(Contains(error, "(rig:SurfaceProjector) failed: SurfaceProjector: "
                          "ray "));
    CHECK(Contains(error, "is not finite or has no direction"));
    payload.rayDirection = Vec3d(0.0, 0.0, -1.0);
    // A face corner past the surface's points stops the evaluation.
    std::vector<int> pastEnd = grid.indices;
    pastEnd[0] = int(grid.points.size());
    const auto badIndices = ToArray(pastEnd);
    Node badTopology(SurfaceProjector(), missInputs, SurfaceProjectorOut(),
                     &payload);
    error.clear();
    CHECK(!badTopology.Run({&base, &posedArray, &counts, &badIndices}, &actual,
                           &error));
    CHECK(Contains(error, "surface has no usable points or topology"));
    // Half a source fails compile.
    Node half(SurfaceProjector(),
              {&SurfaceProjectorBasePoints(), &SurfaceProjectorPoints(),
               &SurfaceProjectorCounts(), &SurfaceProjectorIndices(),
               &SurfaceProjectorSourceBase()},
              SurfaceProjectorOut(), &payload);
    error.clear();
    CHECK(!half.Run({&base, &posedArray, &counts, &indices, &sourceBaseArray},
                    &actual, &error));
    CHECK(Contains(error, "(rig:SurfaceProjector) is invalid: "
                          "SurfaceProjector: a source or source space needs "
                          "its base and final"));
}

void
TestShaderDials()
{
    using namespace rigExecOps;
    Node node(ShaderDials(), {&ShaderDialsDials()}, ShaderDialsOut());
    for (const size_t count : {size_t(0), size_t(5), size_t(16), size_t(20)}) {
        std::vector<double> dials(count);
        for (size_t i = 0; i < count; ++i)
            dials[i] = i == 3 ? std::nan("") : 0.37 * double(i) - 1.0;
        const auto array = ToArray(dials);
        rigExecStd::Mat4d actual(1.0);
        CHECK(node.Run({&array}, &actual));
        CHECK(SameBits(RigExecPackShaderDialsT<GfMatrix4d>(dials), actual));
    }
}

const std::array<GfVec3d, 4> kIdentityLandmarks = {
    GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};

/// A frame carried by translate, rotate (degrees about an axis) and scale.
RigExecPointFrame
MakeFrame(const GfVec3d &t, const GfVec3d &axis, double degrees,
          const GfVec3d &scale)
{
    GfMatrix4d s(1.0), r(1.0), m;
    s.SetScale(scale);
    r.SetRotate(GfRotation(axis, degrees));
    m = s * r;
    m.SetTranslateOnly(t);
    return RigExecMatrixToPoints(kIdentityLandmarks, m);
}

rigExecRig::PointFrame
ToRig(const RigExecPointFrame &f)
{
    rigExecRig::PointFrame out;
    for (int i = 0; i < 4; ++i)
        out.points[i] =
            rigExecStd::Vec3d(f.points[i][0], f.points[i][1], f.points[i][2]);
    out.flags = f.flags;
    return out;
}

rigExecStd::Vec3d
ToRig(const GfVec3d &v)
{
    return rigExecStd::Vec3d(v[0], v[1], v[2]);
}

rigExecStd::Mat4d
ToRig(const GfMatrix4d &m)
{
    rigExecStd::Mat4d out;
    std::memcpy(&out._mtx[0][0], m.GetArray(), sizeof(double) * 16);
    return out;
}

bool
SameBits(const RigExecPointFrame &expected, const rigExecRig::PointFrame &a)
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) {
            const double e = expected.points[i][j], v = a.points[i][j];
            if (std::memcmp(&e, &v, sizeof(double)) != 0)
                return false;
        }
    return expected.flags == a.flags;
}

void
TestMoverTransform()
{
    using namespace rigExecOps;
    GfMatrix4d transform(1.0);
    transform.SetRotate(GfRotation(GfVec3d(0.2, 1, 0.4), 35.0));
    transform.SetTranslateOnly(GfVec3d(1.5, -0.5, 2));
    const GfMatrix4d space =
        GfMatrix4d(1.0).SetScale(GfVec3d(1.3, 0.7, 2.0)) *
        GfMatrix4d(1.0).SetRotate(GfRotation(GfVec3d(1, 0, 0.3), -20.0)) *
        GfMatrix4d(1.0).SetTranslate(GfVec3d(0.25, 1, -3));
    const GfMatrix4d unscaledSpace =
        GfMatrix4d(1.0).SetRotate(GfRotation(GfVec3d(0, 1, 0), 15.0)) *
        GfMatrix4d(1.0).SetTranslate(GfVec3d(-1, 0, 0.5));
    const GfMatrix4d reference =
        GfMatrix4d(1.0).SetRotate(GfRotation(GfVec3d(0, 0, 1), 10.0)) *
        GfMatrix4d(1.0).SetTranslate(GfVec3d(0.5, 0.5, 0));
    const GfMatrix4d referenceSpace =
        GfMatrix4d(1.0).SetScale(GfVec3d(1.1, 1.1, 0.9)) *
        GfMatrix4d(1.0).SetTranslate(GfVec3d(0, -2, 1));
    const GfMatrix4d carry =
        GfMatrix4d(1.0).SetRotate(GfRotation(GfVec3d(0, 1, 1), 50.0)) *
        GfMatrix4d(1.0).SetTranslate(GfVec3d(4, 0, -1));
    struct Case {
        const GfMatrix4d *space, *reference, *referenceSpace, *carry;
        bool posed;
    };
    const Case cases[] = {
        {nullptr, nullptr, nullptr, nullptr, false},
        {nullptr, nullptr, nullptr, nullptr, true},
        {&space, nullptr, nullptr, nullptr, false},
        {&space, nullptr, nullptr, nullptr, true},
        {&unscaledSpace, nullptr, nullptr, nullptr, true},
        {&space, nullptr, nullptr, &carry, true},
        {&space, nullptr, nullptr, &carry, false},
        {nullptr, &reference, nullptr, nullptr, true},
        {&space, &reference, nullptr, nullptr, true},
        {&space, &reference, &referenceSpace, nullptr, true},
        {&space, &reference, &referenceSpace, &carry, true},
    };
    for (const Case &c : cases) {
        MoverTransformPayload payload;
        payload.posedPoints = c.posed;
        std::vector<const RigExecPortHandle *> ports = {
            &MoverTransformTransform()};
        std::vector<rigExecStd::Mat4d> values = {ToStd(transform)};
        const auto feed = [&](const GfMatrix4d *m,
                              const RigExecPortHandle &port) {
            if (m) {
                ports.push_back(&port);
                values.push_back(ToStd(*m));
            }
        };
        feed(c.space, MoverTransformSpace());
        feed(c.reference, MoverTransformReference());
        feed(c.referenceSpace, MoverTransformReferenceSpace());
        feed(c.carry, MoverTransformCarry());
        std::vector<const void *> bound;
        for (const auto &value : values)
            bound.push_back(&value);
        Node node(MoverTransform(), ports, MoverTransformOut(), &payload);
        rigExecStd::Mat4d actual;
        std::string error;
        CHECK(node.Run(bound, &actual, &error));
        const GfMatrix4d expected = RigExecMatrixInPointFrameT<GfVec3d>(
            transform, c.space, c.reference, c.referenceSpace, c.posed,
            c.carry);
        if (!SameBits(expected, actual)) {
            std::printf("mover transform case %zu differs: %s\n",
                        size_t(&c - cases), error.c_str());
            CHECK(false);
        }
    }

    // Unposed points, no space: the transform unchanged; a still space with
    // no carry and no scale: the offset alone.
    const GfMatrix4d *none = nullptr;
    CHECK(RigExecMatrixInPointFrameT<GfVec3d>(transform, none, none, none, true,
                                              none) == transform);
    CHECK(RigExecMatrixInPointFrameT<GfVec3d>(transform, &unscaledSpace, none,
                                              none, true, none) ==
          RigExecMeasureInSpaceT(transform, unscaledSpace));

    // A reference space with no space to measure is a compile error.
    MoverTransformPayload payload;
    Node orphan(MoverTransform(),
                {&MoverTransformTransform(), &MoverTransformReference(),
                 &MoverTransformReferenceSpace()},
                MoverTransformOut(), &payload);
    const auto m = ToStd(transform);
    rigExecStd::Mat4d actual;
    std::string error;
    CHECK(!orphan.Run({&m, &m, &m}, &actual, &error));
    CHECK(error.find("a reference space needs Space and Reference") !=
          std::string::npos);
}

void
TestConstraints()
{
    using namespace rigExecOps;
    const RigExecPointFrame input =
        MakeFrame(GfVec3d(0.5, -1.0, 2.0), GfVec3d(0.2, 1.0, -0.4), 37.0,
                  GfVec3d(1.0, 1.3, 0.8));
    std::vector<RigExecConstraintSource> gfSources(3);
    gfSources[0].frame = MakeFrame(GfVec3d(3, 1, -2), GfVec3d(1, 0.3, 0), 120.0,
                                   GfVec3d(2.0, 0.5, 1.0));
    gfSources[1].frame = MakeFrame(GfVec3d(-1, 4, 0.5), GfVec3d(0, 0.4, 1),
                                   -75.0, GfVec3d(0.7, 0.7, 1.9));
    gfSources[2].frame = MakeFrame(GfVec3d(2, 2, 2), GfVec3d(1, 1, 1), 170.0,
                                   GfVec3d(1.1, 1.0, 0.9));
    const double sourceWeights[3] = {0.5, 1.25, 0.0};
    rigExecRig::PointFrameArray sources;
    sources.frames = RigExecArray<rigExecRig::PointFrame>(3);
    sources.rests = RigExecArray<rigExecRig::PointLandmarks>(3);
    std::vector<double> weightValues(3);
    for (size_t i = 0; i < 3; ++i) {
        gfSources[i].normalizedWeight = sourceWeights[i];
        sources.frames[i] = ToRig(gfSources[i].frame);
        weightValues[i] = sourceWeights[i];
    }
    const auto weights = ToArray(weightValues);
    const rigExecRig::PointFrame in = ToRig(input);
    const double weight = 0.6, full = 1.0;
    GfMatrix4d carry(1.0);
    carry.SetRotate(GfRotation(GfVec3d(0.3, 0.1, 1.0), 64.0));
    carry.SetTranslateOnly(GfVec3d(4.0, -2.0, 1.0));
    const rigExecStd::Mat4d carryArray = ToRig(carry);
    rigExecRig::PointFrame actual;

    // Position, partial mask, at a partial and at full weight.
    ConstraintPayload position;
    position.offset = rigExecStd::Vec3d(0.25, 0.0, -0.5);
    position.affect = {true, false, true};
    RigExecPositionConstraintParams positionParams;
    positionParams.offset = GfVec3d(0.25, 0.0, -0.5);
    positionParams.affect = {true, false, true};
    Node positionNode(PositionConstraint(),
                      {&PositionConstraintIn(), &PositionConstraintSources(),
                       &PositionConstraintSourceWeights(),
                       &PositionConstraintWeight()},
                      PositionConstraintOut(), &position);
    for (const double *w : {&weight, &full}) {
        positionParams.weight = *w;
        CHECK(positionNode.Run({&in, &sources, &weights, w}, &actual));
        CHECK(SameBits(
            RigExecApplyPositionConstraint(input, gfSources, positionParams),
            actual));
    }

    // Rotation, ZXY with a partial mask, bare and carried by a space.
    ConstraintPayload rotation;
    rotation.offset = rigExecStd::Vec3d(10.0, -5.0, 0.0);
    rotation.affect = {true, true, false};
    rotation.rotationOrder = ConstraintEulerOrder::ZXY;
    RigExecRotationConstraintParams rotationParams;
    rotationParams.offsetDegrees = GfVec3d(10.0, -5.0, 0.0);
    rotationParams.affect = {true, true, false};
    rotationParams.rotationOrder = RigExecEulerOrder::ZXY;
    rotationParams.weight = weight;
    Node rotationNode(RotationConstraint(),
                      {&RotationConstraintIn(), &RotationConstraintSources(),
                       &RotationConstraintSourceWeights(),
                       &RotationConstraintWeight()},
                      RotationConstraintOut(), &rotation);
    CHECK(rotationNode.Run({&in, &sources, &weights, &weight}, &actual));
    CHECK(SameBits(
        RigExecApplyRotationConstraint(input, gfSources, rotationParams),
        actual));
    Node rotationSpaceNode(
        RotationConstraint(),
        {&RotationConstraintIn(), &RotationConstraintSources(),
         &RotationConstraintSourceWeights(), &RotationConstraintWeight(),
         &RotationConstraintSpace()},
        RotationConstraintOut(), &rotation);
    rotationParams.carry = &carry;
    CHECK(rotationSpaceNode.Run({&in, &sources, &weights, &weight, &carryArray},
                                &actual));
    CHECK(SameBits(
        RigExecApplyRotationConstraint(input, gfSources, rotationParams),
        actual));

    // Scale blending shear; unfed source weights weigh 1 each.
    ConstraintPayload scale;
    scale.offset = rigExecStd::Vec3d(0.1, 0.0, -0.1);
    scale.blendShear = true;
    RigExecScaleConstraintParams scaleParams;
    scaleParams.offset = GfVec3d(0.1, 0.0, -0.1);
    scaleParams.blendShear = true;
    scaleParams.weight = weight;
    Node scaleNode(ScaleConstraint(),
                   {&ScaleConstraintIn(), &ScaleConstraintSources(),
                    &ScaleConstraintWeight()},
                   ScaleConstraintOut(), &scale);
    std::vector<RigExecConstraintSource> unweighted = gfSources;
    for (auto &source : unweighted)
        source.normalizedWeight = 1.0;
    CHECK(scaleNode.Run({&in, &sources, &weight}, &actual));
    CHECK(SameBits(RigExecApplyScaleConstraint(input, unweighted, scaleParams),
                   actual));

    // Parent with per-source offsets, scale axes and a space.
    ConstraintPayload parent;
    parent.scaleAxes = {true, true, true};
    parent.rotationAxes = {false, true, true};
    parent.rotationOrder = ConstraintEulerOrder::YZX;
    parent.blendShear = true;
    RigExecParentConstraintParams parentParams;
    parentParams.scaleAxes = {true, true, true};
    parentParams.rotationAxes = {false, true, true};
    parentParams.rotationOrder = RigExecEulerOrder::YZX;
    parentParams.blendShear = true;
    parentParams.weight = weight;
    parentParams.carry = &carry;
    for (size_t i = 0; i < 3; ++i) {
        const GfVec3d t(0.5 * i, -0.25, 1.0), r(15.0 * i, 30.0, -10.0);
        gfSources[i].translationOffset = t;
        gfSources[i].rotationOffsetDegrees = r;
        parent.translationOffsets.push_back(ToRig(t));
        parent.rotationOffsetsDegrees.push_back(ToRig(r));
    }
    Node parentNode(ParentConstraint(),
                    {&ParentConstraintIn(), &ParentConstraintSources(),
                     &ParentConstraintSourceWeights(),
                     &ParentConstraintWeight(), &ParentConstraintSpace()},
                    ParentConstraintOut(), &parent);
    CHECK(parentNode.Run({&in, &sources, &weights, &weight, &carryArray},
                         &actual));
    CHECK(SameBits(RigExecApplyParentConstraint(input, gfSources, parentParams),
                   actual));

    // Aim with a world up, then without one taking the minimum swing.
    ConstraintPayload aim;
    aim.localAimVector = rigExecStd::Vec3d(0.0, 0.0, 1.0);
    aim.localUpVector = rigExecStd::Vec3d(1.0, 0.0, 0.0);
    aim.offset = rigExecStd::Vec3d(0.0, 0.0, 12.0);
    RigExecAimConstraintParams aimParams;
    aimParams.localAimVector = GfVec3d(0.0, 0.0, 1.0);
    aimParams.localUpVector = GfVec3d(1.0, 0.0, 0.0);
    aimParams.rotationOffsetDegrees = GfVec3d(0.0, 0.0, 12.0);
    aimParams.weight = weight;
    const GfVec3d target(5.0, 3.0, -4.0), worldUp(0.0, 1.0, 0.2);
    const rigExecStd::Vec3d targetValue = ToRig(target);
    const rigExecStd::Vec3d worldUpValue = ToRig(worldUp);
    Node aimNode(AimConstraint(),
                 {&AimConstraintIn(), &AimConstraintTarget(),
                  &AimConstraintWeight(), &AimConstraintWorldUp()},
                 AimConstraintOut(), &aim);
    aimParams.worldUpDirection = worldUp;
    CHECK(aimNode.Run({&in, &targetValue, &weight, &worldUpValue}, &actual));
    CHECK(
        SameBits(RigExecApplyAimConstraint(input, target, aimParams), actual));
    aim.preserveInputUp = false;
    Node aimSwingNode(
        AimConstraint(),
        {&AimConstraintIn(), &AimConstraintTarget(), &AimConstraintWeight()},
        AimConstraintOut(), &aim);
    aimParams.worldUpDirection.reset();
    aimParams.preserveInputUp = false;
    CHECK(aimSwingNode.Run({&in, &targetValue, &weight}, &actual));
    CHECK(
        SameBits(RigExecApplyAimConstraint(input, target, aimParams), actual));

    // A degenerate source gives what the native kernel gives.
    rigExecRig::PointFrameArray broken = sources;
    broken.frames[0].flags |= rigExecRig::kPointFrameDegenerate;
    std::vector<RigExecConstraintSource> brokenSources = gfSources;
    brokenSources[0].frame.flags |= RigExecPointFrameDegenerate;
    positionParams.weight = weight;
    CHECK(positionNode.Run({&in, &broken, &weights, &weight}, &actual));
    CHECK(SameBits(
        RigExecApplyPositionConstraint(input, brokenSources, positionParams),
        actual));

    // Weights of another length, or offsets, fail the evaluation that binds
    // them, before the node runs.
    const auto shortWeights = ToArray(std::vector<double>{1.0});
    std::string error;
    CHECK(!positionNode.Run({&in, &sources, &shortWeights, &weight}, &actual,
                            &error));
    CHECK(Contains(error, "(rig:PositionConstraint) is invalid: "
                          "PositionConstraint: needs one source weight per "
                          "source"));
    parent.translationOffsets.pop_back();
    Node shortParent(ParentConstraint(),
                     {&ParentConstraintIn(), &ParentConstraintSources(),
                      &ParentConstraintWeight()},
                     ParentConstraintOut(), &parent);
    CHECK(!shortParent.Run({&in, &sources, &weight}, &actual, &error));
    CHECK(Contains(error, "(rig:ParentConstraint) is invalid: "
                          "ParentConstraint: needs no offsets or one per "
                          "source"));
}

/// The native field for \p points under \p placement, divided by \p scale,
/// with the range policy applied; false when the policy refuses.
bool
NativeWeights(
    const std::vector<GfVec3f> &points, const GfMatrix4d &placement,
    const GfVec3f *scale, bool clamp,
    const std::function<void(const GfMatrix4d &, std::vector<float> *)> &field,
    std::vector<float> *out)
{
    GfMatrix4d worldToLocal = placement.RemoveScaleShear().GetInverse();
    if (scale) {
        GfMatrix4d divide(1.0);
        divide.SetScale(GfVec3d(1.0 / double((*scale)[0]),
                                1.0 / double((*scale)[1]),
                                1.0 / double((*scale)[2])));
        worldToLocal = worldToLocal * divide;
    }
    field(worldToLocal, out);
    for (float &w : *out)
        if (!RigExecApplyWeightRangeKernel(clamp, &w))
            return false;
    return true;
}

bool
SameBits(const std::vector<float> &expected, const RigExecArray<float> &actual)
{
    return expected.size() == actual.Size() &&
           (expected.empty() ||
            std::memcmp(expected.data(), actual.Data(),
                        expected.size() * sizeof(float)) == 0);
}

void
TestWeightFields()
{
    using namespace rigExecOps;
    const Grid grid(9, 41);
    std::vector<GfVec3f> gfPoints = grid.points;
    for (GfVec3f &p : gfPoints)
        p -= GfVec3f(4.0f, 4.0f, 0.0f);
    const Points points = ToStd(gfPoints);
    GfMatrix4d placement(1.0);
    placement.SetScale(GfVec3d(1.7, 0.6, 2.2));
    GfMatrix4d turn(1.0);
    turn.SetRotate(GfRotation(GfVec3d(0.4, 1.0, -0.3), 52.0));
    turn.SetTranslateOnly(GfVec3d(0.5, -0.75, 0.3));
    placement = placement * turn;
    const rigExecStd::Mat4d placementStd = ToRig(placement);
    const std::vector<float> lut = {0.0f, 0.1f, 0.45f, 0.9f, 1.0f};
    RigExecFalloffParams params;
    params.falloffMin = 0.5f;
    params.falloffMax = 3.25f;
    params.invert = 0.25f;
    params.strength = 0.9f;
    params.curve = lut;
    const float fMin = params.falloffMin, fMax = params.falloffMax,
                invert = params.invert, strength = params.strength;
    RigExecArray<float> actual;
    std::vector<float> expected;
    std::string error;

    // Sphere: scaled, signed scales, payload falloff curve, clamped.
    WeightFieldPayload clamped;
    clamped.falloffCurve = lut;
    clamped.clamp = true;
    const GfVec3f scale(1.5f, 0.8f, 2.0f), positive(1.0f, 2.0f, 0.5f),
        negative(0.75f, 1.0f, 3.0f);
    const Vec3f scaleStd(1.5f, 0.8f, 2.0f), positiveStd(1.0f, 2.0f, 0.5f),
        negativeStd(0.75f, 1.0f, 3.0f);
    Node sphere(SphereWeight(),
                {&SphereWeightPoints(), &SphereWeightPlacement(),
                 &SphereWeightStrength(), &SphereWeightInvert(),
                 &SphereWeightFalloffMin(), &SphereWeightFalloffMax(),
                 &SphereWeightScale(), &SphereWeightPositiveScale(),
                 &SphereWeightNegativeScale()},
                SphereWeightOut(), &clamped);
    const auto sphereField = [&](const GfMatrix4d &m, std::vector<float> *w) {
        RigExecSphereWeightField(gfPoints, m, params, w, positive, negative);
    };
    CHECK(NativeWeights(gfPoints, placement, &scale, true, sphereField,
                        &expected));
    CHECK(sphere.Run({&points, &placementStd, &strength, &invert, &fMin, &fMax,
                      &scaleStd, &positiveStd, &negativeStd},
                     &actual));
    CHECK(SameBits(expected, actual));

    // Strict: a strength past 1 refuses; a bad scale refuses.
    WeightFieldPayload strict = clamped;
    strict.clamp = false;
    Node strictSphere(SphereWeight(),
                      {&SphereWeightPoints(), &SphereWeightPlacement(),
                       &SphereWeightStrength(), &SphereWeightFalloffMax(),
                       &SphereWeightScale()},
                      SphereWeightOut(), &strict);
    const float loud = 2.0f, unit = 1.0f, wide = 10.0f;
    const Vec3f badScale(1.0f, 0.0f, 1.0f);
    CHECK(!strictSphere.Run({&points, &placementStd, &loud, &wide, &scaleStd},
                            &actual, &error));
    CHECK(strictSphere.Run({&points, &placementStd, &unit, &wide, &scaleStd},
                           &actual, &error));
    CHECK(!strictSphere.Run({&points, &placementStd, &unit, &wide, &badScale},
                            &actual, &error));
    const rigExecStd::Mat4d flat = ToRig(GfMatrix4d(0.0));
    CHECK(!strictSphere.Run({&points, &flat, &unit, &wide, &scaleStd}, &actual,
                            &error));

    // Plane: bounded along Y, and unbounded along X with fed samples.
    WeightFieldPayload bounded = clamped;
    bounded.planeAxis = WeightFieldAxis::Y;
    bounded.planeBounded = true;
    Node plane(PlaneWeight(),
               {&PlaneWeightPoints(), &PlaneWeightPlacement(),
                &PlaneWeightStrength(), &PlaneWeightInvert(),
                &PlaneWeightFalloffMin(), &PlaneWeightFalloffMax(),
                &PlaneWeightExtentU(), &PlaneWeightExtentV()},
               PlaneWeightOut(), &bounded);
    const float extentU = 2.5f, extentV = 1.75f;
    const RigExecPlaneBounds bounds{extentU, extentV};
    CHECK(NativeWeights(
        gfPoints, placement, nullptr, true,
        [&](const GfMatrix4d &m, std::vector<float> *w) {
            RigExecPlaneWeightField(gfPoints, m, 1, params, w, &bounds);
        },
        &expected));
    CHECK(plane.Run({&points, &placementStd, &strength, &invert, &fMin, &fMax,
                     &extentU, &extentV},
                    &actual));
    CHECK(SameBits(expected, actual));
    const float zero = 0.0f;
    CHECK(!plane.Run({&points, &placementStd, &strength, &invert, &fMin, &fMax,
                      &zero, &extentV},
                     &actual, &error));

    WeightFieldPayload open;
    open.clamp = true;
    open.planeAxis = WeightFieldAxis::X;
    Node openPlane(PlaneWeight(),
                   {&PlaneWeightPoints(), &PlaneWeightPlacement(),
                    &PlaneWeightFalloffMin(), &PlaneWeightFalloffMax(),
                    &PlaneWeightFalloffSamples()},
                   PlaneWeightOut(), &open);
    const std::vector<float> fed = {1.0f, 0.2f, 0.0f};
    const auto fedArray = ToArray(fed);
    RigExecFalloffParams fedParams;
    fedParams.falloffMin = -1.0f;
    fedParams.falloffMax = 1.0f;
    fedParams.curve = fed;
    const float lo = -1.0f, hi = 1.0f;
    CHECK(NativeWeights(
        gfPoints, placement, nullptr, true,
        [&](const GfMatrix4d &m, std::vector<float> *w) {
            RigExecPlaneWeightField(gfPoints, m, 0, fedParams, w);
        },
        &expected));
    CHECK(
        openPlane.Run({&points, &placementStd, &lo, &hi, &fedArray}, &actual));
    CHECK(SameBits(expected, actual));

    // Curve: a short polyline (scan) and a long one (the BVH path).
    Node curve(CurveWeight(),
               {&CurveWeightPoints(), &CurveWeightPlacement(),
                &CurveWeightStrength(), &CurveWeightInvert(),
                &CurveWeightFalloffMin(), &CurveWeightFalloffMax(),
                &CurveWeightScale(), &CurveWeightCurve()},
               CurveWeightOut(), &clamped);
    for (size_t count : {size_t(5), size_t(48)}) {
        std::vector<GfVec3f> gfCurve;
        for (size_t k = 0; k < count; ++k) {
            const float t = float(k) / float(count - 1);
            gfCurve.emplace_back(-4.0f + 8.0f * t, std::sin(6.0f * t) * 2.0f,
                                 0.5f * std::cos(3.0f * t));
        }
        const Points curvePoints = ToStd(gfCurve);
        CHECK(NativeWeights(
            gfPoints, placement, &scale, true,
            [&](const GfMatrix4d &m, std::vector<float> *w) {
                RigExecCurveWeightField(gfPoints, gfCurve, m, params, w);
            },
            &expected));
        CHECK(curve.Run({&points, &placementStd, &strength, &invert, &fMin,
                         &fMax, &scaleStd, &curvePoints},
                        &actual));
        CHECK(SameBits(expected, actual));
    }
    // Above the BVH thresholds, a steady curve allocates nothing.
    std::vector<GfVec3f> longCurve;
    for (size_t k = 0; k < 48; ++k)
        longCurve.emplace_back(-4.0f + float(k) / 6.0f, 0.25f * float(k % 3),
                               0.0f);
    const Points longCurvePoints = ToStd(longCurve);
    float steadyStrength = strength;
    CHECK(opAllocationCount::SteadyFrameAllocations(
              CurveWeight(), &clamped,
              {&CurveWeightPoints(), &CurveWeightPlacement(),
               &CurveWeightStrength(), &CurveWeightInvert(),
               &CurveWeightFalloffMin(), &CurveWeightFalloffMax(),
               &CurveWeightScale(), &CurveWeightCurve()},
              {&points, &placementStd, &strength, &invert, &fMin, &fMax,
               &scaleStd, &longCurvePoints},
              CurveWeightOut(), 2,
              [&](int frame) {
                  steadyStrength = 0.5f + 0.05f * float(frame);
                  return &steadyStrength;
              },
              &error) == 0);

    // A sparse dynamic field over its base's support allocates nothing on
    // a steady frame.
    const WeightPacketPayload dense{rigExecRig::WeightRepresentation::Dense,
                                    false};
    const auto dynamicBase = ToArray(std::vector<float>{0.5f, 0.0f, 0.25f});
    const auto own = ToArray(std::vector<int32_t>{0, 2});
    const auto baseOwn = ToArray(std::vector<int32_t>{2, 0});
    const float scaleOne = 1.0f, biasZero = 0.0f;
    float driver = 1.0f;
    CHECK(opAllocationCount::SteadyFrameAllocations(
              DynamicWeightField(), &dense,
              {&DynamicWeightFieldBase(), &DynamicWeightFieldDriver(),
               &DynamicWeightFieldScale(), &DynamicWeightFieldBias(),
               &DynamicWeightFieldIndices(), &DynamicWeightFieldBaseIndices()},
              {&dynamicBase, &driver, &scaleOne, &biasZero, &own, &baseOwn},
              DynamicWeightFieldOut(), 1,
              [&](int frame) {
                  driver = 0.5f + 0.1f * float(frame);
                  return &driver;
              },
              &error) == 0);

    const Points noCurve;
    CHECK(!curve.Run({&points, &placementStd, &strength, &invert, &fMin, &fMax,
                      &scaleStd, &noCurve},
                     &actual, &error));
}

rigExecRig::PointLandmarks
ToRigLandmarks(const std::array<GfVec3d, 4> &p)
{
    rigExecRig::PointLandmarks out;
    for (int i = 0; i < 4; ++i)
        out[i] = rigExecStd::Vec3d(p[i][0], p[i][1], p[i][2]);
    return out;
}

void
TestChainSolvers()
{
    using namespace rigExecOps;
    // A three-control chain, the second a child of the first and the third
    // of the second, posed and rested apart.
    std::vector<RigExecFkChainElement> elements(3);
    rigExecRig::PointFrameArray controls;
    controls.frames = RigExecArray<rigExecRig::PointFrame>(3);
    controls.rests = RigExecArray<rigExecRig::PointLandmarks>(3);
    RigExecArray<rigExecRig::PointLandmarks> outRests(3);
    for (size_t i = 0; i < 3; ++i) {
        const double x = 2.0 * double(i);
        const RigExecPointFrame rest =
            MakeFrame(GfVec3d(x, 0.1 * x, 0), GfVec3d(0, 0, 1),
                      10.0 * double(i), GfVec3d(1.0));
        const RigExecPointFrame pose =
            MakeFrame(GfVec3d(x + 0.3, 0.5, -0.2 * x), GfVec3d(0.3, 1, 0.2),
                      25.0 + 15.0 * double(i), GfVec3d(1.0, 1.1, 0.9));
        const RigExecPointFrame outRest = MakeFrame(
            GfVec3d(x, 0.0, 0.05), GfVec3d(0, 1, 0), 5.0, GfVec3d(1.0));
        elements[i].restPoints = rest.points;
        elements[i].posePoints = pose.points;
        elements[i].outRestPoints = outRest.points;
        elements[i].parentIndex = int(i) - 1;
        controls.frames[i] = ToRig(pose);
        controls.rests[i] = ToRigLandmarks(rest.points);
        outRests[i] = ToRigLandmarks(outRest.points);
    }
    FkChainPayload fk;
    fk.parentIndices = {-1, 0, 1};
    rigExecRig::PointFrameArray actual;
    for (const bool scale : {false, true}) {
        fk.scaleSegments = scale;
        Node bare(FkChain(), {&FkChainControls()}, FkChainOut(), &fk);
        std::vector<RigExecPointFrame> expected = RigExecSolveFkChain(elements);
        if (scale)
            RigExecScaleFkSegments(elements, &expected);
        CHECK(bare.Run({&controls}, &actual));
        CHECK(actual.frames.Size() == 3);
        for (size_t i = 0; i < 3 && i < actual.frames.Size(); ++i)
            CHECK(SameBits(expected[i], actual.frames[i]));

        auto withOut = elements;
        for (auto &e : withOut)
            e.hasOutRest = true;
        expected = RigExecSolveFkChain(withOut);
        if (scale)
            RigExecScaleFkSegments(withOut, &expected);
        Node rebased(FkChain(), {&FkChainControls(), &FkChainOutRests()},
                     FkChainOut(), &fk);
        CHECK(rebased.Run({&controls, &outRests}, &actual));
        for (size_t i = 0; i < 3 && i < actual.frames.Size(); ++i)
            CHECK(SameBits(expected[i], actual.frames[i]));
    }
    FkChainPayload loop;
    loop.parentIndices = {-1, 2, 1};
    Node badParents(FkChain(), {&FkChainControls()}, FkChainOut(), &loop);
    std::string error;
    CHECK(!badParents.Run({&controls}, &actual, &error));

    // Two-bone IK: both stretch policies, scaled segments and a space.
    std::array<std::array<GfVec3d, 4>, 3> gfRests;
    RigExecArray<rigExecRig::PointLandmarks> rests(3);
    for (size_t i = 0; i < 3; ++i) {
        gfRests[i] = MakeFrame(GfVec3d(0, -2.0 * double(i), 0.1 * double(i)),
                               GfVec3d(0, 0, 1), -90.0, GfVec3d(1.0))
                         .points;
        rests[i] = ToRigLandmarks(gfRests[i]);
    }
    const RigExecPointFrame root = MakeFrame(
        GfVec3d(0.2, 0.0, 0.1), GfVec3d(0, 0, 1), -80.0, GfVec3d(1.0));
    const RigExecPointFrame pole =
        MakeFrame(GfVec3d(0.5, -2.0, 3.0), GfVec3d(1, 0, 0), 0.0, GfVec3d(1.0));
    GfMatrix4d gfSpace(1.0);
    gfSpace.SetScale(GfVec3d(1.3));
    const rigExecStd::Mat4d space = ToRig(gfSpace);
    const double bend = 0.4, upperOffset = 0.1, lowerOffset = -0.05,
                 upperScale = 1.1, lowerScale = 0.95;
    const float stretch = 0.7f, softness = 0.15f, pin = 0.2f,
                softDistance = 0.3f, twist = 12.0f;
    for (const double reach : {2.5, 6.5}) {
        const RigExecPointFrame effector = MakeFrame(
            GfVec3d(0.4, -reach, 0.8), GfVec3d(1, 0, 0), 30.0, GfVec3d(1.0));
        for (const bool softPolicy : {false, true}) {
            TwoBoneIkPayload ik;
            ik.softDistancePolicy = softPolicy;
            ik.scaleSegments = softPolicy;
            ik.scaleCalibration = softPolicy ? 0.05 : 0.0;
            RigExecTwoBoneIkParams params;
            params.preferredBendRadians = bend;
            params.stretch = stretch;
            params.softness = softness;
            params.space = gfSpace;
            params.softDistancePolicy = softPolicy;
            params.scaleSegments = softPolicy;
            params.limb.scaleCalibration = ik.scaleCalibration;
            RigExecTwoBoneIkLengths(gfRests, gfSpace, upperOffset, lowerOffset,
                                    &params.upperLength, &params.lowerLength);
            RigExecSetTwoBoneLimbParams(gfRests, gfSpace, stretch, pin,
                                        upperScale, lowerScale, softDistance,
                                        twist, &params);
            const auto expected =
                RigExecSolveTwoBoneIk(root, effector, pole, gfRests, params);
            Node node(TwoBoneIk(),
                      {&TwoBoneIkRoot(), &TwoBoneIkEffector(), &TwoBoneIkPole(),
                       &TwoBoneIkRests(), &TwoBoneIkSpace(), &TwoBoneIkBend(),
                       &TwoBoneIkUpperOffset(), &TwoBoneIkLowerOffset(),
                       &TwoBoneIkStretch(), &TwoBoneIkSoftness(),
                       &TwoBoneIkPin(), &TwoBoneIkUpperScale(),
                       &TwoBoneIkLowerScale(), &TwoBoneIkSoftDistance(),
                       &TwoBoneIkTwist()},
                      TwoBoneIkOut(), &ik);
            const rigExecRig::PointFrame r = ToRig(root), e = ToRig(effector),
                                         p = ToRig(pole);
            CHECK(node.Run({&r, &e, &p, &rests, &space, &bend, &upperOffset,
                            &lowerOffset, &stretch, &softness, &pin,
                            &upperScale, &lowerScale, &softDistance, &twist},
                           &actual));
            CHECK(actual.frames.Size() == 3);
            for (size_t i = 0; i < 3 && i < actual.frames.Size(); ++i)
                CHECK(SameBits(expected[i], actual.frames[i]));
        }
    }
    // Defaults match the native defaults; two rests refuse.
    {
        const RigExecPointFrame effector = MakeFrame(
            GfVec3d(0.4, -3.0, 0.8), GfVec3d(1, 0, 0), 30.0, GfVec3d(1.0));
        RigExecTwoBoneIkParams params;
        RigExecTwoBoneIkLengths(gfRests, GfMatrix4d(1.0), 0.0, 0.0,
                                &params.upperLength, &params.lowerLength);
        RigExecSetTwoBoneLimbParams(gfRests, GfMatrix4d(1.0), 1.0, 0.0, 1.0,
                                    1.0, 0.0, 0.0, &params);
        const auto expected =
            RigExecSolveTwoBoneIk(root, effector, pole, gfRests, params);
        TwoBoneIkPayload ik;
        Node node(TwoBoneIk(),
                  {&TwoBoneIkRoot(), &TwoBoneIkEffector(), &TwoBoneIkPole(),
                   &TwoBoneIkRests()},
                  TwoBoneIkOut(), &ik);
        const rigExecRig::PointFrame r = ToRig(root), e = ToRig(effector),
                                     p = ToRig(pole);
        CHECK(node.Run({&r, &e, &p, &rests}, &actual));
        for (size_t i = 0; i < 3 && i < actual.frames.Size(); ++i)
            CHECK(SameBits(expected[i], actual.frames[i]));
        RigExecArray<rigExecRig::PointLandmarks> two(2);
        CHECK(!node.Run({&r, &e, &p, &two}, &actual, &error));
    }
}

bool
SameBits(const std::array<GfVec3d, 4> &expected,
         const rigExecRig::PointLandmarks &actual)
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 3; ++j) {
            const double e = expected[i][j], v = actual[i][j];
            if (std::memcmp(&e, &v, sizeof(double)) != 0)
                return false;
        }
    return true;
}

void
TestFrameBlend()
{
    using namespace rigExecOps;
    const RigExecPointFrame restFrame =
        MakeFrame(GfVec3d(0.5, 0, 0), GfVec3d(0, 0, 1), 10.0, GfVec3d(1.0));
    const RigExecPointFrame outRestFrame =
        MakeFrame(GfVec3d(0, 1, 0), GfVec3d(1, 0, 0), 20.0, GfVec3d(1.0));
    const RigExecPointFrame a = MakeFrame(
        GfVec3d(1, 2, 3), GfVec3d(0.2, 1, 0.1), 40.0, GfVec3d(1.2, 0.8, 1.0));
    const RigExecPointFrame b =
        MakeFrame(GfVec3d(-1, 0.5, 2), GfVec3d(1, 0.3, -0.4), 200.0,
                  GfVec3d(0.7, 1.5, 2));
    RigExecPointFrame flat = a;
    flat.points[3] = flat.points[0];
    const rigExecRig::PointFrame rigA = ToRig(a), rigB = ToRig(b),
                                 rigFlat = ToRig(flat);
    const rigExecRig::PointLandmarks rest = ToRigLandmarks(restFrame.points),
                                     outRest =
                                         ToRigLandmarks(outRestFrame.points);
    rigExecRig::PointFrame actual;
    for (const bool logScale : {false, true}) {
        BlendFramesPayload payload;
        payload.logScale = logScale;
        const RigExecScaleBlend scaleBlend =
            logScale ? RigExecScaleBlend::Log : RigExecScaleBlend::Linear;
        Node bare(BlendFrames(),
                  {&BlendFramesA(), &BlendFramesB(), &BlendFramesRest(),
                   &BlendFramesWeight()},
                  BlendFramesOut(), &payload);
        Node rebased(BlendFrames(),
                     {&BlendFramesA(), &BlendFramesB(), &BlendFramesRest(),
                      &BlendFramesWeight(), &BlendFramesOutRest()},
                     BlendFramesOut(), &payload);
        for (const double w : {-0.5, 0.0, 0.3, 0.75, 1.0, 2.0}) {
            CHECK(bare.Run({&rigA, &rigB, &rest, &w}, &actual));
            CHECK(SameBits(RigExecBlendFrames(a, b, restFrame.points, w,
                                              RigExecRotationBlend::ShortestArc,
                                              scaleBlend),
                           actual));
            CHECK(rebased.Run({&rigA, &rigB, &rest, &w, &outRest}, &actual));
            CHECK(SameBits(RigExecBlendFrames(a, b, restFrame.points, w,
                                              RigExecRotationBlend::ShortestArc,
                                              scaleBlend, &outRestFrame.points),
                           actual));
            // A singular input holds the stronger one, flagged.
            CHECK(bare.Run({&rigA, &rigFlat, &rest, &w}, &actual));
            CHECK(SameBits(RigExecBlendFrames(a, flat, restFrame.points, w,
                                              RigExecRotationBlend::ShortestArc,
                                              scaleBlend),
                           actual));
        }
        // A NaN weight blends as the native kernel does.
        const double nan = std::nan("");
        CHECK(bare.Run({&rigA, &rigB, &rest, &nan}, &actual));
        CHECK(SameBits(RigExecBlendFrames(a, b, restFrame.points, nan,
                                          RigExecRotationBlend::ShortestArc,
                                          scaleBlend),
                       actual));
    }

    // Twist distribution: a twist past a half turn, extra turns, an
    // unfed turn count and a singular end.
    const RigExecPointFrame endRestFrame =
        MakeFrame(GfVec3d(0, 0, 4), GfVec3d(0, 1, 0), 0.0, GfVec3d(1.0));
    const rigExecRig::PointLandmarks endRest =
        ToRigLandmarks(endRestFrame.points);
    const RigExecPointFrame start = MakeFrame(
        GfVec3d(0.1, 0.2, 0.3), GfVec3d(0, 0.2, 1), 30.0, GfVec3d(1.0));
    const RigExecPointFrame end = MakeFrame(
        GfVec3d(0.3, 0.1, 4.2), GfVec3d(0.1, 0.1, 1), 250.0, GfVec3d(1.3));
    const std::vector<double> weights = {-0.2, 0.0, 0.25, 0.5, 0.9, 1.0, 1.4};
    const auto weightArray =
        RigExecArray<double>(weights.data(), weights.size());
    const rigExecRig::PointFrame rigStart = ToRig(start), rigEnd = ToRig(end);
    rigExecRig::PointFrameArray frames;
    Node bareTwist(DistributeTwist(),
                   {&DistributeTwistStart(), &DistributeTwistEnd(),
                    &DistributeTwistStartRest(), &DistributeTwistEndRest(),
                    &DistributeTwistWeights()},
                   DistributeTwistOut(), nullptr);
    Node turnsTwist(DistributeTwist(),
                    {&DistributeTwistStart(), &DistributeTwistEnd(),
                     &DistributeTwistStartRest(), &DistributeTwistEndRest(),
                     &DistributeTwistWeights(), &DistributeTwistTurns()},
                    DistributeTwistOut(), nullptr);
    auto check = [&](const std::vector<RigExecPointFrame> &expected) {
        CHECK(frames.frames.Size() == expected.size());
        CHECK(frames.rests.Size() == expected.size());
        for (size_t k = 0; k < expected.size() && k < frames.frames.Size();
             ++k) {
            CHECK(SameBits(expected[k], frames.frames[k]));
            CHECK(SameBits(restFrame.points, frames.rests[k]));
        }
    };
    CHECK(bareTwist.Run({&rigStart, &rigEnd, &rest, &endRest, &weightArray},
                        &frames));
    check(RigExecDistributeTwist(start, end, restFrame.points,
                                 endRestFrame.points, weights, 0.0));
    for (const double turns : {-1.0, 2.0}) {
        CHECK(turnsTwist.Run(
            {&rigStart, &rigEnd, &rest, &endRest, &weightArray, &turns},
            &frames));
        check(RigExecDistributeTwist(start, end, restFrame.points,
                                     endRestFrame.points, weights, turns));
    }
    RigExecPointFrame flatEnd = end;
    flatEnd.points[2] = flatEnd.points[0];
    const rigExecRig::PointFrame rigFlatEnd = ToRig(flatEnd);
    CHECK(bareTwist.Run({&rigStart, &rigFlatEnd, &rest, &endRest, &weightArray},
                        &frames));
    check(RigExecDistributeTwist(start, flatEnd, restFrame.points,
                                 endRestFrame.points, weights, 0.0));
    const double inf = std::numeric_limits<double>::infinity();
    std::string error;
    CHECK(!turnsTwist.Run(
        {&rigStart, &rigEnd, &rest, &endRest, &weightArray, &inf}, &frames,
        &error));
}

void
TestSplineIk()
{
    using namespace rigExecOps;
    // A six-joint spine curving up and forward, with rest controls at its
    // root, middle and end.
    std::vector<RigExecPointFrame> joints;
    RigExecArray<rigExecRig::PointFrame> restJoints(6);
    for (size_t i = 0; i < 6; ++i) {
        const double t = double(i);
        joints.push_back(MakeFrame(GfVec3d(0.1 * t * t, t, 0.05 * t),
                                   GfVec3d(1, 0, 0), -90.0 + 4.0 * t,
                                   GfVec3d(1.0)));
        restJoints[i] = ToRig(joints.back());
    }
    const RigExecPointFrame restRoot = joints[0];
    const RigExecPointFrame restMid = MakeFrame(
        GfVec3d(0.6, 2.5, 0.1), GfVec3d(1, 0, 0), -80.0, GfVec3d(1.0));
    const RigExecPointFrame restEnd = joints[5];
    const RigExecPointFrame root = MakeFrame(
        GfVec3d(0.1, -0.2, 0.0), GfVec3d(0.2, 0.1, 1), -70.0, GfVec3d(1.0));
    const RigExecPointFrame mid = MakeFrame(
        GfVec3d(1.4, 2.2, 0.6), GfVec3d(1, 0.1, 0), -60.0, GfVec3d(1.0));
    const std::vector<double> volume = {0.0, 0.2, 0.6, 1.0, 0.6, 0.2};
    const auto volumeArray = RigExecArray<double>(volume.data(), volume.size());
    const double preserve = 0.8, follow = 0.35, roll = 0.2, twist = -0.4;
    const rigExecRig::PointFrame rigRestRoot = ToRig(restRoot),
                                 rigRestMid = ToRig(restMid),
                                 rigRestEnd = ToRig(restEnd),
                                 rigRoot = ToRig(root), rigMid = ToRig(mid);
    rigExecRig::PointFrameArray actual;
    // A stretched end, and one driven back past the root (the floor).
    for (const double reach : {6.5, -1.0}) {
        const RigExecPointFrame end = MakeFrame(
            GfVec3d(2.0, reach, 1.0), GfVec3d(0, 1, 0.3), 30.0, GfVec3d(1.0));
        const rigExecRig::PointFrame rigEnd = ToRig(end);
        for (const bool variant : {false, true}) {
            SplineIkPayload payload;
            payload.restLengthChain = variant;
            payload.minLengthRatio = variant ? 0.6 : 0.0;
            payload.aimRootTangent = variant;
            const RigExecSplineIkRest rest = RigExecSplineIkMakeRest(
                joints, restRoot, restMid, restEnd, volume,
                variant ? RigExecSplineIkRestLength::Chain
                        : RigExecSplineIkRestLength::Curve);
            RigExecSplineIkParams params;
            params.preserveVolume = preserve;
            params.midFollowWeight = follow;
            params.roll = roll;
            params.twist = twist;
            params.minLengthRatio = payload.minLengthRatio;
            params.aimRootTangent = payload.aimRootTangent;
            RigExecSplineIkResult expected;
            RigExecSolveSplineIk(rest, {root, mid, end}, params, &expected);
            Node full(SplineIk(),
                      {&SplineIkRestJoints(), &SplineIkRestRoot(),
                       &SplineIkRestMid(), &SplineIkRestEnd(), &SplineIkRoot(),
                       &SplineIkMid(), &SplineIkEnd(), &SplineIkVolumeWeights(),
                       &SplineIkPreserveVolume(), &SplineIkMidFollow(),
                       &SplineIkRoll(), &SplineIkTwist()},
                      SplineIkOut(), &payload);
            CHECK(full.Run({&restJoints, &rigRestRoot, &rigRestMid, &rigRestEnd,
                            &rigRoot, &rigMid, &rigEnd, &volumeArray, &preserve,
                            &follow, &roll, &twist},
                           &actual));
            CHECK(actual.frames.Size() == 6 && actual.rests.Size() == 6);
            for (size_t i = 0; i < 6 && i < actual.frames.Size(); ++i) {
                CHECK(SameBits(expected.joints[i].frame, actual.frames[i]));
                CHECK(SameBits(joints[i].points, actual.rests[i]));
            }

            // Defaults: no volume weights, follow 0.5, no roll or twist.
            const RigExecSplineIkRest bareRest = RigExecSplineIkMakeRest(
                joints, restRoot, restMid, restEnd, {},
                variant ? RigExecSplineIkRestLength::Chain
                        : RigExecSplineIkRestLength::Curve);
            RigExecSplineIkParams bareParams;
            bareParams.minLengthRatio = payload.minLengthRatio;
            bareParams.aimRootTangent = payload.aimRootTangent;
            RigExecSolveSplineIk(bareRest, {root, mid, end}, bareParams,
                                 &expected);
            Node bare(SplineIk(),
                      {&SplineIkRestJoints(), &SplineIkRestRoot(),
                       &SplineIkRestMid(), &SplineIkRestEnd(), &SplineIkRoot(),
                       &SplineIkMid(), &SplineIkEnd()},
                      SplineIkOut(), &payload);
            CHECK(bare.Run({&restJoints, &rigRestRoot, &rigRestMid, &rigRestEnd,
                            &rigRoot, &rigMid, &rigEnd},
                           &actual));
            for (size_t i = 0; i < 6 && i < actual.frames.Size(); ++i)
                CHECK(SameBits(expected.joints[i].frame, actual.frames[i]));
        }
    }

    // A singular rest root holds every joint at rest, flagged.
    RigExecPointFrame flatRoot = restRoot;
    flatRoot.points[2] = flatRoot.points[0];
    const rigExecRig::PointFrame rigFlatRoot = ToRig(flatRoot);
    const RigExecSplineIkRest flatRest =
        RigExecSplineIkMakeRest(joints, flatRoot, restMid, restEnd);
    RigExecSplineIkResult expected;
    CHECK(!RigExecSolveSplineIk(flatRest, {root, mid, root},
                                RigExecSplineIkParams(), &expected));
    SplineIkPayload payload;
    Node bare(SplineIk(),
              {&SplineIkRestJoints(), &SplineIkRestRoot(), &SplineIkRestMid(),
               &SplineIkRestEnd(), &SplineIkRoot(), &SplineIkMid(),
               &SplineIkEnd()},
              SplineIkOut(), &payload);
    CHECK(bare.Run({&restJoints, &rigFlatRoot, &rigRestMid, &rigRestEnd,
                    &rigRoot, &rigMid, &rigRoot},
                   &actual));
    for (size_t i = 0; i < 6 && i < actual.frames.Size(); ++i)
        CHECK(SameBits(expected.joints[i].frame, actual.frames[i]));

    // Volume weights of the wrong length and an empty chain refuse.
    const std::vector<double> short3 = {0.1, 0.2, 0.3};
    const auto shortArray = RigExecArray<double>(short3.data(), short3.size());
    Node weighted(SplineIk(),
                  {&SplineIkRestJoints(), &SplineIkRestRoot(),
                   &SplineIkRestMid(), &SplineIkRestEnd(), &SplineIkRoot(),
                   &SplineIkMid(), &SplineIkEnd(), &SplineIkVolumeWeights()},
                  SplineIkOut(), &payload);
    std::string error;
    CHECK(!weighted.Run({&restJoints, &rigRestRoot, &rigRestMid, &rigRestEnd,
                         &rigRoot, &rigMid, &rigRoot, &shortArray},
                        &actual, &error));
    const RigExecArray<rigExecRig::PointFrame> none;
    CHECK(!bare.Run({&none, &rigRestRoot, &rigRestMid, &rigRestEnd, &rigRoot,
                     &rigMid, &rigRoot},
                    &actual, &error));
}

void
TestRibbonFrames()
{
    using namespace rigExecOps;
    using RrVec = rigExecStd::Vec3f;
    // The frame points as the samples give them: offsets summed in float.
    const auto landmarks = [](const RigExecCurveFrameSamples &f, size_t k) {
        const GfVec3f o = f.positions[k];
        const GfVec3f p[4] = {o, o + f.tangents[k], o + f.normals[k],
                              o + f.binormals[k]};
        std::array<GfVec3d, 4> l;
        for (int j = 0; j < 4; ++j)
            l[j] = GfVec3d(p[j]);
        return l;
    };
    const auto sameFloats = [](const std::vector<GfVec3f> &a,
                               const std::vector<RrVec> &b) {
        if (a.size() != b.size())
            return false;
        for (size_t i = 0; i < a.size(); ++i)
            for (size_t j = 0; j < 3; ++j) {
                const float x = a[i][j], y = b[i][j];
                if (std::memcmp(&x, &y, sizeof(float)) != 0)
                    return false;
            }
        return true;
    };
    // A B-spline (six points) and a short polygon (three).
    for (const int count : {6, 3}) {
        std::vector<GfVec3f> restCurve, posedCurve;
        for (int i = 0; i < count; ++i) {
            restCurve.emplace_back(1.4f * i, 0.5f * i, 0.3f * (i % 2));
            posedCurve.emplace_back(1.3f * i, 0.5f * i + 0.2f * i * i,
                                    0.4f * i - 0.3f * (i % 2));
        }
        const Points restPts = ToStd(restCurve), posedPts = ToStd(posedCurve);
        for (const int sampleCount : {2, 9}) {
            const RigExecCurveFrameSamples rest =
                RigExecSampleCurveRMF(restCurve, sampleCount);
            const RigExecCurveFrameSamples posed =
                RigExecSampleCurveRMF(posedCurve, sampleCount);
            CHECK(posed.GetSize() == size_t(sampleCount));

            // The runtime (Rr) instantiation matches the Gf one bit for bit.
            const auto rr = rigExec::RigExecSampleCurveRMFKernel(
                posedPts.Data(), posedPts.Size(), sampleCount);
            CHECK(sameFloats(posed.positions, rr.positions));
            CHECK(sameFloats(posed.tangents, rr.tangents));
            CHECK(sameFloats(posed.normals, rr.normals));
            CHECK(sameFloats(posed.binormals, rr.binormals));

            SampleRibbonFramesPayload payload;
            payload.sampleCount = sampleCount;
            Node node(SampleRibbonFrames(),
                      {&SampleRibbonFramesPosed(), &SampleRibbonFramesRest()},
                      SampleRibbonFramesOut(), &payload);
            rigExecRig::PointFrameArray actual;
            CHECK(node.Run({&posedPts, &restPts}, &actual));
            CHECK(actual.frames.Size() == size_t(sampleCount) &&
                  actual.rests.Size() == size_t(sampleCount));
            for (size_t k = 0; k < actual.frames.Size(); ++k) {
                CHECK(SameBits(landmarks(posed, k), actual.frames[k].points));
                CHECK(actual.frames[k].flags == rigExecRig::kPointFrameValid);
                CHECK(SameBits(landmarks(rest, k), actual.rests[k]));
            }

            // Odd samples re-base onto a joint rest, the same both ways.
            std::vector<RigExecPointFrame> gfFrames;
            std::vector<std::array<GfVec3d, 4>> gfRests, joints;
            std::vector<rigExecRig::PointFrame> rrFrames;
            std::vector<rigExecRig::PointLandmarks> rrRests, rrJoints;
            for (size_t k = 0; k < size_t(sampleCount); ++k) {
                RigExecPointFrame f;
                f.points = landmarks(posed, k);
                gfFrames.push_back(f);
                gfRests.push_back(landmarks(rest, k));
                joints.push_back(MakeFrame(GfVec3d(0.1 * k, 1.0, 0.0),
                                           GfVec3d(0, 0, 1), 15.0 * k,
                                           GfVec3d(1.0))
                                     .points);
                rrFrames.push_back(actual.frames[k]);
                rrRests.push_back(actual.rests[k]);
                rrJoints.push_back(ToRigLandmarks(joints.back()));
            }
            rigExec::RigExecRebaseFramesKernel<GfVec3f, GfVec3d>(
                [&](size_t k) { return k % 2 ? &joints[k] : nullptr; },
                &gfFrames, &gfRests);
            rigExec::RigExecRebaseFramesKernel<rigExecStd::Vec3f,
                                               rigExecStd::Vec3d>(
                [&](size_t k) { return k % 2 ? &rrJoints[k] : nullptr; },
                &rrFrames, &rrRests);
            for (size_t k = 0; k < gfFrames.size(); ++k) {
                CHECK(SameBits(gfFrames[k].points, rrFrames[k].points));
                CHECK(SameBits(gfRests[k], rrRests[k]));
                CHECK(SameBits(k % 2 ? joints[k] : landmarks(rest, k),
                               rrRests[k]));
            }
        }
    }

    // One point, one sample and a zero-length curve refuse.
    SampleRibbonFramesPayload payload;
    payload.sampleCount = 5;
    Node node(SampleRibbonFrames(),
              {&SampleRibbonFramesPosed(), &SampleRibbonFramesRest()},
              SampleRibbonFramesOut(), &payload);
    const Points curve = ToStd({GfVec3f(0, 0, 0), GfVec3f(1, 2, 0),
                                GfVec3f(2, 2, 1), GfVec3f(3, 1, 1)});
    const Points one = ToStd({GfVec3f(1, 2, 3)});
    // Two coincident points: a spline of four would sum float weights.
    const Points flat = ToStd(std::vector<GfVec3f>(2, GfVec3f(1, 2, 3)));
    rigExecRig::PointFrameArray actual;
    std::string error;
    CHECK(node.Run({&curve, &curve}, &actual));
    CHECK(!node.Run({&one, &curve}, &actual, &error));
    CHECK(!node.Run({&curve, &flat}, &actual, &error));
    payload.sampleCount = 1;
    Node single(SampleRibbonFrames(),
                {&SampleRibbonFramesPosed(), &SampleRibbonFramesRest()},
                SampleRibbonFramesOut(), &payload);
    CHECK(!single.Run({&curve, &curve}, &actual, &error));
}

// PoseCommit: two joints and three descendants, the second blocked by an
// authored parent space. Nothing present keeps every carry; a present
// candidate writes itself and carries its delta onto the descendants under
// it; a singular delta is an error; a descendant without its frames does
// not compile.
void
TestPoseCommit()
{
    using namespace rigExecOps;
    using Frames = RigExecArray<rigExecRig::PointFrame>;
    const auto frames = [](std::vector<RigExecPointFrame> in) {
        Frames out(in.size());
        for (size_t k = 0; k < in.size(); ++k)
            out[k] = ToRig(in[k]);
        return out;
    };
    const auto frame = [](double x, double degrees) {
        return MakeFrame(GfVec3d(x, 0.5, 0.0), GfVec3d(0, 0, 1), degrees,
                         GfVec3d(1.0));
    };
    const std::vector<RigExecPointFrame> entering = {frame(0, 0), frame(2, 10)};
    const std::vector<RigExecPointFrame> candidate = {frame(0.5, 30),
                                                      frame(2, 10)};
    const std::vector<RigExecPointFrame> current = {frame(1, 5), frame(3, 0),
                                                    frame(4, 20)};
    const Frames enteringIn = frames(entering), candidates = frames(candidate),
                 carry = frames({frame(-1, 0), frame(-2, 0)}),
                 currentIn = frames(current),
                 closest = frames({entering[0], entering[0], entering[1]}),
                 descendantCarry =
                     frames({frame(-3, 0), frame(-4, 0), frame(-5, 0)});
    PoseCommitPayload p;
    p.slotCount = 2;
    p.closest = {0, 0, 1};
    p.blocked = {0, 1, 0};
    const std::vector<const RigExecPortHandle *> ports = {
        &PoseCommitCandidates(),     &PoseCommitPresent(),
        &PoseCommitEntering(),       &PoseCommitCarry(),
        &PoseCommitCurrent(),        &PoseCommitClosest(),
        &PoseCommitDescendantCarry()};
    Node slots(PoseCommit(), ports, PoseCommitSlots(), &p);
    Node descendants(PoseCommit(), ports, PoseCommitDescendants(), &p);
    RigExecArray<uint8_t> none(2), first(2);
    none[0] = none[1] = 0;
    first[0] = 1;
    first[1] = 0;
    Frames actual;
    const auto values = [&](const RigExecArray<uint8_t> &present,
                            const Frames &enter) {
        return std::vector<const void *>{&candidates,     &present,   &enter,
                                         &carry,          &currentIn, &closest,
                                         &descendantCarry};
    };
    CHECK(slots.Run(values(none, enteringIn), &actual));
    CHECK(actual.Size() == 2 && SameResult(actual, carry));
    CHECK(descendants.Run(values(none, enteringIn), &actual));
    CHECK(actual.Size() == 3 && SameResult(actual, descendantCarry));

    CHECK(slots.Run(values(first, enteringIn), &actual));
    CHECK(actual.Size() == 2 && SameBits(candidate[0], actual[0]) &&
          SameBits(RigExecPointFrame(frame(-2, 0)), actual[1]));
    GfMatrix4d delta(1.0);
    CHECK(
        RigExecPointsToMatrix(entering[0].points, candidate[0].points, &delta));
    CHECK(descendants.Run(values(first, enteringIn), &actual));
    CHECK(
        actual.Size() == 3 &&
        SameBits(RigExecMatrixToPoints(current[0].points, delta), actual[0]) &&
        SameBits(frame(-4, 0), actual[1]) && SameBits(frame(-5, 0), actual[2]));

    RigExecPointFrame collapsed = entering[0];
    collapsed.points = {GfVec3d(1.0), GfVec3d(1.0), GfVec3d(1.0), GfVec3d(1.0)};
    const Frames singular = frames({collapsed, entering[1]});
    std::string error;
    CHECK(!descendants.Run(values(first, singular), &actual, &error));
    CHECK(error.find("singular hierarchy delta") != std::string::npos);

    Node unfed(
        PoseCommit(),
        {&PoseCommitCandidates(), &PoseCommitEntering(), &PoseCommitCarry()},
        PoseCommitSlots(), &p);
    CHECK(!unfed.Run({&candidates, &enteringIn, &carry}, &actual, &error));
    CHECK(error.find("per descendant") != std::string::npos);
}

void
TestSingleChainIk()
{
    using namespace rigExecOps;
    // A four-joint chain with a gentle bend and a turn at each joint.
    std::vector<RigExecPointFrame> chain;
    rigExecRig::PointFrameArray input;
    input.frames = RigExecArray<rigExecRig::PointFrame>(4);
    input.rests = RigExecArray<rigExecRig::PointLandmarks>(4);
    for (size_t i = 0; i < 4; ++i) {
        const double t = double(i);
        chain.push_back(MakeFrame(GfVec3d(t, 0.2 * t * t, 0.1 * t),
                                  GfVec3d(0, 0, 1), 10.0 * t, GfVec3d(1.0)));
        input.frames[i] = ToRig(chain.back());
        input.rests[i] = ToRigLandmarks(chain.back().points);
    }
    const GfVec3d pole(0.5, 2.0, -1.0);
    const rigExecStd::Vec3d rigPole = ToRig(pole);
    const double twist = 15.0;
    rigExecRig::PointFrameArray actual;
    const auto same = [&](const std::vector<RigExecPointFrame> &expected) {
        bool ok = expected.size() == actual.frames.Size() &&
                  actual.rests.Size() == input.rests.Size();
        for (size_t i = 0; ok && i < expected.size(); ++i)
            ok = SameBits(expected[i], actual.frames[i]) &&
                 actual.rests[i] == input.rests[i];
        return ok;
    };
    // A reachable goal and one past full reach.
    for (const GfVec3d goal :
         {GfVec3d(1.5, 1.2, 0.4), GfVec3d(9.0, 1.0, 0.0)}) {
        const RigExecPointFrame effector =
            MakeFrame(goal, GfVec3d(1, 1, 0), 25.0, GfVec3d(1.0));
        const rigExecRig::PointFrame rigEffector = ToRig(effector);
        for (const bool singleChain : {false, true})
            for (const bool preserve : {false, true})
                for (const double weight : {1.0, 0.4, 0.0})
                    for (const double stretch : {0.0, 0.5}) {
                        RigExecSingleChainIkParams params;
                        params.mode =
                            singleChain ? RigExecSingleChainIkMode::SingleChain
                                        : RigExecSingleChainIkMode::RotatePlane;
                        params.pole = pole;
                        params.twistDegrees = twist;
                        params.weight = weight;
                        params.preserveJointOrientation = preserve;
                        params.stretch = stretch;
                        SingleChainIkPayload payload;
                        payload.singleChain = singleChain;
                        payload.preserveJointOrientation = preserve;
                        Node node(
                            SingleChainIk(),
                            {&SingleChainIkChain(), &SingleChainIkEffector(),
                             &SingleChainIkPole(), &SingleChainIkTwist(),
                             &SingleChainIkWeight(), &SingleChainIkStretch()},
                            SingleChainIkOut(), &payload);
                        CHECK(node.Run({&input, &rigEffector, &rigPole, &twist,
                                        &weight, &stretch},
                                       &actual));
                        CHECK(same(RigExecSolveSingleChainIk(chain, effector,
                                                             params)));
                    }

        // Defaults: pole 0, 1, 0, no twist, full weight, no stretch.
        SingleChainIkPayload payload;
        Node bare(SingleChainIk(),
                  {&SingleChainIkChain(), &SingleChainIkEffector()},
                  SingleChainIkOut(), &payload);
        CHECK(bare.Run({&input, &rigEffector}, &actual));
        CHECK(same(RigExecSolveSingleChainIk(chain, effector)));
    }

    // A non-finite pole fails the solve: the chain comes back flagged.
    SingleChainIkPayload payload;
    Node node(
        SingleChainIk(),
        {&SingleChainIkChain(), &SingleChainIkEffector(), &SingleChainIkPole()},
        SingleChainIkOut(), &payload);
    const rigExecStd::Vec3d badPole(
        0.0, std::numeric_limits<double>::quiet_NaN(), 0.0);
    const rigExecRig::PointFrame rigEffector = ToRig(chain.back());
    CHECK(node.Run({&input, &rigEffector, &badPole}, &actual));
    for (size_t i = 0; i < actual.frames.Size(); ++i)
        CHECK(actual.frames[i].flags & rigExecRig::kPointFrameDegenerate);
}

/// A node's retained topology, bind or triangulation answers later runs
/// with new posed values as a fresh node does, and a new rest rebinds.
void
TestRetainedState()
{
    using namespace rigExecOps;
    const auto check = [](const std::vector<GfVec3f> &expected,
                          const Points &actual, const char *what, int run) {
        if (!SameBits(expected, actual)) {
            std::printf("%s run %d differs from a fresh run\n", what, run);
            CHECK(false);
        }
    };
    {
        const Grid rest(8, 9, true);
        const Points restArray = ToStd(rest.points);
        const auto counts = ToArray(rest.counts);
        const auto indices = ToArray(rest.indices);
        WrinklePayload payload;
        payload.settings.iterations = 12;
        Node node(Wrinkle(),
                  {&WrinklePoints(), &WrinkleRest(), &WrinkleCounts(),
                   &WrinkleIndices()},
                  WrinkleOut(), &payload);
        for (int run = 0; run < 3; ++run) {
            std::vector<GfVec3f> posed = rest.points;
            for (GfVec3f &p : posed)
                p = GfVec3f(p[0] * (0.6f + 0.1f * run), p[1],
                            p[2] + 0.01f * p[1] * run);
            std::vector<GfVec3f> expected = posed;
            CHECK(RigExecApplyWrinkle(&expected, rest.points, rest.counts,
                                      rest.indices, payload.settings));
            const Points in = ToStd(posed);
            Points actual;
            CHECK(node.Run({&in, &restArray, &counts, &indices}, &actual));
            check(expected, actual, "wrinkle", run);
        }
    }
    {
        const Grid grid(9, 3);
        const GfVec3i d(3, 4, 2);
        const auto restCage =
            Cage(d, GfVec3f(-0.5f, -0.5f, -2), GfVec3f(8.5f, 8.5f, 2), 0, 1);
        LatticePayload payload;
        payload.divisions = {d[0], d[1], d[2]};
        Node node(Lattice(),
                  {&LatticePoints(), &LatticeRestPoints(), &LatticeRestCage(),
                   &LatticePosedCage()},
                  LatticeOut(), &payload);
        const Points in = ToStd(grid.points), restCageArray = ToStd(restCage);
        for (int run = 0; run < 4; ++run) {
            std::vector<GfVec3f> rest = grid.points;
            if (run == 3)
                for (GfVec3f &p : rest)
                    p += GfVec3f(0.25f, -0.1f, 0.05f);
            const auto posedCage =
                Cage(d, GfVec3f(-1, -0.2f, -2.5f), GfVec3f(9, 9.5f, 1.5f), 0.4f,
                     11u + unsigned(run));
            std::vector<GfVec3f> expected = grid.points;
            RigExecApplyLattice(&expected, rest, restCage, posedCage, d);
            const Points restArray = ToStd(rest), posedArray = ToStd(posedCage);
            Points actual;
            CHECK(node.Run({&in, &restArray, &restCageArray, &posedArray},
                           &actual));
            check(expected, actual, "lattice", run);
        }
    }
    {
        const Grid surface(11, 32);
        const Grid target(13, 15);
        const auto counts = ToArray(surface.counts);
        const auto indices = ToArray(surface.indices);
        const GfMatrix4d identity(1.0);
        SurfaceProjectPayload payload;
        payload.settings.mode = RigExecSurfaceSnapMode::Outside;
        payload.settings.offset = 0.15f;
        Node node(SurfaceProject(),
                  {&SurfaceProjectPoints(), &SurfaceProjectSurface(),
                   &SurfaceProjectCounts(), &SurfaceProjectIndices()},
                  SurfaceProjectOut(), &payload);
        for (int run = 0; run < 3; ++run) {
            std::vector<GfVec3f> moved = surface.points, points = target.points;
            for (GfVec3f &p : moved)
                p[2] += 0.05f * run * p[0];
            for (GfVec3f &p : points)
                p = GfVec3f(p[0] * 0.9f - 0.5f, p[1] * 0.9f - 0.5f,
                            p[2] * 0.5f + 0.7f - 0.1f * run);
            std::vector<GfVec3f> expected = points;
            CHECK((RigExecApplySurfaceSnapKernel<GfVec3f, GfVec3d>(
                &expected, moved, surface.counts, surface.indices,
                payload.settings, identity, identity, identity)));
            const Points in = ToStd(points), surfaceArray = ToStd(moved);
            Points actual;
            CHECK(node.Run({&in, &surfaceArray, &counts, &indices}, &actual));
            check(expected, actual, "surface snap", run);
        }
    }
}

/// One node's input values, kept alive while they are bound.
struct BoundValues {
    std::vector<std::shared_ptr<const void>> owned;
    std::vector<const void *> values;

    template <class T> void Add(T value)
    {
        auto held = std::make_shared<const T>(std::move(value));
        values.push_back(held.get());
        owned.push_back(std::move(held));
    }
};

/// \p frames runs of one rig holding one node of \p op per payload, for
/// each batch size and thread count: every node's result on every run
/// equals \p expected(frame, node), the kernel run fresh on the values
/// \p values(frame, node) binds.
template <class Values, class Expected>
void
CheckRetainedAcrossBatches(const char *what, const RigExecOpHandle &op,
                           const std::vector<const RigExecPortHandle *> &ins,
                           const RigExecPortHandle &output,
                           const std::vector<const void *> &payloads,
                           int frames, const Values &values,
                           const Expected &expected)
{
    std::vector<std::vector<std::vector<GfVec3f>>> want(frames);
    std::vector<std::vector<BoundValues>> bound(frames);
    for (int frame = 0; frame < frames; ++frame)
        for (size_t node = 0; node < payloads.size(); ++node) {
            want[frame].push_back(expected(frame, node));
            bound[frame].push_back(values(frame, node));
        }
    const size_t hardware =
        std::max<size_t>(1, std::thread::hardware_concurrency());
    for (const size_t threads : {size_t(1), size_t(2), size_t(3), hardware}) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch : {0, 1, 2, 3, 7}) {
            RigExecRigDescBuilder b;
            std::vector<std::vector<RigExecInputId>> inputs;
            std::vector<RigExecOutputId> outs;
            for (const void *payload : payloads) {
                const RigExecNodeId node = b.AddOp(op, payload);
                inputs.emplace_back();
                for (const RigExecPortHandle *port : ins)
                    inputs.back().push_back(b.BindInput(node, *port));
                outs.push_back(b.BindOutput(node, output));
            }
            std::string error;
            RigExecCompileOptions compileOptions;
            compileOptions.maxBatchNodes = batch;
            RigExecEvaluatorOptions evaluatorOptions;
            evaluatorOptions.tasks = tasks.get();
            auto desc = b.Build(&error);
            auto program =
                desc ? RigExecCompile(*desc, compileOptions, &error) : nullptr;
            auto evaluator = program ? RigExecEvaluator::Create(
                                           program, evaluatorOptions, &error)
                                     : nullptr;
            CHECK(evaluator);
            if (!evaluator)
                return;
            for (int frame = 0; frame < frames; ++frame) {
                for (size_t node = 0; node < payloads.size(); ++node)
                    for (size_t i = 0; i < ins.size(); ++i)
                        CHECK(evaluator->SetInput(inputs[node][i],
                                                  bound[frame][node].values[i],
                                                  &error));
                CHECK(evaluator->Evaluate(&error));
                for (size_t node = 0; node < payloads.size(); ++node) {
                    const auto *actual = static_cast<const Points *>(
                        evaluator->GetOutput(outs[node]));
                    if (!actual || !SameBits(want[frame][node], *actual)) {
                        std::printf("%s: threads %zu batch %zu frame %d node "
                                    "%zu differs from a fresh run\n",
                                    what, threads, batch, frame, node);
                        CHECK(false);
                    }
                }
            }
        }
    }
}

/// Retained topology, binds and triangulations stay per node in batches
/// of any size on any thread count; a node whose rest moves rebinds.
void
TestRetainedStateBatches()
{
    using namespace rigExecOps;
    const size_t nodes = 5;
    const int frames = 4;
    {
        std::vector<Grid> rests;
        std::vector<WrinklePayload> settings(nodes);
        std::vector<const void *> payloads;
        for (size_t n = 0; n < nodes; ++n) {
            rests.emplace_back(int(6 + n), unsigned(9 + n), true);
            settings[n].settings.iterations = int(8 + n);
            payloads.push_back(&settings[n]);
        }
        const auto posed = [&](int frame, size_t n) {
            std::vector<GfVec3f> points = rests[n].points;
            for (GfVec3f &p : points)
                p = GfVec3f(p[0] * (0.6f + 0.1f * frame + 0.02f * n), p[1],
                            p[2] + 0.01f * p[1] * frame);
            return points;
        };
        CheckRetainedAcrossBatches(
            "wrinkle", Wrinkle(),
            {&WrinklePoints(), &WrinkleRest(), &WrinkleCounts(),
             &WrinkleIndices()},
            WrinkleOut(), payloads, frames,
            [&](int frame, size_t n) {
                BoundValues v;
                v.Add(ToStd(posed(frame, n)));
                v.Add(ToStd(rests[n].points));
                v.Add(ToArray(rests[n].counts));
                v.Add(ToArray(rests[n].indices));
                return v;
            },
            [&](int frame, size_t n) {
                std::vector<GfVec3f> expected = posed(frame, n);
                CHECK(RigExecApplyWrinkle(&expected, rests[n].points,
                                          rests[n].counts, rests[n].indices,
                                          settings[n].settings));
                return expected;
            });
    }
    {
        const Grid grid(9, 3);
        const GfVec3i d(3, 4, 2);
        const auto restCage =
            Cage(d, GfVec3f(-0.5f, -0.5f, -2), GfVec3f(8.5f, 8.5f, 2), 0, 1);
        LatticePayload settings;
        settings.divisions = {d[0], d[1], d[2]};
        const std::vector<const void *> payloads(nodes, &settings);
        // Odd nodes' rest points move from frame 2 on.
        const auto rest = [&](int frame, size_t n) {
            std::vector<GfVec3f> points = grid.points;
            for (GfVec3f &p : points)
                p += GfVec3f(0.01f * n, 0, 0);
            if (frame >= 2 && n % 2)
                for (GfVec3f &p : points)
                    p += GfVec3f(0.25f, -0.1f, 0.05f * frame);
            return points;
        };
        const auto posedCage = [&](int frame, size_t n) {
            return Cage(d, GfVec3f(-1, -0.2f, -2.5f), GfVec3f(9, 9.5f, 1.5f),
                        0.4f, unsigned(11 + frame * 7 + n));
        };
        CheckRetainedAcrossBatches(
            "lattice", Lattice(),
            {&LatticePoints(), &LatticeRestPoints(), &LatticeRestCage(),
             &LatticePosedCage()},
            LatticeOut(), payloads, frames,
            [&](int frame, size_t n) {
                BoundValues v;
                v.Add(ToStd(grid.points));
                v.Add(ToStd(rest(frame, n)));
                v.Add(ToStd(restCage));
                v.Add(ToStd(posedCage(frame, n)));
                return v;
            },
            [&](int frame, size_t n) {
                std::vector<GfVec3f> expected = grid.points;
                RigExecApplyLattice(&expected, rest(frame, n), restCage,
                                    posedCage(frame, n), d);
                return expected;
            });
    }
    {
        std::vector<Grid> surfaces;
        const Grid target(7, 15);
        SurfaceProjectPayload settings;
        settings.settings.mode = RigExecSurfaceSnapMode::Outside;
        settings.settings.offset = 0.15f;
        const std::vector<const void *> payloads(nodes, &settings);
        for (size_t n = 0; n < nodes; ++n)
            surfaces.emplace_back(int(5 + n), unsigned(32 + n));
        const auto moved = [&](int frame, size_t n) {
            std::vector<GfVec3f> points = surfaces[n].points;
            for (GfVec3f &p : points)
                p[2] += 0.05f * frame * p[0];
            return points;
        };
        const auto points = [&](int frame) {
            std::vector<GfVec3f> out = target.points;
            for (GfVec3f &p : out)
                p = GfVec3f(p[0] * 0.6f - 0.5f, p[1] * 0.6f - 0.5f,
                            p[2] * 0.5f + 0.7f - 0.1f * frame);
            return out;
        };
        const GfMatrix4d identity(1.0);
        CheckRetainedAcrossBatches(
            "surface snap", SurfaceProject(),
            {&SurfaceProjectPoints(), &SurfaceProjectSurface(),
             &SurfaceProjectCounts(), &SurfaceProjectIndices()},
            SurfaceProjectOut(), payloads, frames,
            [&](int frame, size_t n) {
                BoundValues v;
                v.Add(ToStd(points(frame)));
                v.Add(ToStd(moved(frame, n)));
                v.Add(ToArray(surfaces[n].counts));
                v.Add(ToArray(surfaces[n].indices));
                return v;
            },
            [&](int frame, size_t n) {
                std::vector<GfVec3f> expected = points(frame);
                CHECK((RigExecApplySurfaceSnapKernel<GfVec3f, GfVec3d>(
                    &expected, moved(frame, n), surfaces[n].counts,
                    surfaces[n].indices, settings.settings, identity, identity,
                    identity)));
                return expected;
            });
    }
}

/// Checks that need no frame fail compile, or the first evaluation that
/// gives a node inputs it cannot take, naming the node and the reason; a
/// failure only a frame can show stops that evaluation and publishes
/// nothing.
void
TestBuildTimeChecks()
{
    using namespace rigExecOps;
    std::string error;
    const auto compiles = [&](RigExecRigDescBuilder &b) {
        auto desc = b.Build(&error);
        return desc && RigExecCompile(*desc, {}, &error) != nullptr;
    };
    {
        SkinPayload skin;
        skin.elementSize = 0;
        RigExecRigDescBuilder b;
        const RigExecNodeId n = b.AddOp(Skin(), &skin);
        for (const RigExecPortHandle *port :
             {&SkinPoints(), &SkinTransforms(), &SkinIndices(), &SkinWeights()})
            b.BindInput(n, *port);
        b.BindOutput(n, SkinOut());
        CHECK(!compiles(b));
        CHECK(Contains(error, "node 0 (rig:Skin) is invalid: Skin: "
                              "elementSize must be at least 1"));
    }
    {
        MatrixDeformPayload matrix;
        RigExecRigDescBuilder b;
        const RigExecNodeId n = b.AddOp(MatrixDeform(), &matrix);
        b.BindInput(n, MatrixDeformPoints());
        b.BindInput(n, MatrixDeformTransform());
        b.BindInput(n, MatrixDeformWeightIndices());
        b.BindOutput(n, MatrixDeformOut());
        CHECK(!compiles(b));
        CHECK(Contains(error, "weight indices and weights go together"));
    }
    {
        LatticePayload lattice;
        RigExecRigDescBuilder b;
        const RigExecNodeId n = b.AddOp(Lattice(), &lattice);
        b.BindInput(n, LatticePoints());
        b.BindInput(n, LatticePosedCage());
        b.BindInput(n, LatticeRestPoints());
        b.BindOutput(n, LatticeOut());
        CHECK(!compiles(b));
        CHECK(Contains(error, "needs rest points and a rest cage"));
    }
    {
        BlendShapePayload blend;
        blend.surfaceFrame = true;
        RigExecRigDescBuilder b;
        const RigExecNodeId n = b.AddOp(BlendShape(), &blend);
        b.BindInput(n, BlendShapePoints());
        b.BindInput(n, BlendShapeWeights());
        b.BindOutput(n, BlendShapeOut());
        CHECK(!compiles(b));
        CHECK(Contains(error, "surface frame needs a usable rest mesh"));
    }
    // Counts the host writes: refused at the first evaluation that sees
    // them, and accepted once they fit.
    const Grid grid(4, 3);
    const Points in = ToStd(grid.points);
    const size_t n = grid.points.size();
    SkinPayload skin;
    skin.elementSize = 1;
    RigExecRigDescBuilder b;
    const RigExecNodeId node = b.AddOp(Skin(), &skin);
    const RigExecInputId points = b.BindInput(node, SkinPoints());
    const RigExecInputId transforms = b.BindInput(node, SkinTransforms());
    const RigExecInputId indices = b.BindInput(node, SkinIndices());
    const RigExecInputId weights = b.BindInput(node, SkinWeights());
    const RigExecInputId envelope = b.BindInput(node, SkinEnvelope());
    const RigExecOutputId out = b.BindOutput(node, SkinOut());
    auto program = RigExecCompile(*b.Build(&error), {}, &error);
    auto evaluator = RigExecEvaluator::Create(program);
    std::vector<rigExecStd::Mat4d> mats(2, rigExecStd::Mat4d(1.0));
    mats[1][3][0] = 0.5;
    const auto matArray = ToArray(mats);
    const RigExecArray<int> zeros(n);
    const RigExecArray<int> shortIndices(n - 1);
    std::vector<float> ones(n, 1.0f);
    const auto weightArray = ToArray(ones);
    std::vector<float> half(n, 0.5f);
    const auto envelopeArray = ToArray(half);
    const auto shortEnvelope = ToArray(std::vector<float>(n - 1, 0.5f));
    std::vector<int> wrong(n, 0);
    wrong[2] = 7;
    const auto outside = ToArray(wrong);
    CHECK(evaluator->SetInput(points, &in));
    CHECK(evaluator->SetInput(transforms, &matArray));
    CHECK(evaluator->SetInput(indices, &shortIndices));
    CHECK(evaluator->SetInput(weights, &weightArray));
    CHECK(evaluator->SetInput(envelope, &envelopeArray));
    CHECK(!evaluator->Evaluate(&error));
    CHECK(Contains(error, "node 0 (rig:Skin) is invalid: Skin: indices and "
                          "weights need elementSize per point"));
    CHECK(evaluator->SetInput(indices, &outside));
    CHECK(!evaluator->Evaluate(&error));
    CHECK(Contains(error, "an index is outside the influences"));
    CHECK(evaluator->SetInput(indices, &zeros));
    CHECK(evaluator->SetInput(envelope, &shortEnvelope));
    CHECK(!evaluator->Evaluate(&error));
    CHECK(Contains(error, "envelope length differs from the points"));
    CHECK(evaluator->SetInput(envelope, &envelopeArray));
    CHECK(evaluator->Evaluate(&error));
    const RigExecValueView good = evaluator->ViewOutput(out);
    const Points kept = *static_cast<const Points *>(evaluator->GetOutput(out));
    // A transform that is not finite is a frame's error: the output it was
    // writing holds no valid value; the view keeps the last good one.
    mats[0][0][0] = std::numeric_limits<double>::quiet_NaN();
    const auto badMats = ToArray(mats);
    CHECK(evaluator->SetInput(transforms, &badMats));
    CHECK(!evaluator->Evaluate(&error));
    CHECK(Contains(error, "node 0 (rig:Skin) failed: Skin: transforms must "
                          "be finite and affine"));
    std::string why;
    CHECK(!evaluator->GetOutput(out, &why));
    CHECK(Contains(why, "no valid value: node 0 (rig:Skin) failed"));
    CHECK(SamePoints(kept, *good.As<Points>(rigExecStd::Vec3fArray())));
}

/// What compile, or the first evaluation that binds the inputs, refuses of
/// the pose and solver ops, naming the node and the reason.
void
TestOpChecks()
{
    using namespace rigExecOps;
    std::string error;
    const auto compiles = [&](RigExecRigDescBuilder &b) {
        auto desc = b.Build(&error);
        return desc && RigExecCompile(*desc, {}, &error) != nullptr;
    };
    const auto bind = [](RigExecRigDescBuilder &b, RigExecNodeId n,
                         std::initializer_list<const RigExecPortHandle *> ins) {
        for (const RigExecPortHandle *port : ins)
            b.BindInput(n, *port);
    };
    {
        AutoClavicleOpPayload clavicle;
        clavicle.hasLimb = true;
        RigExecRigDescBuilder b;
        const RigExecNodeId n = b.AddOp(AutoClavicle(), &clavicle);
        bind(b, n,
             {&AutoClavicleTarget(), &AutoClaviclePivot(),
              &AutoClavicleAnchor(), &AutoClavicleAnchorDefault(),
              &AutoClavicleFkPosed(), &AutoClavicleFkDefault(),
              &AutoClavicleIkTarget(), &AutoClaviclePole(),
              &AutoClavicleIkBlend(), &AutoClavicleAmount()});
        b.BindOutput(n, AutoClavicleOut());
        CHECK(!compiles(b));
        CHECK(Contains(error, "node 0 (rig:AutoClavicle) is invalid: "
                              "AutoClavicle: needs three limb rest frames"));
    }
    {
        SpaceMatchPayload space;
        RigExecRigDescBuilder b;
        const RigExecNodeId n = b.AddOp(SpaceMatch(), &space);
        bind(b, n,
             {&SpaceMatchTarget(), &SpaceMatchChannels(),
              &SpaceMatchPosedDefault(), &SpaceMatchParentDefaultInverse(),
              &SpaceMatchParentPosed(), &SpaceMatchParentDefault(),
              &SpaceMatchSourceDefaults(), &SpaceMatchSourcePosed()});
        b.BindOutput(n, SpaceMatchValues());
        CHECK(!compiles(b));
        CHECK(Contains(error, "SpaceMatch: needs at least one space"));
        space.space.world = {0};
        space.space.hasCarry = true;
        RigExecRigDescBuilder carry;
        const RigExecNodeId c = carry.AddOp(SpaceMatch(), &space);
        bind(carry, c,
             {&SpaceMatchTarget(), &SpaceMatchChannels(),
              &SpaceMatchPosedDefault(), &SpaceMatchParentDefaultInverse(),
              &SpaceMatchParentPosed(), &SpaceMatchParentDefault(),
              &SpaceMatchSourceDefaults(), &SpaceMatchSourcePosed()});
        carry.BindOutput(c, SpaceMatchValues());
        CHECK(!compiles(carry));
        CHECK(Contains(error, "its switch's carry space is not fed"));
    }
    {
        IkFkMatchPayload match;
        RigExecRigDescBuilder b;
        const RigExecNodeId n = b.AddOp(IkFkMatch(), &match);
        const RigExecInputId sources = b.BindInput(n, IkFkMatchSources());
        bind(b, n,
             {&IkFkMatchEnabled(),
              &IkFkMatchWeight(),
              &IkFkMatchSwitch(),
              &IkFkMatchStretch(),
              &IkFkMatchSoftDistance(),
              &IkFkMatchUpperScale(),
              &IkFkMatchLowerScale(),
              &IkFkMatchPin(),
              &IkFkMatchTwist(),
              &IkFkMatchRests(),
              &IkFkMatchChannels(),
              &IkFkMatchHasSpace(),
              &IkFkMatchSpaceRest(),
              &IkFkMatchSpacePosed(),
              &IkFkMatchPoleMode(),
              &IkFkMatchPosedDefault(),
              &IkFkMatchParentDefaultInverse(),
              &IkFkMatchParentPosed(),
              &IkFkMatchParentDefault(),
              &IkFkMatchSourceDefaults(),
              &IkFkMatchSourcePosed(),
              &IkFkMatchActive()});
        b.BindOutput(n, IkFkMatchIkValues());
        auto desc = b.Build(&error);
        auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
        auto evaluator = program ? RigExecEvaluator::Create(program) : nullptr;
        CHECK(evaluator);
        if (!evaluator)
            return;
        const RigExecArray<rigExecStd::Mat4d> nine(9);
        CHECK(evaluator->SetInput(sources, &nine));
        CHECK(!evaluator->Evaluate(&error));
        CHECK(Contains(error, "node 0 (rig:IkFkMatch) is invalid: IkFkMatch: "
                              "needs 10 sources, 8 rests and 5 controls' "
                              "channels"));
    }
}

/// A skin, matrix, smooth and delta mush chain runs as one range chain:
/// every grain, thread count and the node-by-node run give the same bits,
/// in each SIMD mode.
void
TestRangeChain()
{
    using namespace rigExecOps;
    const Grid grid(40, 9);
    const size_t n = grid.points.size();
    const int elementSize = 2;
    std::vector<rigExecStd::Mat4d> mats(4, rigExecStd::Mat4d(1.0));
    for (size_t t = 0; t < mats.size(); ++t)
        mats[t][3][0] = 0.05 * double(t + 1);
    std::vector<int> indices(n * elementSize);
    std::vector<float> weights(n * elementSize, 0.5f);
    for (size_t i = 0; i < indices.size(); ++i)
        indices[i] = int((i * 5) % mats.size());
    const Points in = ToStd(grid.points);
    const auto matArray = ToArray(mats);
    const auto indexArray = ToArray(indices);
    const auto weightArray = ToArray(weights);
    const auto counts = ToArray(grid.counts);
    const auto faces = ToArray(grid.indices);
    rigExecStd::Mat4d move(1.0);
    move[3][1] = 0.25;
    SkinPayload skin;
    skin.elementSize = elementSize;
    MatrixDeformPayload matrix;
    DeltaMushPayload mush;
    mush.iterations = 3;
    RigExecRigDescBuilder b;
    const RigExecNodeId s = b.AddOp(Skin(), &skin);
    const RigExecNodeId m = b.AddOp(MatrixDeform(), &matrix);
    const RigExecNodeId sm = b.AddOp(Smooth());
    const RigExecNodeId dm = b.AddOp(DeltaMush(), &mush);
    b.Connect(s, SkinOut(), m, MatrixDeformPoints());
    b.Connect(m, MatrixDeformOut(), sm, SmoothPoints());
    b.Connect(sm, SmoothOut(), dm, DeltaMushPoints());
    std::vector<std::pair<RigExecInputId, const void *>> inputs = {
        {b.BindInput(s, SkinPoints()), &in},
        {b.BindInput(s, SkinTransforms()), &matArray},
        {b.BindInput(s, SkinIndices()), &indexArray},
        {b.BindInput(s, SkinWeights()), &weightArray},
        {b.BindInput(m, MatrixDeformTransform()), &move},
        {b.BindInput(sm, SmoothCounts()), &counts},
        {b.BindInput(sm, SmoothIndices()), &faces},
        {b.BindInput(dm, DeltaMushRest()), &in},
        {b.BindInput(dm, DeltaMushCounts()), &counts},
        {b.BindInput(dm, DeltaMushIndices()), &faces},
    };
    const RigExecOutputId out = b.BindOutput(dm, DeltaMushOut());
    auto desc = b.Build();
    CHECK(desc);
    if (!desc)
        return;
    const size_t hardware =
        std::max<size_t>(1, std::thread::hardware_concurrency());
    auto wide = RigExecCreateThreadTaskSystem(hardware);
    for (const bool simd : {true, false}) {
        // Node by node: each op's program on its own, whole.
        Points stepped = in;
        {
            const auto step =
                [&](const RigExecOpHandle &op, const void *payload,
                    const RigExecPortHandle &pointsIn,
                    const RigExecPortHandle &pointsOut,
                    std::vector<
                        std::pair<const RigExecPortHandle *, const void *>>
                        bound) {
                    RigExecRigDescBuilder one;
                    const RigExecNodeId node = one.AddOp(op, payload);
                    std::vector<std::pair<RigExecInputId, const void *>> set = {
                        {one.BindInput(node, pointsIn), &stepped}};
                    for (const auto &[port, value] : bound)
                        set.push_back({one.BindInput(node, *port), value});
                    const RigExecOutputId result =
                        one.BindOutput(node, pointsOut);
                    RigExecCompileOptions options;
                    options.useSimd = simd;
                    auto program = RigExecCompile(*one.Build(), options);
                    auto evaluator = RigExecEvaluator::Create(program);
                    for (const auto &[id, value] : set)
                        CHECK(evaluator->SetInput(id, value));
                    CHECK(evaluator->Evaluate());
                    stepped = *static_cast<const Points *>(
                        evaluator->GetOutput(result));
                };
            step(Skin(), &skin, SkinPoints(), SkinOut(),
                 {{&SkinTransforms(), &matArray},
                  {&SkinIndices(), &indexArray},
                  {&SkinWeights(), &weightArray}});
            step(MatrixDeform(), &matrix, MatrixDeformPoints(),
                 MatrixDeformOut(), {{&MatrixDeformTransform(), &move}});
            step(Smooth(), nullptr, SmoothPoints(), SmoothOut(),
                 {{&SmoothCounts(), &counts}, {&SmoothIndices(), &faces}});
            step(DeltaMush(), &mush, DeltaMushPoints(), DeltaMushOut(),
                 {{&DeltaMushRest(), &in},
                  {&DeltaMushCounts(), &counts},
                  {&DeltaMushIndices(), &faces}});
        }
        for (const size_t grain :
             {size_t(0), size_t(1), size_t(13), size_t(100), size_t(512)}) {
            RigExecCompileOptions options;
            options.useSimd = simd;
            options.rangeGrain = grain;
            auto program = RigExecCompile(*desc, options);
            for (RigExecTaskSystem *tasks :
                 {static_cast<RigExecTaskSystem *>(nullptr), gTasks.get(),
                  wide.get()}) {
                RigExecEvaluatorOptions evaluate;
                evaluate.tasks = tasks;
                auto evaluator = RigExecEvaluator::Create(program, evaluate);
                for (const auto &[id, value] : inputs)
                    CHECK(evaluator->SetInput(id, value));
                std::string error;
                CHECK(evaluator->Evaluate(&error));
                const auto *result =
                    static_cast<const Points *>(evaluator->GetOutput(out));
                const bool same = result && SamePoints(stepped, *result);
                if (!same)
                    std::printf("range chain: simd %d grain %zu differs\n",
                                int(simd), grain);
                CHECK(same);
            }
        }
        // In a host's schedule over groups it cuts, the skin reading the
        // host's points group by group and the chain keeping the values
        // only its own members read as one buffer per group.
        RigExecCompileOptions options;
        options.useSimd = simd;
        options.hostOrder = true;
        auto program = RigExecCompile(*desc, options);
        RigExecChainDesc chain;
        CHECK(program && RigExecProgramChainCount(*program) == 1 &&
              RigExecProgramChainAt(*program, 0, &chain));
        if (!program)
            continue;
        CHECK(chain.grouped == std::vector<char>({1, 1, 1, 0}));
        for (const uint32_t groups : {1u, 4u, 9u}) {
            std::vector<int> cut;
            for (uint32_t g = 0; g <= groups; ++g)
                cut.push_back(int(n * g / groups));
            std::vector<const void *> pointsAt(groups);
            for (uint32_t g = 0; g < groups; ++g)
                pointsAt[g] = in.Data() + cut[g];
            const RigExecOpGroups given{cut.data(), groups, pointsAt.data()};
            auto state = RigExecRunState::Create(program);
            for (const auto &[id, value] : inputs)
                CHECK(state->PointInput(id, value));
            std::string error;
            CHECK(state->PointInputGroups(inputs[0].first, &given, &error));
            CHECK(state->RunChainHead(0, cut.data(), groups, &error));
            for (uint32_t segment = 0; segment < chain.members.size();
                 ++segment)
                if (chain.segments[segment] == segment)
                    for (uint32_t g = 0; g < groups; ++g)
                        CHECK(state->RunChainRange(0, segment, g, &error));
            RigExecValueRef ref;
            CHECK(RigExecResolveNodeOutput(*program, dm, DeltaMushOut(), &ref));
            const auto *result =
                static_cast<const Points *>(state->GetValue(ref));
            const bool same = result && SamePoints(stepped, *result);
            if (!same)
                std::printf("host range chain: simd %d groups %u differs\n",
                            int(simd), groups);
            CHECK(same);
            CHECK(
                RigExecResolveNodeOutput(*program, m, MatrixDeformOut(), &ref));
            CHECK(!state->GetValue(ref));
            const RigExecInputId transform = inputs[4].first;
            CHECK(!state->PointInputGroups(transform, &given, &error));
        }
    }
}

/// The per-frame time of a deform chain (skin, matrix, smooth) over a large
/// mesh, whole and cut into the scene's ranges on worker threads; every
/// setting gives the same bits.
void
ReportRangeTiming()
{
    using namespace rigExecOps;
    const Grid grid(400, 7);
    const size_t n = grid.points.size();
    const int elementSize = 4;
    std::vector<rigExecStd::Mat4d> mats(8, rigExecStd::Mat4d(1.0));
    std::vector<int> indices(n * elementSize);
    std::vector<float> weights(n * elementSize);
    for (size_t i = 0; i < n; ++i)
        for (int s = 0; s < elementSize; ++s) {
            indices[i * elementSize + s] = int((i + size_t(s) * 3) % 8);
            weights[i * elementSize + s] = 0.25f;
        }
    const Points in = ToStd(grid.points);
    const auto indexArray = ToArray(indices);
    const auto weightArray = ToArray(weights);
    const auto counts = ToArray(grid.counts);
    const auto faces = ToArray(grid.indices);
    SkinPayload skin;
    skin.elementSize = elementSize;
    MatrixDeformPayload matrix;
    RigExecRigDescBuilder b;
    const RigExecNodeId s = b.AddOp(Skin(), &skin);
    const RigExecNodeId m = b.AddOp(MatrixDeform(), &matrix);
    const RigExecNodeId sm = b.AddOp(Smooth());
    b.Connect(s, SkinOut(), m, MatrixDeformPoints());
    b.Connect(m, MatrixDeformOut(), sm, SmoothPoints());
    const RigExecInputId points = b.BindInput(s, SkinPoints());
    const RigExecInputId transforms = b.BindInput(s, SkinTransforms());
    const RigExecInputId skinIndices = b.BindInput(s, SkinIndices());
    const RigExecInputId skinWeights = b.BindInput(s, SkinWeights());
    const RigExecInputId transform = b.BindInput(m, MatrixDeformTransform());
    const RigExecInputId faceCounts = b.BindInput(sm, SmoothCounts());
    const RigExecInputId faceIndices = b.BindInput(sm, SmoothIndices());
    const RigExecOutputId out = b.BindOutput(sm, SmoothOut());
    auto desc = b.Build();
    const size_t hardware =
        std::max<size_t>(1, std::thread::hardware_concurrency());
    auto wide = RigExecCreateThreadTaskSystem(hardware);
    struct Setting {
        const char *name;
        size_t grain;
        RigExecTaskSystem *tasks;
    };
    const Setting settings[] = {
        {"whole, inline", 0, nullptr},
        {"ranges of 512, 4 threads", 512, gTasks.get()},
        {"ranges of 512, all threads", 512, wide.get()},
    };
    Points first;
    for (const Setting &setting : settings) {
        RigExecCompileOptions options;
        options.rangeGrain = setting.grain;
        options.rangeThreshold = 4096;
        RigExecEvaluatorOptions evaluate;
        evaluate.tasks = setting.tasks;
        auto program = desc ? RigExecCompile(*desc, options) : nullptr;
        auto evaluator =
            program ? RigExecEvaluator::Create(program, evaluate) : nullptr;
        CHECK(evaluator);
        if (!evaluator)
            return;
        evaluator->SetInput(points, &in);
        evaluator->SetInput(skinIndices, &indexArray);
        evaluator->SetInput(skinWeights, &weightArray);
        evaluator->SetInput(faceCounts, &counts);
        evaluator->SetInput(faceIndices, &faces);
        const int frames = 20;
        double seconds = 0.0;
        for (int frame = -2; frame < frames; ++frame) {
            for (size_t t = 0; t < mats.size(); ++t)
                mats[t][3][0] = 0.01 * double(frame + 3) * double(t);
            const auto matArray = ToArray(mats);
            rigExecStd::Mat4d move(1.0);
            move[3][1] = 0.1 * double(frame);
            evaluator->SetInput(transforms, &matArray);
            evaluator->SetInput(transform, &move);
            const auto start = std::chrono::steady_clock::now();
            CHECK(evaluator->Evaluate());
            const auto stop = std::chrono::steady_clock::now();
            if (frame >= 0)
                seconds += std::chrono::duration<double>(stop - start).count();
        }
        const auto &result =
            *static_cast<const Points *>(evaluator->GetOutput(out));
        if (first.Empty())
            first = result;
        CHECK(SamePoints(first, result));
        std::printf("range timing: %zu points, %s: %.2f ms per frame\n", n,
                    setting.name, 1000.0 * seconds / frames);
    }
}

int
main()
{
    std::vector<std::string> errors;
    gTasks = RigExecCreateThreadTaskSystem(4);
    if (!RigExecRegistry::Initialize(RigExecLinkedRegistrations(), &errors)) {
        for (const std::string &e : errors)
            std::printf("%s\n", e.c_str());
        return 1;
    }
    TestSmooth();
    TestVolumeCorrect();
    TestDeltaMush();
    TestLattice();
    TestWrinkle();
    TestSurfaceProject();
    TestRetainedState();
    TestRetainedStateBatches();
    TestMatrixDeform();
    TestMoverTransform();
    TestSkin();
    TestBlendShape();
    TestRecomputeNormals();
    TestRecomputeExtent();
    TestWire();
    TestRibbon();
    TestSurfaceProjector();
    TestShaderDials();
    TestConstraints();
    TestWeightFields();
    TestChainSolvers();
    TestFrameBlend();
    TestRibbonFrames();
    TestSingleChainIk();
    TestPoseCommit();
    TestSplineIk();
    TestRangeChain();
    TestBuildTimeChecks();
    TestOpChecks();
    ReportRangeTiming();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
