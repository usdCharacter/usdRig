// The point ops evaluated through the core give the bits the baked
// evaluator's Gf kernels give, envelope blend included.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExecOps/deform/blendShapeOp.h"
#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecOps/deform/surfaceProjectOp.h"
#include "rigExecOps/deform/wrinkleOp.h"

#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>
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
         const RigExecPortHandle &output, const void *payload = nullptr)
    {
        RigExecRigDescBuilder b;
        const RigExecNodeId node = b.AddOp(op, payload);
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
    TestDeltaMush();
    TestLattice();
    TestWrinkle();
    TestSurfaceProject();
    TestMatrixDeform();
    TestSkin();
    TestBlendShape();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
