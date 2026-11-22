// The point ops evaluated through the core give the bits the baked
// evaluator's Gf kernels give, envelope blend included.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecOps/deform/deltaMushOp.h"
#include "rigExecOps/deform/latticeOp.h"
#include "rigExecOps/deform/pointOps.h"
#include "rigExecOps/deform/wrinkleOp.h"

#include "pxr/base/gf/rotation.h"

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
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
