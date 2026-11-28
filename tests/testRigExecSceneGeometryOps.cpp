// A geometry revision run as rigExecOps' deform op gives the bits
// RigExecRunGeometry gives: per op, at full strength, under a constant, a
// dense and a sparse envelope, alone and chained, and both refuse the same
// envelopes.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExecBuiltins/builtins.h"
#include "opRunners/geometryOpRunner.h"
#include "rigExecScene/geometryOps.h"
#include "rigExecScene/geometryProgram.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/internal/weightPackets.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/weightFieldKernel.h"
#include "rigExecOps/deform/projectorOps.h"
#include "rigExecOps/weight/weightOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/rotation.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace rigExec;

namespace {
int failures = 0;
/// Whether the ops and the native kernels take their SSE2 forms.
bool g_useSimd = true;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

/// A bumpy quad grid with its topology.
struct Grid {
    std::vector<GfVec3f> points, rest;
    std::vector<int> counts, indices;

    Grid(int nx, int ny)
    {
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x) {
                const float fx = float(x), fy = float(y);
                rest.emplace_back(fx, fy, 0.0f);
                points.emplace_back(fx + 0.05f * std::sin(fy),
                                    fy + 0.03f * std::cos(fx),
                                    0.2f * std::sin(0.7f * fx + 0.3f * fy));
            }
        for (int y = 0; y + 1 < ny; ++y)
            for (int x = 0; x + 1 < nx; ++x) {
                const int a = y * nx + x;
                counts.push_back(4);
                indices.insert(indices.end(), {a, a + 1, a + nx + 1, a + nx});
            }
    }
};

RigExecWeightPacket
Dense(size_t count)
{
    RigExecWeightPacket w;
    w.representation = TfToken("dense");
    w.rangePolicy = TfToken("strict");
    w.valid = true;
    for (size_t i = 0; i < count; ++i)
        w.values.push_back(i % 5 == 0   ? 0.0f
                           : i % 7 == 0 ? 1.0f
                                        : 0.11f * float(i % 9));
    return w;
}

/// Weight on the first quarter only: the native delta mush runs its masked
/// kernel there.
RigExecWeightPacket
Narrow(size_t count)
{
    RigExecWeightPacket w = Dense(count);
    for (size_t i = count / 4; i < count; ++i)
        w.values[i] = 0.0f;
    return w;
}

/// A sparse field with a zero default naming every third point: the matrix
/// and wire kernels walk it entry by entry.
RigExecWeightPacket
Sparse(size_t count)
{
    RigExecWeightPacket w;
    w.representation = TfToken("sparse");
    w.rangePolicy = TfToken("strict");
    w.valid = true;
    for (size_t i = 0; i < count; i += 3) {
        w.indices.push_back(int(i));
        w.values.push_back(i % 2 ? 0.3f : 0.85f);
    }
    return w;
}

/// The envelopes each op is run under.
std::vector<std::pair<const char *, RigExecWeightPacket>>
Envelopes(size_t count)
{
    return {{"full", RigExecWeightPacket::Constant(1.0f)},
            {"constant", RigExecWeightPacket::Constant(0.37f)},
            {"dense", Dense(count)},
            {"narrow", Narrow(count)},
            {"sparse", Sparse(count)}};
}

bool
SameBits(const std::vector<GfVec3f> &expected, const RigExecArray<Vec3f> &got)
{
    return expected.size() == got.Size() &&
           (expected.empty() ||
            std::memcmp(expected.data(), got.Data(),
                        expected.size() * sizeof(GfVec3f)) == 0);
}

/// \p packets run as a chain of op nodes over \p points.
struct Chain {
    std::vector<RigExecGeometryOpNode> nodes;
    std::unique_ptr<RigExecEvaluator> evaluator;

    Chain(const std::vector<
          std::pair<RigExecRevisionOp, const RigExecMoverParameters *>> &ops)
    {
        RigExecRigDescBuilder builder;
        std::string error;
        for (const auto &[op, p] : ops) {
            RigExecGeometryOpNode node;
            const bool added = RigExecAddGeometryOp(
                op, *p, &builder, nodes.empty() ? nullptr : &nodes.back(),
                &node, &error);
            CHECK(added);
            if (!added) {
                std::printf("add: %s\n", error.c_str());
                return;
            }
            nodes.push_back(node);
        }
        auto desc = builder.Build(&error);
        RigExecCompileOptions options;
        options.useSimd = g_useSimd;
        auto program = desc ? RigExecCompile(*desc, options, &error) : nullptr;
        evaluator =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
        if (!evaluator)
            std::printf("chain: %s\n", error.c_str());
        CHECK(evaluator);
    }

    bool Run(const std::vector<const RigExecMoverParameters *> &packets,
             const std::vector<GfVec3f> &points, RigExecArray<Vec3f> *out,
             std::string *error)
    {
        if (!evaluator || nodes.size() != packets.size())
            return false;
        for (size_t i = 0; i < nodes.size(); ++i)
            if (!RigExecWriteGeometryOpInputs(*packets[i], nodes[i], &points,
                                              points.size(), evaluator.get(),
                                              error))
                return false;
        if (!evaluator->Evaluate(error))
            return false;
        *out = *static_cast<const RigExecArray<Vec3f> *>(
            evaluator->GetOutput(nodes.back().out));
        return true;
    }
};

/// The native answer over \p points, in the mode the ops are compiled in.
bool
Native(RigExecRevisionOp op, const RigExecMoverParameters &p,
       std::vector<GfVec3f> *points)
{
    RigExecGeometryWorkspace workspace;
    return RigExecRunGeometry(op, p, points, g_useSimd, &workspace.wire,
                              &workspace.surface);
}

/// \p p under each envelope, native against the op.
void
Compare(const char *label, RigExecRevisionOp op, RigExecMoverParameters p,
        const std::vector<GfVec3f> &points)
{
    for (const auto &[name, weights] : Envelopes(points.size())) {
        p.weights = weights;
        std::vector<GfVec3f> expected = points;
        const bool native = Native(op, p, &expected);
        CHECK(native);
        Chain chain({{op, &p}});
        RigExecArray<Vec3f> actual;
        std::string error;
        const bool ran = chain.Run({&p}, points, &actual, &error);
        CHECK(ran);
        if (native && ran && !SameBits(expected, actual)) {
            std::printf("%s %s differs\n", label, name);
            CHECK(false);
        }
        if (!ran)
            std::printf("%s %s: %s\n", label, name, error.c_str());
    }
}

RigExecMoverParameters
Packet(const char *kind)
{
    RigExecMoverParameters p;
    p.kind = TfToken(kind);
    p.valid = true;
    return p;
}

void
TestMatrix(const Grid &grid)
{
    GfMatrix4d turn(1.0);
    turn.SetRotate(GfRotation(GfVec3d(0.1, 0.3, 1), 70.0));
    turn.SetTranslateOnly(GfVec3d(3, -2, 5));
    for (const bool radial : {false, true}) {
        RigExecMoverParameters p = Packet("matrix");
        p.transform = turn;
        p.radialWeight = radial;
        Compare(radial ? "matrix radial" : "matrix", RigExecRevisionOp::Matrix,
                p, grid.points);
    }
}

void
TestSkin(const Grid &grid)
{
    GfMatrix4d a(1.0), b(1.0);
    a.SetRotate(GfRotation(GfVec3d(0, 0, 1), 25.0));
    b.SetRotate(GfRotation(GfVec3d(1, 0, 0), -40.0));
    b.SetTranslateOnly(GfVec3d(0.5, 1, -0.25));
    for (const char *method : {"classicLinear", "dualQuaternion"}) {
        RigExecMoverParameters p = Packet("skin");
        p.skinningMethod = TfToken(method);
        p.skinTransforms = {GfMatrix4d(1.0), a, b};
        p.skinElementSize = 2;
        for (size_t i = 0; i < grid.points.size(); ++i) {
            const float t = float(i % 11) / 10.0f;
            p.skinIndices.insert(p.skinIndices.end(), {int(i % 3), 2});
            p.skinWeights.insert(p.skinWeights.end(), {1.0f - t, t});
        }
        Compare(method, RigExecRevisionOp::Skin, p, grid.points);
    }
}

void
TestPointOps(const Grid &grid)
{
    RigExecMoverParameters volume = Packet("volumeCorrect");
    volume.referenceVolume = 3.5;
    volume.strength = 0.8f;
    Compare("volumeCorrect", RigExecRevisionOp::VolumeCorrect, volume,
            grid.points);

    RigExecMoverParameters smooth = Packet("smooth");
    smooth.topologyCounts = grid.counts;
    smooth.topologyIndices = grid.indices;
    smooth.strength = 0.6f;
    Compare("smooth", RigExecRevisionOp::Smooth, smooth, grid.points);

    RigExecMoverParameters mush = Packet("deltaMush");
    mush.restPoints = grid.rest;
    mush.topologyCounts = grid.counts;
    mush.topologyIndices = grid.indices;
    mush.mushIterations = 4;
    mush.mushStep = 0.4f;
    mush.mushDisplacement = 0.9f;
    Compare("deltaMush", RigExecRevisionOp::DeltaMush, mush, grid.points);
}

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
TestWrinkle(const Grid &grid)
{
    RigExecMoverParameters p = Packet("wrinkle");
    p.restPoints = grid.rest;
    p.topologyCounts = grid.counts;
    p.topologyIndices = grid.indices;
    p.wrinkleSettings.iterations = 6;
    // Compressed in x, so the edges buckle.
    std::vector<GfVec3f> posed = grid.points;
    for (GfVec3f &v : posed)
        v[0] *= 0.8f;
    Compare("wrinkle", RigExecRevisionOp::Wrinkle, p, posed);
}

void
TestLattice(const Grid &grid)
{
    const GfVec3i d(3, 4, 2);
    RigExecMoverParameters bound = Packet("lattice");
    bound.divisions = d;
    bound.restPoints = grid.points;
    bound.auxPoints =
        Cage(d, GfVec3f(-0.5f, -0.5f, -2), GfVec3f(8.5f, 6.5f, 2), 0, 1);
    bound.auxPointsB =
        Cage(d, GfVec3f(-1, -0.2f, -2.5f), GfVec3f(9, 7.5f, 1.5f), 0.4f, 7);
    Compare("lattice bound", RigExecRevisionOp::Lattice, bound, grid.points);

    const GfVec3i g(4, 4, 3);
    RigExecMoverParameters regular = Packet("lattice");
    regular.divisions = g;
    RigExecLatticeSettings &s = regular.latticeSettings;
    s.regularGrid = true;
    s.origin = {-1, -1, -2};
    s.spacing = {10.0f / 3, 8.0f / 3, 2.0f};
    regular.auxPointsB =
        Cage(g, GfVec3f(-1, -1, -2), GfVec3f(9, 7, 2), 0.6f, 11);
    regular.targetToLattice.SetTranslate(GfVec3d(0.25, -0.5, 0.1));
    regular.latticeToTarget.SetTranslate(GfVec3d(-0.25, 0.5, -0.1));
    Compare("lattice grid", RigExecRevisionOp::Lattice, regular, grid.points);
}

void
TestSurfaceProject(const Grid &grid)
{
    const Grid surface(6, 5);
    RigExecMoverParameters p = Packet("surfaceProject");
    for (const GfVec3f &v : surface.rest)
        p.auxPoints.push_back(GfVec3f(v[0] * 1.5f - 0.5f, v[1] * 1.4f, 0.3f));
    p.topologyCounts = surface.counts;
    p.topologyIndices = surface.indices;
    p.strength = 0.75f;
    std::vector<GfVec3f> above = grid.points;
    for (GfVec3f &v : above)
        v[2] += 0.8f;
    Compare("surfaceProject legacy", RigExecRevisionOp::SurfaceProject, p,
            above);
    p.targetToSurface.SetTranslate(GfVec3d(0, 0, -0.1));
    p.surfaceToTarget.SetTranslate(GfVec3d(0, 0, 0.1));
    Compare("surfaceProject snap", RigExecRevisionOp::SurfaceProject, p, above);
}

/// A rest and a posed curve's RMF samples as a frame array.
RigExecPointFrameArray
RibbonFrames()
{
    std::vector<GfVec3f> restCurve, posedCurve;
    for (int i = 0; i < 6; ++i) {
        restCurve.emplace_back(1.4f * i, 0.5f * i, 0.3f * (i % 2));
        posedCurve.emplace_back(1.3f * i, 0.5f * i + 0.2f * i * i,
                                0.4f * i - 0.3f * (i % 2));
    }
    const RigExecCurveFrameSamples rest = RigExecSampleCurveRMF(restCurve, 9);
    const RigExecCurveFrameSamples posed = RigExecSampleCurveRMF(posedCurve, 9);
    const auto landmarks = [](const RigExecCurveFrameSamples &f, size_t k) {
        const GfVec3f o = f.positions[k];
        return std::array<GfVec3d, 4>{GfVec3d(o), GfVec3d(o + f.tangents[k]),
                                      GfVec3d(o + f.normals[k]),
                                      GfVec3d(o + f.binormals[k])};
    };
    RigExecPointFrameArray frames;
    frames.frames.resize(rest.GetSize());
    for (size_t k = 0; k < rest.GetSize(); ++k) {
        frames.rests.push_back(landmarks(rest, k));
        frames.frames[k].points = landmarks(posed, k);
    }
    return frames;
}

void
TestFrameDriven(const Grid &grid)
{
    RigExecMoverParameters ribbon = Packet("ribbon");
    ribbon.frames = RibbonFrames();
    for (size_t i = 0; i < grid.points.size(); ++i)
        ribbon.bindCoords.emplace_back(
            -0.1f + 1.2f * float(i) / float(grid.points.size()), 0.0f);
    Compare("ribbon", RigExecRevisionOp::Ribbon, ribbon, grid.points);

    RigExecMoverParameters guides = Packet("emitGuidePoints");
    guides.frames = ribbon.frames;
    Compare("guidePoints", RigExecRevisionOp::EmitGuidePoints, guides,
            std::vector<GfVec3f>(guides.frames.frames.size(),
                                 GfVec3f(0.5f, -1.0f, 2.0f)));
}

void
TestWire(const Grid &grid)
{
    RigExecMoverParameters p = Packet("wire");
    for (int i = 0; i < 6; ++i) {
        p.restPoints.emplace_back(1.6f * i, 1.2f * i, 0.4f * (i % 2));
        p.auxPoints.push_back(p.restPoints.back() +
                              GfVec3f(0.3f * i, -0.2f * (i % 3), 0.5f));
    }
    p.curveOrder = 4;
    p.curveKnots = {0, 0, 0, 0, 1, 2, 3, 3, 3, 3};
    p.dropoffDistance = 2.5;
    const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder, &p.curveKnots};
    const std::vector<GfVec2f> bind = RigExecBindWire(grid.points, rest);
    p.wireBindCoords = VtArray<GfVec2f>(bind.begin(), bind.end());
    Compare("wire", RigExecRevisionOp::Wire, p, grid.points);

    // A sparse zero-default field: the op folds it in, as the native wire.
    p.weights = RigExecWeightPacket();
    p.weights.representation = TfToken("sparse");
    p.weights.rangePolicy = TfToken("strict");
    p.weights.valid = true;
    for (int i = 2; i < int(grid.points.size()); i += 3) {
        p.weights.indices.push_back(i);
        p.weights.values.push_back(0.1f * float(i % 10));
    }
    std::vector<GfVec3f> expected = grid.points;
    CHECK(Native(RigExecRevisionOp::Wire, p, &expected));
    Chain chain({{RigExecRevisionOp::Wire, &p}});
    CHECK(chain.nodes.size() == 1 &&
          chain.nodes[0].envelope.index == kRigExecInvalidIndex);
    RigExecArray<Vec3f> actual;
    std::string error;
    CHECK(chain.Run({&p}, grid.points, &actual, &error));
    CHECK(SameBits(expected, actual));
    // The field's shape is fixed at build.
    p.weights = RigExecWeightPacket::Constant(0.5f);
    CHECK(!chain.Run({&p}, grid.points, &actual, &error));
}

/// The native derived answer: recomputed from the final points, blended
/// toward \p authored.
bool
NativeDerived(RigExecRevisionOp op, const RigExecMoverParameters &p,
              std::vector<GfVec3f> *authored)
{
    return RigExecRunGeometryDerived(op, p, authored->data(), authored->size(),
                                     authored);
}

void
TestDerived(const Grid &grid)
{
    for (const bool perCorner : {false, true})
        for (const bool leftHanded : {false, true}) {
            RigExecMoverParameters p = Packet("recomputeNormals");
            p.auxPoints = grid.points;
            p.topologyCounts = grid.counts;
            p.topologyIndices = grid.indices;
            p.normalsPerCorner = perCorner;
            p.normalsLeftHanded = leftHanded;
            const std::vector<GfVec3f> authored(perCorner ? grid.indices.size()
                                                          : grid.points.size(),
                                                GfVec3f(0, 0, 1));
            for (const auto &[name, weights] : Envelopes(authored.size())) {
                p.weights = weights;
                std::vector<GfVec3f> expected = authored;
                CHECK(NativeDerived(RigExecRevisionOp::RecomputeNormals, p,
                                    &expected));
                Chain chain({{RigExecRevisionOp::RecomputeNormals, &p}});
                RigExecArray<Vec3f> actual;
                std::string error;
                CHECK(chain.Run({&p}, authored, &actual, &error));
                if (!SameBits(expected, actual)) {
                    std::printf("normals corner %d left %d %s differs\n",
                                int(perCorner), int(leftHanded), name);
                    CHECK(false);
                }
            }
        }

    RigExecMoverParameters extent = Packet("recomputeExtent");
    extent.auxPoints = grid.points;
    const std::vector<GfVec3f> authored = {GfVec3f(-1), GfVec3f(1)};
    for (const std::vector<float> &widths :
         {std::vector<float>{}, std::vector<float>{0.5f},
          std::vector<float>(grid.points.size(), 0.25f)}) {
        extent.widths = widths;
        extent.weights = RigExecWeightPacket::Constant(0.6f);
        std::vector<GfVec3f> expected = authored;
        CHECK(NativeDerived(RigExecRevisionOp::RecomputeExtent, extent,
                            &expected));
        Chain chain({{RigExecRevisionOp::RecomputeExtent, &extent}});
        RigExecArray<Vec3f> actual;
        std::string error;
        CHECK(chain.Run({&extent}, authored, &actual, &error));
        CHECK(SameBits(expected, actual));
    }
}

/// Runs one projector or dials node and compares it with the native
/// matrix: both succeed with the same bits, or both fail. Where \p miss,
/// the ray measures no frame and both output the shaderOffset fallback.
/// True when the native run succeeded.
bool
CompareMatrix(RigExecRevisionOp op, const RigExecRevisionBinding &binding,
              const RigExecSurfaceProjectorFrames &frames,
              const RigExecProjectorReads &reads,
              const std::vector<GfVec3f> &base,
              const std::vector<GfVec3f> &final, const char *label,
              bool miss = false)
{
    GfMatrix4d expected(1.0);
    std::vector<std::string> diagnostics;
    const std::string who = op == RigExecRevisionOp::SurfaceProjector
                                ? RigExecGeometryMoverText(binding)
                                : std::string();
    const bool nativeOk =
        RigExecRunGeometryMatrix(op, binding, frames, reads, base, final, who,
                                 &expected, &diagnostics, nullptr);

    if (miss) {
        CHECK(nativeOk);
        CHECK(std::memcmp(&expected, &reads.shaderOffset, sizeof(expected)) ==
              0);
        CHECK(!diagnostics.empty() &&
              diagnostics.back().find("cast=MISS") != std::string::npos);
    }

    // Where the native run fails on unusable input, the op's evaluation
    // stops with an error holding each of the native run's notes.
    RigExecGeometryOpRunner runner;
    runner.compile.useSimd = g_useSimd;
    GfMatrix4d ran(1.0);
    std::string diagnostic;
    const bool runnerOk = RigExecRunGeometryMatrixOp(
        op, binding, frames, reads, base, final, runner, ran, diagnostic);
    CHECK(runnerOk == nativeOk);
    if (!nativeOk) {
        for (const std::string &line : diagnostics) {
            const std::string body = line.substr(line.find(": ") + 2);
            if (diagnostic.find("SurfaceProjector: ") == std::string::npos ||
                diagnostic.find(body) == std::string::npos) {
                std::printf("%s: native '%s' runner '%s'\n", label,
                            line.c_str(), diagnostic.c_str());
                CHECK(false);
            }
        }
    } else {
        CHECK(std::memcmp(&ran, &expected, sizeof(ran)) == 0);
    }

    RigExecRigDescBuilder builder;
    RigExecGeometryMatrixNode node;
    std::string error;
    CHECK(
        RigExecAddGeometryMatrixOp(op, frames, reads, &builder, &node, &error));
    auto desc = builder.Build(&error);
    CHECK(desc);
    if (!desc)
        return false;
    auto program = RigExecCompile(*desc, {}, &error);
    CHECK(program);
    if (!program)
        return false;
    auto evaluator = RigExecEvaluator::Create(program, {}, &error);
    CHECK(evaluator);
    CHECK(RigExecWriteGeometryMatrixInputs(binding, frames, reads, base, final,
                                           node, evaluator.get(), &error));
    const bool opOk = evaluator->Evaluate(&error);
    if (opOk != nativeOk) {
        std::printf("%s: native %d op %d (%s)\n", label, int(nativeOk),
                    int(opOk), error.c_str());
        CHECK(false);
        return false;
    }
    if (!opOk) {
        CHECK(error.find(") failed: ") == std::string::npos ||
              !evaluator->GetOutput(node.out));
        return false;
    }
    const auto *actual =
        static_cast<const Mat4d *>(evaluator->GetOutput(node.out));
    CHECK(actual);
    if (!actual)
        return false;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            if (std::memcmp(&expected[r][c], &actual->_mtx[r][c],
                            sizeof(double)) != 0) {
                std::printf("%s: [%d][%d] %.17g vs %.17g\n", label, r, c,
                            expected[r][c], actual->_mtx[r][c]);
                CHECK(false);
                return false;
            }
    return nativeOk;
}

void
TestProjector(const Grid &grid)
{
    RigExecRevisionBinding binding;
    RigExecProjectorReads reads;
    reads.rayOrigin = GfVec3d(3.3, 2.6, 2.0);
    reads.rayDirection = GfVec3d(0.05, -0.02, -1.0);
    reads.rayUp = GfVec3d(0.0, 1.0, 0.0);
    reads.shaderOffset.SetTranslate(GfVec3d(0.1, 0.2, 0.3));
    reads.faceVertexCounts = grid.counts;
    reads.faceVertexIndices = grid.indices;

    // A source casts along its own z: placed above the grid, facing down.
    GfMatrix4d sourceBase, sourceFinal, spaceFinal, carry;
    sourceBase.SetRotate(GfRotation(GfVec3d(1, 0, 0), 180.0));
    sourceBase.SetTranslateOnly(GfVec3d(3.3, 2.6, 2.0));
    sourceFinal.SetRotate(GfRotation(GfVec3d(1, 0, 0), 180.0) *
                          GfRotation(GfVec3d(0, 0, 1), 12.0));
    sourceFinal.SetTranslateOnly(GfVec3d(3.6, 2.4, 2.2));
    spaceFinal.SetTranslate(GfVec3d(-0.3, 0.1, 0.05));
    carry.SetScale(1.1);

    struct Case {
        const char *label;
        bool named[3], resolved[3];
        bool reproject;
        bool meshOffset;
    };
    const Case cases[] = {
        {"no frames", {}, {}, false, false},
        {"source", {true, false, false}, {true, false, false}, false, false},
        {"source reproject",
         {true, false, false},
         {true, false, false},
         true,
         false},
        {"source and space",
         {true, true, false},
         {true, true, false},
         false,
         false},
        {"all three", {true, true, true}, {true, true, true}, false, true},
        {"space unresolved",
         {true, true, true},
         {true, false, false},
         false,
         false},
        {"source unresolved",
         {true, false, false},
         {false, false, false},
         false,
         true},
    };
    for (const Case &c : cases) {
        RigExecSurfaceProjectorFrames frames;
        for (int k = 0; k < 3; ++k) {
            frames.named[k] = c.named[k];
            frames.resolved[k] = c.resolved[k];
        }
        frames.base[0] = sourceBase;
        frames.final[0] = sourceFinal;
        frames.base[1] = GfMatrix4d(1.0);
        frames.final[1] = spaceFinal;
        frames.final[2] = carry;
        reads.reproject = c.reproject;
        binding.meshWorldInverse = GfMatrix4d(1.0);
        if (c.meshOffset)
            binding.meshWorldInverse.SetTranslate(GfVec3d(0.0, 0.0, -0.1));
        CHECK(CompareMatrix(RigExecRevisionOp::SurfaceProjector, binding,
                            frames, reads, grid.rest, grid.points, c.label));
    }

    // A ray that misses: native and the op output the shaderOffset, also
    // when an unresolved rig space is noted before the miss.
    binding.moverPath = SdfPath("/Rig/Projector");
    RigExecProjectorReads miss = reads;
    miss.rayOrigin = GfVec3d(40.0, 40.0, 2.0);
    for (const bool reproject : {false, true}) {
        miss.reproject = reproject;
        CHECK(CompareMatrix(RigExecRevisionOp::SurfaceProjector, binding,
                            RigExecSurfaceProjectorFrames(), miss, grid.rest,
                            grid.points, "miss", true));
        RigExecSurfaceProjectorFrames unprovided;
        unprovided.named[2] = true;
        CHECK(CompareMatrix(RigExecRevisionOp::SurfaceProjector, binding,
                            unprovided, miss, grid.rest, grid.points,
                            "miss, space not a provider", true));
    }
    // No usable surface, and a ray with no direction: both fail.
    CHECK(!CompareMatrix(RigExecRevisionOp::SurfaceProjector, binding,
                         RigExecSurfaceProjectorFrames(), reads, {}, {},
                         "no surface"));
    RigExecProjectorReads noDirection = reads;
    noDirection.rayDirection = GfVec3d(0.0, 0.0, 0.0);
    CHECK(!CompareMatrix(RigExecRevisionOp::SurfaceProjector, binding,
                         RigExecSurfaceProjectorFrames(), noDirection,
                         grid.rest, grid.points, "no direction"));

    RigExecProjectorReads dials;
    dials.dials = {0.5, -2.0, std::numeric_limits<double>::quiet_NaN(), 7.25,
                   1e300};
    CHECK(CompareMatrix(RigExecRevisionOp::ShaderDials, binding,
                        RigExecSurfaceProjectorFrames(), dials, {}, {},
                        "dials"));
    dials.dials.assign(20, 3.0);
    CompareMatrix(RigExecRevisionOp::ShaderDials, binding,
                  RigExecSurfaceProjectorFrames(), dials, {}, {},
                  "twenty dials");

    // A frame that resolved at compile and not now, or new ray settings,
    // needs a rebuild.
    RigExecSurfaceProjectorFrames frames;
    frames.named[0] = frames.resolved[0] = true;
    RigExecRigDescBuilder builder;
    RigExecGeometryMatrixNode node;
    std::string error;
    CHECK(RigExecAddGeometryMatrixOp(RigExecRevisionOp::SurfaceProjector,
                                     frames, reads, &builder, &node, &error));
    auto desc = builder.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    if (evaluator) {
        RigExecSurfaceProjectorFrames lost = frames;
        lost.resolved[0] = false;
        CHECK(!RigExecWriteGeometryMatrixInputs(binding, lost, reads, grid.rest,
                                                grid.points, node,
                                                evaluator.get(), &error));
        CHECK(error.find("resolution") != std::string::npos);
        RigExecProjectorReads moved = reads;
        moved.rayUp = GfVec3d(1.0, 0.0, 0.0);
        CHECK(!RigExecWriteGeometryMatrixInputs(binding, frames, moved,
                                                grid.rest, grid.points, node,
                                                evaluator.get(), &error));
        CHECK(error.find("settings") != std::string::npos);
    }
    CHECK(!RigExecAddGeometryMatrixOp(RigExecRevisionOp::Smooth, frames, reads,
                                      &builder, &node, &error));
}

/// One blend shape node over \p points, native against the op.
bool
CompareBlend(const std::vector<RigExecBlendChannel> &channels,
             RigExecMoverParameters p, const std::vector<GfVec3f> &base,
             const std::vector<GfVec3f> &points, const char *label)
{
    if (!RigExecGeometryBlendDeltas(channels, base, &p.blendDeltas))
        return false;
    std::vector<GfVec3f> expected = points;
    const bool native = Native(RigExecRevisionOp::BlendShape, p, &expected);

    RigExecRigDescBuilder builder;
    RigExecGeometryOpNode node;
    std::string error;
    CHECK(RigExecAddGeometryBlendOp(channels, p, &builder, nullptr, &node,
                                    &error));
    auto desc = builder.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    if (!evaluator)
        return false;
    const bool ran = RigExecWriteGeometryBlendInputs(channels, p, node, base,
                                                     &points, points.size(),
                                                     evaluator.get(), &error) &&
                     evaluator->Evaluate(&error);
    if (ran != native) {
        std::printf("%s: native %d op %d (%s)\n", label, int(native), int(ran),
                    error.c_str());
        CHECK(false);
        return false;
    }
    if (!ran)
        return false;
    const auto &actual = *static_cast<const RigExecArray<Vec3f> *>(
        evaluator->GetOutput(node.out));
    if (!SameBits(expected, actual)) {
        std::printf("%s differs\n", label);
        CHECK(false);
    }
    return true;
}

void
TestBlendShape(const Grid &grid)
{
    const size_t n = grid.rest.size();
    const auto moved = [&](float scale) {
        std::vector<GfVec3f> target = grid.rest;
        for (size_t i = 0; i < n; ++i)
            target[i] += GfVec3f(0.1f * std::sin(float(i) * scale),
                                 0.05f * scale, 0.2f * std::cos(float(i)));
        return target;
    };
    RigExecBlendSampleData half, full, sparse, denseLayout;
    half.activation = 0.5f;
    half.points = moved(0.7f);
    full.points = moved(1.3f);
    auto sparseLayout = std::make_shared<RigExecBlendSampleLayout>();
    for (size_t i = 3; i < n; i += 5) {
        sparseLayout->indices.push_back(int(i));
        sparseLayout->offsets.emplace_back(0.01f * float(i), -0.2f, 0.07f);
    }
    sparseLayout->pointCount = n;
    sparseLayout->valid = true;
    sparse.layout = sparseLayout;
    auto denseOffsets = std::make_shared<RigExecBlendSampleLayout>();
    for (size_t i = 0; i < n; ++i)
        denseOffsets->offsets.emplace_back(0.0f, 0.0f, 0.03f * float(i % 7));
    denseOffsets->pointCount = n;
    denseOffsets->valid = true;
    denseLayout.layout = denseOffsets;

    std::vector<RigExecBlendChannel> channels(3);
    channels[0].samples = {half, full};
    channels[1].samples = {sparse};
    channels[2].samples = {denseLayout};

    for (const bool surfaceFrame : {false, true})
        for (const auto &w : std::vector<std::array<float, 3>>{
                 {0.3f, 0.8f, -0.4f}, {0.75f, 1.0f, 1.0f}, {1.2f, 0.0f, 0.5f}})
            for (const auto &[name, weights] : Envelopes(n)) {
                for (size_t c = 0; c < 3; ++c)
                    channels[c].weight = w[c];
                RigExecMoverParameters p = Packet("blendShape");
                p.weights = weights;
                p.blendSurfaceFrame = surfaceFrame;
                if (surfaceFrame) {
                    p.restPoints = grid.rest;
                    p.topologyCounts = grid.counts;
                    p.topologyIndices = grid.indices;
                }
                const std::string label = std::string("blend ") +
                                          (surfaceFrame ? "surface " : "") +
                                          name;
                CHECK(CompareBlend(channels, p, grid.rest, grid.points,
                                   label.c_str()));
            }

    // A sample that moved after compile needs a rebuild.
    RigExecRigDescBuilder builder;
    RigExecGeometryOpNode node;
    std::string error;
    const RigExecMoverParameters p = Packet("blendShape");
    CHECK(RigExecAddGeometryBlendOp(channels, p, &builder, nullptr, &node,
                                    &error));
    auto desc = builder.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    if (!evaluator)
        return;
    std::vector<RigExecBlendChannel> changed = channels;
    changed[0].samples[1].points[4][0] += 1.0f;
    CHECK(!RigExecWriteGeometryBlendInputs(
        changed, p, node, grid.rest, &grid.points, n, evaluator.get(), &error));
    CHECK(error.find("samples") != std::string::npos);
    changed = channels;
    changed[1].samples[0].layout =
        std::make_shared<RigExecBlendSampleLayout>(*sparseLayout);
    CHECK(!RigExecWriteGeometryBlendInputs(
        changed, p, node, grid.rest, &grid.points, n, evaluator.get(), &error));
    changed.pop_back();
    CHECK(!RigExecWriteGeometryBlendInputs(
        changed, p, node, grid.rest, &grid.points, n, evaluator.get(), &error));
}

/// Smooth, then a matrix, as two connected nodes.
void
TestChain(const Grid &grid)
{
    RigExecMoverParameters smooth = Packet("smooth");
    smooth.topologyCounts = grid.counts;
    smooth.topologyIndices = grid.indices;
    smooth.strength = 0.5f;
    smooth.weights = Dense(grid.points.size());
    RigExecMoverParameters matrix = Packet("matrix");
    matrix.transform.SetTranslate(GfVec3d(1, 2, 3));
    matrix.weights = RigExecWeightPacket::Constant(0.5f);

    std::vector<GfVec3f> expected = grid.points;
    CHECK(Native(RigExecRevisionOp::Smooth, smooth, &expected));
    CHECK(Native(RigExecRevisionOp::Matrix, matrix, &expected));
    Chain chain({{RigExecRevisionOp::Smooth, &smooth},
                 {RigExecRevisionOp::Matrix, &matrix}});
    CHECK(chain.nodes.size() == 2 &&
          chain.nodes[1].points.index == kRigExecInvalidIndex);
    RigExecArray<Vec3f> actual;
    std::string error;
    CHECK(chain.Run({&smooth, &matrix}, grid.points, &actual, &error));
    CHECK(SameBits(expected, actual));
}

void
TestRefusals(const Grid &grid)
{
    // An envelope of another length or out of range fails both.
    RigExecMoverParameters p = Packet("volumeCorrect");
    p.referenceVolume = 1.0;
    p.strength = 1.0f;
    for (const bool shortField : {true, false}) {
        p.weights = Dense(grid.points.size());
        if (shortField)
            p.weights.values.pop_back();
        else
            p.weights.values[3] = 1.5f;
        std::vector<GfVec3f> native = grid.points;
        CHECK(!Native(RigExecRevisionOp::VolumeCorrect, p, &native));
        Chain chain({{RigExecRevisionOp::VolumeCorrect, &p}});
        RigExecArray<Vec3f> actual;
        std::string error;
        CHECK(!chain.Run({&p}, grid.points, &actual, &error));
    }

    // An unknown skinning method and an unmapped op are refused at build.
    RigExecRigDescBuilder builder;
    RigExecGeometryOpNode node;
    std::string error;
    RigExecMoverParameters skin = Packet("skin");
    skin.skinningMethod = TfToken("bogus");
    CHECK(!RigExecAddGeometryOp(RigExecRevisionOp::Skin, skin, &builder,
                                nullptr, &node, &error));
    CHECK(!RigExecGeometryOpSupported(RigExecRevisionOp::External));
    CHECK(!RigExecAddGeometryOp(RigExecRevisionOp::External, Packet("external"),
                                &builder, nullptr, &node, &error));

    // A setting baked into the payload that moves after compile refuses
    // the write; a per-frame value does not.
    RigExecMoverParameters mush = Packet("deltaMush");
    mush.restPoints = grid.rest;
    mush.topologyCounts = grid.counts;
    mush.topologyIndices = grid.indices;
    mush.mushIterations = 4;
    mush.mushStep = 0.5f;
    mush.weights = RigExecWeightPacket::Constant(1.0f);
    Chain mushChain({{RigExecRevisionOp::DeltaMush, &mush}});
    RigExecArray<Vec3f> out;
    CHECK(mushChain.Run({&mush}, grid.points, &out, &error));
    RigExecMoverParameters moved = mush;
    moved.mushComputationToTarget.SetTranslate(GfVec3d(0.0, 0.0, 1.0));
    CHECK(mushChain.Run({&moved}, grid.points, &out, &error));
    moved.mushIterations = 5;
    CHECK(!mushChain.Run({&moved}, grid.points, &out, &error));
    CHECK(error.find("settings") != std::string::npos);
}

// Every setting baked into a payload makes its node stale when it moves,
// with or without a reused scratch; a per-frame value does not.
void
TestStaleSettings(const Grid &grid)
{
    using Op = RigExecRevisionOp;
    using Edit = void (*)(RigExecMoverParameters &);
    struct Case {
        const char *label;
        Op op;
        const char *kind;
        Edit setup, moved, perFrame;
    };
    const Case cases[] = {
        {"matrix", Op::Matrix, "matrix", [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) { p.radialWeight = true; },
         [](RigExecMoverParameters &p) {
             p.transform.SetTranslate(GfVec3d(1, 0, 0));
         }},
        {"skin method", Op::Skin, "skin",
         [](RigExecMoverParameters &p) {
             p.skinningMethod = TfToken("classicLinear");
             p.skinElementSize = 2;
         },
         [](RigExecMoverParameters &p) {
             p.skinningMethod = TfToken("dualQuaternion");
         },
         [](RigExecMoverParameters &p) { p.skinTransforms.resize(3); }},
        {"skin unknown method", Op::Skin, "skin",
         [](RigExecMoverParameters &p) {
             p.skinningMethod = TfToken("classicLinear");
         },
         [](RigExecMoverParameters &p) { p.skinningMethod = TfToken("bogus"); },
         [](RigExecMoverParameters &p) { p.skinWeights = {1.0f}; }},
        {"skin element size", Op::Skin, "skin",
         [](RigExecMoverParameters &p) {
             p.skinningMethod = TfToken("classicLinear");
             p.skinElementSize = 2;
         },
         [](RigExecMoverParameters &p) { p.skinElementSize = 3; },
         [](RigExecMoverParameters &p) { p.skinIndices = {0, 1}; }},
        {"deltaMush step", Op::DeltaMush, "deltaMush",
         [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) { p.mushStep = 0.25f; },
         [](RigExecMoverParameters &p) { p.strength = 0.5f; }},
        {"deltaMush settings", Op::DeltaMush, "deltaMush",
         [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) { p.mushSettings.edges = {0, 1}; },
         [](RigExecMoverParameters &p) {
             p.mushComputationToTarget.SetTranslate(GfVec3d(0, 1, 0));
         }},
        {"wrinkle", Op::Wrinkle, "wrinkle", [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) {
             p.wrinkleSettings.bendStiffness = 0.3f;
         },
         [](RigExecMoverParameters &p) { p.restPoints.resize(2); }},
        {"lattice divisions", Op::Lattice, "lattice",
         [](RigExecMoverParameters &p) { p.divisions = GfVec3i(2, 3, 2); },
         [](RigExecMoverParameters &p) { p.divisions = GfVec3i(2, 2, 2); },
         [](RigExecMoverParameters &p) { p.auxPointsB.resize(12); }},
        {"lattice settings", Op::Lattice, "lattice",
         [](RigExecMoverParameters &p) { p.divisions = GfVec3i(2, 2, 2); },
         [](RigExecMoverParameters &p) { p.latticeSettings.strength = 0.5f; },
         [](RigExecMoverParameters &p) { p.auxPoints.resize(8); }},
        {"surfaceProject", Op::SurfaceProject, "surfaceProject",
         [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) { p.surfaceSettings.offset = 0.1f; },
         [](RigExecMoverParameters &p) {
             p.targetToSurface.SetTranslate(GfVec3d(0, 0, 1));
         }},
        {"wire knots", Op::Wire, "wire",
         [](RigExecMoverParameters &p) {
             p.curveOrder = 2;
             p.curveKnots = {0, 0, 1, 1};
         },
         [](RigExecMoverParameters &p) { p.curveKnots = {0, 0, 2, 2}; },
         [](RigExecMoverParameters &p) { p.auxPoints.resize(2); }},
        {"wire order", Op::Wire, "wire",
         [](RigExecMoverParameters &p) { p.curveOrder = 2; },
         [](RigExecMoverParameters &p) { p.curveOrder = 3; },
         [](RigExecMoverParameters &p) { p.restPoints.resize(2); }},
        {"wire dropoff", Op::Wire, "wire", [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) { p.dropoffDistance = 0.5; },
         [](RigExecMoverParameters &p) {
             p.weights = RigExecWeightPacket::Constant(0.5f);
         }},
        {"normals", Op::RecomputeNormals, "recomputeNormals",
         [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) { p.normalsLeftHanded = true; },
         [](RigExecMoverParameters &p) { p.auxPoints.resize(3); }},
        {"normals per corner", Op::RecomputeNormals, "recomputeNormals",
         [](RigExecMoverParameters &) {},
         [](RigExecMoverParameters &p) { p.normalsPerCorner = true; },
         [](RigExecMoverParameters &p) { p.topologyCounts = {3}; }},
    };
    RigExecGeometryPayloadScratchPtr scratch =
        RigExecMakeGeometryPayloadScratch();
    for (const Case &c : cases) {
        RigExecMoverParameters p = Packet(c.kind);
        c.setup(p);
        RigExecRigDescBuilder builder;
        RigExecGeometryOpNode node;
        std::string error;
        const bool added =
            RigExecAddGeometryOp(c.op, p, &builder, nullptr, &node, &error);
        CHECK(added);
        if (!added) {
            std::printf("%s: %s\n", c.label, error.c_str());
            continue;
        }
        RigExecMoverParameters moved = p, perFrame = p;
        c.moved(moved);
        c.perFrame(perFrame);
        for (RigExecGeometryPayloadScratch *s :
             {static_cast<RigExecGeometryPayloadScratch *>(nullptr),
              scratch.get()}) {
            const bool same =
                !RigExecGeometryOpNodeStale(c.op, p, nullptr, node, s);
            const bool stale =
                RigExecGeometryOpNodeStale(c.op, moved, nullptr, node, s);
            const bool kept =
                !RigExecGeometryOpNodeStale(c.op, perFrame, nullptr, node, s);
            CHECK(same && stale && kept);
            if (!(same && stale && kept))
                std::printf("%s: same %d stale %d kept %d\n", c.label,
                            int(same), int(stale), int(kept));
        }
    }

    // An op without a payload is never stale from its settings.
    RigExecMoverParameters volume = Packet("volumeCorrect");
    RigExecRigDescBuilder builder;
    RigExecGeometryOpNode node;
    CHECK(RigExecAddGeometryOp(Op::VolumeCorrect, volume, &builder, nullptr,
                               &node));
    CHECK(!node.payload);
    volume.strength = 0.25f;
    volume.referenceVolume = 2.0;
    CHECK(!RigExecGeometryOpNodeStale(Op::VolumeCorrect, volume, nullptr, node,
                                      scratch.get()));
    CHECK(RigExecGeometryOpNodeStale(Op::Smooth, volume, nullptr, node));

    // A blend shape's surface frame is a payload setting.
    std::vector<RigExecBlendChannel> channels(1);
    RigExecBlendSampleData sample;
    sample.points = grid.rest;
    channels[0].samples = {sample};
    RigExecMoverParameters blend = Packet("blendShape");
    RigExecGeometryOpNode blendNode;
    CHECK(RigExecAddGeometryBlendOp(channels, blend, &builder, nullptr,
                                    &blendNode));
    CHECK(!RigExecGeometryOpNodeStale(Op::BlendShape, blend, &channels,
                                      blendNode, scratch.get()));
    blend.blendSurfaceFrame = true;
    CHECK(RigExecGeometryOpNodeStale(Op::BlendShape, blend, &channels,
                                     blendNode, scratch.get()));
    CHECK(RigExecGeometryOpNodeStale(Op::BlendShape, blend, &channels,
                                     blendNode));
}

rigExecRig::PointWeights
ToPointWeights(const RigExecWeightPacket &w)
{
    using rigExecRig::WeightRepresentation;
    rigExecRig::PointWeights out;
    out.values = RigExecArray<float>(w.values.data(), w.values.size());
    out.indices = RigExecArray<int32_t>(w.indices.data(), w.indices.size());
    out.defaultWeight = w.defaultWeight;
    out.representation = uint32_t(w.representation == TfToken("dense")
                                      ? WeightRepresentation::Dense
                                  : w.representation == TfToken("sparse")
                                      ? WeightRepresentation::Sparse
                                      : WeightRepresentation::Constant);
    out.valid = w.valid ? 1 : 0;
    return out;
}

/// One CombineWeights node with a host-written list of \p count inputs, or
/// a second node whose list takes the first node's output between two host
/// inputs.
struct Combine {
    std::unique_ptr<RigExecEvaluator> evaluator;
    std::vector<RigExecInputId> items;
    RigExecInputId target, strength, invert;
    RigExecOutputId out;

    Combine(const rigExecOps::CombineWeightsPayload &payload, size_t count)
    {
        using namespace rigExecOps;
        RigExecRigDescBuilder b;
        const RigExecNodeId node = b.AddOp(CombineWeights(), &payload);
        for (size_t k = 0; k < count; ++k)
            items.push_back(b.BindInput(node, CombineWeightsInputs()));
        Finish(b, node);
    }
    Combine(const rigExecOps::CombineWeightsPayload &inner,
            const rigExecOps::CombineWeightsPayload &outer)
    {
        using namespace rigExecOps;
        RigExecRigDescBuilder b;
        const RigExecNodeId first = b.AddOp(CombineWeights(), &inner);
        const RigExecNodeId second = b.AddOp(CombineWeights(), &outer);
        items.push_back(b.BindInput(first, CombineWeightsInputs()));
        items.push_back(b.BindInput(first, CombineWeightsInputs()));
        items.push_back(b.BindInput(second, CombineWeightsInputs()));
        b.Connect(first, CombineWeightsOut(), second, CombineWeightsInputs());
        items.push_back(b.BindInput(second, CombineWeightsInputs()));
        Finish(b, second);
    }
    void Finish(RigExecRigDescBuilder &b, RigExecNodeId node)
    {
        using namespace rigExecOps;
        target = b.BindInput(node, CombineWeightsTargetCount());
        strength = b.BindInput(node, CombineWeightsStrength());
        invert = b.BindInput(node, CombineWeightsInvert());
        out = b.BindOutput(node, CombineWeightsOut());
        std::string error;
        const auto desc = b.Build(&error);
        const auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
        evaluator =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
        if (!evaluator)
            std::printf("combine: %s\n", error.c_str());
        CHECK(evaluator);
    }
    /// False where the op refused.
    bool Run(const std::vector<rigExecRig::PointWeights> &inputs,
             uint32_t targetCount, float s, float i,
             rigExecRig::PointWeights *result, std::string *error = nullptr)
    {
        if (!evaluator)
            return false;
        for (size_t k = 0; k < inputs.size(); ++k)
            CHECK(evaluator->SetInput(items[k], &inputs[k]));
        CHECK(evaluator->SetInput(target, &targetCount));
        CHECK(evaluator->SetInput(strength, &s));
        CHECK(evaluator->SetInput(invert, &i));
        if (!evaluator->Evaluate(error))
            return false;
        *result = *static_cast<const rigExecRig::PointWeights *>(
            evaluator->GetOutput(out));
        return true;
    }
};

bool
SameWeights(const std::vector<float> &a, const RigExecArray<float> &b)
{
    return a.size() == b.Size() &&
           (a.empty() || !std::memcmp(a.data(), b.Data(), a.size() * 4));
}

/// The op's outcome is the native packet: valid with the same bits, or,
/// where the native packet is invalid or bare (a range failure), an error of
/// the op's compile or evaluation.
bool
MatchesNative(const RigExecWeightPacket &native, bool ran,
              const rigExecRig::PointWeights &op, const std::string &error)
{
    if (native.representation.IsEmpty() || !native.valid)
        return !ran && error.find("CombineWeights: ") != std::string::npos;
    return ran && op.valid && SameWeights(native.values, op.values);
}

void
TestCombineWeights()
{
    using namespace rigExecOps;
    const size_t n = 23;
    const TfToken dense("dense"), sparse("sparse"), constant("constant"),
        strict("strict"), clamp("clamp");
    const auto make = [&](const TfToken &representation,
                          std::vector<float> values, std::vector<int> indices,
                          float defaultWeight, bool valid = true) {
        RigExecWeightPacket w;
        w.representation = representation;
        w.rangePolicy = strict;
        w.values = std::move(values);
        w.indices = std::move(indices);
        w.defaultWeight = defaultWeight;
        w.valid = valid;
        return w;
    };
    std::vector<float> a(n), b(n), negative(n, 0.5f);
    for (size_t i = 0; i < n; ++i) {
        a[i] = 0.07f * float(i % 13);
        b[i] = 1.0f - 0.043f * float((i * 7) % 19);
    }
    negative[4] = -0.25f;
    const RigExecWeightPacket A = make(dense, a, {}, 0.0f);
    const RigExecWeightPacket B = make(dense, b, {}, 0.0f);
    const RigExecWeightPacket S =
        make(sparse, {0.9f, 0.3f, 0.65f, 0.1f}, {1, 4, 9, 22}, 0.35f);
    const RigExecWeightPacket U =
        make(sparse, {0.2f, 0.8f, 0.5f}, {17, 2, 30}, 0.6f);
    const RigExecWeightPacket C = make(constant, {}, {}, 0.45f);
    const RigExecWeightPacket I = make(dense, a, {}, 0.0f, false);
    const RigExecWeightPacket M =
        make(dense, std::vector<float>(n + 1, 0.5f), {}, 0.0f);
    const RigExecWeightPacket N = make(dense, negative, {}, 0.0f);
    const std::vector<std::vector<const RigExecWeightPacket *>> cases = {
        {&A, &B},
        {&B, &A},
        {&A, &S, &C},
        {&S, &C},
        {&C, &S, &U},
        {&A},
        {&C},
        {},
        {&A, &I},
        {&A, &M},
        {&N, &A},
        {&U, &A, &S},
        {&S, &B, &A, &C, &U}};
    const std::pair<const char *, WeightCombineMode> modes[] = {
        {"multiply", WeightCombineMode::Multiply},
        {"add", WeightCombineMode::Add},
        {"subtract", WeightCombineMode::Subtract},
        {"max", WeightCombineMode::Max},
        {"min", WeightCombineMode::Min},
        {"average", WeightCombineMode::Average},
        {"overlay", WeightCombineMode::Overlay}};
    const std::pair<float, float> scales[] = {{1.0f, 0.0f}, {0.8f, 0.25f}};
    rigExecRig::PointWeights result;
    for (const auto &[token, mode] : modes)
        for (const bool clamped : {false, true}) {
            CombineWeightsPayload payload;
            payload.mode = mode;
            payload.clamp = clamped;
            for (const auto &inputs : cases)
                for (const uint32_t target : {uint32_t(0), uint32_t(n)})
                    for (const auto &[s, i] : scales) {
                        const RigExecWeightPacket native =
                            RigExecBuildCombineWeightPacket(
                                dense, clamped ? clamp : strict, TfToken(token),
                                inputs, target, s, i);
                        std::vector<rigExecRig::PointWeights> packed;
                        for (const RigExecWeightPacket *w : inputs)
                            packed.push_back(ToPointWeights(*w));
                        Combine node(payload, inputs.size());
                        std::string error;
                        const bool ran =
                            node.Run(packed, target, s, i, &result, &error);
                        CHECK(MatchesNative(native, ran, result, error));
                        if (!MatchesNative(native, ran, result, error))
                            std::printf("combine %s %zu inputs differs: %s\n",
                                        token, inputs.size(), error.c_str());
                    }
        }

    // Order matters: subtract of (A, B) is not of (B, A).
    CombineWeightsPayload subtract;
    subtract.mode = WeightCombineMode::Subtract;
    subtract.clamp = true;
    Combine pair(subtract, 2);
    rigExecRig::PointWeights ab, ba;
    CHECK(pair.Run({ToPointWeights(A), ToPointWeights(B)}, 0, 1.0f, 0.0f, &ab));
    CHECK(pair.Run({ToPointWeights(B), ToPointWeights(A)}, 0, 1.0f, 0.0f, &ba));
    CHECK(ab.valid && ba.valid &&
          std::memcmp(ab.values.Data(), ba.values.Data(), n * 4) != 0);

    // An upstream combine feeds the middle of a list.
    CombineWeightsPayload overlay;
    overlay.mode = WeightCombineMode::Overlay;
    overlay.clamp = true;
    Combine chained(subtract, overlay);
    const RigExecWeightPacket inner = RigExecBuildCombineWeightPacket(
        dense, clamp, TfToken("subtract"),
        std::vector<const RigExecWeightPacket *>{&B, &S}, n, 1.0f, 0.0f);
    const RigExecWeightPacket outer = RigExecBuildCombineWeightPacket(
        dense, clamp, TfToken("overlay"),
        std::vector<const RigExecWeightPacket *>{&C, &inner, &A}, n, 0.9f,
        0.1f);
    CHECK(inner.valid && outer.valid);
    CHECK(chained.Run({ToPointWeights(B), ToPointWeights(S), ToPointWeights(C),
                       ToPointWeights(A)},
                      uint32_t(n), 0.9f, 0.1f, &result));
    CHECK(MatchesNative(outer, true, result, std::string()));

    // Fields: folded as they are over the target count, as the field
    // resolver folds them, with no range policy.
    CombineWeightsPayload fields;
    fields.inputs = WeightCombineInputs::Fields;
    for (const auto &[token, mode] : modes) {
        fields.mode = mode;
        Combine node(fields, 3);
        const std::vector<float> *sources[] = {&b, &a, &negative};
        std::vector<float> expected;
        CHECK(RigExecCombineWeightFieldsStreamed(
            RigExecWeightCombine(int(mode)), 3, n,
            [&](size_t k, const float **data, size_t *size) {
                *data = sources[k]->data();
                *size = sources[k]->size();
                return true;
            },
            &expected));
        for (float &w : expected)
            w = (w + (1.0f - 2.0f * w) * 0.3f) * 1.7f;
        CHECK(
            node.Run({ToPointWeights(B), ToPointWeights(A), ToPointWeights(N)},
                     uint32_t(n), 1.7f, 0.3f, &result));
        CHECK(result.valid && SameWeights(expected, result.values));
        std::string error;
        CHECK(
            !node.Run({ToPointWeights(B), ToPointWeights(A), ToPointWeights(M)},
                      uint32_t(n), 1.0f, 0.0f, &result, &error));
        CHECK(error.find("disagree on element count") != std::string::npos);
    }
    Combine none(fields, 0);
    CHECK(none.Run({}, uint32_t(n), 1.0f, 0.0f, &result));
    CHECK(result.valid && result.values.Size() == n);
}

// Each revision op's registered op carries, as capability bits, the answer
// every evaluator list still gives for it.
void
TestCapabilityFlags()
{
    using Op = RigExecRevisionOp;
    const Op ops[] = {Op::Matrix,
                      Op::Skin,
                      Op::BlendShape,
                      Op::VolumeCorrect,
                      Op::Smooth,
                      Op::Lattice,
                      Op::SurfaceProject,
                      Op::Ribbon,
                      Op::Wire,
                      Op::EmitGuidePoints,
                      Op::RecomputeNormals,
                      Op::RecomputeExtent,
                      Op::DeltaMush,
                      Op::Wrinkle,
                      Op::External,
                      Op::SurfaceProjector,
                      Op::ShaderDials};
    RigExecMoverParameters gated;
    gated.weights.valid = true;
    gated.weights.representation = TfToken("sparse");
    gated.weights.defaultWeight = 0.0f;
    gated.skinningMethod = TfToken("classicLinear");
    const RigExecRegistry &registry = RigExecRegistry::Get();
    int mapped = 0;
    for (const Op op : ops) {
        const RigExecOpHandle *handle = RigExecGeometryOpHandle(op);
        if (!handle) {
            CHECK(op == Op::External);
            continue;
        }
        ++mapped;
        CHECK(handle->IsValid());
        const uint32_t caps = registry.Op(*handle).desc->capabilities;
        const auto has = [caps](uint32_t bit) { return (caps & bit) != 0; };
        const bool derived = op == Op::RecomputeNormals ||
                             op == Op::RecomputeExtent ||
                             RigExecIsDerivedMatrixOp(op);
        CHECK(RigExecRevisionTakesSeparateBlend(op, RigExecWeightPacket()) ==
              has(kRigExecOpSeparateBlend));
        // A whole member runs through range handling over one range.
        const bool whole = op == Op::VolumeCorrect || op == Op::Wrinkle;
        CHECK((RigExecRevisionIsRangeOp(op) || whole) == has(kRigExecOpRanged));
        CHECK(RigExecRevisionGateHolds(op, gated) == has(kRigExecOpGate));
        CHECK(RigExecRevisionOpAssemblesFromLeaves(op) ==
              has(kRigExecOpFromLeaves));
        CHECK(RigExecRevisionOpBakes(op, derived) == has(kRigExecOpBakeable));
        CHECK(RigExecRevisionOpFreezes(op, derived) ==
              has(kRigExecOpFreezable));
        CHECK(RigExecGeometryOpSupported(op) !=
              (op == Op::BlendShape || RigExecIsDerivedMatrixOp(op)));
        CHECK(RigExecGeometryMatrixOpSupported(op) ==
              RigExecIsDerivedMatrixOp(op));
    }
    CHECK(mapped == 16);
    // A sparse walk folds the wire's envelope into its own motion.
    CHECK(!RigExecRevisionTakesSeparateBlend(Op::Wire, gated.weights));
    CHECK(!RigExecGeometryOpSupported(Op::External) &&
          !RigExecGeometryMatrixOpSupported(Op::External));
    for (size_t k = 0; k < registry.OpCount(); ++k) {
        const uint32_t caps = registry.OpAt(k).desc->capabilities;
        CHECK((caps & kRigExecOpBakeable) && (caps & kRigExecOpFreezable));
    }
}

} // namespace

int
main()
{
    std::string registryError;
    if (!RigExecEnsureBuiltinRegistry(&registryError)) {
        std::printf("%s\n", registryError.c_str());
        return 1;
    }
    const Grid grid(9, 7);
    for (const bool simd : {true, false}) {
        g_useSimd = simd;
        TestMatrix(grid);
        TestSkin(grid);
        TestPointOps(grid);
        TestWrinkle(grid);
        TestLattice(grid);
        TestSurfaceProject(grid);
        TestFrameDriven(grid);
        TestWire(grid);
        TestDerived(grid);
        TestProjector(grid);
        TestBlendShape(grid);
        TestChain(grid);
        TestRefusals(grid);
        TestStaleSettings(grid);
    }
    TestCombineWeights();
    TestCapabilityFlags();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
