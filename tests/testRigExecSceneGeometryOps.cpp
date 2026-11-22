// A geometry revision run as rigExecOps' deform op gives the bits
// RigExecRunGeometry gives: per op, at full strength, under a constant and
// a dense envelope, alone and chained, and both refuse the same envelopes.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExecScene/geometryOps.h"
#include "rigExecScene/geometryProgram.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/gf/rotation.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

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

/// The envelopes each op is run under.
std::vector<std::pair<const char *, RigExecWeightPacket>>
Envelopes(size_t count)
{
    return {{"full", RigExecWeightPacket::Constant(1.0f)},
            {"constant", RigExecWeightPacket::Constant(0.37f)},
            {"dense", Dense(count)},
            {"narrow", Narrow(count)}};
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
        auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
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

/// The native answer over \p points.
bool
Native(RigExecRevisionOp op, const RigExecMoverParameters &p,
       std::vector<GfVec3f> *points)
{
    RigExecGeometryWorkspace workspace;
    return RigExecRunGeometry(op, p, points, /*useSimd=*/true, &workspace.wire,
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
    const Grid grid(9, 7);
    TestMatrix(grid);
    TestSkin(grid);
    TestPointOps(grid);
    TestChain(grid);
    TestRefusals(grid);
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
