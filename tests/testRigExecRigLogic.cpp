// RigLogic's behaviours in a rig, through every evaluator: the pose
// interpolator's RigLogic solves (additive and interpolative, several frame
// drivers, numeric drivers), the ML mover (a network over a frame driver
// and a dial, its outputs and the sparse morphs they drive, under a keyed
// mask) and animated maps (conditional tables over double and float inputs,
// their mesh primvars and a morph connected to one), each one against the
// shared kernel driven directly, against the frame-cache worker, and against
// the .rigexec runtime played over the same timeline, bit for bit.
#include "rigExec/rigEvaluator.h"
#include "rigExecMath/conditionalTableKernel.h"
#include "rigExecMath/mlpKernel.h"
#include "rigExecMath/rbfMethodKernel.h"
#include "rigExecRigging/rigBuilder.h"

#include "pxr/base/gf/quatf.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"

namespace {

const SdfPath kRig("/Asset/Rig");
const SdfPath kShoulder("/Asset/Rig/Controls/Shoulder");
const SdfPath kElbow("/Asset/Rig/Controls/Shoulder/Elbow");
const SdfPath kWrist("/Asset/Rig/Controls/Wrist");
const SdfPath kDial("/Asset/Rig/Controls/Dial");
const SdfPath kInterpolator("/Asset/Rig/Poses/Arm");

GfQuatf
About(const GfVec3d &axis, double degrees)
{
    const GfQuatd q = GfRotation(axis, degrees).GetQuat();
    return GfQuatf(float(q.GetReal()), GfVec3f(q.GetImaginary()));
}

// (x, y, z, w) on the w >= 0 hemisphere, the row layout the solve reads.
void
Push(std::vector<double> *row, const GfQuatf &q)
{
    const double s = q.GetReal() < 0 ? -1.0 : 1.0;
    const GfVec3f &v = q.GetImaginary();
    row->insert(row->end(), {s * v[0], s * v[1], s * v[2], s * q.GetReal()});
}

UsdStageRefPtr
MakeStage()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(kRig, TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls"), TfToken("Scope"));
    for (const SdfPath &control : {kShoulder, kElbow, kWrist, kDial}) {
        stage->DefinePrim(control, TfToken("RigExecControl"));
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Poses"), TfToken("Scope"));
    return stage;
}

struct Case {
    const char *solver;
    const char *distance;
    const char *kernel;
    bool numeric;
};

// Two frame drivers (the elbow under the rotated shoulder, so its delta is
// local, and the wrist at the root), or the dial's two numeric channels.
std::vector<SdfPath>
AuthorInterpolator(const UsdStageRefPtr &stage, const Case &c,
                   RigExecRbfMethodDesc *desc)
{
    auto builder = RigExecRigBuilder::Create(stage, kRig);
    UsdPrim prim = stage->DefinePrim(kInterpolator,
                                     TfToken("RigExecPoseInterpolator"));
    RigExecPoseInterpolatorHandle interpolator(stage, kInterpolator);
    interpolator.SetSolver(TfToken(c.solver));
    interpolator.SetKernel(TfToken(c.kernel));
    interpolator.SetDistanceMethod(TfToken(c.distance));
    interpolator.SetTwistAxis(TfToken("Y"));
    interpolator.SetRadius(c.numeric ? 2.0f : 1.1f, false);
    interpolator.SetNormalizeMethod(TfToken("always"));
    interpolator.SetWeightThreshold(0.01f);
    if (c.numeric) {
        interpolator.SetDriverAttributes(
            {kDial.AppendProperty(TfToken("avars:tx")),
             kDial.AppendProperty(TfToken("avars:ty"))});
    } else {
        interpolator.SetDrivers({kElbow, kWrist});
    }
    desc->solverType = std::string(c.solver) == "interpolative"
                           ? RigExecRbfSolverType::Interpolative
                           : RigExecRbfSolverType::Additive;
    desc->distanceMethod =
        std::string(c.distance) == "euclidean" ? RigExecRbfDistanceMethod::Euclidean :
        std::string(c.distance) == "swingAngle" ? RigExecRbfDistanceMethod::SwingAngle :
        std::string(c.distance) == "twistAngle" ? RigExecRbfDistanceMethod::TwistAngle :
        RigExecRbfDistanceMethod::Quaternion;
    desc->weightFunction =
        std::string(c.kernel) == "linear" ? RigExecRbfWeightFunction::Linear :
        std::string(c.kernel) == "cubic" ? RigExecRbfWeightFunction::Cubic :
        RigExecRbfWeightFunction::Gaussian;
    desc->normalizeMethod = RigExecRbfNormalizeMethod::Always;
    desc->twistAxis = 1;
    desc->radius = c.numeric ? 2.0f : 1.1f;
    desc->weightThreshold = 0.01f;
    desc->rawCount = c.numeric ? 2 : 8;
    struct PoseSpec {
        const char *name;
        GfQuatf elbow, wrist;
        GfVec3f values;
        float scale;
    };
    const PoseSpec poses[] = {
        {"neutral", About({0, 1, 0}, 0), About({1, 0, 0}, 0), {0, 0, 0}, 1.0f},
        {"bend", About({0, 0.3, 1}, 70), About({1, 0.2, 0}, 15), {3, 0, 0}, 1.0f},
        {"twist", About({0.3, 1, 0}, 80), About({0, 1, 0.4}, -40), {0, 2, 0}, 0.5f},
        {"both", About({1, 0, 1}, 60), About({1, 1, 0}, 35), {2, 2, 0}, 1.5f},
    };
    std::vector<SdfPath> weights;
    for (const PoseSpec &spec : poses) {
        RigExecPoseHandle pose = interpolator.AddPose(spec.name);
        pose.SetRotations({spec.elbow, spec.wrist});
        pose.SetTranslation(spec.values);
        pose.SetScale(spec.scale);
        if (c.numeric) {
            desc->targets.insert(desc->targets.end(),
                                 {double(spec.values[0]), double(spec.values[1])});
        } else {
            Push(&desc->targets, spec.elbow);
            Push(&desc->targets, spec.wrist);
        }
        desc->targetScales.push_back(spec.scale);
        weights.push_back(pose.GetWeightOutput());
    }
    (void)prim;
    return weights;
}

void
SetAvar(const UsdStageRefPtr &stage, const SdfPath &control, const char *name,
        double value, UsdTimeCode time = UsdTimeCode::Default())
{
    UsdAttribute attribute =
        stage->GetPrimAtPath(control).GetAttribute(TfToken(name));
    if (!attribute) {
        attribute = stage->GetPrimAtPath(control).CreateAttribute(
            TfToken(name), SdfValueTypeNames->Double);
    }
    attribute.Set(value, time);
}

double
Published(const RigExecRigPose &pose, const SdfPath &weight)
{
    const auto found = pose.movedProperties.find(weight);
    if (found == pose.movedProperties.end() ||
        !found->second.IsHolding<float>()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return found->second.UncheckedGet<float>();
}

// Each driver's avars at each of four frames: rotations for frame drivers
// (degrees), translations for the dial.
struct Key {
    double shoulderZ, elbowZ, elbowY, wristX, dialX, dialY;
};
const Key kKeys[] = {
    {0, 0, 0, 0, 0, 0},
    {25, 55, 10, -5, 2.5, 0.4},
    {-30, 10, 70, -35, 0.5, 1.7},
    {10, 40, 40, 20, 1.8, 1.9},
};

void
KeyStage(const UsdStageRefPtr &stage)
{
    for (size_t f = 0; f < 4; ++f) {
        const UsdTimeCode t(double(f + 1));
        const Key &k = kKeys[f];
        SetAvar(stage, kShoulder, "avars:rz", k.shoulderZ, t);
        SetAvar(stage, kElbow, "avars:rz", k.elbowZ, t);
        SetAvar(stage, kElbow, "avars:ry", k.elbowY, t);
        SetAvar(stage, kWrist, "avars:rx", k.wristX, t);
        SetAvar(stage, kDial, "avars:tx", k.dialX, t);
        SetAvar(stage, kDial, "avars:ty", k.dialY, t);
    }
}

// The row the evaluator should hand the solve at frame f: the elbow's
// rotation relative to its own rest under the shoulder (the shoulder's
// rotation cancels, which is what makes it LOCAL), then the wrist's.
std::vector<double>
ExpectedRow(const Case &c, size_t f)
{
    const Key &k = kKeys[f];
    std::vector<double> row;
    if (c.numeric) {
        return {k.dialX, k.dialY};
    }
    // Avars compose XYZ, Ry * Rz in USD's row-vector order, and
    // M(q1 * q2) = M(q2) * M(q1), so the quaternion is qz * qy.
    const GfQuatd elbow = GfRotation(GfVec3d(0, 0, 1), k.elbowZ).GetQuat() *
                          GfRotation(GfVec3d(0, 1, 0), k.elbowY).GetQuat();
    Push(&row, GfQuatf(float(elbow.GetReal()), GfVec3f(elbow.GetImaginary())));
    Push(&row, About({1, 0, 0}, k.wristX));
    return row;
}

void
TestCase(const Case &c)
{
    const std::string label = std::string(c.solver) + "/" + c.distance + "/" +
                              c.kernel + (c.numeric ? "/numeric" : "/frames");
    UsdStageRefPtr stage = MakeStage();
    RigExecRbfMethodDesc desc;
    const std::vector<SdfPath> weights = AuthorInterpolator(stage, c, &desc);
    KeyStage(stage);
    RigExecRbfMethodTable table;
    CHECK(RigExecRbfMethodBuild(desc, &table));

    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        ++failures;
        for (const auto &e : errors) std::printf("FAIL %s compile: %s\n", label.c_str(), e.c_str());
        return;
    }
    CHECK(evaluator.GetSkippedOperations().empty());
    std::vector<std::string> reasons;
    CHECK(evaluator.IsBakeable(&reasons));

    // Live against the kernel driven directly. The live row comes from
    // composed frames, so it agrees with the hand-built one to float
    // rounding of the frames, not bit for bit.
    std::vector<RigExecRigPose> live;
    for (size_t f = 0; f < 4; ++f) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f + 1)));
        CHECK(pose.valid);
        live.push_back(pose);
        const std::vector<double> row = ExpectedRow(c, f);
        std::vector<double> scratch(RigExecRbfMethodScratchSize(table));
        std::vector<double> expected(table.targetCount);
        RigExecRbfMethodEvaluate(table, row.data(), scratch.data(), expected.data());
        bool any = false;
        for (size_t p = 0; p < weights.size(); ++p) {
            const double got = Published(pose, weights[p]);
            any = any || got > 0.0;
            if (!(std::abs(got - expected[p]) <= 2e-5)) {
                ++failures;
                std::printf("FAIL %s frame %zu pose %zu: %.8f, kernel %.8f\n",
                            label.c_str(), f + 1, p, got, expected[p]);
            }
        }
        CHECK(any);
    }
    // At rest an interpolative solve stands exactly on its neutral pose.
    if (desc.solverType == RigExecRbfSolverType::Interpolative &&
        !(std::abs(Published(live[0], weights[0]) - 1.0) < 1e-6)) {
        ++failures;
        std::printf("FAIL %s: neutral reads %.8f at rest\n", label.c_str(),
                    Published(live[0], weights[0]));
    }

    // The frame-cache worker reproduces live, bit for bit.
    for (size_t f = 0; f < 4; ++f) {
        std::string why;
        if (!RigExecFrozenMatchesLive(&evaluator, kRig,
                                      UsdTimeCode(double(f + 1)), &why)) {
            ++failures;
            std::printf("FAIL %s frozen frame %zu: %s\n", label.c_str(), f + 1,
                        why.c_str());
        }
    }

    // The .rigexec runtime, played over the stage's timeline.
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecTestBakeAt(evaluator, 1.0, &bytes, &error)) {
        ++failures;
        std::printf("FAIL %s bake: %s\n", label.c_str(), error.c_str());
        return;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        ++failures;
        std::printf("FAIL %s open: %s\n", label.c_str(), error.c_str());
        return;
    }
    for (size_t f : {2u, 0u, 3u, 1u, 1u}) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f + 1)));
        if (!player.Play(double(f + 1), &error)) {
            ++failures;
            std::printf("FAIL %s play: %s\n", label.c_str(), error.c_str());
            return;
        }
        std::vector<std::string> diffs;
        if (!RigExecCompareRuntimeRun(pose, player.Reader(), &diffs)) {
            ++failures;
            std::printf("FAIL %s runtime frame %zu:\n", label.c_str(), f + 1);
            for (const auto &d : diffs) std::printf("    %s\n", d.c_str());
        }
    }

    // A solve setting is epoch identity: an edit recompiles and changes the
    // weights, and evaluation never authors.
    const RigExecRigPose beforePose = evaluator.Evaluate(UsdTimeCode(3.0));
    RigExecPoseInterpolatorHandle(stage, kInterpolator).SetRadius(
        c.numeric ? 0.9f : 0.5f);
    const RigExecRigPose afterPose = evaluator.Evaluate(UsdTimeCode(3.0));
    bool changed = false;
    for (const SdfPath &weight : weights) {
        changed = changed || Published(beforePose, weight) != Published(afterPose, weight);
        CHECK(std::isfinite(Published(afterPose, weight)));
    }
    if (!changed) {
        ++failures;
        std::printf("FAIL %s: a radius edit left every weight as it was\n",
                    label.c_str());
    }
}

// The runtime and the frame cache against live at every frame, in a
// shuffled order with a repeat.
void
CheckEvaluators(RigExecRigEvaluator &evaluator, const UsdStageRefPtr &stage,
                const std::string &label)
{
    for (size_t f = 0; f < 4; ++f) {
        std::string why;
        if (!RigExecFrozenMatchesLive(&evaluator, kRig,
                                      UsdTimeCode(double(f + 1)), &why)) {
            ++failures;
            std::printf("FAIL %s frozen frame %zu: %s\n", label.c_str(), f + 1,
                        why.c_str());
        }
    }
    std::vector<uint8_t> bytes;
    std::string error;
    if (!RigExecTestBakeAt(evaluator, 1.0, &bytes, &error)) {
        ++failures;
        std::printf("FAIL %s bake: %s\n", label.c_str(), error.c_str());
        return;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, stage, &error)) {
        ++failures;
        std::printf("FAIL %s open: %s\n", label.c_str(), error.c_str());
        return;
    }
    for (size_t f : {2u, 0u, 3u, 1u, 1u}) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f + 1)));
        if (!player.Play(double(f + 1), &error)) {
            ++failures;
            std::printf("FAIL %s play: %s\n", label.c_str(), error.c_str());
            return;
        }
        std::vector<std::string> diffs;
        if (!RigExecCompareRuntimeRun(pose, player.Reader(), &diffs)) {
            ++failures;
            std::printf("FAIL %s runtime frame %zu:\n", label.c_str(), f + 1);
            for (const auto &d : diffs) std::printf("    %s\n", d.c_str());
        }
    }
}

const SdfPath kMesh("/Asset/Geom/Face");

// Six points, two sparse morphs, and an ML mover with five inputs (the
// elbow's rotation and the dial's tx) through a tanh layer of four to two
// sigmoid outputs, each driving one morph.
struct MlRig {
    UsdStageRefPtr stage;
    RigExecMlNetwork network;
    SdfPath mover;
    std::vector<SdfPath> outputs;
    VtVec3fArray base;
    std::vector<VtIntArray> indices;
    std::vector<VtVec3fArray> offsets;
};

MlRig
MakeMlRig()
{
    MlRig rig;
    rig.stage = MakeStage();
    const UsdStageRefPtr &stage = rig.stage;
    for (int i = 0; i < 6; ++i) rig.base.push_back(GfVec3f(float(i), 0.5f * i, 0.0f));
    UsdPrim mesh = stage->DefinePrim(kMesh, TfToken("Mesh"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray).Set(rig.base);
    rig.indices = {VtIntArray{0, 2, 5}, VtIntArray{1, 2}};
    rig.offsets = {VtVec3fArray{GfVec3f(0, 1, 0), GfVec3f(0.5f, 0, 0.25f), GfVec3f(0, 0, 2)},
                   VtVec3fArray{GfVec3f(-1, 0, 0), GfVec3f(0, 0.75f, -0.5f)}};
    std::vector<SdfPath> shapes;
    for (size_t k = 0; k < 2; ++k) {
        const SdfPath path("/Asset/Geom/Shape" + std::to_string(k));
        UsdPrim shape = stage->DefinePrim(path, TfToken("BlendShape"));
        shape.CreateAttribute(TfToken("offsets"), SdfValueTypeNames->Vector3fArray)
            .Set(rig.offsets[k]);
        shape.CreateAttribute(TfToken("pointIndices"), SdfValueTypeNames->IntArray)
            .Set(rig.indices[k]);
        shapes.push_back(path);
    }
    auto builder = RigExecRigBuilder::Create(stage, kRig);
    RigExecMlMoverHandle ml =
        builder.NewMoverChain("Face", kMesh.AppendProperty(TfToken("points")))
            .AddMlMover("Ml");
    rig.mover = ml.GetPath();
    ml.SetDrivers({kElbow});
    ml.SetDriverAttributes({kDial.AppendProperty(TfToken("avars:tx"))});
    const std::vector<float> weights = {
        // layer 1: 4 x 5
        0.8f, -0.3f, 0.5f, 0.2f, 0.4f,   -0.6f, 0.9f, 0.1f, -0.2f, 0.3f,
        0.3f, 0.3f, -0.7f, 0.5f, -0.5f,  0.1f, -0.4f, 0.6f, 0.9f, 0.2f,
        // layer 2: 2 x 4
        1.2f, -0.8f, 0.5f, 0.3f,  -0.5f, 1.1f, 0.7f, -0.9f};
    const std::vector<float> biases = {0.1f, -0.2f, 0.05f, 0.0f, -0.3f, 0.2f};
    ml.SetNetwork({4, 2}, {TfToken("tanh"), TfToken("sigmoid")}, {}, weights, biases);
    ml.SetInputNormalization({0, 0, 0, 0.9f, 1.0f}, {2, 2, 2, 3, 0.5f});
    ml.SetOutputDefaults({0.0f, 0.25f});
    std::string error;
    CHECK(RigExecMlBuildNetwork(5, {4, 2}, {3, 4}, {}, weights, biases,
                                &rig.network, &error));
    rig.network.inputMean = {0, 0, 0, 0.9f, 1.0f};
    rig.network.inputScale = {2, 2, 2, 3, 0.5f};
    rig.network.defaults = {0.0f, 0.25f};
    for (size_t k = 0; k < 2; ++k) {
        rig.outputs.push_back(ml.AddMorph("out" + std::to_string(k), shapes[k]));
    }
    // A keyed mask: on, half, off (the defaults), on.
    const double masks[4] = {1.0, 0.5, 0.0, 1.0};
    for (size_t f = 0; f < 4; ++f) {
        ml.GetPrim().GetAttribute(TfToken("inputs:mask"))
            .Set(float(masks[f]), UsdTimeCode(double(f + 1)));
    }
    KeyStage(stage);
    return rig;
}

void
TestMlMover()
{
    MlRig rig = MakeMlRig();
    RigExecRigEvaluator evaluator(rig.stage, kRig);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        ++failures;
        for (const auto &e : errors) std::printf("FAIL ml compile: %s\n", e.c_str());
        return;
    }
    CHECK(evaluator.GetSkippedOperations().empty());
    std::vector<std::string> reasons;
    CHECK(evaluator.IsBakeable(&reasons));
    const SdfPath points = kMesh.AppendProperty(TfToken("points"));
    const double masks[4] = {1.0, 0.5, 0.0, 1.0};
    for (size_t f = 0; f < 4; ++f) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f + 1)));
        CHECK(pose.valid);
        // The network on the row the evaluator should build: the elbow's
        // local rotation from rest, then the dial.
        std::vector<double> row = ExpectedRow({"", "", "", false}, f);
        row.resize(4);
        row.push_back(kKeys[f].dialX);
        std::vector<double> scratch(RigExecMlScratchSize(rig.network)), out(2);
        RigExecMlEvaluate(rig.network, row.data(), masks[f], scratch.data(), out.data());
        for (size_t k = 0; k < 2; ++k) {
            const double got = Published(pose, rig.outputs[k]);
            if (!(std::abs(got - out[k]) <= 2e-6)) {
                ++failures;
                std::printf("FAIL ml frame %zu output %zu: %.8f, kernel %.8f\n",
                            f + 1, k, got, out[k]);
            }
        }
        if (masks[f] == 0.0) {
            CHECK(Published(pose, rig.outputs[0]) == 0.0 &&
                  Published(pose, rig.outputs[1]) == 0.25);
        }
        // The morphs, sparse offsets weighted by the published outputs.
        VtVec3fArray expected = rig.base;
        for (size_t k = 0; k < 2; ++k) {
            const double w = Published(pose, rig.outputs[k]);
            for (size_t e = 0; e < rig.indices[k].size(); ++e) {
                expected[size_t(rig.indices[k][e])] += GfVec3f(rig.offsets[k][e] * float(w));
            }
        }
        const auto found = pose.movedProperties.find(points);
        CHECK(found != pose.movedProperties.end() &&
              found->second.IsHolding<VtVec3fArray>());
        if (found != pose.movedProperties.end() && found->second.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &moved = found->second.UncheckedGet<VtVec3fArray>();
            CHECK(moved.size() == expected.size());
            for (size_t i = 0; i < moved.size() && i < expected.size(); ++i) {
                if ((moved[i] - expected[i]).GetLength() > 1e-5f) {
                    ++failures;
                    std::printf("FAIL ml frame %zu point %zu\n", f + 1, i);
                }
            }
        }
    }
    CheckEvaluators(evaluator, rig.stage, "ml");

    // The MoverAPI envelope scales the morphs, not the published outputs.
    const RigExecRigPose full = evaluator.Evaluate(UsdTimeCode(2.0));
    RigExecMoverHandle(rig.stage, rig.mover).SetDefaultWeight(0.5f);
    const RigExecRigPose half = evaluator.Evaluate(UsdTimeCode(2.0));
    CHECK(Published(half, rig.outputs[0]) == Published(full, rig.outputs[0]));
    const auto pointsOf = [&](const RigExecRigPose &pose) {
        const auto found = pose.movedProperties.find(points);
        return found == pose.movedProperties.end() ? VtVec3fArray()
                                                    : found->second.Get<VtVec3fArray>();
    };
    const VtVec3fArray fullPoints = pointsOf(full), halfPoints = pointsOf(half);
    CHECK(fullPoints.size() == 6 && halfPoints.size() == 6);
    for (size_t i = 0; i < fullPoints.size() && i < halfPoints.size(); ++i) {
        const GfVec3f want = rig.base[i] + 0.5f * (fullPoints[i] - rig.base[i]);
        CHECK((halfPoints[i] - want).GetLength() < 1e-5f);
    }
    // A weight edit is epoch identity: it recompiles and moves the outputs.
    VtFloatArray weights;
    rig.stage->GetPrimAtPath(rig.mover).GetAttribute(TfToken("rigExec:weights")).Get(&weights);
    weights[20] += 1.0f;
    rig.stage->GetPrimAtPath(rig.mover).GetAttribute(TfToken("rigExec:weights")).Set(weights);
    const RigExecRigPose edited = evaluator.Evaluate(UsdTimeCode(2.0));
    CHECK(Published(edited, rig.outputs[0]) != Published(half, rig.outputs[0]));

    // A network whose outputs do not match its RigExecMlOutput children is
    // set aside, alone.
    MlRig bad = MakeMlRig();
    RigExecMlMoverHandle(bad.stage, bad.mover).AddOutput("extra");
    RigExecRigEvaluator refused(bad.stage, kRig);
    CHECK(refused.Compile(&errors));
    CHECK(refused.GetSkippedOperations().count(bad.mover) == 1);
}

const SdfPath kMapsMesh("/Asset/Geom/Brow");

// Two maps over the dial's tx and ty (double) and a float blink, published
// on the brow mesh; map 0 also drives a sparse morph through a blend input.
void
TestAnimatedMaps()
{
    UsdStageRefPtr stage = MakeStage();
    KeyStage(stage);
    const SdfPath blink = kDial.AppendProperty(TfToken("userBlink"));
    UsdAttribute blinkAttr = stage->GetPrimAtPath(kDial).CreateAttribute(
        blink.GetNameToken(), SdfValueTypeNames->Float);
    const float blinks[4] = {0.0f, 0.4f, 0.9f, 0.65f};
    for (size_t f = 0; f < 4; ++f) blinkAttr.Set(blinks[f], UsdTimeCode(double(f + 1)));
    const VtVec3fArray base{GfVec3f(0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)};
    stage->DefinePrim(kMapsMesh, TfToken("Mesh"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray).Set(base);
    UsdPrim shape = stage->DefinePrim(SdfPath("/Asset/Geom/BrowShape"), TfToken("BlendShape"));
    shape.CreateAttribute(TfToken("offsets"), SdfValueTypeNames->Vector3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 1)});
    shape.CreateAttribute(TfToken("pointIndices"), SdfValueTypeNames->IntArray)
        .Set(VtIntArray{2});

    auto builder = RigExecRigBuilder::Create(stage, kRig);
    RigExecAnimatedMapsHandle maps = builder.AddAnimatedMaps("Wrinkles");
    maps.SetInputs({kDial.AppendProperty(TfToken("avars:tx")),
                    kDial.AppendProperty(TfToken("avars:ty")), blink});
    maps.SetMeshes({kMapsMesh});
    // Map 0: tx ramps up over [0, 1] and back down over [1, 3] (a group
    // sharing the endpoint 1), plus ty over a reversed interval.
    const SdfPath raise = maps.AddMap("raise", {0, 0, 1}, {0, 1, 2}, {1, 3, 0},
                                      {1, -0.5f, 0.25f}, {0, 1.5f, 0},
                                      "wrinkleBrowRaise");
    // Map 1: the blink alone, steep, clamped to [0, 1].
    const SdfPath squint = maps.AddMap("squint", {2}, {0.2f}, {1}, {2.5f}, {-0.5f});
    RigExecBlendShapeMoverHandle morph =
        builder.NewMoverChain("Brow", kMapsMesh.AppendProperty(TfToken("points")))
            .AddBlendShapeMover("Morph");
    RigExecBlendInputHandle input = morph.AddBlendInput("raiseMorph");
    input.ConnectWeight(raise);
    input.AddSample("shape").SetBlendShape(SdfPath("/Asset/Geom/BrowShape"));

    RigExecAnimatedMapsTable table;
    table.inputCount = 3;
    CHECK(table.AddMap({{0, 0, 1, 1, 0}, {0, 1, 3, -0.5, 1.5}, {1, 2, 0, 0.25, 0}}));
    CHECK(table.AddMap({{2, 0.2f, 1, 2.5f, -0.5f}}));

    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        ++failures;
        for (const auto &e : errors) std::printf("FAIL maps compile: %s\n", e.c_str());
        return;
    }
    CHECK(evaluator.GetSkippedOperations().empty());
    const std::vector<SdfPath> primvars = evaluator.GetAnimatedMapPrimvars();
    CHECK(primvars.size() == 2);
    const SdfPath raisePrimvar = kMapsMesh.AppendProperty(TfToken("primvars:wrinkleBrowRaise"));
    const SdfPath squintPrimvar = kMapsMesh.AppendProperty(TfToken("primvars:squint"));
    CHECK(primvars.size() == 2 && primvars[0] == squintPrimvar &&
          primvars[1] == raisePrimvar);
    bool sawClamp = false, sawGroup = false;
    for (size_t f = 0; f < 4; ++f) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(double(f + 1)));
        CHECK(pose.valid);
        const double inputs[3] = {kKeys[f].dialX, kKeys[f].dialY, double(blinks[f])};
        double values[2];
        RigExecAnimatedMapsEvaluate(table, inputs, values);
        sawClamp = sawClamp || values[1] == 1.0;
        sawGroup = sawGroup || (inputs[0] > 1.0 && inputs[0] <= 3.0);
        const SdfPath outputs[2] = {raise, squint};
        const SdfPath meshPrimvars[2] = {raisePrimvar, squintPrimvar};
        for (int m = 0; m < 2; ++m) {
            const double got = Published(pose, outputs[m]);
            if (!(std::abs(got - values[m]) <= 1e-6)) {
                ++failures;
                std::printf("FAIL maps frame %zu map %d: %.8f, kernel %.8f\n",
                            f + 1, m, got, values[m]);
            }
            CHECK(Published(pose, meshPrimvars[m]) == got);
        }
        const auto found = pose.movedProperties.find(kMapsMesh.AppendProperty(TfToken("points")));
        CHECK(found != pose.movedProperties.end() &&
              found->second.IsHolding<VtVec3fArray>());
        if (found != pose.movedProperties.end() && found->second.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &moved = found->second.UncheckedGet<VtVec3fArray>();
            CHECK(moved.size() == 3 &&
                  std::abs(moved[2][2] - float(Published(pose, raise))) < 1e-6f &&
                  moved[0] == base[0] && moved[1] == base[1]);
        }
    }
    CHECK(sawClamp && sawGroup);
    CheckEvaluators(evaluator, stage, "maps");

    // A row edit is epoch identity; a disabled node publishes zeros.
    stage->GetPrimAtPath(raise.GetPrimPath()).GetAttribute(TfToken("rigExec:rowCut"))
        .Set(VtFloatArray{0.25f, 1.5f, 0.0f});
    const RigExecRigPose cut = evaluator.Evaluate(UsdTimeCode(2.0));
    CHECK(Published(cut, raise) != 0.0);
    maps.SetEnabled(false);
    const RigExecRigPose off = evaluator.Evaluate(UsdTimeCode(2.0));
    CHECK(Published(off, raise) == 0.0 && Published(off, raisePrimvar) == 0.0);

    // A row naming an input that does not exist sets the node aside alone.
    maps.SetEnabled(true);
    stage->GetPrimAtPath(squint.GetPrimPath()).GetAttribute(TfToken("rigExec:rowInputs"))
        .Set(VtIntArray{7});
    RigExecRigEvaluator refused(stage, kRig);
    CHECK(refused.Compile(&errors));
    CHECK(refused.GetSkippedOperations().count(SdfPath("/Asset/Rig/AnimatedMaps/Wrinkles")) == 1);
}

void
TestValidation()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRbfMethodDesc desc;
    AuthorInterpolator(stage, {"additive", "quaternion", "gaussian", false},
                       &desc);
    // Coincident poses under the distance method are reported: a pure twist
    // about the twist axis has the neutral's swing.
    {
        UsdStageRefPtr coincident = MakeStage();
        AuthorInterpolator(coincident,
                           {"interpolative", "swingAngle", "cubic", false}, &desc);
        coincident->GetPrimAtPath(kInterpolator.AppendChild(TfToken("twist")))
            .GetAttribute(TfToken("rigExec:rotations"))
            .Set(VtQuatfArray{About({0, 1, 0}, 80), About({0, 1, 0}, -40)});
        RigExecRigEvaluator evaluator(coincident, kRig);
        std::vector<std::string> notes;
        CHECK(evaluator.Compile(&notes));
        bool warned = false;
        for (const std::string &note : notes) {
            warned = warned || note.find("coincident") != std::string::npos;
        }
        CHECK(warned);
    }
    // A pose with one rotation for two drivers is refused, and only this
    // operation is set aside.
    stage->GetPrimAtPath(kInterpolator.AppendChild(TfToken("bend")))
        .GetAttribute(TfToken("rigExec:rotations"))
        .Set(VtQuatfArray{About({0, 0, 1}, 70)});
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.GetSkippedOperations().count(kInterpolator) == 1);

    // Numeric drivers measured by a quaternion metric are refused too.
    UsdStageRefPtr numeric = MakeStage();
    AuthorInterpolator(numeric, {"interpolative", "quaternion", "linear", true},
                       &desc);
    RigExecRigEvaluator second(numeric, kRig);
    CHECK(second.Compile(&errors));
    CHECK(second.GetSkippedOperations().count(kInterpolator) == 1);

    // The pose solver keeps refusing a second driver.
    UsdStageRefPtr pose = MakeStage();
    AuthorInterpolator(pose, {"additive", "quaternion", "gaussian", false},
                       &desc);
    RigExecPoseInterpolatorHandle(pose, kInterpolator).SetSolver(TfToken("pose"));
    RigExecRigEvaluator third(pose, kRig);
    CHECK(third.Compile(&errors));
    CHECK(third.GetSkippedOperations().count(kInterpolator) == 1);
}

} // namespace

int
main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const Case cases[] = {
        {"additive", "quaternion", "gaussian", false},
        {"interpolative", "quaternion", "gaussian", false},
        {"interpolative", "swingAngle", "cubic", false},
        {"additive", "twistAngle", "linear", false},
        {"interpolative", "euclidean", "gaussian", false},
        {"additive", "euclidean", "cubic", true},
        {"interpolative", "euclidean", "linear", true},
    };
    for (const Case &c : cases) {
        TestCase(c);
    }
    TestMlMover();
    TestAnimatedMaps();
    TestValidation();
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecRigLogic: all tests passed\n");
    return 0;
}
