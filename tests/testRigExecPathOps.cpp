// The float math mover's path and quantize operations. First the kernel
// alone: arc length and sampling of each UsdGeomBasisCurves basis, the
// heading convention, the local offset, clamping, and the quantize stair.
// Then a rig whose movers sample a pinned catmullRom path: the evaluator
// publishes the kernel's bits; a baked binary plays them along the
// timeline; a points set on the binary (the game's moving window) plays
// what the same points authored in the session layer publish; an
// interactive override on the points previews them; a stage edit of the
// points re-evaluates; and compile refuses a path mover whose relationship
// names no supported curve, which skips it with the reason. Every run is
// audited for step bodies that read the stage or the overlay, and frozen
// jobs (the worker's snapshot) publish what the live program publishes.
#include "rigExecRuntimeDrive.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecMath/pathCurveKernel.h"
#include "rigExecRuntime/runtime.h"

#include "pxr/base/arch/env.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/basisCurves.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

bool
Near(double a, double b, double tolerance = 1e-9)
{
    return std::fabs(a - b) <= tolerance;
}

RigExecPathCurve
Curve(const std::vector<float> &xyz, RigExecPathBasis basis, bool pinned)
{
    RigExecPathCurve curve;
    curve.points = xyz.data();
    curve.pointCount = xyz.size() / 3;
    curve.basis = basis;
    curve.pinned = pinned;
    return curve;
}

// Arc length measured by a dense chord sum over the sampled positions.
double
ChordLength(const RigExecPathCurve &curve, double length)
{
    double total = 0.0, previous[3], heading = 0.0;
    RigExecSamplePath(curve, 0.0, previous, &heading);
    const int steps = 20000;
    for (int i = 1; i <= steps; ++i) {
        double p[3];
        RigExecSamplePath(curve, length * i / steps, p, &heading);
        total += std::sqrt((p[0] - previous[0]) * (p[0] - previous[0]) +
                           (p[1] - previous[1]) * (p[1] - previous[1]) +
                           (p[2] - previous[2]) * (p[2] - previous[2]));
        std::memcpy(previous, p, sizeof(p));
    }
    return total;
}

void
TestKernel()
{
    // An L of two linear segments: 2 along +Z, then 2 along +X.
    const std::vector<float> ell{0, 0, 0, 0, 0, 2, 2, 0, 2};
    const RigExecPathCurve linear = Curve(ell, RigExecPathBasis::Linear, false);
    CHECK(RigExecPathCurveValid(linear));
    CHECK(Near(RigExecPathLength(linear), 4.0));
    double p[3], heading = 0.0;
    CHECK(RigExecSamplePath(linear, 1.0, p, &heading));
    CHECK(Near(p[0], 0.0) && Near(p[2], 1.0) && Near(heading, 0.0));
    CHECK(RigExecSamplePath(linear, 3.0, p, &heading));
    CHECK(Near(p[0], 1.0) && Near(p[2], 2.0) && Near(heading, 90.0));
    // Clamped at both ends.
    CHECK(RigExecSamplePath(linear, 9.0, p, &heading));
    CHECK(Near(p[0], 2.0) && Near(p[2], 2.0));
    CHECK(RigExecSamplePath(linear, -1.0, p, &heading));
    CHECK(Near(p[0], 0.0) && Near(p[2], 0.0) && Near(heading, 0.0));

    // Components: the local (x, z) offset turns with the heading (90 here:
    // local +X becomes world -Z), the heading offset adds and wraps.
    RigExecPathSettings settings;
    settings.valid = true;
    settings.offsetX = 1.0f;
    settings.component = RigExecPathComponent::X;
    float r = 0.0f;
    CHECK(RigExecEvaluatePathComponent(linear, settings, 5.0f, 2.0f, &r));
    CHECK(Near(r, 1.0, 1e-6));
    settings.component = RigExecPathComponent::Z;
    CHECK(RigExecEvaluatePathComponent(linear, settings, 5.0f, 2.0f, &r));
    CHECK(Near(r, 1.0, 1e-6));
    settings.component = RigExecPathComponent::Heading;
    settings.headingOffset = 120.0f;
    CHECK(RigExecEvaluatePathComponent(linear, settings, 5.0f, 2.0f, &r));
    CHECK(Near(r, -150.0, 1e-5));
    settings.component = RigExecPathComponent::End;
    CHECK(RigExecEvaluatePathComponent(linear, settings, 0.0f, 2.5f, &r));
    CHECK(Near(r, 6.5, 1e-6));

    // Evenly spaced collinear points: every cubic basis is the line, at
    // unit speed per point spacing, over the span it covers.
    const std::vector<float> line{0, 0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 3};
    CHECK(Near(RigExecPathLength(Curve(line, RigExecPathBasis::Bezier, false)),
               3.0));
    CHECK(Near(RigExecPathLength(
                   Curve(line, RigExecPathBasis::CatmullRom, false)),
               1.0));
    CHECK(Near(RigExecPathLength(
                   Curve(line, RigExecPathBasis::CatmullRom, true)),
               3.0));
    CHECK(Near(RigExecPathLength(Curve(line, RigExecPathBasis::Bspline, true)),
               3.0));
    {
        const RigExecPathCurve rom =
            Curve(line, RigExecPathBasis::CatmullRom, true);
        CHECK(RigExecSamplePath(rom, 1.5, p, &heading));
        CHECK(Near(p[2], 1.5, 1e-9) && Near(heading, 0.0));
    }
    // Too few points for the basis, and a bezier count that is not 3k+1.
    CHECK(!RigExecPathCurveValid(
        Curve({0, 0, 0, 0, 0, 1}, RigExecPathBasis::CatmullRom, false)));
    CHECK(RigExecPathCurveValid(
        Curve({0, 0, 0, 0, 0, 1}, RigExecPathBasis::CatmullRom, true)));
    CHECK(!RigExecPathCurveValid(
        Curve({0, 0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0, 4},
              RigExecPathBasis::Bezier, false)));

    // A curved path: the integrated length matches a dense chord sum, and
    // equal arc steps are equal chord steps.
    const std::vector<float> ess{0, 0, 0, 1, 0, 1, 0, 0, 2,
                                 -1, 0, 3, 0, 0, 4, 2, 0, 4};
    for (const RigExecPathBasis basis :
         {RigExecPathBasis::Bspline, RigExecPathBasis::CatmullRom}) {
        const RigExecPathCurve curve = Curve(ess, basis, true);
        const double length = RigExecPathLength(curve);
        CHECK(length > 4.0);
        CHECK(Near(ChordLength(curve, length), length, 1e-6));
        double a[3], b[3];
        RigExecSamplePath(curve, 2.0, a, &heading);
        RigExecSamplePath(curve, 2.001, b, &heading);
        const double step = std::sqrt((b[0] - a[0]) * (b[0] - a[0]) +
                                      (b[2] - a[2]) * (b[2] - a[2]));
        CHECK(Near(step, 0.001, 1e-8));
    }

    // A planar measure is ground distance: a ramp rising 1 over 2 on Z
    // measures 2, its height carried along.
    {
        const std::vector<float> rising{0, 0, 0, 0, 1, 2};
        RigExecPathCurve ramp = Curve(rising, RigExecPathBasis::Linear, false);
        CHECK(Near(RigExecPathLength(ramp), std::sqrt(5.0)));
        ramp.planar = true;
        CHECK(Near(RigExecPathLength(ramp), 2.0));
        CHECK(RigExecSamplePath(ramp, 1.0, p, &heading));
        CHECK(Near(p[1], 0.5) && Near(p[2], 1.0) && Near(heading, 0.0));
    }

    // Quantize: the stair through the origin, boundaries as floats hold
    // them, negative values, and a step that is not positive.
    CHECK(RigExecQuantizeDown(0.48f, 0.0f, 0.16f) == float(0.0 + 3 * 0.16));
    CHECK(RigExecQuantizeDown(0.479f, 0.0f, 0.16f) == float(2 * 0.16));
    CHECK(RigExecQuantizeDown(-0.01f, 0.0f, 0.16f) == float(-0.16));
    CHECK(RigExecQuantizeDown(0.03f, 0.0267f, 0.16f) == 0.0267f);
    CHECK(RigExecQuantizeDown(0.03f, 0.0267f, 0.0f) == 0.03f);
}

const SdfPath kRig("/Rig");
const SdfPath kRoot("/Rig/Controls/Root");
const SdfPath kPath("/Rig/Path");
const SdfPath kGait("/Rig/Gait");

const std::vector<GfVec3f> kPoints{{0, 0, 0},   {0.5, 0, 1},  {0, 0, 2},
                                   {-0.5, 0, 3}, {0.5, 0, 4}};

UsdPrim
Mover(const UsdStageRefPtr &stage, const std::string &name,
      const SdfPath &target, const TfToken &op)
{
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Rig/Movers").AppendChild(TfToken(name)),
        TfToken("RigExecFloatMathMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    mover.GetAttribute(TfToken("rigExec:operation")).Set(op);
    return mover;
}

UsdPrim
PathMover(const UsdStageRefPtr &stage, const std::string &name,
          const SdfPath &target, const char *component)
{
    const UsdPrim mover = Mover(stage, name, target, TfToken("path"));
    mover.GetRelationship(TfToken("rigExec:path")).SetTargets({kPath});
    mover.GetAttribute(TfToken("rigExec:pathComponent"))
        .Set(TfToken(component));
    mover.GetAttribute(TfToken("rigExec:pathOffset")).Set(GfVec2f(0.1f, 0.2f));
    mover.GetAttribute(TfToken("rigExec:pathHeadingOffset")).Set(30.0f);
    mover.GetAttribute(TfToken("inputs:value")).Set(0.25f);
    return mover;
}

// The rig: Root's tx, tz and ry follow a pinned catmullRom path at the
// distance each avar holds (keyed 0.25 -> 5 over frames 1 -> 10), and two
// gait floats take the path's end and a quantized distance.
UsdStageRefPtr
PathRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(kRig, TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Rig/Joints"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Rig/Joints/J"), TfToken("RigExecJoint"))
        .GetAttribute(TfToken("rest:space"))
        .Set(GfMatrix4d(1.0));
    stage->DefinePrim(SdfPath("/Rig/Controls"), TfToken("Scope"));
    const UsdPrim root = stage->DefinePrim(kRoot, TfToken("RigExecControl"));
    root.GetAttribute(TfToken("rest:space")).Set(GfMatrix4d(1.0));
    for (const char *avar : {"avars:tx", "avars:tz", "avars:ry"}) {
        const UsdAttribute a = root.GetAttribute(TfToken(avar));
        a.Set(0.25, UsdTimeCode(1.0));
        a.Set(5.0, UsdTimeCode(10.0));
    }
    UsdGeomBasisCurves curves = UsdGeomBasisCurves::Define(stage, kPath);
    curves.CreateTypeAttr().Set(UsdGeomTokens->cubic);
    curves.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
    curves.CreateWrapAttr().Set(UsdGeomTokens->pinned);
    curves.CreateCurveVertexCountsAttr().Set(VtIntArray{int(kPoints.size())});
    curves.CreatePointsAttr().Set(
        VtVec3fArray(kPoints.begin(), kPoints.end()));
    const UsdPrim gait = stage->DefinePrim(kGait, TfToken("Scope"));
    for (const char *name : {"end", "cycle"}) {
        const UsdAttribute a =
            gait.CreateAttribute(TfToken(name), SdfValueTypeNames->Float);
        a.Set(0.25f, UsdTimeCode(1.0));
        a.Set(5.0f, UsdTimeCode(10.0));
    }
    stage->DefinePrim(SdfPath("/Rig/Movers"), TfToken("Scope"));
    PathMover(stage, "X", kRoot.AppendProperty(TfToken("avars:tx")), "x");
    PathMover(stage, "Z", kRoot.AppendProperty(TfToken("avars:tz")), "z");
    PathMover(stage, "Heading", kRoot.AppendProperty(TfToken("avars:ry")),
              "heading");
    PathMover(stage, "End", kGait.AppendProperty(TfToken("end")), "end");
    const UsdPrim stair = Mover(stage, "Stair",
                                kGait.AppendProperty(TfToken("cycle")),
                                TfToken("quantize"));
    stair.GetAttribute(TfToken("inputs:value")).Set(0.16f);
    stair.GetAttribute(TfToken("inputs:min")).Set(0.0267f);
    return stage;
}

// What each target should hold at \p time: the kernel over the stage's
// points and its distance there.
std::map<std::string, float>
Expected(const UsdStageRefPtr &stage, double time,
         const VtVec3fArray &points)
{
    std::map<std::string, float> out;
    double distance = 0.0;
    stage->GetAttributeAtPath(kRoot.AppendProperty(TfToken("avars:tx")))
        .Get(&distance, UsdTimeCode(time));
    RigExecPathCurve curve;
    curve.points = points.cdata()->data();
    curve.pointCount = points.size();
    curve.basis = RigExecPathBasis::CatmullRom;
    curve.pinned = true;
    RigExecPathSettings settings;
    settings.valid = true;
    settings.offsetX = 0.1f;
    settings.offsetZ = 0.2f;
    settings.headingOffset = 30.0f;
    const auto component = [&](RigExecPathComponent c, float base) {
        settings.component = c;
        float r = base;
        RigExecEvaluatePathComponent(curve, settings, base, 0.25f, &r);
        return r;
    };
    // A double chain is computed in float.
    const float base = float(distance);
    out[kRoot.AppendProperty(TfToken("avars:tx")).GetString()] =
        component(RigExecPathComponent::X, base);
    out[kRoot.AppendProperty(TfToken("avars:tz")).GetString()] =
        component(RigExecPathComponent::Z, base);
    out[kRoot.AppendProperty(TfToken("avars:ry")).GetString()] =
        component(RigExecPathComponent::Heading, base);
    float gaitBase = 0.0f;
    stage->GetAttributeAtPath(kGait.AppendProperty(TfToken("end")))
        .Get(&gaitBase, UsdTimeCode(time));
    out[kGait.AppendProperty(TfToken("end")).GetString()] =
        component(RigExecPathComponent::End, gaitBase);
    out[kGait.AppendProperty(TfToken("cycle")).GetString()] =
        RigExecQuantizeDown(gaitBase, 0.0267f, 0.16f);
    return out;
}

float
PoseValue(const RigExecRigPose &pose, const std::string &path)
{
    const auto found = pose.movedProperties.find(SdfPath(path));
    if (found == pose.movedProperties.end()) {
        return NAN;
    }
    if (found->second.IsHolding<double>()) {
        return float(found->second.UncheckedGet<double>());
    }
    return found->second.IsHolding<float>()
               ? found->second.UncheckedGet<float>()
               : NAN;
}

float
ReaderValue(const RigExecRuntimeReader &reader, const std::string &path)
{
    for (const RigExecRuntimePropertyValue &v : reader.GetPropertyValues()) {
        if (v.path == path) {
            return v.value.tag == RrPropertyValue::Tag::Double
                       ? float(v.value.f64)
                       : v.value.f32;
        }
    }
    return NAN;
}

bool
SameBits(float a, float b)
{
    return std::memcmp(&a, &b, sizeof(float)) == 0;
}

void
CheckPose(const char *what, const RigExecRigPose &pose,
          const std::map<std::string, float> &expected)
{
    for (const auto &[path, value] : expected) {
        const float got = PoseValue(pose, path);
        if (!SameBits(got, value)) {
            ++failures;
            std::printf("FAIL %s: %s = %.9g, expected %.9g\n", what,
                        path.c_str(), got, value);
        }
    }
}

void
CheckReader(const char *what, const RigExecRuntimeReader &reader,
            const RigExecRigPose &pose)
{
    for (const auto &[path, value] : pose.movedProperties) {
        if (!value.IsHolding<double>() && !value.IsHolding<float>()) {
            continue;
        }
        const float want = PoseValue(pose, path.GetString());
        const float got = ReaderValue(reader, path.GetString());
        if (!SameBits(got, want)) {
            ++failures;
            std::printf("FAIL %s: binary %s = %.9g, evaluator %.9g\n", what,
                        path.GetText(), got, want);
        }
    }
}

// No step body read the stage or the overlay in the program's last run.
void
CheckPurity(const char *what, const RigExecRigEvaluator &evaluator)
{
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    CHECK(program);
    if (!program) {
        return;
    }
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    CHECK(B.purityAudit);
    const uint64_t reads =
        B.purityViolations.count.load(std::memory_order_relaxed);
    if (reads != 0) {
        ++failures;
        std::printf("FAIL %s: %llu step-body read(s) of the stage\n", what,
                    static_cast<unsigned long long>(reads));
    }
}

// A frozen job at \p time under \p overrides publishes \p live's values.
void
CheckFrozen(const char *what, RigExecRigEvaluator *evaluator, double time,
            const std::vector<RigExecValueOverride> &overrides,
            const RigExecRigPose &live)
{
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    CHECK(RigExecFreezeProgram(*evaluator, &frozen, &error));
    if (!frozen) {
        std::printf("FAIL %s: freeze refused: %s\n", what, error.c_str());
        return;
    }
    RigExecFrameInputs inputs;
    CHECK(RigExecSampleFrameInputs(*evaluator, UsdTimeCode(time), overrides,
                                   &inputs, &error));
    RigExecBackgroundScheduler scheduler;
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator->GetBindingEpochDigest();
    context.generation = scheduler.CurrentGeneration(kRig);
    context.slotCount = evaluator->GetBakedProgram()->GetProviderCount();
    context.varyingInputCount = inputs.values.size();
    context.frozen = frozen.get();
    const RigExecRigPose warmed = RigExecEvaluateFrozen(
        context, inputs, RigExecMakeProductionStepRunner(), &scheduler, kRig);
    if (!warmed.valid) {
        ++failures;
        std::printf("FAIL %s: the frozen job was declined\n", what);
        return;
    }
    std::map<std::string, float> expected;
    for (const auto &[path, value] : live.movedProperties) {
        if (value.IsHolding<double>() || value.IsHolding<float>()) {
            expected[path.GetString()] = PoseValue(live, path.GetString());
        }
    }
    CheckPose(what, warmed, expected);
}

void
TestRig()
{
    // Read at Build: every program below counts step-body stage reads.
    ArchSetEnv("RIGEXEC_PURITY_AUDIT", "1", /*overwrite=*/true);
    UsdStageRefPtr stage = PathRig();
    const VtVec3fArray authored(kPoints.begin(), kPoints.end());
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> notices;
    CHECK(evaluator.Compile(&notices));
    for (const std::string &n : notices) {
        std::printf("  compile: %s\n", n.c_str());
    }
    const std::vector<double> times{1.0, 4.0, 7.5, 10.0};
    std::vector<RigExecRigPose> poses;
    for (const double t : times) {
        poses.push_back(evaluator.Evaluate(UsdTimeCode(t)));
        CHECK(poses.back().valid);
        CheckPose("evaluator", poses.back(), Expected(stage, t, authored));
        CheckPurity("evaluator", evaluator);
    }
    CheckFrozen("frozen", &evaluator, 7.5, {}, poses[2]);

    // The binary along the timeline.
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
    if (bytes.empty()) {
        std::printf("  bake: %s\n", error.c_str());
        return;
    }
    RigExecTestPlayer player;
    CHECK(player.Open(bytes, stage, &error));
    if (!error.empty()) {
        std::printf("  open: %s\n", error.c_str());
        return;
    }
    size_t pointsInput = 0;
    const std::string pointsName = kPath.AppendProperty(TfToken("points")).GetString();
    CHECK(player->FindInput(pointsName, &pointsInput));
    CHECK(player->GetInputInfo(pointsInput).type == RrInputTag::Vec3fArray);
    CHECK(player->GetInputInfo(pointsInput).defaultCount == kPoints.size());
    for (size_t i = 0; i < times.size(); ++i) {
        CHECK(player.Play(times[i], &error));
        CheckReader("timeline", player.Reader(), poses[i]);
    }

    // The moving window: a points set of the same count plays what the
    // same points authored publish; another count is refused.
    VtVec3fArray moved = authored;
    for (GfVec3f &point : moved) {
        point += GfVec3f(1.0f, 0.0f, 0.5f);
    }
    moved[2] += GfVec3f(0.3f, 0.0f, 0.0f);
    std::vector<RigExecRigPose> edited;
    CHECK(RigExecTestEditedPoses(stage, kRig,
                                 {{kPath.AppendProperty(TfToken("points")),
                                   VtValue(moved)}},
                                 {7.5}, &edited, &error));
    if (!edited.empty()) {
        CheckPose("authored window", edited[0], Expected(stage, 7.5, moved));
        RigExecRuntimeArray array;
        array.tag = RrInputTag::Vec3fArray;
        array.data = moved.cdata();
        array.count = moved.size();
        // At the time the reader last ran, so only the points move.
        CHECK(player.Play(7.5, &error));
        CheckReader("before window set", player.Reader(), poses[2]);
        CHECK(player->SetInputArray(pointsName, array, &error));
        CHECK(player.Play(7.5, &error));
        CheckReader("window set", player.Reader(), edited[0]);
        // And back, again with nothing else moving.
        RigExecRuntimeArray back = array;
        back.data = authored.cdata();
        CHECK(player->SetInputArray(pointsName, back, &error));
        CHECK(player.Play(7.5, &error));
        CheckReader("window restored", player.Reader(), poses[2]);
        array.count = moved.size() - 1;
        CHECK(!player->SetInputArray(pointsName, array, &error));
    }

    // An interactive override on the points previews them, at the time the
    // evaluator last ran.
    CheckPose("before override", evaluator.Evaluate(UsdTimeCode(7.5)),
              Expected(stage, 7.5, authored));
    evaluator.SetInteractiveOverrides(
        {RigExecValueOverride{kPath, TfToken(), TfToken("points"),
                              VtValue(moved)}});
    const RigExecRigPose preview = evaluator.Evaluate(UsdTimeCode(7.5));
    CHECK(preview.valid);
    if (!preview.valid) {
        for (const std::string &d : preview.diagnostics) {
            std::printf("  preview: %s\n", d.c_str());
        }
    }
    CheckPose("override", preview, Expected(stage, 7.5, moved));
    CheckPurity("override", evaluator);
    CheckFrozen("frozen override", &evaluator, 7.5,
                {RigExecValueOverride{kPath, TfToken(), TfToken("points"),
                                      VtValue(moved)}},
                preview);
    evaluator.ClearInteractiveOverrides();
    CheckPose("override released", evaluator.Evaluate(UsdTimeCode(7.5)),
              Expected(stage, 7.5, authored));

    // A stage edit of the points re-evaluates.
    stage->GetAttributeAtPath(kPath.AppendProperty(TfToken("points")))
        .Set(moved);
    CheckPose("stage edit", evaluator.Evaluate(UsdTimeCode(7.5)),
              Expected(stage, 7.5, moved));
    CheckPurity("stage edit", evaluator);
    ArchRemoveEnv("RIGEXEC_PURITY_AUDIT");
}

void
TestCompileRefusals()
{
    const auto refused = [](const std::function<void(const UsdStageRefPtr &)>
                                &edit,
                            const char *needle) {
        UsdStageRefPtr stage = PathRig();
        edit(stage);
        RigExecRigEvaluator evaluator(stage, kRig);
        std::vector<std::string> notices;
        const bool compiled = evaluator.Compile(&notices);
        bool named = false;
        for (const std::string &n : notices) {
            named = named || n.find(needle) != std::string::npos;
        }
        // A mover that fails validation is skipped with the reason.
        if (!compiled || !named) {
            ++failures;
            std::printf("FAIL refusal '%s': compiled %d\n", needle,
                        int(compiled));
            for (const std::string &n : notices) {
                std::printf("  %s\n", n.c_str());
            }
        }
    };
    refused([](const UsdStageRefPtr &stage) {
        stage->GetPrimAtPath(SdfPath("/Rig/Movers/X"))
            .GetRelationship(TfToken("rigExec:path"))
            .SetTargets({kGait});
    }, "is not a UsdGeomBasisCurves prim");
    refused([](const UsdStageRefPtr &stage) {
        UsdGeomBasisCurves(stage->GetPrimAtPath(kPath))
            .GetWrapAttr()
            .Set(UsdGeomTokens->periodic);
    }, "nonperiodic or pinned");
    refused([](const UsdStageRefPtr &stage) {
        stage->GetPrimAtPath(SdfPath("/Rig/Movers/Z"))
            .GetAttribute(TfToken("rigExec:pathComponent"))
            .Set(TfToken("w"));
    }, "unknown rigExec:pathComponent");
    refused([](const UsdStageRefPtr &stage) {
        UsdGeomBasisCurves(stage->GetPrimAtPath(kPath))
            .GetCurveVertexCountsAttr()
            .Set(VtIntArray{2, 3});
    }, "exactly one curve");
}

}  // namespace

int
main()
{
    PlugRegistry::GetInstance().RegisterPlugins(
        TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR));
    TestKernel();
    TestRig();
    TestCompileRefusals();
    std::printf("testRigExecPathOps: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
