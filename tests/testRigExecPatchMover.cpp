#include "rigExec/rigEvaluator.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecRigging/rigBuilder.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usdGeom/nurbsPatch.h"
#include "pxr/usd/usdGeom/points.h"

#include <cmath>
#include <cstdio>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

static int failures = 0;
#define CHECK(x)                                                           \
    do {                                                                   \
        if (!(x)) {                                                        \
            std::printf("FAIL %d: %s\n", __LINE__, #x);                    \
            ++failures;                                                    \
        }                                                                  \
    } while (0)

#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"

namespace {

const char *kFixture = R"USD(#usda 1.0
def Xform "Asset" {
 def RigExecRoot "Rig" {
  def RigExecControl "UvDriver" { }
  def RigExecControl "PatchDriver" { }
  def Scope "Movers" {
   def RigExecMatrixMover "MoveUvs" (prepend apiSchemas = ["RigExecMoverAPI"]) {
    rel rigExec:moves = </Asset/UVs.points>
    rel rigExec:transform = </Asset/Rig/UvDriver>
   }
   def RigExecMatrixMover "MovePatch" (prepend apiSchemas = ["RigExecMoverAPI"]) {
    rel rigExec:moves = </Asset/Patch.points>
    rel rigExec:transform = </Asset/Rig/PatchDriver>
   }
   def RigExecPatchMover "Stick" (prepend apiSchemas = ["RigExecMoverAPI"]) {
    rel rigExec:moves = </Asset/Target.points>
    rel rigExec:patch = </Asset/Patch> (rigExecReadPhase = "final")
    rel rigExec:uvPoints = </Asset/UVs> (rigExecReadPhase = "final")
   }
  }
 }
 def NurbsPatch "Patch" {
  int uVertexCount = 3
  int vVertexCount = 2
  int uOrder = 3
  int vOrder = 2
  double[] uKnots = [0, 0, 0, 1, 1, 1]
  double[] vKnots = [0, 0, 1, 1]
  uniform token uForm = "open"
  uniform token vForm = "open"
  point3f[] points = [
   (0, 0, 0), (1, 0, 2), (2, 0, 0),
   (0, 4, 0), (1, 4, 2), (2, 4, 0)
  ]
 }
 def Points "UVs" {
  point3f[] points = [(0.5, 0.25, 0), (0.0, 1.0, 0)]
 }
 def Points "Target" {
  point3f[] points = [(10, 20, 30), (-5, -6, -7)]
 }
}
)USD";

UsdStageRefPtr MakeStage()
{
    auto stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(kFixture));
    return stage;
}

// The arch of kFixture with two UV shapes, Smile (u + 0.5) and Open
// (v + 0.5 at point 0, v - 0.5 at point 1), weighted by a FLOAT and a DOUBLE
// avar through inputs:uvWeights' ordered connections.
const char *kShapesFixture = R"USD(#usda 1.0
def Xform "Asset" {
 def RigExecRoot "Rig" {
  def RigExecControl "Mouth" (prepend apiSchemas = ["RigExecControlAPI"]) {
   float avars:smile = 0
   double avars:open = 0
   matrix4d rest:space = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))
  }
  def Scope "Movers" {
   def RigExecPatchMover "Stick" (prepend apiSchemas = ["RigExecMoverAPI"]) {
    rel rigExec:moves = </Asset/Target.points>
    rel rigExec:patch = </Asset/Patch>
    rel rigExec:uvPoints = </Asset/UVs>
    rel rigExec:uvShapes = [</Asset/SmileUVs>, </Asset/OpenUVs>]
    float[] inputs:uvWeights.connect = [</Asset/Rig/Mouth.avars:smile>, </Asset/Rig/Mouth.avars:open>]
   }
  }
 }
 def NurbsPatch "Patch" {
  int uVertexCount = 3
  int vVertexCount = 2
  int uOrder = 3
  int vOrder = 2
  double[] uKnots = [0, 0, 0, 1, 1, 1]
  double[] vKnots = [0, 0, 1, 1]
  uniform token uForm = "open"
  uniform token vForm = "open"
  point3f[] points = [
   (0, 0, 0), (1, 0, 2), (2, 0, 0),
   (0, 4, 0), (1, 4, 2), (2, 4, 0)
  ]
 }
 def Points "UVs" {
  point3f[] points = [(0.25, 0.25, 0), (0.0, 1.0, 0)]
 }
 def Points "SmileUVs" {
  point3f[] points = [(0.75, 0.25, 0), (0.5, 1.0, 0)]
 }
 def Points "OpenUVs" {
  point3f[] points = [(0.25, 0.75, 0), (0.0, 0.5, 0)]
 }
 def Points "Target" {
  point3f[] points = [(10, 20, 30), (-5, -6, -7)]
 }
}
)USD";

UsdStageRefPtr MakeShapesStage()
{
    auto stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(kShapesFixture));
    return stage;
}

VtVec3fArray EvaluateTarget(RigExecRigEvaluator &rig)
{
    rig.cpuReference = true;
    auto pose = rig.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(pose.referenceMismatches == 0);
    CHECK(pose.referenceAgreements > 0);
    auto it = pose.movedProperties.find(SdfPath("/Asset/Target.points"));
    CHECK(it != pose.movedProperties.end());
    return it == pose.movedProperties.end()
               ? VtVec3fArray()
               : it->second.Get<VtVec3fArray>();
}

void CheckNear(const GfVec3f &actual, const GfVec3f &expected,
               float tol = 2e-6f)
{
    if ((actual - expected).GetLength() > tol) {
        std::printf("actual (%g, %g, %g) expected (%g, %g, %g)\n",
                    actual[0], actual[1], actual[2],
                    expected[0], expected[1], expected[2]);
    }
    CHECK((actual - expected).GetLength() <= tol);
}

void CheckFrozen(RigExecRigEvaluator &rig)
{
    std::string why;
    const bool same = RigExecFrozenMatchesLive(
        &rig, SdfPath("/Asset/Rig"), UsdTimeCode::Default(), &why);
    if (!same) {
        std::printf("frozen mismatch: %s\n", why.c_str());
    }
    CHECK(same);
}

void CheckRuntime(const UsdStageRefPtr &stage, const char *what,
                  const SdfPath &avar, double posed)
{
    RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    std::vector<uint8_t> bytes;
    std::string error;
    const bool baked = RigExecTestBakeAt(rig, 1, &bytes, &error);
    CHECK(baked);
    if (!baked) {
        std::printf("%s bake: %s\n", what, error.c_str());
        return;
    }
    RigExecTestPlayer player;
    const bool opened = player.Open(bytes, stage, &error);
    CHECK(opened);
    if (!opened) {
        std::printf("%s open: %s\n", what, error.c_str());
        return;
    }
    const auto *handler =
        RigExecFindMoverHandler(TfToken("RigExecPatchMover"));
    CHECK(handler && handler->runtimeKernel.IsSet());
    if (!handler) {
        return;
    }
    CHECK(player->SetExternalKernel("RigExecPatchMover",
                                    handler->runtimeKernel, &error));
    CHECK(player->GetMissingExternalKernels().empty());
    const auto compare = [&](const RigExecRigPose &pose, const char *phase) {
        std::vector<std::string> diffs;
        const bool same =
            RigExecCompareRuntimeOutputs(pose, player.Reader(), &diffs);
        if (!same) {
            for (const auto &line : diffs) {
                std::printf("%s %s: %s\n", what, phase, line.c_str());
            }
        }
        CHECK(same);
    };
    const bool played = player.Play(1, &error);
    CHECK(played);
    if (!played) {
        std::printf("%s play: %s\n", what, error.c_str());
        return;
    }
    compare(rig.Evaluate(UsdTimeCode(1)), "unposed");
    const bool held =
        player.Hold(avar.GetString(), posed, &error) &&
        player.Play(1, &error);
    CHECK(held);
    if (!held) {
        std::printf("%s posed play: %s\n", what, error.c_str());
        return;
    }
    // The edit in the avar's own precision: a float dial takes a float.
    const UsdAttribute avarAttr = stage->GetAttributeAtPath(avar);
    const VtValue edited =
        avarAttr && avarAttr.GetTypeName() == SdfValueTypeNames->Float
            ? VtValue(float(posed)) : VtValue(posed);
    std::vector<RigExecRigPose> poses;
    const bool referenced = RigExecTestEditedPoses(
        stage, SdfPath("/Asset/Rig"), {{avar, edited}}, {1.0}, &poses,
        &error);
    CHECK(referenced);
    if (referenced) {
        compare(poses.front(), "posed");
    }
}

// Quadratic Bezier in u: P0=(0,y,0), P1=(1,y,2), P2=(2,y,0) ->
// x(u) = 2*(1-u)*u + 2*u^2 = 2*u
// z(u) = 4*(1-u)*u
// Linear in v: y(v) = 4*v
GfVec3f ExpectedArch(double u, double v)
{
    u = std::min(std::max(u, 0.0), 1.0);
    v = std::min(std::max(v, 0.0), 1.0);
    return GfVec3f(float(2.0 * u), float(4.0 * v), float(4.0 * (1.0 - u) * u));
}

}  // namespace

int main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);

    // 1. Runtime playback with both controller-driven UVs and controller-driven patch.
    CheckRuntime(MakeStage(), "uv_driver",
                 SdfPath("/Asset/Rig/UvDriver.avars:tx"), 0.25);
    CheckRuntime(MakeStage(), "uv_driver_v",
                 SdfPath("/Asset/Rig/UvDriver.avars:ty"), 0.5);
    CheckRuntime(MakeStage(), "patch_driver",
                 SdfPath("/Asset/Rig/PatchDriver.avars:tz"), 1.5);

    // 2. Live evaluation, controller-driven UVs, patch deformation, envelope, rational weights, wrap.
    {
        auto stage = MakeStage();
        RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(rig.Compile(&errors));
        CHECK(errors.empty());
        std::vector<std::string> reasons;
        CHECK(rig.IsBakeable(&reasons));

        std::string before, after;
        stage->GetRootLayer()->ExportToString(&before);
        VtVec3fArray pts = EvaluateTarget(rig);
        stage->GetRootLayer()->ExportToString(&after);
        CHECK(before == after);
        CHECK(pts.size() == 2);
        if (pts.size() == 2) {
            CheckNear(pts[0], ExpectedArch(0.5, 0.25));
            CheckNear(pts[1], ExpectedArch(0.0, 1.0));
        }
        CheckFrozen(rig);

        // Drive UVs with controller UvDriver: translate U by +0.25, V by +0.5.
        auto uvDriver = stage->GetPrimAtPath(SdfPath("/Asset/Rig/UvDriver"));
        CHECK(uvDriver.GetAttribute(TfToken("avars:tx")).Set(0.25));
        CHECK(uvDriver.GetAttribute(TfToken("avars:ty")).Set(0.5));
        pts = EvaluateTarget(rig);
        if (pts.size() == 2) {
            CheckNear(pts[0], ExpectedArch(0.75, 0.75));
            // v = 1.0 + 0.5 = 1.5 clamps to 1.0 on open patch.
            CheckNear(pts[1], ExpectedArch(0.25, 1.0));
        }
        CheckFrozen(rig);

        // Periodic vForm wraps v = 1.5 to 0.5.
        auto patchPrim = stage->GetPrimAtPath(SdfPath("/Asset/Patch"));
        CHECK(patchPrim.GetAttribute(TfToken("vForm")).Set(TfToken("periodic")));
        pts = EvaluateTarget(rig);
        if (pts.size() == 2) {
            CheckNear(pts[0], ExpectedArch(0.75, 0.75));
            CheckNear(pts[1], ExpectedArch(0.25, 0.5));
        }
        CheckFrozen(rig);
        CHECK(patchPrim.GetAttribute(TfToken("vForm")).Set(TfToken("open")));

        // Deform the NURBS patch via PatchDriver at the same time.
        auto patchDriver =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/PatchDriver"));
        CHECK(patchDriver.GetAttribute(TfToken("avars:tz")).Set(3.0));
        pts = EvaluateTarget(rig);
        if (pts.size() == 2) {
            CheckNear(pts[0], ExpectedArch(0.75, 0.75) + GfVec3f(0, 0, 3));
            CheckNear(pts[1], ExpectedArch(0.25, 1.0) + GfVec3f(0, 0, 3));
        }
        CheckFrozen(rig);
        CHECK(patchDriver.GetAttribute(TfToken("avars:tz")).Set(0.0));
        CHECK(uvDriver.GetAttribute(TfToken("avars:tx")).Set(0.0));
        CHECK(uvDriver.GetAttribute(TfToken("avars:ty")).Set(0.0));

        // Rational pointWeights: double the middle CV weights (w=3 vs w=1).
        // At u=0.5, B0=0.25, B1=0.5, B2=0.25. With w1=3:
        // W = 0.25*1 + 0.5*3 + 0.25*1 = 2.0
        // z = (0.5 * 3 * 2) / 2.0 = 1.5 (instead of 1.0).
        CHECK(patchPrim.GetAttribute(TfToken("pointWeights"))
                  .Set(VtDoubleArray{1.0, 3.0, 1.0, 1.0, 3.0, 1.0}));
        pts = EvaluateTarget(rig);
        if (pts.size() == 2) {
            CheckNear(pts[0], GfVec3f(1.0f, 1.0f, 1.5f));
            CheckNear(pts[1], GfVec3f(0.0f, 4.0f, 0.0f));
        }
        CheckFrozen(rig);
        CHECK(patchPrim.GetAttribute(TfToken("pointWeights"))
                  .Set(VtDoubleArray()));

        // Broadcast single UV point to all target vertices.
        auto uvPrim = stage->GetPrimAtPath(SdfPath("/Asset/UVs"));
        CHECK(uvPrim.GetAttribute(TfToken("points"))
                  .Set(VtVec3fArray{GfVec3f(0.5f, 0.5f, 0.0f)}));
        pts = EvaluateTarget(rig);
        if (pts.size() == 2) {
            CheckNear(pts[0], ExpectedArch(0.5, 0.5));
            CheckNear(pts[1], ExpectedArch(0.5, 0.5));
        }
        CheckFrozen(rig);

        // Common mover envelope (defaultWeight = 0.5) and enabled = false.
        auto stick = stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Stick"));
        CHECK(stick.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f));
        pts = EvaluateTarget(rig);
        if (pts.size() == 2) {
            CheckNear(pts[0],
                      GfVec3f(10, 20, 30) * 0.5f +
                          ExpectedArch(0.5, 0.5) * 0.5f);
            CheckNear(pts[1],
                      GfVec3f(-5, -6, -7) * 0.5f +
                          ExpectedArch(0.5, 0.5) * 0.5f);
        }
        CHECK(stick.GetAttribute(TfToken("inputs:enabled")).Set(false));
        pts = EvaluateTarget(rig);
        if (pts.size() == 2) {
            CheckNear(pts[0], GfVec3f(10, 20, 30));
            CheckNear(pts[1], GfVec3f(-5, -6, -7));
        }
    }

    // 3. RigBuilder AddPatchMover authoring & per-point SkinMover UV controllers.
    {
        auto stage = UsdStage::CreateInMemory();
        RigExecRigBuilder builder =
            RigExecRigBuilder::Create(stage, SdfPath("/Asset/Rig"));
        auto c0 = builder.AddControl("UvCtl0");
        auto c1 = builder.AddControl("UvCtl1");
        c0.SetAvarTranslation(0.25, 0.0, 0.0);
        c1.SetAvarTranslation(0.0, 0.75, 0.0);

        auto patch = UsdGeomNurbsPatch::Define(stage, SdfPath("/Asset/Patch"));
        patch.CreateUVertexCountAttr().Set(2);
        patch.CreateVVertexCountAttr().Set(2);
        patch.CreateUOrderAttr().Set(2);
        patch.CreateVOrderAttr().Set(2);
        patch.CreateUKnotsAttr().Set(VtDoubleArray{0, 0, 1, 1});
        patch.CreateVKnotsAttr().Set(VtDoubleArray{0, 0, 1, 1});
        patch.CreatePointsAttr().Set(VtVec3fArray{
            GfVec3f(0, 0, 0), GfVec3f(10, 0, 0),
            GfVec3f(0, 20, 0), GfVec3f(10, 20, 0)});

        auto uvs = UsdGeomPoints::Define(stage, SdfPath("/Asset/UVs"));
        uvs.CreatePointsAttr().Set(
            VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0.5f, 0, 0)});

        auto target = UsdGeomPoints::Define(stage, SdfPath("/Asset/Target"));
        target.CreatePointsAttr().Set(
            VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 0, 0)});

        // Note: movers execute bottom-sibling first, so AddPatchMover is
        // authored before AddSkinMover in child order so SkinMover runs first!
        auto targetChain =
            builder.NewMoverChain("TargetChain", SdfPath("/Asset/Target.points"));
        auto patchMover = targetChain.AddPatchMover(
            "StickToPatch", SdfPath("/Asset/Patch"), SdfPath("/Asset/UVs"),
            SdfPath("/Asset/Target.points"), TfToken("base"), TfToken("final"));
        CHECK(patchMover.GetPrim().GetTypeName() ==
              TfToken("RigExecPatchMover"));

        auto uvChain =
            builder.NewMoverChain("UvChain", SdfPath("/Asset/UVs.points"));
        auto skin = uvChain.AddSkinMover(
            "DriveUvs", {c0.GetPath(), c1.GetPath()});
        skin.SetJointInfluences({0, 1}, {1.0f, 1.0f}, 1);

        RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(rig.Compile(&errors));
        CHECK(errors.empty());
        VtVec3fArray pts = EvaluateTarget(rig);
        CHECK(pts.size() == 2);
        if (pts.size() == 2) {
            CheckNear(pts[0], GfVec3f(2.5f, 0.0f, 0.0f));
            CheckNear(pts[1], GfVec3f(5.0f, 15.0f, 0.0f));
        }
    }

    // 4. Validation rejects invalid patch topology, weights, or UV count.
    {
        auto stage = MakeStage();
        stage->GetPrimAtPath(SdfPath("/Asset/UVs"))
            .GetAttribute(TfToken("points"))
            .Set(VtVec3fArray{GfVec3f(0), GfVec3f(0), GfVec3f(0)});
        RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(rig.Compile(&errors));
        CHECK(!errors.empty());
        auto pose = rig.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        CHECK(pose.movedProperties.count(SdfPath("/Asset/Target.points")) == 0);
    }

    // 5. UV shapes: avars weight Smile and Open, which add to the neutral.
    {
        const GfVec3f neutral[2] = {GfVec3f(0.25f, 0.25f, 0), GfVec3f(0, 1, 0)};
        const GfVec3f smile[2] = {GfVec3f(0.75f, 0.25f, 0), GfVec3f(0.5f, 1, 0)};
        const GfVec3f open[2] = {GfVec3f(0.25f, 0.75f, 0), GfVec3f(0, 0.5f, 0)};
        // The target at weights (s, o): the patch at
        // uv = neutral + s (smile - neutral) + o (open - neutral).
        const auto expectAt = [&](const VtVec3fArray &pts, double s, double o) {
            CHECK(pts.size() == 2);
            for (size_t i = 0; i < 2 && i < pts.size(); ++i) {
                const double u = neutral[i][0] + s * (smile[i][0] - neutral[i][0]) +
                                 o * (open[i][0] - neutral[i][0]);
                const double v = neutral[i][1] + s * (smile[i][1] - neutral[i][1]) +
                                 o * (open[i][1] - neutral[i][1]);
                CheckNear(pts[i], ExpectedArch(u, v), 1e-5f);
            }
        };
        auto stage = MakeShapesStage();
        RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(rig.Compile(&errors));
        CHECK(errors.empty());
        std::vector<std::string> reasons;
        CHECK(rig.IsBakeable(&reasons));
        const UsdAttribute smileAvar =
            stage->GetAttributeAtPath(SdfPath("/Asset/Rig/Mouth.avars:smile"));
        const UsdAttribute openAvar =
            stage->GetAttributeAtPath(SdfPath("/Asset/Rig/Mouth.avars:open"));
        // Each alone, both together (they add), and a partial mix.
        for (const auto &[s, o] : std::vector<std::pair<float, double>>{
                 {0.0f, 0.0}, {1.0f, 0.0}, {0.0f, 1.0}, {1.0f, 1.0},
                 {0.5f, 0.25}}) {
            CHECK(smileAvar.Set(s));
            CHECK(openAvar.Set(o));
            expectAt(EvaluateTarget(rig), s, o);
            CheckFrozen(rig);
        }
        // A drag on an avar reaches the mover.
        CHECK(smileAvar.Set(0.0f));
        CHECK(openAvar.Set(0.0));
        rig.SetInteractiveOverrides({RigExecValueOverride{
            SdfPath("/Asset/Rig/Mouth"), TfToken(), TfToken("avars:open"),
            VtValue(1.0)}});
        expectAt(EvaluateTarget(rig), 0.0, 1.0);
        rig.ClearInteractiveOverrides();
        expectAt(EvaluateTarget(rig), 0.0, 0.0);

        // Unconnected, the authored array is the weights; an empty one
        // leaves every shape at zero.
        const UsdAttribute weights = stage->GetAttributeAtPath(
            SdfPath("/Asset/Rig/Movers/Stick.inputs:uvWeights"));
        CHECK(weights.ClearConnections());
        CHECK(weights.Set(VtFloatArray{0.25f, 0.5f}));
        CHECK(rig.Compile(&errors));
        expectAt(EvaluateTarget(rig), 0.25, 0.5);
        CheckFrozen(rig);
        CHECK(weights.Set(VtFloatArray()));
        CHECK(rig.Compile(&errors));
        expectAt(EvaluateTarget(rig), 0.0, 0.0);
    }
    // 0.625 is exact in float and double, so the held input and the float
    // edit the reference authors are the same avar value.
    CheckRuntime(MakeShapesStage(), "uv_shape_smile",
                 SdfPath("/Asset/Rig/Mouth.avars:smile"), 0.625);
    CheckRuntime(MakeShapesStage(), "uv_shape_open",
                 SdfPath("/Asset/Rig/Mouth.avars:open"), 0.5);

    // 6. Every shape has the neutral's point count, and the weights match
    // the shapes: one float or double source each, or an array of that size.
    // Contact lists are as long as each other, in range, and name a point
    // once.
    for (const char *edit : {"shape count", "connections", "source type",
                             "array size", "contact length", "contact range",
                             "contact repeat"}) {
        auto stage = MakeShapesStage();
        const UsdAttribute weights = stage->GetAttributeAtPath(
            SdfPath("/Asset/Rig/Movers/Stick.inputs:uvWeights"));
        const UsdPrim stick =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Stick"));
        const auto contacts = [&](VtIntArray lower, VtIntArray upper) {
            CHECK(stick.GetAttribute(TfToken("rigExec:uvContactLower"))
                      .Set(lower));
            CHECK(stick.GetAttribute(TfToken("rigExec:uvContactUpper"))
                      .Set(upper));
        };
        std::string expected;
        if (std::string(edit) == "contact length") {
            contacts({0}, {1, 0});
            expected = "rigExec:uvContactLower holds 1 indices";
        } else if (std::string(edit) == "contact range") {
            contacts({0}, {2});
            expected = "names point 2; rigExec:uvPoints has 2";
        } else if (std::string(edit) == "contact repeat") {
            contacts({0, 1}, {1, 0});
            expected = "name point 0 more than once";
        } else if (std::string(edit) == "shape count") {
            stage->GetPrimAtPath(SdfPath("/Asset/OpenUVs"))
                .GetAttribute(TfToken("points"))
                .Set(VtVec3fArray{GfVec3f(0), GfVec3f(0), GfVec3f(0)});
            expected = "UV shape";
        } else if (std::string(edit) == "connections") {
            weights.SetConnections({SdfPath("/Asset/Rig/Mouth.avars:smile")});
            expected = "connects 1 sources";
        } else if (std::string(edit) == "source type") {
            weights.SetConnections({SdfPath("/Asset/Rig/Mouth.avars:smile"),
                                    SdfPath("/Asset/Patch.uVertexCount")});
            expected = "is not a float or double";
        } else {
            weights.ClearConnections();
            weights.Set(VtFloatArray{1.0f, 1.0f, 1.0f});
            expected = "holds 3 weights";
        }
        RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        rig.Compile(&errors);
        bool named = false;
        for (const std::string &e : errors) {
            named = named || e.find(expected) != std::string::npos;
        }
        if (!named) {
            std::printf("%s: no \"%s\" error among %zu\n", edit,
                        expected.c_str(), errors.size());
        }
        CHECK(named);
    }

    // 7. Contacts: point 0 never rises above point 1 in v. Open lifts point
    // 0 by 0.5 and lowers point 1 by 0.5, so from open = 0.75 on the two
    // would cross; they meet at the mean of their v instead, and u (which
    // Smile moves) is left alone.
    {
        const auto expectTouching = [&](const VtVec3fArray &pts, double s,
                                        double o) {
            double u[2] = {0.25 + 0.5 * s, 0.5 * s};
            double v[2] = {0.25 + 0.5 * o, 1.0 - 0.5 * o};
            if (v[0] > v[1]) {
                v[0] = v[1] = 0.5 * (v[0] + v[1]);
            }
            CHECK(pts.size() == 2);
            for (size_t i = 0; i < 2 && i < pts.size(); ++i) {
                CheckNear(pts[i], ExpectedArch(u[i], v[i]), 1e-5f);
            }
        };
        auto stage = MakeShapesStage();
        const UsdPrim stick =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Stick"));
        CHECK(stick.GetAttribute(TfToken("rigExec:uvContactLower"))
                  .Set(VtIntArray{0}));
        CHECK(stick.GetAttribute(TfToken("rigExec:uvContactUpper"))
                  .Set(VtIntArray{1}));
        RigExecRigEvaluator rig(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(rig.Compile(&errors));
        CHECK(errors.empty());
        std::vector<std::string> reasons;
        CHECK(rig.IsBakeable(&reasons));
        const UsdAttribute smileAvar =
            stage->GetAttributeAtPath(SdfPath("/Asset/Rig/Mouth.avars:smile"));
        const UsdAttribute openAvar =
            stage->GetAttributeAtPath(SdfPath("/Asset/Rig/Mouth.avars:open"));
        // Apart, touching, crossing, and crossing past the patch's edge.
        for (const auto &[s, o] : std::vector<std::pair<float, double>>{
                 {0.0f, 0.5}, {0.0f, 0.75}, {0.0f, 1.0}, {1.0f, 1.0},
                 {0.5f, 2.5}}) {
            CHECK(smileAvar.Set(s));
            CHECK(openAvar.Set(o));
            const VtVec3fArray pts = EvaluateTarget(rig);
            expectTouching(pts, s, o);
            if (pts.size() == 2) {
                CHECK(pts[0][1] <= pts[1][1]);
            }
            CheckFrozen(rig);
        }
    }
    {
        // The same contact in baked playback, posed past contact.
        auto stage = MakeShapesStage();
        const UsdPrim stick =
            stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/Stick"));
        CHECK(stick.GetAttribute(TfToken("rigExec:uvContactLower"))
                  .Set(VtIntArray{0}));
        CHECK(stick.GetAttribute(TfToken("rigExec:uvContactUpper"))
                  .Set(VtIntArray{1}));
        CheckRuntime(stage, "uv_contact_open",
                     SdfPath("/Asset/Rig/Mouth.avars:open"), 1.0);
    }

    return failures == 0 ? 0 : 1;
}
