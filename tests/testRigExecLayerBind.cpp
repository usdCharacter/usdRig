// The 2D layer bind: a transform-domain RigExecMatrixMover reading a bone at
// final carries a group by the bone's setup-to-pose map. The group lands
// where its content's points land, flips and cross scale included, with the
// group under a translated, scaled parent; native, frozen and .rigexec agree.
// The tests/fixtures/layerBind goldens come from: rigExecPose <fixture>.usda --rig /Char/Rig --frames 0,1,2,3,4 --golden-out <fixture>.golden
#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

static int failures = 0;
#define CHECK(x) do { if (!(x)) { std::printf("FAIL %d: %s\n", __LINE__, #x); ++failures; } } while (0)
#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"

namespace {
const SdfPath kRig("/Char/Rig");
const SdfPath kBone("/Char/Rig/Bones/FC");

void Near(double actual, double expected, double tolerance, const char *what, double at)
{
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s at %g: %.9f, expected %.9f (+-%g)\n", what, at, actual, expected,
                    tolerance);
        ++failures;
    }
}

bool Compiles(RigExecRigEvaluator &rig)
{
    std::vector<std::string> errors;
    const bool ok = rig.Compile(&errors);
    if (!ok)
        for (const auto &e : errors) std::printf("  compile: %s\n", e.c_str());
    return ok;
}

// The frame-cache worker reproduces live evaluation bit for bit.
void Frozen(RigExecRigEvaluator &rig, double time, const char *what)
{
    std::string why;
    const bool same = RigExecFrozenMatchesLive(&rig, kRig, UsdTimeCode(time), &why);
    if (!same) std::printf("%s frozen at %g: %s\n", what, time, why.c_str());
    CHECK(same);
}

UsdStageRefPtr Stage(const char *name)
{
    std::ifstream file(std::string(RIGEXEC_LAYER_BIND_FIXTURES) + "/" + name);
    std::stringstream text;
    text << file.rdbuf();
    auto stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(text.str()));
    return stage;
}

// A .rigexec baked from \p stage plays every time bit for bit as native
// evaluation does, unposed and with \p avar held at \p posed.
void Runtime(const UsdStageRefPtr &stage, const char *what, const std::vector<double> &times,
             const SdfPath &avar, double posed)
{
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    std::vector<uint8_t> bytes;
    std::string error;
    const bool baked = RigExecTestBakeAt(rig, 1, &bytes, &error);
    CHECK(baked);
    if (!baked) { std::printf("%s bake: %s\n", what, error.c_str()); return; }
    RigExecTestPlayer player;
    const bool opened = player.Open(bytes, stage, &error);
    CHECK(opened);
    if (!opened) { std::printf("%s open: %s\n", what, error.c_str()); return; }
    const auto compare = [&](const RigExecRigPose &pose, const char *phase, double time) {
        std::vector<std::string> diffs;
        const bool same = RigExecCompareRuntimeOutputs(pose, player.Reader(), &diffs);
        if (!same)
            for (const auto &line : diffs) std::printf("%s %s %g: %s\n", what, phase, time, line.c_str());
        CHECK(same);
    };
    for (const double time : times) {
        const bool played = player.Play(time, &error);
        CHECK(played);
        if (!played) { std::printf("%s play: %s\n", what, error.c_str()); return; }
        compare(rig.Evaluate(UsdTimeCode(time)), "unposed", time);
    }
    const bool held = player.Hold(avar.GetString(), posed, &error) && player.Play(times.back(), &error);
    CHECK(held);
    if (!held) { std::printf("%s posed play: %s\n", what, error.c_str()); return; }
    std::vector<RigExecRigPose> poses;
    const bool referenced = RigExecTestEditedPoses(stage, kRig, {{avar, VtValue(posed)}},
                                                   {times.back()}, &poses, &error);
    CHECK(referenced);
    if (referenced) compare(poses.front(), "posed", times.back());
    else std::printf("%s posed reference: %s\n", what, error.c_str());
}

void LayerBind()
{
    auto stage = Stage("layer_bind.usda");
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const SdfPath head("/Char/Art/Parent/Head");
    const GfVec3d marker(-0.25, -1.05, 0);  // local; world (0.5, -0.1) at setup
    const double expected[5][2] = {
        {0.5, -0.1},     // rest
        {0.5, -0.3},     // side flip: +y content below the bone
        {0.3, -0.1},     // end flip: extends backward from the base
        {0.6, -0.15},    // scale 2 along, 0.5 across
        {0.4 + 0.1 * std::cos(M_PI / 6) - 0.1 * std::sin(M_PI / 6) + 0.1,
         -0.2 + 0.1 * std::sin(M_PI / 6) + 0.1 * std::cos(M_PI / 6) + 0.05}};
    for (int t = 0; t <= 4; ++t) {
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(t));
        CHECK(pose.valid);
        const auto revised = pose.providerXforms.find(head);
        const auto base = pose.providerBaseXforms.find(head);
        const auto bone = pose.jointMatricesFinal.find(kBone);
        CHECK(revised != pose.providerXforms.end() && base != pose.providerBaseXforms.end() &&
              bone != pose.jointMatricesFinal.end());
        if (revised == pose.providerXforms.end() || base == pose.providerBaseXforms.end() ||
            bone == pose.jointMatricesFinal.end()) continue;
        const GfVec3d world = revised->second.Transform(marker);
        Near(world[0], expected[t][0], 1e-9, "layer bind x", t);
        Near(world[1], expected[t][1], 1e-9, "layer bind y", t);
        // "A transform lands where a point lands": revised = base * D_k.
        const GfMatrix4d carried = base->second * bone->second;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                Near(revised->second[r][c], carried[r][c], 1e-12, "revised = base * D", t);
        if (t == 3) CHECK(revised->second.GetDeterminant() > 0);
        if (t == 1 || t == 2) CHECK(revised->second.GetDeterminant() < 0);  // mirrored
        Frozen(rig, t, "layer bind");
    }
    // The same group without the transformed parent gives the same world
    // result.
    auto flat = Stage("layer_bind.usda");
    UsdGeomXformable parent(flat->GetPrimAtPath(SdfPath("/Char/Art/Parent")));
    parent.ClearXformOpOrder();
    flat->GetPrimAtPath(SdfPath("/Char/Art/Parent/Head/Marker")).GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0.5f, -0.1f, 0)});
    RigExecRigEvaluator flatRig(flat, kRig);
    CHECK(Compiles(flatRig));
    for (int t = 0; t <= 4; ++t) {
        const RigExecRigPose pose = flatRig.Evaluate(UsdTimeCode(t));
        const auto revised = pose.providerXforms.find(head);
        CHECK(revised != pose.providerXforms.end());
        if (revised == pose.providerXforms.end()) continue;
        const GfVec3d world = revised->second.Transform(GfVec3d(GfVec3f(0.5f, -0.1f, 0)));
        Near(world[0], expected[t][0], 1e-7, "flat layer bind x", t);
        Near(world[1], expected[t][1], 1e-7, "flat layer bind y", t);
    }
    Runtime(stage, "layer bind", {0, 1, 2, 3, 4}, SdfPath("/Char/Rig/Bones/FC.avars:rz"), 45);
}

}  // namespace

int main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    LayerBind();
    std::printf("testRigExecLayerBind: %s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
