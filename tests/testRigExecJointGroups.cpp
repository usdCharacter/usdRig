// RigExecJointGroupAPI on a RigExecFkChain: two alternative chains under one
// group with a held rigExec:activeMember switching them across frames, a
// joint under an inactive member publishing its rest through the start
// frame, a disabled group keeping every member active, native, frozen and
// .rigexec parity (with the active member held in the runtime), structural
// group edits that recompile against a value edit that does not, and the
// compile refusals.
#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/relationship.h"

#include <cmath>
#include <cstdio>
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
const SdfPath kGroup("/Char/Rig/Groups/Heads");
const SdfPath kActive("/Char/Rig/Groups/Heads.rigExec:activeMember");
constexpr double kDeg = 3.14159265358979323846 / 180.0;

std::string Translate(double x, double y)
{
    return "( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (" + std::to_string(x) + ", " +
           std::to_string(y) + ", 0, 1) )";
}

// A torso with two alternative heads (A carries a jaw), as joints and as
// nested parent-relative controls, on one FK chain hanging from a start
// control moved +5 in X. The group's members are the heads; \p groupExtra
// is authored inside the group, \p chainExtra on the chain and \p after as
// further Rig children.
std::string Usda(const std::string &groupExtra = "", const std::string &chainExtra = "",
                 const std::string &after = "")
{
    const auto nest = [](const char *kind, const char *rz) {
        return std::string("            def ") + kind + " \"Torso\"\n            {\n"
               "                matrix4d rest:space = " + Translate(0, 0) + "\n"
               "                def " + kind + " \"HeadA\"\n                {\n"
               "                    matrix4d rest:space = " + Translate(0, 1) + "\n" + rz +
               "                    def " + kind + " \"JawA\"\n                    {\n"
               "                        matrix4d rest:space = " + Translate(0, 1.5) + "\n"
               "                    }\n                }\n"
               "                def " + kind + " \"HeadB\"\n                {\n"
               "                    matrix4d rest:space = " + Translate(0, 1) + "\n" + rz +
               "                }\n            }\n";
    };
    return std::string("#usda 1.0\n(\n    startTimeCode = 0\n    endTimeCode = 4\n)\n") +
           "def Xform \"Char\"\n{\n    def RigExecRoot \"Rig\"\n    {\n"
           "        def Scope \"Controls\"\n        {\n" +
           nest("RigExecControl", "                    double avars:rz = 30\n") +
           "            def RigExecControl \"Start\"\n            {\n"
           "                matrix4d rest:space = " + Translate(0, 0) + "\n"
           "                double avars:tx = 5\n            }\n        }\n"
           "        def Scope \"Joints\"\n        {\n" + nest("RigExecJoint", "") +
           "        }\n"
           "        def Scope \"Groups\"\n        {\n"
           "            def Scope \"Heads\" (prepend apiSchemas = [\"RigExecJointGroupAPI\"])\n"
           "            {\n"
           "                rel collection:members:includes = [</Char/Rig/Joints/Torso/HeadA>, "
           "</Char/Rig/Joints/Torso/HeadB>]\n"
           "                double rigExec:activeMember.spline = { 1: 0; post held, 2: 1; post held, }\n" +
           groupExtra + "            }\n        }\n"
           "        def RigExecFkChain \"Chain\"\n        {\n"
           "            uniform token rigExec:controlSpace = \"parentRelative\"\n"
           "            rel rigExec:startFrame = </Char/Rig/Controls/Start>\n"
           "            rel rigExec:controls = [</Char/Rig/Controls/Torso>, "
           "</Char/Rig/Controls/Torso/HeadA>, </Char/Rig/Controls/Torso/HeadA/JawA>, "
           "</Char/Rig/Controls/Torso/HeadB>]\n"
           "            rel rigExec:joints = [</Char/Rig/Joints/Torso>, "
           "</Char/Rig/Joints/Torso/HeadA>, </Char/Rig/Joints/Torso/HeadA/JawA>, "
           "</Char/Rig/Joints/Torso/HeadB>]\n"
           "            rel rigExec:jointGroups = </Char/Rig/Groups/Heads>\n" + chainExtra +
           "        }\n" + after + "    }\n}\n";
}

UsdStageRefPtr Stage(const std::string &text)
{
    auto stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(text));
    return stage;
}

SdfPath Joint(const std::string &name)
{
    return SdfPath(name == "Torso" ? "/Char/Rig/Joints/Torso"
                   : name == "JawA" ? "/Char/Rig/Joints/Torso/HeadA/JawA"
                                    : "/Char/Rig/Joints/Torso/" + name);
}

bool Compiles(RigExecRigEvaluator &rig)
{
    std::vector<std::string> errors;
    const bool ok = rig.Compile(&errors);
    if (!ok)
        for (const auto &e : errors) std::printf("  compile: %s\n", e.c_str());
    return ok;
}

// Refused with a message naming \p text: the compile sets the chain aside,
// or evaluation answers an invalid pose carrying the reason.
void Refused(const std::string &usda, const char *text)
{
    auto stage = Stage(usda);
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    const bool compiled = rig.Compile(&errors);
    bool named = false;
    for (const auto &e : errors) named = named || e.find(text) != std::string::npos;
    if (named) return;
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1));
    bool refused = false;
    for (const auto &r : pose.diagnostics) refused = refused || r.find(text) != std::string::npos;
    if (refused && compiled && !pose.valid) return;
    std::printf("FAIL refusal \"%s\": compiled=%d valid=%d\n", text, int(compiled),
                int(pose.valid));
    for (const auto &e : errors) std::printf("  %s\n", e.c_str());
    for (const auto &r : pose.diagnostics) std::printf("  reason: %s\n", r.c_str());
    ++failures;
}

RigExecRigPose At(RigExecRigEvaluator &rig, double time)
{
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(time));
    CHECK(pose.valid);
    for (const auto &line : pose.diagnostics)
        if (line.find("Chain") != std::string::npos)
            std::printf("  diagnostic at %g: %s\n", time, line.c_str());
    return pose;
}

struct Frame2 {
    double x = NAN, y = NAN, angle = NAN;
};

Frame2 FrameOf(const RigExecRigPose &pose, const std::string &joint)
{
    Frame2 f;
    const auto found = pose.jointFramesFinal.find(Joint(joint));
    CHECK(found != pose.jointFramesFinal.end());
    if (found == pose.jointFramesFinal.end()) return f;
    const auto &p = found->second.points;
    f.x = p[0][0];
    f.y = p[0][1];
    f.angle = std::atan2(p[1][1] - p[0][1], p[1][0] - p[0][0]) / kDeg;
    return f;
}

void Near(double actual, double expected, const std::string &what)
{
    if (!(std::fabs(actual - expected) <= 1e-9)) {
        std::printf("FAIL %s: %.12f, expected %.12f\n", what.c_str(), actual, expected);
        ++failures;
    }
}

void At(const RigExecRigPose &pose, const std::string &joint, double x, double y, double angle,
        const std::string &what)
{
    const Frame2 f = FrameOf(pose, joint);
    Near(f.x, x, what + " x");
    Near(f.y, y, what + " y");
    Near(f.angle, angle, what + " angle");
}

// Native, frozen and .rigexec agree bit for bit on every frame of \p times,
// and with the active member held at \p held in the runtime.
void Parity(const UsdStageRefPtr &stage, const std::string &what,
            const std::vector<double> &times, double held)
{
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    std::vector<std::string> reasons;
    CHECK(rig.IsBakeable(&reasons));
    for (const auto &r : reasons) std::printf("  %s not bakeable: %s\n", what.c_str(), r.c_str());
    for (const double t : times) {
        std::string why;
        const bool same = RigExecFrozenMatchesLive(&rig, kRig, UsdTimeCode(t), &why);
        if (!same) std::printf("%s frozen at %g: %s\n", what.c_str(), t, why.c_str());
        CHECK(same);
    }
    std::vector<uint8_t> bytes;
    std::string error;
    const bool baked = RigExecTestBakeAt(rig, times.front(), &bytes, &error);
    CHECK(baked);
    if (!baked) { std::printf("%s bake: %s\n", what.c_str(), error.c_str()); return; }
    RigExecTestPlayer player;
    const bool opened = player.Open(bytes, stage, &error);
    CHECK(opened);
    if (!opened) { std::printf("%s open: %s\n", what.c_str(), error.c_str()); return; }
    const auto compare = [&](const RigExecRigPose &pose, const char *phase, double time) {
        std::vector<std::string> diffs;
        const bool same = RigExecCompareRuntimeOutputs(pose, player.Reader(), &diffs);
        if (!same)
            for (const auto &line : diffs)
                std::printf("%s %s %g: %s\n", what.c_str(), phase, time, line.c_str());
        CHECK(same);
    };
    for (const double time : times) {
        const bool played = player.Play(time, &error);
        CHECK(played);
        if (!played) { std::printf("%s play: %s\n", what.c_str(), error.c_str()); return; }
        compare(rig.Evaluate(UsdTimeCode(time)), "sampled", time);
        CHECK(!player.Reader().GetJointMatrices().empty());
    }
    const bool heldPlay =
        player.Hold(kActive.GetString(), held, &error) && player.Play(times.front(), &error);
    CHECK(heldPlay);
    if (!heldPlay) { std::printf("%s held play: %s\n", what.c_str(), error.c_str()); return; }
    std::vector<RigExecRigPose> poses;
    const bool referenced = RigExecTestEditedPoses(stage, kRig, {{kActive, VtValue(held)}},
                                                   {times.front()}, &poses, &error);
    CHECK(referenced);
    if (referenced) compare(poses.front(), "held", times.front());
    else std::printf("%s held reference: %s\n", what.c_str(), error.c_str());
}

// One member active per frame; the other chain at rest through the start
// frame; a disabled group keeps both heads posed.
void AlternativeChains()
{
    auto stage = Stage(Usda());
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const double jawX = 5 - 1.5 * std::sin(30 * kDeg), jawY = 1 + 1.5 * std::cos(30 * kDeg);
    const RigExecRigPose first = At(rig, 1);
    At(first, "Torso", 5, 0, 0, "torso");
    At(first, "HeadA", 5, 1, 30, "head A active");
    At(first, "JawA", jawX, jawY, 30, "jaw rides head A");
    At(first, "HeadB", 5, 1, 0, "head B rests through the start");
    const RigExecRigPose second = At(rig, 2);
    At(second, "HeadA", 5, 1, 0, "head A rests");
    At(second, "JawA", 5, 2.5, 0, "jaw rests under inactive head A");
    At(second, "HeadB", 5, 1, 30, "head B active");
    // Held keys step: frame 1.9 still shows head A.
    At(At(rig, 1.9), "HeadA", 5, 1, 30, "held key");
    Parity(stage, "groups", {1, 2, 3}, 1);
    // A value edit to the active member re-evaluates without a recompile.
    const size_t signature = rig.GetCompiledRigSignature();
    UsdAttribute active = stage->GetAttributeAtPath(kActive);
    CHECK(active.Clear());
    CHECK(active.Set(1.0));
    At(At(rig, 1), "HeadB", 5, 1, 30, "active member edited");
    CHECK(rig.GetCompiledRigSignature() == signature);
    // Disabling is structural: both heads pose.
    UsdPrim group = stage->GetPrimAtPath(kGroup);
    CHECK(group.GetAttribute(TfToken("rigExec:enabled")).Set(false));
    const RigExecRigPose disabled = At(rig, 1);
    At(disabled, "HeadA", 5, 1, 30, "disabled group head A");
    At(disabled, "HeadB", 5, 1, 30, "disabled group head B");
    const size_t disabledSignature = rig.GetCompiledRigSignature();
    CHECK(disabledSignature != signature);
    Parity(stage, "disabled", {1, 2}, 0);
    // Membership is structural: excluding head B leaves head A the only
    // member, active at every frame.
    CHECK(group.GetAttribute(TfToken("rigExec:enabled")).Set(true));
    CHECK(group.CreateRelationship(TfToken("collection:members:excludes"))
              .SetTargets({Joint("HeadB")}));
    const RigExecRigPose single = At(rig, 1);
    At(single, "HeadA", 5, 1, 30, "lone member active");
    At(single, "HeadB", 5, 1, 30, "excluded head is ungrouped");
    CHECK(rig.GetCompiledRigSignature() != disabledSignature);
    const size_t excludedSignature = rig.GetCompiledRigSignature();
    // The group prim's activity is structural: inactive refuses the chain
    // and reactivating recompiles.
    CHECK(group.SetActive(false));
    CHECK(!rig.Evaluate(UsdTimeCode(1)).valid);
    CHECK(rig.GetCompiledRigSignature() != excludedSignature);
    CHECK(group.SetActive(true));
    At(At(rig, 1), "HeadA", 5, 1, 30, "group reactivated");
    CHECK(rig.GetCompiledRigSignature() == excludedSignature);
    // Dropping the group from the chain is structural too.
    CHECK(stage->GetPrimAtPath(SdfPath("/Char/Rig/Chain"))
              .GetRelationship(TfToken("rigExec:jointGroups"))
              .ClearTargets(true));
    At(At(rig, 2), "HeadA", 5, 1, 30, "ungrouped chain");
    CHECK(rig.GetCompiledRigSignature() != excludedSignature);
}

void Refusals()
{
    // Each rule on its own; the stage is otherwise valid.
    Refused(Usda("                uniform token collection:members:expansionRule = \"expandPrims\"\n"),
            "must list its members explicitly");
    Refused(Usda("                pathExpression collection:members:membershipExpression = "
                 "\"/Char/Rig/Joints/*\"\n"),
            "membershipExpression");
    Refused(Usda("                prepend rel collection:members:includes = "
                 "</Char/Rig/Controls/Start>\n"),
            "not a joint of this solver");
    Refused(Usda("                uniform bool rigExec:enabled.timeSamples = { 1: 1, 2: 0 }\n"),
            "must not be animated");
    // A joint group's target must carry the API.
    Refused(Usda("", "            rel rigExec:jointGroups = </Char/Rig/Joints>\n"),
            "not an active prim with RigExecJointGroupAPI");
    // A joint is a member of one group at most.
    const std::string again =
        "        def Scope \"Again\" (prepend apiSchemas = [\"RigExecJointGroupAPI\"])\n"
        "        {\n"
        "            rel collection:members:includes = </Char/Rig/Joints/Torso/HeadB>\n"
        "        }\n";
    std::string twice = Usda("", "", again);
    const std::string single = "            rel rigExec:jointGroups = </Char/Rig/Groups/Heads>\n";
    const size_t at = twice.find(single);
    CHECK(at != std::string::npos);
    if (at != std::string::npos) {
        twice.replace(at, single.size(),
                      "            rel rigExec:jointGroups = [</Char/Rig/Groups/Heads>, "
                      "</Char/Rig/Again>]\n");
    }
    Refused(twice, "is a member of two joint groups");
}

}  // namespace

int main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    AlternativeChains();
    Refusals();
    if (failures) {
        std::printf("testRigExecJointGroups: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecJointGroups: ok\n");
    return 0;
}
