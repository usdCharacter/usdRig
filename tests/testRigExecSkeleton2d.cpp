// RigExecSkeleton2d in a rig over nested bones (parent-relative rests, tips
// from child joints): the independent-angle, lock, cross-scale, flip and
// parent-scale numbers measured through the whole pipeline (joint frames and
// rest->pose matrices); native, frozen and .rigexec parity on every test
// frame; value edits (rests, avars) that re-evaluate with the signature
// unchanged; structural edits (limits, a namespace reparent) that recompile;
// held lock references with two lock keys; a lock solve the angle limits
// bind; math movers on a bone's avar reaching the solve; live rests (a pose
// step below the skeleton hands it a bone's frame); tips from child bones
// and End joints; a skeleton hanging from a tilted, constrained start
// frame; space switches on bones (same-skeleton, outside and base sources,
// whole and fractional indices); the compile refusals; rigging (offsets, IK
// targets, control bones, joint groups) with parity, value/structural edits
// and refusals; and no stage authoring.
#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/layer.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
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
constexpr double kDeg = 3.14159265358979323846 / 180.0;

// Where the last Usda() put each bone (bones nest under their parents).
std::map<std::string, std::string> &BonePaths()
{
    static std::map<std::string, std::string> paths;
    return paths;
}

SdfPath Bone(const std::string &name)
{
    const auto found = BonePaths().find(name);
    return SdfPath(found != BonePaths().end() ? found->second : "/Char/Rig/Bones/" + name);
}

struct BoneSpec {
    std::string name, parent;
    double x = 0, y = 0, angle = 0;  ///< the absolute setup frame
    double end = 0;     ///< an End joint this far along +X (0: none)
    std::string extra;  ///< authored lines inside the joint
};

struct GroupSpec {
    std::string name;
    std::vector<std::string> members;
    bool enabled = true;
    std::string extra;  ///< authored lines inside the group
};

std::string Num(double v)
{
    char text[64];
    std::snprintf(text, sizeof(text), "%.17g", v);
    return text;
}

/// The base of a bone at the tip of \p parent (length \p length), turned
/// \p local degrees from it.
BoneSpec AtTip(const BoneSpec &parent, double length, double local, const std::string &name)
{
    BoneSpec b;
    b.name = name;
    b.parent = parent.name;
    b.x = parent.x + length * std::cos(parent.angle * kDeg);
    b.y = parent.y + length * std::sin(parent.angle * kDeg);
    b.angle = parent.angle + local;
    return b;
}

/// The path a bone of \p bones takes: nested under its parent's.
std::string BonePath(const std::vector<BoneSpec> &bones, const std::string &name)
{
    for (const BoneSpec &b : bones) {
        if (b.name != name) continue;
        for (const BoneSpec &p : bones) {
            if (p.name == b.parent) return BonePath(bones, p.name) + "/" + name;
        }
        return "/Char/Rig/Bones/" + name;
    }
    return "/Char/Rig/Bones/" + name;
}

/// \p b's rest:space relative to \p parent (null: absolute).
std::string RestSpace(const BoneSpec &b, const BoneSpec *parent)
{
    double x = b.x, y = b.y, angle = b.angle;
    if (parent) {
        const double c = std::cos(parent->angle * kDeg), n = std::sin(parent->angle * kDeg);
        const double dx = b.x - parent->x, dy = b.y - parent->y;
        x = dx * c + dy * n;
        y = -dx * n + dy * c;
        angle = b.angle - parent->angle;
    }
    const double c = std::cos(angle * kDeg), n = std::sin(angle * kDeg);
    return "( (" + Num(c) + ", " + Num(n) + ", 0, 0), (" + Num(-n) + ", " + Num(c) +
           ", 0, 0), (0, 0, 1, 0), (" + Num(x) + ", " + Num(y) + ", 0, 1) )";
}

void EmitBone(std::ostringstream &s, const std::vector<BoneSpec> &bones, const BoneSpec &b,
              const BoneSpec *parent, size_t depth)
{
    const std::string pad(12 + 4 * depth, ' ');
    s << pad << "def RigExecJoint \"" << b.name
      << "\" (prepend apiSchemas = [\"RigExecBone2dAPI\"])\n" << pad << "{\n"
      << pad << "    matrix4d rest:space = " << RestSpace(b, parent) << "\n" << b.extra;
    if (b.end > 0)
        s << pad << "    def RigExecJoint \"End\"\n" << pad << "    {\n" << pad
          << "        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), ("
          << Num(b.end) << ", 0, 0, 1) )\n" << pad << "    }\n";
    for (const BoneSpec &child : bones)
        if (child.parent == b.name) EmitBone(s, bones, child, &b, depth + 1);
    s << pad << "}\n";
}

/// A 2D rig: one skeleton over \p bones (rigExec:joints in list order, each
/// nested under its parent with a parent-relative rest) with \p groups
/// nested inside it, \p skeletonExtra authored on the skeleton, \p before
/// authored as Rig children above the skeleton and \p after below the bones
/// (the bottom sibling fires first).
std::string Usda(const std::vector<BoneSpec> &bones, const std::string &before = "",
                 const std::string &after = "",
                 const std::vector<GroupSpec> &groups = {},
                 const std::string &skeletonExtra = "")
{
    BonePaths().clear();
    for (const BoneSpec &b : bones) BonePaths()[b.name] = BonePath(bones, b.name);
    std::ostringstream s;
    s << "#usda 1.0\n(\n    startTimeCode = 0\n    endTimeCode = 30\n)\n"
      << "def Xform \"Char\"\n{\n    def RigExecRoot \"Rig\"\n    {\n" << before
      << "        def RigExecSkeleton2d \"Skeleton\"\n        {\n" << skeletonExtra
      << "            rel rigExec:joints = [";
    for (size_t i = 0; i < bones.size(); ++i)
        s << (i ? ", " : "") << "<" << BonePath(bones, bones[i].name) << ">";
    s << "]\n";
    if (!groups.empty()) {
        s << "            rel rigExec:jointGroups = [";
        for (size_t i = 0; i < groups.size(); ++i)
            s << (i ? ", " : "") << "</Char/Rig/Skeleton/" << groups[i].name << ">";
        s << "]\n";
        for (const GroupSpec &g : groups) {
            s << "            def Scope \"" << g.name
              << "\" (prepend apiSchemas = [\"RigExecJointGroupAPI\"])\n            {\n"
              << "                rel collection:members:includes = [";
            for (size_t i = 0; i < g.members.size(); ++i)
                s << (i ? ", " : "") << "<" << BonePath(bones, g.members[i]) << ">";
            s << "]\n"
              << "                uniform bool rigExec:enabled = " << (g.enabled ? "1" : "0") << "\n"
              << g.extra << "            }\n";
        }
    }
    s << "        }\n        def Scope \"Bones\"\n        {\n";
    for (const BoneSpec &b : bones) {
        bool nested = false;
        for (const BoneSpec &p : bones) nested = nested || p.name == b.parent;
        if (!nested) EmitBone(s, bones, b, nullptr, 0);
    }
    s << "        }\n" << after << "    }\n}\n";
    return s.str();
}

UsdStageRefPtr Stage(const std::string &text)
{
    auto stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(text));
    return stage;
}

bool Compiles(RigExecRigEvaluator &rig)
{
    std::vector<std::string> errors;
    const bool ok = rig.Compile(&errors);
    if (!ok)
        for (const auto &e : errors) std::printf("  compile: %s\n", e.c_str());
    return ok;
}

// The skeleton is refused with a message naming \p text, by one of the two
// routes every evaluator shares: the compile sets the operation aside (a
// failed or warning compile), or the program build refuses it (evaluation
// answers an invalid pose carrying the reason; frozen and bake build the
// same program).
void Refused(const std::string &usda, const char *text)
{
    auto stage = Stage(usda);
    RigExecRigEvaluator rig(stage, kRig);
    std::vector<std::string> errors;
    const bool compiled = rig.Compile(&errors);
    bool named = false, skipped = false;
    for (const auto &e : errors) {
        const bool names = e.find(text) != std::string::npos;
        named = named || names;
        skipped = skipped || (names && e.find("Skipped operation /Char/Rig/Skeleton") !=
                                           std::string::npos);
    }
    if (named && (!compiled || skipped)) return;
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1));
    const bool invalid = compiled && !pose.valid;
    const std::vector<std::string> &reasons = pose.diagnostics;
    bool refused = false;
    for (const auto &r : reasons) refused = refused || r.find(text) != std::string::npos;
    if (refused && invalid) return;
    std::printf("FAIL refusal \"%s\": compiled=%d invalid=%d\n", text, int(compiled),
                int(invalid));
    for (const auto &e : errors) std::printf("  %s\n", e.c_str());
    for (const auto &r : reasons) std::printf("  reason: %s\n", r.c_str());
    ++failures;
}

RigExecRigPose At(RigExecRigEvaluator &rig, double time)
{
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(time));
    CHECK(pose.valid);
    for (const auto &line : pose.diagnostics)
        if (line.find("Skeleton") != std::string::npos || line.find("2D") != std::string::npos)
            std::printf("  diagnostic at %g: %s\n", time, line.c_str());
    return pose;
}

/// A joint's published frame: base, unit-free axes (the deform frame).
struct Frame2 {
    double x = NAN, y = NAN, xAxisX = NAN, xAxisY = NAN, yAxisX = NAN, yAxisY = NAN;
    double Angle() const { return std::atan2(xAxisY, xAxisX) / kDeg; }
    double AlongScale() const { return std::hypot(xAxisX, xAxisY); }
    double CrossScale() const { return std::hypot(yAxisX, yAxisY); }
};

Frame2 FrameOf(const RigExecRigPose &pose, const std::string &bone)
{
    Frame2 f;
    const auto found = pose.jointFramesFinal.find(Bone(bone));
    CHECK(found != pose.jointFramesFinal.end());
    if (found == pose.jointFramesFinal.end()) return f;
    const auto &p = found->second.points;
    f.x = p[0][0]; f.y = p[0][1];
    f.xAxisX = p[1][0] - p[0][0]; f.xAxisY = p[1][1] - p[0][1];
    f.yAxisX = p[2][0] - p[0][0]; f.yAxisY = p[2][1] - p[0][1];
    return f;
}

/// Where the bone's rest->pose map takes setup-space point (x, y): a point
/// rigidly bound to the bone.
GfVec3d Mapped(const RigExecRigPose &pose, const std::string &bone, double x, double y)
{
    const auto found = pose.jointMatricesFinal.find(Bone(bone));
    CHECK(found != pose.jointMatricesFinal.end());
    if (found == pose.jointMatricesFinal.end()) return GfVec3d(NAN);
    return found->second.Transform(GfVec3d(x, y, 0));
}

void Near(double actual, double expected, double tolerance, const std::string &what)
{
    if (!(std::fabs(actual - expected) <= tolerance)) {
        std::printf("FAIL %s: %.9f, expected %.9f (+-%g)\n", what.c_str(), actual, expected,
                    tolerance);
        ++failures;
    }
}

double Wrap(double degrees)
{
    while (degrees > 180) degrees -= 360;
    while (degrees <= -180) degrees += 360;
    return degrees;
}

// The frame-cache worker reproduces live evaluation bit for bit.
void Frozen(RigExecRigEvaluator &rig, double time, const std::string &what)
{
    std::string why;
    const bool same = RigExecFrozenMatchesLive(&rig, kRig, UsdTimeCode(time), &why);
    if (!same) std::printf("%s frozen at %g: %s\n", what.c_str(), time, why.c_str());
    CHECK(same);
}

// A .rigexec baked from \p stage plays every time bit for bit as native
// evaluation does, and with \p avar held at \p posed.
void Runtime(const UsdStageRefPtr &stage, const std::string &what,
             const std::vector<double> &times, const SdfPath &avar = SdfPath(),
             double posed = 0)
{
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
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
        compare(rig.Evaluate(UsdTimeCode(time)), "unposed", time);
        CHECK(!player.Reader().GetJointMatrices().empty());  // not a vacuous match
    }
    if (avar.IsEmpty()) return;
    const bool held =
        player.Hold(avar.GetString(), posed, &error) && player.Play(times.back(), &error);
    CHECK(held);
    if (!held) { std::printf("%s posed play: %s\n", what.c_str(), error.c_str()); return; }
    std::vector<RigExecRigPose> poses;
    const bool referenced = RigExecTestEditedPoses(stage, kRig, {{avar, VtValue(posed)}},
                                                   {times.back()}, &poses, &error);
    CHECK(referenced);
    if (referenced) compare(poses.front(), "posed", times.back());
    else std::printf("%s posed reference: %s\n", what.c_str(), error.c_str());
}

// Native, frozen and .rigexec agree on every frame of \p times.
void Parity(const UsdStageRefPtr &stage, const std::string &what,
            const std::vector<double> &times, const SdfPath &avar = SdfPath(),
            double posed = 0)
{
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    std::vector<std::string> reasons;
    const bool bakeable = rig.IsBakeable(&reasons);
    CHECK(bakeable);
    for (const auto &r : reasons) std::printf("  %s not bakeable: %s\n", what.c_str(), r.c_str());
    for (const double t : times) Frozen(rig, t, what);
    Runtime(stage, what, times, avar, posed);
}

// Independent-angle rig: Q root, F at Q's tip, independent angle.
std::vector<BoneSpec> IndependentRig(bool independent, const std::string &qExtra, const std::string &fExtra)
{
    BoneSpec q;
    q.name = "Q"; q.x = 0; q.y = 0.6; q.angle = 20;
    q.extra = qExtra;
    BoneSpec f = AtTip(q, 0.3, 30, "F");
    f.extra = fExtra + (independent ? "                uniform bool bone2d:independentAngle = 1\n" : "");
    return {q, f};
}

const char *kIndependentQ = "                double avars:rz.timeSamples = { 1: 0, 2: 40, 3: 40, 4: -30, }\n";
const char *kIndependentF = "                double avars:rz.timeSamples = { 1: 0, 2: 0, 3: 20, 4: 0, }\n";

void IndependentAngle()
{
    auto stage = Stage(Usda(IndependentRig(true, kIndependentQ, kIndependentF)));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    std::string before, after, sessionBefore, sessionAfter;
    stage->GetRootLayer()->ExportToString(&before);
    stage->GetSessionLayer()->ExportToString(&sessionBefore);
    const double expected[] = {50, 50, 70, 50};
    for (int frame = 1; frame <= 4; ++frame) {
        const RigExecRigPose pose = At(rig, frame);
        const Frame2 q = FrameOf(pose, "Q"), f = FrameOf(pose, "F");
        Near(Wrap(f.Angle()), expected[frame - 1], 1e-9, "independent angle F world angle " + std::to_string(frame));
        // F's base on Q's tip throughout.
        Near(f.x, q.x + 0.3 * q.xAxisX, 1e-12, "independent angle F base x");
        Near(f.y, q.y + 0.3 * q.xAxisY, 1e-12, "independent angle F base y");
        // A joint's rest->pose map is the identity at the setup frame.
        if (frame == 1) {
            const GfVec3d p = Mapped(pose, "F", 0.1, 0.7);
            Near(p[0], 0.1, 1e-12, "setup identity x");
            Near(p[1], 0.7, 1e-12, "setup identity y");
        }
    }
    stage->GetRootLayer()->ExportToString(&after);
    stage->GetSessionLayer()->ExportToString(&sessionAfter);
    CHECK(before == after && sessionBefore == sessionAfter);  // evaluation authors nothing
    // Reference: the same rig without the flag turns F to 90 at frame 2.
    auto reference = Stage(Usda(IndependentRig(false, kIndependentQ, kIndependentF)));
    RigExecRigEvaluator plain(reference, kRig);
    CHECK(Compiles(plain));
    Near(Wrap(FrameOf(At(plain, 2), "F").Angle()), 90, 1e-9, "independent angle reference");
    Parity(stage, "independent angle", {1, 2, 3, 4}, Bone("Q").AppendProperty(TfToken("avars:rz")), 25);
}

// Lock rig: the locked foot holds while the hip moves, then lifts.
// S is limited to [-150, 0] (setup local -10 plus [-140, 10]) unless
// \p limited is false.
std::vector<BoneSpec> LegRig(const std::string &hExtra, std::string fExtra, double *lockX,
                            double *lockY, bool limited = true)
{
    BoneSpec h;
    h.name = "H"; h.x = 0; h.y = 0.3; h.angle = -90;
    h.extra = hExtra;
    BoneSpec t = AtTip(h, 0.05, 5, "T");
    BoneSpec s = AtTip(t, 0.4, -10, "S");
    if (limited)
        s.extra = "                uniform bool bone2d:angleLimits = 1\n"
                  "                uniform double bone2d:minAngle = -140\n"
                  "                uniform double bone2d:maxAngle = 10\n";
    BoneSpec f = AtTip(s, 0.4, 95, "F");
    *lockX = f.x;
    *lockY = f.y;
    f.extra = fExtra;
    return {h, t, s, f};
}

void Lock()
{
    double lockX = 0, lockY = 0;
    LegRig("", "", &lockX, &lockY);
    const std::string hip =
        "                double avars:tx.timeSamples = { 5: 0, 6: 0.05, 7: 0.1, 8: 0.15, 9: 0.2, 10: 0.2, }\n"
        "                double avars:ty.timeSamples = { 5: 0, 6: -0.03, 7: -0.06, 8: -0.09, 9: -0.12, 10: 0.38, }\n";
    const std::string foot =
        "                bool bone2d:lock.timeSamples = { 0: 0, 5: 1, }\n"
        "                double bone2d:lockX = " + Num(lockX) + "\n"
        "                double bone2d:lockY = " + Num(lockY) + "\n"
        "                double bone2d:lockAngle = 0\n";
    auto stage = Stage(Usda(LegRig(hip, foot, &lockX, &lockY)));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const Frame2 at5 = FrameOf(At(rig, 5), "F");
    Near(at5.x, lockX, 1e-12, "lock frame 5 base x");
    Near(at5.y, lockY, 1e-12, "lock frame 5 base y");
    for (int frame = 6; frame <= 9; ++frame) {
        const RigExecRigPose pose = At(rig, frame);
        const Frame2 f = FrameOf(pose, "F");
        Near(f.x, lockX, 1e-3, "lock F base x " + std::to_string(frame));
        Near(f.y, lockY, 1e-3, "lock F base y " + std::to_string(frame));
        Near(Wrap(f.Angle()), 0, 1e-3, "lock F angle " + std::to_string(frame));
        const double local = Wrap(FrameOf(pose, "S").Angle() - FrameOf(pose, "T").Angle());
        CHECK(local >= -150 - 1e-9 && local <= 1e-9);
    }
    const RigExecRigPose lifted = At(rig, 10);
    const Frame2 f = FrameOf(lifted, "F"), t = FrameOf(lifted, "T");
    CHECK(std::hypot(f.x - lockX, f.y - lockY) > 0.1);       // the foot lifts
    Near(std::hypot(f.x - t.x, f.y - t.y), 0.8, 2e-3, "lock legs straighten");
    Parity(stage, "lock", {4, 5, 6, 9, 10});

    // Two lock keys: held references (splines with held knots) never
    // slide; F holds A through frame 19.999 and B from 20.
    const double bx = lockX + 0.05, by = lockY + 0.02;
    const std::string keyed =
        "                bool bone2d:lock.timeSamples = { 0: 0, 5: 1, }\n"
        "                double bone2d:lockX.spline = { 5: " + Num(lockX) + "; post held, 20: " +
        Num(bx) + "; post held, }\n"
        "                double bone2d:lockY.spline = { 5: " + Num(lockY) + "; post held, 20: " +
        Num(by) + "; post held, }\n"
        "                double bone2d:lockAngle.spline = { 5: 0; post held, 20: 10; post held, }\n";
    auto keyedStage = Stage(Usda(LegRig("", keyed, &lockX, &lockY)));
    RigExecRigEvaluator keyedRig(keyedStage, kRig);
    CHECK(Compiles(keyedRig));
    for (const double time : {6.0, 12.0, 19.0, 19.999}) {
        const Frame2 g = FrameOf(At(keyedRig, time), "F");
        Near(g.x, lockX, 1e-3, "two keys A x " + std::to_string(time));
        Near(g.y, lockY, 1e-3, "two keys A y " + std::to_string(time));
        Near(Wrap(g.Angle()), 0, 1e-3, "two keys A angle");
    }
    for (const double time : {20.0, 25.0}) {
        const Frame2 g = FrameOf(At(keyedRig, time), "F");
        Near(g.x, bx, 1e-3, "two keys B x " + std::to_string(time));
        Near(g.y, by, 1e-3, "two keys B y " + std::to_string(time));
        Near(Wrap(g.Angle()), 10, 1e-3, "two keys B angle");
    }
    Parity(keyedStage, "two lock keys", {4, 12, 19.5, 20, 25});
}

// Limits: the lock solve never turns S past its limits. The foot is
// locked at its setup pose; frame 1 puts the hip at (-0.15, -0.10), frame 2
// at (-0.30, -0.30). Without S's limits the solve bends S forward at both
// (the closed-form bends +53.97 and +86.96), so the limited checks fail if
// the solve stops clamping. Limited, S stays in [-150, 0]; whether it
// reaches the backward bend at frame 2 depends on rounding, so it is not
// asserted.
void LockWithinLimits()
{
    double lockX = 0, lockY = 0;
    LegRig("", "", &lockX, &lockY);
    const std::string hip =
        "                double avars:tx.timeSamples = { 0: 0, 1: -0.15, 2: -0.3, }\n"
        "                double avars:ty.timeSamples = { 0: 0, 1: -0.1, 2: -0.3, }\n";
    const std::string foot =
        "                bool bone2d:lock = 1\n"
        "                double bone2d:lockX = " + Num(lockX) + "\n"
        "                double bone2d:lockY = " + Num(lockY) + "\n"
        "                double bone2d:lockAngle = 0\n";
    const auto knee = [](const RigExecRigPose &pose) {
        return Wrap(FrameOf(pose, "S").Angle() - FrameOf(pose, "T").Angle());
    };
    const auto miss = [&](const RigExecRigPose &pose) {
        const Frame2 f = FrameOf(pose, "F");
        return std::hypot(f.x - lockX, f.y - lockY);
    };
    auto unlimitedStage = Stage(Usda(LegRig(hip, foot, &lockX, &lockY, false)));
    RigExecRigEvaluator unlimited(unlimitedStage, kRig);
    CHECK(Compiles(unlimited));
    const double unlimitedKnee[] = {53.9655, 86.9610};
    for (int frame = 1; frame <= 2; ++frame) {
        const RigExecRigPose pose = At(unlimited, frame);
        const std::string at = std::to_string(frame);
        CHECK(knee(pose) > 0);
        Near(knee(pose), unlimitedKnee[frame - 1], 1e-3, "limits unlimited knee " + at);
        Near(miss(pose), 0, 1e-6, "limits unlimited foot " + at);
    }
    Parity(unlimitedStage, "limits unlimited", {0, 1, 2});

    auto stage = Stage(Usda(LegRig(hip, foot, &lockX, &lockY)));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    for (int frame = 0; frame <= 2; ++frame) {
        const RigExecRigPose pose = At(rig, frame);
        const std::string at = std::to_string(frame);
        const double k = knee(pose);
        if (!(k >= -150 - 1e-9 && k <= 1e-9)) {
            std::printf("FAIL limits limited knee %s: %.9f outside [-150, 0]\n", at.c_str(), k);
            ++failures;
        }
        Near(Wrap(FrameOf(pose, "F").Angle()), 0, 1e-9, "limits limited foot angle " + at);
    }
    Parity(stage, "limits limited", {0, 1, 2});
}

// Cross-scale rig: cross scale per mode, as bound points see it.
void CrossScale()
{
    struct Mode { const char *name, *extra; double at2, atHalf; };
    const Mode modes[] = {
        {"Z1", "", 1, 1},
        {"Z2", "                uniform token bone2d:scalingMode = \"squashStretch\"\n", 0.5, 2},
        {"Z3", "                uniform token bone2d:scalingMode = \"squashStretch\"\n"
               "                uniform double bone2d:squashFactor = 0.5\n",
         std::pow(2.0, -0.5), std::pow(0.5, -0.5)},
        {"Z4", "                uniform token bone2d:scalingMode = \"squashStretch\"\n"
               "                uniform double bone2d:squashFactor = 2\n", 0.25, 4},
        {"Z5", "                uniform token bone2d:scalingMode = \"uniform\"\n", 2, 0.5},
    };
    std::vector<BoneSpec> bones;
    for (const Mode &m : modes) {
        BoneSpec b;
        b.name = m.name; b.x = -0.2; b.y = -0.6; b.angle = 0;
        b.extra = std::string(m.extra) +
                  "                double avars:sx.timeSamples = { 0: 1, 1: 2, 2: 0.5, }\n";
        bones.push_back(b);
    }
    auto stage = Stage(Usda(bones));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    for (const int frame : {1, 2}) {
        const double s = frame == 1 ? 2 : 0.5;
        const RigExecRigPose pose = At(rig, frame);
        for (const Mode &m : modes) {
            const double across = frame == 1 ? m.at2 : m.atHalf;
            const Frame2 f = FrameOf(pose, m.name);
            Near(f.AlongScale(), s, 1e-12, std::string("cross scale along ") + m.name);
            Near(f.CrossScale(), across, 1e-12, std::string("cross scale across ") + m.name);
            for (const double x : {-0.2, 0.0, 0.2}) {
                for (const double y : {-0.7, -0.6, -0.5}) {
                    const GfVec3d p = Mapped(pose, m.name, x, y);
                    Near(p[0] + 0.2, s * (x + 0.2), 1e-12, std::string("cross scale x ") + m.name);
                    Near(p[1] + 0.6, across * (y + 0.6), 1e-12, std::string("cross scale y ") + m.name);
                }
            }
        }
    }
    Parity(stage, "cross scale", {0, 1, 2});
}

// Flip rig: flips mirror bound points; parity reverses turns.
void Flips()
{
    BoneSpec fp;
    fp.name = "FP"; fp.x = 0.4; fp.y = -0.2; fp.angle = 0;
    fp.extra = "                bool bone2d:sideFlip.timeSamples = { 0: 0, 5: 1, }\n";
    BoneSpec fc = AtTip(fp, 0.3, 0, "FC");
    fc.extra = "                bool bone2d:sideFlip.timeSamples = { 0: 0, 2: 1, 3: 0, 4: 1, 5: 0, }\n"
               "                bool bone2d:endFlip.timeSamples = { 0: 0, 3: 1, 4: 0, }\n"
               "                double avars:rz.timeSamples = { 0: 0, 3: 0, 4: 30, 5: 30, }\n";
    auto stage = Stage(Usda({fp, fc}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    // Frame 2: side flip on FC puts the column above its tip below it.
    GfVec3d p = Mapped(At(rig, 2), "FC", 1.0, -0.1);
    Near(p[0], 1.0, 1e-12, "flips side flip x");
    Near(p[1], -0.3, 1e-12, "flips side flip y");
    // Frame 3: end flip extends the row backward from FC's base.
    p = Mapped(At(rig, 3), "FC", 0.8, -0.2);
    Near(p[0], 0.6, 1e-12, "flips end flip x");
    Near(p[1], -0.2, 1e-12, "flips end flip y");
    // Frame 4: FC's own side flip, +30 still turns counter-clockwise.
    Near(Wrap(FrameOf(At(rig, 4), "FC").Angle()), 30, 1e-9, "flips own flip turn");
    // Frame 5: under a side-flipped parent (odd parity) +30 turns clockwise.
    const RigExecRigPose five = At(rig, 5);
    Near(Wrap(FrameOf(five, "FC").Angle()), -30, 1e-9, "flips parity turn");
    Near(FrameOf(five, "FC").x, 0.7, 1e-12, "flips FC base stays on FP's tip");
    Parity(stage, "flips", {0, 2, 3, 4, 5});
}

// Parent scale: the parent's scale puts the child on the stretched tip; the
// child's own frame is unscaled.
void ParentScale()
{
    BoneSpec r;
    r.name = "R"; r.x = -0.8; r.y = -0.8; r.angle = 0;
    r.extra = "                double avars:sx.timeSamples = { 0: 1, 1: 2, }\n";
    BoneSpec h = AtTip(r, 0.3, 0, "H");
    auto stage = Stage(Usda({r, h}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const RigExecRigPose pose = At(rig, 1);
    const Frame2 fr = FrameOf(pose, "R"), fh = FrameOf(pose, "H");
    Near(fh.x - fr.x, 0.6, 1e-12, "parent scale child base");
    Near(fh.y - fr.y, 0, 1e-12, "parent scale child base y");
    Near(fh.AlongScale(), 1, 1e-15, "parent scale child unscaled");
    Parity(stage, "parent scale", {0, 1});
}

// A math mover adding to a bone's avar reaches the solve (the chain's
// result), with no read phase on rigExec:joints.
void ActionOffset()
{
    const std::string mover =
        "        def Scope \"Movers\"\n        {\n"
        "            def RigExecFloatMathMover \"Offset\" (prepend apiSchemas = [\"RigExecMoverAPI\"])\n"
        "            {\n"
        "                rel rigExec:moves = </Char/Rig/Bones/Q.avars:rz>\n"
        "                uniform token rigExec:operation = \"add\"\n"
        "                float inputs:value = 30\n"
        "            }\n        }\n";
    auto stage = Stage(Usda(IndependentRig(false, kIndependentQ, kIndependentF), "", mover));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    // Frame 2: Q keyed 60, +30 from the mover; F follows at 30 local.
    const RigExecRigPose pose = At(rig, 2);
    Near(Wrap(FrameOf(pose, "Q").Angle()), 90, 1e-5, "action offset Q");
    Near(Wrap(FrameOf(pose, "F").Angle()), 120, 1e-5, "action offset F");
    Parity(stage, "action offset", {1, 2});
}

// Value edits re-evaluate with the signature unchanged; structural edits
// recompile with a new signature.
void Edits()
{
    BoneSpec p;
    p.name = "P"; p.x = -0.8; p.y = 0.6; p.angle = 20;
    p.extra = "                double avars:rz = 0\n";
    BoneSpec c = AtTip(p, 0.3, 30, "C");
    c.extra = "                uniform bool bone2d:angleLimits = 1\n"
              "                uniform double bone2d:minAngle = -10\n"
              "                uniform double bone2d:maxAngle = 40\n"
              "                double avars:rz = 60\n";
    auto stage = Stage(Usda({p, c}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    // Keyed angle unclamped: a keyed 90 (local) is not clamped by evaluation.
    RigExecRigPose pose = At(rig, 1);
    Near(Wrap(FrameOf(pose, "C").Angle() - FrameOf(pose, "P").Angle()), 90, 1e-9, "keyed angle unclamped");
    const size_t signature = rig.GetCompiledRigSignature();
    const UsdPrim bp = stage->GetPrimAtPath(Bone("P")), bc = stage->GetPrimAtPath(Bone("C"));
    // An unanimated avar edited at Default (the in-place avar patch).
    CHECK(bp.GetAttribute(TfToken("avars:rz")).Set(40.0));
    pose = At(rig, 1);
    CHECK(rig.GetCompiledRigSignature() == signature);
    Near(Wrap(FrameOf(pose, "P").Angle()), 60, 1e-9, "avar edit P");
    Near(Wrap(FrameOf(pose, "C").Angle()), 150, 1e-9, "avar edit C");
    Frozen(rig, 1, "avar edit");
    // A rest edit: P's setup base moves, the pose follows, no recompile.
    CHECK(bp.GetAttribute(TfToken("rest:space")).Set(
        GfMatrix4d(1).SetRotate(GfRotation(GfVec3d(0, 0, 1), 20)) *
        GfMatrix4d(1).SetTranslate(GfVec3d(-0.7, 0.5, 0))));
    pose = At(rig, 1);
    CHECK(rig.GetCompiledRigSignature() == signature);
    Near(FrameOf(pose, "P").x, -0.7, 1e-12, "rest edit base x");
    Near(FrameOf(pose, "P").y, 0.5, 1e-12, "rest edit base y");
    Frozen(rig, 1, "rest edit");
    // Lock channels are values too.
    CHECK(bc.CreateAttribute(TfToken("bone2d:lock"), SdfValueTypeNames->Bool).Set(true));
    pose = At(rig, 1);
    CHECK(rig.GetCompiledRigSignature() == signature);
    // A limit edit recompiles.
    CHECK(bc.GetAttribute(TfToken("bone2d:maxAngle")).Set(45.0));
    pose = At(rig, 1);
    const size_t limited = rig.GetCompiledRigSignature();
    CHECK(limited != signature);
    // A reparent is a namespace move and recompiles: C moves out from under
    // P (its rest re-expressed so it keeps its setup frame) and becomes a
    // root.
    CHECK(bc.GetAttribute(TfToken("bone2d:lock")).Set(false));
    const SdfPath moved("/Char/Rig/Bones/C");
    {
        SdfBatchNamespaceEdit edit;
        edit.Add(SdfNamespaceEdit::Reparent(Bone("C"), SdfPath("/Char/Rig/Bones"),
                                            SdfNamespaceEdit::AtEnd));
        CHECK(stage->GetRootLayer()->Apply(edit));
    }
    BonePaths()["C"] = moved.GetString();
    const UsdPrim root = stage->GetPrimAtPath(moved);
    CHECK(root);
    CHECK(root.GetAttribute(TfToken("rest:space")).Set(
        GfMatrix4d(1).SetRotate(GfRotation(GfVec3d(0, 0, 1), 50)) *
        GfMatrix4d(1).SetTranslate(GfVec3d(c.x, c.y, 0))));
    CHECK(stage->GetPrimAtPath(SdfPath("/Char/Rig/Skeleton"))
              .GetRelationship(TfToken("rigExec:joints"))
              .SetTargets({Bone("P"), moved}));
    pose = At(rig, 1);
    CHECK(rig.GetCompiledRigSignature() != limited);
    Near(Wrap(FrameOf(pose, "C").Angle()), 50 + 60, 1e-9, "reparented C");
    Near(FrameOf(pose, "C").x, c.x, 1e-12, "reparented C keeps its base");
    Frozen(rig, 1, "reparent");
    Runtime(stage, "edited", {1}, Bone("C").AppendProperty(TfToken("avars:rz")), 10);
}

// Custom storage on a provider is not a pose input until a consumer names it.
void CustomProviderInputs()
{
    BoneSpec b; b.name = "B";
    auto stage = Stage(Usda({b}));
    UsdPrim bone = stage->GetPrimAtPath(Bone("B"));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    At(rig, 1);
    const size_t signature = rig.GetCompiledRigSignature();
    const size_t builds = rig.GetBakedProgramBuildCount();
    const auto unchanged = [&]() {
        Near(FrameOf(At(rig, 1), "B").Angle(), 0, 1e-9, "unused custom key");
        CHECK(rig.GetCompiledRigSignature() == signature);
        CHECK(rig.GetBakedProgramBuildCount() == builds);
    };
    // Presence undo/redo, including the same storage under another namespace.
    for (const char *name : {"actions:test:avars:rz", "editor:storedAngle"}) {
        const TfToken token(name);
        for (int redo = 0; redo != 2; ++redo) {
            auto key = bone.CreateAttribute(token, SdfValueTypeNames->Double, true);
            CHECK(key.Set(15.0));
            CHECK(key.Set(40.0, UsdTimeCode(1)));
            unchanged();
            Frozen(rig, 1, "unused custom key");
            CHECK(bone.RemoveProperty(token));
            unchanged();
        }
    }
    Runtime(stage, "custom key undo", {1});

    const TfToken keyName("actions:test:avars:rz");
    const SdfPath source = bone.GetPath().AppendProperty(keyName);
    const UsdAttribute angle = bone.GetAttribute(TfToken("avars:rz"));
    const auto compare = [&](double expected, const char *label, bool freeze = true) {
        Near(FrameOf(At(rig, 1), "B").Angle(), expected, 1e-9, label);
        RigExecRigEvaluator fresh(stage, kRig);
        CHECK(Compiles(fresh));
        Near(FrameOf(At(fresh, 1), "B").Angle(), expected, 1e-9, label);
        if (freeze) Frozen(rig, 1, label);
    };
    // The named source starts missing. Its existence and type still matter.
    CHECK(angle.SetConnections({source}));
    compare(0, "missing custom source");
    size_t connected = rig.GetCompiledRigSignature();
    CHECK(connected != signature);
    for (int redo = 0; redo != 2; ++redo) {
        auto key = bone.CreateAttribute(keyName, SdfValueTypeNames->Double, true);
        CHECK(key.Set(15.0));
        compare(15, "create referenced custom key");
        CHECK(rig.GetCompiledRigSignature() != connected);
        connected = rig.GetCompiledRigSignature();
        CHECK(key.Set(40.0));
        compare(40, "referenced custom value");
        CHECK(rig.GetCompiledRigSignature() == connected);
        Runtime(stage, "referenced custom value", {1});
        CHECK(bone.RemoveProperty(keyName));
        compare(0, "remove referenced custom key");
        CHECK(rig.GetCompiledRigSignature() != connected);
        connected = rig.GetCompiledRigSignature();
    }
    // An incompatible retype is structural too; restoring the type recovers.
    CHECK(bone.CreateAttribute(keyName, SdfValueTypeNames->Float, true).Set(9.0f));
    compare(0, "incompatible custom source type", false);
    // Native uses the channel fallback. Static bakeability accepts this
    // graph, but the worker declines its incompatible sampled binding.
    // Restoring the type below must restore frozen and binary parity.
    std::vector<std::string> bakeReasons;
    CHECK(rig.IsBakeable(&bakeReasons));
    CHECK(bakeReasons.empty());
    std::string frozenReason;
    CHECK(!RigExecFrozenMatchesLive(&rig, kRig, UsdTimeCode(1), &frozenReason));
    CHECK(frozenReason == "the warming job was declined");
    CHECK(rig.GetCompiledRigSignature() != connected);
    connected = rig.GetCompiledRigSignature();
    CHECK(bone.RemoveProperty(keyName));
    CHECK(bone.CreateAttribute(keyName, SdfValueTypeNames->Double, true).Set(25.0));
    compare(25, "restore custom source type");
    CHECK(rig.GetCompiledRigSignature() != connected);

    // A forwarding source on another provider keeps the terminal property in
    // the exact source footprint, even though it is not an own seed there.
    const UsdPrim other = stage->DefinePrim(kRig.AppendChild(TfToken("Other")),
                                           TfToken("RigExecControl"));
    const TfToken aliasName("actions:test:alias");
    const auto alias = other.CreateAttribute(aliasName, SdfValueTypeNames->Double, true);
    CHECK(alias.SetConnections({source}));
    CHECK(angle.SetConnections({alias.GetPath()}));
    compare(25, "foreign custom alias");
    connected = rig.GetCompiledRigSignature();
    CHECK(bone.RemoveProperty(keyName));
    compare(0, "foreign alias missing source");
    CHECK(rig.GetCompiledRigSignature() != connected);
    connected = rig.GetCompiledRigSignature();
    CHECK(bone.CreateAttribute(keyName, SdfValueTypeNames->Double, true).Set(35.0));
    compare(35, "foreign alias restored source");
    CHECK(rig.GetCompiledRigSignature() != connected);
    Runtime(stage, "foreign alias restored source", {1});
}

// A pose step below the skeleton that writes a bone hands the skeleton that
// frame as the bone's rest, in every evaluator.
void LiveRest()
{
    BoneSpec a;
    a.name = "A"; a.x = 0; a.y = 0; a.angle = 0;
    BoneSpec b = AtTip(a, 0.5, 30, "B");
    const std::string control =
        "        def RigExecControl \"Target\"\n        {\n"
        "            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 1, 0, 1) )\n"
        "        }\n";
    const std::string aim =
        "        def RigExecAimConstraint \"Aim\" (prepend apiSchemas = [\"RigExecMoverAPI\"])\n"
        "        {\n"
        "            float inputs:defaultWeight = 1\n"
        "            uniform token rigExec:aimAxis = \"x\"\n"
        "            rel rigExec:aimTarget = </Char/Rig/Target>\n"
        "            rel rigExec:moves = </Char/Rig/Bones/A/B>\n"
        "        }\n";
    // Aimed at (1, 1) from B's rest base (0.5, 0).
    const double aimed = std::atan2(1.0 - b.y, 1.0 - b.x) / kDeg;
    {
        auto stage = Stage(Usda({a, b}, control, aim));
        RigExecRigEvaluator rig(stage, kRig);
        CHECK(Compiles(rig));
        Near(Wrap(FrameOf(At(rig, 1), "B").Angle() - aimed), 0, 1e-6, "live rest at zero channels");
        Parity(stage, "live rest zero", {1});
    }
    {
        BoneSpec turned = b;
        turned.extra = "                double avars:rz = 10\n";
        auto stage = Stage(Usda({a, turned}, control, aim));
        RigExecRigEvaluator rig(stage, kRig);
        CHECK(Compiles(rig));
        Near(Wrap(FrameOf(At(rig, 1), "B").Angle() - (aimed + 10)), 0, 1e-6, "live rest with avar");
        // Above the skeleton the aim revises the solved frame instead.
        auto above = Stage(Usda({a, turned}, control + aim));
        RigExecRigEvaluator aboveRig(above, kRig);
        CHECK(Compiles(aboveRig));
        const Frame2 fb = FrameOf(At(aboveRig, 1), "B");
        Near(fb.Angle(), std::atan2(1 - fb.y, 1 - fb.x) / kDeg, 1e-6, "aim above, avar present");
    }
    {
        // The step below reads B where the namespace composes it, A's
        // avars:rz included (B nests under A): that frame is B's rest, and
        // the solve then turns B about A by A's 20 degrees.
        BoneSpec parentTurned = a, turned = b;
        parentTurned.extra = "                double avars:rz = 20\n";
        turned.extra = "                double avars:rz = 10\n";
        auto stage = Stage(Usda({parentTurned, turned}, control, aim));
        RigExecRigEvaluator rig(stage, kRig);
        CHECK(Compiles(rig));
        const Frame2 fb = FrameOf(At(rig, 1), "B");
        const double restX = 0.5 * std::cos(20 * kDeg), restY = 0.5 * std::sin(20 * kDeg);
        const double aimedTurned = std::atan2(1.0 - restY, 1.0 - restX) / kDeg;
        Near(fb.x, 0.5 * std::cos(40 * kDeg), 1e-9, "live rest base x about parent");
        Near(fb.y, 0.5 * std::sin(40 * kDeg), 1e-9, "live rest base y about parent");
        Near(Wrap(fb.Angle() - (aimedTurned + 30)), 0, 1e-6, "live rest angle");
        Parity(stage, "live rest", {1}, Bone("B").AppendProperty(TfToken("avars:rz")), 10);
    }
    {
        // A non-planar incoming frame: the skeleton still compiles, publishes
        // the bones' rests and says why.
        std::string usda = Usda({a, b}, control, aim);
        const std::string at = "(1, 1, 0, 1) )";
        const size_t where = usda.find(at);
        CHECK(where != std::string::npos);
        if (where != std::string::npos) usda.replace(where, at.size(), "(1, 1, 1, 1) )");
        auto stage = Stage(usda);
        RigExecRigEvaluator rig(stage, kRig);
        CHECK(Compiles(rig));
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1));
        CHECK(pose.valid);
        bool said = false;
        for (const auto &line : pose.diagnostics)
            said = said || line.find("is not a rotation about +Z") != std::string::npos;
        CHECK(said);
        for (const char *name : {"A", "B"}) {
            const Frame2 f = FrameOf(pose, name);
            CHECK(std::isfinite(f.x) && std::isfinite(f.y));
        }
        Near(FrameOf(pose, "A").Angle(), 0, 1e-9, "non-planar: A keeps its rest");
        Frozen(rig, 1, "live rest non-planar");
        Runtime(stage, "live rest non-planar", {1});
    }
}

// A bone's tip is its first joint child: a child bone's rest is a value (an
// edit re-solves without a recompile), an End joint's rest is compiled (an
// edit recompiles), in every evaluator.
void TipIsSetup()
{
    BoneSpec thigh;
    thigh.name = "T"; thigh.x = 0; thigh.y = 2; thigh.angle = -90;
    BoneSpec shin = AtTip(thigh, 1.0, 0, "S");
    shin.end = 1;
    shin.extra = "                rel bone2d:targetCandidates = </Char/Rig/Bones/F>\n"
                 "                double bone2d:targetIndex = 0\n";
    BoneSpec foot;
    foot.name = "F"; foot.x = 0.3; foot.y = 0.1; foot.angle = 0;
    foot.extra = "                uniform bool bone2d:ignoredByIk = 1\n";
    auto stage = Stage(Usda({thigh, shin, foot}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const double before = FrameOf(At(rig, 1), "S").Angle();
    // The End joint's rest is compiled: an edit recompiles and re-solves.
    size_t signature = rig.GetCompiledRigSignature();
    const SdfPath end = Bone("S").AppendChild(TfToken("End"));
    CHECK(stage->GetPrimAtPath(end).GetAttribute(TfToken("rest:space"))
              .Set(GfMatrix4d(1).SetTranslate(GfVec3d(0.4, 0, 0))));
    const double after = FrameOf(At(rig, 1), "S").Angle();
    CHECK(rig.GetCompiledRigSignature() != signature);
    CHECK(std::fabs(Wrap(after - before)) > 1e-3);
    Frozen(rig, 1, "End joint edit");
    Runtime(stage, "End joint edit", {1});
    // The shin is the thigh's tip: moving its rest is a value edit, and the
    // shortened thigh carries the shin's base with it.
    signature = rig.GetCompiledRigSignature();
    const auto thighReach = [&]() {
        const RigExecRigPose pose = At(rig, 1);
        const Frame2 t = FrameOf(pose, "T"), sh = FrameOf(pose, "S");
        return std::hypot(sh.x - t.x, sh.y - t.y);
    };
    Near(thighReach(), 1.0, 1e-9, "tip bone before");
    CHECK(stage->GetPrimAtPath(Bone("S")).GetAttribute(TfToken("rest:space"))
              .Set(GfMatrix4d(1).SetTranslate(GfVec3d(0.7, 0, 0))));
    Near(thighReach(), 0.7, 1e-9, "tip bone after");
    CHECK(rig.GetCompiledRigSignature() == signature);
    Frozen(rig, 1, "tip bone rest edit");
    Runtime(stage, "tip bone rest edit", {1});
}

// A RigExecSpaceSwitch prim on bone \p target (a path) with \p sources
// (paths, in order) and \p active lines, under Rig/Spaces as the authoring
// tools put it.
std::string Switch(const std::string &name, const std::string &target,
                   const std::vector<std::string> &sources, const std::string &active,
                   const std::string &extra = "")
{
    std::string text = "        def Scope \"Spaces\"\n        {\n"
                       "            def RigExecSpaceSwitch \"" + name + "\"\n            {\n"
                       "                rel rigExec:target = <" + target + ">\n"
                       "                rel rigExec:sources = [";
    for (size_t i = 0; i < sources.size(); ++i)
        text += (i ? ", <" : "<") + sources[i] + ">";
    return text + "]\n" + active + extra + "            }\n        }\n";
}

void Refusals()
{
    BoneSpec a;
    a.name = "A"; a.x = 0; a.y = 0; a.angle = 0;
    BoneSpec b = AtTip(a, 0.5, 30, "B");
    // The same constraint above the skeleton revises the solved frame.
    const std::string control =
        "        def RigExecControl \"Target\"\n        {\n"
        "            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 1, 0, 1) )\n"
        "        }\n";
    const std::string aim =
        "        def RigExecAimConstraint \"Aim\" (prepend apiSchemas = [\"RigExecMoverAPI\"])\n"
        "        {\n"
        "            float inputs:defaultWeight = 1\n"
        "            uniform token rigExec:aimAxis = \"x\"\n"
        "            rel rigExec:aimTarget = </Char/Rig/Target>\n"
        "            rel rigExec:moves = </Char/Rig/Bones/A/B>\n"
        "        }\n";
    {
        auto stage = Stage(Usda({a, b}, control + aim));
        RigExecRigEvaluator rig(stage, kRig);
        CHECK(Compiles(rig));
        const RigExecRigPose pose = At(rig, 1);
        // Aimed at (1, 1) from B's base.
        const Frame2 fb = FrameOf(pose, "B");
        Near(fb.Angle(), std::atan2(1 - fb.y, 1 - fb.x) / kDeg, 1e-6, "aim above the skeleton");
    }
    {
        // A rest tilted about X: replace A's authored rest:space.
        std::string usda = Usda({a, b});
        const std::string flat = "( (1, 0, 0, 0), (-0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )";
        const size_t at = usda.find(flat);
        CHECK(at != std::string::npos);
        if (at != std::string::npos)
            usda.replace(at, flat.size(), "( (1, 0, 0, 0), (0, 0.6, 0.8, 0), (0, -0.8, 0.6, 0), (0, 0, 0, 1) )");
        Refused(usda, "not a rotation about +Z");
    }
    BoneSpec restTx = b;
    restTx.extra = "                double rest:tx = 0.1\n";
    Refused(Usda({a, restTx}), "authors rest:tx");
    BoneSpec animatedRest = b;
    animatedRest.extra = "                matrix4d rest:space.timeSamples = { 1: ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) ), }\n";
    Refused(Usda({a, animatedRest}), "animated or connected rest:space");
    BoneSpec sampled = b;
    sampled.extra = "                uniform double bone2d:maxAngle.timeSamples = { 1: 10, 2: 20, }\n";
    Refused(Usda({a, sampled}), "must not be animated");
    // A cycle through the namespace and a switch: A may hang under its own
    // child B.
    Refused(Usda({a, b}, "", Switch("A", "/Char/Rig/Bones/A", {"/Char/Rig", "/Char/Rig/Bones/A/B"},
                                    "                double inputs:activeSpace = 0\n")),
            "form a cycle");
    // A switch source under a bone of the same skeleton (B's End joint)
    // moves with the skeleton's own result.
    {
        BoneSpec ended = b;
        ended.end = 0.5;
        Refused(Usda({a, ended}, "", Switch("A", "/Char/Rig/Bones/A",
                                            {"/Char/Rig/Bones/A/B/End"},
                                            "                double inputs:activeSpace = 0\n")),
                "sits under the bone");
    }
    // A bone cannot be its own parent space.
    Refused(Usda({a, b}, "", Switch("B", "/Char/Rig/Bones/A/B", {"/Char/Rig/Bones/A/B"},
                                    "                double inputs:activeSpace = 0\n")),
            "its own parent space");
    // The skeleton takes a switch's spaces whole, by a numeric index.
    Refused(Usda({a, b}, "", Switch("B", "/Char/Rig/Bones/A/B", {"/Char/Rig/Bones/A", "/Char/Rig"},
                                    "                double inputs:activeSpace = 0\n",
                                    "                rel rigExec:space = </Char/Rig/Bones/A>\n")),
            "authors rigExec:space");
    Refused(Usda({a, b}, "", Switch("B", "/Char/Rig/Bones/A/B", {"/Char/Rig/Bones/A", "/Char/Rig"},
                                    "                double inputs:activeSpace = 0\n",
                                    "                uniform token[] rigExec:rotationFilters = [\"all\", \"twist\"]\n")),
            "rigExec:rotationFilters");
    Refused(Usda({a, b}, "", Switch("B", "/Char/Rig/Bones/A/B", {"/Char/Rig/Bones/A", "/Char/Rig"},
                                    "                double inputs:activeSpace = 0\n",
                                    "                bool inputs:affectRotationZ = 0\n")),
            "clears an axis mask");
    {
        BoneSpec labelled = b;
        labelled.extra = "                token avars:space = \"Root\"\n";
        Refused(Usda({a, labelled}, "",
                     Switch("B", "/Char/Rig/Bones/A/B", {"/Char/Rig/Bones/A", "/Char/Rig"},
                            "                rel rigExec:activeSpaceAttribute = </Char/Rig/Bones/A/B.avars:space>\n",
                            "                uniform token[] rigExec:spaceLabels = [\"Arm\", \"Root\"]\n")),
                "selects its space by label");
    }
    // A pin (no joint child) with a target and no parent bone has nothing
    // to turn.
    {
        BoneSpec lone;
        lone.name = "L"; lone.x = 2; lone.y = 0; lone.angle = 0;
        lone.extra = "                rel bone2d:targetCandidates = </Char/Rig/Bones/A>\n"
                     "                double bone2d:targetIndex = 0\n";
        Refused(Usda({a, b, lone}), "is a pin (no joint child)");
    }
    // The start frame is not one of the bones, nor under one.
    Refused(Usda({a, b}, "", "", {},
                 "            rel rigExec:startFrame = </Char/Rig/Bones/A/B>\n"),
            "a skeleton cannot hang from its own bones");
    // A joint without RigExecBone2dAPI.
    {
        std::string usda = Usda({a, b});
        const std::string api = "\"B\" (prepend apiSchemas = [\"RigExecBone2dAPI\"])";
        const size_t at = usda.find(api);
        CHECK(at != std::string::npos);
        if (at != std::string::npos) usda.replace(at, api.size(), "\"B\"");
        Refused(usda, "is not a RigExecJoint with RigExecBone2dAPI");
    }
    // rigExec:jointElements is not a 2D skeleton's.
    {
        std::string usda = Usda({a, b});
        const std::string rel = "        def RigExecSkeleton2d \"Skeleton\"\n        {\n";
        const size_t at = usda.find(rel);
        CHECK(at != std::string::npos);
        if (at != std::string::npos)
            usda.insert(at + rel.size(), "            int[] rigExec:jointElements = [1, 0]\n");
        Refused(usda, "rigExec:jointElements is not read");
    }
}

// Hierarchy is namespace nesting: the nearest RigExecJoint ancestor is the
// parent when it is a bone; a bone under a joint that is not a bone is a root
// of the skeleton and keeps its setup frame whatever that joint does.
void Nesting()
{
    BoneSpec a;
    a.name = "A"; a.x = 0; a.y = 0; a.angle = 0;
    a.extra = "                double avars:rz = 90\n";
    BoneSpec b = AtTip(a, 1.0, 0, "B");
    std::string usda = Usda({a, b});
    // Wrap B in a plain joint (not a bone): B is a root then.
    const std::string open = "                def RigExecJoint \"B\"";
    const size_t at = usda.find(open);
    CHECK(at != std::string::npos);
    if (at == std::string::npos) return;
    const size_t close = usda.find("                }\n", at);
    CHECK(close != std::string::npos);
    std::string inner = usda.substr(at, close + 18 - at);
    std::string wrapped = "                def RigExecJoint \"Mid\"\n                {\n" + inner +
                          "                }\n";
    usda.replace(at, close + 18 - at, wrapped);
    const std::string listed = "</Char/Rig/Bones/A/B>";
    const size_t list = usda.find(listed);
    CHECK(list != std::string::npos);
    if (list != std::string::npos) usda.replace(list, listed.size(), "</Char/Rig/Bones/A/Mid/B>");
    BonePaths()["B"] = "/Char/Rig/Bones/A/Mid/B";
    auto stage = Stage(usda);
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const RigExecRigPose pose = At(rig, 1);
    Near(Wrap(FrameOf(pose, "A").Angle()), 90, 1e-9, "nesting A turns");
    // B's composed rest is (1, 0) (Mid adds nothing); as a root it stays.
    Near(FrameOf(pose, "B").x, 1.0, 1e-12, "nesting under a plain joint x");
    Near(FrameOf(pose, "B").y, 0.0, 1e-12, "nesting under a plain joint y");
    Parity(stage, "nesting", {1});
}

// the offset translates the posed bone in its parent's frame
// at every frame except the setup frame (held 0 there), never the rest.
void Offset()
{
    BoneSpec r;
    r.name = "R"; r.x = 0; r.y = 0; r.angle = 0;
    BoneSpec c = AtTip(r, 1.0, 0, "C");
    c.extra = "                double bone2d:offsetX.spline = { 0: 0; post held, 1: 0.5; post held, }\n"
              "                double bone2d:offsetY.spline = { 0: 0; post held, 1: 0.25; post held, }\n";
    auto stage = Stage(Usda({r, c}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    std::string before, after;
    stage->GetRootLayer()->ExportToString(&before);
    // The setup frame shows the bone without the offset: identity map.
    const RigExecRigPose setup = At(rig, 0);
    const Frame2 c0 = FrameOf(setup, "C");
    Near(c0.x, 1.0, 1e-12, "offset setup x");
    Near(c0.y, 0.0, 1e-12, "offset setup y");
    const GfVec3d p0 = Mapped(setup, "C", 1.0, 0.0);
    Near(p0[0], 1.0, 1e-12, "offset setup map x");
    Near(p0[1], 0.0, 1e-12, "offset setup map y");
    for (const double time : {1.0, 2.0, 7.5}) {
        const Frame2 posed = FrameOf(At(rig, time), "C");
        Near(posed.x, 1.5, 1e-12, "offset x");
        Near(posed.y, 0.25, 1e-12, "offset y");
        const GfVec3d p = Mapped(At(rig, time), "C", 1.0, 0.0);
        Near(p[0], 1.5, 1e-12, "offset map x");
        Near(p[1], 0.25, 1e-12, "offset map y");
    }
    stage->GetRootLayer()->ExportToString(&after);
    CHECK(before == after);  // evaluation authors nothing
    // Offsets are values: a Default edit re-evaluates without a recompile.
    BoneSpec d = AtTip(r, 1.0, 0, "D");
    d.extra = "                double bone2d:offsetX = 0.5\n";
    auto edited = Stage(Usda({r, d}));
    RigExecRigEvaluator rig2(edited, kRig);
    CHECK(Compiles(rig2));
    At(rig2, 1);
    const size_t signature = rig2.GetCompiledRigSignature();
    CHECK(edited->GetPrimAtPath(Bone("D")).GetAttribute(TfToken("bone2d:offsetX")).Set(1.0));
    Near(FrameOf(At(rig2, 1), "D").x, 2.0, 1e-12, "offset edit");
    CHECK(rig2.GetCompiledRigSignature() == signature);
    Frozen(rig2, 1, "offset edit");
    Parity(stage, "offset", {0, 1, 2});
}

// the shin owner's tip tracks its target bone's base;
// out of reach the leg straightens toward it, stretching up to maxStretch.
void IkTarget()
{
    BoneSpec thigh;
    thigh.name = "T"; thigh.x = 0; thigh.y = 2; thigh.angle = -90;
    BoneSpec shin = AtTip(thigh, 1.0, 0, "S");
    shin.end = 1;  // the shin's tip: an End joint
    shin.extra = "                rel bone2d:targetCandidates = </Char/Rig/Bones/F>\n"
                 "                double bone2d:targetIndex = 0\n";
    BoneSpec foot;
    foot.name = "F"; foot.x = 0.3; foot.y = 0.1; foot.angle = 0;
    foot.extra = "                uniform bool bone2d:ignoredByIk = 1\n";
    auto stage = Stage(Usda({thigh, shin, foot}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const auto tip = [](const RigExecRigPose &pose) {
        const Frame2 s = FrameOf(pose, "S");
        return std::make_pair(s.x + s.xAxisX, s.y + s.xAxisY);
    };
    const auto reached = tip(At(rig, 1));
    Near(reached.first, 0.3, 1e-6, "target tip x");
    Near(reached.second, 0.1, 1e-6, "target tip y");
    // Out of reach with maxStretch 1: straight, short of the goal.
    BoneSpec distantGoal;
    distantGoal.name = "G"; distantGoal.x = 2.5; distantGoal.y = 2.0; distantGoal.angle = 0;
    distantGoal.extra = foot.extra;
    auto farStage = Stage(Usda({thigh, shin, distantGoal}));
    const SdfPath farShin = Bone("S");
    // Repoint the shin at G (it named F, which this stage lacks).
    CHECK(farStage->GetPrimAtPath(farShin)
              .GetRelationship(TfToken("bone2d:targetCandidates"))
              .SetTargets({Bone("G")}));
    RigExecRigEvaluator farRig(farStage, kRig);
    CHECK(Compiles(farRig));
    const auto shortTip = tip(At(farRig, 1));
    Near(std::hypot(shortTip.first - 0.0, shortTip.second - 2.0), 2.0, 1e-3, "target reach");
    Near(shortTip.second, 2.0, 1e-3, "target line");
    // maxStretch 1.5 is structural: it recompiles, then the leg reaches.
    const size_t signature = farRig.GetCompiledRigSignature();
    CHECK(farStage->GetPrimAtPath(farShin)
              .CreateAttribute(TfToken("bone2d:maxStretch"), SdfValueTypeNames->Double)
              .Set(1.5));
    const auto longTip = tip(At(farRig, 1));
    CHECK(farRig.GetCompiledRigSignature() != signature);
    Near(longTip.first, 2.5, 1e-6, "target stretch x");
    Near(longTip.second, 2.0, 1e-6, "target stretch y");
    // An animated target index steps between two pins.
    BoneSpec f2;
    f2.name = "F2"; f2.x = -0.4; f2.y = 0.2; f2.angle = 0;
    f2.extra = foot.extra;
    BoneSpec switched = shin;
    switched.extra = "                rel bone2d:targetCandidates = [</Char/Rig/Bones/F>, </Char/Rig/Bones/F2>]\n"
                     "                double bone2d:targetIndex.spline = { 1: 0; post held, 2: 1; post held, }\n";
    auto switchedStage = Stage(Usda({thigh, switched, foot, f2}));
    RigExecRigEvaluator switchedRig(switchedStage, kRig);
    CHECK(Compiles(switchedRig));
    const auto first = tip(At(switchedRig, 1));
    Near(first.first, 0.3, 1e-6, "target switch A x");
    Near(first.second, 0.1, 1e-6, "target switch A y");
    const auto second = tip(At(switchedRig, 2));
    Near(second.first, -0.4, 1e-6, "target switch B x");
    Near(second.second, 0.2, 1e-6, "target switch B y");
    Parity(stage, "target", {1, 2});
    Parity(switchedStage, "target switch", {1, 2});
}

// control bones add the controller's change from rest, scaled.
void ControlBones()
{
    BoneSpec d;
    d.name = "D"; d.x = 0; d.y = 0; d.angle = 0;
    d.extra = "                double avars:rz = 40\n"
              "                double avars:tx = 3\n"
              "                double avars:ty = 4\n"
              "                double avars:sx = 2\n";
    BoneSpec m;
    m.name = "M"; m.x = 2; m.y = 0; m.angle = 0;
    m.extra = "                rel bone2d:angleControl = </Char/Rig/Bones/D>\n"
              "                uniform double bone2d:angleControlFactor = 0.5\n"
              "                rel bone2d:posControl = </Char/Rig/Bones/D>\n"
              "                uniform double bone2d:posControlFactorX = 2\n"
              "                uniform double bone2d:posControlFactorY = -1\n"
              "                rel bone2d:scaleControl = </Char/Rig/Bones/D>\n"
              "                uniform double bone2d:scaleControlFactor = 1\n"
              "                double avars:rz = 10\n"
              "                double avars:tx = 1\n"
              "                double avars:sx = 3\n";
    auto stage = Stage(Usda({d, m}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const Frame2 f = FrameOf(At(rig, 1), "M");
    Near(Wrap(f.Angle()), 10 + 20, 1e-9, "control angle");
    Near(f.x, 2.0 + 1.0 + 2.0 * 3.0, 1e-9, "control x");
    Near(f.y, 0.0 + (-1.0) * 4.0, 1e-9, "control y");
    Near(f.AlongScale(), 3.0 * 2.0, 1e-9, "control scale");
    // A factor edit recompiles; the counter-moving pair answers.
    const size_t signature = rig.GetCompiledRigSignature();
    CHECK(stage->GetPrimAtPath(Bone("M"))
              .GetAttribute(TfToken("bone2d:angleControlFactor"))
              .Set(-0.5));
    Near(Wrap(FrameOf(At(rig, 1), "M").Angle()), 10 - 20, 1e-9, "control counter");
    CHECK(rig.GetCompiledRigSignature() != signature);
    Parity(stage, "control", {1});
}

// exactly one member active; inactive chains publish
// their setup frames; a disabled group keeps every member active.
void JointGroups()
{
    BoneSpec torso;
    torso.name = "T"; torso.x = 0; torso.y = 0; torso.angle = 0;
    BoneSpec headA;
    headA.name = "HA"; headA.parent = "T"; headA.x = 0; headA.y = 1; headA.angle = 0;
    headA.extra = "                double avars:rz = 30\n";
    BoneSpec jawA;
    jawA.name = "JA"; jawA.parent = "HA"; jawA.x = 0; jawA.y = 1.5; jawA.angle = 0;
    BoneSpec headB;
    headB.name = "HB"; headB.parent = "T"; headB.x = 0; headB.y = 1; headB.angle = 0;
    headB.extra = "                double avars:rz = 30\n";
    GroupSpec g;
    g.name = "G"; g.members = {"HA", "HB"};
    g.extra = "                double rigExec:activeMember.spline = { 1: 0; post held, 2: 1; post held, }\n";
    auto stage = Stage(Usda({torso, headA, jawA, headB}, "", "", {g}));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    const RigExecRigPose first = At(rig, 1);
    Near(Wrap(FrameOf(first, "HA").Angle()), 30, 1e-9, "group A active");
    Near(Wrap(FrameOf(first, "JA").Angle()), 30, 1e-9, "group A carries jaw");
    Near(Wrap(FrameOf(first, "HB").Angle()), 0, 1e-9, "group B rests");
    Near(FrameOf(first, "HB").x, 0, 1e-12, "group B base x");
    Near(FrameOf(first, "HB").y, 1, 1e-12, "group B base y");
    const RigExecRigPose second = At(rig, 2);
    Near(Wrap(FrameOf(second, "HA").Angle()), 0, 1e-9, "group A rests");
    Near(Wrap(FrameOf(second, "HB").Angle()), 30, 1e-9, "group B active");
    // Disabling the group is structural: every member wakes.
    const size_t signature = rig.GetCompiledRigSignature();
    CHECK(stage->GetPrimAtPath(SdfPath("/Char/Rig/Skeleton/G"))
              .GetAttribute(TfToken("rigExec:enabled"))
              .Set(false));
    Near(Wrap(FrameOf(At(rig, 1), "HB").Angle()), 30, 1e-9, "group disabled");
    CHECK(rig.GetCompiledRigSignature() != signature);
    Parity(stage, "groups", {1, 2});
    // Membership is the members collection: excluding a member drops it
    // (structural), so the remaining member is active at every frame.
    CHECK(stage->GetPrimAtPath(SdfPath("/Char/Rig/Skeleton/G"))
              .GetAttribute(TfToken("rigExec:enabled"))
              .Set(true));
    Near(Wrap(FrameOf(At(rig, 1), "HB").Angle()), 0, 1e-9, "group re-enabled");
    const size_t enabledSignature = rig.GetCompiledRigSignature();
    CHECK(stage->GetPrimAtPath(SdfPath("/Char/Rig/Skeleton/G"))
              .CreateRelationship(TfToken("collection:members:excludes"))
              .SetTargets({Bone("HA")}));
    Near(Wrap(FrameOf(At(rig, 1), "HB").Angle()), 30, 1e-9, "excluded member leaves");
    Near(Wrap(FrameOf(At(rig, 1), "HA").Angle()), 30, 1e-9, "excluded bone is ungrouped");
    CHECK(rig.GetCompiledRigSignature() != enabledSignature);
}

// A RigExecSpaceSwitch on a bone, in the authoring tools' shape (one switch
// per bone under Rig/Spaces, source 0 the namespace parent, the active index
// the bone's own avars:space): the skeleton reads it, at whole and
// fractional indices, over a bone of the skeleton, an outside control and the
// rig root (the base); at rest channels a switch keeps the bone's setup
// placement, then the bone follows the selected space.
void SpaceSwitch()
{
    BoneSpec a;
    a.name = "A"; a.x = 0; a.y = 0; a.angle = 0;
    a.extra = "                double avars:rz.spline = { 1: 90; post held, 2: 90; post held, }\n";
    BoneSpec h;
    h.name = "H"; h.parent = "A"; h.x = 1; h.y = 0; h.angle = 0;
    h.extra = "                double avars:space = 0\n"
              "                double avars:space.spline = { 1: 0; post held, 2: 1; post held, "
              "3: 0.5; post held, 4: 2; post held, 5: 3; post held, }\n";
    BoneSpec t;
    t.name = "T"; t.x = 5; t.y = 5; t.angle = 0;
    // An outside control the hand can hang from, at (2, 0).
    const std::string hook =
        "        def RigExecControl \"Hook\"\n        {\n"
        "            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 0, 0, 1) )\n"
        "            double avars:rz = 90\n"
        "        }\n";
    const std::string sw = Switch("H", "/Char/Rig/Bones/A/H",
        {"/Char/Rig/Bones/A", "/Char/Rig/Bones/T", "/Char/Rig/Hook", "/Char/Rig"},
        "                rel rigExec:activeSpaceAttribute = </Char/Rig/Bones/A/H.avars:space>\n");
    auto stage = Stage(Usda({a, h, t}, hook, sw));
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    // 1: under the arm (turned 90: the tip is up).
    const Frame2 under1 = FrameOf(At(rig, 1), "H");
    Near(under1.x, 0.0, 1e-12, "switch arm x");
    Near(under1.y, 1.0, 1e-12, "switch arm y");
    Near(Wrap(under1.Angle()), 90, 1e-9, "switch arm angle");
    // 2: under the table at rest channels: the setup placement.
    const Frame2 under2 = FrameOf(At(rig, 2), "H");
    Near(under2.x, 1.0, 1e-12, "switch table x");
    Near(under2.y, 0.0, 1e-12, "switch table y");
    // 3: half way: the origin lerps, the angle takes the shortest arc.
    const Frame2 half = FrameOf(At(rig, 3), "H");
    Near(half.x, 0.5, 1e-12, "switch half x");
    Near(half.y, 0.5, 1e-12, "switch half y");
    Near(Wrap(half.Angle()), 45, 1e-9, "switch half angle");
    // 4: the outside control, turned 90 about (2, 0).
    const Frame2 hooked = FrameOf(At(rig, 4), "H");
    Near(hooked.x, 2.0, 1e-12, "switch hook x");
    Near(hooked.y, -1.0, 1e-12, "switch hook y");
    Near(Wrap(hooked.Angle()), 90, 1e-9, "switch hook angle");
    // 5: the rig root: the base, the setup placement whatever moves.
    const Frame2 based = FrameOf(At(rig, 5), "H");
    Near(based.x, 1.0, 1e-12, "switch base x");
    Near(based.y, 0.0, 1e-12, "switch base y");
    // From then on the hand follows the table, not the arm.
    CHECK(stage->GetPrimAtPath(Bone("T")).GetAttribute(TfToken("avars:tx")).Set(2.0));
    const Frame2 follows = FrameOf(At(rig, 2), "H");
    Near(follows.x, 3.0, 1e-12, "switch follows x");
    Near(follows.y, 0.0, 1e-12, "switch follows y");
    // The switch is read by the skeleton, never run as its own step: the
    // index is a channel, its sources are structure.
    const size_t signature = rig.GetCompiledRigSignature();
    CHECK(stage->GetPrimAtPath(Bone("H")).GetAttribute(TfToken("avars:space")).Set(1.0));
    At(rig, 1);
    CHECK(rig.GetCompiledRigSignature() == signature);
    CHECK(stage->GetPrimAtPath(SdfPath("/Char/Rig/Spaces/H"))
              .GetRelationship(TfToken("rigExec:sources"))
              .SetTargets({Bone("A"), Bone("T")}));
    At(rig, 1);
    CHECK(rig.GetCompiledRigSignature() != signature);
    Parity(stage, "space switch", {1, 2, 3});
    auto fresh = Stage(Usda({a, h, t}, hook, sw));
    Parity(fresh, "space switch sources", {1, 2, 3, 4, 5},
           Bone("H").AppendProperty(TfToken("avars:space")), 2.5);
    // A lock below a switched bone reaches through its active bone parent.
    BoneSpec f = AtTip(h, 0.5, 0, "F");
    f.extra = "                bool bone2d:lock = 1\n"
              "                double bone2d:lockX = 0.3\n"
              "                double bone2d:lockY = 1.4\n";
    auto locked = Stage(Usda({a, h, t, f}, hook, sw));
    RigExecRigEvaluator lockedRig(locked, kRig);
    CHECK(Compiles(lockedRig));
    // At 1 the hand hangs from the arm (its base at (0, 1)): the lock turns
    // the hand so the finger's base, 0.5 along it, meets (0.3, 1.4).
    const Frame2 lf = FrameOf(At(lockedRig, 1), "F");
    Near(lf.x, 0.3, 1e-6, "switched lock x");
    Near(lf.y, 1.4, 1e-6, "switched lock y");
    Parity(locked, "switched lock", {1, 2});
}

// A skeleton hanging from a start frame: a tilted joint posed by a
// RigExecParentConstraint (a 2D face on a 3D head). The bones lie in the
// head's XY plane, turn about its +Z and follow it.
void StartFrame()
{
    // The head's rest turns 90 about X: its XY plane is the rig's XZ plane,
    // its +Z the rig's -Y. It sits at (1, 2, 3). The driver turns about its
    // own Y (the rig's +Z) at 3, which tilts the plane.
    const std::string rest =
        "( (1, 0, 0, 0), (0, 0, 1, 0), (0, -1, 0, 0), (1, 2, 3, 1) )";
    // The driver sits where the head does, so the constraint (no offsets)
    // poses the head as the driver's frame.
    const std::string head =
        "        def RigExecControl \"Driver\"\n        {\n"
        "            matrix4d rest:space = " + rest + "\n"
        "            double avars:tx.spline = { 0: 0; post held, 1: 0; post held, 2: 10; post held, }\n"
        "            double avars:ry.spline = { 0: 0; post held, 1: 0; post held, 2: 0; post held, 3: 90; post held, }\n"
        "        }\n"
        "        def RigExecJoint \"Head\"\n        {\n"
        "            matrix4d rest:space = " + rest + "\n"
        "        }\n"
        "        def RigExecParentConstraint \"Follow\" (prepend apiSchemas = [\"RigExecMoverAPI\"])\n"
        "        {\n"
        "            float inputs:defaultWeight = 1\n"
        "            rel rigExec:moves = </Char/Rig/Head>\n"
        "            rel rigExec:sources = </Char/Rig/Driver>\n"
        "            uniform bool rigExec:maintainOffset = 1\n"
        "        }\n";
    // Bones in the head's plane: a root A at plane (0, 0) along plane +X
    // and B at plane (1, 0). Their absolute rests in the rig's space:
    // origin (1 + px, 2, 3 + py), X (c, 0, s), Y (-s, 0, c), Z (0, -1, 0).
    const auto lifted = [](double px, double py, double angle) {
        const double c = std::cos(angle * kDeg), n = std::sin(angle * kDeg);
        return "( (" + Num(c) + ", 0, " + Num(n) + ", 0), (" + Num(-n) + ", 0, " + Num(c) +
               ", 0), (0, -1, 0, 0), (" + Num(1 + px) + ", 2, " + Num(3 + py) + ", 1) )";
    };
    const std::string usda =
        "#usda 1.0\n(\n    startTimeCode = 0\n    endTimeCode = 30\n)\n"
        "def Xform \"Char\"\n{\n    def RigExecRoot \"Rig\"\n    {\n"
        "        def RigExecSkeleton2d \"Skeleton\"\n        {\n"
        "            rel rigExec:startFrame = </Char/Rig/Head>\n"
        "            rel rigExec:joints = [</Char/Rig/Bones/A>, </Char/Rig/Bones/A/B>]\n"
        "        }\n"
        "        def Scope \"Bones\"\n        {\n"
        "            def RigExecJoint \"A\" (prepend apiSchemas = [\"RigExecBone2dAPI\"])\n"
        "            {\n"
        "                matrix4d rest:space = " + lifted(0, 0, 0) + "\n"
        "                double avars:rz.spline = { 0: 0; post held, 1: 90; post held, }\n"
        "                def RigExecJoint \"B\" (prepend apiSchemas = [\"RigExecBone2dAPI\"])\n"
        "                {\n"
        "                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (1, 0, 0, 1) )\n"
        "                }\n"
        "            }\n"
        "        }\n" + head +
        "    }\n}\n";
    BonePaths().clear();
    BonePaths()["A"] = "/Char/Rig/Bones/A";
    BonePaths()["B"] = "/Char/Rig/Bones/A/B";
    auto stage = Stage(usda);
    RigExecRigEvaluator rig(stage, kRig);
    CHECK(Compiles(rig));
    // 0: the setup. 1: A turns 90 about the plane's +Z (the rig's -Y): its
    // X axis becomes the plane's +Y (the rig's +Z), B one unit along it.
    const RigExecRigPose one = At(rig, 1);
    const auto origin = [&](const RigExecRigPose &pose, const std::string &bone) {
        return pose.jointFramesFinal.at(Bone(bone)).points[0];
    };
    const GfVec3d b1 = origin(one, "B");
    Near(b1[0], 1, 1e-12, "start frame B x");
    Near(b1[1], 2, 1e-12, "start frame B y");
    Near(b1[2], 4, 1e-12, "start frame B z");
    const auto &a1 = one.jointFramesFinal.at(Bone("A")).points;
    Near(a1[1][2] - a1[0][2], 1, 1e-12, "start frame A turns about the plane's Z");
    // 2: the head follows the driver 10 along X; the bones follow it.
    const GfVec3d b2 = origin(At(rig, 2), "B");
    Near(b2[0], 11, 1e-9, "start frame follows x");
    Near(b2[1], 2, 1e-9, "start frame follows y");
    Near(b2[2], 4, 1e-9, "start frame follows z");
    // 3: the head turns 90 about the rig's Y with the driver: the bones turn
    // with their plane, and stay in it.
    const RigExecRigPose three = At(rig, 3);
    const auto &head3 = three.jointFramesFinal.at(SdfPath("/Char/Rig/Head")).points;
    const auto &b3 = three.jointFramesFinal.at(Bone("B")).points;
    const GfVec3d planeZ = head3[3] - head3[0];
    Near(GfDot(b3[0] - head3[0], planeZ), 0, 1e-9, "start frame keeps B in the plane");
    Near(GfDot(b3[1] - b3[0], planeZ), 0, 1e-9, "start frame keeps B's X in the plane");
    Near((b3[0] - head3[0]).GetLength(), 1, 1e-9, "start frame B one from the head");
    Parity(stage, "start frame", {0, 1, 2, 3}, Bone("A").AppendProperty(TfToken("avars:rz")), 45);
    // Without the start frame these rests are not in the asset space's plane.
    std::string flat = usda;
    const std::string startRel = "            rel rigExec:startFrame = </Char/Rig/Head>\n";
    flat.erase(flat.find(startRel), startRel.size());
    Refused(flat, "not a rotation about +Z");
    // The policy finds the head when the bones nest under it.
    std::string nested = usda;
    nested.erase(nested.find(startRel), startRel.size());
    nested.insert(nested.find("            rel rigExec:joints"),
                  "            uniform token rigExec:startFramePolicy = \"parent\"\n");
    // Move the bones under the head (A's rest becomes head-relative: plane
    // (0, 0) at the head's origin is identity).
    const std::string bonesOpen = "        def Scope \"Bones\"\n        {\n";
    const size_t bonesAt = nested.find(bonesOpen);
    const size_t bonesEnd = nested.find("        }\n        def RigExecControl \"Driver\"");
    CHECK(bonesAt != std::string::npos && bonesEnd != std::string::npos);
    if (bonesAt == std::string::npos || bonesEnd == std::string::npos) return;
    std::string bones = nested.substr(bonesAt + bonesOpen.size(), bonesEnd - bonesAt - bonesOpen.size());
    nested.erase(bonesAt, bonesEnd + 10 - bonesAt);
    const std::string aRest = lifted(0, 0, 0);
    bones.replace(bones.find(aRest), aRest.size(),
                  "( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )");
    const std::string headOpen = "            matrix4d rest:space = " + rest + "\n";
    nested.insert(nested.find(headOpen, nested.find("def RigExecJoint \"Head\"")) +
                      headOpen.size(),
                  bones);
    const std::string joints = "[</Char/Rig/Bones/A>, </Char/Rig/Bones/A/B>]";
    nested.replace(nested.find(joints), joints.size(),
                   "[</Char/Rig/Head/A>, </Char/Rig/Head/A/B>]");
    BonePaths()["A"] = "/Char/Rig/Head/A";
    BonePaths()["B"] = "/Char/Rig/Head/A/B";
    auto derived = Stage(nested);
    RigExecRigEvaluator derivedRig(derived, kRig);
    CHECK(Compiles(derivedRig));
    const GfVec3d d2 = derivedRig.Evaluate(UsdTimeCode(2)).jointFramesFinal.at(Bone("B")).points[0];
    Near(d2[0], 11, 1e-9, "derived start frame x");
    Near(d2[2], 4, 1e-9, "derived start frame z");
    Parity(derived, "derived start frame", {1, 2, 3});
}

// Rigging refusals: every evaluator refuses the same malformed rigs.
void RiggingRefusals()
{
    BoneSpec a;
    a.name = "A"; a.x = 0; a.y = 0; a.angle = 0;
    BoneSpec b = AtTip(a, 1.0, 0, "B");
    // Two switches on one bone.
    Refused(Usda({a, b}, "",
                 Switch("B", "/Char/Rig/Bones/A/B", {"/Char/Rig/Bones/A"},
                        "                double inputs:activeSpace = 0\n") +
                 "        def RigExecSpaceSwitch \"Again\"\n        {\n"
                 "            rel rigExec:target = </Char/Rig/Bones/A/B>\n"
                 "            rel rigExec:sources = </Char/Rig>\n        }\n"),
            "more than one space switch");
    BoneSpec target = b;
    target.extra = "                rel bone2d:targetCandidates = </Char/Rig/Skeleton>\n";
    Refused(Usda({a, target}), "not a bone of this skeleton");
    BoneSpec narcissist = b;
    narcissist.extra = "                rel bone2d:targetCandidates = </Char/Rig/Bones/A/B>\n";
    Refused(Usda({a, narcissist}), "targets itself");
    // Controls: one controller per kind, a bone, never itself.
    BoneSpec vain = b;
    vain.extra = "                rel bone2d:angleControl = </Char/Rig/Bones/A/B>\n";
    Refused(Usda({a, vain}), "controls itself");
    BoneSpec greedy = b;
    greedy.extra = "                rel bone2d:angleControl = [</Char/Rig/Bones/A>, </Char/Rig/Bones/A/B>]\n";
    Refused(Usda({a, greedy}), "at most one");
    BoneSpec foreign = b;
    foreign.extra = "                rel bone2d:posControl = </Char/Rig/Skeleton>\n";
    Refused(Usda({a, foreign}), "not a bone of this skeleton");
    // Groups hold bones of the skeleton, each in at most one group.
    GroupSpec bad;
    bad.name = "G"; bad.members = {"Missing"};
    Refused(Usda({a, b}, "", "", {bad}), "not a joint of this solver");
    GroupSpec first, second;
    first.name = "G"; first.members = {"B"};
    second.name = "H"; second.members = {"A", "B"};
    Refused(Usda({a, b}, "", "", {first, second}), "is a member of two joint groups");
    // The skeleton holds groups: a bone targeted as a group is refused.
    GroupSpec ok;
    ok.name = "G"; ok.members = {"B"};
    std::string impostor = Usda({a, b}, "", "", {ok});
    const std::string groupPath = "</Char/Rig/Skeleton/G>";
    const size_t relAt = impostor.find(groupPath);
    CHECK(relAt != std::string::npos);
    if (relAt != std::string::npos) {
        impostor.replace(relAt, groupPath.size(), "</Char/Rig/Bones/A>");
    }
    Refused(impostor, "not an active prim with RigExecJointGroupAPI");
    // A group's collection lists its members explicitly.
    GroupSpec expanding;
    expanding.name = "G"; expanding.members = {"B"};
    expanding.extra = "                uniform token collection:members:expansionRule = \"expandPrims\"\n";
    Refused(Usda({a, b}, "", "", {expanding}), "must list its members explicitly");
    GroupSpec pattern;
    pattern.name = "G"; pattern.members = {"B"};
    pattern.extra = "                pathExpression collection:members:membershipExpression = \"/Char/Rig/Bones/*\"\n";
    Refused(Usda({a, b}, "", "", {pattern}), "membershipExpression");
    // Every refusal above leaves the bones' paths to the last rig.
    Usda({a, b});
}

}  // namespace

int main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    IndependentAngle();
    Lock();
    LockWithinLimits();
    CrossScale();
    Flips();
    ParentScale();
    ActionOffset();
    Edits();
    CustomProviderInputs();
    LiveRest();
    TipIsSetup();
    Refusals();
    Nesting();
    Offset();
    IkTarget();
    ControlBones();
    JointGroups();
    SpaceSwitch();
    StartFrame();
    RiggingRefusals();
    if (failures) {
        std::printf("testRigExecSkeleton2d: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecSkeleton2d: ok\n");
    return 0;
}
