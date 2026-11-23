// A scene solver or constraint run as rigExecOps' pose ops gives the bits
// RigExecRunSolver and RigExecRunConstraint give: per kind over its
// settings, through refusals and a moved setting's rebuild, and per
// operation of authored rigs, where the scene program's own publish, a
// native rerun and an op rerun are compared value for value.
#include "rigExecScene/poseOpRunner.h"
#include "rigExecScene/poseOps.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/types.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecMath/splineIk.h"
#include "rigExecScene/constraintGraphBinding.h"
#include "rigExecScene/solverGraphBinding.h"
#include "rigExecSceneProgramDrive.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/primRange.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <variant>
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

using Rest = std::array<GfVec3d, 4>;

RigExecPointFrame
Frame(const GfVec3d &origin, const GfVec3d &axis = GfVec3d(0, 0, 1),
      double degrees = 0.0, const GfVec3d &scale = GfVec3d(1.0))
{
    GfMatrix4d s(1.0), r(1.0), t(1.0);
    s.SetScale(scale);
    r.SetRotate(GfRotation(axis, degrees));
    t.SetTranslate(origin);
    const GfMatrix4d m = s * r * t;
    RigExecPointFrame f;
    f.points = {m.Transform(GfVec3d(0)), m.Transform(GfVec3d(1, 0, 0)),
                m.Transform(GfVec3d(0, 1, 0)), m.Transform(GfVec3d(0, 0, 1))};
    return f;
}

Rest
RestAt(const GfVec3d &origin, double degrees = 0.0)
{
    return Frame(origin, GfVec3d(0.2, 0.3, 1.0), degrees).points;
}

bool
Bits(const void *a, const void *b, size_t n)
{
    return std::memcmp(a, b, n) == 0;
}

bool
SameRest(const Rest &a, const Rest &b)
{
    for (size_t k = 0; k < 4; ++k)
        if (!Bits(a[k].data(), b[k].data(), 3 * sizeof(double)))
            return false;
    return true;
}

bool
SameFrame(const RigExecPointFrame &a, const RigExecPointFrame &b)
{
    return a.flags == b.flags && SameRest(a.points, b.points);
}

bool
SameArray(const RigExecPointFrameArray &a, const RigExecPointFrameArray &b)
{
    if (a.frames.size() != b.frames.size() || a.rests.size() != b.rests.size())
        return false;
    for (size_t k = 0; k < a.frames.size(); ++k)
        if (!SameFrame(a.frames[k], b.frames[k]))
            return false;
    for (size_t k = 0; k < a.rests.size(); ++k)
        if (!SameRest(a.rests[k], b.rests[k]))
            return false;
    return true;
}

/// Per kind, the comparisons whose native answer moved a frame.
std::map<std::string, int> solvedFrames;

/// Native against the op, one frame, on \p runner.
void
CompareSolver(const std::string &label, const RigExecSolverRecord &s,
              const RigExecSolverInputs &input, RigExecPoseOpRunner &runner)
{
    CHECK(RigExecSolverOpSupported(s));
    RigExecSolverWorkspace workspace;
    RigExecPointFrameArray want, got;
    std::string wantError, gotError;
    const bool native =
        RigExecRunSolver(s, input, &workspace, &want, &wantError);
    const bool op = RigExecRunSolverOp(s, input, runner, &got, &gotError);
    const bool same = native == op && (native || wantError == gotError) &&
                      SameArray(want, got);
    if (!same)
        std::printf("%s: native %d (%s, %zu frames), op %d (%s, %zu frames)\n",
                    label.c_str(), int(native), wantError.c_str(),
                    want.frames.size(), int(op), gotError.c_str(),
                    got.frames.size());
    CHECK(same);
    if (native && !want.frames.empty())
        ++solvedFrames[label.substr(0, label.find(' '))];
}

void
CompareConstraint(const std::string &label, const RigExecConstraintRecord &r,
                  const RigExecConstraintInputs &input,
                  RigExecPoseOpRunner &runner)
{
    CHECK(RigExecConstraintOpSupported(r.kind));
    RigExecConstraintResult want, got;
    std::string wantError, gotError;
    const bool native = RigExecRunConstraint(r, input, &want, &wantError);
    const bool op = RigExecRunConstraintOp(r, input, runner, &got, &gotError);
    bool same = native == op && SameFrame(want.frame, got.frame) &&
                want.chain.size() == got.chain.size();
    for (size_t k = 0; same && k < want.chain.size(); ++k)
        same = SameFrame(want.chain[k], got.chain[k]);
    if (!same)
        std::printf("%s: constraint differs (%s / %s)\n", label.c_str(),
                    wantError.c_str(), gotError.c_str());
    CHECK(same);
    if (native && (!SameFrame(want.frame, input.incoming) ||
                   (!want.chain.empty() &&
                    !SameFrame(want.chain.back(), input.chain.back()))))
        ++solvedFrames["constraint"];
}

// Solvers.

RigExecSolverRecord
FkRecord(bool start, bool parentRelative, bool scale, bool live)
{
    RigExecSolverRecord s;
    s.kind = RigExecSolverKind::FkChain;
    s.hasStart = start;
    s.parentRelative = parentRelative;
    s.scaleSegments = scale;
    s.startRest = RestAt(GfVec3d(-1, 0, 0), 10);
    for (int k = 0; k < 4; ++k) {
        s.controlRests.push_back(RestAt(GfVec3d(2.0 * k, 0.1 * k, 0), 5.0 * k));
        s.jointRests.push_back(RestAt(GfVec3d(2.0 * k, 0.3, -0.2), -7.0 * k));
        s.restIsLive.push_back(live && k % 2 == 0);
    }
    return s;
}

RigExecSolverInputs
FkInputs(double t)
{
    RigExecSolverInputs in;
    in.start = Frame(GfVec3d(-1, 0.2 * t, 0), GfVec3d(0, 1, 0), 20.0 * t);
    for (int k = 0; k < 4; ++k)
        in.controls.push_back(
            Frame(GfVec3d(2.0 * k + 0.3 * t, 0.5 * t, 0.1 * k),
                  GfVec3d(0.1, 0.4 * k, 1.0), 15.0 * k * t,
                  GfVec3d(1.0 + 0.1 * k * t, 1.0, 0.9)));
    return in;
}

void
TestFkChain()
{
    for (const bool start : {false, true})
        for (const bool relative : {false, true})
            for (const bool scale : {false, true})
                for (const bool live : {false, true}) {
                    RigExecPoseOpRunner runner;
                    const RigExecSolverRecord s =
                        FkRecord(start, relative, scale, live);
                    for (const double t : {0.0, 0.5, 1.0})
                        CompareSolver("fk " + std::to_string(start) +
                                          std::to_string(relative) +
                                          std::to_string(scale) +
                                          std::to_string(live),
                                      s, FkInputs(t), runner);
                }
    // A control count the rests do not match: native's own refusal.
    RigExecPoseOpRunner runner;
    RigExecSolverInputs shortInputs = FkInputs(1.0);
    shortInputs.controls.pop_back();
    CompareSolver("fk cardinality", FkRecord(true, false, false, false),
                  shortInputs, runner);
    // Joint groups stay native.
    RigExecSolverRecord grouped = FkRecord(false, false, false, false);
    grouped.jointGroups.resize(1);
    CHECK(!RigExecSolverOpSupported(grouped));
}

RigExecSolverRecord
IkRecord(bool softDistance, bool scale)
{
    RigExecSolverRecord s;
    s.kind = RigExecSolverKind::TwoBoneIk;
    s.ikRests = {RestAt(GfVec3d(0, 0, 0), 3), RestAt(GfVec3d(3, 0.2, 0), 8),
                 RestAt(GfVec3d(6, 0, 0.1), -4)};
    s.ikParams.softDistancePolicy = softDistance;
    s.ikParams.scaleSegments = scale;
    s.ikParams.limb.scaleCalibration = softDistance ? 0.05 : 0.0;
    return s;
}

RigExecSolverInputs
IkInputs(const GfVec3d &goal, bool space)
{
    RigExecSolverInputs in;
    in.root = Frame(GfVec3d(0.1, 0, 0), GfVec3d(0, 0, 1), 4);
    in.end = Frame(goal, GfVec3d(1, 0.2, 0), 30);
    in.pole = Frame(GfVec3d(3, 0.5, 4));
    in.refreshIkParams = true;
    in.bend = 0.3;
    in.upperOffset = 0.1;
    in.lowerOffset = -0.2;
    in.stretch = 0.7f;
    in.softness = 0.2f;
    in.pin = 0.25f;
    in.upperScale = 1.1;
    in.lowerScale = 0.95;
    in.softDistance = 0.4f;
    in.limbTwist = 12.0f;
    in.ikSpace.SetScale(GfVec3d(1.2, 1.2, 1.2));
    in.hasSpace = space;
    in.spaceRest = RestAt(GfVec3d(0), 0);
    in.space = Frame(GfVec3d(0.5, -0.25, 0), GfVec3d(0, 1, 0), 15.0);
    return in;
}

void
TestTwoBoneIk()
{
    for (const bool softDistance : {false, true})
        for (const bool scale : {false, true})
            for (const bool space : {false, true}) {
                RigExecPoseOpRunner runner;
                const RigExecSolverRecord s = IkRecord(softDistance, scale);
                const std::string label =
                    "two-bone " + std::to_string(softDistance) +
                    std::to_string(scale) + std::to_string(space);
                // Inside reach, near full reach and past it.
                for (const GfVec3d &goal :
                     {GfVec3d(3.5, 2.0, 1.0), GfVec3d(5.9, 0.5, 0.0),
                      GfVec3d(9.0, 1.0, -1.0)})
                    CompareSolver(label, s, IkInputs(goal, space), runner);
            }
}

RigExecPointFrameArray
Aggregate(double shift, size_t n)
{
    RigExecPointFrameArray a;
    for (size_t k = 0; k < n; ++k) {
        a.frames.push_back(Frame(GfVec3d(2.0 * k, shift, 0),
                                 GfVec3d(0.3, 1.0, 0.2), 40.0 * shift + 9.0 * k,
                                 GfVec3d(1.0 + 0.2 * shift, 1.0, 1.0)));
        a.rests.push_back(RestAt(GfVec3d(2.0 * k, 0, 0), 2.0 * k));
    }
    return a;
}

void
TestBlendFrames()
{
    const RigExecPointFrameArray a = Aggregate(0.0, 3), b = Aggregate(1.0, 3);
    for (const RigExecScaleBlend mode :
         {RigExecScaleBlend::Log, RigExecScaleBlend::Linear})
        for (const bool live : {false, true}) {
            RigExecSolverRecord s;
            s.kind = RigExecSolverKind::BlendPointFrames;
            s.scaleMode = mode;
            for (int k = 0; k < 3; ++k) {
                s.jointRests.push_back(RestAt(GfVec3d(2.0 * k, 0.4, 0), 11));
                s.restIsLive.push_back(live && k != 1);
            }
            RigExecPoseOpRunner runner;
            for (const double w : {-0.5, 0.0, 0.35, 1.0, 1.5,
                                   std::numeric_limits<double>::quiet_NaN()}) {
                RigExecSolverInputs in;
                in.blendA = &a;
                in.blendB = &b;
                in.blendWeight = w;
                CompareSolver("blend", s, in, runner);
            }
            // One input unwired passes the other through; none gives none.
            RigExecSolverInputs one;
            one.blendB = &b;
            CompareSolver("blend pass-through", s, one, runner);
            one.blendA = &a;
            one.blendB = nullptr;
            CompareSolver("blend pass-through a", s, one, runner);
            CompareSolver("blend none", s, RigExecSolverInputs(), runner);
            // Mismatched sizes publish nothing; a longer pair rebuilds.
            const RigExecPointFrameArray shorter = Aggregate(1.0, 2),
                                         longer = Aggregate(0.5, 5),
                                         longerB = Aggregate(0.25, 5);
            RigExecSolverInputs mismatch;
            mismatch.blendA = &a;
            mismatch.blendB = &shorter;
            mismatch.blendWeight = 0.5;
            CompareSolver("blend mismatch", s, mismatch, runner);
            mismatch.blendA = &longer;
            mismatch.blendB = &longerB;
            CompareSolver("blend rebuilt", s, mismatch, runner);
        }
}

void
TestTwist()
{
    for (const bool live : {false, true}) {
        RigExecSolverRecord s;
        s.kind = RigExecSolverKind::TwistDistribution;
        s.twistStartRest = RestAt(GfVec3d(0), 0);
        s.twistEndRest = RestAt(GfVec3d(4, 0, 0), 0);
        s.twistWeights = {0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0};
        for (int k = 0; k < 4; ++k) {
            s.jointRests.push_back(RestAt(GfVec3d(1.3 * k, 0.1, 0), 3.0 * k));
            s.restIsLive.push_back(live && k % 2 == 1);
        }
        RigExecPoseOpRunner runner;
        for (const double turns :
             {0.0, 1.0, std::numeric_limits<double>::infinity()}) {
            RigExecSolverInputs in;
            in.root = Frame(GfVec3d(0.2, 0, 0), GfVec3d(1, 0, 0), 15);
            in.end = Frame(GfVec3d(4, 0.3, 0), GfVec3d(1, 0, 0), 200);
            in.twistTurns = turns;
            CompareSolver("twist", s, in, runner);
        }
    }
}

void
TestRibbon()
{
    std::vector<GfVec3f> rest, posed;
    for (int i = 0; i < 6; ++i) {
        rest.emplace_back(1.4f * i, 0.5f * i, 0.3f * (i % 2));
        posed.emplace_back(1.3f * i, 0.5f * i + 0.2f * i * i,
                           0.4f * i - 0.3f * (i % 2));
    }
    for (const bool live : {false, true}) {
        RigExecSolverRecord s;
        s.kind = RigExecSolverKind::Ribbon;
        s.ribbonRestPoints = rest;
        for (int k = 0; k < 5; ++k) {
            s.jointRests.push_back(RestAt(GfVec3d(1.5 * k, 0.2, 0), 6.0 * k));
            s.restIsLive.push_back(live && k != 2);
        }
        RigExecPoseOpRunner runner;
        // Five samples, a moved count (rebuilt), one sample and no curve:
        // the last two publish nothing.
        for (const int count : {5, 3, 1}) {
            RigExecSolverInputs in;
            in.ribbonPoints = posed;
            in.ribbonSampleCount = count;
            CompareSolver("ribbon " + std::to_string(count), s, in, runner);
        }
        RigExecSolverInputs none;
        none.ribbonSampleCount = 5;
        CompareSolver("ribbon no curve", s, none, runner);
    }
}

RigExecSolverRecord
SplineRecord(bool chain, bool aim, bool weighted)
{
    RigExecSolverRecord s;
    s.kind = RigExecSolverKind::SplineIk;
    s.splineCount = 5;
    for (int k = 0; k < 5; ++k) {
        RigExecPointFrame f = Frame(GfVec3d(0, 1.0 * k, 0.05 * k * k),
                                    GfVec3d(0, 0, 1), 90.0 + 2.0 * k);
        s.splineRestFrames.push_back(f);
        s.splineJointRests.push_back(f.points);
        if (weighted)
            s.splineRestWeights.push_back(0.25 * k);
    }
    s.splineRootRest = Frame(GfVec3d(0, 0, 0));
    s.splineMidRest = Frame(GfVec3d(0, 2, 0.1));
    s.splineEndRest = Frame(GfVec3d(0, 4, 0.8));
    s.splineRestMode = chain ? RigExecSplineIkRestLength::Chain
                             : RigExecSplineIkRestLength::Curve;
    s.splineParams.aimRootTangent = aim;
    s.splineRest = RigExecSplineIkMakeRest(
        s.splineRestFrames, s.splineRootRest, s.splineMidRest, s.splineEndRest,
        s.splineRestWeights, s.splineRestMode);
    return s;
}

RigExecSolverInputs
SplineInputs(double t, double minLength, bool space)
{
    RigExecSolverInputs in;
    in.root = Frame(GfVec3d(0.1 * t, 0, 0), GfVec3d(1, 0, 0), 10.0 * t);
    in.mid = Frame(GfVec3d(0.6 * t, 2.1, 0.2), GfVec3d(0, 0, 1), -8.0 * t);
    in.end = Frame(GfVec3d(1.2 * t, 3.4 + t, 0.5), GfVec3d(0, 1, 0), 35.0 * t);
    in.refreshSplineParams = true;
    in.preserveVolume = 0.8;
    in.midFollowWeight = 0.4;
    in.rollDegrees = 12.0;
    in.twistDegrees = 25.0 * t;
    in.minLengthRatio = minLength;
    in.hasSpace = space;
    in.spaceRest = RestAt(GfVec3d(0), 0);
    in.space = Frame(GfVec3d(0.3, 0.2, 0), GfVec3d(0, 0, 1), 20.0);
    return in;
}

void
TestSplineIk()
{
    for (const bool chain : {false, true})
        for (const bool aim : {false, true})
            for (const bool weighted : {false, true})
                for (const bool space : {false, true}) {
                    RigExecPoseOpRunner runner;
                    const RigExecSolverRecord s =
                        SplineRecord(chain, aim, weighted);
                    // The floor moving between frames rebuilds the node.
                    for (const double minLength : {0.0, 0.95})
                        for (const double t : {0.0, 1.0, -0.7})
                            CompareSolver("spline", s,
                                          SplineInputs(t, minLength, space),
                                          runner);
                }
}

/// A node built from one record refuses a write whose baked settings moved,
/// and the runner rebuilds it.
void
TestStaleSettings()
{
    std::string error;
    CHECK(RigExecEnsureBuiltinRegistry(&error));
    const RigExecSolverRecord fk = FkRecord(true, false, false, false),
                              scaled = FkRecord(true, false, true, false);
    RigExecRigDescBuilder builder;
    RigExecSolverOpNode node;
    CHECK(RigExecAddSolverOp(fk, FkInputs(1.0), &builder, &node, &error));
    const auto desc = builder.Build(&error);
    const auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    if (!evaluator)
        return;
    RigExecPoseOpScratch scratch;
    CHECK(RigExecWriteSolverOpInputs(fk, FkInputs(1.0), node, &scratch,
                                     evaluator.get(), &error));
    CHECK(!RigExecWriteSolverOpInputs(scaled, FkInputs(1.0), node, &scratch,
                                      evaluator.get(), &error));
    CHECK(error.find("moved") != std::string::npos);
    RigExecPoseOpRunner runner;
    CompareSolver("stale fk", fk, FkInputs(1.0), runner);
    CompareSolver("rebuilt fk", scaled, FkInputs(1.0), runner);
    CompareSolver("rebuilt ik", IkRecord(true, false),
                  IkInputs(GfVec3d(4, 1, 0), false), runner);
    // A record the bridge does not map is refused at build.
    RigExecSolverRecord skeleton;
    skeleton.kind = RigExecSolverKind::Skeleton2d;
    CHECK(!RigExecSolverOpSupported(skeleton));
    RigExecRigDescBuilder other;
    CHECK(!RigExecAddSolverOp(skeleton, RigExecSolverInputs(), &other, &node,
                              &error));
}

// Constraints.

std::vector<RigExecConstraintSource>
Sources()
{
    std::vector<RigExecConstraintSource> s(3);
    s[0].frame = Frame(GfVec3d(3, 1, -2), GfVec3d(1, 0.3, 0), 120.0,
                       GfVec3d(2.0, 0.5, 1.0));
    s[1].frame = Frame(GfVec3d(-1, 4, 0.5), GfVec3d(0, 0.4, 1), -75.0,
                       GfVec3d(0.7, 0.7, 1.9));
    s[2].frame = Frame(GfVec3d(2, 2, 2), GfVec3d(1, 1, 1), 170.0,
                       GfVec3d(1.1, 1.0, 0.9));
    const double weights[3] = {0.5, 1.25, 0.0};
    for (size_t k = 0; k < 3; ++k) {
        s[k].normalizedWeight = weights[k];
        s[k].translationOffset = GfVec3d(0.1 * k, -0.2, 0.3);
        s[k].rotationOffsetDegrees = GfVec3d(5.0 * k, 0, -10);
    }
    return s;
}

RigExecConstraintInputs
ConstraintInputs(bool carry)
{
    RigExecConstraintInputs in;
    in.incoming = Frame(GfVec3d(0.5, -1.0, 2.0), GfVec3d(0.2, 1.0, -0.4), 37.0,
                        GfVec3d(1.0, 1.3, 0.8));
    in.sources = Sources();
    if (carry) {
        GfMatrix4d m(1.0);
        m.SetRotate(GfRotation(GfVec3d(0.3, 0.1, 1.0), 64.0));
        m.SetTranslateOnly(GfVec3d(4.0, -2.0, 1.0));
        in.carry = m;
    }
    in.aimTarget = GfVec3d(4, 3, -1);
    return in;
}

void
TestConstraints()
{
    const RigExecConstraintAxisMask partial{true, false, true};
    for (const double weight : {0.0, 0.6, 1.0})
        for (const bool carry : {false, true})
            for (const bool masked : {false, true}) {
                const RigExecConstraintAxisMask mask =
                    masked ? partial : RigExecConstraintAxisMask();
                const RigExecConstraintInputs in = ConstraintInputs(carry);
                RigExecPoseOpRunner runner;
                RigExecConstraintRecord r;
                r.kind = RigExecConstraintKind::Position;
                r.position.offset = GfVec3d(0.25, 0, -0.5);
                r.position.affect = mask;
                r.position.weight = weight;
                CompareConstraint("position", r, in, runner);
                r.kind = RigExecConstraintKind::Rotation;
                r.rotation.offsetDegrees = GfVec3d(10, -20, 5);
                r.rotation.affect = mask;
                r.rotation.rotationOrder = RigExecEulerOrder::ZXY;
                r.rotation.weight = weight;
                CompareConstraint("rotation", r, in, runner);
                r.kind = RigExecConstraintKind::Scale;
                r.scale.offset = GfVec3d(0.1, 0, 0);
                r.scale.affect = mask;
                r.scale.weight = weight;
                for (const bool shear : {false, true}) {
                    r.scale.blendShear = shear;
                    CompareConstraint("scale", r, in, runner);
                }
                r.kind = RigExecConstraintKind::Parent;
                r.parent.translationAxes = mask;
                r.parent.rotationAxes = partial;
                r.parent.scaleAxes = RigExecConstraintAxisMask();
                r.parent.rotationOrder = RigExecEulerOrder::YZX;
                r.parent.weight = weight;
                for (const bool shear : {false, true}) {
                    r.parent.blendShear = shear;
                    CompareConstraint("parent", r, in, runner);
                }
                r.kind = RigExecConstraintKind::Aim;
                r.aim.localAimVector = GfVec3d(0, 1, 0);
                r.aim.localUpVector = GfVec3d(0, 0, 1);
                r.aim.rotationOffsetDegrees = GfVec3d(0, 5, 0);
                r.aim.affectRotation = mask;
                r.aim.rotationOrder = RigExecEulerOrder::XZY;
                r.aim.weight = weight;
                for (const bool preserve : {false, true}) {
                    r.aim.preserveInputUp = preserve;
                    r.aim.worldUpDirection.reset();
                    CompareConstraint("aim", r, in, runner);
                    r.aim.worldUpDirection = GfVec3d(0.2, 1, 0);
                    CompareConstraint("aim world up", r, in, runner);
                }
            }
    // A degenerate source and a non-finite offset: the kernel's flagged
    // input, as native publishes it.
    RigExecPoseOpRunner runner;
    RigExecConstraintInputs broken = ConstraintInputs(false);
    broken.sources[0].frame.flags |= RigExecPointFrameDegenerate;
    RigExecConstraintRecord r;
    r.kind = RigExecConstraintKind::Rotation;
    r.rotation.weight = 1.0;
    CompareConstraint("rotation broken source", r, broken, runner);
    r.kind = RigExecConstraintKind::Position;
    r.position.offset = GfVec3d(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    CompareConstraint("position nan offset", r, ConstraintInputs(false),
                      runner);
    // A moved offset rebuilds the node.
    r.position.offset = GfVec3d(1, 2, 3);
    CompareConstraint("position moved offset", r, ConstraintInputs(false),
                      runner);
    CHECK(!RigExecConstraintOpSupported(RigExecConstraintKind::IkFkMatch));
    CHECK(!RigExecConstraintOpSupported(RigExecConstraintKind::Harmonics));
}

void
TestSingleChainIk()
{
    RigExecConstraintInputs in;
    for (int k = 0; k < 4; ++k)
        in.chain.push_back(
            Frame(GfVec3d(1.5 * k, 0.1 * k, 0), GfVec3d(0, 0, 1), 3.0 * k));
    for (const bool single : {false, true})
        for (const bool preserve : {false, true})
            for (const double weight : {0.0, 0.5, 1.0})
                for (const double stretch : {0.0, 0.6}) {
                    RigExecPoseOpRunner runner;
                    RigExecConstraintRecord r;
                    r.kind = RigExecConstraintKind::SingleChainIk;
                    r.singleChain.mode =
                        single ? RigExecSingleChainIkMode::SingleChain
                               : RigExecSingleChainIkMode::RotatePlane;
                    r.singleChain.preserveJointOrientation = preserve;
                    r.singleChain.weight = weight;
                    r.singleChain.stretch = stretch;
                    r.singleChain.pole = GfVec3d(2, 3, 4);
                    r.singleChain.twistDegrees = 20;
                    for (const GfVec3d &goal :
                         {GfVec3d(3, 2, 0.5), GfVec3d(7, 1, 0)}) {
                        in.effector = Frame(goal, GfVec3d(0.2, 1, 0), 30.0);
                        CompareConstraint("single-chain", r, in, runner);
                    }
                }
    // A broken chain: the kernel flags every frame.
    RigExecPoseOpRunner runner;
    RigExecConstraintRecord r;
    r.kind = RigExecConstraintKind::SingleChainIk;
    in.chain[1].flags |= RigExecPointFrameDegenerate;
    CompareConstraint("single-chain broken", r, in, runner);
}

// Authored rigs.

const char *const kSingleChainRig = R"usda(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 4
)

def Xform "Asset"
{
    def "Sources"
    {
        def Xform "Effector"
        {
            double3 xformOp:translate.timeSamples = {
                1: (3, 2, 0),
                4: (5, -1, 1),
            }
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "PoleA"
        {
            double3 xformOp:translate = (2, 0, 8)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "PoleB"
        {
            double3 xformOp:translate = (2, 8, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "Big"
        {
            double3 xformOp:scale.timeSamples = {
                1: (2, 1, 0.5),
                4: (1, 3, 1),
            }
            uniform token[] xformOpOrder = ["xformOp:scale"]
        }
    }

    def Xform "Scaled"
    {
        double3 xformOp:translate = (1, 1, 5)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def RigExecRoot "Rig"
    {
        def "Joints"
        {
            def RigExecJoint "Root"
            {
                matrix4d posed:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )

                def RigExecJoint "Mid"
                {
                    matrix4d posed:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 0, 0, 1) )

                    def RigExecJoint "End"
                    {
                        matrix4d posed:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (4, 0, 0, 1) )
                    }
                }
            }
        }

        def Scope "Movers"
        {
            def RigExecSingleChainIkConstraint "IK" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = [</Asset/Rig/Joints/Root>, </Asset/Rig/Joints/Root/Mid>, </Asset/Rig/Joints/Root/Mid/End>]
                rel rigExec:firstJoint = </Asset/Rig/Joints/Root>
                rel rigExec:endJoint = </Asset/Rig/Joints/Root/Mid/End>
                rel rigExec:effector = </Asset/Sources/Effector>
                uniform token rigExec:poleVectorMode = "object"
                rel rigExec:poleVectorObjects = [</Asset/Sources/PoleA>, </Asset/Sources/PoleB>]
                float[] inputs:poleVectorWeights = [1, 3]
                float inputs:stretch.timeSamples = {
                    1: 0,
                    4: 0.5,
                }
            }

            def RigExecScaleConstraint "Scale" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Scaled>
                rel rigExec:sources = [</Asset/Sources/Big>, </Asset/Sources/PoleA>]
                float[] inputs:sourceWeights = [1, 0.5]
                bool inputs:affectScaleY = 0
                double3 inputs:scaleOffset = (0.25, 0, 0)
                float inputs:defaultWeight.timeSamples = {
                    1: 1,
                    4: 0.4,
                }
            }
        }
    }
}
)usda";

std::map<std::string, int> sceneSolvers, sceneConstraints;

const char *
SolverName(RigExecSolverKind kind)
{
    switch (kind) {
    case RigExecSolverKind::FkChain:
        return "FkChain";
    case RigExecSolverKind::TwoBoneIk:
        return "TwoBoneIk";
    case RigExecSolverKind::BlendPointFrames:
        return "BlendPointFrames";
    case RigExecSolverKind::TwistDistribution:
        return "TwistDistribution";
    case RigExecSolverKind::Ribbon:
        return "Ribbon";
    case RigExecSolverKind::SplineIk:
        return "SplineIk";
    case RigExecSolverKind::Skeleton2d:
        return "Skeleton2d";
    }
    return "?";
}

const char *
ConstraintName(RigExecConstraintKind kind)
{
    switch (kind) {
    case RigExecConstraintKind::Position:
        return "Position";
    case RigExecConstraintKind::Rotation:
        return "Rotation";
    case RigExecConstraintKind::Scale:
        return "Scale";
    case RigExecConstraintKind::Parent:
        return "Parent";
    case RigExecConstraintKind::Aim:
        return "Aim";
    case RigExecConstraintKind::SingleChainIk:
        return "SingleChainIk";
    default:
        return "?";
    }
}

/// Bit for bit, including the blocked flag, count and diagnostic.
bool
SameState(const RigExecTypedValueState &a, const RigExecTypedValueState &b)
{
    if (a.initialized != b.initialized || a.blocked != b.blocked ||
        a.count != b.count || a.error != b.error ||
        a.value.index() != b.value.index())
        return false;
    if (const auto *x = std::get_if<RigExecPointFrame>(&a.value))
        return SameFrame(*x, std::get<RigExecPointFrame>(b.value));
    if (const auto *x = std::get_if<VtValue>(&a.value)) {
        const VtValue &y = std::get<VtValue>(b.value);
        if (x->IsHolding<RigExecPointFrameArray>())
            return y.IsHolding<RigExecPointFrameArray>() &&
                   SameArray(x->UncheckedGet<RigExecPointFrameArray>(),
                             y.UncheckedGet<RigExecPointFrameArray>());
        return RigExecTypedSame(*x, y);
    }
    return a.value == b.value;
}

/// Runs \p run twice from the same store, natively and with \p runner, and
/// compares what each published to \p writes, and what the scene program
/// itself published there.
template <class Run>
bool
SamePublish(RigExecTypedValueStore &store,
            const std::vector<RigExecValueId> &writes,
            RigExecPoseOpRunner &runner, const Run &run)
{
    std::vector<RigExecTypedValueState> saved, native;
    for (const RigExecValueId id : writes)
        saved.push_back(store.values[size_t(id)]);
    run(nullptr);
    for (const RigExecValueId id : writes)
        native.push_back(store.values[size_t(id)]);
    for (size_t k = 0; k < writes.size(); ++k)
        store.values[size_t(writes[k])] = saved[k];
    run(&runner);
    bool same = true;
    for (size_t k = 0; k < writes.size(); ++k) {
        same = same && SameState(native[k], saved[k]) &&
               SameState(native[k], store.values[size_t(writes[k])]);
        store.values[size_t(writes[k])] = saved[k];
    }
    return same;
}

void
CompareRig(const std::string &label, const UsdStageRefPtr &stage,
           const SdfPath &rig, int first, int last)
{
    rigExecSceneProgramDrive::SceneRun scene(stage, rig, first, last);
    if (!scene.ok) {
        std::printf("%s: no scene program\n", label.c_str());
        return;
    }
    const auto &operations = scene.program.operations;
    std::vector<RigExecPoseOpRunner> runners(operations.size());
    for (int t = first; t <= last; ++t) {
        CHECK(scene.Evaluate(t));
        RigExecTypedValueStore &store = scene.runtime.values;
        for (size_t i = 0; i < operations.size(); ++i) {
            if (const auto *op =
                    std::get_if<RigExecSolverGraphBinding>(&operations[i])) {
                if (!RigExecSolverOpSupported(op->record))
                    continue;
                RigExecSolverWorkspace workspace;
                const bool same =
                    SamePublish(store, {op->output}, runners[i],
                                [&](RigExecPoseOpRunner *ops) {
                                    std::string error;
                                    RigExecRunSolverGraph(
                                        *op, &store, &workspace, &error, ops);
                                });
                if (!same)
                    std::printf("%s: %s solver %zu differs at %d\n",
                                label.c_str(), SolverName(op->record.kind), i,
                                t);
                CHECK(same);
                ++sceneSolvers[SolverName(op->record.kind)];
            } else if (const auto *op =
                           std::get_if<RigExecConstraintGraphBinding>(
                               &operations[i])) {
                const RigExecConstraintKind kind = op->structural.record.kind;
                if (!RigExecConstraintOpSupported(kind) || op->match ||
                    op->windowFrames)
                    continue;
                std::vector<RigExecValueId> writes = {op->target.output};
                for (const RigExecValueId id : op->target.chainOutputs)
                    if (id < store.values.size())
                        writes.push_back(id);
                const bool same = SamePublish(
                    store, writes, runners[i], [&](RigExecPoseOpRunner *ops) {
                        std::string error;
                        RigExecRunConstraintGraph(*op, &store, &error, nullptr,
                                                  ops);
                    });
                if (!same)
                    std::printf("%s: %s constraint %zu differs at %d\n",
                                label.c_str(), ConstraintName(kind), i, t);
                CHECK(same);
                ++sceneConstraints[ConstraintName(kind)];
            }
        }
    }
}

void
CompareStage(const std::string &label, const UsdStageRefPtr &stage)
{
    if (!stage) {
        std::printf("%s: stage did not open\n", label.c_str());
        ++failures;
        return;
    }
    int first = int(std::floor(stage->GetStartTimeCode()));
    int last = int(std::floor(stage->GetEndTimeCode()));
    if (last < first)
        last = first;
    last = std::min(last, first + 5);
    for (const UsdPrim &prim : stage->Traverse())
        if (prim.GetTypeName() == "RigExecRoot")
            CompareRig(label + " " + prim.GetPath().GetString(), stage,
                       prim.GetPath(), first, last);
}

void
TestAuthoredRigs(const std::string &examples)
{
    const char *files[] = {
        "01_FkChainTail.usda",
        "02_TwoBoneIkLeg.usda",
        "03_IkFkBlendClamp.usda",
        "05_TwistRibbonSpine.usda",
        "08_AimEyes.usda",
        "10_AimXformTurret.usda",
        "13_ReadPhases.usda",
        "14_VolumeConstrainedSweep.usda",
        "16_ConnectionReadPhases.usda",
        "19_HarmonicsChain.usda",
        "ArmRig.usda",
        "aimtest.usda",
        "rotateConstraint.usda",
        "../tests/fixtures/computed_ik_space.usda",
        "../tests/fixtures/computed_chains.usda",
        "../tests/fixtures/frame_record_fallbacks.usda",
        "../tests/fixtures/projector_spaces.usda",
        "../tests/fixtures/oneloop_cross_domain.usda",
        "../tests/fixtures/solver_checkpoint.usda",
    };
    for (const char *name : files)
        CompareStage(name, UsdStage::Open(examples + "/" + name));
    SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(layer->ImportFromString(kSingleChainRig));
    CompareStage("single-chain rig", UsdStage::Open(layer));

    for (const char *kind : {"FkChain", "TwoBoneIk", "BlendPointFrames",
                             "TwistDistribution", "Ribbon", "SplineIk"}) {
        std::printf("scene %s solvers compared: %d\n", kind,
                    sceneSolvers[kind]);
        CHECK(sceneSolvers[kind] > 0);
    }
    for (const char *kind :
         {"Position", "Rotation", "Scale", "Parent", "Aim", "SingleChainIk"}) {
        std::printf("scene %s constraints compared: %d\n", kind,
                    sceneConstraints[kind]);
        CHECK(sceneConstraints[kind] > 0);
    }
}

std::string
DefaultResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

} // namespace

int
main(int argc, char **argv)
{
    std::string error;
    if (!RigExecEnsureBuiltinRegistry(&error)) {
        std::printf("registry: %s\n", error.c_str());
        return 1;
    }
    TestFkChain();
    TestTwoBoneIk();
    TestBlendFrames();
    TestTwist();
    TestRibbon();
    TestSplineIk();
    TestStaleSettings();
    TestConstraints();
    TestSingleChainIk();
    // Each kind's comparisons include solved frames, not only empty ones.
    for (const char *kind : {"fk", "two-bone", "blend", "twist", "ribbon",
                             "spline", "constraint"}) {
        std::printf("%s: %d solved comparisons\n", kind, solvedFrames[kind]);
        CHECK(solvedFrames[kind] > 0);
    }

    std::string resources = DefaultResourceDir();
    if (argc > 1 && resources.empty())
        resources = TfAbsPath(std::string(argv[1]) +
                              "/../plugin/rigExecSchema/resources");
    if (!resources.empty())
        PlugRegistry::GetInstance().RegisterPlugins(resources);
    if (argc > 1)
        TestAuthoredRigs(argv[1]);
    else {
        std::printf("no examples directory given\n");
        ++failures;
    }

    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
