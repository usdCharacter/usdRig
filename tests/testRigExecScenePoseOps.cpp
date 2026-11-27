// A solver or constraint run as rigExecOps' pose ops gives the bits
// RigExecRunSolver, RigExecRunConstraint and the native kernels give: per
// kind over its settings, through refusals and a moved setting's rebuild,
// and for the auto clavicle, harmonics and IK/FK match ops against their
// kernels.
#include "opRunners/poseOpRunner.h"
#include "rigExecScene/poseOps.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/types.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExecMath/autoClavicleKernel.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecMath/spaceMatchKernel.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecOps/constraint/harmonicsOps.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecMath/splineIk.h"
#include "rigExecScene/constraintProgram.h"
#include "rigExecSchema/schemaNames.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/transform.h"
#include "pxr/base/vt/types.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
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

/// Native against the op, one frame, on \p runner. With \p opError the
/// native solve refuses the frame (fails, or publishes the rests or an empty
/// aggregate); the op's compile or evaluation fails instead with an error
/// holding \p opError and publishes nothing.
void
CompareSolver(const std::string &label, const RigExecSolverRecord &s,
              const RigExecSolverInputs &input, RigExecPoseOpRunner &runner,
              const char *opError = nullptr)
{
    CHECK(RigExecSolverOpSupported(s));
    RigExecSolverWorkspace workspace;
    RigExecPointFrameArray want, got;
    std::string wantError, gotError;
    const bool native =
        RigExecReferenceSolve(s, input, &workspace, &want, &wantError);
    const bool op = RigExecRunSolverOp(s, input, runner, &got, &gotError);
    const bool same = opError ? !op && got.frames.empty() &&
                                    gotError.find(opError) != std::string::npos
                              : native && op && SameArray(want, got);
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
    // A control count the rests do not match: refused before any node is
    // built.
    RigExecPoseOpRunner runner;
    RigExecSolverInputs shortInputs = FkInputs(1.0);
    shortInputs.controls.pop_back();
    CompareSolver("fk cardinality", FkRecord(true, false, false, false),
                  shortInputs, runner, "FK control/rest cardinality mismatch");
    // Joint groups: two alternative branches under joint 0 and a disabled
    // group, across in-range, rounded, clamped and NaN selectors.
    for (const bool start : {false, true})
        for (const bool live : {false, true}) {
            RigExecSolverRecord grouped = FkRecord(start, false, false, live);
            grouped.jointGroupParents = {-1, 0, 0, 1};
            grouped.jointGroups = {{0, 2, true}, {2, 1, false}};
            grouped.jointGroupMembers = {1, 2, 3};
            RigExecPoseOpRunner grouping;
            for (const double selector :
                 {0.0, 1.0, 0.6, -3.0, 7.0, std::nan("")}) {
                RigExecSolverInputs in = FkInputs(0.5);
                in.jointGroupSelectors = {selector, 0.0};
                CompareSolver("fkGroups " + std::to_string(start) +
                                  std::to_string(live) + " " +
                                  std::to_string(selector),
                              grouped, in, grouping);
            }
        }
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
            s.frameCount = 3;
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
            // Mismatched sizes publish nothing; so does a pair of another
            // count than Build's, and a record of that count rebuilds.
            const RigExecPointFrameArray shorter = Aggregate(1.0, 2),
                                         longer = Aggregate(0.5, 5),
                                         longerB = Aggregate(0.25, 5);
            RigExecSolverInputs mismatch;
            mismatch.blendA = &a;
            mismatch.blendB = &shorter;
            mismatch.blendWeight = 0.5;
            CompareSolver("blend mismatch", s, mismatch, runner,
                          "frame blend needs one frame and rest per joint on "
                          "both inputs");
            mismatch.blendA = &longer;
            mismatch.blendB = &longerB;
            CompareSolver("blend off count", s, mismatch, runner,
                          "frame blend inputs differ from its Build count");
            RigExecSolverRecord longerRecord = s;
            longerRecord.frameCount = 5;
            CompareSolver("blend rebuilt", longerRecord, mismatch, runner);
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
            CompareSolver("twist", s, in, runner,
                          std::isfinite(turns)
                              ? nullptr
                              : "DistributeTwist: needs a finite twist");
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
        // Five samples, a record with another count (rebuilt), one sample,
        // which fails compile, and no curve, which publishes no frame.
        constexpr char kTooFew[] = "SampleRibbonFrames: needs two samples";
        for (const int count : {5, 3, 1}) {
            s.ribbonSamples = count;
            RigExecSolverInputs in;
            in.ribbonPoints = posed;
            CompareSolver("ribbon " + std::to_string(count), s, in, runner,
                          count < 2 ? kTooFew : nullptr);
        }
        s.ribbonSamples = 5;
        CompareSolver("ribbon no curve", s, RigExecSolverInputs(), runner);
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

/// A one-bone 2D skeleton: solved, out of the plane (the setup frames), with
/// a non-finite channel (an error of the evaluation), with a malformed order (a compile
/// error) and with a channel missing (refused before any node is built).
void
TestSkeleton2d()
{
    RigExecSolverRecord s;
    s.kind = RigExecSolverKind::Skeleton2d;
    s.bone2dSetup.resize(2);
    s.bone2dSetup[1].parent = 0;
    s.jointRests = {Frame(GfVec3d(0, 0, 0), GfVec3d(0, 0, 1), 0).points,
                    Frame(GfVec3d(2, 0, 0), GfVec3d(0, 0, 1), 30).points};
    s.bone2dOrder = {0, 1};
    RigExecSolverInputs in;
    in.bone2dChannels.resize(2);
    in.bone2dChannels[0].rz = 25;
    in.bone2dChannels[1].rz = -40;
    in.bone2dChannels[1].tx = 0.5;
    in.bone2dChannels[1].sx = 1.3;
    RigExecPoseOpRunner runner;
    CompareSolver("skeleton2d", s, in, runner);
    s.hasStart = true;
    s.startRest = RestAt(GfVec3d(0), 0);
    in.start = Frame(GfVec3d(1, 2, 0), GfVec3d(0, 0, 1), 15);
    CompareSolver("skeleton2d start", s, in, runner);
    RigExecSolverInputs nan = in;
    nan.bone2dChannels[1].rz = std::numeric_limits<double>::quiet_NaN();
    CompareSolver("skeleton2d nan", s, nan, runner,
                  "Skeleton2d: 2D skeleton solve failed (a non-finite "
                  "channel)");
    RigExecSolverRecord tilted = s;
    tilted.jointRests[1] = Frame(GfVec3d(2, 0, 0), GfVec3d(1, 0, 0), 30).points;
    CompareSolver("skeleton2d tilted", tilted, in, runner);
    RigExecSolverRecord malformed = s;
    malformed.bone2dOrder = {0, 7};
    CompareSolver("skeleton2d malformed", malformed, in, runner,
                  "is invalid: Skeleton2d: 2D skeleton solve order is "
                  "malformed");
    RigExecSolverInputs missing = in;
    missing.bone2dChannels.pop_back();
    CompareSolver("skeleton2d counts", s, missing, runner,
                  kRigExecSkeleton2dCountsDiffer);
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
    RigExecSolverRecord degenerate = fk;
    degenerate.degenerate = true;
    CHECK(!RigExecSolverOpSupported(degenerate));
    RigExecRigDescBuilder other;
    CHECK(!RigExecAddSolverOp(degenerate, RigExecSolverInputs(), &other, &node,
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
    {
        // A non-finite offset is invalid input: the op stops where the
        // scene kernel hands back the input flagged degenerate.
        RigExecConstraintResult got;
        std::string gotError;
        CHECK(!RigExecRunConstraintOp(r, ConstraintInputs(false), runner, &got,
                                      &gotError));
        CHECK(gotError.find("non-finite offset") != std::string::npos);
    }
    // A moved offset is a new input value on the same node.
    const RigExecEvaluator *built = runner.evaluator.get();
    r.position.offset = GfVec3d(1, 2, 3);
    CompareConstraint("position moved offset", r, ConstraintInputs(false),
                      runner);
    CHECK(runner.evaluator.get() == built);
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

/// An auto clavicle's shift on the op core against RigExecAutoClavicleShift
/// over the same frames and scalars, as RigExecRunAutoClavicle feeds it:
/// over its optional frames, the limb and a zero shift.
void
TestAutoClavicle()
{
    const auto matrix = [](const RigExecPointFrame &frame) {
        GfMatrix4d m(1.0);
        RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points, &m);
        return m;
    };
    std::array<GfMatrix4d, 13> m;
    m[0] = matrix(Frame(GfVec3d(1, 0, 0)));
    m[1] = matrix(Frame(GfVec3d(0.5, 0.2, 0)));
    m[2] = matrix(Frame(GfVec3d(0, 0.3, 0), GfVec3d(0, 1, 0), 10));
    m[3] = matrix(Frame(GfVec3d(0, 0.3, 0)));
    m[4] = matrix(Frame(GfVec3d(1, 0, 0), GfVec3d(0, 0, 1), 70));
    for (size_t k = 0; k < 3; ++k)
        m[5 + k] = matrix(Frame(GfVec3d(1.0 + 2.0 * k, 0, 0)));
    m[8] = matrix(Frame(GfVec3d(3, 3, 1), GfVec3d(0, 1, 0), 20));
    m[9] = matrix(Frame(GfVec3d(2, 0, -3)));
    for (size_t k = 0; k < 3; ++k)
        m[10 + k] = matrix(Frame(GfVec3d(1.0 + 2.1 * k, 0.1, 0)));
    RigExecPoseOpRunner runner;
    int shifted = 0;
    for (const bool ik : {false, true})
        for (const bool limb : {false, true})
            for (const double gain : {0.0, 1.0}) {
                RigExecAutoClavicleConstants constants;
                constants.gain = gain;
                constants.swings = {1, 0, 0, 0, 0.9238795, 0, 0.3826834, 0};
                constants.widths = {10, 0.8};
                constants.gains = {1, 0.5};
                constants.weights = {1, 0.2, 0.2, 1};
                RigExecAutoClavicleOpInputs in;
                const size_t count = limb ? 13 : ik ? 10 : 8;
                for (size_t i = 0; i < count; ++i)
                    in.frames[i] = &m[i];
                if (!ik)
                    in.frames[8] = in.frames[9] = nullptr;
                in.ikBlend = 1 - constants.ikValue;
                in.hasLimb = limb;
                if (limb) {
                    in.stretch = 1.1;
                    in.twistDegrees = 15;
                    in.upperOffset = 0.05;
                }

                RigExecAutoClavicleFrames frames;
                frames.targetPosed = m[0].data();
                frames.pivotPosed = m[1].data();
                frames.anchorPosed = m[2].data();
                frames.anchorDefault = m[3].data();
                frames.fkPosed = m[4].data();
                for (size_t i = 0; i < 3; ++i)
                    frames.fkDefault[i] = m[5 + i].data();
                if (ik) {
                    frames.ikTargetPosed = m[8].data();
                    frames.polePosed = m[9].data();
                }
                frames.ikBlend = in.ikBlend;
                frames.amount = in.amount;
                frames.hasLimb = limb;
                if (limb) {
                    frames.limb.stretch = in.stretch;
                    frames.limb.pin = in.pin;
                    frames.limb.upperScale = in.upperScale;
                    frames.limb.lowerScale = in.lowerScale;
                    frames.limb.softDistance = in.softDistance;
                    frames.limb.scaleCalibration = in.scaleCalibration;
                    frames.twistRadians =
                        in.twistDegrees * std::acos(-1.0) / 180;
                    frames.limbRestUpper = (m[11].ExtractTranslation() -
                                            m[10].ExtractTranslation())
                                               .GetLength() +
                                           in.upperOffset;
                    frames.limbRestLower = (m[12].ExtractTranslation() -
                                            m[11].ExtractTranslation())
                                               .GetLength() +
                                           in.lowerOffset;
                }
                double native[3];
                RigExecAutoClavicleShift(constants, frames, native);
                GfMatrix4d nativeShifted = m[0];
                nativeShifted.SetTranslateOnly(
                    nativeShifted.ExtractTranslation() +
                    GfVec3d(native[0], native[1], native[2]));

                double delta[3] = {0, 0, 0};
                GfMatrix4d opShifted = m[0];
                std::string error;
                const bool ran = RigExecRunAutoClavicleOp(
                    constants, in, runner, delta, &opShifted, &error);
                const bool moved =
                    native[0] != 0 || native[1] != 0 || native[2] != 0;
                const bool same =
                    ran && Bits(native, delta, sizeof(native)) &&
                    (!moved || Bits(nativeShifted.data(), opShifted.data(),
                                    16 * sizeof(double)));
                if (!same)
                    std::printf("auto clavicle %d%d%g differs (%s)\n", int(ik),
                                int(limb), gain, error.c_str());
                CHECK(same);
                shifted += moved ? 1 : 0;
            }
    std::printf("auto clavicle: %d shifted comparisons\n", shifted);
    CHECK(shifted > 0);
}

/// A harmonics constraint's frame: what the op published and the window
/// capture it took, or the evaluation's error. applied: the offset moved it.
struct HarmonicsRun {
    RigExecWindowPacketPtr capture;
    RigExecPointFrame frame;
    bool ran = false;
    bool applied = false;
    bool published = true;
    std::string error;
};

/// A harmonics constraint's tick over Gf with RigExecHarmonicsOffset: the
/// capture is the incoming origin in its space, taken before any gate, and
/// the offset rings from the usable tail of \p ring and this tick.
HarmonicsRun
NativeHarmonics(const RigExecHarmonicsOpConstants &constants,
                const RigExecHarmonicsOpInputs &in,
                const RigExecWindowRing &ring, bool captureOnly)
{
    HarmonicsRun run;
    run.ran = true;
    run.frame = in.incoming;
    const auto fail = [&](const char *why) {
        run.ran = false;
        run.error = std::string("Harmonics: ") + why;
        return run;
    };
    const bool usable = RigExecConstraintFrameUsable(in.incoming);
    GfMatrix4d space(1.0), toSpace(1.0);
    bool spaceUsable = true;
    if (in.space) {
        space = *in.space;
        const double det = space.GetDeterminant();
        spaceUsable = std::isfinite(det) && det != 0.0;
        if (spaceUsable)
            toSpace = space.GetInverse();
    }
    if (usable && spaceUsable) {
        const GfVec3d p = in.space ? toSpace.Transform(in.incoming.Origin())
                                   : in.incoming.Origin();
        if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
            auto packet = std::make_shared<RigExecWindowPacket>();
            packet->scalars = {p[0], p[1], p[2]};
            run.capture = std::move(packet);
        }
    }
    if (captureOnly || !in.enabled || in.weight <= 0.0)
        return run;
    bool finite = true;
    for (const GfVec3d &q : in.incoming.points)
        finite = finite && std::isfinite(q[0]) && std::isfinite(q[1]) &&
                 std::isfinite(q[2]);
    if ((in.incoming.flags & RigExecPointFrameValid) && !finite)
        return fail("its incoming frame is not finite");
    if (!run.capture)
        return run;
    size_t begin = ring.size();
    while (begin > 0 && ring[begin - 1] && ring[begin - 1]->scalars.size() == 3)
        --begin;
    std::vector<double> positions;
    for (size_t k = begin; k < ring.size(); ++k)
        positions.insert(positions.end(), ring[k]->scalars.begin(),
                         ring[k]->scalars.end());
    positions.insert(positions.end(), run.capture->scalars.begin(),
                     run.capture->scalars.end());
    RigExecHarmonicsParameters params;
    params.amplitude = in.amplitude;
    params.decay = in.decay;
    params.termination = in.termination;
    params.matchVelocity = constants.matchVelocity;
    for (size_t a = 0; a < 3; ++a)
        params.axisAmplitude[a] = in.axisAmplitude[a];
    if (!std::isfinite(params.amplitude) || !std::isfinite(params.decay) ||
        !std::isfinite(params.termination) ||
        !std::isfinite(in.axisAmplitude[0]) ||
        !std::isfinite(in.axisAmplitude[1]) ||
        !std::isfinite(in.axisAmplitude[2]))
        return fail("a parameter is not finite");
    double ring3[3];
    if (!RigExecHarmonicsOffset(
            constants.settings, params,
            reinterpret_cast<const double(*)[3]>(positions.data()),
            positions.size() / 3, ring3))
        return run;
    GfVec3d offset(ring3[0], ring3[1], ring3[2]);
    if (in.space)
        offset = space.TransformDir(offset);
    offset *= in.weight;
    RigExecPointFrame candidate = in.incoming;
    for (auto &q : candidate.points)
        q += offset;
    if (!RigExecConstraintFrameUsable(candidate))
        return fail("its result is not a usable frame");
    run.frame = candidate;
    run.applied = true;
    return run;
}

HarmonicsRun
OpHarmonics(const RigExecHarmonicsOpConstants &constants,
            const RigExecHarmonicsOpInputs &in, const RigExecWindowRing *ring,
            bool captureOnly, RigExecPoseOpRunner &runner)
{
    HarmonicsRun run;
    run.ran = RigExecRunHarmonicsOp(constants, in, ring, captureOnly, runner,
                                    &run.capture, &run.frame, &run.error);
    RigExecPointFrame published;
    run.published = runner.evaluator &&
                    RigExecReadHarmonicsOpOutput(runner.harmonics,
                                                 *runner.evaluator, &published);
    return run;
}

bool
SameHarmonics(const HarmonicsRun &a, const HarmonicsRun &b)
{
    if (!a.ran || !b.ran)
        return a.ran == b.ran && !b.published &&
               b.error.find(a.error) != std::string::npos;
    return SameFrame(a.frame, b.frame) &&
           RigExecWindowPacketsEqual(a.capture, b.capture);
}

/// Frame by frame over a ring the native captures fill, on one runner that
/// rebuilds when a setting moves: with and without a space (a singular and
/// an unreadable one), on capture-only ticks, disabled, at zero and partial
/// weights. A non-finite amplitude stops the evaluation with an error and
/// publishes nothing, and so does a space the settings do not expect, at
/// compile; an unusable incoming frame and a space that measures nothing
/// leave the frame as it entered.
void
TestHarmonics()
{
    struct Case {
        const char *label;
        int space = 0; // 0 none, 1 usable, 2 singular, 3 unreadable
        bool enabled = true;
        double weight = 0.8;
        double amplitude = 1.3;
        bool ignoreFirst = true;
        bool unusableAt3 = false;
    };
    const Case cases[] = {
        {"plain"},
        {"space", 1},
        {"singular space", 2},
        {"unreadable space", 3},
        {"disabled", 0, false},
        {"zero weight", 0, true, 0.0},
        {"partial weight", 1, true, 0.6},
        {"non-finite amplitude", 0, true, 0.8,
         std::numeric_limits<double>::quiet_NaN()},
        {"from rest", 1, true, 1.0, 0.7, false},
        {"unusable frame", 0, true, 0.8, 1.3, true, true},
    };
    RigExecPoseOpRunner runner;
    int applied = 0, errors = 0;
    for (const Case &c : cases) {
        RigExecHarmonicsOpConstants constants;
        constants.settings.waves = 2.0;
        constants.settings.waveLength = 3.0;
        constants.settings.frequency = 1.0;
        constants.settings.ignoreFirstFrame = c.ignoreFirst;
        constants.frames =
            uint32_t(RigExecHarmonicsWindowFrames(constants.settings));
        RigExecHarmonicsOpInputs in;
        GfMatrix4d space(1.0);
        space.SetRotate(GfRotation(GfVec3d(0.3, 1.0, 0.2), 35.0));
        space.SetTranslateOnly(GfVec3d(0.5, -2.0, 1.25));
        if (c.space == 2)
            space = GfMatrix4d(1.0).SetScale(GfVec3d(1.0, 0.0, 1.0));
        if (c.space == 3)
            space = GfMatrix4d(0.0);
        if (c.space)
            in.space = space;
        in.enabled = c.enabled;
        in.weight = c.weight;
        in.amplitude = c.amplitude;
        in.decay = 2.5;
        in.termination = 0.1;
        in.axisAmplitude = GfVec3d(1.0, 0.5, 2.0);

        RigExecWindowRing ring;
        for (int t = 0; t < 9; ++t) {
            const double s = double(t);
            in.incoming =
                Frame(GfVec3d(0.3 * s * s, std::sin(0.7 * s), -0.4 * s),
                      GfVec3d(0, 1, 0), 9.0 * s);
            if (c.unusableAt3 && t == 3)
                in.incoming.flags = 0;
            for (const bool captureOnly : {false, true}) {
                const HarmonicsRun native =
                    NativeHarmonics(constants, in, ring, captureOnly);
                const HarmonicsRun op =
                    OpHarmonics(constants, in, &ring, captureOnly, runner);
                const bool same = SameHarmonics(native, op);
                if (!same)
                    std::printf("harmonics %s at %d%s: native %d (%s), op "
                                "%d (%s)\n",
                                c.label, t, captureOnly ? " captured only" : "",
                                int(native.ran), native.error.c_str(),
                                int(op.ran), op.error.c_str());
                CHECK(same);
                if (!captureOnly && native.applied)
                    ++applied;
                if (!native.ran)
                    ++errors;
                if (!captureOnly) {
                    ring.push_back(native.capture);
                    if (ring.size() > constants.frames)
                        ring.erase(ring.begin());
                }
            }
        }
        // Without a window: no history, nothing rings.
        const bool same =
            SameHarmonics(NativeHarmonics(constants, in, {}, false),
                          OpHarmonics(constants, in, nullptr, false, runner));
        if (!same)
            std::printf("harmonics %s without a window differs\n", c.label);
        CHECK(same);
    }
    std::printf("harmonics: %d applied comparisons, %d errors\n", applied,
                errors);
    CHECK(applied > 0);
    CHECK(errors > 0);

    // A space fed where the settings measure in none fails compile.
    rigExecOps::HarmonicsPayload payload;
    payload.frames = 4;
    RigExecRigDescBuilder builder;
    const RigExecNodeId id = builder.AddOp(rigExecOps::Harmonics(), &payload);
    for (const auto *port :
         {&rigExecOps::HarmonicsIn(), &rigExecOps::HarmonicsSpace(),
          &rigExecOps::HarmonicsEnabled(), &rigExecOps::HarmonicsWeight(),
          &rigExecOps::HarmonicsAmplitude(), &rigExecOps::HarmonicsDecay(),
          &rigExecOps::HarmonicsTermination(),
          &rigExecOps::HarmonicsAxisAmplitude()})
        builder.BindInput(id, *port);
    builder.BindOutput(id, rigExecOps::HarmonicsOut());
    std::string error;
    const auto desc = builder.Build(&error);
    CHECK(desc);
    CHECK(desc && !RigExecCompile(*desc, {}, &error));
    CHECK(error.find("Harmonics: its space is fed exactly when its settings "
                     "measure in one") != std::string::npos);
}

/// The baked space match (bakedSpaces.cpp): per space, the channels that
/// keep the control on \p target, over the native space switch.
RigExecSpaceMatchOutcome
NativeSpaceMatch(const RigExecSpaceSwitchRecord &record,
                 const RigExecSpaceSwitchInputs &input,
                 const GfMatrix4d &target,
                 const RigExecIkFkMatchChannels &current,
                 std::vector<double> *values)
{
    RigExecSpaceSwitchInputs trial = input;
    const auto compose = [&](const double *avars, double active,
                             GfMatrix4d *composed) {
        std::copy(avars, avars + 16, trial.avars.data());
        trial.active = active;
        return RigExecRunSpaceSwitchMatrix(record, trial, composed);
    };
    values->resize(input.sources.size() * RigExecSpaceMatchStride);
    return RigExecSpaceMatchT<GfMatrix3d, GfVec3d>(
        target, current, input.sources.size(), compose, values->data());
}

rigExecStd::Mat4d
StdMatrix(const GfMatrix4d &m)
{
    rigExecStd::Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out._mtx[r][c] = m[r][c];
    return out;
}

GfMatrix4d
Placement(const GfVec3d &axis, double degrees, const GfVec3d &at,
          double scale = 1.0)
{
    GfMatrix4d s(1.0), r(1.0), t(1.0);
    s.SetScale(scale);
    r.SetRotate(GfRotation(axis, degrees));
    t.SetTranslate(at);
    return s * r * t;
}

/// GfTransform's decomposition and recomposition, GfLerp and GfSlerp: the
/// blend, filter and mask the native space switch ran before the shared
/// kernel (rigExecMath/switchMathKernel.h).
namespace gfReference {

struct Parts {
    GfVec3d translation, scale;
    GfQuatd rotation;
};

Parts
Decompose(const GfMatrix4d &m)
{
    const GfTransform transform(m);
    return {transform.GetTranslation(), transform.GetScale(),
            transform.GetRotation().GetQuat()};
}

GfMatrix4d
Recompose(const Parts &d)
{
    GfTransform transform;
    transform.SetScale(d.scale);
    transform.SetRotation(GfRotation(d.rotation));
    transform.SetTranslation(d.translation);
    return transform.GetMatrix();
}

GfMatrix4d
Blend(const GfMatrix4d &a, const GfMatrix4d &b, double weight)
{
    if (!(weight > 0.0))
        return a;
    if (weight >= 1.0)
        return b;
    const Parts da = Decompose(a), db = Decompose(b);
    Parts out;
    out.translation = GfLerp(weight, da.translation, db.translation);
    out.scale = GfLerp(weight, da.scale, db.scale);
    GfQuatd target = db.rotation;
    if (GfDot(da.rotation, target) < 0.0)
        target = GfQuatd(-target.GetReal(), -target.GetImaginary());
    out.rotation = GfSlerp(weight, da.rotation, target).GetNormalized();
    return Recompose(out);
}

GfMatrix4d
Filter(const GfMatrix4d &m, const GfVec3d &axis, int filter)
{
    if (filter != 1 && filter != 2)
        return m;
    const double length = axis.GetLength();
    if (length < 1e-12)
        return m;
    const GfVec3d a = axis / length;
    Parts d = Decompose(m);
    const GfQuatd q = d.rotation.GetNormalized();
    const double along = GfDot(q.GetImaginary(), a);
    GfQuatd twist(q.GetReal(), along * a);
    const double norm = std::sqrt(twist.GetReal() * twist.GetReal() +
                                  twist.GetImaginary().GetLengthSq());
    twist = norm < 1e-12
                ? GfQuatd::GetIdentity()
                : GfQuatd(twist.GetReal() / norm, twist.GetImaginary() / norm);
    d.rotation = filter == 1 ? twist : (q * twist.GetInverse()).GetNormalized();
    return Recompose(d);
}

GfMatrix4d
Mask(const GfMatrix4d &m, const bool translation[3], const bool rotation[3],
     const bool scale[3])
{
    Parts d = Decompose(m);
    for (int axis = 0; axis < 3; ++axis) {
        if (!translation[axis])
            d.translation[axis] = 0.0;
        if (!scale[axis])
            d.scale[axis] = 1.0;
    }
    if (!rotation[0] || !rotation[1] || !rotation[2]) {
        const GfMatrix4d r(GfRotation(d.rotation), GfVec3d(0));
        double x = 0.0, y = 0.0, z = 0.0;
        const double sinY = GfClamp(r[0][2], -1.0, 1.0);
        y = std::asin(sinY);
        if (std::abs(sinY) < 1.0 - 1e-9) {
            x = std::atan2(r[1][2], r[2][2]);
            z = std::atan2(r[0][1], r[0][0]);
        } else {
            x = std::atan2(-r[2][1], r[1][1]);
            z = 0.0;
        }
        const GfVec3d axes[3] = {GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
                                 GfVec3d(0, 0, 1)};
        const double angles[3] = {GfRadiansToDegrees(x), GfRadiansToDegrees(y),
                                  GfRadiansToDegrees(z)};
        GfMatrix4d composed(1.0);
        for (int axis = 0; axis < 3; ++axis) {
            if (!rotation[axis] || angles[axis] == 0.0)
                continue;
            composed =
                composed *
                GfMatrix4d(GfRotation(axes[axis], angles[axis]), GfVec3d(0));
        }
        d.rotation = composed.ExtractRotationQuat();
    }
    return Recompose(d);
}

} // namespace gfReference

/// The shared switch kernel over Gf gives GfTransform's bits, and over the
/// runtime's mirror types the same bits: sheared, reflected, scaled,
/// signed-zero and degenerate matrices, every filter and mask.
void
TestSwitchMath()
{
    std::vector<GfMatrix4d> matrices = {
        GfMatrix4d(1.0), Placement(GfVec3d(0, 0, 1), 0.0, GfVec3d(0)),
        Placement(GfVec3d(1, 0, 0), 180.0, GfVec3d(-0.0, 0.0, -0.0)),
        Placement(GfVec3d(0.3, 1, 0.2), 47.0, GfVec3d(1, -2, 3), 0.7),
        Placement(GfVec3d(1, 1, 0), 89.9999999, GfVec3d(0.5), 2.0)};
    GfMatrix4d sheared(1.0);
    sheared[1][0] = 0.4;
    sheared[2][1] = -0.25;
    sheared[3][2] = 1.5;
    matrices.push_back(sheared * matrices[3]);
    GfMatrix4d reflected(1.0);
    reflected[0][0] = -1.0;
    matrices.push_back(reflected * matrices[3]);
    GfMatrix4d flat = matrices[3];
    flat[2][0] = flat[2][1] = flat[2][2] = 0.0;
    matrices.push_back(flat);
    GfMatrix4d negativeZero(1.0);
    negativeZero[0][1] = negativeZero[1][0] = -0.0;
    negativeZero[3][0] = -0.0;
    matrices.push_back(negativeZero);
    uint64_t state = 0x9e3779b97f4a7c15ull;
    const auto next = [&state] {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return double(state >> 11) / double(1ull << 53) * 2.0 - 1.0;
    };
    for (int k = 0; k < 24; ++k)
        matrices.push_back(Placement(
            GfVec3d(next(), next(), next()), 180.0 * next(),
            GfVec3d(next(), next(), next()) * 3.0, 1.0 + 0.5 * next()));
    const auto same = [](const GfMatrix4d &a, const rigExecStd::Mat4d &b) {
        return Bits(a.data(), &b._mtx[0][0], sizeof(double) * 16);
    };
    const auto bits = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        return Bits(a.data(), b.data(), sizeof(double) * 16);
    };
    const auto rrSlerp = [](double t, const RrQuatd &a, const RrQuatd &b) {
        return RrSlerp(t, a, b);
    };
    const GfVec3d axis(0.2, 1.0, -0.1);
    const RrVec3d rrAxis(0.2, 1.0, -0.1);
    int compared = 0;
    for (size_t i = 0; i < matrices.size(); ++i) {
        const GfMatrix4d &a = matrices[i];
        const GfMatrix4d &b = matrices[(i * 7 + 3) % matrices.size()];
        const rigExecStd::Mat4d ra = StdMatrix(a), rb = StdMatrix(b);
        for (const double w : {0.0, 0.25, 0.5, 0.875, 1.0}) {
            const GfMatrix4d native = RigExecBlendTransforms(a, b, w);
            CHECK(bits(native, gfReference::Blend(a, b, w)));
            CHECK(same(native, RigExecBlendTransformsT(ra, rb, w, rrSlerp)));
            ++compared;
        }
        for (const int filter : {0, 1, 2, 3}) {
            const GfMatrix4d native = RigExecFilterSpaceRotation(
                a, axis, RigExecRotationFilter(filter));
            CHECK(bits(native, gfReference::Filter(a, axis, filter)));
            CHECK(
                same(native, RigExecFilterSpaceRotationT(ra, rrAxis, filter)));
            ++compared;
        }
        for (int mask = 0; mask < 512; mask += 37) {
            bool t[3], r[3], s[3];
            for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
                t[axisIndex] = mask & (1 << axisIndex);
                r[axisIndex] = mask & (1 << (3 + axisIndex));
                s[axisIndex] = mask & (1 << (6 + axisIndex));
            }
            const GfMatrix4d native = RigExecMaskTransform(a, t, r, s);
            CHECK(bits(native, gfReference::Mask(a, t, r, s)));
            CHECK(same(native, RigExecMaskTransformT(ra, t, r, s)));
            ++compared;
        }
        GfMatrix4d roundTrip(1.0);
        RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                              RigExecFrameFromMatrix(a).points, &roundTrip);
        CHECK(bits(RigExecRoundTripT<GfMatrix3d, GfVec3d>(a), roundTrip));
        CHECK(same(roundTrip, RigExecRoundTripT<RrMat3d, RrVec3d>(ra)));
    }
    std::printf("switch math: %d comparisons\n", compared);
}

/// The values a landmark is poisoned with: a NaN and both infinities.
const double kNonFinite[3] = {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()};

/// A non-finite landmark, NaN or either infinity in each landmark position
/// and axis: the shared kernel keeps the first non-finite landmark and
/// leaves the later ones at identity, and the one usability check
/// (RigExecFrameUsableT) refuses the frame however it is flagged, over Gf
/// and over the runtime's types.
void
TestNonFiniteLandmarks()
{
    GfMatrix4d m(1.0);
    m[3][1] = std::numeric_limits<double>::infinity();
    std::array<GfVec3d, 4> points;
    CHECK(!RigExecMatrixLandmarksT(m, &points));
    CHECK(std::isinf(points[0][1]));
    CHECK(points[1] == GfVec3d(1, 0, 0) && points[3] == GfVec3d(0, 0, 1));
    const RigExecPointFrame frame = RigExecFrameFromMatrix(m);
    CHECK(frame.IsDegenerate() && !RigExecConstraintFrameUsable(frame));
    int refused = 0;
    const GfMatrix4d placed =
        Placement(GfVec3d(0.3, 1, 0.2), 47.0, GfVec3d(1, -2, 3), 0.7);
    for (const double poison : kNonFinite) {
        for (size_t landmark = 0; landmark < 4; ++landmark) {
            for (int axis = 0; axis < 3; ++axis) {
                // Row 3 is landmark 0's and row a < 3 landmark a + 1's;
                // either reaches landmark 0 (0 * x is not finite either),
                // where the landmarks stop.
                GfMatrix4d poisoned = placed;
                poisoned[landmark ? int(landmark) - 1 : 3][axis] = poison;
                CHECK(!RigExecMatrixLandmarksT(poisoned, &points));
                CHECK(!std::isfinite(points[0][axis]));
                CHECK(!RigExecConstraintFrameUsable(
                    RigExecFrameFromMatrix(poisoned)));
                GfMatrix4d read;
                rigExecStd::Mat4d rrRead;
                CHECK((RigExecReadLandmarkMatrixT<GfMatrix3d, GfVec3d>(
                          poisoned, &read)) == RigExecLandmarkRead::NotUsable);
                CHECK((RigExecReadLandmarkMatrixT<RrMat3d, RrVec3d>(
                          StdMatrix(poisoned), &rrRead)) ==
                      RigExecLandmarkRead::NotUsable);
                RigExecPointFrame flagged = RigExecFrameFromMatrix(placed);
                CHECK(RigExecConstraintFrameUsable(flagged));
                flagged.points[landmark][axis] = poison;
                CHECK(flagged.IsValid() && !flagged.IsDegenerate() &&
                      !RigExecConstraintFrameUsable(flagged));
                std::array<RrVec3d, 4> rrPoints;
                for (size_t p = 0; p < 4; ++p)
                    rrPoints[p] =
                        RrVec3d(flagged.points[p][0], flagged.points[p][1],
                                flagged.points[p][2]);
                CHECK(!RigExecFrameUsableT(rrPoints, true, false));
                ++refused;
            }
        }
    }
    std::printf("non-finite landmarks: %d refused\n", refused);
}

/// The SpaceMatch op against the baked space match, bit for bit, over
/// rotation filters, axis masks, a carry, a world source and channels in
/// several orders, signs, spins and scales. Past the ordinary variants a
/// space's posed frame, or the target, holds a NaN or an infinity in one
/// landmark position: the baked match refuses it (NotFinite) and the op's
/// evaluation stops with the same diagnostic, publishing nothing.
void
TestSpaceMatch()
{
    using namespace rigExecOps;
    int solved = 0, refused = 0;
    constexpr int kOrdinary = 6;
    constexpr int kPoisoned = 3 * 4 * 2;
    for (int variant = 0; variant < kOrdinary + kPoisoned; ++variant) {
        const int poisonIndex = variant - kOrdinary;
        const double poison =
            poisonIndex >= 0 ? kNonFinite[poisonIndex / 8] : 0.0;
        const int poisonRow = poisonIndex >= 0 ? (poisonIndex / 2) % 4 : 0;
        const bool poisonTarget = poisonIndex >= 0 && poisonIndex % 2 == 1;
        RigExecSpaceSwitchRecord record;
        if (variant == 1)
            record.filters = {1, 2, 0};
        if (variant == 2)
            record.filters = {3, 0, 2};
        record.twistAxis = GfVec3d(0.2, 1.0, 0.1);
        if (variant == 3) {
            record.affectTranslation = {{true, false, true}};
            record.affectRotation = {{false, true, true}};
        }
        if (variant == 4)
            record.affectScale = {{true, true, false}};
        RigExecSpaceSwitchInputs input;
        input.posedDefault = Placement(GfVec3d(0, 0, 1), 10, GfVec3d(1, 2, 0));
        input.parentDefault = Placement(GfVec3d(1, 0, 0), 5, GfVec3d(0, 1, 0));
        input.parentDefaultInverse = input.parentDefault.GetInverse();
        input.parentPosed =
            Placement(GfVec3d(0.3, 1, 0), 25, GfVec3d(0.5, 1.5, -0.25));
        input.hasCarry = variant >= 3;
        input.spaceDefault = Placement(GfVec3d(0, 1, 0), 0, GfVec3d(0));
        input.spacePosed =
            Placement(GfVec3d(0, 1, 0), 30, GfVec3d(0.1, 0, 0.2), 1.1);
        input.sources.resize(3);
        input.sources[0].world = variant == 5;
        for (size_t k = 0; k < 3; ++k) {
            input.sources[k].defaultSpace = Placement(
                GfVec3d(0, 0, 1), 7.0 * double(k), GfVec3d(double(k), 0, 0));
            input.sources[k].posedSpace = Placement(
                GfVec3d(1, 0.4, 0.2), 20.0 + 11.0 * double(k),
                GfVec3d(double(k), 0.5 * double(k), -1.0), 1.0 + 0.1 * k);
        }
        if (poisonIndex >= 0 && !poisonTarget)
            input.sources[2].posedSpace[poisonRow][poisonRow % 3] = poison;
        RigExecIkFkMatchChannels channels;
        const char *orders[3] = {"XYZ", "ZXY", "YZX"};
        std::copy(orders[variant % 3], orders[variant % 3] + 3, channels.order);
        for (int k = 0; k < 3; ++k) {
            channels.t[k] = 0.3 * (k + 1) - 0.2 * variant;
            channels.r[k] = 12.0 * (k + 1) + 3.0 * variant;
            channels.s[k] = variant == 2 ? 1.2 : 1.0;
        }
        channels.sign[1] = variant % 2 ? -1.0 : 1.0;
        channels.spin = variant == 4 ? 15.0 : 0.0;
        // The target is the frame the switch composes at the second space.
        RigExecSpaceSwitchInputs at = input;
        const auto composed = ikFkMatchKernel::Compose(channels);
        std::copy(composed.begin(), composed.end(), at.avars.data());
        at.active = 1.0;
        RigExecPointFrame frame;
        GfMatrix4d target(1.0);
        CHECK(RigExecRunSpaceSwitch(record, at, &frame) &&
              RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                                    &target));
        if (poisonTarget)
            target[poisonRow][poisonRow % 3] = poison;
        std::vector<double> want;
        const RigExecSpaceMatchOutcome outcome =
            NativeSpaceMatch(record, input, target, channels, &want);
        const bool native = outcome == RigExecSpaceMatchOutcome::Matched;
        CHECK((poisonIndex >= 0) ==
              (outcome == RigExecSpaceMatchOutcome::NotFinite));
        if (outcome == RigExecSpaceMatchOutcome::NotFinite)
            ++refused;

        SpaceMatchPayload payload;
        payload.space.filters = record.filters;
        payload.space.twistAxis = rigExecStd::Vec3d(
            record.twistAxis[0], record.twistAxis[1], record.twistAxis[2]);
        payload.space.affectTranslation = record.affectTranslation;
        payload.space.affectRotation = record.affectRotation;
        payload.space.affectScale = record.affectScale;
        payload.space.hasCarry = input.hasCarry;
        for (const auto &source : input.sources)
            payload.space.world.push_back(source.world ? 1 : 0);
        RigExecRigDescBuilder builder;
        const RigExecNodeId id = builder.AddOp(SpaceMatch(), &payload);
        const RigExecInputId targetIn =
            builder.BindInput(id, SpaceMatchTarget());
        const RigExecInputId channelsIn =
            builder.BindInput(id, SpaceMatchChannels());
        const RigExecInputId posedDefault =
            builder.BindInput(id, SpaceMatchPosedDefault());
        const RigExecInputId parentDefaultInverse =
            builder.BindInput(id, SpaceMatchParentDefaultInverse());
        const RigExecInputId parentPosed =
            builder.BindInput(id, SpaceMatchParentPosed());
        const RigExecInputId parentDefault =
            builder.BindInput(id, SpaceMatchParentDefault());
        RigExecInputId spaceDefault, spacePosed;
        if (input.hasCarry) {
            spaceDefault = builder.BindInput(id, SpaceMatchSpaceDefault());
            spacePosed = builder.BindInput(id, SpaceMatchSpacePosed());
        }
        const RigExecInputId sourceDefaults =
            builder.BindInput(id, SpaceMatchSourceDefaults());
        const RigExecInputId sourcePosed =
            builder.BindInput(id, SpaceMatchSourcePosed());
        const RigExecOutputId values =
            builder.BindOutput(id, SpaceMatchValues());
        std::string error;
        const auto desc = builder.Build(&error);
        const auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
        auto evaluator =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
        CHECK(evaluator);
        if (!evaluator) {
            std::printf("space match: %s\n", error.c_str());
            continue;
        }
        const auto set = [&](const RigExecInputId &in, const auto &value) {
            CHECK(evaluator->SetInput(in, &value));
        };
        set(targetIn, StdMatrix(target));
        rigExecStd::RigExecArray<double> packed(kMatchChannelStride);
        for (int k = 0; k < 3; ++k) {
            packed[size_t(k)] = channels.t[k];
            packed[size_t(3 + k)] = channels.s[k];
            packed[size_t(6 + k)] = channels.r[k];
            packed[size_t(10 + k)] = channels.sign[k];
            packed[size_t(15 + k)] = double(channels.order[k]);
        }
        packed[9] = channels.spin;
        packed[13] = channels.units;
        packed[14] = 0.0;
        set(channelsIn, packed);
        set(posedDefault, StdMatrix(input.posedDefault));
        set(parentDefaultInverse, StdMatrix(input.parentDefaultInverse));
        set(parentPosed, StdMatrix(input.parentPosed));
        set(parentDefault, StdMatrix(input.parentDefault));
        if (input.hasCarry) {
            set(spaceDefault, StdMatrix(input.spaceDefault));
            set(spacePosed, StdMatrix(input.spacePosed));
        }
        rigExecStd::RigExecArray<rigExecStd::Mat4d> defaults(3), posed(3);
        for (size_t k = 0; k < 3; ++k) {
            defaults[k] = StdMatrix(input.sources[k].defaultSpace);
            posed[k] = StdMatrix(input.sources[k].posedSpace);
        }
        set(sourceDefaults, defaults);
        set(sourcePosed, posed);
        const bool ran = evaluator->Evaluate(&error);
        const auto *got = static_cast<const rigExecStd::RigExecArray<double> *>(
            evaluator->GetOutput(values));
        const bool same = native
                              ? ran && got && got->Size() == want.size() &&
                                    Bits(got->Data(), want.data(),
                                         want.size() * sizeof(double))
                              : !ran && !got &&
                                    error.find(std::string("SpaceMatch:") +
                                               kRigExecSpaceMatchNotFinite) !=
                                        std::string::npos;
        if (!same)
            std::printf("space match variant %d differs\n", variant);
        CHECK(same);
        if (native)
            ++solved;
    }
    std::printf("space match: %d solved, %d refused comparisons\n", solved,
                refused);
    CHECK(solved == kOrdinary);
    CHECK(refused == kPoisoned);
}

/// RigExecIkFkMatchConstraint's arithmetic over Gf with the ikFkMatchKernel
/// functions, as the evaluator runs it, from the op's inputs. \p matched:
/// both halves matched.
RigExecIkFkMatchOpOutputs
NativeIkFkMatch(const RigExecIkFkMatchOpConstants &m,
                const RigExecIkFkMatchOpInputs &in, bool *matched)
{
    *matched = false;
    using rigExecOps::IkFkPoleMode;
    RigExecIkFkMatchOpOutputs out;
    const GfMatrix4d *input = in.sources.data();
    const GfMatrix4d *rest = in.rests.data();
    const RigExecIkFkMatchChannels *channels = in.channels.data();
    const GfMatrix4d *control = input + 5;
    RigExecLimbStretch limb;
    limb.stretch = in.scalars[1];
    limb.softDistance = in.scalars[2];
    limb.upperScale = in.scalars[3];
    limb.lowerScale = in.scalars[4];
    limb.pin = in.scalars[5];
    limb.scaleCalibration = m.scaleCalibration;
    double space[16];
    const double *spaceMatrix = nullptr;
    if (in.hasSpace) {
        const GfMatrix4d carry = in.spaceRest.GetInverse() * in.spacePosed;
        std::copy(carry.data(), carry.data() + 16, space);
        spaceMatrix = space;
    }
    double restOrigins[3][3], restUpper = 0, restLower = 0, spaceFactor = 1;
    for (size_t i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            restOrigins[i][k] = rest[i][3][k];
    RigExecIkFkMatchRestLengths(restOrigins, spaceMatrix, m.upperLengthOffset,
                                m.lowerLengthOffset, &restUpper, &restLower,
                                &spaceFactor);
    limb.softDistance *= spaceFactor;
    RigExecIkFkMatchIkInputs ikIn;
    for (size_t i = 0; i < 3; ++i)
        ikIn.joints[i] = input[i].data();
    ikIn.root = input[3].data();
    ikIn.effectorPosed = input[4].data();
    ikIn.ikPosed = control[0].data();
    ikIn.ik = channels[0];
    ikIn.twistRadians = in.scalars[6] * std::acos(-1.0) / 180.0;
    ikIn.poleDistance =
        (rest[4].ExtractTranslation() - rest[1].ExtractTranslation())
            .GetLength() *
        spaceFactor;
    ikIn.polePosed = control[1].data();
    ikIn.hasStretch = m.stretchLimb;
    ikIn.limb = limb;
    ikIn.restUpper = restUpper;
    ikIn.restLower = restLower;
    RigExecIkFkMatchIkResult ik;
    RigExecIkFkMatchIk(ikIn, &ik);
    RigExecIkFkMatchChannels pole = channels[1];
    double poleFrame[16];
    bool poleOk = false;
    if (ik.ok) {
        GfMatrix4d ikNew;
        std::copy(ik.ikFrame, ik.ikFrame + 16, ikNew.data());
        const GfMatrix4d moved = control[0].GetInverse() * ikNew;
        const auto composed = ikFkMatchKernel::Compose(channels[1]);
        GfMatrix4d avars;
        std::copy(composed.begin(), composed.end(), avars.data());
        GfMatrix4d parent = avars.GetInverse() * control[1];
        if (in.poleMode == uint32_t(IkFkPoleMode::Switch)) {
            RigExecSpaceSwitchInputs inputs = in.poleSwitch;
            inputs.avars = GfMatrix4d(1.0);
            for (size_t k = 0;
                 k < inputs.sources.size() && k < m.poleSourceRides.size(); ++k)
                if (m.poleSourceRides[k] && !inputs.sources[k].world)
                    inputs.sources[k].posedSpace =
                        inputs.sources[k].posedSpace * moved;
            RigExecPointFrame frame;
            GfMatrix4d switched(1.0);
            if (RigExecRunSpaceSwitch(m.poleRecord, inputs, &frame) &&
                RigExecConstraintFrameUsable(frame) &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                                      &switched))
                parent = switched;
        } else if (in.poleMode == uint32_t(IkFkPoleMode::Ride)) {
            parent = parent * moved;
        }
        poleOk = RigExecIkFkMatchPole(ik.poleTarget, parent.data(), &pole,
                                      poleFrame);
    }
    RigExecIkFkMatchFkInputs fkIn;
    GfMatrix4d offsets[3];
    for (size_t i = 0; i < 3; ++i) {
        fkIn.joints[i] = input[i].data();
        fkIn.fkPosed[i] = control[2 + i].data();
        fkIn.fk[i] = channels[2 + i];
        offsets[i] = rest[5 + i] * rest[i].GetInverse();
        fkIn.offsets[i] = offsets[i].data();
    }
    fkIn.hasStretch = m.stretchLimb;
    fkIn.upperScale = limb.upperScale;
    fkIn.lowerScale = limb.lowerScale;
    fkIn.restUpper = restUpper;
    fkIn.restLower = restLower;
    RigExecIkFkMatchFkResult fk;
    RigExecIkFkMatchFk(fkIn, &fk);
    if (ik.ok && poleOk && fk.ok) {
        RigExecIkFkMatchPack(ik, pole, fk, out.ikValues.data(),
                             out.fkValues.data());
        *matched = true;
    }
    const auto show = [&](size_t k, const double *frame) {
        std::copy(frame, frame + 16, out.shown[k].data());
        out.shownMask |= 1u << k;
    };
    switch (RigExecIkFkMatchShownSide(in.scalars[0], m.ikValue, in.weight)) {
    case RigExecIkFkMatchShown::IkSide:
        if (ik.ok && poleOk) {
            show(0, ik.ikFrame);
            show(1, poleFrame);
        }
        break;
    case RigExecIkFkMatchShown::FkSide:
        if (fk.ok)
            for (size_t i = 0; i < 3; ++i)
                show(2 + i, fk.frames[i]);
        break;
    case RigExecIkFkMatchShown::None:
        break;
    }
    return out;
}

/// An IK/FK match on the op core against NativeIkFkMatch on a two-bone arm:
/// the published values and the presented frames bit for bit, on either
/// side and between, with the pole kept, riding the IK control and switched,
/// with and without a stretching limb and a space. Where a half cannot be
/// matched, a non-finite IK control channel among them, both report no match
/// and present the same frames.
void
TestIkFkMatch()
{
    using rigExecOps::IkFkPoleMode;
    const GfVec3d up(0, 0, 1);
    const GfMatrix4d shoulder = Placement(up, -10, GfVec3d(0, 0, 0));
    const GfMatrix4d elbow = Placement(up, 15, GfVec3d(0.98, -0.17, 0));
    const GfMatrix4d wrist = Placement(up, 5, GfVec3d(2.0, 0.1, 0));
    RigExecIkFkMatchOpInputs base;
    base.sources = {shoulder,
                    elbow,
                    wrist,
                    shoulder,
                    wrist,
                    Placement(GfVec3d(0.2, 1, 0), 25, GfVec3d(1.8, 0.4, 0.1)),
                    Placement(up, 0, GfVec3d(1.1, -1.4, 0.3)),
                    shoulder,
                    elbow,
                    wrist};
    base.rests = {Placement(up, 0, GfVec3d(0, 0, 0)),
                  Placement(up, 0, GfVec3d(1, 0, 0)),
                  Placement(up, 0, GfVec3d(2, 0, 0)),
                  Placement(up, 0, GfVec3d(2, 0, 0)),
                  Placement(up, 0, GfVec3d(1, -1.5, 0)),
                  Placement(up, 0, GfVec3d(0, 0, 0)),
                  Placement(up, 0, GfVec3d(1, 0, 0)),
                  Placement(up, 0, GfVec3d(2, 0, 0))};
    const char *orders[3] = {"XYZ", "ZXY", "YZX"};
    for (size_t k = 0; k < 5; ++k) {
        RigExecIkFkMatchChannels &c = base.channels[k];
        std::copy(orders[k % 3], orders[k % 3] + 3, c.order);
        for (int a = 0; a < 3; ++a) {
            c.t[a] = 0.1 * double(k + a);
            c.r[a] = 5.0 * double(k + 1) + 2.0 * a;
        }
    }
    base.scalars = {0, 1, 0, 1, 1, 0, 0};
    base.poleSwitch.posedDefault = Placement(up, 0, GfVec3d(1, -1.5, 0));
    base.poleSwitch.parentDefault = Placement(up, 0, GfVec3d(0));
    base.poleSwitch.parentDefaultInverse =
        base.poleSwitch.parentDefault.GetInverse();
    base.poleSwitch.parentPosed = Placement(up, 5, GfVec3d(0.1, 0, 0));
    base.poleSwitch.sources.resize(2);
    base.poleSwitch.sources[1].world = true;
    for (size_t k = 0; k < 2; ++k) {
        base.poleSwitch.sources[k].defaultSpace =
            Placement(up, 0, GfVec3d(double(k), 0, 0));
        base.poleSwitch.sources[k].posedSpace =
            Placement(up, 10.0 * double(k + 1), GfVec3d(double(k), 0.2, 0));
    }
    base.poleSwitch.active = 0.0;

    RigExecPoseOpRunner runner;
    int valid = 0, compared = 0;
    const auto compare = [&](const RigExecIkFkMatchOpConstants &constants,
                             const RigExecIkFkMatchOpInputs &in,
                             const std::string &label) {
        bool matched = false;
        const RigExecIkFkMatchOpOutputs want =
            NativeIkFkMatch(constants, in, &matched);
        RigExecIkFkMatchOpOutputs got;
        std::string error;
        const bool ran =
            RigExecRunIkFkMatchOp(constants, in, runner, &got, &error);
        bool same =
            ran && got.matched == matched && got.shownMask == want.shownMask;
        if (same && matched)
            same = Bits(got.ikValues.data(), want.ikValues.data(),
                        sizeof(want.ikValues)) &&
                   Bits(got.fkValues.data(), want.fkValues.data(),
                        sizeof(want.fkValues));
        for (size_t k = 0; same && k < 5; ++k)
            if (want.shownMask & (1u << k))
                same = Bits(got.shown[k].data(), want.shown[k].data(),
                            16 * sizeof(double));
        if (!same)
            std::printf("IK/FK match %s differs: ran %d (%s), matched %d, "
                        "shown %u/%u\n",
                        label.c_str(), int(ran), error.c_str(), int(matched),
                        got.shownMask, want.shownMask);
        CHECK(same);
        ++compared;
        valid += matched ? 1 : 0;
    };
    for (const IkFkPoleMode mode :
         {IkFkPoleMode::Keep, IkFkPoleMode::Ride, IkFkPoleMode::Switch})
        for (const bool stretch : {true, false})
            for (const bool space : {false, true})
                for (const double side : {0.0, 0.5, 1.0}) {
                    RigExecIkFkMatchOpConstants constants;
                    constants.stretchLimb = stretch;
                    constants.scaleCalibration = 0.1;
                    constants.upperLengthOffset = 0.05;
                    constants.poleWorld = {0, 1};
                    constants.poleSourceRides = {1, 0};
                    RigExecIkFkMatchOpInputs in = base;
                    in.poleMode = uint32_t(mode);
                    in.scalars[0] = side;
                    in.scalars[6] = 12.0;
                    in.hasSpace = space;
                    if (space) {
                        in.spaceRest = Placement(up, 0, GfVec3d(0.5, 0, 0));
                        in.spacePosed =
                            Placement(up, 20, GfVec3d(0.5, 0.1, 0), 1.2);
                    }
                    compare(constants, in,
                            "mode " + std::to_string(int(mode)) +
                                (stretch ? " stretch" : "") +
                                (space ? " space" : "") + " side " +
                                std::to_string(side));
                }
    // A non-finite IK control channel: both refuse it alike.
    for (const double poison : kNonFinite)
        for (int a = 0; a < 3; ++a) {
            RigExecIkFkMatchOpConstants constants;
            RigExecIkFkMatchOpInputs in = base;
            in.scalars[0] = 1.0;
            in.channels[0].t[a] = poison;
            compare(constants, in, "non-finite channel");
            in = base;
            in.channels[0].r[a] = poison;
            compare(constants, in, "non-finite rotation");
        }
    std::printf("IK/FK match: %d comparisons, %d matched\n", compared, valid);
    CHECK(valid > 0);
    CHECK(valid < compared);
}

} // namespace

int
main()
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
    TestSkeleton2d();
    TestStaleSettings();
    TestConstraints();
    TestSingleChainIk();
    TestAutoClavicle();
    TestHarmonics();
    TestSwitchMath();
    TestNonFiniteLandmarks();
    TestSpaceMatch();
    TestIkFkMatch();
    // Each kind's comparisons include solved frames, not only empty ones.
    for (const char *kind : {"fk", "two-bone", "blend", "twist", "ribbon",
                             "spline", "skeleton2d", "constraint"}) {
        std::printf("%s: %d solved comparisons\n", kind, solvedFrames[kind]);
        CHECK(solvedFrames[kind] > 0);
    }

    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
