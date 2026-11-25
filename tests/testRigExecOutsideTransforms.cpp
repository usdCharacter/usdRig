// Rigs whose asset sits under translated, rotated, non-uniformly scaled and
// animated ancestors. Results are in the asset root's space (the rig prim's
// parent), so an ancestor changes nothing a rig computes from inside its
// asset, and a target outside the asset enters as
// world(target) * world(asset)^-1. Native, frozen and .rigexec playback
// publish the same values under the ancestor.
#include "rigExecRuntimeDrive.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/api/cancellation.h"
#include "rigExec/api/frameCache.h"
#include "rigExec/api/frozenContext.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExecSchema/schemaNames.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/references.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"
#include "pxr/usd/usdGeom/xformOp.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

const TfToken kSources(SchemaAttrText::rigExecSources);
const TfToken kAimTarget(SchemaAttrText::rigExecAimTarget);
const SdfPath kWorld("/World");
const SdfPath kAway("/Away");

using Motion = std::function<GfMatrix4d(double)>;

GfMatrix4d
Scale(const GfVec3d &s)
{
    return GfMatrix4d(1.0).SetScale(s);
}

GfMatrix4d
Rotate(const GfVec3d &axis, double degrees)
{
    return GfMatrix4d(1.0).SetRotate(GfRotation(axis, degrees));
}

GfMatrix4d
Translate(const GfVec3d &t)
{
    return GfMatrix4d(1.0).SetTranslate(t);
}

// The asset's ancestor: non-uniform scale, then rotation, then translation
// about origin, every part animated (row vectors: p_world = p * A).
Motion
AncestorMotion(const GfVec3d &origin)
{
    return [origin](double t) {
        return Scale(GfVec3d(2.0 + 0.25 * std::sin(0.3 * t), 0.5, 3.0)) *
               Rotate(GfVec3d(1, 2, 3), 25.0 + 7.0 * std::sin(0.3 * t)) *
               Translate(origin + GfVec3d(4.0 * std::sin(0.2 * t),
                                          -2.0 * std::cos(0.1 * t),
                                          1.5 * std::sin(0.05 * t)));
    };
}

// A differently placed and animated parent for an outside target.
Motion
AwayMotion(const GfVec3d &origin)
{
    return [origin](double t) {
        return Scale(GfVec3d(1.0, 1.5, 0.75)) *
               Rotate(GfVec3d(0, 1, 1), -40.0 + 10.0 * std::cos(0.15 * t)) *
               Translate(origin + GfVec3d(-6.0 + std::sin(0.25 * t), 3.0,
                                          2.0 * std::cos(0.2 * t)));
    };
}

void
Animate(const UsdStageRefPtr &stage, const SdfPath &path, const Motion &motion,
        const std::vector<double> &times)
{
    const UsdGeomXform xform = UsdGeomXform::Define(stage, path);
    if (!motion) {
        return;
    }
    const UsdGeomXformOp op =
        xform.AddTransformOp(UsdGeomXformOp::PrecisionDouble);
    for (const double t : times) {
        CHECK(op.Set(motion(t), UsdTimeCode(t)));
    }
}

// /World/<asset> referencing \p file's default prim, /World moved by
// \p ancestor (none: an Xform with no transform).
UsdStageRefPtr
AssetStage(const std::string &file, const std::string &asset,
           const Motion &ancestor, const std::vector<double> &times)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    Animate(stage, kWorld, ancestor, times);
    const UsdPrim prim = stage->DefinePrim(kWorld.AppendChild(TfToken(asset)));
    CHECK(prim.GetReferences().AddReference(file));
    return stage;
}

// An outside target /Away/<name> placed by \p away, its own local motion
// translating it through \p local's samples; \p rel on \p mover retargeted
// to it.
void
AddOutsideTarget(const UsdStageRefPtr &stage, const Motion &away,
                 const std::vector<double> &times, const std::string &name,
                 const std::map<double, GfVec3d> &local, const SdfPath &mover,
                 const TfToken &rel)
{
    Animate(stage, kAway, away, times);
    const SdfPath path = kAway.AppendChild(TfToken(name));
    const UsdGeomXform target = UsdGeomXform::Define(stage, path);
    const UsdGeomXformOp op =
        target.AddTranslateOp(UsdGeomXformOp::PrecisionDouble);
    for (const auto &[t, value] : local) {
        CHECK(op.Set(value, UsdTimeCode(t)));
    }
    const UsdRelationship relationship =
        stage->GetPrimAtPath(mover).GetRelationship(rel);
    CHECK(relationship);
    CHECK(relationship.SetTargets({path}));
}

bool
SameBits(double a, double b)
{
    uint64_t x = 0, y = 0;
    std::memcpy(&x, &a, sizeof(x));
    std::memcpy(&y, &b, sizeof(y));
    return x == y;
}

bool
SameBits(float a, float b)
{
    uint32_t x = 0, y = 0;
    std::memcpy(&x, &a, sizeof(x));
    std::memcpy(&y, &b, sizeof(y));
    return x == y;
}

// How far two poses lie apart over their published values. Exact when every
// value has the same bits; otherwise the largest absolute difference and,
// over float values, the largest difference in float ulps at the larger
// magnitude.
struct Deviation {
    bool exact = true;
    bool shapeDiffers = false;
    double maxAbs = 0.0;
    double maxFloatUlps = 0.0;
    size_t values = 0;
    std::string first;

    void Note(const std::string &where)
    {
        if (exact) {
            first = where;
        }
        exact = false;
    }
    void Double(double a, double b, const std::string &where)
    {
        ++values;
        if (SameBits(a, b)) {
            return;
        }
        Note(where);
        const double d = std::abs(a - b);
        maxAbs = std::isfinite(d) ? std::max(maxAbs, d)
                                  : std::numeric_limits<double>::infinity();
    }
    void Float(float a, float b, const std::string &where)
    {
        ++values;
        if (SameBits(a, b)) {
            return;
        }
        Note(where);
        const float m = std::max(std::abs(a), std::abs(b));
        const double ulp =
            double(std::nextafter(m, std::numeric_limits<float>::infinity())) -
            double(m);
        const double d = std::abs(double(a) - double(b));
        maxAbs = std::isfinite(d) ? std::max(maxAbs, d)
                                  : std::numeric_limits<double>::infinity();
        maxFloatUlps = std::isfinite(d)
                           ? std::max(maxFloatUlps, d / ulp)
                           : std::numeric_limits<double>::infinity();
    }
    void Matrix(const GfMatrix4d &a, const GfMatrix4d &b,
                const std::string &where)
    {
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                Double(a[r][c], b[r][c], where);
    }
    void Frame(const RigExecPointFrame &a, const RigExecPointFrame &b,
               const std::string &where)
    {
        if (a.flags != b.flags) {
            Note(where + " flags");
            shapeDiffers = true;
        }
        for (size_t i = 0; i < 4; ++i)
            for (size_t c = 0; c < 3; ++c)
                Double(a.points[i][c], b.points[i][c], where);
    }
    void Value(const VtValue &a, const VtValue &b, const std::string &where)
    {
        if (a.IsHolding<VtVec3fArray>() && b.IsHolding<VtVec3fArray>()) {
            const VtVec3fArray &x = a.UncheckedGet<VtVec3fArray>();
            const VtVec3fArray &y = b.UncheckedGet<VtVec3fArray>();
            if (x.size() != y.size()) {
                Note(where + " size");
                shapeDiffers = true;
                return;
            }
            for (size_t i = 0; i < x.size(); ++i)
                for (size_t c = 0; c < 3; ++c)
                    Float(x[i][c], y[i][c], where);
        } else if (a.IsHolding<GfMatrix4d>() && b.IsHolding<GfMatrix4d>()) {
            Matrix(a.UncheckedGet<GfMatrix4d>(), b.UncheckedGet<GfMatrix4d>(),
                   where);
        } else if (a.IsHolding<double>() && b.IsHolding<double>()) {
            Double(a.UncheckedGet<double>(), b.UncheckedGet<double>(), where);
        } else if (a.IsHolding<float>() && b.IsHolding<float>()) {
            Float(a.UncheckedGet<float>(), b.UncheckedGet<float>(), where);
        } else if (a.IsHolding<GfVec3f>() && b.IsHolding<GfVec3f>()) {
            for (size_t c = 0; c < 3; ++c)
                Float(a.UncheckedGet<GfVec3f>()[c],
                      b.UncheckedGet<GfVec3f>()[c], where);
        } else if (!(a == b)) {
            Note(where);
            shapeDiffers = true;
        }
    }
    template <class Map, class Fn>
    void Maps(const Map &a, const Map &b, const char *domain, Fn each)
    {
        if (a.size() != b.size()) {
            Note(std::string(domain) + " count");
            shapeDiffers = true;
        }
        for (const auto &[key, value] : a) {
            const auto found = b.find(key);
            const std::string where =
                std::string(domain) + " " + key.GetString();
            if (found == b.end()) {
                Note(where + " missing");
                shapeDiffers = true;
                continue;
            }
            each(value, found->second, where);
        }
    }
    void Poses(const RigExecRigPose &a, const RigExecRigPose &b)
    {
        if (a.valid != b.valid) {
            Note("validity");
            shapeDiffers = true;
        }
        const auto frame =
            [this](const RigExecPointFrame &x, const RigExecPointFrame &y,
                   const std::string &where) { Frame(x, y, where); };
        const auto matrix = [this](const GfMatrix4d &x, const GfMatrix4d &y,
                                   const std::string &where) {
            Matrix(x, y, where);
        };
        Maps(a.jointFramesBase, b.jointFramesBase, "jointBase", frame);
        Maps(a.jointFramesFinal, b.jointFramesFinal, "jointFinal", frame);
        Maps(a.jointMatricesFinal, b.jointMatricesFinal, "jointMatrix", matrix);
        Maps(a.controlFrames, b.controlFrames, "control", frame);
        Maps(a.providerXforms, b.providerXforms, "provider", matrix);
        Maps(a.providerBaseXforms, b.providerBaseXforms, "providerBase",
             matrix);
        Maps(a.movedProperties, b.movedProperties, "property",
             [this](const VtValue &x, const VtValue &y,
                    const std::string &where) { Value(x, y, where); });
    }
    std::string Describe() const
    {
        char text[256];
        std::snprintf(text, sizeof(text),
                      "%zu values, %s, max |d| %.3e, max %.2f float ulp%s%s",
                      values, exact ? "bit-identical" : "differ", maxAbs,
                      maxFloatUlps, exact ? "" : ", first at ",
                      exact ? "" : first.c_str());
        return text;
    }
};

struct Evaluation {
    UsdStageRefPtr stage;
    SdfPath rig;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    std::vector<RigExecRigPose> poses;
    std::vector<std::string> notices;
    bool compiled = false;
};

std::unique_ptr<Evaluation>
Evaluate(const UsdStageRefPtr &stage, const SdfPath &rig,
         const std::vector<double> &times)
{
    auto run = std::make_unique<Evaluation>();
    run->stage = stage;
    run->rig = rig;
    run->evaluator = std::make_unique<RigExecRigEvaluator>(stage, rig);
    run->compiled = run->evaluator->Compile(&run->notices);
    if (!run->compiled) {
        for (const std::string &notice : run->notices)
            std::printf("    compile: %s\n", notice.c_str());
        return run;
    }
    for (const double t : times) {
        run->poses.push_back(run->evaluator->Evaluate(UsdTimeCode(t)));
        if (!run->poses.back().valid) {
            std::printf("    invalid pose at %g\n", t);
            for (const std::string &d : run->poses.back().diagnostics)
                std::printf("      %s\n", d.c_str());
        }
    }
    return run;
}

Deviation
Compare(const Evaluation &a, const Evaluation &b)
{
    Deviation deviation;
    if (a.poses.size() != b.poses.size()) {
        deviation.Note("pose count");
        deviation.shapeDiffers = true;
        return deviation;
    }
    for (size_t i = 0; i < a.poses.size(); ++i)
        deviation.Poses(a.poses[i], b.poses[i]);
    return deviation;
}

// Frozen jobs and the baked binary publish what native evaluation of the
// same stage published, bit for bit. A binary whose animated source is an
// Xform outside the rig holds it as static data at the bake time
// (tests/exampleFixtures.cmake's 'static' class, as for example 14), so with
// \p followsTime false it is held to native at the bake time only.
void
CheckEvaluators(const char *what, Evaluation &run,
                const std::vector<double> &times, bool followsTime = true)
{
    RigExecRigEvaluator &evaluator = *run.evaluator;
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string error;
    if (!RigExecFreezeProgram(evaluator, &frozen, &error) || !frozen) {
        ++failures;
        std::printf("FAIL %s: freeze refused: %s\n", what, error.c_str());
    } else {
        Deviation deviation;
        RigExecCancellationSource cancellation;
        RigExecFrozenEvalContext context;
        context.compiledRigSignature = evaluator.GetCompiledRigSignature();
        context.generation = cancellation.Serial();
        context.slotCount = evaluator.GetBakedProgram()->GetProviderCount();
        context.frozen = frozen.get();
        for (size_t i = 0; i < times.size(); ++i) {
            RigExecFrameInputs inputs;
            CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(times[i]), {},
                                           &inputs, &error));
            context.varyingInputCount = inputs.values.size();
            const RigExecRigPose pose = RigExecEvaluateFrozen(
                context, inputs, RigExecMakeProductionStepRunner(),
                cancellation.Token(context.generation));
            deviation.Poses(pose, run.poses[i]);
        }
        std::printf("  %s frozen vs native: %s\n", what,
                    deviation.Describe().c_str());
        CHECK(deviation.exact);
    }
    std::vector<uint8_t> bytes;
    if (!RigExecTestBakeAt(evaluator, times.front(), &bytes, &error)) {
        ++failures;
        std::printf("FAIL %s: bake refused: %s\n", what, error.c_str());
        return;
    }
    RigExecTestPlayer player;
    if (!player.Open(bytes, run.stage, &error)) {
        ++failures;
        std::printf("FAIL %s: open refused: %s\n", what, error.c_str());
        return;
    }
    size_t same = 0;
    const size_t played = followsTime ? times.size() : 1;
    for (size_t i = 0; i < played; ++i) {
        std::vector<std::string> diffs;
        if (!player.Play(times[i], &error)) {
            ++failures;
            std::printf("FAIL %s: runtime at %g: %s\n", what, times[i],
                        error.c_str());
            continue;
        }
        if (RigExecCompareRuntimeRun(run.poses[i], player.Reader(), &diffs,
                                     /*outputsOnly=*/true)) {
            ++same;
            continue;
        }
        ++failures;
        std::printf("FAIL %s: runtime differs from native at %g\n", what,
                    times[i]);
        for (const std::string &line : diffs)
            std::printf("    %s\n", line.c_str());
    }
    std::printf("  %s runtime vs native: %zu/%zu frames bit-identical\n", what,
                same, played);
}

struct Case {
    const char *name;
    std::string file;
    std::string asset;
    std::vector<double> times;
};

SdfPath
RigOf(const Case &c)
{
    return kWorld.AppendChild(TfToken(c.asset)).AppendChild(TfToken("Rig"));
}

// A rig that reads nothing outside its asset publishes the same bits under
// any ancestor.
void
CheckUnchanged(const Case &c, const GfVec3d &origin, const char *label,
               bool followsTime)
{
    const auto bare =
        Evaluate(AssetStage(c.file, c.asset, {}, c.times), RigOf(c), c.times);
    auto placed =
        Evaluate(AssetStage(c.file, c.asset, AncestorMotion(origin), c.times),
                 RigOf(c), c.times);
    CHECK(bare->compiled && placed->compiled);
    if (!bare->compiled || !placed->compiled) {
        return;
    }
    const Deviation deviation = Compare(*placed, *bare);
    std::printf("%s %s, ancestor vs none: %s\n", c.name, label,
                deviation.Describe().c_str());
    CHECK(deviation.values > 0);
    CHECK(deviation.exact);
    const std::string what = std::string(c.name) + " " + label;
    CheckEvaluators(what.c_str(), *placed, c.times, followsTime);
}

// A target outside the asset, under its own animated parent B, enters the
// rig as world(target) * world(asset)^-1. Under ancestor A that equals the
// same rig with no ancestor and the target's parent at B * A^-1, which the
// reference authors directly.
Deviation
CheckOutsideTarget(const Case &c, const SdfPath &mover, const TfToken &rel,
                   const std::map<double, GfVec3d> &local,
                   const Motion &ancestor, const Motion &away,
                   const Motion &awayRelative, const char *label,
                   bool evaluators)
{
    const std::string targetName = "Target";
    const UsdStageRefPtr placedStage =
        AssetStage(c.file, c.asset, ancestor, c.times);
    AddOutsideTarget(placedStage, away, c.times, targetName, local, mover, rel);
    const UsdStageRefPtr bareStage = AssetStage(c.file, c.asset, {}, c.times);
    AddOutsideTarget(bareStage, awayRelative, c.times, targetName, local, mover,
                     rel);
    auto placed = Evaluate(placedStage, RigOf(c), c.times);
    const auto bare = Evaluate(bareStage, RigOf(c), c.times);
    Deviation deviation;
    CHECK(placed->compiled && bare->compiled);
    if (!placed->compiled || !bare->compiled) {
        deviation.Note("compile");
        return deviation;
    }
    deviation = Compare(*placed, *bare);
    std::printf("%s %s, ancestor vs relative placement: %s\n", c.name, label,
                deviation.Describe().c_str());
    CHECK(deviation.values > 0);
    CHECK(!deviation.shapeDiffers);
    if (evaluators) {
        const std::string what = std::string(c.name) + " " + label;
        CheckEvaluators(what.c_str(), *placed, c.times, false);
    }
    return deviation;
}

Motion
Relative(const Motion &away, const Motion &ancestor)
{
    return [away, ancestor](double t) {
        return away(t) * ancestor(t).GetInverse();
    };
}

const GfVec3d kNear(30.0, -12.0, 7.0);
const GfVec3d kFar(1.0e6, -7.0e5, 3.0e5);

std::string examples, fixtures;

Case
Skin()
{
    return {"skin",
            fixtures + "/oneloop_two_limbs.usda",
            "LimbsAsset",
            {1, 3, 6, 9, 12}};
}

Case
Sweep()
{
    return {"constraint",
            examples + "/14_VolumeConstrainedSweep.usda",
            "SweepAsset",
            {1001, 1012, 1024, 1036, 1048}};
}

Case
Drape()
{
    return {"surface",
            examples + "/07_SurfaceDrape.usda",
            "DrapeAsset",
            {1001, 1012, 1024, 1036, 1048}};
}

Case
Turret()
{
    return {"aim",
            examples + "/10_AimXformTurret.usda",
            "TurretAsset",
            {1001, 1012, 1024, 1036, 1048}};
}

const std::map<double, GfVec3d> kTargetPath = {{1001, GfVec3d(0, 5, 0)},
                                               {1024, GfVec3d(6, 5, 0)},
                                               {1048, GfVec3d(6, 5, 4)}};

const std::map<double, GfVec3d> kLookPath = {{1001, GfVec3d(-8, 2, 10)},
                                             {1024, GfVec3d(8, 2, 10)},
                                             {1048, GfVec3d(-8, 4, 6)}};

// (a) skinning, (c) surface movers on an animated driver inside the asset,
// (d) an aim at a control, and (b) example 14's position constraint to an
// animated Xform outside the rig but inside the asset: nothing outside the
// asset is read, so every value keeps its bits, also ~1e6 from the origin.
void
TestInsideAsset()
{
    for (const Case &c : {Skin(), Drape(), Turret(), Sweep()}) {
        const bool followsTime = c.asset != Sweep().asset;
        CheckUnchanged(c, kNear, "near", followsTime);
        CheckUnchanged(c, kFar, "far", followsTime);
    }
}

// (b) a position constraint and (d) an aim whose target lies outside the
// asset under a differently placed, animated parent. The target's asset
// space position is computed two ways that differ only by rounding
// (world(target) * world(asset)^-1 against B * A^-1 composed in the test).
// Published points are float, so they agree to one float ulp at their
// magnitude; published matrices are double. Measured (Windows, MSVC): points
// bit-identical, matrices 1.3e-15; kNearMatrixTolerance is the bound the
// float32 switch has to meet or consciously relax.
constexpr double kNearMatrixTolerance = 1e-14;

// Times past every sample the example authors, where only the asset's
// ancestor moves. The outside target's asset-space frame is a stage seed of
// the frame-cache key, so a frame there must not be served from the cache
// entry of another such time, and the control (a static ancestor) shows
// nothing else moved.
void
TestCacheKeyFollowsAncestor()
{
    const Case c = Sweep();
    const std::vector<double> times = {2000, 2001};
    const SdfPath mover = RigOf(c).AppendPath(SdfPath("Movers/Sweep"));
    const Motion moving = AncestorMotion(kNear);
    const Motion still = [moving](double) { return moving(2000); };
    const auto digests = [&](const Motion &ancestor, bool *posesDiffer) {
        const UsdStageRefPtr stage =
            AssetStage(c.file, c.asset, ancestor, times);
        AddOutsideTarget(stage, {}, times, "Target", {{1001, GfVec3d(0, 5, 0)}},
                         mover, kSources);
        const auto run = Evaluate(stage, RigOf(c), times);
        std::vector<uint64_t> out;
        CHECK(run->compiled && run->poses.size() == times.size());
        if (!run->compiled || run->poses.size() != times.size()) {
            return out;
        }
        Deviation deviation;
        deviation.Poses(run->poses[0], run->poses[1]);
        *posesDiffer = !deviation.exact;
        for (const double t : times) {
            RigExecFrameInputs inputs;
            std::string error;
            CHECK(RigExecSampleFrameInputs(*run->evaluator, UsdTimeCode(t), {},
                                           &inputs, &error));
            CHECK(RigExecControlStateDigestible(inputs));
            out.push_back(RigExecControlStateDigest(inputs));
        }
        return out;
    };
    bool stillDiffers = true, movingDiffers = false;
    const std::vector<uint64_t> fixed = digests(still, &stillDiffers);
    const std::vector<uint64_t> moved = digests(moving, &movingDiffers);
    std::printf("cache key, static ancestor: poses %s, digests %s\n",
                stillDiffers ? "differ" : "equal",
                fixed.size() == 2 && fixed[0] == fixed[1] ? "equal" : "differ");
    std::printf("cache key, animated ancestor: poses %s, digests %s\n",
                movingDiffers ? "differ" : "equal",
                moved.size() == 2 && moved[0] == moved[1] ? "equal" : "differ");
    CHECK(!stillDiffers);
    CHECK(fixed.size() == 2 && fixed[0] == fixed[1]);
    CHECK(movingDiffers);
    CHECK(moved.size() == 2 && moved[0] != moved[1]);
}

void
TestOutsideTarget()
{
    {
        const Case c = Sweep();
        const SdfPath mover = RigOf(c).AppendPath(SdfPath("Movers/Sweep"));
        const Motion a = AncestorMotion(kNear), b = AwayMotion(kNear);
        const Deviation d =
            CheckOutsideTarget(c, mover, kSources, kTargetPath, a, b,
                               Relative(b, a), "outside target near", true);
        CHECK(d.maxFloatUlps <= 1.0);
    }
    {
        const Case c = Turret();
        const SdfPath mover = RigOf(c).AppendPath(SdfPath("Movers/AimBarrel"));
        const Motion a = AncestorMotion(kNear), b = AwayMotion(kNear);
        const Deviation d =
            CheckOutsideTarget(c, mover, kAimTarget, kLookPath, a, b,
                               Relative(b, a), "outside target near", true);
        CHECK(d.maxAbs <= kNearMatrixTolerance);
    }
}

// The asset ~1e6 units from the origin and the outside target's parent a
// modest offset O(t) from it (B = O * A, so B * A^-1 = O exactly). Forming
// world(target) * world(asset)^-1 from world matrices whose translations are
// ~1e6 cancels in double. Measured (Windows, MSVC): constraint points
// bit-identical, aim matrices 4.3e-12. The bounds below are what the float32
// switch has to meet or consciously relax; points keep one float ulp because
// a double residue rounds to float either way.
constexpr double kFarMatrixBound = 1e-11;
constexpr double kFarPointUlpBound = 1.0;

void
TestFarFromOrigin()
{
    const Motion a = AncestorMotion(kFar);
    const Motion offset = AwayMotion(GfVec3d(0.0));
    const Motion b = [a, offset](double t) { return offset(t) * a(t); };
    {
        const Case c = Sweep();
        const SdfPath mover = RigOf(c).AppendPath(SdfPath("Movers/Sweep"));
        const Deviation d =
            CheckOutsideTarget(c, mover, kSources, kTargetPath, a, b, offset,
                               "outside target far", false);
        std::printf("FAR constraint points: max |d| %.3e, %.2f float ulp\n",
                    d.maxAbs, d.maxFloatUlps);
        CHECK(d.maxFloatUlps <= kFarPointUlpBound);
    }
    {
        const Case c = Turret();
        const SdfPath mover = RigOf(c).AppendPath(SdfPath("Movers/AimBarrel"));
        const Deviation d =
            CheckOutsideTarget(c, mover, kAimTarget, kLookPath, a, b, offset,
                               "outside target far", false);
        std::printf("FAR aim matrices: max |d| %.3e\n", d.maxAbs);
        CHECK(d.maxAbs <= kFarMatrixBound);
    }
}

const char *kProjectorRig = R"USDA(#usda 1.0
def Xform "World"
{
    def Xform "Asset"
    {
        def RigExecRoot "Rig"
        {
            def RigExecControl "Driver"
            {
                double avars:tx.timeSamples = {1: 0, 2: 0.25, 3: 0}
            }
            def RigExecControl "Source"
            {
                double rest:tx = 4
                double rest:ty = -2
                double rest:tz = 6
                double avars:ty.timeSamples = {1: 0, 2: 0.5, 3: -0.25}
            }
            def Scope "Movers"
            {
                def RigExecMatrixMover "Deform" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    float inputs:defaultWeight = 1
                    rel rigExec:moves = </World/Asset/Surface.points>
                    rel rigExec:transform = </World/Asset/Rig/Driver>
                }
                def RigExecSurfaceProjector "Project" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    rel rigExec:moves = </World/Asset/Surface.points>
                    rel rigExec:sources = </World/Asset/Rig/Source>
                    token rigExec:shaderPrimvar = "projectionFrame"
                }
            }
        }
        def Mesh "Surface"
        {
            double3 xformOp:translate = (4, -2, 7)
            float xformOp:rotateY = 20
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateY"]
            point3f[] points = [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
        }
    }
}
)USDA";

const std::vector<double> kProjectorTimes = {1, 1.5, 2, 2.5, 3};

UsdStageRefPtr
ProjectorStage(const Motion &ancestor, bool animated)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(kProjectorRig));
    if (ancestor) {
        const UsdGeomXformOp op =
            UsdGeomXform(stage->GetPrimAtPath(kWorld))
                .AddTransformOp(UsdGeomXformOp::PrecisionDouble);
        if (animated) {
            for (const double t : kProjectorTimes)
                CHECK(op.Set(ancestor(t), UsdTimeCode(t)));
        } else {
            CHECK(op.Set(ancestor(kProjectorTimes.front())));
        }
    }
    return stage;
}

// (c) a surface projector onto a static mesh inside the asset. The ray's
// mesh-to-asset map is formed at compile from world matrices, so a static
// ancestor changes it by rounding only. Measured today: 5.6e-17 near and
// 8.3e-17 ~1e6 from the origin in the published frame, float points
// bit-identical; held to kProjectorTolerance and one float ulp, the bound
// the float32 switch has to meet or consciously relax.
Deviation
CheckProjector(const GfVec3d &origin, bool animated, const char *label)
{
    const SdfPath rig("/World/Asset/Rig");
    const auto bare = Evaluate(ProjectorStage({}, false), rig, kProjectorTimes);
    auto placed = Evaluate(ProjectorStage(AncestorMotion(origin), animated),
                           rig, kProjectorTimes);
    Deviation deviation;
    CHECK(bare->compiled && placed->compiled);
    if (!bare->compiled || !placed->compiled) {
        deviation.Note("compile");
        deviation.shapeDiffers = true;
        return deviation;
    }
    deviation = Compare(*placed, *bare);
    std::printf("projector %s, ancestor vs none: %s\n", label,
                deviation.Describe().c_str());
    CHECK(deviation.values > 0);
    CHECK(!deviation.shapeDiffers);
    const std::string what = std::string("projector ") + label;
    CheckEvaluators(what.c_str(), *placed, kProjectorTimes);
    return deviation;
}

constexpr double kProjectorTolerance = 1e-15;

void
TestProjector()
{
    const Deviation close = CheckProjector(kNear, false, "static near");
    CHECK(close.maxFloatUlps <= 1.0 && close.maxAbs <= kProjectorTolerance);
    const Deviation distant = CheckProjector(kFar, false, "static far");
    std::printf("FAR projector: max |d| %.3e, %.2f float ulp\n", distant.maxAbs,
                distant.maxFloatUlps);
    CHECK(distant.maxFloatUlps <= 1.0 && distant.maxAbs <= kProjectorTolerance);
}

// The mesh-to-asset map does not change when only an ancestor above the
// asset root animates, so the projector publishes the static ancestor's
// values.
void
TestProjectorAnimatedAncestor()
{
    const Deviation d = CheckProjector(kNear, true, "animated near");
    CHECK(d.maxFloatUlps <= 1.0 && d.maxAbs <= kProjectorTolerance);
}

} // namespace

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    if (argc < 3) {
        std::printf("usage: %s <examples dir> <fixtures dir> [case]\n",
                    argv[0]);
        return 1;
    }
    examples = argv[1];
    fixtures = argv[2];
    const std::string only = argc > 3 ? argv[3] : "";
    if (only.empty()) {
        TestInsideAsset();
        TestProjector();
    } else if (only == "outside-target") {
        TestOutsideTarget();
        TestCacheKeyFollowsAncestor();
    } else if (only == "outside-target-far") {
        TestFarFromOrigin();
    } else if (only == "projector-animated-ancestor") {
        TestProjectorAnimatedAncestor();
    } else {
        std::printf("unknown case %s\n", only.c_str());
        return 1;
    }
    std::printf("OutsideTransforms: %d failures\n", failures);
    return failures ? 1 : 0;
}
