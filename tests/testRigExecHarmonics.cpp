// RigExecHarmonicsConstraint in a rig: the native evaluator against the
// kernel fed with the motion written out by hand, random-access and
// sequential evaluation agreeing bit for bit, the drag reading the authored
// history, the measuring space, the envelope, and the history memo.
#include "rigExecRuntimeDrive.h"
#include "rigExecSceneProgramDrive.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecMath/harmonicsKernel.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace rigExec;

namespace {

int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

const SdfPath kRig("/Asset/Rig");
const SdfPath kRoot("/Asset/Rig/Controls/Root");
const SdfPath kTip("/Asset/Rig/Controls/Root/Tip");
const SdfPath kJiggle("/Asset/Rig/Movers/TipJiggle");

// Root's avars:tx keys: a ramp from frame 1 to 11, a hold, a drop at 30.
const double kKeys[][2] = {{1, 0}, {11, 10}, {20, 10}, {30, -4}, {40, -4}};

double
AuthoredTx(double t)
{
    const size_t n = sizeof(kKeys) / sizeof(kKeys[0]);
    if (t <= kKeys[0][0]) return kKeys[0][1];
    for (size_t k = 1; k < n; ++k) {
        if (t <= kKeys[k][0]) {
            const double u = (t - kKeys[k - 1][0]) / (kKeys[k][0] - kKeys[k - 1][0]);
            return kKeys[k - 1][1] + (kKeys[k][1] - kKeys[k - 1][1]) * u;
        }
    }
    return kKeys[n - 1][1];
}

std::string
StageText(const std::string &jiggleBody)
{
    std::string samples;
    for (const auto &key : kKeys) {
        samples += std::to_string(int(key[0])) + ": " + std::to_string(key[1]) + ", ";
    }
    return R"(#usda 1.0
(
    endTimeCode = 80
    metersPerUnit = 0.01
    startTimeCode = 1
    timeCodesPerSecond = 24
    upAxis = "Y"
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        uniform token rigExec:partition = "Asset"
        def Scope "Controls"
        {
            def RigExecControl "Root" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:tx = 0
                double avars:tx.timeSamples = { )" + samples + R"( }
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                def RigExecControl "Tip" (
                    prepend apiSchemas = ["RigExecControlAPI"]
                )
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 5, 0, 1) )
                }
            }
        }
        def Scope "Weights"
        {
            def RigExecStaticWeight "CardW"
            {
                uniform float rigExec:defaultWeight = 1
                uniform token rigExec:rangePolicy = "strict"
                uniform token rigExec:representation = "constant"
                rel rigExec:weightTarget = </Asset/Geom/Card.points>
            }
        }
        def Scope "Movers"
        {
            def RigExecMatrixMover "CardFollow" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Geom/Card.points>
                rel rigExec:transform = </Asset/Rig/Controls/Root/Tip> (
                    rigExecReadPhase = "final"
                )
                rel rigExec:weightObject = </Asset/Rig/Weights/CardW>
            }
            def RigExecHarmonicsConstraint "TipJiggle" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Rig/Controls/Root/Tip>
)" + jiggleBody + R"(
            }
        }
    }
    def Scope "Geom"
    {
        def Mesh "Card"
        {
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-1, 4, 0), (1, 4, 0), (1, 6, 0), (-1, 6, 0)]
            uniform token subdivisionScheme = "none"
        }
    }
}
)";
}

UsdStageRefPtr
MakeStage(const std::string &jiggleBody = std::string())
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(StageText(jiggleBody)));
    return stage;
}

GfVec3d
TipAt(const RigExecRigPose &pose)
{
    const auto found = pose.controlFrames.find(kTip);
    if (found == pose.controlFrames.end()) return GfVec3d(std::nan(""));
    return found->second.Origin();
}

// The tip's expected position at t: where the root puts it, plus the ring
// of the tip's own motion over the window, from \p txAt.
template <class TxAt>
GfVec3d
Expected(double t, TxAt txAt, double weight = 1.0,
         RigExecHarmonicsParameters params = RigExecHarmonicsParameters())
{
    const RigExecHarmonicsSettings settings;
    const size_t window = RigExecHarmonicsWindowFrames(settings);
    std::vector<double> positions;
    for (size_t k = window; k >= 1; --k) {
        positions.insert(positions.end(), {txAt(t - double(k)), 5.0, 0.0});
    }
    positions.insert(positions.end(), {txAt(t), 5.0, 0.0});
    double ring[3];
    RigExecHarmonicsOffset(settings, params,
                           reinterpret_cast<const double(*)[3]>(positions.data()),
                           positions.size() / 3, ring);
    return GfVec3d(txAt(t) + weight * ring[0], 5.0 + weight * ring[1],
                   weight * ring[2]);
}

bool
Same(const GfVec3d &a, const GfVec3d &b)
{
    return std::memcmp(a.data(), b.data(), sizeof(GfVec3d)) == 0;
}

void
TestMatchesKernel()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const std::string &e : errors) std::printf("compile: %s\n", e.c_str());
    bool moved = false;
    for (int t = 1; t <= 70; ++t) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(pose.valid);
        if (!pose.valid) {
            for (const auto &d : pose.diagnostics) std::printf("  %s\n", d.c_str());
            return;
        }
        const GfVec3d got = TipAt(pose);
        const GfVec3d want = Expected(double(t), AuthoredTx);
        if ((got - want).GetLength() > 1e-9) {
            std::printf("t=%d got (%g %g %g) want (%g %g %g)\n", t, got[0], got[1],
                        got[2], want[0], want[1], want[2]);
            CHECK(false);
        }
        moved = moved || std::fabs(got[0] - AuthoredTx(double(t))) > 1e-3;
    }
    // The ring is visible: the tip overshoots the root somewhere.
    CHECK(moved);
}

void
TestRandomAccessAgrees()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRigEvaluator sequential(stage, kRig);
    RigExecRigEvaluator jumping(stage, kRig);
    CHECK(sequential.Compile());
    CHECK(jumping.Compile());
    std::vector<GfVec3d> ordered(71);
    for (int t = 1; t <= 70; ++t) ordered[size_t(t)] = TipAt(sequential.Evaluate(UsdTimeCode(t)));
    for (int t : {44, 12, 70, 33, 1, 34, 33, 25, 60}) {
        CHECK(Same(TipAt(jumping.Evaluate(UsdTimeCode(t))), ordered[size_t(t)]));
    }
    // A second visit runs no history: the memo holds every tick.
    const size_t before = sequential.GetHistoryTickCountForTesting();
    CHECK(Same(TipAt(sequential.Evaluate(UsdTimeCode(40))), ordered[40]));
    CHECK(sequential.GetHistoryTickCountForTesting() == before);
    // Sequential playback costs no history tick per frame: each frame's
    // own captures become the next one's history.
    RigExecRigEvaluator playback(stage, kRig);
    CHECK(playback.Compile());
    playback.Evaluate(UsdTimeCode(10));
    const size_t first = playback.GetHistoryTickCountForTesting();
    for (int t = 11; t <= 20; ++t) playback.Evaluate(UsdTimeCode(t));
    CHECK(playback.GetHistoryTickCountForTesting() == first);
}

void
TestDragReadsAuthoredHistory()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    CHECK(evaluator.Compile());
    const GfVec3d authored = TipAt(evaluator.Evaluate(UsdTimeCode(25)));
    RigExecValueOverride drag;
    drag.prim = kRoot;
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(20.0);
    evaluator.SetInteractiveOverrides({drag});
    const GfVec3d dragged = TipAt(evaluator.Evaluate(UsdTimeCode(25)));
    // The earlier frames stay the authored animation; the current frame is
    // the drag, so the tip lags behind the jump.
    const auto draggedTx = [](double t) { return t == 25.0 ? 20.0 : AuthoredTx(t); };
    CHECK((dragged - Expected(25.0, draggedTx)).GetLength() < 1e-9);
    CHECK(std::fabs(dragged[0] - 20.0) > 1e-3);
    // Releasing restores the authored frame exactly.
    evaluator.ClearInteractiveOverrides();
    CHECK(Same(TipAt(evaluator.Evaluate(UsdTimeCode(25))), authored));
}

void
TestSpaceAndEnvelope()
{
    // Measured in the root, the tip never moves: nothing rings.
    {
        UsdStageRefPtr stage =
            MakeStage("                rel rigExec:space = </Asset/Rig/Controls/Root>");
        RigExecRigEvaluator evaluator(stage, kRig);
        CHECK(evaluator.Compile());
        for (int t : {5, 12, 31}) {
            const GfVec3d got = TipAt(evaluator.Evaluate(UsdTimeCode(t)));
            CHECK((got - GfVec3d(AuthoredTx(double(t)), 5, 0)).GetLength() < 1e-9);
        }
    }
    // Half the envelope, and the ring's own inputs.
    {
        UsdStageRefPtr stage = MakeStage(
            "                float inputs:defaultWeight = 0.5\n"
            "                double inputs:amplitude = 2\n"
            "                double inputs:decay = 1\n"
            "                double3 inputs:axisAmplitude = (1, 0, 1)\n");
        RigExecRigEvaluator evaluator(stage, kRig);
        CHECK(evaluator.Compile());
        RigExecHarmonicsParameters params;
        params.amplitude = 2;
        params.decay = 1;
        params.axisAmplitude[1] = 0;
        for (int t : {8, 14, 33}) {
            const GfVec3d got = TipAt(evaluator.Evaluate(UsdTimeCode(t)));
            CHECK((got - Expected(double(t), AuthoredTx, 0.5, params)).GetLength() < 1e-9);
        }
    }
}

// The scene program runs the same window: sequential play pushing each
// frame, and jumps that rebuild the history, match the native evaluator.
void
TestSceneProgramMatchesNative()
{
    using rigExecSceneProgramDrive::SceneRun;
    for (const char *body : {"",
             "                rel rigExec:space = </Asset/Rig/Controls/Root>",
             "                float inputs:defaultWeight = 0.5\n"
             "                double inputs:amplitude = 2\n"
             "                double3 inputs:axisAmplitude = (1, 0, 1)\n"}) {
        UsdStageRefPtr stage = MakeStage(body);
        RigExecRigEvaluator native(stage, kRig);
        CHECK(native.Compile());
        std::vector<GfVec3d> want(71);
        for (int t = 1; t <= 70; ++t) want[size_t(t)] = TipAt(native.Evaluate(UsdTimeCode(t)));
        SceneRun scene(stage, kRig, -40, 70);
        CHECK(scene.ok);
        if (!scene.ok) return;
        CHECK(scene.program.windowedOps.size() == 1);
        const auto tip = [&]() {
            const VtValue frame = scene.Final(kTip, RigExecSceneValueDomain::Pose);
            return frame.IsHolding<RigExecPointFrame>() ? frame.UncheckedGet<RigExecPointFrame>().Origin()
                                                        : GfVec3d(std::nan(""));
        };
        for (int t = -40; t <= 70; ++t) {
            CHECK(scene.Evaluate(t));
            scene.runtime.PushHistory();
            if (t >= 1 && !Same(tip(), want[size_t(t)])) {
                const GfVec3d got = tip();
                std::printf("t=%d scene (%g %g %g) native (%g %g %g)\n", t, got[0], got[1], got[2],
                            want[size_t(t)][0], want[size_t(t)][1], want[size_t(t)][2]);
                CHECK(false);
                break;
            }
        }
        SceneRun jumping(stage, kRig, -40, 70);
        for (int t : {44, 12, 70, 33, 1, 34}) {
            CHECK(jumping.Jump(t));
            CHECK(Same(TipAt(native.Evaluate(UsdTimeCode(t))), want[size_t(t)]));
            const VtValue frame = jumping.Final(kTip, RigExecSceneValueDomain::Pose);
            CHECK(frame.IsHolding<RigExecPointFrame>() &&
                  Same(frame.UncheckedGet<RigExecPointFrame>().Origin(), want[size_t(t)]));
        }
    }
}

void
TestEditClearsHistory()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    CHECK(evaluator.Compile());
    evaluator.Evaluate(UsdTimeCode(35));
    const size_t before = evaluator.GetHistoryTickCountForTesting();
    // Move the drop: the history before frame 35 changes.
    UsdAttribute tx = stage->GetPrimAtPath(kRoot).GetAttribute(TfToken("avars:tx"));
    CHECK(tx.Set(-8.0, UsdTimeCode(30)));
    const GfVec3d got = TipAt(evaluator.Evaluate(UsdTimeCode(35)));
    CHECK(evaluator.GetHistoryTickCountForTesting() > before);
    const auto edited = [](double t) {
        if (t <= 20) return AuthoredTx(t);
        if (t <= 30) return 10.0 + (-8.0 - 10.0) * (t - 20.0) / 10.0;
        if (t <= 40) return -8.0 + (-4.0 + 8.0) * (t - 30.0) / 10.0;
        return -4.0;
    };
    CHECK((got - Expected(35.0, edited)).GetLength() < 1e-9);
}

// A background job (frozen snapshot, sampled inputs with their history)
// publishes exactly what live evaluation does, and two frames with the same
// current inputs but different histories digest apart.
void
TestFrozenJobsMatchLive()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    CHECK(evaluator.Compile());
    evaluator.Evaluate(UsdTimeCode(1));
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    std::string why;
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &why));
    if (!frozen) {
        std::printf("freeze: %s\n", why.c_str());
        return;
    }
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    RigExecFrozenEvalContext context;
    context.compiledRigSignature = evaluator.GetCompiledRigSignature();
    context.slotCount = program->GetProviderCount();
    context.programDigest =
        context.compiledRigSignature ^
        (uint64_t(program->GetBoundInputCount()) * 1099511628211ull) ^
        (uint64_t(program->GetVaryingInputCount()) * 16777619ull);
    context.frozen = frozen.get();
    const RigExecFrozenStepRunner runner = RigExecMakeProductionStepRunner();
    for (int t : {12, 31, 5, 47}) {
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(t), {}, &inputs, &why));
        CHECK(inputs.windowRings.size() == 1);
        context.varyingInputCount = inputs.values.size();
        const RigExecRigPose job = RigExecEvaluateFrozen(context, inputs, runner);
        CHECK(job.valid);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(Same(TipAt(job), TipAt(live)));
    }
    // Frames 16 and 18 hold the root at 10 (current inputs equal) after
    // different motion: different digests.
    RigExecFrameInputs a, b;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(16), {}, &a, &why));
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(18), {}, &b, &why));
    CHECK(RigExecControlStateDigest(a) != RigExecControlStateDigest(b));
    // The same frame sampled twice digests the same.
    RigExecFrameInputs again;
    CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(16), {}, &again, &why));
    CHECK(RigExecControlStateDigest(a) == RigExecControlStateDigest(again));
}

// The tip's final frame as the reader published it.
GfVec3d
ReaderTip(const RigExecRuntimeReader &reader)
{
    for (const RigExecRuntimeProviderFrames &frames :
         reader.GetProviderFramePublications()) {
        if (frames.path == kTip.GetString()) {
            const RrVec3d &o = frames.final.points[0];
            return GfVec3d(o[0], o[1], o[2]);
        }
    }
    return GfVec3d(std::nan(""));
}

bool
SameAsNative(const RigExecRigPose &pose, const RigExecRuntimeReader &reader,
             int t)
{
    std::vector<std::string> diffs;
    const bool outputs = RigExecCompareRuntimeOutputs(pose, reader, &diffs);
    const bool tip = Same(TipAt(pose), ReaderTip(reader));
    if (!outputs || !tip) {
        std::printf("t=%d native tip (%.17g %.17g) reader tip (%.17g %.17g)\n", t,
                    TipAt(pose)[0], TipAt(pose)[1], ReaderTip(reader)[0],
                    ReaderTip(reader)[1]);
        for (const std::string &d : diffs) std::printf("  %s\n", d.c_str());
    }
    return outputs && tip;
}

// The .rigexec reader, fed the stage's frames, publishes what the native
// program does: in sequence once its history is full, from any frame after
// a history prefill, and under a drag.
void
TestBinaryParity()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    std::string why;
    CHECK(evaluator.Compile());
    evaluator.Evaluate(UsdTimeCode(1));
    std::vector<uint8_t> bytes;
    if (!RigExecTestBakeAt(evaluator, 1.0, &bytes, &why)) {
        std::printf("bake: %s\n", why.c_str());
        CHECK(false);
        return;
    }
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &why);
    if (!reader) {
        std::printf("open: %s\n", why.c_str());
        CHECK(false);
        return;
    }
    CHECK(reader->GetWindowedOperationCount() == 1);
    const size_t depth = reader->GetHistoryDepth();
    CHECK(depth == RigExecHarmonicsWindowFrames(RigExecHarmonicsSettings()));
    RigExecInputSampler sampler;
    CHECK(sampler.Bind(stage, *reader, &why));
    // Sequential: a game loop's history, full after `depth` frames.
    for (int t = 1; t <= 70; ++t) {
        CHECK(RigExecTestDrive(reader.get(), &sampler, double(t), &why));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        if (size_t(t) > depth + 1) CHECK(SameAsNative(pose, *reader, t));
        reader->PushHistory();
    }
    // Random access: rebuild the history from the stage, then the frame.
    for (int t : {40, 12, 55, 33, 3}) {
        reader->ClearHistory();
        for (size_t k = depth; k >= 1; --k) {
            CHECK(sampler.Apply(UsdTimeCode(double(t) - double(k)), reader.get(), &why));
            CHECK(reader->ExecuteHistory(&why));
        }
        CHECK(RigExecTestDrive(reader.get(), &sampler, double(t), &why));
        CHECK(SameAsNative(evaluator.Evaluate(UsdTimeCode(t)), *reader, t));
    }
    // A drag at the current frame: a set input, no push.
    reader->ClearHistory();
    for (size_t k = depth; k >= 1; --k) {
        CHECK(sampler.Apply(UsdTimeCode(25.0 - double(k)), reader.get(), &why));
        CHECK(reader->ExecuteHistory(&why));
    }
    CHECK(sampler.Apply(UsdTimeCode(25), reader.get(), &why));
    CHECK(reader->SetInput(kRoot.GetString() + ".avars:tx", 20.0, &why));
    CHECK(reader->Execute(&why));
    RigExecValueOverride drag;
    drag.prim = kRoot;
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(20.0);
    evaluator.SetInteractiveOverrides({drag});
    CHECK(SameAsNative(evaluator.Evaluate(UsdTimeCode(25)), *reader, 25));
    evaluator.ClearInteractiveOverrides();
}

void
TestValidation()
{
    UsdStageRefPtr stage = MakeStage("                uniform double rigExec:waves = 0");
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    // Refused, or set aside with a warning: either way it never runs.
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(12));
    CHECK(!compiled || evaluator.GetSkippedOperations().count(kJiggle) == 1 ||
          !pose.valid);
}

}  // namespace

int
main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestMatchesKernel();
    TestRandomAccessAgrees();
    TestDragReadsAuthoredHistory();
    TestSpaceAndEnvelope();
    TestEditClearsHistory();
    TestSceneProgramMatchesNative();
    TestFrozenJobsMatchLive();
    TestBinaryParity();
    TestValidation();
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecHarmonics: all tests passed\n");
    return 0;
}
