// RigExecClothMover in a rig: a curtain pinned along its top edge follows a
// control and drapes onto a box that moves with it. The native evaluator
// against the cloth kernel fed the same window by hand, random access and
// sequential evaluation agreeing bit for bit, the drag reading the authored
// history, a background job publishing what live evaluation does, and the
// .rigexec reader, with the cloth kernel the runtime installs itself,
// matching the native points.
#include "rigExecClothStage.h"
#include "rigExecRuntimeDrive.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/clothPlayback.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace rigExec;
using namespace rigExecClothStage;

namespace {

int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

// The epoch the mover compiles for the stage: its topology, rest points and
// pins, the default settings, 24 frames per second, and the box.
RigExecClothEpoch
StageEpoch(const UsdStageRefPtr &stage)
{
    RigExecClothEpoch epoch;
    const UsdPrim curtain = stage->GetPrimAtPath(kCurtain.GetPrimPath());
    VtIntArray counts, indices;
    VtVec3fArray rest;
    VtFloatArray pins;
    curtain.GetAttribute(TfToken("faceVertexCounts")).Get(&counts);
    curtain.GetAttribute(TfToken("faceVertexIndices")).Get(&indices);
    curtain.GetAttribute(TfToken("points")).Get(&rest);
    stage->GetPrimAtPath(kCloth).GetAttribute(TfToken("rigExec:pinWeights")).Get(&pins);
    epoch.pointCount = uint32_t(rest.size());
    epoch.counts.assign(counts.begin(), counts.end());
    epoch.indices.assign(indices.begin(), indices.end());
    epoch.rest.resize(3 * rest.size());
    std::memcpy(epoch.rest.data(), rest.cdata(), epoch.rest.size() * sizeof(float));
    epoch.pins.assign(pins.begin(), pins.end());
    epoch.frameSeconds = 1.0 / 24.0;
    const UsdPrim box = stage->GetPrimAtPath(kBox.GetPrimPath());
    VtIntArray boxCounts, boxIndices;
    VtVec3fArray boxPoints;
    box.GetAttribute(TfToken("faceVertexCounts")).Get(&boxCounts);
    box.GetAttribute(TfToken("faceVertexIndices")).Get(&boxIndices);
    box.GetAttribute(TfToken("points")).Get(&boxPoints);
    RigExecClothColliderTopology collider;
    collider.path = kBox.GetString();
    collider.pointCount = uint32_t(boxPoints.size());
    collider.counts.assign(boxCounts.begin(), boxCounts.end());
    collider.indices.assign(boxIndices.begin(), boxIndices.end());
    epoch.colliders.push_back(collider);
    return epoch;
}

// The cloth at t from the kernel: the entering curtain and the box over the
// history a window may read, as an evaluator whose cloth is disabled publishes
// them, solved by hand. \p dragAt, when set, is the drag the current frame
// carries.
Points
Expected(RigExecRigEvaluator &entering, const RigExecClothPrepared &prepared, int t,
         const std::vector<RigExecValueOverride> *drag = nullptr,
         std::vector<RigExecWindowPacket> *window = nullptr)
{
    const int frames = RigExecClothHistoryFrames(prepared.epoch.settings);
    std::vector<RigExecWindowPacket> packets;
    for (int k = frames; k >= 0; --k) {
        if (k == 0 && drag) entering.SetInteractiveOverrides(*drag);
        const RigExecRigPose pose = entering.Evaluate(UsdTimeCode(double(t - k)));
        if (k == 0 && drag) entering.ClearInteractiveOverrides();
        const Points curtain = PointsOf(pose, kCurtain);
        const Points box = PointsOf(pose, kBox);
        RigExecWindowPacket packet;
        const RigExecClothParameters params;
        CHECK(RigExecClothPacket(prepared.epoch,
                                 reinterpret_cast<const float *>(curtain.data()),
                                 curtain.size(),
                                 {reinterpret_cast<const float *>(box.data())},
                                 {box.size()}, params, &packet));
        packets.push_back(std::move(packet));
    }
    std::vector<const RigExecWindowPacket *> ticks;
    for (const auto &packet : packets) ticks.push_back(&packet);
    Points out(prepared.epoch.pointCount);
    std::string why;
    CHECK(RigExecClothSolvePackets(prepared, ticks.data(), ticks.size(),
                                   reinterpret_cast<float *>(out.data()), &why));
    if (!why.empty()) std::printf("solve: %s\n", why.c_str());
    if (window) *window = std::move(packets);
    return out;
}

bool
Compile(RigExecRigEvaluator &evaluator)
{
    std::vector<std::string> errors;
    const bool ok = evaluator.Compile(&errors);
    for (const std::string &e : errors) std::printf("compile: %s\n", e.c_str());
    return ok;
}

UsdStageRefPtr FeatureStage()
{
    auto stage = MakeStage();
    auto cloth = stage->GetPrimAtPath(kCurtain.GetPrimPath());
    auto mover = stage->GetPrimAtPath(kCloth);
    VtVec3fArray rest;
    cloth.GetAttribute(TfToken("points")).Get(&rest);
    VtVec2fArray uv;
    for (const auto &point : rest) uv.push_back(GfVec2f(point[0], point[1]));
    auto st = cloth.CreateAttribute(TfToken("primvars:st"), SdfValueTypeNames->TexCoord2fArray);
    st.Set(uv); st.SetMetadata(TfToken("interpolation"), VtValue(TfToken("vertex")));
    mover.CreateRelationship(TfToken("rigExec:uvSet")).SetTargets({st.GetPath()});
    auto shape = stage->DefinePrim(SdfPath("/Asset/Rest"), TfToken("Mesh"));
    shape.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray).Set(rest);
    mover.CreateRelationship(TfToken("rigExec:restShape")).SetTargets({shape.GetPath()});
    const auto profile = [&](const char *name, const char *kind) {
        auto prim = stage->DefinePrim(kCloth.AppendChild(TfToken(name)), TfToken("RigExecClothConstraintSet"));
        prim.CreateAttribute(TfToken("rigExec:kind"), SdfValueTypeNames->Token).Set(TfToken(kind));
        return prim;
    };
    auto stretch = profile("Stretch", "stretch");
    stretch.CreateAttribute(TfToken("inputs:warp"), SdfValueTypeNames->Float).Set(4.0f);
    stretch.CreateAttribute(TfToken("inputs:weft"), SdfValueTypeNames->Float).Set(.5f);
    stretch.CreateAttribute(TfToken("rigExec:forceCurve"), SdfValueTypeNames->Float2Array)
        .Set(VtVec2fArray{GfVec2f(0, 1), GfVec2f(10000, .2f)});
    auto damping = profile("Damping", "damping");
    damping.CreateAttribute(TfToken("inputs:minStrength"), SdfValueTypeNames->Float).Set(8.0f);
    damping.CreateAttribute(TfToken("inputs:maxStrength"), SdfValueTypeNames->Float).Set(8.0f);
    auto paint = cloth.CreateAttribute(TfToken("primvars:damping"), SdfValueTypeNames->FloatArray);
    paint.Set(VtFloatArray(rest.size(), 1.0f));
    damping.CreateRelationship(TfToken("rigExec:maskMap")).SetTargets({paint.GetPath()});
    auto a = stage->DefinePrim(kCloth.AppendChild(TfToken("A")), TfToken("RigExecClothBinding"));
    a.CreateAttribute(TfToken("rigExec:indices"), SdfValueTypeNames->IntArray).Set(VtIntArray{42});
    auto b = stage->DefinePrim(kCloth.AppendChild(TfToken("B")), TfToken("RigExecClothBinding"));
    b.CreateAttribute(TfToken("rigExec:mode"), SdfValueTypeNames->Token).Set(TfToken("uv"));
    b.CreateAttribute(TfToken("rigExec:coordinates"), SdfValueTypeNames->Float3Array)
        .Set(VtVec3fArray{GfVec3f(0, 40, 0)});
    auto attachment = profile("Link", "attachment");
    attachment.CreateRelationship(TfToken("rigExec:sourceBinding")).SetTargets({a.GetPath()});
    attachment.CreateRelationship(TfToken("rigExec:targetBinding")).SetTargets({b.GetPath()});
    mover.CreateRelationship(TfToken("rigExec:constraints"))
        .SetTargets({stretch.GetPath(), damping.GetPath(), attachment.GetPath()});
    return stage;
}

void TestPaintAndRestInvalidation()
{
    auto stage = FeatureStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    CHECK(Compile(evaluator));
    auto first = evaluator.Evaluate(UsdTimeCode(12));
    CHECK(first.valid);
    auto paint = stage->GetAttributeAtPath(kCurtain.GetPrimPath().AppendProperty(TfToken("primvars:damping")));
    paint.Set(VtFloatArray(kColumns * kRows, 0.0f));
    const auto changed = evaluator.Evaluate(UsdTimeCode(12));
    CHECK(changed.valid);
    CHECK(Furthest(PointsOf(first, kCurtain), PointsOf(changed, kCurtain)) > 1e-3f);
    RigExecRigEvaluator fresh(stage, kRig);
    CHECK(Compile(fresh));
    CHECK(SameBits(PointsOf(changed, kCurtain), PointsOf(fresh.Evaluate(UsdTimeCode(12)), kCurtain)));
    // The solve keeps its material model across frames while the model's
    // reads hold, and rebuilds it when a profile value changes.
    CHECK(evaluator.Evaluate(UsdTimeCode(9)).valid);
    CHECK(SameBits(PointsOf(changed, kCurtain), PointsOf(evaluator.Evaluate(UsdTimeCode(12)), kCurtain)));
    CHECK(stage->GetAttributeAtPath(kCloth.AppendChild(TfToken("Stretch"))
                                        .AppendProperty(TfToken("inputs:warp"))).Set(1.0f));
    const auto rewarped = evaluator.Evaluate(UsdTimeCode(12));
    CHECK(rewarped.valid);
    CHECK(!SameBits(PointsOf(changed, kCurtain), PointsOf(rewarped, kCurtain)));
    RigExecRigEvaluator again(stage, kRig);
    CHECK(Compile(again));
    CHECK(SameBits(PointsOf(rewarped, kCurtain), PointsOf(again.Evaluate(UsdTimeCode(12)), kCurtain)));
    auto shape = stage->GetAttributeAtPath(SdfPath("/Asset/Rest.points"));
    VtVec3fArray points; shape.Get(&points);
    for (auto &p : points) p[0] *= .8f;
    shape.Set(points);
    const auto reshaped = evaluator.Evaluate(UsdTimeCode(12));
    CHECK(reshaped.valid);
    CHECK(Furthest(PointsOf(changed, kCurtain), PointsOf(reshaped, kCurtain)) > 1e-3f);
}

void TestBindingAndMapTransfer()
{
    auto stage = FeatureStage();
    const auto evaluate = [&]() {
        RigExecRigEvaluator rig(stage, kRig);
        CHECK(Compile(rig));
        const auto pose = rig.Evaluate(UsdTimeCode(12));
        CHECK(pose.valid);
        return PointsOf(pose, kCurtain);
    };
    const auto uvResult = evaluate();
    auto binding = stage->GetPrimAtPath(kCloth.AppendChild(TfToken("B")));
    binding.GetAttribute(TfToken("rigExec:mode")).Set(TfToken("vertex"));
    binding.CreateAttribute(TfToken("rigExec:indices"), SdfValueTypeNames->IntArray).Set(VtIntArray{3});
    binding.GetAttribute(TfToken("rigExec:coordinates")).Set(VtVec3fArray());
    CHECK(SameBits(uvResult, evaluate()));
    binding.GetAttribute(TfToken("rigExec:mode")).Set(TfToken("barycentric"));
    binding.GetAttribute(TfToken("rigExec:indices")).Set(VtIntArray{4});
    binding.GetAttribute(TfToken("rigExec:coordinates")).Set(VtVec3fArray{GfVec3f(0, 1, 0)});
    CHECK(SameBits(uvResult, evaluate()));

    auto cloth = stage->GetPrimAtPath(kCurtain.GetPrimPath());
    VtVec3fArray rest; cloth.GetAttribute(TfToken("points")).Get(&rest);
    auto transfer = stage->DefinePrim(kCloth.AppendChild(TfToken("Transfer")), TfToken("RigExecClothBinding"));
    transfer.CreateAttribute(TfToken("rigExec:mode"), SdfValueTypeNames->Token).Set(TfToken("uv"));
    VtVec3fArray coordinates;
    for (const auto &point : rest) coordinates.push_back(GfVec3f(point[0], point[1], 0));
    transfer.CreateAttribute(TfToken("rigExec:coordinates"), SdfValueTypeNames->Float3Array).Set(coordinates);
    auto map = stage->DefinePrim(kCloth.AppendChild(TfToken("Paint")), TfToken("RigExecClothMap"));
    map.CreateRelationship(TfToken("rigExec:values")).SetTargets({cloth.GetPath().AppendProperty(TfToken("primvars:damping"))});
    map.CreateRelationship(TfToken("rigExec:binding")).SetTargets({transfer.GetPath()});
    stage->GetPrimAtPath(kCloth.AppendChild(TfToken("Damping")))
        .GetRelationship(TfToken("rigExec:maskMap")).SetTargets({map.GetPath()});
    CHECK(SameBits(uvResult, evaluate()));
    // Indexed face-varying UVs use corners, not vertex indices as UV indices.
    VtIntArray corners; cloth.GetAttribute(TfToken("faceVertexIndices")).Get(&corners);
    auto st = cloth.GetAttribute(TfToken("primvars:st"));
    st.SetMetadata(TfToken("interpolation"), VtValue(TfToken("faceVarying")));
    cloth.CreateAttribute(TfToken("primvars:st:indices"), SdfValueTypeNames->IntArray).Set(corners);
    CHECK(SameBits(uvResult, evaluate()));
}

void
TestMatchesKernel()
{
    UsdStageRefPtr stage = MakeStage();
    UsdStageRefPtr disabled = MakeStage(false);
    RigExecRigEvaluator evaluator(stage, kRig);
    RigExecRigEvaluator entering(disabled, kRig);
    CHECK(Compile(evaluator));
    CHECK(Compile(entering));
    RigExecClothPrepared prepared;
    std::string why;
    CHECK(RigExecPrepareCloth(StageEpoch(stage), &prepared, &why));
    float sag = 0.0f;
    for (int t = 1; t <= 40; t += 3) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(pose.valid);
        if (!pose.valid) {
            for (const auto &d : pose.diagnostics) std::printf("  %s\n", d.c_str());
            return;
        }
        const Points got = PointsOf(pose, kCurtain);
        std::vector<RigExecWindowPacket> reference;
        const Points want = Expected(entering, prepared, t, nullptr, &reference);
        if (!SameBits(got, want)) {
            std::printf("t=%d native and kernel differ by %g\n", t, Furthest(got, want));
            CHECK(false);
        }
        // The history the evaluator holds is the reference's earlier ticks.
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(t), {}, &inputs, &why));
        if (inputs.windowRings.size() == 1 &&
            inputs.windowRings[0].size() + 1 == reference.size()) {
            const RigExecWindowRing &ring = inputs.windowRings[0];
            for (size_t k = 0; k < ring.size(); ++k) {
                if (!ring[k] || *ring[k] != reference[k]) {
                    std::printf("t=%d history tick %d differs from the reference\n", t,
                                t - int(ring.size()) + int(k));
                    CHECK(false);
                    break;
                }
            }
        } else {
            CHECK(false);
        }
        const Points in = PointsOf(entering.Evaluate(UsdTimeCode(t)), kCurtain);
        sag = std::max(sag, Furthest(got, in));
        // The pinned top row stands where it entered.
        for (int i = 0; i < kColumns && got.size() == in.size(); ++i)
            CHECK(std::memcmp(got[size_t(i)].data(), in[size_t(i)].data(),
                              sizeof(GfVec3f)) == 0);
        // Nothing passes into the box: its top face is at 15.5.
        const Points box = PointsOf(pose, kBox);
        for (const GfVec3f &p : got) {
            const float x = p[0] - (box.empty() ? 0.0f : box[0][0] + 8.0f);
            const bool over = std::fabs(x) < 7.5f && std::fabs(p[2]) < 7.5f;
            if (over) CHECK(p[1] > 15.5f - 1e-3f);
        }
    }
    // The cloth moves: it swings away from the points it entered with.
    CHECK(sag > 0.5f);
}

void
TestRandomAccessAgrees()
{
    UsdStageRefPtr stage = MakeStage();
    RigExecRigEvaluator sequential(stage, kRig);
    RigExecRigEvaluator jumping(stage, kRig);
    CHECK(Compile(sequential));
    CHECK(Compile(jumping));
    std::vector<Points> ordered(41);
    for (int t = 1; t <= 40; ++t)
        ordered[size_t(t)] = PointsOf(sequential.Evaluate(UsdTimeCode(t)), kCurtain);
    for (int t : {33, 9, 40, 21, 1, 22, 21, 14}) {
        CHECK(SameBits(PointsOf(jumping.Evaluate(UsdTimeCode(t)), kCurtain),
                       ordered[size_t(t)]));
    }
    // Sequential playback costs no history tick per frame.
    RigExecRigEvaluator playback(stage, kRig);
    CHECK(Compile(playback));
    playback.Evaluate(UsdTimeCode(10));
    const size_t first = playback.GetHistoryTickCountForTesting();
    for (int t = 11; t <= 16; ++t) playback.Evaluate(UsdTimeCode(t));
    CHECK(playback.GetHistoryTickCountForTesting() == first);
}

std::vector<RigExecValueOverride>
Drag(double tx)
{
    RigExecValueOverride drag;
    drag.prim = kRoot;
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(tx);
    return {drag};
}

void
TestDragReadsAuthoredHistory()
{
    UsdStageRefPtr stage = MakeStage();
    UsdStageRefPtr disabled = MakeStage(false);
    RigExecRigEvaluator evaluator(stage, kRig);
    RigExecRigEvaluator entering(disabled, kRig);
    CHECK(Compile(evaluator));
    CHECK(Compile(entering));
    RigExecClothPrepared prepared;
    std::string why;
    CHECK(RigExecPrepareCloth(StageEpoch(stage), &prepared, &why));
    const Points authored = PointsOf(evaluator.Evaluate(UsdTimeCode(14)), kCurtain);
    const auto drag = Drag(-6.0);
    evaluator.SetInteractiveOverrides(drag);
    const Points dragged = PointsOf(evaluator.Evaluate(UsdTimeCode(14)), kCurtain);
    CHECK(SameBits(dragged, Expected(entering, prepared, 14, &drag)));
    CHECK(Furthest(dragged, authored) > 1.0f);
    evaluator.ClearInteractiveOverrides();
    CHECK(SameBits(PointsOf(evaluator.Evaluate(UsdTimeCode(14)), kCurtain), authored));
}

void
TestFrozenJobsMatchLive(bool features = false)
{
    UsdStageRefPtr stage = features ? FeatureStage() : MakeStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    CHECK(Compile(evaluator));
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
    context.epochDigest = evaluator.GetBindingEpochDigest();
    context.slotCount = program->GetProviderCount();
    context.programDigest =
        context.epochDigest ^
        (uint64_t(program->GetBoundInputCount()) * 1099511628211ull) ^
        (uint64_t(program->GetVaryingInputCount()) * 16777619ull);
    context.frozen = frozen.get();
    const RigExecFrozenStepRunner runner = RigExecMakeProductionStepRunner();
    for (int t : {12, 31, 5}) {
        RigExecFrameInputs inputs;
        CHECK(RigExecSampleFrameInputs(evaluator, UsdTimeCode(t), {}, &inputs, &why));
        CHECK(inputs.windowRings.size() == 1);
        // A full history: every earlier frame a window may reach back to
        // captured the cloth's inputs.
        if (inputs.windowRings.size() == 1) {
            const RigExecWindowRing &ring = inputs.windowRings[0];
            const size_t history = size_t(RigExecClothHistoryFrames(RigExecClothSettings()));
            size_t held = 0;
            for (const auto &packet : ring) held += packet ? 1 : 0;
            if (ring.size() != history || held != history)
                std::printf("t=%d ring %zu packets, %zu held\n", t, ring.size(), held);
            CHECK(ring.size() == history && held == history);
        }
        context.varyingInputCount = inputs.values.size();
        const RigExecRigPose job = RigExecEvaluateFrozen(context, inputs, runner);
        CHECK(job.valid);
        const RigExecRigPose live = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(SameBits(PointsOf(job, kCurtain), PointsOf(live, kCurtain)));
    }
}

Points
ReaderPoints(const RigExecRuntimeReader &reader, const SdfPath &path)
{
    for (const RigExecRuntimePoints &points : reader.GetPoints()) {
        if (points.path != path.GetString()) continue;
        Points out(points.points.size());
        for (size_t i = 0; i < out.size(); ++i)
            out[i] = GfVec3f(points.points[i][0], points.points[i][1], points.points[i][2]);
        return out;
    }
    return {};
}

bool
SameAsNative(const RigExecRigPose &pose, const RigExecRuntimeReader &reader, int t)
{
    std::vector<std::string> diffs;
    const bool outputs = RigExecCompareRuntimeOutputs(pose, reader, &diffs);
    const Points native = PointsOf(pose, kCurtain);
    const Points played = ReaderPoints(reader, kCurtain);
    const bool same = SameBits(native, played);
    if (!outputs || !same) {
        std::printf("t=%d native and reader cloth differ by %g\n", t,
                    Furthest(native, played));
        for (const std::string &d : diffs) std::printf("  %s\n", d.c_str());
    }
    return outputs && same;
}

void
TestBinaryParity(bool features = false)
{
    UsdStageRefPtr stage = features ? FeatureStage() : MakeStage();
    RigExecRigEvaluator evaluator(stage, kRig);
    std::string why;
    CHECK(Compile(evaluator));
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
    // The runtime plays the cloth itself.
    CHECK(reader->GetMissingExternalKernels().empty());
    CHECK(reader->GetWindowedOperationCount() == 1);
    const size_t depth = reader->GetHistoryDepth();
    CHECK(depth == size_t(RigExecClothHistoryFrames(RigExecClothSettings())));
    RigExecInputSampler sampler;
    CHECK(sampler.Bind(stage, *reader, &why));
    for (int t = 1; t <= 40; ++t) {
        CHECK(RigExecTestDrive(reader.get(), &sampler, double(t), &why));
        if (size_t(t) > depth + 1)
            CHECK(SameAsNative(evaluator.Evaluate(UsdTimeCode(t)), *reader, t));
        reader->PushHistory();
    }
    for (int t : {30, 12, 37}) {
        reader->ClearHistory();
        for (size_t k = depth; k >= 1; --k) {
            CHECK(sampler.Apply(UsdTimeCode(double(t) - double(k)), reader.get(), &why));
            CHECK(reader->ExecuteHistory(&why));
        }
        CHECK(RigExecTestDrive(reader.get(), &sampler, double(t), &why));
        CHECK(SameAsNative(evaluator.Evaluate(UsdTimeCode(t)), *reader, t));
    }
    // A drag at the current frame.
    reader->ClearHistory();
    for (size_t k = depth; k >= 1; --k) {
        CHECK(sampler.Apply(UsdTimeCode(14.0 - double(k)), reader.get(), &why));
        CHECK(reader->ExecuteHistory(&why));
    }
    CHECK(sampler.Apply(UsdTimeCode(14), reader.get(), &why));
    CHECK(reader->SetInput(kRoot.GetString() + ".avars:tx", -6.0, &why));
    CHECK(reader->Execute(&why));
    evaluator.SetInteractiveOverrides(Drag(-6.0));
    CHECK(SameAsNative(evaluator.Evaluate(UsdTimeCode(14)), *reader, 14));
    evaluator.ClearInteractiveOverrides();
}

void
TestValidation()
{
    UsdStageRefPtr stage = MakeStage(true, "                uniform int rigExec:substeps = 0");
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(12));
    CHECK(!compiled || evaluator.GetSkippedOperations().count(kCloth) == 1 ||
          !pose.valid);
}

void
TestExampleSettlesDuringContact()
{
    UsdStageRefPtr stage = UsdStage::Open(RIGEXEC_CLOTH_EXAMPLE);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, SdfPath("/ClothAsset/Rig"));
    CHECK(Compile(evaluator));
    const SdfPath curtain("/ClothAsset/Geom/Curtain.points");
    Points previous, beforePrevious;
    double worst = 0.0;
    // The ball holds from 58 through 66. By 63, adjacent frames should
    // describe settling cloth, without jumps from changing its startup.
    // 0.35 is under one fifth of the example's two-unit edge length.
    for (int t = 61; t <= 66; ++t) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(pose.valid);
        const Points points = PointsOf(pose, curtain);
        CHECK(points.size() == 17 * 17);
        if (points.size() != 17 * 17) return;
        if (!beforePrevious.empty()) {
            double sum = 0.0;
            for (size_t i = 17; i < points.size(); ++i) {
                const GfVec3f delta = points[i] - 2.0f * previous[i] + beforePrevious[i];
                sum += double(delta.GetLengthSq());
            }
            worst = std::max(worst, std::sqrt(sum / double(points.size() - 17)));
        }
        beforePrevious = previous;
        previous = points;
    }
    std::printf("cloth example held-contact RMS second difference: %g\n", worst);
    CHECK(worst < 0.35);
}

}  // namespace

int
main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestMatchesKernel();
    TestRandomAccessAgrees();
    TestDragReadsAuthoredHistory();
    TestFrozenJobsMatchLive();
    TestBinaryParity();
    TestFrozenJobsMatchLive(true);
    TestBinaryParity(true);
    TestPaintAndRestInvalidation();
    TestBindingAndMapTransfer();
    TestValidation();
    TestExampleSettlesDuringContact();
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecCloth: all tests passed\n");
    return 0;
}
