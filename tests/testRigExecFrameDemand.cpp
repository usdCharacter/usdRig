// testRigExecFrameDemand: frames on demand (docs/specs/frames-on-demand.md).
// A rig output sampled through Hydra at a time offset answers the frame at
// the displayed time + offset from the context's frame table, and never
// evaluates. The contract, one group per test:
//   * TABLE. Keys are exact to 1e-6 frames; demand is de-duplicated and
//     capped; Insert keeps at most kMaxEntries, dropping the entry farthest
//     from the playhead; ResetIfStale drops entries only on a new stamp.
//   * NOT YET, THEN READY. On examples/ArmRig.usda, a points read at
//     offset +3 from frame 1 is an empty VtValue and records frame 4 as
//     demand (a typed read answers the displayed value). ServiceFrameDemand
//     evaluates nothing and leaves the displayed generation alone; once the
//     background workers have warmed frame 4, it becomes an entry and the
//     same data source answers exactly what SetTime(4) publishes.
//   * PREVIEW. While an interactive preview stands, demand waits; after
//     EndPreview it is served.
//   * EDITS. An authored edit makes every entry stale: the next SetTime
//     drops the table, and the frame answers the edited pose once warmed.
//   * NOTICES. NotifyFramesChanged dirties the rig's published value leaves.
//   * PARITY. On a 2D cut-out rig (a skinned stroke, a layer bound to a bone,
//     an animated opacity map, a stepped switch index), every value leaf read
//     at offset k from frame 1 equals what offset 0 answers after SetTime(1 +
//     k), bit for bit: points, the driven world transform of the bound group
//     and of the prim riding it, opacity and switchIndex; an undriven prim's
//     points are the authored ones at every offset.
//   * PLAYBACK. A .rigexec playback answers other frames by playing its
//     binary on the owner thread and puts its displayed generation back.
#include "rigExecImaging/frameTable.h"
#include "rigExecImaging/registry.h"
#include "rigExecImaging/sceneIndices.h"
#include "rigExecImaging/snapshotStore.h"
#include "rigExecBake/bake.h"
#include "rigExec/backgroundScheduler.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/retainedSceneIndex.h"
#include "pxr/imaging/hd/sceneIndexObserver.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/basisCurves.h"
#include "pxr/usd/usdGeom/xform.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
static int skips = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            ++failures;                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                                \
    } while (0)

namespace {

void
SetEnv(const char *name, const char *value)
{
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}

void
Skip(const char *what)
{
    ++skips;
    std::printf("SKIP %s\n", what);
}

bool
SameArrays(const VtVec3fArray &a, const VtVec3fArray &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------

void
TestTable()
{
    std::printf("progress: TestTable\n");
    CHECK(RigExecFrameTable::Key(4.0) == RigExecFrameTable::Key(4.0 + 1e-8));
    CHECK(RigExecFrameTable::Key(4.0) != RigExecFrameTable::Key(4.00001));
    // 1.0f + 3.0f offsets read back the owner's key.
    CHECK(RigExecFrameTable::Key(1.0 + double(3.0f)) ==
          RigExecFrameTable::Key(4.0));

    RigExecFrameTable table;
    CHECK(!table.HasDemand());
    table.Demand(4.0);
    table.Demand(4.0 + 1e-9);
    table.Demand(std::nan(""));
    CHECK(table.HasDemand());
    std::vector<double> demand = table.TakeDemand();
    CHECK(demand.size() == 1 && demand[0] == 4.0);
    CHECK(!table.HasDemand());
    for (size_t i = 0; i < RigExecFrameTable::kMaxDemand + 10; ++i) {
        table.Demand(double(i));
    }
    demand = table.TakeDemand();
    CHECK(demand.size() == RigExecFrameTable::kMaxDemand);
    CHECK(!demand.empty() && demand.front() == 10.0);

    auto entry = std::make_shared<RigExecImagingSnapshot>();
    CHECK(!table.ResetIfStale(7));   // first stamp: nothing to drop
    table.Insert(4.0, entry, 1.0);
    CHECK(table.Find(4.0) == entry && table.Contains(4.0));
    CHECK(!table.Find(5.0));
    CHECK(!table.ResetIfStale(7));
    CHECK(table.Find(4.0) == entry);
    CHECK(table.ResetIfStale(8));
    CHECK(!table.Find(4.0));

    for (size_t i = 0; i < RigExecFrameTable::kMaxEntries + 5; ++i) {
        table.Insert(double(i), entry, 0.0);
    }
    CHECK(table.EntryCountForTesting() == RigExecFrameTable::kMaxEntries);
    CHECK(table.Contains(0.0));
    CHECK(!table.Contains(double(RigExecFrameTable::kMaxEntries + 4)));
    table.Clear();
    CHECK(table.EntryCountForTesting() == 0 && !table.HasDemand());
}

// ---------------------------------------------------------------------------
// ArmRig through a results scene index.

const SdfPath kArmRig("/ArmAsset/Rig");
const SdfPath kArmBody("/ArmAsset/Geom/ArmBody");
const char *kShoulderRz = "/ArmAsset/Rig/Controls/ShoulderFK.avars:rz";

HdSampledDataSourceHandle
PointsSource(const HdSceneIndexBaseRefPtr &index, const SdfPath &path)
{
    const HdSceneIndexPrim prim = index->GetPrim(path);
    HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(prim.dataSource);
    HdPrimvarSchema points = primvars.GetPrimvar(HdTokens->points);
    return points ? points.GetPrimvarValue() : nullptr;
}

VtVec3fArray
Published(const RigExecImagingRegistry::Ptr &registry, const SdfPath &prim)
{
    const auto snapshot = registry->GetStore()->Get();
    if (!snapshot) return VtVec3fArray();
    const auto it = snapshot->prims.find(prim);
    return it == snapshot->prims.end() ? VtVec3fArray() : it->second.points;
}

// Services demand and drives the background workers until \p source
// answers \p offset, or gives up.
VtValue
WaitForFrame(const RigExecImagingRegistry::Ptr &registry,
             const HdSampledDataSourceHandle &source, float offset)
{
    for (int round = 0; round < 60; ++round) {
        const VtValue value = source->GetValue(offset);
        if (!value.IsEmpty()) {
            return value;
        }
        registry->ServiceFrameDemand();
        registry->OnIdle();
        registry->WaitUntilBackgroundIdle();
    }
    return source->GetValue(offset);
}

struct DirtyRecorder : public HdSceneIndexObserver {
    std::vector<HdSceneIndexObserver::DirtiedPrimEntry> dirtied;
    void PrimsAdded(const HdSceneIndexBase &, const AddedPrimEntries &) override {}
    void PrimsRemoved(const HdSceneIndexBase &, const RemovedPrimEntries &) override {}
    void PrimsDirtied(const HdSceneIndexBase &,
                      const DirtiedPrimEntries &entries) override {
        dirtied.insert(dirtied.end(), entries.begin(), entries.end());
    }
    void PrimsRenamed(const HdSceneIndexBase &, const RenamedPrimEntries &) override {}
};

void
TestArmRig(const std::string &examples)
{
    std::printf("progress: TestArmRig\n");
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    SetEnv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    if (!RigExecBackgroundWarmingEnabled()) {
        Skip("ArmRig: background warming is off in this run");
        return;
    }
    const UsdStageRefPtr stage = UsdStage::Open(examples + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdAttribute shoulder = stage->GetAttributeAtPath(SdfPath(kShoulderRz));
    const UsdAttribute elbow = stage->GetAttributeAtPath(
        SdfPath("/ArmAsset/Rig/Controls/ElbowFK.avars:rz"));
    CHECK(shoulder && elbow);
    for (int t = 1; t <= 8; ++t) {
        shoulder.Set(6.0 * t, UsdTimeCode(t));
        elbow.Set(3.0 - 9.0 * t, UsdTimeCode(t));
    }

    const RigExecImagingRegistry::Ptr registry =
        RigExecImagingRegistry::ForStage(stage, /* create = */ true);
    std::vector<std::string> errors;
    CHECK(registry->Activate(stage, kArmRig, UsdTimeCode(1.0), &errors));
    for (const std::string &error : errors) {
        std::printf("activation: %s\n", error.c_str());
    }
    if (!registry->IsActive()) return;

    // The reference: what playback publishes frame by frame.
    std::map<int, VtVec3fArray> body;
    for (int t = 1; t <= 6; ++t) {
        CHECK(registry->SetTime(UsdTimeCode(t)));
        body[t] = Published(registry, kArmBody);
        CHECK(!body[t].empty());
    }
    CHECK(!SameArrays(body[1], body[4]));
    CHECK(registry->SetTime(UsdTimeCode(1.0)));
    registry->ClearFrameCache(kArmRig);
    registry->WaitUntilBackgroundIdle();

    // A results scene index over a plain upstream: the published prim and
    // its ancestors as empty shells.
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims({
        {SdfPath("/ArmAsset"), TfToken(), HdRetainedContainerDataSource::New()},
        {SdfPath("/ArmAsset/Geom"), TfToken(), HdRetainedContainerDataSource::New()},
        {kArmBody, HdPrimTypeTokens->mesh, HdRetainedContainerDataSource::New()}});
    const RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(upstream, registry->GetStore());

    const HdSampledDataSourceHandle points = PointsSource(results, kArmBody);
    CHECK(points);
    if (!points) return;
    const VtValue now = points->GetValue(0.0f);
    CHECK(now.IsHolding<VtVec3fArray>() &&
          SameArrays(now.UncheckedGet<VtVec3fArray>(), body[1]));

    // Not yet: an empty value, the demand recorded, a typed read answers
    // the displayed value.
    const auto table = registry->GetFrameTableForTesting();
    CHECK(table);
    const auto generation = registry->GetStore()->Get()->generation;
    const size_t evaluations = registry->GetSessionEvaluationCount(kArmRig);
    CHECK(points->GetValue(3.0f).IsEmpty());
    CHECK(registry->HasFrameDemand());
    const auto typedPoints =
        HdTypedSampledDataSource<VtVec3fArray>::Cast(points);
    CHECK(typedPoints && SameArrays(typedPoints->GetTypedValue(3.0f), body[1]));

    // Servicing alone evaluates nothing and leaves the displayed pose.
    CHECK(registry->ServiceFrameDemand() == 0);
    CHECK(registry->GetSessionEvaluationCount(kArmRig) == evaluations);
    CHECK(registry->GetStore()->Get()->generation == generation);
    CHECK(points->GetValue(3.0f).IsEmpty());
    CHECK(registry->HasFrameDemand());   // pending until warmed

    // Warmed by the background workers, then an entry: exactly SetTime(4).
    const VtValue ready = WaitForFrame(registry, points, 3.0f);
    CHECK(ready.IsHolding<VtVec3fArray>());
    if (ready.IsHolding<VtVec3fArray>()) {
        CHECK(SameArrays(ready.UncheckedGet<VtVec3fArray>(), body[4]));
    }
    CHECK(registry->GetSessionEvaluationCount(kArmRig) == evaluations);
    CHECK(registry->GetStore()->Get()->generation == generation);
    CHECK(SameArrays(typedPoints->GetTypedValue(3.0f), body[4]));
    CHECK(table->Contains(4.0));

    // A later displayed time reads the same frame at its own offset.
    CHECK(registry->SetTime(UsdTimeCode(2.0)));
    const HdSampledDataSourceHandle points2 = PointsSource(results, kArmBody);
    const VtValue fromTwo = points2->GetValue(2.0f);
    CHECK(fromTwo.IsHolding<VtVec3fArray>() &&
          SameArrays(fromTwo.UncheckedGet<VtVec3fArray>(), body[4]));
    CHECK(registry->SetTime(UsdTimeCode(1.0)));

    // A preview stands: demand waits for it to end.
    CHECK(registry->BeginPreview(kShoulderRz) == 1);
    const double dragged = 70.0;
    CHECK(registry->UpdatePreview(&dragged, 1));
    const HdSampledDataSourceHandle previewed = PointsSource(results, kArmBody);
    CHECK(previewed->GetValue(4.0f).IsEmpty());   // frame 5
    for (int round = 0; round < 3; ++round) {
        CHECK(registry->ServiceFrameDemand() == 0);
    }
    CHECK(!table->Contains(5.0));
    // Frames from before the drag still answer.
    const VtValue held = previewed->GetValue(3.0f);
    CHECK(held.IsHolding<VtVec3fArray>() &&
          SameArrays(held.UncheckedGet<VtVec3fArray>(), body[4]));
    CHECK(registry->EndPreview());
    const HdSampledDataSourceHandle released = PointsSource(results, kArmBody);
    const VtValue five = WaitForFrame(registry, released, 4.0f);
    CHECK(five.IsHolding<VtVec3fArray>() &&
          SameArrays(five.UncheckedGet<VtVec3fArray>(), body[5]));

    // Notices: an observed index dirties the published value leaves.
    DirtyRecorder recorder;
    results->AddObserver(HdSceneIndexObserverPtr(&recorder));
    results->NotifyFramesChanged();
    bool sawBody = false;
    for (const auto &entry : recorder.dirtied) {
        if (entry.primPath == kArmBody &&
            entry.dirtyLocators.Intersects(HdPrimvarsSchema::GetPointsLocator())) {
            sawBody = true;
        }
    }
    CHECK(sawBody);
    results->RemoveObserver(HdSceneIndexObserverPtr(&recorder));

    // An authored edit makes every other frame stale.
    shoulder.Set(-40.0, UsdTimeCode(4));
    CHECK(registry->SetTime(UsdTimeCode(4.0)));
    const VtVec3fArray edited = Published(registry, kArmBody);
    CHECK(!SameArrays(edited, body[4]));
    CHECK(registry->SetTime(UsdTimeCode(1.0)));
    CHECK(!table->Contains(4.0));
    const HdSampledDataSourceHandle after = PointsSource(results, kArmBody);
    const VtValue rewarmed = WaitForFrame(registry, after, 3.0f);
    CHECK(rewarmed.IsHolding<VtVec3fArray>() &&
          SameArrays(rewarmed.UncheckedGet<VtVec3fArray>(), edited));

    registry->Deactivate();
}

// A rig that never warms in the background (warming off) still answers:
// after kFrameDemandFallbackAge idle services, one evaluation on the idle
// path; never on the playback path (ServiceFrameDemand's default).
void
TestFallback(const std::string &examples)
{
    std::printf("progress: TestFallback\n");
    SetEnv("RIGEXEC_FRAME_CACHE", "warm-off");
    const UsdStageRefPtr stage = UsdStage::Open(examples + "/ArmRig.usda");
    CHECK(stage);
    if (!stage) return;
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdAttribute shoulder = stage->GetAttributeAtPath(SdfPath(kShoulderRz));
    for (int t = 1; t <= 8; ++t) shoulder.Set(5.0 * t, UsdTimeCode(t));
    const RigExecImagingRegistry::Ptr registry =
        RigExecImagingRegistry::ForStage(stage, /* create = */ true);
    std::vector<std::string> errors;
    CHECK(registry->Activate(stage, kArmRig, UsdTimeCode(1.0), &errors));
    if (!registry->IsActive()) {
        SetEnv("RIGEXEC_FRAME_CACHE", "on");
        return;
    }
    CHECK(registry->SetTime(UsdTimeCode(3.0)));
    const VtVec3fArray three = Published(registry, kArmBody);
    CHECK(registry->SetTime(UsdTimeCode(1.0)));
    registry->ClearFrameCache(kArmRig);

    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    upstream->AddPrims({
        {SdfPath("/ArmAsset"), TfToken(), HdRetainedContainerDataSource::New()},
        {SdfPath("/ArmAsset/Geom"), TfToken(), HdRetainedContainerDataSource::New()},
        {kArmBody, HdPrimTypeTokens->mesh, HdRetainedContainerDataSource::New()}});
    const RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(upstream, registry->GetStore());
    const HdSampledDataSourceHandle points = PointsSource(results, kArmBody);
    CHECK(points && points->GetValue(2.0f).IsEmpty());
    const size_t evaluations = registry->GetSessionEvaluationCount(kArmRig);

    // The playback path never evaluates, however long the frame waits.
    for (int i = 0; i < RigExecImagingRegistry::kFrameDemandFallbackAge + 5; ++i) {
        CHECK(registry->ServiceFrameDemand() == 0);
    }
    CHECK(points->GetValue(2.0f).IsEmpty());
    CHECK(registry->GetSessionEvaluationCount(kArmRig) == evaluations);

    // The idle path answers it once the frame has waited long enough.
    size_t rounds = 0;
    while (points->GetValue(2.0f).IsEmpty() &&
           rounds < size_t(RigExecImagingRegistry::kFrameDemandFallbackAge) + 5) {
        registry->ServiceFrameDemand(8, /* allowFallback = */ true);
        ++rounds;
    }
    const VtValue answered = points->GetValue(2.0f);
    CHECK(answered.IsHolding<VtVec3fArray>() &&
          SameArrays(answered.UncheckedGet<VtVec3fArray>(), three));
    CHECK(registry->GetSessionEvaluationCount(kArmRig) == evaluations + 1);
    registry->Deactivate();
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
}

// ---------------------------------------------------------------------------
// PARITY on a 2D cut-out rig, and PLAYBACK.

const SdfPath k2dRig("/Char/Rig");
const SdfPath kB1("/Char/Rig/Bones/B1");
const SdfPath kB2("/Char/Rig/Bones/B2");
const SdfPath kStroke("/Char/Art/Stroke");
const SdfPath kHead("/Char/Art/Head");
const SdfPath kFace("/Char/Art/Head/Face");
const SdfPath kStatic("/Char/Art/Static");
constexpr size_t kStrokePoints = 6;

GfMatrix4d
Translate(double x, double y)
{
    GfMatrix4d m(1.0);
    m.SetTranslate(GfVec3d(x, y, 0.0));
    return m;
}

UsdPrim
DefineMover(const UsdStageRefPtr &stage, const SdfPath &path,
            const char *type, const SdfPath &moves)
{
    UsdPrim prim = stage->DefinePrim(path, TfToken(type));
    prim.ApplyAPI(TfToken("RigExecMoverAPI"));
    prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({moves});
    return prim;
}

UsdStageRefPtr
Make2dRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdGeomXform character = UsdGeomXform::Define(stage, SdfPath("/Char"));
    character.AddTranslateOp().Set(GfVec3d(5.0, 2.0, 0.0));
    stage->DefinePrim(k2dRig, TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Char/Rig/Bones"), TfToken("Scope"));
    const UsdPrim b1 = stage->DefinePrim(kB1, TfToken("RigExecControl"));
    const UsdPrim b2 = stage->DefinePrim(kB2, TfToken("RigExecControl"));
    b1.GetAttribute(TfToken("rest:space")).Set(Translate(-0.5, -0.2));
    b2.GetAttribute(TfToken("rest:space")).Set(Translate(0.5, 0.0));
    const UsdAttribute b1rz = b1.GetAttribute(TfToken("avars:rz"));
    const UsdAttribute b2rz = b2.GetAttribute(TfToken("avars:rz"));
    const UsdAttribute b2tx = b2.GetAttribute(TfToken("avars:tx"));
    for (int t = 1; t <= 4; ++t) {
        b1rz.Set(30.0 * (t - 1), UsdTimeCode(t));
        b2rz.Set(-15.0 * (t - 1), UsdTimeCode(t));
        b2tx.Set(0.125 * (t - 1), UsdTimeCode(t));
    }

    UsdGeomXform::Define(stage, SdfPath("/Char/Art"));
    UsdGeomBasisCurves stroke = UsdGeomBasisCurves::Define(stage, kStroke);
    VtVec3fArray points(kStrokePoints);
    for (size_t i = 0; i < kStrokePoints; ++i) {
        points[i] = GfVec3f(-0.5f + 0.25f * float(i), 0.0625f * float(i), 0.0f);
    }
    stroke.CreatePointsAttr().Set(points);
    stroke.CreateCurveVertexCountsAttr().Set(VtIntArray{int(kStrokePoints)});
    stroke.CreateTypeAttr().Set(TfToken("linear"));
    UsdGeomXform head = UsdGeomXform::Define(stage, kHead);
    head.AddTranslateOp().Set(GfVec3d(0.5, 0.5, 0.0));
    head.GetPrim()
        .CreateAttribute(TfToken("primvars:switchIndex"),
                         SdfValueTypeNames->Float)
        .Set(0.0f);
    UsdGeomBasisCurves face = UsdGeomBasisCurves::Define(stage, kFace);
    face.CreatePointsAttr().Set(VtVec3fArray{
        GfVec3f(0, 0, 0), GfVec3f(0.1f, 0, 0), GfVec3f(0.1f, 0.1f, 0)});
    face.CreateCurveVertexCountsAttr().Set(VtIntArray{3});
    UsdGeomBasisCurves still = UsdGeomBasisCurves::Define(stage, kStatic);
    still.CreatePointsAttr().Set(VtVec3fArray{GfVec3f(1, 1, 0),
                                              GfVec3f(2, 1, 0)});
    still.CreateCurveVertexCountsAttr().Set(VtIntArray{2});

    stage->DefinePrim(SdfPath("/Char/Rig/Movers"), TfToken("Scope"));
    UsdPrim skin = DefineMover(stage, SdfPath("/Char/Rig/Movers/StrokeSkin"),
                               "RigExecSkinMover",
                               kStroke.AppendProperty(TfToken("points")));
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({kB1, kB2});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(2);
    VtIntArray indices(kStrokePoints * 2);
    VtFloatArray weights(kStrokePoints * 2);
    for (size_t i = 0; i < kStrokePoints; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
        const float w = float(i) / float(kStrokePoints - 1);
        weights[i * 2] = 1.0f - w;
        weights[i * 2 + 1] = w;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(indices);
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray).Set(weights);

    // Layer bind: the group's frame rides B2's rest->posed map.
    UsdPrim bind = DefineMover(stage, SdfPath("/Char/Rig/Movers/HeadBind"),
                               "RigExecMatrixMover", kHead);
    bind.CreateRelationship(TfToken("rigExec:transform")).SetTargets({kB2});

    // A stepped switch index on the group: a property chain on a float
    // primvar, published from the compiled scalar list.
    UsdPrim swap = DefineMover(
        stage, SdfPath("/Char/Rig/Movers/Switch"), "RigExecFloatMathMover",
        kHead.AppendProperty(TfToken("primvars:switchIndex")));
    swap.CreateAttribute(TfToken("rigExec:operation"), SdfValueTypeNames->Token,
                         /* custom = */ false, SdfVariabilityUniform)
        .Set(TfToken("add"));
    const UsdAttribute switchValue = swap.CreateAttribute(
        TfToken("inputs:value"), SdfValueTypeNames->Float);
    const float switches[4] = {0.0f, 1.0f, 2.0f, 2.0f};
    for (int t = 1; t <= 4; ++t) {
        switchValue.Set(switches[t - 1], UsdTimeCode(t));
    }

    // The group's opacity follows B1's angle: rz / 90, clamped.
    stage->DefinePrim(SdfPath("/Char/Rig/AnimatedMaps"), TfToken("Scope"));
    UsdPrim maps = stage->DefinePrim(SdfPath("/Char/Rig/AnimatedMaps/Fade"),
                                     TfToken("RigExecAnimatedMaps"));
    maps.CreateRelationship(TfToken("rigExec:driverAttributes"))
        .SetTargets({kB1.AppendProperty(TfToken("avars:rz"))});
    maps.CreateRelationship(TfToken("rigExec:meshes")).SetTargets({kHead});
    UsdPrim map = stage->DefinePrim(
        SdfPath("/Char/Rig/AnimatedMaps/Fade/opacity"),
        TfToken("RigExecAnimatedMap"));
    map.GetAttribute(TfToken("rigExec:rowInputs")).Set(VtIntArray{0});
    map.GetAttribute(TfToken("rigExec:rowFrom")).Set(VtFloatArray{0.0f});
    map.GetAttribute(TfToken("rigExec:rowTo")).Set(VtFloatArray{90.0f});
    map.GetAttribute(TfToken("rigExec:rowSlope"))
        .Set(VtFloatArray{1.0f / 90.0f});
    map.GetAttribute(TfToken("rigExec:rowCut")).Set(VtFloatArray{0.0f});
    return stage;
}

// The value leaves of a published prim at an offset: points, driven world
// transform, the two scalar primvars. Empty values where the prim has none.
struct Leaves {
    VtValue points, xform, opacity, switchIndex;
};

Leaves
Read(const HdSceneIndexBaseRefPtr &index, const SdfPath &path, float offset)
{
    Leaves out;
    const HdSceneIndexPrim prim = index->GetPrim(path);
    const HdPrimvarsSchema primvars = HdPrimvarsSchema::GetFromParent(prim.dataSource);
    const auto value = [&](const TfToken &name) {
        const HdPrimvarSchema primvar = primvars.GetPrimvar(name);
        const HdSampledDataSourceHandle ds = primvar ? primvar.GetPrimvarValue() : nullptr;
        return ds ? ds->GetValue(offset) : VtValue();
    };
    out.points = value(HdTokens->points);
    out.opacity = value(TfToken("opacity"));
    out.switchIndex = value(TfToken("switchIndex"));
    if (const HdMatrixDataSourceHandle m =
            HdXformSchema::GetFromParent(prim.dataSource).GetMatrix()) {
        out.xform = m->GetValue(offset);
    }
    return out;
}

bool
Ready(const Leaves &leaves, const Leaves &reference)
{
    // Every leaf the reference has is answered.
    return (reference.points.IsEmpty() || !leaves.points.IsEmpty()) &&
           (reference.xform.IsEmpty() || !leaves.xform.IsEmpty()) &&
           (reference.opacity.IsEmpty() || !leaves.opacity.IsEmpty()) &&
           (reference.switchIndex.IsEmpty() || !leaves.switchIndex.IsEmpty());
}

// The published prims and their ancestors as upstream shells, with the
// stage's world transforms (static in this fixture) as flattening gives them.
HdSceneIndexBaseRefPtr
Upstream(const UsdStageRefPtr &stage, const std::vector<SdfPath> &prims)
{
    HdRetainedSceneIndexRefPtr upstream = HdRetainedSceneIndex::New();
    std::set<SdfPath> all;
    for (const SdfPath &p : prims)
        for (SdfPath q = p; q != SdfPath::AbsoluteRootPath(); q = q.GetParentPath()) all.insert(q);
    UsdGeomXformCache cache{UsdTimeCode(1.0)};
    HdRetainedSceneIndex::AddedPrimEntries entries;
    for (const SdfPath &p : all) {
        const GfMatrix4d world = cache.GetLocalToWorldTransform(stage->GetPrimAtPath(p));
        static const TfToken xformName = HdXformSchemaTokens->xform;
        const HdDataSourceBaseHandle xform =
            HdXformSchema::Builder()
                .SetMatrix(HdRetainedTypedSampledDataSource<GfMatrix4d>::New(world))
                .SetResetXformStack(HdRetainedTypedSampledDataSource<bool>::New(true))
                .Build();
        const UsdPrim prim = stage->GetPrimAtPath(p);
        const TfToken type = prim.IsA<UsdGeomBasisCurves>() ? HdPrimTypeTokens->basisCurves : TfToken();
        entries.push_back({p, type, HdRetainedContainerDataSource::New(1, &xformName, &xform)});
    }
    upstream->AddPrims(entries);
    return upstream;
}

// Offset k from frame 1 equals offset 0 after SetTime(1 + k), for every
// leaf of every prim.
void
CheckParity(const RigExecImagingRegistry::Ptr &registry, const UsdStageRefPtr &stage,
            const char *what, bool checkValues = true)
{
    const std::vector<SdfPath> prims = {kStroke, kHead, kFace, kStatic};
    const RigExecResultsSceneIndexRefPtr results =
        RigExecResultsSceneIndex::New(Upstream(stage, prims), registry->GetStore());
    std::map<int, std::map<SdfPath, Leaves>> reference;
    for (int t = 1; t <= 4; ++t) {
        CHECK(registry->SetTime(UsdTimeCode(t)));
        for (const SdfPath &p : prims) reference[t][p] = Read(results, p, 0.0f);
    }
    CHECK(!reference[1][kStroke].points.IsEmpty() && !reference[1][kHead].xform.IsEmpty());
    CHECK(reference[1][kStroke].points != reference[3][kStroke].points);
    CHECK(reference[1][kHead].xform != reference[3][kHead].xform);
    CHECK(registry->SetTime(UsdTimeCode(1.0)));
    const uint64_t generation = registry->GetStore()->Get()->generation;
    for (int t = 2; t <= 4; ++t) {
        const float offset = float(t - 1);
        for (int round = 0; round < 60; ++round) {
            bool ready = true;
            for (const SdfPath &p : prims)
                ready = ready && Ready(Read(results, p, offset), reference[t][p]);
            if (ready) break;
            registry->ServiceFrameDemand();
            registry->OnIdle();
            registry->WaitUntilBackgroundIdle();
        }
        for (const SdfPath &p : prims) {
            const Leaves got = Read(results, p, offset);
            const Leaves &want = reference[t][p];
            const bool same = got.points == want.points && got.xform == want.xform &&
                              got.opacity == want.opacity && got.switchIndex == want.switchIndex;
            if (!same) {
                std::printf("parity [%s] %s at frame %d differs\n", what, p.GetText(), t);
            }
            CHECK(same);
        }
    }
    CHECK(registry->GetStore()->Get()->generation == generation);
    if (!checkValues) return;  // a baked binary publishes no animated-map scalars
    const VtValue opacity = Read(results, kHead, 2.0f).opacity;  // frame 3: rz 60 / 90
    CHECK(opacity.IsHolding<float>() &&
          std::abs(opacity.UncheckedGet<float>() - 60.0f / 90.0f) < 1e-6f);
    const VtValue switchIndex = Read(results, kHead, 2.0f).switchIndex;
    CHECK(switchIndex.IsHolding<float>() && switchIndex.UncheckedGet<float>() == 2.0f);
}

void
TestParity2d()
{
    std::printf("progress: TestParity2d\n");
    SetEnv("RIGEXEC_FRAME_CACHE", "on");
    if (!RigExecBackgroundWarmingEnabled()) {
        Skip("Parity2d: background warming is off in this run");
        return;
    }
    const UsdStageRefPtr stage = Make2dRig();
    const RigExecImagingRegistry::Ptr registry =
        RigExecImagingRegistry::ForStage(stage, /* create = */ true);
    std::vector<std::string> errors;
    CHECK(registry->Activate(stage, k2dRig, UsdTimeCode(1.0), &errors));
    for (const std::string &error : errors) std::printf("activation: %s\n", error.c_str());
    if (!registry->IsActive()) return;
    CheckParity(registry, stage, "live");
    registry->Deactivate();
}

void
TestPlayback()
{
    std::printf("progress: TestPlayback\n");
    const UsdStageRefPtr stage = Make2dRig();
    const std::filesystem::path scratch =
        std::filesystem::temp_directory_path() / "testRigExecFrameDemand";
    std::filesystem::create_directories(scratch);
    const std::string binary = (scratch / "frameDemand2d.rigexec").string();
    {
        RigExecRigEvaluator evaluator(stage, k2dRig);
        RigExecBakeOpts opts;
        opts.time = 1.0;
        RigExecBakeResult baked;
        std::string error;
        const bool ok = RigExecBakeToBinary(evaluator, opts, &baked, &error);
        if (!ok) std::printf("bake: %s\n", error.c_str());
        CHECK(ok && !baked.bytes.empty());
        if (!ok || baked.bytes.empty()) return;
        std::ofstream stream(binary, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(baked.bytes.data()),
                     std::streamsize(baked.bytes.size()));
    }
    stage->GetPrimAtPath(k2dRig)
        .CreateAttribute(TfToken("rigExec:asset"), SdfValueTypeNames->Asset)
        .Set(SdfAssetPath(binary));
    const RigExecImagingRegistry::Ptr registry =
        RigExecImagingRegistry::ForStage(stage, /* create = */ true);
    std::vector<std::string> errors;
    CHECK(registry->Activate(stage, k2dRig, UsdTimeCode(1.0), &errors));
    CHECK(registry->GetPlayback(k2dRig) != nullptr);
    if (registry->IsActive() && registry->GetPlayback(k2dRig)) {
        CheckParity(registry, stage, "playback", /* checkValues = */ false);
        // The displayed generation was put back: it plays on from frame 1.
        CHECK(registry->SetTime(UsdTimeCode(2.0)));
    }
    registry->Deactivate();
    std::error_code ignored;
    std::filesystem::remove_all(scratch, ignored);
}

}  // namespace

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const std::string examples =
        argc > 1 ? std::string(argv[1]) : std::string(RIGEXEC_EXAMPLES_DIR);
    TestTable();
    TestArmRig(examples);
    TestFallback(examples);
    TestParity2d();
    TestPlayback();
    std::printf("testRigExecFrameDemand: %d failure(s), %d skip(s)\n",
                failures, skips);
    return failures == 0 ? 0 : 1;
}
