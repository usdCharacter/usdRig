// Deterministic properties of the production EvalCore executor, driven
// only through the two public front ends: the baked program
// (RigExecRigEvaluator::Evaluate) and .rigexec playback
// (RigExecRuntimeReader through RigExecTestPlayer). No second interpreter,
// no engine-identity selection: every comparison is between results the
// production paths published, so a front end that silently switched
// implementations would fail here.
//
// Properties, for every case below:
//   partitioning   byte-identical points under every valid partitioning
//                  of the same rig (chunk/group caps, fine grain, serial
//                  schedule), on both front ends;
//   dense/sparse   a dense envelope and the eligible sparse walk of the
//                  same field publish the same bytes, and sparse input
//                  sampling disabled publishes the sparse program's bytes;
//   memo/recompute a repeated evaluation at one time (the memo hit)
//                  publishes a fresh evaluation's bytes (the recompute),
//                  on both front ends and through the frozen worker;
//   retained       a snapshot retained across an edit still holds its
//                  original bytes after the edited run, and releasing
//                  the edit republishes the retained bytes.
//
// Cases: the biped body skin (elementSize 10, as authored in
// examples/biped/Biped_body.usda), one HarmonicsChain case from the
// parity snapshot (examples/19_HarmonicsChain.usda at the snapshot's
// frames 1060 and 1064), a synthetic classicLinear skin at elementSize
// 10, and a synthetic matrix mover under dense and sparse envelopes.
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExecSchema/schemaTokens.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdSkel/tokens.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static const TfToken _dense(SchemaValueText::dense);
static const TfToken _inputsDefaultWeight(SchemaAttrText::inputsDefaultWeight);
static const TfToken
    _rigExecDefaultWeight(SchemaAttrText::rigExecDefaultWeight);
static const TfToken _rigExecElementSize(SchemaAttrText::rigExecElementSize);
static const TfToken _rigExecIndices(SchemaAttrText::rigExecIndices);
static const TfToken _rigExecInfluences(SchemaAttrText::rigExecInfluences);
static const TfToken _rigExecJointIndices(SchemaAttrText::rigExecJointIndices);
static const TfToken _rigExecJointWeights(SchemaAttrText::rigExecJointWeights);
static const TfToken _rigExecMoves(SchemaAttrText::rigExecMoves);
static const TfToken
    _rigExecRepresentation(SchemaAttrText::rigExecRepresentation);
static const TfToken
    _rigExecSkinningMethod(SchemaAttrText::rigExecSkinningMethod);
static const TfToken _rigExecTransform(SchemaAttrText::rigExecTransform);
static const TfToken _rigExecValues(SchemaAttrText::rigExecValues);
static const TfToken _rigExecWeightObject(SchemaAttrText::rigExecWeightObject);
static const TfToken _rigExecWeightTarget(SchemaAttrText::rigExecWeightTarget);
static const TfToken _sparse(SchemaValueText::sparse);

static int failures = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                      \
    } while (0)

#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"

static std::string _GetEnv(const char *name);
static void _SetEnv(const char *name, const std::string &value);

// Holds an environment knob for a scope in both environments Build and
// Open read (TfGetenv and std::getenv), restoring it on exit.
struct _ScopedEnv {
    _ScopedEnv(const char *name, const char *value)
        : _name(name), _old(TfGetenv(name)), _oldCrt(_GetEnv(name))
    {
        TfSetenv(_name, value);
        _SetEnv(name, value);
    }
    ~_ScopedEnv()
    {
        if (_old.empty()) {
            TfUnsetenv(_name);
        } else {
            TfSetenv(_name, _old);
        }
        _SetEnv(_name.c_str(), _oldCrt);
    }
    _ScopedEnv(const _ScopedEnv &) = delete;
    _ScopedEnv &operator=(const _ScopedEnv &) = delete;
    std::string _name;
    std::string _old;
    std::string _oldCrt;
};

static std::string
_GetEnv(const char *name)
{
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char *value = std::getenv(name);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
    return value ? value : "";
}

static void
_SetEnv(const char *name, const std::string &value)
{
#ifdef _WIN32
    _putenv_s(name, value.c_str());
#else
    if (value.empty()) {
        unsetenv(name);
    } else {
        setenv(name, value.c_str(), 1);
    }
#endif
}

static SdfPath
_FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == SchemaType::RigExecRoot) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

static std::vector<double>
_ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece = text.substr(
            begin, end == std::string::npos ? std::string::npos : end - begin);
        if (!piece.empty()) {
            frames.push_back(std::stod(piece));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return frames;
}

static const RigExecExampleFixture *
_Fixture(const char *stage)
{
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (std::string(fixture.stage) == stage) {
            return &fixture;
        }
    }
    return nullptr;
}

// The moved point arrays of a pose, deep-copied in path order: the bytes
// a retained snapshot must keep while later evaluations run.
using _PointBytes =
    std::vector<std::pair<std::string, std::vector<unsigned char>>>;

static _PointBytes
_PosePointBytes(const RigExecRigPose &pose)
{
    _PointBytes out;
    for (const auto &[path, value] : pose.movedProperties) {
        if (value.IsHolding<VtVec3fArray>()) {
            const auto &points = value.UncheckedGet<VtVec3fArray>();
            auto &bytes =
                out.emplace_back(path.GetString(), std::vector<unsigned char>())
                    .second;
            bytes.resize(points.size() * sizeof(GfVec3f));
            if (!bytes.empty()) {
                std::memcpy(bytes.data(), points.cdata(), bytes.size());
            }
        }
    }
    return out;
}

// The published result bytes of a pose: joint matrices and moved point
// arrays. Diagnostics are not compared here: a fresh evaluator's first
// generation also carries its compile notices, which later generations
// of the same rig do not repeat.
static bool
_SamePoseBytes(const RigExecRigPose &a, const RigExecRigPose &b)
{
    if (!a.valid || !b.valid ||
        a.jointMatricesFinal.size() != b.jointMatricesFinal.size()) {
        return false;
    }
    for (const auto &[path, matrix] : a.jointMatricesFinal) {
        const auto found = b.jointMatricesFinal.find(path);
        if (found == b.jointMatricesFinal.end() ||
            std::memcmp(&matrix, &found->second, sizeof(GfMatrix4d)) != 0) {
            return false;
        }
    }
    return _PosePointBytes(a) == _PosePointBytes(b);
}

static bool
_SameRuntimePoints(const std::vector<RigExecRuntimePoints> &a,
                   const std::vector<RigExecRuntimePoints> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        const std::vector<RrVec3f> &left = a[i].points.Vector();
        const std::vector<RrVec3f> &right = b[i].points.Vector();
        if (a[i].path != b[i].path || left.size() != right.size() ||
            (!left.empty() &&
             std::memcmp(left.data(), right.data(),
                         left.size() * sizeof(RrVec3f)) != 0)) {
            return false;
        }
    }
    return true;
}

static std::vector<RrVec3f>
_DeepPoints(const RigExecRuntimePoints &moved)
{
    const std::vector<RrVec3f> &points = moved.points.Vector();
    return std::vector<RrVec3f>(points.begin(), points.end());
}

static bool
_RetainedMatchesDeep(const std::vector<RigExecRuntimePoints> &retained,
                     const std::vector<std::vector<RrVec3f>> &deep)
{
    if (retained.size() != deep.size()) {
        return false;
    }
    for (size_t i = 0; i < retained.size(); ++i) {
        const std::vector<RrVec3f> &points = retained[i].points.Vector();
        if (points.size() != deep[i].size() ||
            (!points.empty() &&
             std::memcmp(points.data(), deep[i].data(),
                         points.size() * sizeof(RrVec3f)) != 0)) {
            return false;
        }
    }
    return true;
}

// One case: a stage, its frames, and the control attribute a drag edits
// (an authored value on the native side, a held input on the binary).
struct _Case {
    std::string name;
    UsdStageRefPtr stage;
    SdfPath rig;
    std::vector<double> frames;
    SdfPath dragAttribute;
    double dragDelta = 0.375;
};

// Bakes the case at its first frame under the current environment and
// opens a player on the bytes. False with the reason.
static bool
_BakeAndOpen(const _Case &testCase, RigExecTestPlayer *player,
             std::string *error)
{
    RigExecRigEvaluator baker(testCase.stage, testCase.rig);
    std::vector<uint8_t> bytes;
    if (!RigExecTestBakeAt(baker, testCase.frames.front(), &bytes, error)) {
        return false;
    }
    return player->Open(bytes, testCase.stage, error);
}

struct _RunWitness {
    size_t nativeGroups = 0;
    size_t nativeClusters = 0;
    size_t runtimeClusters = 0;
    size_t sparseEntries = 0;
};

// Native pose and runtime points of every frame, each front end fresh:
// the cross-front-end baseline every other property compares against.
static bool
_RunBoth(const _Case &testCase, std::vector<RigExecRigPose> *poses,
         std::vector<std::vector<RigExecRuntimePoints>> *played,
         std::string *error, _RunWitness *witness = nullptr)
{
    RigExecTestPlayer player;
    if (!_BakeAndOpen(testCase, &player, error)) {
        return false;
    }
    RigExecRigEvaluator evaluator(testCase.stage, testCase.rig);
    for (const double frame : testCase.frames) {
        RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(frame));
        if (!pose.valid) {
            *error = "the native pose is invalid";
            return false;
        }
        if (!player.Play(frame, error)) {
            return false;
        }
        std::vector<std::string> diffs;
        if (!RigExecCompareRuntimeOutputs(pose, player.Reader(), &diffs)) {
            *error = diffs.empty() ? "front ends differ" : diffs.front();
            return false;
        }
        if (witness && poses->empty()) {
            const auto *program = evaluator.GetBakedProgram();
            if (!program) {
                *error = "no baked program for the execution witness";
                return false;
            }
            const auto &graph = program->GetStepGraph();
            witness->nativeClusters = graph.lastClosedClusters;
            witness->runtimeClusters =
                player.Reader().GetClosedClusterCountForTesting();
            for (const auto &chain : graph.chains) {
                witness->nativeGroups += chain.baseGroups.size();
                for (const auto &revision : chain.revisions) {
                    for (const auto &group : revision.groups) {
                        witness->sparseEntries +=
                            group.walk.computed + group.walk.reused;
                    }
                }
            }
        }
        poses->push_back(std::move(pose));
        played->push_back(player.Reader().GetPoints());
    }
    return true;
}

static void
_CheckRunBoth(const _Case &testCase)
{
    std::vector<RigExecRigPose> poses;
    std::vector<std::vector<RigExecRuntimePoints>> played;
    std::string error;
    const bool ok = _RunBoth(testCase, &poses, &played, &error);
    if (!ok) {
        std::printf("%s: %s\n", testCase.name.c_str(), error.c_str());
    }
    CHECK(ok);
    CHECK(poses.size() == testCase.frames.size());
    if (ok) {
        std::printf("%s: baked program and .rigexec playback agree on %zu "
                    "frame(s)\n",
                    testCase.name.c_str(), poses.size());
    }
}

// Byte-identical results across valid partitionings: the chunk/group
// caps, the fine grain (one step per cluster) and the serial schedule
// each repartition the same rig; neither front end may publish a
// different byte for any of them.
static void
_CheckPartitioning(const _Case &testCase, bool checkWitness)
{
    const _ScopedEnv chunkTarget("RIGEXEC_BAKED_CHUNK_VERTS", "1024");
    const _ScopedEnv grain("RIGEXEC_BAKED_GRAIN_US", "1000");
    const _ScopedEnv schedule("RIGEXEC_BAKED_SCHEDULE", "parallel");
    struct Config {
        const char *label;
        std::vector<std::pair<const char *, const char *>> knobs;
    };
    const std::vector<Config> configs = {
        {"default", {}},
        {"chunked",
         {{"RIGEXEC_BAKED_CHUNK_VERTS", "16"},
          {"RIGEXEC_BAKED_GROUP_VERTS", "8"}}},
        {"fine grain", {{"RIGEXEC_BAKED_GRAIN_US", "0"}}},
        {"serial", {{"RIGEXEC_BAKED_SCHEDULE", "serial"}}},
    };
    std::vector<RigExecRigPose> baselinePoses;
    std::vector<std::vector<RigExecRuntimePoints>> baselinePlayed;
    _RunWitness baseline;
    for (size_t i = 0; i < configs.size(); ++i) {
        const Config &config = configs[i];
        std::vector<std::unique_ptr<_ScopedEnv>> scoped;
        for (const auto &[name, value] : config.knobs) {
            scoped.push_back(std::make_unique<_ScopedEnv>(name, value));
        }
        std::vector<RigExecRigPose> poses;
        std::vector<std::vector<RigExecRuntimePoints>> played;
        std::string error;
        _RunWitness witness;
        const bool ok = _RunBoth(testCase, &poses, &played, &error,
                                 checkWitness ? &witness : nullptr);
        if (!ok) {
            std::printf("%s partitioning %s: %s\n", testCase.name.c_str(),
                        config.label, error.c_str());
        }
        CHECK(ok);
        if (!ok) {
            continue;
        }
        if (baselinePoses.empty()) {
            baselinePoses = poses;
            baselinePlayed = played;
            baseline = witness;
            continue;
        }
        if (checkWitness && i == 1) {
            CHECK(baseline.nativeGroups == 0);
            CHECK(witness.nativeGroups > 1);
        }
        if (checkWitness && i == 2) {
            CHECK(witness.nativeClusters > baseline.nativeClusters);
            CHECK(witness.runtimeClusters > baseline.runtimeClusters);
        }
        bool same = poses.size() == baselinePoses.size() &&
                    played.size() == baselinePlayed.size();
        for (size_t f = 0; same && f < poses.size(); ++f) {
            same = _SamePoseBytes(poses[f], baselinePoses[f]) &&
                   _SameRuntimePoints(played[f], baselinePlayed[f]);
        }
        if (!same) {
            std::printf("%s partitioning %s: bytes differ from the default "
                        "partitioning\n",
                        testCase.name.c_str(), config.label);
        }
        CHECK(same);
    }
    std::printf("%s: %zu partitionings publish identical bytes\n",
                testCase.name.c_str(), configs.size());
}

// Memo hit versus recompute: a second evaluation at a time the front
// end has already published, then a fresh front end's first evaluation
// of that time, publish the same bytes; the frozen worker's warmed
// pose is the live pose's bytes too.
static void
_CheckMemoRecompute(const _Case &testCase)
{
    const double frame = testCase.frames.front();
    RigExecRigEvaluator evaluator(testCase.stage, testCase.rig);
    const RigExecRigPose first = evaluator.Evaluate(UsdTimeCode(frame));
    CHECK(first.valid);
    const RigExecRigPose memoHit = evaluator.Evaluate(UsdTimeCode(frame));
    CHECK(_SamePoseBytes(first, memoHit));
    RigExecRigEvaluator fresh(testCase.stage, testCase.rig);
    const RigExecRigPose recomputed = fresh.Evaluate(UsdTimeCode(frame));
    CHECK(_SamePoseBytes(first, recomputed));

    std::string why;
    RigExecRigEvaluator frozenSource(testCase.stage, testCase.rig);
    const bool frozenOk = RigExecFrozenMatchesLive(&frozenSource, testCase.rig,
                                                   UsdTimeCode(frame), &why);
    if (!frozenOk) {
        std::printf("%s frozen: %s\n", testCase.name.c_str(), why.c_str());
    }
    CHECK(frozenOk);

    RigExecTestPlayer player;
    std::string error;
    const bool opened = _BakeAndOpen(testCase, &player, &error);
    CHECK(opened);
    if (!opened) {
        std::printf("%s memo: %s\n", testCase.name.c_str(), error.c_str());
        return;
    }
    CHECK(player.Play(frame, &error));
    const std::vector<RigExecRuntimePoints> playedFirst =
        player.Reader().GetPoints();
    CHECK(player.Play(frame, &error));
    CHECK(_SameRuntimePoints(playedFirst, player.Reader().GetPoints()));
    RigExecTestPlayer freshPlayer;
    const bool reopened = _BakeAndOpen(testCase, &freshPlayer, &error);
    CHECK(reopened);
    if (reopened) {
        CHECK(freshPlayer.Play(frame, &error));
        CHECK(
            _SameRuntimePoints(playedFirst, freshPlayer.Reader().GetPoints()));
    }
    std::printf("%s: memo hit, recompute and frozen warm agree\n",
                testCase.name.c_str());
}

// Retained snapshots unchanged after an edit: a pose and a points
// snapshot retained across a drag keep their bytes, the edited run
// publishes different bytes on both front ends (and the same bytes as
// each other), and releasing the drag republishes the retained bytes.
static void
_CheckRetainedAfterEdit(const _Case &testCase)
{
    const double frame = testCase.frames.front();
    RigExecRigEvaluator evaluator(testCase.stage, testCase.rig);
    const RigExecRigPose retainedPose = evaluator.Evaluate(UsdTimeCode(frame));
    CHECK(retainedPose.valid);
    const _PointBytes retainedBytes = _PosePointBytes(retainedPose);
    CHECK(!retainedBytes.empty());

    RigExecTestPlayer player;
    std::string error;
    const bool opened = _BakeAndOpen(testCase, &player, &error);
    CHECK(opened);
    if (!opened) {
        std::printf("%s retained: %s\n", testCase.name.c_str(), error.c_str());
        return;
    }
    CHECK(player.Play(frame, &error));
    const std::vector<RigExecRuntimePoints> retainedPoints =
        player.Reader().GetPoints();
    std::vector<std::vector<RrVec3f>> retainedDeep;
    for (const RigExecRuntimePoints &moved : retainedPoints) {
        retainedDeep.push_back(_DeepPoints(moved));
    }
    CHECK(!retainedPoints.empty());

    const UsdAttribute attribute =
        testCase.stage->GetAttributeAtPath(testCase.dragAttribute);
    CHECK(attribute);
    double authored = 0.0;
    CHECK(attribute && attribute.Get(&authored, UsdTimeCode(frame)));
    const double dragged = authored + testCase.dragDelta;

    CHECK(player.Hold(testCase.dragAttribute.GetString(), dragged, &error));
    CHECK(player.Play(frame, &error));
    const bool runtimeMoved =
        !_SameRuntimePoints(retainedPoints, player.Reader().GetPoints());
    CHECK(runtimeMoved);
    CHECK(_RetainedMatchesDeep(retainedPoints, retainedDeep));

    RigExecRigPose editedPose;
    {
        RigExecTestSessionEdit edit(attribute, dragged);
        CHECK(edit.IsSet());
        editedPose = evaluator.Evaluate(UsdTimeCode(frame));
    }
    CHECK(editedPose.valid);
    CHECK(!_SamePoseBytes(retainedPose, editedPose));
    CHECK(_PosePointBytes(retainedPose) == retainedBytes);
    std::vector<std::string> diffs;
    const bool crossOk =
        RigExecCompareRuntimeOutputs(editedPose, player.Reader(), &diffs);
    if (!crossOk) {
        for (const std::string &diff : diffs) {
            std::printf("%s retained cross: %s\n", testCase.name.c_str(),
                        diff.c_str());
        }
    }
    CHECK(crossOk);

    player.ReleaseAll();
    CHECK(player.Play(frame, &error));
    CHECK(_RetainedMatchesDeep(retainedPoints, retainedDeep));
    CHECK(_SameRuntimePoints(retainedPoints, player.Reader().GetPoints()));
    const RigExecRigPose released = evaluator.Evaluate(UsdTimeCode(frame));
    CHECK(_PosePointBytes(retainedPose) == retainedBytes);
    CHECK(_SamePoseBytes(retainedPose, released));
    std::printf("%s: retained snapshots survive the drag, release "
                "republishes them\n",
                testCase.name.c_str());
}

// Dense versus eligible-sparse on the native front end: sparse input
// sampling (the default) and the dense sweep over every input publish
// the same pose. Playback eligibility is covered by the paired
// dense/sparse stages below, whose fields the shared predicate walks
// sparsely on one side and resolves densely on the other.
static void
_CheckSparseSampling(const _Case &testCase)
{
    const double frame = testCase.frames.front();
    RigExecRigPose sparsePose;
    {
        RigExecRigEvaluator evaluator(testCase.stage, testCase.rig);
        sparsePose = evaluator.Evaluate(UsdTimeCode(frame));
    }
    CHECK(sparsePose.valid);
    RigExecRigPose densePose;
    {
        const _ScopedEnv swept("RIGEXEC_SPARSE_SAMPLING", "0");
        RigExecRigEvaluator evaluator(testCase.stage, testCase.rig);
        densePose = evaluator.Evaluate(UsdTimeCode(frame));
    }
    CHECK(densePose.valid);
    CHECK(_SamePoseBytes(sparsePose, densePose));
    std::printf("%s: sparse sampling and dense sweep publish identical "
                "bytes\n",
                testCase.name.c_str());
}

static void
_CheckCase(const _Case &testCase, bool checkPartitionWitness = false)
{
    _CheckRunBoth(testCase);
    _CheckPartitioning(testCase, checkPartitionWitness);
    _CheckSparseSampling(testCase);
    _CheckMemoRecompute(testCase);
    _CheckRetainedAfterEdit(testCase);
}

// A classicLinear skin at elementSize 10 over four joints, 48 points:
// point i binds joints i % 4 and (i + 1) % 4 at 0.625 / 0.375, the
// remaining eight slots at weight zero.
static UsdStageRefPtr
_ClassicLinearSkinStage()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), UsdGeomTokens->Scope);
    stage->DefinePrim(SdfPath("/Asset/Rig"), SchemaType::RigExecRoot);
    SdfPathVector joints;
    for (int j = 0; j < 4; ++j) {
        const UsdPrim joint = stage->DefinePrim(
            SdfPath("/Asset/Rig/Joints/J" + std::to_string(j)),
            SchemaType::RigExecControl);
        const UsdAttribute ty = joint.CreateAttribute(
            SchemaAttr::avarsTy, SdfValueTypeNames->Double);
        ty.Set(0.25 * double(j), UsdTimeCode(1.0));
        ty.Set(0.25 * double(j) + 0.5, UsdTimeCode(2.0));
        ty.Set(0.25 * double(j) - 0.25, UsdTimeCode(3.0));
        joints.push_back(joint.GetPath());
    }
    constexpr size_t kPoints = 48;
    VtVec3fArray points(kPoints);
    for (size_t i = 0; i < kPoints; ++i) {
        points[i] = GfVec3f(0.125f * float(i % 8), 0.25f * float(i / 8),
                            0.0625f * float(i % 5));
    }
    stage->DefinePrim(SdfPath("/Asset/Geom/Body"), UsdGeomTokens->Points)
        .CreateAttribute(UsdGeomTokens->points, SdfValueTypeNames->Point3fArray)
        .Set(points);
    const UsdPrim skin = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Skin"),
                                           SchemaType::RigExecSkinMover);
    skin.ApplyAPI(SchemaType::RigExecMoverAPI);
    skin.GetRelationship(_rigExecMoves)
        .SetTargets({SdfPath("/Asset/Geom/Body")
                         .AppendProperty(UsdGeomTokens->points)});
    skin.GetAttribute(_inputsDefaultWeight).Set(1.0f);
    skin.CreateRelationship(_rigExecInfluences).SetTargets(joints);
    skin.CreateAttribute(_rigExecElementSize, SdfValueTypeNames->Int).Set(10);
    skin.CreateAttribute(_rigExecSkinningMethod, SdfValueTypeNames->Token)
        .Set(UsdSkelTokens->classicLinear);
    VtIntArray indices(kPoints * 10, 0);
    VtFloatArray weights(kPoints * 10, 0.0f);
    for (size_t i = 0; i < kPoints; ++i) {
        indices[i * 10] = int(i % 4);
        indices[i * 10 + 1] = int((i + 1) % 4);
        weights[i * 10] = 0.625f;
        weights[i * 10 + 1] = 0.375f;
    }
    skin.CreateAttribute(_rigExecJointIndices, SdfValueTypeNames->IntArray)
        .Set(indices);
    skin.CreateAttribute(_rigExecJointWeights, SdfValueTypeNames->FloatArray)
        .Set(weights);
    return stage;
}

// One matrix mover under a static weight: sparse (indices and values,
// zero default -- the eligible sparse walk) or dense (one value per
// point) spelling of the same field over the same points.
static UsdStageRefPtr
_EnvelopeStage(bool sparse)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), UsdGeomTokens->Scope);
    stage->DefinePrim(SdfPath("/Asset/Rig"), SchemaType::RigExecRoot);
    const UsdPrim moving = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Moving"), SchemaType::RigExecControl);
    const UsdAttribute tx =
        moving.CreateAttribute(SchemaAttr::avarsTx, SdfValueTypeNames->Double);
    tx.Set(0.5, UsdTimeCode(1.0));
    tx.Set(1.0, UsdTimeCode(2.0));
    tx.Set(1.5, UsdTimeCode(3.0));
    constexpr size_t kPoints = 32;
    VtVec3fArray points(kPoints);
    for (size_t i = 0; i < kPoints; ++i) {
        points[i] = GfVec3f(0.25f * float(i % 8), 0.5f * float(i / 8),
                            0.125f * float(i % 3));
    }
    const SdfPath target =
        SdfPath("/Asset/Geom/Body").AppendProperty(UsdGeomTokens->points);
    stage->DefinePrim(target.GetPrimPath(), UsdGeomTokens->Points)
        .CreateAttribute(UsdGeomTokens->points, SdfValueTypeNames->Point3fArray)
        .Set(points);
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Field"), SchemaType::RigExecStaticWeight);
    weight.CreateRelationship(_rigExecWeightTarget).SetTargets({target});
    weight.CreateAttribute(_rigExecDefaultWeight, SdfValueTypeNames->Float)
        .Set(0.0f);
    const VtIntArray sparseIndices{1, 2, 5, 6, 17, 30};
    const VtFloatArray sparseValues{0.25f, 0.5f, 0.75f, 1.0f, 0.5f, 0.25f};
    if (sparse) {
        weight.CreateAttribute(_rigExecRepresentation, SdfValueTypeNames->Token)
            .Set(_sparse);
        weight.CreateAttribute(_rigExecIndices, SdfValueTypeNames->IntArray)
            .Set(sparseIndices);
        weight.CreateAttribute(_rigExecValues, SdfValueTypeNames->FloatArray)
            .Set(sparseValues);
    } else {
        VtFloatArray dense(kPoints, 0.0f);
        for (size_t k = 0; k < sparseIndices.size(); ++k) {
            dense[size_t(sparseIndices[k])] = sparseValues[k];
        }
        weight.CreateAttribute(_rigExecRepresentation, SdfValueTypeNames->Token)
            .Set(_dense);
        weight.CreateAttribute(_rigExecValues, SdfValueTypeNames->FloatArray)
            .Set(dense);
    }
    const UsdPrim mover = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/Lift"),
                                            SchemaType::RigExecMatrixMover);
    mover.ApplyAPI(SchemaType::RigExecMoverAPI);
    mover.CreateRelationship(_rigExecMoves).SetTargets({target});
    mover.CreateRelationship(_rigExecTransform).SetTargets({moving.GetPath()});
    mover.CreateRelationship(_rigExecWeightObject)
        .SetTargets({weight.GetPath()});
    return stage;
}

// Dense versus eligible-sparse equivalence: the same field spelled
// densely and sparsely publishes the same bytes on both front ends.
static void
_CheckDenseSparseEquivalence()
{
    const char *const name = "dense versus eligible sparse";
    _Case denseCase{name,
                    _EnvelopeStage(false),
                    SdfPath("/Asset/Rig"),
                    {1.0, 2.0, 3.0},
                    SdfPath("/Asset/Rig/Controls/Moving")
                        .AppendProperty(SchemaAttr::avarsTx)};
    _Case sparseCase{name,
                     _EnvelopeStage(true),
                     SdfPath("/Asset/Rig"),
                     {1.0, 2.0, 3.0},
                     SdfPath("/Asset/Rig/Controls/Moving")
                         .AppendProperty(SchemaAttr::avarsTx)};
    std::vector<RigExecRigPose> densePoses, sparsePoses;
    std::vector<std::vector<RigExecRuntimePoints>> densePlayed, sparsePlayed;
    const _ScopedEnv chunkTarget("RIGEXEC_BAKED_CHUNK_VERTS", "16");
    const _ScopedEnv groupTarget("RIGEXEC_BAKED_GROUP_VERTS", "8");
    _RunWitness denseWitness, sparseWitness;
    std::string error;
    const bool denseOk =
        _RunBoth(denseCase, &densePoses, &densePlayed, &error, &denseWitness);
    if (!denseOk) {
        std::printf("%s dense: %s\n", name, error.c_str());
    }
    CHECK(denseOk);
    const bool sparseOk = _RunBoth(sparseCase, &sparsePoses, &sparsePlayed,
                                   &error, &sparseWitness);
    if (!sparseOk) {
        std::printf("%s sparse: %s\n", name, error.c_str());
    }
    CHECK(sparseOk);
    CHECK(sparseWitness.nativeGroups > 1);
    CHECK(sparseWitness.sparseEntries == 6);
    CHECK(denseWitness.sparseEntries == 0);
    bool same = densePoses.size() == sparsePoses.size() &&
                densePlayed.size() == sparsePlayed.size();
    for (size_t f = 0; same && f < densePoses.size(); ++f) {
        same = _SamePoseBytes(densePoses[f], sparsePoses[f]) &&
               _SameRuntimePoints(densePlayed[f], sparsePlayed[f]);
    }
    CHECK(same);
    // The sparse spelling really moves points: the property is not
    // comparing two untouched arrays.
    bool moved = false;
    if (!sparsePlayed.empty() && !sparsePlayed.front().empty()) {
        const UsdAttribute points = sparseCase.stage->GetAttributeAtPath(
            SdfPath("/Asset/Geom/Body").AppendProperty(UsdGeomTokens->points));
        VtVec3fArray rest;
        moved = points.Get(&rest) &&
                sparsePlayed.front().front().points.size() == rest.size() &&
                std::memcmp(sparsePlayed.front().front().points.data(),
                            rest.cdata(), rest.size() * sizeof(GfVec3f)) != 0;
    }
    CHECK(moved);
    std::printf("%s: dense and sparse publish identical bytes\n", name);
}

static _Case
_FileCase(const std::string &examplesDir, const char *stageName,
          const std::vector<double> &frames, const char *label)
{
    const RigExecExampleFixture *fixture = _Fixture(stageName);
    CHECK(fixture != nullptr && fixture->bakesToday);
    const UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/" + stageName);
    CHECK(stage);
    _Case testCase;
    testCase.name = label;
    testCase.stage = stage;
    testCase.frames = frames;
    if (stage) {
        testCase.rig = _FindRig(stage);
        CHECK(!testCase.rig.IsEmpty());
    }
    if (fixture && fixture->controlPrim[0] != '\0') {
        testCase.dragAttribute = SdfPath(std::string(fixture->controlPrim) +
                                         "." + fixture->controlAvar);
    }
    return testCase;
}

static void
_CheckBipedBodySkin(const _Case &testCase)
{
    // The case really is the biped body skin at elementSize 10.
    bool found = false;
    for (const UsdPrim &prim : testCase.stage->TraverseAll()) {
        if (prim.GetTypeName() != SchemaType::RigExecSkinMover) {
            continue;
        }
        int elementSize = 0;
        const UsdAttribute authored = prim.GetAttribute(_rigExecElementSize);
        if (authored && authored.Get(&elementSize) && elementSize == 10) {
            TfToken method;
            const UsdAttribute methodAttr =
                prim.GetAttribute(_rigExecSkinningMethod);
            if (methodAttr) {
                methodAttr.Get(&method);
            }
            std::printf("biped body skin: %s elementSize 10 method %s\n",
                        prim.GetPath().GetString().c_str(),
                        method.GetString().c_str());
            found = true;
        }
    }
    CHECK(found);
}

int
main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }

    // The biped body skin, at the fixture's frames 1 to 3.
    const RigExecExampleFixture *biped = _Fixture("biped/Biped_body.usda");
    CHECK(biped != nullptr);
    if (biped) {
        _Case testCase =
            _FileCase(examplesDir, biped->stage, _ParseFrames(biped->frames),
                      "biped body skin");
        if (testCase.stage) {
            _CheckBipedBodySkin(testCase);
            _CheckCase(testCase);
        }
    }

    // One HarmonicsChain case from the parity snapshot: the snapshot's
    // frames 1060 and 1064, beside the first authored frame.
    const RigExecExampleFixture *harmonics = _Fixture("19_HarmonicsChain.usda");
    CHECK(harmonics != nullptr);
    if (harmonics) {
        _Case testCase =
            _FileCase(examplesDir, harmonics->stage, {1001.0, 1060.0, 1064.0},
                      "HarmonicsChain parity snapshot");
        if (testCase.stage) {
            _CheckCase(testCase);
        }
    }

    // A synthetic classicLinear skin at elementSize 10.
    {
        _Case testCase{"classicLinear skin elementSize 10",
                       _ClassicLinearSkinStage(),
                       SdfPath("/Asset/Rig"),
                       {1.0, 2.0, 3.0},
                       SdfPath("/Asset/Rig/Joints/J1")
                           .AppendProperty(SchemaAttr::avarsTy)};
        _CheckCase(testCase, true);
    }

    _CheckDenseSparseEquivalence();

    std::printf("testRigExecEvalCoreProperties: %d failures\n", failures);
    return failures ? 1 : 0;
}
