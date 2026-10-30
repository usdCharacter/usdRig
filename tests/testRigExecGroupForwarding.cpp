// A vertex group forwards points its mover does not weight without
// recomputing the ones it does. A range chain's group is one content
// version, so a mover downstream of a moved group republishes it even when
// every moved point lies outside its support; a sparse Matrix group step
// then takes its supported points from the group's walk memo
// (pointBlocks.h), which records them once the mover's transform has held
// for a run. Checked on a small gated chain through the native
// program, a frame-cache worker and the binary runtime, and on the biped,
// where an arm edit moves shoulder-corrective points that share index
// ranges with the face clusters.
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"
#include "rigExecMath/pointBlocks.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

using Program = RigExecBakedProgramImpl;
using Chain = Program::GeomChain;
using Revision = Program::GeomRevision;

bool
SameBits(const GfVec3f &a, const GfVec3f &b)
{
    return std::memcmp(&a, &b, sizeof(GfVec3f)) == 0;
}

/// The chain moving \p target, or -1.
int
FindChain(const Program &B, const SdfPath &target)
{
    for (size_t c = 0; c < B.chains.size(); ++c) {
        if (B.chains[c].target == target) {
            return int(c);
        }
    }
    return -1;
}

/// The chain index of the revision of the mover named \p name, or -1.
int
FindRevision(const Chain &chain, const std::string &name)
{
    for (size_t r = 0; r < chain.revisions.size(); ++r) {
        if (chain.revisions[r].moverPath.GetName() == name) {
            return int(r);
        }
    }
    return -1;
}

/// Whether \p revision is a Range Matrix revision over a sparse weight: the
/// one whose group steps walk their support through a memo.
bool
WalksSparse(const Program &B, const Revision &revision)
{
    return revision.op == RigExecRevisionOp::Matrix &&
           revision.role == RigExecBakedRevisionRole::Range &&
           revision.weightObject >= 0 &&
           size_t(revision.weightObject) < B.weightObjects.size() &&
           !B.weightObjects[size_t(revision.weightObject)].indices.empty();
}

/// The chain index of the last revision before \p r that writes group \p g,
/// or -1 when the group enters \p r from the chain base.
int
EnteringWriter(const Chain &chain, size_t r, size_t g)
{
    for (size_t q = r; q > 0; --q) {
        const Revision &before = chain.revisions[q - 1];
        if (g < before.groupWritten.size() && before.groupWritten[g]) {
            return int(q - 1);
        }
    }
    return -1;
}

/// What a chain published, copied: every group of every revision and of the
/// base, each revision's walk inputs, and whether each group's walk memo
/// held recorded entries.
struct Snapshot {
    std::vector<std::vector<std::vector<GfVec3f>>> groups;
    std::vector<std::vector<char>> recorded;
    std::vector<std::vector<GfVec3f>> base;
    std::vector<GfMatrix4d> transforms;
    std::vector<char> radial;

    static std::vector<GfVec3f> Copy(const RigExecPointsRef<GfVec3f> &ref)
    {
        return ref.data ? std::vector<GfVec3f>(ref.data, ref.data + ref.count)
                        : std::vector<GfVec3f>();
    }
    explicit Snapshot(const Chain &chain)
    {
        for (const auto &state : chain.baseGroups) {
            base.push_back(Copy(state.published));
        }
        for (const Revision &revision : chain.revisions) {
            groups.emplace_back();
            recorded.emplace_back();
            for (const auto &state : revision.groups) {
                groups.back().push_back(Copy(state.published));
                recorded.back().push_back(state.walk.indices.empty() ? 0 : 1);
            }
            transforms.push_back(revision.parameters.transform);
            radial.push_back(revision.parameters.radialWeight ? 1 : 0);
        }
    }
    const std::vector<GfVec3f> &Entering(const Chain &chain, size_t r,
                                         size_t g) const
    {
        const int writer = EnteringWriter(chain, r, g);
        return writer < 0 ? base[g] : groups[size_t(writer)][g];
    }
};

/// The steps of \p kind the last run executed, as (revision id, part).
std::set<std::pair<int, int>>
RanSteps(const Program &B, RigExecBakedStepKind kind)
{
    std::set<std::pair<int, int>> ran;
    for (size_t c = 0; c < B.opGraph.ops.size() && c < B.opExecution.ran.size();
         ++c) {
        if (!B.opExecution.ran[c]) {
            continue;
        }
        const RigExecBakedStep &step = B.steps[B.opGraph.ops[c].originalIndex];
        if (step.kind == kind) {
            ran.emplace(step.object, step.part);
        }
    }
    return ran;
}

/// Whether a step of revision \p id is among \p ran.
bool
RanFor(const std::set<std::pair<int, int>> &ran, int id)
{
    const auto found = ran.lower_bound({id, INT32_MIN});
    return found != ran.end() && found->first == id;
}

struct WalkTotals {
    size_t steps = 0, forwards = 0, computed = 0, reused = 0;
};

/// Every sparse Matrix group step of chain \p c that the last run executed,
/// against \p before (the chain as the run found it):
///  - it computed exactly the supported entries whose entering point moved a
///    bit and reused the rest; or all of them when its transform or blend
///    arm moved, or held but nothing was recorded yet;
///  - a walk whose transform and arm held left its entries recorded, and
///    one whose transform or arm moved left none;
///  - it published the entering point at every point outside its support,
///    and its previous result at every supported entry it reused.
/// \p only, when not empty, restricts the check to that mover and returns
/// its totals; a `forward` is a step that computed nothing.
WalkTotals
CheckWalks(const Program &B, int c, const Snapshot &before,
           const std::string &only, const char *what)
{
    WalkTotals totals;
    const Chain &chain = B.chains[size_t(c)];
    const int first = B.chainRevisionBegin[size_t(c)];
    const std::set<std::pair<int, int>> ran =
        RanSteps(B, RigExecBakedStepKind::RevisionChunk);
    for (size_t r = 0; r < chain.revisions.size(); ++r) {
        const Revision &revision = chain.revisions[r];
        if (!WalksSparse(B, revision) ||
            (!only.empty() && revision.moverPath.GetName() != only)) {
            continue;
        }
        const std::vector<int> &indices =
            B.weightObjects[size_t(revision.weightObject)].indices;
        const bool paramsMoved =
            std::memcmp(&before.transforms[r], &revision.parameters.transform,
                        sizeof(GfMatrix4d)) != 0 ||
            (before.radial[r] != 0) != revision.parameters.radialWeight;
        for (size_t g = 0; g < revision.groups.size(); ++g) {
            if (!ran.count({first + int(r), int(g)})) {
                continue;
            }
            const auto &state = revision.groups[g];
            const int begin = chain.groupBounds[g];
            const int end = chain.groupBounds[g + 1];
            const int writer = EnteringWriter(chain, r, g);
            const RigExecPointsRef<GfVec3f> &entering =
                writer < 0 ? chain.baseGroups[g].published
                           : chain.revisions[size_t(writer)].groups[g].published;
            const std::vector<GfVec3f> &enteringBefore =
                before.Entering(chain, r, g);
            const std::vector<GfVec3f> &ownBefore = before.groups[r][g];
            bool sized = entering.count == size_t(end - begin) &&
                         state.published.count == entering.count &&
                         enteringBefore.size() == entering.count;
            CHECK(sized);
            if (!sized) {
                continue;
            }
            if (state.ownPublished < 0) {
                // Passed through whole (an identity transform): no walk.
                CHECK(state.walk.computed == 0 && state.walk.reused == 0);
                continue;
            }
            size_t entries = 0, moved = 0, wrongForward = 0, wrongReuse = 0;
            std::vector<char> supported(size_t(end - begin), 0);
            for (const int index : indices) {
                if (index < begin || index >= end) {
                    continue;
                }
                const size_t k = size_t(index - begin);
                supported[k] = 1;
                ++entries;
                const bool same = SameBits(enteringBefore[k], entering.data[k]);
                moved += same ? 0 : 1;
                if (same && !paramsMoved && ownBefore.size() == entering.count &&
                    !SameBits(ownBefore[k], state.published.data[k])) {
                    ++wrongReuse;
                }
            }
            for (size_t k = 0; k < supported.size(); ++k) {
                if (!supported[k] &&
                    !SameBits(entering.data[k], state.published.data[k])) {
                    ++wrongForward;
                }
            }
            const bool reusing = !paramsMoved && before.recorded[r][g];
            const size_t expected = reusing ? moved : entries;
            const bool ok = state.walk.computed == expected &&
                            state.walk.computed + state.walk.reused == entries &&
                            state.walk.indices.empty() == paramsMoved &&
                            wrongForward == 0 && wrongReuse == 0;
            if (!ok) {
                std::printf("  %s: %s group %zu computed %u and reused %u of "
                            "%zu entries, expected %zu computed; %zu point(s) "
                            "not forwarded, %zu reused result(s) moved\n",
                            what, revision.moverPath.GetText(), g,
                            state.walk.computed, state.walk.reused, entries,
                            expected, wrongForward, wrongReuse);
            }
            CHECK(ok);
            ++totals.steps;
            totals.forwards += state.walk.computed == 0 ? 1 : 0;
            totals.computed += state.walk.computed;
            totals.reused += state.walk.reused;
        }
    }
    return totals;
}

/// Every group of the final version of chain \p c in \p a and \p b holds the
/// same bits.
bool
SameResult(const Program &a, const Program &b, int c)
{
    const Chain &left = a.chains[size_t(c)];
    const Chain &right = b.chains[size_t(c)];
    if (left.revisions.size() != right.revisions.size() ||
        left.groupBounds != right.groupBounds) {
        return false;
    }
    const size_t groups =
        left.groupBounds.empty() ? 0 : left.groupBounds.size() - 1;
    const size_t last = left.revisions.size();
    for (size_t g = 0; g < groups; ++g) {
        const int lw = EnteringWriter(left, last, g);
        const int rw = EnteringWriter(right, last, g);
        const auto &l = lw < 0 ? left.baseGroups[g].published
                               : left.revisions[size_t(lw)].groups[g].published;
        const auto &r = rw < 0 ? right.baseGroups[g].published
                               : right.revisions[size_t(rw)].groups[g].published;
        if (!RigExecPointsBitsEqual(l.data, l.count, r.data, r.count)) {
            return false;
        }
    }
    return true;
}

RigExecValueOverride
Override(const char *prim, const char *attribute, const VtValue &value)
{
    return RigExecValueOverride{SdfPath(prim), TfToken(), TfToken(attribute),
                                value};
}

// --- A gated chain ---------------------------------------------------------

constexpr size_t kPoints = 10000;
const char *const kRig = "/Asset/Rig";
const char *const kRaise = "/Asset/Rig/BlendInputs/Raise";
const char *const kLift = "/Asset/Rig/BlendInputs/Lift";
const char *const kMoving = "/Asset/Rig/Moving";

/// The gated mover's support: every tenth point of [500, 2500), so three of
/// the chain's ten groups hold supported points beside unsupported ones.
bool
Supported(size_t i)
{
    return i >= 500 && i < 2500 && i % 10 == 3;
}

/// One 10000-point target under a blend shape and then a gated radial
/// Matrix mover. `Raise` moves points outside the mover's support in the
/// groups it writes; `Lift` moves ten supported points of one group;
/// `Moving` is the mover's transform. \p raise, \p lift and \p moving are
/// the authored values.
UsdStageRefPtr
MakeGatedStage(float raise, float lift, double moving)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath(kRig), TfToken("RigExecRoot"));
    const UsdPrim control =
        stage->DefinePrim(SdfPath(kMoving), TfToken("RigExecControl"));
    control.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(moving);
    const SdfPath target("/Asset/Shape.points");
    VtVec3fArray base(kPoints), raised(kPoints), lifted(kPoints);
    VtIntArray support;
    VtFloatArray weights;
    for (size_t i = 0; i < kPoints; ++i) {
        base[i] = GfVec3f(float(i % 100), float(i / 100), 0.0f);
        raised[i] = base[i];
        lifted[i] = base[i];
        if (i < 4000 && i % 10 == 7) {
            raised[i][2] += 1.0f;
        }
        if (i >= 1000 && i < 1100 && Supported(i)) {
            lifted[i][1] += 2.0f;
        }
        if (Supported(i)) {
            support.push_back(int(i));
            // A falloff of a few distinct values, some at the ends.
            weights.push_back(float((i / 10) % 5) / 4.0f);
        }
    }
    const auto points = [&](const char *path, const VtVec3fArray &values) {
        stage->DefinePrim(SdfPath(path), TfToken("Points"))
            .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
            .Set(values);
    };
    points("/Asset/Shape", base);
    points("/Asset/Raised", raised);
    points("/Asset/Lifted", lifted);
    const auto input = [&](const char *path, const char *shape, float value) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecBlendInput"));
        prim.CreateAttribute(TfToken("inputs:weight"), SdfValueTypeNames->Float)
            .Set(value);
        const UsdPrim sample = stage->DefinePrim(
            SdfPath(path).AppendChild(TfToken("Full")),
            TfToken("RigExecBlendSample"));
        sample.CreateAttribute(TfToken("rigExec:activation"),
                               SdfValueTypeNames->Float)
            .Set(1.0f);
        sample.CreateRelationship(TfToken("rigExec:targetPoints"))
            .SetTargets({SdfPath(shape).AppendProperty(TfToken("points"))});
        prim.CreateRelationship(TfToken("rigExec:samples"))
            .SetTargets({sample.GetPath()});
    };
    input(kRaise, "/Asset/Raised", raise);
    input(kLift, "/Asset/Lifted", lift);
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sparse"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    weight.CreateAttribute(TfToken("rigExec:indices"),
                           SdfValueTypeNames->IntArray, false)
        .Set(support);
    weight.CreateAttribute(TfToken("rigExec:values"),
                           SdfValueTypeNames->FloatArray, false)
        .Set(weights);
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float, false)
        .Set(0.0f);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto mover = [&](const char *name, const char *type) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name), TfToken(type));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves")).SetTargets({target});
        return prim;
    };
    // Movers run last sibling first: the blend, then the gated mover.
    const UsdPrim gated = mover("Gated", "RigExecMatrixMover");
    gated.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({control.GetPath()});
    gated.CreateRelationship(TfToken("rigExec:weightObject"), false)
        .SetTargets({weight.GetPath()});
    gated.CreateAttribute(TfToken("rigExec:weightBlend"),
                          SdfValueTypeNames->Token)
        .Set(TfToken("radial"));
    const UsdPrim blend = mover("Blend", "RigExecBlendShapeMover");
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({SdfPath(kRaise), SdfPath(kLift)});
    blend.CreateAttribute(TfToken("inputs:defaultWeight"),
                          SdfValueTypeNames->Float)
        .Set(1.0f);
    return stage;
}

struct GatedValues {
    float raise = 0.5f, lift = 0.0f;
    double moving = 1.0;
    const char *what = "";
};

void
TestAGatedChainForwards()
{
    const UsdTimeCode time(1.0);
    const SdfPath target("/Asset/Shape.points");
    const GatedValues start;
    const UsdStageRefPtr stage =
        MakeGatedStage(start.raise, start.lift, start.moving);
    // The judge reruns every range revision whole after each run.
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "1");
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "0");
    CHECK(compiled);
    if (!compiled) {
        for (const std::string &line : errors) {
            std::printf("  %s\n", line.c_str());
        }
        return;
    }
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(evaluator.Evaluate(time).valid);
    const Program &B = evaluator.GetBakedProgram()->GetStepGraph();
    CHECK(B.verifyRangeChains);
    const int c = FindChain(B, target);
    CHECK(c >= 0);
    if (c < 0) {
        return;
    }
    const Chain &chain = B.chains[size_t(c)];
    const int blend = FindRevision(chain, "Blend");
    const int gated = FindRevision(chain, "Gated");
    CHECK(blend >= 0 && gated > blend);
    CHECK(chain.groupBounds.size() == 11);
    if (blend < 0 || gated <= blend || chain.groupBounds.size() != 11) {
        return;
    }
    const Revision &mover = chain.revisions[size_t(gated)];
    CHECK(WalksSparse(B, mover) && mover.parameters.radialWeight);
    CHECK(!(mover.parameters.transform == GfMatrix4d(1.0)));
    // Gated to the three groups holding its support: 50, 100 and 50 entries.
    for (size_t g = 0; g < mover.groupWritten.size(); ++g) {
        CHECK(bool(mover.groupWritten[g]) == (g < 3));
    }

    // The binary runtime over the same rig, and a reference stage whose
    // authored values are each step's.
    const UsdStageRefPtr bakedStage =
        MakeGatedStage(start.raise, start.lift, start.moving);
    RigExecRigEvaluator bakeEvaluator(bakedStage, SdfPath(kRig));
    std::vector<uint8_t> bytes;
    std::string error;
    RigExecTestPlayer player;
    const bool baked =
        RigExecTestBakeAt(bakeEvaluator, 1.0, &bytes, &error) &&
        player.Open(bytes, bakedStage, &error);
    if (!baked) {
        std::printf("  the gated chain did not bake or open: %s\n",
                    error.c_str());
    }
    CHECK(baked);

    const int first = B.chainRevisionBegin[size_t(c)];
    // Each step: the values, the mover's group steps that run, and the
    // entries they compute.
    struct Step {
        GatedValues values;
        std::vector<int> groups;
        size_t computed;
    };
    const std::vector<Step> steps = {
        // Points outside the support move in every written group. The
        // transform held, so this walk records its 200 entries.
        {{0.6f, 0.0f, 1.0, "raise, recording"}, {0, 1, 2}, 200},
        // The same again: forwarded, nothing computed.
        {{0.75f, 0.0f, 1.0, "raise"}, {0, 1, 2}, 0},
        // Ten supported points of group 1 move.
        {{0.75f, 0.5f, 1.0, "lift"}, {1}, 10},
        // The mover's own transform moves: every entry, none recorded.
        {{0.75f, 0.5f, 2.0, "moving"}, {0, 1, 2}, 200},
        // The first forward at the posed transform records again.
        {{0.25f, 0.5f, 2.0, "raise back, recording"}, {0, 1, 2}, 200},
        {{0.3f, 0.5f, 2.0, "raise at the posed transform"}, {0, 1, 2}, 0},
        // Supported and unsupported points together.
        {{0.5f, 0.25f, 2.0, "raise and lift"}, {0, 1, 2}, 10},
    };
    for (const Step &step : steps) {
        const GatedValues &v = step.values;
        const Snapshot before(chain);
        evaluator.SetInteractiveOverrides(
            {Override(kRaise, "inputs:weight", VtValue(v.raise)),
             Override(kLift, "inputs:weight", VtValue(v.lift)),
             Override(kMoving, "avars:tx", VtValue(v.moving))});
        const RigExecRigPose pose = evaluator.Evaluate(time);
        CHECK(pose.valid);
        const std::set<std::pair<int, int>> ran =
            RanSteps(B, RigExecBakedStepKind::RevisionChunk);
        for (size_t g = 0; g < 10; ++g) {
            const bool expected =
                std::find(step.groups.begin(), step.groups.end(), int(g)) !=
                step.groups.end();
            CHECK(bool(ran.count({first + gated, int(g)})) == expected);
        }
        const WalkTotals totals = CheckWalks(B, c, before, "Gated", v.what);
        CHECK(totals.steps == step.groups.size());
        if (totals.computed != step.computed) {
            std::printf("  %s: computed %zu entries, expected %zu\n", v.what,
                        totals.computed, step.computed);
        }
        CHECK(totals.computed == step.computed);
        CHECK(B.rangeVerifyMismatches == 0);

        // A cold evaluator over the same values authored.
        const UsdStageRefPtr authored = MakeGatedStage(v.raise, v.lift, v.moving);
        RigExecRigEvaluator reference(authored, SdfPath(kRig));
        const bool referenceCompiled = reference.Compile(&errors);
        CHECK(referenceCompiled);
        if (!referenceCompiled) {
            continue;
        }
        const RigExecRigPose expectedPose = reference.Evaluate(time);
        CHECK(expectedPose.valid);
        const bool sameAsCold =
            SameResult(reference.GetBakedProgram()->GetStepGraph(), B, c);
        if (!sameAsCold) {
            std::printf("  %s: the warm chain differs from a cold evaluator's\n",
                        v.what);
        }
        CHECK(sameAsCold);

        // The runtime, driven warm through the same inputs.
        if (baked) {
            std::vector<std::string> diffs;
            const bool played =
                player.Hold(std::string(kRaise) + ".inputs:weight", v.raise,
                            &error) &&
                player.Hold(std::string(kLift) + ".inputs:weight", v.lift,
                            &error) &&
                player.Hold(std::string(kMoving) + ".avars:tx", v.moving,
                            &error) &&
                player.Play(1.0, &error);
            if (!played) {
                std::printf("  %s: the runtime refused: %s\n", v.what,
                            error.c_str());
            }
            CHECK(played);
            const bool same = played && RigExecCompareRuntimeOutputs(
                                            expectedPose, player.Reader(),
                                            &diffs);
            for (const std::string &line : diffs) {
                std::printf("  %s runtime: %s\n", v.what, line.c_str());
            }
            CHECK(same);
        }
    }

    // A forced full re-run at unchanged values skips every sparse walk
    // whole: nothing recomputes, every entry reuses, the walks stay
    // recorded, and the groups keep their bytes and versions.
    {
        const Snapshot rerunBefore(chain);
        std::vector<uint64_t> versions;
        for (size_t g = 0; g < 3; ++g) {
            versions.push_back(mover.groups[g].version);
        }
        evaluator.GetBakedProgram()->RequestFullRun();
        CHECK(evaluator.Evaluate(time).valid);
        const WalkTotals rerun =
            CheckWalks(B, c, rerunBefore, "Gated", "full re-run");
        CHECK(rerun.steps == 3);
        CHECK(rerun.computed == 0);
        if (rerun.reused != 200) {
            std::printf("  full re-run: reused %zu entries, expected 200\n",
                        rerun.reused);
        }
        CHECK(rerun.reused == 200);
        for (size_t g = 0; g < 3; ++g) {
            CHECK(mover.groups[g].walk.skippedWhole == 1);
            CHECK(mover.groups[g].version == versions[g]);
        }
        CHECK(B.rangeVerifyMismatches == 0);
        const GatedValues &last = steps.back().values;
        const UsdStageRefPtr authored =
            MakeGatedStage(last.raise, last.lift, last.moving);
        RigExecRigEvaluator reference(authored, SdfPath(kRig));
        const bool referenceCompiled = reference.Compile(&errors);
        CHECK(referenceCompiled);
        if (referenceCompiled) {
            CHECK(reference.Evaluate(time).valid);
            CHECK(SameResult(reference.GetBakedProgram()->GetStepGraph(), B,
                             c));
        }
    }

    // A frame-cache worker over a copy of the program, memos included,
    // at the authored values again.
    evaluator.ClearInteractiveOverrides();
    std::string why;
    const bool frozen =
        RigExecFrozenMatchesLive(&evaluator, SdfPath(kRig), time, &why);
    if (!frozen) {
        std::printf("  frozen: %s\n", why.c_str());
    }
    CHECK(frozen);
    std::printf("  a gated chain forwards without recomputing its support\n");
}

// --- A paired Range chain --------------------------------------------------

const char *const kPairedMovingA = "/Asset/Rig/MovingA";
const char *const kPairedMovingB = "/Asset/Rig/MovingB";

/// Whether point \p i is in the first / second mover's support: every
/// tenth point of [0, 5000) / [3000, 8000), so the second mover aliases
/// groups the first wrote (0-2) and groups nobody wrote (8-9).
bool
PairedSupportedA(size_t i)
{
    return i < 5000 && i % 10 == 3;
}

/// Whether point \p i is in the second mover's support.
bool
PairedSupportedB(size_t i)
{
    return i >= 3000 && i < 8000 && i % 10 == 3;
}

/// One 10000-point target under two gated radial Matrix movers: `First`,
/// posed by `MovingA`, then `Second`, posed by `MovingB`. \p movingA and
/// \p movingB are the authored control values.
UsdStageRefPtr
MakePairedStage(double movingA, double movingB)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath(kRig), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, double value) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
            .Set(value);
        return prim;
    };
    const UsdPrim controlA = control(kPairedMovingA, movingA);
    const UsdPrim controlB = control(kPairedMovingB, movingB);
    const SdfPath target("/Asset/Shape.points");
    VtVec3fArray base(kPoints);
    for (size_t i = 0; i < kPoints; ++i) {
        base[i] = GfVec3f(float(i % 100), float(i / 100), 0.0f);
    }
    stage->DefinePrim(SdfPath("/Asset/Shape"), TfToken("Points"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(base);
    const auto weight = [&](const char *path, bool (*supported)(size_t)) {
        VtIntArray support;
        VtFloatArray weights;
        for (size_t i = 0; i < kPoints; ++i) {
            if (!supported(i)) {
                continue;
            }
            support.push_back(int(i));
            weights.push_back(float((i / 10) % 5) / 4.0f);
        }
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecStaticWeight"));
        prim.CreateRelationship(TfToken("rigExec:weightTarget"), false)
            .SetTargets({target});
        prim.CreateAttribute(TfToken("rigExec:representation"),
                             SdfValueTypeNames->Token, false)
            .Set(TfToken("sparse"));
        prim.CreateAttribute(TfToken("rigExec:indices"),
                             SdfValueTypeNames->IntArray, false)
            .Set(support);
        prim.CreateAttribute(TfToken("rigExec:values"),
                             SdfValueTypeNames->FloatArray, false)
            .Set(weights);
        prim.CreateAttribute(TfToken("rigExec:defaultWeight"),
                             SdfValueTypeNames->Float, false)
            .Set(0.0f);
        return prim;
    };
    const UsdPrim weightA =
        weight("/Asset/Rig/Weights/SparseA", &PairedSupportedA);
    const UsdPrim weightB =
        weight("/Asset/Rig/Weights/SparseB", &PairedSupportedB);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto mover = [&](const char *name, const UsdPrim &fromControl,
                           const UsdPrim &fromWeight) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecMatrixMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({target});
        prim.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({fromControl.GetPath()});
        prim.CreateRelationship(TfToken("rigExec:weightObject"), false)
            .SetTargets({fromWeight.GetPath()});
        prim.CreateAttribute(TfToken("rigExec:weightBlend"),
                             SdfValueTypeNames->Token)
            .Set(TfToken("radial"));
        return prim;
    };
    // Movers run last sibling first: First, then Second.
    mover("Second", controlB, weightB);
    mover("First", controlA, weightA);
    return stage;
}

/// Whether chain \p c of \p stage, compiled and evaluated cold at its
/// authored values, publishes the bits \p B holds.
bool
MatchesPairedCold(const UsdStageRefPtr &stage, const Program &B, int c,
                  std::vector<std::string> *errors)
{
    RigExecRigEvaluator reference(stage, SdfPath(kRig));
    if (!reference.Compile(errors)) {
        return false;
    }
    if (!reference.Evaluate(UsdTimeCode(1.0)).valid) {
        return false;
    }
    const int rc =
        FindChain(reference.GetBakedProgram()->GetStepGraph(),
                  SdfPath("/Asset/Shape.points"));
    return rc == c &&
           SameResult(reference.GetBakedProgram()->GetStepGraph(), B, c);
}

/// A Range matrix join takes an aliased group's entering source from the
/// predecessor's join output rather than scanning for its writer. The
/// second mover aliases groups the first wrote and groups nobody wrote;
/// a move of its control alone runs its join while the first join skips,
/// so the join reads held notes, and every step still matches a cold
/// evaluator's bits.
void
TestAMatrixJoinReadsPredecessorIds()
{
    const UsdTimeCode time(1.0);
    const SdfPath target("/Asset/Shape.points");
    const UsdStageRefPtr stage = MakePairedStage(1.0, 1.0);
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "1");
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "0");
    CHECK(compiled);
    if (!compiled) {
        for (const std::string &line : errors) {
            std::printf("  %s\n", line.c_str());
        }
        return;
    }
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(evaluator.Evaluate(time).valid);
    const Program &B = evaluator.GetBakedProgram()->GetStepGraph();
    CHECK(B.verifyRangeChains);
    const int c = FindChain(B, target);
    CHECK(c >= 0);
    if (c < 0) {
        return;
    }
    const Chain &chain = B.chains[size_t(c)];
    const int first = FindRevision(chain, "First");
    const int second = FindRevision(chain, "Second");
    CHECK(first == 0 && second == 1);
    CHECK(chain.groupBounds.size() == 11);
    if (first != 0 || second != 1 || chain.groupBounds.size() != 11) {
        return;
    }
    const Revision &moverA = chain.revisions[size_t(first)];
    const Revision &moverB = chain.revisions[size_t(second)];
    CHECK(WalksSparse(B, moverA) && moverA.parameters.radialWeight);
    CHECK(WalksSparse(B, moverB) && moverB.parameters.radialWeight);
    CHECK(!(moverA.parameters.transform == GfMatrix4d(1.0)));
    CHECK(!(moverB.parameters.transform == GfMatrix4d(1.0)));
    // The test's engagement: the second mover aliases a group the first
    // wrote and a group nobody wrote.
    bool aliasedWritten = false, aliasedBase = false;
    for (size_t g = 0; g < moverB.groupWritten.size(); ++g) {
        if (bool(moverB.groupWritten[g])) {
            continue;
        }
        if (g < moverA.groupWritten.size() && moverA.groupWritten[g]) {
            aliasedWritten = true;
        } else {
            aliasedBase = true;
        }
    }
    CHECK(aliasedWritten && aliasedBase);
    if (!aliasedWritten || !aliasedBase) {
        return;
    }
    // An aliased group's id is the predecessor's note: its own slot and
    // version where it wrote, the entering source where it aliased.
    const auto idsMatch = [&] {
        for (size_t g = 0; g < moverB.groupIds.size(); ++g) {
            if (g < moverB.groupWritten.size() && moverB.groupWritten[g]) {
                continue;
            }
            if (g >= moverA.groupIds.size() ||
                !(moverB.groupIds[g] == moverA.groupIds[g])) {
                return false;
            }
        }
        return true;
    };
    CHECK(idsMatch());
    const int idA = B.chainRevisionBegin[size_t(c)] + first;
    const int idB = B.chainRevisionBegin[size_t(c)] + second;
    // Forced full re-runs at rest hold every version and id.
    for (int round = 0; round < 2; ++round) {
        const uint64_t versionA = moverA.doneVersion;
        const uint64_t versionB = moverB.doneVersion;
        const std::vector<RigExecGroupSource> idsA = moverA.groupIds;
        const std::vector<RigExecGroupSource> idsB = moverB.groupIds;
        evaluator.GetBakedProgram()->RequestFullRun();
        CHECK(evaluator.Evaluate(time).valid);
        CHECK(moverA.doneVersion == versionA);
        CHECK(moverB.doneVersion == versionB);
        CHECK(moverA.groupIds == idsA);
        CHECK(moverB.groupIds == idsB);
        CHECK(idsMatch());
        CHECK(B.rangeVerifyMismatches == 0);
    }
    // Only the second control moves, unforced: its join runs while the
    // first join skips, so the join reads held notes.
    {
        const Snapshot before(chain);
        const uint64_t versionB = moverB.doneVersion;
        evaluator.SetInteractiveOverrides(
            {Override(kPairedMovingB, "avars:tx", VtValue(2.0))});
        CHECK(evaluator.Evaluate(time).valid);
        CHECK(B.rangeVerifyMismatches == 0);
        const std::set<std::pair<int, int>> fuses =
            RanSteps(B, RigExecBakedStepKind::RevisionFuse);
        if (!RanFor(fuses, idB) || RanFor(fuses, idA)) {
            std::printf("  second-only move: second fuse %s, first fuse %s\n",
                        RanFor(fuses, idB) ? "ran" : "skipped",
                        RanFor(fuses, idA) ? "ran" : "skipped");
        }
        CHECK(RanFor(fuses, idB));
        CHECK(!RanFor(fuses, idA));
        CHECK(moverB.doneVersion != versionB);
        // Non-vacuous: a written group of the second mover moved bytes.
        bool movedBytes = false;
        for (size_t g = 0; g < moverB.groups.size(); ++g) {
            if (g < moverB.groupWritten.size() && !moverB.groupWritten[g]) {
                continue;
            }
            const std::vector<GfVec3f> &was = before.groups[size_t(second)][g];
            const auto &now = moverB.groups[g].published;
            if (!RigExecPointsBitsEqual(was.data(), was.size(), now.data,
                                        now.count)) {
                movedBytes = true;
                break;
            }
        }
        CHECK(movedBytes);
        CHECK(idsMatch());
        if (!MatchesPairedCold(MakePairedStage(1.0, 2.0), B, c, &errors)) {
            std::printf("  second-only move: the warm chain differs from a "
                        "cold evaluator's\n");
        }
        CHECK(MatchesPairedCold(MakePairedStage(1.0, 2.0), B, c, &errors));
    }
    // The first control moves too: both joins run, and every aliased id
    // still matches the predecessor's note.
    {
        const uint64_t versionA = moverA.doneVersion;
        const uint64_t versionB = moverB.doneVersion;
        evaluator.SetInteractiveOverrides(
            {Override(kPairedMovingA, "avars:tx", VtValue(3.0)),
             Override(kPairedMovingB, "avars:tx", VtValue(2.0))});
        CHECK(evaluator.Evaluate(time).valid);
        CHECK(B.rangeVerifyMismatches == 0);
        const std::set<std::pair<int, int>> fuses =
            RanSteps(B, RigExecBakedStepKind::RevisionFuse);
        CHECK(RanFor(fuses, idA));
        CHECK(RanFor(fuses, idB));
        CHECK(moverA.doneVersion != versionA);
        CHECK(moverB.doneVersion != versionB);
        CHECK(idsMatch());
        if (!MatchesPairedCold(MakePairedStage(3.0, 2.0), B, c, &errors)) {
            std::printf("  both move: the warm chain differs from a cold "
                        "evaluator's\n");
        }
        CHECK(MatchesPairedCold(MakePairedStage(3.0, 2.0), B, c, &errors));
    }
    // Back at the authored values, then a forced full run.
    evaluator.SetInteractiveOverrides(
        {Override(kPairedMovingA, "avars:tx", VtValue(1.0)),
         Override(kPairedMovingB, "avars:tx", VtValue(1.0))});
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(B.rangeVerifyMismatches == 0);
    CHECK(MatchesPairedCold(MakePairedStage(1.0, 1.0), B, c, &errors));
    evaluator.GetBakedProgram()->RequestFullRun();
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(B.rangeVerifyMismatches == 0);
    CHECK(idsMatch());
    std::printf("  a matrix join reads its predecessor's group ids\n");
}

// --- The biped ---------------------------------------------------------------

const char *const kBipedRig = "/Biped/Rig";
const char *const kBody = "/Biped/Geom/body_geo.points";
const char *const kUpArm =
    "/Biped/Rig/Main/Shot/Aux/Controls/M_Body/M_Torso/M_Chest/M_ChestTop/"
    "L_Shldr/L_UpArmSwing/L_UpArm";
const char *const kSkull =
    "/Biped/Rig/Main/Shot/Aux/Controls/M_Body/M_Torso/M_Chest/M_ChestTop/"
    "M_Neck/M_Head/M_HeadGimbal/skull_follow/M_Skull";

/// A cold evaluator of \p stagePath under \p overrides publishes chain
/// \p c's result with the bits \p B holds.
bool
MatchesColdEvaluator(const std::string &stagePath,
                     const std::vector<RigExecValueOverride> &overrides,
                     const Program &B, int c)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    if (!stage) {
        return false;
    }
    RigExecRigEvaluator reference(stage, SdfPath(kBipedRig));
    std::vector<std::string> errors;
    if (!reference.Compile(&errors)) {
        return false;
    }
    reference.SetInteractiveOverrides(overrides);
    if (!reference.Evaluate(UsdTimeCode::Default()).valid) {
        return false;
    }
    return SameResult(reference.GetBakedProgram()->GetStepGraph(), B, c);
}

void
TestTheBipedArmEditForwards(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped_stack.usda";
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const std::string cheekPuff =
        std::string(kSkull) +
        "/M_LoFace/face_mid_compress/cheekPuff_l_auto/L_CheekPuff";
    const std::string cheek =
        std::string(kSkull) + "/M_UpFace/cheek_l_auto/L_Cheek";
    for (const std::string &path : {std::string(kUpArm), cheekPuff, cheek}) {
        CHECK(stage->GetPrimAtPath(SdfPath(path)));
    }
    const UsdTimeCode time = UsdTimeCode::Default();
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "1");
    RigExecRigEvaluator evaluator(stage, SdfPath(kBipedRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    TfSetenv("RIGEXEC_VERIFY_RANGE_CHAINS", "0");
    CHECK(compiled);
    if (!compiled) {
        return;
    }
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(evaluator.Evaluate(time).valid);
    const Program &B = evaluator.GetBakedProgram()->GetStepGraph();
    const int c = FindChain(B, SdfPath(kBody));
    CHECK(c >= 0);
    if (c < 0) {
        return;
    }
    const Chain &chain = B.chains[size_t(c)];
    const std::string focus = "cheekPuff_l_cluster";
    const int r = FindRevision(chain, focus);
    CHECK(r >= 0 && chain.groupBounds.size() >= 3);
    if (r < 0 || chain.groupBounds.size() < 3) {
        return;
    }
    CHECK(WalksSparse(B, chain.revisions[size_t(r)]));
    const int first = B.chainRevisionBegin[size_t(c)];
    // Whether the focus mover's own packet was reassembled by the last run.
    const auto reassembled = [&] {
        return RanFor(RanSteps(B, RigExecBakedStepKind::RevisionStatic),
                      first + r);
    };
    const auto edit = [&](const std::vector<RigExecValueOverride> &overrides,
                          Snapshot *before) {
        *before = Snapshot(chain);
        evaluator.SetInteractiveOverrides(overrides);
        CHECK(evaluator.Evaluate(time).valid);
        CHECK(B.rangeVerifyMismatches == 0);
    };
    Snapshot before(chain);

    // An arm-only edit, warmed: the first step records the walks it reruns.
    // The cheek cluster's own inputs stand; the groups it reruns enter with
    // moved points, none in its support; so it forwards them and computes
    // no entry.
    std::vector<RigExecValueOverride> overrides = {
        Override(kUpArm, "avars:rz", VtValue(5.0))};
    edit(overrides, &before);
    CheckWalks(B, c, before, std::string(), "arm, recording");
    overrides = {Override(kUpArm, "avars:rz", VtValue(12.0))};
    edit(overrides, &before);
    CHECK(!reassembled());
    const WalkTotals armFocus = CheckWalks(B, c, before, focus, "arm");
    CHECK(armFocus.steps > 0);
    CHECK(armFocus.computed == 0 && armFocus.reused > 0);
    const WalkTotals arm = CheckWalks(B, c, before, std::string(), "arm");
    CHECK(arm.forwards > 0 && arm.reused > arm.computed);
    CHECK(MatchesColdEvaluator(stagePath, overrides, B, c));
    std::printf("  arm edit: %zu sparse group step(s) ran, %zu computed "
                "nothing; %zu entries computed, %zu reused\n",
                arm.steps, arm.forwards, arm.computed, arm.reused);

    // The cheek cluster's own control: every entry of every group it writes.
    overrides.push_back(Override(cheekPuff.c_str(), "avars:tx", VtValue(0.4)));
    edit(overrides, &before);
    const WalkTotals own = CheckWalks(B, c, before, focus, "cheek puff");
    size_t written = 0;
    for (const char flag : chain.revisions[size_t(r)].groupWritten) {
        written += flag ? 1 : 0;
    }
    CHECK(own.steps == written && own.reused == 0 && own.computed > 0);
    CHECK(MatchesColdEvaluator(stagePath, overrides, B, c));

    // The next arm step records the cheek cluster's walks at its new
    // transform.
    overrides[0] = Override(kUpArm, "avars:rz", VtValue(14.0));
    edit(overrides, &before);
    const WalkTotals again = CheckWalks(B, c, before, focus, "arm, recording");
    CHECK(again.steps > 0 && again.reused == 0);

    // A cluster upstream of it in the chain moves points it weights: those
    // entries and no others.
    overrides.push_back(Override(cheek.c_str(), "avars:tx", VtValue(0.3)));
    edit(overrides, &before);
    const WalkTotals shared = CheckWalks(B, c, before, focus, "cheek");
    CHECK(shared.steps > 0 && shared.computed > 0);
    CHECK(reassembled() || shared.reused > 0);
    CHECK(MatchesColdEvaluator(stagePath, overrides, B, c));

    // The arm again, with the face posed: forwarded as before, once a step
    // has recorded whatever the cheek edit left unrecorded.
    overrides[0] = Override(kUpArm, "avars:rz", VtValue(17.0));
    edit(overrides, &before);
    CheckWalks(B, c, before, std::string(), "arm, face posed, recording");
    overrides[0] = Override(kUpArm, "avars:rz", VtValue(20.0));
    edit(overrides, &before);
    CHECK(!reassembled());
    const WalkTotals posed = CheckWalks(B, c, before, focus, "arm, face posed");
    CHECK(posed.steps > 0 && posed.computed == 0 && posed.reused > 0);
    CheckWalks(B, c, before, std::string(), "arm, face posed");
    CHECK(MatchesColdEvaluator(stagePath, overrides, B, c));
    std::printf("  the biped's cheek cluster forwards an arm edit and follows "
                "its own\n");
}

// --- A Legacy chain ------------------------------------------------------------

/// 600 points sit under the 1024-point group target, so the chain never
/// pipelines: every revision stays Legacy. The test asserts that.
constexpr size_t kLegacyPoints = 600;
const SdfPath kLegacyTarget("/Asset/LegacyShape.points");
const char *const kLegacyRaise = "/Asset/Rig/BlendInputs/LegacyRaise";
const char *const kResting = "/Asset/Resting";

/// One 600-point target under a blend shape and then two sparse linear
/// Matrix movers: `Legacy`, posed by `Moving`, and `LegacyRest` at rest.
/// \p raise and \p moving are the authored values; the rest control stays
/// at zero, so its mover walks the identity shortcut and never begins a
/// memo walk.
UsdStageRefPtr
MakeLegacyStage(float raise, double moving)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath(kRig), TfToken("RigExecRoot"));
    const UsdPrim control =
        stage->DefinePrim(SdfPath(kMoving), TfToken("RigExecControl"));
    control.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(moving);
    const UsdPrim rest =
        stage->DefinePrim(SdfPath(kResting), TfToken("RigExecControl"));
    rest.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(0.0);
    VtVec3fArray base(kLegacyPoints), raised(kLegacyPoints);
    VtIntArray support;
    VtFloatArray weights;
    for (size_t i = 0; i < kLegacyPoints; ++i) {
        base[i] = GfVec3f(float(i % 30), float(i / 30), 0.0f);
        raised[i] = base[i];
        if (i % 5 == 0) {
            raised[i][2] += 1.0f;
        }
        if (i >= 100 && i < 500 && i % 11 == 4) {
            support.push_back(int(i));
            weights.push_back(float((i / 11) % 4) / 3.0f);
        }
    }
    const auto points = [&](const char *path, const VtVec3fArray &values) {
        stage->DefinePrim(SdfPath(path), TfToken("Points"))
            .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
            .Set(values);
    };
    points("/Asset/LegacyShape", base);
    points("/Asset/LegacyRaised", raised);
    const UsdPrim input = stage->DefinePrim(SdfPath(kLegacyRaise),
                                            TfToken("RigExecBlendInput"));
    input.CreateAttribute(TfToken("inputs:weight"), SdfValueTypeNames->Float)
        .Set(raise);
    const UsdPrim sample = stage->DefinePrim(
        SdfPath(kLegacyRaise).AppendChild(TfToken("Full")),
        TfToken("RigExecBlendSample"));
    sample.CreateAttribute(TfToken("rigExec:activation"),
                           SdfValueTypeNames->Float)
        .Set(1.0f);
    sample.CreateRelationship(TfToken("rigExec:targetPoints"))
        .SetTargets(
            {SdfPath("/Asset/LegacyRaised").AppendProperty(TfToken("points"))});
    input.CreateRelationship(TfToken("rigExec:samples"))
        .SetTargets({sample.GetPath()});
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/LegacySparse"),
        TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({kLegacyTarget});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    weight.CreateAttribute(TfToken("rigExec:indices"),
                           SdfValueTypeNames->IntArray, false)
        .Set(support);
    weight.CreateAttribute(TfToken("rigExec:values"),
                           SdfValueTypeNames->FloatArray, false)
        .Set(weights);
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float, false)
        .Set(0.0f);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto mover = [&](const char *name, const SdfPath &provider) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecMatrixMover"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({kLegacyTarget});
        prim.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({provider});
        prim.CreateRelationship(TfToken("rigExec:weightObject"), false)
            .SetTargets({weight.GetPath()});
        return prim;
    };
    // Movers run last sibling first: the blend, then the posed mover,
    // then the resting one.
    mover("LegacyRest", rest.GetPath());
    mover("Legacy", control.GetPath());
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend"), TfToken("RigExecBlendShapeMover"));
    CHECK(blend.ApplyAPI(TfToken("RigExecMoverAPI")));
    blend.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({kLegacyTarget});
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({SdfPath(kLegacyRaise)});
    blend.CreateAttribute(TfToken("inputs:defaultWeight"),
                          SdfValueTypeNames->Float)
        .Set(1.0f);
    return stage;
}

/// Whether revision \p r of chain \p c holds the bytes \p reference left.
bool
SameLegacyResult(const Program &reference, const Program &B, int c, int r)
{
    const std::vector<GfVec3f> &left =
        reference.chains[size_t(c)].revisions[size_t(r)].output;
    const std::vector<GfVec3f> &right =
        B.chains[size_t(c)].revisions[size_t(r)].output;
    return RigExecPointsBitsEqual(left.data(), left.size(), right.data(),
                                  right.size());
}

void
TestALegacyChainSkipsWhole()
{
    const UsdTimeCode time(1.0);
    const float raise = 0.5f;
    const double moving = 1.0;
    const UsdStageRefPtr stage = MakeLegacyStage(raise, moving);
    RigExecRigEvaluator evaluator(stage, SdfPath(kRig));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
    CHECK(compiled);
    if (!compiled) {
        for (const std::string &line : errors) {
            std::printf("  %s\n", line.c_str());
        }
        return;
    }
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(evaluator.Evaluate(time).valid);
    const Program &B = evaluator.GetBakedProgram()->GetStepGraph();
    const int c = FindChain(B, kLegacyTarget);
    CHECK(c >= 0);
    if (c < 0) {
        return;
    }
    const Chain &chain = B.chains[size_t(c)];
    const int blend = FindRevision(chain, "Blend");
    const int legacy = FindRevision(chain, "Legacy");
    const int rest = FindRevision(chain, "LegacyRest");
    CHECK(blend == 0 && legacy == 1 && rest == 2);
    CHECK(chain.groupBounds.size() < 2);
    if (blend != 0 || legacy != 1 || rest != 2) {
        return;
    }
    const Revision &posed = chain.revisions[size_t(legacy)];
    const Revision &resting = chain.revisions[size_t(rest)];
    for (const Revision *revision : {&posed, &resting}) {
        CHECK(revision->op == RigExecRevisionOp::Matrix);
        CHECK(revision->role == RigExecBakedRevisionRole::Legacy);
    }
    CHECK(!(posed.parameters.transform == GfMatrix4d(1.0)));
    CHECK(resting.parameters.transform == GfMatrix4d(1.0));
    const size_t entries = posed.parameters.weights.indices.size();
    CHECK(entries > 0 &&
          entries == resting.parameters.weights.indices.size());
    // A forced full re-run at unchanged values skips both walks whole:
    // nothing seeds, walks or publishes, and the revisions keep their
    // bytes, versions and sources. The posed walk reports its entries, as
    // a reusing walk does; the resting one reports none, as the identity
    // shortcut it stands in for does.
    for (int round = 0; round < 2; ++round) {
        const std::vector<GfVec3f> posedBefore = posed.output;
        const std::vector<GfVec3f> restBefore = resting.output;
        const uint64_t posedVersion = posed.doneVersion;
        const uint64_t restVersion = resting.doneVersion;
        evaluator.GetBakedProgram()->RequestFullRun();
        CHECK(evaluator.Evaluate(time).valid);
        CHECK(posed.matrixWalk.skippedWhole == 1);
        CHECK(posed.matrixWalk.computed == 0);
        CHECK(posed.matrixWalk.reused == entries);
        CHECK(resting.matrixWalk.skippedWhole == 1);
        CHECK(resting.matrixWalk.computed == 0);
        CHECK(resting.matrixWalk.reused == 0);
        CHECK(posed.doneVersion == posedVersion);
        CHECK(resting.doneVersion == restVersion);
        CHECK(posed.currentSource == legacy);
        CHECK(resting.currentSource == rest);
        CHECK(RigExecPointsBitsEqual(posed.output.data(), posed.output.size(),
                                     posedBefore.data(), posedBefore.size()));
        CHECK(RigExecPointsBitsEqual(resting.output.data(),
                                     resting.output.size(), restBefore.data(),
                                     restBefore.size()));
    }
    // Moved inputs walk, and match a cold evaluator's bytes: the
    // transform first, then the entering points under a held transform.
    struct Step {
        float raise;
        double moving;
        const char *what;
    };
    for (const Step &step :
         {Step{raise, 2.0, "transform moves"},
          Step{0.75f, 2.0, "entering moves"},
          Step{raise, moving, "values back"}}) {
        evaluator.SetInteractiveOverrides(
            {Override(kLegacyRaise, "inputs:weight", VtValue(step.raise)),
             Override(kMoving, "avars:tx", VtValue(step.moving))});
        evaluator.GetBakedProgram()->RequestFullRun();
        CHECK(evaluator.Evaluate(time).valid);
        CHECK(posed.matrixWalk.skippedWhole == 0);
        const UsdStageRefPtr authored =
            MakeLegacyStage(step.raise, step.moving);
        RigExecRigEvaluator reference(authored, SdfPath(kRig));
        const bool referenceCompiled = reference.Compile(&errors);
        CHECK(referenceCompiled);
        if (!referenceCompiled) {
            continue;
        }
        CHECK(reference.Evaluate(time).valid);
        const Program &R =
            reference.GetBakedProgram()->GetStepGraph();
        const bool same =
            SameLegacyResult(R, B, c, legacy) && SameLegacyResult(R, B, c,
                                                                  rest);
        if (!same) {
            std::printf("  %s: the warm chain differs from a cold "
                        "evaluator's\n",
                        step.what);
        }
        CHECK(same);
    }
    // Back at standing inputs the walks skip whole again.
    evaluator.GetBakedProgram()->RequestFullRun();
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(posed.matrixWalk.skippedWhole == 1);
    CHECK(resting.matrixWalk.skippedWhole == 1);
    std::printf("  a legacy chain skips its matrix walks whole\n");
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecGroupForwarding <examplesDir>\n");
        return 2;
    }
    if (PlugRegistry::GetInstance()
            .RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR)
            .empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    RIGEXEC_SCHEMA_RESOURCE_DIR);
        return 2;
    }
    TestAGatedChainForwards();
    TestAMatrixJoinReadsPredecessorIds();
    TestALegacyChainSkipsWhole();
    TestTheBipedArmEditForwards(argv[1]);
    if (failures) {
        std::printf("testRigExecGroupForwarding: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecGroupForwarding: OK\n");
    return 0;
}
