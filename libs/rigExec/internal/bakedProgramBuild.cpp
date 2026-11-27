// Building a baked program: the build context, the driver and the steps up
// to the input binding table.
#include "rigExec/internal/weightField.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/inputReplay.h"

#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/bakedProgramShared.h"
#include "rigExec/internal/bakedBuildAttempt.h"
#include "rigExec/internal/projectorCaptureNotice.h"
#include "rigExec/api/sceneDispatch.h"
#include "rigExec/internal/oracleDispatch.h"
#include "rigExec/internal/bakedSchedule.h"
#include "rigExec/internal/bakedOpGraph.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/frozenContextInternal.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/api/moverRegistry.h"
#include "rigExec/internal/movers/pathOperation.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/api/types.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecSchema/schemaTokens.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

using namespace bakedProgramDetail;

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((xyz, SchemaValueText::xyz))
);

// The channels Build's parallel resolve lanes bind. They are the interned
// SchemaAttr tokens, so no lane builds a token from text.
const TfToken &kPosedSpace = SchemaAttr::posedSpace;
const TfToken &kRestSpace = SchemaAttr::restSpace;
const TfToken &kDefaultSpace = SchemaAttr::defaultSpace;
const TfToken &kParentSpace = SchemaAttr::parentSpace;
const TfToken &kParentDefaultSpace = SchemaAttr::parentDefaultSpace;
const TfToken &kAvarDefaultSpace = SchemaAttr::avarsDefaultSpace;
const TfToken &kPosedDefaultSpace = SchemaAttr::posedDefaultSpace;
const TfToken &kRotationSign = SchemaAttr::avarsRotationSign;
const TfToken &kRotationOrder = SchemaAttr::avarsRotationOrder;
const TfToken kRestAvarTokens[6] = {SchemaAttr::restTx, SchemaAttr::restTy,
                                    SchemaAttr::restTz, SchemaAttr::restRx,
                                    SchemaAttr::restRy, SchemaAttr::restRz};
const TfToken kDefaultAvarTokens[6] = {
    SchemaAttr::defaultTx, SchemaAttr::defaultTy, SchemaAttr::defaultTz,
    SchemaAttr::defaultRx, SchemaAttr::defaultRy, SchemaAttr::defaultRz};
// RigExecBakedAvarNames, in its order.
static_assert(std::size(RigExecBakedAvarNames) == 11,
              "kAvarTokens interns every avar name");
const TfToken kAvarTokens[11] = {SchemaAttr::avarsTx,
                                 SchemaAttr::avarsTy,
                                 SchemaAttr::avarsTz,
                                 SchemaAttr::avarsSx,
                                 SchemaAttr::avarsSy,
                                 SchemaAttr::avarsSz,
                                 SchemaAttr::avarsRx,
                                 SchemaAttr::avarsRy,
                                 SchemaAttr::avarsRz,
                                 SchemaAttr::avarsRspin,
                                 SchemaAttr::avarsUnitScaleFactor};

// A plain attribute read with no time and no resolution walk, for uniform
// tokens (rigExec:rotationOrder, rigExec:worldUpType, rigExec:aimAxis, ...).
TfToken
_ReadToken(const UsdPrim &prim, const TfToken &name, const TfToken &fallback)
{
    TfToken value = fallback;
    if (prim) {
        if (const UsdAttribute a = prim.GetAttribute(name)) {
            TfToken read;
            if (a.Get(&read) && !read.IsEmpty()) {
                value = read;
            }
        }
    }
    return value;
}

} // namespace

// Build.

void
RigExecBakedBuildContext::Refuse(const std::string &what, const SdfPath &where)
{
    if (reasons)
        reasons->push_back(what + ": " + where.GetString());
    ok = false;
}

void
RigExecBakedBuildContext::Fold(const UsdPrim &prim, const TfToken &name)
{
    RigExecBakedProgramSink sink{program};
    RigExecBakedRecordFold(&sink, prim, name);
}

void
RigExecBakedCommitShard::Seal()
{
    for (SdfPathVector *paths : {&prims, &named, &rebuild, &folded, &live}) {
        std::sort(paths->begin(), paths->end());
        paths->erase(std::unique(paths->begin(), paths->end()), paths->end());
    }
    // Not deduplicated: one input names each walk step once (the walk
    // stops at the first attribute it revisits), so an equal pair is two
    // inputs and both numbers belong in the entry.
    std::sort(overridable.begin(), overridable.end());
}

std::vector<int>
RigExecBakedMergeCommitShards(RigExecBakedProgramImpl *program,
                              std::vector<RigExecBakedCommitShard> *shards)
{
    RigExecBakedProgramImpl &B = *program;
    std::vector<int> first(shards->size(), 0);
    int next = int(B.overridden.size());
    for (size_t k = 0; k < shards->size(); ++k) {
        first[k] = next;
        next += (*shards)[k].numbered;
        B.boundInputs += (*shards)[k].boundInputs;
        B.varyingInputs += (*shards)[k].varyingInputs;
    }
    // Every flag starts clear, so growing the vector is the same as the
    // serial commit's one push_back per number.
    B.overridden.resize(size_t(next), 0);
    // Each shard's paths are sorted, so each insert is hinted at the
    // successor of the one before it -- constant time wherever the set
    // holds nothing in between, a plain insert wherever it does. The hint
    // only decides the cost, never the contents.
    const auto insertSorted = [](std::set<SdfPath> *set,
                                 const SdfPathVector &paths) {
        auto hint = set->end();
        for (const SdfPath &path : paths) {
            hint = std::next(set->insert(hint, path));
        }
    };
    for (RigExecBakedCommitShard &shard : *shards) {
        insertSorted(&B.prims, shard.prims);
        insertSorted(&B.named, shard.named);
        insertSorted(&B.rebuild, shard.rebuild);
        insertSorted(&B.folded, shard.folded);
        insertSorted(&B.liveInputPaths, shard.live);
    }
    // In chunk order and, inside a shard, by (path, number): every number
    // shard k hands out is below every number shard k+1 does, and every
    // number the phase hands out is above whatever an earlier phase put in
    // an entry, so each entry grows in exactly the ascending order the
    // serial commit appends in.
    for (size_t k = 0; k < shards->size(); ++k) {
        auto hint = B.overridableInputs.end();
        for (const auto &[path, relative] : (*shards)[k].overridable) {
            const auto it = B.overridableInputs.try_emplace(hint, path);
            it->second.push_back(first[k] + relative);
            hint = std::next(it);
        }
    }
    return first;
}

TfToken
RigExecBakedBuildContext::ReadToken(const UsdPrim &prim, const TfToken &name,
                                    const TfToken &fallback)
{
    Fold(prim, name);
    return _ReadToken(prim, name, fallback);
}

SdfPathVector
RigExecBakedBuildContext::Targets(const UsdPrim &prim, const TfToken &name)
{
    Fold(prim, name);
    return _Targets(prim, name);
}

int
RigExecBakedBuildContext::SlotOf(const SdfPath &path) const
{
    const auto it = program->index.find(path);
    return it == program->index.end() ? -1 : it->second;
}

std::unique_ptr<RigExecBakedProgram>
RigExecBakedProgram::Build(RigExecRigEvaluator *evaluator,
                           std::vector<std::string> *reasons)
{
    return _BuildWithSceneCapture(evaluator, reasons, nullptr,
                                  UsdStageWeakPtr(), 0);
}

std::unique_ptr<RigExecBakedProgram>
RigExecBakedProgram::_BuildWithSceneCapture(
    RigExecRigEvaluator *evaluator, std::vector<std::string> *reasons,
    const RigExecSceneDescriptors *scene, const UsdStageWeakPtr &sceneStage,
    uint64_t sceneSerial)
{
    // Every candidate remains unpublished. Normalize only actual canonical
    // SCC writer owners, then rebuild all frame bindings and version tables
    // from the same authored facts. A removed writer never revives its Solve.
    if (!evaluator)
        return nullptr;
    const uint64_t sourceSerial = evaluator->_stageEditSerial;
    std::set<SdfPath> excludedPoseWriters, excludedSolverBodies;
    RigExecOpExclusionProof exclusionProof;
    std::map<SdfPath, std::string> cycleSkipReasons;
    for (;;) {
        auto candidate = _BuildWithSceneCaptureAttempt(
            evaluator, reasons, scene, sceneStage, sceneSerial,
            excludedPoseWriters, excludedSolverBodies, exclusionProof,
            cycleSkipReasons);
        if (!candidate)
            return nullptr;
        if (evaluator->_stageEditSerial != sourceSerial) {
            if (reasons)
                reasons->push_back(
                    "source changed during cycle writer normalization");
            return nullptr;
        }
        auto &B = *candidate->_impl;
        std::set<SdfPath> newWriters;
        using K = RigExecBakedStepKind;
        for (const auto &step : B.excludedSteps) {
            const RigExecBakedStepIdentity id = RigExecBakedIdentityOf(B, step);
            if (id.kind != K::SolverCommit && id.kind != K::Constraint &&
                id.kind != K::CommitApply)
                continue;
            if (id.object < 0 || size_t(id.object) >= B.commits.size() ||
                size_t(id.object) >= B.walkSteps.size()) {
                if (reasons)
                    reasons->push_back("cycle writer has no authored binding");
                return nullptr;
            }
            const auto &walk = B.walkSteps[size_t(id.object)];
            if (walk.solverBatch) {
                if (walk.batchSolvers.empty()) {
                    if (reasons)
                        reasons->push_back(
                            "cycle writer has an empty solver batch");
                    return nullptr;
                }
                // A batch is one atomic application: remove its full joint
                // ownership, not merely the solver named first by its key.
                for (int index : walk.batchSolvers) {
                    if (index < 0 || size_t(index) >= B.solvers.size())
                        return nullptr;
                    newWriters.insert(B.solvers[size_t(index)].path);
                }
            } else {
                const SdfPath &owner = B.commits[size_t(id.object)].moverPath;
                if (owner.IsEmpty())
                    return nullptr;
                newWriters.insert(owner);
            }
        }
        bool progress = false;
        for (const auto &step : B.excludedSteps)
            if (const int si = RigExecBakedSolverOf(B, step); si >= 0) {
                if (size_t(si) >= B.solvers.size())
                    return nullptr;
                progress =
                    excludedSolverBodies.insert(B.solvers[size_t(si)].path)
                        .second ||
                    progress;
            }
        for (const auto &owner : newWriters)
            progress = excludedPoseWriters.insert(owner).second || progress;
        if (!progress) {
            if (!newWriters.empty()) {
                if (reasons)
                    reasons->push_back(
                        "normalized cycle writer still has an application");
                return nullptr;
            }
            // Bodies index the spelled slot paths without a check.
            if (!TF_VERIFY(B.pathTexts &&
                           B.pathTexts->size() == B.paths.size())) {
                if (reasons)
                    reasons->push_back(
                        "slot path texts do not match the slot table");
                return nullptr;
            }
            return candidate;
        }
        std::map<SdfPath, SdfPath> switchOwners;
        for (const auto &sw : evaluator->_spaceSwitches)
            switchOwners.emplace(sw.target, sw.switchPath);
        // The getter merges earlier snapshots with every new SCC reason
        // while its current descriptor arrays still match the report.
        cycleSkipReasons = candidate->_GetCycleSkips(switchOwners);
        const auto withdrawnGeometryOwner = [&](const SdfPath &owner) {
            if (owner.IsEmpty())
                return false;
            bool found = false;
            for (const auto *chains :
                 {&evaluator->_graphChains, &evaluator->_graphDerivedChains})
                for (const auto &chain : *chains)
                    for (const auto &revision : chain.second)
                        if (revision.moverPath == owner &&
                            excludedSolverBodies.count(
                                revision.binding.driverFrames))
                            found = true;
            return found;
        };
        const auto withdrawnCommit = [&](int index) {
            if (index < 0 || size_t(index) >= B.commits.size() ||
                size_t(index) >= B.walkSteps.size())
                return false;
            const auto &walk = B.walkSteps[size_t(index)];
            if (walk.solverBatch)
                return std::any_of(
                    walk.batchSolvers.begin(), walk.batchSolvers.end(),
                    [&](int solver) {
                        return solver >= 0 &&
                               size_t(solver) < B.solvers.size() &&
                               excludedPoseWriters.count(
                                   B.solvers[size_t(solver)].path) != 0;
                    });
            return excludedPoseWriters.count(
                       B.commits[size_t(index)].moverPath) != 0;
        };
        const auto withdrawnCommitHelper = [&](const RigExecBakedStep &step) {
            if (step.kind == K::FrameMatrix && step.object >= 0 &&
                size_t(step.object) < B.frameRecords.size())
                return withdrawnCommit(
                    B.frameRecords[size_t(step.object)].commit);
            if (const int f = RigExecBakedWeightFieldOf(B, step); f >= 0) {
                const auto &field = B.weightFields[size_t(f)];
                return field.form == RigExecBakedProgramImpl::WeightField::
                                         Form::EnvelopeConstraint &&
                       withdrawnCommit(field.consumer);
            }
            return false;
        };
        const auto withdrawnGeometryRole = [&](const RigExecBakedStep &step) {
            // Only descriptors belonging to the exact optional revision whose
            // captured driverFrames source became unavailable are withdrawn.
            SdfPath owner;
            const auto revisionOwner = [&](int index) {
                if (index < 0 || size_t(index) >= B.revisionIndex.size())
                    return SdfPath();
                const auto &revision = B.revisionIndex[size_t(index)];
                return B.chains[size_t(revision.first)]
                    .revisions[size_t(revision.second)]
                    .moverPath;
            };
            if (const int f = RigExecBakedWeightFieldOf(B, step);
                f >= 0 &&
                B.weightFields[size_t(f)].form ==
                    RigExecBakedProgramImpl::WeightField::Form::Revision)
                owner = revisionOwner(B.weightFields[size_t(f)].consumer);
            switch (step.kind) {
            case K::SkinTopology:
                if (step.object >= 0)
                    if (const auto *revision =
                            RigExecBakedLayoutRevision(B, size_t(step.object)))
                        owner = revision->moverPath;
                break;
            case K::InfluenceFold:
            case K::RevisionStatic:
            case K::RevisionChunk:
            case K::RevisionFuse:
                owner = revisionOwner(step.object);
                break;
            case K::Derived:
                if (step.object >= 0 &&
                    size_t(step.object) < B.derivedIndex.size()) {
                    const auto &derived = B.derivedIndex[size_t(step.object)];
                    owner = B.chains[size_t(derived.first)]
                                .derived[size_t(derived.second)]
                                .revision.moverPath;
                }
                break;
            case K::FrameMatrix:
                if (step.object >= 0 &&
                    size_t(step.object) < B.frameRecords.size())
                    owner = B.frameRecords[size_t(step.object)].mover;
                break;
            default:
                break;
            }
            return withdrawnGeometryOwner(owner);
        };
        for (const auto &step : B.excludedSteps) {
            if (step.descriptorKey.empty())
                return nullptr;
            const K kind = RigExecBakedIdentityOf(B, step).kind;
            const bool withdrawn =
                withdrawnCommitHelper(step) || withdrawnGeometryRole(step) ||
                kind == K::SolverCommit || kind == K::Constraint ||
                kind == K::CommitApply;
            if (withdrawn) {
                // A later discovered unavailable driver may explicitly
                // withdraw an optional geometry role retained earlier.
                auto &retained = exclusionProof.keys;
                retained.erase(std::remove(retained.begin(), retained.end(),
                                           step.descriptorKey),
                               retained.end());
            }
            auto &keys =
                withdrawn ? exclusionProof.removedKeys : exclusionProof.keys;
            if (std::find(keys.begin(), keys.end(), step.descriptorKey) ==
                keys.end())
                keys.push_back(step.descriptorKey);
        }
        // SCC report stays aligned to authored slots; no generated edges kept.
        exclusionProof.cycles = B.opGraph.cycles;
        exclusionProof.cycleMembers = B.opGraph.cycleMembers;
    }
}

namespace {

template <class Input>
void
_RenumberOverride(Input *input, int first)
{
    if (input->overrideIndex >= 0) {
        input->overrideIndex += first;
    }
}

} // namespace

std::unique_ptr<RigExecBakedProgram>
RigExecBakedProgram::_BuildWithSceneCaptureAttempt(
    RigExecRigEvaluator *evaluator, std::vector<std::string> *reasons,
    const RigExecSceneDescriptors *scene, const UsdStageWeakPtr &sceneStage,
    uint64_t sceneSerial, const std::set<SdfPath> &excludedPoseWriters,
    const std::set<SdfPath> &excludedSolverBodies,
    const RigExecOpExclusionProof &exclusionProof,
    const std::map<SdfPath, std::string> &cycleSkipReasons)
{
    RigExecInputReplayComparisonScope replayComparison(
        "RigExecBakedProgram::Build");
    if (!evaluator) {
        return nullptr;
    }
    _BuildAttempt build(*evaluator, &evaluator->_profiler, reasons, scene,
                        sceneStage, sceneSerial, excludedPoseWriters,
                        excludedSolverBodies, exclusionProof, cycleSkipReasons);
    if (!_BuildCheckBakeable(build)) {
        return nullptr;
    }
    _BuildCreateProgram(build);
    // Where Build spends its time, part by part. The parts run one
    // after another in this block rather than nested, so they are
    // marked rather than scoped; see RigExecProfilePhases.
    build.phases.Next("Bake.entry");
    _BuildEntry(build);
    build.phases.Next("Bake.dense_provider_slots");
    _BuildProviderSlots(build);
    build.phases.Next("Bake.the_rest_chain_and_the_default-space_ladder");
    if (!_BuildLadder(build)) {
        return nullptr;
    }
    build.phases.Next("Bake.the_input_binding_table");
    _BuildInputTable(build);
    build.phases.Next("Bake.pose_descriptors");
    _BuildPoseDescriptors(build);
    build.phases.Next("Bake.publication");
    _BuildPublication(build);
    build.phases.Next("Bake.geometry");
    build.phases.Next("Bake.pose_interpolators");
    if (!_BuildPoseInterpolators(build)) {
        return nullptr;
    }
    _BuildGeometry(build);
    build.phases.Next("Bake.the_rest_of_the_invalidation_index");
    if (!_BuildInvalidationIndex(build)) {
        return nullptr;
    }
    build.phases.Next("Bake.the_step_graph");
    return _BuildStepGraph(build);
}

RigExecBakedProgram::_BuildAttempt::_BuildAttempt(
    RigExecRigEvaluator &E, RigExecProfiler *profiler,
    std::vector<std::string> *reasons, const RigExecSceneDescriptors *scene,
    const UsdStageWeakPtr &sceneStage, uint64_t sceneSerial,
    const std::set<SdfPath> &excludedPoseWriters,
    const std::set<SdfPath> &excludedSolverBodies,
    const RigExecOpExclusionProof &exclusionProof,
    const std::map<SdfPath, std::string> &cycleSkipReasons)
    : E(E), evaluator(&E), reasons(reasons), scene(scene),
      sceneStage(sceneStage), sceneSerial(sceneSerial),
      excludedPoseWriters(excludedPoseWriters),
      excludedSolverBodies(excludedSolverBodies),
      exclusionProof(exclusionProof), cycleSkipReasons(cycleSkipReasons),
      phases(profiler, "compile"), bindPhases(profiler, "compile")
{
}

bool
RigExecBakedProgram::_BuildCheckBakeable(_BuildAttempt &build)
{
    auto &reasons = build.reasons;
    auto &phases = build.phases;
    auto &bakeableLane = build.bakeableLane;
    RigExecRigEvaluator &E = build.E;
    auto &nativeCheckpoints = build.nativeCheckpoints;
    for (const auto &[target, writers] : E._frameChains)
        nativeCheckpoints[target].insert(writers.begin(), writers.end());
    // Bakeability walk is a task joined before the ladder commit (which hands
    // out override indices); work before the join writes only the program and
    // `ctx`, discarded on refusal. Refusals in a separate vector, merged in
    // program order; on a refusal only the walk's reasons are returned.
    // Inline when parallel eval is off, in a frozen run, or uncompiled.
    auto &bakeableBeside = build.bakeableBeside;
    bakeableBeside = E._rigState.Compiled() &&
                     RigExecParallelEvaluationEnabled() &&
                     !RigExecFrozenSerialActive();
    auto &bakeable = build.bakeable;
    auto &bakeableReasons = build.bakeableReasons;
    auto &buildReasons = build.buildReasons;
    if (bakeableBeside) {
        bakeableLane.Run([&E, &bakeable, &bakeableReasons] {
            RigExecProfileScope scope(&E._profiler, "Bake.IsBakeable",
                                      "compile");
            bakeable = IsBakeable(E, &bakeableReasons);
        });
    } else {
        phases.Next("Bake.IsBakeable");
        if (!IsBakeable(E, reasons)) {
            return false;
        }
    }
    return true;
}

void
RigExecBakedProgram::_BuildCreateProgram(_BuildAttempt &build)
{
    auto &evaluator = build.evaluator;
    RigExecRigEvaluator &E = build.E;
    auto &impl = build.impl;
    impl = std::make_unique<RigExecBakedProgramImpl>();
    RigExecBakedProgramImpl &B = *impl;
    B.evaluator = evaluator;
    B.stage = E._stage;
    // What the bodies would otherwise read from the environment or a
    // function-local static, read once here (bodyPurity.h).
    B.chunkVertexTarget = RigExecBakedChunkVertexTargetFromEnvironment();
    B.gpuDeformStats = TfGetenvBool("RIGEXEC_GPU_DEFORM_STATS", false);
    RigExecBakedReadScheduleSettings(&B);
    B.inputReplayCapture = RigExecInputReplayCaptureRequested();
    B.chunkCap = RigExecBakedChunkCapFromEnvironment();
    B.rangeChains = RigExecBakedRangeChainsFromEnvironment();
    B.groupVertexTarget = RigExecBakedGroupVertexTargetFromEnvironment();
    B.groupCap = RigExecBakedGroupCapFromEnvironment();
    B.groupGates = RigExecBakedGroupGatesFromEnvironment();
    B.useSimd = RigExecSimdEnabled();
    B.purityAudit = TfGetenvBool("RIGEXEC_PURITY_AUDIT", false);
    B.verifyFrozenStatic = TfGetenvBool("RIGEXEC_VERIFY_FROZEN_STATIC", false);
    B.pathLeafGating = TfGetenvBool("RIGEXEC_PATH_LEAF_GATING", true);
    B.verifyPathLeafGating =
        TfGetenvBool("RIGEXEC_VERIFY_PATH_LEAF_GATING", false);
    B.sparseSampling = TfGetenvBool("RIGEXEC_SPARSE_SAMPLING", true);
    B.verifySparseSampling =
        TfGetenvBool("RIGEXEC_VERIFY_SPARSE_SAMPLING", false);
    RigExecRevisionKernelTouchTokens();
    RigExecWeightPacketsTouchTokens();
    RigExecBakedGeometryTouchTokens();
    RigExecBakedWeightsTouchTokens();
    RigExecFrozenGeometryTouchTokens();
    B.assetRootPath = E._rigPath.GetParentPath();
    // The evaluator state a frame reads.
    B.resolvedInputs = &E._resolvedInputs;

    B.profiler = &E._profiler;
}

void
RigExecBakedProgram::_BuildEntry(_BuildAttempt &build)
{
    auto &reasons = build.reasons;
    auto &excludedPoseWriters = build.excludedPoseWriters;
    auto &exclusionProof = build.exclusionProof;
    auto &cycleSkipReasons = build.cycleSkipReasons;
    RigExecRigEvaluator &E = build.E;
    auto &bakeableBeside = build.bakeableBeside;
    auto &buildReasons = build.buildReasons;
    RigExecBakedProgramImpl &B = *build.impl;
    B.interactiveOverrides = &E._interactiveOverrides;
    // The roles a range chain's revisions take, and what the geometry Build
    // classifies them from: the standing overrides above and the admitted
    // upstream values, placed with SetUpstreamInputs' filter, so Build reads
    // what the first prologue will (a value admitted later flips a role and
    // the evaluator rebuilds).
    B.roleMode = E.GetBakedRoleMode();
    B.exportKeep = E.GetBakedExportKeep();
    B.upstream.clear();
    for (const RigExecValueOverride &o : E._upstreamAdmitted) {
        if (!o.attribute.IsEmpty() && o.computation.IsEmpty()) {
            B.upstream[o.prim.AppendProperty(o.attribute)] = o.value;
        }
    }
    B.ownedJointSolverBinding = E._jointSolverBinding;
    for (auto it = B.ownedJointSolverBinding.begin();
         it != B.ownedJointSolverBinding.end();) {
        auto &writers = it->second;
        writers.erase(std::remove_if(writers.begin(), writers.end(),
                                     [&](const auto &writer) {
                                         return excludedPoseWriters.count(
                                                    writer.first) != 0;
                                     }),
                      writers.end());
        if (writers.empty())
            it = B.ownedJointSolverBinding.erase(it);
        else
            ++it;
    }
    B.jointSolverBinding = &B.ownedJointSolverBinding;
    B.cycleExclusionProof = exclusionProof;
    B.cycleSkipReasons = cycleSkipReasons;
    B.solverGuidesEnabled = &E._solverGuidesEnabled;
    B.hasPropertyChains = !E._propertyChains.empty();
    // Every volumetric weight's baked falloff remap, copied rather than
    // recomputed: these are the same bytes the authoritative snapshot hands
    // exec, and resampling the curve again here would risk a different
    // answer for a spline edited between Compile and Build.
    B.currentPhaseWeights = E._currentPhaseWeights;
    for (const RigExecValueOverride &override : E._falloffLutOverrides) {
        if (override.value.IsHolding<RigExecFalloffLut>()) {
            B.falloffLuts[override.prim] =
                override.value.UncheckedGet<RigExecFalloffLut>().samples;
        }
    }

    auto &ctx = build.ctx;
    ctx.program = &B;
    ctx.stage = B.stage;
    ctx.capture = UsdTimeCode::Default();
    ctx.probe = _ProbeTime(B.stage);
    // Until the bakeability task is joined, this build's own refusals go to
    // a vector of their own; see the head of Build.
    ctx.reasons = bakeableBeside && reasons ? &buildReasons : reasons;
    // Every property a chain writes. An input resolving through one of these
    // cannot be captured, because the chain recomputes it every generation.
    for (const auto &[target, revisions] : E._propertyChains) {
        ctx.chainTargets.insert(target);
    }
    B.chainTargets = ctx.chainTargets;
    // Each ribbon's driver-points attribute, resolved once by the compiler.
    ctx.ribbonDriverPoints = E._ribbonDriverPoints;
    // _solverDependencies holds every discovered solver as a key, which is
    // the test "is an aggregate solver" uses below too.
    for (const auto &[solverPath, dependencies] : E._solverDependencies) {
        ctx.aggregateSolvers.insert(ctx.aggregateSolvers.end(), solverPath);
    }
    auto &capture = build.capture;
    capture = ctx.capture;
    const std::set<SdfPath> &chainTargets = ctx.chainTargets;

    // Stage metadata, which arrives as a changed-info notice on the
    // pseudo-root: the up axis an aim constraint resolves against, and the
    // time-code range the input classification probes at.
    B.rebuild.insert(SdfPath::AbsoluteRootPath());

    // What a plain Xformable's transform is measured relative to, which is
    // the one prim rigEvaluator.cpp's pose walk measures against too.
    B.assetRoot = B.stage->GetPrimAtPath(B.assetRootPath);
}

void
RigExecBakedProgram::_BuildProviderSlots(_BuildAttempt &build)
{
    RigExecRigEvaluator &E = build.E;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    // The ordered union of the two provider families: the exec-seeded
    // providers and the plain Xformables a constraint targets. Both are in
    // SdfPath order, so merging them through one ordered map keeps namespace
    // DFS pre-order and with it the "a parent has a lower slot" invariant the
    // compose relies on. The sets are disjoint by construction (only what
    // isFrameProvider accepts is seeded); FirstFramePose wins a collision,
    // because that is the kind the compose can write.
    {
        std::map<SdfPath, RigExecBakedSlotKind> ordered;
        for (const SdfPath &path : E._nativeProviderPaths) {
            ordered.emplace(path, RigExecBakedSlotKind::FirstFramePose);
        }
        for (const SdfPath &path : E._xformDerivedProviders) {
            ordered.emplace(path, RigExecBakedSlotKind::XformDerived);
        }
        for (const auto &[path, kind] : ordered) {
            B.index[path] = int(B.paths.size());
            B.paths.push_back(path);
            B.slotKind.push_back(kind);
            // Read here, with the type already in hand, rather than in the
            // compose: a volume weight's placement is rigid, and the
            // transform-scale avars it would otherwise compose with are
            // exactly the ones exec never binds.
            const UsdPrim prim = B.stage->GetPrimAtPath(path);
            B.noScaleAvars.push_back(
                prim && _IsVolumeWeightTypeName(prim.GetTypeName()) ? 1 : 0);
            B.rotationSign.push_back(0);
        }
        B.pathTexts = RigExecBakedSpellPathTexts(B.paths);
    }
    auto &N = build.N;
    N = int(B.paths.size());
    B.parent.assign(N, -1);
    B.propParent.assign(N, -1);
    for (int i = 0; i < N; ++i) {
        // One climb fills both: the first slot found is the propagation
        // parent whatever its kind, and the climb continues past an
        // xform-derived one because the compose ladder can only inherit from
        // a provider exec composed.
        for (SdfPath p = B.paths[i].GetParentPath();
             !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
             p = p.GetParentPath()) {
            const auto it = B.index.find(p);
            if (it == B.index.end()) {
                continue;
            }
            if (B.propParent[i] < 0) {
                B.propParent[i] = it->second;
            }
            if (B.slotKind[size_t(it->second)] ==
                RigExecBakedSlotKind::FirstFramePose) {
                B.parent[i] = it->second;
                break;
            }
        }
        if (B.parent[i] >= i) {
            ctx.Refuse("provider slots are not in namespace DFS order",
                       B.paths[i]);
        }
    }
}

bool
RigExecBakedProgram::_BuildLadder(_BuildAttempt &build)
{
    auto &reasons = build.reasons;
    auto &scene = build.scene;
    auto &sceneStage = build.sceneStage;
    auto &sceneSerial = build.sceneSerial;
    auto &bakeableLane = build.bakeableLane;
    RigExecRigEvaluator &E = build.E;
    auto &bakeableBeside = build.bakeableBeside;
    auto &bakeable = build.bakeable;
    auto &bakeableReasons = build.bakeableReasons;
    auto &buildReasons = build.buildReasons;
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    auto &capture = build.capture;
    auto &N = build.N;
    // computations.cpp resolves both per provider per evaluation through
    // eight computations. Every channel feeding them is BOUND here, not
    // folded: an ordinary rig binds fifteen constants per provider and the
    // frame path never reads one again, while a rig that animates, connects
    // or chain-writes a rest keeps the same fifteen bindings and recomposes
    // the ladder in the prologue. One resolution either way --
    // RigExecBakedComposeRests and RigExecBakedComposeLadderRange, called
    // once below over every slot, run the kernels the RestCompose op nodes
    // and the ladder head ops run per compose group -- so the two cannot
    // drift.
    B.xyzToken = _tokens->xyz;
    B.posedAuthored.assign(size_t(N), 0);
    B.posedAuthoredM.assign(size_t(N), GfMatrix4d(1.0));
    B.restM.assign(N, GfMatrix4d(1.0));
    B.restPts.resize(N);
    B.restFrames.resize(N);
    B.selfD.assign(N, GfMatrix4d(1.0));
    B.posedD.assign(N, GfMatrix4d(1.0));
    B.parentSpaceM.assign(N, GfMatrix4d(1.0));
    B.parentSpaceAuthored.assign(N, 0);
    B.lastPosedD = B.posedD;
    B.lastParentSpaceM = B.parentSpaceM;
    B.lastParentSpaceAuthored = B.parentSpaceAuthored;
    B.lastRotationSign = B.rotationSign;
    B.parentDinv.assign(N, GfMatrix4d(1.0));
    B.rotOrder.assign(N, B.xyzToken);
    B.restRoundTrip.assign(N, GfMatrix4d(1.0));
    B.defaultRoundTrip.assign(N, GfMatrix4d(1.0));
    B.ladders.resize(size_t(N));
    for (auto &ladder : B.ladders)
        ladder.interveningSpace.constant = GfMatrix4d(1.0);
    // Bindings resolved in parallel (stage read-only); ResolveBind results stay
    // typed. Each chunk records into its own shard (RigExecBakedRecordBind) in
    // slot-then-channel order; shards merge in chunk order after the
    // bakeability join, and a refused rig never merges.
    auto &bindPhases = build.bindPhases;
    bindPhases.Next("Bake.ladder.resolve");
    struct _LadderResolution {
        RigExecBakedInput<GfMatrix4d> posedSpace;
        SdfPathVector posedSpaceWalk;
        RigExecBakedInput<GfMatrix4d> restSpace;
        SdfPathVector restSpaceWalk;
        RigExecBakedInput<GfMatrix4d> defaultSpace;
        SdfPathVector defaultSpaceWalk;
        RigExecBakedInput<TfToken> rotationOrder;
        SdfPathVector rotationOrderWalk;
        RigExecBakedInput<GfMatrix4d> parentSpace;
        SdfPathVector parentSpaceWalk;
        RigExecBakedInput<GfMatrix4d> parentDefaultSpace;
        SdfPathVector parentDefaultSpaceWalk;
        RigExecBakedInput<GfMatrix4d> avarDefaultSpace;
        SdfPathVector avarDefaultSpaceWalk;
        RigExecBakedInput<GfMatrix4d> posedDefaultSpace;
        SdfPathVector posedDefaultSpaceWalk;
        RigExecBakedInput<GfVec3d> rotationSign;
        SdfPathVector rotationSignWalk;
        RigExecBakedInput<double> restAvars[6];
        SdfPathVector restAvarWalks[6];
        RigExecBakedInput<double> defaultAvars[6];
        SdfPathVector defaultAvarWalks[6];
    };
    std::vector<_LadderResolution> ladderResolved(static_cast<size_t>(N));
    // A chunk is a run of consecutive slots. Its size decides only how the
    // work is cut, never what the merge produces, so it is tuned for cost
    // alone: large enough that a shard's sort has something to amortize,
    // small enough to spread a character's few hundred slots over the pool.
    auto &commitChunkSlots = build.commitChunkSlots;
    auto &commitChunks = build.commitChunks;
    commitChunks = (size_t(N) + commitChunkSlots - 1) / commitChunkSlots;
    std::vector<RigExecBakedCommitShard> ladderShards(commitChunks);
    const auto resolveLadderChunks = [&](size_t beginChunk, size_t endChunk) {
        for (size_t k = beginChunk; k < endChunk; ++k) {
            RigExecBakedCommitShard &shard = ladderShards[k];
            const size_t end = std::min(size_t(N), (k + 1) * commitChunkSlots);
            for (size_t i = k * commitChunkSlots; i < end; ++i) {
                // An xform-derived slot binds no ladder -- its rest frame
                // is the identity outright, below -- so there is nothing
                // here for it to resolve.
                if (B.slotKind[i] != RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
                _LadderResolution &out = ladderResolved[i];
                out.posedSpace = ctx.ResolveBind(
                    prim, kPosedSpace, GfMatrix4d(1.0), &out.posedSpaceWalk);
                out.restSpace = ctx.ResolveBind(
                    prim, kRestSpace, GfMatrix4d(1.0), &out.restSpaceWalk);
                out.defaultSpace =
                    ctx.ResolveBind(prim, kDefaultSpace, GfMatrix4d(1.0),
                                    &out.defaultSpaceWalk);
                out.parentSpace = ctx.ResolveBind(
                    prim, kParentSpace, GfMatrix4d(1.0), &out.parentSpaceWalk);
                out.parentDefaultSpace =
                    ctx.ResolveBind(prim, kParentDefaultSpace, GfMatrix4d(1.0),
                                    &out.parentDefaultSpaceWalk);
                out.avarDefaultSpace =
                    ctx.ResolveBind(prim, kAvarDefaultSpace, GfMatrix4d(1.0),
                                    &out.avarDefaultSpaceWalk);
                out.posedDefaultSpace =
                    ctx.ResolveBind(prim, kPosedDefaultSpace, GfMatrix4d(1.0),
                                    &out.posedDefaultSpaceWalk);
                out.rotationSign = ctx.ResolveBind(
                    prim, kRotationSign, GfVec3d(1.0), &out.rotationSignWalk,
                    /*sourceValue=*/true);
                out.rotationOrder = ctx.ResolveBind(
                    prim, kRotationOrder, B.xyzToken, &out.rotationOrderWalk);
                for (int c = 0; c < 6; ++c) {
                    out.restAvars[c] = ctx.ResolveBind(
                        prim, kRestAvarTokens[c], 0.0, &out.restAvarWalks[c],
                        /*sourceValue=*/true);
                    out.defaultAvars[c] =
                        ctx.ResolveBind(prim, kDefaultAvarTokens[c], 0.0,
                                        &out.defaultAvarWalks[c]);
                }
                shard.Prim(B.paths[i]);
                // Recorded in the posedSpace/restSpace/defaultSpace/
                // rotationOrder/avar sequence the serial commit bound them
                // in: the shard numbers in recording order, so this is the
                // order the override indices come out in.
                RigExecBakedRecordBind(&shard, prim, kPosedSpace,
                                       &out.posedSpace, out.posedSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, kRestSpace, &out.restSpace,
                                       out.restSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, kDefaultSpace,
                                       &out.defaultSpace, out.defaultSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, kParentSpace,
                                       &out.parentSpace, out.parentSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, kParentDefaultSpace,
                                       &out.parentDefaultSpace,
                                       out.parentDefaultSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, kAvarDefaultSpace,
                                       &out.avarDefaultSpace,
                                       out.avarDefaultSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, kPosedDefaultSpace,
                                       &out.posedDefaultSpace,
                                       out.posedDefaultSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, kRotationSign,
                                       &out.rotationSign, out.rotationSignWalk);
                RigExecBakedRecordBind(&shard, prim, kRotationOrder,
                                       &out.rotationOrder,
                                       out.rotationOrderWalk);
                for (int c = 0; c < 6; ++c) {
                    RigExecBakedRecordBind(&shard, prim, kRestAvarTokens[c],
                                           &out.restAvars[c],
                                           out.restAvarWalks[c]);
                    RigExecBakedRecordBind(&shard, prim, kDefaultAvarTokens[c],
                                           &out.defaultAvars[c],
                                           out.defaultAvarWalks[c]);
                }
            }
            shard.Seal();
        }
    };
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        commitChunks > 1) {
        WorkParallelForN(commitChunks, resolveLadderChunks);
    } else {
        resolveLadderChunks(0, commitChunks);
    }
    // The bakeability join, here because everything above only read, and
    // the commit below is the first step a refused rig must not take (see
    // the head of Build).
    if (bakeableBeside) {
        {
            RigExecProfileScope join(&E._profiler, "Bake.IsBakeableJoin",
                                     "compile");
            bakeableLane.Wait();
        }
        if (!bakeable) {
            if (reasons) {
                reasons->insert(reasons->end(), bakeableReasons.begin(),
                                bakeableReasons.end());
            }
            return false;
        }
        // The walk passed, so it added nothing; what the slots refused
        // follows it, in the order a serial Build appends them.
        if (reasons) {
            reasons->insert(reasons->end(), buildReasons.begin(),
                            buildReasons.end());
        }
        ctx.reasons = reasons;
    }
    bindPhases.Next("Bake.ladder.commit");
    // The order-bearing half, serial: the merge hands each chunk its first
    // override number, which every input the chunk numbered is moved up by.
    const std::vector<int> ladderFirst =
        RigExecBakedMergeCommitShards(&B, &ladderShards);
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            // An xform-derived slot has no rest chain and no default-space
            // ladder: it gets the identity rest frame and reads its pose off
            // the stage. Invalidation-index coverage: the prim and every
            // ancestor whose transform the prologue composes go into B.prims,
            // so a RESYNC that retypes or reparents one rebuilds the program.
            // Only in B.prims: value notices must not rebuild (the prologue
            // re-reads the ladder every frame).
            B.restFrames[i] = RigExecFrameFromMatrix(GfMatrix4d(1.0));
            B.restPts[i] = B.restFrames[i].points;
            B.xformSlots.push_back(i);
            B.xformPrimsBySlot.push_back(B.stage->GetPrimAtPath(B.paths[i]));
            for (SdfPath p = B.paths[i];
                 !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
                 p = p.GetParentPath()) {
                B.prims.insert(p);
                if (p == B.assetRootPath) {
                    break;
                }
            }
            continue;
        }
        _LadderResolution &out = ladderResolved[size_t(i)];
        const int first = ladderFirst[size_t(i) / commitChunkSlots];
        RigExecBakedProgramImpl::Ladder &ladder = B.ladders[size_t(i)];
        _RenumberOverride(&out.posedSpace, first);
        ladder.posedSpace = std::move(out.posedSpace);
        _RenumberOverride(&out.restSpace, first);
        ladder.restSpace = std::move(out.restSpace);
        _RenumberOverride(&out.defaultSpace, first);
        ladder.defaultSpace = std::move(out.defaultSpace);
        _RenumberOverride(&out.parentSpace, first);
        ladder.parentSpace = std::move(out.parentSpace);
        _RenumberOverride(&out.parentDefaultSpace, first);
        ladder.parentDefaultSpace = std::move(out.parentDefaultSpace);
        _RenumberOverride(&out.avarDefaultSpace, first);
        ladder.avarDefaultSpace = std::move(out.avarDefaultSpace);
        _RenumberOverride(&out.posedDefaultSpace, first);
        ladder.posedDefaultSpace = std::move(out.posedDefaultSpace);
        _RenumberOverride(&out.rotationSign, first);
        ladder.rotationSign = std::move(out.rotationSign);
        const UsdPrim provider = B.stage->GetPrimAtPath(B.paths[size_t(i)]);
        const auto connected = [&provider](const TfToken &name) {
            const UsdAttribute attribute = provider.GetAttribute(name);
            SdfPathVector sources;
            return attribute && attribute.GetConnections(&sources) &&
                   sources.size() == 1 &&
                   attribute.GetStage()->GetAttributeAtPath(sources[0]) &&
                   attribute.GetStage()
                           ->GetAttributeAtPath(sources[0])
                           .GetTypeName() == attribute.GetTypeName();
        };
        ladder.posedSpaceConnected = connected(kPosedSpace);
        ladder.defaultSpaceConnected = connected(kDefaultSpace);
        ladder.parentSpaceConnected = connected(kParentSpace);
        ladder.parentDefaultSpaceConnected = connected(kParentDefaultSpace);
        ladder.avarDefaultSpaceConnected = connected(kAvarDefaultSpace);
        ladder.posedDefaultSpaceConnected = connected(kPosedDefaultSpace);
        _RenumberOverride(&out.rotationOrder, first);
        ladder.rotationOrder = std::move(out.rotationOrder);
        for (int c = 0; c < 6; ++c) {
            _RenumberOverride(&out.restAvars[c], first);
            ladder.restAvars[c] = std::move(out.restAvars[c]);
            _RenumberOverride(&out.defaultAvars[c], first);
            ladder.defaultAvars[c] = std::move(out.defaultAvars[c]);
        }
    }
    // What a frame may have to re-resolve, and where a drag on one lands.
    // Both are asked of the WHOLE ladder rather than per provider: exec
    // re-pulls every rest the moment any one of them can move (rigEvaluator
    // .cpp's newRestsMightVary), and a chain's answer depends on its
    // ancestors in any case.
    bindPhases.Next("Bake.ladder.varies_and_compose");
    const auto noteLadderInput = [&B](const auto &input) {
        if (input.varying) {
            B.ladderVarying = true;
        }
        if (input.overrideIndex >= 0) {
            B.ladderOverrides.push_back(input.overrideIndex);
        }
    };
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            continue;
        }
        const RigExecBakedProgramImpl::Ladder &ladder = B.ladders[size_t(i)];
        noteLadderInput(ladder.posedSpace);
        noteLadderInput(ladder.restSpace);
        noteLadderInput(ladder.defaultSpace);
        noteLadderInput(ladder.parentSpace);
        noteLadderInput(ladder.parentDefaultSpace);
        noteLadderInput(ladder.avarDefaultSpace);
        noteLadderInput(ladder.posedDefaultSpace);
        noteLadderInput(ladder.rotationSign);
        noteLadderInput(ladder.rotationOrder);
        for (int c = 0; c < 6; ++c) {
            noteLadderInput(ladder.restAvars[c]);
            noteLadderInput(ladder.defaultAvars[c]);
        }
    }
    // Per slot, whether the rest chain reaching it can move. A rest is its
    // ancestors' rests as well, so this propagates down the chain -- slots
    // are in namespace DFS pre-order and a parent's slot is always lower,
    // so one forward pass settles it.
    B.restChainVaries.assign(size_t(N), 0);
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            continue;
        }
        const RigExecBakedProgramImpl::Ladder &ladder = B.ladders[size_t(i)];
        bool varies = ladder.restSpace.varying;
        for (int c = 0; c < 6; ++c) {
            varies = varies || ladder.restAvars[c].varying;
        }
        if (B.parent[size_t(i)] >= 0 &&
            B.restChainVaries[size_t(B.parent[size_t(i)])]) {
            varies = true;
        }
        B.restChainVaries[size_t(i)] = varies ? 1 : 0;
    }
    std::sort(B.ladderOverrides.begin(), B.ladderOverrides.end());
    B.ladderOverrides.erase(
        std::unique(B.ladderOverrides.begin(), B.ladderOverrides.end()),
        B.ladderOverrides.end());
    // And the ladder itself, at the capture time, through the same function
    // the prologue calls. Nothing is captured that the frame path cannot
    // re-resolve, which is what makes the two agree by construction. It
    // reads leaves, as the prologue does, so the ones bound so far are
    // numbered and sampled first; the end of Build numbers them all again.
    for (const SdfPath &provider : E._interveningXformProviders) {
        const int slot = ctx.SlotOf(provider);
        if (slot < 0)
            continue;
        B.interveningSlots.push_back(slot);
        const auto anchor = E._poseProviderAnchors.find(provider);
        B.interveningAnchors.push_back(anchor == E._poseProviderAnchors.end() ||
                                               anchor->second.IsEmpty()
                                           ? B.assetRootPath
                                           : anchor->second);
        B.ladders[size_t(slot)].interveningSpace.constant = GfMatrix4d(1.0);
        B.ladders[size_t(slot)].interveningSpace.varying = true;
        B.ladderVarying = true;
    }
    _SampleIntervening(&B, capture);
    {
        std::string error;
        if (!RigExecBakedBuildSpaces(&B, capture, &error, scene, sceneStage,
                                     sceneSerial)) {
            ctx.Refuse("provider expressions could not translate: " + error,
                       E._rigPath);
            return false;
        }
    }
    RigExecBakedNumberLeaves(&B);
    RigExecBakedSampleLeaves(&B, capture, /* all = */ true);
    // Every rest before any ladder: a ladder reads its own slot's rest and
    // its parent's, and a slot's parent is below it.
    RigExecBakedComposeRests(&B);
    RigExecBakedComposeLadderRange(&B, 0, N, /* trackMoves = */ false);
    bindPhases.Close();
    // The comparison buffers the rest and ladder ops compare against,
    // seeded with what Build just composed: a later compose that lands on
    // the same bits finds nothing moved.
    B.lastRestM = B.restM;
    B.lastSelfD = B.selfD;
    B.lastPosedD = B.posedD;
    B.lastParentSpaceM = B.parentSpaceM;
    B.lastParentSpaceAuthored = B.parentSpaceAuthored;
    B.lastRotationSign = B.rotationSign;
    B.lastParentDinv = B.parentDinv;
    B.lastPosedAuthored = B.posedAuthored;
    B.lastPosedAuthoredM = B.posedAuthoredM;
    B.lastRotOrder = B.rotOrder;

    // The seed's two comparison buffers, sized once and never resized in a
    // run. Their first-run values are never read: the first run of a program
    // dirties every pose cluster outright, because it has nothing to compare
    // against.
    B.xformBase.assign(B.xformSlots.size(), GfMatrix4d(1.0));
    B.lastXformBase.assign(B.xformSlots.size(), GfMatrix4d(1.0));
    return true;
}

void
RigExecBakedProgram::_BuildInputTable(_BuildAttempt &build)
{
    RigExecBakedProgramImpl &B = *build.impl;
    auto &ctx = build.ctx;
    auto &N = build.N;
    auto &bindPhases = build.bindPhases;
    auto &commitChunkSlots = build.commitChunkSlots;
    auto &commitChunks = build.commitChunks;
    B.avarConstants.assign(size_t(N) * 11, 0.0);
    // Eleven channels on every provider, each an independent read of the
    // composed stage, and on a character this is one of the two loops Build
    // spends most of its time in. Resolved across threads, and the commit's
    // order-free half recorded there too, into one shard per chunk of slots
    // exactly as the ladder above does; what stays on this thread is what
    // only the program order can decide -- the override numbers, by the
    // merge's prefix sum, and the positions in the two binding lists.
    struct _AvarResolution {
        RigExecBakedInput<double> input;
        SdfPathVector walk;
        /// The avar's own property, and whether a registered constant was
        /// captured from it and nowhere else -- decided in the parallel
        /// pass, which already holds the prim, so the serial one only
        /// files it.
        SdfPath property;
        bool patchable = false;
    };
    bindPhases.Next("Bake.input_table.resolve");
    std::vector<_AvarResolution> resolved(size_t(N) * 11);
    std::vector<RigExecBakedCommitShard> avarShards(commitChunks);
    const auto resolveChunks = [&](size_t beginChunk, size_t endChunk) {
        for (size_t k = beginChunk; k < endChunk; ++k) {
            RigExecBakedCommitShard &shard = avarShards[k];
            const size_t end = std::min(size_t(N), (k + 1) * commitChunkSlots);
            for (size_t i = k * commitChunkSlots; i < end; ++i) {
                // An xform-derived slot binds no avars -- its pose is the
                // stage's -- so the bound/varying counts and the
                // overridable set stay exactly what the RigExec providers
                // alone make them.
                if (B.slotKind[i] != RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
                for (int c = 0; c < 11; ++c) {
                    _AvarResolution &out = resolved[i * 11 + size_t(c)];
                    out.input =
                        ctx.ResolveBind(prim, kAvarTokens[c],
                                        RigExecBakedAvarDefaults[c], &out.walk);
                    RigExecBakedRecordBind(&shard, prim, kAvarTokens[c],
                                           &out.input, out.walk);
                    if (out.input.varying || out.input.overrideIndex < 0) {
                        continue;
                    }
                    // Patchable only when the value came from the attribute
                    // itself. A walk longer than one captured it upstream,
                    // through a connection, and an edit on the head would
                    // not be the value the slot holds.
                    out.property =
                        prim.GetPath().AppendProperty(kAvarTokens[c]);
                    out.patchable = out.walk.size() <= 1 && out.input.head &&
                                    out.input.head.GetPath() == out.property;
                }
            }
            shard.Seal();
        }
    };
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        commitChunks > 1) {
        WorkParallelForN(commitChunks, resolveChunks);
    } else {
        resolveChunks(0, commitChunks);
    }
    bindPhases.Next("Bake.input_table.commit");
    const std::vector<int> avarFirst =
        RigExecBakedMergeCommitShards(&B, &avarShards);
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            continue;
        }
        const int first = avarFirst[size_t(i) / commitChunkSlots];
        for (int c = 0; c < 11; ++c) {
            const size_t slot = size_t(i) * 11 + size_t(c);
            _AvarResolution &out = resolved[slot];
            _RenumberOverride(&out.input, first);
            B.avarConstants[slot] = out.input.constant;
            if (out.input.varying) {
                B.avarBindings.push_back({slot, std::move(out.input)});
            } else if (out.input.overrideIndex >= 0) {
                if (out.patchable) {
                    B.patchableAvars[out.property] =
                        B.avarConstantBindings.size();
                }
                B.avarConstantBindings.push_back({slot, std::move(out.input)});
            }
        }
    }
    RigExecBakedIndexAvarBindings(&B);
    B.avars = B.avarConstants;
    bindPhases.Close();
}

} // namespace rigExec
