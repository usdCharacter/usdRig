// benchMatrixMover -- matrix-mover execution bench across the example rigs.
//
// Walks the shared fixture table (tests/exampleFixtures.cmake, the same rows
// the parity suite runs) plus docs/examples/matrix_mover.usda and reports,
// per rig: compile time, baked Evaluate frame time, baked matrix-mover step
// time attributed through the program's own op stamps, and dynamic
// (Exec-network cross-check) Evaluate frame time. Ends with a kernel
// microbench of RigExecApplyWeightedMatrixSimd at three point counts and a
// geomean of baked frame times as the guard metric.
//
//   benchMatrixMover <examplesDir> [reps] [filter]
//
// reps scales the baked Evaluate repetitions per frame (default 10); filter
// keeps only stages whose path contains it. Pin to stable cores, e.g.
// taskset -c 15-19, and interleave A/B runs when comparing libraries.
//
// Timing is entirely outside the hot path: wall clock around Evaluate calls
// plus the op stamps the program already publishes at the run boundary under
// SetOpTimingEnabled (no locks, no body instrumentation). The matrix lines
// are attribution, not wall time: under a parallel schedule overlapping
// steps sum past the frame. Run with RIGEXEC_BAKED_SCHEDULE=serial for
// additive attribution.
//
// A second table splits matrix steps into memo/body/publish phases (needs
// RIGEXEC_BAKED_STEP_TIMING=1 and RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1 at
// Compile), chunk time by revision role, sparse-walk computed/reused/
// skippedWhole counters, and fold/fuse steps (supporting: outside the
// headline matrix sum, which stays static+chunk+walk). Compute-only extras
// (animated stages whose matrix inputs move every visited frame) print in
// their own section with the same columns plus walk counters; they never
// enter the headline sums.
//
// Built but deliberately NOT registered with ctest (like benchFrameCache):
// it prints numbers and asserts nothing.
#include "rigExec/bakedExecCrossCheck.h"
#include "rigExec/bakedExecCrossCheckRows.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include "rigExecExampleFixtures.h"

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

using Clock = std::chrono::steady_clock;

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

std::vector<UsdTimeCode>
ParseFrames(const std::string &text)
{
    std::vector<UsdTimeCode> frames;
    for (const std::string &piece : TfStringSplit(text, ",")) {
        if (!piece.empty()) {
            frames.push_back(UsdTimeCode(TfStringToDouble(piece)));
        }
    }
    return frames;
}

double
Median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double median = values[mid];
    if (values.size() % 2 == 0) {
        std::nth_element(values.begin(), values.begin() + mid - 1,
                         values.end());
        median = 0.5 * (median + values[mid - 1]);
    }
    return median;
}

// One rig's baked matrix attribution over its frames, in microseconds per
// frame: RevisionStatic (packet/status/decision) and RevisionChunk (point
// loop) steps whose revision op is Matrix, plus every chunk step and every
// stamped step for context.
struct MatrixAttribution {
    double matrixStaticUs = 0.0;
    double matrixChunkUs = 0.0;
    // Transform-domain matrix movers ride the constraint walk, not the
    // geometry revisions: Constraint/Commit steps of a RigExecMatrixMover.
    double matrixWalkUs = 0.0;
    double allChunkUs = 0.0;
    double allStampedUs = 0.0;
    size_t matrixRevisions = 0;
    size_t matrixWalkOps = 0;
    // Memo/body/publish split of the matrix static+chunk steps above, from
    // the step stamps' inner intervals. Nonzero only with
    // RIGEXEC_BAKED_STEP_TIMING=1 and RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1 in
    // the environment at Compile (forced runs are otherwise excluded from
    // measurement). Scheduling gaps between the phases are in neither.
    double matrixMemoUs = 0.0;
    double matrixBodyUs = 0.0;
    double matrixPublishUs = 0.0;
    double matrixStaticMemoUs = 0.0;
    double matrixStaticBodyUs = 0.0;
    double matrixStaticPublishUs = 0.0;
    double matrixChunkMemoUs = 0.0;
    double matrixChunkBodyUs = 0.0;
    double matrixChunkPublishUs = 0.0;
    // Matrix chunk time by revision role: Range groups, Legacy wholes,
    // Whole fuses.
    double matrixChunkRangeUs = 0.0;
    double matrixChunkLegacyUs = 0.0;
    double matrixChunkWholeUs = 0.0;
    size_t matrixRangeGroups = 0;
    size_t matrixChunkSteps = 0;
    size_t matrixChunkRangeSteps = 0;
    size_t matrixChunkLegacySteps = 0;
    // Last-run sparse-walk counts over matrix revisions: Range groups'
    // walks plus Legacy/Whole revisions' own. Computed>0 means the
    // attribution pass ran kernels; skippedWhole counts whole-walk skips.
    uint64_t matrixWalkComputed = 0;
    uint64_t matrixWalkReused = 0;
    uint64_t matrixWalkSkipped = 0;
    // A matrix revision's fold (influence table) and fuse (join/applied
    // decision) steps. Supporting only: outside the headline matrix sum,
    // which stays static+chunk+walk for round-over-round comparability.
    double matrixFoldUs = 0.0;
    double matrixFuseUs = 0.0;
    double matrixFoldMemoUs = 0.0;
    double matrixFoldBodyUs = 0.0;
    double matrixFoldPublishUs = 0.0;
    double matrixFuseMemoUs = 0.0;
    double matrixFuseBodyUs = 0.0;
    double matrixFusePublishUs = 0.0;
};

MatrixAttribution
AttributeMatrix(const RigExecBakedProgramImpl &B)
{
    MatrixAttribution attribution;
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.op == RigExecRevisionOp::Matrix) {
                ++attribution.matrixRevisions;
                if (revision.role == RigExecBakedRevisionRole::Range) {
                    attribution.matrixRangeGroups +=
                        revision.groups.size();
                    for (const auto &group : revision.groups) {
                        attribution.matrixWalkComputed +=
                            group.walk.computed;
                        attribution.matrixWalkReused += group.walk.reused;
                        attribution.matrixWalkSkipped +=
                            group.walk.skippedWhole;
                    }
                } else {
                    attribution.matrixWalkComputed +=
                        revision.matrixWalk.computed;
                    attribution.matrixWalkReused +=
                        revision.matrixWalk.reused;
                    attribution.matrixWalkSkipped +=
                        revision.matrixWalk.skippedWhole;
                }
            }
        }
    }
    std::set<SdfPath> matrixWalkPaths;
    for (const auto &constraint : B.constraints) {
        if (constraint.type == "RigExecMatrixMover") {
            matrixWalkPaths.insert(constraint.path);
            ++attribution.matrixWalkOps;
        }
    }
    for (const uint32_t index : B.stampedSteps) {
        if (index >= B.steps.size()) {
            continue;
        }
        const RigExecBakedStep &step = B.steps[index];
        if (step.publishEndNs <= step.memoStartNs) {
            continue;
        }
        const double us =
            double(step.publishEndNs - step.memoStartNs) / 1000.0;
        attribution.allStampedUs += us;
        const bool isStatic =
            step.kind == RigExecBakedStepKind::RevisionStatic;
        const bool isChunk =
            step.kind == RigExecBakedStepKind::RevisionChunk;
        const bool isFold =
            step.kind == RigExecBakedStepKind::InfluenceFold;
        const bool isFuse =
            step.kind == RigExecBakedStepKind::RevisionFuse;
        const bool isWalk =
            step.kind == RigExecBakedStepKind::Constraint ||
            step.kind == RigExecBakedStepKind::CommitDelta ||
            step.kind == RigExecBakedStepKind::PropagateChunk ||
            step.kind == RigExecBakedStepKind::CommitApply;
        if (isWalk && step.object >= 0 &&
            size_t(step.object) < B.commits.size() &&
            matrixWalkPaths.count(
                B.commits[size_t(step.object)].moverPath)) {
            attribution.matrixWalkUs += us;
            continue;
        }
        if (!isStatic && !isChunk && !isFold && !isFuse) {
            continue;
        }
        if (isChunk) {
            attribution.allChunkUs += us;
        }
        if (step.object < 0 ||
            size_t(step.object) >= B.revisionIndex.size()) {
            continue;
        }
        const auto [chain, revision] =
            B.revisionIndex[size_t(step.object)];
        if (chain < 0 || size_t(chain) >= B.chains.size() ||
            revision < 0 ||
            size_t(revision) >= B.chains[size_t(chain)].revisions.size()) {
            continue;
        }
        const auto &geomRevision =
            B.chains[size_t(chain)].revisions[size_t(revision)];
        if (geomRevision.op == RigExecRevisionOp::Matrix) {
            if (step.memoEndNs > step.memoStartNs &&
                step.bodyEndNs > step.memoEndNs &&
                step.publishEndNs > step.bodyEndNs) {
                const double memoUs =
                    double(step.memoEndNs - step.memoStartNs) / 1000.0;
                const double bodyUs =
                    double(step.bodyEndNs - step.memoEndNs) / 1000.0;
                const double pubUs =
                    double(step.publishEndNs - step.bodyEndNs) / 1000.0;
                if (isFold) {
                    attribution.matrixFoldMemoUs += memoUs;
                    attribution.matrixFoldBodyUs += bodyUs;
                    attribution.matrixFoldPublishUs += pubUs;
                } else if (isFuse) {
                    attribution.matrixFuseMemoUs += memoUs;
                    attribution.matrixFuseBodyUs += bodyUs;
                    attribution.matrixFusePublishUs += pubUs;
                }
            }
            if (isFold) {
                attribution.matrixFoldUs += us;
                continue;
            }
            if (isFuse) {
                attribution.matrixFuseUs += us;
                continue;
            }
            if (isChunk) {
                attribution.matrixChunkUs += us;
                ++attribution.matrixChunkSteps;
                switch (geomRevision.role) {
                case RigExecBakedRevisionRole::Range:
                    attribution.matrixChunkRangeUs += us;
                    ++attribution.matrixChunkRangeSteps;
                    break;
                case RigExecBakedRevisionRole::Legacy:
                    attribution.matrixChunkLegacyUs += us;
                    ++attribution.matrixChunkLegacySteps;
                    break;
                case RigExecBakedRevisionRole::Whole:
                    attribution.matrixChunkWholeUs += us;
                    break;
                }
            } else {
                attribution.matrixStaticUs += us;
            }
            if (step.memoEndNs > step.memoStartNs &&
                step.bodyEndNs > step.memoEndNs &&
                step.publishEndNs > step.bodyEndNs) {
                const double memoUs =
                    double(step.memoEndNs - step.memoStartNs) / 1000.0;
                const double bodyUs =
                    double(step.bodyEndNs - step.memoEndNs) / 1000.0;
                const double pubUs =
                    double(step.publishEndNs - step.bodyEndNs) / 1000.0;
                attribution.matrixMemoUs += memoUs;
                attribution.matrixBodyUs += bodyUs;
                attribution.matrixPublishUs += pubUs;
                if (isChunk) {
                    attribution.matrixChunkMemoUs += memoUs;
                    attribution.matrixChunkBodyUs += bodyUs;
                    attribution.matrixChunkPublishUs += pubUs;
                } else {
                    attribution.matrixStaticMemoUs += memoUs;
                    attribution.matrixStaticBodyUs += bodyUs;
                    attribution.matrixStaticPublishUs += pubUs;
                }
            }
        }
    }
    return attribution;
}

struct RigResult {
    std::string stage;
    bool compiled = false;
    bool baked = false;
    double compileMs = 0.0;
    double bakedFrameUs = 0.0;
    double matrixStaticUs = 0.0;
    double matrixChunkUs = 0.0;
    double matrixWalkUs = 0.0;
    double allChunkUs = 0.0;
    double allStampedUs = 0.0;
    size_t matrixRevisions = 0;
    size_t matrixWalkOps = 0;
    double matrixMemoUs = 0.0;
    double matrixBodyUs = 0.0;
    double matrixPublishUs = 0.0;
    double matrixStaticMemoUs = 0.0;
    double matrixStaticBodyUs = 0.0;
    double matrixStaticPublishUs = 0.0;
    double matrixChunkMemoUs = 0.0;
    double matrixChunkBodyUs = 0.0;
    double matrixChunkPublishUs = 0.0;
    double matrixChunkRangeUs = 0.0;
    double matrixChunkLegacyUs = 0.0;
    double matrixChunkWholeUs = 0.0;
    size_t matrixRangeGroups = 0;
    size_t matrixChunkSteps = 0;
    size_t matrixChunkRangeSteps = 0;
    size_t matrixChunkLegacySteps = 0;
    uint64_t matrixWalkComputed = 0;
    uint64_t matrixWalkReused = 0;
    uint64_t matrixWalkSkipped = 0;
    double matrixFoldUs = 0.0;
    double matrixFuseUs = 0.0;
    double matrixFoldMemoUs = 0.0;
    double matrixFoldBodyUs = 0.0;
    double matrixFoldPublishUs = 0.0;
    double matrixFuseMemoUs = 0.0;
    double matrixFuseBodyUs = 0.0;
    double matrixFusePublishUs = 0.0;
    bool computeOnly = false;
    size_t clusters = 0;
    size_t minClustersRun = 0;
    double dynamicFrameUs = 0.0;
    std::string dynamicNote = "n/a";
};

RigResult
BenchRig(const std::string &examplesDir, const char *stage,
         const char *framesText, int reps)
{
    RigResult result;
    result.stage = stage;
    const UsdStageRefPtr usdStage =
        UsdStage::Open(examplesDir + "/" + stage);
    if (!usdStage) {
        return result;
    }
    const SdfPath rigPath = FindRig(usdStage);
    if (rigPath.IsEmpty()) {
        return result;
    }
    const std::vector<UsdTimeCode> frames = ParseFrames(framesText);
    if (frames.empty()) {
        return result;
    }
    RigExecRigEvaluator rig(usdStage, rigPath);
    // Headless bench: guide frames are viewport data no deformed point
    // reads; skipping them keeps the frame about the movers.
    rig.SetSolverGuidesEnabled(false);
    std::vector<std::string> errors;
    const auto compileBegan = Clock::now();
    const bool compiled = rig.Compile(&errors);
    result.compileMs =
        std::chrono::duration<double, std::milli>(Clock::now() -
                                                 compileBegan)
            .count();
    result.compiled = compiled;
    if (!compiled) {
        return result;
    }
    std::vector<std::string> reasons;
    if (!rig.IsBakeable(&reasons)) {
        return result;
    }
    result.baked = true;
    // Warm the program, then time baked frames with op timing off so the
    // measured frame is the frame a caller gets. Every timed frame is a
    // forced full run: without it a repeated frame's cone skips the matrix
    // steps and the median measures the skip, not the mover.
    for (const UsdTimeCode frame : frames) {
        rig.Evaluate(frame);
    }
    const RigExecBakedProgram *bakedProgram = rig.GetBakedProgram();
    if (!bakedProgram) {
        result.baked = false;
        return result;
    }
    result.clusters = rig.GetBakedClusterCount();
    result.minClustersRun = result.clusters;
    std::vector<double> frameUs;
    frameUs.reserve(size_t(reps) * frames.size());
    for (int rep = 0; rep < reps; ++rep) {
        for (const UsdTimeCode frame : frames) {
            bakedProgram->RequestFullRun();
            const auto began = Clock::now();
            const RigExecRigPose pose = rig.Evaluate(frame);
            frameUs.push_back(
                std::chrono::duration<double, std::micro>(Clock::now() -
                                                          began)
                    .count());
            result.minClustersRun =
                std::min(result.minClustersRun,
                         rig.GetBakedClustersRunLastGeneration());
            if (!pose.valid) {
                result.baked = false;
                return result;
            }
        }
    }
    result.bakedFrameUs = Median(frameUs);
    // Attribution pass with op timing on: one full run per frame, stamps
    // read after the join at the existing boundary.
    rig.SetOpTimingEnabled(true);
    MatrixAttribution total;
    for (const UsdTimeCode frame : frames) {
        bakedProgram->RequestFullRun();
        rig.Evaluate(frame);
        const RigExecBakedProgram *program = rig.GetBakedProgram();
        if (!program) {
            continue;
        }
        const MatrixAttribution attribution =
            AttributeMatrix(program->GetStepGraph());
        total.matrixStaticUs += attribution.matrixStaticUs;
        total.matrixChunkUs += attribution.matrixChunkUs;
        total.matrixWalkUs += attribution.matrixWalkUs;
        total.allChunkUs += attribution.allChunkUs;
        total.allStampedUs += attribution.allStampedUs;
        total.matrixMemoUs += attribution.matrixMemoUs;
        total.matrixBodyUs += attribution.matrixBodyUs;
        total.matrixPublishUs += attribution.matrixPublishUs;
        total.matrixStaticMemoUs += attribution.matrixStaticMemoUs;
        total.matrixStaticBodyUs += attribution.matrixStaticBodyUs;
        total.matrixStaticPublishUs += attribution.matrixStaticPublishUs;
        total.matrixChunkMemoUs += attribution.matrixChunkMemoUs;
        total.matrixChunkBodyUs += attribution.matrixChunkBodyUs;
        total.matrixChunkPublishUs += attribution.matrixChunkPublishUs;
        total.matrixChunkRangeUs += attribution.matrixChunkRangeUs;
        total.matrixChunkLegacyUs += attribution.matrixChunkLegacyUs;
        total.matrixChunkWholeUs += attribution.matrixChunkWholeUs;
        total.matrixRevisions = attribution.matrixRevisions;
        total.matrixWalkOps = attribution.matrixWalkOps;
        total.matrixRangeGroups = attribution.matrixRangeGroups;
        total.matrixChunkSteps =
            std::max(total.matrixChunkSteps, attribution.matrixChunkSteps);
        total.matrixChunkRangeSteps = std::max(total.matrixChunkRangeSteps,
                                              attribution.matrixChunkRangeSteps);
        total.matrixChunkLegacySteps = std::max(
            total.matrixChunkLegacySteps, attribution.matrixChunkLegacySteps);
        total.matrixWalkComputed += attribution.matrixWalkComputed;
        total.matrixWalkReused += attribution.matrixWalkReused;
        total.matrixWalkSkipped += attribution.matrixWalkSkipped;
        total.matrixFoldUs += attribution.matrixFoldUs;
        total.matrixFuseUs += attribution.matrixFuseUs;
        total.matrixFoldMemoUs += attribution.matrixFoldMemoUs;
        total.matrixFoldBodyUs += attribution.matrixFoldBodyUs;
        total.matrixFoldPublishUs += attribution.matrixFoldPublishUs;
        total.matrixFuseMemoUs += attribution.matrixFuseMemoUs;
        total.matrixFuseBodyUs += attribution.matrixFuseBodyUs;
        total.matrixFusePublishUs += attribution.matrixFusePublishUs;
    }
    rig.SetOpTimingEnabled(false);
    const double framesCount = double(frames.size());
    result.matrixStaticUs = total.matrixStaticUs / framesCount;
    result.matrixChunkUs = total.matrixChunkUs / framesCount;
    result.matrixWalkUs = total.matrixWalkUs / framesCount;
    result.allChunkUs = total.allChunkUs / framesCount;
    result.allStampedUs = total.allStampedUs / framesCount;
    result.matrixMemoUs = total.matrixMemoUs / framesCount;
    result.matrixBodyUs = total.matrixBodyUs / framesCount;
    result.matrixPublishUs = total.matrixPublishUs / framesCount;
    result.matrixStaticMemoUs = total.matrixStaticMemoUs / framesCount;
    result.matrixStaticBodyUs = total.matrixStaticBodyUs / framesCount;
    result.matrixStaticPublishUs = total.matrixStaticPublishUs / framesCount;
    result.matrixChunkMemoUs = total.matrixChunkMemoUs / framesCount;
    result.matrixChunkBodyUs = total.matrixChunkBodyUs / framesCount;
    result.matrixChunkPublishUs = total.matrixChunkPublishUs / framesCount;
    result.matrixChunkRangeUs = total.matrixChunkRangeUs / framesCount;
    result.matrixChunkLegacyUs = total.matrixChunkLegacyUs / framesCount;
    result.matrixChunkWholeUs = total.matrixChunkWholeUs / framesCount;
    result.matrixRevisions = total.matrixRevisions;
    result.matrixWalkOps = total.matrixWalkOps;
    result.matrixRangeGroups = total.matrixRangeGroups;
    result.matrixChunkSteps = total.matrixChunkSteps;
    result.matrixChunkRangeSteps = total.matrixChunkRangeSteps;
    result.matrixChunkLegacySteps = total.matrixChunkLegacySteps;
    result.matrixWalkComputed = total.matrixWalkComputed;
    result.matrixWalkReused = total.matrixWalkReused;
    result.matrixWalkSkipped = total.matrixWalkSkipped;
    result.matrixFoldUs = total.matrixFoldUs / framesCount;
    result.matrixFuseUs = total.matrixFuseUs / framesCount;
    result.matrixFoldMemoUs = total.matrixFoldMemoUs / framesCount;
    result.matrixFoldBodyUs = total.matrixFoldBodyUs / framesCount;
    result.matrixFoldPublishUs = total.matrixFoldPublishUs / framesCount;
    result.matrixFuseMemoUs = total.matrixFuseMemoUs / framesCount;
    result.matrixFuseBodyUs = total.matrixFuseBodyUs / framesCount;
    result.matrixFusePublishUs = total.matrixFusePublishUs / framesCount;
    // Dynamic path: the Exec-network cross-check rows, which evaluate the
    // VDF mover callbacks (including the matrix point-array expression)
    // rather than the baked program. A fresh evaluator mimics the
    // cross-check test's own setup exactly (guides on from the start,
    // Evaluate before rows), so row bindings never span a rebuild.
    {
        RigExecRigEvaluator dynamicRig(usdStage, rigPath);
        dynamicRig.SetSolverGuidesEnabled(true);
        std::vector<std::string> dynamicErrors;
        if (!dynamicRig.Compile(&dynamicErrors)) {
            result.dynamicNote = "compile-fail";
            return result;
        }
        const RigExecBakedProgram *program =
            dynamicRig.GetBakedProgram();
        std::string error;
        if (!program) {
            result.dynamicNote = "no-program";
        } else if (!RigExecBakedProgramTesting::EnableExecCrossCheck(
                       *program, &error)) {
            result.dynamicNote = "enable-fail";
        } else if (const auto rows =
                       RigExecBakedProgramTesting::ExecCrossCheckRows(
                           *program)) {
            if (rows->Descriptors().empty()) {
                result.dynamicNote = "no-rows";
            } else {
                const int dynamicReps = std::min(reps, 3);
                std::vector<double> dynamicUs;
                size_t checked = 0, failed = 0, dropped = 0;
                bool passed = true;
                for (int rep = 0; rep < dynamicReps; ++rep) {
                    for (const UsdTimeCode frame : frames) {
                        dynamicRig.Evaluate(frame);
                        const auto began = Clock::now();
                        const RigExecExecCrossCheckReport report =
                            rows->Evaluate(frame);
                        dynamicUs.push_back(
                            std::chrono::duration<double, std::micro>(
                                Clock::now() - began)
                                .count());
                        checked = report.checked;
                        failed = report.failed;
                        dropped = report.dropped;
                        passed = passed && report.Passed();
                    }
                }
                // Timing counts when rows ran; the comparison outcome is
                // the parity suite's verdict, annotated here, not a veto.
                if (checked > 0 && !dynamicUs.empty()) {
                    result.dynamicFrameUs = Median(dynamicUs);
                    result.dynamicNote = TfStringPrintf(
                        "%.1f%s", result.dynamicFrameUs,
                        passed ? ""
                               : TfStringPrintf("(chk%zu f%zu d%zu)",
                                                checked, failed,
                                                dropped)
                                     .c_str());
                } else {
                    result.dynamicNote = TfStringPrintf(
                        "unchecked(f%zu d%zu)", failed, dropped);
                }
            }
        } else {
            result.dynamicNote = "no-rows-object";
        }
    }
    return result;
}

struct KernelResult {
    size_t count = 0;
    double arrayNsPerPoint = 0.0;
    double scalarNsPerPoint = 0.0;
};

KernelResult
BenchKernel(size_t count, int outerReps, int innerReps)
{
    KernelResult result;
    result.count = count;
    // Deterministic spread of points and weights: full/empty endpoints
    // plus a ramp, so every weight arm runs.
    std::vector<GfVec3f> in(count), out(count);
    std::vector<float> weights(count);
    for (size_t i = 0; i < count; ++i) {
        in[i] = GfVec3f(float(i % 63) - 31.0f, float(i % 17) * 1.5f,
                        float(i % 41) - 20.0f);
        weights[i] =
            (i % 7 == 0) ? 0.0f : ((i % 7 == 1) ? 1.0f : float(i % 100) / 100.0f);
    }
    const GfMatrix4d transform =
        GfMatrix4d(1.0).SetScale(GfVec3d(1.5, 0.8, 2.0)) *
        GfMatrix4d(GfRotation(GfVec3d(1, 1, 0), 33.0),
                   GfVec3d(2, -1, 4));
    std::vector<double> arrayNs, scalarNs;
    for (int rep = 0; rep < outerReps; ++rep) {
        auto began = Clock::now();
        for (int k = 0; k < innerReps; ++k) {
            RigExecApplyWeightedMatrixSimd(in.data(), out.data(),
                                           weights.data(), count,
                                           transform);
        }
        arrayNs.push_back(
            std::chrono::duration<double, std::nano>(Clock::now() - began)
                .count() /
            (double(innerReps) * double(count)));
        began = Clock::now();
        for (int k = 0; k < innerReps; ++k) {
            RigExecApplyWeightedMatrixSimd(in.data(), out.data(), 0.5f,
                                           count, transform);
        }
        scalarNs.push_back(
            std::chrono::duration<double, std::nano>(Clock::now() - began)
                .count() /
            (double(innerReps) * double(count)));
    }
    result.arrayNsPerPoint = Median(arrayNs);
    result.scalarNsPerPoint = Median(scalarNs);
    // Keep the results live so the loops are not optimized away.
    volatile float sink = out[count - 1][0] + out[0][2];
    (void)sink;
    return result;
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: benchMatrixMover <examplesDir> [reps] "
                    "[filter]\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const int reps = argc > 2 ? std::max(1, std::atoi(argv[2])) : 10;
    const std::string filter = argc > 3 ? argv[3] : "";
    std::printf("benchMatrixMover: simd=%d reps=%d filter='%s'\n",
                RigExecSimdEnabled() ? 1 : 0, reps, filter.c_str());

    struct ExtraStage {
        std::string stage;
        std::string frames;
        // Compute-only extras animate their matrix inputs every visited
        // frame, so their matrix steps run kernels rather than skips.
        // They print in their own section and never enter the headline
        // sums, which stay comparable across rounds.
        bool computeOnly = false;
    };
    std::vector<ExtraStage> extras;
    // The matrix mover's own doc example is not in the fixture table.
    extras.push_back({"docs/examples/matrix_mover.usda",
                      "1001,1008,1016", false});
    // The animated stack layers timeSamples over the resting stack, so
    // consecutive visits move every matrix input: the compute path.
    extras.push_back({"examples/biped/Biped_stack_anim.usda",
                      "1,50,100,150,200", true});

    std::vector<RigResult> results;
    const std::string lowerFilter = TfStringToLower(filter);
    const auto matches = [&](const std::string &stage) {
        return lowerFilter.empty() ||
               TfStringToLower(stage).find(lowerFilter) !=
                   std::string::npos;
    };
    for (const RigExecExampleFixture &fixture :
         kRigExecExampleFixtures) {
        if (!matches(fixture.stage)) {
            continue;
        }
        results.push_back(
            BenchRig(examplesDir, fixture.stage, fixture.frames, reps));
    }
    // Extra stages resolve against the examples dir only when they name
    // no parent; the doc example sits beside it.
    for (const ExtraStage &extra : extras) {
        if (!matches(extra.stage)) {
            continue;
        }
        const std::string dirEnd = "/examples";
        std::string root = examplesDir;
        const size_t at = root.rfind(dirEnd);
        const std::string stagePath =
            (at == std::string::npos)
                ? extra.stage
                : root.substr(0, at) + "/" + extra.stage;
        RigResult result;
        result.stage = extra.stage;
        result.computeOnly = extra.computeOnly;
        const UsdStageRefPtr usdStage = UsdStage::Open(stagePath);
        if (usdStage) {
            const SdfPath rigPath = FindRig(usdStage);
            if (!rigPath.IsEmpty()) {
                // Reuse BenchRig through a synthetic examples-relative
                // path: open from the stage's own directory instead.
                const size_t slash = stagePath.rfind('/');
                const std::string dir = stagePath.substr(0, slash);
                const std::string file =
                    stagePath.substr(slash + 1);
                result = BenchRig(dir, file.c_str(),
                                  extra.frames.c_str(), reps);
                result.stage = extra.stage;
                result.computeOnly = extra.computeOnly;
            }
        }
        results.push_back(result);
    }

    std::printf("%-38s %5s %8s %9s %9s %9s %9s %5s %5s %9s %10s\n",
                "stage", "baked", "compile", "baked", "mtxStatic",
                "mtxChunk", "mtxWalk", "mRev", "mWalk", "clusters",
                "dynamic");
    std::printf("%-38s %5s %8s %9s %9s %9s %9s %5s %5s %9s %10s\n", "",
                "", "ms", "us/frame", "us/frame", "us/frame", "us/frame",
                "", "", "min/all", "us/frame");
    double logSum = 0.0, bakedSum = 0.0, matrixSum = 0.0;
    size_t bakedCount = 0;
    for (const RigResult &result : results) {
        if (result.computeOnly) {
            continue;
        }
        std::printf(
            "%-38s %5s %8.2f %9.1f %9.2f %9.2f %9.2f %5zu %5zu "
            "%4zu/%-4zu %10s\n",
            result.stage.c_str(), result.baked ? "yes" : "no",
            result.compileMs, result.bakedFrameUs, result.matrixStaticUs,
            result.matrixChunkUs, result.matrixWalkUs,
            result.matrixRevisions, result.matrixWalkOps,
            result.minClustersRun, result.clusters,
            result.dynamicNote.c_str());
        if (result.baked && result.bakedFrameUs > 0.0) {
            logSum += std::log(result.bakedFrameUs);
            bakedSum += result.bakedFrameUs;
            matrixSum += result.matrixChunkUs + result.matrixStaticUs +
                         result.matrixWalkUs;
            ++bakedCount;
        }
    }
    const double geomean =
        bakedCount ? std::exp(logSum / double(bakedCount)) : 0.0;
    std::printf("baked %zu rigs: total %.1f us/frame, geomean %.2f "
                "us/frame, matrix %.1f us/frame (%.1f%%)\n",
                bakedCount, bakedSum, geomean, matrixSum,
                bakedSum > 0.0 ? 100.0 * matrixSum / bakedSum : 0.0);

    // Matrix phase/role split for rigs with matrix revisions. Memo/body/
    // publish are zero unless the step-timing environment above was set.
    std::printf("%-38s %9s %9s %9s %9s %9s %9s %6s %6s\n", "stage",
                "mtxMemo", "mtxBody", "mtxPub", "chkRange", "chkLegacy",
                "chkWhole", "rGroups", "chkSteps");
    std::printf("%-38s %9s %9s %9s %9s %9s %9s %6s %6s\n", "",
                "us/frame", "us/frame", "us/frame", "us/frame", "us/frame",
                "us/frame", "", "");
    double memoSum = 0.0, bodySum = 0.0, pubSum = 0.0, rangeSum = 0.0,
           legacySum = 0.0, wholeSum = 0.0, sMemoSum = 0.0, sBodySum = 0.0,
           sPubSum = 0.0, cMemoSum = 0.0, cBodySum = 0.0, cPubSum = 0.0,
           foldSum = 0.0, fuseSum = 0.0, foMemoSum = 0.0, foBodySum = 0.0,
           foPubSum = 0.0, fuMemoSum = 0.0, fuBodySum = 0.0, fuPubSum = 0.0;
    for (const RigResult &result : results) {
        if (!result.baked || result.matrixRevisions == 0 ||
            result.computeOnly) {
            continue;
        }
        std::printf("%-38s %9.2f %9.2f %9.2f %9.2f %9.2f %9.2f %6zu %6zu\n",
                    result.stage.c_str(), result.matrixMemoUs,
                    result.matrixBodyUs, result.matrixPublishUs,
                    result.matrixChunkRangeUs, result.matrixChunkLegacyUs,
                    result.matrixChunkWholeUs, result.matrixRangeGroups,
                    result.matrixChunkSteps);
        std::printf("%-38s %9s %9.2f %9.2f %9.2f %9s %9s %6s %6s\n", "",
                    "static:", result.matrixStaticMemoUs,
                    result.matrixStaticBodyUs, result.matrixStaticPublishUs,
                    "", "", "", "");
        std::printf("%-38s %9s %9.2f %9.2f %9.2f %9s %9s %6s %6zu/%zu\n", "",
                    "chunk:", result.matrixChunkMemoUs,
                    result.matrixChunkBodyUs, result.matrixChunkPublishUs,
                    "", "", "", result.matrixChunkRangeSteps,
                    result.matrixChunkLegacySteps);
        std::printf("%-38s walk computed %llu reused %llu skippedWhole "
                    "%llu\n",
                    "", (unsigned long long)result.matrixWalkComputed,
                    (unsigned long long)result.matrixWalkReused,
                    (unsigned long long)result.matrixWalkSkipped);
        std::printf("%-38s fold %9.2f fuse %9.2f us/frame (supporting, "
                    "outside headline)\n",
                    "", result.matrixFoldUs, result.matrixFuseUs);
        std::printf("%-38s %9s %9.2f %9.2f %9.2f %9s %9s %6s %6s\n", "",
                    "fold:", result.matrixFoldMemoUs,
                    result.matrixFoldBodyUs, result.matrixFoldPublishUs,
                    "", "", "", "");
        std::printf("%-38s %9s %9.2f %9.2f %9.2f %9s %9s %6s %6s\n", "",
                    "fuse:", result.matrixFuseMemoUs,
                    result.matrixFuseBodyUs, result.matrixFusePublishUs,
                    "", "", "", "");
        memoSum += result.matrixMemoUs;
        bodySum += result.matrixBodyUs;
        pubSum += result.matrixPublishUs;
        sMemoSum += result.matrixStaticMemoUs;
        sBodySum += result.matrixStaticBodyUs;
        sPubSum += result.matrixStaticPublishUs;
        cMemoSum += result.matrixChunkMemoUs;
        cBodySum += result.matrixChunkBodyUs;
        cPubSum += result.matrixChunkPublishUs;
        rangeSum += result.matrixChunkRangeUs;
        legacySum += result.matrixChunkLegacyUs;
        wholeSum += result.matrixChunkWholeUs;
        foldSum += result.matrixFoldUs;
        fuseSum += result.matrixFuseUs;
        foMemoSum += result.matrixFoldMemoUs;
        foBodySum += result.matrixFoldBodyUs;
        foPubSum += result.matrixFoldPublishUs;
        fuMemoSum += result.matrixFuseMemoUs;
        fuBodySum += result.matrixFuseBodyUs;
        fuPubSum += result.matrixFusePublishUs;
    }
    std::printf("matrix phases total: memo %.1f body %.1f publish %.1f "
                "us/frame; chunk range %.1f legacy %.1f whole %.1f\n",
                memoSum, bodySum, pubSum, rangeSum, legacySum, wholeSum);
    std::printf("matrix static phases: memo %.1f body %.1f publish %.1f; "
                "chunk phases: memo %.1f body %.1f publish %.1f\n",
                sMemoSum, sBodySum, sPubSum, cMemoSum, cBodySum, cPubSum);
    std::printf("matrix fold %.1f fuse %.1f us/frame (supporting, outside "
                "headline)\n",
                foldSum, fuseSum);
    std::printf("matrix fold phases: memo %.1f body %.1f publish %.1f; "
                "fuse phases: memo %.1f body %.1f publish %.1f\n",
                foMemoSum, foBodySum, foPubSum, fuMemoSum, fuBodySum,
                fuPubSum);

    // Compute-frame matrix: animated inputs, so kernels run. Supporting
    // metric only; excluded from every sum above.
    bool anyCompute = false;
    for (const RigResult &result : results) {
        if (result.computeOnly && result.baked) {
            anyCompute = true;
            break;
        }
    }
    if (anyCompute) {
        std::printf("compute-frame matrix (supporting, excluded from "
                    "sums):\n");
        std::printf("%-38s %9s %9s %9s %9s %9s %12s %12s %9s\n", "stage",
                    "baked", "mtxStatic", "mtxChunk", "chkRange",
                    "chkLegacy", "computed", "reused", "skipped");
        std::printf("%-38s %9s %9s %9s %9s %9s %12s %12s %9s\n", "",
                    "us/frame", "us/frame", "us/frame", "us/frame",
                    "us/frame", "entries", "entries", "walks");
        for (const RigResult &result : results) {
            if (!result.computeOnly || !result.baked) {
                continue;
            }
            std::printf("%-38s %9.1f %9.2f %9.2f %9.2f %9.2f %12llu "
                        "%12llu %9llu\n",
                        result.stage.c_str(), result.bakedFrameUs,
                        result.matrixStaticUs, result.matrixChunkUs,
                        result.matrixChunkRangeUs, result.matrixChunkLegacyUs,
                        (unsigned long long)result.matrixWalkComputed,
                        (unsigned long long)result.matrixWalkReused,
                        (unsigned long long)result.matrixWalkSkipped);
        }
    }

    std::printf("kernel RigExecApplyWeightedMatrixSimd ns/point "
                "(median):\n");
    std::printf("%10s %12s %12s\n", "points", "array", "scalar0.5");
    for (const size_t count : {size_t(1024), size_t(16384),
                               size_t(262144)}) {
        const int inner =
            count <= 1024 ? 200 : (count <= 16384 ? 20 : 3);
        const KernelResult kernel = BenchKernel(count, 5, inner);
        std::printf("%10zu %12.2f %12.2f\n", count,
                    kernel.arrayNsPerPoint, kernel.scalarNsPerPoint);
    }
    return 0;
}
