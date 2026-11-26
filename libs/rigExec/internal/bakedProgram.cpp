// The baked program: construction, accessors and the frame it runs. See
// bakedProgram.h for what it is. Build captures what a frame needs of the
// evaluator once; the pose and geometry halves read the program.
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

// The bake.

RigExecBakedProgram::RigExecBakedProgram(
    _FactoryTag, std::unique_ptr<RigExecBakedProgramImpl> impl)
    : RigExecBakedProgram(std::move(impl))
{
}

RigExecBakedProgram::RigExecBakedProgram(
    std::unique_ptr<RigExecBakedProgramImpl> impl)
    : _impl(std::move(impl)),
      _compiled(std::make_shared<RigExecCompiledProgram>(
          RigExecCompiledProgram::_VersionTag(0), _impl))
{
}

RigExecBakedProgram::~RigExecBakedProgram() = default;

size_t
RigExecBakedProgram::GetProviderCount() const
{
    return _impl->paths.size();
}
std::vector<RigExecBakedProgram::WindowedOpInfo>
RigExecBakedProgram::GetWindowedOps() const
{
    std::vector<WindowedOpInfo> out;
    out.reserve(_impl->windowedOps.size());
    for (const auto &op : _impl->windowedOps) {
        WindowedOpInfo info;
        info.path = op.path;
        info.frames = op.frames;
        info.inner = op.inner;
        info.feedsWindow = op.feedsWindow;
        out.push_back(std::move(info));
    }
    return out;
}
size_t
RigExecBakedProgram::GetWindowedOpCount() const
{
    return _impl->windowedOps.size();
}
void
RigExecBakedProgram::SetWindowRings(const std::vector<RigExecWindowRing> &rings)
{
    RigExecBakedPlaceWindowRings(_impl.get(), rings);
}
std::vector<RigExecWindowPacketPtr>
RigExecBakedProgram::GetWindowCaptures() const
{
    return _impl->windowCaptures;
}
size_t
RigExecBakedProgram::GetBoundInputCount() const
{
    return _impl->boundInputs;
}
size_t
RigExecBakedProgram::GetVaryingInputCount() const
{
    return _impl->varyingInputs;
}

void
RigExecBakedProgram::ReleaseStageReferences()
{
    _impl->runState = RigExecBakedRunState::Retired;
    _impl->stage.Reset();
}

// Exact equality, not a tolerance: the program exists to produce the same
// numbers, so "close" is a failure with extra steps.
void
RigExecComparePoses(const RigExecRigPose &reference,
                    const RigExecRigPose &baked, RigExecRigPose *out)
{
    // Read FIRST, before anything below appends: `out` is allowed to be the
    // reference pose -- the evaluator passes it that way, so a disagreement
    // is reported on the generation it publishes -- and a diagnostic appended
    // by an earlier domain would otherwise make this domain disagree about
    // a pose that agreed.
    const bool sameDiagnostics = reference.diagnostics == baked.diagnostics;
    const auto sameFrame = [](const RigExecPointFrame &a,
                              const RigExecPointFrame &b) {
        return a.flags == b.flags && a.points == b.points;
    };
    const auto compare = [&out](const auto &referenceMap, const auto &bakedMap,
                                const char *what, auto equal) {
        for (const auto &[path, value] : referenceMap) {
            const auto found = bakedMap.find(path);
            if (found == bakedMap.end()) {
                out->diagnostics.push_back(std::string("baked parity: no ") +
                                           what + " for " + path.GetString());
                ++out->comparisonMismatches;
            } else if (!equal(value, found->second)) {
                out->diagnostics.push_back(std::string("baked parity: ") +
                                           what + " differs at " +
                                           path.GetString());
                ++out->comparisonMismatches;
            }
        }
        for (const auto &[path, value] : bakedMap) {
            if (!referenceMap.count(path)) {
                out->diagnostics.push_back(
                    std::string("baked parity: unexpected ") + what + " at " +
                    path.GetString());
                ++out->comparisonMismatches;
            }
        }
    };
    const auto sameMatrix = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        return a == b;
    };
    compare(reference.jointFramesBase, baked.jointFramesBase,
            "base joint frame", sameFrame);
    compare(reference.jointFramesFinal, baked.jointFramesFinal,
            "final joint frame", sameFrame);
    compare(reference.jointMatricesFinal, baked.jointMatricesFinal,
            "joint matrix", sameMatrix);
    compare(reference.controlFrames, baked.controlFrames, "control frame",
            sameFrame);
    compare(reference.providerXforms, baked.providerXforms,
            "provider transform", sameMatrix);
    compare(reference.providerBaseXforms, baked.providerBaseXforms,
            "provider base transform", sameMatrix);
    compare(reference.movedProperties, baked.movedProperties, "moved property",
            [](const VtValue &a, const VtValue &b) { return a == b; });
    // The observational guides too: they are a published domain like any
    // other, and a solver aggregate that drifts shows up here first.
    compare(reference.solverFrames, baked.solverFrames, "solver frames",
            [&sameFrame](const std::vector<RigExecPointFrame> &a,
                         const std::vector<RigExecPointFrame> &b) {
                return a.size() == b.size() &&
                       std::equal(a.begin(), a.end(), b.begin(), sameFrame);
            });
    // Weight domains: per-mover resolved field and per-volume placement.
    // A field is equal iff it weights the same property with the same
    // floats, bit for bit -- a resolved field is what a mover actually
    // consumed, and an element one path clamped and the other did not is
    // exactly the difference a size check cannot see.
    compare(reference.weightFields, baked.weightFields, "weight field",
            [](const RigExecResolvedWeightField &a,
               const RigExecResolvedWeightField &b) {
                return a.target == b.target && a.weights == b.weights;
            });
    compare(reference.weightFrames, baked.weightFrames, "weight frame",
            sameMatrix);

    // movedPropertiesCpu, referenceMismatches and
    // referenceAgreements are left out for the opposite reason: they
    // are filled only by cpuReference, and cpuReference turns the baked
    // path OFF (the independent CPU oracle is what that mode asks for, and
    // the program is not it -- it shares the kernels). So in every
    // generation this function ever sees, all three are empty or zero on
    // both sides, and comparing them would assert a tautology. `valid` and
    // `time` likewise: the parity generation publishes the DYNAMIC pose, so
    // its own are the only ones a consumer reads.
    if (!sameDiagnostics) {
        out->diagnostics.push_back(
            "baked parity: diagnostics differ (" +
            std::to_string(reference.diagnostics.size()) + " vs " +
            std::to_string(baked.diagnostics.size()) + " line(s))");
        ++out->comparisonMismatches;
    }
}

// Interactive overrides.

void
RigExecBakedProgram::BumpProgramStamp()
{
    ++_impl->programStamp;
}

namespace {

// One frame.
// The prologue is here rather than in bakedPose.cpp because it is the one
// part of a frame that calls the evaluator's own private routines; everything
// it produces reaches the two halves through the program.

// The per-source constraint tables Run reads, interned once at load:
// interning takes the token registry's lock.
const std::array<TfToken, 4> kConstraintArrayNames = {
    TfToken(SchemaAttrText::inputsSourceWeights),
    TfToken(SchemaAttrText::inputsTranslationOffsets),
    TfToken(SchemaAttrText::inputsRotationOffsets),
    TfToken(SchemaAttrText::inputsPoleVectorWeights)};

} // namespace

const std::array<TfToken, 4> &
RigExecBakedProgramImpl::ConstraintArrays::Names()
{
    return kConstraintArrayNames;
}

bool
RigExecBakedProgram::Run(UsdTimeCode time, RigExecRigPose *pose)
{
    if (_impl->runState == RigExecBakedRunState::Retired) {
        if (pose) {
            pose->diagnostics.push_back("program released: run refused");
        }
        return false;
    }
    RigExecBakedProgramImpl &B = *_impl;
    if (B.inputReplayCapture) {
        RigExecInputReplayObserver::UnsupportedAction(
            "RigExecBakedProgram::Run");
    }
    _lastBail = RigExecBakedBail::None;
    RigExecRigEvaluator &E = *B.evaluator;
    if (E.cpuReference) {
        std::string error;
        if (!RigExecOracleAvailable(&error)) {
            pose->valid = false;
            pose->diagnostics.push_back(std::move(error));
            _lastBail = RigExecBakedBail::Step;
            return false;
        }
    }
    B.recordOpTimings = E.GetOpTimingEnabled();
    if (E.cpuReference) {
        if (!B.oraclePublications)
            B.oraclePublications.emplace();
    } else {
        B.oraclePublications.reset();
        B.oracleWeightInputs.clear();
    }
    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "Baked", "baked");
    RigExecBakedClearRunStamps(&B);
    // Consumed by this run whatever becomes of it.
    const bool fullRunRequested = _fullRunRequested;
    _fullRunRequested = false;

    // Two clock reads per phase, and only when asked: the profiler's scopes
    // take three mutexes apiece and cost more than the prologue they would
    // be measuring, which is why the phase a frame spends its time in was
    // never readable without the tracer distorting it. See
    // RigExecBakedStepTimingRequested, read at Build.
    const bool measuring = B.scheduleStepTiming;
    const auto now = [] {
        return std::chrono::duration<double, std::micro>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };
    const double frameBegan = measuring ? now() : 0;
    double phaseMark = frameBegan;
    // This frame's three phases, held here and folded into the program's
    // sums at the one exit that counts a frame: the divisor is incremented
    // there too, and a generation a step hands back or the publication
    // declines must not put a prologue and a region into a numerator whose
    // denominator stood still.
    double prologueUs = 0, regionUs = 0;

    // Capture required target sources in authored order. A failed read is
    // request-wide typed admission refusal, not a degenerate numerical frame.
    const auto stageFrames = [&B, &E, time]() {
        B.requiredStageFramesAdmission = RigExecRequiredStageFramesAdmission();
        if (B.xformSlots.empty() && B.nativeSources.empty() &&
            B.deltaBasePaths.empty()) {
            return true;
        }
        UsdGeomXformCache cache(time);
        for (size_t k = 0; k < B.xformSlots.size(); ++k) {
            const size_t slot = size_t(B.xformSlots[k]);
            RigExecPointFrame frame;
            GfMatrix4d matrix(1.0);
            if (!E._FrameFromXformRelativeToAsset(
                    B.assetRoot, &cache, B.paths[slot], &frame, &matrix)) {
                if (B.requiredStageFramesAdmission.admitted) {
                    B.requiredStageFramesAdmission.admitted = false;
                    B.requiredStageFramesAdmission.firstBadTarget = int32_t(k);
                }
                return false;
            }
            B.xformBase[k] = matrix;
            B.base[slot] = frame;
            B.fin[slot] = frame;
        }
        // And the target transform each geometry-domain constraint measures its
        // delta against: a stage read even when the target is a RigExecJoint,
        // because the delta is measured against the authored transform, not the
        // rig frame.
        for (size_t k = 0; k < B.deltaBasePaths.size(); ++k) {
            GfMatrix4d matrix(1.0);
            B.deltaBaseOk[k] = E._FrameFromXformRelativeToAsset(
                B.assetRoot, &cache, B.deltaBasePaths[k], nullptr, &matrix);
            B.deltaBaseMatrix[k] = matrix;
        }
        // And the plain Xformables a constraint reads as a SOURCE. Only the
        // stage half is settled here: the delta such a source rides is the
        // deepest revision above it, which is a frame the walk has not produced
        // yet, so the constraint step performs the ride out of the slots it
        // declared. A source the stage cannot answer for is not a bail: the
        // failure is recorded and carried into the step, which diagnoses it
        // and passes that constraint through.
        for (size_t k = 0; k < B.nativeSources.size(); ++k) {
            RigExecPointFrame frame;
            B.nativeFrameOk[k] = E._FrameFromXformRelativeToAsset(
                B.assetRoot, &cache, B.nativeSources[k].path, &frame, nullptr);
            B.nativeFrames[k] =
                B.nativeFrameOk[k] ? frame : RigExecPointFrame();
        }
        return true;
    };

    // Capture authored arrays only. Their owning Constraint validates and
    // expands neutral defaults after graph readiness has selected the body.
    // A channel is read again only when something that can move its raw
    // read moved (ConstraintArrays::variance); every other one keeps what
    // it holds, which is what the read would answer now.
    const auto constraintArrays = [&B, &E, time, fullRunRequested]() {
        using Arrays = RigExecBakedProgramImpl::ConstraintArrays;
        const uint64_t serial = E.GetStageEditSerial();
        for (Arrays &arrays : B.constraintArrays) {
            const bool all = fullRunRequested || !arrays.sampled ||
                             arrays.stamp != B.programStamp ||
                             arrays.serial != serial;
            const bool timeMoved = !arrays.sampled || time != arrays.time;
            const bool defaultMoved =
                timeMoved && (time.IsDefault() || arrays.time.IsDefault());
            for (size_t channel = 0; channel < 4; ++channel) {
                uint8_t &variance = arrays.variance[channel];
                if (all) {
                    // An edit can author time samples where there were none.
                    variance = Arrays::kVarianceUnknown;
                } else if (!timeMoved) {
                    continue;
                } else if (!defaultMoved) {
                    if (variance == Arrays::kVarianceUnknown) {
                        variance = arrays.Varies(channel) ? Arrays::kVaries
                                                          : Arrays::kFixed;
                    }
                    if (variance == Arrays::kFixed) {
                        continue;
                    }
                }
                VtValue value;
                const UsdAttribute &attribute = arrays.attributes[channel];
                if (attribute) {
                    attribute.Get(&value, time);
                }
                arrays.raw[channel] = std::move(value);
                ++B.constraintArrayReads;
            }
            arrays.sampled = true;
            arrays.time = time;
            arrays.stamp = B.programStamp;
            arrays.serial = serial;
        }
    };

    // Serial, always run, and the only part of a frame that may take a lock,
    // read the stage through anything but a pinned query, or touch the pose
    // as it goes. Everything the region needs from outside itself is settled
    // here.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedPrologue", "baked");
        // The standing overrides' paths, which gate the path leaves' reads.
        RigExecBakedPrepareOverridePaths(&B);
        // A math mover's inputs are all authored on itself, so its chain owes
        // exec nothing and resolves first: the head tier runs the chains
        // before anything else reads a result.
        B.propertyResults.clear();
        // The geometry-domain constraint deltas are NOT emptied here. They
        // are slots: what a skipped constraint step left is what it would
        // have measured again, and clearing them would make a cone that
        // skips one deform as though no constraint had ever measured a
        // delta. Keeping them across runs the way a slot is kept avoids that.
        B.resolvedInputs->Clear();

        // Upstream values: what moved since the last run marks the leaves
        // it reaches, and the weight field's paths take their values.
        RigExecBakedPlaceUpstream(&B);
        // Interactive overrides are applied before the property chains, as
        // the source boundary requires: a drag on a chain's target is its
        // base, and the chain publishes its own result in the drag's place.
        if (!B.interactiveOverrides->empty()) {
            E._ApplyInteractiveOverridesToResolved(B.resolvedInputs);
        }
        // Every bound input's value at this time, read here, on this
        // thread: nothing after this point reads a binding any other way.
        // A forced run trusts no leaf it holds. The bindings whose walks
        // reach a chain result are read once the chains are published.
        RigExecBakedSampleLeaves(&B, time, fullRunRequested,
                                 RigExecBakedLeafPass::BeforeHead);
        if (B.hasPropertyChains || !B.weightFields.empty() ||
            !B.headLeaves.empty()) {
            RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "PropertyChains",
                                      "property");
            // The property revisions, from leaves sampled now. The bake's
            // forced run executes every op, so what it exports is computed
            // at its time; every other run re-runs only what moved, and the
            // publication refills the results and the overlay either way.
            for (auto &object : B.weightObjects) {
                RigExecBakedSamplePathLeaves(&B, &object.fieldLeaves, time,
                                             fullRunRequested);
            }
            RigExecBakedSampleHeadLeaves(&B, time, fullRunRequested);
        }
        RigExecBakedSampleLeaves(&B, time, fullRunRequested,
                                 RigExecBakedLeafPass::ChainRouted);
        // The rest and ladder ops, from leaves sampled now (a chain-written
        // channel among them): each re-runs only when what it reads moved,
        // and the bake's forced run composes them all at its time.
        _SampleIntervening(&B, time);
        RigExecBakedSampleSpaces(&B, time, fullRunRequested);

        // The skin layouts, from leaves sampled now: an op builds a new
        // handle only when its leaves moved, on every generation, and the
        // geometry prologue adopts it where it resolves a topology.
        for (auto &chain : B.chains) {
            for (auto &revision : chain.revisions)
                if (revision.op == RigExecRevisionOp::Skin)
                    RigExecBakedSampleLayoutLeaves(&B, &revision, time,
                                                   fullRunRequested);
            for (auto &derived : chain.derived)
                if (derived.revision.op == RigExecRevisionOp::Skin)
                    RigExecBakedSampleLayoutLeaves(&B, &derived.revision, time,
                                                   fullRunRequested);
        }

        RigExecBakedRunSolverSources(&B, time);
        if (stageFrames()) {
            constraintArrays();
            RigExecBakedRunGeometryPrologue(&B, time, pose, fullRunRequested);
            // A Range role or group gate rests on a value this run sampled
            // otherwise (an override, an upstream input, a point count): no
            // step runs on it, the pose is untouched, and the owner rebuilds.
            if (!RigExecBakedRolesStand(B)) {
                // The prologue's samples were consumed by no step and the
                // run commits none of its bookkeeping, so a program run
                // again (a held replay) re-reads everything, as after a
                // refused stage-frames run.
                if (B.programStamp == B.lastProgramStamp) {
                    ++B.programStamp;
                }
                _lastBail = RigExecBakedBail::RoleFlip;
                return false;
            }
        }
    }
    if (measuring) {
        const double mark = now();
        prologueUs = mark - phaseMark;
        phaseMark = mark;
    }

    // A run executes the CLOSURE of what the sources say moved, not the whole
    // program. Under RIGEXEC_BAKED_VERIFY_CONES it does both: the cone
    // run's whole answer is shadowed, the state the prologue left is put
    // back, every step runs, and the two are compared. A difference is
    // reported rather than fatal, so a test can assert on the count.
    const bool verifying = RigExecBakedVerifyConesRequested();
    RigExecBakedRunShadow before, after;
    if (verifying) {
        before.Capture(B);
    }
    bool bailed = false;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedRegion", "baked");
        bailed = !RigExecBakedRunSteps(&B, time, fullRunRequested);
    }
    if (measuring) {
        const double mark = now();
        regionUs = mark - phaseMark;
        phaseMark = mark;
    }
    if (verifying) {
        after.Capture(B);
        // What the generation DID, taken before the second pass overwrites
        // it. A verification pass runs everything by construction, so
        // leaving its numbers in place would make every observer -- the run
        // report, GetClustersRunLastGeneration, and so the tests that prove
        // a cone skips anything -- describe the instrument instead of the
        // frame.
        const RigExecBakedRunStatistics coneRun(B);
        const size_t coneClusters = coneRun.closedClusters;
        before.Restore(&B);
        // The second pass is the verifier, not the frame: it is kept out of
        // the per-step accumulators for the same reason the phase marks
        // below skip over it, so that a verified frame's table still says
        // what one frame costs.
        B.measurementSuspended = true;
        const bool bailedFull = !RigExecBakedRunSteps(&B, time, true);
        B.measurementSuspended = false;
        coneRun.Restore(&B);
        std::vector<std::string> differences;
        // A generation both passes stopped publishes nothing: the steps each
        // reached before the stop are no result to compare.
        size_t mismatches =
            bailed && bailedFull ? 0 : after.Compare(B, &differences);
        // Verification must leave the frame's values and memo history intact.
        after.Restore(&B);
        coneRun.Restore(&B);
        if (bailedFull != bailed) {
            ++mismatches;
            differences.push_back(
                "baked cone mismatch: the cone run and the whole program "
                "disagree about giving the generation back");
        }
        bailed = bailedFull;
        pose->comparisonMismatches += mismatches;
        for (std::string &difference : differences) {
            pose->diagnostics.push_back(std::move(difference));
        }
        if (mismatches > 0) {
            // The same wording a real parity disagreement uses, on the same
            // stream, so one regular expression catches both.
            TF_WARN("rigExec: baked parity mismatch: %zu difference(s) "
                    "between the cone run (%zu of %zu cluster(s)) and the "
                    "whole program",
                    mismatches, coneClusters, B.clustering.clusters.size());
        }
        if (measuring) {
            // Charged to no phase at all. The block above ran the whole
            // program a second time and compared two copies of the state;
            // adding that to the epilogue would make the epilogue of a
            // verified frame read several times the epilogue a caller gets.
            phaseMark = now();
        }
    }

    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedEpilogue", "baked");
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedReplayTimings", "baked");
        RigExecBakedReplayStepTimings(B);
    }
    if (B.scheduleCalibrate) {
        RigExecBakedScheduleCalibrate(&B);
    }
    if (B.scheduleReport) {
        // What the schedule DID, as against what Build predicted it would:
        // the structural half was printed once, at Build, and this is the
        // only half that needs a frame to have happened.
        const std::string report = RigExecBakedScheduleRunReport(B);
        std::fwrite(report.data(), 1, report.size(), stderr);
    }
    if (!B.requiredStageFramesAdmission.admitted) {
        RigExecBakedPublishStageFramesRefusal(&B, pose);
        _lastBail = RigExecBakedBail::StageFrames;
        return false;
    }
    if (bailed) {
        _lastBail = RigExecBakedBail::Step;
        RigExecBakedAppendFailedOpLines(B, &pose->diagnostics);
        pose->valid = false;
        return false;
    }
    if (!RigExecBakedPublishPose(&B, pose)) {
        _lastBail = RigExecBakedBail::Publish;
        pose->valid = false;
        return false;
    }
    // Where every volume the walk places ended up: each slot's placement is
    // as its own VolumePlacement op node left it. A slot whose node the cone
    // skipped kept a placement whose final frame did not move, so it is
    // this generation's.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedPublishGeometry", "baked");
        RigExecBakedPublishVolumePlacements(B, &pose->weightFrames);
        RigExecBakedResolveGpuChains(&B, time);
        RigExecBakedPublishGeometry(&B, pose);
        RigExecBakedPublishOpOutputs(B, pose);
        RigExecBakedAppendCycleDiagnostics(B, pose);
    }
    const bool scalarReferenceValid =
        RigExecBakedRunScalarReference(&B, time, pose);

    pose->executedOpCount = B.opExecution.executed;
    pose->valid = scalarReferenceValid;
    if (measuring) {
        // Counted here, at the one exit that published a pose: a frame that
        // bailed before publish did not run the epilogue this is measuring,
        // and its prologue and region are dropped with it so that every
        // term of the table is divided by the frames that produced it. A
        // cold run the per-step sums left out goes to sums of its own.
        const double epilogueUs = now() - phaseMark;
        if (B.coldRunExcluded) {
            B.coldPrologueUs += prologueUs;
            B.coldRegionUs += regionUs;
            B.coldEpilogueUs += epilogueUs;
            ++B.coldFrames;
        } else {
            B.timedPrologueUs += prologueUs;
            B.timedRegionUs += regionUs;
            B.timedEpilogueUs += epilogueUs;
            ++B.timedFrames;
        }
        RigExecBakedStepTimingReport(&B);
    }
    return true;
}

bool
RigExecBakedProgram::SampleStageFrameSeeds(UsdTimeCode time,
                                           RigExecStageFrameSeeds *seeds,
                                           std::string *error) const
{
    if (!seeds) {
        if (error) {
            *error = "no seeds to sample into";
        }
        return false;
    }
    const RigExecBakedProgramImpl &B = *_impl;
    const RigExecRigEvaluator &E = *B.evaluator;
    // Transport exactly live's successfully sampled prefix and typed refusal.
    // Failed and later target buffers retain their captured values; no stage
    // handles cross the source boundary.
    UsdGeomXformCache cache(time);
    RigExecStageFrameSeeds sampled;
    sampled.xformBase = B.xformBase;
    sampled.xformFrames.reserve(B.xformSlots.size());
    for (const int slot : B.xformSlots)
        sampled.xformFrames.push_back(B.base[size_t(slot)]);
    for (size_t k = 0; k < B.xformSlots.size(); ++k) {
        const size_t slot = size_t(B.xformSlots[k]);
        RigExecPointFrame frame;
        GfMatrix4d matrix(1.0);
        if (!E._FrameFromXformRelativeToAsset(B.assetRoot, &cache,
                                              B.paths[slot], &frame, &matrix)) {
            if (sampled.requiredStageFramesAdmission.admitted) {
                sampled.requiredStageFramesAdmission.admitted = false;
                sampled.requiredStageFramesAdmission.firstBadTarget =
                    int32_t(k);
            }
            break;
        }
        sampled.xformBase[k] = matrix;
        sampled.xformFrames[k] = frame;
    }
    sampled.deltaOk.reserve(B.deltaBasePaths.size());
    sampled.deltaBase.reserve(B.deltaBasePaths.size());
    for (size_t k = 0; k < B.deltaBasePaths.size(); ++k) {
        GfMatrix4d matrix(1.0);
        if (sampled.requiredStageFramesAdmission.admitted) {
            sampled.deltaOk.push_back(E._FrameFromXformRelativeToAsset(
                B.assetRoot, &cache, B.deltaBasePaths[k], nullptr, &matrix));
            sampled.deltaBase.push_back(matrix);
        } else {
            sampled.deltaOk.push_back(B.deltaBaseOk[k]);
            sampled.deltaBase.push_back(B.deltaBaseMatrix[k]);
        }
    }
    sampled.nativeOk.reserve(B.nativeSources.size());
    sampled.nativeFrames.reserve(B.nativeSources.size());
    for (size_t k = 0; k < B.nativeSources.size(); ++k) {
        RigExecPointFrame frame;
        const bool ok = sampled.requiredStageFramesAdmission.admitted
                            ? E._FrameFromXformRelativeToAsset(
                                  B.assetRoot, &cache, B.nativeSources[k].path,
                                  &frame, nullptr)
                            : B.nativeFrameOk[k];
        if (!sampled.requiredStageFramesAdmission.admitted)
            frame = B.nativeFrames[k];
        sampled.nativeOk.push_back(ok);
        sampled.nativeFrames.push_back(ok ? frame : RigExecPointFrame());
    }
    sampled.intervening.reserve(B.interveningSlots.size());
    sampled.interveningReset.reserve(B.interveningSlots.size());
    for (size_t k = 0; k < B.interveningSlots.size(); ++k) {
        const size_t slot = size_t(B.interveningSlots[k]);
        const UsdPrim parent =
            B.stage->GetPrimAtPath(B.paths[slot].GetParentPath());
        const UsdPrim anchor = B.stage->GetPrimAtPath(B.interveningAnchors[k]);
        bool reset = false;
        GfMatrix4d matrix =
            parent && anchor && parent != anchor
                ? cache.ComputeRelativeTransform(parent, anchor, &reset)
                : GfMatrix4d(1.0);
        sampled.intervening.push_back(reset ? GfMatrix4d(1.0) : matrix);
        sampled.interveningReset.push_back(reset);
    }
    *seeds = sampled;
    return true;
}

size_t
RigExecBakedProgram::GetClusterCount() const
{
    return _impl->clustering.clusters.size();
}

size_t
RigExecBakedProgram::GetClustersRunLastGeneration() const
{
    return _impl->lastClosedClusters;
}

const RigExecBakedProgramImpl &
RigExecBakedProgram::GetStepGraph() const
{
    return *_impl;
}

const RigExecBakedProgramImpl &
RigExecBakedProgramAccess::StepGraph(const RigExecBakedProgram &program)
{
    return program.GetStepGraph();
}

const RigExecCompiledProgram &
RigExecBakedProgram::GetCompiledProgram() const noexcept
{
    return *_compiled;
}

std::shared_ptr<const RigExecCompiledProgram>
RigExecBakedProgram::ShareCompiledProgram() const noexcept
{
    return _compiled;
}

const std::set<SdfPath> &
RigExecBakedProgram::GetExportPinnedPaths() const
{
    return _impl->exportPinnedPaths;
}

UsdTimeCode
RigExecBakedProbeTime(const UsdStageRefPtr &stage)
{
    return _ProbeTime(stage);
}

void
RigExecBakedProgramTesting::SetOpObservers(const RigExecBakedProgram &program,
                                           std::function<void(uint32_t)> before,
                                           std::function<void(uint32_t)> after)
{
    program._impl->opBeforeBody = std::move(before);
    program._impl->opAfterBody = std::move(after);
}

bool
RigExecBakedProgramTesting::CapturePointReads(
    const RigExecBakedProgram &program)
{
    RigExecBakedProgramImpl &B = *program._impl;
    B.pointCaptures.assign(size_t(B.pointBindingCount),
                           RigExecBakedPointCapture());
    B.capturePointReads = !B.scheduleParallel;
    return B.capturePointReads;
}

std::vector<SdfPath>
RigExecBakedProgramTesting::SolverCheckpointsWithoutAnOutput(
    const std::map<SdfPath, std::set<SdfPath>> &snapshots,
    const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> &solverJoints,
    const std::set<SdfPath> &batched)
{
    return _SolverCheckpointsWithoutAnOutput(snapshots, solverDependencies,
                                             solverJoints, batched);
}

} // namespace rigExec
