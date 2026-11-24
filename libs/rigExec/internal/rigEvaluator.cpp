// Lifecycle, compiled program ownership, and generation dispatch.

#include "rigExec/internal/rigEvaluatorInternal.h"
#include "rigExec/internal/rigEvaluatorDependencies.h"
#include "rigExec/internal/rigEvaluatorPropertyBindings.h"
#include "rigExec/internal/parallel.h"
#include "rigExec/api/evaluationObserver.h"
#include "rigExec/internal/oracleDispatch.h"

#include "pxr/base/work/detachedTask.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/pyLock.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"

#include <algorithm>
#include <cassert>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

RigExecEvaluationObserverFactory gObserverFactory = nullptr;

/// Destroys a program the evaluator has replaced, off this thread.
///
/// A retiring program is a large batch of frees -- every
/// step, table, query and geometry buffer of a compiled rig -- and nothing waits on
/// them: once the replacement exists, no frame, notice or tool can reach the
/// old one again. So the destroy is detached rather than paid on the thread
/// that is recompiling.
///
/// Two rules make that safe. The stage references go FIRST, here, so the
/// detached delete can never be the one that drops a stage's last reference
/// and tears it down on a worker; what remains (prims, attributes, queries,
/// shared layouts) holds atomic refcounts and owns nothing of the stage. And
/// the destroy touches nothing but the program: the evaluator pointers it
/// carries are never followed by its destructor, so the evaluator may itself
/// be gone by the time it runs. It owns no exec object either -- the guide
/// taps it reads are the evaluator's -- so the single exec lane is not
/// entered.
///
/// Inline where a task is not allowed: with parallel evaluation off, and
/// inside a frozen run, whose thread must dispatch nothing.
void
_RetireBakedProgram(std::unique_ptr<RigExecBakedProgram> retiring)
{
    if (!retiring || !RigExecParallelEvaluationEnabled() ||
        RigExecFrozenSerialActive()) {
        return;
    }
    retiring->ReleaseStageReferences();
    // A raw pointer, because a detached task is invoked through a const call
    // operator and so cannot move out of a capture it owns.
    WorkRunDetachedTask([program = retiring.release()] { delete program; });
}

} // namespace

RigExecRigEvaluator::RigExecRigEvaluator(const UsdStageRefPtr &stage,
                                         const SdfPath &rigPath)
    : _stage(stage), _rigPath(rigPath),
      _restInputNames(evaluatorDetail::_MakeRestInputNames())
{
    if (gObserverFactory)
        gObserverFactory(*this);
    // The static-input cache lives here and is consulted through the
    // resolved-input lookup every read already goes through.
    _resolvedInputs.SetStaticCache(&_staticInputs);
    if (_stage) {
        _noticeKey = TfNotice::Register(TfCreateWeakPtr(this),
                                        &RigExecRigEvaluator::_OnObjectsChanged,
                                        UsdStageWeakPtr(_stage));
    }
    // The rig evaluates directly against the source stage; the engine
    // authors nothing, so there is no derived stage.
}

RigExecRigEvaluator::~RigExecRigEvaluator()
{
    _observers.clear();
    TfNotice::Revoke(_noticeKey);
}

void
RigExecSetEvaluationObserverFactory(RigExecEvaluationObserverFactory factory)
{
    gObserverFactory = factory;
}

void
RigExecRigEvaluator::AddEvaluationObserver(
    std::unique_ptr<RigExecEvaluationObserver> observer)
{
    _AssertOwnerThread();
    if (observer)
        _observers.push_back(std::move(observer));
}

RigExecRigPose
RigExecRigEvaluator::Evaluate(UsdTimeCode time)
{
    _AssertOwnerThread();
    struct ObserverFrameScope {
        RigExecRigEvaluator &evaluator;
        ~ObserverFrameScope()
        {
            evaluator._NotifyReverse(
                [this](RigExecEvaluationObserver &observer) {
                    observer.FrameEnded(evaluator);
                });
        }
    } observerScope{*this};
    _Notify([&](RigExecEvaluationObserver &observer) {
        observer.FrameStarting(*this, time);
    });
    RigExecRigPose pose = _EvaluateGeneration(time);
    _Notify([&](RigExecEvaluationObserver &observer) {
        observer.FrameEvaluated(*this, time, pose);
    });
    return pose;
}

void
RigExecRigEvaluator::SetPublishWeightFields(bool publish)
{
    _AssertOwnerThread();
    _Notify([&](RigExecEvaluationObserver &observer) {
        observer.OptionsChanging(*this, _solverGuidesEnabled, publish);
    });
    _publishWeightFields = publish;
}

void
RigExecRigEvaluator::SetBakedRoleMode(RigExecBakedRoleMode mode,
                                      std::set<SdfPath> exportKeep)
{
    _AssertOwnerThread();
    _Notify([&](RigExecEvaluationObserver &observer) {
        observer.BakedRoleModeChanging(*this, mode, exportKeep);
    });
    if (mode == _bakedRoleMode && exportKeep == _bakedExportKeep) {
        return;
    }
    _bakedRoleMode = mode;
    _bakedExportKeep = std::move(exportKeep);
    // Build reads both; the standing program was classified without them.
    _programState.Transition(_ProgramState::Edge::Staled);
}

RigExecScopedBakedRoleMode::RigExecScopedBakedRoleMode(
    RigExecRigEvaluator &evaluator, RigExecBakedRoleMode mode,
    std::set<SdfPath> exportKeep)
    : _evaluator(evaluator), _previous(evaluator.GetBakedRoleMode()),
      _previousKeep(evaluator.GetBakedExportKeep())
{
    _evaluator.SetBakedRoleMode(mode, std::move(exportKeep));
}

RigExecScopedBakedRoleMode::~RigExecScopedBakedRoleMode()
{
    _evaluator.SetBakedRoleMode(_previous, std::move(_previousKeep));
}

void
RigExecRigEvaluator::SetSolverGuidesEnabled(bool enabled)
{
    _AssertOwnerThread();
    _Notify([&](RigExecEvaluationObserver &observer) {
        observer.OptionsChanging(*this, enabled, _publishWeightFields);
    });
    _solverGuidesEnabled = enabled;
}

RigExecRigPose
RigExecRigEvaluator::_EvaluateGeneration(UsdTimeCode time)
{
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    _programState.Transition(_ProgramState::Edge::GenerationBegan);
    RIGEXEC_PROFILE_SCOPE_CAT(
        _profiler,
        time.IsDefault() ? std::string("Evaluate@default")
                         : "Evaluate@" + TfStringPrintf("%g", time.GetValue()),
        "evaluate");
    RigExecRigPose pose;
    pose.time = time;
    if (cpuReference) {
        std::string error;
        if (!RigExecOracleAvailable(&error)) {
            pose.diagnostics.push_back(std::move(error));
            return pose;
        }
    }
    // Preserve the observable rest-classification transition at this owner
    // boundary. The standing program supplies the compiled rig's chain authority;
    // the existing rebuild below remains the sole binding rebuild.
    const size_t buildsBeforeSettle = _bakedProgramBuilds;
    bool restBecameVarying = false;
    if (_rigState.Compiled() && _compiledRigRestsConstant &&
        _programState.Stale() && !_restEditedProviders.empty()) {
        std::set<SdfPath> chainTargets;
        for (const auto &entry : _propertyChains)
            chainTargets.insert(entry.first);
        for (const SdfPath &provider : _restEditedProviders) {
            if (_ProviderRestMightVary(_stage, _restInputNames, provider,
                                       chainTargets)) {
                restBecameVarying = true;
                break;
            }
        }
    }
    {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.Settle", "evaluate");
        if (!_SettleCompiledRig(&pose.diagnostics))
            return pose;
    }
    if (_programState.Stale()) {
        // A full structural recompile already published its own compiled rig reason.
        const bool restTransition =
            restBecameVarying && _bakedProgramBuilds == buildsBeforeSettle;
        _programState.Transition(_ProgramState::Edge::StaleTaken);
        _RebuildBakedProgram(std::move(_bakedProgram));
        if (restTransition && _bakedProgram) {
            // Rest channels become per-frame after the first varying
            // transition.
            _compiledRigRestsConstant = false;
            pose.diagnostics.push_back(
                "rest channel became time-varying: compiled rig rebuilt");
        }
    }
    if (_rigState.Compiled() && !_bakedProgram &&
        _programState.MayBuildAt(_stageEditSerial)) {
        _RebuildBakedProgram();
    }
    if (!_upstreamRequested.empty() || !_upstreamAdmitted.empty()) {
        _AdmitUpstreamInputs(time);
        pose.diagnostics.insert(pose.diagnostics.end(),
                                _upstreamDropLines.begin(),
                                _upstreamDropLines.end());
    }
    const auto unavailable = [&]() {
        pose.diagnostics.push_back(
            "native program unavailable for compiled rig");
        const std::vector<std::string> &reasons =
            _programState.RefusalReasons();
        pose.diagnostics.insert(pose.diagnostics.end(), reasons.begin(),
                                reasons.end());
    };
    if (!_bakedProgram) {
        unavailable();
        return pose;
    }
    // Places this generation's inputs on the standing program and runs it.
    // False with *placed false when it refuses the interactive inputs.
    const size_t diagnosticsBeforeRun = pose.diagnostics.size();
    const auto placeAndRun = [&](bool *placed) {
        {
            RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.PlaceOverrides",
                                      "evaluate");
            _bakedProgram->SetUpstreamInputs(_upstreamAdmitted);
            *placed = _bakedProgram->SetOverrides(_interactiveOverrides);
            if (!*placed) {
                pose.diagnostics.push_back(
                    "native program refused interactive inputs");
                return false;
            }
            _bakedProgram->SetPublishWeightFields(_publishWeightFields);
        }
        // Windowed operations read their inputs at the earlier ticks of
        // their windows: run the ticks the memo lacks, then place the rings.
        if (_bakedProgram->GetWindowedOpCount()) {
            RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.WindowHistory",
                                      "evaluate");
            std::vector<RigExecWindowRing> rings;
            std::string why;
            if (!GetWindowRings(time, &rings, &why))
                pose.diagnostics.push_back(why);
            _bakedProgram->SetWindowRings(rings);
        }
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Evaluate.Run", "evaluate");
        return _bakedProgram->Run(time, &pose);
    };
    bool placed = true;
    bool completed = placeAndRun(&placed);
    if (!placed)
        return pose;
    if (!completed &&
        _bakedProgram->GetLastBail() == RigExecBakedBail::RoleFlip) {
        // A Range role or group gate rested on a value this generation
        // sampled otherwise. Rebuild from the values standing now (Build
        // classifies with the overrides and admitted upstream inputs) and
        // run once more; the bailed run leaves no line, and a second
        // failure is reported like any other.
        pose.diagnostics.erase(pose.diagnostics.begin() +
                                   std::ptrdiff_t(diagnosticsBeforeRun),
                               pose.diagnostics.end());
        _RebuildBakedProgram(std::move(_bakedProgram));
        if (!_bakedProgram) {
            unavailable();
            return pose;
        }
        completed = placeAndRun(&placed);
        if (!placed)
            return pose;
    }
    if (!completed) {
        pose.valid = false;
        pose.diagnostics.push_back(
            "native program did not complete generation");
        return pose;
    }
    _bakedProgram->AdoptBlendSampleLayouts(&_blendSampleShapes);
    _NoteWindowCaptures(time);
    _rigState.skinTopologyObservationPending = false;
    ++_bakedGenerations;
    _programState.Transition(_ProgramState::Edge::Published);
    return pose;
}

void
RigExecRigEvaluator::_RebuildBakedProgram(
    std::unique_ptr<RigExecBakedProgram> outgoing,
    const RigExecSceneDescriptors *scene, const UsdStageWeakPtr &sceneStage,
    uint64_t sceneSerial)
{
    // Read before anything can replace the program it describes, and cleared
    // here because from this line on no program of this evaluator has
    // published anything.
    // The replacement has answered no generation, so the op trace and op
    // graph accessors describe none until it runs one.
    const bool outgoingPublished = _programState.Published();
    _programState.Transition(_ProgramState::Edge::RebuildBegan);
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "Compile.Bake", "compile");
    ++_bakedProgramBuildAttempts;
    // An edit during the build makes a refusal transient: key it on the
    // serial the build started from.
    const uint64_t buildSerial = _stageEditSerial;
    // Preserve concrete compile-admission reasons for an invalid generation.
    std::vector<std::string> reasons;
    _bakedProgram = RigExecBakedProgram::_BuildWithSceneCapture(
        this, &reasons, scene, sceneStage, sceneSerial);
    if (_bakedProgram) {
        ++_bakedProgramBuilds;
        std::map<SdfPath, SdfPath> switchOwners;
        for (const auto &sw : _spaceSwitches)
            switchOwners.emplace(sw.target, sw.switchPath);
        for (const auto &[owner, reason] :
             _bakedProgram->_GetCycleSkips(switchOwners))
            _skippedOperations.emplace(owner, reason);
        if (outgoing && outgoingPublished) {
            // The replacement inherits the geometry nodes that survive. Without
            // it a rebuilt program re-runs every per-point kernel and
            // publishes "created"/"schedule(s) built" counters, and the mover
            // graph diagnostic, for nodes nothing rebuilt. Only from a program
            // that PUBLISHED a generation: an unrun program carries only
            // `created = false` on nodes whose creation no consumer has been
            // told about, so adopting it would under-report work. That case is
            // reached
            // by building at Compile and changing the mode afterwards, which is
            // what a rig carrying rigExec:baked does whenever a tool then asks
            // for the parity check.
            _bakedProgram->AdoptGeometryStateFrom(*outgoing);
        }
    }
    _programState.Transition(_ProgramState::Edge::RebuildEnded,
                             _bakedProgram != nullptr, buildSerial,
                             std::move(reasons));
    // Last, once the replacement has taken what it adopts from it.
    _RetireBakedProgram(std::move(outgoing));
    _Notify([&](RigExecEvaluationObserver &observer) {
        observer.ProgramReplaced(*this);
    });
}

bool
RigExecRigEvaluator::IsBakeable(std::vector<std::string> *reasons) const
{
    return RigExecBakedProgram::IsBakeable(*this, reasons);
}

size_t
RigExecRigEvaluator::GetBakedClusterCount() const
{
    return _bakedProgram ? _bakedProgram->GetClusterCount() : 0;
}

size_t
RigExecRigEvaluator::GetBakedClustersRunLastGeneration() const
{
    return _bakedProgram ? _bakedProgram->GetClustersRunLastGeneration() : 0;
}

void
RigExecRigEvaluator::_ProgramState::Transition(Edge edge)
{
    switch (edge) {
    case Edge::GenerationBegan:
        _lastGenerationRanProgram = false;
        break;
    case Edge::Published:
        assert(!_lastGenerationRanProgram);
        _published = true;
        _lastGenerationRanProgram = true;
        break;
    case Edge::RebuildBegan:
        _published = false;
        _lastGenerationRanProgram = false;
        break;
    case Edge::RebuildEnded:
        assert(false && "RebuildEnded carries its outcome");
        break;
    case Edge::Staled:
        _stale = true;
        break;
    case Edge::NoticeStaled:
        _stale = true;
        _refused = false;
        _refusalReasons.clear();
        break;
    case Edge::StaleTaken:
        assert(_stale);
        _stale = false;
        break;
    case Edge::CompileBegan:
        _stale = false;
        _refused = false;
        _refusalReasons.clear();
        break;
    }
}

void
RigExecRigEvaluator::_ProgramState::Transition(Edge edge, bool built,
                                               uint64_t serial,
                                               std::vector<std::string> reasons)
{
    assert(edge == Edge::RebuildEnded);
    (void)edge;
    // Remembered per stage edit serial so Evaluate does not rebuild per
    // frame; Evaluate reports the reasons on each frame without a program.
    _refused = !built;
    _refusedSerial = serial;
    _refusalReasons = std::move(reasons);
}

} // namespace rigExec
