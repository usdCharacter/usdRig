// Private state of one compile attempt, shared by its steps.
#ifndef RIGEXEC_RIG_EVALUATOR_COMPILE_ATTEMPT_H
#define RIGEXEC_RIG_EVALUATOR_COMPILE_ATTEMPT_H

#include "rigExec/api/rigEvaluator.h"
#include "rigExec/api/profiler.h"
#include "rigExec/internal/rigEvaluatorDependencies.h"
#include "rigExec/internal/valueAddress.h"

#include "pxr/base/tf/scoped.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

/// Everything the steps of RigExecRigEvaluator::_CompileRigAttempt hand to
/// one another. Members are declared in the order the steps fill them, so
/// they are destroyed in the reverse of that order.
struct RigExecRigEvaluator::_CompileAttempt {
    _CompileAttempt(RigExecRigEvaluator &evaluator,
                    std::vector<std::string> *errors, _CompileFailure *failure,
                    uint64_t compileSerial);
    ~_CompileAttempt();

    _CompileAttempt(const _CompileAttempt &) = delete;
    _CompileAttempt &operator=(const _CompileAttempt &) = delete;

    /// Records \p message as the attempt's failure and returns false.
    bool Fail(const std::string &message, SdfPathVector operations = {});

    // Sequential region stamps: a region spans several steps and their early
    // returns, so RAII scopes cannot mark it; each stamp closes the previous
    // region and opens the next. One branch when disabled. The clock starts
    // when the attempt is made rather than beside the first stamped phase,
    // so the time before DiscoverValidate.Validate starts goes to
    // Compile.Prologue instead of being lost in the parent scope.
    void StampCompileRegion(const char *name);

    void AppendDerivedStart(const UsdRelationship &rel,
                            SdfPathVector *targets) const;
    bool StackBefore(const SdfPath &a, const SdfPath &b) const;
    int StackOrdinalOf(const SdfPath &path) const;

    RigExecRigEvaluator &evaluator;
    std::vector<std::string> *errors;
    _CompileFailure *failure;
    const uint64_t compileSerial;
    const bool profileCompile;
    uint64_t compileRegionStart;
    std::unique_ptr<RigExecBakedProgram> retiringBakedProgram;
    std::map<SdfPath, SdfPath> derivedStartFrames;
    // Digest: 3 tasks (_DigestSegment), stage read-only, joined before commit;
    // the join concatenates segments in order and hashes. digestParts is
    // declared after retiringBakedProgram and before the dispatcher, whose
    // destructor joins; tasks capture only `this`, &digestParts and
    // &digestFootprints.
    std::string digestParts[_DigestSegmentCount];
    _DigestFootprint digestFootprints[_DigestSegmentCount];
    WorkDispatcher digestDispatcher;
    bool digestJoined = false;
    // A compile that returns early discards the digest unread, but the
    // split verifier still checks it there: the rigs Compile rejects
    // include the ones whose pose inputs close a cycle, where the Solvers
    // segment's bytes depend on its walk order. Declared after the
    // dispatcher, so it runs first and waits for the tasks itself.
    const TfScoped<> verifyDigestOnEarlyReturn;
    // The parts WITHIN those regions, for the same reason and on the same
    // clock: a phase that takes a fifth of the compile says nothing about
    // which of its passes to go and look at. Strictly nested inside the
    // stamps above, so the trace shows them as children rather than as a
    // second, competing set of rows. Closed at each phase boundary so no
    // block spans two phases, and on destruction so an early return
    // cannot leave one open.
    RigExecProfilePhases compileBlocks;
    std::function<void()> invalidateCompiledRig;

    // _CompileDiscoverOutputs
    UsdPrim rig;
    std::vector<SdfPath> newJointPaths;
    std::vector<SdfPath> newControlPaths;
    std::vector<SdfPath> newVolumeWeightPaths;
    std::vector<UsdPrim> aggregateSolvers;
    std::vector<SdfPath> solverArrayPaths;

    // _CompileDiscoverProviders
    std::unique_ptr<RigExecSceneDescriptors> compileScene;
    UsdStageWeakPtr compileSceneStage;
    uint64_t compileSceneSerial = 0;
    bool compileSceneReusable = false;
    std::vector<_PoseInterpolator> newPoseInterpolators;
    std::map<SdfPath, SdfPath> newRibbonDriverPoints;

    // _CompileDiscoverMovers
    std::vector<RigExecMoverRecord> newMovers;
    SdfPathVector inertMovers;
    std::vector<_SurfaceProjectorRecord> newSurfaceProjectors;

    // _CompileBindSolverJoints
    std::map<SdfPath, int> solverStackOrdinal;
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> newJointBinding;

    // _CompileSolverDag
    std::set<SdfPath> skeleton2dBones;
    std::map<SdfPath, SdfPathVector> skeleton2dSwitchSources;
    std::set<SdfPath> jointWritingSolvers;
    std::vector<std::pair<SdfPath, SdfPath>> poseReverseEdges;
    std::map<SdfPath, std::set<SdfPath>> newSolverDependencies;
    std::map<SdfPath, std::set<SdfPath>> newSolverAggregateReads;

    // _CompileVolumeWeights
    std::vector<RigExecValueOverride> newFalloffLutOverrides;
    std::set<SdfPath> newCurrentPhaseWeights;
    std::set<SdfPath> newVolumeWeightProviders;
    std::string volumeWeightError;
    std::set<SdfPath> visiting;

    // _CompilePoseConstraints
    std::vector<_FrameConstraint> newFrameConstraints;
    std::map<SdfPath, std::vector<SdfPath>> newFrameChains;
    std::map<SdfPath, int> poseStackOrdinal;
    std::set<SdfPath> newXformDerivedProviders;

    // _CompilePropertyChains
    std::map<SdfPath, std::vector<_PropertyRevision>> newPropertyChains;
    std::vector<SdfPath> newPropertyChainOrder;
    std::vector<RigExecPhasedConnection> newPhasedConnections;

    // _CompilePointGraph
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphChains;

    // _CompileDerivedMaintenance
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphDerivedChains;

    // _CompilePoseDag
    std::map<SdfPath, evaluatorDetail::_PoseInputInfo> newPoseInputInfo;
    std::unordered_map<SdfPath, std::set<SdfPath>, SdfPath::Hash>
        poseClosureCache;
    std::unordered_map<SdfPath, evaluatorDetail::_PoseInputInfo, SdfPath::Hash>
        poseInfoPrefetch;
    std::shared_ptr<evaluatorDetail::_PoseInputGraph> poseInputGraph;
    std::function<bool(const SdfPath &)> isBoundJoint;
    std::map<SdfPath, std::set<SdfPath>> solverFrameInputs;
    std::map<SdfPath, std::set<SdfPath>> solverPoseReads;
    std::map<SdfPath, std::map<SdfPath, std::vector<RigExecReadPhase>>>
        solverClosurePhases;
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> newSolverJoints;
    std::set<SdfPath> requiredSolvers;

    // _CompileRequiredSolvers
    std::map<SdfPath, std::set<SdfPath>> poseDependencies;
    std::map<SdfPath, size_t> constraintIndices;

    // _CompileConstraintDependencies
    std::map<SdfPath, std::vector<SdfPath>> constraintsByTarget;

    // _CompileFrameInheritance
    std::map<SdfPath, std::map<SdfPath, SdfPath>> solverRestPredecessor;

    // _CompilePoseReady
    struct NativePoseOwner {
        bool solverBatch;
        size_t index;
    };
    std::vector<NativePoseOwner> nativePoseOwners;
    std::vector<_SolverBatch> newSolverBatches;
    std::set<SdfPath> newNativeProviderPaths;
    std::set<SdfPath> newRestCompiledRigProviders;
    std::set<SdfPath> restLatePoseInputs;

    // _CompileSpaceSwitches
    std::vector<_SpaceSwitch> newSpaceSwitches;

    // _CompileRestConstancy
    bool newCompiledRigRestsConstant = true;

    // _CompileValidatePhases
    std::map<SdfPath, std::set<SdfPath>> newPhaseCheckpoints;
};

inline SdfPathVector
_CompileRelationshipTargets(const UsdPrim &prim, const TfToken &name)
{
    SdfPathVector paths;
    if (const UsdRelationship rel = prim.GetRelationship(name)) {
        rel.GetTargets(&paths);
    }
    return paths;
}

} // namespace rigExec

#endif
