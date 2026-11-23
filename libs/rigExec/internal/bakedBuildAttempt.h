// Private state of one baked program build, shared by its steps.
#ifndef RIGEXEC_BAKED_BUILD_ATTEMPT_H
#define RIGEXEC_BAKED_BUILD_ATTEMPT_H

#include "rigExec/api/bakedProgram.h"
#include "rigExec/api/profiler.h"
#include "rigExec/internal/bakedProgramImpl.h"

#include "pxr/base/work/dispatcher.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace rigExec {

/// Everything the steps of RigExecBakedProgram::_BuildWithSceneCaptureAttempt
/// hand to one another. Members are declared in the order the steps fill
/// them, so they are destroyed in the reverse of that order.
struct RigExecBakedProgram::_BuildAttempt {
    _BuildAttempt(RigExecRigEvaluator &E, RigExecProfiler *profiler,
                  std::vector<std::string> *reasons,
                  const RigExecSceneDescriptors *scene,
                  const UsdStageWeakPtr &sceneStage, uint64_t sceneSerial,
                  const std::set<SdfPath> &excludedPoseWriters,
                  const std::set<SdfPath> &excludedSolverBodies,
                  const RigExecOpExclusionProof &exclusionProof,
                  const std::map<SdfPath, std::string> &cycleSkipReasons);

    _BuildAttempt(const _BuildAttempt &) = delete;
    _BuildAttempt &operator=(const _BuildAttempt &) = delete;

    RigExecRigEvaluator &E;
    RigExecRigEvaluator *evaluator;
    std::vector<std::string> *reasons;
    const RigExecSceneDescriptors *scene;
    const UsdStageWeakPtr &sceneStage;
    uint64_t sceneSerial;
    const std::set<SdfPath> &excludedPoseWriters;
    const std::set<SdfPath> &excludedSolverBodies;
    const RigExecOpExclusionProof &exclusionProof;
    const std::map<SdfPath, std::string> &cycleSkipReasons;

    // _BuildCheckBakeable
    std::map<SdfPath, std::set<SdfPath>> nativeCheckpoints;
    // Bound here, ahead of IsBakeable, so the profiler exists before the
    // refusal walk runs. A rig that refuses still costs whatever
    // IsBakeable spends deciding why, and with `phases` opened only after
    // that call the cost had nowhere to attribute to -- it showed up as a
    // gap between Compile.Bake opening and the first Bake.* mark. Marking
    // it here costs the one branch per mark every other phase already pays.
    RigExecProfilePhases phases;
    bool bakeableBeside = false;
    bool bakeable = true;
    std::vector<std::string> bakeableReasons;
    std::vector<std::string> buildReasons;
    // Declared after everything the task writes, so an exit from anywhere
    // below destroys the dispatcher, which joins it, before those die.
    WorkDispatcher bakeableLane;

    // _BuildCreateProgram
    std::unique_ptr<RigExecBakedProgramImpl> impl;

    // _BuildEntry
    RigExecBakedBuildContext ctx;
    UsdTimeCode capture;

    // _BuildProviderSlots
    int N = 0;

    // _BuildLadder
    RigExecProfilePhases bindPhases;
    size_t commitChunkSlots = 8;
    size_t commitChunks = 0;

    // _BuildPoseDescriptors
    std::vector<RigExecBakedWalkEntry> walk;
};

} // namespace rigExec

#endif
