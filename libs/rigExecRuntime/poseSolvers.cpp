// Runtime aggregate solvers: FK, two-bone IK, spline IK, ribbon, and twist.

#include "rigExecSchema/schemaNames.h"
#include "poseEvalHost.h"
#include "poseInternal.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/chainSolverKernel.h"
#include "rigExecMath/poseCommitKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecStdTypes/stdTypes.h"
#include "rigExecMath/frameBlendKernel.h"
#include "rigExecMath/ribbonFramesKernel.h"
#include "rigExecMath/splineIkKernel.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Solver kernels (rigExecMath/solvers.cpp, splineIk.cpp): Gf -> Rr, the
// arithmetic untouched.

// RigExecSolveFkChain (rigExecMath/chainSolverKernel.h).
std::vector<RrPointFrame>
_RrSolveFkChain(const std::vector<RrPoseFkElement> &elements)
{
    return RigExecSolveFkChainKernel<RrVec3f, RrVec3d, RrPointFrame>(elements);
}

// RigExecSplineIkMakeRest (rigExecMath/splineIkKernel.h).
RrPoseSplineIkRest
_RrSplineIkMakeRest(const std::vector<RrPointFrame> &joints,
                    const RrPointFrame &rootControl,
                    const RrPointFrame &midControl,
                    const RrPointFrame &endControl,
                    const std::vector<double> &volumeWeights,
                    uint8_t restLengthChain)
{
    RrPoseSplineIkRest rest;
    RigExecSplineIkMakeRestKernel<RrVec3d>(joints, rootControl, midControl,
                                           endControl, volumeWeights,
                                           restLengthChain != 0, &rest);
    return rest;
}

// Rebuilds one solver's rest description from the ladder this run
// composed (RefreshSolverRests, bakedPose.cpp).
bool
_RrRefreshSolverRests(RrProgram *program, size_t step, size_t solver,
                      std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSolver &wire = program->poses->solvers[solver];
    RrPoseSolverState &s = scratch->solvers[solver];
    const auto liveRest = [&](size_t k) {
        return k < wire.restIsLive.size() && wire.restIsLive[k] &&
               k < wire.restReads.size() &&
               size_t(wire.restReads[k]) < store.fin.size();
    };
    for (size_t k = 0; k < wire.restRefs.size() && k < s.jointRests.size();
         ++k) {
        const int slot = wire.restRefs[k].first;
        if (liveRest(k)) {
            s.jointRests[k] = store.fin[size_t(wire.restReads[k])].points;
        } else if (slot >= 0 && size_t(slot) < scratch->restPts.size()) {
            s.jointRests[k] = scratch->restPts[size_t(slot)];
        } else {
            if (error) {
                *error = _RrStepHead(program, step) + " names no rest slot";
            }
            return false;
        }
    }
    // The space target's rest, read where the file's space_rest was read:
    // the composed rest table, never a live ref.
    const auto refreshSpaceRest = [&]() {
        if (wire.spaceSlot < 0) {
            return true;
        }
        if (size_t(wire.spaceSlot) >= scratch->restPts.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no rest slot";
            }
            return false;
        }
        s.spaceRest = scratch->restPts[size_t(wire.spaceSlot)];
        return true;
    };
    const std::string type = program->TextOrEmpty(wire.type);
    // A 2D skeleton's controls are its outside parent spaces.
    if (type == SchemaTypeName::RigExecSkeleton2d) {
        for (size_t k = 0; k < wire.controls.size(); ++k) {
            if (k >= s.controlRests.size() || wire.controls[k] < 0 ||
                size_t(wire.controls[k]) >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) + " names no rest slot";
                }
                return false;
            }
            s.controlRests[k] = scratch->restPts[size_t(wire.controls[k])];
        }
        if (wire.start >= 0) {
            if (size_t(wire.start) >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) + " names no rest slot";
                }
                return false;
            }
            s.startRest = scratch->restPts[size_t(wire.start)];
        }
    } else if (type == SchemaTypeName::RigExecSplineIk) {
        std::vector<RrPointFrame> restJoints(size_t(wire.splineCount));
        for (size_t k = 0; k < wire.restRefs.size(); ++k) {
            const int slot = wire.restRefs[k].first;
            const int element = wire.restRefs[k].second;
            if (element >= 0 && size_t(element) < size_t(wire.splineCount)) {
                if (liveRest(k)) {
                    restJoints[size_t(element)] =
                        store.fin[size_t(wire.restReads[k])];
                    if (size_t(element) < s.splineJointRests.size()) {
                        s.splineJointRests[size_t(element)] =
                            store.fin[size_t(wire.restReads[k])].points;
                    }
                } else if (slot >= 0 &&
                           size_t(slot) < scratch->restPts.size() &&
                           size_t(slot) < scratch->restFrames.size()) {
                    restJoints[size_t(element)] =
                        scratch->restFrames[size_t(slot)];
                    if (size_t(element) < s.splineJointRests.size()) {
                        s.splineJointRests[size_t(element)] =
                            scratch->restPts[size_t(slot)];
                    }
                } else {
                    if (error) {
                        *error =
                            _RrStepHead(program, step) + " names no rest slot";
                    }
                    return false;
                }
            }
        }
        const RrPointFrame noFrame;
        s.splineRest = _RrSplineIkMakeRest(
            restJoints,
            wire.root >= 0 ? scratch->restFrames[size_t(wire.root)] : noFrame,
            wire.mid >= 0 ? scratch->restFrames[size_t(wire.mid)] : noFrame,
            wire.end >= 0 ? scratch->restFrames[size_t(wire.end)] : noFrame,
            wire.splineRestWeights, wire.splineRestMode);
        if (!refreshSpaceRest()) {
            return false;
        }
    }
    return true;
}

RrPointFrameArray
_RrSampleRibbonFrames(const std::vector<RrVec3f> &posed,
                      const std::vector<RrVec3f> &rest, int sampleCount,
                      const std::vector<std::array<RrVec3d, 4>> &jointRests,
                      const std::vector<char> &jointRestLive);

struct _RrSplineIkControls {
    RrPointFrame root;
    RrPointFrame mid;
    RrPointFrame end;
};

struct _RrSplineIkParams {
    double preserveVolume = 1.0;
    double midFollowWeight = 0.5;
    double roll = 0.0;
    double twist = 0.0;
    double minLengthRatio = 0.0;
    bool aimRootTangent = false;
};

struct _RrSplineIkJoint {
    RrPointFrame frame;
    RrVec3d scale{1.0, 1.0, 1.0};
    double arcDistance = 0.0;
    double arcParam = 0.0;
    double twist = 0.0;
};

struct _RrSplineIkResult {
    std::array<RrVec3d, 4> cvs{};
    double arcLength = 0.0;
    double ratio = 1.0;
    double roll = 0.0;
    double twist = 0.0;
    std::vector<_RrSplineIkJoint> joints;
};

bool _RrSolveSplineIk(const RrPoseSplineIkRest &rest,
                      const _RrSplineIkControls &controls,
                      const _RrSplineIkParams &params,
                      _RrSplineIkResult *result);

// The joint rest a pose step below the solver wrote for element k
// (solverKernels.cpp); null keeps the solver's own rest.
auto
_RrLiveJointRest(const std::vector<std::array<RrVec3d, 4>> &jointRests,
                 const std::vector<char> &jointRestLive)
{
    return [rests = &jointRests,
            live = &jointRestLive](size_t k) -> const std::array<RrVec3d, 4> * {
        return k < rests->size() && k < live->size() && (*live)[k]
                   ? &(*rests)[k]
                   : nullptr;
    };
}

// RigExecSampleRibbonFrames (rigExecMath/ribbonFramesKernel.h).
RrPointFrameArray
_RrSampleRibbonFrames(const std::vector<RrVec3f> &posed,
                      const std::vector<RrVec3f> &rest, int sampleCount,
                      const std::vector<std::array<RrVec3d, 4>> &jointRests,
                      const std::vector<char> &jointRestLive)
{
    RrPointFrameArray result;
    RigExecSampleRibbonFramesKernel<RrVec3f, RrVec3d>(
        posed.data(), posed.size(), rest.data(), rest.size(), sampleCount,
        &result.frames, &result.rests);
    RigExecRebaseFramesKernel<RrVec3f, RrVec3d>(
        _RrLiveJointRest(jointRests, jointRestLive), &result.frames,
        &result.rests);
    return result;
}

// RigExecSolveSplineIk (rigExecMath/splineIkKernel.h).
bool
_RrSolveSplineIk(const RrPoseSplineIkRest &rest,
                 const _RrSplineIkControls &controls,
                 const _RrSplineIkParams &params, _RrSplineIkResult *result)
{
    return RigExecSolveSplineIkKernel<RrVec3f, RrVec3d, RrPointFrame>(
        rest, controls, params, result);
}

} // namespace

namespace runtimePoseDetail {

bool _RrRunSolveStep(RrProgram *program, size_t step, std::string *error);

bool
_RrRunSolveStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->solvers.size() ||
        size_t(wire.object) >= scratch->solvers.size() ||
        size_t(wire.object) >= store.aggregates.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    const RigExecWireSolver &ws = program->poses->solvers[size_t(wire.object)];
    RrPoseSolverState &s = scratch->solvers[size_t(wire.object)];
    const std::vector<std::array<RrVec3d, 4>> &liveRests = s.jointRests;
    std::vector<char> liveFlags;
    if (ws.hasLiveRest) {
        liveFlags.assign(s.jointRests.size(), 0);
        for (size_t k = 0; k < ws.restIsLive.size() && k < liveFlags.size();
             ++k) {
            liveFlags[k] = ws.restIsLive[k];
        }
    }
    bool restMoved = false;
    for (const int slot : ws.restSlots) {
        restMoved = restMoved ||
                    (slot >= 0 && size_t(slot) < store.restChanged.size() &&
                     store.restChanged[size_t(slot)]);
    }
    if (restMoved || ws.hasLiveRest) {
        if (!_RrRefreshSolverRests(program, step, size_t(wire.object), error)) {
            return false;
        }
    }
    RrPointFrameArray &aggregate = store.aggregates[size_t(wire.object)];
    aggregate.frames.clear();
    aggregate.rests.clear();
    const std::string type = program->TextOrEmpty(ws.type);
    if (size_t(wire.object) >= program->solverJointGroupRead.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    {
        const std::vector<int32_t> &groupReads =
            program->solverJointGroupRead[size_t(wire.object)];
        s.jointGroupSelectors.resize(groupReads.size());
        for (size_t g = 0; g < groupReads.size(); ++g) {
            s.jointGroupSelectors[g] =
                program->ReadRegistered(groupReads[g]).f64;
        }
    }
    const auto finAt = [&](uint32_t version, const RrPointFrame **out) {
        if (size_t(version) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no fin version";
            }
            return false;
        }
        *out = &store.fin[size_t(version)];
        return true;
    };
    if (ws.degenerate) {
    } else if (type == SchemaTypeName::RigExecBlendPointFrames) {
        if ((ws.inA >= 0 && size_t(ws.inA) >= store.aggregates.size()) ||
            (ws.inB >= 0 && size_t(ws.inB) >= store.aggregates.size())) {
            if (error) {
                *error =
                    _RrStepHead(program, step) + " names no solver aggregate";
            }
            return false;
        }
        const RrPointFrameArray *a =
            ws.inA >= 0 ? &store.aggregates[size_t(ws.inA)] : nullptr;
        const RrPointFrameArray *b =
            ws.inB >= 0 ? &store.aggregates[size_t(ws.inB)] : nullptr;
        if (!a) {
            if (b) {
                aggregate = *b;
            }
        } else if (!b) {
            aggregate = *a;
        } else if (ws.blendRotationRejected) {
        } else {
            const size_t n = a->frames.size();
            const double w =
                std::min(std::max(double(program
                                             ->ReadSolver(size_t(wire.object),
                                                          RrSolverBlendWeight)
                                             .f32),
                                  0.0),
                         1.0);
            if (n == b->frames.size() && a->rests.size() == n) {
                aggregate.frames.reserve(n);
                aggregate.rests.reserve(n);
                for (size_t k = 0; k < n; ++k) {
                    const bool live = k < liveFlags.size() && liveFlags[k] &&
                                      k < liveRests.size();
                    aggregate.frames.push_back(_RrBlendFrames(
                        a->frames[k], b->frames[k], a->rests[k], w,
                        ws.scaleMode == 0, live ? &liveRests[k] : nullptr));
                    aggregate.rests.push_back(live ? liveRests[k]
                                                   : a->rests[k]);
                }
            }
        }
    } else if (type == SchemaTypeName::RigExecRibbon) {
        if (size_t(wire.object) >= store.ribbonConstant.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no solver";
            }
            return false;
        }
        aggregate = _RrSampleRibbonFrames(
            store.ribbonConstant[size_t(wire.object)], s.ribbonRestPoints,
            program->ReadSolver(size_t(wire.object), RrSolverRibbonSampleCount)
                .i32,
            liveRests, liveFlags);
    } else if (type == SchemaTypeName::RigExecSkeleton2d) {
        // The native body (RigExecRunSolver's Skeleton2d arm): rests and
        // outputs in rigExec:joints order, the kernel in solve order in the
        // start frame's plane, setup frames for every bone when the solve
        // fails. A rest is the bone's setup frame or the frame a pose step
        // below the skeleton wrote; the outside parent spaces are the
        // row's controls.
        const size_t n = s.bone2dOrder.size();
        const size_t object = size_t(wire.object);
        if (object >= program->solverBone2dRead.size() ||
            object >= program->solverBone2dSelectorRead.size() ||
            s.jointRests.size() != n || s.bone2dSetup.size() != n ||
            ws.controlReads.size() != ws.controls.size() ||
            s.controlRests.size() != ws.controls.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no 2D skeleton";
            }
            return false;
        }
        const std::vector<int32_t> &reads = program->solverBone2dRead[object];
        if (reads.size() != n * kRigExecBone2dChannelCount) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no 2D skeleton";
            }
            return false;
        }
        const std::vector<int32_t> &selectorReads =
            program->solverBone2dSelectorRead[object];
        s.bone2dSelectors.resize(selectorReads.size());
        for (size_t g = 0; g < selectorReads.size(); ++g) {
            s.bone2dSelectors[g] =
                program->ReadRegistered(selectorReads[g]).f64;
        }
        RigExecSkeleton2dLandmarkInputs<std::array<RrVec3d, 4>> frames;
        if (ws.start >= 0) {
            const RrPointFrame *startPose = nullptr;
            if (!finAt(ws.startRead, &startPose)) {
                return false;
            }
            s.bone2dStartPose = startPose->points;
            frames.startRest = &s.startRest;
            frames.startPose = &s.bone2dStartPose;
        }
        s.bone2dExternalPoses.resize(ws.controls.size());
        for (size_t e = 0; e < ws.controls.size(); ++e) {
            const RrPointFrame *posed = nullptr;
            if (!finAt(ws.controlReads[e], &posed)) {
                return false;
            }
            s.bone2dExternalPoses[e] = posed->points;
        }
        frames.externalRests = s.controlRests.data();
        frames.externalPoses = s.bone2dExternalPoses.data();
        frames.externalCount = s.bone2dExternalPoses.size();
        frames.selectors = s.bone2dSelectors.data();
        frames.selectorCount = s.bone2dSelectors.size();
        frames.groupSelectors = s.jointGroupSelectors.data();
        frames.groupSelectorCount = s.jointGroupSelectors.size();
        const RigExecSkeleton2dStatic extra = RigExecSkeleton2dStaticOf(
            s.bone2dSpaceSources, s.bone2dTargetCandidates, s.jointGroups,
            s.jointGroupMembers);
        const RigExecSkeleton2dResult result = RigExecSolveSkeleton2dJoints(
            s.bone2dOrder, s.jointRests, s.bone2dSetup, extra, frames,
            [&](size_t j, RigExecBone2dChannels *channels) {
                for (size_t c = 0; c < kRigExecBone2dChannelCount; ++c) {
                    const RrInputValue value = program->ReadRegistered(
                        reads[j * kRigExecBone2dChannelCount + c]);
                    RigExecBone2dSetChannel(channels, c,
                                            value.tag == RrInputTag::Bool
                                                ? (value.boolean ? 1.0 : 0.0)
                                                : value.f64);
                }
            },
            RrPointFrameValid, &s.bone2d, &aggregate.frames);
        if (result == RigExecSkeleton2dResult::Malformed) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no 2D bone";
            }
            return false;
        }
        aggregate.rests = s.jointRests;
    } else if (type == SchemaTypeName::RigExecSplineIk) {
        _RrSplineIkParams params;
        params.preserveVolume = ws.splineParams.preserveVolume;
        params.midFollowWeight = ws.splineParams.midFollowWeight;
        params.roll = ws.splineParams.roll;
        params.twist = ws.splineParams.twist;
        params.minLengthRatio = ws.splineParams.minLengthRatio;
        params.aimRootTangent = ws.splineParams.aimRootTangent;
        if (ws.splineParamsVary ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverPreserveVolume) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverMidFollowWeight) ||
            _RrLiveSolver(program, size_t(wire.object), RrSolverRoll) ||
            _RrLiveSolver(program, size_t(wire.object), RrSolverTwist) ||
            _RrLiveSolver(program, size_t(wire.object),
                          RrSolverMinLengthRatio)) {
            params.preserveVolume =
                program->ReadSolver(size_t(wire.object), RrSolverPreserveVolume)
                    .f64;
            params.midFollowWeight =
                program
                    ->ReadSolver(size_t(wire.object), RrSolverMidFollowWeight)
                    .f64;
            params.roll = RrDegreesToRadians(
                program->ReadSolver(size_t(wire.object), RrSolverRoll).f64);
            params.twist = RrDegreesToRadians(
                program->ReadSolver(size_t(wire.object), RrSolverTwist).f64);
            params.minLengthRatio =
                program->ReadSolver(size_t(wire.object), RrSolverMinLengthRatio)
                    .f64;
        }
        // The rest rebuilt in rigExec:space once the space has moved
        // (bakedPose.cpp): the bind-time rest is measured at identity and
        // the placement ratio is arcLength / restArcLength, so a scaled
        // space would otherwise read as stretch. s.splineRest holds the
        // rest frames and the root, mid and end rests it was made from.
        const RrPoseSplineIkRest *restForSolve = &s.splineRest;
        RrPoseSplineIkRest spacedRest;
        if (ws.spaceSlot >= 0) {
            const RrPointFrame *spaceFrame = nullptr;
            if (!finAt(ws.spaceRead, &spaceFrame)) {
                return false;
            }
            RrMat4d delta(1.0);
            if (RrPointsToMatrix(s.spaceRest, *spaceFrame, &delta) &&
                delta != RrMat4d(1.0)) {
                std::vector<RrPointFrame> spaced;
                spaced.reserve(s.splineRest.joints.size());
                for (const RrPointFrame &f : s.splineRest.joints) {
                    spaced.push_back(_RrTransformFrame(f, delta));
                }
                spacedRest = _RrSplineIkMakeRest(
                    spaced, _RrTransformFrame(s.splineRest.rootControl, delta),
                    _RrTransformFrame(s.splineRest.midControl, delta),
                    _RrTransformFrame(s.splineRest.endControl, delta),
                    ws.splineRestWeights, ws.splineRestMode);
                restForSolve = &spacedRest;
            }
        }
        const RrPointFrame *root = nullptr;
        const RrPointFrame *mid = nullptr;
        const RrPointFrame *end = nullptr;
        if (!finAt(ws.rootRead, &root) || !finAt(ws.midRead, &mid) ||
            !finAt(ws.endRead, &end)) {
            return false;
        }
        _RrSplineIkControls controls;
        controls.root = *root;
        controls.mid = *mid;
        controls.end = *end;
        _RrSplineIkResult solved;
        _RrSolveSplineIk(*restForSolve, controls, params, &solved);
        if (solved.joints.size() == size_t(ws.splineCount)) {
            aggregate.frames.reserve(size_t(ws.splineCount));
            for (const auto &joint : solved.joints) {
                aggregate.frames.push_back(joint.frame);
            }
            aggregate.rests = s.splineJointRests;
        }
    }
    if (size_t(wire.object) >= store.solverOutFrames.size() ||
        size_t(wire.object) >= store.solverOutPresent.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    if (store.solverOutFrames[size_t(wire.object)].size() !=
            ws.outputs.size() ||
        store.solverOutPresent[size_t(wire.object)].size() !=
            ws.outputs.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no solver";
        }
        return false;
    }
    for (size_t k = 0; k < ws.outputs.size(); ++k) {
        const int32_t slot = ws.outputs[k].first;
        const int32_t element = ws.outputs[k].second;
        if (element < 0 || size_t(element) >= aggregate.frames.size()) {
            store.solverOutPresent[size_t(wire.object)][k] = 0;
            (void)slot;
            continue;
        }
        store.solverOutFrames[size_t(wire.object)][k] =
            RrExtractElementFrame(&aggregate, size_t(element));
        store.solverOutPresent[size_t(wire.object)][k] = 1;
    }
    return true;
}

} // namespace runtimePoseDetail

namespace {

bool
_RrSolverFail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// Whether \p type holds one \p T, or with \p array an array of them.
template <class T>
bool
_RrSolverType(const RigExecTypeDesc &type, bool array)
{
    if (!array)
        return !type.element && type.size == sizeof(T);
    return type.element && type.resize && type.data &&
           RigExecRegistry::Get().TypeAt(type.element->Index()).desc->size ==
               sizeof(T);
}

/// \p v into \p value, a scalar of \p type.
template <class T>
bool
_RrSolverScalar(const RigExecTypeDesc &type, T v, void *value,
                std::string *error)
{
    if (type.element || type.size != sizeof(T))
        return _RrSolverFail(error, "a solver scalar of another type");
    std::memcpy(value, &v, sizeof(T));
    return true;
}

rigExecRig::PointLandmarks
_RrSolverLandmarks(const std::array<RrVec3d, 4> &points)
{
    rigExecRig::PointLandmarks out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = rigExecStd::Vec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

rigExecRig::PointFrame
_RrSolverFrame(const RrPointFrame &frame)
{
    rigExecRig::PointFrame out;
    out.points = _RrSolverLandmarks(frame.points);
    out.flags = frame.flags;
    return out;
}

std::array<RrVec3d, 4>
_RrFromSolverLandmarks(const rigExecRig::PointLandmarks &points)
{
    std::array<RrVec3d, 4> out;
    for (size_t i = 0; i < 4; ++i)
        out[i] = RrVec3d(points[i][0], points[i][1], points[i][2]);
    return out;
}

} // namespace

int64_t
RrSolveOf(const RrProgram *program, const RigExecWireStep &step)
{
    if (step.kind == RigExecWireStepKind::Solve)
        return step.object;
    return step.kind == RigExecWireStepKind::OpNode && step.object >= 0 &&
                   size_t(step.object) < program->solverOfNode.size()
               ? program->solverOfNode[size_t(step.object)]
               : -1;
}

bool
RrLookUpSolver(const RrProgram *program, const eval::OpLookup &lookup,
               const RigExecTypeDesc &type, void *value, std::string *error)
{
    using Read = eval::OpSolverRead;
    const RrProgram &p = *program;
    if (!p.poses || lookup.object >= p.poses->solvers.size())
        return _RrSolverFail(error, "solver " + std::to_string(lookup.object) +
                                        " is no solver of the program");
    const RigExecWireSolver &s = p.poses->solvers[lookup.object];
    const RrPoseScratch &scratch =
        *static_cast<const RrPoseScratch *>(p.pose.get());
    const std::vector<std::array<RrVec3d, 4>> &restPts = scratch.restPts;
    const std::vector<RrPointFrame> &fin = p.store.fin;
    const size_t base = s.start >= 0 ? 1 : 0, n = s.controls.size();
    const auto rest = [&](int slot, rigExecRig::PointLandmarks *out) {
        if (slot < 0 || size_t(slot) >= restPts.size())
            return _RrSolverFail(
                error, "solver " + std::to_string(lookup.object) +
                           " names no rest slot " + std::to_string(slot));
        *out = _RrSolverLandmarks(restPts[size_t(slot)]);
        return true;
    };
    const auto frame = [&](uint32_t version, rigExecRig::PointFrame *out) {
        if (version >= fin.size())
            return _RrSolverFail(error, "pose version " +
                                            std::to_string(version) +
                                            " is no pose frame of the "
                                            "program");
        *out = _RrSolverFrame(fin[version]);
        return true;
    };
    switch (Read(lookup.part)) {
    case Read::FkControls: {
        if (!_RrSolverType<rigExecRig::PointFrameArray>(type, false) ||
            s.controlReads.size() != n)
            break;
        auto &out = *static_cast<rigExecRig::PointFrameArray *>(value);
        out.frames.Resize(n + base);
        out.rests.Resize(n + base);
        if (base && (!frame(s.startRead, &out.frames[0]) ||
                     !rest(s.start, &out.rests[0])))
            return false;
        for (size_t k = 0; k < n; ++k)
            if (!frame(s.controlReads[k], &out.frames[k + base]) ||
                !rest(s.controls[k], &out.rests[k + base]))
                return false;
        return true;
    }
    case Read::FkOutRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true))
            break;
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(n + base);
        if (base && !rest(s.start, &out[0]))
            return false;
        const bool jointBasis = s.jointRests.size() == n;
        for (size_t k = 0; k < n; ++k) {
            if (!jointBasis || k >= s.restIsLive.size() || !s.restIsLive[k]) {
                if (!rest(s.controls[k], &out[k + base]))
                    return false;
                continue;
            }
            if (k >= s.restReads.size() || s.restReads[k] >= fin.size())
                return _RrSolverFail(
                    error, "solver " + std::to_string(lookup.object) +
                               " names no live rest " + std::to_string(k));
            out[k + base] = _RrSolverLandmarks(fin[s.restReads[k]].points);
        }
        return true;
    }
    case Read::StartRest:
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, false))
            break;
        return rest(s.start, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::IkRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true) ||
            lookup.object >= scratch.solvers.size())
            break;
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(3);
        for (size_t e = 0; e < 3; ++e)
            out[e] =
                _RrSolverLandmarks(scratch.solvers[lookup.object].ikRests[e]);
        for (size_t k = 0; k < s.restRefs.size(); ++k) {
            const int slot = s.restRefs[k].first;
            const int element = s.restRefs[k].second;
            if (element < 0 || element >= 3)
                continue;
            if (k < s.restIsLive.size() && s.restIsLive[k]) {
                if (k >= s.restReads.size() || s.restReads[k] >= fin.size())
                    return _RrSolverFail(
                        error, "solver " + std::to_string(lookup.object) +
                                   " names no live rest " + std::to_string(k));
                out[size_t(element)] =
                    _RrSolverLandmarks(fin[s.restReads[k]].points);
            } else if (!rest(slot, &out[size_t(element)])) {
                return false;
            }
        }
        return true;
    }
    case Read::IkSpace: {
        if (!_RrSolverType<RrMat4d>(type, false))
            break;
        RrMat4d space = p.ReadSolver(lookup.object, RrSolverIkSpace).matrix;
        if (s.spaceSlot >= 0) {
            if (size_t(s.spaceSlot) >= restPts.size() ||
                size_t(s.spaceRead) >= fin.size())
                return _RrSolverFail(error, "solver " +
                                                std::to_string(lookup.object) +
                                                " names no space frame");
            RrMat4d delta(1.0);
            if (RrPointsToMatrix(restPts[size_t(s.spaceSlot)],
                                 fin[size_t(s.spaceRead)], &delta))
                space = delta * space;
        }
        std::memcpy(value, &space, sizeof(RrMat4d));
        return true;
    }
    case Read::Bend:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverBend).f64, value, error);
    case Read::UpperOffset:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverUpperOffset).f64, value,
            error);
    case Read::LowerOffset:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverLowerOffset).f64, value,
            error);
    case Read::Stretch:
        return _RrSolverScalar(type,
                               p.ReadSolver(lookup.object, RrSolverStretch).f32,
                               value, error);
    case Read::Softness:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverSoftness).f32, value,
            error);
    case Read::Pin:
        return _RrSolverScalar(
            type, s.pin ? p.ReadSolver(lookup.object, RrSolverPin).f32 : 0.0f,
            value, error);
    case Read::UpperScale:
        return _RrSolverScalar(
            type,
            s.upperScale ? p.ReadSolver(lookup.object, RrSolverUpperScale).f64
                         : 1.0,
            value, error);
    case Read::LowerScale:
        return _RrSolverScalar(
            type,
            s.lowerScale ? p.ReadSolver(lookup.object, RrSolverLowerScale).f64
                         : 1.0,
            value, error);
    case Read::SoftDistance:
        return _RrSolverScalar(
            type,
            s.softDistance
                ? p.ReadSolver(lookup.object, RrSolverSoftDistance).f32
                : 0.0f,
            value, error);
    case Read::Twist:
        return _RrSolverScalar(
            type,
            s.limbTwist ? p.ReadSolver(lookup.object, RrSolverLimbTwist).f32
                        : 0.0f,
            value, error);
    case Read::JointRests: {
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, true) ||
            lookup.object >= scratch.solvers.size())
            break;
        const RrPoseSolverState &state = scratch.solvers[lookup.object];
        auto &out = *static_cast<
            rigExecStd::RigExecArray<rigExecRig::PointLandmarks> *>(value);
        out.Resize(state.jointRests.size());
        for (size_t k = 0; k < state.jointRests.size(); ++k) {
            if (k < s.restIsLive.size() && s.restIsLive[k] &&
                k < s.restReads.size() && s.restReads[k] < fin.size())
                out[k] = _RrSolverLandmarks(fin[s.restReads[k]].points);
            else if (k >= s.restRefs.size())
                out[k] = _RrSolverLandmarks(state.jointRests[k]);
            else if (!rest(s.restRefs[k].first, &out[k]))
                return false;
        }
        return true;
    }
    case Read::JointRestLive: {
        if (!_RrSolverType<uint8_t>(type, true) ||
            lookup.object >= scratch.solvers.size())
            break;
        const size_t joints = scratch.solvers[lookup.object].jointRests.size();
        auto &out = *static_cast<rigExecStd::RigExecArray<uint8_t> *>(value);
        out.Resize(joints);
        for (size_t k = 0; k < joints; ++k)
            out[k] = k < s.restIsLive.size() && s.restIsLive[k] ? 1 : 0;
        return true;
    }
    case Read::TwistStartRest:
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, false))
            break;
        return rest(s.root, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::TwistEndRest:
        if (!_RrSolverType<rigExecRig::PointLandmarks>(type, false))
            break;
        return rest(s.end, static_cast<rigExecRig::PointLandmarks *>(value));
    case Read::TwistTurns:
        return _RrSolverScalar(
            type, p.ReadSolver(lookup.object, RrSolverTwistTurns).f64, value,
            error);
    case Read::GroupSelectors: {
        if (!_RrSolverType<double>(type, true) ||
            lookup.object >= p.solverJointGroupRead.size())
            break;
        const std::vector<int32_t> &reads =
            p.solverJointGroupRead[lookup.object];
        auto &out = *static_cast<rigExecStd::RigExecArray<double> *>(value);
        out.Resize(reads.size());
        for (size_t g = 0; g < reads.size(); ++g)
            out[g] = p.ReadRegistered(reads[g]).f64;
        return true;
    }
    }
    return _RrSolverFail(
        error, "solver " + std::to_string(lookup.object) + " has no input " +
                   std::to_string(lookup.part) + " of that type");
}

bool
RrStoreSolver(RrProgram *program, const eval::OpHostOutput &output,
              const void *value, std::string *error)
{
    RrProgram &p = *program;
    RrStore &store = p.store;
    if (!p.poses || output.object >= p.poses->solvers.size() ||
        output.object >= store.aggregates.size() ||
        output.object >= store.solverOutFrames.size() ||
        output.object >= store.solverOutPresent.size())
        return _RrSolverFail(error, "solver " + std::to_string(output.object) +
                                        " is no solver of the program");
    const RigExecWireSolver &s = p.poses->solvers[output.object];
    const auto &out = *static_cast<const rigExecRig::PointFrameArray *>(value);
    const size_t first =
        p.opNodes && output.object < p.opNodes->solverLeading.size()
            ? p.opNodes->solverLeading[output.object]
            : 0;
    RrPointFrameArray &aggregate = store.aggregates[output.object];
    const size_t frames =
        out.frames.Size() > first ? out.frames.Size() - first : 0;
    const size_t rests =
        out.rests.Size() > first ? out.rests.Size() - first : 0;
    aggregate.frames.resize(frames);
    aggregate.rests.resize(rests);
    for (size_t k = 0; k < frames; ++k) {
        aggregate.frames[k].points =
            _RrFromSolverLandmarks(out.frames[k + first].points);
        aggregate.frames[k].flags = out.frames[k + first].flags;
    }
    for (size_t k = 0; k < rests; ++k)
        aggregate.rests[k] = _RrFromSolverLandmarks(out.rests[k + first]);
    std::vector<RrPointFrame> &outFrames = store.solverOutFrames[output.object];
    std::vector<char> &outPresent = store.solverOutPresent[output.object];
    if (outFrames.size() != s.outputs.size() ||
        outPresent.size() != s.outputs.size())
        return _RrSolverFail(error, "solver " + std::to_string(output.object) +
                                        " publishes no outputs");
    for (size_t k = 0; k < s.outputs.size(); ++k) {
        const int32_t element = s.outputs[k].second;
        if (element < 0 || size_t(element) >= aggregate.frames.size()) {
            outPresent[k] = 0;
            continue;
        }
        outFrames[k] = RrExtractElementFrame(&aggregate, size_t(element));
        outPresent[k] = 1;
    }
    return true;
}

std::string
RrSolverFailure(const RrProgram *program, uint32_t solver,
                const std::string &error)
{
    const RrProgram &p = *program;
    if (!p.poses || solver >= p.poses->solvers.size())
        return error;
    return p.TextOrEmpty(p.poses->solvers[solver].path) + ": " + error;
}

namespace {

/// The versions list \p part (OpCommitList) of \p commit names, and
/// whether they are base versions; null for no list.
const std::vector<uint32_t> *
_RrCommitVersions(const RigExecWireCommit &commit, int32_t part, bool *base)
{
    using L = eval::OpCommitList;
    *base = part == int32_t(L::BaseCarry) ||
            part == int32_t(L::DescendantBaseCarry);
    switch (L(part)) {
    case L::Entering:
        return &commit.slotReads;
    case L::Carry:
        return &commit.slotCarry;
    case L::BaseCarry:
        return &commit.slotBaseCarry;
    case L::Current:
        return &commit.descendantReads;
    case L::Closest:
        return &commit.closestReads;
    case L::DescendantCarry:
        return &commit.descendantCarry;
    case L::DescendantBaseCarry:
        return &commit.descendantBaseCarry;
    }
    return nullptr;
}

bool
_RrCommitFail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// Resizes the array \p value of \p type to \p count elements of
/// \p size bytes, its first in \p data; false when \p type holds no such
/// array.
template <class T>
bool
_RrCommitElements(const RigExecTypeDesc &type, void *value, size_t count,
                  T **data)
{
    if (!type.element || !type.resize || !type.data ||
        RigExecRegistry::Get().TypeAt(type.element->Index()).desc->size !=
            sizeof(T))
        return false;
    type.resize(value, count);
    *data = static_cast<T *>(type.data(value));
    return *data || !count;
}

/// Calls \p visit with each candidate position solver batch \p w's
/// solvers published and its frame, the last in batch order winning.
template <class Visit>
void
_RrEachSolverCandidate(const RrProgram &p, size_t w, Visit visit)
{
    for (const int si : p.poses->walkSteps[w].batchSolvers) {
        if (si < 0 || size_t(si) >= p.poses->solvers.size() ||
            size_t(si) >= p.store.solverOutFrames.size() ||
            size_t(si) >= p.store.solverOutPresent.size())
            continue;
        const RigExecWireSolver &s = p.poses->solvers[size_t(si)];
        const auto &frames = p.store.solverOutFrames[size_t(si)];
        const auto &present = p.store.solverOutPresent[size_t(si)];
        for (size_t k = 0; k < s.outPosition.size() && k < frames.size() &&
                           k < present.size();
             ++k)
            if (present[k] && s.outPosition[k] >= 0)
                visit(size_t(s.outPosition[k]), frames[k]);
    }
}

/// Whether candidate \p pos of commit \p w is present this frame, and its
/// frame in \p frame when it is.
bool
_RrCandidateAt(const RrProgram &p, size_t w, size_t pos, RrPointFrame *frame)
{
    const RigExecWireCommit &commit = p.poses->commits[w];
    const RrCommitScratch &scratch = p.store.commits[w];
    if (!commit.solverOutput) {
        if (pos < scratch.frames.size())
            *frame = scratch.frames[pos];
        return pos < scratch.present.size() && scratch.present[pos];
    }
    bool present = false;
    _RrEachSolverCandidate(p, w, [&](size_t at, const RrPointFrame &f) {
        if (at == pos) {
            *frame = f;
            present = true;
        }
    });
    return present;
}

/// The candidate position whose write descendant \p k of \p commit
/// carries, or -1 when it carries a version written before the commit.
int
_RrCarriedCandidate(const RigExecWireCommit &commit, size_t k, bool base)
{
    const std::vector<uint32_t> &writes =
        base ? commit.slotBaseWrites : commit.slotWrites;
    const std::vector<uint32_t> &carry =
        base ? commit.descendantBaseCarry : commit.descendantCarry;
    for (size_t pos = 0; pos < commit.slots.size() && pos < writes.size();
         ++pos)
        if (commit.slots[pos] == commit.propagate[k].first &&
            writes[pos] == carry[k])
            return int(pos);
    return -1;
}

} // namespace

bool
RrLookUpCommit(const RrProgram *program, const eval::OpLookup &lookup,
               const RigExecTypeDesc &type, void *value, std::string *error)
{
    const RrProgram &p = *program;
    if (lookup.object >= p.poses->commits.size() ||
        lookup.object >= p.store.commits.size())
        return _RrCommitFail(error, "commit " + std::to_string(lookup.object) +
                                        " is no commit of the program");
    const RigExecWireCommit &commit = p.poses->commits[lookup.object];
    const RrCommitScratch &scratch = p.store.commits[lookup.object];
    const size_t slots = commit.slots.size();
    if (lookup.source == uint8_t(eval::OpLookupSource::CommitVersions)) {
        bool base = false;
        const std::vector<uint32_t> *versions =
            _RrCommitVersions(commit, lookup.part, &base);
        const std::vector<RrPointFrame> &store =
            base ? p.store.base : p.store.fin;
        RrPointFrame *out = nullptr;
        if (!versions ||
            !_RrCommitElements(type, value, versions->size(), &out))
            return _RrCommitFail(error, "commit " +
                                            std::to_string(lookup.object) +
                                            " names no frame list " +
                                            std::to_string(lookup.part));
        const bool descendants =
            lookup.part == int32_t(eval::OpCommitList::DescendantCarry) ||
            lookup.part == int32_t(eval::OpCommitList::DescendantBaseCarry);
        for (size_t i = 0; i < versions->size(); ++i) {
            const int pos =
                descendants ? _RrCarriedCandidate(commit, i, base) : -1;
            if (pos >= 0 &&
                _RrCandidateAt(p, lookup.object, size_t(pos), &out[i]))
                continue;
            const uint32_t version =
                pos >= 0 ? (base ? commit.slotBaseCarry
                                 : commit.slotCarry)[size_t(pos)]
                         : (*versions)[i];
            if (version >= store.size())
                return _RrCommitFail(error, "pose version " +
                                                std::to_string(version) +
                                                " is no pose frame of the "
                                                "program");
            out[i] = store[version];
        }
        return true;
    }
    if (lookup.part == 0) {
        RrPointFrame *out = nullptr;
        if (!_RrCommitElements(type, value, slots, &out) ||
            (!commit.solverOutput && scratch.frames.size() < slots))
            return _RrCommitFail(error, "the candidates of commit " +
                                            std::to_string(lookup.object) +
                                            " are no frame list");
        if (!commit.solverOutput) {
            for (size_t pos = 0; pos < slots; ++pos)
                out[pos] = scratch.frames[pos];
            return true;
        }
        for (size_t pos = 0; pos < slots; ++pos)
            out[pos] = RrPointFrame();
        _RrEachSolverCandidate(p, lookup.object,
                               [&](size_t pos, const RrPointFrame &frame) {
                                   if (pos < slots)
                                       out[pos] = frame;
                               });
        return true;
    }
    uint8_t *out = nullptr;
    if (lookup.part != 1 || !_RrCommitElements(type, value, slots, &out) ||
        (!commit.solverOutput && scratch.present.size() < slots))
        return _RrCommitFail(error, "commit " + std::to_string(lookup.object) +
                                        " names no candidate list " +
                                        std::to_string(lookup.part));
    for (size_t pos = 0; pos < slots; ++pos)
        out[pos] = commit.solverOutput ? 0 : uint8_t(scratch.present[pos] != 0);
    if (commit.solverOutput)
        _RrEachSolverCandidate(p, lookup.object,
                               [&](size_t pos, const RrPointFrame &) {
                                   if (pos < slots)
                                       out[pos] = 1;
                               });
    return true;
}

bool
RrStoreCommit(RrProgram *program, const eval::OpHostOutput &output,
              const void *value, std::string *error)
{
    RrProgram &p = *program;
    if (output.object >= p.poses->commits.size() ||
        output.object >= p.store.commits.size())
        return _RrCommitFail(error, "commit " + std::to_string(output.object) +
                                        " is no commit of the program");
    const RigExecWireCommit &commit = p.poses->commits[output.object];
    RrCommitScratch &scratch = p.store.commits[output.object];
    using T = eval::OpOutputTarget;
    const auto &array =
        *static_cast<const rigExecStd::RigExecArray<uint8_t> *>(value);
    if (output.target == uint8_t(T::CommitPresent)) {
        if (array.Size() != scratch.present.size())
            return _RrCommitFail(
                error, "commit " + std::to_string(output.object) + " has " +
                           std::to_string(scratch.present.size()) +
                           " candidates, not " + std::to_string(array.Size()));
        bool any = false;
        for (size_t pos = 0; pos < array.Size(); ++pos) {
            scratch.present[pos] = array[pos] ? 1 : 0;
            any = any || array[pos];
        }
        scratch.abandoned = !any;
        return true;
    }
    const bool base = output.target == uint8_t(T::CommitBaseSlots) ||
                      output.target == uint8_t(T::CommitBaseDescendants);
    const std::vector<uint32_t> *versions =
        output.target == uint8_t(T::CommitSlots) ? &commit.slotWrites
        : output.target == uint8_t(T::CommitDescendants)
            ? &commit.descendantWrites
        : output.target == uint8_t(T::CommitBaseSlots) ? &commit.slotBaseWrites
        : output.target == uint8_t(T::CommitBaseDescendants)
            ? &commit.descendantBaseWrites
            : nullptr;
    std::vector<RrPointFrame> &store = base ? p.store.base : p.store.fin;
    const auto &frames =
        *static_cast<const rigExecStd::RigExecArray<RrPointFrame> *>(value);
    if (!versions || frames.Size() != versions->size())
        return _RrCommitFail(
            error, "commit " + std::to_string(output.object) + " writes no " +
                       std::to_string(frames.Size()) + " frames at target " +
                       std::to_string(output.target));
    for (size_t i = 0; i < frames.Size(); ++i) {
        if ((*versions)[i] >= store.size())
            return _RrCommitFail(error, "pose version " +
                                            std::to_string((*versions)[i]) +
                                            " is no pose frame of the program");
        store[(*versions)[i]] = frames[i];
    }
    return true;
}

std::string
RrCommitFailure(const RrProgram *program, uint32_t commit,
                const std::string &error)
{
    const RrProgram &p = *program;
    if (commit >= p.poses->commits.size())
        return error;
    const RigExecWireCommit &c = p.poses->commits[commit];
    std::string mover = p.TextOrEmpty(c.moverPath);
    if (mover.empty() && commit < p.poses->walkSteps.size()) {
        const auto &batch = p.poses->walkSteps[commit].batchSolvers;
        if (!batch.empty() && batch.front() >= 0 &&
            size_t(batch.front()) < p.poses->solvers.size())
            mover = p.TextOrEmpty(p.poses->solvers[size_t(batch.front())].path);
    }
    const auto &meta = *p.slotMeta;
    return RigExecCommitFailureText(mover, error, [&](size_t k) {
        return k < c.propagate.size() &&
                       size_t(c.propagate[k].first) < meta.paths.size()
                   ? p.TextOrEmpty(meta.paths[size_t(c.propagate[k].first)])
                   : "descendant " + std::to_string(k);
    });
}

} // namespace rigExec
