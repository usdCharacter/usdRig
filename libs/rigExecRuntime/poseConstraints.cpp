// Runtime constraints, source resolution, and single-chain IK.

#include "rigExecSchema/schemaNames.h"
#include "poseEvalHost.h"
#include "rigExecEvalCore/constraintLookups.h"
#include "poseInternal.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecMath/constraintKernel.h"
#include "rigExecMath/singleChainIkKernel.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Single-chain IK (singleChainIk.cpp).

enum class _RrSingleChainIkMode : uint8_t {
    RotatePlane = 0,
    SingleChain = 1,
};

struct _RrSingleChainIkParams {
    _RrSingleChainIkMode mode = _RrSingleChainIkMode::RotatePlane;
    RrVec3d pole{0.0, 1.0, 0.0};
    double twistDegrees = 0.0;
    double stretch = 0;
    double weight = 1.0;
    bool preserveJointOrientation = false;
};

// RigExecSolveSingleChainIk (rigExecMath/singleChainIkKernel.h).
std::vector<RrPointFrame>
_RrSolveSingleChainIk(const std::vector<RrPointFrame> &currentFrames,
                      const RrPointFrame &effectorFrame,
                      const _RrSingleChainIkParams &params)
{
    RigExecSingleChainIkKernelParams<RrVec3d> p;
    p.mode = int(params.mode);
    p.pole = params.pole;
    p.twistDegrees = params.twistDegrees;
    p.weight = params.weight;
    p.stretch = params.stretch;
    p.preserveJointOrientation = params.preserveJointOrientation;
    return RigExecSolveSingleChainIkKernel<RrVec3f, RrVec3d>(currentFrames,
                                                             effectorFrame, p);
}

// RigExecPrepareRestDerivedIkChain (rigEvaluator.cpp): rest segment
// lengths laid along the current chain's directions.
bool
_RrPrepareRestDerivedIkChain(const std::vector<RrPointFrame> &current,
                             const std::vector<RrPointFrame> &rest,
                             std::vector<RrPointFrame> *prepared)
{
    if (current.size() != rest.size() || current.empty()) {
        return false;
    }
    bool usableRestLayout = true;
    for (size_t i = 1; i < rest.size(); ++i) {
        const double segmentLength =
            (rest[i].points[0] - rest[i - 1].points[0]).GetLength();
        if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
            usableRestLayout = false;
            break;
        }
    }
    const std::vector<RrPointFrame> &lengthReference =
        usableRestLayout ? rest : current;
    prepared->clear();
    prepared->reserve(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        if (!RrFrameUsable(current[i]) || !RrFrameUsable(rest[i])) {
            return false;
        }
        RrVec3d origin = current[i].points[0];
        if (i > 0) {
            RrMat4d parentRest = _RrIdentity();
            RrMat4d parentPrepared = _RrIdentity();
            if (!RrPointsToMatrix(RrIdentityLandmarks(),
                                  lengthReference[i - 1].points, &parentRest) ||
                !RrPointsToMatrix(RrIdentityLandmarks(),
                                  prepared->back().points, &parentPrepared)) {
                return false;
            }
            origin = parentPrepared.TransformAffine(
                parentRest.GetInverse().TransformAffine(
                    lengthReference[i].points[0]));
        }

        RrPointFrame frame = current[i];
        frame.points[0] = origin;
        for (size_t axis = 1; axis < frame.points.size(); ++axis) {
            RrVec3d direction = current[i].points[axis] - current[i].points[0];
            const double directionLength = direction.GetLength();
            const double length =
                (lengthReference[i].points[axis] - lengthReference[i].points[0])
                    .GetLength();
            if (!std::isfinite(length) || length <= 0.0 ||
                !std::isfinite(directionLength) || directionLength <= 0.0) {
                return false;
            }
            direction /= directionLength;
            frame.points[axis] = origin + direction * length;
        }
        if (!RrFrameUsable(frame)) {
            return false;
        }
        prepared->push_back(frame);
    }
    return true;
}

// The Constraint step (bakedPose.cpp).

} // namespace

// RrPoseState::solveIkChain (poseEvalHost.h): the single-chain IK
// core, called from pose.h with gathered frames. The core itself
// stays here until the Solve lift unifies it.
bool
RrPoseState::solveIkChain(size_t object, const std::vector<RrPointFrame> &chain,
                          const std::vector<RrPointFrame> &rests,
                          const RrPointFrame &effector, int mode,
                          double stretch, double weight, const RrVec3d &pole,
                          double twistDegrees, bool useAnimatedTs,
                          std::vector<RrPointFrame> *solved)
{
    std::vector<RrPointFrame> prepared;
    const std::vector<RrPointFrame> *solveChain = &chain;
    if (!useAnimatedTs) {
        if (!_RrPrepareRestDerivedIkChain(chain, rests, &prepared)) {
            return false;
        }
        solveChain = &prepared;
    }
    _RrSingleChainIkParams params;
    params.mode = _RrSingleChainIkMode(mode);
    params.pole = pole;
    params.twistDegrees = twistDegrees;
    params.stretch = stretch;
    params.weight = weight;
    const size_t ci = size_t(_program->poses->walkSteps[object].index);
    params.preserveJointOrientation =
        (_program->poses->constraints[ci].flags &
         RigExecWireConstraintPreserveJointOrientation) != 0;
    *solved = _RrSolveSingleChainIk(*solveChain, effector, params);
    return true;
}

namespace runtimePoseDetail {

bool _RrRunConstraintStep(RrProgram *program, size_t step, std::string *error);

bool
_RrRunConstraintStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->commits.size() ||
        size_t(wire.object) >= store.commits.size() ||
        size_t(wire.object) >= program->poses->walkSteps.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    const RigExecWireCommit &wireCommit =
        program->poses->commits[size_t(wire.object)];
    RrCommitScratch &commit = store.commits[size_t(wire.object)];
    const int walkIndex = program->poses->walkSteps[size_t(wire.object)].index;
    if (walkIndex < 0 ||
        size_t(walkIndex) >= program->poses->constraints.size() ||
        size_t(walkIndex) >= scratch->weightScratch.size() ||
        size_t(walkIndex) >= scratch->weightError.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no constraint";
        }
        return false;
    }
    const size_t ci = size_t(walkIndex);
    const RigExecWireConstraint &c = program->poses->constraints[ci];
    if (ci < store.matchValid.size())
        store.matchValid[ci] = 0;
    commit.abandoned = true;
    std::fill(commit.present.begin(), commit.present.end(), 0);

    const int deltaBase = c.deltaBase;
    if (size_t(wire.object) >= scratch->recordAfter.size() ||
        size_t(wire.object) >= scratch->recordEveryTarget.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    scratch->recordAfter[size_t(wire.object)] = 1;
    scratch->recordEveryTarget[size_t(wire.object)] = 1;
    if (deltaBase >= 0) {
        if (size_t(deltaBase) >= scratch->deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaPresent.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no delta path";
            }
            return false;
        }
        scratch->deltaPresent[size_t(deltaBase)] = 0;
        // Published for the geometry fold (the store pair mirrors the
        // baked B.deltaValues/deltaPresent the fold consumes).
        store.deltaPresent[size_t(deltaBase)] = 0;
    }
    if (ci < store.matchShownMask.size())
        store.matchShownMask[ci] = 0;
    RrPoseHost host(program, step);
    eval::StepRef ref;
    ref.kind = uint32_t(wire.kind);
    ref.object = wire.object;
    ref.part = wire.part;
    ref.slot = step;
    const eval::StepResult result = eval::RunValueStep(host, ref);
    if (result.outcome != eval::Outcome::Applied) {
        if (error) {
            const char *what = " names no commit";
            if (result.detail == eval::pose_detail::kPoseDetailNoConstraint) {
                what = " names an incomplete IK/FK match";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoChain) {
                what = " names no chain target";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoRest) {
                what = " names no rest slot";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoPole) {
                what = " names no pole object";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoVersion) {
                what = " names no fin version";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoDeltaPath) {
                what = " names no delta path";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoCandidate) {
                what = " names no candidate slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoTarget) {
                what = " names no target slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoArrays) {
                what = " names no constraint arrays";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoSource) {
                what = " names no constraint source";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoSpace) {
                what = " names no space slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoWindow) {
                what = " names no harmonics window";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoSlot) {
                what = " names no slot";
            }
            *error = _RrStepHead(program, step) + what;
        }
        return false;
    }
    return true;
}

} // namespace runtimePoseDetail

bool
RrLookUpConstraint(RrProgram *program, eval::ConstraintStepCache &cache,
                   const eval::OpLookup &lookup, const RigExecTypeDesc &type,
                   void *value, std::string *error)
{
    RrPoseHost host(program);
    return eval::LookUpConstraint(host, cache, lookup, type, value, error);
}

bool
RrStoreConstraint(RrProgram *program, size_t step,
                  eval::ConstraintStepCache &cache,
                  const eval::OpHostOutput &output, const void *value,
                  std::string *error)
{
    RrPoseHost host(program, step);
    return eval::StoreConstraint(host, cache, output, value, error);
}

std::string
RrConstraintFailure(const RrProgram *program, uint32_t commit,
                    const std::string &error)
{
    const RrProgram &p = *program;
    if (commit >= p.poses->commits.size())
        return error;
    const std::string mover = p.TextOrEmpty(p.poses->commits[commit].moverPath);
    return mover.empty() ? error : mover + ": " + error;
}

} // namespace rigExec
