#include "solverProgram.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExec/api/types.h"
#include "pxr/base/gf/math.h"
#include <algorithm>
namespace rigExec {
bool
RigExecRunSolver(const RigExecSolverRecord &s, const RigExecSolverInputs &input,
                 RigExecSolverWorkspace *scratch,
                 RigExecPointFrameArray *output, std::string *error)
{
    if (!scratch || !output) {
        if (error)
            *error = "null solver workspace/output";
        return false;
    }
    auto &aggregate = *output;
    aggregate.frames.clear();
    aggregate.rests.clear();
    if (s.degenerate)
        return true;
    switch (s.kind) {
    case RigExecSolverKind::FkChain:
        if (error)
            *error = "an FK chain runs as op nodes";
        return false;
    case RigExecSolverKind::TwoBoneIk:
        if (error)
            *error = "a two-bone IK runs as an op node";
        return false;
    case RigExecSolverKind::BlendPointFrames: {
        const auto *a = input.blendA, *b = input.blendB;
        if (!a) {
            if (b)
                aggregate = *b;
            return true;
        }
        if (!b) {
            aggregate = *a;
            return true;
        }
        if (s.blendRotationRejected)
            return true;
        const size_t n = a->GetSize();
        const double weight = std::min(std::max(input.blendWeight, 0.0), 1.0);
        if (n == b->GetSize() && a->rests.size() == n) {
            aggregate.frames.reserve(n);
            aggregate.rests.reserve(n);
            for (size_t k = 0; k < n; ++k) {
                const bool live = k < s.restIsLive.size() && s.restIsLive[k] &&
                                  k < s.jointRests.size();
                aggregate.frames.push_back(RigExecBlendFrames(
                    a->frames[k], b->frames[k], a->rests[k], weight,
                    RigExecRotationBlend::ShortestArc, s.scaleMode,
                    live ? &s.jointRests[k] : nullptr));
                aggregate.rests.push_back(live ? s.jointRests[k] : a->rests[k]);
            }
        }
        return true;
    }
    case RigExecSolverKind::TwistDistribution:
        if (error)
            *error = "a twist distribution runs as an op node";
        return false;
    case RigExecSolverKind::Ribbon:
        if (error)
            *error = "a ribbon runs as an op node";
        return false;
    case RigExecSolverKind::SplineIk: {
        auto params = RigExecSolverSplineParams(s, input);
        const auto *rest = &s.splineRest;
        if (input.hasSpace) {
            GfMatrix4d delta(1.0);
            if (RigExecPointsToMatrix(input.spaceRest, input.space, &delta) &&
                delta != GfMatrix4d(1.0)) {
                auto &spaced = scratch->spacedFrames;
                spaced.clear();
                spaced.reserve(s.splineRestFrames.size());
                for (const auto &frame : s.splineRestFrames)
                    spaced.push_back(RigExecTransformFrame(frame, delta));
                scratch->spacedRest = RigExecSplineIkMakeRest(
                    spaced, RigExecTransformFrame(s.splineRootRest, delta),
                    RigExecTransformFrame(s.splineMidRest, delta),
                    RigExecTransformFrame(s.splineEndRest, delta),
                    s.splineRestWeights, s.splineRestMode);
                rest = &scratch->spacedRest;
            }
        }
        RigExecSplineIkControls controls;
        controls.root = input.root;
        controls.mid = input.mid;
        controls.end = input.end;
        RigExecSolveSplineIk(*rest, controls, params, &scratch->splineResult);
        if (scratch->splineResult.joints.size() == s.splineCount) {
            aggregate.frames.reserve(s.splineCount);
            for (const auto &joint : scratch->splineResult.joints)
                aggregate.frames.push_back(joint.frame);
            aggregate.rests = s.splineJointRests;
        }
        return true;
    }
    case RigExecSolverKind::Skeleton2d: {
        // Rests and outputs in rigExec:joints order; the kernel runs in solve
        // order, in the start frame's plane. A failed solve publishes every
        // bone's setup frame (no deformation) and says why.
        const size_t n = s.bone2dOrder.size();
        if (s.jointRests.size() != n || s.bone2dSetup.size() != n ||
            input.bone2dChannels.size() != n ||
            input.controls.size() != s.controlRests.size()) {
            if (error)
                *error = kRigExecSkeleton2dCountsDiffer;
            return false;
        }
        const RigExecSkeleton2dStatic extra = RigExecSkeleton2dStaticOf(
            s.bone2dSpaceSources, s.bone2dTargetCandidates, s.jointGroups,
            s.jointGroupMembers);
        auto &posed = scratch->bone2dExternalPoses;
        posed.resize(input.controls.size());
        for (size_t e = 0; e < input.controls.size(); ++e)
            posed[e] = input.controls[e].points;
        RigExecSkeleton2dLandmarkInputs<RigExecSolverRest> frames;
        if (s.hasStart) {
            frames.startRest = &s.startRest;
            frames.startPose = &input.start.points;
        }
        frames.externalRests = s.controlRests.data();
        frames.externalPoses = posed.data();
        frames.externalCount = posed.size();
        frames.selectors = input.bone2dSelectors.data();
        frames.selectorCount = input.bone2dSelectors.size();
        frames.groupSelectors = input.jointGroupSelectors.data();
        frames.groupSelectorCount = input.jointGroupSelectors.size();
        const RigExecSkeleton2dResult result = RigExecSolveSkeleton2dJoints(
            s.bone2dOrder, s.jointRests, s.bone2dSetup, extra, frames,
            [&](size_t j, RigExecBone2dChannels *channels) {
                *channels = input.bone2dChannels[j];
            },
            RigExecPointFrameValid, &scratch->bone2d, &aggregate.frames);
        if (result == RigExecSkeleton2dResult::Malformed) {
            if (error)
                *error = kRigExecSkeleton2dMalformed;
            return false;
        }
        aggregate.rests = s.jointRests;
        if (result == RigExecSkeleton2dResult::NotPlanar) {
            if (error)
                *error = kRigExecSkeleton2dNotPlanar;
            return false;
        }
        if (result == RigExecSkeleton2dResult::NotFinite) {
            if (error)
                *error = kRigExecSkeleton2dNotFinite;
            return false;
        }
        return true;
    }
    }
    if (error)
        *error = "unknown solver kind";
    return false;
}
} // namespace rigExec
