// RigExec solver glue shared by exec and the bake.
// See solverKernels.h.
#include "rigExec/internal/solverKernels.h"
#include "rigExec/internal/frameExtraction.h"

#include "rigExecMath/dualQuat.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/ribbonFramesKernel.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/switchMathKernel.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
// The ribbon sampler has one definition for every caller, so none can drift
// into publishing frames where this one publishes nothing.

namespace {

// The joint rest a pose step BELOW this solver really wrote for element
// \p k, onto which that element is re-based; null keeps the solver's own
// rest, which is what keeps every other rig bit-identical.
auto
_LiveJointRest(const std::vector<std::array<GfVec3d, 4>> &jointRests,
               const std::vector<bool> &jointRestLive)
{
    return [rests = &jointRests,
            live = &jointRestLive](size_t k) -> const std::array<GfVec3d, 4> * {
        return k < rests->size() && k < live->size() && (*live)[k]
                   ? &(*rests)[k]
                   : nullptr;
    };
}

} // namespace

RigExecPointFrameArray
RigExecSampleRibbonFrames(const std::vector<GfVec3f> &posed,
                          const std::vector<GfVec3f> &rest, int sampleCount,
                          const std::vector<std::array<GfVec3d, 4>> &jointRests,
                          const std::vector<bool> &jointRestLive)
{
    RigExecPointFrameArray result;
    RigExecSampleRibbonFramesKernel<GfVec3f, GfVec3d>(
        posed.data(), posed.size(), rest.data(), rest.size(), sampleCount,
        &result.frames, &result.rests);
    RigExecRebaseFramesKernel<GfVec3f, GfVec3d>(
        _LiveJointRest(jointRests, jointRestLive), &result.frames,
        &result.rests);
    return result;
}

// The twist weight defaulting, shared by the RigExecTwistDistribution exec
// callback and by the baked program. The caller drains the authored floats
// into the vector first -- the attribute's type is float, the distribution
// solves in double -- and this fills it only when nothing was authored.
void
RigExecResolveTwistWeights(int count, std::vector<double> *weights)
{
    if (weights->empty()) {
        const int n = std::max(count, 1);
        for (int k = 0; k < n; ++k) {
            weights->push_back(n == 1 ? 0.0 : double(k) / (n - 1));
        }
    }
}

// The twist distribution, shared by the RigExecTwistDistribution exec
// callback and by the baked program. One definition, so the rests half --
// every frame paired with the START landmarks, which is what the aggregate's
// consumers re-solve against -- cannot be re-derived differently.
RigExecPointFrameArray
RigExecSolveTwistDistribution(
    const RigExecPointFrame &start, const RigExecPointFrame &end,
    const std::array<GfVec3d, 4> &startRest,
    const std::array<GfVec3d, 4> &endRest, const std::vector<double> &weights,
    double twistTurns, const std::vector<std::array<GfVec3d, 4>> &jointRests,
    const std::vector<bool> &jointRestLive)
{
    RigExecPointFrameArray result;
    result.frames = RigExecDistributeTwist(start, end, startRest, endRest,
                                           weights, twistTurns);
    result.rests.assign(result.frames.size(), startRest);
    RigExecRebaseFramesKernel<GfVec3f, GfVec3d>(
        _LiveJointRest(jointRests, jointRestLive), &result.frames,
        &result.rests);
    return result;
}

bool
RigExecFrameRotation(const RigExecPointFrame &frame, GfQuatd *out)
{
    GfMatrix4d matrix(1.0);
    if (!frame.IsValid() || frame.IsDegenerate() ||
        !RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points,
                               &matrix)) {
        return false;
    }
    matrix.SetTranslateOnly(GfVec3d(0.0));
    *out = matrix.GetOrthonormalized(/* issueWarning = */ false)
               .ExtractRotationQuat();
    return true;
}

namespace {

bool
_FrameMatrix(const RigExecPointFrame &frame, GfMatrix4d *out)
{
    return frame.IsValid() && !frame.IsDegenerate() &&
           RigExecPointsToMatrix(RigExecIdentityLandmarks(), frame.points, out);
}

} // namespace

bool
RigExecFrameTranslation(const RigExecPointFrame &driverFinal,
                        const RigExecPointFrame &driverRest,
                        const RigExecPointFrame *parentFinal,
                        const RigExecPointFrame *parentRest, GfVec3d *out)
{
    GfMatrix4d world(1.0), restWorld(1.0);
    if (!_FrameMatrix(driverFinal, &world) ||
        !_FrameMatrix(driverRest, &restWorld)) {
        return false;
    }
    GfMatrix4d local = world, restLocal = restWorld;
    double det = 0.0;
    if (parentFinal && parentRest) {
        GfMatrix4d parent(1.0), restParent(1.0);
        if (!_FrameMatrix(*parentFinal, &parent) ||
            !_FrameMatrix(*parentRest, &restParent)) {
            return false;
        }
        const GfMatrix4d parentInverse = parent.GetInverse(&det);
        if (det == 0.0) {
            return false;
        }
        const GfMatrix4d restParentInverse = restParent.GetInverse(&det);
        if (det == 0.0) {
            return false;
        }
        local = world * parentInverse;
        restLocal = restWorld * restParentInverse;
    }
    const GfMatrix4d restLocalInverse = restLocal.GetInverse(&det);
    if (det == 0.0) {
        return false;
    }
    *out = restLocalInverse.Transform(local.ExtractTranslation());
    return true;
}

GfMatrix4d
RigExecBlendTransforms(const GfMatrix4d &a, const GfMatrix4d &b, double weight)
{
    return RigExecBlendTransformsT(
        a, b, weight, [](double t, const GfQuatd &q0, const GfQuatd &q1) {
            return GfSlerp(t, q0, q1);
        });
}

GfMatrix4d
RigExecOrientSpaceDelta(const GfMatrix4d &delta, const GfMatrix4d &local,
                        const GfMatrix4d &localInverse,
                        const GfMatrix4d &unswitched)
{
    GfMatrix4d frame = delta * local;
    frame[3][0] = unswitched[3][0];
    frame[3][1] = unswitched[3][1];
    frame[3][2] = unswitched[3][2];
    return frame * localInverse;
}

GfMatrix4d
RigExecFilterSpaceRotation(const GfMatrix4d &m, const GfVec3d &axis,
                           RigExecRotationFilter filter)
{
    return RigExecFilterSpaceRotationT(m, axis, int(filter));
}

GfMatrix4d
RigExecMaskTransform(const GfMatrix4d &m, const bool translation[3],
                     const bool rotation[3], const bool scale[3])
{
    return RigExecMaskTransformT(m, translation, rotation, scale);
}

} // namespace rigExec
