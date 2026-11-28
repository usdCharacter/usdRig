// Runtime pose decomposition and frame blending; no USD dependencies.

#include "poseInternal.h"
#include "rigExecMath/dualQuatMath.h"
#include "rigExecMath/frameBlendKernel.h"
#include "rigExecMath/frameParamsKernel.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace runtimePoseDetail {

RrPointFrame _RrBlendFrames(const RrPointFrame &a, const RrPointFrame &b,
                            const std::array<RrVec3d, 4> &restPoints,
                            double weight, bool logScale,
                            const std::array<RrVec3d, 4> *outRestPoints);

namespace {

using RrFrameParams = RigExecTransformParamsKernelT<RrVec3f, RrVec3d>;

} // namespace

bool
_RrPointsToParams(const std::array<RrVec3d, 4> &restPoints,
                  const std::array<RrVec3d, 4> &posePoints, int reflectionAxis,
                  _RrTransformParams *params)
{
    RrFrameParams p;
    if (!RigExecPointsToParamsKernel<RrVec3f, RrVec3d>(restPoints, posePoints,
                                                       reflectionAxis, &p)) {
        return false;
    }
    params->translation = p.translation;
    params->rotation = p.rotation;
    params->scale = p.scale;
    params->shear = p.shear;
    params->reflectionAxis = p.reflectionAxis;
    return true;
}

RrMat4d
_RrParamsToMatrix(const _RrTransformParams &params)
{
    RrFrameParams p;
    p.translation = params.translation;
    p.rotation = params.rotation;
    p.scale = params.scale;
    p.shear = params.shear;
    p.reflectionAxis = params.reflectionAxis;
    return RigExecParamsToMatrixKernel<RrVec3f, RrVec3d>(p);
}

// RigExecBlendFrames (solvers.cpp), always shortest arc.
RrPointFrame
_RrBlendFrames(const RrPointFrame &a, const RrPointFrame &b,
               const std::array<RrVec3d, 4> &restPoints, double weight,
               bool logScale, const std::array<RrVec3d, 4> *outRestPoints)
{
    return RigExecBlendFramesKernel<RrVec3f, RrVec3d>(a, b, restPoints, weight,
                                                      logScale, outRestPoints);
}

RrPointFrame
_RrTransformFrame(const RrPointFrame &frame, const RrMat4d &space)
{
    RrPointFrame out = frame;
    for (RrVec3d &p : out.points) {
        p = space.Transform(p);
    }
    return out;
}

} // namespace runtimePoseDetail

} // namespace rigExec
