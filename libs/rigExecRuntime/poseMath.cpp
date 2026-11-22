// Runtime pose decomposition and frame blending; no USD dependencies.

#include "poseInternal.h"
#include "dualQuatMath.h"
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

// RigExecBlendFrames (solvers.cpp).
RrPointFrame
_RrBlendFrames(const RrPointFrame &a, const RrPointFrame &b,
               const std::array<RrVec3d, 4> &restPoints, double weight,
               bool logScale, const std::array<RrVec3d, 4> *outRestPoints)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (!outRestPoints) {
        if (w <= 0.0) {
            return a;
        }
        if (w >= 1.0) {
            return b;
        }
    } else if (w <= 0.0 || w >= 1.0) {
        const RrPointFrame &pick = (w <= 0.0) ? a : b;
        RrMat4d end = _RrIdentity();
        if (!RrPointsToMatrix(restPoints, pick.points, &end)) {
            RrPointFrame held = pick;
            held.flags |= RrPointFrameDegenerate;
            return held;
        }
        return RrMatrixToPoints(*outRestPoints, end);
    }

    _RrTransformParams pa, pb;
    if (!_RrPointsToParams(restPoints, a.points, 2, &pa) ||
        !_RrPointsToParams(restPoints, b.points, 2, &pb)) {
        RrPointFrame held = (w < 0.5) ? a : b;
        held.flags |= RrPointFrameDegenerate;
        return held;
    }

    _RrTransformParams pr;
    pr.translation = pa.translation * (1 - w) + pb.translation * w;

    RrQuatd qa = pa.rotation, qb = pb.rotation;
    if (RrDot(qa.GetImaginary(), qb.GetImaginary()) +
            qa.GetReal() * qb.GetReal() <
        0) {
        qb = -qb;
    }
    pr.rotation = RrSlerp(w, qa, qb).GetNormalized();

    for (int i = 0; i < 3; ++i) {
        if (logScale && pa.scale[i] > 0 && pb.scale[i] > 0) {
            pr.scale[i] = std::exp(std::log(pa.scale[i]) * (1 - w) +
                                   std::log(pb.scale[i]) * w);
        } else {
            pr.scale[i] = pa.scale[i] * (1 - w) + pb.scale[i] * w;
        }
        pr.shear[i] = pa.shear[i] * (1 - w) + pb.shear[i] * w;
    }

    const RrMat4d m = _RrParamsToMatrix(pr);
    return RrMatrixToPoints(outRestPoints ? *outRestPoints : restPoints, m);
}

// Swing/twist decomposition of q about unit axis (solvers.cpp).
void
_RrSwingTwist(const RrQuatd &q, const RrVec3d &axis, RrQuatd *swing,
              RrQuatd *twist)
{
    const RrVec3d im = q.GetImaginary();
    const double proj = RrDot(im, axis);
    RrQuatd t(q.GetReal(), axis * proj);
    const double len = std::sqrt(t.GetReal() * t.GetReal() +
                                 RrDot(t.GetImaginary(), t.GetImaginary()));
    if (len < 1e-15) {
        *twist = RrQuatd::GetIdentity();
    } else {
        *twist = t * (1.0 / len);
    }
    *swing = q * twist->GetInverse();
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
