// rigExecRuntime shared kernels (M2 framework).
// Bit-identical ports of the point-frame kernels every family reads,
// from libs/rigExecMath/pointFrame.cpp, libs/rigExec/internal/frameExtraction.h
// and libs/rigExec/solverKernels.cpp. Gf -> Rr, algorithm untouched.
#include "rigExecRuntime/store.h"

#include "rigExecMath/dualQuatMath.h"
#include "rigExecRuntime/rbfSolver.h"
#include "rigExecMath/switchMathKernel.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

RrPointFrame
RrFrameFromMatrix(const RrMat4d &m)
{
    RrPointFrame frame;
    frame.flags = RigExecMatrixLandmarksT(m, &frame.points)
                      ? RrPointFrameValid
                      : RrPointFrameDegenerate;
    return frame;
}

double
RrFrameEpsilon(const std::array<RrVec3d, 4> &restPoints)
{
    return RrDualQuatFrameEpsilon(restPoints);
}

bool
RrPointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                 const std::array<RrVec3d, 4> &posePoints, RrMat4d *matrix)
{
    return RrDualQuatPointsToMatrix(restPoints, posePoints, matrix);
}

bool
RrPointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                 const RrPointFrame &frame, RrMat4d *matrix)
{
    return RrPointsToMatrix(restPoints, frame.points, matrix);
}

RrPointFrame
RrMatrixToPoints(const std::array<RrVec3d, 4> &restPoints,
                 const RrMat4d &matrix)
{
    RrPointFrame frame;
    bool reflected = false, degenerate = false;
    frame.points = RigExecMatrixToPointsT<RrMat3d>(restPoints, matrix,
                                                   &reflected, &degenerate);
    frame.flags = RrPointFrameValid;
    if (reflected)
        frame.flags |= RrPointFrameReflected;
    if (degenerate)
        frame.flags |= RrPointFrameDegenerate;
    return frame;
}

bool
RrFrameRotation(const RrPointFrame &frame, RrQuatd *out)
{
    return RrPoseInterpolatorMathT<RrPointFrame>::FrameRotation(frame, out);
}

RrMat4d
RrElementOutSpace(const RrPointFrameArray *source, size_t index)
{
    RrMat4d m;
    m.SetIdentity();
    if (!source || index >= source->frames.size() ||
        !source->frames[index].IsValid()) {
        return m;
    }
    const std::array<RrVec3d, 4> &posed = source->frames[index].points;
    const RrVec3d origin = posed[0];
    const bool hasRest = index < source->rests.size();
    RrMat4d rm;
    rm.SetIdentity();
    for (int a = 0; a < 3; ++a) {
        RrVec3d direction = posed[size_t(a) + 1] - origin;
        const double posedLength = direction.GetLength();
        if (posedLength < 1e-12) {
            return m;
        }
        direction /= posedLength;
        double ratio = 1.0;
        if (hasRest) {
            const double restLength =
                (source->rests[index][size_t(a) + 1] - source->rests[index][0])
                    .GetLength();
            if (restLength > 1e-12) {
                ratio = posedLength / restLength;
            }
        }
        rm.SetRow(a, RrVec4d(direction[0] * ratio, direction[1] * ratio,
                             direction[2] * ratio, 0));
    }
    rm.SetRow(3, RrVec4d(origin[0], origin[1], origin[2], 1));
    return rm;
}

RrPointFrame
RrExtractElementFrame(const RrPointFrameArray *source, size_t index)
{
    if (source && index < source->frames.size()) {
        const RrPointFrame &f = source->frames[index];
        if (f.IsValid() && !f.IsDegenerate()) {
            return RrFrameFromMatrix(RrElementOutSpace(source, index));
        }
    }
    RrPointFrame degenerate;
    degenerate.flags = RrPointFrameDegenerate;
    return degenerate;
}

bool
RrFrameTranslation(const RrPointFrame &driverFinal,
                   const RrPointFrame &driverRest,
                   const RrPointFrame *parentFinal,
                   const RrPointFrame *parentRest, RrVec3d *out)
{
    return RrPoseInterpolatorMathT<RrPointFrame>::FrameTranslation(
        driverFinal, driverRest, parentFinal, parentRest, out);
}

} // namespace rigExec
