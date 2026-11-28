// RigExec spline-IK spine: the Gf instantiation of splineIkKernel.h.
#include "splineIk.h"

#include "dualQuat.h"

namespace rigExec {

RigExecSplineIkRest
RigExecSplineIkMakeRest(const std::vector<RigExecPointFrame> &joints,
                        const RigExecPointFrame &rootControl,
                        const RigExecPointFrame &midControl,
                        const RigExecPointFrame &endControl,
                        const std::vector<double> &volumeWeights,
                        RigExecSplineIkRestLength restLength)
{
    RigExecSplineIkRest rest;
    RigExecSplineIkMakeRestKernel<GfVec3d>(
        joints, rootControl, midControl, endControl, volumeWeights,
        restLength == RigExecSplineIkRestLength::Chain, &rest);
    return rest;
}

bool
RigExecSplineIkPoseCvs(const RigExecSplineIkRest &rest,
                       const RigExecSplineIkControls &controls,
                       const RigExecSplineIkParams &params,
                       std::array<GfVec3d, 4> *cvs)
{
    return RigExecSplineIkPoseCvsKernel<GfVec3f, GfVec3d>(rest, controls,
                                                          params, cvs);
}

double
RigExecSplineIkTwistAboutAxis(const RigExecPointFrame &restFrame,
                              const RigExecPointFrame &posedFrame,
                              const GfVec3d &axis)
{
    return RigExecSplineIkTwistAboutAxisKernel<GfVec3f, GfVec3d>(
        restFrame, posedFrame, axis);
}

bool
RigExecSolveSplineIk(const RigExecSplineIkRest &rest,
                     const RigExecSplineIkControls &controls,
                     const RigExecSplineIkParams &params,
                     RigExecSplineIkResult *result)
{
    return RigExecSolveSplineIkKernel<GfVec3f, GfVec3d, RigExecPointFrame>(
        rest, controls, params, result);
}

} // namespace rigExec
