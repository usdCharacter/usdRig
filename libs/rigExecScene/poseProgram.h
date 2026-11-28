#ifndef RIGEXEC_GRAPH_POSE_PROGRAM_H
#define RIGEXEC_GRAPH_POSE_PROGRAM_H
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecMath/rbf.h"
#include "rigExecMath/pointFrame.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/token.h"
namespace rigExec {
using RigExecSpaceSwitchRecord = RigExecSpaceSwitchRecordT<GfVec3d>;
using RigExecSpaceSwitchInputs = RigExecSpaceSwitchInputsT<GfMatrix4d>;
using RigExecSpaceCheckpointInput = RigExecSpaceCheckpointInputT<GfMatrix4d>;
using RigExecPoseInterpolatorRecord =
    RigExecPoseInterpolatorRecordT<RigExecRbfSolver>;
using RigExecPoseInterpolatorInputs =
    RigExecPoseInterpolatorInputsT<RigExecPointFrame>;
bool
RigExecPrepareRestDerivedIkChain(const std::vector<RigExecPointFrame> &current,
                                 const std::vector<RigExecPointFrame> &rest,
                                 std::vector<RigExecPointFrame> *prepared);
GfMatrix4d RigExecComposePoseAvars(double tx, double ty, double tz, double sx,
                                   double sy, double sz, double rx, double ry,
                                   double rz, double spin,
                                   const TfToken &order);
bool RigExecRunSpaceSwitch(const RigExecSpaceSwitchRecord &,
                           const RigExecSpaceSwitchInputs &,
                           RigExecPointFrame *);
bool RigExecRunSpaceSwitchMatrix(const RigExecSpaceSwitchRecord &,
                                 const RigExecSpaceSwitchInputs &,
                                 GfMatrix4d *);
RigExecPoseInterpolatorStatus
RigExecRunPoseInterpolator(RigExecPoseInterpolatorRecord &,
                           const RigExecPoseInterpolatorInputs &,
                           std::vector<double> *);
} // namespace rigExec
#endif
