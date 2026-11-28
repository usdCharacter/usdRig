// Private pose implementation contracts.

#ifndef RIGEXEC_RUNTIME_POSE_INTERNAL_H
#define RIGEXEC_RUNTIME_POSE_INTERNAL_H

#include "store.h"
#include "rigExecMath/autoClavicleKernel.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecMath/jointGroupKernel.h"
#include "rigExecMath/skeleton2dKernel.h"
#include "rigExecMath/spaceMatchKernel.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecRuntime/rbfSolver.h"
#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

struct RrPoseSolverState;

struct RrPoseScratch;

/// The rest frames the pose scratch holds per slot: the constants until a
/// prologue recomposes the ladder, then what it composed.
const std::vector<RrPointFrame> &RrPoseRestFrames(const RrProgram *program);

namespace runtimePoseDetail {

RrMat4d _RrIdentity();

// std::acos(-1) rather than M_PI, as the kernels do.
inline const double _RrPi = std::acos(-1);

RrMat4d _RrComposeAvars(double tx, double ty, double tz, double sx, double sy,
                        double sz, double rx, double ry, double rz,
                        double rspin, const std::string &order);

bool _RrLive(const RrProgram *program, const RigExecWireInput &read);

bool _RrLiveSolver(const RrProgram *program, size_t solver, int field);

RrPoseScratch *_RrScratch(RrProgram *program);

// RigExecTransformParams (pointFrame.h), field for field.
struct _RrTransformParams {
    RrVec3d translation{0, 0, 0};
    RrQuatd rotation{1, RrVec3d(0, 0, 0)};
    RrVec3d scale{1, 1, 1};
    RrVec3d shear{0, 0, 0};
    int reflectionAxis = 2;
};

bool _RrPointsToParams(const std::array<RrVec3d, 4> &restPoints,
                       const std::array<RrVec3d, 4> &posePoints,
                       int reflectionAxis, _RrTransformParams *params);

RrMat4d _RrParamsToMatrix(const _RrTransformParams &params);

// RigExecBlendEnvelope (envelope.h): the endpoint branches are the
// contract, not an optimization.
template <class T, class Weight>
T
_RrBlendEnvelope(const T &preceding, const T &full, Weight weight)
{
    if (weight <= Weight(0)) {
        return preceding;
    }
    if (weight >= Weight(1)) {
        return full;
    }
    return preceding + (full - preceding) * weight;
}

RrPointFrame _RrBlendFrames(const RrPointFrame &a, const RrPointFrame &b,
                            const std::array<RrVec3d, 4> &restPoints,
                            double weight, bool logScale,
                            const std::array<RrVec3d, 4> *outRestPoints);

// RigExecTransformFrame: every landmark carried by \p space, nothing else.
RrPointFrame _RrTransformFrame(const RrPointFrame &frame, const RrMat4d &space);

// A control's channels as the compose reads them, for the match kernels.
RigExecIkFkMatchChannels _RrMatchChannelsOf(const RrProgram &program,
                                            const RrPoseScratch &scratch,
                                            size_t slot);

} // namespace runtimePoseDetail

// One solver's rest copies the op hosts fall back on where no rest slot or
// live rest stands: the wire solver is the compiled rig half.
struct RrPoseSolverState {
    std::vector<std::array<RrVec3d, 4>> jointRests;
    std::array<std::array<RrVec3d, 4>, 3> ikRests;
    std::vector<std::array<RrVec3d, 4>> splineJointRests;
};

struct RrPoseScratch {
    // The composed ladder and its move-comparison lasts, mirroring the
    // baked program's restM/restPts/restFrames/selfD/parentDinv/rotOrder/
    // posedAuthored(M)/restRoundTrip/defaultRoundTrip tables.
    std::vector<RrMat4d> restM, selfD, parentDinv, posedAuthoredM;
    std::vector<RrMat4d> posedD, parentSpaceM;
    std::vector<char> parentSpaceAuthored;
    std::vector<RrMat4d> restRoundTrip, defaultRoundTrip;
    std::vector<std::array<RrVec3d, 4>> restPts;
    std::vector<RrPointFrame> restFrames;
    std::vector<uint32_t> rotOrder;
    std::vector<char> posedAuthored;
    std::vector<char> noScaleAvars;
    /// avars:rotationSign per slot, packed as RigExecRotationSignMask does.
    std::vector<unsigned char> rotationSign;
    std::vector<RrMat4d> lastRestM, lastSelfD, lastParentDinv;
    std::vector<RrMat4d> lastPosedAuthoredM;
    std::vector<RrMat4d> lastPosedD, lastParentSpaceM;
    std::vector<char> lastParentSpaceAuthored;
    std::vector<unsigned char> lastRotationSign;
    std::vector<char> lastPosedAuthored;
    std::vector<uint32_t> lastRotOrder;
    // A drag stood last run, so this run writes the constant avars back.
    bool avarsDisturbed = false;
    // Solver live rests and per-run elements.
    std::vector<RrPoseSolverState> solvers;
    // A constraint commit's exit flags, written by its constraint step on
    // every run and read by the commit's FrameMatrix steps. Set to 1 at
    // Open and never reset at the head of a run.
    std::vector<char> recordAfter, recordEveryTarget;
    // Constraint envelope scratch, one step's own storage.
    std::vector<std::vector<float>> weightScratch;
    std::vector<std::string> weightError;
    // Geometry-domain constraint deltas (the Matrix revision reads them).
    std::vector<RrMat4d> deltaValues;
    std::vector<char> deltaPresent;
};

} // namespace rigExec

#endif
