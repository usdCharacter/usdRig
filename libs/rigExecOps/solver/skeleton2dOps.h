#ifndef RIGEXEC_OPS_SOLVER_SKELETON2D_OPS_H
#define RIGEXEC_OPS_SOLVER_SKELETON2D_OPS_H

/// The 2D skeleton solve on rigExecRig frames. The kernel is
/// rigExecMath/skeleton2dKernel.h, shared with the native and zero-USD
/// runtimes. USD-free.

#include "rigExecMath/skeleton2dKernel.h"
#include "rigExecOps/library.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

/// The compiled skeleton: the solve order (rigExec:joints index per entry),
/// each bone's setup in that order, and the parent-space, target and
/// joint-group arrays the setup ranges index.
struct Skeleton2dPayload {
    std::vector<uint32_t> order;
    std::vector<rigExec::RigExecBone2dSetup> setup;
    std::vector<int32_t> spaceSources, targetCandidates;
    std::vector<rigExec::RigExecJointGroup> groups;
    std::vector<int32_t> groupMembers;
};

const rigExec::RigExecOpHandle &Skeleton2d();

// Rests (PointLandmarksList, rigExec:joints order), optional StartRest and
// StartPose (PointLandmarks, the start frame; fed together), optional
// ExternalRests and ExternalPoses (PointLandmarksList, one each per outside
// parent space), Channels (DoubleArray, kRigExecBone2dChannelCount per
// joint in rigExec:joints order, RigExecBone2dSetChannel's order, a bool
// channel 0 or 1), optional Selectors and GroupSelectors (DoubleArray). Out
// (PointFrameArray, the joints' frames on Rests; a failed solve gives the
// rests themselves and a malformed one nothing) and Result (UInt, a
// RigExecSkeleton2dResult).
const rigExec::RigExecPortHandle &Skeleton2dRests();
const rigExec::RigExecPortHandle &Skeleton2dStartRest();
const rigExec::RigExecPortHandle &Skeleton2dStartPose();
const rigExec::RigExecPortHandle &Skeleton2dExternalRests();
const rigExec::RigExecPortHandle &Skeleton2dExternalPoses();
const rigExec::RigExecPortHandle &Skeleton2dChannels();
const rigExec::RigExecPortHandle &Skeleton2dSelectors();
const rigExec::RigExecPortHandle &Skeleton2dGroupSelectors();
const rigExec::RigExecPortHandle &Skeleton2dOut();
const rigExec::RigExecPortHandle &Skeleton2dResult();

} // namespace rigExecOps

#endif
