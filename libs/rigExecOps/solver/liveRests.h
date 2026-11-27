#ifndef RIGEXEC_OPS_SOLVER_LIVE_RESTS_H
#define RIGEXEC_OPS_SOLVER_LIVE_RESTS_H

/// The solver ops' optional joint-rest re-basing. Private to rigExecOps.

#include "rigExecOps/opIo.h"
#include "rigExecMath/ribbonFramesKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecMath/dualQuatMath.h"

#include <vector>

namespace rigExecOps {
namespace opIo {

/// Re-bases each frame k of \p out that \p livePort marks onto \p restsPort's
/// k-th rest, as RigExecSampleRibbonFrames does; nothing when either is
/// unfed. \p out holds as many rests as frames.
inline void
RebaseLive(const rigExec::RigExecOpIo &io,
           const rigExec::RigExecPortHandle &restsPort,
           const rigExec::RigExecPortHandle &livePort,
           rigExecRig::PointFrameArray *out)
{
    using Landmarks = rigExecStd::RigExecArray<rigExecRig::PointLandmarks>;
    const Landmarks *joints = In<Landmarks>(io, restsPort);
    const auto *live = In<rigExecStd::RigExecArray<uint8_t>>(io, livePort);
    if (!joints || !live)
        return;
    rigExec::RigExecRebaseFramesKernel<rigExecStd::Vec3f, rigExecStd::Vec3d>(
        [&](size_t k) -> const rigExecRig::PointLandmarks * {
            return k < joints->Size() && k < live->Size() && (*live)[k]
                       ? &(*joints)[k]
                       : nullptr;
        },
        out->frames.Data(), out->rests.Data(), out->frames.Size());
}

} // namespace opIo
} // namespace rigExecOps

#endif
