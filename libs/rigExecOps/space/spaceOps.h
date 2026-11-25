#ifndef RIGEXEC_OPS_SPACE_SPACE_OPS_H
#define RIGEXEC_OPS_SPACE_SPACE_OPS_H

/// A provider's matrices, a frame record's matrix and a connected provider's
/// refresh. The kernels are rigExecMath/providerMatrixKernel.h,
/// rigExecEvalCore/pose.h's frame record and
/// rigExecMath/providerRefreshKernel.h.

#include "rigExecOps/library.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

struct ProviderMatrixPayload {
    /// rest -> final; rest -> base otherwise.
    bool posed = true;
    /// A published joint, whose base fallback follows base -> final.
    bool joint = false;

    bool operator==(const ProviderMatrixPayload &o) const
    {
        return posed == o.posed && joint == o.joint;
    }
};

const rigExec::RigExecOpHandle &ProviderMatrix();

// Rest, Final and Base (PointFrame; Final only for a posed matrix). Out
// (Mat4d, row vectors).
const rigExec::RigExecPortHandle &ProviderMatrixRest();
const rigExec::RigExecPortHandle &ProviderMatrixFinal();
const rigExec::RigExecPortHandle &ProviderMatrixBase();
const rigExec::RigExecPortHandle &ProviderMatrixOut();

struct FrameMatrixPayload {
    /// A solver commit records the frames its solver published; a
    /// constraint commit records by its record gates.
    bool solverOutput = false;
    /// The solver output position the record names.
    int32_t position = -1;
    /// The constraint target the record names.
    int32_t target = -1;

    bool operator==(const FrameMatrixPayload &o) const
    {
        return solverOutput == o.solverOutput && position == o.position &&
               target == o.target;
    }
};

const rigExec::RigExecOpHandle &FrameMatrix();

// Rest and Frame (PointFrame). A solver commit's Present (BoolArray, one
// per solver output, set where the solver published it this run); a
// constraint commit's RecordAfter and RecordEveryTarget (Bool). Out (Mat4d,
// identity where not recorded or not convertible) and Valid (Bool).
const rigExec::RigExecPortHandle &FrameMatrixRest();
const rigExec::RigExecPortHandle &FrameMatrixFrame();
const rigExec::RigExecPortHandle &FrameMatrixPresent();
const rigExec::RigExecPortHandle &FrameMatrixRecordAfter();
const rigExec::RigExecPortHandle &FrameMatrixRecordEveryTarget();
const rigExec::RigExecPortHandle &FrameMatrixOut();
const rigExec::RigExecPortHandle &FrameMatrixValid();

struct ProviderRefreshPayload {
    /// Per carried descendant, how many of BlockingParents are its.
    std::vector<uint32_t> blockingCounts;

    bool operator==(const ProviderRefreshPayload &o) const
    {
        return blockingCounts == o.blockingCounts;
    }
};

const rigExec::RigExecOpHandle &ProviderRefresh();

// Base and Final (PointFrame) are the provider's entering versions, Raw and
// Current (PointFrame) its re-evaluated base and current frames. Optional
// Constrained (BoolArray): per prior constraint on the provider, whether it
// committed this run. CarryBase and CarryFinal (PointFrameList) are the
// carried descendants' entering versions; BlockingParents (Mat4dArray) the
// parent spaces between each descendant and the provider, in payload order,
// any not identity holding the descendant in place. BaseOut, FinalOut
// (PointFrame), CarryBaseOut and CarryFinalOut (PointFrameList).
const rigExec::RigExecPortHandle &ProviderRefreshBase();
const rigExec::RigExecPortHandle &ProviderRefreshFinal();
const rigExec::RigExecPortHandle &ProviderRefreshRaw();
const rigExec::RigExecPortHandle &ProviderRefreshCurrent();
const rigExec::RigExecPortHandle &ProviderRefreshConstrained();
const rigExec::RigExecPortHandle &ProviderRefreshCarryBase();
const rigExec::RigExecPortHandle &ProviderRefreshCarryFinal();
const rigExec::RigExecPortHandle &ProviderRefreshBlockingParents();
const rigExec::RigExecPortHandle &ProviderRefreshBaseOut();
const rigExec::RigExecPortHandle &ProviderRefreshFinalOut();
const rigExec::RigExecPortHandle &ProviderRefreshCarryBaseOut();
const rigExec::RigExecPortHandle &ProviderRefreshCarryFinalOut();

} // namespace rigExecOps

#endif
