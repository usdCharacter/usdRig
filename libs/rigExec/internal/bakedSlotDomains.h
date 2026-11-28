#ifndef RIGEXEC_INTERNAL_BAKED_SLOT_DOMAINS_H
#define RIGEXEC_INTERNAL_BAKED_SLOT_DOMAINS_H

#include "rigExec/internal/compiledProgram.h"
#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/matrix4d.h"

#include <cstdint>
#include <string>

namespace rigExec {

struct RigExecBakedProgramImpl;
class RigExecPathText;

/// What the baked program knows how to do with the values of one slot
/// domain. RigExecBakedSlotDomainOpsOf returns the entry for a domain; each
/// function is defined beside the code that owns that domain's storage.
struct RigExecBakedSlotDomainOps {
    RigExecBakedSlotDomain domain;
    const char *name;
    /// Returns the value at \p slot to its not-computed state, as when an
    /// operation cycle sets its writer aside. Null when there is nothing to
    /// reset.
    void (*reset)(RigExecBakedProgramImpl &, uint32_t slot);
    /// Appends to \p out a name for the value at \p slot that two compiles
    /// of one rig agree on, such as a prim path; false when the slot has
    /// none. Null when the domain has no such names.
    bool (*identity)(const RigExecBakedProgramImpl &, uint32_t slot,
                     RigExecPathText &, std::string *out);
};

/// The entry for \p domain; an entry with no functions for a value past
/// the last domain.
const RigExecBakedSlotDomainOps &
RigExecBakedSlotDomainOpsOf(RigExecBakedSlotDomain domain);

/// The value a set-aside pose frame holds.
inline RigExecPointFrame
RigExecBakedSetAsideFrame()
{
    RigExecPointFrame frame;
    frame.flags = 0;
    return frame;
}

/// The value a set-aside matrix holds.
GfMatrix4d RigExecBakedSetAsideMatrix();

// bakedPose.cpp
void RigExecBakedResetAvars(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetPoseBase(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetPoseFin(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetPosedM(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetFinalMatrix(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetBaseMatrix(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetSwitchFrame(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetAggregate(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetCandidates(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetCommitTable(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetCommitDelta(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetCommitStaging(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetConstraintDelta(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetPoseWeight(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetFrameMatrix(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetRest(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetLadder(RigExecBakedProgramImpl &, uint32_t);
bool RigExecBakedPathIdentity(const RigExecBakedProgramImpl &, uint32_t,
                              RigExecPathText &, std::string *);

// bakedProperties.cpp
void RigExecBakedResetPropertyResult(RigExecBakedProgramImpl &, uint32_t);

// bakedWeights.cpp
void RigExecBakedResetWeightPacket(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetWeightFrames(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetWeightFramesBase(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetWeightFieldSlot(RigExecBakedProgramImpl &, uint32_t);

// bakedSpaces.cpp
void RigExecBakedResetSpaceValue(RigExecBakedProgramImpl &, uint32_t);

// bakedGeometry.cpp
void RigExecBakedResetChainBase(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetChainPoints(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetDerivedOut(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetSkinTopology(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetRevisionOut(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetRevisionPacket(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetRevisionTransforms(RigExecBakedProgramImpl &, uint32_t);
void RigExecBakedResetRevisionDone(RigExecBakedProgramImpl &, uint32_t);
bool RigExecBakedChainIdentity(const RigExecBakedProgramImpl &, uint32_t,
                               RigExecPathText &, std::string *);
bool RigExecBakedRevisionIdentity(const RigExecBakedProgramImpl &, uint32_t,
                                  RigExecPathText &, std::string *);
bool RigExecBakedRevisionOutIdentity(const RigExecBakedProgramImpl &, uint32_t,
                                     RigExecPathText &, std::string *);

// bakedSchedule.cpp
bool RigExecBakedStageFramesIdentity(const RigExecBakedProgramImpl &, uint32_t,
                                     RigExecPathText &, std::string *);

} // namespace rigExec

#endif
