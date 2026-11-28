#ifndef RIGEXEC_OPS_SOLVER_POSE_COMMIT_OPS_H
#define RIGEXEC_OPS_SOLVER_POSE_COMMIT_OPS_H

/// The pose commit: a constraint's or solver batch's candidate frames
/// written over their joints, and each candidate's hierarchy delta carried
/// onto the descendants it moves. The kernel is
/// rigExecMath/poseCommitKernel.h.

#include "rigExecOps/library.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

struct PoseCommitPayload {
    /// Candidate joints.
    uint64_t slotCount = 0;
    /// Per descendant: its closest candidate's index, or -1.
    std::vector<int32_t> closest;
    /// Per descendant: whether a joint with an authored parent space lies
    /// between it and that candidate.
    std::vector<uint8_t> blocked;
    /// A solver batch's commit, which also writes base versions and steps
    /// over a descendant whose frames are unusable.
    bool solverOutput = false;
    /// Whether a Sources entry is a solver element, read as its out-space,
    /// rather than a frame taken as it is.
    bool elementFrames = false;
    /// Per Sources entry, in order, a later entry winning a candidate: the
    /// Sources input, the frame in it, and the candidate it supplies.
    std::vector<int32_t> entrySources;
    std::vector<int32_t> entryFrames;
    std::vector<int32_t> entryCandidates;
    /// Per descendant: the candidate whose write it carries, or -1; for the
    /// final (carried) and base (baseCarried) versions.
    std::vector<int32_t> carried;
    std::vector<int32_t> baseCarried;

    bool operator==(const PoseCommitPayload &o) const
    {
        return slotCount == o.slotCount && closest == o.closest &&
               blocked == o.blocked && solverOutput == o.solverOutput &&
               elementFrames == o.elementFrames &&
               entrySources == o.entrySources && entryFrames == o.entryFrames &&
               entryCandidates == o.entryCandidates && carried == o.carried &&
               baseCarried == o.baseCarried;
    }
};

const rigExec::RigExecOpHandle &PoseCommit();

// The candidates come from one Candidate per joint (PointFrame, an input
// that takes slotCount values; one flagged degenerate is absent, its solver
// having found no frame) or from Sources (PointFrameArray, an input that
// takes many values) through the payload's entries; a candidate no entry
// supplies is absent. Per candidate (PointFrameList, slotCount each):
// Entering (the joint's frame the delta is measured from) and Carry (what
// an absent candidate's joint keeps); optional Present (BoolArray; unfed,
// all present) and Status (UInt, the producing op's result: unless it is 1,
// solved, every candidate is absent). Per descendant (PointFrameList, one
// per closest entry, fed when there are descendants): Current, Closest (its
// closest candidate's entering frame) and DescendantCarry, which a present
// carried candidate overrides. With solverOutput, BaseCarry and, with
// descendants, DescendantBaseCarry.
//
// Out: Slots and Descendants (PointFrameList), each joint's new frame; with
// solverOutput, BaseSlots and BaseDescendants, the same frames for the base
// versions (empty otherwise); Presence (BoolArray), which candidates were
// present. With no candidate present every joint keeps its carry. A
// descendant that cannot carry the delta is an error.
const rigExec::RigExecPortHandle &PoseCommitPresent();
const rigExec::RigExecPortHandle &PoseCommitEntering();
const rigExec::RigExecPortHandle &PoseCommitCarry();
const rigExec::RigExecPortHandle &PoseCommitBaseCarry();
const rigExec::RigExecPortHandle &PoseCommitCurrent();
const rigExec::RigExecPortHandle &PoseCommitClosest();
const rigExec::RigExecPortHandle &PoseCommitDescendantCarry();
const rigExec::RigExecPortHandle &PoseCommitDescendantBaseCarry();
const rigExec::RigExecPortHandle &PoseCommitSlots();
const rigExec::RigExecPortHandle &PoseCommitDescendants();
const rigExec::RigExecPortHandle &PoseCommitBaseSlots();
const rigExec::RigExecPortHandle &PoseCommitBaseDescendants();
const rigExec::RigExecPortHandle &PoseCommitCandidate();
const rigExec::RigExecPortHandle &PoseCommitSources();
const rigExec::RigExecPortHandle &PoseCommitStatus();
const rigExec::RigExecPortHandle &PoseCommitPresence();

} // namespace rigExecOps

#endif
