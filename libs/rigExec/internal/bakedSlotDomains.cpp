#include "rigExec/internal/bakedSlotDomains.h"

#include <iterator>
#include <limits>

namespace rigExec {

namespace {

using D = RigExecBakedSlotDomain;

constexpr RigExecBakedSlotDomainOps kDomains[] = {
    {D::Avars, "Avars", &RigExecBakedResetAvars, nullptr},
    {D::PoseBase, "PoseBase", &RigExecBakedResetPoseBase,
     &RigExecBakedPathIdentity},
    {D::PoseFin, "PoseFin", &RigExecBakedResetPoseFin,
     &RigExecBakedPathIdentity},
    {D::PosedM, "PosedM", &RigExecBakedResetPosedM, &RigExecBakedPathIdentity},
    {D::FinalMatrix, "FinalMatrix", &RigExecBakedResetFinalMatrix,
     &RigExecBakedPathIdentity},
    {D::BaseMatrix, "BaseMatrix", &RigExecBakedResetBaseMatrix,
     &RigExecBakedPathIdentity},
    {D::Aggregate, "Aggregate", &RigExecBakedResetAggregate, nullptr},
    {D::SolverPoints, "SolverPoints", nullptr, nullptr},
    {D::Candidates, "Candidates", &RigExecBakedResetCandidates, nullptr},
    {D::CommitTable, "CommitTable", &RigExecBakedResetCommitTable, nullptr},
    {D::CommitDelta, "CommitDelta", &RigExecBakedResetCommitDelta, nullptr},
    {D::CommitStaging, "CommitStaging", &RigExecBakedResetCommitStaging,
     nullptr},
    {D::ConstraintDelta, "ConstraintDelta", &RigExecBakedResetConstraintDelta,
     nullptr},
    {D::PropertyResult, "PropertyResult", &RigExecBakedResetPropertyResult,
     nullptr},
    {D::ChainBase, "ChainBase", &RigExecBakedResetChainBase,
     &RigExecBakedChainIdentity},
    {D::RevisionPacket, "RevisionPacket", &RigExecBakedResetRevisionPacket,
     &RigExecBakedRevisionIdentity},
    {D::RevisionTransforms, "RevisionTransforms",
     &RigExecBakedResetRevisionTransforms, &RigExecBakedRevisionIdentity},
    {D::RevisionOut, "RevisionOut", &RigExecBakedResetRevisionOut,
     &RigExecBakedRevisionOutIdentity},
    {D::RevisionDone, "RevisionDone", &RigExecBakedResetRevisionDone,
     &RigExecBakedRevisionIdentity},
    {D::ChainDirty, "ChainDirty", &RigExecBakedResetRevisionDone,
     &RigExecBakedRevisionIdentity},
    {D::ChainPoints, "ChainPoints", &RigExecBakedResetChainPoints,
     &RigExecBakedChainIdentity},
    {D::DerivedOut, "DerivedOut", &RigExecBakedResetDerivedOut, nullptr},
    {D::WeightPacket, "WeightPacket", &RigExecBakedResetWeightPacket, nullptr},
    {D::WeightFrames, "WeightFrames", &RigExecBakedResetWeightFrames, nullptr},
    {D::PoseWeight, "PoseWeight", &RigExecBakedResetPoseWeight, nullptr},
    {D::Snapshots, "Snapshots", nullptr, nullptr},
    {D::FrameMatrix, "FrameMatrix", &RigExecBakedResetFrameMatrix, nullptr},
    {D::Rest, "Rest", &RigExecBakedResetRest, nullptr},
    {D::Ladder, "Ladder", &RigExecBakedResetLadder, nullptr},
    {D::SkinTopology, "SkinTopology", &RigExecBakedResetSkinTopology,
     &RigExecBakedRevisionIdentity},
    {D::WeightField, "WeightField", &RigExecBakedResetWeightFieldSlot, nullptr},
    {D::WeightFramesBase, "WeightFramesBase",
     &RigExecBakedResetWeightFramesBase, nullptr},
    {D::SpaceValue, "SpaceValue", &RigExecBakedResetSpaceValue, nullptr},
    {D::SpaceLeaf, "SpaceLeaf", nullptr, nullptr},
    {D::DerivedBase, "DerivedBase", nullptr, nullptr},
    {D::ChainInput, "ChainInput", nullptr, &RigExecBakedChainIdentity},
    {D::ConstraintInputs, "ConstraintInputs", nullptr, nullptr},
    {D::SwitchFrame, "SwitchFrame", &RigExecBakedResetSwitchFrame, nullptr},
    {D::RequiredStageFramesAdmission, "RequiredStageFramesAdmission", nullptr,
     &RigExecBakedStageFramesIdentity},
    {D::OpValue, "OpValue", nullptr, nullptr},
    {D::OpGroupValue, "OpGroupValue", nullptr, nullptr},
};

constexpr RigExecBakedSlotDomainOps kNoDomain = {D(0xff), "unknown", nullptr,
                                                 nullptr};

constexpr bool
InDomainOrder()
{
    for (size_t i = 0; i < std::size(kDomains); ++i)
        if (size_t(kDomains[i].domain) != i)
            return false;
    return true;
}

static_assert(std::size(kDomains) == RigExecBakedSlotDomainCount,
              "one entry per slot domain");
static_assert(InDomainOrder(), "entries in domain order");

} // namespace

const RigExecBakedSlotDomainOps &
RigExecBakedSlotDomainOpsOf(RigExecBakedSlotDomain domain)
{
    return size_t(domain) < std::size(kDomains) ? kDomains[size_t(domain)]
                                                : kNoDomain;
}

const char *
RigExecBakedSlotDomainName(RigExecBakedSlotDomain domain)
{
    return RigExecBakedSlotDomainOpsOf(domain).name;
}

GfMatrix4d
RigExecBakedSetAsideMatrix()
{
    GfMatrix4d matrix(1);
    matrix[3][0] = std::numeric_limits<double>::quiet_NaN();
    return matrix;
}

} // namespace rigExec
