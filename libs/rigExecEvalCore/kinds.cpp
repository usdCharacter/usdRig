#include "kinds.h"

namespace rigExec {
namespace eval {

// The ordinals the shared contract names, pinned against the schema.
static_assert(static_cast<uint8_t>(fb::StepKind::ComposeSubtree) == 0,
              "ComposeSubtree is ordinal 0");
static_assert(static_cast<uint8_t>(fb::StepKind::SnapshotFinals) == 8,
              "the reserved ordinal is 8");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedProviderMatrix) == 7,
              "the retired provider matrix ordinal is 7");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedFrameMatrix) == 18,
              "the retired frame matrix ordinal is 18");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedVolumePlacements) ==
                  10,
              "the retired volume placement ordinal is 10");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedWeightPacket) == 11,
              "the retired weight packet ordinal is 11");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedWeightField) == 23,
              "the retired weight field ordinal is 23");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedPropertyRevision) ==
                  19,
              "the retired property revision ordinal is 19");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedSpaceExpression) == 24,
              "the retired space expression ordinal is 24");
static_assert(static_cast<uint8_t>(fb::StepKind::ChainInputs) == 25,
              "ChainInputs keeps its ordinal");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedAvarInputs) == 26,
              "the retired avar inputs ordinal is 26");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedSpaceCheckpoint) == 27,
              "the retired space checkpoint ordinal is 27");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedPoseInterpolator) == 9,
              "the retired pose interpolator ordinal is 9");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedSolverCommit) == 2 &&
                  static_cast<uint8_t>(fb::StepKind::ReservedCommitDelta) ==
                      4 &&
                  static_cast<uint8_t>(fb::StepKind::ReservedPropagateChunk) ==
                      5 &&
                  static_cast<uint8_t>(fb::StepKind::ReservedCommitApply) == 6,
              "the retired commit ordinals are 2, 4, 5 and 6");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedProviderRefresh) == 28,
              "the retired provider refresh ordinal is 28");
static_assert(static_cast<uint8_t>(fb::StepKind::ReservedOpGraph) == 29,
              "the retired op graph ordinal is 29");
static_assert(static_cast<uint8_t>(fb::StepKind::OpNode) == 30 &&
                  static_cast<uint8_t>(fb::StepKind::OpChainHead) == 31 &&
                  static_cast<uint8_t>(fb::StepKind::OpChainRange) == 32,
              "the op step kinds are 30, 31 and 32 (eval::OpStepKind)");

const char *
StepKindName(StepKind kind)
{
    return fb::EnumNameStepKind(kind);
}

bool
IsReservedStepKind(StepKind kind)
{
    return kind == fb::StepKind::SnapshotFinals ||
           kind == fb::StepKind::ReservedVolumePlacements ||
           kind == fb::StepKind::ReservedWeightPacket ||
           kind == fb::StepKind::ReservedWeightField ||
           kind == fb::StepKind::ReservedPropertyRevision ||
           kind == fb::StepKind::ReservedOpGraph ||
           kind == fb::StepKind::ReservedProviderMatrix ||
           kind == fb::StepKind::ReservedFrameMatrix ||
           kind == fb::StepKind::ReservedSolverCommit ||
           kind == fb::StepKind::ReservedCommitDelta ||
           kind == fb::StepKind::ReservedPropagateChunk ||
           kind == fb::StepKind::ReservedCommitApply ||
           kind == fb::StepKind::ReservedProviderRefresh ||
           kind == fb::StepKind::ReservedSpaceExpression ||
           kind == fb::StepKind::ReservedAvarInputs ||
           kind == fb::StepKind::ReservedSpaceCheckpoint ||
           kind == fb::StepKind::ReservedPoseInterpolator;
}

} // namespace eval
} // namespace rigExec
