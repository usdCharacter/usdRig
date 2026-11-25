#include "kinds.h"

namespace rigExec {
namespace eval {

// The ordinals the shared contract names, pinned against the schema.
static_assert(static_cast<uint8_t>(fb::StepKind::ComposeSubtree) == 0,
              "ComposeSubtree is ordinal 0");
static_assert(static_cast<uint8_t>(fb::StepKind::SnapshotFinals) == 8,
              "the reserved ordinal is 8");
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
static_assert(static_cast<uint8_t>(fb::StepKind::ChainInputs) == 25,
              "ChainInputs keeps its ordinal");
static_assert(static_cast<uint8_t>(fb::StepKind::AvarInputs) == 26,
              "AvarInputs keeps its ordinal");
static_assert(static_cast<uint8_t>(fb::StepKind::ProviderRefresh) == 28,
              "ProviderRefresh keeps its ordinal");
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
           kind == fb::StepKind::ReservedOpGraph;
}

} // namespace eval
} // namespace rigExec
