#include "kinds.h"

namespace rigExec {
namespace eval {

// The ordinals the shared contract names, pinned against the schema.
static_assert(static_cast<uint8_t>(fb::StepKind::ComposeSubtree) == 0,
              "ComposeSubtree is ordinal 0");
static_assert(static_cast<uint8_t>(fb::StepKind::SnapshotFinals) == 8,
              "the reserved ordinal is 8");
static_assert(static_cast<uint8_t>(fb::StepKind::PropertyRevision) == 19,
              "PropertyRevision keeps its ordinal");
static_assert(static_cast<uint8_t>(fb::StepKind::ChainInputs) == 25,
              "ChainInputs keeps its ordinal");
static_assert(static_cast<uint8_t>(fb::StepKind::AvarInputs) == 26,
              "AvarInputs keeps its ordinal");
static_assert(static_cast<uint8_t>(fb::StepKind::ProviderRefresh) == 28,
              "ProviderRefresh is the last ordinal");

const char *
StepKindName(StepKind kind)
{
    return fb::EnumNameStepKind(kind);
}

bool
IsReservedStepKind(StepKind kind)
{
    return kind == fb::StepKind::SnapshotFinals;
}

} // namespace eval
} // namespace rigExec
