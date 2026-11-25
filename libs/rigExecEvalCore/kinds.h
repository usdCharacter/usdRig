// The step kinds EvalCore runs, in the ordinals the wire schema and the baked
// program already agree on: rigexec.fbs declares its StepKind as
// "RigExecBakedStepKind, in the same order". Nothing here renames or
// renumbers a kind.
#ifndef RIGEXEC_EVAL_CORE_KINDS_H
#define RIGEXEC_EVAL_CORE_KINDS_H

#include <cstddef>
#include <cstdint>

// The step-kind enum comes from the generated wire schema, whose Pack/UnPack
// helpers format.h declares before including it.
#include "rigExecBinary/format.h"

namespace rigExec {
namespace eval {

/// The wire ordinal of a step; the same spelling the baked enum uses.
using StepKind = RigExecWireStepKind;

/// Every declared ordinal, the reserved one included.
constexpr std::size_t StepKindCount =
    static_cast<std::size_t>(fb::StepKind::OpChainRange) + 1;

/// The kind's name, spelled by the schema the wire enum is generated from.
const char *StepKindName(StepKind kind);

/// The retired ordinals no step may declare: the wire reader refuses a file
/// that holds one.
bool IsReservedStepKind(StepKind kind);

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_KINDS_H
