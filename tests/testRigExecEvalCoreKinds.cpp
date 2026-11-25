// Witness that EvalCore's step kinds are the ordinals the baked program and
// the wire schema already agree on: same numbers, same spellings, same
// reserved ordinal.
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecEvalCore/kinds.h"
#include "rigExecEvalCore/step.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstddef>

using namespace rigExec;
namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

void
CheckOrdinals()
{
    CHECK(eval::StepKindCount == 33);
    for (size_t i = 0; i < eval::StepKindCount; ++i) {
        eval::StepKind kind = static_cast<eval::StepKind>(i);
        RigExecBakedStepKind baked = static_cast<RigExecBakedStepKind>(i);
        CHECK(static_cast<uint8_t>(kind) == static_cast<uint8_t>(baked));
        // Ordinal 10 is retired on the wire; VolumePlacement op nodes keep
        // the baked name as their identity.
        if (kind == eval::StepKind::ReservedVolumePlacements) {
            CHECK(std::strcmp(eval::StepKindName(kind),
                              "ReservedVolumePlacements") == 0);
            CHECK(std::strcmp(RigExecBakedStepKindName(baked),
                              "VolumePlacements") == 0);
            continue;
        }
        if (kind == eval::StepKind::ReservedWeightPacket) {
            CHECK(std::strcmp(eval::StepKindName(kind),
                              "ReservedWeightPacket") == 0);
            CHECK(std::strcmp(RigExecBakedStepKindName(baked),
                              "WeightPacket") == 0);
            continue;
        }
        if (kind == eval::StepKind::ReservedWeightField) {
            CHECK(std::strcmp(eval::StepKindName(kind),
                              "ReservedWeightField") == 0);
            CHECK(std::strcmp(RigExecBakedStepKindName(baked), "WeightField") ==
                  0);
            continue;
        }
        CHECK(std::strcmp(eval::StepKindName(kind),
                          RigExecBakedStepKindName(baked)) == 0);
    }
    CHECK(eval::IsReservedStepKind(eval::StepKind::ReservedVolumePlacements));
    CHECK(eval::IsReservedStepKind(eval::StepKind::ReservedWeightPacket));
    CHECK(eval::IsReservedStepKind(eval::StepKind::ReservedWeightField));
    CHECK(eval::IsReservedStepKind(eval::StepKind::SnapshotFinals));
    CHECK(eval::IsReservedStepKind(eval::StepKind::ReservedOpGraph));
    CHECK(!eval::IsReservedStepKind(eval::StepKind::ComposeSubtree));
    CHECK(eval::StepKindName(
              static_cast<eval::StepKind>(eval::StepKindCount))[0] == '\0');
    CHECK(static_cast<uint8_t>(eval::StepKind::ChainInputs) == 25);
    CHECK(static_cast<uint8_t>(eval::StepKind::AvarInputs) == 26);
    CHECK(static_cast<uint8_t>(eval::StepKind::ProviderRefresh) == 28);
    CHECK(static_cast<uint8_t>(eval::StepKind::ReservedOpGraph) == 29);
    CHECK(static_cast<uint8_t>(eval::StepKind::OpNode) == 30);
    CHECK(static_cast<uint8_t>(eval::StepKind::OpChainRange) == 32);
}

void
CheckStepShape()
{
    eval::StepRef step{static_cast<uint32_t>(eval::StepKind::Solve), 3, 1, 7};
    eval::StepResult result{eval::Outcome::Held, 0};
    CHECK(step.kind == static_cast<uint32_t>(eval::StepKind::Solve));
    CHECK(result.outcome == eval::Outcome::Held);
    CHECK(result.detail == 0);
}
} // namespace
int
main()
{
    CheckOrdinals();
    CheckStepShape();
    std::printf("EvalCoreKinds: %d failures\n", failures);
    return failures ? 1 : 0;
}
