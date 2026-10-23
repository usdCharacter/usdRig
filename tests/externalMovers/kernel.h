#ifndef RIGEXEC_TEST_QUADRATIC_KERNEL_H
#define RIGEXEC_TEST_QUADRATIC_KERNEL_H
#include "rigExecBinary/external.h"

namespace rigExec {
struct TestQuadraticDefinition {
    struct State {};
    inline static constexpr char CompiledTag[] = {'Q', 'U', 'A', '1'};
    static bool Deform(float gain, float referenceY, float *xyz, size_t count);
    static std::shared_ptr<const State> Prepare(
        const uint8_t *data, size_t size, std::string *error);
    static bool Apply(const State &, const uint8_t *frame, size_t frameSize,
        const RigExecExternalPhasedPoints *phased, size_t phasedCount,
        const RigExecExternalInputValue *inputs, size_t inputCount,
        float *xyz, size_t count);
};
}
#endif
