#include "kernel.h"
#include <cstring>
#include <limits>

namespace rigExec {
bool TestQuadraticDefinition::Deform(float gain, float referenceY, float *xyz, size_t count)
{
    if (gain == -3.0f && count > 0) {
        xyz[2] = std::numeric_limits<float>::quiet_NaN();
        return true;
    }
    for (size_t i = 0; i < count; ++i) {
        xyz[i * 3 + 1] += referenceY;
        xyz[i * 3 + 2] += gain * xyz[i * 3] * xyz[i * 3];
    }
    return gain >= 0.0f;
}

std::shared_ptr<const TestQuadraticDefinition::State>
TestQuadraticDefinition::Prepare(const uint8_t *data, size_t size, std::string *error)
{
    if (size != sizeof(CompiledTag) || !data || std::memcmp(data, CompiledTag, size) != 0) {
        if (error) *error = "unknown compiled rig format";
        return nullptr;
    }
    return std::make_shared<const State>();
}

bool TestQuadraticDefinition::Apply(const State &, const uint8_t *frame, size_t frameSize,
    const RigExecExternalPhasedPoints *phased, size_t phasedCount,
    const RigExecExternalInputValue *inputs, size_t inputCount, float *xyz, size_t count)
{
    float payload[3];
    if (frameSize != sizeof(payload) || !frame) return false;
    std::memcpy(payload, frame, sizeof(payload));
    if (inputCount > 0) {
        if (inputs[0].type != uint8_t(RigExecExternalInputType::Float) ||
            !inputs[0].hasValue || !inputs[0].data || inputs[0].count != 1) return false;
        payload[0] = *static_cast<const float *>(inputs[0].data);
    }
    if (inputCount > 1) {
        if (inputs[1].type != uint8_t(RigExecExternalInputType::Vec3fArray) ||
            !inputs[1].hasValue || !inputs[1].data || inputs[1].count == 0) return false;
        payload[1] = static_cast<const float *>(inputs[1].data)[1];
        payload[2] = 1.0f;
    }
    if (payload[0] == -2.0f) return false;
    if (payload[2] != 0.0f && phasedCount == 1 && phased[0].xyz && phased[0].count > 0)
        payload[1] = phased[0].xyz[1];
    return Deform(payload[0], payload[1], xyz, count);
}
}
