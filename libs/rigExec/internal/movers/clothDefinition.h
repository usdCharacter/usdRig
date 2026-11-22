#ifndef RIGEXEC_MOVERS_CLOTH_DEFINITION_H
#define RIGEXEC_MOVERS_CLOTH_DEFINITION_H

#include "rigExecBinary/clothPlayback.h"

namespace rigExec {
// The USD adapter and standalone runtime instantiate this same definition.
struct RigExecClothDefinition {
    using State = cloth_playback_detail::State;
    static std::shared_ptr<const State> Prepare(
        const uint8_t *data, size_t size, std::string *error)
    {
        return std::static_pointer_cast<const State>(
            cloth_playback_detail::Prepare(data, size, error));
    }
    static bool Apply(const State &, const uint8_t *, size_t,
        const RigExecExternalPhasedPoints *, size_t,
        const RigExecExternalInputValue *, size_t, float *, size_t)
    {
        return true;
    }
    static bool CaptureWindow(const State &state,
        const RigExecExternalPhasedPoints *phased, size_t phasedCount,
        const RigExecExternalInputValue *inputs, size_t inputCount,
        const float *xyz, size_t pointCount, RigExecWindowPacket *packet)
    {
        return cloth_playback_detail::Capture(&state, phased, phasedCount,
            inputs, inputCount, xyz, pointCount, packet);
    }
    static bool ApplyWindow(const State &state,
        const RigExecWindowPacket *const *ticks, size_t tickCount,
        float *xyz, size_t pointCount)
    {
        return cloth_playback_detail::ApplyWindow(&state, ticks, tickCount, xyz, pointCount);
    }
};
}
#endif
