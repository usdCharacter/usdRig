// USD-free mover callbacks. Registration binds these before graph execution.
#ifndef RIGEXEC_BINARY_MOVER_DEFINITION_H
#define RIGEXEC_BINARY_MOVER_DEFINITION_H

#include "external.h"

#include <type_traits>
#include <utility>

namespace rigExec {

struct RigExecMoverDefinition {
    const char *schemaType;
    RigExecExternalKernel kernel;
};
using RigExecMoverFactory = RigExecMoverDefinition (*)();

namespace mover_definition_detail {

template <class T, class = void> struct HasProviders : std::false_type {};
template <class T> struct HasProviders<T,
    std::void_t<decltype(&T::ApplyWithProviders)>> : std::true_type {};
template <class T, class = void> struct HasCapture : std::false_type {};
template <class T> struct HasCapture<T,
    std::void_t<decltype(&T::CaptureWindow)>> : std::true_type {};
template <class T, class = void> struct HasWindow : std::false_type {};
template <class T> struct HasWindow<T,
    std::void_t<decltype(&T::ApplyWindow)>> : std::true_type {};

template <class Definition> struct Adapter {
    using State = typename Definition::State;

    static std::shared_ptr<const void> Prepare(
        const uint8_t *data, size_t size, std::string *error)
    {
        static_assert(std::is_same_v<decltype(Definition::Prepare(data, size, error)),
            std::shared_ptr<const State>>,
            "A mover prepares immutable, typed state");
        return Definition::Prepare(data, size, error);
    }

    static bool Apply(const void *state, const uint8_t *frame, size_t frameSize,
        const RigExecExternalPhasedPoints *phased, size_t phasedCount,
        const RigExecExternalInputValue *inputs, size_t inputCount,
        float *xyz, size_t pointCount)
    {
        return state && Definition::Apply(*static_cast<const State *>(state),
            frame, frameSize, phased, phasedCount, inputs, inputCount, xyz, pointCount);
    }

    static bool ApplyWithProviders(const void *state, const uint8_t *frame,
        size_t frameSize, const RigExecExternalPhasedPoints *phased,
        size_t phasedCount, const RigExecExternalInputValue *inputs,
        size_t inputCount, const RigExecExternalProviders &providers,
        float *xyz, size_t pointCount)
    {
        return state && Definition::ApplyWithProviders(
            *static_cast<const State *>(state), frame, frameSize, phased,
            phasedCount, inputs, inputCount, providers, xyz, pointCount);
    }

    static bool CaptureWindow(const void *state,
        const RigExecExternalPhasedPoints *phased, size_t phasedCount,
        const RigExecExternalInputValue *inputs, size_t inputCount,
        const float *xyz, size_t pointCount, RigExecWindowPacket *packet)
    {
        return state && Definition::CaptureWindow(*static_cast<const State *>(state),
            phased, phasedCount, inputs, inputCount, xyz, pointCount, packet);
    }

    static bool ApplyWindow(const void *state,
        const RigExecWindowPacket *const *ticks, size_t tickCount,
        float *xyz, size_t pointCount)
    {
        return state && Definition::ApplyWindow(*static_cast<const State *>(state),
            ticks, tickCount, xyz, pointCount);
    }
};
} // namespace mover_definition_detail

/// Definition supplies State, Prepare and Apply (or ApplyWithProviders).
/// Windowed definitions supply both CaptureWindow and ApplyWindow. All apply
/// functions are pure graph bodies; the engine owns envelopes and publication.
/// The prepared state's decoder owns payload version validation.
template <class Definition>
constexpr RigExecExternalKernel RigExecMakeMoverKernel()
{
    using namespace mover_definition_detail;
    using A = Adapter<Definition>;
    static_assert(HasCapture<Definition>::value == HasWindow<Definition>::value,
        "A windowed mover needs both capture and solve callbacks");
    RigExecExternalKernel kernel;
    kernel.prepare = &A::Prepare;
    if constexpr (HasProviders<Definition>::value)
        kernel.applyWithProviders = &A::ApplyWithProviders;
    else
        kernel.apply = &A::Apply;
    if constexpr (HasCapture<Definition>::value) {
        kernel.captureWindow = &A::CaptureWindow;
        kernel.applyWindow = &A::ApplyWindow;
    }
    return kernel;
}

} // namespace rigExec
#endif
