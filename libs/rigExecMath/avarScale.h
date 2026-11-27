// Shared normalization for RigExecControl/RigExecJoint avars:sx/sy/sz.
// Kept in the common math layer so evaluation, authoring, and imaging apply
// one identical scale policy. Other avars require no analogous normalization.
#ifndef RIGEXEC_MATH_AVAR_SCALE_H
#define RIGEXEC_MATH_AVAR_SCALE_H

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rigExec {

/// Smallest supported magnitude for one control/joint avar scale channel.
///
/// This remains well above the point-frame degeneracy epsilon (1e-10 for a
/// unit rest frame), while still representing a visually collapsed axis.
inline constexpr double RigExecAvarScaleFloor = 1e-4;

/// True when strict authoring may accept this scale value.
inline bool
RigExecIsFiniteAvarScale(double value)
{
    return std::isfinite(value);
}

/// Apply the runtime avar-scale contract.
///
/// Finite magnitudes below RigExecAvarScaleFloor are raised to the floor with
/// their sign intact (including negative zero), because negative scale is a
/// supported reflection. Non-finite raw USD values cannot describe a usable
/// transform and resolve to identity scale. Strict authoring rejects those
/// values before they reach the stage.
inline double
RigExecNormalizeAvarScale(double value)
{
    if (!RigExecIsFiniteAvarScale(value)) {
        return 1.0;
    }
    return std::abs(value) < RigExecAvarScaleFloor
               ? std::copysign(RigExecAvarScaleFloor, value)
               : value;
}

/// Apply the runtime avars:rotationSign contract to one axis.
///
/// The channel carries only a sign, so the magnitude is discarded: negative
/// selects -1, everything else -- including zero, which would otherwise erase
/// an axis, and a non-finite value -- selects +1. The kernel is total so that
/// a malformed value degrades to the unmirrored axis rather than collapsing
/// the pose; strict authoring rejects anything but +1/-1 before it reaches
/// the stage.
inline double
RigExecNormalizeRotationSign(double value)
{
    return (std::isfinite(value) && value < 0.0) ? -1.0 : 1.0;
}

/// Pack one avars:rotationSign triple into the three low bits of a byte,
/// bit N set meaning axis N is negated. The baked program, the wire record
/// and the zero-USD runtime carry the sign in this form: it is one byte per
/// provider slot, and it survives the round trip exactly.
inline unsigned char
RigExecRotationSignMask(double x, double y, double z)
{
    return (unsigned char)((RigExecNormalizeRotationSign(x) < 0.0 ? 1u : 0u) |
                           (RigExecNormalizeRotationSign(y) < 0.0 ? 2u : 0u) |
                           (RigExecNormalizeRotationSign(z) < 0.0 ? 4u : 0u));
}

/// The sign one packed axis selects: -1 when its bit is set, +1 otherwise.
inline double
RigExecRotationSignFromMask(unsigned mask, int axis)
{
    return (mask & (1u << axis)) ? -1.0 : 1.0;
}

/// Writes \p count bound avar values in order: binding k writes value(k) to
/// slot(k), so a later binding for the same slot wins.
template <class Slot, class Value, class Write>
void
RigExecWriteAvarBindings(size_t count, const Slot &slot, const Value &value,
                         const Write &write)
{
    for (size_t k = 0; k < count; ++k) {
        write(slot(k), value(k));
    }
}

/// The avars of one provider slot: avar slot provider * 11 + channel.
inline constexpr size_t kRigExecSlotAvars = 11;

/// The channels \p count bindings of one provider write, bit c for channel
/// c, \p slot(k) binding k's avar slot.
template <class Slot>
unsigned
RigExecBoundAvarChannels(size_t count, const Slot &slot)
{
    unsigned mask = 0;
    for (size_t k = 0; k < count; ++k)
        mask |= 1u << (size_t(slot(k)) % kRigExecSlotAvars);
    return mask;
}

/// Channel \p channel's element in an AvarInputs output over the channels
/// \p mask holds: its rank among them.
inline uint32_t
RigExecAvarElement(unsigned mask, size_t channel)
{
    uint32_t element = 0;
    for (size_t c = 0; c < channel; ++c)
        element += (mask >> c) & 1u;
    return element;
}

} // namespace rigExec

#endif // RIGEXEC_MATH_AVAR_SCALE_H
