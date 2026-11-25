// What a property chain's op nodes read per frame and where their values
// go: the baked program, its frozen clones and the runtime resolve the same
// reads and keep the same version slots.
#ifndef RIGEXEC_EVAL_CORE_PROPERTY_LOOKUPS_H
#define RIGEXEC_EVAL_CORE_PROPERTY_LOOKUPS_H

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rigExec {
namespace eval {

/// OpLookup::source of a property chain read: object is the chain, part
/// the revision (-1 for the chain's base) and read a PropertyRead.
constexpr uint8_t kPropertyLookup = 0;
/// OpHostOutput::target of a property version: object is its slot.
constexpr uint8_t kPropertyVersion = 0;

enum class PropertyRead : uint8_t {
    /// A drag on the target converted to the chain's type, else the
    /// target's own value; finite or the evaluation fails.
    Base,
    /// The revision's mover exists and inputs:enabled (true unless read).
    Enabled,
    /// The one value of the weight object's field, else
    /// inputs:defaultWeight (1 unless read).
    Weight,
    Value,
    Minimum,
    Maximum,
    Keys,
    Tangents,
    PathPoints,
};

/// Whether a chain's value is usable: a double chain computes in float, so
/// it is checked as the float it becomes.
inline bool
PropertyFinite(float value)
{
    return std::isfinite(value);
}
inline bool
PropertyFinite(double value)
{
    return std::isfinite(float(value));
}
template <class E>
bool
PropertyFinite(const E *elements, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        if (!std::isfinite(elements[i]))
            return false;
    return true;
}

} // namespace eval
} // namespace rigExec

#endif
