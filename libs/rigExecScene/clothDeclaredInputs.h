#ifndef RIGEXEC_SCENE_CLOTH_DECLARED_INPUTS_H
#define RIGEXEC_SCENE_CLOTH_DECLARED_INPUTS_H

// What a cloth mover's declared inputs (rigExecBinary/clothPlayback.h's
// RigExecClothInput) hold, read the same way by the native assembly and the
// scene's op binding: the per-tick parameters and the collider count.

#include "rigExecBinary/clothPlayback.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/base/vt/value.h"

#include <array>
#include <cstdint>
#include <vector>

namespace rigExec {

/// \p value as a float, from a float or a double; \p fallback otherwise.
inline float
RigExecClothFloatOr(const PXR_NS::VtValue &value, float fallback)
{
    if (value.IsHolding<float>())
        return value.UncheckedGet<float>();
    if (value.IsHolding<double>())
        return float(value.UncheckedGet<double>());
    return fallback;
}

/// The parameters in packet order: stretch and bend compliance, gravity
/// xyz, damping, attract, thickness, collision depth and friction, each
/// its declared input or the schema fallback. \p in holds at least
/// RigExecClothFirstCollider inputs.
inline std::array<float, RigExecClothParameterCount>
RigExecClothParameterValues(const std::vector<PXR_NS::VtValue> &in)
{
    const RigExecClothParameters fallback;
    std::array<float, RigExecClothParameterCount> p;
    p[0] = RigExecClothFloatOr(in[RigExecClothStretchCompliance],
                               fallback.stretchCompliance);
    p[1] = RigExecClothFloatOr(in[RigExecClothBendCompliance],
                               fallback.bendCompliance);
    PXR_NS::GfVec3f gravity(fallback.gravity[0], fallback.gravity[1],
                            fallback.gravity[2]);
    if (in[RigExecClothGravity].IsHolding<PXR_NS::GfVec3f>())
        gravity = in[RigExecClothGravity].UncheckedGet<PXR_NS::GfVec3f>();
    p[2] = gravity[0];
    p[3] = gravity[1];
    p[4] = gravity[2];
    p[5] = RigExecClothFloatOr(in[RigExecClothDamping], fallback.damping);
    p[6] = RigExecClothFloatOr(in[RigExecClothAttract], fallback.attract);
    p[7] = RigExecClothFloatOr(in[RigExecClothThickness], fallback.thickness);
    p[8] = RigExecClothFloatOr(in[RigExecClothCollisionDepth],
                               fallback.collisionDepth);
    p[9] = RigExecClothFloatOr(in[RigExecClothFriction], fallback.friction);
    return p;
}

/// The declared-input layout a binding's compile data carries, or null for
/// a binding declared without one.
inline const PXR_NS::VtDictionary *
RigExecClothLayout(const PXR_NS::VtValue &compileData)
{
    if (!compileData.IsHolding<PXR_NS::VtDictionary>())
        return nullptr;
    const auto &compiled = compileData.UncheckedGet<PXR_NS::VtDictionary>();
    const auto found = compiled.find("layout");
    return found != compiled.end() &&
                   found->second.IsHolding<PXR_NS::VtDictionary>()
               ? &found->second.UncheckedGet<PXR_NS::VtDictionary>()
               : nullptr;
}

/// How many colliders \p inputCount declared inputs hold: the count
/// \p layout records, or without one every input past the parameters, four
/// per collider. A layout without a count answers SIZE_MAX.
inline size_t
RigExecClothColliderCount(const PXR_NS::VtDictionary *layout, size_t inputCount)
{
    if (!layout)
        return (inputCount - RigExecClothFirstCollider) /
               RigExecClothInputsPerCollider;
    const auto found = layout->find("colliderCount");
    return found != layout->end() && found->second.IsHolding<int>()
               ? size_t(found->second.UncheckedGet<int>())
               : SIZE_MAX;
}

} // namespace rigExec

#endif
