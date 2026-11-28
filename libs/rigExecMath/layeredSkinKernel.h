#ifndef RIGEXEC_MATH_LAYERED_SKIN_KERNEL_H
#define RIGEXEC_MATH_LAYERED_SKIN_KERNEL_H

/// The layered skin mover's arithmetic: points skinned (linear or dual
/// quaternion) from their entering or base positions, the entering points
/// first carried by an input transform, then blended toward the skinned
/// result by a per-point mask. One definition for the native mover and the
/// LayeredSkin op. V is a float point, D its double twin and M a double
/// 4x4, each with Gf's operators and members (GfVec3f, GfVec3d and
/// GfMatrix4d, or the runtime's RrVec3f, RrVec3d and RrMat4d).

#include "rigExecMath/skinKernel.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

/// Whether \p indexCount joint indices and \p weightCount weights with
/// \p elementSize slots per point, and \p maskCount mask weights, skin
/// \p targetCount points through \p transformCount influences: the
/// weights as long as the indices, a mask empty or one finite weight in
/// [0, 1] per point, and the layout valid (RigExecValidateSkinLayoutKernel).
template <class M>
bool
RigExecLayeredSkinAccepts(const M *transforms, size_t transformCount,
                          const int *indices, size_t indexCount,
                          const float *weights, size_t weightCount,
                          int elementSize, const float *mask, size_t maskCount,
                          size_t targetCount)
{
    if (weightCount != indexCount || elementSize < 1 ||
        (maskCount != 0 && maskCount != targetCount))
        return false;
    for (size_t i = 0; i < maskCount; ++i)
        if (!std::isfinite(mask[i]) || mask[i] < 0 || mask[i] > 1)
            return false;
    const RigExecSkinLayoutOf<M> layout{
        transforms, transformCount,      indices,    weights,
        indexCount, size_t(elementSize), targetCount};
    return RigExecValidateSkinLayoutKernel(layout);
}

/// Skins the \p count \p points through \p layout, from \p base when it
/// holds \p count points and from the entering points when \p baseCount
/// is 0; carries the entering points by \p inputTransform; then blends
/// each toward its skinned position by mask[i], or replaces it when
/// \p maskCount is 0. False, \p points untouched, where the base, the mask
/// or the layout does not fit, or the dual quaternion blend is degenerate.
template <class V, class D, class M, class L>
bool
RigExecApplyLayeredSkinKernel(const L &layout, bool dualQuaternion,
                              const V *base, size_t baseCount,
                              const float *mask, size_t maskCount,
                              const M &inputTransform, V *points, size_t count)
{
    if ((baseCount != 0 && baseCount != count) ||
        (maskCount != 0 && maskCount != count) || layout.pointCount != count ||
        !RigExecValidateSkinLayoutKernel(layout))
        return false;
    std::vector<V> candidate(count);
    const V *input = baseCount == 0 ? points : base;
    if (dualQuaternion) {
        const auto palette = RigExecSkinDualQuatPaletteKernel<V, D>(
            layout.transforms, layout.transformCount);
        if (!RigExecApplyDualQuatSkinKernel<V, D>(input, candidate.data(),
                                                  layout, palette.data(),
                                                  palette.size()))
            return false;
    } else {
        RigExecApplyLinearBlendSkinKernel<V, D>(input, candidate.data(),
                                                layout);
    }
    for (size_t i = 0; i < count; ++i) {
        const V &p = points[i];
        const D carried = inputTransform.Transform(
            D(double(p[0]), double(p[1]), double(p[2])));
        points[i] = V(float(carried[0]), float(carried[1]), float(carried[2]));
        const float w = maskCount == 0 ? 1.0f : mask[i];
        if (w == 1)
            points[i] = candidate[i];
        else if (w != 0)
            points[i] += w * (candidate[i] - points[i]);
    }
    return true;
}

} // namespace rigExec

#endif
