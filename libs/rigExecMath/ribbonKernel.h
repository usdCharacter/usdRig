// Rest-relative ribbon transport, shared by the native (Gf) path, the
// zero-USD runtime (Rr) and the Ribbon op. Header-only and USD-free.
#ifndef RIGEXEC_MATH_RIBBON_KERNEL_H
#define RIGEXEC_MATH_RIBBON_KERNEL_H

#include "matrixKernel.h"

#include <algorithm>
#include <cstddef>
#include <type_traits>
#include <vector>

namespace rigExec {

/// Moves each point by the rigid map taking its rest frame sample to its
/// posed frame sample, p' = M(u) p, lerped between the two samples around
/// u = bind[0] in [0, 1]. \p restFrame(k) and \p posedFrame(k) give sample
/// k's four frame points (origin, origin + tangent, + normal, + binormal)
/// in double; Math::PointsToMatrix solves each map, an unsolvable one is
/// identity. False, writing nothing, without a bind per point or two
/// samples.
template <class Math, class V, class B, class RestFrame, class PosedFrame>
bool
RigExecApplyRibbonKernel(V *points, size_t count, const B *bindCoords,
                         size_t bindCount, size_t sampleCount,
                         const RestFrame &restFrame,
                         const PosedFrame &posedFrame)
{
    using Mat4 = typename Math::Mat4;
    using D = typename std::decay_t<decltype(restFrame(0))>::value_type;
    if (bindCount != count || sampleCount < 2 || (count && !bindCoords))
        return false;
    std::vector<Mat4> maps(sampleCount);
    for (size_t k = 0; k < sampleCount; ++k)
        if (!Math::PointsToMatrix(restFrame(k), posedFrame(k), &maps[k]))
            maps[k].SetIdentity();
    for (size_t i = 0; i < count; ++i) {
        const float u = std::min(1.0f, std::max(0.0f, bindCoords[i][0]));
        const float s = u * float(sampleCount - 1);
        const size_t k = std::min(sampleCount - 2, size_t(s));
        const float t = s - float(k);
        const D p = matrixDetail::Wide<D>(points[i]);
        const D a = maps[k].TransformAffine(p);
        const D b = maps[k + 1].TransformAffine(p);
        points[i] = matrixDetail::Narrow<V>(a + (b - a) * double(t));
    }
    return true;
}

} // namespace rigExec

#endif
