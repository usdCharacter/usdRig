// Linear blend skinning (scalar and SSE2) over a jointIndices/jointWeights
// layout; the scale-aware dual quaternion blend is dualQuatKernel.h's. One
// definition for native evaluation, the runtime and the ops. V is a float point, D its double
// twin, M a double 4x4 with Gf's operators, L any layout with transforms,
// transformCount, indices, elementSize, pointCount, Transform(i, k) and
// Weight(i, k). USD-free.
#ifndef RIGEXEC_MATH_SKIN_KERNEL_H
#define RIGEXEC_MATH_SKIN_KERNEL_H

#include "dualQuatKernel.h"
#include "matrixKernel.h"
#include "skinAcceptance.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace rigExec {

/// A view over one influence table; pointers outlive the call.
template <class M> struct RigExecSkinLayoutOf {
    const M *transforms = nullptr;
    size_t transformCount = 0;
    const int *indices = nullptr;
    const float *weights = nullptr;
    size_t indexCount = 0;
    size_t elementSize = 0;
    size_t pointCount = 0;

    const M &Transform(size_t point, size_t slot) const
    {
        return transforms[size_t(indices[point * elementSize + slot])];
    }
    float Weight(size_t point, size_t slot) const
    {
        return weights[point * elementSize + slot];
    }
};

/// Shape and range check of a skin layout: at least one influence,
/// elementSize >= 1, indexCount == pointCount * elementSize, every index in
/// [0, transformCount), every weight finite and non-negative, and every
/// transform finite and affine. Fills \p error on failure.
template <class L>
bool
RigExecValidateSkinLayoutKernel(const L &layout, std::string *error = nullptr)
{
    const auto fail = [error](const std::string &why) {
        if (error)
            *error = why;
        return false;
    };
    if (layout.elementSize < 1)
        return fail("elementSize must be at least 1");
    if (!layout.transforms || layout.transformCount == 0)
        return fail("no influences");
    if (layout.indexCount != layout.pointCount * layout.elementSize)
        return fail("jointIndices/jointWeights length " +
                    std::to_string(layout.indexCount) + " must equal " +
                    std::to_string(layout.pointCount) +
                    " points * elementSize " +
                    std::to_string(layout.elementSize));
    if (layout.indexCount && (!layout.indices || !layout.weights))
        return fail("jointIndices and jointWeights are both required");
    for (size_t t = 0; t < layout.transformCount; ++t) {
        const auto &m = layout.transforms[t];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                if (!std::isfinite(m[r][c]))
                    return fail("influence " + std::to_string(t) +
                                " has a non-finite matrix");
        if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1)
            return fail("influence " + std::to_string(t) +
                        " has a non-affine matrix");
    }
    for (size_t i = 0; i < layout.indexCount; ++i) {
        if (layout.indices[i] < 0 ||
            size_t(layout.indices[i]) >= layout.transformCount)
            return fail("jointIndices[" + std::to_string(i) + "] = " +
                        std::to_string(layout.indices[i]) + " is outside the " +
                        std::to_string(layout.transformCount) + " influences");
        if (!std::isfinite(layout.weights[i]) || layout.weights[i] < 0.0f)
            return fail("jointWeights[" + std::to_string(i) +
                        "] must be finite and non-negative");
    }
    return true;
}

/// p' = (1 - sum w) p + sum w T p; the complement stays with the rest point.
template <class D, class L>
D
RigExecLinearBlendSkinPointKernel(const D &point, const L &layout, size_t i)
{
    D sum(0.0);
    double total = 0.0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const double w = layout.Weight(i, k);
        if (w == 0.0)
            continue;
        sum += layout.Transform(i, k).TransformAffine(point) * w;
        total += w;
    }
    return point * (1.0 - total) + sum;
}

/// in/out may alias.
template <class V, class D, class L>
void
RigExecApplyLinearBlendSkinKernel(const V *in, V *out, const L &layout)
{
    for (size_t i = 0; i < layout.pointCount; ++i)
        out[i] = matrixDetail::Narrow<V>(RigExecLinearBlendSkinPointKernel(
            matrixDetail::Wide<D>(in[i]), layout, i));
}

inline constexpr size_t kRigExecSkinRowStride = 16;

template <class M>
void
RigExecNarrowSkinRowsKernel(const M &transform, float *rows)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c)
            rows[r * 4 + c] = float(transform[r][c]);
        rows[r * 4 + 3] = 0.0f;
    }
}

/// SSE2 linear blend in float. \p rows holds kRigExecSkinRowStride floats
/// per influence from RigExecNarrowSkinRowsKernel, or null to narrow here.
/// Without SSE2 this is the scalar kernel. in/out may alias.
template <class V, class D, class L>
void
RigExecApplyLinearBlendSkinSimdKernel(const V *in, V *out, const L &layout,
                                      const float *rows)
{
#if !RIGEXEC_MATH_HAS_SSE2
    (void)rows;
    RigExecApplyLinearBlendSkinKernel<V, D>(in, out, layout);
#else
    std::vector<float> narrowed;
    if (!rows) {
        narrowed.resize(layout.transformCount * kRigExecSkinRowStride);
        for (size_t t = 0; t < layout.transformCount; ++t)
            RigExecNarrowSkinRowsKernel(layout.transforms[t],
                                        &narrowed[t * kRigExecSkinRowStride]);
        rows = narrowed.data();
    }
    for (size_t i = 0; i < layout.pointCount; ++i) {
        const __m128 q = _mm_setr_ps(in[i][0], in[i][1], in[i][2], 0.0f);
        const __m128 qx = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0));
        const __m128 qy = _mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1));
        const __m128 qz = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2));
        __m128 sum = _mm_setzero_ps();
        float total = 0.0f;
        for (size_t k = 0; k < layout.elementSize; ++k) {
            const float w = layout.Weight(i, k);
            if (w == 0.0f)
                continue;
            const float *t =
                rows + size_t(layout.indices[i * layout.elementSize + k]) *
                           kRigExecSkinRowStride;
            // Row-vector convention: p' = x*row0 + y*row1 + z*row2 + row3.
            const __m128 moved =
                _mm_add_ps(_mm_add_ps(_mm_mul_ps(qx, _mm_loadu_ps(t)),
                                      _mm_mul_ps(qy, _mm_loadu_ps(t + 4))),
                           _mm_add_ps(_mm_mul_ps(qz, _mm_loadu_ps(t + 8)),
                                      _mm_loadu_ps(t + 12)));
            sum = _mm_add_ps(sum, _mm_mul_ps(_mm_set1_ps(w), moved));
            total += w;
        }
        const __m128 blended =
            _mm_add_ps(_mm_mul_ps(_mm_set1_ps(1.0f - total), q), sum);
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i] = V(result[0], result[1], result[2]);
    }
#endif
}

} // namespace rigExec

#endif
