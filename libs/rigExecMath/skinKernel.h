// Skinning over a jointIndices/jointWeights layout: linear blend (scalar
// and SSE2) and scale-aware dual quaternion. One definition for native
// evaluation, the runtime and the ops. V is a float point, D its double
// twin, M a double 4x4 with Gf's operators, L any layout with transforms,
// transformCount, indices, elementSize, pointCount, Transform(i, k) and
// Weight(i, k). USD-free.
#ifndef RIGEXEC_MATH_SKIN_KERNEL_H
#define RIGEXEC_MATH_SKIN_KERNEL_H

#include "dualQuatKernel.h"
#include "matrixKernel.h"
#include "skinAcceptance.h"

#include <cstddef>
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

/// One scaled dual quaternion per influence, then identity for the
/// weight complement.
template <class V, class D, class L>
std::vector<RigExecScaledDualQuatKernelT<V, D>>
RigExecSkinDualQuatPaletteKernel(const L &layout)
{
    std::vector<RigExecScaledDualQuatKernelT<V, D>> palette;
    palette.reserve(layout.transformCount + 1);
    for (size_t t = 0; t < layout.transformCount; ++t)
        palette.push_back(
            RigExecScaledDualQuatFromMatrixKernel<V, D>(layout.transforms[t]));
    palette.emplace_back();
    return palette;
}

/// False when a blend degenerates. in/out may alias.
template <class V, class D, class L>
bool
RigExecApplyDualQuatSkinKernel(
    const V *in, V *out, const L &layout,
    const RigExecScaledDualQuatKernelT<V, D> *palette, size_t paletteSize)
{
    std::vector<int> indices;
    std::vector<double> weights;
    indices.reserve(layout.elementSize + 1);
    weights.reserve(layout.elementSize + 1);
    for (size_t i = 0; i < layout.pointCount; ++i) {
        RigExecGatherDualQuatInfluences(layout, i, &indices, &weights);
        RigExecScaledDualQuatKernelT<V, D> blend;
        if (!RigExecBlendScaledDualQuatsKernel(palette, paletteSize,
                                               indices.data(), weights.data(),
                                               indices.size(), &blend))
            return false;
        out[i] =
            matrixDetail::Narrow<V>(RigExecScaledDualQuatTransformPointKernel(
                blend, matrixDetail::Wide<D>(in[i])));
    }
    return true;
}

} // namespace rigExec

#endif
