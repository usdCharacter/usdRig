#ifndef RIGEXEC_MATH_SKIN_ACCEPTANCE_H
#define RIGEXEC_MATH_SKIN_ACCEPTANCE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

template <class Layout>
void RigExecGatherDualQuatInfluences(const Layout &layout, size_t point,
                                    std::vector<int> *indices,
                                    std::vector<double> *weights)
{
    indices->clear();
    weights->clear();
    size_t pivot = 0;
    float pivotWeight = -1.0f;
    double total = 0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const float weight = layout.Weight(point, k);
        if (pivotWeight < weight) {
            pivotWeight = weight;
            pivot = k;
        }
        total += weight;
    }
    const auto push = [&](size_t k) {
        const double weight = layout.Weight(point, k);
        if (weight != 0) {
            indices->push_back(layout.indices[point * layout.elementSize + k]);
            weights->push_back(weight);
        }
    };
    push(pivot);
    for (size_t k = 0; k < layout.elementSize; ++k) {
        if (k != pivot) push(k);
    }
    const double complement = 1.0 - total;
    if (complement != 0) {
        indices->push_back(int(layout.transformCount));
        weights->push_back(complement);
    }
}

// Conservative certificate, not a change to the kernel's refusal threshold.
// Hemisphere correction bounds the real sum below by pivot - negative complement.
template <class Layout>
bool RigExecDualQuatWeightsSafe(const Layout &layout)
{
    if (!layout.elementSize || layout.elementSize > 4096) return false;
    for (size_t point = 0; point < layout.pointCount; ++point) {
        double total = 0, pivot = 0;
        for (size_t k = 0; k < layout.elementSize; ++k) {
            const double weight = layout.Weight(point, k);
            if (!(weight >= 0) || !std::isfinite(weight)) return false;
            total += weight;
            pivot = std::max(pivot, weight);
        }
        if (total == 0) continue;
        const double absolute = total + std::abs(1.0 - total);
        if (absolute > 1e6 ||
            !(pivot - std::max(0.0, total - 1.0) >
              1e-6 * std::max(1.0, absolute))) return false;
    }
    return true;
}

// The certificate allows rounding error and excludes arithmetic overflow;
// palettes outside these bounds use the exact blend predicates instead.
template <class Palette>
bool RigExecDualQuatPaletteSafe(const Palette *palette, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        const auto &entry = palette[i];
        const double norm = entry.rigid.real.GetLength();
        if (!(norm >= 1.0 - 1e-12 && norm <= 1.0 + 1e-12)) return false;
        const auto &dual = entry.rigid.dual;
        if (!(std::abs(dual.GetReal()) <= 1e100)) return false;
        for (int j = 0; j < 3; ++j) {
            if (!(std::abs(dual.GetImaginary()[j]) <= 1e100)) return false;
            for (int k = 0; k < 3; ++k) {
                if (!(std::abs(entry.stretch[j][k]) <= 1e100)) return false;
            }
        }
    }
    return true;
}

template <class Layout, class Blend>
bool RigExecDualQuatBlendsAccept(const Layout &layout, Blend blend)
{
    std::vector<int> indices;
    std::vector<double> weights;
    indices.reserve(layout.elementSize + 1);
    weights.reserve(layout.elementSize + 1);
    for (size_t point = 0; point < layout.pointCount; ++point) {
        RigExecGatherDualQuatInfluences(layout, point, &indices, &weights);
        if (!blend(indices.data(), weights.data(), indices.size())) return false;
    }
    return true;
}

} // namespace rigExec

#endif
