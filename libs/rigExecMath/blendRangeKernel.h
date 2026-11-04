#ifndef RIGEXEC_MATH_BLEND_RANGE_KERNEL_H
#define RIGEXEC_MATH_BLEND_RANGE_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rigExec {

// A nonfinite channel weight contributes zero without invalidating other channels.
inline float RigExecBlendInputWeight(float weight)
{
    return std::isfinite(weight) ? weight : 0.0f;
}

// Samples are validated before publication. Preserve channel, sample and
// duplicate-index accumulation order independently within each vertex group.
template<class Samples, class Vec>
void RigExecAccumulateBlendRange(const Samples &samples, float weight,
                                 size_t begin, size_t end, Vec *deltas)
{
    const float w = std::min(std::max(RigExecBlendInputWeight(weight), 0.0f),
                             samples.back().activation);
    if (w == 0.0f) return;
    size_t hi = 0;
    while (hi + 1 < samples.size() && samples[hi].activation < w) ++hi;
    const float aHi = samples[hi].activation;
    const float aLo = hi ? samples[hi - 1].activation : 0.0f;
    const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
    const auto accumulate = [&](const auto &sample, float scale) {
        const auto &layout = *sample.layout;
        if (layout.indices.empty()) {
            if (layout.offsets.empty()) return;
            for (size_t i = begin; i < end; ++i)
                deltas[i - begin] += layout.offsets[i] * scale;
        } else {
            for (size_t k = 0; k < layout.indices.size(); ++k) {
                const size_t i = size_t(layout.indices[k]);
                if (i >= begin && i < end)
                    deltas[i - begin] += layout.offsets[k] * scale;
            }
        }
    };
    if (hi) accumulate(samples[hi - 1], 1.0f - t);
    accumulate(samples[hi], t);
}

} // namespace rigExec
#endif
