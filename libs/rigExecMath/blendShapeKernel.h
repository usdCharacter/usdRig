// Blend-shape channel sum: each channel's weight picks its in-between pair and
// lerps their deltas. One definition for native evaluation, the runtime and the
// ops.
#ifndef RIGEXEC_MATH_BLEND_SHAPE_KERNEL_H
#define RIGEXEC_MATH_BLEND_SHAPE_KERNEL_H

#include "blendRangeKernel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

/// A sparse (or, with no indices, dense) offset table for one sample.
template <class V> struct RigExecBlendLayoutOf {
    std::vector<V> offsets;
    std::vector<int> indices;
    size_t pointCount = 0;
    bool valid = false;
};

namespace blendDetail {

// T is a traits type: Activation(s), Layout(s) (null when dense),
// DensePoints(s) and DenseCount(s) (the target's moved points).
template <class T, class S, class V>
void
Accumulate(const S &sample, const V *base, size_t baseCount, float scale,
           V *deltas)
{
    if (const auto *layout = T::Layout(sample)) {
        if (layout->indices.empty()) {
            for (size_t i = 0; i < layout->offsets.size(); ++i)
                deltas[i] += layout->offsets[i] * scale;
            return;
        }
        for (size_t k = 0; k < layout->indices.size(); ++k)
            deltas[size_t(layout->indices[k])] += layout->offsets[k] * scale;
        return;
    }
    const V *points = T::DensePoints(sample);
    for (size_t i = 0; i < baseCount; ++i)
        deltas[i] += (points[i] - base[i]) * scale;
}

} // namespace blendDetail

/// Fills \p deltas (baseCount) with the channels' summed deltas. Channel c
/// has weight weightOf(c) and samples c.samples, sorted by activation.
/// \p forRange(n, fn) runs fn(begin, end) over [0, n), serially or split;
/// each point is independent, so both give the same bits. False, atomically,
/// on a channel with no samples or a sample that doesn't fit the base.
template <class T, class V, class Channels, class WeightOf, class ForRange>
bool
RigExecSumBlendChannelsKernel(const Channels &channels, WeightOf weightOf,
                              const V *base, size_t baseCount,
                              std::vector<V> *deltas, ForRange forRange)
{
    if (!deltas || !base || !baseCount)
        return false;
    deltas->assign(baseCount, V(0.0f));
    for (const auto &channel : channels) {
        const auto &samples = channel.samples;
        if (samples.empty())
            return false;
        for (size_t k = 0; k < samples.size(); ++k) {
            const auto &sample = samples[k];
            const auto *layout = T::Layout(sample);
            const bool shapeOk =
                layout ? (layout->valid && layout->pointCount == baseCount &&
                          T::DenseCount(sample) == 0)
                       : (T::DenseCount(sample) == baseCount &&
                          T::DensePoints(sample));
            const float a = T::Activation(sample);
            if (!std::isfinite(a) || a <= 0 || !shapeOk ||
                (k > 0 && a == T::Activation(samples[k - 1])))
                return false;
        }
        // Clamped into the authored range; an implicit zero sample sits at 0.
        const float w =
            std::min(std::max(RigExecBlendInputWeight(weightOf(channel)), 0.0f),
                     T::Activation(samples.back()));
        if (w == 0.0f)
            continue;
        size_t hi = 0;
        while (hi < samples.size() && T::Activation(samples[hi]) < w)
            ++hi;
        if (hi >= samples.size())
            hi = samples.size() - 1;
        const float aHi = T::Activation(samples[hi]);
        const float aLo = hi > 0 ? T::Activation(samples[hi - 1]) : 0.0f;
        const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
        const auto *loSample = hi > 0 ? &samples[hi - 1] : nullptr;
        const auto &hiSample = samples[hi];
        V *const out = deltas->data();

        if (!T::Layout(hiSample) && (!loSample || !T::Layout(*loSample))) {
            // All dense: bit-identical to the original lerp, which the
            // sparse form below rounds differently in the last place.
            const V *lo = loSample ? T::DensePoints(*loSample) : nullptr;
            const V *hiPts = T::DensePoints(hiSample);
            forRange(baseCount, [&](size_t begin, size_t end) {
                for (size_t i = begin; i < end; ++i) {
                    const V dHi = hiPts[i] - base[i];
                    const V dLo = lo ? V(lo[i] - base[i]) : V(0.0f);
                    out[i] += dLo + (dHi - dLo) * t;
                }
            });
            continue;
        }
        // dLo (1 - t) + dHi t, each sample over its own indices.
        if (loSample)
            blendDetail::Accumulate<T>(*loSample, base, baseCount, 1.0f - t,
                                       out);
        blendDetail::Accumulate<T>(hiSample, base, baseCount, t, out);
    }
    return true;
}

} // namespace rigExec

#endif
