// Method reference: Epic Games OpenRigLogic (MIT, see THIRD_PARTY_NOTICES.md),
// RigLogic 13.5.0 at commit 9ca65ac08b64a1ea5f0d79466e64706b3ac524ba,
// src/riglogic/ml/cpu/Operation.h (MLPOperationSet::execute, the mask and
// default values) and src/riglogic/ml/cpu/layers/*LayerEvaluator.h (the
// activations), https://github.com/EpicGames/OpenRigLogic. See
// docs/references.md and docs/concepts/riglogic.md.
//
// The multilayer perceptron of RigExecMlMover. Header-only and USD-free:
// every evaluator, the zero-USD runtime included, instantiates this one
// definition, so the forward pass sums in the same order everywhere.
//
// Conventions:
// - Layer weights are row-major with one row per output unit, RigLogic's
//   layout: output[r] = act(sum_c W[r * inputs + c] * x[c] + bias[r]), the
//   sum taken in ascending c and the bias added last. Parameters are stored
//   as float, as RigLogic stores them, and accumulated in double.
// - An optional input normalization (x - mean) * scale runs before the first
//   layer. RigLogic has none; it is the form ML deformer trainers export.
// - The region mask follows RigLogic: at exactly zero the outputs take their
//   defaults, otherwise every output is multiplied by the mask.
// - The arithmetic is double where RigLogic's is float, and RigLogic's SIMD
//   blocking sums in a different order, so results agree with RigLogic to
//   float rounding, not bit for bit.
#ifndef RIGEXEC_MATH_MLP_KERNEL_H
#define RIGEXEC_MATH_MLP_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace rigExec {

/// Serialized values are the enum values; append only.
enum class RigExecMlActivation : int {
    Linear = 0,
    Relu = 1,
    /// x below zero is multiplied by the layer's alpha.
    LeakyRelu = 2,
    Tanh = 3,
    Sigmoid = 4,
};

struct RigExecMlLayer {
    size_t inputCount = 0;
    size_t outputCount = 0;
    RigExecMlActivation activation = RigExecMlActivation::Linear;
    /// LeakyRelu's slope below zero; unused by the other activations.
    double alpha = 0.0;
    /// outputCount * inputCount weights at this offset into parameters.
    size_t weightOffset = 0;
    /// outputCount biases at this offset into parameters.
    size_t biasOffset = 0;
    bool operator==(const RigExecMlLayer &o) const
    {
        return inputCount == o.inputCount && outputCount == o.outputCount &&
               activation == o.activation && alpha == o.alpha &&
               weightOffset == o.weightOffset && biasOffset == o.biasOffset;
    }
};

/// A network as data. Built once per epoch; evaluation only reads it.
struct RigExecMlNetwork {
    size_t inputCount = 0;
    std::vector<RigExecMlLayer> layers;
    std::vector<float> parameters;
    /// Empty, or inputCount each.
    std::vector<float> inputMean;
    std::vector<float> inputScale;
    /// Empty (all zero), or one per output: the outputs at mask zero.
    std::vector<float> defaults;

    bool operator==(const RigExecMlNetwork &o) const
    {
        return inputCount == o.inputCount && layers == o.layers &&
               parameters == o.parameters && inputMean == o.inputMean &&
               inputScale == o.inputScale && defaults == o.defaults;
    }

    size_t OutputCount() const
    {
        return layers.empty() ? inputCount : layers.back().outputCount;
    }

    /// The widest vector the forward pass holds.
    size_t Width() const
    {
        size_t width = inputCount;
        for (const RigExecMlLayer &layer : layers) {
            width = std::max(width, layer.outputCount);
        }
        return width;
    }
};

/// Whether the layers chain, every offset is inside `parameters`, and the
/// normalization and defaults have the right lengths. Run at compile and at
/// load, never per frame.
inline bool
RigExecMlNetworkValid(const RigExecMlNetwork &network, std::string *error)
{
    const auto fail = [&](const char *message) {
        if (error) {
            *error = message;
        }
        return false;
    };
    if (network.inputCount == 0) {
        return fail("ML network has no inputs");
    }
    if (network.layers.empty()) {
        return fail("ML network has no layers");
    }
    size_t width = network.inputCount;
    for (const RigExecMlLayer &layer : network.layers) {
        if (layer.inputCount != width || layer.outputCount == 0) {
            return fail("ML layer widths do not chain");
        }
        const size_t size = network.parameters.size();
        const size_t weights = layer.inputCount * layer.outputCount;
        if (layer.weightOffset > size || weights > size - layer.weightOffset ||
            layer.biasOffset > size ||
            layer.outputCount > size - layer.biasOffset) {
            return fail("ML layer parameters are out of range");
        }
        if (int(layer.activation) < int(RigExecMlActivation::Linear) ||
            int(layer.activation) > int(RigExecMlActivation::Sigmoid)) {
            return fail("ML layer activation is unknown");
        }
        width = layer.outputCount;
    }
    if ((!network.inputMean.empty() &&
         network.inputMean.size() != network.inputCount) ||
        (!network.inputScale.empty() &&
         network.inputScale.size() != network.inputCount)) {
        return fail("ML input normalization does not match the inputs");
    }
    if (!network.defaults.empty() &&
        network.defaults.size() != network.OutputCount()) {
        return fail("ML defaults do not match the outputs");
    }
    return true;
}

inline double
RigExecMlActivate(RigExecMlActivation activation, double x, double alpha)
{
    switch (activation) {
    case RigExecMlActivation::Relu:
        return x < 0.0 ? 0.0 : x;
    case RigExecMlActivation::LeakyRelu:
        return x < 0.0 ? x * alpha : x;
    case RigExecMlActivation::Tanh:
        return std::tanh(x);
    case RigExecMlActivation::Sigmoid:
        return 1.0 / (1.0 + std::exp(-x));
    case RigExecMlActivation::Linear:
    default:
        return x;
    }
}

/// A network from its authored form: layer output widths, activation codes
/// (RigExecMlActivation values), optional per-layer alphas, and every layer's
/// weights and then every layer's biases, concatenated in layer order. False
/// with the reason when the pieces do not fit together.
inline bool
RigExecMlBuildNetwork(size_t inputCount, const std::vector<int> &sizes,
                      const std::vector<int> &activations,
                      const std::vector<float> &alphas,
                      const std::vector<float> &weights,
                      const std::vector<float> &biases,
                      RigExecMlNetwork *network, std::string *error)
{
    const auto fail = [&](const char *message) {
        if (error) {
            *error = message;
        }
        return false;
    };
    if (sizes.empty() || activations.size() != sizes.size() ||
        (!alphas.empty() && alphas.size() != sizes.size())) {
        return fail("ML layer sizes, activations and alphas must be parallel");
    }
    RigExecMlNetwork out;
    out.inputCount = inputCount;
    size_t width = inputCount, weightTotal = 0, biasTotal = 0;
    for (size_t l = 0; l < sizes.size(); ++l) {
        if (sizes[l] <= 0 || activations[l] < 0 ||
            activations[l] > int(RigExecMlActivation::Sigmoid)) {
            return fail("ML layer size or activation is out of range");
        }
        RigExecMlLayer layer;
        layer.inputCount = width;
        layer.outputCount = size_t(sizes[l]);
        layer.activation = RigExecMlActivation(activations[l]);
        layer.alpha = alphas.empty() ? 0.0 : double(alphas[l]);
        layer.weightOffset = weightTotal;
        weightTotal += layer.inputCount * layer.outputCount;
        biasTotal += layer.outputCount;
        out.layers.push_back(layer);
        width = layer.outputCount;
    }
    if (weights.size() != weightTotal || biases.size() != biasTotal) {
        return fail("ML weights or biases do not match the layer sizes");
    }
    out.parameters = weights;
    out.parameters.insert(out.parameters.end(), biases.begin(), biases.end());
    size_t biasAt = weightTotal;
    for (RigExecMlLayer &layer : out.layers) {
        layer.biasOffset = biasAt;
        biasAt += layer.outputCount;
    }
    *network = std::move(out);
    return RigExecMlNetworkValid(*network, error);
}

/// Doubles of scratch RigExecMlEvaluate needs.
inline size_t
RigExecMlScratchSize(const RigExecMlNetwork &network)
{
    return 2 * network.Width();
}

/// The network's outputs for one input vector, attenuated by `mask`.
/// `scratch` holds RigExecMlScratchSize doubles and `output` OutputCount.
/// The network must have passed RigExecMlNetworkValid.
inline void
RigExecMlEvaluate(const RigExecMlNetwork &network, const double *input,
                  double mask, double *scratch, double *output)
{
    const size_t outputs = network.OutputCount();
    if (mask == 0.0) {
        for (size_t r = 0; r < outputs; ++r) {
            output[r] = network.defaults.empty() ? 0.0
                                                  : double(network.defaults[r]);
        }
        return;
    }
    const size_t width = network.Width();
    double *current = scratch;
    double *next = scratch + width;
    for (size_t i = 0; i < network.inputCount; ++i) {
        double x = input[i];
        if (!network.inputMean.empty()) {
            x -= double(network.inputMean[i]);
        }
        if (!network.inputScale.empty()) {
            x *= double(network.inputScale[i]);
        }
        current[i] = x;
    }
    const float *parameters = network.parameters.data();
    for (const RigExecMlLayer &layer : network.layers) {
        const float *weights = parameters + layer.weightOffset;
        const float *biases = parameters + layer.biasOffset;
        for (size_t r = 0; r < layer.outputCount; ++r) {
            const float *row = weights + r * layer.inputCount;
            double sum = 0.0;
            for (size_t c = 0; c < layer.inputCount; ++c) {
                sum += double(row[c]) * current[c];
            }
            sum += double(biases[r]);
            next[r] = RigExecMlActivate(layer.activation, sum, layer.alpha);
        }
        std::swap(current, next);
    }
    for (size_t r = 0; r < outputs; ++r) {
        output[r] = current[r] * mask;
    }
}

} // namespace rigExec

#endif // RIGEXEC_MATH_MLP_KERNEL_H
