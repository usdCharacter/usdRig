#include "weightOpRunner.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExecOps/weight/weightOps.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecSchema/schemaNames.h"

#include <cstring>

namespace rigExec {

struct RigExecCombineOpNode {
    std::unique_ptr<RigExecEvaluator> evaluator;
    rigExecOps::CombineWeightsPayload payload;
    std::vector<RigExecInputId> items;
    RigExecInputId target, strength, invert;
    RigExecOutputId out;
    rigExecRig::PointWeights scratch;
};

namespace {

using rigExecOps::CombineWeightsPayload;
using rigExecRig::PointWeights;
using rigExecRig::WeightRepresentation;

/// The record's node; null with \p error when it cannot be built.
RigExecCombineOpNode *
Ready(std::vector<std::unique_ptr<RigExecCombineOpNode>> &nodes, size_t record,
      const CombineWeightsPayload &payload, size_t inputCount,
      const RigExecCompileOptions &options,
      const RigExecEvaluatorOptions &evaluate, std::string *error)
{
    if (nodes.size() <= record)
        nodes.resize(record + 1);
    std::unique_ptr<RigExecCombineOpNode> &node = nodes[record];
    if (node && node->payload.mode == payload.mode &&
        node->payload.inputs == payload.inputs &&
        node->payload.clamp == payload.clamp &&
        node->items.size() == inputCount)
        return node.get();
    node = std::make_unique<RigExecCombineOpNode>();
    node->payload = payload;
    RigExecRigDescBuilder builder;
    const RigExecNodeId id =
        builder.AddOp(rigExecOps::CombineWeights(), &payload);
    for (size_t k = 0; k < inputCount; ++k)
        node->items.push_back(
            builder.BindInput(id, rigExecOps::CombineWeightsInputs()));
    node->target =
        builder.BindInput(id, rigExecOps::CombineWeightsTargetCount());
    node->strength =
        builder.BindInput(id, rigExecOps::CombineWeightsStrength());
    node->invert = builder.BindInput(id, rigExecOps::CombineWeightsInvert());
    node->out = builder.BindOutput(id, rigExecOps::CombineWeightsOut());
    const auto desc = builder.Build(error);
    const auto program = desc ? RigExecCompile(*desc, options, error) : nullptr;
    node->evaluator =
        program ? RigExecEvaluator::Create(program, evaluate, error) : nullptr;
    if (!node->evaluator)
        node.reset();
    return node.get();
}

/// Writes the shared inputs and runs; the output, or null with \p error
/// where the evaluation fails.
const PointWeights *
Run(RigExecCombineOpNode &node, size_t targetCount, float strength,
    float invert, std::string *error)
{
    const uint32_t count = uint32_t(targetCount);
    RigExecEvaluator &e = *node.evaluator;
    if (!e.SetInput(node.target, &count, error) ||
        !e.SetInput(node.strength, &strength, error) ||
        !e.SetInput(node.invert, &invert, error) || !e.Evaluate(error))
        return nullptr;
    return static_cast<const PointWeights *>(e.GetOutput(node.out, error));
}

template <class T>
void
Assign(const T *data, size_t size, rigExecStd::RigExecArray<T> *array)
{
    array->Resize(size);
    if (size)
        std::memcpy(array->Data(), data, size * sizeof(T));
}

} // namespace

RigExecWeightCombineOps::RigExecWeightCombineOps() = default;
RigExecWeightCombineOps::~RigExecWeightCombineOps() = default;
RigExecWeightCombineOps::RigExecWeightCombineOps(
    RigExecWeightCombineOps &&) noexcept = default;
RigExecWeightCombineOps &RigExecWeightCombineOps::operator=(
    RigExecWeightCombineOps &&) noexcept = default;

bool
RigExecWeightCombineOps::Combine(size_t record, const RigExecWeightRecord &r,
                                 size_t count, float strength, float invert,
                                 Resolve resolve, const void *context,
                                 std::vector<float> *out, std::string *error)
{
    CombineWeightsPayload payload;
    payload.mode = rigExecOps::WeightCombineMode(r.combineMode);
    payload.inputs = rigExecOps::WeightCombineInputs::Fields;
    payload.clamp = r.clamp;
    RigExecCombineOpNode *node = Ready(_nodes, record, payload, r.inputs.size(),
                                       compile, evaluate, error);
    if (!node)
        return false;
    PointWeights &field = node->scratch;
    field.indices.Resize(0);
    field.defaultWeight = 0.0f;
    field.representation = uint32_t(WeightRepresentation::Dense);
    field.valid = 1;
    for (size_t k = 0; k < r.inputs.size(); ++k) {
        const float *data = nullptr;
        size_t size = 0;
        if (!resolve(context, k, &data, &size))
            return false;
        Assign(data, size, &field.values);
        if (!node->evaluator->SetInput(node->items[k], &field))
            return false;
    }
    std::string why;
    const PointWeights *result = Run(*node, count, strength, invert, &why);
    if (!result)
        return false;
    out->assign(result->values.begin(), result->values.end());
    return true;
}

bool
RigExecWeightCombineOps::CombinePackets(
    size_t record, const RigExecWeightRecord &r,
    const std::vector<const RigExecWeightPacket *> &inputs, size_t targetCount,
    float strength, float invert, RigExecWeightPacket *packet,
    std::string *error)
{
    *packet = RigExecWeightPacket();
    if (r.representation != 1 ||
        (!r.clamp && r.rangePolicy != SchemaValueText::strict) ||
        r.combineMode < 0) {
        if (error)
            *error = "the combine record has no CombineWeights form";
        return false;
    }
    CombineWeightsPayload payload;
    payload.mode = rigExecOps::WeightCombineMode(r.combineMode);
    payload.clamp = r.clamp;
    RigExecCombineOpNode *node =
        Ready(_nodes, record, payload, inputs.size(), compile, evaluate, error);
    if (!node)
        return false;
    PointWeights &w = node->scratch;
    for (size_t k = 0; k < inputs.size(); ++k) {
        const RigExecWeightPacket *in = inputs[k];
        if (in) {
            Assign(in->values.data(), in->values.size(), &w.values);
            Assign(in->indices.data(), in->indices.size(), &w.indices);
            w.defaultWeight = in->defaultWeight;
            w.representation =
                uint32_t(in->representation == SchemaValueText::dense
                             ? WeightRepresentation::Dense
                         : in->representation == SchemaValueText::sparse
                             ? WeightRepresentation::Sparse
                             : WeightRepresentation::Constant);
            w.valid = in->valid ? 1 : 0;
        } else {
            w = PointWeights();
        }
        if (!node->evaluator->SetInput(node->items[k], &w, error))
            return false;
    }
    const PointWeights *result =
        Run(*node, targetCount, strength, invert, error);
    if (!result)
        return false;
    packet->representation = r.representationToken;
    packet->rangePolicy = r.rangePolicy;
    packet->values.assign(result->values.begin(), result->values.end());
    packet->valid = true;
    return true;
}

} // namespace rigExec
