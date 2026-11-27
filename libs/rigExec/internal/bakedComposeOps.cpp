#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/avarScale.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecStdTypes/array.h"

#include <cstring>
#include <string>
#include <vector>

namespace rigExec {

namespace {

using Doubles = rigExecStd::RigExecArray<double>;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// Provider slot \p slot's avar bindings: the varying ones, then the
/// constant ones.
struct AvarBindings {
    const RigExecBakedProgramImpl &B;
    std::pair<size_t, size_t> varying, constant;

    AvarBindings(const RigExecBakedProgramImpl &program, uint32_t slot)
        : B(program), varying(RigExecBakedAvarBindingRange(
                          program.avarBindingBegin, int64_t(slot))),
          constant(RigExecBakedAvarBindingRange(
              program.avarConstantBindingBegin, int64_t(slot)))
    {
    }
    size_t Count() const
    {
        return (varying.second - varying.first) +
               (constant.second - constant.first);
    }
    const RigExecBakedProgramImpl::AvarBinding &At(size_t k) const
    {
        const size_t n = varying.second - varying.first;
        return k < n ? B.avarBindings[varying.first + k]
                     : B.avarConstantBindings[constant.first + (k - n)];
    }
    unsigned Channels() const
    {
        return RigExecBoundAvarChannels(Count(),
                                        [&](size_t k) { return At(k).slot; });
    }
};

/// Resizes list \p value to \p count elements of \p T and fills element k
/// with \p at(k).
template <class T, class At>
bool
FillList(const RigExecTypeDesc &type, void *value, size_t count, At &&at)
{
    if (!type.resize || !type.data)
        return false;
    type.resize(value, count);
    T *out = static_cast<T *>(type.data(value));
    for (size_t k = 0; k < count; ++k)
        out[k] = at(k);
    return true;
}

/// Fills Mat4d list \p value with \p table's entries at \p slots.
bool
FillMatrices(const RigExecTypeDesc &type, void *value,
             const std::vector<int> &slots,
             const std::vector<GfMatrix4d> &table)
{
    if (!type.resize || !type.data)
        return false;
    type.resize(value, slots.size());
    auto *out = static_cast<double *>(type.data(value));
    for (size_t k = 0; k < slots.size(); ++k)
        std::memcpy(out + 16 * k, table[size_t(slots[k])].data(),
                    16 * sizeof(double));
    return true;
}

/// Answers input \p read of space checkpoint \p object's SpaceCheckpoint
/// node from the slots it recomposes.
bool
LookUpCheckpoint(const RigExecBakedProgramImpl &B, uint32_t object,
                 eval::OpCheckpointRead read, const RigExecTypeDesc &type,
                 void *value)
{
    using R = eval::OpCheckpointRead;
    const auto &context = B.switchFrameContexts[object];
    const std::vector<int> &slots = context.recompose;
    const auto flags = [&](const std::vector<char> &table) {
        return FillList<uint8_t>(type, value, slots.size(), [&](size_t k) {
            return uint8_t(table[size_t(slots[k])] != 0);
        });
    };
    switch (read) {
    case R::Avars:
        return FillList<double>(
            type, value, slots.size() * kRigExecSlotAvars, [&](size_t k) {
                return B.avars[size_t(slots[k / kRigExecSlotAvars]) *
                                   kRigExecSlotAvars +
                               k % kRigExecSlotAvars];
            });
    case R::RotOrders:
        return FillList<uint32_t>(type, value, slots.size(), [&](size_t k) {
            const TfToken &order = B.rotOrder[size_t(slots[k])];
            return RigExecRotOrderCode(order.GetText(), order.size());
        });
    case R::RotationSigns:
        return FillList<uint32_t>(type, value, slots.size(), [&](size_t k) {
            const size_t s = size_t(slots[k]);
            return s < B.rotationSign.size() ? uint32_t(B.rotationSign[s]) : 0u;
        });
    case R::PosedDefault:
        return FillMatrices(type, value, slots, B.posedD);
    case R::ParentDefaultInverse:
        return FillMatrices(type, value, slots, B.parentDinv);
    case R::ParentExpression:
        return FillMatrices(type, value, slots, B.parentSpaceM);
    case R::ParentExpressionAuthored:
        return flags(B.parentSpaceAuthored);
    case R::PosedAuthored:
        return flags(B.posedAuthored);
    case R::PosedAuthoredMatrix:
        return FillMatrices(type, value, slots, B.posedAuthoredM);
    case R::Anchor:
        if (context.anchor < 0 || type.size != 16 * sizeof(double))
            return false;
        std::memcpy(value, B.posedM[size_t(context.anchor)].data(),
                    16 * sizeof(double));
        return true;
    }
    return false;
}

/// The SpaceCheckpoint node of space checkpoint \p object: every recomposed
/// slot's inputs and the anchor's posed matrix, into its switch frame.
bool
AddCheckpointOp(const RigExecBakedProgramImpl &B,
                RigExecRigDescBuilder *builder, eval::OpStepLayout *layout,
                size_t o, uint32_t object, std::string *error)
{
    using namespace rigExecOps;
    using R = eval::OpCheckpointRead;
    const auto &context = B.switchFrameContexts[object];
    SpaceCheckpointPayload payload;
    payload.noScale.reserve(context.recompose.size());
    for (const int slot : context.recompose)
        payload.noScale.push_back(B.noScaleAvars[size_t(slot)] != 0);
    const RigExecNodeId node = builder->AddOp(SpaceCheckpoint(), &payload);
    if (node != o)
        return Fail(error, "space checkpoint " + std::to_string(object) +
                               " was added as node " + std::to_string(node));
    const auto feed = [&](const RigExecPortHandle &port, R read) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(eval::OpLookupSource::CheckpointInputs), object, -1,
             uint8_t(read)});
    };
    feed(SpaceCheckpointAvars(), R::Avars);
    feed(SpaceCheckpointRotOrders(), R::RotOrders);
    feed(SpaceCheckpointRotationSigns(), R::RotationSigns);
    feed(SpaceCheckpointPosedDefault(), R::PosedDefault);
    feed(SpaceCheckpointParentDefaultInverse(), R::ParentDefaultInverse);
    feed(SpaceCheckpointParentExpression(), R::ParentExpression);
    feed(SpaceCheckpointParentExpressionAuthored(),
         R::ParentExpressionAuthored);
    feed(SpaceCheckpointPosedAuthored(), R::PosedAuthored);
    feed(SpaceCheckpointPosedAuthoredMatrix(), R::PosedAuthoredMatrix);
    if (context.anchor >= 0)
        feed(SpaceCheckpointAnchor(), R::Anchor);
    builder->BindOutput(node, SpaceCheckpointOut());
    layout->hostOutputs.push_back({node, SpaceCheckpointOut().Index(),
                                   uint8_t(eval::OpOutputTarget::SwitchFrame),
                                   object});
    return true;
}

} // namespace

bool
RigExecBakedAddComposeOp(const RigExecBakedProgramImpl &B,
                         RigExecRigDescBuilder *builder,
                         eval::OpStepLayout *layout, size_t o,
                         std::string *error)
{
    const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
    if (origin.family == RigExecBakedOpFamily::SpaceCheckpoint &&
        origin.object >= 0 &&
        size_t(origin.object) < B.switchFrameContexts.size())
        return AddCheckpointOp(B, builder, layout, o, uint32_t(origin.object),
                               error);
    return Fail(error, "node " + std::to_string(o) + " has no compose origin");
}

void
RigExecBakedPlanComposeOps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (size_t slot = 0; slot < B.paths.size(); ++slot) {
        RigExecBakedOpOrigin origin;
        origin.family = RigExecBakedOpFamily::AvarInputs;
        origin.kind = RigExecBakedStepKind::AvarInputs;
        origin.object = int32_t(slot);
        B.opNodeOrigins.push_back(origin);
    }
}

bool
RigExecBakedAddComposeOps(const RigExecBakedProgramImpl &B,
                          RigExecRigDescBuilder *builder,
                          eval::OpStepLayout *layout, std::string *error)
{
    using namespace rigExecOps;
    for (size_t o = 0; o < B.opNodeOrigins.size(); ++o) {
        const RigExecBakedOpOrigin &origin = B.opNodeOrigins[o];
        if (origin.family != RigExecBakedOpFamily::AvarInputs)
            continue;
        const uint32_t slot = uint32_t(origin.object);
        const AvarBindings bindings(B, slot);
        const unsigned channels = bindings.Channels();
        AvarInputsPayload payload;
        payload.avarCount = RigExecAvarElement(channels, kRigExecSlotAvars);
        payload.targets.reserve(bindings.Count());
        for (size_t k = 0; k < bindings.Count(); ++k)
            payload.targets.push_back(RigExecAvarElement(
                channels, bindings.At(k).slot % kRigExecSlotAvars));
        const RigExecNodeId node = builder->AddOp(AvarInputs(), &payload);
        if (node != o)
            return Fail(error, "the avar inputs of node " + std::to_string(o) +
                                   " were added as node " +
                                   std::to_string(node));
        builder->BindInput(node, AvarInputsValues());
        layout->hostInputs.push_back({node, AvarInputsValues().Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back(
            {uint8_t(eval::OpLookupSource::AvarBindings), slot, -1, 0});
        builder->BindOutput(node, AvarInputsOut());
        layout->hostOutputs.push_back({node, AvarInputsOut().Index(),
                                       uint8_t(eval::OpOutputTarget::Avars),
                                       slot});
    }
    return true;
}

bool
RigExecBakedLookUpCompose(const RigExecBakedProgramImpl &B,
                          const eval::OpLookup &lookup,
                          const RigExecTypeDesc &type, void *value,
                          std::string *error)
{
    if (lookup.source == uint8_t(eval::OpLookupSource::CheckpointInputs)) {
        if (lookup.object < B.switchFrameContexts.size() &&
            LookUpCheckpoint(B, lookup.object,
                             eval::OpCheckpointRead(lookup.read), type, value))
            return true;
        return Fail(error, "space checkpoint " + std::to_string(lookup.object) +
                               " has no input " + std::to_string(lookup.read));
    }
    if (lookup.source != uint8_t(eval::OpLookupSource::AvarBindings) ||
        lookup.object >= B.paths.size() || !type.resize || !type.data)
        return Fail(error, "no avar bindings of slot " +
                               std::to_string(lookup.object));
    const AvarBindings bindings(B, lookup.object);
    type.resize(value, bindings.Count());
    auto *out = static_cast<double *>(type.data(value));
    for (size_t k = 0; k < bindings.Count(); ++k)
        out[k] = RigExecBakedLeafRead(B, bindings.At(k).input);
    return true;
}

bool
RigExecBakedStoreCompose(RigExecBakedProgramImpl &B,
                         const eval::OpHostOutput &output, const void *value,
                         std::string *error)
{
    if (output.target == uint8_t(eval::OpOutputTarget::SwitchFrame)) {
        if (output.object >= B.switchFrames.size())
            return Fail(error,
                        "no space checkpoint " + std::to_string(output.object));
        std::memcpy(B.switchFrames[output.object].data(), value,
                    16 * sizeof(double));
        return true;
    }
    const Doubles &values = *static_cast<const Doubles *>(value);
    const unsigned channels = AvarBindings(B, output.object).Channels();
    const size_t first = size_t(output.object) * kRigExecSlotAvars;
    if (output.target != uint8_t(eval::OpOutputTarget::Avars) ||
        first + kRigExecSlotAvars > B.avars.size() ||
        values.Size() != RigExecAvarElement(channels, kRigExecSlotAvars))
        return Fail(error, "slot " + std::to_string(output.object) +
                               " stores no " + std::to_string(values.Size()) +
                               " avars");
    size_t element = 0;
    for (size_t c = 0; c < kRigExecSlotAvars; ++c)
        if ((channels >> c) & 1u)
            B.avars[first + c] = values[element++];
    return true;
}

} // namespace rigExec
