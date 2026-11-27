#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/avarScale.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecStdTypes/array.h"

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

} // namespace

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
