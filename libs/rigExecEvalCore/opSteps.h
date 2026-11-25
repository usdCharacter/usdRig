// Op nodes as steps of the common schedule: one OpNode step per node that
// does not run in ranges, and per range chain one OpChainHead step and one
// OpChainRange step per (segment, vertex group), the groups those of the
// point chain whose points the chain reads, so an edit reruns only the
// groups it reaches. Header-only and free of OpenUSD: the baked program,
// its frozen clones and the runtime run one definition.
#ifndef RIGEXEC_EVAL_CORE_OP_STEPS_H
#define RIGEXEC_EVAL_CORE_OP_STEPS_H

#include "rigExec/api/registry.h"
#include "rigExec/api/runState.h"
#include "rigExec/api/view.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {
namespace eval {

/// Where an op input the host writes reads its value from.
enum class OpHostSource : uint8_t {
    /// A point chain's final points, vertex group by vertex group. It feeds
    /// pointwise inputs of one range chain only.
    ChainPoints = 0,
    /// A joint's or control's final matrix.
    FinalMatrix = 1,
    /// A joint's or control's base matrix.
    BaseMatrix = 2,
    /// A value the host looks up before the node runs: \p object indexes
    /// the layout's lookups. Which source answers may change per frame.
    Lookup = 4,
};

/// A per-frame read of the rig the host resolves for a Lookup input; what
/// \p object, \p part and \p read name depends on \p source.
struct OpLookup {
    uint8_t source = 0;
    uint32_t object = 0;
    int32_t part = -1;
    uint8_t read = 0;
};

/// A port value the host stores in one of its own slots after the node
/// runs: an output, or an input a lookup feeds. What \p object names
/// depends on \p target.
struct OpHostOutput {
    uint32_t node = 0;
    uint32_t port = 0;
    uint8_t target = 0;
    uint32_t object = 0;
};

/// One value of a port's type, constructed and destroyed with it.
class OpPortValue {
public:
    explicit OpPortValue(const RigExecTypeDesc &type)
        : _type(type), _storage(type.size / sizeof(std::max_align_t) + 1)
    {
        if (_type.construct)
            _type.construct(_storage.data());
    }
    ~OpPortValue()
    {
        if (_type.destroy)
            _type.destroy(_storage.data());
    }
    OpPortValue(const OpPortValue &) = delete;
    OpPortValue &operator=(const OpPortValue &) = delete;
    const RigExecTypeDesc &Type() const
    {
        return _type;
    }
    void *Value()
    {
        return _storage.data();
    }

private:
    const RigExecTypeDesc &_type;
    std::vector<std::max_align_t> _storage;
};

/// An op input the host writes from the rig: the node and port it feeds,
/// and the chain or joint slot its value comes from.
struct OpHostInput {
    uint32_t node = 0;
    uint32_t port = 0;
    OpHostSource source = OpHostSource::ChainPoints;
    uint32_t object = 0;
};

/// An op input fixed when the rig was built: the node and port it feeds,
/// and its value's bytes, an array's elements one after another.
struct OpConstant {
    uint32_t node = 0;
    uint32_t port = 0;
    std::vector<uint8_t> bytes;
};

/// A constant as a value of its port's type: an array borrows the
/// constant's bytes, a value holding no array copies them.
class OpConstantValue {
public:
    OpConstantValue(const RigExecTypeDesc &type, const OpConstant &constant)
        : _type(type), _bytes(constant.bytes),
          _storage(type.size / sizeof(std::max_align_t) + 1)
    {
        if (_type.construct)
            _type.construct(_storage.data());
        if (_type.element && _type.borrow) {
            const size_t element = RigExecRegistry::Get()
                                       .TypeAt(_type.element->Index())
                                       .desc->size;
            _valid = element && _bytes.size() % element == 0;
            if (_valid)
                _type.borrow(_storage.data(), _bytes.data(),
                             _bytes.size() / element);
        } else if (_type.triviallyCopyable && _bytes.size() == _type.size) {
            std::memcpy(_storage.data(), _bytes.data(), _type.size);
            _valid = true;
        }
    }
    ~OpConstantValue()
    {
        if (_type.destroy)
            _type.destroy(_storage.data());
    }
    OpConstantValue(const OpConstantValue &) = delete;
    OpConstantValue &operator=(const OpConstantValue &) = delete;
    bool Valid() const
    {
        return _valid;
    }
    const void *Value() const
    {
        return _storage.data();
    }

private:
    const RigExecTypeDesc &_type;
    std::vector<uint8_t> _bytes;
    std::vector<std::max_align_t> _storage;
    bool _valid = false;
};

/// One range chain as steps: the point chain whose vertex groups it runs
/// in and their bounds; per member output, its first OpGroupValue slot.
struct OpChainLayout {
    uint32_t pointChain = 0;
    std::vector<int> bounds;
    uint32_t Groups() const
    {
        return bounds.size() > 1 ? uint32_t(bounds.size() - 1) : 0;
    }
};

/// The step kinds' wire and baked numbers.
enum class OpStepKind : uint8_t { Node = 30, ChainHead = 31, ChainRange = 32 };

/// What every run of the op steps shares; fixed once laid out. OpValue
/// slots number the program's values (their value numbers) and then one
/// plan per chain; OpGroupValue slots number each group of each value a
/// chain member writes.
struct OpStepLayout {
    std::shared_ptr<const RigExecProgram> program;
    std::vector<OpHostInput> hostInputs;
    std::vector<OpLookup> lookups;
    std::vector<OpHostOutput> hostOutputs;
    std::vector<OpConstant> constants;
    std::vector<OpChainLayout> chains;
    /// Filled by LayOutOpSteps.
    std::vector<std::shared_ptr<const OpConstantValue>> constantValues;
    std::vector<uint32_t> constantIds;
    std::vector<RigExecNodeDesc> nodes;
    std::vector<RigExecChainDesc> chainDescs;
    /// Per program node: its chain and member index, or kRigExecInvalidIndex.
    std::vector<uint32_t> chainOf, memberOf;
    /// Per hostInputs entry: the program input it writes.
    std::vector<uint32_t> hostInputIds;
    /// Per program node: its Lookup hostInputs entries and its hostOutputs
    /// entries.
    std::vector<std::vector<uint32_t>> lookupsOf, hostOutputsOf;
    /// Per value number: its first OpGroupValue slot, or kRigExecInvalidIndex,
    /// and the node writing it, or kRigExecInvalidIndex for an input.
    std::vector<uint32_t> groupBase, producerOf;
    uint32_t valueCount = 0;
    uint32_t groupValueCount = 0;

    uint32_t PlanSlot(size_t chain) const
    {
        return valueCount + uint32_t(chain);
    }
    uint32_t OpValueCount() const
    {
        return valueCount + uint32_t(chains.size());
    }
};

inline bool
OpLayoutFail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op steps: " + why;
    return false;
}

/// Fills \p layout's derived tables from its program, host inputs and
/// chains. False with the reason when the program was not compiled for a
/// host's schedule, a chain has no groups, or a host input does not exist
/// or feeds what its source cannot.
inline bool
LayOutOpSteps(OpStepLayout *layout, std::string *error)
{
    const RigExecProgram &program = *layout->program;
    const size_t nodes = RigExecProgramNodeCount(program);
    const size_t chains = RigExecProgramChainCount(program);
    if (layout->chains.size() != chains)
        return OpLayoutFail(error, "the program has " + std::to_string(chains) +
                                       " range chains, the layout " +
                                       std::to_string(layout->chains.size()));
    layout->valueCount = uint32_t(RigExecProgramValueCount(program));
    layout->nodes.assign(nodes, RigExecNodeDesc());
    for (size_t n = 0; n < nodes; ++n)
        RigExecProgramNodeAt(program, n, &layout->nodes[n]);
    layout->chainDescs.assign(chains, RigExecChainDesc());
    layout->chainOf.assign(nodes, kRigExecInvalidIndex);
    layout->memberOf.assign(nodes, kRigExecInvalidIndex);
    layout->groupBase.assign(layout->valueCount, kRigExecInvalidIndex);
    layout->producerOf.assign(layout->valueCount, kRigExecInvalidIndex);
    for (size_t n = 0; n < nodes; ++n) {
        const RigExecNodeDesc &d = layout->nodes[n];
        for (size_t p = 0; p < d.values.size(); ++p)
            if ((d.op->ports[p].flags & kRigExecPortOutput) &&
                d.values[p] != kRigExecInvalidIndex)
                layout->producerOf[d.values[p]] = uint32_t(n);
    }
    layout->groupValueCount = 0;
    for (size_t c = 0; c < chains; ++c) {
        RigExecChainDesc &desc = layout->chainDescs[c];
        RigExecProgramChainAt(program, c, &desc);
        const uint32_t groups = layout->chains[c].Groups();
        if (!groups)
            return OpLayoutFail(error, "chain " + std::to_string(c) +
                                           " runs in no vertex groups");
        for (uint32_t m = 0; m < desc.members.size(); ++m) {
            const uint32_t node = desc.members[m];
            layout->chainOf[node] = uint32_t(c);
            layout->memberOf[node] = m;
            const RigExecNodeDesc &d = layout->nodes[node];
            for (size_t p = 0; p < d.values.size(); ++p) {
                if (!(d.op->ports[p].flags & kRigExecPortOutput) ||
                    d.values[p] == kRigExecInvalidIndex)
                    continue;
                layout->groupBase[d.values[p]] = layout->groupValueCount;
                layout->groupValueCount += groups;
            }
        }
    }
    layout->hostInputIds.clear();
    layout->lookupsOf.assign(nodes, std::vector<uint32_t>());
    layout->hostOutputsOf.assign(nodes, std::vector<uint32_t>());
    for (const OpHostInput &input : layout->hostInputs) {
        if (input.node >= nodes ||
            input.port >= layout->nodes[input.node].inputs.size() ||
            layout->nodes[input.node].inputs[input.port] ==
                kRigExecInvalidIndex)
            return OpLayoutFail(error, "node " + std::to_string(input.node) +
                                           " has no host input " +
                                           std::to_string(input.port));
        const uint32_t id = layout->nodes[input.node].inputs[input.port];
        if (input.source == OpHostSource::ChainPoints) {
            const uint32_t c = layout->chainOf[input.node];
            if (c == kRigExecInvalidIndex ||
                !(layout->nodes[input.node].op->ports[input.port].flags &
                  kRigExecPortPointwise))
                return OpLayoutFail(error,
                                    "chain points feed node " +
                                        std::to_string(input.node) +
                                        ", which does not read them point "
                                        "by point in a range chain");
        }
        if (input.source == OpHostSource::Lookup) {
            if (input.object >= layout->lookups.size())
                return OpLayoutFail(
                    error, "node " + std::to_string(input.node) +
                               " reads lookup " + std::to_string(input.object) +
                               ", which the layout lacks");
            if (layout->nodes[input.node].op->ports[input.port].flags &
                kRigExecPortChecked)
                return OpLayoutFail(error, "a lookup feeds checked input " +
                                               std::to_string(input.port) +
                                               " of node " +
                                               std::to_string(input.node));
            layout->lookupsOf[input.node].push_back(
                uint32_t(layout->hostInputIds.size()));
        }
        layout->hostInputIds.push_back(id);
    }
    for (uint32_t o = 0; o < layout->hostOutputs.size(); ++o) {
        const OpHostOutput &output = layout->hostOutputs[o];
        bool stored = output.node < nodes &&
                      output.port < layout->nodes[output.node].values.size() &&
                      layout->nodes[output.node].values[output.port] !=
                          kRigExecInvalidIndex;
        if (stored &&
            !(layout->nodes[output.node].op->ports[output.port].flags &
              kRigExecPortOutput)) {
            stored = false;
            for (const OpHostInput &input : layout->hostInputs)
                stored = stored || (input.node == output.node &&
                                    input.port == output.port &&
                                    input.source == OpHostSource::Lookup);
        }
        if (!stored)
            return OpLayoutFail(error, "node " + std::to_string(output.node) +
                                           " has no output or looked-up "
                                           "input " +
                                           std::to_string(output.port) +
                                           " to store");
        layout->hostOutputsOf[output.node].push_back(o);
    }
    layout->constantValues.clear();
    layout->constantIds.clear();
    for (const OpConstant &constant : layout->constants) {
        if (constant.node >= nodes ||
            constant.port >= layout->nodes[constant.node].inputs.size() ||
            layout->nodes[constant.node].inputs[constant.port] ==
                kRigExecInvalidIndex)
            return OpLayoutFail(error, "node " + std::to_string(constant.node) +
                                           " has no input " +
                                           std::to_string(constant.port) +
                                           " to fix");
        const RigExecPortDesc &port =
            layout->nodes[constant.node].op->ports[constant.port];
        auto value = std::make_shared<const OpConstantValue>(
            *RigExecRegistry::Get().TypeAt(port.type->Index()).desc, constant);
        if (!value->Valid())
            return OpLayoutFail(
                error, "the value fixed for input " + std::string(port.name) +
                           " of node " + std::to_string(constant.node) +
                           " is not of its type");
        layout->constantValues.push_back(std::move(value));
        layout->constantIds.push_back(
            layout->nodes[constant.node].inputs[constant.port]);
    }
    return true;
}

/// One run of a layout: its run state, the groups each chain-points input
/// is handed (each range step sets its own group's entry), and the
/// version of every op value, bumped by the step that writes it.
struct OpStepState {
    std::unique_ptr<RigExecRunState> run;
    /// Per hostInputs entry: the value a Lookup input reads, else null.
    std::vector<std::unique_ptr<OpPortValue>> lookedUp;
    std::vector<std::vector<const void *>> groupPoints;
    std::vector<RigExecOpGroups> groups;
    std::vector<uint64_t> versions, groupVersions;
};

/// Points \p state's run at the host's values: each matrix input at what
/// \p host.Matrix answers, each chain-points input at a group
/// table its range steps fill. Call it on a new or cloned state, before
/// its first step, whenever the host's storage moved.
template <class Host>
bool
BindOpStepState(const OpStepLayout &layout, Host &host, OpStepState *state,
                std::string *error)
{
    state->groupPoints.assign(layout.hostInputs.size(),
                              std::vector<const void *>());
    state->groups.assign(layout.hostInputs.size(), RigExecOpGroups());
    state->lookedUp.clear();
    state->lookedUp.resize(layout.hostInputs.size());
    state->versions.resize(layout.OpValueCount(), 0);
    state->groupVersions.resize(layout.groupValueCount, 0);
    for (size_t i = 0; i < layout.constantIds.size(); ++i)
        state->run->PointInput(RigExecInputId{layout.constantIds[i]},
                               layout.constantValues[i]->Value());
    for (size_t i = 0; i < layout.hostInputs.size(); ++i) {
        const OpHostInput &input = layout.hostInputs[i];
        const RigExecInputId id{layout.hostInputIds[i]};
        if (input.source == OpHostSource::Lookup) {
            const RigExecPortDesc &port =
                layout.nodes[input.node].op->ports[input.port];
            state->lookedUp[i] = std::make_unique<OpPortValue>(
                *RigExecRegistry::Get().TypeAt(port.type->Index()).desc);
            if (!state->run->PointInput(id, state->lookedUp[i]->Value()))
                return OpLayoutFail(
                    error, "node " + std::to_string(input.node) +
                               " has no input " + std::to_string(input.port));
            continue;
        }
        if (input.source != OpHostSource::ChainPoints) {
            const void *value = host.Matrix(input.source, input.object);
            if (!value || !state->run->PointInput(id, value))
                return OpLayoutFail(error, "slot " +
                                               std::to_string(input.object) +
                                               " has no such matrix");
            continue;
        }
        const OpChainLayout &chain = layout.chains[layout.chainOf[input.node]];
        state->groupPoints[i].assign(chain.Groups(), nullptr);
        for (uint32_t g = 0; g < chain.Groups(); ++g)
            state->groupPoints[i][g] = host.ChainGroupPoints(input.object, g);
        state->groups[i] = {chain.bounds.data(), chain.Groups(),
                            state->groupPoints[i].data()};
        if (!state->run->PointInputGroups(id, &state->groups[i], error))
            return false;
    }
    return true;
}

/// Runs op step (\p kind, \p object, \p part): node \p object, after
/// \p host.Lookup(index, lookup, type, value, error) resolved each of its
/// Lookup inputs in layout order, a hostOutput of a looked-up input stored
/// by \p host.Output(output, value, error) before the next lookup, and the
/// hostOutputs of its outputs stored after it ran; chain \p object's head;
/// or group part % G of the
/// segment starting at member part / G of chain \p object. False with the
/// reason in \p error. Steps that do not feed each other may run at once.
template <class Host>
bool
RunOpStep(const OpStepLayout &layout, Host &host, OpStepState *state,
          OpStepKind kind, int object, int part, std::string *error)
{
    RigExecRunState &run = *state->run;
    if (object < 0)
        return OpLayoutFail(error, "an op step names no object");
    const size_t index = size_t(object);
    if (kind == OpStepKind::Node) {
        if (index >= layout.nodes.size())
            return OpLayoutFail(error, "no node " + std::to_string(object));
        const RigExecNodeDesc &d = layout.nodes[index];
        for (const uint32_t i : layout.lookupsOf[index]) {
            OpPortValue &value = *state->lookedUp[i];
            const uint32_t lookup = layout.hostInputs[i].object;
            if (!host.Lookup(lookup, layout.lookups[lookup], value.Type(),
                             value.Value(), error))
                return false;
            for (const uint32_t o : layout.hostOutputsOf[index])
                if (layout.hostOutputs[o].port == layout.hostInputs[i].port &&
                    !host.Output(layout.hostOutputs[o], value.Value(), error))
                    return false;
        }
        if (!run.RunNode(RigExecNodeId(object), error))
            return false;
        for (const uint32_t o : layout.hostOutputsOf[index]) {
            const OpHostOutput &output = layout.hostOutputs[o];
            if (!(d.op->ports[output.port].flags & kRigExecPortOutput))
                continue;
            RigExecValueRef ref;
            ref.value = d.values[output.port];
            const void *value = run.GetValue(ref);
            if (!value)
                return OpLayoutFail(error, "node " + std::to_string(object) +
                                               " holds no value at port " +
                                               std::to_string(output.port));
            if (!host.Output(output, value, error))
                return false;
        }
        for (size_t p = 0; p < d.values.size(); ++p)
            if ((d.op->ports[p].flags & kRigExecPortOutput) &&
                d.values[p] != kRigExecInvalidIndex)
                ++state->versions[d.values[p]];
        return true;
    }
    if (index >= layout.chains.size())
        return OpLayoutFail(error, "no chain " + std::to_string(object));
    const OpChainLayout &chain = layout.chains[index];
    const uint32_t groups = chain.Groups();
    if (kind == OpStepKind::ChainHead) {
        if (!run.RunChainHead(index, chain.bounds.data(), groups, error))
            return false;
        ++state->versions[layout.PlanSlot(index)];
        return true;
    }
    if (part < 0 || !groups)
        return OpLayoutFail(error, "an op range step names no group");
    const uint32_t segment = uint32_t(part) / groups;
    const uint32_t g = uint32_t(part) % groups;
    const RigExecChainDesc &desc = layout.chainDescs[index];
    for (size_t i = 0; i < layout.hostInputs.size(); ++i) {
        const OpHostInput &input = layout.hostInputs[i];
        if (input.source != OpHostSource::ChainPoints ||
            layout.chainOf[input.node] != index ||
            desc.segments[layout.memberOf[input.node]] != segment)
            continue;
        state->groupPoints[i][g] = host.ChainGroupPoints(input.object, g);
    }
    if (!run.RunChainRange(index, segment, g, error))
        return false;
    for (uint32_t m = segment;
         m < desc.members.size() && desc.segments[m] == segment; ++m) {
        const RigExecNodeDesc &d = layout.nodes[desc.members[m]];
        for (size_t p = 0; p < d.values.size(); ++p) {
            if (!(d.op->ports[p].flags & kRigExecPortOutput) ||
                d.values[p] == kRigExecInvalidIndex)
                continue;
            const uint32_t base = layout.groupBase[d.values[p]];
            if (desc.whole[m])
                for (uint32_t h = 0; h < groups; ++h)
                    ++state->groupVersions[base + h];
            else
                ++state->groupVersions[base + g];
        }
    }
    return true;
}

} // namespace eval
} // namespace rigExec

#endif
