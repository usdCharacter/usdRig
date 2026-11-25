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

#include <algorithm>
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

/// OpLookup::source: the family of the rig's values a lookup reads. The
/// numbers are part of the file format.
enum class OpLookupSource : uint8_t {
    Property = 0,
    PoseVersion = 1,
    WeightArrays = 2,
    /// The points entering revision `part` of point chain `object`, as one
    /// array.
    ChainVersion = 3,
    /// An input of weight object `part` as weight field `object` reads it.
    WeightFieldInputs = 4,
    /// An array of `part` elements, element i what lookup `object` + i
    /// answers.
    List = 5,
    /// Mat4d: the final (part 0) or base (part 1) matrix of slot `object`.
    Matrix = 6,
};

/// OpLookup::read of a WeightArrays lookup, whose object is a weight object:
/// which of its inputs, at the frame. The numbers are part of the file
/// format.
enum class OpWeightRead : uint8_t {
    DefaultWeight = 0,
    Driver = 1,
    Scale = 2,
    Bias = 3,
    Strength = 4,
    Invert = 5,
    FalloffMin = 6,
    FalloffMax = 7,
    /// Vec3f: the volume's x, y and z scale; its positive and negative
    /// scales.
    ScaleAxes = 8,
    PositiveScales = 9,
    NegativeScales = 10,
    ExtentU = 11,
    ExtentV = 12,
    /// The painted values and indices.
    Values = 13,
    Indices = 14,
    /// The sampled, target and curve points, each list's attributes one
    /// after another.
    SamplePoints = 15,
    TargetPoints = 16,
    CurvePoints = 17,
    /// The falloff remap: the authored samples, else the compiled curve.
    FalloffSamples = 18,
    /// UInt: the combine's target point count.
    CombineTargetCount = 19,
};

/// OpLookup::read of a WeightFieldInputs lookup. The numbers are part of
/// the file format.
enum class OpFieldRead : uint8_t {
    /// Float: the weight object's scalar inputs, by the field's member
    /// number: 0 defaultWeight, 1 driver, 2 scale, 3 bias, 4 strength, 5
    /// invert, 6 falloffMin, 7 falloffMax, 17 extentU, 18 extentV.
    LastScalar = 18,
    /// Vec3f: the x, y and z scale; the positive and the negative scales.
    ScaleAxes = 19,
    PositiveScales = 20,
    NegativeScales = 21,
    /// The points a volume samples, as many as the field's points.
    SamplePoints = 22,
    CurvePoints = 23,
    /// The falloff remap: the sampled falloff, else the compiled curve.
    FalloffSamples = 24,
    /// Mat4d: the volume's placement at the field's placement phase.
    Placement = 25,
    /// UInt: how many points the field weighs.
    TargetCount = 26,
    /// The painted values and indices.
    Values = 27,
    Indices = 28,
    /// UInt: the plane's axis (0 x, 1 y, 2 z) and whether it is bounded.
    PlaneAxis = 29,
    PlaneBounded = 30,
};

/// The value of each scalar OpFieldRead a weight object does not author.
inline constexpr float kOpFieldScalarDefaults[19] = {
    0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

/// OpHostOutput::target: the family of host slots a port value is stored
/// in. The numbers are part of the file format.
enum class OpOutputTarget : uint8_t {
    PropertyVersion = 0,
    VolumePlacement = 1,
    VolumePlacementBase = 2,
    WeightPacket = 3,
    /// A range chain member's output, group by group, into the host's
    /// vertex-group state.
    ChainGroup = 4,
    /// The weight field `object`'s weights.
    WeightField = 5,
};

/// Whether OpLookUp resolves lookups of \p source.
constexpr bool
OpLookupSourceRuns(uint8_t source)
{
    return source == uint8_t(OpLookupSource::Property) ||
           source == uint8_t(OpLookupSource::PoseVersion) ||
           source == uint8_t(OpLookupSource::WeightArrays) ||
           source == uint8_t(OpLookupSource::ChainVersion) ||
           source == uint8_t(OpLookupSource::WeightFieldInputs) ||
           source == uint8_t(OpLookupSource::List) ||
           source == uint8_t(OpLookupSource::Matrix);
}

/// Whether OpStoreOutput stores outputs of \p target, or, for a range chain
/// member, the host's StoreGroup does.
constexpr bool
OpOutputTargetRuns(uint8_t target, bool chainMember)
{
    return chainMember
               ? target == uint8_t(OpOutputTarget::ChainGroup)
               : target == uint8_t(OpOutputTarget::PropertyVersion) ||
                     target == uint8_t(OpOutputTarget::VolumePlacement) ||
                     target == uint8_t(OpOutputTarget::VolumePlacementBase) ||
                     target == uint8_t(OpOutputTarget::WeightPacket) ||
                     target == uint8_t(OpOutputTarget::WeightField);
}

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
    /// Per hostInputs entry: the program input it writes, and how a range
    /// chain member reads it group by group: 0 not, 1 through a pointwise
    /// input, 2 through a neighbourhood input. Chain points and a
    /// ChainVersion lookup feeding a point input are read group by group.
    std::vector<uint32_t> hostInputIds;
    std::vector<uint8_t> grouped;
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
    layout->grouped.clear();
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
        const uint32_t flags =
            layout->nodes[input.node].op->ports[input.port].flags;
        const bool member = layout->chainOf[input.node] != kRigExecInvalidIndex;
        const bool points =
            (flags & (kRigExecPortPointwise | kRigExecPortNeighbourhood)) != 0;
        const uint8_t grouped =
            (flags & kRigExecPortNeighbourhood) ? uint8_t(2) : uint8_t(1);
        layout->grouped.push_back(0);
        if (input.source == OpHostSource::ChainPoints) {
            if (!member || !points)
                return OpLayoutFail(error,
                                    "chain points feed node " +
                                        std::to_string(input.node) +
                                        ", which does not read them point "
                                        "by point in a range chain");
            layout->grouped.back() = grouped;
        }
        if (input.source == OpHostSource::Lookup) {
            if (input.object >= layout->lookups.size())
                return OpLayoutFail(
                    error, "node " + std::to_string(input.node) +
                               " reads lookup " + std::to_string(input.object) +
                               ", which the layout lacks");
            if (!OpLookupSourceRuns(layout->lookups[input.object].source))
                return OpLayoutFail(
                    error,
                    "node " + std::to_string(input.node) +
                        " reads lookup source " +
                        std::to_string(layout->lookups[input.object].source) +
                        ", which no host resolves");
            const OpLookup &lookup = layout->lookups[input.object];
            const bool version =
                lookup.source == uint8_t(OpLookupSource::ChainVersion);
            if (points && !(version && member))
                return OpLayoutFail(error, "a lookup feeds point input " +
                                               std::to_string(input.port) +
                                               " of node " +
                                               std::to_string(input.node));
            if (flags & kRigExecPortChecked)
                return OpLayoutFail(error, "a lookup feeds checked input " +
                                               std::to_string(input.port) +
                                               " of node " +
                                               std::to_string(input.node));
            if (lookup.source == uint8_t(OpLookupSource::List)) {
                const RigExecTypeDesc &type =
                    *RigExecRegistry::Get()
                         .TypeAt(layout->nodes[input.node]
                                     .op->ports[input.port]
                                     .type->Index())
                         .desc;
                bool listed = lookup.part >= 0 && type.element && type.resize &&
                              type.data &&
                              RigExecRegistry::Get()
                                  .TypeAt(type.element->Index())
                                  .desc->triviallyCopyable &&
                              size_t(lookup.object) + size_t(lookup.part) <=
                                  layout->lookups.size();
                for (int32_t e = 0; listed && e < lookup.part; ++e) {
                    const uint8_t source =
                        layout->lookups[lookup.object + uint32_t(e)].source;
                    listed = OpLookupSourceRuns(source) &&
                             source != uint8_t(OpLookupSource::List);
                }
                if (!listed)
                    return OpLayoutFail(
                        error, "node " + std::to_string(input.node) +
                                   " reads lookup list " +
                                   std::to_string(input.object) +
                                   ", whose elements the layout lacks or "
                                   "its input cannot hold");
            }
            if (points)
                layout->grouped.back() = grouped;
            else
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
        const bool member =
            layout->chainOf[output.node] != kRigExecInvalidIndex;
        if (!OpOutputTargetRuns(output.target, member) ||
            (member &&
             !(layout->nodes[output.node].op->ports[output.port].flags &
               kRigExecPortOutput)))
            return OpLayoutFail(
                error, "node " + std::to_string(output.node) + " stores port " +
                           std::to_string(output.port) + " in output target " +
                           std::to_string(output.target) +
                           ", which no host stores");
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
    /// Per hostInputs entry read through a neighbourhood input, per group:
    /// the groups table that group's range reads and its RigExecOpGroups;
    /// and the groups it reads, found by the chain's head.
    std::vector<std::vector<std::vector<const void *>>> haloPoints;
    std::vector<std::vector<RigExecOpGroups>> haloGroups;
    std::vector<std::vector<std::vector<uint32_t>>> haloReads;
    std::vector<uint64_t> versions, groupVersions;
};

/// Group \p g of the points host input \p input reads group by group: of
/// its chain's final points, or of the chain version its lookup names.
template <class Host>
const void *
OpGroupPoints(const OpStepLayout &layout, Host &host, const OpHostInput &input,
              uint32_t g)
{
    return input.source == OpHostSource::ChainPoints
               ? host.ChainGroupPoints(input.object, g)
               : host.ChainVersionGroupPoints(layout.lookups[input.object], g);
}

/// Points \p state's run at the host's values: each matrix input at what
/// \p host.Matrix answers, each input read group by group at a group
/// table its range steps fill, one per group for a neighbourhood input.
/// Call it on a new or cloned state, before its first step, whenever the
/// host's storage moved; a clone takes its source's haloReads.
template <class Host>
bool
BindOpStepState(const OpStepLayout &layout, Host &host, OpStepState *state,
                std::string *error)
{
    state->groupPoints.assign(layout.hostInputs.size(),
                              std::vector<const void *>());
    state->groups.assign(layout.hostInputs.size(), RigExecOpGroups());
    state->haloPoints.assign(layout.hostInputs.size(), {});
    state->haloGroups.assign(layout.hostInputs.size(), {});
    state->haloReads.resize(layout.hostInputs.size());
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
        if (layout.grouped[i] == 2) {
            const OpChainLayout &chain =
                layout.chains[layout.chainOf[input.node]];
            const uint32_t groups = chain.Groups();
            state->haloPoints[i].assign(
                groups, std::vector<const void *>(groups, nullptr));
            state->haloGroups[i].assign(groups, RigExecOpGroups());
            for (uint32_t g = 0; g < groups; ++g) {
                state->haloPoints[i][g][g] =
                    OpGroupPoints(layout, host, input, g);
                state->haloGroups[i][g] = {chain.bounds.data(), groups,
                                           state->haloPoints[i][g].data()};
            }
            if (!state->run->PointInputHaloGroups(
                    id, state->haloGroups[i].data(), error))
                return false;
            continue;
        }
        if (input.source == OpHostSource::Lookup && !layout.grouped[i]) {
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
        if (!layout.grouped[i]) {
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
            state->groupPoints[i][g] = OpGroupPoints(layout, host, input, g);
        state->groups[i] = {chain.bounds.data(), chain.Groups(),
                            state->groupPoints[i].data()};
        if (!state->run->PointInputGroups(id, &state->groups[i], error))
            return false;
    }
    return true;
}

/// Resolves lookup \p index through the \p host method of its source's
/// family: LookUpProperty, LookUpPoseVersion, LookUpWeight,
/// LookUpChainVersion or LookUpWeightField(index, lookup, type, value,
/// error), or a Matrix lookup through Matrix(source, slot). A List lookup
/// resolves through OpLookUpAt.
template <class Host>
bool
OpLookUp(Host &host, uint32_t index, const OpLookup &lookup,
         const RigExecTypeDesc &type, void *value, std::string *error)
{
    switch (OpLookupSource(lookup.source)) {
    case OpLookupSource::Property:
        return host.LookUpProperty(index, lookup, type, value, error);
    case OpLookupSource::PoseVersion:
        return host.LookUpPoseVersion(index, lookup, type, value, error);
    case OpLookupSource::WeightArrays:
        return host.LookUpWeight(index, lookup, type, value, error);
    case OpLookupSource::ChainVersion:
        return host.LookUpChainVersion(index, lookup, type, value, error);
    case OpLookupSource::WeightFieldInputs:
        return host.LookUpWeightField(index, lookup, type, value, error);
    case OpLookupSource::Matrix: {
        const void *matrix = host.Matrix(
            lookup.part ? OpHostSource::BaseMatrix : OpHostSource::FinalMatrix,
            lookup.object);
        if (!matrix || !type.triviallyCopyable ||
            type.size != 16 * sizeof(double))
            return OpLayoutFail(error, "slot " + std::to_string(lookup.object) +
                                           " has no such matrix");
        std::memcpy(value, matrix, type.size);
        return true;
    }
    case OpLookupSource::List:
        break;
    }
    return OpLayoutFail(error, "no host resolves lookup source " +
                                   std::to_string(lookup.source));
}

/// Stores \p value through the \p host method of \p output's target
/// family: StorePropertyVersion, StoreVolumePlacement, StoreWeightPacket
/// or StoreWeightField(output, value, error).
template <class Host>
bool
OpStoreOutput(Host &host, const OpHostOutput &output, const void *value,
              std::string *error)
{
    switch (OpOutputTarget(output.target)) {
    case OpOutputTarget::PropertyVersion:
        return host.StorePropertyVersion(output, value, error);
    case OpOutputTarget::VolumePlacement:
    case OpOutputTarget::VolumePlacementBase:
        return host.StoreVolumePlacement(output, value, error);
    case OpOutputTarget::WeightPacket:
        return host.StoreWeightPacket(output, value, error);
    case OpOutputTarget::WeightField:
        return host.StoreWeightField(output, value, error);
    case OpOutputTarget::ChainGroup:
        break;
    }
    return OpLayoutFail(error, "no host stores output target " +
                                   std::to_string(output.target));
}

/// Resolves lookup \p index of \p layout: a List lookup element by element
/// into an array of its length, any other through OpLookUp.
template <class Host>
bool
OpLookUpAt(const OpStepLayout &layout, Host &host, uint32_t index,
           const RigExecTypeDesc &type, void *value, std::string *error)
{
    const OpLookup &lookup = layout.lookups[index];
    if (lookup.source != uint8_t(OpLookupSource::List))
        return OpLookUp(host, index, lookup, type, value, error);
    const RigExecTypeDesc &element =
        *RigExecRegistry::Get().TypeAt(type.element->Index()).desc;
    type.resize(value, size_t(lookup.part));
    auto *elements = static_cast<uint8_t *>(type.data(value));
    for (uint32_t e = 0; e < uint32_t(lookup.part); ++e)
        if (!OpLookUp(host, lookup.object + e,
                      layout.lookups[lookup.object + e], element,
                      elements + e * element.size, error))
            return false;
    return true;
}

/// Resolves node \p node's Lookup inputs in layout order, storing a
/// hostOutput of a looked-up input before the next lookup.
template <class Host>
bool
ResolveOpLookups(const OpStepLayout &layout, Host &host, OpStepState *state,
                 size_t node, std::string *error)
{
    for (const uint32_t i : layout.lookupsOf[node]) {
        OpPortValue &value = *state->lookedUp[i];
        const uint32_t lookup = layout.hostInputs[i].object;
        if (!OpLookUpAt(layout, host, lookup, value.Type(), value.Value(),
                        error))
            return false;
        for (const uint32_t o : layout.hostOutputsOf[node])
            if (layout.hostOutputs[o].port == layout.hostInputs[i].port &&
                !OpStoreOutput(host, layout.hostOutputs[o], value.Value(),
                               error))
                return false;
    }
    return true;
}

/// Runs op step (\p kind, \p object, \p part): node \p object, after its
/// lookups resolved (ResolveOpLookups), its outputs' hostOutputs stored
/// after it ran; chain \p object's head, after its members' lookups
/// resolved; or group part % G of the segment starting at member part / G
/// of chain \p object, each of the segment's hostOutputs stored for the
/// groups it wrote by \p host.StoreGroup(output, group, elements, count,
/// error). False with the reason in \p error. Steps that do not feed each
/// other may run at once.
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
        if (!ResolveOpLookups(layout, host, state, index, error))
            return false;
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
            if (!OpStoreOutput(host, output, value, error))
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
    const RigExecChainDesc &desc = layout.chainDescs[index];
    if (kind == OpStepKind::ChainHead) {
        for (const RigExecNodeId member : desc.members)
            if (!ResolveOpLookups(layout, host, state, member, error))
                return false;
        if (!run.RunChainHead(index, chain.bounds.data(), groups, error))
            return false;
        for (size_t i = 0; i < layout.hostInputs.size(); ++i) {
            const uint32_t node = layout.hostInputs[i].node;
            if (layout.grouped[i] != 2 || layout.chainOf[node] != index)
                continue;
            state->haloReads[i].resize(groups);
            for (uint32_t g = 0; g < groups; ++g) {
                size_t lo = 0, hi = 0;
                run.ChainReads(index, layout.memberOf[node], g, &lo, &hi);
                std::vector<uint32_t> &reads = state->haloReads[i][g];
                reads.clear();
                for (uint32_t h = 0; h < groups; ++h)
                    if (h == g || (size_t(chain.bounds[h]) < hi &&
                                   size_t(chain.bounds[h + 1]) > lo))
                        reads.push_back(h);
            }
        }
        ++state->versions[layout.PlanSlot(index)];
        return true;
    }
    if (part < 0 || !groups)
        return OpLayoutFail(error, "an op range step names no group");
    const uint32_t segment = uint32_t(part) / groups;
    const uint32_t g = uint32_t(part) % groups;
    for (size_t i = 0; i < layout.hostInputs.size(); ++i) {
        const OpHostInput &input = layout.hostInputs[i];
        if (!layout.grouped[i] || layout.chainOf[input.node] != index ||
            desc.segments[layout.memberOf[input.node]] != segment)
            continue;
        if (layout.grouped[i] == 1) {
            state->groupPoints[i][g] = OpGroupPoints(layout, host, input, g);
            continue;
        }
        std::vector<const void *> &table = state->haloPoints[i][g];
        std::fill(table.begin(), table.end(), nullptr);
        if (desc.whole[layout.memberOf[input.node]]) {
            for (uint32_t h = 0; h < groups; ++h)
                table[h] = OpGroupPoints(layout, host, input, h);
            continue;
        }
        table[g] = OpGroupPoints(layout, host, input, g);
        if (g < state->haloReads[i].size())
            for (const uint32_t h : state->haloReads[i][g])
                table[h] = OpGroupPoints(layout, host, input, h);
    }
    if (!run.RunChainRange(index, segment, g, error))
        return false;
    for (uint32_t m = segment;
         m < desc.members.size() && desc.segments[m] == segment; ++m) {
        const RigExecNodeDesc &d = layout.nodes[desc.members[m]];
        for (const uint32_t o : layout.hostOutputsOf[desc.members[m]]) {
            const OpHostOutput &output = layout.hostOutputs[o];
            RigExecValueRef ref;
            ref.value = d.values[output.port];
            for (uint32_t h = desc.whole[m] ? 0 : g;
                 h < (desc.whole[m] ? groups : g + 1); ++h) {
                size_t count = 0;
                const void *elements = run.GetGroupValue(ref, h, &count);
                if (!elements && count)
                    return OpLayoutFail(
                        error, "node " + std::to_string(desc.members[m]) +
                                   " holds no group " + std::to_string(h) +
                                   " at port " + std::to_string(output.port));
                if (!host.StoreGroup(output, h, elements, count, error))
                    return false;
            }
        }
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
