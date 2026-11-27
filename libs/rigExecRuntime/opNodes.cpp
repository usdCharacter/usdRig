#include "rigExecRuntime/opNodes.h"

#include "rigExec/api/opFields.h"

#include "rigExec/api/compile.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/store.h"
#include "rigExecEvalCore/constraintLookups.h"
#include "rigExecEvalCore/propertyLookups.h"
#include "rigExecSchema/schemaNames.h"

#include <cstring>
#include <utility>

namespace rigExec {

namespace {

bool
Fail(std::string *error, std::string why)
{
    if (error)
        *error = "op nodes: " + std::move(why);
    return false;
}

// What the validator cannot see without the op registry: every op value a
// step or the common graph names is one the laid-out program has; a range
// chain member reads its own point chain's groups; a whole ChainVersion
// lookup belongs to a field node; and each range chain runs as one head and
// one range step per (segment, group) a member writes, as native emits.
bool
CheckOpSteps(const RrProgram &p, const eval::OpStepLayout &layout,
             const std::vector<char> &fieldNode, std::string *error)
{
    const auto width = [&](fb::SlotDomain domain) -> uint64_t {
        return domain == fb::SlotDomain::OpValue        ? layout.OpValueCount()
               : domain == fb::SlotDomain::OpGroupValue ? layout.groupValueCount
                                                        : UINT64_MAX;
    };
    for (size_t i = 0; i < p.steps->size(); ++i)
        for (const auto *ranges : {&(*p.steps)[i].reads, &(*p.steps)[i].writes})
            for (const fb::SlotRange &range : *ranges)
                if (range.end() > width(range.domain()))
                    return Fail(error, "step " + std::to_string(i) +
                                           " names an op value past the "
                                           "program's");
    if (p.file->commonGraph)
        for (const auto &spec : p.file->commonGraph->valueSpecs)
            if (spec.slot >= width(fb::SlotDomain(spec.domain)))
                return Fail(error, "the common graph names an op value past "
                                   "the program's");
    for (size_t i = 0; i < layout.hostInputs.size(); ++i) {
        const eval::OpHostInput &input = layout.hostInputs[i];
        const bool version = input.source == eval::OpHostSource::Lookup &&
                             layout.lookups[input.object].source ==
                                 uint8_t(eval::OpLookupSource::ChainVersion);
        if (version && !layout.grouped[i] && !fieldNode[input.node])
            return Fail(error, "node " + std::to_string(input.node) +
                                   " looks up a whole chain version outside "
                                   "a weight field");
        if (!layout.grouped[i])
            continue;
        const uint32_t chain = input.source == eval::OpHostSource::ChainPoints
                                   ? input.object
                                   : layout.lookups[input.object].object;
        if (chain != layout.chains[layout.chainOf[input.node]].pointChain)
            return Fail(error, "node " + std::to_string(input.node) +
                                   " reads the groups of point chain " +
                                   std::to_string(chain) +
                                   ", not its range chain's");
    }
    std::vector<uint32_t> heads(layout.chains.size(), 0);
    std::vector<std::vector<char>> ranges(layout.chains.size());
    for (size_t c = 0; c < layout.chains.size(); ++c)
        ranges[c].assign(
            layout.chainDescs[c].members.size() * layout.chains[c].Groups(), 0);
    for (const auto &step : *p.steps) {
        if (step.kind != RigExecWireStepKind::OpChainHead &&
            step.kind != RigExecWireStepKind::OpChainRange)
            continue;
        const std::string at = "op chain " + std::to_string(step.object);
        if (step.object < 0 || size_t(step.object) >= layout.chains.size())
            return Fail(error, at + " is no range chain");
        const size_t c = size_t(step.object);
        if (step.kind == RigExecWireStepKind::OpChainHead) {
            if (heads[c]++)
                return Fail(error, at + " has two heads");
            continue;
        }
        const RigExecChainDesc &desc = layout.chainDescs[c];
        const uint32_t groups = layout.chains[c].Groups();
        const uint32_t s = step.part < 0 ? 0 : uint32_t(step.part) / groups;
        const uint32_t g = step.part < 0 ? 0 : uint32_t(step.part) % groups;
        if (step.part < 0 || s >= desc.members.size() ||
            desc.segments[s] != s || (desc.whole[s] && g) ||
            !eval::OpSegmentWrites(layout.chains[c], desc, s, g) ||
            ranges[c][size_t(step.part)]++)
            return Fail(error, at + " runs part " + std::to_string(step.part) +
                                   ", which is no group a segment writes or "
                                   "one run twice");
    }
    for (size_t c = 0; c < layout.chains.size(); ++c) {
        const RigExecChainDesc &desc = layout.chainDescs[c];
        const uint32_t groups = layout.chains[c].Groups();
        if (heads[c] != 1)
            return Fail(error,
                        "op chain " + std::to_string(c) + " has no head");
        for (uint32_t s = 0; s < desc.members.size(); ++s)
            for (uint32_t g = 0;
                 desc.segments[s] == s && g < (desc.whole[s] ? 1 : groups); ++g)
                if (eval::OpSegmentWrites(layout.chains[c], desc, s, g) &&
                    !ranges[c][size_t(s) * groups + g])
                    return Fail(error, "op chain " + std::to_string(c) +
                                           " runs group " + std::to_string(g) +
                                           " of segment " + std::to_string(s) +
                                           " in no step");
    }
    return true;
}

std::string
Text(const fb::RigExecWireManifestEntry &entry)
{
    return entry.nameSpace + ":" + entry.name + " v" +
           std::to_string(entry.version);
}

uint32_t
FindOp(const RigExecRegistry &registry,
       const fb::RigExecWireManifestEntry &entry)
{
    for (size_t i = 0; i < registry.OpCount(); ++i) {
        const RigExecOpDesc &desc = *registry.OpAt(uint32_t(i)).desc;
        if (entry.nameSpace == desc.nameSpace && entry.name == desc.name &&
            entry.version == desc.version)
            return uint32_t(i);
    }
    return kRigExecInvalidIndex;
}

uint32_t
FindType(const RigExecRegistry &registry, const char *nameSpace,
         const char *name, uint32_t major)
{
    for (size_t i = 0; i < registry.TypeCount(); ++i) {
        const RigExecTypeDesc &desc = *registry.TypeAt(uint32_t(i)).desc;
        if (std::strcmp(nameSpace, desc.nameSpace) == 0 &&
            std::strcmp(name, desc.name) == 0 && major == desc.major)
            return uint32_t(i);
    }
    return kRigExecInvalidIndex;
}

/// A node's settings decoded into owned storage for AddOp, which copies
/// them.
class Payload {
public:
    explicit Payload(const RigExecOpPayloadHandling &h)
        : _h(h), _storage(h.size / sizeof(std::max_align_t) + 1)
    {
        _h.construct(_storage.data());
    }
    ~Payload()
    {
        _h.destroy(_storage.data());
    }
    Payload(const Payload &) = delete;
    Payload &operator=(const Payload &) = delete;
    void *Data()
    {
        return _storage.data();
    }

private:
    const RigExecOpPayloadHandling &_h;
    std::vector<std::max_align_t> _storage;
};

/// What the runtime's op steps read from the program and store in it.
struct RrOpHost {
    RrProgram &p;
    size_t step = size_t(-1);
    mutable eval::ConstraintStepCache constraintCache;

    const void *Matrix(eval::OpHostSource source, uint32_t slot) const
    {
        const std::vector<RrMat4d> &matrices =
            source == eval::OpHostSource::FinalMatrix ? p.store.finalMatrix
                                                      : p.store.baseMatrix;
        return slot < matrices.size() ? &matrices[slot] : nullptr;
    }
    const void *PresentMatrix(const eval::OpLookup &lookup) const
    {
        const bool record =
            lookup.source == uint8_t(eval::OpLookupSource::FrameRecord);
        const std::vector<RrMat4d> &matrices =
            record ? p.store.frameMatrix : p.store.deltaValues;
        const std::vector<char> &present =
            record ? p.store.frameMatrixValid : p.store.deltaPresent;
        return lookup.object < matrices.size() &&
                       lookup.object < present.size() && present[lookup.object]
                   ? &matrices[lookup.object]
                   : nullptr;
    }
    bool RevisionLeaf(uint32_t id, bool *enabled, float *weight, bool *radial,
                      std::string *error) const
    {
        return RrGeoRevisionLeaf(&p, id, enabled, weight, radial, error);
    }
    bool LookUpRevision(uint32_t, const eval::OpLookup &lookup,
                        const RigExecTypeDesc &type, void *value,
                        std::string *error) const
    {
        return RrGeoLookUpRevision(&p, lookup, type, value, error);
    }
    void LookupGroups(const eval::OpLookup &lookup, const uint32_t **groups,
                      size_t *count) const
    {
        RrGeoLookupGroups(&p, lookup, groups, count);
    }
    bool StoreRevisionEnabled(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RrGeoStoreRevisionEnabled(
            &p, output.object, *static_cast<const uint8_t *>(value) != 0,
            error);
    }
    bool RevisionChainHasBase(uint32_t revision) const
    {
        return RrGeoRevisionChainHasBase(&p, revision);
    }
    const void *ChainGroupPoints(uint32_t chain, uint32_t g) const
    {
        return RrGeoFinalGroupPoints(&p, chain, g);
    }
    const void *ChainVersionGroupPoints(const eval::OpLookup &lookup,
                                        uint32_t g) const
    {
        return lookup.part < 0 ? nullptr
                               : RrGeoGroupPointsAt(&p, lookup.object,
                                                    size_t(lookup.part), g);
    }
    bool LookUpProperty(uint32_t, const eval::OpLookup &lookup,
                        const RigExecTypeDesc &type, void *value,
                        std::string *error) const
    {
        return RrLookUpProperty(&p, lookup, type, value, error);
    }
    bool StorePropertyVersion(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RrStorePropertyVersion(&p, output.object, value, error);
    }
    bool LookUpPoseVersion(uint32_t, const eval::OpLookup &lookup,
                           const RigExecTypeDesc &type, void *value,
                           std::string *error) const
    {
        return RrLookUpPoseVersion(&p, lookup, type, value, error);
    }
    bool StoreVolumePlacement(const eval::OpHostOutput &output,
                              const void *value, std::string *error) const
    {
        return RrStoreVolumePlacement(&p, output, value, error);
    }
    bool LookUpWeight(uint32_t, const eval::OpLookup &lookup,
                      const RigExecTypeDesc &type, void *value,
                      std::string *error) const
    {
        return RrLookUpWeight(&p, lookup, type, value, error);
    }
    bool StoreWeightPacket(const eval::OpHostOutput &output, const void *value,
                           std::string *error) const
    {
        return RrStoreWeightPacket(&p, output, value, error);
    }
    bool LookUpChainVersion(uint32_t index, const eval::OpLookup &lookup,
                            const RigExecTypeDesc &type, void *value,
                            std::string *error) const
    {
        std::vector<std::vector<RrVec3f>> &gather =
            p.opNodes->chainVersionGather;
        if (index >= gather.size()) {
            if (error)
                *error = "no lookup " + std::to_string(index);
            return false;
        }
        return RrLookUpChainVersion(&p, lookup, &gather[index], type, value,
                                    error);
    }
    bool LookUpWeightField(uint32_t, const eval::OpLookup &lookup,
                           const RigExecTypeDesc &type, void *value,
                           std::string *error) const
    {
        return RrLookUpWeightField(&p, lookup, type, value, error);
    }
    bool StoreWeightField(const eval::OpHostOutput &output, const void *value,
                          std::string *error) const
    {
        return RrStoreWeightField(&p, output, value, error);
    }
    bool LookUpSpace(uint32_t, const eval::OpLookup &lookup,
                     const RigExecTypeDesc &type, void *value,
                     std::string *error) const
    {
        return RrLookUpSpace(&p, lookup, type, value, error);
    }
    bool StoreSpace(const eval::OpHostOutput &output, const void *value,
                    std::string *error) const
    {
        return RrStoreSpace(&p, step, output, value, error);
    }
    bool StoreGroup(const eval::OpHostOutput &output, uint32_t g,
                    const void *elements, size_t count,
                    std::string *error) const
    {
        return RrStoreChainGroup(&p, output, g, elements, count, error);
    }
    bool LookUpCommit(uint32_t, const eval::OpLookup &lookup,
                      const RigExecTypeDesc &type, void *value,
                      std::string *error) const
    {
        return RrLookUpCommit(&p, lookup, type, value, error);
    }
    bool StoreCommit(const eval::OpHostOutput &output, const void *value,
                     std::string *error) const
    {
        return RrStoreCommit(&p, output, value, error);
    }
    bool LookUpConstraint(uint32_t, const eval::OpLookup &lookup,
                          const RigExecTypeDesc &type, void *value,
                          std::string *error) const
    {
        return RrLookUpConstraint(&p, constraintCache, lookup, type, value,
                                  error);
    }
    bool LendWindow(RigExecRunState &run, uint32_t node,
                    std::string *error) const
    {
        const int32_t w =
            step < p.windowOfStep.size() ? p.windowOfStep[step] : -1;
        if (w < 0 || size_t(w) >= p.store.windowRings.size() ||
            size_t(w) >= p.store.windowCaptures.size())
            return Fail(error, "node " + std::to_string(node) +
                                   " keeps no history window");
        return run.LendWindow(
            RigExecNodeId(node), p.store.windowRings[size_t(w)],
            p.store.windowCaptures[size_t(w)],
            p.store.historyTick && !p.windowedOps[size_t(w)].feedsWindow,
            error);
    }
    bool StoreConstraint(const eval::OpHostOutput &output, const void *value,
                         std::string *error) const
    {
        return RrStoreConstraint(&p, step, constraintCache, output, value,
                                 error);
    }
    bool LookUpCompose(uint32_t, const eval::OpLookup &lookup,
                       const RigExecTypeDesc &type, void *value,
                       std::string *error) const
    {
        return RrLookUpCompose(&p, lookup, type, value, error);
    }
    bool StoreCompose(const eval::OpHostOutput &output, const void *value,
                      std::string *error) const
    {
        return RrStoreCompose(&p, output, value, error);
    }
    bool LookUpSolver(uint32_t, const eval::OpLookup &lookup,
                      const RigExecTypeDesc &type, void *value,
                      std::string *error) const
    {
        return RrLookUpSolver(&p, lookup, type, value, error);
    }
    bool StoreSolver(const eval::OpHostOutput &output, const void *value,
                     std::string *error) const
    {
        return RrStoreSolver(&p, step, output, value, error);
    }
};

} // namespace

bool
RrOpenOpNodes(RrProgram *p, std::string *error)
{
    const fb::RigExecWireFile *file = p->file.get();
    if (!file || !file->opGraph)
        return true;
    if (!RigExecRegistry::IsInitialized())
        return Fail(error, "the file holds op nodes; open it with the op "
                           "and type libraries");
    const RigExecRegistry &registry = RigExecRegistry::Get();
    const fb::RigExecWireOpGraph &graph = *file->opGraph;
    const fb::RigExecWireManifest &manifest = *file->manifest;

    std::vector<uint32_t> ops, types;
    for (const auto &entry : manifest.coreOps) {
        ops.push_back(FindOp(registry, *entry));
        if (ops.back() == kRigExecInvalidIndex)
            return Fail(error, "op " + Text(*entry) +
                                   " is in no op library this process "
                                   "registered");
    }
    for (const auto &entry : manifest.coreTypes) {
        types.push_back(FindType(registry, entry->nameSpace.c_str(),
                                 entry->name.c_str(), entry->version));
        if (types.back() == kRigExecInvalidIndex)
            return Fail(error, "type " + Text(*entry) +
                                   " is in no type library this process "
                                   "registered");
    }

    auto nodes = std::make_unique<RrOpNodes>();
    eval::OpStepLayout &layout = nodes->layout;
    RigExecRigDescBuilder builder;
    const auto opOf = [&](size_t node) -> const RigExecRegisteredOp & {
        return registry.OpAt(ops[graph.nodes[node]->op]);
    };
    const auto port = [&](size_t node, uint32_t index, uint32_t flag,
                          const RigExecPortHandle **out) {
        const RigExecOpDesc &desc = *opOf(node).desc;
        if (index >= desc.portCount || !(desc.ports[index].flags & flag))
            return Fail(error,
                        "node " + std::to_string(node) + " (" + desc.name +
                            ") has no " +
                            (flag == kRigExecPortInput ? "input" : "output") +
                            " port " + std::to_string(index));
        *out = desc.ports[index].port;
        return true;
    };
    for (size_t n = 0; n < graph.nodes.size(); ++n) {
        const fb::RigExecWireOpNode &node = *graph.nodes[n];
        const RigExecRegisteredOp &op = opOf(n);
        const RigExecOpPayloadHandling &handling = op.desc->payload;
        const std::string where =
            "node " + std::to_string(n) + " (" + op.desc->name + ")";
        if (op.desc->payloadFieldCount) {
            RigExecFieldValues values;
            std::string why;
            if (!RigExecDecodePayloadFields(*op.desc, node.payload.data(),
                                            node.payload.size(), &values,
                                            &why) ||
                !RigExecCheckPayloadFields(*op.desc, values, &why))
                return Fail(error, where + ": settings do not decode: " + why);
            builder.AddOp(*op.handle, values);
            continue;
        }
        if (!handling.size) {
            if (!node.payload.empty())
                return Fail(error, where + " has settings its op does not "
                                           "take");
            builder.AddOp(*op.handle);
            continue;
        }
        if (!handling.decode)
            return Fail(error, where + ": its op cannot read saved settings");
        Payload payload(handling);
        std::string why;
        if (!handling.decode(node.payload.data(), node.payload.size(),
                             payload.Data(), &why))
            return Fail(error, where + ": settings do not decode: " + why);
        builder.AddOp(*op.handle, payload.Data());
    }
    std::vector<std::pair<uint32_t, uint32_t>> fieldObjects;
    for (size_t n = 0; n < graph.nodes.size(); ++n) {
        for (const auto &input : graph.nodes[n]->inputs) {
            const RigExecPortHandle *in = nullptr;
            if (!port(n, input->port, kRigExecPortInput, &in))
                return false;
            if (input->node >= 0) {
                const RigExecPortHandle *out = nullptr;
                if (!port(size_t(input->node), input->output,
                          kRigExecPortOutput, &out))
                    return false;
                builder.Connect(RigExecNodeId(input->node), *out,
                                RigExecNodeId(n), *in);
                continue;
            }
            builder.BindInput(RigExecNodeId(n), *in);
            if (input->host == 3) {
                eval::OpConstant constant;
                constant.node = uint32_t(n);
                constant.port = input->port;
                constant.bytes = input->constant;
                layout.constants.push_back(std::move(constant));
                continue;
            }
            eval::OpHostInput host;
            host.node = uint32_t(n);
            host.port = input->port;
            host.source = eval::OpHostSource(input->host);
            host.object = input->object;
            layout.hostInputs.push_back(host);
            if (host.source == eval::OpHostSource::Lookup &&
                host.object < graph.lookups.size() &&
                graph.lookups[host.object]->source ==
                    uint8_t(eval::OpLookupSource::WeightFieldInputs) &&
                graph.lookups[host.object]->part >= 0)
                fieldObjects.emplace_back(
                    uint32_t(n), uint32_t(graph.lookups[host.object]->part));
        }
    }
    for (const auto &pair : graph.order)
        builder.Order(RigExecNodeId(pair.first), RigExecNodeId(pair.second));
    for (const auto &output : graph.outputs) {
        const RigExecPortHandle *out = nullptr;
        if (!port(output->node, output->port, kRigExecPortOutput, &out))
            return false;
        const uint32_t type = opOf(output->node).portTypes[output->port];
        if (types[output->type] != type)
            return Fail(error, "output " + output->name +
                                   " is not of its port's type");
        RrOpNodeOutput published;
        published.name = output->name;
        published.id = builder.BindOutput(output->node, *out);
        published.type = type;
        nodes->outputs.push_back(std::move(published));
    }
    for (const auto &lookup : graph.lookups)
        layout.lookups.push_back(
            {lookup->source, lookup->object, lookup->part, lookup->read});
    nodes->propertyChain.assign(graph.nodes.size(), kRigExecInvalidIndex);
    nodes->propertyPart.assign(graph.nodes.size(), kRigExecInvalidIndex);
    nodes->weightObject.assign(graph.nodes.size(), kRigExecInvalidIndex);
    nodes->commit.assign(graph.nodes.size(), kRigExecInvalidIndex);
    nodes->constraint.assign(graph.nodes.size(), kRigExecInvalidIndex);
    nodes->solver.assign(graph.nodes.size(), kRigExecInvalidIndex);
    for (size_t n = 0; n < graph.nodes.size(); ++n)
        for (const auto &input : graph.nodes[n]->inputs)
            if (input && input->host == 4 &&
                input->object < graph.lookups.size() &&
                graph.lookups[input->object]->source ==
                    uint8_t(eval::OpLookupSource::SolverInput))
                nodes->solver[n] = graph.lookups[input->object]->object;
    if (p->poses) {
        nodes->solverLeading.assign(p->poses->solvers.size(), 0);
        for (size_t s = 0; s < p->poses->solvers.size(); ++s)
            nodes->solverLeading[s] =
                p->poses->solvers[s].start >= 0 &&
                        p->TextOrEmpty(p->poses->solvers[s].type) ==
                            SchemaTypeName::RigExecFkChain
                    ? 1
                    : 0;
    }
    // A field node's first field lookup in port order reads its own object;
    // a sparse dynamic weight's base indices read its base's, later.
    for (const auto &[node, object] : fieldObjects)
        if (nodes->weightObject[node] == kRigExecInvalidIndex)
            nodes->weightObject[node] = object;
    for (const auto &output : graph.hostOutputs) {
        if (output->node >= graph.nodes.size())
            return Fail(error, "a stored value names no node");
        const RigExecOpDesc &desc = *opOf(output->node).desc;
        if (output->port >= desc.portCount)
            return Fail(error, "node " + std::to_string(output->node) +
                                   " has no port " +
                                   std::to_string(output->port));
        layout.hostOutputs.push_back(
            {output->node, output->port, output->target, output->object});
        if (!(desc.ports[output->port].flags & kRigExecPortOutput))
            continue;
        builder.BindOutput(output->node, *desc.ports[output->port].port);
        if (output->target == uint8_t(eval::OpOutputTarget::WeightPacket))
            nodes->weightObject[output->node] = output->object;
        if (output->target == uint8_t(eval::OpOutputTarget::CommitSlots))
            nodes->commit[output->node] = output->object;
        if (eval::OpConstraintOutputTarget(output->target))
            nodes->constraint[output->node] = output->object;
        if (output->target == uint8_t(eval::OpOutputTarget::SolverFrames))
            nodes->solver[output->node] = output->object;
        size_t chain = 0, part = 0;
        if (output->target == uint8_t(eval::OpOutputTarget::PropertyVersion) &&
            RrPropertyPartOfVersion(p, output->object, &chain, &part) &&
            part > 0) {
            nodes->propertyChain[output->node] = uint32_t(chain);
            nodes->propertyPart[output->node] = uint32_t(part);
        }
    }
    for (const auto &chain : graph.chains) {
        eval::OpChainLayout steps;
        steps.pointChain = chain->pointChain;
        steps.bounds = chain->bounds;
        steps.writes = chain->writes;
        layout.chains.push_back(std::move(steps));
    }
    std::string why;
    std::shared_ptr<const RigExecRigDesc> desc = builder.Build(&why);
    RigExecCompileOptions options;
    options.useSimd = p->geoSettings.useSimd;
    options.hostOrder = true;
    if (desc)
        layout.program = RigExecCompile(*desc, options, &why);
    if (!layout.program)
        return Fail(error, why);
    if (!eval::LayOutOpSteps(&layout, error))
        return false;
    for (size_t c = 0; c < graph.chains.size(); ++c)
        if (graph.chains[c]->members != layout.chainDescs[c].members)
            return Fail(error, "range chain " + std::to_string(c) +
                                   " has other members than the file names");
    for (const eval::OpHostOutput &output : layout.hostOutputs)
        if (output.target == uint8_t(eval::OpOutputTarget::ChainGroup) &&
            !RrGeoAdoptOpStored(p, output.object, error))
            return false;
    std::vector<char> fieldNode(graph.nodes.size(), 0);
    for (const auto &entry : fieldObjects)
        fieldNode[entry.first] = 1;
    if (!CheckOpSteps(*p, layout, fieldNode, error))
        return false;
    nodes->chainVersionGather.resize(layout.lookups.size());
    nodes->state = std::make_unique<eval::OpStepState>();
    nodes->state->run = RigExecRunState::Create(layout.program, {}, &why);
    if (!nodes->state->run)
        return Fail(error, why);
    const RrOpHost host{*p};
    if (!eval::BindOpStepState(layout, host, nodes->state.get(), error))
        return false;
    nodes->state->run->ReleaseOwner();
    p->opNodes = std::move(nodes);
    return true;
}

bool
RrRunOpStep(RrProgram *p, size_t i, std::string *error)
{
    RrOpNodes *nodes = p->opNodes.get();
    if (!nodes)
        return Fail(error, "an op step without op nodes");
    const fb::RigExecWireStep &step = (*p->steps)[i];
    const RrOpHost host{*p, i};
    RigExecNodeId failed = kRigExecInvalidIndex;
    if (eval::RunOpStep(nodes->layout, host, nodes->state.get(),
                        eval::OpStepKind(uint8_t(step.kind)), step.object,
                        step.part, error, &failed))
        return true;
    // A failing chain member speaks as the revision whose groups it stores.
    int64_t failedRevision = -1;
    if (failed < nodes->layout.hostOutputsOf.size())
        for (const uint32_t o : nodes->layout.hostOutputsOf[failed])
            if (nodes->layout.hostOutputs[o].target ==
                uint8_t(eval::OpOutputTarget::ChainGroup))
                failedRevision = int64_t(nodes->layout.hostOutputs[o].object);
    size_t chain = 0, part = 0;
    if (error && RrPropertyPartOf(p, step, &chain, &part))
        *error = RrPropertyRevisionLabel(p, chain, part) + ": " + *error;
    else if (error && step.kind == RigExecWireStepKind::OpNode &&
             step.object >= 0 && size_t(step.object) < nodes->commit.size() &&
             nodes->commit[size_t(step.object)] != kRigExecInvalidIndex)
        *error = RrCommitFailure(p, nodes->commit[size_t(step.object)], *error);
    else if (error && step.kind == RigExecWireStepKind::OpNode &&
             step.object >= 0 && size_t(step.object) < nodes->solver.size() &&
             nodes->solver[size_t(step.object)] != kRigExecInvalidIndex)
        *error = RrSolverFailure(p, nodes->solver[size_t(step.object)], *error);
    else if (error && step.kind == RigExecWireStepKind::OpNode &&
             step.object >= 0 &&
             size_t(step.object) < nodes->constraint.size() &&
             nodes->constraint[size_t(step.object)] != kRigExecInvalidIndex)
        *error = RrConstraintFailure(p, nodes->constraint[size_t(step.object)],
                                     *error);
    else if (error && step.kind == RigExecWireStepKind::OpNode &&
             step.object >= 0 &&
             size_t(step.object) < nodes->weightObject.size()) {
        const uint32_t object = nodes->weightObject[size_t(step.object)];
        if (p->geometry && object < p->geometry->weightObjects.size())
            *error = p->TextOrEmpty(p->geometry->weightObjects[object].path) +
                     ": " + *error;
    } else if (error && p->geometry) {
        // An op step standing in for a revision speaks as its mover.
        const auto &geo = *p->geometry;
        int64_t revision = failedRevision;
        for (const auto &write : step.writes) {
            if (revision >= 0)
                break;
            const auto domain = write.domain();
            if (domain == RigExecWireSlotDomain::RevisionPacket ||
                domain == RigExecWireSlotDomain::RevisionTransforms)
                revision = write.begin();
            else if (domain == RigExecWireSlotDomain::RevisionOut)
                for (size_t r = 0; r < geo.revisionChunkBase.size(); ++r)
                    if (geo.revisionChunkBase[r] >= 0 &&
                        write.begin() >= uint32_t(geo.revisionChunkBase[r]) &&
                        write.begin() - uint32_t(geo.revisionChunkBase[r]) <
                            uint32_t(geo.revisionChunkCount[r]))
                        revision = int64_t(r);
            if (revision >= 0)
                break;
        }
        if (revision >= 0 && size_t(revision) < geo.revisionIndex.size()) {
            const auto &at = geo.revisionIndex[size_t(revision)];
            *error = p->TextOrEmpty(geo.chains[size_t(at.first)]
                                        .revisions[size_t(at.second)]
                                        .moverPath) +
                     ": " + *error;
        }
    }
    return false;
}

bool
RrPropertyPartOf(const RrProgram *p, const RigExecWireStep &step, size_t *chain,
                 size_t *part)
{
    const RrOpNodes *nodes = p->opNodes.get();
    if (!nodes || step.kind != RigExecWireStepKind::OpNode || step.object < 0 ||
        size_t(step.object) >= nodes->propertyChain.size() ||
        nodes->propertyChain[size_t(step.object)] == kRigExecInvalidIndex)
        return false;
    if (chain)
        *chain = nodes->propertyChain[size_t(step.object)];
    if (part)
        *part = nodes->propertyPart[size_t(step.object)];
    return true;
}

uint64_t
RrOpValueVersion(const RrProgram *p, uint32_t domain, uint32_t slot)
{
    const RrOpNodes *nodes = p->opNodes.get();
    if (!nodes || !nodes->state)
        return 0;
    const std::vector<uint64_t> &versions =
        domain == uint32_t(RigExecWireSlotDomain::OpValue)
            ? nodes->state->versions
            : nodes->state->groupVersions;
    return slot < versions.size() ? versions[slot] : 0;
}

namespace {

bool
Registered(const RigExecRegistry &registry, const char *library, bool op)
{
    const size_t count = op ? registry.OpCount() : registry.TypeCount();
    for (size_t i = 0; i < count; ++i) {
        const char *name = op ? registry.OpAt(uint32_t(i)).library->name
                              : registry.TypeAt(uint32_t(i)).library->name;
        if (std::strcmp(name, library) == 0)
            return true;
    }
    return false;
}

} // namespace

std::unique_ptr<RigExecRuntimeReader>
RigExecRuntimeReader::Open(const uint8_t *bytes, size_t size,
                           const RigExecMoverDefinition *movers,
                           size_t moverCount,
                           const RigExecRegistryInit &opLibraries,
                           std::string *error)
{
    const auto fail = [error](std::string why) {
        if (error)
            *error = std::move(why);
        return std::unique_ptr<RigExecRuntimeReader>();
    };
    if (!RigExecRegistry::IsInitialized()) {
        std::vector<std::string> errors;
        if (!RigExecRegistry::Initialize(opLibraries, &errors))
            return fail("op libraries: " + (errors.empty()
                                                ? std::string("refused")
                                                : errors.front()));
    } else {
        const RigExecRegistry &registry = RigExecRegistry::Get();
        for (const auto *library : opLibraries.typeLibraries)
            if (library && !Registered(registry, library->name, false))
                return fail(std::string("type library ") + library->name +
                            " is not in this process's registry");
        for (const auto *library : opLibraries.opLibraries)
            if (library && !Registered(registry, library->name, true))
                return fail(std::string("op library ") + library->name +
                            " is not in this process's registry");
    }
    return Open(bytes, size, movers, moverCount, error);
}

size_t
RigExecRuntimeReader::GetOpOutputCount() const
{
    return _program->opNodes ? _program->opNodes->outputs.size() : 0;
}

bool
RigExecRuntimeReader::GetOpOutput(size_t index,
                                  RigExecRuntimeOpOutput *out) const
{
    if (index >= GetOpOutputCount())
        return false;
    const RrOpNodes &nodes = *_program->opNodes;
    const RrOpNodeOutput &output = nodes.outputs[index];
    out->name = output.name;
    out->type = output.type;
    RigExecValueRef ref;
    out->value = IsValid() && RigExecResolveOutput(*nodes.layout.program,
                                                   output.id, &ref)
                     ? nodes.state->run->GetValue(ref)
                     : nullptr;
    return true;
}

bool
RigExecRuntimeReader::FindOpOutput(const std::string &name, size_t *index) const
{
    for (size_t i = 0; i < GetOpOutputCount(); ++i)
        if (_program->opNodes->outputs[i].name == name) {
            *index = i;
            return true;
        }
    return false;
}

} // namespace rigExec
