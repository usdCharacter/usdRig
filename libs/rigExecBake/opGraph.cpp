#include "rigExecBake/opGraph.h"

#include "rigExec/internal/bakedProgramImpl.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

std::unique_ptr<fb::RigExecWireManifestEntry>
Entry(const char *nameSpace, const char *name, uint32_t version)
{
    auto entry = std::make_unique<fb::RigExecWireManifestEntry>();
    entry->nameSpace = nameSpace;
    entry->name = name;
    entry->version = version;
    return entry;
}

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

uint32_t
OutputPortOf(const eval::OpStepLayout &layout, uint32_t producer, uint32_t v)
{
    const RigExecNodeDesc &from = layout.nodes[producer];
    uint32_t port = 0;
    for (uint32_t o = 0; o < from.values.size(); ++o)
        if ((from.op->ports[o].flags & kRigExecPortOutput) &&
            from.values[o] == v)
            port = o;
    return port;
}

} // namespace

bool
RigExecBakeOpGraph(const RigExecBakedProgramImpl &program,
                   fb::RigExecWireFile *file, std::string *error)
{
    if (!program.opProgram)
        return true;
    const RigExecBakedOpProgram &op = *program.opProgram;
    const eval::OpStepLayout &layout = op.layout;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    auto graph = std::make_unique<fb::RigExecWireOpGraph>();
    std::vector<const RigExecOpDesc *> ops;
    std::vector<uint32_t> types;
    for (size_t n = 0; n < layout.nodes.size(); ++n) {
        const RigExecNodeDesc &d = layout.nodes[n];
        if (!op.savable[n])
            return Fail(error, "node " + std::to_string(n) + " (" + d.op->name +
                                   ") cannot save its settings");
        auto node = std::make_unique<fb::RigExecWireOpNode>();
        auto at = std::find(ops.begin(), ops.end(), d.op);
        if (at == ops.end()) {
            ops.push_back(d.op);
            file->manifest->coreOps.push_back(
                Entry(d.op->nameSpace, d.op->name, d.op->version));
            at = ops.end() - 1;
        }
        node->op = uint32_t(at - ops.begin());
        node->payload = op.payloads[n];
        for (uint32_t p = 0; p < d.values.size(); ++p) {
            if (!(d.op->ports[p].flags & kRigExecPortInput))
                continue;
            if (!d.lists[p].empty()) {
                for (const uint32_t v : d.lists[p]) {
                    const uint32_t producer = layout.producerOf[v];
                    if (producer == kRigExecInvalidIndex)
                        return Fail(error,
                                    "a value of input " +
                                        std::string(d.op->ports[p].name) +
                                        " of node " + std::to_string(n) +
                                        " is fed by no node");
                    auto input = std::make_unique<fb::RigExecWireOpNodeInput>();
                    input->port = p;
                    input->node = int32_t(producer);
                    input->output = OutputPortOf(layout, producer, v);
                    node->inputs.push_back(std::move(input));
                }
                continue;
            }
            auto input = std::make_unique<fb::RigExecWireOpNodeInput>();
            input->port = p;
            const uint32_t v = d.values[p];
            const uint32_t producer = v == kRigExecInvalidIndex
                                          ? kRigExecInvalidIndex
                                          : layout.producerOf[v];
            if (producer != kRigExecInvalidIndex) {
                input->node = int32_t(producer);
                input->output = OutputPortOf(layout, producer, v);
                node->inputs.push_back(std::move(input));
                continue;
            }
            bool fed = false;
            for (const eval::OpHostInput &host : layout.hostInputs)
                if (host.node == n && host.port == p) {
                    input->host = int8_t(host.source);
                    input->object = host.object;
                    fed = true;
                }
            for (const eval::OpConstant &constant : layout.constants)
                if (constant.node == n && constant.port == p) {
                    input->host = 3;
                    input->constant = constant.bytes;
                    fed = true;
                }
            if (fed)
                node->inputs.push_back(std::move(input));
            else if (v != kRigExecInvalidIndex)
                return Fail(error, "input " + std::string(d.op->ports[p].name) +
                                       " of node " + std::to_string(n) +
                                       " is fed by nothing the file holds");
        }
        graph->nodes.push_back(std::move(node));
    }
    for (const auto &[name, output] : op.outputs) {
        RigExecValueRef ref;
        if (!RigExecResolveOutput(*layout.program, output, &ref))
            return Fail(error, "output " + name + " does not resolve");
        auto out = std::make_unique<fb::RigExecWireOpGraphOutput>();
        out->name = name;
        out->node = layout.producerOf[ref.value];
        const RigExecNodeDesc &d = layout.nodes[out->node];
        for (uint32_t o = 0; o < d.values.size(); ++o)
            if ((d.op->ports[o].flags & kRigExecPortOutput) &&
                d.values[o] == ref.value)
                out->port = o;
        const uint32_t type = ref.type->Index();
        auto at = std::find(types.begin(), types.end(), type);
        if (at == types.end()) {
            types.push_back(type);
            const RigExecTypeDesc &desc = *registry.TypeAt(type).desc;
            file->manifest->coreTypes.push_back(
                Entry(desc.nameSpace, desc.name, desc.major));
            at = types.end() - 1;
        }
        out->type = uint32_t(at - types.begin());
        graph->outputs.push_back(std::move(out));
    }
    for (const eval::OpLookup &lookup : layout.lookups) {
        auto out = std::make_unique<fb::RigExecWireOpLookup>();
        out->source = lookup.source;
        out->object = lookup.object;
        out->part = lookup.part;
        out->read = lookup.read;
        graph->lookups.push_back(std::move(out));
    }
    for (const eval::OpHostOutput &output : layout.hostOutputs) {
        auto out = std::make_unique<fb::RigExecWireOpHostOutput>();
        out->node = output.node;
        out->port = output.port;
        out->target = output.target;
        out->object = output.object;
        graph->hostOutputs.push_back(std::move(out));
    }
    for (const eval::OpChainLayout &chain : layout.chains) {
        auto out = std::make_unique<fb::RigExecWireOpChain>();
        out->pointChain = chain.pointChain;
        out->bounds = chain.bounds;
        graph->chains.push_back(std::move(out));
    }
    file->opGraph = std::move(graph);
    return true;
}

} // namespace rigExec
