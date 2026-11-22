#include "rigExec/api/compile.h"
#include "rigExec/internal/compile/rigDescData.h"
#include "rigExec/internal/program/program.h"

#include <algorithm>
#include <numeric>
#include <string>
#include <tuple>
#include <vector>

namespace rigExec {

RigExecProgram::~RigExecProgram()
{
    if (!payloads.Data() || !RigExecRegistry::IsInitialized())
        return;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (const RigExecProgramNode &node : nodes) {
        if (node.payloadOffset == SIZE_MAX)
            continue;
        const RigExecOpDesc &d = *registry.OpAt(node.op).desc;
        if (d.payload.destroy)
            d.payload.destroy(payloads.Data() + node.payloadOffset);
    }
}

namespace {

std::string
NodeText(const RigExecRegistry &registry, const RigExecRigDescData &data,
         uint32_t node)
{
    const RigExecOpDesc &d = *registry.OpAt(data.nodes[node].op).desc;
    return "node " + std::to_string(node) + " (" +
           (d.nameSpace ? d.nameSpace : "") + ":" + (d.name ? d.name : "") +
           ")";
}

bool
Fail(std::string *error, std::string message)
{
    if (error)
        *error = std::move(message);
    return false;
}

} // namespace

std::shared_ptr<const RigExecProgram>
RigExecCompile(const RigExecRigDesc &desc, const RigExecCompileOptions &options,
               std::string *error)
{
    if (!RigExecRegistry::IsInitialized()) {
        Fail(error, "the registry is not initialized");
        return nullptr;
    }
    const RigExecRegistry &registry = RigExecRegistry::Get();
    const RigExecRigDescData &data = desc.Data();
    if (data.registryFingerprint != registry.Fingerprint()) {
        Fail(error, "the description was built against another registry");
        return nullptr;
    }
    const uint32_t nodeCount = uint32_t(data.nodes.size());

    auto program =
        std::make_shared<RigExecProgram>(RigExecProgram::_CompileKey(0));
    program->registryFingerprint = registry.Fingerprint();
    program->nodes.resize(nodeCount);

    // Port table and output slots, in node then port order.
    for (uint32_t n = 0; n < nodeCount; ++n) {
        const RigExecRegisteredOp &op = registry.OpAt(data.nodes[n].op);
        RigExecProgramNode &node = program->nodes[n];
        node.op = data.nodes[n].op;
        node.firstPort = uint32_t(program->portSlots.size());
        node.portCount = uint32_t(op.desc->portCount);
        for (uint32_t p = 0; p < node.portCount; ++p) {
            uint32_t slot = kRigExecInvalidIndex;
            if (op.desc->ports[p].flags & kRigExecPortOutput) {
                slot = uint32_t(program->slots.size());
                program->slots.push_back({0, op.portTypes[p]});
            }
            program->portSlots.push_back(slot);
        }
    }
    const auto portIndex = [&](const RigExecDescPort &port) {
        return program->nodes[port.node].firstPort + port.port;
    };
    const auto portType = [&](const RigExecDescPort &port) {
        return registry.OpAt(data.nodes[port.node].op).portTypes[port.port];
    };

    // Feeds: one connection or binding per input.
    std::vector<uint8_t> fed(program->portSlots.size(), 0);
    std::vector<std::vector<uint32_t>> successors(nodeCount);
    for (const RigExecDescConnection &c : data.connections) {
        const uint32_t to = portIndex(c.to);
        if (fed[to]++) {
            Fail(error, NodeText(registry, data, c.to.node) + " input " +
                            std::to_string(c.to.port) + " is fed twice");
            return nullptr;
        }
        if (portType(c.from) != portType(c.to)) {
            Fail(error, NodeText(registry, data, c.from.node) + " to " +
                            NodeText(registry, data, c.to.node) +
                            " joins different types");
            return nullptr;
        }
        program->portSlots[to] = program->portSlots[portIndex(c.from)];
        if (c.from.node == c.to.node) {
            Fail(error, NodeText(registry, data, c.to.node) + " feeds itself");
            return nullptr;
        }
        successors[c.from.node].push_back(c.to.node);
    }
    for (const RigExecDescPort &in : data.inputs) {
        const uint32_t to = portIndex(in);
        if (fed[to]++) {
            Fail(error, NodeText(registry, data, in.node) + " input " +
                            std::to_string(in.port) + " is fed twice");
            return nullptr;
        }
        const uint32_t slot = uint32_t(program->slots.size());
        program->slots.push_back({0, portType(in)});
        program->portSlots[to] = slot;
        program->inputSlots.push_back(slot);
    }
    for (uint32_t n = 0; n < nodeCount; ++n) {
        const RigExecOpDesc &d = *registry.OpAt(data.nodes[n].op).desc;
        for (uint32_t p = 0; p < d.portCount; ++p) {
            const uint32_t flags = d.ports[p].flags;
            if ((flags & kRigExecPortInput) &&
                !(flags & kRigExecPortOptional) &&
                !fed[program->nodes[n].firstPort + p]) {
                Fail(error, NodeText(registry, data, n) + " input " +
                                (d.ports[p].name ? d.ports[p].name : "") +
                                " is not fed");
                return nullptr;
            }
        }
    }
    for (const RigExecDescPort &out : data.outputs) {
        program->outputSlots.push_back(program->portSlots[portIndex(out)]);
    }

    // Order: explicit pairs and the registry's op ordering.
    for (const auto &[first, second] : data.orders) {
        successors[first].push_back(second);
    }
    std::vector<std::vector<uint32_t>> nodesOfOp(registry.OpCount());
    for (uint32_t n = 0; n < nodeCount; ++n) {
        nodesOfOp[data.nodes[n].op].push_back(n);
    }
    for (uint32_t n = 0; n < nodeCount; ++n) {
        for (const uint32_t before : registry.OpAt(data.nodes[n].op).after) {
            for (const uint32_t m : nodesOfOp[before]) {
                successors[m].push_back(n);
            }
        }
    }

    // Levels by longest path; a node runs after everything it depends on.
    std::vector<uint32_t> pending(nodeCount, 0);
    for (uint32_t n = 0; n < nodeCount; ++n) {
        std::sort(successors[n].begin(), successors[n].end());
        successors[n].erase(
            std::unique(successors[n].begin(), successors[n].end()),
            successors[n].end());
        for (const uint32_t s : successors[n]) {
            ++pending[s];
        }
    }
    std::vector<uint32_t> level(nodeCount, 0);
    std::vector<uint32_t> ready;
    for (uint32_t n = 0; n < nodeCount; ++n) {
        if (!pending[n])
            ready.push_back(n);
    }
    uint32_t placed = 0;
    while (!ready.empty()) {
        const uint32_t n = ready.back();
        ready.pop_back();
        ++placed;
        for (const uint32_t s : successors[n]) {
            level[s] = std::max(level[s], level[n] + 1);
            if (!--pending[s])
                ready.push_back(s);
        }
    }
    if (placed != nodeCount) {
        for (uint32_t n = 0; n < nodeCount; ++n) {
            if (pending[n]) {
                Fail(error,
                     "order cycle through " + NodeText(registry, data, n));
                return nullptr;
            }
        }
    }

    // Batches: by level, then op, then node id; split at maxBatchNodes.
    std::vector<uint32_t> order(nodeCount);
    std::iota(order.begin(), order.end(), 0u);
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
        return std::make_tuple(level[a], data.nodes[a].op, a) <
               std::make_tuple(level[b], data.nodes[b].op, b);
    });
    program->batchNodes = order;
    const size_t limit = options.maxBatchNodes;
    uint32_t currentLevel = kRigExecInvalidIndex;
    for (uint32_t i = 0; i < nodeCount; ++i) {
        const uint32_t n = order[i];
        while (currentLevel != level[n]) {
            currentLevel =
                currentLevel == kRigExecInvalidIndex ? 0 : currentLevel + 1;
            program->levelBatches.push_back(uint32_t(program->batches.size()));
        }
        const uint32_t op = data.nodes[n].op;
        RigExecProgramBatch *batch =
            program->batches.empty() ? nullptr : &program->batches.back();
        const bool sameLevel =
            batch && level[program->batchNodes[batch->firstNode]] == level[n];
        if (!batch || !sameLevel || batch->op != op ||
            (limit && batch->nodeCount >= limit)) {
            const RigExecOpDesc &d = *registry.OpAt(op).desc;
            program->batches.push_back(
                {op, i, 0, d.threading != RigExecOpThreading::Pure});
            batch = &program->batches.back();
        }
        ++batch->nodeCount;
    }
    program->levelBatches.push_back(uint32_t(program->batches.size()));

    // Arena layout in slot order.
    size_t offset = 0;
    for (RigExecProgramSlot &slot : program->slots) {
        const RigExecTypeDesc &t = *registry.TypeAt(slot.type).desc;
        offset = RigExecPlaceValue(offset, t.size, t.align);
        slot.offset = offset;
        offset += t.size;
    }
    program->arenaSize = offset;

    // Payload blob and state layout in node order.
    size_t payloadSize = 0;
    size_t stateSize = 0;
    for (uint32_t n = 0; n < nodeCount; ++n) {
        const RigExecOpDesc &d = *registry.OpAt(data.nodes[n].op).desc;
        if (data.nodes[n].hasPayload) {
            payloadSize =
                RigExecPlaceValue(payloadSize, d.payload.size, d.payload.align);
            program->nodes[n].payloadOffset = payloadSize;
            payloadSize += d.payload.size;
        }
        if (d.state.size) {
            stateSize =
                RigExecPlaceValue(stateSize, d.state.size, d.state.align);
            program->nodes[n].stateOffset = stateSize;
            stateSize += d.state.size;
        }
    }
    program->stateSize = stateSize;
    if (payloadSize) {
        program->payloads.Allocate(payloadSize);
        for (uint32_t n = 0; n < nodeCount; ++n) {
            if (program->nodes[n].payloadOffset == SIZE_MAX)
                continue;
            const RigExecOpDesc &d = *registry.OpAt(data.nodes[n].op).desc;
            d.payload.copy(program->payloads.Data() +
                               program->nodes[n].payloadOffset,
                           data.nodes[n].payload.Data());
        }
    }
    return program;
}

} // namespace rigExec
