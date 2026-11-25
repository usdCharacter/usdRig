#include "rigExec/api/compile.h"
#include "rigExec/internal/compile/rigDescData.h"
#include "rigExec/internal/program/program.h"
#include "rigExecGraph/windowHistory.h"

#include <algorithm>
#include <cstring>
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

/// What the compile steps share while they fill one program.
struct CompileState {
    const RigExecRegistry &registry;
    const RigExecRigDescData &data;
    RigExecProgram &program;
    std::string *error;
    uint32_t nodeCount = 0;
    /// Per node port: how many connections and bindings feed it.
    std::vector<uint8_t> fed;
    /// Per array input port: its element slots, by element index.
    std::vector<std::vector<uint32_t>> elements;
    /// Per node: the length of the longest chain of nodes it depends on.
    std::vector<uint32_t> level;

    const RigExecOpDesc &OpDesc(uint32_t node) const
    {
        return *registry.OpAt(data.nodes[node].op).desc;
    }
    uint32_t PortIndex(const RigExecDescPort &port) const
    {
        return program.nodes[port.node].firstPort + port.port;
    }
    uint32_t PortType(const RigExecDescPort &port) const
    {
        return registry.OpAt(data.nodes[port.node].op).portTypes[port.port];
    }
    bool IsList(const RigExecDescPort &port) const
    {
        return (OpDesc(port.node).ports[port.port].flags & kRigExecPortArray) !=
               0;
    }
};

/// Gives every node its port range and every output port its own slot, in
/// node then port order.
void
InitNodes(CompileState &s)
{
    RigExecProgram &program = s.program;
    program.nodes.resize(s.nodeCount);
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        const RigExecRegisteredOp &op = s.registry.OpAt(s.data.nodes[n].op);
        RigExecProgramNode &node = program.nodes[n];
        node.op = s.data.nodes[n].op;
        node.firstPort = uint32_t(program.portSlots.size());
        node.portCount = uint32_t(op.desc->portCount);
        for (uint32_t p = 0; p < node.portCount; ++p) {
            uint32_t slot = kRigExecInvalidIndex;
            if (op.desc->ports[p].flags & kRigExecPortOutput) {
                slot = uint32_t(program.slots.size());
                program.slots.push_back({0, op.portTypes[p]});
            }
            program.portSlots.push_back(slot);
        }
    }
    s.fed.assign(program.portSlots.size(), 0);
    s.elements.assign(program.portSlots.size(), {});
}

/// Points input \p to at \p slot; an array input takes it at the element
/// index the description recorded.
void
Feed(CompileState &s, const RigExecDescPort &to, uint32_t element,
     uint32_t slot)
{
    const uint32_t port = s.PortIndex(to);
    if (!s.IsList(to)) {
        s.program.portSlots[port] = slot;
        return;
    }
    std::vector<uint32_t> &list = s.elements[port];
    if (list.size() <= element)
        list.resize(element + 1, kRigExecInvalidIndex);
    list[element] = slot;
}

/// Counts one more feed of \p to; fails when an input other than an array
/// is fed twice.
bool
FedTwice(CompileState &s, const RigExecDescPort &to)
{
    if (s.fed[s.PortIndex(to)]++ && !s.IsList(to)) {
        Fail(s.error, NodeText(s.registry, s.data, to.node) + " input " +
                          std::to_string(to.port) + " is fed twice");
        return true;
    }
    return false;
}

/// Feeds each connected input from its source output's slot, checking that
/// both ends have one type.
bool
ConnectNodes(CompileState &s)
{
    for (const RigExecDescConnection &c : s.data.connections) {
        if (FedTwice(s, c.to))
            return false;
        if (s.PortType(c.from) != s.PortType(c.to))
            return Fail(s.error, NodeText(s.registry, s.data, c.from.node) +
                                     " to " +
                                     NodeText(s.registry, s.data, c.to.node) +
                                     " joins different types");
        Feed(s, c.to, c.element, s.program.portSlots[s.PortIndex(c.from)]);
        if (c.from.node == c.to.node)
            return Fail(s.error, NodeText(s.registry, s.data, c.to.node) +
                                     " feeds itself");
    }
    return true;
}

/// Gives each host-written input a slot of its own.
bool
BindInputs(CompileState &s)
{
    for (const RigExecDescInput &in : s.data.inputs) {
        if (FedTwice(s, in.to))
            return false;
        const uint32_t slot = uint32_t(s.program.slots.size());
        s.program.slots.push_back({0, s.PortType(in.to)});
        Feed(s, in.to, in.element, slot);
        s.program.inputSlots.push_back(slot);
    }
    return true;
}

/// Fails on an unfed required input, and lays out each array input's
/// element slots in node then port order.
bool
CheckInputsAndLists(CompileState &s)
{
    RigExecProgram &program = s.program;
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        const RigExecOpDesc &d = s.OpDesc(n);
        for (uint32_t p = 0; p < d.portCount; ++p) {
            const uint32_t flags = d.ports[p].flags;
            if ((flags & kRigExecPortInput) &&
                !(flags & kRigExecPortOptional) &&
                !s.fed[program.nodes[n].firstPort + p])
                return Fail(s.error,
                            NodeText(s.registry, s.data, n) + " input " +
                                (d.ports[p].name ? d.ports[p].name : "") +
                                " is not fed");
            if ((flags & kRigExecPortInput) && (flags & kRigExecPortArray)) {
                const uint32_t port = program.nodes[n].firstPort + p;
                const std::vector<uint32_t> &list = s.elements[port];
                program.lists.push_back({port,
                                         uint32_t(program.listSlots.size()),
                                         uint32_t(list.size())});
                program.listSlots.insert(program.listSlots.end(), list.begin(),
                                         list.end());
            }
        }
    }
    return true;
}

void
BindOutputs(CompileState &s)
{
    for (const RigExecDescPort &out : s.data.outputs) {
        s.program.outputSlots.push_back(s.program.portSlots[s.PortIndex(out)]);
    }
}

/// The node's key in the dependency compiler: its id, zero-padded so the
/// compiler's key order is node order.
std::string
NodeKey(uint32_t node)
{
    std::string digits = std::to_string(node);
    return std::string(10 - digits.size(), '0') + digits;
}

/// What node \p n reads and writes: the slots of its fed inputs, list
/// elements included, and of its outputs.
RigExecOpDescriptor
NodeDescriptor(const CompileState &s, uint32_t n)
{
    const RigExecProgramNode &node = s.program.nodes[n];
    const RigExecOpDesc &d = s.OpDesc(n);
    RigExecOpDescriptor descriptor;
    descriptor.key = NodeKey(n);
    descriptor.kind = node.op;
    descriptor.volatileInput = d.threading != RigExecOpThreading::Pure;
    for (uint32_t p = 0; p < node.portCount; ++p) {
        const uint32_t port = node.firstPort + p;
        if (d.ports[p].flags & kRigExecPortOutput) {
            descriptor.writes.push_back(s.program.portSlots[port]);
            continue;
        }
        if (s.program.portSlots[port] != kRigExecInvalidIndex)
            descriptor.reads.push_back(s.program.portSlots[port]);
        for (const uint32_t slot : s.elements[port])
            if (slot != kRigExecInvalidIndex)
                descriptor.reads.push_back(slot);
    }
    return descriptor;
}

/// Names the nodes of the first cycle \p graph found, in edge order.
std::string
CycleText(const CompileState &s, const RigExecCompiledGraph &graph)
{
    std::string text = "order cycle:";
    const std::vector<std::string> &loop = graph.cycles.front();
    for (size_t i = 0; i < loop.size(); ++i) {
        uint32_t node = 0;
        for (const char digit : loop[i])
            node = node * 10 + uint32_t(digit - '0');
        text += (i ? " -> " : " ") + NodeText(s.registry, s.data, node);
    }
    return text;
}

/// Orders the nodes with the dependency compiler every evaluator shares:
/// data edges come from the slots each node reads and writes, the host's
/// order pairs are predecessor edges, and host-written slots are leaves.
/// Each node's level is one after its deepest predecessor's. Fails on a
/// cycle.
bool
OrderNodes(CompileState &s)
{
    RigExecProgram &program = s.program;
    std::vector<RigExecOpDescriptor> descriptors;
    descriptors.reserve(s.nodeCount);
    for (uint32_t n = 0; n < s.nodeCount; ++n)
        descriptors.push_back(NodeDescriptor(s, n));
    for (const auto &[first, second] : s.data.orders)
        descriptors[second].predecessors.push_back(first);
    const std::vector<RigExecValueId> leaves(program.inputSlots.begin(),
                                             program.inputSlots.end());
    std::string why;
    if (!RigExecCompileOpGraph(descriptors, leaves,
                               RigExecCyclePolicy::SetAside, &program.graph,
                               &why))
        return Fail(s.error, why);
    if (!program.graph.cycles.empty())
        return Fail(s.error, CycleText(s, program.graph));
    s.level.assign(s.nodeCount, 0);
    if (program.hostOrder) {
        for (const RigExecCompiledOp &op : program.graph.ops) {
            const uint32_t n = op.originalIndex;
            if (s.OpDesc(n).threading == RigExecOpThreading::Windowed)
                return Fail(s.error, NodeText(s.registry, s.data, n) +
                                         " keeps history, which a host-ordered "
                                         "program cannot run");
            for (const uint32_t predecessor : op.predecessors)
                if (program.graph.ops[predecessor].originalIndex > n)
                    return Fail(
                        s.error,
                        NodeText(s.registry, s.data, n) + " depends on " +
                            NodeText(
                                s.registry, s.data,
                                program.graph.ops[predecessor].originalIndex) +
                            ", which was added after it");
            s.level[n] = n;
        }
        return true;
    }
    for (const RigExecCompiledOp &op : program.graph.ops)
        for (const uint32_t predecessor : op.predecessors)
            s.level[op.originalIndex] = std::max(
                s.level[op.originalIndex],
                s.level[program.graph.ops[predecessor].originalIndex] + 1);
    return true;
}

/// Records each Windowed node's frame count, and settles over the compiled
/// graph which windowed nodes each depends on and which nodes a history
/// tick runs, as every evaluator settles its windowed ops.
void
SettleWindows(CompileState &s)
{
    RigExecProgram &program = s.program;
    const RigExecCompiledGraph &graph = program.graph;
    program.windowOfNode.assign(s.nodeCount, kRigExecInvalidIndex);
    std::vector<uint32_t> windowed;
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        const RigExecOpDesc &d = s.OpDesc(n);
        if (d.threading != RigExecOpThreading::Windowed)
            continue;
        const RigExecDescNode &node = s.data.nodes[n];
        RigExecProgramWindow window;
        window.node = n;
        window.frames = d.windowFrames(
            d.payloadFieldCount ? static_cast<const void *>(&node.fields)
            : node.hasPayload   ? node.payload.Data()
                                : nullptr);
        program.windowOfNode[n] = uint32_t(windowed.size());
        windowed.push_back(uint32_t(graph.canonicalIndex[n]));
        program.windowedNodes.push_back(std::move(window));
    }
    if (windowed.empty())
        return;
    RigExecWindowLayout layout;
    RigExecSettleWindowLayout(graph, windowed, &layout);
    program.historyMask.resize(s.nodeCount);
    for (uint32_t n = 0; n < s.nodeCount; ++n)
        program.historyMask[n] = layout.mask[graph.canonicalIndex[n]];
    for (size_t w = 0; w < windowed.size(); ++w) {
        program.windowedNodes[w].inner = std::move(layout.inner[w]);
        program.windowedNodes[w].feedsWindow = layout.feedsWindow[w] != 0;
    }
}

/// Groups nodes into batches by level, then op, then the compiled graph's
/// canonical order, splitting a batch at \p limit nodes (0 for no limit).
void
BuildBatches(CompileState &s, size_t limit)
{
    RigExecProgram &program = s.program;
    const std::vector<uint32_t> &level = s.level;
    std::vector<uint32_t> order(s.nodeCount);
    std::iota(order.begin(), order.end(), 0u);
    const std::vector<int32_t> &canonical = program.graph.canonicalIndex;
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
        return std::make_tuple(level[a], s.data.nodes[a].op, canonical[a]) <
               std::make_tuple(level[b], s.data.nodes[b].op, canonical[b]);
    });
    program.batchNodes = order;
    uint32_t currentLevel = kRigExecInvalidIndex;
    for (uint32_t i = 0; i < s.nodeCount; ++i) {
        const uint32_t n = order[i];
        while (currentLevel != level[n]) {
            currentLevel =
                currentLevel == kRigExecInvalidIndex ? 0 : currentLevel + 1;
            program.levelBatches.push_back(uint32_t(program.batches.size()));
        }
        const uint32_t op = s.data.nodes[n].op;
        RigExecProgramBatch *batch =
            program.batches.empty() ? nullptr : &program.batches.back();
        const bool sameLevel =
            batch && level[program.batchNodes[batch->firstNode]] == level[n];
        if (!batch || !sameLevel || batch->op != op ||
            (limit && batch->nodeCount >= limit)) {
            const RigExecOpDesc &d = *s.registry.OpAt(op).desc;
            program.batches.push_back(
                {op, i, 0, d.threading != RigExecOpThreading::Pure});
            batch = &program.batches.back();
        }
        ++batch->nodeCount;
    }
    program.levelBatches.push_back(uint32_t(program.batches.size()));
    if (program.historyMask.empty())
        return;
    program.batchHistoryMask.resize(s.nodeCount);
    for (uint32_t i = 0; i < s.nodeCount; ++i)
        program.batchHistoryMask[i] = program.historyMask[order[i]];
}

/// Renumbers the node ports in batch order, so each batch's ports are one
/// run: node by node, each node's ports in port order.
void
PlacePortsInBatchOrder(CompileState &s)
{
    RigExecProgram &program = s.program;
    std::vector<uint32_t> portSlots;
    portSlots.reserve(program.portSlots.size());
    std::vector<uint32_t> moved(program.portSlots.size());
    for (const uint32_t n : program.batchNodes) {
        RigExecProgramNode &node = program.nodes[n];
        const uint32_t first = uint32_t(portSlots.size());
        for (uint32_t p = 0; p < node.portCount; ++p) {
            moved[node.firstPort + p] = first + p;
            portSlots.push_back(program.portSlots[node.firstPort + p]);
        }
        node.firstPort = first;
    }
    program.portSlots = std::move(portSlots);
    for (RigExecProgramList &list : program.lists)
        list.port = moved[list.port];
    for (RigExecProgramBatch &batch : program.batches)
        batch.firstPort =
            program.nodes[program.batchNodes[batch.firstNode]].firstPort;
}

/// Gives every slot its arena: each distinct bound output a published
/// arena of its own, the host-written inputs the input arena, in binding
/// order, and every other value the work arena, in slot order.
void
LayoutStore(CompileState &s)
{
    RigExecProgram &program = s.program;
    for (const uint32_t slot : program.outputSlots) {
        RigExecProgramSlot &place = program.slots[slot];
        if (place.arena == RigExecArenaKind::Published)
            continue;
        place.arena = RigExecArenaKind::Published;
        place.arenaIndex = uint32_t(program.publishedSlots.size());
        place.offset = 0;
        program.publishedSlots.push_back(slot);
    }
    for (uint32_t i = 0; i < program.inputSlots.size(); ++i) {
        RigExecProgramSlot &place = program.slots[program.inputSlots[i]];
        place.arena = RigExecArenaKind::Input;
        place.arenaIndex = i;
    }
    size_t input = 0;
    size_t work = 0;
    for (RigExecProgramSlot &slot : program.slots) {
        if (slot.arena == RigExecArenaKind::Published)
            continue;
        const RigExecTypeDesc &t = *s.registry.TypeAt(slot.type).desc;
        size_t &end = slot.arena == RigExecArenaKind::Input ? input : work;
        end = RigExecPlaceValue(end, t.size, t.align);
        slot.offset = end;
        end += t.size;
    }
    program.inputArenaSize = input;
    program.workSize = work;
}

/// Indexes what dirty propagation walks: each slot's readers, from the
/// compiled graph, and its writer; each node's outputs and batchNodes
/// entry; the nodes that run every frame; each batchNodes entry's batch.
void
IndexReaders(CompileState &s)
{
    RigExecProgram &program = s.program;
    const RigExecCompiledGraph &graph = program.graph;
    const size_t slots = program.slots.size();
    program.readerStart.assign(1, 0);
    std::vector<uint32_t> readers;
    for (uint32_t slot = 0; slot < slots; ++slot) {
        readers.clear();
        const auto found = graph.readers.find(slot);
        if (found != graph.readers.end())
            for (const uint32_t op : found->second)
                readers.push_back(graph.ops[op].originalIndex);
        std::sort(readers.begin(), readers.end());
        readers.erase(std::unique(readers.begin(), readers.end()),
                      readers.end());
        program.readers.insert(program.readers.end(), readers.begin(),
                               readers.end());
        program.readerStart.push_back(uint32_t(program.readers.size()));
    }
    program.producer.assign(slots, kRigExecInvalidIndex);
    program.nodeOutputStart.assign(1, 0);
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        const RigExecProgramNode &node = program.nodes[n];
        const RigExecOpDesc &d = s.OpDesc(n);
        for (uint32_t p = 0; p < node.portCount; ++p) {
            if (!(d.ports[p].flags & kRigExecPortOutput))
                continue;
            const uint32_t slot = program.portSlots[node.firstPort + p];
            program.producer[slot] = n;
            program.nodeOutputs.push_back(slot);
        }
        program.nodeOutputStart.push_back(uint32_t(program.nodeOutputs.size()));
    }
    program.batchPosition.assign(s.nodeCount, 0);
    for (uint32_t i = 0; i < s.nodeCount; ++i)
        program.batchPosition[program.batchNodes[i]] = i;
    for (const uint32_t op : graph.volatileOps)
        program.volatileNodes.push_back(graph.ops[op].originalIndex);
    std::sort(program.volatileNodes.begin(), program.volatileNodes.end());
    program.positionBatch.assign(s.nodeCount, 0);
    for (uint32_t b = 0; b < program.batches.size(); ++b) {
        const RigExecProgramBatch &batch = program.batches[b];
        for (uint32_t k = 0; k < batch.nodeCount; ++k)
            program.positionBatch[batch.firstNode + k] = b;
    }
}

/// Records, per slot, the port and element entries a run points at it.
void
PlaceCells(CompileState &s)
{
    RigExecProgram &program = s.program;
    std::vector<std::vector<RigExecProgramCell>> cells(program.slots.size());
    for (uint32_t port = 0; port < program.portSlots.size(); ++port) {
        const uint32_t slot = program.portSlots[port];
        if (slot != kRigExecInvalidIndex)
            cells[slot].push_back({RigExecProgramCell::Input, port});
    }
    for (const RigExecProgramNode &node : program.nodes) {
        const RigExecOpDesc &d = *s.registry.OpAt(node.op).desc;
        for (uint32_t p = 0; p < node.portCount; ++p) {
            const uint32_t port = node.firstPort + p;
            const uint32_t slot = program.portSlots[port];
            if (slot != kRigExecInvalidIndex &&
                (d.ports[p].flags & kRigExecPortOutput))
                for (RigExecProgramCell &cell : cells[slot])
                    if (cell.index == port)
                        cell.kind = RigExecProgramCell::Output;
        }
    }
    for (uint32_t i = 0; i < program.listSlots.size(); ++i)
        cells[program.listSlots[i]].push_back(
            {RigExecProgramCell::ListItem, i});
    program.cellStart.assign(1, 0);
    for (const std::vector<RigExecProgramCell> &list : cells) {
        program.cells.insert(program.cells.end(), list.begin(), list.end());
        program.cellStart.push_back(uint32_t(program.cells.size()));
    }
}

/// The integer one-value field \p field of \p values holds; a negative
/// signed value counts as zero.
uint64_t IntegerField(const RigExecRegistry &registry, const RigExecOpDesc &d,
                      const RigExecFieldValues &values, uint32_t field);

/// Settles each bounded output's bound: Fixed, Field and Host bounds to a
/// count, Input and InputValue bounds to the slots they follow. Fails on
/// a Host bound the description does not give.
bool
SettleBounds(CompileState &s)
{
    RigExecProgram &program = s.program;
    program.bounds.assign(program.slots.size(), RigExecProgramBound());
    // The slots fed to input \p handle of node \p n: one, or an array
    // input's elements.
    const auto addSources = [&](uint32_t n, const RigExecPortHandle *handle,
                                bool integer) {
        if (!handle)
            return;
        const uint32_t port = program.nodes[n].firstPort + handle->Index();
        if (program.portSlots[port] != kRigExecInvalidIndex) {
            program.boundSources.push_back({program.portSlots[port], integer});
            return;
        }
        for (const RigExecProgramList &list : program.lists)
            if (list.port == port)
                for (uint32_t i = 0; i < list.count; ++i)
                    program.boundSources.push_back(
                        {program.listSlots[list.firstSlot + i], false});
    };
    for (const RigExecCompiledOp &op : program.graph.ops) {
        const uint32_t n = op.originalIndex;
        const RigExecProgramNode &node = program.nodes[n];
        const RigExecRegisteredOp &registered = s.registry.OpAt(node.op);
        const RigExecOpDesc &d = *registered.desc;
        for (const uint32_t p : registered.boundedOutputs) {
            const RigExecPortDesc &port = d.ports[p];
            const uint32_t slot = program.portSlots[node.firstPort + p];
            RigExecProgramBound &bound = program.bounds[slot];
            bound.kind = RigExecOutputBound::Fixed;
            bound.count = port.boundCount;
            switch (port.bound) {
            case RigExecOutputBound::Input:
            case RigExecOutputBound::InputValue:
                bound.firstSource = uint32_t(program.boundSources.size());
                addSources(n, port.boundPort,
                           port.bound == RigExecOutputBound::InputValue);
                addSources(n, port.boundAltPort, false);
                bound.sourceCount =
                    uint32_t(program.boundSources.size()) - bound.firstSource;
                if (!bound.sourceCount) {
                    bound.count = 0;
                    break;
                }
                bound.kind = port.bound;
                program.derivedBounds.push_back(slot);
                break;
            case RigExecOutputBound::Field:
                bound.count *= size_t(IntegerField(
                    s.registry, d, s.data.nodes[n].fields, port.boundField));
                break;
            case RigExecOutputBound::Host: {
                bool given = false;
                for (const auto &[at, count] : s.data.outputBounds) {
                    if (at.node == n && at.port == p) {
                        bound.count = count;
                        given = true;
                    }
                }
                if (!given)
                    return Fail(s.error, NodeText(s.registry, s.data, n) +
                                             " output " +
                                             (port.name ? port.name : "") +
                                             " needs a bound from the "
                                             "description");
                break;
            }
            case RigExecOutputBound::Fixed:
            case RigExecOutputBound::None:
                break;
            }
        }
    }
    return true;
}

/// Asks each ranged node's settings whether it runs whole, and checks every
/// node's settings with its op's validate, before any input is known.
bool
CheckNodes(CompileState &s)
{
    RigExecProgram &program = s.program;
    program.wholePositions.assign(s.nodeCount, 0);
    std::vector<RigExecFieldRunOf<const std::byte>> runs(
        program.payloadRuns.size());
    for (size_t i = 0; i < runs.size(); ++i)
        runs[i] = {program.payloads.Data() + program.payloadRuns[i].offset,
                   program.payloadRuns[i].count};
    std::vector<RigExecPayloadColumn> columns(program.payloadColumns.size());
    for (size_t i = 0; i < columns.size(); ++i) {
        const RigExecProgramColumn &c = program.payloadColumns[i];
        columns[i] = {program.payloads.Data() + c.offset, c.stride, c.count,
                      c.firstRun == kRigExecInvalidIndex ? nullptr
                                                         : &runs[c.firstRun]};
    }
    std::vector<const void *> inputs;
    std::vector<size_t> elements;
    for (const RigExecProgramBatch &batch : program.batches) {
        const RigExecOpDesc &d = *s.registry.OpAt(batch.op).desc;
        if (!d.range.plan && !d.validate)
            continue;
        RigExecOpBatch view;
        view.op = &d;
        view.count = batch.nodeCount;
        view.portCount = d.portCount;
        view.useSimd = program.useSimd;
        if (batch.payloadOffset != SIZE_MAX) {
            view.payloads = program.payloads.Data() + batch.payloadOffset;
            view.payloadStride = batch.payloadStride;
        }
        if (d.payloadFieldCount)
            view.payloadFields = columns.data() + batch.firstPayloadColumn;
        inputs.assign(size_t(batch.nodeCount) * d.portCount, nullptr);
        view.inputs = inputs.data();
        for (uint32_t k = 0; k < batch.nodeCount; ++k) {
            const uint32_t at = batch.firstNode + k;
            elements.assign(d.portCount, SIZE_MAX);
            for (uint32_t port = 0; port < d.portCount; ++port)
                if ((d.ports[port].flags & kRigExecPortInput) &&
                    !(d.ports[port].flags & kRigExecPortArray) &&
                    program.portSlots[batch.firstPort + k * d.portCount +
                                      port] == kRigExecInvalidIndex)
                    elements[port] = kRigExecPortUnfed;
            if (d.range.plan)
                program.wholePositions[at] =
                    d.range.plan(view, k) == RigExecRangePlan::Whole;
            if (!d.validate)
                continue;
            RigExecOpCheck check;
            check.batch = view;
            check.k = k;
            check.elements = elements.data();
            check.compiling = true;
            std::string why;
            if (!d.validate(check, &why))
                return Fail(s.error, NodeText(s.registry, s.data,
                                              program.batchNodes[at]) +
                                         " is invalid: " + why);
        }
    }
    return true;
}

/// Puts every ranged node in a range chain, in canonical order. A node
/// continues the chain of the node feeding the input its element count
/// comes from when that node ends its chain, every input it reads from
/// the chain is pointwise or neighbourhood, and everything else it depends
/// on is at a level before the chain's first node; otherwise it starts a
/// chain of its own.
void
FormRangeChains(CompileState &s)
{
    RigExecProgram &program = s.program;
    const RigExecCompiledGraph &graph = program.graph;
    program.positionChain.assign(s.nodeCount, kRigExecInvalidIndex);
    std::vector<uint32_t> chainOf(s.nodeCount, kRigExecInvalidIndex);
    std::vector<uint32_t> memberOf(s.nodeCount, 0);
    std::vector<std::vector<uint32_t>> members;
    std::vector<std::vector<std::vector<RigExecProgramChainFeed>>> feeds;
    std::vector<uint32_t> listOfPort(program.portSlots.size(),
                                     kRigExecInvalidIndex);
    for (uint32_t i = 0; i < program.lists.size(); ++i)
        listOfPort[program.lists[i].port] = i;
    std::vector<RigExecProgramChainFeed> joined;
    for (const RigExecCompiledOp &op : graph.ops) {
        const uint32_t n = op.originalIndex;
        const RigExecProgramNode &node = program.nodes[n];
        const RigExecRegisteredOp &registered = s.registry.OpAt(node.op);
        const RigExecOpDesc &d = *registered.desc;
        if (!d.range.run)
            continue;
        const uint32_t countSlot =
            program.portSlots[node.firstPort + registered.rangeCountPort];
        const uint32_t from = countSlot == kRigExecInvalidIndex
                                  ? kRigExecInvalidIndex
                                  : program.producer[countSlot];
        uint32_t chain = kRigExecInvalidIndex;
        if (from != kRigExecInvalidIndex &&
            chainOf[from] != kRigExecInvalidIndex &&
            members[chainOf[from]].back() == from) {
            chain = chainOf[from];
            const uint32_t headLevel = s.level[members[chain].front()];
            for (const uint32_t predecessor : op.predecessors) {
                const uint32_t p = graph.ops[predecessor].originalIndex;
                if (chainOf[p] != chain && s.level[p] >= headLevel)
                    chain = kRigExecInvalidIndex;
            }
            joined.clear();
            for (uint32_t p = 0;
                 chain != kRigExecInvalidIndex && p < node.portCount; ++p) {
                const uint32_t flags = d.ports[p].flags;
                if (!(flags & kRigExecPortInput))
                    continue;
                const uint32_t port = node.firstPort + p;
                const uint32_t list = listOfPort[port];
                const uint32_t first = list == kRigExecInvalidIndex
                                           ? 0
                                           : program.lists[list].firstSlot;
                const uint32_t count = list == kRigExecInvalidIndex
                                           ? 1
                                           : program.lists[list].count;
                for (uint32_t e = 0; e < count; ++e) {
                    const uint32_t slot = list == kRigExecInvalidIndex
                                              ? program.portSlots[port]
                                              : program.listSlots[first + e];
                    if (slot == kRigExecInvalidIndex)
                        continue;
                    const uint32_t producer = program.producer[slot];
                    if (producer == kRigExecInvalidIndex ||
                        chainOf[producer] != chain)
                        continue;
                    const bool neighbourhood =
                        (flags & kRigExecPortNeighbourhood) != 0;
                    if (list != kRigExecInvalidIndex ||
                        !(flags & (kRigExecPortPointwise |
                                   kRigExecPortNeighbourhood))) {
                        chain = kRigExecInvalidIndex;
                        break;
                    }
                    joined.push_back({memberOf[producer], neighbourhood});
                }
            }
            for (const uint32_t predecessor : op.predecessors) {
                const uint32_t p = graph.ops[predecessor].originalIndex;
                if (chain == kRigExecInvalidIndex || chainOf[p] != chain)
                    continue;
                bool fed = false;
                for (const RigExecProgramChainFeed &feed : joined)
                    fed = fed || feed.member == memberOf[p];
                if (!fed)
                    chain = kRigExecInvalidIndex;
            }
        }
        if (chain == kRigExecInvalidIndex) {
            chain = uint32_t(members.size());
            members.emplace_back();
            feeds.emplace_back();
            joined.clear();
        }
        chainOf[n] = chain;
        memberOf[n] = uint32_t(members[chain].size());
        members[chain].push_back(n);
        feeds[chain].push_back(joined);
    }
    program.chainFeedStart.assign(1, 0);
    for (uint32_t c = 0; c < members.size(); ++c) {
        RigExecProgramChain chain;
        chain.firstMember = uint32_t(program.chainMembers.size());
        chain.memberCount = uint32_t(members[c].size());
        chain.level = s.level[members[c].front()];
        program.chains.push_back(chain);
        for (uint32_t m = 0; m < members[c].size(); ++m) {
            const uint32_t at = program.batchPosition[members[c][m]];
            program.chainMembers.push_back(at);
            program.positionChain[at] = c;
            program.chainFeeds.insert(program.chainFeeds.end(),
                                      feeds[c][m].begin(), feeds[c][m].end());
            program.chainFeedStart.push_back(
                uint32_t(program.chainFeeds.size()));
        }
    }
    program.chainSegment.assign(program.chainMembers.size(), 0);
    for (const RigExecProgramChain &chain : program.chains) {
        for (uint32_t m = 0; m < chain.memberCount; ++m) {
            const uint32_t g = chain.firstMember + m;
            bool starts = m == 0 ||
                          program.wholePositions[program.chainMembers[g]] ||
                          program.wholePositions[program.chainMembers[g - 1]];
            for (uint32_t f = program.chainFeedStart[g];
                 f < program.chainFeedStart[g + 1]; ++f)
                starts = starts || program.chainFeeds[f].neighbourhood;
            program.chainSegment[g] = starts ? m : program.chainSegment[g - 1];
        }
    }
}

void
GroupChainOutputs(CompileState &s)
{
    RigExecProgram &program = s.program;
    program.groupedSlots.assign(program.slots.size(), 0);
    std::vector<char> bound(program.slots.size(), 0);
    for (const uint32_t slot : program.outputSlots)
        bound[slot] = 1;
    for (uint32_t c = 0; c < program.chains.size(); ++c) {
        const RigExecProgramChain &chain = program.chains[c];
        for (uint32_t m = 0; m < chain.memberCount; ++m) {
            const uint32_t at = program.chainMembers[chain.firstMember + m];
            if (program.wholePositions[at])
                continue;
            const RigExecProgramNode &node =
                program.nodes[program.batchNodes[at]];
            const RigExecRegisteredOp &op = s.registry.OpAt(node.op);
            for (const uint32_t port : op.rangeOutputs) {
                const uint32_t slot = program.portSlots[node.firstPort + port];
                const RigExecTypeDesc &type =
                    *s.registry.TypeAt(op.portTypes[port]).desc;
                if (slot == kRigExecInvalidIndex || bound[slot] ||
                    !type.borrow || !type.data || !type.element)
                    continue;
                bool inside = true;
                for (uint32_t r = program.readerStart[slot];
                     inside && r < program.readerStart[slot + 1]; ++r) {
                    const uint32_t reader =
                        program.batchPosition[program.readers[r]];
                    inside = program.positionChain[reader] == c &&
                             !program.wholePositions[reader];
                }
                program.groupedSlots[slot] = inside ? 1 : 0;
            }
        }
    }
}

/// Places \p count values of \p size and \p align from \p end, one every
/// \p stride bytes; returns the first one's offset and moves \p end past
/// the last.
size_t
PlaceRun(size_t *end, size_t size, size_t align, size_t count, size_t *stride)
{
    *stride = RigExecPlaceValue(size, size, align);
    const size_t first = RigExecPlaceValue(*end, size, align);
    *end = first + count * *stride;
    return first;
}

/// Places a column of \p bytes from \p end on 64 bytes, or on 256 from
/// 4 KiB; returns its offset and moves \p end past it.
size_t
PlaceColumn(size_t *end, size_t bytes)
{
    const size_t align = bytes >= 4096 ? 256 : 64;
    const size_t first = (*end + align - 1) / align * align;
    *end = first + bytes;
    return first;
}

/// Columns hold whole groups of this many instances.
constexpr size_t kColumnLanes = 16;

/// The unsigned integer field \p field of \p values holds.
uint64_t
UnsignedField(const RigExecFieldValues &values, size_t field)
{
    uint64_t value = 0;
    const std::byte *data = values.Data(field);
    for (size_t i = 0; i < values.ByteSize(field); ++i)
        value |= uint64_t(data[i]) << (8 * i);
    return value;
}

uint64_t
IntegerField(const RigExecRegistry &registry, const RigExecOpDesc &d,
             const RigExecFieldValues &values, uint32_t field)
{
    const RigExecTypeDesc &t =
        *registry.TypeAt(d.payloadFields[field].type->Index()).desc;
    if (t.columnKind == RigExecColumnKind::Int32) {
        int32_t value = 0;
        if (values.ByteSize(field) == sizeof(value))
            std::memcpy(&value, values.Data(field), sizeof(value));
        return value < 0 ? 0 : uint64_t(value);
    }
    return UnsignedField(values, field);
}

/// Lays out one batch's declared payload or state fields from \p end.
void
PlaceFields(const CompileState &s, const RigExecProgramBatch &batch,
            bool payload, size_t *end,
            std::vector<RigExecProgramColumn> *columns,
            std::vector<RigExecProgramRun> *runs)
{
    const RigExecRegisteredOp &op = s.registry.OpAt(batch.op);
    const RigExecOpDesc &d = *op.desc;
    const RigExecFieldDesc *fields = payload ? d.payloadFields : d.stateFields;
    const size_t fieldCount = payload ? d.payloadFieldCount : d.stateFieldCount;
    const uint32_t *nodes = s.program.batchNodes.data() + batch.firstNode;
    for (size_t f = 0; f < fieldCount; ++f) {
        const RigExecFieldDesc &field = fields[f];
        RigExecProgramColumn column;
        column.type = payload ? op.payloadFieldTypes[f] : op.stateFieldTypes[f];
        const RigExecTypeDesc &t = *s.registry.TypeAt(column.type).desc;
        if (field.extent == RigExecFieldExtent::Scalar ||
            field.extent == RigExecFieldExtent::Fixed) {
            column.count =
                field.extent == RigExecFieldExtent::Scalar ? 1 : field.count;
            column.stride = column.count * t.size;
            const size_t lanes = (batch.nodeCount + kColumnLanes - 1) /
                                 kColumnLanes * kColumnLanes;
            column.offset = PlaceColumn(end, lanes * column.stride);
            columns->push_back(column);
            continue;
        }
        column.firstRun = uint32_t(runs->size());
        if (!payload && field.extent == RigExecFieldExtent::FromInput) {
            column.fromInput = true;
            runs->resize(runs->size() + batch.nodeCount);
            columns->push_back(column);
            continue;
        }
        size_t pool = 0;
        for (uint32_t k = 0; k < batch.nodeCount; ++k) {
            const RigExecFieldValues &values = s.data.nodes[nodes[k]].fields;
            RigExecProgramRun run;
            run.count = payload ? values.Count(f)
                                : size_t(UnsignedField(values, field.source));
            pool = (pool + t.align - 1) / t.align * t.align;
            run.offset = pool;
            pool += run.count * t.size;
            runs->push_back(run);
        }
        column.offset = PlaceColumn(end, pool);
        for (uint32_t k = 0; k < batch.nodeCount; ++k)
            (*runs)[column.firstRun + k].offset += column.offset;
        columns->push_back(column);
    }
}

/// Copies each node's declared payload values into its batch's columns.
void
FillPayloadFields(CompileState &s)
{
    RigExecProgram &program = s.program;
    for (const RigExecProgramBatch &batch : program.batches) {
        if (batch.firstPayloadColumn == kRigExecInvalidIndex)
            continue;
        const RigExecOpDesc &d = *s.registry.OpAt(batch.op).desc;
        const uint32_t *nodes = program.batchNodes.data() + batch.firstNode;
        for (size_t f = 0; f < d.payloadFieldCount; ++f) {
            const RigExecProgramColumn &column =
                program.payloadColumns[batch.firstPayloadColumn + f];
            for (uint32_t k = 0; k < batch.nodeCount; ++k) {
                const RigExecFieldValues &values =
                    s.data.nodes[nodes[k]].fields;
                const size_t offset =
                    column.firstRun == kRigExecInvalidIndex
                        ? column.offset + k * column.stride
                        : program.payloadRuns[column.firstRun + k].offset;
                if (values.ByteSize(f))
                    std::memcpy(program.payloads.Data() + offset,
                                values.Data(f), values.ByteSize(f));
            }
        }
    }
}

/// Places each batch's payloads in the payload blob and its states in the
/// state arena, one evenly spaced run per batch, then its declared fields,
/// and copies the payloads in.
void
LayoutPayloadsAndState(CompileState &s)
{
    RigExecProgram &program = s.program;
    size_t payloadSize = 0;
    size_t stateSize = 0;
    for (RigExecProgramBatch &batch : program.batches) {
        const RigExecOpDesc &d = *s.registry.OpAt(batch.op).desc;
        const uint32_t *nodes = program.batchNodes.data() + batch.firstNode;
        if (d.payload.size) {
            batch.payloadOffset =
                PlaceRun(&payloadSize, d.payload.size, d.payload.align,
                         batch.nodeCount, &batch.payloadStride);
            for (uint32_t k = 0; k < batch.nodeCount; ++k)
                program.nodes[nodes[k]].payloadOffset =
                    batch.payloadOffset + k * batch.payloadStride;
        }
        if (d.state.size) {
            batch.stateOffset =
                PlaceRun(&stateSize, d.state.size, d.state.align,
                         batch.nodeCount, &batch.stateStride);
            for (uint32_t k = 0; k < batch.nodeCount; ++k)
                program.nodes[nodes[k]].stateOffset =
                    batch.stateOffset + k * batch.stateStride;
        }
        if (d.payloadFieldCount) {
            batch.firstPayloadColumn = uint32_t(program.payloadColumns.size());
            PlaceFields(s, batch, true, &payloadSize, &program.payloadColumns,
                        &program.payloadRuns);
        }
        if (d.stateFieldCount) {
            batch.firstStateColumn = uint32_t(program.stateColumns.size());
            PlaceFields(s, batch, false, &stateSize, &program.stateColumns,
                        &program.stateRuns);
        }
    }
    program.stateSize = stateSize;
    program.stateBase = (program.workSize + RigExecAlignedBuffer::kAlign - 1) /
                        RigExecAlignedBuffer::kAlign *
                        RigExecAlignedBuffer::kAlign;
    program.workSize = program.stateBase + stateSize;
    if (!payloadSize)
        return;
    program.payloads.Allocate(payloadSize);
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        if (program.nodes[n].payloadOffset == SIZE_MAX)
            continue;
        s.OpDesc(n).payload.copy(program.payloads.Data() +
                                     program.nodes[n].payloadOffset,
                                 s.data.nodes[n].payload.Data());
    }
    FillPayloadFields(s);
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
    auto program =
        std::make_shared<RigExecProgram>(RigExecProgram::_CompileKey(0));
    program->registryFingerprint = registry.Fingerprint();
    program->useSimd = options.useSimd;
    program->rangeGrain = options.rangeGrain;
    program->rangeThreshold = options.rangeThreshold;
    program->hostOrder = options.hostOrder;

    CompileState s{registry, data, *program, error};
    s.nodeCount = uint32_t(data.nodes.size());
    InitNodes(s);
    if (!ConnectNodes(s) || !BindInputs(s) || !CheckInputsAndLists(s))
        return nullptr;
    BindOutputs(s);
    if (!OrderNodes(s))
        return nullptr;
    SettleWindows(s);
    BuildBatches(s, options.maxBatchNodes);
    PlacePortsInBatchOrder(s);
    LayoutStore(s);
    LayoutPayloadsAndState(s);
    PlaceCells(s);
    IndexReaders(s);
    if (!CheckNodes(s))
        return nullptr;
    FormRangeChains(s);
    GroupChainOutputs(s);
    if (!SettleBounds(s))
        return nullptr;
    return program;
}

} // namespace rigExec
