#include "rigExec/api/compile.h"
#include "rigExec/internal/compile/rigDescData.h"
#include "rigExec/internal/program/program.h"
#include "rigExecGraph/windowHistory.h"

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
    /// Per node: the nodes that run after it, from data and order edges.
    std::vector<std::vector<uint32_t>> successors;
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
    s.successors.assign(s.nodeCount, {});
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
/// both ends have one type, and records the data edges.
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
        s.successors[c.from.node].push_back(c.to.node);
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

/// Adds the host's explicit order pairs to the data edges.
void
AddOrders(CompileState &s)
{
    for (const auto &[first, second] : s.data.orders) {
        s.successors[first].push_back(second);
    }
}

/// Places each node one level after the deepest node it depends on; fails
/// on a cycle.
bool
AssignLevels(CompileState &s)
{
    std::vector<uint32_t> pending(s.nodeCount, 0);
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        std::vector<uint32_t> &next = s.successors[n];
        std::sort(next.begin(), next.end());
        next.erase(std::unique(next.begin(), next.end()), next.end());
        for (const uint32_t succ : next) {
            ++pending[succ];
        }
    }
    s.level.assign(s.nodeCount, 0);
    std::vector<uint32_t> ready;
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        if (!pending[n])
            ready.push_back(n);
    }
    uint32_t placed = 0;
    while (!ready.empty()) {
        const uint32_t n = ready.back();
        ready.pop_back();
        ++placed;
        for (const uint32_t succ : s.successors[n]) {
            s.level[succ] = std::max(s.level[succ], s.level[n] + 1);
            if (!--pending[succ])
                ready.push_back(succ);
        }
    }
    if (placed != s.nodeCount) {
        for (uint32_t n = 0; n < s.nodeCount; ++n) {
            if (pending[n])
                return Fail(s.error, "order cycle through " +
                                         NodeText(s.registry, s.data, n));
        }
    }
    return true;
}

/// Records each Windowed node's frame count, and settles over data and
/// order predecessors which windowed nodes each depends on and which nodes
/// a history tick runs, as every evaluator settles its windowed ops.
void
SettleWindows(CompileState &s)
{
    RigExecProgram &program = s.program;
    program.windowOfNode.assign(s.nodeCount, kRigExecInvalidIndex);
    std::vector<uint32_t> windowed;
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        const RigExecOpDesc &d = s.OpDesc(n);
        if (d.threading != RigExecOpThreading::Windowed)
            continue;
        const RigExecDescNode &node = s.data.nodes[n];
        RigExecProgramWindow window;
        window.node = n;
        window.frames =
            d.windowFrames(node.hasPayload ? node.payload.Data() : nullptr);
        program.windowOfNode[n] = uint32_t(windowed.size());
        windowed.push_back(n);
        program.windowedNodes.push_back(std::move(window));
    }
    if (windowed.empty())
        return;
    std::vector<std::vector<uint32_t>> predecessors(s.nodeCount);
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        for (const uint32_t succ : s.successors[n])
            predecessors[succ].push_back(n);
    }
    RigExecWindowLayout layout;
    RigExecSettleWindowLayout(
        s.nodeCount,
        [&](uint32_t n) -> const std::vector<uint32_t> & {
            return predecessors[n];
        },
        windowed, &layout);
    program.historyMask = std::move(layout.mask);
    for (size_t w = 0; w < windowed.size(); ++w) {
        program.windowedNodes[w].inner = std::move(layout.inner[w]);
        program.windowedNodes[w].feedsWindow = layout.feedsWindow[w] != 0;
    }
}

/// Groups nodes into batches by level, then op, then node id, splitting a
/// batch at \p limit nodes (0 for no limit).
void
BuildBatches(CompileState &s, size_t limit)
{
    RigExecProgram &program = s.program;
    const std::vector<uint32_t> &level = s.level;
    std::vector<uint32_t> order(s.nodeCount);
    std::iota(order.begin(), order.end(), 0u);
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
        return std::make_tuple(level[a], s.data.nodes[a].op, a) <
               std::make_tuple(level[b], s.data.nodes[b].op, b);
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
}

/// Places every slot in the value arena, in slot order.
void
LayoutArena(CompileState &s)
{
    size_t offset = 0;
    for (RigExecProgramSlot &slot : s.program.slots) {
        const RigExecTypeDesc &t = *s.registry.TypeAt(slot.type).desc;
        offset = RigExecPlaceValue(offset, t.size, t.align);
        slot.offset = offset;
        offset += t.size;
    }
    s.program.arenaSize = offset;
}

/// Places each node's payload in the payload blob and its state in the
/// state arena, in node order, and copies the payloads in.
void
LayoutPayloadsAndState(CompileState &s)
{
    RigExecProgram &program = s.program;
    size_t payloadSize = 0;
    size_t stateSize = 0;
    for (uint32_t n = 0; n < s.nodeCount; ++n) {
        const RigExecOpDesc &d = s.OpDesc(n);
        if (s.data.nodes[n].hasPayload) {
            payloadSize =
                RigExecPlaceValue(payloadSize, d.payload.size, d.payload.align);
            program.nodes[n].payloadOffset = payloadSize;
            payloadSize += d.payload.size;
        }
        if (d.state.size) {
            stateSize =
                RigExecPlaceValue(stateSize, d.state.size, d.state.align);
            program.nodes[n].stateOffset = stateSize;
            stateSize += d.state.size;
        }
    }
    program.stateSize = stateSize;
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

    CompileState s{registry, data, *program, error};
    s.nodeCount = uint32_t(data.nodes.size());
    InitNodes(s);
    if (!ConnectNodes(s) || !BindInputs(s) || !CheckInputsAndLists(s))
        return nullptr;
    BindOutputs(s);
    AddOrders(s);
    if (!AssignLevels(s))
        return nullptr;
    SettleWindows(s);
    BuildBatches(s, options.maxBatchNodes);
    LayoutArena(s);
    LayoutPayloadsAndState(s);
    return program;
}

} // namespace rigExec
