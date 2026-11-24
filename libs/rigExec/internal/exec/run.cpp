#include "rigExec/internal/exec/run.h"
#include "rigExec/internal/exec/addressWait.h"

#include <algorithm>
#include <cstring>
#include <thread>
#include <utility>

namespace rigExec {

namespace {

const RigExecTypeDesc &
TypeOf(uint32_t type)
{
    return *RigExecRegistry::Get().TypeAt(type).desc;
}

const RigExecOpDesc &
OpOf(uint32_t op)
{
    return *RigExecRegistry::Get().OpAt(op).desc;
}

/// The integer \p value holds, read as \p t's one column; SIZE_MAX for a
/// type that is not one integer.
size_t
IntegerOf(const RigExecTypeDesc &t, const void *value)
{
    if (t.columnCount != 1)
        return SIZE_MAX;
    switch (t.columnKind) {
    case RigExecColumnKind::UInt8:
        return *static_cast<const uint8_t *>(value);
    case RigExecColumnKind::UInt32:
        return *static_cast<const uint32_t *>(value);
    case RigExecColumnKind::Int32: {
        const int32_t v = *static_cast<const int32_t *>(value);
        return v < 0 ? 0 : size_t(v);
    }
    case RigExecColumnKind::UInt64:
        return size_t(*static_cast<const uint64_t *>(value));
    default:
        return SIZE_MAX;
    }
}

enum RangePlanCode : uint8_t { kPlanSkip, kPlanRanges, kPlanWhole };

size_t
Scaled(size_t elements, size_t count)
{
    if (elements == SIZE_MAX || (count && elements > SIZE_MAX / count))
        return SIZE_MAX;
    return elements * count;
}

} // namespace

RigExecPublishedSet::~RigExecPublishedSet()
{
    for (const RigExecPublishedPool &pool : pools) {
        const RigExecTypeDesc &t = TypeOf(pool.type);
        if (!t.destroy)
            continue;
        for (const std::unique_ptr<RigExecPublishedArena> &arena : pool.arenas)
            t.destroy(arena->storage.Data());
    }
}

RigExecPublishedArena &
RigExecPublishedSet::Add(RigExecPublishedPool &pool)
{
    const RigExecTypeDesc &t = TypeOf(pool.type);
    auto arena = std::make_unique<RigExecPublishedArena>();
    arena->storage.Allocate(t.size);
    if (t.construct)
        t.construct(arena->storage.Data());
    if (t.reserve && pool.reserved)
        t.reserve(arena->storage.Data(), pool.reserved);
    pool.arenas.push_back(std::move(arena));
    return *pool.arenas.back();
}

RigExecRun::~RigExecRun()
{
    if (!program)
        return;
    for (const RigExecProgramSlot &slot : program->slots) {
        const RigExecTypeDesc &t = TypeOf(slot.type);
        if (slot.arena == RigExecArenaKind::Work && t.destroy)
            t.destroy(work.Data() + slot.offset);
    }
    for (const RigExecProgramNode &node : program->nodes) {
        const RigExecOpDesc &d = OpOf(node.op);
        if (node.stateOffset != SIZE_MAX && d.state.destroy)
            d.state.destroy(State() + node.stateOffset);
    }
    for (size_t at = 0; at < scratch.size(); ++at) {
        const RigExecOpDesc &d =
            OpOf(program->nodes[program->batchNodes[at]].op);
        for (const RigExecAlignedBuffer &buffer : scratch[at])
            d.range.scratch.destroy(buffer.Data());
    }
}

const void *
RigExecRun::Value(uint32_t slot) const
{
    const RigExecProgramSlot &place = program->slots[slot];
    switch (place.arena) {
    case RigExecArenaKind::Input:
        return channel->Front() + place.offset;
    case RigExecArenaKind::Published: {
        const RigExecPublishedPool &pool = published->pools[place.arenaIndex];
        return pool.arenas[pool.current]->storage.Data();
    }
    case RigExecArenaKind::Work:
    case RigExecArenaKind::Constant:
        break;
    }
    return work.Data() + place.offset;
}

void
RigExecRun::_Point(uint32_t slot, void *value)
{
    for (uint32_t c = program->cellStart[slot];
         c < program->cellStart[slot + 1]; ++c) {
        const RigExecProgramCell &cell = program->cells[c];
        if (cell.kind == RigExecProgramCell::Output)
            outputs[cell.index] = value;
    }
    _PointReaders(slot, value);
}

void
RigExecRun::_PointReaders(uint32_t slot, const void *value)
{
    for (uint32_t c = program->cellStart[slot];
         c < program->cellStart[slot + 1]; ++c) {
        const RigExecProgramCell &cell = program->cells[c];
        if (cell.kind == RigExecProgramCell::Input)
            inputs[cell.index] = value;
        else if (cell.kind == RigExecProgramCell::ListItem)
            listItems[cell.index] = value;
    }
}

void *
RigExecRun::_Written(uint32_t slot) const
{
    const RigExecProgramSlot &place = program->slots[slot];
    if (place.arena == RigExecArenaKind::Published) {
        const RigExecPublishedPool &pool = published->pools[place.arenaIndex];
        return pool.arenas[pool.current]->storage.Data();
    }
    return work.Data() + place.offset;
}

void
RigExecRun::Bind(const RigExecProgram &p, const RigExecInputChannel &input,
                 const std::vector<RigExecWindowRing> &rings)
{
    program = &p;
    channel = &input;
    work.Allocate(p.workSize);
    for (const RigExecProgramSlot &slot : p.slots) {
        const RigExecTypeDesc &t = TypeOf(slot.type);
        if (slot.arena == RigExecArenaKind::Work && t.construct)
            t.construct(work.Data() + slot.offset);
    }
    for (const RigExecProgramNode &node : p.nodes) {
        const RigExecOpDesc &d = OpOf(node.op);
        if (node.stateOffset != SIZE_MAX && d.state.create)
            d.state.create(State() + node.stateOffset);
    }
    published = std::make_shared<RigExecPublishedSet>();
    published->pools.resize(p.publishedSlots.size());
    for (size_t i = 0; i < p.publishedSlots.size(); ++i) {
        RigExecPublishedPool &pool = published->pools[i];
        pool.slot = p.publishedSlots[i];
        pool.type = p.slots[pool.slot].type;
        published->Add(pool);
    }
    inputs.assign(p.portSlots.size(), nullptr);
    outputs.assign(p.portSlots.size(), nullptr);
    listItems.assign(p.listSlots.size(), nullptr);
    lists.resize(p.lists.size());
    for (size_t i = 0; i < p.lists.size(); ++i) {
        const RigExecProgramList &list = p.lists[i];
        lists[i] = {listItems.data() + list.firstSlot, list.count};
        inputs[list.port] = &lists[i];
    }
    boundFront = channel->front;
    for (uint32_t slot = 0; slot < p.slots.size(); ++slot) {
        if (p.slots[slot].arena == RigExecArenaKind::Input)
            _PointReaders(slot, Value(slot));
        else
            _Point(slot, _Written(slot));
    }
    captures.assign(p.windowedNodes.size(), nullptr);
    windows.assign(p.windowedNodes.size(), RigExecWindowRun());
    for (size_t w = 0; w < windows.size(); ++w) {
        windows[w].ring = &rings[w];
        windows[w].capture = &captures[w];
    }
    batchWindows.assign(p.batchNodes.size(), nullptr);
    for (size_t i = 0; i < p.batchNodes.size(); ++i) {
        const uint32_t w = p.windowOfNode[p.batchNodes[i]];
        if (w != kRigExecInvalidIndex)
            batchWindows[i] = &windows[w];
    }
    _BindFields();
    bounds.assign(p.slots.size(), SIZE_MAX);
    reserved.assign(p.slots.size(), 0);
    for (uint32_t slot = 0; slot < p.slots.size(); ++slot) {
        if (p.bounds[slot].kind == RigExecOutputBound::Fixed) {
            bounds[slot] = p.bounds[slot].count;
            _Reserve(slot, bounds[slot]);
        }
    }
    failed.assign(p.batches.size(), 0);
    errors.assign(p.batches.size(), std::string());
    active.assign(p.batchNodes.size(), 0);
    batchActive.assign(p.batches.size(), 0);
    marked.reserve(p.batchNodes.size());
    walk.reserve(p.batchNodes.size());
    seen.assign(p.inputSlots.size(), 0);
    everything = true;
    const size_t levelCount =
        p.levelBatches.empty() ? 0 : p.levelBatches.size() - 1;
    batchLevel.assign(p.batches.size(), 0);
    rangedBatch.assign(p.batches.size(), 0);
    for (size_t level = 0; level < levelCount; ++level) {
        for (uint32_t b = p.levelBatches[level]; b < p.levelBatches[level + 1];
             ++b) {
            batchLevel[b] = uint32_t(level);
            rangedBatch[b] = OpOf(p.batches[b].op).range.run != nullptr;
        }
    }
    shared.reserve(p.batches.size() + p.chains.size());
    caller.reserve(p.batches.size());
    sharedStart.reserve(2 * levelCount + 1);
    callerStart.reserve(2 * levelCount + 1);
    levelChainStart.assign(levelCount + 1, 0);
    for (const RigExecProgramChain &chain : p.chains)
        ++levelChainStart[chain.level + 1];
    for (size_t level = 0; level < levelCount; ++level)
        levelChainStart[level + 1] += levelChainStart[level];
    levelChains.assign(p.chains.size(), 0);
    {
        std::vector<uint32_t> next(levelChainStart.begin(),
                                   levelChainStart.end() - 1);
        for (uint32_t ch = 0; ch < p.chains.size(); ++ch)
            levelChains[next[p.chains[ch].level]++] = ch;
    }
    positionMember.assign(p.batchNodes.size(), kRigExecInvalidIndex);
    for (const RigExecProgramChain &chain : p.chains)
        for (uint32_t m = 0; m < chain.memberCount; ++m)
            positionMember[p.chainMembers[chain.firstMember + m]] = m;
    chainActive.assign(p.chains.size(), 0);
    chainHeadRan.assign(p.chains.size(), 0);
    chainRanges.assign(p.chains.size(), 0);
    chainBase.assign(p.chains.size(), 0);
    chainSlice.assign(p.chains.size(), 0);
    chainStop.assign(p.chains.size(), 0);
    chainErrors.assign(p.chains.size(), std::string());
    chainEmitted.resize(p.chains.size());
    segmentOf.assign(p.chainMembers.size(), kRigExecInvalidIndex);
    plans.assign(p.batchNodes.size(), kPlanSkip);
    elementCounts.assign(p.batchNodes.size(), 0);
    instanceRanges.assign(p.batchNodes.size(), 0);
    scratch.resize(p.batchNodes.size());
    levelSplit.assign(levelCount, 0);
    levelFailed = std::make_unique<std::atomic<uint8_t>[]>(levelCount);
}

void
RigExecRun::_BindFields()
{
    const RigExecProgram &p = *program;
    payloadRuns.resize(p.payloadRuns.size());
    for (size_t i = 0; i < p.payloadRuns.size(); ++i)
        payloadRuns[i] = {p.payloads.Data() + p.payloadRuns[i].offset,
                          p.payloadRuns[i].count};
    payloadColumns.resize(p.payloadColumns.size());
    for (size_t i = 0; i < p.payloadColumns.size(); ++i) {
        const RigExecProgramColumn &c = p.payloadColumns[i];
        payloadColumns[i] = {p.payloads.Data() + c.offset, c.stride, c.count,
                             c.firstRun == kRigExecInvalidIndex
                                 ? nullptr
                                 : &payloadRuns[c.firstRun]};
    }
    stateRuns.resize(p.stateRuns.size());
    stateBuffers.resize(p.stateRuns.size());
    for (size_t i = 0; i < p.stateRuns.size(); ++i) {
        const RigExecProgramRun &r = p.stateRuns[i];
        stateRuns[i] = {r.offset == SIZE_MAX ? nullptr : State() + r.offset,
                        r.offset == SIZE_MAX ? 0 : r.count};
    }
    stateColumns.resize(p.stateColumns.size());
    for (size_t i = 0; i < p.stateColumns.size(); ++i) {
        const RigExecProgramColumn &c = p.stateColumns[i];
        const bool runs = c.firstRun != kRigExecInvalidIndex;
        stateColumns[i] = {State() + c.offset, c.stride, c.count,
                           runs ? &stateRuns[c.firstRun] : nullptr};
    }
    for (const RigExecProgramBatch &batch : p.batches) {
        if (batch.firstStateColumn == kRigExecInvalidIndex)
            continue;
        const size_t fields = OpOf(batch.op).stateFieldCount;
        for (size_t i = batch.firstStateColumn;
             i < batch.firstStateColumn + fields; ++i) {
            const RigExecTypeDesc &t = TypeOf(p.stateColumns[i].type);
            if (!t.construct || p.stateColumns[i].fromInput)
                continue;
            for (uint32_t k = 0; k < batch.nodeCount; ++k) {
                const RigExecFieldView<std::byte> values =
                    RigExecFieldAt<std::byte>(stateColumns[i], k);
                for (size_t v = 0; v < values.count; ++v)
                    t.construct(values.data + v * t.size);
            }
        }
    }
}

void
RigExecRun::_Reserve(uint32_t slot, size_t count)
{
    if (count == SIZE_MAX || count <= reserved[slot])
        return;
    const RigExecProgramSlot &place = program->slots[slot];
    const RigExecTypeDesc &t = TypeOf(place.type);
    if (!t.reserve)
        return;
    reserved[slot] = count;
    if (place.arena == RigExecArenaKind::Work) {
        t.reserve(work.Data() + place.offset, count);
    } else if (place.arena == RigExecArenaKind::Published) {
        RigExecPublishedPool &pool = published->pools[place.arenaIndex];
        pool.reserved = count;
        for (const std::unique_ptr<RigExecPublishedArena> &arena : pool.arenas)
            t.reserve(arena->storage.Data(), count);
    }
}

void
RigExecRun::_SettleBounds()
{
    const RigExecProgram &p = *program;
    for (const uint32_t slot : p.derivedBounds) {
        const RigExecProgramBound &bound = p.bounds[slot];
        size_t most = 0;
        for (uint32_t i = 0; i < bound.sourceCount; ++i) {
            const RigExecProgramBoundSource &source =
                p.boundSources[bound.firstSource + i];
            const RigExecProgramSlot &place = p.slots[source.slot];
            size_t measured = SIZE_MAX;
            if (place.arena == RigExecArenaKind::Input) {
                const RigExecTypeDesc &t = TypeOf(place.type);
                const void *value = channel->Front() + place.offset;
                measured = source.integer ? IntegerOf(t, value)
                           : t.elements   ? t.elements(value)
                                          : SIZE_MAX;
            } else if (!source.integer) {
                measured = bounds[source.slot];
            }
            most = std::max(most, measured);
        }
        bounds[slot] = Scaled(most, bound.count);
        _Reserve(slot, bounds[slot]);
    }
}

void
RigExecRun::_Mark(uint32_t node, const char *allowed)
{
    const uint32_t at = program->batchPosition[node];
    if (active[at] || (allowed && !allowed[at]))
        return;
    active[at] = 1;
    marked.push_back(at);
    ++batchActive[program->positionBatch[at]];
    walk.push_back(node);
}

void
RigExecRun::_MarkDirty(bool historyTick)
{
    const RigExecProgram &p = *program;
    for (const uint32_t at : marked) {
        active[at] = 0;
        batchActive[p.positionBatch[at]] = 0;
    }
    marked.clear();
    walk.clear();
    const char *allowed = historyTick ? p.batchHistoryMask.data() : nullptr;
    if (everything) {
        for (uint32_t n = 0; n < p.nodes.size(); ++n)
            _Mark(n, allowed);
        return;
    }
    for (uint32_t i = 0; i < p.inputSlots.size(); ++i) {
        if (channel->FrontVersion(i) == seen[i])
            continue;
        const uint32_t slot = p.inputSlots[i];
        for (uint32_t r = p.readerStart[slot]; r < p.readerStart[slot + 1]; ++r)
            _Mark(p.readers[r], allowed);
    }
    for (const uint32_t n : p.volatileNodes)
        _Mark(n, allowed);
    for (size_t at = 0; at < walk.size(); ++at) {
        const uint32_t n = walk[at];
        for (uint32_t o = p.nodeOutputStart[n]; o < p.nodeOutputStart[n + 1];
             ++o) {
            const uint32_t slot = p.nodeOutputs[o];
            for (uint32_t r = p.readerStart[slot]; r < p.readerStart[slot + 1];
                 ++r)
                _Mark(p.readers[r], allowed);
        }
    }
}

bool
RigExecRun::_Runs(uint32_t slot) const
{
    const uint32_t n = program->producer[slot];
    return n != kRigExecInvalidIndex && active[program->batchPosition[n]];
}

void
RigExecRun::_PrepareOutputs()
{
    for (RigExecPublishedPool &pool : published->pools) {
        if (!_Runs(pool.slot))
            continue;
        RigExecPublishedArena *arena = pool.arenas[pool.current].get();
        if (arena->pins.load(std::memory_order_acquire) == 0)
            continue;
        uint32_t next = kRigExecInvalidIndex;
        for (uint32_t i = 0; i < pool.arenas.size(); ++i) {
            if (i != pool.current &&
                pool.arenas[i]->pins.load(std::memory_order_acquire) == 0) {
                next = i;
                break;
            }
        }
        if (next == kRigExecInvalidIndex) {
            next = uint32_t(pool.arenas.size());
            published->Add(pool);
        }
        pool.current = next;
        _Point(pool.slot, pool.arenas[next]->storage.Data());
    }
}

void
RigExecRun::_Publish()
{
    ++generation;
    for (RigExecPublishedPool &pool : published->pools)
        if (_Runs(pool.slot))
            pool.arenas[pool.current]->generation = generation;
    for (uint32_t i = 0; i < program->inputSlots.size(); ++i)
        seen[i] = channel->FrontVersion(i);
    everything = false;
}

size_t
RigExecRun::_SizeFromInputs(const RigExecOpBatch &view,
                            const RigExecProgramBatch &batch,
                            const RigExecRegisteredOp &op, std::string *error,
                            size_t only)
{
    const RigExecOpDesc &d = *op.desc;
    if (!d.payloadFieldCount && !d.stateFieldCount)
        return view.count;
    for (size_t k = 0; k < view.count; ++k) {
        if (!RigExecBatchActive(view, k) || (only != SIZE_MAX && k != only))
            continue;
        for (int payload = 1; payload >= 0; --payload) {
            const RigExecFieldDesc *fields =
                payload ? d.payloadFields : d.stateFields;
            const size_t count =
                payload ? d.payloadFieldCount : d.stateFieldCount;
            for (size_t f = 0; f < count; ++f) {
                const RigExecFieldDesc &field = fields[f];
                if (field.extent != RigExecFieldExtent::FromInput)
                    continue;
                const uint32_t port = field.port->Index();
                const void *value = view.inputs[k * view.portCount + port];
                const size_t elements =
                    value ? TypeOf(op.portTypes[port]).count(value) : 0;
                if (payload) {
                    const RigExecPayloadColumn &column = view.payloadFields[f];
                    if (column.runs[k].count == elements)
                        continue;
                    if (error)
                        *error = std::string("field ") + field.name +
                                 " holds " +
                                 std::to_string(column.runs[k].count) +
                                 " values for " + std::to_string(elements) +
                                 " input elements";
                    return k;
                }
                const RigExecProgramColumn &c =
                    program->stateColumns[batch.firstStateColumn + f];
                RigExecFieldRunOf<std::byte> &run = stateRuns[c.firstRun + k];
                if (run.data && run.count == elements)
                    continue;
                const RigExecTypeDesc &t = TypeOf(c.type);
                RigExecAlignedBuffer &buffer = stateBuffers[c.firstRun + k];
                buffer.Allocate(elements * t.size);
                if (t.construct)
                    for (size_t v = 0; v < elements; ++v)
                        t.construct(buffer.Data() + v * t.size);
                run = {buffer.Data(), elements};
            }
        }
    }
    return view.count;
}

bool
RigExecRun::_CheckBounds(const RigExecOpBatch &view,
                         const RigExecProgramBatch &batch,
                         const RigExecRegisteredOp &op, size_t *at,
                         std::string *error) const
{
    for (size_t k = 0; k < view.count; ++k) {
        if (!RigExecBatchActive(view, k))
            continue;
        for (const uint32_t p : op.boundedOutputs) {
            const size_t port = k * view.portCount + p;
            const size_t bound =
                bounds[program->portSlots[batch.firstPort + port]];
            if (bound == SIZE_MAX)
                continue;
            const size_t held =
                TypeOf(op.portTypes[p]).elements(view.outputs[port]);
            if (held <= bound)
                continue;
            *at = k;
            if (error)
                *error = std::string("output ") + op.desc->ports[p].name +
                         " holds " + std::to_string(held) +
                         " elements, past its bound of " +
                         std::to_string(bound);
            return false;
        }
    }
    return true;
}

RigExecOpBatch
RigExecRun::_View(uint32_t b) const
{
    const RigExecProgramBatch &batch = program->batches[b];
    const RigExecOpDesc &d = OpOf(batch.op);
    RigExecOpBatch view;
    view.op = &d;
    view.count = batch.nodeCount;
    view.active = active.data() + batch.firstNode;
    if (batch.payloadOffset != SIZE_MAX) {
        view.payloads = program->payloads.Data() + batch.payloadOffset;
        view.payloadStride = batch.payloadStride;
    }
    if (batch.stateOffset != SIZE_MAX) {
        view.states = State() + batch.stateOffset;
        view.stateStride = batch.stateStride;
    }
    view.portCount = d.portCount;
    view.useSimd = program->useSimd;
    view.inputs = inputs.data() + batch.firstPort;
    view.outputs = outputs.data() + batch.firstPort;
    if (d.threading == RigExecOpThreading::Windowed)
        view.windows = batchWindows.data() + batch.firstNode;
    if (d.payloadFieldCount)
        view.payloadFields = payloadColumns.data() + batch.firstPayloadColumn;
    if (d.stateFieldCount)
        view.stateFields = stateColumns.data() + batch.firstStateColumn;
    return view;
}

void
RigExecRun::_Fail(uint32_t b, uint32_t instance, const std::string &error)
{
    const RigExecProgramBatch &batch = program->batches[b];
    const RigExecOpDesc &d = OpOf(batch.op);
    const uint32_t n = program->batchNodes[batch.firstNode + instance];
    const uint32_t w = program->windowOfNode[n];
    if (w != kRigExecInvalidIndex)
        captures[w].reset();
    failed[b] = 1;
    errors[b] = "node " + std::to_string(n) + " (" +
                (d.nameSpace ? d.nameSpace : "") + ":" +
                (d.name ? d.name : "") + ") failed" +
                (error.empty() ? "" : ": " + error);
}

void
RigExecRun::_RunBatch(uint32_t b)
{
    if (!batchActive[b])
        return;
    const RigExecProgramBatch &batch = program->batches[b];
    const RigExecRegisteredOp &op = RigExecRegistry::Get().OpAt(batch.op);
    RigExecOpBatch view = _View(b);
    size_t at = 0;
    std::string error;
    const size_t ready = _SizeFromInputs(view, batch, op, &error);
    if (ready == view.count) {
        if (op.runBatch(view, &at, &error) &&
            _CheckBounds(view, batch, op, &at, &error))
            return;
    } else {
        at = ready;
        std::string before;
        view.count = ready;
        if (ready && !op.runBatch(view, &at, &before))
            error = std::move(before);
    }
    _Fail(b, uint32_t(at), error);
}

size_t
RigExecRun::_RangeCount(size_t elements) const
{
    const size_t grain = program->rangeGrain;
    if (!grain || elements == SIZE_MAX || elements < program->rangeThreshold ||
        elements <= grain)
        return 1;
    return (elements + grain - 1) / grain;
}

void
RigExecRun::_Schedule()
{
    const RigExecProgram &p = *program;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    const uint32_t batches = uint32_t(p.batches.size());
    const uint32_t chains = uint32_t(p.chains.size());
    sharedStart.assign(1, 0);
    callerStart.assign(1, 0);
    shared.clear();
    caller.clear();
    slotChain.clear();
    const auto close = [&] {
        sharedStart.push_back(uint32_t(shared.size()));
        callerStart.push_back(uint32_t(caller.size()));
    };
    for (uint32_t c = 0; c < chains; ++c) {
        const RigExecProgramChain &chain = p.chains[c];
        chainActive[c] = 0;
        chainHeadRan[c] = 0;
        for (uint32_t m = 0; m < chain.memberCount; ++m)
            chainActive[c] |= active[p.chainMembers[chain.firstMember + m]];
    }
    for (size_t level = 0; level < levelSplit.size(); ++level) {
        for (uint32_t b = p.levelBatches[level]; b < p.levelBatches[level + 1];
             ++b)
            if (!rangedBatch[b])
                (p.batches[b].ownerThread ? caller : shared).push_back(b);
        for (uint32_t i = levelChainStart[level];
             i < levelChainStart[level + 1]; ++i)
            if (chainActive[levelChains[i]])
                shared.push_back(batches + levelChains[i]);
        close();
        const size_t before = slotChain.size();
        for (uint32_t i = levelChainStart[level];
             i < levelChainStart[level + 1]; ++i) {
            const uint32_t c = levelChains[i];
            if (!chainActive[c])
                continue;
            const RigExecProgramChain &chain = p.chains[c];
            const uint32_t head = p.chainMembers[chain.firstMember];
            const RigExecProgramBatch &batch = p.batches[p.positionBatch[head]];
            const RigExecRegisteredOp &op = registry.OpAt(batch.op);
            const uint32_t out =
                p.portSlots[batch.firstPort +
                            (head - batch.firstNode) * op.desc->portCount +
                            op.rangeOutputs.front()];
            const uint32_t ranges = uint32_t(_RangeCount(bounds[out]));
            chainRanges[c] = ranges;
            chainBase[c] = uint32_t(slotChain.size());
            chainSlice[c] = uint32_t(shared.size());
            for (uint32_t m = 0; m < chain.memberCount; ++m)
                instanceRanges[p.chainMembers[chain.firstMember + m]] = ranges;
            const uint32_t slots = chain.memberCount * ranges;
            for (uint32_t s = 0; s < slots; ++s) {
                shared.push_back(batches + chains + chainBase[c] + s);
                slotChain.push_back(c);
            }
        }
        levelSplit[level] = slotChain.size() != before;
        levelFailed[level].store(0, std::memory_order_relaxed);
        if (levelSplit[level])
            close();
    }
    const size_t slots = slotChain.size();
    if (slotDoneSize < slots) {
        slotDone = std::make_unique<std::atomic<uint8_t>[]>(slots);
        slotDoneSize = slots;
    }
    if (slotFailed.size() < slots)
        slotFailed.resize(slots);
    if (slotErrors.size() < slots)
        slotErrors.resize(slots);
    if (slotReads.size() < slots)
        slotReads.resize(slots);
}

bool
RigExecRun::_RunHead(uint32_t at, std::string *error)
{
    const uint32_t b = program->positionBatch[at];
    const RigExecProgramBatch &batch = program->batches[b];
    const RigExecRegisteredOp &op = RigExecRegistry::Get().OpAt(batch.op);
    const RigExecOpDesc &d = *op.desc;
    const uint32_t k = at - batch.firstNode;
    const RigExecOpBatch view = _View(b);
    if (_SizeFromInputs(view, batch, op, error, k) != view.count)
        return false;
    const size_t ports = size_t(k) * d.portCount;
    const void *counted = view.inputs[ports + op.rangeCountPort];
    const size_t count = TypeOf(op.portTypes[op.rangeCountPort]).count(counted);
    for (const uint32_t port : op.rangeOutputs) {
        const size_t bound =
            bounds[program->portSlots[batch.firstPort + ports + port]];
        if (count > bound) {
            *error = std::string("output ") + d.ports[port].name + " holds " +
                     std::to_string(count) + " elements, past its bound of " +
                     std::to_string(bound);
            return false;
        }
        TypeOf(op.portTypes[port]).resize(view.outputs[ports + port], count);
    }
    RigExecRangePlan plan = RigExecRangePlan::Ranges;
    if (d.range.accept)
        plan = d.range.accept(view, k, count, error);
    if (plan == RigExecRangePlan::Refuse) {
        if (error->empty())
            *error = "refused";
        return false;
    }
    if (d.range.prepare && !d.range.prepare(view, k, count, error))
        return false;
    if (plan == RigExecRangePlan::Ranges &&
        _RangeCount(count) > instanceRanges[at])
        plan = RigExecRangePlan::Whole;
    std::vector<RigExecAlignedBuffer> &buffers = scratch[at];
    if (d.range.scratch.size) {
        while (buffers.size() < instanceRanges[at]) {
            buffers.emplace_back();
            buffers.back().Allocate(d.range.scratch.size);
            d.range.scratch.create(buffers.back().Data());
        }
    }
    plans[at] = plan == RigExecRangePlan::Whole ? kPlanWhole : kPlanRanges;
    elementCounts[at] = count;
    return true;
}

uint32_t
RigExecRun::_Ranges(uint32_t at) const
{
    return plans[at] == kPlanWhole ? 1
                                   : uint32_t(_RangeCount(elementCounts[at]));
}

bool
RigExecRun::_RangeOf(uint32_t at, uint32_t index, RigExecOpRange *range) const
{
    range->count = elementCounts[at];
    range->index = index;
    bool runs = plans[at] == kPlanWhole ? index == 0 : plans[at] == kPlanRanges;
    if (runs && plans[at] == kPlanRanges) {
        range->ranges = uint32_t(_RangeCount(range->count));
        runs = index < range->ranges;
    }
    if (runs && range->ranges > 1) {
        const size_t grain = program->rangeGrain;
        range->begin = size_t(index) * grain;
        range->end = std::min(range->count, range->begin + grain);
    } else {
        range->end = range->count;
    }
    return runs && range->begin < range->end;
}

template <class Visit>
void
RigExecRun::_ForEachRead(uint32_t c, uint32_t start, uint32_t r,
                         const Visit &visit) const
{
    const RigExecProgram &p = *program;
    const RigExecProgramChain &chain = p.chains[c];
    const uint32_t ranges = chainRanges[c];
    for (uint32_t m = start;
         m < chainStop[c] &&
         (segmentOf[chain.firstMember + m] == start ||
          segmentOf[chain.firstMember + m] == kRigExecInvalidIndex);
         ++m) {
        if (segmentOf[chain.firstMember + m] != start)
            continue;
        const uint32_t at = p.chainMembers[chain.firstMember + m];
        const uint32_t g = chain.firstMember + m;
        for (uint32_t f = p.chainFeedStart[g]; f < p.chainFeedStart[g + 1];
             ++f) {
            const RigExecProgramChainFeed &feed = p.chainFeeds[f];
            const uint32_t from = segmentOf[chain.firstMember + feed.member];
            if (from == kRigExecInvalidIndex || from == start)
                continue;
            const uint32_t source =
                p.chainMembers[chain.firstMember + feed.member];
            const uint32_t sourceRanges = _Ranges(source);
            uint32_t first = 0, last = sourceRanges - 1;
            if (sourceRanges > 1 && plans[at] != kPlanWhole) {
                if (!feed.neighbourhood) {
                    first = last = r;
                } else {
                    const auto &reads =
                        slotReads[chainBase[c] + m * ranges + r];
                    if (reads.second <= reads.first)
                        continue;
                    const size_t grain = program->rangeGrain;
                    first = uint32_t(reads.first / grain);
                    last = std::min(uint32_t((reads.second - 1) / grain),
                                    sourceRanges - 1);
                }
            }
            visit(feed.member, first, last);
        }
    }
}

void
RigExecRun::_OrderChain(uint32_t c)
{
    const RigExecProgram &p = *program;
    const RigExecProgramChain &chain = p.chains[c];
    const uint32_t ranges = chainRanges[c];
    const uint32_t batches = uint32_t(p.batches.size());
    const uint32_t first = batches + uint32_t(p.chains.size()) + chainBase[c];
    std::vector<uint32_t> &emitted = chainEmitted[c];
    emitted.assign(chain.memberCount, 0);
    uint32_t *out = shared.data() + chainSlice[c];
    size_t placed = 0;
    for (bool progress = true; progress;) {
        progress = false;
        for (uint32_t m = 0; m < chainStop[c]; ++m) {
            if (segmentOf[chain.firstMember + m] != m)
                continue;
            const uint32_t tasks =
                _Ranges(p.chainMembers[chain.firstMember + m]);
            while (emitted[m] < tasks) {
                bool ready = true;
                _ForEachRead(c, m, emitted[m],
                             [&](uint32_t source, uint32_t, uint32_t last) {
                                 const uint32_t start =
                                     segmentOf[chain.firstMember + source];
                                 ready = ready && emitted[start] > last;
                             });
                if (!ready)
                    break;
                out[placed++] = first + m * ranges + emitted[m]++;
                progress = true;
            }
        }
    }
    for (uint32_t m = 0; m < chain.memberCount; ++m)
        for (uint32_t r = emitted[m]; r < ranges; ++r)
            out[placed++] = first + m * ranges + r;
}

void
RigExecRun::_RunChainHeads(uint32_t c)
{
    const RigExecProgram &p = *program;
    const RigExecProgramChain &chain = p.chains[c];
    const RigExecRegistry &registry = RigExecRegistry::Get();
    const uint32_t ranges = chainRanges[c];
    chainHeadRan[c] = 1;
    chainStop[c] = chain.memberCount;
    for (uint32_t m = 0; m < chain.memberCount; ++m) {
        const uint32_t at = p.chainMembers[chain.firstMember + m];
        plans[at] = kPlanSkip;
        if (!active[at])
            continue;
        chainErrors[c].clear();
        if (!_RunHead(at, &chainErrors[c])) {
            chainStop[c] = m;
            levelFailed[chain.level].store(1, std::memory_order_release);
            break;
        }
    }
    const uint32_t base = chainBase[c];
    for (uint32_t s = base; s < base + chain.memberCount * ranges; ++s) {
        slotDone[s].store(0, std::memory_order_relaxed);
        slotFailed[s] = 0;
    }
    uint32_t start = kRigExecInvalidIndex;
    uint32_t previous = kRigExecInvalidIndex;
    for (uint32_t m = 0; m < chain.memberCount; ++m) {
        const uint32_t g = chain.firstMember + m;
        const uint32_t at = p.chainMembers[g];
        segmentOf[g] = kRigExecInvalidIndex;
        if (m >= chainStop[c] || !active[at])
            continue;
        bool starts = start == kRigExecInvalidIndex ||
                      plans[at] == kPlanWhole || plans[previous] == kPlanWhole;
        bool halo = false;
        for (uint32_t f = p.chainFeedStart[g]; f < p.chainFeedStart[g + 1];
             ++f) {
            const RigExecProgramChainFeed &feed = p.chainFeeds[f];
            if (feed.neighbourhood &&
                segmentOf[chain.firstMember + feed.member] !=
                    kRigExecInvalidIndex)
                halo = true;
        }
        starts = starts || halo;
        if (starts)
            start = m;
        segmentOf[g] = start;
        previous = at;
        if (!halo)
            continue;
        const uint32_t b = p.positionBatch[at];
        const RigExecOpDesc &d = *registry.OpAt(p.batches[b].op).desc;
        const RigExecOpBatch view = _View(b);
        for (uint32_t r = 0; r < _Ranges(at); ++r) {
            auto &reads = slotReads[base + m * ranges + r];
            reads = {0, elementCounts[at]};
            RigExecOpRange range;
            if (!_RangeOf(at, r, &range)) {
                reads = {0, 0};
                continue;
            }
            if (d.range.scratch.size)
                range.scratch = scratch[at][r].Data();
            if (d.range.reads)
                d.range.reads(view, at - p.batches[b].firstNode, range,
                              &reads.first, &reads.second);
        }
    }
    _OrderChain(c);
}

bool
RigExecRun::_RunSlot(uint32_t s)
{
    const RigExecProgram &p = *program;
    const uint32_t c = slotChain[s];
    const RigExecProgramChain &chain = p.chains[c];
    const uint32_t ranges = chainRanges[c];
    const uint32_t start = (s - chainBase[c]) / ranges;
    const uint32_t r = (s - chainBase[c]) % ranges;
    std::atomic<uint8_t> &levelFail = levelFailed[chain.level];
    if (start >= chainStop[c] ||
        segmentOf[chain.firstMember + start] != start ||
        r >= _Ranges(p.chainMembers[chain.firstMember + start]))
        return !levelFail.load(std::memory_order_acquire);
    bool poisoned = false;
    _ForEachRead(c, start, r, [&](uint32_t source, uint32_t lo, uint32_t hi) {
        for (uint32_t h = lo; h <= hi; ++h) {
            const std::atomic<uint8_t> &done =
                slotDone[chainBase[c] + source * ranges + h];
            uint8_t state = 0;
            for (int spins = 0;
                 (state = done.load(std::memory_order_acquire)) == 0; ++spins) {
                if (spins < 128)
                    RigExecCpuRelax();
                else
                    std::this_thread::yield();
            }
            poisoned = poisoned || state == 2;
        }
    });
    for (uint32_t m = start;
         m < chainStop[c] &&
         (segmentOf[chain.firstMember + m] == start ||
          segmentOf[chain.firstMember + m] == kRigExecInvalidIndex);
         ++m) {
        if (segmentOf[chain.firstMember + m] != start)
            continue;
        const uint32_t at = p.chainMembers[chain.firstMember + m];
        const uint32_t slot = chainBase[c] + m * ranges + r;
        RigExecOpRange range;
        if (!poisoned && _RangeOf(at, r, &range)) {
            const uint32_t b = p.positionBatch[at];
            const RigExecOpDesc &d = OpOf(p.batches[b].op);
            if (d.range.scratch.size)
                range.scratch = scratch[at][r].Data();
            if (!d.range.run(_View(b), at - p.batches[b].firstNode, range,
                             &slotErrors[slot])) {
                slotFailed[slot] = 1;
                poisoned = true;
                levelFail.store(1, std::memory_order_release);
            }
        }
        const uint8_t state = poisoned ? 2 : 1;
        if (plans[at] == kPlanWhole) {
            for (uint32_t h = 0; h < ranges; ++h)
                slotDone[chainBase[c] + m * ranges + h].store(
                    state, std::memory_order_release);
        } else {
            slotDone[slot].store(state, std::memory_order_release);
        }
    }
    return !levelFail.load(std::memory_order_acquire);
}

void
RigExecRun::_CompleteChains()
{
    const RigExecProgram &p = *program;
    for (uint32_t b = 0; b < p.batches.size(); ++b) {
        if (!rangedBatch[b])
            continue;
        const RigExecProgramBatch &batch = p.batches[b];
        for (uint32_t k = 0; k < batch.nodeCount && !failed[b]; ++k) {
            const uint32_t at = batch.firstNode + k;
            const uint32_t c = p.positionChain[at];
            if (!active[at] || !chainHeadRan[c])
                continue;
            const uint32_t m = positionMember[at];
            if (m == chainStop[c]) {
                _Fail(b, k, chainErrors[c]);
                break;
            }
            if (m > chainStop[c])
                continue;
            const uint32_t ranges = chainRanges[c];
            for (uint32_t r = 0; r < ranges; ++r) {
                const uint32_t slot = chainBase[c] + m * ranges + r;
                if (slotFailed[slot]) {
                    _Fail(b, k, slotErrors[slot]);
                    break;
                }
            }
        }
    }
}

bool
RigExecRun::_RunTask(void *context, uint32_t task)
{
    auto *self = static_cast<RigExecRun *>(context);
    const uint32_t batches = uint32_t(self->program->batches.size());
    const uint32_t chains = uint32_t(self->program->chains.size());
    if (task >= batches + chains)
        return self->_RunSlot(task - batches - chains);
    if (task >= batches) {
        self->_RunChainHeads(task - batches);
        return true;
    }
    self->_RunBatch(task);
    if (!self->failed[task])
        return true;
    const uint32_t level = self->batchLevel[task];
    if (!self->levelSplit[level])
        return false;
    self->levelFailed[level].store(1, std::memory_order_release);
    return true;
}

bool
RigExecRun::Run(bool historyTick, RigExecTaskSystem *tasks, std::string *error)
{
    const RigExecProgram &p = *program;
    if (channel->front != boundFront) {
        boundFront = channel->front;
        for (const uint32_t slot : p.inputSlots)
            _PointReaders(slot, channel->Front() + p.slots[slot].offset);
    }
    _SettleBounds();
    _MarkDirty(historyTick);
    _Schedule();
    _PrepareOutputs();
    for (size_t w = 0; w < windows.size(); ++w) {
        captures[w].reset();
        windows[w].captureOnly = historyTick && !p.windowedNodes[w].feedsWindow;
    }
    std::fill(failed.begin(), failed.end(), uint8_t(0));
    RigExecTaskSchedule schedule;
    schedule.levelCount = sharedStart.size() - 1;
    schedule.sharedStart = sharedStart.data();
    schedule.shared = shared.data();
    schedule.callerStart = callerStart.data();
    schedule.caller = caller.data();
    schedule.run = &RigExecRun::_RunTask;
    schedule.context = this;
    if (tasks ? tasks->Run(schedule) : RigExecRunTaskScheduleInline(schedule)) {
        _Publish();
        return true;
    }
    _CompleteChains();
    // The first failure in batch order, whichever thread hit it.
    for (size_t b = 0; b < p.batches.size(); ++b) {
        if (failed[b]) {
            if (error)
                *error = errors[b];
            break;
        }
    }
    for (RigExecWindowPacketPtr &capture : captures)
        capture.reset();
    everything = true;
    return false;
}

void
RigExecRun::CopyFrom(const RigExecRun &source)
{
    const RigExecProgram &p = *program;
    for (const RigExecProgramSlot &slot : p.slots) {
        if (slot.arena != RigExecArenaKind::Work)
            continue;
        const RigExecTypeDesc &t = TypeOf(slot.type);
        if (t.copy)
            t.copy(work.Data() + slot.offset, source.work.Data() + slot.offset);
        else
            std::memcpy(work.Data() + slot.offset,
                        source.work.Data() + slot.offset, t.size);
    }
    for (const RigExecProgramNode &node : p.nodes) {
        const RigExecOpDesc &d = OpOf(node.op);
        if (node.stateOffset == SIZE_MAX || !d.state.copy)
            continue;
        if (d.state.destroy)
            d.state.destroy(State() + node.stateOffset);
        d.state.copy(State() + node.stateOffset,
                     source.State() + node.stateOffset);
    }
    for (const RigExecProgramBatch &batch : p.batches) {
        if (batch.firstStateColumn == kRigExecInvalidIndex)
            continue;
        const size_t fields = OpOf(batch.op).stateFieldCount;
        for (size_t i = batch.firstStateColumn;
             i < batch.firstStateColumn + fields; ++i) {
            const RigExecProgramColumn &c = p.stateColumns[i];
            const size_t size = TypeOf(c.type).size;
            if (c.firstRun == kRigExecInvalidIndex) {
                std::memcpy(State() + c.offset, source.State() + c.offset,
                            batch.nodeCount * c.stride);
                continue;
            }
            for (uint32_t k = 0; k < batch.nodeCount; ++k) {
                const uint32_t r = c.firstRun + k;
                const RigExecFieldRunOf<std::byte> &from = source.stateRuns[r];
                if (!from.data)
                    continue;
                if (c.fromInput) {
                    stateBuffers[r].Allocate(from.count * size);
                    stateRuns[r] = {stateBuffers[r].Data(), from.count};
                }
                std::memcpy(stateRuns[r].data, from.data, from.count * size);
            }
        }
    }
    for (size_t i = 0; i < published->pools.size(); ++i) {
        const RigExecPublishedPool &from = source.published->pools[i];
        RigExecPublishedPool &pool = published->pools[i];
        const RigExecTypeDesc &t = TypeOf(pool.type);
        RigExecPublishedArena &to = *pool.arenas[pool.current];
        const RigExecPublishedArena &value = *from.arenas[from.current];
        if (t.copy)
            t.copy(to.storage.Data(), value.storage.Data());
        else
            std::memcpy(to.storage.Data(), value.storage.Data(), t.size);
        to.generation = value.generation;
    }
    captures = source.captures;
    generation = source.generation;
    everything = true;
}

} // namespace rigExec
