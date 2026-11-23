#include "rigExec/api/evaluator.h"
#include "rigExec/internal/program/program.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <utility>
#include <vector>

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

struct ResolvedTap {
    uint32_t slot = kRigExecInvalidIndex;
};

/// Slots for \p taps on \p program; false with \p error if one names a
/// node or output the program lacks.
bool
ResolveTaps(const RigExecProgram &program, const RigExecTapList &taps,
            std::vector<ResolvedTap> *resolved, std::string *error)
{
    std::vector<ResolvedTap> out;
    out.reserve(taps.taps.size());
    for (size_t i = 0; i < taps.taps.size(); ++i) {
        const RigExecTap &tap = taps.taps[i];
        uint32_t slot = kRigExecInvalidIndex;
        if (tap.node < program.nodes.size() && tap.output &&
            tap.output->IsValid()) {
            const RigExecProgramNode &node = program.nodes[tap.node];
            const RigExecOpDesc &d = OpOf(node.op);
            const uint32_t p = tap.output->Index();
            if (p < d.portCount && d.ports[p].port == tap.output &&
                (d.ports[p].flags & kRigExecPortOutput))
                slot = program.portSlots[node.firstPort + p];
        }
        if (slot == kRigExecInvalidIndex) {
            if (error)
                *error = "tap " + std::to_string(i) +
                         " names a node or output the program lacks";
            return false;
        }
        out.push_back({slot});
    }
    *resolved = std::move(out);
    return true;
}

/// One run's values, op state and run pointers: the evaluator's own, or a
/// history tick's, which leaves the evaluator's outputs alone.
struct RigExecRunState {
    const RigExecProgram *program = nullptr;
    RigExecAlignedBuffer arena;
    RigExecAlignedBuffer opState;
    /// Per program port: the value an op reads or writes there.
    std::vector<const void *> inputs;
    std::vector<void *> outputs;
    /// Array inputs: element values, and the records inputs point to.
    std::vector<const void *> listItems;
    std::vector<RigExecPortList> lists;
    /// Per window: this run's capture, and the record its node is handed.
    std::vector<RigExecWindowPacketPtr> captures;
    std::vector<RigExecWindowRun> windows;
    /// Per batchNodes entry: its window record, or null.
    std::vector<const RigExecWindowRun *> batchWindows;

    RigExecRunState() = default;
    RigExecRunState(const RigExecRunState &) = delete;
    RigExecRunState &operator=(const RigExecRunState &) = delete;

    ~RigExecRunState()
    {
        if (!program)
            return;
        for (const RigExecProgramSlot &slot : program->slots) {
            const RigExecTypeDesc &t = TypeOf(slot.type);
            if (t.destroy)
                t.destroy(arena.Data() + slot.offset);
        }
        for (const RigExecProgramNode &node : program->nodes) {
            const RigExecOpDesc &d = OpOf(node.op);
            if (node.stateOffset != SIZE_MAX && d.state.destroy)
                d.state.destroy(opState.Data() + node.stateOffset);
        }
    }

    /// \p rings outlives this state and keeps its size.
    void Bind(const RigExecProgram &p,
              const std::vector<RigExecWindowRing> &rings)
    {
        arena.Allocate(p.arenaSize);
        for (const RigExecProgramSlot &slot : p.slots) {
            const RigExecTypeDesc &t = TypeOf(slot.type);
            if (t.construct)
                t.construct(arena.Data() + slot.offset);
        }
        opState.Allocate(p.stateSize);
        for (const RigExecProgramNode &node : p.nodes) {
            const RigExecOpDesc &d = OpOf(node.op);
            if (node.stateOffset != SIZE_MAX && d.state.create)
                d.state.create(opState.Data() + node.stateOffset);
        }
        program = &p;
        inputs.assign(p.portSlots.size(), nullptr);
        outputs.assign(p.portSlots.size(), nullptr);
        for (const RigExecProgramNode &node : p.nodes) {
            const RigExecOpDesc &d = OpOf(node.op);
            for (uint32_t port = 0; port < node.portCount; ++port) {
                const uint32_t slot = p.portSlots[node.firstPort + port];
                if (slot == kRigExecInvalidIndex)
                    continue;
                std::byte *value = arena.Data() + p.slots[slot].offset;
                if (d.ports[port].flags & kRigExecPortOutput)
                    outputs[node.firstPort + port] = value;
                else
                    inputs[node.firstPort + port] = value;
            }
        }
        listItems.resize(p.listSlots.size());
        for (size_t i = 0; i < p.listSlots.size(); ++i)
            listItems[i] = arena.Data() + p.slots[p.listSlots[i]].offset;
        lists.resize(p.lists.size());
        for (size_t i = 0; i < p.lists.size(); ++i) {
            const RigExecProgramList &list = p.lists[i];
            lists[i] = {listItems.data() + list.firstSlot, list.count};
            inputs[list.port] = &lists[i];
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
    }
};

/// Appends \p capture to \p ring and keeps its newest \p frames.
void
PushTick(RigExecWindowRing *ring, RigExecWindowPacketPtr capture, size_t frames)
{
    ring->push_back(std::move(capture));
    if (ring->size() > frames)
        ring->erase(ring->begin(),
                    ring->begin() + std::ptrdiff_t(ring->size() - frames));
}

} // namespace

/// Everything bound to one program: values, op state, window history and
/// run pointers.
struct RigExecEvaluatorState {
    std::shared_ptr<const RigExecProgram> program;
    /// Per window, oldest first; sized once at bind.
    std::vector<RigExecWindowRing> rings;
    RigExecRunState main;
    /// Bound on the first EvaluateHistory.
    std::unique_ptr<RigExecRunState> history;
    /// The run in progress, and the nodes it runs by batchNodes entry (null
    /// runs all).
    RigExecRunState *current = nullptr;
    const char *active = nullptr;
    /// Per batch run: whether it failed and why.
    std::vector<uint8_t> failed;
    std::vector<std::string> errors;
    /// The program's batches by level, as the task system takes them:
    /// owner-thread batches run on the caller, the rest anywhere.
    std::vector<uint32_t> sharedStart;
    std::vector<uint32_t> shared;
    std::vector<uint32_t> callerStart;
    std::vector<uint32_t> caller;
    RigExecTapList tapSet;
    std::vector<ResolvedTap> taps;
    RigExecTapCallback tapCallback = nullptr;
    void *tapContext = nullptr;

    void RunBatch(uint32_t b)
    {
        RigExecRunState &run = *current;
        const RigExecProgramBatch &batch = program->batches[b];
        const RigExecRegisteredOp &op = RigExecRegistry::Get().OpAt(batch.op);
        const RigExecOpDesc &d = *op.desc;
        RigExecOpBatch view;
        view.op = &d;
        view.count = batch.nodeCount;
        view.active = active ? active + batch.firstNode : nullptr;
        if (batch.payloadOffset != SIZE_MAX) {
            view.payloads = program->payloads.Data() + batch.payloadOffset;
            view.payloadStride = batch.payloadStride;
        }
        if (batch.stateOffset != SIZE_MAX) {
            view.states = run.opState.Data() + batch.stateOffset;
            view.stateStride = batch.stateStride;
        }
        view.portCount = d.portCount;
        view.inputs = run.inputs.data() + batch.firstPort;
        view.outputs = run.outputs.data() + batch.firstPort;
        if (d.threading == RigExecOpThreading::Windowed)
            view.windows = run.batchWindows.data() + batch.firstNode;
        size_t at = 0;
        std::string error;
        if (op.runBatch(view, &at, &error))
            return;
        const uint32_t n = program->batchNodes[batch.firstNode + at];
        const uint32_t w = program->windowOfNode[n];
        if (w != kRigExecInvalidIndex)
            run.captures[w].reset();
        failed[b] = 1;
        errors[b] = "node " + std::to_string(n) + " (" +
                    (d.nameSpace ? d.nameSpace : "") + ":" +
                    (d.name ? d.name : "") + ") failed" +
                    (error.empty() ? "" : ": " + error);
    }

    static bool RunTask(void *context, uint32_t b)
    {
        auto *self = static_cast<RigExecEvaluatorState *>(context);
        self->RunBatch(b);
        return !self->failed[b];
    }

    void BuildSchedule()
    {
        const RigExecProgram &p = *program;
        const size_t levelCount =
            p.levelBatches.empty() ? 0 : p.levelBatches.size() - 1;
        sharedStart.assign(1, 0);
        callerStart.assign(1, 0);
        shared.clear();
        caller.clear();
        for (size_t level = 0; level < levelCount; ++level) {
            for (uint32_t b = p.levelBatches[level];
                 b < p.levelBatches[level + 1]; ++b)
                (p.batches[b].ownerThread ? caller : shared).push_back(b);
            sharedStart.push_back(uint32_t(shared.size()));
            callerStart.push_back(uint32_t(caller.size()));
        }
    }

    /// Runs \p run over every node, or on a history tick over the nodes the
    /// windowed nodes depend on. A failed run leaves every capture null.
    bool Run(RigExecRunState &run, bool historyTick, RigExecTaskSystem *tasks,
             std::string *error)
    {
        const RigExecProgram &p = *program;
        for (size_t w = 0; w < run.windows.size(); ++w) {
            run.captures[w].reset();
            run.windows[w].captureOnly =
                historyTick && !p.windowedNodes[w].feedsWindow;
        }
        current = &run;
        active = historyTick ? p.batchHistoryMask.data() : nullptr;
        std::fill(failed.begin(), failed.end(), uint8_t(0));
        RigExecTaskSchedule schedule;
        schedule.levelCount = sharedStart.size() - 1;
        schedule.sharedStart = sharedStart.data();
        schedule.shared = shared.data();
        schedule.callerStart = callerStart.data();
        schedule.caller = caller.data();
        schedule.run = &RigExecEvaluatorState::RunTask;
        schedule.context = this;
        if (tasks ? tasks->Run(schedule)
                  : RigExecRunTaskScheduleInline(schedule))
            return true;
        // The first failure in batch order, whichever thread hit it.
        for (size_t b = 0; b < p.batches.size(); ++b) {
            if (failed[b]) {
                if (error)
                    *error = errors[b];
                break;
            }
        }
        for (RigExecWindowPacketPtr &capture : run.captures)
            capture.reset();
        return false;
    }
};

RigExecEvaluator::RigExecEvaluator(
    _CreateKey, std::shared_ptr<const RigExecProgram> program,
    const RigExecEvaluatorOptions &options)
    : _options(options)
{
    _Bind(std::move(program), nullptr);
}

RigExecEvaluator::~RigExecEvaluator() = default;

void
RigExecEvaluator::_AssertOwnerThread() const
{
    assert(std::this_thread::get_id() == _ownerThread);
}

std::unique_ptr<RigExecEvaluator>
RigExecEvaluator::Create(std::shared_ptr<const RigExecProgram> program,
                         const RigExecEvaluatorOptions &options,
                         std::string *error)
{
    if (!program) {
        if (error)
            *error = "no program";
        return nullptr;
    }
    if (!RigExecRegistry::IsInitialized() ||
        program->registryFingerprint != RigExecRegistry::Get().Fingerprint()) {
        if (error)
            *error = "the program was compiled against another registry";
        return nullptr;
    }
    return std::make_unique<RigExecEvaluator>(_CreateKey(0), std::move(program),
                                              options);
}

bool
RigExecEvaluator::_Bind(std::shared_ptr<const RigExecProgram> program,
                        std::string *error)
{
    auto state = std::make_unique<RigExecEvaluatorState>();
    if (_state && !_state->tapSet.taps.empty()) {
        if (!ResolveTaps(*program, _state->tapSet, &state->taps, error))
            return false;
        state->tapSet = _state->tapSet;
        state->tapCallback = _state->tapCallback;
        state->tapContext = _state->tapContext;
    }
    const RigExecProgram &p = *program;
    state->rings.assign(p.windowedNodes.size(), RigExecWindowRing());
    state->main.Bind(p, state->rings);
    state->failed.assign(p.batches.size(), 0);
    state->errors.assign(p.batches.size(), std::string());
    state->program = std::move(program);
    state->BuildSchedule();
    _state = std::move(state);
    return true;
}

bool
RigExecEvaluator::SetInput(RigExecInputId input, const void *value,
                           std::string *error)
{
    _AssertOwnerThread();
    const RigExecProgram &p = *_state->program;
    if (input.index >= p.inputSlots.size() || !value) {
        if (error)
            *error = "SetInput: no such input";
        return false;
    }
    const RigExecProgramSlot &slot = p.slots[p.inputSlots[input.index]];
    const RigExecTypeDesc &t = TypeOf(slot.type);
    std::byte *dst = _state->main.arena.Data() + slot.offset;
    if (t.copy)
        t.copy(dst, value);
    else
        std::memcpy(dst, value, t.size);
    return true;
}

bool
RigExecEvaluator::Evaluate(std::string *error)
{
    _AssertOwnerThread();
    RigExecEvaluatorState &s = *_state;
    if (!s.Run(s.main, false, _options.tasks, error))
        return false;
    if (s.tapCallback) {
        const RigExecProgram &p = *s.program;
        for (size_t i = 0; i < s.taps.size(); ++i) {
            const RigExecProgramSlot &slot = p.slots[s.taps[i].slot];
            s.tapCallback(s.tapContext, i, s.main.arena.Data() + slot.offset,
                          slot.type);
        }
    }
    return true;
}

const void *
RigExecEvaluator::GetOutput(RigExecOutputId output) const
{
    const RigExecProgram &p = *_state->program;
    if (output.index >= p.outputSlots.size())
        return nullptr;
    return _state->main.arena.Data() +
           p.slots[p.outputSlots[output.index]].offset;
}

bool
RigExecEvaluator::SwapProgram(std::shared_ptr<const RigExecProgram> program,
                              std::string *error)
{
    _AssertOwnerThread();
    if (!program ||
        program->registryFingerprint != RigExecRegistry::Get().Fingerprint()) {
        if (error)
            *error = "SwapProgram: the program was compiled against another "
                     "registry";
        return false;
    }
    return _Bind(std::move(program), error);
}

bool
RigExecEvaluator::SetTaps(const RigExecTapList &taps,
                          RigExecTapCallback callback, void *context,
                          std::string *error)
{
    _AssertOwnerThread();
    std::vector<ResolvedTap> resolved;
    if (!ResolveTaps(*_state->program, taps, &resolved, error))
        return false;
    _state->tapSet = taps;
    _state->taps = std::move(resolved);
    _state->tapCallback = taps.taps.empty() ? nullptr : callback;
    _state->tapContext = context;
    return true;
}

void
RigExecEvaluator::PushHistory()
{
    _AssertOwnerThread();
    RigExecEvaluatorState &s = *_state;
    for (size_t w = 0; w < s.rings.size(); ++w)
        PushTick(&s.rings[w], s.main.captures[w],
                 s.program->windowedNodes[w].frames);
}

void
RigExecEvaluator::ClearHistory()
{
    _AssertOwnerThread();
    for (RigExecWindowRing &ring : _state->rings)
        ring.clear();
}

bool
RigExecEvaluator::EvaluateHistory(std::string *error)
{
    _AssertOwnerThread();
    RigExecEvaluatorState &s = *_state;
    const RigExecProgram &p = *s.program;
    if (p.windowedNodes.empty())
        return true;
    if (!s.history) {
        s.history = std::make_unique<RigExecRunState>();
        s.history->Bind(p, s.rings);
    }
    for (const uint32_t input : p.inputSlots) {
        const RigExecProgramSlot &slot = p.slots[input];
        const RigExecTypeDesc &t = TypeOf(slot.type);
        std::byte *dst = s.history->arena.Data() + slot.offset;
        const std::byte *src = s.main.arena.Data() + slot.offset;
        if (t.copy)
            t.copy(dst, src);
        else
            std::memcpy(dst, src, t.size);
    }
    const bool ran = s.Run(*s.history, true, _options.tasks, error);
    for (size_t w = 0; w < s.rings.size(); ++w)
        PushTick(&s.rings[w], s.history->captures[w],
                 p.windowedNodes[w].frames);
    return ran;
}

size_t
RigExecEvaluator::GetHistoryDepth() const
{
    std::vector<uint32_t> frames;
    std::vector<std::vector<uint32_t>> inner;
    for (const RigExecProgramWindow &window : _state->program->windowedNodes) {
        frames.push_back(window.frames);
        inner.push_back(window.inner);
    }
    return RigExecWindowHistoryDepth(frames, inner);
}

size_t
RigExecEvaluator::GetWindowCount() const
{
    return _state->rings.size();
}

bool
RigExecEvaluator::SetWindowRings(const std::vector<RigExecWindowRing> &rings,
                                 std::string *error)
{
    _AssertOwnerThread();
    RigExecEvaluatorState &s = *_state;
    if (rings.size() != s.rings.size()) {
        if (error)
            *error = "SetWindowRings: " + std::to_string(rings.size()) +
                     " rings for " + std::to_string(s.rings.size()) +
                     " windows";
        return false;
    }
    for (size_t w = 0; w < rings.size(); ++w) {
        const size_t frames = s.program->windowedNodes[w].frames;
        const size_t skip =
            rings[w].size() > frames ? rings[w].size() - frames : 0;
        s.rings[w].assign(rings[w].begin() + std::ptrdiff_t(skip),
                          rings[w].end());
    }
    return true;
}

const std::vector<RigExecWindowRing> &
RigExecEvaluator::GetWindowRings() const
{
    return _state->rings;
}

std::vector<RigExecWindowPacketPtr>
RigExecEvaluator::GetWindowCaptures() const
{
    return _state->main.captures;
}

} // namespace rigExec
