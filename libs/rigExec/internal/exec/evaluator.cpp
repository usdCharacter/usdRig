#include "rigExec/api/evaluator.h"
#include "rigExec/internal/program/program.h"

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

} // namespace

/// Everything bound to one program: values, op state and run pointers.
struct RigExecEvaluatorState {
    std::shared_ptr<const RigExecProgram> program;
    RigExecAlignedBuffer arena;
    RigExecAlignedBuffer opState;
    /// Per program port: the value an op reads or writes there.
    std::vector<const void *> inputs;
    std::vector<void *> outputs;
    /// Array inputs: element values, and the records inputs point to.
    std::vector<const void *> listItems;
    std::vector<RigExecPortList> lists;
    /// Per batch run: whether it failed and why.
    std::vector<uint8_t> failed;
    std::vector<std::string> errors;
    /// Pure batches of the level being run.
    std::vector<uint32_t> parallel;
    RigExecTapList tapSet;
    std::vector<ResolvedTap> taps;
    RigExecTapCallback tapCallback = nullptr;
    void *tapContext = nullptr;

    ~RigExecEvaluatorState()
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

    void RunBatch(uint32_t b)
    {
        const RigExecProgramBatch &batch = program->batches[b];
        const RigExecOpDesc &d = OpOf(batch.op);
        for (uint32_t i = 0; i < batch.nodeCount; ++i) {
            const uint32_t n = program->batchNodes[batch.firstNode + i];
            const RigExecProgramNode &node = program->nodes[n];
            RigExecOpIo io;
            io.inputs = inputs.data() + node.firstPort;
            io.outputs = outputs.data() + node.firstPort;
            io.portCount = node.portCount;
            const void *payload =
                node.payloadOffset == SIZE_MAX
                    ? nullptr
                    : program->payloads.Data() + node.payloadOffset;
            void *state = node.stateOffset == SIZE_MAX
                              ? nullptr
                              : opState.Data() + node.stateOffset;
            std::string error;
            if (!d.run(payload, state, io, &error)) {
                failed[b] = 1;
                errors[b] = "node " + std::to_string(n) + " (" +
                            (d.nameSpace ? d.nameSpace : "") + ":" +
                            (d.name ? d.name : "") + ") failed" +
                            (error.empty() ? "" : ": " + error);
                return;
            }
        }
    }

    static void RunParallel(void *context, size_t i)
    {
        auto *self = static_cast<RigExecEvaluatorState *>(context);
        self->RunBatch(self->parallel[i]);
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
    state->arena.Allocate(p.arenaSize);
    for (const RigExecProgramSlot &slot : p.slots) {
        const RigExecTypeDesc &t = TypeOf(slot.type);
        if (t.construct)
            t.construct(state->arena.Data() + slot.offset);
    }
    state->opState.Allocate(p.stateSize);
    for (const RigExecProgramNode &node : p.nodes) {
        const RigExecOpDesc &d = OpOf(node.op);
        if (node.stateOffset != SIZE_MAX && d.state.create)
            d.state.create(state->opState.Data() + node.stateOffset);
    }
    state->inputs.assign(p.portSlots.size(), nullptr);
    state->outputs.assign(p.portSlots.size(), nullptr);
    for (const RigExecProgramNode &node : p.nodes) {
        const RigExecOpDesc &d = OpOf(node.op);
        for (uint32_t port = 0; port < node.portCount; ++port) {
            const uint32_t slot = p.portSlots[node.firstPort + port];
            if (slot == kRigExecInvalidIndex)
                continue;
            std::byte *value = state->arena.Data() + p.slots[slot].offset;
            if (d.ports[port].flags & kRigExecPortOutput)
                state->outputs[node.firstPort + port] = value;
            else
                state->inputs[node.firstPort + port] = value;
        }
    }
    state->listItems.resize(p.listSlots.size());
    for (size_t i = 0; i < p.listSlots.size(); ++i)
        state->listItems[i] =
            state->arena.Data() + p.slots[p.listSlots[i]].offset;
    state->lists.resize(p.lists.size());
    for (size_t i = 0; i < p.lists.size(); ++i) {
        const RigExecProgramList &list = p.lists[i];
        state->lists[i] = {state->listItems.data() + list.firstSlot,
                           list.count};
        state->inputs[list.port] = &state->lists[i];
    }
    state->failed.assign(p.batches.size(), 0);
    state->errors.assign(p.batches.size(), std::string());
    state->program = std::move(program);
    _state = std::move(state);
    return true;
}

bool
RigExecEvaluator::SetInput(RigExecInputId input, const void *value,
                           std::string *error)
{
    const RigExecProgram &p = *_state->program;
    if (input.index >= p.inputSlots.size() || !value) {
        if (error)
            *error = "SetInput: no such input";
        return false;
    }
    const RigExecProgramSlot &slot = p.slots[p.inputSlots[input.index]];
    const RigExecTypeDesc &t = TypeOf(slot.type);
    std::byte *dst = _state->arena.Data() + slot.offset;
    if (t.copy)
        t.copy(dst, value);
    else
        std::memcpy(dst, value, t.size);
    return true;
}

bool
RigExecEvaluator::Evaluate(std::string *error)
{
    RigExecEvaluatorState &s = *_state;
    const RigExecProgram &p = *s.program;
    const size_t levelCount =
        p.levelBatches.empty() ? 0 : p.levelBatches.size() - 1;
    for (size_t level = 0; level < levelCount; ++level) {
        const uint32_t first = p.levelBatches[level];
        const uint32_t last = p.levelBatches[level + 1];
        s.parallel.clear();
        for (uint32_t b = first; b < last; ++b) {
            s.failed[b] = 0;
            if (p.batches[b].ownerThread || !_options.tasks)
                s.RunBatch(b);
            else
                s.parallel.push_back(b);
        }
        if (s.parallel.size() == 1)
            s.RunBatch(s.parallel.front());
        else if (!s.parallel.empty())
            _options.tasks->ParallelFor(
                s.parallel.size(), &RigExecEvaluatorState::RunParallel, &s);
        // The first failure in batch order, whichever thread hit it.
        for (uint32_t b = first; b < last; ++b) {
            if (s.failed[b]) {
                if (error)
                    *error = s.errors[b];
                return false;
            }
        }
    }
    if (s.tapCallback) {
        for (size_t i = 0; i < s.taps.size(); ++i) {
            const RigExecProgramSlot &slot = p.slots[s.taps[i].slot];
            s.tapCallback(s.tapContext, i, s.arena.Data() + slot.offset,
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
    return _state->arena.Data() + p.slots[p.outputSlots[output.index]].offset;
}

bool
RigExecEvaluator::SwapProgram(std::shared_ptr<const RigExecProgram> program,
                              std::string *error)
{
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
    std::vector<ResolvedTap> resolved;
    if (!ResolveTaps(*_state->program, taps, &resolved, error))
        return false;
    _state->tapSet = taps;
    _state->taps = std::move(resolved);
    _state->tapCallback = taps.taps.empty() ? nullptr : callback;
    _state->tapContext = context;
    return true;
}

} // namespace rigExec
