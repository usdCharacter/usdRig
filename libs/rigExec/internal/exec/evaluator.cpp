#include "rigExec/api/evaluator.h"
#include "rigExec/internal/exec/inputChannel.h"
#include "rigExec/internal/exec/run.h"
#include "rigExec/internal/exec/runStateData.h"
#include "rigExec/internal/exec/view.h"
#include "rigExec/internal/program/program.h"

#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

struct ResolvedTap {
    uint32_t slot = kRigExecInvalidIndex;
    const RigExecTypeHandle *type = nullptr;
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
        RigExecValueRef ref;
        if (!tap.output ||
            !RigExecResolveNodeOutput(program, tap.node, *tap.output, &ref)) {
            if (error)
                *error = "tap " + std::to_string(i) +
                         " names a node or output the program lacks";
            return false;
        }
        const RigExecProgramNode &node = program.nodes[tap.node];
        out.push_back({program.portSlots[node.firstPort + tap.output->Index()],
                       ref.type});
    }
    *resolved = std::move(out);
    return true;
}

/// Appends \p capture to \p ring and keeps its newest \p frames.
void
PushTick(RigExecWindowRing *ring, RigExecWindowPacketPtr capture, size_t frames)
{
    if (!frames) {
        ring->clear();
        return;
    }
    if (ring->size() >= frames)
        ring->erase(ring->begin(),
                    ring->begin() + std::ptrdiff_t(ring->size() + 1 - frames));
    ring->push_back(std::move(capture));
}

} // namespace

/// Everything bound to one program: the input channel, the main run, the
/// history run, window history and taps.
struct RigExecEvaluatorState {
    std::shared_ptr<const RigExecProgram> program;
    RigExecInputChannel channel;
    /// Per window, oldest first; sized once at bind.
    std::vector<RigExecWindowRing> rings;
    RigExecRun main;
    /// Bound on the first EvaluateHistory.
    std::unique_ptr<RigExecRun> history;
    RigExecTapList tapSet;
    std::vector<ResolvedTap> taps;
    RigExecTapCallback tapCallback = nullptr;
    void *tapContext = nullptr;
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
    state->channel.Bind(p);
    state->rings.assign(p.windowedNodes.size(), RigExecWindowRing());
    for (size_t w = 0; w < p.windowedNodes.size(); ++w)
        state->rings[w].reserve(p.windowedNodes[w].frames);
    state->main.Bind(p, state->channel, state->rings);
    state->program = std::move(program);
    _pending = RigExecInputWriter();
    _state = std::move(state);
    return true;
}

RigExecInputWriter
RigExecEvaluator::BeginInputEdit()
{
    return _state->channel.Begin();
}

bool
RigExecEvaluator::SetInput(RigExecInputId input, const void *value,
                           std::string *error)
{
    _AssertOwnerThread();
    if (!_pending.IsOpen())
        _pending = BeginInputEdit();
    return _pending.Set(input, value, error);
}

bool
RigExecEvaluator::Evaluate(std::string *error)
{
    _AssertOwnerThread();
    RigExecEvaluatorState &s = *_state;
    _pending.Commit();
    s.channel.Acquire();
    if (!s.main.Run(false, _options.tasks, error))
        return false;
    if (s.tapCallback) {
        for (size_t i = 0; i < s.taps.size(); ++i) {
            RigExecTapValue value;
            value.data = s.main.Value(s.taps[i].slot);
            value.type = s.taps[i].type;
            s.tapCallback(s.tapContext, i, value);
        }
    }
    return true;
}

const void *
RigExecEvaluator::GetOutput(RigExecOutputId output, std::string *error) const
{
    const RigExecProgram &p = *_state->program;
    if (output.index >= p.outputSlots.size() ||
        !_state->main.OutputValid(p.outputSlots[output.index], error))
        return nullptr;
    return _state->main.Value(p.outputSlots[output.index]);
}

RigExecValueView
RigExecEvaluator::ViewOutput(RigExecOutputId output, std::string *error) const
{
    _AssertOwnerThread();
    return RigExecViewAccess::Output(_state->main, output, error);
}

std::unique_ptr<RigExecRunState>
RigExecEvaluator::CreateRunState()
{
    _AssertOwnerThread();
    _pending.Commit();
    RigExecEvaluatorState &s = *_state;
    return std::make_unique<RigExecRunState>(
        RigExecRunState::_CreateKey(0),
        RigExecRunStateData::CloneOf(s.program, _options, s.channel, s.rings,
                                     s.main));
}

const RigExecProgram &
RigExecEvaluator::GetProgram() const
{
    return *_state->program;
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
        s.history = std::make_unique<RigExecRun>();
        s.history->Bind(p, s.channel, s.rings);
    }
    _pending.Commit();
    s.channel.Acquire();
    const bool ran = s.history->Run(true, _options.tasks, error);
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
