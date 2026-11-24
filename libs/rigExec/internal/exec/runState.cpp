#include "rigExec/api/runState.h"
#include "rigExec/internal/exec/inputChannel.h"
#include "rigExec/internal/exec/run.h"
#include "rigExec/internal/exec/runStateData.h"
#include "rigExec/internal/exec/view.h"

#include <cassert>
#include <utility>

namespace rigExec {

RigExecRunStateData::RigExecRunStateData(
    std::shared_ptr<const RigExecProgram> p, const RigExecEvaluatorOptions &o)
    : program(std::move(p)), options(o)
{
    channel.Bind(*program);
    rings.assign(program->windowedNodes.size(), RigExecWindowRing());
    for (size_t w = 0; w < program->windowedNodes.size(); ++w)
        rings[w].reserve(program->windowedNodes[w].frames);
    run.Bind(*program, channel, rings);
}

RigExecRunState::RigExecRunState(_CreateKey,
                                 std::unique_ptr<RigExecRunStateData> data)
    : _data(std::move(data))
{
}

RigExecRunState::~RigExecRunState() = default;

void
RigExecRunState::_AssertOwner() const
{
#ifndef NDEBUG
    if (_owner == std::thread::id())
        _owner = std::this_thread::get_id();
    assert(_owner == std::this_thread::get_id());
#endif
}

std::unique_ptr<RigExecRunState>
RigExecRunState::Create(std::shared_ptr<const RigExecProgram> program,
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
    return std::make_unique<RigExecRunState>(
        _CreateKey(0),
        std::make_unique<RigExecRunStateData>(std::move(program), options));
}

RigExecInputWriter
RigExecRunState::BeginInputEdit()
{
    return _data->channel.Begin();
}

bool
RigExecRunState::Evaluate(std::string *error)
{
    _AssertOwner();
    _data->channel.Acquire();
    return _data->run.Run(false, _data->options.tasks, error);
}

const void *
RigExecRunState::GetOutput(RigExecOutputId output) const
{
    _AssertOwner();
    const RigExecProgram &p = *_data->program;
    if (output.index >= p.outputSlots.size())
        return nullptr;
    return _data->run.Value(p.outputSlots[output.index]);
}

RigExecValueView
RigExecRunState::ViewOutput(RigExecOutputId output) const
{
    _AssertOwner();
    return RigExecViewAccess::Output(_data->run, output);
}

std::unique_ptr<RigExecRunState>
RigExecRunState::Clone() const
{
    _AssertOwner();
    return std::make_unique<RigExecRunState>(
        _CreateKey(0),
        RigExecRunStateData::CloneOf(_data->program, _data->options,
                                     _data->channel, _data->rings, _data->run));
}

void
RigExecRunState::ReleaseOwner()
{
    _AssertOwner();
    _owner = std::thread::id();
}

const RigExecProgram &
RigExecRunState::GetProgram() const
{
    return *_data->program;
}

std::unique_ptr<RigExecRunStateData>
RigExecRunStateData::CloneOf(std::shared_ptr<const RigExecProgram> program,
                             const RigExecEvaluatorOptions &options,
                             const RigExecInputChannel &channel,
                             const std::vector<RigExecWindowRing> &rings,
                             const RigExecRun &run)
{
    auto data =
        std::make_unique<RigExecRunStateData>(std::move(program), options);
    data->channel.CopyFrom(channel);
    for (size_t w = 0; w < rings.size(); ++w)
        data->rings[w].assign(rings[w].begin(), rings[w].end());
    data->run.CopyFrom(run);
    return data;
}

} // namespace rigExec
