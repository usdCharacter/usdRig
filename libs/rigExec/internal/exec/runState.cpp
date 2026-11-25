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
RigExecRunState::GetOutput(RigExecOutputId output, std::string *error) const
{
    _AssertOwner();
    const RigExecProgram &p = *_data->program;
    if (output.index >= p.outputSlots.size() ||
        !_data->run.OutputValid(p.outputSlots[output.index], error))
        return nullptr;
    return _data->run.Value(p.outputSlots[output.index]);
}

RigExecValueView
RigExecRunState::ViewOutput(RigExecOutputId output, std::string *error) const
{
    _AssertOwner();
    return RigExecViewAccess::Output(_data->run, output, error);
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

bool
RigExecRunState::PointInput(RigExecInputId input, const void *value)
{
    if (input.index >= _data->program->inputSlots.size())
        return false;
    _data->run.PointInput(input.index, value);
    return true;
}

bool
RigExecRunState::RunNode(RigExecNodeId node, std::string *error)
{
    return _data->run.RunNode(node, error);
}

bool
RigExecRunState::RunChainHead(size_t chain, uint32_t groups, std::string *error)
{
    return _data->run.RunChainHead(uint32_t(chain), groups, error);
}

bool
RigExecRunState::RunChainRange(size_t chain, uint32_t segment, size_t begin,
                               size_t end, uint32_t group, std::string *error)
{
    const RigExecProgram &p = *_data->program;
    if (chain >= p.chains.size() || segment >= p.chains[chain].memberCount ||
        p.chainSegment[p.chains[chain].firstMember + segment] != segment) {
        if (error)
            *error = "chain " + std::to_string(chain) + " has no segment " +
                     std::to_string(segment);
        return false;
    }
    return _data->run.RunChainRange(uint32_t(chain), segment, begin, end, group,
                                    error);
}

void
RigExecRunState::ChainReads(size_t chain, uint32_t member, size_t begin,
                            size_t end, uint32_t group, size_t *lo,
                            size_t *hi) const
{
    _data->run.ChainReads(uint32_t(chain), member, begin, end, group, lo, hi);
}

size_t
RigExecRunState::ChainElements(size_t chain) const
{
    return _data->run.ChainElements(uint32_t(chain));
}

const void *
RigExecRunState::GetValue(const RigExecValueRef &ref) const
{
    if (ref.value >= _data->program->slots.size())
        return nullptr;
    return _data->run.Value(ref.value);
}

size_t
RigExecProgramChainCount(const RigExecProgram &program)
{
    return program.chains.size();
}

bool
RigExecProgramChainAt(const RigExecProgram &program, size_t chain,
                      RigExecChainDesc *desc)
{
    if (chain >= program.chains.size() || !desc)
        return false;
    const RigExecProgramChain &c = program.chains[chain];
    *desc = RigExecChainDesc();
    for (uint32_t m = 0; m < c.memberCount; ++m) {
        const uint32_t g = c.firstMember + m;
        const uint32_t at = program.chainMembers[g];
        desc->members.push_back(program.batchNodes[at]);
        desc->segments.push_back(program.chainSegment[g]);
        desc->whole.push_back(program.wholePositions[at]);
        desc->feeds.emplace_back();
        for (uint32_t f = program.chainFeedStart[g];
             f < program.chainFeedStart[g + 1]; ++f)
            desc->feeds.back().push_back({program.chainFeeds[f].member,
                                          program.chainFeeds[f].neighbourhood});
    }
    return true;
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
