#include "rigExec/api/runState.h"
#include "rigExec/internal/exec/inputChannel.h"
#include "rigExec/internal/exec/run.h"
#include "rigExec/internal/exec/runStateData.h"
#include "rigExec/internal/exec/view.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
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

void
RigExecRunState::_AssertUnownedOrOwner() const
{
#ifndef NDEBUG
    assert(_owner == std::thread::id() || _owner == std::this_thread::get_id());
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

RigExecFrameView
RigExecRunState::ViewFrame(std::string *error) const
{
    _AssertOwner();
    return RigExecViewAccess::Frame(_data->run, error);
}

std::unique_ptr<RigExecRunState>
RigExecRunState::Clone() const
{
    _AssertUnownedOrOwner();
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
RigExecRunState::PointInputGroups(RigExecInputId input,
                                  const RigExecOpGroups *groups,
                                  std::string *error)
{
    if (input.index >= _data->program->inputSlots.size()) {
        if (error)
            *error = "no input " + std::to_string(input.index);
        return false;
    }
    return _data->run.PointInputGroups(input.index, groups, error);
}

bool
RigExecRunState::PointInputHaloGroups(RigExecInputId input,
                                      const RigExecOpGroups *groups,
                                      std::string *error)
{
    if (input.index >= _data->program->inputSlots.size()) {
        if (error)
            *error = "no input " + std::to_string(input.index);
        return false;
    }
    return _data->run.PointInputHaloGroups(input.index, groups, error);
}

bool
RigExecRunState::RunChainHead(size_t chain, const int *bounds, uint32_t groups,
                              std::string *error, RigExecNodeId *failed)
{
    return _data->run.RunChainHead(uint32_t(chain), bounds, groups, error,
                                   failed);
}

bool
RigExecRunState::RunChainRange(size_t chain, uint32_t segment, uint32_t group,
                               std::string *error, RigExecNodeId *failed)
{
    const RigExecProgram &p = *_data->program;
    if (chain >= p.chains.size() || segment >= p.chains[chain].memberCount ||
        p.chainSegment[p.chains[chain].firstMember + segment] != segment) {
        if (error)
            *error = "chain " + std::to_string(chain) + " has no segment " +
                     std::to_string(segment);
        return false;
    }
    return _data->run.RunChainRange(uint32_t(chain), segment, group, error,
                                    failed);
}

void
RigExecRunState::ChainReads(size_t chain, uint32_t member, uint32_t group,
                            size_t *lo, size_t *hi) const
{
    _data->run.ChainReads(uint32_t(chain), member, group, lo, hi);
}

size_t
RigExecRunState::ChainElements(size_t chain) const
{
    return _data->run.ChainElements(uint32_t(chain));
}

const void *
RigExecRunState::GetValue(const RigExecValueRef &ref) const
{
    if (ref.value >= _data->program->slots.size() ||
        _data->program->groupedSlots[ref.value])
        return nullptr;
    return _data->run.Value(ref.value);
}

const void *
RigExecRunState::GetGroupValue(const RigExecValueRef &ref, uint32_t group,
                               size_t *count) const
{
    const RigExecProgram &p = *_data->program;
    const RigExecRun &run = _data->run;
    *count = 0;
    if (ref.value >= p.slots.size())
        return nullptr;
    const uint32_t node = p.producer[ref.value];
    if (node == kRigExecInvalidIndex)
        return nullptr;
    const uint32_t chain = p.positionChain[p.batchPosition[node]];
    if (chain == kRigExecInvalidIndex || chain >= run.chainBounds.size())
        return nullptr;
    const std::vector<int> &cut = run.chainBounds[chain];
    if (size_t(group) + 1 >= cut.size())
        return nullptr;
    const size_t elements = size_t(cut[group + 1] - cut[group]);
    if (p.groupedSlots[ref.value]) {
        const std::vector<RigExecAlignedBuffer> &store =
            run.groupStore[ref.value];
        if (group >= store.size())
            return nullptr;
        *count = elements;
        return store[group].Data();
    }
    const RigExecTypeDesc &type =
        *RigExecRegistry::Get().TypeAt(p.slots[ref.value].type).desc;
    void *value = const_cast<void *>(run.Value(ref.value));
    if (!value || !type.data || !type.element)
        return nullptr;
    const size_t size =
        RigExecRegistry::Get().TypeAt(type.element->Index()).desc->size;
    *count = elements;
    return static_cast<const std::byte *>(type.data(value)) +
           size_t(cut[group]) * size;
}

const void *
RigExecRunState::GetRangeScratch(RigExecNodeId node, uint32_t group) const
{
    const RigExecProgram &p = *_data->program;
    const RigExecRun &run = _data->run;
    if (node >= p.batchPosition.size())
        return nullptr;
    const uint32_t at = p.batchPosition[node];
    if (at >= run.scratch.size() || group >= run.scratch[at].size())
        return nullptr;
    return run.scratch[at][group].Data();
}

size_t
RigExecProgramNodeCount(const RigExecProgram &program)
{
    return program.nodes.size();
}

bool
RigExecProgramNodeAt(const RigExecProgram &program, size_t node,
                     RigExecNodeDesc *desc)
{
    if (node >= program.nodes.size() || !desc)
        return false;
    const RigExecProgramNode &n = program.nodes[node];
    *desc = RigExecNodeDesc();
    desc->op = RigExecRegistry::Get().OpAt(n.op).desc;
    desc->values.assign(n.portCount, kRigExecInvalidIndex);
    desc->producers.assign(n.portCount, kRigExecInvalidIndex);
    desc->inputs.assign(n.portCount, kRigExecInvalidIndex);
    desc->lists.assign(n.portCount, std::vector<uint32_t>());
    for (uint32_t p = 0; p < n.portCount; ++p) {
        const uint32_t slot = program.portSlots[n.firstPort + p];
        desc->values[p] = slot;
        if (slot == kRigExecInvalidIndex ||
            !(desc->op->ports[p].flags & kRigExecPortInput))
            continue;
        desc->producers[p] = program.producer[slot];
        // inputSlots ascend: BindInputs gives each host input a new slot.
        const auto input = std::lower_bound(program.inputSlots.begin(),
                                            program.inputSlots.end(), slot);
        if (input != program.inputSlots.end() && *input == slot)
            desc->inputs[p] = uint32_t(input - program.inputSlots.begin());
    }
    // lists are in node then port order (CheckInputsAndLists).
    for (auto list = std::lower_bound(
             program.lists.begin(), program.lists.end(), n.firstPort,
             [](const RigExecProgramList &l, uint32_t port) {
                 return l.port < port;
             });
         list != program.lists.end() && list->port < n.firstPort + n.portCount;
         ++list)
        desc->lists[list->port - n.firstPort].assign(
            program.listSlots.begin() + list->firstSlot,
            program.listSlots.begin() + list->firstSlot + list->count);
    return true;
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
        char grouped = 0;
        for (uint32_t o = program.nodeOutputStart[program.batchNodes[at]];
             o < program.nodeOutputStart[program.batchNodes[at] + 1]; ++o)
            grouped |= program.groupedSlots[program.nodeOutputs[o]];
        desc->grouped.push_back(grouped);
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
