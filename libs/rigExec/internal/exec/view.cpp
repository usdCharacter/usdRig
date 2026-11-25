#include "rigExec/internal/exec/view.h"

#include <utility>

namespace rigExec {

namespace {

void
Retain(std::atomic<uint32_t> *pins)
{
    if (pins)
        pins->fetch_add(1, std::memory_order_relaxed);
}

/// Fills \p ref for slot \p slot of \p program.
void
Describe(const RigExecProgram &program, uint32_t slot, RigExecValueRef *ref)
{
    const RigExecProgramSlot &place = program.slots[slot];
    switch (place.arena) {
    case RigExecArenaKind::Constant:
        ref->arena = RigExecArena::Constant;
        break;
    case RigExecArenaKind::Input:
        ref->arena = RigExecArena::Input;
        break;
    case RigExecArenaKind::Work:
        ref->arena = RigExecArena::Work;
        break;
    case RigExecArenaKind::Published:
        ref->arena = RigExecArena::Published;
        break;
    }
    ref->index = place.arenaIndex;
    ref->offset = place.offset;
    ref->count = 1;
    ref->type = RigExecRegistry::Get().TypeAt(place.type).handle;
}

} // namespace

RigExecValueView::RigExecValueView(const RigExecValueView &other)
    : _owner(other._owner), _pins(other._pins), _data(other._data),
      _type(other._type), _generation(other._generation)
{
    Retain(_pins);
}

RigExecValueView::RigExecValueView(RigExecValueView &&other) noexcept
    : _owner(std::move(other._owner)),
      _pins(std::exchange(other._pins, nullptr)),
      _data(std::exchange(other._data, nullptr)),
      _type(std::exchange(other._type, nullptr)),
      _generation(std::exchange(other._generation, 0))
{
}

RigExecValueView &
RigExecValueView::operator=(const RigExecValueView &other)
{
    if (this != &other) {
        RigExecValueView copy(other);
        *this = std::move(copy);
    }
    return *this;
}

RigExecValueView &
RigExecValueView::operator=(RigExecValueView &&other) noexcept
{
    if (this != &other) {
        Reset();
        _owner = std::move(other._owner);
        _pins = std::exchange(other._pins, nullptr);
        _data = std::exchange(other._data, nullptr);
        _type = std::exchange(other._type, nullptr);
        _generation = std::exchange(other._generation, 0);
    }
    return *this;
}

RigExecValueView::~RigExecValueView()
{
    Reset();
}

void
RigExecValueView::Reset()
{
    if (_pins)
        _pins->fetch_sub(1, std::memory_order_release);
    _pins = nullptr;
    _data = nullptr;
    _type = nullptr;
    _generation = 0;
    _owner.reset();
}

RigExecValueView
RigExecViewAccess::Output(const RigExecRun &run, RigExecOutputId output,
                          std::string *error)
{
    RigExecValueView view;
    const RigExecProgram &p = *run.program;
    if (output.index >= p.outputSlots.size() ||
        !run.OutputValid(p.outputSlots[output.index], error))
        return view;
    const RigExecProgramSlot &place = p.slots[p.outputSlots[output.index]];
    RigExecPublishedPool &pool = run.published->pools[place.arenaIndex];
    RigExecPublishedArena &arena = *pool.arenas[pool.current];
    arena.pins.fetch_add(1, std::memory_order_relaxed);
    view._owner = run.published;
    view._pins = &arena.pins;
    view._data = arena.storage.Data();
    view._type = RigExecRegistry::Get().TypeAt(place.type).handle;
    view._generation = arena.generation;
    return view;
}

bool
RigExecResolveInput(const RigExecProgram &program, RigExecInputId input,
                    RigExecValueRef *ref)
{
    if (input.index >= program.inputSlots.size() || !ref)
        return false;
    Describe(program, program.inputSlots[input.index], ref);
    return true;
}

bool
RigExecResolveOutput(const RigExecProgram &program, RigExecOutputId output,
                     RigExecValueRef *ref)
{
    if (output.index >= program.outputSlots.size() || !ref)
        return false;
    Describe(program, program.outputSlots[output.index], ref);
    return true;
}

bool
RigExecResolveNodeOutput(const RigExecProgram &program, RigExecNodeId node,
                         const RigExecPortHandle &out, RigExecValueRef *ref)
{
    if (node >= program.nodes.size() || !out.IsValid() || !ref)
        return false;
    const RigExecProgramNode &n = program.nodes[node];
    const RigExecOpDesc &d = *RigExecRegistry::Get().OpAt(n.op).desc;
    const uint32_t p = out.Index();
    if (p >= d.portCount || d.ports[p].port != &out ||
        !(d.ports[p].flags & kRigExecPortOutput))
        return false;
    Describe(program, program.portSlots[n.firstPort + p], ref);
    return true;
}

} // namespace rigExec
