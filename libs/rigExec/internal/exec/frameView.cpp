#include "rigExec/api/frameView.h"
#include "rigExec/internal/exec/view.h"

#include <utility>

namespace rigExec {

namespace {

const RigExecTypeDesc *
DescOf(const RigExecValueView &view)
{
    return view ? RigExecRegistry::Get().TypeAt(view.Type()->Index()).desc
                : nullptr;
}

} // namespace

size_t
RigExecValueBytes(const RigExecValueView &view)
{
    const RigExecTypeDesc *d = DescOf(view);
    if (!d)
        return 0;
    if (!d->element || !d->count)
        return d->size;
    const RigExecTypeDesc &element =
        *RigExecRegistry::Get().TypeAt(d->element->Index()).desc;
    return d->size + d->count(view.Data()) * element.size;
}

size_t
RigExecValueCount(const RigExecValueView &view)
{
    const RigExecTypeDesc *d = DescOf(view);
    if (!d)
        return 0;
    return d->count ? d->count(view.Data()) : 1;
}

const void *
RigExecValueElements(const RigExecValueView &view)
{
    const RigExecTypeDesc *d = DescOf(view);
    if (!d || !d->element || !d->data)
        return nullptr;
    // data takes a mutable value but only reports where the elements are.
    return d->data(const_cast<void *>(view.Data()));
}

RigExecFrameView::RigExecFrameView(const RigExecFrameView &other)
    : _owner(other._owner), _entries(other._entries), _error(other._error),
      _generation(other._generation), _valid(other._valid)
{
    for (const _Entry &entry : _entries)
        entry.pins->fetch_add(1, std::memory_order_relaxed);
}

RigExecFrameView::RigExecFrameView(RigExecFrameView &&other) noexcept
    : _owner(std::move(other._owner)), _entries(std::move(other._entries)),
      _error(std::move(other._error)),
      _generation(std::exchange(other._generation, 0)),
      _valid(std::exchange(other._valid, false))
{
    other._entries.clear();
}

RigExecFrameView &
RigExecFrameView::operator=(const RigExecFrameView &other)
{
    if (this != &other) {
        RigExecFrameView copy(other);
        *this = std::move(copy);
    }
    return *this;
}

RigExecFrameView &
RigExecFrameView::operator=(RigExecFrameView &&other) noexcept
{
    if (this != &other) {
        Reset();
        _owner = std::move(other._owner);
        _entries = std::move(other._entries);
        other._entries.clear();
        _error = std::move(other._error);
        _generation = std::exchange(other._generation, 0);
        _valid = std::exchange(other._valid, false);
    }
    return *this;
}

RigExecFrameView::~RigExecFrameView()
{
    Reset();
}

void
RigExecFrameView::Reset()
{
    for (const _Entry &entry : _entries)
        entry.pins->fetch_sub(1, std::memory_order_release);
    _entries.clear();
    _error.clear();
    _generation = 0;
    _valid = false;
    _owner.reset();
}

RigExecValueView
RigExecFrameView::View(RigExecOutputId output) const
{
    if (output.index >= _entries.size())
        return RigExecValueView();
    const _Entry &entry = _entries[output.index];
    return RigExecViewAccess::Lend(_owner, entry.pins, entry.data, entry.type,
                                   entry.generation);
}

RigExecValueIdentity
RigExecFrameView::Identity(RigExecOutputId output) const
{
    if (output.index >= _entries.size())
        return RigExecValueIdentity();
    const _Entry &entry = _entries[output.index];
    return RigExecValueIdentity{_owner.get(), entry.pool, entry.arena,
                                entry.generation};
}

RigExecValueView
RigExecViewAccess::Lend(std::shared_ptr<const void> owner,
                        std::atomic<uint32_t> *pins, const void *data,
                        const RigExecTypeHandle *type, uint64_t generation)
{
    RigExecValueView view;
    pins->fetch_add(1, std::memory_order_relaxed);
    view._owner = std::move(owner);
    view._pins = pins;
    view._data = data;
    view._type = type;
    view._generation = generation;
    return view;
}

RigExecFrameView
RigExecViewAccess::Frame(const RigExecRun &run, std::string *error)
{
    RigExecFrameView frame;
    frame._generation = run.generation;
    if (!run.failure.empty()) {
        frame._error = "no valid value: " + run.failure;
        if (error)
            *error = frame._error;
        return frame;
    }
    const RigExecProgram &p = *run.program;
    const RigExecRegistry &registry = RigExecRegistry::Get();
    frame._entries.resize(p.outputSlots.size());
    for (size_t i = 0; i < p.outputSlots.size(); ++i) {
        const RigExecProgramSlot &place = p.slots[p.outputSlots[i]];
        RigExecPublishedPool &pool = run.published->pools[place.arenaIndex];
        RigExecPublishedArena &arena = *pool.arenas[pool.current];
        arena.pins.fetch_add(1, std::memory_order_relaxed);
        RigExecFrameView::_Entry &entry = frame._entries[i];
        entry.pins = &arena.pins;
        entry.arena = &arena;
        entry.data = arena.storage.Data();
        entry.type = registry.TypeAt(place.type).handle;
        entry.generation = arena.generation;
        entry.pool = place.arenaIndex;
    }
    frame._owner = run.published;
    frame._valid = true;
    return frame;
}

} // namespace rigExec
