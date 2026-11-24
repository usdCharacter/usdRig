#include "rigExec/internal/exec/inputChannel.h"

#include <cstring>
#include <utility>

namespace rigExec {

namespace {

const RigExecTypeDesc &
TypeOf(uint32_t type)
{
    return *RigExecRegistry::Get().TypeAt(type).desc;
}

} // namespace

RigExecInputChannel::~RigExecInputChannel()
{
    if (!program)
        return;
    for (uint32_t copy = 0; copy < 3; ++copy)
        for (uint32_t i = 0; i < program->inputSlots.size(); ++i) {
            const RigExecTypeDesc &t =
                TypeOf(program->slots[program->inputSlots[i]].type);
            if (t.destroy)
                t.destroy(_Value(copy, i));
        }
}

std::byte *
RigExecInputChannel::_Value(uint32_t copy, uint32_t input) const
{
    return buffers[copy].Data() +
           program->slots[program->inputSlots[input]].offset;
}

void
RigExecInputChannel::Bind(const RigExecProgram &p)
{
    program = &p;
    const size_t inputs = p.inputSlots.size();
    for (uint32_t copy = 0; copy < 3; ++copy) {
        buffers[copy].Allocate(p.inputArenaSize);
        versions[copy].assign(inputs, 0);
        for (uint32_t i = 0; i < inputs; ++i) {
            const RigExecTypeDesc &t = TypeOf(p.slots[p.inputSlots[i]].type);
            if (t.construct)
                t.construct(_Value(copy, i));
        }
    }
    edited.assign(inputs, 0);
    editedList.reserve(inputs);
}

void
RigExecInputChannel::_Copy(uint32_t to, uint32_t from, uint32_t input)
{
    const RigExecTypeDesc &t =
        TypeOf(program->slots[program->inputSlots[input]].type);
    if (t.copy)
        t.copy(_Value(to, input), _Value(from, input));
    else
        std::memcpy(_Value(to, input), _Value(from, input), t.size);
    versions[to][input] = versions[from][input];
}

void
RigExecInputChannel::CopyFrom(const RigExecInputChannel &source)
{
    for (uint32_t copy = 0; copy < 3; ++copy)
        for (uint32_t i = 0; i < program->inputSlots.size(); ++i) {
            const RigExecTypeDesc &t =
                TypeOf(program->slots[program->inputSlots[i]].type);
            std::byte *dst = _Value(copy, i);
            const std::byte *src = source._Value(source.committed, i);
            if (t.copy)
                t.copy(dst, src);
            else
                std::memcpy(dst, src, t.size);
            versions[copy][i] = source.versions[source.committed][i];
        }
}

bool
RigExecInputChannel::Acquire()
{
    if (!(middle.load(std::memory_order_acquire) & kFresh))
        return false;
    front = middle.exchange(front, std::memory_order_acq_rel) & kIndex;
    return true;
}

RigExecInputWriter
RigExecInputChannel::Begin()
{
    if (writing.exchange(true, std::memory_order_acquire))
        return RigExecInputWriter();
    return RigExecInputWriter(*this);
}

void
RigExecInputChannel::_Mark(uint32_t input)
{
    versions[back][input] = versions[committed][input] + 1;
    if (!edited[input]) {
        edited[input] = 1;
        editedList.push_back(input);
    }
}

bool
RigExecInputChannel::Set(uint32_t input, const void *value)
{
    if (input >= program->inputSlots.size() || !value)
        return false;
    const RigExecTypeDesc &t =
        TypeOf(program->slots[program->inputSlots[input]].type);
    if (t.copy)
        t.copy(_Value(back, input), value);
    else
        std::memcpy(_Value(back, input), value, t.size);
    _Mark(input);
    return true;
}

void *
RigExecInputChannel::Edit(uint32_t input)
{
    if (!edited[input] && versions[back][input] != versions[committed][input])
        _Copy(back, committed, input);
    _Mark(input);
    return _Value(back, input);
}

void
RigExecInputChannel::Commit()
{
    for (uint32_t i = 0; i < program->inputSlots.size(); ++i)
        if (!edited[i] && versions[back][i] != versions[committed][i])
            _Copy(back, committed, i);
    committed = back;
    back = middle.exchange(back | kFresh, std::memory_order_acq_rel) & kIndex;
    Close();
}

void
RigExecInputChannel::Close()
{
    for (const uint32_t input : editedList)
        edited[input] = 0;
    editedList.clear();
    writing.store(false, std::memory_order_release);
}

RigExecInputWriter::RigExecInputWriter(RigExecInputChannel &channel)
    : _channel(&channel)
{
}

RigExecInputWriter::RigExecInputWriter(RigExecInputWriter &&other) noexcept
    : _channel(std::exchange(other._channel, nullptr))
{
}

RigExecInputWriter &
RigExecInputWriter::operator=(RigExecInputWriter &&other) noexcept
{
    if (this != &other) {
        _Close();
        _channel = std::exchange(other._channel, nullptr);
    }
    return *this;
}

RigExecInputWriter::~RigExecInputWriter()
{
    _Close();
}

void
RigExecInputWriter::_Close()
{
    if (_channel)
        std::exchange(_channel, nullptr)->Close();
}

bool
RigExecInputWriter::Set(RigExecInputId input, const void *value,
                        std::string *error)
{
    if (_channel && _channel->Set(input.index, value))
        return true;
    if (error)
        *error = _channel ? "SetInput: no such input"
                          : "SetInput: another edit is open";
    return false;
}

void *
RigExecInputWriter::_Edit(RigExecInputId input,
                          const RigExecTypeHandle &expected)
{
    if (!_channel || input.index >= _channel->program->inputSlots.size())
        return nullptr;
    const RigExecProgram &p = *_channel->program;
    if (p.slots[p.inputSlots[input.index]].type != expected.Index())
        return nullptr;
    return _channel->Edit(input.index);
}

void
RigExecInputWriter::Commit()
{
    if (_channel)
        std::exchange(_channel, nullptr)->Commit();
}

} // namespace rigExec
