#ifndef RIGEXEC_API_VIEW_H
#define RIGEXEC_API_VIEW_H

/// Reading a program's values from outside the evaluator. A value
/// reference says where one value of a compiled program lives; a view lends
/// one evaluated output and keeps it exactly as it was while the view is
/// held, whatever later evaluations write.

#include "rigExec/api/rigDesc.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace rigExec {

class RigExecProgram;
struct RigExecViewAccess;

/// The kinds of storage a program's values live in.
enum class RigExecArena : uint8_t {
    /// Settings fixed when the program was compiled, shared by everything
    /// that runs the program.
    Constant,
    /// The values the host writes, through RigExecInputWriter.
    Input,
    /// Values only the program's own ops read, and their working state;
    /// each run of the program has its own.
    Work,
    /// One bound output: what the host reads after each evaluation.
    Published,
};

/// Where one value of a program lives: its kind of storage, which one of
/// that kind (a Published value's output arena, an Input value's input
/// index), its byte offset there, how many values and their type. Resolve
/// a reference once per program and use it for every evaluation.
struct RigExecValueRef {
    RigExecArena arena = RigExecArena::Work;
    uint32_t index = 0;
    size_t offset = 0;
    size_t count = 0;
    const RigExecTypeHandle *type = nullptr;
    /// The value's number in its program, from 0 to below
    /// RigExecProgramValueCount. A host that runs the program in its own
    /// schedule (RigExecCompileOptions::hostOrder) names each node's
    /// inputs and outputs by it.
    uint32_t value = 0;
};

/// How many values \p program holds: every node output and every input.
size_t RigExecProgramValueCount(const RigExecProgram &program);

/// Where input \p input of \p program lives; false when the program has
/// no such input.
bool RigExecResolveInput(const RigExecProgram &program, RigExecInputId input,
                         RigExecValueRef *ref);
/// Where bound output \p output of \p program lives; false when the
/// program has no such output.
bool RigExecResolveOutput(const RigExecProgram &program, RigExecOutputId output,
                          RigExecValueRef *ref);
/// Where output \p out of node \p node lives; false when the node or the
/// output does not exist.
bool RigExecResolveNodeOutput(const RigExecProgram &program, RigExecNodeId node,
                              const RigExecPortHandle &out,
                              RigExecValueRef *ref);

/// One evaluated output, lent to a reader. While any view of it is held,
/// later evaluations write that output somewhere else, so the value stays
/// unchanged; release the view (destroy or Reset it) when done so the
/// storage can be reused. A view may be copied, moved and released on any
/// thread, and it stays valid after the evaluator that made it is gone.
class RigExecValueView {
public:
    RigExecValueView() = default;
    RigExecValueView(const RigExecValueView &other);
    RigExecValueView(RigExecValueView &&other) noexcept;
    RigExecValueView &operator=(const RigExecValueView &other);
    RigExecValueView &operator=(RigExecValueView &&other) noexcept;
    ~RigExecValueView();

    explicit operator bool() const
    {
        return _data != nullptr;
    }
    const void *Data() const
    {
        return _data;
    }
    const RigExecTypeHandle *Type() const
    {
        return _type;
    }
    /// Which evaluation wrote the value: its run's successful evaluations
    /// count from 1.
    uint64_t Generation() const
    {
        return _generation;
    }
    /// The value as a \p T when it has type \p expected; null when it has
    /// another type or the view is empty.
    template <class T> const T *As(const RigExecTypeHandle &expected) const
    {
        return _type == &expected ? static_cast<const T *>(_data) : nullptr;
    }
    /// Lets the value go; the view is empty afterwards.
    void Reset();

private:
    friend struct RigExecViewAccess;

    std::shared_ptr<const void> _owner;
    /// Borrowed from the storage _owner keeps alive.
    std::atomic<uint32_t> *_pins = nullptr;
    const void *_data = nullptr;
    const RigExecTypeHandle *_type = nullptr;
    uint64_t _generation = 0;
};

} // namespace rigExec

#endif
