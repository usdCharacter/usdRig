// Counts the allocations one evaluation makes, so an op test can pin a
// steady-state frame to none. Include from exactly one translation unit of
// a test executable: it replaces the global allocation functions.
#ifndef RIGEXEC_TESTS_OP_ALLOCATION_COUNT_H
#define RIGEXEC_TESTS_OP_ALLOCATION_COUNT_H

#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

namespace opAllocationCount {
inline std::atomic<bool> counting{false};
inline std::atomic<size_t> allocations{0};

inline void *
Allocate(size_t size)
{
    if (counting.load(std::memory_order_relaxed))
        allocations.fetch_add(1, std::memory_order_relaxed);
    if (void *p = std::malloc(size ? size : 1))
        return p;
    throw std::bad_alloc();
}

/// The allocations \p run makes.
template <class Fn>
size_t
Count(Fn &&run)
{
    allocations.store(0);
    counting.store(true);
    run();
    counting.store(false);
    return allocations.load();
}

/// The most allocations one steady frame of a lone \p op node makes: every
/// input is set to \p values once, then before each frame only input
/// \p varied, to \p vary(frame). The first two frames warm up. SIZE_MAX when
/// the node does not compile or a frame fails.
template <class Vary>
size_t
SteadyFrameAllocations(
    const rigExec::RigExecOpHandle &op, const void *payload,
    const std::vector<const rigExec::RigExecPortHandle *> &ins,
    const std::vector<const void *> &values,
    const rigExec::RigExecPortHandle &output, size_t varied, Vary &&vary,
    std::string *error)
{
    rigExec::RigExecRigDescBuilder b;
    const rigExec::RigExecNodeId node = b.AddOp(op, payload);
    std::vector<rigExec::RigExecInputId> ids;
    for (const rigExec::RigExecPortHandle *port : ins)
        ids.push_back(b.BindInput(node, *port));
    b.BindOutput(node, output);
    auto desc = b.Build(error);
    auto program = desc ? rigExec::RigExecCompile(*desc, {}, error) : nullptr;
    auto evaluator = program
                         ? rigExec::RigExecEvaluator::Create(program, {}, error)
                         : nullptr;
    if (!evaluator || values.size() != ids.size() || varied >= ids.size())
        return SIZE_MAX;
    size_t most = 0;
    for (int frame = 0; frame < 6; ++frame) {
        // Committed outside the count: a commit copies the inputs it did not
        // edit into the next buffer, which is the channel's work, not the op's.
        rigExec::RigExecInputWriter edit = evaluator->BeginInputEdit();
        if (frame == 0)
            for (size_t i = 0; i < ids.size(); ++i)
                if (!edit.Set(ids[i], values[i], error))
                    return SIZE_MAX;
        if (!edit.Set(ids[varied], vary(frame), error))
            return SIZE_MAX;
        edit.Commit();
        bool ran = false;
        const size_t made = Count([&] { ran = evaluator->Evaluate(error); });
        if (!ran)
            return SIZE_MAX;
        if (frame >= 2)
            most = std::max(most, made);
    }
    return most;
}
} // namespace opAllocationCount

void *
operator new(size_t size)
{
    return opAllocationCount::Allocate(size);
}

void *
operator new[](size_t size)
{
    return opAllocationCount::Allocate(size);
}

void
operator delete(void *p) noexcept
{
    std::free(p);
}

void
operator delete[](void *p) noexcept
{
    std::free(p);
}

void
operator delete(void *p, size_t) noexcept
{
    std::free(p);
}

void
operator delete[](void *p, size_t) noexcept
{
    std::free(p);
}

#endif
