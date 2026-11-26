#ifndef RIGEXEC_API_FRAME_VIEW_H
#define RIGEXEC_API_FRAME_VIEW_H

/// What one evaluated value holds, and every bound output of one evaluation
/// held at once.

#include "rigExec/api/rigDesc.h"
#include "rigExec/api/view.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// The value's own size, plus, for an array type (one with an element
/// type), the elements it holds; 0 for an empty view.
size_t RigExecValueBytes(const RigExecValueView &view);
/// How many elements a type with an element count holds; 1 for any other
/// value; 0 for an empty view.
size_t RigExecValueCount(const RigExecValueView &view);
/// An array's first element; null for other types and empty views.
const void *RigExecValueElements(const RigExecValueView &view);

/// Which stored value an output's view reads: the published arenas of the
/// run, the output's pool and arena there, and the evaluation that wrote
/// it. Equal identities read the same bytes, so a reader can tell an
/// unchanged output, or two holders of one value, without reading it.
struct RigExecValueIdentity {
    const void *set = nullptr;
    uint32_t pool = kRigExecInvalidIndex;
    const void *arena = nullptr;
    uint64_t generation = 0;

    bool operator==(const RigExecValueIdentity &other) const
    {
        return set == other.set && pool == other.pool && arena == other.arena &&
               generation == other.generation;
    }
    bool operator!=(const RigExecValueIdentity &other) const
    {
        return !(*this == other);
    }
};

/// Every bound output of one evaluation, held together: while the frame
/// view is held, later evaluations write each output elsewhere, so all of
/// them stay as that evaluation left them. After a failed evaluation the
/// frame view is invalid and holds no value. A frame view may be copied,
/// moved and released on any thread, and it stays valid after the
/// evaluator or run state that made it is gone.
class RigExecFrameView {
public:
    RigExecFrameView() = default;
    RigExecFrameView(const RigExecFrameView &other);
    RigExecFrameView(RigExecFrameView &&other) noexcept;
    RigExecFrameView &operator=(const RigExecFrameView &other);
    RigExecFrameView &operator=(RigExecFrameView &&other) noexcept;
    ~RigExecFrameView();

    explicit operator bool() const
    {
        return _valid;
    }
    /// Why the frame view is invalid: "no valid value: " and the failure.
    const std::string &Error() const
    {
        return _error;
    }
    /// The successful evaluations of the run when the view was taken.
    uint64_t Generation() const
    {
        return _generation;
    }
    /// How many bound outputs the view holds; 0 when invalid.
    size_t OutputCount() const
    {
        return _entries.size();
    }
    /// Output \p output as a value view; empty when invalid or when the
    /// program has no such output.
    RigExecValueView View(RigExecOutputId output) const;
    /// Where output \p output's value is stored; a default identity when
    /// invalid or when the program has no such output.
    RigExecValueIdentity Identity(RigExecOutputId output) const;
    /// Lets every value go; the frame view is invalid afterwards.
    void Reset();

private:
    friend struct RigExecViewAccess;

    struct _Entry {
        /// Borrowed from the storage _owner keeps alive.
        std::atomic<uint32_t> *pins = nullptr;
        const void *arena = nullptr;
        const void *data = nullptr;
        const RigExecTypeHandle *type = nullptr;
        uint64_t generation = 0;
        uint32_t pool = kRigExecInvalidIndex;
    };

    std::shared_ptr<const void> _owner;
    std::vector<_Entry> _entries;
    std::string _error;
    uint64_t _generation = 0;
    bool _valid = false;
};

} // namespace rigExec

#endif
