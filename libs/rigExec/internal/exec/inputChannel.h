#ifndef RIGEXEC_INTERNAL_EXEC_INPUT_CHANNEL_H
#define RIGEXEC_INTERNAL_EXEC_INPUT_CHANNEL_H

/// A program's input arena, kept as three copies so a writer on any thread
/// never blocks or tears a running evaluation: the writer fills its back
/// copy and swaps it into the middle on commit; the evaluator swaps the
/// middle into its front copy at the start of a frame when a commit is
/// waiting. The atomic middle index is the only shared state.

#include "rigExec/api/inputWriter.h"
#include "rigExec/internal/program/program.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

struct RigExecInputChannel {
    static constexpr uint32_t kFresh = 4;
    static constexpr uint32_t kIndex = 3;

    const RigExecProgram *program = nullptr;
    RigExecAlignedBuffer buffers[3];
    /// Per copy, per input: the commit its value came from.
    std::vector<uint64_t> versions[3];
    std::atomic<uint32_t> middle{1};
    std::atomic<bool> writing{false};
    /// Owned by the open edit.
    uint32_t back = 2;
    uint32_t committed = 1;
    std::vector<char> edited;
    std::vector<uint32_t> editedList;
    /// Owned by the evaluating thread.
    uint32_t front = 0;

    RigExecInputChannel() = default;
    RigExecInputChannel(const RigExecInputChannel &) = delete;
    RigExecInputChannel &operator=(const RigExecInputChannel &) = delete;
    ~RigExecInputChannel();

    void Bind(const RigExecProgram &p);
    /// Makes every copy hold \p source's last committed values. Neither
    /// may have an edit open.
    void CopyFrom(const RigExecInputChannel &source);

    /// Evaluating thread: takes the last commit as the front copy when one
    /// is waiting; true when the front copy changed.
    bool Acquire();
    const std::byte *Front() const
    {
        return buffers[front].Data();
    }
    uint64_t FrontVersion(uint32_t input) const
    {
        return versions[front][input];
    }

    RigExecInputWriter Begin();
    bool Set(uint32_t input, const void *value);
    void *Edit(uint32_t input);
    void Commit();
    void Close();

private:
    std::byte *_Value(uint32_t copy, uint32_t input) const;
    void _Copy(uint32_t to, uint32_t from, uint32_t input);
    void _Mark(uint32_t input);
};

} // namespace rigExec

#endif
