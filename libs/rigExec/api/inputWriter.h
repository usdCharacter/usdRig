#ifndef RIGEXEC_API_INPUT_WRITER_H
#define RIGEXEC_API_INPUT_WRITER_H

/// Changing the values a host feeds a program. Begin an edit, set any
/// number of inputs, then Commit: the next evaluation sees every change of
/// the edit together, and an evaluation already running sees none of them.

#include "rigExec/api/rigDesc.h"

#include <string>

namespace rigExec {

struct RigExecInputChannel;

/// One open edit of a program's inputs (RigExecEvaluator::BeginInputEdit,
/// RigExecRunState::BeginInputEdit). One edit may be open at a time per
/// evaluator or run state. It may be made on any thread, also while
/// another thread evaluates, but must end before the evaluator or run
/// state it came from is destroyed or switches programs. An edit that is
/// destroyed without Commit changes nothing.
class RigExecInputWriter {
public:
    RigExecInputWriter() = default;
    RigExecInputWriter(RigExecInputWriter &&other) noexcept;
    RigExecInputWriter &operator=(RigExecInputWriter &&other) noexcept;
    RigExecInputWriter(const RigExecInputWriter &) = delete;
    RigExecInputWriter &operator=(const RigExecInputWriter &) = delete;
    ~RigExecInputWriter();

    /// False when another edit was already open when this one was begun;
    /// such a writer changes nothing.
    bool IsOpen() const
    {
        return _channel != nullptr;
    }
    /// Copies \p value, which has the type input \p input was declared
    /// with, into that input.
    bool Set(RigExecInputId input, const void *value,
             std::string *error = nullptr);
    /// Input \p input's value, to change in place; it starts as the value
    /// the last committed edit left. Null when the edit is not open, the
    /// input does not exist or its type is not \p expected. Valid until
    /// Commit.
    template <class T>
    T *Edit(RigExecInputId input, const RigExecTypeHandle &expected)
    {
        return static_cast<T *>(_Edit(input, expected));
    }
    /// Hands the edit to the next evaluation and closes it.
    void Commit();

private:
    friend struct RigExecInputChannel;
    explicit RigExecInputWriter(RigExecInputChannel &channel);
    void *_Edit(RigExecInputId input, const RigExecTypeHandle &expected);
    void _Close();

    /// Borrowed: the channel of the evaluator or run state it came from.
    RigExecInputChannel *_channel = nullptr;
};

} // namespace rigExec

#endif
