// One already-scheduled step of a compiled rig, in the shape both front ends
// hand to EvalCore. Adapters translate their own records into this; nothing
// here traverses a graph or reads a stage. Internal to the evaluator and
// playback, never an exported API.
#ifndef RIGEXEC_EVAL_CORE_STEP_H
#define RIGEXEC_EVAL_CORE_STEP_H

#include <cstddef>
#include <cstdint>

namespace rigExec {
namespace eval {

/// The kind ordinal, the object and part the step works on, and the graph
/// slot its output is bound to. `object` and `part` keep the adapters'
/// sentinels until a lift validates them.
struct StepRef {
    uint32_t kind;
    int32_t object;
    int32_t part;
    size_t slot;
};

/// Applied writes the result, Held keeps the previously bound one, Rejected
/// is the operation's own rejection, Invalid goes to the existing diagnostic
/// path.
enum class Outcome { Applied, Held, Rejected, Invalid };

/// `detail` is the reason the adapter already holds, zero when there is none.
struct StepResult {
    Outcome outcome;
    int32_t detail;
};

// RunGeometryStep and RunValueStep come from the family and chain headers
// that instantiate them against a host's storage and math binding.

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_STEP_H
