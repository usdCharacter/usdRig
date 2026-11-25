#ifndef RIGEXEC_API_COMPILE_H
#define RIGEXEC_API_COMPILE_H

/// Turns a rig description into a program an evaluator can run: it checks
/// the connections, works out the order the ops run in and which can run
/// at the same time, and lays out the memory for every value.

#include "rigExec/api/rigDesc.h"

#include <cstddef>
#include <memory>
#include <string>

namespace rigExec {

class RigExecProgram;

struct RigExecCompileOptions {
    /// The most nodes handed to one worker thread at a time; 0 for no
    /// limit. Smaller groups spread work across more threads. Results are
    /// the same for any value.
    size_t maxBatchNodes = 0;
    /// Whether ops that have both an SSE2 form and a plain scalar form use
    /// the SSE2 form. The two forms can differ in the last bit of a result,
    /// so every evaluator of one rig must use the same setting; the scalar
    /// form is the one the recorded fixtures use.
    bool useSimd = true;
    /// Elements per range of a node whose op runs in ranges
    /// (RigExecOpRangeHandling); 0 runs each such node as one range. A
    /// node with fewer than rangeThreshold elements, or at most rangeGrain,
    /// also runs as one range. Results are the same for any values.
    size_t rangeGrain = 0;
    size_t rangeThreshold = 0;
    /// Whether a host runs the program's steps in a schedule of its own
    /// (runState.h): a node at a time, and each range chain as a head and
    /// ranges of elements the host cuts. Compile fails when a connection
    /// or order pair runs from a node to one added before it, or when a
    /// node keeps history (a Windowed op).
    bool hostOrder = false;
};

/// The compiled program, shared by any number of evaluators; it never
/// changes. Null with \p error when the description is invalid, for example
/// when a required input has nothing feeding it, an input that takes one
/// value is fed twice, a connection joins different types, or the order
/// runs in a circle.
std::shared_ptr<const RigExecProgram>
RigExecCompile(const RigExecRigDesc &desc,
               const RigExecCompileOptions &options = {},
               std::string *error = nullptr);

} // namespace rigExec

#endif
