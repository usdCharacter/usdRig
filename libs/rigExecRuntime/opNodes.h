#ifndef RIGEXEC_RUNTIME_OP_NODES_H
#define RIGEXEC_RUNTIME_OP_NODES_H

/// File.op_graph in the runtime: Open resolves its ops and types against
/// the op registry, decodes its payloads and compiles it into an op-core
/// program; the OpGraph step writes the slots its nodes read into the
/// evaluator, evaluates, and keeps the named outputs.

#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

struct RrProgram;

/// How a slot's value becomes an op input of the matching standard type.
enum class RrOpSlotKind : uint8_t {
    Double,
    Float,
    Bool,
    Int,
    Mat4d,
    Vec3d,
    Vec3f,
    IntArray,
    FloatArray,
    DoubleArray,
    Vec2fArray,
    Vec3fArray,
    Vec3dArray,
    Mat4dArray,
    BoolArray,
};

struct RrOpNodeInput {
    RigExecInputId id;
    uint32_t slot = 0;
    RrOpSlotKind kind = RrOpSlotKind::Double;
};

/// A published output: a copy of the evaluator's value, owned here and
/// replaced by each run of the step.
struct RrOpNodeOutput {
    std::string name;
    RigExecOutputId id;
    uint32_t type = kRigExecInvalidIndex;
    std::vector<std::max_align_t> storage;
    bool held = false;
};

struct RrOpNodes {
    std::shared_ptr<const RigExecProgram> program;
    std::unique_ptr<RigExecEvaluator> evaluator;
    std::vector<RrOpNodeInput> inputs;
    std::vector<RrOpNodeOutput> outputs;

    RrOpNodes() = default;
    RrOpNodes(const RrOpNodes &) = delete;
    RrOpNodes &operator=(const RrOpNodes &) = delete;
    ~RrOpNodes();
};

/// Builds \p program's op nodes from its file, when it holds any. False
/// with the reason for an op or type the registry does not know, a port
/// past its op's, a payload that does not decode, a slot or connection of
/// another type, or a program the op core refuses.
bool RrOpenOpNodes(RrProgram *program, std::string *error);

/// The OpGraph step's body.
bool RrRunOpNodes(RrProgram *program, std::string *error);

} // namespace rigExec

#endif
