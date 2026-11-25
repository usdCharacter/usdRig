#ifndef RIGEXEC_RUNTIME_OP_NODES_H
#define RIGEXEC_RUNTIME_OP_NODES_H

/// File.op_graph in the runtime: Open resolves its ops and types against
/// the op registry, decodes its payloads and compiles it for the common
/// graph's schedule; its OpNode, OpChainHead and OpChainRange steps run it
/// (rigExecEvalCore/opSteps.h).

#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExecEvalCore/opSteps.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

struct RrProgram;

/// A published output: its name, the program output and its type.
struct RrOpNodeOutput {
    std::string name;
    RigExecOutputId id;
    uint32_t type = kRigExecInvalidIndex;
};

struct RrOpNodes {
    eval::OpStepLayout layout;
    std::unique_ptr<eval::OpStepState> state;
    std::vector<RrOpNodeOutput> outputs;
};

/// Builds \p program's op nodes from its file, when it holds any. False
/// with the reason for an op or type the registry does not know, a port
/// past its op's, a payload that does not decode, a constant or connection
/// of another type, a program the op core refuses, or range chains that
/// are not the file's.
bool RrOpenOpNodes(RrProgram *program, std::string *error);

/// The body of op step \p step.
bool RrRunOpStep(RrProgram *program, size_t step, std::string *error);

/// Op value \p slot's version in \p domain (OpValue or OpGroupValue); 0
/// when there is none.
uint64_t RrOpValueVersion(const RrProgram *program, uint32_t domain,
                          uint32_t slot);

} // namespace rigExec

#endif
