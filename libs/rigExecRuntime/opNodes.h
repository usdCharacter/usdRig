#ifndef RIGEXEC_RUNTIME_OP_NODES_H
#define RIGEXEC_RUNTIME_OP_NODES_H

/// File.op_graph in the runtime: Open resolves its ops and types against
/// the op registry, decodes its payloads and compiles it for the common
/// graph's schedule; its OpNode, OpChainHead and OpChainRange steps run it
/// (rigExecEvalCore/opSteps.h).

#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecRuntime/values.h"

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
    /// Per node: the property chain and part (1 to its revision count) it
    /// runs, or kRigExecInvalidIndex for a node of no property chain.
    std::vector<uint32_t> propertyChain, propertyPart;
    /// Per node: the weight object whose packet it stores, or
    /// kRigExecInvalidIndex.
    std::vector<uint32_t> weightObject;
    /// Per lookup: the points a ChainVersion lookup gathers, owned by the
    /// one step that resolves it.
    std::vector<std::vector<RrVec3f>> chainVersionGather;
    /// Per node: the commit whose PoseCommit it runs, or
    /// kRigExecInvalidIndex.
    std::vector<uint32_t> commit;
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
