#ifndef RIGEXEC_INTERNAL_COMPILE_RIG_DESC_DATA_H
#define RIGEXEC_INTERNAL_COMPILE_RIG_DESC_DATA_H

#include "rigExec/api/opFields.h"
#include "rigExec/api/rigDesc.h"
#include "rigExec/internal/program/alignedBuffer.h"

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace rigExec {

struct RigExecDescNode {
    uint32_t op = kRigExecInvalidIndex;
    /// Holds a payload copy; empty for an op without a payload.
    RigExecAlignedBuffer payload;
    bool hasPayload = false;
    /// The values of an op's declared payload fields.
    RigExecFieldValues fields;
};

struct RigExecDescPort {
    RigExecNodeId node = 0;
    uint32_t port = kRigExecInvalidIndex;
};

struct RigExecDescConnection {
    RigExecDescPort from;
    RigExecDescPort to;
    /// Position in an array input's list; 0 otherwise.
    uint32_t element = 0;
};

struct RigExecDescInput {
    RigExecDescPort to;
    uint32_t element = 0;
};

struct RigExecRigDescData {
    uint64_t registryFingerprint = 0;
    std::vector<RigExecDescNode> nodes;
    std::vector<RigExecDescConnection> connections;
    std::vector<RigExecDescInput> inputs;
    std::vector<RigExecDescPort> outputs;
    std::vector<std::pair<RigExecNodeId, RigExecNodeId>> orders;
    /// Bounds the description gives for outputs whose op declares a Host
    /// bound; the last one given for a port counts.
    std::vector<std::pair<RigExecDescPort, size_t>> outputBounds;
    /// Elements recorded so far per array input.
    std::map<std::pair<RigExecNodeId, uint32_t>, uint32_t> listSizes;

    ~RigExecRigDescData();
};

} // namespace rigExec

#endif
