#ifndef RIGEXEC_INTERNAL_COMPILE_RIG_DESC_DATA_H
#define RIGEXEC_INTERNAL_COMPILE_RIG_DESC_DATA_H

#include "rigExec/api/rigDesc.h"
#include "rigExec/internal/program/alignedBuffer.h"

#include <cstdint>
#include <vector>

namespace rigExec {

struct RigExecDescNode {
    uint32_t op = kRigExecInvalidIndex;
    /// Holds a payload copy; empty for an op without a payload.
    RigExecAlignedBuffer payload;
    bool hasPayload = false;
};

struct RigExecDescPort {
    RigExecNodeId node = 0;
    uint32_t port = kRigExecInvalidIndex;
};

struct RigExecDescConnection {
    RigExecDescPort from;
    RigExecDescPort to;
};

struct RigExecRigDescData {
    uint64_t registryFingerprint = 0;
    std::vector<RigExecDescNode> nodes;
    std::vector<RigExecDescConnection> connections;
    std::vector<RigExecDescPort> inputs;
    std::vector<RigExecDescPort> outputs;
    std::vector<std::pair<RigExecNodeId, RigExecNodeId>> orders;

    ~RigExecRigDescData();
};

} // namespace rigExec

#endif
