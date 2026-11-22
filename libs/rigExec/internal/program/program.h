#ifndef RIGEXEC_INTERNAL_PROGRAM_PROGRAM_H
#define RIGEXEC_INTERNAL_PROGRAM_PROGRAM_H

/// The compiled program: immutable, USD-free, shared by evaluators. Compile
/// writes it; exec only reads it.

#include "rigExec/api/compile.h"
#include "rigExec/internal/program/alignedBuffer.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

/// One value in the arena.
struct RigExecProgramSlot {
    size_t offset = 0;
    uint32_t type = kRigExecInvalidIndex;
};

struct RigExecProgramNode {
    uint32_t op = kRigExecInvalidIndex;
    /// Into the payload blob; SIZE_MAX without a payload.
    size_t payloadOffset = SIZE_MAX;
    /// Into the state arena; SIZE_MAX without state.
    size_t stateOffset = SIZE_MAX;
    /// Into portSlots; one entry per op port, in port order.
    uint32_t firstPort = 0;
    uint32_t portCount = 0;
};

/// Nodes of one op at one level, run in node order.
struct RigExecProgramBatch {
    uint32_t op = kRigExecInvalidIndex;
    uint32_t firstNode = 0;
    uint32_t nodeCount = 0;
    bool ownerThread = false;
};

class RigExecProgram {
public:
    struct _CompileKey {
        explicit _CompileKey(int)
        {
        }
    };
    explicit RigExecProgram(_CompileKey)
    {
    }
    ~RigExecProgram();
    RigExecProgram(const RigExecProgram &) = delete;
    RigExecProgram &operator=(const RigExecProgram &) = delete;

    uint64_t registryFingerprint = 0;
    std::vector<RigExecProgramSlot> slots;
    std::vector<RigExecProgramNode> nodes;
    /// Slot per node port; kRigExecInvalidIndex for an unfed optional input.
    std::vector<uint32_t> portSlots;
    /// Node ids in batch order.
    std::vector<uint32_t> batchNodes;
    std::vector<RigExecProgramBatch> batches;
    /// Batches of level l are [levelBatches[l], levelBatches[l + 1]).
    std::vector<uint32_t> levelBatches;
    /// Slot per bound input and output, in binding order.
    std::vector<uint32_t> inputSlots;
    std::vector<uint32_t> outputSlots;
    size_t arenaSize = 0;
    size_t stateSize = 0;
    RigExecAlignedBuffer payloads;
};

} // namespace rigExec

#endif
