#ifndef RIGEXEC_INTERNAL_PROGRAM_PROGRAM_H
#define RIGEXEC_INTERNAL_PROGRAM_PROGRAM_H

/// The compiled program: immutable and shared by evaluators. Compile writes
/// it; exec only reads it.

#include "rigExec/api/compile.h"
#include "rigExec/internal/program/alignedBuffer.h"
#include "rigExecGraph/opGraph.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

/// The kinds of arena a program's values live in. The constant arena (the
/// payload blob) is fixed by compile and shared by every run of the
/// program. The input arena holds the values the host writes. A run's work
/// arena holds the values only its nodes read, then its op state. Each
/// bound output has a published arena of its own, which hosts read and pin.
enum class RigExecArenaKind : uint8_t { Constant, Input, Work, Published };

/// One value: its arena, its offset there and its type. arenaIndex is the
/// input's binding index for an Input slot and the published arena's index
/// for a Published slot.
struct RigExecProgramSlot {
    size_t offset = 0;
    uint32_t type = kRigExecInvalidIndex;
    RigExecArenaKind arena = RigExecArenaKind::Work;
    uint32_t arenaIndex = 0;
};

/// One value an Input or InputValue bound follows: its elements, or the
/// integer it holds.
struct RigExecProgramBoundSource {
    uint32_t slot = kRigExecInvalidIndex;
    bool integer = false;
};

/// The most elements a value whose type holds arrays may hold. Compile
/// settles Fixed, Field and Host bounds to `count`; an Input or
/// InputValue bound is `count` times the largest of its sources,
/// boundSources[firstSource, firstSource + sourceCount), which each run
/// settles from the values it is given.
struct RigExecProgramBound {
    RigExecOutputBound kind = RigExecOutputBound::None;
    uint32_t firstSource = 0;
    uint32_t sourceCount = 0;
    size_t count = 0;
};

/// Where a run keeps a pointer to a slot's value: an input or output port
/// entry, or an array input's element entry.
struct RigExecProgramCell {
    enum Kind : uint8_t { Input, Output, ListItem };
    Kind kind = Input;
    uint32_t index = 0;
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

/// An array input's elements: listSlots[firstSlot, firstSlot + count).
struct RigExecProgramList {
    /// Into portSlots.
    uint32_t port = 0;
    uint32_t firstSlot = 0;
    uint32_t count = 0;
};

/// A Windowed node (rigExecGraph/windowHistory.h), in node order.
struct RigExecProgramWindow {
    uint32_t node = kRigExecInvalidIndex;
    uint32_t frames = 0;
    /// The windowed nodes among its ancestors, as indices into
    /// windowedNodes, ascending.
    std::vector<uint32_t> inner;
    /// Some windowed node depends on it, so it solves at history ticks.
    bool feedsWindow = false;
};

/// One declared field across a batch: a column of `count` values per
/// instance at offset + k * stride, or, for a count that varies, one run
/// per instance in payloadRuns or stateRuns from firstRun.
struct RigExecProgramColumn {
    uint32_t type = kRigExecInvalidIndex;
    size_t offset = 0;
    size_t stride = 0;
    size_t count = 0;
    uint32_t firstRun = kRigExecInvalidIndex;
    /// A state field an evaluator sizes from an input when the node runs.
    bool fromInput = false;
};

/// One instance's values of a field whose count varies; offset is
/// SIZE_MAX until an evaluator sizes a field from an input.
struct RigExecProgramRun {
    size_t offset = SIZE_MAX;
    size_t count = 0;
};

/// Nodes of one op at one level, run together by one call of the op's
/// batch function. Its nodes' ports, payloads and states are each laid out
/// contiguously in batch order.
struct RigExecProgramBatch {
    uint32_t op = kRigExecInvalidIndex;
    /// Into batchNodes.
    uint32_t firstNode = 0;
    uint32_t nodeCount = 0;
    bool ownerThread = false;
    /// Into portSlots.
    uint32_t firstPort = 0;
    /// Into the payload blob and the state arena; SIZE_MAX without one.
    size_t payloadOffset = SIZE_MAX;
    size_t payloadStride = 0;
    size_t stateOffset = SIZE_MAX;
    size_t stateStride = 0;
    /// Into payloadColumns and stateColumns, one per declared field.
    uint32_t firstPayloadColumn = kRigExecInvalidIndex;
    uint32_t firstStateColumn = kRigExecInvalidIndex;
};

/// A range chain: ranged nodes over one partition of their elements, each
/// fed by the one before it through a pointwise or neighbourhood input,
/// run range by range through the chain (RigExecOpRangeHandling). Every
/// other input of a member is complete before the chain's first node's
/// level, so the chain runs in that level.
struct RigExecProgramChain {
    /// Into chainMembers.
    uint32_t firstMember = 0;
    uint32_t memberCount = 0;
    /// The program level of its first node.
    uint32_t level = 0;
};

/// A member's pointwise or neighbourhood input fed by an earlier member,
/// by that member's index in the chain.
struct RigExecProgramChainFeed {
    uint32_t member = 0;
    bool neighbourhood = false;
};

class RigExecProgram {
    friend std::shared_ptr<const RigExecProgram>
    RigExecCompile(const RigExecRigDesc &desc,
                   const RigExecCompileOptions &options, std::string *error);
    struct _CompileKey {
        explicit _CompileKey(int)
        {
        }
    };

public:
    explicit RigExecProgram(_CompileKey)
    {
    }
    ~RigExecProgram();
    RigExecProgram(const RigExecProgram &) = delete;
    RigExecProgram &operator=(const RigExecProgram &) = delete;

    uint64_t registryFingerprint = 0;
    /// RigExecCompileOptions::useSimd, rangeGrain and rangeThreshold.
    bool useSimd = true;
    size_t rangeGrain = 0;
    size_t rangeThreshold = 0;
    /// RigExecCompileOptions::hostOrder: every batch is one node, the
    /// batches in node order.
    bool hostOrder = false;
    /// The nodes as the shared dependency compiler ordered them, one op
    /// per node keyed by node id, reading and writing slot ids. Its
    /// readers map each slot to the nodes that read it.
    RigExecCompiledGraph graph;
    std::vector<RigExecProgramSlot> slots;
    std::vector<RigExecProgramNode> nodes;
    /// Slot per node port, nodes in batch order; kRigExecInvalidIndex for
    /// an unfed optional input and for an array input.
    std::vector<uint32_t> portSlots;
    /// One per array input, in node then port order.
    std::vector<RigExecProgramList> lists;
    std::vector<uint32_t> listSlots;
    /// Node ids in batch order.
    std::vector<uint32_t> batchNodes;
    std::vector<RigExecProgramBatch> batches;
    /// Batches of level l are [levelBatches[l], levelBatches[l + 1]).
    std::vector<uint32_t> levelBatches;
    /// Slot per bound input and output, in binding order.
    std::vector<uint32_t> inputSlots;
    std::vector<uint32_t> outputSlots;
    std::vector<RigExecProgramWindow> windowedNodes;
    /// Per node: 1 for the windowed nodes and every node one depends on.
    /// Empty without windowed nodes.
    std::vector<char> historyMask;
    /// historyMask in batchNodes order.
    std::vector<char> batchHistoryMask;
    /// Per node: its index in windowedNodes, or kRigExecInvalidIndex.
    std::vector<uint32_t> windowOfNode;
    /// Per slot: its bound, and where runs point at its value, as
    /// cells[cellStart[s], cellStart[s + 1]).
    std::vector<RigExecProgramBound> bounds;
    std::vector<RigExecProgramBoundSource> boundSources;
    std::vector<uint32_t> cellStart;
    std::vector<RigExecProgramCell> cells;
    /// The slots whose bound depends on another slot, in node order of the
    /// compiled graph, so each one's sources are settled before it.
    std::vector<uint32_t> derivedBounds;
    /// The slots of the published arenas, one per distinct bound output.
    std::vector<uint32_t> publishedSlots;
    /// Per slot: the nodes that read it, ascending, as
    /// readers[readerStart[s], readerStart[s + 1]), from the compiled
    /// graph's readers, and the node that writes it, or
    /// kRigExecInvalidIndex for a host-written input.
    std::vector<uint32_t> readerStart;
    std::vector<uint32_t> readers;
    std::vector<uint32_t> producer;
    /// Per node: the slots it writes, as
    /// nodeOutputs[nodeOutputStart[n], nodeOutputStart[n + 1]), and its
    /// batchNodes entry. volatileNodes run every frame: the nodes that are
    /// not Pure, ascending.
    std::vector<uint32_t> nodeOutputStart;
    std::vector<uint32_t> nodeOutputs;
    std::vector<uint32_t> batchPosition;
    std::vector<uint32_t> volatileNodes;
    /// Per batchNodes entry: its batch.
    std::vector<uint32_t> positionBatch;
    /// Every ranged node is in exactly one chain. chainMembers holds the
    /// members' batchNodes entries in chain order; per batchNodes entry,
    /// positionChain names its chain (kRigExecInvalidIndex for a node that
    /// does not run in ranges). Member m's feeds are
    /// chainFeeds[chainFeedStart[m], chainFeedStart[m + 1]), m indexing
    /// chainMembers.
    std::vector<RigExecProgramChain> chains;
    /// Per batchNodes entry: 1 for a ranged node whose settings run it
    /// whole (RigExecOpRangeHandling::plan).
    std::vector<char> wholePositions;
    std::vector<uint32_t> chainMembers;
    std::vector<uint32_t> positionChain;
    std::vector<uint32_t> chainFeedStart;
    /// Per chain member: the member its segment starts at when every member
    /// runs. A segment starts at the chain's first member, at a member that
    /// runs whole or follows one, and at a member reading an earlier one
    /// through a neighbourhood input.
    std::vector<uint32_t> chainSegment;
    std::vector<RigExecProgramChainFeed> chainFeeds;
    size_t inputArenaSize = 0;
    /// The work arena: values from 0, op state from stateBase.
    size_t workSize = 0;
    size_t stateBase = 0;
    size_t stateSize = 0;
    RigExecAlignedBuffer payloads;
    /// Declared fields, batch by batch: payload columns live in payloads,
    /// state columns in each evaluator's state arena.
    std::vector<RigExecProgramColumn> payloadColumns;
    std::vector<RigExecProgramRun> payloadRuns;
    std::vector<RigExecProgramColumn> stateColumns;
    std::vector<RigExecProgramRun> stateRuns;
};

} // namespace rigExec

#endif
