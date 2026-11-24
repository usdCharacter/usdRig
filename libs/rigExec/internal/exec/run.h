#ifndef RIGEXEC_INTERNAL_EXEC_RUN_H
#define RIGEXEC_INTERNAL_EXEC_RUN_H

/// One run of a program: its work arena (the values only its nodes read,
/// then its op state), its published arenas, and the port pointers its
/// batches are handed. It reads the program's constant arena and an input
/// channel's front copy, and is used by one thread at a time.

#include "rigExec/api/opFields.h"
#include "rigExec/api/taskSystem.h"
#include "rigExec/internal/exec/inputChannel.h"
#include "rigExec/internal/program/program.h"
#include "rigExecGraph/windowHistory.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// One bound output's value for the frames it was written in.
struct RigExecPublishedArena {
    /// Views holding the value.
    std::atomic<uint32_t> pins{0};
    uint64_t generation = 0;
    RigExecAlignedBuffer storage;
};

/// One bound output's arenas: the current one and the ones views still
/// hold or that wait to be reused. Every arena of a pool is reserved for
/// `reserved` elements.
struct RigExecPublishedPool {
    uint32_t slot = kRigExecInvalidIndex;
    uint32_t type = kRigExecInvalidIndex;
    uint32_t current = 0;
    size_t reserved = 0;
    std::vector<std::unique_ptr<RigExecPublishedArena>> arenas;
};

/// A run's published arenas; views keep it alive, so it may outlive the
/// run and the program.
struct RigExecPublishedSet {
    std::vector<RigExecPublishedPool> pools;

    RigExecPublishedSet() = default;
    RigExecPublishedSet(const RigExecPublishedSet &) = delete;
    RigExecPublishedSet &operator=(const RigExecPublishedSet &) = delete;
    ~RigExecPublishedSet();

    /// A new arena of \p pool with a constructed value reserved as the
    /// pool is.
    RigExecPublishedArena &Add(RigExecPublishedPool &pool);
};

/// One range of one instance of a ranged batch, run as one task.
struct RigExecRangeTask {
    uint32_t batch = 0;
    uint32_t instance = 0;
    uint32_t range = 0;
};

struct RigExecRun {
    const RigExecProgram *program = nullptr;
    /// Borrowed; outlives the run.
    const RigExecInputChannel *channel = nullptr;
    uint32_t boundFront = kRigExecInvalidIndex;
    RigExecAlignedBuffer work;
    std::shared_ptr<RigExecPublishedSet> published;
    /// Per slot: the elements its value is reserved for; SIZE_MAX for a
    /// value without a bound.
    std::vector<size_t> bounds;
    std::vector<size_t> reserved;
    /// Successful frames so far.
    uint64_t generation = 0;
    /// Per program port: the value an op reads or writes there.
    std::vector<const void *> inputs;
    std::vector<void *> outputs;
    /// Array inputs: element values, and the records inputs point to.
    std::vector<const void *> listItems;
    std::vector<RigExecPortList> lists;
    /// Per window: this run's capture, and the record its node is handed.
    std::vector<RigExecWindowPacketPtr> captures;
    std::vector<RigExecWindowRun> windows;
    /// Per batchNodes entry: its window record, or null.
    std::vector<const RigExecWindowRun *> batchWindows;
    /// Declared fields as batch functions see them, and the storage of the
    /// state fields sized from an input, per state run.
    std::vector<RigExecPayloadColumn> payloadColumns;
    std::vector<RigExecFieldRunOf<const std::byte>> payloadRuns;
    std::vector<RigExecStateColumn> stateColumns;
    std::vector<RigExecFieldRunOf<std::byte>> stateRuns;
    std::vector<RigExecAlignedBuffer> stateBuffers;
    /// The nodes this frame runs, by batchNodes entry.
    std::vector<char> active;
    /// How many of each batch's nodes this frame runs; marked lists the
    /// active entries set, walk the nodes still to propagate from.
    std::vector<uint32_t> batchActive;
    std::vector<uint32_t> marked;
    std::vector<uint32_t> walk;
    /// Per input: the version of its value the last successful frame read.
    std::vector<uint64_t> seen;
    /// Whether the next frame runs every node: before the first frame and
    /// after a failed one.
    bool everything = true;
    /// Per batch: whether it failed this frame and why.
    std::vector<uint8_t> failed;
    std::vector<std::string> errors;
    /// This frame's tasks by task level, as the task system takes them:
    /// each program level's batches, owner-thread ones on the caller, then,
    /// when it has range tasks, those as a task level of their own. Task
    /// number batches + t is range task t.
    std::vector<uint32_t> sharedStart;
    std::vector<uint32_t> shared;
    std::vector<uint32_t> callerStart;
    std::vector<uint32_t> caller;
    /// Per batch: its program level and whether its op runs in ranges.
    std::vector<uint32_t> batchLevel;
    std::vector<char> rangedBatch;
    /// This frame's range tasks, by batch, instance and range; per batch
    /// its first one and how many have still to finish.
    std::vector<RigExecRangeTask> rangeTasks;
    std::vector<uint32_t> firstRangeTask;
    std::unique_ptr<std::atomic<uint32_t>[]> rangesLeft;
    std::vector<uint8_t> rangeFailed;
    std::vector<std::string> rangeErrors;
    /// Per ranged batch: the instance whose head failed this frame, or
    /// kRigExecInvalidIndex, and why.
    std::vector<uint32_t> headFailedAt;
    std::vector<std::string> headErrors;
    /// Per batchNodes entry of a ranged op: this frame's plan, element
    /// count and scheduled ranges, and each range's scratch.
    std::vector<uint8_t> plans;
    std::vector<size_t> elementCounts;
    std::vector<uint32_t> instanceRanges;
    std::vector<std::vector<RigExecAlignedBuffer>> scratch;
    /// Per program level: whether its range tasks run this frame, and
    /// whether a task of that level failed.
    std::vector<char> levelSplit;
    std::unique_ptr<std::atomic<uint8_t>[]> levelFailed;

    RigExecRun() = default;
    RigExecRun(const RigExecRun &) = delete;
    RigExecRun &operator=(const RigExecRun &) = delete;
    ~RigExecRun();

    /// \p channel and \p rings outlive the run; \p rings keeps its size.
    void Bind(const RigExecProgram &p, const RigExecInputChannel &channel,
              const std::vector<RigExecWindowRing> &rings);

    std::byte *State() const
    {
        return work.Data() + program->stateBase;
    }
    /// Where slot \p slot's value is now.
    const void *Value(uint32_t slot) const;

    /// Runs one frame on the channel's front copy: the nodes reading an
    /// input whose value changed since the last successful frame, every
    /// node that is not Pure, and every node a rerun node feeds; on a
    /// history tick only those the windowed nodes depend on. Every other
    /// value carries forward unchanged. A failed run leaves every capture
    /// null, returns the first failure in batch order, and makes the next
    /// frame run every node.
    bool Run(bool historyTick, RigExecTaskSystem *tasks, std::string *error);
    /// Takes \p source's work values, op state, published values and
    /// captures; both are bound to one program. The next frame runs every
    /// node.
    void CopyFrom(const RigExecRun &source);

private:
    void _BindFields();
    void _Point(uint32_t slot, void *value);
    void _PointReaders(uint32_t slot, const void *value);
    void *_Written(uint32_t slot) const;
    void _SettleBounds();
    void _Reserve(uint32_t slot, size_t count);
    void _MarkDirty(bool historyTick);
    void _Mark(uint32_t node, const char *allowed);
    bool _Runs(uint32_t slot) const;
    void _PrepareOutputs();
    void _Publish();
    void _Schedule();
    size_t _RangeCount(size_t elements) const;
    RigExecOpBatch _View(uint32_t b) const;
    void _RunBatch(uint32_t b);
    bool _RunHeads(uint32_t b);
    bool _RunRange(uint32_t t);
    void _CompleteRanges(uint32_t b);
    void _Fail(uint32_t b, uint32_t instance, const std::string &error);
    static bool _RunTask(void *context, uint32_t task);
    size_t _SizeFromInputs(const RigExecOpBatch &view,
                           const RigExecProgramBatch &batch,
                           const RigExecRegisteredOp &op, std::string *error);
    bool _CheckBounds(const RigExecOpBatch &view,
                      const RigExecProgramBatch &batch,
                      const RigExecRegisteredOp &op, size_t *at,
                      std::string *error) const;
};

} // namespace rigExec

#endif
