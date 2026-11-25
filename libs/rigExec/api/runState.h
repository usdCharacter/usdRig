#ifndef RIGEXEC_API_RUN_STATE_H
#define RIGEXEC_API_RUN_STATE_H

/// A run state is one independent set of values for a compiled program:
/// the inputs a host sets, the values its ops compute and keep from frame
/// to frame, and the outputs of its last evaluation. What compile fixed,
/// such as op settings, stays in the program and is shared by every run
/// state of it. Use one to evaluate a program beside an evaluator without
/// touching it: a what-if pose, a copy that continues from where an
/// evaluator is (RigExecEvaluator::CreateRunState), or a worker that
/// evaluates the same program on another thread.

#include "rigExec/api/evaluator.h"
#include "rigExec/api/view.h"

#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rigExec {

struct RigExecRunStateData;

/// One range chain of a program compiled with
/// RigExecCompileOptions::hostOrder: ranged nodes over one set of elements
/// that a host runs as one head and then, segment by segment, over groups
/// of elements it cuts itself (RigExecRunState::RunChainRange).
struct RigExecChainDesc {
    /// The member nodes, in the order a range runs through them.
    std::vector<RigExecNodeId> members;
    /// Per member: the member its segment starts at. A segment runs a
    /// group through its members as one step; a member that runs whole is
    /// a segment of its own, run as one group over every element.
    std::vector<uint32_t> segments;
    std::vector<char> whole;
    /// Per member: 1 when the chain keeps its pointwise outputs as one
    /// buffer per group, which GetValue does not return, because only
    /// later members of the chain read them.
    std::vector<char> grouped;
    /// Per member: the earlier members feeding it, and whether through a
    /// neighbourhood input, whose group reads the elements
    /// RigExecRunState::ChainReads names.
    struct Feed {
        uint32_t member = 0;
        bool neighbourhood = false;
    };
    std::vector<std::vector<Feed>> feeds;
};

/// What node \p node of a program reads and writes, by port of its op:
/// the value number (RigExecValueRef::value) a port reads or writes,
/// kRigExecInvalidIndex for an unfed optional input or an input that takes
/// many values, whose values `lists` holds; the node writing an input's
/// value, kRigExecInvalidIndex for one the host writes; and the host's
/// input index for one it writes, else kRigExecInvalidIndex.
struct RigExecNodeDesc {
    const RigExecOpDesc *op = nullptr;
    std::vector<uint32_t> values;
    std::vector<uint32_t> producers;
    std::vector<uint32_t> inputs;
    std::vector<std::vector<uint32_t>> lists;
};

size_t RigExecProgramNodeCount(const RigExecProgram &program);
bool RigExecProgramNodeAt(const RigExecProgram &program, size_t node,
                          RigExecNodeDesc *desc);

/// How many range chains \p program has, and chain \p chain.
size_t RigExecProgramChainCount(const RigExecProgram &program);
bool RigExecProgramChainAt(const RigExecProgram &program, size_t chain,
                           RigExecChainDesc *desc);

/// Use a run state from one thread at a time: the thread that created it,
/// or, after ReleaseOwner, the next thread that uses it. Run states of one
/// program may evaluate on different threads at once.
class RigExecRunState {
    struct _CreateKey {
        explicit _CreateKey(int)
        {
        }
    };

public:
    RigExecRunState(_CreateKey, std::unique_ptr<RigExecRunStateData> data);
    ~RigExecRunState();
    RigExecRunState(const RigExecRunState &) = delete;
    RigExecRunState &operator=(const RigExecRunState &) = delete;

    /// A run state of \p program with every input at its default value.
    /// Null with \p error as for RigExecEvaluator::Create.
    static std::unique_ptr<RigExecRunState>
    Create(std::shared_ptr<const RigExecProgram> program,
           const RigExecEvaluatorOptions &options = {},
           std::string *error = nullptr);

    /// Opens an edit of this run state's inputs (see inputWriter.h).
    RigExecInputWriter BeginInputEdit();
    /// As RigExecEvaluator::Evaluate, for this run state's values. Ops
    /// that keep history read the history this run state was given when
    /// it was made; evaluating adds none.
    bool Evaluate(std::string *error = nullptr);
    /// As RigExecEvaluator::GetOutput and ViewOutput.
    const void *GetOutput(RigExecOutputId output,
                          std::string *error = nullptr) const;
    RigExecValueView ViewOutput(RigExecOutputId output,
                                std::string *error = nullptr) const;
    /// A new run state that continues from this one: the same committed
    /// inputs, op state, outputs and history. Call it while no edit of
    /// this run state's inputs is open.
    std::unique_ptr<RigExecRunState> Clone() const;
    /// Running a program compiled with RigExecCompileOptions::hostOrder in
    /// a schedule of the host's: every node that does not run in ranges by
    /// RunNode, and every range chain by RunChainHead and then
    /// RunChainRange per segment and group, each after what it reads.
    /// Steps that do not feed each other may run at once on different
    /// threads; so may groups of one segment. A step that fails returns
    /// false with the reason, and the host stops its evaluation.
    ///
    /// Reads input \p input straight from \p value, without copying it,
    /// instead of from the value BeginInputEdit set; null goes back to
    /// that. The host keeps \p value unchanged while a step reads it.
    bool PointInput(RigExecInputId input, const void *value);
    /// Reads input \p input, which only range chains' members read through
    /// pointwise inputs, from the groups of elements \p groups lists, without
    /// copying them: a chain whose head is given the same bounds reads
    /// group g where it runs group g. Null goes back to the value
    /// BeginInputEdit set. The host keeps the groups unchanged while a step
    /// reads them.
    bool PointInputGroups(RigExecInputId input, const RigExecOpGroups *groups,
                          std::string *error = nullptr);
    bool RunNode(RigExecNodeId node, std::string *error = nullptr);
    /// Prepares every member of chain \p chain for a frame cut into
    /// \p groups groups of elements, group g holding [bounds[g],
    /// bounds[g + 1]).
    bool RunChainHead(size_t chain, const int *bounds, uint32_t groups,
                      std::string *error = nullptr);
    /// Runs group \p group of the segment starting at member \p segment of
    /// chain \p chain; a member that runs whole runs as group 0, over
    /// every element.
    bool RunChainRange(size_t chain, uint32_t segment, uint32_t group,
                       std::string *error = nullptr);
    /// The elements, [*lo, *hi), of its neighbourhood input that member
    /// \p member reads for group \p group; known after RunChainHead.
    void ChainReads(size_t chain, uint32_t member, uint32_t group, size_t *lo,
                    size_t *hi) const;
    /// How many elements chain \p chain's members hold this frame, known
    /// after RunChainHead.
    size_t ChainElements(size_t chain) const;
    /// The value \p ref names (RigExecResolveNodeOutput) as the step that
    /// wrote it last left it; null when the program has no such value or
    /// keeps it in its chain's groups.
    const void *GetValue(const RigExecValueRef &ref) const;
    /// The elements of group \p group of the value \p ref names, written
    /// by a member of a range chain, and their count in \p count; null
    /// with a count of 0 when the program has no such value or group.
    /// Valid until the next step writes that value.
    const void *GetGroupValue(const RigExecValueRef &ref, uint32_t group,
                              size_t *count) const;

    /// Lets another thread take the run state over: the next call, from
    /// any thread, makes that thread its owner.
    void ReleaseOwner();
    const RigExecProgram &GetProgram() const;

private:
    friend class RigExecEvaluator;
    /// Debug builds check calls come from the owner thread.
    void _AssertOwner() const;

    std::unique_ptr<RigExecRunStateData> _data;
    mutable std::thread::id _owner = std::this_thread::get_id();
};

} // namespace rigExec

#endif
