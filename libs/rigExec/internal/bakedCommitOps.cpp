#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExecEvalCore/opSteps.h"
#include "rigExecMath/poseCommitKernel.h"
#include "rigExecOps/solver/poseCommitOps.h"
#include "rigExecRigTypes/rigTypes.h"

#include <string>
#include <vector>

namespace rigExec {

namespace {

using Commit = RigExecBakedCommit;
using Frames = rigExecStd::RigExecArray<rigExecRig::PointFrame>;
using Flags = rigExecStd::RigExecArray<uint8_t>;

bool
Fail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op nodes: " + why;
    return false;
}

/// The versions list \p part (OpCommitList) of \p commit names, and
/// whether they are base versions; null for no list.
const std::vector<uint32_t> *
Versions(const Commit &commit, int32_t part, bool *base)
{
    using L = eval::OpCommitList;
    *base = part == int32_t(L::BaseCarry) ||
            part == int32_t(L::DescendantBaseCarry);
    switch (L(part)) {
    case L::Entering:
        return &commit.slotReads;
    case L::Carry:
        return &commit.slotCarry;
    case L::BaseCarry:
        return &commit.slotBaseCarry;
    case L::Current:
        return &commit.descendantReads;
    case L::Closest:
        return &commit.closestReads;
    case L::DescendantCarry:
        return &commit.descendantCarry;
    case L::DescendantBaseCarry:
        return &commit.descendantBaseCarry;
    }
    return nullptr;
}

/// The versions output target \p target of \p commit writes, and whether
/// they are base versions; null for any other target.
const std::vector<uint32_t> *
Writes(const Commit &commit, uint8_t target, bool *base)
{
    using T = eval::OpOutputTarget;
    *base = target == uint8_t(T::CommitBaseSlots) ||
            target == uint8_t(T::CommitBaseDescendants);
    switch (T(target)) {
    case T::CommitSlots:
        return &commit.slotWrites;
    case T::CommitDescendants:
        return &commit.descendantWrites;
    case T::CommitBaseSlots:
        return &commit.slotBaseWrites;
    case T::CommitBaseDescendants:
        return &commit.descendantBaseWrites;
    default:
        return nullptr;
    }
}

rigExecRig::PointFrame
ToOp(const RigExecPointFrame &f)
{
    rigExecRig::PointFrame out;
    for (size_t i = 0; i < 4; ++i)
        out.points[i] =
            rigExecStd::Vec3d(f.points[i][0], f.points[i][1], f.points[i][2]);
    out.flags = f.flags;
    return out;
}

RigExecPointFrame
FromOp(const rigExecRig::PointFrame &f)
{
    RigExecPointFrame out;
    for (size_t i = 0; i < 4; ++i)
        out.points[i] = GfVec3d(f.points[i][0], f.points[i][1], f.points[i][2]);
    out.flags = f.flags;
    return out;
}

/// Resizes the array \p value of \p type to \p count elements of
/// \p size bytes, its first in \p data; false when \p type holds no such
/// array.
template <class T>
bool
Elements(const RigExecTypeDesc &type, void *value, size_t count, T **data)
{
    if (!type.element || !type.resize || !type.data ||
        RigExecRegistry::Get().TypeAt(type.element->Index()).desc->size !=
            sizeof(T))
        return false;
    type.resize(value, count);
    *data = static_cast<T *>(type.data(value));
    return *data || !count;
}

/// The frame each candidate position of solver batch \p w holds and
/// whether one is present: its solvers' published outputs, the last in
/// batch order winning a position.
template <class Visit>
void
EachSolverCandidate(const RigExecBakedProgramImpl &B, size_t w, Visit visit)
{
    for (const int si : B.walkSteps[w].batchSolvers) {
        const auto &s = B.solvers[size_t(si)];
        for (size_t k = 0; k < s.outputs.size(); ++k)
            if (s.outPresent[k] && s.outPosition[k] >= 0)
                visit(size_t(s.outPosition[k]), s.outFrames[k]);
    }
}

/// Whether candidate \p pos of commit \p w is present this frame, and its
/// frame in \p frame when it is.
bool
CandidateAt(const RigExecBakedProgramImpl &B, size_t w, size_t pos,
            RigExecPointFrame *frame)
{
    const Commit &commit = B.commits[w];
    if (!commit.solverOutput) {
        if (pos < commit.frames.size())
            *frame = commit.frames[pos];
        return pos < commit.present.size() && commit.present[pos];
    }
    bool present = false;
    EachSolverCandidate(B, w, [&](size_t at, const RigExecPointFrame &f) {
        if (at == pos) {
            *frame = f;
            present = true;
        }
    });
    return present;
}

/// The candidate position whose write descendant \p k of \p commit
/// carries, or -1 when it carries a version written before the commit.
int
CarriedCandidate(const Commit &commit, size_t k, bool base)
{
    const std::vector<uint32_t> &writes =
        base ? commit.slotBaseWrites : commit.slotWrites;
    const std::vector<uint32_t> &carry =
        base ? commit.descendantBaseCarry : commit.descendantCarry;
    for (size_t pos = 0; pos < commit.slots.size() && pos < writes.size();
         ++pos)
        if (commit.slots[pos] == commit.propagate[k].first &&
            writes[pos] == carry[k])
            return int(pos);
    return -1;
}

} // namespace

bool
RigExecBakedAddPoseCommitOp(const RigExecBakedProgramImpl &B,
                            RigExecRigDescBuilder *builder,
                            eval::OpStepLayout *layout, size_t o,
                            std::string *error)
{
    using namespace rigExecOps;
    using L = eval::OpCommitList;
    using T = eval::OpOutputTarget;
    const uint32_t w = uint32_t(B.opNodeOrigins[o].object);
    const Commit &commit = B.commits[w];
    const size_t slots = commit.slots.size();
    const size_t pairs = commit.propagate.size();
    PoseCommitPayload p;
    p.slotCount = slots;
    p.solverOutput = commit.solverOutput;
    for (size_t k = 0; k < pairs; ++k) {
        bool known = true;
        const bool blocked = RigExecCommitParentBlocked(
            commit.propagate[k].first, commit.propagate[k].second,
            [&](int slot, bool *authored, int *next) {
                if (slot < 0 || size_t(slot) >= B.propParent.size())
                    return false;
                *authored = B.parentSpaceAuthored[size_t(slot)] != 0;
                *next = B.propParent[size_t(slot)];
                return true;
            },
            &known);
        if (!known)
            return Fail(error, commit.moverPathText +
                                   ": a propagated descendant has no "
                                   "parent chain");
        p.closest.push_back(commit.closestPos[k]);
        p.blocked.push_back(blocked ? 1 : 0);
    }
    const RigExecNodeId node = builder->AddOp(PoseCommit(), &p);
    if (node != o)
        return Fail(error, "the pose commit of node " + std::to_string(o) +
                               " was added as node " + std::to_string(node));
    const auto look = [&](const RigExecPortHandle &port,
                          eval::OpLookupSource source, int32_t part) {
        builder->BindInput(node, port);
        layout->hostInputs.push_back({node, port.Index(),
                                      eval::OpHostSource::Lookup,
                                      uint32_t(layout->lookups.size())});
        layout->lookups.push_back({uint8_t(source), w, part, 0});
    };
    const auto versions = [&](const RigExecPortHandle &port, L list) {
        look(port, eval::OpLookupSource::CommitVersions, int32_t(list));
    };
    const auto store = [&](const RigExecPortHandle &port, T target) {
        layout->hostOutputs.push_back({node, port.Index(), uint8_t(target), w});
    };
    look(PoseCommitCandidates(), eval::OpLookupSource::CommitCandidates, 0);
    look(PoseCommitPresent(), eval::OpLookupSource::CommitCandidates, 1);
    versions(PoseCommitEntering(), L::Entering);
    versions(PoseCommitCarry(), L::Carry);
    if (commit.solverOutput)
        versions(PoseCommitBaseCarry(), L::BaseCarry);
    if (pairs) {
        versions(PoseCommitCurrent(), L::Current);
        versions(PoseCommitClosest(), L::Closest);
        versions(PoseCommitDescendantCarry(), L::DescendantCarry);
        if (commit.solverOutput)
            versions(PoseCommitDescendantBaseCarry(), L::DescendantBaseCarry);
    }
    builder->BindOutput(node, PoseCommitSlots());
    builder->BindOutput(node, PoseCommitDescendants());
    store(PoseCommitSlots(), T::CommitSlots);
    store(PoseCommitDescendants(), T::CommitDescendants);
    if (commit.solverOutput) {
        builder->BindOutput(node, PoseCommitBaseSlots());
        builder->BindOutput(node, PoseCommitBaseDescendants());
        store(PoseCommitBaseSlots(), T::CommitBaseSlots);
        store(PoseCommitBaseDescendants(), T::CommitBaseDescendants);
        store(PoseCommitPresent(), T::CommitPresent);
    }
    return true;
}

bool
RigExecBakedLookUpCommit(const RigExecBakedProgramImpl &B,
                         const eval::OpLookup &lookup,
                         const RigExecTypeDesc &type, void *value,
                         std::string *error)
{
    if (lookup.object >= B.commits.size())
        return Fail(error, "commit " + std::to_string(lookup.object) +
                               " is no commit of the program");
    const Commit &commit = B.commits[lookup.object];
    const size_t slots = commit.slots.size();
    if (lookup.source == uint8_t(eval::OpLookupSource::CommitVersions)) {
        bool base = false;
        const std::vector<uint32_t> *versions =
            Versions(commit, lookup.part, &base);
        const std::vector<RigExecPointFrame> &store = base ? B.base : B.fin;
        rigExecRig::PointFrame *out = nullptr;
        if (!versions || !Elements(type, value, versions->size(), &out))
            return Fail(error, "commit " + std::to_string(lookup.object) +
                                   " names no frame list " +
                                   std::to_string(lookup.part));
        const bool descendants =
            lookup.part == int32_t(eval::OpCommitList::DescendantCarry) ||
            lookup.part == int32_t(eval::OpCommitList::DescendantBaseCarry);
        for (size_t i = 0; i < versions->size(); ++i) {
            // A descendant that is also a candidate carries what the commit
            // writes for that candidate: its frame when present, else the
            // candidate's own carry.
            const int pos =
                descendants ? CarriedCandidate(commit, i, base) : -1;
            RigExecPointFrame candidate;
            if (pos >= 0 &&
                CandidateAt(B, lookup.object, size_t(pos), &candidate)) {
                out[i] = ToOp(candidate);
                continue;
            }
            const uint32_t version =
                pos >= 0 ? (base ? commit.slotBaseCarry
                                 : commit.slotCarry)[size_t(pos)]
                         : (*versions)[i];
            if (version >= store.size())
                return Fail(error, "pose version " + std::to_string(version) +
                                       " is no pose frame of the program");
            out[i] = ToOp(store[version]);
        }
        return true;
    }
    if (lookup.part == 0) {
        rigExecRig::PointFrame *out = nullptr;
        if (!Elements(type, value, slots, &out))
            return Fail(error, "the candidates of commit " +
                                   std::to_string(lookup.object) +
                                   " are no frame list");
        if (!commit.solverOutput) {
            for (size_t pos = 0; pos < slots; ++pos)
                out[pos] = ToOp(commit.frames[pos]);
            return true;
        }
        for (size_t pos = 0; pos < slots; ++pos)
            out[pos] = rigExecRig::PointFrame();
        EachSolverCandidate(B, lookup.object,
                            [&](size_t pos, const RigExecPointFrame &frame) {
                                if (pos < slots)
                                    out[pos] = ToOp(frame);
                            });
        return true;
    }
    uint8_t *out = nullptr;
    if (lookup.part != 1 || !Elements(type, value, slots, &out))
        return Fail(error, "commit " + std::to_string(lookup.object) +
                               " names no candidate list " +
                               std::to_string(lookup.part));
    for (size_t pos = 0; pos < slots; ++pos)
        out[pos] = commit.solverOutput ? 0 : uint8_t(commit.present[pos] != 0);
    if (commit.solverOutput)
        EachSolverCandidate(B, lookup.object,
                            [&](size_t pos, const RigExecPointFrame &) {
                                if (pos < slots)
                                    out[pos] = 1;
                            });
    return true;
}

bool
RigExecBakedStoreCommit(RigExecBakedProgramImpl &B,
                        const eval::OpHostOutput &output, const void *value,
                        std::string *error)
{
    if (output.object >= B.commits.size())
        return Fail(error, "commit " + std::to_string(output.object) +
                               " is no commit of the program");
    Commit &commit = B.commits[output.object];
    if (output.target == uint8_t(eval::OpOutputTarget::CommitPresent)) {
        const Flags &flags = *static_cast<const Flags *>(value);
        if (flags.Size() != commit.present.size())
            return Fail(error,
                        "commit " + std::to_string(output.object) + " has " +
                            std::to_string(commit.present.size()) +
                            " candidates, not " + std::to_string(flags.Size()));
        bool any = false;
        for (size_t pos = 0; pos < flags.Size(); ++pos) {
            commit.present[pos] = flags[pos] ? 1 : 0;
            any = any || flags[pos];
        }
        commit.abandoned = !any;
        return true;
    }
    bool base = false;
    const std::vector<uint32_t> *versions =
        Writes(commit, output.target, &base);
    std::vector<RigExecPointFrame> &store = base ? B.base : B.fin;
    const Frames &frames = *static_cast<const Frames *>(value);
    if (!versions || frames.Size() != versions->size())
        return Fail(error, "commit " + std::to_string(output.object) +
                               " writes no " + std::to_string(frames.Size()) +
                               " frames at target " +
                               std::to_string(output.target));
    for (size_t i = 0; i < frames.Size(); ++i) {
        if ((*versions)[i] >= store.size())
            return Fail(error, "pose version " +
                                   std::to_string((*versions)[i]) +
                                   " is no pose frame of the program");
        store[(*versions)[i]] = FromOp(frames[i]);
    }
    return true;
}

std::string
RigExecBakedCommitFailure(const RigExecBakedProgramImpl &B, int commit,
                          const std::string &error)
{
    if (commit < 0 || size_t(commit) >= B.commits.size())
        return error;
    const Commit &c = B.commits[size_t(commit)];
    std::string mover = c.moverPathText;
    if (mover.empty() && size_t(commit) < B.walkSteps.size() &&
        !B.walkSteps[size_t(commit)].batchSolvers.empty())
        mover =
            B.solvers[size_t(B.walkSteps[size_t(commit)].batchSolvers.front())]
                .pathText;
    return RigExecCommitFailureText(mover, error, [&](size_t k) {
        return k < c.propagate.size() &&
                       size_t(c.propagate[k].first) < B.pathTexts->size()
                   ? (*B.pathTexts)[size_t(c.propagate[k].first)]
                   : "descendant " + std::to_string(k);
    });
}

} // namespace rigExec
