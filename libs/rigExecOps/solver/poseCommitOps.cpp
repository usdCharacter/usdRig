#include "rigExecOps/solver/poseCommitOps.h"

#include "rigExecOps/fieldIo.h"
#include "rigExecOps/internalOps.h"
#include "rigExecOps/opIo.h"
#include "rigExecEvalCore/pose.h"
#include "rigExecMath/poseCommitKernel.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecRigTypes/rigTypes.h"
#include "rigExecRuntime/poseMath.h"

#include <iterator>
#include <string>
#include <vector>

namespace rigExecOps {

RIGEXEC_DEFINE_OP_HANDLE(PoseCommit)

RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitPresent)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitEntering)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitCarry)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitBaseCarry)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitCurrent)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitClosest)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitDescendantCarry)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitDescendantBaseCarry)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitSlots)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitDescendants)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitBaseSlots)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitBaseDescendants)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitCandidate)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitSources)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitStatus)
RIGEXEC_DEFINE_PORT_HANDLE(PoseCommitPresence)

namespace {

using rigExec::RigExecOpBatch;
using rigExec::RigExecOpIo;
using rigExecRig::PointFrame;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using Frames = RigExecArray<PointFrame>;
using Flags = RigExecArray<uint8_t>;
using PoseMath = rigExec::RrPoseMathT<PointFrame>;
using Math = rigExec::eval::PosePropagateMath<PoseMath>;
using Outcome = rigExec::RigExecPosePropagateOutcome;
using namespace opIo;
using namespace fieldIo;

/// PoseCommit's declared fields, in saved order.
enum CommitField : uint32_t {
    kSlotCount,
    kPairCount,
    kClosest,
    kBlockedCount,
    kBlocked,
    kSolverOutput,
    kElementFrames,
    kEntrySourceCount,
    kEntrySources,
    kEntryFrameCount,
    kEntryFrames,
    kEntryCandidateCount,
    kEntryCandidates,
    kCarriedCount,
    kCarried,
    kBaseCarriedCount,
    kBaseCarried
};

/// What SingleChainIkResult and HarmonicsResult call a solved frame.
constexpr uint32_t kSolved = 1;

/// The frame of element \p index of solver frames \p source, as
/// RigExecExtractElementFrame reads it: a valid, non-degenerate element's
/// out-space, any other flagged degenerate.
PointFrame
ElementFrame(const rigExecRig::PointFrameArray &source, size_t index)
{
    using rigExecRig::kPointFrameDegenerate;
    using rigExecRig::kPointFrameValid;
    PointFrame out;
    const PointFrame &f = source.frames[index];
    if (!(f.flags & kPointFrameValid) || (f.flags & kPointFrameDegenerate)) {
        out.flags = kPointFrameDegenerate;
        return out;
    }
    const Mat4d m = rigExec::RigExecElementOutSpaceT<Mat4d, rigExecStd::Vec4d>(
        f.points, index < source.rests.Size() ? &source.rests[index] : nullptr);
    out.flags = rigExec::RigExecMatrixLandmarksT(m, &out.points)
                    ? kPointFrameValid
                    : kPointFrameDegenerate;
    return out;
}

constexpr const char *kOp = "PoseCommit";

/// Whether \p port is fed with \p count frames, as far as validate knows.
bool
FedWith(const rigExec::RigExecOpCheck &check,
        const rigExec::RigExecPortHandle &port, size_t count)
{
    return Fed(check, port) && SameCount(CheckElements(check, port), count);
}

bool
CheckPoseCommit(const rigExec::RigExecOpCheck &check, std::string *error)
{
    const size_t slots = ValueAt<uint64_t>(check.batch, kSlotCount, check.k);
    const auto closest = ListAt<int32_t>(check.batch, kClosest, check.k);
    const size_t pairs = closest.Size();
    const bool solver = FlagAt(check.batch, kSolverOutput, check.k);
    if (ListAt<uint8_t>(check.batch, kBlocked, check.k).Size() != pairs)
        return Fail(error, kOp, "needs one blocked flag per descendant");
    for (size_t k = 0; k < pairs; ++k)
        if (closest[k] < -1 || closest[k] >= int64_t(slots))
            return Fail(error, kOp, "names a closest candidate it lacks");
    const auto entrySources =
        ListAt<int32_t>(check.batch, kEntrySources, check.k);
    const auto entryFrames =
        ListAt<int32_t>(check.batch, kEntryFrames, check.k);
    const auto entryCandidates =
        ListAt<int32_t>(check.batch, kEntryCandidates, check.k);
    const size_t sources = CheckElements(check, PoseCommitSources());
    const bool gathered = sources != 0 && sources != rigExec::kRigExecPortUnfed;
    if (entryFrames.Size() != entrySources.Size() ||
        entryCandidates.Size() != entrySources.Size() ||
        (!gathered && entrySources.Size()))
        return Fail(error, kOp,
                    "needs a source, frame and candidate per entry");
    for (size_t e = 0; e < entrySources.Size(); ++e)
        if (entrySources[e] < 0 || entryFrames[e] < 0 ||
            entryCandidates[e] < 0 || entryCandidates[e] >= int64_t(slots) ||
            (sources < rigExec::kRigExecPortUnfed &&
             size_t(entrySources[e]) >= sources))
            return Fail(error, kOp, "names a source or candidate it lacks");
    const auto carried = ListAt<int32_t>(check.batch, kCarried, check.k);
    const auto baseCarried =
        ListAt<int32_t>(check.batch, kBaseCarried, check.k);
    if (carried.Size() != pairs || baseCarried.Size() != pairs)
        return Fail(error, kOp, "needs one carried candidate per descendant");
    for (size_t k = 0; k < pairs; ++k)
        if (carried[k] < -1 || carried[k] >= int64_t(slots) ||
            baseCarried[k] < -1 || baseCarried[k] >= int64_t(slots) ||
            (!solver && baseCarried[k] != -1))
            return Fail(error, kOp, "carries a candidate it lacks");
    const size_t each = CheckElements(check, PoseCommitCandidate());
    if ((gathered ? each != 0 && each < rigExec::kRigExecPortUnfed
                  : !SameCount(each, slots)) ||
        !FedWith(check, PoseCommitEntering(), slots) ||
        !FedWith(check, PoseCommitCarry(), slots) ||
        !SameCount(CheckElements(check, PoseCommitPresent()), slots))
        return Fail(error, kOp,
                    "needs one candidate, entering frame, carry and present "
                    "flag per joint");
    if (solver != Fed(check, PoseCommitBaseCarry()) ||
        (solver && !FedWith(check, PoseCommitBaseCarry(), slots)))
        return Fail(error, kOp,
                    "needs one base carry per joint exactly when it commits "
                    "a solver");
    const bool descendants = pairs > 0;
    for (const rigExec::RigExecPortHandle *port :
         {&PoseCommitCurrent(), &PoseCommitClosest(),
          &PoseCommitDescendantCarry()})
        if (descendants != Fed(check, *port) ||
            (descendants && !FedWith(check, *port, pairs)))
            return Fail(error, kOp,
                        "needs one current, closest and carry frame per "
                        "descendant");
    if ((solver && descendants) !=
            Fed(check, PoseCommitDescendantBaseCarry()) ||
        (solver && descendants &&
         !FedWith(check, PoseCommitDescendantBaseCarry(), pairs)))
        return Fail(error, kOp,
                    "needs one base carry per descendant exactly when it "
                    "commits a solver");
    return true;
}

bool
PoseCommitOne(const RigExecOpBatch &batch, size_t k, std::string *error)
{
    using rigExecRig::PointFrameArray;
    const RigExecOpIo io = rigExec::RigExecBatchInstance(batch, k);
    const size_t slots = ValueAt<uint64_t>(batch, kSlotCount, k);
    const auto closest = ListAt<int32_t>(batch, kClosest, k);
    const auto blocked = ListAt<uint8_t>(batch, kBlocked, k);
    const size_t pairs = closest.Size();
    const bool solver = FlagAt(batch, kSolverOutput, k);
    const bool elements = FlagAt(batch, kElementFrames, k);
    const auto entrySources = ListAt<int32_t>(batch, kEntrySources, k);
    const auto entryFrames = ListAt<int32_t>(batch, kEntryFrames, k);
    const auto entryCandidates = ListAt<int32_t>(batch, kEntryCandidates, k);
    const auto carried = ListAt<int32_t>(batch, kCarried, k);
    const auto baseCarried = ListAt<int32_t>(batch, kBaseCarried, k);
    const InList<PointFrame> each(io, PoseCommitCandidate());
    const InList<PointFrameArray> sources(io, PoseCommitSources());
    const bool gathered = sources.Size() != 0;
    const uint32_t *status = In<uint32_t>(io, PoseCommitStatus());
    const bool solved = !status || *status == kSolved;
    const Frames &entering = *In<Frames>(io, PoseCommitEntering());
    const Frames &carry = *In<Frames>(io, PoseCommitCarry());
    const Frames *baseCarry = In<Frames>(io, PoseCommitBaseCarry());
    const Flags *presentIn = In<Flags>(io, PoseCommitPresent());
    const Frames *current = In<Frames>(io, PoseCommitCurrent());
    const Frames *closestIn = In<Frames>(io, PoseCommitClosest());
    const Frames *descendantCarry = In<Frames>(io, PoseCommitDescendantCarry());
    const Frames *descendantBaseCarry =
        In<Frames>(io, PoseCommitDescendantBaseCarry());
    const auto fits = [](const Frames *frames, size_t count) {
        return frames && frames->Size() == count;
    };
    if ((!gathered && each.Size() != slots) || entering.Size() != slots ||
        carry.Size() != slots || (presentIn && presentIn->Size() != slots) ||
        carried.Size() != pairs || baseCarried.Size() != pairs ||
        (solver && !fits(baseCarry, slots)) ||
        (pairs && (!fits(current, pairs) || !fits(closestIn, pairs) ||
                   !fits(descendantCarry, pairs) ||
                   (solver && !fits(descendantBaseCarry, pairs)))))
        return Fail(error, kOp, "needs one frame per joint and descendant");
    // The candidates are gathered into Slots, where a present one stays.
    Frames &outSlots = Out<Frames>(io, PoseCommitSlots());
    Frames &outDescendants = Out<Frames>(io, PoseCommitDescendants());
    Frames &baseSlots = Out<Frames>(io, PoseCommitBaseSlots());
    Frames &baseDescendants = Out<Frames>(io, PoseCommitBaseDescendants());
    Flags &presence = Out<Flags>(io, PoseCommitPresence());
    outSlots.Resize(slots);
    outDescendants.Resize(pairs);
    baseSlots.Resize(solver ? slots : 0);
    baseDescendants.Resize(solver ? pairs : 0);
    presence.Resize(slots);
    std::vector<char> present(slots, 0);
    if (!gathered) {
        for (size_t pos = 0; pos < slots; ++pos) {
            outSlots[pos] = each[pos];
            present[pos] =
                solved && !(each[pos].flags &
                            rigExec::eval::pose_ops::kPoseFrameDegenerate);
        }
    } else {
        for (size_t e = 0; solved && e < entrySources.Size(); ++e) {
            const size_t from = size_t(entrySources[e]);
            const size_t index = size_t(entryFrames[e]);
            const size_t pos = size_t(entryCandidates[e]);
            if (from >= sources.Size() || pos >= slots)
                return Fail(error, kOp, "names a source or candidate it lacks");
            const PointFrameArray &source = sources[from];
            if (index >= source.frames.Size()) {
                if (elements)
                    continue;
                return Fail(error, kOp,
                            "needs a frame for every joint it commits");
            }
            outSlots[pos] =
                elements ? ElementFrame(source, index) : source.frames[index];
            present[pos] = 1;
        }
    }
    for (size_t pos = 0; presentIn && pos < slots; ++pos)
        if (!(*presentIn)[pos])
            present[pos] = 0;
    bool any = false;
    for (const char p : present)
        any = any || p;
    std::vector<uint8_t> outcome(pairs, uint8_t(Outcome::Skipped));
    std::vector<PointFrame> staged(pairs);
    if (any) {
        std::vector<Mat4d> deltas(slots, PoseMath::Identity4());
        std::vector<char> deltaOk(slots, 0);
        for (size_t pos = 0; pos < slots; ++pos)
            if (present[pos])
                deltaOk[pos] = char(rigExec::RigExecMeasureCommitDelta<Math>(
                    entering[pos].points, outSlots[pos].points, &deltas[pos]));
        for (size_t d = 0; d < pairs; ++d) {
            const int pos = closest[d];
            if (rigExec::RigExecCommitPairSkipped(blocked[d] != 0, pos,
                                                  present.data(), slots))
                continue;
            outcome[d] = uint8_t(rigExec::RigExecStageCommitPair<Math>(
                (*current)[d], (*closestIn)[d], outSlots[size_t(pos)],
                deltas[size_t(pos)], deltaOk[size_t(pos)] == 1, solver,
                &staged[d]));
        }
        const size_t failed =
            rigExec::RigExecFirstCommitFailure(outcome.data(), pairs);
        if (failed < pairs) {
            const std::string which = "descendant " + std::to_string(failed);
            switch (Outcome(outcome[failed])) {
            case Outcome::UnusableDescendant:
                return Fail(error, kOp,
                            "could not propagate its pose revision through " +
                                which);
            case Outcome::SingularDelta:
                return Fail(error, kOp,
                            "produced a singular hierarchy delta for " + which);
            default:
                return Fail(error, kOp,
                            "produced an invalid descendant frame for " +
                                which);
            }
        }
    }
    for (size_t pos = 0; pos < slots; ++pos) {
        presence[pos] = uint8_t(present[pos] ? 1 : 0);
        if (solver)
            baseSlots[pos] = present[pos] ? outSlots[pos] : (*baseCarry)[pos];
        if (!present[pos])
            outSlots[pos] = carry[pos];
    }
    const auto kept = [&](int32_t pos, const PointFrame &otherwise) {
        return pos >= 0 && present[size_t(pos)] ? outSlots[size_t(pos)]
                                                : otherwise;
    };
    for (size_t d = 0; d < pairs; ++d) {
        const bool moved = Outcome(outcome[d]) == Outcome::Staged;
        outDescendants[d] =
            moved ? staged[d] : kept(carried[d], (*descendantCarry)[d]);
        if (solver)
            baseDescendants[d] =
                moved ? staged[d]
                      : kept(baseCarried[d], (*descendantBaseCarry)[d]);
    }
    return true;
}

bool
RunPoseCommit(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    return EachInstance(batch, failed, [&](size_t k) {
        return PoseCommitOne(batch, k, error);
    });
}

using rigExec::kRigExecPortArray;
using rigExec::kRigExecPortInput;
using rigExec::kRigExecPortOptional;
using rigExec::kRigExecPortOutput;
constexpr uint32_t kOptional = kRigExecPortInput | kRigExecPortOptional;
constexpr auto kField = rigExec::RigExecOutputBound::Field;

const rigExec::RigExecPortDesc kPorts[] = {
    {&PoseCommitPresent(), &rigExecStd::BoolArray(), kOptional, "present"},
    {&PoseCommitEntering(), &rigExecRig::PointFrameList(), kRigExecPortInput,
     "entering"},
    {&PoseCommitCarry(), &rigExecRig::PointFrameList(), kRigExecPortInput,
     "carry"},
    {&PoseCommitBaseCarry(), &rigExecRig::PointFrameList(), kOptional,
     "baseCarry"},
    {&PoseCommitCurrent(), &rigExecRig::PointFrameList(), kOptional, "current"},
    {&PoseCommitClosest(), &rigExecRig::PointFrameList(), kOptional, "closest"},
    {&PoseCommitDescendantCarry(), &rigExecRig::PointFrameList(), kOptional,
     "descendantCarry"},
    {&PoseCommitDescendantBaseCarry(), &rigExecRig::PointFrameList(), kOptional,
     "descendantBaseCarry"},
    {&PoseCommitSlots(), &rigExecRig::PointFrameList(), kRigExecPortOutput,
     "slots", kField, nullptr, nullptr, kSlotCount},
    {&PoseCommitDescendants(), &rigExecRig::PointFrameList(),
     kRigExecPortOutput, "descendants", kField, nullptr, nullptr, kPairCount},
    {&PoseCommitBaseSlots(), &rigExecRig::PointFrameList(), kRigExecPortOutput,
     "baseSlots", kField, nullptr, nullptr, kSlotCount},
    {&PoseCommitBaseDescendants(), &rigExecRig::PointFrameList(),
     kRigExecPortOutput, "baseDescendants", kField, nullptr, nullptr,
     kPairCount},
    {&PoseCommitCandidate(), &rigExecRig::PointFrameType(),
     kOptional | kRigExecPortArray, "candidate"},
    {&PoseCommitSources(), &rigExecRig::PointFrameArrayType(),
     kOptional | kRigExecPortArray, "sources"},
    {&PoseCommitStatus(), &rigExecStd::UIntType(), kOptional, "status"},
    {&PoseCommitPresence(), &rigExecStd::BoolArray(), kRigExecPortOutput,
     "presence", kField, nullptr, nullptr, kSlotCount},
};

void
CommitValues(const void *payload, RigExecFieldValues &out)
{
    const auto &p = *static_cast<const PoseCommitPayload *>(payload);
    out.Set(kSlotCount, p.slotCount);
    SetList(out, kPairCount, p.closest);
    SetList(out, kBlockedCount, p.blocked);
    SetFlag(out, kSolverOutput, p.solverOutput);
    SetFlag(out, kElementFrames, p.elementFrames);
    SetList(out, kEntrySourceCount, p.entrySources);
    SetList(out, kEntryFrameCount, p.entryFrames);
    SetList(out, kEntryCandidateCount, p.entryCandidates);
    SetList(out, kCarriedCount, p.carried);
    SetList(out, kBaseCarriedCount, p.baseCarried);
}

const rigExec::RigExecFieldDesc kFields[] = {
    Value("slotCount", rigExecStd::UInt64Type()),
    Count("pairCount"),
    Items("closest", rigExecStd::IntType(), kPairCount),
    Count("blockedCount"),
    Items("blocked", rigExecStd::BoolType(), kBlockedCount),
    Flag("solverOutput"),
    Flag("elementFrames"),
    Count("entrySourceCount"),
    Items("entrySources", rigExecStd::IntType(), kEntrySourceCount),
    Count("entryFrameCount"),
    Items("entryFrames", rigExecStd::IntType(), kEntryFrameCount),
    Count("entryCandidateCount"),
    Items("entryCandidates", rigExecStd::IntType(), kEntryCandidateCount),
    Count("carriedCount"),
    Items("carried", rigExecStd::IntType(), kCarriedCount),
    Count("baseCarriedCount"),
    Items("baseCarried", rigExecStd::IntType(), kBaseCarriedCount),
};

const rigExec::RigExecOpDesc kDesc = [] {
    rigExec::RigExecOpDesc d;
    d.nameSpace = "rig";
    d.name = "PoseCommit";
    d.schema = &internalOps::kPoseCommit;
    d.ports = kPorts;
    d.portCount = std::size(kPorts);
    d.runBatch = &RunPoseCommit;
    d.validate = &CheckPoseCommit;
    d.capabilities = rigExec::kRigExecOpBakeable | rigExec::kRigExecOpFreezable;
    d.payloadFields = kFields;
    d.payloadFieldCount = std::size(kFields);
    d.payloadVersion = 2;
    d.payloadValues = &CommitValues;
    return d;
}();

} // namespace

void
RegisterPoseCommitOps(rigExec::RigExecOpRegistrar &registrar)
{
    registrar.Add(PoseCommit(), kDesc);
}

} // namespace rigExecOps
