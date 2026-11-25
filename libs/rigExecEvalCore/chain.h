// The revision-chain step bodies the baked program and .rigexec playback
// share: ChainInputs, InfluenceFold, RevisionStatic, RevisionChunk,
// RevisionFuse, ChainStatus and Derived, plus the paired skip switch and
// the chain helpers both front ends used to carry twice -- base-group
// publication, group resolution, point-version resolution, partition
// adoption, point-bit equality and the moved-points copy
// (docs/specs/eval-core-parity.md). One definition per body, so a
// second front end cannot drift into a different answer. Adapter binding
// stays with each front end: the core classifies roles and reads Build
// facts from its leaves, playback binds the partition and roles the file
// recorded; neither front end keeps a second decision. ClassifyRevision,
// GateDefault and GateRevision stay core-adapter Build functions and are
// never ported here; the gate predicate itself is gate.h's, not this
// header's. The weight-field overlay publication and the revisionsBuilt
// counting stay adapter-local, as host.h records. A GPU chain's Derived
// deferral to the publication boundary and its GPU fuse stay with the
// baked adapter around the shared bodies; discardable probes stay native.
//
// Each host borrows compiled partitions and packet views for one scheduled
// step; state publishes only through the graph-owned output slots.
#ifndef RIGEXEC_EVAL_CORE_CHAIN_H
#define RIGEXEC_EVAL_CORE_CHAIN_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>

#include "rigExecEvalCore/envelope.h"
#include "rigExecEvalCore/host.h"
#include "rigExecEvalCore/kinds.h"
#include "rigExecEvalCore/skin.h"
#include "rigExecEvalCore/step.h"
#include "rigExecMath/blendRangeKernel.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/pointBlocks.h"
#include "rigExecMath/pointRanges.h"
#include "rigExecMath/rangeRunner.h"

namespace rigExec {
namespace eval {

/// StepResult details for the chain steps: which binding an Invalid
/// names. Adapters map these to their own error or diagnostic text.
enum ChainStepDetail : int32_t {
    kChainDetailNone = kNoStepDetail,
    kChainDetailNoChain = 1,
    kChainDetailNoRevision = 2,
    kChainDetailNoChunk = 3,
    kChainDetailFold = 4,
    kChainDetailChunk = 5,
    kChainDetailNoDerived = 6
};

/// The op ordinals the chain bodies dispatch on, in wire order.
inline constexpr int kChainOpMatrix = int(fb::RevisionOp::Matrix);
inline constexpr int kChainOpSkin = int(fb::RevisionOp::Skin);
inline constexpr int kChainOpBlendShape = int(fb::RevisionOp::BlendShape);
inline constexpr int kChainOpSmooth = int(fb::RevisionOp::Smooth);
inline constexpr int kChainOpLattice = int(fb::RevisionOp::Lattice);
inline constexpr int kChainOpWire = int(fb::RevisionOp::Wire);
inline constexpr int kChainOpDeltaMush = int(fb::RevisionOp::DeltaMush);

/// Acceptance ordinals both front ends already spell
/// (RigExecRevisionAcceptance and its playback twin).
inline constexpr int kChainAcceptanceRefuses = 0;
inline constexpr int kChainAcceptanceApplies = 1;
inline constexpr int kChainAcceptanceDeferred = 2;

/// A chain's base binding as ChainInputs reads it: the sampled base and
/// its availability, the held base, and the publication state the step
/// moves. Borrowed pointers into the adapter's chain; valid for the call.
template <class Point, class BaseContainer, class ResultContainer>
struct ChainInputsAccessT {
    bool sampledHave = false;
    const BaseContainer *sampled = nullptr;
    BaseContainer *last = nullptr;
    bool *haveBase = nullptr;
    bool *haveResult = nullptr;
    ResultContainer *result = nullptr;
    uint64_t *resultVersion = nullptr;
    uint64_t *baseVersion = nullptr;
    bool *scheduleDirty = nullptr;
    size_t revisionCount = 0;
};

/// The fields one group step reads and writes, as the adapter's revision
/// holds them. Borrowed; valid for the call.
template <class Point> struct GroupStepAccessT {
    RigExecGroupState<Point> *state = nullptr;
    bool rangeSetAside = false;
    bool applies = false;
    bool skin = false;
    bool blendChunked = false;
    bool partitionStale = false;
    bool fullStrength = false;
    size_t precedingCount = 0;
    size_t chainPointCount = 0;
    size_t chunksSize = 0;
};

/// The fields one fuse reads and writes, as the adapter's revision holds
/// them. Borrowed; valid for the call.
template <class Point> struct FuseRevisionAccessT {
    bool *ran = nullptr;
    int *currentSource = nullptr;
    std::vector<Point> *output = nullptr;
    std::vector<Point> *stagingOutput = nullptr;
    bool *stagingFresh = nullptr;
    std::vector<Point> *passedPoints = nullptr;
    uint64_t *doneVersion = nullptr;
    std::vector<RigExecGroupState<Point>> *groups = nullptr;
    std::vector<RigExecGroupSource> *groupIds = nullptr;
    std::vector<char> *groupWritten = nullptr;
    uint32_t *rangeRefusals = nullptr;
    size_t precedingCount = 0;
};

/// The publication state ChainStatus reads and writes. Borrowed; valid
/// for the call.
template <class Point> struct ChainStatusAccessT {
    bool *haveResult = nullptr;
    uint64_t *resultVersion = nullptr;
    std::vector<RigExecGroupSource> *resultIds = nullptr;
};

/// Point-bit equality, one definition: same count and same bytes, the
/// rule every content version in the chain follows.
template <class Point>
inline bool
PointsBitsEqualBody(const Point *a, size_t na, const Point *b, size_t nb)
{
    return RigExecPointsBitsEqual(a, na, b, nb);
}

/// The moved-points copy, one definition: \p output takes \p staging's
/// points, copying only the blocks whose bytes differ, and returns
/// whether any did (a size change included).
template <class Point>
bool
CopyMovedPointsBody(const std::vector<Point> &staging,
                    std::vector<Point> *output)
{
    if (output->size() != staging.size()) {
        output->assign(staging.begin(), staging.end());
        return true;
    }
    constexpr size_t kBlock = 1024;
    bool moved = false;
    for (size_t begin = 0; begin < staging.size(); begin += kBlock) {
        const size_t count = std::min(kBlock, staging.size() - begin);
        if (std::memcmp(staging.data() + begin, output->data() + begin,
                        count * sizeof(Point)) != 0) {
            std::copy(staging.begin() + long(begin),
                      staging.begin() + long(begin + count),
                      output->begin() + long(begin));
            moved = true;
        }
    }
    return moved;
}

/// The whole-array skin a fuse runs when its partition went stale, one
/// definition: the layout and envelope checks, then the host's whole
/// skin range into \p fused.
template <class Host>
bool
FuseWholeRevisionBody(Host &host, size_t chain, size_t revision,
                      const typename Host::Math::Vec3f *points, size_t count,
                      std::vector<typename Host::Math::Vec3f> *fused)
{
    if (!host.state().revisionLayoutUsableValue(chain, revision) ||
        !host.state().revisionEnvelopeOkValue(chain, revision) ||
        count != host.state().fusePrecedingCount(chain, revision)) {
        return false;
    }
    fused->resize(count);
    return host.state().skinWholeRange(chain, revision, points, count, fused);
}

/// Base-group publication, one definition: the chain base as groups,
/// one immutable owner sharing the array's buffer and each base group a
/// ref into it, so publishing the base copies no point. Each group's
/// version moves exactly when its bytes differ from the ones it last
/// published, or on its first publication since a reset. While the
/// bytes stand (\p moved false and an owner already held) nothing is
/// rebuilt. \p makeOwner copies the base into a fresh shared owner;
/// \p ownerData and \p ownerCount read the copy back.
template <class Point, class Owner, class MakeOwner, class OwnerData,
          class OwnerCount>
void
PublishBaseGroupsBody(const std::vector<int> &bounds, bool moved,
                      std::vector<RigExecGroupState<Point>> &baseGroups,
                      std::shared_ptr<Owner> &baseOwner, MakeOwner &&makeOwner,
                      OwnerData &&ownerData, OwnerCount &&ownerCount)
{
    if (bounds.size() < 2) {
        return;
    }
    const size_t groups = bounds.size() - 1;
    if (!moved && baseOwner && baseGroups.size() == groups) {
        return;
    }
    baseGroups.resize(groups);
    std::shared_ptr<Owner> owner = makeOwner();
    const Point *const data = ownerData(owner);
    const size_t count = ownerCount(owner);
    for (size_t g = 0; g < groups; ++g) {
        size_t begin = 0, end = 0;
        RigExecPointRangeAt(bounds[g], bounds[g + 1], g + 1 == groups, count,
                            &begin, &end);
        RigExecPointsRef<Point> ref;
        ref.owner = owner;
        ref.data = data + begin;
        ref.count = end - begin;
        RigExecGroupState<Point> &state = baseGroups[g];
        if (!state.ran ||
            !RigExecPointsBitsEqual(ref.data, ref.count, state.published.data,
                                    state.published.count)) {
            ++state.version;
        }
        state.published = std::move(ref);
        state.ran = true;
        state.ok = true;
    }
    baseOwner = std::move(owner);
}

/// The revision index whose slot holds group \p g of point version
/// \p version, or -1 for the base: the last revision before the version
/// that writes g, unless a revision a cycle set aside (which passes the
/// base through) comes first. \p writes(revision, g) and
/// \p setAside(revision) read the adapter's compiled facts.
template <class Writes, class SetAside>
int
ChainGroupWriter(size_t revisionCount, size_t version, size_t g,
                 Writes &&writes, SetAside &&setAside)
{
    for (size_t q = std::min(version, revisionCount); q > 0; --q) {
        const size_t revision = q - 1;
        if (setAside(revision)) {
            return -1;
        }
        if (writes(revision, g)) {
            return int(revision);
        }
    }
    return -1;
}

/// Group \p g of version \p version, resolved through ChainGroupWriter:
/// the writer's published ref and content id, or the base group's.
/// \p baseRef, \p revisionRef, \p baseSource and \p revisionSource read
/// the adapter's groups; the resolution itself is shared.
template <class Point, class Writes, class SetAside, class BaseRef,
          class RevisionRef, class BaseSource, class RevisionSource>
const RigExecPointsRef<Point> &
ChainGroupAt(size_t revisionCount, size_t version, size_t g, Writes &&writes,
             SetAside &&setAside, BaseRef &&baseRef, RevisionRef &&revisionRef,
             BaseSource &&baseSource, RevisionSource &&revisionSource,
             RigExecGroupSource *source, const RigExecPointsRef<Point> &none)
{
    const int writer =
        ChainGroupWriter(revisionCount, version, g, writes, setAside);
    if (writer < 0) {
        if (source) {
            *source = baseSource(g);
        }
        return baseRef(g, none);
    }
    if (source) {
        *source = revisionSource(size_t(writer), g);
    }
    return revisionRef(size_t(writer), g, none);
}

/// Whether a revision applies this run, from what its static step and
/// fold published: a valid packet (for a skin, with a valid fold), an
/// allowing status, and an Applies acceptance. One definition for the
/// group steps and the join on both front ends.
inline bool
RevisionAppliesBody(bool packetValid, bool statusAllows, int acceptance)
{
    return packetValid && statusAllows && acceptance == kChainAcceptanceApplies;
}

/// Partition adoption, one definition over adapter-only inputs: the
/// chunk keys stay Build's whatever the layout says, because each chunk
/// step's reads were declared from its key, so a handle is adopted only
/// when it has the arrays the keys were cut from. \p matches answers
/// that for the candidate layout; \p adopt records it.
template <class Matches, class Adopt>
void
AdoptPartitionBody(bool chunked, bool hasLayout, bool alreadyAdopted,
                   Matches &&matches, Adopt &&adopt)
{
    if (!chunked || !hasLayout || alreadyAdopted) {
        return;
    }
    if (matches()) {
        adopt();
    }
}

/// The candidate walk a declared point binding resolves by: the first
/// candidate whose chain read a base -- the chain's published points
/// for a final read, else that version. \p chainUsable, \p finalAnswer
/// and \p versionAnswer read the adapter's chains; the walk, the
/// have-base gate and the first-answer rule are shared. Answers through
/// \p points and \p count; false answers nothing.
template <class Candidate, class ChainOf, class VersionOf, class ChainUsable,
          class FinalAnswer, class VersionAnswer>
bool
ResolvePointsBody(const std::vector<Candidate> &candidates, bool finalRead,
                  ChainOf &&chainOf, VersionOf &&versionOf,
                  ChainUsable &&chainUsable, FinalAnswer &&finalAnswer,
                  VersionAnswer &&versionAnswer, const void **points,
                  size_t *count)
{
    for (const Candidate &candidate : candidates) {
        const int64_t chain = chainOf(candidate);
        if (chain < 0 || !chainUsable(size_t(chain), finalRead)) {
            continue;
        }
        if (finalRead) {
            if (finalAnswer(size_t(chain), points, count)) {
                return true;
            }
            continue;
        }
        const int64_t version = versionOf(candidate);
        if (version < 0) {
            *points = nullptr;
            *count = 0;
            return true;
        }
        if (versionAnswer(size_t(chain), size_t(version), points, count)) {
            return true;
        }
    }
    return false;
}

/// ChainInputs: bind the chain's sampled base. A chain with no base
/// leaves every revision step to answer Held, and publishes nothing.
/// The size-change reset, the created/schedule accounting, the
/// content-version rules and the base-group publication are shared;
/// the reset and counters themselves stay in the adapter's types.
template <class Host>
StepResult
RunChainInputsStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    size_t chain = 0;
    if (!host.inputs().chainIndexForObject(step.object, &chain)) {
        return {Outcome::Invalid, kChainDetailNoChain};
    }
    auto access = host.state().chainInputs(chain);
    *access.haveBase = access.sampledHave;
    host.state().noteChainBaseDirty(chain, false);
    if (!access.sampledHave) {
        return {Outcome::Held, kNoStepDetail};
    }
    const typename Host::ChainPoints &sampled = *access.sampled;
    if (*access.haveResult && sampled.size() != access.last->size()) {
        if (!access.result->empty()) {
            ++*access.resultVersion;
        }
        *access.haveResult = false;
        access.result->clear();
        *access.scheduleDirty = true;
        for (size_t r = 0; r < access.revisionCount; ++r) {
            host.state().resetChainRevision(chain, r);
            if (host.state().revisionRangeNotAside(chain, r)) {
                host.state().setRevisionSource(chain, r, int(r));
            }
        }
        host.state().resetBaseGroups(chain);
        host.state().clearChainResultIds(chain);
    }
    host.state().countChainCreated(chain);
    if (*access.scheduleDirty && access.revisionCount > 0) {
        host.state().noteChainSchedules(chain);
    }
    *access.scheduleDirty = false;
    const bool moved =
        !host.state().chainPointsEqual(chain, sampled, *access.last);
    const bool baseDirty =
        !*access.haveResult ||
        !host.state().chainBaseValueEqual(chain, sampled, *access.last);
    host.state().noteChainBaseDirty(chain, baseDirty);
    if (moved) {
        ++*access.baseVersion;
    }
    *access.last = sampled;
    host.state().publishChainBaseGroups(chain, moved);
    return {Outcome::Applied, kNoStepDetail};
}

/// InfluenceFold: resolve the declared influences, fold the transform
/// forms, and prove the dual-quaternion palette safe. The fold's slot
/// resolution stays adapter-local; the validity, form-fold and palette
/// decisions are shared, in this order on both front ends.
template <class Host>
StepResult
RunInfluenceFoldStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    size_t chain = 0, revision = 0;
    if (!host.inputs().revisionIndexForObject(step.object, &chain, &revision)) {
        return {Outcome::Invalid, kChainDetailNoRevision};
    }
    if (!host.inputs().chainHasBase(chain)) {
        return {Outcome::Held, kNoStepDetail};
    }
    bool changed = false;
    if (!host.state().foldInfluences(chain, revision, &changed)) {
        return {Outcome::Invalid, kChainDetailFold};
    }
    host.state().setInfluencesChanged(chain, revision, changed);
    if (host.state().revisionGpuOwned(chain, revision)) {
        host.state().noteRevisionGpuTouched(chain, revision);
    }
    const bool skin = host.state().revisionIsSkin(chain, revision);
    const bool valid =
        !skin || host.state().skinTransformsUsable(chain, revision);
    host.state().setInfluencesValid(chain, revision, valid);
    const bool range = host.state().revisionIsRange(chain, revision);
    const bool rangeDq =
        skin && range && host.state().revisionIsDualQuat(chain, revision);
    if (skin && valid && (rangeDq || !range)) {
        host.state().foldTransformForms(chain, revision);
    }
    if (rangeDq && valid && host.state().skinPacketUsable(chain, revision)) {
        host.state().setInfluencesValid(
            chain, revision, host.state().dualQuatPaletteSafe(chain, revision));
    }
    return {Outcome::Applied, kNoStepDetail};
}

/// RevisionStatic: assemble the packet, publish its status, and decide
/// whether the revision runs. The current-phase patch, the assembly's
/// read binding, the status publication, the weight-field overlay and
/// the revisionsBuilt counting stay adapter-local; the staging, the
/// envelope resolve, the acceptance branches and the gate note are
/// shared, in this order on both front ends.
template <class Host>
StepResult
RunRevisionStaticStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    size_t chain = 0, revision = 0;
    if (!host.inputs().revisionIndexForObject(step.object, &chain, &revision)) {
        return {Outcome::Invalid, kChainDetailNoRevision};
    }
    if (!host.inputs().chainHasBase(chain)) {
        return {Outcome::Held, kNoStepDetail};
    }
    const bool skin = host.state().revisionIsSkin(chain, revision);
    if (skin) {
        host.state().adoptRevisionLayout(chain, revision);
    }
    if (!host.state().applyCurrentPhasePacket(chain, revision)) {
        return {Outcome::Invalid, kChainDetailNoRevision};
    }
    host.state().assembleRevision(chain, revision);
    host.state().publishAssembledStatus(chain, revision);
    host.state().finishRevisionStaticHead(chain, revision);
    if (host.state().revisionGpuOwned(chain, revision)) {
        host.state().noteRevisionGpuTouched(chain, revision);
        host.state().gpuRevisionStatic(chain, revision);
        return {Outcome::Applied, kNoStepDetail};
    }
    const size_t count = host.state().chainBaseCount(chain);
    host.state().sizeRevisionStaging(chain, revision, count);
    host.state().setPrecedingCount(chain, revision, count);
    const auto gateValid = [&] {
        if (host.state().gateCheckApplies(chain, revision) &&
            !host.state().groupGateHolds(chain, revision, count)) {
            host.state().noteGateViolation(chain, revision);
            host.state().setAcceptance(chain, revision,
                                       kChainAcceptanceRefuses);
            return false;
        }
        return true;
    };
    if (host.state().revisionIsRange(chain, revision) && !skin) {
        const EnvelopePacketView envelope =
            host.state().envelopeView(chain, revision);
        if (RevisionTakesSeparateBlend(host.inputs().opCode(chain, revision),
                                       envelope)) {
            const bool full = EnvelopeIsFullStrength(envelope);
            host.state().setFullStrength(chain, revision, full);
            if (!full) {
                host.state().resolveRevisionEnvelope(chain, revision, count);
            }
        }
        host.state().setAcceptance(
            chain, revision,
            host.state().prepareRevisionRanges(chain, revision));
        if (!gateValid()) {
            return {Outcome::Rejected, kChainDetailNoRevision};
        }
        return {Outcome::Applied, kNoStepDetail};
    }
    if (!skin) {
        const EnvelopePacketView envelope =
            host.state().envelopeView(chain, revision);
        if (host.inputs().opCode(chain, revision) == kChainOpWire &&
            RevisionTakesSeparateBlend(host.inputs().opCode(chain, revision),
                                       envelope)) {
            const bool full = EnvelopeIsFullStrength(envelope);
            host.state().setFullStrength(chain, revision, full);
            if (!full) {
                host.state().resolveRevisionEnvelope(chain, revision, count);
            }
        }
        host.state().setAcceptance(
            chain, revision,
            host.state().revisionKernelAcceptance(chain, revision));
        return {Outcome::Applied, kNoStepDetail};
    }
    const bool layoutUsable =
        host.state().skinLayoutUsable(chain, revision, count);
    host.state().setLayoutUsable(chain, revision, layoutUsable);
    const EnvelopePacketView envelope =
        host.state().envelopeView(chain, revision);
    const bool full = EnvelopeIsFullStrength(envelope);
    host.state().setFullStrength(chain, revision, full);
    if (!full) {
        host.state().resolveRevisionEnvelope(chain, revision, count);
    }
    host.state().setAcceptance(
        chain, revision, host.state().skinRevisionAcceptance(chain, revision));
    host.state().setDualQuatWeightsSafe(
        chain, revision,
        layoutUsable && host.state().dualQuatWeightsSafe(chain, revision));
    host.state().setPartitionStale(
        chain, revision,
        host.state().revisionPartitionStale(chain, revision, count));
    if (!gateValid()) {
        return {Outcome::Rejected, kChainDetailNoRevision};
    }
    return {Outcome::Applied, kNoStepDetail};
}

/// One group of a Range revision: the assigned group/halo interval.
/// Publishes group g of the entering version -- the group kernel's
/// points where the revision applies, the entering group shared by
/// reference where it does not or where the kernel leaves the group
/// untouched -- with the shared pointBlocks publication, so the
/// group's content version moves exactly when its bytes do. A refusal
/// after an Applies acceptance passes the group through; the join
/// counts it.
template <class Host>
StepResult
RunGroupStepBody(Host &host, size_t chain, size_t revision, size_t g)
{
    using Point = typename Host::Math::Vec3f;
    auto access = host.state().groupStep(chain, revision, g);
    if (!access.state) {
        return {Outcome::Held, kNoStepDetail};
    }
    RigExecGroupState<Point> &state = *access.state;
    state.walk.computed = 0;
    state.walk.reused = 0;
    if (access.rangeSetAside) {
        host.state().passBaseGroup(chain, revision, g, &state);
        state.ok = false;
        return {Outcome::Applied, kNoStepDetail};
    }
    RigExecGroupSource from;
    const RigExecPointsRef<Point> &entering =
        host.state().enteringGroup(chain, revision, g, &from);
    size_t begin = 0, end = 0;
    host.state().groupRange(chain, revision, g, access.precedingCount, &begin,
                            &end);
    const size_t count = access.precedingCount;
    if (count != access.chainPointCount || end < begin ||
        entering.count != end - begin) {
        state.ok = false;
        RigExecPublishPassedGroup(&state, entering, from);
        host.state().noteGateViolation(chain, revision);
        return {Outcome::Applied, kNoStepDetail};
    }
    if (!access.applies) {
        state.ok = true;
        RigExecPublishPassedGroup(&state, entering, from);
        return {Outcome::Applied, kNoStepDetail};
    }
    if (access.skin) {
        if (access.partitionStale) {
            state.ok = false;
            RigExecPublishPassedGroup(&state, entering, from);
            host.state().noteGateViolation(chain, revision);
            return {Outcome::Applied, kNoStepDetail};
        }
        if (!host.state().gatherChunkTransforms(chain, revision, g)) {
            return {Outcome::Invalid, kChainDetailChunk};
        }
        const int k = RigExecGroupScratch(&state, end - begin);
        std::vector<Point> &buffer = *state.own[size_t(k)];
        buffer.resize(end - begin);
        bool untouched = false;
        const bool ok = host.state().runSkinGroup(
            chain, revision, g, entering.data, buffer.data(), count, begin, end,
            &untouched);
        state.ok = ok;
        if (ok && !untouched) {
            RigExecPublishOwnGroup(&state, k);
            return {Outcome::Applied, kNoStepDetail};
        }
        RigExecPublishPassedGroup(&state, entering, from);
        return {Outcome::Applied, kNoStepDetail};
    }
    if (access.blendChunked) {
        host.state().runBlendGroup(chain, revision, g, begin, end, entering,
                                   &state);
        state.ok = true;
        return {Outcome::Applied, kNoStepDetail};
    }
    if (!host.state().prepareGroupHalo(chain, revision, g, entering)) {
        state.ok = false;
        RigExecPublishPassedGroup(&state, entering, from);
        host.state().noteGateViolation(chain, revision);
        return {Outcome::Applied, kNoStepDetail};
    }
    const int k = RigExecGroupScratch(&state, end - begin);
    std::vector<Point> &buffer = *state.own[size_t(k)];
    buffer.resize(end - begin);
    bool untouched = false;
    const bool ok = host.state().runRevisionGroup(
        chain, revision, g, entering.data, buffer.data(), count, begin, end,
        host.state().groupEnvelope(chain, revision), &untouched);
    state.ok = ok;
    if (!ok || untouched) {
        RigExecPublishPassedGroup(&state, entering, from);
        return {Outcome::Applied, kNoStepDetail};
    }
    RigExecPublishOwnGroup(&state, k);
    return {Outcome::Applied, kNoStepDetail};
}

/// RevisionChunk: apply the assigned group/halo interval. A Range
/// revision runs its group step, a Whole keyed skin its speculative
/// chunk, and every other revision its one ungrouped chunk through the
/// adapter's staging. The dispatch and the group bodies are shared;
/// the ungrouped arm's window and device state stay adapter-local.
template <class Host>
StepResult
RunRevisionChunkStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    size_t chain = 0, revision = 0;
    if (!host.inputs().revisionIndexForObject(step.object, &chain, &revision)) {
        return {Outcome::Invalid, kChainDetailNoRevision};
    }
    if (!host.inputs().chainHasBase(chain)) {
        return {Outcome::Held, kNoStepDetail};
    }
    if (host.state().revisionGpuOwned(chain, revision)) {
        return {Outcome::Held, kNoStepDetail};
    }
    if (step.part < 0) {
        return {Outcome::Invalid, kChainDetailNoChunk};
    }
    const size_t part = size_t(step.part);
    if (host.state().revisionIsRange(chain, revision)) {
        return RunGroupStepBody<Host>(host, chain, revision, part);
    }
    if (host.state().revisionWholeChunked(chain, revision)) {
        if (!host.state().runWholeSkinChunk(chain, revision, part)) {
            return {Outcome::Invalid, kChainDetailChunk};
        }
        return {Outcome::Applied, kNoStepDetail};
    }
    if (!host.state().runSingleChunk(chain, revision, step.part)) {
        return {Outcome::Invalid, kChainDetailChunk};
    }
    return {Outcome::Applied, kNoStepDetail};
}

/// RevisionFuse: the apply-or-pass-through decision, the RevisionOut
/// publication and the sticky dirty bit. A Range revision joins its
/// groups, a Whole revision fuses them, and a Legacy revision publishes
/// its one array; a revision a cycle set aside passes the base through.
/// Content versions move exactly when the published bytes or group ids
/// do. The GPU-owned fuse stays with the baked adapter: it intercepts
/// before this body ever sees a GPU revision.
template <class Host>
StepResult
RunRevisionFuseStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    using Point = typename Host::Math::Vec3f;
    size_t chain = 0, revision = 0;
    if (!host.inputs().revisionIndexForObject(step.object, &chain, &revision)) {
        return {Outcome::Invalid, kChainDetailNoRevision};
    }
    if (!host.inputs().chainHasBase(chain)) {
        return {Outcome::Held, kNoStepDetail};
    }
    if (host.state().revisionRangeSetAside(chain, revision)) {
        host.state().passSetAsideFuse(chain, revision);
        return {Outcome::Applied, kNoStepDetail};
    }
    const bool skin = host.state().revisionIsSkin(chain, revision);
    const bool packetValid = host.state().fusePacketValid(chain, revision);
    host.state().emitFuseDiagnostic(chain, revision, packetValid);
    if (host.state().revisionIsRangeRole(chain, revision)) {
        auto access = host.state().fuseRevision(chain, revision);
        host.state().setRevisionExecuted(chain, revision, true);
        host.state().noteFuseExecuted(chain, revision);
        const bool applied = RevisionAppliesBody(
            packetValid, host.state().revisionStatusAllows(chain, revision),
            host.state().revisionAcceptanceValue(chain, revision));
        host.state().setFuseResultFromStatus(chain, revision);
        if (!applied && host.state().revisionStatusAllows(chain, revision)) {
            host.state().setFuseResultFailed(chain, revision);
        }
        bool moved = false;
        if (access.groupIds && access.groups &&
            access.groupIds->size() != access.groups->size()) {
            access.groupIds->assign(access.groups->size(),
                                    RigExecGroupSource());
            moved = true;
        }
        uint32_t refusals = 0;
        if (access.groupIds) {
            for (size_t g = 0; g < access.groupIds->size(); ++g) {
                if (access.groupWritten && g < access.groupWritten->size() &&
                    (*access.groupWritten)[g] && access.groups &&
                    g < access.groups->size() && applied &&
                    !(*access.groups)[g].ok) {
                    ++refusals;
                }
                const RigExecGroupSource source =
                    host.state().fuseVersionSource(chain, revision + 1, g);
                if ((*access.groupIds)[g] != source) {
                    (*access.groupIds)[g] = source;
                    moved = true;
                }
            }
        }
        if (access.rangeRefusals) {
            *access.rangeRefusals = refusals;
        }
        if (moved && access.doneVersion) {
            ++*access.doneVersion;
        }
        host.state().saveFuseLastStatus(chain, revision);
        if (access.ran) {
            *access.ran = true;
        }
        host.state().publishFuseState(chain, revision, step.object);
        return {Outcome::Applied, kNoStepDetail};
    }
    if (host.state().revisionIsWholeRole(chain, revision)) {
        auto access = host.state().fuseRevision(chain, revision);
        host.state().setRevisionExecuted(chain, revision, true);
        host.state().noteFuseExecuted(chain, revision);
        host.state().setFuseResultFromStatus(chain, revision);
        const size_t groupCount = access.groups ? access.groups->size() : 0;
        const size_t count = access.precedingCount;
        const bool keyed = host.state().revisionChunkedValue(chain, revision);
        bool applied = packetValid &&
                       host.state().revisionStatusAllows(chain, revision) &&
                       host.state().wholeFuseSized(chain, revision);
        const bool whole =
            applied && skin &&
            host.state().revisionPartitionStaleValue(chain, revision);
        std::vector<Point> fused;
        if (whole) {
            const Point *points = nullptr;
            size_t enteringCount = 0;
            host.state().enteringVersionPoints(chain, revision, &points,
                                               &enteringCount);
            applied = FuseWholeRevisionBody<Host>(host, chain, revision, points,
                                                  enteringCount, &fused);
        } else if (applied &&
                   host.state().revisionAcceptanceValue(chain, revision) !=
                       kChainAcceptanceDeferred) {
            applied = host.state().revisionAcceptanceValue(chain, revision) ==
                      kChainAcceptanceApplies;
        } else if (applied) {
            const size_t chunks =
                host.state().revisionChunkCount(chain, revision);
            for (size_t k = 0; k < chunks; ++k) {
                if (!host.state().chunkHoldsAnswer(chain, revision, k)) {
                    applied = false;
                    break;
                }
            }
        }
        if (applied && keyed && !whole) {
            for (size_t g = 0; g < groupCount; ++g) {
                if (!host.state().chunkHoldsAnswer(chain, revision, g)) {
                    applied = false;
                    break;
                }
            }
        }
        if (applied && !whole && !keyed && access.stagingOutput &&
            access.stagingOutput->size() != count) {
            applied = false;
        }
        if (!applied && host.state().revisionStatusAllows(chain, revision)) {
            host.state().setFuseResultFailed(chain, revision);
        }
        const std::vector<Point> *slices =
            whole ? &fused : (keyed ? nullptr : access.stagingOutput);
        bool moved = false;
        if (access.groupIds && access.groupIds->size() != groupCount) {
            access.groupIds->assign(groupCount, RigExecGroupSource());
            moved = true;
        }
        for (size_t g = 0; g < groupCount; ++g) {
            RigExecGroupState<Point> &state = (*access.groups)[g];
            if (applied && !slices) {
                RigExecPublishOwnGroup(&state, state.ownComputed);
            } else if (applied) {
                size_t begin = 0, end = 0;
                host.state().groupRange(chain, revision, g, count, &begin,
                                        &end);
                const int k = RigExecGroupScratch(&state, end - begin);
                std::vector<Point> &buffer = *state.own[size_t(k)];
                std::copy(slices->begin() + long(begin),
                          slices->begin() + long(end), buffer.begin());
                RigExecPublishOwnGroup(&state, k);
            } else {
                RigExecGroupSource from;
                const RigExecPointsRef<Point> &entering =
                    host.state().enteringGroup(chain, revision, g, &from);
                RigExecPublishPassedGroup(&state, entering, from);
            }
            RigExecGroupSource source;
            source.slot = host.state().fuseGroupSlot(chain, revision, g);
            source.version = state.version;
            if (access.groupIds && (*access.groupIds)[g] != source) {
                (*access.groupIds)[g] = source;
                moved = true;
            }
        }
        if (moved && access.doneVersion) {
            ++*access.doneVersion;
        }
        host.state().saveFuseLastStatus(chain, revision);
        if (access.ran) {
            *access.ran = true;
        }
        host.state().noteWholeFuseSource(chain, revision, int(revision));
        host.state().publishFuseState(chain, revision, step.object);
        return {Outcome::Applied, kNoStepDetail};
    }
    {
        auto access = host.state().fuseRevision(chain, revision);
        host.state().setRevisionExecuted(chain, revision, true);
        host.state().noteFuseExecuted(chain, revision);
        const bool hadOwn = access.ran && *access.ran && access.currentSource &&
                            *access.currentSource == int(revision);
        bool moved = !(access.ran && *access.ran);
        host.state().setFuseResultFromStatus(chain, revision);
        bool applied =
            packetValid && host.state().revisionStatusAllows(chain, revision);
        const bool whole =
            applied && skin &&
            host.state().revisionPartitionStaleValue(chain, revision);
        std::vector<Point> fused;
        if (whole) {
            const Point *points = nullptr;
            size_t count = 0;
            host.state().enteringVersionPoints(chain, revision, &points,
                                               &count);
            applied =
                host.state().legacyWholeStagingOk(chain, revision, count) &&
                FuseWholeRevisionBody<Host>(host, chain, revision, points,
                                            count, &fused);
        } else if (applied &&
                   host.state().revisionAcceptanceValue(chain, revision) !=
                       kChainAcceptanceDeferred) {
            applied = host.state().revisionAcceptanceValue(chain, revision) ==
                      kChainAcceptanceApplies;
        } else if (applied) {
            const size_t chunks =
                host.state().revisionChunkCount(chain, revision);
            for (size_t k = 0; k < chunks; ++k) {
                if (!host.state().chunkOk(chain, revision, k)) {
                    applied = false;
                    break;
                }
            }
        }
        if (applied) {
            if (host.state().revisionChunkedValue(chain, revision) || whole) {
                const std::vector<Point> &result =
                    whole ? fused : *access.stagingOutput;
                if (!hadOwn) {
                    moved = moved ||
                            !PointsBitsEqualBody(result.data(), result.size(),
                                                 access.passedPoints->data(),
                                                 access.passedPoints->size());
                }
                const bool copied =
                    CopyMovedPointsBody<Point>(result, access.output);
                moved = moved || (hadOwn && copied);
            } else if (access.stagingFresh && *access.stagingFresh) {
                const std::vector<Point> &previous =
                    hadOwn ? *access.output : *access.passedPoints;
                moved = moved ||
                        !PointsBitsEqualBody(access.stagingOutput->data(),
                                             access.stagingOutput->size(),
                                             previous.data(), previous.size());
                access.output->swap(*access.stagingOutput);
                access.stagingOutput->resize(access.output->size());
                *access.stagingFresh = false;
            } else if (!hadOwn) {
                moved =
                    moved || !PointsBitsEqualBody(access.output->data(),
                                                  access.output->size(),
                                                  access.passedPoints->data(),
                                                  access.passedPoints->size());
            }
            if (access.currentSource) {
                *access.currentSource = int(revision);
            }
        } else {
            const Point *points = nullptr;
            size_t count = 0;
            host.state().enteringVersionPoints(chain, revision, &points,
                                               &count);
            const std::vector<Point> &previous =
                hadOwn ? *access.output : *access.passedPoints;
            const bool same = PointsBitsEqualBody(
                points, count, previous.data(), previous.size());
            moved = moved || !same;
            if (hadOwn || !same) {
                if (points) {
                    access.passedPoints->assign(points, points + count);
                } else {
                    access.passedPoints->clear();
                }
            }
            if (access.currentSource) {
                *access.currentSource =
                    host.state().precedingCurrentSource(chain, revision);
            }
            if (host.state().revisionStatusAllows(chain, revision)) {
                host.state().setFuseResultFailed(chain, revision);
            }
        }
        if (moved && access.doneVersion) {
            ++*access.doneVersion;
        }
        host.state().saveFuseLastStatus(chain, revision);
        if (access.ran) {
            *access.ran = true;
        }
        host.state().publishFuseState(chain, revision, step.object);
        return {Outcome::Applied, kNoStepDetail};
    }
}

/// ChainStatus: sweep the per-revision statuses and publish ChainPoints.
/// A chain cut into groups is re-gathered only when its group content
/// ids moved; ids that still stand leave the published array in place,
/// identity and all. Content version moves exactly when the bytes do.
template <class Host>
StepResult
RunChainStatusStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    using Point = typename Host::Math::Vec3f;
    size_t chain = 0;
    if (!host.inputs().chainIndexForObject(step.object, &chain)) {
        return {Outcome::Invalid, kChainDetailNoChain};
    }
    auto access = host.state().chainStatus(chain);
    if (!host.inputs().chainHasBase(chain)) {
        if (access.haveResult) {
            *access.haveResult = false;
        }
        host.state().clearChainStatusPublish(chain);
        return {Outcome::Held, kNoStepDetail};
    }
    host.state().emitChainStatusDiagnostics(chain);
    host.state().noteChainBuilt(chain);
    if (host.state().chainIsGpu(chain)) {
        host.state().noteChainGpuPending(chain);
        return {Outcome::Applied, kNoStepDetail};
    }
    const size_t last = host.state().chainRevisionCount(chain);
    if (host.state().chainHasGroups(chain) && last > 0) {
        const size_t groups = host.state().chainGroupCount(chain);
        std::vector<RigExecGroupSource> ids(groups);
        for (size_t g = 0; g < groups; ++g) {
            ids[g] = host.state().chainStatusSource(chain, last, g);
        }
        const bool same =
            access.haveResult && *access.haveResult && access.resultIds &&
            access.resultIds->size() == groups && *access.resultIds == ids;
        if (same) {
            return {Outcome::Applied, kNoStepDetail};
        }
        std::vector<Point> gathered;
        for (size_t g = 0; g < groups; ++g) {
            const RigExecPointsRef<Point> &ref =
                host.state().chainStatusGroup(chain, last, g);
            if (ref.count > 0 && ref.data) {
                gathered.insert(gathered.end(), ref.data, ref.data + ref.count);
            }
        }
        host.state().publishChainGroups(chain, gathered, ids);
        return {Outcome::Applied, kNoStepDetail};
    }
    const Point *points = nullptr;
    size_t count = 0;
    host.state().chainFinalPoints(chain, last, &points, &count);
    std::vector<Point> gathered;
    if (count > 0 && points) {
        gathered.assign(points, points + count);
    }
    host.state().publishChainFinal(chain, gathered);
    return {Outcome::Applied, kNoStepDetail};
}

/// Derived: recompute a derived target from the graph-bound base and
/// the chain's published points, including matrix outputs. The GPU
/// deferral stays with the baked adapter around this body.
template <class Host>
StepResult
RunDerivedStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    using Point = typename Host::Math::Vec3f;
    size_t chain = 0, derived = 0;
    if (!host.inputs().derivedIndexForObject(step.object, &chain, &derived)) {
        return {Outcome::Invalid, kChainDetailNoDerived};
    }
    host.state().captureDerivedPublished(chain, derived);
    host.state().prepareDerivedBase(chain, derived);
    if (!host.inputs().chainHasBase(chain) ||
        !host.state().derivedHasBase(chain, derived)) {
        host.state().noteDerivedEarlyVersion(chain, derived);
        return {Outcome::Held, kNoStepDetail};
    }
    host.state().adoptDerivedLayout(chain, derived);
    if (host.state().derivedIsMatrix(chain, derived)) {
        if (!host.state().runDerivedMatrix(chain, derived)) {
            return {Outcome::Invalid, kChainDetailNoDerived};
        }
        return {Outcome::Applied, kNoStepDetail};
    }
    if (!host.state().foldDerivedInfluences(chain, derived)) {
        return {Outcome::Invalid, kChainDetailFold};
    }
    host.state().assembleDerived(chain, derived);
    host.state().noteDerivedBuilt(chain, derived);
    host.state().noteDerivedExecuted(chain, derived);
    std::vector<Point> values;
    const bool applied = host.state().derivedStatusAllows(chain, derived) &&
                         host.state().runDerivedKernel(chain, derived, &values);
    host.state().setDerivedResultFromStatus(chain, derived);
    if (!applied) {
        host.state().derivedLastBasePoints(chain, derived, &values);
        if (host.state().derivedStatusAllows(chain, derived)) {
            host.state().setDerivedResultFailed(chain, derived);
        }
    }
    host.state().saveDerivedRevision(chain, derived, std::move(values));
    if (host.state().derivedResultIsFailed(chain, derived)) {
        host.state().emitDerivedDiagnostic(chain, derived);
    }
    host.state().publishDerivedResult(chain, derived);
    return {Outcome::Applied, kNoStepDetail};
}

/// The paired skip switch: the same per-kind flag clears on both front
/// ends, publishing nothing fabricated.
template <class Host>
void
RunSkipGeometryStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    switch (StepKind(step.kind)) {
    case StepKind::ChainInputs: {
        size_t chain = 0;
        if (host.inputs().chainIndexForObject(step.object, &chain)) {
            host.state().skipChainInputs(chain);
        }
        return;
    }
    case StepKind::Derived: {
        size_t chain = 0, derived = 0;
        if (host.inputs().derivedIndexForObject(step.object, &chain,
                                                &derived)) {
            host.state().skipDerived(chain, derived);
        }
        return;
    }
    case StepKind::ChainStatus:
        return;
    default:
        break;
    }
    size_t chain = 0, revision = 0;
    if (!host.inputs().revisionIndexForObject(step.object, &chain, &revision)) {
        return;
    }
    switch (StepKind(step.kind)) {
    case StepKind::InfluenceFold:
        host.state().skipInfluenceFold(chain, revision);
        return;
    case StepKind::RevisionStatic:
        host.state().skipRevisionStatic(chain, revision, step.object);
        return;
    case StepKind::RevisionChunk:
        host.state().skipRevisionChunk(chain, revision, step.part);
        return;
    case StepKind::RevisionFuse:
        host.state().skipRevisionFuse(chain, revision);
        return;
    default:
        return;
    }
}

/// The one geometry-step dispatch for the revision chain: ChainInputs,
/// InfluenceFold, RevisionStatic, RevisionChunk, RevisionFuse,
/// ChainStatus and Derived. Every other kind answers Invalid here.
template <class Host>
StepResult
RunGeometryStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "chain Host is not bound");
    switch (StepKind(step.kind)) {
    case StepKind::ChainInputs:
        return RunChainInputsStep<Host>(host, step);
    case StepKind::InfluenceFold:
        return RunInfluenceFoldStep<Host>(host, step);
    case StepKind::RevisionStatic:
        return RunRevisionStaticStep<Host>(host, step);
    case StepKind::RevisionChunk:
        return RunRevisionChunkStep<Host>(host, step);
    case StepKind::RevisionFuse:
        return RunRevisionFuseStep<Host>(host, step);
    case StepKind::ChainStatus:
        return RunChainStatusStep<Host>(host, step);
    case StepKind::Derived:
        return RunDerivedStep<Host>(host, step);
    default:
        return {Outcome::Invalid, kNoStepDetail};
    }
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_CHAIN_H
