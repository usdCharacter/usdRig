// The revision-chain step bodies the baked program and .rigexec playback
// share: ChainInputs, InfluenceFold, RevisionStatic and RevisionChunk, plus
// the chain helpers both front ends used to carry twice -- base-group
// publication, group resolution, point-version resolution and partition
// adoption (docs/specs/eval-core-parity.md). One definition per body, so a
// second front end cannot drift into a different answer. Adapter binding
// stays with each front end: the core classifies roles and reads Build
// facts from its leaves, playback binds the partition and roles the file
// recorded; neither front end keeps a second decision. ClassifyRevision,
// GateDefault and GateRevision stay core-adapter Build functions and are
// never ported here; the gate predicate itself is gate.h's, not this
// header's. The weight-field overlay publication and the revisionsBuilt
// counting stay adapter-local, as host.h records.
//
// Bodies are templated on the host Math trait of envelope.h's convention
// (Math::Vec3f is the chain's point type) and read the host contract of
// host.h: nested Math, Inputs and State, with inputs(), state() and
// ranges(). A chain host adds:
//
//   typename Host::ChainPoints
//        the container a chain's base and result points live in
//        (VtVec3fArray on the baked side, std::vector<RrVec3f> at
//        playback); borrowed for the call, never outliving its owner.
//
//   inputs() adds, all borrowed and valid for the call:
//     bool chainIndexForObject(int32_t object, size_t *chain) const;
//     bool revisionIndexForObject(int32_t object, size_t *chain,
//                                 size_t *revision) const;
//     bool chainHasBase(size_t chain) const;
//     int opCode(size_t chain, size_t revision) const;
//          the revision's op as a wire RevisionOp ordinal (kinds.h);
//     bool useSimd() const;
//
//   state() adds the slot, revision and group results by compiled index:
//     ChainInputsAccessT<Math::Vec3f, Host::ChainPoints>
//         chainInputs(size_t chain);
//     void resetChainRevision(size_t chain, size_t revision);
//     bool revisionRangeNotAside(size_t chain, size_t revision);
//     void setRevisionSource(size_t chain, size_t revision, int source);
//     void resetBaseGroups(size_t chain);
//     void clearChainResultIds(size_t chain);
//     void countChainCreated(size_t chain);
//     void noteChainSchedules(size_t chain);
//          the adapter's own counters (step counters on the baked side,
//          the chain's created/schedule counts at playback);
//     void noteChainBaseDirty(size_t chain, bool dirty);
//     void publishChainBaseGroups(size_t chain, bool moved);
//     bool chainPointsEqual(size_t chain, const Host::ChainPoints &a,
//                           const Host::ChainPoints &b);
//          the bit-exact base comparison (the version rule);
//     bool chainBaseValueEqual(size_t chain, const Host::ChainPoints &a,
//                              const Host::ChainPoints &b);
//          the adapter's base-dirty comparison (a head-value compare on
//          the baked side, the same bits at playback);
//     bool foldInfluences(size_t chain, size_t revision, bool *changed);
//          false is the adapter's existing diagnostic path (playback's
//          labeled slot errors); the fold's slot resolution and error
//          text stay adapter-local, its ordering and changed rule here.
//     void setInfluencesChanged(size_t chain, size_t revision, bool);
//     bool influencesValid(size_t chain, size_t revision);
//     void setInfluencesValid(size_t chain, size_t revision, bool);
//     bool skinTransformsUsable(size_t chain, size_t revision);
//     void foldTransformForms(size_t chain, size_t revision);
//     bool skinPacketUsable(size_t chain, size_t revision);
//     bool dualQuatPaletteSafe(size_t chain, size_t revision);
//     bool revisionIsRange(size_t chain, size_t revision);
//     bool revisionIsDualQuat(size_t chain, size_t revision);
//     bool revisionGpuOwned(size_t chain, size_t revision);
//     void noteRevisionGpuTouched(size_t chain, size_t revision);
//     void adoptRevisionLayout(size_t chain, size_t revision);
//     bool applyCurrentPhasePacket(size_t chain, size_t revision);
//          false is the adapter's diagnostic path; the current-phase
//          packet patch and its wording stay adapter-local.
//     void assembleRevision(size_t chain, size_t revision);
//          the packet assembly: parameter assembly is shared through
//          the family headers, the read binding stays adapter-local.
//     void publishAssembledStatus(size_t chain, size_t revision);
//          the packet status, the defaultWeight diagnostic scalar, the
//          static-dirty flag, the weight-field overlay publication and
//          the revisionsBuilt counting -- adapter-local by design.
//     bool revisionIsSkin(size_t chain, size_t revision);
//     bool revisionRangeChain(size_t chain, size_t revision);
//     bool revisionRangeLattice(size_t chain, size_t revision);
//     void finishRevisionStaticHead(size_t chain, size_t revision);
//          the range-chain delta version, the held lattice rest and the
//          staged sizing the assemble produced, in the adapter's types.
//     void sizeRevisionStaging(size_t chain, size_t revision,
//                              size_t count);
//          the staged sizing and the tail flags, in the adapter's types;
//     void setPrecedingCount(size_t chain, size_t revision, size_t count);
//     size_t chainBaseCount(size_t chain);
//     EnvelopePacketView envelopeView(size_t chain, size_t revision);
//     const float *resolvedEnvelope(size_t chain, size_t revision);
//     void resolveRevisionEnvelope(size_t chain, size_t revision,
//                                  size_t count);
//     int prepareRevisionRanges(size_t chain, size_t revision);
//     int revisionKernelAcceptance(size_t chain, size_t revision);
//     int skinRevisionAcceptance(size_t chain, size_t revision);
//          acceptance as a RigExecRevisionAcceptance ordinal (Refuses 0,
//          Applies 1, Deferred 2), decided by envelope.h and skin.h.
//     bool skinLayoutUsable(size_t chain, size_t revision, size_t count);
//     bool dualQuatWeightsSafe(size_t chain, size_t revision);
//     bool revisionPartitionStale(size_t chain, size_t revision,
//                                 size_t count);
//     bool gateCheckApplies(size_t chain, size_t revision);
//     bool groupGateHolds(size_t chain, size_t revision, size_t count);
//     void noteGateViolation(size_t chain, size_t revision);
//     void setFullStrength(size_t chain, size_t revision, bool);
//     void setAcceptance(size_t chain, size_t revision, int);
//     void setLayoutUsable(size_t chain, size_t revision, bool);
//     void setDualQuatWeightsSafe(size_t chain, size_t revision, bool);
//     void setPartitionStale(size_t chain, size_t revision, bool);
//     void gpuRevisionStatic(size_t chain, size_t revision);
//          the GPU-owned static arm; the baked adapter's alone.
//     bool revisionWholeChunked(size_t chain, size_t revision);
//     bool runSingleChunk(size_t chain, size_t revision, int32_t part);
//          the ungrouped chunk arm (window, device and staging state are
//          the adapter's); false is its diagnostic path.
//     GroupStepAccessT<Math::Vec3f> groupStep(size_t chain,
//                                              size_t revision, size_t g);
//     void groupRange(size_t chain, size_t revision, size_t g,
//                     size_t count, size_t *begin, size_t *end);
//     const RigExecPointsRef<Math::Vec3f> &
//         enteringGroup(size_t chain, size_t revision, size_t g,
//                       RigExecGroupSource *from);
//     void passBaseGroup(size_t chain, size_t revision, size_t g,
//                        RigExecGroupState<Math::Vec3f> *state);
//     bool gatherChunkTransforms(size_t chain, size_t revision, size_t g);
//     bool runSkinGroup(size_t chain, size_t revision, size_t g,
//                       const Math::Vec3f *in, Math::Vec3f *out,
//                       size_t count, size_t begin, size_t end,
//                       bool *untouched);
//          the group's skin kernel over its chunk table (skin.h's
//          ApplySkinGroupRange at playback, the revision-group kernel
//          with the chunk view on the baked side);
//     void runBlendGroup(size_t chain, size_t revision, size_t g,
//                        size_t begin, size_t end,
//                        const RigExecPointsRef<Math::Vec3f> &entering,
//                        RigExecGroupState<Math::Vec3f> *state);
//          the chunked blend arm; its channel weights are the adapter's
//          declared reads, its accumulation blendRangeKernel.h's;
//     const float *groupEnvelope(size_t chain, size_t revision);
//     bool prepareGroupHalo(size_t chain, size_t revision, size_t g,
//                           const RigExecPointsRef<Math::Vec3f> &entering);
//     bool runRevisionGroup(size_t chain, size_t revision, size_t g,
//                           const Math::Vec3f *in, Math::Vec3f *out,
//                           size_t count, size_t begin, size_t end,
//                           const float *envelope, bool *untouched);
//     bool runWholeSkinChunk(size_t chain, size_t revision, size_t g);
//
// Views cannot outlive the packet or program they borrow. Bodies are
// no-mutex zones: no guarded statics, no text-built tokens, no
// environment reads, no bare new. Range and Whole results stay bitwise
// identical: group bodies write disjoint groups through the shared
// pointBlocks publication, and untouched groups keep their handles.
#ifndef RIGEXEC_EVAL_CORE_CHAIN_H
#define RIGEXEC_EVAL_CORE_CHAIN_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
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
    kChainDetailChunk = 5
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
    const auto noteGate = [&] {
        if (host.state().gateCheckApplies(chain, revision) &&
            !host.state().groupGateHolds(chain, revision, count)) {
            host.state().noteGateViolation(chain, revision);
        }
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
        noteGate();
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
    noteGate();
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

/// The one geometry-step dispatch for the revision chain: ChainInputs,
/// InfluenceFold, RevisionStatic and RevisionChunk. Every other kind
/// answers Invalid here; the fuse, status, derived and skip bodies are
/// their own lifts and stay with their adapters.
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
    default:
        return {Outcome::Invalid, kNoStepDetail};
    }
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_CHAIN_H
