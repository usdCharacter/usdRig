#ifndef RIGEXEC_COMPILED_PROGRAM_H
#define RIGEXEC_COMPILED_PROGRAM_H

#include "rigExec/internal/bakedClusters.h"
#include "rigExec/api/bakedProgram.h"
#include "rigExec/internal/bakedTrace.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/frameExtraction.h"
#include "rigExec/internal/gpuDeformTranslation.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/internal/moverPacket.h"
#include "rigExec/internal/oracleInputs.h"
#include "rigExec/internal/pathText.h"
#include "rigExec/internal/scalarReferenceAdapter.h"
#include "rigExec/internal/solverKernels.h"
#include "rigExec/api/types.h"
#include "rigExec/internal/valueAddress.h"
#include "rigExec/internal/weightPackets.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExecMath/dualQuat.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecMath/pointBlocks.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/spaceMatchKernel.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecScene/autoClavicleGraph.h"
#include "rigExecScene/constraintProgram.h"
#include "rigExecScene/geometrySceneTranslation.h"
#include "rigExecScene/poseProgram.h"
#include "rigExecScene/providerProgram.h"
#include "rigExecScene/solverProgram.h"
#include "rigExecScene/weightProgram.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/timeCode.h"

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

struct RigExecBakedProgramImpl;

// The input binding table.
// RigExecResolvedInputs::GetAttribute -- which is what both exec's
// AttributeValue accessor and every static read in the evaluator resolve
// through -- walks a single authored connection chain and takes the nearest
// readable upstream value, preferring anything this generation already
// resolved. That WALK is structural: connections cannot change without
// a resync, and a resync recompiles. Only the VALUES on it can move.
// So each input is classified once: if nothing on its walk is written by a
// property chain and nothing on it might vary with time, the value is a
// compiled rig constant and the frame path never touches USD for it. Otherwise it is
// read per frame -- through a retained UsdAttributeQuery when USD alone can
// answer, and through the generation's resolved inputs when a property chain
// is in the way, which is the same route the dynamic path takes.

template <class T> struct RigExecBakedInput {
    T constant{};
    /// Direct source sampling is independent of temporal variation.
    bool sourceBacked = false;
    T sourceFallback{};
    UsdAttributeQuery query;   ///< set when USD alone answers, per frame
    UsdAttribute resolvedAttr; ///< set when a property chain is on the walk
    UsdAttribute head;         ///< where the walk starts, for an override
    bool varying = false;
    /// Index into RigExecBakedProgramImpl::overridden, or -1 when the
    /// input was never registered (the rest ladder, which is folded rather
    /// than read).
    int overrideIndex = -1;
    /// Index into the program's leaf pool for T (RigExecBakedLeafPools), or
    /// -1 before RigExecBakedNumberLeaves numbered it. The leaf holds this
    /// input's value at the run's time; the fields above stay the binding.
    int leaf = -1;
    /// Index into RigExecBakedProgramImpl::readerWalks when the walk from
    /// `resolvedAttr` meets a chain target or a phased record consumer: the
    /// leaf is then resolved from head leaves, overrides and versions
    /// (RigExecBakedReadWalked), never through the resolved inputs. -1
    /// otherwise. Set at Build only.
    int walk = -1;
};

enum class RigExecBakedLeafType : uint8_t {
    Double,
    Float,
    Int,
    Bool,
    Token,
    Matrix4d,
    Vec3d,
    Vec3f
};

/// The pool a binding of type T samples into, and how the pool stores it
/// (bool as a byte, so a pool is a plain array).
template <class T> struct RigExecBakedLeafTraits {
    static_assert(sizeof(T) == 0, "no leaf pool holds this input type");
};
template <> struct RigExecBakedLeafTraits<double> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Double;
    using Stored = double;
};
template <> struct RigExecBakedLeafTraits<float> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Float;
    using Stored = float;
};
template <> struct RigExecBakedLeafTraits<int> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Int;
    using Stored = int;
};
template <> struct RigExecBakedLeafTraits<bool> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Bool;
    using Stored = unsigned char;
};
template <> struct RigExecBakedLeafTraits<TfToken> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Token;
    using Stored = TfToken;
};
template <> struct RigExecBakedLeafTraits<GfMatrix4d> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Matrix4d;
    using Stored = GfMatrix4d;
};
template <> struct RigExecBakedLeafTraits<GfVec3d> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Vec3d;
    using Stored = GfVec3d;
};
template <> struct RigExecBakedLeafTraits<GfVec3f> {
    static constexpr RigExecBakedLeafType type = RigExecBakedLeafType::Vec3f;
    using Stored = GfVec3f;
};

template <class T> struct RigExecBakedLeafPool {
    using Type = T;
    using Stored = typename RigExecBakedLeafTraits<T>::Stored;
    std::vector<Stored> value;
    /// 1 when this run's sample differs bitwise from the one before it; 0
    /// for a leaf this run did not re-sample.
    std::vector<char> changed;
    /// 1 when the next sample must re-read the leaf whatever else holds.
    std::vector<char> mustSample;
    /// The leaf id (RigExecBakedProgramImpl::leafRefs) of each entry.
    std::vector<uint32_t> id;
    /// Pool indices whose `changed` byte went 0 -> 1 since the first sample
    /// pass last cleared the flags (an index may repeat); the next clear
    /// zeroes only these. Kept by RigExecBakedNoteLeafWrite.
    std::vector<uint32_t> changedList;
    /// Pool indices whose `mustSample` byte went 0 -> 1 and that no sample
    /// pass has consumed yet. Kept by RigExecBakedMarkPoolLeaf; every pass
    /// keeps only the entries still marked whose input it samples.
    std::vector<uint32_t> markedList;
    /// Advanced by every mark, consumed or not: what the frozen samplers'
    /// memo compares to re-read only the leaves an edit reached. Grown on
    /// the first mark; an index past its end has never been marked.
    std::vector<uint64_t> editSerial;
};

/// \p pool's leaf \p k's `editSerial`.
template <class Pool>
inline uint64_t
RigExecBakedPoolLeafEditSerial(const Pool &pool, size_t k)
{
    return k < pool.editSerial.size() ? pool.editSerial[k] : 0;
}

/// Where leaf id k lives: its pool and its index in that pool.
struct RigExecBakedLeafRef {
    RigExecBakedLeafType type = RigExecBakedLeafType::Double;
    uint32_t index = 0;
};

struct RigExecBakedLeafPools {
    std::tuple<RigExecBakedLeafPool<double>, RigExecBakedLeafPool<float>,
               RigExecBakedLeafPool<int>, RigExecBakedLeafPool<bool>,
               RigExecBakedLeafPool<TfToken>, RigExecBakedLeafPool<GfMatrix4d>,
               RigExecBakedLeafPool<GfVec3d>, RigExecBakedLeafPool<GfVec3f>>
        pools;

    template <class T> RigExecBakedLeafPool<T> &Of()
    {
        return std::get<RigExecBakedLeafPool<T>>(pools);
    }
    template <class T> const RigExecBakedLeafPool<T> &Of() const
    {
        return std::get<RigExecBakedLeafPool<T>>(pools);
    }
    template <class Fn> void ForEach(Fn &&fn)
    {
        std::apply([&fn](auto &...pool) { (fn(pool), ...); }, pools);
    }
};

/// The path leaves of one revision or weight object: the reads its packet
/// assembly makes (RigExecRevisionLeafDecl), sampled on the owning thread in
/// the geometry prologue (RigExecBakedSamplePathLeaves) and read by the step
/// bodies instead of the stage. No binding of _ForEachPatchableInput owns
/// them; RigExecBakedNumberLeaves numbers them after the visitor's leaves.
struct RigExecBakedPathLeaves {
    /// Build state.
    RigExecRevisionLeafDecl decl;
    /// Per key, the attribute at its path (invalid when none stands there),
    /// every path its read can reach, and whether the read can move with the
    /// time. Owning thread only: a frozen worker never touches them, except
    /// that RigExecBakedRolesStand compares its clone's own copy of `hops`
    /// by path identity.
    std::vector<UsdAttribute> attributes;
    std::vector<std::vector<SdfPath>> hops;
    std::vector<char> varying;
    /// Per key, whether it reads topology (RigExecRevisionLeafRoleIsTopology,
    /// or a sparse blend sample's offsets and point indices): compiled rig state,
    /// whose authored edits rebuild the program. Build state. For such a key
    /// `overrideReached` says whether an interactive override stood on one
    /// of its hops at its last sample.
    std::vector<char> compiledRig;
    std::vector<char> overrideReached;
    /// Per key, its RigExecBakedProgramImpl::readerWalks index when its
    /// read goes through the resolved inputs and its hops meet a chain
    /// target or a record consumer, else -1 (or empty: none). Build state.
    std::vector<int> walks;
    std::vector<int>
        exactVersions; ///< per-key current property/record output, -1 for raw
    std::vector<int>
        exactRecordIndices; ///< record index or -1 for direct version
    std::vector<int>
        exactValueTypes; ///< producer PropertyChain::Arm or -1 for raw
    /// Per key, the value the last sample read, and whether that sample
    /// moved it (bitwise, as the binding leaves compare).
    std::vector<VtValue> values;
    mutable std::vector<VtValue>
        consumedValues; ///< per-owner body values, never sampled memo keys
    std::vector<char> changed;
    /// Per key, the content version the operation keys of a geometry owner
    /// carry for its value: two runs that read the key read the same
    /// version exactly when they read the same bytes. Written only through
    /// RigExecBakedSetPathLeaf, which compares a new value with `observed`,
    /// the value the last run to read the key saw (taken at the first write
    /// after that run, `observedRuns` naming it), and its `observedVersions`.
    std::vector<uint64_t> versions;
    std::vector<VtValue> observed;
    std::vector<uint64_t> observedVersions;
    std::vector<uint64_t> observedRuns;
    /// Per key, the next sample must re-read it whatever else holds: a value
    /// edit reached one of its paths, or a sample skipped it.
    std::vector<char> mustSample;
    /// What the last sample saw: whether there was one, its time, the
    /// program stamp, whether an interactive override stood, and the chain
    /// results' serial (RigExecBakedProgramImpl::pathLeafChainSerial).
    bool sampled = false;
    UsdTimeCode time = UsdTimeCode::Default();
    uint64_t stamp = 0;
    bool overrides = false;
    uint64_t chainSerial = 0;

    /// Whether the variance answers for key \p k (`varying[k]`, and for a
    /// skin's layout leaves GeomRevision::layoutFixed) may predate an edit:
    /// a notice the program could not route moved \p programStamp since the
    /// last sample, or a routed edit or a skipped sample marked the key.
    /// The next sample asks them again (RigExecBakedSamplePathLeaves,
    /// RigExecBakedSampleLayoutLeaves); a reader before it asks too
    /// (RigExecBakedLeafVaryingNow, RigExecBakedLayoutFixedNow). The bind
    /// answers them as well, so a table not yet sampled is current unless
    /// one of these holds.
    bool VarianceStale(uint64_t programStamp, size_t k) const
    {
        return stamp != programStamp ||
               (k < mustSample.size() && mustSample[k] != 0);
    }
    /// VarianceStale for any key.
    bool AnyVarianceStale(uint64_t programStamp) const
    {
        if (stamp != programStamp) {
            return true;
        }
        for (const char marked : mustSample) {
            if (marked != 0) {
                return true;
            }
        }
        return false;
    }
    /// Keeps the keys k with keep[k] != 0, in order, in `decl.keys` and every
    /// per-key vector of this table, and remaps `decl.roles`. False, changing
    /// nothing, when a key-range start (`scalarBegin`, `externalBegin`,
    /// `dialBegin`) is set. Scalars are untouched. Build only.
    bool Select(const std::vector<char> &keep);

    /// Key \p k's value as \p T, or \p fallback when \p k is out of range or
    /// holds another type.
    template <class T> T Value(int k, const T &fallback) const
    {
        if (k < 0 || size_t(k) >= values.size() ||
            !values[size_t(k)].IsHolding<T>()) {
            return fallback;
        }
        return values[size_t(k)].UncheckedGet<T>();
    }
};

/// Which path-leaf owner a path leaf id names (ids from
/// RigExecBakedProgramImpl::leafRefs.size() on).
enum class RigExecBakedPathLeafOwner : uint8_t {
    Revision,
    Derived,
    Weight,
    WeightOracle,
    /// The layout leaves of a chain revision's or a derived target's
    /// SkinTopology op.
    RevisionLayout,
    DerivedLayout,
    Provider,
};

/// One path leaf: its owner (chain and revision, chain and derived target,
/// or weight object) and its key there.
struct RigExecBakedPathLeafRef {
    RigExecBakedPathLeafOwner owner = RigExecBakedPathLeafOwner::Revision;
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t key = 0;
};

/// The provider leaves' publication tables, built with the operation graph
/// from Build-only state and shared, immutable, by every clone. A provider
/// leaf's key reads its sample and the overlay RigExecBakedSpaceLeafOverlay
/// selects; these name, per leaf, every table that overlay reads, so a run
/// re-keys only leaves whose key can have moved and the overlay skips the
/// map finds.
struct RigExecBakedSpaceLeafIndex {
    /// Provider leaf k is operation value `first + k`. The program's other
    /// leaves, re-keyed on every run, fall below or above that range.
    RigExecValueId first = 0;
    std::vector<RigExecValueId> before, after;
    /// Per leaf: its `headOverrideSlots` slot, or -1, and the override
    /// numbers `overridableInputs` files at its path, in that order:
    /// numbers[numberBegin[k], numberBegin[k + 1]).
    std::vector<int> headSlot;
    std::vector<uint32_t> numberBegin;
    std::vector<int> numbers;
    /// The same facts the other way round: (path, leaf) sorted by path, for
    /// the routed and upstream maps, and (number, leaf) and (slot, leaf).
    std::vector<std::pair<SdfPath, uint32_t>> byPath;
    std::vector<std::pair<uint32_t, uint32_t>> byNumber, byHeadSlot;
    /// RIGEXEC_VERIFY_SPARSE_LEAVES, read at Build: every leaf a run skips
    /// is re-keyed anyway and verified unchanged.
    bool verify = false;
};

/// Bits of RigExecBakedProgramImpl::spaceLeafRekey.
enum : uint8_t {
    /// The sampler moved the leaf's value or blocked flag since its key.
    kRigExecSpaceLeafSampled = 1,
    /// Its last key had an overlay in reach or was not exact, so the same
    /// sample can key differently.
    kRigExecSpaceLeafHeld = 2,
    /// An overlay can stand on it in this publication.
    kRigExecSpaceLeafOverlaid = 4,
};

/// What the source keys (RigExecBakedOpInputKey) read that a run can move,
/// built with the operation graph from compiled steps and shared, immutable,
/// by every clone: one entry per leaf, override slot or avar constant a
/// keyed op reads, with the ops reading it.
struct RigExecBakedSourceLeafIndex {
    enum class Kind : uint8_t {
        Typed,        ///< leaves pool `type` (RigExecBakedLeafType), entry `at`
        Head,         ///< headLeaves[at]: its type match and value
        PathVersion,  ///< pathLeafRefs[at], keyed by its content version
        PathValue,    ///< pathLeafRefs[at], keyed by its value
        Override,     ///< headOverrides[at]
        AvarConstant, ///< the constant of avarBindings[at] (`type` 0) or
                      ///< avarConstantBindings[at] (1)
    };
    struct Entry {
        Kind kind = Kind::Typed;
        uint8_t type = 0;
        uint32_t at = 0;
    };
    std::vector<Entry> entries;
    /// Entry w's readers: ops[begin[w], begin[w + 1]).
    std::vector<uint32_t> begin, ops;
    /// Per op: its key reads state no entry describes (a Constraint's native
    /// frames and delta bases), so every run rebuilds it.
    std::vector<char> untracked;
    /// The sizes the entries were classified and numbered by; another
    /// shape rebuilds every key.
    size_t leafRefs = 0, pathLeafRefs = 0;
    size_t avarBindings = 0, avarConstantBindings = 0;
    std::vector<size_t> pools;
    /// Reverse maps for a sparse visit: per pool (RigExecBakedLeafType
    /// order) and pool index, its Typed entry or -1; per head override slot,
    /// its Override entry or -1; every AvarConstant entry; every Head,
    /// PathVersion and PathValue entry, and any entry the maps cannot hold
    /// (always compared).
    std::vector<std::vector<int32_t>> typedEntry;
    std::vector<int32_t> overrideEntry;
    std::vector<uint32_t> avarEntries, alwaysEntries;
    /// Ops whose step always runs, ascending.
    std::vector<uint32_t> alwaysRunOps;
};

struct RigExecWeightOracleFacts {
    /// _VolumeWeightSamplesInFlight: a volume whose rigExec:weightTarget
    /// reads `preceding`.
    bool samplesInFlight = false;
    /// A volume not in flight: _ReadTargetPoints of rigExec:sampleSource,
    /// else of rigExec:weightTarget. False when neither reads.
    bool haveSamples = false;
    std::vector<GfVec3f> samples;
    /// A curve volume: _ReadTargetPoints of rigExec:curve, non-empty.
    bool haveCurve = false;
    std::vector<GfVec3f> curve;
    /// A plane volume's rigExec:planeAxis and rigExec:planeBounds as the
    /// oracle reads them: Get at the time, the fallback only when the
    /// attribute is absent. Empty for every other type.
    TfToken planeAxis, planeBounds;
    /// The first attribute whose answer these facts hold at one time while
    /// the oracle reads it at every evaluation time -- the points a volume
    /// not in flight samples, a curve volume's curve points, a plane's axis
    /// and bounds -- when it is animated (time samples, or a value that
    /// might vary); empty otherwise.
    SdfPath timeVarying;
    /// The oracle's whole error text when the read-phase check fails.
    std::string phaseError;
    /// The oracle's whole error text for the object's first structural
    /// failure: an unknown type, combine mode or range policy, a dynamic
    /// weight's operation, base, target, representation or sparse-support
    /// rule, or a static field carrying time samples or connections. Each
    /// precedes every per-run read of the object, so the port reports it
    /// on entering the object.
    std::string staticError;
};

// The program.

/// What a provider slot IS, which decides both what writes it and what the
/// walk may do with it.
///
/// The dynamic path keeps one frame map covering both families and tells them
/// apart by asking whether the path is in _nativeProviderPaths -- its
/// `hierarchicalProviders` set. The program answers the same question off a
/// dense table instead, because it asks it once per descendant per commit.
enum class RigExecBakedSlotKind {
    /// An exec-seeded RigExec provider: composed from avars, propagated to as
    /// a descendant, and the only kind a solver or the compose writes.
    FirstFramePose,
    /// A plain Xformable a constraint targets. The dynamic walk seeds it from
    /// the stage and revises it like any other target. Not in
    /// hierarchicalProviders: never propagated to, but can be the closest
    /// revised ancestor a RigExec descendant rides.
    XformDerived,
};

// The step graph.
// A run is a dependency graph of steps over dense slots rather than one
// straight line. PROGRAM ORDER -- the order the straight line used -- is still
// the reference: a step's index is its position in it, every edge points
// forward, and the serial executor runs the steps in index order. That is what
// makes "byte-identical to the serial run" a property any topological order of
// this graph has, because floating-point results depend only on operand values
// and each step's arithmetic is fixed.
// A step declares the slot RANGES it reads and writes; the edges are computed
// from those declarations alone (bakedSchedule.cpp). Declared writes are an
// UPPER BOUND: a disabled constraint, a revision that did not execute and a
// commit that passed through all write fewer slots than they declared, and no
// executor may ever "write what was declared".

/// Which dense table a slot indexes.
///
/// The specification calls this a slot KIND. That word is already spent in
/// this header on RigExecBakedSlotKind, which says what a PROVIDER slot is
/// (FirstFramePose or XformDerived), and two similarly named enums in one header is
/// how the wrong one gets declared -- so the step graph's is a slot's DOMAIN,
/// the table it indexes, and the provider one keeps its name.
enum class RigExecBakedSlotDomain : uint8_t {
    Avars,              ///< avars[i*11 .. +10]; filled by the prologue
    PoseBase,           ///< base[i]
    PoseFin,            ///< fin[i]
    PosedM,             ///< posedM[i]
    FinalMatrix,        ///< finalMatrix[i]
    BaseMatrix,         ///< baseMatrix[i]
    Aggregate,          ///< aggregates[s]
    SolverPoints,       ///< one ribbon's live driver points; the prologue
    Candidates,         ///< one Solve step's own (slot, frame) scratch
    CommitTable,        ///< one commit's merged candidate table
    CommitDelta,        ///< one commit's per-candidate hierarchy delta
    CommitStaging,      ///< one propagation pair's staged frame and outcome
    ConstraintDelta,    ///< one geometry-domain constraint's measured delta
    PropertyResult,     ///< propertyValues[v]; written by PropertyRevision
    ChainBase,          ///< chains[c].lastBase; filled by the prologue
    RevisionPacket,     ///< one revision's assembled packet, status, executed
    RevisionTransforms, ///< one revision's influence table
    RevisionOut,        ///< one revision's OWN output buffer
    RevisionDone,       ///< one revision's applied/status flags
    ChainDirty,         ///< the chain's sticky dirty bit as of one revision
    ChainPoints,        ///< the chain's published points
    DerivedOut,         ///< one derived target's output
    WeightPacket,       ///< one weight object's packet for this frame
    WeightFrames,       ///< where one volume weight is placed, by provider slot
    PoseWeight,         ///< poseWeights[k]: one pose interpolator's weights
    /// Reserved; never declared. The step-graph validator refuses a step
    /// that names it.
    Snapshots,
    /// frameMatrix[k] and frameMatrixValid[k]: one RigExecBakedFrameRecord.
    /// Appended after Snapshots, so every earlier enumerator keeps its
    /// exported value.
    FrameMatrix,
    Rest = 27,
    Ladder = 28,
    SkinTopology = 29,
    WeightField = 30,
    WeightFramesBase = 31,
    SpaceValue = 32,
    SpaceLeaf = 33,
    DerivedBase = 34,
    ChainInput = 35,
    ConstraintInputs = 36,
    SwitchFrame = 37,
    RequiredStageFramesAdmission = 38,
};
/// Derived from the last enumerator rather than written out: an array
/// indexed by domain is how the edge sweep is written, and a count that
/// drifted from the enum is an out-of-bounds write with no symptom at Build.
inline constexpr size_t RigExecBakedSlotDomainCount =
    size_t(RigExecBakedSlotDomain::RequiredStageFramesAdmission) + 1;

/// A slot id: the domain in the top 8 bits, the index in the low 24.
using RigExecBakedSlot = uint32_t;

inline RigExecBakedSlot
RigExecBakedMakeSlot(RigExecBakedSlotDomain domain, uint32_t index)
{
    return (uint32_t(domain) << 24) | (index & 0xffffffu);
}

/// A half-open run of slots of one domain.
///
/// Provider slots are in namespace DFS pre-order, so a subtree is contiguous
/// and a commit's descendants are a scan rather than a set; declaring ranges
/// rather than slots is what collapses the edge count and makes the edge
/// computation an interval sweep.
struct RigExecBakedSlotRange {
    RigExecBakedSlotDomain domain = RigExecBakedSlotDomain::Avars;
    uint32_t begin = 0;
    uint32_t end = 0;

    bool IsEmpty() const
    {
        return end <= begin;
    }
    bool Overlaps(const RigExecBakedSlotRange &other) const
    {
        return domain == other.domain && begin < other.end && other.begin < end;
    }
    bool operator==(const RigExecBakedSlotRange &other) const
    {
        return domain == other.domain && begin == other.begin &&
               end == other.end;
    }
    bool operator<(const RigExecBakedSlotRange &other) const
    {
        if (domain != other.domain)
            return domain < other.domain;
        if (begin != other.begin)
            return begin < other.begin;
        return end < other.end;
    }
};

inline RigExecBakedSlotRange
RigExecBakedRange(RigExecBakedSlotDomain domain, int begin, int end)
{
    RigExecBakedSlotRange range;
    range.domain = domain;
    range.begin = uint32_t(begin);
    range.end = uint32_t(end);
    return range;
}

inline RigExecBakedSlotRange
RigExecBakedOne(RigExecBakedSlotDomain domain, int index)
{
    return RigExecBakedRange(domain, index, index + 1);
}

// Head operations occupy the shared graph's prefix. Their leaves are sampled
// before their serial bodies; rest/layout sampling follows chain publication.

/// The type a head leaf holds and a walk reads.
enum class RigExecBakedHeadValueType : uint8_t {
    Bool,
    Float,
    Double,
    Vec3f,
    Matrix4d,
    Vec2fArray,
    Int,
    Token,
    Vec3d,
    Vec3fArray
};

/// One stage value a head op reads: an attribute's own value at the run's
/// time as one type, with no connection followed and no overlay consulted;
/// empty when the attribute has no value of that type then. Sampled on the
/// owning thread (RigExecBakedSampleHeadLeaves); leaf id
/// `leafRefs.size() + pathLeafRefs.size() + i` in `leafByPath`.
struct RigExecBakedHeadLeaf {
    SdfPath path;
    /// Invalid when no attribute stands at `path`. Owning thread only.
    UsdAttribute attribute;
    RigExecBakedHeadValueType type = RigExecBakedHeadValueType::Float;
    /// The attribute's value type is `type`. A typed read of another type
    /// has no value, so such a leaf stays empty and is never read.
    bool typeMatches = false;
    bool varying = false;
    VtValue value;
    /// This run's sample differs bitwise from the one before it.
    char changed = 0;
    /// The next sample re-reads it whatever else holds (a value edit
    /// reached `path`), and re-derives `varying`.
    char mustSample = 0;
    /// Advanced with every mark, which the live sample consumes: what the
    /// frozen samplers' constants table compares to re-read only the
    /// leaves an edit reached (_HeadLeafConstants).
    uint64_t editSerial = 0;
    /// The key a frozen job samples it under, set at Build for a leaf whose
    /// type matches (`<prim>.frozenChainOwn:<name>` for a chain target's own
    /// value, `<prim>.frozenChainHop:<name>` otherwise); empty for a leaf
    /// that never holds a value. Never `path`: a reader binding whose head
    /// is a chain target samples the chain's result at that path, and
    /// first-wins lookup would serve one to the other.
    SdfPath frozenKey;
};

/// Calls \p fn on every head leaf a frozen job samples (those with a
/// `frozenKey`), in leaf order: the sampler's emission order and the
/// worker's patch order.
template <class Program, class Fn>
inline void
RigExecForEachHeadLeaf(Program &program, Fn &&fn)
{
    for (auto &leaf : program.headLeaves) {
        if (!leaf.frozenKey.IsEmpty()) {
            fn(leaf);
        }
    }
}

/// One attribute of a walk, and what can stand in the overlay there when a
/// head op reads it.
struct RigExecBakedWalkHop {
    SdfPath path;
    /// `headOverrideSlots`' slot of `path`.
    int overrideSlot = -1;
    /// An earlier chain whose target `path` is: its final version stands
    /// here while the chain is valid, in place of any override.
    int chain = -1;
    /// An earlier chain's phased record whose consumer `path` is.
    int record = -1;
    /// The head leaf holding `path`'s own value as the walk reads it; -1
    /// where the walk never falls back to it.
    int leaf = -1;
    int poseWeight =
        -1; ///< poseWeights index of a pose interpolator's Float publication read in place of the overlay: weight-field scalar reads, and property-revision Float inputs connected at rigExecReadPhase "final"
    int crossDomain = -1; ///< explicit cross-domain pose/geometry version read
};

/// One input of a property revision as _EvaluatePropertyChains reads it
/// (_PinnedRead, rigEvaluatorProperties.cpp), restated over head leaves,
/// overrides and versions by RigExecBakedResolveWalk (bakedProperties.cpp).
/// The hops are fixed at Build: a connection, a type or an attribute
/// appearing on them moves the compiled rig signature, which recompiles.
struct RigExecBakedWalk {
    enum class Flavour : uint8_t {
        Absent,    ///< no attribute: the read's fallback
        Pinned,    ///< unconnected: the overlay at its path, else its value
        Connected, ///< RigExecResolvedInputs::GetAttribute's walk
    };
    Flavour flavour = Flavour::Absent;
    RigExecBakedHeadValueType type = RigExecBakedHeadValueType::Float;
    /// The hops read as `type`, in walk order. A float walk that meets a
    /// double hop ends `hops` there (that hop still answers a float overlay
    /// first) and goes on in `doubleHops`, GetAttribute's double recursion
    /// from that hop with its own cycle guard; a float walk whose head is a
    /// double has only `doubleHops`.
    std::vector<RigExecBakedWalkHop> hops, doubleHops;
};

/// A read after the head tier that RigExecResolvedInputs::GetAttribute
/// would answer over the published overlay, restated over head leaves,
/// overrides and versions: a chain-routed binding's walk, or a path leaf's
/// whose hops meet a chain target or a record consumer. Every chain and
/// record counts on its hops, since all have run. Bound at Build.
struct RigExecBakedReaderWalk {
    /// Always the Connected flavour: GetAttribute's walk from `head`, with
    /// the float-from-double recursion.
    RigExecBakedWalk walk;
    SdfPath head;
    /// The head's own value as the walk's type, for a path-leaf read that
    /// falls back to the raw attribute when the walk answers nothing; -1
    /// for a binding, which keeps its constant.
    int rawLeaf = -1;
    /// What the value depends on beside the time: the head leaves of its
    /// hops (and `rawLeaf`), their override slots, and the PropertyVersion
    /// ids it declares (each met chain's final version and each met
    /// record), sorted.
    std::vector<uint32_t> leaves, slots, versions;
    /// Declared final versions the walk reaches only past one of that
    /// chain's records, as (version id, record index): while the record
    /// does not stand aside it answers the walk (or, with the chain
    /// skipped, the target holds no version), so a move of the version
    /// alone moves nothing here (RigExecBakedStep::shadowedReads).
    std::vector<std::pair<uint32_t, uint32_t>> shadowed;
};

/// What RigExecBakedBuildOracleRead indexes over the whole program, kept
/// across the reads of one build. Head leaves and override slots only grow
/// between those reads and chains and records do not change, so each read
/// indexes only what came after the previous one. Without an index a read
/// indexes the whole program, which is quadratic over a build's weight reads.
struct RigExecBakedOracleReadIndex {
    std::map<std::pair<SdfPath, int>, uint32_t> leafOf;
    size_t leavesIndexed = 0;
    size_t leavesFrozen = 0;
    size_t slotsNamed = 0;
    /// Per hop path: (chain, PropertyVersion id) for each chain that targets
    /// it and each record that consumes it.
    std::unordered_map<SdfPath, std::vector<std::pair<int, uint32_t>>,
                       SdfPath::Hash>
        versions;
    size_t versionChains = 0, versionRecords = 0;
    bool versionsBuilt = false;
};

/// One version's value, in the member the chain's arm computes.
struct RigExecBakedPropertyValue {
    float f = 0.0f;
    double d = 0.0;
    GfVec3f v{0.0f};
    GfMatrix4d m{1.0};
};

/// A property chain as the head tier runs it, bound at Build from the
/// evaluator's compiled chain the way _EvaluatePropertyChains binds it
/// (rigEvaluatorProperties.cpp).
struct RigExecBakedPropertyChain {
    /// The runChain arm the target's type name selects.
    enum class Arm : uint8_t { Float, Double, Matrix4d, Vec3f };
    SdfPath target;
    bool targetExists = false;
    SdfValueTypeName valueType;
    Arm arm = Arm::Float;
    /// The target's own value (no connection, no overlay) as the arm's type.
    int ownLeaf = -1;
    int targetSlot = -1;
    /// The PropertyVersion id of part 0; part k writes versionBase + k.
    uint32_t versionBase = 0;
    struct Revision {
        SdfPath mover;
        bool moverExists = false;
        /// rigExec:operation as _ReadOperation reads it (its own value at
        /// Default), parsed at Build. Structural: an edit rebuilds.
        bool opValid = false;
        RigExecPropertyOp op = RigExecPropertyOp::Add;
        /// rigExec:weightObject's first target; empty when none.
        SdfPath weightObject;
        int weightField = -1;
        RigExecBakedWalk enabled, defaultWeight, value, minimum, maximum, keys,
            tangents;
        /// inputs:tangents exists, which is what decides it is read.
        bool hasTangents = false;
        /// The path operation: its curve's points (a Vec3fArray walk) and
        /// the settings Build read off the mover and the curve. Structural:
        /// an edit to what the settings came from rebuilds.
        RigExecBakedWalk pathPoints;
        RigExecPathSettings path;
        /// "diag <mover>", the head of every line the revision emits.
        std::string linePrefix;
    };
    std::vector<Revision> revisions;
    /// Its phased records, as indices into `propertyRecords`.
    std::vector<uint32_t> records;
    /// "property chain <target>", the head of every line the base emits.
    std::string linePrefix;
};

/// A phased record (RigExecPhasedConnection): its consumer reads the chain
/// after `applied` revisions, converted to `consumerType`, unless an
/// override stands on one of its hops.
struct RigExecBakedPropertyRecord {
    uint32_t chain = 0;
    SdfPath consumer;
    SdfValueTypeName consumerType;
    size_t applied = 0;
    /// The override slots of the consumer and every hop before the target.
    std::vector<int> hopSlots;
    /// Its PropertyVersion id.
    uint32_t id = 0;
};

/// One version of a chain's running points: 0 is the authored base, v > 0
/// what the fuse of the chain's revision v - 1 left.
struct RigExecBakedPointVersion {
    int chain = -1;
    int version = 0;
};

/// A phased point read, bound at Build to the versions the dynamic walk's
/// phased-read store would have answered from: the records of the revisions some phase
/// names (`snapshotAfter`) whose fuse -- or, for `final`, whose chain's
/// status step -- runs before the reader, matched to the phase as
/// the original AtPrim lookup matches them, newest first. `preceding` on
/// the reader's own chain is instead the one version entering the reader,
/// which the walk reads off the chain built so far. The reader
/// takes the first candidate whose chain read a base this run -- every
/// record of a chain is written exactly then -- and otherwise the tail: the
/// resolved input, with the "resolved to nothing" line when `diagnoseMiss`.
struct RigExecBakedPointsBinding {
    /// The chain target the reader reads, which the overlay sets.
    SdfPath input;
    /// As declared; the diagnostic names it.
    RigExecReadPhase phase;
    /// Versions per RigExecBakedPointVersion. For a `finalRead` the one
    /// candidate is the chain's published ChainPoints, at version = the
    /// chain's revision count.
    std::vector<RigExecBakedPointVersion> candidates;
    bool finalRead = false;
    /// `phases` reads other than `preceding`; never a blend sample.
    bool diagnoseMiss = false;
    /// The miss diagnostic, spelled at Build: a body may not ask SdfPath for
    /// text, which interns it under Sdf's table locks.
    std::string missDiagnostic;
    /// Dense over every binding of the program, for the test capture.
    int id = -1;
    /// The binding's own gather buffer for a version that is not one buffer
    /// (RigExecBakedResolvePoints); one reading step per binding.
    mutable std::vector<GfVec3f> gather;
};

/// One provider frame as one writer of its stack -- a constraint or a
/// solver -- left it, for an AtPrim read phase on a transform that names that
/// writer. A record exists only for a pair (provider, writer) the compile
/// named (`_nativePhaseCheckpoints`), exactly as the walk records only named
/// pairs; its FrameMatrix step writes the matrix and a valid byte that is 0
/// exactly where the walk declines to record (RigExecBakedEvalFrameRecord).
struct RigExecBakedFrameRecord {
    /// The provider slot.
    int slot = -1;
    /// The writer's commit: an index into `commits` / `walkSteps`.
    int commit = -1;
    /// A constraint's record: the provider's position in its `targetSlots`.
    /// -1 for a solver's.
    int target = -1;
    /// A solver's record: the provider's position in the commit's `slots`,
    /// whose `present` byte says whether the solver published it this run.
    /// -1 for a constraint's.
    int position = -1;
    /// The `fin` entry the record reads: the commit's last version of the
    /// slot where it declares one, else the version it read. Bound after
    /// BindPoseVersions.
    uint32_t version = 0;
    /// The constraint or solver, which the read phase names.
    SdfPath mover;
};

/// Test-only: one point binding's answer as its reader took it this run
/// (RigExecBakedProgramTesting::CapturePointReads). `read` is false when the
/// reader did not run.
struct RigExecBakedPointCapture {
    bool read = false;
    bool bindingAnswered = false;
    VtVec3fArray bound;
};

/// The name of \p domain, for the schedule report.
const char *RigExecBakedSlotDomainName(RigExecBakedSlotDomain domain);

/// Whether the prologue, and not a step, fills \p domain: a read of one needs
/// no producer in the graph. Aggregate is not one: a blend reads its inputs'
/// aggregates from this run, so their Solve steps must precede it.
inline bool
RigExecBakedIsSourceDomain(RigExecBakedSlotDomain domain)
{
    return domain == RigExecBakedSlotDomain::SolverPoints ||
           domain == RigExecBakedSlotDomain::SpaceLeaf ||
           domain == RigExecBakedSlotDomain::DerivedBase ||
           domain == RigExecBakedSlotDomain::ChainInput ||
           domain == RigExecBakedSlotDomain::ConstraintInputs ||
           domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission;
}

/// Whether \p domain's storage is SSA, one entry per writer.
///
/// The two pose frame domains are. Everything else has one writer per slot
/// per run, or is scratch a single step owns, so there is no version to
/// speak of: the entry IS the value. The distinction is the edge sweep's:
/// a write-after-read edge exists only where a writer can land on storage a
/// reader is still entitled to, and in a versioned domain it never can.
inline bool
RigExecBakedIsVersionedDomain(RigExecBakedSlotDomain domain)
{
    return domain == RigExecBakedSlotDomain::PoseFin ||
           domain == RigExecBakedSlotDomain::PoseBase;
}

/// What one step of the program does.
enum class RigExecBakedStepKind {
    ComposeSubtree, ///< one subtree of the provider forest
    Solve,          ///< one solver's aggregate and candidate frames
    SolverCommit,   ///< one solver batch's merged candidate table
    Constraint,     ///< one constraint's candidate frame
    CommitDelta,    ///< a split commit's per-candidate hierarchy deltas
    PropagateChunk, ///< a split commit's staged descendant frames
    CommitApply,    ///< a split commit's decision and write-back
    ProviderMatrix, ///< one provider's rest -> final or rest -> base matrix
    /// Reserved; never emitted.
    SnapshotFinals,
    PoseInterpolator, ///< one pose interpolator's weights, from the final pose
    VolumePlacements, ///< one volume weight's placement, from the walk
    WeightPacket,     ///< one weight object's packet, built once per frame
    InfluenceFold,    ///< one revision's influence table
    RevisionStatic,   ///< one revision's packet, status and executed decision
    RevisionChunk,    ///< one vertex range of one revision
    RevisionFuse,     ///< one revision's applied decision and chain dirty bit
    ChainStatus,      ///< one chain's status sweep and published points
    Derived,          ///< one derived target maintained from a chain
    /// One RigExecBakedFrameRecord. A pose step; appended after Derived so
    /// every earlier enumerator keeps its exported value.
    FrameMatrix,
    PropertyRevision = 19,
    RestCompose = 20,
    LadderCompose = 21,
    SkinTopology = 22,
    WeightField = 23,
    SpaceExpression = 24,
    AvarInputs = 26,
    SpaceCheckpoint = 27,
    ProviderRefresh = 28,
    ChainInputs = 25,
    /// File.op_graph's op nodes, run through the op core.
    OpGraph = 29,
};

/// The name of \p kind, for the schedule report.
const char *RigExecBakedStepKindName(RigExecBakedStepKind kind);

/// Whether \p kind belongs to the geometry half of a frame.
///
/// The executor uses it to pick a body and the epilogue to replay the two
/// halves' diagnostics where the published generation puts them: the walk's
/// before the joint block, the geometry's interleaved with the chains.
inline bool
RigExecBakedIsGeometryStep(RigExecBakedStepKind kind)
{
    switch (kind) {
    case RigExecBakedStepKind::ChainInputs:
    case RigExecBakedStepKind::WeightField:
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
    case RigExecBakedStepKind::ChainStatus:
    case RigExecBakedStepKind::Derived:
        return true;
    default:
        return false;
    }
}

/// How many diagnostics a walk step may emit in one run.
///
/// Every diagnostic site in the walk is terminal -- it passes the constraint
/// through or gives the generation back -- so a walk step's list is short and
/// bounded, which is what lets the epilogue replay it without ever growing a
/// shared vector inside the region. The two-line case is a constraint whose
/// source did not resolve: it names the source, then says the constraint
/// passed through. Geometry steps carry their own bound (a chain's status
/// sweep has one line per failed revision), which Build records per step.
inline constexpr size_t RigExecBakedMaxStepDiagnostics = 4;

/// What one step added to the generation's work counters.
///
/// Counted per step and summed in the epilogue rather than incremented on the
/// pose, because a step body never touches the pose.
struct RigExecBakedStepCounters {
    uint32_t revisionsExecuted = 0;
    uint32_t revisionsCreated = 0;
    uint32_t schedulesBuilt = 0;
    uint32_t chainsBuilt = 0;
    uint32_t revisionsBuilt = 0;

    void Clear()
    {
        *this = RigExecBakedStepCounters();
    }
};

/// One step of the program.
struct RigExecBakedStep {
    bool isHead = false;
    std::vector<uint32_t> leaves, overrideSlots, bindingLeaves;
    bool varyingLeaves = false;
    bool alwaysRuns = false;
    std::vector<std::string> lines;
    RigExecBakedStepKind kind = RigExecBakedStepKind::ComposeSubtree;
    /// What this step is about, by kind:
    ///   ComposeSubtree                              index into composeGroups
    ///   Solve                                       index into solvers
    ///   SolverCommit/Constraint/CommitDelta/
    ///     PropagateChunk/CommitApply                index into commits
    ///   ProviderMatrix                              provider slot
    ///   PoseInterpolator                            index into poseInterpolators
    ///   VolumePlacements                            provider slot (part 1)
    ///   WeightPacket                                index into weightObjects
    ///   InfluenceFold/RevisionStatic/
    ///     RevisionChunk/RevisionFuse                index into revisionIndex
    ///   ChainStatus                                 index into chains
    ///   Derived                                     index into derivedIndex
    ///   FrameMatrix                                 index into frameRecords
    int object = -1;
    /// The part of it, by kind: the vertex chunk of a RevisionChunk, the
    /// propagation-pair chunk of a PropagateChunk, 1 for a final-phase
    /// ProviderMatrix against 0 for a base-phase one, and 1 for the
    /// per-volume VolumePlacements form (the whole-map form, -1, is no
    /// longer emitted but keeps its wire meaning). -1 where unused.
    int part = -1;

    /// Sorted, deduplicated. Writes are an upper bound (see above).
    std::vector<RigExecBakedSlotRange> reads, writes;
    /// Program indices, sorted; every entry of preds is < this step's index.
    ///
    /// There is no second list of RESTORE predecessors beside this one.
    /// Versioned storage makes such a list empty by construction: no version is ever
    /// overwritten, so re-running a reader never means re-running a writer
    /// to put a slot back.
    std::vector<int> preds, succs;
    /// Declared authored prerequisites, separate from generated graph edges.
    /// Stable keys survive canonicalization and local SCC exclusion.
    std::vector<std::string> semanticPredecessorKeys;

    // A step is a pure function of its declared reads with three exceptions,
    // and every one of them is recorded here so that the dirty set can name
    // it rather than the executor having to guess.
    /// The step reads source slots or completed head outputs, so it can be
    /// run before the dirty set is computed, every run, and compared by
    /// VALUE. Heads themselves never participate in this source pass.
    bool isSource = false;
    /// The step reads outside the program at a point the graph cannot order
    /// -- a packet assembled against the influence table, a derived target's
    /// own inputs -- and is not a source because it has predecessors. Such a
    /// step's cluster is dirty every run; its own value comparisons are what
    /// keep the counters honest.
    bool externalReads = false;
    /// The step reads a baked input whose value is a function of TIME, so a
    /// frame at a different time may hand it different numbers.
    bool varyingInputs = false;
    /// The override indices of the baked inputs this step reads, sorted and
    /// deduplicated. A step is dirty while an override stands on one of
    /// them, and for the one run after it is lifted.
    std::vector<int> overrideInputs;
    /// The reader walks (RigExecBakedProgramImpl::readerWalks) of the
    /// inputs and path leaves this step reads, sorted and deduplicated, and
    /// the head outputs it reads: the PropertyVersion ids those walks
    /// declare, then the Rest and Ladder slots its body indexes. Not
    /// exported; a version that moved dirties the step
    /// (RigExecBakedCones::headReaders), and so does a walk whose value
    /// moved (`walkReaders`) and a Rest or Ladder slot that moved
    /// (`restReaders`, `ladderReaders`).
    std::vector<int> readerWalks;
    std::vector<std::pair<RigExecBakedLeafType, uint32_t>> walkedBindingLeaves;
    /// The `reads` ids every walk of the step that declares them reads
    /// only past a record (RigExecBakedReaderWalk::shadowed), with those
    /// records: such an id seeds the step only while one of them stands
    /// aside.
    std::vector<std::pair<uint32_t, uint32_t>> shadowedReads;

    /// Filled by the clustering pass; the serial executor ignores all four.
    /// `level` is the longest-path level the packing groups by, `sizeUnits`
    /// the size term of the cost model and `cost` the microseconds
    /// that model predicts.
    int cluster = -1;
    int level = 0;
    double sizeUnits = 0;
    double cost = 0;

    /// What the calibration mode measured: the summed interval of this step
    /// over the frames it watched, and how many of them it saw. Untouched
    /// unless RIGEXEC_BAKED_SCHEDULE_CALIBRATE or RIGEXEC_BAKED_STEP_TIMING
    /// asked for a measurement, and written only by the thread that runs the
    /// step -- which is what "lock-free per-step timer" comes to.
    double measuredUs = 0;
    uint32_t measuredRuns = 0;
    /// Beside it, under the same requests: the summed memo of every run that
    /// evaluated this op's memo (one that then skipped the body included),
    /// how many did, and the summed value publication of the runs that ran
    /// the body. Folded from the stamps below by the executor's owner after
    /// the join.
    double measuredMemoUs = 0, measuredPublishUs = 0;
    uint32_t measuredMemoRuns = 0;

    /// This run's output, all of it per step so that nothing in a body
    /// touches shared state.
    std::vector<std::string> diagnostics;
    size_t maxDiagnostics = RigExecBakedMaxStepDiagnostics;
    RigExecBakedStepCounters counters;
    /// When the profiler is on: the step's interval, replayed into it by the
    /// epilogue in step order so the trace is deterministic.
    uint64_t startUs = 0, endUs = 0;
    /// Thread that ran this body, retained for profiler replay.
    std::thread::id runner;
    /// This run's op stamps in nanoseconds on the same steady clock, written
    /// only by the thread that runs the op: memo start and publication end
    /// while op timing, the profiler or a measurement is on; memo end and
    /// body end only for a measurement. The op holds its thread from
    /// memoStartNs to publishEndNs. Cleared with the body interval at the
    /// start of the next run when a run listed it (`stampedSteps`).
    uint64_t memoStartNs = 0, memoEndNs = 0, bodyEndNs = 0, publishEndNs = 0;
    /// What the step is called in the report and the trace, built once at
    /// Build so that neither costs a string per step per frame.
    std::string label;
    std::string
        descriptorKey; ///< owner/category/authored revision/subindex identity

    /// Empties this run's output. Called by the executor before the body, so
    /// that a step that is skipped keeps last run's lines for the epilogue
    /// to replay. The timestamps are NOT cleared here for that same reason:
    /// a skipped step never reaches this, so RigExecBakedClearRunStamps
    /// clears the stamped steps' intervals at the start of the run instead.
    void BeginRun()
    {
        diagnostics.clear();
        counters.Clear();
    }

    /// Records that this run skipped the step.
    ///
    /// The diagnostics stay, and so do the two counters that are PROGRAM
    /// CONSTANTS: a skipped step's lines describe state nothing changed, and
    /// how many chains and revisions the program holds is a property of the
    /// program rather than of a run. What does not stay is the three
    /// counters that are OBSERVATIONS of this run -- a revision the frame
    /// did not execute did not execute, a node it did not report did not
    /// report its creation, a schedule it did not build did not build one.
    /// These observations are reported by the operations that do the work.
    void MarkSkipped()
    {
        counters.revisionsExecuted = 0;
        counters.revisionsCreated = 0;
        counters.schedulesBuilt = 0;
    }
};

/// The readers RigExecBakedCones::editRoute names per override index.
enum : uint8_t {
    kEditRouteStep = 1,
    kEditRouteAvar = 2,
    /// A ladder channel: the rest and ladder ops that read its leaf re-run,
    /// and their moved outputs seed their readers (RigExecBakedHeadSeeds).
    kEditRouteHead = 4,
};

/// The cone table of a program whose cones were never built. Namespace
/// scope, so reading it takes no initialization guard.
inline const std::vector<RigExecBakedClusterSet> kRigExecBakedNoCones{};

/// What Build knows about re-running part of a program.
///
/// Both families are closures computed once, over clusters rather than over
/// steps, because a cluster is what the executor can skip: `cone[c]` is
/// everything that has to run when c does, and `restore[c]` is everything
/// that has to run BEFORE c can, so that the slots c reads hold the version
/// its program point saw rather than the version the end of the last run
/// left behind.
struct RigExecBakedCones {
    /// Forward closure of each cluster, including itself. Built once by
    /// RigExecBakedBuildCones and immutable after; program copies and the
    /// imaging bridge's affected index share it. Null before BuildCones.
    ///
    /// The only closure there is: versioned storage needs no restore
    /// closure.
    std::shared_ptr<const std::vector<RigExecBakedClusterSet>> cone;
    /// *cone, or kRigExecBakedNoCones when null.
    const std::vector<RigExecBakedClusterSet> &Cone() const
    {
        return cone ? *cone : kRigExecBakedNoCones;
    }
    /// Clusters holding a step that reads outside the graph at a point the
    /// graph cannot order. Dirty every run.
    RigExecBakedClusterSet always;
    /// Clusters holding any step that is not a geometry step. The FIRST run
    /// of a program is dirty here and in the geometry clusters of the
    /// revisions a rebuild did not carry, rather than everywhere.
    RigExecBakedClusterSet poseClusters;
    /// Provider slot -> the cluster of the compose step that reads its
    /// avars, or -1. What an avar that moved makes dirty.
    std::vector<int> avarCluster;
    /// Chain -> the clusters of every step that reads its base points.
    std::vector<std::vector<int>> chainBaseClusters;
    /// Solver -> the clusters of every step that reads its driver points.
    /// What a ribbon's moved driver curve makes dirty, and nothing else.
    std::vector<std::vector<int>> solverPointsClusters;
    /// Dense revision id -> the clusters of every step of that revision, and
    /// the cluster of its RevisionStatic alone.
    std::vector<std::vector<int>> revisionClusters;
    std::vector<int> revisionStaticCluster;
    /// Native source -> the clusters of every constraint step that reads the
    /// frame the prologue read for it. What a moved stage transform under a
    /// constraint source makes dirty.
    std::vector<std::vector<int>> nativeSourceClusters;
    /// Geometry-domain delta base -> the cluster of the constraint step that
    /// measures against it.
    std::vector<std::vector<int>> deltaBaseClusters;
    /// Constraint-array entry -> the cluster of the constraint step that
    /// reads it.
    std::vector<std::vector<int>> constraintArrayClusters;
    /// Steps whose dirtiness depends on time or on a standing override.
    std::vector<int> varyingSteps, overrideSteps;
    /// PropertyVersion id -> the steps that declare it in `reads`, and
    /// reader walk -> the steps that read it. What a moved version and a
    /// moved walk value dirty.
    std::vector<std::vector<int>> headReaders, walkReaders, fieldReaders;
    std::vector<int> fieldSteps;
    /// Provider slot -> the steps that declare its Rest, and its Ladder, in
    /// `reads`. What a moved rest or ladder output dirties.
    std::vector<std::vector<int>> restReaders, ladderReaders;
    /// Override index -> what re-reads an input of that number when the
    /// stage value under it moves: a step that lists it in its
    /// `overrideInputs` (kEditRouteStep), the prologue's avar table
    /// (kEditRouteAvar), or the rest and ladder ops (kEditRouteHead). Zero
    /// where nothing does -- a value read into the prologue and handed to a
    /// step no dirty set can name -- and an edit there bumps the program
    /// stamp instead of being routed.
    std::vector<uint8_t> editRoute;

    // What a live run decides with. A cluster is the unit a task runs, not
    // the unit that moved: one dirty step does not make its cluster-mates
    // dirty, because they read exactly what they read last run and versioned
    // storage left it where it was. So the live closure is taken over
    // `step.succs` and a run skips every step outside it, including the clean
    // members of a cluster it does dispatch.
    // The per-cluster tables above are KEPT beside these rather than derived
    // from them on demand: the output-affected index, the sparse frame-cache
    // planner and the frozen clone read them, and each wants a cluster
    // answer. Every entry below is the step-grain twin of the entry of the
    // same name above. Invariant: step and cluster tables list the same seeds.

    /// Steps that read outside the graph (dirty every run), and steps that
    /// are not geometry steps (dirty on a program's first run).
    RigExecBakedClusterSet alwaysSteps, poseSteps;
    /// Provider slot -> the compose step that reads its avars, or -1.
    std::vector<int> avarStep;
    /// Provider slot -> the OTHER compose steps that declare a read of its
    /// avars: a switched group recomposing an earlier version of an
    /// ancestor (RigExecBakedProgramImpl::SpaceSwitch::FrameVersion).
    /// Whatever dirties avarStep dirties these too. Empty on most rigs.
    std::vector<std::vector<int>> avarVersionSteps;
    std::vector<std::vector<int>> chainBaseSteps;
    std::vector<std::vector<int>> solverPointsSteps;
    std::vector<std::vector<int>> revisionSteps;
    std::vector<int> revisionStaticStep;
    std::vector<std::vector<int>> nativeSourceSteps;
    std::vector<std::vector<int>> deltaBaseSteps;
    std::vector<std::vector<int>> constraintArraySteps;
};

/// One contiguous group of provider slots the compose pass runs as one step.
struct RigExecBakedComposeGroup {
    int begin = 0, end = 0;
    /// The PosedM slots below `begin` that slots inside the group inherit
    /// from. More than one entry needs a plain
    /// Xformable between a provider and its compose ancestor.
    std::vector<int> parentSlots;
};

/// Everything one commit -- a solver batch's or a constraint's -- needs.
///
/// The candidate table is DENSE and in slot order, because both halves of
/// the commit walk it in slot order: the usability sweep and the write-back.
/// Which position each producer writes is decided at Build, so the merge is
/// an indexed store rather than a map insertion, and "last writer wins on a
/// duplicate slot" is the rule.
///
/// A solver STACK is two commits on one slot, never two producers inside one
/// commit: each solver batch holds exactly one solver, so each write of a
/// stacked joint gets its own SSA version. Build refuses duplicate slots in
/// one batch (bakedPose.cpp).
struct RigExecBakedCommit {
    /// One provider above a NATIVE Xformable source: the slot, and the
    /// versions of its base and final frames live where this commit runs.
    /// The deepest one whose points moved is the revision such a source
    /// rides (RigExecApplyRevisedAncestorDelta).
    struct AncestorRead {
        int slot = -1;
        uint32_t fin = 0;
        uint32_t base = 0;
    };
    /// Empty for a solver batch, whose diagnostics carry no mover path.
    SdfPath moverPath;
    /// moverPath spelled at Build, for the body's diagnostics.
    std::string moverPathText;
    bool solverOutput = false;
    /// Sorted, unique.
    std::vector<int> slots;
    /// Filled this run, in `slots` order.
    std::vector<char> present;
    std::vector<RigExecPointFrame> frames;
    /// (descendant, closest revised ancestor), in the order the dynamic walk
    /// enumerates them.
    std::vector<std::pair<int, int>> propagate;
    /// For each pair, the position of its `closest` in `slots`, or -1 when
    /// the batch declares no candidate for it at all.
    std::vector<int> closestPos;
    /// Per candidate position: the hierarchy delta and whether it resolved.
    std::vector<GfMatrix4d> deltas;
    std::vector<char> deltaOk;
    /// Per pair: the staged frame and what happened to it.
    std::vector<RigExecPointFrame> staged;
    std::vector<uint8_t> outcome;
    /// The candidate set was not produced at all this run -- a disabled
    /// constraint, an unusable candidate, a solver batch that published
    /// nothing -- so the whole commit writes nothing.
    bool abandoned = true;
    /// Build's decision to run the commit as three steps rather than one.
    bool split = false;
    /// Where this commit's pairs start in the CommitStaging domain. The
    /// staging scratch is per commit, but the slot ids may not be: two split
    /// commits both declaring [0, 64) would make the edge sweep order their
    /// chunks against each other and hide a real conflict behind a false one.
    int stagingBase = 0;
    /// A constraint step's source scratch, sized at Build.
    std::vector<RigExecConstraintSource> sources;

    // Every index below is into `fin` / `base`, and every one of them was
    // decided at Build: a read names the VERSION of a slot that was live at
    // this commit's point in the program, and a write names storage no other
    // step writes. Which is why a re-run of this commit cannot need an
    // earlier step re-run first to put a slot back the way it found it --
    // nothing ever puts it back, because nothing else writes there.
    /// Per candidate position: the version the delta is measured against and
    /// the version the write-back produces, in `fin` and (solver commits
    /// only) in `base`.
    std::vector<uint32_t> slotReads, slotWrites, slotBaseWrites;
    /// Per propagation pair: the descendant's version and its closest
    /// revised ancestor's, then the descendant's own new versions.
    std::vector<uint32_t> descendantReads, closestReads;
    std::vector<uint32_t> descendantWrites, descendantBaseWrites;
    /// Per write site, the version whose value stands where the site does
    /// not write one -- a candidate the batch published nothing for, a pair
    /// that was stepped over, a commit that passed through. Usually the read
    /// beside it; not always, because a slot can be both a candidate and a
    /// propagated descendant and the second write carries the first.
    std::vector<uint32_t> slotCarry, slotBaseCarry;
    std::vector<uint32_t> descendantCarry, descendantBaseCarry;
    /// A constraint's own reads: one per source, plus the world-up object,
    /// plus the target frame it solves FROM -- which a geometry-domain
    /// constraint reads without ever declaring the target a candidate.
    std::vector<uint32_t> sourceReads;
    uint32_t worldUpRead = 0;
    uint32_t targetRead = 0;
    /// Per target, the version live where this commit runs. Used to record
    /// the target of a commit that declares no candidate for it.
    std::vector<uint32_t> targetReads;
    /// The SingleChainIK bindings' reads, in the same two shapes a source's
    /// are: the version of a slot, and the ancestors of a native Xformable.
    uint32_t effectorRead = 0;
    std::vector<AncestorRead> effectorAncestors;
    std::vector<uint32_t> poleReads;
    std::vector<std::vector<AncestorRead>> poleAncestors;
    /// Scratch for the chain the solver is handed and the chain it returns.
    /// The first three are sized at Build; `ikSolved` cannot be, because the
    /// shared solve kernel RETURNS its chain by value and the assignment
    /// takes that buffer -- reserving here would only be discarded. It is
    /// the one per-frame allocation a SingleChainIK step makes, and closing
    /// it means giving the kernel an out-parameter form, which is a change
    /// to a kernel the dynamic path calls too.
    std::vector<RigExecPointFrame> ikChain, ikPrepared, ikRest, ikSolved;
    /// Whether this run's exit records the target's frame for a read phase.
    /// True for every transform-domain exit and for a geometry-domain
    /// constraint that never got as far as solving; false once a
    /// geometry-domain constraint has its sources, which is where the
    /// dynamic walk stops recording for one.
    bool recordAfter = true;
    /// And HOW MANY targets that exit records. The dynamic walk is not
    /// uniform about it: its early exits -- disabled, an unusable weight
    /// object, a bad envelope, a dormant one -- loop every target, and so
    /// does the SingleChainIK exit, whose targets ARE the solved chain; but
    /// its two late exits, unusable sources and the ordinary one, record
    /// targets[0] alone. Only a multi-target NON-IK constraint can tell the
    /// two apart, and no rig in the tree is one -- but the program must not
    /// invent a snapshot the reference path never published. Both flags are
    /// what a FrameMatrix step reads through CommitTable (and what the
    /// exporter carries).
    bool recordEveryTarget = true;
    /// Parallel to `sources`, and empty for a source the walk holds a frame
    /// for: the ancestor slots of a native source, in increasing depth.
    std::vector<std::vector<AncestorRead>> sourceAncestors;
    std::vector<AncestorRead> worldUpAncestors;
};

/// What a staged propagation pair turned into. Ordered so that the first
/// non-Staged, non-Skipped outcome in pair order decides the commit.
using RigExecBakedPropagateOutcome = RigExecPosePropagateOutcome;

/// A revision's place in its chain (Build state).
enum class RigExecBakedRevisionRole : uint8_t {
    /// Not in a range-pipelined chain: chunks and fuse, with no vertex groups.
    Legacy,
    /// One RevisionChunk step per written group and a join.
    Range,
    /// Decided whole: speculative chunks (one per group for a keyed skin,
    /// else one) and a fuse that decides and publishes every group.
    Whole,
};

/// The most vertex groups one range chain is cut into when
/// RIGEXEC_BAKED_GROUP_CAP is unset; Live and Export alike.
inline constexpr size_t RigExecBakedDefaultGroupCap = 32;

/// One fact a Range role, a Whole role chosen by value, or a group gate rests
/// on (Build state); RigExecBakedRolesStand checks them.
struct RigExecBakedRolePin {
    enum class Kind : uint8_t {
        LeafToken,         ///< `leaves` key `leaf` still holds `token`
        WeightDefaultZero, ///< weightObjects[weightObject]'s resolved
                           ///< unlisted value reads exactly 0
        NoLayoutOverride,  ///< no override or upstream key reaches the
                           ///< skin's JointIndices / ElementSize hops
    };
    Kind kind = Kind::LeafToken;
    int leaf = -1;
    TfToken token;
    int weightObject = -1;
};

// inputs:sourceWeights and the parent offsets are read RAW off the
// attribute -- no connection walk, no resolved input, no interactive
// override -- because that is what the dynamic walk does with them, and
// an operator input is not a rig input. The read is USD, so it is the
// PROLOGUE's, kept as compiled rig state (`sampled` below) and taken again
// only where a read at the frame's time could answer differently; the
// cardinality diagnostic belongs to the constraint step, at the
// constraint's own place in the walk, so what the prologue leaves
// behind is the line rather than the pose it would have gone into.
struct RigExecBakedConstraintArray {
    UsdPrim prim;
    SdfPath path;
    /// `path` spelled at Build, for the body's diagnostics.
    std::string pathText;
    std::array<VtValue, 4>
        raw; ///< source weights/translation/rotation/pole raw samples
    size_t sourceCount = 0;
    bool parentOffsets = false;
    std::vector<double> weights;
    std::vector<GfVec3d> translationOffsets, rotationOffsets;
    /// At most one line: the dynamic walk's buildSources stops at the
    /// first array whose cardinality is wrong.
    std::vector<std::string> diagnostics;
    bool ok = true;
    /// What the run before read. A source is compared by VALUE.
    std::vector<double> lastWeights;
    std::vector<GfVec3d> lastTranslationOffsets, lastRotationOffsets;
    std::vector<std::string> lastDiagnostics;
    bool lastOk = true;
    /// inputs:poleVectorWeights, read the same way for a SingleChainIK
    /// in object pole mode. Separate because it is read at a different
    /// point in the walk and owns its own diagnostic.
    bool readPole = false;
    size_t poleCount = 0;
    std::vector<double> poleWeights, lastPoleWeights;
    std::vector<std::string> poleDiagnostics, lastPoleDiagnostics;
    bool poleOk = true, lastPoleOk = true;

    /// The four channels in `raw` order (Names()), taken off `prim` at
    /// Build, and the frozen samplers' keys for them. A handle is what
    /// prim.GetAttribute answers: its validity is asked at every read,
    /// so a property authored later reads through it.
    std::array<UsdAttribute, 4> attributes;
    std::array<SdfPath, 4> keys;
    static const std::array<TfToken, 4> &Names();
    /// Whether the frozen samplers read channel \p k: the weights
    /// always, the parent offsets for a ParentConstraint, the pole
    /// weights where the step reads them (frozenSnapshot arrayKeys).
    bool Sampled(size_t k) const
    {
        return k == 0 || (k < 3 ? parentOffsets : readPole);
    }
    /// Whether channel \p k's read can move with the time: the
    /// predicate RigExecBakedClassifyInput applies (neither half
    /// implies the other: a single time sample; a Ts spline).
    bool Varies(size_t k) const
    {
        const UsdAttribute &a = attributes[k];
        return a && (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0);
    }
    /// The prologue's compiled rig state, owner thread only. Every channel is
    /// read again on a forced run and whenever the program stamp or
    /// the evaluator's stage edit serial moved since `sampled`: every
    /// stage notice advances the serial, so between them only the time
    /// can move a raw read. Then a channel is read again only when the
    /// time moves and its read can move with it (`variance`, asked on
    /// the first such move after a full read), or the time moves to or
    /// from Default, which read different opinions.
    enum : uint8_t { kVarianceUnknown = 0, kFixed, kVaries };
    std::array<uint8_t, 4> variance{};
    bool sampled = false;
    UsdTimeCode time = UsdTimeCode::Default();
    uint64_t stamp = 0;
    uint64_t serial = 0;
};

// A constraint source (or aim world-up object) that is neither a
// RigExecControl nor a RigExecJoint is read off the stage, exactly as a
// target is -- and then RIDDEN on the revision of the deepest provider
// above it that the walk has already moved. The stage read is the
// prologue's; the ride is the constraint step's, out of declared slots.
struct RigExecBakedNativeXformSource {
    SdfPath path;
    /// Every slot that is a STRICT namespace prefix of `path`, in
    /// increasing depth. Which of them is the revised one is a per-frame
    /// question the step asks; which of them could be is namespace
    /// topology and is settled here.
    std::vector<int> ancestorSlots;
};

// The rest chain and the default-space ladder, resolved once at Build
// and again on any frame that can move them. Every channel below is a
// BOUND input rather than a folded constant, which is what lets an
// animated, connected or chain-written rest bake and what lets a drag
// on one be placed; when nothing varies and nothing is dragged the
// Build-time answer stands and the frame path never looks at any of
// them.
struct RigExecBakedLadder {
    RigExecBakedInput<GfMatrix4d> restSpace, defaultSpace, posedSpace;
    RigExecBakedInput<GfMatrix4d> parentSpace, parentDefaultSpace,
        avarDefaultSpace, posedDefaultSpace;
    RigExecBakedInput<GfVec3d> rotationSign;
    RigExecBakedInput<GfMatrix4d> interveningSpace;
    bool interveningReset = false;
    std::array<int, 7> spaceValues{{-1, -1, -1, -1, -1, -1, -1}};
    bool posedSpaceConnected = false, defaultSpaceConnected = false;
    bool parentSpaceConnected = false, parentDefaultSpaceConnected = false;
    bool avarDefaultSpaceConnected = false, posedDefaultSpaceConnected = false;
    /// rest:tx/ty/tz/rx/ry/rz and default:tx/ty/tz/rx/ry/rz, in that
    /// order, which is the order RigExecBakedComposeAvars takes them in.
    RigExecBakedInput<double> restAvars[6];
    RigExecBakedInput<double> defaultAvars[6];
    RigExecBakedInput<TfToken> rotationOrder;
};

// A RigExecPoseInterpolator reads the FINAL pose of its driver and writes
// floats the geometry chains consume, so on the dynamic path it is a
// phase of its own between the pose walk and the chains. Here it is a
// step: it reads its driver's (and the driver's parent's) last PoseFin
// version and writes a range of PoseWeight slots, and a blend channel
// whose inputs:weight is connected to one of those reads the slot. The
// edges follow, and so does the cone -- a drag that cannot reach the
// driver leaves every corrective it drives untouched.
struct RigExecBakedPoseInterpolator : RigExecPoseInterpolatorRecord {
    SdfPath path;
    /// `path` spelled at Build, for the body's diagnostics.
    std::string pathText;
    int driverSlot = -1;
    /// The nearest frame-publishing ancestor the driver's local rotation
    /// is measured against, or -1 when its local rotation is its world
    /// one. Compiled by the evaluator, restated here as a slot.
    int parentSlot = -1;
    /// A RigLogic solve's frame drivers after the first, and each one's
    /// measuring ancestor (-1 for none), parallel.
    std::vector<int> extraDriverSlots;
    std::vector<int> extraParentSlots;
    /// A NUMERIC driver (rigExec:driverAttributes): the dials read in
    /// place of a transform's translation, one per axis, bound like any
    /// other per-frame input so a dragged or animated one is honoured.
    /// Empty on a transform-driven interpolator.
    std::vector<RigExecBakedInput<double>> valueInputs;
    /// ML movers and animated maps bind a float property as a float:
    /// where valueIsFloat is set, floatValueInputs holds the binding and
    /// valueInputs the unbound default, and the other way round.
    std::vector<RigExecBakedInput<float>> floatValueInputs;
    std::vector<char> valueIsFloat;
    std::vector<double> values;
    /// Animated maps: weight slots republishing a map's value as a mesh
    /// primvar, and the map (an index into poseSlots) each one copies.
    std::vector<int> primvarSlots;
    std::vector<int> primvarSources;
    /// `inputs:enabled`, a per-frame input the prologue reads into
    /// `enabledValue` so the body touches no USD.
    RigExecBakedInput<bool> enabled;
    bool enabledValue = true;
    /// A RigExecMlMover's `inputs:mask` (a float), bound only when
    /// mlSolve.
    RigExecBakedInput<float> mask;
    double maskValue = 1.0;
    /// This interpolator's slots in poseWeights: the whole range it
    /// writes, the subset the solver fills in ITS pose order, and the
    /// disabled poses' slots, which publish a hard zero.
    int weightBegin = 0, weightEnd = 0;
    std::vector<int> poseSlots;
    std::vector<int> disabledSlots;
    /// The solved table, copied from the evaluator's compiled record: a
    /// constant of the compiled rig, and the program is dropped with the compiled rig.
    RigExecPoseInterpolatorInputs kernelInputs;
    /// The solve's output, retained so a frame allocates nothing.
    std::vector<double> scratch;
};

struct RigExecBakedAvarBinding {
    size_t slot = 0;
    RigExecBakedInput<double> input;
};

struct RigExecBakedSolver : RigExecSolverRecord {
    RigExecSolverInputs kernelInputs;
    RigExecSolverWorkspace kernelWorkspace;
    SdfPath path;
    /// `path` spelled at Build, for the body's diagnostics.
    std::string pathText;
    TfToken type;
    /// Exact supported schema port and compiled target solver identity.
    /// This is a structural prerequisite, not a fabricated numerical read.
    std::vector<std::pair<std::string, int>> relationshipRequirements;
    /// Canonical Solve identity, retained even when the common SCC excludes it.
    std::string solveDescriptorKey;
    // Exec rebuilds every one of the rest members below from the
    // compiled rig's rests on EVERY evaluation, because it is a pure function
    // of them. The bake resolves it once, and the Solve step resolves it
    // again (RefreshSolverRests) on a run on which one of its
    // `restSlots` moved -- which is what lets a solver measure against
    // an animated, chain-written or dragged rest instead of refusing
    // the rig.
    /// Every provider slot whose rest this description folded in.
    std::vector<int> restSlots;
    /// (slot, element) for the two computations that remap by element:
    /// TwoBoneIk and SplineIk.
    std::vector<std::pair<int, int>> restRefs;
    /// Parallel to restRefs: true where a pose step BELOW this solver in
    /// the rig hierarchical stack wrote that joint, so the rest is the
    /// frame that step left rather than the authored one.
    /// Parallel to restRefs: the `fin` version each LIVE rest reads, bound
    /// by bindSolverReads to the version standing where this batch begins
    /// -- the same moment the dynamic path reads finalFrames for its
    /// computeRestFrame override.
    std::vector<unsigned> restReads;
    /// True when any rest ref is live, which makes the description
    /// per-frame: it is refreshed on EVERY evaluation and can never be
    /// concluded constant.
    bool hasLiveRest = false;
    /// Test hook: how many times the Solve step refreshed this
    /// description, the cone verifier's second pass excluded. Written
    /// only by this solver's own Solve step; read by no step.
    uint64_t restRefreshes = 0;
    /// One per rest ref, in element order: the basis a solver APPLIES its
    /// solved map to. For RigExecFkChain that is the joint's rest
    /// reference rather than the control's own rest, so a step below the
    /// chain is carried through the solve instead of replaced.
    /// Empty when the solver's joints do not line up one-for-one with its
    /// elements, and the solver then keeps its own basis.
    /// True when any slot's whole rest CHAIN can move with time.
    bool restsVary = false;
    /// Every override index a rest channel of that chain registered, so
    /// a drag on one dirties this solver's step.
    std::vector<int> restOverrides;
    /// The bake found this solver's rigExec:joints / jointElements
    /// binding malformed in one of the ways its exec computation checks
    /// at runtime. Such a computation warns and returns an EMPTY
    /// aggregate, so the program publishes one too: every joint the
    /// solver names then falls back to its rest chain, with the
    /// diagnostic that carries.
    // FkChain
    std::vector<int> controls;
    /// rigExec:startFrame provider slot (-1 when unwired or not a
    /// provider, the computation's silent absolute path), its rest,
    /// and the `fin` version bound where this batch begins -- the
    /// baked mirror of the computation's synthetic base element.
    int start = -1;
    uint32_t startRead = 0;
    // IK / spline controls
    int root = -1, mid = -1, end = -1, pole = -1;
    // TwoBoneIk: rests and the measured bone lengths, both compiled rig constant
    RigExecBakedInput<double> bend, upperOffset, lowerOffset;
    RigExecBakedInput<float> stretch, softness, pin, softDistance, limbTwist;
    RigExecBakedInput<double> upperScale, lowerScale;
    double upperLengthBase = 0, lowerLengthBase = 0;
    /// rigExec:spaceMatrix: an explicit factor, composed after the
    /// space prim below.
    RigExecBakedInput<GfMatrix4d> ikSpace;
    /// rigExec:space: the prim whose movement from its own rest is the
    /// space the chain is measured in. Read as a frame, exactly like
    /// the root/effector/pole, so an animated master is handled and
    /// the baked path cannot disagree with the dynamic one.
    int spaceSlot = -1, spaceRead = -1;
    /// The rest frames a spline needs to rebuild its rest description
    /// when the space has moved. Kept because the bind-time
    /// splineRest is measured at identity.
    std::array<GfVec3d, 4> spaceRest = {GfVec3d(0), GfVec3d(1, 0, 0),
                                        GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    // BlendPointFrames
    int inA = -1, inB = -1;
    RigExecBakedInput<float> blendWeight;
    /// An unsupported rigExec:rotationBlend, which is NOT a
    /// `degenerate`: the computation returns the surviving input before
    /// it ever looks at the token, so the rejection only bites when
    /// BOTH inputs are bound. It is checked where the computation
    /// checks it -- in the arm that has two aggregates in hand.
    // SplineIk: the rest description is a pure function of compiled rig constant
    // rests, so exec's per-evaluation rebuild bakes out.
    RigExecBakedInput<double> preserveVolume, midFollowWeight, roll, twist,
        minLengthRatio;
    bool splineParamsVary = false;
    /// Skeleton2d: per bone in rigExec:joints order, ten doubles and three
    /// bools (RigExecBone2dChannelStorage); then each space switch's
    /// index. Outside parent spaces are `controls`.
    std::vector<RigExecBakedInput<double>> bone2dDoubles;
    std::vector<RigExecBakedInput<bool>> bone2dBools;
    std::vector<RigExecBakedInput<double>> bone2dSelectors;
    /// FkChain and Skeleton2d: each joint group's rigExec:activeMember.
    std::vector<RigExecBakedInput<double>> jointGroupSelectors;
    // TwistDistribution. rigExec:start and rigExec:end ride on the shared
    // `root` and `end` members rather than a pair of their own: they are
    // provider slots read out of `fin` exactly as an IK control is, and
    // the version binding and the Solve step's read declaration are each
    // written once over those members. The two landmark sets are the
    // START and END rests, which exec substitutes with identity landmarks
    // for an unwired end -- a case the bake answers with `degenerate`
    // instead, because an unwired end also leaves the REQUIRED frame
    // input unbound and the computation publishes nothing at all.
    // Exact immutable authored array inputs, retained before remapping
    // or default expansion for the optional independent exec witness.
    VtIntArray execJointElements;
    VtFloatArray execTwistWeights;
    VtFloatArray execSplineWeights;
    int execTwistCount = 1;
    RigExecBakedInput<double> twistTurns;
    // Ribbon. The driver curve's points are the one solver input that
    // is scene data rather than rig state: the dynamic path reads the
    // attribute straight off the stage and hands exec both values as
    // packet overrides, honouring neither connections nor the resolved
    // inputs. So they are read the same way -- but in the PROLOGUE, into
    // the SolverPoints slot the Solve step declares, because a step body
    // may not touch USD.
    SdfPath ribbonPointsPath;
    UsdAttributeQuery ribbonPointsQuery;
    RigExecReadPhase ribbonPointsPhase;
    RigExecBakedPointsBinding ribbonPointsBinding;
    /// The bind-time value, read once at Default. An attribute carrying
    /// only time samples answers nothing there, which is how the dynamic
    /// path ends up with an empty rest and an empty aggregate.
    /// This run's live value and the last run's, compared by value so a
    /// moved driver curve dirties this solver's cluster and nothing
    /// else. Both empty for a curve that cannot vary with time, whose
    /// value is folded into ribbonConstantPoints instead.
    std::vector<GfVec3f> ribbonPoints, lastRibbonPoints;
    std::vector<GfVec3f> ribbonConstantPoints;
    bool ribbonPointsVarying = false;
    bool ribbonPointsDirty = false;
    RigExecBakedInput<int> ribbonSampleCount;
    // (providerSlot, element) pairs this solver writes
    std::vector<std::pair<int, int>> outputs;
    /// Parallel to `outputs`: whether a read phase names this solver's
    /// checkpoint of that joint (the evaluator's _nativePhaseCheckpoints
    /// membership), which is what gives the output a frame record.
    std::vector<char> outputSnapshots;

    // The candidates this solver published, in `outputs` order and
    // nowhere else: the merge into the batch's table is the commit's
    // job, so two solvers of one batch never write the same storage --
    // an invariant Build now refuses to violate rather than assumes (the
    // duplicate-slot check in bakedPose.cpp's commit sizing).
    std::vector<RigExecPointFrame> outFrames;
    std::vector<char> outPresent;
    /// Where each output lands in its commit's slot-ordered table.
    std::vector<int> outPosition;
    /// Slots of the joints this solver published no element for. Their
    /// key spells each as `pathTexts` holds it.
    std::vector<int> fallbackSlots;
    /// The `fin` versions this solve reads: one per control, then the
    /// four named controls. Decided at Build like every other read;
    /// -1 controls are bound to their own slot's seed and never
    /// read.
    std::vector<uint32_t> controlReads;
    uint32_t rootRead = 0, midRead = 0, endRead = 0, poleRead = 0;
};

struct RigExecBakedConstraint {
    RigExecConstraintRecord kernelRecord;
    RigExecConstraintInputs kernelInputs;
    RigExecConstraintResult kernelResult;
    SdfPath path;
    /// `path`, `sourcePaths` and `deltaBasePath` spelled at Build, for
    /// the body's diagnostics.
    std::string pathText;
    std::vector<std::string> sourcePathTexts;
    std::string deltaBasePathText;
    TfToken type;
    /// rigExec:weightObject, empty when the constraint binds none.
    ///
    /// Path, not index: resolved by the oracle from the stage.
    SdfPath weightObject;
    int weightField = -1;
    /// The one float the oracle resolves into, and the string it would
    /// report. Sized at Build so the step body allocates neither.
    std::vector<float> weightScratch;
    std::string weightError;
    int target = -1;
    /// Every target this constraint names, in compiled order, and which
    /// of them a read phase wants recorded after it. `target` is the
    /// first of them, which is the only one an ordinary constraint
    /// revises.
    std::vector<int> targetSlots;
    std::vector<char> snapshotTargets;
    /// Per source, in compiled order: the slot the walk holds a frame
    /// for, or -1; the entry in `nativeSources` read off the stage when
    /// it does not; and the path either way, which is what a diagnostic
    /// about the source names.
    std::vector<int> sources;
    std::vector<int> sourceNatives;
    std::vector<SdfPath> sourcePaths;
    /// This constraint's entry in `constraintArrays`, or -1 when it
    /// reads no per-frame array at all.
    int arrays = -1;
    RigExecBakedInput<bool> enabled;
    RigExecBakedInput<float> defaultWeight;
    // position/rotation/scale
    RigExecBakedInput<GfVec3d> offset;
    // The operator's own affect group.
    RigExecBakedInput<bool> affectX, affectY, affectZ;
    RigExecBakedInput<bool> tX, tY, tZ, rX, rY, rZ, sX, sY, sZ; // parent
    RigExecEulerOrder order = RigExecEulerOrder::XYZ;
    // aim
    /// aimVector holds the aimAxis fallback as a constant when unauthored.
    RigExecBakedInput<GfVec3d> aimVector, upVector, rotationOffset,
        worldUpVector;
    bool preserveInputUp = false;
    TfToken worldUpType;
    GfVec3d sceneUp{0, 1, 0};
    /// The GEOMETRY domain: non-empty when rigExec:moves named
    /// <prim>.points, in which case this constraint revises no
    /// transform at all -- it measures a delta against the target's own
    /// authored transform and hands it to the Matrix revision the same
    /// mover contributes. `deltaBase` indexes the dense delta tables.
    SdfPath pointsTarget;
    SdfPath deltaBasePath;
    int deltaBase = -1;
    int worldUpObject = -1;
    int worldUpNative = -1;
    SdfPath worldUpPath;
    bool worldUpObjectNamed = false;
    /// rigExec:space on a rotation constraint: the provider whose own
    /// rest->pose map carries the whole rig, or -1 when none is named.
    /// The carry is the space switch's, from the same two tables
    /// (posedM and defaultRoundTrip); see the dynamic path, which
    /// derives it from the same pair of taps.
    int spaceSlot = -1;
    /// rigExec:blendShear (Scale, Parent) and
    /// rigExec:worldUpRotationOnly (Aim), compiled; both off is the
    /// original arithmetic.
    bool blendShear = false;
    bool worldUpRotationOnly = false;
    /// rigExec:weightBlend == "radial" on a transform-domain
    /// RigExecMatrixMover, compiled.
    bool radialBlend = false;

    // The one multi-target built-in: it revises its whole inferred joint
    // chain atomically, so `targetSlots` IS the chain and the commit
    // declares every one of them.
    bool singleChainIk = false;
    RigExecSingleChainIkMode ikMode = RigExecSingleChainIkMode::RotatePlane;
    /// rigExec:poleVectorMode == "object", and rigExec:evaluationMode
    /// resolved against _IkUsesAnimatedTs. Both are uniform tokens over
    /// structural state, so both are settled at Build.
    bool poleModeObject = false;
    bool preserveJointOrientation = false;
    bool useAnimatedTs = false;
    /// Parallel to targetSlots: the compiled "a step below wrote this
    /// joint" flags, so the rest reference is that step's frame.
    std::vector<char> ikRestLive;
    /// The effector and the pole objects, as source references: a slot
    /// when the walk holds a frame, an entry in `nativeSources` when the
    /// stage does.
    int effector = -1, effectorNative = -1;
    SdfPath effectorPath;
    std::vector<int> poleObjects, poleObjectNatives;
    /// Read only in RotatePlane mode, which is where the dynamic walk
    /// reads them.
    RigExecBakedInput<GfVec3d> poleVector;
    RigExecBakedInput<double> twistDegrees;
    RigExecBakedInput<float> stretch;

    // RigExecIkFkMatchConstraint (ikFkMatchKernel.h). It has no targets:
    // `sources` are the three joints, the IK root, the effector, then
    // the IK control, the pole and the three FK controls. Its revision
    // of those controls is presentation, applied to the published
    // control frames after the walk, so no reader in the rig sees it.
    bool ikFkMatch = false;
    /// One scalar input bound in its source's own precision: a float
    /// read as a double, or the reverse, answers its fallback.
    struct MatchScalar {
        RigExecBakedInput<float> narrow;
        RigExecBakedInput<double> wide;
        bool isFloat = false;
    };
    /// inputs:switch, stretch, softDistance, upperScale, lowerScale, pin,
    /// twist, in that order.
    std::array<MatchScalar, 7> matchScalars;
    double matchIkValue = 1.0;
    double matchScaleCalibration = 0.0;
    double matchUpperOffset = 0.0;
    double matchLowerOffset = 0.0;
    bool matchStretchLimb = true;
    /// The pole's space switch (index into spaceSwitches) or -1, and per
    /// source of it whether the source rides the IK control; with no
    /// switch, whether its namespace parent does. Settled after the
    /// switches are built.
    int matchPoleSwitch = -1;
    std::vector<char> matchPoleSourceRides;
    bool matchPoleParentRides = false;
    /// The pole switch's inputs as its compose left them, re-run with
    /// the IK control moved; scratch, so the body does not allocate.
    RigExecSpaceSwitchInputs matchPoleInputs;
    /// This run's outputs:ikValues and outputs:fkValues, published after
    /// the walk when `matchValuesValid`.
    std::array<double, 11> matchIkValues{};
    std::array<double, 20> matchFkValues{};
    bool matchValuesValid = false;
    /// This run's presented frames of the five controls, in source order
    /// from index 5, and which of them are shown (bit k for control k).
    std::array<RigExecPointFrame, 5> matchShown;
    unsigned matchShownMask = 0;
    /// Every other control under a presented one, and which of the five
    /// it rides (the nearest above it): it is presented carried by that
    /// control's revision. Namespace topology, settled at Build.
    std::vector<std::pair<int, int>> matchRiders;

    // RigExecHarmonicsConstraint (harmonicsKernel.h): one target, the
    // optional measuring space in `spaceSlot`, and a history window.
    // inputs:axisAmplitude is bound into `offset`.
    bool harmonics = false;
    RigExecHarmonicsSettings harmonicsSettings;
    bool harmonicsMatchVelocity = true;
    /// inputs:amplitude, decay and termination, in that order.
    std::array<MatchScalar, 3> harmonicsScalars;
    /// This constraint's entry in `windowedOps`, or -1.
    int window = -1;
    /// Scratch for the solve's positions, so the body does not
    /// allocate per run once it has run.
    std::vector<double> harmonicsPositions;
};

/// A windowed operation (rigExecGraph/windowHistory.h): a step whose
/// solve reads the inputs it captured at the earlier ticks of a fixed
/// window.
struct RigExecBakedWindowedOp {
    /// The step (original index) whose body captures and solves, and
    /// its canonical op; both settled once the graph is compiled.
    int step = -1;
    uint32_t op = UINT32_MAX;
    /// Earlier ticks its solve reads.
    uint32_t frames = 0;
    /// The windowed ops whose results its inputs depend on, ascending.
    std::vector<uint32_t> inner;
    /// Some windowed op's inputs depend on this one's result, so it
    /// solves at history ticks too.
    bool feedsWindow = false;
    SdfPath path;
};

/// One compiled RigExecSpaceSwitch, in the program's own terms.
///
/// The compose for an ordinary slot is
///
///     base = avars * selfD * parentDinv * posedM[parent]
///
/// and a switched slot's is the same shape with the namespace parent's
/// pair of spaces replaced by the SELECTED source's pair:
///
///     base = avars * selfD * inverse(selfD[source]) * posedM[source]
///
/// so the two halves cancel at rest and no space can move the rig
/// standing still. A source slot of -1 is world: it contributes
/// identity, which pins the control at its zero pose.
///
/// Every provider frame a switch reads -- its namespace parent, each
/// source, its space -- is read at a VERSION bound at Build (see
/// FrameVersion), never as whatever the slot holds when the step runs.
struct RigExecBakedSpaceSwitch {
    /// Which version of one provider frame a switch reads: the version
    /// standing when the dynamic walk resolves this switch, with every
    /// switch it resolves earlier applied and every other one not.
    ///
    /// `anchor`'s last version (identity at -1), composed UNSWITCHED
    /// through `recompose`, top down. Empty `recompose` is the anchor's
    /// last version as is, which is every read on a rig whose switches
    /// do not nest; a non-empty one re-derives the pre-switch frame of a
    /// control whose own switch resolves at or after this one.
    struct FrameVersion {
        int anchor = -1;
        std::vector<int> recompose;
        int context = -1;
    };
    int slot = -1;
    std::vector<int> sourceSlots;
    /// Parallel to sourceSlots; unused where the slot is -1 (world).
    std::vector<FrameVersion> sourceReads;
    /// The namespace parent's frame `local` is divided back out of.
    FrameVersion parentRead;
    /// spaceSlot's frame, for the carry.
    FrameVersion spaceRead;
    /// Parallel to sourceSlots: which part of that source's rotation
    /// reaches the target (a pole vector in its hand's space takes the
    /// twist and not the swing).
    std::vector<RigExecRotationFilter> filters;
    GfVec3d twistAxis = GfVec3d(1, 0, 0);
    /// rigExec:space -- the provider whose own rest->pose map carries
    /// the whole rig. -1 when none is named, and then the filter runs
    /// exactly as it did before the masters existed. See the dynamic
    /// path, which derives the same carry from a pair of taps.
    int spaceSlot = -1;
    RigExecBakedInput<double> activeInput;
    RigExecBakedInput<TfToken> activeTokenInput;
    bool tokenIndex = false;
    std::vector<TfToken> labels;
    bool affectTranslation[3] = {true, true, true};
    bool affectRotation[3] = {true, true, true};
    bool affectScale[3] = {true, true, true};
    RigExecSpaceSwitchRecord kernelRecord;
    RigExecSpaceSwitchInputs kernelInputs;
    /// This run's space match (spaceMatchKernel.h): per source index,
    /// the control's tx ty tz rx ry rz that keep it where it is with
    /// that space selected; published as `<control>.spaces:matchValues`.
    /// Written only by this switch's compose.
    std::vector<double> matchValues;
    bool matchValid = false;
};

// One entry per revision of one chain, in chain order. The packet is
// assembled by the SAME RigExecAssembleParameters the dynamic path calls
// and the kernel is the same shared kernel, so only the plumbing around
// them is baked.
/// One contiguous vertex range of one revision, and the influences the
/// vertices in it reference.
///
/// A chunk is SPECULATIVE: it starts as soon as its own influences are
/// final, without waiting for the revision's validity fold, and writes
/// its range of the revision's own output buffer. The fuse is what
/// decides afterwards whether the revision applied at all; a chunk whose
/// work is not wanted is discarded rather than undone.
struct RigExecBakedGeomChunk {
    /// The range, half-open, in the vertex order the mesh is authored
    /// in. Vertex order is never permuted: chunk boundaries are the only
    /// thing the partition chooses.
    int begin = 0, end = 0;
    /// Influence positions, or sparse blend channel indices, ascending.
    /// An upper bound is safe and a missing entry is not, so this is the
    /// UNION over the range, computed from the same indices the kernel
    /// reads.
    std::vector<int> key;
    std::vector<GfVec3f> blendDeltas;
    /// The revision's influence table as this chunk sees it: identity
    /// everywhere, with the key's entries copied from the matrix slots
    /// each run. Linear blend skinning reads an entry only through an
    /// index of one of its own vertices, so the identities are never
    /// read -- they are there so the table has the shape the layout
    /// describes rather than to contribute a value.
    std::vector<GfMatrix4d> transforms;
    /// The same table narrowed for the SIMD path, and split for the
    /// dual-quaternion one. Filled entry by entry beside `transforms`,
    /// so the chunk pays for |key| matrices rather than for all of them.
    std::vector<float> rows;
    std::vector<RigExecScaledDualQuat> palette;
    /// Whether the chunk's key matrices moved since it last ran, decided
    /// while they are copied in. A chunk whose own influences stand
    /// still, over points that stand still, already holds its answer.
    bool keyChanged = false;
    /// The range of the output buffer holds this run's value for it.
    /// Sticky across a run the chunk sat out, which is what makes the
    /// skip above sound. In a range chain, chunk g is vertex group g
    /// (RigExecBakedGeomRevision::groups holds its published points).
    bool ok = false;
};

/// One blend channel's stage handles, resolved once at Build.
///
/// The channels themselves are per-frame reads -- a channel weight is
/// what an animator drags -- so nothing here is a VALUE. What is captured
/// is which attribute to ask, in the order the accumulation is defined
/// over: `binding.blendInputs` is canonically sorted at compile and the
/// samples of one channel are stable-sorted by activation every frame.
/// Float addition is not associative, so an order that differs from the
/// dynamic walk's is a different last bit.
struct RigExecBakedGeomBlendChannel {
    /// `inputs:weight` on the RigExecBlendInput prim. Invalid when the
    /// prim or the attribute is absent, which reads as the channel's
    /// default exactly as the dynamic walk's invalid handle does.
    UsdAttribute weight;
    SdfPath weightPath;
    /// The PoseWeight slot `inputs:weight` resolves to through its
    /// authored connection, or -1 when it resolves to the stage. The
    /// dynamic walk finds the same value by following the connection
    /// into the generation's resolved inputs, which the interpolator
    /// phase filled; here the phase is a step and the value is a slot,
    /// which is what puts the edge in the graph.
    int poseWeight = -1;
    struct Sample {
        /// The RigExecBlendSample prim, which keys the shape cache.
        SdfPath samplePath;
        /// `rigExec:activation` on the RigExecBlendSample prim.
        UsdAttribute activation;
        /// The activation attribute's path, beside the handle: the frozen
        /// worker keys the sampled activation by this, because it may
        /// not touch the cloned handle -- not even GetPath.
        SdfPath activationPath;
        /// The sample's target-shape points, and the path a declared
        /// read phase looks that array up under. Dense form only.
        UsdAttribute points;
        SdfPath pointsPath;
        RigExecReadPhase phase;
        /// A dense sample's non-base phase, bound at Build; `id` is -1
        /// for a base or sparse sample, which never reads a record.
        RigExecBakedPointsBinding pointBinding;
        /// Sparse shape identity and the last body-normalized immutable layout.
        SdfPath blendShape;
        std::shared_ptr<const RigExecBlendSampleLayout> layout;
        bool shapeValid = false;
        /// What `layout` was built from while `layoutKeyed`: the content
        /// versions of the offsets and indices leaves and the point
        /// count. The assembly rebuilds the layout only when one of them
        /// moved; `layoutBuilds` counts the builds. Owned by the body
        /// that assembles the revision.
        bool layoutKeyed = false;
        uint64_t layoutOffsetsVersion = 0;
        uint64_t layoutIndicesVersion = 0;
        size_t layoutPointCount = 0;
        uint64_t layoutBuilds = 0;
        /// Structural cache refusal retained for the runtime stream policy.
        bool layoutRefused = false;
        /// The dense points the last assembly consumed; the bake stores
        /// them as the sample's static points. Dense form only; a sparse
        /// sample's shape rides its layout instead.
        std::vector<GfVec3f> lastPoints;
        /// The revision's declared activation, dense points, and sparse
        /// Raw Default offsets/indices leaf IDs, or -1.
        int activationLeaf = -1;
        int pointsLeaf = -1;
        int offsetsLeaf = -1;
        int indicesLeaf = -1;
    };
    std::vector<Sample> samples;
    /// The revision's path leaves holding `inputs:weight` and whether the
    /// generation's resolved inputs hold its path, or -1.
    int weightLeaf = -1;
    int weightHeldLeaf = -1;
};

struct RigExecCompiledChainFacts {
    SdfPath target;
};

struct RigExecCompiledRevisionFacts {
    SdfPath moverPath;
    SdfPath target;
    RigExecRevisionOp op = RigExecRevisionOp::Skin;
};

struct RigExecCompiledWeightFacts {
    SdfPath path;
    TfToken type;
};

/// The compiled rig, sealed: counts, the records still lent by index in the
/// order the program runs them, the operation graph, the declared reads and
/// writes, the stage-frame admission, the bound paths, and the inputs the
/// program overrides. Chains, revisions and weight objects are evaluator
/// state: the view answers them with facts, never with a record. A pose
/// interpolator's record keeps the solve's output buffer, so the view lends no
/// pose-interpolator accessor: a retained view cannot read a buffer the
/// evaluator is rewriting. Nothing here mutates, and nothing here reaches the
/// builder.
///
/// A record or fact reference borrowed from a view lives as long as the view
/// does: hold the program, or the shared version from
/// RigExecBakedProgram::ShareCompiledProgram. A structural edit builds a new
/// program with a new view; a value patch keeps this one, since it moves no
/// record.
class RigExecCompiledProgram final {
public:
    RigExecCompiledProgram(const RigExecCompiledProgram &) = delete;
    RigExecCompiledProgram &operator=(const RigExecCompiledProgram &) = delete;

    /// How many of each compiled record the program holds, and the record at
    /// \p index in canonical order. An out-of-range index answers
    /// the empty record for its kind only where stated; the count is the
    /// guard.
    size_t GetStepCount() const noexcept;
    const RigExecBakedStep &GetStep(size_t index) const noexcept;
    size_t GetExcludedStepCount() const noexcept;
    const RigExecBakedStep &GetExcludedStep(size_t index) const noexcept;
    size_t GetCommitCount() const noexcept;
    const RigExecBakedCommit &GetCommit(size_t index) const noexcept;
    size_t GetComposeGroupCount() const noexcept;
    const RigExecBakedComposeGroup &
    GetComposeGroup(size_t index) const noexcept;
    size_t GetProviderSlotCount() const noexcept;
    size_t GetLadderCount() const noexcept;
    const RigExecBakedLadder &GetLadder(size_t slot) const noexcept;
    size_t GetSpaceSwitchCount() const noexcept;
    const RigExecBakedSpaceSwitch &GetSpaceSwitch(size_t index) const noexcept;
    size_t GetChainCount() const noexcept;
    size_t GetChainRevisionCount(size_t chainIndex) const noexcept;
    size_t GetRevisionCount() const noexcept;
    std::pair<int, int> GetRevisionIdentity(size_t index) const noexcept;
    size_t GetDerivedTargetCount() const noexcept;
    std::pair<int, int> GetDerivedIdentity(size_t index) const noexcept;
    size_t GetWeightObjectCount() const noexcept;

    /// The chain index guards the revision index.
    const RigExecCompiledChainFacts &GetChainFacts(size_t index) const noexcept;
    const RigExecCompiledRevisionFacts &
    GetChainRevisionFacts(size_t chainIndex,
                          size_t revisionIndex) const noexcept;
    const RigExecCompiledWeightFacts &
    GetWeightObjectFacts(size_t index) const noexcept;

    size_t GetConstraintCount() const noexcept;
    const RigExecBakedConstraint &GetConstraint(size_t index) const noexcept;
    size_t GetSolverArrayCount() const noexcept;
    const std::vector<std::pair<SdfPath, int>> &
    GetSolverArrays() const noexcept;
    size_t GetConstraintArrayCount() const noexcept;
    const RigExecBakedConstraintArray &
    GetConstraintArray(size_t index) const noexcept;
    size_t GetPropertyChainCount() const noexcept;
    const RigExecBakedPropertyChain &
    GetPropertyChain(size_t index) const noexcept;
    size_t GetPropertyRecordCount() const noexcept;
    const RigExecBakedPropertyRecord &
    GetPropertyRecord(size_t index) const noexcept;
    size_t GetFrameRecordCount() const noexcept;
    const RigExecBakedFrameRecord &GetFrameRecord(size_t index) const noexcept;
    size_t GetClusterCount() const noexcept;
    const RigExecBakedCluster &GetCluster(size_t index) const noexcept;
    size_t GetPointBindingCount() const noexcept;
    size_t GetNativeSourceCount() const noexcept;
    const RigExecBakedNativeXformSource &
    GetNativeSource(size_t index) const noexcept;
    size_t GetXformSlotCount() const noexcept;
    int GetXformSlot(size_t index) const noexcept;
    const UsdPrim &GetXformPrim(size_t index) const noexcept;
    size_t GetReaderWalkCount() const noexcept;
    const RigExecBakedReaderWalk &GetReaderWalk(size_t index) const noexcept;

    /// The operation graph the program runs, as plain records (bakedTrace.h),
    /// and the windowed operations in program order.
    std::vector<RigExecOpGraphNode> GetOpGraph() const;
    const std::vector<RigExecBakedWindowedOp> &GetWindowedOps() const noexcept;

    /// What one step declares it reads and writes, and the cross-domain
    /// reads the compile selected.
    const std::vector<RigExecBakedSlotRange> &
    GetStepReads(size_t stepIndex) const noexcept;
    const std::vector<RigExecBakedSlotRange> &
    GetStepWrites(size_t stepIndex) const noexcept;
    const RigExecBakedCones &GetCones() const noexcept;
    const std::vector<RigExecCrossDomainRead> &
    GetCrossDomainReads() const noexcept;

    /// The stage-frame prologue's compiled admission, and the paths whose
    /// transforms it reads.
    const RigExecRequiredStageFramesAdmission &
    GetRequiredStageFramesAdmission() const noexcept;
    const std::vector<SdfPath> &GetDeltaBasePaths() const noexcept;

    /// The two fingerprints a compiled version answers for. The program
    /// digest folds this version's input counts onto the evaluator's
    /// structural signature (RigExecRigEvaluator::GetCompiledRigSignature),
    /// which is run state the view never reads; the constant digest folds
    /// the avar constants this version holds (frozenContext.h). A value
    /// edit patches a constant in place, so the constant digest can move
    /// without opening a new version.
    uint64_t GetProgramDigest(uint64_t compiledRigSignature) const noexcept;
    uint64_t GetConstantDigest() const noexcept;

    /// The tables those digests fold: the avar constants, the bindings that
    /// hold them, the constants an edit has since animated, and the width
    /// of the avar table the prologue fills.
    const std::vector<double> &GetAvarConstants() const noexcept;
    const std::vector<RigExecBakedAvarBinding> &
    GetAvarBindings() const noexcept;
    const std::vector<RigExecBakedAvarBinding> &
    GetAvarConstantBindings() const noexcept;
    const std::vector<size_t> &GetPromotedAvars() const noexcept;
    size_t GetAvarConstantCount() const noexcept;
    size_t GetAvarConstantBindingCount() const noexcept;
    size_t GetAvarBindingCount() const noexcept;
    size_t GetAvarSlotCount() const noexcept;

    /// The provider slots in namespace DFS order, their Build-spelled text,
    /// the asset root they are measured against, and the published paths.
    const std::vector<SdfPath> &GetBoundPaths() const noexcept;
    const std::vector<std::string> &GetPathTexts() const noexcept;
    const SdfPath &GetAssetRootPath() const noexcept;
    const std::vector<SdfPath> &GetJointPaths() const noexcept;
    const std::vector<SdfPath> &GetControlPaths() const noexcept;
    const std::vector<SdfPath> &GetPoseWeightPaths() const noexcept;
    const std::set<SdfPath> &GetChainTargets() const noexcept;

    /// The inputs the program registered, and which of them an interactive
    /// override can reach; the head override slots; the oracle's
    /// publications; the weight objects' falloff remaps; and the joint to
    /// solver binding the epilogue reads.
    size_t GetBoundInputCount() const noexcept;
    size_t GetVaryingInputCount() const noexcept;
    const std::vector<char> &GetOverridden() const noexcept;
    const std::map<SdfPath, std::vector<int>> &
    GetOverridableInputs() const noexcept;
    const std::map<SdfPath, uint32_t> &GetHeadOverrideSlots() const noexcept;
    const std::map<SdfPath, std::vector<float>> &
    GetFalloffLuts() const noexcept;
    const std::optional<RigExecOraclePublicationContext> &
    GetOraclePublications() const noexcept;
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> *
    GetJointSolverBinding() const noexcept;

    /// Whether \p other is the same compiled version as this one: the same
    /// records, not a copy of them.
    bool IsSameVersionAs(const RigExecCompiledProgram &other) const noexcept;

private:
    friend class RigExecBakedProgram;

    /// Factory-only construction: this passkey is private and not
    /// default-constructible, so {} cannot build one outside this class.
    struct _VersionTag {
        explicit _VersionTag(int)
        {
        }
    };

public:
    /// Binds the view to one compiled version. Only RigExecBakedProgram can
    /// name the tag; the view is what every consumer holds.
    RigExecCompiledProgram(
        _VersionTag, std::shared_ptr<const RigExecBakedProgramImpl> compiled);

private:
    const RigExecBakedProgramImpl &Storage() const noexcept
    {
        return *_compiled;
    }

    std::shared_ptr<const RigExecBakedProgramImpl> _compiled;

    /// The facts projected from the compiled storage once at construction,
    /// in program order, one revision list per chain. The facts queries
    /// borrow from here; nothing re-projects per getter or per frame.
    std::vector<RigExecCompiledChainFacts> _chainFacts;
    std::vector<std::vector<RigExecCompiledRevisionFacts>> _chainRevisionFacts;
    std::vector<RigExecCompiledWeightFacts> _weightFacts;
};

} // namespace rigExec

#endif // RIGEXEC_COMPILED_PROGRAM_H
