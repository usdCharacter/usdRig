// Frame-cache state and policy owned by the imaging bridge: the shared
// frame cache, its mode/verify switches, the warm-index handle, the
// task-list memo, the freshness-proof map, the compiled rig's affected
// index, the cache edit serial, and the lookup / sparse-serve / carry /
// memoize lanes. It never publishes a viewport generation: a serve fills
// a typed outcome and the facade publishes through its single snapshot
// path. The facade holds this component, passes one borrowed request
// record per call, and keeps the interactive-drag and cache-off bypasses,
// the edit-here gate, and the RIGEXEC_FRAME_CACHE_VERIFY shadow
// verification. Background completions stay in the bridge-free free
// functions (bridgeWarmPublication.h) over the shared cache, so a worker
// never holds the bridge or this component.
#ifndef RIGEXEC_IMAGING_BRIDGE_FRAME_CACHE_H
#define RIGEXEC_IMAGING_BRIDGE_FRAME_CACHE_H

#include "bridgeFreshness.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/outputAffectedIndex.h"
#include "rigExec/taskListCache.h"

#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/timeCode.h"

PXR_NAMESPACE_USING_DIRECTIVE

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

class RigExecBridgeSampling;
class RigExecFrameCache;
class RigExecRigEvaluator;
class RigExecWarmFrameIndex;

/// Everything a cache operation reads outside its own state, borrowed for
/// one synchronous call: the owner-thread evaluator, the sampling
/// component the lookup and memoize lanes sample through, the frame being
/// served, and the facade's standing evaluation overrides and upstream
/// values. Nothing here outlives the call: the record is built fresh on
/// every cache entry point, no reference is retained, and the component
/// holds no bridge pointer and no closure.
struct RigExecCacheCall {
    const RigExecRigEvaluator *evaluator = nullptr;
    RigExecBridgeSampling *sampling = nullptr;
    UsdTimeCode time = UsdTimeCode::Default();
    /// The numerical evaluation overrides (display-only values never
    /// reach the cache), and the standing upstream values.
    const std::vector<RigExecValueOverride> *overrides = nullptr;
    const std::vector<RigExecUpstreamValue> *upstream = nullptr;
};

/// The typed outcome of a publishing cache lookup. The cache fills it and
/// returns; the facade publishes the pose through its single snapshot
/// path. \p served false means the facade evaluates live. The lookup's
/// miss facts (the sampled inputs, the unfolded digest, the retained
/// base) stay inside the cache: the sparse lane runs there.
struct RigExecCacheServe {
    bool served = false;
    /// The cached pose, owned by the outcome (the store entry keeps its
    /// own copy) and already re-stamped for the requested time the way
    /// publication reads it.
    RigExecRigPose pose;
    /// The key the entry was served under.
    RigExecFrameCacheKey key{0, 0};
    /// RIGEXEC_FRAME_CACHE_VERIFY=1: after serving, the facade proves the
    /// pose against a live evaluation. The evaluator pull counts as a
    /// live run whatever the verdict: cacheHit stays false.
    bool liveEvaluationRequested = false;
    /// The facade memoizes the live pose it evaluated when the shadow
    /// verdict mismatches, repairing the entry. Only meaningful with
    /// liveEvaluationRequested.
    bool needsRepairMemoization = false;
};

/// The bridge-owned frame-cache component. UI thread only, like the
/// bridge: the proof map and affected index have no lock because the
/// bridge serializes its callers (the registry holds its mutex on the
/// memoizing and notice paths).
class RigExecBridgeFrameCache {
public:
    explicit RigExecBridgeFrameCache(const SdfPath &rigPath);

    /// The shared frame cache. Shared (not unique) so worker closures can
    /// hold it past the session that sampled them; the facade's
    /// GetFrameCache returns this same reference.
    const std::shared_ptr<RigExecFrameCache> &GetFrameCache() const
    {
        return _frameCache;
    }

    /// Re-reads RIGEXEC_FRAME_CACHE and RIGEXEC_FRAME_CACHE_VERIFY. Read
    /// once at construction and on refresh only; every lane consults the
    /// stored values, never the environment. Owner thread only.
    void RefreshFrameCacheSettings();

    /// The stored RIGEXEC_FRAME_CACHE mode and verify switch.
    RigExecFrameCacheMode mode() const
    {
        return _frameCacheMode;
    }
    bool verifyRequested() const
    {
        return _frameCacheVerify;
    }

    /// The shared warm-frame index memoized completions record into (null
    /// for bare bridges, which warm nothing to query). Set once at
    /// activation; the registry owns the index.
    void SetWarmFrameIndex(std::shared_ptr<RigExecWarmFrameIndex> index);

    /// Drops every cached frame and every freshness proof (cache half:
    /// frames, memo, proof map, scope flags). The sampling component's
    /// sample-support memo reset stays on the facade -- an explicit
    /// cross-domain step. Never takes the scheduler fence (the registry's
    /// fenced entries call it under that mutex; non-recursive).
    void ClearFrameCache();

    /// Retires serial-keyed poses after any stage edit, including edits
    /// outside the rig's sampled dependency set.
    void RetirePoseOnlyCacheEntries();

    /// Retires the freshness proofs when a stage edit moved the serial
    /// and no registry does it. Returns the evaluator's current stage
    /// edit serial, which the facade compares against its published-time
    /// gate.
    uint64_t NoteEditSerial(const RigExecRigEvaluator &evaluator);

    /// Records the freshness proof for a warming job built for the call's
    /// time whose sampled inputs digest to \p controlDigest; see the
    /// facade's NoteWarmingEnqueued for the enqueue-time contract. No-op
    /// when the frame cache is off.
    void NoteWarmingEnqueued(const RigExecCacheCall &call,
                             uint64_t controlDigest, uint64_t unfoldedDigest,
                             const RigExecFrameInputs &inputs);

    /// Retires every freshness proof whose dependency set intersects \p
    /// controls; answers how many proofs were dropped. UI thread only.
    size_t
    RetireProofsForControls(const std::vector<RigExecControlId> &controls);

    /// Re-points the proof for the call's time at \p newDigest (carry-over).
    /// Answers false when no proof is recorded for the time.
    bool RepointProof(const RigExecCacheCall &call, uint64_t newDigest);

    /// Answers the recorded proof for the call's time, or false when none
    /// is held.
    bool GetFreshProof(const RigExecCacheCall &call,
                       RigExecFreshProof *proof) const;
    size_t GetFreshProofCount() const
    {
        return _freshDigests.size();
    }

    /// The compiled rig's output-affected index, synced to the current
    /// program and compiled rig. UI thread only: no worker consults it.
    const RigExecOutputAffectedIndex *
    GetAffectedIndex(const RigExecCacheCall &call);

    /// Admits \p overrides' identities to the compiled rig index; see the
    /// facade. No-op with no baked program. UI thread only.
    void
    AdmitOverrideControls(const RigExecCacheCall &call,
                          const std::vector<RigExecValueOverride> &overrides);

    /// Routes a stage notice through the baked capture index; answers
    /// whether the notice hit. The chain-binding drop runs first, on the
    /// facade, with or without a frame cache.
    bool NoteCaptureIndex(const RigExecCacheCall &call,
                          const UsdNotice::ObjectsChanged &notice);

    /// Carries the entry published for the call's time under \p oldKey
    /// into the constant namespace \p newConstants: publish-new,
    /// evict-old, re-record the warm row, re-point the proof, with zero
    /// re-evaluation; see the facade's CarryEntry. UI thread only.
    bool CarryEntry(const RigExecCacheCall &call,
                    const RigExecFrameCacheKey &oldKey, uint64_t newConstants);

    /// The publishing lookup lane: serves \p serve from a proven entry,
    /// or from this time's retained base through the sparse planner, and
    /// answers false to fall through to live evaluation. A serve records
    /// the provenance alias, re-stamps the pose, and asks for shadow
    /// verification when RIGEXEC_FRAME_CACHE_VERIFY stands; it never runs
    /// the evaluator and never publishes.
    bool TryServe(const RigExecCacheCall &call, RigExecCacheServe *serve);

    /// The query lookup lane (SnapshotAtTimeWithoutPublishing): the pose
    /// and proof lanes with no profiler scope or lane count, and no
    /// sparse re-key lane -- a query counts a rewritten entry as a miss.
    /// True with a valid \p pose on a hit.
    bool QueryLookup(const RigExecCacheCall &call, RigExecRigPose *pose);

    /// Memoizes a just-evaluated pose into the frame cache and records its
    /// digest as its time's freshness proof. Sampled after the evaluation,
    /// so the proof names fresh inputs.
    bool MemoizeLive(const RigExecCacheCall &call, const RigExecRigPose &pose);

    /// Lifetime counters of this rig's frame cache (zeros when empty).
    RigExecFrameCacheStats GetFrameCacheStats() const;

private:
    /// What a lookup sampled on its way to a miss, for the sparse lane.
    /// Internal to this component: the serve outcome never carries it.
    struct _CacheMiss {
        bool sampled = false;
        bool haveBase = false;
        RigExecFrameInputs inputs;
        RigExecFrameCacheKey key;
        uint64_t unfolded = 0;
    };

    /// Records the stage identity and sample time on a pose-only key.
    /// Complete conservative key for operations without a sampled input
    /// contract: time, stage-edit serial, compiled rig, standing
    /// overrides and upstream values.
    bool _ComputePoseOnlyCacheKey(const RigExecCacheCall &call,
                                  RigExecFrameCacheKey *key) const;

    /// Rebuilds the compiled rig index when the program or compiled rig
    /// moved. Answers the synced index, or null with no baked program.
    const RigExecOutputAffectedIndex *
    _SyncAffectedIndex(const RigExecCacheCall &call);

    /// The frame-cache lanes shared by publication and pose queries: the
    /// pose-only proof, the warm-index row, then the sampled proof
    /// compare. True with \p pose and \p key filled on a hit. With \p
    /// publishing false it records no profiler scope or lane count;
    /// either way it takes the frame-cache shard and warm-index locks (in
    /// their existing order) and writes nothing to the stage. A miss
    /// fills \p miss with the sample the sparse lane plans on.
    bool _LookupCachedPose(const RigExecCacheCall &call, bool publishing,
                           RigExecRigPose *pose, RigExecFrameCacheKey *key,
                           _CacheMiss *miss);

    /// Serves the call's time from its same-time retained base when the
    /// sparse planner proves the retained pose is still the answer: the
    /// digest missed (values moved, or the proof was retired) but nothing
    /// the entry's computation reaches did. Re-keys the entry under \p
    /// key (the fresh sample's key) and completes with zero evaluator
    /// pulls; false falls through to live evaluation. The base is ALWAYS
    /// this time's own completion -- a cross-time base could alias one
    /// frame's pose onto another's through stale chain values. \p
    /// unfolded is the fresh sample's pre-fold digest, recorded into the
    /// re-pointed proof. UI thread only.
    bool _TrySparseServe(const RigExecCacheCall &call,
                         const RigExecFrameInputs &inputs,
                         const RigExecFrameCacheKey &key, uint64_t unfolded,
                         RigExecCacheServe *serve);

    /// Completes a servable cached \p pose -- a store hit or a sparse
    /// serve -- into the typed outcome: the alias record, the re-stamp,
    /// and the shadow-verify/repair request when RIGEXEC_FRAME_CACHE_-
    /// VERIFY stands. Shared by the hit lane and the sparse lane so the
    /// two cannot serve differently.
    void _CompleteServe(const RigExecCacheCall &call,
                        const RigExecRigPose &pose,
                        const RigExecFrameCacheKey &key,
                        RigExecCacheServe *serve);

    SdfPath _rigPath;
    /// This rig's frame cache, held shared so background completions can
    /// publish into it after the session that sampled them is gone.
    std::shared_ptr<RigExecFrameCache> _frameCache;
    /// Frame-cache switches, read at construction and on
    /// RefreshFrameCacheSettings only.
    RigExecFrameCacheMode _frameCacheMode = RigExecFrameCacheMode::On;
    bool _frameCacheVerify = false;
    /// The shared warm-frame index, or null for a bare bridge.
    std::shared_ptr<RigExecWarmFrameIndex> _warmIndex;
    /// The memoized affected-set table for sparse planning: the capture
    /// index retires its compiled rig versions here, beside the frames.
    RigExecTaskListCache _taskListMemo;
    /// The compiled rig half of the last memoized key. Atomic for bare
    /// bridges, which serialize on no lock (registry callers hold its
    /// mutex on both the memoizing and the notice paths). A notice that
    /// finds it drifted from the current compiled rig evicts the old half
    /// eagerly -- its frames are unreachable by construction -- instead of
    /// waiting for LRU.
    std::atomic<uint64_t> _lastPublishedCompiledRig{0};
    /// Freshness proofs: (isDefault, timeValue) -> the proof recorded for
    /// a sample taken right after a live evaluation AT that time, when
    /// every chain-resolved input was fresh. A lookup serves a cached pose
    /// only when its pre-evaluation sample digests to the recorded proof,
    /// which is what makes a hit bit-identical to a live evaluation:
    /// chain inputs sampled for a time the evaluator has not run come
    /// from the standing (stale) resolved state, and without the proof a
    /// scrub forth and back over animated chains could alias one frame's
    /// pose onto another's. Scoped by _freshCompiledRig: either moving
    /// clears the map (entries stay in the cache, unreachable, for LRU).
    /// Bounded: past _kFreshDigestCap entries the map clears wholesale,
    /// which only costs misses, never correctness. Path-scoped retirement
    /// drops intersecting proofs via RetireProofsForControls instead of
    /// clearing wholesale.
    std::map<std::pair<bool, double>, RigExecFreshProof> _freshDigests;
    uint64_t _freshCompiledRig = 0;
    bool _freshCompiledRigValid = false;
    uint64_t _cacheEditSerial = 0;
    static constexpr size_t _kFreshDigestCap = 1024;
    /// The compiled rig's output-affected index, built for
    /// _affectedCompiledRig by _SyncAffectedIndex and re-admitted per
    /// compiled rig. UI thread only: no worker consults it, so no lock
    /// guards it.
    std::unique_ptr<RigExecOutputAffectedIndex> _affectedIndex;
    const RigExecBakedProgram *_affectedProgram = nullptr;
    uint64_t _affectedCompiledRig = 0;
    bool _affectedValid = false;
};

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_FRAME_CACHE_H
