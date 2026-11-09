// RigExec Hydra publication bridge.
// Owns the evaluator-side of the imaging chain: evaluation completes
// first, then the complete immutable generation is atomically published
// to the snapshot store, and only then are precise dirtied notices sent.
// Hydra pulls never compute or wait.
#ifndef RIGEXEC_IMAGING_BRIDGE_H
#define RIGEXEC_IMAGING_BRIDGE_H

#include "bridgeFreshness.h"
#include "bridgeWarmPublication.h"
#include "sceneIndices.h"
#include "snapshotStore.h"

#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"
#include "rigExec/outputAffectedIndex.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/usd/usd/notice.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

class RigExecBridgeFrameCache;
class RigExecBridgeGuides;
class RigExecBridgeSampling;
class RigExecBridgeSnapshotBuilder;
struct RigExecCacheServe;

/// Drives one rig's evaluation into the imaging chain.
class RigExecImagingBridge {
public:
    RigExecImagingBridge(const UsdStageRefPtr &stage, const SdfPath &rigPath);

    /// Constructs over an externally owned store (the application may
    /// create the store before any scene index or bridge exists).
    RigExecImagingBridge(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                         std::shared_ptr<RigExecSnapshotStore> store);

    /// Out-of-line: the domain components are held by unique_ptr and
    /// destroyed where their types are complete.
    ~RigExecImagingBridge();

    /// The outcome of one evaluate-and-publish step, for callers that
    /// broadcast notices to multiple filter chains.
    struct PublishResult {
        bool ok = false;
        RigExecPublishedDirtyVector dirtied;
        /// Non-null when the compiled rig changed this generation.
        RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr
            compiledRig;
        /// True when the published generation came from the frame cache
        /// without running the evaluator: no evaluator pull happened. False
        /// on the live path (including a shadow-verified hit, which DID
        /// evaluate) and on every path that bypasses the cache.
        bool cacheHit = false;
    };

    /// The renderer's HdSceneIndexCreateArgsSchema.motionBlurSupport
    /// capability bit: false permits one sample, true
    /// allows the explicit render-preflight sample set, and when absent
    /// the application's explicit render profile is authoritative.
    enum class MotionBlurSupport { Absent, False, True };

    /// Validates a render-motion profile against the renderer capability
    /// bit before any evaluation: a multi-sample
    /// profile fails preflight when the renderer advertises
    /// motionBlurSupport = false, and an empty offset set never renders.
    static bool PreflightMotionProfile(const std::vector<float> &shutterOffsets,
                                       MotionBlurSupport support,
                                       std::string *whyNot = nullptr);

    /// Evaluate-then-publish without notifying any scene index; the
    /// caller forwards the result to its chains.
    /// With RIGEXEC_FRAME_CACHE=off this is exactly the historical path:
    /// live evaluation, then publication into the snapshot store. Otherwise
    /// a frame whose inputs digest to a cached entry publishes the stored
    /// pose through the same atomic snapshot without evaluating
    /// (result.cacheHit), and a miss evaluates live as today and memoizes
    /// the result into both the snapshot and the cache. While interactive
    /// overrides stand (an active drag), the cache is bypassed entirely:
    /// live evaluation with neither lookup nor memoization. The multi-sample
    /// render path below never consults the cache.
    PublishResult EvaluateAndPublishResult(UsdTimeCode time);

    /// Warms one frame synchronously without publishing a viewport snapshot.
    /// The caller serializes access to the evaluator and bounds idle work.
    /// Returns true only when a valid pose was stored in the frame cache.
    bool WarmFrameOnCallingThread(UsdTimeCode time);

    /// Frames on demand (frameTable.h): \p time's pose built into
    /// an unpublished snapshot -- no store swap, no Hydra notice, no
    /// generation, no change to the published time, and no GPU deform or
    /// guide state touched. A frame-cache entry serves under the proofs
    /// EvaluateAndPublishResult serves with (the sparse re-key lane aside:
    /// it rewrites entries, so it counts as a miss here). On a miss with
    /// \p evaluate set, one evaluation on the calling thread, memoized like
    /// WarmFrameOnCallingThread unless RIGEXEC_FRAME_CACHE=off. False
    /// while interactive overrides stand, on a miss without \p evaluate, or
    /// when the evaluation fails (\p diagnostic then carries its first
    /// message). \p evaluated says whether the evaluator ran. Owner thread,
    /// serialized by the caller.
    bool
    SnapshotAtTimeWithoutPublishing(UsdTimeCode time, bool evaluate,
                                    RigExecImagingSnapshotConstPtr *snapshot,
                                    std::string *diagnostic, bool *evaluated);

    /// Render-motion publication: evaluates the rig at
    /// baseTime + offset for every explicit frame-relative shutter offset,
    /// retains points, normals, extents and driven transforms per offset,
    /// and publishes them under one complete generation fence. Diagnostic
    /// guides use the offset nearest zero. Preflight (capability check and offset
    /// validation) fails before any evaluation; an incomplete sample set
    /// never publishes. The workflow lives in bridgeMotion.cpp over the
    /// facade's own state; it never consults the frame cache and always
    /// publishes diffs through the store, independent of
    /// SetPublishesDirtied.
    PublishResult EvaluateAndPublishSamples(
        UsdTimeCode baseTime, const std::vector<float> &shutterOffsets,
        MotionBlurSupport support = MotionBlurSupport::Absent);

    /// Compiles the rig; returns false with messages on failure.
    bool Compile(std::vector<std::string> *errors = nullptr);

    /// Append authored rest guides only. Never publishes evaluated control or
    /// joint frames, deformation, shader results or computed weight fields.
    void AppendAuthoredGuides(UsdTimeCode time,
                              RigExecImagingSnapshot *snapshot);

    /// The store the results scene index reads from.
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const
    {
        return _store;
    }

    /// The reserved generated scope this rig owns (for pruning).
    SdfPath GetGeneratedScope() const;

    size_t GetCompiledRigSignature() const
    {
        return _evaluator->GetCompiledRigSignature();
    }

    /// Wires the filters that receive compiled rig swaps and generation notices.
    void SetSceneIndices(const RigExecBindingResolvingSceneIndexRefPtr &binding,
                         const RigExecResultsSceneIndexRefPtr &results)
    {
        _binding = binding;
        _results = results;
    }

    /// Serialized evaluate-then-publish for one explicit time:
    /// compute the complete generation, atomically swap the
    /// snapshot, then send coalesced dirtied notices.
    bool EvaluateAndPublish(UsdTimeCode time);

    /// Selects the weight object whose resolved field is painted onto its
    /// weighted geometry as the influence overlay; an empty path turns
    /// the overlay off.
    ///
    /// ONE at a time on purpose. Two gradients composited onto one mesh
    /// are a picture of neither field, and a rigger placing a volume is
    /// asking about exactly one of them.
    ///
    /// Takes effect on the next publication -- this only records the
    /// selection, because publishing is the caller's serialization point
    /// (the registry republishes at the current time right after).
    void SetWeightOverlay(const SdfPath &weightPrimPath)
    {
        if (weightPrimPath != _weightOverlay) {
            // The overlay flag changes POSE CONTENT (weightFields resolved
            // or not) without changing any sampled input, so entries stored
            // under the other flag describe poses this flag must not serve.
            ClearFrameCache();
            _weightOverlay = weightPrimPath;
        }
        // Resolving the per-point influence field is the overlay's whole
        // cost, and this bridge is its only consumer: pay it only while an
        // overlay is actually selected.
        _evaluator->SetPublishWeightFields(!_weightOverlay.IsEmpty());
    }

    const SdfPath &GetWeightOverlay() const
    {
        return _weightOverlay;
    }

    /// The standing uncommitted manipulation values (see
    /// SetInteractiveOverrides), including display-only values. Numerical
    /// inputs are filtered separately for evaluation and cache keys.
    const std::vector<RigExecValueOverride> &GetInteractiveOverrides() const
    {
        return _interactiveOverrides;
    }

    /// This rig's frame cache: the (compiledRigSignature, controlDigest) store the
    /// UI thread memoizes live results into and background completions warm
    /// behind the generation fence. Shared (not unique) so worker closures
    /// can hold it past the session that sampled them.
    const std::shared_ptr<RigExecFrameCache> &GetFrameCache() const;

    /// Re-reads RIGEXEC_FRAME_CACHE and RIGEXEC_FRAME_CACHE_VERIFY. The
    /// bridge reads both once at construction and keeps them: the frame,
    /// warm-enqueue and cache-hit paths consult the stored values, never
    /// the environment. Owner thread only.
    void RefreshFrameCacheSettings();

    /// The stored RIGEXEC_FRAME_CACHE mode (see RefreshFrameCacheSettings).
    RigExecFrameCacheMode GetFrameCacheMode() const;

    /// Drops every cached frame and every freshness proof. The next
    /// publication evaluates live. Called when the overlay selection moves
    /// (pose content changes without the digest moving) and available to
    /// tests that must isolate one warming sequence from the next. The
    /// registry's fenced entries call this under the scheduler's fence
    /// mutex -- it never takes the fence itself (non-recursive).
    void ClearFrameCache();

    /// Retires serial-keyed poses after any stage edit, including edits
    /// outside the rig's sampled dependency set.
    void RetirePoseOnlyCacheEntries();

    /// The shared warm-frame index memoized completions record into (null
    /// for bare bridges, which warm nothing to query). Set once at
    /// activation; the registry owns the index.
    void SetWarmFrameIndex(std::shared_ptr<RigExecWarmFrameIndex> index);

    /// Records the freshness proof for a warming job built for \p time whose
    /// sampled inputs digest to \p controlDigest. Called on the UI thread at
    /// ENQUEUE time by BuildWarmWork, under the registry lock, for chain-free
    /// jobs only -- never for a vector carrying chain-resolved samples, whose
    /// digest names stale values and must prove nothing.
    ///
    /// Without this, a warmed entry is unreachable: lookups serve only under
    /// a proof, and proofs were recorded only right after a live evaluation
    /// -- which memoizes its own result anyway. Recording the enqueue-time
    /// digest as the proof is what lets a later lookup serve the warmed pose
    /// without evaluating. Safe by the same argument as the live path: the
    /// sampler is deterministic in the stage, so a lookup that samples the
    /// same values proves the same digest, and anything that moved digests
    /// apart and misses. A job that never completes leaves a proof pointing
    /// at no entry, which reads as a miss, never as a wrong pose.
    ///
    /// No-op when the frame cache is off (nothing is served, so nothing is
    /// proven). Applies the same compiled rig scope discipline as the live
    /// memoization: a moved scope clears every proof, this one included.
    /// \p unfoldedDigest is the pre-constant-fold digest and \p inputs the
    /// sampled vector, recorded into the proof's dependency set.
    void NoteWarmingEnqueued(UsdTimeCode time, uint64_t controlDigest,
                             uint64_t unfoldedDigest,
                             const RigExecFrameInputs &inputs);

    /// The compiled rig's output-affected index, synced to the current
    /// program and compiled rig: control IDs to dirty-cluster closures, the
    /// notice adapter's oracle for path-scoped retirement. Null with no
    /// baked program; empty when the program's cones were never built (a
    /// caller that finds it empty retires conservatively, as before).
    /// UI thread only, like the bridge itself: no worker consults it.
    const RigExecOutputAffectedIndex *GetAffectedIndex();

    /// Admits \p overrides' identities to the compiled rig index (the
    /// MapControl production caller): the dynamic-override bootstrapping
    /// path, so a changed override maps to its clusters instead of
    /// collapsing to foreign (all clusters). Seeds mirror
    /// RigExecBakedProgram::SetOverrides placement exactly; an override
    /// that places nowhere is left foreign -- conservative, never wrong.
    /// No-op with no baked program. UI thread only.
    void
    AdmitOverrideControls(const std::vector<RigExecValueOverride> &overrides);

    /// Retires every freshness proof whose dependency set intersects \p
    /// controls: the path-scoped counterpart to the wholesale
    /// clears. Answers how many proofs were dropped. UI thread only.
    size_t
    RetireProofsForControls(const std::vector<RigExecControlId> &controls);

    /// Re-points the proof for \p time at \p newDigest (carry-over): the
    /// entry was re-published under a new key with
    /// identical content, so the surviving proof names the new digest.
    /// Answers false when no proof is recorded for the time. The
    /// dependency set is unchanged: the inputs did not move, only the
    /// constant namespace did. UI thread only.
    bool RepointProof(UsdTimeCode time, uint64_t newDigest);

    /// Answers the recorded proof for \p time, or false when none is held.
    /// For tests pinning proof scoping; production serves through the
    /// lookup path, never through this.
    bool GetFreshProof(UsdTimeCode time, RigExecFreshProof *proof) const;
    size_t GetFreshProofCount() const;

    /// Carries the entry published for \p time under \p oldKey into the
    /// constant namespace \p newConstants: re-publishes
    /// the pose with its retained handle and provenance under the re-keyed
    /// key, evicts the old key, re-points the proof, and re-records the
    /// warm-index completion -- with zero re-evaluation. The caller has
    /// already proven the entry clean (its provenance touches nothing the
    /// edit dirtied); this only proves the MECHANICS: answers false,
    /// having changed nothing observable, when the entry, its retained
    /// handle, or its unfolded digest is missing, when the re-publish
    /// declines, or when the entry already keys under the new namespace
    /// (not a carry -- the caller stands it instead). A false degrades
    /// the frame to lane c (retire + re-warm), never strands it. UI
    /// thread only.
    bool CarryEntry(UsdTimeCode time, const RigExecFrameCacheKey &oldKey,
                    uint64_t newConstants);

    /// Routes a stage notice through the baked capture index: a hit drops
    /// this rig's cached frames (and memoized task lists) for the compiled rig
    /// while its half still names it -- the evaluator rebuilds lazily, and
    /// between the notice and the rebuild the old compiled rig still keys lookups.
    /// A miss drops nothing (digest reachability decides). With no baked
    /// program nothing is dropped: refusal entries key on the
    /// stage-edit serial, which already retires them. Answers whether the
    /// notice hit. Called from the registry's notice path, under its lock;
    /// the cache and memo serialize bare-bridge callers too.
    bool NoteCaptureIndex(const UsdNotice::ObjectsChanged &notice);
    /// Every stage notice that reaches the rig: drops the live sampler's
    /// pinned chain bindings when the notice could move a chain without
    /// moving the compiled rig signature. The registry calls it for EVERY affected
    /// notice, whatever its disposition -- NoteCaptureIndex runs for only
    /// some of them.
    void NoteChainEdits(const UsdNotice::ObjectsChanged &notice);

    /// The rig's chain bindings, pinned for the current compiled rig and
    /// kept current by NoteChainEdits -- the ones the live sampler uses --
    /// rebound first when a notice or a recompile dropped them. Null when
    /// the chains cannot be bound. \p generation, when given, receives a
    /// counter that moves on every rebind, so a holder of a copy (the
    /// warming session) refreshes it only when it actually changed instead
    /// of re-verifying it after every edit. UI thread only.
    const RigExecChainSampleBindings *
    AcquireChainBindings(uint64_t *generation = nullptr) const;

    /// Lifetime counters of this rig's frame cache (zeros when empty).
    RigExecFrameCacheStats GetFrameCacheStats() const;

    /// Uncommitted manipulation values for this rig, with nothing authored
    /// (RigExecRigEvaluator::SetInteractiveOverrides). Records only, like
    /// SetWeightOverlay: publishing is the caller's serialization point.
    void SetInteractiveOverrides(std::vector<RigExecValueOverride> overrides);

    void ClearInteractiveOverrides()
    {
        _interactiveOverrides.clear();
        _evaluationOverrides.clear();
        _evaluator->ClearInteractiveOverrides();
    }

    /// This frame's upstream values (authored-level; RigExecRigEvaluator::
    /// SetUpstreamInputs admits them), sorted by path. Records only, like
    /// SetInteractiveOverrides. Unlike a drag they keep the frame cache on:
    /// the live sampler carries them (RigExecFrameInputs::upstream), so
    /// every sampled key folds them, and the pose-only key folds them too.
    void SetUpstreamInputs(std::vector<RigExecUpstreamValue> values);

    const std::vector<RigExecUpstreamValue> &GetUpstreamInputs() const
    {
        return _upstreamInputs;
    }

    /// The stage the rig evaluates against, for a caller that has to read the
    /// authored value an override is standing in for.
    const UsdStageRefPtr &GetEvaluationStage() const
    {
        return _evaluator->GetEvaluationStage();
    }

private:
    /// Publishes \p pose -- live or cached, already stamped for \p time --
    /// as one complete immutable generation: geometry, guides, the binding
    /// compiled rig when it moved, and the atomic snapshot swap. Shared by the hit
    /// path and the live path so the two cannot publish differently.
    void _PublishPoseSnapshot(UsdTimeCode time, const RigExecRigPose &pose,
                              PublishResult *result);

    /// Publishes a replacement compiled-rig binding into \p result when the
    /// compiled rig signature moved since the last published generation:
    /// the binding names \p snapshot's published prim set, and the
    /// signature is recorded so the next publication re-announces only
    /// after another recompile. Shared by the single-frame and render-
    /// motion paths so both announce before the store swap.
    void _AnnounceCompiledRig(const RigExecImagingSnapshot &snapshot,
                              PublishResult *result);

    /// Publishes the pose a cache lookup served -- a store hit or a
    /// sparse-serve -- as the generation for \p time: the shadow verify
    /// under RIGEXEC_FRAME_CACHE_VERIFY=1 (a mismatch serves the live
    /// pose and asks the cache to repair the entry), and the snapshot
    /// publication through the single _PublishPoseSnapshot path, with
    /// result.cacheHit set only when the evaluator never ran. The cache
    /// owns the alias record and the re-stamp; the facade owns
    /// publication and the cacheHit accounting, so a shadow-verified
    /// serve still reports false.
    void _PublishServedPose(UsdTimeCode time, const RigExecCacheServe &serve,
                            PublishResult *result);

    void _StampGeneration(UsdTimeCode time,
                          RigExecImagingSnapshot *snapshot) const;

    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    /// Weight object currently painted as the influence overlay; empty
    /// means off, which is the default and the ordinary render.
    SdfPath _weightOverlay;

public:
    /// The evaluator's profiler, which the imaging layer also records its
    /// own publish phases into, so one summary covers a whole viewport
    /// update. Recording is off unless RIGEXEC_IMAGING_PROFILE is set.
    const RigExecProfiler &GetProfiler() const
    {
        return _evaluator->GetProfiler();
    }
    /// Whether a publication diffs against the previous generation into
    /// PublishResult::dirtied (the default). A caller that combines the
    /// generations and diffs the combination turns it off.
    void SetPublishesDirtied(bool on)
    {
        _publishesDirtied = on;
    }

    RigExecProfiler *MutableProfiler() const
    {
        return const_cast<RigExecProfiler *>(&_evaluator->GetProfiler());
    }
    /// The evaluator owning the compiled operation graph.
    const RigExecRigEvaluator &GetEvaluator() const
    {
        return *_evaluator;
    }
    void SetOpTimingEnabled(bool enabled)
    {
        _evaluator->SetOpTimingEnabled(enabled);
    }

    /// Forgets every cached guide input. Called for any stage notice that
    /// touches the rig and on every recompile: the caches hold AUTHORED
    /// styling, and only an edit can move that.
    void InvalidateGuideCaches();
    /// The registry's per-notice entry: drops only the guide-cache entries
    /// \p notice can change -- everything on a resync, one prim's styling
    /// on a guide:* edit, a subtree's on a purpose edit (it inherits) --
    /// and leaves the rest standing. A pose edit (avars, rest channels,
    /// mover inputs) changes no guide styling and no hierarchy, so it drops
    /// nothing, so a release does not re-read every provider's styling.
    /// The first call also tells the guide component that notices ARE being
    /// forwarded, which retires its coarse stage-edit-serial clear; a
    /// bridge driven directly keeps that.
    void NoteGuideEdits(const UsdNotice::ObjectsChanged &notice);

private:
    std::unique_ptr<RigExecRigEvaluator> _evaluator;
    /// Guide rendering (bridgeGuides.h): authored-styling caches, the
    /// joint-hierarchy cache, cloth guide bindings and the fills. Owned
    /// here; built after the evaluator and destroyed before it; it holds
    /// no store, frame cache, scheduler, registry or bridge reference and
    /// reads the stage, serials, overrides and selection through the call
    /// passed on each fill.
    std::unique_ptr<RigExecBridgeGuides> _guides;
    /// Snapshot payload construction (bridgeSnapshot.h): the pose ->
    /// published-prims payload, the provider reset-stack walk and its
    /// serial/provider-set memo, and the GPU deform layout tables and
    /// per-prim frame history. Owned here; holds no store, frame cache,
    /// scheduler, results index or bridge reference: the facade passes the
    /// stage, time, evaluator stage-edit serial and compiled publication
    /// metadata on every call, and keeps the generation stamp, the guide
    /// fills, the generation numbering, the compiled-rig announcement and
    /// the store Publish/Replace.
    std::unique_ptr<RigExecBridgeSnapshotBuilder> _snapshot;
    /// Live sampling (bridgeSampling.h): the sampler's chain bindings
    /// pinned for the current compiled rig, their notice-driven invalidation
    /// and the sample-support memo. Owned here; built after the evaluator
    /// and destroyed before it; it holds no store, frame cache, warm index,
    /// registry state or bridge back-pointer: the facade passes the
    /// effective evaluation overrides and upstream values on every
    /// sampling call, and keeps SetUpstreamInputs, the display/evaluation
    /// override storage and the evaluator override translation.
    std::unique_ptr<RigExecBridgeSampling> _sampling;
    /// Frame-cache state and policy (bridgeFrameCache.h): the shared frame
    /// cache, its mode/verify switches, the warm-index handle, the
    /// task-list memo, the freshness-proof map, the affected index, the
    /// cache edit serial, and the serve/lookup/carry/memoize lanes. Owned
    /// here; constructed with the rig path; it holds no store, snapshot,
    /// evaluator-override storage or bridge back-pointer: the facade
    /// passes one borrowed RigExecCacheCall record (evaluator, sampling
    /// component, time, overrides, upstream values) on every cache call,
    /// and keeps the edit-here gate, the interactive-drag and cache-off
    /// bypasses, the RIGEXEC_FRAME_CACHE_VERIFY shadow verification,
    /// publication through _PublishPoseSnapshot, and memoization after a
    /// live evaluation. Background completions stay in the bridge-free
    /// bridgeWarmPublication free functions over the shared cache, so
    /// destroying the components cannot invalidate a running worker.
    std::unique_ptr<RigExecBridgeFrameCache> _cache;
    std::shared_ptr<RigExecSnapshotStore> _store;
    RigExecBindingResolvingSceneIndexRefPtr _binding;
    RigExecResultsSceneIndexRefPtr _results;
    uint64_t _generation = 0;
    size_t _publishedCompiledRigSignature = 0;
    bool _publishesDirtied = true;
    /// Owner-thread manipulation values, including display-only styling.
    std::vector<RigExecValueOverride> _interactiveOverrides;
    // Display inputs never enter the numerical graph or its cache key.
    std::vector<RigExecValueOverride> _evaluationOverrides;
    /// The upstream values as handed to SetUpstreamInputs: the live sampler
    /// carries them and the pose-only key folds them.
    std::vector<RigExecUpstreamValue> _upstreamInputs;
    /// The time and stage edit serial of the last publication: the same
    /// time under a newer serial is the frame an edit was just made at.
    UsdTimeCode _publishedTime = UsdTimeCode::Default();
    uint64_t _publishedSerial = 0;
    bool _publishedTimeValid = false;
};

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_H
