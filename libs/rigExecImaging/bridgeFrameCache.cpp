// The bridge-owned frame-cache component's storage TU: the frame cache,
// warm-index rows, the affected index, carry-over, sparse serving, lookup
// lanes and memoization. The freshness proof operations -- warming-enqueue
// proof records, override admission, path-scoped retirement, proof
// re-pointing and capture-index notice routing -- are this class's second
// TU, bridgeCacheFreshness.cpp. This TU publishes no viewport generation:
// a serve fills a RigExecCacheServe outcome and the facade publishes
// through its single snapshot path (bridge.cpp).
#include "bridgeFrameCache.h"

#include "bridgeSampling.h"
#include "warmIndex.h"

#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/tf/diagnostic.h"

#include <optional>
#include <utility>

namespace rigExec {

namespace {

/// The evaluator's profiler, which the imaging lanes record their cache
/// phases into. Const away the same way the facade's MutableProfiler does.
RigExecProfiler *
_Profiler(const RigExecCacheCall &call)
{
    return const_cast<RigExecProfiler *>(&call.evaluator->GetProfiler());
}

} // namespace

RigExecBridgeFrameCache::RigExecBridgeFrameCache(const SdfPath &rigPath)
    : _rigPath(rigPath), _frameCache(std::make_shared<RigExecFrameCache>())
{
}

void
RigExecBridgeFrameCache::RefreshFrameCacheSettings()
{
    _frameCacheMode = RigExecFrameCacheModeFromEnvironment();
    _frameCacheVerify = RigExecFrameCacheVerifyRequested();
}

void
RigExecBridgeFrameCache::SetWarmFrameIndex(
    std::shared_ptr<RigExecWarmFrameIndex> index)
{
    _warmIndex = std::move(index);
}

void
RigExecBridgeFrameCache::ClearFrameCache()
{
    if (_frameCache) {
        _frameCache->Clear();
    }
    _taskListMemo.Clear();
    _lastPublishedCompiledRig.store(0, std::memory_order_relaxed);
    _freshDigests.clear();
    _freshCompiledRigValid = false;
}

void
RigExecBridgeFrameCache::RetirePoseOnlyCacheEntries()
{
    if (_warmIndex && _frameCache) {
        for (const auto &entry : _warmIndex->CompletedKeys(_rigPath)) {
            RigExecEntryProvenance provenance;
            if (_frameCache->LookupProvenance(entry.second, &provenance) &&
                provenance.unfoldedControlDigest == 0 &&
                entry.first.IsNumeric()) {
                _warmIndex->NoteDirtied(_rigPath, entry.first.GetValue());
            }
        }
    }
    for (auto it = _freshDigests.begin(); it != _freshDigests.end();) {
        if (!it->second.sampledInputs) {
            it = _freshDigests.erase(it);
        } else {
            ++it;
        }
    }
}

uint64_t
RigExecBridgeFrameCache::NoteEditSerial(const RigExecRigEvaluator &evaluator)
{
    // A bare bridge has no registry to retire proofs on edits.
    const uint64_t serial = evaluator.GetStageEditSerial();
    if (!_warmIndex && _cacheEditSerial != serial) {
        _freshDigests.clear();
    }
    _cacheEditSerial = serial;
    return serial;
}

const RigExecOutputAffectedIndex *
RigExecBridgeFrameCache::_SyncAffectedIndex(const RigExecCacheCall &call)
{
    const RigExecBakedProgram *program = call.evaluator->GetBakedProgram();
    if (!program) {
        _affectedIndex.reset();
        _affectedValid = false;
        return nullptr;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*call.evaluator);
    // The program object test matters as much as the compiled rig: a rebuild can
    // answer a colliding compiled rig for wholly new clustering, and an index
    // built for the old shape would retire against the wrong cones.
    // (The compiled rig signature folds the build count, so a rebuild moves it in
    // practice; the pointer pins the shape regardless.)
    if (_affectedValid && _affectedIndex &&
        _affectedCompiledRig == compiledRig && _affectedProgram == program) {
        return _affectedIndex.get();
    }
    _affectedIndex.reset(new RigExecOutputAffectedIndex());
    _affectedIndex->Build(impl, compiledRig);
    _affectedCompiledRig = compiledRig;
    _affectedProgram = program;
    _affectedValid = true;
    // A rebuild drops every admission with the old index: the standing
    // overrides re-admit against the new shape, so a drag straddling a
    // rebuild keeps its exact seeds.
    AdmitOverrideControls(call, *call.overrides);
    return _affectedIndex.get();
}

const RigExecOutputAffectedIndex *
RigExecBridgeFrameCache::GetAffectedIndex(const RigExecCacheCall &call)
{
    return _SyncAffectedIndex(call);
}

RigExecFrameCacheStats
RigExecBridgeFrameCache::GetFrameCacheStats() const
{
    if (!_frameCache) {
        return RigExecFrameCacheStats();
    }
    return _frameCache->Stats();
}

bool
RigExecBridgeFrameCache::_ComputePoseOnlyCacheKey(
    const RigExecCacheCall &call, RigExecFrameCacheKey *key) const
{
    if (!key || !_frameCache ||
        !RigExecRefusalControlDigestible(*call.overrides, *call.upstream)) {
        return false;
    }
    // Complete live results remain cacheable when an operation has no
    // frozen input contract. Every stage edit moves this namespace; no
    // per-operation input list or partial retained-state proof is needed.
    key->compiledRigSignature =
        RigExecFrameCacheCompiledRigSignature(*call.evaluator);
    key->controlDigest = RigExecRefusalControlDigest(
        call.time, call.evaluator->GetStageEditSerial(), *call.overrides,
        *call.upstream);
    return true;
}

void
RigExecBridgeFrameCache::_CompleteServe(const RigExecCacheCall &call,
                                        const RigExecRigPose &pose,
                                        const RigExecFrameCacheKey &key,
                                        RigExecCacheServe *serve)
{
    _frameCache->NoteProvenanceAlias(key, call.time);
    serve->pose = pose;
    // Guide styling reads pose.time; the stored pose was evaluated for an
    // earlier visit, so it is re-stamped for the requested frame before
    // anything reads it.
    serve->pose.time = call.time;
    serve->key = key;
    serve->served = true;
    // The shadow verify is the facade's evaluator step: the cache asks
    // for it, and for the repair memoization when the verdict mismatches,
    // and never runs the evaluator itself. A verified serve still counts
    // as a live run for cacheHit -- the facade owns that accounting.
    serve->liveEvaluationRequested = _frameCacheVerify;
    serve->needsRepairMemoization = _frameCacheVerify;
}

bool
RigExecBridgeFrameCache::CarryEntry(const RigExecCacheCall &call,
                                    const RigExecFrameCacheKey &oldKey,
                                    uint64_t newConstants)
{
    if (!_frameCache) {
        return false;
    }
    const UsdTimeCode time = call.time;
    RigExecEntryProvenance provenance;
    if (!_frameCache->LookupProvenance(oldKey, &provenance) ||
        provenance.unfoldedControlDigest == 0) {
        return false;
    }
    RigExecFrameCacheKey newKey;
    newKey.compiledRigSignature = oldKey.compiledRigSignature;
    newKey.controlDigest = RigExecFoldConstantDigest(
        provenance.unfoldedControlDigest, newConstants);
    if (newKey.controlDigest == oldKey.controlDigest) {
        // No namespace move for this entry -- the caller stands it.
        return false;
    }
    RigExecRigPose pose;
    std::shared_ptr<const void> retained;
    size_t bytes = 0;
    if (!_frameCache->Lookup(oldKey, &pose, &retained, &bytes) || !pose.valid ||
        !retained) {
        return false;
    }
    // The rewrite sequence: publish under the new key, evict the old,
    // re-record the completion -- which REPLACES the row, so no Drop (a
    // Drop erases by (compiled rig, time) and would eat the new row). The old key
    // is unreachable by construction past the namespace move (no fresh
    // sample folds the old constants), so the evict frees without robbing
    // aliases: a frame whose values match serves the carried entry.
    RigExecEntryProvenance carried = provenance;
    if (time.IsNumeric()) {
        carried.aliasTimes.clear();
        carried.aliasTimes.push_back(time.GetValue());
    }
    if (!_frameCache->Publish(newKey, time, pose, bytes, retained, &carried)) {
        return false;
    }
    _frameCache->Evict(oldKey);
    if (_warmIndex && time.IsNumeric()) {
        _warmIndex->NoteCompleted(_rigPath, time.GetValue(), newKey,
                                  _warmIndex->CurrentGeneration(_rigPath));
    }
    RepointProof(call, newKey.controlDigest);
    return true;
}

bool
RigExecBridgeFrameCache::_TrySparseServe(const RigExecCacheCall &call,
                                         const RigExecFrameInputs &inputs,
                                         const RigExecFrameCacheKey &key,
                                         uint64_t unfolded,
                                         RigExecCacheServe *serve)
{
    if (!serve || !_frameCache || !_warmIndex) {
        return false;
    }
    const UsdTimeCode time = call.time;
    if (!time.IsNumeric()) {
        return false;
    }
    const RigExecBakedProgram *program = call.evaluator->GetBakedProgram();
    if (!program) {
        return false;
    }
    const double stamped = time.GetValue();
    RigExecFrameCacheKey baseKey{0, 0};
    if (!_warmIndex->FindKey(_rigPath, stamped, &baseKey)) {
        return false;
    }
    // Same key, retired proof: the entry stands under the fresh digest
    // (a no-op edit retired the proof without moving values). Serve it
    // and heal the proof -- no re-key, no pulls.
    if (baseKey == key) {
        RigExecRigPose cached;
        bool hit = false;
        {
            RigExecProfileScope scope(_Profiler(call),
                                      "Imaging.FrameCacheLookup", "imaging");
            hit = _frameCache->Lookup(key, &cached);
        }
        _Profiler(call)->RecordCacheLookup(hit, stamped);
        if (!hit || !cached.valid) {
            return false;
        }
        _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
            key.controlDigest, unfolded, &inputs, call.overrides);
        _warmIndex->NoteCompleted(_rigPath, stamped, key,
                                  _warmIndex->CurrentGeneration(_rigPath));
        _CompleteServe(call, cached, key, serve);
        return true;
    }
    RigExecRigPose base;
    std::shared_ptr<const void> handle;
    bool baseHit = false;
    {
        RigExecProfileScope scope(_Profiler(call), "Imaging.FrameCacheLookup",
                                  "imaging");
        baseHit = _frameCache->Lookup(baseKey, &base, &handle);
    }
    if (!baseHit || !base.valid || !handle) {
        // No retained base (evicted, or pose-only): the frame evaluates
        // live. Records the miss: the store was consulted.
        _Profiler(call)->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecRetainedFrameState *retained =
        static_cast<const RigExecRetainedFrameState *>(handle.get());
    // The constant gate: sampled values exclude avar constants by design,
    // so a value comparison alone cannot see a constant patch. A moved
    // namespace plans nothing here -- the eager carry covers clean
    // entries, and anything else re-warms.
    if (retained->compiledRigSignature != key.compiledRigSignature ||
        retained->constantDigest !=
            RigExecCompiledRigConstantDigest(*program)) {
        _Profiler(call)->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecOutputAffectedIndex *index = _SyncAffectedIndex(call);
    if (!index || index->Empty()) {
        // An empty index answers empty sets, not all-clusters: planning
        // against it would Hit everything and serve stale poses. Live.
        _Profiler(call)->RecordCacheLookup(false, stamped);
        return false;
    }
    if (!RigExecCanReuseRetainedPose(*index, *retained,
                                     key.compiledRigSignature, inputs,
                                     *call.overrides)) {
        _Profiler(call)->RecordCacheLookup(false, stamped);
        return false;
    }
    // Proven identical with zero cluster work: re-key under the fresh
    // digest and serve with zero pulls. The old entry is deliberately
    // NOT evicted: another frame's values may still digest to it (an
    // alias serves it directly), and LRU drains what nobody serves.
    RigExecEntryProvenance provenance;
    const bool haveProvenance =
        _frameCache->LookupProvenance(baseKey, &provenance);
    if (haveProvenance) {
        provenance.unfoldedControlDigest = unfolded;
    }
    RigExecRigPose served = base;
    if (_frameCache->Publish(key, time, served, retained->RetainedBytes(),
                             handle, haveProvenance ? &provenance : nullptr)) {
        _warmIndex->NoteCompleted(_rigPath, stamped, key,
                                  _warmIndex->CurrentGeneration(_rigPath));
    }
    _freshDigests[{time.IsDefault(), stamped}] =
        RigExecFreshProof(key.controlDigest, unfolded, &inputs, call.overrides);
    _Profiler(call)->RecordCacheLookup(true, stamped);
    _CompleteServe(call, served, key, serve);
    return true;
}

bool
RigExecBridgeFrameCache::_LookupCachedPose(const RigExecCacheCall &call,
                                           bool publishing,
                                           RigExecRigPose *pose,
                                           RigExecFrameCacheKey *key,
                                           _CacheMiss *miss)
{
    const UsdTimeCode time = call.time;
    // Profiler scopes and lane records belong to the publishing lookup
    // only; a query that publishes nothing leaves the profiler alone.
    *miss = _CacheMiss();
    if (!_frameCache || call.evaluator->cpuReference) {
        return false;
    }
    // Scope check: a new compiled rig retires every
    // freshness proof. Cached entries stay for LRU (their digests decide
    // reachability); proofs name the old scope's inputs and must not be
    // consulted. No proof can exist yet, so this is a miss either way.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*call.evaluator);
    if (!_freshCompiledRigValid || _freshCompiledRig != compiledRig) {
        _freshDigests.clear();
        _freshCompiledRig = compiledRig;
        _freshCompiledRigValid = true;
        return false;
    }
    const auto storeLookup = [&](const RigExecFrameCacheKey &lookupKey,
                                 std::shared_ptr<const void> *retained) {
        if (!publishing) {
            return _frameCache->Lookup(lookupKey, pose, retained);
        }
        RigExecProfileScope scope(_Profiler(call), "Imaging.FrameCacheLookup",
                                  "imaging");
        return _frameCache->Lookup(lookupKey, pose, retained);
    };
    // Servable completions skip sampling entirely: the index row names the
    // exact key, and the retirement paths (generation tags, compiled rig tags,
    // path-scoped dirt, eviction) keep a clean row exact, so a store hit
    // under it IS the proof compare. Sampling first here would cost every
    // hit the full input vector, which can exceed the evaluation it saves.
    // A row miss, a store miss under the row (concurrent eviction), or a rig
    // with no baked program falls through to the proof/sparse paths below.
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const RigExecBakedProgram *program = call.evaluator->GetBakedProgram();
    const auto proof = _freshDigests.find({time.IsDefault(), stamped});
    const bool haveProof = proof != _freshDigests.end();
    if (haveProof && !proof->second.sampledInputs) {
        if (!_ComputePoseOnlyCacheKey(call, key) ||
            proof->second.digest != key->controlDigest) {
            return false;
        }
        const bool hit = storeLookup(*key, nullptr);
        if (publishing) {
            _Profiler(call)->RecordCacheLookup(hit, stamped);
        }
        if (!hit || !pose->valid) {
            *pose = RigExecRigPose();
            return false;
        }
        return true;
    }
    if (_warmIndex && time.IsNumeric()) {
        RigExecFrameCacheKey cachedKey{0, 0};
        if (_warmIndex->FindCachedKey(_rigPath, stamped,
                                      _warmIndex->CurrentGeneration(_rigPath),
                                      compiledRig, &cachedKey)) {
            std::shared_ptr<const void> retained;
            RigExecFrameCacheKey poseOnlyKey;
            const bool currentPoseOnly =
                _ComputePoseOnlyCacheKey(call, &poseOnlyKey) &&
                cachedKey == poseOnlyKey;
            const bool hit = storeLookup(cachedKey, &retained);
            // A pose-only entry whose proof was retired cannot use an
            // index row as its edit-serial check. Sampled entries carry
            // retained inputs (or their enqueue-time proof).
            if (hit && pose->valid &&
                (currentPoseOnly || (program && (retained || haveProof)))) {
                if (publishing) {
                    _Profiler(call)->RecordCacheLookup(true, stamped);
                }
                *key = cachedKey;
                return true;
            }
            *pose = RigExecRigPose();
            // Store miss under a live row: unrecorded, so the lane keeps
            // one record per SetTime -- the fallthrough records its own.
        }
    }
    // The proof is consulted by time BEFORE anything is sampled: a time with
    // no proof cannot hit whatever its digest says, and a cold frame is the
    // common case on a first scrub. Sampling and digesting it here would
    // make every miss pay the full vector twice (once here, once in the
    // memoization after the live run), which is exactly the cold-frame
    // latency the cache-off path never pays. A time with no proof but WITH
    // a retained base still samples when publishing: the sparse plan may
    // serve it with zero pulls, which is worth one sample.
    RigExecFrameCacheKey baseKey{0, 0};
    // Only the sparse lane reads the base, and it runs only when publishing;
    // a non-publishing query with no proof cannot hit, so it skips the sample.
    miss->haveBase = publishing && _warmIndex && time.IsNumeric() &&
                     _warmIndex->FindKey(_rigPath, stamped, &baseKey);
    if (!haveProof && !miss->haveBase) {
        return false;
    }
    if (!program || !call.sampling->CanSampleCacheInputs()) {
        return false;
    }
    // One sample serves the proof compare and the sparse plan.
    {
        std::optional<RigExecProfileScope> sampleScope;
        if (publishing) {
            sampleScope.emplace(_Profiler(call), "Imaging.LookupSampleDigest",
                                "imaging");
        }
        if (!call.sampling->SampleLive(time, *call.overrides, *call.upstream,
                                       &miss->inputs) ||
            !RigExecControlStateDigestible(miss->inputs, *call.overrides)) {
            return false;
        }
        miss->unfolded =
            RigExecControlStateDigest(miss->inputs, *call.overrides);
    }
    miss->sampled = true;
    miss->key.compiledRigSignature = compiledRig;
    miss->key.controlDigest = RigExecFoldConstantDigest(
        miss->unfolded, RigExecCompiledRigConstantDigest(*program));
    if (haveProof && proof->second.digest == miss->key.controlDigest) {
        // The frameCache lane records actual store consultations only: a
        // lookup that never reaches the store (no proof, unsampleable,
        // stale scope) records nothing.
        const bool hit = storeLookup(miss->key, nullptr);
        if (publishing) {
            _Profiler(call)->RecordCacheLookup(hit, stamped);
        }
        if (hit && pose->valid) {
            *key = miss->key;
            return true;
        }
        // Proven but evicted: the sparse base may still serve (a re-keyed
        // entry under a moved digest), so the caller plans on the miss.
        *pose = RigExecRigPose();
    }
    return false;
}

bool
RigExecBridgeFrameCache::TryServe(const RigExecCacheCall &call,
                                  RigExecCacheServe *serve)
{
    if (!call.evaluator || !call.sampling || !serve) {
        return false;
    }
    *serve = RigExecCacheServe();
    RigExecRigPose cached;
    RigExecFrameCacheKey key{0, 0};
    _CacheMiss miss;
    if (_LookupCachedPose(call, /* publishing = */ true, &cached, &key,
                          &miss)) {
        _CompleteServe(call, cached, key, serve);
        return true;
    }
    // No proof the pre-evaluation sample is fresh -- never evaluated here,
    // retired by an edit, or the inputs moved since. Chain-resolved inputs
    // sampled for a time the evaluator has not run come from the standing
    // resolved state, so an unproven sample must never serve directly; the
    // sparse plan compares it by VALUE against this time's own retained
    // base instead, which a stale sample cannot forge into a Hit.
    if (!miss.sampled || !miss.haveBase) {
        return false;
    }
    return _TrySparseServe(call, miss.inputs, miss.key, miss.unfolded, serve);
}

bool
RigExecBridgeFrameCache::QueryLookup(const RigExecCacheCall &call,
                                     RigExecRigPose *pose)
{
    if (!call.evaluator || !pose) {
        return false;
    }
    RigExecFrameCacheKey key{0, 0};
    _CacheMiss miss;
    return _LookupCachedPose(call, /* publishing = */ false, pose, &key, &miss);
}

bool
RigExecBridgeFrameCache::MemoizeLive(const RigExecCacheCall &call,
                                     const RigExecRigPose &pose)
{
    if (!_frameCache || !pose.valid) {
        return false;
    }
    const UsdTimeCode time = call.time;
    // Scope check, as on the lookup path: the evaluation above may have
    // settled a new compiled rig.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*call.evaluator);
    if (!_freshCompiledRigValid || _freshCompiledRig != compiledRig) {
        _freshDigests.clear();
        _freshCompiledRig = compiledRig;
        _freshCompiledRigValid = true;
    }
    // Sampled AFTER the evaluation, so the resolved state is fresh at exactly
    // this time: the digest below is the TRUE digest, and recording it as
    // this time's proof is what later lookups prove freshness against.
    // Sampled once, here, for the key AND the retained state:
    RigExecFrameCacheKey key;
    RigExecFrameInputs memoInputs;
    uint64_t memoUnfolded = 0;
    bool haveInputs = false;
    {
        RigExecProfileScope scope(_Profiler(call),
                                  "Imaging.MemoizeSampleDigest", "imaging");
        if (call.sampling->CanSampleCacheInputs() &&
            call.sampling->SampleLive(time, *call.overrides, *call.upstream,
                                      &memoInputs) &&
            !memoInputs.HasChainResolvedInputs() &&
            RigExecControlStateDigestible(memoInputs, *call.overrides)) {
            key.compiledRigSignature =
                RigExecFrameCacheCompiledRigSignature(*call.evaluator);
            const uint64_t unfolded =
                RigExecControlStateDigest(memoInputs, *call.overrides);
            key.controlDigest = RigExecFoldConstantDigest(
                unfolded, RigExecCompiledRigConstantDigest(
                              *call.evaluator->GetBakedProgram()));
            memoUnfolded = unfolded;
            haveInputs = true;
        } else if (!_ComputePoseOnlyCacheKey(call, &key)) {
            return false;
        }
    }
    bool stored = false;
    {
        RigExecProfileScope scope(_Profiler(call), "Imaging.FrameCachePublish",
                                  "imaging");
        if (haveInputs) {
            // Retained-state publish: the source snapshot and
            // the dependency record ride with the pose, so a later edit
            // retains the source snapshot used to validate cache reuse.
            const RigExecBakedProgramImpl &impl =
                call.evaluator->GetBakedProgram()->GetStepGraph();
            RigExecRetainedFrameState retainedState =
                RigExecCaptureRetainedState(
                    memoInputs, *call.overrides, key.compiledRigSignature,
                    impl.clustering.clusters.size(),
                    RigExecCompiledRigConstantDigest(
                        *call.evaluator->GetBakedProgram()));
            auto retained = std::make_shared<RigExecRetainedFrameState>(
                std::move(retainedState));
            const size_t retainedBytes = retained->RetainedBytes();
            RigExecEntryProvenance provenance =
                RigExecFullEvalProvenance(impl, time);
            provenance.unfoldedControlDigest = memoUnfolded;
            stored = _frameCache->Publish(key, time, pose, retainedBytes,
                                          retained, &provenance);
        } else {
            stored = _frameCache->Publish(key, time, pose);
        }
    }
    // A memoized entry is a cached frame: record the completion the strip
    // reads, under the generation this evaluation ran in. Default times
    // are not strip frames and record nothing.
    if (stored && _warmIndex && time.IsNumeric()) {
        _warmIndex->NoteCompleted(_rigPath, time.GetValue(), key,
                                  _warmIndex->CurrentGeneration(_rigPath));
    }
    // The compiled rig half frames were last stored under, for the notice path's
    // drift check -- kept even when the publish declines, so a later notice
    // evicts the half it would have named.
    _lastPublishedCompiledRig.store(key.compiledRigSignature,
                                    std::memory_order_relaxed);
    if (_freshDigests.size() >= _kFreshDigestCap) {
        _freshDigests.clear();
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    _freshDigests[{time.IsDefault(), stamped}] =
        RigExecFreshProof(key.controlDigest, memoUnfolded,
                          haveInputs ? &memoInputs : nullptr, call.overrides);
    // No override admission here: memoization runs only when no overrides
    // stand (the drag bypass), so the list is always empty. Admission
    // lives on the warming path, which samples under standing overrides.
    return stored;
}

} // namespace rigExec
