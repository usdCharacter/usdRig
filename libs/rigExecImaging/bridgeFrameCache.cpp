// Frame-cache operations owned by the imaging bridge: the freshness proofs,
// the affected index, warm-index rows, carry-over, sparse serving, lookup
// lanes and memoization. The members are still the bridge's; this TU holds
// their definitions.
#include "bridge.h"

#include "bridgeSampling.h"
#include "warmIndex.h"

#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCacheSparsity.h"

#include "pxr/base/tf/diagnostic.h"

#include <optional>
#include <set>
#include <string>
#include <utility>

namespace rigExec {

void
RigExecImagingBridge::SetWarmFrameIndex(
    std::shared_ptr<RigExecWarmFrameIndex> index)
{
    _warmIndex = std::move(index);
}

void
RigExecImagingBridge::ClearFrameCache()
{
    if (_frameCache) {
        _frameCache->Clear();
    }
    _taskListMemo.Clear();
    _lastPublishedCompiledRig.store(0, std::memory_order_relaxed);
    _freshDigests.clear();
    _freshCompiledRigValid = false;

    _sampling->InvalidateSampleSupport();
}

void
RigExecImagingBridge::RetirePoseOnlyCacheEntries()
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

void
RigExecImagingBridge::NoteWarmingEnqueued(UsdTimeCode time,
                                          uint64_t controlDigest,
                                          uint64_t unfoldedDigest,
                                          const RigExecFrameInputs &inputs)
{
    if (!_frameCache || _frameCacheMode == RigExecFrameCacheMode::Off) {
        return;
    }
    // Scope check, as on the lookup and memoization paths: the evaluation
    // compiled rig may have moved since the last proof was recorded.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    if (!_freshCompiledRigValid || _freshCompiledRig != compiledRig) {
        _freshDigests.clear();
        _freshCompiledRig = compiledRig;
        _freshCompiledRigValid = true;
    }
    if (_freshDigests.size() >= _kFreshDigestCap) {
        _freshDigests.clear();
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        controlDigest, unfoldedDigest, &inputs, &_evaluationOverrides);
}

const RigExecOutputAffectedIndex *
RigExecImagingBridge::_SyncAffectedIndex()
{
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        _affectedIndex.reset();
        _affectedValid = false;
        return nullptr;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
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
    AdmitOverrideControls(_evaluationOverrides);
    return _affectedIndex.get();
}

const RigExecOutputAffectedIndex *
RigExecImagingBridge::GetAffectedIndex()
{
    return _SyncAffectedIndex();
}

void
RigExecImagingBridge::AdmitOverrideControls(
    const std::vector<RigExecValueOverride> &overrides)
{
    if (overrides.empty()) {
        return;
    }
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        return;
    }
    const RigExecOutputAffectedIndex *synced = _SyncAffectedIndex();
    if (!synced || synced->Empty()) {
        // No oracle, no admission: the overrides stay foreign and every
        // plan over them stays conservative, as before.
        return;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    for (const RigExecValueOverride &o : overrides) {
        const RigExecControlId id = RigExecControlIdForOverride(o);
        if (_affectedIndex->IsKnownControl(id)) {
            continue;
        }
        const std::vector<int> seeds = RigExecOverrideSeeds(impl, o);
        if (seeds.empty()) {
            // Places nowhere (folded, exec-typed, routed, or unknown):
            // left foreign, never admitted seedless. A seedless
            // admission would claim the override reaches nothing,
            // which only the placement below can prove.
            continue;
        }
        _affectedIndex->MapControl(id, seeds);
    }
}

size_t
RigExecImagingBridge::RetireProofsForControls(
    const std::vector<RigExecControlId> &controls)
{
    if (_freshDigests.empty() || controls.empty()) {
        return 0;
    }
    // Each doomed id once, beside its path when the id spells one
    // canonically: then the id equals a sample path's text exactly when
    // the paths are equal, and an id that spells none equals no sample
    // path's text, so it can only name an override.
    std::vector<std::pair<std::string, SdfPath>> doomed;
    {
        const std::set<std::string> unique(controls.begin(), controls.end());
        doomed.reserve(unique.size());
        for (const std::string &id : unique) {
            SdfPath path =
                SdfPath::IsValidPathString(id) ? SdfPath(id) : SdfPath();
            if (!path.IsEmpty() && path.GetString() != id) {
                path = SdfPath();
            }
            doomed.emplace_back(id, std::move(path));
        }
    }
    size_t retired = 0;
    for (auto it = _freshDigests.begin(); it != _freshDigests.end();) {
        bool hit = false;
        for (const auto &[id, path] : doomed) {
            if (it->second.Covers(id, path)) {
                hit = true;
                break;
            }
        }
        if (hit) {
            it = _freshDigests.erase(it);
            ++retired;
        } else {
            ++it;
        }
    }
    return retired;
}

bool
RigExecImagingBridge::RepointProof(UsdTimeCode time, uint64_t newDigest)
{
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const auto found = _freshDigests.find({time.IsDefault(), stamped});
    if (found == _freshDigests.end()) {
        return false;
    }
    // The digest names the new namespace; the unfolded half and the paths
    // are untouched -- the sampled values did not move, only the constants
    // they fold beside did. A second carry re-folds the same unfolded half
    // against the newer constants.
    found->second.digest = newDigest;
    return true;
}

bool
RigExecImagingBridge::GetFreshProof(UsdTimeCode time,
                                    RigExecFreshProof *proof) const
{
    if (!proof) {
        return false;
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const auto found = _freshDigests.find({time.IsDefault(), stamped});
    if (found == _freshDigests.end()) {
        return false;
    }
    *proof = found->second;
    return true;
}

bool
RigExecImagingBridge::NoteCaptureIndex(const UsdNotice::ObjectsChanged &notice)
{
    // First and unconditionally: the pinned bindings serve every live sample,
    // with or without a frame cache.
    _sampling->NoteChainEdits(notice);
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program || !_frameCache) {
        return false;
    }
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    const bool hit = RigExecNoteCaptureIndex(*_frameCache, _taskListMemo,
                                             compiledRig, *program, notice);
    if (hit && _freshCompiledRigValid && _freshCompiledRig == compiledRig) {
        // The proofs name evicted entries: drop them so the next visit
        // misses without paying a sample for the proof check.
        _freshDigests.clear();
    }
    // Compiled rig drift since the last memoization (an in-place avar patch, a
    // rebuild, a structural edit): the old half's frames are unreachable by
    // construction, so evict them eagerly rather than waiting for LRU.
    const uint64_t last =
        _lastPublishedCompiledRig.load(std::memory_order_relaxed);
    if (last != compiledRig) {
        _frameCache->EvictCompiledRig(last);
        _taskListMemo.InvalidateCompiledRig(last);
        if (_freshCompiledRigValid && _freshCompiledRig == last) {
            _freshDigests.clear();
        }
    }
    return hit;
}

RigExecFrameCacheStats
RigExecImagingBridge::GetFrameCacheStats() const
{
    if (!_frameCache) {
        return RigExecFrameCacheStats();
    }
    return _frameCache->Stats();
}

bool
RigExecImagingBridge::_ComputePoseOnlyCacheKey(UsdTimeCode time,
                                               RigExecFrameCacheKey *key) const
{
    if (!key || !_frameCache ||
        !RigExecRefusalControlDigestible(_evaluationOverrides,
                                         _upstreamInputs)) {
        return false;
    }
    // Complete live results remain cacheable when an operation has no
    // frozen input contract. Every stage edit moves this namespace; no
    // per-operation input list or partial retained-state proof is needed.
    key->compiledRigSignature =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
    key->controlDigest =
        RigExecRefusalControlDigest(time, _evaluator->GetStageEditSerial(),
                                    _evaluationOverrides, _upstreamInputs);
    return true;
}

void
RigExecImagingBridge::_ServeCachedPose(UsdTimeCode time,
                                       const RigExecRigPose &pose,
                                       const RigExecFrameCacheKey &key,
                                       PublishResult *result)
{
    _frameCache->NoteProvenanceAlias(key, time);
    RigExecRigPose cached = pose;
    const bool verify = _frameCacheVerify;
    if (verify) {
        // Verify the cached pose against a live evaluation. This evaluator
        // pull counts as a live publication regardless of its verdict.
        RigExecRigPose live;
        {
            RigExecProfileScope scope(MutableProfiler(), "Imaging.Evaluate",
                                      "imaging");
            live = _evaluator->Evaluate(time);
        }
        const RigExecShadowVerdict verdict =
            RigExecVerifyHitWithLive(cached, [&live]() { return live; });
        if (!verdict.match && verdict.mismatches > 0) {
            TF_WARN("rigExec: frame-cache hit at %s mismatches live "
                    "evaluation; serving live and repairing the entry:\n%s",
                    time.IsDefault() ? "default"
                                     : std::to_string(time.GetValue()).c_str(),
                    verdict.report.c_str());
            _PublishPoseSnapshot(time, verdict.poseToServe, result);
            _MemoizeLiveResult(time, verdict.poseToServe);
            result->cacheHit = false;
            return;
        }
        // A match serves the cached pose (proven identical); an unverified
        // shadow (live failed) serves it too, advisory-only. Either way the
        // evaluator ran, so cacheHit stays false.
        cached.time = time;
        _PublishPoseSnapshot(time, cached, result);
        result->cacheHit = false;
        return;
    }
    // Guide styling reads pose.time; the stored pose was evaluated for an
    // earlier visit, so it is re-stamped for the requested frame before
    // anything reads it.
    cached.time = time;
    _PublishPoseSnapshot(time, cached, result);
    result->cacheHit = true;
}

bool
RigExecImagingBridge::CarryEntry(UsdTimeCode time,
                                 const RigExecFrameCacheKey &oldKey,
                                 uint64_t newConstants)
{
    if (!_frameCache) {
        return false;
    }
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
    RepointProof(time, newKey.controlDigest);
    return true;
}

bool
RigExecImagingBridge::_TrySparseServe(UsdTimeCode time,
                                      const RigExecFrameInputs &inputs,
                                      const RigExecFrameCacheKey &key,
                                      uint64_t unfolded, PublishResult *result)
{
    if (!result || !_frameCache || !_warmIndex || !time.IsNumeric()) {
        return false;
    }
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
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
            RigExecProfileScope scope(MutableProfiler(),
                                      "Imaging.FrameCacheLookup", "imaging");
            hit = _frameCache->Lookup(key, &cached);
        }
        MutableProfiler()->RecordCacheLookup(hit, stamped);
        if (!hit || !cached.valid) {
            return false;
        }
        _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
            key.controlDigest, unfolded, &inputs, &_evaluationOverrides);
        _warmIndex->NoteCompleted(_rigPath, stamped, key,
                                  _warmIndex->CurrentGeneration(_rigPath));
        _ServeCachedPose(time, cached, key, result);
        return true;
    }
    RigExecRigPose base;
    std::shared_ptr<const void> handle;
    bool baseHit = false;
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.FrameCacheLookup",
                                  "imaging");
        baseHit = _frameCache->Lookup(baseKey, &base, &handle);
    }
    if (!baseHit || !base.valid || !handle) {
        // No retained base (evicted, or pose-only): the frame evaluates
        // live. Records the miss: the store was consulted.
        MutableProfiler()->RecordCacheLookup(false, stamped);
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
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecOutputAffectedIndex *index = _SyncAffectedIndex();
    if (!index || index->Empty()) {
        // An empty index answers empty sets, not all-clusters: planning
        // against it would Hit everything and serve stale poses. Live.
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    if (!RigExecCanReuseRetainedPose(*index, *retained,
                                     key.compiledRigSignature, inputs,
                                     _evaluationOverrides)) {
        MutableProfiler()->RecordCacheLookup(false, stamped);
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
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        key.controlDigest, unfolded, &inputs, &_evaluationOverrides);
    MutableProfiler()->RecordCacheLookup(true, stamped);
    _ServeCachedPose(time, served, key, result);
    return true;
}

uint64_t
RigExecImagingBridge::_NoteCacheEditSerial()
{
    // A bare bridge has no registry to retire proofs on edits.
    const uint64_t serial = _evaluator->GetStageEditSerial();
    if (!_warmIndex && _cacheEditSerial != serial) {
        _freshDigests.clear();
    }
    _cacheEditSerial = serial;
    return serial;
}

bool
RigExecImagingBridge::_LookupCachedPose(UsdTimeCode time, bool publishing,
                                        RigExecRigPose *pose,
                                        RigExecFrameCacheKey *key,
                                        _CacheMiss *miss)
{
    // Profiler scopes and lane records belong to the publishing lookup
    // only; a query that publishes nothing leaves the profiler alone.
    *miss = _CacheMiss();
    if (!_frameCache || _evaluator->cpuReference) {
        return false;
    }
    // Scope check: a new compiled rig retires every
    // freshness proof. Cached entries stay for LRU (their digests decide
    // reachability); proofs name the old scope's inputs and must not be
    // consulted. No proof can exist yet, so this is a miss either way.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
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
        RigExecProfileScope scope(MutableProfiler(), "Imaging.FrameCacheLookup",
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
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    const auto proof = _freshDigests.find({time.IsDefault(), stamped});
    const bool haveProof = proof != _freshDigests.end();
    if (haveProof && !proof->second.sampledInputs) {
        if (!_ComputePoseOnlyCacheKey(time, key) ||
            proof->second.digest != key->controlDigest) {
            return false;
        }
        const bool hit = storeLookup(*key, nullptr);
        if (publishing) {
            MutableProfiler()->RecordCacheLookup(hit, stamped);
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
                _ComputePoseOnlyCacheKey(time, &poseOnlyKey) &&
                cachedKey == poseOnlyKey;
            const bool hit = storeLookup(cachedKey, &retained);
            // A pose-only entry whose proof was retired cannot use an
            // index row as its edit-serial check. Sampled entries carry
            // retained inputs (or their enqueue-time proof).
            if (hit && pose->valid &&
                (currentPoseOnly || (program && (retained || haveProof)))) {
                if (publishing) {
                    MutableProfiler()->RecordCacheLookup(true, stamped);
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
    if (!program || !_sampling->CanSampleCacheInputs()) {
        return false;
    }
    // One sample serves the proof compare and the sparse plan.
    {
        std::optional<RigExecProfileScope> sampleScope;
        if (publishing) {
            sampleScope.emplace(MutableProfiler(), "Imaging.LookupSampleDigest",
                                "imaging");
        }
        if (!_sampling->SampleLive(time, _evaluationOverrides, _upstreamInputs,
                                   &miss->inputs) ||
            !RigExecControlStateDigestible(miss->inputs,
                                           _evaluationOverrides)) {
            return false;
        }
        miss->unfolded =
            RigExecControlStateDigest(miss->inputs, _evaluationOverrides);
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
            MutableProfiler()->RecordCacheLookup(hit, stamped);
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
RigExecImagingBridge::_TryPublishCachedResult(UsdTimeCode time,
                                              PublishResult *result)
{
    if (!result) {
        return false;
    }
    RigExecRigPose cached;
    RigExecFrameCacheKey key{0, 0};
    _CacheMiss miss;
    if (_LookupCachedPose(time, /* publishing = */ true, &cached, &key,
                          &miss)) {
        _ServeCachedPose(time, cached, key, result);
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
    return _TrySparseServe(time, miss.inputs, miss.key, miss.unfolded, result);
}

bool
RigExecImagingBridge::_MemoizeLiveResult(UsdTimeCode time,
                                         const RigExecRigPose &pose)
{
    if (!_frameCache || !pose.valid) {
        return false;
    }
    // Scope check, as on the lookup path: the evaluation above may have
    // settled a new compiled rig.
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*_evaluator);
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
        RigExecProfileScope scope(MutableProfiler(),
                                  "Imaging.MemoizeSampleDigest", "imaging");
        if (_sampling->CanSampleCacheInputs() &&
            _sampling->SampleLive(time, _evaluationOverrides, _upstreamInputs,
                                  &memoInputs) &&
            !memoInputs.HasChainResolvedInputs() &&
            RigExecControlStateDigestible(memoInputs, _evaluationOverrides)) {
            key.compiledRigSignature =
                RigExecFrameCacheCompiledRigSignature(*_evaluator);
            const uint64_t unfolded =
                RigExecControlStateDigest(memoInputs, _evaluationOverrides);
            key.controlDigest = RigExecFoldConstantDigest(
                unfolded, RigExecCompiledRigConstantDigest(
                              *_evaluator->GetBakedProgram()));
            memoUnfolded = unfolded;
            haveInputs = true;
        } else if (!_ComputePoseOnlyCacheKey(time, &key)) {
            return false;
        }
    }
    bool stored = false;
    {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Imaging.FrameCachePublish", "imaging");
        if (haveInputs) {
            // Retained-state publish: the source snapshot and
            // the dependency record ride with the pose, so a later edit
            // retains the source snapshot used to validate cache reuse.
            const RigExecBakedProgramImpl &impl =
                _evaluator->GetBakedProgram()->GetStepGraph();
            RigExecRetainedFrameState retainedState =
                RigExecCaptureRetainedState(
                    memoInputs, _evaluationOverrides, key.compiledRigSignature,
                    impl.clustering.clusters.size(),
                    RigExecCompiledRigConstantDigest(
                        *_evaluator->GetBakedProgram()));
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
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        key.controlDigest, memoUnfolded, haveInputs ? &memoInputs : nullptr,
        &_evaluationOverrides);
    // No override admission here: memoization runs only when no overrides
    // stand (the drag bypass), so the list is always empty. Admission
    // lives on the warming path, which samples under standing overrides.
    return stored;
}

} // namespace rigExec
