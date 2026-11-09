// Frame-cache component, part 2: freshness proofs and retirement. Same
// component as bridgeFrameCache.cpp (bridgeFrameCache.h): the proof map's
// compiled-rig scope clearing and digest-cap retirement, the warming-
// enqueue proof record, path-scoped retirement through the compiled rig's
// output-affected index (override admission included), the carry-over
// proof re-point, and the capture-index notice routing. The digest-order
// dependency storage and the RigExecFreshProof contract (bridgeFreshness.h)
// are shared with the storage TU: no second proof mechanism, no parallel
// dependency model.
#include "bridgeFrameCache.h"

#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/rigEvaluator.h"

#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

void
RigExecBridgeFrameCache::NoteWarmingEnqueued(const RigExecCacheCall &call,
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
        RigExecFrameCacheCompiledRigSignature(*call.evaluator);
    if (!_freshCompiledRigValid || _freshCompiledRig != compiledRig) {
        _freshDigests.clear();
        _freshCompiledRig = compiledRig;
        _freshCompiledRigValid = true;
    }
    if (_freshDigests.size() >= _kFreshDigestCap) {
        _freshDigests.clear();
    }
    const double stamped = call.time.IsDefault() ? 0.0 : call.time.GetValue();
    _freshDigests[{call.time.IsDefault(), stamped}] = RigExecFreshProof(
        controlDigest, unfoldedDigest, &inputs, call.overrides);
}

void
RigExecBridgeFrameCache::AdmitOverrideControls(
    const RigExecCacheCall &call,
    const std::vector<RigExecValueOverride> &overrides)
{
    if (overrides.empty()) {
        return;
    }
    const RigExecBakedProgram *program = call.evaluator->GetBakedProgram();
    if (!program) {
        return;
    }
    const RigExecOutputAffectedIndex *synced = _SyncAffectedIndex(call);
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
RigExecBridgeFrameCache::RetireProofsForControls(
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
RigExecBridgeFrameCache::RepointProof(const RigExecCacheCall &call,
                                      uint64_t newDigest)
{
    const double stamped = call.time.IsDefault() ? 0.0 : call.time.GetValue();
    const auto found = _freshDigests.find({call.time.IsDefault(), stamped});
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
RigExecBridgeFrameCache::GetFreshProof(const RigExecCacheCall &call,
                                       RigExecFreshProof *proof) const
{
    if (!proof) {
        return false;
    }
    const double stamped = call.time.IsDefault() ? 0.0 : call.time.GetValue();
    const auto found = _freshDigests.find({call.time.IsDefault(), stamped});
    if (found == _freshDigests.end()) {
        return false;
    }
    *proof = found->second;
    return true;
}

bool
RigExecBridgeFrameCache::NoteCaptureIndex(
    const RigExecCacheCall &call, const UsdNotice::ObjectsChanged &notice)
{
    const RigExecBakedProgram *program = call.evaluator->GetBakedProgram();
    if (!program || !_frameCache) {
        return false;
    }
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(*call.evaluator);
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

} // namespace rigExec
