// RigExec imaging frames on demand: the stamp that scopes the frame table,
// the locked read of one frame from the sessions' frame caches, and the
// service call that turns Hydra readers' demand into table entries. This is
// how a host reads a frame the display does not hold (docs/specs/
// frames-on-demand.md). It never evaluates a live rig except through the
// documented opt-in fallback, never samples topology per frame, and adds no
// time record to any .rigexec.
#include "registry.h"

#include "frameTable.h"

#include "pxr/base/work/utils.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {

uint64_t
RigExecImagingRegistry::_FrameTableStampLocked() const
{
    // Everything an entry depends on besides its time: the stage, the
    // compiled rigs, their edit serials and their upstream values.
    uint64_t hash = 1469598103934665603ULL;
    const auto mix = [&hash](uint64_t value) {
        hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
    };
    mix(reinterpret_cast<uintptr_t>(get_pointer(_stage)));
    mix(_publishedCompiledRigId);
    for (const RigSession &session : _sessions) {
        mix(session.rigPath.GetHash());
        mix(session.upstreamSerial);
        if (session.playback) {
            mix(session.playback->GetCompiledRigSignature());
        } else if (session.bridge) {
            mix(session.bridge->GetCompiledRigSignature());
            mix(session.bridge->GetEvaluator().GetStageEditSerial());
        }
    }
    return hash;
}

bool
RigExecImagingRegistry::_CachedFrameLocked(
    double time, std::shared_ptr<RigExecImagingSnapshot> *frame, bool evaluate)
{
    const UsdTimeCode at(time);
    auto combined = std::make_shared<RigExecImagingSnapshot>();
    combined->stage = UsdStageWeakPtr(_stage);
    combined->sampleTimeIsDefault = false;
    combined->sampleTime = time;
    SdfPath commonAssetRoot;
    bool firstRoot = true;
    for (RigSession &session : _sessions) {
        // The upstream values that time reads, handed over the way SetTime
        // hands them, and put back afterwards with the dirty flag: the
        // displayed generation stays exactly as it was.
        const bool wasDirty = session.dirty;
        std::vector<RigExecUpstreamValue> held =
            session.playback ? session.playback->GetUpstreamInputs()
                             : session.bridge->GetUpstreamInputs();
        _HandOffUpstreamLocked(session, at);
        RigExecImagingSnapshotConstPtr part;
        bool ok = false;
        if (session.playback) {
            // A .rigexec playback has no frame cache: it plays its binary
            // at the time into its own store (the chains read the combined
            // store, never this one) and the displayed one is put back.
            const RigExecImagingSnapshotConstPtr shown = session.store->Get();
            const RigExecImagingBridge::PublishResult played =
                session.playback->EvaluateAndPublishResult(at);
            part = session.store->Get();
            session.store->Replace(
                shown ? std::make_shared<RigExecImagingSnapshot>(*shown)
                      : nullptr);
            ok = played.ok && part && part->Describes(_stage, at);
        } else if (session.bridge) {
            std::string diagnostic;
            bool evaluated = false;
            ok = session.bridge->SnapshotAtTimeWithoutPublishing(
                at, evaluate, &part, &diagnostic, &evaluated);
            if (evaluated) {
                ++session.evaluationCount;
            }
        }
        if (session.playback) {
            if (session.playback->GetUpstreamInputs() != held) {
                session.playback->SetUpstreamInputs(std::move(held));
            }
        } else if (session.bridge &&
                   session.bridge->GetUpstreamInputs() != held) {
            session.bridge->SetUpstreamInputs(std::move(held));
        }
        session.dirty = wasDirty;
        if (!ok || !part) {
            if (!session.playback) {
                // Not cached yet: the background workers warm it (the idle
                // sweep visits the rewarm lane first).
                auto &lane = session.pendingRewarm;
                if (std::find(lane.begin(), lane.end(), time) == lane.end()) {
                    if (lane.size() >= RigExecFrameTable::kMaxDemand) {
                        lane.erase(lane.begin());
                    }
                    lane.push_back(time);
                }
            }
            return false;
        }
        combined->xformResetPaths.insert(part->xformResetPaths.begin(),
                                         part->xformResetPaths.end());
        if (firstRoot) {
            commonAssetRoot = session.assetRoot;
            firstRoot = false;
        } else if (commonAssetRoot != session.assetRoot) {
            commonAssetRoot = SdfPath();
        }
        for (const auto &[path, published] : part->prims) {
            // Two rigs never publish one prim (SetTime refuses it).
            combined->prims.emplace(path, published);
        }
    }
    combined->assetRoot = commonAssetRoot;
    // What RigExecSnapshotStore::Publish derives for a published generation.
    for (const auto &[path, prim] : combined->prims) {
        if (prim.hasXform) {
            combined->hasDrivenXforms = true;
            break;
        }
    }
    *frame = std::move(combined);
    return true;
}

size_t
RigExecImagingRegistry::ServiceFrameDemand(size_t maxFrames, bool allowFallback)
{
    size_t added = 0;
    bool reset = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_stage) {
            _frameTable->Clear();
            _pendingFrameDemand.clear();
            _pendingFrameAge.clear();
            return 0;
        }
        reset = _frameTable->ResetIfStale(_FrameTableStampLocked());
        // A live preview's displayed pose is not the authored one; frames
        // asked for meanwhile wait for it to end.
        bool previewing = _previewActive;
        for (const RigSession &session : _sessions) {
            previewing = previewing ||
                         (session.bridge &&
                          !session.bridge->GetInteractiveOverrides().empty());
        }
        std::vector<double> wanted = _frameTable->TakeDemand();
        wanted.insert(wanted.end(), _pendingFrameDemand.begin(),
                      _pendingFrameDemand.end());
        _pendingFrameDemand.clear();
        const double playhead =
            _lastTime.IsNumeric() ? _lastTime.GetValue() : 0.0;
        std::sort(wanted.begin(), wanted.end(), [playhead](double a, double b) {
            const double da = std::abs(a - playhead);
            const double db = std::abs(b - playhead);
            return da != db ? da < db : a > b;
        });
        std::set<int64_t> seen;
        bool fallbackUsed = false;
        std::map<int64_t, int> ages;
        for (const double time : wanted) {
            const int64_t key = RigExecFrameTable::Key(time);
            if (!std::isfinite(time) || !seen.insert(key).second ||
                _frameTable->Contains(time)) {
                continue;
            }
            const auto aged = _pendingFrameAge.find(key);
            const int age =
                (aged == _pendingFrameAge.end() ? 0 : aged->second) + 1;
            std::shared_ptr<RigExecImagingSnapshot> frame;
            if (!previewing && added < maxFrames) {
                bool ready = _CachedFrameLocked(time, &frame);
                if (!ready && allowFallback && !fallbackUsed &&
                    age >= kFrameDemandFallbackAge) {
                    // Still not warmed: one evaluation here, so the frame
                    // is answered even when no worker ever will.
                    fallbackUsed = true;
                    ready = _CachedFrameLocked(time, &frame,
                                               /* evaluate = */ true);
                }
                if (ready) {
                    _frameTable->Insert(time, std::move(frame), playhead);
                    ++added;
                    continue;
                }
            }
            if (_pendingFrameDemand.size() < RigExecFrameTable::kMaxDemand) {
                _pendingFrameDemand.push_back(time);
                ages[key] = age;
            }
        }
        _pendingFrameAge.swap(ages);
    }
    if (added || reset) {
        _BroadcastFramesChanged();
    }
    return added;
}

bool
RigExecImagingRegistry::HasFrameDemand()
{
    if (_frameTable->HasDemand()) {
        return true;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return !_pendingFrameDemand.empty();
}

void
RigExecImagingRegistry::_BroadcastFramesChanged()
{
    // Unlocked, like _Broadcast: readers re-enter through their dirties.
    for (const Chain &chain : _BoundChains()) {
        if (chain.results) {
            chain.results->NotifyFramesChanged();
        }
    }
}

} // namespace rigExec
