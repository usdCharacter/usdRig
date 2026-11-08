// RigExec imaging registry: the imaging contexts and the directory that
// maps stages to them. The host-facing C surface is in registryCapi.cpp.
#include <fstream>
#include "registry.h"
#include "rigExecGpu/gpuDeformExecutor.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/parallel.h"

#include "pxr/base/work/utils.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdUtils/stageCache.h"

#include <algorithm>
#include <chrono>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

namespace rigExec {

// A session owns independent worker lanes. A lease protects lazy construction,
// snapshot replacement and the mutable retained execution state.
struct RigExecWarmWorkspacePool {
    struct Lane {
        std::atomic<bool> busy{false};
        std::shared_ptr<const RigExecFrozenProgram> snapshot;
        uint64_t programDigest = 0;
        std::unique_ptr<RigExecFrozenWorkspace> workspace;
    };
    std::array<Lane, kRigExecBackgroundSchedulerMaxWorkers> lanes;

    Lane *Acquire()
    {
        for (Lane &lane : lanes) {
            bool expected = false;
            if (lane.busy.compare_exchange_strong(expected, true))
                return &lane;
        }
        return nullptr;
    }
};

namespace {

const TfToken _tRigRootType("RigExecRoot");

uint64_t
_HashCompiledRigPart(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= uint64_t(bytes[i]);
        hash *= 1099511628211ULL;
    }
    return hash;
}

uint64_t
_HashCompiledRigPart(uint64_t hash, const std::string &value)
{
    return _HashCompiledRigPart(hash, value.data(), value.size());
}

// The entries of \p table at or under \p roots (prim paths): a session's
// share of a chain's upstream table. The table itself when every entry is
// the session's, null when none is.
std::shared_ptr<const RigExecUpstreamTable>
_UpstreamShare(const std::shared_ptr<const RigExecUpstreamTable> &table,
               const std::set<SdfPath> &roots)
{
    if (!table || table->entries.empty()) {
        return nullptr;
    }
    const auto mine = [&](const SdfPath &path) {
        const SdfPath prim = path.GetPrimPath();
        for (const SdfPath &root : roots) {
            if (prim.HasPrefix(root)) {
                return true;
            }
        }
        return false;
    };
    size_t kept = 0;
    for (const auto &[path, entry] : table->entries) {
        kept += mine(path) ? 1 : 0;
    }
    if (kept == table->entries.size()) {
        return table;
    }
    if (kept == 0) {
        return nullptr;
    }
    auto share = std::make_shared<RigExecUpstreamTable>(*table);
    for (auto it = share->entries.begin(); it != share->entries.end();) {
        it = mine(it->first) ? std::next(it) : share->entries.erase(it);
    }
    return share;
}

// Read dependencies may live outside the assets whose outputs we publish.
// Follow authored relationships and attribute connections transitively, also
// inspecting ancestors because provider frames can inherit their inputs.
std::set<SdfPath>
_CollectReadRoots(const UsdStageRefPtr &stage,
                  const std::set<SdfPath> &assetRoots)
{
    std::set<SdfPath> roots = assetRoots;
    std::vector<SdfPath> pending(assetRoots.begin(), assetRoots.end());
    std::set<SdfPath> scannedPrims;
    std::set<SdfPath> scannedSubtrees;
    const auto enqueue = [&](const SdfPath &target) {
        const SdfPath path = target.GetPrimPath();
        if (!path.IsAbsolutePath() || !path.IsPrimPath())
            return;
        roots.insert(path);
        for (const SdfPath &scope : scannedSubtrees) {
            if (path.HasPrefix(scope))
                return;
        }
        pending.push_back(path);
    };
    const auto scan = [&](const UsdPrim &prim) {
        if (!prim || !scannedPrims.insert(prim.GetPath()).second)
            return;
        for (const UsdRelationship &rel : prim.GetRelationships()) {
            SdfPathVector targets;
            rel.GetTargets(&targets);
            for (const SdfPath &target : targets)
                enqueue(target);
        }
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            SdfPathVector connections;
            attr.GetConnections(&connections);
            for (const SdfPath &target : connections)
                enqueue(target);
        }
    };
    while (!pending.empty()) {
        const SdfPath path = pending.back();
        pending.pop_back();
        if (!scannedSubtrees.insert(path).second)
            continue;
        const UsdPrim root = stage->GetPrimAtPath(path);
        if (!root)
            continue;
        for (const UsdPrim &prim : UsdPrimRange(root))
            scan(prim);
        for (UsdPrim parent = root.GetParent();
             parent && !parent.IsPseudoRoot(); parent = parent.GetParent()) {
            scan(parent);
        }
    }
    // Keep only maximal scopes; scanning a notice then costs the number of
    // independent dependency regions, not the number of connected properties.
    std::set<SdfPath> compact;
    for (const SdfPath &path : roots) {
        bool covered = false;
        for (const SdfPath &root : compact) {
            if (path.HasPrefix(root)) {
                covered = true;
                break;
            }
        }
        if (!covered)
            compact.insert(path);
    }
    return compact;
}

// The injected runner, or the production one when none was given. Null used
// to mean "skip every frame"; since the production runner factory exists it
// means "warm for real", which is what the C-API triggers run under.
RigExecFrozenStepRunner
_ProductionRunner(const RigExecFrozenStepRunner &runner)
{
    return runner ? runner : RigExecMakeProductionStepRunner();
}

// The nearest-frame-first sweep around \p playhead: closest-first, skipping
// the neighbor band the commit trigger already covers (+-1..N) and the
// playhead itself. 64 frames: one full commit burst (16 neighbors + 64
// sweep) fits the default per-rig in-flight cap with room to spare.
std::vector<UsdTimeCode>
_SweepTimes(UsdTimeCode playhead)
{
    std::vector<UsdTimeCode> times;
    if (playhead.IsDefault() || !std::isfinite(playhead.GetValue())) {
        return times;
    }
    const double center = playhead.GetValue();
    for (int i = kRigExecFrameCacheDefaultNeighborRadius + 1;
         i <= kRigExecFrameCacheDefaultNeighborRadius + 32; ++i) {
        times.push_back(UsdTimeCode(center - double(i)));
        times.push_back(UsdTimeCode(center + double(i)));
    }
    return times;
}

} // namespace

// The directory: stage -> imaging context (registry.h, "PER-STAGE CONTEXTS").

namespace {

// The warming pool a context owns while it has an active rig. The hook holds
// the index shared: transitions record into shared state past whatever the
// sessions do next, and a shutdown purge during destruction still lands (the
// hook keeps the index alive).
std::shared_ptr<RigExecBackgroundScheduler>
_MakeScheduler(const std::shared_ptr<RigExecWarmFrameIndex> &warmIndex)
{
    auto scheduler = std::make_shared<RigExecBackgroundScheduler>();
    std::shared_ptr<RigExecWarmFrameIndex> index = warmIndex;
    scheduler->SetTransitionHook(
        [index](const RigExecWarmTransition &transition) {
            index->NoteTransition(transition);
        });
    return scheduler;
}

// Whether the directory (a function-local static) exists right now. A
// results scene index released during static destruction -- an engine a
// host leaks until exit -- must not resurrect or touch a destroyed
// directory. Constant-initialized and trivially destructible, so it stays
// readable after the directory is gone.
std::atomic<bool> _directoryAlive{false};

// Adds \p from's counters into \p into. Gauges (queue depth, running) are
// instantaneous, so they are added only when \p gauges is set -- a stopped
// scheduler has none.
void
_AddSchedulerStats(RigExecBackgroundSchedulerStats *into,
                   const RigExecBackgroundSchedulerStats &from, bool gauges)
{
    if (gauges) {
        into->queuedDepth += from.queuedDepth;
        into->running += from.running;
    }
    into->completed += from.completed;
    into->published += from.published;
    into->declinedInvalid += from.declinedInvalid;
    into->declinedGeneration += from.declinedGeneration;
    into->droppedStale += from.droppedStale;
    into->canceled += from.canceled;
    into->shed += from.shed;
    into->coalesced += from.coalesced;
    into->upgraded += from.upgraded;
    into->declined += from.declined;
    into->declinedFreezeRefused += from.declinedFreezeRefused;
    into->declinedUnsampleable += from.declinedUnsampleable;
    into->declinedD7Exempt += from.declinedD7Exempt;
    into->droppedAtShutdown += from.droppedAtShutdown;
    into->factoryInvocations += from.factoryInvocations;
}

// An overlay selection string, checked before it reaches SdfPath (whose
// constructor is loud about a malformed one). Empty turns the overlay off.
bool
_ParseOverlayPath(const std::string &text, SdfPath *path)
{
    *path = SdfPath();
    if (text.empty()) {
        return true;
    }
    if (!SdfPath::IsValidPathString(text)) {
        return false;
    }
    const SdfPath resolved(text);
    if (!resolved.IsAbsolutePath() || !resolved.IsPrimPath()) {
        return false;
    }
    *path = resolved;
    return true;
}

} // namespace

/// Process-level map from a stage to its imaging context, the chain
/// registrations, and the legacy current context.
///
/// Its mutex is a LEAF (registry.h, LOCK ORDER): it may be taken while a
/// context holds its own _mutex, and while it is held nothing calls into a
/// context and no context is destroyed -- contexts it lets go of are handed
/// back to the caller and released after unlocking.
class RigExecImagingDirectory {
public:
    using Ctx = RigExecImagingRegistry;
    using Ptr = Ctx::Ptr;
    using Chain = Ctx::Chain;

    static RigExecImagingDirectory &Get()
    {
        static RigExecImagingDirectory directory;
        return directory;
    }

    Ctx &LegacyHandle()
    {
        return _legacyHandle;
    }

    Ptr ForStage(const UsdStageRefPtr &stage, bool create)
    {
        if (!stage) {
            return nullptr;
        }
        std::vector<Ptr> released;
        Ptr result;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _PruneLocked(&released);
            const UsdStage *raw = get_pointer(stage);
            const auto found = _keyByStage.find(raw);
            if (found != _keyByStage.end()) {
                const auto entry = _entries.find(found->second);
                // Pruning dropped every expired entry, so a surviving one
                // whose raw pointer matches IS this stage -- but compare the
                // weak identities anyway: an address is never an identity.
                if (entry != _entries.end() &&
                    entry->second.stage == UsdStageWeakPtr(stage)) {
                    result = entry->second.context;
                }
            }
            if (!result && create) {
                const uint64_t key = ++_nextKey;
                result = Ptr(new Ctx(key, stage));
                _entries[key] = Entry{UsdStageWeakPtr(stage), result};
                _keyByStage[raw] = key;
            }
        }
        return result; // `released` dies here, after the unlock
    }

    Ptr ForKey(uint64_t key)
    {
        if (key == 0) {
            return nullptr;
        }
        std::vector<Ptr> released;
        std::lock_guard<std::mutex> lock(_mutex);
        _PruneLocked(&released);
        const auto entry = _entries.find(key);
        Ptr result = entry == _entries.end() ? nullptr : entry->second.context;
        // `released` is declared before the guard, so it is destroyed after
        // the guard unlocks.
        return result;
    }

    size_t ContextCount()
    {
        std::vector<Ptr> released;
        std::lock_guard<std::mutex> lock(_mutex);
        _PruneLocked(&released);
        return _entries.size();
    }

    Ptr Current()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _current;
    }

    /// A legacy activation succeeded on \p context: it is current now.
    void SetCurrent(const Ptr &context)
    {
        Ptr previous;
        std::lock_guard<std::mutex> lock(_mutex);
        previous = std::move(_current);
        _current = context;
    }

    /// An automatic activation succeeded on \p context: it becomes current
    /// only when no current context is live, so it never takes the legacy
    /// surface away from an explicit activation.
    void OfferCurrent(const Ptr &context)
    {
        Ptr previous;
        std::lock_guard<std::mutex> lock(_mutex);
        if (_current && _current->_active.load()) {
            return;
        }
        previous = std::move(_current);
        _current = context;
    }

    void RegisterChain(const Chain &chain, uint64_t key)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _PruneChainsLocked();
        _chains.push_back(ChainRecord{chain, key});
    }

    /// Records that the chain whose results index is \p results is bound to
    /// \p key now, and reports the key it was bound to in \p previous.
    /// False (and \p chain untouched) for an unknown chain or one already
    /// bound to \p key.
    bool Rebind(const RigExecResultsSceneIndex *results, uint64_t key,
                Chain *chain, uint64_t *previous)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _PruneChainsLocked();
        for (ChainRecord &record : _chains) {
            if (get_pointer(record.chain.results) != results) {
                continue;
            }
            if (record.key == key) {
                return false;
            }
            *previous = record.key;
            record.key = key;
            *chain = record.chain;
            return true;
        }
        return false;
    }

    /// Forgets the chain whose results index is \p results -- by identity:
    /// it is called from that index's destructor, while its weak pointers
    /// still resolve -- and returns the key it was bound to (0 when unknown
    /// or unbound).
    uint64_t Unregister(const RigExecResultsSceneIndex *results)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        uint64_t key = 0;
        for (auto it = _chains.begin(); it != _chains.end();) {
            if (get_pointer(it->chain.results) == results) {
                key = it->key;
                it = _chains.erase(it);
            } else {
                ++it;
            }
        }
        _PruneChainsLocked();
        return key;
    }

    /// The live chains bound to \p key.
    std::vector<Chain> ChainsFor(uint64_t key)
    {
        std::vector<Chain> chains;
        std::lock_guard<std::mutex> lock(_mutex);
        _PruneChainsLocked();
        for (const ChainRecord &record : _chains) {
            if (record.key == key) {
                chains.push_back(record.chain);
            }
        }
        return chains;
    }

    ~RigExecImagingDirectory()
    {
        _directoryAlive.store(false);
    }

private:
    RigExecImagingDirectory() : _legacyHandle(Ctx::_LegacyHandleTag{})
    {
        _directoryAlive.store(true);
    }

    struct Entry {
        UsdStageWeakPtr stage;
        Ptr context;
    };
    struct ChainRecord {
        Chain chain;
        uint64_t key = 0;
    };

    /// Drops contexts whose stage has expired, handing them to \p released
    /// so their destructors run after the caller unlocks. An expired stage
    /// means an inactive context: an active one holds its stage.
    void _PruneLocked(std::vector<Ptr> *released)
    {
        for (auto it = _entries.begin(); it != _entries.end();) {
            if (it->second.stage) {
                ++it;
                continue;
            }
            for (auto raw = _keyByStage.begin(); raw != _keyByStage.end();) {
                raw = raw->second == it->first ? _keyByStage.erase(raw)
                                               : std::next(raw);
            }
            released->push_back(std::move(it->second.context));
            it = _entries.erase(it);
        }
    }

    void _PruneChainsLocked()
    {
        _chains.erase(std::remove_if(_chains.begin(), _chains.end(),
                                     [](const ChainRecord &record) {
                                         return !record.chain.results;
                                     }),
                      _chains.end());
    }

    std::mutex _mutex;
    uint64_t _nextKey = 0;
    std::map<uint64_t, Entry> _entries;
    std::map<const UsdStage *, uint64_t> _keyByStage;
    std::vector<ChainRecord> _chains;
    /// The legacy current context, held STRONGLY: the legacy handle hands
    /// out references into it (GetStore), exactly as the old singleton
    /// lived for the process. Replaced only by the next legacy (or, when
    /// none is live, automatic) activation.
    Ptr _current;
    /// Declared last: destroyed first at exit, while the map still stands.
    Ctx _legacyHandle;
};

RigExecImagingRegistry &
RigExecImagingRegistry::GetInstance()
{
    return RigExecImagingDirectory::Get().LegacyHandle();
}

RigExecImagingRegistry::Ptr
RigExecImagingRegistry::ForStage(const UsdStageRefPtr &stage, bool create)
{
    return RigExecImagingDirectory::Get().ForStage(stage, create);
}

RigExecImagingRegistry::Ptr
RigExecImagingRegistry::ForStageCacheId(long long id, bool create)
{
    const UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(
        UsdStageCache::Id::FromLongInt(static_cast<long int>(id)));
    return stage ? ForStage(stage, create) : nullptr;
}

RigExecImagingRegistry::Ptr
RigExecImagingRegistry::ForKey(uint64_t key)
{
    return RigExecImagingDirectory::Get().ForKey(key);
}

RigExecImagingRegistry::Ptr
RigExecImagingRegistry::Current()
{
    return RigExecImagingDirectory::Get().Current();
}

size_t
RigExecImagingRegistry::ContextCount()
{
    return RigExecImagingDirectory::Get().ContextCount();
}

RigExecImagingRegistry::RigExecImagingRegistry(uint64_t key,
                                               const UsdStageRefPtr &stage)
    : _key(key), _boundStage(stage), _legacyHandle(false),
      _store(std::make_shared<RigExecSnapshotStore>()),
      _warmIndex(std::make_shared<RigExecWarmFrameIndex>())
{
    // RIGEXEC_GPU_DEFORM=readback deforms on the GPU executor; the device
    // starts on its first run, not here.
    RigExecGpuRegisterDeformRunner();
    // Cloth solves on the GPU by default (RIGEXEC_GPU_CLOTH=0 keeps it on
    // the CPU); the device starts on the first solve.
    RigExecGpuRegisterClothRunner();
    // No scheduler: a context without an active rig owns no threads. The
    // first activation starts one (see Activate).
}

RigExecImagingRegistry::RigExecImagingRegistry(_LegacyHandleTag)
    : _key(0), _legacyHandle(true),
      _store(std::make_shared<RigExecSnapshotStore>()),
      _warmIndex(std::make_shared<RigExecWarmFrameIndex>())
{
}

RigExecImagingRegistry::~RigExecImagingRegistry() = default;

RigExecImagingRegistry::Ptr
RigExecImagingRegistry::_Routed() const
{
    return _legacyHandle ? RigExecImagingDirectory::Get().Current() : nullptr;
}

const std::shared_ptr<RigExecSnapshotStore> &
RigExecImagingRegistry::GetStore() const
{
    if (const Ptr routed = _Routed()) {
        // The directory holds the current context strongly, so the member
        // reference stays valid after `routed` goes out of scope.
        return routed->_store;
    }
    return _store;
}

bool
RigExecImagingRegistry::IsActive() const
{
    if (const Ptr routed = _Routed()) {
        return routed->IsActive();
    }
    return _active.load();
}

UsdStageRefPtr
RigExecImagingRegistry::GetBoundStage() const
{
    return UsdStageRefPtr(_boundStage);
}

UsdStageRefPtr
RigExecImagingRegistry::_PreviewStageLocked() const
{
    if (_stage) {
        return _stage;
    }
    return UsdStageRefPtr(_boundStage);
}

std::vector<RigExecImagingRegistry::Chain>
RigExecImagingRegistry::_BoundChains() const
{
    if (_legacyHandle) {
        // The handle owns no stage, so no chain is ever bound to it.
        return {};
    }
    return RigExecImagingDirectory::Get().ChainsFor(_key);
}

void
RigExecImagingRegistry::RegisterUnboundChain(
    const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
    const RigExecBindingResolvingSceneIndexRefPtr &binding,
    const RigExecResultsSceneIndexRefPtr &results,
    const RigExecXformOverrideSceneIndexRefPtr &xforms)
{
    if (results) {
        results->SetContextKey(0);
    }
    RigExecImagingDirectory::Get().RegisterChain(
        {TfWeakPtr<RigExecInternalPrimPruningSceneIndex>(get_pointer(pruning)),
         TfWeakPtr<RigExecBindingResolvingSceneIndex>(get_pointer(binding)),
         TfWeakPtr<RigExecResultsSceneIndex>(get_pointer(results)),
         TfWeakPtr<RigExecXformOverrideSceneIndex>(get_pointer(xforms))},
        0);
}

void
RigExecImagingRegistry::RegisterChain(
    const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
    const RigExecBindingResolvingSceneIndexRefPtr &binding,
    const RigExecResultsSceneIndexRefPtr &results,
    const RigExecXformOverrideSceneIndexRefPtr &xforms)
{
    if (const Ptr routed = _Routed()) {
        routed->RegisterChain(pruning, binding, results, xforms);
        return;
    }
    if (_legacyHandle) {
        RegisterUnboundChain(pruning, binding, results, xforms);
        return;
    }
    const Chain chain{
        TfWeakPtr<RigExecInternalPrimPruningSceneIndex>(get_pointer(pruning)),
        TfWeakPtr<RigExecBindingResolvingSceneIndex>(get_pointer(binding)),
        TfWeakPtr<RigExecResultsSceneIndex>(get_pointer(results)),
        TfWeakPtr<RigExecXformOverrideSceneIndex>(get_pointer(xforms))};
    RigExecImagingDirectory::Get().RegisterChain(chain, _key);
    // A chain built mid-manipulation -- a second viewport opened during a
    // drag -- starts out holding the deltas the others already have, and a
    // chain constructed after activation adopts every active rig's scope.
    _AdoptChain(chain);
}

void
RigExecImagingRegistry::BindChain(const RigExecResultsSceneIndex *results,
                                  uint64_t key)
{
    Chain chain;
    uint64_t previous = 0;
    if (!results || !RigExecImagingDirectory::Get().Rebind(results, key, &chain,
                                                           &previous)) {
        return;
    }
    if (const Ptr context = ForKey(key)) {
        context->_AdoptChain(chain);
    } else {
        // Unbound (or bound to a context whose stage is gone): nothing
        // stage-specific. Seeded as a fresh chain is.
        if (chain.results) {
            chain.results->SetContextKey(key);
        }
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes({});
        }
        if (chain.xforms) {
            chain.xforms->SetWorldDeltas({});
        }
        if (chain.binding) {
            chain.binding->SetCompiledRigBinding(nullptr);
        }
        if (chain.results) {
            chain.results->SetStore(std::make_shared<RigExecSnapshotStore>());
        }
    }
    // Its upstream sources described the old stage.
    if (const Ptr old = ForKey(previous)) {
        old->DropUpstreamTable(results);
    }
    // The stage this chain showed was replaced under its engine: when that
    // was the last chain of an automatic activation, nothing images the old
    // stage any more, and nothing else would release it.
    _ReleaseIfUnbound(previous);
}

void
RigExecImagingRegistry::ReleaseChain(const RigExecResultsSceneIndex *results)
{
    if (!results || !_directoryAlive.load()) {
        return;
    }
    const uint64_t key = RigExecImagingDirectory::Get().Unregister(results);
    if (const Ptr context = ForKey(key)) {
        context->DropUpstreamTable(results);
    }
    _ReleaseIfUnbound(key);
}

void
RigExecImagingRegistry::_ReleaseIfUnbound(uint64_t key)
{
    if (const Ptr context = ForKey(key)) {
        context->_Deactivate(/* byHost = */ false);
    }
}

void
RigExecImagingRegistry::_AdoptChain(const Chain &chain)
{
    if (chain.results) {
        chain.results->SetContextKey(_key);
    }
    std::set<SdfPath> scopes;
    std::map<SdfPath, GfMatrix4d> deltas;
    RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr compiledRig;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scopes = _generatedScopes;
        deltas = _previewXformDeltas;
        compiledRig = _publishedCompiledRig;
    }
    // Unlocked: every one of these sends scene index notices, and the
    // results index's time trigger re-enters this context on them.
    if (chain.pruning) {
        chain.pruning->SetOwnedScopes(scopes);
    }
    if (chain.xforms) {
        chain.xforms->SetWorldDeltas(deltas);
    }
    if (chain.binding) {
        chain.binding->SetCompiledRigBinding(compiledRig);
    }
    if (chain.results) {
        chain.results->SetStore(_store);
    }
}

bool
RigExecImagingRegistry::_LegacyActivate(const UsdStageRefPtr &stage,
                                        const SdfPath &rigPath,
                                        UsdTimeCode initialTime,
                                        std::vector<std::string> *errors)
{
    if (!stage) {
        if (errors) {
            errors->push_back("no stage to activate");
        }
        return false;
    }
    RigExecImagingRegistry &handle =
        RigExecImagingDirectory::Get().LegacyHandle();
    const Ptr target = ForStage(stage, /* create = */ true);
    if (!target) {
        if (errors) {
            errors->push_back("no imaging context for the stage");
        }
        return false;
    }
    // The legacy overlay selection is the old singleton's process-wide
    // viewer mode: a legacy activation carries it into its target, before
    // the first evaluation when the target is inactive (so the initial
    // generation already has it), through SetWeightOverlay otherwise.
    SdfPath overlay;
    bool overlaySet = false;
    {
        std::lock_guard<std::mutex> lock(handle._mutex);
        overlay = handle._weightOverlay;
        overlaySet = handle._legacyOverlaySet;
    }
    if (overlaySet && !target->IsActive()) {
        std::lock_guard<std::mutex> lock(target->_mutex);
        target->_weightOverlay = overlay;
    }
    if (!target->Activate(stage, rigPath, initialTime, errors)) {
        if (target->GetStore()->Get()) {
            RigExecImagingDirectory::Get().OfferCurrent(target);
        }
        return false;
    }
    RigExecImagingDirectory::Get().SetCurrent(target);
    if (overlaySet) {
        bool differs = false;
        {
            std::lock_guard<std::mutex> lock(target->_mutex);
            differs = target->_weightOverlay != overlay;
        }
        if (differs) {
            target->SetWeightOverlay(overlay.GetString());
        }
    }
    return true;
}

bool
RigExecImagingRegistry::_AdoptBridgeChainBindings(RigSession *session)
{
    uint64_t generation = 0;
    const RigExecChainSampleBindings *live =
        session->bridge->AcquireChainBindings(&generation);
    if (!live) {
        session->chainBindingsValid = false;
        return false;
    }
    if (!session->chainBindingsValid ||
        session->chainBindingsGeneration != generation) {
        session->chainBindings = *live;
        session->chainBindingsGeneration = generation;
    }
    session->chainBindingsValid = true;
    return true;
}

void
RigExecImagingRegistry::_RefreshFrozenSnapshot(RigSession *session)
{
    if (!session || session->playback || !session->bridge) {
        return;
    }
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    const uint64_t compiledRig = evaluator.GetCompiledRigSignature();
    const uint64_t serial = evaluator.GetStageEditSerial();
    // The pins: the bridge's, which its notice tracking keeps current (a
    // constant edited mid-compiled rig moves no digest, but its notice drops them).
    // Re-verifying a private copy after every edit would re-probe every chain
    // mover on every commit.
    const auto refreshPins = [&]() { _AdoptBridgeChainBindings(session); };
    if (session->frozenValid && session->frozenProgram == program &&
        session->frozenCompiledRig == compiledRig) {
        // Still serial, same program and compiled rig, valid pins: no notice
        // moved the bindings or the avar region since the standing
        // snapshot verified, so there is nothing to re-walk or
        // re-digest. (The burst pins, including overrides, compare
        // after the refresh regardless.) Failed pins still retry
        // every trigger -- only verified pins skip. During
        // notice-free playback this skips the refresh on every tick.
        if (session->frozenSerial == serial && session->chainBindingsValid) {
            return;
        }
        refreshPins();
        if (!program) {
            session->frozen.reset();
            session->frozenPatch.reset();
            session->frozenAvarDigest = 0;
            session->frozenError.clear();
            session->frozenSerial = serial;
            return;
        }
        // Same program, same compiled rig: what may have moved is the avar region
        // (ApplyAvarValueEdits), the inputs a routed value edit marked
        // (ApplyValueEdits), and the program stamp. Capture those values;
        // each worker materializes its copy under its lane lease. The last
        // two move no digest,
        // and the snapshot's history predates them either way, so they are
        // asked of the snapshot itself. A refused patch (a shape change
        // wearing a patch's clothes) falls through to a re-freeze.
        const uint64_t avarDigest = RigExecFrozenAvarRegionDigest(*program);
        if (avarDigest == session->frozenAvarDigest &&
            !(session->frozen &&
              RigExecFrozenSnapshotOwesLiveEdits(*session->frozen, *program))) {
            session->frozenSerial = serial;
            return;
        }
        if (session->frozen) {
            std::shared_ptr<const RigExecFrozenAvarPatch> patched;
            std::string ignored;
            if (RigExecCaptureFrozenAvarPatch(*session->frozen, *program,
                                              &patched, &ignored)) {
                session->frozenPatch = std::move(patched);
                session->frozenAvarDigest = avarDigest;
                session->frozenSerial = serial;
                return;
            }
        }
    }
    // A new program, a new compiled rig, or a patch that refused: re-freeze from
    // the live state. A refusal parks a validly empty entry -- production
    // jobs decline and the frame evaluates live when asked.
    session->frozen.reset();
    session->frozenPatch.reset();
    session->frozenProgram = program;
    session->frozenCompiledRig = compiledRig;
    session->frozenAvarDigest = 0;
    session->frozenValid = true;
    session->frozenSerial = serial;
    refreshPins();
    if (!program) {
        session->frozenError.clear();
        return;
    }
    session->frozenAvarDigest = RigExecFrozenAvarRegionDigest(*program);
    std::shared_ptr<const RigExecFrozenProgram> fresh;
    std::string frozenError;
    if (RigExecFreezeProgram(evaluator, &fresh, &frozenError)) {
        session->frozen = std::move(fresh);
        session->frozenError.clear();
    } else {
        session->frozenError = std::move(frozenError);
    }
}

RigExecBurstSampleCache *
RigExecImagingRegistry::_PrepareWarmBurst(RigSession *session)
{
    if (!session || session->playback || !session->bridge) {
        return nullptr;
    }
    RigExecProfileScope prepScope(session->bridge->MutableProfiler(),
                                  "Imaging.PrepareWarmBurst", "imaging");
    _RefreshFrozenSnapshot(session);
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        session->standingBurst.Clear();
        session->burstProgram = nullptr;
        session->burstCompiledRigSignature = 0;
        session->burstSerial = ~uint64_t(0);
        session->burstOverrides.clear();
        session->burstOverrun = false;
        session->burstDeclined = false;
        // Session-level verdict: no factory means no per-frame verdicts,
        // so prep names the cause instead of spinning silently.
        session->lastWarmSkip = RigExecWarmSkipReason::D7Exempt;
        session->lastWarmSkipDetail = "no baked program (D7)";
        return &session->standingBurst;
    }
    // No burst while an upstream source varies: a burst samples every frame
    // under one upstream list, and each job needs its own time's values, so
    // those frames take the plain per-frame route (no skip is recorded).
    session->burstUpstreamVaries =
        session->upstream && session->upstream->AnyVaries();
    if (session->burstUpstreamVaries) {
        session->standingBurst.Clear();
        session->burstProgram = nullptr;
        session->burstSerial = ~uint64_t(0);
        session->burstUpstreamSerial = ~uint64_t(0);
        session->burstOverrun = false;
        session->burstDeclined = false;
        return &session->standingBurst;
    }
    // The uniform values every frame of the burst samples under.
    std::vector<RigExecUpstreamValue> upstream;
    if (session->upstream) {
        session->upstream->ValuesAt(session->upstream->t0, &upstream);
    }
    const uint64_t compiledRigSignature =
        RigExecFrameCacheCompiledRigSignature(evaluator);
    const uint64_t serial = evaluator.GetStageEditSerial();
    const std::vector<RigExecValueOverride> overrides =
        session->bridge->GetInteractiveOverrides();
    // Pins current and the standing burst usable: serve it, with no
    // rebuild. The burst embeds its own pins copy, built from the bindings
    // generation it records, so a rebind moves it. Include the stage serial:
    // routed value edits can leave the compiled rig unchanged while invalidating
    // the burst's memoized static input values and attribute queries.
    const bool pinsCurrent =
        session->burstProgram == program &&
        session->burstCompiledRigSignature == compiledRigSignature &&
        session->burstSerial == serial &&
        session->burstOverrides == overrides &&
        session->burstUpstreamSerial == session->upstreamSerial &&
        session->burstUpstream == upstream &&
        session->burstChainGeneration == session->chainBindingsGeneration;
    if (pinsCurrent && session->standingBurst.usable) {
        return &session->standingBurst;
    }
    // The refresh above verified the pins, rebinding or invalidating
    // them; valid here means verified-current, and no notice can move
    // them before the burst's last frame samples. A burst is never
    // rebuilt onto unverified pins; the standing pins stay, so the next
    // trigger retries. The Clear parks neither latch flag, so the retry
    // is a rebuild, not a latched serve.
    if (!session->chainBindingsValid) {
        session->standingBurst.Clear();
        session->burstOverrun = false;
        session->burstDeclined = false;
        // Session-level verdict, as above (same words as the factory's).
        session->lastWarmSkip = RigExecWarmSkipReason::Unsampleable;
        session->lastWarmSkipDetail = "chain-bind";
        return &session->standingBurst;
    }
    // A pins-determined unusable outcome under the current pins:
    // re-serve it without paying for the doomed rebuild that produced
    // it. The latch sits past the transient check above, so a failed
    // bindings refresh never latches.
    if (pinsCurrent && (session->burstOverrun || session->burstDeclined)) {
        return &session->standingBurst;
    }
    RigExecBurstSampleCache rebuilt;
    std::string buildError;
    const uint64_t buildStartUs = RigExecProfiler::NowUs();
    const bool built = RigExecBuildBurstSampleCache(
        *program, session->chainBindings, overrides, upstream,
        compiledRigSignature, &rebuilt, &buildError);
    const double buildMs =
        double(RigExecProfiler::NowUs() - buildStartUs) / 1000.0;
    session->burstProgram = program;
    session->burstCompiledRigSignature = compiledRigSignature;
    session->burstSerial = serial;
    session->burstOverrides = overrides;
    session->burstUpstreamSerial = session->upstreamSerial;
    session->burstUpstream = std::move(upstream);
    session->burstChainGeneration = session->chainBindingsGeneration;
    if (buildMs > _warmBurstPrepSliceMs) {
        // Past the slice: park unusable and mark the overrun, so this
        // tick and every tick until the pins move take the plain
        // per-frame route instead of paying for another doomed build.
        session->standingBurst.Clear();
        session->burstOverrun = true;
        session->burstDeclined = false;
    } else {
        session->standingBurst = std::move(rebuilt);
        session->standingBurst.usable = built && session->standingBurst.usable;
        session->burstOverrun = false;
        session->burstDeclined = !session->standingBurst.usable;
        if (session->burstDeclined) {
            // Session-level verdict, as above; the latched serve below
            // re-records nothing (this verdict stands while it latches).
            session->lastWarmSkip = RigExecWarmSkipReason::Unsampleable;
            session->lastWarmSkipDetail = "burst-declined: " + buildError;
        }
    }
    session->bridge->MutableProfiler()->RecordInstant(
        "warmBurstRebuild", "imaging", RigExecProfiler::NowUs(),
        {{"usable", session->standingBurst.usable ? "1" : "0"},
         {"overrun", session->burstOverrun ? "1" : "0"},
         {"ms", std::to_string(buildMs)}});
    return &session->standingBurst;
}

bool
RigExecImagingRegistry::GetWarmBurstUsable(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmBurstUsable(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.standingBurst.usable;
        }
    }
    return false;
}

void
RigExecImagingRegistry::SetWarmBurstPrepSliceMs(double ms)
{
    if (const Ptr routed = _Routed()) {
        routed->SetWarmBurstPrepSliceMs(ms);
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _warmBurstPrepSliceMs = ms;
    // A new slice re-times every verdict: an overrun latched under the
    // old slice unlatches (the next trigger retries the build), while a
    // usable burst stands -- the slice guards rebuild cost, not standing
    // bursts -- and a shape decline, which no slice forgives, stays.
    for (RigSession &session : _sessions) {
        session.burstOverrun = false;
    }
}

double
RigExecImagingRegistry::GetWarmBurstPrepSliceMs()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmBurstPrepSliceMs();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmBurstPrepSliceMs;
}

RigExecWarmSkipReason
RigExecImagingRegistry::GetLastWarmSkipReason(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetLastWarmSkipReason(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.lastWarmSkip;
        }
    }
    return RigExecWarmSkipReason::None;
}

std::string
RigExecImagingRegistry::GetLastWarmSkipDetail(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetLastWarmSkipDetail(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.lastWarmSkipDetail;
        }
    }
    return std::string();
}

void
RigExecImagingRegistry::SetWarmSamplingBudget(size_t maxFactoryInvocations,
                                              double maxMs)
{
    if (const Ptr routed = _Routed()) {
        routed->SetWarmSamplingBudget(maxFactoryInvocations, maxMs);
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _warmSamplingMaxInvocations = maxFactoryInvocations;
    _warmSamplingMaxMs = maxMs;
}

size_t
RigExecImagingRegistry::GetWarmSamplingMaxInvocations()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmSamplingMaxInvocations();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmSamplingMaxInvocations;
}

double
RigExecImagingRegistry::GetWarmSamplingMaxMs()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmSamplingMaxMs();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _warmSamplingMaxMs;
}

bool
RigExecImagingRegistry::_EvaluateSessions(
    RigSessions *sessions, const UsdStageRefPtr &stage, UsdTimeCode time,
    std::shared_ptr<RigExecImagingSnapshot> *snapshot,
    RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr *compiledRig,
    std::vector<std::string> *errors)
{
    auto combined = std::make_shared<RigExecImagingSnapshot>();
    combined->stage = UsdStageWeakPtr(stage);
    combined->sampleTimeIsDefault = time.IsDefault();
    combined->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();

    auto combinedCompiledRig = std::make_shared<
        RigExecBindingResolvingSceneIndex::CompiledRigBinding>();
    uint64_t compiledRigId = 1469598103934665603ULL;
    SdfPath commonAssetRoot;
    bool firstRoot = true;

    for (RigSession &session : *sessions) {
        RigExecImagingSnapshotConstPtr rigSnapshot = session.store->Get();
        if (session.dirty || !rigSnapshot ||
            !rigSnapshot->Describes(stage, time)) {
            const RigExecImagingBridge::PublishResult result =
                session.playback
                    ? session.playback->EvaluateAndPublishResult(time)
                    : session.bridge->EvaluateAndPublishResult(time);
            // A cache hit is not an evaluator pull: the count is what the
            // scrub tests hold to zero over a warmed range.
            if (!result.cacheHit) {
                ++session.evaluationCount;
            }
            if (!result.ok) {
                if (errors) {
                    errors->push_back(
                        "initial/evaluated generation failed for rig " +
                        session.rigPath.GetString());
                }
                return false;
            }
            if (result.compiledRig)
                session.compiledRig = result.compiledRig;
            session.dirty = false;
            // Usually the last reference to the replaced generation.
            WorkMoveDestroyAsync(rigSnapshot);
            rigSnapshot = session.store->Get();
        }
        // No frozen-snapshot refresh here: only warming reads the snapshot,
        // so BuildWarmWork refreshes it lazily at warm-build time. Refreshing
        // eagerly on this per-frame path made every SetTime -- evaluated or
        // cache-hit, drag tick or scrub -- pay a chain-currency walk plus an
        // avar-region digest for state nobody on this path consumes.

        if (!rigSnapshot || !rigSnapshot->Describes(stage, time)) {
            if (errors) {
                errors->push_back(
                    "rig did not publish the requested stage/time: " +
                    session.rigPath.GetString());
            }
            return false;
        }

        // A zero digest is legal.  The bridge normally supplies the compiled rig on
        // its first publication; construct the same payload defensively so a
        // legal zero cannot leave a stage without a compiled rig.
        if (!session.compiledRig) {
            auto rigCompiledRig = std::make_shared<
                RigExecBindingResolvingSceneIndex::CompiledRigBinding>();
            rigCompiledRig->id =
                session.playback ? session.playback->GetCompiledRigSignature()
                                 : session.bridge->GetCompiledRigSignature();
            for (const auto &[path, published] : rigSnapshot->prims) {
                rigCompiledRig->publishedPrims.insert(path);
            }
            session.compiledRig = std::move(rigCompiledRig);
        }

        compiledRigId =
            _HashCompiledRigPart(compiledRigId, session.rigPath.GetString());
        const uint64_t rigCompiledRigId = session.compiledRig->id;
        compiledRigId = _HashCompiledRigPart(compiledRigId, &rigCompiledRigId,
                                             sizeof(rigCompiledRigId));
        combinedCompiledRig->publishedPrims.insert(
            session.compiledRig->publishedPrims.begin(),
            session.compiledRig->publishedPrims.end());
        combined->xformResetPaths.insert(rigSnapshot->xformResetPaths.begin(),
                                         rigSnapshot->xformResetPaths.end());

        // Carried across for the same reason xformResetPaths is: the
        // combined snapshot is BUILT here, field by field, so anything
        // not named is silently dropped. That is how this one was lost
        // the first time -- the bridge recorded every pose weight, the
        // store published them, and the merge quietly left them behind,
        // so RigExecImaging_GetMovedFloats found 0 of 121 with no error
        // anywhere. Two rigs cannot publish the same property path (the
        // prim check below already rejects a shared prim), so a plain
        // insert is right.
        combined->movedFloats.insert(rigSnapshot->movedFloats.begin(),
                                     rigSnapshot->movedFloats.end());
        combined->movedArrays.insert(rigSnapshot->movedArrays.begin(),
                                     rigSnapshot->movedArrays.end());

        if (firstRoot) {
            commonAssetRoot = session.assetRoot;
            firstRoot = false;
        } else if (commonAssetRoot != session.assetRoot) {
            commonAssetRoot = SdfPath();
        }

        // In path order, so a lone rig's prims each land at the end.
        for (const auto &[path, published] : rigSnapshot->prims) {
            const size_t size = combined->prims.size();
            combined->prims.emplace_hint(combined->prims.end(), path,
                                         published);
            if (combined->prims.size() == size) {
                if (errors) {
                    errors->push_back("two rigs publish the same Hydra prim " +
                                      path.GetString() +
                                      "; cross-rig output overlap is "
                                      "not allowed");
                }
                return false;
            }
        }
    }

    // Reserve zero for the registry's no-published-compiled rig state.
    combinedCompiledRig->id = compiledRigId ? compiledRigId : 1;
    combined->assetRoot = commonAssetRoot;
    *snapshot = std::move(combined);
    *compiledRig = std::move(combinedCompiledRig);
    return true;
}

RigExecImagingBridge::PublishResult
RigExecImagingRegistry::_Publish(
    std::shared_ptr<RigExecImagingSnapshot> snapshot,
    const RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr
        &compiledRig)
{
    RigExecImagingBridge::PublishResult result;
    snapshot->generation = ++_generation;
    // Frames on demand: offset reads of this generation's outputs go here.
    snapshot->frameTable = _frameTable;
    // The generation this replaces is freed on a worker: a thousand
    // published prims take longer to destroy than to diff.
    RigExecImagingSnapshotConstPtr replaced = _store->Get();
    result.dirtied = _store->Publish(std::move(snapshot));
    WorkMoveDestroyAsync(replaced);
    if (compiledRig && compiledRig->id != _publishedCompiledRigId) {
        result.compiledRig = compiledRig;
        _publishedCompiledRigId = compiledRig->id;
        _publishedCompiledRig = compiledRig;
    }
    result.ok = true;
    return result;
}

bool
RigExecImagingRegistry::Activate(const UsdStageRefPtr &stage,
                                 const SdfPath &rigPath,
                                 UsdTimeCode initialTime,
                                 std::vector<std::string> *errors)
{
    return _Activate(stage, rigPath, initialTime, errors,
                     /* automatic = */ false);
}

std::shared_ptr<RigExecImagingSnapshot>
RigExecImagingRegistry::_BuildAuthoredGuides(
    const UsdStageRefPtr &stage, const std::vector<SdfPath> &rigPaths,
    UsdTimeCode time)
{
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->stage = UsdStageWeakPtr(stage);
    snapshot->sampleTimeIsDefault = time.IsDefault();
    snapshot->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();
    // Live sessions own the display state: a session without a bridge (a
    // playback) draws through a scratch builder that is not retained, so
    // _authoredGuides stays the guide-only state alone.
    const bool guideOnly = _sessions.empty();
    std::set<SdfPath> retained;
    std::unique_ptr<RigExecImagingBridge> scratch;
    SdfPath commonAssetRoot;
    bool firstRoot = true;
    for (const SdfPath &path : rigPaths) {
        const UsdPrim root = stage->GetPrimAtPath(path);
        if (!root || root.GetTypeName() != _tRigRootType)
            continue;
        RigExecImagingBridge *builder = nullptr;
        for (RigSession &session : _sessions) {
            if (session.rigPath == path && session.bridge) {
                builder = session.bridge.get();
                break;
            }
        }
        if (!builder && guideOnly) {
            auto &bridge = _authoredGuides[path];
            if (!bridge) {
                bridge = std::make_unique<RigExecImagingBridge>(stage, path);
            }
            builder = bridge.get();
            retained.insert(path);
        } else if (!builder) {
            scratch = std::make_unique<RigExecImagingBridge>(stage, path);
            builder = scratch.get();
        }
        builder->AppendAuthoredGuides(time, snapshot.get());
        if (firstRoot) {
            commonAssetRoot = path.GetParentPath();
            firstRoot = false;
        } else if (commonAssetRoot != path.GetParentPath()) {
            commonAssetRoot = SdfPath();
        }
    }
    if (guideOnly) {
        for (auto it = _authoredGuides.begin(); it != _authoredGuides.end();) {
            it = retained.count(it->first) ? std::next(it)
                                           : _authoredGuides.erase(it);
        }
        _authoredGuidesDirty = false;
    }
    snapshot->assetRoot = commonAssetRoot;
    return snapshot;
}

bool
RigExecImagingRegistry::_Activate(const UsdStageRefPtr &stage,
                                  const SdfPath &rigPath,
                                  UsdTimeCode initialTime,
                                  std::vector<std::string> *errors,
                                  bool automatic)
{
    if (_legacyHandle) {
        return _LegacyActivate(stage, rigPath, initialTime, errors);
    }
    if (stage && UsdStageWeakPtr(stage) != _boundStage) {
        // A context only ever evaluates its own stage. Another stage is
        // forwarded to ITS context (created when it has none), which is
        // exactly what the old singleton's "activate replaces" becomes when
        // every stage has its own registry: nothing here is replaced.
        const Ptr target = ForStage(stage, /* create = */ true);
        if (!target || target.get() == this) {
            if (errors) {
                errors->push_back("no imaging context for the stage");
            }
            return false;
        }
        return target->_Activate(stage, rigPath, initialTime, errors,
                                 automatic);
    }
    // unique_lock, not lock_guard: the broadcast at the end runs WITHOUT the
    // lock (see _Broadcast), because scene indices call back into the
    // registry -- the results index's time trigger evaluates through
    // IsActiveRigRoot/SetTime -- and broadcasting while holding it is a
    // self-deadlock. Same rule SetWeightOverlay documents, applied here.
    std::unique_lock<std::mutex> lock(_mutex);
    if (!stage) {
        if (errors) {
            errors->push_back("no stage to activate");
        }
        return false;
    }
    if (automatic) {
        // EnsureActivated tested these unlocked. Two rigs populating at
        // once both get here; the first commits and the second is a no-op
        // instead of a recompile -- and it never downgrades an explicit
        // activation that landed in between to an automatic one. A host
        // deactivation in between still wins.
        if (!_sessions.empty()) {
            return true;
        }
        if (_deactivatedByHost) {
            return false;
        }
    }

    std::vector<SdfPath> rigPaths;
    if (!rigPath.IsEmpty()) {
        rigPaths.push_back(rigPath);
    } else {
        for (const UsdPrim &prim : stage->Traverse()) {
            if (prim.GetTypeName() == "RigExecRoot") {
                rigPaths.push_back(prim.GetPath());
            }
        }
    }
    std::sort(rigPaths.begin(), rigPaths.end());
    rigPaths.erase(std::unique(rigPaths.begin(), rigPaths.end()),
                   rigPaths.end());
    // A (re-)activation never inherits another stage's warming: the records
    // name the old stage's keys. Reset BEFORE the candidates evaluate, so
    // the initial evaluation's own memos survive; a failed activation below
    // leaves uncached records, which re-warm on demand.
    for (const SdfPath &path : rigPaths) {
        _warmIndex->ResetRig(path);
    }
    if (rigPaths.empty()) {
        if (errors) {
            errors->push_back("no RigExecRoot prim found");
        }
        return false;
    }

    // Edit-driven re-evaluation: listen on the source stage so property
    // edits under any active character asset republish at the current time.
    // REGISTERED FIRST, BEFORE Compile(), AND THAT ORDER IS LOAD-BEARING.
    // Tf_NoticeRegistry::_Register PREPENDS its deliverer, so listeners are
    // delivered most-recently-registered FIRST. OpenExec's own
    // ExecUsdSystem::_NoticeListener -- the thing that invalidates every
    // cached computed value when an attribute is authored -- is created by
    // the ExecUsdSystem constructor, which happens inside Compile(). So
    // registering after Compile() would put us AHEAD of exec in delivery
    // order, and _OnObjectsChanged would re-evaluate while exec still held
    // the pre-edit value: every generation published from an edit would be
    // one edit stale, and a second Evaluate in the same dispatch equally so.
    // Registering first puts us at the BACK of the list, which is where a
    // re-evaluating listener belongs: every recompile builds a new
    // ExecUsdSystem that prepends ahead of us again, so this holds for the
    // life of the rig rather than only until the first structural edit.
    // Registering this early also arms _OnObjectsChanged across the whole
    // compile-and-evaluate window, so _assetRoots -- what arms it -- stays
    // empty until the commit below, and a failure retains the previously
    // active stage's listener. Nothing in that window may author to \p stage
    // regardless: _OnObjectsChanged takes the same non-recursive _mutex this
    // function holds. Neither Compile() nor evaluation authors anything by
    // construction (that is what testRigExecNoAuthoring asserts), which is
    // what makes this safe.
    const std::set<SdfPath> previousAssetRoots = _assetRoots;
    _assetRoots.clear();
    TfNotice::Key candidateChangeKey =
        TfNotice::Register(TfCreateWeakPtr(this),
                           &RigExecImagingRegistry::_OnObjectsChanged, stage);

    // Activation is transactional: a rig that fails to compile or evaluate
    // leaves the previously published generation, and the listener feeding
    // it, exactly as they were rather than tearing the viewport down.
    // Keep the previous key registered until commit: re-registering it on
    // rollback would put it ahead of the retained ExecUsdSystem listener,
    // causing the next edit to publish before exec invalidates its cache.
    RigSessions candidate;
    candidate.reserve(rigPaths.size());
    const auto abandon = [&](bool evaluationFailed) {
        const bool hasRoot = std::any_of(
            rigPaths.begin(), rigPaths.end(), [&stage](const SdfPath &path) {
                const UsdPrim prim = stage->GetPrimAtPath(path);
                return prim && prim.GetTypeName() == _tRigRootType;
            });
        if (!_sessions.empty() || !hasRoot) {
            TfNotice::Revoke(candidateChangeKey);
            _assetRoots = previousAssetRoots;
            return;
        }
        // An incomplete rig still has editable authored guides. Evaluation
        // remains failed; only display data enters the snapshot store.
        TfNotice::Revoke(_changeKey);
        _changeKey = candidateChangeKey;
        _stage = stage;
        _lastTime = initialTime;
        _autoActivated = automatic;
        _deactivatedByHost = false;
        _authoredGuidesRequest = rigPath;
        _authoredGuidesRetryOnValues = evaluationFailed;
        for (RigSession &session : candidate) {
            if (session.bridge) {
                _authoredGuides[session.rigPath] = std::move(session.bridge);
            }
        }
        auto snapshot = _BuildAuthoredGuides(stage, rigPaths, initialTime);
        // Same scope and noted-root state a commit leaves: the guide
        // builders own their generated scopes, and the roots this attempt
        // covered are no longer pending eager activation.
        {
            std::lock_guard<std::mutex> notedLock(_notedMutex);
            _notedRigRoots.clear();
        }
        _generatedScopes.clear();
        for (const auto &[path, bridge] : _authoredGuides) {
            _generatedScopes.insert(bridge->GetGeneratedScope());
        }
        for (const Chain &chain : _BoundChains()) {
            if (chain.pruning) {
                chain.pruning->SetOwnedScopes(_generatedScopes);
            }
        }
        const auto published = _Publish(std::move(snapshot), nullptr);
        lock.unlock();
        _Broadcast(published);
    };

    // Prepare a complete replacement without touching the active stage,
    // scene-index scopes, or published store.
    for (const SdfPath &path : rigPaths) {
        const UsdPrim rig = stage->GetPrimAtPath(path);
        if (!rig || rig.GetTypeName() != "RigExecRoot") {
            if (errors) {
                errors->push_back("activation path is not a RigExecRoot: " +
                                  path.GetString());
            }
            abandon(false);
            return false;
        }
        RigSession session;
        session.rigPath = path;
        session.assetRoot = path.GetParentPath();
        session.store = std::make_shared<RigExecSnapshotStore>();
        // Baked playback (M2b): a rig naming rigExec:asset plays the
        // file instead of evaluating. An unreadable or unplayable
        // file falls back to live evaluation with a warning -- the
        // schema contract -- rather than failing the activation.
        std::string assetPath;
        if (RigExecPlaybackAssetFor(rig, &assetPath)) {
            auto playback = std::make_unique<RigExecBakedPlayback>(
                stage, path, session.store);
            std::string why;
            if (playback->Open(assetPath, &why)) {
                playback->SetWeightOverlay(_weightOverlay);
                session.playback = std::move(playback);
                session.readRoots = {session.assetRoot};
                session.readRootsDirty = false;
            } else {
                TF_WARN("rigExec: %s names rigExec:asset %s (%s); "
                        "evaluating live",
                        path.GetString().c_str(), assetPath.c_str(),
                        why.c_str());
            }
        }
        if (!session.playback) {
            session.bridge = std::make_unique<RigExecImagingBridge>(
                stage, path, session.store);
            // The combined generation is diffed in _Publish; the
            // session's own changed-leaf sets would go unread.
            session.bridge->SetPublishesDirtied(false);
            session.bridge->SetWarmFrameIndex(_warmIndex);
            if (std::shared_ptr<RigExecFrameCache> cache =
                    session.bridge->GetFrameCache()) {
                // Evictions retire the index's completions: the strip never
                // shows cached for a drained entry. The callback holds the
                // index shared and the rig by value, so a drop for a dead
                // session lands in a live index under a dead rig.
                std::shared_ptr<RigExecWarmFrameIndex> index = _warmIndex;
                const SdfPath rigPath = path;
                cache->SetEvictionCallback(
                    [index, rigPath](
                        const std::vector<RigExecFrameCacheEviction> &evicted) {
                        for (const RigExecFrameCacheEviction &entry : evicted) {
                            if (entry.time.IsNumeric()) {
                                index->NoteEvicted(rigPath, entry.key,
                                                   entry.time.GetValue());
                            }
                        }
                    });
            }
            // A selection made before this rig was activated applies to it: the
            // overlay is a viewer mode, not a property of one bridge. Set before
            // the first evaluation so the initial generation already carries it.
            session.bridge->SetWeightOverlay(_weightOverlay);
            if (!session.bridge->Compile(errors)) {
                if (_sessions.empty()) {
                    _authoredGuides[path] = std::move(session.bridge);
                }
                abandon(false);
                return false;
            }
        }
        candidate.push_back(std::move(session));
    }

    // Upstream values already pulled for this stage stand from the first
    // generation (read roots are not known yet: the asset root decides,
    // and the commit below widens the share).
    if (!_upstreamTables.empty()) {
        const auto found = _upstreamTables.find(_upstreamChain);
        for (RigSession &session : candidate) {
            if (!session.bridge && !session.playback) {
                continue;
            }
            session.upstream = _UpstreamShare(
                found != _upstreamTables.end() ? found->second : nullptr,
                {session.assetRoot});
            _HandOffUpstreamLocked(&session, initialTime);
        }
    }
    std::shared_ptr<RigExecImagingSnapshot> initialSnapshot;
    RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr
        initialCompiledRig;
    if (!_EvaluateSessions(&candidate, stage, initialTime, &initialSnapshot,
                           &initialCompiledRig, errors)) {
        abandon(true);
        return false;
    }

    // Commit the fully compiled and evaluated stage in one transaction.
    // Repopulating _assetRoots is what arms _OnObjectsChanged.
    TfNotice::Revoke(_changeKey);
    _changeKey = candidateChangeKey;
    _sessions = std::move(candidate);
    _authoredGuides.clear();
    _authoredGuidesRequest = SdfPath();
    _authoredGuidesRetryOnValues = false;
    _active = !_sessions.empty();
    _deactivatedByHost = false;
    // Who owns the activation's lifetime: the host (explicit), or the
    // library, which releases it with its last bound chain (ReleaseChain).
    _autoActivated = automatic;
    // The warming pool starts with the first activation (a context without
    // an active rig owns no threads) and stops with Deactivate. It serves
    // this context only, so its per-rig-path keys never meet another
    // stage's equal paths.
    if (!_scheduler) {
        _scheduler = _MakeScheduler(_warmIndex);
    }
    // Fresh sessions retire every queued or running job for their rigs:
    // a completion holds its sampled cache, so this drops work, never
    // results, and a re-activation never inherits another stage's warming.
    for (const RigSession &session : _sessions) {
        // Fresh sessions retire in-flight jobs for their rigs -- but only
        // when any are in flight: an idle bump retires nothing and would
        // orphan the initial evaluation's memos, which stamped the
        // pre-commit generation.
        if (_scheduler->InFlightCount(session.rigPath) > 0) {
            _CancelGenerationLocked(session.rigPath);
        }
    }
    _stage = stage;
    {
        // Nested _mutex -> _notedMutex: the documented lock order.
        std::lock_guard<std::mutex> notedLock(_notedMutex);
        _notedRigRoots.clear();
    }
    _generatedScopes.clear();
    _assetRoots.clear();
    for (const RigSession &session : _sessions) {
        _generatedScopes.insert(session.playback
                                    ? session.playback->GetGeneratedScope()
                                    : session.bridge->GetGeneratedScope());
        _assetRoots.insert(session.assetRoot);
    }
    _RefreshReadRoots();
    for (const Chain &chain : _BoundChains()) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes(_generatedScopes);
        }
    }
    _lastTime = initialTime;
    RigExecImagingBridge::PublishResult published =
        _Publish(std::move(initialSnapshot), initialCompiledRig);
    // Unlocked: the commit above is complete, and _Broadcast sends scene
    // index notices whose observers call back into this registry.
    lock.unlock();
    _Broadcast(published);
    return true;
}

bool
RigExecImagingRegistry::EnsureActivated(const UsdStageRefPtr &stage,
                                        UsdTimeCode time)
{
    if (!stage) {
        return false;
    }
    if (_legacyHandle || UsdStageWeakPtr(stage) != _boundStage) {
        // The stage's own context decides; there is no other stage to
        // refuse on account of.
        const Ptr target = ForStage(stage, /* create = */ true);
        return target && target.get() != this &&
               target->EnsureActivated(stage, time);
    }
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_sessions.empty()) {
            // Already active on this (its only) stage: a no-op.
            return true;
        }
        if (!_authoredGuides.empty())
            return false;
        if (_deactivatedByHost) {
            // The host turned this stage off; only its explicit Activate
            // turns it back on (see the header).
            return false;
        }
    }
    // Outside the lock: Activate takes the same non-recursive mutex (the
    // rule SetWeightOverlay documents). Two rigs populating at once can both
    // reach here; the activations serialize inside _Activate, which
    // re-checks both tests above under the lock, so the second is a no-op.
    // Automatic: the library releases this activation with the last chain
    // bound to this context (ReleaseChain).
    std::vector<std::string> errors;
    if (!_Activate(stage, SdfPath::EmptyPath(), time, &errors,
                   /* automatic = */ true)) {
        if (_store->Get()) {
            RigExecImagingDirectory::Get().OfferCurrent(ForKey(_key));
        }
        // One line, not silent: a broken rig in a batch host (usdrecord)
        // otherwise renders its rest pose with no explanation. During live
        // authoring this fires at most once per root definition -- defining
        // the outputs that complete the rig does not resync the root, so the
        // adapter is not asked again; the host's explicit activation (or the
        // next resync of the root itself) retries.
        TF_WARN("RigExec: automatic activation failed for %s: %s",
                stage ? stage->GetRootLayer()->GetIdentifier().c_str()
                      : "<no stage>",
                errors.empty() ? "unknown error" : errors.front().c_str());
        return false;
    }
    // A host with no explicit activation (usdrecord, a test driving the
    // real chain) reaches the legacy surface through this context -- but
    // only when no current context is live, so the automatic path never
    // takes the legacy surface away from an explicit activation.
    RigExecImagingDirectory::Get().OfferCurrent(ForKey(_key));
    return true;
}

bool
RigExecImagingRegistry::IsActiveRigRoot(const SdfPath &path)
{
    if (const Ptr routed = _Routed()) {
        return routed->IsActiveRigRoot(path);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == path) {
            return true;
        }
    }
    return false;
}

bool
RigExecImagingRegistry::IsDisplayedRigRoot(const SdfPath &path)
{
    if (const Ptr routed = _Routed()) {
        return routed->IsDisplayedRigRoot(path);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (_authoredGuides.count(path))
        return true;
    for (const RigSession &session : _sessions) {
        if (session.rigPath == path)
            return true;
    }
    return false;
}

void
RigExecImagingRegistry::NoteRigRoot(const SdfPath &rigPath)
{
    if (const Ptr routed = _Routed()) {
        routed->NoteRigRoot(rigPath);
        return;
    }
    if (rigPath.IsEmpty()) {
        return;
    }
    // _notedMutex, never _mutex: the adapter calls this from inside scene
    // index traversals that run while _mutex is held.
    std::lock_guard<std::mutex> lock(_notedMutex);
    _notedRigRoots.insert(rigPath);
}

bool
RigExecImagingRegistry::IsNotedRigRoot(const SdfPath &path)
{
    if (const Ptr routed = _Routed()) {
        return routed->IsNotedRigRoot(path);
    }
    std::lock_guard<std::mutex> lock(_notedMutex);
    return _notedRigRoots.count(path) != 0;
}

namespace {

// The pull window under the context's lock: the union of the sessions'
// warm ranges, else the stage's time codes.
RigExecUpstreamPullWindow
_PullWindow(const std::vector<std::pair<double, double>> &ranges,
            const UsdStageRefPtr &stage, UsdTimeCode lastTime)
{
    RigExecUpstreamPullWindow window;
    bool ranged = false;
    for (const auto &[lo, hi] : ranges) {
        window.lo = ranged ? std::min(window.lo, lo) : lo;
        window.hi = ranged ? std::max(window.hi, hi) : hi;
        ranged = true;
    }
    if (stage) {
        if (!ranged) {
            window.lo = stage->GetStartTimeCode();
            window.hi = std::max(window.lo, stage->GetEndTimeCode());
        }
        window.held = stage->GetInterpolationType() == UsdInterpolationTypeHeld;
    }
    if (lastTime.IsNumeric() && std::isfinite(lastTime.GetValue())) {
        window.haveTime = true;
        window.time = lastTime.GetValue();
    }
    return window;
}

} // namespace

RigExecUpstreamPullWindow
RigExecImagingRegistry::GetUpstreamPullWindow()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetUpstreamPullWindow();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<std::pair<double, double>> ranges;
    for (const RigSession &session : _sessions) {
        if (session.warmRangeActive && !session.warmRange.empty()) {
            ranges.emplace_back(session.warmRange.front(),
                                session.warmRange.back());
        }
    }
    return _PullWindow(ranges, _PreviewStageLocked(), _lastTime);
}

void
RigExecImagingRegistry::SetUpstreamTable(
    const RigExecResultsSceneIndex *chain,
    std::shared_ptr<const RigExecUpstreamTable> table)
{
    if (const Ptr routed = _Routed()) {
        routed->SetUpstreamTable(chain, std::move(table));
        return;
    }
    if (!chain) {
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (table && !table->entries.empty()) {
        // The chain that delivered the table supplies the values; a key
        // another chain publishes differently is reported.
        for (const auto &[other, otherTable] : _upstreamTables) {
            if (other == chain || !otherTable) {
                continue;
            }
            for (const auto &[path, entry] : table->entries) {
                const auto found = otherTable->entries.find(path);
                // Tables can have different anchors. Compare the values at
                // this trigger, using stored samples only; an older table
                // outside its window cannot establish a conflict.
                RigExecUpstreamValue incoming, previous;
                if (found != otherTable->entries.end() &&
                    table->ValueAt(path, entry, table->t0, &incoming) &&
                    otherTable->ValueAt(path, found->second, table->t0,
                                        &previous) &&
                    !RigExecSameSourceValue(incoming.value, true,
                                            previous.value, true)) {
                    TF_WARN("upstream input %s: two scene-index chains "
                            "publish different values; the triggering "
                            "chain's are used",
                            path.GetText());
                }
            }
        }
        _upstreamTables[chain] = std::move(table);
    } else {
        _upstreamTables.erase(chain);
    }
    _upstreamChain = chain;
    for (RigSession &session : _sessions) {
        _UpdateSessionUpstreamLocked(&session);
    }
}

void
RigExecImagingRegistry::DropUpstreamTable(const RigExecResultsSceneIndex *chain)
{
    if (const Ptr routed = _Routed()) {
        routed->DropUpstreamTable(chain);
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    const bool held = _upstreamTables.erase(chain) != 0;
    if (_upstreamChain == chain) {
        _upstreamChain = nullptr;
    }
    if (held) {
        for (RigSession &session : _sessions) {
            _UpdateSessionUpstreamLocked(&session);
        }
    }
}

bool
RigExecImagingRegistry::RefreshUpstreamInputs()
{
    if (const Ptr routed = _Routed()) {
        return routed->RefreshUpstreamInputs();
    }
    UsdTimeCode time;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_stage) {
            return false;
        }
        time = _lastTime;
    }
    // SetTime hands every session its values at that time and evaluates
    // only the sessions they moved.
    return SetTime(time);
}

std::shared_ptr<const RigExecUpstreamTable>
RigExecImagingRegistry::GetUpstreamTable(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetUpstreamTable(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.upstream;
        }
    }
    return nullptr;
}

uint64_t
RigExecImagingRegistry::GetUpstreamSerial(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetUpstreamSerial(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return session.upstreamSerial;
        }
    }
    return 0;
}

void
RigExecImagingRegistry::_UpdateSessionUpstreamLocked(RigSession *session)
{
    if (!session || (!session->bridge && !session->playback)) {
        return;
    }
    const auto found = _upstreamTables.find(_upstreamChain);
    std::set<SdfPath> roots{session->assetRoot};
    if (!session->readRootsDirty) {
        roots.insert(session->readRoots.begin(), session->readRoots.end());
    }
    std::shared_ptr<const RigExecUpstreamTable> share = _UpstreamShare(
        found != _upstreamTables.end() ? found->second : nullptr, roots);
    const bool same = share == session->upstream ||
                      (share && session->upstream &&
                       RigExecSameUpstreamSources(*share, *session->upstream));
    session->upstream = std::move(share);
    if (!same) {
        // The sources, the window or a frame value moved: rows warmed under
        // the old ones must not serve (FindCachedKey compares no digest). A
        // playback rig never warms.
        ++session->upstreamSerial;
        if (session->bridge) {
            _CancelGenerationLocked(session->rigPath);
        }
    }
}

void
RigExecImagingRegistry::_HandOffUpstreamLocked(RigSession *session,
                                               UsdTimeCode time)
{
    if (!session || (!session->bridge && !session->playback)) {
        return;
    }
    std::vector<RigExecUpstreamValue> values;
    if (session->upstream) {
        if (time.IsNumeric()) {
            session->upstream->ValuesAt(time.GetValue(), &values);
        } else {
            // At Default only uniform sources have a value.
            for (const auto &[path, entry] : session->upstream->entries) {
                if (!entry.varies) {
                    values.push_back(
                        RigExecUpstreamValue{path, entry.atT0, entry.hashAtT0});
                }
            }
        }
    }
    // A playback session applies them to the binary's inputs at its next
    // generation (RigExecBakedPlayback::SetUpstreamInputs).
    if (session->playback) {
        if (values != session->playback->GetUpstreamInputs()) {
            session->playback->SetUpstreamInputs(std::move(values));
            session->dirty = true;
        }
    } else if (values != session->bridge->GetUpstreamInputs()) {
        session->bridge->SetUpstreamInputs(std::move(values));
        session->dirty = true;
    }
}

bool
RigExecImagingRegistry::SetTime(UsdTimeCode time)
{
    if (const Ptr routed = _Routed()) {
        return routed->SetTime(time);
    }
    // unique_lock: both broadcasts below run unlocked (see Activate), because
    // the notices they send re-enter this registry through the results
    // index's time trigger.
    std::unique_lock<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_stage) {
        if (_stage && !_authoredGuides.empty()) {
            const auto current = _store->Get();
            if (!_authoredGuidesDirty && current &&
                current->Describes(_stage, time)) {
                return false;
            }
            std::vector<SdfPath> roots;
            for (const auto &entry : _authoredGuides)
                roots.push_back(entry.first);
            auto snapshot = _BuildAuthoredGuides(_stage, roots, time);
            _lastTime = time;
            const auto published = _Publish(std::move(snapshot), nullptr);
            if (_authoredGuides.empty()) {
                TfNotice::Revoke(_changeKey);
                _changeKey = TfNotice::Key();
                _stage.Reset();
                _autoActivated = false;
            }
            lock.unlock();
            _Broadcast(published);
            return false;
        }
        // A plain stage has no evaluator, but its xform previews still
        // compose against the host's displayed frame.
        _lastTime = time;
        return false;
    }
    // Live hand-off: each session's upstream values at this time. A moved
    // value dirties the session; a time move alone cancels no warming (the
    // warm rows fold each frame's own values).
    for (RigSession &session : _sessions) {
        _HandOffUpstreamLocked(&session, time);
    }
    // A publish still dirties every prim in the generation through
    // _Broadcast, even when it republishes content nothing changed -- so a
    // redundant SetTime cost a full Hydra redraw for nothing. In usdview the
    // generation was observed advancing by 2 per frame change while the
    // plugin's Python _OnFrameChanged fired once, which means a second
    // SetTime arrives from somewhere else (UpdatePreview/EndPreview,
    // SetWeightOverlay and _OnObjectsChanged all republish at _lastTime).
    // Rather than chase which, make the redundant one free: nothing dirty
    // plus the same stage/time already published means there is nothing to
    // say. Headless, generation advanced by exactly 1 per SetTime, so this
    // guard is what closes the gap the usdview session showed.
    if (!_readRootsDirty && _publishedCompiledRigId != 0) {
        bool anyDirty = false;
        for (const RigSession &session : _sessions) {
            anyDirty = anyDirty || session.dirty || session.readRootsDirty;
        }
        if (!anyDirty) {
            const RigExecImagingSnapshotConstPtr current = _store->Get();
            if (current && current->Describes(UsdStageWeakPtr(_stage), time)) {
                _lastTime = time;
                return true;
            }
        }
    }
    for (RigSession &session : _sessions) {
        session.settleOverrides.clear();
    }
    if (_readRootsDirty) {
        _RefreshReadRoots();
        // Wider read roots may widen a session's upstream share.
        for (RigSession &session : _sessions) {
            _HandOffUpstreamLocked(&session, time);
        }
    }
    std::shared_ptr<RigExecImagingSnapshot> snapshot;
    RigExecBindingResolvingSceneIndex::CompiledRigBindingConstPtr compiledRig;
    if (!_EvaluateSessions(&_sessions, _stage, time, &snapshot, &compiledRig,
                           nullptr)) {
        // A rig that can no longer evaluate stops driving the scene.
        // The per-session bridge already cleared its own store on the way
        // out, but this COMBINED generation is the one the scene indices
        // read, and returning here left the last good one current: remove a
        // mover's rigExec:moves and the recompile fails, so the mesh under
        // the driven Xform stayed where the constraint had put it through
        // every later edit and frame change.
        // Replace driven results with authored guides and notify observers.
        _lastTime = time;
        RigExecImagingBridge::PublishResult cleared;
        cleared.ok = true;
        std::vector<SdfPath> roots;
        for (const RigSession &session : _sessions)
            roots.push_back(session.rigPath);
        auto guides = _BuildAuthoredGuides(_stage, roots, time);
        guides->generation = ++_generation;
        RigExecImagingSnapshotConstPtr replaced = _store->Get();
        cleared.dirtied = _store->Publish(std::move(guides));
        WorkMoveDestroyAsync(replaced);
        // The next good generation must re-announce its compiled rig; the one the
        // binding index holds names prims this clear just removed.
        _publishedCompiledRigId = 0;
        _publishedCompiledRig.reset();
        lock.unlock();
        _Broadcast(cleared);
        return false;
    }
    const UsdTimeCode previousTime = _lastTime;
    _lastTime = time;
    // During active playback the sweep jobs the playhead already passed are
    // dead weight: the live path serves those frames now. Neighbors stay --
    // the frames ahead still warm the imminent scrub path. Only a FORWARD
    // step sheds: a backward scrub is heading into exactly the frames at or
    // behind the new playhead, and purging those would strip the warming
    // the scrub is about to use.
    const bool advanced = previousTime.IsNumeric() && time.IsNumeric() &&
                          time.GetValue() > previousTime.GetValue();
    if (advanced && _scheduler) {
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                _scheduler->NotePlaybackAdvanced(session.rigPath, time);
            }
        }
    }
    // Recorded into the first rig's profiler, so one summary holds the whole
    // update: the rig (Imaging.EvaluateAndPublish, per session, inside
    // _EvaluateSessions above) and then the COMBINED generation's publish
    // and the dirty notices that drive Hydra's sync.
    RigExecProfiler *profiler = nullptr;
    if (!_sessions.empty() && !_sessions.front().playback) {
        profiler = _sessions.front().bridge->MutableProfiler();
    }
    RigExecImagingBridge::PublishResult published;
    {
        RigExecProfileScope scope(profiler, "Imaging.CombinedPublish",
                                  "imaging");
        published = _Publish(std::move(snapshot), compiledRig);
    }
    // Frames on demand: an edit, a recompile or moved upstream values make
    // every other frame stale; readers holding them are told below.
    const bool framesReset =
        _frameTable->ResetIfStale(_FrameTableStampLocked());
    lock.unlock();
    {
        RigExecProfileScope scope(profiler, "Imaging.Broadcast", "imaging");
        _Broadcast(published);
    }
    if (framesReset) {
        _BroadcastFramesChanged();
    }
    return true;
}

void
RigExecImagingRegistry::_RefreshReadRoots()
{
    _readRoots.clear();
    for (RigSession &session : _sessions) {
        if (session.readRootsDirty) {
            session.readRoots = _CollectReadRoots(_stage, {session.assetRoot});
            session.readRootsDirty = false;
        }
        _readRoots.insert(session.readRoots.begin(), session.readRoots.end());
    }
    _readRootsDirty = false;
    if (!_upstreamTables.empty()) {
        for (RigSession &session : _sessions) {
            _UpdateSessionUpstreamLocked(&session);
        }
    }
}

size_t
RigExecImagingRegistry::GetSessionEvaluationCount(const SdfPath &rigPath)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetSessionEvaluationCount(rigPath);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rigPath)
            return session.evaluationCount;
    }
    return 0;
}

void
RigExecImagingRegistry::RefreshFrameCacheSettings()
{
    if (const Ptr routed = _Routed()) {
        routed->RefreshFrameCacheSettings();
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.bridge) {
            session.bridge->RefreshFrameCacheSettings();
        }
    }
}

RigExecWarmFactoryResult
RigExecImagingRegistry::BuildWarmWork(const SdfPath &rig, UsdTimeCode time,
                                      RigExecFrameGeneration generation,
                                      RigExecFrozenStepRunner runner,
                                      RigExecBurstSampleCache *burst)
{
    if (const Ptr routed = _Routed()) {
        return routed->BuildWarmWork(rig, time, generation, runner, burst);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    // Production-ness is read BEFORE the runner resolves: a null runner
    // means "warm for real", and real execution needs the session's
    // snapshot, while an injected test runner brings its own answers and
    // runs without one.
    const bool production = !runner;
    runner = _ProductionRunner(runner);
    if (!runner || time.IsDefault() || !std::isfinite(time.GetValue())) {
        return RigExecWarmFactoryResult();
    }
    // Non-const: a built job records its freshness proof on the bridge.
    RigSession *session = nullptr;
    for (RigSession &candidate : _sessions) {
        if (candidate.rigPath == rig) {
            session = &candidate;
            break;
        }
    }
    // Unknown rigs, playback rigs (a .rigexec never warms), and anything
    // without a bridge take no job: the frame evaluates live when asked.
    if (!session || session->playback || !session->bridge) {
        return RigExecWarmFactoryResult();
    }
    // A reasoned skip: records the session's last skip (the named cause a
    // test or the strip reads back) and returns it with the empty work.
    const auto skip = [&](RigExecWarmSkipReason reason,
                          const std::string &detail) {
        session->lastWarmSkip = reason;
        session->lastWarmSkipDetail = detail;
        // A reasoned skip is a non-publish signal toward un-warmable.
        _warmIndex->NoteSkipped(rig, time.GetValue());
        RigExecWarmFactoryResult result;
        result.skip = reason;
        result.detail = detail;
        return result;
    };
    if (!_scheduler || !_scheduler->IsGenerationCurrent(rig, generation)) {
        return RigExecWarmFactoryResult();
    }
    const RigExecRigEvaluator &evaluator = session->bridge->GetEvaluator();
    // The cached route, when the burst prepared one for this rig's
    // program: prepared pins, visit sets, and compiled rig signature, with static
    // reads and digests served from the burst maps. Anything else takes
    // the plain per-frame route below, which re-derives everything
    // exactly as before.
    const std::vector<RigExecValueOverride> overrides =
        session->bridge->GetInteractiveOverrides();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (program == nullptr) {
        // A rig with no baked program evaluates dynamically against
        // the live stage, which a worker must never touch. No job, ever;
        // the rig still memoizes its own UI-thread results.
        return skip(RigExecWarmSkipReason::D7Exempt, "no baked program (D7)");
    }
    // The job time's upstream values, from the session's pulled table only
    // (no data source is called here; a varying array is reconstructed
    // from its samples, with the frame's fold hash, and nothing is written
    // back). A varying source with no value at this time -- outside the
    // window, or a pull still pending -- warms nothing.
    std::vector<RigExecUpstreamValue> upstream;
    if (session->upstream &&
        !session->upstream->ValuesAt(time.GetValue(), &upstream)) {
        return skip(RigExecWarmSkipReason::Unsampleable,
                    "upstream not pulled for this time");
    }
    const bool cached = burst && burst->usable && program == burst->program &&
                        upstream == burst->upstream;
    if (!cached) {
        // Refreshed here -- lazily, at warm-build time -- and not on the
        // per-frame path: only warming reads the snapshot. The live state
        // a refresh pins is the last evaluated frame's either way (no
        // evaluation runs between that frame and this build), so warming
        // sees the same history it did under the eager refresh, including
        // after cache hits, which evaluate nothing on either path.
        _RefreshFrozenSnapshot(session);
        // The session's compiled rig chain bindings, verified at use and
        // rebound when a mid-compiled rig edit moved them. A rig that cannot bind
        // takes no job: the sampler would mark its chains stale and decline
        // anyway, but declining here skips the doomed sample.
        if (!_AdoptBridgeChainBindings(session)) {
            return skip(RigExecWarmSkipReason::Unsampleable, "chain-bind");
        }
    }
    // No snapshot, no job -- checked BEFORE sampling on both routes (the
    // cached route's refresh ran in burst prep), so a refusing rig skips
    // every sweep time without paying for doomed samples. Production
    // execution runs from the session's snapshot; an injected test runner
    // brings its own answers and runs without one.
    if (production && !session->frozen) {
        return skip(RigExecWarmSkipReason::FreezeRefused,
                    session->frozenError.empty() ? "freeze refused"
                                                 : session->frozenError);
    }
    RigExecProfiler *warmProfiler = session->bridge->MutableProfiler();
    uint64_t warmPhaseStart =
        warmProfiler->IsEnabled() ? RigExecProfiler::NowUs() : 0;
    const auto warmPhase = [&](const char *name) {
        if (warmProfiler->IsEnabled()) {
            const uint64_t end = RigExecProfiler::NowUs();
            warmProfiler->Record(name, "imagingWarm", warmPhaseStart, end);
            warmPhaseStart = end;
        }
    };
    RigExecFrameInputs inputs;
    if (cached) {
        if (!RigExecSampleFrameInputsWithBurstCache(evaluator, time, overrides,
                                                    upstream, burst, &inputs)) {
            return skip(RigExecWarmSkipReason::Unsampleable, "burst-sample");
        }
    } else {
        // Sampled here, on the UI thread: the worker receives values, never
        // live state, stage handles, or queries. The chains evaluate through
        // the session's pins -- the same values a fresh bind would read, held
        // to account sample by sample by the equivalence test.
        if (!RigExecSampleFrameInputsWithChainBindings(
                evaluator, time, overrides, upstream, session->chainBindings,
                &inputs)) {
            return skip(RigExecWarmSkipReason::Unsampleable, "chain-sample");
        }
    }
    warmPhase("Warm.SampleInputs");
    // A vector carrying a sample marked stale (viaChain) builds no job; the
    // frame evaluates live when asked and still memoizes through the live
    // path. The samplers mark none: chain reads resolve on the worker.
    if (inputs.HasChainResolvedInputs()) {
        return skip(RigExecWarmSkipReason::Unsampleable, "chain-resolved");
    }
    if (!RigExecControlStateDigestible(inputs, overrides)) {
        return skip(RigExecWarmSkipReason::Unsampleable, "undigestible");
    }
    RigExecFrozenEvalContext context;
    context.compiledRigSignature = evaluator.GetCompiledRigSignature();
    context.generation = generation;
    context.slotCount = program->GetProviderCount();
    context.programDigest =
        context.compiledRigSignature ^
        (uint64_t(program->GetBoundInputCount()) * 1099511628211ull) ^
        (uint64_t(program->GetVaryingInputCount()) * 16777619ull);
    context.varyingInputCount = inputs.values.size();
    uint32_t flags = 0;
    if (evaluator.GetPublishWeightFields()) {
        flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator.GetSolverGuidesEnabled()) {
        flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.flags = flags;
    // The session's snapshot, bound per job; the existence check ran
    // before sampling above (the lock never dropped between, so the
    // session cannot have swapped it out from under this build).
    context.frozen = session->frozen.get();
    if (!session->warmWorkspaces)
        session->warmWorkspaces = std::make_shared<RigExecWarmWorkspacePool>();
    auto workspaces = session->warmWorkspaces;
    RigExecFrameCacheKey key;
    // The same compiled rig half the bridge's lookup computes (compiled rig structure +
    // build count), so a completion lands where the UI thread will look.
    // The context keeps the bare compiled rig: that is what the frozen
    // snapshot pins and cross-checks against. The cached route reads the
    // prepared halves; the digests agree by construction (the same level-1s
    // over the same served values). The constant digest folds beside the
    // sampled one on both routes: the session's frozen avar
    // digest IS the compiled rig constant digest, refreshed by
    // _RefreshFrozenSnapshot above, so a value patch opens a new key
    // namespace while the compiled rig half stands.
    const uint64_t constantDigest = session->frozenAvarDigest;
    uint64_t unfoldedDigest = 0;
    if (cached) {
        key.compiledRigSignature = burst->compiledRigSignature;
        unfoldedDigest =
            RigExecControlStateDigestWithBurstCache(inputs, overrides, burst);
        key.controlDigest =
            RigExecFoldConstantDigest(unfoldedDigest, constantDigest);
    } else {
        key.compiledRigSignature =
            RigExecFrameCacheCompiledRigSignature(evaluator);
        unfoldedDigest = RigExecControlStateDigest(inputs, overrides);
        key.controlDigest =
            RigExecFoldConstantDigest(unfoldedDigest, constantDigest);
    }
    warmPhase("Warm.DigestContext");
    // Recorded at enqueue, on this thread: a warmed entry with no proof is
    // unreachable (lookups serve only under one), and the worker cannot
    // record it -- the bridge is UI-thread state. Every sample is fresh at
    // this time (the checks above), so the digest names fresh values and
    // proves them.
    session->bridge->NoteWarmingEnqueued(time, key.controlDigest,
                                         unfoldedDigest, inputs);
    // Dynamic-override bootstrapping: an override identity the
    // compiled rig index never learned would answer every plan as foreign (all
    // clusters). Admission is exact -- the seeds mirror SetOverrides'
    // placement -- and a no-op for identities already known.
    session->bridge->AdmitOverrideControls(overrides);
    std::shared_ptr<RigExecFrameCache> cache = session->bridge->GetFrameCache();
    RigExecBackgroundScheduler *scheduler = _scheduler.get();
    std::shared_ptr<RigExecWarmFrameIndex> warmIndex = _warmIndex;
    const auto progressPublication = warmIndex->RequestPublicationFor(
        rig, time.GetValue(), generation, key.compiledRigSignature);
    // Null unless explicitly opted in; no worker registry access or lock.
    const WarmPublishObserver publishObserver =
        production ? _warmPublishObserver : WarmPublishObserver();
    // Only sampled values cross the thread boundary, plus the snapshot the
    // context points at: the job's own shared_ptr keeps it alive past a
    // deactivation or a refresh that swaps the session's entry out. The
    // cache is shared (a completion for a dead session lands in a dead
    // cache); the scheduler outlives every job it runs -- it owns the pool,
    // and its destructor (Deactivate stopping the context's pool) joins
    // the workers before it goes.
    std::shared_ptr<const RigExecFrozenProgram> frozen = session->frozen;
    const auto frozenPatch = session->frozenPatch;
    // The source snapshot and dependency record travel with the pose.
    RigExecRetainedFrameState retainedState = RigExecCaptureRetainedState(
        inputs, overrides, key.compiledRigSignature,
        program->GetStepGraph().clustering.clusters.size(), constantDigest);
    warmPhase("Warm.EnqueueProofRetained");
    auto retained =
        std::make_shared<RigExecRetainedFrameState>(std::move(retainedState));
    const size_t retainedBytes = retained->RetainedBytes();
    RigExecEntryProvenance provenance =
        RigExecFullEvalProvenance(program->GetStepGraph(), time);
    provenance.unfoldedControlDigest = unfoldedDigest;
    warmPhase("Warm.Provenance");
    // The retained state holds its own copy, so the job takes the sampled
    // vector itself, immutable from here; nothing below reads `inputs`.
    std::shared_ptr<const RigExecFrameInputs> jobInputs =
        std::make_shared<RigExecFrameInputs>(std::move(inputs));
    RigExecWarmWork work = [jobInputs, context, key, cache, scheduler, rig,
                            runner, frozen, frozenPatch, workspaces, warmIndex,
                            retained, retainedBytes, provenance,
                            progressPublication, publishObserver](
                               const RigExecWarmRequest &request) {
        // Phase marks go to the scheduler's per-run timings (null unless
        // its profiler records); _FinishJob records them as one event.
        RigExecWarmPhaseTimings *const timings = request.timings;
        const auto workerPhase = [timings](const char *name) {
            if (timings)
                timings->Mark(name);
        };
        workerPhase(nullptr);
        RigExecFrozenEvalContext laneContext = context;
        RigExecWarmWorkspacePool::Lane *lane = workspaces->Acquire();
        struct Lease {
            RigExecWarmWorkspacePool::Lane *lane;
            ~Lease()
            {
                if (lane)
                    lane->busy.store(false);
            }
        } lease{lane};
        if (lane) {
            // The leased worker retires its old arena. A value edit must not
            // destroy every lane's potentially large arena on the UI thread.
            if (!lane->workspace || lane->snapshot != frozen ||
                lane->programDigest != context.programDigest) {
                lane->workspace.reset();
                lane->snapshot = frozen;
                lane->programDigest = context.programDigest;
                lane->workspace = RigExecCreateFrozenWorkspace(frozen);
            }
            laneContext.workspace = lane->workspace.get();
        }
        laneContext.avarPatch = frozenPatch.get();
        workerPhase("Warm.WorkerLane");
        const RigExecRigPose pose = RigExecEvaluateFrozen(
            laneContext, *jobInputs, runner, scheduler, rig);
        workerPhase("Warm.WorkerEvaluate");
        // Stale at publish means an edit landed mid-run -- a generation
        // bump or a scoped purge that moved this time's fence token (the
        // frame requeues under the live generation); anything else that
        // fails to publish is a genuine decline (the frame evaluates live
        // when asked). The fence is re-read per failure -- never trusted
        // across the evaluate/publish calls -- so a purge between them
        // still lands DeclinedGeneration.
        const auto stale = [&]() {
            return scheduler && !scheduler->IsWarmRequestCurrent(
                                    rig, request.generation, request.time,
                                    request.fenceToken);
        };
        if (!pose.valid) {
            return stale() ? RigExecWarmOutcome::DeclinedGeneration
                           : RigExecWarmOutcome::DeclinedInvalid;
        }
        const bool pubOk = RigExecPublishBackgroundCompletion(
            cache, key, request.time, pose, request.generation, scheduler, rig,
            request.fenceToken, retained, retainedBytes, &provenance,
            warmIndex.get());
        workerPhase("Warm.WorkerPublish");
        if (!pubOk) {
            return stale() ? RigExecWarmOutcome::DeclinedGeneration
                           : RigExecWarmOutcome::DeclinedInvalid;
        }
        // No new worker mutex: publication is fenced above; this immutable
        // captured cell only marks its own request, never owner request maps.
        progressPublication.Publish(key, request.generation);
        if (publishObserver)
            publishObserver(rig, request.time, key, request.generation, pose,
                            cache);
        return RigExecWarmOutcome::Published;
    };
    warmPhase("Warm.WorkConstruction");
    return RigExecWarmFactoryResult{std::move(work)};
}

void
RigExecImagingRegistry::SetWarmPublishObserverForTesting(
    WarmPublishObserver observer)
{
    if (const Ptr routed = _Routed()) {
        routed->SetWarmPublishObserverForTesting(std::move(observer));
        return;
    }
    _warmPublishObserver = std::move(observer);
}

RigExecProfiler *
RigExecImagingRegistry::MutableSchedulerProfiler()
{
    if (const Ptr routed = _Routed()) {
        return routed->MutableSchedulerProfiler();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _scheduler ? _scheduler->MutableProfiler() : nullptr;
}

RigExecProfiler *
RigExecImagingRegistry::MutableBridgeProfiler(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->MutableBridgeProfiler(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge) {
            return session.bridge->MutableProfiler();
        }
    }
    return nullptr;
}

std::vector<UsdTimeCode>
RigExecImagingRegistry::_CursorSweepTimes(const RigSession &session,
                                          UsdTimeCode playhead)
{
    if (!session.warmRangeActive || !session.bridge || !_scheduler ||
        !_warmIndex) {
        return _SweepTimes(playhead);
    }
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(session.rigPath);
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(session.bridge->GetEvaluator());
    const double center = playhead.GetValue();
    std::vector<std::pair<double, double>> scored;
    for (const double time : session.warmRange) {
        // The playhead is never queued, however it arises (the scheduler
        // enforces this too; skipping here keeps the vector honest).
        if (time == center) {
            continue;
        }
        if (!_warmIndex->IsCursorVisitable(session.rigPath, time, generation,
                                           compiledRig)) {
            continue;
        }
        scored.emplace_back(std::abs(time - center), time);
    }
    // Closest first; ties prefer the future (playback warms ahead).
    std::sort(scored.begin(), scored.end(),
              [](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) {
                  if (a.first != b.first) {
                      return a.first < b.first;
                  }
                  return a.second > b.second;
              });
    std::vector<UsdTimeCode> times;
    times.reserve(scored.size());
    for (const std::pair<double, double> &entry : scored) {
        times.emplace_back(entry.second);
    }
    return times;
}

size_t
RigExecImagingRegistry::OnEditCommitted(RigExecFrozenStepRunner runner)
{
    if (const Ptr routed = _Routed()) {
        return routed->OnEditCommitted(runner);
    }
    // Snapshot the rigs and the playhead under a short lock; the triggers
    // below sample outside it (their factories re-resolve each session, so
    // a Deactivate mid-trigger only skips frames, never dangles).
    std::vector<SdfPath> rigs;
    UsdTimeCode playhead;
    // Held across the unlocked trigger below: a Deactivate in between stops
    // the context's pool, and the last holder is what destroys it.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_scheduler) {
            return 0;
        }
        scheduler = _scheduler;
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                rigs.push_back(session.rigPath);
            }
        }
        playhead = _lastTime;
    }
    if (rigs.empty() || playhead.IsDefault() ||
        !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The runner passes through unresolved: BuildWarmWork reads
    // production-ness off the null before resolving it to the production
    // runner itself.
    size_t enqueued = 0;
    for (const SdfPath &rig : rigs) {
        const RigExecFrameGeneration generation =
            scheduler->CurrentGeneration(rig);
        // The session's standing burst: rebuilt when its pins moved,
        // served across ticks while current. Usable takes the cached
        // route; overrun takes the plain per-frame route (null burst),
        // since frames can still sample; shape-declined takes no
        // factory -- no frame could sample. The pointer names session
        // storage, captured for synchronous same-thread use inside the
        // trigger only (no session can activate or deactivate under it).
        RigExecBurstSampleCache *burst = nullptr;
        bool useCachedRoute = false;
        bool allowPlainRoute = false;
        std::vector<UsdTimeCode> sweepTimes;
        RigExecWarmSamplingBudget budget;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (RigSession &session : _sessions) {
                if (session.rigPath == rig && !session.playback &&
                    session.bridge) {
                    burst = _PrepareWarmBurst(&session);
                    useCachedRoute = burst && burst->usable;
                    allowPlainRoute =
                        burst && !burst->usable &&
                        (session.burstOverrun || session.burstUpstreamVaries ||
                         (session.burstDeclined && session.upstream));
                    sweepTimes = _CursorSweepTimes(session, playhead);
                    // Retired frames re-warm FIRST: the
                    // affected-time set replaces the fixed sweep for the
                    // edit, so a release re-warms what it retired before
                    // the cursor wanders on.
                    _PrependPendingRewarm(&session, playhead, &sweepTimes);
                    budget.maxFactoryInvocations = _warmSamplingMaxInvocations;
                    budget.maxMs = _warmSamplingMaxMs;
                    // Stamped after prep: the stop bounds factory sampling,
                    // not the burst prep above (frozen refresh + rebuild),
                    // which runs outside the factory budget under its own
                    // slice. Measured from the tick's start, a prep slower
                    // than the stop would trip it before the first sample on
                    // every tick, and the rig would never warm.
                    budget.startUs = RigExecProfiler::NowUs();
                    break;
                }
            }
        }
        RigExecWarmJobFactory factory;
        if (useCachedRoute || allowPlainRoute) {
            RigExecBurstSampleCache *route = useCachedRoute ? burst : nullptr;
            factory = [this, rig, generation, runner, route](UsdTimeCode time) {
                return BuildWarmWork(rig, time, generation, runner, route);
            };
        }
        enqueued += scheduler->OnEditCommitted(
            rig, playhead, generation, std::move(sweepTimes),
            std::move(factory), kRigExecFrameCacheDefaultNeighborRadius,
            budget);
    }
    return enqueued;
}

bool
RigExecImagingRegistry::_WarmOneFallbackFrame(RigSession *session,
                                              UsdTimeCode playhead)
{
    if (!session || !session->warmRangeActive || session->warmRange.empty() ||
        !session->bridge || session->playback || !_warmIndex || !_scheduler ||
        !session->bridge->GetInteractiveOverrides().empty()) {
        return false;
    }
    // The live evaluator holds this time's upstream values, not the
    // candidate's: a varying source warms through jobs only.
    if (session->upstream && session->upstream->AnyVaries()) {
        return false;
    }
    const uint64_t serial =
        session->bridge->GetEvaluator().GetStageEditSerial();
    if (session->fallbackSerial != serial) {
        session->fallbackSerial = serial;
        session->fallbackDeclined.clear();
    }
    const auto states = _warmIndex->States(
        session->rigPath, session->warmRange,
        _scheduler->CurrentGeneration(session->rigPath),
        RigExecFrameCacheCompiledRigSignature(session->bridge->GetEvaluator()));
    double nearest = std::numeric_limits<double>::infinity();
    double candidate = 0.0;
    bool found = false;
    for (size_t i = 0; i < session->warmRange.size(); ++i) {
        const double time = session->warmRange[i];
        if (_warmIndex->HasRequestPublished(
                session->rigPath, time,
                _scheduler->CurrentGeneration(session->rigPath),
                RigExecFrameCacheCompiledRigSignature(
                    session->bridge->GetEvaluator())))
            continue;
        if (states[i] == RigExecWarmFrameState::Cached ||
            states[i] == RigExecWarmFrameState::Warming ||
            session->fallbackDeclined.count(time)) {
            continue;
        }
        const double distance = std::abs(time - playhead.GetValue());
        if (!found || distance < nearest) {
            candidate = time;
            nearest = distance;
            found = true;
        }
    }
    if (!found) {
        return false;
    }
    ++session->fallbackAttempts;
    ++session->evaluationCount;
    const uint64_t token = _warmIndex->RequestToken(session->rigPath);
    if (!session->bridge->WarmFrameOnCallingThread(UsdTimeCode(candidate))) {
        session->fallbackDeclined.insert(candidate);
    } else {
        RigExecFrameCacheKey key;
        const auto generation = _scheduler->CurrentGeneration(session->rigPath);
        const uint64_t compiledRig = RigExecFrameCacheCompiledRigSignature(
            session->bridge->GetEvaluator());
        if (_warmIndex->FindCachedKey(session->rigPath, candidate, generation,
                                      compiledRig, &key))
            _warmIndex->NoteRequestPublished(session->rigPath, candidate, key,
                                             generation, token);
    }
    return true;
}

size_t
RigExecImagingRegistry::OnIdle(RigExecFrozenStepRunner runner)
{
    if (const Ptr routed = _Routed()) {
        return routed->OnIdle(runner);
    }
    // Frames on demand first: cached frames become entries, the rest join
    // the rewarm lane the sweep below enqueues before anything else.
    ServiceFrameDemand(8, /* allowFallback = */ true);
    std::vector<SdfPath> rigs;
    UsdTimeCode playhead;
    // Held across the unlocked trigger below: a Deactivate in between stops
    // the context's pool, and the last holder is what destroys it.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || !_scheduler) {
            return 0;
        }
        scheduler = _scheduler;
        for (const RigSession &session : _sessions) {
            if (!session.playback) {
                rigs.push_back(session.rigPath);
            }
        }
        playhead = _lastTime;
    }
    if (rigs.empty() || playhead.IsDefault() ||
        !std::isfinite(playhead.GetValue())) {
        return 0;
    }
    // The runner passes through unresolved: BuildWarmWork reads
    // production-ness off the null before resolving it to the production
    // runner itself.
    size_t enqueued = 0;
    for (const SdfPath &rig : rigs) {
        const RigExecFrameGeneration generation =
            scheduler->CurrentGeneration(rig);
        // The standing burst, as in OnEditCommitted: cached route when
        // usable, plain per-frame route on overrun, no factory on a
        // shape decline.
        RigExecBurstSampleCache *burst = nullptr;
        bool useCachedRoute = false;
        bool allowPlainRoute = false;
        std::vector<UsdTimeCode> sweepTimes;
        RigExecWarmSamplingBudget budget;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (RigSession &session : _sessions) {
                if (session.rigPath == rig && !session.playback &&
                    session.bridge) {
                    burst = _PrepareWarmBurst(&session);
                    useCachedRoute = burst && burst->usable;
                    allowPlainRoute =
                        burst && !burst->usable &&
                        (session.burstOverrun || session.burstUpstreamVaries ||
                         (session.burstDeclined && session.upstream));
                    sweepTimes = _CursorSweepTimes(session, playhead);
                    // Retired frames first here too: without-Qt the idle
                    // tick is the only recurring driver.
                    _PrependPendingRewarm(&session, playhead, &sweepTimes);
                    budget.maxFactoryInvocations = _warmSamplingMaxInvocations;
                    budget.maxMs = _warmSamplingMaxMs;
                    // Stamped after prep: the stop bounds factory sampling,
                    // not the burst prep above (frozen refresh + rebuild),
                    // which runs outside the factory budget under its own
                    // slice. Measured from the tick's start, a prep slower
                    // than the stop would trip it before the first sample on
                    // every tick, and the rig would never warm.
                    budget.startUs = RigExecProfiler::NowUs();
                    break;
                }
            }
        }
        RigExecWarmJobFactory factory;
        if (useCachedRoute || allowPlainRoute) {
            RigExecBurstSampleCache *route = useCachedRoute ? burst : nullptr;
            factory = [this, rig, generation, runner, route](UsdTimeCode time) {
                return BuildWarmWork(rig, time, generation, runner, route);
            };
        }
        enqueued +=
            scheduler->OnIdle(rig, playhead, generation, std::move(sweepTimes),
                              std::move(factory), budget);
    }
    // Workers remain the preferred route. If they made no progress and
    // have drained, fill one outstanding frame through the live evaluator.
    // This covers dynamic rigs and operations whose inputs cannot freeze,
    // without maintaining another operation allowlist. Never run during a
    // preview, when warming is disabled, or for an injected test runner.
    const auto background = scheduler->Stats();
    if (!runner && enqueued == 0 && background.queuedDepth == 0 &&
        background.running == 0 && RigExecParallelEvaluationEnabled()) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_previewActive && _warmSamplingMaxInvocations > 0 &&
            _warmSamplingMaxMs > 0.0) {
            for (RigSession &session : _sessions) {
                // The bridge's stored switch, not the environment.
                if (!session.bridge ||
                    !RigExecBackgroundWarmingEnabled(
                        session.bridge->GetFrameCacheMode(), true)) {
                    continue;
                }
                if (_WarmOneFallbackFrame(&session, playhead)) {
                    break;
                }
            }
        }
    }
    return enqueued;
}

size_t
RigExecImagingRegistry::GetWarmingProgressCount()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetWarmingProgressCount();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    size_t progress = _scheduler ? _scheduler->Stats().factoryInvocations : 0;
    for (const RigSession &session : _sessions) {
        progress += session.fallbackAttempts;
    }
    return progress;
}

RigExecFrameGeneration
RigExecImagingRegistry::CurrentFrameGeneration(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->CurrentFrameGeneration(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_scheduler) {
        return 0;
    }
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return _scheduler->CurrentGeneration(rig);
        }
    }
    return 0;
}

RigExecWarmFenceToken
RigExecImagingRegistry::CurrentFenceToken(const SdfPath &rig, UsdTimeCode time)
{
    if (const Ptr routed = _Routed()) {
        return routed->CurrentFenceToken(rig, time);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (_sessions.empty() || !_scheduler) {
        return 0;
    }
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig) {
            return _scheduler->CurrentFenceToken(rig, time);
        }
    }
    return 0;
}

void
RigExecImagingRegistry::_CancelGenerationLocked(const SdfPath &rig)
{
    if (_scheduler) {
        _scheduler->CancelGeneration(rig);
        _warmIndex->NoteGeneration(rig, _scheduler->CurrentGeneration(rig));
    }
}

void
RigExecImagingRegistry::_CancelGenerationTimesLocked(const SdfPath &rig)
{
    if (!_scheduler) {
        return;
    }
    _warmIndex->InvalidateRequest(rig);
    // The unpartitioned affected set: completions the warm index holds
    // plus queued and running times the scheduler holds. No generation
    // bump or index generation push: unaffected times keep their tokens and
    // their running jobs publish normally, while affected times get
    // fresh fence tokens and dirty flags. The partitioned path
    // (_RetireAffectedTimesLocked) narrows this by entry provenance;
    // this stays as its no-oracle fallback.
    std::vector<UsdTimeCode> times = _scheduler->InFlightTimes(rig);
    const std::vector<UsdTimeCode> completed = _warmIndex->CompletedTimes(rig);
    times.insert(times.end(), completed.begin(), completed.end());
    _scheduler->CancelGenerationTimes(rig, times);
    for (const UsdTimeCode &time : times) {
        _warmIndex->NoteDirtied(rig, time.IsDefault() ? 0.0 : time.GetValue());
    }
}

void
RigExecImagingRegistry::_NotePendingRewarm(RigSession *session,
                                           const std::vector<double> &times)
{
    if (!session) {
        return;
    }
    // De-duplicated against the standing list; capped past which the
    // overflow is dropped (dirtied frames stay visitable, so the cursor
    // sweep covers them when a range is set, and every frame still
    // evaluates live on visit when none is).
    static constexpr size_t kPendingRewarmCap = 4096;
    for (const double time : times) {
        if (!std::isfinite(time)) {
            continue;
        }
        if (session->pendingRewarm.size() >= kPendingRewarmCap) {
            break;
        }
        if (std::find(session->pendingRewarm.begin(),
                      session->pendingRewarm.end(),
                      time) == session->pendingRewarm.end()) {
            session->pendingRewarm.push_back(time);
        }
    }
}

void
RigExecImagingRegistry::_PrependPendingRewarm(
    RigSession *session, UsdTimeCode playhead,
    std::vector<UsdTimeCode> *sweepTimes)
{
    if (!session || !sweepTimes || session->pendingRewarm.empty() ||
        !session->bridge || !_scheduler || !_warmIndex) {
        return;
    }
    const double center = playhead.IsNumeric() ? playhead.GetValue() : 0.0;
    // One batched read, no sampling on the query path.
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(session->rigPath);
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(session->bridge->GetEvaluator());
    const std::vector<RigExecWarmFrameState> states = _warmIndex->States(
        session->rigPath, session->pendingRewarm, generation, compiledRig);
    std::vector<std::pair<double, double>> scored;
    std::vector<double> keep;
    for (size_t i = 0; i < session->pendingRewarm.size(); ++i) {
        const double time = session->pendingRewarm[i];
        const RigExecWarmFrameState state =
            i < states.size() ? states[i] : RigExecWarmFrameState::Uncached;
        if (state == RigExecWarmFrameState::Cached) {
            continue;
        }
        keep.push_back(time);
        if (state == RigExecWarmFrameState::Warming) {
            continue;
        }
        scored.emplace_back(std::abs(time - center), time);
    }
    session->pendingRewarm.swap(keep);
    std::sort(scored.begin(), scored.end(),
              [](const std::pair<double, double> &a,
                 const std::pair<double, double> &b) {
                  if (a.first != b.first) {
                      return a.first < b.first;
                  }
                  return a.second > b.second;
              });
    std::vector<UsdTimeCode> first;
    first.reserve(scored.size());
    for (const auto &entry : scored) {
        // The playhead is never queued, however it arises (the scheduler
        // enforces this too; skipping here keeps the vector honest).
        if (playhead.IsNumeric() && entry.second == center) {
            continue;
        }
        first.emplace_back(entry.second);
    }
    sweepTimes->insert(sweepTimes->begin(), first.begin(), first.end());
}

void
RigExecImagingRegistry::_RetireAffectedTimesLocked(
    RigSession *session, const UsdNotice::ObjectsChanged &notice,
    RigExecNoticeDisposition disposition,
    const std::vector<SdfPath> &patchedPaths)
{
    if (!session || !session->bridge || session->playback || !_scheduler ||
        !_warmIndex) {
        return;
    }
    const SdfPath &rig = session->rigPath;
    RigExecProfileScope retireScope(session->bridge->MutableProfiler(),
                                    "Notice.RetireAffectedTimes", "notice");
    _warmIndex->InvalidateRequest(rig);
    RigExecImagingBridge *bridge = session->bridge.get();
    // Snapshot in-flight work before completions: a first-fill job that
    // finishes between the reads must be covered by at least one set.
    const std::vector<UsdTimeCode> inFlight = _scheduler->InFlightTimes(rig);
    const RigExecBakedProgram *program =
        bridge->GetEvaluator().GetBakedProgram();
    const RigExecOutputAffectedIndex *index = [&]() {
        RigExecProfileScope scope(bridge->MutableProfiler(),
                                  "Notice.AffectedIndex", "notice");
        return bridge->GetAffectedIndex();
    }();
    if (!program || !index || index->Empty()) {
        // No oracle: every completed time is affected, as before. In-flight
        // times retire (their provenance publishes later); proofs retire
        // by the notice's paths.
        _CancelGenerationTimesLocked(rig);
        const std::vector<UsdTimeCode> completed =
            _warmIndex->CompletedTimes(rig);
        std::vector<double> rewarm;
        for (const UsdTimeCode &time : inFlight) {
            rewarm.push_back(time.GetValue());
        }
        for (const UsdTimeCode &time : completed) {
            rewarm.push_back(time.IsDefault() ? 0.0 : time.GetValue());
        }
        _NotePendingRewarm(session, rewarm);
        bridge->RetireProofsForControls(RigExecNoticeControlIds(notice));
        return;
    }
    // The edit's controls: the evaluator's exact patched set for a
    // patch (no notice noise -- a Patched notice carries nothing but
    // patchable values and their ancestors), the full adapter for a
    // stamp bump. A routed edit names only the properties the program
    // reads, so a property nothing reads retires nothing; each named one
    // stands for itself where the index knows it -- an overridable input,
    // seeded as an override on it is -- and brings its prim along where
    // it does not, which is the adapter's own answer for it.
    std::vector<RigExecControlId> controls;
    if (disposition == RigExecNoticeDisposition::Edited) {
        std::set<RigExecControlId> ids;
        for (const SdfPath &path : patchedPaths) {
            if (path.IsEmpty()) {
                continue;
            }
            const RigExecControlId id = RigExecControlIdForPath(path);
            ids.insert(id);
            if (path.IsPropertyPath() && !index->IsKnownControl(id)) {
                ids.insert(RigExecControlIdForPath(path.GetPrimPath()));
            }
        }
        controls.assign(ids.begin(), ids.end());
    } else if (disposition == RigExecNoticeDisposition::Patched) {
        std::set<RigExecControlId> ids;
        for (const SdfPath &path : patchedPaths) {
            if (path.IsEmpty()) {
                continue;
            }
            ids.insert(path.GetString());
            if (path.IsPropertyPath()) {
                ids.insert(path.GetPrimPath().GetString());
            }
        }
        controls.assign(ids.begin(), ids.end());
    } else {
        controls = RigExecNoticeControlIds(notice);
    }
    const RigExecBakedClusterSet dirty = index->AffectedByControls(controls);
    // Weight-object edits: a notice path at or under a weight object an
    // entry read (WeightPacket steps resolve through step.object, outside
    // the avar-only index, so the provenance read set is the oracle).
    std::vector<SdfPath> noticePaths;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        noticePaths.push_back(path);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        noticePaths.push_back(path);
        if (path.IsPropertyPath()) {
            noticePaths.push_back(path.GetPrimPath());
        }
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        noticePaths.push_back(path);
    }
    const uint64_t newConstants = [&]() {
        RigExecProfileScope scope(bridge->MutableProfiler(),
                                  "Notice.ConstantDigest", "notice");
        return RigExecCompiledRigConstantDigest(*program);
    }();
    const std::shared_ptr<RigExecFrameCache> cache = bridge->GetFrameCache();
    const std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>> done =
        _warmIndex->CompletedKeys(rig);
    std::vector<UsdTimeCode> purge = inFlight;
    std::vector<double> rewarm;
    for (const UsdTimeCode &time : inFlight) {
        rewarm.push_back(time.GetValue());
    }
    for (const auto &entry : done) {
        RigExecProfileScope entryScope(bridge->MutableProfiler(),
                                       "Notice.RetireEntry", "notice");
        const UsdTimeCode time = entry.first;
        const RigExecFrameCacheKey &oldKey = entry.second;
        const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
        RigExecEntryProvenance provenance;
        bool affected = true;
        if (cache && cache->LookupProvenance(oldKey, &provenance) &&
            provenance.unfoldedControlDigest != 0) {
            // The affected test is the executor's own dirtiness rule
            // (bakedSchedule): an entry's pose moved exactly when a
            // dirty cluster reaches its computation, or an edited weight
            // object feeds it. Constant regions are recorded in the
            // provenance but deliberately NOT consulted: the patchable
            // binding's compose-cluster cone is the exact oracle (a
            // seedless binding dirties nothing and carries), while
            // regions over-approximate by stride.
            affected = false;
            for (const int cluster : provenance.clusters) {
                // Bounded by hand: ClusterSet::Test guards negative
                // only, and provenance is publisher-controlled.
                if (cluster >= 0 && size_t(cluster) < dirty.words.size() * 64 &&
                    dirty.Test(cluster)) {
                    affected = true;
                    break;
                }
            }
            for (size_t i = 0; !affected && i < provenance.weightReads.size();
                 ++i) {
                const SdfPath weight(provenance.weightReads[i]);
                if (weight.IsEmpty()) {
                    continue;
                }
                for (const SdfPath &path : noticePaths) {
                    if (path == weight || path.HasPrefix(weight)) {
                        affected = true;
                        break;
                    }
                }
            }
        }
        if (!affected) {
            // Lane a/b: the entry's computation touches nothing dirty.
            // Across a namespace move it carries (re-keyed, re-pointed,
            // zero recompute); otherwise it stands untouched -- no purge,
            // no dirty flag, no requeue. A failed carry degrades to
            // lane c rather than stranding the frame.
            if (cache && bridge->CarryEntry(time, oldKey, newConstants)) {
                // The new key holds the row now: old-key jobs queued or
                // running for this time must drop, not resurrect it.
                purge.push_back(time);
                continue;
            }
            if (cache) {
                RigExecEntryProvenance carried;
                if (cache->LookupProvenance(oldKey, &carried) &&
                    carried.unfoldedControlDigest != 0 &&
                    RigExecFoldConstantDigest(carried.unfoldedControlDigest,
                                              newConstants) !=
                        oldKey.controlDigest) {
                    // A namespace move the carry declined (evicted base,
                    // pose-only entry, over-cap re-publish): retire and
                    // re-warm rather than serve stale.
                    purge.push_back(time);
                    rewarm.push_back(stamped);
                    _warmIndex->NoteDirtied(rig, stamped);
                    continue;
                }
            }
            continue;
        }
        // Lane c: retire the time. The stale entry strands under its old
        // key (unreachable by construction -- the digest backstop -- and
        // drained by LRU); the row goes dirty and the frame requeues for
        // a full re-warm. (Cone-granular re-execution waits on slot
        // capture: sources-only retained state plans but cannot run a
        // Partial, so the planner's verdict here would only restate this
        // partition at higher cost.)
        purge.push_back(time);
        rewarm.push_back(stamped);
        _warmIndex->NoteDirtied(rig, stamped);
    }
    // Every in-flight time is included: its provenance may not have
    // published yet, so it cannot safely be classified as unaffected.
    _scheduler->CancelGenerationTimes(rig, purge);
    _NotePendingRewarm(session, rewarm);
    RigExecProfileScope proofScope(bridge->MutableProfiler(),
                                   "Notice.RetireProofs", "notice");
    bridge->RetireProofsForControls(controls);
}

void
RigExecImagingRegistry::CancelFrameGeneration(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        routed->CancelFrameGeneration(rig);
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex);
    _CancelGenerationLocked(rig);
}

RigExecBackgroundSchedulerStats
RigExecImagingRegistry::GetBackgroundStats()
{
    if (const Ptr routed = _Routed()) {
        return routed->GetBackgroundStats();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    RigExecBackgroundSchedulerStats stats = _retiredStats;
    if (_scheduler) {
        _AddSchedulerStats(&stats, _scheduler->Stats(), /* gauges = */ true);
    }
    return stats;
}

void
RigExecImagingRegistry::WaitUntilBackgroundIdle()
{
    if (const Ptr routed = _Routed()) {
        routed->WaitUntilBackgroundIdle();
        return;
    }
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    // Outside _mutex: workers never take it, but idleness can outlast a
    // caller's patience for holding it.
    if (scheduler) {
        scheduler->WaitUntilIdle();
    }
}

RigExecFrameCacheStats
RigExecImagingRegistry::GetFrameCacheStats(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetFrameCacheStats(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig && session.bridge) {
            return session.bridge->GetFrameCacheStats();
        }
    }
    return RigExecFrameCacheStats();
}

void
RigExecImagingRegistry::ClearFrameCache(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        routed->ClearFrameCache(rig);
        return;
    }
    // The fence is outermost: read the scheduler under a short lock, then
    // hold the fence across the whole cancel-THEN-clear below. Lock order
    // fence, then registry, then scheduler, then cache shards, then the
    // warm index -- never inverted.
    // Held, not borrowed: a Deactivate while the fence is held stops the
    // pool, and the scheduler (and its fence mutex) must outlive the lock.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
    // Cancel in its own scope BEFORE locking for the clear: the registry
    // mutex is non-recursive, so cancel-under-clear-lock self-deadlocks.
    CancelFrameGeneration(rig);
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig && session.bridge) {
                session.bridge->ClearFrameCache();
            }
        }
        _warmIndex->ResetRig(rig);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig && session.warmRangeActive &&
                session.bridge)
                _warmIndex->SetRequest(
                    rig, session.warmRange,
                    _scheduler ? _scheduler->CurrentGeneration(rig) : 0,
                    RigExecFrameCacheCompiledRigSignature(
                        session.bridge->GetEvaluator()));
        }
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig) {
                session.fallbackDeclined.clear();
            }
        }
    }
}

bool
RigExecImagingRegistry::SetWarmRange(const SdfPath &rig,
                                     const std::vector<double> &frames)
{
    if (const Ptr routed = _Routed()) {
        return routed->SetWarmRange(rig, frames);
    }
    bool found = false;
    bool repull = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        const auto ranges = [&]() {
            std::vector<std::pair<double, double>> out;
            for (const RigSession &session : _sessions) {
                if (session.warmRangeActive && !session.warmRange.empty()) {
                    out.emplace_back(session.warmRange.front(),
                                     session.warmRange.back());
                }
            }
            return out;
        };
        const UsdStageRefPtr stage = _PreviewStageLocked();
        const RigExecUpstreamPullWindow before =
            _PullWindow(ranges(), stage, _lastTime);
        for (RigSession &session : _sessions) {
            if (session.rigPath == rig && !session.playback && session.bridge) {
                session.warmRange.clear();
                for (const double time : frames) {
                    if (std::isfinite(time)) {
                        session.warmRange.push_back(time);
                    }
                }
                std::sort(session.warmRange.begin(), session.warmRange.end());
                session.warmRange.erase(std::unique(session.warmRange.begin(),
                                                    session.warmRange.end()),
                                        session.warmRange.end());
                session.warmRangeActive = true;
                _warmIndex->SetRequest(
                    rig, session.warmRange,
                    _scheduler ? _scheduler->CurrentGeneration(rig) : 0,
                    RigExecFrameCacheCompiledRigSignature(
                        session.bridge->GetEvaluator()));
                session.fallbackDeclined.clear();
                found = true;
                break;
            }
        }
        // Pulled tables reconstruct the window's frames: a moved window
        // makes them stale, and the re-pull below (its serial move cancels
        // the generation) replaces them before any job reads them.
        if (found && !_upstreamTables.empty()) {
            const RigExecUpstreamPullWindow after =
                _PullWindow(ranges(), stage, _lastTime);
            repull = after.lo != before.lo || after.hi != before.hi;
        }
    }
    if (repull) {
        // Unlocked: the pull calls into upstream data sources.
        for (const Chain &chain : _BoundChains()) {
            if (chain.results) {
                chain.results->PullUpstreamInputs();
            }
        }
    }
    return found;
}

std::vector<RigExecWarmFrameState>
RigExecImagingRegistry::GetFrameStates(const SdfPath &rig,
                                       const std::vector<double> &frames)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetFrameStates(rig, frames);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    if (frames.empty() || frames.size() > 100000) {
        return {};
    }
    const RigSession *session = nullptr;
    for (const RigSession &candidate : _sessions) {
        if (candidate.rigPath == rig && !candidate.playback &&
            candidate.bridge) {
            session = &candidate;
            break;
        }
    }
    if (!session || !_scheduler || !_warmIndex) {
        return {};
    }
    const RigExecFrameGeneration generation =
        _scheduler->CurrentGeneration(rig);
    const uint64_t compiledRig =
        RigExecFrameCacheCompiledRigSignature(session->bridge->GetEvaluator());
    return _warmIndex->States(rig, frames, generation, compiledRig);
}

RigExecImagingBridge *
RigExecImagingRegistry::GetBridge(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetBridge(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge) {
            return session.bridge.get();
        }
    }
    return nullptr;
}

RigExecBakedPlayback *
RigExecImagingRegistry::GetPlayback(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetPlayback(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (RigSession &session : _sessions) {
        if (session.rigPath == rig && session.playback) {
            return session.playback.get();
        }
    }
    return nullptr;
}

std::vector<std::pair<UsdTimeCode, RigExecFrameCacheKey>>
RigExecImagingRegistry::GetCompletedKeys(const SdfPath &rig)
{
    if (const Ptr routed = _Routed()) {
        return routed->GetCompletedKeys(rig);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    for (const RigSession &session : _sessions) {
        if (session.rigPath == rig && !session.playback && session.bridge &&
            _warmIndex) {
            return _warmIndex->CompletedKeys(rig);
        }
    }
    return {};
}

bool
RigExecImagingRegistry::SetWeightOverlay(const std::string &weightPrimPath)
{
    if (_legacyHandle) {
        // The legacy selection is remembered on the handle whatever it
        // routes to -- the old singleton's viewer mode outlived every
        // activation, so the next legacy activation carries it (see
        // _LegacyActivate) -- and applied to the current context now.
        SdfPath resolved;
        if (!_ParseOverlayPath(weightPrimPath, &resolved)) {
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _weightOverlay = resolved;
            _legacyOverlaySet = true;
        }
        if (const Ptr routed = _Routed()) {
            return routed->SetWeightOverlay(weightPrimPath);
        }
        return true;
    }
    UsdTimeCode time;
    // Fenced cancel-THEN-clear like ClearFrameCache: the fence
    // is outermost, so it is taken before the registry lock below and held
    // across every session's cancel plus clear.
    // Held, not borrowed: a Deactivate while the fence is held stops the
    // pool, and the scheduler (and its fence mutex) must outlive the lock.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // An arbitrary caller-supplied string reaches SdfPath here and its
        // constructor is loud about a malformed one. Ask first -- the same
        // guard RigExecImaging_GetGuideBoundsAssetSpace uses.
        SdfPath resolved;
        if (!_ParseOverlayPath(weightPrimPath, &resolved)) {
            return false;
        }
        if (_weightOverlay == resolved)
            return true;
        _weightOverlay = resolved;
        if (_sessions.empty()) {
            // Remembered for whenever a rig is activated. Not a failure:
            // a host that sets its viewer mode before opening a stage has
            // done nothing wrong.
            return true;
        }
        // Offered to every active rig: the path selects at most one rig's
        // weight object, and a bridge that does not own it draws no overlay.
        for (RigSession &session : _sessions) {
            if (session.playback) {
                session.playback->SetWeightOverlay(resolved);
            } else {
                // Cancel BEFORE the bridge clears: the overlay
                // flag rides to workers in context.flags and changes pose
                // content without moving the key, so the old clear-then-
                // cancel order let a worker publish a stale-overlay pose
                // into the cleared cache between the two. Retired jobs
                // re-enqueue fresh on the next commit.
                _CancelGenerationLocked(session.rigPath);
                session.bridge->SetWeightOverlay(resolved);
            }
            session.dirty = true;
        }
        time = _lastTime;
    }
    // Republished OUTSIDE the lock, exactly as _OnObjectsChanged does it:
    // SetTime takes the same plain (non-recursive) std::mutex, so calling
    // it while still holding _mutex is a self-deadlock. This is also what
    // makes the change visible immediately -- the fresh generation's diff
    // reports the overlay's displayColor primvar appearing or disappearing
    // as structural, which dirties the mesh universally.
    return SetTime(time);
}

// Manipulation preview

namespace {

// The largest element count an array slot may declare.
constexpr size_t kMaxPreviewElements = size_t(1) << 20;
const TfToken kPreviewFloatArray("float[]");

// A declaration line's element count: plain decimal digits only, at most
// kMaxPreviewElements.
bool
_ParsePreviewElementCount(const std::string &text, size_t *count)
{
    if (text.empty()) {
        return false;
    }
    size_t value = 0;
    for (const char c : text) {
        if (c < '0' || c > '9') {
            return false;
        }
        value = value * 10 + size_t(c - '0');
        if (value > kMaxPreviewElements) {
            return false;
        }
    }
    *count = value;
    return true;
}

// How many doubles a value of this type takes, and the token naming how to
// rebuild it. An unsupported type returns an empty token: a manipulation
// cannot preview a value it cannot reconstruct, and saying so at declaration
// time is better than guessing per sample. An element count is required for,
// and only accepted on, an array slot: float[], float2[], or any type held
// as a VtVec3fArray (point3f[], float3[], vector3f[], normal3f[], ...).
TfToken
_PreviewValueKind(const SdfValueTypeName &type, bool hasElementCount,
                  size_t elementCount, size_t *arity)
{
    static const TfToken kDouble("double");
    static const TfToken kFloat("float");
    static const TfToken kVec3d("vec3d");
    static const TfToken kVec3f("vec3f");
    static const TfToken kMatrix4d("matrix4d");
    static const TfToken kFloat2Array("float2[]");
    static const TfToken kVec3fArray("vec3f[]");
    *arity = 0;
    if (type == SdfValueTypeNames->FloatArray) {
        if (!hasElementCount)
            return TfToken();
        *arity = elementCount;
        return kPreviewFloatArray;
    }
    if (type == SdfValueTypeNames->Float2Array) {
        if (!hasElementCount) {
            return TfToken();
        }
        *arity = 2 * elementCount;
        return kFloat2Array;
    }
    if (type.GetType() == TfType::Find<VtVec3fArray>()) {
        if (!hasElementCount) {
            return TfToken();
        }
        *arity = 3 * elementCount;
        return kVec3fArray;
    }
    if (hasElementCount) {
        return TfToken();
    }
    if (type == SdfValueTypeNames->Double) {
        *arity = 1;
        return kDouble;
    }
    if (type == SdfValueTypeNames->Float) {
        *arity = 1;
        return kFloat;
    }
    if (type == SdfValueTypeNames->Double3 ||
        type == SdfValueTypeNames->Vector3d ||
        type == SdfValueTypeNames->Point3d ||
        type == SdfValueTypeNames->Normal3d) {
        *arity = 3;
        return kVec3d;
    }
    if (type == SdfValueTypeNames->Float3 ||
        type == SdfValueTypeNames->Vector3f ||
        type == SdfValueTypeNames->Point3f ||
        type == SdfValueTypeNames->Normal3f) {
        *arity = 3;
        return kVec3f;
    }
    if (type == SdfValueTypeNames->Matrix4d) {
        *arity = 16;
        return kMatrix4d;
    }
    return TfToken();
}

// \p arity is the slot's double count; a float2[] slot reads it as
// consecutive (x, y) pairs and a vec3f[] slot as (x, y, z) triples.
VtValue
_PreviewValue(const TfToken &kind, const double *v, size_t arity)
{
    if (kind == kPreviewFloatArray) {
        VtFloatArray array(arity);
        float *out = array.data();
        for (size_t i = 0; i < arity; ++i)
            out[i] = float(v[i]);
        return VtValue(std::move(array));
    }
    if (kind == "float2[]") {
        VtVec2fArray array(arity / 2);
        GfVec2f *out = array.data();
        for (size_t i = 0; i < array.size(); ++i) {
            out[i] = GfVec2f(float(v[2 * i]), float(v[2 * i + 1]));
        }
        return VtValue(std::move(array));
    }
    if (kind == "vec3f[]") {
        VtVec3fArray array(arity / 3);
        GfVec3f *out = array.data();
        for (size_t i = 0; i < array.size(); ++i) {
            out[i] = GfVec3f(float(v[3 * i]), float(v[3 * i + 1]),
                             float(v[3 * i + 2]));
        }
        return VtValue(std::move(array));
    }
    if (kind == "double") {
        return VtValue(v[0]);
    }
    if (kind == "float") {
        return VtValue(float(v[0]));
    }
    if (kind == "vec3d") {
        return VtValue(GfVec3d(v[0], v[1], v[2]));
    }
    if (kind == "vec3f") {
        return VtValue(GfVec3f(float(v[0]), float(v[1]), float(v[2])));
    }
    if (kind == "matrix4d") {
        return VtValue(GfMatrix4d(v[0], v[1], v[2], v[3], v[4], v[5], v[6],
                                  v[7], v[8], v[9], v[10], v[11], v[12], v[13],
                                  v[14], v[15]));
    }
    return VtValue();
}

std::vector<std::string>
_SplitLines(const std::string &packed)
{
    std::vector<std::string> out;
    std::string current;
    for (const char c : packed) {
        if (c == '\n' || c == '\r') {
            if (!current.empty()) {
                out.push_back(current);
                current.clear();
            }
            continue;
        }
        current.push_back(c);
    }
    if (!current.empty()) {
        out.push_back(current);
    }
    return out;
}

} // namespace

int
RigExecImagingRegistry::BeginPreview(const std::string &packedAttributePaths)
{
    if (const Ptr routed = _Routed()) {
        return routed->BeginPreview(packedAttributePaths);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    // The active stage, else the one this context belongs to: a stage with
    // no rig still previews its Xforms, through this context's chains only.
    const UsdStageRefPtr stage = _PreviewStageLocked();
    if (!stage) {
        return -1;
    }
    std::vector<PreviewSlot> slots;
    size_t total = 0;
    for (const std::string &line : _SplitLines(packedAttributePaths)) {
        // `<path>` or `<path>\t<elementCount>`.
        const size_t tab = line.find('\t');
        const bool hasElementCount = tab != std::string::npos;
        size_t elementCount = 0;
        if (hasElementCount &&
            !_ParsePreviewElementCount(line.substr(tab + 1), &elementCount)) {
            return -1;
        }
        const std::string text = line.substr(0, tab);
        // Caller-supplied strings: ask before constructing, the same guard
        // SetWeightOverlay uses.
        if (!SdfPath::IsValidPathString(text)) {
            return -1;
        }
        const SdfPath path(text);
        if (!path.IsAbsolutePath() || !path.IsPropertyPath()) {
            return -1;
        }
        const UsdAttribute attribute = stage->GetAttributeAtPath(path);
        if (!attribute) {
            return -1;
        }
        PreviewSlot slot;
        slot.primPath = path.GetPrimPath();
        slot.attributeName = path.GetNameToken();
        slot.valueKind =
            _PreviewValueKind(attribute.GetTypeName(), hasElementCount,
                              elementCount, &slot.arity);
        if (slot.valueKind.IsEmpty()) {
            return -1;
        }
        // Which lane. An attribute inside an active rig previews through that
        // rig's evaluator; everything else is a transform of its own.
        // A playback rig offers no evaluator lane: its inputs are baked,
        // so an attribute under one falls through to the xform lane (or
        // is refused, when it is not an xform op either).
        for (const RigSession &session : _sessions) {
            if (!session.playback && slot.primPath.HasPrefix(session.rigPath)) {
                slot.rigPath = session.rigPath;
                break;
            }
        }
        if (slot.rigPath.IsEmpty() &&
            (hasElementCount ||
             !UsdGeomXformOp::IsXformOp(slot.attributeName))) {
            // Nothing to preview: the prim has no rig to re-run and this is
            // not part of its transform (an array never is). Refused rather
            // than silently ignored, because the caller would then be
            // sending values that change nothing and wondering why.
            return -1;
        }
        total += slot.arity;
        // The count is returned as an int.
        if (total > size_t(std::numeric_limits<int>::max())) {
            return -1;
        }
        slots.push_back(std::move(slot));
    }
    _previewSlots = std::move(slots);
    _previewStage = stage;
    _previewActive = true;
    return int(total);
}

bool
RigExecImagingRegistry::IsPreviewActive() const
{
    if (const Ptr routed = _Routed()) {
        return routed->IsPreviewActive();
    }
    return _previewActive;
}

bool
RigExecImagingRegistry::_ComposeXformDelta(
    const UsdPrim &prim, const std::map<TfToken, VtValue> &opValues,
    UsdGeomXformCache *cache, GfMatrix4d *delta) const
{
    const UsdGeomXformable xformable(prim);
    if (!xformable || !cache || !delta) {
        return false;
    }
    bool resetsXformStack = false;
    const std::vector<UsdGeomXformOp> ops =
        xformable.GetOrderedXformOps(&resetsXformStack);

    // The authored local transform and the previewed one, composed the same
    // way out of the same ops, so the only difference between them is the
    // overridden values. Composing op by op rather than calling
    // GetLocalTransformation twice is what lets an override stand in for one
    // op's value without being authored anywhere.
    GfMatrix4d authored(1.0), previewed(1.0);
    const UsdTimeCode time = cache->GetTime();
    for (const UsdGeomXformOp &op : ops) {
        VtValue value;
        const bool hasAuthoredValue = op.GetAttr().Get(&value, time);
        if (hasAuthoredValue) {
            authored =
                op.GetOpTransform(op.GetOpType(), value, op.IsInverseOp()) *
                authored;
        }
        // An inverse op shares its attribute with the forward op; the
        // !invert! marker belongs to the order token, not the value key.
        const auto found = opValues.find(op.GetAttr().GetName());
        // A newly created ordered op has no authored value yet. It is an
        // identity in the authored transform but its preview still applies.
        if (found == opValues.end() && !hasAuthoredValue) {
            continue;
        }
        const VtValue &previewValue =
            found == opValues.end() ? value : found->second;
        previewed =
            op.GetOpTransform(op.GetOpType(), previewValue, op.IsInverseOp()) *
            previewed;
    }

    // delta = parentToWorld^-1 . local^-1 . local' . parentToWorld
    // which is exactly xform^-1 . xform', the quantity
    // RigExecXformOverrideSceneIndex post-multiplies. A prim that resets the
    // xform stack has no parent contribution, and then the conjugation
    // collapses to local^-1 . local' on its own.
    const GfMatrix4d parentToWorld =
        resetsXformStack ? GfMatrix4d(1.0)
                         : cache->GetParentToWorldTransform(prim);
    const GfMatrix4d parentInverse = parentToWorld.GetInverse();
    *delta = parentInverse * authored.GetInverse() * previewed * parentToWorld;
    return true;
}

bool
RigExecImagingRegistry::_ResolvePreviewSample(
    const double *values, size_t count,
    std::map<SdfPath, std::vector<RigExecValueOverride>> *byRig,
    std::map<SdfPath, GfMatrix4d> *xformDeltas) const
{
    size_t offset = 0;
    std::map<SdfPath, std::map<TfToken, VtValue>> xformOps;
    for (const PreviewSlot &slot : _previewSlots) {
        if (offset + slot.arity > count) {
            return false;
        }
        const VtValue value =
            _PreviewValue(slot.valueKind, values + offset, slot.arity);
        offset += slot.arity;
        if (value.IsEmpty()) {
            return false;
        }
        if (!slot.rigPath.IsEmpty()) {
            (*byRig)[slot.rigPath].push_back(RigExecValueOverride{
                slot.primPath, TfToken(), slot.attributeName, value});
        } else {
            xformOps[slot.primPath][slot.attributeName] = value;
        }
    }
    if (offset != count) {
        return false;
    }
    if (!xformOps.empty()) {
        UsdGeomXformCache cache(_lastTime);
        for (const auto &[primPath, opValues] : xformOps) {
            const UsdStageRefPtr stage(_previewStage);
            const UsdPrim prim =
                stage ? stage->GetPrimAtPath(primPath) : UsdPrim();
            GfMatrix4d delta(1.0);
            if (!prim || !_ComposeXformDelta(prim, opValues, &cache, &delta)) {
                return false;
            }
            (*xformDeltas)[primPath] = delta;
        }
    }
    return true;
}

void
RigExecImagingRegistry::_SetChainXformDeltas(
    const std::map<SdfPath, GfMatrix4d> &deltas)
{
    _previewXformDeltas = deltas;
    // This context's chains only: a preview on one stage never moves the
    // same path in another stage's viewport. (Sent under _mutex, as
    // before; the xform dirties never intersect the time trigger, so the
    // results index does not re-enter.)
    for (const Chain &chain : _BoundChains()) {
        if (chain.xforms) {
            chain.xforms->SetWorldDeltas(deltas);
        }
    }
}

bool
RigExecImagingRegistry::UpdatePreview(const double *values, size_t count)
{
    if (const Ptr routed = _Routed()) {
        return routed->UpdatePreview(values, count);
    }
    UsdTimeCode time;
    bool needsRigPublish = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_previewActive || !_previewStage || (!values && count)) {
            return false;
        }
        std::map<SdfPath, std::vector<RigExecValueOverride>> byRig;
        std::map<SdfPath, GfMatrix4d> xformDeltas;
        if (!_ResolvePreviewSample(values, count, &byRig, &xformDeltas)) {
            return false;
        }
        // The xform lane needs no evaluation at all: the delta goes straight
        // into the chains and the dirty notice it sends is the whole update.
        _SetChainXformDeltas(xformDeltas);

        // Every active rig is told, including the ones with no slots this
        // sample: a rig that was being previewed and no longer is has to stop.
        for (RigSession &session : _sessions) {
            const auto found = byRig.find(session.rigPath);
            if (found == byRig.end()) {
                if (session.bridge) {
                    session.bridge->ClearInteractiveOverrides();
                }
            } else if (session.bridge) {
                session.bridge->SetInteractiveOverrides(found->second);
                needsRigPublish = true;
            }
            // Any new edit bumps the generation and cancels the rig's
            // in-flight jobs. A job sampled before this tick would land
            // under an unreachable key anyway (the digest folds the
            // overrides), but it would still burn a worker on a pose no
            // lookup can reach; the commit trigger re-enqueues fresh.
            _CancelGenerationLocked(session.rigPath);
            session.dirty = true;
            session.settleOverrides.clear();
        }
        time = _lastTime;
    }
    // Republished OUTSIDE the lock: SetTime takes the same non-recursive mutex
    // (the rule _OnObjectsChanged and SetWeightOverlay both follow).
    // Only when a rig is involved. An xform-lane drag has already dirtied what
    // it changed, and evaluating every active rig to redraw a prim no rig
    // drives would make the cheap lane pay for the expensive one.
    return needsRigPublish ? SetTime(time) : true;
}

bool
RigExecImagingRegistry::WriteProfileSummary(const std::string &path)
{
    if (const Ptr routed = _Routed()) {
        return routed->WriteProfileSummary(path);
    }
    std::lock_guard<std::mutex> lock(_mutex);
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "rig\tname\tcategory\tcount\ttotal_ms\tmax_ms\n";
    for (const RigSession &session : _sessions) {
        // Playback records no phases, so it contributes no rows.
        if (session.playback) {
            continue;
        }
        const RigExecRigEvaluator &evaluator = session.bridge->GetEvaluator();
        for (const RigExecProfileSummaryRow &row :
             session.bridge->GetProfiler().Summarize()) {
            out << session.rigPath.GetString() << '\t' << row.name << '\t'
                << row.category << '\t' << row.count << '\t'
                << (row.totalUs / 1000.0) << '\t' << (row.maxUs / 1000.0)
                << '\n';
        }
        session.bridge->MutableProfiler()->Clear();
    }
    return true;
}

bool
RigExecImagingRegistry::EndPreview(bool publish)
{
    if (const Ptr routed = _Routed()) {
        return routed->EndPreview(publish);
    }
    UsdTimeCode time;
    bool hadRigOverrides = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _previewSlots.clear();
        _previewActive = false;
        _previewStage = UsdStageWeakPtr();
        _SetChainXformDeltas({});
        for (RigSession &session : _sessions) {
            session.settleOverrides.clear();
            if (session.bridge) {
                // Clean means the last publish evaluated these overrides at
                // _lastTime, so a commit of the same values changes nothing
                // on screen.
                if (!publish && !session.dirty && !session.readRootsDirty &&
                    !_readRootsDirty) {
                    session.settleOverrides =
                        session.bridge->GetInteractiveOverrides();
                }
                session.bridge->ClearInteractiveOverrides();
            }
            session.dirty = true;
            hadRigOverrides = true;
        }
        time = _lastTime;
    }
    // Republished so the authored rig is what is drawn again. The caller
    // authors its committed values separately; when it has already done so,
    // this generation is the committed one and the artist sees no flicker,
    // and when it has not -- an aborted drag -- this is the rig going back.
    // A caller about to commit asks for no republish: the sessions are dirty
    // and the commit's notice evaluates them, once, from the stage.
    return hadRigOverrides && publish ? SetTime(time) : true;
}

bool
RigExecImagingRegistry::_NoticeSettlesPreview(
    const RigSession &session, const UsdNotice::ObjectsChanged &notice) const
{
    if (!_stage || _readRootsDirty || session.readRootsDirty ||
        !notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    std::set<SdfPath> stashed;
    for (const RigExecValueOverride &entry : session.settleOverrides) {
        if (entry.attribute.IsEmpty()) {
            return false;
        }
        stashed.insert(entry.prim.AppendProperty(entry.attribute));
    }
    // First opinions create inert ancestor overs, reported without fields.
    // Metadata edits and prim resyncs still require evaluation.
    const auto inertAncestor = [&](const SdfPath &path) {
        return path.IsPrimPath() && notice.GetChangedFields(path).empty() &&
               std::any_of(stashed.begin(), stashed.end(),
                           [&](const SdfPath &property) {
                               return property.HasPrefix(path);
                           });
    };
    const auto valueEdit = [&](const SdfPath &path) {
        if (!stashed.count(path)) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != "default" && field != "timeSamples" &&
                field != "spline" && field != "typeName") {
                return false;
            }
        }
        return true;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!valueEdit(path)) {
            return false;
        }
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (!inertAncestor(path) && !valueEdit(path)) {
            return false;
        }
    }
    // And the stage now answers every withdrawn value exactly, including
    // the ones this notice did not touch -- on attributes whose authored
    // value is what the rig reads. A connection overrules an authored
    // value, while the override stood in for the connection's answer. A
    // property chain revises an authored value exactly as it revised the
    // override, which was its base.
    if (!session.bridge) {
        return false;
    }
    for (const RigExecValueOverride &entry : session.settleOverrides) {
        const UsdAttribute attribute = _stage->GetAttributeAtPath(
            entry.prim.AppendProperty(entry.attribute));
        VtValue authored;
        if (!attribute || attribute.HasAuthoredConnections() ||
            !attribute.Get(&authored, _lastTime) || authored != entry.value) {
            return false;
        }
    }
    return true;
}

void
RigExecImagingRegistry::_OnGuideOnlyObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &sender,
    const std::set<SdfPath> &guideRoots, const SdfPath &request,
    UsdTimeCode guideTime, bool automatic, bool retryOnValues)
{
    // Only edits inside or above a guide asset can change what it draws or
    // complete its missing inputs.
    const auto touchesGuides = [&guideRoots](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        for (const SdfPath &root : guideRoots) {
            if (primPath.HasPrefix(root) || root.HasPrefix(primPath)) {
                return true;
            }
        }
        return false;
    };
    bool relevant = false;
    bool structural = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (touchesGuides(path)) {
            relevant = structural = true;
            break;
        }
    }
    if (!structural) {
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (!touchesGuides(path)) {
                continue;
            }
            relevant = true;
            // A value edit to a display attribute (visibility, transform,
            // rest placement, guide styling) cannot complete a rig that
            // failed to compile; any other edit (targets, connections,
            // metadata, other values) can.
            bool displayValue = path.IsPropertyPath() && !retryOnValues;
            if (displayValue) {
                const TfToken &name = path.GetNameToken();
                displayValue =
                    name == UsdGeomTokens->visibility ||
                    name == UsdGeomTokens->purpose ||
                    name == UsdGeomTokens->xformOpOrder ||
                    TfStringStartsWith(name.GetString(), "xformOp:") ||
                    TfStringStartsWith(name.GetString(), "rest:") ||
                    TfStringStartsWith(name.GetString(), "guide:");
            }
            if (displayValue) {
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field != SdfFieldKeys->Default &&
                        field != SdfFieldKeys->TimeSamples &&
                        field != SdfFieldKeys->Spline) {
                        displayValue = false;
                        break;
                    }
                }
            }
            if (!displayValue) {
                structural = true;
                break;
            }
        }
    }
    if (!relevant) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _authoredGuidesDirty = true;
    }
    // Completing missing inputs can restore the ordinary evaluator. A
    // failed retry, or a display-only edit, republishes authored guides
    // with the edited values.
    if (structural) {
        std::vector<std::string> errors;
        if (_Activate(UsdStageRefPtr(sender), request, guideTime, &errors,
                      automatic)) {
            return;
        }
    }
    SetTime(guideTime);
}

void
RigExecImagingRegistry::_OnObjectsChanged(
    const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &sender)
{
    bool authoredOnly = false;
    bool automatic = false;
    bool retryOnValues = false;
    UsdTimeCode guideTime;
    SdfPath request;
    std::set<SdfPath> guideRoots;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        authoredOnly = _sessions.empty() && !_authoredGuides.empty() &&
                       sender == UsdStageWeakPtr(_stage);
        if (authoredOnly) {
            automatic = _autoActivated;
            retryOnValues = _authoredGuidesRetryOnValues;
            guideTime = _lastTime;
            request = _authoredGuidesRequest;
            for (const auto &entry : _authoredGuides) {
                guideRoots.insert(entry.first.GetParentPath());
            }
        }
    }
    if (authoredOnly) {
        _OnGuideOnlyObjectsChanged(notice, sender, guideRoots, request,
                                   guideTime, automatic, retryOnValues);
        return;
    }
    RigExecTapSet::PrepareStageChange(UsdStageRefPtr(sender), notice);
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        scheduler = _scheduler;
    }
    // Retiring cache rows and fencing their running jobs is atomic with
    // worker cache/index publication. The fence precedes the registry
    // lock, and is released before publishing any scene-index notices.
    std::unique_lock<std::mutex> fenceLock;
    if (scheduler) {
        fenceLock = std::unique_lock<std::mutex>(scheduler->FenceMutex());
    }
    std::set<SdfPath> readRoots;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_sessions.empty() || _assetRoots.empty() ||
            sender != UsdStageWeakPtr(_stage)) {
            return;
        }
        readRoots = _readRoots;
        // Pose-only keys include every stage edit. Retire their display
        // rows even when this notice misses the rig's known input roots.
        // A playback re-reads its static inputs after every edit too: one
        // it binds may lie outside them.
        for (RigSession &session : _sessions) {
            if (!session.playback && session.bridge) {
                session.bridge->RetirePoseOnlyCacheEntries();
            }
            if (session.playback) {
                session.playback->NoteStageChanged();
            }
        }
    }
    // Any edit touching the asset or its transitive reads can factor into
    // the final frame (solvers, joints, movers, controls, weights, driver geometry,
    // guide styling): re-evaluate at the current time. The evaluator's
    // compiled rig signature turns structural edits into recompiles; value edits
    // flow through exec invalidation on the shared layers.
    auto touchesInput = [&readRoots](const SdfPath &path) {
        const SdfPath primPath = path.GetPrimPath();
        for (const SdfPath &root : readRoots) {
            if (primPath.HasPrefix(root) || root.HasPrefix(primPath)) {
                return true;
            }
        }
        return false;
    };
    bool relevant = false;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (touchesInput(path)) {
            relevant = true;
            break;
        }
    }
    if (!relevant) {
        for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
            if (touchesInput(path)) {
                relevant = true;
                break;
            }
        }
    }
    if (!relevant) {
        // Resolved-asset resyncs: the evaluator honors them (they break
        // the avar-only fast path and hit the capture index), so the
        // registry scan reads them too.
        for (const SdfPath &path :
             notice.GetResolvedAssetPathsResyncedPaths()) {
            if (touchesInput(path)) {
                relevant = true;
                break;
            }
        }
    }
    if (relevant) {
        UsdTimeCode time;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // A value-only edit keeps the cached dependency regions. Resyncs
            // and connection/relationship edits may introduce a new external
            // input even when the evaluator's compiled rig stays unchanged.
            // A property resync that names only a value (a spec created by
            // its first authored value, stating `custom` and `variability`
            // with it, or removed by the undo of it) is a value edit here
            // too: a new connection or target would arrive
            // as its own field, and a removed one leaves the cached regions
            // a superset of what is read, which is safe. The one input a
            // removal can reveal is a weaker layer's connection the removed
            // spec was hiding, so a property still connected afterwards
            // counts as structure.
            for (const SdfPath &path : notice.GetResyncedPaths()) {
                if (_readRootsDirty) {
                    break;
                }
                if (!path.IsPropertyPath()) {
                    _readRootsDirty = true;
                    break;
                }
                const UsdAttribute attribute =
                    _stage ? _stage->GetAttributeAtPath(path) : UsdAttribute();
                const UsdRelationship relationship =
                    _stage ? _stage->GetRelationshipAtPath(path)
                           : UsdRelationship();
                if ((attribute && attribute.HasAuthoredConnections()) ||
                    (relationship && relationship.HasAuthoredTargets())) {
                    _readRootsDirty = true;
                    break;
                }
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field != "typeName" && field != "default" &&
                        field != "timeSamples" && field != "spline" &&
                        field != "custom" && field != "variability") {
                        _readRootsDirty = true;
                        break;
                    }
                }
            }
            for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
                for (const TfToken &field : notice.GetChangedFields(path)) {
                    if (field == "connectionPaths" || field == "targetPaths") {
                        _readRootsDirty = true;
                    }
                }
            }
            // A resolved asset arriving from a new file can reveal a new
            // external input the cached regions never named.
            if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
                _readRootsDirty = true;
            }
            for (RigSession &session : _sessions) {
                const auto affects = [&session](const SdfPath &path) {
                    const SdfPath prim = path.GetPrimPath();
                    for (const SdfPath &root : session.readRoots) {
                        if (prim.HasPrefix(root) || root.HasPrefix(prim))
                            return true;
                    }
                    return false;
                };
                bool affected = false;
                for (const SdfPath &path : notice.GetResyncedPaths())
                    affected = affected || affects(path);
                for (const SdfPath &path : notice.GetChangedInfoOnlyPaths())
                    affected = affected || affects(path);
                for (const SdfPath &path :
                     notice.GetResolvedAssetPathsResyncedPaths())
                    affected = affected || affects(path);
                // A playback session never dirties: the binary is static,
                // so no stage edit changes what it publishes. Picking up
                // a new asset (or a new file behind it) needs a
                // re-activation.
                if (affected && !session.playback) {
                    RigExecProfileScope noticeScope(
                        session.bridge->MutableProfiler(), "Notice.Imaging",
                        "notice");
                    // A commit of the pose on screen: the published
                    // generation is already the stage's answer.
                    const bool settled = !session.settleOverrides.empty() &&
                                         _NoticeSettlesPreview(session, notice);
                    if (!settled) {
                        session.settleOverrides.clear();
                    }
                    session.dirty = !settled;
                    session.readRootsDirty =
                        session.readRootsDirty || _readRootsDirty;
                    // The bridge caches authored guide styling (shape, scale,
                    // colour, purpose...) across generations; an edit that
                    // reaches the rig is the only thing that can move it,
                    // and only the entries it names are dropped.
                    session.bridge->NoteGuideEdits(notice);
                    // The live sampler's pinned chain bindings, on the
                    // same footing: an edit under a chain mover moves a
                    // chain without moving the compiled rig signature.
                    session.bridge->NoteChainEdits(notice);
                    // Outcome-driven retirement: the
                    // evaluator classifies this notice (patched plus
                    // paths, routed plus the paths it reads, stamp-bumped,
                    // or stale) through a read-only query, so the verdict
                    // holds whatever order the two notice handlers ran in.
                    // Patch/routed/stamp edits partition
                    // completions by entry provenance -- clean entries
                    // stand or carry, dirty ones retire -- and purge
                    // affected times only, without bumping the
                    // generation, so other times' queued and running jobs
                    // still publish; a stale program (or no program)
                    // keeps the global cancel plus the capture-index
                    // compiled rig retirement.
                    std::vector<SdfPath> patchedPaths;
                    const RigExecNoticeDisposition disposition =
                        session.bridge->GetEvaluator()
                            .ClassifyNoticeDisposition(notice, &patchedPaths);
                    const auto namesRig = [&session](const SdfPath &path) {
                        return path.HasPrefix(session.rigPath) ||
                               session.rigPath.HasPrefix(path);
                    };
                    const auto resynced = notice.GetResyncedPaths();
                    const auto changed = notice.GetChangedInfoOnlyPaths();
                    const bool rigStructureChanged =
                        session.readRootsDirty &&
                        (std::any_of(resynced.begin(), resynced.end(),
                                     namesRig) ||
                         std::any_of(changed.begin(), changed.end(), namesRig));
                    if (rigStructureChanged) {
                        // A removed binding can disappear from the capture
                        // index before this handler sees its notice.
                        _CancelGenerationLocked(session.rigPath);
                        session.bridge->ClearFrameCache();
                    } else if (disposition ==
                                   RigExecNoticeDisposition::Patched ||
                               disposition ==
                                   RigExecNoticeDisposition::Edited ||
                               disposition ==
                                   RigExecNoticeDisposition::StampBumped) {
                        _RetireAffectedTimesLocked(&session, notice,
                                                   disposition, patchedPaths);
                    } else {
                        // A new edit bumps the warming generation and
                        // cancels in-flight jobs for the rig; the commit
                        // trigger re-enqueues fresh jobs under the new
                        // token.
                        _CancelGenerationLocked(session.rigPath);
                        // The baked capture index retires cached frames
                        // eagerly: a notice that hits it drops the compiled rig
                        // now, while its half still names it (the
                        // evaluator rebuilds lazily); a miss drops
                        // nothing and digest reachability decides.
                        session.bridge->NoteCaptureIndex(notice);
                    }
                }
            }
            time = _lastTime;
        }
        if (fenceLock.owns_lock()) {
            fenceLock.unlock();
        }
        SetTime(time);
    }
}

void
RigExecImagingRegistry::Deactivate()
{
    if (const Ptr routed = _Routed()) {
        routed->Deactivate();
        return;
    }
    _Deactivate(/* byHost = */ true);
}

bool
RigExecImagingRegistry::IsAutoActivated() const
{
    if (const Ptr routed = _Routed()) {
        return routed->IsAutoActivated();
    }
    std::lock_guard<std::mutex> lock(_mutex);
    return _autoActivated && (!_sessions.empty() || !_authoredGuides.empty());
}

void
RigExecImagingRegistry::_Deactivate(bool byHost)
{
    if (_legacyHandle) {
        return;
    }
    // Stopped outside every lock, below: the scheduler's destructor joins
    // its workers, and a fenced clear may be holding its fence while it
    // waits for _mutex.
    std::shared_ptr<RigExecBackgroundScheduler> scheduler;
    // unique_lock: the broadcast below runs unlocked (see Activate), because
    // the notices it sends re-enter this registry through the results
    // index's time trigger.
    std::unique_lock<std::mutex> lock(_mutex);
    if (!byHost) {
        // The library releasing an automatic activation: only while it is
        // still automatic (an explicit Activate took it over otherwise) and
        // no chain is bound any more. Tested under _mutex -- the directory
        // lock nests inside it -- so a chain binding meanwhile either shows
        // up here or adopts the cleared state and re-activates through its
        // own population.
        if (!_autoActivated || (_sessions.empty() && _authoredGuides.empty()) ||
            !_BoundChains().empty()) {
            return;
        }
    }
    TfNotice::Revoke(_changeKey);
    _changeKey = TfNotice::Key();
    _assetRoots.clear();
    _readRoots.clear();
    _readRootsDirty = false;
    _generatedScopes.clear();
    for (const RigSession &session : _sessions) {
        _warmIndex->ResetRig(session.rigPath);
        if (_scheduler) {
            _scheduler->ResetRig(session.rigPath);
        }
    }
    _sessions.clear();
    _authoredGuides.clear();
    _authoredGuidesDirty = false;
    _authoredGuidesRequest = SdfPath();
    _authoredGuidesRetryOnValues = false;
    _active = false;
    // A library release is not a host decision: a later engine on the same
    // stage activates it again (EnsureActivated). A host deactivation of an
    // inactive context still counts -- it keeps the automatic path off.
    if (byHost) {
        _deactivatedByHost = true;
    }
    _autoActivated = false;
    _stage.Reset();
    // A context without an active rig owns no threads.
    scheduler = std::move(_scheduler);
    {
        // Nested _mutex -> _notedMutex: the documented lock order.
        std::lock_guard<std::mutex> notedLock(_notedMutex);
        _notedRigRoots.clear();
    }
    _publishedCompiledRigId = 0;
    _publishedCompiledRig.reset();
    for (const Chain &chain : _BoundChains()) {
        if (chain.pruning) {
            chain.pruning->SetOwnedScopes({});
        }
    }
    RigExecImagingBridge::PublishResult cleared;
    cleared.ok = true;
    cleared.dirtied = _store->Publish(nullptr);
    lock.unlock();
    _Broadcast(cleared);
    if (scheduler) {
        // The stopped scheduler's totals stay part of this context's
        // warming counters (GetBackgroundStats): the cache strip's repaint
        // gate and its "completed" label must not fall back to zero on
        // every re-activation. Read as late as possible, unlocked (the
        // scheduler's own lock is a leaf); completions of jobs still
        // running past this point are not counted.
        const RigExecBackgroundSchedulerStats stats = scheduler->Stats();
        std::lock_guard<std::mutex> relock(_mutex);
        _AddSchedulerStats(&_retiredStats, stats, /* gauges = */ false);
    }
    // The last holder joins the workers; running jobs finish, queued ones
    // drop (RigExecBackgroundScheduler's destructor).
    scheduler.reset();
}

// ---------------------------------------------------------------------------
// Frames on demand (frameTable.h, docs/specs/frames-on-demand.md).

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
        _HandOffUpstreamLocked(&session, at);
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
    if (const Ptr routed = _Routed()) {
        return routed->ServiceFrameDemand(maxFrames, allowFallback);
    }
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
    if (const Ptr routed = _Routed()) {
        return routed->HasFrameDemand();
    }
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

void
RigExecImagingRegistry::_Broadcast(
    const RigExecImagingBridge::PublishResult &result)
{
    if (!result.ok) {
        return;
    }
    // The chain list is snapshotted under a short lock; the sends go out
    // WITHOUT it. Scene indices call back into this registry -- the results
    // index's time trigger evaluates through IsActiveRigRoot/SetTime -- so
    // every caller unlocks first (see Activate), and this function never
    // assumes the lock is held. A chain whose graph dies between the snapshot
    // and its send simply fails its weak-pointer check and is pruned on the
    // next broadcast.
    // This context's chains only, snapshotted under the directory's leaf
    // lock (which also prunes chains whose graphs were destroyed).
    const std::vector<Chain> chains = _BoundChains();
    for (const Chain &chain : chains) {
        if (result.compiledRig && chain.binding) {
            chain.binding->SetCompiledRigBinding(result.compiledRig);
        }
        if (chain.results) {
            chain.results->NotifyGenerationPublished(result.dirtied);
        }
    }
}

} // namespace rigExec
