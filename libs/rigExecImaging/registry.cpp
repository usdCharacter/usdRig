// RigExec imaging registry: the imaging contexts and the directory that
// maps stages to them. The host-facing C surface is in registryCapi.cpp.
#include "registry.h"
#include "registryInternal.h"
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

} // namespace rigExec
