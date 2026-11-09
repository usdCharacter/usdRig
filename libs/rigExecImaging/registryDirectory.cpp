// RigExec imaging directory and chain binding: the process-level map from
// stage to imaging context, the monotonic context keys, and the chain
// registrations. The directory is ONE process object: Get() holds a
// function-local static and no other TU declares or instantiates it. Its
// mutex is a leaf lock -- nothing enters or destroys a context while it is
// held -- so every prune hands the released contexts to a vector declared
// before the guard, destroyed only after the unlock. A raw-pointer stage
// match is always re-compared against the weak identity. The _directoryAlive
// atomic (constant-initialized, trivially destructible) keeps a scene index
// released during static destruction from resurrecting or touching a
// destroyed directory. Chain registration, rebinding and the automatic
// last-chain release live here with the directory records they share.
#include "registry.h"
#include "rigExecGpu/gpuDeformExecutor.h"

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

// The directory: stage -> imaging context (registry.h, "PER-STAGE CONTEXTS").

namespace {

// Whether the directory (a function-local static) exists right now. A
// results scene index released during static destruction -- an engine a
// host leaks until exit -- must not resurrect or touch a destroyed
// directory. Constant-initialized and trivially destructible, so it stays
// readable after the directory is gone.
std::atomic<bool> _directoryAlive{false};

} // namespace

/// Process-level map from a stage to its imaging context, the chain
/// registrations.
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
                result = std::make_shared<Ctx>(Ctx::_FactoryTag{0}, key,
                                             stage);
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
    RigExecImagingDirectory()
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
};

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

size_t
RigExecImagingRegistry::ContextCount()
{
    return RigExecImagingDirectory::Get().ContextCount();
}

RigExecImagingRegistry::RigExecImagingRegistry(_FactoryTag, uint64_t key,
                                               const UsdStageRefPtr &stage)
    : _key(key), _boundStage(stage),
      _store(std::make_shared<RigExecSnapshotStore>()),
      _warmIndex(std::make_shared<RigExecWarmFrameIndex>())
{
    // RIGEXEC_GPU_DEFORM=readback deforms on the GPU executor; the device
    // starts on its first run, not here.
    // TODO: investigate whether these runner registrations belong at
    // activation, so a context that never deforms or solves on the GPU
    // pays nothing at construction.
    RigExecGpuRegisterDeformRunner();
    // Cloth solves on the GPU by default (RIGEXEC_GPU_CLOTH=0 keeps it on
    // the CPU); the device starts on the first solve.
    RigExecGpuRegisterClothRunner();
    // No scheduler: a context without an active rig owns no threads. The
    // first activation starts one (see Activate).
}

RigExecImagingRegistry::~RigExecImagingRegistry() = default;

const std::shared_ptr<RigExecSnapshotStore> &
RigExecImagingRegistry::GetStore() const
{
    return _store;
}

bool
RigExecImagingRegistry::IsActive() const
{
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

// Seeds a chain that just bound to no live context: fresh, not
// stage-specific state, with every handle null-checked before use.
void
RigExecImagingRegistry::_UpdateChainUnbound(const Chain &chain, uint64_t key)
{
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
        _UpdateChainUnbound(chain, key);
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

} // namespace rigExec
