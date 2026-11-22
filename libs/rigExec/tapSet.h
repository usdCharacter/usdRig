// RigExec typed extraction API.
// Backend-neutral addresses resolve through the USD implementation to
// ExecUsdValueKey batches; snapshots copy immutable values before crossing
// threads.
#ifndef RIGEXEC_TAP_SET_H
#define RIGEXEC_TAP_SET_H

#include "valueAddress.h"

#include "pxr/pxr.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usd/notice.h"

#include <atomic>
#include <memory>
#include <vector>

PXR_NAMESPACE_OPEN_SCOPE
class ExecUsdSystem;
class ExecUsdRequest;
PXR_NAMESPACE_CLOSE_SCOPE

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecTapContext;

/// Typed batched extraction over one ExecUsdSystem.
///
/// Usage: Add() every address, Prepare() once, then Evaluate() per time.
/// Adding taps after Prepare() rebuilds the request on the next Prepare().
class RigExecTapSet {
public:
    explicit RigExecTapSet(const UsdStageRefPtr &stage);
    ~RigExecTapSet();

    RigExecTapSet(const RigExecTapSet &) = delete;
    RigExecTapSet &operator=(const RigExecTapSet &) = delete;

    /// Registers an address; returns its stable tap id.
    RigExecTapId Add(const RigExecValueAddress &address);

    /// Registers a public canonical address whose value is produced by a
    /// compiler-private provider (a generated chain head). The private
    /// path is used only to build the internal value key; it is never
    /// part of the public address.
    RigExecTapId AddResolved(const RigExecValueAddress &publicAddress,
                             const SdfPath &privateProvider);

    /// How many taps stand. Tap ids are dense from zero, so the live
    /// operation graph enumerates every address through this.
    size_t GetTapCount() const
    {
        return _addresses.size();
    }

    /// The public canonical address of a tap.
    const RigExecValueAddress &GetAddress(RigExecTapId tap) const;

    /// Builds and front-loads the batched request schedule. Returns
    /// false when the request could not be built valid.
    bool Prepare();

    /// Populates the shared executor at \p time without copying values out.
    ///
    /// Every pull in this evaluator carries overrides, and an override-bearing
    /// compute runs in a throwaway sub-executor seeded from the main one: with
    /// nothing ever computed into the main executor it starts empty every
    /// time, so the whole network is recomputed on every call. One plain
    /// Compute() first leaves the shared cache warm and the overridden pull
    /// only has to recompute what the overrides actually reach.
    void Warm(UsdTimeCode time);

    /// Serialized ChangeTime + Compute + immutable value copy.
    RigExecSnapshot Evaluate(UsdTimeCode time);

    /// Evaluates with \p overrides standing in for the named computations.
    ///
    /// Overrides apply to this call only; they do not persist into later
    /// Evaluate calls or affect cached values (ExecUsdSystem contract).
    /// dropped counts missing prim/attribute addresses during key construction;
    /// it cannot report whether an existing key lies in the request's cone.
    RigExecSnapshot Evaluate(UsdTimeCode time,
                             const std::vector<RigExecValueOverride> &overrides,
                             size_t *dropped = nullptr);

    /// Returns and clears the dirty flag raised by invalidation callbacks.
    bool ConsumeDirty()
    {
        return _dirty.exchange(false);
    }

    ExecUsdSystem *GetSystem();

    /// Retire requests before stock Esf processes a removed prim. Consumers
    /// that evaluate synchronously inside a USD notice call this first.
    static void PrepareStageChange(const UsdStageRefPtr &stage,
                                   const UsdNotice::ObjectsChanged &notice);

private:
    friend class RigExecTapContext;
    UsdStageRefPtr _stage;
    std::shared_ptr<RigExecTapContext> _context;
    std::unique_ptr<ExecUsdRequest> _request;
    std::vector<RigExecValueAddress> _addresses;
    /// Compiler-private resolutions parallel to _addresses; an empty path
    /// means the public address resolves directly.
    std::vector<SdfPath> _resolutions;
    std::atomic<bool> _dirty{false};
    bool _prepared = false;
    /// Diagnostic only (TF_DEBUG=RIGEXEC_TAP_TIMING): how many times this
    /// tap set has rebuilt its request.
    size_t _prepareCount = 0;
};

} // namespace rigExec

#endif // RIGEXEC_TAP_SET_H
