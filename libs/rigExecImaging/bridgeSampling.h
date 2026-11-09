// Chain-sample input bindings and live input sampling owned by the imaging
// bridge: the live sampler's chain bindings pinned for the current compiled
// rig (rebound on a recompile or on a stage notice that could move a chain),
// the sorted edit-guard lists behind that notice decision, and the
// sample-support memo the frame-cache lanes consult. One per bridge,
// constructed after the evaluator and destroyed before it. The component
// holds no store, frame cache, warm index, registry state or bridge
// back-pointer: it reads the evaluator through the reference the bridge
// bound at construction, and the bridge passes the effective evaluation
// overrides and upstream values on every sampling call. The upstream
// setter, the display/evaluation override storage and the evaluator
// override translation stay on the facade.
#ifndef RIGEXEC_IMAGING_BRIDGE_SAMPLING_H
#define RIGEXEC_IMAGING_BRIDGE_SAMPLING_H

#include "rigExec/frozenContext.h"

#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/notice.h"

PXR_NAMESPACE_USING_DIRECTIVE

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

class RigExecBakedProgram;

/// The bridge-owned live-sampling component. UI thread only, like the
/// bridge: bindings hold live stage handles.
class RigExecBridgeSampling {
public:
    /// Borrows the owner-thread evaluator the facade owns. The evaluator
    /// outlives this component: the facade builds it first and destroys
    /// its components before it.
    explicit RigExecBridgeSampling(const RigExecRigEvaluator &evaluator);

    /// Drops the pinned chain bindings when \p notice could move a chain
    /// without moving the compiled rig signature: any resync, or an edit
    /// under a chain mover, one of its weight objects, or its target's
    /// prim.
    void NoteChainEdits(const UsdNotice::ObjectsChanged &notice);

    /// The rig's chain bindings, pinned for the current compiled rig and
    /// kept current by NoteChainEdits -- the ones the live sampler uses --
    /// rebound first when a notice or a recompile dropped them. Null when
    /// the chains cannot be bound; otherwise the pointer is into storage
    /// owned here, valid until the next rebind -- a holder that needs them
    /// longer copies them. \p generation, when given, receives a counter
    /// that moves on every rebind, so a holder of a copy (the warming
    /// session) refreshes it only when it actually changed instead of
    /// re-verifying it after every edit. UI thread only.
    const RigExecChainSampleBindings *
    AcquireChainBindings(uint64_t *generation = nullptr) const;

    /// Samples one frame's input vector for the live lookup and memoize
    /// paths: through the pinned chain bindings, falling back to the
    /// self-binding sampler whenever the pinned route declines (a bind
    /// that failed, an override it cannot place, a hook that refused).
    /// The vector is the plain sampler's, element for element; only the
    /// cost differs: the self-binding sampler re-discovers and re-orders
    /// every property chain on each call. \p overrides and \p upstream are
    /// the facade's effective evaluation overrides and upstream values,
    /// borrowed for the call: the upstream values ride the vector
    /// (RigExecFrameInputs::upstream), admitted as live admits them, so
    /// every digest of it folds them.
    bool SampleLive(UsdTimeCode time,
                    const std::vector<RigExecValueOverride> &overrides,
                    const std::vector<RigExecUpstreamValue> &upstream,
                    RigExecFrameInputs *out) const;

    /// Shares the frozen executor's input capability check: whether the
    /// evaluator's program freezes, memoized by program object, compiled
    /// rig signature and stage-structure serial -- whether the program
    /// freezes is a question about connections and prims, which no value
    /// edit moves. Unknown or unsupported operations still cache complete
    /// live poses.
    bool CanSampleCacheInputs();

    /// Drops the sample-support memo. Called by the facade's
    /// ClearFrameCache; the pinned bindings survive a frame-cache clear.
    void InvalidateSampleSupport();

private:
    /// Binds the chain bindings when they are not current; true when they
    /// are usable afterwards.
    bool _EnsureChainBindings() const;

    const RigExecRigEvaluator &_evaluator;

    /// The live sampler's pinned chain bindings: bound once per compiled
    /// rig (the compiled mover topology -- the chains are a function of
    /// nothing else) and dropped by NoteChainEdits. Not the frame-cache
    /// compiled rig: that also counts baked-program rebuilds, which a
    /// release triggers and which move no chain. Mutable because sampling
    /// through them is a const query.
    mutable RigExecChainSampleBindings _liveChainBindings;
    mutable size_t _liveChainCompiledRig = 0;
    mutable bool _liveChainValid = false;
    /// Moves on every real rebind: what a holder of a copy compares.
    mutable uint64_t _liveChainGeneration = 0;
    /// The prims an edit ON can move a bound chain: every chain mover and
    /// every target's prim. Exact matches only -- a chain targeting an
    /// attribute on a control high in the hierarchy is not moved by an
    /// edit to a control below it.
    mutable std::vector<SdfPath> _liveChainPrims;
    /// The chain movers alone, whose every property a binding reads, and
    /// the chain targets: on a target's prim only the target itself is
    /// bound, so an avar beside it moves nothing.
    mutable std::vector<SdfPath> _liveChainMovers;
    mutable std::vector<SdfPath> _liveChainTargets;
    /// The weight objects the chains read, matched with their subtrees: a
    /// weight's answer can come from geometry parented under it.
    mutable std::vector<SdfPath> _liveChainWeights;

    const RigExecBakedProgram *_sampleSupportProgram = nullptr;
    uint64_t _sampleSupportCompiledRig = 0;
    uint64_t _sampleSupportSerial = 0;
    bool _sampleSupportValid = false;
    bool _sampleSupport = false;
};

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_SAMPLING_H
