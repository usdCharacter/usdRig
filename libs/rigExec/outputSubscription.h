// The bounded subscription lifecycle for evaluated outputs at exact frames.
// A host asks for outputs at exact frames; the service keeps one demand per
// exact (rig, compile epoch, frame, captured generation), runs the work on the
// existing background pool through the explicit admission in
// backgroundScheduler.h, publishes the answers as coverage records in the
// frame cache, and hands the host immutable payload packets (
// docs/specs/output-subscriptions.md).
//
// The service owns no pool, evaluator or stage. The host supplies the
// owner-thread capture -- the only place live USD is read -- and the step
// runner a lane executes, so the service adds no second evaluator and no
// fallback path. Subscribe, UpdateSubscription, PollResults, PumpCallbacks and
// Close are owner-thread calls; a worker carries only values captured before
// it was queued and validates liveness and the scheduler's fences at
// completion, holding the pool's publish fence across the coverage insert and
// that frame's completion, and never the state lock.
//
// Demand is shared, never mutated after dispatch: outputs union into one
// launched mask per exact key — rig, frame, compile epoch, and the compiled
// program that minted the handles, so a program rebuild at a standing epoch
// gives a re-subscribed client its own demand rather than the old client's. An
// expansion joins the covered work when the launched mask already answers it and
// otherwise re-dispatches after the running job's terminal state. Closing a
// client releases only demand no other live client holds, and releasing every
// client releases queued work and byte reservations while leaving any live
// coverage lease untouched.
#ifndef RIGEXEC_OUTPUT_SUBSCRIPTION_H
#define RIGEXEC_OUTPUT_SUBSCRIPTION_H

#include "backgroundScheduler.h"
#include "frameCache.h"
#include "frozenContext.h"
#include "outputCatalog.h"
#include "rigExecGraph/outputDemand.h"

#include "pxr/usd/sdf/path.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

class RigExecSubscriptionState;

/// Why Subscribe or UpdateSubscription refused a request. A refusal changes
/// nothing: no demand, reservation or lease moved.
enum class RigExecSubscriptionStatus {
    /// The request stands, and its demand is reconciled toward dispatch.
    Accepted,
    /// The request is malformed: no demand, no handle, or handles that name
    /// more than one rig identity or compile epoch.
    InvalidRequest,
    /// The host's catalog hook named no catalog for the rig.
    UnknownRig,
    /// A handle names another rig instance or an older compile epoch: the
    /// host re-subscribes with handles from the current catalog.
    StaleEpoch,
    /// A handle's semantic address is not one this catalog can name.
    UnknownOutput,
    /// Every named output is unsupported, so no dispatch could answer any.
    NothingProducible,
    /// The subscription id names no live client.
    UnknownSubscription,
    /// The service is closing: no new demand is accepted.
    Stopping,
};

inline const char *
RigExecSubscriptionStatusText(RigExecSubscriptionStatus status)
{
    switch (status) {
    case RigExecSubscriptionStatus::Accepted:
        return "accepted";
    case RigExecSubscriptionStatus::InvalidRequest:
        return "invalid request";
    case RigExecSubscriptionStatus::UnknownRig:
        return "unknown rig";
    case RigExecSubscriptionStatus::StaleEpoch:
        return "stale compile epoch";
    case RigExecSubscriptionStatus::UnknownOutput:
        return "unknown output";
    case RigExecSubscriptionStatus::NothingProducible:
        return "no producible output";
    case RigExecSubscriptionStatus::UnknownSubscription:
        return "unknown subscription";
    case RigExecSubscriptionStatus::Stopping:
        return "stopping";
    }
    return "unknown";
}

/// What Subscribe and UpdateSubscription report. `unsupported` lists the
/// accepted handles the catalog can name but cannot produce (a GPU-only point
/// chain, an address with no retained producer); those outputs answer
/// `Unsupported` with the catalog's reason at every poll, and the rest of the
/// subscription stands.
struct RigExecSubscriptionReceipt {
    RigExecSubscriptionStatus status = RigExecSubscriptionStatus::Accepted;
    std::string reason;
    uint64_t subscription = 0;
    std::vector<RigExecOutputHandle> unsupported;
    /// The exact frames the accepted request demands.
    size_t frames = 0;

    bool IsAccepted() const
    {
        return status == RigExecSubscriptionStatus::Accepted;
    }
};

/// A live subscription. Movable, not copyable; destroying it closes the
/// subscription and releases the demand no other client holds, so a host
/// cannot leak a subscription by losing its handle.
class RigExecSubscriptionHandle {
public:
    RigExecSubscriptionHandle() = default;
    ~RigExecSubscriptionHandle();

    RigExecSubscriptionHandle(const RigExecSubscriptionHandle &) = delete;
    RigExecSubscriptionHandle &
    operator=(const RigExecSubscriptionHandle &) = delete;
    RigExecSubscriptionHandle(RigExecSubscriptionHandle &&other)
        : _state(other._state), _id(other._id)
    {
        other._id = 0;
    }
    RigExecSubscriptionHandle &operator=(RigExecSubscriptionHandle &&other)
    {
        if (this != &other) {
            Close();
            _state = std::move(other._state);
            _id = other._id;
            other._id = 0;
        }
        return *this;
    }

    /// Zero when the handle is empty or already closed.
    uint64_t Id() const
    {
        return _id;
    }
    bool IsValid() const
    {
        return _id != 0;
    }

    /// Ends the subscription now. Idempotent, and a no-op once the service
    /// has closed every client.
    void Close();

private:
    friend class RigExecOutputSubscriptionService;
    RigExecSubscriptionHandle(std::shared_ptr<RigExecSubscriptionState> state,
                              uint64_t id)
        : _state(std::move(state)), _id(id)
    {
    }

    std::shared_ptr<RigExecSubscriptionState> _state;
    uint64_t _id = 0;
};

/// One exact frame's captured job, built on the owner thread: everything a
/// detached lane needs, with the ownership that keeps the context's raw
/// pointers alive. A capture holds no stage, evaluator, imaging registry or
/// live handle, which is what makes it safe to queue.
///
/// The capture hook fills `inputs`, `context`, `captured`, `upstreamRevision`
/// and `bytes`; the lane hook fills `snapshot`, `patch` and `workspace`. The
/// service calls the lane hook only after the coverage record missed and the
/// byte reservation was granted, so a frame answered from coverage or refused
/// by the budget never builds an arena.
///
/// `bytes` is the admission estimate: `capture` for the sampled vector,
/// `workspace` for the lane's private arena, `payload` for the values the run
/// will publish. The frame cache refuses the job before it allocates when the
/// estimate would take the coverage budget past its cap, and the run extends
/// the charge to the payloads it actually published before it publishes them,
/// so an over-budget answer is declined rather than stored.
struct RigExecSubscriptionCapture {
    RigExecFrozenEvalContext context;
    RigExecFrameInputs inputs;
    std::shared_ptr<const RigExecFrozenProgram> snapshot;
    std::shared_ptr<const RigExecFrozenAvarPatch> patch;
    std::shared_ptr<RigExecFrozenWorkspace> workspace;
    /// The full generation the answers come from. `programIdentity` names the
    /// compiled program by its compile epoch, the identity the admission keys
    /// by, so a recompile is always a different demand.
    RigExecOutputGeneration captured;
    /// The upstream-revision half of the coverage key.
    uint64_t upstreamRevision = 0;
    RigExecCoverageBytes bytes;

    /// Points `context` at the owners this capture holds. The service calls it
    /// once the capture has reached the address it stays at, so the pointers
    /// a worker reads cannot move.
    void BindContext()
    {
        context.frozen = snapshot.get();
        context.avarPatch = patch.get();
        context.workspace = workspace.get();
    }
};

/// The catalog for the rig's current compiled epoch, or null when the rig is
/// unknown. Owner thread only: it reads compiled tables.
using RigExecSubscriptionCatalogHook =
    std::function<std::shared_ptr<const RigExecOutputCatalog>(const SdfPath &)>;

/// Preflights one exact frame's job for `demand`: sample its inputs and state
/// the generation and byte estimate the answers belong to. Owner thread only --
/// this is the only live-USD read in the lifecycle, and it allocates no
/// snapshot and no arena. False, having dispatched nothing, when the frame
/// cannot be captured; the packets then report `Failed` with `error`, and the
/// service tries again on the owner's next pump.
using RigExecSubscriptionCaptureHook = std::function<bool(
    const SdfPath &rig, const RigExecOutputTimeDemand &demand,
    const RigExecOutputCatalog &catalog, RigExecSubscriptionCapture *out,
    std::string *error)>;

/// Builds the lane `capture` runs on: the frozen snapshot of the standing
/// program and the private arena cloned from it. The service calls this only
/// after the coverage record missed and `TryReserveCoverage` granted the
/// estimate the capture hook stated, so a frame that coverage answers or the
/// budget refuses allocates nothing. Owner thread only, and it must not read
/// live USD: the frame's facts are already in `capture`. False leaves the frame
/// `Failed`, and the reservation the frame held comes back with it.
using RigExecSubscriptionLaneHook = std::function<bool(
    const SdfPath &rig, const RigExecOutputTimeDemand &demand,
    const RigExecOutputCatalog &catalog, RigExecSubscriptionCapture *capture,
    std::string *error)>;

/// What the service needs from its host. `stepRunner` names the lane's
/// executor; null means `RigExecMakeProductionRequestedStepRunner()`, and an
/// injected runner is how a test answers outputs without a rig -- the entry
/// point, fences and coverage bookkeeping stay the service's.
struct RigExecSubscriptionHost {
    RigExecBackgroundScheduler *scheduler = nullptr;
    RigExecFrameCache *frameCache = nullptr;
    RigExecSubscriptionCatalogHook resolveCatalog;
    RigExecSubscriptionCaptureHook capture;
    RigExecSubscriptionLaneHook lane;
    RigExecFrozenRequestedStepRunner stepRunner;
};

/// One subscription's completion, reported to the owner-thread callback. The
/// service queues it and never runs the callback on a worker, and an event for
/// a client that closed before its turn is dropped.
struct RigExecSubscriptionEvent {
    uint64_t subscription = 0;
    SdfPath rig;
    RigExecOutputTime time;
    /// The frame's disposition: Ready when the coverage record answers every
    /// demanded output, otherwise the status that stopped it.
    RigExecOutputStatus status = RigExecOutputStatus::Pending;
    size_t answered = 0;
    // The outputs the frame's run names. A handle the catalog marks
    // unsupported answers on its own and is not counted here.
    size_t demanded = 0;
    std::string reason;
};

using RigExecSubscriptionCallback =
    std::function<void(const RigExecSubscriptionEvent &)>;

/// Lifecycle counters. `coverageHits` counts frames answered from a coverage
/// record without admitting work; `dispatches` counts admissions the service
/// asked the pool for; the two declines split by which bound refused.
struct RigExecSubscriptionStats {
    size_t subscriptions = 0;
    size_t framesDemanded = 0;
    size_t demandsInFlight = 0;
    size_t coverageHits = 0;
    size_t dispatches = 0;
    size_t redispatches = 0;
    size_t declinedCoverageBudget = 0;
    size_t declinedAdmission = 0;
    size_t captureFailures = 0;
    size_t retiredInFlight = 0;
    size_t pinnedBytes = 0;
    size_t pinnedPayloads = 0;
    size_t callbacksQueued = 0;
};

/// The subscription service. The host owns the pool and the cache it is given,
/// and keeps them alive until the service is destroyed: closing waits for the
/// pool's lanes, which publish into that cache behind the wait. Closing closes
/// every client, releases every lease and reservation it holds, and retires
/// queued demand -- without holding the state lock across the wait. A
/// moved-from service holds no state: only destroying it is valid there, and
/// assigning over a live service retires the replaced one first.
class RigExecOutputSubscriptionService {
public:
    explicit RigExecOutputSubscriptionService(RigExecSubscriptionHost host);
    ~RigExecOutputSubscriptionService();

    RigExecOutputSubscriptionService(const RigExecOutputSubscriptionService &) =
        delete;
    RigExecOutputSubscriptionService &
    operator=(const RigExecOutputSubscriptionService &) = delete;
    RigExecOutputSubscriptionService(RigExecOutputSubscriptionService &&other) =
        default;
    /// Takes over `other`'s state and retires the service it replaces the way
    /// its destructor would: the replaced state can outlive this object
    /// through its live handles and its queued work, and nothing else closes
    /// it. Self-assignment changes nothing.
    RigExecOutputSubscriptionService &
    operator=(RigExecOutputSubscriptionService &&other);

    /// Asks for `request`'s outputs at their exact frames on `rig`. The
    /// request is validated against the rig's current catalog before anything
    /// moves; a handle naming another rig instance or compile epoch refuses
    /// the whole subscription rather than resolving to one of them. `receipt`
    /// reports why a refusal happened and which demanded outputs the catalog
    /// can name but cannot produce.
    RigExecSubscriptionHandle
    Subscribe(const SdfPath &rig, const RigExecOutputRequest &request,
              RigExecSubscriptionReceipt *receipt = nullptr);

    /// Replaces a live subscription's request atomically: the new request is
    /// validated first, and only then does the old demand become unreferenced
    /// demand. A refused update leaves the old subscription, its demand and
    /// its leases exactly as they were.
    RigExecSubscriptionReceipt
    UpdateSubscription(const RigExecSubscriptionHandle &subscription,
                       const RigExecOutputRequest &request);
    RigExecSubscriptionReceipt
    UpdateSubscription(uint64_t subscription,
                       const RigExecOutputRequest &request);

    /// What stands for each demanded (output, frame) pair, in request order:
    /// a Ready packet copied from the coverage record, whose payload the
    /// service pins for as long as the subscription lives, or a packet
    /// carrying the status that stopped it and the reason. Polling never
    /// evaluates and never dispatches; it only pins what it answers.
    std::vector<RigExecOutputPacket>
    PollResults(const RigExecSubscriptionHandle &subscription);
    std::vector<RigExecOutputPacket> PollResults(uint64_t subscription);

    /// Runs what the pool completed since the last pump, and re-dispatches the
    /// demand a retired generation or a launched-mask expansion left owed.
    /// Owner thread; the completion callback runs here, outside every lock.
    /// Returns how many events were delivered.
    size_t PumpCallbacks();

    /// Installs the completion callback (null uninstalls). Callbacks are
    /// queued by completions and run only on the owner's pump.
    void SetCompletionCallback(RigExecSubscriptionCallback callback);

    /// Ends one subscription. Its demand is dropped, and a frame no other
    /// live client demands has its queued work retired and its byte
    /// reservations returned. A coverage lease the service holds for this
    /// client is released; a lease another owner holds is never touched.
    void Close(const RigExecSubscriptionHandle &subscription);
    void Close(uint64_t subscription);
    /// Closes every client: the same release for every demand, and the pins.
    void CloseAll();

    RigExecSubscriptionStats Stats() const;
    bool IsStopping() const;

private:
    std::shared_ptr<RigExecSubscriptionState> _state;
};

/// Test-only probe for the deterministic publish-fence race: when set, the
/// job body of the service above runs it while holding the scheduler's fence
/// mutex, after a passing freshness check and before the coverage insert and
/// that frame's completion. Null in production. Set it only while no publish
/// is in flight (a test sets it around one driven job); the probe itself must
/// never publish, clear, or take the fence.
void RigExecSetExplicitPublishFenceProbeForTesting(std::function<void()> probe);

/// A coarse container-size estimate for one sampled vector: what a capture
/// charges the coverage budget for the values it holds. It counts the
/// containers, not what a `VtValue` points at, so a host whose samples carry
/// large buffers adds its own measure beside it.
size_t RigExecEstimateCaptureBytes(const RigExecFrameInputs &inputs);

} // namespace rigExec

#endif // RIGEXEC_OUTPUT_SUBSCRIPTION_H
