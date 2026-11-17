// The subscription lifecycle implementation: demand bookkeeping on the owner
// thread, one bounded job per exact (rig, epoch, compilation, frame, captured
// generation) on the existing pool, coverage publication, and pinned answers
// for polling clients.
//
// Lock discipline. The state mutex guards the client, group, demand and pin
// maps only. It is never held across a host hook, a frame-cache call, a pool
// call, or an evaluation: those run with it closed, and each map edit takes it
// briefly. A worker takes it twice per job -- once for liveness, once to record
// a terminal state -- and holds nothing else while it does. The pool's own
// order stays outermost: fence, pool, cache shard.
#include "outputSubscription.h"

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <utility>

namespace rigExec {

// The test-only publish-fence probe (see
// RigExecSetExplicitPublishFenceProbeForTesting): null in production, set
// only while no publish is in flight. The fenced publish below runs it while
// holding the fence mutex, so probe and publication share a translation unit.
static std::function<void()> _sPublishFenceProbe;

namespace {

// One frame of one rig at one compiled epoch of one compilation: the unit
// demand unions into, and the unit one launched mask belongs to. A program
// rebuild can leave the epoch standing, so the epoch alone does not say whose
// catalog and compile-local slots a frame's work belongs to.
struct _GroupKey {
    SdfPath rig;
    RigExecOutputTime time;
    uint64_t compileEpoch = 0;
    std::shared_ptr<const RigExecProgramAffinity> affinity;

    bool operator<(const _GroupKey &other) const
    {
        if (rig != other.rig) {
            return rig < other.rig;
        }
        if (!(time == other.time)) {
            return time < other.time;
        }
        if (compileEpoch != other.compileEpoch) {
            return compileEpoch < other.compileEpoch;
        }
        return affinity.get() < other.affinity.get();
    }
};

bool
_HandleLess(const RigExecOutputHandle &a, const RigExecOutputHandle &b)
{
    if (a.rigIdentity != b.rigIdentity) {
        return a.rigIdentity < b.rigIdentity;
    }
    if (a.semanticKey != b.semanticKey) {
        return a.semanticKey < b.semanticKey;
    }
    if (a.readPhase != b.readPhase) {
        return a.readPhase < b.readPhase;
    }
    if (a.type != b.type) {
        return a.type < b.type;
    }
    if (a.compileEpoch != b.compileEpoch) {
        return a.compileEpoch < b.compileEpoch;
    }
    return a.valueRevision < b.valueRevision;
}

// True when these packets answer every output the frame demands now, which can
// be wider than the mask one launched job ran.
bool
_AnswersEvery(const std::vector<RigExecOutputHandle> &desired,
              const std::vector<RigExecOutputPacket> &packets)
{
    for (const RigExecOutputHandle &handle : desired) {
        bool answered = false;
        for (const RigExecOutputPacket &packet : packets) {
            if (packet.status == RigExecOutputStatus::Ready &&
                packet.handle.SameTarget(handle)) {
                answered = true;
                break;
            }
        }
        if (!answered) {
            return false;
        }
    }
    return true;
}

// One payload the service pins for its live clients. The frame cache holds one
// lease per (record, output), so a payload two clients share is pinned once.
struct _PinKey {
    RigExecCoverageKey coverage;
    RigExecOutputHandle handle;

    bool operator<(const _PinKey &other) const
    {
        if (!(coverage == other.coverage)) {
            return coverage < other.coverage;
        }
        return _HandleLess(handle, other.handle);
    }
};

void
_UnionHandle(std::vector<RigExecOutputHandle> *list,
             const RigExecOutputHandle &handle)
{
    if (RigExecFindTarget(*list, handle) < 0) {
        list->push_back(handle);
    }
}

// A record's answer for one output: Ready, for the record's own time and
// generation, with a readable payload. Targets are matched as outputDemand.h
// matches them, so a handle pinned to a value revision still resolves to the
// output the record answers; the revision is the caller's to compare.
const RigExecOutputPacket *
_RecordAnswer(const RigExecCoverageRecord &record,
              const RigExecOutputHandle &handle)
{
    for (const RigExecOutputPacket &packet : record.packets) {
        if (packet.status != RigExecOutputStatus::Ready ||
            !(packet.time == record.key.time) ||
            !(packet.Generation() == record.key.generation) ||
            !packet.handle.SameTarget(handle)) {
            continue;
        }
        std::string error;
        if (!RigExecPayloadMatchesHandle(packet.handle, packet.payload,
                                         &error)) {
            continue;
        }
        return &packet;
    }
    return nullptr;
}

// One element of the declared layout per demanded output: the smallest answer a
// run could publish. A host that can state the real element count sets the
// estimate on its capture instead.
size_t
_PayloadFloorBytes(const std::vector<RigExecOutputHandle> &handles)
{
    size_t bytes = 0;
    for (const RigExecOutputHandle &handle : handles) {
        RigExecPayloadKind kind = RigExecPayloadKind::Unknown;
        if (!RigExecPayloadKindForType(handle.type, &kind)) {
            continue;
        }
        uint64_t elementBytes = 0;
        if (RigExecPayloadByteCount(kind, 1, &elementBytes)) {
            bytes += size_t(elementBytes);
        }
    }
    return bytes;
}

} // namespace

class RigExecSubscriptionState
    : public std::enable_shared_from_this<RigExecSubscriptionState> {
public:
    explicit RigExecSubscriptionState(RigExecSubscriptionHost host)
        : _host(std::move(host))
    {
        if (!_host.stepRunner) {
            _host.stepRunner = RigExecMakeProductionRequestedStepRunner();
        }
    }

    RigExecSubscriptionReceipt Subscribe(const SdfPath &rig,
                                         const RigExecOutputRequest &request)
    {
        RigExecSubscriptionReceipt receipt;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_stopping) {
                receipt.status = RigExecSubscriptionStatus::Stopping;
                receipt.reason = "the subscription service is closing";
                return receipt;
            }
        }
        std::string error;
        auto client = std::make_shared<Client>();
        if (!_Validate(request, rig, /*currentCatalog=*/true, client.get(),
                       &receipt, &error)) {
            return receipt;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_stopping) {
                receipt.status = RigExecSubscriptionStatus::Stopping;
                receipt.reason = "the subscription service is closing";
                return receipt;
            }
            client->id = ++_nextId;
            _clients.emplace(client->id, client);
        }
        receipt.subscription = client->id;
        _DispatchDue();
        return receipt;
    }

    RigExecSubscriptionReceipt Update(uint64_t id,
                                      const RigExecOutputRequest &request)
    {
        RigExecSubscriptionReceipt receipt;
        std::shared_ptr<Client> current;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_stopping) {
                receipt.status = RigExecSubscriptionStatus::Stopping;
                receipt.reason = "the subscription service is closing";
                return receipt;
            }
            const auto found = _clients.find(id);
            if (found == _clients.end()) {
                receipt.status = RigExecSubscriptionStatus::UnknownSubscription;
                receipt.reason = "no live subscription has that id";
                return receipt;
            }
            current = found->second;
        }
        // Validated against the catalog that minted this client's handles,
        // before the old request is dropped: a refused update changes nothing.
        std::shared_ptr<Client> staged = std::make_shared<Client>();
        *staged = *current;
        staged->request = request;
        staged->unsupported.clear();
        std::string error;
        if (!_Validate(request, current->rig, /*currentCatalog=*/false,
                       staged.get(), &receipt, &error)) {
            return receipt;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto found = _clients.find(id);
            if (found == _clients.end() || _stopping) {
                receipt.status = RigExecSubscriptionStatus::UnknownSubscription;
                receipt.reason =
                    "the subscription closed while the update was validated";
                return receipt;
            }
            found->second->request = staged->request;
            found->second->unsupported = std::move(staged->unsupported);
        }
        receipt.subscription = id;
        _DispatchDue();
        return receipt;
    }

    std::vector<RigExecOutputPacket> Poll(uint64_t id)
    {
        std::shared_ptr<Client> client;
        std::map<_GroupKey, _GroupView> views;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto found = _clients.find(id);
            if (found == _clients.end()) {
                return {};
            }
            client = found->second;
            for (const RigExecOutputTimeDemand &demand :
                 client->request.demands) {
                const _GroupKey key = _GroupOf(*client, demand.time);
                if (views.count(key)) {
                    continue;
                }
                _GroupView view;
                const auto group = _groups.find(key);
                if (group != _groups.end()) {
                    view.coverage = group->second.coverage;
                    view.hasCoverage = group->second.hasCoverage;
                    view.status = group->second.status;
                    view.reason = group->second.reason;
                } else {
                    view.status = RigExecOutputStatus::Pending;
                    view.reason = "the demand has not been reconciled yet";
                }
                views.emplace(key, std::move(view));
            }
        }

        std::vector<RigExecOutputPacket> packets;
        std::vector<_PinKey> toPin;
        for (const RigExecOutputTimeDemand &demand : client->request.demands) {
            const _GroupKey groupKey = _GroupOf(*client, demand.time);
            const _GroupView &view = views[groupKey];
            for (const RigExecOutputHandle &handle : demand.handles) {
                RigExecOutputPacket packet;
                packet.handle = handle;
                packet.time = demand.time;
                const RigExecOutputHandle *refused = _Refused(*client, handle);
                if (refused) {
                    packet.status = RigExecOutputStatus::Unsupported;
                    packet.reason = _RefusalReason(*client, handle);
                    packets.push_back(std::move(packet));
                    continue;
                }
                const RigExecOutputPacket *answer = nullptr;
                RigExecCoverageRecord record;
                const RigExecCoverageKey *answeredKey = nullptr;
                if (view.hasCoverage && _host.frameCache &&
                    _host.frameCache->LookupCoverage(view.coverage, &record)) {
                    answer = _RecordAnswer(record, handle);
                    if (answer) {
                        answeredKey = &view.coverage;
                    }
                }
                if (answer) {
                    if (handle.valueRevision != 0 &&
                        answer->handle.valueRevision != handle.valueRevision) {
                        packet.status = RigExecOutputStatus::Invalidated;
                        packet.reason =
                            handle.semanticKey + " stands at " +
                            std::to_string(answer->handle.valueRevision) +
                            "; the request named " +
                            std::to_string(handle.valueRevision);
                        packets.push_back(std::move(packet));
                        continue;
                    }
                    // The record's handle names the revision its run
                    // materialized; the request's may be a wildcard.
                    packet = *answer;
                    const _PinKey pinKey{*answeredKey, answer->handle};
                    if (!client->pins.count(pinKey)) {
                        toPin.push_back(pinKey);
                    }
                    packets.push_back(std::move(packet));
                    continue;
                }
                packet.status = view.status;
                packet.reason = view.reason;
                if (view.status == RigExecOutputStatus::Ready) {
                    // The frame's status says how its launched mask ended, not
                    // that this output answered: an output no record covers is
                    // owed, and never inherits its sibling's Ready.
                    packet.status = RigExecOutputStatus::Pending;
                    packet.reason = "no materialized answer for this output at "
                                    "this frame; the next pump demands it";
                }
                packets.push_back(std::move(packet));
            }
        }
        for (const _PinKey &pinKey : toPin) {
            _Pin(pinKey, id);
        }
        return packets;
    }

    size_t Pump()
    {
        _DispatchDue();
        std::vector<RigExecSubscriptionEvent> events;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            events.swap(_events);
            if (!_callback || _stopping) {
                return 0;
            }
        }
        size_t delivered = 0;
        for (const RigExecSubscriptionEvent &event : events) {
            // A callback may close its own subscription or every client, so
            // liveness and the current callback are re-read per event; the
            // callback itself always runs with the state lock closed.
            RigExecSubscriptionCallback active;
            {
                std::lock_guard<std::mutex> lock(_mutex);
                if (_stopping) {
                    break;
                }
                if (_clients.count(event.subscription)) {
                    active = _callback;
                }
            }
            if (!active) {
                continue;
            }
            active(event);
            ++delivered;
        }
        return delivered;
    }

    void SetCallback(RigExecSubscriptionCallback callback)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _callback = std::move(callback);
    }

    void Close(uint64_t id)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _CloseClientLocked(id);
        }
        _ReleaseStagedPins();
        _DispatchDue();
    }

    void CloseAll()
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            for (uint64_t id : _clientIds()) {
                _CloseClientLocked(id);
            }
        }
        _ReleaseStagedPins();
        _DispatchDue();
    }

    RigExecSubscriptionStats Stats() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        RigExecSubscriptionStats stats = _stats;
        stats.subscriptions = _clients.size();
        stats.demandsInFlight = _demands.size();
        stats.framesDemanded = _groups.size();
        for (const auto &pin : _pins) {
            ++stats.pinnedPayloads;
            stats.pinnedBytes += pin.second.lease.ByteCount();
        }
        stats.callbacksQueued = _events.size();
        return stats;
    }

    bool IsStopping() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _stopping;
    }

    /// Closes every client, releases every lease and reservation this service
    /// owns, drops its queued jobs from the pool, and joins the lanes --
    /// holding no lock while it does.
    void Shutdown()
    {
        std::vector<uint64_t> ids;
        std::vector<RigExecExplicitJobKey> keys;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _stopping = true;
            _callback = nullptr;
            _events.clear();
            ids = _clientIds();
            keys.reserve(_demands.size());
            for (const auto &entry : _demands) {
                entry.second->live = false;
                keys.push_back(entry.first);
            }
            _clients.clear();
            _demands.clear();
            _groups.clear();
        }
        for (uint64_t id : ids) {
            _StageClientPins(id);
        }
        _ReleaseStagedPins();
        if (!_host.scheduler) {
            return;
        }
        // A queued closure holds this state and its demand, so its byte and
        // coverage reservations stand with it: a manual pool never drains on
        // its own, and joining cannot be waited for there. Purge exactly this
        // service's keys -- another host's queued work is untouched -- then
        // join what already popped, whose body sees the demand went dead.
        for (const RigExecExplicitJobKey &key : keys) {
            _host.scheduler->CancelExplicitJob(key);
        }
        if (_host.scheduler->GetWorkerCount() > 0) {
            _host.scheduler->WaitUntilIdle();
        }
    }

private:
    struct Client {
        uint64_t id = 0;
        SdfPath rig;
        std::string rigIdentity;
        uint64_t compileEpoch = 0;
        RigExecOutputRequest request;
        std::shared_ptr<const RigExecOutputCatalog> catalog;
        // The compilation that minted this client's handles, taken from its
        // catalog: what separates a client from one re-subscribed after a
        // program rebuild at the same epoch.
        std::shared_ptr<const RigExecProgramAffinity> affinity;
        // Named outputs the catalog cannot produce, with its reason.
        std::vector<std::pair<RigExecOutputHandle, std::string>> unsupported;
        std::set<_PinKey> pins;
    };

    struct Demand {
        RigExecExplicitJobKey key;
        RigExecCoverageKey coverage;
        // The compilation this job runs, matching its group and its catalog.
        std::shared_ptr<const RigExecProgramAffinity> affinity;
        // The union this job answers, fixed at admission: a launched mask is
        // never widened afterwards.
        std::vector<RigExecOutputHandle> outputs;
        std::shared_ptr<RigExecSubscriptionCapture> capture;
        RigExecFrozenRequestedRun requested;
        RigExecCoverageReservation reservation;
        bool live = true;
    };

    struct Group {
        std::shared_ptr<const RigExecOutputCatalog> catalog;
        std::vector<RigExecOutputHandle> desired;
        std::vector<RigExecExplicitJobKey> inFlight;
        // The coverage key of the generation this frame stands at now, from
        // the latest capture. A key from a retired generation is not kept:
        // it would answer a poll with values the current inputs do not stand
        // for.
        RigExecCoverageKey coverage;
        bool hasCoverage = false;
        RigExecOutputStatus status = RigExecOutputStatus::Pending;
        std::string reason;
    };

    struct Pin {
        RigExecCoverageLease lease;
        std::set<uint64_t> clients;
    };

    struct _GroupView {
        RigExecCoverageKey coverage;
        bool hasCoverage = false;
        RigExecOutputStatus status = RigExecOutputStatus::Pending;
        std::string reason;
    };

    // Validates a request against the rig's catalog and fills `client`'s
    // identity and unsupported list. `currentCatalog` asks the host for the
    // rig's current epoch (a new subscription) or keeps the client's own
    // (a replacement for an existing one).
    bool _Validate(const RigExecOutputRequest &request, const SdfPath &rig,
                   bool currentCatalog, Client *client,
                   RigExecSubscriptionReceipt *receipt, std::string *error)
    {
        const auto refuse = [&](RigExecSubscriptionStatus status,
                                const std::string &message) {
            receipt->status = status;
            receipt->reason = message;
            return false;
        };
        if (!request.IsWellFormed(error)) {
            return refuse(RigExecSubscriptionStatus::InvalidRequest, *error);
        }
        std::string rigIdentity;
        uint64_t compileEpoch = 0;
        if (!RigExecRequestGroup(request, &rigIdentity, &compileEpoch, error)) {
            return refuse(RigExecSubscriptionStatus::InvalidRequest, *error);
        }
        std::shared_ptr<const RigExecOutputCatalog> catalog = client->catalog;
        if (currentCatalog) {
            catalog =
                _host.resolveCatalog ? _host.resolveCatalog(rig) : nullptr;
        }
        if (!catalog) {
            return refuse(RigExecSubscriptionStatus::UnknownRig,
                          "no output catalog names this rig");
        }
        if (catalog->GetRigIdentity() != rigIdentity ||
            catalog->GetCompileEpoch() != compileEpoch) {
            return refuse(RigExecSubscriptionStatus::StaleEpoch,
                          "the request names rig " + rigIdentity + " epoch " +
                              std::to_string(compileEpoch) + "; this rig is " +
                              catalog->GetRigIdentity() + " epoch " +
                              std::to_string(catalog->GetCompileEpoch()));
        }
        client->rig = rig;
        client->rigIdentity = rigIdentity;
        client->compileEpoch = compileEpoch;
        client->request = request;
        client->catalog = catalog;
        client->affinity = catalog->GetProgramAffinity();
        size_t producible = 0;
        for (const RigExecOutputTimeDemand &demand : request.demands) {
            for (const RigExecOutputHandle &handle : demand.handles) {
                size_t slot = 0;
                if (!catalog->GetDemandIndex().Find(handle, &slot)) {
                    return refuse(
                        RigExecSubscriptionStatus::UnknownOutput,
                        handle.semanticKey +
                            " is not a semantic address this catalog exports");
                }
                std::string reason;
                const RigExecOutputStatus resolved =
                    catalog->Resolve(handle, nullptr, &reason);
                if (resolved == RigExecOutputStatus::Invalidated) {
                    return refuse(RigExecSubscriptionStatus::StaleEpoch,
                                  handle.semanticKey + ": " + reason);
                }
                if (resolved == RigExecOutputStatus::Unsupported) {
                    client->unsupported.emplace_back(handle, reason);
                    continue;
                }
                ++producible;
            }
        }
        if (!producible) {
            return refuse(RigExecSubscriptionStatus::NothingProducible,
                          "no named output has a retained producer");
        }
        receipt->status = RigExecSubscriptionStatus::Accepted;
        receipt->frames = request.demands.size();
        for (const auto &entry : client->unsupported) {
            receipt->unsupported.push_back(entry.first);
        }
        return true;
    }

    static _GroupKey _GroupOf(const Client &client,
                              const RigExecOutputTime &time)
    {
        return _GroupKey{client.rig, time, client.compileEpoch,
                         client.affinity};
    }

    static const RigExecOutputHandle *_Refused(const Client &client,
                                               const RigExecOutputHandle &want)
    {
        for (const auto &entry : client.unsupported) {
            if (entry.first.SameTarget(want)) {
                return &entry.first;
            }
        }
        return nullptr;
    }

    static std::string _RefusalReason(const Client &client,
                                      const RigExecOutputHandle &want)
    {
        for (const auto &entry : client.unsupported) {
            if (entry.first.SameTarget(want)) {
                return entry.second;
            }
        }
        return std::string();
    }

    std::vector<uint64_t> _clientIds() const
    {
        std::vector<uint64_t> ids;
        ids.reserve(_clients.size());
        for (const auto &entry : _clients) {
            ids.push_back(entry.first);
        }
        return ids;
    }

    // Call with the state lock held. Stages the client's last lease on each
    // payload so the caller can drop it outside the lock. The pin leaves
    // `_pins` through this caller's iterator.
    void _CloseClientLocked(uint64_t id)
    {
        const auto found = _clients.find(id);
        if (found == _clients.end()) {
            return;
        }
        for (const _PinKey &pinKey : found->second->pins) {
            const auto pin = _pins.find(pinKey);
            if (pin == _pins.end()) {
                continue;
            }
            pin->second.clients.erase(id);
            if (pin->second.clients.empty()) {
                _stagedPins.push_back(std::move(pin->second.lease));
                _pins.erase(pin);
            }
        }
        _clients.erase(found);
        _events.erase(
            std::remove_if(_events.begin(), _events.end(),
                           [id](const RigExecSubscriptionEvent &event) {
                               return event.subscription == id;
                           }),
            _events.end());
    }

    // Pins are the owner thread's; releasing one takes a cache shard lock, so
    // the lease always dies with the state lock closed. The iteration advances
    // through the erase: restaging by key would free the entry being walked.
    void _StageClientPins(uint64_t id)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (auto pin = _pins.begin(); pin != _pins.end();) {
            if (pin->second.clients.erase(id) && pin->second.clients.empty()) {
                _stagedPins.push_back(std::move(pin->second.lease));
                pin = _pins.erase(pin);
            } else {
                ++pin;
            }
        }
    }

    void _ReleaseStagedPins()
    {
        std::vector<RigExecCoverageLease> staged;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            staged.swap(_stagedPins);
        }
        staged.clear();
    }

    void _Pin(const _PinKey &pinKey, uint64_t id)
    {
        if (!_host.frameCache) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto pin = _pins.find(pinKey);
            if (pin != _pins.end()) {
                pin->second.clients.insert(id);
                const auto client = _clients.find(id);
                if (client != _clients.end()) {
                    client->second->pins.insert(pinKey);
                }
                return;
            }
        }
        RigExecCoverageLease lease;
        if (!_host.frameCache->PinCoverage(pinKey.coverage, pinKey.handle,
                                           &lease)) {
            return;
        }
        std::lock_guard<std::mutex> lock(_mutex);
        Pin &pin = _pins[pinKey];
        if (pin.lease.IsValid()) {
            // Another client pinned it while this poll was outside the lock;
            // the extra lease dies here.
            return;
        }
        pin.lease = std::move(lease);
        pin.clients.insert(id);
        const auto client = _clients.find(id);
        if (client != _clients.end()) {
            client->second->pins.insert(pinKey);
        }
    }

    // Owner-thread reconcile: prune jobs the pool already retired, re-union the
    // live demand, then capture and admit what is still owed. Every host hook,
    // cache call and pool call here runs with the state lock closed.
    void _DispatchDue()
    {
        if (!_host.scheduler || !_host.frameCache) {
            return;
        }
        std::vector<_GroupKey> owed;
        std::vector<RigExecExplicitJobKey> purges;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _PruneRetiredLocked();
            _RebuildDemandLocked(&purges);
            for (const auto &group : _groups) {
                if (group.second.desired.empty()) {
                    continue;
                }
                // Work already launched for this frame owns it: a launched
                // mask is fixed at admission, so an expansion joins it when
                // that job's record covers the demand and re-dispatches
                // beside it once the job reaches a terminal state.
                if (!group.second.inFlight.empty()) {
                    continue;
                }
                owed.push_back(group.first);
            }
        }
        for (const RigExecExplicitJobKey &key : purges) {
            // Retirement for a frame no live client demands any more: exactly
            // this job leaves the queue and its byte reservation comes back.
            // No other client's work, warm or explicit, is touched.
            _host.scheduler->CancelExplicitJob(key);
        }
        for (const _GroupKey &group : owed) {
            _CaptureAndAdmit(group);
        }
    }

    // A key the pool no longer reports in flight reached a terminal state this
    // service could not observe -- purged by another host's cancel, or dropped
    // at shutdown. Its demand is owed again.
    void _PruneRetiredLocked()
    {
        std::map<SdfPath, std::vector<std::shared_ptr<Demand>>> queued;
        for (const auto &entry : _demands) {
            queued[entry.first.rig].push_back(entry.second);
        }
        for (const auto &entry : queued) {
            const std::vector<RigExecExplicitJobKey> standing =
                _host.scheduler->InFlightExplicitKeys(entry.first);
            for (const std::shared_ptr<Demand> &job : entry.second) {
                if (std::find(standing.begin(), standing.end(), job->key) !=
                    standing.end()) {
                    continue;
                }
                _RetireDemandLocked(job->key, RigExecOutputStatus::Pending,
                                    "the pool retired this frame's job; the "
                                    "next pump re-admits it",
                                    job->affinity.get());
                ++_stats.retiredInFlight;
            }
        }
    }

    // Re-union every live client's demand. A frame no client demands leaves
    // with its queued job.
    void _RebuildDemandLocked(std::vector<RigExecExplicitJobKey> *purges)
    {
        for (auto &group : _groups) {
            group.second.desired.clear();
        }
        for (const auto &entry : _clients) {
            const Client &client = *entry.second;
            for (const RigExecOutputTimeDemand &demand :
                 client.request.demands) {
                Group &group = _groups[_GroupOf(client, demand.time)];
                if (!group.catalog) {
                    group.catalog = client.catalog;
                }
                for (const RigExecOutputHandle &handle : demand.handles) {
                    // A handle this client's own catalog marks unsupported has
                    // no producer in this generation: naming it in the demand
                    // would refuse the whole requested run and starve its
                    // supported siblings. It answers Unsupported per handle in
                    // the poll instead.
                    if (_Refused(client, handle)) {
                        continue;
                    }
                    _UnionHandle(&group.desired, handle);
                }
            }
        }
        for (auto it = _groups.begin(); it != _groups.end();) {
            if (!it->second.desired.empty()) {
                ++it;
                continue;
            }
            // Retirement drops the key from this group's in-flight list, so
            // the walk takes the list out of the group first: nothing is left
            // under it to shrink.
            std::vector<RigExecExplicitJobKey> keys;
            keys.swap(it->second.inFlight);
            for (const RigExecExplicitJobKey &key : keys) {
                _RetireDemandLocked(key, RigExecOutputStatus::Pending,
                                    "no live subscription demands this frame",
                                    it->first.affinity.get());
                purges->push_back(key);
            }
            it = _groups.erase(it);
        }
    }

    // Drop one demand and its group link. The queued closure, and with it the
    // byte reservation, dies wherever the pool retires the job. \p affinity is
    // the compilation that job ran: two compilations of one epoch can name the
    // same job key, and only the group it belongs to hears about it.
    void _RetireDemandLocked(const RigExecExplicitJobKey &key,
                             RigExecOutputStatus status,
                             const std::string &reason,
                             const RigExecProgramAffinity *affinity)
    {
        const auto demand = _demands.find(key);
        if (demand != _demands.end() &&
            (!affinity || demand->second->affinity.get() == affinity)) {
            demand->second->live = false;
            _demands.erase(demand);
        }
        for (auto &group : _groups) {
            if (!(group.first.rig == key.rig && group.first.time == key.time &&
                  group.first.compileEpoch == key.generation.programIdentity) ||
                (affinity && group.first.affinity.get() != affinity)) {
                continue;
            }
            group.second.inFlight.erase(
                std::remove(group.second.inFlight.begin(),
                            group.second.inFlight.end(), key),
                group.second.inFlight.end());
            group.second.status = status;
            group.second.reason = reason;
        }
    }

    // Capture this frame's job on the owner thread and admit it -- unless the
    // generation that capture names is already answered, which pulls nothing.
    void _CaptureAndAdmit(const _GroupKey &groupKey)
    {
        std::shared_ptr<const RigExecOutputCatalog> catalog;
        std::vector<RigExecOutputHandle> desired;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const auto group = _groups.find(groupKey);
            if (group == _groups.end() || group->second.desired.empty() ||
                !group->second.inFlight.empty() || _stopping) {
                return;
            }
            catalog = group->second.catalog;
            desired = group->second.desired;
        }
        if (!catalog || !_host.capture || !_host.lane) {
            std::lock_guard<std::mutex> lock(_mutex);
            _DropCoverageLocked(groupKey);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Failed,
                              "no capture and lane hook for this rig");
            return;
        }
        // The epoch the handles were minted at has to be the epoch standing
        // now: a recompile would otherwise run this epoch's job closure over
        // the new program and label the answers with the old epoch.
        const std::shared_ptr<const RigExecOutputCatalog> current =
            _host.resolveCatalog ? _host.resolveCatalog(groupKey.rig) : nullptr;
        if (!current) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(
                groupKey, RigExecOutputStatus::Invalidated,
                "the rig names no output catalog any more; re-subscribe with "
                "handles from the current catalog");
            return;
        }
        if (current->GetCompileEpoch() != catalog->GetCompileEpoch() ||
            current->GetRigIdentity() != catalog->GetRigIdentity()) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(
                groupKey, RigExecOutputStatus::Invalidated,
                "the rig recompiled to epoch " +
                    std::to_string(current->GetCompileEpoch()) +
                    "; these outputs were minted at epoch " +
                    std::to_string(catalog->GetCompileEpoch()) +
                    ": re-subscribe with handles from the current catalog");
            return;
        }
        // A program rebuild can leave the epoch standing. Its catalogs name
        // the previous compilation, and no run of the new program answers them.
        if (current->GetProgramAffinity() != catalog->GetProgramAffinity()) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(
                groupKey, RigExecOutputStatus::Invalidated,
                "the rig rebuilt its program at epoch " +
                    std::to_string(current->GetCompileEpoch()) +
                    "; these outputs were minted from the previous "
                    "compilation: re-subscribe with handles from the current "
                    "catalog");
            return;
        }

        RigExecOutputTimeDemand demand;
        demand.time = groupKey.time;
        demand.handles = desired;
        auto capture = std::make_shared<RigExecSubscriptionCapture>();
        std::string error;
        // The preflight only: the sampled vector, the generation the answers
        // come from, and the byte estimate the arena will be charged at. The
        // snapshot and the arena wait until the frame is actually owed work.
        if (!_host.capture(groupKey.rig, demand, *catalog, capture.get(),
                           &error)) {
            std::lock_guard<std::mutex> lock(_mutex);
            // Nothing read this frame's live generation, so nothing can say a
            // standing record still answers it.
            _DropCoverageLocked(groupKey);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Failed,
                              "capture: " + error);
            ++_stats.captureFailures;
            return;
        }
        const RigExecFrameGeneration editGeneration =
            _host.scheduler->CurrentGeneration(groupKey.rig);
        capture->context.generation = editGeneration;
        if (capture->captured.programIdentity == 0) {
            capture->captured.programIdentity = catalog->GetCompileEpoch();
        } else if (capture->captured.programIdentity !=
                   catalog->GetCompileEpoch()) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(
                groupKey, RigExecOutputStatus::Invalidated,
                "the capture names program " +
                    std::to_string(capture->captured.programIdentity) +
                    "; this catalog is epoch " +
                    std::to_string(catalog->GetCompileEpoch()));
            return;
        }

        RigExecCoverageKey coverageKey;
        coverageKey.rigIdentity = catalog->GetRigIdentity();
        coverageKey.compileEpoch = catalog->GetCompileEpoch();
        coverageKey.time = groupKey.time;
        coverageKey.generation = capture->captured;
        coverageKey.upstreamRevision = capture->upstreamRevision;
        // Capture named the generation that stands now, so this frame is
        // answerable only from that key. Install it before anything can
        // decline the replacement work: a refusal then lands on its own status
        // instead of leaving a retired generation's record answering the frame.
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _RememberCoverageLocked(groupKey, coverageKey);
        }
        // The capture names the generation that stands now, and its answers
        // already stand: serve them, dispatch nothing.
        if (_AnswersCover(coverageKey, desired)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Ready,
                              "the coverage record answers this frame");
            ++_stats.coverageHits;
            return;
        }

        RigExecCoverageBytes budget = capture->bytes;
        if (budget.payload == 0) {
            budget.payload = _PayloadFloorBytes(desired);
        }
        RigExecCoverageReservation reservation;
        if (!_host.frameCache->TryReserveCoverage(budget, &reservation)) {
            // A record whose demand is gone is the cost of an answered frame,
            // not a toll on every frame after it: the records no lease holds
            // and no live frame reads come back, oldest first, and this frame
            // asks once more.
            size_t needed = 0;
            budget.TryTotal(&needed);
            std::vector<RigExecCoverageKey> live;
            {
                std::lock_guard<std::mutex> lock(_mutex);
                live = _LiveCoverageKeysLocked();
            }
            _host.frameCache->ReclaimCoverage(needed, live);
            if (!_host.frameCache->TryReserveCoverage(budget, &reservation)) {
                std::lock_guard<std::mutex> lock(_mutex);
                _NoteStatusLocked(groupKey, RigExecOutputStatus::Pending,
                                  "the coverage budget declined this frame; "
                                  "the next pump retries it");
                ++_stats.declinedCoverageBudget;
                return;
            }
        }
        // The frame is owed work and the budget holds for it, so the arena is
        // paid for before it exists. A lane the host cannot build leaves the
        // reservation to die with this scope.
        if (!_host.lane(groupKey.rig, demand, *catalog, capture.get(),
                        &error)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Failed,
                              "lane: " + error);
            return;
        }
        capture->BindContext();

        RigExecFrozenRequestedRun requested;
        // One record answers every client of this frame: the job asks for
        // whatever the run produces, and Poll validates a pinned revision.
        for (RigExecOutputHandle &handle : demand.handles)
            handle.valueRevision = 0;
        if (!RigExecMakeFrozenRequestedRun(catalog, demand, capture->captured,
                                           &requested, &error)) {
            std::lock_guard<std::mutex> lock(_mutex);
            _NoteStatusLocked(groupKey, RigExecOutputStatus::Unsupported,
                              error);
            return;
        }

        auto job = std::make_shared<Demand>();
        job->key.rig = groupKey.rig;
        job->key.time = groupKey.time;
        job->key.generation = capture->captured;
        job->affinity = catalog->GetProgramAffinity();
        job->coverage = coverageKey;
        job->outputs = desired;
        job->capture = capture;
        job->requested = std::move(requested);
        job->reservation = std::move(reservation);

        RigExecExplicitRequest admission;
        admission.rig = groupKey.rig;
        admission.time = groupKey.time;
        admission.generation = capture->captured;
        admission.outputs = desired;
        admission.editGeneration = editGeneration;
        size_t reservedBytes = 0;
        budget.TryTotal(&reservedBytes);
        admission.reservationBytes = reservedBytes;

        RigExecExplicitWork work = [state = shared_from_this(), job](
                                       const RigExecExplicitRequest &request) {
            return state->RunDemand(job, request);
        };
        // The demand and its launched mask are owned before the pool sees the
        // job. Admission notifies the lanes and returns without a completion
        // barrier, so a worker can reach a terminal state inside this call:
        // registering afterwards would resurrect that job's demand and stamp
        // its terminal status back to Pending.
        {
            std::lock_guard<std::mutex> lock(_mutex);
            // The pool keys a job by rig, frame and captured generation, which
            // two compilations of one epoch can share. A job already in flight
            // from another compilation owns that key: this pump waits for it to
            // reach its terminal state rather than registering a second demand
            // the pool would fold into the first.
            const auto standing = _demands.find(job->key);
            if (standing != _demands.end() &&
                standing->second->affinity != job->affinity) {
                _NoteStatusLocked(
                    groupKey, RigExecOutputStatus::Pending,
                    "a job admitted from another compilation still owns this "
                    "frame; the next pump re-admits it");
                return;
            }
            _demands.emplace(job->key, job);
            ++_stats.dispatches;
            const auto group = _groups.find(groupKey);
            if (group != _groups.end()) {
                group->second.inFlight.push_back(job->key);
                _NoteStatusLocked(groupKey, RigExecOutputStatus::Pending,
                                  "a job for this frame is queued");
            }
        }
        if (!_host.scheduler->AdmitExplicitOutput(admission, std::move(work))) {
            std::lock_guard<std::mutex> lock(_mutex);
            // Nothing was queued, so the launched mask goes back with it and
            // the reservation dies with this job.
            _RetireDemandLocked(job->key, RigExecOutputStatus::Pending,
                                "the pool declined this admission; the next "
                                "pump retries it",
                                job->affinity.get());
            --_stats.dispatches;
            ++_stats.declinedAdmission;
            return;
        }
    }

    // Every demanded output already stands under this generation's record.
    bool _AnswersCover(const RigExecCoverageKey &key,
                       const std::vector<RigExecOutputHandle> &desired) const
    {
        if (!_host.frameCache) {
            return false;
        }
        RigExecCoverageRecord record;
        if (!_host.frameCache->LookupCoverage(key, &record)) {
            return false;
        }
        for (const RigExecOutputHandle &handle : desired) {
            if (!_RecordAnswer(record, handle)) {
                return false;
            }
        }
        return true;
    }

    // The coverage keys live demand reads now, one per frame a client still
    // demands: pressure reclaims every other unpinned record and leaves these.
    std::vector<RigExecCoverageKey> _LiveCoverageKeysLocked() const
    {
        std::vector<RigExecCoverageKey> keys;
        keys.reserve(_groups.size());
        for (const auto &group : _groups) {
            if (group.second.hasCoverage) {
                keys.push_back(group.second.coverage);
            }
        }
        return keys;
    }

    void _NoteStatusLocked(const _GroupKey &key, RigExecOutputStatus status,
                           const std::string &reason)
    {
        const auto group = _groups.find(key);
        if (group == _groups.end()) {
            return;
        }
        group->second.status = status;
        group->second.reason = reason;
        // A frame whose generation no longer stands has no answer to give: the
        // coverage key went with the generation it was captured at.
        if (status == RigExecOutputStatus::Invalidated) {
            group->second.coverage = RigExecCoverageKey();
            group->second.hasCoverage = false;
        }
    }

    // A frame whose live generation capture could not read has no answer to
    // give: the record it had was keyed to a generation nothing validated.
    void _DropCoverageLocked(const _GroupKey &key)
    {
        const auto group = _groups.find(key);
        if (group == _groups.end()) {
            return;
        }
        group->second.coverage = RigExecCoverageKey();
        group->second.hasCoverage = false;
    }

    void _RememberCoverageLocked(const _GroupKey &key,
                                 const RigExecCoverageKey &coverageKey)
    {
        const auto group = _groups.find(key);
        if (group == _groups.end()) {
            return;
        }
        // One key per group, from the generation its last capture named. A
        // moved generation replaces it rather than joining a list, so nothing
        // answers a poll from a record the current inputs do not stand for.
        group->second.coverage = coverageKey;
        group->second.hasCoverage = true;
    }

    // The job body, on a pool worker. The capture, the reservation and the
    // coverage key are this job's alone; the maps are read under the state
    // lock, which is never held across the run, a fence, or the publish.
    RigExecExplicitOutcome RunDemand(const std::shared_ptr<Demand> &job,
                                     const RigExecExplicitRequest &request)
    {
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (!job->live || _stopping) {
                return RigExecExplicitOutcome::DeclinedInvalid;
            }
        }
        // The pool's fence first: a scoped edit or a generation bump since this
        // job was queued means its answer belongs to a retired frame.
        if (!_host.scheduler->IsExplicitRequestCurrent(request)) {
            return _RetireRun(job, request,
                              RigExecExplicitOutcome::DeclinedGeneration,
                              RigExecOutputStatus::Invalidated,
                              "the pool retired this frame while its job ran");
        }
        const RigExecFrozenRequestedResult result =
            RigExecEvaluateFrozenRequested(
                job->capture->context, job->capture->inputs, job->requested,
                _host.stepRunner, _host.scheduler, request.rig);
        if (!result.ran) {
            const bool retired =
                result.error.find("generation") != std::string::npos;
            return _RetireRun(job, request,
                              retired
                                  ? RigExecExplicitOutcome::DeclinedGeneration
                                  : RigExecExplicitOutcome::DeclinedInvalid,
                              retired ? RigExecOutputStatus::Invalidated
                                      : RigExecOutputStatus::Failed,
                              "the requested run declined: " + result.error);
        }
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (!job->live || _stopping) {
                // The bodies ran, but no live demand owns this frame: the
                // reservation comes back and nothing is published.
                return RigExecExplicitOutcome::DeclinedInvalid;
            }
        }
        size_t payload = 0;
        for (const RigExecOutputPacket &packet : result.packets) {
            payload += packet.payload.ByteCount();
        }
        // Admission already charged the estimate (or the layout floor) for this
        // payload; a longer answer pays only the gap above it.
        const size_t reserved = job->reservation.Bytes().payload;
        const size_t shortfall = payload > reserved ? payload - reserved : 0;
        if (shortfall > 0 && !job->reservation.Extend(
                                 RigExecCoverageDomain::Payload, shortfall)) {
            return _RetireRun(
                job, request, RigExecExplicitOutcome::DeclinedInvalid,
                RigExecOutputStatus::Failed,
                "the coverage budget declined this frame's payloads");
        }
        // The publish fence spans the freshness check, the coverage insert and
        // this frame's completion: the owner's cancel-then-clear holds the
        // same mutex across generation bump + queue purge + cache clear, so a
        // retired answer can neither slip between the check and the store nor
        // stamp its frame Ready after that clear. Lock order: fence, then the
        // state lock; the bodies and the byte charge above stay outside it.
        std::lock_guard<std::mutex> fence(_host.scheduler->FenceMutex());
        if (!_host.scheduler->IsExplicitRequestCurrent(request)) {
            return _RetireRun(job, request,
                              RigExecExplicitOutcome::DeclinedGeneration,
                              RigExecOutputStatus::Invalidated,
                              "the pool retired this frame while its job ran");
        }
        if (_sPublishFenceProbe) {
            _sPublishFenceProbe();
        }
        // The lane's arena dies with the job, so the record keeps the payloads
        // and claims no retained workspace.
        if (!_host.frameCache->PublishCoverage(job->coverage, result.packets,
                                               std::move(job->reservation),
                                               false)) {
            return _RetireRun(job, request,
                              RigExecExplicitOutcome::DeclinedInvalid,
                              RigExecOutputStatus::Failed,
                              "the frame cache declined this coverage record");
        }
        _CompleteRun(job, request, result.packets);
        return RigExecExplicitOutcome::Published;
    }

    RigExecExplicitOutcome _RetireRun(const std::shared_ptr<Demand> &job,
                                      const RigExecExplicitRequest &request,
                                      RigExecExplicitOutcome outcome,
                                      RigExecOutputStatus status,
                                      const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _RetireDemandLocked(job->key, status, reason, job->affinity.get());
        _QueueEventsLocked(request.rig, job->coverage.time,
                           job->key.generation.programIdentity, job->affinity,
                           status, reason, 0, job->outputs.size());
        return outcome;
    }

    // Records what landed. An expansion noticed while this job ran waits for
    // the owner's next pump: the launched mask stays exactly as admitted.
    void _CompleteRun(const std::shared_ptr<Demand> &job,
                      const RigExecExplicitRequest &request,
                      const std::vector<RigExecOutputPacket> &packets)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        size_t answered = 0;
        for (const RigExecOutputHandle &handle : job->outputs) {
            for (const RigExecOutputPacket &packet : packets) {
                if (packet.status == RigExecOutputStatus::Ready &&
                    packet.handle.SameTarget(handle)) {
                    ++answered;
                    break;
                }
            }
        }
        const bool complete = answered == job->outputs.size();
        const RigExecOutputStatus status =
            complete ? RigExecOutputStatus::Ready : RigExecOutputStatus::Failed;
        const std::string reason =
            complete
                ? std::string()
                : "the run answered " + std::to_string(answered) + " of " +
                      std::to_string(job->outputs.size()) + " demanded outputs";
        const auto demand = _demands.find(job->key);
        if (demand != _demands.end()) {
            demand->second->live = false;
            _demands.erase(demand);
        }
        RigExecOutputStatus frameStatus = status;
        std::string frameReason = reason;
        for (auto &group : _groups) {
            if (!(group.first.rig == job->key.rig &&
                  group.first.time == job->key.time &&
                  group.first.compileEpoch ==
                      job->key.generation.programIdentity) ||
                group.first.affinity != job->affinity) {
                continue;
            }
            group.second.inFlight.erase(
                std::remove(group.second.inFlight.begin(),
                            group.second.inFlight.end(), job->key),
                group.second.inFlight.end());
            _RememberCoverageLocked(group.first, job->coverage);
            RigExecOutputStatus groupStatus = status;
            std::string groupReason = reason;
            // A launched mask is fixed at admission, so a demand that widened
            // while this job ran leaves outputs no packet answers: the frame
            // owes them to the next pump and is not Ready.
            if (status == RigExecOutputStatus::Ready &&
                !_AnswersEvery(group.second.desired, packets)) {
                groupStatus = RigExecOutputStatus::Pending;
                groupReason = "the frame's run answered " +
                              std::to_string(answered) + " of " +
                              std::to_string(group.second.desired.size()) +
                              " demanded outputs; the rest waits for the "
                              "next pump";
            }
            group.second.status = groupStatus;
            group.second.reason = groupReason;
            if (status == RigExecOutputStatus::Ready &&
                groupStatus != RigExecOutputStatus::Ready) {
                frameStatus = groupStatus;
                frameReason = groupReason;
            }
        }
        _QueueEventsLocked(request.rig, job->coverage.time,
                           job->key.generation.programIdentity, job->affinity,
                           frameStatus, frameReason, answered,
                           job->outputs.size());
    }

    // One event per live client of the frame. Pump drops an event whose client
    // closed before it ran. A client whose handles were minted at another
    // compile epoch, or by another compilation of this epoch, is not answered
    // by this run and gets no event: its poll already says the rig rebuilt
    // under it.
    void _QueueEventsLocked(
        const SdfPath &rig, const RigExecOutputTime &time,
        uint64_t compileEpoch,
        const std::shared_ptr<const RigExecProgramAffinity> &affinity,
        RigExecOutputStatus status, const std::string &reason, size_t answered,
        size_t demanded)
    {
        for (const auto &entry : _clients) {
            const Client &client = *entry.second;
            if (!(client.rig == rig) || client.compileEpoch != compileEpoch ||
                client.affinity != affinity) {
                continue;
            }
            size_t handles = 0;
            for (const RigExecOutputTimeDemand &demand :
                 client.request.demands) {
                if (demand.time == time) {
                    for (const RigExecOutputHandle &handle : demand.handles) {
                        if (!_Refused(client, handle)) {
                            ++handles;
                        }
                    }
                    break;
                }
            }
            if (!handles) {
                continue;
            }
            RigExecSubscriptionEvent event;
            event.subscription = client.id;
            event.rig = rig;
            event.time = time;
            event.status = status;
            event.reason = reason;
            event.answered = answered;
            event.demanded = demanded ? demanded : handles;
            _events.push_back(std::move(event));
        }
    }

    mutable std::mutex _mutex;
    RigExecSubscriptionHost _host;
    std::map<uint64_t, std::shared_ptr<Client>> _clients;
    std::map<_GroupKey, Group> _groups;
    std::map<RigExecExplicitJobKey, std::shared_ptr<Demand>> _demands;
    std::map<_PinKey, Pin> _pins;
    std::vector<RigExecCoverageLease> _stagedPins;
    std::vector<RigExecSubscriptionEvent> _events;
    RigExecSubscriptionCallback _callback;
    RigExecSubscriptionStats _stats;
    uint64_t _nextId = 0;
    bool _stopping = false;
};

RigExecSubscriptionHandle::~RigExecSubscriptionHandle()
{
    Close();
}

void
RigExecSubscriptionHandle::Close()
{
    if (!_state || _id == 0) {
        return;
    }
    _state->Close(_id);
    _id = 0;
}

RigExecOutputSubscriptionService::RigExecOutputSubscriptionService(
    RigExecSubscriptionHost host)
    : _state(std::make_shared<RigExecSubscriptionState>(std::move(host)))
{
}

RigExecOutputSubscriptionService::~RigExecOutputSubscriptionService()
{
    if (_state) {
        _state->Shutdown();
    }
}

RigExecOutputSubscriptionService &
RigExecOutputSubscriptionService::operator=(
    RigExecOutputSubscriptionService &&other)
{
    if (this != &other) {
        // Live handles and queued work keep the replaced state alive past this
        // object, so its own destructor never runs: retire it here.
        if (_state) {
            _state->Shutdown();
        }
        _state = std::move(other._state);
    }
    return *this;
}

RigExecSubscriptionHandle
RigExecOutputSubscriptionService::Subscribe(const SdfPath &rig,
                                            const RigExecOutputRequest &request,
                                            RigExecSubscriptionReceipt *receipt)
{
    const RigExecSubscriptionReceipt result = _state->Subscribe(rig, request);
    if (receipt) {
        *receipt = result;
    }
    if (!result.IsAccepted()) {
        return RigExecSubscriptionHandle();
    }
    return RigExecSubscriptionHandle(_state, result.subscription);
}

RigExecSubscriptionReceipt
RigExecOutputSubscriptionService::UpdateSubscription(
    const RigExecSubscriptionHandle &subscription,
    const RigExecOutputRequest &request)
{
    return _state->Update(subscription.Id(), request);
}

RigExecSubscriptionReceipt
RigExecOutputSubscriptionService::UpdateSubscription(
    uint64_t subscription, const RigExecOutputRequest &request)
{
    return _state->Update(subscription, request);
}

std::vector<RigExecOutputPacket>
RigExecOutputSubscriptionService::PollResults(
    const RigExecSubscriptionHandle &subscription)
{
    return _state->Poll(subscription.Id());
}

std::vector<RigExecOutputPacket>
RigExecOutputSubscriptionService::PollResults(uint64_t subscription)
{
    return _state->Poll(subscription);
}

size_t
RigExecOutputSubscriptionService::PumpCallbacks()
{
    return _state->Pump();
}

void
RigExecOutputSubscriptionService::SetCompletionCallback(
    RigExecSubscriptionCallback callback)
{
    _state->SetCallback(std::move(callback));
}

void
RigExecOutputSubscriptionService::Close(
    const RigExecSubscriptionHandle &subscription)
{
    _state->Close(subscription.Id());
}

void
RigExecOutputSubscriptionService::Close(uint64_t subscription)
{
    _state->Close(subscription);
}

void
RigExecOutputSubscriptionService::CloseAll()
{
    _state->CloseAll();
}

RigExecSubscriptionStats
RigExecOutputSubscriptionService::Stats() const
{
    return _state->Stats();
}

bool
RigExecOutputSubscriptionService::IsStopping() const
{
    return _state->IsStopping();
}

void
RigExecSetExplicitPublishFenceProbeForTesting(std::function<void()> probe)
{
    _sPublishFenceProbe = std::move(probe);
}

size_t
RigExecEstimateCaptureBytes(const RigExecFrameInputs &inputs)
{
    size_t bytes = inputs.values.size() * sizeof(RigExecSampledInput);
    for (const std::vector<VtValue> &row : inputs.layoutLeaves) {
        bytes += row.size() * sizeof(VtValue) + sizeof(std::vector<VtValue>);
    }
    for (const std::vector<SdfPath> &row : inputs.layoutSourcePaths) {
        bytes += row.size() * sizeof(SdfPath) + sizeof(std::vector<SdfPath>);
    }
    for (const std::vector<VtValue> &row : inputs.revisionLeaves) {
        bytes += row.size() * sizeof(VtValue) + sizeof(std::vector<VtValue>);
    }
    for (const std::vector<VtValue> &row : inputs.derivedLeaves) {
        bytes += row.size() * sizeof(VtValue) + sizeof(std::vector<VtValue>);
    }
    bytes += inputs.overridePaths.size() * sizeof(SdfPath);
    bytes += inputs.overrides.size() * sizeof(RigExecValueOverride);
    bytes += inputs.upstream.size() * sizeof(RigExecUpstreamValue);
    return bytes;
}

} // namespace rigExec
