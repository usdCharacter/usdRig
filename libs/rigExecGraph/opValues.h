#ifndef RIGEXEC_GRAPH_OP_VALUES_H
#define RIGEXEC_GRAPH_OP_VALUES_H

#include "opGraph.h"
#include "providerRecords.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace rigExec {

/// The adapter retains one exact key for each produced version, including
/// presence, element count and error state. Scratch capacity survives runs.
struct RigExecOpValueState {
    uint32_t domain = 0, slot = 0;
    std::string key, scratch;
    bool initialized = false;
    char changed = 0;
    uint64_t revision = 0;
};

struct RigExecOpAdapterState {
    std::vector<RigExecOpValueState> values;
    std::vector<RigExecValueId> leaves, changedLeaves;
    /// Every value whose changed flag the last run set: its changed leaves
    /// and the changed writes of its ops that ran. Gathered by the owner
    /// after the join; the next run clears exactly these flags.
    std::vector<RigExecValueId> changedValues;
    std::vector<RigExecValueId> excludedValues;
    /// RigExecOpCallbacks::skipEffects, filled by the backend at compile.
    std::vector<uint32_t> skipEffects;
    std::vector<uint32_t> seeds, candidateOps;
    std::vector<uint64_t> inputRevisions;
    std::vector<std::string> inputKeys, inputScratch;
    std::vector<std::string> sourceKeys, sourceScratch;
    std::vector<std::vector<uint32_t>> coveredPropertyInputs;
    std::vector<std::vector<std::pair<uint32_t,uint32_t>>> coveredTypedInputs;
    std::vector<char> inputExact;
    /// Native retained pure bodies are candidates on their replacement compiled rig first run.
    std::vector<char> retainedFirst;
    /// Per op, set at compile: its source key reads no state that can change,
    /// so a run that rebuilds no source keys treats it as an equal compare.
    std::vector<char> constantSource;
    /// Ascending ops such a run still visits: keyed ones and those that always run.
    std::vector<uint32_t> sourceVisits;
    bool compiled = false, everRan = false;
    bool parallel = false, measuring = false;
    /// Native: rebuild every constant source key each run and verify it.
    bool verifyConstantSources = false;
    /// Native: the generation of the leaf values `sourceKeys` were built
    /// from (RigExecBakedSourceWatch::serial); 0 when none.
    uint64_t sourceWatchSerial = 0;
    /// Native (RIGEXEC_VERIFY_LEAF_VERSIONS): beside each key that carries
    /// path-leaf content versions, the same key over the leaves' contents,
    /// and per op whether the two told a different change this run. Each op
    /// writes only its own entries; the owner reports after the join.
    bool verifyLeafVersions = false;
    std::vector<std::string> contentSourceKeys, contentInputKeys;
    std::vector<char> leafVersionMismatch;
};

/// The steps whose output (diagnostics, lines) an epilogue publishes is not
/// empty, so publication visits those rather than every step. Owner thread
/// only: bodies write their own step's output, and the owner notes each op
/// that ran after the join. `holding` is exact per step; `held` lists every
/// holding step and, until Ascending() tidies it, may also repeat one or name
/// one that no longer holds output.
struct RigExecHeldSteps {
    std::vector<char> holding;
    std::vector<uint32_t> held;
    bool ascending = true;

    void Reset(size_t count)
    {
        holding.assign(count, 0);
        held.clear();
        ascending = true;
    }
    void Note(uint32_t step, bool holds)
    {
        if ((holding[step] != 0) == holds) return;
        holding[step] = holds ? 1 : 0;
        if (holds) {
            ascending = ascending && (held.empty() || held.back() < step);
            held.push_back(step);
        } else {
            ascending = false;
        }
    }
    /// The holding steps in step order.
    const std::vector<uint32_t> &Ascending()
    {
        if (!ascending) {
            held.erase(std::remove_if(held.begin(), held.end(),
                [this](uint32_t step) { return !holding[step]; }), held.end());
            std::sort(held.begin(), held.end());
            held.erase(std::unique(held.begin(), held.end()), held.end());
            ascending = true;
        }
        return held;
    }
};

inline RigExecValueId RigExecOpAddValue(RigExecOpAdapterState *, uint32_t, uint32_t);

/// A (domain, slot) pair's value id and writer op, dense where the ids
/// allow. Backends number slots from zero per domain, so a vector per
/// domain answers in O(1) with no per-entry node. Growth past a small
/// entry budget, or a domain past the dense width, spills to maps, so no
/// input -- however wild its ranges -- can size a vector it does not
/// need. Misses read back as kNoValue/kNoWriter on either path, and ids
/// are still assigned in first-visit order, so the table answers exactly
/// what the two maps it replaces answered.
struct RigExecOpTypedTable {
    static constexpr RigExecValueId kNoValue = ~RigExecValueId(0);
    static constexpr uint32_t kNoWriter = ~uint32_t(0);
    // Real backends stay an order of magnitude inside both: dozens of
    // domains, slots below the backend arrays' sizes. Past them the map
    // is the original speed, never a failure.
    static constexpr uint32_t kDenseDomains = 64;
    static constexpr size_t kDenseEntries = size_t(1) << 20;
    std::vector<std::vector<RigExecValueId>> valueOf;
    std::vector<std::vector<uint32_t>> writerOf;
    std::map<std::pair<uint32_t, uint32_t>, RigExecValueId> spillValues;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> spillWriters;
    size_t denseEntries = 0;
    const std::vector<RigExecValueId> *denseValues(uint32_t d) const
    {
        return d < valueOf.size() ? &valueOf[d] : nullptr;
    }
    const std::vector<uint32_t> *denseWriters(uint32_t d) const
    {
        return d < writerOf.size() ? &writerOf[d] : nullptr;
    }
    RigExecValueId value(uint32_t d, uint32_t s) const
    {
        // Spill first: a set files exactly one path, on a miss from both,
        // so a pair lives in at most one of them whatever later growth
        // does. The empty check keeps the dense path one branch.
        if (!spillValues.empty()) {
            const auto found = spillValues.find(std::make_pair(d, s));
            if (found != spillValues.end()) return found->second;
        }
        if (const auto *row = denseValues(d))
            if (s < row->size()) return (*row)[s];
        return kNoValue;
    }
    uint32_t writer(uint32_t d, uint32_t s) const
    {
        if (!spillWriters.empty()) {
            const auto found = spillWriters.find(std::make_pair(d, s));
            if (found != spillWriters.end()) return found->second;
        }
        if (const auto *row = denseWriters(d))
            if (s < row->size()) return (*row)[s];
        return kNoWriter;
    }
    // A slot the dense rows do not cover yet: grow them when the budget
    // allows, so the caller can file it dense. A slot that would break
    // the budget, like a domain past the dense width, stays spilled.
    bool cover(uint32_t d, uint32_t s)
    {
        if (d >= kDenseDomains) return false;
        if (d < valueOf.size() && s < valueOf[d].size()) return true;
        const size_t have =
            d < valueOf.size() ? valueOf[d].size() : 0;
        if (denseEntries + (size_t(s) + 1 - have) > kDenseEntries)
            return false;
        if (d >= valueOf.size()) {
            valueOf.resize(size_t(d) + 1);
            writerOf.resize(size_t(d) + 1);
        }
        if (s >= valueOf[d].size()) {
            denseEntries += size_t(s) + 1 - valueOf[d].size();
            valueOf[d].resize(size_t(s) + 1, kNoValue);
            writerOf[d].resize(size_t(s) + 1, kNoWriter);
        }
        return true;
    }
    void setValue(uint32_t d, uint32_t s, RigExecValueId id)
    {
        if (cover(d, s)) valueOf[d][s] = id;
        else spillValues.emplace(std::make_pair(d, s), id);
    }
    void setWriter(uint32_t d, uint32_t s, uint32_t w)
    {
        if (cover(d, s)) writerOf[d][s] = w;
        else spillWriters.emplace(std::make_pair(d, s), w);
    }
};

/// Preindex typed producers before binding any reads. Backends supply
/// their domain descriptors; the compiler alone owns canonical ordering.
template <class Step, class Domain, class Begin, class End, class Key, class Sampled>
inline bool RigExecOpCompileAdapter(const std::vector<Step> &steps,
    Domain domain, Begin begin, End end, Key key, Sampled sampled,
    RigExecCompiledGraph *graph, RigExecOpAdapterState *state,
    std::string *error, RigExecCyclePolicy cyclePolicy=RigExecCyclePolicy::Reject,
    const std::function<const std::vector<std::string> &(uint32_t)> &semanticPredecessors = {},
    const RigExecOpExclusionProof *retainedExclusions = nullptr)
{
    *state = RigExecOpAdapterState();
    RigExecOpTypedTable typed;
    std::vector<RigExecOpDescriptor> descriptors(steps.size());
    for (uint32_t i = 0; i < steps.size(); ++i) {
        const auto &step = steps[i]; auto &op = descriptors[i];
        op.key = key(step); op.kind = uint32_t(step.kind);
        op.volatileInput = step.externalReads;
        for (const auto &range : step.writes) {
            const uint32_t typedDomain = domain(range);
            const uint32_t first = begin(range), last = end(range);
            for (uint32_t slot = first; slot < last; ++slot) {
                const uint32_t prior = typed.writer(typedDomain, slot);
                if (prior != RigExecOpTypedTable::kNoWriter && prior != i) {
                    if (error) *error = "multiple typed producers for domain " +
                        std::to_string(typedDomain) + " slot " + std::to_string(slot) +
                        ": " + key(steps[prior]) + " and " + key(step);
                    return false;
                }
                if (prior == RigExecOpTypedTable::kNoWriter)
                    typed.setWriter(typedDomain, slot, i);
                RigExecValueId id = typed.value(typedDomain, slot);
                if (id == RigExecOpTypedTable::kNoValue) {
                    id = RigExecOpAddValue(state, typedDomain, slot);
                    typed.setValue(typedDomain, slot, id);
                }
                op.writes.push_back(id);
            }
        }
    }
    // Authored semantic prerequisites are distinct from generated graph
    // predecessors. Backends opt in with stable descriptor identities only.
    // Never import a historical scheduling list from Step.preds.
    if (semanticPredecessors) {
        // Count before indexing: most compiles bind no semantic edge, and
        // building the key index for an empty use is pure waste. A
        // handful of references scans cheaper than the index costs to
        // build; past that the index pays. All three spell the same
        // edges -- the first descriptor carrying the required key -- or
        // the same error.
        size_t references = 0;
        for (uint32_t i = 0; i < descriptors.size(); ++i)
            references += semanticPredecessors(i).size();
        if (references != 0 && references <= 16) {
            for (uint32_t i = 0; i < descriptors.size(); ++i)
                for (const auto &required : semanticPredecessors(i)) {
                    uint32_t found = descriptors.size();
                    for (uint32_t j = 0; j < descriptors.size(); ++j)
                        if (descriptors[j].key == required) { found = j; break; }
                    if (found == descriptors.size()) {
                        if (error) *error = "semantic predecessor has no descriptor: " +
                            descriptors[i].key + " requires " + required;
                        return false;
                    }
                    descriptors[i].predecessors.push_back(found);
                }
        } else if (references != 0) {
            std::map<std::string, uint32_t> byKey;
            for (uint32_t i = 0; i < descriptors.size(); ++i)
                byKey.emplace(descriptors[i].key, i);
            for (uint32_t i = 0; i < descriptors.size(); ++i)
                for (const auto &required : semanticPredecessors(i)) {
                    const auto found = byKey.find(required);
                    if (found == byKey.end()) {
                        if (error) *error = "semantic predecessor has no descriptor: " +
                            descriptors[i].key + " requires " + required;
                        return false;
                    }
                    descriptors[i].predecessors.push_back(found->second);
                }
        }
    }
    for (uint32_t i = 0; i < steps.size(); ++i) {
        auto &op = descriptors[i];
        for (const auto &range : steps[i].reads) {
            const uint32_t typedDomain = domain(range);
            const uint32_t first = begin(range), last = end(range);
            for (uint32_t slot = first; slot < last; ++slot) {
                RigExecValueId id = typed.value(typedDomain, slot);
                if (id == RigExecOpTypedTable::kNoValue) {
                    if (!sampled(typedDomain,slot)) {
                        if(error) *error="read has no producer or declared sampled source: "+
                            key(steps[i])+" domain "+std::to_string(typedDomain)+
                            " slot "+std::to_string(slot);
                        return false;
                    }
                    id = RigExecOpAddValue(state,typedDomain,slot);
                    typed.setValue(typedDomain,slot,id);
                    state->leaves.push_back(id);
                }
                op.reads.push_back(id);
            }
        }
    }
    state->compiled = RigExecCompileOpGraph(descriptors, state->leaves,
        cyclePolicy, retainedExclusions, graph, error);
    if (state->compiled) {
        for(uint32_t i=0;i<descriptors.size();++i) if(graph->canonicalIndex[i]<0)
            for(auto id:descriptors[i].writes) state->excludedValues.push_back(id);
        state->leaves.insert(state->leaves.end(),state->excludedValues.begin(),state->excludedValues.end());
        std::vector<RigExecValueId> order(state->values.size()), remap(state->values.size());
        for (RigExecValueId i=0;i<order.size();++i) order[size_t(i)]=i;
        std::sort(order.begin(),order.end(),[&](RigExecValueId a,RigExecValueId b) {
            const auto &x=state->values[size_t(a)], &y=state->values[size_t(b)];
            return std::make_pair(x.domain,x.slot)<std::make_pair(y.domain,y.slot);
        });
        std::vector<RigExecOpValueState> canonical; canonical.reserve(order.size());
        for (RigExecValueId i=0;i<order.size();++i) {
            remap[size_t(order[size_t(i)])]=i;
            canonical.push_back(std::move(state->values[size_t(order[size_t(i)])]));
        }
        state->values=std::move(canonical);
        for (auto &id:state->leaves) id=remap[size_t(id)];
        for (auto &id:state->excludedValues) id=remap[size_t(id)];
        std::sort(state->leaves.begin(),state->leaves.end());
        state->leaves.erase(std::unique(state->leaves.begin(),state->leaves.end()),state->leaves.end());
        graph->readers.clear();
        for (uint32_t i=0;i<graph->ops.size();++i) {
            auto &op=graph->ops[i];
            for (auto &id:op.descriptor.reads) { id=remap[size_t(id)]; graph->readers[id].push_back(i); }
            for (auto &id:op.descriptor.writes) id=remap[size_t(id)];
            std::sort(op.descriptor.reads.begin(),op.descriptor.reads.end());
            std::sort(op.descriptor.writes.begin(),op.descriptor.writes.end());
            op.descriptor.reads.erase(std::unique(op.descriptor.reads.begin(),op.descriptor.reads.end()),op.descriptor.reads.end());
            op.descriptor.writes.erase(std::unique(op.descriptor.writes.begin(),op.descriptor.writes.end()),op.descriptor.writes.end());
        }
        for (auto &entry:graph->readers) entry.second.erase(std::unique(entry.second.begin(),entry.second.end()),entry.second.end());
    }
    return state->compiled;
}

inline RigExecValueId RigExecOpAddValue(RigExecOpAdapterState *state,
    uint32_t domain, uint32_t slot)
{
    const RigExecValueId id = state->values.size();
    state->values.push_back({domain, slot});
    return id;
}

/// Serialize fields explicitly: object padding and container addresses are
/// never part of a value's identity.
template <class T> inline void RigExecOpKeyAppend(std::string *key, const T &v)
{
    key->append(reinterpret_cast<const char *>(&v), sizeof(T));
}
inline void RigExecOpKeyAppend(std::string *key, const std::string &v)
{
    RigExecOpKeyAppend(key, v.size()); key->append(v);
}
/// Element types whose key bytes are exactly their object bytes: padding-free
/// and trivially copyable, so a contiguous run of them keys with one append.
/// Integers, float, double and std::array of them qualify here. A backend
/// opts in its fixed-size vector and matrix types, asserting their size, in
/// a header that every user of their keys includes.
template <class T> struct RigExecOpKeyBulkElement : std::integral_constant<bool,
    std::is_integral<T>::value || std::is_same<T, float>::value ||
    std::is_same<T, double>::value> {};
template <class T, size_t N> struct RigExecOpKeyBulkElement<std::array<T, N>>
    : RigExecOpKeyBulkElement<T> {
    static_assert(sizeof(std::array<T, N>) == N * sizeof(T),
                  "key runs need padding-free elements");
};
/// Containers whose elements are one contiguous array (std::vector<bool> is not).
template <class C, class = void> struct RigExecOpKeyContiguous : std::false_type {};
template <class C> struct RigExecOpKeyContiguous<C,
    std::void_t<decltype(std::declval<const C &>().data())>>
    : std::is_same<decltype(std::declval<const C &>().data()),
                   const typename C::value_type *> {};
/// Appends \p count elements as one run: the same bytes as appending each
/// element's fields in memory order, signed zeros and NaN payloads included.
template <class T> inline void RigExecOpKeyAppendRun(std::string *key,
    const T *data, size_t count)
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "key runs copy object bytes");
    if (count) key->append(reinterpret_cast<const char *>(data), count * sizeof(T));
}
template <class T> inline void RigExecOpKeyArray(std::string *key, const T &v)
{
    using Element = typename T::value_type;
    RigExecOpKeyAppend(key, v.size());
    if constexpr (RigExecOpKeyBulkElement<Element>::value &&
                  RigExecOpKeyContiguous<T>::value)
        RigExecOpKeyAppendRun(key, v.data(), v.size());
    else
        for (const auto &item : v) RigExecOpKeyAppend(key, static_cast<const Element &>(item));
}
template <class T> struct RigExecOpKeyIsVector : std::false_type {};
template <class T, class A> struct RigExecOpKeyIsVector<std::vector<T, A>> : std::true_type {};
/// A provider value's payload. Vector alternatives key by count and
/// contents, strings element by element; the variant index is the caller's.
inline void RigExecOpKeyPlainValue(std::string *key, const RigExecProviderPlainValue &value)
{
    std::visit([&](const auto &v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same<T, RigExecProviderPlainFrame>::value) {
            RigExecOpKeyAppend(key, v.points); RigExecOpKeyAppend(key, v.flags);
        } else if constexpr (RigExecOpKeyIsVector<T>::value) RigExecOpKeyArray(key, v);
        else if constexpr (!std::is_same<T, std::monostate>::value) RigExecOpKeyAppend(key, v);
    }, value);
}

template <class Sample> inline void RigExecOpPublishValue(
    RigExecOpValueState *value, Sample sample)
{
    value->scratch.clear(); sample(value->domain, value->slot, &value->scratch);
    value->changed = !value->initialized || value->scratch != value->key;
    if (value->changed) ++value->revision;
    value->key.swap(value->scratch); value->initialized = true;
}

/// A fixed-size value's key on the stack: each field's object bytes, in the
/// order its key appends them. A field past \p N sets `overflow` and writes
/// nothing; the caller then publishes through the key.
template <size_t N> struct RigExecOpSmallKey {
    char bytes[N];
    size_t size = 0;
    bool overflow = false;
    template <class T> void Append(const T &v)
    {
        static_assert(std::is_trivially_copyable<T>::value, "key fields copy object bytes");
        if (sizeof(T) > N - size) { overflow = true; return; }
        std::memcpy(bytes + size, &v, sizeof(T)); size += sizeof(T);
    }
};

/// Publishes \p size key bytes compared in place against the stored key:
/// the changed flag, revision and stored key RigExecOpPublishValue leaves
/// when its sample writes exactly these bytes.
inline void RigExecOpPublishKeyBytes(RigExecOpValueState *value,
    const char *bytes, size_t size)
{
    value->changed = !value->initialized || value->key.size() != size ||
        (size && std::memcmp(value->key.data(), bytes, size) != 0);
    if (value->changed) { ++value->revision; value->key.assign(bytes, size); }
    value->initialized = true;
}

/// Clears every change flag at the start of a run. Only publication sets a
/// flag, and the last run gathered each one it set into changedValues.
inline void RigExecOpClearChanges(RigExecOpAdapterState *state)
{
    for (RigExecValueId id : state->changedValues) state->values[size_t(id)].changed = 0;
    state->changedValues.clear();
}

/// Owner, after the join and on failure too: gathers the flags this run set,
/// from its changed leaves and the writes of the ops that ran. With
/// \p candidates (ascending; every op that ran is one), only those ops are
/// visited; the output order is the same.
inline void RigExecOpGatherChanges(RigExecOpAdapterState *state,
    const RigExecCompiledGraph &graph, const std::vector<char> &ran,
    const std::vector<uint32_t> *candidates = nullptr)
{
    state->changedValues.assign(state->changedLeaves.begin(), state->changedLeaves.end());
    const auto gather = [&](size_t c) {
        if (c >= graph.ops.size() || c >= ran.size() || !ran[c]) return;
        for (RigExecValueId id : graph.ops[c].descriptor.writes)
            if (state->values[size_t(id)].changed) state->changedValues.push_back(id);
    };
    if (candidates) for (uint32_t c : *candidates) gather(c);
    else for (size_t c = 0; c < graph.ops.size() && c < ran.size(); ++c) gather(c);
}

} // namespace rigExec
#endif
