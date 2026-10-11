// Copy-on-write vertex groups of a range-pipelined point chain: immutable
// published runs of points shared by refcount, and the single-writer protocol
// that keeps their content versions exact. Header-only and USD-free: the
// native program and the zero-USD runtime include one definition.
#ifndef RIGEXEC_MATH_POINT_BLOCKS_H
#define RIGEXEC_MATH_POINT_BLOCKS_H

#include <atomic>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

namespace rigExec {

/// Points [bound, bound + count) of a chain version as one writer published
/// them. Immutable while `owner` is held; copying a ref is a refcount, never a
/// copy of the points. `data` points into `owner`'s storage.
template <class Point>
struct RigExecPointsRef {
    std::shared_ptr<const void> owner;
    const Point *data = nullptr;
    size_t count = 0;
};

/// Same count and same bytes (memcmp: NaN payloads and signed zeros count);
/// identical pointers short-circuit.
template <class Point>
inline bool
RigExecPointsBitsEqual(const Point *a, size_t na, const Point *b, size_t nb)
{
    if (na != nb) return false;
    if (a == b || na == 0) return true;
    return std::memcmp(a, b, na * sizeof(Point)) == 0;
}

/// A publication's content id: the writer's group slot (program-local; the
/// chain base's group g is -1 - g) and the version that slot carried. Ids
/// compare only within one program numbering.
inline constexpr int64_t kRigExecNoGroupSource = INT64_MIN;
struct RigExecGroupSource {
    int64_t slot = kRigExecNoGroupSource;
    uint64_t version = 0;
    bool operator==(const RigExecGroupSource &o) const
    { return slot == o.slot && version == o.version; }
    bool operator!=(const RigExecGroupSource &o) const { return !(*this == o); }
};

/// What one group's sparse per-point walk last computed, entry by entry. A
/// walk whose result for an entry is a pure function of its per-revision
/// inputs, the entry's weight and the entry's entering point reuses that
/// result while all of them hold the bits they held (memcmp: NaN payloads and
/// signed zeros count). A rerun that only forwards points moved elsewhere in
/// the group then recomputes nothing. Holds what it compares, so it stands
/// on no publication, reset or run order; a copy is independent.
///
/// Entries are recorded only by a walk whose per-revision inputs are the
/// last walk's: a mover whose inputs move on every run (playback) records
/// nothing and pays one compare of those inputs per walk.
template <class Point>
struct RigExecSparseWalkMemo {
    /// The last walk's per-revision inputs, as its kernel spells them.
    std::vector<unsigned char> params;
    /// The recorded entries; empty while none are.
    std::vector<int> indices;
    std::vector<float> weights;
    std::vector<Point> in, out;
    /// Entries the last walk computed and reused. Observability only.
    uint32_t computed = 0, reused = 0;
};

/// What a walk does with its memo (RigExecSparseWalkBegin).
enum class RigExecSparseWalkMode {
    /// Its per-revision inputs moved: compute every entry, record none.
    Compute,
    /// They held and nothing is recorded: compute and record every entry.
    Record,
    /// They held over recorded entries: reuse each one that still answers
    /// (RigExecSparseWalkHolds), compute and record the others.
    Reuse,
};

/// One vertex group of one writer. Two writers may share a state only as a
/// speculative chunk and its fuse: the chunk owns `own`, `ownComputed`,
/// `computedVersion`, `computedRan`, `ok` and `walk`; the fuse owns
/// `published`, `version`, `ran`, `passedFrom` and `ownPublished`. A copy (a
/// program clone) shares buffers. Versions never go down.
template <class Point>
struct RigExecGroupState {
    RigExecPointsRef<Point> published;
    /// Bumped exactly when `published` holds other bytes than the previous
    /// publication, and on the first publication after a reset.
    uint64_t version = 0;
    bool ran = false;
    /// The kernel's answer for the group, sticky across skipped runs.
    bool ok = false;
    /// While `published` shares another writer's ref: that publication.
    RigExecGroupSource passedFrom;
    std::shared_ptr<std::vector<Point>> own[2];
    /// The own buffer `published` is, or -1 (it shares a source).
    int ownPublished = -1;
    /// A speculative writer's last computed result, kept while its fuse
    /// passes the group through; -1 none. Bumps `computedVersion` exactly when
    /// its bytes move.
    int ownComputed = -1;
    uint64_t computedVersion = 0;
    bool computedRan = false;
    /// A group step's sparse walk over this group (RigExecSparseWalkMemo).
    RigExecSparseWalkMemo<Point> walk;
};

/// Readies \p memo for a walk of \p entries entries whose per-revision
/// inputs are the \p size bytes at \p params, and answers how the walk uses
/// it. Compute keeps the bytes and forgets every entry; Record sizes it for
/// this walk with no entry answering yet. Clears the counts.
template <class Point>
RigExecSparseWalkMode RigExecSparseWalkBegin(
    RigExecSparseWalkMemo<Point> *memo, const void *params, size_t size,
    size_t entries);

/// Whether entry \p j of a memo in the Reuse mode was computed for point
/// \p index at \p weight from entering point \p point: then `out[j]` is the
/// walk's result for it.
template <class Point>
bool RigExecSparseWalkHolds(const RigExecSparseWalkMemo<Point> &memo, size_t j,
                            int index, float weight, const Point &point);

/// Records entry \p j's inputs ahead of computing it; the caller stores the
/// result in `out[j]`.
template <class Point>
void RigExecSparseWalkNote(RigExecSparseWalkMemo<Point> *memo, size_t j,
                           int index, float weight, const Point &point);

/// An own buffer index the writer may overwrite with \p count points: never
/// `ownPublished` or `ownComputed`. If both are taken and differ (a chunk ran
/// but its fuse did not), the computed result is forgotten first
/// (`ownComputed = -1`, `computedRan = false`, so the next
/// RigExecNoteComputedGroup bumps) and its buffer is the candidate. A
/// candidate is reused only when nobody else holds it (`use_count() == 1`,
/// then an acquire fence), else replaced by a new buffer of \p count points.
template <class Point>
int RigExecGroupScratch(RigExecGroupState<Point> *state, size_t count);

/// Publishes own buffer \p k. Bumps `version` on the first publication since a
/// reset or when the bytes differ from `published` (compared first; on
/// equality `published` is kept). Returns whether it bumped.
template <class Point>
bool RigExecPublishOwnGroup(RigExecGroupState<Point> *state, int k);

/// Publishes \p source shared (a pass-through). If the previous publication
/// passed through the same slot, the version moves exactly when
/// \p from.version differs from the one it carried; otherwise the bytes are
/// compared. Returns whether it bumped.
template <class Point>
bool RigExecPublishPassedGroup(RigExecGroupState<Point> *state,
                               const RigExecPointsRef<Point> &source,
                               RigExecGroupSource from);

/// A speculative writer records own buffer \p k as its computed result:
/// `computedVersion` bumps on the first one since a reset or when the bytes
/// differ from the kept result; on equality \p k is dropped and the kept one
/// stays. Returns whether it bumped.
template <class Point>
bool RigExecNoteComputedGroup(RigExecGroupState<Point> *state, int k);

/// Forgets every publication and computed result (a reset, a count change):
/// clears `ran`, `computedRan`, `passedFrom`, `ownPublished` and
/// `ownComputed`; keeps the own buffers and never lowers `version` or
/// `computedVersion`. The next publication and result bump.
template <class Point>
void RigExecResetGroup(RigExecGroupState<Point> *state);

/// After a program clone's memberwise copy: drops the own buffers neither
/// published nor computed, so the source's writer still finds its scratch
/// unique; the clone allocates on its first write.
template <class Point>
void RigExecDropGroupSpare(RigExecGroupState<Point> *state);

/// Whether groups [0, groupCount) are slices of one buffer at their bounds
/// (\p bounds has groupCount + 1 entries); then \p *start is its first point.
template <class Point>
bool RigExecGroupsContiguous(const RigExecPointsRef<Point> *const *groups,
                             const int *bounds, size_t groupCount,
                             const Point **start);

/// Copies the groups into \p out[0, bounds[groupCount] - bounds[0]).
template <class Point>
void RigExecGatherGroups(const RigExecPointsRef<Point> *const *groups,
                         const int *bounds, size_t groupCount, Point *out);

// Definitions. Single writer per half of a state (see RigExecGroupState):
// nothing here takes a lock; the only synchronization is the refcount and
// the acquire fence that makes a released holder's reads happen before the
// writer overwrites the buffer.

template <class Point>
RigExecSparseWalkMode
RigExecSparseWalkBegin(RigExecSparseWalkMemo<Point> *memo, const void *params,
                       size_t size, size_t entries)
{
    memo->computed = 0;
    memo->reused = 0;
    const unsigned char *bytes = static_cast<const unsigned char *>(params);
    if (memo->params.size() != size ||
        (size != 0 && std::memcmp(memo->params.data(), bytes, size) != 0)) {
        memo->params.assign(bytes, bytes + size);
        memo->indices.clear();
        memo->weights.clear();
        memo->in.clear();
        memo->out.clear();
        return RigExecSparseWalkMode::Compute;
    }
    if (entries != 0 && memo->indices.size() == entries &&
        memo->weights.size() == entries && memo->in.size() == entries &&
        memo->out.size() == entries) {
        return RigExecSparseWalkMode::Reuse;
    }
    // No point has a negative index, so no entry answers until noted.
    memo->indices.assign(entries, -1);
    memo->weights.assign(entries, 0.0f);
    memo->in.resize(entries);
    memo->out.resize(entries);
    return RigExecSparseWalkMode::Record;
}

template <class Point>
bool
RigExecSparseWalkHolds(const RigExecSparseWalkMemo<Point> &memo, size_t j,
                       int index, float weight, const Point &point)
{
    return memo.indices[j] == index &&
           std::memcmp(&memo.weights[j], &weight, sizeof(float)) == 0 &&
           std::memcmp(&memo.in[j], &point, sizeof(Point)) == 0;
}

template <class Point>
void
RigExecSparseWalkNote(RigExecSparseWalkMemo<Point> *memo, size_t j, int index,
                      float weight, const Point &point)
{
    memo->indices[j] = index;
    memo->weights[j] = weight;
    memo->in[j] = point;
}

template <class Point>
int
RigExecGroupScratch(RigExecGroupState<Point> *state, size_t count)
{
    // A chunk ran but its fuse did not: both buffers are taken by different
    // halves. The kept result is dropped rather than overwriting a buffer the
    // fuse may still publish; its writer recomputes and bumps.
    if (state->ownPublished >= 0 && state->ownComputed >= 0 &&
        state->ownPublished != state->ownComputed) {
        state->ownComputed = -1;
        state->computedRan = false;
    }
    int candidate = -1;
    for (int k = 0; k < 2; ++k) {
        if (k == state->ownPublished || k == state->ownComputed) {
            continue;
        }
        if (candidate < 0) {
            candidate = k;
        }
        // Prefer a free buffer nobody else holds: no allocation.
        if (state->own[k] && state->own[k].use_count() == 1) {
            candidate = k;
            break;
        }
    }
    std::shared_ptr<std::vector<Point>> &buffer = state->own[candidate];
    if (buffer && buffer.use_count() == 1) {
        // Every other holder released it (an acq_rel decrement): their reads
        // of the old bytes happen before this writer's writes.
        std::atomic_thread_fence(std::memory_order_acquire);
        if (buffer->size() != count) {
            buffer->resize(count);
        }
    } else {
        buffer = std::make_shared<std::vector<Point>>(count);
    }
    return candidate;
}

template <class Point>
bool
RigExecPublishOwnGroup(RigExecGroupState<Point> *state, int k)
{
    const std::vector<Point> &bytes = *state->own[k];
    const bool moved =
        !state->ran ||
        !RigExecPointsBitsEqual(state->published.data, state->published.count,
                                bytes.data(), bytes.size());
    if (moved) {
        state->published.owner = state->own[k];
        state->published.data = bytes.data();
        state->published.count = bytes.size();
        state->ownPublished = k;
        state->passedFrom = RigExecGroupSource();
        ++state->version;
    }
    // On equality the earlier publication (own or passed) stands as it is,
    // and so does what it passed from.
    state->ran = true;
    return moved;
}

template <class Point>
bool
RigExecPublishPassedGroup(RigExecGroupState<Point> *state,
                          const RigExecPointsRef<Point> &source,
                          RigExecGroupSource from)
{
    bool moved = true;
    if (state->ran) {
        if (from.slot != kRigExecNoGroupSource &&
            state->passedFrom.slot == from.slot) {
            // One writer's slot: its version moves exactly with its bytes.
            moved = state->passedFrom.version != from.version;
        } else {
            moved = !RigExecPointsBitsEqual(state->published.data,
                                            state->published.count,
                                            source.data, source.count);
        }
    }
    state->published = source;
    state->passedFrom = from;
    state->ownPublished = -1;
    if (moved) {
        ++state->version;
    }
    state->ran = true;
    return moved;
}

template <class Point>
bool
RigExecNoteComputedGroup(RigExecGroupState<Point> *state, int k)
{
    bool moved = !state->computedRan || state->ownComputed < 0;
    if (!moved) {
        const std::vector<Point> &kept = *state->own[state->ownComputed];
        const std::vector<Point> &bytes = *state->own[k];
        moved = !RigExecPointsBitsEqual(kept.data(), kept.size(),
                                        bytes.data(), bytes.size());
    }
    if (moved) {
        state->ownComputed = k;
        ++state->computedVersion;
    }
    state->computedRan = true;
    return moved;
}

template <class Point>
void
RigExecResetGroup(RigExecGroupState<Point> *state)
{
    // `published` keeps its ref until the next publication replaces it (that
    // one bumps, `ran` being false); the ref it holds keeps its buffer from
    // being handed out as a scratch.
    state->ran = false;
    state->computedRan = false;
    state->passedFrom = RigExecGroupSource();
    state->ownPublished = -1;
    state->ownComputed = -1;
}

template <class Point>
void
RigExecDropGroupSpare(RigExecGroupState<Point> *state)
{
    for (int k = 0; k < 2; ++k) {
        if (k != state->ownPublished && k != state->ownComputed) {
            state->own[k].reset();
        }
    }
}

template <class Point>
bool
RigExecGroupsContiguous(const RigExecPointsRef<Point> *const *groups,
                        const int *bounds, size_t groupCount,
                        const Point **start)
{
    // One owner (one allocation) and each group at its bound's offset from
    // the first point; empty groups place nothing.
    const RigExecPointsRef<Point> *first = nullptr;
    const Point *base = nullptr;
    for (size_t g = 0; g < groupCount; ++g) {
        const RigExecPointsRef<Point> &ref = *groups[g];
        const size_t width = size_t(bounds[g + 1] - bounds[g]);
        if (ref.count != width) {
            return false;
        }
        if (width == 0) {
            continue;
        }
        const size_t offset = size_t(bounds[g] - bounds[0]);
        if (!first) {
            // Every group before this one is empty, so its offset is 0.
            first = &ref;
            base = ref.data;
        } else if (ref.owner.owner_before(first->owner) ||
                   first->owner.owner_before(ref.owner) ||
                   reinterpret_cast<uintptr_t>(ref.data) -
                           reinterpret_cast<uintptr_t>(base) !=
                       offset * sizeof(Point)) {
            return false;
        }
    }
    *start = base;
    return true;
}

template <class Point>
void
RigExecGatherGroups(const RigExecPointsRef<Point> *const *groups,
                    const int *bounds, size_t groupCount, Point *out)
{
    for (size_t g = 0; g < groupCount; ++g) {
        const RigExecPointsRef<Point> &ref = *groups[g];
        const size_t width = size_t(bounds[g + 1] - bounds[g]);
        // A group shorter than its bounds leaves the rest of its slice.
        const size_t n = ref.count < width ? ref.count : width;
        if (n) {
            std::memcpy(out + (bounds[g] - bounds[0]), ref.data,
                        n * sizeof(Point));
        }
    }
}

}  // namespace rigExec

#endif
