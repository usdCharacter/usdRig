// Freshness proofs: the digest contract one frame-cache lookup carries.
// The proof map itself stays with the bridge's cache state; this header
// owns only the proof value type and its dependency-set semantics.
#ifndef RIGEXEC_IMAGING_BRIDGE_FRESHNESS_H
#define RIGEXEC_IMAGING_BRIDGE_FRESHNESS_H

#include "rigExec/frameCache.h"

#include "pxr/usd/sdf/path.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// One frame's freshness proof: the folded control digest a
/// lookup must reproduce, the unfolded digest it folds from, and the
/// dependency set the digest covers -- first-win sample paths plus
/// standing override identities. The sample paths are the vector's shared
/// digest order when it matched, so recording a proof builds no per-sample
/// text. An edit retires the proofs whose sets intersect its affected
/// inputs instead of clearing the map wholesale; a constant-namespace carry
/// re-points a surviving proof by re-folding its unfolded half. The digest
/// compare stays the correctness backstop: an over-retained proof only
/// costs a sample and a miss, never a wrong pose.
struct RigExecFreshProof {
    uint64_t digest = 0;
    uint64_t unfolded = 0;
    bool sampledInputs = false;
    /// The sampled vector's recorded digest order when it matched the
    /// vector: its first-win sample paths are the proof's sample
    /// dependencies. Null otherwise.
    std::shared_ptr<const RigExecFrameDigestOrder> order;
    /// Sorted, unique: the standing overrides' control ids, plus, only when
    /// `order` is null, every first-win non-empty sample path's text.
    std::vector<std::string> paths;
    RigExecFreshProof() = default;
    RigExecFreshProof(uint64_t digestIn, uint64_t unfoldedIn,
                      const RigExecFrameInputs *inputs,
                      const std::vector<RigExecValueOverride> *overrides);
    /// Whether control \p id (canonical path \p asPath, empty when \p id is
    /// not one) is in the proof's dependency set.
    bool Covers(const std::string &id, const SdfPath &asPath) const;
};

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_FRESHNESS_H
