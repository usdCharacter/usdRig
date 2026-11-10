// Freshness proof contract implementation: building the dependency set a
// control digest covers, and the per-control coverage query.
#include "bridgeFreshness.h"

#include "rigExec/frozenContext.h"
#include "rigExec/outputAffectedIndex.h"

#include <algorithm>
#include <set>

namespace rigExec {

RigExecFreshProof::RigExecFreshProof(
    uint64_t digestIn, uint64_t unfoldedIn, const RigExecFrameInputs *inputs,
    const std::vector<RigExecValueOverride> *overrides)
    : digest(digestIn), unfolded(unfoldedIn), sampledInputs(inputs != nullptr)
{
    // The dependency set the digest covers: first-win sample paths (the
    // digest's own granularity) plus standing override identities.
    // Stage seeds fold no path string, so they contribute none: a notice
    // that moves only seeds leaves the proof standing, and the digest
    // compare -- which does fold them -- misses honestly on the visit.
    // A matching recorded order is immutable and shared, and its first-win
    // paths are exactly the vector's sample paths, so it stands in for
    // their text.
    if (inputs && RigExecFrameDigestOrderMatches(inputs->digestOrder.get(),
                                                 inputs->values)) {
        order = inputs->digestOrder;
    }
    std::set<std::string> ids;
    if (inputs && !order) {
        for (const RigExecSampledInput &sample : inputs->values) {
            if (!sample.path.IsEmpty()) {
                ids.insert(sample.path.GetString());
            }
        }
    }
    if (overrides) {
        for (const RigExecValueOverride &o : *overrides) {
            ids.insert(RigExecControlIdForOverride(o));
        }
    }
    paths.assign(ids.begin(), ids.end());
}

bool
RigExecFreshProof::Covers(const std::string &id, const SdfPath &asPath) const
{
    // Empty paths are no dependency on either side: `paths` never holds
    // one, and an empty \p asPath is not looked up.
    return std::binary_search(paths.begin(), paths.end(), id) ||
           (order && !asPath.IsEmpty() &&
            RigExecFrameDigestOrderHasPath(order.get(), asPath));
}

} // namespace rigExec
