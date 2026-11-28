// Value contract: the addresses production code names rig values by and the
// overrides it supplies. Plain data types.
#ifndef RIGEXEC_VALUE_ADDRESS_H
#define RIGEXEC_VALUE_ADDRESS_H

#include "pxr/pxr.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/timeCode.h"

#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

const TfToken &RigExecFinalPhase();

/// Value address.
///
/// A prim target with publicComputation names a rig value provider
/// (e.g. computePointFrame). A property target with an empty
/// publicComputation names the property's value. A public address never
/// names a generated provider.
struct RigExecValueAddress {
    SdfPath target;
    TfToken publicComputation;
    /// base, afterMover, or final (informational public identity).
    TfToken phase = RigExecFinalPhase();

    static RigExecValueAddress Prim(const SdfPath &path,
                                    const TfToken &computation,
                                    const TfToken &phase = RigExecFinalPhase())
    {
        return {path, computation, phase};
    }
    static RigExecValueAddress
    Property(const SdfPath &propertyPath,
             const TfToken &phase = RigExecFinalPhase())
    {
        return {propertyPath, TfToken(), phase};
    }
};

/// A value the engine supplies for a prim computation or an attribute.
struct RigExecValueOverride {
    SdfPath prim;
    /// Prim computation to override. Ignored when \p attribute is set.
    TfToken computation;
    /// When non-empty, overrides this attribute's value on \p prim instead
    /// of a prim computation.
    TfToken attribute;
    VtValue value;

    /// Exact equality: the burst cache validates its prepared overrides
    /// against the caller's per frame, and a differing override list means
    /// differing placement, differing samples, and a differing digest.
    bool operator==(const RigExecValueOverride &other) const
    {
        return prim == other.prim && computation == other.computation &&
               attribute == other.attribute && value == other.value;
    }

    bool operator!=(const RigExecValueOverride &other) const
    {
        return !(*this == other);
    }
};

} // namespace rigExec

#endif // RIGEXEC_VALUE_ADDRESS_H
