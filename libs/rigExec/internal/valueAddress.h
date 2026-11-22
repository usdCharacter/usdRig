// Backend-neutral value contract: the addresses production code names rig
// values by, the overrides it supplies, and the extracted generation they
// resolve into. Plain data types; no extraction backend is named here.
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

/// Backend-neutral value address.
///
/// A prim target with publicComputation names a rig value provider
/// (e.g. computePointFrame). A property target with an empty
/// publicComputation resolves to the stock computeValue bridge. The
/// canonical public address never names a generated provider; the
/// compiler-private resolution (RigExecTapSet::AddResolved) maps a public
/// address to its hidden chain-head provider (generated
/// provider paths never enter a public or durable tap address).
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

using RigExecTapId = int;

/// A computed value the engine supplies rather than lets exec derive.
///
/// Used for the solver->joint pose. OpenExec can only reach a
/// provider through a relationship authored on the consuming prim, and the
/// authored direction here is the wrong way round: a solver names its joints,
/// not the reverse (there is no reverse-relationship accessor -- see the
/// accessor list in exec/computationBuilders.h). Supplying
/// the frame as an override keeps the binding in the compiled graph
/// where it belongs, and exec's own consumers -- computeMatrix, frame-chain
/// applications, and the NamespaceAncestor fallback that unbound descendants
/// follow -- all read the overridden value with nothing authored anywhere.
struct RigExecValueOverride {
    SdfPath prim;
    /// Prim computation to override. Ignored when \p attribute is set.
    TfToken computation;
    /// When non-empty, overrides this attribute's value on \p prim instead
    /// of a prim computation. Used for inputs an exec accessor cannot reach
    /// from the consuming prim -- a ribbon's driver-curve points live on the
    /// curve, and a relationship accessor can only request computations on
    /// the targets, not a named attribute of them.
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

/// Immutable extracted generation: one value per tap.
class RigExecSnapshot {
public:
    const VtValue &Get(RigExecTapId tap) const;

    template <class T> T Get(RigExecTapId tap) const
    {
        const VtValue &v = Get(tap);
        return v.IsHolding<T>() ? v.UncheckedGet<T>() : T();
    }

    /// Borrowed access to the tapped value without copying: null when the
    /// tap is missing or holds another type. The pointer stays valid while
    /// this snapshot is alive; callers needing ownership keep Get<T>.
    template <class T> const T *TryGet(RigExecTapId tap) const
    {
        const VtValue &v = Get(tap);
        return v.IsHolding<T>() ? &v.UncheckedGet<T>() : nullptr;
    }

    UsdTimeCode GetTime() const
    {
        return _time;
    }

    /// True when evaluation ran and produced this snapshot.
    bool IsValid() const
    {
        return _valid;
    }

    /// True when every requested tap produced a value. A valid snapshot
    /// with missing values means some computations failed to compile or
    /// evaluate; consumers must not treat defaulted values as results.
    bool IsComplete() const
    {
        return _valid && _complete;
    }

private:
    friend class RigExecTapSet;
    std::vector<VtValue> _values;
    UsdTimeCode _time = UsdTimeCode::Default();
    bool _valid = false;
    bool _complete = false;
};

} // namespace rigExec

#endif // RIGEXEC_VALUE_ADDRESS_H
