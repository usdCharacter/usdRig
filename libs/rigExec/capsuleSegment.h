// The far end of a RigExecCapsuleWeight's segment, resolved off the stage.
#ifndef RIGEXEC_CAPSULE_SEGMENT_H
#define RIGEXEC_CAPSULE_SEGMENT_H

#include "pxr/pxr.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/prim.h"

#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// "RigExecCapsuleWeight", interned at load.
const TfToken &RigExecCapsuleWeightType();

/// The provider whose posed origin ends \p capsule's segment: the single
/// target of rigExec:segmentEnd when it names one, else the first
/// RigExecJoint child, in namespace order, of the capsule's namespace-parent
/// joint. Empty when there is none, which is a zero-length segment (a
/// sphere). A relationship naming more than one target, or a property,
/// returns empty and sets \p error. Compile and the authoring-side readers
/// (guides, bounds) call this one rule; it reads no time-varying value.
SdfPath RigExecCapsuleSegmentEnd(const UsdPrim &capsule,
                                 std::string *error = nullptr);

}  // namespace rigExec

#endif  // RIGEXEC_CAPSULE_SEGMENT_H
