// The pose-free authored rest placement of a RigExec xformable, composed
// without compiling or evaluating the rig.
#ifndef RIGEXEC_SCENE_AUTHORED_REST_SPACE_H
#define RIGEXEC_SCENE_AUTHORED_REST_SPACE_H

#include "pxr/pxr.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/timeCode.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// \p prim's rest frame in the rig's asset space: its own rest:rx/ry/rz
/// (XYZ) and rest:tx/ty/tz, then rest:space, then the same composition of
/// the nearest RigExecJoint or RigExecControl ancestor (USD row vectors:
/// the local rest precedes the parent's). Avars and solvers are not read.
/// \p rigid is false when \p prim's own rest does not orthonormalize; the
/// unparented rest is returned then. A non-rigid ancestor is skipped.
GfMatrix4d RigExecAuthoredGuideRestSpace(
    const UsdPrim &prim, UsdTimeCode time, bool *rigid);

}  // namespace rigExec

#endif  // RIGEXEC_SCENE_AUTHORED_REST_SPACE_H
