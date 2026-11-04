#include "authoredRestSpace.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/attribute.h"

namespace rigExec {

namespace {
// Interned when the library loads, never on a caller's lane.
const TfToken _tControlType("RigExecControl");
const TfToken _tJointType("RigExecJoint");
const TfToken _tRestRx("rest:rx"), _tRestRy("rest:ry"), _tRestRz("rest:rz");
const TfToken _tRestTx("rest:tx"), _tRestTy("rest:ty"), _tRestTz("rest:tz");
const TfToken _tRestSpace("rest:space");
const GfVec3d _restAxes[3] = {
    GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
}  // namespace

GfMatrix4d
RigExecAuthoredGuideRestSpace(const UsdPrim &prim, UsdTimeCode time,
                   bool *rigid)
{
    auto scalar = [&prim, &time](const TfToken &name) {
        double value = 0.0;
        if (const UsdAttribute a =
                prim.GetAttribute(name)) {
            a.Get(&value, time);
        }
        return value;
    };
    const double angles[3] = {scalar(_tRestRx), scalar(_tRestRy),
                              scalar(_tRestRz)};
    GfMatrix4d local(1.0);
    for (int index = 0; index < 3; ++index) {  // XYZ, the rest-avar order
        if (angles[index] != 0.0) {
            local = local * GfMatrix4d(
                                GfRotation(_restAxes[index], angles[index]),
                                GfVec3d(0));
        }
    }
    GfMatrix4d translate(1.0);
    translate.SetTranslate(
        GfVec3d(scalar(_tRestTx), scalar(_tRestTy),
                        scalar(_tRestTz)));
    local = local * translate;

    GfMatrix4d space(1.0);
    if (const UsdAttribute a =
            prim.GetAttribute(_tRestSpace)) {
        a.Get(&space, time);
    }
    GfMatrix4d rest = local * space;
    *rigid = rest.Orthonormalize(/* issueWarning = */ false);
    if (!*rigid) {
        return rest;
    }
    // USD row vectors: the local rest precedes the nearest provider rest.
    for (UsdPrim parent = prim.GetParent();
         parent && !parent.IsPseudoRoot(); parent = parent.GetParent()) {
        const TfToken parentType = parent.GetTypeName();
        if (parentType != _tJointType && parentType != _tControlType) {
            continue;
        }
        bool parentRigid = false;
        const GfMatrix4d parentRest =
            RigExecAuthoredGuideRestSpace(parent, time, &parentRigid);
        if (parentRigid) {
            rest = rest * parentRest;
        }
        break;
    }
    return rest;
}

}  // namespace rigExec
