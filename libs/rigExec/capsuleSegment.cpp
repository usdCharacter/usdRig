#include "capsuleSegment.h"
#include "pathText.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/usd/usd/relationship.h"

// Interned at load: compile reads these on parallel lanes.
TF_DEFINE_PRIVATE_TOKENS(
    _capsuleTokens,
    ((capsuleWeight, "RigExecCapsuleWeight"))
    ((joint, "RigExecJoint"))
    ((segmentEnd, "rigExec:segmentEnd"))
);

namespace rigExec {

const TfToken &
RigExecCapsuleWeightType()
{
    return _capsuleTokens->capsuleWeight;
}

SdfPath
RigExecCapsuleSegmentEnd(const UsdPrim &capsule, std::string *error)
{
    if (!capsule) {
        return SdfPath();
    }
    SdfPathVector targets;
    if (const UsdRelationship rel =
            capsule.GetRelationship(_capsuleTokens->segmentEnd)) {
        rel.GetTargets(&targets);
    }
    if (targets.size() > 1 ||
        (targets.size() == 1 && !targets[0].IsPrimPath())) {
        if (error) {
            // Spelled without interning: compile calls this on its lanes.
            std::string who;
            if (!RigExecSpellPathText(capsule.GetPath(), &who)) {
                who = capsule.GetPath().GetAsString();
            }
            *error = who + ": rigExec:segmentEnd must name at most one prim";
        }
        return SdfPath();
    }
    if (targets.size() == 1) {
        return targets[0];
    }
    // The default: the parent joint's segment, ending at its first joint
    // child (End joints included), the same rule a bone's tip follows.
    const UsdPrim parent = capsule.GetParent();
    if (!parent || parent.GetTypeName() != _capsuleTokens->joint) {
        return SdfPath();
    }
    for (const UsdPrim &child : parent.GetChildren()) {
        if (child.GetTypeName() == _capsuleTokens->joint) {
            return child.GetPath();
        }
    }
    return SdfPath();
}

}  // namespace rigExec
