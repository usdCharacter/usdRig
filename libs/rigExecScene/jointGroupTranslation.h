// Scene translation of the joint groups (RigExecJointGroupAPI) a
// RigExecFkChain or RigExecSkeleton2d lists in rigExec:jointGroups.
// Compiled into rigExecScene; the solver translation calls it.
#ifndef RIGEXEC_SCENE_JOINT_GROUP_TRANSLATION_H
#define RIGEXEC_SCENE_JOINT_GROUP_TRANSLATION_H

#include "sceneDescriptors.h"
#include "sceneTypedReads.h"
#include "rigExecMath/jointGroupKernel.h"

#include <map>
#include <string>
#include <vector>

namespace rigExec {

/// The groups of \p solver in rigExec:jointGroups order, their concatenated
/// members (each joint's index in \p jointIndex) and one rigExec:activeMember
/// read per group. Members are the "members" collection's includes in list
/// order, less its excludes. False with \p error (prefixed by \p who) for a
/// target that is not an active prim with the API, a collection that is not
/// explicitOnly or that authors a membership expression, a member that is
/// not a joint of the solver, a joint that is a member of two groups, and a
/// time-varying expansion rule or enabled flag.
bool RigExecTranslateSceneJointGroups(const RigExecSceneDescriptors &scene,
    const SdfPath &solver, const std::map<SdfPath, int32_t> &jointIndex,
    const std::string &who, std::vector<RigExecJointGroup> *groups,
    std::vector<int32_t> *members, std::vector<RigExecSceneTypedRead> *selectorReads,
    std::string *error = nullptr);

}  // namespace rigExec

#endif  // RIGEXEC_SCENE_JOINT_GROUP_TRANSLATION_H
