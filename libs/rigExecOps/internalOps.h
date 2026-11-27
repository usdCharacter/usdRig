#ifndef RIGEXEC_OPS_INTERNAL_OPS_H
#define RIGEXEC_OPS_INTERNAL_OPS_H

/// The schema info of the ops with no class of their own
/// (RigExecSchemaMode::Internal), one per op.

#include "rigExec/api/registry.h"

namespace rigExecOps::internalOps {

constexpr rigExec::RigExecSchemaInfo
Internal(const char *doc)
{
    return {"", doc, nullptr, rigExec::RigExecSchemaMode::Internal};
}

inline constexpr rigExec::RigExecSchemaInfo kAvarInputs =
    Internal("Gathers bound avar values into one value per avar.");
inline constexpr rigExec::RigExecSchemaInfo kSpaceCheckpoint = Internal(
    "A space switch's checkpoint frame, recomposed from the slots above it.");
inline constexpr rigExec::RigExecSchemaInfo kRestCompose =
    Internal("One pose slot's rest from its rest channels and parent.");
inline constexpr rigExec::RigExecSchemaInfo kLadderCompose =
    Internal("One pose slot's default ladder from its channel spaces.");
inline constexpr rigExec::RigExecSchemaInfo kComposeSlot =
    Internal("One unswitched pose slot's first pose.");
inline constexpr rigExec::RigExecSchemaInfo kSwitchedComposeSlot =
    Internal("One space-switched pose slot's first pose.");
inline constexpr rigExec::RigExecSchemaInfo kShiftedComposeSlot =
    Internal("A composed pose slot after an auto clavicle shifts it.");
inline constexpr rigExec::RigExecSchemaInfo kPoseCommit =
    Internal("A constraint's or solver batch's frames written over their "
             "joints, with the deltas carried to descendants.");
inline constexpr rigExec::RigExecSchemaInfo kProviderRefresh = Internal(
    "A connected provider's frames re-evaluated after its parents move.");
inline constexpr rigExec::RigExecSchemaInfo kProviderMatrix =
    Internal("A provider's rest-to-final or rest-to-base matrix.");
inline constexpr rigExec::RigExecSchemaInfo kFrameMatrix =
    Internal("The matrix a commit's frame record holds.");
inline constexpr rigExec::RigExecSchemaInfo kProviderArithmetic =
    Internal("One provider program operation's matrix or frame.");
inline constexpr rigExec::RigExecSchemaInfo kAffineFrame =
    Internal("One affine frame expression's matrix.");
inline constexpr rigExec::RigExecSchemaInfo kProviderSelect =
    Internal("Which candidate answers a provider value this frame.");
inline constexpr rigExec::RigExecSchemaInfo kFkChain =
    Internal("An FK chain's frames from its controls' rest-to-pose maps.");
inline constexpr rigExec::RigExecSchemaInfo kTwoBoneIk = Internal(
    "A two-bone IK chain's three frames from root, effector and pole.");
inline constexpr rigExec::RigExecSchemaInfo kDistributeTwist =
    Internal("A twist distribution's frames between its start and end frames.");
inline constexpr rigExec::RigExecSchemaInfo kSampleRibbonFrames = Internal(
    "A ribbon's frames sampled along its driver curve and its rest curve.");
inline constexpr rigExec::RigExecSchemaInfo kFkJointGroups =
    Internal("An FK chain's joint groups applied to its frames.");
inline constexpr rigExec::RigExecSchemaInfo kMoverTransform =
    Internal("A mover's transform in its geometry's point frame.");
inline constexpr rigExec::RigExecSchemaInfo kSkinMatrices =
    Internal("One bind set's skinning matrices, folded once per frame.");
inline constexpr rigExec::RigExecSchemaInfo kVolumePlacement =
    Internal("Where a weight volume ended up, as a matrix.");
inline constexpr rigExec::RigExecSchemaInfo kRecomputeNormals =
    Internal("Normals recomputed from final points.");
inline constexpr rigExec::RigExecSchemaInfo kRecomputeExtent =
    Internal("Extent recomputed from final points.");
inline constexpr rigExec::RigExecSchemaInfo kGuidePoints =
    Internal("A solver's guide frames as points.");
inline constexpr rigExec::RigExecSchemaInfo kShaderDials =
    Internal("Dials packed into one matrix for a shader.");

inline rigExec::RigExecOpDesc
WithSchema(rigExec::RigExecOpDesc desc,
           const rigExec::RigExecSchemaInfo &schema)
{
    desc.schema = &schema;
    return desc;
}

} // namespace rigExecOps::internalOps

#endif
