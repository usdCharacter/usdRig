// Scene translation of a RigExecSkeleton2d into its detached record and
// typed reads. Compiled into rigExecScene; the solver translation
// (RigExecSolverKind::Skeleton2d) calls it.
#ifndef RIGEXEC_SCENE_SKELETON2D_TRANSLATION_H
#define RIGEXEC_SCENE_SKELETON2D_TRANSLATION_H

#include "sceneDescriptors.h"
#include "sceneTypedReads.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/skeleton2dKernel.h"

#include <array>
#include <map>
#include <string>
#include <vector>

namespace rigExec {

/// The thirteen channel properties of a bone, in RigExecBone2dChannels
/// order: avars:tx, ty, rz, sx, then bone2d:sideFlip, endFlip, lock, lockX,
/// lockY, lockAngle, offsetX, offsetY, targetIndex. Channels 4..6 are bool,
/// the others double. Namespace-scope tokens, interned once when the
/// library loads; pass these tokens, never their text, so binding a channel
/// builds no token.
inline const std::array<TfToken, kRigExecBone2dChannelCount> RigExecBone2dChannelNames = {
    TfToken("avars:tx"), TfToken("avars:ty"), TfToken("avars:rz"), TfToken("avars:sx"),
    TfToken("bone2d:sideFlip"), TfToken("bone2d:endFlip"), TfToken("bone2d:lock"),
    TfToken("bone2d:lockX"), TfToken("bone2d:lockY"), TfToken("bone2d:lockAngle"),
    TfToken("bone2d:offsetX"), TfToken("bone2d:offsetY"), TfToken("bone2d:targetIndex")};
/// The fallback each channel reads when it is unauthored.
inline constexpr double kRigExecBone2dChannelDefaults[kRigExecBone2dChannelCount] = {
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1};

/// Structural facts plus the per-bone value reads of one skeleton. Rests
/// are not here: they follow the solver rest route (restSlots), so a rest
/// edit refreshes the record without a recompile.
struct RigExecSceneSkeleton2dDescriptor {
    SdfPath path;
    SdfPathVector joints;                     ///< rigExec:joints order (output order)
    std::vector<uint32_t> solveOrder;         ///< rigExec:joints index per solve entry
    std::vector<RigExecBone2dSetup> setup;    ///< in solveOrder; digested
    /// Per bone, in rigExec:joints order, kRigExecBone2dChannelCount reads
    /// in RigExecBone2dChannelNames order. A channel that property movers
    /// revise reads that chain's result.
    std::vector<RigExecSceneTypedRead> channelReads;
    /// Concatenated parent spaces and target candidates (solve-order
    /// indices, kRigExecBone2dSpaceBase and RigExecBone2dSpaceExternal for
    /// spaces); RigExecBone2dSetup ranges index them.
    std::vector<int32_t> spaceSources, targetCandidates;
    /// The selectors: one active-index read per space switch (switches
    /// order), each with the value it reads when unauthored.
    std::vector<RigExecSceneTypedRead> selectorReads;
    std::vector<double> selectorFallbacks;
    /// The RigExecSpaceSwitch prims the skeleton reads, in solve order of
    /// the bones they switch.
    SdfPathVector switches;
    /// Outside parent spaces (frame providers that are not bones), entry e
    /// for RigExecBone2dSpaceExternal(e).
    SdfPathVector externalSources;
    std::vector<std::string> notes;
};

/// Translates \p path hanging from \p start (empty: asset space). \p rests
/// holds every provider's absolute rest frame. A bone's parent is its
/// nearest RigExecJoint namespace ancestor when that joint is a bone of the
/// skeleton; its tip is its first RigExecJoint child; a RigExecSpaceSwitch
/// whose target is a bone gives the bone its parent spaces. False with
/// \p error for a joint that is not a RigExecJoint with RigExecBone2dAPI,
/// authors rest:tx..rest:rz or an animated rest:space, or has a rest that
/// is not planar in the start frame; for a start frame or outside source
/// at or under a bone; for a cycle through the namespace parents and the
/// switch sources; for a switch the skeleton cannot read; for a pin with a
/// target and no parent bone; and for time-sampled uniform settings.
/// A pose step below the skeleton that writes a bone hands its frame to the
/// skeleton as that bone's rest, in every evaluator.
bool RigExecTranslateSceneSkeleton2d(const RigExecSceneDescriptors &scene,
    const SdfPath &path, const SdfPath &start,
    const std::map<SdfPath, RigExecPointFrame> &rests,
    RigExecSceneSkeleton2dDescriptor *output, std::string *error = nullptr);

/// Sets channel \p c of \p channels from a typed read's value; an empty or
/// mistyped value leaves the channel as it is.
inline void RigExecSetBone2dChannel(RigExecBone2dChannels *channels, size_t c,
    const VtValue &value)
{
    if (RigExecBone2dChannelIsBool(c)) {
        if (value.IsHolding<bool>()) {
            RigExecBone2dSetChannel(channels, c, value.UncheckedGet<bool>() ? 1.0 : 0.0);
        }
    } else if (value.IsHolding<double>()) {
        RigExecBone2dSetChannel(channels, c, value.UncheckedGet<double>());
    }
}

}  // namespace rigExec

#endif  // RIGEXEC_SCENE_SKELETON2D_TRANSLATION_H
