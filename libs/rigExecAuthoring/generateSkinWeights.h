// Skin weights generated from volume weights: the authoring step that turns
// the RigExecVolumeWeight children (sphere, plane, curve) of a skin mover's
// influences into the mover's authored rigExec:jointIndices /
// rigExec:jointWeights rows.
#ifndef RIGEXEC_AUTHORING_GENERATE_SKIN_WEIGHTS_H
#define RIGEXEC_AUTHORING_GENERATE_SKIN_WEIGHTS_H

#include "pxr/pxr.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"

#include <cstddef>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// What one RigExecGenerateSkinWeights call authored.
struct RigExecGeneratedSkinWeights {
    /// Rows replaced: every point, or the distinct selected points.
    size_t rowsWritten = 0;
    /// The authored rigExec:elementSize.
    int elementSize = 1;
    /// rigExec:influences entries, in list order, that own no volume
    /// weight; each contributed weight 0 to every row.
    SdfPathVector influencesWithoutVolumes;
};

/// Generates the rows of the RigExecSkinMover at \p skinMover from volume
/// weights and authors them as whole arrays through
/// RigExecSkinMoverHandle::SetJointInfluences, at the stage's edit target.
///
/// The target's points are read at Default and carried into the rig's asset
/// space by their setup transform. For every influence in rigExec:influences
/// (row slots index that list), the fields of its direct RigExecVolumeWeight
/// children of every type (RigExecSphereWeight, RigExecPlaneWeight,
/// RigExecCurveWeight) are measured at their pose-free rest placement
/// through the evaluator's packet builder, with the falloff,
/// inputs:scaleX/Y/Z and shape settings it uses: a sphere's Pos/Neg scales,
/// a plane's axis, bounds and extents, and a curve's one rigExec:curve
/// points source (read at Default and carried into asset space like the
/// skin's points). Several volumes of one influence sum, clamped to 1, and
/// an influence with none contributes 0. Each row is then normalised over
/// the influences; a row whose fields sum to zero stays all zero, which
/// holds the point at its
/// rest under classicLinear's weight shortfall. A row lists its non-zero
/// influences in influence order, padded with index 0, weight 0 to
/// elementSize, the widest row (at least 1).
///
/// With \p pointIndices only those rows are replaced; the other rows keep
/// their authored non-zero slots (an unauthored layout reads as zero rows),
/// and elementSize is recomputed over the merged rows.
///
/// Never evaluates a pose and writes nothing but rigExec:jointIndices,
/// rigExec:jointWeights and rigExec:elementSize. Throws
/// std::invalid_argument for a malformed mover, influence, volume weight or
/// point index, and std::runtime_error when a placement or field cannot be
/// resolved; nothing is authored then.
RigExecGeneratedSkinWeights RigExecGenerateSkinWeights(
    const UsdStagePtr &stage, const SdfPath &skinMover,
    const std::vector<int> *pointIndices = nullptr);

}  // namespace rigExec

#endif  // RIGEXEC_AUTHORING_GENERATE_SKIN_WEIGHTS_H
