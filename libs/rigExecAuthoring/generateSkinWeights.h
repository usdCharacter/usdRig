// Skin rows generated from the volume weights under a skin mover's influences.
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

/// Generates the rows of the RigExecSkinMover at \p skinMover and authors
/// only rigExec:jointIndices, rigExec:jointWeights and rigExec:elementSize,
/// at the stage's edit target. Never evaluates a pose.
///
/// Rows are measured on the target's Default points in asset space. Each
/// influence's direct RigExecVolumeWeight children are measured at their
/// pose-free rest placement through the evaluator's packet builder; several
/// volumes of one influence sum, clamped to 1. Rows are normalised over the
/// influences (an all-zero row stays zero) and padded with index 0, weight 0
/// to elementSize, the widest row. With \p pointIndices only those rows are
/// replaced and the others keep their authored slots.
///
/// Throws std::invalid_argument for a malformed mover, influence, volume
/// weight or point index, and std::runtime_error when a placement or field
/// cannot be resolved; nothing is authored then.
RigExecGeneratedSkinWeights RigExecGenerateSkinWeights(
    const UsdStagePtr &stage, const SdfPath &skinMover,
    const std::vector<int> *pointIndices = nullptr);

}  // namespace rigExec

#endif  // RIGEXEC_AUTHORING_GENERATE_SKIN_WEIGHTS_H
