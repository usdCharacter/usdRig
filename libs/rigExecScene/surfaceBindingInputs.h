// The surface binding mover's declared inputs, in declaration order: the
// native assembly and the scene's op binding read them by position.
#ifndef RIGEXEC_SCENE_SURFACE_BINDING_INPUTS_H
#define RIGEXEC_SCENE_SURFACE_BINDING_INPUTS_H

#include <cstddef>

namespace rigExec {
namespace surfaceBindingInputs {

// The binding's own int arrays (vertices, vertexOffsets, polygonOffsets,
// pointIndices, limitOffsets, limitIndices), its float arrays
// (barycentricWeights, bindingWeights, mask, deltaMultipliers,
// limitWeights, limitDuWeights, limitDvWeights) and settings, then the
// bound surface's points and topology.
inline constexpr size_t kIntArrays = 6;
inline constexpr size_t kFloatArrays = 7;

enum Input : size_t {
    FirstInt = 0,
    FirstFloat = FirstInt + kIntArrays,
    OffsetVectors = FirstFloat + kFloatArrays,
    SurfaceMode,
    NormalMode,
    Strength,
    DeltaMultiplier,
    SurfaceRestMatrix,
    TargetRestMatrix,
    SurfaceToBinding,
    SurfacePoints,
    SurfaceCounts,
    SurfaceIndices,
    InputCount
};

} // namespace surfaceBindingInputs
} // namespace rigExec

#endif
