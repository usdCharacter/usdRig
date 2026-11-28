#ifndef RIGEXEC_SCENE_LAYERED_SKIN_INPUTS_H
#define RIGEXEC_SCENE_LAYERED_SKIN_INPUTS_H

// The layered skin mover's declared inputs, in declaration order: the native
// assembly and the scene's op binding read them by position.

#include <cstddef>

namespace rigExec {
namespace layeredSkinInputs {

enum Input : size_t {
    kJointIndices,
    kJointWeights,
    kElementSize,
    kSkinningMethod,
    kMask,
    kUseBaseInput,
    kTransformInput,
    kInputCount
};

} // namespace layeredSkinInputs
} // namespace rigExec

#endif
