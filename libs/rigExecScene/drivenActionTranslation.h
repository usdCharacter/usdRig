// Scene translation of a RigExecDrivenAction into the pose-interpolator
// family's descriptor (a driven-action solve). Compiled into rigExecScene;
// RigExecTranslateScenePoseInterpolator dispatches the type here.
#ifndef RIGEXEC_SCENE_DRIVEN_ACTION_TRANSLATION_H
#define RIGEXEC_SCENE_DRIVEN_ACTION_TRANSLATION_H

#include "sceneDescriptors.h"
#include "sceneTypedReads.h"
#include "rigExecMath/drivenActionKernel.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecSceneDrivenActionDescriptor {
    SdfPath path;
    SdfPath driverAttribute;                    ///< the one rigExec:driverAttributes target
    /// Whether the driver is a float property (read in its own precision).
    bool driverIsFloat = false;
    RigExecSceneTypedRead driver, enabled;
    /// outputs:frame, outputs:weight, outputs:active, then each channel's
    /// outputs:value, in that order.
    SdfPathVector outputs;
    std::shared_ptr<const RigExecDrivenActionTable> table;
    std::vector<std::string> notes;
};

/// False with \p error when rigExec:driverAttributes does not name exactly
/// one float/double property, for fewer than two driver samples, an unknown
/// rigExec:side, rigExec:frameTies or rigExec:channelMode, a non-finite
/// sample, a channel without samples, or an output carrying authored
/// connections.
bool RigExecTranslateSceneDrivenAction(const RigExecSceneDescriptors &scene,
    const SdfPath &path, RigExecSceneDrivenActionDescriptor *output,
    std::string *error = nullptr);

}  // namespace rigExec

#endif  // RIGEXEC_SCENE_DRIVEN_ACTION_TRANSLATION_H
