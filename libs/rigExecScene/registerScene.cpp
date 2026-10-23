// Installs the translation entry points the evaluator calls. Linked into
// librigExecScene, not librigExec.
#include "rigExec/sceneDispatch.h"
#include "rigExecScene/registerScene.h"
#include "rigExecScene/usdSceneAccess.h"

namespace rigExec {
namespace {

bool CaptureStage(const UsdStageRefPtr &stage, const SdfPath &rigRoot,
                  const std::vector<UsdTimeCode> &identities,
                  RigExecSceneDescriptors *result, std::string *error)
{
    const RigExecUsdSceneAccess source(stage);
    return RigExecCaptureSceneDescriptors(source, rigRoot, identities, result, error);
}

const RigExecSceneHooks hooks = {
    &CaptureStage,
    &RigExecSelectSceneDescriptorIdentities,
    &RigExecTranslateScenePoseInterpolator,
    &RigExecTranslateSceneSpaceSwitch,
    &RigExecTranslateSceneGeometry,
    &RigExecTranslateSceneSolver,
    &RigExecTranslateSceneConstraint,
    &RigExecBindAutoClavicle,
    &RigExecRunAutoClavicle
};

}

void RigExecRegisterScene()
{
    RigExecInstallSceneHooks(&hooks);
}

}
