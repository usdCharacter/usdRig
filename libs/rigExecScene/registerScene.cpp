// Installs the translation entry points the evaluator calls. Linked into
// librigExecScene, not librigExec.
#include "rigExec/sceneDispatch.h"
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

struct Install {
    Install()
    {
        RigExecSceneHooks hooks;
        hooks.capture = &CaptureStage;
        hooks.select = &RigExecSelectSceneDescriptorIdentities;
        hooks.translatePoseInterpolator = &RigExecTranslateScenePoseInterpolator;
        hooks.translateSpaceSwitch = &RigExecTranslateSceneSpaceSwitch;
        hooks.translateGeometry = &RigExecTranslateSceneGeometry;
        hooks.translateSolver = &RigExecTranslateSceneSolver;
        hooks.translateConstraint = &RigExecTranslateSceneConstraint;
        hooks.bindAutoClavicle = &RigExecBindAutoClavicle;
        hooks.runAutoClavicle = &RigExecRunAutoClavicle;
        RigExecInstallSceneHooks(hooks);
    }
};

const Install installed;
}

void RigExecSceneLinkAnchor() {}

}
