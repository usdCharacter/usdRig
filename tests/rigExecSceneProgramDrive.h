// Drives the scene program (rigExecScene) over a stage's frames, for parity
// tests against the native evaluator: identities are whole frames from
// `first` to `last`, so a history reaches frames before the animation starts.
#ifndef RIGEXEC_TESTS_SCENE_PROGRAM_DRIVE_H
#define RIGEXEC_TESTS_SCENE_PROGRAM_DRIVE_H

#include "rigExecScene/sceneDescriptors.h"
#include "rigExecScene/sceneGraphBinding.h"
#include "rigExecScene/sceneProgramTranslation.h"
#include "rigExecScene/usdSceneAccess.h"

#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <string>
#include <vector>

namespace rigExecSceneProgramDrive {

using namespace rigExec;

struct SceneRun {
    RigExecSceneProgram program;
    RigExecSceneProgramRuntime runtime;
    int first = 0;
    bool ok = false;

    SceneRun(const UsdStageRefPtr &stage, const SdfPath &rig, int firstFrame, int lastFrame)
        : first(firstFrame)
    {
        std::vector<UsdTimeCode> identities;
        for (int t = firstFrame; t <= lastFrame; ++t) identities.push_back(UsdTimeCode(t));
        identities.push_back(UsdTimeCode::Default());
        const RigExecUsdSceneAccess access(stage);
        RigExecSceneDescriptors scene;
        std::string error;
        ok = RigExecCaptureSceneDescriptors(access, rig, identities, &scene, &error) &&
             RigExecTranslateSceneProgram(scene, &program, &error) &&
             runtime.Prepare(program, &error);
        if (!ok) std::printf("scene program: %s\n", error.c_str());
    }
    size_t Identity(int t) const { return size_t(t - first); }
    bool Evaluate(int t)
    {
        std::string error;
        if (runtime.Evaluate(program, Identity(t), {}, &error)) return true;
        std::printf("scene program at %d: %s\n", t, error.c_str());
        return false;
    }
    // A jump to t: the history rebuilt from the frames before it.
    bool Jump(int t)
    {
        runtime.ClearHistory();
        std::string error;
        for (size_t k = program.GetHistoryDepth(); k >= 1; --k) {
            if (!runtime.EvaluateHistory(program, Identity(t - int(k)), &error)) {
                std::printf("scene history at %d: %s\n", t - int(k), error.c_str());
                return false;
            }
        }
        return Evaluate(t);
    }
    // The last version of \p path's value in \p domain.
    VtValue Final(const SdfPath &path, RigExecSceneValueDomain domain) const
    {
        const auto chain = program.layout.routes.chains.find({path, domain});
        VtValue value;
        if (chain == program.layout.routes.chains.end()) return value;
        const auto &c = chain->second;
        const RigExecValueId id = !c.versions.empty() ? c.versions.back().value
                                  : domain == RigExecSceneValueDomain::Pose ? c.poseBase
                                                                            : c.base;
        RigExecReadSceneGraphValue(runtime.values, id, &value);
        return value;
    }
};

}  // namespace rigExecSceneProgramDrive

#endif
