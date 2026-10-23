#include "rigExec/rigEvaluator.h"
#ifdef RIGEXEC_EXPLICIT_SCENE_REGISTRATION
#include "rigExecScene/registerScene.h"
#endif
#ifdef RIGEXEC_TEST_SCENE_RECOVERY
#include "rigExec/sceneDispatch.h"
#include "rigExecScene/registerScene.h"
#endif

#include "pxr/base/plug/registry.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

using namespace rigExec;

namespace {
bool HasMissingScene(const std::vector<std::string> &messages)
{
    return std::any_of(messages.begin(), messages.end(), [](const std::string &s) {
        return s.find("RigExec scene library is unavailable") != std::string::npos;
    });
}

int Fail(const char *message, const std::vector<std::string> &details = {})
{
    std::fprintf(stderr, "%s\n", message);
    for (const auto &detail : details) std::fprintf(stderr, "  %s\n", detail.c_str());
    return 1;
}
}

int main(int argc, char **argv)
{
    if (argc != 2) return Fail("Expected schema resource directory");
    PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
#ifdef RIGEXEC_EXPLICIT_SCENE_REGISTRATION
    RigExecRegisterScene();
    RigExecRegisterScene();
#endif
#ifdef RIGEXEC_TEST_SCENE_RECOVERY
    // Start without registration in this isolated process, then enable it
    // after Evaluate has observed a missing dependency at this stage serial.
    RigExecInstallSceneHooks(nullptr);
#endif
    const auto stage = UsdStage::CreateInMemory();
    if (!stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
def RigExecRoot "Rig"
{
    def RigExecControl "Input"
    {
        double avars:tx = 3
    }
}
)USD")) return Fail("Could not create test rig");
    RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
    std::vector<std::string> errors;
    const bool compiled = evaluator.Compile(&errors);
#if defined(RIGEXEC_EXPECT_SCENE) && !defined(RIGEXEC_TEST_SCENE_RECOVERY)
    if (!compiled) return Fail("Linked scene library did not compile", errors);
    const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
    if (!pose.valid) return Fail("Linked scene library did not evaluate", pose.diagnostics);
    const auto control = pose.controlFrames.find(SdfPath("/Rig/Input"));
    if (control == pose.controlFrames.end() || control->second.points[0][0] != 3.0)
        return Fail("Scene evaluation lost the authored control input");
#else
    if (compiled || !HasMissingScene(errors))
        return Fail("Compile must report the missing scene library", errors);
    if (evaluator.Compile()) return Fail("Compile without an error sink must fail");
    // Evaluate compiles implicitly and must preserve the same recoverable error.
    const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
    if (pose.valid || !HasMissingScene(pose.diagnostics))
        return Fail("Evaluate must report the missing scene library", pose.diagnostics);
#endif
#ifdef RIGEXEC_TEST_SCENE_RECOVERY
    RigExecRegisterScene();
    const auto recovered = evaluator.Evaluate(UsdTimeCode::Default());
    if (!recovered.valid || HasMissingScene(recovered.diagnostics))
        return Fail("Evaluate cached a missing scene dependency", recovered.diagnostics);
#endif
    return 0;
}
