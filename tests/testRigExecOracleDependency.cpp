#include "rigExec/goldenPose.h"
#include "rigExec/goldenSuite.h"
#if defined(RIGEXEC_EXPLICIT_ORACLE_REGISTRATION) || defined(RIGEXEC_TEST_ORACLE_RECOVERY)
#include "rigExecOracle/registerOracle.h"
#endif
#ifdef RIGEXEC_TEST_ORACLE_RECOVERY
#include "rigExec/oracleDispatch.h"
#endif
#ifndef RIGEXEC_ORACLE_GOLDEN_ONLY
#include "rigExec/bakedProgram.h"
#include "rigExec/frozenContext.h"
#include "pxr/base/plug/registry.h"
#endif

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>

using namespace rigExec;

#define CHECK(expression) do { if (!(expression)) { \
    std::fprintf(stderr, "failed: %s\n", #expression); return false; } } while (false)

namespace {
bool Missing(const std::string &error)
{
    return error.find("RigExec oracle library is unavailable") != std::string::npos;
}

bool Missing(const RigExecRigPose &pose)
{
    return !pose.valid && std::any_of(pose.diagnostics.begin(), pose.diagnostics.end(),
        [](const std::string &error) { return Missing(error); });
}

template <class Fn>
bool ThrowsMissing(Fn call)
{
    try { call(); }
    catch (const std::runtime_error &error) { return Missing(error.what()); }
    return false;
}

bool CheckGolden(bool available)
{
    std::string encoded = "untouched", error;
    const bool result = RigExecEncodeGoldenValue(VtValue(1.0), &encoded, &error);
    if (available) {
        CHECK(result && encoded == "double 3ff0000000000000");
        CHECK(RigExecGoldenDouble(1.0) == "3ff0000000000000");
        CHECK(RigExecGoldenDigest("one") != RigExecGoldenDigest("two"));
        CHECK(RigExecCompareGolden(encoded, encoded, &error));
        return true;
    }
    CHECK(!result && Missing(error) && encoded == "untouched");
    CHECK(!RigExecEncodeGoldenValue(VtValue(1.0), &encoded));
    RigExecRigPose pose;
    std::vector<RigExecGoldenValue> values{{"keep", "key", "value"}};
    error.clear();
    CHECK(!RigExecEncodeGoldenPose(pose, &values, &error) && Missing(error));
    CHECK(values.size() == 1 && values[0].domain == "keep");
    error.clear();
    CHECK(!RigExecCompareGolden("same", "same", &error) && Missing(error));
    CHECK(!RigExecCompareGolden("same", "same"));
    CHECK(ThrowsMissing([] { RigExecGoldenDouble(1.0); }));
    CHECK(ThrowsMissing([] { RigExecGoldenEscape("value"); }));
    CHECK(ThrowsMissing([] { RigExecGoldenDigest("value"); }));
    CHECK(ThrowsMissing([] { RigExecGoldenHex(1); }));
    CHECK(ThrowsMissing([&] { RigExecGoldenVisit("visit", 0, pose, values); }));
    return true;
}

#ifndef RIGEXEC_ORACLE_GOLDEN_ONLY
bool CheckEvaluator(bool available, const char *resources)
{
    PlugRegistry::GetInstance().RegisterPlugins(resources);
    const auto stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
def RigExecRoot "Rig"
{
    def RigExecControl "Input"
    {
        double avars:tx = 3
    }
}
)USD"));
    RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
    const auto time = UsdTimeCode::Default();
    // Oracle support is optional for ordinary evaluation.
    CHECK(evaluator.Evaluate(time).valid);
    RigExecChainSampleBindings bindings;
    std::string error;
    CHECK(RigExecBindChainSampleInputs(evaluator, &bindings, &error));
    RigExecBurstSampleCache burst;
    CHECK(RigExecBuildBurstSampleCache(*evaluator.GetBakedProgram(), bindings, {},
        RigExecFrameCacheCompiledRigSignature(evaluator), &burst, &error));
    evaluator.cpuReference = true;
    const auto pose = evaluator.Evaluate(time);
    if (available) {
        CHECK(pose.valid);
        return true;
    }
    CHECK(Missing(pose));
    RigExecRigPose direct;
    CHECK(!const_cast<RigExecBakedProgram *>(evaluator.GetBakedProgram())->Run(time, &direct));
    CHECK(Missing(direct));
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    error.clear();
    CHECK(!RigExecFreezeProgram(evaluator, &frozen, &error) && Missing(error) && !frozen);
    RigExecFrameInputs sampled;
    sampled.time = UsdTimeCode(37);
    error.clear();
    CHECK(!RigExecSampleFrameInputs(evaluator, time, {}, &sampled, &error) && Missing(error));
    CHECK(sampled.time == UsdTimeCode(37));
    error.clear();
    CHECK(!RigExecSampleFrameInputsWithBurstCache(evaluator, time, {}, &burst,
        &sampled, &error) && Missing(error));
    CHECK(sampled.time == UsdTimeCode(37));
#ifdef RIGEXEC_TEST_ORACLE_RECOVERY
    // The same evaluator/time must recover without a scene edit or recompile.
    RigExecRegisterOracle();
    RigExecRegisterOracle();
    CHECK(CheckGolden(true));
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(RigExecFreezeProgram(evaluator, &frozen, &error) && frozen);
    CHECK(RigExecSampleFrameInputs(evaluator, time, {}, &sampled, &error));
    CHECK(RigExecSampleFrameInputsWithBurstCache(evaluator, time, {}, &burst,
        &sampled, &error));
#else
    evaluator.cpuReference = false;
    CHECK(evaluator.Evaluate(time).valid);
#endif
    return true;
}
#endif
}

int main(int argc, char **argv)
{
    if (argc == 2 && std::string(argv[1]) == "golden-suite") {
        if (!ThrowsMissing([] { RigExecGoldenSuiteObserver::Create(SdfPath("/Rig")); }) ||
            !ThrowsMissing([] { RigExecFinalizeGoldenSuite(); })) return 1;
        return 0;
    }
#ifdef RIGEXEC_EXPLICIT_ORACLE_REGISTRATION
    RigExecRegisterOracle();
    RigExecRegisterOracle();
#endif
#ifdef RIGEXEC_TEST_ORACLE_RECOVERY
    // Isolate late registration in this process; production never unregisters.
    RigExecInstallOracleHooks(nullptr);
#endif
#if defined(RIGEXEC_EXPECT_ORACLE) && !defined(RIGEXEC_TEST_ORACLE_RECOVERY)
    const bool available = true;
#else
    const bool available = false;
#endif
    if (!CheckGolden(available)) return 1;
#ifndef RIGEXEC_ORACLE_GOLDEN_ONLY
    if (argc != 2 || !CheckEvaluator(available, argv[1])) return 1;
#endif
    return 0;
}
