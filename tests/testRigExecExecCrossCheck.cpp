#include "rigExecSchema/schemaNames.h"
#include "rigExec/internal/bakedExecCrossCheck.h"
#include "rigExec/internal/bakedExecCrossCheckRows.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/api/rigEvaluator.h"
#include "pxr/usd/sdf/layer.h"
#include "rigExec/api/goldenPose.h"
#include "rigExec/api/types.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/attribute.h"
#include <iostream>
#include <type_traits>
#include <utility>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(expression)                                                      \
    do {                                                                       \
        if (!(expression)) {                                                   \
            std::cerr << "failed at " << __LINE__ << ": " << #expression       \
                      << '\n';                                                 \
            return 1;                                                          \
        }                                                                      \
    } while (false)

// Factory gate: the only public RigExecBakedExecCheckRows constructor takes a
// non-aggregate tag that brace-init cannot synthesize, so no caller outside
// the class can allocate rows directly. The probe spells the real arity: a
// tag plus every remaining constructor argument.
template <class Default, class AlwaysVoid, class T, class... Rest>
struct _BraceConstructibleImpl : std::false_type {};
template <class Default, class T, class... Rest>
struct _BraceConstructibleImpl<
    Default, std::void_t<decltype(T({}, std::declval<Rest>()...))>, T, Rest...>
    : std::true_type {};
template <class T, class... Rest>
using _BraceConstructible = _BraceConstructibleImpl<void, void, T, Rest...>;
static_assert(!_BraceConstructible<RigExecBakedExecCheckRows,
                                   const UsdStageRefPtr &>::value,
              "factory gate: brace-init must be rejected");

int
TestIndependentConstraintRows(const std::string &fixtures)
{
    for (const char *type : {"aim", "rotation", "parent"})
        for (const bool geometry : {false, true}) {
            const auto stage =
                UsdStage::Open(fixtures + "/" + type +
                               (geometry ? "_points.usda" : "_xform.usda"));
            CHECK(stage);
            RigExecRigEvaluator evaluator(stage, SdfPath("/World/Rig"));
            std::vector<std::string> errors;
            CHECK(evaluator.Compile(&errors));
            CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
            const auto *program = evaluator.GetBakedProgram();
            CHECK(program);
            std::string error;
            CHECK(RigExecBakedProgramTesting::EnableExecCrossCheck(*program,
                                                                   &error));
            const auto rows =
                RigExecBakedProgramTesting::ExecCrossCheckRows(*program);
            CHECK(rows);
            // Instrumentation attached after admission needs a fresh body capture.
            const_cast<RigExecBakedProgram *>(program)->BumpProgramStamp();
            CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
            auto report = rows->Evaluate(UsdTimeCode::Default());
            for (const auto &line : report.diagnostics)
                std::cerr << line << '\n';
            CHECK(report.Passed());
            CHECK(report.checkedByKind[uint32_t(
                      RigExecBakedStepKind::Constraint)] == 1);
            const auto clean = rows->Inputs();
            size_t observed = 0;
            for (size_t i = 0; i < rows->Descriptors().size(); ++i) {
                const auto &descriptor = rows->Descriptors()[i];
                if (!descriptor.acquireReference)
                    continue;
                ++observed;
                CHECK(rows->WasChecked(i));
                CHECK(clean[i].expected.IsHolding<std::string>());
                CHECK(clean[i].expected.UncheckedGet<std::string>().find(
                          "candidate=") != std::string::npos);
                if (geometry)
                    CHECK(clean[i].expected.UncheckedGet<std::string>().find(
                              "deltaPresent=1") != std::string::npos);
                auto &input = rows->TestingInputs()[i];
                input.expected =
                    VtValue(clean[i].expected.UncheckedGet<std::string>() +
                            " corrupted candidate witness");
                report = rows->Evaluate(UsdTimeCode::Default());
                CHECK(report.failed == 1 &&
                      report.checkedByKind[uint32_t(
                          RigExecBakedStepKind::Constraint)] == 1);
                CHECK(report.diagnostics.size() == 1 &&
                      report.diagnostics[0].find(descriptor.key) == 0);
                input = clean[i];
                CHECK(!input.overrides.empty());
                input.overrides.erase(input.overrides.begin());
                report = rows->Evaluate(UsdTimeCode::Default());
                CHECK(report.failed == 1 &&
                      report.diagnostics[0].find(
                          "missing bound input override") != std::string::npos);
                input = clean[i];
            }
            CHECK(observed == 1);
            CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
            CHECK(rows->Evaluate(UsdTimeCode::Default()).Passed());
        }
    return 0;
}

// Row names come from one interned table. Capture reads it; it never interns
// text, and Build resolves its names through the error channel. A capture
// that cannot intern its name marks the row fatal, and every later
// classification keeps it fatal.
int
TestRowNamesAndSkips()
{
    {
        TfToken token;
        std::string error;
        CHECK(RigExecExecCheckToken(SchemaComputationText::computePointFrame) !=
              nullptr);
        CHECK(RigExecExecCheckToken("noSuchExecCheckName") == nullptr);
        CHECK(!RigExecExecCheckResolve("noSuchExecCheckName", token, &error));
        CHECK(token.IsEmpty());
        CHECK(error.find("noSuchExecCheckName") != std::string::npos);
        CHECK(RigExecExecCheckResolve(SchemaComputationText::computePointFrame,
                                      token, nullptr));
        CHECK(!token.IsEmpty());
    }
    TfToken token;
    RigExecExecCheckInput fatal;
    CHECK(!RigExecExecCheckCaptureName("noSuchExecCheckName", fatal, token));
    CHECK(fatal.skip == RigExecExecCheckSkip::MissingName);
    CHECK(token.IsEmpty());
    RigExecExecCheckApplyAfterStep(fatal, true, false);
    RigExecExecCheckApplyAfterStep(fatal, false, true);
    CHECK(fatal.skip == RigExecExecCheckSkip::MissingName);
    RigExecExecCheckSetSkip(fatal, RigExecExecCheckSkip::NoEquivalent);
    RigExecExecCheckSetSkip(fatal, RigExecExecCheckSkip::ArrayInput);
    RigExecExecCheckSetSkip(fatal, RigExecExecCheckSkip::Unusable);
    CHECK(fatal.skip == RigExecExecCheckSkip::MissingName);
    RigExecExecCheckInput ordinary;
    RigExecExecCheckSetSkip(ordinary, RigExecExecCheckSkip::ArrayInput);
    CHECK(!RigExecExecCheckCaptureName("noSuchExecCheckName", ordinary, token));
    CHECK(ordinary.skip == RigExecExecCheckSkip::MissingName);
    RigExecExecCheckInput skipped;
    RigExecExecCheckSetSkip(skipped, RigExecExecCheckSkip::ArrayInput);
    RigExecExecCheckApplyAfterStep(skipped, false, true);
    RigExecExecCheckApplyAfterStep(skipped, false, false);
    CHECK(skipped.skip == RigExecExecCheckSkip::ArrayInput);
    RigExecExecCheckInput cleanRow;
    RigExecExecCheckApplyAfterStep(cleanRow, true, false);
    CHECK(cleanRow.skip == RigExecExecCheckSkip::Unusable);
    return 0;
}

int
main(int argc, char **argv)
{
    CHECK(argc == 2);
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const auto stage = UsdStage::CreateInMemory();
    const auto parent =
        stage->DefinePrim(SdfPath("/Parent"), TfToken("RigExecControl"));
    const auto child =
        stage->DefinePrim(SdfPath("/Parent/Child"), TfToken("RigExecControl"));
    const TfToken pointFrame(SchemaComputationText::computePointFrame),
        parentSpace(SchemaComputationText::computedParentSpace);
    RigExecPointFrame supplied;
    for (auto &point : supplied.points)
        point += GfVec3d(7, 3, -2);
    RigExecValueOverride frameOverride{parent.GetPath(), pointFrame, TfToken(),
                                       VtValue(supplied)};

    RigExecBakedExecCrossCheck checks(stage);
    RigExecExecCheckDescriptor row;
    row.key = "parent-space /Parent/Child";
    row.kind = 17;
    row.address = RigExecValueAddress::Prim(child.GetPath(), parentSpace);
    row.requiredOverrides = {
        RigExecValueAddress::Prim(parent.GetPath(), pointFrame)};
    std::string error;
    CHECK(checks.Add(row, &error));
    CHECK(!checks.Add(row, &error));
    RigExecExecCheckInput input;
    input.expected = VtValue(GfMatrix4d(1).SetTranslate(GfVec3d(7, 3, -2)));
    input.overrides = {frameOverride};
    // A public address has no independent judge: it is counted, not checked.
    auto report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(report.Passed() && report.checked == 0 &&
          report.skipped[RigExecExecCheckSkip::NoEquivalent] == 1);
    input.skip = RigExecExecCheckSkip::ArrayInput;
    report = checks.Evaluate(UsdTimeCode::Default(), {input});
    CHECK(report.Passed() && report.checked == 0 &&
          report.skipped[input.skip] == 1);
    CHECK(!checks.Evaluate(UsdTimeCode::Default(), {}).Passed());

    // Registering an output as a required input is refused at compile time.
    row.key = "vacuous";
    row.requiredOverrides = {row.address};
    CHECK(!checks.Add(row, &error));
    if (TestRowNamesAndSkips())
        return 1;
    return TestIndependentConstraintRows(argv[1]);
}
