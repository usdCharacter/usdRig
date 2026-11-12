#include "rigExecSchema/schemaNames.h"
#include "rigExec/bakedExecCrossCheckRows.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
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

namespace {
bool
Bump(VtValue *value)
{
    if (value->IsHolding<GfMatrix4d>()) {
        auto m = value->UncheckedGet<GfMatrix4d>();
        m[3][0] =
            std::nextafter(m[3][0], std::numeric_limits<double>::infinity());
        *value = VtValue(m);
        return true;
    }
    if (value->IsHolding<RigExecPointFrame>()) {
        auto frame = value->UncheckedGet<RigExecPointFrame>();
        frame.points[0][0] = std::nextafter(
            frame.points[0][0], std::numeric_limits<double>::infinity());
        *value = VtValue(frame);
        return true;
    }
    if (value->IsHolding<RigExecPointFrameArray>()) {
        auto array = value->UncheckedGet<RigExecPointFrameArray>();
        if (array.frames.empty())
            return false;
        array.frames[0].points[0][0] =
            std::nextafter(array.frames[0].points[0][0],
                           std::numeric_limits<double>::infinity());
        *value = VtValue(array);
        return true;
    }
    if (value->IsHolding<RigExecWeightPacket>()) {
        auto packet = value->UncheckedGet<RigExecWeightPacket>();
        if (!packet.values.empty())
            packet.values[0] = std::nextafter(
                packet.values[0], std::numeric_limits<float>::infinity());
        else
            packet.defaultWeight = std::nextafter(
                packet.defaultWeight, std::numeric_limits<float>::infinity());
        *value = VtValue(packet);
        return true;
    }
    if (value->IsHolding<std::string>()) {
        *value = VtValue(value->UncheckedGet<std::string>() +
                         " deliberate witness corruption");
        return true;
    }
    return false;
}
} // namespace

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

// A stage path and reporting frame make this a supported fixture regression,
// using the native program's real bound inputs and stored outputs.
int
main(int argc, char **argv)
{
    CHECK(argc == 3);
    RigExecLoadComputations();
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const auto stage = UsdStage::Open(argv[1]);
    CHECK(stage);
    SdfPath rigPath;
    for (const auto &prim : stage->Traverse())
        if (prim.GetTypeName() == SchemaTypeName::RigExecRoot) {
            rigPath = prim.GetPath();
            break;
        }
    CHECK(!rigPath.IsEmpty());
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetSolverGuidesEnabled(true);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const auto *program = evaluator.GetBakedProgram();
    CHECK(program);
    std::string error;
    const bool enabled =
        RigExecBakedProgramTesting::EnableExecCrossCheck(*program, &error);
    if (!enabled)
        std::cerr << "exec checker setup: " << error << '\n';
    CHECK(enabled);
    const auto rows = RigExecBakedProgramTesting::ExecCrossCheckRows(*program);
    CHECK(rows);
    const UsdTimeCode time(std::stod(argv[2]));
    CHECK(evaluator.Evaluate(time).valid);
    auto report = rows->Evaluate(time);
    for (const auto &line : report.diagnostics)
        std::cerr << line << '\n';
    CHECK(report.Passed());
    CHECK(report.checked > 0);
    const auto clean = rows->Inputs();
    const auto &descriptors = rows->Descriptors();
    CHECK(clean.size() == descriptors.size());
    // These supported fixtures must judge at least one row of every mirrored
    // kind they contain. A silently all-skipped kind cannot pass the suite.
    std::set<uint32_t> presentKinds;
    for (const auto &descriptor : descriptors)
        if (!descriptor.skipOnly)
            presentKinds.insert(descriptor.kind);
    for (const auto kind : presentKinds)
        CHECK(report.checkedByKind[kind] > 0);
    std::vector<char> eligible(clean.size(), 0);
    for (size_t i = 0; i < clean.size(); ++i)
        eligible[i] = rows->WasChecked(i);
    std::set<uint32_t> eligibleKinds;
    size_t mutations = 0;
    for (size_t i = 0; i < clean.size(); ++i) {
        if (!eligible[i])
            continue;
        eligibleKinds.insert(descriptors[i].kind);
        auto &input = rows->TestingInputs()[i];
        CHECK(Bump(&input.expected));
        report = rows->Evaluate(time);
        CHECK(!report.Passed() && report.failed == 1);
        CHECK(report.diagnostics.size() == 1 &&
              report.diagnostics[0].find(descriptors[i].key) == 0);
        rows->TestingInputs()[i] = clean[i];
        ++mutations;
        CHECK(!input.overrides.empty());
        input.overrides.erase(input.overrides.begin());
        report = rows->Evaluate(time);
        CHECK(!report.Passed() && report.failed == 1);
        CHECK(report.diagnostics.size() == 1 &&
              report.diagnostics[0].find(descriptors[i].key) == 0);
        CHECK(report.diagnostics[0].find("missing bound input override") !=
              std::string::npos);
        rows->TestingInputs()[i] = clean[i];
    }
    CHECK(mutations > 0);
    report = rows->Evaluate(time);
    CHECK(report.Passed());
    for (const auto kind : eligibleKinds)
        CHECK(report.checkedByKind[kind] > 0);
    // Completeness: a row the oracle mirrors names a real computation or
    // attribute, and every captured override names one too. A name missing
    // from the interned table would leave an empty token here.
    for (const auto &descriptor : descriptors) {
        if (descriptor.skipOnly)
            continue;
        const bool named =
            descriptor.address.target.IsPropertyPath()
                ? !descriptor.address.target.GetNameToken().IsEmpty()
                : !descriptor.address.publicComputation.IsEmpty();
        CHECK(named);
    }
    for (const auto &input : clean)
        for (const auto &override : input.overrides)
            CHECK(!override.attribute.IsEmpty() ||
                  !override.computation.IsEmpty());
    // Row names come from one interned table. Capture reads it; it never
    // interns text, and Build resolves its names through the error channel.
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
    // A capture that cannot intern its name marks the row fatal. Every later
    // classification runs through RigExecExecCheckSetSkip, which is the single
    // place a row's skip may change, so an ordinary reason cannot erase it.
    {
        TfToken token;
        RigExecExecCheckInput fatal;
        CHECK(
            !RigExecExecCheckCaptureName("noSuchExecCheckName", fatal, token));
        CHECK(fatal.skip == RigExecExecCheckSkip::MissingName);
        CHECK(token.IsEmpty());
        // The two AfterStep reasons, an invalid frame matrix and a solve that
        // produced no frames, both leave the fatal row fatal.
        RigExecExecCheckApplyAfterStep(fatal, true, false);
        RigExecExecCheckApplyAfterStep(fatal, false, true);
        CHECK(fatal.skip == RigExecExecCheckSkip::MissingName);
        RigExecExecCheckSetSkip(fatal, RigExecExecCheckSkip::NoEquivalent);
        RigExecExecCheckSetSkip(fatal, RigExecExecCheckSkip::ArrayInput);
        RigExecExecCheckSetSkip(fatal, RigExecExecCheckSkip::Unusable);
        CHECK(fatal.skip == RigExecExecCheckSkip::MissingName);
        // A miss recorded after an ordinary reason still decides the row.
        RigExecExecCheckInput ordinary;
        RigExecExecCheckSetSkip(ordinary, RigExecExecCheckSkip::ArrayInput);
        CHECK(!RigExecExecCheckCaptureName("noSuchExecCheckName", ordinary,
                                           token));
        CHECK(ordinary.skip == RigExecExecCheckSkip::MissingName);
        // An already-skipped row keeps its own reason when the post-step
        // classification finds nothing to report.
        RigExecExecCheckInput skipped;
        RigExecExecCheckSetSkip(skipped, RigExecExecCheckSkip::ArrayInput);
        RigExecExecCheckApplyAfterStep(skipped, false, true);
        RigExecExecCheckApplyAfterStep(skipped, false, false);
        CHECK(skipped.skip == RigExecExecCheckSkip::ArrayInput);
        // A clean row still takes the ordinary reasons it earns.
        RigExecExecCheckInput cleanRow;
        RigExecExecCheckApplyAfterStep(cleanRow, true, false);
        CHECK(cleanRow.skip == RigExecExecCheckSkip::Unusable);
        CHECK(rows->Evaluate(time).Passed());
    }
    // Error path: the report on the owning thread counts a row whose capture
    // could not intern its name as a failure, never as a skip.
    size_t judgedRow = clean.size();
    for (size_t i = 0; i < clean.size(); ++i)
        if (clean[i].skip == RigExecExecCheckSkip::None) {
            judgedRow = i;
            break;
        }
    CHECK(judgedRow < clean.size());
    TfToken token;
    CHECK(!RigExecExecCheckCaptureName(
        "noSuchExecCheckName", rows->TestingInputs()[judgedRow], token));
    RigExecExecCheckApplyAfterStep(rows->TestingInputs()[judgedRow], true,
                                   true);
    report = rows->Evaluate(time);
    CHECK(!report.Passed() && report.failed == 1);
    CHECK(report.diagnostics.size() == 1 &&
          report.diagnostics[0].find("interned name missing") !=
              std::string::npos &&
          report.diagnostics[0].find(descriptors[judgedRow].key) == 0);
    CHECK(report.skipped.find(RigExecExecCheckSkip::MissingName) ==
          report.skipped.end());
    rows->TestingInputs()[judgedRow] = clean[judgedRow];
    CHECK(rows->Evaluate(time).Passed());
    // A clean rerun retains each skipped operation's matching input witness.
    CHECK(evaluator.Evaluate(time).valid);
    CHECK(rows->Evaluate(time).Passed());
    std::cout << "exec rows: checked=" << report.checked
              << " mutated=" << mutations << '\n';
    return 0;
}
