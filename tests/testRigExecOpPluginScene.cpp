// The fixture mover plugin and its op plugin together: discovery finds the
// op plugin, its binder turns the mover's declared inputs into the Quadratic
// op's, and one op node run through the op core gives the native evaluator's
// bits -- with a reference read, under an envelope, and when the mover
// refuses.
//
//   testRigExecOpPluginScene <mover plugin resources> <op plugin resources>
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecFrozenParity.h"
#include "rigExecMath/envelope.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecScene/externalOps.h"
#include "rigExecScene/opPlugins.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <cstring>
#include <string>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

static int failures = 0;
#define CHECK(x)                                                               \
    do {                                                                       \
        if (!(x)) {                                                            \
            std::printf("FAIL %d: %s\n", __LINE__, #x);                        \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

namespace {

const char *kRig = R"USD(#usda 1.0
def RigExecRoot "Rig" {
 def Points "Body" {
  point3f[] points = [(-2, 0, 0), (1, 2, 3), (0.5, -1, 2), (3, 1, -1)]
 }
 def Points "Reference" {
  point3f[] points = [(0, 0.25, 0)]
 }
 def ExternalQuadraticMover "External" (prepend apiSchemas = ["RigExecMoverAPI"]) {
  rel rigExec:moves = </Rig/Body.points>
  float inputs:gain = 0.75
 }
}
)USD";

const SdfPath kBody("/Rig/Body.points");

UsdStageRefPtr
Stage()
{
    auto stage = UsdStage::CreateInMemory();
    CHECK(stage->GetRootLayer()->ImportFromString(kRig));
    return stage;
}

/// The Quadratic op on \p points with the mover's declared \p inputs under a
/// constant \p weight. False with the compile or evaluation error, and
/// nothing published, where the op fails.
bool
RunOp(const std::vector<VtValue> &inputs, float weight,
      std::vector<GfVec3f> &points, std::string *error)
{
    RigExecRevisionBinding binding;
    binding.moverPath = SdfPath("/Rig/External");
    binding.target = kBody;
    binding.externalSchema = TfToken("ExternalQuadraticMover");
    // The binder reads how many inputs the mover declares.
    binding.externalInputs.resize(inputs.size());
    const std::vector<GfVec3f> base = points;
    RigExecExternalOpValues values;
    values.inputs = &inputs;
    values.basePoints = &base;
    RigExecRigDescBuilder builder;
    RigExecExternalOpNode node;
    if (!RigExecAddExternalOp(binding, values, &builder, &node, error))
        return false;
    auto rig = builder.Build(error);
    auto program = rig ? RigExecCompile(*rig, {}, error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, error) : nullptr;
    CHECK(evaluator);
    if (!evaluator ||
        !RigExecWriteExternalOpInputs(binding, values,
                                      RigExecWeightPacket::Constant(weight),
                                      points, node, evaluator.get(), error))
        return false;
    if (!evaluator->Evaluate(error))
        return false;
    const auto *out =
        static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
            evaluator->GetOutput(node.out));
    CHECK(out && out->Size() == points.size());
    if (!out || out->Size() != points.size())
        return false;
    // The op's full-strength result, blended in as the engine does.
    for (size_t i = 0; i < points.size(); ++i)
        points[i] = RigExecBlendEnvelope(
            points[i], GfVec3f((*out)[i][0], (*out)[i][1], (*out)[i][2]),
            weight);
    return true;
}

/// The native evaluator runs the mover as the plugin's op, as a frozen job
/// and the .rigexec do, which plays it with no plugin code but the op
/// library and poses its declared input.
void
Evaluators()
{
    auto stage = Stage();
    RigExecRigEvaluator rig(stage, SdfPath("/Rig"));
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    std::string why;
    const bool frozen =
        RigExecFrozenMatchesLive(&rig, SdfPath("/Rig"), UsdTimeCode(1), &why);
    if (!frozen)
        std::printf("frozen: %s\n", why.c_str());
    CHECK(frozen);
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.movedProperties.count(kBody));
    if (!pose.valid || !pose.movedProperties.count(kBody))
        return;
    const VtVec3fArray native =
        pose.movedProperties.at(kBody).Get<VtVec3fArray>();
    RigExecBakeOpts options;
    options.time = 1.0;
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(rig, options, &result, &error);
    if (!baked)
        std::printf("bake: %s\n", error.c_str());
    CHECK(baked);
    auto reader = baked ? RigExecRuntimeReader::Open(
                              result.bytes.data(), result.bytes.size(), &error)
                        : nullptr;
    if (baked && !reader)
        std::printf("open: %s\n", error.c_str());
    CHECK(reader);
    if (!reader)
        return;
    CHECK(reader->GetExternalMoverTypes().empty());
    const auto body = [&]() -> const RigExecRuntimePoints * {
        for (const auto &points : reader->GetPoints())
            if (points.path == kBody.GetString())
                return &points;
        return nullptr;
    };
    const bool ran = reader->Execute(&error);
    if (!ran)
        std::printf("execute: %s\n", error.c_str());
    CHECK(ran && body() && body()->points.size() == native.size());
    if (!ran || !body() || body()->points.size() != native.size())
        return;
    bool same = true;
    for (size_t i = 0; i < native.size(); ++i)
        for (size_t a = 0; a < 3; ++a)
            same = same && body()->points[i][a] == native[i][a];
    CHECK(same);
    CHECK(reader->SetInput("/Rig/External.inputs:gain", 2.0, &error));
    CHECK(reader->Execute(&error));
    CHECK(body() && body()->points[0][2] == 8.0f);
}

/// The native evaluator and the op agree on the body's points. With
/// \p expect the native evaluator refuses the mover and leaves the rest,
/// and the op fails with an error holding \p expect.
void
Compare(const UsdStageRefPtr &stage, const std::vector<VtValue> &inputs,
        float weight, const char *what, const char *expect = nullptr)
{
    RigExecRigEvaluator rig(stage, SdfPath("/Rig"));
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    for (const std::string &e : errors)
        std::printf("compile: %s\n", e.c_str());
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1));
    VtVec3fArray rest;
    CHECK(stage->GetAttributeAtPath(kBody).Get(&rest));
    const auto moved = pose.movedProperties.find(kBody);
    const VtVec3fArray native = moved != pose.movedProperties.end()
                                    ? moved->second.Get<VtVec3fArray>()
                                    : rest;
    std::vector<GfVec3f> points(rest.cbegin(), rest.cend());
    std::string error;
    const bool ran = RunOp(inputs, weight, points, &error);
    if (expect) {
        const bool refused = !ran && error.find(expect) != std::string::npos &&
                             native.size() == rest.size() &&
                             std::memcmp(native.cdata(), rest.cdata(),
                                         rest.size() * sizeof(GfVec3f)) == 0;
        if (!refused)
            std::printf("%s: ran %d (%s)\n", what, int(ran), error.c_str());
        CHECK(refused);
        return;
    }
    const bool same = ran && points.size() == native.size() &&
                      std::memcmp(points.data(), native.cdata(),
                                  native.size() * sizeof(GfVec3f)) == 0;
    if (!same && ran && !points.empty() && !native.empty())
        std::printf("%s: the op's (%g %g %g) differs from native's (%g %g "
                    "%g)\n",
                    what, points[0][0], points[0][1], points[0][2],
                    native[0][0], native[0][1], native[0][2]);
    else if (!same)
        std::printf("%s: the op failed (%s)\n", what, error.c_str());
    CHECK(same);
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc != 3) {
        std::printf("usage: testRigExecOpPluginScene <mover resources> <op "
                    "resources>\n");
        return 2;
    }
    PlugRegistry::GetInstance().RegisterPlugins(
        std::vector<std::string>{argv[1], argv[2]});
    std::string error;
    const bool discovered = RigExecEnsureSceneOpRegistry(&error);
    if (!discovered)
        std::printf("%s\n", error.c_str());
    CHECK(discovered);
    RigExecRevisionBinding binding;
    binding.externalSchema = TfToken("ExternalQuadraticMover");
    const RigExecOpHandle *op = RigExecExternalOpHandle(binding);
    CHECK(op && op->IsValid() &&
          std::strcmp(RigExecRegistry::Get().Op(*op).desc->name, "Quadratic") ==
              0);

    Compare(Stage(), {VtValue(0.75f)}, 1.0f, "gain");
    Evaluators();
    {
        auto stage = Stage();
        auto mover = stage->GetPrimAtPath(SdfPath("/Rig/External"));
        CHECK(mover.CreateRelationship(TfToken("rigExec:reference"))
                  .SetTargets({SdfPath("/Rig/Reference.points")}));
        CHECK(mover.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f));
        Compare(stage,
                {VtValue(0.75f), VtValue(VtVec3fArray{GfVec3f(0, 0.25f, 0)})},
                0.5f, "reference, half weight");
    }
    // Where the native evaluator refuses the mover, the op's evaluation
    // stops with an error before it runs.
    for (const float gain : {-1.0f, -2.0f, -3.0f}) {
        auto stage = Stage();
        CHECK(stage->GetAttributeAtPath(SdfPath("/Rig/External.inputs:gain"))
                  .Set(gain));
        Compare(stage, {VtValue(gain)}, 1.0f,
                ("gain " + std::to_string(gain)).c_str(),
                gain == -2.0f
                    ? "is invalid: Quadratic: the result changes the point "
                      "count"
                    : "is invalid: Quadratic: a negative gain");
    }
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
