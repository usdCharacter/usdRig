// The fixture mover plugin and its op plugin together: the scene program
// discovers the op plugin, binds the mover's inputs to its Quadratic op and
// runs it through the op core, giving the native evaluator's bits -- with a
// reference read, under an envelope, and when the mover refuses.
//
//   testRigExecOpPluginScene <mover plugin resources> <op plugin resources>
#include "rigExec/api/registry.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExecScene/externalOps.h"
#include "rigExecScene/opPlugins.h"

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

#include "rigExecSceneProgramDrive.h"

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

/// Native and scene program agree on the body's points.
void
Compare(const UsdStageRefPtr &stage, const char *what)
{
    RigExecRigEvaluator rig(stage, SdfPath("/Rig"));
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    for (const std::string &e : errors)
        std::printf("compile: %s\n", e.c_str());
    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1));
    const auto moved = pose.movedProperties.find(kBody);
    VtVec3fArray native;
    if (moved != pose.movedProperties.end())
        native = moved->second.Get<VtVec3fArray>();
    rigExecSceneProgramDrive::SceneRun scene(stage, SdfPath("/Rig"), 1, 1);
    CHECK(scene.ok && scene.Evaluate(1));
    const VtValue value = scene.Final(kBody, RigExecSceneValueDomain::Points);
    const bool same =
        value.IsHolding<VtVec3fArray>() &&
        value.UncheckedGet<VtVec3fArray>().size() == native.size() &&
        std::memcmp(value.UncheckedGet<VtVec3fArray>().cdata(), native.cdata(),
                    native.size() * sizeof(GfVec3f)) == 0;
    if (!same)
        std::printf("%s: the scene program differs from native\n", what);
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

    Compare(Stage(), "gain");
    {
        auto stage = Stage();
        auto mover = stage->GetPrimAtPath(SdfPath("/Rig/External"));
        CHECK(mover.CreateRelationship(TfToken("rigExec:reference"))
                  .SetTargets({SdfPath("/Rig/Reference.points")}));
        CHECK(mover.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.5f));
        Compare(stage, "reference, half weight");
    }
    for (const float gain : {-1.0f, -2.0f, -3.0f}) {
        auto stage = Stage();
        CHECK(stage->GetAttributeAtPath(SdfPath("/Rig/External.inputs:gain"))
                  .Set(gain));
        Compare(stage, ("refused, gain " + std::to_string(gain)).c_str());
    }
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
