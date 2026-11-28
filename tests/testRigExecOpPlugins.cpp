// Op plugins join the op registry through plugInfo discovery: the plugin's
// op runs in the op core, its schema type is known to OpenUSD and its binder
// is found by schema type. A plugin built for another registry ABI, one
// without its entry symbol, and one discovered after the registry froze are
// reported by name, and fail only the movers of a schema type the plugin
// binds.
//
//   testRigExecOpPlugins <mode> <plugin resources> <library> <scratch dir>
//       [<mover plugin resources> <quadratic op plugin resources>]
//
// mode: good, abi, symbol, late, or with the mover plugins scopeUsed (a
// broken plugin binds the quadratic mover) and scopeOther (it binds another
// type).
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExecBuiltins/builtins.h"
#include "rigExecScene/externalOps.h"
#include "rigExecScene/opPlugins.h"
#include "rigExecStdTypes/stdTypes.h"

#include "pxr/base/plug/plugin.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/fileUtils.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

const char *kSchemaType = "TestOffsetMover";

/// A plugInfo naming \p library with \p abi and \p entry, in \p dir,
/// registered with OpenUSD.
void
RegisterBroken(const std::string &dir, const std::string &library, int abi,
               const std::string &entry, const char *schemaType = kSchemaType)
{
    CHECK(TfMakeDirs(dir, -1, true) && TfIsDir(dir));
    {
        std::ofstream out(dir + "/plugInfo.json");
        out << "{\"Plugins\": [{\"Type\": \"library\", \"Name\": "
               "\"brokenOffsetOps\", \"Root\": \".\", \"ResourcePath\": "
               "\".\", \"LibraryPath\": \""
            << library
            << "\", \"Info\": {\"RigExecOpLibrary\": {\"abiVersion\": " << abi
            << ", \"entry\": \"" << entry << "\", \"schemaTypes\": [\""
            << schemaType << "\"]}}}]}\n";
        CHECK(out.good());
    }
    const PlugPluginPtrVector added =
        PlugRegistry::GetInstance().RegisterPlugins(dir);
    CHECK(added.size() == 1 && added[0]->GetName() == "brokenOffsetOps");
}

/// The op named \p name in the registry, or null.
const RigExecRegisteredOp *
FindOp(const char *nameSpace, const char *name)
{
    const RigExecRegistry &registry = RigExecRegistry::Get();
    for (size_t k = 0; k < registry.OpCount(); ++k) {
        const RigExecRegisteredOp &op = registry.OpAt(uint32_t(k));
        if (std::string(op.desc->nameSpace) == nameSpace &&
            std::string(op.desc->name) == name)
            return &op;
    }
    return nullptr;
}

void
Good()
{
    std::string error;
    const bool ok = RigExecEnsureSceneOpRegistry(&error);
    if (!ok)
        std::printf("%s\n", error.c_str());
    CHECK(ok);
    CHECK(RigExecSceneOpPluginErrors().empty());
    CHECK(RigExecSceneOpSchemaJoined(kSchemaType));
    CHECK(FindOp("rig", "Patch"));
    const RigExecRegisteredOp *offset = FindOp("test", "Offset");
    CHECK(offset);
    CHECK(UsdSchemaRegistry::GetInstance().FindConcretePrimDefinition(
        TfToken(kSchemaType)));
    RigExecRevisionBinding binding;
    binding.externalSchema = TfToken(kSchemaType);
    const RigExecOpHandle *handle = RigExecExternalOpHandle(binding);
    CHECK(handle && offset && handle->Index() == offset->index);
    if (!handle)
        return;
    // The plugin's op runs in the op core: one node, fed by the host.
    const RigExecOpDesc &desc = *RigExecRegistry::Get().Op(*handle).desc;
    RigExecRigDescBuilder builder;
    const RigExecNodeId node = builder.AddOp(*handle);
    const RigExecInputId points = builder.BindInput(node, *desc.ports[0].port);
    const RigExecInputId by = builder.BindInput(node, *desc.ports[1].port);
    const RigExecOutputId out = builder.BindOutput(node, *desc.ports[2].port);
    auto rig = builder.Build(&error);
    auto program = rig ? RigExecCompile(*rig, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(evaluator);
    if (!evaluator) {
        std::printf("%s\n", error.c_str());
        return;
    }
    rigExecStd::RigExecArray<rigExecStd::Vec3f> in(1);
    in[0] = rigExecStd::Vec3f(1, 2, 3);
    const rigExecStd::Vec3f offsetBy(0.5f, -1, 2);
    CHECK(evaluator->SetInput(points, &in, &error));
    CHECK(evaluator->SetInput(by, &offsetBy, &error));
    CHECK(evaluator->Evaluate(&error));
    const auto *moved =
        static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
            evaluator->GetOutput(out));
    CHECK(moved && moved->Size() == 1 && (*moved)[0][0] == 1.5f &&
          (*moved)[0][1] == 1.0f && (*moved)[0][2] == 5.0f);
}

/// Discovery reports the plugin by name; the registry initializes without
/// it, and only its schema type is failed.
void
Refused(const char *why, const char *schemaType = kSchemaType)
{
    std::string error;
    CHECK(RigExecEnsureSceneOpRegistry(&error));
    std::string reported;
    for (const std::string &line : RigExecSceneOpPluginErrors())
        reported += line + "\n";
    std::printf("reported:\n%s", reported.c_str());
    CHECK(reported.find("op plugins considered: ") != std::string::npos);
    CHECK(reported.find("brokenOffsetOps") != std::string::npos ||
          reported.find("testOffsetOps") != std::string::npos);
    CHECK(reported.find(why) != std::string::npos);
    CHECK(RigExecRegistry::IsInitialized() && FindOp("rig", "Patch"));
    CHECK(!FindOp("test", "Offset"));
    std::string failed;
    CHECK(!RigExecSceneOpSchemaJoined(schemaType, &failed));
    CHECK(failed.find(why) != std::string::npos);
    CHECK(RigExecSceneOpSchemaJoined("RigExecLayeredSkinMover"));
}

/// Whether a mover of \p schemaType runs on its op: every plugin binding
/// the type joined, and its op runs as one node fed through its binder;
/// false with \p error naming the type and the plugins that failed it.
bool
RunsOp(const char *schemaType, std::string *error)
{
    std::string why;
    if (!RigExecSceneOpSchemaJoined(schemaType, &why)) {
        *error =
            std::string("op plugin unavailable for ") + schemaType + ": " + why;
        return false;
    }
    RigExecRevisionBinding binding;
    binding.externalSchema = TfToken(schemaType);
    if (!RigExecExternalOpHandle(binding)) {
        *error = std::string("no op for ") + schemaType;
        return false;
    }
    if (std::string(schemaType) != "ExternalQuadraticMover")
        return true;
    const std::vector<VtValue> inputs = {VtValue(0.75f)};
    std::vector<GfVec3f> points = {GfVec3f(1, 2, 3), GfVec3f(-2, 0, 0)};
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
    return evaluator &&
           RigExecWriteExternalOpInputs(binding, values,
                                        RigExecWeightPacket::Constant(1.0f),
                                        points, node, evaluator.get(), error) &&
           evaluator->Evaluate(error);
}

/// A broken plugin binding \p brokenType: the quadratic mover runs unless
/// that is its type, and a mover on the built-in ops always runs.
void
Scoped(const std::string &scratch, const std::string &library,
       const char *brokenType, bool quadraticRuns)
{
    RegisterBroken(scratch, library, 999, "testOffsetOps_RigExecOpLibrary",
                   brokenType);
    Refused("registry ABI", brokenType);
    std::string error;
    const bool quadratic = RunsOp("ExternalQuadraticMover", &error);
    std::printf("quadratic mover: %s\n", quadratic ? "ran" : error.c_str());
    CHECK(quadratic == quadraticRuns);
    if (!quadraticRuns)
        CHECK(error.find("brokenOffsetOps") != std::string::npos &&
              error.find(brokenType) != std::string::npos);
    error.clear();
    const bool skin = RunsOp("RigExecLayeredSkinMover", &error);
    if (!skin)
        std::printf("layered skin mover: %s\n", error.c_str());
    CHECK(skin);
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc != 5 && argc != 7) {
        std::printf("usage: testRigExecOpPlugins <mode> <resources> "
                    "<library> <scratch> [<mover resources> <quadratic "
                    "resources>]\n");
        return 2;
    }
    const std::string mode = argv[1], resources = argv[2], library = argv[3],
                      scratch = argv[4];
    if (mode == "good") {
        PlugRegistry::GetInstance().RegisterPlugins(resources);
        Good();
    } else if (mode == "abi") {
        RegisterBroken(scratch + "/abi", library, 999,
                       "testOffsetOps_RigExecOpLibrary");
        Refused("registry ABI");
    } else if (mode == "symbol") {
        RegisterBroken(scratch + "/symbol", library,
                       int(kRigExecRegistryAbiVersion), "noSuchEntry");
        Refused("noSuchEntry");
    } else if (mode == "late") {
        CHECK(RigExecEnsureBuiltinRegistry());
        PlugRegistry::GetInstance().RegisterPlugins(resources);
        Refused("initialized before op plugin discovery");
    } else if (argc == 7 && mode == "scopeUsed") {
        PlugRegistry::GetInstance().RegisterPlugins(argv[5]);
        Scoped(scratch + "/scopeUsed", library, "ExternalQuadraticMover",
               false);
    } else if (argc == 7 && mode == "scopeOther") {
        PlugRegistry::GetInstance().RegisterPlugins(
            std::vector<std::string>{argv[5], argv[6]});
        Scoped(scratch + "/scopeOther", library, kSchemaType, true);
    } else {
        std::printf("unknown mode %s\n", mode.c_str());
        return 2;
    }
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
