// Op plugins join the op registry through plugInfo discovery: the plugin's
// op runs in the op core, its schema type is known to OpenUSD and its binder
// is found by schema type. A plugin built for another registry ABI, one
// without its entry symbol, and one discovered after the registry froze are
// reported by name.
//
//   testRigExecOpPlugins <mode> <plugin resources> <library> <scratch dir>
//
// mode: good, abi, symbol or late.
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

#include <cstdio>
#include <fstream>
#include <string>

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
               const std::string &entry)
{
    CHECK(TfMakeDirs(dir, -1, true) && TfIsDir(dir));
    {
        std::ofstream out(dir + "/plugInfo.json");
        out << "{\"Plugins\": [{\"Type\": \"library\", \"Name\": "
               "\"brokenOffsetOps\", \"Root\": \".\", \"ResourcePath\": "
               "\".\", \"LibraryPath\": \""
            << library
            << "\", \"Info\": {\"RigExecOpLibrary\": {\"abiVersion\": " << abi
            << ", \"entry\": \"" << entry << "\"}}}]}\n";
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

/// Discovery refuses the plugin by name; the built-ins still register.
void
Refused(const char *why)
{
    std::string error;
    CHECK(!RigExecEnsureSceneOpRegistry(&error));
    std::printf("reported: %s\n", error.c_str());
    for (const std::string &plugin : RigExecSceneOpPluginsConsidered())
        std::printf("considered: %s\n", plugin.c_str());
    CHECK(error.find("op plugins considered: ") != std::string::npos);
    CHECK(error.find("brokenOffsetOps") != std::string::npos ||
          error.find("testOffsetOps") != std::string::npos);
    CHECK(error.find(why) != std::string::npos);
    CHECK(RigExecRegistry::IsInitialized() && FindOp("rig", "Patch"));
    CHECK(!FindOp("test", "Offset"));
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc != 5) {
        std::printf("usage: testRigExecOpPlugins <mode> <resources> "
                    "<library> <scratch>\n");
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
