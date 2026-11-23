// The playback runtime and the op core link into one executable with no
// OpenUSD library: the runtime plays the minimal program, and a rig
// described by hand runs through the op core's single registry.
#include "registryFixture/fakeOps.h"
#include "registryFixture/fakeTypes.h"
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecRuntime/runtime.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace rigExec;
using fakeTypes::Vec3;

namespace {

int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

void
TestRuntimePlays(const char *fixturePath)
{
    std::ifstream file(fixturePath, std::ios::binary);
    CHECK(file.good());
    const std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(file), {}};
    std::string error;
    auto reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (reader) {
        CHECK(reader->Execute(&error));
    }
    if (!error.empty()) {
        std::printf("%s\n", error.c_str());
    }
}

// in -> Scale(2) -> Offset -> out.
void
TestOpCoreEvaluates()
{
    RigExecRigDescBuilder builder;
    const fakeOps::ScalePayload payload{2.0f};
    const RigExecNodeId scale = builder.AddOp(fakeOps::Scale(), &payload);
    const RigExecNodeId offset = builder.AddOp(fakeOps::Offset());
    builder.Connect(scale, fakeOps::ScaleOut(), offset, fakeOps::OffsetIn());
    const RigExecInputId in = builder.BindInput(scale, fakeOps::ScaleIn());
    const RigExecOutputId out =
        builder.BindOutput(offset, fakeOps::OffsetOut());
    std::string error;
    const auto desc = builder.Build(&error);
    CHECK(desc);
    if (!desc) {
        std::printf("%s\n", error.c_str());
        return;
    }
    const auto program = RigExecCompile(*desc, {}, &error);
    CHECK(program);
    if (!program) {
        std::printf("%s\n", error.c_str());
        return;
    }
    const auto evaluator = RigExecEvaluator::Create(program, {}, &error);
    CHECK(evaluator);
    if (!evaluator) {
        return;
    }
    const Vec3 value{1.0f, 2.0f, 3.0f};
    CHECK(evaluator->SetInput(in, &value));
    CHECK(evaluator->Evaluate(&error));
    const auto *result = static_cast<const Vec3 *>(evaluator->GetOutput(out));
    const Vec3 expected{3.0f, 5.0f, 7.0f};
    CHECK(result && std::memcmp(result, &expected, sizeof(Vec3)) == 0);
}

} // namespace

int
main(int argc, char **argv)
{
    if (argc != 2) {
        std::printf("expected the minimal.rigexec fixture\n");
        return 2;
    }
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(RigExecLinkedRegistrations(), &errors)) {
        for (const std::string &e : errors) {
            std::printf("%s\n", e.c_str());
        }
        return 1;
    }
    TestRuntimePlays(argv[1]);
    TestOpCoreEvaluates();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
