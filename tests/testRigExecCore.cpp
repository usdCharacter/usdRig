// The USD-free core: build a rig description by hand, compile, evaluate,
// read outputs and taps, and check results are bit-identical across
// thread counts and batch sizes.
#include "registryFixture/fakeOps.h"
#include "registryFixture/fakeTypes.h"
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"

#include <cstdio>
#include <cstring>
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

bool
Contains(const std::string &text, const char *part)
{
    return text.find(part) != std::string::npos;
}

bool
Same(const Vec3 &a, const Vec3 &b)
{
    return std::memcmp(&a, &b, sizeof(Vec3)) == 0;
}

// An op that fails on a negative x, to check error reporting.
RIGEXEC_DEFINE_OP_HANDLE(Check)
RIGEXEC_DEFINE_PORT_HANDLE(CheckIn)
RIGEXEC_DEFINE_PORT_HANDLE(CheckOut)

bool
RunCheck(const void *, void *, const RigExecOpIo &io, std::string *error)
{
    const Vec3 &in = *static_cast<const Vec3 *>(io.inputs[CheckIn().Index()]);
    if (in.x < 0.0f) {
        *error = "negative x " + std::to_string(int(in.x));
        return false;
    }
    *static_cast<Vec3 *>(io.outputs[CheckOut().Index()]) = in;
    return true;
}

void
RegisterCheck(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&CheckIn(), &fakeTypes::Vec3Type(), kRigExecPortInput, "in"},
        {&CheckOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecOpDesc desc = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Check";
        d.ports = ports;
        d.portCount = 2;
        d.run = &RunCheck;
        return d;
    }();
    registrar.Add(Check(), desc);
}

const RigExecOpLibraryDesc kCheckLibrary = [] {
    RigExecOpLibraryDesc d;
    d.name = "testCheck";
    d.registerOps = &RegisterCheck;
    return d;
}();

std::shared_ptr<const RigExecProgram>
Compile(RigExecRigDescBuilder &builder, size_t maxBatchNodes = 0,
        std::string *error = nullptr)
{
    std::shared_ptr<const RigExecRigDesc> desc = builder.Build(error);
    if (!desc)
        return nullptr;
    RigExecCompileOptions options;
    options.maxBatchNodes = maxBatchNodes;
    return RigExecCompile(*desc, options, error);
}

struct TapLog {
    std::vector<Vec3> values;
    std::vector<uint32_t> types;
};

void
RecordTap(void *context, size_t, const void *value, uint32_t type)
{
    auto *log = static_cast<TapLog *>(context);
    log->values.push_back(*static_cast<const Vec3 *>(value));
    log->types.push_back(type);
}

// in -> Scale(2) -> Offset -> out, with a tap on the Scale output.
void
TestChain()
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
    auto program = Compile(builder, 0, &error);
    CHECK(program);
    if (!program) {
        std::printf("%s\n", error.c_str());
        return;
    }
    auto evaluator = RigExecEvaluator::Create(program);
    CHECK(evaluator);

    TapLog log;
    RigExecTapSet taps;
    taps.taps.push_back({scale, &fakeOps::ScaleOut()});
    CHECK(evaluator->SetTaps(taps, &RecordTap, &log));

    const Vec3 value{1.0f, 2.0f, 3.0f};
    CHECK(evaluator->SetInput(in, &value));
    CHECK(evaluator->Evaluate(&error));
    const auto *result = static_cast<const Vec3 *>(evaluator->GetOutput(out));
    CHECK(result && Same(*result, Vec3{3.0f, 5.0f, 7.0f}));
    CHECK(log.values.size() == 1 &&
          Same(log.values[0], Vec3{2.0f, 4.0f, 6.0f}));
    CHECK(log.types.size() == 1 &&
          log.types[0] == fakeTypes::Vec3Type().Index());

    // A tap on an input, or on a node the program lacks, is refused and the
    // previous taps stay.
    RigExecTapSet bad;
    bad.taps.push_back({scale, &fakeOps::ScaleIn()});
    CHECK(!evaluator->SetTaps(bad, &RecordTap, &log, &error));
    bad.taps[0] = {7, &fakeOps::ScaleOut()};
    CHECK(!evaluator->SetTaps(bad, &RecordTap, &log, &error));
    log.values.clear();
    CHECK(evaluator->Evaluate());
    CHECK(log.values.size() == 1);

    // Swapping in a program the taps don't fit fails and keeps the old one.
    RigExecRigDescBuilder single;
    const RigExecNodeId lone = single.AddOp(fakeOps::Offset());
    single.BindInput(lone, fakeOps::OffsetIn());
    auto other = Compile(single);
    CHECK(other);
    RigExecTapSet loneTaps;
    loneTaps.taps.push_back({1, &fakeOps::OffsetOut()});
    CHECK(evaluator->SetTaps(loneTaps, &RecordTap, &log));
    CHECK(!evaluator->SwapProgram(other, &error));
    CHECK(Contains(error, "tap 0"));
    CHECK(evaluator->Evaluate());

    // Without taps the swap succeeds and inputs reset.
    CHECK(evaluator->SetTaps({}, nullptr, nullptr));
    CHECK(evaluator->SwapProgram(other));
    CHECK(evaluator->Evaluate());
    CHECK(!evaluator->GetOutput(out));
}

void
TestBuildErrors()
{
    std::string error;
    {
        RigExecRigDescBuilder builder;
        builder.AddOp(fakeOps::Scale());
        CHECK(!builder.Build(&error));
        CHECK(Contains(error, "needs a payload"));
    }
    {
        RigExecRigDescBuilder builder;
        const fakeOps::ScalePayload payload;
        const RigExecNodeId scale = builder.AddOp(fakeOps::Scale(), &payload);
        builder.BindInput(scale, fakeOps::OffsetIn());
        CHECK(!builder.Build(&error));
        CHECK(Contains(error, "no such input"));
    }
    {
        RigExecRigDescBuilder builder;
        const RigExecNodeId a = builder.AddOp(fakeOps::Offset());
        builder.Connect(a, fakeOps::OffsetIn(), a, fakeOps::OffsetIn());
        CHECK(!builder.Build(&error));
        CHECK(Contains(error, "names a port"));
    }
}

void
TestCompileErrors()
{
    std::string error;
    {
        RigExecRigDescBuilder builder;
        builder.AddOp(fakeOps::Offset());
        CHECK(!Compile(builder, 0, &error));
        CHECK(Contains(error, "fake:Offset") && Contains(error, "not fed"));
    }
    {
        RigExecRigDescBuilder builder;
        const RigExecNodeId a = builder.AddOp(fakeOps::Offset());
        const RigExecNodeId b = builder.AddOp(fakeOps::Offset());
        builder.BindInput(a, fakeOps::OffsetIn());
        builder.Connect(a, fakeOps::OffsetOut(), b, fakeOps::OffsetIn());
        builder.BindInput(b, fakeOps::OffsetIn());
        CHECK(!Compile(builder, 0, &error));
        CHECK(Contains(error, "fed twice"));
    }
    {
        RigExecRigDescBuilder builder;
        const RigExecNodeId a = builder.AddOp(fakeOps::Offset());
        const RigExecNodeId b = builder.AddOp(fakeOps::Offset());
        builder.BindInput(a, fakeOps::OffsetIn());
        builder.BindInput(b, fakeOps::OffsetIn());
        builder.Order(a, b);
        builder.Order(b, a);
        CHECK(!Compile(builder, 0, &error));
        CHECK(Contains(error, "cycle"));
    }
}

// Many independent chains of every op, with failures planted in some.
struct Wide {
    std::shared_ptr<const RigExecProgram> program;
    std::vector<RigExecInputId> inputs;
    std::vector<RigExecOutputId> outputs;
};

Wide
BuildWide(size_t chains, size_t maxBatchNodes)
{
    Wide wide;
    RigExecRigDescBuilder builder;
    for (size_t i = 0; i < chains; ++i) {
        const fakeOps::ScalePayload payload{0.37f + 0.013f * float(i)};
        const RigExecNodeId scale = builder.AddOp(fakeOps::Scale(), &payload);
        const RigExecNodeId check = builder.AddOp(Check());
        const RigExecNodeId offset = builder.AddOp(fakeOps::Offset());
        builder.Connect(scale, fakeOps::ScaleOut(), check, CheckIn());
        builder.Connect(check, CheckOut(), offset, fakeOps::OffsetIn());
        wide.inputs.push_back(builder.BindInput(scale, fakeOps::ScaleIn()));
        wide.outputs.push_back(
            builder.BindOutput(offset, fakeOps::OffsetOut()));
    }
    std::string error;
    wide.program = Compile(builder, maxBatchNodes, &error);
    if (!wide.program)
        std::printf("%s\n", error.c_str());
    return wide;
}

std::vector<Vec3>
RunWide(const Wide &wide, RigExecTaskSystem *tasks, int frames,
        std::string *error)
{
    RigExecEvaluatorOptions options;
    options.tasks = tasks;
    auto evaluator = RigExecEvaluator::Create(wide.program, options);
    std::vector<Vec3> results;
    for (int frame = 0; frame < frames; ++frame) {
        for (size_t i = 0; i < wide.inputs.size(); ++i) {
            const float t = float(frame) * 0.1f + float(i);
            const Vec3 value{t * 1.1f, t * -0.7f, t * 3.3f};
            evaluator->SetInput(wide.inputs[i], &value);
        }
        if (!evaluator->Evaluate(error))
            return results;
        for (const RigExecOutputId out : wide.outputs)
            results.push_back(
                *static_cast<const Vec3 *>(evaluator->GetOutput(out)));
    }
    return results;
}

void
TestDeterminism()
{
    const size_t chains = 97;
    std::vector<Vec3> reference;
    {
        std::string error;
        reference = RunWide(BuildWide(chains, 0), nullptr, 5, &error);
        CHECK(reference.size() == chains * 5);
    }
    for (const size_t threads : {size_t(1), size_t(2), size_t(4)}) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch :
             {size_t(0), size_t(1), size_t(2), size_t(7)}) {
            const Wide wide = BuildWide(chains, batch);
            CHECK(wide.program);
            std::string error;
            const std::vector<Vec3> got = RunWide(wide, tasks.get(), 5, &error);
            CHECK(got.size() == reference.size());
            bool same = got.size() == reference.size();
            for (size_t i = 0; same && i < got.size(); ++i)
                same = Same(got[i], reference[i]);
            CHECK(same);
            if (!same)
                std::printf("threads %zu batch %zu differ\n", threads, batch);
        }
    }
}

// The reported failure is the first in batch order, on any thread count.
void
TestFailureOrder()
{
    for (const size_t threads : {size_t(1), size_t(4)}) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch : {size_t(0), size_t(1), size_t(3)}) {
            const Wide wide = BuildWide(16, batch);
            RigExecEvaluatorOptions options;
            options.tasks = tasks.get();
            auto evaluator = RigExecEvaluator::Create(wide.program, options);
            for (size_t i = 0; i < wide.inputs.size(); ++i) {
                const float x = (i == 5 || i == 11) ? -1.0f : 1.0f;
                const Vec3 value{x, 0.0f, 0.0f};
                evaluator->SetInput(wide.inputs[i], &value);
            }
            std::string error;
            CHECK(!evaluator->Evaluate(&error));
            // Chain 5's Check node is node 5 * 3 + 1.
            CHECK(Contains(error, "node 16 (test:Check) failed: negative x"));
        }
    }
}

} // namespace

int
main()
{
    RigExecRegistryInit init = RigExecLinkedRegistrations();
    init.opLibraries.push_back(&kCheckLibrary);
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(init, &errors)) {
        for (const std::string &e : errors)
            std::printf("%s\n", e.c_str());
        return 1;
    }
    TestChain();
    TestBuildErrors();
    TestCompileErrors();
    TestDeterminism();
    TestFailureOrder();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
