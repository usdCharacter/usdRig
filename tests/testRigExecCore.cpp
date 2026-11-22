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

// out = sum over elements k of (k + 1) * item, so order shows; Required
// takes a required list. Norm has a Float output to mistype an element.
RIGEXEC_DEFINE_OP_HANDLE(Weighted)
RIGEXEC_DEFINE_PORT_HANDLE(WeightedItems)
RIGEXEC_DEFINE_PORT_HANDLE(WeightedOut)
RIGEXEC_DEFINE_OP_HANDLE(Required)
RIGEXEC_DEFINE_PORT_HANDLE(RequiredItems)
RIGEXEC_DEFINE_PORT_HANDLE(RequiredOut)
RIGEXEC_DEFINE_OP_HANDLE(Norm)
RIGEXEC_DEFINE_PORT_HANDLE(NormIn)
RIGEXEC_DEFINE_PORT_HANDLE(NormOut)

bool
RunWeighted(const void *, void *, const RigExecOpIo &io, std::string *)
{
    const auto &list = *static_cast<const RigExecPortList *>(io.inputs[0]);
    Vec3 sum{0.0f, 0.0f, 0.0f};
    for (size_t k = 0; k < list.count; ++k) {
        const Vec3 &v = *static_cast<const Vec3 *>(list.items[k]);
        const float w = float(k + 1);
        sum = Vec3{sum.x + w * v.x, sum.y + w * v.y, sum.z + w * v.z};
    }
    *static_cast<Vec3 *>(io.outputs[1]) = sum;
    return true;
}

bool
RunNorm(const void *, void *, const RigExecOpIo &io, std::string *)
{
    const Vec3 &v = *static_cast<const Vec3 *>(io.inputs[0]);
    *static_cast<float *>(io.outputs[1]) = v.x + v.y + v.z;
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

    static const RigExecPortDesc weightedPorts[] = {
        {&WeightedItems(), &fakeTypes::Vec3Type(),
         kRigExecPortInput | kRigExecPortOptional | kRigExecPortArray, "items"},
        {&WeightedOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecPortDesc requiredPorts[] = {
        {&RequiredItems(), &fakeTypes::Vec3Type(),
         kRigExecPortInput | kRigExecPortArray, "items"},
        {&RequiredOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecPortDesc normPorts[] = {
        {&NormIn(), &fakeTypes::Vec3Type(), kRigExecPortInput, "in"},
        {&NormOut(), &fakeTypes::Float(), kRigExecPortOutput, "out"},
    };
    const auto op = [](const char *name, const RigExecPortDesc *ports,
                       RigExecOpRunFn run) {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = name;
        d.ports = ports;
        d.portCount = 2;
        d.run = run;
        return d;
    };
    static const RigExecOpDesc weighted =
        op("Weighted", weightedPorts, &RunWeighted);
    static const RigExecOpDesc required =
        op("Required", requiredPorts, &RunWeighted);
    static const RigExecOpDesc norm = op("Norm", normPorts, &RunNorm);
    registrar.Add(Weighted(), weighted);
    registrar.Add(Required(), required);
    registrar.Add(Norm(), norm);
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
    RigExecTapList taps;
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
    RigExecTapList bad;
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
    RigExecTapList loneTaps;
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

// Host inputs and upstream outputs interleave in call order; every
// element's producer runs first, whatever its level.
void
TestListOrder()
{
    RigExecRigDescBuilder builder;
    const RigExecNodeId a = builder.AddOp(fakeOps::Offset());
    const RigExecNodeId b = builder.AddOp(fakeOps::Offset());
    const RigExecNodeId list = builder.AddOp(Weighted());
    const RigExecInputId aIn = builder.BindInput(a, fakeOps::OffsetIn());
    builder.Connect(a, fakeOps::OffsetOut(), b, fakeOps::OffsetIn());
    const RigExecInputId first = builder.BindInput(list, WeightedItems());
    builder.Connect(b, fakeOps::OffsetOut(), list, WeightedItems());
    const RigExecInputId third = builder.BindInput(list, WeightedItems());
    builder.Connect(a, fakeOps::OffsetOut(), list, WeightedItems());
    const RigExecOutputId out = builder.BindOutput(list, WeightedOut());
    const RigExecNodeId empty = builder.AddOp(Weighted());
    const RigExecOutputId emptyOut = builder.BindOutput(empty, WeightedOut());
    std::string error;
    auto program = Compile(builder, 0, &error);
    CHECK(program);
    if (!program) {
        std::printf("%s\n", error.c_str());
        return;
    }
    auto evaluator = RigExecEvaluator::Create(program);
    const Vec3 x{1.0f, 0.0f, 0.0f}, y{0.0f, 1.0f, 0.0f}, z{0.0f, 0.0f, 1.0f};
    CHECK(evaluator->SetInput(aIn, &x));
    CHECK(evaluator->SetInput(first, &y));
    CHECK(evaluator->SetInput(third, &z));
    CHECK(evaluator->Evaluate(&error));
    // y + 2 * (x + 2) + 3 * z + 4 * (x + 1)
    const auto *result = static_cast<const Vec3 *>(evaluator->GetOutput(out));
    CHECK(result && Same(*result, Vec3{14.0f, 9.0f, 11.0f}));
    const auto *none =
        static_cast<const Vec3 *>(evaluator->GetOutput(emptyOut));
    CHECK(none && Same(*none, Vec3{0.0f, 0.0f, 0.0f}));

    // A swapped program rebinds the lists.
    CHECK(evaluator->SwapProgram(program));
    CHECK(evaluator->SetInput(aIn, &x));
    CHECK(evaluator->SetInput(first, &z));
    CHECK(evaluator->SetInput(third, &y));
    CHECK(evaluator->Evaluate(&error));
    result = static_cast<const Vec3 *>(evaluator->GetOutput(out));
    CHECK(result && Same(*result, Vec3{14.0f, 11.0f, 9.0f}));
}

void
TestListErrors()
{
    std::string error;
    {
        RigExecRigDescBuilder builder;
        builder.AddOp(Required());
        CHECK(!Compile(builder, 0, &error));
        CHECK(Contains(error, "test:Required") && Contains(error, "not fed"));
    }
    {
        // One mistyped element fails the whole list.
        RigExecRigDescBuilder builder;
        const RigExecNodeId norm = builder.AddOp(Norm());
        const RigExecNodeId list = builder.AddOp(Required());
        builder.BindInput(norm, NormIn());
        builder.BindInput(list, RequiredItems());
        builder.Connect(norm, NormOut(), list, RequiredItems());
        CHECK(!Compile(builder, 0, &error));
        CHECK(Contains(error, "joins different types"));
    }
    {
        RigExecRigDescBuilder builder;
        const RigExecNodeId list = builder.AddOp(Weighted());
        builder.Connect(list, WeightedOut(), list, WeightedItems());
        CHECK(!Compile(builder, 0, &error));
        CHECK(Contains(error, "feeds itself"));
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

// Lists over many chains give the same bits on any thread count and batch
// size.
Wide
BuildWideLists(size_t lists, size_t maxBatchNodes)
{
    Wide wide;
    RigExecRigDescBuilder builder;
    for (size_t i = 0; i < lists; ++i) {
        const fakeOps::ScalePayload payload{0.41f + 0.017f * float(i)};
        const RigExecNodeId scale = builder.AddOp(fakeOps::Scale(), &payload);
        const RigExecNodeId offset = builder.AddOp(fakeOps::Offset());
        const RigExecNodeId list = builder.AddOp(Weighted());
        wide.inputs.push_back(builder.BindInput(scale, fakeOps::ScaleIn()));
        builder.Connect(scale, fakeOps::ScaleOut(), offset,
                        fakeOps::OffsetIn());
        for (size_t k = 0; k < 1 + i % 4; ++k) {
            if (k % 2)
                wide.inputs.push_back(builder.BindInput(list, WeightedItems()));
            else
                builder.Connect(k % 4 ? scale : offset,
                                k % 4 ? fakeOps::ScaleOut()
                                      : fakeOps::OffsetOut(),
                                list, WeightedItems());
        }
        wide.outputs.push_back(builder.BindOutput(list, WeightedOut()));
    }
    std::string error;
    wide.program = Compile(builder, maxBatchNodes, &error);
    if (!wide.program)
        std::printf("%s\n", error.c_str());
    return wide;
}

void
TestListDeterminism()
{
    const size_t lists = 61;
    std::string error;
    const std::vector<Vec3> reference =
        RunWide(BuildWideLists(lists, 0), nullptr, 4, &error);
    CHECK(reference.size() == lists * 4);
    for (const size_t threads : {size_t(1), size_t(2), size_t(4)}) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch :
             {size_t(0), size_t(1), size_t(2), size_t(7)}) {
            const Wide wide = BuildWideLists(lists, batch);
            CHECK(wide.program);
            const std::vector<Vec3> got = RunWide(wide, tasks.get(), 4, &error);
            bool same = got.size() == reference.size();
            for (size_t i = 0; same && i < got.size(); ++i)
                same = Same(got[i], reference[i]);
            CHECK(same);
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
    TestListOrder();
    TestListErrors();
    TestListDeterminism();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
