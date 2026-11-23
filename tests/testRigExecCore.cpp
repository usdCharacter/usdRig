// The op core: build a rig description by hand, compile, evaluate,
// read outputs and taps, and check results are bit-identical across
// thread counts and batch sizes.
#include "registryFixture/fakeOps.h"
#include "registryFixture/fakeTypes.h"
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExec/internal/program/program.h"
#include "rigExecGraph/windowHistory.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

using namespace rigExec;
using fakeTypes::Vec3;

namespace {

// Whether T({}, args...) compiles: true when anyone can make the passkey.
template <class T, class Args, class = void>
struct BracedKeyConstructs : std::false_type {};
template <class T, class... A>
struct BracedKeyConstructs<T, std::tuple<A...>,
                           std::void_t<decltype(T({}, std::declval<A>()...))>>
    : std::true_type {};

struct OpenGate {
    struct Key {};
    explicit OpenGate(Key, int)
    {
    }
};
static_assert(BracedKeyConstructs<OpenGate, std::tuple<int>>::value,
              "the probe detects an open passkey");
static_assert(!BracedKeyConstructs<RigExecProgram, std::tuple<>>::value,
              "only RigExecCompile constructs a program");
static_assert(
    !BracedKeyConstructs<RigExecEvaluator,
                         std::tuple<std::shared_ptr<const RigExecProgram>,
                                    const RigExecEvaluatorOptions &>>::value,
    "only RigExecEvaluator::Create constructs an evaluator");
static_assert(!BracedKeyConstructs<
                  RigExecRigDesc,
                  std::tuple<std::unique_ptr<const RigExecRigDescData>>>::value,
              "only RigExecRigDescBuilder::Build constructs a description");

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

// A windowed op: captures its input; out sums the usable tail of its ring
// and this tick, the k-th oldest weighted k + 1, so order shows. A history
// tick that only captures passes its input through; a negative x declines.
RIGEXEC_DEFINE_OP_HANDLE(RingSum)
RIGEXEC_DEFINE_PORT_HANDLE(RingSumIn)
RIGEXEC_DEFINE_PORT_HANDLE(RingSumOut)

struct RingPayload {
    uint32_t frames = 0;
};

uint32_t
RingFrames(const void *payload)
{
    return payload ? static_cast<const RingPayload *>(payload)->frames : 0;
}

bool
RunRingSum(const void *, void *, const RigExecOpIo &io, std::string *error)
{
    const Vec3 &in = *static_cast<const Vec3 *>(io.inputs[0]);
    Vec3 &out = *static_cast<Vec3 *>(io.outputs[1]);
    const RigExecWindowRun &window = *io.window;
    if (in.x < 0.0f) {
        *error = "declined";
        return false;
    }
    auto packet = std::make_shared<RigExecWindowPacket>();
    packet->scalars = {in.x, in.y, in.z};
    *window.capture = packet;
    if (window.captureOnly) {
        out = in;
        return true;
    }
    std::vector<const RigExecWindowPacket *> tail;
    RigExecWindowUsableTail(*window.ring, packet.get(), &tail);
    Vec3 sum{0.0f, 0.0f, 0.0f};
    for (size_t k = 0; k < tail.size(); ++k) {
        const float w = float(k + 1);
        sum = Vec3{sum.x + w * float(tail[k]->scalars[0]),
                   sum.y + w * float(tail[k]->scalars[1]),
                   sum.z + w * float(tail[k]->scalars[2])};
    }
    out = sum;
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

    static const RigExecPortDesc ringPorts[] = {
        {&RingSumIn(), &fakeTypes::Vec3Type(), kRigExecPortInput, "in"},
        {&RingSumOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecOpDesc ring = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "RingSum";
        d.ports = ringPorts;
        d.portCount = 2;
        d.run = &RunRingSum;
        d.threading = RigExecOpThreading::Windowed;
        d.windowFrames = &RingFrames;
        d.payload = RigExecPayloadHandlingFor<RingPayload>();
        return d;
    }();
    registrar.Add(RingSum(), ring);
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

// Per chain: P (Offset) -> A (RingSum, 2 frames) -> B (RingSum, 3 frames)
// -> C (Offset), and Q (Offset) ordered before A. Outputs A, B, C and Q.
struct WindowRig {
    std::shared_ptr<const RigExecProgram> program;
    std::vector<RigExecInputId> inputs;
    std::vector<RigExecOutputId> outputs;
};

WindowRig
BuildWindowRig(size_t chains, size_t maxBatchNodes)
{
    WindowRig rig;
    RigExecRigDescBuilder builder;
    const RingPayload two{2}, three{3};
    for (size_t i = 0; i < chains; ++i) {
        const RigExecNodeId p = builder.AddOp(fakeOps::Offset());
        const RigExecNodeId q = builder.AddOp(fakeOps::Offset());
        const RigExecNodeId a = builder.AddOp(RingSum(), &two);
        const RigExecNodeId b = builder.AddOp(RingSum(), &three);
        const RigExecNodeId c = builder.AddOp(fakeOps::Offset());
        builder.Connect(p, fakeOps::OffsetOut(), a, RingSumIn());
        builder.Connect(a, RingSumOut(), b, RingSumIn());
        builder.Connect(b, RingSumOut(), c, fakeOps::OffsetIn());
        builder.Order(q, a);
        rig.inputs.push_back(builder.BindInput(p, fakeOps::OffsetIn()));
        rig.inputs.push_back(builder.BindInput(q, fakeOps::OffsetIn()));
        rig.outputs.push_back(builder.BindOutput(a, RingSumOut()));
        rig.outputs.push_back(builder.BindOutput(b, RingSumOut()));
        rig.outputs.push_back(builder.BindOutput(c, fakeOps::OffsetOut()));
        rig.outputs.push_back(builder.BindOutput(q, fakeOps::OffsetOut()));
    }
    std::string error;
    rig.program = Compile(builder, maxBatchNodes, &error);
    if (!rig.program)
        std::printf("%s\n", error.c_str());
    return rig;
}

Vec3
FrameValue(int frame, size_t input)
{
    const float t = float(frame), i = float(input);
    return Vec3{0.25f * t + i, 0.5f * t - 0.125f * i, 1.0f + 0.0625f * t};
}

void
SetFrame(const WindowRig &rig, RigExecEvaluator &evaluator, int frame)
{
    for (size_t i = 0; i < rig.inputs.size(); ++i) {
        const Vec3 value = FrameValue(frame, i);
        evaluator.SetInput(rig.inputs[i], &value);
    }
}

std::vector<Vec3>
Outputs(const WindowRig &rig, const RigExecEvaluator &evaluator)
{
    std::vector<Vec3> values;
    for (const RigExecOutputId out : rig.outputs)
        values.push_back(*static_cast<const Vec3 *>(evaluator.GetOutput(out)));
    return values;
}

bool
SameAll(const std::vector<Vec3> &a, const std::vector<Vec3> &b)
{
    bool same = a.size() == b.size();
    for (size_t i = 0; same && i < a.size(); ++i)
        same = Same(a[i], b[i]);
    return same;
}

/// Plays frames 0 .. frames - 1 in order; outputs per frame.
std::vector<std::vector<Vec3>>
Play(const WindowRig &rig, RigExecEvaluator &evaluator, int frames)
{
    std::vector<std::vector<Vec3>> played;
    for (int f = 0; f < frames; ++f) {
        SetFrame(rig, evaluator, f);
        if (!evaluator.Evaluate())
            return played;
        evaluator.PushHistory();
        played.push_back(Outputs(rig, evaluator));
    }
    return played;
}

/// Jumps to \p frame: a cleared history, every earlier tick it reaches,
/// then the frame.
std::vector<Vec3>
Jump(const WindowRig &rig, RigExecEvaluator &evaluator, int frame)
{
    evaluator.ClearHistory();
    for (int k = int(evaluator.GetHistoryDepth()); k >= 1; --k) {
        SetFrame(rig, evaluator, frame - k);
        CHECK(evaluator.EvaluateHistory());
    }
    SetFrame(rig, evaluator, frame);
    CHECK(evaluator.Evaluate());
    return Outputs(rig, evaluator);
}

void
TestWindowLayout()
{
    const WindowRig rig = BuildWindowRig(1, 0);
    CHECK(rig.program);
    if (!rig.program)
        return;
    // Nodes: P 0, Q 1, A 2, B 3, C 4.
    const RigExecProgram &p = *rig.program;
    CHECK(p.windowedNodes.size() == 2);
    if (p.windowedNodes.size() != 2)
        return;
    CHECK(p.windowedNodes[0].node == 2 && p.windowedNodes[0].frames == 2);
    CHECK(p.windowedNodes[0].inner.empty() && p.windowedNodes[0].feedsWindow);
    CHECK(p.windowedNodes[1].node == 3 && p.windowedNodes[1].frames == 3);
    CHECK(p.windowedNodes[1].inner == std::vector<uint32_t>{0});
    CHECK(!p.windowedNodes[1].feedsWindow);
    CHECK((p.historyMask == std::vector<char>{1, 1, 1, 1, 0}));
    CHECK((p.windowOfNode == std::vector<uint32_t>{kRigExecInvalidIndex,
                                                   kRigExecInvalidIndex, 0, 1,
                                                   kRigExecInvalidIndex}));
    auto evaluator = RigExecEvaluator::Create(rig.program);
    CHECK(evaluator->GetWindowCount() == 2);
    // B reads 3 ticks; A's capture at each of those read its own 2.
    CHECK(evaluator->GetHistoryDepth() == 5);
}

void
TestWindowHistory()
{
    const WindowRig rig = BuildWindowRig(1, 0);
    auto played = RigExecEvaluator::Create(rig.program);
    const std::vector<std::vector<Vec3>> reference = Play(rig, *played, 10);
    CHECK(reference.size() == 10);
    if (reference.size() != 10)
        return;
    // Rings keep their newest frames.
    const std::vector<RigExecWindowRing> &rings = played->GetWindowRings();
    CHECK(rings.size() == 2 && rings[0].size() == 2 && rings[1].size() == 3);
    // Frame 0 has no history: A is its input plus one, weighted 1.
    CHECK(Same(reference[0][0], Vec3{1.0f, 1.0f, 2.0f}));

    // A jump reaches every tick a played frame read.
    auto jumped = RigExecEvaluator::Create(rig.program);
    for (const int frame : {5, 7, 9})
        CHECK(SameAll(Jump(rig, *jumped, frame), reference[size_t(frame)]));

    // A history tick leaves the outputs as the last Evaluate left them.
    const std::vector<Vec3> before = Outputs(rig, *jumped);
    SetFrame(rig, *jumped, 3);
    CHECK(jumped->EvaluateHistory());
    CHECK(SameAll(Outputs(rig, *jumped), before));

    // A declined tick leaves a null capture: A then reads only the tick
    // after it.
    std::string error;
    jumped->ClearHistory();
    for (int k = 5; k >= 1; --k) {
        SetFrame(rig, *jumped, 9 - k);
        if (k == 2) {
            const Vec3 negative{-5.0f, 0.0f, 0.0f};
            jumped->SetInput(rig.inputs[0], &negative);
            CHECK(!jumped->EvaluateHistory(&error));
            CHECK(Contains(error, "declined"));
            CHECK(!jumped->GetWindowRings()[0].back());
            CHECK(!jumped->GetWindowRings()[1].back());
        } else {
            CHECK(jumped->EvaluateHistory());
        }
    }
    SetFrame(rig, *jumped, 9);
    CHECK(jumped->Evaluate());
    const Vec3 a8 = FrameValue(8, 0), a9 = FrameValue(9, 0);
    const Vec3 in8{a8.x + 1.0f, a8.y + 1.0f, a8.z + 1.0f};
    const Vec3 in9{a9.x + 1.0f, a9.y + 1.0f, a9.z + 1.0f};
    Vec3 expected{0.0f, 0.0f, 0.0f};
    expected = Vec3{expected.x + 1.0f * in8.x, expected.y + 1.0f * in8.y,
                    expected.z + 1.0f * in8.z};
    expected = Vec3{expected.x + 2.0f * in9.x, expected.y + 2.0f * in9.y,
                    expected.z + 2.0f * in9.z};
    CHECK(Same(Outputs(rig, *jumped)[0], expected));

    // A failed run captures nothing.
    const Vec3 negative{-5.0f, 0.0f, 0.0f};
    jumped->SetInput(rig.inputs[0], &negative);
    CHECK(!jumped->Evaluate());
    const std::vector<RigExecWindowPacketPtr> none =
        jumped->GetWindowCaptures();
    CHECK(none.size() == 2 && !none[0] && !none[1]);

    // A swap resets the history.
    CHECK(played->SwapProgram(rig.program));
    CHECK(played->GetWindowRings().size() == 2 &&
          played->GetWindowRings()[0].empty() &&
          played->GetWindowRings()[1].empty());
    const std::vector<RigExecWindowPacketPtr> fresh =
        played->GetWindowCaptures();
    CHECK(fresh.size() == 2 && !fresh[0] && !fresh[1]);
    SetFrame(rig, *played, 0);
    CHECK(played->Evaluate());
    CHECK(SameAll(Outputs(rig, *played), reference[0]));
}

// A host that keeps the rings itself gets the bits of one that pushes.
void
TestWindowRingRoundTrip()
{
    const WindowRig rig = BuildWindowRig(1, 0);
    auto played = RigExecEvaluator::Create(rig.program);
    const std::vector<std::vector<Vec3>> reference = Play(rig, *played, 8);
    auto hosted = RigExecEvaluator::Create(rig.program);
    std::vector<RigExecWindowRing> rings(2);
    const size_t frames[] = {2, 3};
    for (int f = 0; f < 8; ++f) {
        CHECK(hosted->SetWindowRings(rings));
        SetFrame(rig, *hosted, f);
        CHECK(hosted->Evaluate());
        CHECK(SameAll(Outputs(rig, *hosted), reference[size_t(f)]));
        const std::vector<RigExecWindowPacketPtr> captures =
            hosted->GetWindowCaptures();
        CHECK(captures.size() == 2 && captures[0] && captures[1]);
        for (size_t w = 0; w < 2 && w < captures.size(); ++w) {
            rings[w].push_back(captures[w]);
            if (rings[w].size() > frames[w])
                rings[w].erase(rings[w].begin());
        }
    }
    CHECK(RigExecWindowRingsEqual(rings[0], played->GetWindowRings()[0]));
    CHECK(RigExecWindowRingsEqual(rings[1], played->GetWindowRings()[1]));

    std::string error;
    CHECK(!hosted->SetWindowRings({RigExecWindowRing()}, &error));
    CHECK(Contains(error, "1 rings for 2 windows"));
    // A long ring keeps its newest frames.
    RigExecWindowRing longRing(rings[1]);
    longRing.insert(longRing.begin(), RigExecWindowPacketPtr());
    CHECK(hosted->SetWindowRings({longRing, longRing}));
    CHECK(hosted->GetWindowRings()[0].size() == 2);
    CHECK(RigExecWindowRingsEqual(hosted->GetWindowRings()[1], rings[1]));
}

void
TestWindowDeterminism()
{
    const size_t chains = 23;
    const auto run = [&](RigExecTaskSystem *tasks, size_t batch) {
        const WindowRig rig = BuildWindowRig(chains, batch);
        RigExecEvaluatorOptions options;
        options.tasks = tasks;
        auto evaluator = RigExecEvaluator::Create(rig.program, options);
        std::vector<Vec3> all;
        for (const auto &frame : Play(rig, *evaluator, 7))
            all.insert(all.end(), frame.begin(), frame.end());
        const std::vector<Vec3> jumped = Jump(rig, *evaluator, 11);
        all.insert(all.end(), jumped.begin(), jumped.end());
        return all;
    };
    const std::vector<Vec3> reference = run(nullptr, 0);
    CHECK(reference.size() == chains * 4 * 8);
    for (const size_t threads : {size_t(1), size_t(2), size_t(4)}) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch :
             {size_t(0), size_t(1), size_t(2), size_t(7)}) {
            const bool same = SameAll(run(tasks.get(), batch), reference);
            CHECK(same);
            if (!same)
                std::printf("windows: threads %zu batch %zu differ\n", threads,
                            batch);
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
    TestWindowLayout();
    TestWindowHistory();
    TestWindowRingRoundTrip();
    TestWindowDeterminism();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
