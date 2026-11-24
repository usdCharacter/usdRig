// The op core: build a rig description by hand, compile, evaluate,
// read outputs and taps, and check results are bit-identical across
// thread counts and batch sizes.
#include "registryFixture/fakeOps.h"
#include "registryFixture/fakeTypes.h"
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/opFields.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExec/api/runState.h"
#include "rigExec/api/view.h"
#include "rigExec/internal/program/program.h"
#include "rigExecGraph/windowHistory.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

using namespace rigExec;
using fakeTypes::Vec3;

namespace {
std::atomic<bool> gCountAllocations{false};
std::atomic<size_t> gAllocations{0};

void *
CountedAllocate(size_t size)
{
    if (gCountAllocations.load(std::memory_order_relaxed))
        gAllocations.fetch_add(1, std::memory_order_relaxed);
    if (void *p = std::malloc(size ? size : 1))
        return p;
    throw std::bad_alloc();
}
} // namespace

void *
operator new(size_t size)
{
    return CountedAllocate(size);
}

void *
operator new[](size_t size)
{
    return CountedAllocate(size);
}

void
operator delete(void *p) noexcept
{
    std::free(p);
}

void
operator delete[](void *p) noexcept
{
    std::free(p);
}

void
operator delete(void *p, size_t) noexcept
{
    std::free(p);
}

void
operator delete[](void *p, size_t) noexcept
{
    std::free(p);
}

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

std::atomic<uint32_t> checkRuns{0};

bool
RunCheck(const void *, void *, const RigExecOpIo &io, std::string *error)
{
    checkRuns.fetch_add(1);
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

// out = in + payload.add; records each payload it is handed.
RIGEXEC_DEFINE_OP_HANDLE(Probe)
RIGEXEC_DEFINE_PORT_HANDLE(ProbeIn)
RIGEXEC_DEFINE_PORT_HANDLE(ProbeOut)

struct ProbePayload {
    float add = 0.0f;
};

std::vector<const void *> probePayloads;

bool
RunProbe(const void *payload, void *, const RigExecOpIo &io, std::string *)
{
    probePayloads.push_back(payload);
    const float add = static_cast<const ProbePayload *>(payload)->add;
    const Vec3 &in = *static_cast<const Vec3 *>(io.inputs[0]);
    *static_cast<Vec3 *>(io.outputs[1]) = Vec3{in.x + add, in.y, in.z};
    return true;
}

// out = in; counts its runs per payload id.
RIGEXEC_DEFINE_OP_HANDLE(Mark)
RIGEXEC_DEFINE_PORT_HANDLE(MarkIn)
RIGEXEC_DEFINE_PORT_HANDLE(MarkOut)

struct MarkPayload {
    uint32_t id = 0;
};

std::atomic<uint32_t> markRuns[16];

bool
RunMark(const void *payload, void *, const RigExecOpIo &io, std::string *)
{
    markRuns[static_cast<const MarkPayload *>(payload)->id].fetch_add(1);
    *static_cast<Vec3 *>(io.outputs[1]) =
        *static_cast<const Vec3 *>(io.inputs[0]);
    return true;
}

// A batch function that calls Run on each active instance.
template <bool (*Run)(const void *, void *, const RigExecOpIo &, std::string *)>
bool
BatchOf(const RigExecOpBatch &batch, size_t *failed, std::string *error)
{
    for (size_t k = 0; k < batch.count; ++k) {
        if (!RigExecBatchActive(batch, k))
            continue;
        if (!Run(RigExecBatchPayload(batch, k), RigExecBatchState(batch, k),
                 RigExecBatchInstance(batch, k), error)) {
            *failed = k;
            return false;
        }
    }
    return true;
}

// A batch-only op that copies in to out and counts its calls and the
// instances it ran.
RIGEXEC_DEFINE_OP_HANDLE(Tally)
RIGEXEC_DEFINE_PORT_HANDLE(TallyIn)
RIGEXEC_DEFINE_PORT_HANDLE(TallyOut)

std::atomic<uint32_t> tallyCalls{0};
std::atomic<uint32_t> tallyRuns{0};

bool
RunTally(const RigExecOpBatch &batch, size_t *, std::string *)
{
    tallyCalls.fetch_add(1);
    for (size_t k = 0; k < batch.count; ++k) {
        if (!RigExecBatchActive(batch, k))
            continue;
        tallyRuns.fetch_add(1);
        const RigExecOpIo io = RigExecBatchInstance(batch, k);
        *static_cast<Vec3 *>(io.outputs[TallyOut().Index()]) =
            *static_cast<const Vec3 *>(io.inputs[TallyIn().Index()]);
    }
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
    static const RigExecOpDesc weighted = [&] {
        RigExecOpDesc d = op("Weighted", weightedPorts, nullptr);
        d.runBatch = &BatchOf<&RunWeighted>;
        return d;
    }();
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
        d.runBatch = &BatchOf<&RunRingSum>;
        d.threading = RigExecOpThreading::Windowed;
        d.windowFrames = &RingFrames;
        d.payload = RigExecPayloadHandlingFor<RingPayload>();
        return d;
    }();
    registrar.Add(RingSum(), ring);

    static const RigExecPortDesc tallyPorts[] = {
        {&TallyIn(), &fakeTypes::Vec3Type(), kRigExecPortInput, "in"},
        {&TallyOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecOpDesc tally = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Tally";
        d.ports = tallyPorts;
        d.portCount = 2;
        d.runBatch = &RunTally;
        return d;
    }();
    registrar.Add(Tally(), tally);

    static const RigExecPortDesc probePorts[] = {
        {&ProbeIn(), &fakeTypes::Vec3Type(), kRigExecPortInput, "in"},
        {&ProbeOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecOpDesc probe = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Probe";
        d.ports = probePorts;
        d.portCount = 2;
        d.run = &RunProbe;
        d.payload = RigExecPayloadHandlingFor<ProbePayload>();
        return d;
    }();
    registrar.Add(Probe(), probe);

    static const RigExecPortDesc markPorts[] = {
        {&MarkIn(), &fakeTypes::Vec3Type(), kRigExecPortInput, "in"},
        {&MarkOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecOpDesc mark = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Mark";
        d.ports = markPorts;
        d.portCount = 2;
        d.run = &RunMark;
        d.payload = RigExecPayloadHandlingFor<MarkPayload>();
        return d;
    }();
    registrar.Add(Mark(), mark);
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
    std::vector<const RigExecTypeHandle *> types;
};

void
RecordTap(void *context, size_t, const RigExecTapValue &value)
{
    auto *log = static_cast<TapLog *>(context);
    log->values.push_back(*static_cast<const Vec3 *>(value.data));
    log->types.push_back(value.type);
}

// Every value has one place: host inputs in the input arena, bound outputs
// in published arenas of their own, the rest in the work arena; settings
// live in the program's constant arena, which two evaluators share.
void
TestStoreLayout()
{
    RigExecRigDescBuilder builder;
    const ProbePayload one{1.0f};
    const ProbePayload two{2.0f};
    const RigExecNodeId first = builder.AddOp(Probe(), &one);
    const RigExecNodeId check = builder.AddOp(Check());
    const RigExecNodeId last = builder.AddOp(Probe(), &two);
    builder.Connect(first, ProbeOut(), check, CheckIn());
    builder.Connect(check, CheckOut(), last, ProbeIn());
    const RigExecInputId in = builder.BindInput(first, ProbeIn());
    const RigExecOutputId middle = builder.BindOutput(check, CheckOut());
    const RigExecOutputId out = builder.BindOutput(last, ProbeOut());
    builder.BindOutput(last, ProbeOut());
    std::shared_ptr<const RigExecProgram> program = Compile(builder);
    CHECK(program);
    if (!program)
        return;
    const RigExecProgram &p = *program;
    CHECK(p.publishedSlots.size() == 2);
    std::vector<std::pair<size_t, size_t>> work;
    std::vector<std::pair<size_t, size_t>> input;
    for (uint32_t slot = 0; slot < p.slots.size(); ++slot) {
        const RigExecProgramSlot &place = p.slots[slot];
        const bool bound =
            std::count(p.outputSlots.begin(), p.outputSlots.end(), slot) != 0;
        const bool host =
            std::count(p.inputSlots.begin(), p.inputSlots.end(), slot) != 0;
        CHECK(!(bound && host));
        CHECK((place.arena == RigExecArenaKind::Published) == bound);
        CHECK((place.arena == RigExecArenaKind::Input) == host);
        if (bound)
            CHECK(p.publishedSlots[place.arenaIndex] == slot);
        if (place.arena == RigExecArenaKind::Work)
            work.emplace_back(place.offset, place.offset + sizeof(Vec3));
        if (host)
            input.emplace_back(place.offset, place.offset + sizeof(Vec3));
    }
    CHECK(work.size() == 1 && input.size() == 1);
    for (const auto &[begin, end] : work)
        CHECK(end <= p.stateBase && p.stateBase <= p.workSize);
    for (const auto &[begin, end] : input)
        CHECK(end <= p.inputArenaSize);

    RigExecValueRef ref;
    CHECK(RigExecResolveInput(p, in, &ref) &&
          ref.arena == RigExecArena::Input && ref.index == 0 &&
          ref.count == 1 && ref.type == &fakeTypes::Vec3Type());
    CHECK(RigExecResolveOutput(p, out, &ref) &&
          ref.arena == RigExecArena::Published);
    CHECK(RigExecResolveNodeOutput(p, first, ProbeOut(), &ref) &&
          ref.arena == RigExecArena::Work);
    CHECK(!RigExecResolveNodeOutput(p, first, CheckOut(), &ref));
    CHECK(!RigExecResolveOutput(p, RigExecOutputId{9}, &ref));

    auto a = RigExecEvaluator::Create(program);
    auto b = RigExecEvaluator::Create(program);
    const Vec3 x{1.0f, 2.0f, 3.0f};
    CHECK(a->SetInput(in, &x) && b->SetInput(in, &x));
    probePayloads.clear();
    CHECK(a->Evaluate() && b->Evaluate());
    CHECK(probePayloads.size() == 4);
    if (probePayloads.size() == 4) {
        CHECK(probePayloads[0] == probePayloads[2]);
        CHECK(probePayloads[1] == probePayloads[3]);
        for (const void *payload : probePayloads)
            CHECK(payload >= p.payloads.Data() &&
                  payload < p.payloads.Data() + p.payloads.Size());
    }
    CHECK(a->GetOutput(out) != b->GetOutput(out));
    CHECK(a->GetOutput(middle) != b->GetOutput(middle));
    CHECK(Same(*static_cast<const Vec3 *>(a->GetOutput(out)),
               Vec3{4.0f, 2.0f, 3.0f}));
    CHECK(&a->GetProgram() == &p);
}

// Edits reach the next evaluation whole, never part of one, and only once
// committed; one edit is open at a time.
void
TestInputWriter()
{
    RigExecRigDescBuilder builder;
    const RigExecNodeId sum = builder.AddOp(Weighted());
    const RigExecInputId a = builder.BindInput(sum, WeightedItems());
    const RigExecInputId b = builder.BindInput(sum, WeightedItems());
    const RigExecOutputId out = builder.BindOutput(sum, WeightedOut());
    std::shared_ptr<const RigExecProgram> program = Compile(builder);
    CHECK(program);
    if (!program)
        return;
    auto evaluator = RigExecEvaluator::Create(program);
    const auto value = [&] {
        return *static_cast<const Vec3 *>(evaluator->GetOutput(out));
    };
    {
        RigExecInputWriter writer = evaluator->BeginInputEdit();
        CHECK(writer.IsOpen());
        CHECK(!evaluator->BeginInputEdit().IsOpen());
        const Vec3 one{1.0f, 1.0f, 1.0f};
        CHECK(writer.Set(a, &one) && writer.Set(b, &one));
        CHECK(!writer.Set(RigExecInputId{7}, &one));
        std::string error;
        CHECK(!evaluator->SetInput(a, &one, &error));
        CHECK(Contains(error, "another edit is open"));
        CHECK(evaluator->Evaluate());
        CHECK(Same(value(), Vec3{0.0f, 0.0f, 0.0f}));
        writer.Commit();
        CHECK(!writer.IsOpen());
        CHECK(evaluator->Evaluate());
        CHECK(Same(value(), Vec3{3.0f, 3.0f, 3.0f}));
    }
    {
        RigExecInputWriter discarded = evaluator->BeginInputEdit();
        const Vec3 big{100.0f, 100.0f, 100.0f};
        CHECK(discarded.Set(a, &big));
    }
    CHECK(evaluator->Evaluate());
    CHECK(Same(value(), Vec3{3.0f, 3.0f, 3.0f}));
    {
        RigExecInputWriter writer = evaluator->BeginInputEdit();
        Vec3 *edit = writer.Edit<Vec3>(b, fakeTypes::Vec3Type());
        CHECK(edit && Same(*edit, Vec3{1.0f, 1.0f, 1.0f}));
        CHECK(!writer.Edit<float>(b, fakeTypes::Float()));
        if (edit)
            edit->x = 2.0f;
        writer.Commit();
    }
    CHECK(evaluator->Evaluate());
    CHECK(Same(value(), Vec3{5.0f, 3.0f, 3.0f}));

    // One thread commits a = (i, i, 0), b = (0, 0, i) as one edit while
    // this one evaluates: every result is (i, i, 2i) for one i.
    {
        RigExecInputWriter writer = evaluator->BeginInputEdit();
        const Vec3 zero{0.0f, 0.0f, 0.0f};
        writer.Set(a, &zero);
        writer.Set(b, &zero);
        writer.Commit();
    }
    constexpr int kEdits = 4000;
    std::atomic<bool> done{false};
    std::thread writerThread([&] {
        for (int i = 1; i <= kEdits; ++i) {
            RigExecInputWriter writer = evaluator->BeginInputEdit();
            const Vec3 va{float(i), float(i), 0.0f};
            const Vec3 vb{0.0f, 0.0f, float(i)};
            writer.Set(a, &va);
            writer.Set(b, &vb);
            writer.Commit();
        }
        done.store(true);
    });
    float last = 0.0f;
    int torn = 0;
    int backwards = 0;
    for (bool finished = false; !finished;) {
        finished = done.load();
        CHECK(evaluator->Evaluate());
        const Vec3 v = value();
        if (v.x != v.y || v.z != 2.0f * v.x)
            ++torn;
        if (v.x < last)
            ++backwards;
        last = v.x;
    }
    writerThread.join();
    CHECK(torn == 0);
    CHECK(backwards == 0);
    CHECK(last == float(kEdits));
}

// A view keeps the value it lends unchanged while newer frames publish;
// an output nobody holds is written in place.
void
TestPins()
{
    RigExecRigDescBuilder builder;
    const ProbePayload ten{10.0f};
    const RigExecNodeId check = builder.AddOp(Check());
    const RigExecNodeId probe = builder.AddOp(Probe(), &ten);
    builder.Connect(check, CheckOut(), probe, ProbeIn());
    const RigExecInputId in = builder.BindInput(check, CheckIn());
    const RigExecOutputId out = builder.BindOutput(check, CheckOut());
    const RigExecOutputId after = builder.BindOutput(probe, ProbeOut());
    std::shared_ptr<const RigExecProgram> program = Compile(builder);
    CHECK(program);
    if (!program)
        return;
    auto evaluator = RigExecEvaluator::Create(program);
    const auto frame = [&](float x) {
        const Vec3 v{x, 0.0f, 0.0f};
        CHECK(evaluator->SetInput(in, &v));
        CHECK(evaluator->Evaluate());
        CHECK(static_cast<const Vec3 *>(evaluator->GetOutput(after))->x ==
              x + 10.0f);
        return evaluator->GetOutput(out);
    };
    const auto x = [](const RigExecValueView &view) {
        const Vec3 *v = view.As<Vec3>(fakeTypes::Vec3Type());
        return v ? v->x : -1.0f;
    };
    const void *first = frame(1.0f);
    CHECK(frame(2.0f) == first);
    RigExecValueView two = evaluator->ViewOutput(out);
    CHECK(two && two.Data() == first && x(two) == 2.0f &&
          two.Generation() == 2);
    CHECK(!two.As<float>(fakeTypes::Float()));
    CHECK(!evaluator->ViewOutput(RigExecOutputId{5}));
    const void *third = frame(3.0f);
    CHECK(third != first && x(two) == 2.0f);
    RigExecValueView three = evaluator->ViewOutput(out);
    RigExecValueView copy = two;
    CHECK(x(copy) == 2.0f && three.Generation() == 3);
    const void *fourth = frame(4.0f);
    CHECK(fourth != first && fourth != third);
    CHECK(x(two) == 2.0f && x(copy) == 2.0f && x(three) == 3.0f);
    two.Reset();
    CHECK(!two && x(copy) == 2.0f);
    copy.Reset();
    CHECK(frame(5.0f) == fourth);
    three = evaluator->ViewOutput(out);
    CHECK(frame(6.0f) == first);
    evaluator.reset();
    CHECK(x(three) == 5.0f && three.Generation() == 5);
}

// An evaluation reruns only the nodes an edit reaches through the compiled
// readers, plus the windowed node and what it feeds; an order pair alone
// reruns nothing. Results match running every node, bit for bit, and an
// output nobody reran keeps its published storage.
struct DirtyRig {
    std::shared_ptr<const RigExecProgram> program;
    RigExecInputId a, b, c, d;
    std::vector<RigExecOutputId> outs;
};

DirtyRig
BuildDirtyRig(size_t maxBatchNodes)
{
    // a -> m0 -> m1 -> out0; b -> m2; [m1, m2] -> Weighted -> out1;
    // c -> RingSum -> m3 -> out2; d -> m4 -> out3, ordered after m2;
    // m5 reads a through Check -> out4.
    RigExecRigDescBuilder builder;
    std::vector<RigExecNodeId> m;
    for (uint32_t id = 0; id < 6; ++id) {
        const MarkPayload payload{id};
        m.push_back(builder.AddOp(Mark(), &payload));
    }
    const RingPayload frames{2};
    const RigExecNodeId ring = builder.AddOp(RingSum(), &frames);
    const RigExecNodeId sum = builder.AddOp(Weighted());
    const RigExecNodeId check = builder.AddOp(Check());
    DirtyRig rig;
    rig.a = builder.BindInput(m[0], MarkIn());
    builder.Connect(m[0], MarkOut(), m[1], MarkIn());
    rig.b = builder.BindInput(m[2], MarkIn());
    builder.Connect(m[1], MarkOut(), sum, WeightedItems());
    builder.Connect(m[2], MarkOut(), sum, WeightedItems());
    rig.c = builder.BindInput(ring, RingSumIn());
    builder.Connect(ring, RingSumOut(), m[3], MarkIn());
    rig.d = builder.BindInput(m[4], MarkIn());
    builder.Order(m[2], m[4]);
    builder.Connect(m[0], MarkOut(), check, CheckIn());
    builder.Connect(check, CheckOut(), m[5], MarkIn());
    rig.outs = {builder.BindOutput(m[1], MarkOut()),
                builder.BindOutput(sum, WeightedOut()),
                builder.BindOutput(m[3], MarkOut()),
                builder.BindOutput(m[4], MarkOut()),
                builder.BindOutput(m[5], MarkOut())};
    rig.program = Compile(builder, maxBatchNodes);
    return rig;
}

std::vector<uint32_t>
TakeMarks()
{
    std::vector<uint32_t> runs;
    for (std::atomic<uint32_t> &count : markRuns)
        runs.push_back(count.exchange(0));
    runs.resize(6);
    return runs;
}

void
TestDirtyPropagation()
{
    std::unique_ptr<RigExecTaskSystem> pool = RigExecCreateThreadTaskSystem(3);
    for (const size_t maxBatchNodes : {size_t(0), size_t(1)})
        for (RigExecTaskSystem *tasks :
             {static_cast<RigExecTaskSystem *>(nullptr), pool.get()}) {
            const DirtyRig rig = BuildDirtyRig(maxBatchNodes);
            CHECK(rig.program);
            if (!rig.program)
                return;
            RigExecEvaluatorOptions options;
            options.tasks = tasks;
            auto dirty = RigExecEvaluator::Create(rig.program, options);
            auto reference = RigExecEvaluator::Create(rig.program, options);
            Vec3 values[4] = {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}, {4, 4, 4}};
            const RigExecInputId inputs[4] = {rig.a, rig.b, rig.c, rig.d};
            for (int i = 0; i < 4; ++i)
                CHECK(dirty->SetInput(inputs[i], &values[i]));
            // Each frame edits the inputs listed; the reference sets every
            // input, so every node it has reruns.
            struct Frame {
                std::vector<int> edits;
                std::vector<uint32_t> runs;
            };
            const std::vector<Frame> frames = {
                {{}, {1, 1, 1, 1, 1, 1}},     {{}, {0, 0, 0, 1, 0, 0}},
                {{0}, {1, 1, 0, 1, 0, 1}},    {{1}, {0, 0, 1, 1, 0, 0}},
                {{3}, {0, 0, 0, 1, 1, 0}},    {{2}, {0, 0, 0, 1, 0, 0}},
                {{0, 1}, {1, 1, 1, 1, 0, 1}},
            };
            RigExecValueView kept;
            const void *orderedStorage = nullptr;
            for (size_t f = 0; f < frames.size(); ++f) {
                for (const int i : frames[f].edits) {
                    values[i].x += 1.0f;
                    CHECK(dirty->SetInput(inputs[i], &values[i]));
                }
                TakeMarks();
                CHECK(dirty->Evaluate());
                CHECK(TakeMarks() == frames[f].runs);
                dirty->PushHistory();
                for (int i = 0; i < 4; ++i)
                    CHECK(reference->SetInput(inputs[i], &values[i]));
                CHECK(reference->Evaluate());
                reference->PushHistory();
                for (const RigExecOutputId out : rig.outs)
                    CHECK(Same(
                        *static_cast<const Vec3 *>(dirty->GetOutput(out)),
                        *static_cast<const Vec3 *>(reference->GetOutput(out))));
                if (f == 2) {
                    kept = dirty->ViewOutput(rig.outs[0]);
                    orderedStorage = dirty->GetOutput(rig.outs[3]);
                }
                if (f == 3 || f == 5) {
                    // out0 was not rerun: the same storage and generation.
                    const RigExecValueView now = dirty->ViewOutput(rig.outs[0]);
                    CHECK(now.Data() == kept.Data() &&
                          now.Generation() == kept.Generation());
                }
                if (f == 3)
                    CHECK(dirty->GetOutput(rig.outs[3]) == orderedStorage);
            }
            // The last frame reran out0 while a view held it: the view is
            // unchanged and the new value lives elsewhere.
            const Vec3 held = *kept.As<Vec3>(fakeTypes::Vec3Type());
            CHECK(held.x == 2.0f);
            CHECK(dirty->GetOutput(rig.outs[0]) != kept.Data());
            CHECK(static_cast<const Vec3 *>(dirty->GetOutput(rig.outs[0]))->x ==
                  values[0].x);
            CHECK(dirty->GetOutput(rig.outs[3]) == orderedStorage);

            // A failed frame makes the next one run every node.
            const Vec3 negative{-1.0f, 0.0f, 0.0f};
            CHECK(dirty->SetInput(rig.a, &negative));
            CHECK(!dirty->Evaluate());
            CHECK(dirty->SetInput(rig.a, &values[0]));
            TakeMarks();
            CHECK(dirty->Evaluate());
            CHECK((TakeMarks() == std::vector<uint32_t>{1, 1, 1, 1, 1, 1}));
        }
}

// Run states over one program: workers on their own threads, a copy that
// continues from an evaluator, and a what-if that leaves it untouched.
void
TestRunStates()
{
    const DirtyRig rig = BuildDirtyRig(0);
    CHECK(rig.program);
    if (!rig.program)
        return;
    const RigExecInputId inputs[4] = {rig.a, rig.b, rig.c, rig.d};
    const auto valueOf = [](int worker, int frame, int input) {
        const float v = float(worker * 100 + frame * 4 + input + 1);
        return Vec3{v, v + 0.5f, v + 0.25f};
    };
    constexpr int kWorkers = 4;
    constexpr int kFrames = 12;
    std::vector<std::vector<Vec3>> expected(kWorkers);
    auto reference = RigExecEvaluator::Create(rig.program);
    for (int w = 0; w < kWorkers; ++w)
        for (int f = 0; f < kFrames; ++f) {
            for (int i = 0; i < 4; ++i) {
                const Vec3 v = valueOf(w, f, i);
                CHECK(reference->SetInput(inputs[i], &v));
            }
            CHECK(reference->Evaluate());
            for (const RigExecOutputId out : rig.outs)
                expected[w].push_back(
                    *static_cast<const Vec3 *>(reference->GetOutput(out)));
        }
    std::unique_ptr<RigExecTaskSystem> pool = RigExecCreateThreadTaskSystem(3);
    std::vector<std::vector<Vec3>> got(kWorkers);
    std::vector<std::thread> threads;
    for (int w = 0; w < kWorkers; ++w)
        threads.emplace_back([&, w] {
            RigExecEvaluatorOptions options;
            options.tasks = w % 2 ? pool.get() : nullptr;
            auto state = RigExecRunState::Create(rig.program, options);
            for (int f = 0; f < kFrames; ++f) {
                RigExecInputWriter writer = state->BeginInputEdit();
                for (int i = 0; i < 4; ++i) {
                    const Vec3 v = valueOf(w, f, i);
                    writer.Set(inputs[i], &v);
                }
                writer.Commit();
                if (!state->Evaluate())
                    return;
                for (const RigExecOutputId out : rig.outs)
                    got[w].push_back(
                        *static_cast<const Vec3 *>(state->GetOutput(out)));
            }
        });
    for (std::thread &thread : threads)
        thread.join();
    for (int w = 0; w < kWorkers; ++w) {
        CHECK(got[w].size() == expected[w].size());
        for (size_t k = 0; k < got[w].size() && k < expected[w].size(); ++k)
            CHECK(Same(got[w][k], expected[w][k]));
    }

    // A copy continues from the evaluator's inputs and history.
    auto evaluator = RigExecEvaluator::Create(rig.program);
    for (int f = 0; f < 3; ++f) {
        for (int i = 0; i < 4; ++i) {
            const Vec3 v = valueOf(0, f, i);
            CHECK(evaluator->SetInput(inputs[i], &v));
        }
        CHECK(evaluator->Evaluate());
        evaluator->PushHistory();
    }
    std::unique_ptr<RigExecRunState> copy = evaluator->CreateRunState();
    CHECK(&copy->GetProgram() == &evaluator->GetProgram());
    for (const RigExecOutputId out : rig.outs)
        CHECK(Same(*static_cast<const Vec3 *>(copy->GetOutput(out)),
                   *static_cast<const Vec3 *>(evaluator->GetOutput(out))));
    CHECK(copy->Evaluate() && evaluator->Evaluate());
    for (const RigExecOutputId out : rig.outs)
        CHECK(Same(*static_cast<const Vec3 *>(copy->GetOutput(out)),
                   *static_cast<const Vec3 *>(evaluator->GetOutput(out))));
    std::vector<Vec3> before;
    for (const RigExecOutputId out : rig.outs)
        before.push_back(*static_cast<const Vec3 *>(evaluator->GetOutput(out)));
    {
        RigExecInputWriter writer = copy->BeginInputEdit();
        const Vec3 what{50.0f, 50.0f, 50.0f};
        writer.Set(rig.a, &what);
        writer.Commit();
    }
    CHECK(copy->Evaluate());
    CHECK(static_cast<const Vec3 *>(copy->GetOutput(rig.outs[0]))->x == 50.0f);
    for (size_t k = 0; k < rig.outs.size(); ++k)
        CHECK(
            Same(*static_cast<const Vec3 *>(evaluator->GetOutput(rig.outs[k])),
                 before[k]));
    std::unique_ptr<RigExecRunState> twin = copy->Clone();
    copy->ReleaseOwner();
    std::thread([&] { CHECK(copy->Evaluate()); }).join();
    copy->ReleaseOwner();
    CHECK(twin->Evaluate());
    for (const RigExecOutputId out : rig.outs)
        CHECK(Same(*static_cast<const Vec3 *>(twin->GetOutput(out)),
                   *static_cast<const Vec3 *>(copy->GetOutput(out))));
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
    CHECK(log.types.size() == 1 && log.types[0] == &fakeTypes::Vec3Type());

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
        CHECK(Contains(error, "order cycle: node 0 (fake:Offset) -> "
                              "node 1 (fake:Offset) -> node 0 (fake:Offset)"));
    }
    {
        // A data edge one way and an order pair the other.
        RigExecRigDescBuilder builder;
        const RigExecNodeId a = builder.AddOp(fakeOps::Offset());
        const RigExecNodeId b = builder.AddOp(Check());
        builder.BindInput(a, fakeOps::OffsetIn());
        builder.Connect(a, fakeOps::OffsetOut(), b, CheckIn());
        builder.Order(b, a);
        CHECK(!Compile(builder, 0, &error));
        CHECK(Contains(error, "order cycle: ") &&
              Contains(error, "node 0 (fake:Offset)") &&
              Contains(error, "node 1 (test:Check)"));
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

const size_t kBatchSizes[] = {0, 1, 2, 3, 7};

// 1, 2 and 3 threads and every hardware thread.
std::vector<size_t>
ThreadCounts()
{
    return {1, 2, 3, std::max<size_t>(1, std::thread::hardware_concurrency())};
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
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch : kBatchSizes) {
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
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch : kBatchSizes) {
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
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch : kBatchSizes) {
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

// Plays 7 frames of a window rig, then jumps to frame 11.
std::vector<Vec3>
RunWindowRig(size_t chains, RigExecTaskSystem *tasks, size_t batch)
{
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
}

void
TestWindowDeterminism()
{
    const size_t chains = 23;
    const auto run = [&](RigExecTaskSystem *tasks, size_t batch) {
        return RunWindowRig(chains, tasks, batch);
    };
    const std::vector<Vec3> reference = run(nullptr, 0);
    CHECK(reference.size() == chains * 4 * 8);
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t batch : kBatchSizes) {
            const bool same = SameAll(run(tasks.get(), batch), reference);
            CHECK(same);
            if (!same)
                std::printf("windows: threads %zu batch %zu differ\n", threads,
                            batch);
        }
    }
}

// Three rig descriptions compiled and evaluated side by side on their own
// threads, sharing one task system; each matches its single-threaded run.
void
TestSharedTaskSystem()
{
    std::string error;
    const std::vector<Vec3> wideRef =
        RunWide(BuildWide(53, 0), nullptr, 40, &error);
    const std::vector<Vec3> listRef =
        RunWide(BuildWideLists(37, 0), nullptr, 40, &error);
    const std::vector<Vec3> windowRef = RunWindowRig(19, nullptr, 0);
    CHECK(wideRef.size() == 53 * 40 && listRef.size() == 37 * 40);
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (int round = 0; round < 4; ++round) {
            std::vector<Vec3> wideGot, listGot, windowGot;
            std::string wideError, listError;
            std::thread wide([&] {
                wideGot =
                    RunWide(BuildWide(53, 3), tasks.get(), 40, &wideError);
            });
            std::thread lists([&] {
                listGot =
                    RunWide(BuildWideLists(37, 2), tasks.get(), 40, &listError);
            });
            std::thread window(
                [&] { windowGot = RunWindowRig(19, tasks.get(), 1); });
            wide.join();
            lists.join();
            window.join();
            CHECK(SameAll(wideGot, wideRef));
            CHECK(SameAll(listGot, listRef));
            CHECK(SameAll(windowGot, windowRef));
        }
    }
}

// Thousands of tiny frames, from one thread and from two at once, so a
// lost wakeup hangs or a torn level shows as a wrong value.
void
TestTaskSystemStress()
{
    const int frames = 3000;
    std::string error;
    const std::vector<Vec3> reference =
        RunWide(BuildWide(4, 1), nullptr, frames, &error);
    CHECK(reference.size() == 4 * size_t(frames));
    auto tasks = RigExecCreateThreadTaskSystem(
        std::max<size_t>(2, std::thread::hardware_concurrency()));
    const Wide wide = BuildWide(4, 1);
    CHECK(SameAll(RunWide(wide, tasks.get(), frames, &error), reference));
    std::vector<Vec3> first, second;
    std::string firstError, secondError;
    std::thread a(
        [&] { first = RunWide(wide, tasks.get(), frames, &firstError); });
    std::thread b(
        [&] { second = RunWide(wide, tasks.get(), frames, &secondError); });
    a.join();
    b.join();
    CHECK(SameAll(first, reference));
    CHECK(SameAll(second, reference));
}

// Pools made and destroyed over and over, idle or after a frame.
void
TestTaskSystemLifetimes()
{
    std::string error;
    const Wide wide = BuildWide(6, 1);
    const std::vector<Vec3> reference = RunWide(wide, nullptr, 1, &error);
    for (size_t i = 0; i < 200; ++i) {
        auto tasks = RigExecCreateThreadTaskSystem(1 + i % 6);
        if (i % 2)
            CHECK(SameAll(RunWide(wide, tasks.get(), 1, &error), reference));
    }
}

struct ScheduleProbe {
    static constexpr uint32_t kLevels = 40;
    static constexpr uint32_t kShared = 9;
    std::vector<uint32_t> sharedStart, shared, callerStart, caller;
    /// Per level: tasks finished, and tasks that started before the level
    /// below had finished.
    std::unique_ptr<std::atomic<uint32_t>[]> done;
    std::atomic<uint32_t> early{0};
    std::atomic<uint32_t> foreignCaller{0};
    std::atomic<uint32_t> ran{0};
    std::thread::id owner;
    uint32_t failAt = UINT32_MAX;

    ScheduleProbe() : done(std::make_unique<std::atomic<uint32_t>[]>(kLevels))
    {
        sharedStart.push_back(0);
        callerStart.push_back(0);
        for (uint32_t level = 0; level < kLevels; ++level) {
            done[level].store(0);
            for (uint32_t k = 0; k < kShared; ++k)
                shared.push_back(level * 100 + k);
            caller.push_back(level * 100 + 99);
            sharedStart.push_back(uint32_t(shared.size()));
            callerStart.push_back(uint32_t(caller.size()));
        }
    }

    static bool Run(void *context, uint32_t task)
    {
        auto *self = static_cast<ScheduleProbe *>(context);
        const uint32_t level = task / 100;
        if (level > 0 && self->done[level - 1].load() != kShared + 1)
            self->early.fetch_add(1);
        if (task % 100 == 99 && std::this_thread::get_id() != self->owner)
            self->foreignCaller.fetch_add(1);
        self->ran.fetch_add(1);
        self->done[level].fetch_add(1);
        return task != self->failAt;
    }

    RigExecTaskSchedule Schedule()
    {
        RigExecTaskSchedule s;
        s.levelCount = kLevels;
        s.sharedStart = sharedStart.data();
        s.shared = shared.data();
        s.callerStart = callerStart.data();
        s.caller = caller.data();
        s.run = &ScheduleProbe::Run;
        s.context = this;
        return s;
    }
};

// Levels run in order, caller tasks stay on the calling thread, and a
// failure stops the schedule after its level.
void
TestScheduleOrder()
{
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (int round = 0; round < 20; ++round) {
            ScheduleProbe probe;
            probe.owner = std::this_thread::get_id();
            CHECK(tasks->Run(probe.Schedule()));
            CHECK(probe.early.load() == 0);
            CHECK(probe.foreignCaller.load() == 0);
            CHECK(probe.ran.load() ==
                  ScheduleProbe::kLevels * (ScheduleProbe::kShared + 1));
        }
        ScheduleProbe failing;
        failing.owner = std::this_thread::get_id();
        failing.failAt = 7 * 100 + 3;
        CHECK(!tasks->Run(failing.Schedule()));
        CHECK(failing.ran.load() == 8 * (ScheduleProbe::kShared + 1));
    }
}

// One call per batch for a batch-only op, one run per node for a run-only
// op, at every batch size and thread count.
void
TestBatchCalls()
{
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t limit : kBatchSizes) {
            RigExecRigDescBuilder builder;
            std::vector<RigExecInputId> inputs;
            std::vector<RigExecOutputId> outputs;
            std::vector<RigExecNodeId> first;
            for (size_t i = 0; i < 10; ++i) {
                const RigExecNodeId node = builder.AddOp(Tally());
                first.push_back(node);
                inputs.push_back(builder.BindInput(node, TallyIn()));
                outputs.push_back(builder.BindOutput(node, TallyOut()));
            }
            for (size_t i = 0; i < 4; ++i) {
                const RigExecNodeId node = builder.AddOp(Tally());
                const RigExecNodeId check = builder.AddOp(Check());
                builder.Connect(first[i], TallyOut(), node, TallyIn());
                builder.Connect(node, TallyOut(), check, CheckIn());
                outputs.push_back(builder.BindOutput(check, CheckOut()));
            }
            std::string error;
            auto program = Compile(builder, limit, &error);
            CHECK(program);
            if (!program)
                continue;
            const auto per = [&](size_t nodes) {
                return limit ? (nodes + limit - 1) / limit : size_t(1);
            };
            size_t tallyBatches = 0;
            for (const RigExecProgramBatch &batch : program->batches)
                tallyBatches += batch.op == Tally().Index();
            CHECK(tallyBatches == per(10) + per(4));
            CHECK(program->batches.size() == tallyBatches + per(4));
            RigExecEvaluatorOptions options;
            options.tasks = tasks.get();
            auto evaluator = RigExecEvaluator::Create(program, options);
            for (size_t i = 0; i < inputs.size(); ++i) {
                const Vec3 value{float(i), 1.0f, 2.0f};
                evaluator->SetInput(inputs[i], &value);
            }
            tallyCalls = 0;
            tallyRuns = 0;
            checkRuns = 0;
            CHECK(evaluator->Evaluate(&error));
            CHECK(tallyCalls.load() == tallyBatches);
            CHECK(tallyRuns.load() == 14);
            CHECK(checkRuns.load() == 4);
            for (size_t i = 0; i < outputs.size(); ++i) {
                const Vec3 expected{float(i % 10), 1.0f, 2.0f};
                const auto *out =
                    static_cast<const Vec3 *>(evaluator->GetOutput(outputs[i]));
                CHECK(out && Same(*out, expected));
            }
        }
    }
}

// A history tick hands a batch its whole instance range with the nodes no
// windowed node depends on masked off; their outputs keep their values.
void
TestBatchHistoryMask()
{
    RigExecRigDescBuilder builder;
    const RingPayload two{2};
    const RigExecNodeId feeds = builder.AddOp(Tally());
    const RigExecNodeId aside = builder.AddOp(Tally());
    const RigExecNodeId ring = builder.AddOp(RingSum(), &two);
    builder.Connect(feeds, TallyOut(), ring, RingSumIn());
    const RigExecInputId feedsIn = builder.BindInput(feeds, TallyIn());
    const RigExecInputId asideIn = builder.BindInput(aside, TallyIn());
    const RigExecOutputId asideOut = builder.BindOutput(aside, TallyOut());
    std::string error;
    auto program = Compile(builder, 0, &error);
    CHECK(program);
    if (!program)
        return;
    CHECK((program->historyMask == std::vector<char>{1, 0, 1}));
    CHECK(program->batches.size() == 2 && program->batches[0].nodeCount == 2);
    auto evaluator = RigExecEvaluator::Create(program);
    const Vec3 x{1.0f, 2.0f, 3.0f}, y{4.0f, 5.0f, 6.0f}, z{7.0f, 8.0f, 9.0f};
    CHECK(evaluator->SetInput(feedsIn, &x) && evaluator->SetInput(asideIn, &y));
    CHECK(evaluator->Evaluate(&error));
    CHECK(evaluator->SetInput(asideIn, &z));
    tallyCalls = 0;
    tallyRuns = 0;
    CHECK(evaluator->EvaluateHistory(&error));
    CHECK(tallyCalls.load() == 1);
    CHECK(tallyRuns.load() == 1);
    const auto *out = static_cast<const Vec3 *>(evaluator->GetOutput(asideOut));
    CHECK(out && Same(*out, y));
    CHECK(evaluator->GetWindowRings()[0].size() == 1 &&
          evaluator->GetWindowRings()[0][0]);
}

std::string
Key(uint32_t node)
{
    char text[16];
    std::snprintf(text, sizeof(text), "%010u", node);
    return text;
}

// The op core orders nodes as the shared dependency compiler orders the same
// edges keyed by node id: a diamond into a list, order pairs, a window.
// Batches follow that order within a level and op.
void
TestCanonicalOrder()
{
    RigExecRigDescBuilder builder;
    const RingPayload two{2};
    const RigExecNodeId sink = builder.AddOp(Weighted());
    const RigExecNodeId ring = builder.AddOp(RingSum(), &two);
    const RigExecNodeId left = builder.AddOp(fakeOps::Offset());
    const RigExecNodeId right = builder.AddOp(Check());
    const RigExecNodeId source = builder.AddOp(fakeOps::Offset());
    const RigExecNodeId early = builder.AddOp(fakeOps::Offset());
    const RigExecNodeId after = builder.AddOp(fakeOps::Offset());
    const RigExecNodeId last = builder.AddOp(Tally());
    builder.BindInput(source, fakeOps::OffsetIn());
    builder.BindInput(early, fakeOps::OffsetIn());
    builder.BindInput(last, TallyIn());
    builder.Connect(source, fakeOps::OffsetOut(), ring, RingSumIn());
    builder.Connect(source, fakeOps::OffsetOut(), left, fakeOps::OffsetIn());
    builder.Connect(source, fakeOps::OffsetOut(), right, CheckIn());
    builder.Connect(right, CheckOut(), sink, WeightedItems());
    builder.Connect(left, fakeOps::OffsetOut(), sink, WeightedItems());
    builder.BindInput(sink, WeightedItems());
    builder.Connect(ring, RingSumOut(), after, fakeOps::OffsetIn());
    builder.Order(early, source);
    builder.Order(sink, last);
    std::string error;
    auto program = Compile(builder, 0, &error);
    CHECK(program);
    if (!program)
        return;

    // Value ids: 100 + node for each node's one output, 1000 + k for the
    // host-written inputs.
    std::vector<RigExecOpDescriptor> descriptors(8);
    for (uint32_t n = 0; n < 8; ++n) {
        descriptors[n].key = Key(n);
        descriptors[n].writes = {100 + n};
    }
    descriptors[source].reads = {1000};
    descriptors[early].reads = {1001};
    descriptors[last].reads = {1002};
    descriptors[ring].reads = {100 + source};
    descriptors[left].reads = {100 + source};
    descriptors[right].reads = {100 + source};
    descriptors[sink].reads = {100 + right, 100 + left, 1003};
    descriptors[after].reads = {100 + ring};
    descriptors[source].predecessors = {early};
    descriptors[last].predecessors = {sink};
    RigExecCompiledGraph expected;
    CHECK(RigExecCompileOpGraph(descriptors, {1000, 1001, 1002, 1003},
                                RigExecCyclePolicy::Reject, &expected, &error));
    const RigExecCompiledGraph &graph = program->graph;
    CHECK(graph.canonicalIndex == expected.canonicalIndex);
    CHECK(graph.ops.size() == expected.ops.size());
    for (size_t c = 0; c < graph.ops.size() && c < expected.ops.size(); ++c) {
        CHECK(graph.ops[c].originalIndex == expected.ops[c].originalIndex);
        CHECK(graph.ops[c].predecessors == expected.ops[c].predecessors);
    }

    // The source's output slot is read by the ring, left and right.
    const RigExecProgramNode &from = program->nodes[source];
    const uint32_t slot =
        program->portSlots[from.firstPort + fakeOps::OffsetOut().Index()];
    std::vector<uint32_t> readers = {uint32_t(graph.canonicalIndex[ring]),
                                     uint32_t(graph.canonicalIndex[left]),
                                     uint32_t(graph.canonicalIndex[right])};
    std::sort(readers.begin(), readers.end());
    const auto found = graph.readers.find(slot);
    CHECK(found != graph.readers.end() && found->second == readers);

    // Levels are longest paths; within a level and op, batch order is
    // canonical order.
    std::vector<uint32_t> level(8, 0);
    for (const RigExecCompiledOp &op : graph.ops)
        for (const uint32_t p : op.predecessors)
            level[op.originalIndex] = std::max(
                level[op.originalIndex], level[graph.ops[p].originalIndex] + 1);
    CHECK(level[sink] == 3 && level[last] == 4 && level[after] == 3);
    for (size_t l = 0; l + 1 < program->levelBatches.size(); ++l)
        for (uint32_t b = program->levelBatches[l];
             b < program->levelBatches[l + 1]; ++b) {
            const RigExecProgramBatch &batch = program->batches[b];
            for (uint32_t k = 0; k < batch.nodeCount; ++k) {
                const uint32_t n = program->batchNodes[batch.firstNode + k];
                CHECK(level[n] == l);
                if (k)
                    CHECK(graph.canonicalIndex
                              [program->batchNodes[batch.firstNode + k - 1]] <
                          graph.canonicalIndex[n]);
            }
        }
    CHECK((program->historyMask == std::vector<char>{0, 1, 0, 0, 1, 1, 0, 0}));
}

// Declared fields: a test type library with an unsigned integer and a short
// float array, and an op that declares payload and state fields of every
// extent.
struct Floats8 {
    uint32_t n = 0;
    float v[8] = {};
};

RIGEXEC_DEFINE_TYPE_HANDLE(U32Type)
RIGEXEC_DEFINE_TYPE_HANDLE(Floats8Type)

const RigExecTypeDesc kU32Desc = [] {
    RigExecTypeDesc d;
    d.nameSpace = "test";
    d.name = "U32";
    d.size = sizeof(uint32_t);
    d.align = alignof(uint32_t);
    d.triviallyCopyable = true;
    return d;
}();

const RigExecTypeDesc kFloats8Desc = [] {
    RigExecTypeDesc d;
    d.nameSpace = "test";
    d.name = "Floats8";
    d.size = sizeof(Floats8);
    d.align = alignof(Floats8);
    d.triviallyCopyable = true;
    d.count = [](const void *value) {
        return size_t(static_cast<const Floats8 *>(value)->n);
    };
    return d;
}();

void
RegisterFieldTypes(RigExecTypeRegistrar &registrar)
{
    registrar.Add(U32Type(), kU32Desc);
    registrar.Add(Floats8Type(), kFloats8Desc);
}

const RigExecTypeLibraryDesc kFieldTypes = [] {
    RigExecTypeLibraryDesc d;
    d.name = "testFieldTypes";
    d.registerTypes = &RegisterFieldTypes;
    return d;
}();

RIGEXEC_DEFINE_OP_HANDLE(Fields)
RIGEXEC_DEFINE_PORT_HANDLE(FieldsIn)
RIGEXEC_DEFINE_PORT_HANDLE(FieldsOut)

enum FieldsPayload : uint32_t { kScale, kBias, kN, kWeights, kGains };
enum FieldsState : uint32_t { kRuns, kLast, kScratch, kPerItem };

std::atomic<uint32_t> fieldsRuns{0};

// One step of a Fields node, for the kernel and for the test's own model.
Vec3
FieldsStep(float scale, const float *bias, const float *weights, size_t n,
           const float *gains, const Floats8 &in, uint32_t &runs, float *last,
           float *scratch, float *perItem)
{
    float sum = bias[0] + bias[1];
    for (uint32_t i = 0; i < in.n; ++i)
        sum += scale * in.v[i] * gains[i];
    for (size_t j = 0; j < n; ++j) {
        sum += weights[j];
        scratch[j] += weights[j];
    }
    ++runs;
    last[2] = last[1];
    last[1] = last[0];
    last[0] = sum;
    float items = 0.0f;
    for (uint32_t i = 0; i < in.n; ++i) {
        perItem[i] += in.v[i];
        items += perItem[i];
    }
    float accumulated = 0.0f;
    for (size_t j = 0; j < n; ++j)
        accumulated += scratch[j];
    return Vec3{sum + last[1], float(runs), items + accumulated};
}

bool
RunFields(const RigExecOpBatch &batch, size_t *, std::string *)
{
    const float *scale = RigExecPayloadColumnOf<float>(batch, kScale);
    const float *bias = RigExecPayloadColumnOf<float>(batch, kBias);
    uint32_t *runs = RigExecStateColumnOf<uint32_t>(batch, kRuns);
    float *last = RigExecStateColumnOf<float>(batch, kLast);
    for (size_t k = 0; k < batch.count; ++k) {
        if (!RigExecBatchActive(batch, k))
            continue;
        fieldsRuns.fetch_add(1);
        const RigExecOpIo io = RigExecBatchInstance(batch, k);
        const Floats8 &in =
            *static_cast<const Floats8 *>(io.inputs[FieldsIn().Index()]);
        const auto weights = RigExecPayloadField<float>(batch, kWeights, k);
        const auto gains = RigExecPayloadField<float>(batch, kGains, k);
        const auto scratch = RigExecStateField<float>(batch, kScratch, k);
        const auto perItem = RigExecStateField<float>(batch, kPerItem, k);
        *static_cast<Vec3 *>(io.outputs[FieldsOut().Index()]) = FieldsStep(
            scale[k], bias + 2 * k, weights.data, weights.Size(), gains.data,
            in, runs[k], last + 3 * k, scratch.data, perItem.data);
    }
    return true;
}

RigExecFieldDesc
FieldOf(const char *name, const RigExecTypeHandle &type,
        RigExecFieldExtent extent = RigExecFieldExtent::Scalar)
{
    RigExecFieldDesc f;
    f.name = name;
    f.type = &type;
    f.extent = extent;
    return f;
}

const RigExecFieldDesc kFieldsPayload[] = {
    FieldOf("scale", fakeTypes::Float()),
    [] {
        RigExecFieldDesc f =
            FieldOf("bias", fakeTypes::Float(), RigExecFieldExtent::Fixed);
        f.count = 2;
        return f;
    }(),
    [] {
        RigExecFieldDesc f = FieldOf("n", U32Type());
        f.limit = 8;
        return f;
    }(),
    [] {
        RigExecFieldDesc f = FieldOf("weights", fakeTypes::Float(),
                                     RigExecFieldExtent::FromField);
        f.source = kN;
        return f;
    }(),
    [] {
        RigExecFieldDesc f =
            FieldOf("gains", fakeTypes::Float(), RigExecFieldExtent::FromInput);
        f.port = &FieldsIn();
        return f;
    }(),
};

const RigExecFieldDesc kFieldsState[] = {
    FieldOf("runs", U32Type()),
    [] {
        RigExecFieldDesc f =
            FieldOf("last", fakeTypes::Float(), RigExecFieldExtent::Fixed);
        f.count = 3;
        return f;
    }(),
    [] {
        RigExecFieldDesc f = FieldOf("scratch", fakeTypes::Float(),
                                     RigExecFieldExtent::FromField);
        f.source = kN;
        return f;
    }(),
    [] {
        RigExecFieldDesc f = FieldOf("perItem", fakeTypes::Float(),
                                     RigExecFieldExtent::FromInput);
        f.port = &FieldsIn();
        return f;
    }(),
};

void
RegisterFields(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc ports[] = {
        {&FieldsIn(), &Floats8Type(), kRigExecPortInput, "in"},
        {&FieldsOut(), &fakeTypes::Vec3Type(), kRigExecPortOutput, "out"},
    };
    static const RigExecOpDesc desc = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Fields";
        d.ports = ports;
        d.portCount = 2;
        d.runBatch = &RunFields;
        d.payloadFields = kFieldsPayload;
        d.payloadFieldCount = std::size(kFieldsPayload);
        d.payloadVersion = 3;
        d.stateFields = kFieldsState;
        d.stateFieldCount = std::size(kFieldsState);
        return d;
    }();
    registrar.Add(Fields(), desc);
}

const RigExecOpLibraryDesc kFieldsLibrary = [] {
    RigExecOpLibraryDesc d;
    d.name = "testFields";
    d.registerOps = &RegisterFields;
    return d;
}();

// One Fields node's settings and the state the test keeps for it.
struct FieldsModel {
    float scale = 0.0f;
    float bias[2] = {};
    std::vector<float> weights, gains;
    uint32_t inCount = 0;
    uint32_t runs = 0;
    float last[3] = {};
    std::vector<float> scratch, perItem;

    RigExecFieldValues Values() const
    {
        RigExecFieldValues values(std::size(kFieldsPayload));
        values.Set(kScale, scale);
        values.Set(kBias, bias, 2);
        values.Set(kN, uint32_t(weights.size()));
        values.Set(kWeights, weights);
        values.Set(kGains, gains);
        return values;
    }
    Vec3 Step(const Floats8 &in)
    {
        scratch.resize(weights.size());
        perItem.resize(in.n);
        return FieldsStep(scale, bias, weights.data(), weights.size(),
                          gains.data(), in, runs, last, scratch.data(),
                          perItem.data());
    }
};

FieldsModel
FieldsNode(size_t i)
{
    FieldsModel m;
    m.scale = 1.5f - 0.1f * float(i);
    m.bias[0] = 0.5f * float(i);
    m.bias[1] = -1.0f;
    for (size_t j = 0; j < i % 4; ++j)
        m.weights.push_back(0.25f * float(j + 1) + float(i));
    m.inCount = uint32_t(1 + i % 5);
    for (uint32_t g = 0; g < m.inCount; ++g)
        m.gains.push_back(0.5f + 0.125f * float(g));
    return m;
}

Floats8
FieldsInput(const FieldsModel &m, int frame, size_t i)
{
    Floats8 in;
    in.n = m.inCount;
    for (uint32_t g = 0; g < in.n; ++g)
        in.v[g] = 0.3f * float(frame) + float(g) - 0.2f * float(i);
    return in;
}

// Thirteen Fields nodes of every extent run as batches of each size on
// every thread count, frame after frame, and give the model's bits.
void
TestDeclaredFields()
{
    const size_t nodes = 13;
    const int frames = 4;
    std::vector<Vec3> reference;
    {
        std::vector<FieldsModel> models;
        for (size_t i = 0; i < nodes; ++i)
            models.push_back(FieldsNode(i));
        for (int f = 0; f < frames; ++f)
            for (size_t i = 0; i < nodes; ++i)
                reference.push_back(
                    models[i].Step(FieldsInput(models[i], f, i)));
    }
    for (const size_t threads : ThreadCounts()) {
        auto tasks = RigExecCreateThreadTaskSystem(threads);
        for (const size_t limit : kBatchSizes) {
            RigExecRigDescBuilder builder;
            std::vector<FieldsModel> models;
            std::vector<RigExecInputId> inputs;
            std::vector<RigExecOutputId> outputs;
            for (size_t i = 0; i < nodes; ++i) {
                models.push_back(FieldsNode(i));
                const RigExecNodeId node =
                    builder.AddOp(Fields(), models.back().Values());
                inputs.push_back(builder.BindInput(node, FieldsIn()));
                outputs.push_back(builder.BindOutput(node, FieldsOut()));
            }
            std::string error;
            auto program = Compile(builder, limit, &error);
            CHECK(program);
            if (!program) {
                std::printf("fields: %s\n", error.c_str());
                continue;
            }
            RigExecEvaluatorOptions options;
            options.tasks = tasks.get();
            auto evaluator = RigExecEvaluator::Create(program, options);
            std::vector<Vec3> got;
            for (int f = 0; f < frames; ++f) {
                for (size_t i = 0; i < nodes; ++i) {
                    const Floats8 in = FieldsInput(models[i], f, i);
                    evaluator->SetInput(inputs[i], &in);
                }
                CHECK(evaluator->Evaluate(&error));
                for (const RigExecOutputId out : outputs)
                    got.push_back(
                        *static_cast<const Vec3 *>(evaluator->GetOutput(out)));
            }
            const bool same = SameAll(got, reference);
            CHECK(same);
            if (!same)
                std::printf("fields: threads %zu batch %zu differ\n", threads,
                            limit);
        }
    }
}

// Values that do not fit their fields are refused when added, and a payload
// field sized from an input fails the node whose input disagrees.
void
TestDeclaredFieldErrors()
{
    std::string error;
    {
        RigExecRigDescBuilder builder;
        FieldsModel m = FieldsNode(2);
        RigExecFieldValues values = m.Values();
        const float three[3] = {1.0f, 2.0f, 3.0f};
        values.Set(kBias, three, 3);
        builder.AddOp(Fields(), values);
        CHECK(!builder.Build(&error));
        CHECK(Contains(error, "Fields field bias holds 3 values, not 2"));
    }
    {
        RigExecRigDescBuilder builder;
        RigExecFieldValues values = FieldsNode(3).Values();
        values.Set(kN, uint32_t(9));
        builder.AddOp(Fields(), values);
        CHECK(!builder.Build(&error));
        CHECK(Contains(error, "field n: a value is out of range"));
    }
    {
        RigExecRigDescBuilder builder;
        RigExecFieldValues values = FieldsNode(3).Values();
        values.Set(kN, uint32_t(1));
        builder.AddOp(Fields(), values);
        CHECK(!builder.Build(&error));
        CHECK(Contains(error, "Fields field weights holds 3 values, not 1"));
    }
    {
        RigExecRigDescBuilder builder;
        const float x = 1.0f;
        builder.AddOp(Fields(), &x);
        CHECK(!builder.Build(&error));
        CHECK(Contains(error, "needs its field values"));
    }
    {
        // Node 1 has two gains for three input values: node 0, before it
        // in the batch, still runs.
        RigExecRigDescBuilder builder;
        FieldsModel good = FieldsNode(2), bad = FieldsNode(2);
        bad.gains.resize(2);
        const RigExecNodeId a = builder.AddOp(Fields(), good.Values());
        const RigExecNodeId b = builder.AddOp(Fields(), bad.Values());
        const RigExecInputId aIn = builder.BindInput(a, FieldsIn());
        const RigExecInputId bIn = builder.BindInput(b, FieldsIn());
        const RigExecOutputId aOut = builder.BindOutput(a, FieldsOut());
        auto program = Compile(builder, 0, &error);
        CHECK(program && program->batches.size() == 1);
        if (!program)
            return;
        auto evaluator = RigExecEvaluator::Create(program);
        const Floats8 in = FieldsInput(good, 0, 2);
        CHECK(evaluator->SetInput(aIn, &in) && evaluator->SetInput(bIn, &in));
        CHECK(!evaluator->Evaluate(&error));
        CHECK(Contains(error, "node 1 (test:Fields) failed: field gains "
                              "holds 2 values for 3 input elements"));
        const Vec3 expected = good.Step(in);
        CHECK(Same(*static_cast<const Vec3 *>(evaluator->GetOutput(aOut)),
                   expected));
    }
}

// A history tick runs only the Fields node a window depends on, with the
// history run's own state; the evaluator's state is untouched.
void
TestDeclaredFieldMask()
{
    RigExecRigDescBuilder builder;
    const RingPayload two{2};
    FieldsModel feedsModel = FieldsNode(1), asideModel = FieldsNode(4);
    const RigExecNodeId feeds = builder.AddOp(Fields(), feedsModel.Values());
    const RigExecNodeId aside = builder.AddOp(Fields(), asideModel.Values());
    const RigExecNodeId ring = builder.AddOp(RingSum(), &two);
    builder.Connect(feeds, FieldsOut(), ring, RingSumIn());
    const RigExecInputId feedsIn = builder.BindInput(feeds, FieldsIn());
    const RigExecInputId asideIn = builder.BindInput(aside, FieldsIn());
    const RigExecOutputId feedsOut = builder.BindOutput(feeds, FieldsOut());
    const RigExecOutputId asideOut = builder.BindOutput(aside, FieldsOut());
    std::string error;
    auto program = Compile(builder, 0, &error);
    CHECK(program && program->batches[0].nodeCount == 2);
    if (!program)
        return;
    auto evaluator = RigExecEvaluator::Create(program);
    const auto frame = [&](int f) {
        const Floats8 a = FieldsInput(feedsModel, f, 1);
        const Floats8 b = FieldsInput(asideModel, f, 4);
        CHECK(evaluator->SetInput(feedsIn, &a) &&
              evaluator->SetInput(asideIn, &b));
    };
    frame(0);
    CHECK(evaluator->Evaluate(&error));
    fieldsRuns = 0;
    frame(1);
    CHECK(evaluator->EvaluateHistory(&error));
    CHECK(fieldsRuns.load() == 1);
    frame(2);
    CHECK(evaluator->Evaluate(&error));
    feedsModel.Step(FieldsInput(feedsModel, 0, 1));
    asideModel.Step(FieldsInput(asideModel, 0, 4));
    CHECK(Same(*static_cast<const Vec3 *>(evaluator->GetOutput(feedsOut)),
               feedsModel.Step(FieldsInput(feedsModel, 2, 1))));
    CHECK(Same(*static_cast<const Vec3 *>(evaluator->GetOutput(asideOut)),
               asideModel.Step(FieldsInput(asideModel, 2, 4))));
}

// What TestTaps sees in each callback.
struct TapSeen {
    const RigExecEvaluator *evaluator = nullptr;
    RigExecOutputId last;
    std::vector<size_t> order;
    std::vector<Vec3> vec3s;
    std::vector<float> floats;
    size_t wrongType = 0;
    size_t lastReady = 0;
};

void
SeeTap(void *context, size_t tap, const RigExecTapValue &value)
{
    auto *seen = static_cast<TapSeen *>(context);
    seen->order.push_back(tap);
    if (const Vec3 *v = value.As<Vec3>(fakeTypes::Vec3Type())) {
        seen->vec3s.push_back(*v);
        if (value.As<float>(fakeTypes::Float()))
            ++seen->wrongType;
    } else if (const float *f = value.As<float>(fakeTypes::Float())) {
        seen->floats.push_back(*f);
        if (value.As<Vec3>(fakeTypes::Vec3Type()))
            ++seen->wrongType;
    }
    // The frame is complete before any tap is reported.
    const auto *last =
        static_cast<const Vec3 *>(seen->evaluator->GetOutput(seen->last));
    if (last && last->x != 0.0f)
        ++seen->lastReady;
}

// Taps on a list-port node, a windowed node and a float output arrive typed,
// in tap order, after the whole frame has run.
void
TestTaps()
{
    RigExecRigDescBuilder builder;
    const RingPayload two{2};
    const RigExecNodeId source = builder.AddOp(fakeOps::Offset());
    const RigExecNodeId list = builder.AddOp(Weighted());
    const RigExecNodeId ring = builder.AddOp(RingSum(), &two);
    const RigExecNodeId norm = builder.AddOp(Norm());
    const RigExecNodeId tail = builder.AddOp(fakeOps::Offset());
    const RigExecInputId in = builder.BindInput(source, fakeOps::OffsetIn());
    builder.Connect(source, fakeOps::OffsetOut(), list, WeightedItems());
    builder.Connect(source, fakeOps::OffsetOut(), list, WeightedItems());
    builder.Connect(list, WeightedOut(), ring, RingSumIn());
    builder.Connect(ring, RingSumOut(), norm, NormIn());
    builder.Connect(ring, RingSumOut(), tail, fakeOps::OffsetIn());
    const RigExecOutputId last = builder.BindOutput(tail, fakeOps::OffsetOut());
    std::string error;
    auto program = Compile(builder, 0, &error);
    CHECK(program);
    if (!program)
        return;
    auto evaluator = RigExecEvaluator::Create(program);
    TapSeen seen;
    seen.evaluator = evaluator.get();
    seen.last = last;
    RigExecTapList taps;
    taps.taps.push_back({norm, &NormOut()});
    taps.taps.push_back({ring, &RingSumOut()});
    taps.taps.push_back({list, &WeightedOut()});
    CHECK(evaluator->SetTaps(taps, &SeeTap, &seen));
    const Vec3 x{1.0f, 2.0f, 3.0f};
    CHECK(evaluator->SetInput(in, &x));
    CHECK(evaluator->Evaluate(&error));
    // Source is x + 1; the list weights its two copies 1 and 2.
    const Vec3 weighted{6.0f, 9.0f, 12.0f};
    CHECK((seen.order == std::vector<size_t>{0, 1, 2}));
    CHECK(seen.vec3s.size() == 2 && Same(seen.vec3s[0], weighted) &&
          Same(seen.vec3s[1], weighted));
    CHECK(seen.floats.size() == 1 && seen.floats[0] == 27.0f);
    CHECK(seen.wrongType == 0);
    CHECK(seen.lastReady == 3);
    // A history tick reports no taps.
    seen.order.clear();
    CHECK(evaluator->EvaluateHistory(&error));
    CHECK(seen.order.empty());
}

// Ranged ops over a local array type. RScale writes element i from element
// i of its input and a factor every range shares; RBlur averages each
// element with its neighbours twice, each range keeping its halo plan in its
// scratch.
struct Floats {
    std::vector<float> v;
};

RIGEXEC_DEFINE_TYPE_HANDLE(FloatsType)

const RigExecTypeDesc kFloatsDesc = [] {
    RigExecTypeDesc d;
    d.nameSpace = "test";
    d.name = "Floats";
    d.size = sizeof(Floats);
    d.align = alignof(Floats);
    d.construct = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<Floats *>(dst), 1);
    };
    d.copy = [](void *dst, const void *src) {
        *static_cast<Floats *>(dst) = *static_cast<const Floats *>(src);
    };
    d.move = [](void *dst, void *src) {
        *static_cast<Floats *>(dst) = std::move(*static_cast<Floats *>(src));
    };
    d.destroy = [](void *dst) { static_cast<Floats *>(dst)->~Floats(); };
    d.equal = [](const void *a, const void *b, RigExecEqualMode) {
        const auto &x = static_cast<const Floats *>(a)->v;
        const auto &y = static_cast<const Floats *>(b)->v;
        return x.size() == y.size() &&
               (x.empty() ||
                std::memcmp(x.data(), y.data(), x.size() * sizeof(float)) == 0);
    };
    d.count = [](const void *value) {
        return static_cast<const Floats *>(value)->v.size();
    };
    d.reserve = [](void *value, size_t count) {
        static_cast<Floats *>(value)->v.reserve(count);
    };
    d.elements = [](const void *value) {
        return static_cast<const Floats *>(value)->v.size();
    };
    d.resize = [](void *value, size_t count) {
        static_cast<Floats *>(value)->v.resize(count);
    };
    return d;
}();

void
RegisterRangeTypes(RigExecTypeRegistrar &registrar)
{
    registrar.Add(FloatsType(), kFloatsDesc);
}

const RigExecTypeLibraryDesc kRangeTypes = [] {
    RigExecTypeLibraryDesc d;
    d.name = "testRangeTypes";
    d.registerTypes = &RegisterRangeTypes;
    return d;
}();

RIGEXEC_DEFINE_OP_HANDLE(RScale)
RIGEXEC_DEFINE_PORT_HANDLE(RScaleIn)
RIGEXEC_DEFINE_PORT_HANDLE(RScaleFactor)
RIGEXEC_DEFINE_PORT_HANDLE(RScaleOut)
RIGEXEC_DEFINE_OP_HANDLE(RBlur)
RIGEXEC_DEFINE_PORT_HANDLE(RBlurIn)
RIGEXEC_DEFINE_PORT_HANDLE(RBlurOut)

std::atomic<uint32_t> acceptCalls{0};
std::atomic<uint32_t> prepareCalls{0};
std::atomic<uint32_t> rangeCalls{0};
std::atomic<uint32_t> blurCuts{0};

struct ScaleState {
    float factor = 0.0f;
};

const Floats &
FloatsIn(const RigExecOpBatch &batch, size_t k, const RigExecPortHandle &port)
{
    return *static_cast<const Floats *>(
        batch.inputs[k * batch.portCount + port.Index()]);
}

Floats &
FloatsOut(const RigExecOpBatch &batch, size_t k, const RigExecPortHandle &port)
{
    return *static_cast<Floats *>(
        batch.outputs[k * batch.portCount + port.Index()]);
}

RigExecRangePlan
AcceptScale(const RigExecOpBatch &batch, size_t k, size_t, std::string *error)
{
    acceptCalls.fetch_add(1);
    const float factor = *static_cast<const float *>(
        batch.inputs[k * batch.portCount + RScaleFactor().Index()]);
    if (factor != factor) {
        *error = "factor is not a number";
        return RigExecRangePlan::Refuse;
    }
    return factor == 3.0f ? RigExecRangePlan::Whole : RigExecRangePlan::Ranges;
}

bool
PrepareScale(const RigExecOpBatch &batch, size_t k, size_t, std::string *)
{
    prepareCalls.fetch_add(1);
    static_cast<ScaleState *>(RigExecBatchState(batch, k))->factor =
        *static_cast<const float *>(
            batch.inputs[k * batch.portCount + RScaleFactor().Index()]) *
        0.5f;
    return true;
}

float
ScaleElement(float in, float factor, size_t i)
{
    return in * factor + float(i % 7);
}

bool
RunScale(const RigExecOpBatch &batch, size_t k, const RigExecOpRange &range,
         std::string *error)
{
    rangeCalls.fetch_add(1);
    const std::vector<float> &in = FloatsIn(batch, k, RScaleIn()).v;
    std::vector<float> &out = FloatsOut(batch, k, RScaleOut()).v;
    const float factor =
        static_cast<const ScaleState *>(RigExecBatchState(batch, k))->factor;
    for (size_t i = range.begin; i < range.end; ++i) {
        if (in[i] == -1.0f) {
            *error = "element " + std::to_string(i) + " is -1";
            return false;
        }
        out[i] = ScaleElement(in[i], factor, i);
    }
    return true;
}

/// The mean of a[i - 1], a[i] and a[i + 1], clamped at the ends.
float
Mean3(const float *a, size_t i, size_t n)
{
    return (a[i ? i - 1 : 0] + a[i] + a[i + 1 < n ? i + 1 : n - 1]) / 3.0f;
}

struct BlurScratch {
    size_t begin = SIZE_MAX;
    size_t end = 0;
    size_t count = 0;
    std::vector<float> first;
};

bool
RunBlur(const RigExecOpBatch &batch, size_t k, const RigExecOpRange &range,
        std::string *)
{
    rangeCalls.fetch_add(1);
    const std::vector<float> &in = FloatsIn(batch, k, RBlurIn()).v;
    std::vector<float> &out = FloatsOut(batch, k, RBlurOut()).v;
    BlurScratch &plan = *static_cast<BlurScratch *>(range.scratch);
    const size_t n = range.count;
    const size_t lo = range.begin ? range.begin - 1 : 0;
    const size_t hi = std::min(n, range.end + 1);
    if (plan.begin != range.begin || plan.end != range.end || plan.count != n) {
        plan.begin = range.begin;
        plan.end = range.end;
        plan.count = n;
        plan.first.resize(hi - lo);
        blurCuts.fetch_add(1);
    }
    for (size_t j = lo; j < hi; ++j)
        plan.first[j - lo] = Mean3(in.data(), j, n);
    for (size_t i = range.begin; i < range.end; ++i) {
        const float left = plan.first[(i ? i - 1 : 0) - lo];
        const float right = plan.first[(i + 1 < n ? i + 1 : n - 1) - lo];
        out[i] = (left + plan.first[i - lo] + right) / 3.0f;
    }
    return true;
}

template <class T>
RigExecOpStateHandling
HandlingOf()
{
    RigExecOpStateHandling h;
    h.size = sizeof(T);
    h.align = alignof(T);
    h.create = [](void *dst) {
        std::uninitialized_value_construct_n(static_cast<T *>(dst), 1);
    };
    h.copy = [](void *dst, const void *src) {
        std::uninitialized_copy_n(static_cast<const T *>(src), 1,
                                  static_cast<T *>(dst));
    };
    h.destroy = [](void *dst) { static_cast<T *>(dst)->~T(); };
    return h;
}

void
RegisterRangeOps(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc scalePorts[] = {
        {&RScaleIn(), &FloatsType(), kRigExecPortInput | kRigExecPortPointwise,
         "in"},
        {&RScaleFactor(), &fakeTypes::Float(), kRigExecPortInput, "factor"},
        {&RScaleOut(), &FloatsType(),
         kRigExecPortOutput | kRigExecPortPointwise, "out",
         RigExecOutputBound::Input, &RScaleIn()},
    };
    static const RigExecOpDesc scale = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "RScale";
        d.ports = scalePorts;
        d.portCount = 3;
        d.capabilities = kRigExecOpRanged;
        d.range.accept = &AcceptScale;
        d.range.prepare = &PrepareScale;
        d.range.run = &RunScale;
        d.state = HandlingOf<ScaleState>();
        return d;
    }();
    registrar.Add(RScale(), scale);
    static const RigExecPortDesc blurPorts[] = {
        {&RBlurIn(), &FloatsType(),
         kRigExecPortInput | kRigExecPortNeighbourhood, "in"},
        {&RBlurOut(), &FloatsType(), kRigExecPortOutput | kRigExecPortPointwise,
         "out", RigExecOutputBound::Input, &RBlurIn()},
    };
    static const RigExecOpDesc blur = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "RBlur";
        d.ports = blurPorts;
        d.portCount = 2;
        d.capabilities = kRigExecOpRanged;
        d.range.run = &RunBlur;
        d.range.scratch = HandlingOf<BlurScratch>();
        return d;
    }();
    registrar.Add(RBlur(), blur);
}

const RigExecOpLibraryDesc kRangeOps = [] {
    RigExecOpLibraryDesc d;
    d.name = "testRangeOps";
    d.registerOps = &RegisterRangeOps;
    return d;
}();

/// What RScale, RBlur, RScale computes over \p in, whole.
std::vector<float>
RangeChainReference(const std::vector<float> &in, float factor)
{
    const size_t n = in.size();
    std::vector<float> a(n), first(n), b(n), c(n);
    for (size_t i = 0; i < n; ++i)
        a[i] = ScaleElement(in[i], factor * 0.5f, i);
    for (size_t i = 0; i < n; ++i)
        first[i] = Mean3(a.data(), i, n);
    for (size_t i = 0; i < n; ++i)
        b[i] = Mean3(first.data(), i, n);
    for (size_t i = 0; i < n; ++i)
        c[i] = ScaleElement(b[i], factor * 0.5f, i);
    return c;
}

const RingPayload kRingPayload{3};

/// Chains of RScale, RBlur, RScale over inputs of the given sizes, and with
/// \p sides a windowed chain, a list op and order pairs beside them.
struct RangeRig {
    std::vector<RigExecInputId> ins, factors;
    std::vector<RigExecOutputId> outs;
    std::vector<RigExecNodeId> nodes;
    RigExecInputId ringIn;
    RigExecOutputId ringOut, listOut;
    std::shared_ptr<const RigExecRigDesc> desc;
};

RangeRig
BuildRangeRig(const std::vector<size_t> &sizes, bool sides = true)
{
    RangeRig rig;
    RigExecRigDescBuilder b;
    for (size_t c = 0; c < sizes.size(); ++c) {
        const RigExecNodeId first = b.AddOp(RScale());
        const RigExecNodeId blur = b.AddOp(RBlur());
        const RigExecNodeId last = b.AddOp(RScale());
        b.Connect(first, RScaleOut(), blur, RBlurIn());
        b.Connect(blur, RBlurOut(), last, RScaleIn());
        rig.ins.push_back(b.BindInput(first, RScaleIn()));
        rig.factors.push_back(b.BindInput(first, RScaleFactor()));
        rig.factors.push_back(b.BindInput(last, RScaleFactor()));
        rig.outs.push_back(b.BindOutput(last, RScaleOut()));
        rig.nodes.insert(rig.nodes.end(), {first, blur, last});
    }
    std::string error;
    if (!sides) {
        rig.desc = b.Build(&error);
        return rig;
    }
    const RigExecNodeId ring = b.AddOp(RingSum(), &kRingPayload);
    const RigExecNodeId check = b.AddOp(Check());
    const RigExecNodeId list = b.AddOp(Weighted());
    rig.ringIn = b.BindInput(ring, RingSumIn());
    b.Connect(ring, RingSumOut(), check, CheckIn());
    b.Connect(ring, RingSumOut(), list, WeightedItems());
    b.Connect(check, CheckOut(), list, WeightedItems());
    rig.ringOut = b.BindOutput(ring, RingSumOut());
    rig.listOut = b.BindOutput(list, WeightedOut());
    if (!rig.nodes.empty()) {
        b.Order(rig.nodes[1], check);
        b.Order(ring, rig.nodes.back());
    }
    rig.desc = b.Build(&error);
    if (!rig.desc)
        std::printf("range rig: %s\n", error.c_str());
    return rig;
}

std::vector<float>
RangeInput(size_t n, size_t chain, int frame)
{
    std::vector<float> v(n);
    for (size_t i = 0; i < n; ++i)
        v[i] = float(int((i * 37 + chain * 11 + size_t(frame) * 5) % 101)) *
                   0.37f -
               9.0f;
    return v;
}

bool
SameFloats(const std::vector<float> &a, const std::vector<float> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0);
}

size_t
NonEmptyRanges(size_t n, size_t grain, size_t threshold)
{
    if (!n)
        return 0;
    if (!grain || n < threshold || n <= grain)
        return 1;
    return (n + grain - 1) / grain;
}

std::shared_ptr<const RigExecProgram>
CompileRanges(const RigExecRigDesc &desc, size_t grain, size_t threshold,
              size_t maxBatchNodes)
{
    RigExecCompileOptions options;
    options.rangeGrain = grain;
    options.rangeThreshold = threshold;
    options.maxBatchNodes = maxBatchNodes;
    std::string error;
    auto program = RigExecCompile(desc, options, &error);
    if (!program)
        std::printf("range compile: %s\n", error.c_str());
    return program;
}

// Every partition, batch size and thread count gives the whole run's bits;
// accept and prepare run once per node and frame, each non-empty range
// once, and a halo plan is cut once and kept.
void
TestRangePartitions()
{
    const std::vector<size_t> sizes = {0, 1, 5, 64, 100, 1000, 4097};
    const RangeRig rig = BuildRangeRig(sizes);
    CHECK(rig.desc);
    if (!rig.desc)
        return;
    const float factors[] = {1.5f, 0.75f};
    std::vector<std::unique_ptr<RigExecTaskSystem>> pools;
    for (const size_t threads : {size_t(1), size_t(2), size_t(4), size_t(8)})
        pools.push_back(RigExecCreateThreadTaskSystem(threads));
    for (const size_t grain :
         {size_t(0), size_t(1), size_t(3), size_t(64), size_t(1000)}) {
        for (const size_t threshold : {size_t(0), size_t(100)}) {
            for (const size_t batch : {size_t(0), size_t(1), size_t(2)}) {
                auto program =
                    CompileRanges(*rig.desc, grain, threshold, batch);
                CHECK(program);
                if (!program)
                    return;
                for (size_t pool = 0; pool <= pools.size(); ++pool) {
                    RigExecEvaluatorOptions options;
                    options.tasks = pool ? pools[pool - 1].get() : nullptr;
                    auto evaluator = RigExecEvaluator::Create(program, options);
                    size_t expectedRanges = 0;
                    for (const size_t n : sizes)
                        expectedRanges +=
                            3 * NonEmptyRanges(n, grain, threshold);
                    const Vec3 ring{1.0f, 2.0f, 3.0f};
                    CHECK(evaluator->SetInput(rig.ringIn, &ring));
                    uint32_t cutsAfterFirst = 0;
                    for (int frame = 0; frame < 3; ++frame) {
                        for (size_t c = 0; c < sizes.size(); ++c) {
                            Floats in{RangeInput(sizes[c], c, frame)};
                            CHECK(evaluator->SetInput(rig.ins[c], &in));
                            CHECK(evaluator->SetInput(rig.factors[2 * c],
                                                      &factors[0]));
                            CHECK(evaluator->SetInput(rig.factors[2 * c + 1],
                                                      &factors[0]));
                        }
                        acceptCalls = 0;
                        prepareCalls = 0;
                        rangeCalls = 0;
                        std::string error;
                        CHECK(evaluator->Evaluate(&error));
                        CHECK(acceptCalls == 2 * sizes.size());
                        CHECK(prepareCalls == 2 * sizes.size());
                        CHECK(rangeCalls == expectedRanges);
                        for (size_t c = 0; c < sizes.size(); ++c) {
                            const auto *out = static_cast<const Floats *>(
                                evaluator->GetOutput(rig.outs[c]));
                            const bool same =
                                out &&
                                SameFloats(out->v,
                                           RangeChainReference(
                                               RangeInput(sizes[c], c, frame),
                                               factors[0]));
                            if (!same)
                                std::printf("grain %zu threshold %zu batch "
                                            "%zu pool %zu frame %d chain %zu "
                                            "differs\n",
                                            grain, threshold, batch, pool,
                                            frame, c);
                            CHECK(same);
                        }
                        if (frame == 0)
                            cutsAfterFirst = blurCuts.load();
                    }
                    CHECK(blurCuts.load() == cutsAfterFirst);
                    blurCuts = 0;
                }
            }
        }
    }
}

// A Whole plan runs one range over every element whatever the grain; only
// the nodes an edit reaches run accept, prepare and their ranges again.
void
TestRangeWholeAndDirty()
{
    const std::vector<size_t> sizes = {300, 300};
    const RangeRig rig = BuildRangeRig(sizes);
    auto program = CompileRanges(*rig.desc, 16, 0, 0);
    CHECK(program);
    if (!program)
        return;
    auto pool = RigExecCreateThreadTaskSystem(4);
    RigExecEvaluatorOptions options;
    options.tasks = pool.get();
    auto evaluator = RigExecEvaluator::Create(program, options);
    const Vec3 ring{1.0f, 2.0f, 3.0f};
    CHECK(evaluator->SetInput(rig.ringIn, &ring));
    const float whole = 3.0f;
    for (size_t c = 0; c < sizes.size(); ++c) {
        Floats in{RangeInput(sizes[c], c, 0)};
        CHECK(evaluator->SetInput(rig.ins[c], &in));
        CHECK(evaluator->SetInput(rig.factors[2 * c], &whole));
        CHECK(evaluator->SetInput(rig.factors[2 * c + 1], &whole));
    }
    rangeCalls = 0;
    CHECK(evaluator->Evaluate());
    // Two Whole scales and one blur of 19 ranges per chain.
    CHECK(rangeCalls == 2 * (2 + 19));
    for (size_t c = 0; c < sizes.size(); ++c) {
        const auto *out =
            static_cast<const Floats *>(evaluator->GetOutput(rig.outs[c]));
        CHECK(out &&
              SameFloats(out->v, RangeChainReference(RangeInput(sizes[c], c, 0),
                                                     whole)));
    }
    Floats edited{RangeInput(sizes[1], 1, 4)};
    CHECK(evaluator->SetInput(rig.ins[1], &edited));
    acceptCalls = 0;
    prepareCalls = 0;
    rangeCalls = 0;
    CHECK(evaluator->Evaluate());
    CHECK(acceptCalls == 2);
    CHECK(prepareCalls == 2);
    CHECK(rangeCalls == 2 + 19);
    const auto *out =
        static_cast<const Floats *>(evaluator->GetOutput(rig.outs[1]));
    CHECK(out && SameFloats(out->v, RangeChainReference(edited.v, whole)));
    blurCuts = 0;
}

// The first failure in batch order is reported whatever the partition and
// thread count: a range's failure in an earlier instance before a refusal
// in a later one, and a refusal before a later instance's range failure.
void
TestRangeFailures()
{
    const std::vector<size_t> sizes = {200, 200, 200};
    const RangeRig rig = BuildRangeRig(sizes);
    auto pool = RigExecCreateThreadTaskSystem(4);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float one = 1.0f;
    for (int scenario = 0; scenario < 2; ++scenario) {
        std::string reference;
        for (const size_t grain :
             {size_t(0), size_t(1), size_t(7), size_t(64)}) {
            for (const size_t batch : {size_t(0), size_t(1)}) {
                auto program = CompileRanges(*rig.desc, grain, 0, batch);
                for (RigExecTaskSystem *tasks :
                     {static_cast<RigExecTaskSystem *>(nullptr), pool.get()}) {
                    RigExecEvaluatorOptions options;
                    options.tasks = tasks;
                    auto evaluator = RigExecEvaluator::Create(program, options);
                    const Vec3 ring{1.0f, 2.0f, 3.0f};
                    CHECK(evaluator->SetInput(rig.ringIn, &ring));
                    for (size_t c = 0; c < sizes.size(); ++c) {
                        Floats in{RangeInput(sizes[c], c, 0)};
                        if (c == (scenario ? 2u : 0u))
                            in.v[150] = -1.0f;
                        CHECK(evaluator->SetInput(rig.ins[c], &in));
                        const float &factor =
                            c == (scenario ? 0u : 1u) ? nan : one;
                        CHECK(evaluator->SetInput(rig.factors[2 * c], &factor));
                        CHECK(
                            evaluator->SetInput(rig.factors[2 * c + 1], &one));
                    }
                    std::string error;
                    CHECK(!evaluator->Evaluate(&error));
                    if (reference.empty())
                        reference = error;
                    if (error != reference)
                        std::printf("scenario %d grain %zu: %s, not %s\n",
                                    scenario, grain, error.c_str(),
                                    reference.c_str());
                    CHECK(error == reference);
                }
            }
        }
        CHECK(Contains(reference, scenario ? "factor is not a number"
                                           : "element 150 is -1"));
    }
    blurCuts = 0;
}

// The registry refuses a ranged op without range handling, range handling
// without the capability, pointwise ports on an op without ranges and a
// ranged op with its own run function.
void
TestRangeRegistryErrors()
{
    static const RigExecPortDesc ports[] = {
        {&RScaleIn(), &FloatsType(), kRigExecPortInput | kRigExecPortPointwise,
         "in"},
        {&RScaleOut(), &FloatsType(),
         kRigExecPortOutput | kRigExecPortPointwise, "out",
         RigExecOutputBound::Input, &RScaleIn()},
    };
    static RigExecOpDesc bad;
    bad.nameSpace = "test";
    bad.name = "Bad";
    bad.ports = ports;
    bad.portCount = 2;
    static RigExecOpHandle badHandle;
    const auto check = [&](const char *expected) {
        RigExecOpLibraryDesc library;
        library.name = "testBadRange";
        library.registerOps = [](RigExecOpRegistrar &registrar) {
            registrar.Add(badHandle, bad);
        };
        RigExecRegistryInit init;
        init.typeLibraries = RigExecLinkedRegistrations().typeLibraries;
        init.typeLibraries.push_back(&kRangeTypes);
        init.opLibraries.push_back(&library);
        std::vector<std::string> errors;
        CHECK(!RigExecRegistry::Validate(init, &errors));
        bool found = false;
        for (const std::string &e : errors)
            found = found || Contains(e, expected);
        if (!found)
            std::printf("no error naming \"%s\"\n", expected);
        CHECK(found);
    };
    bad.run = &RunCheck;
    check("is pointwise but the op does not run in ranges");
    bad.capabilities = kRigExecOpRanged;
    check("the Ranged capability and range handling go together");
    bad.range.run = &RunBlur;
    check("a Ranged op runs through its range handling only");
    bad.run = nullptr;
    bad.threading = RigExecOpThreading::OwnerThread;
    check("a Ranged op must be Pure");
}

// After the first frames a ranged rig allocates nothing more in this
// module, inline and on worker threads.
void
TestRangeNoAllocation()
{
    const std::vector<size_t> sizes = {1000, 50};
    const RangeRig rig = BuildRangeRig(sizes, false);
    auto program = CompileRanges(*rig.desc, 32, 0, 0);
    auto pool = RigExecCreateThreadTaskSystem(4);
    for (RigExecTaskSystem *tasks :
         {static_cast<RigExecTaskSystem *>(nullptr), pool.get()}) {
        RigExecEvaluatorOptions options;
        options.tasks = tasks;
        auto evaluator = RigExecEvaluator::Create(program, options);
        const float factor = 1.25f;
        std::vector<Floats> inputs;
        for (size_t c = 0; c < sizes.size(); ++c) {
            inputs.push_back(Floats{RangeInput(sizes[c], c, 0)});
            CHECK(evaluator->SetInput(rig.factors[2 * c], &factor));
            CHECK(evaluator->SetInput(rig.factors[2 * c + 1], &factor));
        }
        for (int frame = 0; frame < 6; ++frame) {
            for (size_t c = 0; c < sizes.size(); ++c) {
                inputs[c].v[0] = float(frame);
                CHECK(evaluator->SetInput(rig.ins[c], &inputs[c]));
            }
            gAllocations = 0;
            gCountAllocations = frame > 2;
            CHECK(evaluator->Evaluate());
            gCountAllocations = false;
            if (frame > 2)
                CHECK(gAllocations.load() == 0);
        }
    }
    blurCuts = 0;
}

} // namespace

int
main()
{
    RigExecRegistryInit init = RigExecLinkedRegistrations();
    init.opLibraries.push_back(&kCheckLibrary);
    init.typeLibraries.push_back(&kFieldTypes);
    init.opLibraries.push_back(&kFieldsLibrary);
    init.typeLibraries.push_back(&kRangeTypes);
    init.opLibraries.push_back(&kRangeOps);
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
    TestSharedTaskSystem();
    TestTaskSystemStress();
    TestTaskSystemLifetimes();
    TestScheduleOrder();
    TestBatchCalls();
    TestBatchHistoryMask();
    TestCanonicalOrder();
    TestDeclaredFields();
    TestDeclaredFieldErrors();
    TestDeclaredFieldMask();
    TestTaps();
    TestStoreLayout();
    TestInputWriter();
    TestPins();
    TestDirtyPropagation();
    TestRunStates();
    TestRangeRegistryErrors();
    TestRangePartitions();
    TestRangeWholeAndDirty();
    TestRangeFailures();
    TestRangeNoAllocation();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
