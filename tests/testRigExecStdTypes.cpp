// The standard type library: layouts match rigExecRuntime's values, arrays
// own and copy their storage through the core evaluator, and equality and
// hashing follow the registered modes.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExec/api/taskSystem.h"
#include "rigExecStdTypes/stdTypes.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>
#include <vector>

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

using namespace rigExec;
using rigExecStd::RigExecArray;
using rigExecStd::Vec3f;

namespace {
int failures = 0;
#define CHECK(value)                                                           \
    do {                                                                       \
        if (!(value)) {                                                        \
            ++failures;                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);       \
        }                                                                      \
    } while (0)

using Vec3fArray = RigExecArray<Vec3f>;

// Scale: Vec3fArray -> Vec3fArray, doubling each point.
// Sum: Vec3fArray -> Vec3f.
// Grow: writes one point more than its bound allows.
// Unbounded and Overbound break the bound rules, for Validate.
RIGEXEC_DEFINE_OP_HANDLE(Scale)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleIn)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleOut)
RIGEXEC_DEFINE_OP_HANDLE(Sum)
RIGEXEC_DEFINE_PORT_HANDLE(SumIn)
RIGEXEC_DEFINE_PORT_HANDLE(SumOut)
RIGEXEC_DEFINE_OP_HANDLE(Grow)
RIGEXEC_DEFINE_PORT_HANDLE(GrowIn)
RIGEXEC_DEFINE_PORT_HANDLE(GrowOut)
RIGEXEC_DEFINE_OP_HANDLE(Unbounded)
RIGEXEC_DEFINE_PORT_HANDLE(UnboundedIn)
RIGEXEC_DEFINE_PORT_HANDLE(UnboundedOut)
RIGEXEC_DEFINE_OP_HANDLE(Overbound)
RIGEXEC_DEFINE_PORT_HANDLE(OverboundIn)
RIGEXEC_DEFINE_PORT_HANDLE(OverboundOut)

bool
RunScale(const void *, void *, const RigExecOpIo &io, std::string *)
{
    const auto &in =
        *static_cast<const Vec3fArray *>(io.inputs[ScaleIn().Index()]);
    auto &out = *static_cast<Vec3fArray *>(io.outputs[ScaleOut().Index()]);
    out.Resize(in.Size());
    for (size_t i = 0; i < in.Size(); ++i) {
        for (int c = 0; c < 3; ++c)
            out[i]._data[c] = in[i]._data[c] * 2.0f;
    }
    return true;
}

bool
RunGrow(const void *, void *, const RigExecOpIo &io, std::string *)
{
    const auto &in = *static_cast<const Vec3fArray *>(io.inputs[0]);
    static_cast<Vec3fArray *>(io.outputs[1])->Resize(in.Size() + 1);
    return true;
}

bool
RunSum(const void *, void *, const RigExecOpIo &io, std::string *)
{
    const auto &in =
        *static_cast<const Vec3fArray *>(io.inputs[SumIn().Index()]);
    Vec3f sum{};
    for (const Vec3f &p : in) {
        for (int c = 0; c < 3; ++c)
            sum._data[c] += p._data[c];
    }
    *static_cast<Vec3f *>(io.outputs[SumOut().Index()]) = sum;
    return true;
}

void
RegisterOps(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc scalePorts[] = {
        {&ScaleIn(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "in"},
        {&ScaleOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
         RigExecOutputBound::Input, &ScaleIn()},
    };
    static const RigExecPortDesc sumPorts[] = {
        {&SumIn(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "in"},
        {&SumOut(), &rigExecStd::Vec3fType(), kRigExecPortOutput, "out"},
    };
    static const RigExecOpDesc scale = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Scale";
        d.ports = scalePorts;
        d.portCount = 2;
        d.run = &RunScale;
        return d;
    }();
    static const RigExecOpDesc sum = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Sum";
        d.ports = sumPorts;
        d.portCount = 2;
        d.run = &RunSum;
        return d;
    }();
    registrar.Add(Scale(), scale);
    registrar.Add(Sum(), sum);
    static const RigExecPortDesc growPorts[] = {
        {&GrowIn(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "in"},
        {&GrowOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out",
         RigExecOutputBound::Input, &GrowIn()},
    };
    static const RigExecOpDesc grow = [] {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = "Grow";
        d.ports = growPorts;
        d.portCount = 2;
        d.run = &RunGrow;
        return d;
    }();
    registrar.Add(Grow(), grow);
}

void
RegisterBadBounds(RigExecOpRegistrar &registrar)
{
    static const RigExecPortDesc unboundedPorts[] = {
        {&UnboundedIn(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "in"},
        {&UnboundedOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
    };
    static const RigExecPortDesc overboundPorts[] = {
        {&OverboundIn(), &rigExecStd::Vec3fArray(), kRigExecPortInput, "in"},
        {&OverboundOut(), &rigExecStd::Vec3fType(), kRigExecPortOutput, "out",
         RigExecOutputBound::Input, &OverboundIn()},
    };
    const auto op = [](const char *name, const RigExecPortDesc *ports) {
        RigExecOpDesc d;
        d.nameSpace = "test";
        d.name = name;
        d.ports = ports;
        d.portCount = 2;
        d.run = &RunGrow;
        return d;
    };
    static const RigExecOpDesc unbounded = op("Unbounded", unboundedPorts);
    static const RigExecOpDesc overbound = op("Overbound", overboundPorts);
    registrar.Add(Unbounded(), unbounded);
    registrar.Add(Overbound(), overbound);
}

const RigExecOpLibraryDesc kBadBounds = [] {
    RigExecOpLibraryDesc d;
    d.name = "testBadBounds";
    d.registerOps = &RegisterBadBounds;
    return d;
}();

const RigExecOpLibraryDesc kOps = [] {
    RigExecOpLibraryDesc d;
    d.name = "testStdTypesOps";
    d.registerOps = &RegisterOps;
    return d;
}();

const RigExecTypeDesc &
Desc(const RigExecTypeHandle &handle)
{
    return *RigExecRegistry::Get().Type(handle).desc;
}

void
TestLayouts()
{
    CHECK(Desc(rigExecStd::Vec3fType()).size == sizeof(rigExec::RrVec3f));
    CHECK(Desc(rigExecStd::Vec3fType()).size == 12);
    CHECK(Desc(rigExecStd::Mat4dType()).size == 128);
    CHECK(Desc(rigExecStd::QuatdType()).size == 32);
    CHECK(Desc(rigExecStd::Vec3iType()).size == 12);
    CHECK(Desc(rigExecStd::Vec3fType()).triviallyCopyable);
    CHECK(!Desc(rigExecStd::Vec3fArray()).triviallyCopyable);
    CHECK(Desc(rigExecStd::Vec3fArray()).element == &rigExecStd::Vec3fType());
    CHECK(Desc(rigExecStd::Vec3fArray()).layoutHash !=
          Desc(rigExecStd::Vec3fType()).layoutHash);
    CHECK(Desc(rigExecStd::Vec3fType()).layoutHash !=
          Desc(rigExecStd::Vec3dType()).layoutHash);
    CHECK(Desc(rigExecStd::IntArray()).layoutHash !=
          Desc(rigExecStd::UIntArray()).layoutHash);
}

void
TestArrayStorage()
{
    Vec3fArray small(3);
    CHECK(reinterpret_cast<uintptr_t>(small.Data()) % 16 == 0);
    CHECK(small[2]._data[1] == 0.0f);
    Vec3fArray large(1000);
    CHECK(reinterpret_cast<uintptr_t>(large.Data()) % 256 == 0);
    small[1]._data[0] = 5.0f;
    small.Resize(400);
    CHECK(small[1]._data[0] == 5.0f && small[399]._data[2] == 0.0f);
    Vec3fArray copy = small;
    copy[1]._data[0] = 6.0f;
    CHECK(small[1]._data[0] == 5.0f);
    Vec3fArray moved = std::move(copy);
    CHECK(copy.Empty() && moved.Size() == 400);
}

void
TestEqualAndHash()
{
    const RigExecTypeDesc &f = Desc(rigExecStd::FloatArray());
    RigExecArray<float> a(2), b(2);
    a[0] = NAN;
    b[0] = NAN;
    CHECK(f.equal(&a, &b, RigExecEqualMode::Bitwise));
    CHECK(f.equal(&a, &b, RigExecEqualMode::NaNEqual));
    CHECK(!f.equal(&a, &b, RigExecEqualMode::Ieee));
    a[1] = 0.0f;
    b[1] = -0.0f;
    CHECK(!f.equal(&a, &b, RigExecEqualMode::Bitwise));
    CHECK(f.equal(&a, &b, RigExecEqualMode::NaNEqual));
    b[1] = 0.0f;
    CHECK(f.hash(&a) == f.hash(&b));
    b.Resize(3);
    CHECK(!f.equal(&a, &b, RigExecEqualMode::NaNEqual));
    CHECK(f.hash(&a) != f.hash(&b));

    const RigExecTypeDesc &v = Desc(rigExecStd::Vec3fType());
    const Vec3f p(1.0f, 2.0f, 3.0f);
    Vec3f q = p;
    CHECK(v.equal(&p, &q, RigExecEqualMode::Ieee) && v.hash(&p) == v.hash(&q));
    q._data[2] = 4.0f;
    CHECK(!v.equal(&p, &q, RigExecEqualMode::Ieee));
}

// in -> Scale -> Sum -> out, with the scaled array also bound as an output.
void
TestEvaluate()
{
    RigExecRigDescBuilder builder;
    const RigExecNodeId scale = builder.AddOp(Scale());
    const RigExecNodeId sum = builder.AddOp(Sum());
    builder.Connect(scale, ScaleOut(), sum, SumIn());
    const RigExecInputId in = builder.BindInput(scale, ScaleIn());
    const RigExecOutputId scaled = builder.BindOutput(scale, ScaleOut());
    const RigExecOutputId total = builder.BindOutput(sum, SumOut());
    std::string error;
    std::shared_ptr<const RigExecRigDesc> desc = builder.Build(&error);
    CHECK(desc);
    if (!desc)
        return;
    std::shared_ptr<const RigExecProgram> program =
        RigExecCompile(*desc, {}, &error);
    CHECK(program);
    if (!program)
        return;
    auto evaluator = RigExecEvaluator::Create(program, {}, &error);
    CHECK(evaluator);
    if (!evaluator)
        return;

    for (size_t count : {size_t(0), size_t(3), size_t(2000), size_t(5)}) {
        Vec3fArray points(count);
        for (size_t i = 0; i < count; ++i)
            points[i] = Vec3f(float(i), 1.0f, -0.5f);
        CHECK(evaluator->SetInput(in, &points, &error));
        // The evaluator copied the input; changing ours changes nothing.
        if (count)
            points[0]._data[1] = 100.0f;
        CHECK(evaluator->Evaluate(&error));
        const auto &out =
            *static_cast<const Vec3fArray *>(evaluator->GetOutput(scaled));
        CHECK(out.Size() == count);
        const Vec3f &s =
            *static_cast<const Vec3f *>(evaluator->GetOutput(total));
        const float n = float(count);
        CHECK(s._data[0] == n * (n - 1.0f));
        CHECK(s._data[1] == 2.0f * n);
        CHECK(s._data[2] == -n);
    }
}

std::unique_ptr<RigExecEvaluator>
Build(const RigExecOpHandle &op, const RigExecPortHandle &in,
      const RigExecPortHandle &out, RigExecInputId *input,
      RigExecOutputId *output)
{
    RigExecRigDescBuilder builder;
    const RigExecNodeId node = builder.AddOp(op);
    *input = builder.BindInput(node, in);
    *output = builder.BindOutput(node, out);
    std::shared_ptr<const RigExecRigDesc> desc = builder.Build();
    std::shared_ptr<const RigExecProgram> program =
        desc ? RigExecCompile(*desc) : nullptr;
    return program ? RigExecEvaluator::Create(program) : nullptr;
}

// An output holds at most its declared bound: room for it is made before
// the frame, and a node writing more fails.
void
TestOutputBounds()
{
    std::vector<std::string> errors;
    RigExecRegistryInit bad = RigExecLinkedRegistrations();
    bad.opLibraries.push_back(&kBadBounds);
    CHECK(!RigExecRegistry::Validate(bad, &errors));
    const auto mentions = [&](const char *text) {
        for (const std::string &e : errors)
            if (e.find(text) != std::string::npos)
                return true;
        return false;
    };
    CHECK(mentions("port out holds arrays and declares no bound"));
    CHECK(mentions("port out declares a bound but is not an output"));

    RigExecInputId in;
    RigExecOutputId out;
    auto grow = Build(Grow(), GrowIn(), GrowOut(), &in, &out);
    CHECK(grow);
    if (!grow)
        return;
    std::string error;
    Vec3fArray points(3);
    CHECK(grow->SetInput(in, &points));
    CHECK(!grow->Evaluate(&error));
    CHECK(error.find("output out holds 4 elements, past its bound of 3") !=
          std::string::npos);

    auto scale = Build(Scale(), ScaleIn(), ScaleOut(), &in, &out);
    CHECK(scale);
    if (!scale)
        return;
    points = Vec3fArray(2000);
    CHECK(scale->SetInput(in, &points));
    CHECK(scale->Evaluate(&error));
    const auto *scaled = static_cast<const Vec3fArray *>(scale->GetOutput(out));
    CHECK(scaled->Size() == 2000 && scaled->Capacity() >= 2000);
    const Vec3f *storage = scaled->Data();
    points = Vec3fArray(5);
    CHECK(scale->SetInput(in, &points));
    CHECK(scale->Evaluate(&error));
    CHECK(scaled->Size() == 5 && scaled->Data() == storage);
}

// After the first frame an evaluation allocates nothing, inline and on
// worker threads, while the host edits its input in place.
void
TestNoAllocation()
{
    RigExecRigDescBuilder builder;
    const RigExecNodeId scale = builder.AddOp(Scale());
    const RigExecNodeId sum = builder.AddOp(Sum());
    builder.Connect(scale, ScaleOut(), sum, SumIn());
    const RigExecInputId in = builder.BindInput(scale, ScaleIn());
    builder.BindOutput(scale, ScaleOut());
    const RigExecOutputId total = builder.BindOutput(sum, SumOut());
    std::shared_ptr<const RigExecRigDesc> desc = builder.Build();
    std::shared_ptr<const RigExecProgram> program =
        desc ? RigExecCompile(*desc) : nullptr;
    CHECK(program);
    if (!program)
        return;
    std::unique_ptr<RigExecTaskSystem> pool = RigExecCreateThreadTaskSystem(3);
    for (RigExecTaskSystem *tasks :
         {static_cast<RigExecTaskSystem *>(nullptr), pool.get()}) {
        RigExecEvaluatorOptions options;
        options.tasks = tasks;
        auto evaluator = RigExecEvaluator::Create(program, options);
        Vec3fArray points(1000);
        CHECK(evaluator->SetInput(in, &points));
        CHECK(evaluator->Evaluate());
        for (int frame = 1; frame < 6; ++frame) {
            RigExecInputWriter writer = evaluator->BeginInputEdit();
            Vec3fArray *edit =
                writer.Edit<Vec3fArray>(in, rigExecStd::Vec3fArray());
            CHECK(edit && edit->Size() == 1000);
            if (!edit)
                return;
            (*edit)[0] = Vec3f(float(frame), 0.0f, 0.0f);
            writer.Commit();
            gAllocations.store(0);
            gCountAllocations.store(true);
            const bool ran = evaluator->Evaluate();
            gCountAllocations.store(false);
            CHECK(ran);
            CHECK(gAllocations.load() == 0);
            const auto &s =
                *static_cast<const Vec3f *>(evaluator->GetOutput(total));
            CHECK(s._data[0] == 2.0f * float(frame));
        }
    }
}

} // namespace

int
main()
{
    RigExecRegistryInit init = RigExecLinkedRegistrations();
    init.opLibraries.push_back(&kOps);
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(init, &errors)) {
        for (const std::string &e : errors)
            std::printf("%s\n", e.c_str());
        return 1;
    }
    TestLayouts();
    TestArrayStorage();
    TestEqualAndHash();
    TestEvaluate();
    TestOutputBounds();
    TestNoAllocation();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
