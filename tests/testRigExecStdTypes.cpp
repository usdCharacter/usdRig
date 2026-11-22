// The standard type library: layouts match rigExecRuntime's values, arrays
// own and copy their storage through the core evaluator, and equality and
// hashing follow the registered modes.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

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
RIGEXEC_DEFINE_OP_HANDLE(Scale)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleIn)
RIGEXEC_DEFINE_PORT_HANDLE(ScaleOut)
RIGEXEC_DEFINE_OP_HANDLE(Sum)
RIGEXEC_DEFINE_PORT_HANDLE(SumIn)
RIGEXEC_DEFINE_PORT_HANDLE(SumOut)

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
        {&ScaleOut(), &rigExecStd::Vec3fArray(), kRigExecPortOutput, "out"},
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
}

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
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
