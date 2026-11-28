// The property math ops evaluated through the core give the same bits as
// rigExecMath's Gf entry points, which the baked evaluator calls, for every
// operation; a disabled node passes its input through, and what cannot be
// computed fails compile or the evaluation.
#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/registry.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecOps/property/mathOps.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

using namespace rigExec;
using rigExecStd::Mat4d;
using rigExecStd::RigExecArray;
using rigExecStd::Vec2f;
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

template <class T>
bool
SameBits(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

/// One op node with every port bound, evaluated over many inputs.
struct Harness {
    std::unique_ptr<RigExecEvaluator> evaluator;
    std::vector<RigExecInputId> inputs;
    RigExecOutputId out;
    std::string error;

    Harness(const RigExecOpHandle &op, const rigExecOps::MathPayload &payload,
            const std::vector<const RigExecPortHandle *> &ins,
            const RigExecPortHandle &output)
    {
        RigExecRigDescBuilder b;
        const RigExecNodeId node = b.AddOp(op, &payload);
        for (const RigExecPortHandle *port : ins)
            inputs.push_back(b.BindInput(node, *port));
        out = b.BindOutput(node, output);
        auto desc = b.Build(&error);
        auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
        evaluator =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    }

    /// Sets every input in order and evaluates; false with \p error on a
    /// refused run.
    bool Run(const std::vector<const void *> &values, std::string *error)
    {
        if (!evaluator)
            return false;
        for (size_t i = 0; i < values.size(); ++i)
            if (!evaluator->SetInput(inputs[i], values[i], error))
                return false;
        return evaluator->Evaluate(error);
    }

    template <class T> const T &Out() const
    {
        return *static_cast<const T *>(evaluator->GetOutput(out));
    }
};

const RigExecPropertyOp kAllOps[] = {
    RigExecPropertyOp::Add,   RigExecPropertyOp::Multiply,
    RigExecPropertyOp::Clamp, RigExecPropertyOp::Remap,
    RigExecPropertyOp::Blend, RigExecPropertyOp::Quantize,
};

/// Finite values with the edges the kernels branch on.
float
Pick(std::mt19937 &rng)
{
    static const float edges[] = {0.0f, -0.0f, 1.0f, -1.0f, 0.5f, 1e-30f};
    std::uniform_int_distribution<int> which(0, 9);
    const int w = which(rng);
    if (w < int(std::size(edges)))
        return edges[w];
    return std::uniform_real_distribution<float>(-10.0f, 10.0f)(rng);
}

/// A weight the ops mix with: finite and in [0, 1], the edges included.
float
PickWeight(std::mt19937 &rng)
{
    static const float edges[] = {0.0f, 1.0f, 0.5f};
    std::uniform_int_distribution<int> which(0, 5);
    const int w = which(rng);
    if (w < int(std::size(edges)))
        return edges[w];
    return std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
}

bool
Finite(float v)
{
    return std::isfinite(v);
}

bool
Finite(const GfVec3f &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
Finite(const GfMatrix4d &m)
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            if (!std::isfinite(m[r][c]))
                return false;
    return true;
}

void
TestFloat()
{
    std::mt19937 rng(7);
    for (const RigExecPropertyOp op : kAllOps) {
        rigExecOps::MathPayload payload;
        payload.op = op;
        Harness h(rigExecOps::FloatMath(), payload,
                  {&rigExecOps::FloatIn(), &rigExecOps::FloatValue(),
                   &rigExecOps::FloatMin(), &rigExecOps::FloatMax(),
                   &rigExecOps::FloatWeight()},
                  rigExecOps::FloatOut());
        CHECK(h.evaluator);
        if (!h.evaluator)
            continue;
        for (int i = 0; i < 2000; ++i) {
            const float in = Pick(rng), value = Pick(rng), lo = Pick(rng),
                        hi = Pick(rng), weight = PickWeight(rng);
            RigExecPropertyMathParams<float> params;
            params.op = op;
            params.value = value;
            params.min = lo;
            params.max = hi;
            params.weight = weight;
            const float expected = RigExecApplyFloatMath(in, params);
            std::string error;
            const bool ran = h.Run({&in, &value, &lo, &hi, &weight}, &error);
            CHECK(ran == Finite(expected));
            CHECK(!ran || SameBits(h.Out<float>(), expected));
        }
    }
}

void
TestFloatCurve()
{
    rigExecOps::MathPayload payload;
    payload.op = RigExecPropertyOp::Curve;
    RigExecArray<Vec2f> keyIn(4), tangentIn;
    keyIn[0] = Vec2f(-1.0f, 2.0f);
    keyIn[1] = Vec2f(0.0f, 0.0f);
    keyIn[2] = Vec2f(0.5f, 1.0f);
    keyIn[3] = Vec2f(3.0f, -4.0f);
    const std::vector<GfVec2f> keys = {GfVec2f(-1, 2), GfVec2f(0, 0),
                                       GfVec2f(0.5f, 1), GfVec2f(3, -4)};
    const std::vector<GfVec2f> tangents = {GfVec2f(1, -2), GfVec2f(0, 0.5f),
                                           GfVec2f(3, 1), GfVec2f(-1, 2)};
    for (const bool hermite : {false, true}) {
        if (hermite) {
            tangentIn.Resize(4);
            tangentIn[0] = Vec2f(1.0f, -2.0f);
            tangentIn[1] = Vec2f(0.0f, 0.5f);
            tangentIn[2] = Vec2f(3.0f, 1.0f);
            tangentIn[3] = Vec2f(-1.0f, 2.0f);
        }
        Harness h(rigExecOps::FloatMath(), payload,
                  {&rigExecOps::FloatIn(), &rigExecOps::FloatWeight(),
                   &rigExecOps::FloatKeys(), &rigExecOps::FloatTangents()},
                  rigExecOps::FloatOut());
        CHECK(h.evaluator);
        if (!h.evaluator)
            continue;
        for (float in = -3.0f; in <= 4.0f; in += 0.125f) {
            const float weight = 0.75f;
            std::string error;
            CHECK(h.Run({&in, &weight, &keyIn, &tangentIn}, &error));
            RigExecPropertyMathParams<float> params;
            params.op = RigExecPropertyOp::Curve;
            params.weight = weight;
            params.keys = keys.data();
            params.keyCount = keys.size();
            if (hermite) {
                params.tangents = tangents.data();
                params.tangentCount = tangents.size();
            }
            CHECK(SameBits(h.Out<float>(), RigExecApplyFloatMath(in, params)));
        }
    }

    // Unordered keys fail the evaluation; a curve fed no keys fails compile.
    RigExecArray<Vec2f> unordered(2), none;
    unordered[0] = Vec2f(1.0f, 0.0f);
    unordered[1] = Vec2f(0.0f, 1.0f);
    Harness bad(rigExecOps::FloatMath(), payload,
                {&rigExecOps::FloatIn(), &rigExecOps::FloatWeight(),
                 &rigExecOps::FloatKeys(), &rigExecOps::FloatTangents()},
                rigExecOps::FloatOut());
    const float in = 0.0f, weight = 1.0f;
    std::string error;
    CHECK(bad.evaluator && !bad.Run({&in, &weight, &unordered, &none}, &error));
    CHECK(error.find("curve") != std::string::npos);
    Harness unfed(rigExecOps::FloatMath(), payload,
                  {&rigExecOps::FloatIn(), &rigExecOps::FloatWeight()},
                  rigExecOps::FloatOut());
    CHECK(!unfed.evaluator &&
          unfed.error.find("curve needs keys") != std::string::npos);
}

void
TestFloatPath()
{
    rigExecOps::MathPayload payload;
    payload.op = RigExecPropertyOp::Path;
    payload.path.valid = true;
    payload.path.basis = RigExecPathBasis::CatmullRom;
    payload.path.component = RigExecPathComponent::Heading;
    payload.path.offsetX = 0.25f;
    payload.path.headingOffset = 10.0f;
    RigExecArray<Vec3f> points(5);
    const float coords[5][3] = {
        {0, 0, 0}, {1, 0, 1}, {2, 0, 0}, {3, 0, -1}, {4, 0, 0}};
    for (size_t i = 0; i < 5; ++i)
        points[i] = Vec3f(coords[i][0], coords[i][1], coords[i][2]);
    Harness h(rigExecOps::FloatMath(), payload,
              {&rigExecOps::FloatIn(), &rigExecOps::FloatValue(),
               &rigExecOps::FloatWeight(), &rigExecOps::FloatPathPoints()},
              rigExecOps::FloatOut());
    CHECK(h.evaluator);
    if (!h.evaluator)
        return;
    for (float in = -1.0f; in <= 6.0f; in += 0.25f) {
        const float start = 0.5f, weight = 1.0f;
        std::string error;
        CHECK(h.Run({&in, &start, &weight, &points}, &error));
        RigExecPropertyMathParams<float> params;
        params.op = RigExecPropertyOp::Path;
        params.value = start;
        params.weight = weight;
        params.path.points = &coords[0][0];
        params.path.pointCount = 5;
        params.path.basis = payload.path.basis;
        params.pathSettings = payload.path;
        CHECK(SameBits(h.Out<float>(), RigExecApplyFloatMath(in, params)));
    }
}

void
TestVec3f()
{
    std::mt19937 rng(11);
    const auto pick3 = [&] { return Vec3f(Pick(rng), Pick(rng), Pick(rng)); };
    const auto gf = [](const Vec3f &v) { return GfVec3f(v[0], v[1], v[2]); };
    for (const RigExecPropertyOp op :
         {RigExecPropertyOp::Add, RigExecPropertyOp::Multiply,
          RigExecPropertyOp::Clamp, RigExecPropertyOp::Remap,
          RigExecPropertyOp::Blend}) {
        rigExecOps::MathPayload payload;
        payload.op = op;
        Harness h(rigExecOps::Vec3fMath(), payload,
                  {&rigExecOps::Vec3fIn(), &rigExecOps::Vec3fValue(),
                   &rigExecOps::Vec3fMin(), &rigExecOps::Vec3fMax(),
                   &rigExecOps::Vec3fWeight()},
                  rigExecOps::Vec3fOut());
        CHECK(h.evaluator);
        if (!h.evaluator)
            continue;
        for (int i = 0; i < 1000; ++i) {
            const Vec3f in = pick3(), value = pick3(), lo = pick3(),
                        hi = pick3();
            const float weight = PickWeight(rng);
            std::string error;
            const bool ran = h.Run({&in, &value, &lo, &hi, &weight}, &error);
            RigExecPropertyMathParams<GfVec3f> params;
            params.op = op;
            params.value = gf(value);
            params.min = gf(lo);
            params.max = gf(hi);
            params.weight = weight;
            const GfVec3f expected = RigExecApplyVec3fMath(gf(in), params);
            CHECK(ran == Finite(expected));
            CHECK(!ran || SameBits(gf(h.Out<Vec3f>()), expected));
        }
    }
    rigExecOps::MathPayload curve;
    curve.op = RigExecPropertyOp::Curve;
    Harness bad(rigExecOps::Vec3fMath(), curve,
                {&rigExecOps::Vec3fIn(), &rigExecOps::Vec3fWeight()},
                rigExecOps::Vec3fOut());
    CHECK(!bad.evaluator && bad.error.find("float-only") != std::string::npos);
}

void
TestMatrix()
{
    std::mt19937 rng(13);
    const auto pick = [&] {
        Mat4d m(1.0);
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                m._mtx[r][c] = double(Pick(rng)) * 1.25;
        return m;
    };
    const auto gf = [](const Mat4d &m) {
        GfMatrix4d out;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                out[r][c] = m._mtx[r][c];
        return out;
    };
    for (const RigExecPropertyOp op :
         {RigExecPropertyOp::Multiply, RigExecPropertyOp::Blend}) {
        rigExecOps::MathPayload payload;
        payload.op = op;
        Harness h(rigExecOps::MatrixMath(), payload,
                  {&rigExecOps::MatrixIn(), &rigExecOps::MatrixValue(),
                   &rigExecOps::MatrixWeight()},
                  rigExecOps::MatrixOut());
        CHECK(h.evaluator);
        if (!h.evaluator)
            continue;
        for (int i = 0; i < 500; ++i) {
            const Mat4d in = pick(), value = pick();
            const float weight = PickWeight(rng);
            std::string error;
            const bool ran = h.Run({&in, &value, &weight}, &error);
            GfMatrix4d expected;
            CHECK(RigExecApplyMatrixMath(gf(in), op, gf(value), weight,
                                         &expected));
            CHECK(ran == Finite(expected));
            CHECK(!ran || SameBits(gf(h.Out<Mat4d>()), expected));
        }
    }
    rigExecOps::MathPayload add;
    add.op = RigExecPropertyOp::Add;
    Harness bad(rigExecOps::MatrixMath(), add,
                {&rigExecOps::MatrixIn(), &rigExecOps::MatrixWeight()},
                rigExecOps::MatrixOut());
    CHECK(!bad.evaluator &&
          bad.error.find("no matrix meaning") != std::string::npos);
}

/// DoubleMath computes in float on float(in) and widens the result, as a
/// double property chain does.
void
TestDouble()
{
    std::mt19937 rng(17);
    for (const RigExecPropertyOp op : kAllOps) {
        rigExecOps::MathPayload payload;
        payload.op = op;
        Harness h(rigExecOps::DoubleMath(), payload,
                  {&rigExecOps::DoubleIn(), &rigExecOps::DoubleValue(),
                   &rigExecOps::DoubleMin(), &rigExecOps::DoubleMax(),
                   &rigExecOps::DoubleWeight()},
                  rigExecOps::DoubleOut());
        CHECK(h.evaluator);
        if (!h.evaluator)
            continue;
        for (int i = 0; i < 2000; ++i) {
            const double in = double(Pick(rng)) * 1.0000001;
            const float value = Pick(rng), lo = Pick(rng), hi = Pick(rng),
                        weight = PickWeight(rng);
            RigExecPropertyMathParams<float> params;
            params.op = op;
            params.value = value;
            params.min = lo;
            params.max = hi;
            params.weight = weight;
            const float expected = RigExecApplyFloatMath(float(in), params);
            std::string error;
            const bool ran = h.Run({&in, &value, &lo, &hi, &weight}, &error);
            CHECK(ran == Finite(expected));
            CHECK(!ran || SameBits(h.Out<double>(), double(expected)));
        }
    }
}

/// A disabled node passes its input through whatever its weight; an
/// enabled one with an unusable weight or operand fails the evaluation.
void
TestEnabledAndErrors()
{
    rigExecOps::MathPayload payload;
    payload.op = RigExecPropertyOp::Add;
    const uint8_t off = 0, on = 1;
    const float bad = 5.0f, nan = std::nanf(""), one = 1.0f;
    {
        Harness h(rigExecOps::FloatMath(), payload,
                  {&rigExecOps::FloatIn(), &rigExecOps::FloatValue(),
                   &rigExecOps::FloatWeight(), &rigExecOps::FloatEnabled()},
                  rigExecOps::FloatOut());
        const float in = -0.0f;
        std::string error;
        CHECK(h.evaluator && h.Run({&in, &nan, &bad, &off}, &error) &&
              SameBits(h.Out<float>(), in));
        CHECK(!h.Run({&in, &one, &bad, &on}, &error) &&
              error.find("weight must be finite") != std::string::npos);
        CHECK(!h.Run({&in, &nan, &one, &on}, &error) &&
              error.find("not finite") != std::string::npos);
        const float huge = 3e38f;
        CHECK(!h.Run({&huge, &huge, &one, &on}, &error) &&
              error.find("non-finite value") != std::string::npos);
    }
    {
        Harness h(rigExecOps::DoubleMath(), payload,
                  {&rigExecOps::DoubleIn(), &rigExecOps::DoubleWeight(),
                   &rigExecOps::DoubleEnabled()},
                  rigExecOps::DoubleOut());
        const double in = 0.1;
        std::string error;
        CHECK(h.evaluator && h.Run({&in, &bad, &off}, &error) &&
              SameBits(h.Out<double>(), in));
    }
    {
        Harness h(rigExecOps::Vec3fMath(), payload,
                  {&rigExecOps::Vec3fIn(), &rigExecOps::Vec3fWeight(),
                   &rigExecOps::Vec3fEnabled()},
                  rigExecOps::Vec3fOut());
        const Vec3f in(1.0f, -0.0f, 2.0f);
        std::string error;
        CHECK(h.evaluator && h.Run({&in, &bad, &off}, &error) &&
              SameBits(h.Out<Vec3f>(), in));
        CHECK(!h.Run({&in, &bad, &on}, &error));
    }
    {
        rigExecOps::MathPayload multiply;
        multiply.op = RigExecPropertyOp::Multiply;
        Harness h(rigExecOps::MatrixMath(), multiply,
                  {&rigExecOps::MatrixIn(), &rigExecOps::MatrixWeight(),
                   &rigExecOps::MatrixEnabled()},
                  rigExecOps::MatrixOut());
        const Mat4d in(2.0);
        std::string error;
        CHECK(h.evaluator && h.Run({&in, &bad, &off}, &error) &&
              SameBits(h.Out<Mat4d>(), in));
        CHECK(!h.Run({&in, &bad, &on}, &error));
    }
}

/// Twelve FloatMath nodes of every operation in one program, run in batches
/// of each size, give each node the bits of its own apply.
void
TestBatches()
{
    using namespace rigExecOps;
    struct Node {
        RigExecPropertyOp op;
        std::vector<RigExecInputId> inputs;
        RigExecOutputId out;
    };
    std::mt19937 rng(29);
    for (const size_t limit : {size_t(0), size_t(1), size_t(3)}) {
        RigExecRigDescBuilder b;
        std::vector<Node> nodes;
        for (size_t i = 0; i < 12; ++i) {
            MathPayload payload;
            payload.op = kAllOps[i % std::size(kAllOps)];
            const RigExecNodeId n = b.AddOp(FloatMath(), &payload);
            Node node{payload.op, {}, {}};
            for (const RigExecPortHandle *port :
                 {&FloatIn(), &FloatValue(), &FloatMin(), &FloatMax(),
                  &FloatWeight()})
                node.inputs.push_back(b.BindInput(n, *port));
            node.out = b.BindOutput(n, FloatOut());
            nodes.push_back(node);
        }
        std::string error;
        auto desc = b.Build(&error);
        RigExecCompileOptions options;
        options.maxBatchNodes = limit;
        auto program = desc ? RigExecCompile(*desc, options, &error) : nullptr;
        auto e =
            program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
        CHECK(e);
        if (!e)
            continue;
        for (int round = 0; round < 50; ++round) {
            std::vector<std::vector<float>> values;
            for (const Node &node : nodes) {
                values.push_back({Pick(rng), Pick(rng), Pick(rng), Pick(rng),
                                  PickWeight(rng)});
                for (size_t k = 0; k < 5; ++k)
                    CHECK(e->SetInput(node.inputs[k], &values.back()[k]));
            }
            std::vector<float> expected;
            bool finite = true;
            for (size_t i = 0; i < nodes.size(); ++i) {
                RigExecPropertyMathParams<float> params;
                params.op = nodes[i].op;
                params.value = values[i][1];
                params.min = values[i][2];
                params.max = values[i][3];
                params.weight = values[i][4];
                expected.push_back(RigExecApplyFloatMath(values[i][0], params));
                finite = finite && Finite(expected.back());
            }
            CHECK(e->Evaluate(&error) == finite);
            for (size_t i = 0; finite && i < nodes.size(); ++i)
                CHECK(SameBits(
                    *static_cast<const float *>(e->GetOutput(nodes[i].out)),
                    expected[i]));
        }
    }
}

/// Add then clamp, wired port to port, is the two applies in sequence.
void
TestChain()
{
    rigExecOps::MathPayload add, clamp;
    add.op = RigExecPropertyOp::Add;
    clamp.op = RigExecPropertyOp::Clamp;
    RigExecRigDescBuilder b;
    const RigExecNodeId a = b.AddOp(rigExecOps::FloatMath(), &add);
    const RigExecNodeId c = b.AddOp(rigExecOps::FloatMath(), &clamp);
    b.Connect(a, rigExecOps::FloatOut(), c, rigExecOps::FloatIn());
    const RigExecInputId in = b.BindInput(a, rigExecOps::FloatIn());
    const RigExecInputId value = b.BindInput(a, rigExecOps::FloatValue());
    const RigExecInputId w0 = b.BindInput(a, rigExecOps::FloatWeight());
    const RigExecInputId lo = b.BindInput(c, rigExecOps::FloatMin());
    const RigExecInputId hi = b.BindInput(c, rigExecOps::FloatMax());
    const RigExecInputId w1 = b.BindInput(c, rigExecOps::FloatWeight());
    const RigExecOutputId out = b.BindOutput(c, rigExecOps::FloatOut());
    std::string error;
    auto desc = b.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto e = program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    CHECK(e);
    if (!e)
        return;
    const float x = 0.75f, v = 2.0f, weight = 1.0f, l = 0.0f, h = 2.5f;
    CHECK(e->SetInput(in, &x) && e->SetInput(value, &v) &&
          e->SetInput(w0, &weight) && e->SetInput(lo, &l) &&
          e->SetInput(hi, &h) && e->SetInput(w1, &weight));
    CHECK(e->Evaluate(&error));
    CHECK(*static_cast<const float *>(e->GetOutput(out)) == 2.5f);
}

} // namespace

int
main()
{
    std::vector<std::string> errors;
    if (!RigExecRegistry::Initialize(RigExecLinkedRegistrations(), &errors)) {
        for (const std::string &e : errors)
            std::printf("%s\n", e.c_str());
        return 1;
    }
    TestFloat();
    TestFloatCurve();
    TestFloatPath();
    TestVec3f();
    TestMatrix();
    TestDouble();
    TestEnabledAndErrors();
    TestChain();
    TestBatches();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
