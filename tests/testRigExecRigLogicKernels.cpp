// The RigLogic ports in rigExecMath against RigLogic itself and against
// direct arithmetic: rbfMethodKernel.h and conditionalTableKernel.h against
// reference values OpenRigLogic's own RBFSolver and ConditionalTable produced
// (tests/fixtures/rigLogicReference.inc), and mlpKernel.h against a forward
// pass written out term by term.
//
// RigLogic computes in float and these ports in double, so the reference
// tolerance is float rounding, not bit identity. Header-only and USD-free.
#include "rigExecMath/conditionalTableKernel.h"
#include "rigExecMath/mlpKernel.h"
#include "rigExecMath/rbfMethodKernel.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace rigExec;

namespace {

int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

struct RigLogicRbfCase {
    const char *name;
    int solverType, distanceMethod, weightFunction, normalizeMethod, twistAxis;
    bool automaticRadius;
    double radius, weightThreshold;
    int rawCount;
    std::vector<double> targets, targetScales;
    std::vector<std::vector<double>> inputs, expected;
    double resolvedRadius;
};

struct RigLogicConditionalRow {
    int input, output;
    double from, to, slope, cut;
};

struct RigLogicConditionalCase {
    double inputs[3];
    double outputs[2];
};

#include "fixtures/rigLogicReference.inc"

// Float rounding of RigLogic's single-precision arithmetic. An interpolative
// solve also passes RigLogic's single-precision LU inverse of the kernel
// matrix, whose conditioning amplifies that rounding to a few 1e-4 on these
// cases (worst measured 4.0e-4); TestRbfExactMath pins the double inverse
// itself exactly.
constexpr double kAdditiveTolerance = 2.0e-6;
constexpr double kInterpolativeTolerance = 1.0e-3;

void
TestRbfAgainstRigLogic()
{
    double worst = 0.0;
    std::string worstAt;
    size_t evaluated = 0;
    for (const RigLogicRbfCase &c : kRigLogicRbfCases) {
        RigExecRbfMethodDesc desc;
        desc.solverType = RigExecRbfSolverType(c.solverType);
        desc.distanceMethod = RigExecRbfDistanceMethod(c.distanceMethod);
        desc.weightFunction = RigExecRbfWeightFunction(c.weightFunction);
        desc.normalizeMethod = RigExecRbfNormalizeMethod(c.normalizeMethod);
        desc.twistAxis = c.twistAxis;
        desc.automaticRadius = c.automaticRadius;
        desc.radius = c.radius;
        desc.weightThreshold = c.weightThreshold;
        desc.rawCount = size_t(c.rawCount);
        desc.targets = c.targets;
        desc.targetScales = c.targetScales;
        RigExecRbfMethodTable table;
        CHECK(RigExecRbfMethodBuild(desc, &table));
        CHECK(std::abs(table.radius - c.resolvedRadius) <=
              1.0e-6 * std::max(1.0, c.resolvedRadius));
        const double tolerance =
            desc.solverType == RigExecRbfSolverType::Interpolative
                ? kInterpolativeTolerance : kAdditiveTolerance;
        std::vector<double> scratch(RigExecRbfMethodScratchSize(table));
        std::vector<double> weights(table.targetCount);
        for (size_t i = 0; i < c.inputs.size(); ++i) {
            RigExecRbfMethodEvaluate(table, c.inputs[i].data(),
                                     scratch.data(), weights.data());
            CHECK(weights.size() == c.expected[i].size());
            for (size_t t = 0; t < weights.size(); ++t) {
                const double gap = std::abs(weights[t] - c.expected[i][t]);
                if (gap > worst) {
                    worst = gap;
                    worstAt = std::string(c.name) + " input " +
                              std::to_string(i) + " target " +
                              std::to_string(t);
                }
                if (gap > tolerance) {
                    ++failures;
                    std::printf("FAIL %s input %zu target %zu: %.9g vs "
                                "RigLogic %.9g\n", c.name, i, t, weights[t],
                                c.expected[i][t]);
                }
            }
            ++evaluated;
        }
    }
    CHECK(evaluated > 100);
    std::printf("   RBF: %zu evaluations, worst gap %.3g at %s\n", evaluated,
                worst, worstAt.c_str());
}

// Exact arithmetic RigLogic does not decide: the interpolative solve
// reproduces each target exactly, the swing-twist split recomposes the
// rotation, a twist reads its own angle, and the automatic radius is the
// mean pairwise distance.
void
TestRbfExactMath()
{
    RigExecRbfMethodDesc desc;
    desc.solverType = RigExecRbfSolverType::Interpolative;
    desc.distanceMethod = RigExecRbfDistanceMethod::Quaternion;
    desc.radius = 1.0;
    desc.rawCount = 4;
    const double h = std::sqrt(0.5);
    desc.targets = {0, 0, 0, 1, h, 0, 0, h, 0, h, 0, h, 0, 0, 0.5, std::sqrt(0.75)};
    RigExecRbfMethodTable table;
    CHECK(RigExecRbfMethodBuild(desc, &table));
    std::vector<double> scratch(RigExecRbfMethodScratchSize(table));
    std::vector<double> weights(table.targetCount);
    for (size_t t = 0; t < table.targetCount; ++t) {
        RigExecRbfMethodEvaluate(table, desc.targets.data() + 4 * t,
                                 scratch.data(), weights.data());
        for (size_t k = 0; k < weights.size(); ++k) {
            CHECK(std::abs(weights[k] - (k == t ? 1.0 : 0.0)) < 1e-12);
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        for (const auto &q0 : std::vector<std::vector<double>>{
                 {0.3, -0.5, 0.2, 0.787}, {-0.6, 0.1, 0.7, -0.38},
                 {0.0, 0.9, 0.1, 0.42}}) {
            double n = 0;
            for (double v : q0) n += v * v;
            double q[4], s[4], t[4];
            for (int i = 0; i < 4; ++i) q[i] = s[i] = t[i] = q0[i] / std::sqrt(n);
            rbfMethodDetail::Swing(axis, s);
            rbfMethodDetail::Twist(axis, t);
            CHECK(std::abs(s[axis]) < 1e-15);
            CHECK(t[(axis + 1) % 3] == 0.0 && t[(axis + 2) % 3] == 0.0);
            // Hamilton swing * twist is the rotation, up to sign.
            const double r[4] = {
                s[3] * t[0] + s[0] * t[3] + s[1] * t[2] - s[2] * t[1],
                s[3] * t[1] - s[0] * t[2] + s[1] * t[3] + s[2] * t[0],
                s[3] * t[2] + s[0] * t[1] - s[1] * t[0] + s[2] * t[3],
                s[3] * t[3] - s[0] * t[0] - s[1] * t[1] - s[2] * t[2]};
            const double dot = r[0] * q[0] + r[1] * q[1] + r[2] * q[2] + r[3] * q[3];
            CHECK(std::abs(std::abs(dot) - 1.0) < 1e-12);
        }
    }
    const double angle = 1.9;
    double twist[4] = {0, std::sin(angle / 2), 0, std::cos(angle / 2)};
    rbfMethodDetail::TwistAngleForm(twist);
    CHECK(std::abs(twist[0] - angle) < 1e-12);
    RigExecRbfMethodDesc square;
    square.distanceMethod = RigExecRbfDistanceMethod::Euclidean;
    square.automaticRadius = true;
    square.rawCount = 2;
    square.targets = {0, 0, 1, 0, 0, 1, 1, 1};
    CHECK(RigExecRbfMethodBuild(square, &table));
    CHECK(std::abs(table.radius - (4.0 + 2.0 * std::sqrt(2.0)) / 6.0) < 1e-15);
}

void
TestRbfBuildRefusals()
{
    RigExecRbfMethodTable table;
    RigExecRbfMethodDesc desc;
    desc.rawCount = 4;
    desc.targets = {0, 0, 0, 1, 0, 0};
    CHECK(!RigExecRbfMethodBuild(desc, &table));  // a partial target
    desc.targets = {0, 0, 0, 1, 0, 0, 0, 1};
    desc.targetScales = {1};
    CHECK(!RigExecRbfMethodBuild(desc, &table));  // one scale for two
    desc.targetScales.clear();
    desc.rawCount = 2;
    desc.targets = {0, 0, 1, 1};
    CHECK(!RigExecRbfMethodBuild(desc, &table));  // quaternions in pairs
    desc.distanceMethod = RigExecRbfDistanceMethod::Euclidean;
    CHECK(RigExecRbfMethodBuild(desc, &table));
    CHECK(table.coefficients.empty() && table.distanceTargets.empty());
    // A zero radius leaves only an exact match with weight, never a NaN.
    desc.radius = 0.0;
    CHECK(RigExecRbfMethodBuild(desc, &table));
    const double exact[2] = {1, 1}, near[2] = {0.9, 1};
    std::vector<double> scratch(RigExecRbfMethodScratchSize(table));
    double weights[2] = {-1, -1};
    RigExecRbfMethodEvaluate(table, exact, scratch.data(), weights);
    CHECK(weights[0] == 0.0 && weights[1] == 1.0);
    RigExecRbfMethodEvaluate(table, near, scratch.data(), weights);
    CHECK(weights[0] == 0.0 && weights[1] == 0.0);
}

void
TestConditionalTableAgainstRigLogic()
{
    // RigExec keeps one table per map: split RigLogic's rows by output,
    // in order.
    std::vector<RigExecConditionalRow> rows[2];
    for (const RigLogicConditionalRow &r : kRigLogicConditionalRows) {
        rows[r.output].push_back({r.input, r.from, r.to, r.slope, r.cut});
    }
    std::vector<size_t> groupEnd[2];
    for (int m = 0; m < 2; ++m) {
        CHECK(RigExecConditionalTablePrepare(&rows[m], &groupEnd[m], 3));
    }
    for (const RigLogicConditionalCase &c : kRigLogicConditionalCases) {
        for (int m = 0; m < 2; ++m) {
            const double value = RigExecConditionalTableEvaluate(
                rows[m].data(), groupEnd[m].data(), rows[m].size(), c.inputs);
            if (std::abs(value - c.outputs[m]) > 1.0e-6) {
                ++failures;
                std::printf("FAIL conditional map %d at (%g, %g, %g): %.9g "
                            "vs RigLogic %.9g\n", m, c.inputs[0], c.inputs[1],
                            c.inputs[2], value, c.outputs[m]);
            }
        }
    }
    // A reversed interval is swapped, and an input outside the row's range
    // refuses the table.
    std::vector<RigExecConditionalRow> reversed = {{0, 1.0, 0.0, 1.0, 0.0}};
    std::vector<size_t> ends;
    CHECK(RigExecConditionalTablePrepare(&reversed, &ends, 1));
    CHECK(reversed[0].from == 0.0 && reversed[0].to == 1.0);
    const double half = 0.5, nan = std::nan("");
    CHECK(RigExecConditionalTableEvaluate(reversed.data(), ends.data(), 1,
                                          &half) == 0.5);
    CHECK(RigExecConditionalTableEvaluate(reversed.data(), ends.data(), 1,
                                          &nan) == 0.0);
    std::vector<RigExecConditionalRow> outside = {{2, 0.0, 1.0, 1.0, 0.0}};
    CHECK(!RigExecConditionalTablePrepare(&outside, &ends, 2));
}

double
Sigmoid(double x)
{
    return 1.0 / (1.0 + std::exp(-x));
}

void
TestMlp()
{
    // 2 inputs -> 3 (leaky relu, alpha 0.1) -> 2 (tanh) -> 2 (linear), with
    // input normalization and defaults.
    RigExecMlNetwork net;
    net.inputCount = 2;
    // Exactly representable in float, so float storage changes nothing.
    const std::vector<float> w1 = {0.5f, -1.0f, 2.0f, 0.25f, -0.75f, 1.5f};
    const std::vector<float> b1 = {0.125f, -0.25f, 0.375f};
    const std::vector<float> w2 = {1.0f, 0.5f, -0.5f, -1.0f, 0.25f, 0.75f};
    const std::vector<float> b2 = {0.0f, 0.125f};
    const std::vector<float> w3 = {2.0f, -1.0f, 0.5f, 0.5f};
    const std::vector<float> b3 = {0.0625f, -0.0625f};
    const auto append = [&](const std::vector<float> &v) {
        const size_t at = net.parameters.size();
        net.parameters.insert(net.parameters.end(), v.begin(), v.end());
        return at;
    };
    RigExecMlLayer l1;
    l1.inputCount = 2; l1.outputCount = 3;
    l1.activation = RigExecMlActivation::LeakyRelu; l1.alpha = 0.1;
    l1.weightOffset = append(w1); l1.biasOffset = append(b1);
    RigExecMlLayer l2;
    l2.inputCount = 3; l2.outputCount = 2;
    l2.activation = RigExecMlActivation::Tanh;
    l2.weightOffset = append(w2); l2.biasOffset = append(b2);
    RigExecMlLayer l3;
    l3.inputCount = 2; l3.outputCount = 2;
    l3.activation = RigExecMlActivation::Linear;
    l3.weightOffset = append(w3); l3.biasOffset = append(b3);
    net.layers = {l1, l2, l3};
    net.inputMean = {0.5f, -0.5f};
    net.inputScale = {2.0f, 0.5f};
    net.defaults = {0.0f, 1.0f};
    std::string error;
    CHECK(RigExecMlNetworkValid(net, &error));
    CHECK(net.OutputCount() == 2 && net.Width() == 3);

    const double input[2] = {0.8, 0.3};
    const double x[2] = {(0.8 - 0.5) * 2.0, (0.3 + 0.5) * 0.5};
    double h1[3], h2[2], y[2];
    for (int r = 0; r < 3; ++r) {
        const double s = w1[2 * r] * x[0] + w1[2 * r + 1] * x[1] + b1[r];
        h1[r] = s < 0 ? 0.1 * s : s;
    }
    for (int r = 0; r < 2; ++r) {
        h2[r] = std::tanh(w2[3 * r] * h1[0] + w2[3 * r + 1] * h1[1] +
                          w2[3 * r + 2] * h1[2] + b2[r]);
    }
    for (int r = 0; r < 2; ++r) {
        y[r] = w3[2 * r] * h2[0] + w3[2 * r + 1] * h2[1] + b3[r];
    }
    std::vector<double> scratch(RigExecMlScratchSize(net));
    double out[2];
    RigExecMlEvaluate(net, input, 1.0, scratch.data(), out);
    CHECK(std::abs(out[0] - y[0]) < 1e-12 && std::abs(out[1] - y[1]) < 1e-12);
    RigExecMlEvaluate(net, input, 0.25, scratch.data(), out);
    CHECK(std::abs(out[0] - 0.25 * y[0]) < 1e-12 &&
          std::abs(out[1] - 0.25 * y[1]) < 1e-12);
    // Mask zero is RigLogic's switch to the defaults, not a zero scale.
    RigExecMlEvaluate(net, input, 0.0, scratch.data(), out);
    CHECK(out[0] == 0.0 && out[1] == 1.0);

    // Every activation, against its definition.
    CHECK(RigExecMlActivate(RigExecMlActivation::Relu, -2.0, 0) == 0.0);
    CHECK(RigExecMlActivate(RigExecMlActivation::Relu, 2.0, 0) == 2.0);
    CHECK(RigExecMlActivate(RigExecMlActivation::LeakyRelu, -2.0, 0.2) ==
          -0.4);
    CHECK(RigExecMlActivate(RigExecMlActivation::Sigmoid, 0.3, 0) ==
          Sigmoid(0.3));
    CHECK(RigExecMlActivate(RigExecMlActivation::Linear, -7.5, 0) == -7.5);

    // Refusals: widths that do not chain, parameters past the end.
    RigExecMlNetwork bad = net;
    bad.layers[1].inputCount = 2;
    CHECK(!RigExecMlNetworkValid(bad, &error));
    bad = net;
    bad.layers[2].biasOffset = net.parameters.size() - 1;
    CHECK(!RigExecMlNetworkValid(bad, &error));
    bad = net;
    bad.defaults = {0.0};
    CHECK(!RigExecMlNetworkValid(bad, &error));

    // The authored form builds the same network: weights, then biases.
    RigExecMlNetwork built;
    std::vector<float> weights, biases;
    for (const auto *w : {&w1, &w2, &w3}) weights.insert(weights.end(), w->begin(), w->end());
    for (const auto *b : {&b1, &b2, &b3}) biases.insert(biases.end(), b->begin(), b->end());
    CHECK(RigExecMlBuildNetwork(2, {3, 2, 2}, {2, 3, 0}, {0.1f, 0.0f, 0.0f},
                                weights, biases, &built, &error));
    built.inputMean = net.inputMean;
    built.inputScale = net.inputScale;
    built.defaults = net.defaults;
    double again[2];
    RigExecMlEvaluate(built, input, 1.0, scratch.data(), again);
    CHECK(std::abs(again[0] - y[0]) < 1e-6 && std::abs(again[1] - y[1]) < 1e-6);
    CHECK(!RigExecMlBuildNetwork(2, {3, 2}, {2, 3, 0}, {}, weights, biases,
                                 &built, &error));
    CHECK(!RigExecMlBuildNetwork(2, {3, 2, 2}, {2, 3, 0}, {}, weights,
                                 std::vector<float>(biases.begin(), biases.end() - 1),
                                 &built, &error));
}

} // namespace

int
main()
{
    TestRbfAgainstRigLogic();
    TestRbfExactMath();
    TestRbfBuildRefusals();
    TestConditionalTableAgainstRigLogic();
    TestMlp();
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecRigLogicKernels: all tests passed\n");
    return 0;
}
