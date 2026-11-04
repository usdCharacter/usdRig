// Driven action kernel: closest whole frame, sides, full turns, tie rules,
// channel values and the weight a blend sample matches. Driver samples
// are linear ramps unless stated, and tie cases use float-representable
// values.
#include "rigExecMath/drivenActionKernel.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace rigExec;

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

namespace {

// samples[k] = value at whole action frame k of a linear ramp 0 -> end
// over `frames` frames, then held to `total` frames (the held tail).
std::vector<double>
Ramp(double end, int frames, int total = -1)
{
    std::vector<double> samples;
    if (total < frames) total = frames;
    for (int k = 0; k <= total; ++k) {
        samples.push_back(k >= frames ? end : end * double(k) / double(frames));
    }
    return samples;
}

// Piecewise linear through (frame, value) keys, sampled at every whole
// frame up to the last key.
std::vector<double>
Keys(const std::vector<std::pair<int, double>> &keys)
{
    std::vector<double> samples;
    for (size_t i = 0; i + 1 < keys.size(); ++i) {
        const int f0 = keys[i].first, f1 = keys[i + 1].first;
        for (int f = f0; f < f1; ++f) {
            const double t = double(f - f0) / double(f1 - f0);
            samples.push_back(keys[i].second + t * (keys[i + 1].second - keys[i].second));
        }
    }
    samples.push_back(keys.back().second);
    return samples;
}

int32_t
Frame(const std::vector<double> &driver, int side, double angle,
      bool tiesLater = false)
{
    const bool full = RigExecDrivenActionFullTurn(driver.data(), driver.size());
    return RigExecDrivenActionFrame(driver.data(), driver.size(), side, full,
                                   tiesLater, angle);
}

int
Side(const std::vector<double> &driver, bool *disagree = nullptr)
{
    bool scratch = false;
    return RigExecDrivenActionSide(driver.data(), driver.size(),
                                  disagree ? disagree : &scratch);
}

void
TestMappingAndSteps()
{
    // "D1": 0 -> +100 over 100 frames; whole steps, clamp past the end.
    const std::vector<double> d1 = Ramp(100.0, 100);
    CHECK(Side(d1) == 1);
    const double angles[] = {0.0, 10.0, 25.3, 25.6, 99.4, 130.0};
    const int32_t frames[] = {0, 10, 25, 26, 99, 100};
    for (size_t i = 0; i < 6; ++i) {
        const int32_t got = Frame(d1, 1, angles[i]);
        if (got != frames[i]) {
            ++failures;
            std::printf("FAIL D1 at %g: frame %d, expected %d\n", angles[i],
                        got, frames[i]);
        }
    }
    // Negative angles are the other action's side.
    CHECK(Frame(d1, 1, -20.0) == 0);
    CHECK(Frame(d1, 1, -80.0) == 0);
    // Non-finite angles and degenerate tables stay at frame 0.
    CHECK(Frame(d1, 1, std::nan("")) == 0);
    CHECK(RigExecDrivenActionFrame(d1.data(), 1, 1, false, false, 50.0) == 0);
    CHECK(RigExecDrivenActionFrame(nullptr, 0, 1, false, false, 50.0) == 0);
}

void
TestSides()
{
    // "D1 2": 0 -> -50 over 50, the negative side.
    const std::vector<double> d12 = Ramp(-50.0, 50);
    CHECK(Side(d12) == -1);
    CHECK(Frame(d12, -1, -20.0) == 20);
    CHECK(Frame(d12, -1, -80.0) == 50);
    CHECK(Frame(d12, -1, 20.0) == 0);
    CHECK(Frame(d12, -1, 0.0) == 0);

    // The authored side wins: "D1 2" keyed in the same direction as "D1"
    // (0 -> +50) but authored negative. Positive angles give 0; negative
    // ones search its samples and clamp to 0.
    const std::vector<double> same = Ramp(50.0, 50);
    CHECK(Frame(same, -1, 30.0) == 0);
    CHECK(Frame(same, -1, -30.0) == 0);
    // fromSamples: the same samples serve the positive side.
    CHECK(Side(same) == 1);
    CHECK(Frame(same, Side(same), 30.0) == 30);

    // "D13" dips first (0, -5 at 1, +50 at 50): the last sample decides,
    // and the disagreement is reported.
    const std::vector<double> d13 = Keys({{0, 0.0}, {1, -5.0}, {50, 50.0}});
    bool disagree = false;
    CHECK(Side(d13, &disagree) == 1);
    CHECK(disagree);
    // On + the closest sample to 30 is frame 32.
    CHECK(Frame(d13, 1, 30.0) == 32);
    CHECK(Frame(d13, 1, -3.0) == 0);
    // A driver that never leaves sample 0 serves no side.
    const std::vector<double> flat(5, 0.0);
    CHECK(Side(flat, &disagree) == 0 && !disagree);
    CHECK(Frame(flat, 0, 10.0) == 0);
    // One test at rest: the other decides, with no disagreement.
    CHECK(Side({0.0, 0.0, 4.0}, &disagree) == 1 && !disagree);
    CHECK(Side({0.0, -2.0, 0.0}, &disagree) == -1 && !disagree);
}

void
TestRange()
{
    // Last key: "D12" sampled to its driver's last key (frame 50).
    const std::vector<double> ra = Ramp(50.0, 50);
    CHECK(Frame(ra, 1, 30.0) == 30);
    CHECK(Frame(ra, 1, 50.0) == 50);
    CHECK(Frame(ra, 1, 60.0) == 50);
    // Duration: sampled to the action's duration (100), a held tail of 50s.
    const std::vector<double> rb = Ramp(50.0, 50, 100);
    CHECK(rb.size() == 101);
    CHECK(Frame(rb, 1, 30.0, false) == 30 && Frame(rb, 1, 30.0, true) == 30);
    CHECK(Frame(rb, 1, 50.0, false) == 50 && Frame(rb, 1, 50.0, true) == 100);
    CHECK(Frame(rb, 1, 60.0, false) == 50 && Frame(rb, 1, 60.0, true) == 100);
    // An exact midpoint tie, float-representable.
    const std::vector<double> steps = {0.0, 1.0, 2.0, 3.0};
    CHECK(Frame(steps, 1, 1.5, false) == 1);
    CHECK(Frame(steps, 1, 1.5, true) == 2);
}

void
TestFullTurns()
{
    // "D3": 0 -> +360 over 72 (5 degrees a frame).
    const std::vector<double> d3 = Ramp(360.0, 72);
    CHECK(RigExecDrivenActionFullTurn(d3.data(), d3.size()));
    CHECK(Side(d3) == 1);
    CHECK(Frame(d3, 1, 370.0) == 2);
    CHECK(Frame(d3, 1, 725.0) == 1);
    CHECK(Frame(d3, 1, -10.0) == 70);
    CHECK(Frame(d3, 1, 180.0) == 36);
    CHECK(Frame(d3, 1, 0.0) == 0);
    // Within the 1 degree tolerance it still wraps; past it, not.
    const std::vector<double> near = Ramp(359.25, 72);
    CHECK(RigExecDrivenActionFullTurn(near.data(), near.size()));
    const std::vector<double> short_ = Ramp(358.5, 72);
    CHECK(!RigExecDrivenActionFullTurn(short_.data(), short_.size()));
    CHECK(Frame(short_, 1, 370.0) == 72);

    // "D3 2": 0 -> -360 over 72, the negative side. Rest keeps frame 0.
    const std::vector<double> d32 = Ramp(-360.0, 72);
    CHECK(RigExecDrivenActionFullTurn(d32.data(), d32.size()));
    CHECK(Side(d32) == -1);
    CHECK(Frame(d32, -1, 0.0) == 0);
    CHECK(Frame(d32, -1, -10.0) == 2);
    CHECK(Frame(d32, -1, -180.0) == 36);
    CHECK(Frame(d32, -1, 10.0) == 70);
    CHECK(Frame(d32, -1, -370.0) == 2);
    // Inside [-360, 0] the angle is unchanged; outside it lands in
    // (-360, 0] on the negative side and [0, 360) on the positive one.
    for (double a : {-360.0, -200.0, -0.5, 0.0}) {
        CHECK(RigExecDrivenActionWrapAngle(d32.data(), d32.size(), -1, a) == a);
    }
    CHECK(RigExecDrivenActionWrapAngle(d32.data(), d32.size(), -1, 10.0) == -350.0);
    CHECK(RigExecDrivenActionWrapAngle(d32.data(), d32.size(), -1, -725.0) == -5.0);
    CHECK(RigExecDrivenActionWrapAngle(d3.data(), d3.size(), 1, -10.0) == 350.0);
    CHECK(RigExecDrivenActionWrapAngle(d3.data(), d3.size(), 1, 720.0) == 0.0);
    CHECK(RigExecDrivenActionWrapAngle(d3.data(), d3.size(), 0, -10.0) == -10.0);
    for (double a : {-1e6, -725.0, -360.0, -0.25, 360.25, 1081.0, 1e6}) {
        const double up = RigExecDrivenActionWrapAngle(d3.data(), d3.size(), 1, a);
        const double down = RigExecDrivenActionWrapAngle(d32.data(), d32.size(), -1, a);
        CHECK(up >= 0.0 && up <= 360.0);
        CHECK(down >= -360.0 && down <= 0.0);
    }
}

void
TestNonMonotonic()
{
    // "D4": 0, +40 at 50, +20 at 100: the globally closest value wins.
    const std::vector<double> d4 = Keys({{0, 0.0}, {50, 40.0}, {100, 20.0}});
    CHECK(Frame(d4, 1, 30.0) == 75);
    CHECK(Frame(d4, 1, 45.0) == 50);
    // "D6": 0, +30 at 10, +90 at 20: the curve is inverted, not the range.
    const std::vector<double> d6 = Keys({{0, 0.0}, {10, 30.0}, {20, 90.0}});
    CHECK(Frame(d6, 1, 30.0) == 10);
}

void
TestChannelsAndWeights()
{
    const double samples[] = {0.25, 0.5, 1.0, 2.0};
    // Offset subtracts sample 0; value is the sample; past the end holds.
    CHECK(RigExecActionChannelValue(samples, 4, 0, true) == 0.0);
    CHECK(RigExecActionChannelValue(samples, 4, 2, true) == 0.75);
    CHECK(RigExecActionChannelValue(samples, 4, 2, false) == 1.0);
    CHECK(RigExecActionChannelValue(samples, 4, 9, true) == 1.75);
    CHECK(RigExecActionChannelValue(samples, 4, 9, false) == 2.0);
    CHECK(RigExecActionChannelValue(samples, 4, -3, false) == 0.25);
    CHECK(RigExecActionChannelValue(samples, 0, 2, false) == 0.0);

    // The weight equals float(k) / float(N) bit for bit and increases.
    for (size_t n = 1; n <= 1000; ++n) {
        float previous = -1.0f;
        bool exact = true, increasing = true;
        for (size_t k = 0; k <= n; ++k) {
            const float w = RigExecDrivenActionWeight(int32_t(k), n + 1);
            const float expected = float(k) / float(n);
            exact = exact && std::memcmp(&w, &expected, sizeof w) == 0;
            increasing = increasing && w > previous;
            previous = w;
        }
        if (!exact || !increasing || previous != 1.0f) {
            ++failures;
            std::printf("FAIL weight for N = %zu\n", n);
            break;
        }
    }
    CHECK(RigExecDrivenActionWeight(0, 1) == 0.0f);

    // The whole table: frame, weight, active, then channels.
    RigExecDrivenActionTable table;
    table.driver = Ramp(100.0, 100);
    table.side = 1;
    std::vector<double> q1;
    for (int k = 0; k <= 100; ++k) q1.push_back(double(k) / 100.0);
    table.channelSamples = {q1, {0.0, 1.0, 2.0}};
    table.channelOffset = {1, 0};
    CHECK(table.OutputCount() == 5);
    double out[5];
    RigExecDrivenActionEvaluate(table, 25.6, out);
    CHECK(out[0] == 26.0);
    CHECK(out[1] == double(26.0f / 100.0f));
    CHECK(out[2] == 1.0);
    CHECK(out[3] == q1[26]);
    CHECK(out[4] == 2.0);
    RigExecDrivenActionEvaluate(table, -5.0, out);
    CHECK(out[0] == 0.0 && out[1] == 0.0 && out[2] == 0.0 && out[3] == 0.0 &&
          out[4] == 0.0);

    // Additivity is the channel itself: two actions at +50 each publish
    // their own offset, which add movers sum on the driven channel.
    RigExecDrivenActionTable d2 = table;
    std::vector<double> half;
    for (double v : q1) half.push_back(v * 0.5);
    d2.channelSamples = {half};
    d2.channelOffset = {1};
    CHECK(table.OutputCount() == 5 && d2.OutputCount() == 4);
    double a[5], b[4];
    RigExecDrivenActionEvaluate(table, 50.0, a);
    RigExecDrivenActionEvaluate(d2, 50.0, b);
    CHECK(a[3] + b[3] == 0.75);
}

}  // namespace

int
main()
{
    TestMappingAndSteps();
    TestSides();
    TestRange();
    TestFullTurns();
    TestNonMonotonic();
    TestChannelsAndWeights();
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecDrivenActionKernel: all tests passed\n");
    return 0;
}
