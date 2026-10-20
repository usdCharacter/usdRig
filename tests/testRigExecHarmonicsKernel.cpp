// harmonicsKernel.h against direct arithmetic: the window length, the ring
// of a single impulse written out term by term, linearity, rest and
// constant-velocity silence, and the start-from-rest rule of a short history.
// Header-only and USD-free.
#include "rigExecMath/harmonicsKernel.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace rigExec;

namespace {

int failures = 0;
constexpr double kPi = 3.14159265358979323846;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

bool Near(double a, double b, double tolerance = 1e-12)
{
    return std::fabs(a - b) <= tolerance * (1.0 + std::fabs(b));
}

using Samples = std::vector<std::array<double, 3>>;

// The offset written as the method describes it, structured differently
// from the kernel: accelerations first, then the ring of each.
void Reference(const RigExecHarmonicsSettings &s,
               const RigExecHarmonicsParameters &p, const Samples &positions,
               double out[3])
{
    const double length = s.waves * s.waveLength;
    const size_t current = positions.size() - 1;
    out[0] = out[1] = out[2] = 0.0;
    for (size_t k = 1; k <= current; ++k) {
        const double phase = double(k) * s.frequency;
        if (!(phase < length)) break;
        const size_t i = current - k;
        if (i == 0 && s.ignoreFirstFrame) break;
        const auto &prev = positions[i == 0 ? 0 : i - 1];
        const auto &here = positions[i];
        const auto &next = positions[i + 1];
        const double envelope =
            (1.0 - p.termination) *
                (std::exp(-p.decay * phase / length) -
                 phase * std::exp(-p.decay) / length) +
            p.termination;
        const double ring =
            std::sin(phase * 2.0 * kPi / s.waveLength) * envelope;
        const double scale = p.matchVelocity
                                 ? p.amplitude * s.waveLength / (2.0 * kPi)
                                 : p.amplitude;
        for (int a = 0; a < 3; ++a) {
            out[a] += -(prev[a] - 2.0 * here[a] + next[a]) * ring * scale *
                      p.axisAmplitude[a];
        }
    }
}

bool Offset(const RigExecHarmonicsSettings &s,
            const RigExecHarmonicsParameters &p, const Samples &positions,
            double out[3])
{
    std::vector<double> flat;
    for (const auto &q : positions) flat.insert(flat.end(), q.begin(), q.end());
    return RigExecHarmonicsOffset(
        s, p, reinterpret_cast<const double(*)[3]>(flat.data()),
        positions.size(), out);
}

void TestWindow()
{
    RigExecHarmonicsSettings s;
    // 5 waves of 5 frames ring for 25 phase frames: k = 1..24.
    CHECK(RigExecHarmonicsRingingCount(s) == 24);
    CHECK(RigExecHarmonicsWindowFrames(s) == 25);
    s.frequency = 2.0;
    CHECK(RigExecHarmonicsRingingCount(s) == 12);
    s.frequency = 0.5;
    CHECK(RigExecHarmonicsRingingCount(s) == 49);
    s.frequency = 3.0;  // 25 / 3: k = 1..8
    CHECK(RigExecHarmonicsRingingCount(s) == 8);
    s.frequency = 0.0;
    CHECK(!RigExecHarmonicsSettingsValid(s));
    CHECK(RigExecHarmonicsWindowFrames(s) == 0);
    s.frequency = 1.0;
    s.waves = 5000.0;
    CHECK(!RigExecHarmonicsSettingsValid(s));
    s.waves = std::nan("");
    CHECK(!RigExecHarmonicsSettingsValid(s));
}

void TestSilence()
{
    RigExecHarmonicsSettings s;
    RigExecHarmonicsParameters p;
    Samples rest(30, {1.0, 2.0, 3.0});
    double out[3] = {9, 9, 9};
    CHECK(Offset(s, p, rest, out));
    CHECK(out[0] == 0.0 && out[1] == 0.0 && out[2] == 0.0);
    // Constant velocity: no acceleration, no ring.
    Samples moving;
    for (int i = 0; i < 30; ++i) moving.push_back({0.5 * i, -0.25 * i, 0.0});
    CHECK(Offset(s, p, moving, out));
    CHECK(Near(out[0], 0.0, 1e-9) && Near(out[1], 0.0, 1e-9) && out[2] == 0.0);
}

void TestImpulseMatchesReference()
{
    RigExecHarmonicsSettings s;
    RigExecHarmonicsParameters p;
    p.amplitude = 0.7;
    p.decay = 2.0;
    p.termination = 0.1;
    p.axisAmplitude[1] = 0.5;
    for (int stepAt : {5, 20, 28}) {
        Samples positions(30, {0.0, 0.0, 0.0});
        for (size_t i = size_t(stepAt); i < positions.size(); ++i)
            positions[i] = {1.0, 2.0, -1.0};
        double got[3], want[3];
        CHECK(Offset(s, p, positions, got));
        Reference(s, p, positions, want);
        for (int a = 0; a < 3; ++a) CHECK(Near(got[a], want[a]));
    }
    // A random walk, with every option moved off its default.
    s.frequency = 1.3;
    s.ignoreFirstFrame = false;
    p.matchVelocity = false;
    Samples walk;
    double x = 0, y = 0, z = 0;
    unsigned seed = 12345;
    for (int i = 0; i < 40; ++i) {
        seed = seed * 1103515245u + 12345u;
        x += double(int(seed >> 16) % 100) / 50.0 - 1.0;
        y += double(int(seed >> 8) % 100) / 70.0 - 0.7;
        z -= 0.1 * i;
        walk.push_back({x, y, z});
    }
    double got[3], want[3];
    CHECK(Offset(s, p, walk, got));
    Reference(s, p, walk, want);
    for (int a = 0; a < 3; ++a) CHECK(Near(got[a], want[a]));
}

void TestLinearAndDeterministic()
{
    RigExecHarmonicsSettings s;
    RigExecHarmonicsParameters p;
    Samples a, b;
    for (int i = 0; i < 30; ++i) {
        a.push_back({std::sin(0.3 * i), std::cos(0.2 * i), 0.01 * i * i});
        b.push_back({3.0 * a.back()[0], 3.0 * a.back()[1], 3.0 * a.back()[2]});
    }
    double oa[3], ob[3], again[3];
    CHECK(Offset(s, p, a, oa));
    CHECK(Offset(s, p, b, ob));
    CHECK(Offset(s, p, a, again));
    for (int k = 0; k < 3; ++k) {
        CHECK(Near(ob[k], 3.0 * oa[k], 1e-10));
        CHECK(std::memcmp(&oa[k], &again[k], sizeof(double)) == 0);
    }
    // Samples older than the window are never read.
    Samples longer(10, {50.0, -50.0, 7.0});
    longer.insert(longer.end(), a.begin(), a.end());
    double ol[3];
    CHECK(Offset(s, p, longer, ol));
    for (int k = 0; k < 3; ++k) CHECK(std::memcmp(&oa[k], &ol[k], sizeof(double)) == 0);
}

void TestShortHistoryStartsFromRest()
{
    RigExecHarmonicsSettings s;
    RigExecHarmonicsParameters p;
    // Three samples, moving from the first: the oldest is the start.
    Samples positions = {{0, 0, 0}, {1, 0, 0}, {3, 0, 0}};
    double kept[3], dropped[3], want[3];
    s.ignoreFirstFrame = false;
    CHECK(Offset(s, p, positions, kept));
    Reference(s, p, positions, want);
    CHECK(Near(kept[0], want[0]));
    s.ignoreFirstFrame = true;
    CHECK(Offset(s, p, positions, dropped));
    Reference(s, p, positions, want);
    CHECK(Near(dropped[0], want[0]));
    CHECK(kept[0] != dropped[0]);
    double none[3];
    CHECK(!Offset(s, p, Samples{{1, 2, 3}}, none));
    CHECK(none[0] == 0.0);
}

}  // namespace

int main()
{
    TestWindow();
    TestSilence();
    TestImpulseMatchesReference();
    TestLinearAndDeterministic();
    TestShortHistoryStartsFromRest();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecHarmonicsKernel: ok\n");
    return 0;
}
