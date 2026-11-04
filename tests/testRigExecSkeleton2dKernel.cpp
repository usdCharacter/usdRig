// 2D skeleton solve: composition, independent angle, flips, cross scale,
// limits, locks, tips, IK targets, control bones, groups, space-switch
// parent spaces (bones, the base, outside spaces, fractional blends) and the
// start-frame plane (rigExecMath/skeleton2dKernel.h), USD-free. Expected
// values are synthetic reference numbers; the lock-within-limits knee angles
// are the closed-form two-bone bends that put the foot on its lock point
// (law of cosines).
#include "rigExecMath/skeleton2dKernel.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace rigExec;

namespace {

int _failures = 0;
std::string _context;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "FAIL [%s] %s:%d: %s\n", _context.c_str(),  \
                         __FILE__, __LINE__, #cond);                         \
            ++_failures;                                                     \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        const double _a = (a), _b = (b);                                     \
        if (!(std::fabs(_a - _b) <= (tol))) {                                \
            std::fprintf(stderr, "FAIL [%s] %s:%d: %s = %.12g, want %.12g "  \
                         "(tol %g)\n", _context.c_str(), __FILE__, __LINE__, \
                         #a, _a, _b, double(tol));                           \
            ++_failures;                                                     \
        }                                                                    \
    } while (0)

constexpr double kDeg = 3.14159265358979323846 / 180.0;

struct Rig {
    std::vector<RigExecBone2dSetup> setup;
    std::vector<RigExecBone2dRest> rests;
    std::vector<RigExecBone2dChannels> channels;
    std::vector<RigExecBone2dFrame> pose, deform;
    RigExecSkeleton2dWorkspace ws;

    /// Adds a bone whose setup base is \p x, \p y and world angle \p angle.
    /// The first child of a bone is its tip, as the compiler sets it.
    size_t Add(int parent, double x, double y, double angle)
    {
        RigExecBone2dSetup s;
        s.parent = parent;
        setup.push_back(s);
        rests.push_back({x, y, angle});
        channels.emplace_back();
        const size_t added = setup.size() - 1;
        if (parent >= 0 && !(setup[size_t(parent)].flags & kRigExecBone2dTip)) {
            setup[size_t(parent)].flags |= kRigExecBone2dTip;
            setup[size_t(parent)].tipBone = int32_t(added);
        }
        return added;
    }
    /// Gives leaf \p i an End joint \p length along its +X.
    void End(size_t i, double length)
    {
        setup[i].flags |= kRigExecBone2dTip;
        setup[i].tipBone = -1;
        setup[i].tipX = length;
        setup[i].tipY = 0;
    }
    /// Adds a bone at the tip of \p parent (length \p parentLength),
    /// turned \p localAngle from it.
    size_t AddAtTip(size_t parent, double parentLength, double localAngle)
    {
        const RigExecBone2dRest &p = rests[parent];
        return Add(int(parent), p.x + parentLength * std::cos(p.angle * kDeg),
                   p.y + parentLength * std::sin(p.angle * kDeg),
                   p.angle + localAngle);
    }
    bool Solve()
    {
        pose.assign(setup.size(), RigExecBone2dFrame());
        deform.assign(setup.size(), RigExecBone2dFrame());
        return RigExecSolveSkeleton2d(setup.data(), rests.data(),
                                      channels.data(), setup.size(), &ws,
                                      pose.data(), deform.data());
    }
    /// Outside parent spaces (rest, pose) and the selectors.
    std::vector<RigExecBone2dRest> externalRests, externalPoses;
    std::vector<double> selectors;
    bool SolveEx(const RigExecSkeleton2dStatic *extra)
    {
        pose.assign(setup.size(), RigExecBone2dFrame());
        deform.assign(setup.size(), RigExecBone2dFrame());
        RigExecSkeleton2dFrameInputs inputs;
        inputs.externalRests = externalRests.data();
        inputs.externalPoses = externalPoses.data();
        inputs.externalCount = externalRests.size();
        inputs.selectors = selectors.data();
        inputs.selectorCount = selectors.size();
        return RigExecSolveSkeleton2d(setup.data(), rests.data(),
                                      channels.data(), setup.size(), &ws,
                                      pose.data(), deform.data(), extra,
                                      inputs);
    }
    double WorldAngle(size_t i) const
    {
        return std::atan2(pose[i].xAxisY, pose[i].xAxisX) / kDeg;
    }
    double LocalAngle(size_t i) const
    {
        const int p = setup[i].parent;
        double a = WorldAngle(i) - (p < 0 ? 0.0 : WorldAngle(size_t(p)));
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
    /// The image of setup-space point (x, y) under bone i's deformation,
    /// deform_i * rest_i^-1 (a rigidly bound marker).
    void Map(size_t i, double x, double y, double *ox, double *oy) const
    {
        const RigExecBone2dRest &r = rests[i];
        const double c = std::cos(r.angle * kDeg), s = std::sin(r.angle * kDeg);
        const double dx = x - r.x, dy = y - r.y;
        const double u = dx * c + dy * s, v = -dx * s + dy * c;
        const RigExecBone2dFrame &f = deform[i];
        *ox = f.originX + u * f.xAxisX + v * f.yAxisX;
        *oy = f.originY + u * f.xAxisY + v * f.yAxisY;
    }
    bool SameFrames(const Rig &o) const
    {
        return pose.size() == o.pose.size() &&
               std::memcmp(pose.data(), o.pose.data(),
                           pose.size() * sizeof(RigExecBone2dFrame)) == 0 &&
               std::memcmp(deform.data(), o.deform.data(),
                           deform.size() * sizeof(RigExecBone2dFrame)) == 0;
    }
};

bool _IsRestFrame(const RigExecBone2dFrame &f, const RigExecBone2dRest &r)
{
    return std::fabs(f.originX - r.x) < 1e-12 && std::fabs(f.originY - r.y) < 1e-12 &&
           std::fabs(f.xAxisX - std::cos(r.angle * kDeg)) < 1e-12 &&
           std::fabs(f.xAxisY - std::sin(r.angle * kDeg)) < 1e-12;
}

// Independent angle keeps the setup world angle plus own rz.
void TestIndependentAngle()
{
    _context = "independent angle";
    for (const bool independent : {true, false}) {
        Rig rig;
        const size_t q = rig.Add(-1, 0, 0.6, 20);
        const size_t f = rig.AddAtTip(q, 0.3, 30);
        if (independent) rig.setup[f].flags |= kRigExecBone2dIndependentAngle;
        const double keysQ[4] = {20, 60, 60, -10};
        const double keysF[4] = {30, 30, 50, 30};
        const double expected[4] = {50, 50, 70, 50};
        for (int frame = 0; frame < 4; ++frame) {
            rig.channels[q].rz = keysQ[frame] - 20;
            rig.channels[f].rz = keysF[frame] - 30;
            CHECK(rig.Solve());
            if (independent) {
                CHECK_NEAR(rig.WorldAngle(f), expected[frame], 1e-9);
            } else if (frame == 1) {
                CHECK_NEAR(rig.WorldAngle(f), 90, 1e-9);
            }
            // F's base on Q's tip throughout.
            const auto &pq = rig.pose[q];
            CHECK_NEAR(rig.pose[f].originX, pq.originX + 0.3 * pq.xAxisX, 1e-12);
            CHECK_NEAR(rig.pose[f].originY, pq.originY + 0.3 * pq.xAxisY, 1e-12);
        }
    }
}

// Limits are relative to the setup local angle; evaluation does
// not clamp keyed values.
void TestAngleLimits()
{
    _context = "limits";
    CHECK(RigExecBone2dClampLocalAngle(30, -10, 40, 90) == 70);
    CHECK(RigExecBone2dClampLocalAngle(30, -10, 40, -30) == 20);
    CHECK(RigExecBone2dClampLocalAngle(30, -10, 40, 45) == 45);
    CHECK(RigExecBone2dClampLocalAngle(30, 40, -10, 90) == 70);
    CHECK(RigExecBone2dClampLocalAngle(0, -10, 40, 60) == 40);
    Rig rig;
    const size_t p = rig.Add(-1, -0.8, 0.6, 20);
    const size_t c = rig.AddAtTip(p, 0.3, 30);
    rig.setup[c].flags |= kRigExecBone2dAngleLimits;
    rig.setup[c].minAngle = -10;
    rig.setup[c].maxAngle = 40;
    // Part B: the local reference survives a turned parent (the clamp
    // takes the setup local angle, not the world angle).
    rig.channels[p].rz = 60;
    CHECK(RigExecBone2dClampLocalAngle(30, -10, 40, 90) == 70);
    // Part C (i): a keyed 90 evaluates to 90.
    rig.channels[p].rz = 0;
    rig.channels[c].rz = 60;
    CHECK(rig.Solve());
    CHECK_NEAR(rig.LocalAngle(c), 90, 1e-9);
}

// A locked foot holds its base and world angle while the hip
// moves; out of reach, the legs straighten and the foot lifts.
void TestLock()
{
    _context = "lock";
    Rig rig;
    const size_t h = rig.Add(-1, 0, 0.3, -90);
    const size_t t = rig.AddAtTip(h, 0.05, 5);
    const size_t s = rig.AddAtTip(t, 0.4, -10);
    const size_t f = rig.AddAtTip(s, 0.4, 95);
    rig.setup[s].flags |= kRigExecBone2dAngleLimits;
    rig.setup[s].minAngle = -140;
    rig.setup[s].maxAngle = 10;
    CHECK(rig.Solve());
    const double lockX = rig.pose[f].originX, lockY = rig.pose[f].originY;
    const double lockAngle = rig.WorldAngle(f);
    CHECK_NEAR(lockAngle, 0, 1e-9);
    rig.channels[f].lock = true;
    rig.channels[f].lockX = lockX;
    rig.channels[f].lockY = lockY;
    rig.channels[f].lockAngle = lockAngle;
    for (int frame = 6; frame <= 9; ++frame) {
        _context = "lock frame " + std::to_string(frame);
        rig.channels[h].tx = 0.05 * (frame - 5);
        rig.channels[h].ty = -0.03 * (frame - 5);
        CHECK(rig.Solve());
        CHECK_NEAR(rig.pose[f].originX, lockX, 1e-3);
        CHECK_NEAR(rig.pose[f].originY, lockY, 1e-3);
        CHECK_NEAR(rig.WorldAngle(f), lockAngle, 1e-3);
        CHECK(rig.LocalAngle(s) >= -150 - 1e-9 && rig.LocalAngle(s) <= 1e-9);
        // The hip keeps its keyed placement (a root is never turned).
        CHECK_NEAR(rig.WorldAngle(h), -90, 1e-12);
    }
    _context = "lock frame 10";
    rig.channels[h].tx = 0.2;
    rig.channels[h].ty = -0.12 + 0.5;
    CHECK(rig.Solve());
    const double dx = rig.pose[f].originX - lockX, dy = rig.pose[f].originY - lockY;
    CHECK(std::sqrt(dx * dx + dy * dy) > 0.1);  // the foot lifts
    // The legs straighten: the thigh-to-foot distance is the full leg.
    const double lx = rig.pose[f].originX - rig.pose[t].originX;
    const double ly = rig.pose[f].originY - rig.pose[t].originY;
    CHECK_NEAR(std::sqrt(lx * lx + ly * ly), 0.8, 2e-3);
    CHECK(rig.LocalAngle(s) >= -150 - 1e-9 && rig.LocalAngle(s) <= 1e-9);
    CHECK_NEAR(rig.WorldAngle(f), lockAngle, 1e-9);  // its angle still holds

    // A lock on a root does nothing (only parented bones lock).
    _context = "lock on a root";
    Rig root;
    const size_t r = root.Add(-1, 0, 0, 0);
    root.channels[r].lock = true;
    root.channels[r].lockX = 5;
    root.channels[r].lockAngle = 45;
    CHECK(root.Solve());
    CHECK(root.pose[r].originX == 0 && root.WorldAngle(r) == 0);
}

// The lock solve never turns a bone past its limits. Leg rig
// (S limited to [-150, 0]: setup local -10 plus [-140, 10]), the foot
// locked at its setup pose. Each hip offset is solved with and without
// S's limits; the unlimited solve bends S forward, outside the limits, so
// the limited checks fail if the solve stops clamping. Only the limits are
// asserted for the limited solve: whether it reaches the backward bend at
// (-0.30, -0.30) or stops at S's bound depends on rounding in the setup.
void TestLockWithinLimits()
{
    struct Case { double tx, ty, unlimitedKnee; };
    const Case cases[] = {
        {-0.15, -0.10, 53.9655},
        {-0.30, -0.30, 86.9610},
    };
    for (const Case &c : cases) {
        for (const bool limited : {true, false}) {
            _context = "lock within limits, hip (" + std::to_string(c.tx) + ", " +
                       std::to_string(c.ty) + (limited ? "), limited" : "), unlimited");
            Rig rig;
            const size_t h = rig.Add(-1, 0, 0.3, -90);
            const size_t t = rig.AddAtTip(h, 0.05, 5);
            const size_t s = rig.AddAtTip(t, 0.4, -10);
            const size_t f = rig.AddAtTip(s, 0.4, 95);
            if (limited) {
                rig.setup[s].flags |= kRigExecBone2dAngleLimits;
                rig.setup[s].minAngle = -140;
                rig.setup[s].maxAngle = 10;
            }
            CHECK(rig.Solve());
            const double lockX = rig.pose[f].originX, lockY = rig.pose[f].originY;
            rig.channels[f].lock = true;
            rig.channels[f].lockX = lockX;
            rig.channels[f].lockY = lockY;
            rig.channels[f].lockAngle = rig.WorldAngle(f);
            rig.channels[h].tx = c.tx;
            rig.channels[h].ty = c.ty;
            CHECK(rig.Solve());
            const double knee = rig.LocalAngle(s);
            if (limited) {
                CHECK(knee >= -150 - 1e-9 && knee <= 1e-9);
            } else {
                CHECK(knee > 0);
                CHECK_NEAR(knee, c.unlimitedKnee, 1e-3);
                CHECK_NEAR(std::hypot(rig.pose[f].originX - lockX,
                                      rig.pose[f].originY - lockY), 0, 1e-6);
            }
            CHECK_NEAR(rig.WorldAngle(f), 0, 1e-9);
        }
    }
}

// Out of reach, one common scale up to maxStretch.
void TestMaxStretch()
{
    for (const double maxStretch : {1.5, 1.0}) {
        _context = "maxStretch " + std::to_string(maxStretch);
        Rig rig;
        const size_t root = rig.Add(-1, 0, 0, 0);
        const size_t a = rig.AddAtTip(root, 0.1, -90);   // (0.1, 0) pointing down
        const size_t b = rig.AddAtTip(a, 0.5, 20);
        const size_t k = rig.AddAtTip(b, 0.5, 0);
        rig.setup[k].maxStretch = maxStretch;
        rig.channels[k].lock = true;
        rig.channels[k].lockX = 0.1;
        rig.channels[k].lockY = -1.25;
        rig.channels[k].lockAngle = -90;
        CHECK(rig.Solve());
        if (maxStretch > 1) {
            CHECK_NEAR(rig.pose[k].originX, 0.1, 1e-6);
            CHECK_NEAR(rig.pose[k].originY, -1.25, 1e-6);
            CHECK_NEAR(rig.ws.scale[a], 1.25, 1e-12);
            CHECK_NEAR(rig.ws.scale[b], 1.25, 1e-12);
            // Normal scaling: stretched along, never across.
            CHECK_NEAR(std::hypot(rig.deform[a].xAxisX, rig.deform[a].xAxisY), 1.25, 1e-12);
            CHECK_NEAR(std::hypot(rig.deform[a].yAxisX, rig.deform[a].yAxisY), 1.0, 1e-12);
        } else {
            // Straight toward the goal, the foot short of it.
            CHECK_NEAR(rig.pose[k].originX, 0.1, 1e-6);
            CHECK_NEAR(rig.pose[k].originY, -1.0, 1e-6);
        }
        CHECK_NEAR(rig.WorldAngle(k), -90, 1e-9);
    }
}

// cross scale per scaling mode.
void TestCrossScale()
{
    _context = "cross scale";
    struct Mode { RigExecBone2dScaling scaling; double k; double at2; double atHalf; };
    const Mode modes[] = {
        {RigExecBone2dScaling::Normal, 1, 1, 1},
        {RigExecBone2dScaling::SquashStretch, 1, 0.5, 2},
        {RigExecBone2dScaling::SquashStretch, 0.5, std::pow(2.0, -0.5), std::pow(0.5, -0.5)},
        {RigExecBone2dScaling::SquashStretch, 2, 0.25, 4},
        {RigExecBone2dScaling::Uniform, 1, 2, 0.5},
    };
    for (const Mode &m : modes) {
        CHECK_NEAR(RigExecBone2dCrossScale(m.scaling, 2, m.k), m.at2, 1e-15);
        CHECK_NEAR(RigExecBone2dCrossScale(m.scaling, 0.5, m.k), m.atHalf, 1e-15);
        for (const double s : {2.0, 0.5}) {
            Rig rig;
            const size_t b = rig.Add(-1, -0.2, -0.6, 0);
            rig.setup[b].scaling = m.scaling;
            rig.setup[b].squashFactor = m.k;
            rig.channels[b].sx = s;
            CHECK(rig.Solve());
            const double across = s == 2 ? m.at2 : m.atHalf;
            for (double x = -0.2; x <= 0.2001; x += 0.1) {
                for (double y = -0.7; y <= -0.4999; y += 0.05) {
                    double ox, oy;
                    rig.Map(b, x, y, &ox, &oy);
                    CHECK_NEAR(ox + 0.2, s * (x + 0.2), 1e-12);
                    CHECK_NEAR(oy + 0.6, across * (y + 0.6), 1e-12);
                }
            }
        }
    }
    CHECK(RigExecBone2dCrossScale(RigExecBone2dScaling::SquashStretch, 0, 1) == 1);
    CHECK_NEAR(RigExecBone2dCrossScale(RigExecBone2dScaling::SquashStretch, -2, 1), 0.5, 1e-15);
}

// Flips mirror in the bone's own frame and compose down.
void TestFlips()
{
    _context = "flips";
    auto make = [](Rig *rig, size_t *fp, size_t *fc) {
        *fp = rig->Add(-1, 0.4, -0.2, 0);
        *fc = rig->AddAtTip(*fp, 0.3, 0);
    };
    {
        Rig rig; size_t fp, fc; make(&rig, &fp, &fc);
        rig.channels[fc].sideFlip = true;
        CHECK(rig.Solve());
        double ox, oy;
        rig.Map(fc, 1.0, -0.1, &ox, &oy);  // above FC's tip
        CHECK_NEAR(ox, 1.0, 1e-12);
        CHECK_NEAR(oy, -0.3, 1e-12);       // now below FC
    }
    {
        Rig rig; size_t fp, fc; make(&rig, &fp, &fc);
        rig.channels[fc].endFlip = true;
        CHECK(rig.Solve());
        double ox, oy;
        rig.Map(fc, 0.8, -0.2, &ox, &oy);  // on the row along FC
        CHECK_NEAR(ox, 0.6, 1e-12);        // backward from FC's base
        CHECK_NEAR(oy, -0.2, 1e-12);
    }
    {
        // Own side flip: FC's +30 still turns counter-clockwise.
        Rig rig; size_t fp, fc; make(&rig, &fp, &fc);
        rig.channels[fc].sideFlip = true;
        rig.channels[fc].rz = 30;
        CHECK(rig.Solve());
        CHECK_NEAR(rig.WorldAngle(fc), 30, 1e-9);
    }
    {
        // Under a side-flipped parent (odd parity) +30 turns clockwise.
        Rig rig; size_t fp, fc; make(&rig, &fp, &fc);
        rig.channels[fp].sideFlip = true;
        rig.channels[fc].rz = 30;
        CHECK(rig.Solve());
        CHECK_NEAR(rig.WorldAngle(fc), -30, 1e-9);
        CHECK_NEAR(rig.pose[fc].originX, 0.7, 1e-12);
        CHECK_NEAR(rig.pose[fc].originY, -0.2, 1e-12);
        // Two flipped ancestors restore the turn direction.
        Rig two;
        const size_t a = two.Add(-1, 0, 0, 0);
        const size_t b = two.AddAtTip(a, 0.3, 0);
        const size_t c = two.AddAtTip(b, 0.3, 0);
        two.channels[a].sideFlip = true;
        two.channels[b].sideFlip = true;
        two.channels[c].rz = 30;
        CHECK(two.Solve());
        CHECK_NEAR(two.WorldAngle(c), 30, 1e-9);
    }
}

// Parent scale: the parent's scale puts the child on the stretched tip; the
// child's own frame is unscaled.
void TestParentScale()
{
    _context = "parent scale";
    Rig rig;
    const size_t r = rig.Add(-1, -0.8, -0.8, 0);
    const size_t h = rig.AddAtTip(r, 0.3, 0);
    rig.channels[r].sx = 2;
    CHECK(rig.Solve());
    CHECK_NEAR(rig.pose[h].originX - rig.pose[r].originX, 0.6, 1e-12);
    CHECK_NEAR(rig.pose[h].originY - rig.pose[r].originY, 0, 1e-12);
    CHECK_NEAR(std::hypot(rig.deform[h].xAxisX, rig.deform[h].xAxisY), 1, 1e-15);
    CHECK_NEAR(std::hypot(rig.deform[r].xAxisX, rig.deform[r].xAxisY), 2, 1e-15);
    // tx/ty move the bone in its parent's frame (the parent turned 90).
    Rig turned;
    const size_t p = turned.Add(-1, 0, 0, 90);
    const size_t c = turned.AddAtTip(p, 0.3, 0);
    turned.channels[c].tx = 0.2;
    turned.channels[c].ty = 0.1;
    CHECK(turned.Solve());
    CHECK_NEAR(turned.pose[c].originX, -0.1, 1e-12);
    CHECK_NEAR(turned.pose[c].originY, 0.5, 1e-12);
    // At rest every deform frame is the setup frame (D = identity).
    Rig rest;
    const size_t a = rest.Add(-1, 0.3, 0.2, 33);
    const size_t b = rest.AddAtTip(a, 0.7, -71);
    CHECK(rest.Solve());
    CHECK(_IsRestFrame(rest.deform[a], rest.rests[a]));
    CHECK(_IsRestFrame(rest.deform[b], rest.rests[b]));
}

// A rest change between two solves equals a fresh solve with that rest;
// malformed input resets every output to its setup frame.
void TestRestsAndFailures()
{
    _context = "rest refresh";
    Rig rig;
    const size_t a = rig.Add(-1, 0, 0, 10);
    const size_t b = rig.AddAtTip(a, 0.5, 20);
    const size_t c = rig.AddAtTip(b, 0.5, -30);
    rig.channels[a].rz = 15;
    rig.channels[b].sx = 1.5;
    rig.channels[c].lock = true;
    rig.channels[c].lockX = 0.6;
    rig.channels[c].lockY = 0.3;
    CHECK(rig.Solve());
    rig.rests[a] = {0.1, -0.2, 12};
    rig.rests[b] = {0.6, -0.1, 40};
    CHECK(rig.Solve());
    Rig fresh;
    fresh.setup = rig.setup;
    fresh.rests = rig.rests;
    fresh.channels = rig.channels;
    CHECK(fresh.Solve());
    CHECK(rig.SameFrames(fresh));

    _context = "planar rests";
    RigExecBone2dRest rest;
    const double c30 = std::cos(30 * kDeg), s30 = std::sin(30 * kDeg);
    const double o[3] = {0.5, -0.25, 0.75};
    const double x[3] = {0.5 + c30, -0.25 + s30, 0.75};
    const double y[3] = {0.5 - s30, -0.25 + c30, 0.75};
    const double z[3] = {0.5, -0.25, 1.75};
    CHECK(RigExecBone2dRestFromLandmarks(o, x, y, z, &rest));
    CHECK_NEAR(rest.x, 0.5, 1e-15);
    CHECK_NEAR(rest.y, -0.25, 1e-15);
    CHECK_NEAR(rest.angle, 30, 1e-12);
    const double tilted[3] = {0.5 + c30, -0.25 + s30, 0.76};
    CHECK(!RigExecBone2dRestFromLandmarks(o, tilted, y, z, &rest));
    const double scaled[3] = {0.5 + 2 * c30, -0.25 + 2 * s30, 0.75};
    CHECK(!RigExecBone2dRestFromLandmarks(o, scaled, y, z, &rest));
    const double mirrored[3] = {0.5 + s30, -0.25 - c30, 0.75};
    CHECK(!RigExecBone2dRestFromLandmarks(o, x, mirrored, z, &rest));

    _context = "malformed";
    Rig bad;
    bad.Add(-1, 0, 0, 0);
    bad.Add(-1, 1, 0, 45);
    bad.channels[0].rz = 20;
    bad.setup[0].parent = 1;   // a child before its parent
    CHECK(!bad.Solve());
    CHECK(_IsRestFrame(bad.pose[0], bad.rests[0]) && _IsRestFrame(bad.deform[1], bad.rests[1]));
    bad.setup[0].parent = -1;
    bad.channels[1].rz = std::nan("");
    CHECK(!bad.Solve());
    CHECK(_IsRestFrame(bad.deform[0], bad.rests[0]) && _IsRestFrame(bad.deform[1], bad.rests[1]));
    bad.channels[1].rz = 0;
    CHECK(bad.Solve());
    CHECK(RigExecSolveSkeleton2d(nullptr, nullptr, nullptr, 0, nullptr, nullptr, nullptr));
}

// Leg rig of the lock tests: a root hip H, thigh T, shin S, foot F (locked
// at its setup pose); S limited to rz in [lo, hi]. \p sign -1 mirrors the
// leg (setup local angles negated) so the legal bend is the other side.
struct LegRig {
    Rig rig;
    size_t h = 0, t = 0, s = 0, f = 0;
};

LegRig _MakeLeg(double sign, bool limited, double lo, double hi)
{
    LegRig k;
    k.h = k.rig.Add(-1, 0, 0.3, -90);
    k.t = k.rig.AddAtTip(k.h, 0.05, sign * 5);
    k.s = k.rig.AddAtTip(k.t, 0.4, sign * -10);
    k.f = k.rig.AddAtTip(k.s, 0.4, sign * 95);
    if (limited) {
        k.rig.setup[k.s].flags |= kRigExecBone2dAngleLimits;
        k.rig.setup[k.s].minAngle = lo;
        k.rig.setup[k.s].maxAngle = hi;
    }
    return k;
}

/// Locks F at its setup pose (a solve at rest), then moves the hip.
void _LockAndMoveHip(LegRig *k, double tx, double ty)
{
    CHECK(k->rig.Solve());
    RigExecBone2dChannels &f = k->rig.channels[k->f];
    f.lock = true;
    f.lockX = k->rig.pose[k->f].originX;
    f.lockY = k->rig.pose[k->f].originY;
    f.lockAngle = k->rig.WorldAngle(k->f);
    k->rig.channels[k->h].tx = tx;
    k->rig.channels[k->h].ty = ty;
    CHECK(k->rig.Solve());
}

double _LockResidual(const Rig &rig, size_t k)
{
    return std::hypot(rig.pose[k].originX - rig.channels[k].lockX,
                      rig.pose[k].originY - rig.channels[k].lockY);
}

/// \p rig's pose and deform frames against \p expected (originX, originY,
/// xAxisX, xAxisY, yAxisX, yAxisY per bone; the deform frames equal the pose
/// frames for these unscaled rigs) within \p tol. A sanity check only: the
/// values were captured from one build, and the last bits depend on compiler
/// flags and libm, so bit-identity is checked against _SolveWithoutRetry.
bool _FramesNear(const Rig &rig, const double (*expected)[6], size_t count, double tol)
{
    if (rig.pose.size() != count || rig.deform.size() != count) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        const double got[6] = {rig.pose[i].originX, rig.pose[i].originY,
                               rig.pose[i].xAxisX, rig.pose[i].xAxisY,
                               rig.pose[i].yAxisX, rig.pose[i].yAxisY};
        for (int c = 0; c < 6; ++c) {
            if (!(std::fabs(got[c] - expected[i][c]) <= tol)) {
                std::fprintf(stderr, "  [%s] bone %zu component %d: %.17g, captured %.17g\n",
                             _context.c_str(), i, c, got[c], expected[i][c]);
                return false;
            }
        }
        if (std::memcmp(&rig.deform[i], &rig.pose[i], sizeof(RigExecBone2dFrame)) != 0) {
            return false;
        }
    }
    return true;
}

/// The solve as it was before the lock retry, computed IN THIS BINARY from
/// the header's own inline pieces, so a bitwise comparison with it holds
/// under any compiler flags, optimisation level and libm (captured %.17g
/// literals do not). The kernel's solve with every lock off leaves the
/// workspace in the state its lock loop starts from (Compose and
/// LocalPosition never read the lock fields, and the lock loop is the
/// solve's last step before the outputs); each lock then runs SolveLock's
/// steps WITHOUT the retry -- LockChain, the stretch step, LockDescent,
/// RecomposeBelow, the locked bone's own angle (Turn) -- and the outputs are
/// formed as RigExecSolveSkeleton2d forms them.
struct NoRetry {
    std::vector<RigExecBone2dFrame> pose, deform;
    bool reached = true;   ///< every lock ended within the retry tolerance
    double worst = 0;      ///< the largest lock residual after the descent
};

void _SolveLockWithoutRetry(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t count, size_t k,
    RigExecSkeleton2dWorkspace &ws, NoRetry *out)
{
    namespace d = skeleton2d_detail;
    const RigExecBone2dChannels &ch = channels[k];
    if (setup[k].parent < 0) {
        return;
    }
    d::LockChain(setup, k, ws);
    const double goalX = ch.lockX, goalY = ch.lockY;
    if (!ws.chain.empty()) {
        const size_t top = size_t(ws.chain.back());
        double reach = 0;
        {
            double fromX = ws.pose[top].originX, fromY = ws.pose[top].originY;
            for (size_t c = ws.chain.size(); c-- > 0;) {
                const size_t next = c == 0 ? k : size_t(ws.chain[c - 1]);
                const double dx = ws.pose[next].originX - fromX;
                const double dy = ws.pose[next].originY - fromY;
                reach += std::sqrt(dx * dx + dy * dy);
                fromX = ws.pose[next].originX; fromY = ws.pose[next].originY;
            }
        }
        const double gx = goalX - ws.pose[top].originX;
        const double gy = goalY - ws.pose[top].originY;
        const double distance = std::sqrt(gx * gx + gy * gy);
        const double maxStretch = std::max(1.0, setup[k].maxStretch);
        if (reach > 0 && distance > reach && maxStretch > 1.0) {
            const double factor = std::min(maxStretch, distance / reach);
            for (const int32_t j : ws.chain) {
                ws.scale[size_t(j)] *= factor;
            }
            for (size_t i = top + 1; i < count; ++i) {
                d::LocalPosition(setup, channels, i, ws);
            }
            d::RecomposeBelow(setup, channels, count, top, ws);
        }
        d::LockDescent(setup, channels, k, goalX, goalY, ws);
        const double tol = kRigExecSkeleton2dLockReachTolerance * std::max(1.0, reach);
        const double r = d::LockResidual(ws, k, goalX, goalY);
        out->reached = out->reached && !(r > tol);
        out->worst = std::max(out->worst, r);
        d::RecomposeBelow(setup, channels, count, top, ws);
    }
    const double delta = d::Wrap180(ch.lockAngle - d::WorldAngle(ws.pose[k]));
    d::Turn(setup, channels, count, k, delta, ws);
}

NoRetry _SolveWithoutRetry(const Rig &rig)
{
    NoRetry out;
    Rig unlocked = rig;
    for (RigExecBone2dChannels &c : unlocked.channels) {
        c.lock = false;
    }
    CHECK(unlocked.Solve());
    const size_t count = rig.setup.size();
    RigExecSkeleton2dWorkspace &ws = unlocked.ws;
    for (size_t k = 0; k < count; ++k) {
        if (rig.channels[k].lock) {
            _SolveLockWithoutRetry(rig.setup.data(), rig.channels.data(), count, k, ws, &out);
        }
    }
    out.pose.resize(count);
    out.deform.resize(count);
    for (size_t i = 0; i < count; ++i) {
        const RigExecBone2dSetup &b = rig.setup[i];
        const RigExecBone2dFrame &p = ws.pose[i];
        out.pose[i] = p;
        const double along = ws.scale[i];
        const double across = RigExecBone2dCrossScale(b.scaling, along, b.squashFactor);
        RigExecBone2dFrame &f = out.deform[i];
        f.originX = p.originX; f.originY = p.originY;
        f.xAxisX = p.xAxisX * along; f.xAxisY = p.xAxisY * along;
        f.yAxisX = p.yAxisX * across; f.yAxisY = p.yAxisY * across;
    }
    return out;
}

/// Bitwise: \p rig's solved frames are the no-retry reference's.
bool _SameAsWithoutRetry(const Rig &rig, const NoRetry &ref)
{
    return rig.pose.size() == ref.pose.size() &&
           rig.deform.size() == ref.deform.size() &&
           std::memcmp(rig.pose.data(), ref.pose.data(),
                       rig.pose.size() * sizeof(RigExecBone2dFrame)) == 0 &&
           std::memcmp(rig.deform.data(), ref.deform.data(),
                       rig.deform.size() * sizeof(RigExecBone2dFrame)) == 0;
}

/// The self-check of the reference: with no lock on, it IS the kernel's
/// solve, bit for bit (a drift in the solve's steps shows here first).
void TestNoRetryReference()
{
    _context = "no-retry reference, no locks";
    LegRig k = _MakeLeg(1, true, -140, 10);
    k.rig.channels[k.h].tx = -0.15;
    k.rig.channels[k.t].rz = 12.5;
    k.rig.channels[k.s].sx = 1.25;
    CHECK(k.rig.Solve());
    const NoRetry ref = _SolveWithoutRetry(k.rig);
    CHECK(ref.reached && ref.worst == 0);
    CHECK(_SameAsWithoutRetry(k.rig, ref));
}

// The lock solve reaches the legal bend whatever the setup's
// last bits. Leg rig limited, each of T, S and F's setup angles perturbed
// by -1e-12, 0 and +1e-12: coordinate descent alone stopped on S's upper
// bound (a straight knee, the foot 0.087 / 0.22 short) in 9 of these 18
// runs; with the closed-form retry every run reaches the backward bend.
void TestLockLimitedRestPerturbation()
{
    struct Case { double tx, ty, kneeRz; };
    const Case cases[] = {
        {-0.15, -0.10, -43.965456},
        {-0.30, -0.30, -76.960999},
    };
    const char *names = "TSF";
    for (const Case &c : cases) {
        for (int bone = 0; bone < 3; ++bone) {
            for (const double eps : {-1e-12, 0.0, 1e-12}) {
                _context = "lock rest perturbation, hip (" + std::to_string(c.tx) + ", " +
                           std::to_string(c.ty) + "), " + names[bone] + " " +
                           std::to_string(eps);
                LegRig k = _MakeLeg(1, true, -140, 10);
                const size_t which[3] = {k.t, k.s, k.f};
                k.rig.rests[which[bone]].angle += eps;
                _LockAndMoveHip(&k, c.tx, c.ty);
                CHECK_NEAR(_LockResidual(k.rig, k.f), 0, 1e-9);
                const double knee = k.rig.ws.localAngle[k.s];
                CHECK_NEAR(knee, c.kneeRz, 1e-6);
                CHECK(knee >= -140 && knee <= 10);
                CHECK_NEAR(k.rig.WorldAngle(k.f), k.rig.channels[k.f].lockAngle, 1e-9);
            }
        }
    }
}

// The two-joint retry: the side the descent stopped on violates S's limits
// and the other side fits -> the other side, reached (forward and mirrored
// legs); both sides violating -> the descent's answer, bit for bit against
// the no-retry reference computed in this binary (_SolveWithoutRetry); the
// frames captured from the solve before the retry existed are a sanity
// check within 1e-12 only.
void TestLockClosedFormBothSides()
{
    struct Case { double sign, lo, hi, tx, ty, kneeRz; };
    const Case reach[] = {
        {1, -140, 10, -0.15, -0.10, -43.965456},
        {-1, -10, 140, 0.15, -0.10, 43.965456},
        {-1, -10, 140, 0.30, -0.30, 76.960999},
    };
    for (const Case &c : reach) {
        _context = "lock closed form, sign " + std::to_string(c.sign) + ", hip (" +
                   std::to_string(c.tx) + ", " + std::to_string(c.ty) + ")";
        LegRig k = _MakeLeg(c.sign, true, c.lo, c.hi);
        _LockAndMoveHip(&k, c.tx, c.ty);
        CHECK_NEAR(_LockResidual(k.rig, k.f), 0, 1e-9);
        CHECK_NEAR(k.rig.ws.localAngle[k.s], c.kneeRz, 1e-6);
        CHECK(k.rig.ws.localAngle[k.s] >= c.lo && k.rig.ws.localAngle[k.s] <= c.hi);
    }
    // S limited to [-20, 10]: neither bend (-43.97 / +63.97 at the first
    // hip) fits, so the descent's answer stays.
    static const double kViolate[2][4][6] = {
    // Leg [-20,10] (-0.15,-0.10) both violate
    {{-0.14999999999999999, 0.19999999999999998, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {-0.14999999999999999, 0.14999999999999999, 0.21040386168662786, -0.97761455338356862, 0.97761455338356862, 0.21040386168662786},
     {-0.065838455325348849, -0.24104582135342748, 0.21040386168662786, -0.97761455338356862, 0.97761455338356862, 0.21040386168662786},
     {0.018323089349302303, -0.63209164270685492, 1, 2.2204460492503131e-16, -2.2204460492503131e-16, 1}},
    // Leg [-20,10] (-0.30,-0.30) both violate
    {{-0.29999999999999999, 0, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {-0.29999999999999999, -0.049999999999999989, 0.72077315127105956, -0.69317102103794426, 0.69317102103794426, 0.72077315127105956},
     {-0.011690739491576174, -0.32726840841517768, 0.27762234884752962, -0.96069028902158715, 0.96069028902158715, 0.27762234884752962},
     {0.099358200047435674, -0.71154452402381252, 1, 0, 0, 1}},
    };
    const double hips[2][2] = {{-0.15, -0.10}, {-0.30, -0.30}};
    for (int i = 0; i < 2; ++i) {
        _context = "lock closed form, both sides violate, hip " + std::to_string(i);
        LegRig k = _MakeLeg(1, true, -20, 10);
        _LockAndMoveHip(&k, hips[i][0], hips[i][1]);
        CHECK(_LockResidual(k.rig, k.f) > 0.05);
        CHECK(k.rig.ws.localAngle[k.s] >= -20 && k.rig.ws.localAngle[k.s] <= 10);
        const NoRetry ref = _SolveWithoutRetry(k.rig);
        CHECK(!ref.reached);                 // the descent stopped short ...
        CHECK(_SameAsWithoutRetry(k.rig, ref));  // ... and its answer stays
        CHECK(_FramesNear(k.rig, kViolate[i], 4, 1e-12));
    }
}

// The longer-chain retry: hip H, three limited-or-free chain bones T, S, U
// (0.3 long) above the locked foot F; the lock chain is [U, S, T].
// (a) S and U limited to [-140, 10]: descent stops with both on the upper
// bound, the foot 0.026 short; the mirrored start reaches.
// (b) S and U limited to [-140, 0]: descent holds the chain straight on the
// bound (0.13 short), its own mirror image; restarting the bound joints
// from their limits' midpoint, the top aimed at the goal, reaches.
// (c) out of reach: no retry, the chain points at the goal.
void TestLockLongChainMirror()
{
    struct Case { double hi, tx, ty; bool reaches; };
    const Case cases[] = {
        {10, -0.10, -0.05, true},
        {0, -0.10, -0.15, true},
        {0, -0.30, -0.05, false},
    };
    for (const Case &c : cases) {
        _context = "lock long chain, max " + std::to_string(c.hi) + ", hip (" +
                   std::to_string(c.tx) + ", " + std::to_string(c.ty) + ")";
        Rig rig;
        const size_t h = rig.Add(-1, 0, 0.5, -90);
        const size_t t = rig.AddAtTip(h, 0.05, 5);
        const size_t s = rig.AddAtTip(t, 0.3, 0);
        const size_t u = rig.AddAtTip(s, 0.3, 0);
        const size_t f = rig.AddAtTip(u, 0.3, 90);
        for (const size_t b : {s, u}) {
            rig.setup[b].flags |= kRigExecBone2dAngleLimits;
            rig.setup[b].minAngle = -140;
            rig.setup[b].maxAngle = c.hi;
        }
        CHECK(rig.Solve());
        rig.channels[f].lock = true;
        rig.channels[f].lockX = rig.pose[f].originX;
        rig.channels[f].lockY = rig.pose[f].originY;
        rig.channels[f].lockAngle = rig.WorldAngle(f);
        rig.channels[h].tx = c.tx;
        rig.channels[h].ty = c.ty;
        CHECK(rig.Solve());
        for (const size_t b : {s, u}) {
            CHECK(rig.ws.localAngle[b] >= -140 && rig.ws.localAngle[b] <= c.hi);
        }
        const double gx = rig.channels[f].lockX - rig.pose[t].originX;
        const double gy = rig.channels[f].lockY - rig.pose[t].originY;
        if (c.reaches) {
            CHECK(std::hypot(gx, gy) < 0.9);
            CHECK_NEAR(_LockResidual(rig, f), 0, 1e-9);
        } else {
            CHECK(std::hypot(gx, gy) > 0.9);
            // Straight toward the goal, short of it by the missing reach.
            CHECK_NEAR(_LockResidual(rig, f), std::hypot(gx, gy) - 0.9, 1e-6);
        }
        CHECK_NEAR(rig.WorldAngle(f), rig.channels[f].lockAngle, 1e-9);
    }
}

// Locks the descent reached are untouched by the retry: lock frames 6-9
// and the unlimited leg hips give, bit for bit, the frames of the no-retry
// reference computed in this binary (_SolveWithoutRetry), so the check holds
// under any flags and libm; the frames captured from the solve before the
// retry existed are a sanity check within 1e-12 only. Then a sweep (leg
// limited and unlimited, forward and mirrored, hips on a grid, the setup
// angles perturbed by -1e-12 / 0 / +1e-12): every lock the descent reaches
// is bit-identical to the reference; every one it leaves short ends no
// farther from its lock point, within the limits.
void TestLockReachedUnchanged()
{
    static const double kExpected[6][4][6] = {
    // Lock frame 6
    {{0.050000000000000003, 0.27000000000000002, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {0.050000000000000003, 0.22000000000000003, 0.21439774371646048, -0.97674643971160235, 0.97674643971160235, 0.21439774371646048},
     {0.13575909748658421, -0.17069857588464094, -0.33939774371646025, -0.94064295647188889, 0.94064295647188889, -0.33939774371646025},
     {1.1754932413893173e-16, -0.5469557584733965, 1, 5.5511151231257827e-17, -5.5511151231257827e-17, 1}},
    // Lock frame 7
    {{0.10000000000000001, 0.23999999999999999, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {0.10000000000000001, 0.19, 0.24013085171951623, -0.97074052869572713, 0.97074052869572713, 0.24013085171951623},
     {0.1960523406878065, -0.19829621147829085, -0.49013085171951609, -0.87164886748776405, 0.87164886748776405, -0.49013085171951609},
     {6.1559430244216311e-17, -0.5469557584733965, 1, -5.5511151231257827e-17, 5.5511151231257827e-17, 1}},
    // Lock frame 8
    {{0.15000000000000002, 0.20999999999999999, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {0.15000000000000002, 0.16, 0.23202815409807406, -0.97270907043464971, 0.97270907043464971, 0.23202815409807406},
     {0.24281126163922967, -0.22908362817385988, -0.60702815409807398, -0.79468032574884162, 0.79468032574884162, -0.60702815409807398},
     {6.1025353699151469e-17, -0.5469557584733965, 1, 5.5511151231257827e-17, -5.5511151231257827e-17, 1}},
    // Lock frame 9
    {{0.20000000000000001, 0.17999999999999999, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {0.20000000000000001, 0.13, 0.20130546938176258, -0.97952851311076605, 0.97952851311076605, 0.20130546938176258},
     {0.28052218775270504, -0.26181140524430646, -0.70130546938176275, -0.7128608830727251, 0.7128608830727251, -0.70130546938176275},
     {-5.056468519537057e-17, -0.5469557584733965, 1, 0, 0, 1}},
    // Leg unlimited (-0.15, -0.10)
    {{-0.14999999999999999, 0.19999999999999998, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {-0.14999999999999999, 0.14999999999999999, -0.25606511698304973, -0.96665953461622522, 0.96665953461622522, -0.25606511698304973},
     {-0.25242604679321989, -0.23666381384649013, 0.63106511698304968, -0.77572986156726609, 0.77572986156726609, 0.63106511698304968},
     {5.3827072075079008e-18, -0.54695575847339661, 1, 1.6653345369377348e-16, -1.6653345369377348e-16, 1}},
    // Leg unlimited (-0.30, -0.30)
    {{-0.29999999999999999, 0, 6.123233995736766e-17, -1, 1, 6.123233995736766e-17},
     {-0.29999999999999999, -0.049999999999999989, -0.21409007266530719, -0.97681392331710926, 0.97681392331710926, -0.21409007266530719},
     {-0.38563602906612288, -0.4407255693268437, 0.96409007266530722, -0.26557547286638206, 0.26557547286638206, 0.96409007266530722},
     {5.7353951260957649e-17, -0.5469557584733965, 0.99999999999999989, 1.6653345369377348e-16, -1.6653345369377348e-16, 0.99999999999999989}},
    };
    _context = "lock reached, unchanged";
    LegRig k = _MakeLeg(1, true, -140, 10);
    _LockAndMoveHip(&k, 0, 0);
    for (int frame = 6; frame <= 9; ++frame) {
        _context = "lock reached unchanged, lock frame " + std::to_string(frame);
        k.rig.channels[k.h].tx = 0.05 * (frame - 5);
        k.rig.channels[k.h].ty = -0.03 * (frame - 5);
        CHECK(k.rig.Solve());
        const NoRetry ref = _SolveWithoutRetry(k.rig);
        CHECK(ref.reached);
        CHECK(_SameAsWithoutRetry(k.rig, ref));
        CHECK(_FramesNear(k.rig, kExpected[frame - 6], 4, 1e-12));
    }
    const double hips[2][2] = {{-0.15, -0.10}, {-0.30, -0.30}};
    for (int i = 0; i < 2; ++i) {
        _context = "lock reached unchanged, leg unlimited, hip " + std::to_string(i);
        LegRig u = _MakeLeg(1, false, 0, 0);
        _LockAndMoveHip(&u, hips[i][0], hips[i][1]);
        const NoRetry ref = _SolveWithoutRetry(u.rig);
        CHECK(ref.reached);
        CHECK(_SameAsWithoutRetry(u.rig, ref));
        CHECK(_FramesNear(u.rig, kExpected[4 + i], 4, 1e-12));
    }
    int reached = 0, retried = 0;
    for (const double sign : {1.0, -1.0}) {
        for (const bool limited : {true, false}) {
            for (int ix = -6; ix <= 6; ++ix) {
                for (int iy = -6; iy <= 1; ++iy) {
                    for (const double eps : {-1e-12, 0.0, 1e-12}) {
                        _context = "lock sweep, sign " + std::to_string(sign) +
                                   (limited ? " limited" : " free") + ", hip (" +
                                   std::to_string(0.05 * ix) + ", " +
                                   std::to_string(0.05 * iy) + "), eps " +
                                   std::to_string(eps);
                        LegRig w = sign > 0 ? _MakeLeg(1, limited, -140, 10)
                                           : _MakeLeg(-1, limited, -10, 140);
                        w.rig.rests[w.s].angle += eps;
                        _LockAndMoveHip(&w, 0.05 * ix, 0.05 * iy);
                        const NoRetry ref = _SolveWithoutRetry(w.rig);
                        if (ref.reached) {
                            ++reached;
                            CHECK(_SameAsWithoutRetry(w.rig, ref));
                        } else {
                            ++retried;
                            CHECK(_LockResidual(w.rig, w.f) <= ref.worst);
                        }
                        if (limited) {
                            const double knee = w.rig.ws.localAngle[w.s];
                            CHECK(sign > 0 ? (knee >= -140 && knee <= 10)
                                           : (knee >= -10 && knee <= 140));
                        }
                    }
                }
            }
        }
    }
    _context = "lock sweep";
    // Both kinds occur, so the sweep exercises both claims.
    CHECK(reached > 100);
    CHECK(retried > 10);
    std::printf("lock sweep: %d reached by the descent (bit-identical), "
                "%d retried (never farther)\n", reached, retried);
}

// The retries are deterministic: the same inputs give the same bits, in a
// fresh workspace and in a reused one.
void TestLockDeterminism()
{
    _context = "lock determinism";
    const double hips[3][2] = {{-0.15, -0.10}, {-0.30, -0.30}, {-0.22, -0.18}};
    for (const auto &hip : hips) {
        LegRig a = _MakeLeg(1, true, -140, 10);
        LegRig b = _MakeLeg(1, true, -140, 10);
        a.rig.rests[a.s].angle += 1e-12;
        b.rig.rests[b.s].angle += 1e-12;
        _LockAndMoveHip(&a, hip[0], hip[1]);
        _LockAndMoveHip(&b, hip[0], hip[1]);
        CHECK(a.rig.SameFrames(b.rig));
        const Rig once = a.rig;
        CHECK(a.rig.Solve());
        CHECK(a.rig.SameFrames(once));
    }
}

// Bone offsets apply to the posed local position only (frame-0
// exclusion is held-knot authoring, stage-tested).
void TestOffset()
{
    _context = "offset";
    Rig rig;
    const size_t r = rig.Add(-1, 0, 0, 0);
    const size_t c = rig.AddAtTip(r, 1.0, 0);
    (void)r;
    rig.channels[c].offsetX = 0.5;
    rig.channels[c].offsetY = 0.25;
    CHECK(rig.Solve());
    CHECK_NEAR(rig.pose[c].originX, 1.5, 1e-12);
    CHECK_NEAR(rig.pose[c].originY, 0.25, 1e-12);
    // A rigidly bound marker moves by the offset.
    double ox = 0, oy = 0;
    rig.Map(c, 1.0, 0.0, &ox, &oy);
    CHECK_NEAR(ox, 1.5, 1e-12);
    CHECK_NEAR(oy, 0.25, 1e-12);
    // Parent space: under a parent turned 90 degrees the offset turns too.
    Rig turned;
    const size_t tr = turned.Add(-1, 0, 0, 0);
    const size_t tc = turned.AddAtTip(tr, 1.0, 0);
    turned.channels[tr].rz = 90;
    turned.channels[tc].offsetX = 0.5;
    turned.channels[tc].offsetY = 0.25;
    CHECK(turned.Solve());
    CHECK_NEAR(turned.pose[tc].originX, -0.25, 1e-12);
    CHECK_NEAR(turned.pose[tc].originY, 1.5, 1e-12);
    // Zero offsets change nothing bit for bit.
    Rig plain, zeroed;
    {
        const size_t pr = plain.Add(-1, 0, 0, 0);
        plain.AddAtTip(pr, 1.0, 0);
        const size_t zr = zeroed.Add(-1, 0, 0, 0);
        const size_t zc = zeroed.AddAtTip(zr, 1.0, 0);
        zeroed.channels[zc].offsetX = 0.0;
        zeroed.channels[zc].offsetY = 0.0;
    }
    CHECK(plain.Solve());
    CHECK(zeroed.Solve());
    CHECK(plain.SameFrames(zeroed));
}

// IK targets.
void TestIkTarget()
{
    _context = "ik-target";
    // Thigh (0,2)->(0,1), shin (0,1)->(0,0), foot pin at (0.3,0.1).
    Rig rig;
    const size_t thigh = rig.Add(-1, 0, 2, -90);
    const size_t shin = rig.AddAtTip(thigh, 1.0, 0);
    const size_t foot = rig.Add(-1, 0.3, 0.1, 0);
    (void)foot;
    rig.setup[shin].targetBegin = 0;
    rig.setup[shin].targetCount = 1;
    rig.channels[shin].targetIndex = 0;
    rig.End(shin, 1.0);
    const int32_t targets[] = {int32_t(foot)};
    RigExecSkeleton2dStatic extra;
    extra.targetCandidates = targets;
    extra.targetCandidateCount = 1;
    CHECK(rig.SolveEx(&extra));
    // The tip includes the along-bone scale: the deform frame's axis.
    const double tipX = rig.deform[shin].originX + rig.deform[shin].xAxisX * 1.0;
    const double tipY = rig.deform[shin].originY + rig.deform[shin].xAxisY * 1.0;
    CHECK_NEAR(tipX, 0.3, 1e-6);
    CHECK_NEAR(tipY, 0.1, 1e-6);
    // Out of reach with maxStretch 1: straight toward the goal, tip short.
    Rig far;
    const size_t fthigh = far.Add(-1, 0, 2, -90);
    const size_t fshin = far.AddAtTip(fthigh, 1.0, 0);
    const size_t ffoot = far.Add(-1, 2.5, 2.0, 0);
    (void)ffoot;
    far.setup[fshin].targetBegin = 0;
    far.setup[fshin].targetCount = 1;
    far.channels[fshin].targetIndex = 0;
    far.End(fshin, 1.0);
    const int32_t ftargets[] = {int32_t(ffoot)};
    RigExecSkeleton2dStatic fextra;
    fextra.targetCandidates = ftargets;
    fextra.targetCandidateCount = 1;
    CHECK(far.SolveEx(&fextra));
    const double ftipX = far.deform[fshin].originX + far.deform[fshin].xAxisX * 1.0;
    const double ftipY = far.deform[fshin].originY + far.deform[fshin].xAxisY * 1.0;
    CHECK_NEAR(std::hypot(ftipX - 0.0, ftipY - 2.0), 2.0, 1e-3);
    CHECK_NEAR(ftipY, 2.0, 1e-3);
    // The same goal with maxStretch 1.5 reaches it.
    far.setup[fshin].maxStretch = 1.5;
    CHECK(far.SolveEx(&fextra));
    const double stipX = far.deform[fshin].originX + far.deform[fshin].xAxisX * 1.0;
    const double stipY = far.deform[fshin].originY + far.deform[fshin].xAxisY * 1.0;
    CHECK_NEAR(stipX, 2.5, 1e-6);
    CHECK_NEAR(stipY, 2.0, 1e-6);
}

// A single bone with a target and no chain aims its tip at it; -1 and
// out-of-range indices are no-ops; a lone pin has nothing to aim.
void TestIkTargetAimAndNone()
{
    _context = "ik-aim";
    Rig rig;
    const size_t bone = rig.Add(-1, 0, 0, 0);
    const size_t pin = rig.Add(-1, 1, 1, 0);
    (void)pin;
    rig.setup[bone].targetBegin = 0;
    rig.setup[bone].targetCount = 1;
    rig.End(bone, 1.0);
    const int32_t targets[] = {int32_t(pin)};
    RigExecSkeleton2dStatic extra;
    extra.targetCandidates = targets;
    extra.targetCandidateCount = 1;
    rig.channels[bone].targetIndex = 0;
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.WorldAngle(bone), 45, 1e-6);
    CHECK_NEAR(rig.pose[bone].originX, 0, 1e-12);
    CHECK_NEAR(rig.pose[bone].originY, 0, 1e-12);
    // No target: plain FK.
    Rig plain;
    const size_t pb = plain.Add(-1, 0, 0, 0);
    plain.Add(-1, 1, 1, 0);
    plain.setup[pb].targetBegin = 0;
    plain.setup[pb].targetCount = 1;
    plain.End(pb, 1.0);
    plain.channels[pb].targetIndex = -1;
    CHECK(plain.SolveEx(&extra));
    CHECK_NEAR(plain.WorldAngle(pb), 0, 1e-12);
    plain.channels[pb].targetIndex = 5;
    CHECK(plain.SolveEx(&extra));
    CHECK_NEAR(plain.WorldAngle(pb), 0, 1e-12);
    // An off-axis tip: the tip, not the +X axis, points at the target.
    Rig skew;
    const size_t sb = skew.Add(-1, 0, 0, 0);
    skew.Add(-1, 1, 1, 0);
    skew.setup[sb].targetBegin = 0;
    skew.setup[sb].targetCount = 1;
    skew.End(sb, 1.0);
    skew.setup[sb].tipY = 1.0;
    skew.channels[sb].targetIndex = 0;
    CHECK(skew.SolveEx(&extra));
    CHECK_NEAR(skew.WorldAngle(sb), 0, 1e-9);
    // A lone pin (no tip) with a target has nothing to turn.
    Rig zero;
    const size_t zb = zero.Add(-1, 0, 0, 0);
    const size_t zp = zero.Add(-1, 1, 1, 0);
    (void)zp;
    zero.setup[zb].targetBegin = 0;
    zero.setup[zb].targetCount = 1;
    zero.channels[zb].targetIndex = 0;
    const int32_t ztargets[] = {int32_t(zp)};
    RigExecSkeleton2dStatic zextra;
    zextra.targetCandidates = ztargets;
    zextra.targetCandidateCount = 1;
    CHECK(zero.SolveEx(&zextra));
    CHECK_NEAR(zero.WorldAngle(zb), 0, 1e-12);
    CHECK_NEAR(zero.pose[zb].originX, 0, 1e-12);
    CHECK_NEAR(zero.pose[zb].originY, 0, 1e-12);
}

// A pin owner turns its ancestors so its base meets the target.
void TestPinTarget()
{
    _context = "pin-target";
    Rig rig;
    const size_t upper = rig.Add(-1, 0, 0, 0);
    const size_t lower = rig.AddAtTip(upper, 1.0, 0);
    const size_t hand = rig.AddAtTip(lower, 1.0, 0);  // a pin: no End joint
    const size_t goal = rig.Add(-1, 1.0, 1.0, 0);
    rig.setup[hand].targetBegin = 0;
    rig.setup[hand].targetCount = 1;
    rig.channels[hand].targetIndex = 0;
    CHECK(!(rig.setup[hand].flags & kRigExecBone2dTip));
    const int32_t targets[] = {int32_t(goal)};
    RigExecSkeleton2dStatic extra;
    extra.targetCandidates = targets;
    extra.targetCandidateCount = 1;
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.pose[hand].originX, 1.0, 1e-6);
    CHECK_NEAR(rig.pose[hand].originY, 1.0, 1e-6);
}

// Control bones, delta form.
void TestControlBones()
{
    _context = "control";
    Rig rig;
    const size_t dial = rig.Add(-1, 0, 0, 0);
    const size_t muscle = rig.Add(-1, 2, 0, 0);
    rig.setup[muscle].angleController = int32_t(dial);
    rig.setup[muscle].angleFactor = 0.5;
    rig.channels[dial].rz = 40;
    rig.channels[muscle].rz = 10;
    CHECK(rig.Solve());
    CHECK_NEAR(rig.WorldAngle(muscle), 10 + 20, 1e-9);
    // Counter-moving pair.
    rig.setup[muscle].angleFactor = -0.5;
    CHECK(rig.Solve());
    CHECK_NEAR(rig.WorldAngle(muscle), 10 - 20, 1e-9);
    // Position factors are per axis, in parent space.
    Rig pos;
    const size_t pd = pos.Add(-1, 0, 0, 0);
    const size_t pm = pos.Add(-1, 2, 0, 0);
    pos.setup[pm].posController = int32_t(pd);
    pos.setup[pm].posFactorX = 2.0;
    pos.setup[pm].posFactorY = -1.0;
    pos.channels[pd].tx = 3.0;
    pos.channels[pd].ty = 4.0;
    pos.channels[pm].tx = 1.0;
    CHECK(pos.Solve());
    CHECK_NEAR(pos.pose[pm].originX, 2.0 + 1.0 + 2.0 * 3.0, 1e-9);
    CHECK_NEAR(pos.pose[pm].originY, 0.0 + (-1.0) * 4.0, 1e-9);
    // Scale multiplies about 1: s_own * (1 + k (s_K - 1)).
    Rig scl;
    const size_t sd = scl.Add(-1, 0, 0, 0);
    const size_t sm = scl.Add(-1, 2, 0, 0);
    scl.setup[sm].scaleController = int32_t(sd);
    scl.setup[sm].scaleFactor = 1.0;
    scl.channels[sd].sx = 2.0;
    scl.channels[sm].sx = 3.0;
    CHECK(scl.Solve());
    CHECK_NEAR(std::hypot(scl.deform[sm].xAxisX, scl.deform[sm].xAxisY), 3.0 * 2.0, 1e-9);
    // Chained controls compose: A -> B -> C.
    Rig chain;
    const size_t a = chain.Add(-1, 0, 0, 0);
    const size_t b = chain.Add(-1, 1, 0, 0);
    const size_t c = chain.Add(-1, 2, 0, 0);
    chain.setup[b].angleController = int32_t(a);
    chain.setup[b].angleFactor = 1.0;
    chain.setup[c].angleController = int32_t(b);
    chain.setup[c].angleFactor = 1.0;
    chain.channels[a].rz = 10;
    CHECK(chain.Solve());
    CHECK_NEAR(chain.WorldAngle(b), 10, 1e-9);
    CHECK_NEAR(chain.WorldAngle(c), 10, 1e-9);
    // A control cycle runs in bone order, deterministically.
    Rig cyc;
    const size_t x = cyc.Add(-1, 0, 0, 0);
    const size_t y = cyc.Add(-1, 1, 0, 0);
    cyc.setup[x].angleController = int32_t(y);
    cyc.setup[x].angleFactor = 1.0;
    cyc.setup[y].angleController = int32_t(x);
    cyc.setup[y].angleFactor = 1.0;
    cyc.channels[x].rz = 10;
    CHECK(cyc.Solve());
    CHECK_NEAR(cyc.WorldAngle(x), 10, 1e-9);
    CHECK_NEAR(cyc.WorldAngle(y), 10, 1e-9);
    Rig cyc2 = cyc;
    CHECK(cyc2.Solve());
    CHECK(cyc.SameFrames(cyc2));
    // Zero factors and no controller change nothing.
    Rig noop;
    const size_t nd = noop.Add(-1, 0, 0, 0);
    const size_t nm = noop.Add(-1, 2, 0, 0);
    noop.setup[nm].angleController = int32_t(nd);
    noop.setup[nm].angleFactor = 0.0;
    noop.channels[nd].rz = 40;
    noop.channels[nm].rz = 10;
    CHECK(noop.Solve());
    CHECK_NEAR(noop.WorldAngle(nm), 10, 1e-9);
}

// Bone groups switch alternative chains.
void TestBoneGroups()
{
    _context = "groups";
    Rig rig;
    const size_t torso = rig.Add(-1, 0, 0, 0);
    const size_t headA = rig.Add(int(torso), 0, 1, 0);
    const size_t jawA = rig.Add(int(headA), 0, 1.5, 0);
    const size_t headB = rig.Add(int(torso), 0, 1, 0);
    const size_t jawB = rig.Add(int(headB), 0, 1.5, 0);
    RigExecBone2dGroup groups[1];
    groups[0].memberBegin = 0;
    groups[0].memberCount = 2;
    groups[0].enabled = true;
    const int32_t members[] = {int32_t(headA), int32_t(headB)};
    RigExecSkeleton2dStatic extra;
    extra.groups = groups;
    extra.groupCount = 1;
    extra.groupMembers = members;
    extra.groupMemberCount = 2;
    rig.channels[headA].rz = 30;
    rig.channels[headB].rz = 30;
    rig.channels[jawB].rz = 15;
    rig.selectors = {0};
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.WorldAngle(headA), 30, 1e-9);
    CHECK_NEAR(rig.WorldAngle(jawA), 30, 1e-9);
    CHECK(_IsRestFrame(rig.pose[headB], rig.rests[headB]));
    CHECK(_IsRestFrame(rig.deform[headB], rig.rests[headB]));
    CHECK(_IsRestFrame(rig.deform[jawB], rig.rests[jawB]));
    rig.selectors = {1};
    CHECK(rig.SolveEx(&extra));
    CHECK(_IsRestFrame(rig.pose[headA], rig.rests[headA]));
    CHECK_NEAR(rig.WorldAngle(headB), 30, 1e-9);
    CHECK(_IsRestFrame(rig.pose[jawA], rig.rests[jawA]));
    // A disabled group keeps every member active.
    groups[0].enabled = false;
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.WorldAngle(headA), 30, 1e-9);
    CHECK_NEAR(rig.WorldAngle(headB), 30, 1e-9);
    groups[0].enabled = true;
    // An out-of-range index clamps to the nearest member.
    rig.selectors = {5};
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.WorldAngle(headB), 30, 1e-9);
    CHECK(_IsRestFrame(rig.pose[headA], rig.rests[headA]));
    rig.selectors = {-1};
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.WorldAngle(headA), 30, 1e-9);
}

// A space switch selects the bone's parent space per evaluation: a bone,
// the base or an outside space, rest-locals following the selected space; a
// fractional index blends the neighbouring spaces.
void TestSpaceSwitch()
{
    _context = "space switch";
    // Arm root A at (0,0), hand H at its tip (1,0); table T at (5,5). The
    // solve order A, T, H covers the namespace parent and every source.
    Rig rig;
    const size_t a = rig.Add(-1, 0, 0, 0);
    const size_t t = rig.Add(-1, 5, 5, 0);
    const size_t h = rig.Add(int(a), 1, 0, 0);
    // Sources: the arm, the table, the base, outside space 0.
    const int32_t sources[] = {int32_t(a), int32_t(t), kRigExecBone2dSpaceBase,
                               RigExecBone2dSpaceExternal(0)};
    rig.setup[h].spaceBegin = 0;
    rig.setup[h].spaceCount = 4;
    rig.setup[h].spaceSelector = 0;
    rig.externalRests = {{2, 0, 0}};
    rig.externalPoses = {{2, 0, 0}};
    RigExecSkeleton2dStatic extra;
    extra.spaceSources = sources;
    extra.spaceSourceCount = 4;
    // Under the arm: the hand rides its tip.
    rig.selectors = {0};
    rig.channels[a].rz = 90;
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.pose[h].originX, 0.0, 1e-12);
    CHECK_NEAR(rig.pose[h].originY, 1.0, 1e-12);
    CHECK_NEAR(rig.WorldAngle(h), 90, 1e-9);
    // Under the table: at rest channels the setup placement holds, and the
    // hand follows the table, not the arm.
    rig.selectors = {1};
    rig.channels[t].tx = 2.0;
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.pose[h].originX, 3.0, 1e-12);
    CHECK_NEAR(rig.pose[h].originY, 0.0, 1e-12);
    CHECK_NEAR(rig.WorldAngle(h), 0, 1e-9);
    // The base: the setup placement, whatever moves.
    rig.selectors = {2};
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.pose[h].originX, 1.0, 1e-12);
    CHECK_NEAR(rig.pose[h].originY, 0.0, 1e-12);
    // An outside space turned 90 degrees about its base (2, 0).
    rig.selectors = {3};
    rig.externalPoses = {{2, 0, 90}};
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.pose[h].originX, 2.0, 1e-12);
    CHECK_NEAR(rig.pose[h].originY, -1.0, 1e-12);
    CHECK_NEAR(rig.WorldAngle(h), 90, 1e-9);
    // Half way between the arm and the table: the origin lerps, the angle
    // takes the shortest arc.
    rig.selectors = {0.5};
    CHECK(rig.SolveEx(&extra));
    CHECK_NEAR(rig.pose[h].originX, 0.5 * 0.0 + 0.5 * 3.0, 1e-12);
    CHECK_NEAR(rig.pose[h].originY, 0.5 * 1.0 + 0.5 * 0.0, 1e-12);
    CHECK_NEAR(rig.WorldAngle(h), 45, 1e-9);
    // Clamped to the list; a whole number is exact.
    rig.selectors = {7};
    Rig last = rig;
    CHECK(rig.SolveEx(&extra));
    last.selectors = {3};
    CHECK(last.SolveEx(&extra));
    CHECK(rig.SameFrames(last));
    rig.selectors = {-3};
    last.selectors = {0};
    CHECK(rig.SolveEx(&extra));
    CHECK(last.SolveEx(&extra));
    CHECK(rig.SameFrames(last));
    // A bone source after its bone, or an outside space that is missing,
    // is malformed.
    const int32_t late[] = {int32_t(h)};
    RigExecSkeleton2dStatic bad;
    bad.spaceSources = late;
    bad.spaceSourceCount = 1;
    rig.setup[h].spaceCount = 1;
    rig.selectors = {0};
    CHECK(!rig.SolveEx(&bad));
    const int32_t missing[] = {RigExecBone2dSpaceExternal(4)};
    bad.spaceSources = missing;
    CHECK(!rig.SolveEx(&bad));
    // An outside space makes the bone a chain root: a lock on it does not
    // reach the arm.
    Rig root;
    const size_t ra = root.Add(-1, 0, 0, 0);
    const size_t rb = root.AddAtTip(ra, 1.0, 0);
    const int32_t outside[] = {RigExecBone2dSpaceExternal(0)};
    root.setup[rb].spaceBegin = 0;
    root.setup[rb].spaceCount = 1;
    root.setup[rb].spaceSelector = 0;
    root.externalRests = {{0, 0, 0}};
    root.externalPoses = {{0, 0, 0}};
    root.selectors = {0};
    root.channels[rb].lock = true;
    root.channels[rb].lockX = 0.5;
    root.channels[rb].lockY = 0.5;
    RigExecSkeleton2dStatic rootExtra;
    rootExtra.spaceSources = outside;
    rootExtra.spaceSourceCount = 1;
    CHECK(root.SolveEx(&rootExtra));
    CHECK_NEAR(root.WorldAngle(ra), 0, 1e-12);
    CHECK_NEAR(root.pose[rb].originX, 1.0, 1e-12);
}

// The start-frame plane: rests and outside spaces in the start frame's rest
// frame, the published frames composed back through its posed frame.
struct V3 {
    double v[3] = {0, 0, 0};
    V3() = default;
    V3(double x, double y, double z) : v{x, y, z} {}
    double operator[](size_t i) const { return v[i]; }
    double &operator[](size_t i) { return v[i]; }
};
using L4 = std::array<V3, 4>;
struct PF {
    L4 points;
    uint32_t flags = 0;
};

// Landmarks of a frame: origin o, axes x, y, z.
L4 Landmarks(const V3 &o, const V3 &x, const V3 &y, const V3 &z)
{
    return {o, V3(o[0] + x[0], o[1] + x[1], o[2] + x[2]),
            V3(o[0] + y[0], o[1] + y[1], o[2] + y[2]),
            V3(o[0] + z[0], o[1] + z[1], o[2] + z[2])};
}

void TestStartFramePlane()
{
    _context = "start frame";
    // A start frame tilted 90 degrees about X: its XY plane is the world XZ
    // plane, its +Z is world -Y. Rest at (1, 2, 3); posed moved by (10, 0, 0).
    const V3 ex(1, 0, 0), ey(0, 0, 1), ez(0, -1, 0);
    const L4 startRest = Landmarks(V3(1, 2, 3), ex, ey, ez);
    const L4 startPose = Landmarks(V3(11, 2, 3), ex, ey, ez);
    // Two bones in that plane: A at plane (0, 0) along plane +X, B at
    // plane (1, 0).
    const auto lift = [&](double px, double py, double angle) {
        const double c = std::cos(angle * kDeg), s = std::sin(angle * kDeg);
        const V3 o(1 + px, 2, 3 + py);
        return Landmarks(o, V3(c, 0, s), V3(-s, 0, c), ez);
    };
    std::vector<L4> rests = {lift(0, 0, 0), lift(1, 0, 0)};
    std::vector<RigExecBone2dSetup> setup(2);
    setup[1].parent = 0;
    setup[0].flags = kRigExecBone2dTip;
    setup[0].tipBone = 1;
    const std::vector<uint32_t> order = {0, 1};
    RigExecSkeleton2dStatic extra;
    RigExecSkeleton2dScratch scratch;
    std::vector<PF> frames;
    RigExecSkeleton2dLandmarkInputs<L4> inputs;
    inputs.startRest = &startRest;
    inputs.startPose = &startPose;
    const auto channels = [](size_t j, RigExecBone2dChannels *c) {
        if (j == 0) c->rz = 90;
    };
    CHECK(RigExecSolveSkeleton2dJoints(order, rests, setup, extra, inputs, channels, 1u,
                                       &scratch, &frames) == RigExecSkeleton2dResult::Solved);
    // A turns 90 about the plane's +Z (world -Y): its X axis becomes the
    // plane's +Y (world +Z); B lands one unit along it. Both moved by the
    // start frame's (10, 0, 0).
    CHECK_NEAR(frames[0].points[0][0], 11, 1e-12);
    CHECK_NEAR(frames[0].points[0][1], 2, 1e-12);
    CHECK_NEAR(frames[0].points[0][2], 3, 1e-12);
    CHECK_NEAR(frames[0].points[1][0], 11, 1e-12);
    CHECK_NEAR(frames[0].points[1][2], 4, 1e-12);
    CHECK_NEAR(frames[1].points[0][0], 11, 1e-12);
    CHECK_NEAR(frames[1].points[0][1], 2, 1e-12);
    CHECK_NEAR(frames[1].points[0][2], 4, 1e-12);
    CHECK_NEAR(frames[0].points[3][1], 1, 1e-12);  // +Z landmark: world -Y
    // Without the start frame these rests are not planar.
    RigExecSkeleton2dLandmarkInputs<L4> flat;
    CHECK(RigExecSolveSkeleton2dJoints(order, rests, setup, extra, flat, channels, 1u,
                                       &scratch, &frames) == RigExecSkeleton2dResult::NotPlanar);
    CHECK(frames[1].points[0][2] == rests[1][0][2]);  // the rest, unchanged
    // An asset-space skeleton maps nothing: identical to the plane solve.
    std::vector<L4> planar = {Landmarks(V3(0, 0, 0), V3(1, 0, 0), V3(0, 1, 0), V3(0, 0, 1)),
                              Landmarks(V3(1, 0, 0), V3(1, 0, 0), V3(0, 1, 0), V3(0, 0, 1))};
    CHECK(RigExecSolveSkeleton2dJoints(order, planar, setup, extra, flat, channels, 1u,
                                       &scratch, &frames) == RigExecSkeleton2dResult::Solved);
    Rig reference;
    reference.Add(-1, 0, 0, 0);
    reference.Add(0, 1, 0, 0);
    reference.channels[0].rz = 90;
    CHECK(reference.Solve());
    CHECK(frames[1].points[0][0] == reference.deform[1].originX);
    CHECK(frames[1].points[0][1] == reference.deform[1].originY);
}

}  // namespace

int main()
{
    TestIndependentAngle();
    TestAngleLimits();
    TestLock();
    TestLockWithinLimits();
    TestMaxStretch();
    TestCrossScale();
    TestFlips();
    TestParentScale();
    TestRestsAndFailures();
    TestNoRetryReference();
    TestLockLimitedRestPerturbation();
    TestLockClosedFormBothSides();
    TestLockLongChainMirror();
    TestLockReachedUnchanged();
    TestLockDeterminism();
    TestOffset();
    TestIkTarget();
    TestIkTargetAimAndNone();
    TestPinTarget();
    TestControlBones();
    TestBoneGroups();
    TestSpaceSwitch();
    TestStartFramePlane();
    if (_failures) {
        std::fprintf(stderr, "testRigExecSkeleton2dKernel: %d failure(s)\n", _failures);
        return 1;
    }
    std::printf("testRigExecSkeleton2dKernel: ok\n");
    return 0;
}
