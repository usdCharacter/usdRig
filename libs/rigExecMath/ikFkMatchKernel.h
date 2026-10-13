// Shared, USD-free arithmetic of RigExecIkFkMatch: the channel values that
// put a switchable limb's other half onto the joints it has now. The native
// program and the binary runtime both call these, so they agree by
// construction.
//
// Conventions: frames are 16 doubles, row-major, row vectors (a point maps as
// p * M; the three handle vectors are rows 0-2 and the origin is row 3), as
// RigExecPointFrame and the runtime's frames publish them. A control's posed
// frame is A * P: A its avar matrix (RigExecProviderComposeAvars, S * R(order)
// * Rspin * T) and P the frame those avars compose onto. Angles are degrees on
// the channels and radians elsewhere.
//
// The values are solved from one evaluated generation. A control whose parent
// moves with another control the match writes has that parent carried
// forward: an FK control's parent rides the control above it (its bone
// offset lengthened by the stretch the match writes, in proportion), and the
// caller supplies the pole's parent once the IK control has moved
// (RigExecIkFkMatchPole), since that follows from the pole's own space.
#ifndef RIGEXEC_MATH_IK_FK_MATCH_KERNEL_H
#define RIGEXEC_MATH_IK_FK_MATCH_KERNEL_H

#include "avarScale.h"
#include "limbStretchKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace rigExec {

/// One control's channels and how they compose.
struct RigExecIkFkMatchChannels {
    double t[3] = {0, 0, 0};
    double s[3] = {1, 1, 1};
    /// Raw channel values: the rig applies r * sign (avars:rotationSign).
    double r[3] = {0, 0, 0};
    double spin = 0;
    double sign[3] = {1, 1, 1};
    /// avars:unitScaleFactor, applied to the translation channels.
    double units = 1;
    /// avars:rotationOrder as three of 'X', 'Y', 'Z'.
    char order[3] = {'X', 'Y', 'Z'};
};

/// What the IK half needs to land on the current joints.
struct RigExecIkFkMatchIkInputs {
    /// The limb's joints, root to end, and the IK solve's root control.
    const double *joints[3] = {nullptr, nullptr, nullptr};
    const double *root = nullptr;
    /// The IK control's posed frame and channels, and the effector the solve
    /// reaches for (the IK control itself, or a frame riding it).
    const double *ikPosed = nullptr;
    const double *effectorPosed = nullptr;
    RigExecIkFkMatchChannels ik;
    /// The solve's bend-plane turn after the pole sets it.
    double twistRadians = 0;
    /// The pole's distance from the middle joint, used when the solve is
    /// unpinned. A pinned solve puts the middle joint on the pole, so the
    /// pole goes on the middle joint.
    double poleDistance = 0;
    /// The current pole origin: its direction from the middle joint is kept
    /// when the limb is too straight to have a bend plane.
    const double *polePosed = nullptr;
    /// softDistance limb: the stretch scales are solved too.
    bool hasStretch = false;
    /// Current limb inputs; upperScale/lowerScale start the solve.
    RigExecLimbStretch limb;
    /// The solve's rest bone lengths, in its space.
    double restUpper = 0;
    double restLower = 0;
};

struct RigExecIkFkMatchIkResult {
    bool ok = false;
    RigExecIkFkMatchChannels ik;
    /// The IK control's frame once its channels are set.
    double ikFrame[16];
    /// Where the pole goes, in asset space.
    double poleTarget[3] = {0, 0, 0};
    double upperScale = 1;
    double lowerScale = 1;
};

/// What the FK half needs to land on the current joints.
struct RigExecIkFkMatchFkInputs {
    const double *joints[3] = {nullptr, nullptr, nullptr};
    const double *fkPosed[3] = {nullptr, nullptr, nullptr};
    RigExecIkFkMatchChannels fk[3];
    /// Each FK control's rest frame times its joint's inverse rest frame.
    const double *offsets[3] = {nullptr, nullptr, nullptr};
    /// The limb lengthens FK by its stretch scales: the bone offset of
    /// controls 1 and 2 scales in proportion to upperScale and lowerScale.
    bool hasStretch = false;
    double upperScale = 1;
    double lowerScale = 1;
    double restUpper = 0;
    double restLower = 0;
};

struct RigExecIkFkMatchFkResult {
    bool ok = false;
    RigExecIkFkMatchChannels fk[3];
    double frames[3][16];
    double upperScale = 1;
    double lowerScale = 1;
};

namespace ikFkMatchKernel {

using M4 = std::array<double, 16>;

inline M4
Identity()
{
    return M4{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

inline M4
Load(const double *m)
{
    M4 out;
    std::copy(m, m + 16, out.begin());
    return out;
}

inline M4
Mul(const M4 &a, const M4 &b)
{
    M4 out{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double sum = 0;
            for (int k = 0; k < 4; ++k) {
                sum += a[r * 4 + k] * b[k * 4 + c];
            }
            out[r * 4 + c] = sum;
        }
    }
    return out;
}

/// Inverse of a general 4x4; false when singular.
inline bool
Inverse(const M4 &m, M4 *out)
{
    const double *a = m.data();
    double inv[16];
    inv[0] = a[5] * a[10] * a[15] - a[5] * a[11] * a[14] - a[9] * a[6] * a[15] +
             a[9] * a[7] * a[14] + a[13] * a[6] * a[11] - a[13] * a[7] * a[10];
    inv[4] = -a[4] * a[10] * a[15] + a[4] * a[11] * a[14] + a[8] * a[6] * a[15] -
             a[8] * a[7] * a[14] - a[12] * a[6] * a[11] + a[12] * a[7] * a[10];
    inv[8] = a[4] * a[9] * a[15] - a[4] * a[11] * a[13] - a[8] * a[5] * a[15] +
             a[8] * a[7] * a[13] + a[12] * a[5] * a[11] - a[12] * a[7] * a[9];
    inv[12] = -a[4] * a[9] * a[14] + a[4] * a[10] * a[13] + a[8] * a[5] * a[14] -
              a[8] * a[6] * a[13] - a[12] * a[5] * a[10] + a[12] * a[6] * a[9];
    inv[1] = -a[1] * a[10] * a[15] + a[1] * a[11] * a[14] + a[9] * a[2] * a[15] -
             a[9] * a[3] * a[14] - a[13] * a[2] * a[11] + a[13] * a[3] * a[10];
    inv[5] = a[0] * a[10] * a[15] - a[0] * a[11] * a[14] - a[8] * a[2] * a[15] +
             a[8] * a[3] * a[14] + a[12] * a[2] * a[11] - a[12] * a[3] * a[10];
    inv[9] = -a[0] * a[9] * a[15] + a[0] * a[11] * a[13] + a[8] * a[1] * a[15] -
             a[8] * a[3] * a[13] - a[12] * a[1] * a[11] + a[12] * a[3] * a[9];
    inv[13] = a[0] * a[9] * a[14] - a[0] * a[10] * a[13] - a[8] * a[1] * a[14] +
              a[8] * a[2] * a[13] + a[12] * a[1] * a[10] - a[12] * a[2] * a[9];
    inv[2] = a[1] * a[6] * a[15] - a[1] * a[7] * a[14] - a[5] * a[2] * a[15] +
             a[5] * a[3] * a[14] + a[13] * a[2] * a[7] - a[13] * a[3] * a[6];
    inv[6] = -a[0] * a[6] * a[15] + a[0] * a[7] * a[14] + a[4] * a[2] * a[15] -
             a[4] * a[3] * a[14] - a[12] * a[2] * a[7] + a[12] * a[3] * a[6];
    inv[10] = a[0] * a[5] * a[15] - a[0] * a[7] * a[13] - a[4] * a[1] * a[15] +
              a[4] * a[3] * a[13] + a[12] * a[1] * a[7] - a[12] * a[3] * a[5];
    inv[14] = -a[0] * a[5] * a[14] + a[0] * a[6] * a[13] + a[4] * a[1] * a[14] -
              a[4] * a[2] * a[13] - a[12] * a[1] * a[6] + a[12] * a[2] * a[5];
    inv[3] = -a[1] * a[6] * a[11] + a[1] * a[7] * a[10] + a[5] * a[2] * a[11] -
             a[5] * a[3] * a[10] - a[9] * a[2] * a[7] + a[9] * a[3] * a[6];
    inv[7] = a[0] * a[6] * a[11] - a[0] * a[7] * a[10] - a[4] * a[2] * a[11] +
             a[4] * a[3] * a[10] + a[8] * a[2] * a[7] - a[8] * a[3] * a[6];
    inv[11] = -a[0] * a[5] * a[11] + a[0] * a[7] * a[9] + a[4] * a[1] * a[11] -
              a[4] * a[3] * a[9] - a[8] * a[1] * a[7] + a[8] * a[3] * a[5];
    inv[15] = a[0] * a[5] * a[10] - a[0] * a[6] * a[9] - a[4] * a[1] * a[10] +
              a[4] * a[2] * a[9] + a[8] * a[1] * a[6] - a[8] * a[2] * a[5];
    const double det = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] +
                       a[3] * inv[12];
    if (!(std::abs(det) > 1e-300) || !std::isfinite(det)) {
        return false;
    }
    for (int i = 0; i < 16; ++i) {
        (*out)[i] = inv[i] / det;
    }
    return true;
}

inline void
Origin(const M4 &m, double out[3])
{
    out[0] = m[12];
    out[1] = m[13];
    out[2] = m[14];
}

inline double
Dot3(const double *a, const double *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline double
Length3(const double *a)
{
    return std::sqrt(Dot3(a, a));
}

/// Rotation about one axis (0, 1, 2) by \p degrees, as GfMatrix4d builds it
/// from GfRotation for row vectors.
inline M4
AxisRotation(int axis, double degrees)
{
    const double a = degrees * std::acos(-1.0) / 180.0;
    const double c = std::cos(a), s = std::sin(a);
    M4 m = Identity();
    const int i = (axis + 1) % 3, j = (axis + 2) % 3;
    m[i * 4 + i] = c;
    m[i * 4 + j] = s;
    m[j * 4 + i] = -s;
    m[j * 4 + j] = c;
    return m;
}

inline int
AxisOf(char c)
{
    return c == 'Y' ? 1 : c == 'Z' ? 2 : 0;
}

/// RigExecProviderComposeAvars over raw channels: S * R(order) * Rspin * T,
/// with rotations applied as r * sign and translations as t * units.
inline M4
Compose(const RigExecIkFkMatchChannels &ch)
{
    M4 m = Identity();
    for (int k = 0; k < 3; ++k) {
        m[k * 4 + k] = RigExecNormalizeAvarScale(ch.s[k]);
    }
    for (int k = 0; k < 3; ++k) {
        const int axis = AxisOf(ch.order[k]);
        const double angle = ch.r[axis] * ch.sign[axis];
        if (angle != 0.0) {
            m = Mul(m, AxisRotation(axis, angle));
        }
    }
    if (ch.spin != 0.0) {
        m = Mul(m, AxisRotation(0, ch.spin * ch.sign[0]));
    }
    M4 t = Identity();
    for (int k = 0; k < 3; ++k) {
        t[12 + k] = ch.t[k] * ch.units;
    }
    return Mul(m, t);
}

/// The rotation rows of \p m made orthonormal, X first: a frame stretched
/// along its own bone axis comes back to the rotation it was stretched from.
inline M4
Orthonormal(const M4 &m)
{
    double x[3] = {m[0], m[1], m[2]}, y[3] = {m[4], m[5], m[6]};
    const double lx = Length3(x);
    for (double &v : x) v /= lx;
    const double d = Dot3(x, y);
    for (int k = 0; k < 3; ++k) y[k] -= d * x[k];
    const double ly = Length3(y);
    for (double &v : y) v /= ly;
    double z[3] = {x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2],
                   x[0] * y[1] - x[1] * y[0]};
    // Keep the input's handedness (a mirrored frame stays mirrored).
    const double zin[3] = {m[8], m[9], m[10]};
    if (Dot3(z, zin) < 0) {
        for (double &v : z) v = -v;
    }
    M4 out = Identity();
    for (int k = 0; k < 3; ++k) {
        out[k] = x[k];
        out[4 + k] = y[k];
        out[8 + k] = z[k];
    }
    return out;
}

inline double
Unwrap(double angle, double hint)
{
    return angle + 360.0 * std::round((hint - angle) / 360.0);
}

/// Euler angles (degrees, indexed by axis) whose rotation in \p order
/// reproduces \p rot, the solution nearest \p hint and unwrapped to its
/// turn. With column-vector C = R^T and order (i, j, k), C = Rk Rj Ri,
/// parity e = +1 for cyclic orders: sin(b) = -e C[k][i], a = atan2(e C[k][j],
/// C[k][k]), g = atan2(e C[j][i], C[i][i]).
inline void
DecomposeEuler(const M4 &rot, const char order[3], const double hint[3],
               double out[3])
{
    const int i = AxisOf(order[0]), j = AxisOf(order[1]), k = AxisOf(order[2]);
    const bool cyclic = (j == (i + 1) % 3);
    const double e = cyclic ? 1.0 : -1.0;
    const auto C = [&](int r, int c) { return rot[c * 4 + r]; };
    const double cosB = std::hypot(C(k, j), C(k, k));
    const double b = std::atan2(-e * C(k, i), cosB);
    double a, g;
    const double deg = 180.0 / std::acos(-1.0);
    if (cosB > 1e-9) {
        a = std::atan2(e * C(k, j), C(k, k));
        g = std::atan2(e * C(j, i), C(i, i));
    } else {
        // Gimbal lock: keep the hint's first angle, solve the last.
        a = hint[i] / deg;
        const M4 partial = Mul(AxisRotation(i, hint[i]), AxisRotation(j, b * deg));
        M4 inverse;
        if (!Inverse(partial, &inverse)) inverse = Identity();
        const M4 residual = Mul(inverse, rot);
        g = std::atan2(e * residual[i * 4 + j], residual[i * 4 + i]);
    }
    double first[3], second[3];
    first[i] = a * deg;
    first[j] = b * deg;
    first[k] = g * deg;
    second[i] = first[i] + 180.0;
    second[j] = 180.0 - first[j];
    second[k] = first[k] + 180.0;
    double best = 0, chosen[3] = {0, 0, 0};
    for (int n = 0; n < 2; ++n) {
        const double *candidate = n == 0 ? first : second;
        double unwrapped[3], distance = 0;
        for (int m = 0; m < 3; ++m) {
            unwrapped[m] = Unwrap(candidate[m], hint[m]);
            distance += std::abs(unwrapped[m] - hint[m]);
        }
        if (n == 0 || distance < best) {
            best = distance;
            std::copy(unwrapped, unwrapped + 3, chosen);
        }
    }
    std::copy(chosen, chosen + 3, out);
}

/// Translate and rotate channels that put a control's frame on \p target,
/// \p parent being what its avars compose onto. Scale and spin are kept: a
/// match never writes them.
inline bool
Solve(const M4 &target, const M4 &parent, RigExecIkFkMatchChannels *ch)
{
    M4 parentInverse;
    if (!Inverse(parent, &parentInverse)) return false;
    const M4 local = Mul(target, parentInverse);
    M4 linear = local;
    linear[12] = linear[13] = linear[14] = 0;
    M4 scale = Identity(), spin = Identity();
    for (int k = 0; k < 3; ++k) {
        scale[k * 4 + k] = 1.0 / RigExecNormalizeAvarScale(ch->s[k]);
    }
    if (ch->spin != 0.0) {
        spin = AxisRotation(0, -ch->spin * ch->sign[0]);
    }
    const M4 rotation = Orthonormal(Mul(Mul(scale, linear), spin));
    double hint[3], angles[3];
    for (int k = 0; k < 3; ++k) hint[k] = ch->r[k] * ch->sign[k];
    DecomposeEuler(rotation, ch->order, hint, angles);
    const double units = ch->units != 0.0 ? ch->units : 1.0;
    for (int k = 0; k < 3; ++k) {
        ch->r[k] = angles[k] * ch->sign[k];
        ch->t[k] = local[12 + k] / units;
    }
    for (int k = 0; k < 3; ++k) {
        if (!std::isfinite(ch->r[k]) || !std::isfinite(ch->t[k])) return false;
    }
    return true;
}

/// \p v turned by \p radians about the unit \p axis (right-handed).
inline void
Rotate(const double v[3], const double axis[3], double radians, double out[3])
{
    const double c = std::cos(radians), s = std::sin(radians);
    const double d = Dot3(axis, v);
    const double cross[3] = {axis[1] * v[2] - axis[2] * v[1],
                             axis[2] * v[0] - axis[0] * v[2],
                             axis[0] * v[1] - axis[1] * v[0]};
    for (int k = 0; k < 3; ++k) {
        out[k] = v[k] * c + cross[k] * s + axis[k] * d * (1 - c);
    }
}

}  // namespace ikFkMatchKernel

/// The two-bone solve's rest bone lengths and soft-distance factor, as
/// RigExecTwoBoneIkLengths and RigExecTwoBoneIkSpaceFactor measure them: the
/// rest joint origins carried by \p space (null for identity), each authored
/// offset riding its own bone's factor.
inline void
RigExecIkFkMatchRestLengths(const double origins[3][3], const double *space,
                            double upperOffset, double lowerOffset,
                            double *upper, double *lower, double *factor)
{
    double spaced[3][3];
    for (int i = 0; i < 3; ++i) {
        if (!space) {
            for (int c = 0; c < 3; ++c) spaced[i][c] = origins[i][c];
            continue;
        }
        // GfMatrix4d::Transform: p * M, divided by w.
        double out[4];
        for (int c = 0; c < 4; ++c) {
            out[c] = origins[i][0] * space[c] + origins[i][1] * space[4 + c] +
                     origins[i][2] * space[8 + c] + space[12 + c];
        }
        const double w = out[3] != 0.0 ? out[3] : 1.0;
        for (int c = 0; c < 3; ++c) spaced[i][c] = out[c] / w;
    }
    const auto distance = [](const double *a, const double *b) {
        const double d[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        return std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    };
    const double rawUpper = distance(origins[0], origins[1]);
    const double rawLower = distance(origins[1], origins[2]);
    const double spacedUpper = distance(spaced[0], spaced[1]);
    const double spacedLower = distance(spaced[1], spaced[2]);
    *upper = spacedUpper +
             upperOffset * (rawUpper > 1e-12 ? spacedUpper / rawUpper : 1.0);
    *lower = spacedLower +
             lowerOffset * (rawLower > 1e-12 ? spacedLower / rawLower : 1.0);
    const double raw = rawUpper + rawLower;
    *factor = raw > 1e-12 ? (spacedUpper + spacedLower) / raw : 1.0;
}

/// The upper and lower stretch scales whose RigExecLimbSegmentLengths are
/// \p want: Newton's method with a finite-difference Jacobian, started from
/// \p limb's scales. The map is smooth and nearly diagonal; a fully pinned
/// limb, whose lengths ignore the scales, keeps the start.
inline void
RigExecIkFkMatchSolveStretch(double restUpper, double restLower, double dist,
                             double poleUpper, double poleLower,
                             const RigExecLimbStretch &limb,
                             const double want[2], double *upperScale,
                             double *lowerScale)
{
    RigExecLimbStretch p = limb;
    for (int iteration = 0; iteration < 30; ++iteration) {
        double u, l, u1, l1, u2, l2;
        RigExecLimbSegmentLengths(restUpper, restLower, dist, poleUpper,
                                  poleLower, p, &u, &l);
        const double eu = u - want[0], el = l - want[1];
        if (std::abs(eu) < 1e-12 && std::abs(el) < 1e-12) break;
        const double h = 1e-6;
        RigExecLimbStretch q = p;
        q.upperScale += h;
        RigExecLimbSegmentLengths(restUpper, restLower, dist, poleUpper,
                                  poleLower, q, &u1, &l1);
        q = p;
        q.lowerScale += h;
        RigExecLimbSegmentLengths(restUpper, restLower, dist, poleUpper,
                                  poleLower, q, &u2, &l2);
        const double j00 = (u1 - u) / h, j01 = (u2 - u) / h;
        const double j10 = (l1 - l) / h, j11 = (l2 - l) / h;
        const double det = j00 * j11 - j01 * j10;
        if (!(std::abs(det) > 1e-14)) break;
        p.upperScale -= (j11 * eu - j01 * el) / det;
        p.lowerScale -= (-j10 * eu + j00 * el) / det;
        if (!std::isfinite(p.upperScale) || !std::isfinite(p.lowerScale)) {
            p = limb;
            break;
        }
    }
    *upperScale = p.upperScale;
    *lowerScale = p.lowerScale;
}

/// IK control channels, pole target and stretch scales that make the IK half
/// reproduce the joints.
///
/// The solve ends its chain on the effector's origin and copies the
/// effector's orientation onto the end joint, so the effector's target is
/// the end joint's frame; the IK control's target follows by the effector's
/// current offset from it. The pole goes on the bend plane, turned back by
/// the solve's twist, \p poleDistance past the middle joint. The stretch
/// scales are solved so the bones come out the joints' lengths.
inline void
RigExecIkFkMatchIk(const RigExecIkFkMatchIkInputs &in,
                   RigExecIkFkMatchIkResult *out)
{
    namespace k = ikFkMatchKernel;
    out->ok = false;
    out->ik = in.ik;
    if (!in.joints[0] || !in.joints[1] || !in.joints[2] || !in.root ||
        !in.ikPosed || !in.effectorPosed) {
        return;
    }
    const k::M4 end = k::Load(in.joints[2]);
    const k::M4 control = k::Load(in.ikPosed);
    const k::M4 effector = k::Load(in.effectorPosed);
    k::M4 controlInverse, avarsInverse, carryInverse;
    if (!k::Inverse(control, &controlInverse)) return;
    // effector = carry * control; target control = carry^-1 * end.
    const k::M4 carry = k::Mul(effector, controlInverse);
    if (!k::Inverse(carry, &carryInverse)) return;
    const k::M4 target = k::Mul(carryInverse, end);
    if (!k::Inverse(k::Compose(in.ik), &avarsInverse)) return;
    const k::M4 parent = k::Mul(avarsInverse, control);
    if (!k::Solve(target, parent, &out->ik)) return;
    const k::M4 frame = k::Mul(k::Compose(out->ik), parent);
    std::copy(frame.begin(), frame.end(), out->ikFrame);

    double root[3], mid[3], tip[3];
    k::Origin(k::Load(in.root), root);
    k::Origin(k::Load(in.joints[1]), mid);
    k::Origin(end, tip);
    double aim[3] = {tip[0] - root[0], tip[1] - root[1], tip[2] - root[2]};
    const double dist = k::Length3(aim);
    const double upper[3] = {mid[0] - root[0], mid[1] - root[1],
                             mid[2] - root[2]};
    double pole[3] = {mid[0], mid[1], mid[2]};
    const double pin = std::min(std::max(in.limb.pin, 0.0), 1.0);
    const double distance = pin > 0.0 ? 0.0 : in.poleDistance;
    if (dist > 1e-12) {
        for (double &v : aim) v /= dist;
        const double along = k::Dot3(upper, aim);
        double perp[3] = {upper[0] - aim[0] * along, upper[1] - aim[1] * along,
                          upper[2] - aim[2] * along};
        double bendLength = k::Length3(perp);
        double bend[3];
        if (bendLength > 1e-4 * dist) {
            for (int c = 0; c < 3; ++c) bend[c] = perp[c] / bendLength;
        } else if (in.polePosed) {
            // Too straight for a bend plane: keep the pole's side.
            double current[3];
            k::Origin(k::Load(in.polePosed), current);
            double side[3] = {current[0] - mid[0], current[1] - mid[1],
                              current[2] - mid[2]};
            const double d = k::Dot3(side, aim);
            for (int c = 0; c < 3; ++c) side[c] -= aim[c] * d;
            const double sl = k::Length3(side);
            if (sl > 1e-12) {
                for (int c = 0; c < 3; ++c) bend[c] = side[c] / sl;
            } else {
                bendLength = -1;
            }
            if (bendLength >= 0) bendLength = 0;
        } else {
            bendLength = -1;
        }
        if (bendLength >= 0) {
            double turned[3];
            k::Rotate(bend, aim, -in.twistRadians, turned);
            for (int c = 0; c < 3; ++c) {
                pole[c] = root[c] + aim[c] * along +
                          turned[c] * (bendLength + distance);
            }
        }
    }
    std::copy(pole, pole + 3, out->poleTarget);

    out->upperScale = in.limb.upperScale;
    out->lowerScale = in.limb.lowerScale;
    if (in.hasStretch) {
        const double want[2] = {
            k::Length3(upper),
            std::sqrt((tip[0] - mid[0]) * (tip[0] - mid[0]) +
                      (tip[1] - mid[1]) * (tip[1] - mid[1]) +
                      (tip[2] - mid[2]) * (tip[2] - mid[2]))};
        const double toPole[3] = {pole[0] - root[0], pole[1] - root[1],
                                  pole[2] - root[2]};
        const double fromPole[3] = {tip[0] - pole[0], tip[1] - pole[1],
                                    tip[2] - pole[2]};
        RigExecIkFkMatchSolveStretch(in.restUpper, in.restLower, dist,
                                     k::Length3(toPole), k::Length3(fromPole),
                                     in.limb, want, &out->upperScale,
                                     &out->lowerScale);
    }
    out->ok = std::isfinite(out->upperScale) && std::isfinite(out->lowerScale);
}

/// The pole's translate channels that put its origin on \p target, with
/// \p parentAfter the frame its avars compose onto once the IK control has
/// moved, and the pole's frame then.
inline bool
RigExecIkFkMatchPole(const double target[3], const double *parentAfter,
                     RigExecIkFkMatchChannels *pole, double frame[16])
{
    namespace k = ikFkMatchKernel;
    const k::M4 parent = k::Load(parentAfter);
    // origin = t * units * (S R Rspin) * P; solve the translation row.
    k::M4 linear = k::Compose(*pole);
    linear[12] = linear[13] = linear[14] = 0;
    const k::M4 combined = k::Mul(linear, parent);
    k::M4 inverse;
    if (!k::Inverse(combined, &inverse)) return false;
    // A point p maps as p * combined; the origin of A * P is t' * P where t'
    // is the translation row of A, so solve t' * P = target directly.
    k::M4 parentInverse;
    if (!k::Inverse(parent, &parentInverse)) return false;
    const double units = pole->units != 0.0 ? pole->units : 1.0;
    for (int c = 0; c < 3; ++c) {
        double v = parentInverse[12 + c];
        for (int r = 0; r < 3; ++r) v += target[r] * parentInverse[r * 4 + c];
        pole->t[c] = v / units;
        if (!std::isfinite(pole->t[c])) return false;
    }
    const k::M4 posed = k::Mul(k::Compose(*pole), parent);
    std::copy(posed.begin(), posed.end(), frame);
    return true;
}

/// FK control channels and stretch scales that make the FK half reproduce
/// the joints. Each control's target is its rest offset carried by its
/// joint; the parent of controls 1 and 2 rides the control above as the
/// match moves it, with the bone offset lengthened in proportion to the new
/// stretch scale.
inline void
RigExecIkFkMatchFk(const RigExecIkFkMatchFkInputs &in,
                   RigExecIkFkMatchFkResult *out)
{
    namespace k = ikFkMatchKernel;
    out->ok = false;
    for (int i = 0; i < 3; ++i) {
        out->fk[i] = in.fk[i];
        if (!in.joints[i] || !in.fkPosed[i] || !in.offsets[i]) return;
    }
    double points[3][3];
    for (int i = 0; i < 3; ++i) k::Origin(k::Load(in.joints[i]), points[i]);
    const auto span = [&](int a, int b) {
        const double d[3] = {points[b][0] - points[a][0],
                             points[b][1] - points[a][1],
                             points[b][2] - points[a][2]};
        return k::Length3(d);
    };
    out->upperScale = in.upperScale;
    out->lowerScale = in.lowerScale;
    double ratio[3] = {1, 1, 1};
    if (in.hasStretch && in.restUpper > 1e-12 && in.restLower > 1e-12) {
        out->upperScale = span(0, 1) / in.restUpper;
        out->lowerScale = span(1, 2) / in.restLower;
        if (std::abs(in.upperScale) > 1e-12) {
            ratio[1] = out->upperScale / in.upperScale;
        }
        if (std::abs(in.lowerScale) > 1e-12) {
            ratio[2] = out->lowerScale / in.lowerScale;
        }
    }
    k::M4 previousOld{}, previousNew{};
    for (int i = 0; i < 3; ++i) {
        const k::M4 posed = k::Load(in.fkPosed[i]);
        k::M4 avarsInverse;
        if (!k::Inverse(k::Compose(in.fk[i]), &avarsInverse)) return;
        k::M4 parent = k::Mul(avarsInverse, posed);
        if (i > 0) {
            k::M4 previousInverse;
            if (!k::Inverse(previousOld, &previousInverse)) return;
            k::M4 local = k::Mul(parent, previousInverse);
            for (int c = 0; c < 3; ++c) local[12 + c] *= ratio[i];
            parent = k::Mul(local, previousNew);
        }
        const k::M4 target = k::Mul(k::Load(in.offsets[i]),
                                    k::Load(in.joints[i]));
        if (!k::Solve(target, parent, &out->fk[i])) return;
        const k::M4 frame = k::Mul(k::Compose(out->fk[i]), parent);
        std::copy(frame.begin(), frame.end(), out->frames[i]);
        previousOld = posed;
        previousNew = frame;
    }
    out->ok = true;
}

/// The published layouts: outputs:ikValues = [ikControl t3 r3, pole t3,
/// upperScale, lowerScale] and outputs:fkValues = [fkControls[0..2] t3 r3,
/// upperScale, lowerScale].
inline void
RigExecIkFkMatchPack(const RigExecIkFkMatchIkResult &ik,
                     const RigExecIkFkMatchChannels &pole,
                     const RigExecIkFkMatchFkResult &fk, double ikValues[11],
                     double fkValues[20])
{
    for (int k = 0; k < 3; ++k) {
        ikValues[k] = ik.ik.t[k];
        ikValues[3 + k] = ik.ik.r[k];
        ikValues[6 + k] = pole.t[k];
    }
    ikValues[9] = ik.upperScale;
    ikValues[10] = ik.lowerScale;
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
            fkValues[i * 6 + k] = fk.fk[i].t[k];
            fkValues[i * 6 + 3 + k] = fk.fk[i].r[k];
        }
    }
    fkValues[18] = fk.upperScale;
    fkValues[19] = fk.lowerScale;
}

/// Which half is presented on the joints: the IK half while the switch sits
/// exactly on its FK value, the FK half on its IK value, neither between or
/// under a zero envelope.
enum class RigExecIkFkMatchShown { None, IkSide, FkSide };

inline RigExecIkFkMatchShown
RigExecIkFkMatchShownSide(double switchValue, double ikValue, double weight)
{
    if (!(weight > 0.0)) return RigExecIkFkMatchShown::None;
    if (std::abs(switchValue - (1.0 - ikValue)) <= 1e-9)
        return RigExecIkFkMatchShown::IkSide;
    if (std::abs(switchValue - ikValue) <= 1e-9)
        return RigExecIkFkMatchShown::FkSide;
    return RigExecIkFkMatchShown::None;
}

}  // namespace rigExec

#endif
