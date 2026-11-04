// 2D skeleton solve: the one numerical body behind RigExecSkeleton2d in
// native, frozen and .rigexec evaluation.
//
// USD-free and header-only: the runtime instantiates it. No locks, statics
// or diagnostics; the workspace is owned by the calling operation.
//
// Plane and units: bones lie in the XY plane of the skeleton's start frame
// (the rig's asset space without one), angles are degrees about its +Z,
// counter-clockwise. RigExecSolveSkeleton2dJoints expresses the rests and
// the outside parent spaces in the start frame's rest frame, solves there,
// and composes the published frames back through the start frame's posed
// frame, as RigExecFkChain hangs its chain from it. A frame is a 2D affine
// map stored as origin plus the images of the unit X and Y axes.
//
// Per bone i with parent space p(t) (row-vector order, p' = p * M): its
// namespace parent bone, or what a space switch selects this frame -- a
// bone, an outside frame, or the base (the plane's origin frame):
//   controls       a kernel pre-pass revises tx/ty/rz/sx from the
//                  controller's channels (delta form)
//   rest local     L0_i = Rest_i * Rest_p(t)^-1  (position x0_i, angle a0_i)
//   posed local    x_i = x0_i + (tx, ty) + (offsetX, offsetY) with x scaled
//                  by the parent's sx; a_i = a0_i + rz, s_i = sx
//   pose frame     P_i = Flip_i * Rot(a_i) * T(x_i) * P_p
//                  (independentAngle: Rot(restWorldAngle_i + rz), base from P_p)
//                  A fractional switch index blends the frames under its two
//                  neighbouring spaces: origin linearly, angle along the
//                  shortest arc, as the space switch blends its spaces.
//   lock solve     cyclic coordinate descent over the lock chain so a locked
//                  bone's base and world angle meet bone2d:lockX/lockY and
//                  lockAngle, within limits (method reference: the "Cyclic
//                  coordinate descent" row of docs/references.md); a lock
//                  the descent leaves short retries from the other bend
//                  (two joints: the closed-form bends; longer chains: a
//                  mirrored start) and keeps a strictly closer legal result
//   target solve   after locks: cyclic coordinate descent over the target
//                  chain (the owner plus its ancestors) so the owner's tip
//                  -- its first joint child's setup base, carried by its
//                  along-bone scale -- meets its target bone's base; a lone
//                  bone aims, with stretch-to-reach up to maxStretch
//   deform frame   Scale(s_i, cross_i) * P_i, published to the joint; a bone
//                  under an inactive group member publishes its rest frame
// Children compose from the pose frame, never the deform frame, so a
// bone's cross scale never shears its children.
//
// The lock and target rules, retries, chain cap, control bones and groups
// have no public source; they come from an unpublished internal
// specification (see docs/references.md, "2D skeleton rules without a
// public source").
//
// Rigging inputs (RigExecSkeleton2dStatic, all solve-order indices): per-bone
// parent spaces of a space switch, selected per evaluation by its active
// index; per-bone target candidates selected by bone2d:targetIndex (-1:
// none); bone groups whose active member index arrives per evaluation. The
// static solve order covers the union of the namespace parents and every
// switch source (the compile refuses a cyclic union), so no per-frame
// reorder is needed. Locks win over targets: a locked bone keeps its hold
// and target chains stop before locked bones. A bone whose parent space is
// not a bone of the skeleton is a chain root.
//
// Rests are the bones' setup frames, absolute: the composed rest:space or,
// where a pose step below the skeleton wrote the bone, that step's frame
// (the solver live-rest route every aggregate solver uses). A rest must be
// a rotation about the plane's +Z plus a translation.
//
// A bone's world angle is the direction of its pose frame's X axis. Pose
// frames are orthonormal (det +1, or -1 under an odd number of flips).
#ifndef RIGEXEC_MATH_SKELETON2D_KERNEL_H
#define RIGEXEC_MATH_SKELETON2D_KERNEL_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

namespace rigExec {

enum class RigExecBone2dScaling : uint8_t { Normal = 0, SquashStretch = 1, Uniform = 2 };

/// Bit flags of RigExecBone2dSetup::flags.
enum : uint8_t {
    kRigExecBone2dIndependentAngle = 1u << 0,
    kRigExecBone2dAngleLimits = 1u << 1,
    kRigExecBone2dIgnoredByIk = 1u << 2,
    /// The bone has a tip (a joint child); without it the bone is a pin.
    kRigExecBone2dTip = 1u << 3,
};

/// A parent space in RigExecSkeleton2dStatic::spaceSources: a bone of the
/// skeleton (>= 0, solve order), the base (the plane's origin frame: the
/// start frame, or asset space), or outside space e
/// (RigExecBone2dSpaceExternal(e)).
inline constexpr int32_t kRigExecBone2dSpaceBase = -1;
inline constexpr int32_t RigExecBone2dSpaceExternal(size_t e) { return -2 - int32_t(e); }
/// The outside space a source names, or -1 for a bone or the base.
inline constexpr int64_t RigExecBone2dSpaceExternalIndex(int32_t source)
{
    return source <= -2 ? int64_t(-2) - int64_t(source) : int64_t(-1);
}

/// Longest lock or target chain, in bones.
inline constexpr size_t kRigExecSkeleton2dMaxChain = 32;
/// Fixed sweep cap of the lock solve, so every evaluator iterates alike; a
/// sweep that turns nothing, or an exact hit, ends it early. Coordinate
/// descent converges linearly and slowest on a nearly straight chain that
/// has to fold: two-bone legs locked 0.02 to 0.1 inside their reach take
/// 150 to 700 sweeps to reach rounding (16 sweeps still miss by 1e-2). A
/// sweep turns each chain bone once and recomposes only the chain, so its
/// cost is the chain length squared.
inline constexpr int kRigExecSkeleton2dLockSweeps = 1024;
/// A lock is reached when the locked base is within this of its reference,
/// relative to the chain's reach (max(1, reach)). Retries run only past it.
inline constexpr double kRigExecSkeleton2dLockReachTolerance = 1e-9;
/// A closed-form turn within this many degrees outside a limit is clamped onto it.
inline constexpr double kRigExecSkeleton2dLimitSlack = 1e-9;
/// |cross| below this * l1 * l2: a straight chain (no bend side).
inline constexpr double kRigExecSkeleton2dBendSideEpsilon = 1e-9;
/// The thirteen channels a bone reads, in RigExecBone2dChannels order.
inline constexpr size_t kRigExecBone2dChannelCount = 13;
/// Whether channel \p c (sideFlip, endFlip, lock) is a bool.
inline constexpr bool RigExecBone2dChannelIsBool(size_t c) { return c >= 4 && c <= 6; }
/// Baked per-bone storage: ten doubles (tx, ty, rz, sx, lockX, lockY,
/// lockAngle, offsetX, offsetY, targetIndex) and three bools (sideFlip,
/// endFlip, lock).
inline constexpr size_t kRigExecBone2dDoubleChannels = kRigExecBone2dChannelCount - 3;
inline constexpr size_t kRigExecBone2dBoolChannels = 3;
/// Where channel \p c sits in its type's storage (the bool or double slot).
inline constexpr size_t RigExecBone2dChannelStorage(size_t c)
{
    return RigExecBone2dChannelIsBool(c) ? c - 4 : (c < 4 ? c : c - 3);
}
/// Along-bone scales whose magnitude is below this get no cross scale.
inline constexpr double kRigExecBone2dMinScale = 1e-6;

/// Structural per-bone facts: the hierarchy and the uniform
/// RigExecBone2dAPI settings, compiled into the record (an edit recompiles).
/// Bones are ordered so a parent precedes its children (the compiler sorts;
/// rigExec:joints order is kept for output). The solve order covers the
/// union of the namespace parents and every space-switch source, so any
/// per-frame parent comes before its child.
struct RigExecBone2dSetup {
    int32_t parent = -1;                ///< namespace parent bone, -1 root
    uint8_t flags = 0;
    double minAngle = 0, maxAngle = 0;  ///< degrees, relative to the setup local angle
    RigExecBone2dScaling scaling = RigExecBone2dScaling::Normal;
    double squashFactor = 1;
    double maxStretch = 1;
    /// The tip (with kRigExecBone2dTip): bone tipBone's setup base when the
    /// first joint child is a bone of the skeleton (solve order), otherwise
    /// (tipX, tipY) in this bone's setup frame (an End joint's rest).
    int32_t tipBone = -1;
    double tipX = 0, tipY = 0;
    /// Parent spaces of a space switch on the bone, in rigExec:sources
    /// order, in RigExecSkeleton2dStatic::spaceSources. Empty (count 0)
    /// keeps the namespace parent every frame. spaceSelector names the
    /// switch's active index among the selectors (-1 without a switch).
    uint32_t spaceBegin = 0, spaceCount = 0;
    int32_t spaceSelector = -1;
    /// Target candidates (bone2d:targetCandidates) in
    /// RigExecSkeleton2dStatic::targetCandidates; empty keeps no target.
    uint32_t targetBegin = 0, targetCount = 0;
    /// Control bones (solve-order indices, -1 none) with their factors.
    /// A zero factor, or no controller, leaves that kind alone.
    // Delta form: a controller adds its change from rest, scaled by the factor.
    int32_t angleController = -1, posController = -1, scaleController = -1;
    double angleFactor = 0, posFactorX = 0, posFactorY = 0, scaleFactor = 0;
};

/// One alternative-chain bone group: alternative chains whose first
/// bones are the members. Structural, except the active member, which
/// arrives per evaluation. A disabled group keeps every member active.
struct RigExecBone2dGroup {
    /// Members in RigExecSkeleton2dStatic::groupMembers (solve order).
    uint32_t memberBegin = 0, memberCount = 0;
    bool enabled = true;
};

/// Parent-space, target and group arrays, shared by reference (never copied
/// per frame). All bone indices are solve-order indices. Counts bound every
/// range the kernel reads, so a malformed record fails safely.
struct RigExecSkeleton2dStatic {
    /// Concatenated parent spaces (kRigExecBone2dSpaceBase and
    /// RigExecBone2dSpaceExternal); RigExecBone2dSetup ranges.
    const int32_t *spaceSources = nullptr;
    size_t spaceSourceCount = 0;
    /// Concatenated target candidates; RigExecBone2dSetup ranges.
    const int32_t *targetCandidates = nullptr;
    size_t targetCandidateCount = 0;
    const RigExecBone2dGroup *groups = nullptr;
    size_t groupCount = 0;
    /// Concatenated group members; RigExecBone2dGroup ranges.
    const int32_t *groupMembers = nullptr;
    size_t groupMemberCount = 0;
};

/// A bone's setup frame, planar: a value. The solver rest route refreshes
/// it from rest:space whenever that rest moves, without a recompile.
struct RigExecBone2dRest {
    double x = 0, y = 0;   ///< setup base in the plane
    double angle = 0;      ///< setup world angle, degrees
};

/// One evaluation's values besides the channels: each outside parent
/// space's setup frame and posed placement in the plane (entry e for
/// RigExecBone2dSpaceExternal(e)), and the selectors -- each group's active
/// member index, then each space switch's active index.
struct RigExecSkeleton2dFrameInputs {
    const RigExecBone2dRest *externalRests = nullptr;
    const RigExecBone2dRest *externalPoses = nullptr;
    size_t externalCount = 0;
    const double *selectors = nullptr;
    size_t selectorCount = 0;
};

/// One evaluation's channel values, read at the boundary. A channel that
/// property movers revise arrives as that chain's result.
struct RigExecBone2dChannels {
    double tx = 0, ty = 0, rz = 0, sx = 1;
    bool sideFlip = false, endFlip = false, lock = false;
    double lockX = 0, lockY = 0, lockAngle = 0;
    // The offset sits in the parent's frame.
    double offsetX = 0, offsetY = 0;
    /// Index into the bone's target candidates (rounded; negative or out
    /// of range keeps no target).
    double targetIndex = -1;
};

/// Sets channel \p c of \p channels (avars:tx, ty, rz, sx, sideFlip,
/// endFlip, lock, lockX, lockY, lockAngle, offsetX, offsetY, targetIndex;
/// a bool channel is true when \p value is not 0).
inline void RigExecBone2dSetChannel(RigExecBone2dChannels *channels, size_t c,
    double value)
{
    switch (c) {
    case 0: channels->tx = value; break;
    case 1: channels->ty = value; break;
    case 2: channels->rz = value; break;
    case 3: channels->sx = value; break;
    case 4: channels->sideFlip = value != 0; break;
    case 5: channels->endFlip = value != 0; break;
    case 6: channels->lock = value != 0; break;
    case 7: channels->lockX = value; break;
    case 8: channels->lockY = value; break;
    case 9: channels->lockAngle = value; break;
    case 10: channels->offsetX = value; break;
    case 11: channels->offsetY = value; break;
    case 12: channels->targetIndex = value; break;
    default: break;
    }
}

/// A 2D affine frame: p' = origin + p.x * xAxis + p.y * yAxis.
struct RigExecBone2dFrame {
    double originX = 0, originY = 0;
    double xAxisX = 1, xAxisY = 0;
    double yAxisX = 0, yAxisY = 1;
};

/// Scratch the solve reuses; owned by the operation's workspace. Every
/// vector is resized by the solve; nothing in it outlives one call.
struct RigExecSkeleton2dWorkspace {
    std::vector<RigExecBone2dFrame> pose;
    /// Per bone: the solved rz and along-bone scale. Per bone and parent
    /// space (index 2i under space spaceA[i], 2i + 1 under spaceB[i]): the
    /// angle the bone turns from (setup local angle, or the setup world
    /// angle for an independent bone), the rest local and solved local
    /// positions.
    std::vector<double> localAngle, scale;
    std::vector<double> baseAngle, restLocalX, restLocalY, localX, localY;
    /// Per bone: this frame's parent spaces (spaceB blends in by
    /// spaceBlend; equal to spaceA when it does not) and the tip in the
    /// bone's setup frame (zero for a pin).
    std::vector<int32_t> spaceA, spaceB;
    std::vector<double> spaceBlend, tipX, tipY;
    /// Each outside parent space's posed frame this evaluation.
    std::vector<RigExecBone2dFrame> external;
    std::vector<int32_t> chain;
    std::vector<int32_t> countedChildren;
    std::vector<uint8_t> dirty;
    /// The setup with each bone's chain parent this frame (the bone of its
    /// nearer parent space, -1 outside the skeleton), the channels after
    /// the control pre-pass, group activity, and the control kinds resolved
    /// so far (bits 1/2/4: angle/position/scale).
    std::vector<RigExecBone2dSetup> effSetup;
    std::vector<RigExecBone2dChannels> eff;
    std::vector<uint8_t> active;
    std::vector<uint8_t> ctrlDone;
    std::vector<int32_t> groupSlot;
};

/// The cross scale of \p scaling for along-bone scale \p sx: normal 1,
/// uniform sx, squashStretch |sx|^(-squashFactor) (1 when |sx| is below
/// kRigExecBone2dMinScale, where the volume rule has no finite answer).
inline double RigExecBone2dCrossScale(RigExecBone2dScaling scaling,
    double sx, double squashFactor)
{
    switch (scaling) {
    case RigExecBone2dScaling::Uniform:
        return sx;
    case RigExecBone2dScaling::SquashStretch: {
        const double magnitude = std::fabs(sx);
        if (!(magnitude >= kRigExecBone2dMinScale)) {
            return 1.0;
        }
        return std::pow(magnitude, -squashFactor);
    }
    case RigExecBone2dScaling::Normal:
        break;
    }
    return 1.0;
}

/// The angle an authoring tool may write: \p value clamped to
/// [restLocal + minAngle, restLocal + maxAngle]. Evaluation never applies
/// it to keyed or driven channels. A tool clamping an avars:rz delta passes
/// restLocal = 0. Reversed limits are taken in order.
inline double RigExecBone2dClampLocalAngle(double restLocal, double minAngle,
    double maxAngle, double value)
{
    const double lo = restLocal + std::min(minAngle, maxAngle);
    const double hi = restLocal + std::max(minAngle, maxAngle);
    return std::min(std::max(value, lo), hi);
}

/// Tolerance of the planar rest check.
inline constexpr double kRigExecBone2dPlanarTolerance = 1e-9;

/// A setup frame given as four landmark points (origin, origin + X,
/// origin + Y, origin + Z; rig-common space) as a planar rest. False when
/// the frame is not a rotation about +Z plus a translation: X and Y unit,
/// orthogonal and in the XY plane, Z = +Z (each within
/// kRigExecBone2dPlanarTolerance). The origin's z is not part of the rest.
inline bool RigExecBone2dRestFromLandmarks(const double origin[3],
    const double xPoint[3], const double yPoint[3], const double zPoint[3],
    RigExecBone2dRest *rest)
{
    const double tol = kRigExecBone2dPlanarTolerance;
    double x[3], y[3], z[3];
    for (int a = 0; a < 3; ++a) {
        x[a] = xPoint[a] - origin[a];
        y[a] = yPoint[a] - origin[a];
        z[a] = zPoint[a] - origin[a];
    }
    for (int a = 0; a < 3; ++a) {
        if (!std::isfinite(origin[a]) || !std::isfinite(x[a]) ||
            !std::isfinite(y[a]) || !std::isfinite(z[a])) {
            return false;
        }
    }
    const bool planar =
        std::fabs(x[2]) <= tol && std::fabs(y[2]) <= tol &&
        std::fabs(z[0]) <= tol && std::fabs(z[1]) <= tol &&
        std::fabs(z[2] - 1.0) <= tol &&
        std::fabs(x[0] * x[0] + x[1] * x[1] - 1.0) <= tol &&
        std::fabs(y[0] * y[0] + y[1] * y[1] - 1.0) <= tol &&
        std::fabs(x[0] * y[0] + x[1] * y[1]) <= tol &&
        std::fabs(x[0] * y[1] - x[1] * y[0] - 1.0) <= tol;
    if (!planar) {
        return false;
    }
    if (rest) {
        rest->x = origin[0];
        rest->y = origin[1];
        rest->angle = std::atan2(x[1], x[0]) * (180.0 / 3.14159265358979323846);
    }
    return true;
}

/// An outside parent space's placement in the plane from its four landmarks:
/// as RigExecBone2dRestFromLandmarks, but a scale on the axes is not part of
/// the placement (the axes are normalized before the checks). False for a
/// frame that is degenerate, mirrored or not a rotation about +Z.
inline bool RigExecBone2dPlacementFromLandmarks(const double origin[3],
    const double xPoint[3], const double yPoint[3], const double zPoint[3],
    RigExecBone2dRest *placement)
{
    double x[3], y[3], z[3];
    for (int a = 0; a < 3; ++a) {
        x[a] = xPoint[a] - origin[a];
        y[a] = yPoint[a] - origin[a];
        z[a] = zPoint[a] - origin[a];
    }
    const double lx = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    const double ly = std::sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
    const double lz = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
    if (!(lx > 0) || !(ly > 0) || !(lz > 0) || !std::isfinite(lx) ||
        !std::isfinite(ly) || !std::isfinite(lz) || !std::isfinite(origin[0]) ||
        !std::isfinite(origin[1]) || !std::isfinite(origin[2])) {
        return false;
    }
    double ox[3], oy[3], oz[3];
    for (int a = 0; a < 3; ++a) {
        ox[a] = x[a] / lx;
        oy[a] = y[a] / ly;
        oz[a] = z[a] / lz;
    }
    double stand[3];
    for (int a = 0; a < 3; ++a) stand[a] = origin[a] + ox[a];
    double side[3];
    for (int a = 0; a < 3; ++a) side[a] = origin[a] + oy[a];
    double up[3];
    for (int a = 0; a < 3; ++a) up[a] = origin[a] + oz[a];
    return RigExecBone2dRestFromLandmarks(origin, stand, side, up, placement);
}

namespace skeleton2d_detail {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kRadToDeg = 180.0 / kPi;

inline bool Finite(double v) { return std::isfinite(v); }

inline RigExecBone2dFrame RestFrame(const RigExecBone2dRest &rest)
{
    RigExecBone2dFrame f;
    const double c = std::cos(rest.angle * kDegToRad);
    const double s = std::sin(rest.angle * kDegToRad);
    f.originX = rest.x; f.originY = rest.y;
    f.xAxisX = c; f.xAxisY = s;
    f.yAxisX = -s; f.yAxisY = c;
    return f;
}

/// Wraps degrees into (-180, 180].
inline double Wrap180(double degrees)
{
    double w = std::fmod(degrees, 360.0);
    if (w <= -180.0) w += 360.0;
    else if (w > 180.0) w -= 360.0;
    return w;
}

/// +1 when \p f keeps orientation, -1 when it mirrors.
inline double Parity(const RigExecBone2dFrame &f)
{
    return (f.xAxisX * f.yAxisY - f.xAxisY * f.yAxisX) < 0 ? -1.0 : 1.0;
}

inline double WorldAngle(const RigExecBone2dFrame &f)
{
    return std::atan2(f.xAxisY, f.xAxisX) * kRadToDeg;
}

/// An affine frame of four landmarks: a point is o + c * m (row c, the rows
/// of m the frame's axes); inverse is m^-1, so c = (p - o) * inverse.
struct Affine {
    double o[3] = {0, 0, 0};
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double inverse[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

/// The frame of landmarks \p lm (origin, +X, +Y, +Z tips). False when the
/// axes are degenerate or not finite.
template <class Landmarks>
inline bool AffineOf(const Landmarks &lm, Affine *f)
{
    for (int a = 0; a < 3; ++a) {
        f->o[a] = lm[0][a];
        for (int k = 0; k < 3; ++k) f->m[k][a] = lm[size_t(k) + 1][a] - lm[0][a];
    }
    const double (&m)[3][3] = f->m;
    const double c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const double c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const double c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const double det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if (!(det != 0) || !std::isfinite(det)) {
        return false;
    }
    const double inv = 1.0 / det;
    f->inverse[0][0] = c00 * inv;
    f->inverse[1][0] = c01 * inv;
    f->inverse[2][0] = c02 * inv;
    f->inverse[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
    f->inverse[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
    f->inverse[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
    f->inverse[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
    f->inverse[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
    f->inverse[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
    for (int k = 0; k < 3; ++k) {
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(f->inverse[k][a])) return false;
        }
    }
    return true;
}

/// Point \p p in frame \p f's coordinates: (p - o) * m^-1.
inline void ToFrame(const Affine &f, const double p[3], double c[3])
{
    const double d[3] = {p[0] - f.o[0], p[1] - f.o[1], p[2] - f.o[2]};
    for (int a = 0; a < 3; ++a) {
        c[a] = d[0] * f.inverse[0][a] + d[1] * f.inverse[1][a] + d[2] * f.inverse[2][a];
    }
}

/// Frame \p f's point at coordinates \p c: o + c * m.
inline void FromFrame(const Affine &f, const double c[3], double p[3])
{
    for (int a = 0; a < 3; ++a) {
        p[a] = f.o[a] + c[0] * f.m[0][a] + c[1] * f.m[1][a] + c[2] * f.m[2][a];
    }
}

/// Landmarks \p lm expressed in frame \p f (each of the four points), or
/// copied when \p f is null.
template <class Landmarks>
inline void LandmarksIn(const Affine *f, const Landmarks &lm, double out[4][3])
{
    for (size_t i = 0; i < 4; ++i) {
        const double p[3] = {double(lm[i][0]), double(lm[i][1]), double(lm[i][2])};
        if (f) {
            ToFrame(*f, p, out[i]);
        } else {
            for (int a = 0; a < 3; ++a) out[i][a] = p[a];
        }
    }
}

inline RigExecBone2dFrame IdentityFrame() { return RigExecBone2dFrame(); }

/// Whether bone \p i blends a second parent space this frame.
inline bool Blends(const RigExecSkeleton2dWorkspace &ws, size_t i)
{
    return ws.spaceBlend[i] > 0;
}

/// The posed frame of parent space \p source, null for the base.
inline const RigExecBone2dFrame *SpaceFrame(const RigExecSkeleton2dWorkspace &ws,
    int32_t source)
{
    if (source >= 0) return &ws.pose[size_t(source)];
    const int64_t e = RigExecBone2dSpaceExternalIndex(source);
    return e >= 0 ? &ws.external[size_t(e)] : nullptr;
}

/// Bone \p i's frame under parent space \p source (candidate \p c):
/// rz is ws.localAngle[i], the solved local position ws.localX/Y[2i + c].
inline RigExecBone2dFrame ComposeUnder(const RigExecBone2dSetup &b,
    const RigExecBone2dChannels &ch, size_t i, size_t c, int32_t source,
    const RigExecSkeleton2dWorkspace &ws)
{
    const size_t at = 2 * i + c;
    const double angle = (ws.baseAngle[at] + ws.localAngle[i]) * kDegToRad;
    const double cs = std::cos(angle), sn = std::sin(angle);
    const double fx = ch.endFlip ? -1.0 : 1.0;
    const double fy = ch.sideFlip ? -1.0 : 1.0;
    // The bone's axes in the frame they turn in: its parent space's pose
    // frame, or the plane for the base or an independent angle.
    const double lxX = fx * cs, lxY = fx * sn;
    const double lyX = -fy * sn, lyY = fy * cs;
    RigExecBone2dFrame out;
    const RigExecBone2dFrame *parent = SpaceFrame(ws, source);
    if (!parent) {
        out.originX = ws.localX[at];
        out.originY = ws.localY[at];
        out.xAxisX = lxX; out.xAxisY = lxY;
        out.yAxisX = lyX; out.yAxisY = lyY;
        return out;
    }
    const RigExecBone2dFrame &p = *parent;
    out.originX = p.originX + ws.localX[at] * p.xAxisX + ws.localY[at] * p.yAxisX;
    out.originY = p.originY + ws.localX[at] * p.xAxisY + ws.localY[at] * p.yAxisY;
    if (b.flags & kRigExecBone2dIndependentAngle) {
        // Ancestors' turns and flips leave the orientation unchanged.
        out.xAxisX = lxX; out.xAxisY = lxY;
        out.yAxisX = lyX; out.yAxisY = lyY;
        return out;
    }
    // Flips compose down the hierarchy (parent parity).
    out.xAxisX = lxX * p.xAxisX + lxY * p.yAxisX;
    out.xAxisY = lxX * p.xAxisY + lxY * p.yAxisY;
    out.yAxisX = lyX * p.xAxisX + lyY * p.yAxisX;
    out.yAxisY = lyX * p.xAxisY + lyY * p.yAxisY;
    return out;
}

/// The blend of two pose frames by weight \p w in (0, 1): the origin
/// linearly, the X axis along the shortest arc, the handedness of the
/// nearer frame (the space switch's blend, which lerps translation and
/// slerps rotation, restricted to the plane).
inline RigExecBone2dFrame BlendFrames(const RigExecBone2dFrame &a,
    const RigExecBone2dFrame &b, double w)
{
    RigExecBone2dFrame out;
    out.originX = (1 - w) * a.originX + w * b.originX;
    out.originY = (1 - w) * a.originY + w * b.originY;
    const double ta = std::atan2(a.xAxisY, a.xAxisX);
    double turn = std::atan2(b.xAxisY, b.xAxisX) - ta;
    if (turn > kPi) turn -= 2 * kPi;
    else if (turn <= -kPi) turn += 2 * kPi;
    const double t = ta + w * turn;
    const double cs = std::cos(t), sn = std::sin(t);
    const double parity = Parity(w < 0.5 ? a : b);
    out.xAxisX = cs; out.xAxisY = sn;
    out.yAxisX = -parity * sn; out.yAxisY = parity * cs;
    return out;
}

/// Composes bone \p i's pose frame from its parent spaces' (already
/// composed).
inline void Compose(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t i,
    RigExecSkeleton2dWorkspace &ws)
{
    const RigExecBone2dFrame under =
        ComposeUnder(setup[i], channels[i], i, 0, ws.spaceA[i], ws);
    if (!Blends(ws, i)) {
        ws.pose[i] = under;
        return;
    }
    ws.pose[i] = BlendFrames(under,
        ComposeUnder(setup[i], channels[i], i, 1, ws.spaceB[i], ws), ws.spaceBlend[i]);
}

/// The bone's local positions from its rest-local positions and channels:
/// a parent bone's along-bone scale moves the attach point along the
/// parent, and the offset translates the posed bone in the parent's frame.
inline void LocalPosition(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t i,
    RigExecSkeleton2dWorkspace &ws)
{
    (void)setup;
    const size_t spaces = Blends(ws, i) ? 2 : 1;
    for (size_t c = 0; c < spaces; ++c) {
        const int32_t p = c == 0 ? ws.spaceA[i] : ws.spaceB[i];
        // The parent's scale multiplies the attach point's along-bone coordinate only.
        const double parentScale = p < 0 ? 1.0 : ws.scale[size_t(p)];
        // The offset is an extra posed translation only.
        const size_t at = 2 * i + c;
        ws.localX[at] = ws.restLocalX[at] * parentScale + channels[i].tx + channels[i].offsetX;
        ws.localY[at] = ws.restLocalY[at] + channels[i].ty + channels[i].offsetY;
    }
}

/// Whether bone \p i composes from a bone marked dirty.
inline bool ReadsDirty(const RigExecSkeleton2dWorkspace &ws, size_t i)
{
    const int32_t a = ws.spaceA[i];
    if (a >= 0 && ws.dirty[size_t(a)]) return true;
    const int32_t b = ws.spaceB[i];
    return Blends(ws, i) && b >= 0 && ws.dirty[size_t(b)];
}

/// Recomposes every bone composed from \p root (solve order is parents first).
inline void RecomposeBelow(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t count, size_t root,
    RigExecSkeleton2dWorkspace &ws)
{
    std::fill(ws.dirty.begin(), ws.dirty.end(), uint8_t(0));
    ws.dirty[root] = 1;
    Compose(setup, channels, root, ws);
    for (size_t i = root + 1; i < count; ++i) {
        if (ReadsDirty(ws, i)) {
            ws.dirty[i] = 1;
            LocalPosition(setup, channels, i, ws);
            Compose(setup, channels, i, ws);
        }
    }
}

/// Clamps bone \p i's solved turn to its limits when they are on.
inline void ClampTurn(const RigExecBone2dSetup &b, double *turn)
{
    if (b.flags & kRigExecBone2dAngleLimits) {
        *turn = RigExecBone2dClampLocalAngle(0.0, b.minAngle, b.maxAngle, *turn);
    }
}

/// The world-space sign a turn of bone \p i takes: an independent bone or
/// a root turns in asset space; any other bone turns in its parent's frame.
inline double TurnSign(const RigExecBone2dSetup *setup, size_t i,
    const RigExecSkeleton2dWorkspace &ws)
{
    const int32_t p = setup[i].parent;
    if (p < 0 || (setup[i].flags & kRigExecBone2dIndependentAngle)) {
        return 1.0;
    }
    return Parity(ws.pose[size_t(p)]);
}

/// Turns bone \p j so the ray from its base to the effector point turns by
/// the world angle \p delta (degrees), within its limits; recomposes below.
inline void Turn(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t count, size_t j,
    double delta, RigExecSkeleton2dWorkspace &ws)
{
    double turn = ws.localAngle[j] + delta * TurnSign(setup, j, ws);
    ClampTurn(setup[j], &turn);
    if (turn == ws.localAngle[j]) {
        return;
    }
    ws.localAngle[j] = turn;
    RecomposeBelow(setup, channels, count, j, ws);
}

/// Turns chain bone \p ws.chain[c] by the world angle \p delta (degrees),
/// within its limits, and recomposes the chain below it down to \p k only:
/// the rest of the subtree is recomposed once the lock is solved.
inline bool TurnOnChain(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t c, size_t k, double delta,
    RigExecSkeleton2dWorkspace &ws)
{
    const size_t j = size_t(ws.chain[c]);
    double turn = ws.localAngle[j] + delta * TurnSign(setup, j, ws);
    ClampTurn(setup[j], &turn);
    if (turn == ws.localAngle[j]) {
        return false;
    }
    ws.localAngle[j] = turn;
    Compose(setup, channels, j, ws);
    for (size_t below = c; below-- > 0;) {
        Compose(setup, channels, size_t(ws.chain[below]), ws);
    }
    Compose(setup, channels, k, ws);
    return true;
}

/// The lock chain of bone \p k: its ancestors nearest first, ending before
/// the first stop -- a root, an independent-angle bone, an inactive bone, a
/// bone with more than one child not ignored by IK -- and at most
/// kRigExecSkeleton2dMaxChain.
// A stop ends the chain and is not turned; roots keep their keyed placement.
inline void LockChain(const RigExecBone2dSetup *setup, size_t k,
    RigExecSkeleton2dWorkspace &ws)
{
    ws.chain.clear();
    for (int32_t a = setup[k].parent; a >= 0; a = setup[size_t(a)].parent) {
        const RigExecBone2dSetup &b = setup[size_t(a)];
        if (b.parent < 0 || (b.flags & kRigExecBone2dIndependentAngle) ||
            !ws.active[size_t(a)] || ws.countedChildren[size_t(a)] > 1) {
            break;
        }
        if (ws.chain.size() >= kRigExecSkeleton2dMaxChain) {
            break;
        }
        ws.chain.push_back(a);
    }
}

/// Distance from bone \p k's base to (goalX, goalY).
inline double LockResidual(const RigExecSkeleton2dWorkspace &ws, size_t k,
    double goalX, double goalY)
{
    const double dx = ws.pose[k].originX - goalX;
    const double dy = ws.pose[k].originY - goalY;
    return std::sqrt(dx * dx + dy * dy);
}

/// Recomposes the lock chain top-down, then the locked bone \p k.
inline void ComposeLockChain(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k,
    RigExecSkeleton2dWorkspace &ws)
{
    for (size_t c = ws.chain.size(); c-- > 0;) {
        Compose(setup, channels, size_t(ws.chain[c]), ws);
    }
    Compose(setup, channels, k, ws);
}

/// Cyclic coordinate descent of bone \p k's base toward (goalX, goalY) over
/// ws.chain, from the bone nearest k outward, a fixed number of sweeps; a
/// sweep that turns nothing, or an exact hit, ends it early (method: the
/// "Cyclic coordinate descent" row of docs/references.md).
inline void LockDescent(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k, double goalX,
    double goalY, RigExecSkeleton2dWorkspace &ws)
{
    for (int sweep = 0; sweep < kRigExecSkeleton2dLockSweeps; ++sweep) {
        bool moved = false;
        for (size_t c = 0; c < ws.chain.size(); ++c) {
            const size_t j = size_t(ws.chain[c]);
            const double bx = ws.pose[j].originX, by = ws.pose[j].originY;
            const double ex = ws.pose[k].originX - bx;
            const double ey = ws.pose[k].originY - by;
            const double tx = goalX - bx, ty = goalY - by;
            if ((ex == 0 && ey == 0) || (tx == 0 && ty == 0)) {
                continue;
            }
            const double delta = std::atan2(ex * ty - ey * tx,
                                            ex * tx + ey * ty) * kRadToDeg;
            if (delta == 0) {
                continue;
            }
            moved = TurnOnChain(setup, channels, c, k, delta, ws) || moved;
        }
        if (!moved || (ws.pose[k].originX == goalX &&
                       ws.pose[k].originY == goalY)) {
            break;
        }
    }
}

/// Fits a solved turn (an rz) to bone \p b's limits: \p *turn, then
/// \p *turn - 360, then \p *turn + 360; the first within
/// kRigExecSkeleton2dLimitSlack degrees of [min, max] is clamped onto it.
/// False when none fits. A bone without limits always fits.
inline bool FitLimits(const RigExecBone2dSetup &b, double *turn)
{
    if (!(b.flags & kRigExecBone2dAngleLimits)) {
        return true;
    }
    const double lo = std::min(b.minAngle, b.maxAngle);
    const double hi = std::max(b.minAngle, b.maxAngle);
    const double tries[3] = {*turn, *turn - 360.0, *turn + 360.0};
    for (const double t : tries) {
        if (t >= lo - kRigExecSkeleton2dLimitSlack &&
            t <= hi + kRigExecSkeleton2dLimitSlack) {
            *turn = std::min(std::max(t, lo), hi);
            return true;
        }
    }
    return false;
}

/// The two-joint closed-form retry shared by the lock and target rules,
/// for the chain [mid, top] in ws.chain whose end point (endX, endY) fell
/// short of the goal. \p recompose recomposes the chain from ws.localAngle;
/// \p residual returns the current miss. Law of cosines on l1 = |mid - top|,
/// l2 = |end - mid|, d = |goal - top|, the cosine clamped to [-1, 1]; the
/// current bend side is tried first (a chain straighter than
/// kRigExecSkeleton2dBendSideEpsilon counts as the positive side), then the
/// other. A bend is a candidate only when both turns fit their limits
/// (FitLimits); it replaces the state when its residual is below
/// \p *best - \p tol. Leaves the kept angles composed.
template <class Recompose, class Residual>
inline void RetryTwoLinkClosedForm(const RigExecBone2dSetup *setup, double endX,
    double endY, double goalX, double goalY, double tol, double *best,
    RigExecSkeleton2dWorkspace &ws, Recompose &&recompose, Residual &&residual)
{
    const size_t mid = size_t(ws.chain[0]), top = size_t(ws.chain[1]);
    const double ox = ws.pose[top].originX, oy = ws.pose[top].originY;
    const double ax = ws.pose[mid].originX - ox, ay = ws.pose[mid].originY - oy;
    const double bx = endX - ws.pose[mid].originX;
    const double by = endY - ws.pose[mid].originY;
    const double gx = goalX - ox, gy = goalY - oy;
    const double l1 = std::sqrt(ax * ax + ay * ay);
    const double l2 = std::sqrt(bx * bx + by * by);
    const double d = std::sqrt(gx * gx + gy * gy);
    if (!(l1 > 0) || !(l2 > 0) || !(d > 0)) {
        return;
    }
    // Positive side: link 2 turns counter-clockwise from link 1, which
    // puts link 1 clockwise of the goal direction (phi - alpha).
    const double cross = ax * by - ay * bx;
    const double side = cross < -kRigExecSkeleton2dBendSideEpsilon * l1 * l2 ? -1.0 : 1.0;
    const double cosAlpha = std::min(1.0, std::max(-1.0,
        (l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d)));
    const double alpha = std::acos(cosAlpha) * kRadToDeg;
    const double phi = std::atan2(gy, gx) * kRadToDeg;
    const double theta1 = std::atan2(ay, ax) * kRadToDeg;
    const double theta2 = std::atan2(by, bx) * kRadToDeg;
    // Turning top by a world angle turns link 2 with it; a local turn is the
    // world turn times TurnSign (the parent frame's parity).
    const double signTop = TurnSign(setup, top, ws);
    const double signMid = TurnSign(setup, mid, ws);
    const double fromTop = ws.localAngle[top], fromMid = ws.localAngle[mid];
    double keepTop = fromTop, keepMid = fromMid;
    const double sides[2] = {side, -side};
    for (const double s : sides) {
        const double t1 = phi - s * alpha;
        const double midX = ox + l1 * std::cos(t1 * kDegToRad);
        const double midY = oy + l1 * std::sin(t1 * kDegToRad);
        const double t2 = std::atan2(goalY - midY, goalX - midX) * kRadToDeg;
        const double d1 = Wrap180(t1 - theta1);
        const double d2 = Wrap180(t2 - (theta2 + d1));
        double turnTop = fromTop + d1 * signTop;
        double turnMid = fromMid + d2 * signMid;
        if (!FitLimits(setup[top], &turnTop) || !FitLimits(setup[mid], &turnMid)) {
            continue;
        }
        ws.localAngle[top] = turnTop;
        ws.localAngle[mid] = turnMid;
        recompose();
        const double r = residual();
        if (r < *best - tol) {
            *best = r;
            keepTop = turnTop;
            keepMid = turnMid;
        }
    }
    ws.localAngle[top] = keepTop;
    ws.localAngle[mid] = keepMid;
    recompose();
}

/// Lock retry for a two-joint chain [mid, top] whose descent ended short
/// (it can stop on a limit when only the other bend reaches). From the
/// descent's end state, the two closed-form bends -- law of cosines on
/// l1 = |mid - top|, l2 = |k - mid|, d = |goal - top|, the cosine clamped to
/// [-1, 1] -- are tried in a fixed order: the current bend side first (a
/// chain straighter than kRigExecSkeleton2dBendSideEpsilon counts as the
/// positive side), then the other. A bend is a candidate only when both
/// turns fit their limits (FitLimits); it replaces the state when its
/// residual is below \p *best - \p tol. Leaves the kept angles composed.
inline void RetryLockTwoJoint(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k, double goalX,
    double goalY, double tol, double *best, RigExecSkeleton2dWorkspace &ws)
{
    RetryTwoLinkClosedForm(setup, ws.pose[k].originX, ws.pose[k].originY, goalX, goalY,
        tol, best, ws, [&] { ComposeLockChain(setup, channels, k, ws); },
        [&] { return LockResidual(ws, k, goalX, goalY); });
}

/// Runs the descent from the start the caller set in ws and keeps the result
/// when its residual is below \p *best - \p tol; otherwise restores the
/// chain angles \p kept (ws.chain order) and recomposes the chain.
inline void DescendFromStart(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k, double goalX,
    double goalY, double tol, const double *kept, double *best,
    RigExecSkeleton2dWorkspace &ws)
{
    LockDescent(setup, channels, k, goalX, goalY, ws);
    const double r = LockResidual(ws, k, goalX, goalY);
    if (r < *best - tol) {
        *best = r;
        return;
    }
    for (size_t c = 0; c < ws.chain.size(); ++c) {
        ws.localAngle[size_t(ws.chain[c])] = kept[c];
    }
    ComposeLockChain(setup, channels, k, ws);
}

/// Lock retry for a chain of three or more joints whose descent ended
/// short of a goal within the chain's reach (beyond it no bend reaches, and
/// the descent's straightened chain stays). Two further starts, each running
/// the same descent and kept only when its residual is below
/// \p *best - \p tol (DescendFromStart):
///  1. the mirror image: each link reflected across the line from the top
///     bone's base to the goal, the turns applied top-down, each clamped to
///     its limits (the other bend of a bent chain);
///  2. when the result so far still misses and has a joint on a limit
///     bound: those joints restart from their limits' midpoint and the top
///     bone turns the effector toward the goal (the other bend of a chain
///     held straight by a bound, whose mirror image is itself).
inline void RetryLockLongChain(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k, double goalX,
    double goalY, double tol, double *best, RigExecSkeleton2dWorkspace &ws)
{
    const size_t n = ws.chain.size();
    const size_t top = size_t(ws.chain[n - 1]);
    const double ox = ws.pose[top].originX, oy = ws.pose[top].originY;
    const double lx = goalX - ox, ly = goalY - oy;
    const double length = std::sqrt(lx * lx + ly * ly);
    double kept[kRigExecSkeleton2dMaxChain];
    double mirrored[kRigExecSkeleton2dMaxChain];
    double links = 0;
    for (size_t c = 0; c < n; ++c) {
        const size_t j = size_t(ws.chain[c]);
        const size_t next = c == 0 ? k : size_t(ws.chain[c - 1]);
        const double vx = ws.pose[next].originX - ws.pose[j].originX;
        const double vy = ws.pose[next].originY - ws.pose[j].originY;
        links += std::sqrt(vx * vx + vy * vy);
    }
    if (!(length > 0) || length > links) {
        return;
    }
    const double ux = lx / length, uy = ly / length;
    for (size_t c = 0; c < n; ++c) {
        const size_t j = size_t(ws.chain[c]);
        const size_t next = c == 0 ? k : size_t(ws.chain[c - 1]);
        const double vx = ws.pose[next].originX - ws.pose[j].originX;
        const double vy = ws.pose[next].originY - ws.pose[j].originY;
        const double along = vx * ux + vy * uy;
        kept[c] = ws.localAngle[j];
        mirrored[c] = std::atan2(2.0 * along * uy - vy, 2.0 * along * ux - vx) * kRadToDeg;
    }
    for (size_t c = n; c-- > 0;) {
        const size_t j = size_t(ws.chain[c]);
        const size_t next = c == 0 ? k : size_t(ws.chain[c - 1]);
        const double vx = ws.pose[next].originX - ws.pose[j].originX;
        const double vy = ws.pose[next].originY - ws.pose[j].originY;
        if (vx == 0 && vy == 0) {
            continue;
        }
        const double delta = Wrap180(mirrored[c] - std::atan2(vy, vx) * kRadToDeg);
        TurnOnChain(setup, channels, c, k, delta, ws);
    }
    DescendFromStart(setup, channels, k, goalX, goalY, tol, kept, best, ws);
    if (*best <= tol) {
        return;
    }
    bool released = false;
    for (size_t c = 0; c < n; ++c) {
        const size_t j = size_t(ws.chain[c]);
        kept[c] = ws.localAngle[j];
        const RigExecBone2dSetup &b = setup[j];
        if (!(b.flags & kRigExecBone2dAngleLimits)) {
            continue;
        }
        const double lo = std::min(b.minAngle, b.maxAngle);
        const double hi = std::max(b.minAngle, b.maxAngle);
        if (hi > lo && (kept[c] <= lo + kRigExecSkeleton2dLimitSlack ||
                        kept[c] >= hi - kRigExecSkeleton2dLimitSlack)) {
            ws.localAngle[j] = 0.5 * (lo + hi);
            released = true;
        }
    }
    if (!released) {
        return;
    }
    ComposeLockChain(setup, channels, k, ws);
    // The folded chain aimed at the goal: the lower joints then unfold
    // toward it on their legal side instead of straightening onto the bound.
    const double ex = ws.pose[k].originX - ox, ey = ws.pose[k].originY - oy;
    if (!(ex == 0 && ey == 0)) {
        const double delta = std::atan2(ex * ly - ey * lx, ex * lx + ey * ly) * kRadToDeg;
        if (delta != 0) {
            TurnOnChain(setup, channels, n - 1, k, delta, ws);
        }
    }
    DescendFromStart(setup, channels, k, goalX, goalY, tol, kept, best, ws);
}

/// Rounds \p v to an index into [0, \p count), or -1 when it names
/// nothing (not finite, negative, or past the end). Half rounds away from
/// zero. Never passes a huge value to llround.
inline int64_t SelectIndex(double v, size_t count)
{
    if (count == 0 || !(v >= -0.5) || !(v < double(count) - 0.5)) {
        return -1;
    }
    return std::llround(v);
}

enum : uint8_t {
    kCtrlAngle = 1u << 0,
    kCtrlPos = 1u << 1,
    kCtrlScale = 1u << 2,
};

/// Revises ws.eff's tx/ty/rz/sx from each bone's controllers (delta form, so
/// the setup frame stays unchanged): rz += kA * rz_K, tx += kX * tx_K,
/// ty += kY * ty_K, sx *= 1 + kS * (sx_K - 1). Each kind resolves once its
/// controller's own channels are resolved, so chained controls compose; a
/// kind caught in a control cycle resolves in bone order against the
/// controller's current values. Controllers read keyed (pre-solve) values.
/// A zero factor, or no controller, applies nothing bit for bit.
// Delta form, in the parent's frame.
inline void ResolveControls(const RigExecBone2dSetup *setup, size_t count,
    RigExecSkeleton2dWorkspace &ws)
{
    for (size_t i = 0; i < count; ++i) {
        const RigExecBone2dSetup &b = setup[i];
        ws.ctrlDone[i] = uint8_t((b.angleController < 0 ? kCtrlAngle : 0) |
                                 (b.posController < 0 ? kCtrlPos : 0) |
                                 (b.scaleController < 0 ? kCtrlScale : 0));
    }
    for (;;) {
        bool progress = false, all = true;
        for (size_t i = 0; i < count; ++i) {
            const uint8_t done = ws.ctrlDone[i];
            if (done == (kCtrlAngle | kCtrlPos | kCtrlScale)) {
                continue;
            }
            all = false;
            const RigExecBone2dSetup &b = setup[i];
            RigExecBone2dChannels &ch = ws.eff[i];
            if (!(done & kCtrlAngle) && b.angleController >= 0 &&
                ws.ctrlDone[size_t(b.angleController)] ==
                    (kCtrlAngle | kCtrlPos | kCtrlScale)) {
                if (b.angleFactor != 0) {
                    ch.rz += b.angleFactor * ws.eff[size_t(b.angleController)].rz;
                }
                ws.ctrlDone[i] = uint8_t(done | kCtrlAngle);
                progress = true;
            }
            const uint8_t afterAngle = ws.ctrlDone[i];
            if (!(afterAngle & kCtrlPos) && b.posController >= 0 &&
                ws.ctrlDone[size_t(b.posController)] ==
                    (kCtrlAngle | kCtrlPos | kCtrlScale)) {
                if (b.posFactorX != 0) {
                    ch.tx += b.posFactorX * ws.eff[size_t(b.posController)].tx;
                }
                if (b.posFactorY != 0) {
                    ch.ty += b.posFactorY * ws.eff[size_t(b.posController)].ty;
                }
                ws.ctrlDone[i] = uint8_t(afterAngle | kCtrlPos);
                progress = true;
            }
            const uint8_t afterPos = ws.ctrlDone[i];
            if (!(afterPos & kCtrlScale) && b.scaleController >= 0 &&
                ws.ctrlDone[size_t(b.scaleController)] ==
                    (kCtrlAngle | kCtrlPos | kCtrlScale)) {
                if (b.scaleFactor != 0) {
                    ch.sx *= 1.0 + b.scaleFactor *
                                        (ws.eff[size_t(b.scaleController)].sx - 1.0);
                }
                ws.ctrlDone[i] = uint8_t(afterPos | kCtrlScale);
                progress = true;
            }
        }
        if (all) {
            return;
        }
        if (progress) {
            continue;
        }
        // A control cycle: the lowest bone's pending kinds resolve against
        // the controllers' current values, which unblocks the rest.
        for (size_t i = 0; i < count; ++i) {
            const uint8_t done = ws.ctrlDone[i];
            if (done == (kCtrlAngle | kCtrlPos | kCtrlScale)) {
                continue;
            }
            const RigExecBone2dSetup &b = setup[i];
            RigExecBone2dChannels &ch = ws.eff[i];
            if (!(done & kCtrlAngle) && b.angleController >= 0 &&
                b.angleFactor != 0) {
                ch.rz += b.angleFactor * ws.eff[size_t(b.angleController)].rz;
            }
            if (!(done & kCtrlPos) && b.posController >= 0) {
                if (b.posFactorX != 0) {
                    ch.tx += b.posFactorX * ws.eff[size_t(b.posController)].tx;
                }
                if (b.posFactorY != 0) {
                    ch.ty += b.posFactorY * ws.eff[size_t(b.posController)].ty;
                }
            }
            if (!(done & kCtrlScale) && b.scaleController >= 0 &&
                b.scaleFactor != 0) {
                ch.sx *= 1.0 + b.scaleFactor *
                                    (ws.eff[size_t(b.scaleController)].sx - 1.0);
            }
            ws.ctrlDone[i] = kCtrlAngle | kCtrlPos | kCtrlScale;
            break;
        }
    }
}

/// Resolves each bone's parent spaces this frame into ws.spaceA/spaceB and
/// ws.spaceBlend, and copies \p setup to ws.effSetup with each bone's chain
/// parent: a bone without a switch keeps its namespace parent; a switch's
/// active index, clamped to its sources (a non-finite one reads 0), selects
/// the space below it and blends the next one in by its fraction. The chain
/// parent is the nearer space when it is a bone, otherwise none. False when
/// a bone source does not precede its bone (the static order covers every
/// source, so the compile rules that out).
inline bool ResolveSpaces(const RigExecBone2dSetup *setup, size_t count,
    const RigExecSkeleton2dStatic *extra, const RigExecSkeleton2dFrameInputs &inputs,
    RigExecSkeleton2dWorkspace &ws)
{
    ws.effSetup.assign(setup, setup + count);
    ws.spaceA.resize(count);
    ws.spaceB.resize(count);
    ws.spaceBlend.assign(count, 0.0);
    for (size_t i = 0; i < count; ++i) {
        const RigExecBone2dSetup &b = setup[i];
        int32_t lower = b.parent < 0 ? kRigExecBone2dSpaceBase : b.parent;
        int32_t upper = lower;
        double blend = 0;
        if (extra && b.spaceCount > 0) {
            const double value = inputs.selectors[size_t(b.spaceSelector)];
            double active = std::isfinite(value) ? value : 0.0;
            active = std::max(0.0, std::min(active, double(b.spaceCount - 1)));
            const size_t below = size_t(std::floor(active));
            const size_t above = std::min(below + 1, size_t(b.spaceCount) - 1);
            lower = extra->spaceSources[b.spaceBegin + below];
            upper = extra->spaceSources[b.spaceBegin + above];
            blend = above == below ? 0.0 : active - double(below);
            if (!(blend > 0)) upper = lower;
        }
        for (const int32_t source : {lower, upper}) {
            if (source >= int32_t(i)) {
                return false;
            }
        }
        ws.spaceA[i] = lower;
        ws.spaceB[i] = upper;
        ws.spaceBlend[i] = blend;
        const int32_t nearer = blend < 0.5 ? lower : upper;
        ws.effSetup[i].parent = nearer >= 0 ? nearer : -1;
    }
    return true;
}

/// Marks ws.active: a bone under an inactive member of an enabled group is
/// inactive, unless it is also under an active member (active wins); every
/// other bone is active. An out-of-range active index clamps to the nearest
/// member. Inactive bones publish their rest frames and neither solve nor
/// join chains.
// An active member wins where chains overlap; the active index clamps to the nearest member.
inline void ResolveGroups(const RigExecBone2dSetup *effSetup, size_t count,
    const RigExecSkeleton2dStatic *extra, const double *groupActive,
    RigExecSkeleton2dWorkspace &ws)
{
    ws.active.assign(count, 1);
    if (!extra || extra->groupCount == 0) {
        return;
    }
    ws.groupSlot.assign(extra->groupCount, -1);
    for (size_t g = 0; g < extra->groupCount; ++g) {
        const RigExecBone2dGroup &group = extra->groups[g];
        if (!group.enabled || group.memberCount == 0) {
            continue;
        }
        const double v = groupActive[g];
        int64_t slot = SelectIndex(v, group.memberCount);
        if (slot < 0) {
            slot = (!(v >= 0.0)) ? 0 : int64_t(group.memberCount) - 1;
        }
        ws.groupSlot[g] = int32_t(slot);
    }
    for (size_t i = 0; i < count; ++i) {
        bool underActive = false, underInactive = false;
        for (int32_t a = int32_t(i);
             a >= 0 && !(underActive && underInactive);
             a = effSetup[size_t(a)].parent) {
            for (size_t g = 0; g < extra->groupCount; ++g) {
                const int32_t slot = ws.groupSlot[g];
                if (slot < 0) {
                    continue;
                }
                const RigExecBone2dGroup &group = extra->groups[g];
                for (uint32_t m = 0; m < group.memberCount; ++m) {
                    if (extra->groupMembers[group.memberBegin + m] == a) {
                        if (int32_t(m) == slot) {
                            underActive = true;
                        } else {
                            underInactive = true;
                        }
                    }
                }
            }
        }
        ws.active[i] = (underActive || !underInactive) ? uint8_t(1) : uint8_t(0);
    }
}

/// The target chain of bone \p k: itself plus its ancestors nearest first,
/// ending before the first stop -- an independent-angle or inactive bone, a
/// locked bone (locks hold against targets), a bone with more than one
/// child not ignored by IK -- and at most kRigExecSkeleton2dMaxChain bones
/// including the owner. A root is included: a lone bone aims.
// The owner aims its tip.
inline void TargetChain(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k,
    RigExecSkeleton2dWorkspace &ws)
{
    ws.chain.clear();
    ws.chain.push_back(int32_t(k));
    for (int32_t a = setup[k].parent; a >= 0; a = setup[size_t(a)].parent) {
        const RigExecBone2dSetup &b = setup[size_t(a)];
        if ((b.flags & kRigExecBone2dIndependentAngle) || !ws.active[size_t(a)] ||
            channels[size_t(a)].lock || ws.countedChildren[size_t(a)] > 1) {
            break;
        }
        if (ws.chain.size() >= kRigExecSkeleton2dMaxChain) {
            break;
        }
        ws.chain.push_back(a);
    }
}

/// The owner's tip: its tip offset in its pose frame, the along-bone part
/// times its along-bone scale (where a child's base lands). A pin's tip is
/// its base.
inline void TipOf(const RigExecSkeleton2dWorkspace &ws, size_t k, double *x, double *y)
{
    const double along = ws.tipX[k] * ws.scale[k];
    const double across = ws.tipY[k];
    *x = ws.pose[k].originX + along * ws.pose[k].xAxisX + across * ws.pose[k].yAxisX;
    *y = ws.pose[k].originY + along * ws.pose[k].xAxisY + across * ws.pose[k].yAxisY;
}

/// Distance from bone \p k's base to its tip (zero for a pin).
inline double TipReach(const RigExecSkeleton2dWorkspace &ws, size_t k)
{
    const double along = ws.tipX[k] * ws.scale[k];
    const double across = ws.tipY[k];
    return std::sqrt(along * along + across * across);
}

/// Distance from bone \p k's tip to (goalX, goalY).
inline double TargetResidual(const RigExecSkeleton2dWorkspace &ws, size_t k,
    double goalX, double goalY)
{
    double tipX = 0, tipY = 0;
    TipOf(ws, k, &tipX, &tipY);
    const double dx = tipX - goalX, dy = tipY - goalY;
    return std::sqrt(dx * dx + dy * dy);
}

/// Turns chain bone \p ws.chain[c] by the world angle \p delta (degrees)
/// and recomposes the chain below it down to the owner.
inline bool TargetTurnOnChain(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t c, double delta,
    RigExecSkeleton2dWorkspace &ws)
{
    const size_t j = size_t(ws.chain[c]);
    double turn = ws.localAngle[j] + delta * TurnSign(setup, j, ws);
    ClampTurn(setup[j], &turn);
    if (turn == ws.localAngle[j]) {
        return false;
    }
    ws.localAngle[j] = turn;
    Compose(setup, channels, j, ws);
    for (size_t below = c; below-- > 0;) {
        Compose(setup, channels, size_t(ws.chain[below]), ws);
    }
    return true;
}

/// Recomposes the target chain top-down.
inline void ComposeTargetChain(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, RigExecSkeleton2dWorkspace &ws)
{
    for (size_t c = ws.chain.size(); c-- > 0;) {
        Compose(setup, channels, size_t(ws.chain[c]), ws);
    }
}

/// Cyclic coordinate descent of bone \p k's tip toward (goalX, goalY) over
/// ws.chain, from the owner outward, a fixed number of sweeps; a sweep that
/// turns nothing, or an exact hit, ends it early.
inline void TargetDescent(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k,
    double goalX, double goalY, RigExecSkeleton2dWorkspace &ws)
{
    for (int sweep = 0; sweep < kRigExecSkeleton2dLockSweeps; ++sweep) {
        bool moved = false;
        for (size_t c = 0; c < ws.chain.size(); ++c) {
            const size_t j = size_t(ws.chain[c]);
            double tipX = 0, tipY = 0;
            TipOf(ws, k, &tipX, &tipY);
            const double bx = ws.pose[j].originX, by = ws.pose[j].originY;
            const double ex = tipX - bx, ey = tipY - by;
            const double tx = goalX - bx, ty = goalY - by;
            if ((ex == 0 && ey == 0) || (tx == 0 && ty == 0)) {
                continue;
            }
            const double delta = std::atan2(ex * ty - ey * tx,
                                            ex * tx + ey * ty) * kRadToDeg;
            if (delta == 0) {
                continue;
            }
            moved = TargetTurnOnChain(setup, channels, c, delta, ws) || moved;
        }
        double tipX = 0, tipY = 0;
        TipOf(ws, k, &tipX, &tipY);
        if (!moved || (tipX == goalX && tipY == goalY)) {
            break;
        }
    }
}

/// Target retry for a two-bone chain [owner, top] whose descent ended short:
/// the two closed-form bends (law of cosines on l1 = |owner - top|,
/// l2 = |tip - owner|, d = |goal - top|), current bend side first, each
/// within its limits, kept only below \p *best - \p tol.
inline void TargetRetryTwoJoint(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t k,
    double goalX, double goalY, double tol, double *best,
    RigExecSkeleton2dWorkspace &ws)
{
    double tipX = 0, tipY = 0;
    TipOf(ws, k, &tipX, &tipY);
    RetryTwoLinkClosedForm(setup, tipX, tipY, goalX, goalY, tol, best, ws,
        [&] { ComposeTargetChain(setup, channels, ws); },
        [&] { return TargetResidual(ws, k, goalX, goalY); });
}

/// Solves bone \p k's IK target: its tip toward its target bone's base.
/// A lone bone aims its tip (with stretch-to-reach; a lone pin has nothing
/// to aim); longer chains descend, and a two-bone chain retries
/// closed-form. A pin owner turns its ancestors so its base meets the
/// target. Locked or inactive owners keep their state, and out-of-reach
/// chains straighten toward the goal.
inline void SolveTarget(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t count, size_t k,
    const RigExecSkeleton2dStatic *extra, RigExecSkeleton2dWorkspace &ws)
{
    const RigExecBone2dChannels &ch = channels[k];
    const RigExecBone2dSetup &b = setup[k];
    if (!ws.active[k] || ch.lock || !extra || b.targetCount == 0) {
        return;
    }
    const int64_t index = SelectIndex(ch.targetIndex, b.targetCount);
    if (index < 0) {
        return;
    }
    const int64_t target = extra->targetCandidates[b.targetBegin + size_t(index)];
    if (target < 0 || target >= int64_t(count) || target == int64_t(k)) {
        return;
    }
    const double goalX = ws.pose[size_t(target)].originX;
    const double goalY = ws.pose[size_t(target)].originY;
    TargetChain(setup, channels, k, ws);
    if (ws.chain.empty()) {
        return;
    }
    const double baseX = ws.pose[k].originX, baseY = ws.pose[k].originY;
    if (ws.chain.size() == 1) {
        const double gx = goalX - baseX, gy = goalY - baseY;
        const double tipReach = TipReach(ws, k);
        if ((gx == 0 && gy == 0) || !(tipReach > 0)) {
            return;
        }
        // maxStretch clamps to >= 1; one common factor.
        if (setup[k].maxStretch > 1.0) {
            const double distance = std::sqrt(gx * gx + gy * gy);
            if (distance > tipReach) {
                ws.scale[k] *= std::min(std::max(1.0, setup[k].maxStretch),
                                        distance / tipReach);
                for (size_t i = k + 1; i < count; ++i) {
                    LocalPosition(setup, channels, i, ws);
                }
            }
        }
        double tipX = 0, tipY = 0;
        TipOf(ws, k, &tipX, &tipY);
        const double delta = Wrap180(std::atan2(gy, gx) * kRadToDeg -
                                     std::atan2(tipY - baseY, tipX - baseX) * kRadToDeg);
        Turn(setup, channels, count, k, delta, ws);
        return;
    }
    const size_t top = size_t(ws.chain.back());
    double reach = 0;
    {
        double fromX = ws.pose[top].originX, fromY = ws.pose[top].originY;
        for (size_t c = ws.chain.size(); c-- > 0;) {
            const size_t next = size_t(ws.chain[c]);
            const double dx = ws.pose[next].originX - fromX;
            const double dy = ws.pose[next].originY - fromY;
            reach += std::sqrt(dx * dx + dy * dy);
            fromX = ws.pose[next].originX;
            fromY = ws.pose[next].originY;
        }
        reach += TipReach(ws, k);
    }
    const double gx = goalX - ws.pose[top].originX;
    const double gy = goalY - ws.pose[top].originY;
    const double distance = std::sqrt(gx * gx + gy * gy);
    // maxStretch clamps to >= 1; one common factor.
    const double maxStretch = std::max(1.0, setup[k].maxStretch);
    if (reach > 0 && distance > reach && maxStretch > 1.0) {
        const double factor = std::min(maxStretch, distance / reach);
        for (const int32_t j : ws.chain) {
            ws.scale[size_t(j)] *= factor;
        }
        for (size_t i = top + 1; i < count; ++i) {
            LocalPosition(setup, channels, i, ws);
        }
        RecomposeBelow(setup, channels, count, top, ws);
    }
    TargetDescent(setup, channels, k, goalX, goalY, ws);
    const double tol = kRigExecSkeleton2dLockReachTolerance * std::max(1.0, reach);
    double best = TargetResidual(ws, k, goalX, goalY);
    if (best > tol && ws.chain.size() == 2 && TipReach(ws, k) > 0) {
        TargetRetryTwoJoint(setup, channels, k, goalX, goalY, tol, &best, ws);
    }
    RecomposeBelow(setup, channels, count, top, ws);
}

/// Solves bone \p k's lock: its base toward (lockX, lockY) by turning the
/// chain, then its world angle to lockAngle.
// Only parented bones lock.
inline void SolveLock(const RigExecBone2dSetup *setup,
    const RigExecBone2dChannels *channels, size_t count, size_t k,
    RigExecSkeleton2dWorkspace &ws)
{
    const RigExecBone2dChannels &ch = channels[k];
    if (setup[k].parent < 0) {
        return;
    }
    LockChain(setup, k, ws);
    const double goalX = ch.lockX, goalY = ch.lockY;
    if (!ws.chain.empty()) {
        // Out of reach: one common along-bone scale, up to the locked bone's
        // maxStretch (clamped >= 1), stretches every chain bone.
        // maxStretch clamps to >= 1; one common factor.
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
            // Children of the stretched bones move their attach points.
            for (size_t i = top + 1; i < count; ++i) {
                LocalPosition(setup, channels, i, ws);
            }
            RecomposeBelow(setup, channels, count, top, ws);
        }
        // Cyclic coordinate descent from the bone nearest k outward, a
        // fixed number of sweeps; an exact hit ends it early.
        LockDescent(setup, channels, k, goalX, goalY, ws);
        // Descent with clamped turns can stop on a limit although a legal
        // bend reaches the lock point. Only a lock left short retries, so a
        // lock the descent reached keeps its result bit for bit.
        const double tol = kRigExecSkeleton2dLockReachTolerance * std::max(1.0, reach);
        double best = LockResidual(ws, k, goalX, goalY);
        if (best > tol) {
            if (ws.chain.size() == 2) {
                RetryLockTwoJoint(setup, channels, k, goalX, goalY, tol, &best, ws);
            } else if (ws.chain.size() > 2) {
                RetryLockLongChain(setup, channels, k, goalX, goalY, tol, &best, ws);
            }
        }
        RecomposeBelow(setup, channels, count, top, ws);
    }
    // The locked bone's own world angle, within its limits.
    const double delta = Wrap180(ch.lockAngle - WorldAngle(ws.pose[k]));
    Turn(setup, channels, count, k, delta, ws);
}

}  // namespace skeleton2d_detail

/// Whether the parent-space, target and group arrays \p extra (or their
/// absence) and the per-evaluation inputs are sane: every range inside its
/// array, every stored bone index a bone (or -1 where a root fits), every
/// outside space and selector present, every controller a bone (or -1),
/// and finite factors, tips and group active indices.
inline bool RigExecBone2dStaticIsValid(const RigExecBone2dSetup *setup,
    size_t count, const RigExecSkeleton2dStatic *extra,
    const RigExecSkeleton2dFrameInputs &inputs)
{
    namespace d = skeleton2d_detail;
    if ((inputs.externalCount > 0 && (!inputs.externalRests || !inputs.externalPoses)) ||
        (inputs.selectorCount > 0 && !inputs.selectors)) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        const RigExecBone2dSetup &b = setup[i];
        if (b.tipBone < -1 || b.tipBone >= int32_t(count) || !d::Finite(b.tipX) ||
            !d::Finite(b.tipY)) {
            return false;
        }
    }
    if (!extra) {
        return true;
    }
    if ((extra->spaceSourceCount > 0 && !extra->spaceSources) ||
        (extra->targetCandidateCount > 0 && !extra->targetCandidates) ||
        (extra->groupCount > 0 && (!extra->groups || inputs.selectorCount < extra->groupCount)) ||
        (extra->groupMemberCount > 0 && !extra->groupMembers)) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        const RigExecBone2dSetup &b = setup[i];
        if (b.spaceBegin > extra->spaceSourceCount ||
            b.spaceCount > extra->spaceSourceCount - b.spaceBegin ||
            b.targetBegin > extra->targetCandidateCount ||
            b.targetCount > extra->targetCandidateCount - b.targetBegin) {
            return false;
        }
        if (b.spaceCount > 0 &&
            (b.spaceSelector < int32_t(extra->groupCount) ||
             size_t(b.spaceSelector) >= inputs.selectorCount)) {
            return false;
        }
        for (uint32_t k = 0; k < b.spaceCount; ++k) {
            const int32_t c = extra->spaceSources[b.spaceBegin + k];
            const int64_t e = RigExecBone2dSpaceExternalIndex(c);
            if (c >= int32_t(count) || (e >= 0 && size_t(e) >= inputs.externalCount)) {
                return false;
            }
        }
        for (uint32_t k = 0; k < b.targetCount; ++k) {
            const int32_t c = extra->targetCandidates[b.targetBegin + k];
            if (c < 0 || c >= int32_t(count)) {
                return false;
            }
        }
        if (b.angleController < -1 || b.angleController >= int32_t(count) ||
            b.posController < -1 || b.posController >= int32_t(count) ||
            b.scaleController < -1 || b.scaleController >= int32_t(count) ||
            !d::Finite(b.angleFactor) || !d::Finite(b.posFactorX) ||
            !d::Finite(b.posFactorY) || !d::Finite(b.scaleFactor)) {
            return false;
        }
    }
    for (size_t g = 0; g < extra->groupCount; ++g) {
        const RigExecBone2dGroup &group = extra->groups[g];
        if (group.memberBegin > extra->groupMemberCount ||
            group.memberCount > extra->groupMemberCount - group.memberBegin ||
            !d::Finite(inputs.selectors[g])) {
            return false;
        }
        for (uint32_t m = 0; m < group.memberCount; ++m) {
            const int32_t bone = extra->groupMembers[group.memberBegin + m];
            if (bone < 0 || bone >= int32_t(count)) {
                return false;
            }
        }
    }
    return true;
}

/// Solves every bone. \p setup and \p rests are in solve order, in the
/// plane; \p pose receives the pose frames (what children compose from),
/// \p deform the published frames. All are sized to \p count. \p extra
/// carries the rigging arrays (null keeps namespace parents, no switches,
/// targets or groups) and \p inputs the outside parent spaces and the
/// selectors. False, with every output reset to its setup frame, for a
/// malformed setup (a parent or source after its child) or a non-finite
/// rest, outside space or consumed channel.
///
/// Controls revise the channels before anything composes; locks solve in
/// solve order, each seeing the corrections of the ones before; targets
/// solve after locks (locks hold against targets). Evaluation never clamps
/// keyed or driven angles; limits bound only the turns the solves give.
inline bool RigExecSolveSkeleton2d(const RigExecBone2dSetup *setup,
    const RigExecBone2dRest *rests, const RigExecBone2dChannels *channels,
    size_t count, RigExecSkeleton2dWorkspace *workspace,
    RigExecBone2dFrame *pose, RigExecBone2dFrame *deform,
    const RigExecSkeleton2dStatic *extra = nullptr,
    const RigExecSkeleton2dFrameInputs &inputs = RigExecSkeleton2dFrameInputs())
{
    namespace d = skeleton2d_detail;
    if (count == 0) {
        return true;
    }
    const auto reset = [&]() {
        for (size_t i = 0; i < count; ++i) {
            if (rests && d::Finite(rests[i].x) && d::Finite(rests[i].y) &&
                d::Finite(rests[i].angle)) {
                const RigExecBone2dFrame f = d::RestFrame(rests[i]);
                if (pose) pose[i] = f;
                if (deform) deform[i] = f;
            } else {
                if (pose) pose[i] = RigExecBone2dFrame();
                if (deform) deform[i] = RigExecBone2dFrame();
            }
        }
        return false;
    };
    if (!setup || !rests || !channels || !workspace || !pose || !deform) {
        return reset();
    }
    if (!RigExecBone2dStaticIsValid(setup, count, extra, inputs)) {
        return reset();
    }
    for (size_t e = 0; e < inputs.externalCount; ++e) {
        const RigExecBone2dRest &r = inputs.externalRests[e];
        const RigExecBone2dRest &q = inputs.externalPoses[e];
        if (!d::Finite(r.x) || !d::Finite(r.y) || !d::Finite(r.angle) ||
            !d::Finite(q.x) || !d::Finite(q.y) || !d::Finite(q.angle)) {
            return reset();
        }
    }
    for (size_t i = 0; i < count; ++i) {
        const RigExecBone2dSetup &b = setup[i];
        const RigExecBone2dChannels &c = channels[i];
        if (b.parent >= int32_t(i) || b.parent < -1) {
            return reset();
        }
        const bool readsTarget = extra && b.targetCount > 0;
        if (!d::Finite(rests[i].x) || !d::Finite(rests[i].y) ||
            !d::Finite(rests[i].angle) || !d::Finite(c.tx) ||
            !d::Finite(c.ty) || !d::Finite(c.rz) || !d::Finite(c.sx) ||
            (c.lock && (!d::Finite(c.lockX) || !d::Finite(c.lockY) ||
                        !d::Finite(c.lockAngle))) ||
            !d::Finite(c.offsetX) || !d::Finite(c.offsetY) ||
            (readsTarget && !d::Finite(c.targetIndex)) ||
            !d::Finite(b.minAngle) || !d::Finite(b.maxAngle) ||
            !d::Finite(b.squashFactor) || !d::Finite(b.maxStretch)) {
            return reset();
        }
    }
    RigExecSkeleton2dWorkspace &ws = *workspace;
    ws.pose.resize(count);
    ws.localAngle.resize(count);
    ws.scale.resize(count);
    ws.baseAngle.resize(2 * count);
    ws.restLocalX.resize(2 * count);
    ws.restLocalY.resize(2 * count);
    ws.localX.resize(2 * count);
    ws.localY.resize(2 * count);
    ws.tipX.resize(count);
    ws.tipY.resize(count);
    ws.countedChildren.assign(count, 0);
    ws.dirty.assign(count, 0);
    ws.eff.assign(channels, channels + count);
    ws.ctrlDone.assign(count, 0);
    ws.chain.reserve(std::min(count, kRigExecSkeleton2dMaxChain));
    ws.external.resize(inputs.externalCount);
    for (size_t e = 0; e < inputs.externalCount; ++e) {
        ws.external[e] = d::RestFrame(inputs.externalPoses[e]);
    }
    // Controls read keyed (pre-solve) channel values.
    d::ResolveControls(setup, count, ws);
    for (size_t i = 0; i < count; ++i) {
        if (!d::Finite(ws.eff[i].tx) || !d::Finite(ws.eff[i].ty) ||
            !d::Finite(ws.eff[i].rz) || !d::Finite(ws.eff[i].sx)) {
            return reset();
        }
    }
    if (!d::ResolveSpaces(setup, count, extra, inputs, ws)) {
        return reset();
    }
    const RigExecBone2dSetup *effSetup = ws.effSetup.data();
    const RigExecBone2dChannels *eff = ws.eff.data();
    d::ResolveGroups(effSetup, count, extra, inputs.selectors, ws);
    for (size_t i = 0; i < count; ++i) {
        const int32_t p = effSetup[i].parent;
        // Pins and bones ignored by IK leave a parent's lock and target
        // chains open.
        if (p >= 0 && !(effSetup[i].flags & kRigExecBone2dIgnoredByIk) &&
            (effSetup[i].flags & kRigExecBone2dTip)) {
            ++ws.countedChildren[size_t(p)];
        }
        // Rest local: L0_i = Rest_i * Rest_p^-1 under each parent space this
        // frame; the base is the plane itself.
        for (size_t c = 0; c < (d::Blends(ws, i) ? 2u : 1u); ++c) {
            const int32_t source = c == 0 ? ws.spaceA[i] : ws.spaceB[i];
            const size_t at = 2 * i + c;
            if (source == kRigExecBone2dSpaceBase) {
                ws.restLocalX[at] = rests[i].x;
                ws.restLocalY[at] = rests[i].y;
                ws.baseAngle[at] = rests[i].angle;
                continue;
            }
            const int64_t e = RigExecBone2dSpaceExternalIndex(source);
            const RigExecBone2dRest &pr =
                e >= 0 ? inputs.externalRests[size_t(e)] : rests[size_t(source)];
            const double a = pr.angle * d::kDegToRad;
            const double cs = std::cos(a), sn = std::sin(a);
            const double dx = rests[i].x - pr.x, dy = rests[i].y - pr.y;
            ws.restLocalX[at] = dx * cs + dy * sn;
            ws.restLocalY[at] = -dx * sn + dy * cs;
            ws.baseAngle[at] = (effSetup[i].flags & kRigExecBone2dIndependentAngle)
                                   ? rests[i].angle
                                   : rests[i].angle - pr.angle;
        }
        // The tip in the bone's setup frame: a child bone's setup base, or
        // the compiled End joint offset; a pin has none.
        ws.tipX[i] = 0;
        ws.tipY[i] = 0;
        if (effSetup[i].flags & kRigExecBone2dTip) {
            if (effSetup[i].tipBone >= 0) {
                const RigExecBone2dRest &tip = rests[size_t(effSetup[i].tipBone)];
                const double a = rests[i].angle * d::kDegToRad;
                const double cs = std::cos(a), sn = std::sin(a);
                const double dx = tip.x - rests[i].x, dy = tip.y - rests[i].y;
                ws.tipX[i] = dx * cs + dy * sn;
                ws.tipY[i] = -dx * sn + dy * cs;
            } else {
                ws.tipX[i] = effSetup[i].tipX;
                ws.tipY[i] = effSetup[i].tipY;
            }
        }
        ws.localAngle[i] = eff[i].rz;
        ws.scale[i] = eff[i].sx;
    }
    for (size_t i = 0; i < count; ++i) {
        d::LocalPosition(effSetup, eff, i, ws);
        d::Compose(effSetup, eff, i, ws);
    }
    // Locks solve after keys and property-mover offsets compose, so a lock
    // holds.
    for (size_t k = 0; k < count; ++k) {
        if (eff[k].lock && ws.active[k]) {
            d::SolveLock(effSetup, eff, count, k, ws);
        }
    }
    // Targets solve after locks, so a target tracks through mover-driven
    // channels.
    for (size_t k = 0; k < count; ++k) {
        d::SolveTarget(effSetup, eff, count, k, extra, ws);
    }
    for (size_t i = 0; i < count; ++i) {
        if (!ws.active[i]) {
            const RigExecBone2dFrame f = d::RestFrame(rests[i]);
            pose[i] = f;
            deform[i] = f;
            continue;
        }
        const RigExecBone2dSetup &b = effSetup[i];
        const RigExecBone2dFrame &p = ws.pose[i];
        pose[i] = p;
        const double along = ws.scale[i];
        const double across = RigExecBone2dCrossScale(b.scaling, along, b.squashFactor);
        RigExecBone2dFrame &out = deform[i];
        out.originX = p.originX; out.originY = p.originY;
        out.xAxisX = p.xAxisX * along; out.xAxisY = p.xAxisY * along;
        out.yAxisX = p.yAxisX * across; out.yAxisY = p.yAxisY * across;
        if (!d::Finite(out.originX) || !d::Finite(out.originY) ||
            !d::Finite(out.xAxisX) || !d::Finite(out.xAxisY) ||
            !d::Finite(out.yAxisX) || !d::Finite(out.yAxisY)) {
            return reset();
        }
    }
    return true;
}

/// Per-solve scratch the calling operation owns (never shared across lanes).
struct RigExecSkeleton2dScratch {
    RigExecSkeleton2dWorkspace solve;
    std::vector<RigExecBone2dRest> rests, externalRests, externalPoses;
    std::vector<double> heights;
    std::vector<RigExecBone2dChannels> channels;
    std::vector<RigExecBone2dFrame> pose, deform;
};

enum class RigExecSkeleton2dResult : uint8_t { Solved, Malformed, NotPlanar, NotFinite };

/// The shared parent-space, target and group arrays as the kernel reads
/// them; the vectors must outlive the result.
inline RigExecSkeleton2dStatic RigExecSkeleton2dStaticOf(
    const std::vector<int32_t> &spaceSources, const std::vector<int32_t> &targetCandidates,
    const std::vector<RigExecBone2dGroup> &groups, const std::vector<int32_t> &groupMembers)
{
    RigExecSkeleton2dStatic extra;
    extra.spaceSources = spaceSources.data();
    extra.spaceSourceCount = spaceSources.size();
    extra.targetCandidates = targetCandidates.data();
    extra.targetCandidateCount = targetCandidates.size();
    extra.groups = groups.data();
    extra.groupCount = groups.size();
    extra.groupMembers = groupMembers.data();
    extra.groupMemberCount = groupMembers.size();
    return extra;
}

/// A bone's planar rest from its four landmarks (origin, +X, +Y, +Z tips).
template <class Landmarks>
inline bool RigExecBone2dRestFromLandmarkSet(const Landmarks &rest, RigExecBone2dRest *out)
{
    double p[4][3];
    for (size_t i = 0; i < 4; ++i) {
        for (size_t a = 0; a < 3; ++a) p[i][a] = rest[i][a];
    }
    return RigExecBone2dRestFromLandmarks(p[0], p[1], p[2], p[3], out);
}

/// The frame a skeleton solves in: the start frame's rest and posed
/// landmarks (rig-common), both null for the rig's asset space. Holds the
/// two affine frames the solve maps through.
class RigExecSkeleton2dPlane {
public:
    RigExecSkeleton2dPlane() = default;
    /// False when either frame is degenerate.
    template <class Landmarks>
    bool Set(const Landmarks *rest, const Landmarks *posed)
    {
        _start = rest && posed;
        if (!_start) return true;
        return skeleton2d_detail::AffineOf(*rest, &_rest) &&
               skeleton2d_detail::AffineOf(*posed, &_posed);
    }
    bool HasStart() const { return _start; }
    /// Rest landmarks \p lm as a planar rest in the start frame's rest
    /// frame, and its origin's height there (\p height may be null).
    template <class Landmarks>
    bool Rest(const Landmarks &lm, RigExecBone2dRest *out, double *height = nullptr) const
    {
        double p[4][3];
        skeleton2d_detail::LandmarksIn(_start ? &_rest : nullptr, lm, p);
        if (height) *height = p[0][2];
        return RigExecBone2dRestFromLandmarks(p[0], p[1], p[2], p[3], out);
    }
    /// An outside space's placement: rest landmarks in the rest frame,
    /// posed landmarks in the posed frame (\p posed true).
    template <class Landmarks>
    bool Placement(const Landmarks &lm, bool posed, RigExecBone2dRest *out) const
    {
        double p[4][3];
        skeleton2d_detail::LandmarksIn(_start ? (posed ? &_posed : &_rest) : nullptr, lm, p);
        return RigExecBone2dPlacementFromLandmarks(p[0], p[1], p[2], p[3], out);
    }
    /// Point \p point (rig-common) in the start frame's rest frame.
    template <class Vec3>
    void PointIn(const Vec3 &point, double out[3]) const
    {
        const double p[3] = {double(point[0]), double(point[1]), double(point[2])};
        if (_start) {
            skeleton2d_detail::ToFrame(_rest, p, out);
        } else {
            for (int a = 0; a < 3; ++a) out[a] = p[a];
        }
    }
    /// The landmarks of plane frame \p frame at height \p z, composed back
    /// through the start frame's posed frame: origin, +X, +Y and +Z (a 2D
    /// frame never tilts out of its plane).
    template <class Vec3>
    std::array<Vec3, 4> Landmarks(const RigExecBone2dFrame &frame, double z) const
    {
        const double plane[4][3] = {
            {frame.originX, frame.originY, z},
            {frame.originX + frame.xAxisX, frame.originY + frame.xAxisY, z},
            {frame.originX + frame.yAxisX, frame.originY + frame.yAxisY, z},
            {frame.originX, frame.originY, z + 1}};
        std::array<Vec3, 4> out;
        for (size_t i = 0; i < 4; ++i) {
            if (!_start) {
                out[i] = Vec3(plane[i][0], plane[i][1], plane[i][2]);
                continue;
            }
            double p[3];
            skeleton2d_detail::FromFrame(_posed, plane[i], p);
            out[i] = Vec3(p[0], p[1], p[2]);
        }
        return out;
    }

private:
    bool _start = false;
    skeleton2d_detail::Affine _rest, _posed;
};

/// One evaluation's landmarks besides the bones' rests (rig-common, as the
/// caller holds them): the start frame's rest and posed landmarks (both null:
/// asset space), each outside parent space's rest and posed landmarks
/// (externalCount of each), and the selectors (RigExecSkeleton2dFrameInputs).
template <class Landmarks>
struct RigExecSkeleton2dLandmarkInputs {
    const Landmarks *startRest = nullptr, *startPose = nullptr;
    const Landmarks *externalRests = nullptr, *externalPoses = nullptr;
    size_t externalCount = 0;
    const double *selectors = nullptr;
    size_t selectorCount = 0;
};

/// The whole skeleton solve over rigExec:joints-ordered rests and frames.
/// \p order is the solve order (rigExec:joints index per entry) and
/// \p setup is in that order. readChannels(j, RigExecBone2dChannels *) fills
/// joint j's channels. Rests and outside spaces are expressed in the start
/// frame's rest frame, solved there, and the published frames composed back
/// through its posed frame. Checks every size and order index before
/// writing (Malformed, frames untouched). On any other failure each frame
/// carries its rest unchanged, so a failed solve publishes the setup frames.
template <class Landmarks, class PointFrame, class ReadChannels>
inline RigExecSkeleton2dResult RigExecSolveSkeleton2dJoints(
    const std::vector<uint32_t> &order, const std::vector<Landmarks> &rests,
    const std::vector<RigExecBone2dSetup> &setup, const RigExecSkeleton2dStatic &extra,
    const RigExecSkeleton2dLandmarkInputs<Landmarks> &frames, ReadChannels &&readChannels,
    uint32_t validFlag, RigExecSkeleton2dScratch *scratch, std::vector<PointFrame> *out)
{
    using Vec3 = std::decay_t<decltype(std::declval<PointFrame &>().points[0])>;
    const size_t n = order.size();
    if (rests.size() != n || setup.size() != n ||
        (frames.externalCount > 0 && (!frames.externalRests || !frames.externalPoses))) {
        return RigExecSkeleton2dResult::Malformed;
    }
    for (size_t e = 0; e < n; ++e) {
        if (size_t(order[e]) >= n) return RigExecSkeleton2dResult::Malformed;
    }
    scratch->rests.resize(n);
    scratch->heights.resize(n);
    scratch->channels.resize(n);
    scratch->pose.resize(n);
    scratch->deform.resize(n);
    scratch->externalRests.resize(frames.externalCount);
    scratch->externalPoses.resize(frames.externalCount);
    RigExecSkeleton2dPlane plane;
    bool planar = plane.Set(frames.startRest, frames.startPose);
    for (size_t e = 0; e < n; ++e) {
        const size_t j = order[e];
        planar = planar && plane.Rest(rests[j], &scratch->rests[e], &scratch->heights[e]);
        scratch->channels[e] = RigExecBone2dChannels();
        readChannels(j, &scratch->channels[e]);
    }
    for (size_t e = 0; e < frames.externalCount; ++e) {
        planar = planar &&
                 plane.Placement(frames.externalRests[e], false, &scratch->externalRests[e]) &&
                 plane.Placement(frames.externalPoses[e], true, &scratch->externalPoses[e]);
    }
    RigExecSkeleton2dFrameInputs inputs;
    inputs.externalRests = scratch->externalRests.data();
    inputs.externalPoses = scratch->externalPoses.data();
    inputs.externalCount = frames.externalCount;
    inputs.selectors = frames.selectors;
    inputs.selectorCount = frames.selectorCount;
    const bool solved = planar && RigExecSolveSkeleton2d(setup.data(), scratch->rests.data(),
        scratch->channels.data(), n, &scratch->solve, scratch->pose.data(),
        scratch->deform.data(), &extra, inputs);
    out->resize(n);
    for (size_t e = 0; e < n; ++e) {
        const size_t j = order[e];
        PointFrame &frame = (*out)[j];
        frame.flags = validFlag;
        if (!solved) {
            frame.points = rests[j];
            continue;
        }
        // The deform frame at the bone's own height: children were composed
        // from the pose frame.
        frame.points = plane.template Landmarks<Vec3>(scratch->deform[e], scratch->heights[e]);
    }
    if (solved) return RigExecSkeleton2dResult::Solved;
    return planar ? RigExecSkeleton2dResult::NotFinite : RigExecSkeleton2dResult::NotPlanar;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SKELETON2D_KERNEL_H
