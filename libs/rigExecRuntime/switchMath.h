// The space switch's matrix primitives on the runtime's Gf mirror types:
// GfTransform's decomposition, transform blending, the twist and swing
// rotation filters, axis masks and the round trip through point landmarks.
// Bit-identical ports of the native program's (solverKernels.cpp,
// rigExecMath/pointFrame.cpp). Header-only and USD-free, so the runtime and
// the op libraries run one definition.
#ifndef RIGEXEC_RUNTIME_SWITCH_MATH_H
#define RIGEXEC_RUNTIME_SWITCH_MATH_H

#include "rigExecRuntime/dualQuatMath.h"
#include "rigExecRuntime/runtimeMath.h"

#include <array>
#include <cmath>

namespace rigExec {

/// The landmarks the identity frame maps from.
inline const std::array<RrVec3d, 4> kRrSwitchIdentityLandmarks = {
    RrVec3d(0.0), RrVec3d(1.0, 0.0, 0.0), RrVec3d(0.0, 1.0, 0.0),
    RrVec3d(0.0, 0.0, 1.0)};

/// RigExecFrameFromMatrix's landmarks of \p m. False at the first
/// non-finite landmark, the later ones left at their identity values.
inline bool
RrSwitchLandmarks(const RrMat4d &m, std::array<RrVec3d, 4> *points)
{
    *points = kRrSwitchIdentityLandmarks;
    for (size_t i = 0; i < 4; ++i) {
        (*points)[i] = m.TransformAffine(kRrSwitchIdentityLandmarks[i]);
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite((*points)[i][a])) {
                return false;
            }
        }
    }
    return true;
}

/// \p m through its point landmarks and back.
inline RrMat4d
RrSwitchRoundTrip(const RrMat4d &m)
{
    RrMat4d out;
    out.SetIdentity();
    std::array<RrVec3d, 4> points;
    RrSwitchLandmarks(m, &points);
    RrDualQuatPointsToMatrix(kRrSwitchIdentityLandmarks, points, &out);
    return out;
}

namespace rrSwitchMath {

// Translation, rotation and scale of a transform, taken the way
// GfTransform takes them so that the decomposition and the recomposition
// below are each other's inverse by construction.
struct Decomposed {
    RrVec3d translation = RrVec3d(0.0);
    RrQuatd rotation = RrQuatd::GetIdentity();
    RrVec3d scale = RrVec3d(1.0);
};

inline Decomposed
Decompose(const RrMat4d &m)
{
    // GfTransform::SetMatrix with a zero pivot; the scale orientation is
    // dropped exactly as Recompose drops it.
    RrMat4d shearRot, rotMat, proj;
    Decomposed out;
    m.Factor(&shearRot, &out.scale, &rotMat, &out.translation, &proj);
    out.rotation = rotMat.ExtractRotation().GetQuat();
    return out;
}

inline RrMat4d
Recompose(const Decomposed &d)
{
    // GfTransform::GetMatrix with no pivot and no scale orientation: each
    // identity step is skipped, which keeps an unscaled, unrotated transform
    // bit-identical to its own translation.
    const RrRotation rotation(d.rotation);
    const bool doScale = d.scale != RrVec3d(1.0, 1.0, 1.0);
    const bool doRotation = rotation.GetAngle() != 0.0;
    const bool doTranslation = d.translation != RrVec3d(0.0);
    bool anySet = false;
    RrMat4d mtx;
    RrMat4d tmp;
    if (doScale) {
        mtx.SetScale(d.scale);
        anySet = true;
    }
    if (doRotation) {
        if (anySet) {
            tmp.SetRotate(rotation);
            mtx = mtx * tmp;
        } else {
            mtx.SetRotate(rotation);
            anySet = true;
        }
    }
    if (doTranslation) {
        if (anySet) {
            tmp.SetTranslate(d.translation);
            mtx = mtx * tmp;
        } else {
            mtx.SetTranslate(d.translation);
            anySet = true;
        }
    }
    if (!anySet) {
        mtx.SetIdentity();
    }
    return mtx;
}

} // namespace rrSwitchMath

inline RrMat4d
RrSwitchBlend(const RrMat4d &a, const RrMat4d &b, double weight)
{
    if (!(weight > 0.0)) {
        return a;
    }
    if (weight >= 1.0) {
        return b;
    }
    using rrSwitchMath::Decomposed;
    const Decomposed da = rrSwitchMath::Decompose(a),
                     db = rrSwitchMath::Decompose(b);
    Decomposed out;
    // GfLerp's own expression, not the algebraically equal a + w * (b - a).
    out.translation = (1 - weight) * da.translation + weight * db.translation;
    out.scale = (1 - weight) * da.scale + weight * db.scale;
    RrQuatd target = db.rotation;
    const double cosArc =
        da.rotation.GetReal() * target.GetReal() +
        RrDot(da.rotation.GetImaginary(), target.GetImaginary());
    if (cosArc < 0.0) {
        target = RrQuatd(-target.GetReal(), -target.GetImaginary());
    }
    out.rotation = RrSlerp(weight, da.rotation, target).GetNormalized();
    return rrSwitchMath::Recompose(out);
}

/// Keeps only the twist of \p m's rotation about \p axis (filter 1) or
/// only the swing (2); 0 returns \p m untouched.
inline RrMat4d
RrSwitchFilter(const RrMat4d &m, const RrVec3d &axis, int filter)
{
    if (filter == 0) {
        return m;
    }
    const double length = axis.GetLength();
    if (length < 1e-12) {
        return m;
    }
    const RrVec3d a = axis / length;
    rrSwitchMath::Decomposed d = rrSwitchMath::Decompose(m);
    const RrQuatd q = d.rotation.GetNormalized();
    const double along = RrDot(q.GetImaginary(), a);
    RrQuatd twist(q.GetReal(), a * along);
    const double norm = std::sqrt(twist.GetReal() * twist.GetReal() +
                                  twist.GetImaginary().GetLengthSq());
    if (norm < 1e-12) {
        twist = RrQuatd::GetIdentity();
    } else {
        twist = RrQuatd(twist.GetReal() / norm, twist.GetImaginary() / norm);
    }
    d.rotation = filter == 1 ? twist : (q * twist.GetInverse()).GetNormalized();
    return rrSwitchMath::Recompose(d);
}

inline RrMat4d
RrSwitchMask(const RrMat4d &m, const bool translation[3],
             const bool rotation[3], const bool scale[3])
{
    const bool all = translation[0] && translation[1] && translation[2] &&
                     rotation[0] && rotation[1] && rotation[2] && scale[0] &&
                     scale[1] && scale[2];
    if (all) {
        return m;
    }
    rrSwitchMath::Decomposed d = rrSwitchMath::Decompose(m);
    for (int axis = 0; axis < 3; ++axis) {
        if (!translation[axis])
            d.translation[axis] = 0.0;
        if (!scale[axis])
            d.scale[axis] = 1.0;
    }
    if (!rotation[0] || !rotation[1] || !rotation[2]) {
        // The same XYZ row-vector extraction the program does, so a mask
        // agrees with the avar compose at every angle.
        const RrMat4d r(RrRotation(d.rotation), RrVec3d(0.0));
        double x = 0.0, y = 0.0, z = 0.0;
        const double sinY = RrClamp(r._mtx[0][2], -1.0, 1.0);
        y = std::asin(sinY);
        if (std::abs(sinY) < 1.0 - 1e-9) {
            x = std::atan2(r._mtx[1][2], r._mtx[2][2]);
            z = std::atan2(r._mtx[0][1], r._mtx[0][0]);
        } else {
            x = std::atan2(-r._mtx[2][1], r._mtx[1][1]);
            z = 0.0;
        }
        const RrVec3d axes[3] = {RrVec3d(1, 0, 0), RrVec3d(0, 1, 0),
                                 RrVec3d(0, 0, 1)};
        const double angles[3] = {RrRadiansToDegrees(x), RrRadiansToDegrees(y),
                                  RrRadiansToDegrees(z)};
        RrMat4d composed(1.0);
        for (int axis = 0; axis < 3; ++axis) {
            if (!rotation[axis] || angles[axis] == 0.0)
                continue;
            composed = composed * RrMat4d(RrRotation(axes[axis], angles[axis]),
                                          RrVec3d(0.0));
        }
        d.rotation = composed.ExtractRotationQuat();
    }
    return rrSwitchMath::Recompose(d);
}

} // namespace rigExec

#endif
