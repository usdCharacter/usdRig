// RigExec point-frame math.
#include "rigExecSchema/schemaNames.h"
#include "pointFrame.h"
#include "dualQuat.h"
#include "switchMathKernel.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

RigExecFramePolicy
RigExecParseFramePolicy(const TfToken &token)
{
    if (token == "affine")
        return RigExecFramePolicy::Affine;
    if (token == "axial")
        return RigExecFramePolicy::Axial;
    if (token == "rigid")
        return RigExecFramePolicy::Rigid;
    return RigExecFramePolicy::Orthogonal;
}

RigExecAxis
RigExecParseAxis(const TfToken &token, RigExecAxis fallback)
{
    if (token == SchemaValueText::x)
        return RigExecAxis::X;
    if (token == SchemaValueText::y)
        return RigExecAxis::Y;
    if (token == SchemaValueText::z)
        return RigExecAxis::Z;
    return fallback;
}

double
RigExecFrameEpsilon(const std::array<GfVec3d, 4> &restPoints)
{
    return RigExecFrameEpsilonT(restPoints);
}

namespace {

GfMatrix3d
_RowBasis(const std::array<GfVec3d, 4> &pts)
{
    return RigExecRowBasisT<GfMatrix3d>(pts);
}

} // namespace

bool
RigExecPointsToMatrix(const std::array<GfVec3d, 4> &restPoints,
                      const std::array<GfVec3d, 4> &posePoints,
                      GfMatrix4d *matrix)
{
    return RigExecPointsToMatrixT<GfMatrix3d>(restPoints, posePoints, matrix);
}

bool
RigExecPointsToMatrix(const std::array<GfVec3d, 4> &restPoints,
                      const RigExecPointFrame &frame, GfMatrix4d *matrix)
{
    return RigExecPointsToMatrix(restPoints, frame.points, matrix);
}

RigExecPointFrame
RigExecMatrixToPoints(const std::array<GfVec3d, 4> &restPoints,
                      const GfMatrix4d &matrix)
{
    RigExecPointFrame frame;
    for (size_t i = 0; i < 4; ++i) {
        frame.points[i] = matrix.TransformAffine(restPoints[i]);
    }
    frame.flags = RigExecPointFrameValid;

    // Classify: reflection and shear/nonuniform-scale.
    const GfMatrix3d posed = _RowBasis(frame.points);
    if (posed.GetDeterminant() < 0) {
        frame.flags |= RigExecPointFrameReflected;
    }
    const double eps = RigExecFrameEpsilon(restPoints);
    if (std::abs(posed.GetDeterminant()) <= eps * eps * eps) {
        frame.flags |= RigExecPointFrameDegenerate;
    }
    return frame;
}

namespace {

GfVec3d
_WorldAxisLeastParallel(const GfVec3d &ex)
{
    const GfVec3d axes[3] = {GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
                             GfVec3d(0, 0, 1)};
    double best = 2.0;
    GfVec3d bestAxis(0, 1, 0);
    for (const GfVec3d &axis : axes) {
        const double a = std::abs(GfDot(axis, ex));
        if (a < best) {
            best = a;
            bestAxis = axis;
        }
    }
    return bestAxis;
}

// Projects candidate up onto the plane orthogonal to ex; returns
// (projected, length).
std::pair<GfVec3d, double>
_ProjectUp(const GfVec3d &up, const GfVec3d &ex)
{
    const GfVec3d u = up - ex * GfDot(ex, up);
    return {u, u.GetLength()};
}

} // namespace

RigExecPointFrame
RigExecReconstructFrame(const std::array<GfVec3d, 4> &restPoints,
                        const std::array<GfVec3d, 4> &posePoints,
                        const RigExecFrameReconstructionArgs &args)
{
    RigExecPointFrame frame;
    frame.points = posePoints;
    frame.flags = RigExecPointFrameValid;

    if (args.policy == RigExecFramePolicy::Affine) {
        // Use all four points directly; retain scale/reflection/shear.
        const GfMatrix3d posed = _RowBasis(posePoints);
        const double det = posed.GetDeterminant();
        const double eps = RigExecFrameEpsilon(restPoints);
        frame.flags |= RigExecPointFrameAffine;
        if (det < 0) {
            frame.flags |= RigExecPointFrameReflected;
        }
        if (std::abs(det) <= eps * eps * eps) {
            frame.flags |= RigExecPointFrameDegenerate;
        }
        return frame;
    }

    // Orthogonal family. The aim/up landmark roles come from
    // the authored aimAxis/upAxis tokens; the remaining landmark carries
    // side scale and handedness.
    const int aimIdx = static_cast<int>(args.aimAxis) + 1;
    const int upIdx = static_cast<int>(args.upAxis) + 1;
    if (upIdx == aimIdx) {
        // aim == up is an invalid authoring configuration: fail explicitly
        // with the raw points rather than silently substituting roles.
        frame.flags |= RigExecPointFrameDegenerate;
        return frame;
    }
    const int sideIdx = 6 - aimIdx - upIdx;
    // Permutation parity of (aim, up, side) relative to (x, y, z): an even
    // permutation completes the frame with side = aim x up; odd negates.
    const bool evenPermutation = (aimIdx == 1 && upIdx == 2) ||
                                 (aimIdx == 2 && upIdx == 3) ||
                                 (aimIdx == 3 && upIdx == 1);
    const double sideSign = evenPermutation ? 1.0 : -1.0;

    const GfVec3d po = posePoints[0];
    const double eps = RigExecFrameEpsilon(restPoints);

    const double lAim =
        std::max((restPoints[aimIdx] - restPoints[0]).GetLength(), eps);
    const double lUp =
        std::max((restPoints[upIdx] - restPoints[0]).GetLength(), eps);
    const double lSide =
        std::max((restPoints[sideIdx] - restPoints[0]).GetLength(), eps);

    const GfVec3d a = posePoints[aimIdx] - po;
    const double aLen = a.GetLength();
    if (aLen < eps) {
        // Aim invalid: orientation cannot be derived. Keep raw points,
        // flag degenerate (no silent identity).
        frame.flags |= RigExecPointFrameDegenerate;
        return frame;
    }
    const GfVec3d eAim = a / aLen;

    // Up projection with the deterministic fallback ladder:
    // authored up landmark, parent computed up, authored rest up,
    // world axis least parallel to the aim direction.
    auto [u, uLen] = _ProjectUp(posePoints[upIdx] - po, eAim);
    bool twistUnderdetermined = false;
    if (uLen < eps) {
        twistUnderdetermined = true;
        if (args.hasParentUp) {
            std::tie(u, uLen) = _ProjectUp(args.parentUp, eAim);
        }
        if (uLen < eps) {
            std::tie(u, uLen) =
                _ProjectUp(restPoints[upIdx] - restPoints[0], eAim);
        }
        if (uLen < eps) {
            std::tie(u, uLen) = _ProjectUp(_WorldAxisLeastParallel(eAim), eAim);
        }
    }
    GfVec3d eUp = u / uLen;
    GfVec3d eSide = sideSign * GfCross(eAim, eUp);
    eUp = sideSign * GfCross(eSide, eAim); // re-cross removes drift
    eUp.Normalize();
    eSide.Normalize();

    // Scale magnitudes and handedness are measured on the UNTWISTED
    // orthogonal basis (dz is defined against e_z, then twist is applied
    // to the reconstructed transverse axes).
    double sAim = aLen / lAim;
    // When the up landmark was degenerate the transverse scale is
    // underdetermined; unit scale is the deterministic choice.
    double sUp = twistUnderdetermined ? 1.0 : uLen / lUp;
    const double dSide = GfDot(posePoints[sideIdx] - po, eSide);
    double sSide = std::abs(dSide) / lSide;
    // Zero side determinant follows the authored reflection policy.
    double sigma = (dSide > 0) ? 1.0 : (dSide < 0 ? -1.0 : args.zeroSideSign);

    switch (args.policy) {
    case RigExecFramePolicy::Rigid:
        // Scale magnitudes only; handedness is separate authored policy.
        sAim = sUp = sSide = 1.0;
        break;
    case RigExecFramePolicy::Axial:
        sUp = sSide = 1.0;
        break;
    case RigExecFramePolicy::Orthogonal:
        if (sSide < eps) {
            // Side landmark in the aim/up plane: no usable side scale.
            sSide = 1.0;
        }
        break;
    default:
        break;
    }

    // Unwrapped twist rotates the transverse axes about the aim axis.
    const double c = std::cos(args.twist);
    const double s = std::sin(args.twist);
    const GfVec3d crossUp = GfCross(eAim, eUp);
    const GfVec3d crossSide = GfCross(eAim, eSide);
    const GfVec3d eUpT = c * eUp + s * crossUp;
    const GfVec3d eSideT = c * eSide + s * crossSide;

    frame.points[0] = po;
    frame.points[aimIdx] = po + lAim * sAim * eAim;
    frame.points[upIdx] = po + lUp * sUp * eUpT;
    frame.points[sideIdx] = po + lSide * sigma * sSide * eSideT;
    if (sigma < 0) {
        frame.flags |= RigExecPointFrameReflected;
    }
    if (twistUnderdetermined) {
        frame.flags |= RigExecPointFrameDegenerate;
    }
    return frame;
}

namespace {

using GfFrameParams = RigExecTransformParamsKernelT<GfVec3f, GfVec3d>;

} // namespace

bool
RigExecPointsToParams(const std::array<GfVec3d, 4> &restPoints,
                      const std::array<GfVec3d, 4> &posePoints,
                      RigExecAxis reflectionAxis,
                      RigExecTransformParams *params)
{
    GfFrameParams p;
    if (!RigExecPointsToParamsKernel<GfVec3f, GfVec3d>(
            restPoints, posePoints, static_cast<int>(reflectionAxis), &p)) {
        return false;
    }
    params->translation = p.translation;
    params->rotation = p.rotation;
    params->scale = p.scale;
    params->shear = p.shear;
    params->reflectionAxis = reflectionAxis;
    return true;
}

GfMatrix4d
RigExecParamsToMatrix(const RigExecTransformParams &params)
{
    GfFrameParams p;
    p.translation = params.translation;
    p.rotation = params.rotation;
    p.scale = params.scale;
    p.shear = params.shear;
    p.reflectionAxis = static_cast<int>(params.reflectionAxis);
    return RigExecParamsToMatrixKernel<GfVec3f, GfVec3d>(p);
}

RigExecPointFrame
RigExecTransformFrame(const RigExecPointFrame &frame, const GfMatrix4d &space)
{
    RigExecPointFrame out = frame;
    for (auto &p : out.points) {
        p = space.Transform(p);
    }
    return out;
}

} // namespace rigExec
