// The pose family's value-step bodies: one ladder, one compose, one solver
// selection, one interpolator, one commit fold, one harmonics and one
// IK/FK body, shared by the baked program and the .rigexec runtime. Both
// adapters keep their own input binding (resolved leaves on the baked
// side, decoded wire records and input slots at playback) and their own
// typed output store; this header owns the decisions between them: the
// rest/ladder fold, solver and constraint selection, interpolation,
// commit propagation order, missing-input behavior, result versions,
// and every contraction-sensitive operation the bodies reach.
//
// Why the arithmetic lives here: the pose path multiplies and inverts
// 4x4 matrices and inverts 3x3 matrices, and USD precompiles those three
// operators into libusd_gf. That build fuses multiply-adds (48 fmadd in
// GfMatrix4d::operator*=, 15 in GetInverse, 8 in GfMatrix3d::GetInverse
// on the ARM64 install this tree builds against) while every project
// translation unit builds with -ffp-contract=off, so the same source
// shape answers different bits on each side. The bodies below therefore
// never call a precompiled Gf operator: pose_detail spells every
// multiply, inverse, determinant, compose, orthonormalization,
// point-frame conversion and round trip from scalar operations in the
// order the runtime's inline mirrors already use. Both instantiations
// compile under the same flags and agree bitwise; on targets without
// fused instructions the spelling matches what both sides computed
// before, so exact captures stay valid.
//
// Host inputs borrow compiled tables only for the scheduled step; no solver
// reads the live stage or starts another traversal.
#ifndef RIGEXEC_EVAL_CORE_POSE_H
#define RIGEXEC_EVAL_CORE_POSE_H

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "rigExecEvalCore/host.h"
#include "rigExecEvalCore/kinds.h"
#include "rigExecEvalCore/step.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExecMath/autoClavicleKernel.h"
#include "rigExecMath/avarScale.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecMath/jointGroupKernel.h"
#include "rigExecMath/poseArithmeticKernel.h"
#include "rigExecMath/poseCommitKernel.h"
#include "rigExecMath/rangeRunner.h"
#include "rigExecMath/skeleton2dKernel.h"
#include "rigExecMath/spaceMatchKernel.h"
#include "rigExecMath/switchMathKernel.h"
#include "rigExecSchema/schemaNames.h"

namespace rigExec {
namespace eval {

// The solver kinds the Solve step selects, as the integer codes both
// adapters already classify to at Build/Open. The bodies never compare
// type text; an unlisted type keeps the adapter's prior answer (the
// aggregate stays empty and the commit carries through).
enum PoseSolverCode : int {
    kPoseSolverUnknown = -1,
    kPoseSolverFkChain = 0,
    kPoseSolverTwoBoneIk = 1,
    kPoseSolverBlendPointFrames = 2,
    kPoseSolverTwistDistribution = 3,
    kPoseSolverRibbon = 4,
    kPoseSolverSplineIk = 5,
    kPoseSolverSkeleton2d = 6
};

// The constraint arms the Constraint step selects, in the order the
// step tests them: a split commit finishes first, then the IK/FK
// presentation, the target admissibility gate, the harmonics window,
// the enabled gate, the weight gate, the single-chain IK arm, and the
// FBX-family type dispatch last.
enum PoseConstraintCode : int {
    kPoseConstraintUnknown = -1,
    kPoseConstraintMatrixMover = 0,
    kPoseConstraintPosition = 1,
    kPoseConstraintRotation = 2,
    kPoseConstraintScale = 3,
    kPoseConstraintParent = 4,
    kPoseConstraintAim = 5
};

// The interpolator driver KIND the PoseInterpolator step selects: a
// NUMERIC driver reads dials, a transform driver reads frames, and an
// ML mover may read both. The method/network/maps tables select the
// solver the shared interpolator arithmetic runs.
enum PoseInterpDriverCode : int {
    kPoseInterpTransform = 0,
    kPoseInterpNumeric = 1
};

namespace pose_detail {

// The propagate-chunk grain both commit folds walk (bakedPose.cpp and
// poseSteps.cpp cut the same 64-wide chunks).
constexpr size_t kPosePropagateChunkSize = 64;

// StepResult details: which binding an Invalid names. Adapters map
// these to their own error or diagnostic text; a detail an adapter
// ignores still documents the failure.
enum PoseStepDetail : int32_t {
    kPoseDetailNone = 0,
    kPoseDetailNoCommit = 1,
    kPoseDetailNoChunk = 2,
    kPoseDetailNoSlot = 3,
    kPoseDetailNoVersion = 4,
    kPoseDetailNoWindow = 5,
    kPoseDetailNoSource = 6,
    kPoseDetailNoSpace = 7,
    kPoseDetailNoDeltaPath = 8,
    kPoseDetailNoTarget = 9,
    kPoseDetailNoCandidate = 10,
    kPoseDetailNoConstraint = 11,
    kPoseDetailNoArrays = 12,
    kPoseDetailNoRecord = 13,
    kPoseDetailNoChain = 14,
    kPoseDetailNoRest = 15,
    kPoseDetailNoPole = 16
};

// The propagation outcomes, in RigExecBakedPropagateOutcome order.
enum PosePropagateOutcome : uint8_t {
    kPosePropagateStaged = 0,
    kPosePropagateSkipped = 1,
    kPosePropagateUnusableDescendant = 3,
    kPosePropagateSingularDelta = 4,
    kPosePropagateInvalidResult = 5
};

} // namespace pose_detail

// The explicitly ordered affine arithmetic every pose body below runs
// through. Each operation spells its scalar order once, in the order
// the runtime's inline mirrors already use, so both instantiations
// agree bitwise under the project's -ffp-contract=off. Nothing here
// calls a precompiled Gf operator.
namespace pose_ops {

// C = A * B, one element as ((a0*b0 + a1*b1) + a2*b2) + a3*b3, the
// order GfMatrix4d::operator*= and RrMat4d::operator*= both spell.
template <class Math>
typename Math::Mat4
Multiply4(const typename Math::Mat4 &a, const typename Math::Mat4 &b)
{
    typename Math::Mat4 c = Math::Identity4();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            c[i][j] =
                ((a[i][0] * b[0][j] + a[i][1] * b[1][j]) + a[i][2] * b[2][j]) +
                a[i][3] * b[3][j];
        }
    }
    return c;
}

// C = A * B for 3x3, ((a0*b0 + a1*b1) + a2*b2) per element.
template <class Math>
typename Math::Mat3
Multiply3(const typename Math::Mat3 &a, const typename Math::Mat3 &b)
{
    typename Math::Mat3 c = Math::Identity3();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            c[i][j] =
                (a[i][0] * b[0][j] + a[i][1] * b[1][j]) + a[i][2] * b[2][j];
        }
    }
    return c;
}

// The 3x3 determinant in Sarrus order, as both mirrors spell it.
template <class Math>
double
Determinant3(const typename Math::Mat3 &m)
{
    const double t0 = m[0][0] * m[1][1] * m[2][2];
    const double t1 = m[0][1] * m[1][2] * m[2][0];
    const double t2 = m[0][2] * m[1][0] * m[2][1];
    const double t3 = m[0][0] * m[1][2] * m[2][1];
    const double t4 = m[0][1] * m[1][0] * m[2][2];
    const double t5 = m[0][2] * m[1][1] * m[2][0];
    return (((t0 + t1) + t2) - t3) - t4 - t5;
}

// One 3x3 minor of rows r1/r2/r3 and columns c1/c2/c3, the
// _GetDeterminant3 both mirrors share.
template <class Math>
double
Determinant3Of(const typename Math::Mat4 &m, size_t r1, size_t r2, size_t r3,
               size_t c1, size_t c2, size_t c3)
{
    const double t0 = m[r1][c1] * m[r2][c2] * m[r3][c3];
    const double t1 = m[r1][c2] * m[r2][c3] * m[r3][c1];
    const double t2 = m[r1][c3] * m[r2][c1] * m[r3][c2];
    const double t3 = m[r1][c1] * m[r2][c3] * m[r3][c2];
    const double t4 = m[r1][c2] * m[r2][c1] * m[r3][c3];
    const double t5 = m[r1][c3] * m[r2][c2] * m[r3][c1];
    return (((t0 + t1) + t2) - t3) - t4 - t5;
}

// The 4x4 determinant by expansion along the last column, as both
// mirrors spell it.
template <class Math>
double
Determinant4(const typename Math::Mat4 &m)
{
    const double t0 = -m[0][3] * Determinant3Of<Math>(m, 1, 2, 3, 0, 1, 2);
    const double t1 = m[1][3] * Determinant3Of<Math>(m, 0, 2, 3, 0, 1, 2);
    const double t2 = -m[2][3] * Determinant3Of<Math>(m, 0, 1, 3, 0, 1, 2);
    const double t3 = m[3][3] * Determinant3Of<Math>(m, 0, 1, 2, 0, 1, 2);
    return ((t0 + t1) + t2) + t3;
}

// The 3x3 inverse both mirrors spell (local cofactors, reciprocal of
// the determinant). On |det| <= eps writes the mirrors' singular
// answer (SetScale(FLT_MAX)) and returns false.
template <class Math>
bool
Invert3(const typename Math::Mat3 &m, double eps, typename Math::Mat3 *out,
        double *detOut)
{
    const double a00 = m[0][0];
    const double a01 = m[0][1];
    const double a02 = m[0][2];
    const double a10 = m[1][0];
    const double a11 = m[1][1];
    const double a12 = m[1][2];
    const double a20 = m[2][0];
    const double a21 = m[2][1];
    const double a22 = m[2][2];
    const double det = (-(a02 * a11 * a20) + a01 * a12 * a20 + a02 * a10 * a21 -
                        a00 * a12 * a21 - a01 * a10 * a22 + a00 * a11 * a22);
    if (detOut) {
        *detOut = det;
    }
    if (!(std::fabs(det) > eps)) {
        *out = Math::Mat3FromElements(double(FLT_MAX), 0, 0, 0, double(FLT_MAX),
                                      0, 0, 0, double(FLT_MAX));
        return false;
    }
    const double rcp = 1.0 / det;
    typename Math::Mat3 inv = Math::Identity3();
    inv[0][0] = (-(a12 * a21) + a11 * a22) * rcp;
    inv[0][1] = (a02 * a21 - a01 * a22) * rcp;
    inv[0][2] = (-(a02 * a11) + a01 * a12) * rcp;
    inv[1][0] = (a12 * a20 - a10 * a22) * rcp;
    inv[1][1] = (-(a02 * a20) + a00 * a22) * rcp;
    inv[1][2] = (a02 * a10 - a00 * a12) * rcp;
    inv[2][0] = (-(a11 * a20) + a10 * a21) * rcp;
    inv[2][1] = (a01 * a20 - a00 * a21) * rcp;
    inv[2][2] = (-(a01 * a10) + a00 * a11) * rcp;
    *out = inv;
    return true;
}

// The 4x4 inverse both mirrors spell (Pickle's column cofactors,
// reciprocal of the determinant). On |det| <= eps writes the
// mirrors' singular answer (SetScale(FLT_MAX): FLT_MAX on the 3x3
// diagonal, 1 in [3][3]) and returns false.
template <class Math>
bool
Invert4(const typename Math::Mat4 &m, double eps, typename Math::Mat4 *out,
        double *detOut)
{
    const double x00 = m[0][0];
    const double x01 = m[0][1];
    const double x10 = m[1][0];
    const double x11 = m[1][1];
    const double x20 = m[2][0];
    const double x21 = m[2][1];
    const double x30 = m[3][0];
    const double x31 = m[3][1];
    double y01 = x00 * x11 - x10 * x01;
    double y02 = x00 * x21 - x20 * x01;
    double y03 = x00 * x31 - x30 * x01;
    double y12 = x10 * x21 - x20 * x11;
    double y13 = x10 * x31 - x30 * x11;
    double y23 = x20 * x31 - x30 * x21;
    const double x02 = m[0][2];
    const double x03 = m[0][3];
    const double x12 = m[1][2];
    const double x13 = m[1][3];
    const double x22 = m[2][2];
    const double x23 = m[2][3];
    const double x32 = m[3][2];
    const double x33 = m[3][3];
    const double z33 = x02 * y12 - x12 * y02 + x22 * y01;
    const double z23 = x12 * y03 - x32 * y01 - x02 * y13;
    const double z13 = x02 * y23 - x22 * y03 + x32 * y02;
    const double z03 = x22 * y13 - x32 * y12 - x12 * y23;
    const double z32 = x13 * y02 - x23 * y01 - x03 * y12;
    const double z22 = x03 * y13 - x13 * y03 + x33 * y01;
    const double z12 = x23 * y03 - x33 * y02 - x03 * y23;
    const double z02 = x13 * y23 - x23 * y13 + x33 * y12;
    y01 = x02 * x13 - x12 * x03;
    y02 = x02 * x23 - x22 * x03;
    y03 = x02 * x33 - x32 * x03;
    y12 = x12 * x23 - x22 * x13;
    y13 = x12 * x33 - x32 * x13;
    y23 = x22 * x33 - x32 * x23;
    const double z30 = x11 * y02 - x21 * y01 - x01 * y12;
    const double z20 = x01 * y13 - x11 * y03 + x31 * y01;
    const double z10 = x21 * y03 - x31 * y02 - x01 * y23;
    const double z00 = x11 * y23 - x21 * y13 + x31 * y12;
    const double z31 = x00 * y12 - x10 * y02 + x20 * y01;
    const double z21 = x10 * y03 - x30 * y01 - x00 * y13;
    const double z11 = x00 * y23 - x20 * y03 + x30 * y02;
    const double z01 = x20 * y13 - x30 * y12 - x10 * y23;
    const double det = ((x30 * z30 + x20 * z20) + x10 * z10) + x00 * z00;
    if (detOut) {
        *detOut = det;
    }
    if (!(std::fabs(det) > eps)) {
        *out = Math::Mat4FromElements(double(FLT_MAX), 0, 0, 0, 0,
                                      double(FLT_MAX), 0, 0, 0, 0,
                                      double(FLT_MAX), 0, 0, 0, 0, 1.0);
        return false;
    }
    const double rcp = 1.0 / det;
    typename Math::Mat4 inv = Math::Identity4();
    inv[0][0] = z00 * rcp;
    inv[0][1] = z10 * rcp;
    inv[1][0] = z01 * rcp;
    inv[0][2] = z20 * rcp;
    inv[2][0] = z02 * rcp;
    inv[0][3] = z30 * rcp;
    inv[3][0] = z03 * rcp;
    inv[1][1] = z11 * rcp;
    inv[1][2] = z21 * rcp;
    inv[2][1] = z12 * rcp;
    inv[1][3] = z31 * rcp;
    inv[3][1] = z13 * rcp;
    inv[2][2] = z22 * rcp;
    inv[2][3] = z32 * rcp;
    inv[3][2] = z23 * rcp;
    inv[3][3] = z33 * rcp;
    *out = inv;
    return true;
}

// The 3-vector dot product, ((x0*y0 + x1*y1) + x2*y2).
template <class Math>
double
Dot3(const typename Math::Vec3 &a, const typename Math::Vec3 &b)
{
    return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2];
}

// The 3-vector cross product, in component order.
template <class Math>
typename Math::Vec3
Cross3(const typename Math::Vec3 &a, const typename Math::Vec3 &b)
{
    return Math::Vec3Of(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                        a[0] * b[1] - a[1] * b[0]);
}

template <class Math>
typename Math::Vec3
Add3(const typename Math::Vec3 &a, const typename Math::Vec3 &b)
{
    return Math::Vec3Of(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

template <class Math>
typename Math::Vec3
Sub3(const typename Math::Vec3 &a, const typename Math::Vec3 &b)
{
    return Math::Vec3Of(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

template <class Math>
typename Math::Vec3
Scale3(const typename Math::Vec3 &a, double s)
{
    return Math::Vec3Of(a[0] * s, a[1] * s, a[2] * s);
}

template <class Math>
double
Length3(const typename Math::Vec3 &a)
{
    return std::sqrt(Dot3<Math>(a, a));
}

// Normalize by the reciprocal, dividing by eps instead when the
// length is at or below it; returns the original length. Both
// mirrors spell it this way.
template <class Math>
double
Normalize3(typename Math::Vec3 *v, double eps)
{
    const double length = Length3<Math>(*v);
    const double inv = 1.0 / ((length > eps) ? length : eps);
    *v = Scale3<Math>(*v, inv);
    return length;
}

template <class Math>
typename Math::Vec3
Normalized3(const typename Math::Vec3 &v, double eps)
{
    typename Math::Vec3 out = v;
    Normalize3<Math>(&out, eps);
    return out;
}

// Scalar and vector closeness, as both mirrors spell them.
inline bool
IsClose(double a, double b, double epsilon)
{
    return std::fabs(a - b) < epsilon;
}

template <class Math>
bool
IsCloseVec3(const typename Math::Vec3 &a, const typename Math::Vec3 &b,
            double tolerance)
{
    const typename Math::Vec3 delta = Sub3<Math>(a, b);
    return Dot3<Math>(delta, delta) <= tolerance * tolerance;
}

// GfMatrix4d::Transform: the row vector (vec, 1) times the matrix,
// projected back by scaling with 1/w, or 1 when w is zero.
template <class Math>
typename Math::Vec3
Transform3(const typename Math::Mat4 &m, const typename Math::Vec3 &v)
{
    const double x =
        ((v[0] * m[0][0] + v[1] * m[1][0]) + v[2] * m[2][0]) + m[3][0];
    const double y =
        ((v[0] * m[0][1] + v[1] * m[1][1]) + v[2] * m[2][1]) + m[3][1];
    const double z =
        ((v[0] * m[0][2] + v[1] * m[1][2]) + v[2] * m[2][2]) + m[3][2];
    const double w =
        ((v[0] * m[0][3] + v[1] * m[1][3]) + v[2] * m[2][3]) + m[3][3];
    const double inv = (w != 0.0) ? 1.0 / w : 1.0;
    return Math::Vec3Of(inv * x, inv * y, inv * z);
}

// The direction transform (no translation, no projection).
template <class Math>
typename Math::Vec3
TransformDir3(const typename Math::Mat4 &m, const typename Math::Vec3 &v)
{
    return Math::Vec3Of((v[0] * m[0][0] + v[1] * m[1][0]) + v[2] * m[2][0],
                        (v[0] * m[0][1] + v[1] * m[1][1]) + v[2] * m[2][1],
                        (v[0] * m[0][2] + v[1] * m[1][2]) + v[2] * m[2][2]);
}

// The affine transform (fourth column assumed (0, 0, 0, 1)).
template <class Math>
typename Math::Vec3
TransformAffine3(const typename Math::Mat4 &m, const typename Math::Vec3 &v)
{
    const double x =
        ((v[0] * m[0][0] + v[1] * m[1][0]) + v[2] * m[2][0]) + m[3][0];
    const double y =
        ((v[0] * m[0][1] + v[1] * m[1][1]) + v[2] * m[2][1]) + m[3][1];
    const double z =
        ((v[0] * m[0][2] + v[1] * m[1][2]) + v[2] * m[2][2]) + m[3][2];
    return Math::Vec3Of(x, y, z);
}

// Point-frame flag bits, spelled once here; both frame types use
// 1/2/4 for valid/degenerate/reflected.
constexpr unsigned kPoseFrameValid = 1u;
constexpr unsigned kPoseFrameDegenerate = 2u;
constexpr unsigned kPoseFrameReflected = 4u;
constexpr unsigned kPoseFrameAffine = 8u;

// The identity landmarks both frame types convert against.
template <class Math>
std::array<typename Math::Vec3, 4>
IdentityLandmarks()
{
    return {Math::Vec3Of(0, 0, 0), Math::Vec3Of(1, 0, 0), Math::Vec3Of(0, 1, 0),
            Math::Vec3Of(0, 0, 1)};
}

// The row basis of four points: rows p1-p0, p2-p0, p3-p0.
template <class Math>
typename Math::Mat3
RowBasis3(const std::array<typename Math::Vec3, 4> &pts)
{
    return Math::Mat3FromElements(
        pts[1][0] - pts[0][0], pts[1][1] - pts[0][1], pts[1][2] - pts[0][2],
        pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2],
        pts[3][0] - pts[0][0], pts[3][1] - pts[0][1], pts[3][2] - pts[0][2]);
}

// The frame epsilon: 1e-10 over the rest edge lengths and 1.
template <class Math>
double
FrameEpsilon(const std::array<typename Math::Vec3, 4> &restPoints)
{
    const double lx = Length3<Math>(Sub3<Math>(restPoints[1], restPoints[0]));
    const double ly = Length3<Math>(Sub3<Math>(restPoints[2], restPoints[0]));
    const double lz = Length3<Math>(Sub3<Math>(restPoints[3], restPoints[0]));
    return 1e-10 * std::max({1.0, lx, ly, lz});
}

// The affine map restPoints -> posePoints, decomposed through the row
// bases: m3 = restBasis^-1 * poseBasis, t = pose0 - rest0*m3. False
// for a singular rest (leaves *matrix as identity there, as both
// mirrors do).
template <class Math>
bool
PointsToMatrix(const std::array<typename Math::Vec3, 4> &restPoints,
               const std::array<typename Math::Vec3, 4> &posePoints,
               typename Math::Mat4 *matrix)
{
    const typename Math::Mat3 mq = RowBasis3<Math>(restPoints);
    const typename Math::Mat3 mp = RowBasis3<Math>(posePoints);
    const double det = Determinant3<Math>(mq);
    const double eps = FrameEpsilon<Math>(restPoints);
    if (std::fabs(det) < eps * eps * eps) {
        if (matrix) {
            *matrix = Math::Identity4();
        }
        return false;
    }
    typename Math::Mat3 inv = Math::Identity3();
    Invert3<Math>(mq, 0.0, &inv, nullptr);
    const typename Math::Mat3 m3 = Multiply3<Math>(inv, mp);
    const typename Math::Vec3 t = Sub3<Math>(
        posePoints[0],
        Math::Vec3Of(
            (restPoints[0][0] * m3[0][0] + restPoints[0][1] * m3[1][0]) +
                restPoints[0][2] * m3[2][0],
            (restPoints[0][0] * m3[0][1] + restPoints[0][1] * m3[1][1]) +
                restPoints[0][2] * m3[2][1],
            (restPoints[0][0] * m3[0][2] + restPoints[0][1] * m3[1][2]) +
                restPoints[0][2] * m3[2][2]));
    if (matrix) {
        *matrix = Math::Mat4FromElements(
            m3[0][0], m3[0][1], m3[0][2], 0.0, m3[1][0], m3[1][1], m3[1][2],
            0.0, m3[2][0], m3[2][1], m3[2][2], 0.0, t[0], t[1], t[2], 1.0);
    }
    return true;
}

// A rotation about a unit axis through the quaternion both rotation
// types spell: half-angle sincos, axis scaled by the sine, normalized
// as the quaternion Normalize spells it (the real part divided, the
// imaginary scaled by the reciprocal; identity below the floor).
template <class Math>
typename Math::Quat
QuatFromAxisAngle(const typename Math::Vec3 &axis, double degrees)
{
    constexpr double pi = 3.141592653589793;
    const double radians = degrees * (pi / 180.0) / 2.0;
    double sinR = 0.0;
    double cosR = 1.0;
    Math::SinCos(radians, &sinR, &cosR);
    const typename Math::Vec3 im = Scale3<Math>(axis, sinR);
    const double len = std::sqrt(Dot3<Math>(im, im) + cosR * cosR);
    if (len < 1e-10) {
        return Math::QuatFromComponents(1.0, 0.0, 0.0, 0.0);
    }
    const double inv = 1.0 / len;
    return Math::QuatFromComponents(cosR / len, im[0] * inv, im[1] * inv,
                                    im[2] * inv);
}

// The 3x3 rotation block of a unit quaternion, as both
// _SetRotateFromQuat spell it.
template <class Math>
void
SetRotateFromQuat(double r, const typename Math::Vec3 &i,
                  typename Math::Mat4 *m)
{
    (*m)[0][0] = 1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2]);
    (*m)[0][1] = 2.0 * (i[0] * i[1] + i[2] * r);
    (*m)[0][2] = 2.0 * (i[2] * i[0] - i[1] * r);
    (*m)[1][0] = 2.0 * (i[0] * i[1] - i[2] * r);
    (*m)[1][1] = 1.0 - 2.0 * (i[2] * i[2] + i[0] * i[0]);
    (*m)[1][2] = 2.0 * (i[1] * i[2] + i[0] * r);
    (*m)[2][0] = 2.0 * (i[2] * i[0] + i[1] * r);
    (*m)[2][1] = 2.0 * (i[1] * i[2] - i[0] * r);
    (*m)[2][2] = 1.0 - 2.0 * (i[1] * i[1] + i[0] * i[0]);
}

// The rotation matrix M(R(axis, degrees), 0): the quaternion above
// through the block above, with a zero translation and unit corner.
template <class Math>
typename Math::Mat4
RotationMatrix(const typename Math::Vec3 &axis, double degrees)
{
    double w = 1.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(QuatFromAxisAngle<Math>(axis, degrees), &w, &im);
    typename Math::Mat4 m = Math::Identity4();
    SetRotateFromQuat<Math>(w, im, &m);
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;
    return m;
}

// Slot avars as one matrix: S * R(order) * Rspin * T, in the order
// RigExecProviderComposeAvars and _RrComposeAvars both spell. Scales
// normalize through RigExecNormalizeAvarScale; a zero angle skips its
// rotation; anything but a 3-letter order composes XYZ.
template <class Math>
typename Math::Mat4
ComposeAvars(double tx, double ty, double tz, double sx, double sy, double sz,
             double rx, double ry, double rz, double spin,
             std::string_view order)
{
    const typename Math::Vec3 axes[3] = {
        Math::Vec3Of(1, 0, 0), Math::Vec3Of(0, 1, 0), Math::Vec3Of(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    const std::string_view sequence =
        order.size() == 3 ? order : std::string_view(SchemaValueText::xyz);
    typename Math::Mat4 m =
        Math::Mat4FromElements(RigExecNormalizeAvarScale(sx), 0, 0, 0, 0,
                               RigExecNormalizeAvarScale(sy), 0, 0, 0, 0,
                               RigExecNormalizeAvarScale(sz), 0, 0, 0, 0, 1.0);
    for (char axis : sequence) {
        const int i = axis == 'X' ? 0 : axis == 'Y' ? 1 : 2;
        if (angles[i] != 0.0) {
            m = Multiply4<Math>(m, RotationMatrix<Math>(axes[i], angles[i]));
        }
    }
    if (spin != 0.0) {
        m = Multiply4<Math>(m, RotationMatrix<Math>(axes[0], spin));
    }
    const typename Math::Mat4 t = Math::Mat4FromElements(
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, tx, ty, tz, 1.0);
    return Multiply4<Math>(m, t);
}

// GfOrthogonalizeBasis, arithmetic untouched: the iterative
// orthonormalization the 4x4 Orthonormalize runs over its rows.
template <class Math>
bool
OrthogonalizeBasis3(typename Math::Vec3 *tx, typename Math::Vec3 *ty,
                    typename Math::Vec3 *tz, bool normalize, double eps)
{
    typename Math::Vec3 ax = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 bx = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 cx = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 ay = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 by = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 cy = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 az = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 bz = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 cz = Math::Vec3Of(0, 0, 0);
    if (normalize) {
        Normalize3<Math>(tx, 1e-10);
        Normalize3<Math>(ty, 1e-10);
        Normalize3<Math>(tz, 1e-10);
        ax = *tx;
        ay = *ty;
        az = *tz;
    } else {
        ax = *tx;
        ay = *ty;
        az = *tz;
        Normalize3<Math>(&ax, 1e-10);
        Normalize3<Math>(&ay, 1e-10);
        Normalize3<Math>(&az, 1e-10);
    }
    if (IsCloseVec3<Math>(ax, ay, eps) || IsCloseVec3<Math>(ax, az, eps) ||
        IsCloseVec3<Math>(ay, az, eps)) {
        return false;
    }
    const int maxIters = 20;
    int iter = 0;
    for (; iter < maxIters; ++iter) {
        bx = *tx;
        by = *ty;
        bz = *tz;
        bx = Sub3<Math>(bx, Scale3<Math>(ay, Dot3<Math>(ay, bx)));
        bx = Sub3<Math>(bx, Scale3<Math>(az, Dot3<Math>(az, bx)));
        by = Sub3<Math>(by, Scale3<Math>(ax, Dot3<Math>(ax, by)));
        by = Sub3<Math>(by, Scale3<Math>(az, Dot3<Math>(az, by)));
        bz = Sub3<Math>(bz, Scale3<Math>(ax, Dot3<Math>(ax, bz)));
        bz = Sub3<Math>(bz, Scale3<Math>(ay, Dot3<Math>(ay, bz)));
        cx = Scale3<Math>(Add3<Math>(*tx, bx), 0.5);
        cy = Scale3<Math>(Add3<Math>(*ty, by), 0.5);
        cz = Scale3<Math>(Add3<Math>(*tz, bz), 0.5);
        if (normalize) {
            Normalize3<Math>(&cx, 1e-10);
            Normalize3<Math>(&cy, 1e-10);
            Normalize3<Math>(&cz, 1e-10);
        }
        const typename Math::Vec3 xDiff = Sub3<Math>(*tx, cx);
        const typename Math::Vec3 yDiff = Sub3<Math>(*ty, cy);
        const typename Math::Vec3 zDiff = Sub3<Math>(*tz, cz);
        const double error =
            (Dot3<Math>(xDiff, xDiff) + Dot3<Math>(yDiff, yDiff)) +
            Dot3<Math>(zDiff, zDiff);
        if (error < eps * eps) {
            break;
        }
        *tx = cx;
        *ty = cy;
        *tz = cz;
        ax = *tx;
        ay = *ty;
        az = *tz;
        if (!normalize) {
            Normalize3<Math>(&ax, 1e-10);
            Normalize3<Math>(&ay, 1e-10);
            Normalize3<Math>(&az, 1e-10);
        }
    }
    return iter < maxIters;
}

// The 4x4 Orthonormalize: orthonormal rows, then the homogeneous
// coordinate divided out unless it is 1 or close to 0.
template <class Math>
bool
Orthonormalize4(typename Math::Mat4 *m)
{
    typename Math::Vec3 r0 = Math::Vec3Of((*m)[0][0], (*m)[0][1], (*m)[0][2]);
    typename Math::Vec3 r1 = Math::Vec3Of((*m)[1][0], (*m)[1][1], (*m)[1][2]);
    typename Math::Vec3 r2 = Math::Vec3Of((*m)[2][0], (*m)[2][1], (*m)[2][2]);
    const bool result = OrthogonalizeBasis3<Math>(&r0, &r1, &r2, true, 1e-6);
    (*m)[0][0] = r0[0];
    (*m)[0][1] = r0[1];
    (*m)[0][2] = r0[2];
    (*m)[1][0] = r1[0];
    (*m)[1][1] = r1[1];
    (*m)[1][2] = r1[2];
    (*m)[2][0] = r2[0];
    (*m)[2][1] = r2[1];
    (*m)[2][2] = r2[2];
    if ((*m)[3][3] != 1.0 && !IsClose((*m)[3][3], 0.0, 1e-10)) {
        (*m)[3][0] /= (*m)[3][3];
        (*m)[3][1] /= (*m)[3][3];
        (*m)[3][2] /= (*m)[3][3];
        (*m)[3][3] = 1.0;
    }
    return result;
}

// The 4x4 ExtractRotationQuat both mirrors spell (largest-diagonal
// branch first, the int-typed 4 in the else arm, clamped real part,
// no normalization after).
template <class Math>
typename Math::Quat
ExtractRotationQuat(const typename Math::Mat4 &m)
{
    int i = 0;
    if (m[0][0] > m[1][1]) {
        i = (m[0][0] > m[2][2] ? 0 : 2);
    } else {
        i = (m[1][1] > m[2][2] ? 1 : 2);
    }
    double im[3] = {0.0, 0.0, 0.0};
    double r = 0.0;
    if (m[0][0] + m[1][1] + m[2][2] > m[i][i]) {
        r = 0.5 * std::sqrt(m[0][0] + m[1][1] + m[2][2] + m[3][3]);
        im[0] = (m[1][2] - m[2][1]) / (4.0 * r);
        im[1] = (m[2][0] - m[0][2]) / (4.0 * r);
        im[2] = (m[0][1] - m[1][0]) / (4.0 * r);
    } else {
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const double q = 0.5 * std::sqrt(m[i][i] - m[j][j] - m[k][k] + m[3][3]);
        im[i] = q;
        im[j] = (m[i][j] + m[j][i]) / (4 * q);
        im[k] = (m[k][i] + m[i][k]) / (4 * q);
        r = (m[j][k] - m[k][j]) / (4 * q);
    }
    const double clamped = r < -1.0 ? -1.0 : r > 1.0 ? 1.0 : r;
    return Math::QuatFromComponents(clamped, im[0], im[1], im[2]);
}

// A matrix as a point frame: the identity landmarks through the
// affine transform; a non-finite component marks the frame
// degenerate, keeps that landmark and leaves the later ones at
// identity (RigExecMatrixLandmarksT). Returns the points and flags
// separately so bodies can publish through either frame type.
template <class Math>
void
FrameFromMatrix(const typename Math::Mat4 &m,
                std::array<typename Math::Vec3, 4> *points, unsigned *flags)
{
    const std::array<typename Math::Vec3, 4> identity =
        IdentityLandmarks<Math>();
    *points = identity;
    for (size_t i = 0; i < 4; ++i) {
        (*points)[i] = TransformAffine3<Math>(m, identity[i]);
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite((*points)[i][a])) {
                *flags = kPoseFrameDegenerate;
                return;
            }
        }
    }
    *flags = kPoseFrameValid;
}

// Rest landmarks through a matrix, with the reflected/degenerate
// classification both MatrixToPoints spell.
template <class Math>
void
MatrixToPoints(const std::array<typename Math::Vec3, 4> &restPoints,
               const typename Math::Mat4 &matrix,
               std::array<typename Math::Vec3, 4> *points, unsigned *flags)
{
    for (size_t i = 0; i < 4; ++i) {
        (*points)[i] = TransformAffine3<Math>(matrix, restPoints[i]);
    }
    unsigned out = kPoseFrameValid;
    const typename Math::Mat3 posed = RowBasis3<Math>(*points);
    if (Determinant3<Math>(posed) < 0) {
        out |= kPoseFrameReflected;
    }
    const double eps = FrameEpsilon<Math>(restPoints);
    if (std::fabs(Determinant3<Math>(posed)) <= eps * eps * eps) {
        out |= kPoseFrameDegenerate;
    }
    *flags = out;
}

// The frame round trip both RoundTrips spell: points from the
// matrix, then the matrix back from the identity landmarks.
template <class Math>
typename Math::Mat4
RoundTripMatrix(const typename Math::Mat4 &m)
{
    std::array<typename Math::Vec3, 4> points = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned flags = 0;
    FrameFromMatrix<Math>(m, &points, &flags);
    typename Math::Mat4 out = Math::Identity4();
    PointsToMatrix<Math>(IdentityLandmarks<Math>(), points, &out);
    return out;
}

// _SpaceOfFrame: an unusable frame selects the NaN sentinel, so the
// failure survives into every descendant instead of being scrubbed
// into a plausible identity.
template <class Math>
typename Math::Mat4
SpaceOfFrame(const std::array<typename Math::Vec3, 4> &points, unsigned flags)
{
    typename Math::Mat4 space = Math::Identity4();
    if (!(flags & kPoseFrameValid) || (flags & kPoseFrameDegenerate) ||
        !PointsToMatrix<Math>(IdentityLandmarks<Math>(), points, &space)) {
        space = Math::Identity4();
        space[3][0] = std::numeric_limits<double>::quiet_NaN();
    }
    return space;
}

// Clamp, branch for branch.
inline double
Clamp(double value, double lo, double hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

// BlendEnvelope: the endpoint branches are the contract.
inline double
BlendEnvelope(double preceding, double full, double weight)
{
    if (weight <= 0.0) {
        return preceding;
    }
    if (weight >= 1.0) {
        return full;
    }
    return preceding + (full - preceding) * weight;
}

// The inverse as a value: Invert4 with the default epsilon,
// discarding the determinant.
template <class Math>
typename Math::Mat4
Invert4Result(const typename Math::Mat4 &m)
{
    typename Math::Mat4 out = Math::Identity4();
    Invert4<Math>(m, 0.0, &out, nullptr);
    return out;
}

// The translation row as a vector.
template <class Math>
typename Math::Vec3
ExtractTranslation(const typename Math::Mat4 &m)
{
    return Math::Vec3Of(m[3][0], m[3][1], m[3][2]);
}

// M(transform) * inverse(M(space)) with the projective column set
// exactly.
template <class Math>
typename Math::Mat4
MeasureInSpace(const typename Math::Mat4 &transform,
               const typename Math::Mat4 &space)
{
    typename Math::Mat4 inverse = Math::Identity4();
    Invert4<Math>(space, 0.0, &inverse, nullptr);
    typename Math::Mat4 m = Multiply4<Math>(transform, inverse);
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}

// The partial-transform decomposition: the original, the row
// scales, the rotation axis/angle in degrees, and the translation.
template <class Math> struct PosePartialDecompositionT {
    typename Math::Mat4 original;
    typename Math::Vec3 scale;
    typename Math::Vec3 axis;
    typename Math::Vec3 translation;
    double angle = 0.0;
};

template <class Math>
PosePartialDecompositionT<Math>
DecomposePartialTransform(const typename Math::Mat4 &transform)
{
    PosePartialDecompositionT<Math> d;
    d.original = transform;
    typename Math::Mat4 basis = transform;
    basis[3][0] = 0.0;
    basis[3][1] = 0.0;
    basis[3][2] = 0.0;
    typename Math::Vec3 scale = Math::Vec3Of(1, 1, 1);
    for (int axis = 0; axis < 3; ++axis) {
        const typename Math::Vec3 row =
            Math::Vec3Of(basis[axis][0], basis[axis][1], basis[axis][2]);
        const double length = Length3<Math>(row);
        if (length > 1e-12) {
            scale[axis] = length;
            const typename Math::Vec3 unit = Scale3<Math>(row, 1.0 / length);
            basis[axis][0] = unit[0];
            basis[axis][1] = unit[1];
            basis[axis][2] = unit[2];
        }
    }
    d.scale = scale;
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(ExtractRotationQuat<Math>(basis), &w, &im);
    const double len = Length3<Math>(im);
    typename Math::Vec3 rotAxis = Math::Vec3Of(1, 0, 0);
    double angle = 0.0;
    if (len > 1e-10) {
        const double x = std::acos(std::min(std::max(w, -1.0), 1.0));
        rotAxis = Scale3<Math>(im, 1.0 / len);
        if (std::fabs(Dot3<Math>(rotAxis, rotAxis) - 1.0) >= 1e-10) {
            const double nlen = Length3<Math>(rotAxis);
            rotAxis =
                Scale3<Math>(rotAxis, 1.0 / (nlen > 1e-10 ? nlen : 1e-10));
        }
        angle = 2.0 * x * (180.0 / 3.14159265358979323846);
    }
    const double axisLen = Length3<Math>(rotAxis);
    d.axis = Scale3<Math>(rotAxis, 1.0 / (axisLen > 1e-10 ? axisLen : 1e-10));
    if (angle > 180.0) {
        angle -= 360.0;
    }
    d.angle = angle;
    d.translation =
        Math::Vec3Of(transform[3][0], transform[3][1], transform[3][2]);
    return d;
}

template <class Math>
typename Math::Mat4
ApplyPartialDecomposition(const PosePartialDecompositionT<Math> &d,
                          double weight)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (w <= 0.0) {
        return Math::Identity4();
    }
    if (w >= 1.0) {
        return d.original;
    }
    const typename Math::Vec3 scale = d.scale;
    const typename Math::Vec3 axis = d.axis;
    const typename Math::Vec3 translation = d.translation;
    const double angle = d.angle;
    typename Math::Mat4 scaled = Math::Identity4();
    scaled[0][0] = 1.0 + (scale[0] - 1.0) * w;
    scaled[1][1] = 1.0 + (scale[1] - 1.0) * w;
    scaled[2][2] = 1.0 + (scale[2] - 1.0) * w;
    typename Math::Mat4 out = Math::Identity4();
    double qw = 0.0;
    typename Math::Vec3 qim = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(QuatFromAxisAngle<Math>(axis, angle * w), &qw, &qim);
    SetRotateFromQuat<Math>(qw, qim, &out);
    out = Multiply4<Math>(scaled, out);
    const double radians = angle * (3.14159265358979323846 / 180.0);
    if (std::abs(std::sin(0.5 * radians)) < 1e-4) {
        out[3][0] = translation[0] * w;
        out[3][1] = translation[1] * w;
        out[3][2] = translation[2] * w;
        return out;
    }
    const double along = Dot3<Math>(translation, axis);
    const typename Math::Vec3 across =
        Sub3<Math>(translation, Scale3<Math>(axis, along));
    const double half = 0.5 / std::tan(0.5 * radians);
    const typename Math::Vec3 pivot =
        Add3<Math>(Scale3<Math>(across, 0.5),
                   Scale3<Math>(Cross3<Math>(axis, across), half));
    const typename Math::Vec3 shifted =
        Add3<Math>(Sub3<Math>(pivot, TransformDir3<Math>(out, pivot)),
                   Scale3<Math>(axis, along * w));
    out[3][0] = shifted[0];
    out[3][1] = shifted[1];
    out[3][2] = shifted[2];
    return out;
}

template <class Math>
typename Math::Mat4
PartialTransform(const typename Math::Mat4 &transform, double weight)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (w <= 0.0) {
        return Math::Identity4();
    }
    if (w >= 1.0) {
        return transform;
    }
    return ApplyPartialDecomposition<Math>(
        DecomposePartialTransform<Math>(transform), w);
}

// Quaternion length squared: imaginary dot plus real squared.
template <class Math>
double
QuatLengthSquared(double w, const typename Math::Vec3 &im)
{
    return Dot3<Math>(im, im) + w * w;
}

// Quaternion normalize: identity below the floor, else the real part
// divided and the imaginary scaled by the reciprocal.
template <class Math>
typename Math::Quat
QuatNormalized(const typename Math::Quat &q, double eps)
{
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q, &w, &im);
    const double len = std::sqrt(QuatLengthSquared<Math>(w, im));
    if (len < eps) {
        return Math::QuatFromComponents(1.0, 0.0, 0.0, 0.0);
    }
    const double inv = 1.0 / len;
    return Math::QuatFromComponents(w / len, im[0] * inv, im[1] * inv,
                                    im[2] * inv);
}

// The Hamilton product, as both mirrors spell it.
template <class Math>
typename Math::Quat
QuatMultiply(const typename Math::Quat &q1, const typename Math::Quat &q2)
{
    double r1 = 0.0;
    double r2 = 0.0;
    typename Math::Vec3 i1 = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 i2 = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q1, &r1, &i1);
    Math::QuatToComponents(q2, &r2, &i2);
    return Math::QuatFromComponents(
        r1 * r2 - Dot3<Math>(i1, i2),
        (r1 * i2[0] + r2 * i1[0]) + (i1[1] * i2[2] - i1[2] * i2[1]),
        (r1 * i2[1] + r2 * i1[1]) + (i1[2] * i2[0] - i1[0] * i2[2]),
        (r1 * i2[2] + r2 * i1[2]) + (i1[0] * i2[1] - i1[1] * i2[0]));
}

// Conjugate, inverse (conjugate over length squared), dot.
template <class Math>
typename Math::Quat
QuatConjugate(const typename Math::Quat &q)
{
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q, &w, &im);
    return Math::QuatFromComponents(w, -im[0], -im[1], -im[2]);
}

template <class Math>
typename Math::Quat
QuatInverse(const typename Math::Quat &q)
{
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q, &w, &im);
    // Conjugate divided: the real part divided, the imaginary scaled
    // by the reciprocal, exactly as operator/= spells it.
    const double lenSq = Dot3<Math>(im, im) + w * w;
    return Math::QuatFromComponents(w / lenSq, -im[0] * (1.0 / lenSq),
                                    -im[1] * (1.0 / lenSq),
                                    -im[2] * (1.0 / lenSq));
}

template <class Math>
double
QuatDot(const typename Math::Quat &q1, const typename Math::Quat &q2)
{
    double r1 = 0.0;
    double r2 = 0.0;
    typename Math::Vec3 i1 = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 i2 = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q1, &r1, &i1);
    Math::QuatToComponents(q2, &r2, &i2);
    return Dot3<Math>(i1, i2) + r1 * r2;
}

// Slerp, as both mirrors spell it (the dot, the flip, the small-angle
// lerp, the scale combination).
template <class Math>
typename Math::Quat
QuatSlerp(double alpha, const typename Math::Quat &q0,
          const typename Math::Quat &q1)
{
    double cosTheta = QuatDot<Math>(q0, q1);
    bool flip1 = false;
    if (cosTheta < 0.0) {
        cosTheta = -cosTheta;
        flip1 = true;
    }
    double scale0 = 0.0;
    double scale1 = 0.0;
    if (1.0 - cosTheta > 0.00001) {
        const double theta = std::acos(cosTheta);
        const double sinTheta = std::sin(theta);
        scale0 = std::sin((1.0 - alpha) * theta) / sinTheta;
        scale1 = std::sin(alpha * theta) / sinTheta;
    } else {
        scale0 = 1.0 - alpha;
        scale1 = alpha;
    }
    if (flip1) {
        scale1 = -scale1;
    }
    double r0 = 0.0;
    double r1 = 0.0;
    typename Math::Vec3 i0 = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 i1 = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q0, &r0, &i0);
    Math::QuatToComponents(q1, &r1, &i1);
    return Math::QuatFromComponents(
        scale0 * r0 + scale1 * r1, scale0 * i0[0] + scale1 * i1[0],
        scale0 * i0[1] + scale1 * i1[1], scale0 * i0[2] + scale1 * i1[2]);
}

// Rotate a point by a unit quaternion: the expanded q*p*q'
// without forming the products.
template <class Math>
typename Math::Vec3
QuatTransform(const typename Math::Quat &q, const typename Math::Vec3 &point)
{
    double r = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q, &r, &im);
    const double tmpDot = Dot3<Math>(im, im);
    const double tmpSqr = r * r;
    const typename Math::Vec3 term0 =
        Scale3<Math>(im, 2 * Dot3<Math>(im, point));
    const typename Math::Vec3 term1 = Scale3<Math>(point, tmpSqr - tmpDot);
    const typename Math::Vec3 term2 =
        Scale3<Math>(Cross3<Math>(im, point), 2 * r);
    const typename Math::Vec3 sum = Add3<Math>(Add3<Math>(term0, term1), term2);
    const double inv = 1.0 / (tmpSqr + tmpDot);
    return Scale3<Math>(sum, inv);
}

// One axis-angle rotation as plain data (the rotation types'
// axis/angle pair, without their registry).
template <class Math> struct PoseRotationT {
    typename Math::Vec3 axis;
    double angle = 0.0;
};

// SetRotateInto: the rotation taking rotateFrom onto rotateTo.
template <class Math>
PoseRotationT<Math>
RotateInto(const typename Math::Vec3 &rotateFrom,
           const typename Math::Vec3 &rotateTo)
{
    PoseRotationT<Math> out;
    const typename Math::Vec3 from = Normalized3<Math>(rotateFrom, 1e-10);
    const typename Math::Vec3 to = Normalized3<Math>(rotateTo, 1e-10);
    const double cos = Dot3<Math>(from, to);
    if (cos > 0.9999999) {
        out.axis = Math::Vec3Of(1.0, 0.0, 0.0);
        out.angle = 0.0;
        return out;
    }
    if (cos < -0.9999999) {
        typename Math::Vec3 tmp =
            Cross3<Math>(from, Math::Vec3Of(1.0, 0.0, 0.0));
        if (Length3<Math>(tmp) < 0.00001) {
            tmp = Cross3<Math>(from, Math::Vec3Of(0.0, 1.0, 0.0));
        }
        out.axis = Normalized3<Math>(tmp, 1e-10);
        // SetAxisAngle normalizes a non-unit axis; the normalized
        // cross product is unit up to rounding, and the 1e-10
        // closeness test passes it through unchanged.
        out.angle = 180.0;
        return out;
    }
    typename Math::Vec3 axis =
        Normalized3<Math>(Cross3<Math>(rotateFrom, rotateTo), 1e-10);
    // The axis/angle pair SetAxisAngle keeps (see above).
    out.axis = axis;
    constexpr double pi = 3.141592653589793;
    out.angle = std::acos(cos) * (180.0 / pi);
    return out;
}

// Rotation::Decompose: the rotation as euler about three axes.
template <class Math>
typename Math::Vec3
RotationDecompose(const PoseRotationT<Math> &rotation,
                  const typename Math::Vec3 &axis0,
                  const typename Math::Vec3 &axis1,
                  const typename Math::Vec3 &axis2)
{
    typename Math::Mat4 mat = Math::Identity4();
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(
        QuatFromAxisAngle<Math>(rotation.axis, rotation.angle), &w, &im);
    SetRotateFromQuat<Math>(w, im, &mat);
    const typename Math::Vec3 n0 = Normalized3<Math>(axis0, 1e-10);
    const typename Math::Vec3 n1 = Normalized3<Math>(axis1, 1e-10);
    const typename Math::Vec3 n2 = Normalized3<Math>(axis2, 1e-10);
    const typename Math::Mat4 axes =
        Math::Mat4FromElements(n0[0], n1[0], n2[0], 0, n0[1], n1[1], n2[1], 0,
                               n0[2], n1[2], n2[2], 0, 0, 0, 0, 1);
    typename Math::Mat4 axesT = Math::Identity4();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            axesT[r][c] = axes[c][r];
        }
    }
    const typename Math::Mat4 m =
        Multiply4<Math>(Multiply4<Math>(axesT, mat), axes);
    constexpr int i = 0;
    constexpr int j = 1;
    constexpr int k = 2;
    double r0 = 0.0;
    double r1 = 0.0;
    double r2 = 0.0;
    const double cy = std::sqrt(m[i][i] * m[i][i] + m[j][i] * m[j][i]);
    if (cy > 1e-6) {
        r0 = std::atan2(m[k][j], m[k][k]);
        r1 = std::atan2(-m[k][i], cy);
        r2 = std::atan2(m[j][i], m[i][i]);
    } else {
        r0 = std::atan2(-m[j][k], m[j][j]);
        r1 = std::atan2(-m[k][i], cy);
        r2 = 0;
    }
    const double hand = Dot3<Math>(Cross3<Math>(n0, n1), n2);
    if (hand >= 0.0) {
        r0 = -r0;
        r1 = -r1;
        r2 = -r2;
    }
    constexpr double pi = 3.141592653589793;
    return Math::Vec3Of(r0 * (180.0 / pi), r1 * (180.0 / pi),
                        r2 * (180.0 / pi));
}

// Rotation::TransformDir: the rotation as a matrix, transforming a
// direction.
template <class Math>
typename Math::Vec3
RotationTransformDir(const PoseRotationT<Math> &rotation,
                     const typename Math::Vec3 &vec)
{
    return TransformDir3<Math>(
        RotationMatrix<Math>(rotation.axis, rotation.angle), vec);
}

// RotateOntoProjected: the twist about the axis taking v1's
// projection onto v2's.
template <class Math>
PoseRotationT<Math>
RotateOntoProjected(const typename Math::Vec3 &v1,
                    const typename Math::Vec3 &v2,
                    const typename Math::Vec3 &axisParam)
{
    const typename Math::Vec3 axis = Normalized3<Math>(axisParam, 1e-10);
    typename Math::Vec3 v1Proj =
        Sub3<Math>(v1, Scale3<Math>(axis, Dot3<Math>(v1, axis)));
    typename Math::Vec3 v2Proj =
        Sub3<Math>(v2, Scale3<Math>(axis, Dot3<Math>(v2, axis)));
    Normalize3<Math>(&v1Proj, 1e-10);
    Normalize3<Math>(&v2Proj, 1e-10);
    const double sinTheta = Dot3<Math>(Cross3<Math>(v1Proj, v2Proj), axis);
    const double cosTheta = Dot3<Math>(v1Proj, v2Proj);
    double theta = 0;
    if (!(std::fabs(sinTheta) < 1e-6 && std::fabs(cosTheta) < 1e-6)) {
        theta = std::atan2(sinTheta, cosTheta);
    }
    constexpr double pi = 3.141592653589793;
    PoseRotationT<Math> out;
    out.axis = axis;
    out.angle = theta * (180.0 / pi);
    return out;
}

// Jacobi eigen decomposition of a symmetric 3x3 (pointFrame.cpp):
// eigenvalues descending, eigenvectors as the columns of V.
inline void
SymmetricEigen3(const double m[3][3], double eval[3], double evec[3][3])
{
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = m[i][j];
        }
    }
    for (int sweep = 0; sweep < 64; ++sweep) {
        const double off =
            std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (off < 1e-15) {
            break;
        }
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-18) {
                    continue;
                }
                const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t =
                    (theta >= 0 ? 1.0 : -1.0) /
                    (std::abs(theta) + std::sqrt(theta * theta + 1));
                const double cth = 1.0 / std::sqrt(t * t + 1);
                const double sth = t * cth;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p];
                    const double akq = a[k][q];
                    a[k][p] = cth * akp - sth * akq;
                    a[k][q] = sth * akp + cth * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k];
                    const double aqk = a[q][k];
                    a[p][k] = cth * apk - sth * aqk;
                    a[q][k] = sth * apk + cth * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p];
                    const double vkq = v[k][q];
                    v[k][p] = cth * vkp - sth * vkq;
                    v[k][q] = sth * vkp + cth * vkq;
                }
            }
        }
    }
    int order[3] = {0, 1, 2};
    double d[3] = {a[0][0], a[1][1], a[2][2]};
    std::sort(order, order + 3, [&](int x, int y) { return d[x] > d[y]; });
    for (int i = 0; i < 3; ++i) {
        eval[i] = d[order[i]];
        for (int k = 0; k < 3; ++k) {
            evec[k][i] = v[k][order[i]];
        }
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(evec[k][i]) > std::abs(evec[maxK][i])) {
                maxK = k;
            }
        }
        if (evec[maxK][i] < 0) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = -evec[k][i];
            }
        }
    }
    const double tol =
        1e-8 * std::max({std::abs(eval[0]), std::abs(eval[2]), 1.0});
    const bool eq01 = std::abs(eval[0] - eval[1]) <= tol;
    const bool eq12 = std::abs(eval[1] - eval[2]) <= tol;
    if (eq01 && eq12) {
        for (int i = 0; i < 3; ++i) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = (k == i) ? 1.0 : 0.0;
            }
        }
    } else if (eq01 || eq12) {
        const int a = eq01 ? 0 : 1;
        const int other = eq01 ? 2 : 0;
        const double n[3] = {evec[0][other], evec[1][other], evec[2][other]};
        double bestLen = -1.0;
        double b1[3] = {0, 0, 0};
        for (int axis = 0; axis < 3; ++axis) {
            double p[3];
            for (int k = 0; k < 3; ++k) {
                p[k] = ((k == axis) ? 1.0 : 0.0) - n[k] * n[axis];
            }
            const double len =
                std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            if (len > bestLen) {
                bestLen = len;
                for (int k = 0; k < 3; ++k) {
                    b1[k] = p[k] / len;
                }
            }
        }
        double b2[3] = {n[1] * b1[2] - n[2] * b1[1],
                        n[2] * b1[0] - n[0] * b1[2],
                        n[0] * b1[1] - n[1] * b1[0]};
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(b2[k]) > std::abs(b2[maxK])) {
                maxK = k;
            }
        }
        if (b2[maxK] < 0) {
            for (int k = 0; k < 3; ++k) {
                b2[k] = -b2[k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            evec[k][a] = b1[k];
            evec[k][a + 1] = b2[k];
        }
    }
}

// The 3x3 orthonormalize both mirrors spell.
template <class Math>
void
Orthonormalize3(typename Math::Mat3 *m)
{
    typename Math::Vec3 r0 = Math::Vec3Of((*m)[0][0], (*m)[0][1], (*m)[0][2]);
    typename Math::Vec3 r1 = Math::Vec3Of((*m)[1][0], (*m)[1][1], (*m)[1][2]);
    typename Math::Vec3 r2 = Math::Vec3Of((*m)[2][0], (*m)[2][1], (*m)[2][2]);
    OrthogonalizeBasis3<Math>(&r0, &r1, &r2, true, 1e-6);
    (*m)[0][0] = r0[0];
    (*m)[0][1] = r0[1];
    (*m)[0][2] = r0[2];
    (*m)[1][0] = r1[0];
    (*m)[1][1] = r1[1];
    (*m)[1][2] = r1[2];
    (*m)[2][0] = r2[0];
    (*m)[2][1] = r2[1];
    (*m)[2][2] = r2[2];
}

// The 3x3 rotation from a unit quaternion.
template <class Math>
void
Mat3SetRotate(typename Math::Mat3 *m, double r, const typename Math::Vec3 &i)
{
    (*m)[0][0] = 1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2]);
    (*m)[0][1] = 2.0 * (i[0] * i[1] + i[2] * r);
    (*m)[0][2] = 2.0 * (i[2] * i[0] - i[1] * r);
    (*m)[1][0] = 2.0 * (i[0] * i[1] - i[2] * r);
    (*m)[1][1] = 1.0 - 2.0 * (i[2] * i[2] + i[0] * i[0]);
    (*m)[1][2] = 2.0 * (i[1] * i[2] + i[0] * r);
    (*m)[2][0] = 2.0 * (i[2] * i[0] + i[1] * r);
    (*m)[2][1] = 2.0 * (i[1] * i[2] - i[0] * r);
    (*m)[2][2] = 1.0 - 2.0 * (i[1] * i[1] + i[0] * i[0]);
}

// The 3x3 ExtractRotationQuaternion (including the int-typed 4 in
// the else arm), followed by the rotation round trip
// ExtractRotation().GetQuat() performs.
template <class Math>
typename Math::Quat
Mat3ExtractRotationQuat(const typename Math::Mat3 &m)
{
    int i = 0;
    if (m[0][0] > m[1][1]) {
        i = (m[0][0] > m[2][2] ? 0 : 2);
    } else {
        i = (m[1][1] > m[2][2] ? 1 : 2);
    }
    double im[3] = {0.0, 0.0, 0.0};
    double r = 0.0;
    if (m[0][0] + m[1][1] + m[2][2] > m[i][i]) {
        r = 0.5 * std::sqrt(m[0][0] + m[1][1] + m[2][2] + 1);
        im[0] = (m[1][2] - m[2][1]) / (4.0 * r);
        im[1] = (m[2][0] - m[0][2]) / (4.0 * r);
        im[2] = (m[0][1] - m[1][0]) / (4.0 * r);
    } else {
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const double q = 0.5 * std::sqrt(m[i][i] - m[j][j] - m[k][k] + 1);
        im[i] = q;
        im[j] = (m[i][j] + m[j][i]) / (4 * q);
        im[k] = (m[k][i] + m[i][k]) / (4 * q);
        r = (m[j][k] - m[k][j]) / (4 * q);
    }
    // SetQuat then GetQuat: the axis/angle round trip.
    const double clamped = Clamp(r, -1.0, 1.0);
    typename Math::Vec3 imv = Math::Vec3Of(im[0], im[1], im[2]);
    const double len = Length3<Math>(imv);
    typename Math::Vec3 axis = Math::Vec3Of(1.0, 0.0, 0.0);
    double angle = 0.0;
    if (len > 1e-10) {
        const double x = std::acos(Clamp(clamped, -1.0, 1.0));
        const double inv = 1.0 / len;
        axis = Scale3<Math>(imv, inv);
        constexpr double pi = 3.141592653589793;
        angle = 2.0 * (x * (180.0 / pi));
    }
    return QuatFromAxisAngle<Math>(axis, angle);
}

// SRT substrate (pointFrame.cpp): translation, rotation, scale,
// shear and the reflection axis.
template <class Math> struct PoseTransformParamsT {
    typename Math::Vec3 translation;
    typename Math::Quat rotation;
    typename Math::Vec3 scale;
    typename Math::Vec3 shear;
    int reflectionAxis = 2;
};

// PointsToParams: the affine map rest->pose decomposed into SRT via
// SVD. False for a singular reference or posed frame.
template <class Math>
bool
PointsToParams(const std::array<typename Math::Vec3, 4> &restPoints,
               const std::array<typename Math::Vec3, 4> &posePoints,
               int reflectionAxis, pose_ops::PoseTransformParamsT<Math> *params)
{
    typename Math::Mat4 m = Math::Identity4();
    if (!PointsToMatrix<Math>(restPoints, posePoints, &m)) {
        return false;
    }
    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            L[i][j] = m[j][i];
        }
    }
    double ltl[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += L[k][i] * L[k][j];
            }
            ltl[i][j] = sum;
        }
    }
    double s2[3];
    double V[3][3];
    SymmetricEigen3(ltl, s2, V);
    double sigma[3];
    for (int i = 0; i < 3; ++i) {
        sigma[i] = std::sqrt(std::max(s2[i], 0.0));
    }
    const double detL = L[0][0] * (L[1][1] * L[2][2] - L[1][2] * L[2][1]) -
                        L[0][1] * (L[1][0] * L[2][2] - L[1][2] * L[2][0]) +
                        L[0][2] * (L[1][0] * L[2][1] - L[1][1] * L[2][0]);
    if (sigma[2] < 1e-14 * std::max(1.0, sigma[0])) {
        return false;
    }
    double U[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += L[i][k] * V[k][j];
            }
            U[i][j] = sum / sigma[j];
        }
    }
    const double delta = (detL >= 0) ? 1.0 : -1.0;
    int k = 2;
    if (delta < 0) {
        const int axis = reflectionAxis;
        double best = -1.0;
        for (int i = 0; i < 3; ++i) {
            const double align = std::abs(V[axis][i]);
            if (align > best) {
                best = align;
                k = i;
            }
        }
    }
    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                const double dkk = (kk == k) ? delta : 1.0;
                sum += U[i][kk] * dkk * V[j][kk];
            }
            R[i][j] = sum;
        }
    }
    double H[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                sum += R[kk][i] * L[kk][j];
            }
            H[i][j] = sum;
        }
    }
    typename Math::Mat3 rowR =
        Math::Mat3FromElements(R[0][0], R[1][0], R[2][0], R[0][1], R[1][1],
                               R[2][1], R[0][2], R[1][2], R[2][2]);
    Orthonormalize3<Math>(&rowR);
    const typename Math::Quat q = Mat3ExtractRotationQuat<Math>(rowR);
    params->translation = Math::Vec3Of(m[3][0], m[3][1], m[3][2]);
    params->rotation = QuatNormalized<Math>(q, 1e-10);
    params->scale = Math::Vec3Of(H[0][0], H[1][1], H[2][2]);
    params->shear =
        Math::Vec3Of(H[0][1] / H[0][0], H[0][2] / H[0][0], H[1][2] / H[1][1]);
    params->reflectionAxis = reflectionAxis;
    return true;
}

// ParamsToMatrix: L = R(q) H(s,h), t.
template <class Math>
typename Math::Mat4
ParamsToMatrix(const pose_ops::PoseTransformParamsT<Math> &params)
{
    typename Math::Mat3 rowR = Math::Identity3();
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(params.rotation, &w, &im);
    Mat3SetRotate<Math>(&rowR, w, im);
    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = rowR[j][i];
        }
    }
    const typename Math::Vec3 &sc = params.scale;
    const typename Math::Vec3 &sh = params.shear;
    const double H[3][3] = {
        {sc[0], sh[0] * sc[0], sh[1] * sc[0]},
        {sh[0] * sc[0], sc[1], sh[2] * sc[1]},
        {sh[1] * sc[0], sh[2] * sc[1], sc[2]},
    };
    double L[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) {
                sum += R[i][k] * H[k][j];
            }
            L[i][j] = sum;
        }
    }
    return Math::Mat4FromElements(
        L[0][0], L[1][0], L[2][0], 0.0, L[0][1], L[1][1], L[2][1], 0.0, L[0][2],
        L[1][2], L[2][2], 0.0, params.translation[0], params.translation[1],
        params.translation[2], 1.0);
}

} // namespace pose_ops

// The FBX constraint substrate both Aim/Position/Rotation/Scale/
// Parent solve through.
template <class Math>
bool FrameUsable(const std::array<typename Math::Vec3, 4> &points,
                 unsigned flags);

template <class Math> struct PoseSourceT;

namespace pose_constraints {

enum PoseEulerOrder {
    kPoseEulerXYZ = 0,
    kPoseEulerXZY,
    kPoseEulerYXZ,
    kPoseEulerYZX,
    kPoseEulerZXY,
    kPoseEulerZYX
};

struct PoseAxisMask {
    bool x = true;
    bool y = true;
    bool z = true;
};

template <class Math> struct PoseAimParamsT {
    double weight = 1.0;
    typename Math::Vec3 localAimVector;
    typename Math::Vec3 localUpVector;
    typename Math::Vec3 rotationOffsetDegrees;
    PoseAxisMask affectRotation;
    PoseEulerOrder rotationOrder = kPoseEulerXYZ;
    // The world-up direction, or nullptr. Borrowed; the caller keeps
    // the referent alive for the call.
    const typename Math::Vec3 *worldUpDirection = nullptr;
    bool preserveInputUp = false;
};

template <class Math>
bool
VecFinite(const typename Math::Vec3 &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

template <class Math>
bool
HasFinitePoints(const std::array<typename Math::Vec3, 4> &points)
{
    for (const auto &point : points) {
        if (!VecFinite<Math>(point)) {
            return false;
        }
    }
    return true;
}

template <class Math>
bool
FrameUsableRotation(const typename Math::Quat &q)
{
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(q, &w, &im);
    return std::isfinite(w) && VecFinite<Math>(im);
}

template <class Math>
bool
DecomposeConstraintFrame(const std::array<typename Math::Vec3, 4> &points,
                         unsigned flags,
                         pose_ops::PoseTransformParamsT<Math> *params)
{
    if (!FrameUsable<Math>(points, flags)) {
        return false;
    }
    if (!pose_ops::PointsToParams<Math>(pose_ops::IdentityLandmarks<Math>(),
                                        points, 2, params)) {
        return false;
    }
    return VecFinite<Math>(params->translation) &&
           VecFinite<Math>(params->scale) && VecFinite<Math>(params->shear) &&
           FrameUsableRotation<Math>(params->rotation);
}

template <class Math>
void
FrameFromConstraintParams(const std::array<typename Math::Vec3, 4> &inputPoints,
                          unsigned inputFlags,
                          const pose_ops::PoseTransformParamsT<Math> &params,
                          std::array<typename Math::Vec3, 4> *points,
                          unsigned *flags)
{
    pose_ops::MatrixToPoints<Math>(pose_ops::IdentityLandmarks<Math>(),
                                   pose_ops::ParamsToMatrix<Math>(params),
                                   points, flags);
    if (!HasFinitePoints<Math>(*points)) {
        *points = inputPoints;
        *flags = inputFlags | pose_ops::kPoseFrameDegenerate;
        return;
    }
    *flags |= inputFlags & pose_ops::kPoseFrameAffine;
}

inline bool
IsValidOrder(PoseEulerOrder order)
{
    const int raw = int(order);
    return raw >= 0 && raw <= 5;
}

inline std::array<int, 3>
OrderIndices(PoseEulerOrder order)
{
    switch (order) {
    case kPoseEulerXYZ:
        return {0, 1, 2};
    case kPoseEulerXZY:
        return {0, 2, 1};
    case kPoseEulerYXZ:
        return {1, 0, 2};
    case kPoseEulerYZX:
        return {1, 2, 0};
    case kPoseEulerZXY:
        return {2, 0, 1};
    case kPoseEulerZYX:
        return {2, 1, 0};
    }
    return {0, 1, 2};
}

template <class Math>
typename Math::Quat
QuatFromEulerDegrees(const typename Math::Vec3 &degrees, PoseEulerOrder order)
{
    const typename Math::Vec3 axes[3] = {
        Math::Vec3Of(1, 0, 0), Math::Vec3Of(0, 1, 0), Math::Vec3Of(0, 0, 1)};
    typename Math::Mat4 matrix = Math::Identity4();
    const std::array<int, 3> indices = OrderIndices(order);
    for (const int axis : indices) {
        matrix = pose_ops::Multiply4<Math>(
            matrix, pose_ops::RotationMatrix<Math>(axes[axis], degrees[axis]));
    }
    // ExtractRotation().GetQuat().GetNormalized(): the mat4
    // quaternion through the axis/angle round trip, normalized.
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(pose_ops::ExtractRotationQuat<Math>(matrix), &w,
                           &im);
    const double len = pose_ops::Length3<Math>(im);
    typename Math::Vec3 axis = Math::Vec3Of(1.0, 0.0, 0.0);
    double angle = 0.0;
    if (len > 1e-10) {
        const double x = std::acos(pose_ops::Clamp(w, -1.0, 1.0));
        axis = pose_ops::Scale3<Math>(im, 1.0 / len);
        constexpr double pi = 3.141592653589793;
        angle = 2.0 * (x * (180.0 / pi));
    }
    return pose_ops::QuatNormalized<Math>(
        pose_ops::QuatFromAxisAngle<Math>(axis, angle), 1e-10);
}

template <class Math>
typename Math::Vec3
EulerDegreesFromQuat(const typename Math::Quat &rotation, PoseEulerOrder order)
{
    const typename Math::Vec3 axes[3] = {
        Math::Vec3Of(1, 0, 0), Math::Vec3Of(0, 1, 0), Math::Vec3Of(0, 0, 1)};
    const std::array<int, 3> indices = OrderIndices(order);
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(rotation, &w, &im);
    const double len = pose_ops::Length3<Math>(im);
    pose_ops::PoseRotationT<Math> asRotation;
    asRotation.axis = Math::Vec3Of(1.0, 0.0, 0.0);
    asRotation.angle = 0.0;
    if (len > 1e-10) {
        const double x = std::acos(pose_ops::Clamp(w, -1.0, 1.0));
        asRotation.axis = pose_ops::Scale3<Math>(im, 1.0 / len);
        constexpr double pi = 3.141592653589793;
        asRotation.angle = 2.0 * (x * (180.0 / pi));
    }
    const typename Math::Vec3 ordered = pose_ops::RotationDecompose<Math>(
        asRotation, axes[indices[2]], axes[indices[1]], axes[indices[0]]);
    typename Math::Vec3 result = Math::Vec3Of(0, 0, 0);
    result[indices[2]] = ordered[0];
    result[indices[1]] = ordered[1];
    result[indices[0]] = ordered[2];
    return result;
}

template <class Math>
bool
NormalizeDirection(typename Math::Vec3 *direction, double eps)
{
    if (!VecFinite<Math>(*direction)) {
        return false;
    }
    const double largest =
        std::max({std::abs((*direction)[0]), std::abs((*direction)[1]),
                  std::abs((*direction)[2])});
    if (largest < eps) {
        return false;
    }
    *direction = pose_ops::Scale3<Math>(*direction, 1.0 / largest);
    const double length = pose_ops::Length3<Math>(*direction);
    if (!std::isfinite(length) || length < eps) {
        return false;
    }
    *direction = pose_ops::Scale3<Math>(*direction, 1.0 / length);
    return VecFinite<Math>(*direction);
}

inline double
ShortestDegrees(double degrees)
{
    double wrapped = std::fmod(degrees + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    wrapped -= 180.0;
    return (wrapped == -180.0 && degrees > 0.0) ? 180.0 : wrapped;
}

template <class Math>
typename Math::Vec3
ApplyEulerDelta(const typename Math::Vec3 &inputEuler,
                const typename Math::Vec3 &targetEuler,
                const PoseAxisMask &affect, double weight)
{
    typename Math::Vec3 output = inputEuler;
    for (int axis = 0; axis < 3; ++axis) {
        const bool on = axis == 0 ? affect.x : axis == 1 ? affect.y : affect.z;
        if (on) {
            if (weight >= 1.0) {
                output[axis] = targetEuler[axis];
            } else {
                output[axis] += weight * ShortestDegrees(targetEuler[axis] -
                                                         inputEuler[axis]);
            }
        }
    }
    return output;
}

template <class Math>
void
BlendGovernedShear(const PoseAxisMask &scaleMask,
                   const typename Math::Vec3 &targetShear, double weight,
                   pose_ops::PoseTransformParamsT<Math> *params)
{
    if (!scaleMask.x || !scaleMask.y || !scaleMask.z) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        params->shear[i] =
            pose_ops::BlendEnvelope(params->shear[i], targetShear[i], weight);
    }
}

// Aim: swing the input's aim axis onto the target point's, correct
// the roll, and blend the euler delta by the envelope.
template <class Math>
void
ApplyAimConstraint(const std::array<typename Math::Vec3, 4> &inputPoints,
                   unsigned inputFlags, const typename Math::Vec3 &targetPoint,
                   const PoseAimParamsT<Math> &params,
                   std::array<typename Math::Vec3, 4> *points, unsigned *flags)
{
    const auto fail = [&]() {
        *points = inputPoints;
        *flags = inputFlags | pose_ops::kPoseFrameDegenerate;
    };
    double globalWeight = 0.0;
    if (!std::isfinite(params.weight)) {
        fail();
        return;
    }
    globalWeight = std::min(std::max(params.weight, 0.0), 1.0);
    if (globalWeight <= 0.0) {
        *points = inputPoints;
        *flags = inputFlags;
        return;
    }
    const bool needsRollCorrection =
        params.worldUpDirection != nullptr || params.preserveInputUp;
    if (!IsValidOrder(params.rotationOrder) || !VecFinite<Math>(targetPoint) ||
        !VecFinite<Math>(params.localAimVector) ||
        (needsRollCorrection && !VecFinite<Math>(params.localUpVector)) ||
        !VecFinite<Math>(params.rotationOffsetDegrees) ||
        (params.worldUpDirection &&
         !VecFinite<Math>(*params.worldUpDirection))) {
        fail();
        return;
    }
    pose_ops::PoseTransformParamsT<Math> inputParams;
    if (!DecomposeConstraintFrame<Math>(inputPoints, inputFlags,
                                        &inputParams)) {
        fail();
        return;
    }
    typename Math::Vec3 localAim = params.localAimVector;
    if (!NormalizeDirection<Math>(&localAim, 1e-12)) {
        fail();
        return;
    }
    typename Math::Vec3 targetAim =
        pose_ops::Sub3<Math>(targetPoint, inputParams.translation);
    if (!NormalizeDirection<Math>(&targetAim, 1e-12)) {
        fail();
        return;
    }
    const typename Math::Quat inputRotation =
        pose_ops::QuatNormalized<Math>(inputParams.rotation, 1e-10);
    typename Math::Vec3 currentAim =
        pose_ops::QuatTransform<Math>(inputRotation, localAim);
    if (!NormalizeDirection<Math>(&currentAim, 1e-12)) {
        fail();
        return;
    }
    const pose_ops::PoseRotationT<Math> swing =
        pose_ops::RotateInto<Math>(currentAim, targetAim);
    pose_ops::PoseRotationT<Math> twist;
    twist.axis = Math::Vec3Of(1, 0, 0);
    twist.angle = 0.0;
    if (needsRollCorrection) {
        typename Math::Vec3 localUp = params.localUpVector;
        if (!NormalizeDirection<Math>(&localUp, 1e-12)) {
            fail();
            return;
        }
        localUp = pose_ops::Sub3<Math>(
            localUp, pose_ops::Scale3<Math>(
                         localAim, pose_ops::Dot3<Math>(localAim, localUp)));
        if (!NormalizeDirection<Math>(&localUp, 1e-12)) {
            fail();
            return;
        }
        typename Math::Vec3 currentUp =
            pose_ops::QuatTransform<Math>(inputRotation, localUp);
        if (!NormalizeDirection<Math>(&currentUp, 1e-12)) {
            fail();
            return;
        }
        typename Math::Vec3 swungUp =
            pose_ops::RotationTransformDir<Math>(swing, currentUp);
        swungUp = pose_ops::Sub3<Math>(
            swungUp, pose_ops::Scale3<Math>(
                         targetAim, pose_ops::Dot3<Math>(targetAim, swungUp)));
        if (!NormalizeDirection<Math>(&swungUp, 1e-12)) {
            fail();
            return;
        }
        typename Math::Vec3 desiredUp = Math::Vec3Of(0, 0, 0);
        if (params.worldUpDirection) {
            desiredUp = *params.worldUpDirection;
            if (!NormalizeDirection<Math>(&desiredUp, 1e-12)) {
                fail();
                return;
            }
            desiredUp = pose_ops::Sub3<Math>(
                desiredUp,
                pose_ops::Scale3<Math>(
                    targetAim, pose_ops::Dot3<Math>(targetAim, desiredUp)));
            if (!NormalizeDirection<Math>(&desiredUp, 1e-12)) {
                fail();
                return;
            }
        } else {
            desiredUp = pose_ops::Sub3<Math>(
                currentUp,
                pose_ops::Scale3<Math>(
                    targetAim, pose_ops::Dot3<Math>(targetAim, currentUp)));
            if (!NormalizeDirection<Math>(&desiredUp, 1e-12)) {
                desiredUp = swungUp;
            }
        }
        twist =
            pose_ops::RotateOntoProjected<Math>(swungUp, desiredUp, targetAim);
    }
    typename Math::Mat4 aimedMatrix = Math::Identity4();
    {
        double w = 0.0;
        typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
        Math::QuatToComponents(inputRotation, &w, &im);
        pose_ops::SetRotateFromQuat<Math>(w, im, &aimedMatrix);
    }
    const typename Math::Mat4 swingMatrix =
        pose_ops::RotationMatrix<Math>(swing.axis, swing.angle);
    const typename Math::Mat4 twistMatrix =
        pose_ops::RotationMatrix<Math>(twist.axis, twist.angle);
    aimedMatrix = pose_ops::Multiply4<Math>(
        pose_ops::Multiply4<Math>(aimedMatrix, swingMatrix), twistMatrix);
    // ExtractRotation().GetQuat().GetNormalized(): the quaternion
    // through the axis/angle round trip, normalized.
    double aimedW = 0.0;
    typename Math::Vec3 aimedIm = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(pose_ops::ExtractRotationQuat<Math>(aimedMatrix),
                           &aimedW, &aimedIm);
    const double aimedLen = pose_ops::Length3<Math>(aimedIm);
    typename Math::Vec3 aimedAxis = Math::Vec3Of(1.0, 0.0, 0.0);
    double aimedAngle = 0.0;
    if (aimedLen > 1e-10) {
        const double x = std::acos(pose_ops::Clamp(aimedW, -1.0, 1.0));
        aimedAxis = pose_ops::Scale3<Math>(aimedIm, 1.0 / aimedLen);
        constexpr double pi = 3.141592653589793;
        aimedAngle = 2.0 * (x * (180.0 / pi));
    }
    const typename Math::Quat aimedRotation = pose_ops::QuatNormalized<Math>(
        pose_ops::QuatFromAxisAngle<Math>(aimedAxis, aimedAngle), 1e-10);
    const typename Math::Vec3 inputEuler =
        EulerDegreesFromQuat<Math>(inputRotation, params.rotationOrder);
    const typename Math::Vec3 targetEuler = pose_ops::Add3<Math>(
        EulerDegreesFromQuat<Math>(aimedRotation, params.rotationOrder),
        params.rotationOffsetDegrees);
    if (!VecFinite<Math>(inputEuler) || !VecFinite<Math>(targetEuler)) {
        fail();
        return;
    }
    const typename Math::Vec3 outputEuler = ApplyEulerDelta<Math>(
        inputEuler, targetEuler, params.affectRotation, globalWeight);
    inputParams.rotation =
        QuatFromEulerDegrees<Math>(outputEuler, params.rotationOrder);
    FrameFromConstraintParams<Math>(inputPoints, inputFlags, inputParams,
                                    points, flags);
}

// The source-constraint substrate: weight validation, axis masks,
// and frame transforms shared by the four FBX applies below.
inline bool
ValidateGlobalWeight(double weight, double *clamped)
{
    if (!std::isfinite(weight)) {
        return false;
    }
    *clamped = std::min(std::max(weight, 0.0), 1.0);
    return true;
}

inline bool
Affects(const PoseAxisMask &mask, int axis)
{
    return axis == 0 ? mask.x : axis == 1 ? mask.y : mask.z;
}

template <class Math>
void
TransformFramePoints(const std::array<typename Math::Vec3, 4> &points,
                     unsigned flags, const typename Math::Mat4 &space,
                     std::array<typename Math::Vec3, 4> *outPoints,
                     unsigned *outFlags)
{
    for (size_t i = 0; i < 4; ++i) {
        (*outPoints)[i] = pose_ops::Transform3<Math>(space, points[i]);
    }
    *outFlags = flags;
}

template <class Math> struct PosePositionParamsT {
    double weight = 1.0;
    typename Math::Vec3 offset;
    PoseAxisMask affect;
};

template <class Math>
void
ApplyPositionConstraint(const std::array<typename Math::Vec3, 4> &inputPoints,
                        unsigned inputFlags,
                        const std::vector<PoseSourceT<Math>> &sources,
                        const PosePositionParamsT<Math> &params,
                        std::array<typename Math::Vec3, 4> *points,
                        unsigned *flags)
{
    const auto fail = [&]() {
        *points = inputPoints;
        *flags = inputFlags | pose_ops::kPoseFrameDegenerate;
    };
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        fail();
        return;
    }
    if (globalWeight <= 0.0) {
        *points = inputPoints;
        *flags = inputFlags;
        return;
    }
    if (!FrameUsable<Math>(inputPoints, inputFlags) ||
        !VecFinite<Math>(params.offset)) {
        fail();
        return;
    }
    typename Math::Vec3 target = Math::Vec3Of(0, 0, 0);
    double totalWeight = 0.0;
    for (const PoseSourceT<Math> &source : sources) {
        if (!std::isfinite(source.normalizedWeight) ||
            source.normalizedWeight < 0) {
            fail();
            return;
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!FrameUsable<Math>(source.points, source.flags)) {
            fail();
            return;
        }
        target = pose_ops::Add3<Math>(
            target,
            pose_ops::Scale3<Math>(source.points[0], source.normalizedWeight));
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        if (totalWeight == 0.0) {
            *points = inputPoints;
            *flags = inputFlags;
        } else {
            fail();
        }
        return;
    }
    target = pose_ops::Add3<Math>(
        pose_ops::Scale3<Math>(target, 1.0 / totalWeight), params.offset);
    if (!VecFinite<Math>(target)) {
        fail();
        return;
    }
    typename Math::Vec3 constrainedOrigin = inputPoints[0];
    for (int axis = 0; axis < 3; ++axis) {
        if (Affects(params.affect, axis)) {
            constrainedOrigin[axis] = pose_ops::BlendEnvelope(
                inputPoints[0][axis], target[axis], globalWeight);
        }
    }
    if (!VecFinite<Math>(constrainedOrigin)) {
        fail();
        return;
    }
    *points = inputPoints;
    if (globalWeight >= 1.0) {
        for (size_t point = 1; point < 4; ++point) {
            (*points)[point] = pose_ops::Add3<Math>(
                constrainedOrigin,
                pose_ops::Sub3<Math>((*points)[point], inputPoints[0]));
        }
        (*points)[0] = constrainedOrigin;
    } else {
        const typename Math::Vec3 translation =
            pose_ops::Sub3<Math>(constrainedOrigin, inputPoints[0]);
        for (size_t point = 0; point < 4; ++point) {
            (*points)[point] =
                pose_ops::Add3<Math>((*points)[point], translation);
        }
    }
    *flags = inputFlags;
}

template <class Math> struct PoseRotationParamsT {
    double weight = 1.0;
    typename Math::Vec3 offsetDegrees;
    PoseAxisMask affect;
    PoseEulerOrder rotationOrder = kPoseEulerXYZ;
    const typename Math::Mat4 *carry = nullptr;
};

template <class Math>
void
ApplyRotationConstraint(const std::array<typename Math::Vec3, 4> &inputPoints,
                        unsigned inputFlags,
                        const std::vector<PoseSourceT<Math>> &sources,
                        const PoseRotationParamsT<Math> &params,
                        std::array<typename Math::Vec3, 4> *points,
                        unsigned *flags)
{
    const auto fail = [&]() {
        *points = inputPoints;
        *flags = inputFlags | pose_ops::kPoseFrameDegenerate;
    };
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        fail();
        return;
    }
    if (globalWeight <= 0.0) {
        *points = inputPoints;
        *flags = inputFlags;
        return;
    }
    if (!IsValidOrder(params.rotationOrder) ||
        !VecFinite<Math>(params.offsetDegrees)) {
        fail();
        return;
    }
    typename Math::Mat4 carryInverse = Math::Identity4();
    std::array<typename Math::Vec3, 4> measured = inputPoints;
    unsigned measuredFlags = inputFlags;
    const std::array<typename Math::Vec3, 4> *in = &inputPoints;
    unsigned inFlags = inputFlags;
    if (params.carry) {
        pose_ops::Invert4<Math>(*params.carry, 0.0, &carryInverse, nullptr);
        TransformFramePoints<Math>(inputPoints, inputFlags, carryInverse,
                                   &measured, &measuredFlags);
        in = &measured;
        inFlags = measuredFlags;
    }
    pose_ops::PoseTransformParamsT<Math> inputParams;
    if (!DecomposeConstraintFrame<Math>(*in, inFlags, &inputParams)) {
        fail();
        return;
    }
    const typename Math::Vec3 inputEuler =
        EulerDegreesFromQuat<Math>(inputParams.rotation, params.rotationOrder);
    if (!VecFinite<Math>(inputEuler)) {
        fail();
        return;
    }
    typename Math::Vec3 sourceAnchor = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 weightedDelta = Math::Vec3Of(0, 0, 0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const PoseSourceT<Math> &source : sources) {
        if (!std::isfinite(source.normalizedWeight) ||
            source.normalizedWeight < 0) {
            fail();
            return;
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        pose_ops::PoseTransformParamsT<Math> sourceParams;
        std::array<typename Math::Vec3, 4> sourceMeasured = source.points;
        unsigned sourceMeasuredFlags = source.flags;
        if (params.carry) {
            TransformFramePoints<Math>(source.points, source.flags,
                                       carryInverse, &sourceMeasured,
                                       &sourceMeasuredFlags);
        }
        if (!DecomposeConstraintFrame<Math>(sourceMeasured, sourceMeasuredFlags,
                                            &sourceParams)) {
            fail();
            return;
        }
        const typename Math::Vec3 sourceEuler = EulerDegreesFromQuat<Math>(
            sourceParams.rotation, params.rotationOrder);
        if (!VecFinite<Math>(sourceEuler)) {
            fail();
            return;
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedDelta[axis] +=
                source.normalizedWeight *
                ShortestDegrees(sourceEuler[axis] - sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        if (totalWeight == 0.0) {
            *points = inputPoints;
            *flags = inputFlags;
        } else {
            fail();
        }
        return;
    }
    const typename Math::Vec3 targetEuler = pose_ops::Add3<Math>(
        pose_ops::Add3<Math>(
            sourceAnchor,
            pose_ops::Scale3<Math>(weightedDelta, 1.0 / totalWeight)),
        params.offsetDegrees);
    if (!VecFinite<Math>(weightedDelta) || !VecFinite<Math>(targetEuler)) {
        fail();
        return;
    }
    const typename Math::Vec3 outputEuler = ApplyEulerDelta<Math>(
        inputEuler, targetEuler, params.affect, globalWeight);
    inputParams.rotation =
        QuatFromEulerDegrees<Math>(outputEuler, params.rotationOrder);
    std::array<typename Math::Vec3, 4> solved = inputPoints;
    unsigned solvedFlags = inputFlags;
    FrameFromConstraintParams<Math>(inputPoints, inputFlags, inputParams,
                                    &solved, &solvedFlags);
    if (!params.carry || (solvedFlags & pose_ops::kPoseFrameDegenerate)) {
        *points = solved;
        *flags = solvedFlags;
        return;
    }
    TransformFramePoints<Math>(solved, solvedFlags, *params.carry, points,
                               flags);
}

template <class Math> struct PoseScaleParamsT {
    double weight = 1.0;
    typename Math::Vec3 offset;
    PoseAxisMask affect;
    bool blendShear = false;
};

template <class Math>
void
ApplyScaleConstraint(const std::array<typename Math::Vec3, 4> &inputPoints,
                     unsigned inputFlags,
                     const std::vector<PoseSourceT<Math>> &sources,
                     const PoseScaleParamsT<Math> &params,
                     std::array<typename Math::Vec3, 4> *points,
                     unsigned *flags)
{
    const auto fail = [&]() {
        *points = inputPoints;
        *flags = inputFlags | pose_ops::kPoseFrameDegenerate;
    };
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        fail();
        return;
    }
    if (globalWeight <= 0.0) {
        *points = inputPoints;
        *flags = inputFlags;
        return;
    }
    if (!VecFinite<Math>(params.offset)) {
        fail();
        return;
    }
    pose_ops::PoseTransformParamsT<Math> inputParams;
    if (!DecomposeConstraintFrame<Math>(inputPoints, inputFlags,
                                        &inputParams)) {
        fail();
        return;
    }
    typename Math::Vec3 targetScale = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 targetShear = Math::Vec3Of(0, 0, 0);
    double totalWeight = 0.0;
    for (const PoseSourceT<Math> &source : sources) {
        if (!std::isfinite(source.normalizedWeight) ||
            source.normalizedWeight < 0) {
            fail();
            return;
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        pose_ops::PoseTransformParamsT<Math> sourceParams;
        if (!DecomposeConstraintFrame<Math>(source.points, source.flags,
                                            &sourceParams)) {
            fail();
            return;
        }
        targetScale = pose_ops::Add3<Math>(
            targetScale, pose_ops::Scale3<Math>(sourceParams.scale,
                                                source.normalizedWeight));
        if (params.blendShear) {
            targetShear = pose_ops::Add3<Math>(
                targetShear, pose_ops::Scale3<Math>(sourceParams.shear,
                                                    source.normalizedWeight));
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        if (totalWeight == 0.0) {
            *points = inputPoints;
            *flags = inputFlags;
        } else {
            fail();
        }
        return;
    }
    targetScale = pose_ops::Add3<Math>(
        pose_ops::Scale3<Math>(targetScale, 1.0 / totalWeight), params.offset);
    if (!VecFinite<Math>(targetScale)) {
        fail();
        return;
    }
    if (params.blendShear) {
        targetShear = pose_ops::Scale3<Math>(targetShear, 1.0 / totalWeight);
        if (!VecFinite<Math>(targetShear)) {
            fail();
            return;
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (Affects(params.affect, axis)) {
            inputParams.scale[axis] = pose_ops::BlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    if (params.blendShear) {
        BlendGovernedShear<Math>(params.affect, targetShear, globalWeight,
                                 &inputParams);
    }
    FrameFromConstraintParams<Math>(inputPoints, inputFlags, inputParams,
                                    points, flags);
}

template <class Math> struct PoseParentParamsT {
    double weight = 1.0;
    PoseAxisMask translationAxes;
    PoseAxisMask rotationAxes;
    PoseAxisMask scaleAxes;
    PoseEulerOrder rotationOrder = kPoseEulerXYZ;
    const typename Math::Mat4 *carry = nullptr;
    bool blendShear = false;
};

template <class Math>
void
ApplyParentConstraint(const std::array<typename Math::Vec3, 4> &inputPoints,
                      unsigned inputFlags,
                      const std::vector<PoseSourceT<Math>> &sources,
                      const PoseParentParamsT<Math> &params,
                      std::array<typename Math::Vec3, 4> *points,
                      unsigned *flags)
{
    const auto fail = [&]() {
        *points = inputPoints;
        *flags = inputFlags | pose_ops::kPoseFrameDegenerate;
    };
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        fail();
        return;
    }
    if (globalWeight <= 0.0) {
        *points = inputPoints;
        *flags = inputFlags;
        return;
    }
    if (!IsValidOrder(params.rotationOrder)) {
        fail();
        return;
    }
    typename Math::Mat4 carryInverse = Math::Identity4();
    std::array<typename Math::Vec3, 4> measured = inputPoints;
    unsigned measuredFlags = inputFlags;
    const std::array<typename Math::Vec3, 4> *in = &inputPoints;
    unsigned inFlags = inputFlags;
    if (params.carry) {
        pose_ops::Invert4<Math>(*params.carry, 0.0, &carryInverse, nullptr);
        TransformFramePoints<Math>(inputPoints, inputFlags, carryInverse,
                                   &measured, &measuredFlags);
        in = &measured;
        inFlags = measuredFlags;
    }
    pose_ops::PoseTransformParamsT<Math> inputParams;
    if (!DecomposeConstraintFrame<Math>(*in, inFlags, &inputParams)) {
        fail();
        return;
    }
    const typename Math::Vec3 inputEuler =
        EulerDegreesFromQuat<Math>(inputParams.rotation, params.rotationOrder);
    if (!VecFinite<Math>(inputEuler)) {
        fail();
        return;
    }
    typename Math::Vec3 targetTranslation = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 targetScale = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 targetShear = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 sourceAnchor = Math::Vec3Of(0, 0, 0);
    typename Math::Vec3 weightedRotationDelta = Math::Vec3Of(0, 0, 0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const PoseSourceT<Math> &source : sources) {
        if (!std::isfinite(source.normalizedWeight) ||
            source.normalizedWeight < 0) {
            fail();
            return;
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!VecFinite<Math>(source.translationOffset) ||
            !VecFinite<Math>(source.rotationOffsetDegrees)) {
            fail();
            return;
        }
        std::array<typename Math::Vec3, 4> sourceMeasured = source.points;
        unsigned sourceMeasuredFlags = source.flags;
        if (params.carry) {
            TransformFramePoints<Math>(source.points, source.flags,
                                       carryInverse, &sourceMeasured,
                                       &sourceMeasuredFlags);
        }
        if (!FrameUsable<Math>(sourceMeasured, sourceMeasuredFlags)) {
            fail();
            return;
        }
        typename Math::Mat4 sourceMatrix = Math::Identity4();
        if (!pose_ops::PointsToMatrix<Math>(pose_ops::IdentityLandmarks<Math>(),
                                            sourceMeasured, &sourceMatrix)) {
            fail();
            return;
        }
        pose_ops::PoseTransformParamsT<Math> offsetParams;
        offsetParams.translation = source.translationOffset;
        offsetParams.rotation = QuatFromEulerDegrees<Math>(
            source.rotationOffsetDegrees, params.rotationOrder);
        offsetParams.scale = Math::Vec3Of(1, 1, 1);
        offsetParams.shear = Math::Vec3Of(0, 0, 0);
        const typename Math::Mat4 targetMatrix = pose_ops::Multiply4<Math>(
            pose_ops::ParamsToMatrix<Math>(offsetParams), sourceMatrix);
        std::array<typename Math::Vec3, 4> targetPoints;
        unsigned targetFlags = 0;
        pose_ops::MatrixToPoints<Math>(pose_ops::IdentityLandmarks<Math>(),
                                       targetMatrix, &targetPoints,
                                       &targetFlags);
        pose_ops::PoseTransformParamsT<Math> targetParams;
        if (!DecomposeConstraintFrame<Math>(targetPoints, targetFlags,
                                            &targetParams)) {
            fail();
            return;
        }
        targetTranslation = pose_ops::Add3<Math>(
            targetTranslation, pose_ops::Scale3<Math>(targetParams.translation,
                                                      source.normalizedWeight));
        targetScale = pose_ops::Add3<Math>(
            targetScale, pose_ops::Scale3<Math>(targetParams.scale,
                                                source.normalizedWeight));
        if (params.blendShear) {
            targetShear = pose_ops::Add3<Math>(
                targetShear, pose_ops::Scale3<Math>(targetParams.shear,
                                                    source.normalizedWeight));
        }
        const typename Math::Vec3 sourceEuler = EulerDegreesFromQuat<Math>(
            targetParams.rotation, params.rotationOrder);
        if (!VecFinite<Math>(sourceEuler)) {
            fail();
            return;
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedRotationDelta[axis] +=
                source.normalizedWeight *
                ShortestDegrees(sourceEuler[axis] - sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        if (totalWeight == 0.0) {
            *points = inputPoints;
            *flags = inputFlags;
        } else {
            fail();
        }
        return;
    }
    targetTranslation =
        pose_ops::Scale3<Math>(targetTranslation, 1.0 / totalWeight);
    targetScale = pose_ops::Scale3<Math>(targetScale, 1.0 / totalWeight);
    if (params.blendShear) {
        targetShear = pose_ops::Scale3<Math>(targetShear, 1.0 / totalWeight);
    }
    const typename Math::Vec3 targetEuler = pose_ops::Add3<Math>(
        sourceAnchor,
        pose_ops::Scale3<Math>(weightedRotationDelta, 1.0 / totalWeight));
    if (!VecFinite<Math>(targetTranslation) || !VecFinite<Math>(targetScale) ||
        (params.blendShear && !VecFinite<Math>(targetShear)) ||
        !VecFinite<Math>(weightedRotationDelta) ||
        !VecFinite<Math>(targetEuler)) {
        fail();
        return;
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (Affects(params.translationAxes, axis)) {
            inputParams.translation[axis] =
                pose_ops::BlendEnvelope(inputParams.translation[axis],
                                        targetTranslation[axis], globalWeight);
        }
        if (Affects(params.scaleAxes, axis)) {
            inputParams.scale[axis] = pose_ops::BlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    if (params.blendShear) {
        BlendGovernedShear<Math>(params.scaleAxes, targetShear, globalWeight,
                                 &inputParams);
    }
    const typename Math::Vec3 outputEuler = ApplyEulerDelta<Math>(
        inputEuler, targetEuler, params.rotationAxes, globalWeight);
    inputParams.rotation =
        QuatFromEulerDegrees<Math>(outputEuler, params.rotationOrder);
    std::array<typename Math::Vec3, 4> solved = inputPoints;
    unsigned solvedFlags = inputFlags;
    FrameFromConstraintParams<Math>(inputPoints, inputFlags, inputParams,
                                    &solved, &solvedFlags);
    if (!params.carry || (solvedFlags & pose_ops::kPoseFrameDegenerate)) {
        *points = solved;
        *flags = solvedFlags;
        return;
    }
    TransformFramePoints<Math>(solved, solvedFlags, *params.carry, points,
                               flags);
}

} // namespace pose_constraints

// ---- AvarInputs ----
//
// One ordered write of bound avar values: entry k writes value k to
// slot k, in order, so a later binding for the same slot wins on both
// sides exactly as the baked varying-then-constant lists and the
// runtime's single range already order them.
template <class Host>
StepResult
RunAvarInputsStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    // A negative object, or one past every binding, binds nothing on
    // both sides (the ranges answer empty); the step succeeds with no
    // writes. The adapter clamps the count; there is no Invalid arm.
    const size_t count =
        step.object < 0 ? 0 : host.inputs().avarWriteCount(size_t(step.object));
    const size_t object = size_t(step.object);
    RigExecWriteAvarBindings(
        count, [&](size_t k) { return host.inputs().avarWriteSlot(object, k); },
        [&](size_t k) { return host.inputs().avarWriteValue(object, k); },
        [&](auto slot, double value) { host.state().writeAvar(slot, value); });
    return {Outcome::Applied, kNoStepDetail};
}

// ---- SpaceCheckpoint ----
//
// A slot's compose inputs as one borrowed view, shared by the
// checkpoint and compose bodies: the 11 avar doubles, the rotation
// order text and sign mask, and the ladder tables. Bodies compose
// the avars through pose_ops; adapters never hand a composed matrix
// built by a precompiled operator.
template <class Math> struct PoseSlotComposeT {
    const double *avars11 = nullptr;
    // Owned: the runtime resolves token text per call, so a view
    // would dangle. Short-string storage, never the heap.
    std::string rotOrder;
    unsigned rotationSign = 0;
    bool noScaleAvars = false;
    bool posedAuthored = false;
    const typename Math::Mat4 *posedAuthoredM = nullptr;
    const typename Math::Mat4 *posedD = nullptr;
    const typename Math::Mat4 *parentDinv = nullptr;
    const typename Math::Mat4 *parentSpaceM = nullptr;
    bool parentSpaceAuthored = false;
};

// Slot compose inputs as one matrix (_ComposeAvarsOf): units scale
// the translations, the sign mask the rotations and spin.
template <class Math>
typename Math::Mat4
ComposeSlotAvars(const PoseSlotComposeT<Math> &slot)
{
    const double *a = slot.avars11;
    const double units = a[10];
    const bool noScale = slot.noScaleAvars;
    const unsigned sign = slot.rotationSign;
    const double signX = RigExecRotationSignFromMask(sign, 0);
    return pose_ops::ComposeAvars<Math>(
        a[0] * units, a[1] * units, a[2] * units, noScale ? 1.0 : a[3],
        noScale ? 1.0 : a[4], noScale ? 1.0 : a[5], a[6] * signX,
        a[7] * RigExecRotationSignFromMask(sign, 1),
        a[8] * RigExecRotationSignFromMask(sign, 2), a[9] * signX,
        slot.rotOrder);
}

// The checkpoint's per-slot kernel input, filled from the slot view.
template <class Math>
RigExecSpaceCheckpointInputT<typename Math::Mat4>
CheckpointInputOf(const PoseSlotComposeT<Math> &slot)
{
    RigExecSpaceCheckpointInputT<typename Math::Mat4> input;
    input.avars = ComposeSlotAvars<Math>(slot);
    input.posedDefault = *slot.posedD;
    input.parentDefaultInverse = *slot.parentDinv;
    input.parentExpression = *slot.parentSpaceM;
    input.parentExpressionAuthored = slot.parentSpaceAuthored;
    input.posedAuthoredMatrix = *slot.posedAuthoredM;
    input.posedAuthored = slot.posedAuthored;
    return input;
}

// The unswitched compose both _ComposeUnswitched spell:
// ((avars * posedDefault) * parentDefaultInverse) * parent.
template <class Math>
typename Math::Mat4
ComposeUnswitched(const typename Math::Mat4 &avars,
                  const typename Math::Mat4 &posedDefault,
                  const typename Math::Mat4 &parentDefaultInverse,
                  const typename Math::Mat4 &parent)
{
    return pose_ops::Multiply4<Math>(
        pose_ops::Multiply4<Math>(
            pose_ops::Multiply4<Math>(avars, posedDefault),
            parentDefaultInverse),
        parent);
}

// The checkpoint round trip: the frame of the matrix, back through
// the identity landmarks, with the NaN sentinel when the frame is
// unusable or does not convert.
template <class Math>
typename Math::Mat4
RoundTripCheckpoint(const typename Math::Mat4 &matrix)
{
    std::array<typename Math::Vec3, 4> points = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned flags = 0;
    pose_ops::FrameFromMatrix<Math>(matrix, &points, &flags);
    typename Math::Mat4 result = Math::Identity4();
    if (!(flags & pose_ops::kPoseFrameValid) ||
        (flags & pose_ops::kPoseFrameDegenerate) ||
        !pose_ops::PointsToMatrix<Math>(pose_ops::IdentityLandmarks<Math>(),
                                        points, &result)) {
        result = Math::Identity4();
        result[3][0] = std::numeric_limits<double>::quiet_NaN();
    }
    return result;
}

// A checkpoint's switch frame: each recomposed slot, outermost first, over
// the one before it (\p anchor's posed matrix for the first), round-tripped.
template <class Math>
typename Math::Mat4
SpaceCheckpointMatrix(
    const RigExecSpaceCheckpointInputT<typename Math::Mat4> *inputs,
    size_t count, typename Math::Mat4 anchor)
{
    typename Math::Mat4 posed = anchor;
    for (size_t k = 0; k < count; ++k) {
        const auto &input = inputs[k];
        const typename Math::Mat4 matrix =
            input.posedAuthored
                ? input.posedAuthoredMatrix
                : ComposeUnswitched<Math>(input.avars, input.posedDefault,
                                          input.parentDefaultInverse,
                                          input.parentExpressionAuthored
                                              ? input.parentExpression
                                              : posed);
        posed = RoundTripCheckpoint<Math>(matrix);
    }
    return posed;
}

template <class Host>
StepResult
RunSpaceCheckpointStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    using Math = typename Host::Math;
    if (step.object < 0 ||
        size_t(step.object) >= host.inputs().checkpointObjectCount()) {
        return {Outcome::Invalid, kNoStepDetail};
    }
    const size_t object = size_t(step.object);
    const size_t count = host.inputs().checkpointRecomposeCount(object);
    auto &inputs = host.state().checkpointInputs(object);
    if (inputs.size() < count) {
        return {Outcome::Invalid, kNoStepDetail};
    }
    for (size_t k = 0; k < count; ++k) {
        PoseSlotComposeT<Math> view;
        if (!host.inputs().checkpointSlot(object, k, &view)) {
            return {Outcome::Invalid, kNoStepDetail};
        }
        inputs[k] = CheckpointInputOf<Math>(view);
    }
    const int anchor = host.inputs().checkpointAnchor(object);
    typename Math::Mat4 posed = Math::Identity4();
    if (anchor >= 0 && !host.inputs().posedMatrix(size_t(anchor), &posed)) {
        return {Outcome::Invalid, kNoStepDetail};
    }
    host.state().writeSwitchFrame(
        object, SpaceCheckpointMatrix<Math>(inputs.data(), count, posed));
    return {Outcome::Applied, kNoStepDetail};
}

// ---- RestCompose / LadderCompose ----
//
// The ladder channel ids the spaceValues table is indexed by. Both
// adapters resolve a channel to its ready provider matrix or its
// leaf fallback before the bodies see it.
enum PoseLadderChannel : int {
    kPoseLadderRestSpace = 0,
    kPoseLadderDefaultSpace = 1,
    kPoseLadderPosedSpace = 2,
    kPoseLadderParentSpace = 3,
    kPoseLadderParentDefaultSpace = 4,
    kPoseLadderAvarDefaultSpace = 5,
    kPoseLadderPosedDefaultSpace = 6
};

// The mutable last-values a tracking run compares against, borrowed
// from the adapter's tables. Rotation order compares through the
// adapter (token identity on one side, integer identity on the
// other); every other table compares here, bit for bit.
template <class Math> struct PoseLadderLastsT {
    typename Math::Mat4 *restM = nullptr;
    typename Math::Mat4 *selfD = nullptr;
    typename Math::Mat4 *parentDinv = nullptr;
    typename Math::Mat4 *posedAuthoredM = nullptr;
    typename Math::Mat4 *posedD = nullptr;
    typename Math::Mat4 *parentSpaceM = nullptr;
    char *posedAuthored = nullptr;
    char *parentSpaceAuthored = nullptr;
    unsigned char *rotationSign = nullptr;
};

// Elementwise exact matrix equality (no tolerance, no FMA).
template <class Math>
bool
SameBitsMat4(const typename Math::Mat4 &a, const typename Math::Mat4 &b)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (a[r][c] != b[r][c]) {
                return false;
            }
        }
    }
    return true;
}

// Bitwise matrix equality, as the move trackers key it: a NaN that
// stays NaN has not moved, and a flipped zero sign has.
template <class Math>
bool
BitEqualMat4(const typename Math::Mat4 &a, const typename Math::Mat4 &b)
{
    return std::memcmp(&a[0][0], &b[0][0], 16 * sizeof(double)) == 0;
}

// The unavailable-provider sentinel: identity with a NaN at [3][0].
template <class Math>
typename Math::Mat4
UnavailableSpace()
{
    typename Math::Mat4 value = Math::Identity4();
    value[3][0] = std::numeric_limits<double>::quiet_NaN();
    return value;
}

// The commit gate of the pose walk: valid, non-degenerate, and
// finite in every component.
template <class Math>
bool
FrameUsable(const std::array<typename Math::Vec3, 4> &points, unsigned flags)
{
    return RigExecFrameUsableT(points, (flags & pose_ops::kPoseFrameValid) != 0,
                               (flags & pose_ops::kPoseFrameDegenerate) != 0);
}

// One slot's rest compose: the rest avars against the rest space, the
// intervening space and the parent's rest round trip.
template <class Math> struct PoseRestComposeT {
    typename Math::Mat4 restM = Math::Identity4();
    std::array<typename Math::Vec3, 4> points = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned flags = 0;
    typename Math::Mat4 roundTrip = Math::Identity4();
};

template <class Math>
PoseRestComposeT<Math>
RestComposeSlot(const double *restAvars, const typename Math::Mat4 &restSpace,
                const typename Math::Mat4 &intervening,
                const typename Math::Mat4 &parentRest)
{
    typename Math::Mat4 rest = pose_ops::ComposeAvars<Math>(
        restAvars[0], restAvars[1], restAvars[2], 1, 1, 1, restAvars[3],
        restAvars[4], restAvars[5], 0, std::string_view(SchemaValueText::xyz));
    rest = pose_ops::Multiply4<Math>(rest, restSpace);
    pose_ops::Orthonormalize4<Math>(&rest);
    if (!SameBitsMat4<Math>(intervening, Math::Identity4())) {
        rest = pose_ops::Multiply4<Math>(rest, intervening);
    }
    PoseRestComposeT<Math> out;
    out.restM = pose_ops::Multiply4<Math>(rest, parentRest);
    pose_ops::FrameFromMatrix<Math>(out.restM, &out.points, &out.flags);
    out.roundTrip = pose_ops::RoundTripMatrix<Math>(out.restM);
    return out;
}

// One slot's ladder reads, each channel resolved to its provider value or
// leaf. authored[c] is channel c's authoritative state; the default
// channels (1/4/5/6) also count a ready provider value. The parent
// pointers are null for a slot without a parent.
template <class Math> struct PoseLadderInputT {
    typename Math::Mat4 spaces[7];
    bool authored[7] = {};
    typename Math::Mat4 parentSpaceLeaf;
    typename Math::Vec3 rotationSign;
    double defaultAvars[6] = {};
    typename Math::Mat4 ownRest;
    const typename Math::Mat4 *parentRest = nullptr;
    const typename Math::Mat4 *parentDefault = nullptr;
};

template <class Math> struct PoseLadderT {
    typename Math::Mat4 selfD, parentDinv, defaultRoundTrip, posedD;
    typename Math::Mat4 parentSpaceM, posed;
    bool posedAuthored = false;
    bool parentSpaceAuthored = false;
    unsigned char rotationSign = 0;
};

template <class Math>
PoseLadderT<Math>
LadderComposeSlot(const PoseLadderInputT<Math> &in)
{
    using Mat4 = typename Math::Mat4;
    const Mat4 identity = Math::Identity4();
    const auto authored = [&](int channel) {
        return in.authored[channel] ||
               !SameBitsMat4<Math>(in.spaces[channel], identity);
    };
    PoseLadderT<Math> out;
    out.posed = in.spaces[kPoseLadderPosedSpace];
    out.posedAuthored = authored(kPoseLadderPosedSpace);
    const Mat4 parentRest = in.parentRest ? *in.parentRest : identity;
    Mat4 parentDefault = identity;
    if (authored(kPoseLadderParentDefaultSpace)) {
        parentDefault = in.spaces[kPoseLadderParentDefaultSpace];
    } else if (in.parentDefault) {
        parentDefault = *in.parentDefault;
    }
    if (authored(kPoseLadderDefaultSpace)) {
        out.selfD = in.spaces[kPoseLadderDefaultSpace];
    } else {
        const double *d = in.defaultAvars;
        const Mat4 offset = pose_ops::ComposeAvars<Math>(
            d[0], d[1], d[2], 1, 1, 1, d[3], d[4], d[5], 0,
            std::string_view(SchemaValueText::xyz));
        Mat4 parentRestInv = identity;
        pose_ops::Invert4<Math>(parentRest, 0.0, &parentRestInv, nullptr);
        out.selfD = pose_ops::Multiply4<Math>(
            pose_ops::Multiply4<Math>(
                pose_ops::Multiply4<Math>(offset, in.ownRest), parentRestInv),
            parentDefault);
    }
    out.defaultRoundTrip = pose_ops::RoundTripMatrix<Math>(out.selfD);
    const Mat4 avarDefault = authored(kPoseLadderAvarDefaultSpace)
                                 ? in.spaces[kPoseLadderAvarDefaultSpace]
                                 : out.selfD;
    out.posedD = authored(kPoseLadderPosedDefaultSpace)
                     ? in.spaces[kPoseLadderPosedDefaultSpace]
                     : avarDefault;
    out.parentDinv = identity;
    pose_ops::Invert4<Math>(parentDefault, 0.0, &out.parentDinv, nullptr);
    out.parentSpaceM = in.spaces[kPoseLadderParentSpace];
    out.parentSpaceAuthored = in.authored[kPoseLadderParentSpace] ||
                              !SameBitsMat4<Math>(in.parentSpaceLeaf, identity);
    out.rotationSign = RigExecRotationSignMask(
        in.rotationSign[0], in.rotationSign[1], in.rotationSign[2]);
    return out;
}

template <class Host>
StepResult
RunRestComposeRange(Host &host, int begin, int end, bool trackMoves)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    using Math = typename Host::Math;
    const typename Math::Mat4 identity = Math::Identity4();
    for (int i = begin; i < end; ++i) {
        const size_t slot = size_t(i);
        if (!host.inputs().isPoseSlot(slot)) {
            continue;
        }
        host.state().beforeSlot(StepKind::RestCompose, begin, i);
        if (!host.inputs().providerActive(slot)) {
            host.state().writeRest(slot, UnavailableSpace<Math>(),
                                   pose_ops::IdentityLandmarks<Math>(), 0,
                                   UnavailableSpace<Math>());
            continue;
        }
        double restAvars[6];
        for (size_t k = 0; k < 6; ++k) {
            restAvars[k] = host.inputs().restAvar(slot, k);
        }
        const int parent = host.inputs().parent(slot);
        typename Math::Mat4 parentRest = identity;
        if (parent >= 0 &&
            !host.inputs().restRoundTrip(size_t(parent), &parentRest)) {
            return {Outcome::Invalid, pose_detail::kPoseDetailNoSlot};
        }
        const PoseRestComposeT<Math> composed = RestComposeSlot<Math>(
            restAvars, host.inputs().ladderSpace(slot, kPoseLadderRestSpace),
            host.inputs().interveningSpace(slot), parentRest);
        const typename Math::Mat4 &restM = composed.restM;
        host.state().writeRest(slot, restM, composed.points, composed.flags,
                               composed.roundTrip);
        if (trackMoves) {
            auto lasts = host.state().ladderLasts(slot);
            if (!BitEqualMat4<Math>(restM, *lasts.restM)) {
                *lasts.restM = restM;
                host.state().markRestChanged(slot);
            }
        }
    }
    return {Outcome::Applied, kNoStepDetail};
}

template <class Host>
StepResult
RunRestComposeStep(Host &host, const StepRef &step, bool trackMoves)
{
    if (step.object < 0 ||
        size_t(step.object) >= host.inputs().composeGroupCount()) {
        return {Outcome::Invalid, kNoStepDetail};
    }
    int begin = 0;
    int end = 0;
    host.inputs().composeGroupRange(size_t(step.object), &begin, &end);
    return RunRestComposeRange(host, begin, end, trackMoves);
}

// One slot's default-space ladder: the posed/default/parent matrices
// and the rotation order and sign the compose reads.
template <class Host>
StepResult
RunLadderComposeRange(Host &host, int begin, int end, bool trackMoves)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    using Math = typename Host::Math;
    const typename Math::Mat4 identity = Math::Identity4();
    for (int i = begin; i < end; ++i) {
        const size_t slot = size_t(i);
        if (!host.inputs().isPoseSlot(slot)) {
            continue;
        }
        host.state().beforeSlot(StepKind::LadderCompose, begin, i);
        if (!host.inputs().providerActive(slot)) {
            host.state().writeLadderUnavailable(slot);
            continue;
        }
        PoseLadderInputT<Math> in;
        in.spaces[kPoseLadderRestSpace] = identity;
        for (int channel = kPoseLadderDefaultSpace;
             channel <= kPoseLadderPosedDefaultSpace; ++channel) {
            const bool defaultChannel =
                channel == kPoseLadderDefaultSpace ||
                channel == kPoseLadderParentDefaultSpace ||
                channel == kPoseLadderAvarDefaultSpace ||
                channel == kPoseLadderPosedDefaultSpace;
            in.spaces[channel] = host.inputs().ladderSpace(slot, channel);
            in.authored[channel] =
                (defaultChannel &&
                 host.inputs().ladderChannelReady(slot, channel)) ||
                host.inputs().ladderAuthoritative(slot, channel);
        }
        in.parentSpaceLeaf = host.inputs().ladderParentSpaceLeaf(slot);
        in.rotationSign = host.inputs().ladderRotationSign(slot);
        for (size_t k = 0; k < 6; ++k) {
            in.defaultAvars[k] = host.inputs().defaultAvar(slot, k);
        }
        if (!host.inputs().restRoundTrip(slot, &in.ownRest)) {
            return {Outcome::Invalid, pose_detail::kPoseDetailNoSlot};
        }
        const int parent = host.inputs().parent(slot);
        typename Math::Mat4 parentRest = identity;
        typename Math::Mat4 parentDefault = identity;
        if (parent >= 0) {
            if (!host.inputs().restRoundTrip(size_t(parent), &parentRest) ||
                !host.inputs().defaultRoundTrip(size_t(parent),
                                                &parentDefault)) {
                return {Outcome::Invalid, pose_detail::kPoseDetailNoSlot};
            }
            in.parentRest = &parentRest;
            in.parentDefault = &parentDefault;
        }
        const PoseLadderT<Math> ladder = LadderComposeSlot<Math>(in);
        const typename Math::Mat4 &selfD = ladder.selfD;
        const typename Math::Mat4 &parentDinv = ladder.parentDinv;
        const typename Math::Mat4 &defaultRoundTrip = ladder.defaultRoundTrip;
        const typename Math::Mat4 &posedD = ladder.posedD;
        const typename Math::Mat4 &parentSpaceM = ladder.parentSpaceM;
        const typename Math::Mat4 &posed = ladder.posed;
        const bool posedAuthored = ladder.posedAuthored;
        const bool parentSpaceAuthored = ladder.parentSpaceAuthored;
        const unsigned char rotationSign = ladder.rotationSign;
        host.state().writeLadder(slot, selfD, parentDinv, defaultRoundTrip,
                                 posedD, parentSpaceM, posed, posedAuthored,
                                 parentSpaceAuthored, rotationSign);
        host.state().writeRotOrder(slot);
        if (trackMoves) {
            auto lasts = host.state().ladderLasts(slot);
            if (!BitEqualMat4<Math>(selfD, *lasts.selfD) ||
                !BitEqualMat4<Math>(parentDinv, *lasts.parentDinv) ||
                (posedAuthored ? 1 : 0) != *lasts.posedAuthored ||
                !BitEqualMat4<Math>(posed, *lasts.posedAuthoredM) ||
                host.state().rotOrderMoved(slot) ||
                !BitEqualMat4<Math>(posedD, *lasts.posedD) ||
                !BitEqualMat4<Math>(parentSpaceM, *lasts.parentSpaceM) ||
                (parentSpaceAuthored ? 1 : 0) != *lasts.parentSpaceAuthored ||
                rotationSign != *lasts.rotationSign) {
                *lasts.selfD = selfD;
                *lasts.parentDinv = parentDinv;
                *lasts.posedAuthored = posedAuthored ? 1 : 0;
                *lasts.posedAuthoredM = posed;
                host.state().commitRotOrder(slot);
                *lasts.posedD = posedD;
                *lasts.parentSpaceM = parentSpaceM;
                *lasts.parentSpaceAuthored = parentSpaceAuthored ? 1 : 0;
                *lasts.rotationSign = rotationSign;
                host.state().markLadderChanged(slot);
            }
        }
    }
    return {Outcome::Applied, kNoStepDetail};
}

template <class Host>
StepResult
RunLadderComposeStep(Host &host, const StepRef &step, bool trackMoves)
{
    if (step.object < 0 ||
        size_t(step.object) >= host.inputs().composeGroupCount()) {
        return {Outcome::Invalid, kNoStepDetail};
    }
    int begin = 0;
    int end = 0;
    host.inputs().composeGroupRange(size_t(step.object), &begin, &end);
    return RunLadderComposeRange(host, begin, end, trackMoves);
}

// ---- ComposeSubtree ----
//
// One unswitched slot (_ComposeUnswitched): the authored posed matrix
// when posed is authored, else the avars against the namespace
// parent's posed space. The caller publishes base/fin/posedM.
template <class Math>
typename Math::Mat4
ComposeUnswitchedMatrix(const PoseSlotComposeT<Math> &slot,
                        const typename Math::Mat4 &parentPosed)
{
    if (slot.posedAuthored) {
        return *slot.posedAuthoredM;
    }
    return ComposeUnswitched<Math>(
        ComposeSlotAvars<Math>(slot), *slot.posedD, *slot.parentDinv,
        slot.parentSpaceAuthored ? *slot.parentSpaceM : parentPosed);
}

// One unswitched slot as a frame: the view, the matrix, the
// conversion. False when the slot has no bound compose inputs.
template <class Host>
bool
ComposeSlotFrame(Host &host, size_t slot,
                 const typename Host::Math::Mat4 &parentPosed,
                 std::array<typename Host::Math::Vec3, 4> *points,
                 unsigned *flags)
{
    using Math = typename Host::Math;
    PoseSlotComposeT<Math> view;
    if (!host.inputs().slotCompose(slot, &view)) {
        return false;
    }
    pose_ops::FrameFromMatrix<Math>(
        ComposeUnswitchedMatrix<Math>(view, parentPosed), points, flags);
    return true;
}

// Publish one unavailable slot: the default frame with cleared
// flags, and the sentinel space.
template <class Host>
void
PublishComposedSlotUnavailable(Host &host, size_t slot)
{
    using Math = typename Host::Math;
    host.state().writeBase(slot, pose_ops::IdentityLandmarks<Math>(), 0);
    host.state().writeFin(slot, pose_ops::IdentityLandmarks<Math>(), 0);
    host.state().writePosedM(slot, UnavailableSpace<Math>());
}

// The compose tail: fin from this run's base, posedM from its space.
template <class Host>
void
PublishFinPosedM(Host &host, size_t slot)
{
    using Math = typename Host::Math;
    std::array<typename Math::Vec3, 4> points = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned flags = 0;
    host.inputs().baseFrame(slot, &points, &flags);
    host.state().writeFin(slot, points, flags);
    host.state().writePosedM(slot, pose_ops::SpaceOfFrame<Math>(points, flags));
}

// ---- Commit fold (CommitDelta / PropagateChunk / CommitApply) ----
//
// One commit's static index lists, borrowed. All version lists share
// the uint32_t type on both sides; slots, positions and propagate
// pairs arrive through per-element accessors (int on one side,
// int32_t on the other).
struct PoseU32List {
    const uint32_t *data = nullptr;
    size_t count = 0;
};

struct PoseCommitFacts {
    PoseU32List slotReads, slotWrites, slotBaseWrites;
    PoseU32List slotCarry, slotBaseCarry;
    PoseU32List descendantReads, closestReads;
    PoseU32List descendantWrites, descendantBaseWrites;
    PoseU32List descendantCarry, descendantBaseCarry;
    PoseU32List sourceReads, targetReads, poleReads;
    uint32_t worldUpRead = 0;
    uint32_t targetRead = 0;
    uint32_t effectorRead = 0;
    bool solverOutput = false;
    bool split = false;
    std::string moverPath;
};

// One commit's mutable run state, borrowed. Presence, delta-ok and
// outcomes borrow directly with their counts; frames, deltas and
// staged frames travel through per-element accessors (array-of-struct
// on both sides) with their counts checked up front. The record
// gates travel as values: the frame-record step only reads them.
struct PoseCommitState {
    char *present = nullptr;
    size_t presentCount = 0;
    char *deltaOk = nullptr;
    size_t deltaOkCount = 0;
    uint8_t *outcome = nullptr;
    size_t outcomeCount = 0;
    bool *abandoned = nullptr;
    size_t framesCount = 0;
    size_t deltasCount = 0;
    size_t stagedCount = 0;
    size_t sourcesCount = 0;
    bool recordAfter = false;
    bool recordEveryTarget = false;
    bool recordGatesValid = false;
};

// The propagate Math the shared arithmetic runs through: usability
// from flags and finiteness, the carry through MatrixToPoints.
template <class Math> struct PosePropagateMath {
    static bool Usable(const typename Math::Frame &frame)
    {
        return FrameUsable<Math>(frame.points, frame.flags);
    }
    static bool PointsToMatrix(const std::array<typename Math::Vec3, 4> &rest,
                               const std::array<typename Math::Vec3, 4> &pose,
                               typename Math::Mat4 *matrix)
    {
        return pose_ops::PointsToMatrix<Math>(rest, pose, matrix);
    }
    static typename Math::Frame Carry(const typename Math::Frame &frame,
                                      const typename Math::Mat4 &delta)
    {
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        pose_ops::MatrixToPoints<Math>(frame.points, delta, &points, &flags);
        return Math::MakeFrame(points, flags);
    }
};

// The hierarchy delta each candidate carries, once.
template <class Host>
StepResult
RunCommitDeltaStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoCommit;
    using pose_detail::kPoseDetailNoSlot;
    using pose_detail::kPoseDetailNoVersion;
    if (step.object < 0 ||
        size_t(step.object) >= host.inputs().commitObjectCount()) {
        return {Outcome::Invalid, kPoseDetailNoCommit};
    }
    const size_t object = size_t(step.object);
    const PoseCommitFacts facts = host.inputs().commitFacts(object);
    auto state = host.state().commitState(object);
    if (*state.abandoned) {
        return {Outcome::Applied, kNoStepDetail};
    }
    const size_t slots = host.inputs().commitSlotCount(object);
    for (size_t pos = 0; pos < slots; ++pos) {
        if (pos >= state.presentCount || pos >= state.framesCount ||
            pos >= state.deltasCount || pos >= state.deltaOkCount ||
            pos >= facts.slotReads.count) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        if (!state.present[pos]) {
            state.deltaOk[pos] = 0;
            continue;
        }
        std::array<typename Math::Vec3, 4> before = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned beforeFlags = 0;
        std::array<typename Math::Vec3, 4> after = before;
        unsigned afterFlags = 0;
        if (!host.inputs().finFrame(facts.slotReads.data[pos], &before,
                                    &beforeFlags)) {
            return {Outcome::Invalid, kPoseDetailNoVersion};
        }
        host.state().commitFrame(object, pos, &after, &afterFlags);
        typename Math::Mat4 delta = Math::Identity4();
        const uint8_t ok = RigExecMeasureCommitDelta<PosePropagateMath<Math>>(
            before, after, &delta);
        host.state().writeCommitDelta(object, pos, delta);
        state.deltaOk[pos] = char(ok);
    }
    return {Outcome::Applied, kNoStepDetail};
}

// Stages descendants [begin, end) of the commit: each pair decided
// exactly where the walk decides it.
template <class Host>
StepResult
StageCommitPairs(Host &host, size_t object, size_t begin, size_t end)
{
    using Math = typename Host::Math;
    const PoseCommitFacts facts = host.inputs().commitFacts(object);
    auto state = host.state().commitState(object);
    const size_t pairs = host.inputs().commitPropagateCount(object);
    for (size_t k = begin; k < end && k < pairs; ++k) {
        using pose_detail::kPoseDetailNoSlot;
        using pose_detail::kPoseDetailNoVersion;
        int first = -1;
        int second = -1;
        int pos = -1;
        if (!host.inputs().commitPropagateAt(object, k, &first, &second) ||
            !host.inputs().commitClosestPosAt(object, k, &pos) ||
            k >= facts.descendantReads.count || k >= facts.closestReads.count ||
            k >= state.stagedCount || k >= state.outcomeCount) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        bool known = true;
        const bool parentBlocked = RigExecCommitParentBlocked(
            first, second,
            [&](int slot, bool *authored, int *next) {
                return host.inputs().commitPropParent(slot, authored, next);
            },
            &known);
        if (!known) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        if (RigExecCommitPairSkipped(parentBlocked, pos, state.present,
                                     state.presentCount)) {
            state.outcome[k] = uint8_t(pose_detail::kPosePropagateSkipped);
            continue;
        }
        if (size_t(pos) >= state.framesCount ||
            size_t(pos) >= state.deltasCount ||
            size_t(pos) >= state.deltaOkCount) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        std::array<typename Math::Vec3, 4> current = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned currentFlags = 0;
        std::array<typename Math::Vec3, 4> before = current;
        unsigned beforeFlags = 0;
        std::array<typename Math::Vec3, 4> after = current;
        unsigned afterFlags = 0;
        typename Math::Mat4 delta = Math::Identity4();
        if (!host.inputs().finFrame(facts.descendantReads.data[k], &current,
                                    &currentFlags)) {
            return {Outcome::Invalid, kPoseDetailNoVersion};
        }
        if (!host.inputs().finFrame(facts.closestReads.data[k], &before,
                                    &beforeFlags)) {
            return {Outcome::Invalid, kPoseDetailNoVersion};
        }
        host.state().commitFrame(object, size_t(pos), &after, &afterFlags);
        host.state().commitDelta(object, size_t(pos), &delta);
        typename Math::Frame out = Math::MakeFrame(after, afterFlags);
        const RigExecPosePropagateOutcome outcome =
            RigExecStageCommitPair<PosePropagateMath<Math>>(
                Math::MakeFrame(current, currentFlags),
                Math::MakeFrame(before, beforeFlags),
                Math::MakeFrame(after, afterFlags), delta,
                state.deltaOk[size_t(pos)] == 1, facts.solverOutput, &out);
        state.outcome[k] = uint8_t(outcome);
        if (outcome == RigExecPosePropagateOutcome::Staged) {
            host.state().writeCommitStaged(object, k, out.points, out.flags);
        }
    }
    return {Outcome::Applied, kNoStepDetail};
}

template <class Host>
StepResult
RunPropagateChunkStep(Host &host, const StepRef &step)
{
    if (step.object < 0 ||
        size_t(step.object) >= host.inputs().commitObjectCount()) {
        return {Outcome::Invalid, kNoStepDetail};
    }
    if (*host.state().commitState(size_t(step.object)).abandoned) {
        return {Outcome::Applied, kNoStepDetail};
    }
    if (step.part < 0) {
        return {Outcome::Invalid, kNoStepDetail};
    }
    const size_t pairs =
        host.inputs().commitPropagateCount(size_t(step.object));
    const size_t begin =
        size_t(step.part) * pose_detail::kPosePropagateChunkSize;
    const size_t end =
        std::min(begin + pose_detail::kPosePropagateChunkSize, pairs);
    return StageCommitPairs(host, size_t(step.object), begin, end);
}

// Decides the commit and writes it back, or says why it passed
// through. The first pair that neither staged nor was stepped over
// decides; a commit that passes through carries every site instead
// of writing.
template <class Host>
StepResult
RunCommitApplyStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoCommit;
    using pose_detail::kPoseDetailNoSlot;
    using pose_detail::kPoseDetailNoVersion;
    if (step.object < 0 ||
        size_t(step.object) >= host.inputs().commitObjectCount()) {
        return {Outcome::Invalid, kPoseDetailNoCommit};
    }
    const size_t object = size_t(step.object);
    const PoseCommitFacts facts = host.inputs().commitFacts(object);
    auto state = host.state().commitState(object);
    const size_t slots = host.inputs().commitSlotCount(object);
    const size_t pairs = host.inputs().commitPropagateCount(object);
    int32_t carryDetail = kNoStepDetail;
    const auto carry = [&](size_t pos, size_t k, bool candidates,
                           bool descendants) {
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        if (candidates) {
            if (pos >= facts.slotWrites.count || pos >= facts.slotCarry.count) {
                carryDetail = kPoseDetailNoSlot;
                return false;
            }
            if (!host.inputs().finFrame(facts.slotCarry.data[pos], &points,
                                        &flags) ||
                !host.state().writeFinVersion(facts.slotWrites.data[pos],
                                              points, flags)) {
                carryDetail = kPoseDetailNoVersion;
                return false;
            }
            if (facts.solverOutput) {
                if (pos >= facts.slotBaseWrites.count ||
                    pos >= facts.slotBaseCarry.count) {
                    carryDetail = kPoseDetailNoSlot;
                    return false;
                }
                if (!host.inputs().baseFrameAt(facts.slotBaseCarry.data[pos],
                                               &points, &flags) ||
                    !host.state().writeBaseVersion(
                        facts.slotBaseWrites.data[pos], points, flags)) {
                    carryDetail = kPoseDetailNoVersion;
                    return false;
                }
            }
        }
        if (descendants) {
            if (k >= facts.descendantWrites.count ||
                k >= facts.descendantCarry.count) {
                carryDetail = kPoseDetailNoSlot;
                return false;
            }
            if (!host.inputs().finFrame(facts.descendantCarry.data[k], &points,
                                        &flags) ||
                !host.state().writeFinVersion(facts.descendantWrites.data[k],
                                              points, flags)) {
                carryDetail = kPoseDetailNoVersion;
                return false;
            }
            if (facts.solverOutput) {
                if (k >= facts.descendantBaseWrites.count ||
                    k >= facts.descendantBaseCarry.count) {
                    carryDetail = kPoseDetailNoSlot;
                    return false;
                }
                if (!host.inputs().baseFrameAt(
                        facts.descendantBaseCarry.data[k], &points, &flags) ||
                    !host.state().writeBaseVersion(
                        facts.descendantBaseWrites.data[k], points, flags)) {
                    carryDetail = kPoseDetailNoVersion;
                    return false;
                }
            }
        }
        return true;
    };
    const auto carryEverything = [&]() {
        for (size_t pos = 0; pos < slots; ++pos) {
            if (!carry(pos, 0, true, false)) {
                return false;
            }
        }
        for (size_t k = 0; k < pairs; ++k) {
            if (!carry(0, k, false, true)) {
                return false;
            }
        }
        return true;
    };
    if (*state.abandoned) {
        return carryEverything() ? StepResult{Outcome::Applied, kNoStepDetail}
                                 : StepResult{Outcome::Invalid, carryDetail};
    }
    const size_t recorded = std::min(pairs, state.outcomeCount);
    const size_t failed = RigExecFirstCommitFailure(state.outcome, recorded);
    if (failed == recorded && recorded < pairs) {
        return {Outcome::Invalid, kPoseDetailNoSlot};
    }
    if (failed < pairs) {
        const size_t k = failed;
        const pose_detail::PosePropagateOutcome outcome =
            pose_detail::PosePropagateOutcome(state.outcome[k]);
        int first = -1;
        int second = -1;
        if (!host.inputs().commitPropagateAt(object, k, &first, &second) ||
            first < 0) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        const std::string path = host.inputs().commitPathText(size_t(first));
        switch (outcome) {
        case pose_detail::kPosePropagateUnusableDescendant:
            host.state().diagnose(
                facts.moverPath +
                " could not propagate its pose revision through " + path +
                "; constraint passed through");
            break;
        case pose_detail::kPosePropagateSingularDelta:
            host.state().diagnose(
                facts.moverPath +
                " produced a singular hierarchy delta; constraint "
                "passed through");
            break;
        default:
            host.state().diagnose(facts.moverPath +
                                  " produced an invalid descendant frame for " +
                                  path + "; constraint passed through");
            break;
        }
        return carryEverything() ? StepResult{Outcome::Applied, kNoStepDetail}
                                 : StepResult{Outcome::Invalid, carryDetail};
    }
    for (size_t pos = 0; pos < slots; ++pos) {
        if (pos >= state.presentCount) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        if (!state.present[pos]) {
            if (!carry(pos, 0, true, false)) {
                return {Outcome::Invalid, carryDetail};
            }
            continue;
        }
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        if (pos >= facts.slotWrites.count) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        host.state().commitFrame(object, pos, &points, &flags);
        if (!host.state().writeFinVersion(facts.slotWrites.data[pos], points,
                                          flags)) {
            return {Outcome::Invalid, kPoseDetailNoVersion};
        }
        if (facts.solverOutput) {
            if (pos >= facts.slotBaseWrites.count) {
                return {Outcome::Invalid, kPoseDetailNoSlot};
            }
            if (!host.state().writeBaseVersion(facts.slotBaseWrites.data[pos],
                                               points, flags)) {
                return {Outcome::Invalid, kPoseDetailNoVersion};
            }
        }
    }
    for (size_t k = 0; k < pairs; ++k) {
        if (k >= state.outcomeCount) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        if (pose_detail::PosePropagateOutcome(state.outcome[k]) !=
            pose_detail::kPosePropagateStaged) {
            if (!carry(0, k, false, true)) {
                return {Outcome::Invalid, carryDetail};
            }
            continue;
        }
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        if (k >= facts.descendantWrites.count) {
            return {Outcome::Invalid, kPoseDetailNoSlot};
        }
        host.state().commitStagedFrame(object, k, &points, &flags);
        if (!host.state().writeFinVersion(facts.descendantWrites.data[k],
                                          points, flags)) {
            return {Outcome::Invalid, kPoseDetailNoVersion};
        }
        if (facts.solverOutput) {
            if (k >= facts.descendantBaseWrites.count) {
                return {Outcome::Invalid, kPoseDetailNoSlot};
            }
            if (!host.state().writeBaseVersion(
                    facts.descendantBaseWrites.data[k], points, flags)) {
                return {Outcome::Invalid, kPoseDetailNoVersion};
            }
        }
    }
    return {Outcome::Applied, kNoStepDetail};
}

// A non-split commit's inline finish: the same three fold phases the
// split CommitDelta/PropagateChunk/CommitApply steps run, over the
// full pair range, in the same order. Abandoned commits skip the
// measure and stage exactly as both finish() lambdas do.
template <class Host>
StepResult
RunFinishCommit(Host &host, size_t object)
{
    StepRef ref;
    ref.kind = uint32_t(StepKind::CommitDelta);
    ref.object = int(object);
    ref.part = 0;
    ref.slot = 0;
    if (!*host.state().commitState(object).abandoned) {
        StepResult result = RunCommitDeltaStep(host, ref);
        if (result.outcome != Outcome::Applied) {
            return result;
        }
        result = StageCommitPairs(host, object, 0,
                                  host.inputs().commitPropagateCount(object));
        if (result.outcome != Outcome::Applied) {
            return result;
        }
    }
    ref.kind = uint32_t(StepKind::CommitApply);
    return RunCommitApplyStep(host, ref);
}

// ---- Frame matrix ----
//
// The record gate of a frame record: a solver commit records a
// position its solver published this run; a constraint commit records
// the exits its step left the gates on.
inline bool
FrameRecordRecorded(bool solverOutput, int position, int target,
                    const char *present, size_t presentCount, bool recordAfter,
                    bool recordEveryTarget)
{
    if (solverOutput) {
        return position >= 0 && size_t(position) < presentCount &&
               present[size_t(position)];
    }
    return recordAfter && !(target > 0 && !recordEveryTarget);
}

// The record's provider frame, as its writer left it, converted
// against the rest landmarks (the identity landmarks where the rest
// is unusable). Identity and false where the frame does not convert.
template <class Math>
bool
FrameRecordMatrix(const std::array<typename Math::Vec3, 4> &restPoints,
                  unsigned restFlags,
                  const std::array<typename Math::Vec3, 4> &framePoints,
                  unsigned frameFlags, typename Math::Mat4 *matrix)
{
    *matrix = Math::Identity4();
    if (!(frameFlags & pose_ops::kPoseFrameValid)) {
        return false;
    }
    const std::array<typename Math::Vec3, 4> landmarks =
        (restFlags & pose_ops::kPoseFrameValid)
            ? restPoints
            : pose_ops::IdentityLandmarks<Math>();
    return pose_ops::PointsToMatrix<Math>(landmarks, framePoints, matrix);
}

// ---- Harmonics arm ----
//
// One harmonics constraint's bound inputs. Scalars arrive resolved
// (narrow or wide on each side); the envelope field arrives as one
// sampled view.
template <class Math> struct PoseHarmonicsFacts {
    int target = -1;
    int spaceSlot = -1;
    int window = -1;
    std::string pathText;
    bool enabled = true;
    bool hasWeightObject = false;
    bool fieldOk = false;
    size_t fieldCount = 0;
    double fieldValue = 0.0;
    std::string fieldError;
    double defaultWeight = 1.0;
    double scalars[3] = {0.0, 0.0, 0.0};
    typename Math::Vec3 axis = typename Math::Vec3();
    bool matchVelocity = false;
    RigExecHarmonicsSettings settings;
    bool settingsValid = false;
};

// The constraint's step captures its target's incoming position,
// measured in rigExec:space, as this tick's packet. When it solves,
// the earlier ticks' packets and this one feed the shared harmonics
// kernel, and the offset revises the one target. Mid-arm failures
// pass through with the same diagnostic on both sides; only the
// upfront window/read validation is fatal, and only on the runtime.
template <class Host>
StepResult
RunHarmonicsArm(Host &host, size_t object)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    using Math = typename Host::Math;
    const PoseCommitFacts commitFacts = host.inputs().commitFacts(object);
    auto state = host.state().commitState(object);
    host.state().setRecordEveryTarget(object, false);
    const PoseHarmonicsFacts<Math> facts = host.inputs().harmonicsFacts(object);
    if (facts.window < 0 || !host.inputs().windowCaptureValid(facts.window) ||
        !host.inputs().windowedOpValid(facts.window) || !facts.settingsValid) {
        return {Outcome::Invalid, pose_detail::kPoseDetailNoWindow};
    }
    std::array<typename Math::Vec3, 4> inputPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned inputFlags = 0;
    if (!host.inputs().finFrame(commitFacts.targetRead, &inputPoints,
                                &inputFlags)) {
        return {Outcome::Invalid, pose_detail::kPoseDetailNoWindow};
    }
    typename Math::Mat4 space = Math::Identity4();
    typename Math::Mat4 toSpace = Math::Identity4();
    bool spaceUsable = true;
    if (facts.spaceSlot >= 0) {
        if (!host.inputs().posedMatrix(size_t(facts.spaceSlot), &space)) {
            return {Outcome::Invalid, pose_detail::kPoseDetailNoWindow};
        }
        const double det = pose_ops::Determinant4<Math>(space);
        spaceUsable = std::isfinite(det) && det != 0.0;
        if (spaceUsable) {
            pose_ops::Invert4<Math>(space, 0.0, &toSpace, nullptr);
        }
    }
    RigExecWindowPacketPtr captured;
    if (spaceUsable && FrameUsable<Math>(inputPoints, inputFlags)) {
        const typename Math::Vec3 p =
            facts.spaceSlot >= 0
                ? pose_ops::Transform3<Math>(toSpace, inputPoints[0])
                : inputPoints[0];
        if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
            auto packet = std::make_shared<RigExecWindowPacket>();
            packet->scalars = {p[0], p[1], p[2]};
            captured = std::move(packet);
        }
    }
    RigExecWindowPacketPtr &held = host.state().windowCapture(facts.window);
    if (!RigExecWindowPacketsEqual(held, captured)) {
        held = std::move(captured);
    }
    if (host.inputs().historyTick() &&
        !host.inputs().windowFeedsWindow(facts.window)) {
        return {Outcome::Applied, kNoStepDetail};
    }
    if (!facts.enabled) {
        return {Outcome::Applied, kNoStepDetail};
    }
    double weight = 1.0;
    if (facts.hasWeightObject) {
        if (!facts.fieldOk || facts.fieldCount != 1) {
            host.state().diagnose(facts.pathText + ": " + facts.fieldError +
                                  "; constraint passed through");
            return {Outcome::Applied, kNoStepDetail};
        }
        weight = facts.fieldValue;
    } else {
        weight = facts.defaultWeight;
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
            host.state().diagnose(
                facts.pathText +
                " has inputs:defaultWeight outside finite [0, 1]; "
                "constraint passed through");
            return {Outcome::Applied, kNoStepDetail};
        }
    }
    if (weight <= 0.0) {
        return {Outcome::Applied, kNoStepDetail};
    }
    if (!held) {
        host.state().diagnose(
            facts.pathText + " could not measure " +
            host.inputs().commitPathText(size_t(facts.target)) +
            (spaceUsable ? "" : " in a degenerate rigExec:space") +
            "; constraint passed through");
        return {Outcome::Applied, kNoStepDetail};
    }
    const size_t ringCount = host.inputs().windowRingCount(facts.window);
    size_t begin = ringCount;
    while (begin > 0) {
        const RigExecWindowPacketPtr packet =
            host.inputs().windowRingPacket(facts.window, begin - 1);
        if (!packet || packet->scalars.size() != 3) {
            break;
        }
        --begin;
    }
    std::vector<double> positions;
    positions.reserve(3 * (ringCount - begin + 1));
    for (size_t k = begin; k < ringCount; ++k) {
        const RigExecWindowPacketPtr packet =
            host.inputs().windowRingPacket(facts.window, k);
        positions.insert(positions.end(), packet->scalars.begin(),
                         packet->scalars.end());
    }
    positions.insert(positions.end(), held->scalars.begin(),
                     held->scalars.end());
    const RigExecHarmonicsSettings &settings = facts.settings;
    RigExecHarmonicsParameters params;
    params.amplitude = facts.scalars[0];
    params.decay = facts.scalars[1];
    params.termination = facts.scalars[2];
    params.axisAmplitude[0] = facts.axis[0];
    params.axisAmplitude[1] = facts.axis[1];
    params.axisAmplitude[2] = facts.axis[2];
    params.matchVelocity = facts.matchVelocity;
    if (!std::isfinite(params.amplitude) || !std::isfinite(params.decay) ||
        !std::isfinite(params.termination) || !std::isfinite(facts.axis[0]) ||
        !std::isfinite(facts.axis[1]) || !std::isfinite(facts.axis[2])) {
        host.state().diagnose(
            facts.pathText +
            " has a non-finite input; constraint passed through");
        return {Outcome::Applied, kNoStepDetail};
    }
    double ring3[3];
    if (!RigExecHarmonicsOffset(
            settings, params,
            reinterpret_cast<const double(*)[3]>(positions.data()),
            positions.size() / 3, ring3)) {
        return {Outcome::Applied, kNoStepDetail};
    }
    typename Math::Vec3 offset = Math::Vec3Of(ring3[0], ring3[1], ring3[2]);
    if (facts.spaceSlot >= 0) {
        offset = pose_ops::TransformDir3<Math>(space, offset);
    }
    offset = pose_ops::Scale3<Math>(offset, weight);
    std::array<typename Math::Vec3, 4> candidate = inputPoints;
    for (size_t q = 0; q < 4; ++q) {
        candidate[q] = pose_ops::Add3<Math>(candidate[q], offset);
    }
    if (!FrameUsable<Math>(candidate, inputFlags)) {
        host.state().diagnose(
            facts.pathText + " produced an invalid or degenerate frame for " +
            host.inputs().commitPathText(size_t(facts.target)) +
            "; constraint passed through");
        return {Outcome::Applied, kNoStepDetail};
    }
    if (state.framesCount < 1 || state.presentCount < 1) {
        return {Outcome::Invalid, pose_detail::kPoseDetailNoSlot};
    }
    host.state().writeCommitFrame(object, 0, candidate, inputFlags);
    state.present[0] = 1;
    *state.abandoned = false;
    return {Outcome::Applied, kNoStepDetail};
}

// ---- Constraint prologue ----
//
// One constraint's sampled array tables, borrowed from the
// adapter's scratch: source weights, offsets, pole weights, and the
// cardinality diagnostics sampling produced.
template <class Math> struct PoseArraysViewT {
    bool ok = true;
    const double *weights = nullptr;
    size_t weightCount = 0;
    const typename Math::Vec3 *translationOffsets = nullptr;
    const typename Math::Vec3 *rotationOffsets = nullptr;
    size_t offsetCount = 0;
    bool poleOk = true;
    const double *poleWeights = nullptr;
    size_t poleWeightCount = 0;
    const std::string *diagnostics = nullptr;
    size_t diagnosticCount = 0;
    const std::string *poleDiagnostics = nullptr;
    size_t poleDiagnosticCount = 0;
};

enum PoseWorldUpCode : uint8_t {
    kPoseWorldUpNone = 0,
    kPoseWorldUpScene = 1,
    kPoseWorldUpVector = 2,
    kPoseWorldUpObject = 3,
    kPoseWorldUpObjectRotation = 4
};

// One constraint's prologue inputs, adapter-bound: identity and
// switches, the envelope selection, sources, affect/order/carry, and
// the sampled arrays.
template <class Math> struct PoseConstraintFactsT {
    int target = -1;
    int deltaBase = -1;
    int spaceSlot = -1;
    PoseConstraintCode code = kPoseConstraintUnknown;
    std::string pathText;
    bool enabled = true;
    bool hasWeightObject = false;
    bool pointsTarget = false;
    bool fieldOk = false;
    size_t fieldCount = 0;
    double fieldValue = 0.0;
    std::string fieldError;
    double defaultWeight = 1.0;
    PoseArraysViewT<Math> arrays;
    bool affectX = true;
    bool affectY = true;
    bool affectZ = true;
    pose_constraints::PoseEulerOrder order = pose_constraints::kPoseEulerXYZ;
    typename Math::Vec3 offset;
    bool blendShear = false;
    pose_constraints::PoseAxisMask translationAxes;
    pose_constraints::PoseAxisMask rotationAxes;
    pose_constraints::PoseAxisMask scaleAxes;
    bool ikFkMatch = false;
    bool harmonics = false;
    bool radialBlend = false;
    bool singleChainIk = false;
};

// rigExec:space: the space switch's carry, from the same two tables
// the compose reads it from. -1 is "no space named".
template <class Host>
bool
ComputeSpaceCarry(Host &host, int spaceSlot, typename Host::Math::Mat4 *carry)
{
    using Math = typename Host::Math;
    *carry = Math::Identity4();
    if (spaceSlot < 0) {
        return true;
    }
    typename Math::Mat4 roundTrip = Math::Identity4();
    typename Math::Mat4 posed = Math::Identity4();
    if (!host.inputs().defaultRoundTrip(size_t(spaceSlot), &roundTrip) ||
        !host.inputs().posedMatrix(size_t(spaceSlot), &posed)) {
        return false;
    }
    typename Math::Mat4 inverse = Math::Identity4();
    pose_ops::Invert4<Math>(roundTrip, 0.0, &inverse, nullptr);
    *carry = pose_ops::Multiply4<Math>(inverse, posed);
    return true;
}

// One constraint source reference: a provider slot read as a fin version, or a
// native source by index.
struct PoseSourceRef {
    int slot = -1;
    uint32_t finRead = 0;
    int native = -1;
    std::string pathText;
};

// One gathered source: the frame plus its envelope weight and
// offsets.
template <class Math> struct PoseSourceT {
    std::array<typename Math::Vec3, 4> points;
    unsigned flags = 0;
    double normalizedWeight = 0.0;
    typename Math::Vec3 translationOffset;
    typename Math::Vec3 rotationOffsetDegrees;
};

// The Aim arm's bound inputs: vectors, world-up selection, and the
// up-object reference (resolved like a source when named).
template <class Math> struct PoseAimFactsT {
    typename Math::Vec3 aimVector;
    typename Math::Vec3 upVector;
    typename Math::Vec3 rotationOffset;
    bool preserveInputUp = false;
    PoseWorldUpCode worldUpCode = kPoseWorldUpNone;
    typename Math::Vec3 sceneUp;
    typename Math::Vec3 authoredWorldUp;
    bool worldUpObjectNamed = false;
    int worldUpObject = -1;
    int worldUpNative = -1;
    bool worldUpRotationOnly = false;
};

// The rotation-only world-up direction: the up-object's rotation
// applied to the authored vector.
template <class Math>
typename Math::Vec3
RotationOnlyUp(const typename Math::Mat4 &up, bool rotationOnly,
               const typename Math::Vec3 &authored)
{
    typename Math::Mat4 m = up;
    if (rotationOnly) {
        pose_ops::Orthonormalize4<Math>(&m);
    }
    double w = 0.0;
    typename Math::Vec3 im = Math::Vec3Of(0, 0, 0);
    Math::QuatToComponents(pose_ops::ExtractRotationQuat<Math>(m), &w, &im);
    const double len = pose_ops::Length3<Math>(im);
    pose_ops::PoseRotationT<Math> rotation;
    rotation.axis = Math::Vec3Of(1.0, 0.0, 0.0);
    rotation.angle = 0.0;
    if (len > 1e-10) {
        const double x = std::acos(pose_ops::Clamp(w, -1.0, 1.0));
        rotation.axis = pose_ops::Scale3<Math>(im, 1.0 / len);
        constexpr double pi = 3.141592653589793;
        rotation.angle = 2.0 * (x * (180.0 / pi));
    }
    return pose_ops::RotationTransformDir<Math>(rotation, authored);
}

// One constraint's gathered inputs: the commit binding, the facts,
// the resolved source set, the envelope, and the space carry.
template <class Host> struct PoseConstraintPrologueT {
    using Math = typename Host::Math;
    PoseCommitFacts commitFacts;
    PoseCommitState state;
    PoseConstraintFactsT<Math> facts;
    std::vector<PoseSourceT<Math>> sources;
    double weight = 1.0;
    typename Math::Mat4 carry;
    bool carryValid = false;
    bool hasCarry = false;
};

// The constraint prologue head: arrays, facts, and the envelope.
// The single-chain IK exit sits between this and the sources below: past
// the envelope (whose weight the solve takes) and past the arrays (whose
// pole tables it reads), before any source resolves.
template <class Host>
StepResult
ResolveConstraintHead(Host &host, size_t object,
                      PoseConstraintPrologueT<Host> *prologue, bool *proceed)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoArrays;
    using pose_detail::kPoseDetailNoDeltaPath;
    *proceed = false;
    prologue->carry = Math::Identity4();
    prologue->commitFacts = host.inputs().commitFacts(object);
    prologue->state = host.state().commitState(object);
    if (!host.state().sampleConstraintArrays(object)) {
        return {Outcome::Invalid, kPoseDetailNoArrays};
    }
    prologue->facts = host.inputs().constraintFacts(object);
    const PoseConstraintFactsT<Math> &facts = prologue->facts;
    if (!facts.enabled) {
        return {Outcome::Applied, kNoStepDetail};
    }
    double weight = 1.0;
    if (facts.hasWeightObject && !facts.pointsTarget) {
        // The front ends record the resolved field in the constraint's own
        // scratch (baked program record, runtime store slot) before it is
        // checked.
        host.state().recordConstraintEnvelope(object);
        if (!facts.fieldOk || facts.fieldCount != 1) {
            host.state().diagnose(facts.pathText + ": " + facts.fieldError +
                                  "; constraint passed through");
            return {Outcome::Applied, kNoStepDetail};
        }
        weight = facts.fieldValue;
    } else if (!facts.hasWeightObject) {
        weight = facts.defaultWeight;
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
            host.state().diagnose(
                facts.pathText +
                " has inputs:defaultWeight outside finite [0, 1]; "
                "constraint passed through");
            return {Outcome::Applied, kNoStepDetail};
        }
    }
    prologue->weight = weight;
    if (weight <= 0.0 && (!facts.pointsTarget || !facts.hasWeightObject)) {
        if (facts.deltaBase >= 0 &&
            !host.state().writeDeltaIdentity(size_t(facts.deltaBase))) {
            return {Outcome::Invalid, kPoseDetailNoDeltaPath};
        }
        return {Outcome::Applied, kNoStepDetail};
    }
    *proceed = true;
    return {Outcome::Applied, kNoStepDetail};
}

// The constraint prologue tail: the source set, the record gates,
// and the space carry.
template <class Host>
StepResult
ResolveConstraintSources(Host &host, size_t object,
                         PoseConstraintPrologueT<Host> *prologue, bool *proceed)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoSource;
    using pose_detail::kPoseDetailNoSpace;
    *proceed = false;
    const PoseConstraintFactsT<Math> &facts = prologue->facts;
    host.state().setRecordEveryTarget(object, false);
    const size_t sourceCount = host.inputs().constraintSourceCount(object);
    bool sourcesReady = facts.arrays.ok;
    for (size_t d = 0; d < facts.arrays.diagnosticCount; ++d) {
        host.state().diagnose(facts.arrays.diagnostics[d]);
    }
    std::vector<PoseSourceT<Math>> &sources = prologue->sources;
    sources.reserve(sourceCount);
    for (size_t k = 0; sourcesReady && k < sourceCount; ++k) {
        PoseSourceRef ref;
        if (!host.inputs().constraintSourceRef(object, k, &ref) ||
            k >= prologue->state.sourcesCount ||
            k >= facts.arrays.weightCount || k >= facts.arrays.offsetCount) {
            return {Outcome::Invalid, kPoseDetailNoSource};
        }
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        bool resolved = false;
        if (ref.slot >= 0) {
            resolved = host.inputs().finFrame(ref.finRead, &points, &flags) &&
                       (flags & pose_ops::kPoseFrameValid) != 0;
        } else {
            resolved =
                host.inputs().resolveNativeSource(object, k, &points, &flags) &&
                (flags & pose_ops::kPoseFrameValid) != 0;
        }
        if (!resolved) {
            host.state().diagnose(facts.pathText +
                                  " could not resolve source " + ref.pathText);
            sourcesReady = false;
            break;
        }
        PoseSourceT<Math> source;
        source.points = points;
        source.flags = flags;
        source.normalizedWeight = facts.arrays.weights[k];
        source.translationOffset = facts.arrays.translationOffsets[k];
        source.rotationOffsetDegrees = facts.arrays.rotationOffsets[k];
        sources.push_back(source);
        host.state().writeConstraintSource(object, k, points, flags,
                                           facts.arrays.weights[k],
                                           facts.arrays.translationOffsets[k],
                                           facts.arrays.rotationOffsets[k]);
    }
    if (!sourcesReady) {
        host.state().diagnose(
            facts.pathText +
            " has unusable constraint inputs; constraint passed through");
        return {Outcome::Applied, kNoStepDetail};
    }
    // Past this point the walk records nothing for a geometry-domain
    // constraint: it revises no transform, so there is no "the target
    // as of this mover" to record.
    host.state().setRecordAfter(object, facts.deltaBase < 0);
    if (!ComputeSpaceCarry<Host>(host, facts.spaceSlot, &prologue->carry)) {
        return {Outcome::Invalid, pose_detail::kPoseDetailNoSpace};
    }
    prologue->carryValid = true;
    prologue->hasCarry = facts.spaceSlot >= 0;
    *proceed = true;
    return {Outcome::Applied, kNoStepDetail};
}

// The whole prologue, head then sources, for callers with no exit
// between them.
template <class Host>
StepResult
ResolveConstraintInputs(Host &host, size_t object,
                        PoseConstraintPrologueT<Host> *prologue, bool *proceed)
{
    *proceed = false;
    const StepResult head =
        ResolveConstraintHead(host, object, prologue, proceed);
    if (!*proceed) {
        return head;
    }
    return ResolveConstraintSources(host, object, prologue, proceed);
}

// The constraint epilogue every arm writes through: the candidate
// observation, the delta measure, and the commit write.
template <class Host>
StepResult
WriteConstraintCandidate(
    Host &host, size_t object, const PoseCommitFacts &commitFacts,
    const PoseConstraintFactsT<typename Host::Math> &facts,
    PoseCommitState state,
    const std::array<typename Host::Math::Vec3, 4> &candidatePoints,
    unsigned candidateFlags, bool candidateReady)
{
    using pose_detail::kPoseDetailNoCandidate;
    using pose_detail::kPoseDetailNoDeltaPath;
    using pose_detail::kPoseDetailNoTarget;
    (void)commitFacts;
    if (candidateReady) {
        host.state().observeCandidate(object, candidatePoints, candidateFlags);
    }
    if (candidateReady && facts.deltaBase >= 0) {
        if (!host.inputs().deltaBaseValid(facts.deltaBase)) {
            return {Outcome::Invalid, kPoseDetailNoDeltaPath};
        }
        typename Host::Math::Mat4 baseMatrix = Host::Math::Identity4();
        host.inputs().deltaBaseMatrix(facts.deltaBase, &baseMatrix);
        typename Host::Math::Mat4 solvedMatrix = Host::Math::Identity4();
        const double det =
            pose_ops::Determinant4<typename Host::Math>(baseMatrix);
        if (FrameUsable<typename Host::Math>(candidatePoints, candidateFlags) &&
            host.inputs().deltaBaseOk(facts.deltaBase) && std::isfinite(det) &&
            det != 0.0 &&
            pose_ops::PointsToMatrix<typename Host::Math>(
                pose_ops::IdentityLandmarks<typename Host::Math>(),
                candidatePoints, &solvedMatrix)) {
            typename Host::Math::Mat4 baseInv = Host::Math::Identity4();
            pose_ops::Invert4<typename Host::Math>(baseMatrix, 0.0, &baseInv,
                                                   nullptr);
            host.state().writeDeltaMeasured(
                size_t(facts.deltaBase),
                pose_ops::Multiply4<typename Host::Math>(solvedMatrix,
                                                         baseInv));
        } else {
            host.state().diagnose(facts.pathText +
                                  " could not measure its delta against " +
                                  host.inputs().deltaBasePath(object) +
                                  "; constraint passed through");
        }
    } else if (candidateReady) {
        if (!FrameUsable<typename Host::Math>(candidatePoints,
                                              candidateFlags)) {
            if (facts.target < 0 || !host.inputs().pathValid(facts.target)) {
                return {Outcome::Invalid, kPoseDetailNoTarget};
            }
            host.state().diagnose(
                facts.pathText +
                " produced an invalid or degenerate frame for " +
                host.inputs().commitPathText(size_t(facts.target)) +
                "; constraint passed through");
        } else {
            if (state.framesCount < 1 || state.presentCount < 1) {
                return {Outcome::Invalid, kPoseDetailNoCandidate};
            }
            host.state().writeCommitFrame(object, 0, candidatePoints,
                                          candidateFlags);
            state.present[0] = 1;
            *state.abandoned = false;
        }
    }
    return {Outcome::Applied, kNoStepDetail};
}

// The Aim solve over gathered sources: the target point, the
// swing/twist, the delta epilogue, and the commit write.
template <class Host>
StepResult
RunAimSolve(Host &host, size_t object, const PoseCommitFacts &commitFacts,
            const PoseConstraintFactsT<typename Host::Math> &facts,
            std::vector<PoseSourceT<typename Host::Math>> sources,
            double weight, PoseCommitState state)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoVersion;
    const double solveWeight = facts.deltaBase < 0 ? weight : 1.0;
    std::array<typename Math::Vec3, 4> inputPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned inputFlags = 0;
    if (!host.inputs().finFrame(commitFacts.targetRead, &inputPoints,
                                &inputFlags)) {
        return {Outcome::Invalid, kPoseDetailNoVersion};
    }
    typename Math::Vec3 target = Math::Vec3Of(0, 0, 0);
    double total = 0;
    bool candidateReady = true;
    for (const PoseSourceT<Math> &source : sources) {
        if (!std::isfinite(source.normalizedWeight) ||
            source.normalizedWeight < 0) {
            host.state().diagnose(
                facts.pathText +
                " has an invalid source weight; constraint passed through");
            candidateReady = false;
            break;
        }
        target = pose_ops::Add3<Math>(
            target,
            pose_ops::Scale3<Math>(source.points[0], source.normalizedWeight));
        total += source.normalizedWeight;
    }
    std::array<typename Math::Vec3, 4> candidatePoints = inputPoints;
    unsigned candidateFlags = inputFlags;
    if (candidateReady && total > 0) {
        target = pose_ops::Scale3<Math>(target, 1.0 / total);
        const PoseAimFactsT<Math> aim = host.inputs().aimFacts(object);
        pose_constraints::PoseAimParamsT<Math> params;
        params.localAimVector = aim.aimVector;
        params.localUpVector = aim.upVector;
        params.rotationOffsetDegrees = aim.rotationOffset;
        params.affectRotation.x = facts.affectX;
        params.affectRotation.y = facts.affectY;
        params.affectRotation.z = facts.affectZ;
        params.rotationOrder = facts.order;
        params.weight = solveWeight;
        params.preserveInputUp = aim.preserveInputUp;
        typename Math::Vec3 worldUpStorage = Math::Vec3Of(0, 0, 0);
        bool haveWorldUp = false;
        switch (aim.worldUpCode) {
        case kPoseWorldUpScene:
            worldUpStorage = aim.sceneUp;
            haveWorldUp = true;
            break;
        case kPoseWorldUpVector:
            worldUpStorage = aim.authoredWorldUp;
            haveWorldUp = true;
            break;
        case kPoseWorldUpObject:
            if (!aim.worldUpObjectNamed) {
                worldUpStorage = pose_ops::Scale3<Math>(inputPoints[0], -1.0);
                haveWorldUp = true;
            } else {
                std::array<typename Math::Vec3, 4> upPoints = {
                    Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
                    Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0)};
                unsigned upFlags = 0;
                if (!host.inputs().resolveWorldUp(object, &upPoints,
                                                  &upFlags)) {
                    host.state().diagnose(
                        facts.pathText +
                        " could not resolve its world-up object; "
                        "constraint passed through");
                    candidateReady = false;
                } else {
                    worldUpStorage =
                        pose_ops::Sub3<Math>(upPoints[0], inputPoints[0]);
                    haveWorldUp = true;
                }
            }
            break;
        case kPoseWorldUpObjectRotation:
            if (!aim.worldUpObjectNamed) {
                worldUpStorage = aim.authoredWorldUp;
                haveWorldUp = true;
            } else {
                std::array<typename Math::Vec3, 4> upPoints = {
                    Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
                    Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0)};
                unsigned upFlags = 0;
                if (!host.inputs().resolveWorldUp(object, &upPoints,
                                                  &upFlags)) {
                    host.state().diagnose(
                        facts.pathText +
                        " could not resolve its world-up object; "
                        "constraint passed through");
                    candidateReady = false;
                }
                typename Math::Mat4 up = Math::Identity4();
                if (candidateReady &&
                    !pose_ops::PointsToMatrix<Math>(
                        pose_ops::IdentityLandmarks<Math>(), upPoints, &up)) {
                    host.state().diagnose(facts.pathText +
                                          " has a degenerate world-up object");
                    candidateReady = false;
                } else if (candidateReady) {
                    worldUpStorage = RotationOnlyUp<Math>(
                        up, aim.worldUpRotationOnly, aim.authoredWorldUp);
                    haveWorldUp = true;
                }
            }
            break;
        case kPoseWorldUpNone:
            break;
        }
        if (haveWorldUp) {
            params.worldUpDirection = &worldUpStorage;
        }
        if (candidateReady) {
            pose_constraints::ApplyAimConstraint<Math>(
                inputPoints, inputFlags, target, params, &candidatePoints,
                &candidateFlags);
        }
    }
    return WriteConstraintCandidate(host, object, commitFacts, facts, state,
                                    candidatePoints, candidateFlags,
                                    candidateReady);
}

// The Position solve over gathered sources: the shared input read,
// the FBX apply, and the shared epilogue.
template <class Host>
StepResult
RunPositionSolve(Host &host, size_t object, const PoseCommitFacts &commitFacts,
                 const PoseConstraintFactsT<typename Host::Math> &facts,
                 const std::vector<PoseSourceT<typename Host::Math>> &sources,
                 double weight, PoseCommitState state)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoVersion;
    const double solveWeight = facts.deltaBase < 0 ? weight : 1.0;
    std::array<typename Math::Vec3, 4> inputPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned inputFlags = 0;
    if (!host.inputs().finFrame(commitFacts.targetRead, &inputPoints,
                                &inputFlags)) {
        return {Outcome::Invalid, kPoseDetailNoVersion};
    }
    pose_constraints::PosePositionParamsT<Math> params;
    params.offset = facts.offset;
    params.affect.x = facts.affectX;
    params.affect.y = facts.affectY;
    params.affect.z = facts.affectZ;
    params.weight = solveWeight;
    std::array<typename Math::Vec3, 4> candidatePoints = inputPoints;
    unsigned candidateFlags = inputFlags;
    pose_constraints::ApplyPositionConstraint<Math>(
        inputPoints, inputFlags, sources, params, &candidatePoints,
        &candidateFlags);
    return WriteConstraintCandidate(host, object, commitFacts, facts, state,
                                    candidatePoints, candidateFlags, true);
}

// The Rotation solve over gathered sources.
template <class Host>
StepResult
RunRotationSolve(Host &host, size_t object, const PoseCommitFacts &commitFacts,
                 const PoseConstraintFactsT<typename Host::Math> &facts,
                 const std::vector<PoseSourceT<typename Host::Math>> &sources,
                 double weight, PoseCommitState state,
                 const typename Host::Math::Mat4 *carry, bool hasCarry)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoVersion;
    const double solveWeight = facts.deltaBase < 0 ? weight : 1.0;
    std::array<typename Math::Vec3, 4> inputPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned inputFlags = 0;
    if (!host.inputs().finFrame(commitFacts.targetRead, &inputPoints,
                                &inputFlags)) {
        return {Outcome::Invalid, kPoseDetailNoVersion};
    }
    pose_constraints::PoseRotationParamsT<Math> params;
    params.offsetDegrees = facts.offset;
    params.affect.x = facts.affectX;
    params.affect.y = facts.affectY;
    params.affect.z = facts.affectZ;
    params.rotationOrder = facts.order;
    params.weight = solveWeight;
    params.carry = hasCarry ? carry : nullptr;
    std::array<typename Math::Vec3, 4> candidatePoints = inputPoints;
    unsigned candidateFlags = inputFlags;
    pose_constraints::ApplyRotationConstraint<Math>(
        inputPoints, inputFlags, sources, params, &candidatePoints,
        &candidateFlags);
    return WriteConstraintCandidate(host, object, commitFacts, facts, state,
                                    candidatePoints, candidateFlags, true);
}

// The Scale solve over gathered sources.
template <class Host>
StepResult
RunScaleSolve(Host &host, size_t object, const PoseCommitFacts &commitFacts,
              const PoseConstraintFactsT<typename Host::Math> &facts,
              const std::vector<PoseSourceT<typename Host::Math>> &sources,
              double weight, PoseCommitState state)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoVersion;
    const double solveWeight = facts.deltaBase < 0 ? weight : 1.0;
    std::array<typename Math::Vec3, 4> inputPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned inputFlags = 0;
    if (!host.inputs().finFrame(commitFacts.targetRead, &inputPoints,
                                &inputFlags)) {
        return {Outcome::Invalid, kPoseDetailNoVersion};
    }
    pose_constraints::PoseScaleParamsT<Math> params;
    params.offset = facts.offset;
    params.affect.x = facts.affectX;
    params.affect.y = facts.affectY;
    params.affect.z = facts.affectZ;
    params.weight = solveWeight;
    params.blendShear = facts.blendShear;
    std::array<typename Math::Vec3, 4> candidatePoints = inputPoints;
    unsigned candidateFlags = inputFlags;
    pose_constraints::ApplyScaleConstraint<Math>(
        inputPoints, inputFlags, sources, params, &candidatePoints,
        &candidateFlags);
    return WriteConstraintCandidate(host, object, commitFacts, facts, state,
                                    candidatePoints, candidateFlags, true);
}

// The Parent solve over gathered sources.
template <class Host>
StepResult
RunParentSolve(Host &host, size_t object, const PoseCommitFacts &commitFacts,
               const PoseConstraintFactsT<typename Host::Math> &facts,
               const std::vector<PoseSourceT<typename Host::Math>> &sources,
               double weight, PoseCommitState state,
               const typename Host::Math::Mat4 *carry, bool hasCarry)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoVersion;
    const double solveWeight = facts.deltaBase < 0 ? weight : 1.0;
    std::array<typename Math::Vec3, 4> inputPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned inputFlags = 0;
    if (!host.inputs().finFrame(commitFacts.targetRead, &inputPoints,
                                &inputFlags)) {
        return {Outcome::Invalid, kPoseDetailNoVersion};
    }
    pose_constraints::PoseParentParamsT<Math> params;
    params.translationAxes = facts.translationAxes;
    params.rotationAxes = facts.rotationAxes;
    params.scaleAxes = facts.scaleAxes;
    params.rotationOrder = facts.order;
    params.weight = solveWeight;
    params.carry = hasCarry ? carry : nullptr;
    params.blendShear = facts.blendShear;
    std::array<typename Math::Vec3, 4> candidatePoints = inputPoints;
    unsigned candidateFlags = inputFlags;
    pose_constraints::ApplyParentConstraint<Math>(
        inputPoints, inputFlags, sources, params, &candidatePoints,
        &candidateFlags);
    return WriteConstraintCandidate(host, object, commitFacts, facts, state,
                                    candidatePoints, candidateFlags, true);
}

// The MatrixMover solve over gathered sources: each source measured
// as its rest->posed map, the transform in its space, and the
// envelope applied to the target's four landmarks.
template <class Host>
StepResult
RunMatrixSolve(Host &host, size_t object, const PoseCommitFacts &commitFacts,
               const PoseConstraintFactsT<typename Host::Math> &facts,
               const std::vector<PoseSourceT<typename Host::Math>> &sources,
               double weight, PoseCommitState state)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoVersion;
    std::array<typename Math::Vec3, 4> inputPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned inputFlags = 0;
    if (!host.inputs().finFrame(commitFacts.targetRead, &inputPoints,
                                &inputFlags)) {
        return {Outcome::Invalid, kPoseDetailNoVersion};
    }
    const auto mapOf = [&](size_t k, typename Math::Mat4 *out) {
        PoseSourceRef ref;
        if (!host.inputs().constraintSourceRef(object, k, &ref)) {
            return false;
        }
        std::array<typename Math::Vec3, 4> landmarks =
            pose_ops::IdentityLandmarks<Math>();
        unsigned restFlags = 0;
        if (ref.slot >= 0 &&
            host.inputs().restFrame(size_t(ref.slot), &landmarks, &restFlags) &&
            (restFlags & pose_ops::kPoseFrameValid) == 0) {
            landmarks = pose_ops::IdentityLandmarks<Math>();
        }
        return pose_ops::PointsToMatrix<Math>(landmarks, sources[k].points,
                                              out);
    };
    typename Math::Mat4 transform = Math::Identity4();
    if (sources.empty() || !mapOf(0, &transform)) {
        host.state().diagnose(
            facts.pathText +
            " could not resolve rigExec:transform; mover passed through");
        return {Outcome::Applied, kNoStepDetail};
    }
    if (sources.size() > 1) {
        typename Math::Mat4 space = Math::Identity4();
        if (!mapOf(1, &space)) {
            host.state().diagnose(
                facts.pathText +
                " could not resolve rigExec:transformSpace; mover passed "
                "through");
            return {Outcome::Applied, kNoStepDetail};
        }
        transform = pose_ops::MeasureInSpace<Math>(transform, space);
    }
    std::array<typename Math::Vec3, 4> candidatePoints = inputPoints;
    if (facts.radialBlend) {
        const typename Math::Mat4 partial =
            pose_ops::PartialTransform<Math>(transform, weight);
        for (size_t q = 0; q < 4; ++q) {
            candidatePoints[q] =
                pose_ops::TransformAffine3<Math>(partial, candidatePoints[q]);
        }
    } else {
        for (size_t q = 0; q < 4; ++q) {
            candidatePoints[q] = pose_ops::Add3<Math>(
                candidatePoints[q],
                pose_ops::Scale3<Math>(
                    pose_ops::Sub3<Math>(pose_ops::TransformAffine3<Math>(
                                             transform, candidatePoints[q]),
                                         candidatePoints[q]),
                    weight));
        }
    }
    return WriteConstraintCandidate(host, object, commitFacts, facts, state,
                                    candidatePoints, inputFlags, true);
}

// Aim: the shared prologue and the solve above. Mid-arm failures
// pass through with the same diagnostic on both sides; corrupt
// bindings are fatal, and only on the runtime. The caller routes
// Aim (and only Aim) here after the shared entry ran.
template <class Host>
StepResult
RunAimArm(Host &host, size_t object)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    PoseConstraintPrologueT<Host> prologue;
    bool proceed = false;
    const StepResult gathered =
        ResolveConstraintInputs(host, object, &prologue, &proceed);
    if (!proceed) {
        return gathered;
    }
    return RunAimSolve<Host>(host, object, prologue.commitFacts, prologue.facts,
                             std::move(prologue.sources), prologue.weight,
                             prologue.state);
}

// One IK/FK match's bound scalars: the envelope, the seven match
// inputs, the IK value, the limb calibration, and the rest offsets.
struct PoseIkFkFacts {
    double envelope = 1.0;
    double scalars[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double ikValue = 0.0;
    double scaleCalibration = 0.0;
    double upperOffset = 0.0;
    double lowerOffset = 0.0;
    bool stretchLimb = false;
};

// The IK/FK match arm: a limb's match values and presented frames.
// Presentation and values only; the commit stays abandoned. The
// limb solves run through the shared ikFkMatchKernel.h; the pole's
// space-switch parent stays a host call until spaces-family-lift
// unifies the switch.
template <class Host>
StepResult
RunIkFkArm(Host &host, size_t object)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoConstraint;
    using pose_detail::kPoseDetailNoVersion;
    PoseIkFkFacts match;
    if (!host.inputs().ikFkFacts(object, &match)) {
        return {Outcome::Invalid, kPoseDetailNoConstraint};
    }
    const PoseConstraintFactsT<Math> facts =
        host.inputs().constraintFacts(object);
    const PoseCommitFacts commitFacts = host.inputs().commitFacts(object);
    if (host.inputs().constraintSourceCount(object) != 10 ||
        commitFacts.sourceReads.count < 10) {
        return {Outcome::Invalid, kPoseDetailNoConstraint};
    }
    const double weight = std::isfinite(match.envelope) ? match.envelope : 0.0;
    host.state().setMatchValuesValid(object, false);
    host.state().setMatchShownMask(object, 0);
    const auto matrixOf = [&](const std::array<typename Math::Vec3, 4> &pts,
                              unsigned flags, typename Math::Mat4 *out) {
        return (flags & pose_ops::kPoseFrameValid) != 0 &&
               pose_ops::PointsToMatrix<Math>(
                   pose_ops::IdentityLandmarks<Math>(), pts, out);
    };
    typename Math::Mat4 input[10];
    typename Math::Mat4 rest[8];
    bool usable = true;
    for (size_t i = 0; usable && i < 10; ++i) {
        PoseSourceRef ref;
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        usable = host.inputs().constraintSourceRef(object, i, &ref) &&
                 ref.slot >= 0 &&
                 host.inputs().finFrame(commitFacts.sourceReads.data[i],
                                        &points, &flags) &&
                 matrixOf(points, flags, &input[i]);
    }
    const auto restOf = [&](int slot, typename Math::Mat4 *out) {
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        return slot >= 0 &&
               host.inputs().restFrame(size_t(slot), &points, &flags) &&
               matrixOf(points, flags, out);
    };
    for (size_t i = 0; usable && i < 3; ++i) {
        PoseSourceRef ref;
        usable = host.inputs().constraintSourceRef(object, i, &ref) &&
                 restOf(ref.slot, &rest[i]);
    }
    for (size_t i = 0; usable && i < 5; ++i) {
        PoseSourceRef ref;
        usable = host.inputs().constraintSourceRef(object, 5 + i, &ref) &&
                 restOf(ref.slot, &rest[3 + i]);
    }
    if (!usable) {
        host.state().diagnose(
            facts.pathText +
            " could not read its limb; constraint passed through");
        return {Outcome::Applied, kNoStepDetail};
    }
    RigExecIkFkMatchChannels channels[5];
    for (size_t i = 0; i < 5; ++i) {
        PoseSourceRef ref;
        if (!host.inputs().constraintSourceRef(object, 5 + i, &ref) ||
            !host.inputs().matchChannels(size_t(ref.slot), &channels[i])) {
            return {Outcome::Invalid, kPoseDetailNoConstraint};
        }
    }
    const auto raw = [&](const typename Math::Mat4 &m) { return &m[0][0]; };
    RigExecLimbStretch limb;
    limb.stretch = match.scalars[1];
    limb.softDistance = match.scalars[2];
    limb.upperScale = match.scalars[3];
    limb.lowerScale = match.scalars[4];
    limb.pin = match.scalars[5];
    limb.scaleCalibration = match.scaleCalibration;
    double space[16];
    const double *spaceMatrix = nullptr;
    if (facts.spaceSlot >= 0) {
        typename Math::Mat4 spaceRest = Math::Identity4();
        typename Math::Mat4 spacePosed = Math::Identity4();
        std::array<typename Math::Vec3, 4> restPoints = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned restFlags = 0;
        if (host.inputs().restFrame(size_t(facts.spaceSlot), &restPoints,
                                    &restFlags) &&
            matrixOf(restPoints, restFlags, &spaceRest) &&
            host.inputs().posedMatrix(size_t(facts.spaceSlot), &spacePosed) &&
            std::isfinite(spacePosed[3][0])) {
            const typename Math::Mat4 carry = pose_ops::Multiply4<Math>(
                pose_ops::Invert4Result<Math>(spaceRest), spacePosed);
            std::copy(&carry[0][0], &carry[0][0] + 16, space);
            spaceMatrix = space;
        }
    }
    double restOrigins[3][3];
    double restUpper = 0.0;
    double restLower = 0.0;
    double spaceFactor = 1.0;
    for (size_t i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
            restOrigins[i][k] = rest[i][3][k];
        }
    }
    RigExecIkFkMatchRestLengths(restOrigins, spaceMatrix, match.upperOffset,
                                match.lowerOffset, &restUpper, &restLower,
                                &spaceFactor);
    limb.softDistance *= spaceFactor;
    RigExecIkFkMatchIkInputs ikIn;
    for (size_t i = 0; i < 3; ++i) {
        ikIn.joints[i] = raw(input[i]);
    }
    ikIn.root = raw(input[3]);
    ikIn.effectorPosed = raw(input[4]);
    ikIn.ikPosed = raw(input[5]);
    ikIn.ik = channels[0];
    ikIn.twistRadians = match.scalars[6] * std::acos(-1.0) / 180.0;
    typename Math::Vec3 poleRest =
        pose_ops::Sub3<Math>(pose_ops::ExtractTranslation<Math>(rest[4]),
                             pose_ops::ExtractTranslation<Math>(rest[1]));
    ikIn.poleDistance = pose_ops::Length3<Math>(poleRest) * spaceFactor;
    ikIn.polePosed = raw(input[6]);
    ikIn.hasStretch = match.stretchLimb;
    ikIn.limb = limb;
    ikIn.restUpper = restUpper;
    ikIn.restLower = restLower;
    RigExecIkFkMatchIkResult ik;
    RigExecIkFkMatchIk(ikIn, &ik);
    RigExecIkFkMatchChannels pole = channels[1];
    double poleFrame[16];
    bool poleOk = false;
    if (ik.ok) {
        typename Math::Mat4 ikNew = Math::Identity4();
        std::copy(ik.ikFrame, ik.ikFrame + 16, &ikNew[0][0]);
        const typename Math::Mat4 moved = pose_ops::Multiply4<Math>(
            pose_ops::Invert4Result<Math>(input[5]), ikNew);
        const auto composed = ikFkMatchKernel::Compose(channels[1]);
        typename Math::Mat4 avars = Math::Identity4();
        std::copy(composed.begin(), composed.end(), &avars[0][0]);
        typename Math::Mat4 parent = pose_ops::Multiply4<Math>(
            pose_ops::Invert4Result<Math>(avars), input[6]);
        host.state().matchPoleParent(object, moved, &parent);
        poleOk =
            RigExecIkFkMatchPole(ik.poleTarget, raw(parent), &pole, poleFrame);
    }
    RigExecIkFkMatchFkInputs fkIn;
    typename Math::Mat4 offsets[3];
    for (size_t i = 0; i < 3; ++i) {
        fkIn.joints[i] = raw(input[i]);
        fkIn.fkPosed[i] = raw(input[7 + i]);
        fkIn.fk[i] = channels[2 + i];
        offsets[i] = pose_ops::Multiply4<Math>(
            rest[5 + i], pose_ops::Invert4Result<Math>(rest[i]));
        fkIn.offsets[i] = raw(offsets[i]);
    }
    fkIn.hasStretch = match.stretchLimb;
    fkIn.upperScale = limb.upperScale;
    fkIn.lowerScale = limb.lowerScale;
    fkIn.restUpper = restUpper;
    fkIn.restLower = restLower;
    RigExecIkFkMatchFkResult fk;
    RigExecIkFkMatchFk(fkIn, &fk);
    if (ik.ok && poleOk && fk.ok) {
        double *ikOut = nullptr;
        double *fkOut = nullptr;
        if (!host.state().matchValueOut(object, &ikOut, &fkOut)) {
            return {Outcome::Invalid, kPoseDetailNoConstraint};
        }
        RigExecIkFkMatchPack(ik, pole, fk, ikOut, fkOut);
        host.state().setMatchValuesValid(object, true);
    } else {
        host.state().diagnose(facts.pathText +
                              " could not match its limb's other half");
    }
    const auto show = [&](size_t k, const double *frame) {
        typename Math::Mat4 matrix = Math::Identity4();
        std::copy(frame, frame + 16, &matrix[0][0]);
        std::array<typename Math::Vec3, 4> points;
        unsigned flags = 0;
        pose_ops::FrameFromMatrix<Math>(matrix, &points, &flags);
        host.state().writeMatchShown(object, k, points, flags);
    };
    switch (
        RigExecIkFkMatchShownSide(match.scalars[0], match.ikValue, weight)) {
    case RigExecIkFkMatchShown::IkSide:
        if (ik.ok && poleOk) {
            show(0, ik.ikFrame);
            show(1, poleFrame);
        }
        break;
    case RigExecIkFkMatchShown::FkSide:
        if (fk.ok) {
            for (size_t i = 0; i < 3; ++i) {
                show(2 + i, fk.frames[i]);
            }
        }
        break;
    case RigExecIkFkMatchShown::None:
        break;
    }
    return {Outcome::Applied, kNoStepDetail};
}

// Absent and empty tables use defaults; a nonempty table must name every source.
template <class Value, class Read, class Diagnose>
inline bool
NormalizeConstraintArray(size_t channel, size_t count, size_t rawCount,
                         bool present, Read read, Diagnose diagnose,
                         std::vector<Value> *values)
{
    if (present && rawCount != 0 && rawCount != count) {
        diagnose(rawCount, count);
        return false;
    }
    values->assign(count, Value(channel == 0 || channel == 3 ? 1.0 : 0.0));
    if (present) {
        for (size_t k = 0; k < rawCount; ++k) {
            (*values)[k] = read(k);
        }
    }
    return true;
}

// Keep the source, optional offsets and pole channels in the same order on
// both hosts; decoding and diagnostic text remain with their adapters.
template <class Weights, class Offsets>
inline void
NormalizeConstraintTables(size_t sourceCount, size_t poleCount,
                          bool parentOffsets, bool readPole, Weights weights,
                          Offsets offsets, bool *ok, bool *poleOk)
{
    *ok = weights(0, sourceCount);
    if (parentOffsets) {
        *ok = *ok && offsets(1, sourceCount, true) &&
              offsets(2, sourceCount, true);
    } else {
        offsets(1, sourceCount, false);
        offsets(2, sourceCount, false);
    }
    if (readPole) {
        *poleOk = weights(3, poleCount);
    }
}

// One single-chain IK's bound inputs, copied: the chain slots, the
// effector and pole references, and the solve parameters. Small
// (chain-length) vectors; frames travel through the frame accessors.
struct PoseIkChainFacts {
    std::vector<int> targetSlots;
    int effector = -1;
    int effectorNative = -1;
    double stretch = 0.0;
    int ikMode = 0;
    bool useAnimatedTs = false;
    std::vector<char> ikRestLive;
    bool poleModeObject = false;
    std::vector<int> poleObjects;
};

// Gathers the chain, effector, pole and rests and atomically commits the
// result. Rest-derived preparation and the IK solve remain host-specific;
// both hosts honor preserveJointOrientation.
template <class Host>
StepResult
RunSingleChainIkArm(Host &host, size_t object,
                    const PoseCommitFacts &commitFacts,
                    const PoseConstraintFactsT<typename Host::Math> &facts,
                    double weight, PoseCommitState state)
{
    using Math = typename Host::Math;
    using pose_detail::kPoseDetailNoCandidate;
    using pose_detail::kPoseDetailNoChain;
    using pose_detail::kPoseDetailNoPole;
    using pose_detail::kPoseDetailNoRest;
    using pose_detail::kPoseDetailNoVersion;
    PoseIkChainFacts ik;
    if (!host.inputs().ikChainFacts(object, &ik)) {
        return {Outcome::Invalid, kPoseDetailNoChain};
    }
    if (commitFacts.targetReads.count != ik.targetSlots.size()) {
        return {Outcome::Invalid, kPoseDetailNoChain};
    }
    bool inputsValid = true;
    std::vector<typename Math::Frame> chain;
    chain.reserve(ik.targetSlots.size());
    for (size_t k = 0; k < ik.targetSlots.size(); ++k) {
        std::array<typename Math::Vec3, 4> points = {
            Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
            Math::Vec3Of(0, 0, 0)};
        unsigned flags = 0;
        if (!host.inputs().finFrame(commitFacts.targetReads.data[k], &points,
                                    &flags)) {
            return {Outcome::Invalid, kPoseDetailNoVersion};
        }
        chain.push_back(Math::MakeFrame(points, flags));
    }
    std::array<typename Math::Vec3, 4> effectorPoints = {
        Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
        Math::Vec3Of(0, 0, 0)};
    unsigned effectorFlags = 0;
    if (!host.inputs().resolveIkEffector(object, &effectorPoints,
                                         &effectorFlags)) {
        host.state().diagnose(
            facts.pathText +
            " could not resolve its effector; constraint passed through");
        inputsValid = false;
    }
    typename Math::Vec3 pole = Math::Vec3Of(0.0, 1.0, 0.0);
    double twistDegrees = 0.0;
    if (ik.ikMode == 0) {
        host.inputs().ikPoleVector(object, &pole, &twistDegrees);
    }
    if (inputsValid && ik.ikMode == 0 && ik.poleModeObject) {
        if (ik.poleObjects.empty()) {
            host.state().diagnose(
                facts.pathText +
                " uses object pole mode with no pole-vector objects; "
                "constraint passed through");
            inputsValid = false;
        }
        if (inputsValid && !facts.arrays.poleOk) {
            for (size_t d = 0; d < facts.arrays.poleDiagnosticCount; ++d) {
                host.state().diagnose(facts.arrays.poleDiagnostics[d]);
            }
            inputsValid = false;
        }
        typename Math::Vec3 polePoint = Math::Vec3Of(0, 0, 0);
        double total = 0.0;
        for (size_t k = 0; inputsValid && k < ik.poleObjects.size(); ++k) {
            if (!host.inputs().ikPoleBounds(object, k) ||
                k >= facts.arrays.poleWeightCount) {
                return {Outcome::Invalid, kPoseDetailNoPole};
            }
            std::array<typename Math::Vec3, 4> polePoints = {
                Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
                Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0)};
            unsigned poleFlags = 0;
            if (!host.inputs().resolveIkPole(object, k, &polePoints,
                                             &poleFlags) ||
                !std::isfinite(facts.arrays.poleWeights[k]) ||
                facts.arrays.poleWeights[k] < 0) {
                host.state().diagnose(
                    facts.pathText +
                    " has an invalid pole-vector source or weight; "
                    "constraint passed through");
                inputsValid = false;
                break;
            }
            polePoint = pose_ops::Add3<Math>(
                polePoint, pose_ops::Scale3<Math>(polePoints[0],
                                                  facts.arrays.poleWeights[k]));
            total += facts.arrays.poleWeights[k];
        }
        if (inputsValid && total <= 0) {
            host.state().diagnose(
                facts.pathText +
                " has zero total pole-vector weight; constraint passed "
                "through");
            inputsValid = false;
        }
        if (inputsValid) {
            pole = pose_ops::Scale3<Math>(polePoint, 1.0 / total);
        }
    }
    std::vector<typename Math::Frame> rests;
    rests.reserve(ik.targetSlots.size());
    if (inputsValid && !ik.useAnimatedTs) {
        for (size_t k = 0; k < ik.targetSlots.size(); ++k) {
            if (k < ik.ikRestLive.size() && ik.ikRestLive[k] &&
                k < chain.size()) {
                rests.push_back(chain[k]);
            } else {
                std::array<typename Math::Vec3, 4> points = {
                    Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0),
                    Math::Vec3Of(0, 0, 0), Math::Vec3Of(0, 0, 0)};
                unsigned flags = 0;
                if (ik.targetSlots[k] < 0 ||
                    !host.inputs().restFrame(size_t(ik.targetSlots[k]), &points,
                                             &flags)) {
                    return {Outcome::Invalid, kPoseDetailNoRest};
                }
                rests.push_back(Math::MakeFrame(points, flags));
            }
        }
    }
    std::vector<typename Math::Frame> solved;
    if (inputsValid) {
        typename Math::Frame effector =
            Math::MakeFrame(effectorPoints, effectorFlags);
        if (!host.state().solveIkChain(
                object, chain, rests, effector, ik.ikMode, ik.stretch, weight,
                pole, twistDegrees, ik.useAnimatedTs, &solved)) {
            host.state().diagnose(
                facts.pathText +
                " could not prepare rest-derived IK inputs; constraint "
                "passed through");
            inputsValid = false;
        }
    }
    if (inputsValid && (solved.size() != ik.targetSlots.size() ||
                        !std::all_of(solved.begin(), solved.end(),
                                     [](const typename Math::Frame &frame) {
                                         return FrameUsable<Math>(frame.points,
                                                                  frame.flags);
                                     }))) {
        host.state().diagnose(
            facts.pathText +
            " failed to solve its joint chain; constraint passed through "
            "atomically");
        inputsValid = false;
    }
    if (inputsValid) {
        const size_t slots = host.inputs().commitSlotCount(object);
        for (size_t k = 0; k < ik.targetSlots.size(); ++k) {
            size_t pos = slots;
            for (size_t p = 0; p < slots; ++p) {
                int slot = -1;
                if (!host.inputs().commitSlotAt(object, p, &slot)) {
                    return {Outcome::Invalid, kPoseDetailNoCandidate};
                }
                if (slot == ik.targetSlots[k]) {
                    pos = p;
                    break;
                }
            }
            if (pos >= slots) {
                continue;
            }
            if (pos >= state.framesCount || pos >= state.presentCount) {
                return {Outcome::Invalid, kPoseDetailNoCandidate};
            }
            host.state().writeCommitFrame(object, pos, solved[k].points,
                                          solved[k].flags);
            state.present[pos] = 1;
        }
        *state.abandoned = false;
    }
    return {Outcome::Applied, kNoStepDetail};
}

// One constraint step: the arm dispatch over the shared prologue.
// The caller ran the entry gates (abandoned/present, record gates,
// delta-absent); the commit fold stays the caller's too: split
// commits defer it to their CommitApply step.
template <class Host>
StepResult
RunConstraintStep(Host &host, const StepRef &step)
{
    static_assert(IsBoundHost<Host>::value, "pose Host is not bound");
    using pose_detail::kPoseDetailNoCommit;
    if (step.object < 0 ||
        size_t(step.object) >= host.inputs().commitObjectCount()) {
        return {Outcome::Invalid, kPoseDetailNoCommit};
    }
    const size_t object = size_t(step.object);
    const PoseConstraintFactsT<typename Host::Math> facts =
        host.inputs().constraintFacts(object);
    if (facts.ikFkMatch) {
        return RunIkFkArm(host, object);
    }
    if (facts.target < 0) {
        return {Outcome::Applied, kNoStepDetail};
    }
    if (facts.harmonics) {
        return RunHarmonicsArm(host, object);
    }
    if (!facts.enabled) {
        return {Outcome::Applied, kNoStepDetail};
    }
    PoseConstraintPrologueT<Host> prologue;
    bool proceed = false;
    const StepResult headed =
        ResolveConstraintHead(host, object, &prologue, &proceed);
    if (!proceed) {
        return headed;
    }
    if (prologue.facts.singleChainIk) {
        return RunSingleChainIkArm(host, object, prologue.commitFacts,
                                   prologue.facts, prologue.weight,
                                   prologue.state);
    }
    const StepResult gathered =
        ResolveConstraintSources(host, object, &prologue, &proceed);
    if (!proceed) {
        return gathered;
    }
    const PoseCommitFacts &commitFacts = prologue.commitFacts;
    const PoseConstraintFactsT<typename Host::Math> &solvedFacts =
        prologue.facts;
    switch (solvedFacts.code) {
    case kPoseConstraintMatrixMover:
        return RunMatrixSolve(host, object, commitFacts, solvedFacts,
                              prologue.sources, prologue.weight,
                              prologue.state);
    case kPoseConstraintPosition:
        return RunPositionSolve(host, object, commitFacts, solvedFacts,
                                prologue.sources, prologue.weight,
                                prologue.state);
    case kPoseConstraintRotation:
        return RunRotationSolve(host, object, commitFacts, solvedFacts,
                                prologue.sources, prologue.weight,
                                prologue.state, &prologue.carry,
                                prologue.hasCarry);
    case kPoseConstraintScale:
        return RunScaleSolve(host, object, commitFacts, solvedFacts,
                             prologue.sources, prologue.weight, prologue.state);
    case kPoseConstraintParent:
        return RunParentSolve(host, object, commitFacts, solvedFacts,
                              prologue.sources, prologue.weight, prologue.state,
                              &prologue.carry, prologue.hasCarry);
    case kPoseConstraintAim:
    case kPoseConstraintUnknown:
    default:
        return RunAimSolve(host, object, commitFacts, solvedFacts,
                           std::move(prologue.sources), prologue.weight,
                           prologue.state);
    }
}

// The pose family tag: an adapter's RunValueStep entry selects this
// family's dispatch through Host::EvalValueFamily.
struct PoseValueFamily {};

// The pose family's RunValueStep over the value-step kinds both
// adapters route here (step.h's StepRef; object/part are the
// adapters' own bindings). Steps always track ladder moves; the
// Build-time direct calls pass trackMoves=false instead.
template <class Host,
          class = std::enable_if_t<std::is_same<typename Host::EvalValueFamily,
                                                PoseValueFamily>::value>>
StepResult
RunValueStep(Host &host, const StepRef &step)
{
    switch (StepKind(step.kind)) {
    case StepKind::AvarInputs:
        return RunAvarInputsStep(host, step);
    case StepKind::SpaceCheckpoint:
        return RunSpaceCheckpointStep(host, step);
    case StepKind::RestCompose:
        return RunRestComposeStep(host, step, true);
    case StepKind::LadderCompose:
        return RunLadderComposeStep(host, step, true);
    case StepKind::CommitDelta:
        return RunCommitDeltaStep(host, step);
    case StepKind::PropagateChunk:
        return RunPropagateChunkStep(host, step);
    case StepKind::CommitApply:
        return RunCommitApplyStep(host, step);
    case StepKind::Constraint:
        return RunConstraintStep(host, step);
    default:
        return {Outcome::Invalid, kNoStepDetail};
    }
}

} // namespace eval
} // namespace rigExec

#endif // RIGEXEC_EVAL_CORE_POSE_H
