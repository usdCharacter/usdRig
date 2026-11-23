#ifndef RIGEXEC_MATH_DUALQUAT_KERNEL_H
#define RIGEXEC_MATH_DUALQUAT_KERNEL_H

/// The dual-quaternion skinning numerics, \1: the one numeric definition shared
/// by the Gf core (rigExecMath/dualQuat.h) and the runtime
/// (rigExecRuntime/geometry.cpp), so both paths agree bit for bit. Conventions,
/// the scale/shear policy and the degenerate-case rules are documented at the
/// host's instantiations; the method reference is Kavan et al. (2008),
/// https://users.cs.utah.edu/~ladislav/dq/index.html.
///
/// The host hook RigExecDualQuatMath<Point, Double> names the affine types
/// and conversions the polar split reads, following the <Point, Double>
/// convention of latticeKernel.h and affineFrameKernel.h: Point is the
/// geometry's point type, Double its affine double-precision counterpart.
/// Gf form: dualQuat.h. Runtime form: rigExecRuntime/affineMath.h.

#include "frameParamsKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rigExec {

/// Tolerance used for orthonormality checks and for declaring a blended
/// real part too short to normalise.
constexpr double RigExecDualQuatEpsilon = 1e-9;

/// The affine math one dual-quaternion instantiation runs on: the host's
/// matrix, matrix3 and quaternion types, the four-point frame conversions
/// the polar split reads, and the matrix accessors Gf supplies as members
/// that a mirror has to spell itself. The host specializes it; every member
/// is called exactly as written in the kernels below.
template <class Point, class Double> struct RigExecDualQuatMath;

/// A dual quaternion real + eps*dual, defaulting to the identity motion.
template <class Point, class Double> struct RigExecDualQuatKernelT {
    using Math = RigExecDualQuatMath<Point, Double>;
    using PointType = Point;
    using Vector = Double;
    using Mat4 = typename Math::Mat4;
    using Mat3 = typename Math::Mat3;
    using Quat = typename Math::Quat;

    Quat real{1.0}; ///< rotation (unit quaternion)
    Quat dual{0.0}; ///< 1/2 * translation * real

    RigExecDualQuatKernelT() = default;
    RigExecDualQuatKernelT(const Quat &r, const Quat &d) : real(r), dual(d)
    {
    }
};

/// One influence split into rigid motion plus pre-rotation stretch.
template <class Point, class Double> struct RigExecScaledDualQuatKernelT {
    using Math = RigExecDualQuatMath<Point, Double>;
    using DualQuat = RigExecDualQuatKernelT<Point, Double>;
    using PointType = Point;
    using Vector = Double;
    using Mat4 = typename Math::Mat4;
    using Mat3 = typename Math::Mat3;

    DualQuat rigid;      ///< R_i and t_i
    Mat3 stretch{1.0};   ///< S_i, applied before rigid
    bool isRigid = true; ///< stretch is the identity
};

/// Normalises in place; declared before the blends that use it.
template <class DualQuat> bool RigExecDualQuatNormalizeKernel(DualQuat *dq);

namespace dualQuatDetail {

template <class Quat>
bool
QuatIsFinite(const Quat &q)
{
    const auto &v = q.GetImaginary();
    return std::isfinite(q.GetReal()) && std::isfinite(v[0]) &&
           std::isfinite(v[1]) && std::isfinite(v[2]);
}

// Row-vector convention: rows of the 3x3 part are the images of the basis
// vectors. Orthonormal with positive determinant means a proper rotation.
template <class Point, class Double>
bool
IsProperRotation(const typename RigExecDualQuatMath<Point, Double>::Mat4 &m,
                 double tolerance)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    const Double r0(m[0][0], m[0][1], m[0][2]);
    const Double r1(m[1][0], m[1][1], m[1][2]);
    const Double r2(m[2][0], m[2][1], m[2][2]);
    const double gram[6] = {
        Math::Dot(r0, r0) - 1.0, Math::Dot(r1, r1) - 1.0,
        Math::Dot(r2, r2) - 1.0, Math::Dot(r0, r1),
        Math::Dot(r0, r2),       Math::Dot(r1, r2),
    };
    for (const double g : gram) {
        if (!(std::abs(g) <= tolerance)) {
            return false;
        }
    }
    return Math::Dot(Math::Cross(r0, r1), r2) > 0.0;
}

// Shared by the rigid-only and scale-aware conversions so that a rigid input
// yields the same rotation quaternion, bit for bit, on both paths. Returns
// the rigid flag; for non-rigid input, when \p stretch is supplied, it
// receives S = L * R^T (row-vector, symmetrised), or the raw linear part
// when the input is singular (rotation identity).
template <class Point, class Double>
bool
DecomposeMatrix(const typename RigExecDualQuatMath<Point, Double>::Mat4 &matrix,
                typename RigExecDualQuatMath<Point, Double>::Quat *rotation,
                typename RigExecDualQuatMath<Point, Double>::Mat3 *stretch)
{
    using Math = RigExecDualQuatMath<Point, Double>;
    if (IsProperRotation<Point, Double>(matrix, RigExecDualQuatEpsilon)) {
        *rotation = matrix.ExtractRotationQuat();
        if (stretch) {
            stretch->SetIdentity();
        }
        return true;
    }

    const std::array<Double, 4> unitRest = {Double(0, 0, 0), Double(1, 0, 0),
                                            Double(0, 1, 0), Double(0, 0, 1)};
    RigExecTransformParamsKernelT<Point, Double> params;
    const bool nonSingular = RigExecPointsToParamsKernel<Point, Double>(
        unitRest, Math::MatrixToPoints(unitRest, matrix), /*reflectionAxis=*/2,
        &params);
    *rotation = nonSingular ? params.rotation : typename Math::Quat(1.0);

    if (stretch) {
        const typename Math::Mat3 linear = matrix.ExtractRotationMatrix();
        if (!nonSingular) {
            *stretch = linear;
        } else {
            // L = S * R  =>  S = L * R^T; symmetrise away rounding.
            typename Math::Mat3 rowR;
            Math::SetRotate(&rowR, *rotation);
            const typename Math::Mat3 s = linear * rowR.GetTranspose();
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    (*stretch)[i][j] = 0.5 * (s[i][j] + s[j][i]);
                }
            }
        }
    }
    return false;
}

// Row-vector product p * S for the 3x3 stretch.
template <class Point, class Double>
Double
MulRow(const Double &p,
       const typename RigExecDualQuatMath<Point, Double>::Mat3 &s)
{
    return Double(p[0] * s[0][0] + p[1] * s[1][0] + p[2] * s[2][0],
                  p[0] * s[0][1] + p[1] * s[1][1] + p[2] * s[2][1],
                  p[0] * s[0][2] + p[1] * s[1][2] + p[2] * s[2][2]);
}

// Accumulates sum(w_i * dq_i) with shortest-arc sign correction against the
// first non-zero-weight input, then normalises once. Shared by both public
// blend overloads through the Get(i) / Weight(i) accessors.
template <class DualQuat, class GetDq, class GetWeight>
bool
Blend(size_t count, GetDq getDq, GetWeight getWeight, DualQuat *result)
{
    *result = DualQuat();

    typename DualQuat::Quat realSum(0.0);
    typename DualQuat::Quat dualSum(0.0);
    typename DualQuat::Quat reference(0.0);
    bool haveReference = false;
    double absWeightSum = 0.0;

    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (!std::isfinite(w)) {
            return false;
        }
        if (w == 0.0) {
            continue;
        }
        const DualQuat *dq = getDq(i);
        if (!dq) {
            return false;
        }
        double sign = 1.0;
        if (!haveReference) {
            reference = dq->real;
            haveReference = true;
        } else if (DualQuat::Math::QuatDot(dq->real, reference) < 0.0) {
            sign = -1.0;
        }
        const double sw = sign * w;
        realSum += dq->real * sw;
        dualSum += dq->dual * sw;
        absWeightSum += std::abs(w);
    }

    if (!haveReference) {
        return false;
    }
    result->real = realSum;
    result->dual = dualSum;
    const double norm = realSum.GetLength();
    if (!(norm > RigExecDualQuatEpsilon * std::max(1.0, absWeightSum))) {
        *result = DualQuat();
        return false;
    }
    return RigExecDualQuatNormalizeKernel(result);
}

// The stretch half of the scale-aware blend, run after the rigid blend has
// validated every weight (finite) and every index, and found at least one
// non-zero weight. Shared by both public overloads through the same
// Get(i) / Weight(i) accessor pattern as Blend.
template <class ScaledDualQuat, class GetSdq, class GetWeight>
bool
BlendStretch(size_t count, GetSdq getSdq, GetWeight getWeight,
             ScaledDualQuat *result)
{
    // Two passes: the first is scalar work only, so a blend of rigid
    // inputs (the common skinning case) never touches the 3x3 stretches.
    bool allRigid = true;
    double weightSum = 0.0;
    double absWeightSum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (w == 0.0) {
            continue;
        }
        weightSum += w;
        absWeightSum += std::abs(w);
        if (!getSdq(i)->isRigid) {
            allRigid = false;
        }
    }
    if (allRigid) {
        // Exact identity stretch: nothing to normalise, nothing dropped.
        return true;
    }
    if (!(std::abs(weightSum) >
          RigExecDualQuatEpsilon * std::max(1.0, absWeightSum))) {
        *result = ScaledDualQuat();
        return false;
    }
    typename ScaledDualQuat::Mat3 stretchSum(0.0);
    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (w == 0.0) {
            continue;
        }
        stretchSum += getSdq(i)->stretch * w;
    }
    result->stretch = stretchSum * (1.0 / weightSum);
    result->isRigid = false;
    return true;
}

} // namespace dualQuatDetail

template <class DualQuat>
bool
RigExecDualQuatNormalizeKernel(DualQuat *dq)
{
    const double norm = dq->real.GetLength();
    if (!(norm > RigExecDualQuatEpsilon) || !std::isfinite(norm) ||
        !dualQuatDetail::QuatIsFinite(dq->dual)) {
        *dq = DualQuat();
        return false;
    }
    const double inv = 1.0 / norm;
    dq->real *= inv;
    dq->dual *= inv;
    // Project out the component of the dual part along the (now unit) real
    // part so that dot(real, dual) == 0. This changes only the scalar part
    // of 2 * dual * conj(real), i.e. it does not move the translation.
    dq->dual -= dq->real * DualQuat::Math::QuatDot(dq->real, dq->dual);
    return true;
}

template <class Point, class Double>
RigExecDualQuatKernelT<Point, Double>
RigExecDualQuatFromRotationTranslationKernel(
    const typename RigExecDualQuatMath<Point, Double>::Quat &rotation,
    const Double &translation)
{
    RigExecDualQuatKernelT<Point, Double> dq;
    dq.real = rotation.GetNormalized();
    // qd = 1/2 * t * qr, with t a pure quaternion.
    using Quat = typename RigExecDualQuatMath<Point, Double>::Quat;
    dq.dual = (Quat(0.0, translation) * dq.real) * 0.5;
    return dq;
}

template <class Point, class Double>
RigExecDualQuatKernelT<Point, Double>
RigExecDualQuatFromMatrixKernel(
    const typename RigExecDualQuatMath<Point, Double>::Mat4 &matrix,
    bool *isRigid = nullptr)
{
    typename RigExecDualQuatMath<Point, Double>::Quat rotation(1.0);
    const bool rigid = dualQuatDetail::DecomposeMatrix<Point, Double>(
        matrix, &rotation, nullptr);
    if (isRigid) {
        *isRigid = rigid;
    }
    return RigExecDualQuatFromRotationTranslationKernel<Point, Double>(
        rotation, matrix.ExtractTranslation());
}

template <class DualQuat>
typename DualQuat::Mat4
RigExecDualQuatToMatrixKernel(const DualQuat &dq)
{
    typename DualQuat::Mat4 m;
    m.SetRotate(dq.real);
    m.SetTranslateOnly(RigExecDualQuatTranslationKernel(dq));
    return m;
}

template <class DualQuat>
typename DualQuat::Vector
RigExecDualQuatTranslationKernel(const DualQuat &dq)
{
    // t = 2 * qd * conj(qr). With qr = (w, v) and qd = (dw, dv) the vector
    // part of qd * conj(qr) is w*dv - dw*v + v x dv; the scalar part is
    // dropped, which is also what makes a dual part with a stray component
    // along the real part harmless here.
    using Math = typename DualQuat::Math;
    const double w = dq.real.GetReal();
    const auto &v = dq.real.GetImaginary();
    const double dw = dq.dual.GetReal();
    const auto &dv = dq.dual.GetImaginary();
    return (dv * w - v * dw + Math::Cross(v, dv)) * 2.0;
}

template <class DualQuat>
bool
RigExecDualQuatIsUnitKernel(const DualQuat &dq, double tolerance)
{
    using Math = typename DualQuat::Math;
    if (!dualQuatDetail::QuatIsFinite(dq.real) ||
        !dualQuatDetail::QuatIsFinite(dq.dual)) {
        return false;
    }
    return std::abs(dq.real.GetLength() - 1.0) <= tolerance &&
           std::abs(Math::QuatDot(dq.real, dq.dual)) <= tolerance;
}

template <class Point, class Double>
RigExecScaledDualQuatKernelT<Point, Double>
RigExecScaledDualQuatFromMatrixKernel(
    const typename RigExecDualQuatMath<Point, Double>::Mat4 &matrix)
{
    RigExecScaledDualQuatKernelT<Point, Double> sdq;
    typename RigExecDualQuatMath<Point, Double>::Quat rotation(1.0);
    sdq.isRigid = dualQuatDetail::DecomposeMatrix<Point, Double>(
        matrix, &rotation, &sdq.stretch);
    sdq.rigid = RigExecDualQuatFromRotationTranslationKernel<Point, Double>(
        rotation, matrix.ExtractTranslation());
    return sdq;
}

template <class ScaledDualQuat>
typename ScaledDualQuat::Mat4
RigExecScaledDualQuatToMatrixKernel(const ScaledDualQuat &sdq)
{
    const typename ScaledDualQuat::Mat4 rigid =
        RigExecDualQuatToMatrixKernel(sdq.rigid);
    if (sdq.isRigid) {
        return rigid;
    }
    // [S | 0] * [R | t] = [S * R | t] in row-vector form.
    return typename ScaledDualQuat::Mat4(
               sdq.stretch, typename ScaledDualQuat::Vector(0.0, 0.0, 0.0)) *
           rigid;
}

/// Accessor form: influences arrive as GetDq(i) / GetWeight(i), so a caller
/// whose indices need validating (the palette form) and a caller reading an
/// array share one accumulation.
template <class DualQuat, class GetDq, class GetWeight>
bool
RigExecBlendDualQuatsKernel(size_t count, GetDq getDq, GetWeight getWeight,
                            DualQuat *result)
{
    return dualQuatDetail::Blend<DualQuat>(count, getDq, getWeight, result);
}

template <class Point, class Double>
bool
RigExecBlendDualQuatsKernel(const RigExecDualQuatKernelT<Point, Double> *dqs,
                            const double *weights, size_t count,
                            RigExecDualQuatKernelT<Point, Double> *result)
{
    if (count > 0 && (!dqs || !weights)) {
        *result = RigExecDualQuatKernelT<Point, Double>();
        return false;
    }
    return RigExecBlendDualQuatsKernel(
        count, [dqs](size_t i) { return &dqs[i]; },
        [weights](size_t i) { return weights[i]; }, result);
}

template <class Point, class Double>
bool
RigExecBlendDualQuatsKernel(
    const RigExecDualQuatKernelT<Point, Double> *palette, size_t paletteSize,
    const int *jointIndices, const float *jointWeights, size_t count,
    RigExecDualQuatKernelT<Point, Double> *result)
{
    using DualQuat = RigExecDualQuatKernelT<Point, Double>;
    if (count > 0 && (!palette || !jointIndices || !jointWeights)) {
        *result = DualQuat();
        return false;
    }
    return RigExecBlendDualQuatsKernel(
        count,
        [palette, paletteSize, jointIndices](size_t i) -> const DualQuat * {
            const int index = jointIndices[i];
            if (index < 0 || static_cast<size_t>(index) >= paletteSize) {
                return nullptr;
            }
            return &palette[index];
        },
        [jointWeights](size_t i) {
            return static_cast<double>(jointWeights[i]);
        },
        result);
}

/// Accessor form of the scale-aware blend: a null record is a missing
/// influence, rejected (false, identity) unless its weight is exactly zero.
template <class ScaledDualQuat, class GetSdq, class GetWeight>
bool
RigExecBlendScaledDualQuatsKernel(size_t count, GetSdq getSdq,
                                  GetWeight getWeight, ScaledDualQuat *result)
{
    *result = ScaledDualQuat();
    auto getRigid = [&getSdq](size_t i) -> decltype(&getSdq(i)->rigid) {
        auto *sdq = getSdq(i);
        if (!sdq) {
            return nullptr;
        }
        return &sdq->rigid;
    };
    if (!RigExecBlendDualQuatsKernel(count, getRigid, getWeight,
                                     &result->rigid)) {
        return false;
    }
    // Every non-zero-weight record was validated by the rigid blend.
    return dualQuatDetail::BlendStretch<ScaledDualQuat>(count, getSdq,
                                                        getWeight, result);
}

template <class Point, class Double>
bool
RigExecBlendScaledDualQuatsKernel(
    const RigExecScaledDualQuatKernelT<Point, Double> *sdqs,
    const double *weights, size_t count,
    RigExecScaledDualQuatKernelT<Point, Double> *result)
{
    using ScaledDualQuat = RigExecScaledDualQuatKernelT<Point, Double>;
    if (count > 0 && (!sdqs || !weights)) {
        *result = ScaledDualQuat();
        return false;
    }
    return RigExecBlendScaledDualQuatsKernel(
        count, [sdqs](size_t i) { return &sdqs[i]; },
        [weights](size_t i) { return weights[i]; }, result);
}

template <class Point, class Double>
bool
RigExecBlendScaledDualQuatsKernel(
    const RigExecScaledDualQuatKernelT<Point, Double> *palette,
    size_t paletteSize, const int *indices, const double *weights, size_t count,
    RigExecScaledDualQuatKernelT<Point, Double> *result)
{
    using ScaledDualQuat = RigExecScaledDualQuatKernelT<Point, Double>;
    if (count > 0 && (!palette || !indices || !weights)) {
        *result = ScaledDualQuat();
        return false;
    }
    auto getSdq = [palette, paletteSize,
                   indices](size_t i) -> const ScaledDualQuat * {
        const int index = indices[i];
        if (index < 0 || static_cast<size_t>(index) >= paletteSize) {
            return nullptr;
        }
        return &palette[index];
    };
    auto getWeight = [weights](size_t i) { return weights[i]; };
    return RigExecBlendScaledDualQuatsKernel(count, getSdq, getWeight, result);
}

template <class Point, class Double>
Double
RigExecDualQuatRotateVectorKernel(
    const RigExecDualQuatKernelT<Point, Double> &dq, const Double &vector)
{
    // Rodrigues form of q v q*: v + 2 (w (u x v) + u x (u x v)).
    using Math = RigExecDualQuatMath<Point, Double>;
    const double w = dq.real.GetReal();
    const auto &u = dq.real.GetImaginary();
    const Double c = Math::Cross(u, vector);
    return vector + (c * w + Math::Cross(u, c)) * 2.0;
}

template <class DualQuat>
typename DualQuat::Vector
RigExecDualQuatTransformPointKernel(const DualQuat &dq,
                                    const typename DualQuat::Vector &point)
{
    return RigExecDualQuatRotateVectorKernel(dq, point) +
           RigExecDualQuatTranslationKernel(dq);
}

template <class ScaledDualQuat>
typename ScaledDualQuat::Vector
RigExecScaledDualQuatTransformPointKernel(
    const ScaledDualQuat &sdq, const typename ScaledDualQuat::Vector &point)
{
    using Point = typename ScaledDualQuat::PointType;
    using Double = typename ScaledDualQuat::Vector;
    if (sdq.isRigid) {
        return RigExecDualQuatTransformPointKernel(sdq.rigid, point);
    }
    return RigExecDualQuatTransformPointKernel(
        sdq.rigid, dualQuatDetail::MulRow<Point, Double>(point, sdq.stretch));
}

// The skinning gather, palette and apply over the trait above: the one
// numeric definition the Gf wrappers in solvers.cpp and the runtime skin
// path both run, so the two front ends blend the same influences bit for
// bit. A layout names Weight(point, slot), indices, elementSize,
// transformCount and pointCount, as RigExecSkinLayout and its runtime
// mirror do.

/// Per point: the largest-weight slot first (the blend's sign-correction
/// reference), the other non-zero slots in authored order, then the
/// identity entry at transformCount with the weight complement when it is
/// not exactly zero. The total accumulates in double in slot order, as the
/// linear blend accumulates it, so both kernels see the same complement.
template <class Layout>
void
RigExecGatherDualQuatInfluencesKernel(const Layout &layout, size_t point,
                                      std::vector<int> *indices,
                                      std::vector<double> *weights)
{
    indices->clear();
    weights->clear();
    size_t pivot = 0;
    float pivotWeight = -1.0f;
    double total = 0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const float weight = layout.Weight(point, k);
        if (pivotWeight < weight) {
            pivotWeight = weight;
            pivot = k;
        }
        total += weight;
    }
    const auto push = [&](size_t k) {
        const double weight = layout.Weight(point, k);
        if (weight != 0) {
            indices->push_back(layout.indices[point * layout.elementSize + k]);
            weights->push_back(weight);
        }
    };
    push(pivot);
    for (size_t k = 0; k < layout.elementSize; ++k) {
        if (k != pivot)
            push(k);
    }
    const double complement = 1.0 - total;
    if (complement != 0) {
        indices->push_back(int(layout.transformCount));
        weights->push_back(complement);
    }
}

/// One split per influence in table order, then the trailing identity
/// entry that carries the weight complement.
template <class Point, class Double>
std::vector<RigExecScaledDualQuatKernelT<Point, Double>>
RigExecSkinDualQuatPaletteKernel(
    const typename RigExecDualQuatMath<Point, Double>::Mat4 *transforms,
    size_t transformCount)
{
    using ScaledDualQuat = RigExecScaledDualQuatKernelT<Point, Double>;
    std::vector<ScaledDualQuat> palette;
    palette.reserve(transformCount + 1);
    for (size_t t = 0; t < transformCount; ++t) {
        palette.push_back(RigExecScaledDualQuatFromMatrixKernel<Point, Double>(
            transforms[t]));
    }
    palette.emplace_back();
    return palette;
}

/// One point of the scale-aware blend against \p palette. False leaves
/// \p out unspecified when the blend is degenerate.
template <class Point, class Double, class Layout>
bool
RigExecApplyDualQuatSkinPointKernel(
    const Double &point, const Layout &layout, size_t i,
    const RigExecScaledDualQuatKernelT<Point, Double> *palette,
    size_t paletteSize, Double *out)
{
    using ScaledDualQuat = RigExecScaledDualQuatKernelT<Point, Double>;
    std::vector<int> indices;
    std::vector<double> weights;
    indices.reserve(layout.elementSize + 1);
    weights.reserve(layout.elementSize + 1);
    RigExecGatherDualQuatInfluencesKernel(layout, i, &indices, &weights);
    ScaledDualQuat blend;
    if (!RigExecBlendScaledDualQuatsKernel<Point, Double>(
            palette, paletteSize, indices.data(), weights.data(),
            indices.size(), &blend)) {
        return false;
    }
    *out = RigExecScaledDualQuatTransformPointKernel(blend, point);
    return true;
}

/// The array form over \p layout.pointCount points; in/out may alias
/// because point i is widened before it is written.
template <class Point, class Double, class Layout>
bool
RigExecApplyDualQuatSkinKernel(
    const Point *in, Point *out, const Layout &layout,
    const RigExecScaledDualQuatKernelT<Point, Double> *palette,
    size_t paletteSize)
{
    for (size_t i = 0; i < layout.pointCount; ++i) {
        Double moved;
        if (!RigExecApplyDualQuatSkinPointKernel<Point, Double>(
                Double(double(in[i][0]), double(in[i][1]), double(in[i][2])),
                layout, i, palette, paletteSize, &moved)) {
            return false;
        }
        out[i] = Point(float(moved[0]), float(moved[1]), float(moved[2]));
    }
    return true;
}

} // namespace rigExec

#endif // RIGEXEC_MATH_DUALQUAT_KERNEL_H
