// The weighted-matrix movement: p' = q + w (T q - q), or with radial weights a
// fraction of T's rotation about its screw axis. One definition for native
// evaluation, the runtime and the ops. V is a float point (GfVec3f, RrVec3f), D
// its double twin and M a double 4x4 (GfMatrix4d, RrMat4d), whose operators
// share Gf's semantics.
#ifndef RIGEXEC_MATH_MATRIX_KERNEL_H
#define RIGEXEC_MATH_MATRIX_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>

// SSE2 where the target has it; RIGEXEC_DISABLE_SSE2 forces the scalar
// form, which the SSE form matches only to tolerance.
#if !defined(RIGEXEC_DISABLE_SSE2) &&                                          \
    (defined(__SSE2__) || (defined(_M_X64) && !defined(_M_ARM64EC)) ||         \
     (defined(_M_IX86_FP) && _M_IX86_FP >= 2))
#define RIGEXEC_MATH_HAS_SSE2 1
#include <emmintrin.h>
#include <xmmintrin.h>
#else
#define RIGEXEC_MATH_HAS_SSE2 0
#endif

namespace rigExec {
namespace matrixDetail {

template <class D, class V>
D
Wide(const V &p)
{
    return D(double(p[0]), double(p[1]), double(p[2]));
}

template <class V, class D>
V
Narrow(const D &p)
{
    return V(float(p[0]), float(p[1]), float(p[2]));
}

template <class D>
double
Dot(const D &a, const D &b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template <class D>
D
Cross(const D &a, const D &b)
{
    return D(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]);
}

#if RIGEXEC_MATH_HAS_SSE2
/// Row-vector convention: p' = x*row0 + y*row1 + z*row2 + row3.
template <class M>
void
Rows(const M &t, __m128 rows[4])
{
    for (int r = 0; r < 4; ++r)
        rows[r] =
            _mm_setr_ps(float(t[r][0]), float(t[r][1]), float(t[r][2]), 0.0f);
}

template <class V>
__m128
Load(const V &p)
{
    return _mm_setr_ps(p[0], p[1], p[2], 0.0f);
}

inline __m128
Move(__m128 q, const __m128 rows[4])
{
    return _mm_add_ps(
        _mm_add_ps(
            _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0)), rows[0]),
            _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1)), rows[1])),
        _mm_add_ps(
            _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2)), rows[2]),
            rows[3]));
}

template <class V>
V
Store(__m128 v)
{
    alignas(16) float result[4];
    _mm_store_ps(result, v);
    return V(result[0], result[1], result[2]);
}
#endif

} // namespace matrixDetail

/// q + w (T q - q) in double; w <= 0 is q and w >= 1 is T q.
template <class D, class M>
D
RigExecWeightedMatrixPoint(const D &point, const M &transform, double weight)
{
    if (weight <= 0.0)
        return point;
    const D moved = transform.TransformAffine(point);
    if (weight >= 1.0)
        return moved;
    return point + (moved - point) * weight;
}

/// The weighted movement over \p count points in float math (SSE2 when
/// built with it); null \p weights is 1 everywhere. in/out may alias.
template <class V, class D, class M>
void
RigExecApplyWeightedMatrixSimdKernel(const V *in, V *out, const float *weights,
                                     size_t count, const M &transform)
{
#if RIGEXEC_MATH_HAS_SSE2
    using namespace matrixDetail;
    __m128 rows[4];
    Rows(transform, rows);
    for (size_t i = 0; i < count; ++i) {
        const float weight = weights ? weights[i] : 1.0f;
        const __m128 q = Load(in[i]);
        const __m128 moved = Move(q, rows);
        __m128 blended;
        if (weight <= 0.0f) {
            blended = q;
        } else if (weight >= 1.0f) {
            blended = moved;
        } else {
            const __m128 w = _mm_set1_ps(weight);
            blended = _mm_add_ps(q, _mm_mul_ps(w, _mm_sub_ps(moved, q)));
        }
        out[i] = Store<V>(blended);
    }
#else
    for (size_t i = 0; i < count; ++i)
        out[i] = matrixDetail::Narrow<V>(
            RigExecWeightedMatrixPoint(matrixDetail::Wide<D>(in[i]), transform,
                                       weights ? weights[i] : 1.0f));
#endif
}

/// The array form fed a uniform field, its weight branch hoisted; bit-exact
/// against it.
template <class V, class D, class M>
void
RigExecApplyWeightedMatrixSimdKernel(const V *in, V *out, float weight,
                                     size_t count, const M &transform)
{
#if RIGEXEC_MATH_HAS_SSE2
    using namespace matrixDetail;
    if (weight <= 0.0f) {
        for (size_t i = 0; i < count; ++i)
            out[i] = in[i];
        return;
    }
    __m128 rows[4];
    Rows(transform, rows);
    if (weight >= 1.0f) {
        for (size_t i = 0; i < count; ++i)
            out[i] = Store<V>(Move(Load(in[i]), rows));
        return;
    }
    const __m128 w = _mm_set1_ps(weight);
    for (size_t i = 0; i < count; ++i) {
        const __m128 q = Load(in[i]);
        out[i] = Store<V>(
            _mm_add_ps(q, _mm_mul_ps(w, _mm_sub_ps(Move(q, rows), q))));
    }
#else
    for (size_t i = 0; i < count; ++i)
        out[i] = matrixDetail::Narrow<V>(RigExecWeightedMatrixPoint(
            matrixDetail::Wide<D>(in[i]), transform, weight));
#endif
}

/// A transform split into scale factors, screw axis and angle, translation
/// and the pivot it turns about, so several weights share one
/// decomposition.
template <class D> struct RigExecPartialDecompositionOf {
    D scale = D(1.0, 1.0, 1.0);
    D axis = D(1.0, 0.0, 0.0);
    double angle = 0.0;
    D translation = D(0.0, 0.0, 0.0);
    double along = 0.0;
    D pivot = D(0.0, 0.0, 0.0);
    bool smallAngle = true;
};

template <class D, class M>
RigExecPartialDecompositionOf<D>
RigExecDecomposePartialTransformKernel(const M &transform)
{
    RigExecPartialDecompositionOf<D> d;
    // Row lengths are the scale; dividing them out leaves the rotation.
    M basis = transform;
    basis.SetTranslateOnly(D(0.0, 0.0, 0.0));
    for (int axis = 0; axis < 3; ++axis) {
        const D row(basis[axis][0], basis[axis][1], basis[axis][2]);
        const double length = row.GetLength();
        if (length > 1e-12) {
            d.scale[axis] = length;
            const D unit = row / length;
            basis[axis][0] = unit[0];
            basis[axis][1] = unit[1];
            basis[axis][2] = unit[2];
        }
    }
    // A fraction of the angle about the pivot the transform turns about
    // (its screw axis), so a half-weighted point lands on the arc rather
    // than inside it. Matrix extraction may choose either quaternion sign;
    // fractional weights follow the shortest arc for both.
    const auto rotation = basis.ExtractRotation();
    d.axis = rotation.GetAxis().GetNormalized();
    d.angle = rotation.GetAngle();
    if (d.angle > 180.0)
        d.angle -= 360.0;
    d.translation = transform.ExtractTranslation();
    // Below about a tenth of a degree there is no meaningful axis; the
    // straight blend is correct and better conditioned.
    const double radians = d.angle * (3.14159265358979323846 / 180.0);
    d.smallAngle = std::abs(std::sin(0.5 * radians)) < 1e-4;
    if (d.smallAngle)
        return d;
    // The translation along the axis stays linear in w; the part across it
    // is (I - R) applied to the pivot and so names it.
    d.along = matrixDetail::Dot(d.translation, d.axis);
    const D across = d.translation - d.axis * d.along;
    const double half = 0.5 / std::tan(0.5 * radians);
    d.pivot = across * 0.5 + matrixDetail::Cross(d.axis, across) * half;
    return d;
}

/// The fractional matrix for a clamped \p w in (0, 1).
template <class M, class D>
M
RigExecApplyPartialDecompositionKernel(
    const RigExecPartialDecompositionOf<D> &d, double w)
{
    M scaled(1.0);
    scaled.SetScale(D(1.0 + (d.scale[0] - 1.0) * w,
                      1.0 + (d.scale[1] - 1.0) * w,
                      1.0 + (d.scale[2] - 1.0) * w));
    using Rotation = decltype(M(1.0).ExtractRotation());
    M out(1.0);
    out.SetRotate(Rotation(d.axis, d.angle * w));
    out = scaled * out;
    if (d.smallAngle) {
        out.SetTranslateOnly(d.translation * w);
        return out;
    }
    // p' = R_w (p - pivot) + pivot + w * along * axis.
    const M partial = out;
    out.SetTranslateOnly(d.pivot - partial.TransformDir(d.pivot) +
                         d.axis * (d.along * w));
    return out;
}

} // namespace rigExec

#endif // RIGEXEC_MATH_MATRIX_KERNEL_H
