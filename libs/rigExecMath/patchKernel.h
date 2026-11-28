#ifndef RIGEXEC_MATH_PATCH_KERNEL_H
#define RIGEXEC_MATH_PATCH_KERNEL_H

/// The patch mover's arithmetic: neutral UV points mixed toward weighted UV
/// shapes, contact pairs kept from crossing, and a tensor-product NURBS
/// patch evaluated at the result. One definition for the native mover and
/// the Patch op.

#include "rigExecMath/rangeRunner.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

/// A tensor-product NURBS surface patch as UsdGeomNurbsPatch authors one
/// (tensor-product basis evaluation per Piegl and Tiller, see
/// docs/references.md): uVertexCount * vVertexCount control points in
/// u-fastest order, optional rational pointWeights, orders (degree + 1) and
/// knot vectors of vertexCount + order entries in u and v, and open or
/// periodic/closed form per parameter direction. \p V3f is a float triple
/// with operator[] and a three-float constructor.
template <class V3f> struct RigExecNurbsPatchKernel {
    const V3f *points = nullptr;
    size_t pointCount = 0;
    const double *pointWeights = nullptr;
    size_t pointWeightCount = 0;
    int uVertexCount = 0;
    int vVertexCount = 0;
    int uOrder = 0;
    int vOrder = 0;
    const double *uKnots = nullptr;
    size_t uKnotCount = 0;
    const double *vKnots = nullptr;
    size_t vKnotCount = 0;
    bool uPeriodic = false;
    bool vPeriodic = false;

    /// True when the arrays and counts describe an evaluable NURBS patch.
    bool IsValid() const
    {
        if (!points || !uKnots || !vKnots || uOrder < 2 || uOrder > 16 ||
            vOrder < 2 || vOrder > 16 || uVertexCount < uOrder ||
            vVertexCount < vOrder) {
            return false;
        }
        const size_t expectedPoints =
            size_t(uVertexCount) * size_t(vVertexCount);
        if (pointCount != expectedPoints ||
            uKnotCount != size_t(uVertexCount) + size_t(uOrder) ||
            vKnotCount != size_t(vVertexCount) + size_t(vOrder)) {
            return false;
        }
        for (size_t i = 0; i < uKnotCount; ++i) {
            if (!std::isfinite(uKnots[i]) ||
                (i > 0 && uKnots[i] < uKnots[i - 1])) {
                return false;
            }
        }
        for (size_t i = 0; i < vKnotCount; ++i) {
            if (!std::isfinite(vKnots[i]) ||
                (i > 0 && vKnots[i] < vKnots[i - 1])) {
                return false;
            }
        }
        if (!(uKnots[size_t(uOrder) - 1] < uKnots[size_t(uVertexCount)]) ||
            !(vKnots[size_t(vOrder) - 1] < vKnots[size_t(vVertexCount)])) {
            return false;
        }
        for (size_t i = 0; i < pointCount; ++i) {
            const V3f &p = points[i];
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
                !std::isfinite(p[2])) {
                return false;
            }
        }
        if (pointWeightCount != 0) {
            if (!pointWeights || pointWeightCount != pointCount) {
                return false;
            }
            for (size_t i = 0; i < pointWeightCount; ++i) {
                if (!std::isfinite(pointWeights[i]) || pointWeights[i] <= 0.0) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Active parameter domains [knots[order-1], knots[vertexCount]].
    double UDomainStart() const
    {
        return uKnots[size_t(uOrder) - 1];
    }
    double UDomainEnd() const
    {
        return uKnots[size_t(uVertexCount)];
    }
    double VDomainStart() const
    {
        return vKnots[size_t(vOrder) - 1];
    }
    double VDomainEnd() const
    {
        return vKnots[size_t(vVertexCount)];
    }

    /// Evaluates the patch at normalized parameters (uNorm, vNorm) in [0, 1],
    /// clamping for open form and wrapping into [0, 1) for periodic/closed
    /// form before mapping to the active knot domain. Accumulates in double.
    V3f EvaluateNormalized(double uNorm, double vNorm) const
    {
        const auto mapParam = [](double norm, bool periodic, double d0,
                                 double d1) {
            double t = norm;
            if (periodic) {
                t = t - std::floor(t);
                if (t < 0.0 || t >= 1.0) {
                    t = 0.0;
                }
            } else {
                t = std::min(std::max(t, 0.0), 1.0);
            }
            return std::min(std::max(d0 + t * (d1 - d0), d0), d1);
        };
        const double u =
            mapParam(uNorm, uPeriodic, UDomainStart(), UDomainEnd());
        const double v =
            mapParam(vNorm, vPeriodic, VDomainStart(), VDomainEnd());

        const auto evalBasis = [](double param, int order, size_t vertexCount,
                                  const double *knots, size_t *spanOut,
                                  double *N) {
            const int degree = order - 1;
            size_t span = size_t(degree);
            while (span + 1 < vertexCount && knots[span + 1] <= param) {
                ++span;
            }
            *spanOut = span;
            double left[16], right[16];
            N[0] = 1.0;
            for (int j = 1; j <= degree; ++j) {
                left[j] = param - knots[span + 1 - size_t(j)];
                right[j] = knots[span + size_t(j)] - param;
                double saved = 0.0;
                for (int r = 0; r < j; ++r) {
                    const double denom = right[r + 1] + left[j - r];
                    const double temp = denom != 0.0 ? N[r] / denom : 0.0;
                    N[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                }
                N[j] = saved;
            }
        };

        size_t su = 0, sv = 0;
        double Nu[16], Nv[16];
        evalBasis(u, uOrder, size_t(uVertexCount), uKnots, &su, Nu);
        evalBasis(v, vOrder, size_t(vVertexCount), vKnots, &sv, Nv);

        const int pu = uOrder - 1;
        const int pv = vOrder - 1;
        const size_t nu = size_t(uVertexCount);
        double sum[3] = {0.0, 0.0, 0.0};
        if (pointWeightCount == 0) {
            for (int b = 0; b <= pv; ++b) {
                const size_t iv = sv - size_t(pv) + size_t(b);
                double row[3] = {0.0, 0.0, 0.0};
                for (int a = 0; a <= pu; ++a) {
                    const size_t iu = su - size_t(pu) + size_t(a);
                    const V3f &q = points[iv * nu + iu];
                    for (int c = 0; c < 3; ++c)
                        row[c] += double(q[c]) * Nu[a];
                }
                for (int c = 0; c < 3; ++c)
                    sum[c] += row[c] * Nv[b];
            }
            return V3f(float(sum[0]), float(sum[1]), float(sum[2]));
        }
        double wSum = 0.0;
        for (int b = 0; b <= pv; ++b) {
            const size_t iv = sv - size_t(pv) + size_t(b);
            double row[3] = {0.0, 0.0, 0.0};
            double wRow = 0.0;
            for (int a = 0; a <= pu; ++a) {
                const size_t iu = su - size_t(pu) + size_t(a);
                const size_t idx = iv * nu + iu;
                const double w = Nu[a] * pointWeights[idx];
                const V3f &q = points[idx];
                for (int c = 0; c < 3; ++c)
                    row[c] += double(q[c]) * w;
                wRow += w;
            }
            for (int c = 0; c < 3; ++c)
                sum[c] += row[c] * Nv[b];
            wSum += wRow * Nv[b];
        }
        if (!(wSum > 0.0)) {
            return V3f(0.0f, 0.0f, 0.0f);
        }
        const double inverse = 1.0 / wSum;
        return V3f(float(sum[0] * inverse), float(sum[1] * inverse),
                   float(sum[2] * inverse));
    }
};

/// Evaluates \p patch at normalized (u, v) = (uvPoints[i][0], uvPoints[i][1])
/// for each of the \p count points in \p points (or broadcasts uvPoints[0]
/// when uvPointCount == 1 and count is non-zero). Returns false, leaving
/// \p points untouched, when the patch is invalid, uvPoints is missing or
/// mismatched, or any UV or result is non-finite.
///
/// Each point is evaluated independently, so \p ranges may split the loop
/// across threads; the points are bitwise the same either way.
template <class V3f>
bool
RigExecApplyPatchKernel(V3f *points, size_t count,
                        const RigExecNurbsPatchKernel<V3f> &patch,
                        const V3f *uvPoints, size_t uvPointCount,
                        const RigExecRangeRunner *ranges = nullptr)
{
    if (!patch.IsValid()) {
        return false;
    }
    const size_t n = count;
    if (n == 0) {
        return uvPointCount == 0 ||
               (uvPoints && uvPointCount == 1 &&
                std::isfinite(uvPoints[0][0]) && std::isfinite(uvPoints[0][1]));
    }
    if (!points || !uvPoints || (uvPointCount != n && uvPointCount != 1)) {
        return false;
    }
    for (size_t i = 0; i < uvPointCount; ++i) {
        if (!std::isfinite(uvPoints[i][0]) || !std::isfinite(uvPoints[i][1])) {
            return false;
        }
    }
    const bool broadcast = uvPointCount == 1;
    V3f *const out = points;
    // UVs read from the storage being written depend on the order of the
    // writes: only the single loop reproduces them.
    const std::uintptr_t outBegin = reinterpret_cast<std::uintptr_t>(out);
    const std::uintptr_t outEnd = reinterpret_cast<std::uintptr_t>(out + n);
    const std::uintptr_t uvBegin = reinterpret_cast<std::uintptr_t>(uvPoints);
    const std::uintptr_t uvEnd =
        reinterpret_cast<std::uintptr_t>(uvPoints + uvPointCount);
    const bool overlaps = uvBegin < outEnd && outBegin < uvEnd;
    std::atomic<bool> finite(true);
    RigExecRunRanges(
        overlaps ? nullptr : ranges, n, 512, [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                if (!finite.load(std::memory_order_relaxed)) {
                    return;
                }
                const V3f &uv = uvPoints[broadcast ? 0 : i];
                const V3f evaluated =
                    patch.EvaluateNormalized(double(uv[0]), double(uv[1]));
                if (!std::isfinite(evaluated[0]) ||
                    !std::isfinite(evaluated[1]) ||
                    !std::isfinite(evaluated[2])) {
                    finite.store(false, std::memory_order_relaxed);
                    return;
                }
                out[i] = evaluated;
            }
        });
    return finite.load();
}

/// The neutral UV points mixed toward the shapes:
/// uv = neutral + sum_k w_k (shape_k - neutral), in UV space, accumulated in
/// double; only the XY components move. \p shapes holds \p shapeCount
/// arrays of \p count points each. False, \p uvPoints untouched, when a
/// weight is not finite.
template <class V3f>
bool
RigExecMixPatchShapes(const V3f *const *shapes, const double *weights,
                      size_t shapeCount, V3f *uvPoints, size_t count,
                      const RigExecRangeRunner *ranges = nullptr)
{
    for (size_t k = 0; k < shapeCount; ++k) {
        if (!std::isfinite(weights[k])) {
            return false;
        }
    }
    const std::vector<V3f> neutral(uvPoints, uvPoints + count);
    V3f *const mixed = uvPoints;
    // Each point mixes independently, so the loop may be split.
    RigExecRunRanges(ranges, count, 1024, [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            double u = double(neutral[i][0]), v = double(neutral[i][1]);
            for (size_t k = 0; k < shapeCount; ++k) {
                if (weights[k] == 0.0) {
                    continue;
                }
                u += weights[k] *
                     (double(shapes[k][i][0]) - double(neutral[i][0]));
                v += weights[k] *
                     (double(shapes[k][i][1]) - double(neutral[i][1]));
            }
            mixed[i] = V3f(float(u), float(v), neutral[i][2]);
        }
    });
    return true;
}

/// Why the contact lists are unusable for \p pointCount UV points, or empty
/// when they are usable: as long as each other, every index in range, and
/// no point named twice, so pairs do not interact.
inline std::string
RigExecPatchContactError(const int *lower, size_t lowerCount, const int *upper,
                         size_t upperCount, size_t pointCount)
{
    if (lowerCount != upperCount) {
        return "rigExec:uvContactLower holds " + std::to_string(lowerCount) +
               " indices; rigExec:uvContactUpper holds " +
               std::to_string(upperCount);
    }
    std::vector<int> named;
    named.reserve(2 * lowerCount);
    for (size_t k = 0; k < lowerCount; ++k) {
        for (const int index : {lower[k], upper[k]}) {
            if (index < 0 || size_t(index) >= pointCount) {
                return "UV contact " + std::to_string(k) + " names point " +
                       std::to_string(index) + "; rigExec:uvPoints has " +
                       std::to_string(pointCount);
            }
            named.push_back(index);
        }
    }
    std::sort(named.begin(), named.end());
    const auto twice = std::adjacent_find(named.begin(), named.end());
    if (twice != named.end()) {
        return "UV contacts name point " + std::to_string(*twice) +
               " more than once";
    }
    return std::string();
}

/// Contacts, after the shapes mix: point lower[k] never stands above point
/// upper[k] in v. A pair that would cross meets at the midpoint of its two
/// v values, so shapes that close past contact press together instead of
/// passing through. Only v moves; a pair meant to keep two curves apart
/// shares its u (one column of each curve), and then the curves cannot
/// cross between columns either. False when RigExecPatchContactError
/// reports.
template <class V3f>
bool
RigExecTouchPatchContacts(const int *lower, size_t lowerCount, const int *upper,
                          size_t upperCount, V3f *uvPoints, size_t count)
{
    if (lowerCount == 0 && upperCount == 0) {
        return true;
    }
    if (!RigExecPatchContactError(lower, lowerCount, upper, upperCount, count)
             .empty()) {
        return false;
    }
    for (size_t k = 0; k < lowerCount; ++k) {
        V3f &below = uvPoints[lower[k]];
        V3f &above = uvPoints[upper[k]];
        if (below[1] > above[1]) {
            const float mid =
                float(0.5 * (double(below[1]) + double(above[1])));
            below[1] = mid;
            above[1] = mid;
        }
    }
    return true;
}

} // namespace rigExec

#endif
