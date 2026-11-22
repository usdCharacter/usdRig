// NURBS curve evaluation and the wire deformer over borrowed ranges, shared
// by the native (Gf) path, the zero-USD runtime (Rr) and the Wire op.
// Header-only and USD-free.
#ifndef RIGEXEC_MATH_WIRE_KERNEL_H
#define RIGEXEC_MATH_WIRE_KERNEL_H

#include "matrixKernel.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace rigExec {

/// The highest curve order the evaluators carry.
constexpr int kRigExecNurbsMaxOrder = 16;

/// A non-rational NURBS curve over borrowed control points and a knot
/// vector of pointCount + order entries.
template <class V> struct RigExecNurbsCurveSpan {
    const V *points = nullptr;
    size_t pointCount = 0;
    int order = 0;
    const double *knots = nullptr;
    size_t knotCount = 0;

    bool IsValid() const
    {
        return points && knots && order >= 2 &&
               order <= kRigExecNurbsMaxOrder && pointCount >= size_t(order) &&
               knotCount == pointCount + size_t(order) &&
               knots[size_t(order) - 1] < knots[pointCount];
    }

    double DomainStart() const
    {
        return knots[size_t(order) - 1];
    }
    double DomainEnd() const
    {
        return knots[pointCount];
    }

    /// Same order, point count and knots (by value, so a NaN knot differs).
    bool Matches(const RigExecNurbsCurveSpan &other) const
    {
        return order == other.order && pointCount == other.pointCount &&
               knotCount == other.knotCount &&
               std::equal(knots, knots + knotCount, other.knots);
    }

    /// The point at \p u, clamped to the domain, by de Boor in D. Valid
    /// curves only.
    template <class D> V Evaluate(double u) const
    {
        const double *k = knots;
        const int p = order - 1;
        const size_t n = pointCount;
        u = std::min(std::max(u, DomainStart()), DomainEnd());
        // The span: largest s in [p, n-1] with k[s] <= u < k[s+1]; the
        // domain end belongs to the last non-empty span.
        size_t s = size_t(p);
        while (s + 1 < n && k[s + 1] <= u)
            ++s;
        D d[kRigExecNurbsMaxOrder];
        const int count = std::min(order, kRigExecNurbsMaxOrder);
        for (int j = 0; j < count; ++j)
            d[j] = matrixDetail::Wide<D>(points[s - size_t(p) + size_t(j)]);
        for (int r = 1; r <= p && r < kRigExecNurbsMaxOrder; ++r) {
            for (int j = p; j >= r; --j) {
                const size_t i = s - size_t(p) + size_t(j);
                const double denom = k[i + size_t(p) + 1 - size_t(r)] - k[i];
                const double a = denom > 0.0 ? (u - k[i]) / denom : 0.0;
                d[j] = d[j - 1] * (1.0 - a) + d[j] * a;
            }
        }
        return matrixDetail::Narrow<V>(d[p]);
    }
};

/// 1 - smoothstep(0, dropoff, distance); 1 when dropoff <= 0.
inline double
RigExecWireFalloff(double distance, double dropoffDistance)
{
    if (dropoffDistance <= 0.0)
        return 1.0;
    const double s = std::min(std::max(distance / dropoffDistance, 0.0), 1.0);
    return 1.0 - s * s * (3.0 - 2.0 * s);
}

/// The wire's checks before it writes: both curves evaluable and alike and
/// one bind coordinate per point.
template <class V, class B>
bool
RigExecWireInputsAreUsableKernel(const RigExecNurbsCurveSpan<V> &rest,
                                 const RigExecNurbsCurveSpan<V> &posed,
                                 const B *bindCoords, size_t bindCount,
                                 size_t pointCount)
{
    return bindCoords && rest.IsValid() && posed.IsValid() &&
           rest.Matches(posed) && bindCount == pointCount;
}

/// Points [begin, end) of \p count, held at \p out[0, end - begin) and seeded
/// by the caller, move by f(d) * (C(u) - C0(u)) for bind (u, d).
/// \p restEvals, when given, holds C0(u) for every point. False, writing
/// nothing, on a shape mismatch.
template <class V, class D, class B>
bool
RigExecApplyWireGroupKernel(V *out, size_t begin, size_t end, size_t count,
                            const RigExecNurbsCurveSpan<V> &rest,
                            const RigExecNurbsCurveSpan<V> &posed,
                            const B *bindCoords, size_t bindCount,
                            double dropoffDistance, const V *restEvals,
                            size_t restEvalCount)
{
    if (!RigExecWireInputsAreUsableKernel(rest, posed, bindCoords, bindCount,
                                          count) ||
        (restEvals && restEvalCount != count))
        return false;
    end = std::min(end, count);
    if (begin < end && !out)
        return false;
    for (size_t i = begin; i < end; ++i) {
        const double u = bindCoords[i][0];
        const double f = RigExecWireFalloff(bindCoords[i][1], dropoffDistance);
        if (f <= 0.0)
            continue;
        const V restPoint =
            restEvals ? restEvals[i] : rest.template Evaluate<D>(u);
        const V delta = posed.template Evaluate<D>(u) - restPoint;
        out[i - begin] += delta * float(f);
    }
    return true;
}

/// A sparse wire as a fixed linear map from control point motion to point
/// motion: per control point, (position in the weight index list,
/// f(d) N_j(u)).
struct RigExecWireBasis {
    std::vector<std::vector<std::pair<uint32_t, float>>> byControlPoint;
    bool ascendingIndices = false;
};

/// The basis build's checks before it builds: the bind table one per mesh
/// point or one per index, and the curve layout.
template <class B>
bool
RigExecWireBasisInputsAreUsableKernel(const B *bindCoords, size_t bindCount,
                                      size_t meshPointCount, size_t indexCount,
                                      int order, const double *knots,
                                      size_t knotCount,
                                      size_t controlPointCount)
{
    const size_t n = controlPointCount;
    return bindCoords && order >= 1 && order <= kRigExecNurbsMaxOrder &&
           n >= size_t(order) && knotCount == n + size_t(order) &&
           knots[size_t(order - 1)] < knots[n] &&
           (bindCount == meshPointCount || bindCount == indexCount);
}

/// Builds \p basis for the points named by \p indices. False for an
/// invalid layout or an index outside the mesh.
template <class B>
bool
RigExecBuildWireBasisKernel(const B *bindCoords, size_t bindCount,
                            size_t meshPointCount, const int *indices,
                            size_t indexCount, int order, const double *knots,
                            size_t knotCount, size_t controlPointCount,
                            double dropoffDistance, RigExecWireBasis *basis)
{
    const size_t n = controlPointCount;
    if (!basis || (indexCount && !indices) ||
        !RigExecWireBasisInputsAreUsableKernel(bindCoords, bindCount,
                                               meshPointCount, indexCount,
                                               order, knots, knotCount, n))
        return false;
    const bool parallel = bindCount == indexCount;
    const int p = order - 1;
    const double u0 = knots[size_t(p)];
    const double u1 = knots[n];
    basis->byControlPoint.assign(n, {});
    basis->ascendingIndices = std::is_sorted(indices, indices + indexCount);
    double left[kRigExecNurbsMaxOrder], right[kRigExecNurbsMaxOrder],
        N[kRigExecNurbsMaxOrder];
    for (size_t k = 0; k < indexCount; ++k) {
        const int index = indices[k];
        if (index < 0 || size_t(index) >= meshPointCount)
            return false;
        const B &bind = bindCoords[parallel ? k : size_t(index)];
        const double f = RigExecWireFalloff(double(bind[1]), dropoffDistance);
        if (f <= 0.0)
            continue;
        const double u = std::min(std::max(double(bind[0]), u0), u1);
        // The span exactly as Evaluate finds it.
        size_t span = size_t(p);
        while (span + 1 < n && knots[span + 1] <= u)
            ++span;
        // The order nonzero basis functions at u (Piegl and Tiller, A2.2).
        N[0] = 1.0;
        for (int j = 1; j <= p; ++j) {
            left[j] = u - knots[span + 1 - size_t(j)];
            right[j] = knots[span + size_t(j)] - u;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
                const double denom = right[r + 1] + left[j - r];
                const double temp = denom != 0.0 ? N[r] / denom : 0.0;
                N[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            N[j] = saved;
        }
        for (int r = 0; r <= p; ++r) {
            const double c = f * N[r];
            if (c != 0.0)
                basis->byControlPoint[span - size_t(p) + size_t(r)]
                    .emplace_back(uint32_t(k), float(c));
        }
    }
    return true;
}

/// Applies \p basis to points [begin, end), held at \p out[0, end - begin)
/// and seeded by the caller: point indices[k] moves by weights[k] * sum_j
/// coefficient * (posed[j] - rest[j]). Control-point-major, so a point's
/// additions arrive in the same order whichever range it is applied in.
template <class V>
bool
RigExecApplyWireBasisGroupKernel(V *out, size_t begin, size_t end,
                                 const RigExecWireBasis &basis,
                                 const int *indices, size_t indexCount,
                                 const float *weights, size_t weightCount,
                                 const V *restControlPoints, size_t restCount,
                                 const V *posedControlPoints, size_t posedCount)
{
    const size_t n = basis.byControlPoint.size();
    if ((!out && begin < end) || restCount != n || posedCount != n ||
        indexCount != weightCount)
        return false;
    for (size_t j = 0; j < n; ++j) {
        const V delta = posedControlPoints[j] - restControlPoints[j];
        if (delta == V(0.0f))
            continue; // a control point at rest moves nothing
        const auto &entries = basis.byControlPoint[j];
        auto first = entries.begin(), last = entries.end();
        if (basis.ascendingIndices) {
            const auto before = [&](const auto &entry, size_t bound) {
                return indices[entry.first] < 0 ||
                       size_t(indices[entry.first]) < bound;
            };
            first = std::lower_bound(first, last, begin, before);
            last = std::lower_bound(first, last, end, before);
        }
        for (auto entry = first; entry != last; ++entry) {
            const auto [k, coefficient] = *entry;
            const int index = indices[k];
            if (index < 0 || size_t(index) < begin || size_t(index) >= end)
                continue;
            out[size_t(index) - begin] += delta * (coefficient * weights[k]);
        }
    }
    return true;
}

} // namespace rigExec

#endif
