// Wire-basis numerics shared by the USD-side geometry kernels and the
// zero-USD runtime. Header-only and USD-free: the USD evaluators (GfVec3f,
// GfVec3d, GfVec2f) and the runtime (RrVec3f, RrVec3d, RrVec2f) instantiate
// one definition of each helper. A host keeps its own curve, basis and bind
// records; they share the points/order/knots and byControlPoint/
// ascendingIndices shapes the helpers read.
// Method reference: Piegl and Tiller, *The NURBS Book* (1997), de Boor's
// algorithm (A2.1) and the basis-function row (A2.2); see docs/references.md.
#ifndef RIGEXEC_MATH_WIRE_BASIS_KERNEL_H
#define RIGEXEC_MATH_WIRE_BASIS_KERNEL_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

// The largest curve order a wire kernel accepts.
constexpr int kRigExecWireCurveMaxOrder = 16;

// The smoothstep falloff a bind's distance from the curve carries.
inline double
RigExecWireDropoffKernel(double distance, double dropoffDistance)
{
    if (dropoffDistance <= 0.0) {
        return 1.0;
    }
    const double s = std::min(std::max(distance / dropoffDistance, 0.0), 1.0);
    return 1.0 - s * s * (3.0 - 2.0 * s);
}

template <class Point, class Double, class Bind>
bool
RigExecNurbsCurveIsValidKernel(const std::vector<Point> *points,
                               const std::vector<double> *knots, int order)
{
    if (!points || !knots || order < 2) {
        return false;
    }
    const size_t n = points->size();
    return order <= kRigExecWireCurveMaxOrder && n >= size_t(order) &&
           knots->size() == n + size_t(order) &&
           (*knots)[size_t(order) - 1] < (*knots)[n];
}

inline double
RigExecNurbsCurveDomainStartKernel(const std::vector<double> *knots, int order)
{
    return (*knots)[size_t(order) - 1];
}

template <class Point>
double
RigExecNurbsCurveDomainEndKernel(const std::vector<Point> *points,
                                 const std::vector<double> *knots)
{
    return (*knots)[points->size()];
}

// The span of \p knots holding \p u: the largest s in [p, n-1] with
// knots[s] <= u < knots[s+1]; the domain end belongs to the last non-empty
// span.
inline size_t
RigExecNurbsCurveSpanKernel(const std::vector<double> &knots, int order,
                            size_t n, double u)
{
    const int p = order - 1;
    size_t s = size_t(p);
    while (s + 1 < n && knots[s + 1] <= u) {
        ++s;
    }
    return s;
}

template <class Point, class Double, class Bind>
Point
RigExecEvaluateNurbsCurveKernel(const std::vector<Point> *points,
                                const std::vector<double> *knots, int order,
                                double u)
{
    const std::vector<double> &k = *knots;
    const std::vector<Point> &cv = *points;
    const int p = order - 1;
    const size_t n = cv.size();
    u = std::min(std::max(u, k[size_t(p)]), k[n]);
    const size_t s = RigExecNurbsCurveSpanKernel(k, order, n, u);
    // de Boor over the order control points of that span.
    Double d[kRigExecWireCurveMaxOrder];
    const int count = std::min(order, kRigExecWireCurveMaxOrder);
    for (int j = 0; j < count; ++j) {
        const Point &c = cv[s - size_t(p) + size_t(j)];
        d[j] = Double(double(c[0]), double(c[1]), double(c[2]));
    }
    for (int r = 1; r <= p && r < kRigExecWireCurveMaxOrder; ++r) {
        for (int j = p; j >= r; --j) {
            const size_t i = s - size_t(p) + size_t(j);
            const double denom = k[i + size_t(p) + 1 - size_t(r)] - k[i];
            const double a = denom > 0.0 ? (u - k[i]) / denom : 0.0;
            d[j] = d[j - 1] * (1.0 - a) + d[j] * a;
        }
    }
    return Point(float(d[p][0]), float(d[p][1]), float(d[p][2]));
}

// The order nonzero basis functions at \p u, as the basis build needs them.
inline void
RigExecNurbsCurveBasisRowKernel(const std::vector<double> &knots, int order,
                                size_t n, double u, size_t span, double *N)
{
    const int p = order - 1;
    double left[kRigExecWireCurveMaxOrder];
    double right[kRigExecWireCurveMaxOrder];
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
}

template <class Point, class Double, class Bind, class Curve>
bool
RigExecWireInputsAreUsableKernel(const Curve &restCurve,
                                 const Curve &posedCurve,
                                 const Bind *bindCoords, size_t bindCount,
                                 size_t pointCount)
{
    // The knot vectors compare by value, so a NaN knot fails even when both
    // curves share one vector.
    return bindCoords &&
           RigExecNurbsCurveIsValidKernel<Point, Double, Bind>(
               restCurve.points, restCurve.knots, restCurve.order) &&
           RigExecNurbsCurveIsValidKernel<Point, Double, Bind>(
               posedCurve.points, posedCurve.knots, posedCurve.order) &&
           restCurve.order == posedCurve.order &&
           restCurve.points->size() == posedCurve.points->size() &&
           !(*restCurve.knots != *posedCurve.knots) && bindCount == pointCount;
}

template <class Point, class Double, class Bind>
bool
RigExecWireBasisInputsAreUsableKernel(const Bind *bindCoords, size_t bindCount,
                                      size_t meshPointCount, size_t indexCount,
                                      int order,
                                      const std::vector<double> &knots,
                                      size_t controlPointCount)
{
    const size_t n = controlPointCount;
    return bindCoords && order >= 1 && order <= kRigExecWireCurveMaxOrder &&
           n >= size_t(order) && knots.size() == n + size_t(order) &&
           knots[size_t(order - 1)] < knots[n] &&
           (bindCount == meshPointCount || bindCount == indexCount);
}

template <class Point, class Double, class Bind, class Basis>
bool
RigExecBuildWireBasisKernel(const Bind *bindCoords, size_t bindCount,
                            size_t meshPointCount,
                            const std::vector<int> &indices, int order,
                            const std::vector<double> &knots,
                            size_t controlPointCount, double dropoffDistance,
                            Basis *basis)
{
    const size_t n = controlPointCount;
    if (!basis || !RigExecWireBasisInputsAreUsableKernel<Point, Double, Bind>(
                      bindCoords, bindCount, meshPointCount, indices.size(),
                      order, knots, n)) {
        return false;
    }
    const bool parallel = bindCount == indices.size();
    const int p = order - 1;
    const double u0 = knots[size_t(p)];
    const double u1 = knots[n];
    basis->byControlPoint.assign(n, {});
    basis->ascendingIndices = std::is_sorted(indices.begin(), indices.end());
    double N[kRigExecWireCurveMaxOrder];
    for (size_t k = 0; k < indices.size(); ++k) {
        const int index = indices[k];
        if (index < 0 || size_t(index) >= meshPointCount) {
            return false;
        }
        const Bind &bind = bindCoords[parallel ? k : size_t(index)];
        const double f =
            RigExecWireDropoffKernel(double(bind[1]), dropoffDistance);
        if (f <= 0.0) {
            continue;
        }
        const double u = std::min(std::max(double(bind[0]), u0), u1);
        const size_t span = RigExecNurbsCurveSpanKernel(knots, order, n, u);
        RigExecNurbsCurveBasisRowKernel(knots, order, n, u, span, N);
        for (int r = 0; r <= p; ++r) {
            const double c = f * N[r];
            if (c != 0.0) {
                basis->byControlPoint[span - size_t(p) + size_t(r)]
                    .emplace_back(uint32_t(k), float(c));
            }
        }
    }
    return true;
}

// Control-point-major, so a point's additions arrive in control point order
// whichever range it is applied in.
template <class Point, class Double, class Bind, class Basis>
bool
RigExecApplyWireBasisGroupKernel(Point *out, size_t begin, size_t end,
                                 const Basis &basis,
                                 const std::vector<int> &indices,
                                 const std::vector<float> &weights,
                                 const std::vector<Point> &restControlPoints,
                                 const std::vector<Point> &posedControlPoints)
{
    const size_t n = basis.byControlPoint.size();
    if ((!out && begin < end) || restControlPoints.size() != n ||
        posedControlPoints.size() != n || indices.size() != weights.size()) {
        return false;
    }
    for (size_t j = 0; j < n; ++j) {
        const Point delta = posedControlPoints[j] - restControlPoints[j];
        if (delta == Point(0.0f)) {
            continue; // a control point at rest moves nothing
        }
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
            if (index < 0 || size_t(index) < begin || size_t(index) >= end) {
                continue;
            }
            out[size_t(index) - begin] += delta * (coefficient * weights[k]);
        }
    }
    return true;
}

template <class Point, class Double, class Bind, class Basis>
bool
RigExecApplyWireBasisRangeKernel(std::vector<Point> *points, const Basis &basis,
                                 const std::vector<int> &indices,
                                 const std::vector<float> &weights,
                                 const std::vector<Point> &restControlPoints,
                                 const std::vector<Point> &posedControlPoints,
                                 size_t begin, size_t end)
{
    if (!points) {
        return false;
    }
    end = std::min(end, points->size());
    begin = std::min(begin, end);
    return RigExecApplyWireBasisGroupKernel<Point, Double, Bind>(
        points->data() + begin, begin, end, basis, indices, weights,
        restControlPoints, posedControlPoints);
}

template <class Point, class Double, class Bind, class Basis>
bool
RigExecApplyWireBasisKernel(std::vector<Point> *points, const Basis &basis,
                            const std::vector<int> &indices,
                            const std::vector<float> &weights,
                            const std::vector<Point> &restControlPoints,
                            const std::vector<Point> &posedControlPoints)
{
    return RigExecApplyWireBasisRangeKernel<Point, Double, Bind>(
        points, basis, indices, weights, restControlPoints, posedControlPoints,
        0, points ? points->size() : 0);
}

template <class Point, class Double, class Bind, class Curve>
bool
RigExecApplyWireGroupKernel(Point *out, size_t begin, size_t end, size_t count,
                            const Curve &restCurve, const Curve &posedCurve,
                            const Bind *bindCoords, size_t bindCount,
                            double dropoffDistance,
                            const Point *restEvaluations,
                            size_t restEvaluationCount)
{
    if (!RigExecWireInputsAreUsableKernel<Point, Double, Bind>(
            restCurve, posedCurve, bindCoords, bindCount, count) ||
        (restEvaluations && restEvaluationCount != count)) {
        return false;
    }
    end = std::min(end, count);
    if (begin < end && !out) {
        return false;
    }
    for (size_t i = begin; i < end; ++i) {
        const double u = bindCoords[i][0];
        const double f =
            RigExecWireDropoffKernel(double(bindCoords[i][1]), dropoffDistance);
        if (f <= 0.0) {
            continue;
        }
        const Point rest =
            restEvaluations
                ? restEvaluations[i]
                : RigExecEvaluateNurbsCurveKernel<Point, Double, Bind>(
                      restCurve.points, restCurve.knots, restCurve.order, u);
        const Point delta =
            RigExecEvaluateNurbsCurveKernel<Point, Double, Bind>(
                posedCurve.points, posedCurve.knots, posedCurve.order, u) -
            rest;
        out[i - begin] += delta * float(f);
    }
    return true;
}

template <class Point, class Double, class Bind, class Curve>
bool
RigExecApplyWireKernel(std::vector<Point> *points, const Curve &restCurve,
                       const Curve &posedCurve, const Bind *bindCoords,
                       size_t bindCount, double dropoffDistance, size_t begin,
                       size_t end, const Point *restEvaluations,
                       size_t restEvaluationCount)
{
    if (!points) {
        return false;
    }
    end = std::min(end, points->size());
    begin = std::min(begin, end);
    return RigExecApplyWireGroupKernel<Point, Double, Bind>(
        points->data() + begin, begin, end, points->size(), restCurve,
        posedCurve, bindCoords, bindCount, dropoffDistance, restEvaluations,
        restEvaluationCount);
}

template <class Point, class Double, class Bind, class Curve>
bool
RigExecApplyWireSparseKernel(std::vector<Point> *points, const Curve &restCurve,
                             const Curve &posedCurve, const Bind *bindCoords,
                             size_t bindCount, double dropoffDistance,
                             const std::vector<int> &indices,
                             const std::vector<float> &weights)
{
    if (!points || !bindCoords ||
        !RigExecNurbsCurveIsValidKernel<Point, Double, Bind>(
            restCurve.points, restCurve.knots, restCurve.order) ||
        !RigExecNurbsCurveIsValidKernel<Point, Double, Bind>(
            posedCurve.points, posedCurve.knots, posedCurve.order) ||
        restCurve.order != posedCurve.order ||
        restCurve.points->size() != posedCurve.points->size() ||
        *restCurve.knots != *posedCurve.knots ||
        (bindCount != points->size() && bindCount != indices.size()) ||
        indices.size() != weights.size()) {
        return false;
    }
    const bool parallel = bindCount == indices.size();
    for (size_t k = 0; k < indices.size(); ++k) {
        const size_t i = size_t(indices[k]);
        const Bind &bind = bindCoords[parallel ? k : i];
        const double u = bind[0];
        const double f = weights[k] * RigExecWireDropoffKernel(double(bind[1]),
                                                               dropoffDistance);
        if (f <= 0.0) {
            continue;
        }
        const Point delta =
            RigExecEvaluateNurbsCurveKernel<Point, Double, Bind>(
                posedCurve.points, posedCurve.knots, posedCurve.order, u) -
            RigExecEvaluateNurbsCurveKernel<Point, Double, Bind>(
                restCurve.points, restCurve.knots, restCurve.order, u);
        (*points)[i] += delta * float(f);
    }
    return true;
}

} // namespace rigExec

#endif // RIGEXEC_MATH_WIRE_BASIS_KERNEL_H
