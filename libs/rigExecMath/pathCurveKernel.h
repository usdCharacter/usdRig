// RigExec path-curve kernel, USD-free: one non-periodic curve of a
// UsdGeomBasisCurves prim sampled by arc length, for the property movers'
// `path` operation (propertyMathKernel.h). The evaluators and the zero-USD
// runtime call these same inline functions on plain float storage, so every
// adapter computes the same bits.
//
// Conventions:
//   * Points are xyz triples of float (GfVec3f and the runtime's RrVec3f are
//     three packed floats), borrowed for one call. Arithmetic is double.
//   * Bases follow UsdGeomBasisCurves for a non-periodic curve: `linear`,
//     and `cubic` with `bezier` (vstep 3, 3k+1 points), `bspline` or
//     `catmullRom` (vstep 1, at least four points). `pinned` wrap adds the
//     phantom end points 2*P0 - P1 and 2*Pn-1 - Pn-2 so a bspline or
//     catmullRom curve reaches its first and last points; it changes nothing
//     for linear and bezier.
//   * Arc length is integrated with 8-point Gauss-Legendre quadrature on four
//     sub-intervals per segment; the parameter at a distance is found by a
//     safeguarded Newton iteration on that integral. A `planar` curve measures
//     the arc length of its projection onto the XZ plane: ground distance, its
//     y carried as a height.
//   * Heading is a yaw about +Y in degrees: the rotation that takes local +Z
//     to the planar tangent (x' = cos h x + sin h z, z' = -sin h x + cos h z),
//     the convention a control's `ry` avar rotates by. It is wrapped to
//     (-180, 180].
// Sources for the bases and the quadrature: docs/references.md.
#ifndef RIGEXEC_MATH_PATH_CURVE_KERNEL_H
#define RIGEXEC_MATH_PATH_CURVE_KERNEL_H

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rigExec {

/// The curve's interpolation: UsdGeomBasisCurves `type` and `basis` folded.
enum class RigExecPathBasis : uint8_t { Linear = 0, Bezier, Bspline, CatmullRom };

/// What a path operation returns (schema `rigExec:pathComponent`).
enum class RigExecPathComponent : uint8_t { X = 0, Y, Z, Heading, End };

/// One curve as the kernel reads it.
struct RigExecPathCurve {
    const float *points = nullptr;
    size_t pointCount = 0;
    RigExecPathBasis basis = RigExecPathBasis::Linear;
    bool pinned = false;
    /// Arc length on the XZ plane (rigExec:pathMeasure "planar").
    bool planar = false;
};

/// A path operation's uniform settings, as compile reads them off the mover
/// and its curve. `valid` is false when the mover does not name exactly one
/// supported curve; such a revision passes its value through.
struct RigExecPathSettings {
    bool valid = false;
    RigExecPathBasis basis = RigExecPathBasis::Linear;
    bool pinned = false;
    bool planar = false;
    RigExecPathComponent component = RigExecPathComponent::X;
    /// Local (x, z) offset carried by the heading frame.
    float offsetX = 0.0f;
    float offsetZ = 0.0f;
    /// Added to the tangent's heading, in degrees.
    float headingOffset = 0.0f;
};

namespace pathKernelDetail {

constexpr double kGaussNodes[4] = {0.1834346424956498, 0.5255324099163290,
                                   0.7966664774136267, 0.9602898564975363};
constexpr double kGaussWeights[4] = {0.3626837833783620, 0.3137066458778873,
                                     0.2223810344533745, 0.1012285362903763};
constexpr int kSubIntervals = 4;
constexpr double kPi = 3.14159265358979323846;

inline bool
UsesPhantoms(const RigExecPathCurve &c)
{
    return c.pinned && (c.basis == RigExecPathBasis::Bspline ||
                        c.basis == RigExecPathBasis::CatmullRom);
}

// Control point i of the (phantom-extended) control list.
inline void
Control(const RigExecPathCurve &c, long i, double out[3])
{
    const long n = long(c.pointCount);
    if (UsesPhantoms(c)) {
        // Extended list: [2P0-P1, P0 .. Pn-1, 2Pn-1-Pn-2].
        if (i == 0) {
            for (int k = 0; k < 3; ++k) {
                out[k] = 2.0 * c.points[k] - c.points[3 + k];
            }
            return;
        }
        if (i == n + 1) {
            for (int k = 0; k < 3; ++k) {
                out[k] = 2.0 * c.points[3 * (n - 1) + k] -
                         c.points[3 * (n - 2) + k];
            }
            return;
        }
        i -= 1;
    }
    for (int k = 0; k < 3; ++k) {
        out[k] = c.points[3 * i + k];
    }
}

inline long
SegmentCount(const RigExecPathCurve &c)
{
    const long n = long(c.pointCount);
    switch (c.basis) {
    case RigExecPathBasis::Linear:
        return n >= 2 ? n - 1 : 0;
    case RigExecPathBasis::Bezier:
        return n >= 4 && (n - 1) % 3 == 0 ? (n - 1) / 3 : 0;
    case RigExecPathBasis::Bspline:
    case RigExecPathBasis::CatmullRom:
        if (UsesPhantoms(c)) {
            return n >= 2 ? n - 1 : 0;
        }
        return n >= 4 ? n - 3 : 0;
    }
    return 0;
}

// Position (and derivative in t) of segment s at t in [0, 1].
inline void
Evaluate(const RigExecPathCurve &c, long s, double t, double p[3],
         double d[3])
{
    if (c.basis == RigExecPathBasis::Linear) {
        double a[3], b[3];
        Control(c, s, a);
        Control(c, s + 1, b);
        for (int k = 0; k < 3; ++k) {
            p[k] = a[k] + t * (b[k] - a[k]);
            d[k] = b[k] - a[k];
        }
        return;
    }
    const long first = c.basis == RigExecPathBasis::Bezier ? 3 * s : s;
    double q[4][3];
    for (int j = 0; j < 4; ++j) {
        Control(c, first + j, q[j]);
    }
    const double t2 = t * t, t3 = t2 * t, u = 1.0 - t;
    double w[4], dw[4];
    switch (c.basis) {
    case RigExecPathBasis::Bezier:
        w[0] = u * u * u;
        w[1] = 3.0 * u * u * t;
        w[2] = 3.0 * u * t2;
        w[3] = t3;
        dw[0] = -3.0 * u * u;
        dw[1] = 3.0 * u * u - 6.0 * u * t;
        dw[2] = 6.0 * u * t - 3.0 * t2;
        dw[3] = 3.0 * t2;
        break;
    case RigExecPathBasis::Bspline:
        w[0] = u * u * u / 6.0;
        w[1] = (3.0 * t3 - 6.0 * t2 + 4.0) / 6.0;
        w[2] = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0;
        w[3] = t3 / 6.0;
        dw[0] = -u * u / 2.0;
        dw[1] = (9.0 * t2 - 12.0 * t) / 6.0;
        dw[2] = (-9.0 * t2 + 6.0 * t + 3.0) / 6.0;
        dw[3] = t2 / 2.0;
        break;
    default:  // CatmullRom
        w[0] = 0.5 * (-t3 + 2.0 * t2 - t);
        w[1] = 0.5 * (3.0 * t3 - 5.0 * t2 + 2.0);
        w[2] = 0.5 * (-3.0 * t3 + 4.0 * t2 + t);
        w[3] = 0.5 * (t3 - t2);
        dw[0] = 0.5 * (-3.0 * t2 + 4.0 * t - 1.0);
        dw[1] = 0.5 * (9.0 * t2 - 10.0 * t);
        dw[2] = 0.5 * (-9.0 * t2 + 8.0 * t + 1.0);
        dw[3] = 0.5 * (3.0 * t2 - 2.0 * t);
        break;
    }
    for (int k = 0; k < 3; ++k) {
        p[k] = w[0] * q[0][k] + w[1] * q[1][k] + w[2] * q[2][k] +
               w[3] * q[3][k];
        d[k] = dw[0] * q[0][k] + dw[1] * q[1][k] + dw[2] * q[2][k] +
               dw[3] * q[3][k];
    }
}

inline double
Speed(const RigExecPathCurve &c, long s, double t)
{
    double p[3], d[3];
    Evaluate(c, s, t, p, d);
    return std::sqrt(d[0] * d[0] + (c.planar ? 0.0 : d[1] * d[1]) + d[2] * d[2]);
}

// Arc length of segment s over [a, b], one Gauss-Legendre panel.
inline double
Panel(const RigExecPathCurve &c, long s, double a, double b)
{
    const double half = 0.5 * (b - a), mid = 0.5 * (a + b);
    double sum = 0.0;
    for (int i = 0; i < 4; ++i) {
        sum += kGaussWeights[i] * (Speed(c, s, mid - half * kGaussNodes[i]) +
                                   Speed(c, s, mid + half * kGaussNodes[i]));
    }
    return sum * half;
}

// Arc length of segment s over [0, t].
inline double
PartialLength(const RigExecPathCurve &c, long s, double t)
{
    double sum = 0.0;
    for (int i = 0; i < kSubIntervals; ++i) {
        const double a = double(i) / kSubIntervals;
        const double b = double(i + 1) / kSubIntervals;
        if (t <= a) {
            break;
        }
        sum += Panel(c, s, a, t < b ? t : b);
    }
    return sum;
}

// The parameter of segment s at arc length `target` along it, given its
// whole length.
inline double
ParameterAt(const RigExecPathCurve &c, long s, double target, double length)
{
    if (target <= 0.0 || length <= 0.0) {
        return 0.0;
    }
    if (target >= length) {
        return 1.0;
    }
    double lo = 0.0, hi = 1.0, t = target / length;
    const double tolerance = 1e-12 * (length > 1.0 ? length : 1.0);
    for (int i = 0; i < 32; ++i) {
        const double f = PartialLength(c, s, t) - target;
        if (std::fabs(f) <= tolerance) {
            break;
        }
        if (f > 0.0) {
            hi = t;
        } else {
            lo = t;
        }
        const double speed = Speed(c, s, t);
        double next = speed > 0.0 ? t - f / speed : 0.5 * (lo + hi);
        if (!(next > lo && next < hi)) {
            next = 0.5 * (lo + hi);
        }
        t = next;
    }
    return t;
}

inline double
WrapDegrees(double h)
{
    h = std::fmod(h, 360.0);
    if (h <= -180.0) {
        h += 360.0;
    } else if (h > 180.0) {
        h -= 360.0;
    }
    return h;
}

}  // namespace pathKernelDetail

/// Whether \p curve has at least one segment for its basis and only finite
/// points.
inline bool
RigExecPathCurveValid(const RigExecPathCurve &curve)
{
    if (!curve.points || pathKernelDetail::SegmentCount(curve) < 1) {
        return false;
    }
    for (size_t i = 0; i < 3 * curve.pointCount; ++i) {
        if (!std::isfinite(curve.points[i])) {
            return false;
        }
    }
    return true;
}

/// Total arc length of \p curve; 0 for an invalid curve.
inline double
RigExecPathLength(const RigExecPathCurve &curve)
{
    using namespace pathKernelDetail;
    if (!RigExecPathCurveValid(curve)) {
        return 0.0;
    }
    double total = 0.0;
    const long segments = SegmentCount(curve);
    for (long s = 0; s < segments; ++s) {
        total += PartialLength(curve, s, 1.0);
    }
    return total;
}

/// The point and planar heading (degrees, before any offset) at arc length
/// \p distance from the curve's start, clamped to the curve. A point whose
/// own tangent has no planar direction takes the nearest planar direction
/// along its segment's chord, then along earlier segments; a curve with
/// none at all has heading 0. False for an invalid curve.
inline bool
RigExecSamplePath(const RigExecPathCurve &curve, double distance,
                  double point[3], double *heading)
{
    using namespace pathKernelDetail;
    if (!RigExecPathCurveValid(curve)) {
        return false;
    }
    const long segments = SegmentCount(curve);
    long segment = segments - 1;
    double t = 1.0;
    double before = 0.0;
    if (!(distance > 0.0)) {
        segment = 0;
        t = 0.0;
    } else {
        for (long s = 0; s < segments; ++s) {
            const double length = PartialLength(curve, s, 1.0);
            if (distance <= before + length) {
                segment = s;
                t = ParameterAt(curve, s, distance - before, length);
                break;
            }
            before += length;
        }
    }
    double d[3];
    Evaluate(curve, segment, t, point, d);
    const auto planar = [](const double v[3]) {
        return v[0] * v[0] + v[2] * v[2] > 1e-24;
    };
    if (!planar(d)) {
        // The chord of this segment, then of each earlier one.
        bool found = false;
        for (long s = segment; s >= 0 && !found; --s) {
            double a[3], b[3], unused[3];
            Evaluate(curve, s, 0.0, a, unused);
            Evaluate(curve, s, 1.0, b, unused);
            for (int k = 0; k < 3; ++k) {
                d[k] = b[k] - a[k];
            }
            found = planar(d);
        }
        if (!found) {
            d[0] = 0.0;
            d[2] = 1.0;
        }
    }
    *heading = std::atan2(d[0], d[2]) * (180.0 / kPi);
    return true;
}

/// A path operation's result for the incoming value \p base: the curve is
/// sampled at arc length (base - start), where `start` is the distance the
/// curve's first point stands at.
///   * X, Y, Z: the sampled point plus the settings' local (x, z) offset
///     rotated by the heading;
///   * Heading: the tangent's heading plus the settings' heading offset,
///     degrees, wrapped to (-180, 180];
///   * End: start plus the curve's whole length.
/// False for an invalid curve, leaving \p result untouched.
inline bool
RigExecEvaluatePathComponent(const RigExecPathCurve &curve,
                             const RigExecPathSettings &settings, float base,
                             float start, float *result)
{
    using namespace pathKernelDetail;
    if (!RigExecPathCurveValid(curve)) {
        return false;
    }
    if (settings.component == RigExecPathComponent::End) {
        *result = float(double(start) + RigExecPathLength(curve));
        return true;
    }
    double point[3], tangentHeading = 0.0;
    if (!RigExecSamplePath(curve, double(base) - double(start), point,
                           &tangentHeading)) {
        return false;
    }
    const double heading =
        WrapDegrees(tangentHeading + double(settings.headingOffset));
    if (settings.component == RigExecPathComponent::Heading) {
        *result = float(heading);
        return true;
    }
    const double radians = heading * (kPi / 180.0);
    const double c = std::cos(radians), s = std::sin(radians);
    const double ox = settings.offsetX, oz = settings.offsetZ;
    switch (settings.component) {
    case RigExecPathComponent::X:
        *result = float(point[0] + c * ox + s * oz);
        break;
    case RigExecPathComponent::Y:
        *result = float(point[1]);
        break;
    default:  // Z
        *result = float(point[2] - s * ox + c * oz);
        break;
    }
    return true;
}

/// The largest `start + k * step` (each boundary rounded to float, as a
/// float input would hold it) that does not exceed \p base; \p base itself
/// when \p step is not positive and finite.
inline float
RigExecQuantizeDown(float base, float start, float step)
{
    if (!(step > 0.0f) || !std::isfinite(step) || !std::isfinite(start) ||
        !std::isfinite(base)) {
        return base;
    }
    const double b = base, s = start, h = step;
    double k = std::floor((b - s) / h);
    const auto boundary = [&](double i) { return double(float(s + i * h)); };
    if (boundary(k + 1.0) <= b) {
        k += 1.0;
    } else if (boundary(k) > b) {
        k -= 1.0;
    }
    return float(s + k * h);
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_PATH_CURVE_KERNEL_H
