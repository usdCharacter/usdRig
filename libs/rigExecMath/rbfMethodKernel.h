// Method reference: Epic Games OpenRigLogic (MIT, see THIRD_PARTY_NOTICES.md),
// RigLogic 13.5.0 at commit 9ca65ac08b64a1ea5f0d79466e64706b3ac524ba,
// src/riglogic/rbf/cpu/RBFSolver.cpp (distances, weight functions,
// normalization, automatic radius), AdditiveRBFSolver.cpp and
// InterpolativeRBFSolver.cpp (the Crout LU inverse),
// https://github.com/EpicGames/OpenRigLogic. See docs/references.md and
// docs/concepts/riglogic.md.
//
// RigLogic's radial-basis solvers, selected on RigExecPoseInterpolator by
// rigExec:solver "additive" or "interpolative". Header-only and USD-free: the
// compile-time solve and every evaluator, the zero-USD runtime included,
// instantiate this one definition.
//
// Conventions:
// - A target or input row holds rawCount doubles. The quaternion metrics read
//   a row as rawCount / 4 quaternions in (x, y, z, w) order, RigLogic's
//   raw-control order.
// - A radius is in its metric's units: radians for quaternion, swing and
//   twist, and the inputs' own units for euclidean.
// - Gaussian is exp(-d / r^2) and exponential exp(-2 d / r), exactly as
//   RigLogic computes them. Neither is the pose solver's exp(-(d / r)^2).
// - The arithmetic is double where RigLogic's is float, so results agree with
//   RigLogic to float rounding, not bit for bit.
// - Where RigLogic divides by zero (a zero radius, a swing of a half turn
//   about a perpendicular axis, an automatic radius over fewer than two
//   targets), the result here is the limit stated at each function.
#ifndef RIGEXEC_MATH_RBF_METHOD_KERNEL_H
#define RIGEXEC_MATH_RBF_METHOD_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace rigExec {

/// Serialized values are the enum values; append only.
enum class RigExecRbfSolverType : int {
    /// Sums each target's kernel value. Normalization keeps it in range.
    Additive = 0,
    /// Weights each target through the inverse of the target-to-target
    /// kernel matrix, clamped to [0, 1] before normalization.
    Interpolative = 1,
};

enum class RigExecRbfDistanceMethod : int {
    /// n-dimensional distance over the raw row.
    Euclidean = 0,
    /// sqrt(sum of squared arc lengths), one per quaternion.
    Quaternion = 1,
    /// The quaternion metric over the swing part about the twist axis.
    SwingAngle = 2,
    /// The difference of unwound twist angles about the twist axis.
    TwistAngle = 3,
};

enum class RigExecRbfWeightFunction : int {
    Gaussian = 0,
    Exponential = 1,
    Linear = 2,
    Cubic = 3,
    Quintic = 4,
};

enum class RigExecRbfNormalizeMethod : int {
    /// Divide by the sum only when the sum exceeds one.
    OnlyAboveOne = 0,
    /// Always divide by the sum.
    Always = 1,
};

/// Everything a RigLogic solver is built from.
struct RigExecRbfMethodDesc {
    RigExecRbfSolverType solverType = RigExecRbfSolverType::Additive;
    RigExecRbfDistanceMethod distanceMethod =
        RigExecRbfDistanceMethod::Quaternion;
    RigExecRbfWeightFunction weightFunction =
        RigExecRbfWeightFunction::Gaussian;
    RigExecRbfNormalizeMethod normalizeMethod =
        RigExecRbfNormalizeMethod::OnlyAboveOne;
    /// 0 X, 1 Y, 2 Z: the axis swing and twist are measured about.
    int twistAxis = 0;
    /// Replace `radius` by the mean distance between every pair of targets.
    bool automaticRadius = false;
    double radius = 0.0;
    /// A weight at or below this, after normalization and scaling, is zero.
    double weightThreshold = 0.0;
    /// Values per target and per input.
    size_t rawCount = 0;
    /// targetCount * rawCount values, as authored.
    std::vector<double> targets;
    /// One multiplier per target; empty means one for every target.
    std::vector<double> targetScales;
};

/// The solved constants: everything the per-frame half reads. Built once by
/// RigExecRbfMethodBuild and carried as data, so no evaluator re-derives it.
struct RigExecRbfMethodTable {
    RigExecRbfSolverType solverType = RigExecRbfSolverType::Additive;
    RigExecRbfDistanceMethod distanceMethod =
        RigExecRbfDistanceMethod::Quaternion;
    RigExecRbfWeightFunction weightFunction =
        RigExecRbfWeightFunction::Gaussian;
    RigExecRbfNormalizeMethod normalizeMethod =
        RigExecRbfNormalizeMethod::OnlyAboveOne;
    int twistAxis = 0;
    size_t rawCount = 0;
    size_t targetCount = 0;
    /// The radius the kernels use, automatic radius already applied.
    double radius = 0.0;
    double weightThreshold = 0.0;
    /// targetCount * rawCount: the targets in their stored form (swing or
    /// twist quaternions for those metrics, as authored otherwise).
    std::vector<double> targets;
    /// TwistAngle only: the targets in angle form, which the distance reads.
    /// Empty for every other metric.
    std::vector<double> distanceTargets;
    /// targetCount multipliers.
    std::vector<double> targetScales;
    /// Interpolative only: targetCount * targetCount, row-major, the inverse
    /// of the target-to-target kernel matrix. Empty for additive.
    std::vector<double> coefficients;

    const std::vector<double> &RowsForDistance() const
    {
        return distanceMethod == RigExecRbfDistanceMethod::TwistAngle
                   ? distanceTargets
                   : targets;
    }
};

namespace rbfMethodDetail {

constexpr double kPi = 3.141592653589793238462643383279502884;

/// The swing part of q = (x, y, z, w) about the twist axis, RigLogic's sign
/// convention (w <= 0). A half turn about a perpendicular axis has no twist,
/// so where RigLogic would divide by zero the rotation is its own swing.
inline void
Swing(int axis, double *q)
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double t = axis == 0 ? x : (axis == 1 ? y : z);
    const double magnitude = std::sqrt(t * t + w * w);
    if (magnitude == 0.0) {
        return;
    }
    const double sw = -magnitude;
    if (axis == 0) {
        q[3] = sw;
        q[2] = (w * z + x * y) / sw;
        q[1] = (w * y - x * z) / sw;
        q[0] = 0.0;
    } else if (axis == 1) {
        q[3] = sw;
        q[2] = (w * z - y * x) / sw;
        q[1] = 0.0;
        q[0] = (w * x + y * z) / sw;
    } else {
        q[3] = sw;
        q[2] = 0.0;
        q[1] = (w * y + z * x) / sw;
        q[0] = (w * x - z * y) / sw;
    }
}

/// The twist part of q about the axis: the two other vector slots are zeroed
/// and (twist, w) renormalized. A pure perpendicular half turn has no twist
/// and becomes identity.
inline void
Twist(int axis, double *q)
{
    const int keep = std::max(0, std::min(axis, 2));
    q[(keep + 1) % 3] = 0.0;
    q[(keep + 2) % 3] = 0.0;
    const double magnitude = std::sqrt(q[keep] * q[keep] + q[3] * q[3]);
    if (magnitude == 0.0) {
        q[keep] = 0.0;
        q[3] = 1.0;
        return;
    }
    q[keep] /= magnitude;
    q[3] /= magnitude;
}

/// A twist quaternion in angle form: slot 0 holds the unwound angle in
/// [-pi, pi] and the other three are zero, so the row keeps its width and
/// the twist distance is a plain difference. Unwinding leaves a seam at a
/// half turn by design, so +120 and -120 degrees stay 240 apart.
inline void
TwistAngleForm(double *q)
{
    double angle = 2.0 * std::atan2(q[0] + q[1] + q[2], q[3]);
    if (angle > kPi) {
        angle -= 2.0 * kPi;
    } else if (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    q[0] = angle;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
}

/// The stored form of a target row.
inline void
ConvertTarget(RigExecRbfDistanceMethod method, int axis, double *row,
              size_t rawCount)
{
    const size_t quaternions = rawCount / 4;
    if (method == RigExecRbfDistanceMethod::SwingAngle) {
        for (size_t i = 0; i < quaternions; ++i) {
            Swing(axis, row + 4 * i);
        }
    } else if (method == RigExecRbfDistanceMethod::TwistAngle) {
        for (size_t i = 0; i < quaternions; ++i) {
            Twist(axis, row + 4 * i);
        }
    }
}

/// The form an input row meets the distance rows in: as a target, and for
/// twist continued into angle form.
inline void
ConvertInput(RigExecRbfDistanceMethod method, int axis, double *row,
             size_t rawCount)
{
    ConvertTarget(method, axis, row, rawCount);
    if (method == RigExecRbfDistanceMethod::TwistAngle) {
        for (size_t i = 0; i < rawCount / 4; ++i) {
            TwistAngleForm(row + 4 * i);
        }
    }
}

/// The arc length between two unit quaternions; dot^2 makes it sign-blind.
inline double
ArcLength(const double *a, const double *b)
{
    const double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    const double dotSqr = dot * dot;
    if (dotSqr > 1.0) {
        return 0.0;
    }
    return std::acos(2.0 * dotSqr - 1.0);
}

inline double
Distance(RigExecRbfDistanceMethod method, const double *target,
         const double *input, size_t rawCount)
{
    double sum = 0.0;
    if (method == RigExecRbfDistanceMethod::Euclidean) {
        for (size_t i = 0; i < rawCount; ++i) {
            const double d = target[i] - input[i];
            sum += d * d;
        }
    } else if (method == RigExecRbfDistanceMethod::TwistAngle) {
        for (size_t i = 0; i < rawCount / 4; ++i) {
            const double d = target[4 * i] - input[4 * i];
            sum += d * d;
        }
    } else {
        for (size_t i = 0; i < rawCount / 4; ++i) {
            const double arc = ArcLength(target + 4 * i, input + 4 * i);
            sum += arc * arc;
        }
    }
    return std::sqrt(sum);
}

inline double
Power(double base, int exponent)
{
    double result = base;
    for (int i = 1; i < exponent; ++i) {
        result *= base;
    }
    return result;
}

/// A non-positive radius answers 1 at zero distance and 0 elsewhere, the
/// limit of every function as the radius shrinks.
inline double
Weight(RigExecRbfWeightFunction function, double distance, double radius)
{
    if (!(radius > 0.0)) {
        return distance <= 0.0 ? 1.0 : 0.0;
    }
    switch (function) {
    case RigExecRbfWeightFunction::Linear:
        return std::max(0.0, 1.0 - distance / radius);
    case RigExecRbfWeightFunction::Cubic:
        return std::max(0.0, 1.0 - Power(distance / radius, 3));
    case RigExecRbfWeightFunction::Quintic:
        return std::max(0.0, 1.0 - Power(distance / radius, 5));
    case RigExecRbfWeightFunction::Exponential:
        return std::exp(-2.0 * distance / radius);
    case RigExecRbfWeightFunction::Gaussian:
    default:
        return std::exp(-distance / (radius * radius));
    }
}

/// Crout LU decomposition with implicit row scaling, RigLogic's `decompose`.
/// False when a row is all zero, leaving `a` untouched.
inline bool
Decompose(std::vector<double> &a, size_t n, std::vector<size_t> &permute)
{
    constexpr double absmin = 1.0e-20;
    std::vector<double> scale(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        double rowMax = 0.0;
        for (size_t j = 0; j < n; ++j) {
            rowMax = std::max(rowMax, std::abs(a[i * n + j]));
        }
        if (rowMax == 0.0) {
            return false;
        }
        scale[i] = 1.0 / rowMax;
    }
    for (size_t j = 0; j < n; ++j) {
        for (size_t i = 0; i < j; ++i) {
            double sum = a[i * n + j];
            for (size_t k = 0; k < i; ++k) {
                sum -= a[i * n + k] * a[k * n + j];
            }
            a[i * n + j] = sum;
        }
        size_t iMax = 0;
        double colMax = 0.0;
        for (size_t i = j; i < n; ++i) {
            double sum = a[i * n + j];
            for (size_t k = 0; k < j; ++k) {
                sum -= a[i * n + k] * a[k * n + j];
            }
            a[i * n + j] = sum;
            const double temp = scale[i] * std::abs(sum);
            if (temp >= colMax) {
                colMax = temp;
                iMax = i;
            }
        }
        if (j != iMax) {
            for (size_t k = 0; k < n; ++k) {
                std::swap(a[iMax * n + k], a[j * n + k]);
            }
            scale[iMax] = scale[j];
        }
        permute[j] = iMax;
        if (a[j * n + j] == 0.0) {
            a[j * n + j] = absmin;
        }
        if (j != n - 1) {
            const double temp = 1.0 / a[j * n + j];
            for (size_t i = j + 1; i < n; ++i) {
                a[i * n + j] *= temp;
            }
        }
    }
    return true;
}

/// Forward and back substitution against a Decompose result, RigLogic's
/// `substitute`.
inline void
Substitute(const std::vector<double> &a, size_t n,
           const std::vector<size_t> &permute, std::vector<double> &b)
{
    size_t ii = 0;
    for (size_t i = 0; i < n; ++i) {
        const size_t ip = permute[i];
        double sum = b[ip];
        b[ip] = b[i];
        if (ii) {
            for (size_t j = ii - 1; j < i; ++j) {
                sum -= a[i * n + j] * b[j];
            }
        } else if (sum != 0.0) {
            ii = i + 1;
        }
        b[i] = sum;
    }
    for (size_t ipo = n; ipo > 0; --ipo) {
        const size_t i = ipo - 1;
        double sum = b[i];
        for (size_t j = i + 1; j < n; ++j) {
            sum -= a[i * n + j] * b[j];
        }
        b[i] = sum / a[i * n + i];
    }
}

/// Invert in place. A matrix with an all-zero row is left as it is, as
/// RigLogic leaves it; a kernel matrix has ones on its diagonal, so that
/// never happens here.
inline void
Invert(std::vector<double> &m, size_t n)
{
    std::vector<size_t> permute(n, 0);
    if (!Decompose(m, n, permute)) {
        return;
    }
    std::vector<double> inverse(n * n, 0.0);
    std::vector<double> column(n, 0.0);
    for (size_t j = 0; j < n; ++j) {
        std::fill(column.begin(), column.end(), 0.0);
        column[j] = 1.0;
        Substitute(m, n, permute, column);
        for (size_t i = 0; i < n; ++i) {
            inverse[i * n + j] = column[i];
        }
    }
    m.swap(inverse);
}

} // namespace rbfMethodDetail

/// Solve a RigLogic interpolator once. False when the rows do not divide
/// into targets, or a quaternion metric's row is not whole quaternions.
inline bool
RigExecRbfMethodBuild(const RigExecRbfMethodDesc &desc,
                      RigExecRbfMethodTable *table)
{
    using namespace rbfMethodDetail;
    if (!table || desc.rawCount == 0 ||
        desc.targets.size() % desc.rawCount != 0) {
        return false;
    }
    if (desc.distanceMethod != RigExecRbfDistanceMethod::Euclidean &&
        desc.rawCount % 4 != 0) {
        return false;
    }
    const size_t raw = desc.rawCount;
    const size_t count = desc.targets.size() / raw;
    if (!desc.targetScales.empty() && desc.targetScales.size() != count) {
        return false;
    }
    RigExecRbfMethodTable out;
    out.solverType = desc.solverType;
    out.distanceMethod = desc.distanceMethod;
    out.weightFunction = desc.weightFunction;
    out.normalizeMethod = desc.normalizeMethod;
    out.twistAxis = std::max(0, std::min(desc.twistAxis, 2));
    out.rawCount = raw;
    out.targetCount = count;
    out.radius = desc.radius;
    out.weightThreshold = desc.weightThreshold;
    out.targets = desc.targets;
    for (size_t t = 0; t < count; ++t) {
        ConvertTarget(out.distanceMethod, out.twistAxis,
                      out.targets.data() + t * raw, raw);
    }
    if (out.distanceMethod == RigExecRbfDistanceMethod::TwistAngle) {
        out.distanceTargets = out.targets;
        for (size_t t = 0; t < count; ++t) {
            for (size_t q = 0; q < raw / 4; ++q) {
                TwistAngleForm(out.distanceTargets.data() + t * raw + 4 * q);
            }
        }
    }
    out.targetScales = desc.targetScales.empty()
                           ? std::vector<double>(count, 1.0)
                           : desc.targetScales;
    const std::vector<double> &rows = out.RowsForDistance();
    // The mean over every unordered pair; with fewer than two targets there
    // is no pair and the authored radius stands.
    if (desc.automaticRadius && count >= 2) {
        double sum = 0.0;
        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                sum += Distance(out.distanceMethod, rows.data() + j * raw,
                                rows.data() + i * raw, raw);
            }
        }
        out.radius = sum / (double(count) * double(count - 1) / 2.0);
    }
    if (out.solverType == RigExecRbfSolverType::Interpolative) {
        out.coefficients.assign(count * count, 0.0);
        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i; j < count; ++j) {
                const double d = Distance(out.distanceMethod,
                                          rows.data() + j * raw,
                                          rows.data() + i * raw, raw);
                const double w = Weight(out.weightFunction, d, out.radius);
                out.coefficients[i * count + j] = w;
                out.coefficients[j * count + i] = w;
            }
        }
        Invert(out.coefficients, count);
    }
    *table = std::move(out);
    return true;
}

/// Whether targets i and j are the same place under the table's metric, to
/// the precision of float-authored data: equal raw values for euclidean,
/// |dot| >= 1 - 1e-6 per quaternion for quaternion and swing (arc length
/// amplifies float rounding near zero, so a distance test cannot see it),
/// and twist angles within 1e-6. An interpolative solve is singular there.
inline bool
RigExecRbfMethodCoincident(const RigExecRbfMethodTable &table, size_t i,
                           size_t j)
{
    const size_t raw = table.rawCount;
    const double *a = table.RowsForDistance().data() + i * raw;
    const double *b = table.RowsForDistance().data() + j * raw;
    if (table.distanceMethod == RigExecRbfDistanceMethod::Euclidean) {
        for (size_t k = 0; k < raw; ++k) {
            if (std::abs(a[k] - b[k]) > 1.0e-9 * std::max(1.0, std::abs(a[k]))) {
                return false;
            }
        }
        return true;
    }
    for (size_t q = 0; q < raw / 4; ++q) {
        const double *x = a + 4 * q, *y = b + 4 * q;
        if (table.distanceMethod == RigExecRbfDistanceMethod::TwistAngle) {
            if (std::abs(x[0] - y[0]) > 1.0e-6) {
                return false;
            }
        } else if (std::abs(x[0] * y[0] + x[1] * y[1] + x[2] * y[2] +
                            x[3] * y[3]) < 1.0 - 1.0e-6) {
            return false;
        }
    }
    return true;
}

/// Doubles of scratch RigExecRbfMethodEvaluate needs.
inline size_t
RigExecRbfMethodScratchSize(const RigExecRbfMethodTable &table)
{
    return table.rawCount + table.targetCount;
}

/// One weight per target for an input row of rawCount values. `scratch`
/// holds RigExecRbfMethodScratchSize doubles; `weights` holds targetCount.
inline void
RigExecRbfMethodEvaluate(const RigExecRbfMethodTable &table,
                         const double *input, double *scratch,
                         double *weights)
{
    using namespace rbfMethodDetail;
    const size_t raw = table.rawCount;
    const size_t count = table.targetCount;
    double *converted = scratch;
    double *kernel = scratch + raw;
    std::copy(input, input + raw, converted);
    ConvertInput(table.distanceMethod, table.twistAxis, converted, raw);
    const std::vector<double> &rows = table.RowsForDistance();
    for (size_t t = 0; t < count; ++t) {
        kernel[t] = Weight(table.weightFunction,
                           Distance(table.distanceMethod,
                                    rows.data() + t * raw, converted, raw),
                           table.radius);
    }
    if (table.solverType == RigExecRbfSolverType::Interpolative) {
        for (size_t i = 0; i < count; ++i) {
            double weight = 0.0;
            for (size_t j = 0; j < count; ++j) {
                weight += table.coefficients[i * count + j] * kernel[j];
            }
            weights[i] = std::max(0.0, std::min(weight, 1.0));
        }
    } else {
        std::copy(kernel, kernel + count, weights);
    }
    double sum = 0.0;
    for (size_t t = 0; t < count; ++t) {
        sum += weights[t];
    }
    double ratio = 1.0;
    if (sum > 1.0 ||
        table.normalizeMethod == RigExecRbfNormalizeMethod::Always) {
        // Every weight is non-negative, so a zero sum means every weight is
        // zero; RigLogic's 0 * inf also lands on zero after the cutoff.
        ratio = sum > 0.0 ? 1.0 / sum : 0.0;
    }
    for (size_t t = 0; t < count; ++t) {
        const double weight = weights[t] * ratio * table.targetScales[t];
        weights[t] = weight > table.weightThreshold ? weight : 0.0;
    }
}

} // namespace rigExec

#endif // RIGEXEC_MATH_RBF_METHOD_KERNEL_H
