// Radial-basis pose interpolation over a solved table (rigExecMath/rbf.h's
// per-frame half on Rr types): the width fitting and the matrix inverse ran
// once at conversion time, and the table is their answer.
#ifndef RIGEXEC_RUNTIME_RBF_SOLVER_H
#define RIGEXEC_RUNTIME_RBF_SOLVER_H

#include "rigExecMath/rbfMethodKernel.h"
#include "rigExecRuntime/dualQuatMath.h"
#include "rigExecRuntime/runtimeMath.h"
#include "rigExecRuntime/values.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace rigExec {

enum class RrRbfKernel : uint8_t {
    Gaussian = 0,
    Linear = 1,
    Exponential = 2,
    Cubic = 3,
    Quintic = 4,
};

enum class RrRbfPoseType : uint8_t {
    Whole = 0,
    Swing = 1,
    Twist = 2,
};

class RrRbfSolver {
public:
    RrRbfSolver() = default;

    RrRbfSolver(const std::vector<RrVec3d> &poses,
                const std::vector<RrVec3d> &translations,
                const std::vector<RrRbfPoseType> &poseTypes,
                const RrVec3d &twistAxis, RrRbfKernel kernel, double radius,
                double translationRadius, bool normalize, bool enableRotation,
                bool enableTranslation);

    void SetSolvedTable(const std::vector<double> &radii,
                        const std::vector<double> &translationRadii,
                        const std::vector<std::vector<double>> &weights);

    void Evaluate(const RrVec3d &euler, const RrVec3d *translation,
                  std::vector<double> *out, bool allowNegativeWeights) const;

private:
    void Kernels(const RrVec3d &euler, const RrVec3d *translation,
                 std::vector<double> *out) const;

    void Normalize(std::vector<double> *weights) const;

    double Distance(const RrQuatd &quaternion, size_t index) const;

    double TranslationDistance(const RrVec3d *translation, size_t index) const;

    double Ratio(const RrQuatd &quaternion, size_t index,
                 const RrVec3d *translation) const;

    double _Kernel(double ratio) const;

    double _Width(size_t index) const;

    double _TranslationWidth(size_t index) const;

    double _MeasureRadius() const;

    double _MeasureTranslationRadius() const;

    std::vector<RrVec3d> _poses;
    std::vector<RrVec3d> _translations;
    std::vector<std::array<RrQuatd, 3>> _parts;
    std::vector<RrRbfPoseType> _poseTypes;
    RrVec3d _twistAxis{0.0, 1.0, 0.0};
    RrRbfKernel _kernel = RrRbfKernel::Gaussian;
    double _radius = 0.0;
    double _translationRadius = 0.0;
    std::vector<double> _radii;
    std::vector<double> _translationRadii;
    bool _normalize = true;
    bool _enableRotation = true;
    bool _enableTranslation = false;
    std::vector<std::vector<double>> _weights;
};

// std::acos(-1) rather than M_PI, as the kernels do.
inline double
RrRbfPi()
{
    return std::acos(-1);
}

inline constexpr double RrRbfNormalizeFloor = 1.0e-6;
inline constexpr double RrRbfSameRotation = 1.0e-6;
inline constexpr double RrRbfSameTranslation = 1.0e-9;

// Radial-basis pose interpolation (rbf.cpp): the per-frame evaluation
// half plus the solved-table reconstitution. The width fitting and the
// matrix inverse ran once at conversion time; what travels on the wire
// is their answer.

inline RrQuatd
RrRbfMakeQuat(double w, double x, double y, double z)
{
    return RrQuatd(w, RrVec3d(x, y, z));
}

// The 4-vector dot, real-first then x, y, z -- NOT the imaginary dot,
// which sums in a different order.
inline double
RrRbfQuatDot(const RrQuatd &a, const RrQuatd &b)
{
    const RrVec3d &ai = a.GetImaginary();
    const RrVec3d &bi = b.GetImaginary();
    return a.GetReal() * b.GetReal() + ai[0] * bi[0] + ai[1] * bi[1] +
           ai[2] * bi[2];
}

inline double
RrRbfGaussian(double distance, double radius)
{
    if (radius <= 0.0) {
        return distance <= 0.0 ? 1.0 : 0.0;
    }
    const double ratio = distance / radius;
    return std::exp(-(ratio * ratio));
}

inline double
RrRbfLinear(double distance, double radius)
{
    if (radius <= 0.0) {
        return distance <= 0.0 ? 1.0 : 0.0;
    }
    return std::max(0.0, 1.0 - distance / radius);
}

inline double
RrRbfEvalKernel(RrRbfKernel kernel, double distance, double radius)
{
    switch (kernel) {
    case RrRbfKernel::Linear:
        return RrRbfLinear(distance, radius);
    case RrRbfKernel::Exponential:
        return rbfMethodDetail::Weight(RigExecRbfWeightFunction::Exponential,
                                       distance, radius);
    case RrRbfKernel::Cubic:
        return rbfMethodDetail::Weight(RigExecRbfWeightFunction::Cubic,
                                       distance, radius);
    case RrRbfKernel::Quintic:
        return rbfMethodDetail::Weight(RigExecRbfWeightFunction::Quintic,
                                       distance, radius);
    case RrRbfKernel::Gaussian:
    default:
        return RrRbfGaussian(distance, radius);
    }
}

inline double
RrRbfCombine(double angle, double width, double gap, double translationWidth,
             bool enableRotation, bool enableTranslation)
{
    if (!enableTranslation) {
        if (!enableRotation) {
            return 0.0;
        }
        if (width <= 0.0) {
            return angle <= 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
        }
        return angle / width;
    }

    double total = 0.0;
    if (enableRotation) {
        if (width <= 0.0) {
            if (angle > 0.0) {
                return std::numeric_limits<double>::infinity();
            }
        } else {
            const double share = angle / width;
            total += share * share;
        }
    }
    if (translationWidth <= 0.0) {
        if (gap > 0.0) {
            return std::numeric_limits<double>::infinity();
        }
    } else {
        const double share = gap / translationWidth;
        total += share * share;
    }
    return std::sqrt(total);
}

// The closed-form euler -> quaternion, term for term, NOT a GfRotation
// composition.
inline RrQuatd
RrRbfQuaternionFromEuler(const RrVec3d &euler)
{
    const double hx = euler[0] * 0.5;
    const double hy = euler[1] * 0.5;
    const double hz = euler[2] * 0.5;
    const double cx = std::cos(hx), cy = std::cos(hy), cz = std::cos(hz);
    const double sx = std::sin(hx), sy = std::sin(hy), sz = std::sin(hz);
    return RrRbfMakeQuat(
        cx * cy * cz + sx * sy * sz, sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz, cx * cy * sz - sx * sy * cz);
}

inline RrVec3d
RrRbfEulerFromQuaternion(const RrQuatd &quaternion)
{
    const double w = quaternion.GetReal();
    const RrVec3d &v = quaternion.GetImaginary();
    const double x = v[0], y = v[1], z = v[2];

    const double sinr = 2.0 * (w * x + y * z);
    const double cosr = 1.0 - 2.0 * (x * x + y * y);
    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = std::abs(sinp) >= 1.0
                             ? std::copysign(RrRbfPi() / 2.0, sinp)
                             : std::asin(sinp);
    const double siny = 2.0 * (w * z + x * y);
    const double cosy = 1.0 - 2.0 * (y * y + z * z);
    return RrVec3d(std::atan2(sinr, cosr), pitch, std::atan2(siny, cosy));
}

inline double
RrRbfAngleBetween(const RrQuatd &first, const RrQuatd &second)
{
    const double dot = RrRbfQuatDot(first, second);
    return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
}

inline void
RrRbfSwingTwist(const RrQuatd &quaternion, const RrVec3d &axis, RrQuatd *swing,
                RrQuatd *twist)
{
    const double w = quaternion.GetReal();
    const RrVec3d &v = quaternion.GetImaginary();
    const double x = v[0], y = v[1], z = v[2];

    double ax = axis[0], ay = axis[1], az = axis[2];
    double length = std::sqrt(ax * ax + ay * ay + az * az);
    if (length == 0.0) {
        length = 1.0;
    }
    ax /= length;
    ay /= length;
    az /= length;

    const double dot = x * ax + y * ay + z * az;
    double tw[4] = {w, ax * dot, ay * dot, az * dot};
    const double size = std::sqrt(tw[0] * tw[0] + tw[1] * tw[1] +
                                  tw[2] * tw[2] + tw[3] * tw[3]);
    if (size < 1.0e-9) {
        tw[0] = 1.0;
        tw[1] = tw[2] = tw[3] = 0.0;
    } else {
        tw[0] /= size;
        tw[1] /= size;
        tw[2] /= size;
        tw[3] /= size;
    }

    const double tw_w = tw[0], tw_x = -tw[1], tw_y = -tw[2], tw_z = -tw[3];
    const RrQuatd sw = RrRbfMakeQuat(w * tw_w - x * tw_x - y * tw_y - z * tw_z,
                                     w * tw_x + x * tw_w + y * tw_z - z * tw_y,
                                     w * tw_y - x * tw_z + y * tw_w + z * tw_x,
                                     w * tw_z + x * tw_y - y * tw_x + z * tw_w);

    if (swing) {
        *swing = sw;
    }
    if (twist) {
        *twist = RrRbfMakeQuat(tw[0], tw[1], tw[2], tw[3]);
    }
}

inline RrRbfSolver::RrRbfSolver(const std::vector<RrVec3d> &poses,
                                const std::vector<RrVec3d> &translations,
                                const std::vector<RrRbfPoseType> &poseTypes,
                                const RrVec3d &twistAxis, RrRbfKernel kernel,
                                double radius, double translationRadius,
                                bool normalize, bool enableRotation,
                                bool enableTranslation)
    : _poses(poses), _translations(translations), _poseTypes(poseTypes),
      _twistAxis(twistAxis), _kernel(kernel), _normalize(normalize),
      _enableRotation(enableRotation)
{
    _enableTranslation = enableTranslation && !_translations.empty();

    _parts.resize(_poses.size());
    for (size_t i = 0; i < _poses.size(); ++i) {
        const RrQuatd whole = RrRbfQuaternionFromEuler(_poses[i]);
        RrQuatd swing, twist;
        RrRbfSwingTwist(whole, _twistAxis, &swing, &twist);
        _parts[i] = {whole, swing, twist};
    }

    _radius = radius > 0.0 ? radius : _MeasureRadius();
    _translationRadius = translationRadius > 0.0 ? translationRadius
                                                 : _MeasureTranslationRadius();
}

inline void
RrRbfSolver::SetSolvedTable(const std::vector<double> &radii,
                            const std::vector<double> &translationRadii,
                            const std::vector<std::vector<double>> &weights)
{
    _radii = radii;
    _translationRadii = translationRadii;
    _weights = weights;
}

inline void
RrRbfSolver::Kernels(const RrVec3d &euler, const RrVec3d *translation,
                     std::vector<double> *out) const
{
    const RrQuatd here = RrRbfQuaternionFromEuler(euler);
    out->resize(_poses.size());
    for (size_t index = 0; index < _poses.size(); ++index) {
        (*out)[index] = _Kernel(Ratio(here, index, translation));
    }
}

inline void
RrRbfSolver::Normalize(std::vector<double> *weights) const
{
    if (!_normalize) {
        return;
    }
    double total = 0.0;
    for (double value : *weights) {
        total += value;
    }
    if (std::abs(total) < RrRbfNormalizeFloor) {
        return;
    }
    for (double &value : *weights) {
        value /= total;
    }
}

inline void
RrRbfSolver::Evaluate(const RrVec3d &euler, const RrVec3d *translation,
                      std::vector<double> *out, bool allowNegativeWeights) const
{
    out->clear();
    if (_weights.empty()) {
        return;
    }
    std::vector<double> found;
    Kernels(euler, translation, &found);
    out->resize(_weights.size());
    for (size_t row = 0; row < _weights.size(); ++row) {
        double total = 0.0;
        for (size_t index = 0; index < found.size(); ++index) {
            total += _weights[row][index] * found[index];
        }
        (*out)[row] = total;
    }
    Normalize(out);
    if (!allowNegativeWeights) {
        for (double &value : *out) {
            value = std::max(0.0, value);
        }
    }
}

inline double
RrRbfSolver::Distance(const RrQuatd &quaternion, size_t index) const
{
    const RrRbfPoseType kind =
        index < _poseTypes.size() ? _poseTypes[index] : RrRbfPoseType::Whole;
    if (kind != RrRbfPoseType::Swing && kind != RrRbfPoseType::Twist) {
        return RrRbfAngleBetween(quaternion, _parts[index][0]);
    }
    const size_t part = kind == RrRbfPoseType::Swing ? 1 : 2;
    RrQuatd swing, twist;
    RrRbfSwingTwist(quaternion, _twistAxis, &swing, &twist);
    return RrRbfAngleBetween(part == 1 ? swing : twist, _parts[index][part]);
}

inline double
RrRbfSolver::TranslationDistance(const RrVec3d *translation, size_t index) const
{
    if (index >= _translations.size()) {
        return 0.0;
    }
    const RrVec3d rest(0.0);
    const RrVec3d &here = translation ? *translation : rest;
    const RrVec3d &there = _translations[index];
    double total = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double delta = here[i] - there[i];
        total += delta * delta;
    }
    return std::sqrt(total);
}

inline double
RrRbfSolver::Ratio(const RrQuatd &quaternion, size_t index,
                   const RrVec3d *translation) const
{
    const double angle = _enableRotation ? Distance(quaternion, index) : 0.0;
    const double gap =
        _enableTranslation ? TranslationDistance(translation, index) : 0.0;
    return RrRbfCombine(angle, _Width(index), gap, _TranslationWidth(index),
                        _enableRotation, _enableTranslation);
}

inline double
RrRbfSolver::_Kernel(double ratio) const
{
    return RrRbfEvalKernel(_kernel, ratio, 1.0);
}

inline double
RrRbfSolver::_Width(size_t index) const
{
    if (!_radii.empty() && index < _radii.size()) {
        return _radii[index];
    }
    return _radius;
}

inline double
RrRbfSolver::_TranslationWidth(size_t index) const
{
    if (!_translationRadii.empty() && index < _translationRadii.size()) {
        return _translationRadii[index];
    }
    return _translationRadius;
}

inline double
RrRbfSolver::_MeasureRadius() const
{
    if (_poses.size() < 2) {
        return RrRbfPi() / 2.0;
    }
    std::vector<double> nearest;
    for (size_t index = 0; index < _poses.size(); ++index) {
        const RrQuatd &here = _parts[index][0];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _poses.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = Distance(here, other);
            if (value <= RrRbfSameRotation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        if (found) {
            nearest.push_back(best);
        }
    }
    double total = 0.0;
    for (double value : nearest) {
        total += value;
    }
    if (nearest.empty() || total <= 0.0) {
        return RrRbfPi() / 2.0;
    }
    return total / static_cast<double>(nearest.size());
}

inline double
RrRbfSolver::_MeasureTranslationRadius() const
{
    if (_translations.size() < 2) {
        return 0.0;
    }
    std::vector<double> nearest;
    for (size_t index = 0; index < _translations.size(); ++index) {
        const RrVec3d &here = _translations[index];
        double best = 0.0;
        bool found = false;
        for (size_t other = 0; other < _translations.size(); ++other) {
            if (other == index) {
                continue;
            }
            const double value = TranslationDistance(&here, other);
            if (value <= RrRbfSameTranslation) {
                continue;
            }
            if (!found || value < best) {
                best = value;
                found = true;
            }
        }
        if (found) {
            nearest.push_back(best);
        }
    }
    double total = 0.0;
    for (double value : nearest) {
        total += value;
    }
    if (nearest.empty() || total <= 0.0) {
        return 0.0;
    }
    return total / static_cast<double>(nearest.size());
}

// The pose interpolator arithmetic's Math (poseArithmeticKernel.h) over any
// frame type holding `points` (four RrVec3d) and RrPointFrameFlags `flags`.
template <class Frame> struct RrPoseInterpolatorMathT {
    using Quaternion = RrQuatd;
    using Vector = RrVec3d;

    static bool FrameMatrix(const Frame &frame, RrMat4d *out)
    {
        return (frame.flags & RrPointFrameValid) != 0 &&
               (frame.flags & RrPointFrameDegenerate) == 0 &&
               RrDualQuatPointsToMatrix(RrIdentityLandmarks(), frame.points,
                                        out);
    }
    static bool FrameRotation(const Frame &frame, Quaternion *out)
    {
        RrMat4d matrix;
        matrix.SetIdentity();
        if (!FrameMatrix(frame, &matrix))
            return false;
        matrix.SetTranslateOnly(RrVec3d(0.0));
        *out = matrix.GetOrthonormalized(false).ExtractRotationQuat();
        return true;
    }
    static bool FrameTranslation(const Frame &driverFinal,
                                 const Frame &driverRest,
                                 const Frame *parentFinal,
                                 const Frame *parentRest, Vector *out)
    {
        RrMat4d world(1.0), restWorld(1.0);
        if (!FrameMatrix(driverFinal, &world) ||
            !FrameMatrix(driverRest, &restWorld))
            return false;
        RrMat4d local = world, restLocal = restWorld;
        double det = 0.0;
        if (parentFinal && parentRest) {
            RrMat4d parent(1.0), restParent(1.0);
            if (!FrameMatrix(*parentFinal, &parent) ||
                !FrameMatrix(*parentRest, &restParent))
                return false;
            const RrMat4d parentInverse = parent.GetInverse(&det);
            if (det == 0.0)
                return false;
            const RrMat4d restParentInverse = restParent.GetInverse(&det);
            if (det == 0.0)
                return false;
            local = world * parentInverse;
            restLocal = restWorld * restParentInverse;
        }
        const RrMat4d restLocalInverse = restLocal.GetInverse(&det);
        if (det == 0.0)
            return false;
        *out = restLocalInverse.Transform(local.ExtractTranslation());
        return true;
    }
    static Vector EulerFromQuaternion(const Quaternion &q)
    {
        return RrRbfEulerFromQuaternion(q);
    }
};

} // namespace rigExec

#endif // RIGEXEC_RUNTIME_RBF_SOLVER_H
