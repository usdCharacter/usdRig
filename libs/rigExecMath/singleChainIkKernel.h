// Arbitrary-length single-chain IK (deterministic FABRIK) over four-point
// frames, shared by the native (Gf) path, the zero-USD runtime (Rr) and the
// solver ops. Header-only and USD-free.
//
// Host types come from RigExecDualQuatMath<Point, Double>. A frame is
// duck-typed: `points` (four Double) and `flags`.
#ifndef RIGEXEC_MATH_SINGLE_CHAIN_IK_KERNEL_H
#define RIGEXEC_MATH_SINGLE_CHAIN_IK_KERNEL_H

#include "chainSolverKernel.h"
#include "limbStretchKernel.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rigExec {

/// RigExecSingleChainIkParams over the host vector; mode 0 is RotatePlane,
/// 1 SingleChain, anything else fails the solve.
template <class Double> struct RigExecSingleChainIkKernelParams {
    int mode = 0;
    Double pole{0.0, 1.0, 0.0};
    double twistDegrees = 0.0;
    double weight = 1.0;
    bool preserveJointOrientation = false;
    double stretch = 0.0;
};

namespace singleChainIkDetail {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr int kFabrikIterations = 128;
constexpr int kRotatePlane = 0;
constexpr int kSingleChain = 1;

template <class Double>
bool
IsFinite(const Double &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

template <class F>
bool
IsFiniteFrame(const F &frame)
{
    if (!(frame.flags & chainSolverDetail::kFrameValid) ||
        (frame.flags & chainSolverDetail::kFrameDegenerate))
        return false;
    for (const auto &point : frame.points)
        if (!IsFinite(point))
            return false;
    return true;
}

template <class F>
std::vector<F>
Degenerate(const std::vector<F> &frames)
{
    std::vector<F> failed = frames;
    for (F &frame : failed)
        frame.flags |= chainSolverDetail::kFrameDegenerate;
    return failed;
}

template <class Point, class Double> struct Ops {
    using Math = RigExecDualQuatMath<Point, Double>;

    static Double ProjectPerpendicular(const Double &v, const Double &axis)
    {
        return v - axis * Math::Dot(v, axis);
    }

    static Double WorldAxisLeastParallel(const Double &axis)
    {
        const Double world[3] = {
            Double(1.0, 0.0, 0.0),
            Double(0.0, 1.0, 0.0),
            Double(0.0, 0.0, 1.0),
        };
        int best = 0;
        double bestAlignment = std::abs(Math::Dot(world[0], axis));
        for (int i = 1; i < 3; ++i) {
            const double alignment = std::abs(Math::Dot(world[i], axis));
            if (alignment < bestAlignment) {
                best = i;
                bestAlignment = alignment;
            }
        }
        return world[best];
    }

    static Double StablePerpendicular(const Double &axis, const Double &first,
                                      const Double &second, double epsilon)
    {
        const auto normalized = [](const Double &candidate) {
            const double length = candidate.GetLength();
            return std::isfinite(length) && length > 0.0
                       ? candidate / length
                       : Double(0.0, 0.0, 0.0);
        };
        Double result = ProjectPerpendicular(normalized(first), axis);
        if (result.GetLength() <= epsilon)
            result = ProjectPerpendicular(normalized(second), axis);
        if (result.GetLength() <= epsilon)
            result = ProjectPerpendicular(WorldAxisLeastParallel(axis), axis);
        return result.GetNormalized();
    }

    static Double Rotate(const Double &v, const Double &unitAxis,
                         double radians)
    {
        const double c = std::cos(radians);
        const double s = std::sin(radians);
        return v * c + Math::Cross(unitAxis, v) * s +
               unitAxis * Math::Dot(unitAxis, v) * (1.0 - c);
    }

    /// The deterministic shortest rotation from fromAxis to toAxis, scaled
    /// by weight, applied to v. The fallback supplies the 180-degree axis.
    static Double RotateToward(const Double &v, const Double &fromAxis,
                               const Double &toAxis, double weight,
                               const Double &fallback, double epsilon)
    {
        const Double from = fromAxis.GetNormalized();
        const Double to = toAxis.GetNormalized();
        const double dot = std::min(std::max(Math::Dot(from, to), -1.0), 1.0);
        Double rotationAxis = Math::Cross(from, to);
        const double sinAngle = rotationAxis.GetLength();
        double angle = 0.0;
        if (sinAngle > epsilon) {
            rotationAxis /= sinAngle;
            angle = std::atan2(sinAngle, dot);
        } else if (dot < 0.0) {
            rotationAxis = StablePerpendicular(from, fallback,
                                               Double(0.0, 0.0, 0.0), epsilon);
            angle = kPi;
        } else {
            return v;
        }
        return Rotate(v, rotationAxis, angle * weight);
    }

    static Double BlendDirection(const Double &from, const Double &to,
                                 double weight, const Double &fallback,
                                 double epsilon)
    {
        Double result = RotateToward(from, from, to, weight, fallback, epsilon);
        const double length = result.GetLength();
        if (length <= epsilon || !IsFinite(result))
            return from.GetNormalized();
        return result / length;
    }

    struct Basis {
        Double x;
        Double up;
        double xLength = 0.0;
        double yLength = 0.0;
        double zLength = 0.0;
        double handedness = 1.0;
    };

    template <class F>
    static bool ExtractBasis(const F &frame, double epsilon, Basis *basis)
    {
        const Double x = frame.points[1] - frame.points[0];
        const Double y = frame.points[2] - frame.points[0];
        const Double z = frame.points[3] - frame.points[0];
        const double xLength = x.GetLength();
        const double yLength = y.GetLength();
        const double zLength = z.GetLength();
        const double frameScale = std::max({xLength, yLength, zLength});
        if (!std::isfinite(frameScale) || frameScale <= 0.0 ||
            xLength <= frameScale * epsilon ||
            yLength <= frameScale * epsilon || zLength <= frameScale * epsilon)
            return false;

        const Double unitX = x / xLength;
        Double up = ProjectPerpendicular(y / yLength, unitX);
        if (up.GetLength() <= epsilon)
            return false;
        up.Normalize();

        // Normalized input axes keep the handedness test scale independent.
        const double determinant =
            Math::Dot(Math::Cross(x / xLength, y / yLength), z / zLength);
        if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-10)
            return false;

        basis->x = unitX;
        basis->up = up;
        basis->xLength = xLength;
        basis->yLength = yLength;
        basis->zLength = zLength;
        basis->handedness = determinant < 0.0 ? -1.0 : 1.0;
        return true;
    }

    template <class F>
    static F FrameFromBasis(const Basis &basis, const Double &origin,
                            const Double &unitX, const Double &unitUp)
    {
        const Double side =
            Math::Cross(unitX, unitUp).GetNormalized() * basis.handedness;
        F frame;
        frame.points[0] = origin;
        frame.points[1] = origin + unitX * basis.xLength;
        frame.points[2] = origin + unitUp * basis.yLength;
        frame.points[3] = origin + side * basis.zLength;
        frame.flags = chainSolverDetail::kFrameValid;
        if (basis.handedness < 0.0)
            frame.flags |= chainSolverDetail::kFrameReflected;
        return frame;
    }

    static Basis AimedBasis(const Basis &basis, const Double &unitAim,
                            double epsilon)
    {
        Double up =
            RotateToward(basis.up, basis.x, unitAim, 1.0, basis.up, epsilon);
        up = ProjectPerpendicular(up, unitAim);
        if (up.GetLength() <= epsilon)
            up = StablePerpendicular(unitAim, basis.up, basis.x, epsilon);
        else
            up.Normalize();
        Basis aimed = basis;
        aimed.x = unitAim;
        aimed.up = up;
        return aimed;
    }

    static Basis TransportedBasis(const Basis &basis,
                                  const Double &fromDirection,
                                  const Double &toDirection, double epsilon)
    {
        Basis transported = basis;
        transported.x = RotateToward(basis.x, fromDirection, toDirection, 1.0,
                                     basis.up, epsilon);
        transported.x.Normalize();
        transported.up = RotateToward(basis.up, fromDirection, toDirection, 1.0,
                                      basis.up, epsilon);
        transported.up = ProjectPerpendicular(transported.up, transported.x);
        if (transported.up.GetLength() <= epsilon)
            transported.up =
                StablePerpendicular(transported.x, basis.up, basis.x, epsilon);
        else
            transported.up.Normalize();
        return transported;
    }

    static double SignedAngle(const Double &from, const Double &to,
                              const Double &unitAxis)
    {
        return std::atan2(Math::Dot(unitAxis, Math::Cross(from, to)),
                          std::min(std::max(Math::Dot(from, to), -1.0), 1.0));
    }

    template <class F>
    static F BlendEndFrame(const Basis &input, const Basis &effector,
                           const Double &origin, double weight, double epsilon)
    {
        const Double x =
            BlendDirection(input.x, effector.x, weight, input.up, epsilon);

        Double carriedAtTarget =
            RotateToward(input.up, input.x, effector.x, 1.0, input.up, epsilon);
        carriedAtTarget = ProjectPerpendicular(carriedAtTarget, effector.x);
        if (carriedAtTarget.GetLength() <= epsilon)
            carriedAtTarget =
                StablePerpendicular(effector.x, input.up, effector.up, epsilon);
        else
            carriedAtTarget.Normalize();
        const double roll =
            SignedAngle(carriedAtTarget, effector.up, effector.x);

        Double up = RotateToward(input.up, input.x, effector.x, weight,
                                 input.up, epsilon);
        up = ProjectPerpendicular(up, x);
        if (up.GetLength() <= epsilon)
            up = StablePerpendicular(x, input.up, effector.up, epsilon);
        else
            up.Normalize();
        up = Rotate(up, x, roll * weight).GetNormalized();
        return FrameFromBasis<F>(input, origin, x, up);
    }

    static Double DirectionOrFallback(const Double &direction,
                                      const Double &fallback,
                                      double positionEpsilon,
                                      double angularEpsilon)
    {
        const double directionLength = direction.GetLength();
        if (directionLength > positionEpsilon)
            return direction / directionLength;
        const double fallbackLength = fallback.GetLength();
        if (fallbackLength > angularEpsilon)
            return fallback / fallbackLength;
        return Double(1.0, 0.0, 0.0);
    }

    static std::vector<Double>
    SolvePositions(const std::vector<Double> &original,
                   const std::vector<double> &lengths,
                   const std::vector<Basis> &bases, const Basis &effectorBasis,
                   const Double &goal,
                   const RigExecSingleChainIkKernelParams<Double> &params,
                   double positionEpsilon, double angularEpsilon)
    {
        const size_t count = original.size();
        const Double root = original.front();
        double totalLength = 0.0;
        for (double length : lengths)
            totalLength += length;

        const Double rootToGoal = goal - root;
        const double goalDistance = rootToGoal.GetLength();
        Double solveAxis;
        if (goalDistance > positionEpsilon) {
            solveAxis = rootToGoal / goalDistance;
        } else {
            solveAxis = original.back() - root;
            if (solveAxis.GetLength() <= positionEpsilon)
                solveAxis = original[1] - root;
            solveAxis = DirectionOrFallback(solveAxis, bases.front().x,
                                            positionEpsilon, angularEpsilon);
        }

        // A far-unreachable target has one closest solution: full extension
        // down the root-to-goal ray.
        if (goalDistance >= totalLength - positionEpsilon &&
            goalDistance > positionEpsilon) {
            std::vector<Double> extended(count);
            extended[0] = root;
            for (size_t i = 0; i < lengths.size(); ++i)
                extended[i + 1] = extended[i] + solveAxis * lengths[i];
            return extended;
        }

        std::vector<Double> positions = original;
        Double planeUp;
        if (params.mode == kRotatePlane) {
            Double poleOffset = params.pole - root;
            if (poleOffset.GetLength() <= positionEpsilon)
                poleOffset = Double(0.0, 0.0, 0.0);
            planeUp = StablePerpendicular(solveAxis, poleOffset,
                                          bases.front().up, angularEpsilon);
            planeUp =
                Rotate(planeUp, solveAxis, params.twistDegrees * (kPi / 180.0));
            planeUp.Normalize();
        } else {
            // FBX SingleChain orients the plane by the effector's up axis.
            planeUp = StablePerpendicular(solveAxis, effectorBasis.up,
                                          effectorBasis.x, angularEpsilon);
        }
        const Double planeNormal =
            Math::Cross(solveAxis, planeUp).GetNormalized();

        // The initial bend projected into the plane's positive half; every
        // seed, fallback and anchor stays planar.
        for (size_t i = 1; i + 1 < count; ++i) {
            const Double relative = positions[i] - root;
            const double along = Math::Dot(relative, solveAxis);
            const double towardPlane = std::abs(Math::Dot(relative, planeUp));
            positions[i] = root + solveAxis * along + planeUp * towardPlane;
        }

        // A straight seed toward a reachable goal on its own ray gets a
        // deterministic scale-relative arc along planeUp.
        double maxOffAxis = 0.0;
        for (size_t i = 1; i + 1 < count; ++i)
            maxOffAxis = std::max(
                maxOffAxis, ProjectPerpendicular(positions[i] - root, solveAxis)
                                .GetLength());
        if (count > 2 && maxOffAxis <= positionEpsilon * 8.0 &&
            goalDistance < totalLength - positionEpsilon) {
            const Double bend = planeUp;
            double distance = 0.0;
            const double amplitude =
                std::max(totalLength * 0.05, positionEpsilon * 16.0);
            for (size_t i = 1; i + 1 < count; ++i) {
                distance += lengths[i - 1];
                positions[i] +=
                    bend * (amplitude * std::sin(kPi * distance / totalLength));
            }
        }

        std::vector<Double> fallback(lengths.size());
        for (size_t i = 0; i < lengths.size(); ++i) {
            Double direction = positions[i + 1] - positions[i];
            direction -= planeNormal * Math::Dot(direction, planeNormal);
            fallback[i] = DirectionOrFallback(direction, solveAxis,
                                              positionEpsilon, angularEpsilon);
        }

        std::vector<Double> best = positions;
        double bestError = std::numeric_limits<double>::infinity();
        const double tolerance =
            std::max(positionEpsilon * 8.0, totalLength * 1e-12);
        for (int iteration = 0; iteration < kFabrikIterations; ++iteration) {
            positions.back() = goal;
            // Backward: each parent at its segment length from its child.
            for (size_t i = count - 1; i-- > 0;) {
                const Double direction = DirectionOrFallback(
                    positions[i] - positions[i + 1], -fallback[i],
                    positionEpsilon, angularEpsilon);
                positions[i] = positions[i + 1] + direction * lengths[i];
            }
            // Forward: the fixed root and the segment lengths restored.
            positions[0] = root;
            for (size_t i = 0; i < lengths.size(); ++i) {
                const Double direction = DirectionOrFallback(
                    positions[i + 1] - positions[i], fallback[i],
                    positionEpsilon, angularEpsilon);
                positions[i + 1] = positions[i] + direction * lengths[i];
                fallback[i] = direction;
            }
            const double error = (positions.back() - goal).GetLength();
            if (error < bestError) {
                bestError = error;
                best = positions;
            }
            if (error <= tolerance)
                break;
        }
        return best;
    }
};

} // namespace singleChainIkDetail

/// Solves the ordered chain \p current toward \p effector's origin; see
/// RigExecSolveSingleChainIk. A failed solve returns the input flagged
/// degenerate; zero weight returns it unchanged.
template <class Point, class Double, class F>
std::vector<F>
RigExecSolveSingleChainIkKernel(
    const std::vector<F> &current, const F &effector,
    const RigExecSingleChainIkKernelParams<Double> &params)
{
    using namespace singleChainIkDetail;
    using O = Ops<Point, Double>;
    using Basis = typename O::Basis;
    if (!std::isfinite(params.weight))
        return Degenerate(current);
    const double weight = std::min(std::max(params.weight, 0.0), 1.0);
    // Dormant: disconnected or malformed inputs are not inspected.
    if (weight <= 0.0)
        return current;
    if (current.size() < 2)
        return Degenerate(current);
    if (params.mode != kRotatePlane && params.mode != kSingleChain)
        return Degenerate(current);
    if (!IsFinite(effector.points[0]))
        return Degenerate(current);
    // SingleChain ignores pole and twist; RotatePlane ignores the effector's
    // orientation.
    if (params.mode == kRotatePlane &&
        (!IsFinite(params.pole) || !std::isfinite(params.twistDegrees)))
        return Degenerate(current);
    for (const F &frame : current)
        if (!IsFiniteFrame(frame))
            return Degenerate(current);

    std::vector<Double> original(current.size());
    std::vector<double> lengths(current.size() - 1);
    double segmentScale = 0.0;
    for (size_t i = 0; i < current.size(); ++i) {
        original[i] = current[i].points[0];
        if (i + 1 < current.size()) {
            lengths[i] = (current[i + 1].points[0] - original[i]).GetLength();
            segmentScale = std::max(segmentScale, lengths[i]);
        }
    }
    if (!std::isfinite(segmentScale) || segmentScale <= 0.0)
        return Degenerate(current);
    const double positionEpsilon = segmentScale * 1e-10;
    constexpr double angularEpsilon = 1e-12;

    std::vector<Basis> bases(current.size());
    for (size_t i = 0; i < current.size(); ++i)
        if (!O::ExtractBasis(current[i], angularEpsilon, &bases[i]))
            return Degenerate(current);
    Basis effectorBasis = bases.back();
    if (params.mode == kSingleChain &&
        (!IsFiniteFrame(effector) ||
         !O::ExtractBasis(effector, angularEpsilon, &effectorBasis)))
        return Degenerate(current);
    for (double length : lengths)
        if (!std::isfinite(length) || length <= positionEpsilon)
            return Degenerate(current);

    if (!std::isfinite(params.stretch) || params.stretch < 0 ||
        params.stretch > 1)
        return Degenerate(current);
    double totalLength = 0;
    for (double length : lengths)
        totalLength += length;
    const double goalDistance =
        (effector.points[0] - original.front()).GetLength();
    const double growth =
        1 + params.stretch * std::max(0.0, goalDistance / totalLength - 1);
    std::vector<double> solveLengths = lengths;
    if (growth != 1)
        for (double &length : solveLengths)
            length *= growth;

    const std::vector<Double> solved = O::SolvePositions(
        original, solveLengths, bases, effectorBasis, effector.points[0],
        params, positionEpsilon, angularEpsilon);
    if (solved.size() != current.size())
        return Degenerate(current);

    // The full-strength orientations first, then the blend from the input
    // ones: continuous as the weight approaches zero.
    std::vector<Basis> solvedBases = bases;
    for (size_t i = 0; i + 1 < solvedBases.size(); ++i) {
        const Double solvedSegment = solved[i + 1] - solved[i];
        const Double aim = solvedSegment / solvedSegment.GetLength();
        solvedBases[i] =
            params.preserveJointOrientation
                ? O::TransportedBasis(
                      bases[i], (original[i + 1] - original[i]).GetNormalized(),
                      aim, angularEpsilon)
                : O::AimedBasis(bases[i], aim, angularEpsilon);
    }
    if (params.mode == kSingleChain && !params.preserveJointOrientation) {
        solvedBases.back() = effectorBasis;
    } else {
        const Double currentTerminalSegment =
            original.back() - original[original.size() - 2];
        const Double currentTerminal =
            currentTerminalSegment / currentTerminalSegment.GetLength();
        const Double solvedTerminalSegment =
            solved.back() - solved[solved.size() - 2];
        const Double solvedTerminal =
            solvedTerminalSegment / solvedTerminalSegment.GetLength();
        solvedBases.back() = O::TransportedBasis(
            bases.back(), currentTerminal, solvedTerminal, angularEpsilon);
    }

    std::vector<F> result(current.size());
    if (weight >= 1.0) {
        // The full-strength candidate directly: re-blending at weight 1 is
        // equivalent but not bit-exact.
        for (size_t i = 0; i < result.size(); ++i)
            result[i] = O::template FrameFromBasis<F>(
                bases[i], solved[i], solvedBases[i].x, solvedBases[i].up);
    } else {
        // Blend directions, not positions, and re-accumulate the lengths: a
        // rigid chain at every partial weight.
        std::vector<Double> blended(current.size());
        blended[0] = original[0];
        for (size_t i = 0; i < lengths.size(); ++i) {
            const Double currentDirection =
                (original[i + 1] - original[i]) / lengths[i];
            const Double solvedSegment = solved[i + 1] - solved[i];
            const Double solvedDirection =
                solvedSegment / solvedSegment.GetLength();
            const Double direction =
                O::BlendDirection(currentDirection, solvedDirection, weight,
                                  bases[i].up, angularEpsilon);
            blended[i + 1] = blended[i] + direction * lengths[i] *
                                              (1 + weight * (growth - 1));
        }
        for (size_t i = 0; i < result.size(); ++i)
            result[i] = O::template BlendEndFrame<F>(
                bases[i], solvedBases[i], blended[i], weight, angularEpsilon);
    }

    if (growth != 1)
        for (size_t i = 0; i + 1 < result.size(); ++i) {
            const Double segment =
                result[i + 1].points[0] - result[i].points[0];
            RigExecScaleFrameAlong(result[i].points.data(),
                                   segment.GetNormalized(),
                                   segment.GetLength() / lengths[i]);
        }

    for (const F &frame : result)
        if (!IsFiniteFrame(frame))
            return Degenerate(current);
    return result;
}

} // namespace rigExec

#endif
