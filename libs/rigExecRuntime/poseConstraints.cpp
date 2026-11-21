// Runtime constraints, source resolution, and single-chain IK.

#include "rigExecSchema/schemaNames.h"
#include "poseEvalHost.h"
#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Single-chain IK (singleChainIk.cpp).

enum : int {
    _RrFabrikIterations = 128,
};

enum class _RrSingleChainIkMode : uint8_t {
    RotatePlane = 0,
    SingleChain = 1,
};

struct _RrSingleChainIkParams {
    _RrSingleChainIkMode mode = _RrSingleChainIkMode::RotatePlane;
    RrVec3d pole{0.0, 1.0, 0.0};
    double twistDegrees = 0.0;
    double stretch = 0;
    double weight = 1.0;
};

bool
_RrIkVecFinite(const RrVec3d &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
_RrIkFrameFinite(const RrPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const RrVec3d &point : frame.points) {
        if (!_RrIkVecFinite(point)) {
            return false;
        }
    }
    return true;
}

std::vector<RrPointFrame>
_RrIkDegenerate(const std::vector<RrPointFrame> &frames)
{
    std::vector<RrPointFrame> failed = frames;
    for (RrPointFrame &frame : failed) {
        frame.flags |= RrPointFrameDegenerate;
    }
    return failed;
}

RrVec3d
_RrIkProjectPerpendicular(const RrVec3d &v, const RrVec3d &axis)
{
    return v - axis * RrDot(v, axis);
}

RrVec3d
_RrIkWorldAxisLeastParallel(const RrVec3d &axis)
{
    const RrVec3d world[3] = {
        RrVec3d(1.0, 0.0, 0.0),
        RrVec3d(0.0, 1.0, 0.0),
        RrVec3d(0.0, 0.0, 1.0),
    };
    int best = 0;
    double bestAlignment = std::abs(RrDot(world[0], axis));
    for (int i = 1; i < 3; ++i) {
        const double alignment = std::abs(RrDot(world[i], axis));
        if (alignment < bestAlignment) {
            best = i;
            bestAlignment = alignment;
        }
    }
    return world[best];
}

RrVec3d
_RrIkStablePerpendicular(const RrVec3d &axis, const RrVec3d &first,
                         const RrVec3d &second, double epsilon)
{
    auto normalized = [](const RrVec3d &candidate) {
        const double length = candidate.GetLength();
        return std::isfinite(length) && length > 0.0 ? candidate / length
                                                     : RrVec3d(0.0);
    };
    RrVec3d result = _RrIkProjectPerpendicular(normalized(first), axis);
    if (result.GetLength() <= epsilon) {
        result = _RrIkProjectPerpendicular(normalized(second), axis);
    }
    if (result.GetLength() <= epsilon) {
        result =
            _RrIkProjectPerpendicular(_RrIkWorldAxisLeastParallel(axis), axis);
    }
    return result.GetNormalized();
}

RrVec3d
_RrIkRotate(const RrVec3d &v, const RrVec3d &unitAxis, double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return v * c + RrCross(unitAxis, v) * s +
           unitAxis * RrDot(unitAxis, v) * (1.0 - c);
}

RrVec3d
_RrIkRotateToward(const RrVec3d &v, const RrVec3d &fromAxis,
                  const RrVec3d &toAxis, double weight, const RrVec3d &fallback,
                  double epsilon)
{
    const RrVec3d from = fromAxis.GetNormalized();
    const RrVec3d to = toAxis.GetNormalized();
    const double dot = std::min(std::max(RrDot(from, to), -1.0), 1.0);
    RrVec3d rotationAxis = RrCross(from, to);
    const double sinAngle = rotationAxis.GetLength();
    double angle = 0.0;
    if (sinAngle > epsilon) {
        rotationAxis /= sinAngle;
        angle = std::atan2(sinAngle, dot);
    } else if (dot < 0.0) {
        rotationAxis =
            _RrIkStablePerpendicular(from, fallback, RrVec3d(0.0), epsilon);
        angle = _RrPi;
    } else {
        return v;
    }
    return _RrIkRotate(v, rotationAxis, angle * weight);
}

RrVec3d
_RrIkBlendDirection(const RrVec3d &from, const RrVec3d &to, double weight,
                    const RrVec3d &fallback, double epsilon)
{
    RrVec3d result =
        _RrIkRotateToward(from, from, to, weight, fallback, epsilon);
    const double length = result.GetLength();
    if (length <= epsilon || !_RrIkVecFinite(result)) {
        return from.GetNormalized();
    }
    return result / length;
}

struct _RrIkFrameBasis {
    RrVec3d x;
    RrVec3d up;
    double xLength = 0.0;
    double yLength = 0.0;
    double zLength = 0.0;
    double handedness = 1.0;
};

bool
_RrIkExtractBasis(const RrPointFrame &frame, double epsilon,
                  _RrIkFrameBasis *basis)
{
    const RrVec3d x = frame.points[1] - frame.points[0];
    const RrVec3d y = frame.points[2] - frame.points[0];
    const RrVec3d z = frame.points[3] - frame.points[0];
    const double xLength = x.GetLength();
    const double yLength = y.GetLength();
    const double zLength = z.GetLength();
    const double frameScale = std::max({xLength, yLength, zLength});
    if (!std::isfinite(frameScale) || frameScale <= 0.0 ||
        xLength <= frameScale * epsilon || yLength <= frameScale * epsilon ||
        zLength <= frameScale * epsilon) {
        return false;
    }

    const RrVec3d unitX = x / xLength;
    RrVec3d up = _RrIkProjectPerpendicular(y / yLength, unitX);
    if (up.GetLength() <= epsilon) {
        return false;
    }
    up.Normalize();

    const double determinant =
        RrDot(RrCross(x / xLength, y / yLength), z / zLength);
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-10) {
        return false;
    }

    basis->x = unitX;
    basis->up = up;
    basis->xLength = xLength;
    basis->yLength = yLength;
    basis->zLength = zLength;
    basis->handedness = determinant < 0.0 ? -1.0 : 1.0;
    return true;
}

RrPointFrame
_RrIkFrameFromBasis(const _RrIkFrameBasis &basis, const RrVec3d &origin,
                    const RrVec3d &unitX, const RrVec3d &unitUp)
{
    const RrVec3d side =
        basis.handedness * RrCross(unitX, unitUp).GetNormalized();
    RrPointFrame frame;
    frame.points[0] = origin;
    frame.points[1] = origin + unitX * basis.xLength;
    frame.points[2] = origin + unitUp * basis.yLength;
    frame.points[3] = origin + side * basis.zLength;
    frame.flags = RrPointFrameValid;
    if (basis.handedness < 0.0) {
        frame.flags |= RrPointFrameReflected;
    }
    return frame;
}

_RrIkFrameBasis
_RrIkAimedBasis(const _RrIkFrameBasis &basis, const RrVec3d &unitAim,
                double epsilon)
{
    RrVec3d up =
        _RrIkRotateToward(basis.up, basis.x, unitAim, 1.0, basis.up, epsilon);
    up = _RrIkProjectPerpendicular(up, unitAim);
    if (up.GetLength() <= epsilon) {
        up = _RrIkStablePerpendicular(unitAim, basis.up, basis.x, epsilon);
    } else {
        up.Normalize();
    }
    _RrIkFrameBasis aimed = basis;
    aimed.x = unitAim;
    aimed.up = up;
    return aimed;
}

_RrIkFrameBasis
_RrIkTransportedBasis(const _RrIkFrameBasis &basis,
                      const RrVec3d &fromDirection, const RrVec3d &toDirection,
                      double epsilon)
{
    _RrIkFrameBasis transported = basis;
    transported.x = _RrIkRotateToward(basis.x, fromDirection, toDirection, 1.0,
                                      basis.up, epsilon);
    transported.x.Normalize();
    transported.up = _RrIkRotateToward(basis.up, fromDirection, toDirection,
                                       1.0, basis.up, epsilon);
    transported.up = _RrIkProjectPerpendicular(transported.up, transported.x);
    if (transported.up.GetLength() <= epsilon) {
        transported.up =
            _RrIkStablePerpendicular(transported.x, basis.up, basis.x, epsilon);
    } else {
        transported.up.Normalize();
    }
    return transported;
}

double
_RrIkSignedAngle(const RrVec3d &from, const RrVec3d &to,
                 const RrVec3d &unitAxis)
{
    return std::atan2(RrDot(unitAxis, RrCross(from, to)),
                      std::min(std::max(RrDot(from, to), -1.0), 1.0));
}

RrPointFrame
_RrIkBlendEndFrame(const _RrIkFrameBasis &input,
                   const _RrIkFrameBasis &effector, const RrVec3d &origin,
                   double weight, double epsilon)
{
    const RrVec3d x =
        _RrIkBlendDirection(input.x, effector.x, weight, input.up, epsilon);

    RrVec3d carriedAtTarget = _RrIkRotateToward(input.up, input.x, effector.x,
                                                1.0, input.up, epsilon);
    carriedAtTarget = _RrIkProjectPerpendicular(carriedAtTarget, effector.x);
    if (carriedAtTarget.GetLength() <= epsilon) {
        carriedAtTarget = _RrIkStablePerpendicular(effector.x, input.up,
                                                   effector.up, epsilon);
    } else {
        carriedAtTarget.Normalize();
    }
    const double roll =
        _RrIkSignedAngle(carriedAtTarget, effector.up, effector.x);

    RrVec3d up = _RrIkRotateToward(input.up, input.x, effector.x, weight,
                                   input.up, epsilon);
    up = _RrIkProjectPerpendicular(up, x);
    if (up.GetLength() <= epsilon) {
        up = _RrIkStablePerpendicular(x, input.up, effector.up, epsilon);
    } else {
        up.Normalize();
    }
    up = _RrIkRotate(up, x, roll * weight).GetNormalized();
    return _RrIkFrameFromBasis(input, origin, x, up);
}

RrVec3d
_RrIkDirectionOrFallback(const RrVec3d &direction, const RrVec3d &fallback,
                         double positionEpsilon, double angularEpsilon)
{
    const double directionLength = direction.GetLength();
    if (directionLength > positionEpsilon) {
        return direction / directionLength;
    }
    const double fallbackLength = fallback.GetLength();
    if (fallbackLength > angularEpsilon) {
        return fallback / fallbackLength;
    }
    return RrVec3d(1.0, 0.0, 0.0);
}

std::vector<RrVec3d>
_RrIkSolvePositions(const std::vector<RrVec3d> &original,
                    const std::vector<double> &lengths,
                    const std::vector<_RrIkFrameBasis> &bases,
                    const _RrIkFrameBasis &effectorBasis, const RrVec3d &goal,
                    const _RrSingleChainIkParams &params,
                    double positionEpsilon, double angularEpsilon)
{
    const size_t count = original.size();
    const RrVec3d root = original.front();
    const double totalLength = [&]() {
        double total = 0.0;
        for (double length : lengths)
            total += length;
        return total;
    }();

    RrVec3d rootToGoal = goal - root;
    const double goalDistance = rootToGoal.GetLength();
    RrVec3d solveAxis;
    if (goalDistance > positionEpsilon) {
        solveAxis = rootToGoal / goalDistance;
    } else {
        solveAxis = original.back() - root;
        if (solveAxis.GetLength() <= positionEpsilon) {
            solveAxis = original[1] - root;
        }
        solveAxis = _RrIkDirectionOrFallback(solveAxis, bases.front().x,
                                             positionEpsilon, angularEpsilon);
    }

    if (goalDistance >= totalLength - positionEpsilon &&
        goalDistance > positionEpsilon) {
        std::vector<RrVec3d> extended(count);
        extended[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            extended[i + 1] = extended[i] + solveAxis * lengths[i];
        }
        return extended;
    }

    std::vector<RrVec3d> positions = original;
    RrVec3d planeUp;
    RrVec3d planeNormal;
    if (params.mode == _RrSingleChainIkMode::RotatePlane) {
        RrVec3d poleOffset = params.pole - root;
        if (poleOffset.GetLength() <= positionEpsilon) {
            poleOffset = RrVec3d(0.0);
        }
        planeUp = _RrIkStablePerpendicular(solveAxis, poleOffset,
                                           bases.front().up, angularEpsilon);
        planeUp = _RrIkRotate(planeUp, solveAxis,
                              params.twistDegrees * (_RrPi / 180.0));
        planeUp.Normalize();
    } else {
        planeUp = _RrIkStablePerpendicular(solveAxis, effectorBasis.up,
                                           effectorBasis.x, angularEpsilon);
    }
    planeNormal = RrCross(solveAxis, planeUp).GetNormalized();

    for (size_t i = 1; i + 1 < count; ++i) {
        const RrVec3d relative = positions[i] - root;
        const double along = RrDot(relative, solveAxis);
        const double towardPlane = std::abs(RrDot(relative, planeUp));
        positions[i] = root + solveAxis * along + planeUp * towardPlane;
    }

    double maxOffAxis = 0.0;
    for (size_t i = 1; i + 1 < count; ++i) {
        maxOffAxis =
            std::max(maxOffAxis,
                     _RrIkProjectPerpendicular(positions[i] - root, solveAxis)
                         .GetLength());
    }
    if (count > 2 && maxOffAxis <= positionEpsilon * 8.0 &&
        goalDistance < totalLength - positionEpsilon) {
        const RrVec3d bend = planeUp;
        double distance = 0.0;
        const double amplitude =
            std::max(totalLength * 0.05, positionEpsilon * 16.0);
        for (size_t i = 1; i + 1 < count; ++i) {
            distance += lengths[i - 1];
            positions[i] +=
                bend * (amplitude * std::sin(_RrPi * distance / totalLength));
        }
    }

    std::vector<RrVec3d> fallback(lengths.size());
    for (size_t i = 0; i < lengths.size(); ++i) {
        RrVec3d direction = positions[i + 1] - positions[i];
        direction -= planeNormal * RrDot(direction, planeNormal);
        fallback[i] = _RrIkDirectionOrFallback(direction, solveAxis,
                                               positionEpsilon, angularEpsilon);
    }

    std::vector<RrVec3d> best = positions;
    double bestError = std::numeric_limits<double>::infinity();
    const double tolerance =
        std::max(positionEpsilon * 8.0, totalLength * 1e-12);
    for (int iteration = 0; iteration < _RrFabrikIterations; ++iteration) {
        positions.back() = goal;

        for (size_t i = count - 1; i-- > 0;) {
            const RrVec3d direction = _RrIkDirectionOrFallback(
                positions[i] - positions[i + 1], -fallback[i], positionEpsilon,
                angularEpsilon);
            positions[i] = positions[i + 1] + direction * lengths[i];
        }

        positions[0] = root;
        for (size_t i = 0; i < lengths.size(); ++i) {
            const RrVec3d direction = _RrIkDirectionOrFallback(
                positions[i + 1] - positions[i], fallback[i], positionEpsilon,
                angularEpsilon);
            positions[i + 1] = positions[i] + direction * lengths[i];
            fallback[i] = direction;
        }

        const double error = (positions.back() - goal).GetLength();
        if (error < bestError) {
            bestError = error;
            best = positions;
        }
        if (error <= tolerance) {
            break;
        }
    }
    return best;
}

std::vector<RrPointFrame>
_RrSolveSingleChainIk(const std::vector<RrPointFrame> &currentFrames,
                      const RrPointFrame &effectorFrame,
                      const _RrSingleChainIkParams &params)
{
    if (!std::isfinite(params.weight)) {
        return _RrIkDegenerate(currentFrames);
    }
    const double weight = std::min(std::max(params.weight, 0.0), 1.0);
    if (weight <= 0.0) {
        return currentFrames;
    }
    if (currentFrames.size() < 2) {
        return _RrIkDegenerate(currentFrames);
    }
    if (params.mode != _RrSingleChainIkMode::RotatePlane &&
        params.mode != _RrSingleChainIkMode::SingleChain) {
        return _RrIkDegenerate(currentFrames);
    }
    if (!_RrIkVecFinite(effectorFrame.points[0])) {
        return _RrIkDegenerate(currentFrames);
    }
    if (params.mode == _RrSingleChainIkMode::RotatePlane &&
        (!_RrIkVecFinite(params.pole) || !std::isfinite(params.twistDegrees))) {
        return _RrIkDegenerate(currentFrames);
    }
    for (const RrPointFrame &frame : currentFrames) {
        if (!_RrIkFrameFinite(frame)) {
            return _RrIkDegenerate(currentFrames);
        }
    }

    std::vector<RrVec3d> original(currentFrames.size());
    std::vector<double> lengths(currentFrames.size() - 1);
    double segmentScale = 0.0;
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        original[i] = currentFrames[i].points[0];
        if (i + 1 < currentFrames.size()) {
            lengths[i] =
                (currentFrames[i + 1].points[0] - original[i]).GetLength();
            segmentScale = std::max(segmentScale, lengths[i]);
        }
    }
    if (!std::isfinite(segmentScale) || segmentScale <= 0.0) {
        return _RrIkDegenerate(currentFrames);
    }
    const double positionEpsilon = segmentScale * 1e-10;
    constexpr double angularEpsilon = 1e-12;

    std::vector<_RrIkFrameBasis> bases(currentFrames.size());
    for (size_t i = 0; i < currentFrames.size(); ++i) {
        if (!_RrIkExtractBasis(currentFrames[i], angularEpsilon, &bases[i])) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    _RrIkFrameBasis effectorBasis = bases.back();
    if (params.mode == _RrSingleChainIkMode::SingleChain) {
        if (!_RrIkFrameFinite(effectorFrame) ||
            !_RrIkExtractBasis(effectorFrame, angularEpsilon, &effectorBasis)) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    for (double length : lengths) {
        if (!std::isfinite(length) || length <= positionEpsilon) {
            return _RrIkDegenerate(currentFrames);
        }
    }

    if (!std::isfinite(params.stretch) || params.stretch < 0 ||
        params.stretch > 1)
        return _RrIkDegenerate(currentFrames);
    double totalLength = 0;
    for (double length : lengths)
        totalLength += length;
    const double growth =
        1 +
        params.stretch *
            std::max(0.0,
                     (effectorFrame.points[0] - original.front()).GetLength() /
                             totalLength -
                         1);
    auto solveLengths = lengths;
    for (double &length : solveLengths)
        length *= growth;
    const std::vector<RrVec3d> solved = _RrIkSolvePositions(
        original, solveLengths, bases, effectorBasis, effectorFrame.points[0],
        params, positionEpsilon, angularEpsilon);
    if (solved.size() != currentFrames.size()) {
        return _RrIkDegenerate(currentFrames);
    }

    std::vector<_RrIkFrameBasis> solvedBases = bases;
    for (size_t i = 0; i + 1 < solvedBases.size(); ++i) {
        const RrVec3d solvedSegment = solved[i + 1] - solved[i];
        const RrVec3d aim = solvedSegment / solvedSegment.GetLength();
        solvedBases[i] = _RrIkAimedBasis(bases[i], aim, angularEpsilon);
    }
    if (params.mode == _RrSingleChainIkMode::SingleChain) {
        solvedBases.back() = effectorBasis;
    } else {
        const RrVec3d currentTerminalSegment =
            original.back() - original[original.size() - 2];
        const RrVec3d currentTerminal =
            currentTerminalSegment / currentTerminalSegment.GetLength();
        const RrVec3d solvedTerminalSegment =
            solved.back() - solved[solved.size() - 2];
        const RrVec3d solvedTerminal =
            solvedTerminalSegment / solvedTerminalSegment.GetLength();
        solvedBases.back() = _RrIkTransportedBasis(
            bases.back(), currentTerminal, solvedTerminal, angularEpsilon);
    }

    std::vector<RrPointFrame> result(currentFrames.size());
    if (weight >= 1.0) {
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _RrIkFrameFromBasis(
                bases[i], solved[i], solvedBases[i].x, solvedBases[i].up);
        }
    } else {
        std::vector<RrVec3d> blended(currentFrames.size());
        blended[0] = original[0];
        for (size_t i = 0; i < lengths.size(); ++i) {
            const RrVec3d currentDirection =
                (original[i + 1] - original[i]) / lengths[i];
            const RrVec3d solvedSegment = solved[i + 1] - solved[i];
            const RrVec3d solvedDirection =
                solvedSegment / solvedSegment.GetLength();
            const RrVec3d direction =
                _RrIkBlendDirection(currentDirection, solvedDirection, weight,
                                    bases[i].up, angularEpsilon);
            blended[i + 1] = blended[i] + direction * lengths[i] *
                                              (1 + weight * (growth - 1));
        }
        for (size_t i = 0; i < result.size(); ++i) {
            result[i] = _RrIkBlendEndFrame(bases[i], solvedBases[i], blended[i],
                                           weight, angularEpsilon);
        }
    }

    if (growth != 1)
        for (size_t i = 0; i + 1 < result.size(); ++i) {
            const auto bone = result[i + 1].points[0] - result[i].points[0];
            RigExecScaleFrameAlong(result[i].points.data(),
                                   bone.GetNormalized(),
                                   bone.GetLength() / lengths[i]);
        }
    for (const RrPointFrame &frame : result) {
        if (!_RrIkFrameFinite(frame)) {
            return _RrIkDegenerate(currentFrames);
        }
    }
    return result;
}

// RigExecPrepareRestDerivedIkChain (rigEvaluator.cpp): rest segment
// lengths laid along the current chain's directions.
bool
_RrPrepareRestDerivedIkChain(const std::vector<RrPointFrame> &current,
                             const std::vector<RrPointFrame> &rest,
                             std::vector<RrPointFrame> *prepared)
{
    if (current.size() != rest.size() || current.empty()) {
        return false;
    }
    bool usableRestLayout = true;
    for (size_t i = 1; i < rest.size(); ++i) {
        const double segmentLength =
            (rest[i].points[0] - rest[i - 1].points[0]).GetLength();
        if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
            usableRestLayout = false;
            break;
        }
    }
    const std::vector<RrPointFrame> &lengthReference =
        usableRestLayout ? rest : current;
    prepared->clear();
    prepared->reserve(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        if (!RrFrameUsable(current[i]) || !RrFrameUsable(rest[i])) {
            return false;
        }
        RrVec3d origin = current[i].points[0];
        if (i > 0) {
            RrMat4d parentRest = _RrIdentity();
            RrMat4d parentPrepared = _RrIdentity();
            if (!RrPointsToMatrix(RrIdentityLandmarks(),
                                  lengthReference[i - 1].points, &parentRest) ||
                !RrPointsToMatrix(RrIdentityLandmarks(),
                                  prepared->back().points, &parentPrepared)) {
                return false;
            }
            origin = parentPrepared.TransformAffine(
                parentRest.GetInverse().TransformAffine(
                    lengthReference[i].points[0]));
        }

        RrPointFrame frame = current[i];
        frame.points[0] = origin;
        for (size_t axis = 1; axis < frame.points.size(); ++axis) {
            RrVec3d direction = current[i].points[axis] - current[i].points[0];
            const double directionLength = direction.GetLength();
            const double length =
                (lengthReference[i].points[axis] - lengthReference[i].points[0])
                    .GetLength();
            if (!std::isfinite(length) || length <= 0.0 ||
                !std::isfinite(directionLength) || directionLength <= 0.0) {
                return false;
            }
            direction /= directionLength;
            frame.points[axis] = origin + direction * length;
        }
        if (!RrFrameUsable(frame)) {
            return false;
        }
        prepared->push_back(frame);
    }
    return true;
}

// The Constraint step (bakedPose.cpp).

} // namespace

// RrPoseState::solveIkChain (poseEvalHost.h): the single-chain IK
// core, called from pose.h with gathered frames. The core itself
// stays here until the Solve lift unifies it.
bool
RrPoseState::solveIkChain(size_t object, const std::vector<RrPointFrame> &chain,
                          const std::vector<RrPointFrame> &rests,
                          const RrPointFrame &effector, int mode,
                          double stretch, double weight, const RrVec3d &pole,
                          double twistDegrees, bool useAnimatedTs,
                          std::vector<RrPointFrame> *solved)
{
    (void)object;
    std::vector<RrPointFrame> prepared;
    const std::vector<RrPointFrame> *solveChain = &chain;
    if (!useAnimatedTs) {
        if (!_RrPrepareRestDerivedIkChain(chain, rests, &prepared)) {
            return false;
        }
        solveChain = &prepared;
    }
    _RrSingleChainIkParams params;
    params.mode = _RrSingleChainIkMode(mode);
    params.pole = pole;
    params.twistDegrees = twistDegrees;
    params.stretch = stretch;
    params.weight = weight;
    *solved = _RrSolveSingleChainIk(*solveChain, effector, params);
    return true;
}

namespace runtimePoseDetail {

bool _RrRunConstraintStep(RrProgram *program, size_t step, std::string *error);

bool
_RrRunConstraintStep(RrProgram *program, size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireStep &wire = (*program->steps)[step];
    if (wire.object < 0 ||
        size_t(wire.object) >= program->poses->commits.size() ||
        size_t(wire.object) >= store.commits.size() ||
        size_t(wire.object) >= program->poses->walkSteps.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    const RigExecWireCommit &wireCommit =
        program->poses->commits[size_t(wire.object)];
    RrCommitScratch &commit = store.commits[size_t(wire.object)];
    const int walkIndex = program->poses->walkSteps[size_t(wire.object)].index;
    if (walkIndex < 0 ||
        size_t(walkIndex) >= program->poses->constraints.size() ||
        size_t(walkIndex) >= scratch->weightScratch.size() ||
        size_t(walkIndex) >= scratch->weightError.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no constraint";
        }
        return false;
    }
    const size_t ci = size_t(walkIndex);
    const RigExecWireConstraint &c = program->poses->constraints[ci];
    if (ci < store.matchValid.size())
        store.matchValid[ci] = 0;
    commit.abandoned = true;
    std::fill(commit.present.begin(), commit.present.end(), 0);

    const auto finish = [&]() {
        if (wireCommit.split) {
            return true;
        }
        RrPoseHost host(program, step);
        const eval::StepResult result =
            eval::RunFinishCommit(host, size_t(wire.object));
        if (result.outcome == eval::Outcome::Applied) {
            return true;
        }
        if (error) {
            const char *what = " names no commit";
            if (result.detail == eval::pose_detail::kPoseDetailNoChunk) {
                what = " names no chunk";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoSlot) {
                what = " names no slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoVersion) {
                what = " names no version";
            }
            *error = _RrStepHead(program, step) + what;
        }
        return false;
    };

    const int deltaBase = c.deltaBase;
    if (size_t(wire.object) >= scratch->recordAfter.size() ||
        size_t(wire.object) >= scratch->recordEveryTarget.size()) {
        if (error) {
            *error = _RrStepHead(program, step) + " names no commit";
        }
        return false;
    }
    scratch->recordAfter[size_t(wire.object)] = 1;
    scratch->recordEveryTarget[size_t(wire.object)] = 1;
    if (deltaBase >= 0) {
        if (size_t(deltaBase) >= scratch->deltaPresent.size() ||
            size_t(deltaBase) >= store.deltaPresent.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no delta path";
            }
            return false;
        }
        scratch->deltaPresent[size_t(deltaBase)] = 0;
        // Published for the geometry fold (the store pair mirrors the
        // baked B.deltaValues/deltaPresent the fold consumes).
        store.deltaPresent[size_t(deltaBase)] = 0;
    }
    if (ci < store.matchShownMask.size())
        store.matchShownMask[ci] = 0;
    RrPoseHost host(program, step);
    eval::StepRef ref;
    ref.kind = uint32_t(wire.kind);
    ref.object = wire.object;
    ref.part = wire.part;
    ref.slot = step;
    const eval::StepResult result = eval::RunValueStep(host, ref);
    if (result.outcome != eval::Outcome::Applied) {
        if (error) {
            const char *what = " names no commit";
            if (result.detail == eval::pose_detail::kPoseDetailNoConstraint) {
                what = " names an incomplete IK/FK match";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoChain) {
                what = " names no chain target";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoRest) {
                what = " names no rest slot";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoPole) {
                what = " names no pole object";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoVersion) {
                what = " names no fin version";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoDeltaPath) {
                what = " names no delta path";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoCandidate) {
                what = " names no candidate slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoTarget) {
                what = " names no target slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoArrays) {
                what = " names no constraint arrays";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoSource) {
                what = " names no constraint source";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoSpace) {
                what = " names no space slot";
            } else if (result.detail ==
                       eval::pose_detail::kPoseDetailNoWindow) {
                what = " names no harmonics window";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoSlot) {
                what = " names no slot";
            } else if (result.detail == eval::pose_detail::kPoseDetailNoChunk) {
                what = " names no chunk";
            }
            *error = _RrStepHead(program, step) + what;
        }
        return false;
    }
    return finish();
}

} // namespace runtimePoseDetail

} // namespace rigExec
