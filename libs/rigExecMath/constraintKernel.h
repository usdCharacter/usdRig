// The FBX-equivalent Position, Rotation, Scale, Parent and Aim constraints over
// four-point frames, shared by the native (Gf) path, the runtime (Rr) and the
// constraint ops.
//
// Host types come from RigExecDualQuatMath<Point, Double> plus
// RigExecConstraintMath<Point, Double>, which names the rotation type and
// the two members Gf and the mirror spell differently. Frames, sources and
// parameter records are duck-typed: a frame has `points` (four Double) and
// `flags` with the shared bit layout below.
#ifndef RIGEXEC_MATH_CONSTRAINT_KERNEL_H
#define RIGEXEC_MATH_CONSTRAINT_KERNEL_H

#include "envelope.h"
#include "frameParamsKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace rigExec {

template <class Point, class Double> struct RigExecConstraintMath;

namespace constraintDetail {

// Frame flag bits every frame type here shares.
constexpr uint32_t kFrameValid = 1u << 0;
constexpr uint32_t kFrameDegenerate = 1u << 1;
constexpr uint32_t kFrameReflected = 1u << 2;
constexpr uint32_t kFrameAffine = 1u << 3;

template <class V>
bool
IsFinite(const V &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

template <class F>
bool
HasFinitePoints(const F &frame)
{
    for (const auto &point : frame.points) {
        if (!IsFinite(point)) {
            return false;
        }
    }
    return true;
}

template <class F>
bool
IsUsableFrame(const F &frame)
{
    return (frame.flags & kFrameValid) && !(frame.flags & kFrameDegenerate) &&
           HasFinitePoints(frame);
}

template <class F>
F
Failure(const F &input)
{
    F failed = input;
    failed.flags |= kFrameDegenerate;
    return failed;
}

// Any of the six Euler orders, XYZ through ZYX in that order.
template <class Order>
bool
IsValidOrder(Order order)
{
    const int i = static_cast<int>(order);
    return i >= 0 && i <= 5;
}

template <class V>
bool
NormalizeDirection(V *direction, double eps = 1e-12)
{
    if (!IsFinite(*direction)) {
        return false;
    }
    const double largest =
        std::max({std::abs((*direction)[0]), std::abs((*direction)[1]),
                  std::abs((*direction)[2])});
    if (largest < eps) {
        return false;
    }
    *direction /= largest;
    const double length = direction->GetLength();
    if (!std::isfinite(length) || length < eps) {
        return false;
    }
    *direction /= length;
    return IsFinite(*direction);
}

template <class Order>
std::array<int, 3>
OrderIndices(Order order)
{
    constexpr std::array<int, 3> kIndices[6] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    return IsValidOrder(order) ? kIndices[static_cast<int>(order)]
                               : kIndices[0];
}

template <class Point, class Double>
std::array<Double, 4>
IdentityLandmarks()
{
    return {Double(0, 0, 0), Double(1, 0, 0), Double(0, 1, 0), Double(0, 0, 1)};
}

template <class Point, class Double, class Order>
typename RigExecDualQuatMath<Point, Double>::Quat
QuatFromEulerDegrees(const Double &degrees, Order order)
{
    using Mat4 = typename RigExecDualQuatMath<Point, Double>::Mat4;
    using Rotation = typename RigExecConstraintMath<Point, Double>::Rotation;
    const Double axes[3] = {Double(1, 0, 0), Double(0, 1, 0), Double(0, 0, 1)};
    Mat4 matrix(1.0);
    for (const int axis : OrderIndices(order)) {
        matrix =
            matrix * Mat4(Rotation(axes[axis], degrees[axis]), Double(0.0));
    }
    return matrix.ExtractRotation().GetQuat().GetNormalized();
}

template <class Point, class Double, class Order>
Double
EulerDegreesFromQuat(
    const typename RigExecDualQuatMath<Point, Double>::Quat &rotation,
    Order order)
{
    using Rotation = typename RigExecConstraintMath<Point, Double>::Rotation;
    const Double axes[3] = {Double(1, 0, 0), Double(0, 1, 0), Double(0, 0, 1)};
    const std::array<int, 3> indices = OrderIndices(order);
    // Decompose(a, b, c) describes row-matrix factors in the reverse order
    // Rc * Rb * Ra, so pass the application sequence reversed to invert
    // QuatFromEulerDegrees exactly.
    const Double ordered = Rotation(rotation).Decompose(
        axes[indices[2]], axes[indices[1]], axes[indices[0]]);
    Double result(0.0);
    result[indices[2]] = ordered[0];
    result[indices[1]] = ordered[1];
    result[indices[0]] = ordered[2];
    return result;
}

inline double
ShortestDegrees(double degrees)
{
    double wrapped = std::fmod(degrees + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    wrapped -= 180.0;
    // Resolve the exact half-turn tie without depending on fmod's sign.
    return (wrapped == -180.0 && degrees > 0.0) ? 180.0 : wrapped;
}

template <class Mask>
bool
Affects(const Mask &mask, int axis)
{
    return axis == 0 ? mask.x : axis == 1 ? mask.y : mask.z;
}

// Shear belongs to the linear block scale does, so a constraint governing
// all three scale axes governs it; a partial mask has no share of it, as
// each shear term mixes two axes.
template <class Mask, class Double, class Params>
void
BlendGovernedShear(const Mask &scaleMask, const Double &targetShear,
                   double weight, Params *params)
{
    if (!scaleMask.x || !scaleMask.y || !scaleMask.z) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        params->shear[i] =
            RigExecBlendEnvelope(params->shear[i], targetShear[i], weight);
    }
}

template <class Point, class Double, class F>
bool
DecomposeFrame(const F &frame,
               RigExecTransformParamsKernelT<Point, Double> *params)
{
    if (!IsUsableFrame(frame) ||
        !RigExecPointsToParamsKernel<Point, Double>(
            IdentityLandmarks<Point, Double>(), frame.points, 2, params)) {
        return false;
    }
    return IsFinite(params->translation) && IsFinite(params->scale) &&
           IsFinite(params->shear) &&
           std::isfinite(params->rotation.GetReal()) &&
           IsFinite(params->rotation.GetImaginary());
}

// RigExecMatrixToPoints: the identity landmarks carried by \p matrix,
// classified reflected and degenerate.
template <class Point, class Double, class F>
F
MatrixToFrame(const typename RigExecDualQuatMath<Point, Double>::Mat4 &matrix)
{
    using Mat3 = typename RigExecDualQuatMath<Point, Double>::Mat3;
    const std::array<Double, 4> rest = IdentityLandmarks<Point, Double>();
    F frame;
    for (size_t i = 0; i < 4; ++i) {
        frame.points[i] = matrix.TransformAffine(rest[i]);
    }
    frame.flags = kFrameValid;
    const auto &p = frame.points;
    const Mat3 posed(p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2],
                     p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2],
                     p[3][0] - p[0][0], p[3][1] - p[0][1], p[3][2] - p[0][2]);
    if (posed.GetDeterminant() < 0) {
        frame.flags |= kFrameReflected;
    }
    // The identity landmarks' epsilon: 1e-10 * max(1, unit lengths).
    const double eps = 1e-10;
    if (std::abs(posed.GetDeterminant()) <= eps * eps * eps) {
        frame.flags |= kFrameDegenerate;
    }
    return frame;
}

template <class Point, class Double, class F>
F
FrameFromParams(const F &input,
                const RigExecTransformParamsKernelT<Point, Double> &params)
{
    F result = MatrixToFrame<Point, Double, F>(
        RigExecParamsToMatrixKernel<Point, Double>(params));
    if (!HasFinitePoints(result)) {
        return Failure(input);
    }
    // An input classified affine stays classified affine when its shear is
    // carried through.
    result.flags |= input.flags & kFrameAffine;
    return result;
}

template <class F, class Mat4>
F
TransformFrame(const F &frame, const Mat4 &space)
{
    F out = frame;
    for (auto &p : out.points) {
        p = space.Transform(p);
    }
    return out;
}

inline bool
ValidateGlobalWeight(double weight, double *clamped)
{
    if (!std::isfinite(weight)) {
        return false;
    }
    *clamped = std::min(std::max(weight, 0.0), 1.0);
    return true;
}

template <class S>
bool
ValidateSourceWeight(const S &source)
{
    return std::isfinite(source.normalizedWeight) &&
           source.normalizedWeight >= 0.0;
}

template <class Double, class Mask>
Double
ApplyEulerDelta(const Double &inputEuler, const Double &targetEuler,
                const Mask &affect, double weight)
{
    Double output = inputEuler;
    for (int axis = 0; axis < 3; ++axis) {
        if (Affects(affect, axis)) {
            // At full strength take the target representation itself:
            // input + shortestDelta can differ bit for bit (190 versus -170).
            if (weight >= 1.0) {
                output[axis] = targetEuler[axis];
            } else {
                output[axis] += weight * ShortestDegrees(targetEuler[axis] -
                                                         inputEuler[axis]);
            }
        }
    }
    return output;
}

template <class V>
const V *
UpDirection(const std::optional<V> &up)
{
    return up ? &*up : nullptr;
}

template <class V>
const V *
UpDirection(const V *up)
{
    return up;
}

} // namespace constraintDetail

/// Position: the input translated toward the weighted mean source origin
/// plus offset, per affected axis. A zero global or total source weight
/// passes through; bad input fails with the input flagged degenerate.
template <class Point, class Double, class F, class S, class P>
F
RigExecApplyPositionConstraintKernel(const F &input,
                                     const std::vector<S> &sources,
                                     const P &params)
{
    using namespace constraintDetail;
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        return Failure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!IsUsableFrame(input) || !IsFinite(params.offset)) {
        return Failure(input);
    }

    Double target(0.0);
    double totalWeight = 0.0;
    for (const S &source : sources) {
        if (!ValidateSourceWeight(source)) {
            return Failure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!IsUsableFrame(source.frame)) {
            return Failure(input);
        }
        target += source.frame.points[0] * source.normalizedWeight;
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : Failure(input);
    }
    target = target / totalWeight + params.offset;
    if (!IsFinite(target)) {
        return Failure(input);
    }

    const Double origin = input.points[0];
    Double constrainedOrigin = origin;
    for (int axis = 0; axis < 3; ++axis) {
        if (Affects(params.affect, axis)) {
            constrainedOrigin[axis] =
                RigExecBlendEnvelope(origin[axis], target[axis], globalWeight);
        }
    }
    if (!IsFinite(constrainedOrigin)) {
        return Failure(input);
    }
    F output = input;
    if (globalWeight >= 1.0) {
        // At full strength the candidate owns its origin exactly; the other
        // landmarks keep the preceding linear part rather than a + (b - a).
        for (size_t point = 1; point < output.points.size(); ++point) {
            output.points[point] =
                constrainedOrigin + (input.points[point] - origin);
        }
        output.points[0] = constrainedOrigin;
    } else {
        const Double translation = constrainedOrigin - origin;
        for (auto &point : output.points) {
            point += translation;
        }
    }
    return output;
}

/// Rotation: the input's Euler angles moved toward the sources' mean
/// (shortest deltas from the first weighted source) plus offset, per
/// affected axis. A named carry (rigExec:space) is stripped before the
/// mask and restored after it.
template <class Point, class Double, class F, class S, class P>
F
RigExecApplyRotationConstraintKernel(const F &input,
                                     const std::vector<S> &sources,
                                     const P &params)
{
    using namespace constraintDetail;
    using Mat4 = typename RigExecDualQuatMath<Point, Double>::Mat4;
    using Params = RigExecTransformParamsKernelT<Point, Double>;
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        return Failure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!IsValidOrder(params.rotationOrder) ||
        !IsFinite(params.offsetDegrees)) {
        return Failure(input);
    }

    // Every failure hands back the original input; a rig naming no carry
    // never multiplies.
    Mat4 carryInverse(1.0);
    F measured;
    const F *in = &input;
    if (params.carry) {
        carryInverse = params.carry->GetInverse();
        measured = TransformFrame(input, carryInverse);
        in = &measured;
    }
    Params inputParams;
    if (!DecomposeFrame<Point, Double>(*in, &inputParams)) {
        return Failure(input);
    }
    const Double inputEuler = EulerDegreesFromQuat<Point, Double>(
        inputParams.rotation, params.rotationOrder);
    if (!IsFinite(inputEuler)) {
        return Failure(input);
    }
    Double sourceAnchor(0.0), weightedDelta(0.0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const S &source : sources) {
        if (!ValidateSourceWeight(source)) {
            return Failure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        Params sourceParams;
        const F sourceMeasured =
            params.carry ? TransformFrame(source.frame, carryInverse)
                         : source.frame;
        if (!DecomposeFrame<Point, Double>(sourceMeasured, &sourceParams)) {
            return Failure(input);
        }
        const Double sourceEuler = EulerDegreesFromQuat<Point, Double>(
            sourceParams.rotation, params.rotationOrder);
        if (!IsFinite(sourceEuler)) {
            return Failure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedDelta[axis] +=
                source.normalizedWeight *
                ShortestDegrees(sourceEuler[axis] - sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : Failure(input);
    }
    const Double targetEuler =
        sourceAnchor + weightedDelta / totalWeight + params.offsetDegrees;
    if (!IsFinite(weightedDelta) || !IsFinite(targetEuler)) {
        return Failure(input);
    }
    const Double outputEuler =
        ApplyEulerDelta(inputEuler, targetEuler, params.affect, globalWeight);
    inputParams.rotation =
        QuatFromEulerDegrees<Point, Double>(outputEuler, params.rotationOrder);
    const F solved = FrameFromParams<Point, Double>(input, inputParams);
    if (!params.carry || (solved.flags & kFrameDegenerate)) {
        return solved;
    }
    return TransformFrame(solved, *params.carry);
}

/// Scale: the input's scale moved toward the sources' mean plus offset per
/// affected axis, and with blendShear and all three axes, its shear too.
template <class Point, class Double, class F, class S, class P>
F
RigExecApplyScaleConstraintKernel(const F &input, const std::vector<S> &sources,
                                  const P &params)
{
    using namespace constraintDetail;
    using Params = RigExecTransformParamsKernelT<Point, Double>;
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        return Failure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!IsFinite(params.offset)) {
        return Failure(input);
    }

    Params inputParams;
    if (!DecomposeFrame<Point, Double>(input, &inputParams)) {
        return Failure(input);
    }
    Double targetScale(0.0), targetShear(0.0);
    double totalWeight = 0.0;
    for (const S &source : sources) {
        if (!ValidateSourceWeight(source)) {
            return Failure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        Params sourceParams;
        if (!DecomposeFrame<Point, Double>(source.frame, &sourceParams)) {
            return Failure(input);
        }
        targetScale += sourceParams.scale * source.normalizedWeight;
        if (params.blendShear) {
            targetShear += sourceParams.shear * source.normalizedWeight;
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : Failure(input);
    }
    targetScale = targetScale / totalWeight + params.offset;
    if (!IsFinite(targetScale)) {
        return Failure(input);
    }
    if (params.blendShear) {
        targetShear /= totalWeight;
        if (!IsFinite(targetShear)) {
            return Failure(input);
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (Affects(params.affect, axis)) {
            inputParams.scale[axis] = RigExecBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    if (params.blendShear) {
        BlendGovernedShear(params.affect, targetShear, globalWeight,
                           &inputParams);
    }
    return FrameFromParams<Point, Double>(input, inputParams);
}

/// Parent: each source's frame carried by its local offset, then the mean
/// translation, scale and Euler angles blended in per axis mask. A named
/// carry is handled as in Rotation.
template <class Point, class Double, class F, class S, class P>
F
RigExecApplyParentConstraintKernel(const F &input,
                                   const std::vector<S> &sources,
                                   const P &params)
{
    using namespace constraintDetail;
    using Math = RigExecDualQuatMath<Point, Double>;
    using Mat4 = typename Math::Mat4;
    using Params = RigExecTransformParamsKernelT<Point, Double>;
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        return Failure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    if (!IsValidOrder(params.rotationOrder)) {
        return Failure(input);
    }

    Mat4 carryInverse(1.0);
    F measured;
    const F *in = &input;
    if (params.carry) {
        carryInverse = params.carry->GetInverse();
        measured = TransformFrame(input, carryInverse);
        in = &measured;
    }
    Params inputParams;
    if (!DecomposeFrame<Point, Double>(*in, &inputParams)) {
        return Failure(input);
    }
    const Double inputEuler = EulerDegreesFromQuat<Point, Double>(
        inputParams.rotation, params.rotationOrder);
    if (!IsFinite(inputEuler)) {
        return Failure(input);
    }
    Double targetTranslation(0.0), targetScale(0.0), targetShear(0.0);
    Double sourceAnchor(0.0), weightedRotationDelta(0.0);
    bool hasSourceAnchor = false;
    double totalWeight = 0.0;
    for (const S &source : sources) {
        if (!ValidateSourceWeight(source)) {
            return Failure(input);
        }
        if (source.normalizedWeight == 0.0) {
            continue;
        }
        if (!IsFinite(source.translationOffset) ||
            !IsFinite(source.rotationOffsetDegrees)) {
            return Failure(input);
        }
        const F sourceMeasured =
            params.carry ? TransformFrame(source.frame, carryInverse)
                         : source.frame;
        if (!IsUsableFrame(sourceMeasured)) {
            return Failure(input);
        }
        Mat4 sourceMatrix(1.0);
        if (!Math::PointsToMatrix(IdentityLandmarks<Point, Double>(),
                                  sourceMeasured.points, &sourceMatrix)) {
            return Failure(input);
        }

        // The offset is the constrained object's local transform under each
        // source, so in row-vector convention it applies first.
        Params offsetParams;
        offsetParams.translation = source.translationOffset;
        offsetParams.rotation = QuatFromEulerDegrees<Point, Double>(
            source.rotationOffsetDegrees, params.rotationOrder);
        const Mat4 targetMatrix =
            RigExecParamsToMatrixKernel<Point, Double>(offsetParams) *
            sourceMatrix;
        const F targetFrame = MatrixToFrame<Point, Double, F>(targetMatrix);
        Params targetParams;
        if (!DecomposeFrame<Point, Double>(targetFrame, &targetParams)) {
            return Failure(input);
        }
        targetTranslation += targetParams.translation * source.normalizedWeight;
        targetScale += targetParams.scale * source.normalizedWeight;
        if (params.blendShear) {
            targetShear += targetParams.shear * source.normalizedWeight;
        }

        const Double sourceEuler = EulerDegreesFromQuat<Point, Double>(
            targetParams.rotation, params.rotationOrder);
        if (!IsFinite(sourceEuler)) {
            return Failure(input);
        }
        if (!hasSourceAnchor) {
            sourceAnchor = sourceEuler;
            hasSourceAnchor = true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            weightedRotationDelta[axis] +=
                source.normalizedWeight *
                ShortestDegrees(sourceEuler[axis] - sourceAnchor[axis]);
        }
        totalWeight += source.normalizedWeight;
    }
    if (totalWeight <= 0.0 || !std::isfinite(totalWeight)) {
        return totalWeight == 0.0 ? input : Failure(input);
    }
    targetTranslation /= totalWeight;
    targetScale /= totalWeight;
    if (params.blendShear) {
        targetShear /= totalWeight;
    }
    const Double targetEuler =
        sourceAnchor + weightedRotationDelta / totalWeight;
    if (!IsFinite(targetTranslation) || !IsFinite(targetScale) ||
        (params.blendShear && !IsFinite(targetShear)) ||
        !IsFinite(weightedRotationDelta) || !IsFinite(targetEuler)) {
        return Failure(input);
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (Affects(params.translationAxes, axis)) {
            inputParams.translation[axis] =
                RigExecBlendEnvelope(inputParams.translation[axis],
                                     targetTranslation[axis], globalWeight);
        }
        if (Affects(params.scaleAxes, axis)) {
            inputParams.scale[axis] = RigExecBlendEnvelope(
                inputParams.scale[axis], targetScale[axis], globalWeight);
        }
    }
    if (params.blendShear) {
        BlendGovernedShear(params.scaleAxes, targetShear, globalWeight,
                           &inputParams);
    }
    const Double outputEuler = ApplyEulerDelta(
        inputEuler, targetEuler, params.rotationAxes, globalWeight);
    inputParams.rotation =
        QuatFromEulerDegrees<Point, Double>(outputEuler, params.rotationOrder);
    const F solved = FrameFromParams<Point, Double>(input, inputParams);
    if (!params.carry || (solved.flags & kFrameDegenerate)) {
        return solved;
    }
    return TransformFrame(solved, *params.carry);
}

/// Aim: the local aim vector swung onto the target, with roll pinned by a
/// world up direction or the input's up (preserveInputUp), then the Euler
/// offset and the masked, weighted blend.
template <class Point, class Double, class F, class P>
F
RigExecApplyAimConstraintKernel(const F &input, const Double &targetPoint,
                                const P &params)
{
    using namespace constraintDetail;
    using Math = RigExecDualQuatMath<Point, Double>;
    using CMath = RigExecConstraintMath<Point, Double>;
    using Mat4 = typename Math::Mat4;
    using Quat = typename Math::Quat;
    using Rotation = typename CMath::Rotation;
    double globalWeight = 0.0;
    if (!ValidateGlobalWeight(params.weight, &globalWeight)) {
        return Failure(input);
    }
    if (globalWeight <= 0.0) {
        return input;
    }
    const Double *worldUp = UpDirection(params.worldUpDirection);
    const bool needsRollCorrection = worldUp || params.preserveInputUp;
    if (!IsValidOrder(params.rotationOrder) || !IsFinite(targetPoint) ||
        !IsFinite(params.localAimVector) ||
        (needsRollCorrection && !IsFinite(params.localUpVector)) ||
        !IsFinite(params.rotationOffsetDegrees) ||
        (worldUp && !IsFinite(*worldUp))) {
        return Failure(input);
    }

    RigExecTransformParamsKernelT<Point, Double> inputParams;
    if (!DecomposeFrame<Point, Double>(input, &inputParams)) {
        return Failure(input);
    }
    Double localAim = params.localAimVector;
    if (!NormalizeDirection(&localAim)) {
        return Failure(input);
    }
    Double targetAim = targetPoint - inputParams.translation;
    if (!NormalizeDirection(&targetAim)) {
        return Failure(input);
    }

    const Quat inputRotation = inputParams.rotation.GetNormalized();
    Double currentAim = inputRotation.Transform(localAim);
    if (!NormalizeDirection(&currentAim)) {
        return Failure(input);
    }
    const Rotation swing(currentAim, targetAim);
    Rotation twist(Double(1, 0, 0), 0.0);
    if (needsRollCorrection) {
        Double localUp = params.localUpVector;
        if (!NormalizeDirection(&localUp)) {
            return Failure(input);
        }
        localUp -= localAim * Math::Dot(localAim, localUp);
        if (!NormalizeDirection(&localUp)) {
            return Failure(input);
        }

        Double currentUp = inputRotation.Transform(localUp);
        if (!NormalizeDirection(&currentUp)) {
            return Failure(input);
        }
        Double swungUp = swing.TransformDir(currentUp);
        swungUp -= targetAim * Math::Dot(targetAim, swungUp);
        if (!NormalizeDirection(&swungUp)) {
            return Failure(input);
        }

        Double desiredUp;
        if (worldUp) {
            desiredUp = *worldUp;
            if (!NormalizeDirection(&desiredUp)) {
                return Failure(input);
            }
            desiredUp -= targetAim * Math::Dot(targetAim, desiredUp);
            if (!NormalizeDirection(&desiredUp)) {
                return Failure(input);
            }
        } else {
            // preserveInputUp keeps the previous world up rather than FBX
            // worldUpType=None's minimum swing.
            desiredUp = currentUp - targetAim * Math::Dot(targetAim, currentUp);
            if (!NormalizeDirection(&desiredUp)) {
                desiredUp = swungUp;
            }
        }
        twist = CMath::RotateOntoProjected(swungUp, desiredUp, targetAim);
    }

    Mat4 aimedMatrix(1.0);
    CMath::SetRotate(&aimedMatrix, inputRotation);
    Mat4 swingMatrix(1.0), twistMatrix(1.0);
    swingMatrix.SetRotate(swing);
    twistMatrix.SetRotate(twist);
    aimedMatrix = aimedMatrix * swingMatrix * twistMatrix;
    const Quat aimedRotation =
        aimedMatrix.ExtractRotation().GetQuat().GetNormalized();

    const Double inputEuler = EulerDegreesFromQuat<Point, Double>(
        inputRotation, params.rotationOrder);
    const Double targetEuler = EulerDegreesFromQuat<Point, Double>(
                                   aimedRotation, params.rotationOrder) +
                               params.rotationOffsetDegrees;
    if (!IsFinite(inputEuler) || !IsFinite(targetEuler)) {
        return Failure(input);
    }
    const Double outputEuler = ApplyEulerDelta(
        inputEuler, targetEuler, params.affectRotation, globalWeight);
    inputParams.rotation =
        QuatFromEulerDegrees<Point, Double>(outputEuler, params.rotationOrder);
    return FrameFromParams<Point, Double>(input, inputParams);
}

} // namespace rigExec

#endif
