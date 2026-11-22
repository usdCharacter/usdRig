// RigExec solver kernels.
#include "solvers.h"
#include "chainSolverKernel.h"
#include "constraintKernel.h"
#include "dualQuat.h"
#include "matrixKernel.h"
#include "skinKernel.h"
#include "envelope.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/rotation.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace rigExec {

std::vector<RigExecPointFrame>
RigExecSolveFkChain(const std::vector<RigExecFkChainElement> &elements)
{
    return RigExecSolveFkChainKernel<GfVec3f, GfVec3d, RigExecPointFrame>(
        elements);
}

void
RigExecScaleFkSegments(const std::vector<RigExecFkChainElement> &elements,
                       std::vector<RigExecPointFrame> *frames)
{
    if (frames && frames->size() == elements.size())
        RigExecScaleFkSegmentsKernel<GfVec3d>(elements, frames);
}

namespace {

// std::acos(-1) rather than M_PI, which MSVC and strict modes lack.
const double kPi = std::acos(-1);

} // namespace

void
RigExecTwoBoneIkLengths(const std::array<std::array<GfVec3d, 4>, 3> &restPoints,
                        const GfMatrix4d &space, double upperOffset,
                        double lowerOffset, double *upperLength,
                        double *lowerLength)
{
    RigExecTwoBoneIkLengthsKernel(restPoints, space, upperOffset, lowerOffset,
                                  upperLength, lowerLength);
}

double
RigExecTwoBoneIkSpaceFactor(
    const std::array<std::array<GfVec3d, 4>, 3> &restPoints,
    const GfMatrix4d &space)
{
    return RigExecTwoBoneIkSpaceFactorKernel(restPoints, space);
}

void
RigExecSetTwoBoneLimbParams(
    const std::array<std::array<GfVec3d, 4>, 3> &restPoints,
    const GfMatrix4d &space, double stretch, double pin, double upperScale,
    double lowerScale, double softDistance, double twistDegrees,
    RigExecTwoBoneIkParams *params)
{
    if (!params) {
        return;
    }
    params->limb.stretch = stretch;
    params->limb.pin = pin;
    params->limb.upperScale = upperScale;
    params->limb.lowerScale = lowerScale;
    params->limb.softDistance =
        softDistance * RigExecTwoBoneIkSpaceFactor(restPoints, space);
    params->twistRadians = twistDegrees * kPi / 180.0;
}

std::array<RigExecPointFrame, 3>
RigExecSolveTwoBoneIk(const RigExecPointFrame &rootFrame,
                      const RigExecPointFrame &effectorFrame,
                      const RigExecPointFrame &poleFrame,
                      const std::array<std::array<GfVec3d, 4>, 3> &restPoints,
                      const RigExecTwoBoneIkParams &params)
{
    return RigExecSolveTwoBoneIkKernel<GfVec3f, GfVec3d>(
        rootFrame, effectorFrame, poleFrame, restPoints, params);
}

RigExecPointFrame
RigExecBlendFrames(const RigExecPointFrame &a, const RigExecPointFrame &b,
                   const std::array<GfVec3d, 4> &restPoints, double weight,
                   RigExecRotationBlend rotationBlend,
                   RigExecScaleBlend scaleBlend,
                   const std::array<GfVec3d, 4> *outRestPoints)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (!outRestPoints) {
        // At the ends the blend IS the input frame,
        // bit for bit, with no SRT round trip.
        if (w <= 0.0)
            return a;
        if (w >= 1.0)
            return b;
    } else if (w <= 0.0 || w >= 1.0) {
        // Re-based, the end still has to be re-applied to the new basis, so
        // it goes through the map rather than through the frame.
        const RigExecPointFrame &pick = (w <= 0.0) ? a : b;
        GfMatrix4d end;
        if (!RigExecPointsToMatrix(restPoints, pick.points, &end)) {
            RigExecPointFrame held = pick;
            held.flags |= RigExecPointFrameDegenerate;
            return held;
        }
        return RigExecMatrixToPoints(*outRestPoints, end);
    }

    RigExecTransformParams pa, pb;
    if (!RigExecPointsToParams(restPoints, a.points, RigExecAxis::Z, &pa) ||
        !RigExecPointsToParams(restPoints, b.points, RigExecAxis::Z, &pb)) {
        // A singular input frame cannot blend through SRT; hold the
        // stronger input and flag it (no silent identity).
        RigExecPointFrame held = (w < 0.5) ? a : b;
        held.flags |= RigExecPointFrameDegenerate;
        return held;
    }

    RigExecTransformParams pr;
    pr.translation = pa.translation * (1 - w) + pb.translation * w;

    GfQuatd qa = pa.rotation, qb = pb.rotation;
    if (rotationBlend == RigExecRotationBlend::ShortestArc &&
        GfDot(qa.GetImaginary(), qb.GetImaginary()) +
                qa.GetReal() * qb.GetReal() <
            0) {
        qb = -qb;
    }
    pr.rotation = GfSlerp(w, qa, qb).GetNormalized();

    for (int i = 0; i < 3; ++i) {
        if (scaleBlend == RigExecScaleBlend::Log && pa.scale[i] > 0 &&
            pb.scale[i] > 0) {
            pr.scale[i] = std::exp(std::log(pa.scale[i]) * (1 - w) +
                                   std::log(pb.scale[i]) * w);
        } else {
            pr.scale[i] = pa.scale[i] * (1 - w) + pb.scale[i] * w;
        }
        pr.shear[i] = pa.shear[i] * (1 - w) + pb.shear[i] * w;
    }

    const GfMatrix4d m = RigExecParamsToMatrix(pr);
    return RigExecMatrixToPoints(outRestPoints ? *outRestPoints : restPoints,
                                 m);
}

namespace {

// Swing/twist decomposition of q about unit axis: q = swing * twist,
// twist a rotation about the axis (Dobrowolski 2015, see docs/references.md).
void
_SwingTwist(const GfQuatd &q, const GfVec3d &axis, GfQuatd *swing,
            GfQuatd *twist)
{
    const GfVec3d im = q.GetImaginary();
    const double proj = GfDot(im, axis);
    GfQuatd t(q.GetReal(), axis * proj);
    const double len = std::sqrt(t.GetReal() * t.GetReal() +
                                 GfDot(t.GetImaginary(), t.GetImaginary()));
    if (len < 1e-15) {
        *twist = GfQuatd::GetIdentity();
    } else {
        *twist = t * (1.0 / len);
    }
    *swing = q * twist->GetInverse();
}

} // namespace

std::vector<RigExecPointFrame>
RigExecDistributeTwist(const RigExecPointFrame &start,
                       const RigExecPointFrame &end,
                       const std::array<GfVec3d, 4> &startRest,
                       const std::array<GfVec3d, 4> &endRest,
                       const std::vector<double> &weights, double twistTurns)
{
    std::vector<RigExecPointFrame> result;
    if (!std::isfinite(twistTurns))
        return result;
    result.reserve(weights.size());

    RigExecTransformParams ps, pe;
    const bool okS =
        RigExecPointsToParams(startRest, start.points, RigExecAxis::Z, &ps);
    const bool okE =
        RigExecPointsToParams(endRest, end.points, RigExecAxis::Z, &pe);
    if (!okS || !okE) {
        for (double w : weights) {
            RigExecPointFrame f = (w < 0.5) ? start : end;
            f.flags |= RigExecPointFrameDegenerate;
            result.push_back(f);
        }
        return result;
    }

    // Relative rotation decomposed into swing and unwrapped aim-axis twist.
    const GfVec3d aim = (start.X() - start.Origin()).GetNormalized();
    GfQuatd qs = ps.rotation, qe = pe.rotation;
    if (GfDot(qs.GetImaginary(), qe.GetImaginary()) +
            qs.GetReal() * qe.GetReal() <
        0) {
        qe = -qe;
    }
    const GfQuatd rel = qs.GetInverse() * qe;
    // Axis in start-local space (column convention: rotate world aim by
    // qs^-1).
    const GfQuatd qsInv = qs.GetInverse();
    const GfVec3d localAim = qsInv.Transform(aim);
    GfQuatd swing, twist;
    _SwingTwist(rel, localAim, &swing, &twist);

    // Unwrapped twist angle about localAim.
    double twistAngle = 2.0 * std::atan2(GfDot(twist.GetImaginary(), localAim),
                                         twist.GetReal());
    twistAngle += 2.0 * std::acos(-1.0) * twistTurns;
    if (!std::isfinite(twistAngle))
        return result;

    for (size_t k = 0; k < weights.size(); ++k) {
        const double w = std::min(std::max(weights[k], 0.0), 1.0);

        const GfQuatd swingK =
            GfSlerp(w, GfQuatd::GetIdentity(), swing).GetNormalized();
        const double angK = twistAngle * w;
        const GfQuatd twistK(std::cos(angK / 2), localAim * std::sin(angK / 2));
        const GfQuatd qk = (qs * swingK * twistK).GetNormalized();

        RigExecTransformParams pk;
        pk.translation = ps.translation * (1 - w) + pe.translation * w;
        pk.rotation = qk;
        pk.scale = ps.scale * (1 - w) + pe.scale * w;
        pk.shear = ps.shear * (1 - w) + pe.shear * w;

        // Sample origin interpolates between the frame origins; the
        // decomposed translations already carry that motion for identical
        // rest sets, but samples use start's rest landmarks.
        const GfMatrix4d m = RigExecParamsToMatrix(pk);
        RigExecPointFrame f = RigExecMatrixToPoints(startRest, m);
        const GfVec3d origin = start.Origin() * (1 - w) + end.Origin() * w;
        const GfVec3d shift = origin - f.Origin();
        for (auto &p : f.points) {
            p += shift;
        }
        result.push_back(f);
    }
    return result;
}

GfVec3d
RigExecApplyWeightedMatrix(const GfVec3d &point, const GfMatrix4d &transform,
                           double weight)
{
    return RigExecWeightedMatrixPoint(point, transform, weight);
}

bool
RigExecSkinLayout::Validate(std::string *error) const
{
    auto fail = [error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (elementSize < 1) {
        return fail("elementSize must be at least 1");
    }
    if (!transforms || transformCount == 0) {
        return fail("no influences");
    }
    if (indexCount != pointCount * elementSize) {
        return fail("jointIndices/jointWeights length " +
                    std::to_string(indexCount) + " must equal " +
                    std::to_string(pointCount) + " points * elementSize " +
                    std::to_string(elementSize));
    }
    if (indexCount && (!indices || !weights)) {
        return fail("jointIndices and jointWeights are both required");
    }
    for (size_t t = 0; t < transformCount; ++t) {
        const GfMatrix4d &m = transforms[t];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (!std::isfinite(m[r][c])) {
                    return fail("influence " + std::to_string(t) +
                                " has a non-finite matrix");
                }
            }
        }
        if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1) {
            return fail("influence " + std::to_string(t) +
                        " has a non-affine matrix");
        }
    }
    for (size_t i = 0; i < indexCount; ++i) {
        if (indices[i] < 0 || size_t(indices[i]) >= transformCount) {
            return fail("jointIndices[" + std::to_string(i) + "] = " +
                        std::to_string(indices[i]) + " is outside the " +
                        std::to_string(transformCount) + " influences");
        }
        if (!std::isfinite(weights[i]) || weights[i] < 0.0f) {
            return fail("jointWeights[" + std::to_string(i) +
                        "] must be finite and non-negative");
        }
    }
    return true;
}

GfVec3d
RigExecApplyLinearBlendSkin(const GfVec3d &point,
                            const RigExecSkinLayout &layout, size_t i)
{
    return RigExecLinearBlendSkinPointKernel(point, layout, i);
}

void
RigExecApplyLinearBlendSkin(const GfVec3f *in, GfVec3f *out,
                            const RigExecSkinLayout &layout)
{
    RigExecApplyLinearBlendSkinKernel<GfVec3f, GfVec3d>(in, out, layout);
}

std::vector<RigExecScaledDualQuat>
RigExecSkinDualQuatPalette(const RigExecSkinLayout &layout)
{
    return RigExecSkinDualQuatPaletteKernel<GfVec3f, GfVec3d>(layout);
}

bool
RigExecApplyDualQuatSkin(const GfVec3d &point,
                         const RigExecScaledDualQuat *palette,
                         size_t paletteSize, const RigExecSkinLayout &layout,
                         size_t i, GfVec3d *out)
{
    std::vector<int> indices;
    std::vector<double> weights;
    RigExecGatherDualQuatInfluences(layout, i, &indices, &weights);
    RigExecScaledDualQuat blend;
    if (!RigExecBlendScaledDualQuats(palette, paletteSize, indices.data(),
                                     weights.data(), indices.size(), &blend)) {
        return false;
    }
    *out = RigExecScaledDualQuatTransformPoint(blend, point);
    return true;
}

bool
RigExecApplyDualQuatSkin(const GfVec3f *in, GfVec3f *out,
                         const RigExecSkinLayout &layout,
                         const RigExecScaledDualQuat *palette,
                         size_t paletteSize)
{
    return RigExecApplyDualQuatSkinKernel<GfVec3f, GfVec3d>(
        in, out, layout, palette, paletteSize);
}

bool
RigExecApplyDualQuatSkin(const GfVec3f *in, GfVec3f *out,
                         const RigExecSkinLayout &layout)
{
    // The split is a pure function of the influence table, so building it
    // here and building it in a caller that skins several ranges against the
    // same table produce the same palette entry for entry.
    const std::vector<RigExecScaledDualQuat> palette =
        RigExecSkinDualQuatPalette(layout);
    return RigExecApplyDualQuatSkin(in, out, layout, palette.data(),
                                    palette.size());
}

// The constraints run the shared kernel (constraintKernel.h).

RigExecPointFrame
RigExecApplyPositionConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecPositionConstraintParams &params)
{
    return RigExecApplyPositionConstraintKernel<GfVec3f, GfVec3d>(
        input, sources, params);
}

RigExecPointFrame
RigExecApplyRotationConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecRotationConstraintParams &params)
{
    return RigExecApplyRotationConstraintKernel<GfVec3f, GfVec3d>(
        input, sources, params);
}

RigExecPointFrame
RigExecApplyScaleConstraint(const RigExecPointFrame &input,
                            const std::vector<RigExecConstraintSource> &sources,
                            const RigExecScaleConstraintParams &params)
{
    return RigExecApplyScaleConstraintKernel<GfVec3f, GfVec3d>(input, sources,
                                                               params);
}

RigExecPointFrame
RigExecApplyParentConstraint(
    const RigExecPointFrame &input,
    const std::vector<RigExecConstraintSource> &sources,
    const RigExecParentConstraintParams &params)
{
    return RigExecApplyParentConstraintKernel<GfVec3f, GfVec3d>(input, sources,
                                                                params);
}

RigExecPointFrame
RigExecApplyAimConstraint(const RigExecPointFrame &input,
                          const GfVec3d &targetPoint,
                          const RigExecAimConstraintParams &params)
{
    return RigExecApplyAimConstraintKernel<GfVec3f, GfVec3d>(input, targetPoint,
                                                             params);
}

RigExecPointFrame
RigExecApplyAimConstraint(const RigExecPointFrame &input,
                          const GfVec3d &targetOrigin, double weight,
                          int aimLandmarkIndex)
{
    if (!std::isfinite(weight)) {
        RigExecPointFrame flagged = input;
        flagged.flags |= RigExecPointFrameDegenerate;
        return flagged;
    }
    weight = std::min(std::max(weight, 0.0), 1.0);

    const int aimIdx = std::min(std::max(aimLandmarkIndex, 1), 3);
    // Even-permutation role assignment keeps side = aim x up for a
    // right-handed input.
    const int upIdx = (aimIdx % 3) + 1;
    const int sideIdx = (upIdx % 3) + 1;

    const GfVec3d origin = input.Origin();
    const GfVec3d currentAim = input.points[aimIdx] - origin;
    const double aimLen = currentAim.GetLength();
    const GfVec3d toTarget = targetOrigin - origin;
    if (aimLen < 1e-12 || toTarget.GetLength() < 1e-12 || weight <= 0.0) {
        return input;
    }
    const GfVec3d a0 = currentAim / aimLen;
    const GfVec3d a1 = toTarget.GetNormalized();

    // Input handedness is preserved through the rebuild so a reflected
    // frame stays reflected.
    const GfVec3d inUp = input.points[upIdx] - origin;
    const GfVec3d inSide = input.points[sideIdx] - origin;
    const double handedness =
        GfDot(GfCross(currentAim, inUp), inSide) < 0 ? -1.0 : 1.0;

    const GfRotation full(a0, a1);
    const GfRotation partial(full.GetAxis(), full.GetAngle() * weight);
    const GfVec3d newAim =
        weight >= 1.0 ? a1 : partial.TransformDir(a0).GetNormalized();

    const double upLen = inUp.GetLength();
    const double sideLen = inSide.GetLength();
    GfVec3d up = inUp;
    up -= newAim * GfDot(newAim, up);
    if (up.GetLength() < 1e-12) {
        up = weight >= 1.0 ? full.TransformDir(inUp)
                           : partial.TransformDir(inUp);
        up -= newAim * GfDot(newAim, up);
    }
    if (up.GetLength() < 1e-12) {
        RigExecPointFrame flagged = input;
        flagged.flags |= RigExecPointFrameDegenerate;
        return flagged;
    }
    up.Normalize();
    const GfVec3d side = handedness * GfCross(newAim, up).GetNormalized();

    RigExecPointFrame out = input;
    out.points[aimIdx] = origin + newAim * aimLen;
    out.points[upIdx] = origin + up * upLen;
    out.points[sideIdx] = origin + side * sideLen;
    return out;
}

} // namespace rigExec
