// RigExec solver kernels.
#include "solvers.h"
#include "chainSolverKernel.h"
#include "constraintKernel.h"
#include "frameBlendKernel.h"
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
    // ShortestArc is the one rotation blend.
    (void)rotationBlend;
    return RigExecBlendFramesKernel<GfVec3f, GfVec3d>(
        a, b, restPoints, weight, scaleBlend == RigExecScaleBlend::Log,
        outRestPoints);
}

std::vector<RigExecPointFrame>
RigExecDistributeTwist(const RigExecPointFrame &start,
                       const RigExecPointFrame &end,
                       const std::array<GfVec3d, 4> &startRest,
                       const std::array<GfVec3d, 4> &endRest,
                       const std::vector<double> &weights, double twistTurns)
{
    return RigExecDistributeTwistKernel<GfVec3f, GfVec3d>(
        start, end, startRest, endRest, weights, twistTurns);
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
    return RigExecValidateSkinLayoutKernel(*this, error);
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
    return RigExecSkinDualQuatPaletteKernel<GfVec3f, GfVec3d>(
        layout.transforms, layout.transformCount);
}

bool
RigExecApplyDualQuatSkin(const GfVec3d &point,
                         const RigExecScaledDualQuat *palette,
                         size_t paletteSize, const RigExecSkinLayout &layout,
                         size_t i, GfVec3d *out)
{
    return RigExecApplyDualQuatSkinPointKernel<GfVec3f, GfVec3d>(
        point, layout, i, palette, paletteSize, out);
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
