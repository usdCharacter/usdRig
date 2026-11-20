// Method reference: Kavan et al. (2008), https://users.cs.utah.edu/~ladislav/dq/index.html
// The Gf instantiations of dualQuatKernel.h. Conventions, the scale and
// shear policy and the degenerate-case rules: dualQuat.h.
#include "dualQuat.h"

#include <cstddef>

namespace rigExec {

RigExecDualQuat
RigExecDualQuatFromRotationTranslation(const GfQuatd &rotation,
                                       const GfVec3d &translation)
{
    return RigExecDualQuatFromRotationTranslationKernel<GfVec3f, GfVec3d>(
        rotation, translation);
}

RigExecDualQuat
RigExecDualQuatFromMatrix(const GfMatrix4d &matrix, bool *isRigid)
{
    return RigExecDualQuatFromMatrixKernel<GfVec3f, GfVec3d>(matrix, isRigid);
}

RigExecScaledDualQuat
RigExecScaledDualQuatFromMatrix(const GfMatrix4d &matrix)
{
    return RigExecScaledDualQuatFromMatrixKernel<GfVec3f, GfVec3d>(matrix);
}

GfMatrix4d
RigExecDualQuatToMatrix(const RigExecDualQuat &dq)
{
    return RigExecDualQuatToMatrixKernel(dq);
}

GfMatrix4d
RigExecScaledDualQuatToMatrix(const RigExecScaledDualQuat &sdq)
{
    return RigExecScaledDualQuatToMatrixKernel(sdq);
}

GfVec3d
RigExecDualQuatTranslation(const RigExecDualQuat &dq)
{
    return RigExecDualQuatTranslationKernel(dq);
}

bool
RigExecDualQuatIsUnit(const RigExecDualQuat &dq, double tolerance)
{
    return RigExecDualQuatIsUnitKernel(dq, tolerance);
}

bool
RigExecDualQuatNormalize(RigExecDualQuat *dq)
{
    return RigExecDualQuatNormalizeKernel(dq);
}

bool
RigExecBlendDualQuats(const RigExecDualQuat *dqs, const double *weights,
                      size_t count, RigExecDualQuat *result)
{
    if (count > 0 && (!dqs || !weights)) {
        *result = RigExecDualQuat();
        return false;
    }
    return RigExecBlendDualQuatsKernel(
        count, [dqs](size_t i) { return &dqs[i]; },
        [weights](size_t i) { return weights[i]; }, result);
}

bool
RigExecBlendDualQuats(const RigExecDualQuat *palette, size_t paletteSize,
                      const int *jointIndices, const float *jointWeights,
                      size_t count, RigExecDualQuat *result)
{
    return RigExecBlendDualQuatsKernel(palette, paletteSize, jointIndices,
                                       jointWeights, count, result);
}

bool
RigExecBlendScaledDualQuats(const RigExecScaledDualQuat *sdqs,
                            const double *weights, size_t count,
                            RigExecScaledDualQuat *result)
{
    if (count > 0 && (!sdqs || !weights)) {
        *result = RigExecScaledDualQuat();
        return false;
    }
    return RigExecBlendScaledDualQuatsKernel(
        count, [sdqs](size_t i) { return &sdqs[i]; },
        [weights](size_t i) { return weights[i]; }, result);
}

bool
RigExecBlendScaledDualQuats(const RigExecScaledDualQuat *palette,
                            size_t paletteSize, const int *indices,
                            const double *weights, size_t count,
                            RigExecScaledDualQuat *result)
{
    return RigExecBlendScaledDualQuatsKernel(palette, paletteSize, indices,
                                             weights, count, result);
}

GfVec3d
RigExecDualQuatRotateVector(const RigExecDualQuat &dq, const GfVec3d &vector)
{
    return RigExecDualQuatRotateVectorKernel(dq, vector);
}

GfVec3d
RigExecDualQuatTransformPoint(const RigExecDualQuat &dq, const GfVec3d &point)
{
    return RigExecDualQuatTransformPointKernel(dq, point);
}

GfVec3d
RigExecScaledDualQuatTransformPoint(const RigExecScaledDualQuat &sdq,
                                    const GfVec3d &point)
{
    return RigExecScaledDualQuatTransformPointKernel(sdq, point);
}

} // namespace rigExec
