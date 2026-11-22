// RigExec CPU SIMD kernels: Gf entry points over matrixKernel.h and
// skinKernel.h, which hold the SSE2 forms and their scalar fallbacks.
#include "simdKernels.h"

#include "matrixKernel.h"
#include "skinKernel.h"
#include "solvers.h"

namespace rigExec {

void
RigExecNarrowSkinRows(const GfMatrix4d &transform, float *rows)
{
    RigExecNarrowSkinRowsKernel(transform, rows);
}

void
RigExecApplyWeightedMatrixSimd(const GfVec3f *in, GfVec3f *out,
                               const float *weights, size_t count,
                               const GfMatrix4d &transform)
{
    RigExecApplyWeightedMatrixSimdKernel<GfVec3f, GfVec3d>(in, out, weights,
                                                           count, transform);
}

void
RigExecApplyWeightedMatrixSimd(const GfVec3f *in, GfVec3f *out, float weight,
                               size_t count, const GfMatrix4d &transform)
{
    RigExecApplyWeightedMatrixSimdKernel<GfVec3f, GfVec3d>(in, out, weight,
                                                           count, transform);
}

void
RigExecApplyLinearBlendSkinSimd(const GfVec3f *in, GfVec3f *out,
                                const RigExecSkinLayout &layout,
                                const float *rows)
{
    RigExecApplyLinearBlendSkinSimdKernel<GfVec3f, GfVec3d>(in, out, layout,
                                                            rows);
}

void
RigExecApplyLinearBlendSkinSimd(const GfVec3f *in, GfVec3f *out,
                                const RigExecSkinLayout &layout)
{
    RigExecApplyLinearBlendSkinSimd(in, out, layout, /*rows=*/nullptr);
}

} // namespace rigExec
