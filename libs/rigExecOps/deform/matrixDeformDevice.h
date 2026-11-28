#ifndef RIGEXEC_OPS_DEFORM_MATRIX_DEFORM_DEVICE_H
#define RIGEXEC_OPS_DEFORM_MATRIX_DEFORM_DEVICE_H

/// MatrixDeform's weighted-matrix rule (rigExecMath/matrixKernel.h) as a
/// device kernel over point columns: one invocation per point, x, y and z
/// each in its own float column, the weights in a fourth. The host code
/// runs the shared kernel itself, so the reference device answers exactly
/// what the CPU op does.

#include "rigExec/api/gpuDevice.h"
#include "rigExecMath/matrixKernel.h"
#include "rigExecMath/runtimeMath.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace rigExecOps {

enum MatrixDeformDeviceBinding : uint32_t {
    kMatrixDeviceInX,
    kMatrixDeviceInY,
    kMatrixDeviceInZ,
    /// Empty when the uniform weight applies to every point.
    kMatrixDeviceWeights,
    kMatrixDeviceOutX,
    kMatrixDeviceOutY,
    kMatrixDeviceOutZ,
    kMatrixDeviceBindingCount
};

inline constexpr rigExec::RigExecGpuAccess
    kMatrixDeviceAccess[kMatrixDeviceBindingCount] = {
        rigExec::RigExecGpuAccess::Read,  rigExec::RigExecGpuAccess::Read,
        rigExec::RigExecGpuAccess::Read,  rigExec::RigExecGpuAccess::Read,
        rigExec::RigExecGpuAccess::Write, rigExec::RigExecGpuAccess::Write,
        rigExec::RigExecGpuAccess::Write};

/// The transform's affine rows 0..3, columns 0..2, row-vector.
struct MatrixDeformDeviceUniforms {
    double affine[12];
    float weight;
    uint32_t weighted;
};

inline MatrixDeformDeviceUniforms
MatrixDeformDeviceUniformsOf(const rigExec::RrMat4d &transform, float weight,
                             bool weighted)
{
    MatrixDeformDeviceUniforms u{};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 3; ++c)
            u.affine[r * 3 + c] = transform[r][c];
    u.weight = weight;
    u.weighted = weighted ? 1u : 0u;
    return u;
}

inline void
MatrixDeformDeviceHost(const rigExec::RigExecGpuHostInvocation &invocation)
{
    using rigExec::RrMat4d;
    using rigExec::RrVec3d;
    using rigExec::RrVec3f;
    MatrixDeformDeviceUniforms u;
    std::memcpy(&u, invocation.uniforms, sizeof u);
    const double *a = u.affine;
    const RrMat4d transform(a[0], a[1], a[2], 0.0, a[3], a[4], a[5], 0.0, a[6],
                            a[7], a[8], 0.0, a[9], a[10], a[11], 1.0);
    const rigExec::RigExecGpuHostRange *b = invocation.bindings;
    constexpr uint32_t kChunk = 64;
    float x[kChunk], y[kChunk], z[kChunk], w[kChunk];
    RrVec3f points[kChunk];
    for (uint32_t begin = invocation.begin; begin < invocation.end;
         begin += kChunk) {
        const uint32_t n = std::min(kChunk, invocation.end - begin);
        const size_t at = size_t(begin) * sizeof(float);
        const size_t bytes = size_t(n) * sizeof(float);
        std::memcpy(x, b[kMatrixDeviceInX].data + at, bytes);
        std::memcpy(y, b[kMatrixDeviceInY].data + at, bytes);
        std::memcpy(z, b[kMatrixDeviceInZ].data + at, bytes);
        for (uint32_t i = 0; i < n; ++i)
            points[i] = RrVec3f(x[i], y[i], z[i]);
        if (u.weighted) {
            std::memcpy(w, b[kMatrixDeviceWeights].data + at, bytes);
            rigExec::RigExecApplyWeightedMatrixSimdKernel<RrVec3f, RrVec3d>(
                points, points, w, n, transform);
        } else {
            rigExec::RigExecApplyWeightedMatrixSimdKernel<RrVec3f, RrVec3d>(
                points, points, u.weight, n, transform);
        }
        for (uint32_t i = 0; i < n; ++i) {
            x[i] = points[i][0];
            y[i] = points[i][1];
            z[i] = points[i][2];
        }
        std::memcpy(b[kMatrixDeviceOutX].data + at, x, bytes);
        std::memcpy(b[kMatrixDeviceOutY].data + at, y, bytes);
        std::memcpy(b[kMatrixDeviceOutZ].data + at, z, bytes);
    }
}

inline rigExec::RigExecGpuKernelDesc
MatrixDeformDeviceKernel()
{
    rigExec::RigExecGpuKernelDesc desc;
    desc.access = kMatrixDeviceAccess;
    desc.bindingCount = kMatrixDeviceBindingCount;
    desc.uniformBytes = sizeof(MatrixDeformDeviceUniforms);
    desc.host = &MatrixDeformDeviceHost;
    desc.debugName = "MatrixDeform";
    return desc;
}

} // namespace rigExecOps

#endif
