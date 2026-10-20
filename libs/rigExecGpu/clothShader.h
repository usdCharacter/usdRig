// The GLSL compute kernel of RigExecClothMover's device solve
// (rigExec/gpuCloth.h): the passes of rigExecMath/clothKernel.h, one
// dispatch per pass over the whole cloth, issued in the CPU solve's order.
#ifndef RIGEXEC_GPU_CLOTH_SHADER_H
#define RIGEXEC_GPU_CLOTH_SHADER_H

namespace rigExec {

/// Threads per workgroup the kernel is written for.
constexpr int RigExecGpuClothLocalSize = 64;

/// The complete compute shader source, #version line included.
const char *RigExecGpuClothKernelSource();

}  // namespace rigExec

#endif  // RIGEXEC_GPU_CLOTH_SHADER_H
