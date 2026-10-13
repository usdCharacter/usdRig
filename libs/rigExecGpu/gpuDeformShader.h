// The GLSL body of the per-vertex deform program, shared by the compute
// executor (gpuDeformExecutor.h) and the Storm ExtComputation kernel.
#ifndef RIGEXEC_GPU_DEFORM_SHADER_H
#define RIGEXEC_GPU_DEFORM_SHADER_H

namespace rigExec {

/// Functions only: `vec3 rxDeform(int i, vec3 p)` runs every op of the
/// program over point i entering at p, and sets the global `rxFailed` where
/// a skin blend is degenerate. The host defines RX_OP(i), RX_SF(i),
/// RX_SI(i), RX_FR(i) and RX_OP_COUNT before it (see the source).
const char *RigExecGpuDeformKernelBody();

/// The Storm ExtComputation kernel: `compute(int index)` over the inputs
/// rxOps, rxStaticF, rxStaticI, rxBase and rxOpCount (an aggregator's) and
/// rxFrame, rxPrevFrame and rxVelocityScale (per frame), writing the
/// outputs `points` and `velocities` -- the latter the difference of the
/// two frames' points times the scale.
const char *RigExecGpuDeformStormKernel();

}  // namespace rigExec

#endif  // RIGEXEC_GPU_DEFORM_SHADER_H
