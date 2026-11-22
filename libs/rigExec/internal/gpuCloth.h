// The device half of RigExecClothMover's solve: what a GPU library runs in
// place of RigExecSolveCloth for the live program, at the publication
// boundary, and how it is registered. rigExec links no GPU library; with no
// runner, or with RIGEXEC_GPU_CLOTH=0, every cloth solves on the CPU kernel.
//
// A device solve runs the same passes as rigExecMath/clothKernel.h over the
// same window, so it agrees with the CPU to float rounding, not bit for bit.
// Frozen clones (background frames, history ticks) and the .rigexec runtime
// always solve on the CPU.
#ifndef RIGEXEC_GPU_CLOTH_H
#define RIGEXEC_GPU_CLOTH_H

#include "rigExecMath/clothKernel.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rigExec {

/// One cloth's window for a device run.
struct RigExecGpuClothJob {
    /// The caller's stable name for the cloth, which keys its buffers.
    uint64_t key = 0;
    /// Moves whenever the mesh or the colliders do (it keys their upload).
    uint64_t compiledRigSerial = 0;
    const RigExecClothMesh *mesh = nullptr;
    const RigExecClothColliders *colliders = nullptr;
    RigExecClothSettings settings;
    double frameSeconds = 1.0 / 24.0;
    /// The window, oldest first; at least two ticks, each usable
    /// (RigExecClothWindowUsable).
    const RigExecClothTick *ticks = nullptr;
    size_t tickCount = 0;
    /// mesh->pointCount xyz triples, written by the run.
    float *out = nullptr;
    /// Set by the run: the window was solved and every point is finite.
    bool ok = false;
};

/// Runs device jobs synchronously. False when the device failed; each job's
/// `ok` says how its cloth fared.
using RigExecGpuClothRunner = bool (*)(std::vector<RigExecGpuClothJob> *jobs);

/// The runner a GPU library registers (rigExecGpu's
/// RigExecGpuRegisterClothRunner).
void RigExecSetGpuClothRunner(RigExecGpuClothRunner runner);
RigExecGpuClothRunner RigExecGetGpuClothRunner();

/// Whether a program built now sends its cloth solves to the device: a
/// runner is registered and RIGEXEC_GPU_CLOTH is not 0/off/false. Read on
/// the owning thread when a program is built.
bool RigExecGpuClothEnabled();

/// The fewest cloth points a device solve is worth its launch cost for:
/// RIGEXEC_GPU_CLOTH_MIN_POINTS, 2048 by default. A smaller cloth solves on
/// the CPU kernel at the same boundary. Read when a program is built.
size_t RigExecGpuClothMinPoints();

}  // namespace rigExec

#endif  // RIGEXEC_GPU_CLOTH_H
