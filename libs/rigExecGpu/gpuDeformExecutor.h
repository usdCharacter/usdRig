// Runs per-vertex deform programs (rigExecMath/gpuDeformProgram.h) on the
// GPU as an OpenGL 4.5 compute shader and reads the points back.
//
// The device lives on one worker thread that owns its own hidden OpenGL
// context, so the executor never touches a viewer's context and a caller on
// any thread hands it work at a publication boundary. Static layouts are
// uploaded once per chain and layout; a run uploads each chain's frame (a
// few kilobytes), dispatches one thread per point into persistently mapped
// client memory, waits on a fence and copies the points out.
#ifndef RIGEXEC_GPU_DEFORM_EXECUTOR_H
#define RIGEXEC_GPU_DEFORM_EXECUTOR_H

#include "rigExec/gpuCloth.h"
#include "rigExecMath/gpuDeformProgram.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

class RigExecGpuDeformExecutor {
public:
    /// The process's executor, started on first use; null when no GPU
    /// device can be created (no platform context, no Hgi backend).
    static RigExecGpuDeformExecutor *Get();

    /// Frees the buffers a RigExecGpuDeviceKey named, on the device's
    /// thread, without waiting. Does nothing when no device started.
    static void ReleaseKey(uint64_t key);

    /// Runs every job and waits for the points. Returns false when the
    /// device failed; each job's `ok` says how that chain fared.
    bool Run(std::vector<RigExecGpuDeformJob> *jobs);

    /// Solves every cloth window (rigExec/gpuCloth.h) and waits for the
    /// points. Returns false when the device failed; each job's `ok` says
    /// how that cloth fared.
    bool RunCloth(std::vector<RigExecGpuClothJob> *jobs);

    /// How many cloth windows the device has solved, for tests.
    size_t ClothSolveCount() const;

    /// Milliseconds the last Run took from submit to points on the CPU.
    double LastRunMs() const;

    /// The device's description (vendor, renderer, version).
    std::string Describe() const;

    ~RigExecGpuDeformExecutor();

    struct Impl;

private:
    RigExecGpuDeformExecutor();
    std::unique_ptr<Impl> _impl;
};

/// Hands the evaluator this executor as its GPU deform runner
/// (RigExecSetGpuDeformRunner). Idempotent; a host calls it once at load.
void RigExecGpuRegisterDeformRunner();

/// Hands the evaluator this executor as its cloth solver
/// (RigExecSetGpuClothRunner), so the live program solves cloth on the GPU
/// unless RIGEXEC_GPU_CLOTH=0. Idempotent; a host calls it once at load.
void RigExecGpuRegisterClothRunner();

}  // namespace rigExec

#endif  // RIGEXEC_GPU_DEFORM_EXECUTOR_H
