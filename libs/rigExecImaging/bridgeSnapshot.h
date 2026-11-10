// Snapshot payload construction owned by the imaging bridge: a pure
// builder over per-call state; publication and numbering stay on the facade.
#ifndef RIGEXEC_IMAGING_BRIDGE_SNAPSHOT_H
#define RIGEXEC_IMAGING_BRIDGE_SNAPSHOT_H

#include "snapshotStore.h"

#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

PXR_NAMESPACE_USING_DIRECTIVE

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace rigExec {

struct RigExecRigPose;

/// The bridge-owned state a snapshot build reads, built at the call site
/// and borrowed for the call only; the component keeps no copy, so a
/// recompile or stage edit takes effect on the next call.
/// \p scalarPrimvarTargets and \p scalarPrimvarNames are compiled scalar
/// chains paired by index; required borrows, non-null by construction.
/// \p projectorTargets' matrices stay target-local.
struct RigExecSnapshotCall {
    RigExecSnapshotCall(const UsdStageRefPtr &stage, UsdTimeCode time,
                        uint64_t stageEditSerial,
                        std::vector<SdfPath> projectorTargets,
                        std::vector<SdfPath> animatedMapPrimvars,
                        const std::vector<SdfPath> &scalarPrimvarTargets,
                        const std::vector<TfToken> &scalarPrimvarNames)
        : stage(stage), time(time), stageEditSerial(stageEditSerial),
          projectorTargets(std::move(projectorTargets)),
          animatedMapPrimvars(std::move(animatedMapPrimvars)),
          scalarPrimvarTargets(scalarPrimvarTargets),
          scalarPrimvarNames(scalarPrimvarNames)
    {
    }

    UsdStageRefPtr stage;
    UsdTimeCode time;
    /// The evaluator's stage-edit serial: the authored reset walk reruns
    /// only when this or the driven provider set moves.
    uint64_t stageEditSerial = 0;
    std::vector<SdfPath> projectorTargets;
    std::vector<SdfPath> animatedMapPrimvars;
    const std::vector<SdfPath> &scalarPrimvarTargets;
    const std::vector<TfToken> &scalarPrimvarNames;
};

/// The render-motion path's provider-transform publish reads only the
/// stage, the time and the edit serial: no compiled publication lists.
struct RigExecProviderXformCall {
    UsdStageRefPtr stage;
    UsdTimeCode time;
    uint64_t stageEditSerial = 0;
};

/// Which generation a build is for. Displayed: the complete viewport
/// generation, including the GPU deform publication and frame history.
/// Query: no GPU display history is touched.
enum class RigExecSnapshotBuildMode { Displayed, Query };

class RigExecBridgeSnapshotBuilder {
public:
    /// Fills \p snapshot's payload from the exact native results: points
    /// and normals as flat primvars, extent as min/max, matrix and float
    /// primvars from the compiled lists, provider base+revision transforms
    /// and authored reset boundaries. A non-geometry property result never
    /// indexes snapshot->prims: it records into movedFloats/movedArrays
    /// instead of minting an empty published prim.
    void _Build(const RigExecSnapshotCall &call, const RigExecRigPose &pose,
                RigExecSnapshotBuildMode mode,
                RigExecImagingSnapshot *snapshot);

    /// Publishes constraint-driven transforms onto the prims themselves
    /// (base and revision travel together); the render-motion path calls
    /// this directly at the base offset.
    void _FillProviderXforms(const RigExecProviderXformCall &call,
                             const RigExecRigPose &pose,
                             RigExecImagingSnapshot *snapshot);

private:
    /// Fills \p published with a chain's program publication (shader
    /// mode): the shared tables, this frame and the adjacent one.
    void _PublishGpuDeform(const RigExecSnapshotCall &call,
                           const SdfPath &primPath,
                           const RigExecGpuDeformPublication &publication,
                           RigExecPublishedPrim *published);

    struct _GpuDeformLast {
        double time = 0.0;
        bool numeric = false;
        uint64_t serial = 0;
        uint64_t frameVersion = 0;
        VtFloatArray frame;
        VtFloatArray previous;
    };

    // Authored resetXformStack prims under the driven provider roots in
    // _xformResetRoots, walked again only when the provider set or the
    // stage edit serial moves. A new bridge (each activation) starts empty.
    std::vector<SdfPath> _xformResetRoots;
    std::set<SdfPath> _xformResetPaths;
    uint64_t _xformResetSerial = 0;
    bool _xformResetsValid = false;

    /// Program publications (RigExecGpuDeformMode::Shader): each layout's
    /// tables as Hydra arrays, built once, and per mesh the last frame
    /// published with its time, for velocities.
    std::map<uint64_t,
             std::shared_ptr<const RigExecPublishedPrim::GpuDeformStatic>>
        _gpuDeformStatics;
    std::map<SdfPath, _GpuDeformLast> _gpuDeformLast;
};

} // namespace rigExec

#endif // RIGEXEC_IMAGING_BRIDGE_SNAPSHOT_H
