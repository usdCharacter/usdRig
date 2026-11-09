// Snapshot payload construction owned by the imaging bridge: the pose ->
// published-prims payload (geometry, matrix and float primvars, provider
// base+revision transforms), the authored reset-stack walk and its
// serial/provider-set memo, and the GPU deform layout tables and per-prim
// frame history feeding velocities. One per bridge, constructed after the
// evaluator. The component holds no store, frame cache, scheduler, results
// index, evaluator or bridge back-pointer: the bridge passes the stage, the
// sample time, the evaluator's stage-edit serial and the explicit compiled
// publication metadata on every call (RigExecSnapshotCall). Generation
// stamping, guide fills, generation numbering, the compiled-rig
// announcement and the store Publish/Replace stay on the facade, in the
// current order.
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
#include <vector>

namespace rigExec {

struct RigExecRigPose;

/// The bridge-owned state a snapshot build reads, built at the call site
/// and borrowed for the duration of the call. The component keeps no copy:
/// a recompile or a stage edit takes effect on the next call.
/// \p projectorTargets are the Spatial SurfaceProjector output properties
/// whose matrices stay target-local; \p animatedMapPrimvars are the float
/// primvar properties of animated maps; \p scalarPrimvarTargets and \p
/// scalarPrimvarNames are the compiled scalar chains, paired by index.
struct RigExecSnapshotCall {
    UsdStageRefPtr stage;
    UsdTimeCode time;
    /// The evaluator's stage-edit serial: the authored reset walk reruns
    /// only when this or the driven provider set moves.
    uint64_t stageEditSerial = 0;
    std::vector<SdfPath> projectorTargets;
    std::vector<SdfPath> animatedMapPrimvars;
    const std::vector<SdfPath> *scalarPrimvarTargets = nullptr;
    const std::vector<TfToken> *scalarPrimvarNames = nullptr;
};

/// Which generation a build is for. Displayed: the complete viewport
/// generation, including the GPU deform publication and per-prim frame
/// history. Query: the pose-query payload with the gpuDeform snapshot as
/// the publication only -- no GPU display history is touched (guide fills,
/// the store, notices and the generation stamp never reach this component).
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
    void _FillProviderXforms(const RigExecSnapshotCall &call,
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
