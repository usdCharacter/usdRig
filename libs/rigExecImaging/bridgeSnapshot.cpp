// Snapshot payload construction owned by the imaging bridge.
#include "bridgeSnapshot.h"

#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace rigExec {

namespace {

const std::string primvarsPrefix("primvars:");

} // namespace

void
RigExecBridgeSnapshotBuilder::_Build(const RigExecSnapshotCall &call,
                                     const RigExecRigPose &pose,
                                     RigExecSnapshotBuildMode mode,
                                     RigExecImagingSnapshot *snapshot)
{
    // Build the complete immutable generation from the exact native
    // results: points and normals as flat primvars, extent as min/max.
    // Only standard data crosses this boundary. Query mode builds only
    // what a pose query reads; the generation stamp, the guide fills and
    // the GPU deform history belong to the publisher.
    const bool displayed = mode == RigExecSnapshotBuildMode::Displayed;
    // Matrix primvars a rig computes for a shader on the prim (a surface
    // projector's frame and dials), published as constant primvars. The
    // rig writes them as moved `primvars:<name>` properties and nothing
    // authors them; indexed before the geometry loop so a prim carrying
    // only a matrix primvar still gets published.
    const std::vector<SdfPath> &projectorTargets = call.projectorTargets;
    // Animated maps' float primvars: only those the evaluator names, so a
    // property mover's float result never mints a published prim.
    const std::vector<SdfPath> &mapPrimvars = call.animatedMapPrimvars;
    for (const SdfPath &propertyPath : mapPrimvars) {
        const auto found = pose.movedProperties.find(propertyPath);
        if (found == pose.movedProperties.end() ||
            !found->second.IsHolding<float>()) {
            continue;
        }
        const std::string &name = propertyPath.GetName();
        auto &published = snapshot->prims[propertyPath.GetPrimPath()];
        published.shaderFloats[TfToken(name.substr(primvarsPrefix.size()))] =
            found->second.UncheckedGet<float>();
    }
    // Property chains' scalar primvars, from the compiled list only; a
    // double publishes as the float the chain computed.
    const std::vector<SdfPath> &scalarTargets = *call.scalarPrimvarTargets;
    const std::vector<TfToken> &scalarNames = *call.scalarPrimvarNames;
    for (size_t scalar = 0; scalar < scalarTargets.size(); ++scalar) {
        const SdfPath &propertyPath = scalarTargets[scalar];
        const auto found = pose.movedProperties.find(propertyPath);
        if (found == pose.movedProperties.end()) {
            continue;
        }
        float value = 0.0f;
        if (found->second.IsHolding<float>()) {
            value = found->second.UncheckedGet<float>();
        } else if (found->second.IsHolding<double>()) {
            value = float(found->second.UncheckedGet<double>());
        } else {
            continue;
        }
        auto &published = snapshot->prims[propertyPath.GetPrimPath()];
        published.shaderFloats[scalarNames[scalar]] = value;
    }
    for (const auto &[propertyPath, value] : pose.movedProperties) {
        if (!value.IsHolding<GfMatrix4d>()) {
            continue;
        }
        const std::string &name = propertyPath.GetName();
        if (name.compare(0, primvarsPrefix.size(), primvarsPrefix) != 0) {
            continue;
        }
        auto &published = snapshot->prims[propertyPath.GetPrimPath()];
        const TfToken primvar(name.substr(primvarsPrefix.size()));
        published.shaderMatrices[primvar] = value.UncheckedGet<GfMatrix4d>();
        if (std::binary_search(projectorTargets.begin(), projectorTargets.end(),
                               propertyPath)) {
            published.targetLocalShaderMatrices.insert(primvar);
        }
    }
    for (const auto &[propertyPath, value] : pose.movedProperties) {
        const SdfPath primPath = propertyPath.GetPrimPath();
        const TfToken property = propertyPath.GetNameToken();
        // Only geometry crosses into imaging. movedProperties also carries
        // the property-domain math mover results (a float dial, a vector
        // offset, a matrix), which have no Hydra representation -- and
        // indexing snapshot->prims for one of those would MINT an empty
        // published prim for its owner, which then enters the compiled rig
        // below and makes the scene index resolve a prim RigExec publishes
        // nothing for. Look the entry up only once there is something to
        // put in it.
        if (property == "points" &&
            value.IsHolding<RigExecGpuDeformPublication>()) {
            const auto &publication =
                value.UncheckedGet<RigExecGpuDeformPublication>();
            if (displayed) {
                _PublishGpuDeform(call, primPath, publication,
                                  &snapshot->prims[primPath]);
            } else if (publication.layout && publication.frame) {
                snapshot->prims[primPath].gpuDeform =
                    std::make_shared<const RigExecGpuDeformPublication>(
                        publication);
            }
            continue;
        }
        const bool isGeometry = value.IsHolding<VtVec3fArray>() &&
                                (property == "points" ||
                                 property == "normals" || property == "extent");
        if (!isGeometry) {
            // Not Hydra data -- but a scalar here is a rig output a
            // tool may want (a blend weight, a pose-interpolator
            // result), and recovering it otherwise costs a whole
            // second evaluation of the rig. Recorded beside the
            // generation that produced it; see
            // RigExecImagingSnapshot::movedFloats.
            if (value.IsHolding<float>()) {
                snapshot->movedFloats[propertyPath] =
                    value.UncheckedGet<float>();
            } else if (value.IsHolding<double>()) {
                snapshot->movedFloats[propertyPath] =
                    static_cast<float>(value.UncheckedGet<double>());
            } else if (value.IsHolding<VtDoubleArray>()) {
                snapshot->movedArrays[propertyPath] =
                    value.UncheckedGet<VtDoubleArray>();
            }
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[primPath];
        if (property == "points") {
            published.hasPoints = true;
            published.points = value.UncheckedGet<VtVec3fArray>();
        } else if (property == "normals") {
            published.hasNormals = true;
            published.normals = value.UncheckedGet<VtVec3fArray>();
        } else {
            const auto extent = value.UncheckedGet<VtVec3fArray>();
            if (extent.size() == 2) {
                published.hasExtent = true;
                published.extentMin = GfVec3d(extent[0]);
                published.extentMax = GfVec3d(extent[1]);
            }
        }
    }
    _FillProviderXforms(call, pose, snapshot);
}

// Constraint-driven transforms, published onto the prim itself so parented
// geometry rides along. Base and revision travel together: the scene index
// sits downstream of Hydra's flatten and turns the pair into a world-space
// delta (see RigExecPublishedPrim::xformBase).
void
RigExecBridgeSnapshotBuilder::_FillProviderXforms(
    const RigExecSnapshotCall &call, const RigExecRigPose &pose,
    RigExecImagingSnapshot *snapshot)
{
    for (const auto &[providerPath, matrix] : pose.providerXforms) {
        const auto baseIt = pose.providerBaseXforms.find(providerPath);
        if (baseIt == pose.providerBaseXforms.end()) {
            // The evaluator writes both or neither; a revision without its
            // base cannot be turned into a delta, so publishing it would move
            // the prim by an arbitrary amount.
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[providerPath];
        published.xform = matrix;
        published.xformBase = baseIt->second;
        published.hasXform = true;
        snapshot->hasDrivenXforms = true;
    }
    // Flattening stamps resetXformStack=true on every output, so the results
    // index cannot recover authored reset boundaries from Hydra. Capture
    // them while the bridge still owns the source stage, under driven roots.
    // The walk runs when the driven set or the stage changes, not every
    // frame: any authored edit advances the evaluator's stage edit serial.
    std::vector<SdfPath> providerRoots;
    providerRoots.reserve(pose.providerXforms.size());
    for (const auto &[providerPath, matrix] : pose.providerXforms) {
        providerRoots.push_back(providerPath);
    }
    if (!_xformResetsValid || _xformResetSerial != call.stageEditSerial ||
        providerRoots != _xformResetRoots) {
        _xformResetPaths.clear();
        std::vector<SdfPath> scanned;
        for (const SdfPath &providerPath : providerRoots) {
            bool covered = false;
            for (const auto &root : scanned)
                covered = covered || providerPath.HasPrefix(root);
            if (covered)
                continue;
            const auto root = call.stage->GetPrimAtPath(providerPath);
            if (!root)
                continue;
            scanned.push_back(providerPath);
            for (const auto &prim : UsdPrimRange(root)) {
                const UsdGeomXformable xform(prim);
                if (xform && xform.GetResetXformStack())
                    _xformResetPaths.insert(prim.GetPath());
            }
        }
        _xformResetRoots = std::move(providerRoots);
        _xformResetSerial = call.stageEditSerial;
        _xformResetsValid = true;
    }
    snapshot->xformResetPaths.insert(_xformResetPaths.begin(),
                                     _xformResetPaths.end());
}

void
RigExecBridgeSnapshotBuilder::_PublishGpuDeform(
    const RigExecSnapshotCall &call, const SdfPath &primPath,
    const RigExecGpuDeformPublication &publication,
    RigExecPublishedPrim *published)
{
    if (!publication.layout || !publication.frame) {
        return;
    }
    const UsdTimeCode time = call.time;
    const RigExecGpuDeformLayout &layout = *publication.layout;
    // The tables once per layout; a layout no publication names any more is
    // dropped when the cache would otherwise grow past a handful.
    auto &statics = _gpuDeformStatics[layout.serial];
    if (!statics) {
        if (_gpuDeformStatics.size() > 32) {
            _gpuDeformStatics.clear();
        }
        auto built = std::make_shared<RigExecPublishedPrim::GpuDeformStatic>();
        built->serial = layout.serial;
        built->ops.assign(layout.ops.begin(), layout.ops.end());
        built->staticF.assign(layout.staticF.begin(), layout.staticF.end());
        built->staticI.assign(layout.staticI.begin(), layout.staticI.end());
        built->opCount = int(layout.OpCount());
        built->pointCount = layout.pointCount;
        statics = built;
        _gpuDeformStatics[layout.serial] = statics;
    }
    published->gpuDeform =
        std::make_shared<const RigExecGpuDeformPublication>(publication);
    published->gpuDeformStatic = statics;
    // Velocities difference this frame against the one a frame earlier,
    // in units per second; anything else (a drag at one time, a jump, a
    // new layout) has no adjacent frame and publishes zero.
    _GpuDeformLast &last = _gpuDeformLast[primPath];
    if (last.serial != layout.serial ||
        last.frameVersion != publication.frameVersion ||
        last.numeric != time.IsNumeric() ||
        (time.IsNumeric() && last.time != time.GetValue())) {
        VtFloatArray frame(publication.frame->begin(),
                           publication.frame->end());
        const bool adjacent =
            time.IsNumeric() && last.numeric && last.serial == layout.serial &&
            std::abs(time.GetValue() - last.time - 1.0) < 1e-6;
        last.previous = adjacent ? last.frame : frame;
        last.frame = frame;
        last.serial = layout.serial;
        last.frameVersion = publication.frameVersion;
        last.numeric = time.IsNumeric();
        last.time = time.IsNumeric() ? time.GetValue() : 0.0;
    }
    published->gpuDeformFrame = last.frame;
    published->gpuDeformPreviousFrame = last.previous;
    published->gpuDeformVelocityScale =
        call.stage ? float(call.stage->GetTimeCodesPerSecond()) : 24.0f;
}

} // namespace rigExec
