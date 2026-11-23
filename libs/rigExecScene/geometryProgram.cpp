#include "geometryProgram.h"
#include "rigExecMath/blendLayoutKernel.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/internal/pathText.h"
#include <cmath>
namespace rigExec {
namespace {
RigExecRevisionLeafView
LeavesOf(const RigExecGeometryRecord &record,
         const RigExecGeometryInputs &inputs)
{
    RigExecRevisionLeafView leaves;
    leaves.decl = &record.leaves;
    leaves.values = inputs.leaves;
    leaves.phased = inputs.phased;
    return leaves;
}
RigExecProviderValues
ProvidersOf(const RigExecGeometryInputs &inputs)
{
    RigExecProviderValues providers;
    providers.transform = inputs.transform;
    providers.carry = inputs.carry;
    providers.influenceTransforms = inputs.influenceTransforms;
    providers.weights = inputs.weights;
    providers.driverFrames = inputs.driverFrames;
    providers.skinTopology = inputs.skinTopology;
    providers.basePoints = inputs.basePoints;
    providers.blendDeltas = inputs.blendDeltas;
    providers.blendFromChannels = inputs.blendFromChannels;
    return providers;
}
} // namespace
RigExecMoverParameters
RigExecAssembleGeometry(const RigExecGeometryRecord &record,
                        const RigExecGeometryInputs &inputs,
                        RigExecExternalRevision *external)
{
    return RigExecAssembleFromLeaves(record.op, record.binding,
                                     LeavesOf(record, inputs),
                                     ProvidersOf(inputs), external);
}
RigExecMoverParameters
RigExecGatherExternalGeometryInputs(const RigExecGeometryRecord &record,
                                    const RigExecGeometryInputs &inputs,
                                    std::vector<VtValue> *declared)
{
    return RigExecGatherExternalInputs(record.binding, LeavesOf(record, inputs),
                                       ProvidersOf(inputs), declared);
}
RigExecMoverParameters
RigExecAssembleGeometry(RigExecRevisionOp op,
                        const RigExecRevisionBinding &binding,
                        const RigExecRevisionLeafView &leaves,
                        const RigExecProviderValues &values,
                        RigExecExternalRevision *external)
{
    return RigExecAssembleFromLeaves(op, binding, leaves, values, external);
}
GfMatrix4d
RigExecGeometryMatrixInPointFrame(const GfMatrix4d &transform,
                                  const GfMatrix4d *space,
                                  const GfMatrix4d *reference,
                                  const GfMatrix4d *referenceSpace,
                                  bool posedPoints, const GfMatrix4d *carry)
{
    GfMatrix4d result = reference
                            ? RigExecMeasureFromReference(transform, *reference)
                            : transform;
    if (space) {
        const auto measured =
            referenceSpace
                ? RigExecMeasureFromReference(*space, *referenceSpace)
                : *space;
        result =
            RigExecClusterInPointFrame(RigExecMeasureInSpace(result, measured),
                                       measured, posedPoints, carry);
    }
    return result;
}
bool
RigExecBuildGeometryBlendLayout(const VtVec3fArray &offsets,
                                const VtIntArray &indices, size_t pointCount,
                                RigExecBlendSampleLayout *layout)
{
    return RigExecBuildBlendLayout(offsets, indices, pointCount, layout);
}
bool
RigExecGeometryBlendDeltas(const std::vector<RigExecBlendChannel> &channels,
                           const std::vector<GfVec3f> &base,
                           std::vector<GfVec3f> *deltas)
{
    return RigExecSumBlendChannels(channels, base, deltas);
}
bool
RigExecGeometryBlendDeltas(const std::vector<RigExecBlendChannel> &channels,
                           const GfVec3f *base, size_t count,
                           std::vector<GfVec3f> *deltas)
{
    return RigExecSumBlendChannels(channels, base, count, deltas);
}
bool
RigExecRunGeometryDerived(RigExecRevisionOp op,
                          const RigExecMoverParameters &parameters,
                          const GfVec3f *authored, size_t authoredCount,
                          std::vector<GfVec3f> *result)
{
    return RigExecApplyDerivedKernel(op, parameters, authored, authoredCount,
                                     result);
}
bool
RigExecRunGeometry(RigExecRevisionOp op,
                   const RigExecMoverParameters &parameters,
                   std::vector<GfVec3f> *points, bool useSimd,
                   RigExecWireBasisCache *wire,
                   RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *surface,
                   const RigExecExternalRevision *external)
{
    return points && RigExecRunRevisionKernel(op, parameters, points, useSimd,
                                              wire, surface, external);
}
namespace geometryDetail {
bool
RunDiscardableGeometry(RigExecRevisionOp op,
                       const RigExecMoverParameters &parameters,
                       std::vector<GfVec3f> *points, bool useSimd,
                       RigExecWireBasisCache *wire,
                       RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *surface,
                       const std::vector<float> *envelope,
                       const RigExecExternalRevision *external)
{
    return points &&
           RunDiscardableRevisionKernel(op, parameters, points, useSimd, wire,
                                        surface, envelope, external);
}
bool
RunDiscardableGeometry(RigExecRevisionOp op,
                       const RigExecMoverParameters &parameters,
                       const GfVec3f *in, size_t count,
                       std::vector<GfVec3f> *out, bool useSimd,
                       RigExecWireBasisCache *wire,
                       RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *surface,
                       const std::vector<float> *envelope,
                       const RigExecExternalRevision *external)
{
    return out &&
           RunDiscardableRevisionKernel(op, parameters, in, count, out, useSimd,
                                        wire, surface, envelope, external);
}
} // namespace geometryDetail
bool
RigExecRunGeometryMatrix(RigExecRevisionOp op,
                         const RigExecRevisionBinding &binding,
                         const RigExecSurfaceProjectorFrames &frames,
                         const RigExecProjectorReads &reads,
                         const std::vector<GfVec3f> &base,
                         const std::vector<GfVec3f> &final,
                         const std::string &who, GfMatrix4d *matrix,
                         std::vector<std::string> *diagnostics,
                         RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache)
{
    return matrix &&
           RigExecRunProjectorTarget(op, binding, frames, reads, base, final,
                                     who, matrix, diagnostics, cache);
}
bool
RigExecRunGeometryMatrix(const RigExecGeometryRecord &record,
                         const RigExecSurfaceProjectorFrames &frames,
                         const RigExecProjectorReads &reads,
                         const std::vector<GfVec3f> &base,
                         const std::vector<GfVec3f> &final,
                         RigExecGeometryWorkspace *workspace,
                         GfMatrix4d *matrix,
                         std::vector<std::string> *diagnostics)
{
    // Only the surface projector names its mover in a diagnostic.
    const std::string who = record.op == RigExecRevisionOp::SurfaceProjector
                                ? RigExecGeometryMoverText(record.binding)
                                : std::string();
    return workspace && matrix &&
           RigExecRunProjectorTarget(record.op, record.binding, frames, reads,
                                     base, final, who, matrix, diagnostics,
                                     &workspace->surface);
}
bool
RigExecRunGeometry(const RigExecGeometryRecord &record,
                   const RigExecMoverParameters &parameters,
                   RigExecGeometryWorkspace *workspace,
                   std::vector<GfVec3f> *points, bool useSimd,
                   const RigExecExternalRevision *external)
{
    if (!workspace)
        return false;
    return RigExecRunGeometry(record.op, parameters, points, useSimd,
                              &workspace->wire, &workspace->surface, external);
}
bool
RigExecGeometryPacketMatches(RigExecRevisionOp op,
                             const RigExecMoverParameters &parameters)
{
    return parameters.valid && parameters.kind == RigExecRevisionKindToken(op);
}
std::string
RigExecGeometryMoverText(const RigExecRevisionBinding &binding)
{
    std::string who;
    if (!RigExecSpellPathText(binding.moverPath, &who) &&
        !binding.moverPath.IsEmpty())
        who = binding.moverPath.GetAsString();
    return who;
}
} // namespace rigExec
