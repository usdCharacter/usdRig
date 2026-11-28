// RigExec geometry mover kernels implementation.
#include "geometryKernels.h"
#include "deltaMushKernel.h"
#include "dualQuat.h"
#include "latticeKernel.h"
#include "ribbonKernel.h"
#include "spatialAccel.h"
#include "surfaceProjectKernel.h"
#include "meshBoundsKernel.h"
#include "smoothKernel.h"
#include "wireBasisKernel.h"
#include "wrinkleKernel.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/rotation.h"

#include <atomic>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <utility>

namespace rigExec {

double
RigExecBoundVolume(const GfVec3f *points, size_t count)
{
    return RigExecBoundVolumeKernel(points, count);
}

void
RigExecApplyVolumeCorrect(std::vector<GfVec3f> *points, double referenceVolume,
                          double strength)
{
    RigExecApplyVolumeCorrectKernel(points, referenceVolume, strength);
}

bool
RigExecTransportSurfaceOffsets(const std::vector<GfVec3f> &rest,
                               const std::vector<GfVec3f> &posed,
                               const std::vector<int> &faceCounts,
                               const std::vector<int> &faceIndices,
                               const std::vector<GfVec3f> &deltas,
                               std::vector<GfVec3f> *out)
{
    return RigExecTransportSurfaceOffsetsKernel<GfVec3f, GfVec3d>(
        rest, posed, faceCounts, faceIndices, deltas, out);
}

bool
RigExecRaycastSurface(const std::vector<GfVec3f> &points,
                      const std::vector<int> &faceVertexCounts,
                      const std::vector<int> &faceVertexIndices,
                      const GfVec3d &origin, const GfVec3d &direction,
                      RigExecSurfaceHit *hit)
{
    return RigExecRaycastSurfaceT(points, faceVertexCounts, faceVertexIndices,
                                  origin, direction, hit);
}

bool
RigExecSurfaceFrameAtHit(const std::vector<GfVec3f> &points,
                         const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices,
                         const RigExecSurfaceHit &hit, const GfVec3d &upHint,
                         GfMatrix4d *frame)
{
    return RigExecSurfaceFrameAtHitT(
        points, faceVertexCounts, faceVertexIndices, hit, upHint,
        static_cast<RigExecVertexNormalsFn>(&RigExecComputeVertexNormals),
        frame);
}

bool
RigExecRaycastSurfaceFrame(const std::vector<GfVec3f> &points,
                           const std::vector<int> &faceVertexCounts,
                           const std::vector<int> &faceVertexIndices,
                           const GfVec3d &origin, const GfVec3d &direction,
                           const GfVec3d &upHint, GfMatrix4d *frame)
{
    if (!frame)
        return false;
    RigExecSurfaceHit hit;
    if (!RigExecRaycastSurface(points, faceVertexCounts, faceVertexIndices,
                               origin, direction, &hit)) {
        return false;
    }
    return RigExecSurfaceFrameAtHit(points, faceVertexCounts, faceVertexIndices,
                                    hit, upHint, frame);
}

RigExecPartialDecomposition
RigExecDecomposePartialTransform(const GfMatrix4d &transform)
{
    return RigExecDecomposePartialTransformKernel<GfVec3d>(transform);
}

GfMatrix4d
RigExecApplyPartialDecomposition(const RigExecPartialDecomposition &d, double w)
{
    return RigExecApplyPartialDecompositionKernel<GfMatrix4d>(d, w);
}

GfMatrix4d
RigExecPartialTransform(const GfMatrix4d &transform, double weight)
{
    const double w = GfClamp(weight, 0.0, 1.0);
    if (w <= 0.0) {
        return GfMatrix4d(1.0);
    }
    if (w >= 1.0) {
        return transform;
    }
    return RigExecApplyPartialDecomposition(
        RigExecDecomposePartialTransform(transform), w);
}

void
RigExecApplyLaplacianSmoothWithAdjacency(std::vector<GfVec3f> *points,
                                         const RigExecMeshAdjacency &adjacency,
                                         double strength, const GfVec3f *source,
                                         size_t sourceCount)
{
    RigExecApplyLaplacianSmoothWithAdjacencyKernel(points, adjacency, strength,
                                                   source, sourceCount);
}

void
RigExecApplyLaplacianSmooth(std::vector<GfVec3f> *points,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize, double strength,
                            const GfVec3f *source, size_t sourceCount)
{
    RigExecApplyLaplacianSmoothMeshKernel(
        points, faceVertexCounts, faceVertexCountsSize, faceVertexIndices,
        faceVertexIndicesSize, strength, source, sourceCount);
}

void
RigExecApplyLaplacianSmooth(std::vector<GfVec3f> *points,
                            const std::vector<int> &faceVertexCounts,
                            const std::vector<int> &faceVertexIndices,
                            double strength, const GfVec3f *source,
                            size_t sourceCount)
{
    RigExecApplyLaplacianSmooth(
        points, faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), strength, source,
        sourceCount);
}

bool
RigExecApplyDeltaMush(
    std::vector<GfVec3f> *points, const std::vector<GfVec3f> &rest,
    const std::vector<int> &counts, const std::vector<int> &indices,
    int iterations, double step, bool pinBorders, double distanceWeight,
    double displacement, const RigExecDeltaMushSettings &settings,
    const GfMatrix4d &computationToTarget, const RigExecDeltaMushRest *restData)
{
    if (restData) {
        return RigExecApplyDeltaMushInSpaceWithRestData<GfVec3f, GfVec3d,
                                                        GfMatrix4d>(
            points, *restData, displacement, computationToTarget);
    }
    return RigExecApplyDeltaMushInSpaceKernel<GfVec3f, GfVec3d, GfMatrix4d>(
        points, rest, counts, indices, iterations, step, pinBorders,
        distanceWeight, displacement, settings, computationToTarget);
}

bool
RigExecApplyWrinkle(std::vector<GfVec3f> *points,
                    const std::vector<GfVec3f> &rest,
                    const std::vector<int> &counts,
                    const std::vector<int> &indices,
                    const RigExecWrinkleSettings &settings,
                    const RigExecWrinkleMesh *topology)
{
    if (topology) {
        return RigExecApplyWrinkleWithTopology<GfVec3f, GfVec3d>(
            points, rest, counts, indices, settings, *topology);
    }
    return RigExecApplyWrinkleKernel<GfVec3f, GfVec3d>(points, rest, counts,
                                                       indices, settings);
}

RigExecVertexNormalsTopology
RigExecBuildVertexNormalsTopology(size_t pointsSize,
                                  const int *faceVertexCounts,
                                  size_t faceVertexCountsSize,
                                  const int *faceVertexIndices,
                                  size_t faceVertexIndicesSize)
{
    return RigExecBuildVertexNormalsTopologyKernel(
        pointsSize, faceVertexCounts, faceVertexCountsSize, faceVertexIndices,
        faceVertexIndicesSize);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(const GfVec3f *points, size_t pointsSize,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize,
                            const RigExecVertexNormalsTopology &topology,
                            const RigExecRangeRunner *ranges)
{
    return RigExecComputeVertexNormalsKernel(
        points, pointsSize, faceVertexCounts, faceVertexCountsSize,
        faceVertexIndices, faceVertexIndicesSize, &topology, ranges);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(const GfVec3f *points, size_t pointsSize,
                            const int *faceVertexCounts,
                            size_t faceVertexCountsSize,
                            const int *faceVertexIndices,
                            size_t faceVertexIndicesSize,
                            const RigExecRangeRunner *ranges)
{
    return RigExecComputeVertexNormalsKernel(
        points, pointsSize, faceVertexCounts, faceVertexCountsSize,
        faceVertexIndices, faceVertexIndicesSize, nullptr, ranges);
}

std::vector<GfVec3f>
RigExecComputeVertexNormals(const std::vector<GfVec3f> &points,
                            const std::vector<int> &faceVertexCounts,
                            const std::vector<int> &faceVertexIndices)
{
    return RigExecComputeVertexNormals(
        points.data(), points.size(), faceVertexCounts.data(),
        faceVertexCounts.size(), faceVertexIndices.data(),
        faceVertexIndices.size());
}

std::vector<GfVec3f>
RigExecComputeExtent(const GfVec3f *points, size_t pointsSize,
                     const float *widths, size_t widthsSize)
{
    return RigExecComputeExtentKernel(points, pointsSize, widths, widthsSize);
}

std::vector<GfVec3f>
RigExecComputeExtent(const std::vector<GfVec3f> &points,
                     const std::vector<float> &widths)
{
    return RigExecComputeExtent(points.data(), points.size(), widths.data(),
                                widths.size());
}

void
RigExecApplyLattice(std::vector<GfVec3f> *points, const GfVec3f *restPoints,
                    size_t restPointsSize, const GfVec3f *restCage,
                    size_t restCageSize, const GfVec3f *posedCage,
                    size_t posedCageSize, const GfVec3i &divisions)
{
    RigExecApplyLatticeKernel(
        points, restPoints, restPointsSize, restCage, restCageSize, posedCage,
        posedCageSize, divisions[0], divisions[1], divisions[2],
        static_cast<RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *>(nullptr));
}

void
RigExecApplyLattice(std::vector<GfVec3f> *points,
                    const std::vector<GfVec3f> &restPoints,
                    const std::vector<GfVec3f> &restCage,
                    const std::vector<GfVec3f> &posedCage,
                    const GfVec3i &divisions)
{
    RigExecApplyLattice(points, restPoints.data(), restPoints.size(),
                        restCage.data(), restCage.size(), posedCage.data(),
                        posedCage.size(), divisions);
}

void
RigExecApplyLattice(std::vector<GfVec3f> *points,
                    const std::vector<GfVec3f> &restPoints,
                    const std::vector<GfVec3f> &restCage,
                    const std::vector<GfVec3f> &posedCage,
                    const GfVec3i &divisions,
                    RigExecSurfaceKernelCache<GfVec3f, GfVec3d> *cache)
{
    RigExecApplyLatticeKernel(points, restPoints.data(), restPoints.size(),
                              restCage.data(), restCage.size(),
                              posedCage.data(), posedCage.size(), divisions[0],
                              divisions[1], divisions[2], cache);
}

void
RigExecBuildSurfaceAccel(const int *faceVertexCounts,
                         size_t faceVertexCountsSize,
                         const int *faceVertexIndices,
                         size_t faceVertexIndicesSize,
                         const GfVec3f *surfacePoints, size_t surfacePointsSize,
                         RigExecSurfaceAccel *accel)
{
    RigExecBuildSurfaceAccelKernel(faceVertexCounts, faceVertexCountsSize,
                                   faceVertexIndices, faceVertexIndicesSize,
                                   surfacePoints, surfacePointsSize, accel);
}

void
RigExecBuildSurfaceAccel(const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices,
                         const std::vector<GfVec3f> &surfacePoints,
                         RigExecSurfaceAccel *accel)
{
    RigExecBuildSurfaceAccel(faceVertexCounts.data(), faceVertexCounts.size(),
                             faceVertexIndices.data(), faceVertexIndices.size(),
                             surfacePoints.data(), surfacePoints.size(), accel);
}

void
RigExecApplySurfaceProject(std::vector<GfVec3f> *points,
                           const GfVec3f *surfacePoints,
                           size_t surfacePointsSize,
                           const int *faceVertexCounts,
                           size_t faceVertexCountsSize,
                           const int *faceVertexIndices,
                           size_t faceVertexIndicesSize, double weight,
                           const RigExecSurfaceAccel *accel)
{
    RigExecApplySurfaceProjectKernel(points, surfacePoints, surfacePointsSize,
                                     faceVertexCounts, faceVertexCountsSize,
                                     faceVertexIndices, faceVertexIndicesSize,
                                     weight, accel);
}

void
RigExecApplySurfaceProject(std::vector<GfVec3f> *points,
                           const std::vector<GfVec3f> &surfacePoints,
                           const std::vector<int> &faceVertexCounts,
                           const std::vector<int> &faceVertexIndices,
                           double weight, const RigExecSurfaceAccel *accel)
{
    RigExecApplySurfaceProject(
        points, surfacePoints.data(), surfacePoints.size(),
        faceVertexCounts.data(), faceVertexCounts.size(),
        faceVertexIndices.data(), faceVertexIndices.size(), weight, accel);
}

RigExecCurveFrameSamples
RigExecSampleCurveRMF(const GfVec3f *controlPoints, size_t controlPointsSize,
                      int sampleCount)
{
    return RigExecSampleCurveRMFKernel(controlPoints, controlPointsSize,
                                       sampleCount);
}

RigExecCurveFrameSamples
RigExecSampleCurveRMF(const std::vector<GfVec3f> &controlPoints,
                      int sampleCount)
{
    return RigExecSampleCurveRMF(controlPoints.data(), controlPoints.size(),
                                 sampleCount);
}

void
RigExecApplyRibbonTransport(std::vector<GfVec3f> *points,
                            const GfVec2f *bindCoords, size_t bindCoordsSize,
                            const RigExecCurveFrameSamples &restSamples,
                            const RigExecCurveFrameSamples &posedSamples)
{
    const size_t sampleCount = restSamples.GetSize();
    if (points->empty() || posedSamples.GetSize() != sampleCount) {
        return; // invalid binding: pass through
    }
    const auto framePoints = [](const RigExecCurveFrameSamples &samples) {
        std::vector<std::array<GfVec3d, 4>> frames(samples.GetSize());
        for (size_t k = 0; k < frames.size(); ++k) {
            const GfVec3f &o = samples.positions[k];
            frames[k] = {GfVec3d(o), GfVec3d(o + samples.tangents[k]),
                         GfVec3d(o + samples.normals[k]),
                         GfVec3d(o + samples.binormals[k])};
        }
        return frames;
    };
    const std::vector<std::array<GfVec3d, 4>> rest = framePoints(restSamples);
    const std::vector<std::array<GfVec3d, 4>> posed = framePoints(posedSamples);
    // An invalid binding passes the points through.
    RigExecApplyRibbonKernel<RigExecDualQuatMath<GfVec3f, GfVec3d>>(
        points->data(), points->size(), bindCoords, bindCoordsSize, sampleCount,
        [&](size_t k) -> const auto & { return rest[k]; },
        [&](size_t k) -> const auto & { return posed[k]; });
}

void
RigExecApplyRibbonTransport(std::vector<GfVec3f> *points,
                            const std::vector<GfVec2f> &bindCoords,
                            const RigExecCurveFrameSamples &restSamples,
                            const RigExecCurveFrameSamples &posedSamples)
{
    RigExecApplyRibbonTransport(points, bindCoords.data(), bindCoords.size(),
                                restSamples, posedSamples);
}

} // namespace rigExec

namespace rigExec {

bool
RigExecNurbsCurve::IsValid() const
{
    return RigExecNurbsCurveIsValidKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, knots, order);
}

double
RigExecNurbsCurve::DomainStart() const
{
    return RigExecNurbsCurveDomainStartKernel(knots, order);
}

double
RigExecNurbsCurve::DomainEnd() const
{
    return RigExecNurbsCurveDomainEndKernel(points, knots);
}

GfVec3f
RigExecNurbsCurve::Evaluate(double u) const
{
    return RigExecEvaluateNurbsCurveKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, knots, order, u);
}

bool
RigExecBuildWireBasis(const GfVec2f *bindCoords, size_t bindCount,
                      size_t meshPointCount, const std::vector<int> &indices,
                      int order, const std::vector<double> &knots,
                      size_t controlPointCount, double dropoffDistance,
                      RigExecWireBasis *basis)
{
    return RigExecBuildWireBasisKernel<GfVec3f, GfVec3d, GfVec2f>(
        bindCoords, bindCount, meshPointCount, indices, order, knots,
        controlPointCount, dropoffDistance, basis);
}

bool
RigExecWireBasisInputsAreUsable(const GfVec2f *bindCoords, size_t bindCount,
                                size_t meshPointCount, size_t indexCount,
                                int order, const std::vector<double> &knots,
                                size_t controlPointCount)
{
    return RigExecWireBasisInputsAreUsableKernel<GfVec3f, GfVec3d, GfVec2f>(
        bindCoords, bindCount, meshPointCount, indexCount, order, knots,
        controlPointCount);
}

bool
RigExecApplyWireBasis(std::vector<GfVec3f> *points,
                      const RigExecWireBasis &basis,
                      const std::vector<int> &indices,
                      const std::vector<float> &weights,
                      const std::vector<GfVec3f> &restControlPoints,
                      const std::vector<GfVec3f> &posedControlPoints)
{
    return RigExecApplyWireBasisKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, basis, indices, weights, restControlPoints, posedControlPoints);
}

bool
RigExecApplyWireBasisRange(std::vector<GfVec3f> *points,
                           const RigExecWireBasis &basis,
                           const std::vector<int> &indices,
                           const std::vector<float> &weights,
                           const std::vector<GfVec3f> &restControlPoints,
                           const std::vector<GfVec3f> &posedControlPoints,
                           size_t begin, size_t end)
{
    return RigExecApplyWireBasisRangeKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, basis, indices, weights, restControlPoints, posedControlPoints,
        begin, end);
}

bool
RigExecApplyWireBasisGroup(GfVec3f *out, size_t begin, size_t end,
                           const RigExecWireBasis &basis,
                           const std::vector<int> &indices,
                           const std::vector<float> &weights,
                           const std::vector<GfVec3f> &restControlPoints,
                           const std::vector<GfVec3f> &posedControlPoints)
{
    return RigExecApplyWireBasisGroupKernel<GfVec3f, GfVec3d, GfVec2f>(
        out, begin, end, basis, indices, weights, restControlPoints,
        posedControlPoints);
}

std::vector<GfVec2f>
RigExecBindWire(const std::vector<GfVec3f> &points,
                const RigExecNurbsCurve &restCurve)
{
    std::vector<GfVec2f> out(points.size(), GfVec2f(0.0f, 0.0f));
    if (!restCurve.IsValid()) {
        return out;
    }
    const double u0 = restCurve.DomainStart();
    const double u1 = restCurve.DomainEnd();
    const size_t spans = std::max<size_t>(1, restCurve.points->size() -
                                                 size_t(restCurve.order) + 1);
    const size_t samples = spans * 32;
    std::vector<GfVec3f> table(samples + 1);
    for (size_t i = 0; i <= samples; ++i) {
        table[i] = restCurve.Evaluate(u0 + (u1 - u0) * double(i) / samples);
    }
    const double step = (u1 - u0) / double(samples);
    for (size_t pi = 0; pi < points.size(); ++pi) {
        const GfVec3f &p = points[pi];
        size_t best = 0;
        float bestSq = std::numeric_limits<float>::max();
        for (size_t i = 0; i <= samples; ++i) {
            const float dSq = (table[i] - p).GetLengthSq();
            if (dSq < bestSq) {
                bestSq = dSq;
                best = i;
            }
        }
        // Ternary refinement inside the neighbouring samples.
        double lo = std::max(u0, u0 + step * (double(best) - 1.0));
        double hi = std::min(u1, u0 + step * (double(best) + 1.0));
        for (int it = 0; it < 40; ++it) {
            const double m1 = lo + (hi - lo) / 3.0;
            const double m2 = hi - (hi - lo) / 3.0;
            if ((restCurve.Evaluate(m1) - p).GetLengthSq() <
                (restCurve.Evaluate(m2) - p).GetLengthSq()) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        const double u = 0.5 * (lo + hi);
        out[pi] =
            GfVec2f(float(u), float((restCurve.Evaluate(u) - p).GetLength()));
    }
    return out;
}

bool
RigExecApplyWire(std::vector<GfVec3f> *points,
                 const RigExecNurbsCurve &restCurve,
                 const RigExecNurbsCurve &posedCurve, const GfVec2f *bindCoords,
                 size_t bindCount, double dropoffDistance, size_t begin,
                 size_t end, const GfVec3f *restEvals, size_t restEvalCount)
{
    return RigExecApplyWireKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, restCurve, posedCurve, bindCoords, bindCount, dropoffDistance,
        begin, end, restEvals, restEvalCount);
}

bool
RigExecApplyWireGroup(GfVec3f *out, size_t begin, size_t end, size_t count,
                      const RigExecNurbsCurve &restCurve,
                      const RigExecNurbsCurve &posedCurve,
                      const GfVec2f *bindCoords, size_t bindCount,
                      double dropoffDistance, const GfVec3f *restEvals,
                      size_t restEvalCount)
{
    return RigExecApplyWireGroupKernel<GfVec3f, GfVec3d, GfVec2f>(
        out, begin, end, count, restCurve, posedCurve, bindCoords, bindCount,
        dropoffDistance, restEvals, restEvalCount);
}

bool
RigExecWireInputsAreUsable(const RigExecNurbsCurve &restCurve,
                           const RigExecNurbsCurve &posedCurve,
                           const GfVec2f *bindCoords, size_t bindCount,
                           size_t pointCount)
{
    return RigExecWireInputsAreUsableKernel<GfVec3f, GfVec3d, GfVec2f>(
        restCurve, posedCurve, bindCoords, bindCount, pointCount);
}

bool
RigExecApplyWireSparse(std::vector<GfVec3f> *points,
                       const RigExecNurbsCurve &restCurve,
                       const RigExecNurbsCurve &posedCurve,
                       const GfVec2f *bindCoords, size_t bindCount,
                       double dropoffDistance, const std::vector<int> &indices,
                       const std::vector<float> &weights)
{
    return RigExecApplyWireSparseKernel<GfVec3f, GfVec3d, GfVec2f>(
        points, restCurve, posedCurve, bindCoords, bindCount, dropoffDistance,
        indices, weights);
}

bool
RigExecApplyPatchMover(std::vector<GfVec3f> *points,
                       const RigExecNurbsPatch &patch, const GfVec3f *uvPoints,
                       size_t uvPointCount, const RigExecRangeRunner *ranges)
{
    return points &&
           RigExecApplyPatchKernel(points->data(), points->size(), patch,
                                   uvPoints, uvPointCount, ranges);
}

} // namespace rigExec
