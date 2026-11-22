// The per-mover parameter packet: settings, per-frame values and caches
// one revision kernel reads. Internal to rigExec; not an installed API type.
#ifndef RIGEXEC_MOVER_PACKET_H
#define RIGEXEC_MOVER_PACKET_H

#include "rigExec/api/types.h"
#include "rigExecMath/deltaMushSettings.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/surfaceSnapKernel.h"
#include "rigExecMath/wrinkleSettings.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"

#include <memory>
#include <vector>

namespace rigExec {

struct RigExecMoverHandler;

// Defined in rigExecMath/deltaMushKernel.h; the alias is the one
// rigExecMath/geometryKernels.h declares. Packets hold it by shared_ptr.
template <class Point, class Wide> struct RigExecDeltaMushRestData;
using RigExecDeltaMushRest = RigExecDeltaMushRestData<GfVec3f, GfVec3d>;
struct RigExecVertexNormalsTopology;

/// Immutable per-mover parameter packet (every concrete mover
/// schema owns a statically registered computeMoverParameters). The kind
/// token names the owning operation; unused fields stay default.
struct RigExecMoverParameters {
    /// Operation kind: matrix, skin, blendShape, volumeCorrect, smooth,
    /// lattice, surfaceProject, ribbon, emitGuidePoints,
    /// recomputeNormals, or recomputeExtent.
    TfToken kind;
    bool enabled = true;
    bool valid = false; ///< false => MoverFailed pass-through
    GfMatrix4d transform{1.0};

    /// Take a fraction of the transform's ROTATION rather than of the
    /// resulting position, so a weighted point keeps its distance from
    /// the axis instead of cutting the chord. See
    /// RigExecPartialTransform.
    bool radialWeight = false;
    /// Common MoverAPI envelope. A bound rigExec:weightObject supplies this
    /// packet; otherwise it is the constant packet synthesized from
    /// inputs:defaultWeight. It is applied after the operation computes its
    /// full-strength candidate.
    RigExecWeightPacket weights;
    /// Dense per-point combined blend delta (already channel-scaled).
    std::vector<GfVec3f> blendDeltas;
    bool blendSurfaceFrame = false;

    /// Operation scalars: volumeCorrect reference volume and the internal
    /// full-step strength consumed by smooth/volume/surface kernels.
    /// The user-facing blend is always the common weights packet above.
    double referenceVolume = 0.0;
    float strength = 0.0f;
    int mushIterations = 10;
    float mushStep = 0.5f;
    bool mushPinBorders = true;
    float mushDistanceWeight = 0.0f;
    float mushDisplacement = 1.0f;
    RigExecDeltaMushSettings mushSettings;
    GfMatrix4d mushComputationToTarget{1};
    RigExecWrinkleSettings wrinkleSettings;
    RigExecSurfaceSnapSettings surfaceSettings;
    GfMatrix4d targetToSurface{1.0}, surfaceToTarget{1.0}, surfaceToMetric{1.0};
    RigExecLatticeSettings latticeSettings;
    GfMatrix4d targetToLattice{1.0}, latticeToTarget{1.0}, cageToLattice{1.0};

    /// Standard topology for smooth/normals/surface kernels.
    std::vector<int> topologyCounts;
    std::vector<int> topologyIndices;

    /// Auxiliary native point sets: lattice rest cage or surface driver
    /// points (auxPoints), lattice posed cage (auxPointsB), destination
    /// rest points for lattice binding (restPoints).
    std::vector<GfVec3f> auxPoints;
    std::vector<GfVec3f> auxPointsB;
    std::vector<GfVec3f> restPoints;
    GfVec3i divisions{0, 0, 0};

    /// Curve binding: two-component bind coordinates and the driver's
    /// posed+rest frame samples.
    std::vector<GfVec2f> bindCoords;
    RigExecPointFrameArray frames;
    /// Wire: bind coordinates as the authored array itself. Shared, not
    /// copied: a wire's table covers the whole mesh, is read every frame and
    /// compared every frame, and a VtArray read from the stage hands back
    /// the layer's own buffer, so both are a reference count and an
    /// identity test.
    VtArray<GfVec2f> wireBindCoords;

    /// Wire: the driver NURBS curve's order and knots (restPoints are its
    /// rest control points, auxPoints its posed ones) and the dropoff
    /// distance of the falloff.
    int curveOrder = 0;
    std::vector<double> curveKnots;
    double dropoffDistance = 0.0;

    /// Widths for the extent computation (empty, one, or per-point).
    std::vector<float> widths;

    /// Skin: one matrix per rigExec:influences entry, UsdSkel-layout
    /// indices and weights (skinElementSize slots per point), and the
    /// method token the kernel dispatches on (classicLinear |
    /// dualQuaternion).
    std::vector<GfMatrix4d> skinTransforms;
    std::vector<int> skinIndices;
    std::vector<float> skinWeights;
    int skinElementSize = 0;
    TfToken skinningMethod;

    /// The compiled rig layout, when the evaluator resolved one. Set means
    /// skinIndices/skinWeights are empty and the kernel reads the arrays
    /// here instead -- which avoids re-reading, re-copying and re-validating
    /// them once per frame.
    std::shared_ptr<const RigExecSkinTopology> skinTopology;

    /// The compiled rig rest state, when the evaluator resolved one: the
    /// smoothing table, the smoothed rest surface and the transport
    /// frames, derived once instead of on every frame. The packet arrays
    /// stay filled; the entry feeds the kernel only. Excluded from
    /// operator==: pure function of the arrays.
    std::shared_ptr<const RigExecDeltaMushRest> mushRest;

    /// Recomputed normals: one value per face corner (faceVarying) rather
    /// than per point, negated on a leftHanded mesh. Compile facts.
    bool normalsPerCorner = false;
    bool normalsLeftHanded = false;
    /// The compiled vertex-normal walk; kernel-only and outside operator==
    /// for the reason mushRest is.
    std::shared_ptr<const RigExecVertexNormalsTopology> normalsTopology;

    bool operator==(const RigExecMoverParameters &o) const
    {
        return kind == o.kind && enabled == o.enabled && valid == o.valid &&
               transform == o.transform && radialWeight == o.radialWeight &&
               weights == o.weights && blendDeltas == o.blendDeltas &&
               blendSurfaceFrame == o.blendSurfaceFrame &&
               referenceVolume == o.referenceVolume && strength == o.strength &&
               mushIterations == o.mushIterations && mushStep == o.mushStep &&
               mushPinBorders == o.mushPinBorders &&
               mushDistanceWeight == o.mushDistanceWeight &&
               mushDisplacement == o.mushDisplacement &&
               mushSettings == o.mushSettings &&
               mushComputationToTarget == o.mushComputationToTarget &&
               wrinkleSettings == o.wrinkleSettings &&
               surfaceSettings == o.surfaceSettings &&
               targetToSurface == o.targetToSurface &&
               surfaceToTarget == o.surfaceToTarget &&
               surfaceToMetric == o.surfaceToMetric &&
               latticeSettings == o.latticeSettings &&
               targetToLattice == o.targetToLattice &&
               latticeToTarget == o.latticeToTarget &&
               cageToLattice == o.cageToLattice &&
               topologyCounts == o.topologyCounts &&
               topologyIndices == o.topologyIndices &&
               auxPoints == o.auxPoints && auxPointsB == o.auxPointsB &&
               restPoints == o.restPoints && divisions == o.divisions &&
               bindCoords == o.bindCoords && frames == o.frames &&
               wireBindCoords == o.wireBindCoords &&
               curveOrder == o.curveOrder && curveKnots == o.curveKnots &&
               dropoffDistance == o.dropoffDistance && widths == o.widths &&
               skinTransforms == o.skinTransforms &&
               skinIndices == o.skinIndices && skinWeights == o.skinWeights &&
               skinTopology == o.skinTopology &&
               // mushRest: derived, excluded.
               skinElementSize == o.skinElementSize &&
               skinningMethod == o.skinningMethod &&
               normalsPerCorner == o.normalsPerCorner &&
               normalsLeftHanded == o.normalsLeftHanded;
    }
    bool operator!=(const RigExecMoverParameters &o) const
    {
        return !(*this == o);
    }
};

/// An External revision's plugin payload, carried beside its packet. The
/// registered schema identifies the handler's callbacks.
struct RigExecExternalRevision {
    const RigExecMoverHandler *handler = nullptr;
    TfToken schema;
    VtValue data;
    /// A windowed revision's run (rigExecGraph/windowHistory.h), pointed at
    /// by the step body for the length of the kernel run and null otherwise.
    /// Not part of the value: excluded from ==.
    struct RigExecWindowRun *window = nullptr;

    bool operator==(const RigExecExternalRevision &o) const
    {
        return schema == o.schema && data == o.data;
    }
    bool operator!=(const RigExecExternalRevision &o) const
    {
        return !(*this == o);
    }
};

} // namespace rigExec

#endif // RIGEXEC_MOVER_PACKET_H
