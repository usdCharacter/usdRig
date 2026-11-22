#ifndef RIGEXEC_RIG_TYPES_RIG_TYPES_H
#define RIGEXEC_RIG_TYPES_RIG_TYPES_H

/// The rig type library: the values more than one op reads, built from the
/// standard types. Derived structures (adjacency, normal walks, cloth and
/// wrinkle meshes, skin and blend layouts) are op state, not types here.
/// USD-free.

#include "rigExec/api/registry.h"
#include "rigExecStdTypes/stdTypes.h"

#include <array>
#include <cstdint>
#include <tuple>

namespace rigExecRig {

using rigExecStd::RigExecArray;

enum PointFrameFlags : uint32_t {
    kPointFrameValid = 1u << 0,
    kPointFrameDegenerate = 1u << 1,
    kPointFrameReflected = 1u << 2,
    kPointFrameAffine = 1u << 3,
    kPointFrameLiveRest = 1u << 4,
};

/// Origin and the X, Y and Z landmarks of a frame.
using PointLandmarks = std::array<rigExecStd::Vec3d, 4>;

/// A transform carried as four points: origin, X, Y, Z.
struct PointFrame {
    PointLandmarks points{
        rigExecStd::Vec3d(0.0, 0.0, 0.0), rigExecStd::Vec3d(1.0, 0.0, 0.0),
        rigExecStd::Vec3d(0.0, 1.0, 0.0), rigExecStd::Vec3d(0.0, 0.0, 1.0)};
    uint32_t flags = kPointFrameValid;

    static constexpr auto Members()
    {
        return std::make_tuple(&PointFrame::points, &PointFrame::flags);
    }
};

/// A solver's frames and the rest landmarks of each, parallel.
struct PointFrameArray {
    RigExecArray<PointFrame> frames;
    RigExecArray<PointLandmarks> rests;

    static constexpr auto Members()
    {
        return std::make_tuple(&PointFrameArray::frames,
                               &PointFrameArray::rests);
    }
};

enum class WeightRepresentation : uint32_t { Constant, Dense, Sparse };
enum class WeightRange : uint32_t { Strict, Clamp };

/// Per-point weights over a target of some point count: one constant, one
/// value per point, or values at sorted indices with a default elsewhere.
struct PointWeights {
    RigExecArray<float> values;
    RigExecArray<int32_t> indices;
    float defaultWeight = 0.0f;
    uint32_t representation = uint32_t(WeightRepresentation::Constant);
    uint32_t rangePolicy = uint32_t(WeightRange::Strict);
    uint8_t valid = 0;

    static constexpr auto Members()
    {
        return std::make_tuple(
            &PointWeights::values, &PointWeights::indices,
            &PointWeights::defaultWeight, &PointWeights::representation,
            &PointWeights::rangePolicy, &PointWeights::valid);
    }
};

/// Uniform samples of a curve over [0, 1]. Empty means linear.
struct ScalarCurve {
    RigExecArray<float> samples;

    static constexpr auto Members()
    {
        return std::make_tuple(&ScalarCurve::samples);
    }
};

/// Polygon topology: corner counts per face and point indices per corner.
struct MeshTopology {
    RigExecArray<int32_t> faceVertexCounts;
    RigExecArray<int32_t> faceVertexIndices;
    uint32_t pointCount = 0;

    static constexpr auto Members()
    {
        return std::make_tuple(&MeshTopology::faceVertexCounts,
                               &MeshTopology::faceVertexIndices,
                               &MeshTopology::pointCount);
    }
};

/// elementSize influences per point: joint indices and parallel weights.
struct SkinInfluences {
    RigExecArray<int32_t> indices;
    RigExecArray<float> weights;
    uint32_t elementSize = 0;
    uint32_t pointCount = 0;
    /// The joint count indices were checked against.
    uint32_t influenceCount = 0;

    static constexpr auto Members()
    {
        return std::make_tuple(
            &SkinInfluences::indices, &SkinInfluences::weights,
            &SkinInfluences::elementSize, &SkinInfluences::pointCount,
            &SkinInfluences::influenceCount);
    }
};

/// Shape targets over a pointCount-point base. Target t owns offsets
/// [offsetStarts[t], offsetStarts[t + 1]) and indices [indexStarts[t],
/// indexStarts[t + 1]); with no indices its offsets are dense.
struct BlendShapeTargets {
    RigExecArray<rigExecStd::Vec3f> offsets;
    RigExecArray<int32_t> indices;
    RigExecArray<uint32_t> offsetStarts;
    RigExecArray<uint32_t> indexStarts;
    uint32_t pointCount = 0;

    static constexpr auto Members()
    {
        return std::make_tuple(
            &BlendShapeTargets::offsets, &BlendShapeTargets::indices,
            &BlendShapeTargets::offsetStarts, &BlendShapeTargets::indexStarts,
            &BlendShapeTargets::pointCount);
    }
};

/// Collider triangles over pointCount concatenated collider points.
struct CollisionShapes {
    RigExecArray<int32_t> triangles;
    uint32_t pointCount = 0;

    static constexpr auto Members()
    {
        return std::make_tuple(&CollisionShapes::triangles,
                               &CollisionShapes::pointCount);
    }
};

/// A NURBS curve. Empty weights means non-rational.
struct NurbsCurve {
    RigExecArray<double> knots;
    RigExecArray<rigExecStd::Vec3d> points;
    RigExecArray<double> weights;
    uint32_t order = 4;

    static constexpr auto Members()
    {
        return std::make_tuple(&NurbsCurve::knots, &NurbsCurve::points,
                               &NurbsCurve::weights, &NurbsCurve::order);
    }
};

const rigExec::RigExecTypeHandle &PointLandmarksType();
const rigExec::RigExecTypeHandle &PointLandmarksList();
const rigExec::RigExecTypeHandle &PointFrameType();
const rigExec::RigExecTypeHandle &PointFrameList();
const rigExec::RigExecTypeHandle &PointFrameArrayType();
const rigExec::RigExecTypeHandle &PointWeightsType();
const rigExec::RigExecTypeHandle &ScalarCurveType();
const rigExec::RigExecTypeHandle &MeshTopologyType();
const rigExec::RigExecTypeHandle &SkinInfluencesType();
const rigExec::RigExecTypeHandle &BlendShapeTargetsType();
const rigExec::RigExecTypeHandle &CollisionShapesType();
const rigExec::RigExecTypeHandle &NurbsCurveType();

/// Points are the standard point arrays.
inline const rigExec::RigExecTypeHandle &
Points3fType()
{
    return rigExecStd::Vec3fArray();
}
inline const rigExec::RigExecTypeHandle &
Points3dType()
{
    return rigExecStd::Vec3dArray();
}

} // namespace rigExecRig

#endif
