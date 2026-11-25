#ifndef RIGEXEC_OPS_WEIGHT_WEIGHT_OPS_H
#define RIGEXEC_OPS_WEIGHT_WEIGHT_OPS_H

/// The weight producers: sphere, plane and curve fields over a target's
/// points, in the volume's rigid local space; the painted, remapped, volume
/// and combined weight packets; and the ordered combine of weights. The
/// kernels are rigExecMath/weightFieldKernel.h and rigExecEvalCore/weights.h.

#include "rigExecOps/library.h"
#include "rigExecRigTypes/rigTypes.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

enum class WeightFieldAxis : uint8_t { X, Y, Z };

/// What every weight field op is configured with at compile.
struct WeightFieldPayload {
    /// Uniform samples of the falloff curve over r in [0, 1]; fewer than
    /// two is linear. A fed FalloffSamples replaces it.
    std::vector<float> falloffCurve;
    /// Out-of-range weights clamp (true) or fail the op (false).
    bool clamp = false;
    /// Plane: the axis the signed distance runs along, and whether the
    /// field is zero outside the ExtentU by ExtentV rectangle.
    WeightFieldAxis planeAxis = WeightFieldAxis::Z;
    bool planeBounded = false;

    bool operator==(const WeightFieldPayload &o) const
    {
        return falloffCurve == o.falloffCurve && clamp == o.clamp &&
               planeAxis == o.planeAxis && planeBounded == o.planeBounded;
    }
};

/// Codes as rigExecEvalCore's WeightCombineCode.
enum class WeightCombineMode : uint8_t {
    Multiply,
    Add,
    Subtract,
    Max,
    Min,
    Average,
    Overlay
};

/// What a combine folds: weight packets, or fields a resolver already
/// produced over TargetCount points.
enum class WeightCombineInputs : uint8_t { Packets, Fields };

struct CombineWeightsPayload {
    WeightCombineMode mode = WeightCombineMode::Multiply;
    WeightCombineInputs inputs = WeightCombineInputs::Packets;
    /// Packets: out-of-range results clamp (true) or fail the op (false).
    bool clamp = false;

    bool operator==(const CombineWeightsPayload &o) const
    {
        return mode == o.mode && inputs == o.inputs && clamp == o.clamp;
    }
};

const rigExec::RigExecOpHandle &SphereWeight();
const rigExec::RigExecOpHandle &PlaneWeight();
const rigExec::RigExecOpHandle &CurveWeight();

// Every op: Points (Vec3fArray, sampled), Placement (Mat4d, the volume's
// world placement; its scale and shear are removed), optional Strength (1),
// Invert (0), FalloffMin (0), FalloffMax (1) (Float) and FalloffSamples
// (FloatArray), Out (FloatArray, one per point). A non-finite weight, or
// one outside [0, 1] without clamp, is an error.
//
// Sphere and Curve: optional Scale (Vec3f, 1; finite and positive), which
// divides the local axes. Sphere: optional PositiveScale and NegativeScale
// (Vec3f, 1), selected per axis by the scaled coordinate's sign. Curve:
// Curve (Vec3fArray, non-empty, in the points' space). Plane: optional
// ExtentU and ExtentV (Float, 1; finite and positive when bounded).
#define RIGEXEC_WEIGHT_FIELD_PORTS(Op)                                         \
    const rigExec::RigExecPortHandle &Op##Points();                            \
    const rigExec::RigExecPortHandle &Op##Placement();                         \
    const rigExec::RigExecPortHandle &Op##Strength();                          \
    const rigExec::RigExecPortHandle &Op##Invert();                            \
    const rigExec::RigExecPortHandle &Op##FalloffMin();                        \
    const rigExec::RigExecPortHandle &Op##FalloffMax();                        \
    const rigExec::RigExecPortHandle &Op##FalloffSamples();                    \
    const rigExec::RigExecPortHandle &Op##Out();

RIGEXEC_WEIGHT_FIELD_PORTS(SphereWeight)
const rigExec::RigExecPortHandle &SphereWeightScale();
const rigExec::RigExecPortHandle &SphereWeightPositiveScale();
const rigExec::RigExecPortHandle &SphereWeightNegativeScale();

RIGEXEC_WEIGHT_FIELD_PORTS(PlaneWeight)
const rigExec::RigExecPortHandle &PlaneWeightExtentU();
const rigExec::RigExecPortHandle &PlaneWeightExtentV();
/// Optional UInt inputs that replace the payload's plane axis (0 x, 1 y, 2
/// z) and bounds (non-zero bounded) for one run.
const rigExec::RigExecPortHandle &PlaneWeightAxis();
const rigExec::RigExecPortHandle &PlaneWeightBounded();

RIGEXEC_WEIGHT_FIELD_PORTS(CurveWeight)
const rigExec::RigExecPortHandle &CurveWeightScale();
const rigExec::RigExecPortHandle &CurveWeightCurve();

#undef RIGEXEC_WEIGHT_FIELD_PORTS

/// Where a volume ended up: Out (Mat4d), the map from the origin and unit
/// axes to Frame (PointFrame), the identity for a frame that is not valid
/// or is degenerate. A non-finite landmark of a valid frame is an error.
const rigExec::RigExecOpHandle &VolumePlacement();
const rigExec::RigExecPortHandle &VolumePlacementFrame();
const rigExec::RigExecPortHandle &VolumePlacementOut();

/// The settings of a weight packet op: its packet's representation and
/// whether out-of-range weights clamp (true) or fail the op (false).
struct WeightPacketPayload {
    rigExecRig::WeightRepresentation representation =
        rigExecRig::WeightRepresentation::Constant;
    bool clamp = false;

    bool operator==(const WeightPacketPayload &o) const
    {
        return representation == o.representation && clamp == o.clamp;
    }
};

/// A painted packet: Values (FloatArray), at Indices (IntArray, optional)
/// for a sparse packet, which come out sorted, and DefaultWeight (Float, 0),
/// to Out (PointWeights). A sparse packet whose values and indices differ in
/// length or repeat an index, a dense one whose default is not 0, or a
/// weight outside the range policy is an error.
const rigExec::RigExecOpHandle &StaticWeights();
const rigExec::RigExecPortHandle &StaticWeightsValues();
const rigExec::RigExecPortHandle &StaticWeightsIndices();
const rigExec::RigExecPortHandle &StaticWeightsDefaultWeight();
const rigExec::RigExecPortHandle &StaticWeightsOut();

/// A remapped packet: each weight w of Base (PointWeights, optional) as
/// (w * Driver) * Scale + Bias (Float; 1, 1, 0) to Out (PointWeights).
/// Without a base the packet is constant, its weight Driver * Scale + Bias.
/// A base of another representation, a missing base for a packet that is
/// not constant, or a weight outside the range policy is an error.
const rigExec::RigExecOpHandle &DynamicWeights();
const rigExec::RigExecPortHandle &DynamicWeightsBase();
const rigExec::RigExecPortHandle &DynamicWeightsDriver();
const rigExec::RigExecPortHandle &DynamicWeightsScale();
const rigExec::RigExecPortHandle &DynamicWeightsBias();
const rigExec::RigExecPortHandle &DynamicWeightsOut();

enum class VolumeWeightShape : uint8_t { Sphere, Plane, Curve };

/// What a VolumeWeights op is configured with at compile.
struct VolumeWeightsPayload {
    VolumeWeightShape shape = VolumeWeightShape::Sphere;
    /// Out-of-range weights clamp (true) or fail the op (false).
    bool clamp = false;
    /// Plane: as WeightFieldPayload's.
    WeightFieldAxis planeAxis = WeightFieldAxis::Z;
    bool planeBounded = false;

    bool operator==(const VolumeWeightsPayload &o) const
    {
        return shape == o.shape && clamp == o.clamp &&
               planeAxis == o.planeAxis && planeBounded == o.planeBounded;
    }
};

/// A volume's dense packet over the target's Points (Vec3fArray), or over
/// Samples (Vec3fArray, optional) when they are not empty and as many: the
/// shape's field, in the rigid local space of the volume placed by Frame
/// (PointFrame), remapped through Strength, Invert, FalloffMin and
/// FalloffMax (Float; 1, 0, 0, 1) and FalloffSamples (FloatArray, optional;
/// fewer than two is linear), to Out (PointWeights). Sphere and Curve:
/// Scale (Vec3f, 1) divides the local axes; Sphere: PositiveScale and
/// NegativeScale (Vec3f, 1) by the coordinate's sign; Curve: Curve
/// (Vec3fArray, in the points' space); Plane: ExtentU and ExtentV (Float,
/// 1) when bounded. A frame that places nothing invertible, no points,
/// samples of another count, a scale or extent that is not finite and
/// positive, an empty curve, or a weight outside the range policy is an
/// error.
const rigExec::RigExecOpHandle &VolumeWeights();
const rigExec::RigExecPortHandle &VolumeWeightsFrame();
const rigExec::RigExecPortHandle &VolumeWeightsPoints();
const rigExec::RigExecPortHandle &VolumeWeightsSamples();
const rigExec::RigExecPortHandle &VolumeWeightsCurve();
const rigExec::RigExecPortHandle &VolumeWeightsStrength();
const rigExec::RigExecPortHandle &VolumeWeightsInvert();
const rigExec::RigExecPortHandle &VolumeWeightsFalloffMin();
const rigExec::RigExecPortHandle &VolumeWeightsFalloffMax();
const rigExec::RigExecPortHandle &VolumeWeightsFalloffSamples();
const rigExec::RigExecPortHandle &VolumeWeightsScale();
const rigExec::RigExecPortHandle &VolumeWeightsPositiveScale();
const rigExec::RigExecPortHandle &VolumeWeightsNegativeScale();
const rigExec::RigExecPortHandle &VolumeWeightsExtentU();
const rigExec::RigExecPortHandle &VolumeWeightsExtentV();
const rigExec::RigExecPortHandle &VolumeWeightsOut();

/// Folds Inputs (PointWeights list, optional) in feed order, then applies
/// optional Invert (Float, 0) and Strength (Float, 1) to Out (PointWeights,
/// dense). Subtract and Overlay seed from the first input; Average divides
/// by the input count; no inputs is the mode's identity.
///
/// Packets: the packet combine. No input and no TargetCount fails compile
/// or the evaluation that binds them. An invalid input, dense inputs of
/// different lengths, a negative resolved weight, no element count (a dense
/// input's length, else TargetCount (UInt, 0)) or a result outside the range
/// policy fails the op.
/// Fields: every input is a dense field of TargetCount weights, folded as
/// it is; one of another length fails the op. No range policy applies.
const rigExec::RigExecOpHandle &CombineWeights();
const rigExec::RigExecPortHandle &CombineWeightsInputs();
const rigExec::RigExecPortHandle &CombineWeightsTargetCount();
const rigExec::RigExecPortHandle &CombineWeightsStrength();
const rigExec::RigExecPortHandle &CombineWeightsInvert();
const rigExec::RigExecPortHandle &CombineWeightsOut();

/// A weight object's field over TargetCount (UInt) points, one weight per
/// point to Out (FloatArray), its range policy applied last: a weight that
/// is not finite, or outside [0, 1] without clamp, is an error.
///
/// StaticWeightField: the painted Values (FloatArray) as the payload's
/// representation lays them out: a constant field is DefaultWeight (Float,
/// 0) everywhere and paints no values; a dense one paints every point and
/// its default is 0; a sparse one paints Values at Indices (IntArray,
/// optional) over the default. Painting outside that layout, an index out
/// of range or repeated, is an error. The payload is a
/// WeightPacketPayload.
const rigExec::RigExecOpHandle &StaticWeightField();
const rigExec::RigExecPortHandle &StaticWeightFieldValues();
const rigExec::RigExecPortHandle &StaticWeightFieldIndices();
const rigExec::RigExecPortHandle &StaticWeightFieldDefaultWeight();
const rigExec::RigExecPortHandle &StaticWeightFieldTargetCount();
const rigExec::RigExecPortHandle &StaticWeightFieldOut();

/// DynamicWeightField: each weight w of Base (FloatArray, optional; 1
/// everywhere without it) as (w * Driver) * Scale + Bias (Float; 1, 1, 0).
/// The payload is a WeightPacketPayload; only its clamp is read. Non-empty
/// Indices (IntArray, optional) must name the same set as BaseIndices.
const rigExec::RigExecOpHandle &DynamicWeightField();
const rigExec::RigExecPortHandle &DynamicWeightFieldBase();
const rigExec::RigExecPortHandle &DynamicWeightFieldDriver();
const rigExec::RigExecPortHandle &DynamicWeightFieldScale();
const rigExec::RigExecPortHandle &DynamicWeightFieldBias();
const rigExec::RigExecPortHandle &DynamicWeightFieldTargetCount();
const rigExec::RigExecPortHandle &DynamicWeightFieldOut();
const rigExec::RigExecPortHandle &DynamicWeightFieldIndices();
const rigExec::RigExecPortHandle &DynamicWeightFieldBaseIndices();

/// What a CombineWeightFields op is configured with at compile.
struct CombineWeightFieldsPayload {
    WeightCombineMode mode = WeightCombineMode::Multiply;
    /// Out-of-range weights clamp (true) or fail the op (false).
    bool clamp = false;

    bool operator==(const CombineWeightFieldsPayload &o) const
    {
        return mode == o.mode && clamp == o.clamp;
    }
};

/// CombineWeightFields: Inputs (FloatArray list, optional), each a field of
/// TargetCount weights, folded in feed order as CombineWeights folds, then
/// Invert (Float, 0) and Strength (Float, 1). An input of another length is
/// an error.
const rigExec::RigExecOpHandle &CombineWeightFields();
const rigExec::RigExecPortHandle &CombineWeightFieldsInputs();
const rigExec::RigExecPortHandle &CombineWeightFieldsTargetCount();
const rigExec::RigExecPortHandle &CombineWeightFieldsStrength();
const rigExec::RigExecPortHandle &CombineWeightFieldsInvert();
const rigExec::RigExecPortHandle &CombineWeightFieldsOut();

} // namespace rigExecOps

#endif
