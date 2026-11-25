#ifndef RIGEXEC_OPS_WEIGHT_WEIGHT_OPS_H
#define RIGEXEC_OPS_WEIGHT_WEIGHT_OPS_H

/// The volume weight producers: sphere, plane and curve fields over a target's
/// points, in the volume's rigid local space, and the ordered combine of
/// weights. The field kernel is rigExecMath/weightFieldKernel.h and the combine
/// rigExecEvalCore/weights.h.

#include "rigExecOps/library.h"

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

} // namespace rigExecOps

#endif
