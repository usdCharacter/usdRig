#ifndef RIGEXEC_OPS_POSE_POSE_OPS_H
#define RIGEXEC_OPS_POSE_POSE_OPS_H

/// The pose inputs a rig composes from: bound avar values, a space switch's
/// checkpoint frame, a pose interpolator's weights, and one slot's rest,
/// default ladder and first pose, switched or shifted. The kernels are
/// rigExecMath/avarScale.h, rigExecEvalCore/pose.h's SpaceCheckpointMatrix and
/// rigExecMath/poseArithmeticKernel.h.

#include "rigExecOps/constraint/matchOps.h"
#include "rigExecOps/library.h"
#include "rigExecStdTypes/stdTypes.h"

#include <cstdint>
#include <vector>

namespace rigExecOps {

struct AvarInputsPayload {
    /// The avars written, one per element of Out.
    uint64_t avarCount = 0;
    /// Per element of Values, the element of Out it writes.
    std::vector<uint32_t> targets;

    bool operator==(const AvarInputsPayload &o) const
    {
        return avarCount == o.avarCount && targets == o.targets;
    }
};

const rigExec::RigExecOpHandle &AvarInputs();

// Values (DoubleArray, one per binding, in binding order). Out (DoubleArray,
// avarCount): each avar's value, a later binding for the same avar winning.
const rigExec::RigExecPortHandle &AvarInputsValues();
const rigExec::RigExecPortHandle &AvarInputsOut();

struct SpaceCheckpointPayload {
    /// Per recomposed slot, outermost first: whether its scale avars are
    /// ignored.
    std::vector<uint8_t> noScale;

    bool operator==(const SpaceCheckpointPayload &o) const
    {
        return noScale == o.noScale;
    }
};

/// A rotation order's code as the pose ops read it: its three axis letters
/// in the low three bytes, or 0 (the default order) unless it has three
/// letters.
uint32_t SpaceCheckpointRotOrderCode(const char *text, size_t size);

const rigExec::RigExecOpHandle &SpaceCheckpoint();

// Per recomposed slot, outermost first: Avars (DoubleArray, 11 each),
// RotOrders (UIntArray, SpaceCheckpointRotOrderCode), RotationSigns
// (UIntArray, RigExecRotationSignMask), PosedDefault, ParentDefaultInverse, ParentExpression and
// PosedAuthoredMatrix (Mat4dArray), ParentExpressionAuthored and
// PosedAuthored (BoolArray). Optional Anchor (Mat4d, the posed matrix the
// outermost slot composes under; identity when unfed). Out (Mat4d): the
// switch frame, NaN in [3][0] where a slot's frame is unusable.
const rigExec::RigExecPortHandle &SpaceCheckpointAvars();
const rigExec::RigExecPortHandle &SpaceCheckpointRotOrders();
const rigExec::RigExecPortHandle &SpaceCheckpointRotationSigns();
const rigExec::RigExecPortHandle &SpaceCheckpointPosedDefault();
const rigExec::RigExecPortHandle &SpaceCheckpointParentDefaultInverse();
const rigExec::RigExecPortHandle &SpaceCheckpointParentExpression();
const rigExec::RigExecPortHandle &SpaceCheckpointParentExpressionAuthored();
const rigExec::RigExecPortHandle &SpaceCheckpointPosedAuthored();
const rigExec::RigExecPortHandle &SpaceCheckpointPosedAuthoredMatrix();
const rigExec::RigExecPortHandle &SpaceCheckpointAnchor();
const rigExec::RigExecPortHandle &SpaceCheckpointOut();

/// The solve a pose interpolator runs (rigExecMath/poseArithmeticKernel.h):
/// a solved RBF table, RigLogic's RBF method, an ML network, or animated
/// maps.
enum class PoseSolve : uint32_t { Rbf, RbfMethod, Network, Maps };

struct PoseInterpolatorPayload {
    PoseSolve solve = PoseSolve::Rbf;
    /// Elements of Out: the weights the solve produces, in its pose order.
    uint64_t poseCount = 0;
    /// Rbf: also measure the driver's translation.
    bool enableTranslation = false;
    bool allowNegativeWeights = true;
    /// RbfMethod and Network: per extra frame driver, whether it is measured
    /// against a parent frame.
    std::vector<uint8_t> extraParents;

    /// Rbf: the solved table (rigExecRuntime/rbfSolver.h), weights row-major
    /// with one row per weight and one column per pose.
    std::vector<rigExecStd::Vec3d> rbfPoses;
    std::vector<rigExecStd::Vec3d> rbfTranslations;
    std::vector<uint32_t> rbfPoseTypes;
    rigExecStd::Vec3d rbfTwistAxis{0.0, 1.0, 0.0};
    uint32_t rbfKernel = 0;
    double rbfRadius = 0.0;
    double rbfTranslationRadius = 0.0;
    std::vector<double> rbfRadii;
    std::vector<double> rbfTranslationRadii;
    bool rbfNormalize = true;
    bool rbfEnableRotation = true;
    bool rbfEnableTranslation = false;
    uint64_t rbfWeightRows = 0;
    std::vector<double> rbfWeights;

    /// RbfMethod: RigExecRbfMethodTable, enums as their values.
    uint32_t methodSolverType = 0;
    uint32_t methodDistance = 0;
    uint32_t methodWeightFunction = 0;
    uint32_t methodNormalize = 0;
    uint32_t methodTwistAxis = 0;
    uint64_t methodRawCount = 0;
    uint64_t methodTargetCount = 0;
    double methodRadius = 0.0;
    double methodWeightThreshold = 0.0;
    std::vector<double> methodTargets;
    std::vector<double> methodDistanceTargets;
    std::vector<double> methodTargetScales;
    std::vector<double> methodCoefficients;

    /// Network: RigExecMlBuildNetwork's arguments and the input and output
    /// tables.
    uint64_t networkInputCount = 0;
    std::vector<int32_t> networkLayerSizes;
    std::vector<uint32_t> networkActivations;
    std::vector<float> networkAlphas;
    std::vector<float> networkWeights;
    std::vector<float> networkBiases;
    std::vector<float> networkInputMean;
    std::vector<float> networkInputScale;
    std::vector<float> networkDefaults;

    /// Maps: every map's rows, map after map; mapBegin holds the map count
    /// plus one offsets into them.
    uint64_t mapsInputCount = 0;
    std::vector<int32_t> mapRowInputs;
    std::vector<double> mapRowFrom;
    std::vector<double> mapRowTo;
    std::vector<double> mapRowSlope;
    std::vector<double> mapRowCut;
    std::vector<uint64_t> mapBegin;
    /// Empty, or one per map: 1 where its sum is not clamped to [0, 1].
    std::vector<uint8_t> mapUnclamped;

    bool operator==(const PoseInterpolatorPayload &o) const;
};

const rigExec::RigExecOpHandle &PoseInterpolator();

// Enabled (Bool; unfed = on), Numeric (DoubleArray: the dials a numeric
// driver reads, or an ML row's values after its frames), Mask (Double; unfed
// = 1), DriverFinal / DriverRest and ParentFinal / ParentRest (PointFrame,
// fed in pairs), ExtraFinals / ExtraRests (PointFrameList, one per extra
// driver) and ExtraParentFinals / ExtraParentRests (one per extra driver
// with a parent). Out (FloatArray, poseCount): zero when disabled.
const rigExec::RigExecPortHandle &PoseInterpolatorEnabled();
const rigExec::RigExecPortHandle &PoseInterpolatorNumeric();
const rigExec::RigExecPortHandle &PoseInterpolatorMask();
const rigExec::RigExecPortHandle &PoseInterpolatorDriverFinal();
const rigExec::RigExecPortHandle &PoseInterpolatorDriverRest();
const rigExec::RigExecPortHandle &PoseInterpolatorParentFinal();
const rigExec::RigExecPortHandle &PoseInterpolatorParentRest();
const rigExec::RigExecPortHandle &PoseInterpolatorExtraFinals();
const rigExec::RigExecPortHandle &PoseInterpolatorExtraRests();
const rigExec::RigExecPortHandle &PoseInterpolatorExtraParentFinals();
const rigExec::RigExecPortHandle &PoseInterpolatorExtraParentRests();
const rigExec::RigExecPortHandle &PoseInterpolatorOut();

/// RestCompose's and LadderCompose's payload.
struct SlotActivityPayload {
    /// Whether the slot's provider is active.
    bool active = true;

    bool operator==(const SlotActivityPayload &o) const
    {
        return active == o.active;
    }
};

const rigExec::RigExecOpHandle &RestCompose();

// One slot's rest: RestAvars (DoubleArray, 6: translate then rotate),
// RestSpace (Mat4d), optional Intervening (Mat4d; identity when unfed) and
// ParentRest (Mat4d, the parent's RoundTrip; identity when unfed).
// RestMatrix (Mat4d), Rest (PointFrame) and RoundTrip (Mat4d); an inactive
// slot gives the unavailable space and a flagless identity frame.
const rigExec::RigExecPortHandle &RestComposeRestAvars();
const rigExec::RigExecPortHandle &RestComposeRestSpace();
const rigExec::RigExecPortHandle &RestComposeIntervening();
const rigExec::RigExecPortHandle &RestComposeParentRest();
const rigExec::RigExecPortHandle &RestComposeRestMatrix();
const rigExec::RigExecPortHandle &RestComposeRest();
const rigExec::RigExecPortHandle &RestComposeRoundTrip();

const rigExec::RigExecOpHandle &LadderCompose();

// One slot's default ladder: Spaces and Authored (Mat4dArray and BoolArray,
// six each, in channel order default, posed, parent, parentDefault,
// avarDefault, posedDefault: each channel's provider value or leaf, and
// whether it is authoritative, a default channel also counting a ready
// provider value), ParentSpaceLeaf (Mat4d), RotationSign (Vec3d),
// DefaultAvars (DoubleArray, 6), OwnRest (Mat4d, this slot's rest
// RoundTrip), and for a slot with a parent ParentRest and ParentDefault
// (Mat4d, the parent's rest and default round trips, fed together).
// SelfDefault, ParentDefaultInverse, DefaultRoundTrip, PosedDefault,
// ParentSpace and PosedAuthoredMatrix (Mat4d), PosedAuthored and
// ParentSpaceAuthored (Bool), OutRotationSign (UInt mask). An inactive slot
// gives the unavailable space and clear flags.
const rigExec::RigExecPortHandle &LadderComposeSpaces();
const rigExec::RigExecPortHandle &LadderComposeAuthored();
const rigExec::RigExecPortHandle &LadderComposeParentSpaceLeaf();
const rigExec::RigExecPortHandle &LadderComposeRotationSign();
const rigExec::RigExecPortHandle &LadderComposeDefaultAvars();
const rigExec::RigExecPortHandle &LadderComposeOwnRest();
const rigExec::RigExecPortHandle &LadderComposeParentRest();
const rigExec::RigExecPortHandle &LadderComposeParentDefault();
const rigExec::RigExecPortHandle &LadderComposeSelfDefault();
const rigExec::RigExecPortHandle &LadderComposeParentDefaultInverse();
const rigExec::RigExecPortHandle &LadderComposeDefaultRoundTrip();
const rigExec::RigExecPortHandle &LadderComposePosedDefault();
const rigExec::RigExecPortHandle &LadderComposeParentSpace();
const rigExec::RigExecPortHandle &LadderComposePosedAuthoredMatrix();
const rigExec::RigExecPortHandle &LadderComposePosedAuthored();
const rigExec::RigExecPortHandle &LadderComposeParentSpaceAuthored();
const rigExec::RigExecPortHandle &LadderComposeOutRotationSign();

struct ComposeSlotPayload {
    /// Whether the slot's provider is active.
    bool active = true;
    /// Whether its scale avars are ignored.
    bool noScale = false;

    bool operator==(const ComposeSlotPayload &o) const
    {
        return active == o.active && noScale == o.noScale;
    }
};

const rigExec::RigExecOpHandle &ComposeSlot();

// One unswitched slot's first pose: Avars (DoubleArray, 11), optional
// RotOrder (UInt, SpaceCheckpointRotOrderCode; 0 when unfed), RotationSign
// (UInt mask), PosedAuthored (Bool), PosedAuthoredMatrix, PosedDefault,
// ParentDefaultInverse and ParentSpace (Mat4d), ParentSpaceAuthored (Bool),
// optional ParentPosed (Mat4d, the parent's PosedMatrix; identity when
// unfed). Base (PointFrame) and PosedMatrix (Mat4d, the space of Base).
// An inactive slot gives a flagless identity frame and the unavailable
// space.
const rigExec::RigExecPortHandle &ComposeSlotAvars();
const rigExec::RigExecPortHandle &ComposeSlotRotOrder();
const rigExec::RigExecPortHandle &ComposeSlotRotationSign();
const rigExec::RigExecPortHandle &ComposeSlotPosedAuthored();
const rigExec::RigExecPortHandle &ComposeSlotPosedAuthoredMatrix();
const rigExec::RigExecPortHandle &ComposeSlotPosedDefault();
const rigExec::RigExecPortHandle &ComposeSlotParentDefaultInverse();
const rigExec::RigExecPortHandle &ComposeSlotParentSpace();
const rigExec::RigExecPortHandle &ComposeSlotParentSpaceAuthored();
const rigExec::RigExecPortHandle &ComposeSlotParentPosed();
const rigExec::RigExecPortHandle &ComposeSlotBase();
const rigExec::RigExecPortHandle &ComposeSlotPosedMatrix();

struct SwitchedComposeSlotPayload {
    /// Whether the slot's provider is active.
    bool active = true;
    /// Whether its scale avars are ignored.
    bool noScale = false;
    SpaceSwitchSetup space;

    bool operator==(const SwitchedComposeSlotPayload &o) const
    {
        return active == o.active && noScale == o.noScale && space == o.space;
    }
};

const rigExec::RigExecOpHandle &SwitchedComposeSlot();

// One space-switched slot's first pose: Avars (DoubleArray, 11), optional
// RotOrder (UInt; 0 when unfed), RotationSign (UInt mask), PosedAuthored
// (Bool; the frame of PosedAuthoredMatrix when set), PosedAuthoredMatrix,
// PosedDefault and ParentDefaultInverse (Mat4d), optional ParentPosed (the
// posed space the switch composes under) and ParentDefault (the parent's
// default round trip), both identity when unfed, Active (Double, the
// selected space, a label as its index), SpaceDefault and SpacePosed (Mat4d,
// fed with a carry space), SourceDefaults and SourcePosed (Mat4dArray, one
// per space; a world space's entries are unread). Base (PointFrame) and
// PosedMatrix (Mat4d, the space of Base); an inactive slot gives a flagless
// identity frame and the unavailable space.
const rigExec::RigExecPortHandle &SwitchedComposeSlotAvars();
const rigExec::RigExecPortHandle &SwitchedComposeSlotRotOrder();
const rigExec::RigExecPortHandle &SwitchedComposeSlotRotationSign();
const rigExec::RigExecPortHandle &SwitchedComposeSlotPosedAuthored();
const rigExec::RigExecPortHandle &SwitchedComposeSlotPosedAuthoredMatrix();
const rigExec::RigExecPortHandle &SwitchedComposeSlotPosedDefault();
const rigExec::RigExecPortHandle &SwitchedComposeSlotParentDefaultInverse();
const rigExec::RigExecPortHandle &SwitchedComposeSlotParentPosed();
const rigExec::RigExecPortHandle &SwitchedComposeSlotParentDefault();
const rigExec::RigExecPortHandle &SwitchedComposeSlotActive();
const rigExec::RigExecPortHandle &SwitchedComposeSlotSpaceDefault();
const rigExec::RigExecPortHandle &SwitchedComposeSlotSpacePosed();
const rigExec::RigExecPortHandle &SwitchedComposeSlotSourceDefaults();
const rigExec::RigExecPortHandle &SwitchedComposeSlotSourcePosed();
const rigExec::RigExecPortHandle &SwitchedComposeSlotBase();
const rigExec::RigExecPortHandle &SwitchedComposeSlotPosedMatrix();

const rigExec::RigExecOpHandle &ShiftedComposeSlot();

// A composed slot after an auto clavicle shifts it: Base (PointFrame, the
// composed frame), Delta (Vec3d, the clavicle's shift) and Shifted (Mat4d,
// the shifted frame's matrix). Out (PointFrame: Base when Delta is zero,
// else the frame of Shifted) and PosedMatrix (Mat4d, the space of Out).
const rigExec::RigExecPortHandle &ShiftedComposeSlotBase();
const rigExec::RigExecPortHandle &ShiftedComposeSlotDelta();
const rigExec::RigExecPortHandle &ShiftedComposeSlotShifted();
const rigExec::RigExecPortHandle &ShiftedComposeSlotOut();
const rigExec::RigExecPortHandle &ShiftedComposeSlotPosedMatrix();

} // namespace rigExecOps

#endif
