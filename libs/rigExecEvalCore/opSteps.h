// Op nodes as steps of the common schedule: one OpNode step per node that
// does not run in ranges, and per range chain one OpChainHead step and one
// OpChainRange step per (segment, vertex group), the groups those of the
// point chain whose points the chain reads, so an edit reruns only the
// groups it reaches. Header-only and free of OpenUSD: the baked program,
// its frozen clones and the runtime run one definition.
#ifndef RIGEXEC_EVAL_CORE_OP_STEPS_H
#define RIGEXEC_EVAL_CORE_OP_STEPS_H

#include "rigExec/api/registry.h"
#include "rigExec/api/runState.h"
#include "rigExec/api/view.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {
namespace eval {

/// Where an op input the host writes reads its value from.
enum class OpHostSource : uint8_t {
    /// A point chain's final points, vertex group by vertex group. It feeds
    /// pointwise inputs of one range chain only.
    ChainPoints = 0,
    /// A joint's or control's final matrix.
    FinalMatrix = 1,
    /// A joint's or control's base matrix.
    BaseMatrix = 2,
    /// A value the host looks up before the node runs: \p object indexes
    /// the layout's lookups. Which source answers may change per frame.
    Lookup = 4,
};

/// OpLookup::source: the family of the rig's values a lookup reads. The
/// numbers are part of the file format.
enum class OpLookupSource : uint8_t {
    Property = 0,
    PoseVersion = 1,
    WeightArrays = 2,
    /// The points entering revision `part` of point chain `object`, as one
    /// array.
    ChainVersion = 3,
    /// An input of weight object `part` as weight field `object` reads it.
    WeightFieldInputs = 4,
    /// An array of `part` elements, element i what lookup `object` + i
    /// answers.
    List = 5,
    /// Mat4d: the final (part 0) or base (part 1) matrix of slot `object`.
    Matrix = 6,
    /// Mat4d: frame record `object`, present when its writer found it
    /// valid this frame.
    FrameRecord = 7,
    /// Mat4d: the delta geometry-domain constraint `object` measured,
    /// present when it measured one this frame.
    ConstraintDelta = 8,
    /// Mat4d: the first present of the `part` lookups from `object`, each
    /// a FrameRecord, ConstraintDelta or (always present) Matrix lookup.
    Selected = 9,
    /// Revision `object`'s own input `read` (OpRevisionRead).
    RevisionLeaf = 10,
    /// PointFrame: the rest frame of slot `object`.
    RestFrame = 20,
    /// The record gates of pose commit `object`: `read` 0 its per-output
    /// presence (BoolArray), 1 whether it records after itself and 2
    /// whether it records every target (Bool); 3 whether it committed output
    /// `part` this run (Bool).
    CommitRecordGate = 21,
    /// PointFrame: provider refresh `object`'s re-evaluated base (`read` 0)
    /// or current (1) frame; a value missing this frame fails the lookup.
    ProviderFrame = 22,
    /// Mat4d: the raw parent space of slot `object`, the identity where
    /// its leaf is blocked.
    ProviderParentRaw = 23,
    /// A typed input list of provider operation `object`, filled from the
    /// provider values its inputs name; `read` is an
    /// eval::spaces::ProviderRead.
    ProviderInputs = 24,
    /// BoolArray: one flag per candidate that can answer provider value
    /// producer `object` (eval::spaces::ProviderProducers numbering) this
    /// frame; `read` is an eval::spaces::ProviderCandidateRead.
    ProviderCandidates = 25,
    /// PointFrameList: the pose frames at the versions list `part`
    /// (OpCommitList) of commit `object` names.
    CommitVersions = 30,
    /// Commit `object`'s candidates: `part` 0 their frames
    /// (PointFrameList), 1 which are present (BoolArray). A constraint's
    /// come from its commit table, a solver batch's from its solvers'
    /// outputs, the last in batch order winning a position.
    CommitCandidates = 31,
    /// The sources of the constraint commit `object` commits: `part` -1
    /// all of them (PointFrameArray), else source `part`'s frame (`read`
    /// 0) or rest frame (`read` 1). A source that maps to no transform
    /// comes flagged degenerate; one that does not resolve is an error
    /// while the constraint is on and the source weighs anything.
    ConstraintSources = 32,
    /// The per-source tables of constraint commit `object` (OpConstraintRead
    /// SourceWeights, TranslationOffsets or RotationOffsets).
    ConstraintArrays = 33,
    /// An input of constraint commit `object`'s mover (OpConstraintRead).
    ConstraintLeaf = 34,
    /// Constraint commit `object`'s envelope (OpConstraintRead
    /// SolveWeight, DeltaWeight or Present), decided from its enabled flag
    /// and weight alone.
    ConstraintEnvelope = 35,
    /// Mat4d: the rigExec:space carry of constraint commit `object`.
    ConstraintSpace = 36,
    /// Vec3d: the world-up direction constraint commit `object`'s aim
    /// reads; zero where the aim does not read it or its object maps to no
    /// transform.
    ConstraintWorldUp = 37,
    /// An input of harmonics constraint commit `object` (OpHarmonicsRead).
    HarmonicsInput = 38,
    /// An input of single-chain IK constraint commit `object`
    /// (OpSingleChainIkRead).
    SingleChainIkInput = 70,
    /// An input of IK/FK match constraint commit `object`
    /// (OpIkFkMatchRead).
    IkFkMatchInput = 71,
    /// The authored transform of constraint commit `object`'s geometry
    /// target: `read` 0 the Mat4d, 1 whether it was read (Bool).
    ConstraintDeltaBase = 39,
    /// DoubleArray: the values of provider slot `object`'s avar bindings,
    /// its varying ones then its constant ones, each in program order.
    AvarBindings = 40,
    /// Space checkpoint `object`'s recomposed slots, outermost first, as
    /// SpaceCheckpoint input `read` (OpCheckpointRead) reads them; Anchor
    /// its anchor's posed matrix.
    CheckpointInputs = 41,
    /// Pose interpolator `object`'s input `read` (OpInterpolatorRead):
    /// its leaves, or its drivers' last fin versions and rest frames.
    InterpolatorInputs = 42,
    /// Solver `object`'s input `part` (OpSolverRead), from the pose
    /// versions it reads and the rests composed this run.
    SolverInput = 60,
};

/// OpLookup::read of a CheckpointInputs lookup: the SpaceCheckpoint input
/// it answers.
enum class OpCheckpointRead : uint8_t {
    Avars,
    RotOrders,
    RotationSigns,
    PosedDefault,
    ParentDefaultInverse,
    ParentExpression,
    ParentExpressionAuthored,
    PosedAuthored,
    PosedAuthoredMatrix,
    Anchor,
};

/// OpLookup::read of an InterpolatorInputs lookup: the PoseInterpolator
/// input it answers.
enum class OpInterpolatorRead : uint8_t {
    Enabled,
    Numeric,
    Mask,
    DriverFinal,
    DriverRest,
    ParentFinal,
    ParentRest,
    ExtraFinals,
    ExtraRests,
    ExtraParentFinals,
    ExtraParentRests,
};

/// OpLookup::part of a SolverInput lookup. The numbers are part of the
/// file format.
enum class OpSolverRead : uint8_t {
    /// PointFrameArray: an FK chain's controls at their read versions with
    /// their rests, its start frame first when it has one.
    FkControls = 0,
    /// PointLandmarksList: per FK element, the rest its frame is re-based
    /// to: a live joint rest where one stands, else the control's rest.
    FkOutRests = 1,
    /// PointLandmarks: the rest of the solver's start frame.
    StartRest = 2,
    /// DoubleArray: the joint group selectors.
    GroupSelectors = 3,
    /// PointLandmarksList: a two-bone IK's three joint rests, a live one
    /// where a pose step below the solver wrote it.
    IkRests = 4,
    /// Mat4d: rigExec:spaceMatrix, after rigExec:space's rest-to-pose map
    /// where it names a space.
    IkSpace = 5,
    /// A two-bone IK's scalar inputs, each of its port's type.
    Bend = 6,
    UpperOffset = 7,
    LowerOffset = 8,
    Stretch = 9,
    Softness = 10,
    Pin = 11,
    UpperScale = 12,
    LowerScale = 13,
    SoftDistance = 14,
    Twist = 15,
    /// PointLandmarksList: per joint, the rest a pose step below the
    /// solver wrote where JointRestLive marks it, else its slot's rest.
    JointRests = 16,
    /// BoolArray: per joint, whether a pose step below the solver wrote
    /// its rest.
    JointRestLive = 17,
    /// PointLandmarks: a twist distribution's start and end rests.
    TwistStartRest = 18,
    TwistEndRest = 19,
    /// Double: a twist distribution's inputs:twistTurns.
    TwistTurns = 20,
};

/// OpLookup::read of a constraint lookup. The numbers are part of the file
/// format.
enum class OpConstraintRead : uint8_t {
    /// Vec3d inputs.
    Offset = 0,
    AimVector = 1,
    UpVector = 2,
    RotationOffset = 3,
    WorldUpVector = 4,
    /// Bool inputs: the affect group, then Parent's nine axes.
    AffectX = 5,
    AffectY = 6,
    AffectZ = 7,
    TranslateX = 8,
    TranslateY = 9,
    TranslateZ = 10,
    RotateX = 11,
    RotateY = 12,
    RotateZ = 13,
    ScaleX = 14,
    ScaleY = 15,
    ScaleZ = 16,
    /// DoubleArray / Vec3dArray: one entry per source.
    SourceWeights = 20,
    TranslationOffsets = 21,
    RotationOffsets = 22,
    /// Double: the weight the kernel solves at, 0 while the constraint is
    /// off; a geometry target's delta weight; BoolArray: which commit
    /// candidates are present.
    SolveWeight = 30,
    DeltaWeight = 31,
    Present = 32,
};

/// OpLookup::read of a HarmonicsInput lookup. The numbers are part of the
/// file format.
enum class OpHarmonicsRead : uint8_t {
    /// Mat4d: the posed matrix of its rigExec:space.
    Space = 0,
    /// Double inputs.
    Amplitude = 1,
    Decay = 2,
    Termination = 3,
    /// Vec3d: inputs:axisAmplitude.
    AxisAmplitude = 4,
    /// Bool: whether the constraint is on and weighs anything.
    Open = 5,
};

/// OpLookup::read of a SingleChainIkInput lookup. The numbers are part of
/// the file format.
enum class OpSingleChainIkRead : uint8_t {
    /// PointFrameArray: the chain's joints as the commit reads them, root
    /// first; and the rests the solve lays them out at.
    Chain = 0,
    Rests = 1,
    /// PointFrame: the effector, its revised ancestors' motion applied.
    Effector = 2,
    /// Vec3d: the pole point, its pole vector or the weighted mean of its
    /// pole-vector objects; Double: its twist (degrees) and stretch.
    Pole = 3,
    Twist = 4,
    Stretch = 5,
};

/// OpLookup::read of an IkFkMatchInput lookup, one per IkFkMatch input
/// port. The numbers are part of the file format.
enum class OpIkFkMatchRead : uint8_t {
    /// Bool: whether every frame and rest of the limb maps to a transform.
    Readable = 0,
    /// Double: inputs:defaultWeight as authored.
    Weight = 1,
    /// Double: inputs:switch, stretch, softDistance, upperScale, lowerScale,
    /// pin and twist.
    Switch = 2,
    Stretch = 3,
    SoftDistance = 4,
    UpperScale = 5,
    LowerScale = 6,
    Pin = 7,
    Twist = 8,
    /// Mat4dArray: the ten limb frames and the eight rests; DoubleArray: the
    /// five controls' channels.
    Sources = 9,
    Rests = 10,
    Channels = 11,
    /// Bool and Mat4d: the IK solve's rigExec:space.
    HasSpace = 12,
    SpaceRest = 13,
    SpacePosed = 14,
    /// UInt: how the pole's parent follows the IK control.
    PoleMode = 15,
    /// The inputs the pole's space switch composed with this frame.
    PolePosedDefault = 16,
    PoleParentDefaultInverse = 17,
    PoleParentPosed = 18,
    PoleParentDefault = 19,
    PoleSpaceDefault = 20,
    PoleSpacePosed = 21,
    PoleSourceDefaults = 22,
    PoleSourcePosed = 23,
    PoleActive = 24,
};

/// OpLookup::part of a ConstraintWorldUp lookup: where the aim's world-up
/// comes from. The numbers are part of the file format.
enum class OpConstraintWorldUp : uint8_t {
    None = 0,
    /// The scene's up axis.
    Scene = 1,
    /// inputs:worldUpVector.
    Vector = 2,
    /// Toward the world-up object's origin from the target's.
    Object = 3,
    /// inputs:worldUpVector turned by the world-up object's rotation.
    ObjectRotation = 4,
};

/// Whether a lookup of \p source is one the host answers through
/// LookUpConstraint.
constexpr bool
OpConstraintLookupSource(uint8_t source)
{
    return (source >= uint8_t(OpLookupSource::ConstraintSources) &&
            source <= uint8_t(OpLookupSource::HarmonicsInput)) ||
           source == uint8_t(OpLookupSource::ConstraintDeltaBase) ||
           source == uint8_t(OpLookupSource::SingleChainIkInput) ||
           source == uint8_t(OpLookupSource::IkFkMatchInput);
}

/// OpLookup::part of a CommitVersions lookup: the commit's version list.
/// The numbers are part of the file format.
enum class OpCommitList : uint8_t {
    Entering = 0,
    Carry = 1,
    BaseCarry = 2,
    Current = 3,
    Closest = 4,
    DescendantCarry = 5,
    DescendantBaseCarry = 6,
};

/// OpLookup::read of a RevisionLeaf lookup. The numbers are part of the
/// file format.
enum class OpRevisionRead : uint8_t {
    /// Bool: inputs:enabled, true when the mover reads none.
    Enabled = 0,
    /// Float: inputs:defaultWeight, 1 when the mover reads none.
    DefaultWeight = 1,
    /// Bool: whether rigExec:weightBlend is radial.
    Radial = 2,
    /// A delta mush's inputs: inputs:restPoints (Vec3fArray, empty when
    /// it holds none), inputs:step, inputs:distanceWeight and
    /// inputs:displacement (Float), inputs:pinBorders and inputs:onlySmooth
    /// (Bool), inputs:smoothing and inputs:frameTransport (Int: the
    /// RigExecDeltaMushSmoothing or RigExecDeltaMushFrameTransport value, -1
    /// for a token that names none), inputs:smoothWeights (FloatArray) and
    /// inputs:computationToTarget (Mat4d).
    RestPoints = 3,
    MushStep = 4,
    MushPinBorders = 5,
    MushDistanceWeight = 6,
    MushDisplacement = 7,
    MushSmoothing = 8,
    MushFrameTransport = 9,
    MushSmoothWeights = 10,
    MushOnlySmooth = 11,
    MushComputationToTarget = 12,
    /// Vec3fArray: the points entering the revision's point chain.
    ChainBase = 13,
    /// A lattice's cage (Vec3fArray): as authored at the default time, and
    /// at the evaluated time.
    LatticeRestCage = 14,
    LatticeLiveCage = 15,
    /// Float: the weight of a blend shape's channel `part`, as its pose
    /// interpolator or its own input answers it. A List of these feeds a
    /// range chain member group by group (OpLookupKeyed).
    BlendChannelWeight = 16,
};

/// Whether a lookup of \p source is one the host answers through
/// LookUpSpace.
constexpr bool
OpSpaceLookupSource(uint8_t source)
{
    return source >= uint8_t(OpLookupSource::RestFrame) &&
           source <= uint8_t(OpLookupSource::ProviderCandidates);
}

/// Whether a lookup of \p source is one the host answers through
/// LookUpCompose.
constexpr bool
OpComposeLookupSource(uint8_t source)
{
    return source == uint8_t(OpLookupSource::AvarBindings) ||
           source == uint8_t(OpLookupSource::CheckpointInputs) ||
           source == uint8_t(OpLookupSource::InterpolatorInputs);
}

/// Whether a lookup of \p source answers a Mat4d that may be absent or,
/// for Matrix, always is present: a candidate of a Selected lookup.
constexpr bool
OpLookupSourceSelectable(uint8_t source)
{
    return source == uint8_t(OpLookupSource::Matrix) ||
           source == uint8_t(OpLookupSource::FrameRecord) ||
           source == uint8_t(OpLookupSource::ConstraintDelta);
}

/// OpLookup::read of a WeightArrays lookup, whose object is a weight object:
/// which of its inputs, at the frame. The numbers are part of the file
/// format.
enum class OpWeightRead : uint8_t {
    DefaultWeight = 0,
    Driver = 1,
    Scale = 2,
    Bias = 3,
    Strength = 4,
    Invert = 5,
    FalloffMin = 6,
    FalloffMax = 7,
    /// Vec3f: the volume's x, y and z scale; its positive and negative
    /// scales.
    ScaleAxes = 8,
    PositiveScales = 9,
    NegativeScales = 10,
    ExtentU = 11,
    ExtentV = 12,
    /// The painted values and indices.
    Values = 13,
    Indices = 14,
    /// The sampled, target and curve points, each list's attributes one
    /// after another.
    SamplePoints = 15,
    TargetPoints = 16,
    CurvePoints = 17,
    /// The falloff remap: the authored samples, else the compiled curve.
    FalloffSamples = 18,
    /// UInt: the combine's target point count.
    CombineTargetCount = 19,
};

/// OpLookup::read of a WeightFieldInputs lookup. The numbers are part of
/// the file format.
enum class OpFieldRead : uint8_t {
    /// Float: the weight object's scalar inputs, by the field's member
    /// number: 0 defaultWeight, 1 driver, 2 scale, 3 bias, 4 strength, 5
    /// invert, 6 falloffMin, 7 falloffMax, 17 extentU, 18 extentV.
    LastScalar = 18,
    /// Vec3f: the x, y and z scale; the positive and the negative scales.
    ScaleAxes = 19,
    PositiveScales = 20,
    NegativeScales = 21,
    /// The points a volume samples, as many as the field's points.
    SamplePoints = 22,
    CurvePoints = 23,
    /// The falloff remap: the sampled falloff, else the compiled curve.
    FalloffSamples = 24,
    /// Mat4d: the volume's placement at the field's placement phase.
    Placement = 25,
    /// UInt: how many points the field weighs.
    TargetCount = 26,
    /// The painted values and indices.
    Values = 27,
    Indices = 28,
    /// UInt: the plane's axis (0 x, 1 y, 2 z) and whether it is bounded.
    PlaneAxis = 29,
    PlaneBounded = 30,
};

/// The value of each scalar OpFieldRead a weight object does not author.
inline constexpr float kOpFieldScalarDefaults[19] = {
    0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

/// OpHostOutput::target: the family of host slots a port value is stored
/// in. The numbers are part of the file format.
enum class OpOutputTarget : uint8_t {
    PropertyVersion = 0,
    VolumePlacement = 1,
    VolumePlacementBase = 2,
    WeightPacket = 3,
    /// A range chain member's output, group by group, into the host's
    /// vertex-group state.
    ChainGroup = 4,
    /// The weight field `object`'s weights.
    WeightField = 5,
    /// Bool: whether revision `object` is enabled this frame, stored from
    /// the RevisionLeaf Enabled input of the node that stores its groups.
    RevisionEnabled = 6,
    /// Mat4d: the final and the base matrix of slot `object`.
    ProviderFinalMatrix = 20,
    ProviderBaseMatrix = 21,
    /// Frame record `object`'s matrix (Mat4d) and whether it is valid
    /// (Bool).
    FrameMatrix = 22,
    FrameMatrixValid = 23,
    /// Provider refresh `object`'s refreshed base and final frames
    /// (PointFrame) and its carried descendants' (PointFrameList).
    RefreshBase = 24,
    RefreshFinal = 25,
    RefreshCarryBase = 26,
    RefreshCarryFinal = 27,
    /// Provider operation `object`'s result into its provider value: a
    /// matrix (Mat4d) or a frame (PointFrame), published after its
    /// authority (Bool) and the input that left it unavailable (Int, -1
    /// when available) were stored.
    ProviderValueMatrix = 28,
    ProviderValueFrame = 29,
    /// PointFrameList element i into the version commit `object` writes
    /// for its candidate or descendant i: final, then base.
    CommitSlots = 30,
    CommitDescendants = 31,
    CommitBaseSlots = 32,
    CommitBaseDescendants = 33,
    /// BoolArray: which candidates of commit `object` are present.
    CommitPresent = 34,
    /// PointFrame: the frame constraint commit `object`'s mover solved.
    ConstraintCandidate = 35,
    /// Mat4d: the delta constraint commit `object` measured against its
    /// geometry target's authored transform.
    ConstraintDelta = 36,
    /// Bool: whether that delta could be measured; stored after it.
    ConstraintDeltaMeasured = 37,
    /// PointFrameArray: solver `object`'s frames, published as its
    /// aggregate and as the outputs its commit reads.
    SolverFrames = 60,
    /// UInt: what constraint commit `object`'s node made of its frames, as
    /// its kind's op reports it (Harmonics or SingleChainIk Status,
    /// IkFkMatch ShownMask).
    ConstraintStatus = 70,
    /// PointFrameArray: the chain single-chain IK constraint commit
    /// `object`'s node solved, stored after its status.
    SingleChainIkFrames = 71,
    /// DoubleArray: the values IK/FK match constraint commit `object`'s node
    /// matched, IK then FK; Mat4dArray: the controls it presents. Each is
    /// stored after its status.
    IkFkMatchIkValues = 72,
    IkFkMatchFkValues = 73,
    IkFkMatchShown = 74,
    /// DoubleArray: an AvarInputs node's values into provider slot
    /// `object`'s bound avars, in ascending channel order.
    Avars = 40,
    /// Mat4d: space checkpoint `object`'s switch frame.
    SwitchFrame = 41,
    /// FloatArray: pose interpolator `object`'s weights in its solve's
    /// pose order; its other weight slots publish zero.
    PoseWeights = 42,
    /// Provider operation `object`'s authority and unavailable input,
    /// stored before its ProviderValueMatrix or ProviderValueFrame.
    ProviderValueAuthority = 50,
    ProviderValueUnavailable = 51,
    /// Int: the candidate whose value answers provider value producer
    /// `object` (eval::spaces::ProviderProducers numbering), -1 when that
    /// candidate holds none; published after the answer's authority and
    /// blocked flag (Bool) were stored.
    ProviderAnswer = 52,
    ProviderAnswerAuthority = 53,
    ProviderAnswerBlocked = 54,
};

/// Whether an output of \p target is one the host stores through
/// StoreSpace.
constexpr bool
OpSpaceOutputTarget(uint8_t target)
{
    return (target >= uint8_t(OpOutputTarget::ProviderFinalMatrix) &&
            target <= uint8_t(OpOutputTarget::ProviderValueFrame)) ||
           target == uint8_t(OpOutputTarget::ProviderValueAuthority) ||
           target == uint8_t(OpOutputTarget::ProviderValueUnavailable) ||
           (target >= uint8_t(OpOutputTarget::ProviderAnswer) &&
            target <= uint8_t(OpOutputTarget::ProviderAnswerBlocked));
}

/// Whether an output of \p target is one the host stores through
/// StoreConstraint.
constexpr bool
OpConstraintOutputTarget(uint8_t target)
{
    return (target >= uint8_t(OpOutputTarget::ConstraintCandidate) &&
            target <= uint8_t(OpOutputTarget::ConstraintDeltaMeasured)) ||
           target == uint8_t(OpOutputTarget::ConstraintStatus) ||
           (target >= uint8_t(OpOutputTarget::SingleChainIkFrames) &&
            target <= uint8_t(OpOutputTarget::IkFkMatchShown));
}

/// Whether an output of \p target is one the host stores through
/// StoreCompose.
constexpr bool
OpComposeOutputTarget(uint8_t target)
{
    return target == uint8_t(OpOutputTarget::Avars) ||
           target == uint8_t(OpOutputTarget::SwitchFrame) ||
           target == uint8_t(OpOutputTarget::PoseWeights);
}

/// Whether OpLookUp resolves lookups of \p source.
constexpr bool
OpLookupSourceRuns(uint8_t source)
{
    return source == uint8_t(OpLookupSource::Property) ||
           source == uint8_t(OpLookupSource::PoseVersion) ||
           source == uint8_t(OpLookupSource::WeightArrays) ||
           source == uint8_t(OpLookupSource::ChainVersion) ||
           source == uint8_t(OpLookupSource::WeightFieldInputs) ||
           source == uint8_t(OpLookupSource::List) ||
           source == uint8_t(OpLookupSource::Selected) ||
           source == uint8_t(OpLookupSource::RevisionLeaf) ||
           source == uint8_t(OpLookupSource::CommitVersions) ||
           source == uint8_t(OpLookupSource::CommitCandidates) ||
           OpConstraintLookupSource(source) ||
           source == uint8_t(OpLookupSource::SolverInput) ||
           OpLookupSourceSelectable(source) || OpSpaceLookupSource(source) ||
           OpComposeLookupSource(source);
}

/// Whether OpStoreOutput stores outputs of \p target, or, for a range chain
/// member, the host's StoreGroup does.
constexpr bool
OpOutputTargetRuns(uint8_t target, bool chainMember)
{
    return chainMember
               ? target == uint8_t(OpOutputTarget::ChainGroup) ||
                     target == uint8_t(OpOutputTarget::RevisionEnabled)
               : target == uint8_t(OpOutputTarget::PropertyVersion) ||
                     target == uint8_t(OpOutputTarget::VolumePlacement) ||
                     target == uint8_t(OpOutputTarget::VolumePlacementBase) ||
                     target == uint8_t(OpOutputTarget::WeightPacket) ||
                     target == uint8_t(OpOutputTarget::WeightField) ||
                     OpSpaceOutputTarget(target) ||
                     OpComposeOutputTarget(target) ||
                     (target >= uint8_t(OpOutputTarget::CommitSlots) &&
                      target <=
                          uint8_t(OpOutputTarget::ConstraintDeltaMeasured)) ||
                     OpConstraintOutputTarget(target) ||
                     target == uint8_t(OpOutputTarget::SolverFrames);
}

/// A per-frame read of the rig the host resolves for a Lookup input; what
/// \p object, \p part and \p read name depends on \p source.
struct OpLookup {
    uint8_t source = 0;
    uint32_t object = 0;
    int32_t part = -1;
    uint8_t read = 0;
};

/// A port value the host stores in one of its own slots after the node
/// runs: an output, or an input a lookup feeds. What \p object names
/// depends on \p target.
struct OpHostOutput {
    uint32_t node = 0;
    uint32_t port = 0;
    uint8_t target = 0;
    uint32_t object = 0;
};

/// One value of a port's type, constructed and destroyed with it.
class OpPortValue {
public:
    explicit OpPortValue(const RigExecTypeDesc &type)
        : _type(type), _storage(type.size / sizeof(std::max_align_t) + 1)
    {
        if (_type.construct)
            _type.construct(_storage.data());
    }
    ~OpPortValue()
    {
        if (_type.destroy)
            _type.destroy(_storage.data());
    }
    OpPortValue(const OpPortValue &) = delete;
    OpPortValue &operator=(const OpPortValue &) = delete;
    const RigExecTypeDesc &Type() const
    {
        return _type;
    }
    void *Value()
    {
        return _storage.data();
    }
    const void *Value() const
    {
        return _storage.data();
    }

private:
    const RigExecTypeDesc &_type;
    std::vector<std::max_align_t> _storage;
};

/// An op input the host writes from the rig: the node and port it feeds,
/// and the chain or joint slot its value comes from.
struct OpHostInput {
    uint32_t node = 0;
    uint32_t port = 0;
    OpHostSource source = OpHostSource::ChainPoints;
    uint32_t object = 0;
};

/// An op input fixed when the rig was built: the node and port it feeds,
/// and its value's bytes, an array's elements one after another.
struct OpConstant {
    uint32_t node = 0;
    uint32_t port = 0;
    std::vector<uint8_t> bytes;
};

/// A constant as a value of its port's type: an array borrows the
/// constant's bytes, a value holding no array copies them.
class OpConstantValue {
public:
    OpConstantValue(const RigExecTypeDesc &type, const OpConstant &constant)
        : _type(type), _bytes(constant.bytes),
          _storage(type.size / sizeof(std::max_align_t) + 1)
    {
        if (_type.construct)
            _type.construct(_storage.data());
        if (_type.element && _type.borrow) {
            const size_t element = RigExecRegistry::Get()
                                       .TypeAt(_type.element->Index())
                                       .desc->size;
            _valid = element && _bytes.size() % element == 0;
            if (_valid)
                _type.borrow(_storage.data(), _bytes.data(),
                             _bytes.size() / element);
        } else if (_type.triviallyCopyable && _bytes.size() == _type.size) {
            std::memcpy(_storage.data(), _bytes.data(), _type.size);
            _valid = true;
        }
    }
    ~OpConstantValue()
    {
        if (_type.destroy)
            _type.destroy(_storage.data());
    }
    OpConstantValue(const OpConstantValue &) = delete;
    OpConstantValue &operator=(const OpConstantValue &) = delete;
    bool Valid() const
    {
        return _valid;
    }
    const void *Value() const
    {
        return _storage.data();
    }

private:
    const RigExecTypeDesc &_type;
    std::vector<uint8_t> _bytes;
    std::vector<std::max_align_t> _storage;
    bool _valid = false;
};

/// One range chain as steps: the point chain whose vertex groups it runs
/// in and their bounds; and, member by member, the groups each member
/// writes (member-major), empty when every member writes every group. A
/// group no member of a segment writes gets no range step; the host's
/// version of a group a member does not write is the previous writer's.
struct OpChainLayout {
    uint32_t pointChain = 0;
    std::vector<int> bounds;
    std::vector<uint8_t> writes;
    uint32_t Groups() const
    {
        return bounds.size() > 1 ? uint32_t(bounds.size() - 1) : 0;
    }
    bool Writes(uint32_t member, uint32_t g) const
    {
        return writes.empty() || writes[size_t(member) * Groups() + g] != 0;
    }
};

/// Whether a member of the segment starting at member \p segment of
/// \p desc writes group \p g, or any group when the segment runs whole.
inline bool
OpSegmentWrites(const OpChainLayout &chain, const RigExecChainDesc &desc,
                uint32_t segment, uint32_t g)
{
    for (uint32_t m = segment;
         m < desc.members.size() && desc.segments[m] == segment; ++m)
        for (uint32_t h = desc.whole[m] ? 0 : g;
             h < (desc.whole[m] ? chain.Groups() : g + 1); ++h)
            if (chain.Writes(m, h))
                return true;
    return false;
}

/// The step kinds' wire and baked numbers.
enum class OpStepKind : uint8_t { Node = 30, ChainHead = 31, ChainRange = 32 };

/// What every run of the op steps shares; fixed once laid out. OpValue
/// slots number the program's values (their value numbers) and then one
/// plan per chain; OpGroupValue slots number each group of each value a
/// chain member writes.
struct OpStepLayout {
    std::shared_ptr<const RigExecProgram> program;
    std::vector<OpHostInput> hostInputs;
    std::vector<OpLookup> lookups;
    std::vector<OpHostOutput> hostOutputs;
    std::vector<OpConstant> constants;
    std::vector<OpChainLayout> chains;
    /// Filled by LayOutOpSteps.
    std::vector<std::shared_ptr<const OpConstantValue>> constantValues;
    std::vector<uint32_t> constantIds;
    std::vector<RigExecNodeDesc> nodes;
    std::vector<RigExecChainDesc> chainDescs;
    /// Per program node: its chain and member index, or kRigExecInvalidIndex.
    std::vector<uint32_t> chainOf, memberOf;
    /// Per hostInputs entry: the program input it writes, and how a range
    /// chain member reads it group by group: 0 not, 1 through a pointwise
    /// input, 2 through a neighbourhood input. Chain points and a
    /// ChainVersion lookup feeding a point input are read group by group.
    std::vector<uint32_t> hostInputIds;
    std::vector<uint8_t> grouped;
    /// Per program node: its Lookup hostInputs entries and its hostOutputs
    /// entries.
    std::vector<std::vector<uint32_t>> lookupsOf, hostOutputsOf;
    /// Per range chain: the hostInputs entries its members read group by
    /// group.
    std::vector<std::vector<uint32_t>> groupedOf;
    /// Per value number: its first OpGroupValue slot, or kRigExecInvalidIndex,
    /// and the node writing it, or kRigExecInvalidIndex for an input.
    std::vector<uint32_t> groupBase, producerOf;
    /// Per hostInputs entry: whether it is keyed (OpLookupKeyed). Per range
    /// chain: whether a member reads a keyed input, and its first group
    /// plan slot past the chains' plans.
    std::vector<uint8_t> keyed, chainKeyed;
    std::vector<uint32_t> planGroupBase;
    uint32_t valueCount = 0;
    uint32_t groupValueCount = 0;
    uint32_t planGroupCount = 0;

    uint32_t PlanSlot(size_t chain) const
    {
        return valueCount + uint32_t(chain);
    }
    /// The plan of group \p g of a keyed chain: it moves when a keyed
    /// element that reaches the group does.
    uint32_t PlanGroupSlot(size_t chain, uint32_t g) const
    {
        return valueCount + uint32_t(chains.size()) + planGroupBase[chain] + g;
    }
    uint32_t OpValueCount() const
    {
        return valueCount + uint32_t(chains.size()) + planGroupCount;
    }
};

/// Whether \p lookup lists blend channel weights of one revision, each
/// element reaching only the vertex groups the host's LookupGroups names: a
/// range chain's head moves the plans of those groups alone when it moves.
inline bool
OpLookupKeyed(const OpStepLayout &layout, const OpLookup &lookup)
{
    if (lookup.source != uint8_t(OpLookupSource::List) || lookup.part <= 0 ||
        size_t(lookup.object) + size_t(lookup.part) > layout.lookups.size())
        return false;
    for (int32_t e = 0; e < lookup.part; ++e) {
        const OpLookup &element = layout.lookups[lookup.object + uint32_t(e)];
        if (element.source != uint8_t(OpLookupSource::RevisionLeaf) ||
            element.read != uint8_t(OpRevisionRead::BlendChannelWeight))
            return false;
    }
    return true;
}

inline bool
OpLayoutFail(std::string *error, const std::string &why)
{
    if (error)
        *error = "op steps: " + why;
    return false;
}

/// Fills \p layout's derived tables from its program, host inputs and
/// chains. False with the reason when the program was not compiled for a
/// host's schedule, a chain has no groups, or a host input does not exist
/// or feeds what its source cannot.
inline bool
LayOutOpSteps(OpStepLayout *layout, std::string *error)
{
    const RigExecProgram &program = *layout->program;
    const size_t nodes = RigExecProgramNodeCount(program);
    const size_t chains = RigExecProgramChainCount(program);
    if (layout->chains.size() != chains)
        return OpLayoutFail(error, "the program has " + std::to_string(chains) +
                                       " range chains, the layout " +
                                       std::to_string(layout->chains.size()));
    layout->valueCount = uint32_t(RigExecProgramValueCount(program));
    layout->nodes.assign(nodes, RigExecNodeDesc());
    for (size_t n = 0; n < nodes; ++n)
        RigExecProgramNodeAt(program, n, &layout->nodes[n]);
    layout->chainDescs.assign(chains, RigExecChainDesc());
    layout->chainOf.assign(nodes, kRigExecInvalidIndex);
    layout->memberOf.assign(nodes, kRigExecInvalidIndex);
    layout->groupBase.assign(layout->valueCount, kRigExecInvalidIndex);
    layout->producerOf.assign(layout->valueCount, kRigExecInvalidIndex);
    for (size_t n = 0; n < nodes; ++n) {
        const RigExecNodeDesc &d = layout->nodes[n];
        for (size_t p = 0; p < d.values.size(); ++p)
            if ((d.op->ports[p].flags & kRigExecPortOutput) &&
                d.values[p] != kRigExecInvalidIndex)
                layout->producerOf[d.values[p]] = uint32_t(n);
    }
    layout->groupValueCount = 0;
    for (size_t c = 0; c < chains; ++c) {
        RigExecChainDesc &desc = layout->chainDescs[c];
        RigExecProgramChainAt(program, c, &desc);
        const uint32_t groups = layout->chains[c].Groups();
        if (!groups)
            return OpLayoutFail(error, "chain " + std::to_string(c) +
                                           " runs in no vertex groups");
        for (uint32_t m = 0; m < desc.members.size(); ++m) {
            const uint32_t node = desc.members[m];
            layout->chainOf[node] = uint32_t(c);
            layout->memberOf[node] = m;
            const RigExecNodeDesc &d = layout->nodes[node];
            for (size_t p = 0; p < d.values.size(); ++p) {
                if (!(d.op->ports[p].flags & kRigExecPortOutput) ||
                    d.values[p] == kRigExecInvalidIndex)
                    continue;
                layout->groupBase[d.values[p]] = layout->groupValueCount;
                layout->groupValueCount += groups;
            }
        }
    }
    layout->planGroupBase.assign(chains, 0);
    layout->planGroupCount = 0;
    for (size_t c = 0; c < chains; ++c) {
        layout->planGroupBase[c] = layout->planGroupCount;
        layout->planGroupCount += layout->chains[c].Groups();
    }
    // Per value: the nodes reading it, built for the first member that
    // leaves groups unwritten.
    std::vector<uint32_t> readerStart, readers;
    const auto indexReaders = [&] {
        readerStart.assign(size_t(layout->valueCount) + 1, 0);
        const auto visit = [&](auto &&add) {
            for (uint32_t n = 0; n < nodes; ++n) {
                const RigExecNodeDesc &d = layout->nodes[n];
                for (size_t p = 0; p < d.values.size(); ++p) {
                    if (!(d.op->ports[p].flags & kRigExecPortInput))
                        continue;
                    if (d.values[p] < layout->valueCount)
                        add(d.values[p], n);
                    for (const uint32_t v : d.lists[p])
                        if (v < layout->valueCount)
                            add(v, n);
                }
            }
        };
        visit([&](uint32_t v, uint32_t) { ++readerStart[v + 1]; });
        for (size_t v = 0; v < layout->valueCount; ++v)
            readerStart[v + 1] += readerStart[v];
        readers.resize(readerStart.back());
        std::vector<uint32_t> next(readerStart.begin(), readerStart.end() - 1);
        visit([&](uint32_t v, uint32_t n) { readers[next[v]++] = n; });
    };
    for (size_t c = 0; c < chains; ++c) {
        const OpChainLayout &chain = layout->chains[c];
        const RigExecChainDesc &desc = layout->chainDescs[c];
        if (chain.writes.empty())
            continue;
        if (chain.writes.size() != desc.members.size() * chain.Groups())
            return OpLayoutFail(error, "chain " + std::to_string(c) +
                                           " names the groups its members "
                                           "write in " +
                                           std::to_string(chain.writes.size()) +
                                           " entries");
        for (uint32_t m = 0; m < desc.members.size(); ++m) {
            bool all = true;
            for (uint32_t g = 0; g < chain.Groups(); ++g)
                all = all && chain.Writes(m, g);
            if (all)
                continue;
            if (readerStart.empty())
                indexReaders();
            const RigExecNodeDesc &member = layout->nodes[desc.members[m]];
            uint32_t outside = kRigExecInvalidIndex;
            for (size_t q = 0; q < member.values.size(); ++q) {
                const uint32_t v = member.values[q];
                if (!(member.op->ports[q].flags & kRigExecPortOutput) ||
                    v >= layout->valueCount)
                    continue;
                for (uint32_t r = readerStart[v]; r < readerStart[v + 1]; ++r) {
                    const uint32_t n = readers[r];
                    if (layout->chainOf[n] != c ||
                        desc.segments[layout->memberOf[n]] != desc.segments[m])
                        outside = std::min(outside, n);
                }
            }
            if (outside != kRigExecInvalidIndex)
                return OpLayoutFail(
                    error, "node " + std::to_string(outside) + " reads node " +
                               std::to_string(desc.members[m]) +
                               " outside its segment, which leaves "
                               "groups unwritten");
        }
    }
    layout->hostInputIds.clear();
    layout->grouped.clear();
    layout->keyed.clear();
    layout->chainKeyed.assign(chains, 0);
    layout->lookupsOf.assign(nodes, std::vector<uint32_t>());
    layout->hostOutputsOf.assign(nodes, std::vector<uint32_t>());
    for (const OpHostInput &input : layout->hostInputs) {
        if (input.node >= nodes ||
            input.port >= layout->nodes[input.node].inputs.size() ||
            layout->nodes[input.node].inputs[input.port] ==
                kRigExecInvalidIndex)
            return OpLayoutFail(error, "node " + std::to_string(input.node) +
                                           " has no host input " +
                                           std::to_string(input.port));
        const uint32_t id = layout->nodes[input.node].inputs[input.port];
        const uint32_t flags =
            layout->nodes[input.node].op->ports[input.port].flags;
        const bool member = layout->chainOf[input.node] != kRigExecInvalidIndex;
        const bool points =
            (flags & (kRigExecPortPointwise | kRigExecPortNeighbourhood)) != 0;
        const uint8_t grouped =
            (flags & kRigExecPortNeighbourhood) ? uint8_t(2) : uint8_t(1);
        layout->grouped.push_back(0);
        layout->keyed.push_back(0);
        if (input.source == OpHostSource::ChainPoints) {
            if (!member || !points)
                return OpLayoutFail(error,
                                    "chain points feed node " +
                                        std::to_string(input.node) +
                                        ", which does not read them point "
                                        "by point in a range chain");
            layout->grouped.back() = grouped;
        }
        if (input.source == OpHostSource::Lookup) {
            if (input.object >= layout->lookups.size())
                return OpLayoutFail(
                    error, "node " + std::to_string(input.node) +
                               " reads lookup " + std::to_string(input.object) +
                               ", which the layout lacks");
            if (!OpLookupSourceRuns(layout->lookups[input.object].source))
                return OpLayoutFail(
                    error,
                    "node " + std::to_string(input.node) +
                        " reads lookup source " +
                        std::to_string(layout->lookups[input.object].source) +
                        ", which no host resolves");
            const OpLookup &lookup = layout->lookups[input.object];
            const bool version =
                lookup.source == uint8_t(OpLookupSource::ChainVersion);
            if (points && !(version && member))
                return OpLayoutFail(error, "a lookup feeds point input " +
                                               std::to_string(input.port) +
                                               " of node " +
                                               std::to_string(input.node));
            if (flags & kRigExecPortChecked)
                return OpLayoutFail(error, "a lookup feeds checked input " +
                                               std::to_string(input.port) +
                                               " of node " +
                                               std::to_string(input.node));
            if (lookup.source == uint8_t(OpLookupSource::List)) {
                const RigExecTypeDesc &type =
                    *RigExecRegistry::Get()
                         .TypeAt(layout->nodes[input.node]
                                     .op->ports[input.port]
                                     .type->Index())
                         .desc;
                bool listed = lookup.part >= 0 && type.element && type.resize &&
                              type.data &&
                              RigExecRegistry::Get()
                                  .TypeAt(type.element->Index())
                                  .desc->triviallyCopyable &&
                              size_t(lookup.object) + size_t(lookup.part) <=
                                  layout->lookups.size();
                for (int32_t e = 0; listed && e < lookup.part; ++e) {
                    const uint8_t source =
                        layout->lookups[lookup.object + uint32_t(e)].source;
                    listed = OpLookupSourceRuns(source) &&
                             source != uint8_t(OpLookupSource::List) &&
                             source != uint8_t(OpLookupSource::Selected);
                }
                if (!listed)
                    return OpLayoutFail(
                        error, "node " + std::to_string(input.node) +
                                   " reads lookup list " +
                                   std::to_string(input.object) +
                                   ", whose elements the layout lacks or "
                                   "its input cannot hold");
            }
            if (lookup.source == uint8_t(OpLookupSource::Selected)) {
                bool selectable = lookup.part > 0 &&
                                  size_t(lookup.object) + size_t(lookup.part) <=
                                      layout->lookups.size();
                for (int32_t e = 0; selectable && e < lookup.part; ++e)
                    selectable = OpLookupSourceSelectable(
                        layout->lookups[lookup.object + uint32_t(e)].source);
                if (!selectable)
                    return OpLayoutFail(error,
                                        "node " + std::to_string(input.node) +
                                            " selects among lookups from " +
                                            std::to_string(lookup.object) +
                                            ", which the layout lacks or which "
                                            "answer no matrix");
            }
            if (OpLookupKeyed(*layout, lookup)) {
                const RigExecTypeDesc &type =
                    *RigExecRegistry::Get()
                         .TypeAt(layout->nodes[input.node]
                                     .op->ports[input.port]
                                     .type->Index())
                         .desc;
                if (!member || !type.element ||
                    RigExecRegistry::Get()
                            .TypeAt(type.element->Index())
                            .desc->size != sizeof(float))
                    return OpLayoutFail(error,
                                        "node " + std::to_string(input.node) +
                                            " reads channel weights group by "
                                            "group outside a range chain or "
                                            "into no float array");
                layout->keyed.back() = 1;
                layout->chainKeyed[layout->chainOf[input.node]] = 1;
            }
            if (points)
                layout->grouped.back() = grouped;
            else
                layout->lookupsOf[input.node].push_back(
                    uint32_t(layout->hostInputIds.size()));
        }
        layout->hostInputIds.push_back(id);
    }
    layout->groupedOf.assign(chains, std::vector<uint32_t>());
    for (uint32_t i = 0; i < layout->hostInputs.size(); ++i)
        if (layout->grouped[i])
            layout->groupedOf[layout->chainOf[layout->hostInputs[i].node]]
                .push_back(i);
    for (uint32_t o = 0; o < layout->hostOutputs.size(); ++o) {
        const OpHostOutput &output = layout->hostOutputs[o];
        bool stored = output.node < nodes &&
                      output.port < layout->nodes[output.node].values.size() &&
                      layout->nodes[output.node].values[output.port] !=
                          kRigExecInvalidIndex;
        if (stored &&
            !(layout->nodes[output.node].op->ports[output.port].flags &
              kRigExecPortOutput)) {
            const auto looksUp = [&](uint32_t i) {
                const OpHostInput &input = layout->hostInputs[i];
                return input.node == output.node && input.port == output.port &&
                       input.source == OpHostSource::Lookup;
            };
            stored = false;
            for (const uint32_t i : layout->lookupsOf[output.node])
                stored = stored || looksUp(i);
            const uint32_t chain = layout->chainOf[output.node];
            if (chain != kRigExecInvalidIndex)
                for (const uint32_t i : layout->groupedOf[chain])
                    stored = stored || looksUp(i);
        }
        if (!stored)
            return OpLayoutFail(error, "node " + std::to_string(output.node) +
                                           " has no output or looked-up "
                                           "input " +
                                           std::to_string(output.port) +
                                           " to store");
        const bool member =
            layout->chainOf[output.node] != kRigExecInvalidIndex;
        const bool chainGroup =
            output.target == uint8_t(OpOutputTarget::ChainGroup);
        if (!OpOutputTargetRuns(output.target, member) ||
            (member &&
             chainGroup !=
                 ((layout->nodes[output.node].op->ports[output.port].flags &
                   kRigExecPortOutput) != 0)))
            return OpLayoutFail(
                error, "node " + std::to_string(output.node) + " stores port " +
                           std::to_string(output.port) + " in output target " +
                           std::to_string(output.target) +
                           ", which no host stores");
        layout->hostOutputsOf[output.node].push_back(o);
    }
    layout->constantValues.clear();
    layout->constantIds.clear();
    for (const OpConstant &constant : layout->constants) {
        if (constant.node >= nodes ||
            constant.port >= layout->nodes[constant.node].inputs.size() ||
            layout->nodes[constant.node].inputs[constant.port] ==
                kRigExecInvalidIndex)
            return OpLayoutFail(error, "node " + std::to_string(constant.node) +
                                           " has no input " +
                                           std::to_string(constant.port) +
                                           " to fix");
        const RigExecPortDesc &port =
            layout->nodes[constant.node].op->ports[constant.port];
        auto value = std::make_shared<const OpConstantValue>(
            *RigExecRegistry::Get().TypeAt(port.type->Index()).desc, constant);
        if (!value->Valid())
            return OpLayoutFail(
                error, "the value fixed for input " + std::string(port.name) +
                           " of node " + std::to_string(constant.node) +
                           " is not of its type");
        layout->constantValues.push_back(std::move(value));
        layout->constantIds.push_back(
            layout->nodes[constant.node].inputs[constant.port]);
    }
    return true;
}

/// One run of a layout: its run state, the groups each chain-points input
/// is handed (each range step sets its own group's entry), and the
/// version of every op value, bumped by the step that writes it.
struct OpStepState {
    std::unique_ptr<RigExecRunState> run;
    /// Per hostInputs entry: the value a Lookup input reads, else null.
    std::vector<std::unique_ptr<OpPortValue>> lookedUp;
    /// Per hostInputs entry: the bytes a trivially copyable looked-up input
    /// held when its chain's head last ran.
    std::vector<std::vector<uint8_t>> lookedUpBytes;
    /// Per op value a node step writes: the bytes of a trivially copyable
    /// value when its version last moved.
    std::vector<std::vector<uint8_t>> valueBytes;
    std::vector<std::vector<const void *>> groupPoints;
    std::vector<RigExecOpGroups> groups;
    /// Per hostInputs entry read through a neighbourhood input, per group:
    /// the groups table that group's range reads and its RigExecOpGroups;
    /// and the groups it reads, found by the chain's head.
    std::vector<std::vector<std::vector<const void *>>> haloPoints;
    std::vector<std::vector<RigExecOpGroups>> haloGroups;
    std::vector<std::vector<std::vector<uint32_t>>> haloReads;
    std::vector<uint64_t> versions, groupVersions;
};

/// Gives \p to, bound by BindOpStepState, \p from's looked-up input values:
/// a range step can run before its chain's head resolves them again.
inline void
CopyOpLookups(const OpStepState &from, OpStepState *to)
{
    for (size_t i = 0; i < to->lookedUp.size() && i < from.lookedUp.size();
         ++i) {
        if (!to->lookedUp[i] || !from.lookedUp[i])
            continue;
        const RigExecTypeDesc &type = to->lookedUp[i]->Type();
        if (type.triviallyCopyable)
            std::memcpy(to->lookedUp[i]->Value(), from.lookedUp[i]->Value(),
                        type.size);
        else if (type.copy)
            type.copy(to->lookedUp[i]->Value(), from.lookedUp[i]->Value());
    }
    to->lookedUpBytes = from.lookedUpBytes;
    to->valueBytes = from.valueBytes;
}

/// Group \p g of the points host input \p input reads group by group: of
/// its chain's final points, or of the chain version its lookup names.
template <class Host>
const void *
OpGroupPoints(const OpStepLayout &layout, Host &host, const OpHostInput &input,
              uint32_t g)
{
    return input.source == OpHostSource::ChainPoints
               ? host.ChainGroupPoints(input.object, g)
               : host.ChainVersionGroupPoints(layout.lookups[input.object], g);
}

/// Points \p state's run at the host's values: each matrix input at what
/// \p host.Matrix answers, each input read group by group at a group
/// table its range steps fill, one per group for a neighbourhood input.
/// Call it on a new or cloned state, before its first step, whenever the
/// host's storage moved; a clone takes its source's haloReads.
template <class Host>
bool
BindOpStepState(const OpStepLayout &layout, Host &host, OpStepState *state,
                std::string *error)
{
    state->groupPoints.assign(layout.hostInputs.size(),
                              std::vector<const void *>());
    state->groups.assign(layout.hostInputs.size(), RigExecOpGroups());
    state->haloPoints.assign(layout.hostInputs.size(), {});
    state->haloGroups.assign(layout.hostInputs.size(), {});
    state->haloReads.resize(layout.hostInputs.size());
    state->lookedUp.clear();
    state->lookedUp.resize(layout.hostInputs.size());
    state->lookedUpBytes.assign(layout.hostInputs.size(), {});
    state->valueBytes.resize(layout.OpValueCount());
    state->versions.resize(layout.OpValueCount(), 0);
    state->groupVersions.resize(layout.groupValueCount, 0);
    for (size_t i = 0; i < layout.constantIds.size(); ++i)
        state->run->PointInput(RigExecInputId{layout.constantIds[i]},
                               layout.constantValues[i]->Value());
    for (size_t i = 0; i < layout.hostInputs.size(); ++i) {
        const OpHostInput &input = layout.hostInputs[i];
        const RigExecInputId id{layout.hostInputIds[i]};
        if (layout.grouped[i] == 2) {
            const OpChainLayout &chain =
                layout.chains[layout.chainOf[input.node]];
            const uint32_t groups = chain.Groups();
            state->haloPoints[i].assign(
                groups, std::vector<const void *>(groups, nullptr));
            state->haloGroups[i].assign(groups, RigExecOpGroups());
            for (uint32_t g = 0; g < groups; ++g) {
                state->haloPoints[i][g][g] =
                    OpGroupPoints(layout, host, input, g);
                state->haloGroups[i][g] = {chain.bounds.data(), groups,
                                           state->haloPoints[i][g].data()};
            }
            if (!state->run->PointInputHaloGroups(
                    id, state->haloGroups[i].data(), error))
                return false;
            continue;
        }
        if (input.source == OpHostSource::Lookup && !layout.grouped[i]) {
            const RigExecPortDesc &port =
                layout.nodes[input.node].op->ports[input.port];
            state->lookedUp[i] = std::make_unique<OpPortValue>(
                *RigExecRegistry::Get().TypeAt(port.type->Index()).desc);
            if (!state->run->PointInput(id, state->lookedUp[i]->Value()))
                return OpLayoutFail(
                    error, "node " + std::to_string(input.node) +
                               " has no input " + std::to_string(input.port));
            continue;
        }
        if (!layout.grouped[i]) {
            const void *value = host.Matrix(input.source, input.object);
            if (!value || !state->run->PointInput(id, value))
                return OpLayoutFail(error, "slot " +
                                               std::to_string(input.object) +
                                               " has no such matrix");
            continue;
        }
        const OpChainLayout &chain = layout.chains[layout.chainOf[input.node]];
        state->groupPoints[i].assign(chain.Groups(), nullptr);
        for (uint32_t g = 0; g < chain.Groups(); ++g)
            state->groupPoints[i][g] = OpGroupPoints(layout, host, input, g);
        state->groups[i] = {chain.bounds.data(), chain.Groups(),
                            state->groupPoints[i].data()};
        if (!state->run->PointInputGroups(id, &state->groups[i], error))
            return false;
    }
    return true;
}

/// Resolves lookup \p index through the \p host method of its source's
/// family: LookUpProperty, LookUpPoseVersion, LookUpWeight,
/// LookUpChainVersion, LookUpWeightField, LookUpCommit, LookUpConstraint,
/// LookUpCompose or LookUpSolver(index, lookup, type, value, error), or,
/// for a RevisionLeaf read past Radial, LookUpRevision(index, lookup, type,
/// value, error), a Matrix lookup through Matrix(source, slot), and a FrameRecord
/// or ConstraintDelta lookup through PresentMatrix(lookup), null when
/// absent. A List or Selected lookup resolves through OpLookUpAt.
template <class Host>
bool
OpLookUp(Host &host, uint32_t index, const OpLookup &lookup,
         const RigExecTypeDesc &type, void *value, std::string *error)
{
    switch (OpLookupSource(lookup.source)) {
    case OpLookupSource::Property:
        return host.LookUpProperty(index, lookup, type, value, error);
    case OpLookupSource::PoseVersion:
        return host.LookUpPoseVersion(index, lookup, type, value, error);
    case OpLookupSource::WeightArrays:
        return host.LookUpWeight(index, lookup, type, value, error);
    case OpLookupSource::ChainVersion:
        return host.LookUpChainVersion(index, lookup, type, value, error);
    case OpLookupSource::WeightFieldInputs:
        return host.LookUpWeightField(index, lookup, type, value, error);
    case OpLookupSource::RevisionLeaf: {
        if (lookup.read > uint8_t(OpRevisionRead::Radial))
            return host.LookUpRevision(index, lookup, type, value, error);
        bool enabled = true, radial = false;
        float weight = 1.0f;
        if (!host.RevisionLeaf(lookup.object, &enabled, &weight, &radial,
                               error))
            return false;
        if ((lookup.read == uint8_t(OpRevisionRead::Enabled) ||
             lookup.read == uint8_t(OpRevisionRead::Radial)) &&
            type.triviallyCopyable && type.size == 1) {
            const uint8_t flag =
                (lookup.read == uint8_t(OpRevisionRead::Enabled) ? enabled
                                                                 : radial)
                    ? 1
                    : 0;
            std::memcpy(value, &flag, 1);
            return true;
        }
        if (lookup.read == uint8_t(OpRevisionRead::DefaultWeight) &&
            type.triviallyCopyable && type.size == sizeof(float)) {
            std::memcpy(value, &weight, sizeof(float));
            return true;
        }
        return OpLayoutFail(error, "revision " + std::to_string(lookup.object) +
                                       " has no input " +
                                       std::to_string(lookup.read) +
                                       " of that type");
    }
    case OpLookupSource::CommitVersions:
    case OpLookupSource::CommitCandidates:
        return host.LookUpCommit(index, lookup, type, value, error);
    case OpLookupSource::ConstraintSources:
    case OpLookupSource::ConstraintArrays:
    case OpLookupSource::ConstraintLeaf:
    case OpLookupSource::ConstraintEnvelope:
    case OpLookupSource::ConstraintSpace:
    case OpLookupSource::ConstraintWorldUp:
    case OpLookupSource::HarmonicsInput:
    case OpLookupSource::SingleChainIkInput:
    case OpLookupSource::IkFkMatchInput:
    case OpLookupSource::ConstraintDeltaBase:
        return host.LookUpConstraint(index, lookup, type, value, error);
    case OpLookupSource::SolverInput:
        return host.LookUpSolver(index, lookup, type, value, error);
    case OpLookupSource::Matrix: {
        const void *matrix = host.Matrix(
            lookup.part ? OpHostSource::BaseMatrix : OpHostSource::FinalMatrix,
            lookup.object);
        if (!matrix || !type.triviallyCopyable ||
            type.size != 16 * sizeof(double))
            return OpLayoutFail(error, "slot " + std::to_string(lookup.object) +
                                           " has no such matrix");
        std::memcpy(value, matrix, type.size);
        return true;
    }
    case OpLookupSource::FrameRecord:
    case OpLookupSource::ConstraintDelta: {
        const void *matrix = host.PresentMatrix(lookup);
        if (!matrix || !type.triviallyCopyable ||
            type.size != 16 * sizeof(double))
            return OpLayoutFail(
                error, std::string(lookup.source ==
                                           uint8_t(OpLookupSource::FrameRecord)
                                       ? "frame record "
                                       : "constraint delta ") +
                           std::to_string(lookup.object) +
                           " holds no matrix this frame");
        std::memcpy(value, matrix, type.size);
        return true;
    }
    case OpLookupSource::List:
    case OpLookupSource::Selected:
        break;
    case OpLookupSource::RestFrame:
    case OpLookupSource::CommitRecordGate:
    case OpLookupSource::ProviderFrame:
    case OpLookupSource::ProviderParentRaw:
    case OpLookupSource::ProviderInputs:
    case OpLookupSource::ProviderCandidates:
        return host.LookUpSpace(index, lookup, type, value, error);
    case OpLookupSource::AvarBindings:
    case OpLookupSource::CheckpointInputs:
    case OpLookupSource::InterpolatorInputs:
        return host.LookUpCompose(index, lookup, type, value, error);
    }
    return OpLayoutFail(error, "no host resolves lookup source " +
                                   std::to_string(lookup.source));
}

/// Stores \p value through the \p host method of \p output's target
/// family: StorePropertyVersion, StoreVolumePlacement, StoreWeightPacket,
/// StoreWeightField, StoreCommit, StoreRevisionEnabled, StoreConstraint or
/// StoreCompose or StoreSolver(output, value, error).
template <class Host>
bool
OpStoreOutput(Host &host, const OpHostOutput &output, const void *value,
              std::string *error)
{
    switch (OpOutputTarget(output.target)) {
    case OpOutputTarget::PropertyVersion:
        return host.StorePropertyVersion(output, value, error);
    case OpOutputTarget::VolumePlacement:
    case OpOutputTarget::VolumePlacementBase:
        return host.StoreVolumePlacement(output, value, error);
    case OpOutputTarget::WeightPacket:
        return host.StoreWeightPacket(output, value, error);
    case OpOutputTarget::WeightField:
        return host.StoreWeightField(output, value, error);
    case OpOutputTarget::CommitSlots:
    case OpOutputTarget::CommitDescendants:
    case OpOutputTarget::CommitBaseSlots:
    case OpOutputTarget::CommitBaseDescendants:
    case OpOutputTarget::CommitPresent:
        return host.StoreCommit(output, value, error);
    case OpOutputTarget::SolverFrames:
        return host.StoreSolver(output, value, error);
    case OpOutputTarget::RevisionEnabled:
        return host.StoreRevisionEnabled(output, value, error);
    case OpOutputTarget::ConstraintCandidate:
    case OpOutputTarget::ConstraintDelta:
    case OpOutputTarget::ConstraintDeltaMeasured:
    case OpOutputTarget::ConstraintStatus:
    case OpOutputTarget::SingleChainIkFrames:
    case OpOutputTarget::IkFkMatchIkValues:
    case OpOutputTarget::IkFkMatchFkValues:
    case OpOutputTarget::IkFkMatchShown:
        return host.StoreConstraint(output, value, error);
    case OpOutputTarget::ChainGroup:
        break;
    case OpOutputTarget::ProviderFinalMatrix:
    case OpOutputTarget::ProviderBaseMatrix:
    case OpOutputTarget::FrameMatrix:
    case OpOutputTarget::FrameMatrixValid:
    case OpOutputTarget::RefreshBase:
    case OpOutputTarget::RefreshFinal:
    case OpOutputTarget::RefreshCarryBase:
    case OpOutputTarget::RefreshCarryFinal:
    case OpOutputTarget::ProviderValueMatrix:
    case OpOutputTarget::ProviderValueFrame:
    case OpOutputTarget::ProviderValueAuthority:
    case OpOutputTarget::ProviderValueUnavailable:
    case OpOutputTarget::ProviderAnswer:
    case OpOutputTarget::ProviderAnswerAuthority:
    case OpOutputTarget::ProviderAnswerBlocked:
        return host.StoreSpace(output, value, error);
    case OpOutputTarget::Avars:
    case OpOutputTarget::SwitchFrame:
    case OpOutputTarget::PoseWeights:
        return host.StoreCompose(output, value, error);
    }
    return OpLayoutFail(error, "no host stores output target " +
                                   std::to_string(output.target));
}

/// Resolves lookup \p index of \p layout: a List lookup element by element
/// into an array of its length, a Selected lookup as its first present
/// candidate, any other through OpLookUp.
template <class Host>
bool
OpLookUpAt(const OpStepLayout &layout, Host &host, uint32_t index,
           const RigExecTypeDesc &type, void *value, std::string *error)
{
    const OpLookup &lookup = layout.lookups[index];
    if (lookup.source == uint8_t(OpLookupSource::Selected)) {
        for (uint32_t e = 0; e < uint32_t(lookup.part); ++e) {
            const OpLookup &candidate = layout.lookups[lookup.object + e];
            if (candidate.source != uint8_t(OpLookupSource::Matrix) &&
                !host.PresentMatrix(candidate))
                continue;
            return OpLookUp(host, lookup.object + e, candidate, type, value,
                            error);
        }
        return OpLayoutFail(error, "lookup " + std::to_string(index) +
                                       " selects among matrices none of "
                                       "which is present");
    }
    if (lookup.source != uint8_t(OpLookupSource::List))
        return OpLookUp(host, index, lookup, type, value, error);
    const RigExecTypeDesc &element =
        *RigExecRegistry::Get().TypeAt(type.element->Index()).desc;
    type.resize(value, size_t(lookup.part));
    auto *elements = static_cast<uint8_t *>(type.data(value));
    for (uint32_t e = 0; e < uint32_t(lookup.part); ++e)
        if (!OpLookUp(host, lookup.object + e,
                      layout.lookups[lookup.object + e], element,
                      elements + e * element.size, error))
            return false;
    return true;
}

/// Resolves node \p node's Lookup inputs in layout order, storing a
/// hostOutput of a looked-up input before the next lookup.
template <class Host>
bool
ResolveOpLookups(const OpStepLayout &layout, Host &host, OpStepState *state,
                 size_t node, std::string *error)
{
    for (const uint32_t i : layout.lookupsOf[node]) {
        OpPortValue &value = *state->lookedUp[i];
        const uint32_t lookup = layout.hostInputs[i].object;
        if (!OpLookUpAt(layout, host, lookup, value.Type(), value.Value(),
                        error))
            return false;
        for (const uint32_t o : layout.hostOutputsOf[node])
            if (layout.hostOutputs[o].port == layout.hostInputs[i].port &&
                !OpStoreOutput(host, layout.hostOutputs[o], value.Value(),
                               error))
                return false;
    }
    return true;
}

/// Runs op step (\p kind, \p object, \p part): node \p object, after its
/// lookups resolved (ResolveOpLookups) and, for a windowed op, its window
/// lent by \p host.LendWindow(run, node, error), its outputs' hostOutputs
/// stored after it ran; chain \p object's head, after its members' lookups
/// resolved; or group part % G of the segment starting at member part / G
/// of chain \p object, each of the segment's hostOutputs stored for the
/// groups it wrote by \p host.StoreGroup(output, group, elements, count,
/// error). False with the reason in \p error. Steps that do not feed each
/// other may run at once.
template <class Host>
bool
RunOpStep(const OpStepLayout &layout, Host &host, OpStepState *state,
          OpStepKind kind, int object, int part, std::string *error,
          RigExecNodeId *failedNode = nullptr)
{
    RigExecRunState &run = *state->run;
    if (object < 0)
        return OpLayoutFail(error, "an op step names no object");
    const size_t index = size_t(object);
    if (kind == OpStepKind::Node) {
        if (index >= layout.nodes.size())
            return OpLayoutFail(error, "no node " + std::to_string(object));
        const RigExecNodeDesc &d = layout.nodes[index];
        if (!ResolveOpLookups(layout, host, state, index, error))
            return false;
        if (d.op->threading == RigExecOpThreading::Windowed &&
            !host.LendWindow(run, uint32_t(object), error))
            return false;
        if (!run.RunNode(RigExecNodeId(object), error))
            return false;
        for (const uint32_t o : layout.hostOutputsOf[index]) {
            const OpHostOutput &output = layout.hostOutputs[o];
            if (!(d.op->ports[output.port].flags & kRigExecPortOutput))
                continue;
            RigExecValueRef ref;
            ref.value = d.values[output.port];
            const void *value = run.GetValue(ref);
            if (!value)
                return OpLayoutFail(error, "node " + std::to_string(object) +
                                               " holds no value at port " +
                                               std::to_string(output.port));
            if (!OpStoreOutput(host, output, value, error))
                return false;
        }
        // A value whose bytes did not move keeps its version, so its readers
        // do not rerun.
        for (size_t p = 0; p < d.values.size(); ++p) {
            const uint32_t v = d.values[p];
            if (!(d.op->ports[p].flags & kRigExecPortOutput) ||
                v == kRigExecInvalidIndex)
                continue;
            const RigExecTypeDesc &type =
                *RigExecRegistry::Get()
                     .TypeAt(d.op->ports[p].type->Index())
                     .desc;
            RigExecValueRef ref;
            ref.value = v;
            const auto *bytes = static_cast<const uint8_t *>(run.GetValue(ref));
            std::vector<uint8_t> &held = state->valueBytes[v];
            if (type.triviallyCopyable && bytes && held.size() == type.size &&
                std::memcmp(held.data(), bytes, type.size) == 0)
                continue;
            if (type.triviallyCopyable && bytes)
                held.assign(bytes, bytes + type.size);
            ++state->versions[v];
        }
        return true;
    }
    if (index >= layout.chains.size())
        return OpLayoutFail(error, "no chain " + std::to_string(object));
    const OpChainLayout &chain = layout.chains[index];
    const uint32_t groups = chain.Groups();
    const RigExecChainDesc &desc = layout.chainDescs[index];
    if (kind == OpStepKind::ChainHead) {
        // The plan moves, and every range step reruns, only when a member's
        // looked-up input does: range steps read everything else themselves.
        // A keyed element moves the plans of the groups host.LookupGroups
        // names for it.
        bool moved = false;
        for (const RigExecNodeId member : desc.members) {
            if (!ResolveOpLookups(layout, host, state, member, error))
                return false;
            for (const uint32_t i : layout.lookupsOf[member]) {
                const OpPortValue &value = *state->lookedUp[i];
                std::vector<uint8_t> &held = state->lookedUpBytes[i];
                if (layout.keyed[i]) {
                    const OpLookup &list =
                        layout.lookups[layout.hostInputs[i].object];
                    const size_t size = sizeof(float);
                    OpPortValue &weights = *state->lookedUp[i];
                    const auto *bytes = static_cast<const uint8_t *>(
                        weights.Type().data(weights.Value()));
                    const size_t all = size_t(list.part) * size;
                    if (held.size() != all) {
                        held.assign(bytes, bytes + all);
                        moved = true;
                        continue;
                    }
                    for (uint32_t e = 0; e < uint32_t(list.part); ++e) {
                        if (std::memcmp(held.data() + e * size,
                                        bytes + e * size, size) == 0)
                            continue;
                        std::memcpy(held.data() + e * size, bytes + e * size,
                                    size);
                        const uint32_t *reached = nullptr;
                        size_t count = 0;
                        host.LookupGroups(layout.lookups[list.object + e],
                                          &reached, &count);
                        for (size_t r = 0; r < count; ++r)
                            if (reached[r] < groups)
                                ++state->versions[layout.PlanGroupSlot(
                                    index, reached[r])];
                    }
                    continue;
                }
                const auto *bytes = static_cast<const uint8_t *>(value.Value());
                const size_t size = value.Type().size;
                if (!value.Type().triviallyCopyable || held.size() != size ||
                    std::memcmp(held.data(), bytes, size) != 0) {
                    moved = true;
                    if (value.Type().triviallyCopyable)
                        held.assign(bytes, bytes + size);
                }
            }
        }
        if (state->versions[layout.PlanSlot(index)] == 0)
            moved = true;
        if (!run.RunChainHead(index, chain.bounds.data(), groups, error,
                              failedNode))
            return false;
        for (const uint32_t i : layout.groupedOf[index]) {
            const uint32_t node = layout.hostInputs[i].node;
            if (layout.grouped[i] != 2)
                continue;
            state->haloReads[i].resize(groups);
            for (uint32_t g = 0; g < groups; ++g) {
                size_t lo = 0, hi = 0;
                run.ChainReads(index, layout.memberOf[node], g, &lo, &hi);
                std::vector<uint32_t> &reads = state->haloReads[i][g];
                reads.clear();
                for (uint32_t h = 0; h < groups; ++h)
                    if (h == g || (size_t(chain.bounds[h]) < hi &&
                                   size_t(chain.bounds[h + 1]) > lo))
                        reads.push_back(h);
            }
        }
        if (moved)
            ++state->versions[layout.PlanSlot(index)];
        return true;
    }
    if (part < 0 || !groups)
        return OpLayoutFail(error, "an op range step names no group");
    const uint32_t segment = uint32_t(part) / groups;
    const uint32_t g = uint32_t(part) % groups;
    for (const uint32_t i : layout.groupedOf[index]) {
        const OpHostInput &input = layout.hostInputs[i];
        if (desc.segments[layout.memberOf[input.node]] != segment)
            continue;
        if (layout.grouped[i] == 1) {
            const bool whole = desc.whole[layout.memberOf[input.node]] != 0;
            for (uint32_t h = whole ? 0 : g; h < (whole ? groups : g + 1); ++h)
                state->groupPoints[i][h] =
                    OpGroupPoints(layout, host, input, h);
            continue;
        }
        std::vector<const void *> &table = state->haloPoints[i][g];
        std::fill(table.begin(), table.end(), nullptr);
        if (desc.whole[layout.memberOf[input.node]]) {
            for (uint32_t h = 0; h < groups; ++h)
                table[h] = OpGroupPoints(layout, host, input, h);
            continue;
        }
        table[g] = OpGroupPoints(layout, host, input, g);
        if (g < state->haloReads[i].size())
            for (const uint32_t h : state->haloReads[i][g])
                table[h] = OpGroupPoints(layout, host, input, h);
    }
    if (!run.RunChainRange(index, segment, g, error, failedNode))
        return false;
    for (uint32_t m = segment;
         m < desc.members.size() && desc.segments[m] == segment; ++m) {
        const RigExecNodeDesc &d = layout.nodes[desc.members[m]];
        for (const uint32_t o : layout.hostOutputsOf[desc.members[m]]) {
            const OpHostOutput &output = layout.hostOutputs[o];
            if (output.target != uint8_t(OpOutputTarget::ChainGroup))
                continue;
            RigExecValueRef ref;
            ref.value = d.values[output.port];
            for (uint32_t h = desc.whole[m] ? 0 : g;
                 h < (desc.whole[m] ? groups : g + 1); ++h) {
                if (!chain.Writes(m, h))
                    continue;
                size_t count = 0;
                const void *elements = run.GetGroupValue(ref, h, &count);
                if (!elements && count)
                    return OpLayoutFail(
                        error, "node " + std::to_string(desc.members[m]) +
                                   " holds no group " + std::to_string(h) +
                                   " at port " + std::to_string(output.port));
                if (!host.StoreGroup(output, h, elements, count, error))
                    return false;
            }
        }
        for (size_t p = 0; p < d.values.size(); ++p) {
            if (!(d.op->ports[p].flags & kRigExecPortOutput) ||
                d.values[p] == kRigExecInvalidIndex)
                continue;
            const uint32_t base = layout.groupBase[d.values[p]];
            if (desc.whole[m])
                for (uint32_t h = 0; h < groups; ++h)
                    ++state->groupVersions[base + h];
            else
                ++state->groupVersions[base + g];
        }
    }
    return true;
}

} // namespace eval
} // namespace rigExec

#endif
