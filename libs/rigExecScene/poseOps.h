#ifndef RIGEXEC_SCENE_POSE_OPS_H
#define RIGEXEC_SCENE_POSE_OPS_H

/// Host-side bridge from a solver or constraint record to rigExecOps' pose ops:
/// the record's settings become the op's payload and the frame's values its
/// inputs. The scene program runs its solvers and constraints through it.

#include "rigExec/api/evaluator.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecMath/autoClavicleKernel.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecMath/ikFkMatchKernel.h"
#include "rigExecScene/constraintProgram.h"
#include "rigExecScene/poseProgram.h"
#include "rigExecScene/solverProgram.h"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rigExec {

/// A solver's op nodes and their host-written ports. Ports the op lacks
/// keep an invalid id.
struct RigExecSolverOpNode {
    RigExecSolverKind kind = RigExecSolverKind::FkChain;
    RigExecInputId controls, outRests;
    RigExecInputId root, effector, pole, rests, space, bend, upperOffset,
        lowerOffset, stretch, softness, pin, upperScale, lowerScale,
        softDistance, twist;
    RigExecInputId start, end, startRest, endRest, weights, turns;
    RigExecInputId posed, restCurve, jointRests, jointRestLive;
    RigExecInputId restJoints, restRoot, restMid, restEnd, mid, volumeWeights,
        preserveVolume, midFollow, roll;
    RigExecInputId startPose, externalRests, externalPoses, channels, selectors,
        groupSelectors;
    RigExecOutputId out, result;
    /// A frame blend: one node per element.
    struct Blend {
        RigExecInputId a, b, rest, weight, outRest;
        RigExecOutputId out;
    };
    std::vector<Blend> blend;
    /// The settings baked into the payloads and the node layout, to refuse
    /// a frame whose settings moved.
    std::vector<int> parents;
    bool scaleSegments = false, softDistancePolicy = false;
    double scaleCalibration = 0.0, minLengthRatio = 0.0;
    bool restLengthChain = false, aimRootTangent = false, logScale = false;
    std::vector<char> blendLive;
    int sampleCount = 0;
    bool hasStart = false;
    std::vector<uint32_t> bone2dOrder;
    std::vector<RigExecBone2dSetup> bone2dSetup;
    std::vector<int32_t> spaceSources, targetCandidates, groupMembers;
    std::vector<RigExecJointGroup> groups;
};

/// True for the solver records this bridge maps. False for a degenerate
/// record, an FK chain with joint groups, a frame blend whose rotation blend
/// was refused and a spline IK without joints.
bool RigExecSolverOpSupported(const RigExecSolverRecord &record);

/// Host-side port values, kept between frames so a write reuses their
/// storage.
class RigExecPoseOpScratch {
public:
    RigExecPoseOpScratch();
    ~RigExecPoseOpScratch();
    RigExecPoseOpScratch(RigExecPoseOpScratch &&) noexcept;
    RigExecPoseOpScratch &operator=(RigExecPoseOpScratch &&) noexcept;

    struct Values;
    std::unique_ptr<Values> values;
};

/// How a solver frame is answered before any op runs.
enum class RigExecSolverOpAnswer {
    /// Run the op nodes.
    Run,
    /// \p output holds the answer, reached with no arithmetic: a frame
    /// blend's pass-through of one input or its empty answer.
    Done,
    /// Native's own refusal: \p output is empty and \p error says why.
    Failed,
};

/// Answers \p input's frame where no op needs to run.
RigExecSolverOpAnswer RigExecAnswerSolverOp(const RigExecSolverRecord &record,
                                            const RigExecSolverInputs &input,
                                            RigExecPointFrameArray *output,
                                            std::string *error = nullptr);

/// Adds \p record's op nodes to \p builder, the payloads taken from
/// \p record and \p input; a frame blend gets one node per element of
/// \p input's first aggregate. False with \p error for a record the bridge
/// does not map.
bool RigExecAddSolverOp(const RigExecSolverRecord &record,
                        const RigExecSolverInputs &input,
                        RigExecRigDescBuilder *builder,
                        RigExecSolverOpNode *node,
                        std::string *error = nullptr);

/// Writes \p input's values for one frame into \p evaluator. False,
/// writing nothing, where a setting baked into the nodes differs from
/// \p record and \p input: the nodes must then be rebuilt. Requires the
/// scene's refreshed two-bone IK parameters (refreshIkParams).
bool RigExecWriteSolverOpInputs(const RigExecSolverRecord &record,
                                const RigExecSolverInputs &input,
                                const RigExecSolverOpNode &node,
                                RigExecPoseOpScratch *scratch,
                                RigExecEvaluator *evaluator,
                                std::string *error = nullptr);

/// Reads the evaluated nodes into \p output, as RigExecRunSolver gives it,
/// with its validity in \p valid and, where invalid, its reason in
/// \p error. False where the nodes have no output.
bool RigExecReadSolverOpOutput(const RigExecSolverRecord &record,
                               const RigExecSolverInputs &input,
                               const RigExecSolverOpNode &node,
                               const RigExecEvaluator &evaluator,
                               RigExecPointFrameArray *output, bool *valid,
                               std::string *error = nullptr);

/// A constraint's op node and its host-written ports.
struct RigExecConstraintOpNode {
    RigExecConstraintKind kind = RigExecConstraintKind::Position;
    RigExecInputId in, sources, sourceWeights, weight, space, target, worldUp;
    RigExecInputId chain, effector, pole, twist, stretch;
    RigExecOutputId out;
    /// The record and Parent offsets the payload was built from, and the
    /// bound ports, to refuse a frame whose settings moved.
    RigExecConstraintRecord settings;
    std::vector<GfVec3d> translationOffsets, rotationOffsets;
    bool hasSpace = false, hasWorldUp = false;
};

/// True for the constraint kinds RigExecAddConstraintOp maps: every kind but
/// IkFkMatch and Harmonics, which have bridges of their own below.
bool RigExecConstraintOpSupported(RigExecConstraintKind kind);

/// Adds \p record's op node, its payload from \p record and \p input.
bool RigExecAddConstraintOp(const RigExecConstraintRecord &record,
                            const RigExecConstraintInputs &input,
                            RigExecRigDescBuilder *builder,
                            RigExecConstraintOpNode *node,
                            std::string *error = nullptr);

/// Writes one frame's values. False, writing nothing more, where a setting
/// baked into the node differs: the node must then be rebuilt.
bool RigExecWriteConstraintOpInputs(const RigExecConstraintRecord &record,
                                    const RigExecConstraintInputs &input,
                                    const RigExecConstraintOpNode &node,
                                    RigExecPoseOpScratch *scratch,
                                    RigExecEvaluator *evaluator,
                                    std::string *error = nullptr);

/// Reads the evaluated node into \p result, as RigExecRunConstraint gives it.
bool RigExecReadConstraintOpOutput(const RigExecConstraintRecord &record,
                                   const RigExecConstraintInputs &input,
                                   const RigExecConstraintOpNode &node,
                                   const RigExecEvaluator &evaluator,
                                   RigExecConstraintResult *result);

/// One frame of an auto clavicle as RigExecRunAutoClavicle reads it: its
/// frames as matrices in RigExecBoundAutoClavicle::frames order, null where
/// unbound, and its scalars with their fallbacks applied.
struct RigExecAutoClavicleOpInputs {
    std::array<const GfMatrix4d *, 13> frames{};
    double ikBlend = 0.0, amount = 1.0;
    bool hasLimb = false;
    double stretch = 1.0, pin = 0.0, upperScale = 1.0, lowerScale = 1.0,
           softDistance = 0.0, scaleCalibration = 0.0, twistDegrees = 0.0,
           upperOffset = 0.0, lowerOffset = 0.0;
};

/// An auto clavicle's op node and its host-written ports.
struct RigExecAutoClavicleOpNode {
    RigExecInputId target, pivot, anchor, anchorDefault, fkPosed, fkDefault,
        ikTarget, pole, ikBlend, amount, limbRests, stretch, pin, upperScale,
        lowerScale, softDistance, scaleCalibration, twist, upperOffset,
        lowerOffset;
    RigExecOutputId delta, out;
    /// The settings baked into the payload and the bound ports.
    RigExecAutoClavicleConstants constants;
    bool hasLimb = false, hasIkTarget = false, hasPole = false;
};

/// Adds the auto clavicle's op node, its payload from \p constants and the
/// limb and optional frames \p input binds.
bool RigExecAddAutoClavicleOp(const RigExecAutoClavicleConstants &constants,
                              const RigExecAutoClavicleOpInputs &input,
                              RigExecRigDescBuilder *builder,
                              RigExecAutoClavicleOpNode *node,
                              std::string *error = nullptr);

/// Writes one frame's values. False, writing nothing, where a setting
/// baked into the node differs: the node must then be rebuilt.
bool RigExecWriteAutoClavicleOpInputs(
    const RigExecAutoClavicleConstants &constants,
    const RigExecAutoClavicleOpInputs &input,
    const RigExecAutoClavicleOpNode &node, RigExecPoseOpScratch *scratch,
    RigExecEvaluator *evaluator, std::string *error = nullptr);

/// The shift RigExecAutoClavicleShift gives, and the target frame moved by
/// it. False where the node has no output.
bool RigExecReadAutoClavicleOpOutput(const RigExecAutoClavicleOpNode &node,
                                     const RigExecEvaluator &evaluator,
                                     double delta[3], GfMatrix4d *shifted);

/// A harmonics constraint's settings, baked into its op's payload.
struct RigExecHarmonicsOpConstants {
    RigExecHarmonicsSettings settings;
    bool matchVelocity = true;
    /// The window's earlier ticks; 0 without a window.
    uint32_t frames = 0;
};

/// One frame of a harmonics constraint, read and gated by the host: a
/// space that could not be read is a zero matrix, and a gate the host
/// closed (a failed read, a weight out of range) is a disabled constraint.
struct RigExecHarmonicsOpInputs {
    RigExecPointFrame incoming;
    std::optional<GfMatrix4d> space;
    bool enabled = true;
    double weight = 1.0, amplitude = 1.0, decay = 3.0, termination = 0.0;
    GfVec3d axisAmplitude = GfVec3d(1.0);
};

/// A harmonics constraint's op node and its host-written ports.
struct RigExecHarmonicsOpNode {
    RigExecInputId in, space, enabled, weight, amplitude, decay, termination,
        axisAmplitude;
    RigExecOutputId out, status;
    RigExecHarmonicsOpConstants constants;
    bool hasSpace = false;
};

bool RigExecAddHarmonicsOp(const RigExecHarmonicsOpConstants &constants,
                           const RigExecHarmonicsOpInputs &input,
                           RigExecRigDescBuilder *builder,
                           RigExecHarmonicsOpNode *node,
                           std::string *error = nullptr);

/// Writes one frame's values. False, writing nothing, where a setting
/// baked into the node differs: the node must then be rebuilt.
bool RigExecWriteHarmonicsOpInputs(const RigExecHarmonicsOpConstants &constants,
                                   const RigExecHarmonicsOpInputs &input,
                                   const RigExecHarmonicsOpNode &node,
                                   RigExecEvaluator *evaluator,
                                   std::string *error = nullptr);

/// The op's frame and its rigExecOps::HarmonicsStatus. False where the node
/// has no output.
bool RigExecReadHarmonicsOpOutput(const RigExecHarmonicsOpNode &node,
                                  const RigExecEvaluator &evaluator,
                                  RigExecPointFrame *frame, uint32_t *status);

/// An IK/FK match's settings, baked into its op's payload: the pole's space
/// switch and which of its sources ride the IK control.
struct RigExecIkFkMatchOpConstants {
    RigExecSpaceSwitchRecord poleRecord;
    bool poleHasCarry = false;
    std::vector<char> poleWorld, poleSourceRides;
    double ikValue = 1, scaleCalibration = 0, upperLengthOffset = 0,
           lowerLengthOffset = 0;
    bool stretchLimb = true;
};

/// One frame of an IK/FK match as RunIkFkMatch reads it.
struct RigExecIkFkMatchOpInputs {
    double weight = 1.0;
    /// Switch, stretch, softDistance, upperScale, lowerScale, pin, twist.
    std::array<double, 7> scalars{};
    std::array<GfMatrix4d, 10> sources;
    std::array<GfMatrix4d, 8> rests;
    std::array<RigExecIkFkMatchChannels, 5> channels;
    bool hasSpace = false;
    GfMatrix4d spaceRest = GfMatrix4d(1.0), spacePosed = GfMatrix4d(1.0);
    /// A rigExecOps::IkFkPoleMode, and the pole switch's inputs it reads.
    uint32_t poleMode = 0;
    RigExecSpaceSwitchInputs poleSwitch;
};

/// The op's results: the published values, and the frames presented.
struct RigExecIkFkMatchOpOutputs {
    std::array<double, 11> ikValues{};
    std::array<double, 20> fkValues{};
    bool valid = false;
    std::array<GfMatrix4d, 5> shown;
    unsigned shownMask = 0;
};

/// An IK/FK match's op node and its host-written ports.
struct RigExecIkFkMatchOpNode {
    RigExecInputId enabled, weight, sources, rests, channels, hasSpace,
        spaceRest, spacePosed, poleMode, posedDefault, parentDefaultInverse,
        parentPosed, parentDefault, spaceDefault, switchSpacePosed,
        sourceDefaults, sourcePosed, active;
    std::array<RigExecInputId, 7> scalars;
    RigExecOutputId ikValues, fkValues, valid, shown, shownMask;
    RigExecIkFkMatchOpConstants constants;
};

bool RigExecAddIkFkMatchOp(const RigExecIkFkMatchOpConstants &constants,
                           RigExecRigDescBuilder *builder,
                           RigExecIkFkMatchOpNode *node,
                           std::string *error = nullptr);

/// Writes one frame's values. False, writing nothing, where a setting
/// baked into the node differs: the node must then be rebuilt.
bool RigExecWriteIkFkMatchOpInputs(const RigExecIkFkMatchOpConstants &constants,
                                   const RigExecIkFkMatchOpInputs &input,
                                   const RigExecIkFkMatchOpNode &node,
                                   RigExecPoseOpScratch *scratch,
                                   RigExecEvaluator *evaluator,
                                   std::string *error = nullptr);

bool RigExecReadIkFkMatchOpOutput(const RigExecIkFkMatchOpNode &node,
                                  const RigExecEvaluator &evaluator,
                                  RigExecIkFkMatchOpOutputs *output);

} // namespace rigExec

#endif
