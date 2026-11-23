#ifndef RIGEXEC_SCENE_POSE_OPS_H
#define RIGEXEC_SCENE_POSE_OPS_H

/// Host-side bridge from a solver or constraint record to rigExecOps' pose
/// ops: the record's settings become the op's payload and the frame's
/// values its inputs. The scene program runs its solvers and constraints
/// through it; step 10 moves it to rigExecUsd.

#include "rigExec/api/evaluator.h"
#include "rigExec/api/rigDesc.h"
#include "rigExecScene/constraintProgram.h"
#include "rigExecScene/solverProgram.h"

#include <memory>
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
    RigExecOutputId out;
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
};

/// True for the solver records this bridge maps. The others, and a
/// degenerate record, run natively: Skeleton2d, an FK chain with joint
/// groups, a frame blend whose rotation blend was refused and a spline IK
/// without joints.
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
/// scene's refreshed parameters (refreshIkParams, refreshSplineParams).
bool RigExecWriteSolverOpInputs(const RigExecSolverRecord &record,
                                const RigExecSolverInputs &input,
                                const RigExecSolverOpNode &node,
                                RigExecPoseOpScratch *scratch,
                                RigExecEvaluator *evaluator,
                                std::string *error = nullptr);

/// Reads the evaluated nodes into \p output, as RigExecRunSolver gives it.
bool RigExecReadSolverOpOutput(const RigExecSolverRecord &record,
                               const RigExecSolverInputs &input,
                               const RigExecSolverOpNode &node,
                               const RigExecEvaluator &evaluator,
                               RigExecPointFrameArray *output);

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

/// True for the constraint kinds this bridge maps: every kind but IkFkMatch
/// and Harmonics.
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

} // namespace rigExec

#endif
