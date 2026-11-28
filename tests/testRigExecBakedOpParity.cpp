// Each baked step family that already has an op, replayed through that op on
// the inputs the production step body read, after the evaluator ran real rigs.
// The op's answer is compared bit for bit with what the body published,
// including what a refusal publishes. A mismatch is reported with its first
// differing value; a family or kind the op cannot take yet is counted as not
// probed, with the reason.
#include "rigExec/api/goldenPose.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/crossDomainInputs.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExec/internal/poseEvalHost.h"
#include "rigExecScene/constraintSceneTranslation.h"
#include "rigExecScene/geometrySceneTranslation.h"
#include "rigExecScene/sceneDescriptors.h"
#include "rigExecScene/usdSceneAccess.h"
#include "rigExecScene/externalOps.h"
#include "rigExecGraph/windowHistory.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecStdTypes/stdTypes.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/evaluator.h"
#include "rigExec/api/rigDesc.h"
#include "rigExec/api/taskSystem.h"
#include "rigExecMath/poseCommitKernel.h"
#include "rigExecOps/solver/poseCommitOps.h"
#include "rigExecOps/constraint/matrixMoverOps.h"
#include "rigExecOps/constraint/matchOps.h"
#include "rigExecMath/harmonicsKernel.h"
#include "rigExecOps/deform/skinOp.h"
#include "rigExecMath/skinKernel.h"
#include "rigExecOps/pose/poseOps.h"
#include "rigExecOps/deform/matrixDeformOp.h"
#include "opRunners/geometryOpRunner.h"
#include "opRunners/poseOpRunner.h"
#include "opRunners/weightOpRunner.h"
#include "rigExecOps/space/providerOps.h"
#include "rigExecOps/space/spaceOps.h"
#include "rigExecRigTypes/rigTypes.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

namespace {

using B = RigExecBakedProgramImpl;

/// compared: applied results compared bit for bit (refusals: of them, ones
/// both sides answered with a pass-through). compileErrors: the baked step
/// refused and the op failed compile, the evaluation that binds its inputs
/// or the bridge that builds it. runErrors: the baked step refused and the
/// op's evaluation stopped with an error while it ran.
struct Tally {
    size_t compared = 0, refusals = 0, compileErrors = 0, runErrors = 0,
           mismatched = 0;
};

struct Finding {
    std::string key, object, text;
};

std::map<std::string, Tally> g_tally;
std::map<std::string, size_t> g_unprobed;
std::vector<Finding> g_findings;
std::string g_where;

// Per revision kind, the envelope shapes its comparisons ran under.
std::map<std::string, std::map<std::string, size_t>> g_envelopes;

void
Report(const std::string &key, const std::string &object,
       const std::string &text)
{
    ++g_tally[key].mismatched;
    g_findings.push_back(
        {key, object, key + " " + g_where + " " + object + ": " + text});
}

std::string
Encode(const VtValue &value)
{
    std::string text, error;
    if (!RigExecEncodeGoldenValue(value, &text, &error))
        return "unencodable " + error;
    return text;
}

std::vector<std::string>
Tokens(const std::string &text)
{
    std::vector<std::string> out;
    std::istringstream in(text);
    for (std::string token; in >> token;)
        out.push_back(token);
    return out;
}

std::string
FirstDifference(const std::string &baked, const std::string &op)
{
    const auto a = Tokens(baked), b = Tokens(op);
    size_t i = 0;
    while (i < a.size() && i < b.size() && a[i] == b[i])
        ++i;
    const auto at = [](const std::vector<std::string> &t, size_t k) {
        return k < t.size() ? t[k] : std::string("<end>");
    };
    std::string context;
    for (size_t k = i > 3 ? i - 3 : 0; k < i; ++k)
        context += a[k] + " ";
    return "token " + std::to_string(i) + " of " + std::to_string(a.size()) +
           "/" + std::to_string(b.size()) + " after [" + context + "]: baked " +
           at(a, i) + " op " + at(b, i);
}

void
Unprobed(const std::string &family, const std::string &kind,
         const std::string &reason)
{
    ++g_unprobed[family + "/" + kind + ": " + reason];
}

void
Compare(const std::string &family, const std::string &kind,
        const std::string &object, const VtValue &baked, const VtValue &op,
        bool refusal, const std::string &note = std::string())
{
    Tally &tally = g_tally[family + "/" + kind];
    ++tally.compared;
    if (refusal)
        ++tally.refusals;
    const std::string a = Encode(baked), b = Encode(op);
    if (a == b)
        return;
    Report(family + "/" + kind, object,
           FirstDifference(a, b) +
               (note.empty() ? std::string() : " (" + note + ")"));
}

/// An op that failed: counted where the baked step refused, a finding where
/// it applied.
void
OpFailed(const std::string &family, const std::string &kind,
         const std::string &object, bool bakedRefused, const std::string &error)
{
    Tally &tally = g_tally[family + "/" + kind];
    if (!bakedRefused) {
        Report(family + "/" + kind, object,
               "the op failed where the baked step applied: " + error);
        return;
    }
    if (error.find(") failed: ") != std::string::npos)
        ++tally.runErrors;
    else
        ++tally.compileErrors;
}

void
CompareFlag(const std::string &family, const std::string &kind,
            const std::string &object, const char *what, bool baked, bool op)
{
    if (baked == op)
        return;
    Report(family + "/" + kind, object,
           std::string(what) + " baked " + (baked ? "1" : "0") + " op " +
               (op ? "1" : "0"));
}

std::vector<GfVec3f>
VersionPoints(const B::GeomChain &chain, size_t version)
{
    std::vector<GfVec3f> scratch;
    const GfVec3f *points = nullptr;
    size_t count = 0;
    RigExecBakedVersionPoints(chain, version, &scratch, &points, &count);
    return std::vector<GfVec3f>(points, points + count);
}

VtValue
Points(const std::vector<GfVec3f> &points)
{
    return VtValue(VtVec3fArray(points.begin(), points.end()));
}

const char *
OpName(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return "Matrix";
    case RigExecRevisionOp::Skin:
        return "Skin";
    case RigExecRevisionOp::BlendShape:
        return "BlendShape";
    case RigExecRevisionOp::VolumeCorrect:
        return "VolumeCorrect";
    case RigExecRevisionOp::Smooth:
        return "Smooth";
    case RigExecRevisionOp::Lattice:
        return "Lattice";
    case RigExecRevisionOp::SurfaceProject:
        return "SurfaceProject";
    case RigExecRevisionOp::Ribbon:
        return "Ribbon";
    case RigExecRevisionOp::Wire:
        return "Wire";
    case RigExecRevisionOp::EmitGuidePoints:
        return "EmitGuidePoints";
    case RigExecRevisionOp::RecomputeNormals:
        return "RecomputeNormals";
    case RigExecRevisionOp::RecomputeExtent:
        return "RecomputeExtent";
    case RigExecRevisionOp::DeltaMush:
        return "DeltaMush";
    case RigExecRevisionOp::Wrinkle:
        return "Wrinkle";
    case RigExecRevisionOp::External:
        return "External";
    case RigExecRevisionOp::SurfaceProjector:
        return "SurfaceProjector";
    case RigExecRevisionOp::ShaderDials:
        return "ShaderDials";
    }
    return "Unknown";
}

std::string
TypeName(const TfToken &type)
{
    return type.IsEmpty() ? std::string("untyped") : type.GetString();
}

// The inputs a Solve step reads, rebuilt from the program: the frame
// versions it reads, its leaves, the aggregates it blends and its driver
// points as the step resolves them.
RigExecSolverInputs
SolveInputs(const B &b, const B::Solver &s)
{
    const auto rd = [&b](const auto &input) {
        return RigExecBakedLeafRead(b, input);
    };
    const auto live = [&b](const auto &input) {
        const auto flagged = [&input](const std::vector<char> &flags) {
            return input.overrideIndex >= 0 &&
                   size_t(input.overrideIndex) < flags.size() &&
                   flags[size_t(input.overrideIndex)];
        };
        return input.varying || flagged(b.overridden) || flagged(b.upstreamOn);
    };
    const auto frame = [&b](int provider, uint32_t version) {
        return provider >= 0 ? b.fin[size_t(version)] : RigExecPointFrame();
    };
    RigExecSolverInputs input;
    input.hasSpace = s.spaceSlot >= 0;
    if (input.hasSpace) {
        input.spaceRest = s.spaceRest;
        input.space = b.fin[size_t(s.spaceRead)];
    }
    input.root = frame(s.root, s.rootRead);
    input.mid = frame(s.mid, s.midRead);
    input.end = frame(s.end, s.endRead);
    input.pole = frame(s.pole, s.poleRead);
    input.start = frame(s.start, s.startRead);
    for (size_t k = 0; k < s.controls.size(); ++k)
        input.controls.push_back(b.fin[size_t(s.controlReads[k])]);
    switch (s.kind) {
    case RigExecSolverKind::FkChain:
        break;
    case RigExecSolverKind::TwoBoneIk:
        input.ikSpace = rd(s.ikSpace);
        input.refreshIkParams = true;
        input.pin = rd(s.pin);
        input.softDistance = rd(s.softDistance);
        input.limbTwist = rd(s.limbTwist);
        input.upperScale = rd(s.upperScale);
        input.lowerScale = rd(s.lowerScale);
        input.bend = rd(s.bend);
        input.stretch = rd(s.stretch);
        input.softness = rd(s.softness);
        input.upperOffset = rd(s.upperOffset);
        input.lowerOffset = rd(s.lowerOffset);
        break;
    case RigExecSolverKind::BlendPointFrames:
        input.blendA = s.inA >= 0 ? &b.aggregates[size_t(s.inA)] : nullptr;
        input.blendB = s.inB >= 0 ? &b.aggregates[size_t(s.inB)] : nullptr;
        input.blendWeight = rd(s.blendWeight);
        break;
    case RigExecSolverKind::TwistDistribution:
        input.twistTurns = rd(s.twistTurns);
        break;
    case RigExecSolverKind::Ribbon: {
        const GfVec3f *points = nullptr;
        size_t count = 0;
        if (RigExecBakedResolvePoints(b, s.ribbonPointsBinding, &points,
                                      &count))
            input.ribbonPoints.assign(points, points + count);
        else
            input.ribbonPoints =
                s.ribbonPointsVarying ? s.ribbonPoints : s.ribbonConstantPoints;
        break;
    }
    case RigExecSolverKind::SplineIk:
        input.refreshSplineParams = s.splineParamsVary ||
                                    live(s.preserveVolume) ||
                                    live(s.midFollowWeight) || live(s.roll) ||
                                    live(s.twist) || live(s.minLengthRatio);
        input.preserveVolume = rd(s.preserveVolume);
        input.midFollowWeight = rd(s.midFollowWeight);
        input.rollDegrees = rd(s.roll);
        input.twistDegrees = rd(s.twist);
        input.minLengthRatio = rd(s.minLengthRatio);
        break;
    case RigExecSolverKind::Skeleton2d: {
        const size_t bones =
            std::min(s.bone2dDoubles.size() / kRigExecBone2dDoubleChannels,
                     s.bone2dBools.size() / kRigExecBone2dBoolChannels);
        input.bone2dChannels.resize(bones);
        for (size_t k = 0; k < bones; ++k) {
            const auto *d = &s.bone2dDoubles[k * kRigExecBone2dDoubleChannels];
            const auto *f = &s.bone2dBools[k * kRigExecBone2dBoolChannels];
            for (size_t c = 0; c < kRigExecBone2dChannelCount; ++c) {
                const size_t at = RigExecBone2dChannelStorage(c);
                RigExecBone2dSetChannel(&input.bone2dChannels[k], c,
                                        RigExecBone2dChannelIsBool(c)
                                            ? (rd(f[at]) ? 1.0 : 0.0)
                                            : rd(d[at]));
            }
        }
        for (const auto &selector : s.bone2dSelectors)
            input.bone2dSelectors.push_back(rd(selector));
        break;
    }
    }
    for (const auto &selector : s.jointGroupSelectors)
        input.jointGroupSelectors.push_back(rd(selector));
    return input;
}

// Solve: the record and the inputs rebuilt from the program, against the
// aggregate the step published and the diagnostic it emitted.
void
ProbeSolve(const B &b)
{
    for (const auto &step : b.steps) {
        const RigExecBakedStepIdentity id = RigExecBakedIdentityOf(b, step);
        if (id.kind != RigExecBakedStepKind::Solve)
            continue;
        const auto &s = b.solvers[size_t(id.object)];
        const std::string kind =
            TypeName(s.type) + (s.jointGroups.empty() ? "" : " groups");
        if (!RigExecSolverOpSupported(s)) {
            Unprobed("Solve", kind,
                     "RigExecSolverOpSupported refuses the record");
            continue;
        }
        RigExecPointFrameArray out;
        std::string error;
        bool ran = false;
        {
            // The solver's op nodes against its reference kernels, over the
            // rests composed this run.
            B::Solver composed = s;
            for (size_t k = 0; k < s.controls.size(); ++k)
                composed.controlRests[k] = b.restPts[size_t(s.controls[k])];
            if (s.start >= 0)
                composed.startRest = b.restPts[size_t(s.start)];
            composed.hasStart = s.start >= 0;
            for (size_t k = 0; k < s.restRefs.size(); ++k) {
                const int element = s.restRefs[k].second;
                if (s.type == SchemaTypeName::RigExecTwoBoneIk &&
                    element >= 0 && element < 3)
                    composed.ikRests[size_t(element)] =
                        k < s.restIsLive.size() && s.restIsLive[k]
                            ? b.fin[s.restReads[k]].points
                            : b.restPts[size_t(s.restRefs[k].first)];
            }
            if (s.spaceSlot >= 0)
                composed.spaceRest = b.restPts[size_t(s.spaceSlot)];
            if (s.type == SchemaTypeName::RigExecSplineIk) {
                std::vector<RigExecPointFrame> restJoints(s.splineCount);
                for (size_t k = 0; k < s.restRefs.size(); ++k) {
                    const auto [slot, element] = s.restRefs[k];
                    if (element < 0 || size_t(element) >= s.splineCount)
                        continue;
                    const bool live =
                        k < s.restIsLive.size() && s.restIsLive[k];
                    restJoints[size_t(element)] =
                        live ? b.fin[s.restReads[k]]
                             : b.restFrames[size_t(slot)];
                    composed.splineJointRests[size_t(element)] =
                        live ? b.fin[s.restReads[k]].points
                             : b.restPts[size_t(slot)];
                }
                const auto restFrame = [&](int slot) {
                    return slot >= 0 ? b.restFrames[size_t(slot)]
                                     : RigExecPointFrame();
                };
                composed.splineRestFrames = restJoints;
                composed.splineRootRest = restFrame(s.root);
                composed.splineMidRest = restFrame(s.mid);
                composed.splineEndRest = restFrame(s.end);
                composed.splineRest = RigExecSplineIkMakeRest(
                    restJoints, composed.splineRootRest, composed.splineMidRest,
                    composed.splineEndRest, s.splineRestWeights,
                    s.splineRestMode);
            }
            if (s.type == SchemaTypeName::RigExecTwistDistribution) {
                composed.twistStartRest = b.restPts[size_t(s.root)];
                composed.twistEndRest = b.restPts[size_t(s.end)];
            }
            for (size_t k = 0; k < s.restRefs.size() && k < s.jointRests.size();
                 ++k)
                composed.jointRests[k] =
                    k < s.restIsLive.size() && s.restIsLive[k]
                        ? b.fin[s.restReads[k]].points
                        : b.restPts[size_t(s.restRefs[k].first)];
            RigExecSolverWorkspace workspace;
            ran = RigExecReferenceSolve(composed, SolveInputs(b, composed),
                                        &workspace, &out, &error);
        }
        const auto &baked = b.aggregates[size_t(id.object)];
        const std::string prefix = "diag " + s.pathText + ": ";
        std::string bakedError;
        for (const auto &line : step.diagnostics)
            if (line.compare(0, prefix.size(), prefix) == 0)
                bakedError = line.substr(prefix.size());
        if (!ran) {
            OpFailed("Solve", kind, s.pathText,
                     baked.frames.empty() || !bakedError.empty(), error);
            continue;
        }
        Compare("Solve", kind, s.pathText, VtValue(baked), VtValue(out),
                baked.frames.empty());
        if (!bakedError.empty())
            Report("Solve/" + kind, s.pathText,
                   "diagnostic baked \"" + bakedError + "\" where the op ran");
    }
}

// The value the constraint body read for the parameter \p name, from the
// leaves, the compiled arrays and the weight field it resolved.
VtValue
ConstraintParameter(const B &b, const B::Constraint &c,
                    const RigExecBakedConstraintArray *arrays, const char *name)
{
    const std::string n = name;
    const auto is = [&n](const char *attribute) { return n == attribute; };
    const auto leaf = [&b](const auto &input) {
        return VtValue(RigExecBakedLeafRead(b, input));
    };
    const bool parent = c.type == SchemaTypeName::RigExecParentConstraint;
    if (is(SchemaAttrText::inputsEnabled))
        return leaf(c.enabled);
    if (is(SchemaAttrText::inputsDefaultWeight)) {
        if (!c.weightObject.IsEmpty() && c.pointsTarget.IsEmpty())
            return VtValue(b.weightFields[size_t(c.weightField)].values[0]);
        return leaf(c.defaultWeight);
    }
    if (is(SchemaAttrText::inputsSourceWeights)) {
        if (!arrays || arrays->weights.empty())
            return VtValue();
        VtFloatArray weights;
        for (const double w : arrays->weights)
            weights.push_back(float(w));
        return VtValue(weights);
    }
    if (is(SchemaAttrText::inputsTranslationOffsets))
        return arrays && !arrays->translationOffsets.empty()
                   ? VtValue(VtVec3dArray(arrays->translationOffsets.begin(),
                                          arrays->translationOffsets.end()))
                   : VtValue();
    if (is(SchemaAttrText::inputsRotationOffsets))
        return arrays && !arrays->rotationOffsets.empty()
                   ? VtValue(VtVec3dArray(arrays->rotationOffsets.begin(),
                                          arrays->rotationOffsets.end()))
                   : VtValue();
    if (parent) {
        if (is(SchemaAttrText::inputsAffectTranslationX))
            return leaf(c.tX);
        if (is(SchemaAttrText::inputsAffectTranslationY))
            return leaf(c.tY);
        if (is(SchemaAttrText::inputsAffectTranslationZ))
            return leaf(c.tZ);
        if (is(SchemaAttrText::inputsAffectRotationX))
            return leaf(c.rX);
        if (is(SchemaAttrText::inputsAffectRotationY))
            return leaf(c.rY);
        if (is(SchemaAttrText::inputsAffectRotationZ))
            return leaf(c.rZ);
        if (is(SchemaAttrText::inputsAffectScaleX))
            return leaf(c.sX);
        if (is(SchemaAttrText::inputsAffectScaleY))
            return leaf(c.sY);
        if (is(SchemaAttrText::inputsAffectScaleZ))
            return leaf(c.sZ);
        return VtValue();
    }
    if (is(SchemaAttrText::inputsAffectTranslationX) ||
        is(SchemaAttrText::inputsAffectRotationX) ||
        is(SchemaAttrText::inputsAffectScaleX))
        return leaf(c.affectX);
    if (is(SchemaAttrText::inputsAffectTranslationY) ||
        is(SchemaAttrText::inputsAffectRotationY) ||
        is(SchemaAttrText::inputsAffectScaleY))
        return leaf(c.affectY);
    if (is(SchemaAttrText::inputsAffectTranslationZ) ||
        is(SchemaAttrText::inputsAffectRotationZ) ||
        is(SchemaAttrText::inputsAffectScaleZ))
        return leaf(c.affectZ);
    if (c.type == SchemaTypeName::RigExecAimConstraint) {
        if (is(SchemaAttrText::inputsAimVector))
            return leaf(c.aimVector);
        if (is(SchemaAttrText::inputsUpVector))
            return leaf(c.upVector);
        if (is(SchemaAttrText::inputsRotationOffset))
            return leaf(c.rotationOffset);
        if (is(SchemaAttrText::inputsWorldUpVector))
            return leaf(c.worldUpVector);
        return VtValue();
    }
    if (is(SchemaAttrText::inputsTranslationOffset) ||
        is(SchemaAttrText::inputsRotationOffset) ||
        is(SchemaAttrText::inputsScaleOffset))
        return leaf(c.offset);
    return VtValue();
}

std::string
EnvelopeShape(const RigExecMoverParameters &p)
{
    std::string shape = RigExecEnvelopeIsFullStrength(p.weights)    ? "full"
                        : RigExecWireTakesSparseEnvelope(p.weights) ? "sparse"
                        : p.weights.representation == SchemaValueText::dense
                            ? "dense"
                            : "other";
    if (p.radialWeight)
        shape += " radial";
    return shape;
}

/// How a probe runs a ranged op: the scene's geometry grain, or every
/// node cut into ranges of `grain` points, on worker threads or inline.
struct RangeSetting {
    size_t grain;
    bool threads;
    const char *label;
};

constexpr size_t kGeometryGrain = SIZE_MAX;

const RangeSetting kRangeSettings[] = {
    {kGeometryGrain, false, ""}, {0, false, " (whole)"},
    {1, true, " (grain 1)"},     {7, true, " (grain 7)"},
    {64, false, " (grain 64)"},
};

std::unique_ptr<RigExecTaskSystem> g_tasks;

// A revision's commit, replayed through PoseCommit on the frames the commit
// read, and compared with every version it wrote. A commit the baked step
// passed through is an error on the op.
rigExecRig::PointFrame
ToOp(const RigExecPointFrame &f)
{
    rigExecRig::PointFrame out;
    for (size_t i = 0; i < 4; ++i)
        out.points[i] =
            rigExecStd::Vec3d(f.points[i][0], f.points[i][1], f.points[i][2]);
    out.flags = f.flags;
    return out;
}

RigExecPointFrame
FromOp(const rigExecRig::PointFrame &f)
{
    RigExecPointFrame out;
    for (size_t i = 0; i < 4; ++i)
        out.points[i] = GfVec3d(f.points[i][0], f.points[i][1], f.points[i][2]);
    out.flags = f.flags;
    return out;
}

using OpFrames = rigExecStd::RigExecArray<rigExecRig::PointFrame>;

OpFrames
Gather(const std::vector<RigExecPointFrame> &store,
       const std::vector<uint32_t> &versions)
{
    OpFrames out;
    out.Resize(versions.size());
    for (size_t k = 0; k < versions.size(); ++k)
        out[k] = ToOp(store[size_t(versions[k])]);
    return out;
}

RigExecPointFrameArray
Published(const std::vector<RigExecPointFrame> &store,
          const std::vector<uint32_t> &versions)
{
    RigExecPointFrameArray out;
    for (const uint32_t v : versions)
        out.frames.push_back(store[size_t(v)]);
    return out;
}

RigExecPointFrameArray
FromOp(const void *value)
{
    RigExecPointFrameArray out;
    if (const auto *frames = static_cast<const OpFrames *>(value))
        for (const rigExecRig::PointFrame &f : *frames)
            out.frames.push_back(FromOp(f));
    return out;
}

/// The baked pose host's reads, for a probe replaying what a step read.
struct ProbeHost {
    using Math = BakedPoseMath;
    BakedPoseInputs reads;
    const BakedPoseInputs &inputs() const
    {
        return reads;
    }
};

/// One op node built on demand: bind its inputs and outputs, then Run.
struct OneOp {
    RigExecRigDescBuilder builder;
    RigExecNodeId id;
    std::vector<std::pair<RigExecInputId, const void *>> values;
    std::unique_ptr<RigExecEvaluator> evaluator;
    std::string error;

    OneOp(const RigExecOpHandle &op, const void *payload)
        : id(builder.AddOp(op, payload))
    {
    }
    void In(const RigExecPortHandle &port, const void *value)
    {
        values.emplace_back(builder.BindInput(id, port), value);
    }
    RigExecOutputId Out(const RigExecPortHandle &port)
    {
        return builder.BindOutput(id, port);
    }
    bool Run()
    {
        const auto desc = builder.Build(&error);
        const auto program =
            desc ? RigExecCompile(*desc, RigExecCompileOptions(), &error)
                 : nullptr;
        evaluator = program ? RigExecEvaluator::Create(
                                  program, RigExecEvaluatorOptions(), &error)
                            : nullptr;
        if (!evaluator)
            return false;
        for (const auto &[input, value] : values)
            if (!evaluator->SetInput(input, value, &error))
                return false;
        return evaluator->Evaluate(&error);
    }
    template <class T> const T &Get(const RigExecOutputId &output) const
    {
        return *static_cast<const T *>(evaluator->GetOutput(output));
    }
};

/// Constraint source \p k as the step resolved it: a provider's fin
/// version, or a source read off the stage moved by its revised ancestors.
bool
ConstraintSourceFrame(const B &b, int walk, size_t k, RigExecPointFrame *frame)
{
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    if (c.sources[k] >= 0) {
        *frame = b.fin[size_t(commit.sourceReads[k])];
        return (frame->flags & RigExecPointFrameValid) != 0;
    }
    return BakedPoseInputs(&b).resolveNativeSource(
        size_t(walk), k, &frame->points, &frame->flags);
}

/// The envelope a constraint's head resolved: the weight field's one value
/// or the leaf, and 1 where a points target takes a per-point field.
double
ConstraintEnvelope(const B &b, const B::Constraint &c)
{
    if (!c.weightObject.IsEmpty())
        return c.pointsTarget.IsEmpty()
                   ? double(b.weightFields[size_t(c.weightField)].values[0])
                   : 1.0;
    return double(RigExecBakedLeafRead(b, c.defaultWeight));
}

PXR_NS::GfMatrix4d
FromOp(const rigExecStd::Mat4d &m)
{
    PXR_NS::GfMatrix4d out;
    for (int r = 0; r < 4; ++r)
        for (int k = 0; k < 4; ++k)
            out[r][k] = m._mtx[r][k];
    return out;
}

rigExecStd::Mat4d
ToOp(const PXR_NS::GfMatrix4d &m)
{
    rigExecStd::Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int k = 0; k < 4; ++k)
            out._mtx[r][k] = m[r][k];
    return out;
}

// MatrixMover: the target's entering frame moved through MatrixMover by the
// transform source and, with two sources, the space source, each with its
// rest. Compared on what the commit published.
void
ProbeMatrixMover(const B &b, int walk)
{
    using namespace rigExecOps;
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    const std::string kind = TypeName(c.type);
    const bool present = !commit.present.empty() && commit.present[0];
    const RigExecPointFrame &incoming = b.fin[size_t(commit.targetRead)];
    const RigExecPointFrame &baked =
        present ? b.fin[size_t(commit.slotWrites[0])] : incoming;
    const double weight = ConstraintEnvelope(b, c);
    if (!RigExecBakedLeafRead(b, c.enabled) || weight <= 0.0) {
        Compare("Constraint", kind, c.pathText, VtValue(baked),
                VtValue(incoming), !present);
        return;
    }
    std::vector<rigExecRig::PointFrame> frames, rests;
    std::vector<bool> rested;
    for (size_t k = 0; k < c.sources.size() && k < 2; ++k) {
        RigExecPointFrame frame;
        if (!ConstraintSourceFrame(b, walk, k, &frame)) {
            CompareFlag("Constraint", kind, c.pathText, "published", present,
                        false);
            Unprobed("Constraint", kind, "a source the step could not read");
            return;
        }
        frames.push_back(ToOp(frame));
        const int slot = c.sources[k];
        rested.push_back(slot >= 0);
        rests.push_back(slot >= 0 ? ToOp(b.restFrames[size_t(slot)])
                                  : rigExecRig::PointFrame());
    }
    if (frames.empty()) {
        Unprobed("Constraint", kind, "no rigExec:transform source");
        return;
    }
    MatrixMoverPayload payload;
    payload.radialBlend = c.radialBlend;
    OneOp op(MatrixMover(), &payload);
    const rigExecRig::PointFrame in = ToOp(incoming);
    op.In(MatrixMoverIn(), &in);
    op.In(MatrixMoverTransform(), &frames[0]);
    if (rested[0])
        op.In(MatrixMoverTransformRest(), &rests[0]);
    if (frames.size() > 1) {
        op.In(MatrixMoverSpace(), &frames[1]);
        if (rested[1])
            op.In(MatrixMoverSpaceRest(), &rests[1]);
    }
    op.In(MatrixMoverWeight(), &weight);
    const RigExecOutputId out = op.Out(MatrixMoverOut());
    if (!op.Run()) {
        OpFailed("Constraint", kind, c.pathText, !present, op.error);
        return;
    }
    const RigExecPointFrame moved = FromOp(op.Get<rigExecRig::PointFrame>(out));
    const bool applied = RigExecConstraintFrameUsable(moved);
    Compare("Constraint", kind, c.pathText, VtValue(baked),
            VtValue(applied ? moved : incoming), !present);
    CompareFlag("Constraint", kind, c.pathText, "published", present, applied);
}

// The geometry domain: the constraint solved at full weight through its op,
// then ConstraintDelta against the target's authored transform, compared
// with the delta the step measured for the Matrix revision.
void
ProbeConstraintDelta(const B &b, const B::Constraint &c,
                     RigExecConstraintRecord record,
                     const RigExecConstraintInputs &input, bool enabled,
                     double weight, RigExecPoseOpRunner &runner)
{
    using namespace rigExecOps;
    const std::string kind = TypeName(c.type) + " geometry";
    const size_t d = size_t(c.deltaBase);
    const bool present = b.deltaPresent[d] != 0;
    if (!enabled) {
        CompareFlag("Constraint", kind, c.pathText, "delta present", present,
                    false);
        return;
    }
    for (double *w :
         {&record.position.weight, &record.rotation.weight,
          &record.scale.weight, &record.parent.weight, &record.aim.weight})
        *w = 1.0;
    rigExecRig::PointFrame solved = ToOp(input.incoming);
    std::string error;
    if (weight > 0.0) {
        RigExecConstraintResult result;
        if (!RigExecRunConstraintOp(record, input, runner, &result, &error)) {
            OpFailed("Constraint", kind, c.pathText, !present, error);
            return;
        }
        solved = ToOp(result.frame);
    }
    OneOp op(ConstraintDelta(), nullptr);
    const rigExecStd::Mat4d base = ToOp(b.deltaBaseMatrix[d]);
    const uint8_t baseOk = b.deltaBaseOk[d] ? 1 : 0;
    op.In(ConstraintDeltaSolved(), &solved);
    op.In(ConstraintDeltaBase(), &base);
    op.In(ConstraintDeltaBaseOk(), &baseOk);
    op.In(ConstraintDeltaWeight(), &weight);
    const RigExecOutputId out = op.Out(ConstraintDeltaDelta());
    const RigExecOutputId measured = op.Out(ConstraintDeltaMeasured());
    if (!op.Run()) {
        OpFailed("Constraint", kind, c.pathText, !present, op.error);
        return;
    }
    CompareFlag("Constraint", kind, c.pathText, "delta present", present,
                op.Get<uint8_t>(measured) != 0);
    if (!present) {
        return;
    }
    Compare("Constraint", kind, c.pathText, VtValue(b.deltaValues[d]),
            VtValue(FromOp(op.Get<rigExecStd::Mat4d>(out))), false);
}

/// A frame as the match reads it: its matrix, or false when the frame is
/// not valid or maps to no matrix.
bool
MatchMatrix(const RigExecPointFrame &frame, GfMatrix4d *matrix)
{
    return (frame.flags & RigExecPointFrameValid) != 0 &&
           eval::pose_ops::PointsToMatrix<BakedPoseMath>(
               eval::pose_ops::IdentityLandmarks<BakedPoseMath>(), frame.points,
               matrix);
}

// IK/FK match: IkFkMatch on the matrices of the limb's frames and rests the
// step read, the five controls' channels, the solve's space and the pole's
// parent mode; compared on the values and the presented frames.
void
ProbeIkFkMatch(const B &b, int walk)
{
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    const std::string kind = TypeName(c.type);
    const BakedPoseInputs reads(&b);
    eval::PoseIkFkFacts facts;
    reads.ikFkFacts(size_t(walk), &facts);
    RigExecIkFkMatchOpConstants constants;
    constants.ikValue = facts.ikValue;
    constants.scaleCalibration = facts.scaleCalibration;
    constants.upperLengthOffset = facts.upperOffset;
    constants.lowerLengthOffset = facts.lowerOffset;
    constants.stretchLimb = facts.stretchLimb;
    RigExecIkFkMatchOpInputs in;
    in.weight = facts.envelope;
    std::copy(facts.scalars, facts.scalars + 7, in.scalars.begin());
    bool usable = c.sources.size() == 10;
    for (size_t i = 0; usable && i < 10; ++i)
        usable =
            c.sources[i] >= 0 &&
            MatchMatrix(b.fin[size_t(commit.sourceReads[i])], &in.sources[i]);
    for (size_t i = 0; usable && i < 8; ++i) {
        const int slot = c.sources[i < 3 ? i : i + 2];
        usable = MatchMatrix(b.restFrames[size_t(slot)], &in.rests[i]);
    }
    if (!usable) {
        CompareFlag("Constraint", kind, c.pathText, "values",
                    c.matchValuesValid, false);
        CompareFlag("Constraint", kind, c.pathText, "shown",
                    c.matchShownMask != 0, false);
        Unprobed("Constraint", kind, "a limb frame the step could not read");
        return;
    }
    for (size_t i = 0; i < 5; ++i)
        reads.matchChannels(size_t(c.sources[5 + i]), &in.channels[i]);
    if (c.spaceSlot >= 0) {
        const size_t s = size_t(c.spaceSlot);
        in.hasSpace = MatchMatrix(b.restFrames[s], &in.spaceRest) &&
                      std::isfinite(b.posedM[s][3][0]);
        in.spacePosed = b.posedM[s];
    }
    const int poleSlot = c.sources[6];
    if (c.matchPoleSwitch >= 0 && poleSlot >= 0 &&
        !b.posedAuthored[size_t(poleSlot)]) {
        const size_t index = size_t(c.matchPoleSwitch);
        const auto &sw = b.spaceSwitches[index];
        using S = eval::OpSwitchRead;
        RigExecSpaceSwitchInputs &pole = in.poleSwitch;
        reads.switchMatrix(index, S::PosedDefault, 0, &pole.posedDefault);
        reads.switchMatrix(index, S::ParentDefaultInverse, 0,
                           &pole.parentDefaultInverse);
        reads.switchMatrix(index, S::ParentPosed, 0, &pole.parentPosed);
        reads.switchMatrix(index, S::ParentDefault, 0, &pole.parentDefault);
        pole.active = reads.switchActive(index);
        pole.hasCarry = sw.spaceSlot >= 0;
        if (pole.hasCarry) {
            reads.switchMatrix(index, S::SpaceDefault, 0, &pole.spaceDefault);
            reads.switchMatrix(index, S::SpacePosed, 0, &pole.spacePosed);
        }
        pole.sources.resize(sw.sourceSlots.size());
        for (size_t k = 0; k < sw.sourceSlots.size(); ++k) {
            pole.sources[k].world = sw.sourceSlots[k] < 0;
            reads.switchMatrix(index, S::SourceDefaults, k,
                               &pole.sources[k].defaultSpace);
            reads.switchMatrix(index, S::SourcePosed, k,
                               &pole.sources[k].posedSpace);
            constants.poleWorld.push_back(sw.sourceSlots[k] < 0 ? 1 : 0);
        }
        in.poleMode = uint32_t(rigExecOps::IkFkPoleMode::Switch);
        constants.poleRecord = sw.kernelRecord;
        constants.poleHasCarry = pole.hasCarry;
        constants.poleSourceRides = c.matchPoleSourceRides;
    } else if (c.matchPoleParentRides) {
        in.poleMode = uint32_t(rigExecOps::IkFkPoleMode::Ride);
    }
    RigExecPoseOpRunner runner;
    RigExecIkFkMatchOpOutputs out;
    std::string error;
    if (!RigExecRunIkFkMatchOp(constants, in, runner, &out, &error)) {
        OpFailed("Constraint", kind, c.pathText, false, error);
        return;
    }
    CompareFlag("Constraint", kind, c.pathText, "matched", c.matchValuesValid,
                out.matched);
    CompareFlag("Constraint", kind, c.pathText, "shown mask",
                c.matchShownMask == out.shownMask, true);
    if (!out.matched)
        return;
    Compare(
        "Constraint", kind, c.pathText,
        VtValue(VtDoubleArray(c.matchIkValues.begin(), c.matchIkValues.end())),
        VtValue(VtDoubleArray(out.ikValues.begin(), out.ikValues.end())), false,
        "IK values");
    Compare(
        "Constraint", kind, c.pathText + " FK",
        VtValue(VtDoubleArray(c.matchFkValues.begin(), c.matchFkValues.end())),
        VtValue(VtDoubleArray(out.fkValues.begin(), out.fkValues.end())), false,
        "FK values");
    for (size_t k = 0; k < 5; ++k) {
        if (!(out.shownMask & (1u << k)))
            continue;
        RigExecPointFrame shown;
        eval::pose_ops::FrameFromMatrix<BakedPoseMath>(
            out.shown[k], &shown.points, &shown.flags);
        Compare("Constraint", kind, c.pathText + " shown " + std::to_string(k),
                VtValue(c.matchShown[k]), VtValue(shown), false);
    }
}

// Harmonics: the constraint's frame and this tick's capture through the
// Harmonics op over the window ring the step read (the earlier ticks), with
// the host's gate closed where the step passed through before solving.
void
ProbeHarmonics(const B &b, int walk)
{
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    const std::string kind = TypeName(c.type);
    if (c.window < 0) {
        Unprobed("Constraint", kind, "no window");
        return;
    }
    RigExecHarmonicsOpConstants constants;
    constants.settings = c.harmonicsSettings;
    constants.matchVelocity = c.harmonicsMatchVelocity;
    constants.frames =
        uint32_t(RigExecHarmonicsWindowFrames(c.harmonicsSettings));
    RigExecHarmonicsOpInputs in;
    in.incoming = b.fin[size_t(commit.targetRead)];
    if (c.spaceSlot >= 0)
        in.space = b.posedM[size_t(c.spaceSlot)];
    double weight = double(RigExecBakedLeafRead(b, c.defaultWeight));
    bool gate = RigExecBakedLeafRead(b, c.enabled);
    if (!c.weightObject.IsEmpty()) {
        const auto &field = b.weightFields[size_t(c.weightField)];
        gate = gate && field.ok && field.values.size() == 1;
        weight = field.values.empty() ? 0.0 : double(field.values[0]);
    } else {
        gate = gate && std::isfinite(weight) && weight >= 0.0 && weight <= 1.0;
    }
    in.enabled = gate;
    in.weight = weight;
    const auto scalar = [&](size_t k) {
        const auto &s = c.harmonicsScalars[k];
        return s.isFloat ? double(RigExecBakedLeafRead(b, s.narrow))
                         : RigExecBakedLeafRead(b, s.wide);
    };
    in.amplitude = scalar(0);
    in.decay = scalar(1);
    in.termination = scalar(2);
    in.axisAmplitude = RigExecBakedLeafRead(b, c.offset);
    const RigExecWindowRing &ring = b.windowRings[size_t(c.window)];
    RigExecPoseOpRunner runner;
    RigExecWindowPacketPtr capture;
    RigExecPointFrame frame;
    std::string error;
    const bool present = !commit.present.empty() && commit.present[0];
    const RigExecPointFrame &baked =
        present ? b.fin[size_t(commit.slotWrites[0])] : in.incoming;
    if (!RigExecRunHarmonicsOp(constants, in, &ring, false, runner, &capture,
                               &frame, &error)) {
        OpFailed("Constraint", kind, c.pathText, !present, error);
        return;
    }
    Compare("Constraint", kind, c.pathText, VtValue(baked), VtValue(frame),
            !present, "window of " + std::to_string(ring.size()));
    CompareFlag(
        "Constraint", kind, c.pathText, "capture",
        RigExecWindowPacketsEqual(b.windowCaptures[size_t(c.window)], capture),
        true);
}

// SingleChainIk with every input rebuilt from the program as the step
// gathers it: the chain's frame versions, the effector and pole objects
// through the host's resolvers, the rests (live or composed), the envelope;
// then the rest-derived preparation and the op. Compared with the chain the
// step solved and with every slot its commit published.
void
ProbeSingleChainIk(const B &b, int walk, RigExecPoseOpRunner &runner)
{
    const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
    const auto &commit = b.commits[size_t(walk)];
    const std::string kind = TypeName(c.type);
    const BakedPoseInputs in(&b);
    const eval::PoseIkChainView ik = in.ikChainView(size_t(walk));
    const int ikMode = ik.ikMode;
    if (commit.targetReads.size() != ik.count) {
        Unprobed("Constraint", kind, "a chain the step could not gather");
        return;
    }
    const auto facts = in.constraintFacts(size_t(walk));
    std::vector<RigExecPointFrame> chain;
    for (const uint32_t read : commit.targetReads)
        chain.push_back(b.fin[size_t(read)]);
    RigExecPointFrame effector;
    bool usable =
        in.resolveIkEffector(size_t(walk), &effector.points, &effector.flags);
    GfVec3d pole(0.0, 1.0, 0.0);
    double twist = 0.0;
    if (ikMode == 0)
        in.ikPoleVector(size_t(walk), &pole, &twist);
    if (usable && ikMode == 0 && ik.poleModeObject) {
        usable = ik.poleObjects && facts.arrays.poleOk;
        GfVec3d sum(0.0);
        double total = 0.0;
        for (size_t k = 0; usable && k < ik.poleObjects; ++k) {
            RigExecPointFrame source;
            usable = in.ikPoleBounds(size_t(walk), k) &&
                     k < facts.arrays.poleWeightCount &&
                     in.resolveIkPole(size_t(walk), k, &source.points,
                                      &source.flags) &&
                     std::isfinite(facts.arrays.poleWeights[k]) &&
                     facts.arrays.poleWeights[k] >= 0;
            if (usable) {
                sum = sum + source.points[0] * facts.arrays.poleWeights[k];
                total += facts.arrays.poleWeights[k];
            }
        }
        usable = usable && total > 0;
        if (usable)
            pole = sum * (1.0 / total);
    }
    std::vector<RigExecPointFrame> prepared;
    const std::vector<RigExecPointFrame> *solve = &chain;
    if (usable && !ik.useAnimatedTs) {
        std::vector<RigExecPointFrame> rests;
        for (size_t k = 0; k < ik.count; ++k)
            rests.push_back(in.ikRestLive(size_t(walk), k)
                                ? chain[k]
                                : b.restFrames[size_t(ik.slots[k])]);
        usable = RigExecPrepareRestDerivedIkChain(chain, rests, &prepared);
        solve = &prepared;
    }
    const double weight = ConstraintEnvelope(b, c);
    if (!usable || !RigExecBakedLeafRead(b, c.enabled) || !(weight > 0)) {
        Unprobed("Constraint", kind, "a step that solved nothing");
        return;
    }
    RigExecConstraintRecord record;
    record.kind = RigExecConstraintKind::SingleChainIk;
    record.singleChain.mode = c.ikMode;
    record.singleChain.pole = pole;
    record.singleChain.twistDegrees = twist;
    record.singleChain.weight = weight;
    record.singleChain.preserveJointOrientation = c.preserveJointOrientation;
    record.singleChain.stretch = in.ikStretch(size_t(walk));
    RigExecConstraintInputs inputs;
    inputs.chain = *solve;
    inputs.effector = effector;
    RigExecConstraintResult result;
    std::string error;
    const bool ran =
        RigExecRunConstraintOp(record, inputs, runner, &result, &error);
    if (!ran) {
        OpFailed("Constraint", kind, c.pathText, commit.abandoned, error);
        return;
    }
    for (size_t k = 0; k < c.targetSlots.size() && k < result.chain.size(); ++k)
        for (size_t pos = 0; pos < commit.slots.size(); ++pos)
            if (commit.slots[pos] == c.targetSlots[k] && commit.present[pos])
                Compare("Constraint", kind + " slots",
                        c.pathText + " " + std::to_string(k),
                        VtValue(commit.frames[pos]), VtValue(result.chain[k]),
                        false);
}

// Constraint: SingleChainIk rebuilds its inputs (ProbeSingleChainIk);
// MatrixMover runs its own op. Every other frame kernel is
// rebuilt from the scene record and the frames the step read (a source or
// world-up object read off the stage resolved as the host resolves it, the
// rigExec:space carry composed as the step composes it), and compared on
// what the commit published: the candidate when present, the entering frame
// when the constraint passed through or refused. A geometry-domain
// constraint is compared on the delta it measured.
void
ProbeConstraint(const B &b)
{
    for (const auto &step : b.steps) {
        const RigExecBakedStepIdentity id = RigExecBakedIdentityOf(b, step);
        if (id.kind != RigExecBakedStepKind::Constraint || id.part >= 0)
            continue;
        const int walk = id.object;
        const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
        const auto &commit = b.commits[size_t(walk)];
        const std::string kind = TypeName(c.type);
        if (c.ikFkMatch) {
            ProbeIkFkMatch(b, walk);
            continue;
        }
        if (c.target < 0)
            continue;
        if (c.harmonics) {
            ProbeHarmonics(b, walk);
            continue;
        }
        RigExecPoseOpRunner runner;
        std::string error;
        if (c.singleChainIk) {
            ProbeSingleChainIk(b, walk, runner);
            continue;
        }
        if (c.type == SchemaTypeName::RigExecMatrixMover) {
            ProbeMatrixMover(b, walk);
            continue;
        }
        if (c.pointsTarget.IsEmpty() && !c.weightObject.IsEmpty() &&
            (c.weightField < 0 || !b.weightFields[size_t(c.weightField)].ok ||
             b.weightFields[size_t(c.weightField)].values.size() != 1)) {
            Unprobed("Constraint", kind,
                     "rigExec:weightObject whose weight field did not resolve");
            continue;
        }
        if (c.deltaBase < 0 &&
            (commit.split || commit.slotWrites.size() != 1)) {
            Unprobed("Constraint", kind,
                     "a commit of " + std::to_string(commit.slotWrites.size()) +
                         " slots" + (commit.split ? ", split" : "") + " at " +
                         c.pathText);
            continue;
        }
        if (!b.sceneDescriptors) {
            Unprobed("Constraint", kind, "no scene descriptors");
            continue;
        }
        RigExecSceneConstraintDescriptor descriptor;
        if (!RigExecTranslateSceneConstraint(*b.sceneDescriptors, c.path,
                                             {b.paths[size_t(c.target)]},
                                             &descriptor, &error)) {
            Unprobed("Constraint", kind, "scene translation refused");
            continue;
        }
        RigExecConstraintInputs input;
        input.incoming = b.fin[size_t(commit.targetRead)];
        input.sources.resize(c.sources.size());
        bool sourced = true;
        for (size_t k = 0; k < c.sources.size(); ++k)
            sourced = sourced && ConstraintSourceFrame(b, walk, k,
                                                       &input.sources[k].frame);
        if (c.worldUpObjectNamed) {
            RigExecPointFrame up;
            sourced = sourced && BakedPoseInputs(&b).resolveWorldUp(
                                     size_t(walk), &up.points, &up.flags);
            input.worldUp = up;
        }
        if (!sourced) {
            Unprobed("Constraint", kind, "a source the step could not read");
            continue;
        }
        bool stageRead = c.worldUpObjectNamed && c.worldUpObject < 0;
        for (const int source : c.sources)
            stageRead = stageRead || source < 0;
        const std::string shape =
            std::string(c.spaceSlot >= 0 ? " space" : "") +
            (stageRead ? " stage source" : "");
        if (c.spaceSlot >= 0) {
            ProbeHost host{BakedPoseInputs(&b)};
            GfMatrix4d carry(1.0);
            if (!eval::ComputeSpaceCarry(host, c.spaceSlot, &carry)) {
                Unprobed("Constraint", kind, "rigExec:space names no slot");
                continue;
            }
            input.carry = carry;
        }
        const RigExecBakedConstraintArray *arrays =
            c.arrays >= 0 ? &b.constraintArrays[size_t(c.arrays)] : nullptr;
        if (arrays && !arrays->ok) {
            Unprobed("Constraint", kind,
                     "source weight or offset array cardinality refused");
            continue;
        }
        descriptor.order = c.order;
        descriptor.blendShear = c.blendShear;
        descriptor.preserveInputUp = c.preserveInputUp;
        descriptor.aimVectorAuthored = true;
        descriptor.worldUpType = c.worldUpType;
        descriptor.sceneUp = c.sceneUp;
        descriptor.worldUpRotationOnly = c.worldUpRotationOnly;
        const auto read = [&](const char *name, VtValue *value) {
            *value = ConstraintParameter(b, c, arrays, name);
            return true;
        };
        bool enabled = true;
        float weight = 1.0f;
        if (!RigExecRefreshConstraintParameters(&descriptor, read, &input,
                                                &enabled, &weight, &error)) {
            Unprobed("Constraint", kind, "parameter refresh refused: " + error);
            continue;
        }
        if (c.deltaBase >= 0) {
            ProbeConstraintDelta(b, c, descriptor.record, input, enabled,
                                 ConstraintEnvelope(b, c), runner);
            continue;
        }
        const bool present = !commit.present.empty() && commit.present[0];
        const RigExecPointFrame &baked =
            present ? b.fin[size_t(commit.slotWrites[0])] : input.incoming;
        RigExecPointFrame published = input.incoming;
        bool applied = false;
        if (enabled && weight > 0) {
            RigExecConstraintResult result;
            if (RigExecRunConstraintOp(descriptor.record, input, runner,
                                       &result, &error) &&
                RigExecConstraintFrameUsable(result.frame)) {
                published = result.frame;
                applied = true;
            }
        }
        Compare("Constraint", kind + shape, c.pathText, VtValue(baked),
                VtValue(published), !present);
        CompareFlag("Constraint", kind + shape, c.pathText, "published",
                    present, applied);
    }
}

void
ProbeCommits(const B &b)
{
    using namespace rigExecOps;
    using Kind = RigExecBakedStepKind;
    for (const auto &step : b.steps) {
        const RigExecBakedStepIdentity identity =
            RigExecBakedIdentityOf(b, step);
        const bool constraint =
            identity.kind == Kind::Constraint && identity.part < 0 &&
            RigExecBakedConstraintStoresFrames(b, size_t(identity.object));
        if (!constraint && step.kind != Kind::SolverCommit &&
            step.kind != Kind::CommitApply)
            continue;
        const RigExecBakedCommit &commit =
            b.commits[size_t(constraint ? identity.object : step.object)];
        if (commit.split != (step.kind == Kind::CommitApply))
            continue;
        const std::string kind =
            std::string(commit.solverOutput ? "solver" : "constraint") +
            (commit.split ? " split" : "") +
            (commit.slots.size() > 1 ? " multi-slot" : "");
        const size_t slots = commit.slots.size();
        const size_t pairs = commit.propagate.size();
        PoseCommitPayload p;
        p.slotCount = slots;
        p.solverOutput = commit.solverOutput;
        for (size_t k = 0; k < pairs; ++k) {
            bool known = true;
            const bool blocked = RigExecCommitParentBlocked(
                commit.propagate[k].first, commit.propagate[k].second,
                [&](int slot, bool *authored, int *next) {
                    if (size_t(slot) >= b.propParent.size())
                        return false;
                    *authored = b.parentSpaceAuthored[size_t(slot)] != 0;
                    *next = b.propParent[size_t(slot)];
                    return true;
                },
                &known);
            p.closest.push_back(commit.closestPos[k]);
            p.blocked.push_back(blocked ? 1 : 0);
            p.carried.push_back(-1);
            p.baseCarried.push_back(-1);
        }
        for (size_t pos = 0; pos < slots; ++pos)
            if (commit.present[pos]) {
                p.entrySources.push_back(0);
                p.entryFrames.push_back(int32_t(pos));
                p.entryCandidates.push_back(int32_t(pos));
            }
        bool refused = false;
        if (!commit.abandoned)
            refused = RigExecFirstCommitFailure(commit.outcome.data(),
                                                commit.outcome.size()) <
                      commit.outcome.size();
        RigExecRigDescBuilder builder;
        const RigExecNodeId id = builder.AddOp(PoseCommit(), &p);
        const auto bind = [&](const rigExec::RigExecPortHandle &port) {
            return builder.BindInput(id, port);
        };
        const RigExecInputId candidates = bind(PoseCommitSources()),
                             entering = bind(PoseCommitEntering()),
                             carry = bind(PoseCommitCarry());
        RigExecInputId baseCarry, current, closest, descendantCarry,
            descendantBaseCarry;
        if (commit.solverOutput)
            baseCarry = bind(PoseCommitBaseCarry());
        if (pairs) {
            current = bind(PoseCommitCurrent());
            closest = bind(PoseCommitClosest());
            descendantCarry = bind(PoseCommitDescendantCarry());
            if (commit.solverOutput)
                descendantBaseCarry = bind(PoseCommitDescendantBaseCarry());
        }
        const RigExecOutputId outSlots =
                                  builder.BindOutput(id, PoseCommitSlots()),
                              outDescendants = builder.BindOutput(
                                  id, PoseCommitDescendants()),
                              outBaseSlots =
                                  builder.BindOutput(id, PoseCommitBaseSlots()),
                              outBaseDescendants = builder.BindOutput(
                                  id, PoseCommitBaseDescendants());
        std::string error;
        const auto desc = builder.Build(&error);
        const auto program =
            desc ? RigExecCompile(*desc, RigExecCompileOptions(), &error)
                 : nullptr;
        auto evaluator = program
                             ? RigExecEvaluator::Create(
                                   program, RigExecEvaluatorOptions(), &error)
                             : nullptr;
        rigExecRig::PointFrameArray candidateFrames;
        candidateFrames.frames.Resize(slots);
        for (size_t pos = 0; pos < slots; ++pos)
            candidateFrames.frames[pos] = ToOp(commit.frames[pos]);
        const auto set = [&](const RigExecInputId &input,
                             const OpFrames &frames) {
            return evaluator->SetInput(input, &frames, &error);
        };
        bool ran =
            evaluator &&
            evaluator->SetInput(candidates, &candidateFrames, &error) &&
            set(entering, Gather(b.fin, commit.slotReads)) &&
            set(carry, Gather(b.fin, commit.slotCarry)) &&
            (!commit.solverOutput ||
             set(baseCarry, Gather(b.base, commit.slotBaseCarry))) &&
            (!pairs ||
             (set(current, Gather(b.fin, commit.descendantReads)) &&
              set(closest, Gather(b.fin, commit.closestReads)) &&
              set(descendantCarry, Gather(b.fin, commit.descendantCarry)) &&
              (!commit.solverOutput ||
               set(descendantBaseCarry,
                   Gather(b.base, commit.descendantBaseCarry)))));
        ran = ran && evaluator->Evaluate(&error);
        if (!ran) {
            OpFailed("Commit", kind, commit.moverPathText, refused, error);
            continue;
        }
        if (refused) {
            Report("Commit/" + kind, commit.moverPathText,
                   "the baked commit passed through where the op ran");
            continue;
        }
        Compare("Commit", kind, commit.moverPathText,
                VtValue(Published(b.fin, commit.slotWrites)),
                VtValue(FromOp(evaluator->GetOutput(outSlots))), false);
        Compare("Commit", kind, commit.moverPathText + " descendants",
                VtValue(Published(b.fin, commit.descendantWrites)),
                VtValue(FromOp(evaluator->GetOutput(outDescendants))), false);
        if (commit.solverOutput) {
            Compare("Commit", kind, commit.moverPathText + " base",
                    VtValue(Published(b.base, commit.slotBaseWrites)),
                    VtValue(FromOp(evaluator->GetOutput(outBaseSlots))), false);
            Compare("Commit", kind, commit.moverPathText + " base descendants",
                    VtValue(Published(b.base, commit.descendantBaseWrites)),
                    VtValue(FromOp(evaluator->GetOutput(outBaseDescendants))),
                    false);
        }
    }
}

// A blend's channels as its gather read them: each weight from its pose
// slot or its leaf, and for a whole blend each sample's activation leaf and
// sparse layout or consumed dense points, in activation order.
std::vector<RigExecBlendChannel>
BlendChannels(const B &b, const B::GeomRevision &revision)
{
    const std::vector<VtValue> &leaves =
        RigExecBakedResolvePathLeaves(b, revision.leaves);
    const auto leaf = [&](int k) {
        return k >= 0 && size_t(k) < leaves.size() ? leaves[size_t(k)]
                                                   : VtValue();
    };
    std::vector<RigExecBlendChannel> channels;
    for (const auto &bound : revision.blendChannels) {
        RigExecBlendChannel channel;
        for (const auto &s : bound.samples) {
            RigExecBlendSampleData sample;
            const VtValue activation = leaf(s.activationLeaf);
            if (activation.IsHolding<float>())
                sample.activation = activation.UncheckedGet<float>();
            if (s.blendShape.IsEmpty())
                sample.points = s.lastPoints;
            else
                sample.layout = s.layout;
            channel.samples.push_back(std::move(sample));
        }
        std::stable_sort(channel.samples.begin(), channel.samples.end(),
                         [](const RigExecBlendSampleData &x,
                            const RigExecBlendSampleData &y) {
                             return x.activation < y.activation;
                         });
        channels.push_back(std::move(channel));
    }
    for (size_t c = 0; c < channels.size() && c < revision.blendChannels.size();
         ++c) {
        const auto &bound = revision.blendChannels[c];
        const VtValue held = leaf(bound.weightHeldLeaf);
        if (bound.poseWeight >= 0 &&
            !(held.IsHolding<bool>() && held.UncheckedGet<bool>())) {
            channels[c].weight = b.poseWeights[size_t(bound.poseWeight)];
        } else {
            const VtValue value = leaf(bound.weightLeaf);
            channels[c].weight =
                value.IsHolding<float>() ? value.UncheckedGet<float>() : 0.0f;
        }
    }
    return channels;
}

// An External revision's declared inputs, gathered from the leaves and
// provider values its assembly read, and the op values over them.
struct ExternalReplay {
    std::vector<VtValue> declared;
    std::vector<GfVec3f> base;
    RigExecMoverParameters p;
    RigExecExternalOpValues values;

    ExternalReplay(const B &b, const B::GeomChain &chain,
                   const B::GeomRevision &revision)
        : base(chain.lastBase.cbegin(), chain.lastBase.cend())
    {
        RigExecRevisionLeafView view;
        view.decl = &revision.leaves.decl;
        view.values = &RigExecBakedResolvePathLeaves(b, revision.leaves);
        view.external = &revision.externalPayload;
        if (!revision.binding.phases.empty())
            view.phased = &revision.revisionInputs;
        RigExecProviderValues provider;
        if (revision.haveTransform)
            provider.transform = &revision.transform;
        if (revision.haveCarry)
            provider.carry = &revision.carry;
        if (!revision.influences.empty())
            provider.influenceTransforms = &revision.influences;
        if (revision.weightObject >= 0)
            provider.weights =
                revision.weightCurrentPhase
                    ? &revision.currentPhasePacket
                    : &b.weightPackets[size_t(revision.weightObject)];
        provider.basePoints = base;
        p = RigExecGatherExternalInputs(revision.binding, view, provider,
                                        &declared);
        values.inputs = &declared;
        values.transform = provider.transform;
        values.influenceTransforms = provider.influenceTransforms;
        values.basePoints = &base;
    }
    ExternalReplay(const ExternalReplay &) = delete;
    ExternalReplay &operator=(const ExternalReplay &) = delete;
};

// A windowed op starts from an empty history.
bool
RunExternal(const B::GeomRevision &revision, const ExternalReplay &x,
            RigExecGeometryOpRunner &runner, std::vector<GfVec3f> &points,
            std::string *error)
{
    RigExecWindowRing ring;
    RigExecWindowPacketPtr capture;
    std::shared_ptr<const void> cache;
    RigExecWindowRun window;
    window.ring = &ring;
    window.capture = &capture;
    window.cache = &cache;
    return RigExecRunExternalOp(revision.binding, x.p, x.values, runner, points,
                                error, &window);
}

bool
IsCloth(const B::GeomRevision &revision)
{
    return revision.binding.externalSchema == SchemaTypeName::RigExecClothMover;
}

std::string
ExternalKind(const B::GeomRevision &revision)
{
    return "External " + TypeName(revision.binding.externalSchema);
}

// Cloth is exempt from bit identity: the op solves from an empty history
// while the body read its window ring, so only the point count and
// finiteness are compared.
void
CompareShape(const std::string &family, const std::string &kind,
             const std::string &object, const std::vector<GfVec3f> &baked,
             const std::vector<GfVec3f> &op, bool refusal,
             const std::string &note)
{
    Tally &tally = g_tally[family + "/" + kind];
    ++tally.compared;
    if (refusal)
        ++tally.refusals;
    if (baked.size() != op.size()) {
        Report(family + "/" + kind, object,
               "point count baked " + std::to_string(baked.size()) + " op " +
                   std::to_string(op.size()) +
                   (note.empty() ? std::string() : " (" + note + ")"));
        return;
    }
    for (size_t i = 0; i < op.size(); ++i)
        for (size_t k = 0; k < 3; ++k)
            if (std::isfinite(baked[i][k]) && !std::isfinite(op[i][k])) {
                Report(family + "/" + kind, object,
                       "point " + std::to_string(i) + " is not finite");
                return;
            }
}

void
ProbeExternal(const B &b, const B::GeomChain &chain, size_t r)
{
    const auto &revision = chain.revisions[r];
    const std::string kind = ExternalKind(revision);
    const ExternalReplay x(b, chain, revision);
    const bool admitted =
        revision.parameters.valid && revision.status.AllowsApply();
    CompareFlag("RevisionChunk", kind, revision.moverPathText, "packet valid",
                revision.parameters.valid, x.p.valid);
    const auto entering = VersionPoints(chain, r);
    const auto published = VersionPoints(chain, r + 1);
    for (const RangeSetting &setting : kRangeSettings) {
        RigExecGeometryOpRunner runner;
        runner.compile.useSimd = b.useSimd;
        if (setting.grain != kGeometryGrain) {
            runner.compile.rangeGrain = setting.grain;
            runner.compile.rangeThreshold = 0;
        }
        if (setting.threads)
            runner.evaluate.tasks = g_tasks.get();
        std::vector<GfVec3f> points = entering;
        std::string error;
        const bool applied = admitted && x.p.valid &&
                             RunExternal(revision, x, runner, points, &error);
        if (!applied)
            points = entering;
        const std::string object = revision.moverPathText + setting.label;
        const std::string note =
            applied ? std::string() : "op refused: " + error;
        if (IsCloth(revision))
            CompareShape("RevisionChunk", kind, object, published, points,
                         !admitted, note);
        else
            Compare("RevisionChunk", kind, object, Points(published),
                    Points(points), !admitted, note);
    }
    ++g_envelopes[kind][EnvelopeShape(revision.parameters)];
}

// RevisionChunk: the body runs a revision as vertex ranges; the op runs it
// at the scene's grain, whole and cut into ranges of several sizes. Each
// revision is replayed over the version entering it and compared with the
// version its fuse published, as the range verifier does, and the apply
// decisions are compared too.
void
ProbeRevisions(const B &b)
{
    for (const auto &chain : b.chains) {
        if (!chain.haveBase)
            continue;
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const auto &revision = chain.revisions[r];
            const std::string kind = OpName(revision.op);
            if (revision.gpuOwned) {
                Unprobed("RevisionChunk", kind, "GPU-owned chain");
                continue;
            }
            if (revision.rangeSetAside) {
                Unprobed("RevisionChunk", kind, "set aside by a cycle");
                continue;
            }
            if (revision.opMatrix || revision.opSmooth || revision.opMush ||
                revision.opLattice || revision.blendChunked ||
                revision.opExternal) {
                revision.opWhole) {
                Unprobed("RevisionChunk", kind, "runs as op nodes");
                continue;
            }
            if (revision.op == RigExecRevisionOp::External) {
                ProbeExternal(b, chain, r);
                continue;
            }
            if (revision.op != RigExecRevisionOp::BlendShape &&
                !RigExecGeometryOpSupported(revision.op)) {
                Unprobed("RevisionChunk", kind, "no op for this revision");
                continue;
            }
            const auto entering = VersionPoints(chain, r);
            const auto published = VersionPoints(chain, r + 1);
            auto p = revision.parameters;
            if (revision.op == RigExecRevisionOp::Skin)
                p.skinTransforms = revision.influences;
            const bool blend = revision.op == RigExecRevisionOp::BlendShape;
            const std::vector<RigExecBlendChannel> channels =
                blend ? BlendChannels(b, revision)
                      : std::vector<RigExecBlendChannel>();
            const std::vector<GfVec3f> blendBase =
                blend ? std::vector<GfVec3f>(chain.lastBase.cbegin(),
                                             chain.lastBase.cend())
                      : entering;
            for (const RangeSetting &setting : kRangeSettings) {
                RigExecGeometryOpRunner runner;
                runner.compile.useSimd = b.useSimd;
                if (setting.grain != kGeometryGrain) {
                    runner.compile.rangeGrain = setting.grain;
                    runner.compile.rangeThreshold = 0;
                }
                if (setting.threads)
                    runner.evaluate.tasks = g_tasks.get();
                std::vector<GfVec3f> points = entering;
                std::string error;
                bool applied = p.valid && revision.status.AllowsApply() &&
                               RigExecRunGeometryOp(
                                   revision.op, p, blend ? &channels : nullptr,
                                   blendBase, runner, points, &error);
                if (!applied)
                    points = entering;
                const bool admitted = p.valid && revision.status.AllowsApply();
                Compare("RevisionChunk", kind,
                        revision.moverPathText + setting.label,
                        Points(published), Points(points), !admitted,
                        applied ? std::string() : "op refused: " + error);
                if (revision.rangeRole)
                    CompareFlag("RevisionChunk", kind,
                                revision.moverPathText + setting.label,
                                "applies",
                                RigExecBakedRevisionApplies(revision), applied);
            }
            ++g_envelopes[kind][EnvelopeShape(p)];
        }
    }
}

// RangeChain: a chain's revisions as one op program, so consecutive ranged
// ops run as one range chain, at every range setting, against the same ops
// run node by node. An External op takes its points from the host, so it
// runs between the programs of the segments around it.
struct ChainReplay {
    RigExecMoverParameters p;
    std::vector<RigExecBlendChannel> channels;
    std::vector<GfVec3f> base;
    std::unique_ptr<ExternalReplay> external;
};

bool
RunChainSegment(const B &b, const B::GeomChain &chain,
                const std::vector<ChainReplay> &replays, size_t begin,
                size_t end, const RangeSetting &setting,
                std::vector<GfVec3f> &points, std::string *error)
{
    RigExecRigDescBuilder builder;
    std::vector<RigExecGeometryOpNode> nodes(end - begin);
    bool ok = true;
    for (size_t k = 0; k < nodes.size() && ok; ++k) {
        const auto &revision = chain.revisions[begin + k];
        const ChainReplay &x = replays[begin + k];
        const RigExecGeometryOpNode *upstream = k ? &nodes[k - 1] : nullptr;
        ok = revision.op == RigExecRevisionOp::BlendShape
                 ? RigExecAddGeometryBlendOp(x.channels, x.p, &builder,
                                             upstream, &nodes[k], error)
                 : RigExecAddGeometryOp(revision.op, x.p, &builder, upstream,
                                        &nodes[k], error);
    }
    RigExecCompileOptions options = RigExecGeometryCompileOptions();
    options.useSimd = b.useSimd;
    if (setting.grain != kGeometryGrain) {
        options.rangeGrain = setting.grain;
        options.rangeThreshold = 0;
    }
    RigExecEvaluatorOptions evaluate;
    if (setting.threads)
        evaluate.tasks = g_tasks.get();
    const auto desc = ok ? builder.Build(error) : nullptr;
    const auto program = desc ? RigExecCompile(*desc, options, error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, evaluate, error) : nullptr;
    if (!evaluator)
        return false;
    for (size_t k = 0; k < nodes.size(); ++k) {
        const ChainReplay &x = replays[begin + k];
        const std::vector<GfVec3f> *fed = k ? nullptr : &points;
        const bool written =
            chain.revisions[begin + k].op == RigExecRevisionOp::BlendShape
                ? RigExecWriteGeometryBlendInputs(x.channels, x.p, nodes[k],
                                                  x.base, fed, points.size(),
                                                  evaluator.get(), error)
                : RigExecWriteGeometryOpInputs(x.p, nodes[k], fed,
                                               points.size(), evaluator.get(),
                                               error);
        if (!written)
            return false;
    }
    if (!evaluator->Evaluate(error))
        return false;
    const auto *out =
        static_cast<const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
            evaluator->GetOutput(nodes.back().out));
    if (!out || out->Size() != points.size())
        return false;
    for (size_t i = 0; i < points.size(); ++i)
        points[i] = GfVec3f((*out)[i][0], (*out)[i][1], (*out)[i][2]);
    return true;
}

void
ProbeRangeChains(const B &b)
{
    for (const auto &chain : b.chains) {
        if (!chain.haveBase || chain.revisions.size() < 2)
            continue;
        bool usable = true;
        std::string kinds;
        for (const auto &revision : chain.revisions) {
            const bool external = revision.op == RigExecRevisionOp::External;
            kinds += (kinds.empty() ? "" : ">") +
                     (external ? ExternalKind(revision)
                               : std::string(OpName(revision.op)));
            usable =
                usable && !revision.gpuOwned && !revision.rangeSetAside &&
                !revision.opMatrix && !revision.opSmooth && !revision.opMush &&
                !revision.opLattice && !revision.blendChunked &&
                !revision.opWhole &&
                (external ? RigExecExternalOpHandle(revision.binding) != nullptr
                          : revision.op == RigExecRevisionOp::BlendShape ||
                                RigExecGeometryOpSupported(revision.op)) &&
                revision.parameters.valid && revision.status.AllowsApply();
        }
        if (!usable) {
            Unprobed("RangeChain", "chain", "a revision no op takes as is");
            continue;
        }
        const auto entering = VersionPoints(chain, 0);
        std::vector<ChainReplay> replays(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const auto &revision = chain.revisions[r];
            ChainReplay &x = replays[r];
            x.p = revision.parameters;
            if (revision.op == RigExecRevisionOp::Skin)
                x.p.skinTransforms = revision.influences;
            if (revision.op == RigExecRevisionOp::BlendShape) {
                x.channels = BlendChannels(b, revision);
                x.base = std::vector<GfVec3f>(chain.lastBase.cbegin(),
                                              chain.lastBase.cend());
            }
            if (revision.op == RigExecRevisionOp::External)
                x.external =
                    std::make_unique<ExternalReplay>(b, chain, revision);
        }
        const bool cloth =
            std::any_of(chain.revisions.begin(), chain.revisions.end(),
                        [](const B::GeomRevision &revision) {
                            return revision.op == RigExecRevisionOp::External &&
                                   IsCloth(revision);
                        });
        std::vector<GfVec3f> stepped = entering;
        bool stepsApplied = true;
        for (size_t r = 0; r < chain.revisions.size() && stepsApplied; ++r) {
            const auto &revision = chain.revisions[r];
            const ChainReplay &x = replays[r];
            RigExecGeometryOpRunner runner;
            runner.compile.useSimd = b.useSimd;
            runner.compile.rangeGrain = 0;
            std::vector<GfVec3f> next = stepped;
            if (x.external)
                stepsApplied =
                    x.external->p.valid &&
                    RunExternal(revision, *x.external, runner, next, nullptr);
            else
                stepsApplied = RigExecRunGeometryOp(
                    revision.op, x.p,
                    revision.op == RigExecRevisionOp::BlendShape ? &x.channels
                                                                 : nullptr,
                    revision.op == RigExecRevisionOp::BlendShape ? x.base
                                                                 : stepped,
                    runner, next);
            stepped = std::move(next);
        }
        if (!stepsApplied) {
            Unprobed("RangeChain", "chain", "a revision's op refused");
            continue;
        }
        for (const RangeSetting &setting : kRangeSettings) {
            std::vector<GfVec3f> piped = entering;
            std::string error;
            bool ok = true;
            for (size_t r = 0; r < chain.revisions.size() && ok;) {
                if (replays[r].external) {
                    RigExecGeometryOpRunner runner;
                    runner.compile.useSimd = b.useSimd;
                    if (setting.grain != kGeometryGrain) {
                        runner.compile.rangeGrain = setting.grain;
                        runner.compile.rangeThreshold = 0;
                    }
                    if (setting.threads)
                        runner.evaluate.tasks = g_tasks.get();
                    ok = RunExternal(chain.revisions[r], *replays[r].external,
                                     runner, piped, &error);
                    ++r;
                    continue;
                }
                size_t end = r + 1;
                while (end < chain.revisions.size() && !replays[end].external)
                    ++end;
                ok = RunChainSegment(b, chain, replays, r, end, setting, piped,
                                     &error);
                r = end;
            }
            if (!ok)
                piped.clear();
            const std::string object = chain.target.GetString() + setting.label;
            const std::string note =
                ok ? std::string() : "chain program failed: " + error;
            if (cloth && ok)
                CompareShape("RangeChain", kinds, object, stepped, piped, false,
                             note);
            else
                Compare("RangeChain", kinds, object, Points(stepped),
                        Points(piped), false, note);
        }
    }
}

// Derived: a points target replays its last assembly over its authored base
// with the chain's final points; a matrix target replays the projector reads
// over the chain's base and final points.
void
ProbeDerived(const B &b)
{
    for (const auto &chain : b.chains) {
        if (!chain.haveResult)
            continue;
        for (const auto &derived : chain.derived) {
            const auto &revision = derived.revision;
            const std::string kind = OpName(revision.op);
            if (derived.matrixTarget) {
                if (!RigExecGeometryMatrixOpSupported(revision.op)) {
                    Unprobed("Derived", kind, "no matrix op");
                    continue;
                }
                RigExecRevisionLeafView view;
                view.decl = &revision.leaves.decl;
                view.values =
                    &RigExecBakedResolvePathLeaves(b, revision.leaves);
                RigExecProjectorReads reads;
                RigExecReadProjectorTargetFromLeaves(
                    revision.op, revision.binding, view, &reads);
                const std::vector<GfVec3f> base(chain.lastBase.cbegin(),
                                                chain.lastBase.cend());
                const std::vector<GfVec3f> final(chain.result.cbegin(),
                                                 chain.result.cend());
                RigExecGeometryOpRunner runner;
                GfMatrix4d matrix(1.0);
                std::string diagnostic;
                const bool ran = RigExecRunGeometryMatrixOp(
                    revision.op, revision.binding,
                    RigExecBakedProjectorFrames(b, revision), reads, base,
                    final, runner, matrix, diagnostic);
                if (!ran) {
                    OpFailed("Derived", kind, derived.targetText,
                             !derived.haveMatrix, diagnostic);
                    continue;
                }
                Compare("Derived", kind, derived.targetText,
                        derived.haveMatrix ? VtValue(derived.matrix)
                                           : VtValue(),
                        VtValue(matrix), false);
                continue;
            }
            if (!RigExecGeometryOpSupported(revision.op)) {
                Unprobed("Derived", kind, "no op for this target");
                continue;
            }
            if (!revision.ran || !derived.haveBase) {
                Unprobed("Derived", kind, "the derived body did not run");
                continue;
            }
            auto p = revision.lastParameters;
            p.auxPoints.assign(revision.lastAuxPoints.cbegin(),
                               revision.lastAuxPoints.cend());
            const std::vector<GfVec3f> base(derived.lastBase.cbegin(),
                                            derived.lastBase.cend());
            std::vector<GfVec3f> points = base;
            RigExecGeometryOpRunner runner;
            std::string error;
            const bool applied =
                revision.lastStatus.AllowsApply() &&
                RigExecRunGeometryOp(revision.op, p, nullptr, base, runner,
                                     points, &error);
            if (!applied)
                points = base;
            Compare("Derived", kind, derived.targetText,
                    Points(revision.output), Points(points), !applied);
        }
    }
}

// WeightPacket: a combine weight's packet from the packets, target count,
// strength and invert its op node read, through the scene's combine runner.
void
ProbeWeights(const B &b)
{
    for (const auto &bakedStep : b.steps) {
        const RigExecBakedStepIdentity step =
            RigExecBakedIdentityOf(b, bakedStep);
        if (step.kind != RigExecBakedStepKind::WeightPacket)
            continue;
        const auto &w = b.weightObjects[size_t(step.object)];
        const std::string kind = TypeName(w.type);
        if (kind != SchemaTypeName::RigExecCombineWeight) {
            Unprobed("WeightPacket", kind, "the scene runs no op of this kind");
            continue;
        }
        RigExecWeightRecord record;
        record.kind = 2;
        record.representation = w.fieldRepresentation;
        record.representationToken = w.representation;
        record.rangePolicy = w.rangePolicy;
        record.clamp = w.fieldClamp;
        record.combineMode = w.fieldCombineMode;
        record.base = w.base;
        record.inputs = w.inputs;
        std::vector<const RigExecWeightPacket *> inputs;
        for (const int input : w.inputs)
            inputs.push_back(&b.weightPackets[size_t(input)]);
        size_t targetCount = 0;
        for (size_t i = 0; i < w.combineTargetPoints.size(); ++i)
            targetCount +=
                w.pointLeaves.Value<VtVec3fArray>(int(w.combineLeaves + i), {})
                    .size();
        RigExecWeightCombineOps ops;
        RigExecWeightPacket packet;
        std::string error;
        const auto &baked = b.weightPackets[size_t(step.object)];
        if (!ops.CombinePackets(0, record, inputs, targetCount,
                                RigExecBakedLeafRead(b, w.strength),
                                RigExecBakedLeafRead(b, w.invert), &packet,
                                &error)) {
            OpFailed("WeightPacket", kind, w.path.GetString(), !baked.valid,
                     error);
            continue;
        }
        Compare("WeightPacket", kind, w.path.GetString(), VtValue(baked),
                VtValue(packet), !baked.valid);
    }
}

// Spaces: a provider's matrices, a frame record's matrix and a connected
// provider's refresh, each from the versions and gates its step read,
// against what the step stored.
struct SpaceNode {
    std::unique_ptr<RigExecEvaluator> evaluator;
    std::vector<RigExecInputId> inputs;
    std::vector<RigExecOutputId> outputs;
    std::string error;
};

std::map<std::string, SpaceNode> g_spaceNodes;

SpaceNode &
SpaceNodeFor(const std::string &key, const RigExecOpHandle &op,
             const void *payload,
             const std::vector<const RigExecPortHandle *> &ins,
             const std::vector<const RigExecPortHandle *> &outs)
{
    SpaceNode &node = g_spaceNodes[key];
    if (node.evaluator || !node.error.empty())
        return node;
    RigExecRigDescBuilder builder;
    const RigExecNodeId id = builder.AddOp(op, payload);
    for (const RigExecPortHandle *port : ins)
        node.inputs.push_back(builder.BindInput(id, *port));
    for (const RigExecPortHandle *port : outs)
        node.outputs.push_back(builder.BindOutput(id, *port));
    const auto desc = builder.Build(&node.error);
    const auto program =
        desc ? RigExecCompile(*desc, {}, &node.error) : nullptr;
    node.evaluator =
        program ? RigExecEvaluator::Create(program, {}, &node.error) : nullptr;
    return node;
}

bool
RunSpaceNode(SpaceNode &node, const std::vector<const void *> &values,
             std::string *error)
{
    if (!node.evaluator) {
        *error = node.error;
        return false;
    }
    for (size_t i = 0; i < values.size(); ++i)
        if (!node.evaluator->SetInput(node.inputs[i], values[i], error))
            return false;
    return node.evaluator->Evaluate(error);
}

template <class T>
const T &
SpaceOut(const SpaceNode &node, size_t i)
{
    return *static_cast<const T *>(node.evaluator->GetOutput(node.outputs[i]));
}

rigExecRig::PointFrame
OpFrame(const std::array<GfVec3d, 4> &points, uint32_t flags)
{
    rigExecRig::PointFrame frame;
    for (size_t i = 0; i < 4; ++i)
        frame.points[i] =
            rigExecStd::Vec3d(points[i][0], points[i][1], points[i][2]);
    frame.flags = flags;
    return frame;
}

rigExecRig::PointFrame
OpFrame(const RigExecPointFrame &frame)
{
    return OpFrame(frame.points, frame.flags);
}

void
PutBits(std::vector<double> *out, const GfMatrix4d &m)
{
    out->insert(out->end(), m.data(), m.data() + 16);
}

void
PutBits(std::vector<double> *out, const rigExecStd::Mat4d &m)
{
    out->insert(out->end(), &m._mtx[0][0], &m._mtx[0][0] + 16);
}

void
PutBits(std::vector<double> *out, const rigExecRig::PointFrame &frame)
{
    for (const auto &point : frame.points)
        for (int a = 0; a < 3; ++a)
            out->push_back(point[a]);
    out->push_back(double(frame.flags));
}

/// Bit for bit, with the first differing value.
void
CompareBits(const std::string &family, const std::string &kind,
            const std::string &object, const std::vector<double> &baked,
            const std::vector<double> &op, bool refusal)
{
    Tally &tally = g_tally[family + "/" + kind];
    ++tally.compared;
    if (refusal)
        ++tally.refusals;
    if (baked.size() == op.size() &&
        std::memcmp(baked.data(), op.data(), baked.size() * sizeof(double)) ==
            0)
        return;
    size_t i = 0;
    while (i < baked.size() && i < op.size() &&
           std::memcmp(&baked[i], &op[i], sizeof(double)) == 0)
        ++i;
    Report(family + "/" + kind, object,
           "value " + std::to_string(i) + " of " +
               std::to_string(baked.size()) + "/" + std::to_string(op.size()) +
               ": baked " +
               (i < baked.size() ? TfStringify(baked[i]) : "<end>") + " op " +
               (i < op.size() ? TfStringify(op[i]) : "<end>"));
}

void
ProbeProviderMatrix(const B &b, const RigExecBakedStep &step)
{
    using namespace rigExecOps;
    const size_t slot = size_t(step.object);
    const bool posed = step.part != 0;
    const std::string kind = posed ? "final" : "base";
    ProviderMatrixPayload payload;
    payload.posed = posed;
    payload.joint = std::find(b.jointSlots.begin(), b.jointSlots.end(),
                              int(slot)) != b.jointSlots.end();
    std::vector<const RigExecPortHandle *> ins = {&ProviderMatrixRest(),
                                                  &ProviderMatrixBase()};
    if (posed)
        ins.push_back(&ProviderMatrixFinal());
    SpaceNode &node =
        SpaceNodeFor("ProviderMatrix/" + kind + (payload.joint ? "/joint" : ""),
                     ProviderMatrix(), &payload, ins, {&ProviderMatrixOut()});
    const auto rest = OpFrame(b.restPts[slot], b.restFrames[slot].flags);
    const auto base = OpFrame(b.base[size_t(b.baseLast[slot])]);
    const auto fin = OpFrame(b.fin[size_t(b.finLast[slot])]);
    std::vector<const void *> values = {&rest, &base};
    if (posed)
        values.push_back(&fin);
    std::string error;
    const std::string object = b.paths[slot].GetString();
    if (!RunSpaceNode(node, values, &error)) {
        OpFailed("Spaces", "ProviderMatrix/" + kind, object, false, error);
        return;
    }
    std::vector<double> baked, op;
    PutBits(&baked, posed ? b.finalMatrix[slot] : b.baseMatrix[slot]);
    PutBits(&op, SpaceOut<rigExecStd::Mat4d>(node, 0));
    CompareBits("Spaces", "ProviderMatrix/" + kind, object, baked, op, false);
}

void
ProbeFrameMatrix(const B &b, const RigExecBakedStep &step)
{
    using namespace rigExecOps;
    const size_t object = size_t(step.object);
    const auto &record = b.frameRecords[object];
    if (record.commit < 0 || size_t(record.commit) >= b.commits.size()) {
        Unprobed("Spaces", "FrameMatrix", "the record names no commit");
        return;
    }
    const auto &commit = b.commits[size_t(record.commit)];
    const std::string kind = commit.solverOutput ? "solver" : "constraint";
    FrameMatrixPayload payload;
    payload.solverOutput = commit.solverOutput;
    payload.position = record.position;
    payload.target = record.target;
    std::vector<const RigExecPortHandle *> ins = {&FrameMatrixRest(),
                                                  &FrameMatrixFrame()};
    if (commit.solverOutput)
        ins.push_back(&FrameMatrixPresent());
    else {
        ins.push_back(&FrameMatrixRecordAfter());
        ins.push_back(&FrameMatrixRecordEveryTarget());
    }
    SpaceNode &node = SpaceNodeFor(
        "FrameMatrix/" + kind + "/" + std::to_string(record.position) + "/" +
            std::to_string(record.target),
        FrameMatrix(), &payload, ins, {&FrameMatrixOut(), &FrameMatrixValid()});
    const auto rest = OpFrame(b.restFrames[size_t(record.slot)]);
    const auto frame = OpFrame(b.fin[size_t(record.version)]);
    rigExecStd::RigExecArray<uint8_t> present(commit.present.size());
    for (size_t i = 0; i < commit.present.size(); ++i)
        present[i] = commit.present[i] ? 1 : 0;
    const uint8_t after = commit.recordAfter ? 1 : 0;
    const uint8_t every = commit.recordEveryTarget ? 1 : 0;
    std::vector<const void *> values = {&rest, &frame};
    if (commit.solverOutput)
        values.push_back(&present);
    else {
        values.push_back(&after);
        values.push_back(&every);
    }
    std::string error;
    const std::string where = "record " + std::to_string(object);
    if (!RunSpaceNode(node, values, &error)) {
        OpFailed("Spaces", "FrameMatrix/" + kind, where, false, error);
        return;
    }
    std::vector<double> baked, op;
    PutBits(&baked, b.frameMatrix[object]);
    baked.push_back(b.frameMatrixValid[object] ? 1.0 : 0.0);
    PutBits(&op, SpaceOut<rigExecStd::Mat4d>(node, 0));
    op.push_back(SpaceOut<uint8_t>(node, 1) ? 1.0 : 0.0);
    CompareBits("Spaces", "FrameMatrix/" + kind, where, baked, op, false);
}

void
ProbeProviderRefresh(const B &b, const RigExecBakedStep &step)
{
    using namespace rigExecOps;
    const auto &refresh = b.providerRefreshes[size_t(step.object)];
    const auto *raw =
        b.providerValues.Read<RigExecPointFrame>(refresh.baseValue);
    const auto *current =
        b.providerValues.Read<RigExecPointFrame>(refresh.currentValue);
    if (!raw || !current) {
        Unprobed("Spaces", "ProviderRefresh",
                 "the re-evaluated base or current frame is missing");
        return;
    }
    ProviderRefreshPayload payload;
    std::vector<rigExecRig::PointFrame> carryBase, carryFinal;
    std::vector<rigExecStd::Mat4d> parents;
    for (const auto &carry : refresh.carries) {
        payload.blockingCounts.push_back(uint32_t(carry.blockingSlots.size()));
        carryBase.push_back(OpFrame(b.base[carry.baseRead]));
        carryFinal.push_back(OpFrame(b.fin[carry.finRead]));
        for (const int slot : carry.blockingSlots) {
            const GfMatrix4d m = RigExecBakedProviderParentRaw(b, slot);
            rigExecStd::Mat4d out;
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    out._mtx[r][c] = m[r][c];
            parents.push_back(out);
        }
    }
    std::string key = "ProviderRefresh";
    for (const uint32_t count : payload.blockingCounts)
        key += "/" + std::to_string(count);
    SpaceNode &node = SpaceNodeFor(
        key, ProviderRefresh(), &payload,
        {&ProviderRefreshBase(), &ProviderRefreshFinal(), &ProviderRefreshRaw(),
         &ProviderRefreshCurrent(), &ProviderRefreshConstrained(),
         &ProviderRefreshCarryBase(), &ProviderRefreshCarryFinal(),
         &ProviderRefreshBlockingParents()},
        {&ProviderRefreshBaseOut(), &ProviderRefreshFinalOut(),
         &ProviderRefreshCarryBaseOut(), &ProviderRefreshCarryFinalOut()});
    rigExecStd::RigExecArray<uint8_t> constrained(
        refresh.priorConstraints.size());
    for (size_t i = 0; i < refresh.priorConstraints.size(); ++i) {
        const auto &[commit, position] = refresh.priorConstraints[i];
        constrained[i] =
            !b.commits[commit].abandoned && b.commits[commit].present[position]
                ? 1
                : 0;
    }
    const auto base = OpFrame(b.base[refresh.baseRead]);
    const auto fin = OpFrame(b.fin[refresh.finRead]);
    const auto rawFrame = OpFrame(*raw), currentFrame = OpFrame(*current);
    const rigExecStd::RigExecArray<rigExecRig::PointFrame> carryBaseIn(
        carryBase.data(), carryBase.size());
    const rigExecStd::RigExecArray<rigExecRig::PointFrame> carryFinalIn(
        carryFinal.data(), carryFinal.size());
    const rigExecStd::RigExecArray<rigExecStd::Mat4d> parentsIn(parents.data(),
                                                                parents.size());
    const bool refused = !step.diagnostics.empty();
    const std::string object = b.paths[size_t(refresh.slot)].GetString();
    std::string error;
    if (!RunSpaceNode(node,
                      {&base, &fin, &rawFrame, &currentFrame, &constrained,
                       &carryBaseIn, &carryFinalIn, &parentsIn},
                      &error)) {
        OpFailed("Spaces", "ProviderRefresh", object, refused, error);
        return;
    }
    if (refused)
        Report("Spaces/ProviderRefresh", object,
               "the op ran where the baked step reported: " +
                   step.diagnostics.front());
    std::vector<double> baked, op;
    PutBits(&baked, OpFrame(b.base[refresh.baseWrite]));
    PutBits(&baked, OpFrame(b.fin[refresh.finWrite]));
    for (const auto &carry : refresh.carries) {
        PutBits(&baked, OpFrame(b.base[carry.baseWrite]));
        PutBits(&baked, OpFrame(b.fin[carry.finWrite]));
    }
    PutBits(&op, SpaceOut<rigExecRig::PointFrame>(node, 0));
    PutBits(&op, SpaceOut<rigExecRig::PointFrame>(node, 1));
    const auto &opBase =
        SpaceOut<rigExecStd::RigExecArray<rigExecRig::PointFrame>>(node, 2);
    const auto &opFinal =
        SpaceOut<rigExecStd::RigExecArray<rigExecRig::PointFrame>>(node, 3);
    for (size_t k = 0; k < opBase.Size() && k < opFinal.Size(); ++k) {
        PutBits(&op, opBase[k]);
        PutBits(&op, opFinal[k]);
    }
    CompareBits("Spaces", "ProviderRefresh", object, baked, op, false);
}

// SpaceExpression part 0: a provider program operation replayed on the
// values its inputs hold after the frame, against the value it published.
rigExecStd::Mat4d
OpMatrix(const GfMatrix4d &m)
{
    rigExecStd::Mat4d out;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out._mtx[r][c] = m[r][c];
    return out;
}

// MoverTransform: each revision's transform as its fold read it (provider
// table, constraint delta, AtPrim record) replayed through the op, against
// the transform the fold left.
void
ProbeMoverTransforms(const B &b)
{
    using namespace rigExecOps;
    for (const auto &chain : b.chains) {
        if (!chain.haveBase)
            continue;
        for (const auto &revision : chain.revisions) {
            if (!revision.haveTransform || revision.gpuOwned ||
                revision.rangeSetAside)
                continue;
            const auto table = [&](int slot) {
                return revision.finalPhase ? b.finalMatrix[size_t(slot)]
                                           : b.baseMatrix[size_t(slot)];
            };
            GfMatrix4d transform(1.0);
            if (revision.transformSlot >= 0)
                transform = table(revision.transformSlot);
            if (revision.constraintDelta >= 0 &&
                b.deltaPresent[size_t(revision.constraintDelta)])
                transform = b.deltaValues[size_t(revision.constraintDelta)];
            if (revision.binding.transformPhase.kind ==
                RigExecReadPhaseKind::AtPrim)
                for (const int record : revision.transformRecords)
                    if (b.frameMatrixValid[size_t(record)]) {
                        transform = b.frameMatrix[size_t(record)];
                        break;
                    }
            const bool hasSpace = revision.transformSpaceSlot >= 0;
            const bool hasReference =
                revision.op == RigExecRevisionOp::Matrix &&
                !revision.influenceSlots.empty();
            const bool hasReferenceSpace =
                hasSpace && hasReference && revision.influenceSlots.size() > 1;
            MoverTransformPayload payload;
            payload.posedPoints = revision.posedPoints;
            std::vector<const RigExecPortHandle *> ins = {
                &MoverTransformTransform()};
            std::vector<rigExecStd::Mat4d> values = {OpMatrix(transform)};
            std::string key = "MoverTransform/";
            const auto feed = [&](bool fed, const RigExecPortHandle &port,
                                  const GfMatrix4d &m, char tag) {
                if (!fed)
                    return;
                ins.push_back(&port);
                values.push_back(OpMatrix(m));
                key += tag;
            };
            feed(hasSpace, MoverTransformSpace(),
                 hasSpace ? table(revision.transformSpaceSlot) : transform,
                 's');
            feed(hasReference, MoverTransformReference(),
                 hasReference ? table(revision.influenceSlots[0]) : transform,
                 'r');
            feed(hasReferenceSpace, MoverTransformReferenceSpace(),
                 hasReferenceSpace ? table(revision.influenceSlots[1])
                                   : transform,
                 'R');
            feed(revision.haveCarry, MoverTransformCarry(), revision.carry,
                 'c');
            key += payload.posedPoints ? "/posed" : "/rest";
            SpaceNode &node = SpaceNodeFor(key, MoverTransform(), &payload, ins,
                                           {&MoverTransformOut()});
            std::vector<const void *> bound;
            for (const auto &value : values)
                bound.push_back(&value);
            std::string error;
            const std::string kind =
                std::string("MoverTransform/") + OpName(revision.op);
            if (!RunSpaceNode(node, bound, &error)) {
                OpFailed("Transform", kind, revision.moverPathText, false,
                         error);
                continue;
            }
            std::vector<double> baked, op;
            PutBits(&baked, revision.transform);
            PutBits(&op, SpaceOut<rigExecStd::Mat4d>(node, 0));
            CompareBits("Transform", kind, revision.moverPathText, baked, op,
                        false);
        }
    }
}

rigExecStd::Vec3d
OpVector(const GfVec3d &v)
{
    return rigExecStd::Vec3d(v[0], v[1], v[2]);
}

const char *
ProviderKindName(RigExecProviderOpKind kind)
{
    using K = RigExecProviderOpKind;
    switch (kind) {
    case K::Attribute:
        return "Attribute";
    case K::SpaceExpression:
        return "SpaceExpression";
    case K::RestFrame:
        return "RestFrame";
    case K::DefaultSpace:
        return "DefaultSpace";
    case K::FrameToSpace:
        return "FrameToSpace";
    case K::MatrixToFrame:
        return "MatrixToFrame";
    case K::PosedFrame:
        return "PosedFrame";
    case K::JointMatrix:
        return "JointMatrix";
    case K::LocalXform:
        return "LocalXform";
    case K::InterveningXform:
        return "InterveningXform";
    case K::AvarMatrix:
        return "AvarMatrix";
    case K::RelativeXform:
        return "RelativeXform";
    case K::AffineFrame:
        return "AffineFrame";
    }
    return "Unknown";
}

/// Whether the order value \p id holds resets the xform stack.
bool
OrderResets(const B &b, RigExecValueId id)
{
    if (id >= b.providerValues.values.size())
        return false;
    const auto &order = b.providerValues.values[size_t(id)];
    if (!order.initialized || order.blocked ||
        !order.raw.IsHolding<VtTokenArray>())
        return false;
    for (const auto &token : order.raw.UncheckedGet<VtTokenArray>())
        if (token == "!resetXformStack!")
            return true;
    return false;
}

template <class T>
std::string
SlotKey(const std::vector<T> &slots)
{
    std::string key;
    for (const T slot : slots)
        key += "/" + std::to_string(slot);
    return key;
}

void
ProbeProviderArithmetic(const B &b, const RigExecProviderOp &op,
                        const std::string &kind, const std::string &object)
{
    using namespace rigExecOps;
    using C = ProviderInputClass;
    const auto &values = b.providerValues;
    ProviderArithmeticPayload payload;
    payload.kind = op.kind;
    payload.scaleAvars = op.scaleAvars;
    std::vector<rigExecStd::Mat4d> matrices;
    std::vector<uint8_t> authority, resets;
    std::vector<rigExecRig::PointFrame> frames;
    std::vector<double> scalars;
    std::vector<rigExecStd::Vec3d> vectors;
    std::vector<int32_t> orders;
    for (size_t p = 0; p < op.inputs.size(); ++p) {
        const RigExecValueId id = op.inputs[p];
        int32_t slot = -1;
        switch (ProviderArithmeticInputClass(op.kind, p)) {
        case C::Matrix:
            if (const auto *m = values.Read<GfMatrix4d>(id)) {
                slot = int32_t(matrices.size());
                matrices.push_back(OpMatrix(*m));
                authority.push_back(values.values[size_t(id)].authoritative);
            }
            break;
        case C::Frame:
            if (const auto *f = values.Read<RigExecPointFrame>(id)) {
                slot = int32_t(frames.size());
                frames.push_back(OpFrame(*f));
            }
            break;
        case C::Scalar:
            if (const auto *s = values.Read<double>(id)) {
                slot = int32_t(scalars.size());
                scalars.push_back(*s);
            }
            break;
        case C::Vector:
            if (const auto *v = values.Read<GfVec3d>(id)) {
                slot = int32_t(vectors.size());
                vectors.push_back(OpVector(*v));
            }
            break;
        case C::Order:
            if (const auto *t = values.Read<TfToken>(id)) {
                slot = int32_t(orders.size());
                orders.push_back(ProviderPackOrder(t->GetString()));
            }
            break;
        case C::Reset:
            slot = int32_t(resets.size());
            resets.push_back(OrderResets(b, id) ? 1 : 0);
            break;
        case C::None:
            break;
        }
        payload.slots.push_back(slot);
    }
    SpaceNode &node = SpaceNodeFor(
        "ProviderArithmetic/" + kind + (op.scaleAvars ? "/s" : "/u") +
            SlotKey(payload.slots),
        ProviderArithmetic(), &payload,
        {&ProviderArithmeticMatrices(), &ProviderArithmeticAuthoritative(),
         &ProviderArithmeticFrames(), &ProviderArithmeticScalars(),
         &ProviderArithmeticVectors(), &ProviderArithmeticOrders(),
         &ProviderArithmeticResets()},
        {&ProviderArithmeticMatrix(), &ProviderArithmeticFrame(),
         &ProviderArithmeticAuthoritativeOut()});
    const rigExecStd::RigExecArray<rigExecStd::Mat4d> matricesIn(
        matrices.data(), matrices.size());
    const rigExecStd::RigExecArray<uint8_t> authorityIn(authority.data(),
                                                        authority.size());
    const rigExecStd::RigExecArray<rigExecRig::PointFrame> framesIn(
        frames.data(), frames.size());
    const rigExecStd::RigExecArray<double> scalarsIn(scalars.data(),
                                                     scalars.size());
    const rigExecStd::RigExecArray<rigExecStd::Vec3d> vectorsIn(vectors.data(),
                                                                vectors.size());
    const rigExecStd::RigExecArray<int32_t> ordersIn(orders.data(),
                                                     orders.size());
    const rigExecStd::RigExecArray<uint8_t> resetsIn(resets.data(),
                                                     resets.size());
    std::string error;
    if (!RunSpaceNode(node,
                      {&matricesIn, &authorityIn, &framesIn, &scalarsIn,
                       &vectorsIn, &ordersIn, &resetsIn},
                      &error)) {
        OpFailed("Spaces", kind, object, false, error);
        return;
    }
    std::vector<double> baked, out;
    const auto &published = values.values[size_t(op.output)];
    if (const auto *m = values.Read<GfMatrix4d>(op.output)) {
        PutBits(&baked, *m);
        PutBits(&out, SpaceOut<rigExecStd::Mat4d>(node, 0));
    } else if (const auto *f = values.Read<RigExecPointFrame>(op.output)) {
        PutBits(&baked, OpFrame(*f));
        PutBits(&out, SpaceOut<rigExecRig::PointFrame>(node, 1));
    }
    baked.push_back(published.authoritative ? 1.0 : 0.0);
    out.push_back(SpaceOut<uint8_t>(node, 2) ? 1.0 : 0.0);
    CompareBits("Spaces", kind, object, baked, out, false);
}

void
ProbeAffineFrame(const B &b, const RigExecProviderOp &op,
                 const std::string &kind, const std::string &object)
{
    using namespace rigExecOps;
    using C = AffineInputClass;
    const auto &values = b.providerValues;
    const auto boxed = [&](RigExecValueId id) {
        return values.Read<VtValue>(id);
    };
    AffineFramePayload payload;
    payload.kind = op.affineKind;
    payload.targets = op.affineTargets;
    std::vector<rigExecStd::Mat4d> matrices, matrixItems;
    std::vector<double> doubles, doubleItems;
    std::vector<uint8_t> bools;
    std::vector<int32_t> ints, intItems, spans;
    std::vector<rigExecStd::Vec3d> vectors;
    std::vector<rigExecStd::Vec3i> intVectors;
    std::vector<rigExecRig::PointFrame> frames;
    const auto span = [&](size_t begin, size_t count) {
        const int32_t slot = int32_t(spans.size() / 2);
        spans.push_back(int32_t(begin));
        spans.push_back(int32_t(count));
        return slot;
    };
    for (size_t p = 0; p < op.inputs.size(); ++p) {
        const RigExecValueId id = op.inputs[p];
        const VtValue *box = boxed(id);
        int32_t slot = -1;
        switch (AffineFrameInputClass(op.affineKind, p)) {
        case C::Matrix:
            if (const auto *m = values.Read<GfMatrix4d>(id)) {
                slot = int32_t(matrices.size());
                matrices.push_back(OpMatrix(*m));
            }
            break;
        case C::Double:
            if (const auto *d = values.Read<double>(id)) {
                slot = int32_t(doubles.size());
                doubles.push_back(*d);
            } else if (const auto *f = values.Read<float>(id)) {
                slot = int32_t(doubles.size());
                doubles.push_back(*f);
            }
            break;
        case C::Bool:
            if (box && box->IsHolding<bool>()) {
                slot = int32_t(bools.size());
                bools.push_back(box->UncheckedGet<bool>() ? 1 : 0);
            }
            break;
        case C::Int:
            if (box && box->IsHolding<int>()) {
                slot = int32_t(ints.size());
                ints.push_back(box->UncheckedGet<int>());
            }
            break;
        case C::Vector:
            if (const auto *v = values.Read<GfVec3d>(id)) {
                slot = int32_t(vectors.size());
                vectors.push_back(OpVector(*v));
            }
            break;
        case C::IntVector:
            if (box && box->IsHolding<GfVec3i>()) {
                const GfVec3i &v = box->UncheckedGet<GfVec3i>();
                slot = int32_t(intVectors.size());
                intVectors.push_back(rigExecStd::Vec3i(v[0], v[1], v[2]));
            }
            break;
        case C::Text:
            if (const auto *t = values.Read<TfToken>(id)) {
                slot = int32_t(payload.texts.size());
                payload.texts.push_back(t->GetString());
            }
            break;
        case C::IntItems:
            if (box && box->IsHolding<VtIntArray>() &&
                !box->UncheckedGet<VtIntArray>().empty()) {
                const auto &a = box->UncheckedGet<VtIntArray>();
                slot = span(intItems.size(), a.size());
                intItems.insert(intItems.end(), a.begin(), a.end());
            }
            break;
        case C::DoubleItems:
            if (box && box->IsHolding<VtDoubleArray>() &&
                !box->UncheckedGet<VtDoubleArray>().empty()) {
                const auto &a = box->UncheckedGet<VtDoubleArray>();
                slot = span(doubleItems.size(), a.size());
                doubleItems.insert(doubleItems.end(), a.begin(), a.end());
            }
            break;
        case C::MatrixItems:
            if (box && box->IsHolding<VtMatrix4dArray>() &&
                !box->UncheckedGet<VtMatrix4dArray>().empty()) {
                const auto &a = box->UncheckedGet<VtMatrix4dArray>();
                slot = span(matrixItems.size(), a.size());
                for (const GfMatrix4d &m : a)
                    matrixItems.push_back(OpMatrix(m));
            }
            break;
        case C::Frame:
            if (const auto *f = values.Read<RigExecPointFrame>(id)) {
                slot = int32_t(frames.size());
                frames.push_back(OpFrame(*f));
            }
            break;
        }
        payload.slots.push_back(slot);
    }
    std::string key = "AffineFrame/" + std::to_string(op.affineKind) + "/" +
                      std::to_string(op.affineTargets) + SlotKey(payload.slots);
    for (const std::string &text : payload.texts)
        key += "/" + text;
    SpaceNode &node = SpaceNodeFor(
        key, AffineFrame(), &payload,
        {&AffineFrameMatrices(), &AffineFrameDoubles(), &AffineFrameBools(),
         &AffineFrameInts(), &AffineFrameVectors(), &AffineFrameIntVectors(),
         &AffineFrameIntItems(), &AffineFrameDoubleItems(),
         &AffineFrameMatrixItems(), &AffineFrameItemSpans(),
         &AffineFrameFrames()},
        {&AffineFrameOut()});
    const rigExecStd::RigExecArray<rigExecStd::Mat4d> matricesIn(
        matrices.data(), matrices.size());
    const rigExecStd::RigExecArray<double> doublesIn(doubles.data(),
                                                     doubles.size());
    const rigExecStd::RigExecArray<uint8_t> boolsIn(bools.data(), bools.size());
    const rigExecStd::RigExecArray<int32_t> intsIn(ints.data(), ints.size());
    const rigExecStd::RigExecArray<rigExecStd::Vec3d> vectorsIn(vectors.data(),
                                                                vectors.size());
    const rigExecStd::RigExecArray<rigExecStd::Vec3i> intVectorsIn(
        intVectors.data(), intVectors.size());
    const rigExecStd::RigExecArray<int32_t> intItemsIn(intItems.data(),
                                                       intItems.size());
    const rigExecStd::RigExecArray<double> doubleItemsIn(doubleItems.data(),
                                                         doubleItems.size());
    const rigExecStd::RigExecArray<rigExecStd::Mat4d> matrixItemsIn(
        matrixItems.data(), matrixItems.size());
    const rigExecStd::RigExecArray<int32_t> spansIn(spans.data(), spans.size());
    const rigExecStd::RigExecArray<rigExecRig::PointFrame> framesIn(
        frames.data(), frames.size());
    const auto *published = values.Read<GfMatrix4d>(op.output);
    const bool bakedFailed =
        !published && values.values[size_t(op.output)].error.rfind(
                          "affine frame expression failed", 0) == 0;
    std::string error;
    if (!RunSpaceNode(node,
                      {&matricesIn, &doublesIn, &boolsIn, &intsIn, &vectorsIn,
                       &intVectorsIn, &intItemsIn, &doubleItemsIn,
                       &matrixItemsIn, &spansIn, &framesIn},
                      &error)) {
        OpFailed("Spaces", kind, object, bakedFailed, error);
        return;
    }
    if (bakedFailed) {
        Report("Spaces/" + kind, object,
               "the op ran where the baked expression failed");
        return;
    }
    std::vector<double> baked, out;
    PutBits(&baked, *published);
    PutBits(&out, SpaceOut<rigExecStd::Mat4d>(node, 0));
    CompareBits("Spaces", kind, object, baked, out, false);
}

// A provider value's answer: ProviderSelect over the candidate flags read
// from what the baked program holds, against what the step published.
void
ProbeProviderAnswer(const B &b, eval::spaces::ProviderProducer part, size_t row,
                    const std::string &kind, const std::string &object)
{
    using namespace rigExecOps;
    using Producer = eval::spaces::ProviderProducer;
    const auto &program = b.GetProviderProgram();
    std::vector<uint8_t> present, authoritative, blocked;
    const auto add = [&](bool p, bool a, bool k) {
        present.push_back(p ? 1 : 0);
        authoritative.push_back(a ? 1 : 0);
        blocked.push_back(k ? 1 : 0);
    };
    RigExecValueId value = RigExecNoProviderValue;
    switch (part) {
    case Producer::Operation: {
        const auto &op = program.ops[row];
        value = op.output;
        for (size_t k = 0; k < std::min<size_t>(op.inputs.size(), 2); ++k) {
            const RigExecValueId id = op.inputs[k];
            if (k && id == RigExecNoProviderValue)
                break;
            const auto *state = id < b.providerValues.values.size()
                                    ? &b.providerValues.values[size_t(id)]
                                    : nullptr;
            add(state && state->initialized, state && state->authoritative,
                state && state->blocked);
        }
        break;
    }
    case Producer::Sampled: {
        value = program.sampled[row].value;
        const bool overlaid = RigExecBakedSpaceLeafOverlay(b, row) != nullptr;
        add(true, overlaid, !overlaid && b.providerLeafBlocked[row]);
        break;
    }
    case Producer::Routed:
        value = program.routedInputs[row].value;
        add(true, true,
            !RigExecBakedCrossDomainAvailable(b, b.providerRoutedReads[row],
                                              nullptr));
        break;
    case Producer::External:
        value = program.externalInputs[row].value;
        add(true, false, false);
        break;
    case Producer::PoseFrame:
        value = b.providerFrameInputs[row].value;
        add(true, false, false);
        break;
    }
    SpaceNode &node = SpaceNodeFor(
        "ProviderSelect", ProviderSelect(), nullptr,
        {&ProviderSelectPresent(), &ProviderSelectAuthoritative(),
         &ProviderSelectBlocked()},
        {&ProviderSelectSelected(), &ProviderSelectAuthoritativeOut(),
         &ProviderSelectBlockedOut()});
    const rigExecStd::RigExecArray<uint8_t> presentIn(present.data(),
                                                      present.size()),
        authoritativeIn(authoritative.data(), authoritative.size()),
        blockedIn(blocked.data(), blocked.size());
    std::string error;
    if (!RunSpaceNode(node, {&presentIn, &authoritativeIn, &blockedIn},
                      &error)) {
        OpFailed("Spaces", kind, object, false, error);
        return;
    }
    const auto &published = b.providerValues.values[size_t(value)];
    const std::vector<double> baked = {double(published.authoritative),
                                       double(published.blocked)},
                              out = {double(SpaceOut<uint8_t>(node, 1)),
                                     double(SpaceOut<uint8_t>(node, 2))};
    CompareBits("Spaces", kind, object, baked, out, false);
}

void
ProbeSpaceExpression(const B &b, const RigExecBakedStep &step)
{
    using Producer = eval::spaces::ProviderProducer;
    if (step.part != 0) {
        const char *names[] = {"", "External", "Sampled", "Routed",
                               "PoseFrame"};
        ProbeProviderAnswer(b, Producer(step.part), size_t(step.object),
                            std::string("SpaceExpression/") +
                                names[size_t(step.part)],
                            step.label);
        return;
    }
    const auto &program = b.GetProviderProgram();
    if (step.object < 0 || size_t(step.object) >= program.ops.size())
        return;
    const auto &op = program.ops[size_t(step.object)];
    const std::string kind =
        std::string("SpaceExpression/") + ProviderKindName(op.kind);
    if (op.kind == RigExecProviderOpKind::Attribute) {
        ProbeProviderAnswer(b, Producer::Operation, size_t(step.object), kind,
                            step.label);
        return;
    }
    const auto &published = b.providerValues.values[size_t(op.output)];
    if (op.kind != RigExecProviderOpKind::AffineFrame &&
        (!published.initialized || published.blocked ||
         std::holds_alternative<std::monostate>(published.value))) {
        Unprobed("Spaces", kind, "the baked op published no value");
        return;
    }
    const std::string object = step.label;
    if (op.kind == RigExecProviderOpKind::AffineFrame)
        ProbeAffineFrame(b, op, kind, object);
    else
        ProbeProviderArithmetic(b, op, kind, object);
}

void
ProbeSpaces(const B &b)
{
    for (const auto &step : b.steps) {
        const RigExecBakedStepIdentity id = RigExecBakedIdentityOf(b, step);
        RigExecBakedStep identity;
        identity.object = id.object;
        identity.part = id.part;
        if (id.kind == RigExecBakedStepKind::ProviderMatrix)
            ProbeProviderMatrix(b, identity);
        else if (id.kind == RigExecBakedStepKind::FrameMatrix)
            ProbeFrameMatrix(b, identity);
        else if (id.kind == RigExecBakedStepKind::ProviderRefresh) {
            RigExecBakedStep probe = step;
            probe.object = id.object;
            ProbeProviderRefresh(b, probe);
        } else if (id.kind == RigExecBakedStepKind::SpaceExpression) {
            RigExecBakedStep probe = step;
            probe.object = id.object;
            probe.part = id.part;
            ProbeSpaceExpression(b, probe);
        }
    }
}

// Pose: a provider's bound avar values, read as its AvarInputs step reads
// them, against the avars the step wrote.
void
ProbeAvarInputs(const B &b, int provider)
{
    using namespace rigExecOps;
    const auto varying =
        RigExecBakedAvarBindingRange(b.avarBindingBegin, provider);
    const auto constant =
        RigExecBakedAvarBindingRange(b.avarConstantBindingBegin, provider);
    AvarInputsPayload payload;
    std::vector<uint32_t> slots;
    rigExecStd::RigExecArray<double> values;
    std::vector<double> read;
    auto bind = [&](const auto &binding) {
        const auto at =
            std::find(slots.begin(), slots.end(), uint32_t(binding.slot));
        payload.targets.push_back(uint32_t(at - slots.begin()));
        if (at == slots.end())
            slots.push_back(uint32_t(binding.slot));
        read.push_back(RigExecBakedLeafRead(b, binding.input));
    };
    for (size_t i = varying.first; i < varying.second; ++i)
        bind(b.avarBindings[i]);
    for (size_t i = constant.first; i < constant.second; ++i)
        bind(b.avarConstantBindings[i]);
    if (slots.empty())
        return;
    payload.avarCount = slots.size();
    values.Resize(read.size());
    for (size_t i = 0; i < read.size(); ++i)
        values[i] = read[i];
    std::string key = "AvarInputs";
    for (const uint32_t t : payload.targets)
        key += "/" + std::to_string(t);
    SpaceNode &node = SpaceNodeFor(key, AvarInputs(), &payload,
                                   {&AvarInputsValues()}, {&AvarInputsOut()});
    std::string error;
    const std::string object = b.paths[size_t(slots.front()) / 11].GetString();
    if (!RunSpaceNode(node, {&values}, &error)) {
        OpFailed("Pose", "AvarInputs", object, false, error);
        return;
    }
    const auto &out = SpaceOut<rigExecStd::RigExecArray<double>>(node, 0);
    std::vector<double> baked, op(out.begin(), out.end());
    for (const uint32_t slot : slots)
        baked.push_back(b.avars[slot]);
    CompareBits("Pose", "AvarInputs", object, baked, op, false);
}

// A space checkpoint's switch frame from the slots it recomposes and its
// anchor's posed matrix, against the frame the step stored.
void
ProbeSpaceCheckpoint(const B &b, size_t object)
{
    using namespace rigExecOps;
    const auto &context = b.switchFrameContexts[object];
    SpaceCheckpointPayload payload;
    rigExecStd::RigExecArray<double> avars;
    std::vector<uint32_t> orders, signs;
    std::vector<rigExecStd::Mat4d> posedD, parentDinv, parentM, posedM;
    std::vector<uint8_t> parentAuthored, posedAuthored;
    std::vector<double> a;
    for (const int slot : context.recompose) {
        const size_t s = size_t(slot);
        const TfToken &order = b.rotOrder[s];
        orders.push_back(
            SpaceCheckpointRotOrderCode(order.GetText(), order.size()));
        signs.push_back(s < b.rotationSign.size() ? b.rotationSign[s] : 0u);
        payload.noScale.push_back(b.noScaleAvars[s] != 0);
        a.insert(a.end(), &b.avars[s * 11], &b.avars[s * 11] + 11);
        posedD.push_back(OpMatrix(b.posedD[s]));
        parentDinv.push_back(OpMatrix(b.parentDinv[s]));
        parentM.push_back(OpMatrix(b.parentSpaceM[s]));
        parentAuthored.push_back(b.parentSpaceAuthored[s] != 0);
        posedAuthored.push_back(b.posedAuthored[s] != 0);
        posedM.push_back(OpMatrix(b.posedAuthoredM[s]));
    }
    auto array = [](const auto &values) {
        rigExecStd::RigExecArray<std::decay_t<decltype(values[0])>> out(
            values.size());
        for (size_t i = 0; i < values.size(); ++i)
            out[i] = values[i];
        return out;
    };
    const auto avarArray = array(a);
    const auto orderArray = array(orders), signArray = array(signs);
    const auto posedDArray = array(posedD), parentDinvArray = array(parentDinv),
               parentMArray = array(parentM), posedMArray = array(posedM);
    const auto parentAuthoredArray = array(parentAuthored),
               posedAuthoredArray = array(posedAuthored);
    std::vector<const RigExecPortHandle *> ins = {
        &SpaceCheckpointAvars(),
        &SpaceCheckpointRotOrders(),
        &SpaceCheckpointRotationSigns(),
        &SpaceCheckpointPosedDefault(),
        &SpaceCheckpointParentDefaultInverse(),
        &SpaceCheckpointParentExpression(),
        &SpaceCheckpointParentExpressionAuthored(),
        &SpaceCheckpointPosedAuthored(),
        &SpaceCheckpointPosedAuthoredMatrix()};
    std::vector<const void *> values = {
        &avarArray,           &orderArray,         &signArray,
        &posedDArray,         &parentDinvArray,    &parentMArray,
        &parentAuthoredArray, &posedAuthoredArray, &posedMArray};
    rigExecStd::Mat4d anchor;
    if (context.anchor >= 0) {
        anchor = OpMatrix(b.posedM[size_t(context.anchor)]);
        ins.push_back(&SpaceCheckpointAnchor());
        values.push_back(&anchor);
    }
    std::string key = "SpaceCheckpoint/" + std::to_string(context.anchor >= 0);
    for (const uint8_t noScale : payload.noScale)
        key += "/" + std::to_string(noScale);
    SpaceNode &node = SpaceNodeFor(key, SpaceCheckpoint(), &payload, ins,
                                   {&SpaceCheckpointOut()});
    std::string error;
    const std::string label = "checkpoint " + std::to_string(object);
    if (!RunSpaceNode(node, values, &error)) {
        OpFailed("Pose", "SpaceCheckpoint", label, false, error);
        return;
    }
    std::vector<double> baked, op;
    PutBits(&baked, b.switchFrames[object]);
    PutBits(&op, SpaceOut<rigExecStd::Mat4d>(node, 0));
    CompareBits("Pose", "SpaceCheckpoint", label, baked, op, false);
}

// A pose interpolator's weights from the frames, values and enable its step
// read, against the weights it wrote.
void
ProbePoseInterpolator(const B &b, size_t index, const RigExecBakedStep &step)
{
    using namespace rigExecOps;
    const auto &interp = b.poseInterpolators[index];
    PoseInterpolatorPayload payload;
    payload.poseCount = interp.poseSlots.size();
    payload.enableTranslation = interp.enableTranslation;
    payload.allowNegativeWeights = interp.allowNegativeWeights;
    std::string kind = "PoseInterpolator Rbf";
    if (interp.mapsSolve && interp.maps) {
        kind = "PoseInterpolator Maps";
        payload.solve = PoseSolve::Maps;
        const RigExecAnimatedMapsTable &table = *interp.maps;
        payload.mapsInputCount = table.inputCount;
        for (const RigExecConditionalRow &row : table.rows) {
            payload.mapRowInputs.push_back(int32_t(row.input));
            payload.mapRowFrom.push_back(row.from);
            payload.mapRowTo.push_back(row.to);
            payload.mapRowSlope.push_back(row.slope);
            payload.mapRowCut.push_back(row.cut);
        }
        for (const size_t begin : table.mapBegin)
            payload.mapBegin.push_back(begin);
        for (size_t m = 0; m < table.MapCount(); ++m)
            payload.mapUnclamped.push_back(table.Clamps(m) ? 0 : 1);
    } else if (interp.mlSolve && interp.network) {
        kind = "PoseInterpolator Network";
        payload.solve = PoseSolve::Network;
        const RigExecMlNetwork &net = *interp.network;
        payload.networkInputCount = net.inputCount;
        for (const RigExecMlLayer &layer : net.layers) {
            payload.networkLayerSizes.push_back(int32_t(layer.outputCount));
            payload.networkActivations.push_back(uint32_t(layer.activation));
            payload.networkAlphas.push_back(float(layer.alpha));
        }
        const size_t biasBegin =
            net.layers.empty() ? 0 : net.layers.front().biasOffset;
        payload.networkWeights.assign(net.parameters.begin(),
                                      net.parameters.begin() + biasBegin);
        payload.networkBiases.assign(net.parameters.begin() + biasBegin,
                                     net.parameters.end());
        payload.networkInputMean = net.inputMean;
        payload.networkInputScale = net.inputScale;
        payload.networkDefaults = net.defaults;
    } else if (interp.rigLogicSolve) {
        kind = "PoseInterpolator RbfMethod";
        payload.solve = PoseSolve::RbfMethod;
        const RigExecRbfMethodTable &table = interp.method;
        payload.methodSolverType = uint32_t(table.solverType);
        payload.methodDistance = uint32_t(table.distanceMethod);
        payload.methodWeightFunction = uint32_t(table.weightFunction);
        payload.methodNormalize = uint32_t(table.normalizeMethod);
        payload.methodTwistAxis = uint32_t(table.twistAxis);
        payload.methodRawCount = table.rawCount;
        payload.methodTargetCount = table.targetCount;
        payload.methodRadius = table.radius;
        payload.methodWeightThreshold = table.weightThreshold;
        payload.methodTargets = table.targets;
        payload.methodDistanceTargets = table.distanceTargets;
        payload.methodTargetScales = table.targetScales;
        payload.methodCoefficients = table.coefficients;
    } else {
        const RigExecRbfSolver &solver = interp.solver;
        for (const GfVec3d &p : solver.GetPoses())
            payload.rbfPoses.push_back(OpVector(p));
        for (const GfVec3d &t : solver.GetTranslations())
            payload.rbfTranslations.push_back(OpVector(t));
        for (const RigExecRbfPoseType type : solver.GetPoseTypes())
            payload.rbfPoseTypes.push_back(uint32_t(type));
        payload.rbfTwistAxis = OpVector(solver.GetTwistAxis());
        payload.rbfKernel = uint32_t(solver.GetKernel());
        payload.rbfRadius = solver.GetRadius();
        payload.rbfTranslationRadius = solver.GetTranslationRadius();
        payload.rbfRadii = solver.GetRadii();
        payload.rbfTranslationRadii = solver.GetTranslationRadii();
        payload.rbfNormalize = solver.GetNormalize();
        payload.rbfEnableRotation = solver.GetEnableRotation();
        payload.rbfEnableTranslation = solver.GetEnableTranslation();
        payload.rbfWeightRows = solver.GetWeights().size();
        for (const auto &row : solver.GetWeights())
            payload.rbfWeights.insert(payload.rbfWeights.end(), row.begin(),
                                      row.end());
    }
    if (payload.solve == PoseSolve::Rbf && payload.rbfEnableTranslation)
        kind += " translation";
    const bool extras = payload.solve == PoseSolve::RbfMethod ||
                        payload.solve == PoseSolve::Network;
    std::vector<const RigExecPortHandle *> ins = {&PoseInterpolatorEnabled()};
    const uint8_t enabled = RigExecBakedLeafRead(b, interp.enabled) ? 1 : 0;
    std::vector<const void *> values = {&enabled};
    rigExecStd::RigExecArray<double> numeric(interp.valueInputs.size());
    for (size_t i = 0; i < interp.valueInputs.size(); ++i)
        numeric[i] =
            i < interp.valueIsFloat.size() && interp.valueIsFloat[i]
                ? double(RigExecBakedLeafRead(b, interp.floatValueInputs[i]))
                : RigExecBakedLeafRead(b, interp.valueInputs[i]);
    if (!interp.valueInputs.empty()) {
        ins.push_back(&PoseInterpolatorNumeric());
        values.push_back(&numeric);
    }
    const double mask = payload.solve == PoseSolve::Network
                            ? double(RigExecBakedLeafRead(b, interp.mask))
                            : 1.0;
    if (payload.solve == PoseSolve::Network) {
        ins.push_back(&PoseInterpolatorMask());
        values.push_back(&mask);
    }
    auto finalOf = [&](int slot) {
        return OpFrame(b.fin[size_t(b.finLast[size_t(slot)])]);
    };
    auto restOf = [&](int slot) { return OpFrame(b.restFrames[size_t(slot)]); };
    rigExecRig::PointFrame frames[4];
    if (interp.driverSlot >= 0) {
        frames[0] = finalOf(interp.driverSlot);
        frames[1] = restOf(interp.driverSlot);
        ins.push_back(&PoseInterpolatorDriverFinal());
        ins.push_back(&PoseInterpolatorDriverRest());
        values.push_back(&frames[0]);
        values.push_back(&frames[1]);
        if (interp.parentSlot >= 0) {
            frames[2] = finalOf(interp.parentSlot);
            frames[3] = restOf(interp.parentSlot);
            ins.push_back(&PoseInterpolatorParentFinal());
            ins.push_back(&PoseInterpolatorParentRest());
            values.push_back(&frames[2]);
            values.push_back(&frames[3]);
        }
    }
    rigExecStd::RigExecArray<rigExecRig::PointFrame> extraLists[4];
    if (extras && !interp.extraDriverSlots.empty()) {
        std::vector<rigExecRig::PointFrame> lists[4];
        for (size_t e = 0; e < interp.extraDriverSlots.size(); ++e) {
            lists[0].push_back(finalOf(interp.extraDriverSlots[e]));
            lists[1].push_back(restOf(interp.extraDriverSlots[e]));
            const int parent = interp.extraParentSlots[e];
            payload.extraParents.push_back(parent >= 0);
            if (parent >= 0) {
                lists[2].push_back(finalOf(parent));
                lists[3].push_back(restOf(parent));
            }
        }
        const RigExecPortHandle *ports[4] = {
            &PoseInterpolatorExtraFinals(), &PoseInterpolatorExtraRests(),
            &PoseInterpolatorExtraParentFinals(),
            &PoseInterpolatorExtraParentRests()};
        for (int l = 0; l < 4; ++l) {
            if (lists[l].empty())
                continue;
            extraLists[l] = rigExecStd::RigExecArray<rigExecRig::PointFrame>(
                lists[l].size());
            for (size_t i = 0; i < lists[l].size(); ++i)
                extraLists[l][i] = lists[l][i];
            ins.push_back(ports[l]);
            values.push_back(&extraLists[l]);
        }
    }
    const std::string object = interp.pathText;
    std::string key = "PoseInterpolator/" +
                      g_where.substr(0, g_where.rfind('@')) + "/" + object;
    SpaceNode &node = SpaceNodeFor(key, PoseInterpolator(), &payload, ins,
                                   {&PoseInterpolatorOut()});
    std::string error;
    if (!RunSpaceNode(node, values, &error)) {
        OpFailed("Pose", kind, object, !step.diagnostics.empty(), error);
        return;
    }
    const auto &out = SpaceOut<rigExecStd::RigExecArray<float>>(node, 0);
    std::vector<double> baked, op;
    for (const int slot : interp.poseSlots)
        baked.push_back(double(b.poseWeights[size_t(slot)]));
    for (const float w : out)
        op.push_back(double(w));
    CompareBits("Pose", kind, object, baked, op, !step.diagnostics.empty());
}

// Rest, ladder and compose of one slot each, from the values their steps
// read, against what the steps stored.
template <class T>
rigExecStd::RigExecArray<T>
OpArray(const std::vector<T> &values)
{
    rigExecStd::RigExecArray<T> out(values.size());
    for (size_t i = 0; i < values.size(); ++i)
        out[i] = values[i];
    return out;
}

void
ProbeRestCompose(const B &b, size_t g)
{
    using namespace rigExecOps;
    const RigExecBakedComposeGroup &group = b.composeGroups[g];
    const BakedPoseInputs in(&b);
    for (int i = group.begin; i < group.end; ++i) {
        const size_t slot = size_t(i);
        if (!in.isPoseSlot(slot))
            continue;
        SlotActivityPayload payload;
        payload.active = in.providerActive(slot);
        std::vector<double> avars;
        for (size_t k = 0; k < 6; ++k)
            avars.push_back(in.restAvar(slot, k));
        const auto avarArray = OpArray(avars);
        const rigExecStd::Mat4d restSpace =
            OpMatrix(in.ladderSpace(slot, eval::kPoseLadderRestSpace));
        const rigExecStd::Mat4d intervening =
            OpMatrix(in.interveningSpace(slot));
        std::vector<const RigExecPortHandle *> ins = {
            &RestComposeRestAvars(), &RestComposeRestSpace(),
            &RestComposeIntervening()};
        std::vector<const void *> values = {&avarArray, &restSpace,
                                            &intervening};
        const int parent = in.parent(slot);
        rigExecStd::Mat4d parentRest;
        if (parent >= 0) {
            parentRest = OpMatrix(b.restRoundTrip[size_t(parent)]);
            ins.push_back(&RestComposeParentRest());
            values.push_back(&parentRest);
        }
        const std::string key = "RestCompose/" +
                                std::to_string(payload.active) + "/" +
                                std::to_string(parent >= 0);
        SpaceNode &node =
            SpaceNodeFor(key, RestCompose(), &payload, ins,
                         {&RestComposeRestMatrix(), &RestComposeRest(),
                          &RestComposeRoundTrip()});
        std::string error;
        const std::string object = b.paths[slot].GetString();
        if (!RunSpaceNode(node, values, &error)) {
            OpFailed("Pose", "RestCompose", object, false, error);
            continue;
        }
        std::vector<double> baked, op;
        PutBits(&baked, b.restM[slot]);
        PutBits(&baked, OpFrame(b.restFrames[slot]));
        PutBits(&baked, b.restRoundTrip[slot]);
        PutBits(&op, SpaceOut<rigExecStd::Mat4d>(node, 0));
        PutBits(&op, SpaceOut<rigExecRig::PointFrame>(node, 1));
        PutBits(&op, SpaceOut<rigExecStd::Mat4d>(node, 2));
        CompareBits("Pose", "RestCompose", object, baked, op, false);
    }
}

void
ProbeLadderCompose(const B &b, size_t g)
{
    using namespace rigExecOps;
    const RigExecBakedComposeGroup &group = b.composeGroups[g];
    const BakedPoseInputs in(&b);
    for (int i = group.begin; i < group.end; ++i) {
        const size_t slot = size_t(i);
        if (!in.isPoseSlot(slot))
            continue;
        SlotActivityPayload payload;
        payload.active = in.providerActive(slot);
        std::vector<rigExecStd::Mat4d> spaces;
        std::vector<uint8_t> authored;
        std::vector<double> avars;
        for (int c = eval::kPoseLadderDefaultSpace;
             c <= eval::kPoseLadderPosedDefaultSpace; ++c) {
            const bool defaultChannel =
                c == eval::kPoseLadderDefaultSpace ||
                c == eval::kPoseLadderParentDefaultSpace ||
                c == eval::kPoseLadderAvarDefaultSpace ||
                c == eval::kPoseLadderPosedDefaultSpace;
            spaces.push_back(OpMatrix(in.ladderSpace(slot, c)));
            authored.push_back(
                (defaultChannel && in.ladderChannelReady(slot, c)) ||
                in.ladderAuthoritative(slot, c));
        }
        for (size_t k = 0; k < 6; ++k)
            avars.push_back(in.defaultAvar(slot, k));
        const auto spaceArray = OpArray(spaces);
        const auto authoredArray = OpArray(authored);
        const auto avarArray = OpArray(avars);
        const rigExecStd::Mat4d leaf = OpMatrix(in.ladderParentSpaceLeaf(slot));
        const rigExecStd::Vec3d sign = OpVector(in.ladderRotationSign(slot));
        const rigExecStd::Mat4d ownRest = OpMatrix(b.restRoundTrip[slot]);
        std::vector<const RigExecPortHandle *> ins = {
            &LadderComposeSpaces(),          &LadderComposeAuthored(),
            &LadderComposeParentSpaceLeaf(), &LadderComposeRotationSign(),
            &LadderComposeDefaultAvars(),    &LadderComposeOwnRest()};
        std::vector<const void *> values = {
            &spaceArray, &authoredArray, &leaf, &sign, &avarArray, &ownRest};
        const int parent = in.parent(slot);
        rigExecStd::Mat4d parentRest, parentDefault;
        if (parent >= 0) {
            parentRest = OpMatrix(b.restRoundTrip[size_t(parent)]);
            parentDefault = OpMatrix(b.defaultRoundTrip[size_t(parent)]);
            ins.push_back(&LadderComposeParentRest());
            ins.push_back(&LadderComposeParentDefault());
            values.push_back(&parentRest);
            values.push_back(&parentDefault);
        }
        const std::string key = "LadderCompose/" +
                                std::to_string(payload.active) + "/" +
                                std::to_string(parent >= 0);
        SpaceNode &node = SpaceNodeFor(
            key, LadderCompose(), &payload, ins,
            {&LadderComposeSelfDefault(), &LadderComposeParentDefaultInverse(),
             &LadderComposeDefaultRoundTrip(), &LadderComposePosedDefault(),
             &LadderComposeParentSpace(), &LadderComposePosedAuthoredMatrix(),
             &LadderComposePosedAuthored(), &LadderComposeParentSpaceAuthored(),
             &LadderComposeOutRotationSign()});
        std::string error;
        const std::string object = b.paths[slot].GetString();
        if (!RunSpaceNode(node, values, &error)) {
            OpFailed("Pose", "LadderCompose", object, false, error);
            continue;
        }
        std::vector<double> baked, op;
        for (const auto *table :
             {&b.selfD, &b.parentDinv, &b.defaultRoundTrip, &b.posedD,
              &b.parentSpaceM, &b.posedAuthoredM})
            PutBits(&baked, (*table)[slot]);
        baked.push_back(b.posedAuthored[slot] != 0);
        baked.push_back(b.parentSpaceAuthored[slot] != 0);
        baked.push_back(double(b.rotationSign[slot]));
        for (size_t o = 0; o < 6; ++o)
            PutBits(&op, SpaceOut<rigExecStd::Mat4d>(node, o));
        op.push_back(SpaceOut<uint8_t>(node, 6) != 0);
        op.push_back(SpaceOut<uint8_t>(node, 7) != 0);
        op.push_back(double(SpaceOut<uint32_t>(node, 8)));
        CompareBits("Pose", "LadderCompose", object, baked, op, false);
    }
}

void
ProbePose(const B &b)
{
    for (const auto &step : b.steps) {
        if (const RigExecBakedStepIdentity id = RigExecBakedIdentityOf(b, step);
            id.kind == RigExecBakedStepKind::AvarInputs && id.object >= 0)
            ProbeAvarInputs(b, id.object);
        else if (id.kind == RigExecBakedStepKind::SpaceCheckpoint &&
                 id.object >= 0 &&
                 size_t(id.object) < b.switchFrameContexts.size())
            ProbeSpaceCheckpoint(b, size_t(id.object));
        else if (id.kind == RigExecBakedStepKind::PoseInterpolator &&
                 id.object >= 0 &&
                 size_t(id.object) < b.poseInterpolators.size())
            ProbePoseInterpolator(b, size_t(id.object), step);
        else if (id.kind == RigExecBakedStepKind::RestCompose &&
                 id.object >= 0 && size_t(id.object) < b.composeGroups.size())
            ProbeRestCompose(b, size_t(id.object));
        else if (id.kind == RigExecBakedStepKind::LadderCompose &&
                 id.object >= 0 && size_t(id.object) < b.composeGroups.size())
            ProbeLadderCompose(b, size_t(id.object));
    }
}

std::vector<UsdTimeCode>
Frames(const UsdStageRefPtr &stage)
{
    if (!stage->HasAuthoredTimeCodeRange())
        return {UsdTimeCode(1.0), UsdTimeCode(2.0)};
    const double start = stage->GetStartTimeCode();
    const double end = stage->GetEndTimeCode();
    return {UsdTimeCode(start), UsdTimeCode(std::floor((start + end) / 2)),
            UsdTimeCode(end)};
}

// The point movers each rig of the painted set must have compared under a
// dense and a sparse partial envelope. Matrix, Smooth, DeltaMush, Lattice,
// VolumeCorrect, Wrinkle and EmitGuidePoints revisions run only as op chain
// members (testRigExecOpsPoints).
const char *const kPaintedKinds[] = {
    "Skin",
    "Wire",
    "SurfaceProject",
};

// The radial forms also take a matrix mover's partial weight as a fraction
// of its rotation.
enum class Paint { None, Dense, Sparse, DenseRadial, SparseRadial };

const Paint kPaints[] = {Paint::Dense, Paint::Sparse, Paint::DenseRadial,
                         Paint::SparseRadial};

const char *
PaintName(Paint paint)
{
    switch (paint) {
    case Paint::None:
        return "";
    case Paint::Dense:
        return "[dense]";
    case Paint::Sparse:
        return "[sparse]";
    case Paint::DenseRadial:
        return "[dense radial]";
    case Paint::SparseRadial:
        return "[sparse radial]";
    }
    return "";
}

// Authors, in the session layer, one static weight per point mover of the
// rig at \p rigPath and makes it the mover's envelope: zero, partial and
// full weights over every point, or a sparse field with a zero default
// naming every third point. The radial forms also make each matrix mover
// take a fraction of its rotation. False when no mover was painted.
bool
PaintEnvelopes(const UsdStageRefPtr &stage, const SdfPath &rigPath, Paint paint)
{
    const bool sparse = paint == Paint::Sparse || paint == Paint::SparseRadial;
    const bool radial =
        paint == Paint::DenseRadial || paint == Paint::SparseRadial;
    const TfToken moves(SchemaAttrText::rigExecMoves);
    const TfToken weightObject(SchemaAttrText::rigExecWeightObject);
    UsdEditContext context(stage, stage->GetSessionLayer());
    std::vector<UsdPrim> movers;
    for (const UsdPrim &prim : stage->Traverse())
        if (prim.GetRelationship(moves))
            movers.push_back(prim);
    size_t painted = 0;
    for (const UsdPrim &mover : movers) {
        SdfPathVector targets;
        mover.GetRelationship(moves).GetTargets(&targets);
        if (targets.size() != 1 || !targets[0].IsPropertyPath() ||
            targets[0].GetNameToken() != UsdGeomTokens->points)
            continue;
        const UsdAttribute target = stage->GetAttributeAtPath(targets[0]);
        VtVec3fArray points;
        if (!target || !target.Get(&points, UsdTimeCode::EarliestTime()) ||
            points.empty())
            continue;
        const UsdPrim weight =
            stage->DefinePrim(rigPath.AppendChild(TfToken(
                                  "ProbeEnvelope" + std::to_string(painted++))),
                              TfToken(SchemaTypeName::RigExecStaticWeight));
        const auto uniform = [&](const char *name, const SdfValueTypeName &type,
                                 const VtValue &value) {
            weight
                .CreateAttribute(TfToken(name), type, false,
                                 SdfVariabilityUniform)
                .Set(value);
        };
        uniform(SchemaAttrText::rigExecRepresentation, SdfValueTypeNames->Token,
                VtValue(TfToken(sparse ? SchemaValueText::sparse
                                       : SchemaValueText::dense)));
        uniform(SchemaAttrText::rigExecRangePolicy, SdfValueTypeNames->Token,
                VtValue(TfToken(SchemaValueText::strict)));
        uniform(SchemaAttrText::rigExecDefaultWeight, SdfValueTypeNames->Float,
                VtValue(0.0f));
        const float dense[] = {0.0f, 0.25f, 1.0f, 0.6f, 0.1f, 0.85f};
        const float entries[] = {0.3f, 0.7f, 1.0f, 0.45f};
        VtIntArray indices;
        VtFloatArray values;
        for (size_t i = 0; i < points.size(); ++i) {
            if (!sparse) {
                values.push_back(dense[i % 6]);
            } else if (i % 3 == 0) {
                indices.push_back(int(i));
                values.push_back(entries[(i / 3) % 4]);
            }
        }
        if (sparse)
            uniform(SchemaAttrText::rigExecIndices, SdfValueTypeNames->IntArray,
                    VtValue(indices));
        uniform(SchemaAttrText::rigExecValues, SdfValueTypeNames->FloatArray,
                VtValue(values));
        weight.CreateRelationship(TfToken(SchemaAttrText::rigExecWeightTarget))
            .SetTargets({targets[0]});
        mover.CreateRelationship(weightObject).SetTargets({weight.GetPath()});
        if (radial && mover.GetTypeName() == SchemaTypeName::RigExecMatrixMover)
            mover
                .CreateAttribute(TfToken(SchemaAttrText::rigExecWeightBlend),
                                 SdfValueTypeNames->Token, false,
                                 SdfVariabilityUniform)
                .Set(TfToken("radial"));
    }
    return painted > 0;
}

/// Every frame of the stage's range, at most \p count of them evenly spaced.
std::vector<UsdTimeCode>
DenseFrames(const UsdStageRefPtr &stage, int count)
{
    if (!stage->HasAuthoredTimeCodeRange())
        return Frames(stage);
    const double start = stage->GetStartTimeCode();
    const double end = stage->GetEndTimeCode();
    const int steps = std::max(1, std::min(count - 1, int(end - start)));
    std::vector<UsdTimeCode> out;
    for (int k = 0; k <= steps; ++k)
        out.push_back(
            UsdTimeCode(std::floor(start + (end - start) * k / steps)));
    return out;
}

// SkinMatrices: a skin revision's influence matrices, gathered as its
// InfluenceFold gathers them, folded once by the op, against the table and
// forms the fold wrote; a chunked revision's chunks are compared at their
// own keys.
const TfToken kSkinDualQuaternion("dualQuaternion");

void
PutPalette(std::vector<double> *out, const GfQuatd &real, const GfQuatd &dual,
           const GfMatrix3d &stretch, bool rigid)
{
    for (const GfQuatd *q : {&real, &dual}) {
        out->push_back(q->GetReal());
        for (int a = 0; a < 3; ++a)
            out->push_back(q->GetImaginary()[a]);
    }
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out->push_back(stretch[r][c]);
    out->push_back(rigid ? 1.0 : 0.0);
}

void
PutPalette(std::vector<double> *out, const rigExecStd::Quatd &real,
           const rigExecStd::Quatd &dual, const rigExecStd::Mat3d &stretch,
           bool rigid)
{
    for (const rigExecStd::Quatd *q : {&real, &dual}) {
        out->push_back(q->GetReal());
        for (int a = 0; a < 3; ++a)
            out->push_back(q->GetImaginary()[a]);
    }
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out->push_back(stretch._mtx[r][c]);
    out->push_back(rigid ? 1.0 : 0.0);
}

void
ProbeSkinMatrices(const B &b)
{
    using namespace rigExecOps;
    for (const auto &chain : b.chains) {
        if (!chain.haveBase)
            continue;
        for (const auto &revision : chain.revisions) {
            if (revision.op != RigExecRevisionOp::Skin)
                continue;
            const bool dualQuat =
                revision.parameters.skinningMethod == kSkinDualQuaternion;
            const std::string kind = dualQuat ? "dualQuaternion" : "linear";
            if (revision.gpuOwned) {
                Unprobed("SkinMatrices", kind, "GPU-owned chain");
                continue;
            }
            const bool atPrim = revision.binding.transformPhase.kind ==
                                RigExecReadPhaseKind::AtPrim;
            std::vector<rigExecStd::Mat4d> gathered;
            for (size_t k = 0; k < revision.influenceSlots.size(); ++k) {
                const size_t slot = size_t(revision.influenceSlots[k]);
                GfMatrix4d m = revision.finalPhase ? b.finalMatrix[slot]
                                                   : b.baseMatrix[slot];
                if (atPrim && k < revision.influenceRecords.size())
                    for (const int record : revision.influenceRecords[k])
                        if (b.frameMatrixValid[size_t(record)]) {
                            m = b.frameMatrix[size_t(record)];
                            break;
                        }
                gathered.push_back(OpMatrix(m));
            }
            SkinMatricesPayload p;
            p.method = dualQuat ? SkinMethod::DualQuaternion
                                : SkinMethod::ClassicLinear;
            RigExecRigDescBuilder builder;
            const RigExecNodeId id = builder.AddOp(SkinMatrices(), &p);
            const RigExecInputId in =
                builder.BindInput(id, SkinMatricesInfluences());
            const RigExecOutputId
                matricesOut = builder.BindOutput(id, SkinMatricesMatrices()),
                rowsOut = builder.BindOutput(id, SkinMatricesRows()),
                realOut = builder.BindOutput(id, SkinMatricesReal()),
                dualOut = builder.BindOutput(id, SkinMatricesDual()),
                stretchOut = builder.BindOutput(id, SkinMatricesStretch()),
                rigidOut = builder.BindOutput(id, SkinMatricesRigid());
            std::string error;
            const auto desc = builder.Build(&error);
            RigExecCompileOptions options;
            options.useSimd = b.useSimd;
            const auto program =
                desc ? RigExecCompile(*desc, options, &error) : nullptr;
            auto evaluator =
                program ? RigExecEvaluator::Create(
                              program, RigExecEvaluatorOptions(), &error)
                        : nullptr;
            const rigExecStd::RigExecArray<rigExecStd::Mat4d> influences(
                gathered.data(), gathered.size());
            if (!evaluator || !evaluator->SetInput(in, &influences, &error) ||
                !evaluator->Evaluate(&error)) {
                OpFailed("SkinMatrices", kind, revision.moverPathText,
                         !revision.influencesValid, error);
                continue;
            }
            const bool range = revision.role == RigExecBakedRevisionRole::Range;
            if (!revision.influencesValid && !(range && dualQuat)) {
                Report("SkinMatrices/" + kind, revision.moverPathText,
                       "the baked fold refused where the op ran");
                continue;
            }
            const auto &matrices = *static_cast<
                const rigExecStd::RigExecArray<rigExecStd::Mat4d> *>(
                evaluator->GetOutput(matricesOut));
            const auto &rows =
                *static_cast<const rigExecStd::RigExecArray<float> *>(
                    evaluator->GetOutput(rowsOut));
            using Quats = rigExecStd::RigExecArray<rigExecStd::Quatd>;
            const auto &real =
                *static_cast<const Quats *>(evaluator->GetOutput(realOut));
            const auto &dual =
                *static_cast<const Quats *>(evaluator->GetOutput(dualOut));
            const auto &stretch = *static_cast<
                const rigExecStd::RigExecArray<rigExecStd::Mat3d> *>(
                evaluator->GetOutput(stretchOut));
            const auto &rigid =
                *static_cast<const rigExecStd::RigExecArray<uint8_t> *>(
                    evaluator->GetOutput(rigidOut));
            std::vector<double> baked, op;
            for (const GfMatrix4d &m : revision.influences)
                PutBits(&baked, m);
            for (const rigExecStd::Mat4d &m : matrices)
                PutBits(&op, m);
            CompareBits("SkinMatrices", kind, revision.moverPathText, baked, op,
                        !revision.influencesValid);
            // The fold writes the forms of every revision but a ranged
            // linear blend, whose chunks gather their own.
            if (dualQuat || !range) {
                baked.clear();
                op.clear();
                if (dualQuat) {
                    for (size_t t = 0; t < revision.influences.size() &&
                                       t < revision.palette.size();
                         ++t) {
                        const auto &e = revision.palette[t];
                        PutPalette(&baked, e.rigid.real, e.rigid.dual,
                                   e.stretch, e.isRigid);
                    }
                    for (size_t t = 0; t < real.Size(); ++t)
                        PutPalette(&op, real[t], dual[t], stretch[t],
                                   rigid[t] != 0);
                } else {
                    baked.assign(revision.rows.begin(), revision.rows.end());
                    op.assign(rows.begin(), rows.end());
                }
                CompareBits("SkinMatrices", kind + " forms",
                            revision.moverPathText, baked, op,
                            !revision.influencesValid);
            }
            if (!revision.chunked)
                continue;
            for (size_t g = 0; g < revision.chunks.size(); ++g) {
                const auto &chunk = revision.chunks[g];
                baked.clear();
                op.clear();
                for (const int position : chunk.key) {
                    const size_t t = size_t(position);
                    if (t >= chunk.transforms.size() || t >= matrices.Size())
                        continue;
                    PutBits(&baked, chunk.transforms[t]);
                    PutBits(&op, matrices[t]);
                    const size_t stride = kRigExecSkinRowStride;
                    if ((t + 1) * stride <= chunk.rows.size() &&
                        (t + 1) * stride <= rows.Size())
                        for (size_t c = 0; c < stride; ++c) {
                            baked.push_back(chunk.rows[t * stride + c]);
                            op.push_back(rows[t * stride + c]);
                        }
                    if (t < chunk.palette.size() && t < real.Size()) {
                        const auto &e = chunk.palette[t];
                        PutPalette(&baked, e.rigid.real, e.rigid.dual,
                                   e.stretch, e.isRigid);
                        PutPalette(&op, real[t], dual[t], stretch[t],
                                   rigid[t] != 0);
                    }
                }
                CompareBits("SkinMatrices", kind + " chunk",
                            revision.moverPathText + " chunk " +
                                std::to_string(g),
                            baked, op, !revision.influencesValid);
            }
        }
    }
}

// A surface projector's scene translation forms the asset-to-mesh transform
// with the bits native compile bound into the baked revision.
void
ProbeSceneProjectors(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                     const B &b)
{
    RigExecSceneDescriptors scene;
    bool captured = false;
    std::string error;
    for (const auto &chain : b.chains)
        for (const auto &derived : chain.derived) {
            const auto &revision = derived.revision;
            if (!derived.matrixTarget ||
                revision.op != RigExecRevisionOp::SurfaceProjector)
                continue;
            const std::string key = "SceneTranslation/SurfaceProjector";
            if (!captured) {
                const RigExecUsdSceneAccess access(stage);
                if (!RigExecCaptureSceneDescriptors(access, rigPath,
                                                    {UsdTimeCode::Default()},
                                                    &scene, &error)) {
                    Report(key, rigPath.GetString(), "capture: " + error);
                    return;
                }
                captured = true;
            }
            std::vector<RigExecSceneGeometryDescriptor> variants;
            if (!RigExecTranslateSceneGeometryVariants(
                    scene, revision.binding.moverPath, revision.binding.base,
                    &variants, &error) ||
                variants.empty()) {
                Report(key, derived.targetText, "translation: " + error);
                continue;
            }
            const GfMatrix4d &native = revision.binding.meshWorldInverse;
            const GfMatrix4d &translated =
                variants.front().record.binding.meshWorldInverse;
            Compare("SceneTranslation", "SurfaceProjector", derived.targetText,
                    VtValue(native), VtValue(translated), false);
            if (std::memcmp(native.GetArray(), translated.GetArray(),
                            sizeof(double) * 16) != 0)
                Report(key, derived.targetText, "asset-to-mesh bits differ");
        }
}

void
ProbeStage(const UsdStageRefPtr &stage, const std::string &rig, Paint paint,
           bool dense = false)
{
    const std::string name = rig + PaintName(paint);
    if (!stage) {
        ++g_unprobed["rig " + name + ": does not open"];
        return;
    }
    SdfPath rigPath;
    for (const auto &prim : stage->Traverse())
        if (prim.GetTypeName() == SchemaTypeName::RigExecRoot) {
            rigPath = prim.GetPath();
            break;
        }
    if (rigPath.IsEmpty()) {
        ++g_unprobed["rig " + name + ": no RigExecRoot"];
        return;
    }
    if (paint != Paint::None && !PaintEnvelopes(stage, rigPath, paint)) {
        ++g_unprobed["rig " + name + ": no point mover to paint"];
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors) || !evaluator.GetBakedProgram()) {
        ++g_unprobed["rig " + name + ": no baked program"];
        return;
    }
    const std::vector<UsdTimeCode> frames =
        dense ? DenseFrames(stage, 12) : Frames(stage);
    for (const UsdTimeCode time : frames) {
        if (!evaluator.Evaluate(time).valid) {
            ++g_unprobed["rig " + name + ": invalid pose"];
            continue;
        }
        const auto *program = evaluator.GetBakedProgram();
        if (!program)
            continue;
        const B &b = RigExecBakedProgramAccess::StepGraph(*program);
        g_where = name + "@" + TfStringify(time.GetValue());
        if (time == frames.front())
            ProbeSceneProjectors(stage, rigPath, b);
        ProbeSolve(b);
        ProbeConstraint(b);
        ProbeCommits(b);
        ProbeRevisions(b);
        ProbeMoverTransforms(b);
        ProbeRangeChains(b);
        ProbeDerived(b);
        ProbeSkinMatrices(b);
        ProbeWeights(b);
        ProbeSpaces(b);
        ProbePose(b);
    }
}

void
ProbeRig(const std::string &file, Paint paint, bool dense)
{
    ProbeStage(UsdStage::Open(file), TfGetBaseName(file), paint, dense);
}

void
ProbeChunked(const std::string &file)
{
    const std::string always = TfGetenv("RIGEXEC_BAKED_CHUNK_ALWAYS");
    const std::string verts = TfGetenv("RIGEXEC_BAKED_CHUNK_VERTS");
    TfSetenv("RIGEXEC_BAKED_CHUNK_ALWAYS", "1");
    TfSetenv("RIGEXEC_BAKED_CHUNK_VERTS", "2");
    ProbeStage(UsdStage::Open(file), TfGetBaseName(file) + " chunked",
               Paint::None);
    const auto restore = [](const char *name, const std::string &value) {
        if (value.empty())
            TfUnsetenv(name);
        else
            TfSetenv(name, value);
    };
    restore("RIGEXEC_BAKED_CHUNK_ALWAYS", always);
    restore("RIGEXEC_BAKED_CHUNK_VERTS", verts);
}

// A wire: no example rig runs a curve mover as one. The curve's control
// points move over frames 1 to 4 and each point is bound at its own (u, d).
UsdStageRefPtr
WireRig()
{
    const auto stage = UsdStage::CreateInMemory("wire.usda");
    if (!stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 4
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def RigExecCurveMover "Wire" (
            prepend apiSchemas = ["RigExecMoverAPI"]
        )
        {
            uniform token rigExec:mode = "wire"
            rel rigExec:moves = </Asset/Geom/Strip.points>
            rel rigExec:driverCurve = </Asset/Curve>
            rel rigExec:bindCoordinates = </Asset/Bind.st>
            float inputs:dropoffDistance = 0.75
        }
    }
    def Scope "Geom"
    {
        def Mesh "Strip"
        {
            point3f[] points = [(0, 0, 0), (0.1, 0.2, 0), (0.2, -0.1, 0),
                (0.3, 0.3, 0.1), (0.4, 0, 0), (0.5, 0.1, -0.2),
                (0.6, -0.3, 0), (0.7, 0.05, 0), (0.8, 0.4, 0.1),
                (0.9, -0.2, 0), (1, 0, 0.3), (0.55, 0.6, 0)]
        }
    }
    def Scope "Bind"
    {
        float2[] st = [(0, 0), (0.1, 0.2), (0.2, 0.1), (0.3, 0.32),
            (0.4, 0), (0.5, 0.22), (0.6, 0.3), (0.7, 0.05), (0.8, 0.41),
            (0.9, 0.2), (1, 0.3), (0.55, 0.6)]
    }
    def NurbsCurves "Curve"
    {
        int[] curveVertexCounts = [4]
        int[] order = [4]
        double[] knots = [0, 0, 0, 0, 1, 1, 1, 1]
        point3f[] points = [(0, 0, 0), (0.33, 0, 0), (0.66, 0, 0), (1, 0, 0)]
        point3f[] points.timeSamples = {
            1: [(0, 0, 0), (0.33, 0.1, 0), (0.66, -0.2, 0), (1, 0, 0)],
            4: [(0, 0.3, 0), (0.33, 0.9, 0.2), (0.66, -0.5, 0.1), (1, 0.4, 0)],
        }
    }
}
)USD"))
        return UsdStageRefPtr();
    return stage;
}

// Joint groups: a torso with two alternative heads on an FK chain from a
// start control, the active head switching over the frames.
UsdStageRefPtr
JointGroupRig()
{
    const auto stage = UsdStage::CreateInMemory("jointGroups.usda");
    if (!stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
(
    startTimeCode = 0
    endTimeCode = 4
)
def Xform "Char"
{
    def RigExecRoot "Rig"
    {
        def Scope "Controls"
        {
            def RigExecControl "Torso"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                double avars:rz.spline = { 0: 0; post held, 4: 40; post held, }
                def RigExecControl "HeadA"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )
                    double avars:rz = 30
                    def RigExecControl "JawA"
                    {
                        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1.5, 0, 1) )
                    }
                }
                def RigExecControl "HeadB"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )
                    double avars:rz = 30
                }
            }
            def RigExecControl "Start"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                double avars:tx = 5
            }
        }
        def Scope "Joints"
        {
            def RigExecJoint "Torso"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
                def RigExecJoint "HeadA"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )
                    def RigExecJoint "JawA"
                    {
                        matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1.5, 0, 1) )
                    }
                }
                def RigExecJoint "HeadB"
                {
                    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 1, 0, 1) )
                }
            }
        }
        def Scope "Groups"
        {
            def Scope "Heads" (
                prepend apiSchemas = ["RigExecJointGroupAPI"]
            )
            {
                rel collection:members:includes = [</Char/Rig/Joints/Torso/HeadA>, </Char/Rig/Joints/Torso/HeadB>]
                double rigExec:activeMember.spline = { 1: 0; post held, 3: 1; post held, }
            }
        }
        def RigExecFkChain "Chain"
        {
            uniform token rigExec:controlSpace = "parentRelative"
            rel rigExec:startFrame = </Char/Rig/Controls/Start>
            rel rigExec:controls = [</Char/Rig/Controls/Torso>, </Char/Rig/Controls/Torso/HeadA>, </Char/Rig/Controls/Torso/HeadA/JawA>, </Char/Rig/Controls/Torso/HeadB>]
            rel rigExec:joints = [</Char/Rig/Joints/Torso>, </Char/Rig/Joints/Torso/HeadA>, </Char/Rig/Joints/Torso/HeadA/JawA>, </Char/Rig/Joints/Torso/HeadB>]
            rel rigExec:jointGroups = </Char/Rig/Groups/Heads>
        }
    }
}
)USD"))
        return UsdStageRefPtr();
    return stage;
}

} // namespace

#include "rigExecConnectedBridgeFixture.h"

// Connected controls whose providers re-evaluate in the walk: the rig that
// has ProviderRefresh steps.
UsdStageRefPtr
ConnectedBridgeRig()
{
    return RigExecMakeConnectedBridgeFixture(
               [](const UsdStageRefPtr &stage, const SdfPath &path,
                  const GfMatrix4d &matrix) {
                   const auto x = UsdGeomXform::Define(stage, path);
                   x.MakeMatrixXform().Set(matrix);
                   return x.GetPrim();
               },
               [](const UsdStageRefPtr &stage, const char *name,
                  const char *type, const SdfPathVector &targets) {
                   const auto prim = stage->DefinePrim(
                       SdfPath(std::string("/Asset/Rig/Movers/") + name),
                       TfToken(type));
                   prim.ApplyAPI(TfToken("RigExecMoverAPI"));
                   prim.CreateRelationship(
                           TfToken(SchemaAttrText::rigExecMoves))
                       .SetTargets(targets);
                   return prim;
               })
        .stage;
}

// The six affine frame expressions feeding joints, the source control
// animated, and a constraint that reads tokens and arrays: the rig that
// has AffineFrame provider operations.
UsdStageRefPtr
AffineRig()
{
    const char *types[] = {"RigExecCopyFrame",      "RigExecMappedFrame",
                           "RigExecSkinInfluence",  "RigExecArmatureParent",
                           "RigExecBoneFrame",      "RigExecConstraintFrame",
                           "RigExecConstraintFrame"};
    const char *labels[] = {"copy", "mapped",     "influence", "parent",
                            "bone", "constraint", "transforms"};
    const auto translate = [](double x, double y) {
        GfMatrix4d m(1);
        m.SetTranslateOnly(GfVec3d(x, y, 0));
        return m;
    };
    auto stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1);
    stage->SetEndTimeCode(3);
    stage->DefinePrim(SdfPath("/Rig"), TfToken(SchemaTypeName::RigExecRoot));
    auto source = stage->DefinePrim(SdfPath("/Rig/Controls/Source"),
                                    TfToken("RigExecControl"));
    const auto tx = source.GetAttribute(TfToken("avars:tx"));
    tx.Set(2.0, UsdTimeCode(1));
    tx.Set(-1.0, UsdTimeCode(3));
    source.GetAttribute(TfToken("avars:rz")).Set(30.0, UsdTimeCode(2));
    for (int i = 0; i < 7; ++i) {
        auto c = stage->DefinePrim(
            SdfPath(std::string("/Rig/Computations/") + labels[i]),
            TfToken(types[i]));
        if (c.GetRelationship(TfToken("rigExec:source")))
            c.GetRelationship(TfToken("rigExec:source"))
                .SetTargets({source.GetPath()});
        c.GetRelationship(TfToken("rigExec:poseInputs"))
            .SetTargets({source.GetPath()});
        if (i == 1)
            c.GetAttribute(TfToken("inputs:targetRest")).Set(translate(0, 1));
        if (i == 3) {
            c.GetAttribute(TfToken("inputs:useIncoming")).Set(true);
            c.GetAttribute(TfToken("inputs:incoming")).Set(translate(0, 3));
        }
        if (i == 4) {
            c.GetAttribute(TfToken("inputs:local")).Set(translate(0, 1));
            c.GetAttribute(TfToken("inputs:tx")).Set(2.0);
            c.GetRelationship(TfToken("rigExec:sourceObject"))
                .SetTargets({source.GetPath()});
        }
        if (i == 5) {
            c.GetAttribute(TfToken("inputs:incoming")).Set(translate(0, 3));
            c.GetAttribute(TfToken("inputs:influence")).Set(.5);
        }
        if (i == 6) {
            c.GetAttribute(TfToken("inputs:operation"))
                .Set(TfToken("COPY_TRANSFORMS"));
            c.GetAttribute(TfToken("inputs:rotationMix"))
                .Set(TfToken("AFTER_FULL"));
            c.GetAttribute(TfToken("inputs:incoming")).Set(translate(1, 1));
            c.GetAttribute(TfToken("inputs:axisMask")).Set(5);
            c.GetAttribute(TfToken("inputs:targetWeights"))
                .Set(VtDoubleArray{.5});
            c.GetAttribute(TfToken("inputs:influence")).Set(.75);
        }
        auto joint =
            stage->DefinePrim(SdfPath(std::string("/Rig/Joints/") + labels[i]),
                              TfToken("RigExecJoint"));
        joint.GetAttribute(TfToken("posed:space"))
            .SetConnections(
                {c.GetAttribute(TfToken("outputs:matrix")).GetPath()});
    }
    return stage;
}

// Pose interpolators measuring their driver's translation: one on the
// translation alone, one on rotation and translation. The hand slides and
// turns over frames 1 to 9.
UsdStageRefPtr
TranslationPoseRig()
{
    const auto stage = UsdStage::CreateInMemory("translationPose.usda");
    std::string text = R"USD(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 9
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def RigExecControl "Shoulder" (prepend apiSchemas = ["RigExecControlAPI"])
        {
            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
        }
        def RigExecControl "Hand" (prepend apiSchemas = ["RigExecControlAPI"])
        {
            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 0, 0, 1) )
            double avars:tx.timeSamples = {1: 0, 3: 0.5, 5: 1.2, 7: 0.2, 9: -0.4}
            double avars:ty.timeSamples = {1: 0, 3: 0.8, 5: 0.1, 7: 1.1, 9: 0.3}
            double avars:rz.timeSamples = {1: 0, 4: 35, 7: -20, 9: 60}
        }
        def RigExecFkChain "Chain"
        {
            rel rigExec:controls = [</Asset/Rig/Shoulder>, </Asset/Rig/Hand>]
            rel rigExec:joints = [</Asset/Rig/Upper>, </Asset/Rig/Upper/Hand>]
        }
        def RigExecJoint "Upper"
        {
            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )
            def RigExecJoint "Hand"
            {
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (2, 0, 0, 1) )
            }
        }
)USD";
    const auto interpolator = [](const std::string &name, bool rotation) {
        const std::string pose = [&] {
            std::string out;
            const char *names[] = {"rest", "out", "up", "turned"};
            const char *moves[] = {"(0, 0, 0)", "(1, 0, 0)", "(0, 1, 0)",
                                   "(0.5, 0.5, 0)"};
            const char *turns[] = {"(1, 0, 0, 0)", "(1, 0, 0, 0)",
                                   "(1, 0, 0, 0)", "(0.8660254, 0, 0, 0.5)"};
            for (int k = 0; k < 4; ++k)
                out += std::string("            def RigExecPose \"") +
                       names[k] +
                       "\"\n            {\n"
                       "                uniform token rigExec:poseType = "
                       "\"swing\"\n"
                       "                quatf rigExec:rotation = " +
                       turns[k] +
                       "\n"
                       "                float rigExec:rotationRadius = 0.7\n"
                       "                float3 rigExec:translation = " +
                       moves[k] +
                       "\n"
                       "                float rigExec:translationRadius = "
                       "0.8\n"
                       "            }\n";
            return out;
        }();
        return "        def RigExecPoseInterpolator \"" + name +
               "\"\n        {\n"
               "            rel rigExec:driver = </Asset/Rig/Upper/Hand>\n"
               "            bool rigExec:enableRotation = " +
               (rotation ? "1" : "0") +
               "\n"
               "            bool rigExec:enableTranslation = 1\n"
               "            uniform token rigExec:kernel = \"gaussian\"\n"
               "            bool rigExec:normalize = 1\n" +
               pose + "        }\n";
    };
    text += interpolator("Reach", false) + interpolator("ReachTurn", true) +
            "    }\n}\n";
    if (!stage->GetRootLayer()->ImportFromString(text))
        return nullptr;
    return stage;
}

// Auto clavicles: one carried by its FK arm alone, one also by an IK target
// and pole blended in. The arms swing over frames 1 to 4 and the dial and
// blend move, so the shift is zero on some frames and not on others.
UsdStageRefPtr
AutoClavicleRig()
{
    const auto stage = UsdStage::CreateInMemory("autoClavicle.usda");
    std::string text = R"USD(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 4
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def RigExecControl "Chest" (prepend apiSchemas = ["RigExecControlAPI"])
        {
            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 10, 0, 1) )
)USD";
    const auto along = [](double x) {
        return "( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (" +
               TfStringify(x) + ", 0, 0, 1) )";
    };
    const auto side = [&](const std::string &name, double x) {
        const std::string rest = along(x), unit = along(3 * x);
        return "            def RigExecControl \"" + name +
               "Shldr\" (prepend apiSchemas = [\"RigExecControlAPI\"])\n"
               "            {\n"
               "                matrix4d rest:space = " +
               rest +
               "\n"
               "                custom double avars:autoClav = 1\n"
               "                double avars:autoClav.timeSamples = {1: 1, 2: "
               "1, 3: 0.5, 4: 1}\n"
               "                def RigExecControl \"Swing\" (prepend "
               "apiSchemas = [\"RigExecControlAPI\"])\n"
               "                {\n"
               "                    matrix4d rest:space = " +
               unit +
               "\n"
               "                    def RigExecControl \"UpArm\" (prepend "
               "apiSchemas = [\"RigExecControlAPI\"])\n"
               "                    {\n"
               "                        matrix4d rest:space = " +
               unit +
               "\n"
               "                        double avars:ry.timeSamples = {1: 0, "
               "2: -80, 3: -60, 4: 0}\n"
               "                        double avars:rz.timeSamples = {1: 0, "
               "2: 20, 3: 70, 4: -30}\n"
               "                        def RigExecControl \"LoArm\" (prepend "
               "apiSchemas = [\"RigExecControlAPI\"])\n"
               "                        {\n"
               "                            matrix4d rest:space = " +
               unit +
               "\n"
               "                            def RigExecControl \"Hand\" "
               "(prepend apiSchemas = [\"RigExecControlAPI\"])\n"
               "                            {\n"
               "                                matrix4d rest:space = " +
               unit +
               "\n"
               "                            }\n"
               "                        }\n"
               "                    }\n"
               "                }\n"
               "            }\n";
    };
    text += side("L_", 1.0) + side("R_", -1.0) + R"USD(        }
        def RigExecControl "R_ArmIK" (prepend apiSchemas = ["RigExecControlAPI"])
        {
            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (-10, 9, 0, 1) )
            double avars:ty.timeSamples = {1: 0, 2: 0, 3: 3, 4: 4}
        }
        def RigExecControl "R_ArmPV" (prepend apiSchemas = ["RigExecControlAPI"])
        {
            matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (-6, 10, -3, 1) )
        }
        def RigExecControl "R_Arm" (prepend apiSchemas = ["RigExecControlAPI"])
        {
            custom double avars:ikfk = 0
            double avars:ikfk.timeSamples = {1: 0, 2: 0, 3: 0.5, 4: 1}
        }
        def "AutoClavicles"
        {
)USD";
    const auto clavicle = [&](const std::string &name, double basisZ, bool ik) {
        const std::string shoulder = "</Asset/Rig/Chest/" + name + "Shldr";
        std::string out =
            "            def RigExecAutoClavicle \"" + name +
            "auto\"\n"
            "            {\n"
            "                rel rigExec:amountAttribute = " +
            shoulder +
            ".avars:autoClav>\n"
            "                rel rigExec:anchor = </Asset/Rig/Chest>\n"
            "                matrix4d rigExec:basis = ( (0, 0, " +
            TfStringify(basisZ) +
            ", 0), (0, 1, 0, 0), (1, 0, 0, 0), (0, 0, 0, 1) )\n"
            "                rel rigExec:fkControls = [" +
            shoulder + "/Swing/UpArm>, " + shoulder + "/Swing/UpArm/LoArm>, " +
            shoulder +
            "/Swing/UpArm/LoArm/Hand>]\n"
            "                double rigExec:ikValue = 1\n"
            "                uniform token rigExec:kernel = \"gaussian\"\n"
            "                rel rigExec:pivot = " +
            shoulder +
            ">\n"
            "                float[] rigExec:poseFalloffs = [0.5, 0.5, 0.3, "
            "0.5, 0.5, 0.5, 0.5]\n"
            "                double[] rigExec:poseGains = [0, 1.15, 0.1, 1, 1, "
            "2, 2]\n"
            "                quatf[] rigExec:poseRotations = [(1, 0, 0, 0), "
            "(0.7071068, 0, -0.7071068, 0), (0.7071068, 0, 0.7071068, 0), "
            "(0.7071068, 0, 0, -0.7071068), (0.7071068, 0, 0, 0.7071068), "
            "(0.3420201, 0, 0, -0.9396926), (0.3420201, 0, 0, 0.9396926)]\n"
            "                float rigExec:regularization = 100\n"
            "                rel rigExec:target = " +
            shoulder + "/Swing>\n";
        if (ik)
            out +=
                "                rel rigExec:ikBlendAttribute = "
                "</Asset/Rig/R_Arm.avars:ikfk>\n"
                "                rel rigExec:ikTarget = </Asset/Rig/R_ArmIK>\n"
                "                rel rigExec:poleControl = "
                "</Asset/Rig/R_ArmPV>\n";
        return out + "            }\n";
    };
    text += clavicle("L_", -1.0, false) + clavicle("R_", 1.0, true) +
            "        }\n    }\n}\n";
    if (!stage->GetRootLayer()->ImportFromString(text))
        return nullptr;
    return stage;
}

// A surface projector inside an asset under /World: no ancestor transform,
// a static one, or an animated one (translation, rotation, non-uniform scale).
enum class Ancestor { None, Static, Animated };

/// A switchable limb with an IK/FK match (a copy of testRigExecConstraints'
/// BuildIkFkMatchArm): FK on frames 1-4, IK on 5-8, between on 9-10.
UsdStageRefPtr
IkFkMatchRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    if (!stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 10
)
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        reorder nameChildren = ["Controls", "Joints", "Movers", "Solvers"]
        def Scope "Controls"
        {
            def RigExecControl "Shoulder"
            {
                double rest:tx = 0
                double rest:tz = 0
                def RigExecControl "Up"
                {
                    double rest:tx = 0
                    double rest:tz = 0
                    double avars:rz.timeSamples = {1: 3.0, 2: 6.0, 3: 9.0, 4: 12.0, 5: 15.0, 6: 18.0, 7: 21.0, 8: 24.0, 9: 27.0, 10: 30.0}
                    def RigExecControl "Lo"
                    {
                        double rest:tx = 3
                        double rest:tz = 0
                        double avars:rz.timeSamples = {1: -6.0, 2: -12.0, 3: -18.0, 4: -24.0, 5: -30.0, 6: -36.0, 7: -42.0, 8: -48.0, 9: -54.0, 10: -60.0}
                        def RigExecControl "Hand"
                        {
                            double rest:tx = 4
                            double rest:tz = 0
                            double avars:rx.timeSamples = {1: 5.0, 2: 10.0, 3: 15.0, 4: 20.0, 5: 25.0, 6: 30.0, 7: 35.0, 8: 40.0, 9: 45.0, 10: 50.0}
                        }
                    }
                }
            }
            def RigExecControl "Ik"
            {
                double rest:tx = 7
                double rest:tz = 0
                double avars:tx.timeSamples = {1: -0.4, 2: -0.8, 3: -1.2, 4: -1.6, 5: -2.0, 6: -2.4, 7: -2.8, 8: -3.2, 9: -3.6, 10: -4.0}
                double avars:tz.timeSamples = {1: 0.3, 2: 0.6, 3: 0.9, 4: 0.0, 5: 0.3, 6: 0.6, 7: 0.9, 8: 0.0, 9: 0.3, 10: 0.6}
                double avars:ry.timeSamples = {1: 4.0, 2: 8.0, 3: 12.0, 4: 16.0, 5: 20.0, 6: 24.0, 7: 28.0, 8: 32.0, 9: 36.0, 10: 40.0}
                def RigExecControl "Orient"
                {
                    double rest:tx = 0
                    double rest:tz = 0
                }
            }
            def RigExecControl "Pole" (
                prepend apiSchemas = ["PhysicsRigidBodyAPI"]
            )
            {
                double rest:tx = 3.5
                double rest:tz = 2
            }
            def RigExecControl "Settings"
            {
                double rest:tx = 0
                double rest:tz = -3
                float avars:ikfk.timeSamples = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 1.0, 6: 1.0, 7: 1.0, 8: 1.0, 9: 0.5, 10: 0.5}
            }
        }
        def Scope "Joints"
        {
            def RigExecJoint "Shoulder"
            {
                double rest:tx = 0
                def RigExecJoint "Elbow"
                {
                    double rest:tx = 3
                    def RigExecJoint "Wrist"
                    {
                        double rest:tx = 4
                    }
                }
            }
        }
        def Scope "Movers"
        {
            def RigExecIkFkMatchConstraint "Match" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = [</Asset/Rig/Controls/Ik>, </Asset/Rig/Controls/Pole>, </Asset/Rig/Controls/Shoulder/Up>, </Asset/Rig/Controls/Shoulder/Up/Lo>, </Asset/Rig/Controls/Shoulder/Up/Lo/Hand>]
                rel rigExec:ikControl = </Asset/Rig/Controls/Ik>
                rel rigExec:poleControl = </Asset/Rig/Controls/Pole>
                rel rigExec:fkControls = [</Asset/Rig/Controls/Shoulder/Up>, </Asset/Rig/Controls/Shoulder/Up/Lo>, </Asset/Rig/Controls/Shoulder/Up/Lo/Hand>]
                rel rigExec:limbJoints = [</Asset/Rig/Joints/Shoulder>, </Asset/Rig/Joints/Shoulder/Elbow>, </Asset/Rig/Joints/Shoulder/Elbow/Wrist>]
                rel rigExec:rootControl = </Asset/Rig/Controls/Shoulder>
                rel rigExec:effector = </Asset/Rig/Controls/Ik/Orient>
                double inputs:switch.connect = </Asset/Rig/Controls/Settings.avars:ikfk>
                double inputs:twist = 10
            }
        }
        def Scope "Solvers"
        {
            def RigExecFkChain "FK"
            {
                rel rigExec:controls = [</Asset/Rig/Controls/Shoulder/Up>, </Asset/Rig/Controls/Shoulder/Up/Lo>, </Asset/Rig/Controls/Shoulder/Up/Lo/Hand>]
            }
            def RigExecTwoBoneIk "IK"
            {
                rel rigExec:rootControl = </Asset/Rig/Controls/Shoulder>
                rel rigExec:effectorControl = </Asset/Rig/Controls/Ik/Orient>
                rel rigExec:poleControl = </Asset/Rig/Controls/Pole>
                rel rigExec:joints = [</Asset/Rig/Joints/Shoulder>, </Asset/Rig/Joints/Shoulder/Elbow>, </Asset/Rig/Joints/Shoulder/Elbow/Wrist>]
            }
            def RigExecBlendPointFrames "Blend"
            {
                rel rigExec:inputA = </Asset/Rig/Solvers/FK>
                rel rigExec:inputB = </Asset/Rig/Solvers/IK>
                float inputs:weight.connect = </Asset/Rig/Controls/Settings.avars:ikfk>
                rel rigExec:joints = [</Asset/Rig/Joints/Shoulder>, </Asset/Rig/Joints/Shoulder/Elbow>, </Asset/Rig/Joints/Shoulder/Elbow/Wrist>]
            }
        }
    }
}
)USD"))
        return nullptr;
    return stage;
}

/// Parent and Rotation constraints carried by a rigExec:space and an Aim
/// aiming its world up at an object, all reading sources that live outside
/// the rig, so the step reads them off the stage.
UsdStageRefPtr
StageSourceRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    if (!stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 4
)
def Xform "Asset"
{
    def Xform "Source"
    {
        double3 xformOp:translate.timeSamples = {1: (3, 1, -2), 4: (4, 0.5, -1)}
        double3 xformOp:rotateXYZ.timeSamples = {1: (10, 20, 30), 4: (-15, 40, 5)}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
    }
    def Xform "Up"
    {
        double3 xformOp:translate.timeSamples = {1: (0, 5, 0), 4: (1, 5, 2)}
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def RigExecRoot "Rig"
    {
        def Scope "Controls"
        {
            def RigExecControl "Space"
            {
                double rest:tx = 1
                double avars:tx.timeSamples = {1: 0, 4: 2}
                double avars:rz.timeSamples = {1: 0, 4: 30}
                def RigExecControl "Parented"
                {
                    double rest:tx = 2
                    double avars:ry.timeSamples = {1: 0, 4: 15}
                }
                def RigExecControl "Turned"
                {
                    double rest:ty = 2
                }
                def RigExecControl "Aimed"
                {
                    double rest:tz = 1
                }
            }
        }
        def Scope "Movers"
        {
            def RigExecParentConstraint "Parent" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Rig/Controls/Space/Parented>
                rel rigExec:sources = </Asset/Source>
                rel rigExec:space = </Asset/Rig/Controls/Space>
                float inputs:defaultWeight = 0.75
            }
            def RigExecRotationConstraint "Rotation" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Rig/Controls/Space/Turned>
                rel rigExec:sources = </Asset/Source>
                rel rigExec:space = </Asset/Rig/Controls/Space>
            }
            def RigExecAimConstraint "Aim" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </Asset/Rig/Controls/Space/Aimed>
                rel rigExec:sources = </Asset/Source>
                uniform token rigExec:worldUpType = "objectUp"
                rel rigExec:worldUpObject = </Asset/Up>
            }
        }
    }
}
)USD"))
        return nullptr;
    return stage;
}

UsdStageRefPtr
ProjectorRig(Ancestor ancestor)
{
    const auto stage = UsdStage::CreateInMemory("projector.usda");
    if (!stage->GetRootLayer()->ImportFromString(R"USD(#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 3
)
def Xform "World"
{
    def Xform "Asset"
    {
        def RigExecRoot "Rig"
        {
            def RigExecControl "Driver"
            {
                double avars:tx.timeSamples = {1: 0, 2: 0.25, 3: 0}
            }
            def RigExecControl "Source"
            {
                double rest:tx = 4
                double rest:ty = -2
                double rest:tz = 6
                double avars:ty.timeSamples = {1: 0, 2: 0.5, 3: -0.25}
            }
            def RigExecMatrixMover "Deform" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                float inputs:defaultWeight = 1
                rel rigExec:moves = </World/Asset/Surface.points>
                rel rigExec:transform = </World/Asset/Rig/Driver>
            }
            def RigExecSurfaceProjector "Project" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                rel rigExec:moves = </World/Asset/Surface.points>
                rel rigExec:sources = </World/Asset/Rig/Source>
                token rigExec:shaderPrimvar = "projectionFrame"
            }
        }
        def Mesh "Surface"
        {
            double3 xformOp:translate = (4, -2, 7)
            float xformOp:rotateY = 20
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateY"]
            point3f[] points = [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
        }
    }
}
)USD"))
        return nullptr;
    if (ancestor == Ancestor::None)
        return stage;
    const auto placement = [](double t) {
        return GfMatrix4d(1.0).SetScale(GfVec3d(2.0 + 0.25 * t, 0.5, 3.0)) *
               GfMatrix4d(1.0).SetRotate(
                   GfRotation(GfVec3d(1, 2, 3), 25.0 + 7.0 * t)) *
               GfMatrix4d(1.0).SetTranslate(
                   GfVec3d(1.0e6 + 4.0 * t, -7.0e5, 3.0e5 - 2.0 * t));
    };
    const UsdGeomXformOp op =
        UsdGeomXform(stage->GetPrimAtPath(SdfPath("/World")))
            .AddTransformOp(UsdGeomXformOp::PrecisionDouble);
    if (ancestor == Ancestor::Static) {
        op.Set(placement(1.0));
    } else {
        for (const double t : {1.0, 2.0, 3.0})
            op.Set(placement(t), UsdTimeCode(t));
    }
    return stage;
}

int
main(int argc, char **argv)
{
    std::string error;
    g_tasks = RigExecCreateThreadTaskSystem(4);
    if (!RigExecLoadGeometryOpCore(&error)) {
        std::printf("op core: %s\n", error.c_str());
        return 1;
    }
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    // The probe reads points through bindings: one read past its binding's
    // life reads NaN on every platform.
    RigExecBakedProgramTesting::SetPoisonReleasedGathers(true);
    // Rigs after --dense are probed on up to 12 frames of their range; rigs
    // after --painted are probed as they are and again under each painted
    // partial envelope; rigs after --chunked are probed with every skin cut
    // into chunks of two points.
    bool painted = false, dense = false, chunked = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--painted") == 0) {
            painted = true;
            dense = chunked = false;
            continue;
        }
        if (std::strcmp(argv[i], "--dense") == 0) {
            dense = true;
            continue;
        }
        if (std::strcmp(argv[i], "--chunked") == 0) {
            chunked = true;
            dense = false;
            continue;
        }
        if (chunked) {
            ProbeChunked(argv[i]);
            continue;
        }
        ProbeRig(argv[i], Paint::None, dense);
        if (painted)
            for (const Paint paint : kPaints)
                ProbeRig(argv[i], paint, false);
    }
    ProbeStage(ConnectedBridgeRig(), "connected bridge rig", Paint::None);
    ProbeStage(AffineRig(), "affine rig", Paint::None);
    if (painted)
        for (const Paint paint : {Paint::None, Paint::Dense, Paint::Sparse})
            ProbeStage(WireRig(), "wire rig", paint);
    ProbeStage(JointGroupRig(), "joint group rig", Paint::None);
    ProbeStage(AutoClavicleRig(), "auto clavicle rig", Paint::None, true);
    ProbeStage(TranslationPoseRig(), "translation pose rig", Paint::None, true);
    ProbeStage(IkFkMatchRig(), "IK/FK match rig", Paint::None, true);
    ProbeStage(StageSourceRig(), "stage source rig", Paint::None, true);
    ProbeStage(ProjectorRig(Ancestor::None), "projector rig", Paint::None);
    ProbeStage(ProjectorRig(Ancestor::Static), "projector rig, static ancestor",
               Paint::None);
    ProbeStage(ProjectorRig(Ancestor::Animated),
               "projector rig, animated ancestor", Paint::None);
    std::printf("%-44s %9s %9s %9s %9s %9s\n", "family/kind", "compared",
                "refusals", "compile", "run", "mismatch");
    for (const auto &[key, tally] : g_tally)
        std::printf("%-44s %9zu %9zu %9zu %9zu %9zu\n", key.c_str(),
                    tally.compared, tally.refusals, tally.compileErrors,
                    tally.runErrors, tally.mismatched);
    std::printf("not probed:\n");
    for (const auto &[reason, count] : g_unprobed)
        std::printf("  %5zu  %s\n", count, reason.c_str());
    std::printf("findings: %zu\n", g_findings.size());
    for (const auto &finding : g_findings)
        std::printf("  %s\n", finding.text.c_str());
    std::printf("envelopes compared:\n");
    for (const auto &[kind, shapes] : g_envelopes)
        for (const auto &[shape, count] : shapes)
            std::printf("  %-22s %-14s %6zu\n", kind.c_str(), shape.c_str(),
                        count);
    if (painted) {
        for (const char *kind : kPaintedKinds)
            for (const char *shape : {"dense", "sparse"})
                if (!g_envelopes[kind][shape]) {
                    std::printf("FAIL: no %s revision compared under a %s "
                                "partial envelope\n",
                                kind, shape);
                    return 1;
                }
    }
    for (const char *family :
         {"Solve/",
          "Solve/RigExecFkChain",
          "Solve/RigExecTwoBoneIk",
          "Solve/RigExecBlendPointFrames",
          "Solve/RigExecTwistDistribution",
          "Solve/RigExecRibbon",
          "Solve/RigExecSplineIk",
          "Solve/RigExecSkeleton2d",
          "RevisionChunk/Skin",
          "RevisionChunk/BlendShape",
          "RevisionChunk/Wire",
          "RevisionChunk/SurfaceProject",
          "RevisionChunk/Ribbon",
          "RevisionChunk/EmitGuidePoints",
          "RevisionChunk/Wrinkle",
          "RevisionChunk/External RigExecPatchMover",
          "RevisionChunk/External RigExecSurfaceBindingMover",
          "RevisionChunk/External RigExecLayeredSkinMover",
          "RevisionChunk/External RigExecClothMover",
          "Derived/RecomputeNormals",
          "Derived/RecomputeExtent",
          "Derived/SurfaceProjector",
          "Derived/ShaderDials",
          "SkinMatrices/linear chunk",
          "SkinMatrices/dualQuaternion chunk",
          "Derived/",
          "SkinMatrices/linear",
          "SkinMatrices/dualQuaternion",
          "WeightPacket/",
          "Constraint/",
          "Constraint/RigExecMatrixMover",
          "Constraint/RigExecAimConstraint geometry",
          "Constraint/RigExecIkFkMatchConstraint",
          "Constraint/RigExecHarmonicsConstraint",
          "Constraint/RigExecParentConstraint space stage source",
          "Constraint/RigExecRotationConstraint space stage source",
          "Constraint/RigExecAimConstraint stage source",
          "Constraint/RigExecSingleChainIkConstraint slots",
          "Commit/constraint multi-slot",
          "Spaces/ProviderMatrix/",
          "Spaces/FrameMatrix/",
          "Spaces/ProviderRefresh",
          "Spaces/SpaceExpression/SpaceExpression",
          "Spaces/SpaceExpression/AffineFrame",
          "Spaces/SpaceExpression/Attribute",
          "Spaces/SpaceExpression/Sampled",
          "Commit/",
          "Solve/RigExecFkChain groups",
          "Pose/AvarInputs",
          "Pose/SpaceCheckpoint",
          "Pose/PoseInterpolator Rbf",
          "Pose/PoseInterpolator Rbf translation",
          "Pose/PoseInterpolator RbfMethod",
          "Pose/PoseInterpolator Network",
          "Pose/PoseInterpolator Maps",
          "Pose/RestCompose",
          "Pose/LadderCompose"}) {
        size_t compared = 0;
        for (const auto &[key, tally] : g_tally)
            if (key.compare(0, std::strlen(family), family) == 0)
                compared += tally.compared;
        if (!compared) {
            std::printf("FAIL: no %s step compared\n", family);
            return 1;
        }
    }
    return g_findings.empty() ? 0 : 1;
}
