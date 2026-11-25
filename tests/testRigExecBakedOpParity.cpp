// Each baked step family that already has an op, replayed through that op on
// the inputs the production step body read, after the evaluator ran real rigs.
// The op's answer is compared bit for bit with what the body published,
// including what a refusal publishes. A mismatch is reported with its first
// differing value; a family or kind the op cannot take yet is counted as not
// probed, with the reason.
#include "rigExec/api/goldenPose.h"
#include "rigExec/api/rigEvaluator.h"
#include "rigExec/internal/bakedProgramImpl.h"
#include "rigExec/internal/moverGraph.h"
#include "rigExecScene/constraintSceneTranslation.h"
#include "rigExecScene/propertyOps.h"
#include "rigExecSchema/schemaNames.h"
#include "rigExecStdTypes/stdTypes.h"

#include "rigExec/api/compile.h"
#include "rigExec/api/taskSystem.h"
#include "opRunners/geometryOpRunner.h"
#include "opRunners/poseOpRunner.h"
#include "opRunners/weightOpRunner.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/tokens.h"

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

// Solve: the record and the inputs the body handed RigExecRunSolver, against
// the aggregate it published and the diagnostic it emitted.
void
ProbeSolve(const B &b)
{
    for (const auto &step : b.steps) {
        if (step.kind != RigExecBakedStepKind::Solve)
            continue;
        const auto &s = b.solvers[size_t(step.object)];
        const std::string kind = TypeName(s.type);
        if (!RigExecSolverOpSupported(s)) {
            Unprobed("Solve", kind,
                     "RigExecSolverOpSupported refuses the record "
                     "(joint-group selectors, a degenerate record or a "
                     "refused rotation blend)");
            continue;
        }
        RigExecPoseOpRunner runner;
        RigExecPointFrameArray out;
        std::string error;
        const bool ran =
            RigExecRunSolverOp(s, s.kernelInputs, runner, &out, &error);
        const auto &baked = b.aggregates[size_t(step.object)];
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
        if (!c.weightObject.IsEmpty())
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

// Constraint: SingleChainIk keeps its kernel record and inputs, so they are
// replayed directly. Every other frame kernel is rebuilt from the scene
// record and the fin versions the commit read, and compared on what the
// commit published: the candidate when present, the entering frame when
// the constraint passed through or refused.
void
ProbeConstraint(const B &b)
{
    for (const auto &step : b.steps) {
        if (step.kind != RigExecBakedStepKind::Constraint)
            continue;
        const int walk = step.object;
        const auto &c = b.constraints[size_t(b.walkSteps[size_t(walk)].index)];
        const auto &commit = b.commits[size_t(walk)];
        const std::string kind = TypeName(c.type);
        if (c.ikFkMatch) {
            Unprobed("Constraint", kind,
                     "the match body assembles its sources inside the step "
                     "and keeps no RigExecIkFkMatchOpInputs");
            continue;
        }
        if (c.harmonics) {
            Unprobed("Constraint", kind,
                     "the windowed history lives in the program's window ring; "
                     "a replay needs the ring before this tick");
            continue;
        }
        RigExecPoseOpRunner runner;
        std::string error;
        if (c.singleChainIk) {
            if (c.kernelResult.chain.empty()) {
                Unprobed("Constraint", kind, "the kernel did not run");
                continue;
            }
            RigExecConstraintResult result;
            const bool ran = RigExecRunConstraintOp(
                c.kernelRecord, c.kernelInputs, runner, &result, &error);
            RigExecPointFrameArray bakedChain, opChain;
            bakedChain.frames = c.kernelResult.chain;
            if (ran)
                opChain.frames = result.chain;
            Compare("Constraint", kind, c.pathText, VtValue(bakedChain),
                    VtValue(opChain), false);
            continue;
        }
        if (c.deltaBase >= 0) {
            Unprobed("Constraint", kind,
                     "geometry domain: the body measures a delta against the "
                     "target's authored transform");
            continue;
        }
        if (!c.weightObject.IsEmpty() &&
            (c.weightField < 0 || !b.weightFields[size_t(c.weightField)].ok ||
             b.weightFields[size_t(c.weightField)].values.size() != 1)) {
            Unprobed("Constraint", kind,
                     "rigExec:weightObject whose weight field did not resolve");
            continue;
        }
        if (c.spaceSlot >= 0) {
            Unprobed("Constraint", kind,
                     "rigExec:space carry: composed by the body from posedM "
                     "and the default round trip");
            continue;
        }
        if (commit.split || commit.slotWrites.size() != 1 || c.target < 0) {
            Unprobed("Constraint", kind, "split or multi-slot commit");
            continue;
        }
        bool native = false;
        for (const int source : c.sources)
            native = native || source < 0;
        if (c.worldUpObjectNamed && c.worldUpObject < 0)
            native = true;
        if (native) {
            Unprobed("Constraint", kind,
                     "a source or world-up object read off the stage "
                     "(nativeSources), not a fin version");
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
        for (size_t k = 0; k < c.sources.size(); ++k)
            input.sources[k].frame = b.fin[size_t(commit.sourceReads[k])];
        if (c.worldUpObjectNamed)
            input.worldUp = b.fin[size_t(commit.worldUpRead)];
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
        Compare("Constraint", kind, c.pathText, VtValue(baked),
                VtValue(published), !present);
        CompareFlag("Constraint", kind, c.pathText, "published", present,
                    applied);
    }
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
            if (revision.op == RigExecRevisionOp::External) {
                Unprobed("RevisionChunk", kind,
                         "the baked External revision carries the handler's "
                         "assembled payload, not the declared input list its "
                         "op reads");
                continue;
            }
            if (revision.op != RigExecRevisionOp::BlendShape &&
                !RigExecGeometryOpSupported(revision.op)) {
                Unprobed("RevisionChunk", kind, "no op for this revision");
                continue;
            }
            if (revision.op == RigExecRevisionOp::BlendShape &&
                !revision.blendChunked) {
                Unprobed("RevisionChunk", kind,
                         "a whole blend's packet keeps only the summed deltas, "
                         "not the channel samples the op reads");
                continue;
            }
            const auto entering = VersionPoints(chain, r);
            const auto published = VersionPoints(chain, r + 1);
            auto p = revision.parameters;
            if (revision.op == RigExecRevisionOp::Skin)
                p.skinTransforms = revision.influences;
            std::vector<RigExecBlendChannel> channels;
            const bool blend = revision.op == RigExecRevisionOp::BlendShape;
            if (blend) {
                channels = revision.blendSamples;
                for (size_t c = 0;
                     c < channels.size() && c < revision.blendChannels.size();
                     ++c) {
                    const auto &bound = revision.blendChannels[c];
                    const auto held = RigExecBakedResolvePathLeaf(
                        b, revision.leaves, size_t(bound.weightHeldLeaf));
                    if (bound.poseWeight >= 0 &&
                        !(held.IsHolding<bool>() && held.UncheckedGet<bool>()))
                        channels[c].weight =
                            b.poseWeights[size_t(bound.poseWeight)];
                    else {
                        const auto value = RigExecBakedResolvePathLeaf(
                            b, revision.leaves, size_t(bound.weightLeaf));
                        channels[c].weight = value.IsHolding<float>()
                                                 ? value.UncheckedGet<float>()
                                                 : 0.0f;
                    }
                }
            }
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
                                   entering, runner, points, &error);
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
// run node by node. Only chains every revision of which an op takes as is.
void
ProbeRangeChains(const B &b)
{
    for (const auto &chain : b.chains) {
        if (!chain.haveBase || chain.revisions.size() < 2)
            continue;
        bool usable = true;
        std::string kinds;
        for (const auto &revision : chain.revisions) {
            kinds +=
                (kinds.empty() ? "" : ">") + std::string(OpName(revision.op));
            usable = usable && !revision.gpuOwned && !revision.rangeSetAside &&
                     revision.op != RigExecRevisionOp::BlendShape &&
                     RigExecGeometryOpSupported(revision.op) &&
                     revision.parameters.valid && revision.status.AllowsApply();
        }
        if (!usable) {
            Unprobed("RangeChain", "chain", "a revision no op takes as is");
            continue;
        }
        const auto entering = VersionPoints(chain, 0);
        std::vector<RigExecMoverParameters> params;
        for (const auto &revision : chain.revisions) {
            params.push_back(revision.parameters);
            if (revision.op == RigExecRevisionOp::Skin)
                params.back().skinTransforms = revision.influences;
        }
        std::vector<GfVec3f> stepped = entering;
        bool stepsApplied = true;
        for (size_t r = 0; r < chain.revisions.size() && stepsApplied; ++r) {
            RigExecGeometryOpRunner runner;
            runner.compile.useSimd = b.useSimd;
            runner.compile.rangeGrain = 0;
            std::vector<GfVec3f> next = stepped;
            stepsApplied =
                RigExecRunGeometryOp(chain.revisions[r].op, params[r], nullptr,
                                     stepped, runner, next);
            stepped = std::move(next);
        }
        if (!stepsApplied) {
            Unprobed("RangeChain", "chain", "a revision's op refused");
            continue;
        }
        for (const RangeSetting &setting : kRangeSettings) {
            RigExecRigDescBuilder builder;
            std::vector<RigExecGeometryOpNode> nodes(chain.revisions.size());
            std::string error;
            bool ok = true;
            for (size_t r = 0; r < nodes.size() && ok; ++r)
                ok = RigExecAddGeometryOp(chain.revisions[r].op, params[r],
                                          &builder, r ? &nodes[r - 1] : nullptr,
                                          &nodes[r], &error);
            RigExecCompileOptions options = RigExecGeometryCompileOptions();
            options.useSimd = b.useSimd;
            if (setting.grain != kGeometryGrain) {
                options.rangeGrain = setting.grain;
                options.rangeThreshold = 0;
            }
            RigExecEvaluatorOptions evaluate;
            if (setting.threads)
                evaluate.tasks = g_tasks.get();
            const auto desc = ok ? builder.Build(&error) : nullptr;
            const auto program =
                desc ? RigExecCompile(*desc, options, &error) : nullptr;
            auto evaluator =
                program ? RigExecEvaluator::Create(program, evaluate, &error)
                        : nullptr;
            for (size_t r = 0; evaluator && r < nodes.size(); ++r)
                if (!RigExecWriteGeometryOpInputs(
                        params[r], nodes[r], r ? nullptr : &entering,
                        entering.size(), evaluator.get(), &error))
                    evaluator.reset();
            std::vector<GfVec3f> piped;
            if (evaluator && evaluator->Evaluate(&error)) {
                const auto *out = static_cast<
                    const rigExecStd::RigExecArray<rigExecStd::Vec3f> *>(
                    evaluator->GetOutput(nodes.back().out));
                for (const auto &p : *out)
                    piped.emplace_back(p[0], p[1], p[2]);
            }
            Compare("RangeChain", kinds,
                    chain.target.GetString() + setting.label, Points(stepped),
                    Points(piped), false,
                    piped.empty() ? "chain program failed: " + error
                                  : std::string());
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
                Compare("Derived", kind, derived.targetText,
                        derived.haveMatrix ? VtValue(derived.matrix)
                                           : VtValue(),
                        ran ? VtValue(matrix) : VtValue(), !derived.haveMatrix);
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

template <class T>
bool
RunProperty(const RigExecScenePropertyDescriptor &descriptor, const T &in,
            float envelope, T *out, std::string &error)
{
    RigExecRigDescBuilder builder;
    RigExecPropertyOpNode node;
    if (!RigExecAddPropertyOp(descriptor, &builder, nullptr, &node, &error))
        return false;
    auto desc = builder.Build(&error);
    auto program = desc ? RigExecCompile(*desc, {}, &error) : nullptr;
    auto evaluator =
        program ? RigExecEvaluator::Create(program, {}, &error) : nullptr;
    if (!evaluator || !evaluator->SetInput(node.in, &in, &error) ||
        !RigExecWritePropertyOpInputs(descriptor, node, envelope,
                                      evaluator.get(), &error) ||
        !evaluator->Evaluate(&error))
        return false;
    *out = *static_cast<const T *>(evaluator->GetOutput(node.out));
    return true;
}

template <class T>
T
WalkRead(const B &b, const RigExecBakedWalk &walk, T fallback)
{
    VtValue value;
    if (!RigExecBakedResolveWalkValue(b, walk, &value) || !value.IsHolding<T>())
        return fallback;
    return value.UncheckedGet<T>();
}

// PropertyRevision: one revision of a chain, from the version entering it to
// the version it wrote. The record is built from the operands the body read
// through the revision's walks, with the body's fallbacks.
void
ProbeProperty(const B &b)
{
    using Arm = RigExecBakedPropertyChain::Arm;
    for (const auto &chain : b.propertyChains) {
        const char *armName = chain.arm == Arm::Float    ? "float"
                              : chain.arm == Arm::Vec3f  ? "vec3f"
                              : chain.arm == Arm::Double ? "double"
                                                         : "matrix4d";
        for (size_t k = 0; k < chain.revisions.size(); ++k) {
            const auto &revision = chain.revisions[k];
            if (chain.arm == Arm::Double) {
                Unprobed("PropertyRevision", armName,
                         "no math op for a double target");
                continue;
            }
            const uint32_t in = chain.versionBase + uint32_t(k);
            const uint32_t out = in + 1;
            if (!b.propertyVersionValid[in] || !revision.moverExists) {
                Unprobed("PropertyRevision", armName,
                         "the entering version is invalid or the mover is "
                         "gone");
                continue;
            }
            bool admitted = WalkRead(b, revision.enabled, true);
            float envelope = WalkRead(b, revision.defaultWeight, 1.0f);
            if (!revision.weightObject.IsEmpty()) {
                const auto &field =
                    b.weightFields[size_t(revision.weightField)];
                admitted = admitted && field.ok && field.values.size() == 1;
                envelope = admitted ? field.values[0] : 0.0f;
            } else if (!std::isfinite(envelope) || envelope < 0.0f ||
                       envelope > 1.0f)
                admitted = false;
            RigExecScenePropertyDescriptor descriptor;
            descriptor.mover = revision.mover;
            descriptor.target = chain.target;
            descriptor.targetType = chain.valueType;
            const VtVec3fArray pathPoints =
                WalkRead(b, revision.pathPoints, VtVec3fArray());
            if (chain.arm == Arm::Float) {
                RigExecFloatPropertyRecord record;
                record.op = revision.op;
                record.opValid = revision.opValid;
                record.hasTangents = revision.hasTangents;
                record.value = WalkRead(b, revision.value, 0.0f);
                record.minimum = WalkRead(b, revision.minimum, 0.0f);
                record.maximum = WalkRead(b, revision.maximum, 0.0f);
                const auto keys =
                    WalkRead(b, revision.keys, VtArray<GfVec2f>());
                record.keys.assign(keys.cbegin(), keys.cend());
                if (revision.hasTangents) {
                    const auto tangents =
                        WalkRead(b, revision.tangents, VtArray<GfVec2f>());
                    record.tangents.assign(tangents.cbegin(), tangents.cend());
                }
                record.pathPointData = pathPoints.cdata();
                record.pathPointCount = pathPoints.size();
                record.path = revision.path;
                descriptor.record = record;
            } else if (chain.arm == Arm::Vec3f) {
                RigExecVec3PropertyRecord record;
                record.op = revision.op;
                record.opValid = revision.opValid;
                record.value = WalkRead(b, revision.value, GfVec3f(0.0f));
                record.minimum = WalkRead(b, revision.minimum, GfVec3f(0.0f));
                record.maximum = WalkRead(b, revision.maximum, GfVec3f(0.0f));
                descriptor.record = record;
            } else {
                RigExecMatrixPropertyRecord record;
                record.op = revision.op;
                record.opValid = revision.opValid;
                record.value = WalkRead(b, revision.value, GfMatrix4d(1.0));
                descriptor.record = record;
            }
            const auto &entering = b.propertyValues[in];
            const auto &written = b.propertyValues[out];
            const bool valid = b.propertyVersionValid[out];
            std::string error;
            bool applied = false;
            VtValue baked, op, passed;
            if (chain.arm == Arm::Float) {
                float value = entering.f;
                if (admitted)
                    applied = RunProperty(descriptor, entering.f, envelope,
                                          &value, error);
                baked = valid ? VtValue(written.f) : VtValue();
                op = VtValue(applied ? value : entering.f);
                passed = VtValue(entering.f);
            } else if (chain.arm == Arm::Vec3f) {
                const rigExecStd::Vec3f from(entering.v[0], entering.v[1],
                                             entering.v[2]);
                rigExecStd::Vec3f value = from;
                if (admitted)
                    applied =
                        RunProperty(descriptor, from, envelope, &value, error);
                baked = valid ? VtValue(written.v) : VtValue();
                op = VtValue(applied ? GfVec3f(value[0], value[1], value[2])
                                     : entering.v);
                passed = VtValue(entering.v);
            } else {
                rigExecStd::Mat4d from, value;
                for (int row = 0; row < 4; ++row)
                    for (int col = 0; col < 4; ++col)
                        from._mtx[row][col] = entering.m[row][col];
                value = from;
                if (admitted)
                    applied =
                        RunProperty(descriptor, from, envelope, &value, error);
                GfMatrix4d matrix = entering.m;
                for (int row = 0; applied && row < 4; ++row)
                    for (int col = 0; col < 4; ++col)
                        matrix[row][col] = value._mtx[row][col];
                baked = valid ? VtValue(written.m) : VtValue();
                op = VtValue(matrix);
                passed = VtValue(entering.m);
            }
            if (admitted && !applied) {
                OpFailed("PropertyRevision", armName,
                         revision.mover.GetString(), !valid || baked == passed,
                         error);
                continue;
            }
            Compare("PropertyRevision", armName, revision.mover.GetString(),
                    baked, op, !applied);
        }
    }
}

// WeightPacket: a combine weight's packet from the packets, target count,
// strength and invert the body read. Other weight kinds have no op.
void
ProbeWeights(const B &b)
{
    for (const auto &step : b.steps) {
        if (step.kind != RigExecBakedStepKind::WeightPacket)
            continue;
        const auto &w = b.weightObjects[size_t(step.object)];
        const std::string kind = TypeName(w.type);
        if (kind != SchemaTypeName::RigExecCombineWeight) {
            Unprobed("WeightPacket", kind,
                     "no op (CombineWeights is the only weight-packet op)");
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
// dense and a sparse partial envelope.
const char *const kPaintedKinds[] = {
    "Matrix", "Skin",    "Smooth",         "DeltaMush",     "Lattice",
    "Wire",   "Wrinkle", "SurfaceProject", "VolumeCorrect",
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

void
ProbeStage(const UsdStageRefPtr &stage, const std::string &rig, Paint paint)
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
    for (const UsdTimeCode time : Frames(stage)) {
        if (!evaluator.Evaluate(time).valid) {
            ++g_unprobed["rig " + name + ": invalid pose"];
            continue;
        }
        const auto *program = evaluator.GetBakedProgram();
        if (!program)
            continue;
        const B &b = RigExecBakedProgramAccess::StepGraph(*program);
        g_where = name + "@" + TfStringify(time.GetValue());
        ProbeSolve(b);
        ProbeConstraint(b);
        ProbeRevisions(b);
        ProbeRangeChains(b);
        ProbeDerived(b);
        ProbeProperty(b);
        ProbeWeights(b);
    }
}

void
ProbeRig(const std::string &file, Paint paint)
{
    ProbeStage(UsdStage::Open(file), TfGetBaseName(file), paint);
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

} // namespace

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
    // Rigs after --painted are probed as they are and again under each
    // painted partial envelope.
    bool painted = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--painted") == 0) {
            painted = true;
            continue;
        }
        ProbeRig(argv[i], Paint::None);
        if (painted)
            for (const Paint paint : kPaints)
                ProbeRig(argv[i], paint);
    }
    if (painted)
        for (const Paint paint : {Paint::None, Paint::Dense, Paint::Sparse})
            ProbeStage(WireRig(), "wire rig", paint);
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
         {"Solve/", "RevisionChunk/", "RangeChain/", "Derived/",
          "PropertyRevision/", "WeightPacket/", "Constraint/"}) {
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
